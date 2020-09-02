/*
 * TI-RSLK MAX Motor Demonstration
 *
 * Copyright (c) Dr. Xinrong Li @ UNT
 * All rights reserved.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>  //Exact-width integer types
#include <ti\devices\msp432p4xx\driverlib\driverlib.h>  // Driver library


#define CLOCK_HF    48000000 //48MHz
#define CLOCK_LF    32000 //32kHz

#define HEARTBEAT_FREQ    2  //unit: Hz, heart beat LED blinking frequency for debugging

#define RED_LED    GPIO_PIN0  //heart beat LED
#define GREEN_LED    GPIO_PIN1
#define BLUE_LED    GPIO_PIN2

#define BUMP0    GPIO_PIN0  //right side of robot
#define BUMP1    GPIO_PIN2
#define BUMP2    GPIO_PIN3
#define BUMP3    GPIO_PIN5
#define BUMP4    GPIO_PIN6
#define BUMP5    GPIO_PIN7  //left side of robot

#define NUM_DISP_TEXT_LINE    5
#define MAX_STR_BUFFER_LEN    200

#define PWM_FREQ    1000 //unit: Hz
#define PWM_DUTYCYCLE_MIN    0 //unit: percent
#define PWM_DUTYCYCLE_MAX    100 //unit: percent
#define PWM_DUTYCYCLE_STEP    5 //unit: percent

#define LEFT_MOTOR    0
#define RIGHT_MOTOR    1

#define MOTOR_FORWARD    1
#define MOTOR_STOP    0
#define MOTOR_BACKWARD    -1

#define PULSES_PER_ROTATION     360

//Function prototypes
void initDevice_HFXT(void);
void initHeartBeatLED(void);
void initPulseTimer(void);
void initDebugUART(void);
void initMotors(void);
void initADC14(void);
void initBumpSensors(void);
void initTachometers(void);

void leftMotor_activate(int flag);
void rightMotor_activate(int flag);
void leftMotor_run(int direction, int dutyCycle);
void rightMotor_run(int direction, int dutyCycle);

float tacho_calcSpeed(int timerCount);

void uart0_transmitStr(const char *str);


//Global variables
uint32_t clockMCLK, clockSMCLK;
uint8_t currentLED = RED_LED;


int currentMotor = LEFT_MOTOR;

int leftMotor_dutyCycle = PWM_DUTYCYCLE_MIN;
int leftMotor_direction = MOTOR_FORWARD;

int rightMotor_dutyCycle = PWM_DUTYCYCLE_MIN;
int rightMotor_direction = MOTOR_FORWARD;

volatile int leftTacho_timerCount = 0;
volatile int leftTacho_direction = MOTOR_FORWARD;

volatile int rightTacho_timerCount = 0;
volatile int rightTacho_direction = MOTOR_FORWARD;

float lightLvlFront = 0.0;
float lightLvlBack = 0.0;
float lightLvlLeft = 0.0;
float lightLvlRight = 0.0;

float rotationTime = 0.0;

int lightThreshold = 3900;

bool nextPulse = false;

char strBuffer[MAX_STR_BUFFER_LEN];

const char *terminalDisplayText[NUM_DISP_TEXT_LINE] =
{
    "\r\n",
    "TI-RSLK MAX Motor Control Demo\r\n",
    "  C: Change LED Color, L: Left Motor, R: Right Motor, S: Change Direction\r\n",
    "  I: Increase DutyCycle, D: Decrease DutyCycle, V: Display Speed, H: Help\r\n",
    "> "
};

volatile uint_fast16_t bumpStatus = 0;


void main(void)
{
    int i=0;

    initDevice_HFXT();
    //initHeartBeatLED();
    initPulseTimer();
    initDebugUART();
    initMotors();
    initADC14();
    initBumpSensors();
    initTachometers();

    Interrupt_enableMaster();
    Timer32_startTimer(TIMER32_0_BASE, false);
    Timer32_startTimer(TIMER32_1_BASE, false);

    //Start timer for tachometer speed measurement.
    Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_CONTINUOUS_MODE);

    //Initial display on terminal.
    for(i=0; i<NUM_DISP_TEXT_LINE; i++)
    {
        uart0_transmitStr(terminalDisplayText[i]);
    }

    while(1)
    {
        // if the light exceeds the threshold on any of the photoresistors
        if((lightLvlFront > lightThreshold) || (lightLvlBack > lightThreshold) || (lightLvlLeft > lightThreshold) || (lightLvlRight > lightThreshold))
        {
            if(bumpStatus)
            {
                uart0_transmitStr("Bump sensors triggered.\r\n> ");
                if((bumpStatus & BUMP0) || (bumpStatus & BUMP1))  // Right 2 bumpers triggered.
                {
                    // Back Up
                    leftMotor_run(MOTOR_BACKWARD, 30);
                    rightMotor_run(MOTOR_BACKWARD, 30);
                    delay(500);
                    leftMotor_run(MOTOR_BACKWARD, 0);
                    rightMotor_run(MOTOR_BACKWARD, 0);

                    // Turn Left
                    leftMotor_run(MOTOR_BACKWARD, 30);
                    rightMotor_run(MOTOR_FORWARD, 30);
                    delay(500);
                    leftMotor_run(MOTOR_BACKWARD, 0);
                    rightMotor_run(MOTOR_BACKWARD, 0);
                }
                if((bumpStatus & BUMP2) || (bumpStatus & BUMP3))  // Middle 2 bumpers triggered
                {
                    // Back Up
                    leftMotor_run(MOTOR_BACKWARD, 30);
                    rightMotor_run(MOTOR_BACKWARD, 30);
                    delay(500);
                    leftMotor_run(MOTOR_BACKWARD, 0);
                    rightMotor_run(MOTOR_BACKWARD, 0);

                    // Turn Left
                    leftMotor_run(MOTOR_BACKWARD, 30);
                    rightMotor_run(MOTOR_FORWARD, 30);
                    delay(500);
                    leftMotor_run(MOTOR_BACKWARD, 0);
                    rightMotor_run(MOTOR_BACKWARD, 0);
                }
                if((bumpStatus & BUMP4) || (bumpStatus & BUMP5))  // Left 2 bumpers triggered
                {
                    // Back Up
                    leftMotor_run(MOTOR_BACKWARD, 30);
                    rightMotor_run(MOTOR_BACKWARD, 30);
                    delay(500);
                    leftMotor_run(MOTOR_BACKWARD, 0);
                    rightMotor_run(MOTOR_BACKWARD, 0);

                    // Turn Right
                    leftMotor_run(MOTOR_FORWARD, 30);
                    rightMotor_run(MOTOR_BACKWARD, 30);
                    delay(500);
                    leftMotor_run(MOTOR_BACKWARD, 0);
                    rightMotor_run(MOTOR_BACKWARD, 0);
                }

                //Clear the status after proper handling.
                bumpStatus = 0;
            }
            // if the light is behind the robot
            else if(lightLvlBack > lightThreshold){
                leftMotor_run(MOTOR_FORWARD, 30);
                rightMotor_run(MOTOR_FORWARD, 30);
            }
            // if the front light level is highest
            else if((lightLvlFront > lightLvlBack) && (lightLvlFront > lightLvlLeft) || (lightLvlFront > lightLvlRight))
            {
                leftMotor_run(MOTOR_BACKWARD, 30);
                rightMotor_run(MOTOR_FORWARD, 30);
            }
            // if the left light level is highest
            else if((lightLvlLeft > lightLvlBack) && (lightLvlLeft > lightLvlFront) || (lightLvlLeft > lightLvlRight))
            {
                leftMotor_run(MOTOR_FORWARD, 30);
                rightMotor_run(MOTOR_FORWARD, 0);
            }
            // if the right light level is highest
            else if((lightLvlRight > lightLvlBack) && (lightLvlRight > lightLvlLeft) || (lightLvlRight > lightLvlFront))
            {
                leftMotor_run(MOTOR_FORWARD, 0);
                rightMotor_run(MOTOR_FORWARD, 30);
            }
        }else{
            leftMotor_run(MOTOR_FORWARD, 0);
            rightMotor_run(MOTOR_FORWARD, 0);
        }
    } //end of while
}


void initDevice_HFXT(void)
{
    WDT_A_holdTimer();  //Stop watchdog timer.

    //Change VCORE to 1 to support a frequency higher than 24MHz.
    //See MSP432 Data Sheet, Section 5.8 for Flash wait-state requirements for active frequency.
    PCM_setPowerState(PCM_AM_DCDC_VCORE1);
    FlashCtl_setWaitState(FLASH_BANK0, 1);
    FlashCtl_setWaitState(FLASH_BANK1, 1);

    FPU_enableModule();
    FPU_enableLazyStacking(); //Required to use FPU within ISR.

    //Configure PJ.2 and PJ.3 in HFXT mode.
    //Initialize external clock sources HFXT.
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2|GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    CS_setExternalClockSourceFrequency(CLOCK_LF, CLOCK_HF);
    CS_startHFXT(false);

    CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_HSMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_8);
    CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_16);

    clockMCLK = CS_getMCLK();
    clockSMCLK = CS_getSMCLK();
}


void initHeartBeatLED(void)
{
    //Configure P2.0, P2.1, P2.2 as output.
    //P2.0, P2.1, P2.2 are connected to a RGB tri-color LED on LaunchPad.
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);

    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_0_BASE, clockMCLK/HEARTBEAT_FREQ);
    Timer32_enableInterrupt(TIMER32_0_BASE);
    Interrupt_enableInterrupt(INT_T32_INT1); // Enable Timer32_0 interrupt in the interrupt controller.
}

void initPulseTimer(void)
{
    Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_setCount(TIMER32_1_BASE, clockMCLK/1000);
    Timer32_enableInterrupt(TIMER32_1_BASE);
    Interrupt_enableInterrupt(INT_T32_INT2); // Enable Timer32_0 interrupt in the interrupt controller.
}


void initDebugUART(void)
{
    // Configuration for 3MHz SMCLK, 9600 baud rate.
    // Calculated using the online calculator that TI provides at:
    // http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
    const eUSCI_UART_Config config =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK, //SMCLK Clock Source
        19, //BRDIV = 19
        8, //UCxBRF = 8
        0, //UCxBRS = 0
        EUSCI_A_UART_NO_PARITY, //No Parity
        EUSCI_A_UART_LSB_FIRST, //MSB First
        EUSCI_A_UART_ONE_STOP_BIT, //One stop bit
        EUSCI_A_UART_MODE, //UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION //Oversampling
    };

    //Configure GPIO pins for UART. RX: P1.2, TX:P1.3.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2|GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    UART_initModule(EUSCI_A0_BASE, &config);
    UART_enableModule(EUSCI_A0_BASE);
}

//Use P6.1/A14 to sample
void initADC14(void)
{
    ADC14_enableModule();
    ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, ADC_NOROUTE);

    //Configure P6.1 as A14
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    ADC14_setResolution(ADC_12BIT);
    ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, false);
    ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, false);
    ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A9, false);


    //See TechRef Section 20.2.6 for sample timing consideration.
    ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_4, ADC_PULSE_WIDTH_4);

    ADC14_enableConversion();
}

void initBumpSensors(void)
{
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, BUMP0|BUMP1|BUMP2|BUMP3|BUMP4|BUMP5);

    //Any of the pins on Ports 1 – 6 can request an interrupt.
    GPIO_enableInterrupt(GPIO_PORT_P4, BUMP0|BUMP1|BUMP2|BUMP3|BUMP4|BUMP5);
    GPIO_interruptEdgeSelect(GPIO_PORT_P4, BUMP0|BUMP1|BUMP2|BUMP3|BUMP4|BUMP5, GPIO_HIGH_TO_LOW_TRANSITION);

    Interrupt_enableInterrupt(INT_PORT4);
}


void initMotors(void)
{
    Timer_A_PWMConfig leftMotor_pwmConfig =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_3,
            clockSMCLK/(3*PWM_FREQ),
            TIMER_A_CAPTURECOMPARE_REGISTER_4,
            TIMER_A_OUTPUTMODE_RESET_SET,
            clockSMCLK/(3*PWM_FREQ)/100*leftMotor_dutyCycle
    };

    Timer_A_PWMConfig rightMotor_pwmConfig =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,
            TIMER_A_CLOCKSOURCE_DIVIDER_3,
            clockSMCLK/(3*PWM_FREQ),
            TIMER_A_CAPTURECOMPARE_REGISTER_3,
            TIMER_A_OUTPUTMODE_RESET_SET,
            clockSMCLK/(3*PWM_FREQ)/100*rightMotor_dutyCycle
    };

    //P3.7: 1 to activate left motor;
    //P3.6: 1 to activate right motor
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6|GPIO_PIN7);

    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);

    //P5.4: left motor, 0 for forward, 1 for backward;
    //P5.5: right motor, 0 for forward, 1 for backward
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4|GPIO_PIN5);

    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);

    //P2.7/TA0.4: left motor PWM;
    //P2.6/TA0.3: right motor PWM
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);

    Timer_A_generatePWM(TIMER_A0_BASE, &leftMotor_pwmConfig);
    Timer_A_generatePWM(TIMER_A0_BASE, &rightMotor_pwmConfig);
}

void initTachometers(void)
{
    Timer_A_ContinuousModeConfig Timer_A_config =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,  //3MHz
            TIMER_A_CLOCKSOURCE_DIVIDER_5,  //600kHz
            TIMER_A_TAIE_INTERRUPT_DISABLE,
            TIMER_A_DO_CLEAR
    };

    Timer_A_CaptureModeConfig leftTacho_captureConfig =
    {
            TIMER_A_CAPTURECOMPARE_REGISTER_1,
            TIMER_A_CAPTUREMODE_RISING_EDGE,
            TIMER_A_CAPTURE_INPUTSELECT_CCIxA,
            TIMER_A_CAPTURE_SYNCHRONOUS,
            TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
            TIMER_A_OUTPUTMODE_OUTBITVALUE
    };


    Timer_A_CaptureModeConfig rightTacho_captureConfig =
    {
            TIMER_A_CAPTURECOMPARE_REGISTER_0,
            TIMER_A_CAPTUREMODE_RISING_EDGE,
            TIMER_A_CAPTURE_INPUTSELECT_CCIxA,
            TIMER_A_CAPTURE_SYNCHRONOUS,
            TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
            TIMER_A_OUTPUTMODE_OUTBITVALUE
    };

    Timer_A_configureContinuousMode(TIMER_A3_BASE, &Timer_A_config);

    //P10.5/TA3.1: left tachometer
    //P10.4/TA3.0: right tachometer
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P10, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);

    //TA3.1: left tachometer pulse period (i.e., speed) measurement
    //TA3.0: right tachometer pulse period (i.e., speed) measurement
    Timer_A_initCapture(TIMER_A3_BASE, &leftTacho_captureConfig);
    Timer_A_initCapture(TIMER_A3_BASE, &rightTacho_captureConfig);

    //P5.2: left tachometer direction
    //P5.0: right tachometer direction
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN2);

    Interrupt_enableInterrupt(INT_TA3_0);
    Interrupt_enableInterrupt(INT_TA3_N);
}

void delay(int delay_ms)
{
    int timePassed = 0;
    nextPulse = false;

    while(timePassed < delay_ms){
        if(nextPulse == true){
            timePassed+=1;
            nextPulse = false;
        }
    }
}

//flag = 1 to activate, 0 to sleep
void leftMotor_activate(int flag)
{
    if(flag)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
    }
    else
    {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
    }
}


//flag = 1 to activate, 0 to sleep
void rightMotor_activate(int flag)
{
    if(flag)
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
    }
    else
    {
        GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    }
}


void leftMotor_run(int direction, int dutyCycle)
{
    if(leftMotor_direction != direction)
    {
        leftMotor_direction = direction;
        if(direction == MOTOR_FORWARD)
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4);
        else
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4);
    }

    if(dutyCycle > PWM_DUTYCYCLE_MAX)
    {
        dutyCycle = PWM_DUTYCYCLE_MAX;
    }

    if(dutyCycle < PWM_DUTYCYCLE_MIN)
    {
        dutyCycle = PWM_DUTYCYCLE_MIN;
    }

    if(leftMotor_dutyCycle != dutyCycle)
    {
        leftMotor_dutyCycle = dutyCycle;
        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, clockSMCLK/(3*PWM_FREQ)/100*leftMotor_dutyCycle);
    }
}


void rightMotor_run(int direction, int dutyCycle)
{
    if(rightMotor_direction != direction)
    {
        rightMotor_direction = direction;
        if(direction  == MOTOR_FORWARD)
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);
        else
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);
    }

    if(dutyCycle > PWM_DUTYCYCLE_MAX)
    {
        dutyCycle = PWM_DUTYCYCLE_MAX;
    }

    if(dutyCycle < PWM_DUTYCYCLE_MIN)
    {
        dutyCycle = PWM_DUTYCYCLE_MIN;
    }

    if(rightMotor_dutyCycle != dutyCycle)
    {
        rightMotor_dutyCycle = dutyCycle;
        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, clockSMCLK/(3*PWM_FREQ)/100*rightMotor_dutyCycle);
    }
}

float tacho_calcSpeed(int timerCount)
{
    //pulsePeriod = timerCount/timerClockFreq
    //speed (RPM) = 60/(360*pulsePeriod)
    //Hard-coded for Timer_A frequency 3MHz/6 = 600kHz.
    return 100000.0/timerCount;
}

//Transmit a string through UART0.
void uart0_transmitStr(const char *str)
{
    int len, i=0;

    len = strlen(str);
    while(i < len)
    {
        UART_transmitData(EUSCI_A0_BASE, str[i++]);
        while(!UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG));
        UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG);
    }
}

//Port 4 ISR
void PORT4_IRQHandler(void)
{
    bumpStatus = GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    GPIO_clearInterruptFlag(GPIO_PORT_P4, bumpStatus);

    //Some bump sensors triggered.
    //Program critical actions here; for example, stop all motors.
    //Other non-critical actions, for example, deciding where to go, should be done outside the IRQHandler.

}

//Timer32_0 ISR
void T32_INT1_IRQHandler(void)
{
    Timer32_clearInterruptFlag(TIMER32_0_BASE);

    if(GPIO_getInputPinValue(GPIO_PORT_P2, RED_LED|GREEN_LED|BLUE_LED))
    {
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, RED_LED|GREEN_LED|BLUE_LED);
    }
    else
    {
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, currentLED);
    }
}

//Timer32_1 ISR
void T32_INT2_IRQHandler(void)
{
    Timer32_clearInterruptFlag(TIMER32_1_BASE);

    nextPulse = true;
    //uart0_transmitStr("TEST.\r\n> ");

    //Read from ADC.
    ADC14_toggleConversionTrigger();

    while(!(ADC_INT0 & ADC14_getInterruptStatus()));
    ADC14_clearInterruptFlag(ADC_INT0);

    lightLvlFront = ADC14_getResult(ADC_MEM0);
    lightLvlBack = ADC14_getResult(ADC_MEM1);
    lightLvlLeft = ADC14_getResult(ADC_MEM2);
    lightLvlRight = ADC14_getResult(ADC_MEM3);
}

//Left tachometer pulse period measurement
void TA3_N_IRQHandler(void)
{
    static uint_fast16_t lastCount = 0, currentCount = 0;

    Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    currentCount = Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);

    leftTacho_timerCount = currentCount - lastCount;
    if(leftTacho_timerCount < 0)
    {
        leftTacho_timerCount += 0xFFFF;
    }

    lastCount = currentCount;

    //P5.2: 1 for forward, 0 for backward
    if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN2))
    {
        leftTacho_direction = MOTOR_FORWARD;
    }
    else
    {
        leftTacho_direction = MOTOR_BACKWARD;
    }
}


//Right tachometer pulse period measurement
void TA3_0_IRQHandler(void)
{
    static uint_fast16_t lastCount = 0, currentCount = 0;

    Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    currentCount = Timer_A_getCaptureCompareCount(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    rightTacho_timerCount = currentCount - lastCount;
    if(rightTacho_timerCount < 0)
    {
        rightTacho_timerCount += 0xFFFF;
    }

    lastCount = currentCount;

    //P5.0: 1 for forward, 0 for backward
    if(GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0))
    {
        rightTacho_direction = MOTOR_FORWARD;
    }
    else
    {
        rightTacho_direction = MOTOR_BACKWARD;
    }
}
