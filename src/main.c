// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/PWM.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define COMMAND_MAX_LEN 4
#define NUM_COMMANDS   5
#define MAX_DATA_NUM 20

// Global constant definitions
const uint32_t DECAY_COUNT_THRESHOLD = 20000;
const uint16_t THIN_TAPE_THRESHOLD_MS = 100;
const uint8_t  TAPE_MIN_MS = 15;
const uint32_t CENTER_TARGET_ADC = 1900;
const uint32_t START_UTURN_ADC = 2200;
const uint32_t STOP_UTURN_ADC = 1700;
const uint32_t START_RIGHT_TURN_ADC = 1200;
const uint32_t STOP_RIGHT_TURN_ADC = 1700;
const float CONTROL_ITERATION_TIME = 0.05;

volatile float Kp = 1.4;
volatile float Ki = 0.0;
volatile float Kd = 0.03;

// Type definitions
typedef void (*FunctionPointer)(void);

typedef enum {
    KEEPSTRAIGHT,
    UTURN,
    RIGHTTURN
} ControlState;

typedef struct {
    char *command;
    FunctionPointer funcPtr;
} CommandCallback;

typedef struct {
    uint32_t rightSensorADC;
    uint32_t frontSensorADC;
} SensorData_t;

// Global variable declarations (e.g., semaphores and shared variables)
SemaphoreHandle_t xUART1Semaphore;
SemaphoreHandle_t xTask1Semaphore;
SemaphoreHandle_t xTask2Semaphore;
SemaphoreHandle_t xTask4Semaphore;
SemaphoreHandle_t xTask5Semaphore;
SemaphoreHandle_t xTask6Semaphore;
QueueHandle_t xControlDataQueue;

volatile char commandBuffer[COMMAND_MAX_LEN];
volatile uint8_t commandIdx = 0;

volatile uint32_t totalPWMPeriodCount = 0;
volatile uint64_t decayStartCount = 0;
volatile uint64_t decayElapsedCount = 0;
volatile uint64_t onTapeStartCount = 0;
volatile uint64_t onTapeElapsedCount = 0;
volatile uint64_t onTapeElapsedms = 0;
volatile bool wasOnTape = false;
volatile uint8_t thinLinesCrossed = 0;

volatile uint32_t adcBuffer[2];

volatile ControlState controlState = KEEPSTRAIGHT;

volatile float errorPrior = 0;
volatile float integralPrior = 0;

bool dataCollectionEnabled = false;
int pingBuffer[MAX_DATA_NUM];
int pongBuffer[MAX_DATA_NUM];
int* collectBuffer = pingBuffer;
int* sendBuffer;

uint8_t currentDataIdx = 0;
uint8_t endDataIdx = 0;

// Function Headers
void RobotStart(void);
void RobotStop(void);
void Forward(void);
void Brake(void);
void Reverse(void);
void StartUTurn();
void StopUTurn();
void StartRightTurn();
void StopRightTurn();
void executePID(uint32_t rightADCValue);
void SteerLeft(float adjustPercentage);
void SteerRight(float adjustPercentage);
void SignalDataTransmit();

// Global arrays or command lookup tables
const CommandCallback commandCallbacks[NUM_COMMANDS] = {
    { "STR", RobotStart },
    { "STP", RobotStop },
    { "FWD", Forward },
    { "BRK", Brake },
    { "REV", Reverse },
};

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

// This hook is called by FreeRTOS when an stack overflow error is detected.
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    while(1)
    {
    }
}

void UART1IntHandler(void)
{
    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, true));

    if(UARTCharsAvail(UART1_BASE))
    {
        char receivedChar = UARTCharGet(UART1_BASE);
        UARTCharPut(UART1_BASE, receivedChar);
        commandBuffer[commandIdx] = toupper(receivedChar);
        commandIdx++;
        if(commandIdx == COMMAND_MAX_LEN - 1)
        {
            UARTCharPut(UART1_BASE, '\n');
            UARTCharPut(UART1_BASE, '\r');
            int commandNum;
            for(commandNum = 0; commandNum < NUM_COMMANDS; commandNum++)
            {
                if(strcmp((char*)commandBuffer, commandCallbacks[commandNum].command) == 0)
                {
                    commandCallbacks[commandNum].funcPtr();
                }
            }
            commandIdx = 0;
        }
    }
}

void WTimer1IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear the timer interrupt flag
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Give the semaphore and check if a higher-priority task was woken
    xSemaphoreGiveFromISR(xTask1Semaphore, &xHigherPriorityTaskWoken);

    // Yield if a higher-priority task was woken
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void WTimer2IntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear the timer interrupt flag
    TimerIntClear(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Give the semaphore and check if a higher-priority task was woken
    xSemaphoreGiveFromISR(xTask2Semaphore, &xHigherPriorityTaskWoken);

    // Yield if a higher-priority task was woken
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void GPIODIntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    GPIOIntClear(GPIO_PORTD_BASE, GPIO_INT_PIN_2);
    decayElapsedCount = TimerValueGet64(WTIMER0_BASE) - decayStartCount;

    // On dark surface
    if(decayElapsedCount >= 20000)
    {
        if(!wasOnTape)
        {
            wasOnTape = true;
            onTapeStartCount = TimerValueGet64(WTIMER0_BASE);
        }
    }
    // On bright surface
    else
    {
        if(wasOnTape)
        {
            wasOnTape = false;
            onTapeElapsedCount = TimerValueGet64(WTIMER0_BASE) - onTapeStartCount;
            onTapeElapsedms = (uint64_t)(onTapeElapsedCount * 1.0/SysCtlClockGet() * 1000);

            if(onTapeElapsedms > TAPE_MIN_MS)
            {
                UARTprintf("%i\n", (int)onTapeElapsedms);
                // Thin line crossed
                if(onTapeElapsedms < THIN_TAPE_THRESHOLD_MS)
                {
                    thinLinesCrossed++;

                    if(thinLinesCrossed == 1)
                    {
                        GPIOPinWrite(GPIO_PORTF_BASE,
                                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                                 GPIO_PIN_3);
                        dataCollectionEnabled = true;
                    }
                    else if(thinLinesCrossed == 2)
                    {
                        GPIOPinWrite(GPIO_PORTF_BASE,
                                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                                 GPIO_PIN_2);
                        dataCollectionEnabled = false;

                        // Give the semaphore and check if a higher-priority task was woken
                        xSemaphoreGiveFromISR(xTask6Semaphore, &xHigherPriorityTaskWoken);

                        // Yield if a higher-priority task was woken
                        if (xHigherPriorityTaskWoken == pdTRUE)
                        {
                            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                        }
                    }
                }
                // Thick line crossed
                else
                {
                    GPIOPinWrite(GPIO_PORTF_BASE,
                                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                                 GPIO_PIN_1);
                    // Give the semaphore and check if a higher-priority task was woken
                    xSemaphoreGiveFromISR(xTask5Semaphore, &xHigherPriorityTaskWoken);

                    // Yield if a higher-priority task was woken
                    if (xHigherPriorityTaskWoken == pdTRUE)
                    {
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                }
            }
        }
    }
}

// Task to charge reflectance sensor
void vTask1(void* pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xTask1Semaphore, portMAX_DELAY) == pdTRUE)
        {
            GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
            SysCtlDelay(SysCtlClockGet()/3 * 10.0/100000);
            GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2);
            decayStartCount = TimerValueGet64(WTIMER0_BASE);
        }
    }
}

// Task to update distance values
void vTask2(void* pvParameters)
{
    SensorData_t sensorData;
    while (1)
    {
        if (xSemaphoreTake(xTask2Semaphore, portMAX_DELAY) == pdTRUE)
        {
            // Read ADC Values
            ADCIntClear(ADC0_BASE, 1);
            ADCProcessorTrigger(ADC0_BASE, 1);
            while(!ADCIntStatus(ADC0_BASE, 1, false))
            {
            }
            ADCSequenceDataGet(ADC0_BASE, 1, (uint32_t*)adcBuffer);
            sensorData.rightSensorADC = adcBuffer[0];
            sensorData.frontSensorADC = adcBuffer[1];

            // Send sensor data to queue
            xQueueSend(xControlDataQueue, &sensorData, portMAX_DELAY);
        }
    }
}


// Task to update robot state
void vTask3(void* pvParameters)
{
    SensorData_t sensorData;
    while (1)
    {
        if (xQueueReceive(xControlDataQueue, &sensorData, portMAX_DELAY))
        {
            uint32_t rightADCValue = sensorData.rightSensorADC;
            uint32_t frontADCValue = sensorData.frontSensorADC;

            switch(controlState)
            {
            case KEEPSTRAIGHT:
                // Initiate UTurn condition
                if(frontADCValue > START_UTURN_ADC)
                {
                    StartUTurn();
                    controlState = UTURN;
                }
                // Initiate Right Turn condition
                else if(rightADCValue < START_RIGHT_TURN_ADC)
                {
                    StartRightTurn();
                    controlState = RIGHTTURN;
                }
                // Execute PID Control
                else
                {
                    executePID(rightADCValue);
                }
                break;
            case UTURN:
                // Exit UTurn condition
                if(frontADCValue < STOP_UTURN_ADC)
                {
                    StopUTurn();
                    controlState = KEEPSTRAIGHT;
                }
                break;
            case RIGHTTURN:
                // Exit Right Turn Condition
                if(rightADCValue >= STOP_RIGHT_TURN_ADC || frontADCValue >= START_UTURN_ADC)
                {
                    StopRightTurn();
                    controlState = KEEPSTRAIGHT;
                }
                break;
            }
        }
    }
}

// Task to send data
void vTask4(void* pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xTask4Semaphore, portMAX_DELAY) == pdTRUE)
        {
            // Print data header
            if (xSemaphoreTake(xUART1Semaphore, portMAX_DELAY) == pdTRUE)
            {
                UARTprintf("AB12");
                xSemaphoreGive(xUART1Semaphore);
            }

            // Print each data value
            int i;
            for(i = 0; i < endDataIdx; i++)
            {
                if (xSemaphoreTake(xUART1Semaphore, portMAX_DELAY) == pdTRUE)
                {
                    UARTprintf(" %i", sendBuffer[i]);
                    xSemaphoreGive(xUART1Semaphore);
                }
            }

            // Print data footer
            if (xSemaphoreTake(xUART1Semaphore, portMAX_DELAY) == pdTRUE)
            {
                UARTprintf("\r\n");
                xSemaphoreGive(xUART1Semaphore);
            }
        }
    }
}

// Task to prepare to transmit all current data after 2nd line is crossed
void vTask6(void* pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xTask6Semaphore, portMAX_DELAY) == pdTRUE)
        {
            SignalDataTransmit();
        }
    }
}

// Task to stop stop the robot
void vTask5(void* pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xTask5Semaphore, portMAX_DELAY) == pdTRUE)
        {
            TimerDisable(WTIMER1_BASE, TIMER_A);
            TimerDisable(WTIMER2_BASE, TIMER_A);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);

            int i;
            for(i = 0; i < 120; i++)
            {
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
                SysCtlDelay(SysCtlClockGet()/3 * 0.25);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
                SysCtlDelay(SysCtlClockGet()/3 * 0.25);
            }
        }
    }
}

void EnableSystemPeripherals()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
}

// Initialize 3 LED pins
void InitializeGPIOF()
{
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

// Initialize UART for printing to local terminal
void InitializeUART0()
{
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    UARTConfigSetExpClk(
        UART0_BASE,
        SysCtlClockGet(),
        9600,
        UART_CONFIG_WLEN_8 |
        UART_CONFIG_STOP_ONE |
        UART_CONFIG_PAR_NONE
    );
    UARTEnable(UART0_BASE);
}

// Initialize UART for the Bluetooth module
void InitializeUART1()
{
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    UARTConfigSetExpClk(
        UART1_BASE,
        SysCtlClockGet(),
        9600,
        UART_CONFIG_WLEN_8 |
        UART_CONFIG_STOP_ONE |
        UART_CONFIG_PAR_NONE
    );
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    IntPrioritySet(INT_UART1, 0xA0);
    IntRegister(INT_UART1_TM4C123, UART1IntHandler);
    UARTEnable(UART1_BASE);
    UARTStdioConfig(1, 9600, SysCtlClockGet());
}

// Initialize timer that counts up indefinitely to measure elapsed count
void InitializeWTimer0()
{
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);
    TimerLoadSet64(WTIMER0_BASE, 18446744073709551615);
    TimerEnable(WTIMER0_BASE, TIMER_A);
}

void InitializeWTimer1()
{
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    IntEnable(INT_WTIMER1A);
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_WTIMER1A, 0xA0);
    IntRegister(INT_WTIMER1A, WTimer1IntHandler);
    TimerLoadSet(WTIMER1_BASE, TIMER_A, SysCtlClockGet() * 0.007);
}

void InitializeWTimer2()
{
    TimerConfigure(WTIMER2_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    IntEnable(INT_WTIMER2A);
    TimerIntEnable(WTIMER2_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_WTIMER2A, 0xA0);
    IntRegister(INT_WTIMER2A, WTimer2IntHandler);
    TimerLoadSet(WTIMER2_BASE, TIMER_A, SysCtlClockGet() * CONTROL_ITERATION_TIME);
}

void InitializePWM0()
{
    // Setup PWM pins PB6-Left Motor PB7-Right Motor
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    // Setup phase pins PA2-Left Phase PA3-Right Phase
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

    // Sys Clock = 40Mhz PWMClock = 40/8 = 5MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

    uint32_t PWMClockCount = SysCtlClockGet() / 8;

    // Desired frequency = 10Khz
    uint32_t desiredFrequency = 10000;

    // Total Period Count = 5MHz / 10Khz = 500 clock ticks
    totalPWMPeriodCount = PWMClockCount/desiredFrequency;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, totalPWMPeriodCount);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);

    // Enable generator and output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
}

void InitializeADC0()
{
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    // Right Sensor
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Front Sensor
    GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH10 | ADC_CTL_END | ADC_CTL_IE);
    ADCSequenceEnable(ADC0_BASE, 1);
}

void InitializeGPIODInterrupt()
{
    IntEnable(INT_GPIOD);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_2);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntRegister(INT_GPIOD, GPIODIntHandler);
    IntPrioritySet(INT_GPIOD, 0xA0);
}

void RobotStart()
{
    TimerEnable(WTIMER1_BASE, TIMER_A);
    TimerEnable(WTIMER2_BASE, TIMER_A);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
}

void RobotStop()
{
    xSemaphoreGive(xTask5Semaphore);
}

void Forward()
{
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, totalPWMPeriodCount);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, totalPWMPeriodCount);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
}

void Brake()
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);
}

void Reverse()
{
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, totalPWMPeriodCount);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, totalPWMPeriodCount);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
}


void StartUTurn()
{
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, totalPWMPeriodCount);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, totalPWMPeriodCount);
}

void StopUTurn()
{
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
}

void StartRightTurn()
{
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, totalPWMPeriodCount);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, totalPWMPeriodCount * 0.4);
}

void StopRightTurn()
{
}

void executePID(uint32_t rightADCValue)
{
    int errorADC = rightADCValue - (int)CENTER_TARGET_ADC;
    float errorPercent = errorADC/(float)CENTER_TARGET_ADC * 100;

    // Collect ADC error if data collection is enabled
    if(dataCollectionEnabled == true)
    {
        collectBuffer[currentDataIdx] = abs(errorADC);
        currentDataIdx++;
        if(currentDataIdx == MAX_DATA_NUM)
        {
            SignalDataTransmit();
        }
    }

    float P = errorPercent;
    float I = integralPrior + errorPercent * (float)CONTROL_ITERATION_TIME;
    float D = (errorPercent - errorPrior)/(float)CONTROL_ITERATION_TIME;

    float output = fabs(Kp*P + Ki*I + Kd*D);

    if(output > 100)
    {
        output = 100.0;
    }

    if(errorPercent >= 0)
    {
        SteerLeft(output);
    }
    else
    {
        SteerRight(output);
    }

    errorPrior = errorPercent;
    integralPrior = I;

}

void SteerLeft(float adjustPercentage)
{
    uint32_t adjustAmountCount = totalPWMPeriodCount * (1 - adjustPercentage * 0.01);
    if(adjustAmountCount == 0)
    {
        adjustAmountCount = 1;
    }
    // Slow down left motor pwm by a percentage
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, adjustAmountCount);
    // Keep right motor pwm at max speed
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, totalPWMPeriodCount);
}

void SteerRight(float adjustPercentage)
{
    uint32_t adjustAmountCount = totalPWMPeriodCount * (1 - adjustPercentage * 0.01);
    if(adjustAmountCount == 0)
    {
        adjustAmountCount = 1;
    }
    // Keep left motor pwm at max speed
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, totalPWMPeriodCount);
    // Slow down right motor pwm by a percentage
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, adjustAmountCount);
}

void SignalDataTransmit()
{
    // Save the index of where data ends
    endDataIdx = currentDataIdx;
    currentDataIdx = 0;

    // Select buffer to send data
    sendBuffer = collectBuffer;

    // Swap collect buffer
    if(collectBuffer == pingBuffer)
    {
        collectBuffer = pongBuffer;
    }
    else
    {
        collectBuffer = pingBuffer;
    }
    xSemaphoreGive(xTask4Semaphore);
}

int main(void)
{
    // Set the clocking to run at 40 MHz from the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Allow processor to respond to interrupts
    IntMasterEnable();

    EnableSystemPeripherals();

    InitializeGPIOF();

    InitializeUART0();

    InitializeUART1();

    InitializeWTimer0();

    InitializeWTimer1();

    InitializeWTimer2();

    InitializePWM0();

    InitializeADC0();

    InitializeGPIODInterrupt();

    // Create Semaphores
    xTask1Semaphore = xSemaphoreCreateBinary();
    xTask2Semaphore = xSemaphoreCreateBinary();
    xTask4Semaphore = xSemaphoreCreateBinary();
    xTask5Semaphore = xQueueCreate(2, 0);
    xTask6Semaphore = xSemaphoreCreateBinary();

    xUART1Semaphore = xSemaphoreCreateBinary();
    xControlDataQueue = xQueueCreate(5, sizeof(SensorData_t));  // Queue for sensor data

    xTaskCreate(vTask1, "Task 1", 256, NULL, 10, NULL);
    xTaskCreate(vTask2, "Task 2", 256, NULL, 8, NULL);
    xTaskCreate(vTask3, "Task 3", 256, NULL, 4, NULL);
    xTaskCreate(vTask4, "Task 4", 256, NULL, 3, NULL);
    xTaskCreate(vTask5, "Task 5", 256, NULL, 2, NULL);
    xTaskCreate(vTask6, "Task 6", 256, NULL, 1, NULL);

    xSemaphoreGive(xUART1Semaphore);
    UARTprintf("Running main program.\n");

    SysCtlDelay(SysCtlClockGet()/3 * 2);
    RobotStart();

    // Start Task Scheduler
    vTaskStartScheduler();

    // Print error message if task scheduler returns
    UARTprintf("Task Scheduler returned unexpectedly, please restart the program.");

    return -1;
}
