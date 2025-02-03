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
#define NUM_COMMANDS 2

const uint32_t DECAY_COUNT_THRESHOLD = 20000;
const uint16_t TAPE_MS_THRESHOLD = 2000;
const uint8_t TAPE_MS_MIN = 10;

// Semaphore definitions
SemaphoreHandle_t xUART1Semaphore;
SemaphoreHandle_t xTask1Semaphore;

void robotSTART();
void robotSTOP();
void forward();
void brake();
void reverse();

typedef void (*FunctionPointer)(void);

typedef struct
{
    char *command;
    FunctionPointer funcPtr;
} CommandCallback;

volatile char commandBuffer[COMMAND_MAX_LEN];
volatile uint8_t commandIdx = 0;

volatile uint32_t totalPWMPeriodCount = 0;
volatile uint64_t decayStartCount = 0;
volatile uint64_t decayElapsedCount = 0;
volatile uint64_t onTapeStartCount = 0;
volatile uint64_t onTapeElapsedCount = 0;
volatile uint64_t onTapeElapsedms = 0;
volatile bool wasOnTape = false;

const CommandCallback commandCallbacks[NUM_COMMANDS] =
{
    { "STR", robotSTART },
    { "STP", robotSTOP },
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

void GPIODIntHandler(void)
{
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
            UARTprintf("%i\n", (int)onTapeElapsedms);

            if(onTapeElapsedms > TAPE_MS_MIN)
            {
                // Thin line crossed
                if(onTapeElapsedms < TAPE_MS_THRESHOLD)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE,
                                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                                 GPIO_PIN_2);
                }
                // Thick line crossed
                else
                {
                    GPIOPinWrite(GPIO_PORTF_BASE,
                                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
                                 GPIO_PIN_3);
                }
            }
        }
    }
}

void vTask0(void* pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xUART1Semaphore, portMAX_DELAY) == pdTRUE)
        {
            UARTprintf("Task 0 is printing\n");
            xSemaphoreGive(xUART1Semaphore);
        }

    }
}

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
    UARTStdioConfig(0, 9600, SysCtlClockGet());
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

    IntPrioritySet(INT_UART1, 0xA0);
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    UARTEnable(UART1_BASE);
//    UARTStdioConfig(1, 9600, SysCtlClockGet());
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
    TimerEnable(WTIMER1_BASE, TIMER_A);
}

void InitializePWM0()
{

    // Setup PWM pins
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    // Setup phase pins
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);

    // Sys Clock = 40Mhz PWMClock = 40/8 = 5MHz
    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

    uint32_t PWMClockCount = SysCtlClockGet() / 8;

    // Desired frequency = 10Khz
    uint32_t desiredFrequency = 10000;

    // 5MHz / 10Khz = 500 clock ticks
    totalPWMPeriodCount = PWMClockCount/desiredFrequency;

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, totalPWMPeriodCount);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);

    // Enable generator and output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
}

void InitializeGPIODInterrupt()
{
    IntEnable(INT_GPIOD);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_2);
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntRegister(INT_GPIOD, GPIODIntHandler);
    IntPrioritySet(INT_GPIOD, 0xA0);
}

void robotSTART()
{
    UARTprintf("START CALLED\n");
}

void robotSTOP()
{
    UARTprintf("STOP CALLED\n");
}

void forward()
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, totalPWMPeriodCount);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, totalPWMPeriodCount);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
}

void brake()
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 1);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
}

void reverse()
{

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

    InitializePWM0();

    InitializeGPIODInterrupt();

//    GPIOPinWrite(GPIO_PORTF_BASE,
//                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
//                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);


    // Create Semaphores
    xTask1Semaphore = xSemaphoreCreateBinary();
    xUART1Semaphore = xSemaphoreCreateBinary();

//    xTaskCreate(vTask0, "Task 0", 256, NULL, 1, NULL);
    xTaskCreate(vTask1, "Task 1", 256, NULL, 2, NULL);

    xSemaphoreGive(xUART1Semaphore);
    UARTprintf("Running main program.\n");


    // Start Task Scheduler
    vTaskStartScheduler();

    // Print error message if task scheduler returns
    UARTprintf("Task Scheduler returned unexpectedly, please restart the program.");

    return -1;
}
