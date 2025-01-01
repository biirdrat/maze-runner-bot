#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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

SemaphoreHandle_t xTask0Semaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

void WTimer1IntHandler(void)
{
    TimerIntClear(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    xSemaphoreGive(xTask0Semaphore);
}

void vTask0(void* pvParameters)
{
    while (1)
    {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(xTask0Semaphore, portMAX_DELAY) == pdTRUE)
        {
            // Toggle the LED
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
//            vTaskDelay(pdMS_TO_TICKS(500));
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
//            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void vTask1(void* pvParameters)
{
    while (1)
    {
        while(1)
        {
            int x = 0;
        }
    }
}

void EnableSystemPeripherals()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
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

void InitializeGPIOFLEDs()
{
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

void InitializeUART0LocalTerminal()
{
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTEnable(UART0_BASE);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
}

void InitializeWTimer0()
{
    TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT_UP);
    TimerLoadSet64(WTIMER0_BASE, 18446744073709551615);
    TimerEnable(WTIMER0_BASE, TIMER_A);
}

void InitializeWTimer1()
{
    TimerConfigure(WTIMER1_BASE, TIMER_CFG_A_PERIODIC | TIMER_CFG_SPLIT_PAIR);
    TimerIntEnable(WTIMER1_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_WTIMER1A);
    IntPrioritySet(INT_WTIMER1A, 0xA0);
    IntRegister(INT_WTIMER1A, WTimer1IntHandler);
    TimerLoadSet(WTIMER1_BASE, TIMER_A, SysCtlClockGet() * 5);
    TimerEnable(WTIMER1_BASE, TIMER_A);

}


int main(void)
{
    // Set the clocking to run at 40 MHz from the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Allow processor to respond to interrupts
    IntMasterEnable();

    EnableSystemPeripherals();

    InitializeGPIOFLEDs();

    InitializeUART0LocalTerminal();

    InitializeWTimer0();

    InitializeWTimer1();

//    GPIOPinWrite(GPIO_PORTF_BASE,
//                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
//                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);


    // Create Semaphores
    xTask0Semaphore = xSemaphoreCreateBinary();

    xTaskCreate(vTask0, "Task 0", 128, NULL, 1, NULL);
    xTaskCreate(vTask1, "Task 1", 128, NULL, 0, NULL);

    UARTprintf("Running main program.\n");


    // Start Task Scheduler
    vTaskStartScheduler();

    // Print error message if task scheduler returns
    UARTprintf("Task Scheduler returned unexpectedly, please restart the program.");

    return -1;
}
