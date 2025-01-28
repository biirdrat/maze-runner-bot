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

// Semaphore definitions
SemaphoreHandle_t xUARTSemaphore;
SemaphoreHandle_t xTask1Semaphore;

void robotSTART();
void robotSTOP();

typedef void (*FunctionPointer)(void);

typedef struct
{
    char *command;
    FunctionPointer funcPtr;
} CommandCallback;

volatile char commandBuffer[COMMAND_MAX_LEN];
volatile uint8_t commandIdx = 0;

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


void vTask0(void* pvParameters)
{
    while (1)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        vTaskDelay(pdMS_TO_TICKS(1000));
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vTask1(void* pvParameters)
{
    while (1)
    {
//        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
//        SysCtlDelay(SysCtlClockGet()/3 * 1);
//        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
//        SysCtlDelay(SysCtlClockGet()/3 * 1);
        // Wait for the semaphore to be given
        if (xSemaphoreTake(xTask1Semaphore, portMAX_DELAY) == pdTRUE)
        {
//            SysCtlDelay(SysCtlClockGet()/3 * 10);
            // Toggle the LED
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
//            vTaskDelay(pdMS_TO_TICKS(500));
//            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
//            vTaskDelay(pdMS_TO_TICKS(500));
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

    IntPrioritySet(INT_UART1, 0xA0);
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    UARTEnable(UART1_BASE);
    UARTStdioConfig(1, 9600, SysCtlClockGet());
}

// Initialize timer that keeps track of program time
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

void robotSTART()
{
    UARTprintf("START CALLED\n");
}

void robotSTOP()
{
    UARTprintf("STOP CALLED\n");
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

//    GPIOPinWrite(GPIO_PORTF_BASE,
//                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
//                 GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);


    // Create Semaphores
    xTask1Semaphore = xSemaphoreCreateBinary();

//    xTaskCreate(vTask0, "Task 0", 256, NULL, 1, NULL);
//    xTaskCreate(vTask1, "Task 1", 256, NULL, 2, NULL);

    UARTprintf("Running main program.\n");


    // Start Task Scheduler
    vTaskStartScheduler();

    // Print error message if task scheduler returns
    UARTprintf("Task Scheduler returned unexpectedly, please restart the program.");

    return -1;
}
