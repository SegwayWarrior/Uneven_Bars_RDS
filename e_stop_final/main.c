#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
//#include "driverlib/debug.h"
//#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/watchdog.h"
//#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

#define LED_PERIPH SYSCTL_PERIPH_GPIOF
#define LED_BASE GPIO_PORTF_BASE
#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2

#define Button_PERIPH SYSCTL_PERIPH_GPIOF
#define ButtonBase GPIO_PORTF_BASE
#define Button GPIO_PIN_4
#define ButtonInt GPIO_INT_PIN_4

uint8_t estop_status = 0;   // estop flag, must be declared external in other code files

void ButtonHandler(void) { // Sets estop_status flag to true on button press interupt
    if (GPIOIntStatus(GPIO_PORTF_BASE, false) & GPIO_PIN_4) { // PF4 pin was interrupt cause
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);  // Clear interrupt flag
        estop_status = 1;
    }
}

void Timer0Handler(void){ // checks for estop flag every timer cycle, set to 100Hz in main
    if(estop_status){
        GPIOPinWrite(LED_BASE,BLUE_LED, BLUE_LED);
    }
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void UART_setup(void){  // Uart set up
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
   GPIOPinConfigure(GPIO_PA0_U0RX);
   GPIOPinConfigure(GPIO_PA1_U0TX);
   GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
   UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
   UARTStdioConfig(0, 115200, 16000000);
 }

int main(void) {
    // Set clock to 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Setup UART for debuggin purposes
    UART_setup();

    // Pin F4 (button) setup
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);        // Enable port F which also enables LED pins
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);  // Init PF4 as input
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4,
        GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);  // Enable weak pullup resistor for PF4

    // Button interrupt setup
    GPIOIntDisable(GPIO_PORTF_BASE, GPIO_PIN_4);        // Disable interrupt for PF4 (in case it was enabled)
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);      // Clear pending interrupts for PF4
    GPIOIntRegister(GPIO_PORTF_BASE, ButtonHandler);     // Register our handler function for port F
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4,
        GPIO_FALLING_EDGE);                             // Configure PF4 for falling edge trigger
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);     // Enable interrupt for PF4

   // configure Led pins as output
    GPIOPinTypeGPIOOutput(LED_BASE, RED_LED | BLUE_LED);

//    IntMasterEnable();                          // Not needed but handy

    // configure timer.  Timer0Handler saved in startup file
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Enable timer0's peripheral

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);  //  Make timer periodic
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/100); // 100 hz timer

    IntEnable(INT_TIMER0A);                     // Setup the interrupts for the timer timeouts.
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER0_BASE, TIMER_A);          // Enable Timer0, subtimer A

    while(1){
        // Weird problem where if statement wouldn't work without either a UARTprintf or a delay
        // Im guessing its a timing issues but im not sure
        SysCtlDelay(3);
        if(estop_status == 1){
            GPIOPinWrite(LED_BASE,RED_LED, RED_LED);
        }
    }
}
