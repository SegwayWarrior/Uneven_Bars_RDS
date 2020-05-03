

/**
 * main.c
 * single GPIO-level channel is HIGH when e-stop is OFF and LOW when ON
 * line routed to the enable inputs of motor controllers and to GPIO pin on TIVA
 *
 * behavior is as follows:
 *      clearing to LOW triggers hardware interrupt on TIVA with highest possible priority
 *      e-stop ISR produces a signal detected by other ISRs and main routine
 *      main routine and other ISRs prevented from changing state of this flag
 *
 * main loop polls e-stop state and turns on red channel when e-stop event occurs
 *
 * timer interrupt fires at 100Hz and turns on blue channel when e-top event occurs
 *
 * when e-stop line is manually asserted (pulled LOW), LED glows purple
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"

int main(void)
{
    initGPIO();
    // main loop polls e-stop state
	return 0;
}

void initGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    //LEDS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    //use PORTB for interrupt

    //do I need this...? GPIOPinConfigure(uint32_t ui32PinConfig)...

    /*
    void GPIOPinTypeGPIOInput(uint32_t ui32Port, uint8_t ui8Pins)
    Parameters:
    ui32Port is the base address of the GPIO port.
    ui8Pins is the bit-packed representation of the pin(s).
     */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2);  //PF1 RED LED, PF2 BLUE LED, not totally sure what the OR means here?
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_LOW_LEVEL);    //sets interrupt detection to low level

    GPIOIntRegister(GPIO_PORTB_BASE, ESTOP);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0);

    /*
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0);
    IntPrioritySet(INT_GPIOB, 6);
    IntRegister(INT_GPIOB, ESTOP);  //name of ISR
    IntEnable(INT_GPIOB);
    IntMasterEnable();
     */


}

void ESTOP(void){  //should this have the priority and sub-priority levels?...IPL6SRS
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0);

    //TOGGLE LEDS

    //uint8_t valueRED = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1);
    //uint8)t valueBLUE = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2);

    while(1)
    {
        GPIO_PORTF_DATA_R |= 0x02; //turn on Red LED on PF1
        GPIO_PORTF_DATA_R |= 0x04; //turn on Blue LED on PF2


        for(...) //delay, want 100Hz
        {
            ;
        }

        GPIO_PORTF_DATA_R &= ~(0x02);   //turn off Red LED
        GPIO_PORTF_DATA_R &= ~(0x04);   //turn off Blue LED

        for(...)
        {
            ;
        }

    }

}

/*
 * example resource: https://gist.github.com/EddyJMB/73a907524c3fbe8f6c0ee9ecefc0c52d
 */

/*
 * confused whether GPIO Pin generates interrupt or timer because instructions say
 * "timer interrupt fires at 100Hz...?
 *
 * this tutorial seems to do it one way then convert to the other way while keeping the same function
 *
 * https://www.youtube.com/watch?v=bQ50pzqFA5Q
 */
