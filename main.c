#include <stdint.h> //for using uint32 and similar variables
#include <stdbool.h> //fr using boolean operation, true/false
#include <inc/tm4c123gh6pm.h> //definitions of interrupts and register assignments
#include <inc/hw_memmap.h> //hardware memory map
#include <inc/hw_types.h>
#include <driverlib/sysctl.h>
#include <driverlib/interrupt.h> //handler for interrupts, handles priority and enable/disable
#include <driverlib/gpio.h>
#include <driverlib/timer.h>

int main(void){
    uint32_t ui32Period; //to compute timer delays
    int i=0;
    SysCtlClockSet(SYSCTL_SYSDIV_5| SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN ); //configuring system clock to run at 40 Mhz

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);//Enable gpio peripheral for port F
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3); //configure pins connected to leds, pin f1(R),f2(G),f3(B) as output

    //before calling peripheral specific driverlib we need to enable clock to that peripheral

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); //Enable  timer 0;
    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC); //timer 0 enabled in periodic mode and as 32 bit timer by combining timer 0a and 0b, by calling timer0_base

    /*NEED TO MODIFY THIS STUFF AS PER OUR NEEDS*/
    ui32Period = (SysCtlClockGet()/1/2); //here, we are toggling the interrupt at 10Hz (hence the 10) and  at 50% duty cycle(hence again divide by 2)
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period-1); //calculcated period is loaded into the timer's interval load register; need to subtract 1
    //from the timer period since the interrupt fires at the zero count

    //ENABLING INTERRUPTS
    IntEnable(INT_TIMER0A); //enables specific vectors associated with timer0A
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT); //enables a specific event within the  timer to generate an interrupt
    IntMasterEnable(); // master interrupt enable API for all interrupts

    TimerEnable(TIMER0_BASE,TIMER_A);

    while(1){
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , 8);
        while(i<10000000){
            i++;
        }
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , 0);
        while(i<10000000){
                    i++;
                }
    }
}

void Timer0IntHandler(void){
    TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT); //clear the timer interrupt
    //read current state of gpio pin and write opposite state
    if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1)){
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 , 0);
    }
    else{
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,2 ); // if using pin 1, make last digit as 2(red);if using pin 2(blue), make last digit as 4;if using pin 3(green), make last digit as 8
    }
}
