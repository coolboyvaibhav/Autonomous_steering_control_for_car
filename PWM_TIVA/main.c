/*################################################
# Hardware PWM proof of concept using
# the Tiva C Launchpad
#
# Started with example code by
# lawrence_jeff found here:
# http://forum.stellarisiti.com/topic/707-using-hardware-pwm-on-tiva-launchpad/
#
# Altered to use code found on section
# 22.3 of the TivaWare Peripheral Driver
# Library User's Guide found here:
# http://www.ti.com/lit/ug/spmu298a/spmu298a.pdf
#
#
# This example pulses three on-board LEDs
#
#################################################*/


#include "driverlib/pin_map.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}
unsigned long increment ;
unsigned long pwmNow ;// pwm duty cycle can vbe chaanges from here-- pwm_out is at the pf1,6--> at pf2,7 at pf3.

int main(void)
{
    increment = 10;
    pwmNow = 300;
    //Set the clock-using 16MHz freq
   SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

   //Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

   // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    //Configure PB4,PB5,PB6,PB7 Pins as PWM
//    GPIOPinConfigure(GPIO_PF1_M1PWM5);
//    GPIOPinConfigure(GPIO_PF2_M1PWM6);
//    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);//12.5%duty cycle
    GPIOPinConfigure(GPIO_PB5_M0PWM3);//6.25%dutycycle
    GPIOPinConfigure(GPIO_PB6_M0PWM0);//50% duty cycle
    GPIOPinConfigure(GPIO_PB7_M0PWM1);//25%duty cycle

    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |GPIO_PIN_7);

    //Configure PWM Options
    //PWM_GEN_0 Covers M0PWM0 and M0PWM1
    //PWM_GEN_1 Covers M0PWM2 and M1PWM3 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //Set the Period (expressed in clock ticks)-period is 320 clock ticks-50KHz freq
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 320);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 320);

    //Set PWM duty-50% (Period /2)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,160);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,80);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,40);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,20);

    // Enable the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    // Turn on the Output pins--> how to assign?????--->pwm 01,2,3 are used so out bits will be 0,1,2,3.
   // PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

    //Fade
//    bool fadeUp = true;


    while(1)
    {
        delayMS(20);
//        if (fadeUp) {
//            pwmNow += increment;
//            if (pwmNow >= 320) { fadeUp = false; }
//        }
//        else {
//            pwmNow -= increment;
//            if (pwmNow <= 10) { fadeUp = true; }
//        }
//        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,pwmNow);
//        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,pwmNow);
//        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,pwmNow);
//        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,pwmNow);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,160);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,80);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,40);
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,20);

    }

}
