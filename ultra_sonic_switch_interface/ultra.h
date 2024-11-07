/*
 * ultra.h
 *
 *  Created on: 03-Nov-2024
 *      Author: vaibh
 */

#ifndef ULTRA_H_
#define ULTRA_H_
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <./inc/tm4c123gh6pm.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/interrupt.h>
#include <driverlib/systick.h>
#include <driverlib/timer.h>
#include <driverlib/qei.h>
#include "driverlib/debug.h"

void delay_us(int n);
// Ultrasonic Sensor Trigger Function
void send_trigger_pulse(void) {
    GPIO_PORTD_DATA_R |= 0x00;
    delay_us(10);
    GPIO_PORTD_DATA_R = 0x04;
    delay_us(10);
    GPIO_PORTD_DATA_R = 0x00;
}

// Delay in microseconds
void delay_us(int n) {
    int i, j;
    for (i = 0; i < n; i++) {
        for (j = 0; j < 3; j++) { }  // Adjust this loop to achieve accurate delays
    }
}


void HC_SR04_setup(void)
{
    SYSCTL_RCGCGPIO_R |= 0x08;      // Enable clock to GPIO PORTD
    GPIO_PORTD_DIR_R |= 0x04;       // Set PD2 as output (trigger) and PD3 as input (echo)
    GPIO_PORTD_DEN_R |= 0x0C;       // Enable bit 2 and 3 of PORTD
    GPIO_PORTD_AFSEL_R = 0x08;      // Set alternate function WTIMER3CCP1
    GPIO_PORTD_PCTL_R &= ~0x0000F000;
    GPIO_PORTD_PCTL_R |= 0x00007000;// PORTD bit 3 selected for alternate function
    // Set up timer WT3CCP1
    SYSCTL_RCGCWTIMER_R |= 8;       // Enable clock to to timer 3
    WTIMER3_CTL_R &= ~0x0100;       // Disable timer during setup
    WTIMER3_CFG_R = 0x00000004;     // 32 bit mode
    WTIMER3_TBILR_R = (608000);     // Timer LOAD REG value = 38 ms
    // Configure TIMER_B
    // bits 1:0 = [11] Capture Mode
    // bit 2    = 1    Capture Mode - Edge Time
    // bit 3    = 0    Capture Mode enabled
    // bit 4    = 0    Count down
    WTIMER3_TBMR_R = 0x07;          // Configure TIMER_B
    WTIMER3_CTL_R |= 0x0C00;        // Detect both edges
    WTIMER3_CTL_R |= 0x0100;        // Enable Timer
    WTIMER3_ICR_R = 0x0400;         // Clear event interrupt flags
    WTIMER3_IMR_R |= 0x0400;        // Unmask interrupt for capture event mode

    NVIC_EN3_R |= 0x00000020;       // NVIC enabled (IRQ number = 102)
    //NVIC_PRI25_R |= 0x00600000;     // Set priority = 3
}


#endif /* ULTRA_H_ */
