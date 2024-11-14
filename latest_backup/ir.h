/*
 * ir.h
 *
 *  Created on: 14-Nov-2024
 *      Author: vaibh
 */

#ifndef IR_H_
#define IR_H_
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


//********* Defining all Macros ***********//
#define PORTB_CLK_EN        0x02          // clock enable for port B
#define PORTA_CLK_EN        0x01          // clock enable for port A
#define LEDs                      0x0E          // enables all LEDS

//*********************Sensors*****************************//

#define INT_PB0                 0x01          // reads 1st sensor
#define INT_PB1                 0x02          // reads 2nd sensor
#define INT_PB2                 0x04          // reads 3rd sensor
#define INT_PB3                 0x08          // reads 4th sensor
#define INT_PB4                 0x10          // reads 5th sensor
#define INT_PB5                 0x20          // reads 6th sensor

#define PA2                         0x04          // pin 1 of L298
#define PA3             0x08          // pin 2 of L298
#define PA4                         0x10          // pin 3 of L298
#define PA5                         0x20          // pin 4 of L298

#define PORTA_DIR         0xFF          // PA2- PA5 ports will have same direction
#define PORTA_DEN             0xFF          // PA2- PA5 will all be digitally enabled
void init_ir_sensor(void);



void init_ir_sensor(void){
    SYSCTL_RCGCGPIO_R |=  PORTB_CLK_EN;
    volatile unsigned delay_clk;
    delay_clk           = SYSCTL_RCGCGPIO_R;
    GPIO_PORTB_DEN_R     |= INT_PB0 + INT_PB1+ INT_PB2+ INT_PB3+ INT_PB4+ INT_PB5;         // Digitally enabling the ports PF1 and PF2
        GPIO_PORTB_DIR_R    &= ~( INT_PB0 + INT_PB1+ INT_PB2+ INT_PB3+ INT_PB4+ INT_PB5) ;    // setting the direction of PF1 and PF2 as INPUT
        GPIO_PORTB_PDR_R    |= ( INT_PB0 + INT_PB1+ INT_PB2+ INT_PB3+ INT_PB4+ INT_PB5) ;     // Pull down Register
//    GPIO_PORTA_DEN_R  |= PORTA_DEN;                 // Digitally enabling the ports PB0 to PB3
//    GPIO_PORTA_DIR_R    |=  PORTA_DIR;                // setting the direction of PA2 to PA5 as OUTPUT
}

#endif /* IR_H_ */
