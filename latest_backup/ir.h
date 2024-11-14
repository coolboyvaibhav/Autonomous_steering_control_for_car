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

//*********************Sensors*****************************//

#define INT_PB0                 0x01          // reads 1st sensor
#define INT_PB1                 0x02          // reads 2nd sensor

//********* PORT B Initializations. Base Address is 0x40005000 **********//
//#define GPIO_PORTB_DATA_R       (*((volatile unsigned long*) 0x400053FC))
//#define GPIO_PORTB_DIR_R        (*((volatile unsigned long*) 0x40005400))
//#define GPIO_PORTB_DEN_R        (*((volatile unsigned long*) 0x4000551C))
//#define GPIO_PORTB_PDR_R        (*((volatile unsigned long*) 0x40005514))

//********* Defining all Macros ***********//
#define PORTB_CLK_EN        0x02          // clock enable for port B

void init_ir_sensor(void);



void init_ir_sensor(void){
    SYSCTL_RCGCGPIO_R |=  PORTB_CLK_EN;
    volatile unsigned delay_clk;
    delay_clk           = SYSCTL_RCGCGPIO_R;
    GPIO_PORTB_DEN_R    |= INT_PB0 + INT_PB1;         // Digitally enabling the ports PB1 and PB2
    GPIO_PORTB_DIR_R    &= ~( INT_PB0 + INT_PB1) ;    // setting the direction of PF1 and PF2 as INPUT
    GPIO_PORTB_PDR_R    |= ( INT_PB0 + INT_PB1) ;     // Pull down Register
//    GPIO_PORTA_DEN_R  |= PORTA_DEN;                 // Digitally enabling the ports PB0 to PB3
//    GPIO_PORTA_DIR_R    |=  PORTA_DIR;                // setting the direction of PA2 to PA5 as OUTPUT
}

#endif /* IR_H_ */
