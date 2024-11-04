/*
 * switches.h
 *
 *  Created on: 03-Nov-2024
 *      Author: vaibh
 */

#ifndef SWITCHES_H_
#define SWITCHES_H_
/* useful links
 * http://shukra.cedt.iisc.ernet.in/edwiki/EmSys:Lab_Practicals_Using_TivaC_LaunchPad_Board
 * https://microcontrollerslab.com/gpio-interrupts-tm4c123-tiva-launchpad-edge-level-triggered/
 * http://shukra.cedt.iisc.ernet.in/edwiki/EmSys:TM4C123_Using_SysTick_with_Interrupt
 */
// For switch 1 state
// 0xF0 means Run state
// 0x0F means Stop State
int state_sw1 = 0x0F;
// For switch 2 state
// 0xF0 means Run state
// 0x0F means Pause State
int state_sw2 = 0xF0;
// Flag variable to detect if interrupt has occurred, used for 1ms time delay using SysTick_Handler
// It is inverted every time interrupt handler is called. (0x55 <---> 0xAA)
int flag_1ms = 0x55;
// sw_interrupt_flag is set when switch interrupts are detected first, then there is cool off time of 200ms
// during which if an interrupt is detected, it is ignored (assuming humans dont press switches that fast so it is wrong detection / debouncing effect of physical switch)
// cool_off_time is updated in SysTick_Handler() and reset along with sw_interrupt_flag after 200ms
int sw1_interrupt_flag = 0x00;
int sw2_interrupt_flag = 0x00;
int sw1_cool_off_time = 0;
int sw2_cool_off_time = 0;


void GPIO_setup()
{
//    SYSCTL_RCGC2_R |= 0x00000023;     /* enable clock to GPIOA, GPIOB and GPIOF at clock gating control register */
    SYSCTL_RCGC2_R |= 0x0000002B; // Enable clock to GPIOA, GPIOB, GPIOD, and GPIOF

    GPIO_PORTF_LOCK_R = 0x4C4F434B;   /* unlock commit register GPIOCR */
    GPIO_PORTF_CR_R = 0x01;           /* make PORTF0 configurable */
    // Enable the GPIO pins
    // For PORTF the LED (PF3, 2 1) as output
    // For PORTB, all pins are used to set 7 segment display
    // For PORTA, pins 7 to 4 are used for selecting one of the four 7 segment display
    //GPIO_PORTA_DIR_R |= 0xF0;
    //GPIO_PORTB_DIR_R |= 0xFF;
    // X X X SW1 G B R SW2 -> bits in PORTF
    GPIO_PORTF_DIR_R |= 0x0E;
    // enable the GPIO pins for digital function
//    GPIO_PORTA_DEN_R = 0xF0;
//    GPIO_PORTB_DEN_R = 0xFF;
    GPIO_PORTF_DEN_R = 0x1F;
    GPIO_PORTF_PUR_R = 0x11;          // enable pull up for SW1 and SW2

    // Interrupt setup for sw1 and sw2
    // Interrupt sense register sets value for bits 4 and 0. (1 - level triggered, 0 - edge triggered)
    GPIO_PORTF_IS_R &= ~(0x11);
    // Interrupt Both Edges is not set
    GPIO_PORTF_IBE_R &= ~(0x11);
    // If both edges not set then
    // Trigger controlled by IEV (Interrupt Even Register) for negative/positive or rising/falling
    GPIO_PORTF_IEV_R &= ~(0x11);
    // Clearing previous interrupts if any
    GPIO_PORTF_ICR_R |= (0x11);
    // Unmask interrupt for sw1 and sw2 (bit 4 and 0)
    GPIO_PORTF_IM_R |= 0x11;
    // set interrupt priority = 5 for PORTF
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000;
    NVIC_EN0_R = 0x40000000;        // NVIC enabled for PORTF (IRQ number = 30)
}




#endif /* SWITCHES_H_ */
