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


//import user defined files
#include "ultra.h"

#define red_on GPIO_PORTF_DATA_R |= 0x02;
#define magenta_on GPIO_PORTF_DATA_R = 0x06;
#define blue_on GPIO_PORTF_DATA_R = 0x04;
#define cyan_on GPIO_PORTF_DATA_R = 0x0C;
#define green_on GPIO_PORTF_DATA_R = 0x08;
#define yellow_on GPIO_PORTF_DATA_R = 0x0A;
#define white_on GPIO_PORTF_DATA_R = 0x0E;
#define led_off GPIO_PORTF_DATA_R = 0x00;


#define MAX_POS 8191
#define fcpu 16000000
#define edges 4
#define PPR 2047

#define max_speed 3000
#define min_speed 0


#define set_speed 1500
unsigned long long int measured_speed = 0;


volatile long long unsigned int timing_data[4] = {0};
volatile long long unsigned int current_ma = 0;

char adc_val_string[10] = {0};

void printstring(char *str);
void UART0_Transmitter(unsigned char data);
void delay_ms(int n);
unsigned int calculate_distance(void) ;
unsigned int calculate_distance(void);
void uart_print_num(int res);
void systick_init(void);
void uart_init(void);
void gpio_init(void);
void SysTick_Handler(void);
void WTIMER3B_Handler(void);
void  EnableInterrupts(void);
void systick_setup(void);



int count= 0;
bool state;
unsigned int time_value1 = 0;
unsigned int time_value2 = 1;
unsigned int distance;
int update_flag = 0;
int timer_200_msec = 0;


int main(void)
{
    __asm  ("CPSIE  I\n");      // Change processor state-enable interrupts
    HC_SR04_setup();
//    gpio_init();
    uart_init();
//    adc_init();
//    pwm_init();
//    qei_init();
    //GPIO_PORTA_DATA_R |= (1<<2);          //Drive pin
   state = 0;                       // Solenoid OFF

   systick_setup();
   //led_off;
   //printstring("Hello\n\r");
//   EnableInterrupts();

   printstring("Hello\n\r");
    while (1)
    {
        if (update_flag == 1)// 200msec parameter update
            {
                distance = calculate_distance();
                update_flag = 0;                    // reset parameter update bit
                send_trigger_pulse();
                printstring("the distance is  ");
                uart_print_num(distance);
                printstring("\n\r");
//                delay_ms(200);
            }

//            else
//            {
//                printstring("OFF ");
//                printstring("\n\r");
//            }
    }

    return 0;
}


void  EnableInterrupts(void)
{
    // Global interrupts enable
    __asm  ("CPSIE  I\n");      // Change processor state-enable interrupts
}

void SysTick_Handler(void)
{
    timer_200_msec++;
        if(timer_200_msec == 200)
        {
            update_flag = 1;
            timer_200_msec = timer_200_msec - 200;
        }
}


void gpio_init(void)
{

        //RED LED(for status)
        SYSCTL_RCGC2_R |= 0x00000020; // Activate clock for Port F
        GPIO_PORTF_LOCK_R = 0x4C4F434B; // Unlock GPIO Port F
        GPIO_PORTF_CR_R = 0x1F;         // Allow changes to PF4-0
        GPIO_PORTF_AMSEL_R = 0x00;      // Disable analog on PF
        GPIO_PORTF_PCTL_R = 0x00000000; // PCTL GPIO on PF4-0
        GPIO_PORTF_DIR_R = 0x0E;        // PF4, PF0 in, PF3-1 out
        GPIO_PORTF_AFSEL_R = 0x00;      // Disable alt funct on PF7-0
        GPIO_PORTF_PUR_R = 0x11;        // Enable pull-up on PF0 and PF4
        GPIO_PORTF_DEN_R = 0x1F;        // Enable digital I/O on PF4-0

        __asm("CPSIE I\n");                                    // Interrupt enable using asm

        //IN1 AND IN2 PINS FOR MOTOR PA3 AND PA2
            SYSCTL_RCGCGPIO_R |= 0x01;
            GPIO_PORTA_DIR_R |= (1 << 3) | (1 << 2);
            GPIO_PORTA_DEN_R |= (1 << 3) | (1 << 2);
            GPIO_PORTA_DR8R_R |= (1 << 2);


            GPIO_PORTA_DATA_R |= (1 << 3);
            GPIO_PORTA_DATA_R &= ~(1 << 2);
            GPIO_PORTF_DATA_R = 0x00;

}



void uart_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void systick_setup(void)
{
    // clk_freq = 16 MHz
    // 1 tick = 1 / (16000000) = 62.5 ns
    // for time_delay = 1 ms
    // Reload value = (clk_freq * time_delay) - 1
    //              = 16000 - 1
    // reload with number of clocks per second
    NVIC_ST_RELOAD_R = 16000-1;
    // enable SysTick interrupt, use system clock
    NVIC_ST_CTRL_R = 7;
}

void uart_print_num(int res)
{
    int i, rem, len = 0, n;
if(res <= 0)
{
printstring("0 ");
return;
}
        n = res;
        while (n != 0)
        {
            len++;
            n /= 10;
        }
        for (i = 0; i < len; i++)
        {
            rem = res % 10;
            res = res / 10;
            adc_val_string[len - (i + 1)] = rem + '0';
        }
        adc_val_string[len] = '\0';
    printstring(adc_val_string);

}




void UART0_Transmitter(unsigned char data)
{
    while ((UART0_FR_R & (1 << 5)) != 0)
        ;              /* wait until Tx buffer not full */
    UART0_DR_R = data; /* before giving it another byte */
}

void printstring(char *str)
{
    while (*str)
    {
        UART0_Transmitter(*(str++));
    }
}

void delay_ms(int n)
{
    int i, j;

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < 3180; j++)
            ;
    }
}


// Calculate Distance from Echo Pulse Width
unsigned int calculate_distance(void) {
    unsigned int time_diff;
    if (time_value1 < time_value2) {
        time_diff = time_value1 + (608000) - time_value2;
    } else {
        time_diff = time_value1 - time_value2;
    }
    time_diff = time_diff / (16 * 2);
    distance = 34000 * time_diff / 1000000; // Convert time to distance in cm
    return distance;
}


void WTIMER3B_Handler(void)
{
    if((WTIMER3_MIS_R & 0x0400) && (GPIO_PORTD_DATA_R & 0x08))
    {
        time_value1 = WTIMER3_TBR_R;
    }
    if((WTIMER3_MIS_R & 0x0400) && !(GPIO_PORTD_DATA_R & 0x08))
    {
        time_value2 = (WTIMER3_TBR_R);
    }
    WTIMER3_ICR_R = 0x0400 ;
}
