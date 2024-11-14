
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/pwm.h"
#include "ultra.h"
#include "ir.h"


#define PWM_FREQUENCY 55


// initialization functions
void EnableInterrupts(void);
void UART0_setup(void);
void GPIO_setup(void);
void systick_setup(void);
// user defined functions
void UART0_Transmit(char data);
void UART0_command_check(void);
void set_7_seg(int,int);
void disp_stopwatch_time();
void led_action();
void UART0_Transmitter(unsigned char data);
void printstring(char *str);
void uart_print_num(int res);
unsigned int calculate_distance(void) ;
void WTIMER3B_Handler(void);
void init_pwm(void);
void delay_ms(int n);
void PWM_init_new(void);
void mapPWM(int input) ;
void pid(void);
// hex values of digits for 7 segment display
int disp_digits[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
// variable for keeping track of time in RUN state / stopwatch time
unsigned int disp_timer = 0;
unsigned int led_timer = 0;
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
// during which if an interrupt is detected, it is ignored (assuming humans don't press switches that fast so it is wrong value)
// cool_off_time is updated in SysTick_Handler() and reset along with sw_interrupt_flag after 200ms
int sw1_interrupt_flag = 0x00;
int sw2_interrupt_flag = 0x00;
int sw1_cool_off_time = 0;
int sw2_cool_off_time = 0;
int global_duty_cycle = 30;
int global_load_value = 0;

volatile long long int x = 0;
volatile long long int delay_val = 3;

volatile uint32_t ui32Load = 0;;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui8Adjust;
// array to store command characters, top specifying total characters received. (stack implementation - only push allowed)
char command_buffer[6];
int top = 0;
// status flags used in uart
// uart_echo_flag is set each time a new character is received, and reset after echo
int uart_echo_flag = 0x00;
// uart_cmd_select value indicates the command to be executed
// The lower 4 bits correspond to 4 commands
// bits: 3 2 1 0 <=> command : 4 3 2 1, whichever bit is 1 that command will be executed.
// Each time new character is received, it is compared with command arrays
// Only those whose command[0:top-1] == received_char will be 1
// Initially all commands are equally likely so value (0000 1111)
int uart_cmd_select = 0x0F;
// uart_backspace_flag is set when 'backspace' is detected, top is decremented and uart_cmd_select is computed again
int uart_backspace_flag = 0x00;
// flag to indicate buffer overflow so that user can be informed
int uart_buffer_overflow = 0x00;
// flag to indicate status whether command was valid or not.
// Set each time 'enter key' is pressed
// 0x0F : invalid command
// 0xFF : command executed
int uart_state_update = 0x00;

// array storing valid commands
char command1[6] = {'s','t','a','r','t','0'};
char command2[6] = {'s','t','o','p','0','0'};
char command3[6] = {'h','o','l','d','0','0'};
char command4[6] = {'r','e','s','u','m','e'};
// array storing standard dispaly messages
char msg1[15] = {'b','u','f','f','e','r',' ','o','v','e','r','f','l','o','w'};
char msg2[16] = {'c','o','m','m','a','n','d',' ','e','x','e','c','u','t','e','d'};
char msg3[15] = {'i','n','v','a','l','i','d',' ','c','o','m','m','a','n','d'};
char msg4[9] = {'b','a','c','k','s','p','a','c','e'};

int state;
int update_flag;
int timer_200_msec ;
char adc_val_string[10] = {0};
unsigned int distance;
unsigned int time_value1 ;
unsigned int time_value2;
int obstacle10;
unsigned long increment ;
unsigned long pwmNow ;
int stop;
int running;
int kp,kd,ki;
int data0, data1, data2, data3, data4, data5;
int main(void)
{
    kp=0;
    kd=0;
    ki=0;
    state = 0;
    update_flag = 0;
    timer_200_msec = 0;
    distance=0;
    time_value1 = 0;
    time_value2 = 1;
    obstacle10=0;
    pwmNow = 1;
    stop =1;
    running =200;
    ui8Adjust = 18500;
    EnableInterrupts();
    HC_SR04_setup();
    systick_setup();
    GPIO_setup();
    UART0_setup();
    PWM_init_new();
    init_ir_sensor();
    printstring("Hello\n\r");
//
       delay_ms(1);


    while(1)
    {
        PWM1_0_CMPA_R=ui8Adjust;
//        mapPWM((int)(ui8Adjust));
        led_action();
        pid();
        //UART0_command_check();
        if (update_flag == 1)// 200msec parameter update
            {

                distance = calculate_distance();
                update_flag = 0;                    // reset parameter update bit
                send_trigger_pulse();
                printstring("the distance is  ");
                uart_print_num(distance);
                printstring("\n\r");
    //                delay_ms(200);
                if(state==0){
                    printstring("car is ready to start\n\r");
                    pwmNow = stop;
                }
                if(state==1){
                    printstring("car is running\n\r");
                    pwmNow = running;
                }
                if(state==2){
                    printstring("obstacles ahead\n\r");
                    pwmNow = stop;
                }

                /********** If statements to make decision where is should move  ***********/
                                if(data0==0x00 && data1==0x00){
                                    printstring(" out of the track \n\r");
                                }
                                else if(data0==0x01 && data1==0x02)
                                    printstring("go striaght \n\r");
                                else if(data0==0x01 && data1==0x00)
                                    printstring("should move left \n\r");
                                 else if(data0==0x00 && data1==0x02)
                                     printstring("should move right \n\r");


            }
        if(distance<10 && state==1){
            obstacle10=1;
            state=2;
        }
        else if(distance>=10 && state==2)
            state=1;
        //********** stores the output of sensors by ANDing as shown below ***********//
                    //********** for PORT PF1: we have to take AND of the second bit of PORT F i-e: " 0x00010 "   **********//
                    //********** for PORT PF2: we have to take AND of the third bit of PORT F i-e: " 0x00100 "    **********//

                        data0 = GPIO_PORTB_DATA_R & 0x01;   // stores Input of PB0
                        data1 = GPIO_PORTB_DATA_R & 0x02;   // stores Input of PB1
       }

    return 0;
}


void pid(){
    int time=1;
//    int time=(int)(current_timer_register)
    //timer reset

    uint16_t sensor;
    static int error,i_error;
    int prevError;
    sensor=(GPIO_PORTB_DATA_R & 0x3E)>>1;
    prevError=error;
    switch(sensor){
    case 0x01:{
        error=-1000;
        break;
    }
    case 0x03:{
            error=-750;
            break;
        }
    case 0x02:{
                error=-500;
                break;
            }
    case 0x06:{
                error=-250;
                break;
            }
    case 0x04:{
                error=0;
                break;
            }
    case 0x0C:{
                error=250;
                break;
            }
    case 0x08:{
                error=500;
                break;
            }
    case 0x18:{
                error=750;
                break;
            }
    case 0x10:{
                error=1000;
                break;
            }
    default:{
        error=0;
    }

    }
    i_error+=error;
    if(i_error>=100){
        i_error=100;
    }
    else if(i_error<=-100)
        i_error=-100;
    int output=(error*kp)+(int)(((float)error-(float)prevError)*(float)kd/(float)time)+(i_error*ki*time);



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

void inline EnableInterrupts(void)
{
    // Global interrupts enable
    __asm  ("CPSIE  I\n");      // Change processor state-enable interrupts
}

void UART0_setup()
{
    SYSCTL_RCGCUART_R |= 0x01; // Enable UART0
    SYSCTL_RCGCGPIO_R |= 0x01; // Enable corresponding GPIO port (PORTA)
    // Set GPIO AFSEL bits
    // xxx... PA7 PA6 PA5 PA4 PA3 PA2 PA1 PA0
    //                                Tx  Rx
    GPIO_PORTA_AFSEL_R = 0x03;
    // PCTL is 32 bit register, 4 bits are used to select alternate functions of each GPIO pin
    GPIO_PORTA_PCTL_R = 0x00000011;
    GPIO_PORTA_DEN_R |= 0x03; // GPIO digital enable for PA0, PA1
    // clk = 16 MHz
    // Desired baud rate = clk / (16 * clk_div)
    // UARTIBRD and UARTFBRD
    // If Desired baud rate = XXX.YYY
    // then UARTIBRD = XXX and UARTFBRD = (0.YYY * 64) + 0.5
    // baud_rate = 115200
    UART0_IBRD_R = 8;
    UART0_FBRD_R = 44;
    // 0x60 for 8 data bits, 1 stop, 1 start
    UART0_LCRH_R = 0x60;
    UART0_CC_R = 0x0;
    UART0_CTL_R = 0x0301;
    GPIO_PORTA_AMSEL_R = 0;

    // Interrupt setup for UART0
//    UART0_IM_R |= 0x10;       // enable interrupt mask for receive
//    UART0_ICR_R = (0x010);    // clear interrupt if any
//    NVIC_PRI1_R |= 0x00006000;// priority = 3
//    NVIC_EN0_R |= 0x00000020; // enable interrupt for UART0 (IRQ number = 5)
}

void GPIO_setup()
{
    SYSCTL_RCGCGPIO_R |= 0x00000023;     // enable clock to GPIOA, GPIOB and GPIOF at clock gating control register
    GPIO_PORTF_LOCK_R = 0x4C4F434B;   // unlock commit register GPIOCR
    GPIO_PORTF_CR_R = 0x01;           // make PORTF0 configurable
    // Enable the GPIO pins
    // For PORTF the LED (PF3, 2 1) as output
    // For PORTB, all pins are used to set 7 segment display
    // For PORTA, pins 7 to 4 are used for selecting one of the four 7 segment display
    GPIO_PORTA_DIR_R |= 0xF0;
    GPIO_PORTB_DIR_R |= 0xFF;
    // X X X SW1 G B R SW2 -> bits in PORTF
    GPIO_PORTF_DIR_R |= 0x0E;
    // enable the GPIO pins for digital function
    GPIO_PORTA_DEN_R |= 0xF0;
    GPIO_PORTB_DEN_R |= 0xFF;
    GPIO_PORTF_DEN_R |= 0x1F;
    GPIO_PORTF_PUR_R = 0x11;           // enable pull up for SW1 and SW2

    // interrupt setup for sw1 and sw2
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
    //NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00A00000;
    NVIC_PRI7_R |= 0x00A00000;       // priority = 5
    NVIC_EN0_R |= 0x40000000;        // NVIC enabled for PORTF (IRQ number = 30)
}

void systick_setup()
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


//void PWM_init_new()
//{
///* Enable Peripheral Clocks */
//
//    SYSCTL_RCGCPWM_R |= 2;          /* enable clock to PWM1 */
//    SYSCTL_RCGCGPIO_R |= 0x18;     /* enable clock to PORTE & D */
//
////    SYSCTL_RCC_R &= ~0x00100000;   /* no pre-divide for PWM clock */
//    SYSCTL_RCC_R |= (1<<20);  /* PWM clock divisor set to 16. System clock gets divided by 16*/
//    SYSCTL_RCC_R |= 0x00060000;  /* PWM clock divisor set to 16. System clock gets divided by 16*/
//
//    /* Enable port PE4 for PWM1 M1PWM2 */
//
//    GPIO_PORTE_AFSEL_R |= 0X10;              /* PE4 uses alternate function */
//    GPIO_PORTE_PCTL_R &= ~0x00F00000;   /* make PE4 PWM output pin */
//    GPIO_PORTE_PCTL_R |= 0x00050000;      /* Write 5 to the PCTL register of PE4. Refer PWM chapter in datasheet */
//    GPIO_PORTE_DEN_R |= 16;                /*pin digital */
//
//    PWM1_1_CTL_R &= ~(1<<0);      /* stop counter. Disabled the PWM block */
//    PWM1_1_CTL_R &= ~(1<<1);        /* Count down mode and counter wraps back. Write 0 to bit 8 and 9 to immediately load value once count reaches 0 */
//    PWM1_1_GENA_R = 0x0000008C;     /* M1PWM2 output set when reload. Drive pwmA high when loaded and invert when matches compare A reg. This gives left aligned square wave */
//
////    PWM1_1_CTL_R |= (1<<1);  //up-down mode
//
//    /* clear when match PWMCMPA */
//    PWM1_1_LOAD_R = 500-1; /* set load value for 10kHz (16MHz/1600) */
//    PWM1_1_CMPA_R = 100; /* set duty cycle to min */
//
////    PWM1_1_DBCTL_R = 1;   //Enable dead band
////    PWM1_1_DBRISE_R = 100000;   // Rise edge delay for pwmA' signal
////    PWM1_1_DBFALL_R = 100000;   // Fall edge delay for pwmB' signal
////
////    PWM1_1_GENA_R = 0x000000B0;
////    PWM1_1_GENB_R = 0x0000000;
//
//
//    PWM1_1_CTL_R = 1; /* start timer */
//    PWM1_ENABLE_R = 0x0C; /* start PWM1 ch2 */
//
//        GPIO_PORTD_AFSEL_R |= 0X01;              /* PE4 uses alternate function */
//       GPIO_PORTD_PCTL_R &= ~0x00000001;   /* make PE4 PWM output pin */
//       GPIO_PORTD_PCTL_R |= 0x00000005;      /* Write 5 to the PCTL register of PE4. Refer PWM chapter in datasheet */
//       GPIO_PORTD_DEN_R |= 1;                /*pin digital */
//
//       PWM1_0_CTL_R &= ~(1<<0);      /* stop counter. Disabled the PWM block */
//          PWM1_0_CTL_R &= ~(1<<1);        /* Count down mode and counter wraps back. Write 0 to bit 8 and 9 to immediately load value once count reaches 0 */
//          PWM1_0_GENA_R = 0x0000008C;     /* M1PWM2 output set when reload. Drive pwmA high when loaded and invert when matches compare A reg. This gives left aligned square wave */
//
//          PWM1_0_LOAD_R = 20000-1; /* set load value for 3.2kHz (16MHz/16000) */
//          PWM1_0_CMPA_R = 2000; /* set duty cycle to min */
//
//          PWM1_0_CTL_R = 1; /* start timer */
//          PWM1_ENABLE_R = 0x0F; /* start PWM1 ch2 */
//
//}

void PWM_init_new()
{
/* Enable Peripheral Clocks */

    SYSCTL_RCGCPWM_R |= 2;          /* enable clock to PWM1 */
    SYSCTL_RCGCGPIO_R |= 0x18;     /* enable clock to PORTE & D */

//    SYSCTL_RCC_R &= ~0x00100000;   /* no pre-divide for PWM clock */
    SYSCTL_RCC_R |= (1<<20);  /* PWM clock divisor set to 64. System clock gets divided by 64*/
//    SYSCTL_RCC_R |= 0x00060000;  /* PWM clock divisor set to 64. System clock gets divided by 64 */


    SYSCTL_RCC_R &= ~0x000E0000;  // Clear bits 17:19
    SYSCTL_RCC_R |= 0x00060000;   // Set bits 17:19 to 0x3 for divide by 16

    /* Enable port PE4 for PWM1 M1PWM2 */

    GPIO_PORTE_AFSEL_R |= 0X10;              /* PE4 uses alternate function */
    GPIO_PORTE_PCTL_R &= ~0x00F00000;   /* make PE4 PWM output pin */
    GPIO_PORTE_PCTL_R |= 0x00050000;      /* Write 5 to the PCTL register of PE4. Refer PWM chapter in datasheet */
    GPIO_PORTE_DEN_R |= 16;                /*pin digital */

    PWM1_1_CTL_R &= ~(1<<0);      /* stop counter. Disabled the PWM block */
    PWM1_1_CTL_R &= ~(1<<1);        /* Count down mode and counter wraps back. Write 0 to bit 8 and 9 to immediately load value once count reaches 0 */
    PWM1_1_GENA_R = 0x0000008C;     /* M1PWM2 output set when reload. Drive pwmA high when loaded and invert when matches compare A reg. This gives left aligned square wave */

//    PWM1_1_CTL_R |= (1<<1);  //up-down mode

    /* clear when match PWMCMPA */
    PWM1_1_LOAD_R = 1000-1; /* set load value for 10kHz (16MHz/1600) */
    PWM1_1_CMPA_R=100-10; /* set duty cycle to min */

//    PWM1_1_DBCTL_R = 1;   //Enable dead band
//    PWM1_1_DBRISE_R = 100000;   // Rise edge delay for pwmA' signal
//    PWM1_1_DBFALL_R = 100000;   // Fall edge delay for pwmB' signal
//
//    PWM1_1_GENA_R = 0x000000B0;
//    PWM1_1_GENB_R = 0x0000000;


    PWM1_1_CTL_R = 1; /* start timer */
    PWM1_ENABLE_R = 0x0C; /* start PWM1 ch2 */

        GPIO_PORTD_AFSEL_R |= 0X01;              /* PE4 uses alternate function */
       GPIO_PORTD_PCTL_R &= ~0x00000001;   /* make PE4 PWM output pin */
       GPIO_PORTD_PCTL_R |= 0x00000005;      /* Write 5 to the PCTL register of PE4. Refer PWM chapter in datasheet */
       GPIO_PORTD_DEN_R |= 1;                /*pin digital */

       PWM1_0_CTL_R &= ~(1<<0);      /* stop counter. Disabled the PWM block */
          PWM1_0_CTL_R &= ~(1<<1);        /* Count down mode and counter wraps back. Write 0 to bit 8 and 9 to immediately load value once count reaches 0 */
          PWM1_0_GENA_R = 0x0000008C;     /* M1PWM2 output set when reload. Drive pwmA high when loaded and invert when matches compare A reg. This gives left aligned square wave */

          PWM1_0_LOAD_R = 20000-1; /* set load value for 3.2kHz (16MHz/16000) */
          PWM1_0_CMPA_R = 1500; /* set duty cycle to min */
//          PWM1_0_CMPA_R = 4605; /* set duty cycle to min */

          PWM1_0_CTL_R = 1; /* start timer */
          PWM1_ENABLE_R = 0x0F; /* start PWM1 ch2 */

}

void mapPWM(int input) {
    int input_min = 0;
    int input_max = 1000;
    int output_min = 19000;
    int output_max = 18000;

    // Calculate the mapped value
     uint32_t mappedValue = (uint32_t)(18000+input);

    // Cap the output value within the range [18000, 19000]
    if (mappedValue < 18000) {
        mappedValue = 18000;
    } else if (mappedValue > 19000) {
        mappedValue = 19000;
    }
    PWM1_0_CMPA_R=mappedValue;
}
void UART0_Transmit(char data)
{
    while((UART0_FR_R & (1<<5)) != 0)
    {
        // wait until Tx buffer full
    }
    UART0_DR_R = data; // send the byte
}

void UART0_command_check()
{
    // if new character received then update uart_cmd_select and echo back the character
    if(uart_echo_flag == 0xFF)
    {
        // command[top-1] : new received character in command_buffer
        // command_buffer[top-1] : corresponding character of standard commands
        // they are compared for each command, the result is shifted to respective bit position
        // and ANDED with previous value
        uart_cmd_select &= ( ((command1[top-1]==command_buffer[top-1])<<0) | ((command2[top-1]==command_buffer[top-1])<<1) | ((command3[top-1]==command_buffer[top-1])<<2) | ((command4[top-1]==command_buffer[top-1])<<3) );
        uart_echo_flag = 0x00;                      // reset uart_echo_flag
        UART0_Transmit(command_buffer[top-1]);      // echo the received character
    }
    // if buffer_overflow set transmit msg1 to user
    if(uart_buffer_overflow == 0xFF)
    {
        UART0_Transmit('\n');
        for(int i = 0; i < 6; i++)
        {
            UART0_Transmit('\b');
        }
        for(int i = 0; i < 15; i++)
        {
            UART0_Transmit(msg1[i]);
        }
        for(int i = 0; i < 15; i++)
        {
            UART0_Transmit('\b');
        }
        UART0_Transmit('\n');
        uart_buffer_overflow = 0x00;                // reset buffer_overflow
    }
    // if uart_state_update = 'valid', transmit msg2 to user
    if(uart_state_update == 0xFF)
    {
        UART0_Transmit('\n');
        for(int i = 0; i < 6; i++)
        {
            UART0_Transmit('\b');
        }
        for(int i = 0; i < 16; i++)
        {
            UART0_Transmit(msg2[i]);
        }
        for(int i = 0; i < 16; i++)
        {
            UART0_Transmit('\b');
        }
        UART0_Transmit('\n');
        uart_state_update = 0x00;                   // reset uart_state_update
    }
    // if uart_state_update is invalid, transmit msg3 to user
    else if(uart_state_update == 0x0F)
    {
        UART0_Transmit('\n');
        for(int i = 0; i < 6; i++)
        {
            UART0_Transmit('\b');
        }
        for(int i = 0; i < 15; i++)
        {
            UART0_Transmit(msg3[i]);
        }
        for(int i = 0; i < 15; i++)
        {
            UART0_Transmit('\b');
        }
        UART0_Transmit('\n');
        uart_state_update = 0x00;                   // reset uart_state_update
    }
    if(uart_backspace_flag == 0xFF)
    {
        UART0_Transmit('\n');
        for(int i = 0; i < top+1; i++)
        {
            UART0_Transmit('\b');
        }
        for(int i = 0; i < 9; i++)
        {
            UART0_Transmit(msg4[i]);
        }
        UART0_Transmit('\n');
        for(int i = 0; i < 9; i++)
        {
            UART0_Transmit('\b');
        }
        for(int i = 0; i < top; i++)
        {
            uart_cmd_select &= ( ((command1[i]==command_buffer[i])<<0) | ((command2[i]==command_buffer[i])<<1) | ((command3[i]==command_buffer[i])<<2) | ((command4[i]==command_buffer[i])<<3) );
            UART0_Transmit(command_buffer[i]);
        }
        uart_backspace_flag = 0x00;
    }
}

void led_action()
{
    // if less than 200 ms, LED is ON else it is OFF when in blink state
    if(led_timer < 200)
    {
        int color;
        if((state_sw1 == 0xF0) && (state_sw2 == 0xF0))
        {
            color = 0x08;   // green
        }
        else if((state_sw1 == 0xF0) && (state_sw2 == 0x0F))
        {
            color = 0x04;   // blue
        }
        else
        {
            color = 0x02;   // red
        }
        GPIO_PORTF_DATA_R = color;
    }
    else
    {   // LEDs are OFF if not Stop state
        if(state_sw1 != 0x0F)
        {
            GPIO_PORTF_DATA_R = 0x00;
        }
    }
}
//void SysTick_Handler(void)
//{
//    timer_200_msec++;
//        if(timer_200_msec == 200)
//        {
//            update_flag = 1;
//            timer_200_msec = timer_200_msec - 200;
//        }
//}
void UART0_Handler(void)
{
    UART0_ICR_R = (0x010);                  // clear the interrupt
    char c = UART0_DR_R;                    // read the data from UART0_DR register
    // if received character is 'enter key'
    if(c == '\r')
    {
        uart_state_update = 0xFF;           // assume command is valid and check if it matches standard commands
        // if command is 'start' and state_sw1 = 'stop'
        if((uart_cmd_select == 0x01) && (state_sw1 == 0x0F))
        {
            state_sw1 = 0xF0;               // set state_sw1 = 'run'
        }
        // if command is 'stop', reset all states and timer
        else if(uart_cmd_select == 0x02)
        {
            state_sw1 = 0x0F;
            state_sw2 = 0xF0;
            disp_timer = 0;
        }
        // if command is 'hold' and state_sw1 = 'run'
        else if((uart_cmd_select == 0x04) && (state_sw1 == 0xF0))
        {
            state_sw2 = 0x0F;               // set state_sw2 = 'pause'
        }
        // if command is 'resume'
        else if(uart_cmd_select == 0x08)
        {
            state_sw2 = 0xF0;
        }
        else
        {
            uart_state_update = 0x0F;       // set uart_state_update = 'invalid'
        }
        uart_cmd_select = 0x0F;             // reset uart_cmd_select
        top = 0;                            // reset received char count
    }
    // if received character is 'backspace key'
    else if(c == '\b')
    {
        uart_backspace_flag = 0xFF;         // set uart_backspace_flag
        uart_cmd_select = 0x0F;             // reset uart_cmd_select
        top--;                              // decrement top of the command_buffer
    }
    // new character received, add to command_buffer and set corresponding flag
    else if(top < 6)
    {
        command_buffer[top++] = c;
        uart_echo_flag = 0xFF;
    }
    // command_buffer overflow, top is reset again to 0, set uart_buffer_overflow flag
    // inform the user through uart
    else
    {
        top = 0;
    //uart_cmd_select = 0x1F;             // reset uart_cmd_select
        uart_buffer_overflow = 0xFF;
    }
}

void SysTick_Handler(void)
{
    // if it is in RUN mode in SW1 and SW2 both, update disp_timer
    if(state_sw1 == 0xF0 && state_sw2 == 0xF0)
    {
        disp_timer = disp_timer + 1;
    }
    led_timer = led_timer + 1;
    if(led_timer >= 400)
    {
        led_timer = led_timer - 400;
    }
    // sw_interrupt_flag will reset only after 200ns
    if(sw1_interrupt_flag == 0xFF)
    {
        sw1_cool_off_time++;
        if(sw1_cool_off_time == 200)
        {
            sw1_interrupt_flag = 0x00;
        }
    }
//    if(sw2_interrupt_flag == 0xFF)
//    {
//        sw2_cool_off_time++;
//        if(sw2_cool_off_time == 200)
//        {
//            sw2_interrupt_flag = 0x00;
//        }
//    }
    // invert the flag
    flag_1ms ^= 0xFF;
    timer_200_msec++;
    if(timer_200_msec == 1000)
    {
        update_flag = 1;
        timer_200_msec = timer_200_msec - 1000;
    }
}

void GPIO_PORTF_Handler(void)
{
    // interrupt detected in switch 1 and interrupt flag is reset then update state_sw1
    // this is to avoid double clicks induced by mistake that erroneously changes the state
    if(GPIO_PORTF_MIS_R == 0x10 && sw1_interrupt_flag == 0x00)
    {
        state_sw1 ^= 0xFF;
        sw1_interrupt_flag = 0xFF;
        sw1_cool_off_time = 0;
        if(state_sw1 == 0x0F)
        {
            state_sw2 = 0xF0;
            disp_timer = 0;
        }
        if(state==0)
            state=1;
        else
            state=0;
    }
    // interrupt detected in switch 2 and interrupt flag is reset then update state_sw2 only if state_sw1 == RUN
    if(GPIO_PORTF_MIS_R == 0x01 && sw2_interrupt_flag == 0x00)
    {
        if(state_sw1 == 0xF0)
        {
            state_sw2 ^= 0xFF;
            sw2_interrupt_flag = 0xFF;
            sw2_cool_off_time = 0;
        }
    }
    // clear the interrupt flags
    GPIO_PORTF_ICR_R = 0x11;
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

void init_pwm(void){
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC |   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

      //Configure PWM Clock to match system
      SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

      // Enable the peripherals used by this program.
//       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the LED pins
       SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
       //Configure PB4,PB5,PB6,PB7 Pins as PWM
   //    GPIOPinConfigure(GPIO_PF1_M1PWM5);
   //    GPIOPinConfigure(GPIO_PF2_M1PWM6);
   //    GPIOPinConfigure(GPIO_PF3_M1PWM7);
       GPIOPinConfigure(GPIO_PB4_M0PWM2);//12.5%duty cycle
       GPIOPinConfigure(GPIO_PB5_M0PWM3);//6.25%dutycycle
//       GPIOPinConfigure(GPIO_PB6_M0PWM0);//50% duty cycle
//       GPIOPinConfigure(GPIO_PB7_M0PWM1);//25%duty cycle

       GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);// | GPIO_PIN_6 |GPIO_PIN_7
       // Configure PB6 and PB7 as digital outputs
       GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);  // Set PB6 and PB7 as outputs

       //Configure PWM Options
       //PWM_GEN_0 Covers M0PWM0 and M0PWM1
       //PWM_GEN_1 Covers M0PWM2 and M1PWM3 See page 207 4/11/13 DriverLib doc
       PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
       PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

       //Set the Period (expressed in clock ticks)-period is 320 clock ticks-50KHz freq
//       PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 320);
       PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 320);

       //Set PWM duty-50% (Period /2)
//       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,160);
//       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,80);
       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,40);
       PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,20);

       // Enable the PWM generator at pd0 for the servo motor
//       PWMGenEnable(PWM0_BASE, PWM_GEN_0);
       PWMGenEnable(PWM0_BASE, PWM_GEN_1);

       PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
       SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

       SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

       GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
       GPIOPinConfigure(GPIO_PD0_M1PWM0);

       ui32PWMClock = SysCtlClockGet() / 64;
       ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
       PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
       PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);

       PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
       PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
       PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}
void PWM_SetDutyCycle(uint16_t dutyCycle) {
    if (dutyCycle > 100) dutyCycle = 100; // Limit duty cycle to 100%

    // Calculate comparator value for desired duty cycle
    uint32_t cmpValue = (80000 * (100 - dutyCycle)) / 100 - 1;
    PWM0_0_CMPA_R = cmpValue;
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
void SysTick_Handler_servo(void)  // triggers for every 100uSec
{
    if (x < delay_val + 10)
        x++;
}
