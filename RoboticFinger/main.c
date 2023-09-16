#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define SERVO_PERIOD 20000 * 1.048
#define BUTTON0 BIT1
#define BUTTON1 BIT1
#define BUTTON2 BIT0
#define LED0 BIT0
#define LED1 BIT7
#define LED2 BIT1
#define ADC0 BIT0
#define ADC1 BIT1
#define STR BIT7;
#define A0 BIT1;
#define A1 BIT2;
#define D0 BIT2;
#define D1 BIT3;
#define D2 BIT4;
#define D3 BIT5;
#define DEFAULT_ADC0_MIN 1854
#define DEFAULT_ADC0_MAX 2398
#define DEFAULT_ADC1_THRESHOLD 3500
#define ADC0_DISPLAY_MODE 0
#define ADC1_DISPLAY_MODE 1
#define DEFAULT_DISPLAY_MODE ADC0_DISPLAY_MODE
#define MAX_ADC_RATE 100;

int *BAUD_REGISTER_B0 = (int *) 0x1900;         // Minimum rotation ADC memory address
int *BAUD_REGISTER_C0 = (int *) 0x1880;         // Maximum rotation ADC memory address
int *BAUD_REGISTER_D0 = (int *) 0x1800;         // Pressure threshold ADC memory address



uint16_t ADC0_MIN = DEFAULT_ADC0_MIN;
uint16_t ADC0_MAX = DEFAULT_ADC0_MAX;
uint16_t ADC1_THRESHOLD = DEFAULT_ADC1_THRESHOLD;
float DUTY_CYCLE_MAX = 0.097;
float DUTY_CYCLE_MIN = 0.040;
uint16_t DISPLAY_MODE = DEFAULT_DISPLAY_MODE;




void setup_PWM(void) {
    TA0CCR0 = (uint16_t) SERVO_PERIOD;          // Set TimerA0 to have a reset period of servo motor specified PWM period
    TA0CCR1 = 0;                                // Initial value for pulse width to servo motor
    TA0CCR2 = 0;                                // Initial value for pulse width to white LED
    TA0CCTL1 = OUTMOD_7;                        // TA0CCR1 PWM reset/set HIGH voltage when below TA0CCR1 value, LOW voltage when past
    TA0CCTL2 = OUTMOD_7;                        // TA0CCR2 PWM set HIGH voltage when below TA0CCR2 value, LOW voltage when past
    TA0CTL = TASSEL_2 + MC_1 + ID_0;            // TimerA0 control set to SMCLK, and mode count up to TA0CCR0
    TA1CCR0 = MAX_ADC_RATE;                     // Initial value for period of ADC reading timer trigger
    TA1CTL = TASSEL_2 + MC_1 + ID_0 + TAIE;     // TimerA1 control set to SMCLK, and mode count up to TA0CCR0, enable interrupt
    TA1CCTL0 |= CCIE;                           // Enable interrupt for Timer1A
}

void setup_display(void) {
    P4DIR |= BIT1 + BIT2;
    P3DIR |= BIT2 + BIT3 + BIT4 + BIT5;
    P2DIR |= BIT7;
}


void enable_PWM(void) {
    P1DIR |= BIT2 + BIT3;
    P1SEL |= BIT2 + BIT3;
}

void disable_PWM(void) {
    P1DIR &= !BIT2;
    P1SEL &= !BIT2;
}

void setup_ADC(void) {                              // Multiple ADC implementation, from example on course website
    ADC12CTL0 = ADC12SHT02 + ADC12ON + ADC12MSC;    // Sampling time, ADC12 on, automatic multiple conversions
    ADC12CTL1 = ADC12SHP | ADC12CONSEQ_1 | ADC12CSTARTADD_0;    // Sampling timer, multi-channel, starting memory address
    ADC12MCTL0 = ADC12INCH_0;                                   //selects A0 to be stored in memory ADC12MEM0
    ADC12MCTL1 = ADC12INCH_1 + ADC12EOS;                        //selects A1 to be stored in memory ADC12MEM1 and this memory to be the last of sequence
    ADC12CTL0 |= ADC12ENC;                          // ADC enable conversions
    P6SEL |= ADC0 + ADC1;                           // Set ADC input pins

    ADC0_MIN = *BAUD_REGISTER_B0;                   // Load minimum rotation ADC from corresponding flash memory address
    ADC0_MAX = *BAUD_REGISTER_C0;                   // Load maximum rotation ADC from corresponding flash memory address
    ADC1_THRESHOLD = *BAUD_REGISTER_D0;             // Pressure threshold ADC from corresponding flash memory address
}

void setup_LED(void) {
    P1DIR |= LED0;
    P4DIR |= LED1;
    P8DIR |= LED2;
    P1OUT &= ~LED0;
    P4OUT &= ~LED1;
    P8OUT &= ~LED2;
}
void setup_button(void) {
    P1DIR &= ~BUTTON0;
    P1OUT |= BUTTON0;
    P1REN |= BUTTON0;

    P2DIR &= ~BUTTON1;
    P2OUT |= BUTTON1;
    P2REN |= BUTTON1;

    P4DIR &= ~BUTTON2;
    P4OUT |= BUTTON2;
    P4REN |= BUTTON2;
}

void setup_WTD() {
    WDTCTL = WDTPW|WDTHOLD;
}

void setup_LPM() {
    _BIS_SR (LPM0_bits + GIE);
}

void main(void) {
    setup_WTD();
    setup_LED();
    setup_PWM();
    enable_PWM();
    setup_ADC();
    setup_button();
    setup_display();

    setup_LPM();


}

void erase_segment(int *address) {
    int *flash_ptr = address;                 // Temporary reference to memory address
    FCTL3 = FWKEY;                            // Unlock FLASH control register
    FCTL1 = FWKEY + ERASE;                    // Select segment erase
    *flash_ptr = 0;                           // Dummy erase byte
    FCTL3 = FWKEY + LOCK;                     // Lock flash control register

}

void write_flash(int* address, int data) {
    int *ptr = address;                       // Temporary reference to memory address
    FCTL3 = FWKEY;                            // Unlock flash control register
    FCTL1 = FWKEY + WRT;                      // Enable writing of segment
    *ptr = data;                              // Dereference pointer and set its data
    FCTL1 = FWKEY;                            // Disable writing of segment
    FCTL3 = FWKEY + LOCK;                     // Lock flash control register
}

void update_display(int ADC) {                // Analogously to lab notes set 1, with strobe toggle to trigger display update and digit selection, 4 digits total
    P2OUT |= STR;                             // Strobe set to HIGH
    P4OUT &= ~A0;                             // Rightmost digit
    P4OUT &= ~A1;                             // which has (A1, A0) = (0, 0)
    P3OUT = ((ADC % 10) / 1) << 2;            // Get the rightmost ADC digit, and set P3.2, P3.3, P3.4, and P3.5 appropriately
    P2OUT &= ~STR;                            // Strobe set to LOW

    P2OUT |= STR;
    P4OUT |= A0;                              // Second rightmost digit
    P4OUT &= ~A1;                             // which has (A1, A0) = (0, 1)
    P3OUT = ((ADC % 100) / 10) << 2;          // Get the second rightmost ADC digit, and set P3.2, P3.3, P3.4, and P3.5 appropriately
    P2OUT &= ~STR;

    P2OUT |= STR;
    P4OUT &= ~A0;                             // Second leftmost digit
    P4OUT |= A1;                              // which has (A1, A0) = (1, 0)
    P3OUT = ((ADC % 1000) / 100) << 2;        // Get the second leftmost ADC digit, and set P3.2, P3.3, P3.4, and P3.5 appropriately
    P2OUT &= ~STR;

    P2OUT |= STR;
    P4OUT |= A0;                              // Leftmost digit
    P4OUT |= A1;                              // which has (A1, A0) = 11
    P3OUT = ((ADC % 10000) / 1000) << 2;      // Get the leftmost ADC digit, and set P3.2, P3.3, P3.4, and P3.5 appropriately
    P2OUT &= ~STR;
}


void calibrate_min(void) {
    ADC0_MIN = ADC12MEM0;                      // Set minimum ADC setting for rotation
    erase_segment(BAUD_REGISTER_B0);           // Erase corresponding memory address data
    write_flash(BAUD_REGISTER_B0, ADC0_MIN);   // Save new minimum ADC setting for rotation
}


void calibrate_max(void) {
    ADC0_MAX = ADC12MEM0;                      // Set maximum ADC setting for rotation
    erase_segment(BAUD_REGISTER_C0);           // Erase corresponding memory address data
    write_flash(BAUD_REGISTER_C0, ADC0_MAX);   // Save new maximum ADC setting for rotation
}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void TA1CCR0_ISR(void) {        // Interrupt for Timer1A0, used as a "timed loop" for
                                            // better synchronization pulse transmission to servo motor

    if (ADC12CTL1 & ADC12BUSY) {            // If bit ADC12BUSY in register ADC12CTL1 is high, simply return
        return;
    }

    if (ADC12MEM1 >= ADC1_THRESHOLD)  {     // See if the robotic finger tip being pressed too hard to move
        P4OUT |= LED1;                      // Turn on P4.7 LED
    } else {
        float duty_cycle;
        if (ADC12MEM0 < ADC0_MIN) {         // Ensure that motor doesn't rotate beyond 0°
            duty_cycle = DUTY_CYCLE_MIN;
        } else if (ADC12MEM0 > ADC0_MAX) {  // Ensure that motor doesn't rotate beyond 90°
            duty_cycle = DUTY_CYCLE_MAX;
        }
        else {                              // Use current value in ADC12MEM0 (from flex sensor circuit)
                                            // to update duty_cycle
            duty_cycle = (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN) * (((float) ADC12MEM0 - ADC0_MIN) / (ADC0_MAX - ADC0_MIN)) + DUTY_CYCLE_MIN;
            // Linear duty_cycle update as described in report
        }
        uint16_t pulse_duration = duty_cycle * TA0CCR0;
        TA0CCR1 = pulse_duration;           // Set the appropriate pulse width using duty_cycle
        P4OUT &= !LED1;                     // Turn off P4.7 LED
    }


    switch(DISPLAY_MODE) {
        case ADC0_DISPLAY_MODE:
            update_display(ADC12MEM0);
            TA0CCR2 = ((float) ADC12MEM1 / 4095) * ((float) ADC12MEM1 / 4095) * TA0CCR0;  // white LED quadratic-scaled adjustment for touch level indication
            break;
        case ADC1_DISPLAY_MODE:
            update_display(ADC12MEM1);
            if (ADC12MEM0 < ADC0_MIN) {
                TA0CCR2 = 0;                    // white LED set to minimum intensity at minimum rotation ADC and below
            } else if(ADC12MEM0 > ADC0_MAX) {
                TA0CCR2 = TA0CCR0;              // white LED set to maximum intensity at maximum rotation ADC and above
            } else {
                TA0CCR2 = (((float) ADC12MEM0 - ADC0_MIN) / (ADC0_MAX - ADC0_MIN)) * (((float) ADC12MEM0 - ADC0_MIN) / (ADC0_MAX - ADC0_MIN)) * TA0CCR0;
                // white LED quadratic-scaled adjustment for rotation level indication
            }
            break;
    }

    switch(DISPLAY_MODE) {                          // Check current Display Mode
        case ADC0_DISPLAY_MODE:                     // If mode display ADC0 (rotation ADC)
            if (!((P1IN >> 1) & BIT0)) {            // Check if P1.1 button is pressed
                while(!((P1IN >> 1) & BIT0)) {      // Turn on P1.0 LED while P1.1 is held
                    P1OUT |= LED0;
                }
                calibrate_min();                    // Calibrate 0° rotation ADC
                P1OUT &= ~LED0;                     // Turn off P1.0 LED
            }
            if (!((P2IN >> 1) & BIT0)) {            // Check if P2.1 button is pressed
                while(!((P2IN >> 1) & BIT0)) {      // Turn on P8.1 LED while P2.1 is held
                    P8OUT |= LED2;
                }
                calibrate_max();                    // Calibrate 90° rotation ADC, update flash memory
                P8OUT &= ~LED2;                     // Turn off P8.1 LED
            }
            break;

        case ADC1_DISPLAY_MODE:                     // if mode display ADC1 (pressure ADC)
            if (!((P1IN >> 1) & BIT0)) {            // Same as corresponding above case
                while(!((P1IN >> 1) & BIT0)) {
                    P1OUT |= LED0;
                }
                ADC1_THRESHOLD -= 50;               // Decrease pressure ADC threshold by 50
                P1OUT &= ~LED0;
                erase_segment(BAUD_REGISTER_D0);    // Erase current stored threshold
                write_flash(BAUD_REGISTER_D0, ADC1_THRESHOLD);  // Replace with new value
            }
            if (!((P2IN >> 1) & BIT0)) {            // Analogous to first case,
                while(!((P2IN >> 1) & BIT0)) {
                    P8OUT |= LED2;
                }
                ADC1_THRESHOLD += 50;               // Increase pressure ADC threshold by 50
                P8OUT &= ~LED2;
                erase_segment(BAUD_REGISTER_D0);
                write_flash(BAUD_REGISTER_D0, ADC1_THRESHOLD);
            }
            break;
    }
    if (P4IN & BIT0) {                              // Check if P4.0 is pressed
        while(P4IN & BIT0);                         // Wait while P4.0 is held
        DISPLAY_MODE = !DISPLAY_MODE;               // Switch current display mode to the other mode
    }

    ADC12CTL0 |= ADC12SC;                           // Start ADC sampling
}













