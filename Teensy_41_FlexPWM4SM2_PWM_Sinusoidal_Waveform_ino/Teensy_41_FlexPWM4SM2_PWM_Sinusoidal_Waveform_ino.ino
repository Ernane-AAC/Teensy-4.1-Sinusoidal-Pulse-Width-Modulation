// ========================================================================================
// Teensy 4.1 PWM Sinusoidal Generation
// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// This code is only for the demonstration of the sinusoidal pulse width modulation (SPWM)
// This code configures:
// FLEXPWM 4 - SM2 - Outputs 4A2 (pin 2 - GPIO_EMC_04:ALT1) and 4B2 (pin3 - GPIO_EMC_05:ALT1)   
// Operation:  
// - A 50Hz sine wave modulation is generated at the FlexPWM4-SM2's PWMA output(4A2). The corresponding complementary signal is generated at the PWMB output(4B2).
// - The sinusoidal modulating signal is calculated in real time. This calculation is performed by the ISR requested by the reload interrupt of the FlexPWM module.
// - Every 5 seconds, the third harmonic is added/removed from the output signal for observation purposes only.
// - In this example, a PWM carrier frequency of 20kHz will be used.
// - A low-pass filter corresponding to two cascaded RC sections (R=4k99;C=10nF) will be used to display the sinusoidal signal on the oscilloscope.
// - The filter has a cutoff frequency of 3189.5 Hz. Neglecting the loading of one section on the other,
//   the attenuation at the carrier frequency (20KHz) will be -32dB (x0.025).
// Notes: -> In Teensy 4.1 (iMXRT1062), the timers linked to the FlexPWM module are 16-bit and only provide an increasing count (analogous to a sawtooth wave).
//           This implies a slightly different manipulation of the comparison registers than that used in PWM modulators that provide both increasing
//           and decreasing counts (analogous to a triangle wave).
//        -> In a conventional modulator, the intersection of the triangular wave with a 50% offset from the TOP with the reference sinusoid with a 50% offset 
//           from the TOP defines the falling edge of the output pulse on the ascending ramp and the rising edge of the pulse on the descending ramp.
//        -> In the Teensy 4.1 FlexPWM, the intersection of the sawtooth wave with zero offset (2's complement count) and a reference sine wave with amplitude 
//           and offset of 50% of TOP defines the falling edge of the output pulse, and the intersection with a symmetrical sine wave defines the rising edge
//           of the output pulse.
//        -> It is important to note that for a carrier frequency F_carrier=Timer_clk/prescale/counter_steps, the range of counter_steps must go from 
//           -TOP to TOP-1 (counter_steps=2*TOP), since the zero state of the counter must be considered.
//        -> The dead time was set to 0.33 s. It is not necessary in this application since the objective is to visualize PWM signals, 
//           aiming to understand the operation of the FlexPWM module. For the control of the arm of a power inverter, the dead time is important 
//           and should be considered. This can preferably be configured in the gate drive circuit of the power switches. 
// ========================================================================================
#include "Arduino.h"
#include <imxrt.h>
#include <math.h>

volatile uint32_t interrupt_count = 0;
volatile uint16_t LEDcounter = 0;
int16_t TOP, HALF_TOP;
volatile int16_t width_value, cpl_width_value;
int16_t width_value_max, width_value_min;
uint8_t third_harm;
const uint8_t LED_PIN = 13;        //pin of onboard LED
volatile float phase;
float f, w, Vsin, ts, width;

//***************************************************************************************************
void pwm4_sm2_isr() {
    uint16_t status = FLEXPWM4_SM2STS;
    if (status & FLEXPWM_SMSTS_RF) { // This interrupt Vector (IRQ 149) is shared with:
                                      // FLEXPWM4 OR: - capture PWM2 interrupt 
                                      //              - compare PWM2 interrupt
                                      //              - reload  PWM2 interrupt
                                      // See -> pag. 51, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
                                      // In this code only reload interrupt is used, but just in case you enable any other... check respective interrupt flag
        interrupt_count++;
        if (interrupt_count >= 100000)   //insert/remove third harmonic each 5 seconds (interrupt frequency = 20kHz)
           {
            interrupt_count = 0;
            third_harm ^= 1; //toggle third_harm variable
           }
        phase= phase + w*ts; //time integral of w
        if(phase >= TWO_PI) phase = phase-TWO_PI; //anti-windup
        Vsin= 0.7*sin(phase) + third_harm*0.2*sin(3*phase); // calculation of the sinusoidal signal - the total amplitude must be less than 0.95 (width limitation)
        width =  Vsin*(float)HALF_TOP + (float)HALF_TOP;    // explore maximum amplitude and insert offset 
        width_value = (int16_t)width;
        if (width_value > width_value_max) width_value = width_value_max; // maximum pulse saturation 
        if (width_value < width_value_min) width_value = width_value_min; // minimum pulse saturation 

        //Update VAL2, VAL3, VAL4 and VAL5
        FLEXPWM4_SM2VAL2 = -width_value;      // PWM A (VAL2) goes High
        FLEXPWM4_SM2VAL3 =  width_value;      // PWM A (VAL3) goes Low
        //FLEXPWM4_SM2VAL4 =  width_value;      // PWM B (VAL4) goes High 
        //FLEXPWM4_SM2VAL5 = -width_value;      // PWM B (VAL5) goes Low
        //Considering INDEP = 0, (bit-13) of FLEXPWM4_SM2CTRL2, the PWMB will be a complementary signal of PWMA,
        // that is, 180 degrees out of phase, so it is not necessary to define VAL4 and VAL5 

        // Trigger an LDOK (Load OK) to update the active registers
        FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 2);  // Load ok for Submodule 2

        LEDcounter++;
        if(LEDcounter>= 10000 )   //change LED state each 0.5 seconds (interrupt frequency = 20kHz)
           {
             LEDcounter=0;
             digitalWriteFast(LED_PIN,!digitalReadFast(LED_PIN)); //blink onboard LED 1Hz
           }
    }

    FLEXPWM4_SM2STS = status; // Clear any flag that is pending
    asm("dsb");   // Data Synchronization Barrier -> This forces the CPU to wait until the peripheral really clears its interrupt flag,
                  //                                 It prevents the ISR from retriggering due to uncleared interrupt flags.
}

//***************************************************************************************************
void setup() {
  // In spite of reset value shown in pag. 3200 of i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021.
  //  the code starts with the Flexpwm timers already in operation. Perhaps, during startup, 
  //  the Arduino code sets the RUN bit in the Master Control Register (MCTRL). It needs confirmation!.
  FLEXPWM1_MCTRL &= ~FLEXPWM_MCTRL_RUN(1 << 3);   // Stop the counter in FLEXPWM1   
  FLEXPWM4_MCTRL &= ~FLEXPWM_MCTRL_RUN(1 << 2);   // Stop the counter in FLEXPWM4  
  FLEXPWM1_MCTRL &= ~FLEXPWM_MCTRL_CLDOK(1 << 3); // Clear Load OK for FLEXPWM1-SM3 (just in case it is set) 
  FLEXPWM4_MCTRL &= ~FLEXPWM_MCTRL_CLDOK(1 << 2); // Clear Load OK for FLEXPWM4-SM2 (just in case it is set) 

  Serial.begin(115200);
  delay(1000);
  Serial.println("Teensy 4.1 PWM Sinusoidal Generation");
  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN,LOW);

  TOP = 3750;              // FlexPWM4-SM2 freq_carrier=IPG_CLK/prescale/(2*TOP)= 150MHz/1/(2*3750) = 20kHz
  HALF_TOP = TOP >> 1;
  width_value = HALF_TOP;  // initial pulse width duty cycle(%)=(1/2*TOP-(-1/2*TOP))/(2*TOP)*100
                           // initial pulse width duty cycle(%)=(1875-(-1875))/(2*3750)*100= 50%
  width_value_max = 3562;  // pulse width limitation: -> width_value max = 3562  - maximum pulse width ~= 95% (94.9867%)
  width_value_min = 187;   //                         -> width_value min = 187  -  minimum pulse width ~= 05% (4.9867%)

  f=50.0f;          // fundamental frequency;
  w=TWO_PI*f;       // fundamental angular frequency
  ts=1.0f/20000.0f; // sampling period

  init_flexpwm4_sm2();  //starts FlexPWM4-SM2
  Serial.println("FlexPWM4-SM2 was configured!");
  Serial.println("Connect a low-pass filter as described in the header to output 4A2 (pin 2) and another to output 4B2 (pin 3) to view the sinusoidal signals.");
}

//***************************************************************************************************
void loop() {
  // It does nothing; the pulse width is altered by the ISR of the FlexPWM4-SM2 reload interrupt.
}


//***************************************************************************************************
void init_flexpwm4_sm2() {
  // Enable Clock for FlexPWM1 using the macro defined in imxrt.h
  // iMXRT.h definitions: #define CCM_CCGR_OFF        0 ->  00 - Clock is off during all modes. Stop enter hardware handshake is disabled.
  //                      #define CCM_CCGR_ON_RUNONLY 1 ->  01 - Clock is on in run mode, but off in WAIT and STOP modes
  //                                                        10 - Not applicable (Reserved).
  //                      #define CCM_CCGR_ON         3 ->  11 - Clock is on during all modes, except STOP mode.
  CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);  // pwm4_clk_enable -> option 3: Clock is on during all modes, except STOP mode
  
  // Select clk source (EXT_CLK) 
  FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0); // select clock source: CLK_SEL = 0b00 (IPG_CLK) 
                                                  //                                0b01 - EXT_CLK is used as the clock for the local prescaler and counter.
                                                  //                                0b10 - Submodule 0’s clock (AUX_CLK) is used as the source clock for the 
                                                  //                                       local prescaler and counter. This setting should not be used 
                                                  //                                       in submodule 0 as it will force the clock to logic 0.
                                                  //                                0b11 - reserved
  // Define prescaler
  FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_PRSC(0) | FLEXPWM_SMCTRL_FULL; // PRESCALE = 0b000 (Divide by 1) PWM clock frequency = f clk 
                                                                   // PRESCALE = 001b -  PWM clock frequency = f clk /2
                                                                   // PRESCALE = 010b -  PWM clock frequency = f clk /4
                                                                   // PRESCALE = 011b -  PWM clock frequency = f clk /8
                                                                   // PRESCALE = 100b -  PWM clock frequency = f clk /16
                                                                   // PRESCALE = 101b -  PWM clock frequency = f clk /32
                                                                   // PRESCALE = 110b -  PWM clock frequency = f clk /64
                                                                   // PRESCALE = 111b -  PWM clock frequency = f clk /128
                                                                   // Enables full-cycle reloads (at VAL1) 
                                                                   // IMPORTANT: if FULL bit (VAL1) [or HALF bit (VAL0)] is reset, RF is not set when counter
                                                                   // reaches VAL1 and no interrupt is request. 
  //Define counter range and and pulse initial width for PWMA and PWMB
  FLEXPWM4_SM2INIT = -TOP;            // Carrier bottom
  FLEXPWM4_SM2VAL0 = 0;               // 
  FLEXPWM4_SM2VAL1 = TOP-1;           // Carrier TOP  => For TOP= 3750  => F_carrier = EXT_CLk/prescaler/counting steps => 150MHz/1/(2*3750)= 20kHz
  FLEXPWM4_SM2VAL2 = -width_value;       // PWM A (VAL2) goes High => initial pulse width duty cycle(%)=(1/2*TOP-(-1/2*TOP))/(2*TOP)*100
  FLEXPWM4_SM2VAL3 =  width_value;       // PWM A (VAL3) goes Low  => initial pulse width duty cycle(%)=(1875-(-1875))/(2*3750)*100= 50%
  // FLEXPWM4_SM2VAL4 =  width_value;      // PWM B (VAL4) goes High 
  // FLEXPWM4_SM2VAL5 = -width_value;      // PWM B (VAL5) goes Low
  // Considering INDEP = 0, (bit-13) of FLEXPWM4_SM2CTRL2, the PWMB will be a complementary signal of PWMA,
  //  that is, 180 degrees out of phase, so it is not necessary to define VAL4 and VAL5 

  // Configure Pin Multiplexing for FlexPWM4-Submodule2 PWMA and PWMB to see the signals in oscilloscope 
  // Output 4A2 (Teensy 4.1 pin 2 - GPIO_EMC_04:ALT1) 
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 0x1;  // Set ALT1 for FlexPWM4 PWM4A2 

  // Output 4B2 (Teensy 4.1 pin 2 - GPIO_EMC_05:ALT1) 
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = 0x1;  // Set ALT1 for FlexPWM4 PWM4B2 

  // Output Enable (OUTEN): Enable outputs A and B for FlexPWM4 Submodule 2 
  FLEXPWM4_OUTEN = (FLEXPWM_OUTEN_PWMA_EN(1 << 2) | FLEXPWM_OUTEN_PWMB_EN(1 << 2)); 

  //Dead time configuration - number of IPBus clock cycles -> pag. 3170, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
  FLEXPWM4_SM2DTCNT0 = 50;  //deadtime during 0 to 1 transitions of the PWM_A output (assuming normal polarity) -> 50/150MHz=0.33us
  FLEXPWM4_SM2DTCNT1 = 50;  //deadtime during 0 to 1 transitions of the PWM_B output (assuming normal polarity)

  // Output Polarity Control (OCTRL): Keep default polarity (low true)
  //FLEXPWM4_SM2OCTRL &= ~(FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB); ->pag. 3161, bit-10 POLA, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021

  //Complementary Pair Operation: keep default
  //FLEXPWM4_SM2CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP; //->pag. 3144, bit-13 INDEP, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
  //FLEXPWM4_SM2CTRL2 |= FLEXPWM_SMCTRL2_INDEP;
  
  // Enable and Set Priority for the Interrupt
  attachInterruptVector(IRQ_FLEXPWM4_2, pwm4_sm2_isr);
  NVIC_SET_PRIORITY(IRQ_FLEXPWM4_2, 64);   // Only the superior nibble is considered => 64=0x40= 0100 0000 => Level 4 out of 15 (16 levels)
  FLEXPWM4_SM2STS = FLEXPWM_SMSTS_RF;      // The flags in register are Write-1-to-Clear, not use |= (OR-Equals) so it can clear any other pending flag.
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM4_2);
  FLEXPWM4_SM2INTEN |= FLEXPWM_SMINTEN_RIE; // Enable Interrupt on Reload (RSTS)

  // Start the FlexPWM4 submodule2
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 2);  // Set Load OK for Submodule 2 (loads initial VALs)
  FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_RUN(1 << 2);   // Start the counter for Submodule 2
}

