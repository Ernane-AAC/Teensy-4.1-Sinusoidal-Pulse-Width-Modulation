// ========================================================================================
// FLEXPWM 4 - PWM carrier frequency verification test
// UNVERSIDADE FEDERAL DE UBERLÂNDIA - Faculdade de Engenharia Elétrica
// Subject: DSP-EPS - Prof. Ernane A. A. Coelho - http://lattes.cnpq.br/9183492978798433
// This code is only for the demonstration of the FLEXPWM working principle 
// This code configures:
// FLEXPWM 4 - SM2 - Outputs 4A2 (pin 2 - GPIO_EMC_04:ALT1) and 4B2 (pin3 - GPIO_EMC_05:ALT1)
// FLEXPWM 1 - SM3 - Output 1A3 (pin 8 - GPIO_B1_00:ALT6)   
// Operation:  
// FLEXPWM 1 - SM3  generates a 1kHz PWMA signal output which is used as a clock for FLEXPWM 4 - SM2 via external jump and XBAR1 connection
// Notes: -> FLEXPWM1-SM3 is used as a prescaler of FLEXPWM 4. An external jumper is required on the expansion board to connect::
//           the output PWM-1A3 GPIO_B1_00-ALT6 (pin 8) to the input XBAR1_INOUT10 GPIO_B0_12-ALT1 (pin 32)
//           In the expansion board connect 1A3 (JP9) to Pin32 (JP14) - IMPORTANT => check if XBAR1_INOUT10 is configured as input
//        -> Using a relatively low PWM carrier frequency allows the signals to be visualized on a common oscilloscope, 
//           for example, a 20MHz bandwidth, 48MS/s oscilloscope.
//        -> Using a low value for the TOP count allows one to observe how pulse width modulation actually works in the FlexPWM module.
//           For two's complement counting, note that the counting range goes from -TOP to TOP-1, then Fcarrier=Timer_clk/prescale/(2*TOP). 
//           The use of INIT=-TOP and VAL1=TOP will imply a carrier frequency Fcarrier=Timer_clk/prescale/(2*TOP+1), since the counter=0 is one of the states.
//           One can use the counting method from 0 to 2*TOP-1 (only positive values), but in this case the values ​​of VAL2 and VAL3 will not be symmetrical.
//           The advantage of using two's complement counting is that VAL2 = -VAL3.  
// ========================================================================================
#include "Arduino.h"
#include <imxrt.h>

volatile uint16_t interrupt_count = 0;
volatile uint16_t LEDcounter = 0;
uint16_t TOP;
int16_t width_value;
const uint8_t LED_PIN = 13;        //pin of onboard LED

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
        if (interrupt_count >= 500)   //change pulse width each 5 seconds (interrupt frequency = 100Hz)
           {
            interrupt_count = 0;
            width_value++;
            if (width_value >= TOP) width_value=1; //max width_value=TOP-1 => max pulse width= 4-(-4)/10*100=80% | min pulse width= 1-(-1)/10*100=20% 

            // Update VAL2, VAL3, VAL4 and VAL5
            FLEXPWM4_SM2VAL2 = -width_value;              // PWM A (VAL2) goes High
            FLEXPWM4_SM2VAL3 =  width_value;              // PWM A (VAL3) goes Low
            //FLEXPWM4_SM2VAL4 =  width_value;            // PWM B (VAL4) goes High 
            //FLEXPWM4_SM2VAL5 = -width_value;            // PWM B (VAL5) goes Low
            // Considering INDEP = 0, (bit-13) of FLEXPWM4_SM2CTRL2, the PWMB will be a complementary signal of PWMA,
            //  that is, 180 degrees out of phase, so it is not necessary to define VAL4 and VAL5 
            
            //  Trigger an LDOK (Load OK) to update the active registers
            FLEXPWM4_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 2);  // Load ok for Submodule 2
           }

        LEDcounter++;
        if(LEDcounter>= 50 )   //change LED state each 0.5 seconds (interrupt frequency = 100Hz)
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
  Serial.println("Teensy 4.1 PWM Frequency Test");
  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN,LOW);

  TOP=5;          //"FlexPWM4-SM2 freq_carrier=EXT_CLK/prescale/(2*TOP)=1kHz/1/10=100Hz
  width_value=2;  // For Top=5: maximum width_value=4 (80%); minimum width_value=1 (20%)
                  // pulse width minimum step = 2*EXT_CLK period 

  init_flexpwm1_sm3();      //starts FlexPWM1-SM3
  Serial.println("FlexPWM1-SM3 was configured!");

  config_XBAR1();       // Connect EXT_CLK FLEXPWM4 to PAD_GPIO_B0_12(Teensy 4.1 Pin 32)
                        // To complete the clock connection, make a jumper from the PWMA (1A3 - on the expansion board JP9) 
                        //  to the digital input GPIO_B0_12 (pin 32 - on the expansion board JP14).
  Serial.println("EXT_CLK FLEXPWM4 connected to PAD_GPIO_B0_12(Teensy 4.1 Pin 32) via xbar1!");                      
  init_flexpwm4_sm2();  //starts FlexPWM4-SM2
  Serial.println("FlexPWM4-SM2 was configured!");
}

//***************************************************************************************************
void loop() {
  // It does nothing; the pulse width is altered by the ISR of the FlexPWM4-SM2 reload interrupt.
}

//***************************************************************************************************
void init_flexpwm1_sm3() {
  // Enable Clock for FlexPWM1 using the macro defined in imxrt.h
  // iMXRT.h definitions: #define CCM_CCGR_OFF        0 ->  00 - Clock is off during all modes. Stop enter hardware handshake is disabled.
  //                      #define CCM_CCGR_ON_RUNONLY 1 ->  01 - Clock is on in run mode, but off in WAIT and STOP modes
  //                                                        10 - Not applicable (Reserved).
  //                      #define CCM_CCGR_ON         3 ->  11 - Clock is on during all modes, except STOP mode.
  CCM_CCGR4 |= CCM_CCGR4_PWM1(CCM_CCGR_ON); //pwm1_clk_enable

  // Select clk source (IPG CLk:150MHz) 
  FLEXPWM1_SM3CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(0); // select clock source: CLK_SEL = 0b00 (IPG_CLK) 
                                                  //                                0b01 - EXT_CLK is used as the clock for the local prescaler and counter.
                                                  //                                0b10 - Submodule 0’s clock (AUX_CLK) is used as the source clock for the 
                                                  //                                       local prescaler and counter. This setting should not be used 
                                                  //                                       in submodule 0 as it will force the clock to logic 0.
                                                  //                                0b11 - reserved
  
  // Define prescaler
  FLEXPWM1_SM3CTRL = FLEXPWM_SMCTRL_PRSC(3);  // PRESCALE = 0b000 (Divide by 1) PWM clock frequency = f clk 
                                              // PRESCALE = 001b -  PWM clock frequency = f clk /2
                                              // PRESCALE = 010b -  PWM clock frequency = f clk /4
                                              // PRESCALE = 011b -  PWM clock frequency = f clk /8
                                              // PRESCALE = 100b -  PWM clock frequency = f clk /16
                                              // PRESCALE = 101b -  PWM clock frequency = f clk /32
                                              // PRESCALE = 110b -  PWM clock frequency = f clk /64
                                              // PRESCALE = 111b -  PWM clock frequency = f clk /128
 
  //Define counter range and and pulse width for PWMA 
  FLEXPWM1_SM3INIT = -9375;
  FLEXPWM1_SM3VAL0 = 0;
  FLEXPWM1_SM3VAL1 =  9374; // IPG CLk/prescaler/counting steps => 150MHz/8/(2*9375)= 50000 = 1 kHz
  FLEXPWM1_SM3VAL2 = -5000; 
  FLEXPWM1_SM3VAL3 =  5000; //pulse width(%)=(5000-(-5000))/(2*9375)*100= 53,3%

  // Configures pin 8 multiplexing to FlexPWM1-Submodule3-PWMA for oscilloscope signal visualization and connection to pin 32.
  // Output 1A3 (Teensy 4.1 pin 8 - GPIO_B1_00:ALT6) 
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 0x6;  // Set ALT6 for FlexPWM1 PWM1A3 

  //Output Enable (OUTEN): Enable output A for FlexPWM1 Submodule 3 
   FLEXPWM1_OUTEN |= FLEXPWM_OUTEN_PWMA_EN( 1 << 3 ); 

  //Output Polarity Control (OCTRL): Keep default polarity (low true)
  //FLEXPWM1_SM3OCTRL &= ~(FLEXPWM_SMOCTRL_POLA);  

  //Start the FlexPWM1 submodule3 
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(1 << 3);  // Set Load OK for Submodule 3 (loads initial VALs)
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_RUN(1 << 3);    // Start the counter for Submodule 3   
}

//***************************************************************************************************
void init_flexpwm4_sm2() {
  //Enable Clock for FlexPWM4 using the macro defined in imxrt.h
  CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);  // pwm4_clk_enable -> option 3: Clock is on during all modes, except STOP mode
  
  // Select clk source (EXT_CLK) 
  FLEXPWM4_SM2CTRL2 = FLEXPWM_SMCTRL2_CLK_SEL(1); // CLK_SEL = 0b01 - EXT_CLK is used as the clock for the local prescaler and counter.

  // Define prescaler
  FLEXPWM4_SM2CTRL = FLEXPWM_SMCTRL_PRSC(0) | FLEXPWM_SMCTRL_FULL; // PRESCALE = 0b000 (Divide by 1) PWM clk = EXT_CLK/1; enables full-cycle reloads (at VAL1) 
                                                                   // IMPORTANT: if FULL bit (VAL1) [or HALF bit (VAL0)] is reset, RF is not set when counter
                                                                   // reaches VAL1 and no interrupt is request. 
  //Define counter range and and pulse initial width for PWMA and PWMB
  FLEXPWM4_SM2INIT = -TOP;            // Carrier bottom
  FLEXPWM4_SM2VAL0 = 0;               // 
  FLEXPWM4_SM2VAL1 = TOP-1;           // Carrier TOP  => For TOP= 5  => F_carrier = EXT_CLk/prescaler/counting steps => 1kHz/1/(2*5)= 100Hz
  FLEXPWM4_SM2VAL2 = -width_value;          // PWM A (VAL2) goes High
  FLEXPWM4_SM2VAL3 =  width_value;          // PWM A (VAL3) goes Low  => initial pulse width(%)=(2-(-2))/(2*5)*100= 40%
  // FLEXPWM4_SM2VAL4 =  width_value;       // PWM B (VAL4) goes High
  // FLEXPWM4_SM2VAL5 = -width_value;       // PWM B (VAL5) goes Low
  // Considering INDEP = 0, (bit-13) of FLEXPWM4_SM2CTRL2, the PWMB will be a complementary signal of PWMA,
  //  that is, 180 degrees out of phase, so it is not necessary to define VAL4 and VAL5 

  // Configure Pin Multiplexing for FlexPWM4-Submodule2 PWMA and PWMB to see the signals in oscilloscope 
  // Output 4A2 (Teensy 4.1 pin 2 - GPIO_EMC_04:ALT1) 
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 0x1;  // Set ALT1 for FlexPWM4 PWM4A2 

  // Output 4B2 (Teensy 4.1 pin 2 - GPIO_EMC_05:ALT1) 
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = 0x1;  // Set ALT1 for FlexPWM4 PWM4B2 

  // Output Enable (OUTEN): Enable outputs A and B for FlexPWM4 Submodule 2 
  FLEXPWM4_OUTEN |= (FLEXPWM_OUTEN_PWMA_EN(1 << 2) | FLEXPWM_OUTEN_PWMB_EN(1 << 2)); 

  //Dead time configuration - number of IPBus clock cycles -> pag. 3170, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
  FLEXPWM4_SM2DTCNT0 = 50;  //deadtime during 0 to 1 transitions of the PWM_A output (assuming normal polarity) -> 50/150MHz=0.33us
  FLEXPWM4_SM2DTCNT1 = 50;  //deadtime during 0 to 1 transitions of the PWM_B output (assuming normal polarity)

  // Output Polarity Control (OCTRL): Keep default polarity (low true)
  //FLEXPWM4_SM2OCTRL &= ~(FLEXPWM_SMOCTRL_POLA | FLEXPWM_SMOCTRL_POLB); ->pag. 3161, bit-10 POLA, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021

  //Complementary Pair Operation INDEP=0: keep default (reset value)
  //FLEXPWM4_SM2CTRL2 &= ~FLEXPWM_SMCTRL2_INDEP; //->pag. 3144, bit-13 INDEP, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
  
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

//***************************************************************************************************
void config_XBAR1(void){ // connect EXT_CLK FLEXPWM4 to PAD_GPIO_B0_12(Teensy 4.1 Pin 32)
  // Enable Clock for XBAR1 using the imxrt.h macro
  // This sets bits 22-23 of CCGR2 to CCM_CCGR_ON:0x3 (Full On)
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON); //xbar1_clk_enable -> pag. 1080, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021

  // Connect PAD_GPIO_B0_12 (Pin 32) to XBAR1_INOUT10
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_12 = 0x1; //ALT1 mux port: XBAR1_INOUT10 -> pag. 516, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021 

  // Set GPR6 Bit 22 to 0 (Direction: Input for INOUT10)
  IOMUXC_GPR_GPR6 &= ~(1 << 22); // 0 XBAR_INOUT as input
                                 // 1 XBAR_INOUT as output  
                                 //->pag. 342, bit-26 of IOMUXC_GPR_GPR6, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
  //IOMUX_XBAR_INOUT10 as input is assigned to XBAR1_IN10 -> pag. 61, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
 
  // Then, connect  XBAR1_IN10 to XBAR1_OUT48 (FLEXPWM4_EXT_CLK) -> pag. 69, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021
  // The input for XBAR1_OUT48 (FLEXPWM4_EXT_CLK) is select by SEL48 in XBARA1_SEL24 register
  XBARA1_SEL24 = (XBARA1_SEL4 & ~0x7F) | 10;  //->(pag. 3325, i.MX RT1060 Processor Reference Manual, Rev. 3, 07/2021)
}