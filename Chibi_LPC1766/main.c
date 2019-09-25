/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h" 
#include <stdlib.h>

#define GPIO_INJ_4	BIT(21)
#define HIGH 			      1					// for bit shifting operation
#define PINSEL_VALUE	  3				
	
//			PinName			PinNumber	    	            	SIGNIFICANCE						       	      Default Value						
#define PCADC 			12					//	A/D converter (ADC) power/clock control bit.			      0
#define PDN					21					//	Set the bit to make the A/D converter operational 	    0
#define PCLK_ADC	  24					//	Here there are two pins 24 and 25 for ADCPCLK 			    0 (CCLK/4)
#define AD0_4				28					//	Performs AD0.2 Function when 01	value in PINSEL3		    0
#define AD0_2				18					//	Performs AD0.2 Function when 01	value in PINSEL1		    0
#define CLKDIV		 	 8					//	to produce the clock for the A/D converter			  	    0
#define BURST				16					//	Set this bit for ADC to do repeated conversions			    0
#define ADGINTEN		 8					//	individual ADC channels will generate interrupts.		    1 
#define DONE4				 4					//	DONE status flag from the result for A/D channel 4.     0
#define SEL4				 4					//	Selects which AD0.7:0 to be sampled and converted		    0
#define DONE2				 2					//	DONE status flag from the result for A/D channel 2.     0
#define SEL2				 2					//	Selects which AD0.7:0 to be sampled and converted		    0

unsigned int Vtps;
int blinkrate = 500;
//-------------This function will initialize the ADC, set the values and make the ADC operational --------------------------------------
void init_ADC()
{	
    //  STEP 1:-	In the PCONP register (Table 46), set the PCADC bit.
			LPC_SC->PCONP				|=	(HIGH	<< 	PCADC);
	//	STEP 2:-	In the A/D Control Register (AD0CR), make the ADC operational 
			LPC_ADC->CR				|=	(HIGH	<<	PDN);
			//LPC_ADC->CR				|=	(HIGH	<<	SEL4);			//SELECTING CHANNEL 
    /*	STEP 3:-	            This is an important step, wherein we need to set the working frequency
								for our ADC peripheral. Here, we need to select the Clock frequency for
								ADC by Dividing the CCLK (100 MHz in our case) with either 4,1,2,8
								this depends on the value we provide to the PCLKSEL0 register (Pins 25:24) 
					
								Since the default value is 0, So the PCLK for ADC is CCLK/4 = 25 MHzHence, we need not to modify these two bits (25:24)*/
	
	/*	STEP    4:-		        Setting the CLKDIV value, APB clock (PCLK_ADC0) will be divided by Value in CLKDIV+1
								and the resultand frequency will be used by the ADC (make sure resultant is <= 13 MHz) 
								Since, defauld Value of APB clock (PCLK_ADC0) is 25 MHz, hence we will provide CLKDIV
								a value = 1, hence the result frequency will be PCLK_ADC0/(CLKDIV+1), 25/(1+1) = 12.5 MHz */			
			LPC_ADC->CR				|=	(HIGH	<<	CLKDIV);
    // 	STEP5:-			        Important!!! here we will be selecting the ADC Channel, (2 in our case, )
			LPC_ADC->CR				|=	(HIGH	<<	SEL4);
	//	STEP6:-		            Setting ADC0 pins function using PINSEL register.	For our BlueBoard, its ADC0.4 channel	
			LPC_PINCON->PINSEL3	        |=	(3	    <<	AD0_4);
	/*	STEP7:-		            Setting the Conversion Mode For our ADC, here we may either select the Burst Mode (Contineous Conversion)
								, or we may select the normal conversion mode, wherein the conversion will  be done only once,
								In this code, we are using the burst mode for the contineous conversion of the data coming form the POT 
								Also, when this bit is set (and the Start Bits are set to 00), conversion starts		
								As recommended in the DataSheet, we are also clearing the ADGINTEN bit in the AD0INTEN register
								this will make sure that Only the individual ADC channels enabled by ADINTEN7:0 will generate interrupts. */			
			LPC_ADC->INTEN		    &=	 ~(HIGH     <<	ADGINTEN);
			LPC_ADC->CR				|=    (HIGH	    <<	BURST);	

      //
      LPC_ADC->CR				|=	(HIGH	<<	SEL2);
      LPC_PINCON->PINSEL1	|=	(1	    <<	AD0_2);
}
//---------------------------------------------------------------------------------------------------------------------------------------------


//----------------This function will read the data from data register, perform the proper shifting and return data -----------
unsigned int capture_value(int32_t DONEx, int32_t x)//Paramter input DONEx dan 
{
	unsigned int data;
	/*	in order to check for the conversion of the data (polling approach), we will monitor the flags in ADSTAT register
			In our case, since we are using the Channel 4 of the ADC0 */
	//	we will wait for the ADC to complete the conversion, hence we are monitoring the DONE4 flag in ADCSTAT register	
	while(((LPC_ADC->STAT) & (HIGH << DONEx)) != (HIGH << DONEx)){}
	/*	once the conversion is done, we will capture the data from the ADC Data Register, (from RESULT bits 15:4) 
			IMPORTANT !!!, in order to use the data, we need to mask the RESULT bits (15:4) and shift the data to MSB, since
			the data will be apprearing on the bits 15 (LSB) :4 (MSB) */
	data	=	(((LPC_ADC->DR[x]) & (0x0FFF0)) >> 4 );
	//	in the above peice of code we have Masked the data and shifter the data to the right 4 times, now LSB is at bit 0
  //chprintf((BaseSequentialStream *)&SD3,"rawbit: %d\r\n", data);
	return(data);
}
//----------------------------------------------------------------------------------------------------------------------------

/*
 * Baca analog TPS throttle
 * PIN P2.0
 */
static WORKING_AREA(waThread4, 256);
static msg_t thd_tps(void *arg) {
  /*
    VREFP = 2.5V

    Below are the steps for configuring the LPC1768 ADC.

    -Configure the GPIO pin for ADC function using PINSEL register.
    -Enable the CLock to ADC module.
    -Deselect all the channels and Power on the internal ADC module by setting ADCR.PDN bit.
    -Select the Particular channel for A/D conversion by setting the corresponding bits in ADCR.SEL
    -Set the ADCR.START bit for starting the A/D conversion for selected channel.
    -Wait for the conversion to complete, ADGR.DONE bit will be set once conversion is over.
    -Read the 12-bit A/D value from ADGR.RESULT. 
  */
  (void)arg;
  chRegSetThreadName("tps_throttle");
  
  init_ADC();
  chprintf((BaseSequentialStream *)&SD3,"TES TPS\r\n\r\n");
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }
}

static const SerialConfig uart2_cfg = {
  115200,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};

/*
 * LED blinker thread untuk debug
 * PIN P2.0
 */
static WORKING_AREA(waThread1, 128);
static msg_t thd_led_debug(void *arg) {

  (void)arg;
  chRegSetThreadName("Blink_debug");
  LPC_PINCON->PINSEL4  |= ((0 << 1) | (0 << 0)); //P2.0
  LPC_GPIO2->FIODIR0 |= ( 1 << 0 ); //P2.0
  while (TRUE) {
    palTogglePad(GPIO2, 0); //P2,0
    chThdSleepMilliseconds(blinkrate);
  }
}
/*
 * Test IRF840-INJECTOR4-GPIO
 * PIN P0.11, P1.21
 */
static msg_t thd_test_inj_GPIO(void *arg) {

  (void)arg;
  chRegSetThreadName("TEST_INJ_GPIO");
  //INJ = P0.11
  //INJ4 = P1.21
  LPC_PINCON->PINSEL0 |= (0 << 23) | ( 0 << 22) ; //P0.11 -> PINSEL0 22:23 = 00 -> GPIO
  LPC_PINCON->PINMODE0 |= (1 << 23) | ( 1 << 22) ;//pull down res
  LPC_GPIO0->FIODIR1 |= ( 1 << 3) ;//P0.11 sbg OUTPUT

  LPC_PINCON->PINSEL3 |= (0 << 11) | ( 0 << 10) ; //P1.21 -> PINSEL3 10:11 = 00 -> GPIO
  LPC_PINCON->PINMODE3 |= (1 << 11) | ( 1 << 10) ;//pull down res
  LPC_GPIO1->FIODIR2 |= ( 1 << 5) ;//P2.21 sbg OUTPUT

  LPC_GPIO0->FIOSET1 |= ( 1 << 3) ;
  while (TRUE) {
    //SET Coil ON
    LPC_GPIO1->FIOSET2 |= ( 1 << 5) ;
    chThdSleepMilliseconds(3000);
    //SET Coil OFF
    LPC_GPIO1->FIOCLR2 |= ( 1 << 5) ;
    chThdSleepMilliseconds(3000);
  }
}
/*
 * CAN BUS
 * 
 */
struct can_instance {
  CANDriver     *canp;
  uint32_t      led;
};

static const struct can_instance can1 = {&CAND1, NULL};

#if LPC17xx_CAN_USE_FILTER
static const CANFilterExt cfe_id_table[2] = {
  CANFilterExtEntry(0, 0x0ABCDEF0),
  CANFilterExtEntry(1, 0x01234567)

};

static const CANFilterConfig canfcfg = {
 0,
 NULL,
 0,
 NULL,
 0,
 NULL,
 0,
 cfe_id_table,
 2,
 NULL,
 0
};
#endif

/*
 * CANBUS  
 * Can BitRate = PCLK_CAN1/((TSEG1+TSEG2+3)*(BRP+1))
 */
/*100kbps, PCLK = CCLK =98Mhz, PCLK_CAN1=98Mhz*/
// static const CANConfig cancfg = {
//   0,
//   CANBTR_SJW(3) | CANBTR_TESG2(2) |
//   CANBTR_TESG1(5) | CANBTR_BRP(97)
// };

/*125kbps, PCLK = CCLK =98Mhz, PCLK_CAN1=98Mhz*/
// static const CANConfig cancfg = {
//   0,
//   CANBTR_SJW(3) | CANBTR_TESG2(4) |
//   CANBTR_TESG1(7) | CANBTR_BRP(55)
// };

/*250kbps, PCLK = CCLK =98Mhz, PCLK_CAN1=98Mhz*/
// static const CANConfig cancfg = {
//   0,
//   CANBTR_SJW(3) | CANBTR_TESG2(4) |
//   CANBTR_TESG1(7) | CANBTR_BRP(27)
// };

/*500kbps, PCLK = CCLK =98Mhz, PCLK_CAN1=98Mhz*/
static const CANConfig cancfg = {
  0,
  CANBTR_SJW(0) | CANBTR_TESG2(1) |
  CANBTR_TESG1(3) | CANBTR_BRP(27)
};

/*CANx->BTR  = (TSEG2<<20)|(TSEG1<<16)|(3<<14)|BRP*/

/*
 * Receiver thread.
 */
static WORKING_AREA(can_rx1_wa, 256);
static WORKING_AREA(can_rx2_wa, 256);
static msg_t can_rx(void *p) {
  struct can_instance *cip = p;
  EventListener el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("CAN_Receive_Msg");
  chEvtRegister(&cip->canp->rxfull_event, &el, 0);
  int i = 1;
  chprintf((BaseSequentialStream *)&SD3,"Thread - CAN Receiver\r\n");
  //while(!chThdShouldTerminate()) {
  while(true){
    // if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
    //   continue;
    // while (canReceive(cip->canp, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == RDY_OK) {
    //   /* Process message.*/
    //   palTogglePad(GPIO0, cip->led);
    // }

    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
      continue;

    while (canReceive(cip->canp, CAN_ANY_MAILBOX, &rxmsg, TIME_INFINITE) == RDY_OK) {
      /* Process message.*/
      chprintf((BaseSequentialStream *)&SD3,"Receive Message (CANBus) ke-%i\r\n", i++);
      //palToggleLine(LED302);
      chprintf((BaseSequentialStream *)&SD3,"MSG ID (hex): %x\r\n", rxmsg.SID);
      //Print data in hexadecimal format per 1 byte
      chprintf((BaseSequentialStream *)&SD3,"data in hex \r\n");
      for(int y=0 ; y<8; y++){
        chprintf((BaseSequentialStream *)&SD3," data[%i] : %x\r\n", y, rxmsg.data8[y]);
      }
      chprintf((BaseSequentialStream *)&SD3,"data in decimal : %i\r\n", rxmsg.data32[0]);

      if(rxmsg.SID == 32 && rxmsg.data32[0] == 1){
        chprintf((BaseSequentialStream *)&SD3,"Nyalakan sesuatu\r\n");
        chThdSleepMilliseconds(100);
        chThdSleepMilliseconds(5000);
      }else if(rxmsg.SID == 0x20 && rxmsg.data32[0] == 0){
        chprintf((BaseSequentialStream *)&SD3,"Matikan sesuatu\r\n");
      } 
    }
  }
  chEvtUnregister(&CAND1.rxfull_event, &el);
  return 0;
}

/*
 * Transmitter thread.
 */
static WORKING_AREA(can_tx_wa, 256);
static msg_t thd_can_tx(void * p) {
  CANTxFrame txmsg_can1;
  CANTxFrame txmsg_can2;

  (void)p;
  chRegSetThreadName("CAN_Send_Msg");
  txmsg_can1.IDE = CAN_IDE_STD;
  txmsg_can1.SID = 0x20;
  txmsg_can1.RTR = CAN_RTR_DATA;  
  txmsg_can1.DLC = 8;
  txmsg_can1.data32[0] = 0x00000001;
  txmsg_can1.data32[1] = 0x00000000;

  txmsg_can2.IDE = CAN_IDE_STD;
  txmsg_can2.SID = 0x20;
  txmsg_can2.RTR = CAN_RTR_DATA;  
  txmsg_can2.DLC = 8;
  txmsg_can2.data32[0] = 0x00000000;
  txmsg_can2.data32[1] = 0x00000000;

  // txmsg_can1.IDE = CAN_IDE_EXT;
  // txmsg_can1.EID = 0x01234567;
  // txmsg_can1.RTR = CAN_RTR_DATA;
  // txmsg_can1.DLC = 8;
  // txmsg_can1.data32[0] = 0x55AA55AA;
  // txmsg_can1.data32[1] = 0x00FF00FF;

  // txmsg_can2.IDE = CAN_IDE_EXT;
  // txmsg_can2.EID = 0x0ABCDEF0;
  // txmsg_can2.RTR = CAN_RTR_DATA;
  // txmsg_can2.DLC = 8;
  // txmsg_can2.data32[0] = 0x66AA66AA;
  // txmsg_can2.data32[1] = 0x44FF44FF;
  int i = 0;
  chprintf((BaseSequentialStream *)&SD3,"Thread - CAN Transmitter\r\n");
  //while (!chThdShouldTerminate()) {
  while(true){
    chThdSleepMilliseconds(4000);
    if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg_can1, MS2ST(100)) == RDY_OK){
      chprintf((BaseSequentialStream *)&SD3,"Success Sent CAN msg 1 : %d\r\n", i);
      i++;
    }else{
      chprintf((BaseSequentialStream *)&SD3,"Failed Sent CAN msg 1 : %d\r\n", i);
    }
    chThdSleepMilliseconds(4000);
    if(canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg_can2, MS2ST(100)) == RDY_OK){
      chprintf((BaseSequentialStream *)&SD3,"Success Sent CAN msg 2 : %d\r\n", i);
      i++;
    }else{
      chprintf((BaseSequentialStream *)&SD3,"Failed Sent CAN msg 2 : %d\r\n", i);
    }
  }
  return 0;
}



/*
  PWM-Servo thread dengan MCPWM pin
 * PIN P1.25
 */
static msg_t thd_servo(void *arg) {

  (void)arg;
  chRegSetThreadName("Motor_servo");
  LPC_SC->PCONP |= (1 << 17);                     // Reg. PCMCPWM bit 17 -> Set Power on
  LPC_SC->PCLKSEL1 |= (0 << 31) | (1 << 30) ;     // Reg. PCLK_MC bit 31:30 -> Set PCLK = CCLK (100MHz)
  LPC_PINCON->PINSEL3 |= (0 << 19) | ( 1 << 18) ; // Reg. PINSEL3 bit 19:18 -> Set P1.25 sbg MCOA1
  LPC_PINCON->PINMODE3 |= (1 << 19) | (1 <<18) ;  // Reg. PINMODE3 bit 18:19 -> Set P1.25 mode pull down resistor 
  LPC_GPIO1->FIODIR3 |= (1 << 1);                 // Reg. FIODIR3 Bit 1 -> P1.25 set sbg OUTPUT
  LPC_MCPWM->LIM1 = 500000; //500000 = 200Hz
  while (TRUE) {
    LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    LPC_MCPWM->MAT1 = 60000;          //Set Duty Cycle 0 derajat = 0.6ms, 0.6ms/5ms = 12% dari 500000 = 60000
    LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt
    chThdSleepMilliseconds(3000);
    LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    LPC_MCPWM->MAT1 = 200000;         //Set Duty Cycle 0 derajat = 2 ms, 2ms/5ms = 40% dari 500000 = 200000
    LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt ; 0=STOP ; 1=START
    chThdSleepMilliseconds(3000);
  }
}
/*
 * PWM-Servo thread dengan MCPWM pin
 * PIN P1.25
 * 
 * //motor servo
 * 
 * /
 // Set the Limit register
  /*cara hitung limit untuk freq 50 hz
    fclock (PCLK) = 100Mhz/50hz (LIM1=2000000)
  */
  //LPC_MCPWM->LIM1 = 2000000; //limit untuk freq 50hz(20ms) && PCLK=100Mhz

    // LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    // LPC_MCPWM->MAT1 = 60000;          //Set Duty Cycle 0 derajat = 0.6ms (3%) -> 3% dari 2000000 = 60000
    // LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt
    // chThdSleepMilliseconds(5000);
    // LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    // LPC_MCPWM->MAT1 = 200000;         //Set Duty Cycle 0 derajat = 2 ms (10%) -> 10% dari 2000000 = 200000
    // LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt ; 0=STOP ; 1=START
    // chThdSleepMilliseconds(5000);

static WORKING_AREA(waThread2, 512);
static msg_t thd_electronic_throttle(void *arg) {

  (void)arg;
  chRegSetThreadName("Servo_throttle_gas");

  int error, prev_error;
  int target_pos; //set point (SP)
  int current_pos;
  int command_pos;
  int time_change =0;
  unsigned long int time_prev=0;
  unsigned long int time_now=0;
  // variables for the pid function
  unsigned long int errSum = 0;
  unsigned long int dErr=0; 
  float KP, setKP;
  int KI, KD;
  int output;
  /*
    ADC - maks 4400mv ; min 894mv
  */
  int tps_v_max = 4400;
  int tps_v_min = 890;
  int voffset = 140; //offset pembacaan adc 200mv
  int count_to_print=0;

  LPC_SC->PCONP |= (1 << 17);                     // Reg. PCMCPWM bit 17 -> Set Power on
  LPC_SC->PCLKSEL1 |= (0 << 31) | (1 << 30) ;     // Reg. PCLK_MC bit 31:30 -> Set PCLK = CCLK (100MHz)
  LPC_PINCON->PINSEL3 |= (0 << 19) | ( 1 << 18) ; // Reg. PINSEL3 bit 19:18 -> Set P1.25 sbg MCOA1
  LPC_PINCON->PINMODE3 |= (1 << 19) | (1 <<18) ;  // Reg. PINMODE3 bit 18:19 -> Set P1.25 mode pull down resistor 
  LPC_GPIO1->FIODIR3 |= (1 << 1);                 // Reg. FIODIR3 Bit 1 -> P1.25 set sbg OUTPUT

  /* max pwm = CCLK/Freq_signal*/
  int max_pwm = 2000000;
  int min_pwm = 0;

  int Pulse_width;
  LPC_MCPWM->LIM1 = max_pwm; //limit untuk freq 400hz && PCLK=100Mhz
  
  init_ADC();

  setKP = 0.7; KI=5; KD=2;

  while (TRUE) {
    unsigned int rawbit_tps = 0;
    unsigned int rawbit_potensio=0;
    for(int i=0;i<50;i++){
      rawbit_tps += capture_value(DONE4, 4);
    }
    rawbit_tps /= 50;
    for(int i=0;i<50;i++){
      rawbit_potensio += capture_value(DONE2, 2);
    }
    rawbit_potensio /= 50;
    
    command_pos =  (rawbit_potensio*2500/4096)*2;
    current_pos = ((rawbit_tps*2500/4096) - voffset)*2; //dikali 2 karena ada rangkaian pembagi tegangan dan op-amp untuk menyesuaikan dengan vref adc
    
    if((command_pos >= (target_pos+20)) || (command_pos <= (target_pos-20)) ){
      target_pos=command_pos;
    }
    if(target_pos>tps_v_max){
      target_pos=5000; //dipilih 5000 agar tidak pernah tercapai dan membuka sepenuhnya
    }else if(target_pos < tps_v_min){
      target_pos=0; ////dipilih 0 agar tidak pernah tercapai dan menutup sepenuhnya
    }
    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//
    //----------------------PID-----------------------------------------
    error = target_pos - current_pos;
    //error = 3500 - current_pos;
    //Adjust KP
    // if(error >= 0){
    //   if(error > 2350){
    //     KP=setKP*6.0F;
    //   }else if(error > 35  && error <= 120){
    //     KP=setKP*0.75F;
    //   }else if(error > 0 && error <= 35){
    //     KP=setKP*0.1F;
    //   }else{
    //     KP=setKP;
    //   }
    // }else{
    //   if(error < -500){
    //     KP=setKP*6.0F;
    //   }else if(error >= -120 && error < -35){
    //     KP=setKP*0.75F;
    //   }else if(error >= -35 && error < 0 ){
    //     KP=setKP*0.05F;
    //   }else{
    //     KP=setKP;
    //   }
    // }
    if(error >= 0){
      if(error > 2350){
        KP=setKP*5.5F;
      }else if(error > 1500  && error <= 2350){
        KP=setKP*4.25F;
      }else if(error > 1000  && error <= 1500){
        KP=setKP*3.25F;
      }else if(error > 500  && error <= 1000){
        KP=setKP*2.35F;
      }else if(error > 350  && error <= 500){
        KP=setKP*1.6F;
      }else if(error > 150  && error <= 350){
        KP=setKP*1.3F;
      }else if(error > 120  && error <= 150){
        KP=setKP*1.0F;
      }else if(error > 65  && error <= 120){
        KP=setKP*0.75F;
      }else if(error > 35  && error <= 65){
        KP=setKP*0.55F;
      }else if(error > 10 && error <= 35){
        KP=setKP*0.1F;
      }else if(error > 5 && error <= 10){
        KP=setKP*0.05F;
      }else if(error >= 0 && error <= 5){
        KP=setKP*0.01F;
      }else{
        KP=setKP;
      }
    }else{
      if(error < -500){
        KP=setKP*5.0F;
      }else if(error >= -500 && error < -150){
        KP=setKP*1.35F;
      }else if(error >= -150 && error < -120){
        KP=setKP*0.9F;
      }else if(error >= -120 && error < -65){
        KP=setKP*0.65F;
      }else if(error >= -65 && error < -35){
        KP=setKP*0.35F;
      }else if(error >= -35 && error < -10 ){
        KP=setKP*0.1F;
      }else if(error >= -10 && error < 5 ){
        KP=setKP*0.05F;
      }else if(error >= -5 && error < 0 ){
        KP=setKP*0.01F;
      }else{
        KP=setKP;
      }
    }

    //Calculate output Proportional
    output = KP * error ; //Kontrol Proportional

    // time_now =  SysTick->VAL/98666666; //CCLK = 98,6MHz
    // time_change = time_now - time_prev;
    // errSum += (error * time_change);
    // dErr = (error - prev_error) / time_change;
    // time_prev = time_now;
    // output = KP * error + KI * errSum + KD * dErr;

    /*
      F = 50hz
      Pulse_width Buka  1800000 - 1100000  
      Pulse_width Tutup 780000 - 0
    */
    if(output >= 0){
      //PWM buka
      //f(x) = 21.013448x+151455.311355 unt 400hz
      //f(x) = 71.232029x+650695.970696 unt 100hz
      //f(x) = 199.449679x+1101948.717948 unt 50hz
      Pulse_width = (int) (199.449679F*output+1101948.717948F);
    }else if(output < 0){
      //PWM tutup
      //f(x) = 26.725239x+74021.096838 unt 400hz
      //f(x) = 101.646503x+281531.086957 unt 100hz
      //f(x) = 282.653376x+782867.193675 unt 50hz

      //f(x) = 320.030597x+803225.6
      Pulse_width = (int) (320.030597F*output+803225.6F);
    }

    if(Pulse_width >= max_pwm){
      //chprintf((BaseSequentialStream *)&SD3,"Pulse_width > maks, Pulse_width = %d\r\n", Pulse_width);
      Pulse_width = max_pwm;
    }
    else if(Pulse_width <= min_pwm){
      //chprintf((BaseSequentialStream *)&SD3,"Pulse_width < min, Pulse_width = %d\r\n", Pulse_width);
      Pulse_width = min_pwm;
    }
    prev_error = error;
    //------------------------------------------------------------------
    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//
    LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    LPC_MCPWM->MAT1 = Pulse_width;          //
    LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt
    if(count_to_print>=20){//delay loop = 250ms -> sehingga diprint ke serial tiap 1 sekon
      chprintf((BaseSequentialStream *)&SD3,"////////////////////////////////////\r\n");
      chprintf((BaseSequentialStream *)&SD3,"Set Point (mv) : %d\r\n", target_pos);
      chprintf((BaseSequentialStream *)&SD3,"Voltage TPS (mv) : %d\r\n", current_pos);
      //chprintf((BaseSequentialStream *)&SD3,"rawbit TPS : %d\r\n", rawbit_tps);
      chprintf((BaseSequentialStream *)&SD3,"command_pos (mv) : %d\r\n", command_pos);
      chprintf((BaseSequentialStream *)&SD3,"---------------------------------\r\n");
      chprintf((BaseSequentialStream *)&SD3,"Error : %d\r\n", error);
      chprintf((BaseSequentialStream *)&SD3,"output PID : %d\r\n", output);
      chprintf((BaseSequentialStream *)&SD3,"Pulse_width : %d\r\n", Pulse_width);
      //count_to_print=0;
    }
    count_to_print++;
    chThdSleepMilliseconds(50);//nilai delay juga berpengaruh

    //Tuning pwm
    // LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    // LPC_MCPWM->MAT1 = 2000000;//Pulse_width;          //
    // LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt
    // chThdSleepMilliseconds(200);
    // LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    // LPC_MCPWM->MAT1 = 750000;//Pulse_width;          //
    // LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt
    // chThdSleepMilliseconds(5000);
  }
}

/*
 * Low speed SPI configuration (328.125kHz, CPHA=0, CPOL=0, MSb first).
 */
const SPIConfig ls_spicfg_cs1 = {
  false,
  GPIO2,
  1,
  CR0_DSS8BIT | CR0_FRFSPI | CR0_CLOCKRATE(8),
  0
};
uint16_t ReadThermocouple(SPIConfig spicfg){
  uint8_t rxbuf[2] = {0,0};
  spiAcquireBus(&SPID2); /* Acquire ownership of the bus.    */
  spiStart(&SPID2, &spicfg);
  spiSelect(&SPID2); /* Slave Select assertion.          */
  spiReceive(&SPID2, 2, rxbuf);
  spiUnselect(&SPID2); /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID2); /* Ownership release.               */
  uint16_t i = (rxbuf[0] << 8)|(rxbuf[1]);
  i = (i >> 3 ) *0.25; // *0.25degC
  return i;
}
/*
 * Measure temperature oil using thermocouple sensor & max6675
 * SPI1 -> driver SPID2
 * MISO1        -   P0.8
 * SCK1         -   P0.7
 * SSEL1/CS1    -   P2.1
 * SSEL2/CS2    -   P2.2
 * SSEL3/CS3    -   P2.3
 */
static msg_t thd_temp_oil(void *arg) {

  (void)arg;
  chRegSetThreadName("temp_oil");
  LPC_SC->PCONP |= (0 << 8);// Reg. PCSP1 bit 8 -> Set power on sp1
  LPC_SC->PCLKSEL0 |= (0 << 17) | (1 << 16);//Reg. PCLK_SPI 17:16 -> Set PCLK = CCLK (100MHz)
  
  LPC_PINCON->PINSEL0 |= (1 << 15) | ( 0 << 14) ; // Reg. PINSEL0 bit 15:14 -> Set P0.7 sbg SCK1
  LPC_PINCON->PINMODE0 |= (1 << 15) | ( 1 << 14) ; // Reg. PINMODE0 bit 15:14 -> Set P0.7 pull down resistor
  LPC_GPIO0->FIODIR0 |= ( 1 << 7) ; //Reg. FIODIR0 P0.7 bit 7 -> Set P0.7 sebagai output

  LPC_PINCON->PINSEL0 |= (1 << 17) | ( 0 << 16) ; // Reg. PINSEL0 bit 17:16 -> Set P0.8 sbg MISO1
  LPC_PINCON->PINMODE0 |= (1 << 17) | ( 1 << 16) ; // Reg. PINMODE0 bit 17:16 -> Set P0.8 pull down resistor
  LPC_GPIO0->FIODIR1 |= (0 << 0) ; //Reg. FIODIR1 P0.8 bit 0 -> Set P0.8 sebagai input
  
  LPC_PINCON->PINSEL4 |= (0 << 3) | ( 0 << 2) ; // Reg. PINSEL0 bit 17:16 -> Set P2.1 sbg GPIO (SSEL_temp1)
  LPC_PINCON->PINMODE4 |= (1 << 3) | ( 1 << 2) ; // Reg. PINMODE0 bit 15:14 -> Set P0.7 pull down resistor
  LPC_GPIO2->FIODIR0 |= (1 << 1); //Reg. FIODIR0 P2.1 bit 1 -> Set P2.1 sebagai output
  // LPC_GPIO2->FIOSET0 |= (1 << 1); //Set P2.1 HIGH
  while (TRUE) {
    uint16_t temp1 = ReadThermocouple(ls_spicfg_cs1);
    //uint16_t temp2 = ReadThermocouple(ls_spicfg_cs2);
    //chprintf((BaseSequentialStream *)&SD3,"Temp 1 (C) : %.2d \r\n\r\n", temp1);
    //chprintf((BaseSequentialStream *)&SD3,"Temp 2 (C) : %.2d \r\n\r\n", temp2);
    chThdSleepMilliseconds(500);
  }
}
/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  chThdSleepMilliseconds(1000);
  /*
   * Activates the SD1 and SPI1 drivers.
   */
  //UART2 = SD3
  sdStart(&SD3, &uart2_cfg); /* Default: 38400,8,N,1.            */
  chprintf((BaseSequentialStream *)&SD3,"Debug Via Serial Using UART2 \r\n");
  chThdSleepMilliseconds(500);

  /*
   * Activates the CAN drivers 1 and 2.
   */
  canStart(&CAND1, &cancfg);
  // #if LPC17xx_CAN_USE_FILTER
  //   canSetFilter(&canfcfg);
  // #endif  


  /* Creates the blinker threads.
   */
  //chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+3, thd_pwm_servo, NULL);
  //chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+3, thd_servo, NULL);
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, thd_led_debug, NULL);
  //chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO+1, thd_temp_oil, NULL);
  //chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO+5, thd_tps, NULL);
  //chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7, can_rx, (void *)&can1);
  //chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7,can_rx, (void *)&can2);
  chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO+7, thd_can_tx, NULL);
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO+5, thd_test_inj_GPIO, NULL);
  
  // chprintf((BaseSequentialStream *)&SD3,"LPC17 SYS_CLK : %d\r\n", LPC17xx_CCLK);
  // chprintf((BaseSequentialStream *)&SD3,"LPC17xx_PCLK : %d\r\n", LPC17xx_PCLK);
  // chprintf((BaseSequentialStream *)&SD3,"LPC17xx_PCLKSEL0_REGISTER : %d\r\n", LPC_SC->PCLKSEL0);
  // chprintf((BaseSequentialStream *)&SD3,"LPC17xx_PCLKSEL1_REGISTER : %d\r\n", LPC_SC->PCLKSEL1);
  
  //init_ADC();
  int i = 0;
  //chprintf((BaseSequentialStream *)&SD3,"Ready to Launch\r\n");
  while (TRUE) {
    //chprintf((BaseSequentialStream *)&SD3,"Iterasi : %d\r\n", i++);
    chThdSleepMilliseconds(5000);
  }
}
