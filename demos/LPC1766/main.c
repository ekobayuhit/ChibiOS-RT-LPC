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
#include "chprintf.h" //Bayu 4 Feb 2019
#include <stdlib.h> //Bayu 4 Feb 2019


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
    chThdSleepMilliseconds(1000);
  }
}
/*
 * PWM-Servo thread dengan MCPWM pin
 * PIN P1.25
 */
static WORKING_AREA(waThread2, 256);
static msg_t thd_pwm_servo(void *arg) {

  (void)arg;
  chRegSetThreadName("Servo_throttle_gas");
  LPC_SC->PCONP |= (1 << 17);                     // Reg. PCMCPWM bit 17 -> Set Power on
  LPC_SC->PCLKSEL1 |= (0 << 31) | (1 << 30) ;     // Reg. PCLK_MC bit 31:30 -> Set PCLK = CCLK (100MHz)
  LPC_PINCON->PINSEL3 |= (0 << 19) | ( 1 << 18) ; // Reg. PINSEL3 bit 19:18 -> Set P1.25 sbg MCOA1
  LPC_PINCON->PINMODE3 |= (1 << 19) | (1 <<18) ;  // Reg. PINMODE3 bit 18:19 -> Set P1.25 mode pull down resistor 
  LPC_GPIO1->FIODIR3 |= (1 << 1);                 // Reg. FIODIR3 Bit 1 -> P1.25 set sbg OUTPUT

  // Set the Limit register
  /*cara hitung limit untuk freq 50 hz
    fclock (PCLK) = 100Mhz -> 1 clock = 10ns ->untuk jadi 20ms(50hz) maka harus dikalikan 2000000
    nah 2000000 itu dijadikan limit tim/cnt (LIM1=2000000)
  */
  LPC_MCPWM->LIM1 = 2000000; //limit untuk freq 50hz(20ms) && PCLK=100Mhz

  while (TRUE) {
    LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    LPC_MCPWM->MAT1 = 60000;          //Set Duty Cycle 0 derajat = 0.6ms (3%) -> 3% dari 2000000 = 60000
    LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt
    chThdSleepMilliseconds(5000);
    LPC_MCPWM->CON_CLR = 0xffffffff;  //Reset Value reg. MCCON_CLR
    LPC_MCPWM->MAT1 = 200000;         //Set Duty Cycle 0 derajat = 2 ms (10%) -> 10% dari 2000000 = 200000
    LPC_MCPWM->CON_SET = (1<<8);      //Bit ke-8 untuk start timer/cnt ; 0=STOP ; 1=START
    chThdSleepMilliseconds(5000);
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
 * MISO1  -   P0.8
 * SCK1   -   P0.7
 * CS1    -   P2.1
 * CS2    -   P2.2
 * CS3    -   P2.3
 */
static WORKING_AREA(waThread3, 256);
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
    chprintf((BaseSequentialStream *)&SD3,"Temp 1 (C) : %.2d \r\n\r\n", temp1);
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
  chThdSleepMilliseconds(1000);
  /* Creates the blinker threads.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+5, thd_led_debug, NULL);
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+3, thd_pwm_servo, NULL);
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO+4, thd_temp_oil, NULL);
  /*
   * Normal main() thread activity, in this demo it updates the 7-segments
   * display on the LPCXpresso main board using the SPI driver.
   */
	int i = 0;
  while (TRUE) {         
    chThdSleepMilliseconds(1000);
	}
}
