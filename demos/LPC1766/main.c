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
 * LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker1");
  LPC_PINCON->PINSEL4  |= ((0 << 1) | (0 << 0)); //P2.0
  LPC_GPIO2->FIODIR |= 1; //P2.0
  while (TRUE) {
    palTogglePad(GPIO2, 0); //P2,0
    chThdSleepMilliseconds(1000);
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
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+5, Thread1, NULL);
  /*
   * Normal main() thread activity, in this demo it updates the 7-segments
   * display on the LPCXpresso main board using the SPI driver.
   */
	int i = 0;
  while (TRUE) {         
    chprintf((BaseSequentialStream *)&SD3,"i = %d \r\n", i);
    i++;
    chThdSleepMilliseconds(1000);
	}
}
