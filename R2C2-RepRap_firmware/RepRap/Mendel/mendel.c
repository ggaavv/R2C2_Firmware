/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdint.h>
#include <stdlib.h>

#include "lpc17xx_timer.h"
#include "lpc17xx_wdt.h"
#include "lpc17xx_adc.h"
#include "r2c2.h"

#include "machine.h"
#include "gcode_parse.h"
#include "gcode_process.h"
#include "pinout.h"
#include "debug.h"
#include "config.h"
#include "temp.h"

#include "planner.h"
#include "stepper.h"


tTimer temperatureTimer;

tLineBuffer serial_line_buf;
tLineBuffer sd_line_buf;

/* Initialize ADC for reading sensors */
void adc_init(void)
{
  PINSEL_CFG_Type PinCfg;

  PinCfg.Funcnum = PINSEL_FUNC_1; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
  PinCfg.Portnum = EXTRUDER_0_SENSOR_ADC_PORT;
  PinCfg.Pinnum = EXTRUDER_0_SENSOR_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);
  pin_mode(EXTRUDER_0_SENSOR_ADC_PORT, EXTRUDER_0_SENSOR_ADC_BIT, INPUT);

  PinCfg.Funcnum = PINSEL_FUNC_1; /* ADC function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
//  PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
  PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
  PinCfg.Portnum = HEATED_BED_0_ADC_PORT;
  PinCfg.Pinnum = HEATED_BED_0_ADC_PIN;
  PINSEL_ConfigPin(&PinCfg);
  pin_mode(HEATED_BED_0_ADC_PORT, EXTRUDER_0_SENSOR_ADC_BIT, INPUT);

  ADC_Init(LPC_ADC, 200000); /* ADC conversion rate = 200Khz */
}

void io_init(void)
{
	uart_init();
  /* Extruder 0 Heater pin */
  pin_mode(EXTRUDER_0_HEATER_PORT, EXTRUDER_0_HEATER_BIT, OUTPUT);
  extruder_heater_off();

  /* Heated Bed 0 Heater pin */
  pin_mode(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_BIT, OUTPUT);
  heated_bed_off();

  /* Pull ups for all opto switches */
  PINSEL_CFG_Type PinCfg;
  PinCfg.Funcnum = PINSEL_FUNC_0; /* GPIO function */
  PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
  PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;

  PinCfg.Portnum = X_MIN_PORT;
  PinCfg.Pinnum = X_MIN_PIN;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Portnum = X_MAX_PORT;
  PinCfg.Pinnum = X_MAX_PIN;
  PINSEL_ConfigPin(&PinCfg);

  PinCfg.Portnum = Y_MIN_PORT;
  PinCfg.Pinnum = Y_MIN_PIN;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Portnum = Y_MAX_PORT;
  PinCfg.Pinnum = Y_MAX_PIN;
  PINSEL_ConfigPin(&PinCfg);

  PinCfg.Portnum = Z_MIN_PORT;
  PinCfg.Pinnum = Z_MIN_PIN;
  PINSEL_ConfigPin(&PinCfg);
  PinCfg.Portnum = Z_MAX_PORT;
  PinCfg.Pinnum = Z_MAX_PIN;
  PINSEL_ConfigPin(&PinCfg);

  /* setup I/O pins */
//  pin_mode(STEPPERS_RESET_PORT, STEPPERS_RESET_PIN, OUTPUT);
//  digital_write(STEPPERS_RESET_PORT, STEPPERS_RESET_BIT, 1); /* Disable reset for all stepper motors */

  pin_mode(X_STEP_PORT, X_STEP_BIT, OUTPUT);
  pin_mode(X_DIR_PORT, X_DIR_BIT, OUTPUT);
  pin_mode(X_ENABLE_PORT, X_ENABLE_BIT, OUTPUT);
  x_disable();
  pin_mode(X_MIN_PORT, X_MIN_BIT, INPUT);
  pin_mode(X_MAX_PORT, X_MAX_BIT, INPUT);

  pin_mode(Y_STEP_PORT, Y_STEP_BIT, OUTPUT);
  pin_mode(Y_DIR_PORT, Y_DIR_BIT, OUTPUT);
  pin_mode(Y_ENABLE_PORT, Y_ENABLE_BIT, OUTPUT);
  y_disable();
  pin_mode(Y_MIN_PORT, Y_MIN_BIT, INPUT);
  pin_mode(Y_MAX_PORT, Y_MAX_BIT, INPUT);

  pin_mode(Z_STEP_PORT, Z_STEP_BIT, OUTPUT);
  pin_mode(Z_DIR_PORT, Z_DIR_BIT, OUTPUT);
  pin_mode(Z_ENABLE_PORT, Z_ENABLE_BIT, OUTPUT);
  z_disable();
  pin_mode(Z_MIN_PORT, Z_MIN_BIT, INPUT);
  pin_mode(Z_MAX_PORT, Z_MAX_BIT, INPUT);

  pin_mode(E_STEP_PORT, E_STEP_BIT, OUTPUT);
  pin_mode(E_DIR_PORT, E_DIR_BIT, OUTPUT);
  pin_mode(E_ENABLE_PORT, E_ENABLE_BIT, OUTPUT);
  e_disable();

  pin_mode(EXTRUDER_0_FAN_PORT, EXTRUDER_0_FAN_BIT, OUTPUT);
  pin_mode(HEATED_BED_0_HEATER_PORT, HEATED_BED_0_HEATER_BIT, OUTPUT);

  adc_init();
}

void temperatureTimerCallback (tTimer *pTimer)
{
  /* Manage the temperatures */
  temp_tick();
}

void check_boot_request (void)
{
  if (digital_read (4, (1<<29)) == 0)
  {
    WDT_Init (WDT_CLKSRC_PCLK, WDT_MODE_RESET);
    WDT_Start (10);
    while (1);
  }
}

void init(void)
{
  // set up inputs and outputs
  io_init();

  /* Initialize Gcode parse variables */
  gcode_parse_init();

  // set up default feedrate
//TODO  current_position.F = startpoint.F = next_target.target.F =       config.search_feedrate_z;

  AddSlowTimer (&temperatureTimer);
  StartSlowTimer (&temperatureTimer, 10, temperatureTimerCallback);
  temperatureTimer.AutoReload = 1;

  // say hi to host
  serial_writestr("Start\r\nOK\r\n");
}

int app_main (void)
{
  long timer1 = 0;
  eParseResult parse_result;

  buzzer_init();
  buzzer_play(1500, 100); /* low beep */
	buzzer_wait();
  buzzer_play(2500, 200); /* high beep */

  init();

  read_config();

  // grbl init
  plan_init();
  st_init();    
  
  // main loop
  for (;;)
  {

    // process characters from the serial port
    while (!serial_line_buf.seen_lf && (serial_rxchars() != 0) )
    {
      unsigned char c = serial_popchar();
      
      if (serial_line_buf.len < MAX_LINE)
        serial_line_buf.data [serial_line_buf.len++] = c;

      if ((c==10) || (c==13))
      {
        if (serial_line_buf.len > 1)
          serial_line_buf.seen_lf = 1;
        else
          serial_line_buf.len = 0;
      }      
    }

    // process SD file if no serial command pending
    if (!sd_line_buf.seen_lf && sd_printing)
    {
      if (sd_read_file (&sd_line_buf))
      {
          sd_line_buf.seen_lf = 1;
      } 
      else
      {
        sd_printing = false;
        serial_writestr ("Done printing file\r\n");
      }
    }

    // if queue is full, we wait
    if (!plan_queue_full())
    {
  
      /* At end of each line, put the "GCode" on movebuffer.
       * If there are movement to do, Timer will start and execute code which
       * will take data from movebuffer and generate the required step pulses
       * for stepper motors.
       */
  
      // give priority to user commands
      if (serial_line_buf.seen_lf)
      {
        parse_result = gcode_parse_line (&serial_line_buf);
        serial_line_buf.len = 0;
        serial_line_buf.seen_lf = 0;
      }
      else if (sd_line_buf.seen_lf)
      {
        parse_result = gcode_parse_line (&sd_line_buf);
        sd_line_buf.len = 0;
        sd_line_buf.seen_lf = 0;
      }

    }

    /* Do every 100ms */
    #define DELAY1 100
    if (timer1 < millis())
    {
      timer1 = millis() + DELAY1;

      /* If there are no activity during 30 seconds, power off the machine */
      if (steptimeout > (30 * 1000/DELAY1))
      {
        power_off();
      }
      else
      {
        steptimeout++;
      }
    }

#ifdef USE_BOOT_BUTTON
    // OPTION: enter bootloader on "Boot" button
    check_boot_request();
#endif

  }
}
