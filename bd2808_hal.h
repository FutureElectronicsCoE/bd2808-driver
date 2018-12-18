/*
 * Low level Mbed OS driver for ROHM BD2808 24-channel LED Controller
 * on Future Electronics Sequana board (PSoC 6 platform).
 *
 * Copyright (c) 2018 Future Electronics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __BD2808_HAL_H__
#define __BD2808_HAL_H__

#include <stdint.h>
#include <cmsis.h>
#include "objects.h"

/* BD2808 registers */
#define BD2808_REG_DEVADDR      0       // device address
#define BD2808_REG_STATUS       1       // status
#define BD2808_REG_MODE         2       // mode
#define BD2808_REG_DAR          3       // current setup (DAC) red channel
#define BD2808_REG_DAG          4       // current setup (DAC) green channel
#define BD2808_REG_DAB          5       // current setup (DAC) blue channel
#define BD2808_REG_BR_BASE      6       // brightness (PWM) base address
#define BD2808_RED              0       // red register offset
#define BD2808_GREEN            1       // green register offset
#define BD2808_BLUE             2       // blue register offset
#define BD2808_REG_BR(chn, color)       (BD2808_REG_BR_BASE + 3*(chn) + color)

/* BD2808 MODE bits */
#define BD2808_MODE_SLEEP       0       // go to sleep after register address 1E
#define BD2808_MODE_DAC         1       // return to DAR after address 1E
#define BD2808_MODE_BR          2       // return to BRR0 after address 1E
#define BD2808_MODE_ENMD        0x04    // latch brightness immediately
#define BD2808_MODE_SFTRST      0x08    // soft reset

#define BD2808_NUM_CHANNELS     8

// Converts max LED current in mA into DAC configuration value,
// assumes argument is a floating point value.
#define mA(x)   ((uint8_t)((x) * 200/40 - 1))

#define BD2808_MAX_LED_CURRENT  12.8


#ifdef __cplusplus
extern "C" {
#endif

typedef struct spi_s bd2808_t;

void bd2808_send_command(uint8_t *command, uint32_t size, bool use_dma);
void bd2808_init(bd2808_t* obj);

#ifdef __cplusplus
}
#endif


#endif //__BD2808_HAL_H__
