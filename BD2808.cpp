/*
 * Mbed OS device driver for ROHM BD2808 24-channel LED Controller
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

#include "BD2808.h"
#include "platform/mbed_critical.h"
#include "platform/mbed_wait_api.h"
#include "bd2808_hal.h"

namespace mbed {


#define LED_CURRENT_RED         mA(2)
#define LED_CURRENT_GREEN       mA(2)
#define LED_CURRENT_BLUE        mA(2)


uint8_t reset_command[] = {
    0xff, 0xff,                 // wakeup sequence
    0x00,                       // device address
    BD2808_REG_MODE,            // config starts with MODE register
    BD2808_MODE_SFTRST,
    0xff
};

uint8_t init_command[] = {
    0xff, 0xff,                 // wakeup sequence
    0x00,                       // device address
    BD2808_REG_MODE,            // config starts with MODE register
    BD2808_MODE_SLEEP | BD2808_MODE_ENMD,
    LED_CURRENT_RED,
    LED_CURRENT_GREEN,
    LED_CURRENT_BLUE,
    0xff, 0xf7                  // sleep
};

#define RED_CURRENT_OFFSET      5
#define GREEN_CURRENT_OFFSET    6
#define BLUE_CURRENT_OFFSET     7


uint8_t display_command[] = {
    0xff, 0xff,
    0x00,                       // device address
    BD2808_REG_BR_BASE,         // starts with BRR0 register
    0, 0, 0,                    // R/G/B chn #0
    0, 0, 0,                    // R/G/B chn #1
    0, 0, 0,                    // R/G/B chn #2
    0, 0, 0,                    // R/G/B chn #3
    0, 0, 0,                    // R/G/B chn #4
    0, 0, 0,                    // R/G/B chn #5
    0, 0, 0,                    // R/G/B chn #6
    0, 0, 0,                    // R/G/B chn #7
    0xff, 0xf7                  // sleep
};

#define COLOR_OFFSET            4


BD2808 *BD2808::_owner = NULL;

SingletonPtr<PlatformMutex> BD2808::_mutex;


BD2808::BD2808() :
    _serial_dev(),
    _usage(DMA_USAGE_NEVER),
    _deep_sleep_locked(false),
    _hz(1000000)
{
    bd2808_init(&_serial_dev);

    /* reset chip */
    wait_ms(10);
    bd2808_send_command(reset_command, sizeof(reset_command), _usage != DMA_USAGE_NEVER);
    /* initialize chip */
    wait_ms(5);
    bd2808_send_command(init_command, sizeof(init_command), _usage != DMA_USAGE_NEVER);
}

BD2808::~BD2808()
{
   if (_owner == this) {
        _owner = NULL;
    }
}

void BD2808::frequency(int hz)
{

}

void BD2808::max_current(float red_current, float green_current, float blue_current)
{
    if (red_current <= BD2808_MAX_LED_CURRENT) {
        init_command[RED_CURRENT_OFFSET] = mA(red_current);
    }
    if (green_current <= BD2808_MAX_LED_CURRENT) {
        init_command[GREEN_CURRENT_OFFSET] = mA(green_current);
    }
    if (blue_current <= BD2808_MAX_LED_CURRENT) {
        init_command[BLUE_CURRENT_OFFSET] = mA(blue_current);
    }

    bd2808_send_command(init_command, sizeof(init_command), _usage != DMA_USAGE_NEVER);
}


#define limit(c)    ((c == 0)? 0 : c - 1)

void BD2808::set_color(int led_id, BGR24_color_t value)
{
    if ((led_id >= 0) && (led_id < BD2808_NUM_CHANNELS)) {
        uint32_t byte_idx = 3 * led_id + COLOR_OFFSET;
        display_command[byte_idx] = limit(value.r);
        display_command[byte_idx + 1] = limit(value.g);
        display_command[byte_idx + 2] = limit(value.b);
    }
}


void BD2808::refresh(void)
{
    bd2808_send_command(display_command, sizeof(display_command), _usage != DMA_USAGE_NEVER);
}

void BD2808::auto_refresh(bool enable)
{

}

int BD2808::set_dma_usage(DMAUsage usage)
{
    _usage = usage;
    return 0;
}

} // namespace mbed
