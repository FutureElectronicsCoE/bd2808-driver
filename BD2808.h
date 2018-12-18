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
#ifndef MBED_BD2808_H
#define MBED_BD2808_H

#include "platform/platform.h"

#include "platform/PlatformMutex.h"
#include "platform/SingletonPtr.h"
#include "platform/NonCopyable.h"
#include "hal/dma_api.h"
#include "bd2808_hal.h"


struct __PACKED BGR24_color_t {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    BGR24_color_t() :
        r(0), g(0), b(0)  {}

    BGR24_color_t(uint8_t _b, uint8_t _g, uint8_t _r) :
        r(_r), g(_g), b(_b)  {}
};


namespace mbed {


class BD2808 : private NonCopyable<BD2808> {

public:

    /** Create a BD2808 driver.
     *
     *  @note Pins used by the driver are fixed in the implementation.
     *
     */
    BD2808();
    virtual ~BD2808();

    /** Set the bus clock frequency.
     *
     *  @param hz Clock frequency in Hz (default = 1MHz).
     *
     *  @note Maximum frequency is 9MHz.
     */
    void frequency(int hz = 1000000);

    /** Set maximum LED current.
     *
     *  @param .
     *
     *  @note Maximum possible current is 12.8 mA, but not more than 2.5 mA
     *        should be used when powering board from USB.
     */
    void max_current(float red_current, float green_current, float blue_current);

    /** Set LED color to be used with next refresh cycle.
     *
     *  @param led LED index.
     *  @param value LED color to be set.
     */
    void set_color(int led_id, BGR24_color_t value);

    /** Manually refresh all LEDs.
     *
     */
    void refresh(void);

    /** Enable/disable auto-refresh mode.
     *
     *  @param enable Enable or disable.
     */
    void auto_refresh(bool enable);

    /** Configure DMA usage suggestion for non-blocking transfers.
     *
     *  @param usage The usage DMA hint for peripheral.
     *
     *  @return Result of the operation.
     *  @retval 0 The usage was set.
     *  @retval -1 Usage cannot be set as there is an ongoing transaction.
     */
    int set_dma_usage(DMAUsage usage);

#if !defined(DOXYGEN_ONLY)

protected:
    /* Internal SPI object identifying the resources */
    bd2808_t _serial_dev;

    DMAUsage _usage;
    /* Current sate of the sleep manager */
    bool _deep_sleep_locked;

    /* Take over the physical SPI and apply our settings (thread safe) */
    void aquire(void);
    /* Current user of the SPI */
    static BD2808 *_owner;
    /* Used by lock and unlock for thread safety */
    static SingletonPtr<PlatformMutex> _mutex;
    /* Clock frequency */
    int _hz;
    /* refresh data buffer */
    uint8_t command_buffer[30];

private:
    /** Private acquire function without locking/unlocking.
     *  Implemented in order to avoid duplicate locking and boost performance.
     */
    void _acquire(void);

#endif //!defined(DOXYGEN_ONLY)
};

} // namespace mbed

#endif //MBED_BD2808_H
