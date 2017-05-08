/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include "DriverFramework.hpp"
#include "I2CDevObj.hpp"
#include "../common/common.hpp"


#define PWM_CLASS_PATH  "/dev/pwm"
#define PWM_DEVICE_PATH "/dev/i2c-1"

#define PCA9685_RA_MODE1           0x00
#define PCA9685_RA_MODE2           0x01
#define PCA9685_RA_LED0_ON_L       0x06
#define PCA9685_RA_LED0_ON_H       0x07
#define PCA9685_RA_LED0_OFF_L      0x08
#define PCA9685_RA_LED0_OFF_H      0x09
#define PCA9685_RA_ALL_LED_ON_L    0xFA
#define PCA9685_RA_ALL_LED_ON_H    0xFB
#define PCA9685_RA_ALL_LED_OFF_L   0xFC
#define PCA9685_RA_ALL_LED_OFF_H   0xFD
#define PCA9685_RA_PRE_SCALE       0xFE

#define PCA9685_MODE1_RESTART_BIT  (1 << 7)
#define PCA9685_MODE1_EXTCLK_BIT   (1 << 6)
#define PCA9685_MODE1_AI_BIT       (1 << 5)
#define PCA9685_MODE1_SLEEP_BIT    (1 << 4)
#define PCA9685_MODE1_SUB1_BIT     (1 << 3)
#define PCA9685_MODE1_SUB2_BIT     (1 << 2)
#define PCA9685_MODE1_SUB3_BIT     (1 << 1)
#define PCA9685_MODE1_ALLCALL_BIT  (1 << 0)
#define PCA9685_ALL_LED_OFF_H_SHUT (1 << 4)
#define PCA9685_MODE2_INVRT_BIT    (1 << 4)
#define PCA9685_MODE2_OCH_BIT      (1 << 3)
#define PCA9685_MODE2_OUTDRV_BIT   (1 << 2)
#define PCA9685_MODE2_OUTNE1_BIT   (1 << 1)
#define PCA9685_MODE2_OUTNE0_BIT   (1 << 0)
#define PCA9685_CHAN_COUNT 16
#define PCA9685_ENABLE_PIN 27

#define PCA9685_BUS_FREQUENCY_IN_KHZ 400
#define PCA9685_FREQUENCY_PWM 400
#define PCA9685_TRANSFER_TIMEOUT_IN_USECS 9000
#define PCA9685_CHAN_OFFSET 11
#define PCA9685_SLAVE_ADDRESS 0x40
#define PCA9685_INTERNAL_CLOCK (1.04f * 25000000.f)
#define PCA9685_EXTERNAL_CLOCK 24576000.f

namespace DriverFramework
{
#define DRV_DF_DEVTYPE_PCA9685 0x51

class PCA9685 : public I2CDevObj
{
public:
    PCA9685() :
        I2CDevObj("PWM", PWM_DEVICE_PATH, PWM_CLASS_PATH, 0),
        m_armed(false),
        m_enableGPIO(new GPIO(PCA9685_ENABLE_PIN))
    {
        m_osc_clock = PCA9685_EXTERNAL_CLOCK;
        m_pulses_buffer = new uint16_t[PCA9685_CHAN_COUNT - PCA9685_CHAN_OFFSET];
        m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_PCA9685;
        m_id.dev_id_s.address = PCA9685_SLAVE_ADDRESS;
    }
    ~PCA9685(){
        if (m_enableGPIO != nullptr) {
            m_enableGPIO->disable();
            delete m_enableGPIO;
        }
        delete m_enableGPIO;
        delete [] m_pulses_buffer;
    }

    int start();
    int stop();

    void write(uint8_t ch, uint16_t period_us);

    void push();

    virtual void _measure(void) {}

protected:
    SyncObj         m_synchronize;

private:

    int _setFrequency(uint16_t freq_hz);
    int _resetAll();

    float           m_osc_clock;
    uint16_t        m_frequency;
    uint16_t        m_pending_write_mask;
    bool            m_armed;
    uint16_t*       m_pulses_buffer;
    GPIO*           m_enableGPIO;
};

} // namespace DriverFramework
