#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "DriverFramework.hpp"
#include "PCA9685.hpp"
#include "../common/common.hpp"

namespace DriverFramework
{

int PCA9685::start() {

    int result = I2CDevObj::start();
    if (result < 0) {
        DF_LOG_ERR("could not start i2c device for PWM");
    }

    result = _setSlaveConfig(PCA9685_SLAVE_ADDRESS,
                     PCA9685_BUS_FREQUENCY_IN_KHZ,
                     PCA9685_TRANSFER_TIMEOUT_IN_USECS);
    if (result < 0) {
        DF_LOG_ERR("could not set slave config for PWM");
    }
    
    result = _resetAll();
    if (result < 0) {
        DF_LOG_ERR("could not reset PWM outputs");
    }
    /* Set the initial frequency */
    _setFrequency(PCA9685_FREQUENCY_PWM);

    // Output enable - active low (use Linux filesystem to write)
    m_enableGPIO->enable();
    m_enableGPIO->setDirection(GPIO::OUTPUT);
    m_enableGPIO->setValue(GPIO::LOW);

    return 0;
}

int PCA9685::stop(){
    if (m_enableGPIO != nullptr) {
        m_enableGPIO->disable();
    }    
    return 0;
}

void PCA9685::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= (PCA9685_CHAN_COUNT - PCA9685_CHAN_OFFSET)) {
        return;
    }

    m_synchronize.lock();
    m_pulses_buffer[ch] = period_us;
    m_pending_write_mask |= (1U << ch);
    m_synchronize.unlock();
}

void PCA9685::push() {

    if (m_pending_write_mask == 0)
        return;

    // Calculate the number of channels for this transfer.
    uint8_t max_ch = (sizeof(unsigned) * 8) - __builtin_clz(m_pending_write_mask);
    uint8_t min_ch = __builtin_ctz(m_pending_write_mask);

    /*
     * scratch buffer size is always for all the channels, but we write only
     * from min_ch to max_ch
     */
    uint8_t data[PCA9685_CHAN_COUNT * 4] = { };

    for (unsigned ch = min_ch; ch < max_ch; ch++) {
        uint16_t period_us = m_pulses_buffer[ch];
        uint16_t length = 0;

        if (period_us)
            length = round((period_us * 4096) / (1000000.f / PCA9685_FREQUENCY_PWM)) - 1;

        uint8_t *d = &data[ch * 4];
        *d++ = 0;
        *d++ = 0;
        *d++ = length & 0xFF;
        *d++ = length >> 8;
    }

    //is kernel will care i2c lock?
    _writeReg(PCA9685_RA_LED0_ON_L + 4 * (PCA9685_CHAN_OFFSET + min_ch),
              &data[min_ch * 4], (max_ch - min_ch) * 4);

    m_synchronize.lock();
    m_pending_write_mask = 0;
    m_synchronize.unlock();
}


int PCA9685::_resetAll() {

    uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
    int ret = _writeReg(PCA9685_RA_ALL_LED_ON_L, data, 4);

    /* Wait for the last pulse to end */
    usleep(2);
    return ret;
}

int PCA9685::_setFrequency(uint16_t freq_hz) {
    uint8_t cmd[1];
    /* Correctly finish last pulses */
    for (int i = 0; i < (PCA9685_CHAN_COUNT - PCA9685_CHAN_OFFSET); i++) {
        write(i, m_pulses_buffer[i]);
    }

    /* Shutdown before sleeping.
     * see p.14 of PCA9685 product datasheet
     */
    cmd[0] = PCA9685_ALL_LED_OFF_H_SHUT;
    _writeReg(PCA9685_RA_ALL_LED_OFF_H, cmd, 1);

    /* Put PCA9685 to sleep (required to write prescaler) */
    cmd[0] = PCA9685_MODE1_SLEEP_BIT;
    _writeReg(PCA9685_RA_MODE1, cmd, 1);

    /* Calculate prescale and save frequency using this value: it may be
     * different from @freq_hz due to rounding/ceiling. We use ceil() rather
     * than round() so the resulting frequency is never greater than @freq_hz
     */
    uint8_t prescale = ceil(m_osc_clock / (4096 * PCA9685_FREQUENCY_PWM)) - 1;
    m_frequency = m_osc_clock / (4096 * (prescale + 1));

    /* Write prescale value to match frequency */
    cmd[0] = prescale;
    _writeReg(PCA9685_RA_PRE_SCALE, cmd, 1);

    /* Enable external clocking */
    cmd[0] = PCA9685_MODE1_SLEEP_BIT | PCA9685_MODE1_EXTCLK_BIT;
    _writeReg(PCA9685_RA_MODE1, cmd, 1);

    /* Restart the device to apply new settings and enable auto-incremented write */
    cmd[0] = PCA9685_MODE1_RESTART_BIT | PCA9685_MODE1_AI_BIT;
    _writeReg(PCA9685_RA_MODE1, cmd, 1);

    return 0;
}

} //end namespace