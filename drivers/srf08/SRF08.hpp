/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC

 ***************************************************************************/
#pragma once
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include "DriverFramework.hpp"
#include "I2CDevObj.hpp"


#define SRF08_CLASS_PATH  "/dev/srf08"
#define SRF08_DEVICE_PATH "/dev/i2c-1"

#define SRF08_RA_CMD           0x00
#define SRF08_RA_LIGHT           0x01
#define SRF08_RA_ECHO_H       0x02
#define SRF08_RA_ECHO_L       0x03
#define SRF08_RA_ECHO_L_MAX 0x23

#define SRF08_START_RANGE_CMD       0x52
#define SRF08_SAMPLE_PERIOD_US 200000

#define SRF08_BUS_FREQUENCY_IN_KHZ 100
#define SRF08_TRANSFER_TIMEOUT_IN_USECS 9000
#define SRF08_SLAVE_ADDRESS 0x72

namespace DriverFramework
{
#define DRV_DF_DEVTYPE_SRF08 0x53

class SRF08 : public I2CDevObj
{
public:
    SRF08() :
        I2CDevObj("SRF08", SRF08_DEVICE_PATH, SRF08_CLASS_PATH, SRF08_SAMPLE_PERIOD_US),
        _last_us(0)
    {
        m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_SRF08;
        m_id.dev_id_s.address = SRF08_SLAVE_ADDRESS;
    }

    int start();
    int stop();

protected:
    SyncObj         m_synchronize;

private:
    void _measure();
    void _startRanging();
    virtual void _publish(double range) {}
    uint32_t _last_us;
};

} // namespace DriverFramework
