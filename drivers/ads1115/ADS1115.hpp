/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

#pragma once

#include <px4_adc.h>

#include "I2CDevObj.hpp"


#define ADC_CLASS_PATH  "/dev/adc"

namespace DriverFramework
{


#define ADC_DEVICE_PATH "/dev/i2c-1"

// update frequency is 10 Hz
#define ADS1115_MEASURE_INTERVAL_US 100000 // microseconds
#define ADS1115_SLAVE_ADDRESS 0x48       /* 7-bit slave address */
#define ADS1115_CHANNEL_COUNT 6
#define DRV_DF_DEVTYPE_ADS1115 0x59

#define ADS1115_BUS_FREQUENCY_IN_KHZ 400
#define ADS1115_TRANSFER_TIMEOUT_IN_USECS 9000
#define ADS1115_ADC_READ_ADDR     	0x00
#define ADS1115_ADC_CONFIG_ADDR    0x01

#define ADS1115_OS_SHIFT            15
#define ADS1115_OS_INACTIVE         0x00 << ADS1115_OS_SHIFT
#define ADS1115_OS_ACTIVE           0x01 << ADS1115_OS_SHIFT

#define ADS1115_MUX_SHIFT           12
#define ADS1115_MUX_P0_N1           0x00 << ADS1115_MUX_SHIFT /* default */
#define ADS1115_MUX_P0_N3           0x01 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_N3           0x02 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_N3           0x03 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P0_NG           0x04 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P1_NG           0x05 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P2_NG           0x06 << ADS1115_MUX_SHIFT
#define ADS1115_MUX_P3_NG           0x07 << ADS1115_MUX_SHIFT

#define ADS1115_PGA_SHIFT           9
#define ADS1115_PGA_6P144           0x00 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_4P096           0x01 << ADS1115_PGA_SHIFT // THIS is used
#define ADS1115_PGA_2P048           0x02 << ADS1115_PGA_SHIFT // default
#define ADS1115_PGA_1P024           0x03 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P512           0x04 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256           0x05 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256B          0x06 << ADS1115_PGA_SHIFT
#define ADS1115_PGA_0P256C          0x07 << ADS1115_PGA_SHIFT

#define ADS1115_MV_6P144            0.187500f
#define ADS1115_MV_4P096            0.125000f // THIS is used
#define ADS1115_MV_2P048            0.062500f // default
#define ADS1115_MV_1P024            0.031250f
#define ADS1115_MV_0P512            0.015625f
#define ADS1115_MV_0P256            0.007813f
#define ADS1115_MV_0P256B           0.007813f
#define ADS1115_MV_0P256C           0.007813f

#define ADS1115_MODE_SHIFT          8
#define ADS1115_MODE_CONTINUOUS     0x00 << ADS1115_MODE_SHIFT
#define ADS1115_MODE_SINGLESHOT     0x01 << ADS1115_MODE_SHIFT // default

#define ADS1115_RATE_SHIFT          5
#define ADS1115_RATE_8              0x00 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_16             0x01 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_32             0x02 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_64             0x03 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_128            0x04 << ADS1115_RATE_SHIFT // default
#define ADS1115_RATE_250            0x05 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_475            0x06 << ADS1115_RATE_SHIFT
#define ADS1115_RATE_860            0x07 << ADS1115_RATE_SHIFT

#define ADS1115_COMP_MODE_SHIFT         4
#define ADS1115_COMP_MODE_HYSTERESIS    0x00 << ADS1115_COMP_MODE_SHIFT        // default
#define ADS1115_COMP_MODE_WINDOW        0x01 << ADS1115_COMP_MODE_SHIFT

#define ADS1115_COMP_POL_SHIFT          3
#define ADS1115_COMP_POL_ACTIVE_LOW     0x00 << ADS1115_COMP_POL_SHIFT     // default
#define ADS1115_COMP_POL_ACTIVE_HIGH    0x01 << ADS1115_COMP_POL_SHIFT

#define ADS1115_COMP_LAT_SHIFT          2
#define ADS1115_COMP_LAT_NON_LATCHING   0x00 << ADS1115_COMP_LAT_SHIFT    // default
#define ADS1115_COMP_LAT_LATCHING       0x01 << ADS1115_COMP_LAT_SHIFT

#define ADS1115_COMP_QUE_SHIFT      0
#define ADS1115_COMP_QUE_ASSERT1    0x00 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT2    0x01 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_ASSERT4    0x02 << ADS1115_COMP_SHIFT
#define ADS1115_COMP_QUE_DISABLE    0x03 // default


class ADS1115 : public I2CDevObj
{
public:
	ADS1115(const char *device_path) :
		I2CDevObj("ADS1115", ADC_DEVICE_PATH, ADC_CLASS_PATH, ADS1115_MEASURE_INTERVAL_US / 2),
		m_measure_channel(ADC_BATTERY_VOLTAGE_CHANNEL),
		m_gain(ADS1115_PGA_4P096)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_ADS1115;
		m_id.dev_id_s.address =	ADS1115_SLAVE_ADDRESS;
		/* TODO - Fix this - don't need the whole 6 channel */
		_buf_adc[ADC_BATTERY_VOLTAGE_CHANNEL].am_channel = ADC_BATTERY_VOLTAGE_CHANNEL;
		_buf_adc[ADC_BATTERY_CURRENT_CHANNEL].am_channel = ADC_BATTERY_CURRENT_CHANNEL;
	}

	virtual ~ADS1115() = default;

	// @return 0 on success, -errno on failure
	int start();

	// @return 0 on success, -errno on failure
	int stop();

protected:
	void _measure();
	int devRead(void *buf, size_t count);
	struct adc_msg_s _buf_adc[ADS1115_CHANNEL_COUNT]; 
	uint32_t m_temperature_from_sensor;
	SyncObj 			m_synchronize;

	// Request to convert voltage or current data
	int _configWrite(uint16_t cmd);
	// Read out the requested sensor data
	int _collect(int32_t &raw);


	/* Channel 2 is voltage, 3 is current */
	static const uint16_t mux_table[ADS1115_CHANNEL_COUNT];

	// returns 0 on success, -errno on failure
	int ads1115_init();

	// Send reset to device
	int m_measure_channel;
	int m_gain;
};

}; // namespace DriverFramework
