/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

//Driver framework driver - PPM RC IN using DMA

#pragma once

#include <functional>

#include "DevObj.hpp"
#include <signal.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdarg.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <assert.h>
#include <queue>

#include "../pigpio/pigpio.hpp"

namespace DriverFramework
{

#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

#define GPIO_PIN_COUNT 32
#define GPIO_SAMPLE_US 5
#define GPIO_MEASURE_FREQ 1
#define GPIO_CALLBACK 1
#define GPIO_WRITE 2
#define GPIO_MEASURE_INTERVAL_US 1000000/GPIO_MEASURE_FREQ
#define GPIO_DEV_PATH "/dev/gpio_timed"
#define GPIO_PAGEMAP "/proc/self/pagemap"
#define GPIO_CLASS_PATH "/dev/gpio_timed0"

// Pin state enum
enum state_t{
    GPIO_RPI_INITIAL_STATE = -1,
    GPIO_RPI_ZERO_STATE = 0,
    GPIO_RPI_ONE_STATE = 1
};

//Callback registration struct
typedef struct {
    std::function<void(int, int, uint32_t)> callback;
    uint32_t gpio; // The pin
    uint32_t value; // 1 - high, 0 - low
    uint32_t type;
} gpio_write_t;

//Memory table structure
typedef struct {
    void **virt_pages;
    void **phys_pages;
    uint32_t page_count;
} memory_table_t;


//DMA control block structure
typedef struct {
  uint32_t info, src, dst, length,
    stride, next, pad[2];
} dma_cb_t;


class GPIO_TIMED : public DevObj
{
public:
	GPIO_TIMED(const char *device_path) :
		DevObj("URSA_GPIO_TIMED", GPIO_DEV_PATH, GPIO_CLASS_PATH, DeviceBusType_UNKNOWN, GPIO_MEASURE_INTERVAL_US),
	    curr_tick(0),
	    delta_time(0),
	    curr_pointer(0),
	    curr_channel(0),
	    curr_signal(0),
	    last_signal(228)
	{
	}

	~GPIO_TIMED()
	{
	}

	// @return 0 on success, -errno on failure
	int start();

	// @return 0 on success, -errno on failure
	int stop();

    static void callbackWrapper(int gpio, int level, uint32_t tick, void * function);

protected:

	void _measure();
	int devRead(void *buf, size_t count);
    int devWrite(const void *buf, size_t count);
	SyncObj m_synchronize;

	// returns 0 on success, -errno on failure
	int gpio_timed_init();
	
	//Physical adresses of peripherals. Are different on different Raspberries.
    uint32_t dma_base;
    uint32_t clk_base;
    uint32_t pcm_base;

    uint64_t last_high_tick;
    uint64_t curr_tick;
    uint64_t delta_time;

    uint32_t curr_pointer;
    uint32_t curr_channel;

    uint32_t curr_signal;
    uint32_t last_signal;
    uint32_t diff_signal;
    uint32_t bitmask;

    uint32_t state;
    //void (*highTimeCB[GPIO_PIN_COUNT])(uint32_t);
    //void (*totalTimeCB[GPIO_PIN_COUNT])(uint32_t);
    std::function<void(uint32_t)> highTimeCB[GPIO_PIN_COUNT];
    std::function<void(uint32_t)> totalTimeCB[GPIO_PIN_COUNT];
    uint64_t last_change[GPIO_PIN_COUNT];
    uint16_t width_s0[GPIO_PIN_COUNT];
    uint16_t width_s1[GPIO_PIN_COUNT];

    bool _initialized = false;
};

}; // namespace DriverFramework
