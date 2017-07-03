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

namespace DriverFramework
{

#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

#define GPIO_PIN_COUNT 32
#define GPIO_SAMPLE_FREQ 1000000
#define GPIO_MEASURE_FREQ 500
#define GPIO_CALLBACK_HIGHTIME 1
#define GPIO_CALLBACK_TOTALTIME 2
#define GPIO_MEASURE_INTERVAL_US 1000000.0f/GPIO_MEASURE_FREQ
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
    std::function<void(uint32_t)> callback;
    uint32_t pin;
    uint32_t type;
} gpio_callback_t;

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

class Memory_table {
// Allow RCInput_RPI access to private members of Memory_table
friend class GPIO_TIMED;

private:
    void** _virt_pages;
    void** _phys_pages;
    uint32_t _page_count;

public:
    Memory_table();
    Memory_table(uint32_t, int);
    ~Memory_table();

    //Get virtual address from the corresponding physical address from memory_table.
    void* get_virt_addr(const uint32_t phys_addr) const;

    // This function returns physical address with help of pointer, which is offset from the beginning of the buffer.
    void* get_page(void **pages, const uint32_t addr) const;

    // This function returns offset from the beginning of the buffer using (virtual) address in 'pages' and memory_table.
    uint32_t get_offset(void **pages, const uint32_t addr) const;

    //How many bytes are available for reading in circle buffer?
    uint32_t bytes_available(const uint32_t read_addr, const uint32_t write_addr) const;

    uint32_t get_page_count() const;
};



class GPIO_TIMED : public DevObj
{
public:
	GPIO_TIMED(const char *device_path) :
		DevObj("URSA_GPIO_TIMED", GPIO_DEV_PATH, GPIO_CLASS_PATH, DeviceBusType_UNKNOWN, GPIO_MEASURE_INTERVAL_US),
        circle_buffer{nullptr},
	    con_blocks{nullptr},
	    curr_tick(0),
	    delta_time(0),
	    curr_tick_inc(1e6/GPIO_SAMPLE_FREQ),
	    curr_pointer(0),
	    curr_channel(0),
	    curr_signal(0),
	    last_signal(228)
	{
	}

	~GPIO_TIMED()
	{
		delete circle_buffer;
    	delete con_blocks;
	}

	// @return 0 on success, -errno on failure
	int start();

	// @return 0 on success, -errno on failure
	int stop();

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

    //registers
    static volatile uint32_t *pcm_reg;
    static volatile uint32_t *clk_reg;
    static volatile uint32_t *dma_reg;

    Memory_table *circle_buffer;
    Memory_table *con_blocks;

    uint64_t last_high_tick;
    uint64_t curr_tick;
    uint64_t delta_time;

    uint32_t curr_tick_inc;
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

    void init_dma_cb(dma_cb_t** cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb);
    void* map_peripheral(uint32_t base, uint32_t len);
    void init_registers();
    void init_ctrl_data();
    void init_PCM();
    void init_DMA();
    void init_buffer();
    bool stop_dma();
};

}; // namespace DriverFramework
