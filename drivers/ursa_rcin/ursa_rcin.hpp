/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

//Driver framework driver - PPM RC IN using DMA

#pragma once

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

#define RC_SAMPLE_FREQ     500
#define RC_MEASURE_INTERVAL_US 1000000.0f/RC_SAMPLE_FREQ
#define RC_DEV_PATH "/dev/rc_in"
#define RC_PAGEMAP "/proc/self/pagemap"
#define RC_CLASS_PATH "/dev/rc_in0"

enum state_t{
    RCIN_RPI_INITIAL_STATE = -1,
    RCIN_RPI_ZERO_STATE = 0,
    RCIN_RPI_ONE_STATE = 1
};

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
friend class RC_IN;

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



class RC_IN : public DevObj
{
public:
	RC_IN(const char *device_path) :
		DevObj("URSA_RC_IN", RC_DEV_PATH, RC_CLASS_PATH, DeviceBusType_UNKNOWN, RC_MEASURE_INTERVAL_US),
		   circle_buffer{nullptr},
	    con_blocks{nullptr},
	    prev_tick(0),
	    delta_time(0),
	    curr_tick_inc(1000/RC_SAMPLE_FREQ),
	    curr_pointer(0),
	    curr_channel(0),
	    width_s0(0),
	    curr_signal(0),
	    last_signal(228),
	    state(RCIN_RPI_INITIAL_STATE)
	{
	}

	~RC_IN()
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
	virtual int _process_rc_pulse();
	int devRead(void *buf, size_t count);
	SyncObj 			m_synchronize;

	// returns 0 on success, -errno on failure
	int rc_in_init();
	
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

    uint64_t curr_tick;
    uint64_t prev_tick;
    uint64_t delta_time;

    uint32_t curr_tick_inc;
    uint32_t curr_pointer;
    uint32_t curr_channel;

    uint16_t width_s0;
    uint16_t width_s1;

    uint8_t curr_signal;
    uint8_t last_signal;

    bool _initialized = false;

    state_t state;

    void init_dma_cb(dma_cb_t** cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb);
    void* map_peripheral(uint32_t base, uint32_t len);
    void init_registers();
    void init_ctrl_data();
    void init_PCM();
    void init_DMA();
    void init_buffer();
    static bool stop_dma();
    static void termination_handler(int signum);
    void set_sigaction();
};

}; // namespace DriverFramework
