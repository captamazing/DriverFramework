/****************************************************************************
 *
 *   Copyright (C) 2017 LAOSAAC
 *
 ****************************************************************************/

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <px4_shutdown.h>

#include "DriverFramework.hpp"
#include "ursa_gpio.hpp"

using namespace DriverFramework;

//Parameters
#define GPIO_RPI_BUFFER_LENGTH   8
#define GPIO_RPI_DMA_CHANNEL     0
#define GPIO_RPI_MAX_COUNTER     1300
//#define PPM_INPUT_RPI 4
#define GPIO_RPI_MAX_SIZE_LINE   50

//Memory Addresses
#define GPIO_RPI_RPI1_DMA_BASE 0x20007000
#define GPIO_RPI_RPI1_CLK_BASE 0x20101000
#define GPIO_RPI_RPI1_PCM_BASE 0x20203000

#define GPIO_RPI_RPI2_DMA_BASE 0x3F007000
#define GPIO_RPI_RPI2_CLK_BASE 0x3F101000
#define GPIO_RPI_RPI2_PCM_BASE 0x3F203000

#define GPIO_RPI_GPIO_LEV0_ADDR  0x7e200034
#define GPIO_RPI_DMA_LEN         0x1000
#define GPIO_RPI_CLK_LEN         0xA8
#define GPIO_RPI_PCM_LEN         0x24
#define GPIO_RPI_TIMER_BASE      0x7e003004

#define GPIO_RPI_DMA_SGPIO_TIMEDC     (1<<8)
#define GPIO_RPI_DMA_DEST_INC    (1<<4) 
#define GPIO_RPI_DMA_NO_WIDE_BURSTS  (1<<26)
#define GPIO_RPI_DMA_WAIT_RESP   (1<<3)
#define GPIO_RPI_DMA_D_DREQ      (1<<6)
#define GPIO_RPI_DMA_PER_MAP(x)  ((x)<<16)
#define GPIO_RPI_DMA_END         (1<<1)
#define GPIO_RPI_DMA_RESET       (1<<31)
#define GPIO_RPI_DMA_INT         (1<<2)

#define GPIO_RPI_DMA_CS          (0x00/4)
#define GPIO_RPI_DMA_CONBLK_AD   (0x04/4)
#define GPIO_RPI_DMA_DEBUG       (0x20/4)

#define GPIO_RPI_PCM_CS_A        (0x00/4)
#define GPIO_RPI_PCM_FIFO_A      (0x04/4)
#define GPIO_RPI_PCM_MODE_A      (0x08/4)
#define GPIO_RPI_PCM_RXC_A       (0x0c/4)
#define GPIO_RPI_PCM_TXC_A       (0x10/4)
#define GPIO_RPI_PCM_DREQ_A      (0x14/4)
#define GPIO_RPI_PCM_INTEN_A     (0x18/4)
#define GPIO_RPI_PCM_INT_STC_A   (0x1c/4)
#define GPIO_RPI_PCM_GRAY        (0x20/4)

#define GPIO_RPI_PCMCLK_CNTL     38
#define GPIO_RPI_PCMCLK_DIV      39

volatile uint32_t *GPIO_TIMED::pcm_reg;
volatile uint32_t *GPIO_TIMED::clk_reg;
volatile uint32_t *GPIO_TIMED::dma_reg;

//**************
// MEMORY TABLE HELPER CLASS
//**************
Memory_table::Memory_table()
{
    _page_count = 0;
}

// Init Memory table
Memory_table::Memory_table(uint32_t page_count, int version)
{
    uint32_t i;
    int fdMem, file;
    // Cache coherent adresses depends on RPI's version
    uint32_t bus = version == 1 ? 0x40000000 : 0xC0000000;
    uint64_t pageInfo;
    void *offset;

    _virt_pages = (void **)malloc(page_count * sizeof(void *));
    _phys_pages = (void **)malloc(page_count * sizeof(void *));
    _page_count = page_count;

    if ((fdMem = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
        fprintf(stderr, "Failed to open /dev/mem\n");
        exit(-1);
    }

    if ((file = open("/proc/self/pagemap", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
        fprintf(stderr, "Failed to open /proc/self/pagemap\n");
        exit(-1);
    }

    // Magic to determine the physical address for this page:
    offset = mmap(0, _page_count * PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED, -1, 0);
    lseek(file, ((uintptr_t)offset) / PAGE_SIZE * 8, SEEK_SET);

    // Get list of available cache coherent physical addresses
    for (i = 0; i < _page_count; i++) {
        _virt_pages[i] = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS | MAP_NORESERVE | MAP_LOCKED, -1, 0);
        ::read(file, &pageInfo, 8);
        _phys_pages[i] = (void *)((uintptr_t)(pageInfo * PAGE_SIZE) | bus);
    }

    // Map physical addresses to virtual memory
    for (i = 0; i < _page_count; i++) {
        munmap(_virt_pages[i], PAGE_SIZE);
        _virt_pages[i] = mmap(_virt_pages[i], PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_FIXED | MAP_NORESERVE | MAP_LOCKED, fdMem, ((uintptr_t)_phys_pages[i] & (version == 1 ? 0xFFFFFFFF : ~bus)));
        memset(_virt_pages[i], 0xee, PAGE_SIZE);
    }
    close(file);
    close(fdMem);
}

Memory_table::~Memory_table()
{
    free(_virt_pages);
    free(_phys_pages);
}

// This function returns physical address with help of pointer, which is offset
// from the beginning of the buffer.
void *Memory_table::get_page(void **const pages, uint32_t addr) const
{
    if (addr >= PAGE_SIZE * _page_count) {
        return nullptr;
    }
    return (uint8_t *)pages[(uint32_t)addr / 4096] + addr % 4096;
}

//Get virtual address from the corresponding physical address from memory_table.
void *Memory_table::get_virt_addr(const uint32_t phys_addr) const
{
    // FIXME: Can't the address be calculated directly?
    // FIXME: if the address room  in _phys_pages is not fragmented one may avoid
    //        a complete loop ..
    uint32_t i = 0;
    for (; i < _page_count; i++) {
        if ((uintptr_t)_phys_pages[i] == (((uintptr_t)phys_addr) & 0xFFFFF000)) {
            return (void *)((uintptr_t)_virt_pages[i] + (phys_addr & 0xFFF));
        }
    }
    return nullptr;
}

// This function returns offset from the beginning of the buffer using virtual
// address and memory_table.
uint32_t Memory_table::get_offset(void ** const pages, const uint32_t addr) const
{
    uint32_t i = 0;
    for (; i < _page_count; i++) {
        if ((uintptr_t) pages[i] == (addr & 0xFFFFF000) ) {
            return (i*PAGE_SIZE + (addr & 0xFFF));
        }
    }
    return -1;
}

// How many bytes are available for reading in circle buffer?
uint32_t Memory_table::bytes_available(const uint32_t read_addr, const uint32_t write_addr) const
{
    if (write_addr > read_addr) {
        return (write_addr - read_addr);
    } else {
        return _page_count * PAGE_SIZE - (read_addr - write_addr);
    }
}

uint32_t Memory_table::get_page_count() const
{
    return _page_count;
}


//**************
// GPIO_TIMED CLASS
//**************
int GPIO_TIMED::gpio_timed_init()
{
	dma_base = GPIO_RPI_RPI2_DMA_BASE;
    clk_base = GPIO_RPI_RPI2_CLK_BASE;
    pcm_base = GPIO_RPI_RPI2_PCM_BASE;
	circle_buffer = new Memory_table(GPIO_RPI_BUFFER_LENGTH * 2, 2);
    con_blocks = new Memory_table(GPIO_RPI_BUFFER_LENGTH * 113, 2);

    init_registers();

    // Configuration
    set_sigaction();
    init_ctrl_data();
    init_PCM();
    init_DMA();

    usleep(100);

    // Init callbacks and pin states for GPIO
    for (int i=0;i<GPIO_PIN_COUNT;i++){
        state[i]=GPIO_RPI_INITIAL_STATE;
        highTimeCB[i]=nullptr;
    }

    // Reading first sample
    curr_tick = *((uint64_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer));
    prev_tick = curr_tick;
    curr_pointer += 8;
    curr_signal = *((uint8_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer)) & 0x10 ? 1 : 0;
    last_signal = curr_signal;
    curr_pointer++;

    _initialized = true;

	return 0;
}

// Map peripheral to virtual memory
void *GPIO_TIMED::map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_CLOEXEC);
    void *vaddr;

    if (fd < 0) {
        printf("Failed to open /dev/mem: %m\n");
        return nullptr;
    }
    vaddr = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED) {
        printf("rpio-pwm: Failed to map peripheral at 0x%08x: %m\n", base);
    }

    close(fd);
    return vaddr;
}

// Method to init DMA control block
void GPIO_TIMED::init_dma_cb(dma_cb_t **cbp, uint32_t mode, uint32_t source, uint32_t dest, uint32_t length, uint32_t stride, uint32_t next_cb)
{
    (*cbp)->info = mode;
    (*cbp)->src = source;
    (*cbp)->dst = dest;
    (*cbp)->length = length;
    (*cbp)->next = next_cb;
    (*cbp)->stride = stride;
}

bool GPIO_TIMED::stop_dma()
{
	DF_LOG_INFO("Turning off DMA for GPIO");
    dma_reg[GPIO_RPI_DMA_CS | GPIO_RPI_DMA_CHANNEL << 8] = 0;
    return true;
}

/* We need to be sure that the DMA is stopped upon termination */
void GPIO_TIMED::termination_handler(int signum)
{
    stop_dma();
}

// This function is used to init DMA control blocks (setting sampling GPIO
// register, destination adresses, synchronization)
void GPIO_TIMED::init_ctrl_data()
{
    uint32_t phys_fifo_addr;
    uint32_t dest = 0;
    uint32_t cbp = 0;
    dma_cb_t *cbp_curr;
    // Set fifo addr (for delay)
    phys_fifo_addr = ((pcm_base + 0x04) & 0x00FFFFFF) | 0x7e000000;

    // Init dma control blocks.
    /*We are transferring 1 byte of GPIO register. Every 14th iteration we are 
      sampling TIMER register, which length is 8 bytes. So, for every 14 samples of GPIO we need 
      14 * 4 + 8 = 64 bytes of buffer. Value 14 was selected specially to have a 64-byte "block" 
      TIMER - GPIO. So, we have integer count of such "blocks" at one virtual page. (4096 / 64 = 64 
      "blocks" per page. As minimum, we must have 2 virtual pages of buffer (to have integer count of 
      vitual pages for control blocks): for every 14 iterations (64 bytes of buffer) we need 14 control blocks for GPIO
      sampling, 14 control blocks for setting frequency and 1 control block for sampling timer, so,
      we need 14 + 14 + 1 = 29 control blocks. For integer value, we need 29 pages of control blocks.
      Each control block length is 32 bytes. In 29 pages we will have (29 * 4096 / 32) = 29 * 128 control
      blocks. 29 * 128 control blocks = 64 * 128 bytes of buffer = 2 pages of buffer.
      So, for 14 * 64 * 2 iteration we init DMA for sampling GPIO
      and timer to (64 * 64 * 2) = 8192 bytes = 2 pages of buffer.
    */

    for (uint32_t i = 0; i < 14 * 128 * GPIO_RPI_BUFFER_LENGTH; i++) {
        //Transfer timer every 14th sample
        if (i % 14 == 0) {
            cbp_curr = (dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp);

            init_dma_cb(&cbp_curr, GPIO_RPI_DMA_NO_WIDE_BURSTS | GPIO_RPI_DMA_WAIT_RESP | GPIO_RPI_DMA_DEST_INC | GPIO_RPI_DMA_SGPIO_TIMEDC, GPIO_RPI_TIMER_BASE,
                        (uintptr_t)circle_buffer->get_page(circle_buffer->_phys_pages, dest),
                        8,
                        0,
                        (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages,
                                                        cbp + sizeof(dma_cb_t)));

            dest += 8;
            cbp += sizeof(dma_cb_t);
        }

        // Transfer GPIO (4 bytes)
        cbp_curr = (dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp);
        init_dma_cb(&cbp_curr, GPIO_RPI_DMA_NO_WIDE_BURSTS | GPIO_RPI_DMA_WAIT_RESP, GPIO_RPI_GPIO_LEV0_ADDR,
                    (uintptr_t)circle_buffer->get_page(circle_buffer->_phys_pages, dest),
                    4,
                    0,
                    (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages,
                                                    cbp + sizeof(dma_cb_t)));

        dest += 4;
        cbp += sizeof(dma_cb_t);

        // Delay (for setting sampling frequency)
        /* DMA is waiting data request signal (DREQ) from PCM. PCM is set for 1 MhZ freqency, so,
	       each sample of GPIO is limited by writing to PCA queue.
	    */
        cbp_curr = (dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp);
        init_dma_cb(&cbp_curr, GPIO_RPI_DMA_NO_WIDE_BURSTS | GPIO_RPI_DMA_WAIT_RESP | GPIO_RPI_DMA_D_DREQ | GPIO_RPI_DMA_PER_MAP(2),
                    GPIO_RPI_TIMER_BASE, phys_fifo_addr,
                    4,
                    0,
                    (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages,
                                                    cbp + sizeof(dma_cb_t)));

        cbp += sizeof(dma_cb_t);
    }
    //Make last control block point to the first (to make circle)
    cbp -= sizeof(dma_cb_t);
    ((dma_cb_t *)con_blocks->get_page(con_blocks->_virt_pages, cbp))->next = (uintptr_t)con_blocks->get_page(con_blocks->_phys_pages, 0);
}

/*Initialise PCM
  See BCM2835 documentation:
  http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 */
void GPIO_TIMED::init_PCM()
{
    pcm_reg[GPIO_RPI_PCM_CS_A] = 1;                                          // Disable Rx+Tx, Enable PCM block
    usleep(100);
    clk_reg[GPIO_RPI_PCMCLK_CNTL] = 0x5A000006;                              // Source=PLLD (500MHz)
    usleep(100);
    clk_reg[GPIO_RPI_PCMCLK_DIV] = 0x5A000000 | ((50000/GPIO_SAMPLE_FREQ)<<12);   // Set pcm div. If we need to configure DMA frequency.
    usleep(100);
    clk_reg[GPIO_RPI_PCMCLK_CNTL] = 0x5A000016;                              // Source=PLLD and enable
    usleep(100);
    pcm_reg[GPIO_RPI_PCM_TXC_A] = 0<<31 | 1<<30 | 0<<20 | 0<<16;             // 1 channel, 8 bits
    usleep(100);
    pcm_reg[GPIO_RPI_PCM_MODE_A] = (10 - 1) << 10;                           //PCM mode
    usleep(100);
    pcm_reg[GPIO_RPI_PCM_CS_A] |= 1<<4 | 1<<3;                               // Clear FIFOs
    usleep(100);
    pcm_reg[GPIO_RPI_PCM_DREQ_A] = 64<<24 | 64<<8;                           // DMA Req when one slot is free?
    usleep(100);
    pcm_reg[GPIO_RPI_PCM_CS_A] |= 1<<9;                                      // Enable DMA
    usleep(100);
    pcm_reg[GPIO_RPI_PCM_CS_A] |= 1<<2;                                      // Enable Tx
    usleep(100);
}

/*Initialise DMA
  See BCM2835 documentation:
  http://www.raspberrypi.org/wp-content/uploads/2012/02/BCM2835-ARM-Peripherals.pdf
 */
void GPIO_TIMED::init_DMA()
{
    dma_reg[GPIO_RPI_DMA_CS | GPIO_RPI_DMA_CHANNEL << 8] = GPIO_RPI_DMA_RESET;                 //Reset DMA
    usleep(100);
    dma_reg[GPIO_RPI_DMA_CS | GPIO_RPI_DMA_CHANNEL << 8] = GPIO_RPI_DMA_INT | GPIO_RPI_DMA_END;
    dma_reg[GPIO_RPI_DMA_CONBLK_AD | GPIO_RPI_DMA_CHANNEL << 8] = reinterpret_cast<uintptr_t>(con_blocks->get_page(con_blocks->_phys_pages, 0));//Set first control block address
    dma_reg[GPIO_RPI_DMA_DEBUG | GPIO_RPI_DMA_CHANNEL << 8] = 7;                      // clear debug error flags
    dma_reg[GPIO_RPI_DMA_CS | GPIO_RPI_DMA_CHANNEL << 8] = 0x10880001;                // go, mid priority, wait for outstanding writes    
}


// We must stop DMA when the process is killed
void GPIO_TIMED::set_sigaction()
{
    for (int i = 0; i < NSIG; i++) {
        // catch all signals to ensure DMA is disabled - some of them may
        // already be handled elsewhere in cases we consider normal
        // termination. In those cases the teardown() method must be called.
        struct sigaction sa, sa_old;
        memset(&sa, 0, sizeof(sa));
        sigaction(i, nullptr, &sa_old);

        if (sa_old.sa_handler == nullptr) {
            sa.sa_handler = GPIO_TIMED::termination_handler;
            sigaction(i, &sa, nullptr);
        }
    }
    px4_register_shutdown_hook(GPIO_TIMED::stop_dma);
}

//Initializing necessary registers
void GPIO_TIMED::init_registers()
{
    dma_reg = (uint32_t *)map_peripheral(dma_base, GPIO_RPI_DMA_LEN);
    pcm_reg = (uint32_t *)map_peripheral(pcm_base, GPIO_RPI_PCM_LEN);
    clk_reg = (uint32_t *)map_peripheral(clk_base, GPIO_RPI_CLK_LEN);
}

int GPIO_TIMED::start()
{
	/* Initialize the GPIO DMA/PCM */
	int result = gpio_timed_init();

	if (result != 0) {
		DF_LOG_ERR("error: GPIO init failed, sensor read thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("error: could not start DevObj for GPIO Rx");
		goto exit;
	}

exit:

	return result;
}

void GPIO_TIMED::_measure(){
    uint32_t counter = 0;

    if (!_initialized) {
        return;
    }

    // Now we are getting address in which DMAC is writing at current moment
    dma_cb_t *ad = (dma_cb_t *)con_blocks->get_virt_addr(dma_reg[RCIN_RPI_DMA_CONBLK_AD | RCIN_RPI_DMA_CHANNEL << 8]);
    for (int j = 1; j >= -1; j--) {
        void *x = circle_buffer->get_virt_addr((ad + j)->dst);
        if (x != nullptr) {
            counter = circle_buffer->bytes_available(curr_pointer,
                                                     circle_buffer->get_offset(circle_buffer->_virt_pages, (uintptr_t)x));
            break;
        }
    }

    if (counter == 0) {
        return;
    }

    // How many bytes have DMA transferred (and we can process)?
    // We can't stay in method for a long time, because it may lead to delays
    if (counter > RCIN_RPI_MAX_COUNTER) {
        counter = RCIN_RPI_MAX_COUNTER;
    }

    // Processing ready bytes
    for (; counter > 0x40; counter--) {
        // Is it timer sample?
        if (curr_pointer % (64) == 0) {
            curr_tick = *((uint64_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer));
            curr_pointer += 8;
            counter -= 8;
        }

        curr_signal = *((uint32_t *)circle_buffer->get_page(circle_buffer->_virt_pages, curr_pointer));
        uint32_t diff_signal = curr_signal^last_signal;
        for (int i=0; i<GPIO_PIN_COUNT;i++){
            if (highTimeCB[callbackStruct->pin]==nullptr) continue; //Don't waste time on pins with no callbacks registered
            bitmask=0x01<<i; //Bitmask for this pin
            // If the signal changed
            if (diff_signal & bitmask) {
// TO DO - 26 JUNE
// CONTINUE HERE
                delta_time = curr_tick - prev_tick;
                prev_tick = curr_tick;
                switch (state) {
                case RCIN_RPI_INITIAL_STATE:
                    state = RCIN_RPI_ZERO_STATE;
                    break;
                case RCIN_RPI_ZERO_STATE:
                    if (curr_signal == 0) {
                        width_s0 = (uint16_t)delta_time;
                        state = RCIN_RPI_ONE_STATE;
                    }
                    break;
                case RCIN_RPI_ONE_STATE:
                    if (curr_signal == 1) {
                        width_s1 = (uint16_t)delta_time;
                        state = RCIN_RPI_ZERO_STATE;
                    }
                    break;
                }
            }

            
        }
        last_signal = curr_signal;
        curr_pointer+=4;
        if (curr_pointer >= circle_buffer->get_page_count() * PAGE_SIZE) {
            curr_pointer = 0;
        }
        curr_tick += curr_tick_inc;
    }
}

int GPIO_TIMED::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed for GPIO dev obj");
		return result;
	}
	stop_dma();
	return 0;
}

int GPIO_TIMED::devRead(void *buf, size_t count)
{	
    buf = 

	DF_LOG_INFO("READING GPIO");
    return 0;
}

int GPIO_TIMED::devWrite(const void *buf, size_t count){
    gpio_callback_t* callbackStruct;
    callbackStruct=(gpio_callback_t*) buf;
    if (callbackStruct->type==GPIO_CALLBACK_HIGHTIME){
        highTimeCB[callbackStruct->pin]=callbackStruct->callback;
    }
}