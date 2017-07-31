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



#include "DriverFramework.hpp"
#include "ursa_gpio.hpp"

using namespace DriverFramework;

//Parameters
#define GPIO_RPI_BUFFER_LENGTH   500
#define GPIO_RPI_DMA_CHANNEL     0
#define GPIO_RPI_MAX_COUNTER     52000
//#define PPM_INPUT_RPI 4

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

//**************
// GPIO_TIMED CLASS
//**************
int GPIO_TIMED::gpio_timed_init()
{
    PIGPIO::gpioCfgInterfaces(PI_DISABLE_FIFO_IF | PI_DISABLE_SOCK_IF);
    PIGPIO::gpioCfgClock(GPIO_SAMPLE_US,0,0);
    int i = PIGPIO::gpioInitialise();
    if (i<0){
        DF_LOG_INFO("Failed to initialise PIGPIO library");
        return i;
    } 
    _initialized = true;
    return 0;
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
    //Do nothing here...
}

int GPIO_TIMED::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed for GPIO dev obj");
		return result;
	}
    PIGPIO::gpioTerminate();
	return 0;
}

int GPIO_TIMED::devRead(void *buf, size_t count)
{	
	DF_LOG_INFO("READING GPIO");
    return 0;
}

int GPIO_TIMED::devWrite(const void *buf, size_t count){
    gpio_write_t* writeStruct;
    writeStruct=(gpio_write_t*) buf;
    if (writeStruct->type==GPIO_CALLBACK){
        PIGPIO::gpioSetAlertFuncEx(writeStruct->gpio,callbackWrapper, &writeStruct->callback);
    } else if (writeStruct->type==GPIO_WRITE){
        PIGPIO::gpioSetMode(writeStruct->gpio,PI_OUTPUT);
        PIGPIO::gpioWrite(writeStruct->gpio,writeStruct->value);
    }
    return 1;
}

void GPIO_TIMED::callbackWrapper(int gpio, int level, uint32_t tick, void * function){
    std::function<void(int, int, uint32_t)> *cbFunc;
    cbFunc = (std::function<void(int, int, uint32_t)> *)function;
    (*cbFunc)(gpio, level, tick);
}