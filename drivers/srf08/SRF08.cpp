#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h> 
#include "DriverFramework.hpp"
#include "SRF08.hpp"
#include <DevMgr.hpp>

namespace DriverFramework
{

int SRF08::start() {

    int result = I2CDevObj::start();
    if (result < 0) {
        DF_LOG_ERR("could not start i2c device for SRF08");
        return result;
    }

    result = _setSlaveConfig(SRF08_SLAVE_ADDRESS,
                     SRF08_BUS_FREQUENCY_IN_KHZ,
                     SRF08_TRANSFER_TIMEOUT_IN_USECS);
    if (result < 0) {
        DF_LOG_ERR("could not set slave config for SRF08");
        return result;
    }

    result = DevObj::start();
    if (result != 0) {
        DF_LOG_ERR("error: could not start DevObj for SRF08");
        return result;
    }

    // Get an initial measurement
    _startRanging();

    return 0;
}

int SRF08::stop(){
    return 0;
}

void SRF08::_measure(){
    // Take a reading
    // uint32_t range_test=0;
    uint32_t range_us=0;
    // // uint32_t diff=0;
    // int32_t mindiff=-1;
    uint8_t lowByte;
    uint8_t highByte;
    uint8_t highReg = SRF08_RA_ECHO_H;
    uint8_t lowReg = SRF08_RA_ECHO_L;
    _readReg(highReg,&highByte,1);
    _readReg(lowReg,&lowByte,1);
    range_us=((int)highByte<<8) | lowByte;
    // while (lowByte != 0 && highByte != 0 && lowReg<SRF08_RA_ECHO_L_MAX){
    //     range_test=((int)highByte<<8) | lowByte;
    //     diff=abs((int)range_test-(int)_last_us);
    //     if (diff<mindiff || mindiff==-1){
    //         mindiff=diff;
    //         range_us=range_test;
    //     }
    //     highReg+=2;
    //     lowReg+=2;
    //     _readReg(highReg,&highByte,1);
    //     _readReg(lowReg,&lowByte,1);
    // }
    // _last_us=range_us;
    _publish((double)range_us/5882.35);

    // Get a measurement for next time
    _startRanging();
}

void SRF08::_startRanging(){
    uint8_t command = SRF08_START_RANGE_CMD;
    _writeReg(SRF08_RA_CMD,&command,1);
}


} //end namespace