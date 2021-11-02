/*
 * TODO: Add license.
 * Copyright (c) 2021
 *
 * This work is licensed under <>, see the file LICENSE for details.
 *
 * HM01B0 driver.
 */
#ifndef __HIMAX_H
#define __HIMAX_H

#include "camera.h"

class HM01B0: public ImageSensor {
   public:
       int Init();
       int Reset();
       int GetID() { return HM01B0_I2C_ADDR; };
       uint32_t GetClockFrequency() { return 6000000; };
       int SetFrameRate(uint32_t framerate);
       int SetResolution(uint32_t resolution);
       int SetPixelFormat(uint32_t pixelformat);
       int SetTestPattern(bool enable, bool walking);
};
#endif /* __HIMAX_H */
