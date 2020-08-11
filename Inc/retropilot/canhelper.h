/*
  canhelper.h - Helper library to construct and parse can messages from a dbc like format
  ATTENTION: This class is not optimized and uses per-bit operations. Each set/parsed bit takes ~7.5 microseconds!

  Example setting wheelspeeds (toyota corolla 2017) to a can buffer:
    CanHelper canHelper;

    float wheelSpeedKMH = 110.5;
    uint8_t buf[8];
    canHelper.resetBuffer(buf);
    canHelper.putParameterBigEndian(buf, wheelSpeedKMH, -67.67, 0.01, 7, 16);
    canHelper.putParameterBigEndian(buf, wheelSpeedKMH, -67.67, 0.01, 23, 16);
    canHelper.putParameterBigEndian(buf, wheelSpeedKMH, -67.67, 0.01, 39, 16);
    canHelper.putParameterBigEndian(buf, wheelSpeedKMH, -67.67, 0.01, 55, 16);
    
    canHelper.printBitmask(buf);
    // for debugging purposes - will print (matches the cabana bitmask)
      printBitmask:
        0 0 0 1 1 0 1 0 
        0 1 1 0 1 1 1 1 
        0 0 0 1 1 0 1 0 
        0 1 1 0 1 1 1 1 
        0 0 0 1 1 0 1 0 
        0 1 1 0 1 1 1 1 
        0 0 0 1 1 0 1 0 
        0 1 1 0 1 1 1 1 


  Example reading wheelspeeds (toyota corolla 2017) from a canbuffer:
    float extractedWheelSpeedKMH = canHelper.parseParameterBigEndianFloat(buf, -67.67, 0.01, 7, 16);
    Serial.println(extractedWheelSpeedKMH); // will print "110.5"

  Example DBC:
    BO_ 170 WHEEL_SPEEDS: 8 XXX
      SG_ WHEEL_SPEED_FR : 7|16@0+ (0.01,-67.67) [0|250] "kph" XXX
      SG_ WHEEL_SPEED_FL : 23|16@0+ (0.01,-67.67) [0|250] "kph" XXX
      SG_ WHEEL_SPEED_RR : 39|16@0+ (0.01,-67.67) [0|250] "kph" XXX
      SG_ WHEEL_SPEED_RL : 55|16@0+ (0.01,-67.67) [0|250] "kph" XXX
                            ^- most significant bit
                              ^- length
                                 ^- 0=big, 1=little endian
                                  ^- + = unsigned / - = signed
                                      ^- scale
                                            ^- offset



  The MIT License (MIT)

  Copyright (c) 2020 Florian Brede

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#ifndef _CANHELPER_H_
#define _CANHELPER_H_

#include "../core.h"

uint32_t canhelper_parse_be_uint(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size);
int32_t canhelper_parse_be_int(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size);
float canhelper_parse_be_float(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size);  
uint8_t canhelper_parse_be_byte(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size);
void canhelper_put_be_int(uint8_t *buffer, int32_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size);   
void canhelper_put_be_uint(uint8_t *buffer, uint32_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size);   
void canhelper_put_be_float(uint8_t *buffer, float value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size);
uint8_t canhelper_calculate_toyota_checksum(uint8_t *buffer, uint16_t canTx);
void canhelper_put_toyota_checksum(uint8_t *buffer, uint16_t canTx);
bool canhelper_verify_toyota_checksum(uint8_t *buffer, uint16_t canTx);
void canhelper_reset_buffer(uint8_t *buffer);

#endif
