/*
  canhelper.c - Helper library to construct and parse can messages from a dbc like format
  ATTENTION: This class is not optimized and uses per-bit operations. Each set/parsed bit takes ~7.5 microseconds!


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

#include "retropilot/canhelper.h"
#include "logger.h"


uint32_t canhelper_parse_be_uint(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t returnValue=0;

    if (size>32) {
        //Serial.print("CANHelper Error: size is "); Serial.print(size); Serial.println(" but maximum supported size is 32 bit!");
        return 0;
    }
    unsigned char i;

    int16_t currentByte = mostSignifcantBit/8;
    int16_t currentBit = mostSignifcantBit%8-(size-1);
    currentByte+=ABS(currentBit)/8;
    currentBit=ABS(currentBit)%8;

    unsigned char bit;
    for (i = 0; i<size; i++) {
        bit = buffer[currentByte] >> (currentBit) & 0b00000001;
        returnValue |= bit << i;
        currentBit++;
        if (currentBit>7) {
            currentBit=0;
            currentByte--;
        }
    }
    return returnValue;
}

int32_t canhelper_parse_be_int_signed(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    int32_t rawValue = canhelper_parse_be_uint(buffer, 0, 1, mostSignifcantBit, size);
    int32_t max_value = 1<<(size-1);
    int32_t value;
    if (rawValue<max_value)
        value = rawValue*paramScale+paramOffset;
    else
        value = (-max_value+(rawValue-max_value))*paramScale+paramOffset;
    return value;
}


int32_t canhelper_parse_be_int(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t rawValue = canhelper_parse_be_uint(buffer, 0, 1, mostSignifcantBit, size);
    int32_t value = rawValue*paramScale+paramOffset;
    return value;
}


float canhelper_parse_be_float(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t rawValue = canhelper_parse_be_uint(buffer, 0, 1, mostSignifcantBit, size);
    float value = rawValue*paramScale+paramOffset;
    return value;
}

float canhelper_parse_be_float_signed(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    int32_t rawValue = canhelper_parse_be_uint(buffer, 0, 1, mostSignifcantBit, size);
    int32_t max_value = 1<<(size-1);
    float value;
    if (rawValue<max_value)
        value = rawValue*paramScale+paramOffset;
    else
        value = (-max_value+(rawValue-max_value))*paramScale+paramOffset;
    return value;
}

uint8_t canhelper_parse_be_byte(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t rawValue = canhelper_parse_be_uint(buffer, 0, 1, mostSignifcantBit, size);
    uint8_t value = rawValue*paramScale+paramOffset;
    return value;
}

void canhelper_put_be_internal(uint8_t *buffer, uint32_t value, uint8_t mostSignifcantBit, uint8_t size) {
    if (size>32) {
        //Serial.print("CANHelper Error: size is "); Serial.print(size); Serial.println(" but maximum supported size is 32 bit!");
        return;
    }
    unsigned char i;

    int16_t currentByte = mostSignifcantBit/8;
    int16_t currentBit = mostSignifcantBit%8-(size-1);
    currentByte+=ABS(currentBit)/8;
    currentBit=ABS(currentBit)%8;

    unsigned char bit;
    for (i = 0; i<size; i++) {
        bit = value >> (i) & 0b00000001;
        buffer[currentByte] |= bit << currentBit;
        currentBit++;
        if (currentBit>7) {
            currentBit=0;
            currentByte--;
        }
    }
}

void canhelper_put_be_int(uint8_t *buffer, int32_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return canhelper_put_be_internal(buffer, adjustedValue, mostSignifcantBit, size);
}

void canhelper_put_be_int_signed(uint8_t *buffer, int32_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    int32_t adjustedValue = ROUND((value-paramOffset)/paramScale);
    int32_t max_value = 1<<(size-1);
    if (adjustedValue<0)
        adjustedValue = max_value*2+adjustedValue;
    return canhelper_put_be_internal(buffer, adjustedValue, mostSignifcantBit, size);
}

void canhelper_put_be_uint(uint8_t *buffer, uint32_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return canhelper_put_be_internal(buffer, adjustedValue, mostSignifcantBit, size);
}
void canhelper_put_be_float(uint8_t *buffer, float value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = ROUND((value-paramOffset)/paramScale);
    return canhelper_put_be_internal(buffer, adjustedValue, mostSignifcantBit, size);
}

void canhelper_put_be_float_signed(uint8_t *buffer, float value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    int32_t adjustedValue = ROUND((value-paramOffset)/paramScale);
    int32_t max_value = 1<<(size-1);
    if (adjustedValue<0)
        adjustedValue = max_value*2+adjustedValue;
    return canhelper_put_be_internal(buffer, adjustedValue, mostSignifcantBit, size);
}




uint8_t canhelper_crc_checksum(uint8_t *dat, int len, const uint8_t poly) {
  uint8_t crc = 0xFF;
  int i, j;
  for (i = len - 1; i >= 0; i--) {
    crc ^= dat[i];
    for (j = 0; j < 8; j++) {
      if ((crc & 0x80U) != 0U) {
        crc = (uint8_t)((crc << 1) ^ poly);
      }
      else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

uint8_t canhelper_calculate_toyota_checksum(uint8_t *buffer, uint16_t canTx, uint8_t len) {
  uint8_t checksum = 0;
  checksum = ((canTx & 0xFF00) >> 8) + (canTx & 0x00FF) + len + 1;
  for (int ii = 0; ii < len; ii++) {
    checksum += (buffer[ii]);
  }
  return checksum;
}

void canhelper_put_toyota_checksum(uint8_t *buffer, uint16_t canTx, uint8_t len) {
    buffer[len]=canhelper_calculate_toyota_checksum(buffer, canTx, len);
}


bool canhelper_verify_toyota_checksum(uint8_t *buffer, uint16_t canTx, uint8_t len) {
    uint8_t checksum = canhelper_calculate_toyota_checksum(buffer, canTx, len);
    if (checksum==buffer[len])
        return true;
    logger("CAN CHECKSUM MISMATCH for %d (%d vs %d)!\n", canTx, checksum, buffer[len]);
    return false;
}

void canhelper_reset_buffer(uint8_t *buffer) {
    buffer[0]=buffer[1]=buffer[2]=buffer[3]=buffer[4]=buffer[5]=buffer[6]=buffer[7]=0;
}
