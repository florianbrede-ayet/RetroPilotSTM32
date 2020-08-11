/*
  vss.c - Handling VSS sensor interrupts, calculating the current wheelspeed and storing it in RetropilotParams
  
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

#include "retropilot/globals.h"
#include "retropilot/retropilot_params.h"
#include "retropilot/vss.h"
#include "critical.h"

float vss_ring_buffer[VSS_RINGBUFFER_SIZE];
float vss_speed_kmh = 0;
float vss_speed_sum = 0;
float vss_last_valid_speed_kmh = 0;

int vss_ring_bufferIndex = 0;

unsigned long vss_duration = 0;
unsigned long vss_last_refresh = 0;
unsigned long vss_last_valid_speed_ts = 0;

unsigned long vss_latest_handled_trigger_micros = 0;

unsigned long vss_last_wheellock_time = 0;
unsigned long vss_last_wheellock_check = 0;



volatile int vss_sensor_revolutions=0;
volatile unsigned long vss_last_trigger_micros=0;

void vss_interrupt() {
  ENTER_CRITICAL();
  vss_sensor_revolutions++;
  vss_last_trigger_micros=micros();
  EXIT_CRITICAL();
}

/* called at initialization */
void vss_setup()
{
    for (int i=0; i<VSS_RINGBUFFER_SIZE; i++)
        vss_ring_buffer[i]=0;
}

/**
 * This function is called each loop and determines the current retropilotParams.vssAvgSpeedKMH.
 * It measures the exact micros elapsed between the last handled hall sensor trigger and the latest trigger [interrupt driven].
 * The duration is is used to determine the highest current speed within each VSS_REFRESH_RATE_MS interval 
 * (highest speed because at high frequencies, the hall sensor sometimes loses revolutions [capacitance?] so we use the biggest indiviual speed)
 * The speed is averaged for VSS_RINGBUFFER_SIZE*VSS_REFRESH_RATE_MS (< 1s)
 * */
void vss_update_sensor_readings()
{
#if VSS_SENSOR_SMOOTHING == 0 || VSS_SENSOR_SMOOTHING == 1
    if (vss_sensor_revolutions > 0)
    {

        vss_duration = (micros() - vss_latest_handled_trigger_micros);
        uint8_t SaveSREG = SREG;
        noInterrupts();
        int tmpVssSensorRevolutions = vss_sensor_revolutions;
        vss_latest_handled_trigger_micros = vss_last_trigger_micros;
        vss_sensor_revolutions -= tmpVssSensorRevolutions;
        SREG = SaveSREG;

        vss_speed_kmh = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vss_duration * 0.000001)) * 3.6;
#if VSS_SENSOR_SMOOTHING == 1
        vss_speed_kmh = MAX(MIN(vss_speed_kmh, retropilotParams.vssAvgSpeedKMH + 10), retropilotParams.vssAvgSpeedKMH - 10);
#endif
        vss_last_wheellock_check=millis();
    }
#if CAR_WITHOUT_ABS_BRAKES
    else if (retropilotParams.vssAvgSpeedKMH>20 && millis()-vss_last_wheellock_check>40) { // ATTENTION: this section relies on having the sensor installed somewhere at the rear axis, where a locked axis would result in a fatal car instability
         unsigned long lastVssDuration = 1.0f/(retropilotParams.vssAvgSpeedKMH/3.6f/VSS_DISTANCE_PER_REVOLUTION);
         lastVssDuration *= 1000000L;
         if (micros() - vss_latest_handled_trigger_micros > lastVssDuration*2.5) { // if we've been waiting 2.5 times the expected time to trigger assume wheellock (we expect one sensor miss and decelleration, so 2+ a little margin for decell)
            // with a distance of 0.5m/revolution this would trigger ~ 50ms after locking the wheels at 100 km/h
            vss_last_wheellock_time=millis();
            vss_last_wheellock_check=millis();
         }        
    }
#endif    
    else if (micros() - vss_latest_handled_trigger_micros > 1000L * 1000L)
    { // 1 second without hall signal is interpreted as standstill
        vss_speed_kmh = 0;
        vss_last_wheellock_check=millis();
    }
#elif VSS_SENSOR_SMOOTHING == 2 || VSS_SENSOR_SMOOTHING == 3
    if (vss_sensor_revolutions > 0)
    {
        vss_duration = (vss_last_trigger_micros - vss_latest_handled_trigger_micros);
        ENTER_CRITICAL();
        int tmpVssSensorRevolutions = vss_sensor_revolutions;
        vss_latest_handled_trigger_micros = vss_last_trigger_micros;
        vss_sensor_revolutions -= tmpVssSensorRevolutions;
        EXIT_CRITICAL();

        float tmpSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vss_duration * 0.000001)) * 3.6;
        if (tmpSpeedKMH <= VSS_MAX_SPEED) // we cap the speed we measure to max. 150km/h (max. OP speed) because sometimes at high frequencies the hall sensor might bounce and produce incorrect, way too high readings
            vss_speed_kmh = MAX(vss_speed_kmh, tmpSpeedKMH);
#if VSS_SENSOR_SMOOTHING == 3
        vss_speed_kmh = MAX(MIN(vss_speed_kmh, retropilotParams.vssAvgSpeedKMH + 10), retropilotParams.vssAvgSpeedKMH - 10);
#endif
        vss_last_wheellock_check=millis();
    }
#if CAR_WITHOUT_ABS_BRAKES
    else if (retropilotParams.vssAvgSpeedKMH>20 && millis()-vss_last_wheellock_check>50) { // ATTENTION: this section relies on having the sensor installed somewhere at the rear axis, where a locked axis would result in a fatal car instability
         unsigned long lastVssDuration = 1.0f/(retropilotParams.vssAvgSpeedKMH/3.6f/VSS_DISTANCE_PER_REVOLUTION);
         lastVssDuration *= 1000000L;
         if (micros() - vss_latest_handled_trigger_micros > lastVssDuration*3.5) { // if we've been waiting 3.5 times the expected time to trigger assume wheellock (we expect one sensor miss and decelleration, so 3+ a little margin)
            // with a distance of 0.5m/revolution this would trigger ~ 65ms after locking the wheels at 100 km/h
            vss_last_wheellock_time=millis();
            vss_last_wheellock_check=millis();
         }        
    }
#endif      
    else if (micros() - vss_latest_handled_trigger_micros > 1000L * 1000L)
    { // 1 second without hall signal is interpreted as standstill
        vss_speed_kmh = 0;
        vss_last_wheellock_check=millis();
    }
#endif

    if (millis() - vss_last_refresh >= VSS_REFRESH_RATE_MS)
    {
        vss_last_refresh = millis();

        // this allows us to measure accurate low speeds (~1.5-8 km/h)
        if (vss_speed_kmh > 0)
        {
            vss_last_valid_speed_kmh = vss_speed_kmh;
            vss_last_valid_speed_ts = millis();
        }
        else if (vss_speed_kmh == 0 && vss_last_valid_speed_kmh > 0 && millis() - vss_last_valid_speed_ts < 1000)
        {
            vss_speed_kmh = vss_last_valid_speed_kmh;
        }

        vss_speed_sum -= vss_ring_buffer[vss_ring_bufferIndex];
        vss_speed_sum += vss_speed_kmh;
        vss_ring_buffer[vss_ring_bufferIndex] = vss_speed_kmh;
        vss_speed_kmh = 0;
        vss_ring_bufferIndex++;
        if (vss_ring_bufferIndex >= VSS_RINGBUFFER_SIZE)
            vss_ring_bufferIndex = 0;
        retropilotParams.vssAvgSpeedKMH = vss_speed_sum / VSS_RINGBUFFER_SIZE;

    }
}

/* called at ~ 1kHz */
void vss_loop()
{
    vss_update_sensor_readings();
    if (millis()>2000 && millis()-vss_last_wheellock_time<500)
        retropilotParams.OP_WHEELLOCK_DETECTED = true;
    else
        retropilotParams.OP_WHEELLOCK_DETECTED = false;
    
}