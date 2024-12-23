/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "tof.hpp"

VL53LX_Dev_t tof_front;
VL53LX_Dev_t tof_bottom;

VL53LX_DEV ToF_front  = &tof_front;
VL53LX_DEV ToF_bottom = &tof_bottom;

volatile uint8_t ToF_bottom_data_ready_flag;
volatile uint8_t ToF_front_data_ready_flag;

void IRAM_ATTR tof_bottom_int() {
  ToF_bottom_data_ready_flag = 1;
}

void IRAM_ATTR tof_front_int() {
  ToF_front_data_ready_flag = 1;
}

int16_t tof_bottom_get_range() {
	int16_t range = 0;
 	range = tof_range_get(&tof_bottom);
	return range;
}

int16_t tof_front_get_range() {
	int16_t range = 0;	
	range = tof_range_get(&tof_front);
	return range;
}

void tof_init(void) {	
	uint8_t byteData;
	uint16_t wordData;
	
	tof_bottom.comms_speed_khz   = 400;
	tof_bottom.i2c_slave_address = 0x29;

	tof_front.comms_speed_khz   = 400;
	tof_front.i2c_slave_address = 0x29;
	
	vl53lx_i2c_init();
  
	// ToF Pin Initialize
	pinMode(XSHUT_BOTTOM, OUTPUT);
	pinMode(XSHUT_FRONT, OUTPUT);
	pinMode(INT_BOTTOM, INPUT);
	pinMode(INT_FRONT, INPUT);

	digitalWrite(XSHUT_FRONT, LOW);		// Front ToF Disable
	digitalWrite(XSHUT_BOTTOM, HIGH);	// Bottom ToF Enable
	delay(100);

	VL53LX_SetDeviceAddress(ToF_bottom, 0x54);	// ボトムToFアドレス変更。フロントToFはバッテリーを繋がないと動かないので、そちらをデフォルトアドレスにする。 
	ToF_bottom->i2c_slave_address = 0x2A;
	digitalWrite(XSHUT_FRONT, HIGH);	// Front T0F Enable
	delay(100);

	// Bottom ToF setting
	VL53LX_WaitDeviceBooted(ToF_bottom);
	VL53LX_DataInit(ToF_bottom);
	VL53LX_SetDistanceMode(ToF_bottom, VL53LX_DISTANCEMODE_MEDIUM);
	VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_bottom, 33000);
	attachInterrupt(INT_BOTTOM, &tof_bottom_int, FALLING);

// Front ToF Setting
	VL53LX_WaitDeviceBooted(ToF_front);
	VL53LX_DataInit(ToF_front);
	VL53LX_SetDistanceMode(ToF_front, VL53LX_DISTANCEMODE_LONG);
	VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_front, 33000);
	attachInterrupt(INT_FRONT, &tof_front_int, FALLING);

	VL53LX_StartMeasurement(ToF_bottom);		
	delay(11);															// スタートをちょっと遅らせる。（同時だと上手く動かない時がある）
	VL53LX_StartMeasurement(ToF_front);	
}

int16_t tof_range_get(VL53LX_DEV dev) {
	int16_t range;
	int16_t range_min;
	int16_t range_max;
	int16_t range_ave;
	uint8_t count;
	VL53LX_MultiRangingData_t MultiRangingData;

	VL53LX_GetMultiRangingData(dev, &MultiRangingData);
	uint8_t no_of_object_found = MultiRangingData.NumberOfObjectsFound;
	range_min = 10000;
	range_max = 0;
	range_ave = 0;

	if (no_of_object_found == 0) {
		range_min = 9999;
		range_max = 0;
	} else {
		count = 0;
		for (uint8_t j = 0; j < no_of_object_found; j++) {
			if (MultiRangingData.RangeData[j].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID) {
				count++;
				range = MultiRangingData.RangeData[j].RangeMilliMeter;
				if (range_min > range) range_min = range;
				if (range_max < range) range_max = range;
				range_ave = range_ave + range;
			}
		}
		if (count != 0) range_ave = range_ave / count;
//		USBSerial.printf("Min:%dmm Max:%dmm Ave:%dmm\n\r", range_min, range_max, range_ave);
	}
	VL53LX_ClearInterruptAndStartMeasurement(dev);	// 計測再開
	return range_max;
}
