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
#include "flight_control.hpp"
#include "elrs.hpp"
#include "rc.hpp"
#include "sensor.hpp"
#include "buzzer.h"

volatile uint8_t Loop_flag2;
hw_timer_t* timer2 = NULL;

void IRAM_ATTR onTimer2() {
  Loop_flag2 = 1;
}

void loop1()
{
	elrs_update();	// ELRS 受信

	if(Loop_flag2 == 1){	// 2.5ms周期
		Loop_flag2 = 0;
		sensor_read2();			// I2C関連センサー読み込み
	}
}

void setup1(void *arg)
{
	elrs_init();		// ELRS受信開始
	sensor_init2();	// I2C関連の初期化

	// Initialize intrupt
	timer2 = timerBegin(1, 80, true);
	timerAttachInterrupt(timer2, &onTimer2, true);
	timerAlarmWrite(timer2, 2500, true);							// 2.5ms 間隔（400Hz）loop1内で使う
	timerAlarmEnable(timer2);

	USBSerial.printf("Finish StampFly init!\r\n");
	USBSerial.printf("Enjoy Flight!\r\n");
	start_tone();		// 起動音を鳴らす
	
	while(1){
		loop1();			// ELRS受信 と　I2C接続センサーの読み込み
	}
}

void setup() {
	init_copter();
	
	xTaskCreateUniversal(setup1, "core0", 8192, NULL, 1, NULL, 0 );			// Core0でマルチコア起動
	delay(1000);	
}

void loop() {			// Core1 ループ
	loop_400Hz();		// フライトコントロール　SPI接続センサー(IMU と　オプティカルフロー)
}
