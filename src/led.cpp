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
#include <Adafruit_NeoPixel.h>
#include <stdint.h>
#include "sensor.hpp"
#include "flight_control.hpp"
#include "led.hpp"


#define STRIP_COUNT 2 //つなぐLEDの数
Adafruit_NeoPixel led_strip(STRIP_COUNT, PIN_LED_ONBORD, NEO_GRB + NEO_KHZ800);	// LED定義

uint32_t Led_color       = 0x000000;
uint32_t Led_color2      = 255;
uint16_t LedBlinkCounter = 0;
uint32_t led_onboard[2];

void setCcolor(uint32_t col){
	led_onboard[0] = col;
	led_onboard[1] = col;	
}

void onboard_led(int no, uint32_t col){
	led_onboard[no] = col;
}

void esp_led(uint8_t state) {
    if (state == 1)
			digitalWrite(PIN_LED_ESP, LOW);
    else
			digitalWrite(PIN_LED_ESP, HIGH);		
    return;
}

void led_init(void) {
	pinMode(PIN_LED_ESP, OUTPUT);
	esp_led(1);	//On
	led_strip.setBrightness(30);
	led_strip.begin();
}

void led_brightness(uint8_t b){
	led_strip.setBrightness(b);
}

void led_show(void) {
	for(int i=0; i<STRIP_COUNT; i++){
		led_strip.setPixelColor(i, led_onboard[i]);
	}
	led_strip.show();
}

void led_drive(void) {
    if (Mode == AVERAGE_MODE) {
			setCcolor(PERPLE);
    } else if (Mode == AUTO_LANDING_MODE) {
				setCcolor(GREEN);
    } else if (Mode == FLIGHT_MODE) {
        if (Control_mode == ANGLECONTROL) {
            if (Flip_flag == 0)
                Led_color = YELLOW;  // スタビライズモード・マニュアル飛行では黄色
            else
                Led_color = FLIPCOLOR;  // 宙返りではオレンジ
        } else
            Led_color = 0xDC669B;  // アクロモード　べに色

        if (Throttle_control_mode == 1) Led_color = 0xc71585;  // 高度制御初期　赤紫
        if (Alt_flag >= 1) Led_color = 0x331155;               // 高度制御モードでは深紫

        if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) {
					setCcolor(Led_color);
        } else {
					onboard_led(0, POWEROFFCOLOR);
					onboard_led(1, Led_color);
        }
    } else if (Mode == PARKING_MODE) {
        if (Under_voltage_flag < UNDER_VOLTAGE_COUNT) {
            // イルミネーション
            if (LedBlinkCounter == 0) {  //<10
                if (Led_color2 & 0x800000)
                    Led_color2 = (Led_color2 << 1) | 1;
                else
                    Led_color2 = Led_color2 << 1;
								setCcolor(Led_color2);		
                LedBlinkCounter++;
            }
            LedBlinkCounter++;
            if (LedBlinkCounter > 20) LedBlinkCounter = 0;
        } else {				// バッテリー不足
            // 水色点滅
            if (LedBlinkCounter < 10) {
							setCcolor(POWEROFFCOLOR);
            } else if (LedBlinkCounter < 200) {
  						setCcolor(0);
            } else
                LedBlinkCounter = 0;
            LedBlinkCounter++;
        }
    }
    led_show();		// LEDを点灯させる
}


