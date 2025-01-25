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

#ifndef LED_HPP
#define LED_HPP

#include <stdint.h>

#define WHITE         0xffffff
#define BLUE          0x0000ff
#define RED           0xff0000
#define YELLOW        0xffff00
#define GREEN         0x00ff00
#define PERPLE        0xff00ff
#define POWEROFFCOLOR 0x18EBF9
#define FLIPCOLOR     0xFF9933

#define NUM_LEDS       1

extern uint32_t Led_color;

void led_init(void);
void led_brightness(uint8_t b);
void led_show(void);
void led_drive(void);
void onboard_led(int no, uint32_t col);
void esp_led(uint32_t col);
void setCcolor(uint32_t col);

#endif