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

//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//
// 2024-06-20 高度制御改良　段差対応
// 2024-06-25 高度制御改良　上昇持続バグ修正
// 2024-06-29 自動離陸追加
// 2024-06-29 自動着陸追加
// 2024-06-29 送信機OFFで自動着陸
// 2024-06-29 着陸時、Madgwick Filter Off
// 2024-07-21 flip関数追加、高度センサの測定限界で自動降下（暫定版）
// 2024-08-10 Acroモードで高度制御働かないバグを修正

#include "flight_control.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "button.hpp"
#include "buzzer.h"
#include "elrs.hpp"

// モータPWM出力Pinのアサイン
// Motor PWM Pin
const int pwmFrontLeft  = PIN_MT_FL;
const int pwmFrontRight = PIN_MT_FR;
const int pwmRearLeft   = PIN_MT_RL;
const int pwmRearRight  = PIN_MT_RR;

// モータPWM周波数
// Motor PWM Frequency
const int freq = 150000;

// PWM分解能
// PWM Resolution
const int resolution = 8;

// モータチャンネルのアサイン
// Motor Channel
const int FrontLeft_motor  = 0;
const int FrontRight_motor = 1;
const int RearLeft_motor   = 2;
const int RearRight_motor  = 3;

// 制御周期
// Control period
float Control_period = 0.0025f;  // 400Hz

// PID Gain
// Rate control PID gain
const float Roll_rate_kp  = 0.65f;
const float Roll_rate_ti  = 0.7f;
const float Roll_rate_td  = 0.01;
const float Roll_rate_eta = 0.125f;

const float Pitch_rate_kp  = 0.95f;
const float Pitch_rate_ti  = 0.7f;
const float Pitch_rate_td  = 0.025f;
const float Pitch_rate_eta = 0.125f;

const float Yaw_rate_kp  = 3.0f;
const float Yaw_rate_ti  = 0.8f;
const float Yaw_rate_td  = 0.01f;
const float Yaw_rate_eta = 0.125f;

// Angle control PID gain
const float Roll_angle_kp  = 5.0f;  // 8.0
const float Roll_angle_ti  = 4.0f;
const float Roll_angle_td  = 0.04f;
const float Roll_angle_eta = 0.125f;

const float Pitch_angle_kp  = 5.0f;  // 8.0
const float Pitch_angle_ti  = 4.0f;
const float Pitch_angle_td  = 0.04f;
const float Pitch_angle_eta = 0.125f;

// Altitude control PID gain
const float alt_kp     = 0.38f;  // 5.0//soso 0.5
const float alt_ti     = 10.0f;  // 200.0//soso 10.0
const float alt_td     = 0.5f;   // 0.5//soso 0.5
const float alt_eta    = 0.125f;
const float alt_period = 0.0333;

const float z_dot_kp  = 0.08f;  // 0.35//soso 0.1
const float z_dot_ti  = 0.95f;  // 500.0//soso 0.95
const float z_dot_td  = 0.08f;  // 0.15//1.0//soso 0.08
const float z_dot_eta = 0.125f;

const float Duty_bias_up   = 1.581f;  // Altitude Control parameter　Itolab 1.589 M5Stack 1.581
const float Duty_bias_down = 1.578f;  // Auto landing  parameter Itolab 1.578 M5Stack 1.578

// Times
volatile float Elapsed_time     = 0.0f;
volatile float Old_Elapsed_time = 0.0f;
volatile float Interval_time    = 0.0f;
volatile float Interval_time2   = 0.0f;

volatile uint32_t S_time = 0, E_time = 0, D_time = 0, Dt_time = 0;

// Counter
uint8_t AngleControlCounter   = 0;
uint16_t RateControlCounter   = 0;
uint16_t Auto_takeoff_counter = 0;

// Motor Duty
volatile float FrontRight_motor_duty = 0.0f;
volatile float FrontLeft_motor_duty  = 0.0f;
volatile float RearRight_motor_duty  = 0.0f;
volatile float RearLeft_motor_duty   = 0.0f;

// 制御目標
// PID Control reference
// 角速度目標値
// Rate reference
volatile float Roll_rate_reference = 0.0f, Pitch_rate_reference = 0.0f, Yaw_rate_reference = 0.0f;
// 角度目標値
// Angle reference
volatile float Roll_angle_reference = 0.0f, Pitch_angle_reference = 0.0f, Yaw_angle_reference = 0.0f;
// 舵角指令値
// Commanad
// スロットル指令値
// Throttle
volatile float Thrust_command = 0.0f, Thrust_command2 = 0.0f;
// 角速度指令値
// Rate command
volatile float Roll_rate_command = 0.0f, Pitch_rate_command = 0.0f, Yaw_rate_command = 0.0f;
// 角度指令値
// Angle comannd
volatile float Roll_angle_command = 0.0f, Pitch_angle_command = 0.0f, Yaw_angle_command = 0.0f;

// Offset
volatile float Roll_angle_offset = 0.0f, Pitch_angle_offset = 0.0f, Yaw_angle_offset = 0.0f;
volatile float Elevator_center = 0.0f, Aileron_center = 0.0f, Rudder_center = 0.0f;

// Machine state & flag
//float Timevalue          = 0.0f;
volatile uint8_t Mode    = INIT_MODE;
volatile uint8_t OldMode = INIT_MODE;
uint8_t Control_mode     = ANGLECONTROL;
// volatile uint8_t LockMode=0;
float Motor_on_duty_threshold         = 0.1f;
float Angle_control_on_duty_threshold = 0.5f;
int8_t BtnA_counter                   = 0;
uint8_t BtnA_on_flag                  = 0;
uint8_t BtnA_off_flag                 = 1;
volatile uint8_t Loop_flag            = 0;
// volatile uint8_t Angle_control_flag = 0;
uint8_t Stick_return_flag     = 0;
uint8_t Throttle_control_mode = 0;
uint8_t Landing_state         = 0;
uint8_t OldRange0flag        = 0;

// for flip
float FliRoll_rate_time          = 2.0;
uint8_t Flip_flag                = 0;
uint16_t Flip_counter            = 0;
float Flip_time                  = 2.0;
volatile uint8_t Ahrs_reset_flag = 0;
float T_flip;

// PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
// PID alt;
PID alt_pid;
PID z_dot_pid;
Filter Thrust_filtered;
Filter Duty_fr;
Filter Duty_fl;
Filter Duty_rr;
Filter Duty_rl;

volatile float Thrust0 = 0.0;
uint8_t Alt_flag       = 0;		// 高度維持モード

// 速度目標Z
float Z_dot_ref = 0.0f;

// 高度目標
const float Alt_ref0   = 0.5f;
volatile float Alt_ref = Alt_ref0;

//uint8_t ahrs_reset_flag      = 0;
//uint8_t last_ahrs_reset_flag = 0;

// Function declaration
void init_pwm();
void control_init();
void variable_init(void);
void get_command(void);
void angle_control(void);
void rate_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void motor_stop(void);
//uint8_t judge_mode_change(void);
uint8_t arm_check(void);
uint8_t get_arming_button(void);
uint8_t get_flip_button(void);
void reset_rate_control(void);
void reset_angle_control(void);
uint8_t auto_landing(void);
float get_trim_duty(float voltage);
void flip(void);
float get_rate_ref(float x);
uint8_t check_calib_button(void);
uint8_t stick_function();

float limit(float val, float min, float max){	// 上限下限の範囲内の値にする
  if(val < min) return min;
  if(val > max) return max;
  return val;
}

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer() {
    Loop_flag = 1;
}

// Initialize Multi copter
void init_copter(void) {
    // Initialize Mode
    Mode = INIT_MODE;

    // Initialaze LED function
    led_init();
    esp_led(1);
    onboard_led(0, WHITE);
    onboard_led(1, WHITE);
    led_show();
    led_show();
    led_show();

    // Initialize Serial communication
    USBSerial.begin(420000);	//115200
    delay(1500);
    USBSerial.printf("Start StampFly!\r\n");

    // Initialize PWM
    init_pwm();
    sensor_init();
    USBSerial.printf("Finish sensor init!\r\n");

    // PID GAIN and etc. Init
    control_init();

    // Initilize Radio control
    rc_init();

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);	// 2.5ms 間隔（400Hz）
    timerAlarmEnable(timer);

    // init button G0
    init_button();

    setup_pwm_buzzer();
}


// Main loop
void loop_400Hz(void) {
  static uint8_t led = 1;
  float sense_time;
  uint8_t cmd;

  // 割り込みにより400Hz(2.5ms)で以降のコードが実行
  while (Loop_flag == 0){yield();}
  Loop_flag = 0;

  E_time = micros();
  Interval_time = (E_time - S_time) / 1000000.0f;
  Elapsed_time += Interval_time;
  S_time = E_time;
// USBSerial.printf("Interval_time: %fmS\n", Interval_time * 1000);  // インターバル時間

  sense_time = sensor_read();		// センサー読み込み
//  USBSerial.printf("sense_time:%8.3fmS\n", sense_time * 1000);  // 読み込みにかかった時間。
  uint32_t cs_time = micros();

  led_drive();		// LEDを点灯させる

  Control_mode = Stick[CONTROLMODE];        // Stable/Sports モード
  if ((uint8_t)Stick[ALTCONTROLMODE] == 1)  // 高度維持モード
    Throttle_control_mode = 1;
  else if ((uint8_t)Stick[ALTCONTROLMODE] == 0)
    Throttle_control_mode = 0;
  else
  Throttle_control_mode = 0;
  // if (Interval_time>0.006)USBSerial.printf("%9.6f\n\r", Interval_time);
  // USBSerial.printf("Mode=%d OverG=%d\n\r", Mode, OverG_flag);
  // Begin Mode select
  switch(Mode){
    case INIT_MODE:
      motor_stop();
      Elevator_center    = 0.0f;
      Aileron_center     = 0.0f;
      Rudder_center      = 0.0f;
      Roll_angle_offset  = 0.0f;
      Pitch_angle_offset = 0.0f;
      Yaw_angle_offset   = 0.0f;
      sensor_reset_offset();
      Mode = AVERAGE_MODE;
      break;;

    case AVERAGE_MODE:  // ジャイロの平均値を得る
      motor_stop();
      // Gyro offset Estimate
      if ( sensor_calc_offset_avarage() > AVERAGENUM) { // オフセット平均値を得る
        Mode = PARKING_MODE;
        S_time = micros();
      }
      break;

    case FLIGHT_MODE:
      Control_period = Interval_time;
      // Judge Mode change
      if (arm_check() == 1) Mode = AUTO_LANDING_MODE;   // ARMボタンが押されたら自動着地
      if (OverG_flag == 1) Mode = PARKING_MODE;         // 衝撃が加わったらモーターを止める
//    if (Mode != OldMode) ahrs_reset();

      get_command();    // リモコンデータ解析
      angle_control();  // 姿勢制御
      rate_control();   // モーター制御
      break;
	
    case FLIP_MODE:
      flip();
      break;

    case PARKING_MODE:
      motor_stop();
      OverG_flag = 0;
      Thrust0    = 0.0;
      Alt_flag   = 0;
      // flip reset
      Roll_rate_reference  = 0;
      Ahrs_reset_flag      = 0;
      Flip_counter         = 0;
      Flip_flag            = 0;
      Range0flag           = 0;
      Alt_ref              = Alt_ref0;
      Stick_return_flag    = 0;
      Landing_state        = 0;
      Auto_takeoff_counter = 0;
      Thrust_filtered.reset();
      EstimatedAltitude.reset();
      Duty_fr.reset();
      Duty_fl.reset();
      Duty_rr.reset();
      Duty_rl.reset();
      if(Mode != OldMode) ahrs_reset();
			
      if (arm_check() == 1) {     // ARM ボタン　判定
        buzzer_sound(2000, 200);		
        Mode = FLIGHT_MODE;
      }
      break;

    case AUTO_LANDING_MODE:
      if (auto_landing() == 1) Mode = PARKING_MODE;
      if (arm_check() == 1) Mode = PARKING_MODE;
      angle_control();  // Angle Control
      rate_control();   // Rate Control
      break;
  }

  telemetry();  // テレメトリーデータ送信

  uint32_t ce_time = micros();
  Dt_time = ce_time - cs_time;
  OldMode = Mode;  // Memory now mode
		
// End of Loop_400Hz function
// USBSerial.printf("Dt_time: %d us\n",Dt_time);
}

void flip(void) {
    float domega;
    float flip_delay;
    uint16_t flip_step;

    Control_period = Interval_time;
    Led_color      = FLIPCOLOR;

    // Judge Mode change
    if (arm_check() == 1) Mode = AUTO_LANDING_MODE;
    if (rc_isconnected() == 0) Mode = AUTO_LANDING_MODE;
    if (OverG_flag == 1) Mode = PARKING_MODE;

    // Flip parameter set
    Flip_time            = 0.4;
    Pitch_rate_reference = 0.0;
    domega               = 0.00217f * 8.0 * PI / Flip_time / Flip_time;  // 25->22->23->225->222->221->220
    flip_delay           = 180;
    flip_step            = (uint16_t)(Flip_time / 0.0025f);
    T_flip               = get_trim_duty(Voltage) * BATTERY_VOLTAGE;

    // Flip Sequence
    if (Flip_counter < flip_delay)  // 一時的な上昇
    {
        Flip_flag = 1;
        // Roll_rate_reference = 0.0f;
        if (Voltage > 3.8)
            Thrust_command = T_flip + 0.17 * BATTERY_VOLTAGE;	// スロットル値
        else
            Thrust_command = T_flip + 0.15 * BATTERY_VOLTAGE;
        // Angle Control
        Roll_angle_command  = 0.0;
        Pitch_angle_command = 0.0;
        angle_control();	// 姿勢設定
        // Rate Control
        Yaw_rate_command = 0.0;
        rate_control();		// モーター制御
        Flip_counter++;
    } else if (Flip_counter < (flip_step / 4 + flip_delay))  // 宙返り開始(0deg-90deg)
    {
        Flip_flag      = 2;
        Thrust_command = T_flip * 0.3f;  // 1.05//0.4　スロットル値
        // Rate Control
        Roll_rate_reference += domega;
        Pitch_rate_command  = 0.0;
        Yaw_rate_command    = 0.0;
        rate_control();		// モーター制御
        Flip_counter++;
    } else if (Flip_counter < (2 * flip_step / 4 + flip_delay))  // 宙返り(90deg-180deg)
    {
        Flip_flag      = 3;
        Thrust_command = T_flip * 0.15f;  // 1.0//0.2
        // Rate Control
        Roll_rate_reference += domega;
        Pitch_rate_command  = 0.0f;
        Yaw_rate_command    = 0.0f;
        rate_control();		// モーター制御
        Flip_counter++;
    } else if (Flip_counter < (3 * flip_step / 4 + flip_delay))  // 宙返り(180deg-270deg)
    {
        Flip_flag      = 4;
        Thrust_command = T_flip * 0.15f;  // 1.0//0.2
        // Rate Control
        Roll_rate_reference = Roll_rate_reference - domega;
        Pitch_rate_command  = 0.0f;
        Yaw_rate_command    = 0.0f;
        rate_control();
        Flip_counter++;
    } else if (Flip_counter < (flip_step + flip_delay))  // 宙返り(270deg-360deg)
    {
        Flip_flag      = 5;
        Thrust_command = T_flip * 1.0f;
        // Rate Control
        Roll_rate_reference = Roll_rate_reference - domega;
        Pitch_rate_command  = 0.0f;
        Yaw_rate_command    = 0.0f;
        rate_control();
        Flip_counter++;
    } else if (Flip_counter < (flip_step + flip_delay + 10))  // 元に戻す準備
    {
        Flip_flag = 6;
        if (Ahrs_reset_flag == 0) {
            Ahrs_reset_flag = 1;
            ahrs_reset();
        }
        Thrust_command = T_flip + 0.5f * BATTERY_VOLTAGE;	// 0.18f
        // Rate Control
        Roll_rate_reference = 0.0f;
        Pitch_rate_command  = 0.0f;
        Yaw_rate_command    = 0.0f;
        rate_control();

        // Angle PID Reset
        phi_pid.reset();
        theta_pid.reset();

        Flip_counter++;
    } else if (Flip_counter < (flip_step + flip_delay + 200))  // 連続Flipの抑制
    {
        Flip_flag = 0;
        // Get command
        get_command();
        // Angle Control
        angle_control();
        // Rate Control
        rate_control();
        Flip_counter++;
    } else {
        // Return to Flight Mode
        Flip_flag       = 0;
        Ahrs_reset_flag = 0;
        Flip_counter    = 0;
        Mode            = FLIGHT_MODE;
    }
}

//uint8_t judge_mode_change(void) {
uint8_t arm_check(void) {	
    // Ariming Button が押されて離されたかを確認
    uint8_t state;
    static uint8_t chatter = 0;
    state                  = 0;
    if (chatter == 0) {
        if (get_arming_button() == 1) {
            chatter = 1;
        }
    } else {
        if (get_arming_button() == 0) {
            chatter++;
            if (chatter > 40) {
                chatter = 0;
                state   = 1;
            }
        }
    }
    return state;
}

///////////////////////////////////////////////////////////////////
//  PID control gain setting
//
//  Sets the gain of PID control.
//
//  Function usage
//  PID.set_parameter(PGAIN, IGAIN, DGAIN, TC, STEP)
//
//  PGAIN: PID Proportional Gain
//  IGAIN: PID Integral Gain
//   *The larger the value of integral gain, the smaller the effect of integral control.
//  DGAIN: PID Differential Gain
//  TC:    Time constant for Differential control filter
//  STEP:  Control period
//
//  Example
//  Set roll rate control PID gain
//  p_pid.set_parameter(2.5, 10.0, 0.45, 0.01, 0.001);

void control_init(void) {
    // Rate control
    p_pid.set_parameter(Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta,
                        Control_period);  // Roll rate control gain
    q_pid.set_parameter(Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta,
                        Control_period);  // Pitch rate control gain
    r_pid.set_parameter(Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta, Control_period);  // Yaw rate control gain

    // Angle control
    phi_pid.set_parameter(Roll_angle_kp, Roll_angle_ti, Roll_angle_td, Roll_angle_eta,
                          Control_period);  // Roll angle control gain
    theta_pid.set_parameter(Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta,
                            Control_period);  // Pitch angle control gain

    // Altitude control
    alt_pid.set_parameter(alt_kp, alt_ti, alt_td, alt_eta, alt_period);
    z_dot_pid.set_parameter(z_dot_kp, z_dot_ti, z_dot_td, alt_eta, alt_period);

    Duty_fl.set_parameter(0.003, Control_period);
    Duty_fr.set_parameter(0.003, Control_period);
    Duty_rl.set_parameter(0.003, Control_period);
    Duty_rr.set_parameter(0.003, Control_period);

    Thrust_filtered.set_parameter(0.01, Control_period);
}
///////////////////////////////////////////////////////////////////

float get_trim_duty(float voltage) {
    return -0.2448f * voltage + 1.5892f;
}

float get_rate_ref(float x) {
    float ref;
    float h;
    x = limit(x, -1.0f, 1.0f);
    if (x >= 0.0f) {
        h   = x * (x * x * x * x * x * RATE_EXPO + x * (1 - RATE_EXPO));
        ref = PI / 180.0f * ((RATE_MAX - RATE_RATE) * h + RATE_RATE * x);
    } else {
        x   = -x;
        h   = x * (x * x * x * x * x * RATE_EXPO + x * (1 - RATE_EXPO));
        ref = -PI / 180.0f * ((RATE_MAX - RATE_RATE) * h + RATE_RATE * x);
    }
    return ref;
}

void getRPY(bool nocheck){	   // Get Roll Pitch Yaw command
   // Get RPY command	
  if (nocheck || Control_mode == ANGLECONTROL) {  // アングルモードなら目標角度を指定
    Roll_angle_command = 0.4 * Stick[AILERON];
    Roll_angle_command = limit(Roll_angle_command, -1.0f, 1.0f);

    Pitch_angle_command = 0.4 * Stick[ELEVATOR];
    Pitch_angle_command = limit(Pitch_angle_command, -1.0f, 1.0f);	
  }
  if (Control_mode == RATECONTROL) {              // レートモードなら目標角速度（回転速度）を指定
    Roll_rate_reference  = get_rate_ref(Stick[AILERON]);
    Pitch_rate_reference = get_rate_ref(Stick[ELEVATOR]);
  }
  Yaw_angle_command = Stick[RUDDER];
  Yaw_angle_command = limit(Yaw_angle_command, -1.0f, 1.0f);	
  Yaw_rate_reference = 2.0f * PI * (Yaw_angle_command - Rudder_center);
}

void get_command(void) {
  static uint16_t stick_count = 0;
  static float auto_throttle  = 0.0f;
  static float old_alt        = 0.0;
  float th, thlo;
  float throttle_limit = 0.7;
  float thrust_max;

  // Thrust control
  thlo = Stick[THROTTLE];
  // thlo = thlo/throttle_limit;

  if (Throttle_control_mode == 0){  // 高度維持 マニュアルモード
    // Manual Throttle
    thlo = limit(thlo, 0.0f, 1.0f);
    if ((-0.2 < thlo) && (thlo < 0.2)) thlo = 0.0f;  // 不感帯
    th = (get_trim_duty(Voltage) + (thlo - 0.4)) * BATTERY_VOLTAGE;
    if (th < 0) th = 0.0f;
    Thrust_command = Thrust_filtered.update(th, Interval_time);
  } else if (Throttle_control_mode == 1) {  // 高度維持 AUTOモード
    // Auto Throttle Altitude Control
    Alt_flag = 1;
    if (Auto_takeoff_counter < 500) 
    {
      Thrust0 = (float)Auto_takeoff_counter / 1000.0;
      if (Thrust0 > get_trim_duty(3.8)){
        Thrust0 = get_trim_duty(3.8);
      }
      Auto_takeoff_counter++;
    } else if (Auto_takeoff_counter < 1000)
    {
      Thrust0 = (float)Auto_takeoff_counter / 1000.0;
      if (Thrust0 > get_trim_duty(Voltage)){
        Thrust0 = get_trim_duty(Voltage);
      }
      Auto_takeoff_counter++;
    } else{
      Thrust0 = get_trim_duty(Voltage);
    }
    // Get Altitude ref
    if ((-0.2 < thlo) && (thlo < 0.2)){ // 不感帯
      thlo = 0.0f;
    }
    Alt_ref = Alt_ref + thlo * 0.001;
    Alt_ref = limit(Alt_ref, ALT_REF_MIN,  ALT_REF_MAX);

    if ((Range0flag > OldRange0flag) || (Range0flag == RNAGE0FLAG_MAX)) { // 高度を下げる
      Thrust0 = Thrust0 - 0.02;
      OldRange0flag = Range0flag;
    }
    Thrust_command = Thrust0 * BATTERY_VOLTAGE;
  }

  getRPY(false);  // get Roll Pitch Yow

  // flip button check
  if (Flip_flag == 0 /*&& Throttle_control_mode == 0*/) {
    Flip_flag = get_flip_button();
    if (Flip_flag == 1){
      Mode = FLIP_MODE;
    }
  }
}

uint8_t auto_landing(void) {
    // Auto Landing
    uint8_t flag;
    static float auto_throttle;
    static uint16_t counter = 0;
    static float old_alt[10];
    float thrust_max;

    Alt_flag = 0;
    if (Landing_state == 0) {
        Landing_state = 1;
        counter       = 0;
        for (uint8_t i = 0; i < 10; i++) old_alt[i] = Altitude2;
        Thrust0 = get_trim_duty(Voltage);
    }
    if (old_alt[9] >= Altitude2)  // もし降下しなかったら、スロットル更に下げる
    {
        Thrust0 = Thrust0 * 0.9999;
    }
    if (Altitude2 < 0.15)  // 地面効果で降りなかった場合対策
    {
        Thrust0 = Thrust0 * 0.999;
    }
    if (Altitude2 < 0.1) {
        flag          = 1;
        Landing_state = 0;
    } else
        flag = 0;

    for (int i = 1; i < 10; i++) old_alt[i] = old_alt[i - 1];
    old_alt[0] = Altitude2;

    // Thrust_command = Thrust_filtered.update(auto_throttle*BATTERY_VOLTAGE, Interval_time);

    getRPY(true);	// Get RPY command

    return flag;
}

void rate_control(void) {
    float p_rate, q_rate, r_rate;
    float p_ref, q_ref, r_ref;
    float p_err, q_err, r_err, z_dot_err;

    // Rate Control
    if ((Thrust_command / BATTERY_VOLTAGE < Motor_on_duty_threshold) && (Flip_flag == 0)) {
        reset_rate_control();
    } else {
        // Control angle velocity
        p_rate = Roll_rate;
        q_rate = Pitch_rate;
        r_rate = Yaw_rate;

        // Get reference
        p_ref = Roll_rate_reference;
        q_ref = Pitch_rate_reference;
        r_ref = Yaw_rate_reference;

        // Error
        p_err = p_ref - p_rate;
        q_err = q_ref - q_rate;
        r_err = r_ref - r_rate;

        // Rate Control PID
        Roll_rate_command  = p_pid.update(p_err, Interval_time);
        Pitch_rate_command = q_pid.update(q_err, Interval_time);
        Yaw_rate_command   = r_pid.update(r_err, Interval_time);

        // Altutude Control
        if (Alt_flag == 1 && Flip_flag == 0) {
            z_dot_err      = Z_dot_ref - Alt_velocity;
            Thrust_command = Thrust_filtered.update(
                (Thrust0 + z_dot_pid.update(z_dot_err, Interval_time)) * BATTERY_VOLTAGE, Interval_time);
            Thrust_command = limit( Thrust_command, BATTERY_VOLTAGE * Thrust0 * 0.85f, BATTERY_VOLTAGE * Thrust0 * 1.15f);		
        } else if (Mode == AUTO_LANDING_MODE) {
            z_dot_err      = -0.15 - Alt_velocity;
            Thrust_command = Thrust_filtered.update(
                (Thrust0 + z_dot_pid.update(z_dot_err, Interval_time)) * BATTERY_VOLTAGE, Interval_time);
//					Thrust_command = limit( Thrust_command, BATTERY_VOLTAGE * Thrust0 * 0.9f, BATTERY_VOLTAGE * Thrust0 * 1.1f);
        }

        // Motor Control
        // 正規化Duty
        FrontRight_motor_duty = Duty_fr.update(
            (Thrust_command + (-Roll_rate_command + Pitch_rate_command + Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);
        FrontLeft_motor_duty = Duty_fl.update(
            (Thrust_command + (Roll_rate_command + Pitch_rate_command - Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);
        RearRight_motor_duty = Duty_rr.update(
            (Thrust_command + (-Roll_rate_command - Pitch_rate_command - Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);
        RearLeft_motor_duty = Duty_rl.update(
            (Thrust_command + (Roll_rate_command - Pitch_rate_command + Yaw_rate_command) * 0.25f) / BATTERY_VOLTAGE,
            Interval_time);

        const float minimum_duty = 0.0f;
        const float maximum_duty = 0.95f;

        if (FrontRight_motor_duty < minimum_duty) FrontRight_motor_duty = minimum_duty;
        if (FrontRight_motor_duty > maximum_duty) FrontRight_motor_duty = maximum_duty;

        if (FrontLeft_motor_duty < minimum_duty) FrontLeft_motor_duty = minimum_duty;
        if (FrontLeft_motor_duty > maximum_duty) FrontLeft_motor_duty = maximum_duty;

        if (RearRight_motor_duty < minimum_duty) RearRight_motor_duty = minimum_duty;
        if (RearRight_motor_duty > maximum_duty) RearRight_motor_duty = maximum_duty;

        if (RearLeft_motor_duty < minimum_duty) RearLeft_motor_duty = minimum_duty;
        if (RearLeft_motor_duty > maximum_duty) RearLeft_motor_duty = maximum_duty;

        // Duty set
        if (OverG_flag == 0) {
            set_duty_fr(FrontRight_motor_duty);
            set_duty_fl(FrontLeft_motor_duty);
            set_duty_rr(RearRight_motor_duty);
            set_duty_rl(RearLeft_motor_duty);
        } else {
            FrontRight_motor_duty = 0.0;
            FrontLeft_motor_duty  = 0.0;
            RearRight_motor_duty  = 0.0;
            RearLeft_motor_duty   = 0.0;
            motor_stop();
            // OverG_flag=0;
            Mode = PARKING_MODE;
        }
    }
}

void reset_rate_control(void) {
    motor_stop();
    FrontRight_motor_duty = 0.0;
    FrontLeft_motor_duty  = 0.0;
    RearRight_motor_duty  = 0.0;
    RearLeft_motor_duty   = 0.0;
    Duty_fr.reset();
    Duty_fl.reset();
    Duty_rr.reset();
    Duty_rl.reset();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    alt_pid.reset();
    z_dot_pid.reset();
    Roll_rate_reference  = 0.0f;
    Pitch_rate_reference = 0.0f;
    Yaw_rate_reference   = 0.0f;
    Rudder_center        = Yaw_angle_command;
    // angle control value reset
    Roll_rate_reference  = 0.0f;
    Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Flip_flag          = 0;
    Flip_counter       = 0;
    Roll_angle_offset  = 0;
    Pitch_angle_offset = 0;
}

void reset_angle_control(void) {
    Roll_rate_reference  = 0.0f;
    Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    // Alt_ref_filter.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    Flip_flag    = 0;
    Flip_counter = 0;
    /////////////////////////////////////
    // 以下の処理で、角度制御が有効になった時に
    // 急激な目標値が発生して機体が不安定になるのを防止する
    Aileron_center     = Roll_angle_command;
    Elevator_center    = Pitch_angle_command;
    Roll_angle_offset  = 0;
    Pitch_angle_offset = 0;
    /////////////////////////////////////
}

void angle_control(void) {
    float phi_err, theta_err, alt_err;
    static uint8_t cnt   = 0;
    static float timeval = 0.0f;
    // flip
    uint16_t flip_delay = 150;
    uint16_t flip_step;
    float domega;

    // PID Control
    if (Thrust_command / BATTERY_VOLTAGE < Motor_on_duty_threshold){ // スロットル指令値が小さかったら
        reset_angle_control();  // 各種データクリア
    }else{
        // Altitude Control PID
        alt_err = Alt_ref - Altitude2;  // 目標高度と現在高度との差
        if (Alt_flag >= 1) Z_dot_ref = alt_pid.update(alt_err, Interval_time);  // 上昇速度を目標との差のPIDで求める

        if (Control_mode == ANGLECONTROL) {	// アングルモードなら
           // Get Roll and Pitch angle ref
            Roll_angle_reference  = 0.5f * PI * (Roll_angle_command - Aileron_center);    // 角度指令値(-1.0~1.0)から角度目標値(-90°～90°)に変換（単位はラジアン）
            Pitch_angle_reference = 0.5f * PI * (Pitch_angle_command - Elevator_center);  // 角度指令値(-1.0~1.0)から角度目標値(-90°～90°)に変換（単位はラジアン）
            Roll_angle_reference = limit(Roll_angle_reference, -30.0f * PI / 180.0f, 30.0f * PI / 180.0f);		// 角度目標値を +-30°に制限（単位はラジアン）
            Pitch_angle_reference = limit(Pitch_angle_reference, -30.0f * PI / 180.0f, 30.0f * PI / 180.0f);	// 角度目標値を +-30°に制限（単位はラジアン）
            // Error
            phi_err   = Roll_angle_reference - (Roll_angle - Roll_angle_offset);    // 現在の角度と目標角度との差 
            theta_err = Pitch_angle_reference - (Pitch_angle - Pitch_angle_offset); // 現在の角度と目標角度との差
            // Angle Control PID
            Roll_rate_reference  = phi_pid.update(phi_err, Interval_time);     // 角速度（回転速度）目標値を差の値のPIDで求める
            Pitch_rate_reference = theta_pid.update(theta_err, Interval_time); // 角速度（回転速度）目標値を差の値のPIDで求める
        }
    }
}

void set_duty_fr(float duty) {
    ledcWrite(FrontRight_motor, (uint32_t)(255 * duty));
}
void set_duty_fl(float duty) {
    ledcWrite(FrontLeft_motor, (uint32_t)(255 * duty));
}
void set_duty_rr(float duty) {
    ledcWrite(RearRight_motor, (uint32_t)(255 * duty));
}
void set_duty_rl(float duty) {
    ledcWrite(RearLeft_motor, (uint32_t)(255 * duty));
}

void init_pwm(void) {
    ledcSetup(FrontLeft_motor, freq, resolution);
    ledcSetup(FrontRight_motor, freq, resolution);
    ledcSetup(RearLeft_motor, freq, resolution);
    ledcSetup(RearRight_motor, freq, resolution);
    ledcAttachPin(pwmFrontLeft, FrontLeft_motor);
    ledcAttachPin(pwmFrontRight, FrontRight_motor);
    ledcAttachPin(pwmRearLeft, RearLeft_motor);
    ledcAttachPin(pwmRearRight, RearRight_motor);
}

uint8_t get_arming_button(void) {
    static int8_t chatta = 0;
    static uint8_t state = 0;
    if ((int)Stick[BUTTON_ARM] == 1) {
        chatta++;
        if (chatta > 10) {
            chatta = 10;
            state  = 1;
        }
    } else {
        chatta--;
        if (chatta < -10) {
            chatta = -10;
            state  = 0;
        }
    }
    return state;
}

uint8_t get_flip_button(void) {
    static int8_t chatta = 0;
    uint8_t state;

    state = 0;
    if ((int)Stick[BUTTON_FLIP] == 1) {
        chatta++;
        if (chatta > 10) {
            chatta = 0;
            state  = 1;
        }
    } else {
        chatta = 0;
        state  = 0;
    }
    return state;
}

void motor_stop(void) {
    set_duty_fr(0.0);
    set_duty_fl(0.0);
    set_duty_rr(0.0);
    set_duty_rl(0.0);
}

int stick_3pos(float p){
	int pos = -2;
	if( p < -0.7f) pos = -1;
	else if( p > -0.2 && p < 0.2f) pos = 0;
	else if( p > 0.7f) pos = 1;
	return pos;
}

int stick_2pos(float p){
	int pos = -2;
	if( p < 0.15f) pos = -1;
	else if( p > 0.4f && p < 0.6f) pos = 0;
	else if( p > 0.85f) pos = 1;
	return pos;
}

uint8_t stick_function(){
	static int8_t cnt = 0;	
	static int a0,e0,t0,r0;
	uint8_t cmd = 0;
	int a,e,t,r;

	a = stick_3pos(Stick[AILERON]);
	e = stick_3pos(Stick[ELEVATOR]);
	t = stick_2pos(Stick[THROTTLE]);
	r = stick_3pos(Stick[RUDDER]);

//	USBSerial.printf("a:%d e:%d t:%d r:%d chatta:%d\n",a,e,t,r,chatta);

	if(a0 == a && e0 == e && t0 == t && r0 == r){
		cnt ++;
		if(cnt > 10){
			if( t == -1 && r ==  1 && e ==  0 && a == 0 ) cmd = 1;	// Arm ↘・
			if( t == -1 && r == -1 && e ==  0 && a == 0 ) cmd = 2;	// DisArm　↙・			
			if( t == -1 && r == -1 && e == -1 && a == 0 ) cmd = 3;	// Calibration　↙ ↓
			cnt = 0;
		}
	}else{
		cnt = 0;
	}
	a0 = a;
	e0 = e;
	t0 = t;
	r0 = r;
	return cmd;
}