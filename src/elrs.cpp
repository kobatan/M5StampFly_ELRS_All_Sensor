#include <Arduino.h>
#include "sensor.hpp"
#include "elrs.hpp"

volatile float Stick[16];
uint8_t rxbuf[CRSF_MAX_PACKET_LEN + 3];	// 受信した生データ
uint16_t ch[CRSF_NUM_CHANNELS];					// チャンネルデータ
uint8_t rxPos = 0;
uint32_t gaptime;								// フレーム区切り測定用
bool datardyf = false;					// データが揃った。
uint8_t _lut[256];							// CRC計算用

void onReceiveRcChannels()	// ELRS受信チャンネルをスティックに振り分ける
{
    Stick[AILERON]				= 2.0 * (float)(ch[RC_CH_AILERON] - AILERON_MID)/(float)(AILERON_MAX - AILERON_MIN);			// -1 ~ 1
    Stick[ELEVATOR]				= -2.0 * (float)(ch[RC_CH_ELEVATOR] - ELEVATOR_MID)/(float)(ELEVATOR_MAX - ELEVATOR_MIN);	// -1 ~ 1 		(＋－反転)
    Stick[THROTTLE]				= 1.0 * (float)(ch[RC_CH_THORTTLE] - THROTTLE_MID)/(float)(THROTTLE_MAX - THROTTLE_MIN);	// 0 ~ 1 に変更
    Stick[RUDDER]					= 2.0 * (float)(ch[RC_CH_RUFFER] - RUDDER_MID)/(float)(RUDDER_MAX - RUDDER_MIN);					// -1 ~ 1
    Stick[BUTTON_ARM] 		= (uint8_t)(ch[RC_CH_ARM] > 1600);		// ARM  (モーメンタリSWに割り当てる)
    Stick[CONTROLMODE] 		= (uint8_t)(ch[RC_CH_MODE] > 1600);		// Stable(Angle)/Sports(Acro) モード切替
    Stick[ALTCONTROLMODE] = (uint8_t)(ch[RC_CH_ALTCNT] > 1600);	// 高度制御 (Manual/Auto)
    Stick[BUTTON_FLIP] 		= (uint8_t)(ch[RC_CH_FLIP] > 1600);		// フリップする　(モーメンタリSWに割り当てる)
		Stick[LOG] 						= (uint8_t)(ch[RC_CH_LOG] > 1600);		// 
/*
   USBSerial.printf("%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f  %6.3f\n\r", 
                                                Stick[THROTTLE],
                                                Stick[AILERON],
                                                Stick[ELEVATOR],
                                                Stick[RUDDER],
                                                Stick[BUTTON_ARM],
                                                Stick[BUTTON_FLIP],
                                                Stick[CONTROLMODE],
                                                Stick[ALTCONTROLMODE],
                                                Stick[LOG]);
*/
}

// CRSFから受信した11bitシリアルデータを16bitデータにデコード
void crsfdecode () {
	crsf_frame *crsf =(crsf_frame *)rxbuf;

	if (crsf->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER) {	// ヘッダチェック
		if (crsf->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {	// CHデータならデコード
			ch[0] = (uint16_t)crsf->data.b11.ch0;
			ch[1] = (uint16_t)crsf->data.b11.ch1;
			ch[2] = (uint16_t)crsf->data.b11.ch2;
			ch[3] = (uint16_t)crsf->data.b11.ch3;
			ch[4] = (uint16_t)crsf->data.b11.ch4;
			ch[5] = (uint16_t)crsf->data.b11.ch5;
			ch[6] = (uint16_t)crsf->data.b11.ch6;
			ch[7] = (uint16_t)crsf->data.b11.ch7;
			ch[8] = (uint16_t)crsf->data.b11.ch8;
			ch[9] = (uint16_t)crsf->data.b11.ch9;
			ch[10] = (uint16_t)crsf->data.b11.ch10;
			ch[11] = (uint16_t)crsf->data.b11.ch11;
			ch[12] = (uint16_t)crsf->data.b11.ch12;
			ch[13] = (uint16_t)crsf->data.b11.ch13;
			ch[14] = (uint16_t)crsf->data.b11.ch14;
			ch[15] = (uint16_t)crsf->data.b11.ch15;
			datardyf = true; // データ揃ったよフラグ
		}
	}
}	

void crc_init(uint8_t poly)
{
  for (int idx=0; idx<256; ++idx)
	{
		uint8_t crc = idx;
		for (int shift=0; shift<8; ++shift)
		{
			crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
		}
		_lut[idx] = crc & 0xff;
	}
}

uint8_t crc_calc(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	while (len--)
	{
		crc = _lut[crc ^ *data++];
	}
	return crc;
}

// CRSF受信処理
void crsf_read(void) {
	uint8_t data;
	uint8_t len = 0;
	
	// CRSFから1バイト受信
	while(Serial1.available()) {	// Serial1に受信データがあるなら
		data  = Serial1.read();			// 8ビットデータ読込
		gaptime = micros();

		if(rxPos < CRSF_MAX_PACKET_LEN){
			if (rxPos == 1)	len = data;		// 2byte目はフレームサイズ
			rxbuf[rxPos++] = data;				// 受信データをバッファに格納
		}
		if (rxPos >= len + 2) {
			uint8_t CRC = rxbuf[2 + len - 1];						// CRC値
  		uint8_t crc = crc_calc(&rxbuf[2], len - 1);	// CRC計算
			if (crc == CRC)
			{
 				crsfdecode();							// １フレーム受信し終わったらデーコードする
				rxPos = 0;
     }
		}
	}
	if (rxPos > 0 && micros() - gaptime > 300) { // 300us以上データが来なかったら区切りと判定
		rxPos = 0;
	}
}

void elrs_init()
{
	Serial1.begin(CRSF_BAUDRATE, SERIAL_8N1, GROVE_RX, GROVE_TX);	// Rx = G1, Tx = G2
	datardyf = false;
	gaptime = 0;
	rxPos = 0;
	for (uint8_t i = 0; i < 16; i++)
		Stick[i] = 0.0;

	crc_init(0xd5);
}

void elrs_update(){
	crsf_read();	// ELRSフレーム受信
	if(datardyf){							// データが受信できたら
		onReceiveRcChannels();	// チャンネルデータに格納
		datardyf = false;
	}
}