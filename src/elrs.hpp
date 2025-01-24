#ifndef ELRS_HPP
#define ELRS_HPP

// プロポのCHを割り当ててください
#define RC_CH_AILERON		0	// エルロン　チャンネル
#define RC_CH_ELEVATOR	1	// エレベータ　チャンネル
#define RC_CH_THORTTLE	2	// スロットル　チャンネル
#define RC_CH_RUFFER		3	// ラダー　チャンネル
#define RC_CH_ARM				8	// アームSWはモーメンタリ―SW（離すと戻る）を割り当ててください。
#define RC_CH_MODE			5	// コントロールモード　[stable(angle)/sports(acro)]
#define RC_CH_ALTCNT		6	// 高度維持モード [manual/auto]
#define RC_CH_FLIP			7	// フリップSWはモーメンタリSW（離すと戻る）を割り当ててください。
#define RC_CH_LOG				12	// 

extern volatile float Stick[16];
#define RUDDER         0
#define ELEVATOR       1
#define THROTTLE       2
#define AILERON        3
#define LOG            4
#define DPAD_UP        5
#define DPAD_DOWN      6
#define DPAD_LEFT      7
#define DPAD_RIGHT     8
#define BUTTON_ARM     9
#define BUTTON_FLIP    10
#define CONTROLMODE    11
#define ALTCONTROLMODE 12

#define RUDDER_MIN   	172
#define RUDDER_MID		991
#define RUDDER_MAX  	1810

#define ELEVATOR_MIN  172
#define ELEVATOR_MID 	991
#define ELEVATOR_MAX 	1810

#define THROTTLE_MIN  172
#define THROTTLE_MID 	180	//992
#define THROTTLE_MAX 	1810

#define AILERON_MIN  	172
#define AILERON_MID 	991
#define AILERON_MAX 	1810

#define CRSF_BAUDRATE  420000
#define CRSF_MAX_PACKET_LEN 64
#define CRSF_NUM_CHANNELS 16

typedef enum
{
	CRSF_ADDRESS_BROADCAST          = 0x00,
	CRSF_ADDRESS_USB                = 0x10,
	CRSF_ADDRESS_TBS_CORE_PNP_PRO   = 0x80,
	CRSF_ADDRESS_RESERVED1          = 0x8A,
	CRSF_ADDRESS_CURRENT_SENSOR     = 0xC0,
	CRSF_ADDRESS_GPS                = 0xC2,
	CRSF_ADDRESS_TBS_BLACKBOX       = 0xC4,
	CRSF_ADDRESS_FLIGHT_CONTROLLER  = 0xC8,   // 受信データはこれで来る
	CRSF_ADDRESS_RESERVED2          = 0xCA,
	CRSF_ADDRESS_RACE_TAG           = 0xCC,
	CRSF_ADDRESS_RADIO_TRANSMITTER  = 0xEA,
	CRSF_ADDRESS_CRSF_RECEIVER      = 0xEC,
	CRSF_ADDRESS_CRSF_TRANSMITTER   = 0xEE,
} crsf_addr_e;

typedef enum
{
	CRSF_FRAMETYPE_GPS                = 0x02,
	CRSF_FRAMETYPE_BATTERY_SENSOR     = 0x08,
	CRSF_FRAMETYPE_LINK_STATISTICS    = 0x14,
	CRSF_FRAMETYPE_OPENTX_SYNC        = 0x10,
	CRSF_FRAMETYPE_RADIO_ID           = 0x3A,
	CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,   // チャンネルパックフレーム
	CRSF_FRAMETYPE_ATTITUDE           = 0x1E,
	CRSF_FRAMETYPE_FLIGHT_MODE        = 0x21,
	// Extended Header Frames, range: 0x28 to 0x96
	CRSF_FRAMETYPE_DEVICE_PING        = 0x28,
	CRSF_FRAMETYPE_DEVICE_INFO        = 0x29,
	CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
	CRSF_FRAMETYPE_PARAMETER_READ     = 0x2C,
	CRSF_FRAMETYPE_PARAMETER_WRITE    = 0x2D,
	CRSF_FRAMETYPE_COMMAND            = 0x32,
	// MSP commands
	CRSF_FRAMETYPE_MSP_REQ            = 0x7A,   // response request using msp sequence as command
	CRSF_FRAMETYPE_MSP_RESP           = 0x7B,   // reply with 58 byte chunked binary
	CRSF_FRAMETYPE_MSP_WRITE          = 0x7C,   // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsf_frame_type_e;

#pragma pack(push,1)	// データを詰めて配置
//----------- PACK ここから ------------------------
typedef struct{
	unsigned ch0 : 11;	// 11bit チャンネルデータ
	unsigned ch1 : 11;
	unsigned ch2 : 11;
	unsigned ch3 : 11;
	unsigned ch4 : 11;
	unsigned ch5 : 11;
	unsigned ch6 : 11;
	unsigned ch7 : 11;
	unsigned ch8 : 11;
	unsigned ch9 : 11;
	unsigned ch10 : 11;
	unsigned ch11 : 11;
	unsigned ch12 : 11;
	unsigned ch13 : 11;
	unsigned ch14 : 11;
	unsigned ch15 : 11;
} crsf_channels;

typedef union{				// チャンネルデータ共用体
	uint8_t byte[22];
	crsf_channels b11;
} crsf_data;

typedef struct{				// CRSFフレーム
	uint8_t device_addr;	// アドレス
	uint8_t frame_size;		// この後からのバイト数
	uint8_t type;					// タイプ
	crsf_data data;				// チャンネルデータ
	uint8_t crc;					// CRC
} crsf_frame;

#pragma pack(pop)
//--------- PACK ここまで------------------
extern volatile float Stick[16];
extern bool datardyf;

void elrs_init();
void elrs_update();

#endif