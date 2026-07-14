// 使用RC接收器
// Work with the RC receiver

#include <SBUS.h>
#include "util.h"
#include "board_config.h"

SBUS rc(BOARD_RC_SERIAL); // Serial2 on ESP32, Serial1 on ESP32-C3
int rcRxPin = BOARD_RC_RX_PIN; // SBUS RX 引脚，-1 表示禁用 RC，可通过参数 RC_RX_PIN 配置
uint16_t channels[16]; // raw rc channels
float controlTime; // time of the last controls update
float channelZero[16]; // calibration zero values
float channelMax[16]; // calibration max values

// Channels mapping (using float to store in parameters):
float rollChannel = NAN, pitchChannel = NAN, throttleChannel = NAN, yawChannel = NAN, modeChannel = NAN;

void setupRC() {
	if (rcRxPin < 0) return; // Pin 未配置则跳过 RC 初始化
	print("Setup RC\n");
	rc.begin(rcRxPin);
}

bool readRC() {
	if (rcRxPin < 0) return false; // RC 未启用
	if (rc.read()) {
		SBUSData data = rc.data();
		for (int i = 0; i < 16; i++) channels[i] = data.ch[i]; // copy channels data
		normalizeRC();
		controlTime = t;
		return true;
	}
	return false;
}

void normalizeRC() {
	float controls[16];
	for (int i = 0; i < 16; i++) {
		controls[i] = mapf(channels[i], channelZero[i], channelMax[i], 0, 1);
	}
	// Update control values
	controlRoll = rollChannel >= 0 ? controls[(int)rollChannel] : NAN;
	controlPitch = pitchChannel >= 0 ? controls[(int)pitchChannel] : NAN;
	controlYaw = yawChannel >= 0 ? controls[(int)yawChannel] : NAN;
	controlThrottle = throttleChannel >= 0 ? controls[(int)throttleChannel] : NAN;
	controlMode = modeChannel >= 0 ? controls[(int)modeChannel] : NAN;
}

void calibrateRC() {
	uint16_t zero[16];
	uint16_t center[16];
	uint16_t max[16];
	print("1/8 校准遥控,所有摇杆归中位置[3秒]\n");
	pause(8);
	calibrateRCChannel(NULL, zero, zero, "2/8 左摇杆:向下,右摇杆:居中[3秒]\n...     ...\n...     .o.\n.o.     ...\n");
	calibrateRCChannel(NULL, center, center, "3/8 左摇杆:居中,右摇杆:居中[3秒]\n...     ...\n.o.     .o.\n...     ...\n");
	calibrateRCChannel(&throttleChannel, max, max, "4/8 左摇杆:向上,右摇杆:居中[3秒]\n.o.     ...\n...     .o.\n...     ...\n");
	calibrateRCChannel(&rollChannel, max, zero, "5/8 左摇杆:居中,右摇杆:向右[3秒]\n...     ...\n.o.     ...\n...     .o.\n");
	calibrateRCChannel(&rollChannel, zero, max, "6/8 左摇杆:居中,右摇杆:向左[3秒]\n...     .o.\n.o.     ...\n...     ...\n");
	calibrateRCChannel(&pitchChannel, max, zero, "7/8 左摇杆:居中,右摇杆:向前[3秒]\n...     ...\n...     .o.\n.o.     ...\n");
	calibrateRCChannel(&pitchChannel, zero, max, "8/8 左摇杆:居中,右摇杆:向后[3秒]\n...     ...\n.o.     ...\n...     ...\n");

	print("校准完成。请将左摇杆回中，右摇杆回中。\n");

	for (int i = 0; i < 16; i++) {
		channelZero[i] = zero[i];
		channelMax[i] = max[i];
	}

	print("RC calibration complete\n");
}

void calibrateRCChannel(float *channel, uint16_t *max, uint16_t *zero, const char *msg) {
	print(msg);
	pause(3);
	uint16_t data[16];
	readRC();
	for (int i = 0; i < 16; i++) data[i] = channels[i];
	for (int i = 0; i < 16; i++) {
		if (zero) zero[i] = data[i];
		if (max) max[i] = data[i];
	}
	if (channel) *channel = 0; // find the channel index
	for (int i = 0; i < 16; i++) {
		if (data[i] != data[0]) {
			if (channel) *channel = i;
			break;
		}
	}
	print("\n");
}
