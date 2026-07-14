// 参数存储在闪存
// Parameters storage in flash memory

#include <Preferences.h>
#include "util.h"

extern float channelZero[16];
extern float channelMax[16];
extern float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;
extern int wifiMode, udpLocalPort, udpRemotePort;
extern int motorPins[4];
extern int pwmFrequency, pwmResolution, pwmStop, pwmMin, pwmMax;
extern float motThrMin;
extern float motThrMax;
extern int mavlinkSysId;
extern Rate telemetrySlow, telemetryFast;
extern float rcLossTimeout, descendTime;
extern int flightModes[3];
extern Vector accBias, accScale;
extern Vector imuRotation;
extern LowPassFilter<Vector> gyroBiasFilter;
extern int rcRxPin;

Preferences storage;

struct Parameter {
	const char *name; // max length is 15 (Preferences key limit)
	bool integer;
	union { float *f; int *i; };
	float cache; // what's stored in flash
	void (*callback)(); // called after parameter change
	Parameter(const char *name, float *variable, void (*callback)() = nullptr) : name(name), integer(false), f(variable), cache(0), callback(callback) {};
	Parameter(const char *name, int *variable, void (*callback)() = nullptr) : name(name), integer(true), i(variable), cache(0), callback(callback) {};
	float getValue() const { return integer ? *i : *f; }
	void setValue(const float value) { if (integer) *i = (int)value; else *f = value; }
};

Parameter parameters[] = {
	// control
	{"CTL_R_RATE_P", &rollRatePID.p},
	{"CTL_R_RATE_I", &rollRatePID.i},
	{"CTL_R_RATE_D", &rollRatePID.d},
	{"CTL_R_RATE_WU", &rollRatePID.windup},
	{"CTL_P_RATE_P", &pitchRatePID.p},
	{"CTL_P_RATE_I", &pitchRatePID.i},
	{"CTL_P_RATE_D", &pitchRatePID.d},
	{"CTL_P_RATE_WU", &pitchRatePID.windup},
	{"CTL_Y_RATE_P", &yawRatePID.p},
	{"CTL_Y_RATE_I", &yawRatePID.i},
	{"CTL_Y_RATE_D", &yawRatePID.d},
	{"CTL_R_P", &rollPID.p},
	{"CTL_R_I", &rollPID.i},
	{"CTL_R_D", &rollPID.d},
	{"CTL_P_P", &pitchPID.p},
	{"CTL_P_I", &pitchPID.i},
	{"CTL_P_D", &pitchPID.d},
	{"CTL_Y_P", &yawPID.p},
	{"CTL_YAW_R", &yawRate},
	{"CTL_TILT", &tiltMax},
	{"CTL_R_RATE_M", &rollRateMax},
	{"CTL_P_RATE_M", &pitchRateMax},
	{"CTL_Y_RATE_M", &yawRateMax},
	{"CTL_FLT_MODE_0", &flightModes[0], nullptr},
	{"CTL_FLT_MODE_1", &flightModes[1], nullptr},
	{"CTL_FLT_MODE_2", &flightModes[2], nullptr},
	{"MOT_THR_MIN", &motThrMin},
	{"MOT_THR_MAX", &motThrMax},
	// motors
	{"MOT_PIN_RL", &motorPins[0], configureMotors},
	{"MOT_PIN_RR", &motorPins[1], configureMotors},
	{"MOT_PIN_FR", &motorPins[2], configureMotors},
	{"MOT_PIN_FL", &motorPins[3], configureMotors},
	{"MOT_PWM_FREQ", &pwmFrequency, configureMotors},
	{"MOT_PWM_RES", &pwmResolution, configureMotors},
	{"MOT_PWM_STOP", &pwmStop, configureMotors},
	{"MOT_PWM_MIN", &pwmMin, configureMotors},
	{"MOT_PWM_MAX", &pwmMax, configureMotors},
	// RC
	{"RC_CH_ROLL", &rollChannel},
	{"RC_CH_PITCH", &pitchChannel},
	{"RC_CH_THR", &throttleChannel},
	{"RC_CH_YAW", &yawChannel},
	{"RC_CH_MODE", &modeChannel},
	{"RC_RX_PIN", &rcRxPin, setupRC},
	{"RC_LOSS_TIME", &rcLossTimeout},
	{"RC_DESC_TIME", &descendTime},
	// IMU
	{"IMU_ROT_X", &imuRotation.x},
	{"IMU_ROT_Y", &imuRotation.y},
	{"IMU_ROT_Z", &imuRotation.z},
	{"IMU_ACC_W", &accWeight},
	{"IMU_GYRO_A", &gyroBiasFilter.alpha},
	// WiFi
	{"WIFI_MODE", &wifiMode, setupWiFi},
	{"WIFI_UDP_LPORT", &udpLocalPort},
	{"WIFI_UDP_RPORT", &udpRemotePort},
	// MAVLink
	{"MAV_SYS_ID", &mavlinkSysId},
	{"MAV_TEL_SLOW", &telemetrySlow.rate},
	{"MAV_TEL_FAST", &telemetryFast.rate},
};

const int parametersCount = sizeof(parameters) / sizeof(parameters[0]);
bool parametersChanged = false;

void setupParameters() {
	print("Setup Parameters\n");
	storage.begin("parameters");
	readParameters();
	print("Parameters loaded\n");
}

void readParameters() {
	for (int i = 0; i < parametersCount; i++) {
		Parameter &p = parameters[i];
		p.cache = p.getValue();
		float value = storage.getFloat(p.name, p.cache);
		if (value != p.cache) {
			p.setValue(value);
			if (p.callback) p.callback();
		}
	}
}

void writeParameters() {
	for (int i = 0; i < parametersCount; i++) {
		Parameter &p = parameters[i];
		float value = p.getValue();
		if (value != p.cache) {
			storage.putFloat(p.name, value);
			p.cache = value;
			if (p.callback) p.callback();
		}
	}
}

void syncParameters() {
	if (parametersChanged) {
		writeParameters();
		parametersChanged = false;
	}
}

void resetParameters() {
	storage.clear();
	parametersChanged = true;
	print("Parameters reset\n");
}

void printParameters() {
	for (int i = 0; i < parametersCount; i++) {
		print("%s = %g\n", parameters[i].name, parameters[i].getValue());
	}
}

void printParameter(const char *name) {
	for (int i = 0; i < parametersCount; i++) {
		if (strcmp(parameters[i].name, name) == 0) {
			print("%s = %g\n", parameters[i].name, parameters[i].getValue());
			return;
		}
	}
	print("Parameter %s not found\n", name);
}

void setParameter(const char *name, float value) {
	for (int i = 0; i < parametersCount; i++) {
		if (strcmp(parameters[i].name, name) == 0) {
			parameters[i].setValue(value);
			parametersChanged = true;
			print("%s = %g\n", parameters[i].name, parameters[i].getValue());
			return;
		}
	}
	print("Parameter %s not found\n", name);
}
