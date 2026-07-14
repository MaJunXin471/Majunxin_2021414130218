// 故障安全保护
// Fail-safe functions

bool isInverted = false;  // 当前机身是否处于倒置（Z轴cos < INVERTED_COS_THRESHOLD）

float rcLossTimeout = 1;        // RC丢失超时时间（秒），可通过参数 SF_RC_LOSS_TIME 配置
float descendTime = 10;         // 下降至停机的时间（秒），可通过参数 SF_DESCEND_TIME 配置
#define WEB_RC_LOSS_TIMEOUT_MS 8000UL  // Web遥控器失联阈值(ms)，必须大于心跳间隔2000ms

// 倒置保护参数
#define INVERTED_COS_THRESHOLD -0.7f   // cos(134°)，倾角超过134°视为倒置（留出陀螺漂移裕量）
#define INVERTED_TIMEOUT        1.5f   // 持续倒置超过1.5秒触发停机

extern float controlTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;

#if WEB_RC_ENABLED
// Web RC变量声明
extern bool webRCEnabled;
extern bool useWebRC;
extern unsigned long webRCLastUpdate;
bool isUsingWebRC();
#endif

extern bool armed;
extern int mode;
extern float dt;
extern float thrustTarget;
extern float t;
extern float batteryVoltage;  // battery.ino
extern Quaternion attitudeTarget;
extern Quaternion attitude;

void failsafe() {
	rcLossFailsafe();
#if WEB_RC_ENABLED
	webRCLossFailsafe();
#endif
	autoFailsafe();
	invertedFailsafe();
	batteryFailsafe();
}

// RC loss failsafe
void rcLossFailsafe() {
	if (controlTime == 0) return; // no RC at all
	if (!armed) return;
#if WEB_RC_ENABLED
	if (isUsingWebRC()) return; // WebRC独立负责其超时（webRCLossFailsafe）
#endif
	if (t - controlTime > rcLossTimeout) {
		descend();
	}
}

// Smooth descend on RC lost
void descend() {
	mode = AUTO;
	attitudeTarget = Quaternion();
	thrustTarget -= dt / descendTime;
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	
	// control*已统一涳盖SBUS/MAVLink/WebRC输入，直接检查即可
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || abs(throttle - controlThrottle) > 0.05) {
		if (mode == AUTO) mode = STAB;
	}
	
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}

// Inverted failsafe
void invertedFailsafe() {
	if (!armed) {
		isInverted = false;
		return;
	}
	
	Vector down = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	bool currentlyInverted = down.z < INVERTED_COS_THRESHOLD;
	
	static Delay invertedDelay(INVERTED_TIMEOUT);
	isInverted = invertedDelay.update(currentlyInverted);
	
	if (isInverted) {
		print("INVERTED! Disarming\n");
		armed = false;
	}
}

// Battery failsafe
void batteryFailsafe() {
	if (!armed) return;
	
	static Delay batteryDelay(BATTERY_ACTION_DEBOUNCE_TIME);
	bool lowBattery = batteryVoltage < VBAT_LOW_THRESHOLD;
	bool criticalBattery = batteryVoltage < VBAT_CRITICAL_THRESHOLD;
	
	if (criticalBattery && batteryDelay.update(criticalBattery)) {
		print("CRITICAL BATTERY! Landing\n");
		mode = AUTO;
		attitudeTarget = Quaternion();
		thrustTarget = ALTHOLD_HOVER_THRUST;
		descend();
	} else if (lowBattery && batteryDelay.update(lowBattery)) {
		print("LOW BATTERY! Returning home\n");
	}
}

#if WEB_RC_ENABLED
void webRCLossFailsafe() {
	if (!webRCEnabled || !useWebRC) return;
	if (millis() - webRCLastUpdate > WEB_RC_LOSS_TIMEOUT_MS) {
		print("Web RC lost!\n");
		useWebRC = false;
		if (armed) descend();
	}
}
#endif
