// 陀螺仪加速度计相关设置
// Work with the IMU sensor

#include <SPI.h>
#include <FlixPeriph.h>
#include "vector.h"
#include "lpf.h"
#include "util.h"
#include "board_config.h"

MPU9250 imu(SPI, BOARD_SPI_CS);

// IMU 安装方向（欧拉角，单位 rad）。默认值 (0, 0, -PI/2) 对应本 PCB 的安装方式：
// 芯片正面朝上，X 丝印→飞行器右侧，Y 丝印→飞行器前方 → 转换公式 Vector(data.y, -data.x, data.z)
// 如需适配其他安装方向，修改此值并执行 `preset` 重置参数存储。
Vector imuRotation(0, 0, -PI / 2);

Vector accBias;
Vector accScale(1, 1, 1);
Vector gyroBias;
LowPassFilter<Vector> gyroBiasFilter(0.001); // 陀螺仪偏置低通估计滤波器

void setupIMU() {
	print("Setup IMU\n");
	SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI, BOARD_SPI_CS);
	imu.begin();
	configureIMU();
}

void configureIMU() {
	imu.setAccelRange(imu.ACCEL_RANGE_4G);
	imu.setGyroRange(imu.GYRO_RANGE_2000DPS);
	imu.setDLPF(imu.DLPF_MAX);
	imu.setRate(imu.RATE_1KHZ_APPROX);
	imu.setupInterrupt();
}

void readIMU() {
	imu.waitForData();
	imu.getGyro(gyro.x, gyro.y, gyro.z);
	imu.getAccel(acc.x, acc.y, acc.z);
	calibrateGyroOnce();
	// apply scale and bias
	acc = (acc - accBias) / accScale;
	gyro = gyro - gyroBias;
	// rotate to body frame using imuRotation Euler angles
	Quaternion rotation = Quaternion::fromEuler(imuRotation);
	acc  = Quaternion::rotateVector(acc,  rotation.inversed());
	gyro = Quaternion::rotateVector(gyro, rotation.inversed());
}

void calibrateGyroOnce() {
	static Delay landedDelay(2);
	if (!landedDelay.update(landed)) return; // calibrate only if definitely stationary

	gyroBias = gyroBiasFilter.update(gyro);
}

void calibrateAccel() {
	print("校准陀螺仪加速计 Calibrating accelerometer\n");
	imu.setAccelRange(imu.ACCEL_RANGE_2G); // the most sensitive mode

	print("1/6 水平放置：机头朝前（正常飞行方向），底部朝下水平放置在平坦表面，确保完全水平无倾斜。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("水平校准完成。请继续。\n");
	pause(1);
	print("2/6 机头朝上：保持机头朝前,将机头指向天空，尾部朝下并接触支撑面，与地面垂直。保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("机头朝上校准完成。请继续。\n");
	pause(1);
	print("3/6 机头朝下：保持机头朝前，将机头朝下指向地面，尾部朝上，保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("机头朝下校准完成。请继续。\n");
	pause(1);
	print("4/6 左侧朝下：保持机头朝前，将飞行器左侧朝下接触支撑面，保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("左侧朝下校准完成。请继续。\n");
	pause(1);
	print("5/6 右侧朝下：保持机头朝前，将飞行器右侧朝下接触支撑面，保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("右侧朝下校准完成。请继续。\n");
	pause(1);
	print("6/6 尾部朝下：保持机头朝前，将飞行器尾部朝下接触支撑面，保持不动，8秒后开始校准；\n");
	pause(8);
	calibrateAccelOnce();
	print("尾部朝下校准完成。全部校准结束。\n");

	// restore the original accel range
	configureIMU();
}

void calibrateAccelOnce() {
	static Vector biasSum;
	static Vector scaleSum;
	static int samples = 0;

	readIMU();

	// estimate bias and scale by comparing each axis to the gravity vector
	for (int i = 0; i < 3; i++) {
		if (acc[i] > 0) {
			scaleSum[i] += ONE_G / acc[i];
		} else {
			scaleSum[i] -= ONE_G / acc[i];
		}
		biasSum[i] += acc[i];
	}

	samples++;
	for (int i = 0; i < 3; i++) {
		accScale[i] = scaleSum[i] / samples;
		accBias[i] = biasSum[i] / samples;
	}
}
