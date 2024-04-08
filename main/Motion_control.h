#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"

#define TAG "MOTION"

class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
public:
	float a0[3] = {0};
	float g0[3] = {0};
	float m0[3] = {0};
	float a[3] = {0, 0, 0};
	float g[3] = {0, 0, 0};
	float m[3] = {0, 0, 0};
	/* 	{
		{cos30, -sin30, 0},
		{-sin30, -cos30, 0},
		{0, 0, 1}
	} */
	void begin(float sampleFreq, i2c_master_bus_handle_t bus_handle);
	void update();
	void getPRY(float *retbuf);
	void correctInitValue(uint16_t num_loop);
	float transM[3][3] = {{1.7320508 / 2, -1 / 2, 0}, {-1 / 2, -1.7320508 / 2, 0}, {0, 0, 1}};
	void Sensor2Inert(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
};