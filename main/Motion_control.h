#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"

#define TAG "MOTION"

class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
public:
	float v[3] = {0};
	float a0[3] = {0};
	float g0[3] = {0};
	float m0[3] = {0};
	float a[3] = {0, 0, 0};
	float g[3] = {0, 0, 0};
	float m[3] = {0, 0, 0};
	float u = 0.0f;
	float dt = 10000.0f;
	/* 	{
		{cos30, -sin30, 0},
		{-sin30, -cos30, 0},
		{0, 0, 1}
	} */

	//質量 kg
	const float mass = 68.437E-3f;
	//体積 mm3
	const float V = 55190.917;
	//IMUセンサから重心までの距離 m
	const float CG[3] = {-5.515E-3f, 6.349E-3f, 4.342E-3f};
	/*
	長さ	102.40 mm
	幅	147.357 mm
	高さ	131.973 mm */
	//	重心の慣性モーメント   (g mm^2)
	const float J[3][3] = {
		{ 2.008E+05, 0.00, 1.692},
		{ 0.00, 88804.178, 12.504},
		{ 1.692, 12.504 , 1.774E+05}
	};
	void begin(float sampleFreq, i2c_master_bus_handle_t bus_handle);
	void update();
    void calcU();
    void getPRY(float *retbuf);
    void correctInitValue(uint16_t num_loop);
	float transM[3][3] = {{1.7320508 / 2, -1 / 2, 0}, {-1 / 2, -1.7320508 / 2, 0}, {0, 0, 1}};
	void Sensor2Inert(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
};