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
	//操舵面と重心の距離 m
	//y軸の正負で右左を変える
	const float ECG[3] = {-5.515E-3, 6.349E-3, 4.342E-3};
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

	//ろーたーの質量 kg
	const float mass_r = 57.756E-3;
	//ローターの重心
	const float CG_r[3] = {-0.001E-3, 0.00E-3, 60.326E-3};
	//ローターの重心の慣性モーメント   (g mm^2)
	const float J_r[3][3] =	{
		{24772.756, 0.197, -0.272},
		{0.197, 24772.398, -0.066},
		{-0.272, -0.066, 42970.999}
	};
	void begin(float sampleFreq, i2c_master_bus_handle_t bus_handle);
	void update();
    void calcU();
    void getPRY(float *retbuf);
    void correctInitValue(uint16_t num_loop);
	float transM[3][3] = {{1.7320508 / 2.0, -1.0 / 2.0, 0}, {-1.0 / 2.0, -1.7320508 / 2.0, 0.0}, {0.0, 0.0, 1.0}};
	void Sensor2Inert(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
};