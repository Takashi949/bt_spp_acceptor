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
	float u[3] = {0.0f};
	float dt = 10000.0f;
	/* 	{
		{cos30, -sin30, 0},
		{-sin30, -cos30, 0},
		{0, 0, 1}
	} */

	//質量 kg
	const float mass = 857.23E-3f;
	//体積 mm3
	const float V = 4600.0E-9;
	//IMUセンサから重心までの距離 m
	const float CG[3] = {-73.538E-3, 45.016E-3, -19.658e-3};
	//操舵面と重心の距離 m
	//y軸の正負で右左を変える
	const float ECG[3] = {6.565E-3, -33.663E-3, -63.053E-3};
	/*
	長さ	217.098 mm
	幅	221.586 mm
	高さ	174.50 mm
	*/
	//	重心の慣性モーメント   (g mm^2)
	const float J[3][3] = {
		{ 1.546E+06, -2.701E+05,30139.423},
		{ -2.701E+05,2.660E+06,-7183.935},
		{30139.423,-7183.935 ,3.000E+06}
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