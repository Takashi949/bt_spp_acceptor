#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_dsp.h"
#include "dsp_platform.h"

#define TAG "MOTION"

class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
	dspm::Mat P, trans, F, B, H, Q, R, K;
public:
	Motion_control(){
		P = dspm::Mat(6, 6);
		xhat = dspm::Mat(6, 1);
		K = dspm::Mat(6, 1);

		static float transM[] = {
		1.7320508 / 2.0, -1.0 / 2.0, 0.0,
		-1.0 / 2.0, -1.7320508 / 2.0, 0.0,
		0.0, 0.0, 1.0
		};
		trans = dspm::Mat(transM, 3, 3);

		static float Fsrc[] = {
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			dt, 0, 0, 1, 0, 0,
			0, dt, 0, 0, 1, 0,
			0, 0, dt, 0, 0, 1
		};
		F = dspm::Mat(Fsrc, 6, 6);
		
		static float Hsrc[] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
		};
		H = dspm::Mat(Hsrc, 3, 6);

		static float Qsrc[6] = {0};
		Q = dspm::Mat(Qsrc, 6, 6);

		static float Rsrc[] = {
			9, 0, 0,
			0, 9, 0,
			0, 0, 9};
		R = dspm::Mat(Rsrc, 3, 3);
	}

	const float gravity_c = 9.80665;
	const float deg2rad = 0.017453;
	const float rad2deg = 1.0/rad2deg;

	float a0[3] = {0};
	float g0[3] = {0};
	float m0[3] = {0};
	dspm::Mat a_grav = dspm::Mat(3, 1);
	dspm::Mat a = dspm::Mat(3, 1);
	dspm::Mat g = dspm::Mat(3, 1);
	dspm::Mat m = dspm::Mat(3, 1);
	dspm::Mat v = dspm::Mat(3, 1);
	dspm::Mat x = dspm::Mat(3, 1);
	dspm::Mat u = dspm::Mat(3, 1);
	dspm::Mat xhat; 

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
    void Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void filtaUpdate();
    void update();
    void calcU();
    void getPRY(float *retbuf);
    void calib();
    void correctInitValue(uint16_t num_loop);
};