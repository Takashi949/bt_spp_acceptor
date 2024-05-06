#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include <string.h>

#define TAG "MOTION"

class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
	dspm::Mat P, trans, F, B, H, Q, R, K, uF;
	float dt;
public:
	Motion_control(float sample_ms){
		dt = sample_ms / 1000.0f;
		xhat = dspm::Mat(6, 1);
		K = dspm::Mat(6, 1);

		P = dspm::Mat(6, 6);
		float Psrc[] = {
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0,
		};
		memcpy(P.data, Psrc, sizeof(Psrc));

		float transM[] = {
			1.7320508 / 2.0, -1.0 / 2.0, 0.0,
			-1.0 / 2.0, -1.7320508 / 2.0, 0.0,
			0.0, 0.0, 1.0
		};
		trans = dspm::Mat(3, 3);
		memcpy(trans.data, transM, sizeof(transM));

		float Fsrc[] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			dt, 0, 0, 1, 0, 0,
			0, dt, 0, 0, 1, 0,
			0, 0, dt, 0, 0, 1
		};
		F = dspm::Mat(6, 6);
		memcpy(F.data, Fsrc, sizeof(Fsrc));
		
		float Hsrc[] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
		};
		H = dspm::Mat(3, 6);
		memcpy(H.data, Hsrc, sizeof(Hsrc));

		float Qsrc[6] = {0};
		Q = dspm::Mat(6, 6);

		float Rsrc[] = {
			0.1, 0, 0,
			0, 0.1, 0,
			0, 0, 0.9};
		R = dspm::Mat(3, 3);
		memcpy(R.data, Rsrc, sizeof(Rsrc));

		//制御
		//LQR
		//Q =4
/* 		const float uFsrc[] = {
			0, 0, 0, 0,
			71.0444136554440,	124.530283308241,	2.13624045692157,	4.81053393956144,
			-71.0444136554435,	124.530283308241,	-2.13624045692155,	4.81053393956145
		}; */

		//Q = 20
/* 		const float uFsrc[] = {
			0, 0, 0, 0,
			141.011561446479,	194.504843615079,	3.88435003252651,	6.55901414096088,
			-141.011561446647,	194.504843615046,	-3.88435003252739,	6.55901414096009
		}; */

		//Q = 10
		const float uFsrc[] = {
			0, 0, 0, 0,
			103.939703921699,	157.429827151682,	2.95812387836106,	5.63263003985364,
			-103.939703921757,	157.429827151668,	-2.95812387836146,	5.63263003985341
		};

		uF = dspm::Mat(3, 4);
		memcpy(uF.data, uFsrc, sizeof(uFsrc));
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

	/* 	{
		{cos30, -sin30, 0},
		{-sin30, -cos30, 0},
		{0, 0, 1}
	} */

	void begin(i2c_master_bus_handle_t bus_handle);
    void Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void filtaUpdate();
    void update();
    void calcU();
    void getPRY(float *retbuf);
    void calib();
    void correctInitValue(uint16_t num_loop);
};