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
/* 		const float uFsrc[] = {
			1.01209304572667e-14,	-1.94850676217256e-14,	5.83242925269120e-16,	-2.25922914654743e-16,
			59.1887117583658,	-61.4453363797980,	1.54345536206767,	-1.65628659313927,
			59.1887117583659,	61.4453363797979,	1.54345536206767,	1.65628659313927,
		}; */

		//H∞
		const float uFsrc[] = {
			0,	0,	0,	0,
			-2.54248039003449,	-1.47975871924628,	-1.93775586725231,	-23.8213366456232,
			-2.54248039003449,	1.47975871924628,	-1.93775586725231,	23.8213366456232
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