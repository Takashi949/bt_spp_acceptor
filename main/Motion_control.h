#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include <string.h>

#define TAG "MOTION"

class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
	dspm::Mat trans;
	float x_IMU[3] = {-1.449E-3, 59.288E-3, -8.292E-3};
	float g_prev_src[3] = {0.0f, 0.0f, 0.0f};
	dspm::Mat g_prev = dspm::Mat(g_prev_src, 3,1);

	float IMU2body_src[9] = {
		1, 0, 0,
		0, 0, -1,
		0, -1, 0
	};
	dspm::Mat IMU_2_body = dspm::Mat(IMU2body_src, 3, 3);
public:
	Motion_control(){
		//x = [ax ay az vx vy vz]';
		float Fsrc[] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			dt, 0, 0, 1, 0, 0,
			0, dt, 0, 0, 1, 0,
			0, 0, dt, 0, 0, 1
		};
		F = dspm::Mat(Fsrc, 6, 6);

		float Bsrc[] = {
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,
			0, 0, 0, 0, 0,

			0, -225.0f*3.1415f/100.0f/mass, -225.0f*3.1415f/100.0f/mass, 225.0f*3.1415f/100.0f/mass, 225.0f*3.1415f/100.0f/mass,
			0, 225.0f*3.1415f/100.0f/mass, -225.0f*3.1415f/100.0f/mass, -225.0f*3.1415f/100.0f/mass, 225.0f*3.1415f/100.0f/mass,
			1.69f/100.0f/mass, 0, 0, 0, 0,
		};
		B = dspm::Mat(Bsrc, 6, 5);

		float Hsrc[] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
		};
		H = dspm::Mat(Hsrc, 3, 6);

		float Qsrc[6*6] = {
			0.0003, 0, 0, 0, 0, 0,
			0, 0.0003, 0, 0, 0, 0,
			0, 0, 0.0003, 0, 0, 0,
			0, 0, 0, 0.0003, 0, 0,
			0, 0, 0, 0, 0.0003, 0,
			0, 0, 0, 0, 0, 0.0003,
		};
		Q = dspm::Mat(Qsrc, 6, 6);

		float Rsrc[] = {
			0.3, 0, 0,
			0, 0.3, 0, 
			0, 0, 0.3
		};
		R = dspm::Mat(Rsrc, 3, 3);

		for (int i = 0; i < 5; i++)
		{
			u(i, 0) = 0.0;	
		}

		float KCsrc[] = {
			0,0,0,0,0,0,
			0.4974143,0.1474095,3.2075448,-12.441745,-4.3787351,-83.406412,
			0.4693394,1.3929009,3.2149643,-11.736574,-35.631342,-83.665772,
			-0.5689974,1.4207151,3.2203747,14.301187,-36.331286,-83.810488,
			-0.5409225,0.1752237,3.2129552,13.596016,-5.0786796,-83.551128
		};
			
		KC = dspm::Mat(KCsrc, 5, 6);
	}

	const float gravity_c = 9.80665;
	const float deg2rad = 0.0174533;
	const float rad2deg = 1.0/deg2rad;
	const float mass = 1.3022;
	float a0[3] = {0};
	float g0[3] = {0};
	float m0[3] = {0};
	dspm::Mat a_grav = dspm::Mat(3, 1);
	dspm::Mat a = dspm::Mat(3, 1);
	dspm::Mat g = dspm::Mat(3, 1);
	dspm::Mat m = dspm::Mat(3, 1);
	dspm::Mat v = dspm::Mat(3, 1);
	dspm::Mat x = dspm::Mat(3, 1);
	dspm::Mat u = dspm::Mat(5, 1);
	float PRY_value[3] = {0};
	dspm::Mat F, B, H, Q, R, KC;
	float *KCsrc = KC.data;
	dspm::Mat P = dspm::Mat::eye(6);
	dspm::Mat xhat = dspm::Mat(6, 1);
	float gdot[3] = {0};
	float thetadot[3] = {0};
	float dt = 0.05f;

	void begin(float sampleFreq, i2c_master_bus_handle_t bus_handle);
    void Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void filtaUpdate();
    void update();
    void calcU();
    void getPRY(float *retbuf);
    void calib();
    void correctInitValue(uint16_t num_loop);
};