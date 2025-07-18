#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include <string.h>

#define TAG "MOTION"
struct PID {
	float Kp, Ki, Kd;
	float prev_error;
	float integral;
	float target; // 目標値（セットポイント）
};
class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
	dspm::Mat trans;
	float x_IMU[3] = {-1.449E-3, 59.288E-3, -8.292E-3};
	float g_prev_src[3] = {0.0f, 0.0f, 0.0f};
	dspm::Mat g_prev = dspm::Mat(g_prev_src, 3,1);

	float IMU2body_src[9] = {
		0, 0, -1,
		-1, 0, 0,
		0, -1, 0
	};
	//磁気センサはX逆
	float IMU2body_src_Mag[9] = {
		0, 0, -1,
		1, 0, 0,
		0, -1, 0
	};
	dspm::Mat IMU_2_body = dspm::Mat(IMU2body_src, 3, 3);
	dspm::Mat IMU_2_body_mag = dspm::Mat(IMU2body_src_Mag, 3, 3);
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
			0, 0, 0,
			-1,  1,  0.5,
			-1,  -1,  0.5,
			1, -1,  0.5,
			1, 1,  0.5,
		};
			
		KC = dspm::Mat(KCsrc, 5, 3);

		// PID制御用のインスタンス
		pitch_pid = {10.0f, 0.0f, 8.0f, 0.0f, 0.0f, 0.0f};
		roll_pid = {10.0f, 0.0f, 8.0f, 0.0f, 0.0f, 0.0f};
		yaw_pid = {.0f, .0f, 0.00f, 0.0f, 0.0f, 0.0f};
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
	//単位はrad
	float PRY_value[3] = {0};
	dspm::Mat F, B, H, Q, R, KC;
	float *KCsrc = KC.data;
	dspm::Mat P = dspm::Mat::eye(6);
	dspm::Mat xhat = dspm::Mat(6, 1);
	float gdot[3] = {0};
	float thetadot[3] = {0};
	float dt = 0.05f;
	PID pitch_pid, roll_pid, yaw_pid;

	void begin(float sampleFreq, i2c_master_bus_handle_t bus_handle);
    void Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void filtaUpdate();
    void update();
    void calcU();
	//単位はrad
    void getPRY(float *retbuf);
    void calib();
    void correctInitValue(uint16_t num_loop);
	float calculatePID(PID &pid, float current);
};