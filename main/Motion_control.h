#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_dsp.h"
#include "dsp_platform.h"

#define TAG "MOTION"

class Motion_control{
	LSM9DS1 imu;
	Madgwick madgwick;
	dspm::Mat trans;
	float x_IMU[3] = {0, 0.14524, -0.057685};
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
			0.01, 0, 0, 0, 0, 0,
			0, 0.01, 0, 0, 0, 0,
			0, 0, 0.01, 0, 0, 0,
			0, 0, 0, 0.01, 0, 0,
			0, 0, 0, 0, 0.01, 0,
			0, 0, 0, 0, 0, 0.01,
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
			0.0,0.0,0.0,0.0,0.0,0.0,
			-13657.913607444685,10015.518036892878,-10565.188295836602,-894271.1098821459,-653511.4465871861,-1093151.8197624055,
			-14078.873241869318,-21929.055293844012,-12679.82930969919,-922696.321237952,-60792.0601520551,477810.92293504963,
			13617.923030649537,-22425.14210443497,-13443.084149869166,1063680.195987898,797138.2556994188,369027.140997276931,
			14038.88266507418,9519.431226301915,-11328.443136006583,1092105.4073437045,204418.86926428776,-1201935.6017001783
		};
			
		KC = dspm::Mat(KCsrc, 5, 6);
	}

	const float gravity_c = 9.80665;
	const float deg2rad = 0.0174533;
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
	dspm::Mat u = dspm::Mat(5, 1);
	dspm::Mat F, B, H, Q, R, KC;
	dspm::Mat P = dspm::Mat::eye(6);
	dspm::Mat xhat = dspm::Mat(6, 1);
	float gdot[3] = {0};
	float thetadot[3] = {0};
	float dt = 0.05f;
	/* 	{
		{cos30, -sin30, 0},
		{-sin30, -cos30, 0},
		{0, 0, 1}
	} */

	//質量 kg
	const float mass = 3.064;
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