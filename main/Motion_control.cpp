#include "Motion_control.h"
#include <inttypes.h>
#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "mat.h"

void Motion_control::begin(float sampleFreq, i2c_master_bus_handle_t bus_handle){
 	if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }
	dt = 1.0f/sampleFreq;
	madgwick.begin(sampleFreq);
}
void Motion_control::Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz){
	float gsrc[] = {gx, gy, gz};
	gdot[0] = (gx - g(0, 0))/dt;
	gdot[1] = (gy - g(1, 0))/dt;
	gdot[2] = (gz - g(2, 0))/dt;

	float wl = gx*x_IMU[0] + gy*x_IMU[1] + gz*x_IMU[2];
	float ww = gx * gx + gy * gy + gz * gz;

	a(0, 0) = ax - gdot[1] * x_IMU[2] + gdot[2]*x_IMU[1] - wl * gx + ww * x_IMU[0];
	a(1, 0) = ay - gdot[2] * x_IMU[0] + gdot[0]*x_IMU[2] - wl * gy + ww * x_IMU[1];
	a(2, 0) = az - gdot[0] * x_IMU[1] + gdot[1]*x_IMU[0] - wl * gz + ww * x_IMU[2];

	g(0, 0) = gx;
	g(1, 0) = gy;
	g(2, 0) = gz;
}
void Motion_control::filtaUpdate(){
	//ESP_LOGD(TAG, "kalman Start");
	//x = F*x + B*u + Q
	//y = H*x
	
	//x = [ax ay az vx vy vz]';
	
	//y= [ax ay az];
	float ysrc[3] = {a(0, 0), a(1, 0), a(2, 0)};
	dspm::Mat y(ysrc, 3, 1);
	
	//x = *** + Bu
	//Lift = -318.2*alpha cos成分は /1.4142 => 225.0*alpha
 	//u = [Thrust S1 S2  S3 S4]

	//x = Fx + Bu;
	//xhat = F * xhat + B * u;
	xhat = F * xhat;

	//P = F P F' + Q
	P = F*P*F.t() + Q;

	//K = P*H'*(R + H*P*H')^-1
	dspm::Mat K(6, 3);
	K = P*H.t()*(R + H*P*H.t()).inverse();
	
	//xhat = xhat + K(y-Hx)
	xhat = xhat + K*(y-H*xhat);

	//P = (I-KH)P
	P = (dspm::Mat::eye(6) - K*H)*P;
	//ESP_LOGI(TAG, "Kalman Updated");
}
void Motion_control::update(){
	imu.readTemp();
    imu.readAccel();
    imu.readGyro();
    imu.readMag();
	
	a_grav(0, 0) = imu.calcAccel(imu.ax) * gravity_c;
	a_grav(1, 0) = imu.calcAccel(imu.ay) * gravity_c;
	a_grav(2, 0) = imu.calcAccel(imu.az) * gravity_c;

	//センサの姿勢を計算
	//azだけ重力加速度込みの値を入れる
	madgwick.update(imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
					a_grav(0, 0), a_grav(1, 0), a_grav(2, 0),
					imu.calcMag(imu.mx - m0[0]), imu.calcMag(imu.my - m0[1]), imu.calcMag(imu.mz - m0[2]));
	
	//センサ座標系から重心の座標系へ
	Sensor2Body(imu.calcGyro(imu.gx) * deg2rad, imu.calcGyro(imu.gy) * deg2rad, imu.calcGyro(imu.gz) * deg2rad,
				a_grav(0, 0), a_grav(1, 0), a_grav(2, 0),
				imu.calcMag(imu.mx - m0[0]), imu.calcMag(imu.my - m0[1]), imu.calcMag(imu.mz - m0[2]));

	//姿勢から重力の分力を減算
	float gv[] = {0.0, 0.0, -gravity_c};
	float gv_bsrc[3] = {0.0};
	madgwick.trans(gv_bsrc, gv);
	dspm::Mat gv_b(gv_bsrc, 3, 1);

	//LPF用の前回値
	dspm::Mat a_old = a;
	//重力加速度を引く
	a = a_grav + gv_b;

	filtaUpdate();

	calcU();
	
	//ESP_LOGI(TAG, "%1.2f,%1.2f,%1.2f", xhat(0, 0), xhat(1, 0), xhat(2, 0));
	//ESP_LOGI(TAG, "raw%1.2f,%1.2f,%1.2f", a(0, 0), a(1, 0), a(2, 0));
	//ESP_LOGI(TAG, "u%2.1f,%2.1f,%2.1f", u(1, 0), u(2, 0), u(3, 0));
}
void Motion_control::calcU(){
	float xsrc[] = {g(0, 0), g(1, 0), g(2, 0), madgwick.getPitchRadians(), madgwick.getRollRadians(), madgwick.getYawRadians()};
	u = KC * dspm::Mat(xsrc, 6, 1);
}
void Motion_control::getPRY(float* retbuf){
	retbuf[0] = madgwick.getPitch();
	retbuf[1] = madgwick.getRoll();
	retbuf[2] = madgwick.getYaw();
}
void Motion_control::calib(){
	imu.calibrate(true);
}
void Motion_control::correctInitValue(uint16_t num_loop){
	ESP_LOGI(TAG, "calibrate START");
	//ジャイロセンサの初期値を取得
    //whileループで平均をとる
 	for (uint8_t i = 0; i < 3; i++)
	{
		g0[i] = 0;
		a0[i] = 0;
		m0[i] = 0;
	}

    for (uint16_t calibStep = 0; calibStep < num_loop; calibStep++)
    {
        imu.readGyro();
		imu.readAccel();
		imu.readMag();

		//g0[0] += imu.gx;
		//g0[1] += imu.gy;
		//g0[2] += imu.gz;

		//a0[0] += imu.ax;
		//a0[1] += imu.ay;
		//a0[2] += imu.az;

		//m0[0] += imu.mx;
		//m0[1] += imu.my;
		//m0[2] += imu.mz;
	}
	for (uint8_t i = 0; i<3; i++){
		g0[i] = g0[i]/num_loop;
		a0[i] = a0[i]/num_loop;
		m0[i] = m0[i]/num_loop;	
	}
	ESP_LOGI(TAG, "calibrate FINISH");
}