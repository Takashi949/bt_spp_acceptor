#include "Motion_control.h"
#include <inttypes.h>
#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_log.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "mat.h"

void Motion_control::begin(i2c_master_bus_handle_t bus_handle){
 	if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }
	madgwick.begin(1.0/dt);
}
void Motion_control::Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz){
	float gsrc[] = {gx, gy, gz};
	dspm::Mat gm(gsrc, 3, 1);
	g = trans * gm;

	//姿勢判定用 重力込みの加速度
	float asrc[] = {ax, ay, az};
	dspm::Mat am(asrc, 3, 1);
	a = trans * am;
	a_grav = a;

	float msrc[] = {mx, my, mz};
	dspm::Mat mm(msrc, 3, 1);
	m = trans * mm;
}
void Motion_control::filtaUpdate(){
	//x = F*x + u + Q
	//y = H*x + R
	
	//x = [ax ay az vx vy vz]';
	
	//y= [ax ay az];
	dspm::Mat y = a;

	//x = Fx + Bu;
	xhat = F * xhat;

	//P = F P F' + Q
	P = F*P*F.t() + Q;
	
	//K = P*H'*(R + H*P*H')^-1
	K = P*H.t()*(R + H*P*H.t()).inverse();

	//xhat = xhat + K(y-Hx)
	xhat += K*(y-H*xhat);

	//P = (I-KH)P
	P = (dspm::Mat::eye(6) - K*H)*P;
}
void Motion_control::update(){
	imu.readTemp();
    imu.readAccel();
    imu.readGyro();
    imu.readMag();
	
	//姿勢を変換数する+座標変換
	Sensor2Body(imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
				imu.calcAccel(imu.ax) * gravity_c, imu.calcAccel(imu.ay) * gravity_c,  imu.calcAccel(imu.az) * gravity_c,
				imu.calcMag(imu.mx - m0[0]), imu.calcMag(imu.my - m0[1]), imu.calcMag(imu.mz - m0[2]));
    
	//aに重力加速度込みの値を入れる
	madgwick.update(g(0, 0), g(1, 0), g(2, 0),
					a_grav(0, 0), a_grav(1, 0), a_grav(2, 0),
					m(0, 0), m(1, 0), m(2, 0));

	//姿勢から重力の分力を減算
	float gv[] = {0.0, 0.0, gravity_c};
	static float gv_bsrc[3] = {0.0};
	madgwick.trans(gv_bsrc, gv);
	dspm::Mat gv_b(gv_bsrc, 3, 1);

	//LPF用の前回値
	//dspm::Mat a_old = a;

	//重力加速度を引く
	a = a_grav - gv_b;
	//a = a_grav;
	
	//閾値より小さかったら0 大きかったらローパスフィルタを通す
	//abs代わりに二乗乗

	//ローパスフィルタ用係数
	const float alpha = 0.1;
	//a = alpha * a_old  + (1.0 - alpha) * a;

	filtaUpdate();

	//積分
}
void Motion_control::calcU(){
	//u = -(a[2]) -100.0*mass/1.69/dt *(121.0 - 11.0*1.69/100.0/mass) *v[2];
	//u = (a[2] + 9.80)*100.0f/1.69 * mass;
	const float F[3][4] = {
		{-3.31058510375725e-18,	1.40929870205265e-17,	-0.999999999999998,	-3.36923865435662},
		{0.707106781186547,	1.07730505707788, 4.51655336604030e-17, -7.80806010163669e-17},
		{-0.707106781186547, -1.07730505707788, -4.51655336604030e-17, 7.80806010163669e-17}};
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