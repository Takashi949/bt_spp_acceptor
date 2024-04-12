#include "Motion_control.h"
#include <inttypes.h>
#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"
#include "esp_log.h"

void Motion_control::begin(float sampleFreq, i2c_master_bus_handle_t bus_handle){
 	if(imu.begin(LSM9DS1_AG_ADDR(0), LSM9DS1_M_ADDR(0), bus_handle) == 0){
        ESP_LOGE(TAG, "imu initialize faile");
    }
	dt = 1.0f/sampleFreq;
	madgwick.begin(sampleFreq);
}
void Motion_control::Sensor2Body(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz){
	for(uint8_t i = 0; i < 3; i++){
		g[i] = transM[i][0]*gx + transM[i][1]*gy + transM[i][2]*gz;
		//姿勢判定用 重力込みの加速度
		a_grav[i] = transM[i][0]*ax + transM[i][1]*ay + transM[i][2]*az;
		m[i] = transM[i][0]*mx + transM[i][1]*my + transM[i][2]*mz; 
	}
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
	
	//微分計算のため前の値を保存
	float dtheta[3] = {0};
	dtheta[0] = madgwick.getPitch();
	dtheta[1] = madgwick.getRoll();
	dtheta[2] = madgwick.getYaw();
    
	//azだけ重力加速度込みの値を入れる
	madgwick.update(g[0], g[1], g[2],
					a_grav[0], a_grav[1], a_grav[2],
					m[0], m[1], m[2]);

	//姿勢から重力の分力を減算
	float gv[] = {0.0, 0.0, gravity_c};
	float gv_b[3] = {0.0};
	madgwick.trans(gv_b, gv);
	for (uint8_t i = 0; i < 3; i++)
	{
		//LPF用の前回値
		float a_old = a[i];
		//重力加速度を引く
		a[i] = a_grav[i] - gv_b[i];

		//閾値より小さかったら0 大きかったらローパスフィルタを通す
		//abs代わりに二乗乗
		if(a[i]*a[i] < 0.04){
			a[i] = 0;
		}else {
			//ローパスフィルタ用係数
			const float alpha = 0.1;
			a[i] = alpha * a_old  + (1.0 - alpha) * a[i];
		}
	}

	//積分
	v[0] = v[0] + a[0]*dt;
	v[1] = v[1] + a[1]*dt;
	v[2] = v[2] + a[2]*dt;

	//積分
	x[0] = x[0] + v[0]*dt;
	x[1] = x[1] + v[1]*dt;
	x[2] = x[2] + v[2]*dt;

	//微分そのうちクオータニオンに
	thetadot[0] = (madgwick.getPitch() - dtheta[0])/dt;
	thetadot[1] = (madgwick.getRoll() - dtheta[1])/dt;
	thetadot[2] = (madgwick.getYaw() - dtheta[2])/dt;
}
void Motion_control::calcU(){
	//u = -(a[2]) -100.0*mass/1.69/dt *(121.0 - 11.0*1.69/100.0/mass) *v[2];
	//u = (a[2] + 9.80)*100.0f/1.69 * mass;
	const float F[3][4] = {
		{-3.31058510375725e-18,	1.40929870205265e-17,	-0.999999999999998,	-3.36923865435662},
		{0.707106781186547,	1.07730505707788, 4.51655336604030e-17, -7.80806010163669e-17},
		{-0.707106781186547, -1.07730505707788, -4.51655336604030e-17, 7.80806010163669e-17}};
	for(uint8_t i = 0; i < 3; i++){
		u[i] = F[i][0]*g[2] + F[i][1]*thetadot[2] + F[i][2]*x[2] + F[i][3]*v[2];
		if(u[i] < 0.0f)u[i] = 0.0f;
		if(u[i] > 100.0f)u[i] = 100.0f;
	}
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