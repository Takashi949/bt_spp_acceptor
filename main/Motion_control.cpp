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
void Motion_control::Sensor2Inert(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz){
	for(uint8_t i = 0; i < 3; i++){
		g[i] = transM[i][0]*gx + transM[i][1]*gy + transM[i][2]*gz;
		a[i] = transM[i][0]*ax + transM[i][1]*ay + transM[i][2]*az;
		m[i] = transM[i][0]*mx + transM[i][1]*my + transM[i][2]*mz; 
	}
}
void Motion_control::update(){
	imu.readTemp();
    imu.readAccel();
    imu.readGyro();
    imu.readMag();
	
	//姿勢を変換数する
	Sensor2Inert(imu.calcGyro(imu.gx - g0[0]), imu.calcGyro(imu.gy - g0[1]), imu.calcGyro(imu.gz - g0[2]),
				imu.calcAccel(imu.ax - a0[0]),  imu.calcAccel(imu.ay - a0[1]),  imu.calcAccel(imu.az - a0[2]),
				imu.calcMag(imu.mx - m0[0]), imu.calcMag(imu.my - m0[1]), imu.calcMag(imu.mz - m0[2]));
    madgwick.update(g[0], g[1], g[2], a[0], a[1], a[2], m[0], m[1], g[2]);
	//z方向速度だけやってみる
	v[0] = v[0] + a[0]*dt;
	v[1] = v[1] + a[1]*dt;
	v[2] = v[2] + a[2]*dt;
}
void Motion_control::calcU(){
	//u = -(a[2]) -100.0*mass/1.69/dt *(121.0 - 11.0*1.69/100.0/mass) *v[2];
	//u = (a[2] + 9.80)*100.0f/1.69 * mass;
	u[0] = a[2] * -105.3f;
	u[1] = 71.1 * g[2];
	u[2] = -71.1 * g[2];
	for (uint8_t i = 0; i < 3; i++)
	{	
		if(u[i] < 0.0f)u[i] = 0.0f;
		if(u[i] > 100.0f)u[i] = 100.0f;
	}
}
void Motion_control::getPRY(float* retbuf){
	retbuf[0] = madgwick.getPitch();
	retbuf[1] = madgwick.getRoll();
	retbuf[2] = madgwick.getYaw();
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
		for (uint8_t i = 0; i < 3; i++)
		{
        	g0[i] = (g0[i] + g[i])/2;
			a0[i] = (a0[i] + a[i])/2;
			m0[i] = (m0[i] + m[i])/2;
		}
    }
    ESP_LOGI(TAG, "calibrate FINISH");
}