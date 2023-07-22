#ifndef flyHeight_H
#define flyHeight_H

#include <Wire.h>
#include "Arduino.h"
#include "BTFL_430.h"
#include "VL53L0X.h"

namespace fHC{	//常量
	typedef uint8_t errCodeType;
	typedef double heightType;
}

namespace fHErr{	//返回错误值
	using fHC::errCodeType;
	const errCodeType fH_OK  = 0x01;
	const errCodeType fH_InitFAIL = 0x02;
	const errCodeType fH_TimeOut = 0x03;
}

namespace fHVariable{	//变量
	using fHC::heightType;
	
	TaskHandle_t xHandle, fHcon_xHandle;
	VL53L0X sensor;
	
	// PID 控制参数
	double kp = 2;  // 比例系数
	double ki = 0.005;  // 积分系数
	double kd = 5;  // 微分系数

	// 目标高度和当前高度
	heightType taskHeight = 50.0;
	heightType currentHeight = 0.0;

	// PID 输入输出变量
	double error = 0.0;
	double previousError = 0.0;  // 前一次误差
	double integral = 0.0;
	double derivative = 0.0;
	double output = 0.0;

	// 限幅范围
	double integralLimit = 140000.0;  // 积分项限幅
	double outputLimit = 1000.0;    // 输出限幅

	// 时间间隔
	unsigned long previousTime = 0;
	unsigned long currentTime = 0;
	unsigned long deltaTime = 0;
}

namespace fHStruct{	//结构体
	using fHC::errCodeType;
	using namespace fHErr;
	struct fHReturnVal{
		errCodeType errCode = fH_OK;
	};
}

namespace fHNamespace{
	using namespace fHStruct;
	using namespace fHVariable;
	using namespace fHErr;
	using fHC::heightType;
	using namespace BTFL_ENV;
	
	void getHeightLoop(void *);
	void pidLoopControl(void *);
	
	
	class fHModel{
		public:
			//初始化tof数据读取
			fHReturnVal fHInit(uint8_t SDA, uint8_t SCL){
				fHReturnVal RTN;
				Wire.begin(SDA, SCL);
				sensor.setTimeout(500);
				//sensor.setMeasurementTimingBudget(20000);//将时间预算减少到20毫秒(默认是33毫秒)
				if (!sensor.init()) {
					RTN.errCode = fH_InitFAIL;
				}
				sensor.startContinuous(1);
				return RTN;
			}
			//获取当前tof读取的高度数据
			heightType getHeight(){
				return currentHeight;
			}
			//开始不断读取tof数据并保存在变量currentHeight中
			fHReturnVal fHStartGetHeight(){
				fHReturnVal RTN;
				xTaskCreatePinnedToCore(getHeightLoop, "getHeightLoop", 4096, NULL, 1, &xHandle, 1);
				return RTN;
			}
			// 开始使用位置式pid控制无人机飞行高度保持为taskHeightemp
			fHReturnVal fHpidLoopControl(double taskHeightemp){
				fHReturnVal RTN;
				taskHeight = taskHeightemp;
				xTaskCreatePinnedToCore(pidLoopControl, "pidLoopControl", 4096, NULL, 1, &fHcon_xHandle, 1);
				return RTN;
			}
			// 停止pid控制
			fHReturnVal fHpidLoopStop(){
				fHReturnVal RTN;
				vTaskDelete(fHcon_xHandle);
				return RTN;
			}
			// 设置对应参数值，入参值为零则忽视
			fHReturnVal setpidParemeter(double kpc, double kic, double kdc, double integralLimitc, double outputLimitc){
				fHReturnVal RTN;
				if(kpc != 0){
					kp = kpc;
				}
				if(kic != 0){
					ki = kic;
				}
				if(kdc != 0){
					kd = kdc;
				}
				if(integralLimitc != 0){
					integralLimit = integralLimitc;
				}
				if(outputLimitc != 0){
					outputLimit = outputLimitc;
				}
				return RTN;
			}
		private:
			
			
	};
	// 连续读取三次tof数据求取平均值（减小误差）后再赋值给taskHeight
	void getHeightLoop(void *arg){
		while(1){
			heightType count = 0;
			for (int i = 1; i <= 3; i++) {
				heightType temp;
				temp = sensor.readRangeContinuousMillimeters();
				count += temp;
				if (sensor.timeoutOccurred()) {
					Serial.print("TIMEOUT");
					while (1){};
				}
				//vTaskDelay(5);
				/*Serial.print(temp);
				Serial.println();*/
			}
			//Serial.println();
			/*Serial.print(count);
			Serial.print(" ");*/
			currentHeight = count / 3;
			/*Serial.print(currentHeight);
			Serial.println();*/
			vTaskDelay(1);
		}
	}
	//pid核心控制代码
	void pidLoopControl(void *arg){
		while(1){
			currentTime = micros();
			deltaTime = currentTime - previousTime;

			// 计算误差
			error = taskHeight - currentHeight;

			// 积分项
			integral += error * deltaTime / 1000;
			// 积分项限幅
			if (integral > integralLimit) {
				integral = integralLimit;
			} else if (integral < -integralLimit) {
				integral = -integralLimit;
			}

			// 微分项
			derivative = (error - previousError) / deltaTime * 100000;
			Serial.print("serror - previousError:");
			Serial.print(error - previousError);
			Serial.print(" deltaTime:");
			Serial.println(deltaTime);
			
			double I = ki * integral;
			// 积分限幅
			/*if (I > integralLimit) {
				I = integralLimit;
			} else if (I < -integralLimit) {
				I = -integralLimit;
			}*/
			//error变号清零integral
			/*if((error > 0 && previousError < 0) || (error < 0 && previousError > 0)){
				integral = 0;
			}*/

			// PID 控制
			output = kp * error + I + kd * derivative;

			// 输出限幅
			if (output > outputLimit) {
				output = outputLimit;
			} else if (output < 0) {
				output = 0;
			}

			// 输出控制信号（例如调整执行器）
			btfl.flyDirection(Up_and_Down, output);//0~1000

			// 打印调试信息
			Serial.print(currentHeight);
			Serial.println();
			Serial.print(kp * error);
			Serial.print(" integral:");
			Serial.print(integral);
			Serial.print(" ");
			Serial.print(I);
			Serial.print(" ");
			Serial.print(kd * derivative);
			Serial.print(" ");

			Serial.println(output);

			previousTime = currentTime;
			previousError = error;
			
			vTaskDelay(100);
		}
	}
	fHModel fH;
}

namespace fHEnv{
	using fHC::heightType;
	using namespace fHErr;
	using fHStruct::fHReturnVal;
	using fHNamespace::fH;
}

#endif