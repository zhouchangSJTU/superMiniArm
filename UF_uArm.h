/******************************************************************************
* File Name          : UF_uArm.h
* Author             : Evan
* Updated            : Evan
* Version            : V0.3 (BATE)
* Created Date       : 2 May, 2014
* Modified Date      : 2014.10.28
* Description        : 有名称的店铺@taobao.com 根据UFactory的uarm适用库修改。
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include <Arduino.h>
#include <EEPROM.h>
#include "VarSpeedServo.h"

#ifndef UF_uArm_h
#define UF_uArm_h

/****************  Macro definitions  ****************/
#define ARM_A                   80    // upper arm
#define ARM_B                   80    // lower arm
#define ARM_2AB                 12800  // 2*A*B
#define ARM_A2                  6400  // A^2
#define ARM_B2                  6400  // B^2
#define ARM_A2B2                12800  // A^2 + B^2
#define ARM_STRETCH_MIN         35		//伸展，最近端
#define ARM_STRETCH_MAX         158		//伸展，最远端
#define ARM_HEIGHT_MIN          -90	//高度
#define ARM_HEIGHT_MAX          110		//高度
#define ARM_ROTATION_MIN        -90		//底座旋转 左
#define ARM_ROTATION_MAX        90		//底座旋转 右
#define HAND_ROTATION_MIN       -90		//爪旋转（暂无）
#define HAND_ROTATION_MAX       90		//爪旋转（暂无）
#define HAND_ANGLE_OPEN         70		//爪分离 （数值越大，爪打开角度越大）
#define HAND_ANGLE_CLOSE        10		//爪闭合 （闭合后舵机发出声音证明闭合的过紧，时间长会导致舵机发热）
#define FIXED_OFFSET_L          18
#define FIXED_OFFSET_R          36
#define D150A_SERVO_MIN_PUL     535
#define D150A_SERVO_MAX_PUL     2415
#define D009A_SERVO_MIN_PUL     600
#define D009A_SERVO_MAX_PUL     2550
#define SAMPLING_DEADZONE       2
#define INIT_POS_L              37
#define INIT_POS_R              25
#define BTN_TIMEOUT_1000        1000
#define BTN_TIMEOUT_3000        3000
#define CATCH					0x01
#define RELEASE					0x02
#define CALIBRATION_FLAG		0xEE
#define SERVO_MAX				605
#define SERVO_MIN				80
#define MEMORY_SERVO_PER		335   //  eeprom: (1024 - 3 - 14)/3=335
#define DATA_FLAG				255
#define BUFFER_OUTPUT			5
/*****************  Port definitions  *****************/
#define BTN_D4                  4     // 按钮（暂无）
#define BTN_D7                  7     // 按钮（暂无）
#define BUZZER                  3     // 蜂鸣器
#define LIMIT_SW                2     // Limit Switch （限位开关？）
#define PUMP_EN                 6     // 打气泵
#define VALVE_EN                5     // 真空泵
#define SERVO_HAND              9     // 爪
#define SERVO_HAND_ROT          10    // 旋转爪 （暂无）
#define SERVO_ROT               11    // 底座旋转
#define SERVO_R                 12    // 右侧小臂舵机
#define SERVO_L                 13    // 左侧大臂舵机

class UF_uArm
{
public:
	UF_uArm();
	void init();    // initialize the uArm position
//    void calibration();  //校准；刻度；标度
	void recordingMode(unsigned char _sampleDelay = 50); //记录模式
	void setPosition(double _stretch, double _height, int _armRot, int _handRot);    // 
	void setServoSpeed(char _servoNum, unsigned char _servoSpeed); // 0=full speed, 1-255 slower to faster
//	int readAngle(char _servoNum);  //读舵机角度（暂无）
	void gripperCatch();    //爪抓住
	void gripperRelease();  //爪释放
	void gripperDetach();   //爪分离
    void gripperDirectDetach(); //爪直接分离？
    void pumpOn();          // pump enable 打气泵开
    void pumpOff();         // pump disnable 打气泵关
    void valveOn();         // valve enable, decompression 真空泵开
    void valveOff();        // valve disnable 真空泵关
    void detachServo(char _servoNum);
	void sendData(byte _dataAdd, int _dataIn); //
	void alert(int _times, int _runTime, int _stopTime); //警报
//	void writeEEPROM();
//	void readEEPROM();
//	void play(unsigned char buttonPin);
//	void record(unsigned char buttonPin, unsigned char buttonPinC);
	void servoBufOutL(unsigned char _lastDt, unsigned char _dt);
	void servoBufOutR(unsigned char _lastDt, unsigned char _dt);
	void servoBufOutRot(unsigned char _lastDt, unsigned char _dt);

private:
	/*******************  Servo offset  *******************/
	char offsetL;
	char offsetR;
	/*****************  Define variables  *****************/
	int heightLst;
	int height;
	int stretch; 
	int rotation; 
	int handRot;
	boolean playFlag;
    boolean recordFlag;
    boolean firstFlag;
	boolean gripperRst;
	unsigned char sampleDelay;
	unsigned char servoSpdR;
	unsigned char servoSpdL;
	unsigned char servoSpdRot;
	unsigned char servoSpdHand;
	unsigned char servoSpdHandRot;
	unsigned char leftServoLast;
    unsigned char rightServoLast;
    unsigned char rotServoLast;
	unsigned char griperState[14];
	unsigned char data[3][MEMORY_SERVO_PER+1];  // 0: L  1: R  2: Rotation 
    unsigned long delay_loop;
    unsigned long lstTime;  //limit: 50days
	/***************  Create servo objects  ***************/
	VarSpeedServo servoR;
	VarSpeedServo servoL;
	VarSpeedServo servoRot;
	VarSpeedServo servoHand;
	VarSpeedServo servoHandRot;
};

#endif

