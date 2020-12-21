/*
 * FashionStar四自由度机械臂 STM32 SDK
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/19
 */
#ifndef __ARM4DOF_H
#define __ARM4DOF_H

#include "stm32f10x.h"
#include "stdlib.h"
#include "math.h"
#include "common.h"
#include "usart.h"
#include "fashion_star_uart_servo.h"
#include "sys_tick.h"

// 状态码
#define FSARM_STATUS uint8_t
#define FSARM_STATUS_SUCCESS 0 // 成功
#define FSARM_STATUS_FAIL 1 // 失败
#define FSARM_STATUS_JOINT1_OUTRANGE 2 // 关节1超出范围
#define FSARM_STATUS_JOINT2_OUTRANGE 3 // 关节2超出范围
#define FSARM_STATUS_JOINT3_OUTRANGE 4 // 关节3超出范围
#define FSARM_STATUS_JOINT4_OUTRANGE 5 // 关节4超出范围
#define FSARM_STATUS_TOOLPOSI_TOO_FAR 6 // 工具坐标目标点距离机械臂太遥远

// 机械臂常量
#define FSARM_SERVO_NUM 4 // 机械臂舵机的个数
#define FSARM_JOINT1 0 // 关节1对应的舵机ID
#define FSARM_JOINT2 1 // 关节2对应的舵机ID
#define FSARM_JOINT3 2 // 关节3对应的舵机ID
#define FSARM_JOINT4 3 // 关节4对应的舵机ID
#define FSARM_LINK1 0    // 连杆1的长度 单位cm
#define FSARM_LINK2 8    // 连杆2的长度 单位cm
#define FSARM_LINK3 7.6  // 连杆3的长度 单位cm
#define FSARM_LINK4 6.5  // 连杆4的长度 单位cm
#define FSARM_TOOL_LENGTH 4.0 // 工具的长度

// 舵机标定参数
#define FSARM_JOINT1_P90 -92.6  //关节1为90°时的舵机原始角度
#define FSARM_JOINT1_N90 91.4   //关节1为-90°时的舵机原始角度
#define FSARM_JOINT2_P0 89.3    //关节2为0°时的舵机原始角度
#define FSARM_JOINT2_N90 -0.2   //关节2为-90°时的舵机原始角度
#define FSARM_JOINT3_P90 -48.4  //关节3为90°时的舵机原始角度
#define FSARM_JOINT3_N90 134.8  //关节3为-90°时的舵机原始角度
#define FSARM_JOINT4_P90 -90.5  //关节4为90°时的舵机原始角度
#define FSARM_JOINT4_N90 91.4   //关节4为-90°时的舵机原始角度

// 设置关节角度的约束
#define FSARM_JOINT1_MIN -135.0
#define FSARM_JOINT1_MAX 135.0
#define FSARM_JOINT2_MIN -135.0
#define FSARM_JOINT2_MAX 0.0
#define FSARM_JOINT3_MIN -90.0
#define FSARM_JOINT3_MAX 160.0 
#define FSARM_JOINT4_MIN -135.0
#define FSARM_JOINT4_MAX 135.0

// HOME 机械臂机械零点的定义
#define FSARM_HOME_X 12
#define FSARM_HOME_Y 0
#define FSARM_HOME_Z 10

// 气泵的配置
#define PUMP_SERVO_ID 0xFE

// 串口舵机的角度控制死区(稳态误差)
#define FSUS_ANGLE_DEAD_BLOCK 0.2
// 在多少ms内，角度误差没有发生变化
#define FSUS_WAIT_TIMEOUT_MS 1000 

// 笛卡尔空间下的点
typedef struct{
    float x;
    float y;
    float z;
}FSARM_POINT3D_T;

// 关节状态
typedef struct{
    float theta1;
    float theta2;
    float theta3;
    float theta4;
}FSARM_JOINTS_STATE_T;

extern Usart_DataTypeDef *armUsart; // 机械臂的Usart结构体

extern float kJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的比例系数
extern float bJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的偏移量
extern float jointAngleLowerb[FSARM_SERVO_NUM]; 	// 关节角度下限
extern float jointAngleUpperb[FSARM_SERVO_NUM]; 	// 关节角度上限
extern float curServoAngles[FSARM_SERVO_NUM]; 		// 当前的舵机的角度
extern  float nextServoAngles[FSARM_SERVO_NUM];		// 目标舵机角度
extern float armJointSpeed; 							// 关节的旋转速度 单位dps

// 初始化机械臂
void FSARM_Init(Usart_DataTypeDef *usart);

// 机械臂关节标定
void FSARM_Calibration(void);

// 设置机械臂的关节范围
void FSARM_SetAngleRange(void);

// 设置是否开启扭矩
void FSARM_SetTorque(bool enable);

// 设置关节为阻尼模式
void FSARM_SetDamping(uint16_t power);

// 单个关节角度是否超出范围
bool FSARM_IsJointLegal(uint8_t jntIdx, float angle);

// 设置关节转速(估计)
void FSARM_SetSpeed(float speed);

// 关节角度转换为舵机角度 
void FSARM_JointAngle2ServoAngle(FSARM_JOINTS_STATE_T jointAngles, FSARM_JOINTS_STATE_T* servoAngles);

// 舵机角度转换为关节角度
void FSARM_ServoAngle2JointAngle(FSARM_JOINTS_STATE_T servoAngles,FSARM_JOINTS_STATE_T* jointAngles);

// 设置舵机的当前角度
void FSARM_SetCurServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// 设置目标舵机角度
void FSARM_SetNextServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// 批量读取舵机原始角度
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles);

// 批量设置舵机原始角度
void FSARM_SetServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// 批量设置舵机原始角度(带统一的周期)
void FSARM_SetServoAngle2(FSARM_JOINTS_STATE_T servoAngles, uint16_t interval);

// 批量读取关节的角度
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles);

// 批量设置关节的角度
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles);

// 批量设置关节的角度(带统一的周期)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval);

// 批量设置关节的角度(末端平行)
void FSARM_SetJointAngleParallel(FSARM_JOINTS_STATE_T jointAngles);

// 机械臂的正向运动学
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi);

// 机械臂逆向运动学
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* thetas);

// 腕关节的坐标转换为工具坐标
void FSARM_TransWrist2Tool(FSARM_POINT3D_T endPosi, FSARM_POINT3D_T* toolPosi);

// 末端的坐标转换为腕关节
void FSARM_TransTool2Wrist(FSARM_POINT3D_T toolPosi, FSARM_POINT3D_T* endPosi);

// 点控(自由轨迹)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z);

// 直线插补
FSARM_STATUS FSARM_MoveLine(float x, float y, float z);

// 归回到机械零点
void FSARM_Home(void);

// 查询机械臂是否空闲
void FSARM_IsIdle(void);


// 等待当个关节旋转到目标角度
void FSARM_Wait(uint8_t jntIdx);

// 等待所有的关节旋转到目标角度
void FSARM_WaitAll(void);

// 计算末端工具的坐标 
void FSARM_GetToolPosi(FSARM_POINT3D_T *toolPosi);

// 更新机械臂的状态 
void FSARM_Update(void);

// 气泵开启
void FSARM_PumpOn(void);

// 气泵关闭
void FSARM_PumpOff(void);

// 物块抓取
void FSARM_Grab(float x1, float y1, float z1, float x2, float y2, float z2);

#endif
