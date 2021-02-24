/*
 * FashionStar�����ɶȻ�е�� STM32 SDK
 * --------------------------
 * ����: ����|Kyle
 * ��kyle.xing@fashionstar.com.hk.com
 * ����ʱ��: 2020/05/19
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

// ״̬��
#define FSARM_STATUS uint8_t
#define FSARM_STATUS_SUCCESS 0 // �ɹ�
#define FSARM_STATUS_FAIL 1 // ʧ��
#define FSARM_STATUS_JOINT1_OUTRANGE 2 // �ؽ�1������Χ
#define FSARM_STATUS_JOINT2_OUTRANGE 3 // �ؽ�2������Χ
#define FSARM_STATUS_JOINT3_OUTRANGE 4 // �ؽ�3������Χ
#define FSARM_STATUS_JOINT4_OUTRANGE 5 // �ؽ�4������Χ
#define FSARM_STATUS_TOOLPOSI_TOO_FAR 6 // ��������Ŀ�������е��̫ңԶ

// ��е�۳���
#define FSARM_SERVO_NUM 4 // ��е�۶���ĸ���
#define FSARM_JOINT1 0 // �ؽ�1��Ӧ�Ķ��ID
#define FSARM_JOINT2 1 // �ؽ�2��Ӧ�Ķ��ID
#define FSARM_JOINT3 2 // �ؽ�3��Ӧ�Ķ��ID
#define FSARM_JOINT4 3 // �ؽ�4��Ӧ�Ķ��ID
#define FSARM_LINK1 0    // ����1�ĳ��� ��λcm
#define FSARM_LINK2 8    // ����2�ĳ��� ��λcm
#define FSARM_LINK3 7.6  // ����3�ĳ��� ��λcm
#define FSARM_LINK4 6.5  // ����4�ĳ��� ��λcm
#define FSARM_TOOL_LENGTH 4.0 // ���ߵĳ���

// ����궨����
#define FSARM_JOINT1_P90 -92.6  //�ؽ�1Ϊ90��ʱ�Ķ��ԭʼ�Ƕ�
#define FSARM_JOINT1_N90 91.4   //�ؽ�1Ϊ-90��ʱ�Ķ��ԭʼ�Ƕ�
#define FSARM_JOINT2_P0 89.3    //�ؽ�2Ϊ0��ʱ�Ķ��ԭʼ�Ƕ�
#define FSARM_JOINT2_N90 -0.2   //�ؽ�2Ϊ-90��ʱ�Ķ��ԭʼ�Ƕ�
#define FSARM_JOINT3_P90 -48.4  //�ؽ�3Ϊ90��ʱ�Ķ��ԭʼ�Ƕ�
#define FSARM_JOINT3_N90 134.8  //�ؽ�3Ϊ-90��ʱ�Ķ��ԭʼ�Ƕ�
#define FSARM_JOINT4_P90 -90.5  //�ؽ�4Ϊ90��ʱ�Ķ��ԭʼ�Ƕ�
#define FSARM_JOINT4_N90 91.4   //�ؽ�4Ϊ-90��ʱ�Ķ��ԭʼ�Ƕ�

// ���ùؽڽǶȵ�Լ��
#define FSARM_JOINT1_MIN -135.0
#define FSARM_JOINT1_MAX 135.0
#define FSARM_JOINT2_MIN -135.0
#define FSARM_JOINT2_MAX 0.0
#define FSARM_JOINT3_MIN -90.0
#define FSARM_JOINT3_MAX 160.0 
#define FSARM_JOINT4_MIN -135.0
#define FSARM_JOINT4_MAX 135.0

// HOME ��е�ۻ�е���Ķ���
#define FSARM_HOME_X 12
#define FSARM_HOME_Y 0
#define FSARM_HOME_Z 10

// ���õ�����
#define PUMP_SERVO_ID 0xFE

// ���ڶ���ĽǶȿ�������(��̬���)
#define FSUS_ANGLE_DEAD_BLOCK 0.2
// �ڶ���ms�ڣ��Ƕ����û�з����仯
#define FSUS_WAIT_TIMEOUT_MS 1000 

// �ѿ����ռ��µĵ�
typedef struct{
    float x;
    float y;
    float z;
}FSARM_POINT3D_T;

// �ؽ�״̬
typedef struct{
    float theta1;
    float theta2;
    float theta3;
    float theta4;
}FSARM_JOINTS_STATE_T;

extern Usart_DataTypeDef *armUsart; // ��е�۵�Usart�ṹ��

extern float kJoint2Servo[FSARM_SERVO_NUM]; 		// ��е�۹ؽڽǶȵ����ԭʼ�Ƕȵı���ϵ��
extern float bJoint2Servo[FSARM_SERVO_NUM]; 		// ��е�۹ؽڽǶȵ����ԭʼ�Ƕȵ�ƫ����
extern float jointAngleLowerb[FSARM_SERVO_NUM]; 	// �ؽڽǶ�����
extern float jointAngleUpperb[FSARM_SERVO_NUM]; 	// �ؽڽǶ�����
extern float curServoAngles[FSARM_SERVO_NUM]; 		// ��ǰ�Ķ���ĽǶ�
extern  float nextServoAngles[FSARM_SERVO_NUM];		// Ŀ�����Ƕ�
extern float armJointSpeed; 							// �ؽڵ���ת�ٶ� ��λdps

// ��ʼ����е��
void FSARM_Init(Usart_DataTypeDef *usart);

// ��е�۹ؽڱ궨
void FSARM_Calibration(void);

// ���û�е�۵Ĺؽڷ�Χ
void FSARM_SetAngleRange(void);

// �����Ƿ���Ť��
void FSARM_SetTorque(bool enable);

// ���ùؽ�Ϊ����ģʽ
void FSARM_SetDamping(uint16_t power);

// �����ؽڽǶ��Ƿ񳬳���Χ
bool FSARM_IsJointLegal(uint8_t jntIdx, float angle);

// ���ùؽ�ת��(����)
void FSARM_SetSpeed(float speed);

// �ؽڽǶ�ת��Ϊ����Ƕ� 
void FSARM_JointAngle2ServoAngle(FSARM_JOINTS_STATE_T jointAngles, FSARM_JOINTS_STATE_T* servoAngles);

// ����Ƕ�ת��Ϊ�ؽڽǶ�
void FSARM_ServoAngle2JointAngle(FSARM_JOINTS_STATE_T servoAngles,FSARM_JOINTS_STATE_T* jointAngles);

// ���ö���ĵ�ǰ�Ƕ�
void FSARM_SetCurServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// ����Ŀ�����Ƕ�
void FSARM_SetNextServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// ������ȡ���ԭʼ�Ƕ�
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles);

// �������ö��ԭʼ�Ƕ�
void FSARM_SetServoAngle(FSARM_JOINTS_STATE_T servoAngles);

// �������ö��ԭʼ�Ƕ�(��ͳһ������)
void FSARM_SetServoAngle2(FSARM_JOINTS_STATE_T servoAngles, uint16_t interval);

// ������ȡ�ؽڵĽǶ�
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles);

// �������ùؽڵĽǶ�
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles);

// �������ùؽڵĽǶ�(��ͳһ������)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval);

// �������ùؽڵĽǶ�(ĩ��ƽ��)
void FSARM_SetJointAngleParallel(FSARM_JOINTS_STATE_T jointAngles);

// ��е�۵������˶�ѧ
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi);

// ��е�������˶�ѧ
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* thetas);

// ��ؽڵ�����ת��Ϊ��������
void FSARM_TransWrist2Tool(FSARM_POINT3D_T endPosi, FSARM_POINT3D_T* toolPosi);

// ĩ�˵�����ת��Ϊ��ؽ�
void FSARM_TransTool2Wrist(FSARM_POINT3D_T toolPosi, FSARM_POINT3D_T* endPosi);

// ���(���ɹ켣)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z);

// ֱ�߲岹
FSARM_STATUS FSARM_MoveLine(float x, float y, float z);

// ��ص���е���
void FSARM_Home(void);

// ��ѯ��е���Ƿ����
void FSARM_IsIdle(void);


// �ȴ������ؽ���ת��Ŀ��Ƕ�
void FSARM_Wait(uint8_t jntIdx);

// �ȴ����еĹؽ���ת��Ŀ��Ƕ�
void FSARM_WaitAll(void);

// ����ĩ�˹��ߵ����� 
void FSARM_GetToolPosi(FSARM_POINT3D_T *toolPosi);

// ���»�е�۵�״̬ 
void FSARM_Update(void);

// ���ÿ���
void FSARM_PumpOn(void);

// ���ùر�
void FSARM_PumpOff(void);

// ���ץȡ
void FSARM_Grab(float x1, float y1, float z1, float x2, float y2, float z2);

#endif
