/*
 * FashionStar�����ɶȻ�е�� STM32 SDK
 * --------------------------
 * ����: ����|Kyle
 * ��kyle.xing@fashionstar.com.hk.com
 * ����ʱ��: 2020/05/19
 */
#include "arm4dof.h"

Usart_DataTypeDef *armUsart; // ��е�۵�Usart�ṹ��
float kJoint2Servo[FSARM_SERVO_NUM]; 		// ��е�۹ؽڽǶȵ����ԭʼ�Ƕȵı���ϵ��
float bJoint2Servo[FSARM_SERVO_NUM]; 		// ��е�۹ؽڽǶȵ����ԭʼ�Ƕȵ�ƫ����
float jointAngleLowerb[FSARM_SERVO_NUM]; 	// �ؽڽǶ�����
float jointAngleUpperb[FSARM_SERVO_NUM]; 	// �ؽڽǶ�����
float curServoAngles[FSARM_SERVO_NUM]; 		// ��ǰ�Ķ���ĽǶ�
float nextServoAngles[FSARM_SERVO_NUM];		// Ŀ�����Ƕ�
float armJointSpeed;

// ��ʼ����е��
void FSARM_Init(Usart_DataTypeDef *usart){
	armUsart = usart;		// ��ʼ��Usart
	FSARM_Calibration();	// ���ػ�е�۹ؽڱ궨������
	FSARM_SetAngleRange(); 	// ���ýǶȷ�Χ
	FSARM_SetSpeed(100.0); 	// ����Ĭ��ת��
	FSARM_Home(); 			// �ع��е�ۻ�е���
	FSARM_WaitAll();		// �ȴ������ת��Ŀ��Ƕ�
}

// ��е�۹ؽڱ궨
void FSARM_Calibration(void){
	// �ؽ�1
	kJoint2Servo[FSARM_JOINT1] = (FSARM_JOINT1_P90 - FSARM_JOINT1_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT1] = FSARM_JOINT1_P90 - kJoint2Servo[FSARM_JOINT1]*90.0;
	// �ؽ�2
	kJoint2Servo[FSARM_JOINT2] = (FSARM_JOINT2_P0 - FSARM_JOINT2_N90) / (0 - (-90.0));
	bJoint2Servo[FSARM_JOINT2] = FSARM_JOINT2_P0 - kJoint2Servo[FSARM_JOINT2]*0.0;
	// �ؽ�3
	kJoint2Servo[FSARM_JOINT3] = (FSARM_JOINT3_P90 - FSARM_JOINT3_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT3] = FSARM_JOINT3_P90 - kJoint2Servo[FSARM_JOINT3]*90.0;
	// �ؽ�4
	kJoint2Servo[FSARM_JOINT4] = (FSARM_JOINT4_P90 - FSARM_JOINT4_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT4] = FSARM_JOINT4_P90 - kJoint2Servo[FSARM_JOINT4]*90.0;
}

// ���û�е�۵Ĺؽڷ�Χ
void FSARM_SetAngleRange(void){
	jointAngleLowerb[FSARM_JOINT1] = FSARM_JOINT1_MIN;
	jointAngleLowerb[FSARM_JOINT2] = FSARM_JOINT2_MIN;
	jointAngleLowerb[FSARM_JOINT3] = FSARM_JOINT3_MIN;
	jointAngleLowerb[FSARM_JOINT4] = FSARM_JOINT4_MIN;
	jointAngleUpperb[FSARM_JOINT1] = FSARM_JOINT1_MAX;
	jointAngleUpperb[FSARM_JOINT2] = FSARM_JOINT2_MAX;
	jointAngleUpperb[FSARM_JOINT3] = FSARM_JOINT3_MAX;
	jointAngleUpperb[FSARM_JOINT4] = FSARM_JOINT4_MAX;
}

// �����Ƿ���Ť��
void FSARM_SetTorque(bool enable){
	if(!enable){
		FSARM_SetDamping(0);
	}else{
		FSARM_JOINTS_STATE_T thetas;
		FSARM_QueryJointAngle(&thetas);
		FSARM_SetJointAngle(thetas);
	}
}

// ���ùؽ�Ϊ����ģʽ
void FSARM_SetDamping(uint16_t power){
	FSUS_DampingMode(armUsart, FSARM_JOINT1, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT2, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT3, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT4, power);
}

// �����ؽڽǶ��Ƿ񳬳���Χ
bool FSARM_IsJointLegal(uint8_t jntIdx, float angle){
	return angle >= jointAngleLowerb[jntIdx] && angle <= jointAngleUpperb[jntIdx];
}

// ���ùؽ�ת��(����)
void FSARM_SetSpeed(float speed){
	armJointSpeed = speed;
}

// �ؽڽǶ�ת��Ϊ����Ƕ�
void FSARM_JointAngle2ServoAngle(FSARM_JOINTS_STATE_T jointAngles, FSARM_JOINTS_STATE_T* servoAngles){
	servoAngles->theta1 =  kJoint2Servo[FSARM_JOINT1] * jointAngles.theta1 + bJoint2Servo[FSARM_JOINT1];
	servoAngles->theta2 =  kJoint2Servo[FSARM_JOINT2] * jointAngles.theta2 + bJoint2Servo[FSARM_JOINT2];
	servoAngles->theta3 =  kJoint2Servo[FSARM_JOINT3] * jointAngles.theta3 + bJoint2Servo[FSARM_JOINT3];
	servoAngles->theta4 =  kJoint2Servo[FSARM_JOINT4] * jointAngles.theta4 + bJoint2Servo[FSARM_JOINT4];
}

// ����Ƕ�ת��Ϊ�ؽڽǶ�
void FSARM_ServoAngle2JointAngle(FSARM_JOINTS_STATE_T servoAngles,FSARM_JOINTS_STATE_T* jointAngles){
	jointAngles->theta1 = (servoAngles.theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
	jointAngles->theta2 = (servoAngles.theta2 - bJoint2Servo[FSARM_JOINT2]) / kJoint2Servo[FSARM_JOINT2];
	jointAngles->theta3 = (servoAngles.theta3 - bJoint2Servo[FSARM_JOINT3]) / kJoint2Servo[FSARM_JOINT3];
	jointAngles->theta4 = (servoAngles.theta4 - bJoint2Servo[FSARM_JOINT4]) / kJoint2Servo[FSARM_JOINT4];
}

// ���ö���ĵ�ǰ�Ƕ�
void FSARM_SetCurServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	curServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	curServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	curServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	curServoAngles[FSARM_JOINT4] = servoAngles.theta4;
}

// ���ö����Ŀ��Ƕ�
void FSARM_SetNextServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	nextServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	nextServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	nextServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	nextServoAngles[FSARM_JOINT4] = servoAngles.theta4;
}

// ������ȡ���ԭʼ�Ƕ�
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles){
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(servoAngles->theta1));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT2, &(servoAngles->theta2));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT3, &(servoAngles->theta3));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT4, &(servoAngles->theta4));
	// ���µ�ǰ����Ƕ�
	FSARM_SetCurServoAngle(*servoAngles);
}

// �������ö��ԭʼ�Ƕ�
void FSARM_SetServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	FSARM_JOINTS_STATE_T curServoAngles;
	uint16_t interval;
	FSARM_QueryServoAngle(&curServoAngles);
	// ���Ͷ���Ƕȿ���ָ��
	interval = (uint16_t)fabs((servoAngles.theta1 - curServoAngles.theta1)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT1, servoAngles.theta1, interval, 0, false);	
	interval = (uint16_t)fabs((servoAngles.theta2 - curServoAngles.theta2)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT2, servoAngles.theta2, interval, 0, false);	
	interval = (uint16_t)fabs((servoAngles.theta3 - curServoAngles.theta3)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT3, servoAngles.theta3, interval, 0, false);
	interval = (uint16_t)fabs((servoAngles.theta4 - curServoAngles.theta4)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT4, servoAngles.theta4, interval, 0, false);
	// ����Ŀ��Ƕ�
	FSARM_SetNextServoAngle(servoAngles);
}

// �������ö��ԭʼ�Ƕ�(��ͳһ������)
void FSARM_SetServoAngle2(FSARM_JOINTS_STATE_T servoAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// ���Ͷ���Ƕȿ���ָ
	FSUS_SetServoAngle(armUsart, FSARM_JOINT1, servoAngles.theta1, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT2, servoAngles.theta2, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT3, servoAngles.theta3, interval, 0, false);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT4, servoAngles.theta4, interval, 0, false);
	// ���¶��Ŀ��Ƕ�ֵ
	FSARM_SetNextServoAngle(servoAngles);
}

// ������ȡ�ؽڵĽǶ�
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles){
	// ��ѯ���ԭʼ�Ƕ�
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// ӳ��Ϊ�ؽڽǶ�
	FSARM_ServoAngle2JointAngle(curServoAngles, jointAngles);
}

// �������ùؽڵĽǶ�
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles){
	FSARM_JOINTS_STATE_T servoAngles;
	// ӳ��Ϊ����Ƕ�
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// ���ö���Ƕ�
	FSARM_SetServoAngle(servoAngles);
}

// �������ùؽڵĽǶ�(��ͳһ������)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T servoAngles;
	// ӳ��Ϊ����Ƕ�
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// ���ö���Ƕ�
	FSARM_SetServoAngle2(servoAngles, interval);
}

// �������ùؽڵĽǶ�(ĩ��ƽ��)
void FSARM_SetJointAngleParallel(FSARM_JOINTS_STATE_T jointAngles){
	// ��ѯ��е�۹ؽڵĽǶ�
	FSARM_JOINTS_STATE_T curJointAngles;
	FSARM_JOINTS_STATE_T nextJointAngles;
	FSARM_QueryJointAngle(&curJointAngles);
	// ����ǶȲ�ֵ
	float dtheta1 = jointAngles.theta1 - curJointAngles.theta1;
	float dtheta2 = jointAngles.theta2 - curJointAngles.theta2;
	float dtheta3 = jointAngles.theta3 - curJointAngles.theta3;
	float dtheta4 = jointAngles.theta4 - curJointAngles.theta4;	
	float ratio;
	
	// �ҵ��Ƕȵ�����ֵ
	float maxDis = max(max(fabs(dtheta1), fabs(dtheta2)), max(fabs(dtheta3), fabs(dtheta4)));
	long nStep = (long)(maxDis / 2.0); // 1.0��
	TimeTypedef tStart;
	long tDelay;
	for(long i=1; i<=nStep; i++){
		tStart = SysTick_Millis(); // ��ʼ��ʱ
		ratio = (1.0*i)/nStep; // ��ȡ����
		ratio = 0.5*(1-sin(radians(90.0 + 180.0*ratio))); // ��ratio����ƽ��
		// ����ؽڽǶ�
		nextJointAngles.theta1 = curJointAngles.theta1 + dtheta1*ratio;
		nextJointAngles.theta2 = curJointAngles.theta2 + dtheta2*ratio;
		nextJointAngles.theta3 = curJointAngles.theta3 + dtheta3*ratio;
		nextJointAngles.theta4 = curJointAngles.theta4 + dtheta4*ratio;
		// ���ùؽڽǶ�
		FSARM_SetJointAngle2(nextJointAngles, 0);
		// �ȴ�
		// TODO 20.0ms
		tDelay = 10 - (SysTick_Millis() - tStart);
		if(tDelay > 0){
			// �ȴ�20ms
			SysTick_DelayMs((TimeTypedef)tDelay);
		}
	}
	// �ȴ������ת��Ŀ��λ��
	FSARM_WaitAll();
}

// ��е�۵������˶�ѧ
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi){
	FSARM_POINT3D_T wristPosi; //��ؽ�ԭ�������
	// �ؽڽǶ�->����
	float theta1 = radians(jointAngles.theta1);
	float theta2 = radians(jointAngles.theta2);
	float theta3 = radians(jointAngles.theta3);
	float theta4 = radians(jointAngles.theta4);
	// ������ؽڵ�����
    wristPosi.x = cos(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3));
    wristPosi.y = sin(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3));
    wristPosi.z = -FSARM_LINK2*sin(theta2)-FSARM_LINK3*sin(theta2+theta3);
	// ����ĩ�˵�����
	FSARM_TransWrist2Tool(wristPosi, toolPosi);
}

// ��е�������˶�ѧ
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* jointAngles){
	// �ؽڻ���
    float theta1 = 0.0;
    float theta2 = 0.0;
    float theta3 = 0.0;
    float theta4 = 0.0;
	
	FSARM_POINT3D_T wristPosi; // ��ؽ�����
	FSARM_TransTool2Wrist(toolPosi, &wristPosi); // ��������ת��Ϊ��ؽڵ�����
	
	// ������ؽ�ԭ������е�ۻ�����ϵ��ֱ�߾���
    float disO2WristXY = sqrt(pow(wristPosi.x,2) + pow(wristPosi.y, 2) + pow(wristPosi.z, 2));
    if (disO2WristXY > (FSARM_LINK2+FSARM_LINK3)){
        return FSARM_STATUS_TOOLPOSI_TOO_FAR;
    }
	
	// �ж���ؽڵ�ԭ���Ƿ��ڻ�е������ϵ��Z����
    if (toolPosi.x == 0 && toolPosi.y == 0){
        // ��ѯ�ؽ�1�ĽǶ�, ��theta1���ָ�ԭ����ͬ
		FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(jointAngles->theta1));
		jointAngles->theta1 = (jointAngles->theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
        theta1 = radians(jointAngles->theta1);
    }else{
        // ���theta1
        theta1 = atan2(toolPosi.y, toolPosi.x);
        jointAngles->theta1 = degrees(theta1);
        // �ж�theta1�Ƿ�Ϸ�
        if (!FSARM_IsJointLegal(FSARM_JOINT1, jointAngles->theta1)){
            return FSARM_STATUS_JOINT1_OUTRANGE;
        }
    }
	// ����theta3
    float b;
    if(cos(theta1) !=0){
        b = wristPosi.x / cos(theta1);
    }else{
        b = wristPosi.y / sin(theta1);
    }
    float cos_theta3 = (pow(wristPosi.z, 2)+pow(b,2) - pow(FSARM_LINK2,2) - pow(FSARM_LINK3, 2))/(2*FSARM_LINK2*FSARM_LINK3);
    float sin_theta3 = sqrt(1 - pow(cos_theta3, 2));
    theta3 = atan2(sin_theta3, cos_theta3);
    jointAngles->theta3 = degrees(theta3);
    if(!FSARM_IsJointLegal(FSARM_JOINT3, jointAngles->theta3)){
        return FSARM_STATUS_JOINT3_OUTRANGE;
    }
    // ����theta2
    float k1 = FSARM_LINK2 + FSARM_LINK3*cos(theta3);
    float k2 = FSARM_LINK3 * sin(theta3);
    float r = sqrt(pow(k1, 2) + pow(k2, 2));
    theta2 = atan2(-wristPosi.z/r, b/r) - atan2(k2/r, k1/r);
    jointAngles->theta2 = degrees(theta2);
    if(!FSARM_IsJointLegal(FSARM_JOINT2, jointAngles->theta2)){
        return FSARM_STATUS_JOINT2_OUTRANGE;
    }
    // ����theta4
    theta4 = -(theta2 + theta3);
    jointAngles->theta4 = degrees(theta4);
    if(!FSARM_IsJointLegal(FSARM_JOINT4, jointAngles->theta4)){
        return FSARM_STATUS_JOINT4_OUTRANGE;
    }
    // �ɹ�������
    return FSARM_STATUS_SUCCESS;
}

// ��ؽڵ�����ת��Ϊ��������
void FSARM_TransWrist2Tool(FSARM_POINT3D_T endPosi, FSARM_POINT3D_T* toolPosi){
	float theta1_rad = atan2(endPosi.y, endPosi.x);
	toolPosi->x = endPosi.x + FSARM_LINK4*cos(theta1_rad);
	toolPosi->y = endPosi.y + FSARM_LINK4*sin(theta1_rad);
	toolPosi->z = endPosi.z - FSARM_TOOL_LENGTH;
}

// ĩ�˵�����ת��Ϊ��ؽ�
void FSARM_TransTool2Wrist(FSARM_POINT3D_T toolPosi, FSARM_POINT3D_T* endPosi){
	float theta1_rad = atan2(toolPosi.y, toolPosi.x); 
    endPosi->x = toolPosi.x - FSARM_LINK4*cos(theta1_rad);
    endPosi->y = toolPosi.y - FSARM_LINK4*sin(theta1_rad);
    endPosi->z = toolPosi.z + FSARM_TOOL_LENGTH;
}

// ���(���ɹ켣)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z){
	FSARM_JOINTS_STATE_T jointAngles;
	FSARM_POINT3D_T toolPosi;
	toolPosi.x = x;
	toolPosi.y = y;
	toolPosi.z = z;
    FSARM_STATUS status = FSARM_InverseKinematics(toolPosi, &jointAngles); // �����˶�ѧ
    if(status == FSARM_STATUS_SUCCESS){
        // ���ö���ĽǶ�
        FSARM_SetJointAngleParallel(jointAngles);
    }
    return status;
}

// ֱ�߲岹
FSARM_STATUS FSARM_MoveLine(float x, float y, float z){
	FSARM_POINT3D_T curPosi;
    FSARM_POINT3D_T nextPosi;
	FSARM_JOINTS_STATE_T jointAngles;
	
    FSARM_STATUS status;
    // ��ȡ���ߵ�λ��
    FSARM_GetToolPosi(&curPosi);
    // ����ƫ����
    float dx = x - curPosi.x;
    float dy = y - curPosi.y;
    float dz = z - curPosi.z;
    float distance = sqrt(dx*dx + dy*dy +dz*dz);
    float nx, ny, nz;

    float ratio;
    long nStep = (long)(distance / 0.4);
	TimeTypedef tStart; // ��ʼ��ʱ��ʱ��
	long tDelay;
	
    for (long i=1; i<=nStep; i++){
        tStart = SysTick_Millis();
        ratio = (i*1.0 / nStep); // ratio��ȡֵ��Χ [0, 1]
        // ͨ��S���߶�ratio����ƽ��
        ratio = 0.5*(1-sin(radians(90+ 180 *ratio)));
        // ��д
        nx = curPosi.x + dx*ratio;
        ny = curPosi.y + dy*ratio;
        nz = curPosi.z + dz*ratio;
        nextPosi.x = nx;
        nextPosi.y = ny;
        nextPosi.z = nz;

        // �����˶�ѧ���
        status = FSARM_InverseKinematics(nextPosi, &jointAngles); // �����˶�ѧ
        if(status == FSARM_STATUS_SUCCESS){
			FSARM_SetJointAngle2(jointAngles, 0); // ����Ŀ��Ƕ�
			tDelay = 10 - (SysTick_Millis() - tStart);
			if(tDelay > 0){
				SysTick_DelayMs((TimeTypedef)tDelay); // �ȴ�20ms
			}
        }
    }
	
    // �ȴ�����Ŀ�ĵ�
    FSARM_WaitAll();
    return status;
}

// ��ص���е���
void FSARM_Home(void){
	// ���ɹ켣 �˶�����е���
	FSARM_MoveP2P(FSARM_HOME_X, FSARM_HOME_Y, FSARM_HOME_Z);
	FSARM_WaitAll();
}

// ��ѯ��е���Ƿ����
void FSARM_IsIdle(void){
	
}


// ���»�е�۵�״̬ 
void FSARM_Update(void){
	// ���¶���ĽǶ�
	FSARM_JOINTS_STATE_T servoAngles;
	FSARM_QueryServoAngle(&servoAngles);
	FSARM_SetCurServoAngle(servoAngles);
}

// �ȴ������ؽ���ת��Ŀ��Ƕ�
void FSARM_Wait(uint8_t jntIdx){
	// �Ƕ����
    float dAngle = fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]);
	TimeTypedef tStart;
	while(true){
		FSARM_Update(); // ���»�е�۵�״̬
		if(fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]) <= FSUS_ANGLE_DEAD_BLOCK){
			break;
		}else{
			// �ж��Ƿ������������
			// ����ֵ���ֲ���
			if(fabs(dAngle) < 5.0 && fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx])) <= 1.0){
				// �ж��Ƿ�ʱ
				if(SysTick_Millis() - tStart >= FSUS_WAIT_TIMEOUT_MS){
					break;
				}
			}else{
				// ���½Ƕ����
				dAngle = fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]));
				// ��ʼ����ʱ
				// SysTick_CountdownBegin(FSUS_WAIT_TIMEOUT_MS);
				tStart = SysTick_Millis();
			}
		}
	}
}

// �ȴ����еĹؽ���ת��Ŀ��Ƕ�
void FSARM_WaitAll(void){
	for(uint8_t srvIdx=0; srvIdx < FSARM_SERVO_NUM; srvIdx++){
		FSARM_Wait(srvIdx);
	}
}

// ����ĩ�˹��ߵ����� 
void FSARM_GetToolPosi(FSARM_POINT3D_T *toolPosi){
	FSARM_JOINTS_STATE_T curJointAngles; // ��ǰ�ؽڵĽǶ�
	FSARM_QueryJointAngle(&curJointAngles); // ��ѯ�ؽڽǶ�
	FSARM_ForwardKinmatics(curJointAngles, toolPosi);// �����˶�ѧ
}

// ���ÿ���
void FSARM_PumpOn(void){
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, -90, 0, 0, false); // �򿪵��
}

// ���ùر�
void FSARM_PumpOff(void){
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, 0, 0, 0, false);  // �رյ��
	SysTick_DelayMs(200);
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, 90, 0, 0, false); // �򿪵�ŷ�
	SysTick_DelayMs(200);
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, 0, 0, 0, false);  // �رյ�ŷ�
}

// ���ץȡ
void FSARM_Grab(float x1, float y1, float z1, float x2, float y2, float z2){
	FSARM_MoveLine(x1, y1, z1+4); // �˶���Ŀ����Ϸ�
    SysTick_DelayMs(200);
    FSARM_MoveLine(x1, y1, z1); // �˶���Ŀ���
    SysTick_DelayMs(200);
    FSARM_PumpOn(); // ��������
    SysTick_DelayMs(200);
    FSARM_SetSpeed(50);
    FSARM_MoveLine(x1, y1, z1-1); // ������ѹ
    SysTick_DelayMs(200);
    FSARM_MoveLine(x1, y1, z1+4); // ̧��
    SysTick_DelayMs(200);
    FSARM_SetSpeed(100);
    FSARM_MoveLine(x2, y2, z2+4); // �˶���Ŀ����Ϸ�
    SysTick_DelayMs(200);
    FSARM_SetSpeed(50);
    FSARM_MoveLine(x2, y2, z2); // �˶���Ŀ���
    SysTick_DelayMs(200);
    FSARM_PumpOff(); // �ر�����
    SysTick_DelayMs(200);
    FSARM_MoveLine(x2, y2, z2+4); // �˶���Ŀ����Ϸ�
    FSARM_SetSpeed(100);
}
