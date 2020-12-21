/*
 * FashionStar四自由度机械臂 STM32 SDK
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/19
 */
#include "arm4dof.h"

Usart_DataTypeDef *armUsart; // 机械臂的Usart结构体
float kJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的比例系数
float bJoint2Servo[FSARM_SERVO_NUM]; 		// 机械臂关节角度到舵机原始角度的偏移量
float jointAngleLowerb[FSARM_SERVO_NUM]; 	// 关节角度下限
float jointAngleUpperb[FSARM_SERVO_NUM]; 	// 关节角度上限
float curServoAngles[FSARM_SERVO_NUM]; 		// 当前的舵机的角度
float nextServoAngles[FSARM_SERVO_NUM];		// 目标舵机角度
float armJointSpeed;

// 初始化机械臂
void FSARM_Init(Usart_DataTypeDef *usart){
	armUsart = usart;		// 初始化Usart
	FSARM_Calibration();	// 加载机械臂关节标定的数据
	FSARM_SetAngleRange(); 	// 设置角度范围
	FSARM_SetSpeed(100.0); 	// 设置默认转速
	FSARM_Home(); 			// 回归机械臂机械零点
	FSARM_WaitAll();		// 等待舵机旋转到目标角度
}

// 机械臂关节标定
void FSARM_Calibration(void){
	// 关节1
	kJoint2Servo[FSARM_JOINT1] = (FSARM_JOINT1_P90 - FSARM_JOINT1_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT1] = FSARM_JOINT1_P90 - kJoint2Servo[FSARM_JOINT1]*90.0;
	// 关节2
	kJoint2Servo[FSARM_JOINT2] = (FSARM_JOINT2_P0 - FSARM_JOINT2_N90) / (0 - (-90.0));
	bJoint2Servo[FSARM_JOINT2] = FSARM_JOINT2_P0 - kJoint2Servo[FSARM_JOINT2]*0.0;
	// 关节3
	kJoint2Servo[FSARM_JOINT3] = (FSARM_JOINT3_P90 - FSARM_JOINT3_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT3] = FSARM_JOINT3_P90 - kJoint2Servo[FSARM_JOINT3]*90.0;
	// 关节4
	kJoint2Servo[FSARM_JOINT4] = (FSARM_JOINT4_P90 - FSARM_JOINT4_N90) / (90.0 - (-90.0));
	bJoint2Servo[FSARM_JOINT4] = FSARM_JOINT4_P90 - kJoint2Servo[FSARM_JOINT4]*90.0;
}

// 设置机械臂的关节范围
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

// 设置是否开启扭矩
void FSARM_SetTorque(bool enable){
	if(!enable){
		FSARM_SetDamping(0);
	}else{
		FSARM_JOINTS_STATE_T thetas;
		FSARM_QueryJointAngle(&thetas);
		FSARM_SetJointAngle(thetas);
	}
}

// 设置关节为阻尼模式
void FSARM_SetDamping(uint16_t power){
	FSUS_DampingMode(armUsart, FSARM_JOINT1, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT2, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT3, power);
	FSUS_DampingMode(armUsart, FSARM_JOINT4, power);
}

// 单个关节角度是否超出范围
bool FSARM_IsJointLegal(uint8_t jntIdx, float angle){
	return angle >= jointAngleLowerb[jntIdx] && angle <= jointAngleUpperb[jntIdx];
}

// 设置关节转速(估计)
void FSARM_SetSpeed(float speed){
	armJointSpeed = speed;
}

// 关节角度转换为舵机角度
void FSARM_JointAngle2ServoAngle(FSARM_JOINTS_STATE_T jointAngles, FSARM_JOINTS_STATE_T* servoAngles){
	servoAngles->theta1 =  kJoint2Servo[FSARM_JOINT1] * jointAngles.theta1 + bJoint2Servo[FSARM_JOINT1];
	servoAngles->theta2 =  kJoint2Servo[FSARM_JOINT2] * jointAngles.theta2 + bJoint2Servo[FSARM_JOINT2];
	servoAngles->theta3 =  kJoint2Servo[FSARM_JOINT3] * jointAngles.theta3 + bJoint2Servo[FSARM_JOINT3];
	servoAngles->theta4 =  kJoint2Servo[FSARM_JOINT4] * jointAngles.theta4 + bJoint2Servo[FSARM_JOINT4];
}

// 舵机角度转换为关节角度
void FSARM_ServoAngle2JointAngle(FSARM_JOINTS_STATE_T servoAngles,FSARM_JOINTS_STATE_T* jointAngles){
	jointAngles->theta1 = (servoAngles.theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
	jointAngles->theta2 = (servoAngles.theta2 - bJoint2Servo[FSARM_JOINT2]) / kJoint2Servo[FSARM_JOINT2];
	jointAngles->theta3 = (servoAngles.theta3 - bJoint2Servo[FSARM_JOINT3]) / kJoint2Servo[FSARM_JOINT3];
	jointAngles->theta4 = (servoAngles.theta4 - bJoint2Servo[FSARM_JOINT4]) / kJoint2Servo[FSARM_JOINT4];
}

// 设置舵机的当前角度
void FSARM_SetCurServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	curServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	curServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	curServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	curServoAngles[FSARM_JOINT4] = servoAngles.theta4;
}

// 设置舵机的目标角度
void FSARM_SetNextServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	nextServoAngles[FSARM_JOINT1] = servoAngles.theta1;
	nextServoAngles[FSARM_JOINT2] = servoAngles.theta2;
	nextServoAngles[FSARM_JOINT3] = servoAngles.theta3;
	nextServoAngles[FSARM_JOINT4] = servoAngles.theta4;
}

// 批量读取舵机原始角度
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles){
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(servoAngles->theta1));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT2, &(servoAngles->theta2));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT3, &(servoAngles->theta3));
	FSUS_QueryServoAngle(armUsart, FSARM_JOINT4, &(servoAngles->theta4));
	// 更新当前舵机角度
	FSARM_SetCurServoAngle(*servoAngles);
}

// 批量设置舵机原始角度
void FSARM_SetServoAngle(FSARM_JOINTS_STATE_T servoAngles){
	FSARM_JOINTS_STATE_T curServoAngles;
	uint16_t interval;
	FSARM_QueryServoAngle(&curServoAngles);
	// 发送舵机角度控制指令
	interval = (uint16_t)fabs((servoAngles.theta1 - curServoAngles.theta1)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT1, servoAngles.theta1, interval, 0, false);	
	interval = (uint16_t)fabs((servoAngles.theta2 - curServoAngles.theta2)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT2, servoAngles.theta2, interval, 0, false);	
	interval = (uint16_t)fabs((servoAngles.theta3 - curServoAngles.theta3)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT3, servoAngles.theta3, interval, 0, false);
	interval = (uint16_t)fabs((servoAngles.theta4 - curServoAngles.theta4)/armJointSpeed*1000);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT4, servoAngles.theta4, interval, 0, false);
	// 设置目标角度
	FSARM_SetNextServoAngle(servoAngles);
}

// 批量设置舵机原始角度(带统一的周期)
void FSARM_SetServoAngle2(FSARM_JOINTS_STATE_T servoAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// 发送舵机角度控制指
	FSUS_SetServoAngle(armUsart, FSARM_JOINT1, servoAngles.theta1, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT2, servoAngles.theta2, interval, 0, false);	
	FSUS_SetServoAngle(armUsart, FSARM_JOINT3, servoAngles.theta3, interval, 0, false);
	FSUS_SetServoAngle(armUsart, FSARM_JOINT4, servoAngles.theta4, interval, 0, false);
	// 更新舵机目标角度值
	FSARM_SetNextServoAngle(servoAngles);
}

// 批量读取关节的角度
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles){
	// 查询舵机原始角度
	FSARM_JOINTS_STATE_T curServoAngles;
	FSARM_QueryServoAngle(&curServoAngles);
	// 映射为关节角度
	FSARM_ServoAngle2JointAngle(curServoAngles, jointAngles);
}

// 批量设置关节的角度
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles){
	FSARM_JOINTS_STATE_T servoAngles;
	// 映射为舵机角度
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// 设置舵机角度
	FSARM_SetServoAngle(servoAngles);
}

// 批量设置关节的角度(带统一的周期)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval){
	FSARM_JOINTS_STATE_T servoAngles;
	// 映射为舵机角度
	FSARM_JointAngle2ServoAngle(jointAngles, &servoAngles);
	// 设置舵机角度
	FSARM_SetServoAngle2(servoAngles, interval);
}

// 批量设置关节的角度(末端平行)
void FSARM_SetJointAngleParallel(FSARM_JOINTS_STATE_T jointAngles){
	// 查询机械臂关节的角度
	FSARM_JOINTS_STATE_T curJointAngles;
	FSARM_JOINTS_STATE_T nextJointAngles;
	FSARM_QueryJointAngle(&curJointAngles);
	// 计算角度差值
	float dtheta1 = jointAngles.theta1 - curJointAngles.theta1;
	float dtheta2 = jointAngles.theta2 - curJointAngles.theta2;
	float dtheta3 = jointAngles.theta3 - curJointAngles.theta3;
	float dtheta4 = jointAngles.theta4 - curJointAngles.theta4;	
	float ratio;
	
	// 找到角度的最大差值
	float maxDis = max(max(fabs(dtheta1), fabs(dtheta2)), max(fabs(dtheta3), fabs(dtheta4)));
	long nStep = (long)(maxDis / 2.0); // 1.0°
	TimeTypedef tStart;
	long tDelay;
	for(long i=1; i<=nStep; i++){
		tStart = SysTick_Millis(); // 开始计时
		ratio = (1.0*i)/nStep; // 获取进度
		ratio = 0.5*(1-sin(radians(90.0 + 180.0*ratio))); // 对ratio进行平滑
		// 计算关节角度
		nextJointAngles.theta1 = curJointAngles.theta1 + dtheta1*ratio;
		nextJointAngles.theta2 = curJointAngles.theta2 + dtheta2*ratio;
		nextJointAngles.theta3 = curJointAngles.theta3 + dtheta3*ratio;
		nextJointAngles.theta4 = curJointAngles.theta4 + dtheta4*ratio;
		// 设置关节角度
		FSARM_SetJointAngle2(nextJointAngles, 0);
		// 等待
		// TODO 20.0ms
		tDelay = 10 - (SysTick_Millis() - tStart);
		if(tDelay > 0){
			// 等待20ms
			SysTick_DelayMs((TimeTypedef)tDelay);
		}
	}
	// 等待舵机旋转到目标位置
	FSARM_WaitAll();
}

// 机械臂的正向运动学
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi){
	FSARM_POINT3D_T wristPosi; //腕关节原点的坐标
	// 关节角度->弧度
	float theta1 = radians(jointAngles.theta1);
	float theta2 = radians(jointAngles.theta2);
	float theta3 = radians(jointAngles.theta3);
	float theta4 = radians(jointAngles.theta4);
	// 计算腕关节的坐标
    wristPosi.x = cos(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3));
    wristPosi.y = sin(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3));
    wristPosi.z = -FSARM_LINK2*sin(theta2)-FSARM_LINK3*sin(theta2+theta3);
	// 计算末端的坐标
	FSARM_TransWrist2Tool(wristPosi, toolPosi);
}

// 机械臂逆向运动学
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* jointAngles){
	// 关节弧度
    float theta1 = 0.0;
    float theta2 = 0.0;
    float theta3 = 0.0;
    float theta4 = 0.0;
	
	FSARM_POINT3D_T wristPosi; // 腕关节坐标
	FSARM_TransTool2Wrist(toolPosi, &wristPosi); // 工具坐标转换为腕关节的坐标
	
	// 根据腕关节原点距离机械臂基坐标系的直线距离
    float disO2WristXY = sqrt(pow(wristPosi.x,2) + pow(wristPosi.y, 2) + pow(wristPosi.z, 2));
    if (disO2WristXY > (FSARM_LINK2+FSARM_LINK3)){
        return FSARM_STATUS_TOOLPOSI_TOO_FAR;
    }
	
	// 判断腕关节的原点是否在机械臂坐标系的Z轴上
    if (toolPosi.x == 0 && toolPosi.y == 0){
        // 查询关节1的角度, 让theta1保持跟原来相同
		FSUS_QueryServoAngle(armUsart, FSARM_JOINT1, &(jointAngles->theta1));
		jointAngles->theta1 = (jointAngles->theta1 - bJoint2Servo[FSARM_JOINT1]) / kJoint2Servo[FSARM_JOINT1];
        theta1 = radians(jointAngles->theta1);
    }else{
        // 求解theta1
        theta1 = atan2(toolPosi.y, toolPosi.x);
        jointAngles->theta1 = degrees(theta1);
        // 判断theta1是否合法
        if (!FSARM_IsJointLegal(FSARM_JOINT1, jointAngles->theta1)){
            return FSARM_STATUS_JOINT1_OUTRANGE;
        }
    }
	// 计算theta3
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
    // 计算theta2
    float k1 = FSARM_LINK2 + FSARM_LINK3*cos(theta3);
    float k2 = FSARM_LINK3 * sin(theta3);
    float r = sqrt(pow(k1, 2) + pow(k2, 2));
    theta2 = atan2(-wristPosi.z/r, b/r) - atan2(k2/r, k1/r);
    jointAngles->theta2 = degrees(theta2);
    if(!FSARM_IsJointLegal(FSARM_JOINT2, jointAngles->theta2)){
        return FSARM_STATUS_JOINT2_OUTRANGE;
    }
    // 计算theta4
    theta4 = -(theta2 + theta3);
    jointAngles->theta4 = degrees(theta4);
    if(!FSARM_IsJointLegal(FSARM_JOINT4, jointAngles->theta4)){
        return FSARM_STATUS_JOINT4_OUTRANGE;
    }
    // 成功完成求解
    return FSARM_STATUS_SUCCESS;
}

// 腕关节的坐标转换为工具坐标
void FSARM_TransWrist2Tool(FSARM_POINT3D_T endPosi, FSARM_POINT3D_T* toolPosi){
	float theta1_rad = atan2(endPosi.y, endPosi.x);
	toolPosi->x = endPosi.x + FSARM_LINK4*cos(theta1_rad);
	toolPosi->y = endPosi.y + FSARM_LINK4*sin(theta1_rad);
	toolPosi->z = endPosi.z - FSARM_TOOL_LENGTH;
}

// 末端的坐标转换为腕关节
void FSARM_TransTool2Wrist(FSARM_POINT3D_T toolPosi, FSARM_POINT3D_T* endPosi){
	float theta1_rad = atan2(toolPosi.y, toolPosi.x); 
    endPosi->x = toolPosi.x - FSARM_LINK4*cos(theta1_rad);
    endPosi->y = toolPosi.y - FSARM_LINK4*sin(theta1_rad);
    endPosi->z = toolPosi.z + FSARM_TOOL_LENGTH;
}

// 点控(自由轨迹)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z){
	FSARM_JOINTS_STATE_T jointAngles;
	FSARM_POINT3D_T toolPosi;
	toolPosi.x = x;
	toolPosi.y = y;
	toolPosi.z = z;
    FSARM_STATUS status = FSARM_InverseKinematics(toolPosi, &jointAngles); // 逆向运动学
    if(status == FSARM_STATUS_SUCCESS){
        // 设置舵机的角度
        FSARM_SetJointAngleParallel(jointAngles);
    }
    return status;
}

// 直线插补
FSARM_STATUS FSARM_MoveLine(float x, float y, float z){
	FSARM_POINT3D_T curPosi;
    FSARM_POINT3D_T nextPosi;
	FSARM_JOINTS_STATE_T jointAngles;
	
    FSARM_STATUS status;
    // 获取工具的位置
    FSARM_GetToolPosi(&curPosi);
    // 计算偏移量
    float dx = x - curPosi.x;
    float dy = y - curPosi.y;
    float dz = z - curPosi.z;
    float distance = sqrt(dx*dx + dy*dy +dz*dz);
    float nx, ny, nz;

    float ratio;
    long nStep = (long)(distance / 0.4);
	TimeTypedef tStart; // 开始计时的时间
	long tDelay;
	
    for (long i=1; i<=nStep; i++){
        tStart = SysTick_Millis();
        ratio = (i*1.0 / nStep); // ratio的取值范围 [0, 1]
        // 通过S曲线对ratio进行平滑
        ratio = 0.5*(1-sin(radians(90+ 180 *ratio)));
        // 填写
        nx = curPosi.x + dx*ratio;
        ny = curPosi.y + dy*ratio;
        nz = curPosi.z + dz*ratio;
        nextPosi.x = nx;
        nextPosi.y = ny;
        nextPosi.z = nz;

        // 逆向运动学求解
        status = FSARM_InverseKinematics(nextPosi, &jointAngles); // 逆向运动学
        if(status == FSARM_STATUS_SUCCESS){
			FSARM_SetJointAngle2(jointAngles, 0); // 设置目标角度
			tDelay = 10 - (SysTick_Millis() - tStart);
			if(tDelay > 0){
				SysTick_DelayMs((TimeTypedef)tDelay); // 等待20ms
			}
        }
    }
	
    // 等待到达目的地
    FSARM_WaitAll();
    return status;
}

// 归回到机械零点
void FSARM_Home(void){
	// 自由轨迹 运动到机械零点
	FSARM_MoveP2P(FSARM_HOME_X, FSARM_HOME_Y, FSARM_HOME_Z);
	FSARM_WaitAll();
}

// 查询机械臂是否空闲
void FSARM_IsIdle(void){
	
}


// 更新机械臂的状态 
void FSARM_Update(void){
	// 更新舵机的角度
	FSARM_JOINTS_STATE_T servoAngles;
	FSARM_QueryServoAngle(&servoAngles);
	FSARM_SetCurServoAngle(servoAngles);
}

// 等待当个关节旋转到目标角度
void FSARM_Wait(uint8_t jntIdx){
	// 角度误差
    float dAngle = fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]);
	TimeTypedef tStart;
	while(true){
		FSARM_Update(); // 更新机械臂的状态
		if(fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]) <= FSUS_ANGLE_DEAD_BLOCK){
			break;
		}else{
			// 判断是否发生卡死的情况
			// 误差差值保持不变
			if(fabs(dAngle) < 5.0 && fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx])) <= 1.0){
				// 判断是否超时
				if(SysTick_Millis() - tStart >= FSUS_WAIT_TIMEOUT_MS){
					break;
				}
			}else{
				// 更新角度误差
				dAngle = fabs(dAngle - fabs(curServoAngles[jntIdx] - nextServoAngles[jntIdx]));
				// 开始倒计时
				// SysTick_CountdownBegin(FSUS_WAIT_TIMEOUT_MS);
				tStart = SysTick_Millis();
			}
		}
	}
}

// 等待所有的关节旋转到目标角度
void FSARM_WaitAll(void){
	for(uint8_t srvIdx=0; srvIdx < FSARM_SERVO_NUM; srvIdx++){
		FSARM_Wait(srvIdx);
	}
}

// 计算末端工具的坐标 
void FSARM_GetToolPosi(FSARM_POINT3D_T *toolPosi){
	FSARM_JOINTS_STATE_T curJointAngles; // 当前关节的角度
	FSARM_QueryJointAngle(&curJointAngles); // 查询关节角度
	FSARM_ForwardKinmatics(curJointAngles, toolPosi);// 正向运动学
}

// 气泵开启
void FSARM_PumpOn(void){
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, -90, 0, 0, false); // 打开电机
}

// 气泵关闭
void FSARM_PumpOff(void){
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, 0, 0, 0, false);  // 关闭电机
	SysTick_DelayMs(200);
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, 90, 0, 0, false); // 打开电磁阀
	SysTick_DelayMs(200);
	FSUS_SetServoAngle(armUsart, PUMP_SERVO_ID, 0, 0, 0, false);  // 关闭电磁阀
}

// 物块抓取
void FSARM_Grab(float x1, float y1, float z1, float x2, float y2, float z2){
	FSARM_MoveLine(x1, y1, z1+4); // 运动到目标点上方
    SysTick_DelayMs(200);
    FSARM_MoveLine(x1, y1, z1); // 运动到目标点
    SysTick_DelayMs(200);
    FSARM_PumpOn(); // 开启气泵
    SysTick_DelayMs(200);
    FSARM_SetSpeed(50);
    FSARM_MoveLine(x1, y1, z1-1); // 气泵下压
    SysTick_DelayMs(200);
    FSARM_MoveLine(x1, y1, z1+4); // 抬起
    SysTick_DelayMs(200);
    FSARM_SetSpeed(100);
    FSARM_MoveLine(x2, y2, z2+4); // 运动到目标点上方
    SysTick_DelayMs(200);
    FSARM_SetSpeed(50);
    FSARM_MoveLine(x2, y2, z2); // 运动到目标点
    SysTick_DelayMs(200);
    FSARM_PumpOff(); // 关闭气泵
    SysTick_DelayMs(200);
    FSARM_MoveLine(x2, y2, z2+4); // 运动到目标点上方
    FSARM_SetSpeed(100);
}
