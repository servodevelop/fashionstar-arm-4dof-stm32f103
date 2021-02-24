/*
 * FashionStar四自由度机械臂-初始化测试
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/19
 */
 
#include "stm32f10x.h"
#include "usart.h"
#include "sys_tick.h"
#include "fashion_star_uart_servo.h"
#include "arm4dof.h"

// 使用串口1作为舵机控制的端口
// <接线说明>
// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
// STM32F103 GND 	  <----> 串口舵机转接板 GND
// STM32F103 V5 	  <----> 串口舵机转接板 5V
// <注意事项>
// 使用前确保已设置usart.h里面的USART1_ENABLE为1
// 设置完成之后, 将下行取消注释
Usart_DataTypeDef* servoUsart = &usart1; 

int main (void)
{
	
	SysTick_Init(); 			// 嘀嗒定时器初始化
	Usart_Init(); 				// 串口初始化
	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
	while (1){
	}
}


///*
// * FashionStar四自由度机械臂-阻尼模式下角度回读
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//// 使用串口2作为日志输出的端口
//// <接线说明>
//// STM32F103 PA2(Tx) <----> USB转TTL Rx
//// STM32F103 PA3(Rx) <----> USB转TTL Tx
//// STM32F103 GND 	 <----> USB转TTL GND
//// STM32F103 V5 	 <----> USB转TTL 5V (可选)
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART2_ENABLE为1
//Usart_DataTypeDef* loggingUsart = &usart2;

//// 重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//	while((loggingUsart->pUSARTx->SR&0X40)==0){}
//	/* 发送一个字节数据到串口 */
//	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
//	/* 等待发送完毕 */
//	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
//	return (ch);
//}



//int main (void)
//{
//	
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//	uint16_t power = 500; 		// 阻尼模式下的功率
//	FSARM_SetDamping(power);    // 设置舵机为阻尼模式
//	
//	FSARM_JOINTS_STATE_T servoAngles; // 舵机的角度
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度

//	while (1){
//		// 查询舵机当前的角度
//		FSARM_QueryServoAngle(&servoAngles);		
//		// 查询当前关节的角度
//		FSARM_QueryJointAngle(&jointAngles);
//		// 打印日志
//		printf("[INFO] Servo Angles: [%.1f, %.1f, %.1f, %.1f] \r\n", servoAngles.theta1, servoAngles.theta2, servoAngles.theta3, servoAngles.theta4);
//		printf("[INFO] Joint Angles: [%.1f, %.1f, %.1f, %.1f] \r\n", jointAngles.theta1, jointAngles.theta2, jointAngles.theta3, jointAngles.theta4);
//		
//		// 等待500ms
//		SysTick_DelayMs(500);
//	}
//}


///*
// * FashionStar四自由度机械臂-设置关节角度
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 


//int main (void)
//{
//	
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
//	
//	while (1){
//		// 动作A
//		// 设置关节角度
//		jointAngles.theta1 = 0.0;
//		jointAngles.theta2 = -60.0;
//		jointAngles.theta3 = 105.0;
//		jointAngles.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
//									// theta4 = -(theta2 + theta3)
//		// 设置关节角度
//		FSARM_SetJointAngle(jointAngles);
//		FSARM_WaitAll();
//		SysTick_DelayMs(1000); // 等待1s
//		
//		// 动作B
//		jointAngles.theta1 = 90.0; // 修改关节角度
//		jointAngles.theta2 = -90.0;
//		jointAngles.theta3 = 105.0;
//		jointAngles.theta4 = -15.0;
//		
//		uint16_t interval = 1000; // 周期为2000ms
//		FSARM_SetJointAngle2(jointAngles, interval);
//		FSARM_WaitAll();
//		SysTick_DelayMs(1000); // 等待1s
//		
//		// 动作C
//		jointAngles.theta1 = 90.0; // 修改关节角度
//		jointAngles.theta2 = -40.0;
//		jointAngles.theta3 = 105.0;
//		jointAngles.theta4 = -65.0;
//		FSARM_SetJointAngleParallel(jointAngles);
//		SysTick_DelayMs(1000); // 等待1s
//		
//		// 动作D
//		jointAngles.theta1 = 90.0; // 修改关节角度
//		jointAngles.theta2 = -90.0;
//		jointAngles.theta3 = 105.0;
//		jointAngles.theta4 = -15.0;
//		FSARM_SetJointAngleParallel(jointAngles);
//		SysTick_DelayMs(1000); // 等待1s
//	}
//}


///*
// * FashionStar四自由度机械臂-机械臂正向运动学测试
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 
//// 使用串口2作为日志输出的端口
//// <接线说明>
//// STM32F103 PA2(Tx) <----> USB转TTL Rx
//// STM32F103 PA3(Rx) <----> USB转TTL Tx
//// STM32F103 GND 	 <----> USB转TTL GND
//// STM32F103 V5 	 <----> USB转TTL 5V (可选)
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART2_ENABLE为1
//Usart_DataTypeDef* loggingUsart = &usart2;

//// 重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//	while((loggingUsart->pUSARTx->SR&0X40)==0){}
//	/* 发送一个字节数据到串口 */
//	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
//	/* 等待发送完毕 */
//	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
//	return (ch);
//}


//int main (void)
//{
//	
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
//	FSARM_POINT3D_T  toolPosi; 		  // 工具坐标系的坐标
//	// 设置关节角度
//	jointAngles.theta1 = 0.0;
//	jointAngles.theta2 = -60.0;
//	jointAngles.theta3 = 105.0;
//	jointAngles.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
//									// theta4 = -(theta2 + theta3)
//	// 设置关节角度
//	FSARM_SetJointAngle(jointAngles);
//	FSARM_WaitAll();	
//	SysTick_DelayMs(1000); // 等待1s
//	
//	// 测试正向运动学
//	FSARM_ForwardKinmatics(jointAngles, &toolPosi);
//	// 打印当前末端的位置信息
//	printf("Forward Kinematics, ToolPosi = (%.1f, %.1f, %.1f)", toolPosi.x, toolPosi.y, toolPosi.z);
//	
//	while (1){
//	}
//}


///*
// * FashionStar四自由度机械臂-机械臂逆向运动学测试
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 
//// 使用串口2作为日志输出的端口
//// <接线说明>
//// STM32F103 PA2(Tx) <----> USB转TTL Rx
//// STM32F103 PA3(Rx) <----> USB转TTL Tx
//// STM32F103 GND 	 <----> USB转TTL GND
//// STM32F103 V5 	 <----> USB转TTL 5V (可选)
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART2_ENABLE为1
//Usart_DataTypeDef* loggingUsart = &usart2;

//// 重定向c库函数printf到串口，重定向后可使用printf函数
//int fputc(int ch, FILE *f)
//{
//	while((loggingUsart->pUSARTx->SR&0X40)==0){}
//	/* 发送一个字节数据到串口 */
//	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
//	/* 等待发送完毕 */
//	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
//	return (ch);
//}


//int main (void)
//{
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
//	FSARM_POINT3D_T  toolPosi; 		  // 工具坐标系的坐标
//	// 设置关节角度
//	jointAngles.theta1 = 0.0;
//	jointAngles.theta2 = -60.0;
//	jointAngles.theta3 = 105.0;
//	jointAngles.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
//									// theta4 = -(theta2 + theta3)
//	// 设置关节角度
//	FSARM_SetJointAngle(jointAngles);
//	FSARM_WaitAll();	
//	SysTick_DelayMs(1000); // 等待1s
//	
//	// 机械臂正向运动学
//	FSARM_ForwardKinmatics(jointAngles, &toolPosi);
//	// 打印当前末端的位置信息
//	printf("Forward Kinematics, ToolPosi = (%.1f, %.1f, %.1f)\r\n", toolPosi.x, toolPosi.y, toolPosi.z);
//	
//	// 关节角度2(存储逆向运动学的结果)
//	FSARM_JOINTS_STATE_T jointAngles2;
//	// 机械臂逆向运动
//	FSARM_STATUS status = FSARM_InverseKinematics(toolPosi, &jointAngles2);
//	// 打印逆向运动学的结果
//	printf("Inverse Kinematics, result code = %d\r\n", status);
//	printf("-> Joint Angles = (%.1f, %.1f, %.1f, %.1f) \r\n", jointAngles2.theta1, jointAngles2.theta2, jointAngles2.theta3, jointAngles2.theta4);
//	
//	while (1){
//	}
//}


///*
// * FashionStar四自由度机械臂-点控MoveP2P(自由轨迹)
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//int main (void)
//{
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	
//	
//	while (1){
//		FSARM_MoveP2P(10.0, 0, 5.0);
//		SysTick_DelayMs(1000); // 等待1s
//		
//		FSARM_MoveP2P(8.0, 8, -4.0);
//		SysTick_DelayMs(1000); // 等待1s
//		
//		FSARM_MoveP2P(6.0, -6, 5.0);
//		SysTick_DelayMs(1000); // 等待1s
//	}
//}

///*
// * FashionStar四自由度机械臂-直线插补MoveLine
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//int main (void)
//{
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	
//	
//	while (1){
//		FSARM_MoveLine(12.0, 0, 0.0);
//		SysTick_DelayMs(1000); // 等待1s
//		
//		FSARM_MoveLine(9.0, -13, 0.0);
//		SysTick_DelayMs(1000); // 等待1s
//		
//		FSARM_MoveLine(9.0, 9.0, 0.0);
//		SysTick_DelayMs(1000); // 等待1s
//	}
//}


///*
// * FashionStar四自由度机械臂-气泵控制
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//int main (void)
//{
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	while (1){
//		FSARM_PumpOn();			// 气泵开启
//		SysTick_DelayMs(1000); 	// 等待1s
//		
//		FSARM_PumpOff();		// 气泵关闭
//		SysTick_DelayMs(1000); 	// 等待1s
//	}
//}

///*
// * FashionStar四自由度机械臂-物块抓取
// * --------------------------
// * 作者: 阿凯|Kyle
// * 邮箱: kyle.xing@fashionstar.com.hk
// * 更新时间: 2020/05/19
// */
// 
//#include "stm32f10x.h"
//#include "usart.h"
//#include "sys_tick.h"
//#include "fashion_star_uart_servo.h"
//#include "arm4dof.h"

//// 使用串口1作为舵机控制的端口
//// <接线说明>
//// STM32F103 PA9(Tx)  <----> 串口舵机转接板 Rx
//// STM32F103 PA10(Rx) <----> 串口舵机转接板 Tx
//// STM32F103 GND 	  <----> 串口舵机转接板 GND
//// STM32F103 V5 	  <----> 串口舵机转接板 5V
//// <注意事项>
//// 使用前确保已设置usart.h里面的USART1_ENABLE为1
//// 设置完成之后, 将下行取消注释
//Usart_DataTypeDef* servoUsart = &usart1; 

//int main (void)
//{
//	SysTick_Init(); 			// 嘀嗒定时器初始化
//	Usart_Init(); 				// 串口初始化
//	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
//			
//	while (1){
//		FSARM_Grab(9, -13, -5, 9, 9, -5);
//		SysTick_DelayMs(1000); 	// 等待1s
//		
//		
//		FSARM_Grab(15, 0, -5, 9, 9, -5);
//		SysTick_DelayMs(1000); 	// 等待1s
//		
//		FSARM_Grab(22, 1, -3, 9, 11, -3);
//		SysTick_DelayMs(1000); 	// 等待1s
//		
//		FSARM_Grab(1, 22, -3, 11, 9, -3);
//		SysTick_DelayMs(1000); 	// 等待1s
//	}
//}


