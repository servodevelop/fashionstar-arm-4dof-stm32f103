# FashionStar四自由度机械臂(STM32)使用说明



[toc]

## 开发环境配置

### 串口舵机调试工具

在FashionStar的官网，可以下载UART总线舵机的调试软件。

[FashionStar 舵机配套软件下载地址](https://fashionrobo.com/zh-hans/download/servo-software/)

<img src="image/PC软件下载地址.png" alt="PC软件下载地址" style="zoom: 67%;" />



软件包是一个`rar`后缀的压缩包，直接解压就可以，无需安装。



详情请参阅教程 *串口舵机使用入门 / 1.5舵机调试软件的使用*。　

**参阅教程给串口舵机分配ID**

<img src="image/上位机软件功能布局.png" style="zoom:70%;" />



### Keil5安装

Keil5是STM32的IDE，需要去Keil5的官方网站下载最新的IDE。

 [MDK-Arm官方下载地址](https://www.keil.com/download/product/)

<img src="image/MDK-Arm.png" style="zoom:80%;" />



安装好IDE之后，还需要安装STM32 F1系列的芯片包。

请自行搜索相关的视频/文本教程。



### STLinkV2驱动安装

STLink的驱动安装资料以及如何在Keil5中设置下载器选项，可以向销售STLinkV2的店家索要，并获取相关技术支持。

<img src="image/STLink.jpg" style="zoom: 20%;" />

注：如果使用其他下载器也是可以的，并不局限于STLinkV2，只是因为这种下载器比较便宜，占用的管脚也比较少。



### USB转TTL驱动下载与安装

串口舵机转接板使用的USB转TTL串口芯片是`CH340`，需要在Windows上安装CH340的驱动。

[CH340驱动下载地址](http://www.wch.cn/download/CH341SER_EXE.html)

驱动安装完成之后，检查驱动是否安装成功。

[检查CH340驱动是否安装成功](https://jingyan.baidu.com/article/00a07f3872a90982d028dcb9.html)

另外, 你需要安装你所使用的USB转TTL的驱动（例如CP2102）。

<img src="image/CP2102.jpg" style="zoom:20%;" />

 

### 串口调试助手

串口调试助手，推荐使用正点原子开发的XCOM V2.2。

[XCOM V2.2下载地址](https://www.amobbs.com/forum.php?mod=attachment&aid=NDQxNzc5fDE5NzMzYjQ1fDE1NzY2NTQ4NTN8MHw1NzAzODMz)（直接下载，无需注册）

[XCOM V2.2功能介绍](https://www.amobbs.com/thread-5703833-1-1.html)

<img src="image/180451f9ezabukfuya2yuy.png" style="zoom:50%;" />





## 工程结构

### 文件夹

打开文件夹 *src/FashionStarUartServo*

目录结构如下: 

* `Project` 

  Keil5的工程文件，点击 `FashionStarUartServo.uvprojx`   即可**通过Keil5, 双击打开此工程文件**。

* `User`

  主程序以及用户自定义库

  * `main.c` 用户主程序
  * 用户自定义的库文件，例如*串口舵机驱动库*

* `Listings` ：该目录是MDK生成信息输出目录，存放代码分布（.map和.lst）

* `Output` 该目录是MDK生成信息输出目录，存放目标文件(.o)、调试文件(.axf)、下载文件(.hex)、依赖文件（.d）等。

### 用户自定义库文件

![](./image/工程结构.png)



User文件夹 工程结构介绍

* `sys_tick` 

  管理系统时间。通过配置系统定时器中断，实现了延时以及倒计时的逻辑。

* `ring_buffer` 

  用C语言实现了环形缓冲队列。用于存放管理串口通信的字节流。同时还具备向缓冲队列读取/写入特定数据类型的数值的功能。

* `usart`

  串口通信的库。通过配置宏，可以方便的打开关闭STM32F103C8开发板上三个USART资源。

* `fashion_star_uart_servo` 

  封装了Fashion Star 串口舵机的通信协议，是串口舵机STM32F103版本的SDK。

* `arm4dof.c`

  四自由度舵机SDK

* `main.c`

  主程序,程序入口。测试例程都放在这个文件中, 测试的时候取消注释对应的部分.



## 接线说明



### STM32与STLinkV2的接线

通过STLinkV2给STM32下载固件。

*STM32与STLinkV2的接线图*

| STM32       | STLinkV2 |
| ----------- | -------- |
| SWIO / IO   | SWDIO    |
| SWCLK / CLK | SWCLK    |
| GND         | GND      |
| 3V3         | 3.3v     |

<img src="image/A1.jpg" style="zoom: 10%;" />



### 串口资源

![](image/硬件资源图.jpg)



STM32F103一共有三个串口资源，分别为UART1、UART2、UART3。在舵机云台主题课程（STM32版）里，约定三个串口的用途分别为如下所示：

* `UART1` 接串口舵机转接板，控制串口舵机
* `UART2` 接USB转TTL模块，用于日志输出
* `UART3` 空闲未使用

### STM32与串口总线舵机的接线

串口1和串口舵机转接板的TTL接口相连，用于控制串口舵机。

*STM32与串口舵机转接板接线图*

| STM32F103 GPIO    | 串口舵机转接板 |
| ----------------- | -------------- |
| PA_9   (UART1 Tx) | Rx             |
| PA_10 (UART1 Rx)  | Tx             |
| 5v                | 5v             |
| GND               | GND            |

<img src="image/1.png" alt="1" style="zoom:15%;" />

<img src="image/2.png" alt="2" style="zoom:10%;" />



### STM32与USB转TTL模块

STM32的串口2与USB转TTL模块模块相连，给PC发送日志信息。

*STM32与USB转TTL接线图*

| STM32F103 GPIO  | USB转TTL模块 |
| --------------- | ------------ |
| PA_2 (UART2 Tx) | Rx           |
| PA_3 (UART2 Rx) | Tx           |
| GND             | GND          |

USB转TTL模块的USB口与电脑的USB口相连。



<img src="image/A3.png" style="zoom: 70%;" />

<img src="image/A2.jpg" style="zoom: 10%;" />









## 机械臂初始化 InitArm



### 简介

机械臂初始化，让机械臂回归到机械零点的位置.

### API讲解

导入依赖的库文件

```c
#include "stm32f10x.h"
#include "usart.h" // 串口通信
#include "sys_tick.h" // 系统时间管理
#include "fashion_star_uart_servo.h" // 舵机驱动库
#include "arm4dof.h" // 四自由度舵机SDK
```

创建一个Usart结构体, `usart1` 用于跟串口总线舵机进行通信.

```c
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
```

硬件资源初始化

```c
SysTick_Init(); 			// 嘀嗒定时器初始化
Usart_Init(); 				// 串口初始化
FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
```



注: 如果想要修改机械臂机械零点的位置, 需要修改`arm4dof.h`里面的相关宏定义。

```c
// HOME 机械臂机械零点的定义
#define FSARM_HOME_X 12
#define FSARM_HOME_Y 0
#define FSARM_HOME_Z 10
```



### 例程源码-机械臂初始化测试

```c
/*
 * FashionStar四自由度机械臂-初始化测试
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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

```





## 阻尼模式与角度回读 DampingAndAngleRead

### 简介

设置舵机为阻尼模式, 并不断的查询舵机的原始角度。用手掰舵机, 查看舵机原始角度的变化.

### API讲解

#### 阻尼模式`FSARM_SetDamping`

**<函数原型>**

```c
// 设置关节为阻尼模式
void FSARM_SetDamping(uint16_t power);
```

**<输入参数>**

* `power` 舵机的功率

  功率的数值越大，再旋转舵机的时候阻力也就越大.

**<返回参数>**

* 无

**<演示示例>**

```c
uint16_t power = 500; 		// 阻尼模式下的功率
FSARM_SetDamping(power);    // 设置舵机为阻尼模式
```



#### 读取舵机角度`FSARM_QueryServoAngle`

**<函数原型>**

```c
// 批量读取舵机原始角度
void FSARM_QueryServoAngle(FSARM_JOINTS_STATE_T* servoAngles);
```

**<输入参数>**

* `servoAngles` 舵机角度的指针, 数据类型为`FSARM_JOINTS_STATE_T`

  ```c
  // 关节状态
  typedef struct{
      float theta1;
      float theta2;
      float theta3;
      float theta4;
  }FSARM_JOINTS_STATE_T;
  
  ```

**<返回参数>**

* 无

**<演示示例>**

```c
FSARM_JOINTS_STATE_T servoAngles; // 舵机的角度
FSARM_QueryServoAngle(&servoAngles); // 查询舵机当前的角度		
// 打印日志
printf("[INFO] Servo Angles: [%.1f, %.1f, %.1f, %.1f] \r\n", servoAngles.theta1, servoAngles.theta2, servoAngles.theta3, servoAngles.theta4);
```



#### 读取关节角度`FSARM_QueryJointAngle`

**<函数原型>**

```c
// 批量读取关节的角度
void FSARM_QueryJointAngle(FSARM_JOINTS_STATE_T* jointAngles);
```

**<输入参数>**

* `jointAngles`关节角度的指针

**<返回参数>**

* 无

**<演示示例>**

```c
FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
FSARM_QueryJointAngle(&jointAngles); // 查询当前关节的角度
printf("[INFO] Joint Angles: [%.1f, %.1f, %.1f, %.1f] \r\n", jointAngles.theta1, jointAngles.theta2, jointAngles.theta3, jointAngles.theta4);
```



### 例程源码-阻尼模式下的角度回读

```c
/*
 * FashionStar四自由度机械臂-阻尼模式下角度回读
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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

// 使用串口2作为日志输出的端口
// <接线说明>
// STM32F103 PA2(Tx) <----> USB转TTL Rx
// STM32F103 PA3(Rx) <----> USB转TTL Tx
// STM32F103 GND 	 <----> USB转TTL GND
// STM32F103 V5 	 <----> USB转TTL 5V (可选)
// <注意事项>
// 使用前确保已设置usart.h里面的USART2_ENABLE为1
Usart_DataTypeDef* loggingUsart = &usart2;

// 重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	while((loggingUsart->pUSARTx->SR&0X40)==0){}
	/* 发送一个字节数据到串口 */
	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
	/* 等待发送完毕 */
	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
	return (ch);
}



int main (void)
{
	
	SysTick_Init(); 			// 嘀嗒定时器初始化
	Usart_Init(); 				// 串口初始化
	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
	uint16_t power = 500; 		// 阻尼模式下的功率
	FSARM_SetDamping(power);    // 设置舵机为阻尼模式
	
	FSARM_JOINTS_STATE_T servoAngles; // 舵机的角度
	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度

	while (1){
		// 查询舵机当前的角度
		FSARM_QueryServoAngle(&servoAngles);		
		// 查询当前关节的角度
		FSARM_QueryJointAngle(&jointAngles);
		// 打印日志
		printf("[INFO] Servo Angles: [%.1f, %.1f, %.1f, %.1f] \r\n", servoAngles.theta1, servoAngles.theta2, servoAngles.theta3, servoAngles.theta4);
		printf("[INFO] Joint Angles: [%.1f, %.1f, %.1f, %.1f] \r\n", jointAngles.theta1, jointAngles.theta2, jointAngles.theta3, jointAngles.theta4);
		
		// 等待500ms
		SysTick_DelayMs(500);
	}
}
```



**<输出日志>***

```
[INFO] Servo Angles: [56.2, 24.3, 11.2, -52.1] 
[INFO] Joint Angles: [-55.6, -65.4, 31.5, 52.0] 
[INFO] Servo Angles: [55.9, 24.6, 32.5, -52.0] 
[INFO] Joint Angles: [-55.3, -65.2, 9.6, 51.9] 
[INFO] Servo Angles: [56.0, 24.3, 55.1, -52.0] 
[INFO] Joint Angles: [-55.4, -65.4, -11.6, 51.9] 
[INFO] Servo Angles: [56.3, 28.8, 54.9, -43.6] 
[INFO] Joint Angles: [-55.7, -60.4, -11.5, 42.6] 
[INFO] Servo Angles: [56.2, 67.1, 55.1, 0.1] 
[INFO] Joint Angles: [-55.6, -22.4, -11.7, 0.3] 
[INFO] Servo Angles: [56.3, 30.7, 55.3, 0.1] 
[INFO] Joint Angles: [-55.7, -59.5, -11.9, 0.3] 
[INFO] Servo Angles: [55.9, -9.0, 32.3, -11.3] 
[INFO] Joint Angles: [-55.3, -99.5, 12.1, 12.4] 
[INFO] Servo Angles: [56.2, -42.2, -34.7, -36.2] 
[INFO] Joint Angles: [-55.6, -132.2, 76.8, 36.3] 
[INFO] Servo Angles: [56.2, -42.1, -41.3, -36.2] 
[INFO] Joint Angles: [-55.7, -132.1, 83.1, 36.3] 
[INFO] Servo Angles: [56.2, -42.1, -41.8, -36.2] 
[INFO] Joint Angles: [-55.7, -132.1, 83.5, 36.3] 
```



## 机械臂标定

### 简介

通过[例程源码-阻尼模式下的角度回读](#例程源码-阻尼模式下的角度回读)，采集特定关节在特定角度下的原始舵机角度, 并同步修改`arm4dof.h` 里面的相关配置.

### 机械臂舵机ID分配

机械臂舵机ID分配如下: 

![](./image/舵机ID分配.png)

### 世界坐标系

机械臂的世界坐标系/机械臂基坐标系定义如下:

![](./image/世界坐标系.png)



世界坐标系的原点定义在#1号舵机的转轴中心处, 机械臂正前方为X轴, 上方为Z轴, Y轴指向纸面朝外.

### 机械臂关节与关节坐标系定义

关节与关节坐标系定义如下图所示:

![](./image/机械臂关节定义.png)



![](./image/关节1的弧度.png)



![](./image/关节2的弧度.png)



![](./image/关节3的弧度.png)



![](./image/关节4的弧度.png)



### 舵机原始角度数据采集

运行[例程源码-阻尼模式下的角度回读](#例程源码-阻尼模式下的角度回读), 在阻尼模式下，将机械臂的关节掰到指定的位置, 并读取此时**舵机的原始角度**, 记录下来.

将采集的数据同步修改到`arm4dof.h`里面.

```c
// 舵机标定参数
#define FSARM_JOINT1_P90 -92.6  //关节1为90°时的舵机原始角度
#define FSARM_JOINT1_N90 91.4   //关节1为-90°时的舵机原始角度
#define FSARM_JOINT2_P0 89.3    //关节2为0°时的舵机原始角度
#define FSARM_JOINT2_N90 -0.2   //关节2为-90°时的舵机原始角度
#define FSARM_JOINT3_P90 -48.4  //关节3为90°时的舵机原始角度
#define FSARM_JOINT3_N90 134.8  //关节3为-90°时的舵机原始角度
#define FSARM_JOINT4_P90 -90.5  //关节4为90°时的舵机原始角度
#define FSARM_JOINT4_N90 91.4   //关节4为-90°时的舵机原始角度
```



## 设置关节角度 SetJointAngle

### 简介

控制机械臂的关节角度, 等待机械臂运动到特定的位置之后, 再执行后续的动作.

### API讲解



#### 设置关节角度`FSARM_SetJointAngle`

**<函数原型>**

```c
// 批量设置关节的角度
void FSARM_SetJointAngle(FSARM_JOINTS_STATE_T jointAngles);
```

**<输入参数>**

* `jointAngles` : 关节角度

  数据类型为`FSARM_JOINTS_STATE_T`结构体

**<返回参数>**

* 无

**<演示示例>**

```c
FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
// 设置关节角度
jointAngles.theta1 = 0.0;
jointAngles.theta2 = -60.0;
jointAngles.theta3 = 105.0;
jointAngles.theta4 = -45.0; 
// 设置关节角度
FSARM_SetJointAngle(jointAngles);
```



#### 设置关节角度(指定周期)`FSARM_SetJointAngle2`

**<函数原型>**

```c
// 批量设置关节的角度(带统一的周期)
void FSARM_SetJointAngle2(FSARM_JOINTS_STATE_T jointAngles, uint16_t interval);
```



**<输入参数>**

* `jointAngles` 关节角度
* `interval` 周期, 单位为ms

**<返回参数>**

* 无

**<演示示例>**

```c
FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
jointAngles.theta1 = 90.0; // 修改关节角度
jointAngles.theta2 = -90.0;
jointAngles.theta3 = 105.0;
jointAngles.theta4 = -15.0;

uint16_t interval = 1000; // 周期为1000ms
FSARM_SetJointAngle2(jointAngles, interval);
```



#### 等待关节旋转到目标位置`FSARM_WaitAll`

需要注意的是`FSARM_SetJointAngle`与函数`FSARM_SetJointAngle2` 并不是阻塞式的, 并不会等待舵机旋转到目标角度才会执行到下一条指令, 所以如果你想让机械臂执行完一个动作再去执行下一个动作的话, 就需要使用 `FSARM_WaitAll` 函数



**<函数原型>**

```c
// 等待所有的关节旋转到目标角度
void FSARM_WaitAll(void);
```

**<输入参数>**

* 无

**<返回参数>**

* 无

**<演示示例>**

```c
FSARM_WaitAll();
```



#### 设置关节角度(过程中末端与工作台平行) `FSARM_SetJointAngleParallel`



**<函数原型>**

```c
// 批量设置关节的角度(末端平行)
void FSARM_SetJointAngleParallel(FSARM_JOINTS_STATE_T jointAngles);
```

**<输入参数>**

* `jointAngle `关节角度

**<返回参数>**

* 无

**<演示示例>**

```c
FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
jointAngles.theta1 = 90.0; // 修改关节角度
jointAngles.theta2 = -40.0;
jointAngles.theta3 = 105.0;
jointAngles.theta4 = -65.0;
FSARM_SetJointAngleParallel(jointAngles);
```



### 例程源码-设置关节角度

```c
/*
 * FashionStar四自由度机械臂-设置关节角度
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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
			
	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
	
	while (1){
		// 动作A
		// 设置关节角度
		jointAngles.theta1 = 0.0;
		jointAngles.theta2 = -60.0;
		jointAngles.theta3 = 105.0;
		jointAngles.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
									// theta4 = -(theta2 + theta3)
		// 设置关节角度
		FSARM_SetJointAngle(jointAngles);
		FSARM_WaitAll();
		SysTick_DelayMs(1000); // 等待1s
		
		// 动作B
		jointAngles.theta1 = 90.0; // 修改关节角度
		jointAngles.theta2 = -90.0;
		jointAngles.theta3 = 105.0;
		jointAngles.theta4 = -15.0;
		
		uint16_t interval = 1000; // 周期为1000ms
		FSARM_SetJointAngle2(jointAngles, interval);
		FSARM_WaitAll();
		SysTick_DelayMs(1000); // 等待1s
		
		// 动作C
		jointAngles.theta1 = 90.0; // 修改关节角度
		jointAngles.theta2 = -40.0;
		jointAngles.theta3 = 105.0;
		jointAngles.theta4 = -65.0;
		FSARM_SetJointAngleParallel(jointAngles);
		SysTick_DelayMs(1000); // 等待1s
		
		// 动作D
		jointAngles.theta1 = 90.0; // 修改关节角度
		jointAngles.theta2 = -90.0;
		jointAngles.theta3 = 105.0;
		jointAngles.theta4 = -15.0;
		FSARM_SetJointAngleParallel(jointAngles);
		SysTick_DelayMs(1000); // 等待1s
	}
}

```





## 机械臂正向运动学 ForwardKinematics



### 简介

正向运动学就是在已知关节角度的前提下求解机械臂末端在机械臂基坐标系下的位置/位姿.

### API讲解

#### 机械臂正向运动学`FSARM_ForwardKinmatics`

**<函数原型>**

```c
// 机械臂的正向运动学
void FSARM_ForwardKinmatics(FSARM_JOINTS_STATE_T jointAngles, FSARM_POINT3D_T* toolPosi);
```

**<输入参数>**

* `jointAngles` 关节角度

* `toolPosi` 末端坐标的指针, 数据类型为`FSARM_POINT3D_T`

  ```c
  // 笛卡尔空间下的点
  typedef struct{
      float x;
      float y;
      float z;
  }FSARM_POINT3D_T;
  
  ```

**<返回参数>**

* 无

**演示示例**

```c
FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
FSARM_POINT3D_T  toolPosi; 		  // 工具坐标系的坐标

// 设置关节角度
jointAngles.theta1 = 0.0;
jointAngles.theta2 = -60.0;
jointAngles.theta3 = 105.0;
jointAngles.theta4 = -45.0;

// 测试正向运动学
FSARM_ForwardKinmatics(jointAngles, &toolPosi);
// 打印当前末端的位置信息
printf("Forward Kinematics, ToolPosi = (%.1f, %.1f, %.1f)", toolPosi.x, toolPosi.y, toolPosi.z);
```



### 例程源码-机械臂正向运动学测试

```c
/*
 * FashionStar四自由度机械臂-机械臂正向运动学测试
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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
// 使用串口2作为日志输出的端口
// <接线说明>
// STM32F103 PA2(Tx) <----> USB转TTL Rx
// STM32F103 PA3(Rx) <----> USB转TTL Tx
// STM32F103 GND 	 <----> USB转TTL GND
// STM32F103 V5 	 <----> USB转TTL 5V (可选)
// <注意事项>
// 使用前确保已设置usart.h里面的USART2_ENABLE为1
Usart_DataTypeDef* loggingUsart = &usart2;

// 重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	while((loggingUsart->pUSARTx->SR&0X40)==0){}
	/* 发送一个字节数据到串口 */
	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
	/* 等待发送完毕 */
	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
	return (ch);
}


int main (void)
{
	
	SysTick_Init(); 			// 嘀嗒定时器初始化
	Usart_Init(); 				// 串口初始化
	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
			
	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
	FSARM_POINT3D_T  toolPosi; 		  // 工具坐标系的坐标
	// 设置关节角度
	jointAngles.theta1 = 0.0;
	jointAngles.theta2 = -60.0;
	jointAngles.theta3 = 105.0;
	jointAngles.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
									// theta4 = -(theta2 + theta3)
	// 设置关节角度
	FSARM_SetJointAngle(jointAngles);
	FSARM_WaitAll();	
	SysTick_DelayMs(1000); // 等待1s
	
	// 测试正向运动学
	FSARM_ForwardKinmatics(jointAngles, &toolPosi);
	// 打印当前末端的位置信息
	printf("Forward Kinematics, ToolPosi = (%.1f, %.1f, %.1f)", toolPosi.x, toolPosi.y, toolPosi.z);
	
	while (1){
	}
}
```



*<日志输出>*

```
Forward Kinematics, ToolPosi = (15.9, 0.0, -2.4)
```



## 机械臂逆向运动学 InverseKinematics

### 简介

机械臂逆向运动学是指,给定工具在世界坐标系下的位姿, 求解此时关节的角度.

### API讲解

#### 机械臂逆向运动学`FSARM_InverseKinematics`

**<函数原型>**

```c
// 机械臂逆向运动学
FSARM_STATUS  FSARM_InverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* jointAngles);
```

**<输入参数>**

* `toolPosi`工具坐标
* `jointAngles` 关节角度的指针

**<返回参数>**

* 状态码`FSARM_STATUS`

  通过状态码可以知道逆向运动学求解是否成功,  如果失败则可以查看到底是哪个关节超出范围. 状态码的定义如下:

  ```c
  // 状态码
  #define FSARM_STATUS uint8_t
  #define FSARM_STATUS_SUCCESS 0 // 成功
  #define FSARM_STATUS_FAIL 1 // 失败
  #define FSARM_STATUS_JOINT1_OUTRANGE 2 // 关节1超出范围
  #define FSARM_STATUS_JOINT2_OUTRANGE 3 // 关节2超出范围
  #define FSARM_STATUS_JOINT3_OUTRANGE 4 // 关节3超出范围
  #define FSARM_STATUS_JOINT4_OUTRANGE 5 // 关节4超出范围
  #define FSARM_STATUS_TOOLPOSI_TOO_FAR 6 // 工具坐标目标点距离机械臂太遥远
  ```

**<演示示例>**

```c
FSARM_POINT3D_T toolPosi; // 末端位置
FSARM_JOINTS_STATE_T jointAngles; // 关节角度-逆向运动学输出的结果
// 设置末端的坐标
toolPosi.x = 10.0;
toolPosi.y = 0.0;
toolPosi.z = -3.0;
// 机械臂逆向运动学
FSARM_STATUS code = FSARM_InverseKinematics(toolPosi, &jointAngles);
```

### 例程源码-机械臂逆向运动学测试

```c
/*
 * FashionStar四自由度机械臂-机械臂逆向运动学测试
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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
// 使用串口2作为日志输出的端口
// <接线说明>
// STM32F103 PA2(Tx) <----> USB转TTL Rx
// STM32F103 PA3(Rx) <----> USB转TTL Tx
// STM32F103 GND 	 <----> USB转TTL GND
// STM32F103 V5 	 <----> USB转TTL 5V (可选)
// <注意事项>
// 使用前确保已设置usart.h里面的USART2_ENABLE为1
Usart_DataTypeDef* loggingUsart = &usart2;

// 重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	while((loggingUsart->pUSARTx->SR&0X40)==0){}
	/* 发送一个字节数据到串口 */
	USART_SendData(loggingUsart->pUSARTx, (uint8_t) ch);
	/* 等待发送完毕 */
	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) != SET);		
	return (ch);
}


int main (void)
{
	SysTick_Init(); 			// 嘀嗒定时器初始化
	Usart_Init(); 				// 串口初始化
	FSARM_Init(servoUsart); 	// 机械臂初始化-> 回归到机械零点
			
	FSARM_JOINTS_STATE_T jointAngles; // 关节的角度
	FSARM_POINT3D_T  toolPosi; 		  // 工具坐标系的坐标
	// 设置关节角度
	jointAngles.theta1 = 0.0;
	jointAngles.theta2 = -60.0;
	jointAngles.theta3 = 105.0;
	jointAngles.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
									// theta4 = -(theta2 + theta3)
	// 设置关节角度
	FSARM_SetJointAngle(jointAngles);
	FSARM_WaitAll();	
	SysTick_DelayMs(1000); // 等待1s
	
	// 机械臂正向运动学
	FSARM_ForwardKinmatics(jointAngles, &toolPosi);
	// 打印当前末端的位置信息
	printf("Forward Kinematics, ToolPosi = (%.1f, %.1f, %.1f)\r\n", toolPosi.x, toolPosi.y, toolPosi.z);
	
	// 关节角度2(存储逆向运动学的结果)
	FSARM_JOINTS_STATE_T jointAngles2;
	// 机械臂逆向运动
	FSARM_STATUS status = FSARM_InverseKinematics(toolPosi, &jointAngles2);
	// 打印逆向运动学的结果
	printf("Inverse Kinematics, result code = %d\r\n", status);
	printf("-> Joint Angles = (%.1f, %.1f, %.1f, %.1f) \r\n", jointAngles2.theta1, jointAngles2.theta2, jointAngles2.theta3, jointAngles2.theta4);
	
	while (1){
	}
}
```



*<日志输出>*

```
Forward Kinematics, ToolPosi = (15.9, 0.0, -2.4)
Inverse Kinematics, result code = 0
-> Joint Angles = (0.0, -60.0, 105.0, -45.0) 
```



## 自由轨迹 MoveP2P

### 简介

控制机械臂的末端移动到特定的位置, 中间不遵循特定的轨迹, 但是机械臂的末端保持与工作台平行。

### API讲解

#### 自由轨迹`FSARM_MoveP2P`

**<函数原型>**

```c
// 点控(自由轨迹)
FSARM_STATUS FSARM_MoveP2P(float x, float y, float z);
```

**<输入参数>**

* `x` 末端的x坐标
* `y` 末端的y坐标
* `z` 末端的z坐标

**<返回参数>**

* 状态码`FSARM_STATUS`

  与[机械臂逆向运动学`FSARM_InverseKinematics`](#机械臂逆向运动学`FSARM_InverseKinematics`) 中的定义相同.

**<示例代码>**

```c
FSARM_MoveP2P(8.0, 8, -4.0);
```



### 例程源码-点控MoveP2P(自由轨迹)



```c
/*
 * FashionStar四自由度机械臂-点控MoveP2P(自由轨迹)
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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
		FSARM_MoveP2P(10.0, 0, 5.0);
		SysTick_DelayMs(1000); // 等待1s
		
		FSARM_MoveP2P(8.0, 8, -4.0);
		SysTick_DelayMs(1000); // 等待1s
		
		FSARM_MoveP2P(6.0, -6, 5.0);
		SysTick_DelayMs(1000); // 等待1s
	}
}
```



## 直线插补MoveLine

### 简介

机械臂末端直线插补.

### API讲解

#### 直线插补`FSARM_MoveLine`

**<函数原型>**

```c
// 直线插补
FSARM_STATUS FSARM_MoveLine(float x, float y, float z);
```

**<输入参数>**

* `x` 末端的x坐标
* `y` 末端的y坐标
* `z` 末端的z坐标

**<返回参数>**

* 状态码`FSARM_STATUS`

  与[机械臂逆向运动学`FSARM_InverseKinematics`](#机械臂逆向运动学`FSARM_InverseKinematics`) 中的定义相同.

**<示例代码>**

```c
FSARM_MoveLine(9.0, 9.0, 0.0);
```



### 例程源码-直线插补

```c
/*
 * FashionStar四自由度机械臂-直线插补MoveLine
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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
		FSARM_MoveLine(12.0, 0, 0.0);
		SysTick_DelayMs(1000); // 等待1s
		
		FSARM_MoveLine(9.0, -13, 0.0);
		SysTick_DelayMs(1000); // 等待1s
		
		FSARM_MoveLine(9.0, 9.0, 0.0);
		SysTick_DelayMs(1000); // 等待1s
	}
}
```



## 气泵控制 PumpControl

### 简介

气泵模块的开启与关闭

### API讲解

#### 气泵开启`FSARM_PumpOn`

**<函数原型>**

```c
// 气泵开启
void FSARM_PumpOn(void);
```

**<输入参数>**

* 无

**<返回参数>**

* 无

**<示例代码>**

```c
FSARM_PumpOn();			// 气泵开启
```





#### 气泵关闭`FSARM_PumpOff`

**<函数原型>**

```c
// 气泵关闭
void FSARM_PumpOff(void);
```

**<输入参数>**

* 无

**<返回参数>**

* 无

**<示例代码>**

```c
FSARM_PumpOff();			// 气泵关闭
```



### 例程源码-气泵控制

```c
/*
 * FashionStar四自由度机械臂-气泵控制
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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
		FSARM_PumpOn();			// 气泵开启
		SysTick_DelayMs(1000); 	// 等待1s
		
		FSARM_PumpOff();		// 气泵关闭
		SysTick_DelayMs(1000); 	// 等待1s
	}
}
```





## 物块抓取 CubicGrab

### 简介

通过气泵吸头，将物块从一个位置搬运到另外一个位置.

### API讲解

#### 物块抓取`FSARM_Grab`

**<函数原型>**

```c
// 物块抓取
void FSARM_Grab(float x1, float y1, float z1, float x2, float y2, float z2);
```

**<输入参数>**

* `x1` 物块起始位置的x坐标
* `y1` 物块起始位置的y坐标
* `z1` 物块起始位置的z坐标
* `x2` 物块目标位置的x坐标
* `y2` 物块目标位置的y坐标
* `z2` 物块目标位置的z坐标

**<返回参数>**

* 无

**示例代码**

```c
FSARM_Grab(22, 1, -3, 9, 11, -3);
```



### 例程源码-物块抓取

```c
/*
 * FashionStar四自由度机械臂-物块抓取
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
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
		FSARM_Grab(9, -13, -5, 9, 9, -5);
		SysTick_DelayMs(1000); 	// 等待1s
		
		FSARM_Grab(15, 0, -5, 9, 9, -5);
		SysTick_DelayMs(1000); 	// 等待1s
		
		FSARM_Grab(22, 1, -3, 9, 11, -3);
		SysTick_DelayMs(1000); 	// 等待1s
		
		FSARM_Grab(1, 22, -3, 11, 9, -3);
		SysTick_DelayMs(1000); 	// 等待1s
	}
}

```

