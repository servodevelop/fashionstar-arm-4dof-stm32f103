#ifndef __COMMON_H
#define __COMMON_H

#define bool uint8_t
#define true 1
#define false 0

// �����ֵ
#define max(x, y) (((x) > (y)) ? (x):(y))

#define pi 3.1415926

// �Ƕ�ת����
#define radians(theta) ((theta)/180.0 * pi)

// ����ת�Ƕ�
#define degrees(theta) ((theta)/pi*180.0)

#endif
