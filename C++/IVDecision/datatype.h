/*******************************************************

		本文件用于定义数据类型、字长、变量取值范围

********************************************************/
#ifndef _DATATYPE
#define _DATATYPE

//声明字长
typedef unsigned long	DWORD;	//无符号双字
typedef unsigned short 	WORD;	//无符号单字;
typedef long			SDWORD;	//有符号型双字
typedef short			SWORD;	//有符号型单字

//Q15表示的常数
#define Q15_SQRT_2   46341     //(2)^(1/2)的Q15表示,误差比例为-1.1e-6.
#define Q15_1        32768     //1的Q15表示  
#define Q15_1_APP    32767     //1的Q15表示，为避免int型数据溢出，近似为该值  
#define Q15_1DIV6    5461      //1/6,误差比例为-6.1e-5.
#define Q15_1DIV10   3276      //1/10,误差比例为-2.4e-4.
#define Q15_1DIV100  327       //1/100,误差比例为-2.1e-3.
#define Q15_1DIV120  273       //1/120,误差比例-2.4e-4.

//返回消息
#define MSG_SUCCESS         0   //成功完成
#define MSG_ERROR           1   //出现故障


#define CTRLREG_INTFRE		    100			//控制环的中断频率，单位Hz
#define CTRLREG_INTPRD          10          //控制环的中断周期，单位ms

/************************************************************
				系统常量
************************************************************/

//系统调试数组长度
#define DEBUG_ARRAY_SIZE   18000  //最长可保存3分钟的数据


/************************************************************
				油门控制常量与参数最大值范围
************************************************************/
#define GASREG_SPEED_MAX           32       //车速最大值,m/s
#define GASREG_SPEED_MIN            0       //车速最小值,m/s
#define GASREG_PIGAIN_MAX       10000       //PI控制器的P增益与I增益的最大值
#define GASREG_UOUT_SIZE        12          //PI控制器输出的位数
#define GASREG_UOUT_PMAX		 4.99          //PI控制器输出的正向最大值限制
#define GASREG_UOUT_NMAX		 0          //PI控制器输出的负向最大值限制
#define GASREG_INTFRE           10          //油门控制环的频率，单位Hz
#define GASREG_INTPRD          100          //油门控制环的周期，单位ms
#define GASREG_STALL_COUNT_MAX    50        //允许的堵转计数最大值，单位为控制周期

/************************************************************
				方向控制常量与参数最大值范围
************************************************************/
#define STEERREG_ANGLE_RATIO            54                        // 转向减速比，即电机角度/方向盘角度
#define STEERREG_ANGLE_MAX       25000//(1.5*360*STEERREG_ANGLE_RATIO)   //转角最大值，度
#define STEERREG_ANGLE_MIN      -25000//(-1.5*360*STEERREG_ANGLE_RATIO)  //转角最小值，度
#define STEERREG_PIGAIN_MAX    10000          //PI控制器的P增益与I增益的最大值为2
#define STEERREG_UOUT_PMAX      4.99          //位置环PI控制器输出的正向最大值限制
#define STEERREG_UOUT_NMAX     -4.99          //位置环PI控制器输出的负向最大值限制
#define STEERREG_INTFRE       (CTRLREG_INTFRE)  //转向位置控制环的频率，单位Hz
#define STEERREG_INTPRD       (CTRLREG_INTPRD)  //转向位置控制环的周期，单位ms
#define STEERREG_VEL_MAX      ((3000.0*360.0)/(STEERREG_INTFRE*60))
#define STEERREG_VEL_MIN             4     //系统最小速度，单位度/控制周期
#define STEERREG_STALL_COUNT_MAX    20      //允许的堵转计数最大值，堵转延时为200ms；
#define STEERREG_INTP_ENDREGDIS        20    //预留的终点匀速距离，单位为度
#define STEERREG_ERROR_MAX             200    //跟踪误差最大值，单位为度
#define STEERREG_ACC_VELERR_MAX          4    //加速时允许的最大速度误差
#define STEERREG_DEC_VELERR_MAX          4    //减速时允许的最大速度误差

/************************************************************
				制动控制常量与参数最大值范围
************************************************************/
#define BRAKEREG_ANGLE_RATIO            60       //减速比，即电机角度/执行器角度
#define BRAKEREG_ANGLE_MAX          (180*BRAKEREG_ANGLE_RATIO)       //转角最大值，度
#define BRAKEREG_ANGLE_MIN         (-180*BRAKEREG_ANGLE_RATIO)       //转角最小值，度
#define BRAKEREG_PIGAIN_MAX    10000          //PI控制器的P增益与I增益的最大值为2
#define BRAKEREG_UOUT_PMAX      4.99          //位置环PI控制器输出的正向最大值限制
#define BRAKEREG_UOUT_NMAX     -4.99          //位置环PI控制器输出的负向最大值限制
#define BRAKEREG_INTFRE       (CTRLREG_INTFRE)  //转向位置控制环的频率，单位Hz
#define BRAKEREG_INTPRD       (CTRLREG_INTPRD)  //转向位置控制环的周期，单位ms
#define BRAKEREG_VEL_MAX      ((3000.0*360.0)/(BRAKEREG_INTFRE*60))
#define BRAKEREG_VEL_MIN             4     //系统最小速度，单位度/控制周期
#define BRAKEREG_STALL_COUNT_MAX    40      //允许的堵转计数最大值，堵转延时为200ms；
#define BRAKEREG_INTP_ENDREGDIS        60    //预留的终点匀速距离，单位为度
#define BRAKEREG_ERROR_MAX             720    //跟踪误差最大值，单位为度
#define BRAKEREG_ACC_VELERR_MAX          4    //加速时允许的最大速度误差
#define BRAKEREG_DEC_VELERR_MAX          4    //减速时允许的最大速度误差

//模块常量声明
#define PIREG_SAT_POS  1   //正向饱和标志
#define PIREG_SAT_NEG  -1  //负向饱和标志
#define PIREG_SAT_NON  0   //未饱和标志


/*************************************************************

                     插补相关常量宏定义

**************************************************************/
//插补指令执行状态宏定义
//#define INTP_STATUS_END		    0x0000		 //完成运动
//#define INTP_STATUS_STA		    0x0001		 //开始状态
//#define INTP_STATUS_ERROR       0x0002       //插补指令错误
//#define INTP_STATUS_WORK        0x0004       //执行状态

//加减速状态宏定义
#define INTP_ACCSTAT_END        1000       //完成状态
#define INTP_ACCSTAT_STA        1001       //开始状态
#define INTP_ACCSTAT_BREG       1002       //开始匀速
#define INTP_ACCSTAT_ACC	    1003	   //加速状态
#define INTP_ACCSTAT_REG	    1004	   //匀速状态
#define INTP_ACCSTAT_DEC	    1005	   //减速状态
#define INTP_ACCSTAT_EREG       1006       //终点匀速
#define INTP_ACCSTAT_REVDEC     1007       //反向减速状态
#define INTP_ACCSTAT_ORIGIN     1008       //回零状态
#define INTP_ACCSTAT_REST       1009       //静止状态

#define INTP_MAXFRERATIO         2           //允许一步完成的比例

#define VK_OEM_PLUS             0x00BB       //+虚拟键
#define VK_OEM_MINUS            0x00BD       //-虚拟键
#define VK_LEFT_ARROW           0x0025       //<-左箭头虚拟键
#define VK_UP_ARROW             0x0026       //上箭头虚拟键
#define VK_RIGHT_ARROW          0x0027       //->右箭头虚拟键
#define VK_Z                    0x005A       //Z虚拟键
#define VK_X                    0x0058       //X虚拟键
#define VK_C                    0x0043       //C虚拟键
#define VK_F                    0x0046       //F虚拟键
#define VK_J                    0x004A       //J虚拟键



//自定义的一些小函数
void CopyWORD(WORD* wp1,WORD* wp2);//按字复制数据
void CopyDWORD(DWORD* wp1,DWORD* wp2);//按双字复制数据


#endif
