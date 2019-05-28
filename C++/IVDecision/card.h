//定义板卡模块
#ifndef _CARD
#define _CARD

#include "datatype.h"
#include "C:\Program Files\Advantech\Adsapi\Include\Driver.h" 

// ------------------------------------------------------------------------
//	常量定义
// ------------------------------------------------------------------------
#define     MAX_DEVICES     5

//定义板卡打开状态
#define CARD_STATUS_CLOSE		0   //均未打开
#define CARD_STATUS_FAIL_0		1   //1721打开失败
#define CARD_STATUS_FAIL_1		2   //1784打开失败
#define CARD_STATUS_OPEN        3   //均已打开

//模拟输出使能标志
#define CARD_1721_AO_DISABLE    0   //模拟输出禁用，输出为0
#define CARD_1721_AO_ENABLE     1   //模拟输出使能，输出为非0值

//模拟输出通道号
#define CARD_1721_CHL_NUM       4   //1721模拟输出通道数
#define CARD_1721_CHL_GAS_1     0   //油门0~5V输出
#define CARD_1721_CHL_GAS_2     1   //油门0~2.5V输出
#define CARD_1721_CHL_STR       2   //转向0-5V输出
#define CARD_1721_CHL_BRK       3   //制动0-5V输出

//数字输出通道号
//1721
#define CARD_1721_DO_PORT         0  //DO输出端口号
#define CARD_1721_DO_CHL_NUM      6  //1721 DO输出通道数
#define CARD_1721_DO_CHL_STR_EN   0  //转向电机使能（高电平有效）
#define CARD_1721_DO_CHL_STR_REV  1  //转向电机反转（方向）
#define CARD_1721_DO_CHL_STR_BRK  2  //转向电机制动（高电平有效）
#define CARD_1721_DO_CHL_BRK_EN   3  //制动电机使能（高电平有效）
#define CARD_1721_DO_CHL_BRK_REV  4  //制动电机反转（方向）
#define CARD_1721_DO_CHL_BRK_BRK  5  //制动电机制动（高电平有效）

//数字输入通道号
//1721
#define CARD_1721_DI_PORT          1   //DI输入端口号
#define CARD_1721_DI_CHL_NUM       6   //1721 DI输入通道数
#define CARD_1721_DI_CHL_AUTOSEL   8   //自动驾驶开关（高电平有效）
#define CARD_1721_DI_CHL_EMGSTOP   9   //急停开关
#define CARD_1721_DI_CHL_STRPLIM   10  //转向电机正向限位（高电平有效）
#define CARD_1721_DI_CHL_STRNLIM   11  //转向电机负向限位（高电平有效）
#define CARD_1721_DI_CHL_BRKPLIM   12  //制动电机限位（高电平有效）
#define CARD_1721_DI_CHL_BRKZERO   13  //制动电机零点（高电平有效）
//1784
#define CARD_1784_DI_CHL_STRZERO    1  //转向电机零点（高电平有效）



//计数或QEP输入通道号
#define CARD_1784_CHL_NUM       3   //1784输入通道数
#define CARD_1784_CHL_CNT       0   //用于计数的通道
#define CARD_1784_CHL_GAS       0   //车速输入通道号，脉冲/方向计数，方向固定为0（增计数）
#define CARD_1784_CHL_BRK       1   //制动电机QEP输入通道号，QEP计数
#define CARD_1784_CHL_STR       2   //转向电机QEP输入通道号，QEP计数



//板卡模块定义
class CARD {
public:
	//模块的构造函数，对内部变量进行初始化
	CARD();

    LRESULT CARD_ON(void);//打开板卡
	LRESULT CARD_OFF(void);//关闭板卡
	

	//1784配置函数
	//LRESULT config_1784_CNT(WORD chl);
	LRESULT config_1784_EVENT_strori_start();//启动1784卡的转向回零事件
	LRESULT config_1784_EVENT_strori_end();//停止1784卡的转向回零事件

	//1784更新函数，结果从cnt_in中读取
	LRESULT read_1784_CNT(WORD chl);
	LRESULT read_1784_EVENT_strori();//读取1784卡的转向回零事件
	LRESULT read_1784_DI();//读取1784卡的数字输入

	//1721配置函数
//	LRESULT config_1721_AO(WORD chl);
//	LRESULT config_1721_DO(void);

	//1721更新函数，结果从vlt_cout输出
	LRESULT write_1721_AO(WORD chl);
    LRESULT write_1721_DO(WORD chl, WORD cmd_en);
	LRESULT read_1721_DI(void);

	//1784复位函数，清零计数器
	LRESULT reset_1784_CNT(WORD chl);

public:
	float       vlt_out[CARD_1721_CHL_NUM];     //INPUT，期望输出电压，序号与通道号一致
	long        cnt_in[CARD_1784_CHL_NUM];		//OUTPUT，接收的脉冲计数，序号与通道号一致

	WORD        autosel_sw;                     //OUTPUT，手动/自动开关状态
	WORD        emgstop_sw;                     //OUTPUT，急停开关状态
	WORD        strplim_sw;                       //OUTPUT，转向正向限位状态
	WORD        strnlim_sw;                       //OUTPUT，转向负向限位状态
	WORD        strzero_sw;                       //OUTPUT，转向零点状态
	WORD        brkplim_sw;                       //OUTPUT，制动正向限位状态
	WORD        brkzero_sw;                       //OUTPUT，制动零点状态

	
	WORD        autosel_count;                     //STATUS，手动/自动开关状态读取计数
	WORD        emgstop_count;                     //STATUS，急停开关状态读取计数
	WORD        strplim_count;                       //STATUS，转向正向限位状态读取计数
	WORD        strnlim_count;                       //STATUS，转向负向限位状态读取计数
	WORD        strzero_count;                        //STATUS，转向零点状态的读取次数
	WORD        strzero_evt_count;                   //STATUS，转向零点事件的读取次数
	WORD        brkplim_count;                       //STATUS，制动正向限位状态读取计数
	WORD        brknlim_count;                       //STATUS，制动负向限位状态读取计数

	
	USHORT      DiState;                        //STATUS，数字输入状态，每一位对应一个通道的状态
	WORD        DoState;						//STATUS,数字输出状态，每一位对应一个通道的状态
	WORD        AO_Enable;						//STATUS,模拟输出使能，0：禁用，1：使能；
	
	WORD        Status;                 //STATUS，板卡状态：0-未打开，1-1721未打开，2-1784未打开，3-全部打开
	

	LONG        DriverHandle_1721;			//STATUS,1721设备句柄
	LONG        DriverHandle_1784;          //STATUS,1784设备句柄

	ULONG       DeviceNum_1721;             //CONFIG,1721设备号，默认为0
    ULONG       DeviceNum_1784;             //CONFIG,1784设备号，默认为1

	


	char        szErrMsg[80];           //STATUS,Use for MESSAGEBOX function
	LRESULT     ErrCde;                 //STATUS, Return error code

	USHORT      usChan0;	            //STATUS,1721通道号：0-油门5V输出，1-油门2.5V输出，2-转向输出，3-制动输出
	USHORT      usChan1;		        //STATUS,1784通道号：0-车速PWM反馈，2-转向电机QEP反馈，3-制动电机QEP反馈

	WORD        dir_str_type;           //CONFIG,设定转向电机方向类型，0：为正常，1：正向时使能反转电平
	WORD        dir_brk_type;           //CONFIG,设定制动电机方向类型，0：为正常，1：正向时使能反转电平

	
	ULONG					CountRead_last[CARD_1784_CHL_NUM];//STATUS,上一次读取的计数器值

	
	//1721板卡DLL相关变量
	USHORT      gwChannel;                      // input channel

	PT_AOConfig			ptAOConfig[CARD_1721_CHL_NUM];		//CONFIG,1721配置参数
	PT_AOVoltageOut		ptAOVoltageOut[CARD_1721_CHL_NUM];	//STATUS,1721输出电压

	PT_DioWritePortByte	ptDioWritePortByte;                 //STATUS,1721数字输出
	PT_DioReadPortByte	ptDioReadPortByte;                  //STATUS，1721数字输出读取
	PT_DioGetCurrentDOByte ptDioGetCurrentDOByte;
	PT_EnableEvent		EventSetting;

	//1784板卡DLL相关变量
	//PT_QCounterConfig		ptQCounterConfig[CARD_1784_CHL_NUM];		//STATUS,1784 QEP
	//PT_QCounterConfigSys	ptQCounterConfigSys[CARD_1784_CHL_NUM];	//STATUS,
	//PT_QCounterStart		ptQCounterStart[CARD_1784_CHL_NUM];			//STATUS,
	//PT_QCounterRead			ptQCounterRead[CARD_1784_CHL_NUM];			//STATUS,

	//PT_CounterEventStart	ptCounterEventStart[CARD_1784_CHL_NUM];//1784 COUNTER
	//PT_CounterEventRead		ptCounterEventRead[CARD_1784_CHL_NUM];
	USHORT					gwOverflow[CARD_1784_CHL_NUM];//溢出状态

};


#endif