/*----------------------------------------------------------------------

			定义板卡模块card

----------------------------------------------------------------------*/

#include "stdafx.h"
#include <windows.h>
#include "string.h"
#include "limits.h"
#include "card.h"
#include "C:\Program Files\Advantech\Adsapi\Include\driver.h"

//////////////////////////////////////////////////////////////////////////
//名称：card
//类别：TI/AC
//描述：模块的构造函数函数，对内部变量进行初始化
//////////////////////////////////////////////////////////////////////////
CARD::CARD()
{
    Status = CARD_STATUS_CLOSE;			//板卡状态：0-未打开，1-1721未打开，2-1784未打开，3-全部打开
	DoState = 0;						//STATUS,数字输出状态，每一位对应一个通道的状态
	AO_Enable = CARD_1721_AO_DISABLE;	//STATUS,模拟输出使能，0：禁用，1：使能；

//	LONG        DriverHandle_1721;				//1721设备句柄
//	LONG        DriverHandle_1784;              //1784设备句柄
	gwChannel = 0;                      // input channel

	autosel_sw = 0;                     //OUTPUT，手动/自动开关状态
	emgstop_sw = 0;                     //OUTPUT，急停开关状态
	strplim_sw = 0;                       //OUTPUT，转向正向限位状态
	strnlim_sw = 0;                       //OUTPUT，转向负向限位状态
	strzero_sw = 0;                       //OUTPUT，转向零点状态
	brkplim_sw = 0;                       //OUTPUT，制动正向限位状态
	brkzero_sw = 0;                       //OUTPUT，制动零点状态


	autosel_count = 0;                     //STATUS，手动/自动开关状态读取计数
	emgstop_count = 0;                     //STATUS，急停开关状态读取计数
	strplim_count = 0;                       //STATUS，转向正向限位状态读取计数
	strnlim_count = 0;                       //STATUS，转向负向限位状态读取计数
	strzero_count = 0;                        //STATUS，转向零点状态的读取次数
	strzero_evt_count = 0;                   //STATUS，转向零点事件的读取次数
	brkplim_count = 0;                       //STATUS，制动正向限位状态读取计数
	brknlim_count = 0;                       //STATUS，制动负向限位状态读取计数

	DeviceNum_1721 = 0;                 //1721设备号，默认为0
    DeviceNum_1784 = 0;                 //1784设备号，默认为1
}

//////////////////////////////////////////////////////////////////////////
//打开板卡
//////////////////////////////////////////////////////////////////////////
LRESULT CARD::CARD_ON(void)
{
//    int i;
	WORD cha_sel;//通道号
	//1784内部变量
    PT_QCounterConfig  ptQCounterConfig;
    PT_QCounterStart   ptQCounterStart;
//	PT_CounterEventStart ptCounterEventStart;
//PT_CounterEventRead  ptCounterEventRead;

	LONG			pBuffer[5];
	float           pfClock[5];
	ULONG           ulSize;
	
 /*   注销1721板卡的功能
 //打开板卡
	//-------------------打开1721---------------------
    ErrCde = DRV_DeviceOpen(DeviceNum_1721,&DriverHandle_1721);
    if (ErrCde != SUCCESS)
    {
         Status = CARD_STATUS_FAIL_0;	//更新板卡状态
		 strcpy(szErrMsg,"Device 1721 open error !"); //更新返回故障消息		 

         return ErrCde;
    }
	//1.配置1721的模拟输出
	//1.1 油门输出1:0通道
	cha_sel = CARD_1721_CHL_GAS_1;
	ptAOConfig[cha_sel].chan = cha_sel; //设定通道号
	ptAOConfig[cha_sel].RefSrc = 0;		//选取内部参考电压
	ptAOConfig[cha_sel].MaxValue = 5;	//设定输出电压的最大值
	ptAOConfig[cha_sel].MinValue = 0;	//设定输出电压的最小值
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//配置模拟输出
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//更新板卡状态
		 strcpy(szErrMsg,"Device 1721 AO cha-0 config error !"); //更新返回故障消息		 

		return ErrCde;
	}
		
	//1.2 油门输出2:1通道
	cha_sel = CARD_1721_CHL_GAS_2;
	ptAOConfig[cha_sel].chan = cha_sel; //设定通道号
	ptAOConfig[cha_sel].RefSrc = 0;		//选取内部参考电压
	ptAOConfig[cha_sel].MaxValue = 5;	//设定输出电压的最大值
	ptAOConfig[cha_sel].MinValue = 0;	//设定输出电压的最小值
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//配置模拟输出
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//更新板卡状态
		 strcpy(szErrMsg,"Device 1721 AO cha-1 config error !"); //更新返回故障消息		 

		return ErrCde;
	}

	//1.3 转向输出:2通道
	cha_sel = CARD_1721_CHL_STR;
	ptAOConfig[cha_sel].chan = cha_sel; //设定通道号
	ptAOConfig[cha_sel].RefSrc = 0;		//选取内部参考电压
	ptAOConfig[cha_sel].MaxValue = 5;	//设定输出电压的最大值
	ptAOConfig[cha_sel].MinValue = 0;	//设定输出电压的最小值
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//配置模拟输出
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//更新板卡状态
		 strcpy(szErrMsg,"Device 1721 AO cha-2 config error !"); //更新返回故障消息		 

		return ErrCde;
	}

	//1.4 制动输出:3通道
	cha_sel = CARD_1721_CHL_BRK;
	ptAOConfig[cha_sel].chan = cha_sel; //设定通道号
	ptAOConfig[cha_sel].RefSrc = 0;		//选取内部参考电压
	ptAOConfig[cha_sel].MaxValue = 5;	//设定输出电压的最大值
	ptAOConfig[cha_sel].MinValue = 0;	//设定输出电压的最小值
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//配置模拟输出
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//更新板卡状态
		 strcpy(szErrMsg,"Device 1721 AO cha-3 config error !"); //更新返回故障消息		 

		return ErrCde;
	}

	//2.对各个模拟通道输出进行清零（！！！！！！！！！！！）
	//2.设定各个模拟通道输出为0
	for ( i=0;i<CARD_1721_CHL_NUM;i++ )
	{
		vlt_out[i] = 0;	
		ErrCde = write_1721_AO(i);
	}
*/
	 //-------------------------打开1784-------------------------
     ErrCde = DRV_DeviceOpen(DeviceNum_1784,&DriverHandle_1784);
     if (ErrCde != SUCCESS)
     {
         Status = CARD_STATUS_FAIL_1;  //更新板卡状态
		 strcpy(szErrMsg,"Device 1784 open error !"); //更新返回故障消息		 

         return ErrCde;
     }

	 //-----------------配置1784---------------------
	 //1.配置属性参数，注意只需要配置采用QEP计数的1通道与2通道，不需要配置采用脉冲增量计数的0通道。
	 //每个通道均设定使用滤波器
	 ulSize = sizeof(LONG) * 5;
	 ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrDigitalFilter, pBuffer, &ulSize);
	 pBuffer[CARD_1784_CHL_CNT] = 0;//0:不使用, 1：使用
 	 pBuffer[CARD_1784_CHL_STR] = 0;//0:不使用，1：使用；
  	 pBuffer[CARD_1784_CHL_BRK] = 1;//0:不使用，1：使用；

	 ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrDigitalFilter, pBuffer, ulSize);
	
	//设定时钟为8MHz
	ulSize = sizeof(float) * 5;
	ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrClockFrequency, pfClock, &ulSize);
	pfClock[CARD_1784_CHL_CNT] = 8000000.0;
	pfClock[CARD_1784_CHL_STR] = 8000000.0;
    pfClock[CARD_1784_CHL_BRK] = 8000000.0;
	ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrClockFrequency, pfClock, ulSize);
					
	//设定锁止类型为0（无锁止）,1(上溢锁止），2（下溢锁止），3（上溢或下溢均锁止）
	ulSize = sizeof(LONG) * 5;
	ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrCounterLockControl, pBuffer, &ulSize);
	pBuffer[CARD_1784_CHL_CNT] = 0;
	pBuffer[CARD_1784_CHL_STR] = 0;
	pBuffer[CARD_1784_CHL_BRK] = 0;
	ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrCounterLockControl, pBuffer, ulSize);
					
	// index reset，默认为禁用
	ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrIndexReset, pBuffer, &ulSize);
	pBuffer[CARD_1784_CHL_CNT] &= ~(0x1 << CARD_1784_CHL_CNT);
	pBuffer[CARD_1784_CHL_STR] &= ~(0x1 << CARD_1784_CHL_STR);
	pBuffer[CARD_1784_CHL_BRK] &= ~(0x1 << CARD_1784_CHL_BRK);
	ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrIndexReset, pBuffer, ulSize);

	////2.启动通道0，该通道用于油门模块的脉冲计数
	////配置启动计数器
	//cha_sel = CARD_1784_CHL_CNT;
	////配置计数器操作
	//ptQCounterConfig.counter = cha_sel;//配置通道号
	//ptQCounterConfig.LatchSrc = SWLATCH;//软件锁止计数器，用户必须从高到低读取计数器
	//ptQCounterConfig.LatchOverflow = 0;//计数器上溢时不锁止
	//ptQCounterConfig.ResetOnLatch = 1;//计数器锁止后复位
	//ptQCounterConfig.ResetValue = 0;//复位后计数器值为0
	//ErrCde = DRV_QCounterConfig (DriverHandle_1784,&ptQCounterConfig);
	//if ( ErrCde!=SUCCESS )
	//{
	//	Status = CARD_STATUS_FAIL_1;  //更新板卡状态
	//	strcpy(szErrMsg,"Device 1784 QEP-0 Config error !"); //更新返回故障消息
	//	return ErrCde;
	//}
	////启动计数器
	//ptQCounterStart.counter = cha_sel;//配置通道号
 //   ptQCounterStart.InputMode = ABPHASEX4;//配置输入模式为4倍频的AB相正交输入
 //   ErrCde = DRV_QCounterStart(DriverHandle_1784,(LPT_QCounterStart)&ptQCounterStart);
	//if ( ErrCde!=SUCCESS )
	//{
	//	Status = CARD_STATUS_FAIL_1;  //更新板卡状态
	//	strcpy(szErrMsg,"Device 1784 QEP-0 Start error !"); //更新返回故障消息
	//	return ErrCde;
	//}

	//3.配置并启动通道1，该通道用于转向模块的QEP计数
	cha_sel = CARD_1784_CHL_STR;
	//配置计数器操作
	ptQCounterConfig.counter = cha_sel;//配置通道号
	ptQCounterConfig.LatchSrc = SWLATCH;//软件锁止计数器，用户必须从高到低读取计数器
	ptQCounterConfig.LatchOverflow = 0;//计数器上溢时不锁止
	ptQCounterConfig.ResetOnLatch = 1;//计数器锁止后复位
	ptQCounterConfig.ResetValue = 0;//复位后计数器值为0
	ErrCde = DRV_QCounterConfig (DriverHandle_1784,&ptQCounterConfig);
	if ( ErrCde!=SUCCESS )
	{
		Status = CARD_STATUS_FAIL_1;  //更新板卡状态
		strcpy(szErrMsg,"Device 1784 QEP-1 Config error !"); //更新返回故障消息
		return ErrCde;
	}
	//启动计数器
	ptQCounterStart.counter = cha_sel;//配置通道号
    ptQCounterStart.InputMode = ABPHASEX4;//配置输入模式为4倍频的AB相正交输入
 //   ErrCde = DRV_QCounterStart(DriverHandle_1784,(LPT_QCounterStart)&ptQCounterStart);
	//if ( ErrCde!=SUCCESS )
	//{
	//	Status = CARD_STATUS_FAIL_1;  //更新板卡状态
	//	strcpy(szErrMsg,"Device 1784 QEP-1 Start error !"); //更新返回故障消息
	//	return ErrCde;
	//}

	//4.配置并启动通道2，该通道用于制动模块的QEP计数
	cha_sel = CARD_1784_CHL_BRK;
	//配置计数器操作
	ptQCounterConfig.counter = cha_sel;//配置通道号
	ptQCounterConfig.LatchSrc = SWLATCH;//软件锁止计数器，用户必须从高到低读取计数器
	ptQCounterConfig.LatchOverflow = 0;//计数器上溢时不锁止
	ptQCounterConfig.ResetOnLatch = 1;//计数器锁止后复位
	ptQCounterConfig.ResetValue = 0;//复位后计数器值为0
	ErrCde = DRV_QCounterConfig (DriverHandle_1784,&ptQCounterConfig);
	if ( ErrCde!=SUCCESS )
	{
		Status = CARD_STATUS_FAIL_1;  //更新板卡状态
		strcpy(szErrMsg,"Device 1784 QEP-2 Config error !"); //更新返回故障消息
		return ErrCde;
	}
	//启动计数器
	ptQCounterStart.counter = cha_sel;//配置通道号
    ptQCounterStart.InputMode = ABPHASEX4;//配置输入模式为4倍频的AB相正交输入
    ErrCde = DRV_QCounterStart(DriverHandle_1784,(LPT_QCounterStart)&ptQCounterStart);
	if ( ErrCde!=SUCCESS )
	{
		Status = CARD_STATUS_FAIL_1;  //更新板卡状态
		strcpy(szErrMsg,"Device 1784 QEP-2 Start error !"); //更新返回故障消息
		return ErrCde;
	}

	//5. 0-2通道的counter reset，对各通道的计数器进行初始清零（！！！！！！！！！！！）
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)CARD_1784_CHL_CNT);
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)CARD_1784_CHL_STR);
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)CARD_1784_CHL_BRK);
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)3);
	if ( ErrCde != SUCCESS )
	{
		return ErrCde;
	}

/*
	//----------------设定板卡1721的数字输出--------------------
	//3.1 使能转向电机
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_STR_EN,1);
	if ( ErrCde!=SUCCESS )
	{
//	 	MessageBox("1721 DO-0 config error");
		strcpy(szErrMsg,"1721 DO-0 config error!"); //更新返回故障消息

		return ErrCde;
	}

	//3.2 使能制动电机
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_BRK_EN,1);
	if ( ErrCde!=SUCCESS )
	{
//		MessageBox("1721 DO-4 config error");
		strcpy(szErrMsg,"1721 DO-4 config error!"); //更新返回故障消息

		return ErrCde;
	}

	//3.3 使能转向电机与制动电机的制动信号，以避免其失控
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_STR_BRK,1);
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_BRK_BRK,1);
*/
	
	Status = CARD_STATUS_OPEN; //设定板卡为打开状态

	return ErrCde;
}

//////////////////////////////////////////////////////////////////////////
//关闭板卡
//////////////////////////////////////////////////////////////////////////
LRESULT CARD::CARD_OFF(void)
{
//	int i;
/*	
	//1.关闭电机使能
	//关闭转向电机使能
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_STR_EN,0);
	if ( ErrCde!=SUCCESS )
	{
		MessageBox(NULL,"1721 DO-0 write error","",MB_OK);
		return MSG_ERROR;
	}

	//关闭制动电机使能
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_BRK_EN,0);
	if ( ErrCde!=SUCCESS )
	{
		MessageBox(NULL,"1721 DO-4 write error","",MB_OK);
		return MSG_ERROR;
	}

	//2.设定各个模拟通道输出为0（！！！！！！！！！！！）
	for ( i=0;i<CARD_1721_CHL_NUM;i++ )
	{
		vlt_out[i] = 0;	
		ErrCde = write_1721_AO(i);
	}
	
	
	
	//-------------关闭1721-----------------
	ErrCde = DRV_DeviceClose( &DriverHandle_1721 ); 
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}

*/
	//-------------关闭1784------------------
	//0-2通道的counter reset，清零计数器（！！！！！！！！！！！）
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)CARD_1784_CHL_CNT);
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)CARD_1784_CHL_STR);
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)CARD_1784_CHL_BRK);
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}

	ErrCde = DRV_DeviceClose( &DriverHandle_1784 );
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}

	Status = CARD_STATUS_CLOSE; //设定板卡为CLOSE状态

	return ErrCde;
}


//////////////////////////////////////////////////////////////////////////
//功能:读取1784的计数
//输入参数：cha_sel,通道号,0:车速PWM，1-转向QEP，2-制动QEP;
//////////////////////////////////////////////////////////////////////////
LRESULT CARD::read_1784_CNT(WORD cha_sel)
{
	PT_CounterEventRead ptCounterEventRead;
	PT_QCounterRead    ptQCounterRead;
	ULONG  CountRead = 0;
	ULONG  ulHiCount   = 0;     /* high byte of counter              */
	ULONG  data;

	
	switch ( cha_sel )
	{
	case CARD_1784_CHL_CNT://读取PWM的脉冲计数
		ptCounterEventRead.counter = CARD_1784_CHL_GAS;//设定通道号
        ptCounterEventRead.overflow = (USHORT *)&gwOverflow;//设定溢出状态指针，1：溢出
        ptCounterEventRead.count = (ULONG *)&CountRead;//设定读取脉冲数指针
		//注意，此处没有计数溢出的情况，因为返回的计数值为ULONG型，实际连续使用中不会溢出；

		//读取返回脉冲数
		ErrCde = DRV_CounterEventRead(DriverHandle_1784,(LPT_CounterEventRead)&ptCounterEventRead);
		if ( ErrCde != SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-0 read error !"); //更新返回故障消息
			return ErrCde;
		}
		
		//更新输出值
		cnt_in[cha_sel] = CountRead;

		//更新上一次值
		CountRead_last[cha_sel] = CountRead;

		return SUCCESS;
		
		break;
	case CARD_1784_CHL_STR://读取QEP计数

		ptQCounterRead.counter = cha_sel;//设定输入通道
		ptQCounterRead.overflow = (USHORT *)&gwOverflow;//设定返回的溢出指针
		ptQCounterRead.LoCount  = (ULONG *)&CountRead;//设定返回的32位计数指针
		ptQCounterRead.HiCount  = (ULONG  far *)&ulHiCount;
		
		ErrCde = DRV_QCounterRead(DriverHandle_1784,(LPT_QCounterRead)&ptQCounterRead);
		if ( ErrCde!=SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-1 read error !"); //更新返回故障消息
			return ErrCde;
		}
		
		//更新输出值
		if ( CountRead>LONG_MAX )//读到的为负数
		{
			data = (ULONG_MAX - CountRead) + 1;
			cnt_in[cha_sel] = (int)data;
			cnt_in[cha_sel] *= -1;
		}
		else
		{
			cnt_in[cha_sel] = CountRead;
		}


		//更新上一次值
		CountRead_last[cha_sel] = CountRead;
		
		return SUCCESS;

		break;
	case CARD_1784_CHL_BRK://读取QEP计数
		ptQCounterRead.counter = cha_sel;//设定输入通道
		ptQCounterRead.overflow = (USHORT *)&gwOverflow;//设定返回的溢出指针
		ptQCounterRead.LoCount  = (ULONG *)&CountRead;//设定返回的32位计数指针
		ptQCounterRead.HiCount  = (ULONG  far *)&ulHiCount;
		
		ErrCde = DRV_QCounterRead(DriverHandle_1784,(LPT_QCounterRead)&ptQCounterRead);
		if ( ErrCde!=SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-2 read error !"); //更新返回故障消息
			return ErrCde;
		}
		
		//更新输出值
		if ( CountRead>LONG_MAX )//读到的为负数
		{
			data = (ULONG_MAX - CountRead) + 1;
			cnt_in[cha_sel] = (int)data;
			cnt_in[cha_sel] *= -1;
		}
		else
		{
			cnt_in[cha_sel] = CountRead;
		}


		//更新上一次值
		CountRead_last[cha_sel] = CountRead;

		return SUCCESS;

		break;
	case 3://读取QEP计数

		ptQCounterRead.counter = cha_sel;//设定输入通道
		ptQCounterRead.overflow = (USHORT *)&gwOverflow;//设定返回的溢出指针
		ptQCounterRead.LoCount  = (ULONG *)&CountRead;//设定返回的32位计数指针
		ptQCounterRead.HiCount  = (ULONG  far *)&ulHiCount;

		ErrCde = DRV_QCounterRead(DriverHandle_1784,(LPT_QCounterRead)&ptQCounterRead);
		if ( ErrCde!=SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-1 read error !"); //更新返回故障消息
			return ErrCde;
		}

		//更新输出值
		if ( CountRead>LONG_MAX )//读到的为负数
		{
			data = (ULONG_MAX - CountRead) + 1;
			cnt_in[cha_sel] = (int)data;
			cnt_in[cha_sel] *= -1;
		}
		else
		{
			cnt_in[cha_sel] = CountRead;
		}


		//更新上一次值
		CountRead_last[cha_sel] = CountRead;

		return SUCCESS;

		break;
	default:

		return MSG_ERROR;
		break;
	}

	return MSG_ERROR;
}

///////////////////////////////////////////////////////////////
//1721配置函数
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//1721更新函数，结果从vlt_cout输出
///////////////////////////////////////////////////////////////
LRESULT CARD::write_1721_AO(WORD cha_sel )
{
	//模拟输出未使能时，只允许输出0。只有在使能时，才允许输出非0的模拟量
	if ( AO_Enable == CARD_1721_AO_DISABLE  )
	{
		vlt_out[cha_sel] = 0;		
	}	
	else
	{
		//将当前通道的模拟输出限制为不超过4.9V
		if ( vlt_out[cha_sel]>4.9 )
		{
			vlt_out[cha_sel] = (float)4.9;
		}
		else if ( vlt_out[cha_sel]<0.01 ) //不低于0.01
		{
			vlt_out[cha_sel] = (float)0.01;
		}
	}

	
	switch ( cha_sel )
	{
	case CARD_1721_CHL_GAS_1:

		gwChannel = cha_sel;
		ptAOVoltageOut[cha_sel].chan = gwChannel;
        ptAOVoltageOut[cha_sel].OutputValue = vlt_out[cha_sel];

		ErrCde = DRV_AOVoltageOut(DriverHandle_1721,(LPT_AOVoltageOut)&ptAOVoltageOut[cha_sel]);

		return ErrCde;

		break;

	case CARD_1721_CHL_GAS_2:

    	gwChannel = cha_sel;
		ptAOVoltageOut[cha_sel].chan = gwChannel;
        ptAOVoltageOut[cha_sel].OutputValue = vlt_out[cha_sel];

		ErrCde = DRV_AOVoltageOut(DriverHandle_1721,(LPT_AOVoltageOut)&ptAOVoltageOut[cha_sel]);

		return ErrCde;

		break;

	case CARD_1721_CHL_STR:

    	gwChannel = cha_sel;
		ptAOVoltageOut[cha_sel].chan = gwChannel;
        ptAOVoltageOut[cha_sel].OutputValue = vlt_out[cha_sel];

		ErrCde = DRV_AOVoltageOut(DriverHandle_1721,(LPT_AOVoltageOut)&ptAOVoltageOut[cha_sel]);

		return ErrCde;


		break;

	case CARD_1721_CHL_BRK:

	    gwChannel = cha_sel;
		ptAOVoltageOut[cha_sel].chan = gwChannel;
        ptAOVoltageOut[cha_sel].OutputValue = vlt_out[cha_sel];

		ErrCde = DRV_AOVoltageOut(DriverHandle_1721,(LPT_AOVoltageOut)&ptAOVoltageOut[cha_sel]);

		return ErrCde;


		break;

		
	default:

		break;
	
	}

	return MSG_ERROR;
}


///////////////////////////////////////////////////
//更新1721的数字输出
//输入参数为通道号及其高低电平
///////////////////////////////////////////////////
LRESULT CARD::write_1721_DO(WORD cha_sl, WORD cmd_en)
{
    WORD data;

	//首先读取当前IO输出
	/*
	ptDioGetCurrentDOByte.port  = 0;//1721的数字输出共0、1两个端口16位输出，分别对应B0-B7,B8-B15位
    ptDioGetCurrentDOByte.value = (USHORT far *)&DoState;

    ErrCde = DRV_DioGetCurrentDOByte(DriverHandle_1721,(LPT_DioGetCurrentDOByte)&ptDioGetCurrentDOByte);
	if ( ErrCde!=SUCCESS )
	{
		return ErrCde;
	}
	*/

	data = DoState;

	switch (cha_sl )
	{
	case CARD_1721_DO_CHL_STR_EN://转向电机使能
		if ( cmd_en ) //使能
		{
			data |= 1<<cha_sl;
		}
		else //取消
		{
			data &= (~(1<<cha_sl));
		}

		break;
	case CARD_1721_DO_CHL_STR_REV://转向电机反转
		if ( cmd_en ) //使能
		{
			data |= 1<<cha_sl;
		}
		else //取消
		{
			data &= (~(1<<cha_sl));
		}
		break;
	case CARD_1721_DO_CHL_STR_BRK://转向电机制动
		if ( cmd_en ) //使能
		{
			data |= 1<<cha_sl;
		}
		else //取消
		{
			data &= (~(1<<cha_sl));

		}		
		break;
	case CARD_1721_DO_CHL_BRK_EN://制动电机使能
		if ( cmd_en ) //使能
		{
			data |= 1<<cha_sl;
		}
		else //取消
		{
			data &= (~(1<<cha_sl));
		}		
		break;
	case CARD_1721_DO_CHL_BRK_REV://制动电机反转
		if ( cmd_en ) //使能
		{
			data |= 1<<cha_sl;
		}
		else //取消
		{
			data &= (~(1<<cha_sl));
		}
		break;
	case CARD_1721_DO_CHL_BRK_BRK://制动电机制动
		if ( cmd_en ) //使能
		{
			data |= 1<<cha_sl;
		}
		else //取消
		{
			data &= (~(1<<cha_sl));
		}
		break;
	default:
		break;
	}
	
	ptDioWritePortByte.port  = CARD_1721_DO_PORT;//1721的数字输出共0、1两个端口16位输出，分别对应B0-B7,B8-B15位
    ptDioWritePortByte.mask  = 0xff;
    ptDioWritePortByte.state = data;

    ErrCde = DRV_DioWritePortByte(DriverHandle_1721, (LPT_DioWritePortByte)&ptDioWritePortByte);
	if ( ErrCde != SUCCESS )
	{
		strcpy(szErrMsg,"Device 1721 DO write error !"); //更新返回故障消息	
		return ErrCde;
	}
	//更新当前数字输出状态
	DoState = data;

	return ErrCde;
}

//更新1721的DI输入读取
//根据读取值，更新输入开关的状态
LRESULT CARD::read_1721_DI(void)
{
	gwChannel = CARD_1721_DI_PORT;
	ptDioReadPortByte.port = gwChannel;
    ptDioReadPortByte.value = (USHORT far *)&DiState;

	ErrCde = DRV_DioReadPortByte(DriverHandle_1721, (LPT_DioReadPortByte)&ptDioReadPortByte);
    if ( ErrCde != SUCCESS )
    {
		strcpy(szErrMsg,"Device 1721 DI read error !"); //更新返回故障消息	
		return ErrCde;
    }

	//判断自动驾驶开关是否闭合，闭合为高电平
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_AUTOSEL) ) == 0 )
	{
		autosel_sw = 0;//开关断开
	}
	else
	{
		autosel_sw = 1;//开关闭合
	}

	//判断紧急反向开关是否闭合，闭合为高电平
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_EMGSTOP) ) == 0 )
	{
		emgstop_sw = 0;//开关断开
	}
	else
	{
		emgstop_sw = 1;//开关闭合
	}

	
	//判断转向正限位开关是否闭合，闭合为高电平
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_STRPLIM) ) == 0 )
	{
		strplim_sw = 0;//开关断开		
	}
	else
	{
		strplim_sw = 1;//开关闭合
	}

	//判断转向负限位开关是否闭合，闭合为高电平
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_STRNLIM) ) == 0 )
	{
		strnlim_sw = 0;//开关断开
	}
	else
	{
		strnlim_sw = 1;//开关闭合
	}

	//判断制动限位开关是否闭合，闭合为高电平
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_BRKPLIM) ) == 0 )
	{
		brkplim_sw = 0;//开关断开
	}
	else
	{
		brkplim_sw = 1;//开关闭合
	}

	//判断制动零点开关是否闭合，闭合为高电平
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_BRKZERO) ) == 0 )
	{
		brkzero_sw = 0;//开关断开
	}
	else
	{
		brkzero_sw = 1;//开关闭合
	}
	

	//通过查询事件，判断转向零点开关是否发生过闭合
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_STRNLIM) ) == 0 )
	{
		strzero_sw = 0;//开关断开
	}
	else
	{
		strzero_sw = 1;//开关闭合
	}


	return ErrCde;

}

///////////////////////////////////////////////////////////////////
//1784复位函数，清零计数器
///////////////////////////////////////////////////////////////////
LRESULT CARD::reset_1784_CNT(WORD chl)
{
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)chl);

	return ErrCde;
}

////////////////////////////////////////////////////////////////////
//启动1784卡对转向回零的事件使能
////////////////////////////////////////////////////////////////////
LRESULT CARD::config_1784_EVENT_strori_start(void)
{
 	PT_EnableEvent	 EventSetting;
	
	EventSetting.EventType = ADS_EVT_DI_INTERRUPT1;
	EventSetting.Enabled   = TRUE;
	EventSetting.Count     = 1;
	ErrCde = DRV_EnableEvent(DriverHandle_1784, &EventSetting );

	strzero_evt_count = 0;//清零事件计数

	return ErrCde;

}

/////////////////////////////////////////////////////////////////////
//停止1784卡对转向回零的事件使能
/////////////////////////////////////////////////////////////////////
LRESULT CARD::config_1784_EVENT_strori_end(void)
{
 	PT_EnableEvent	 EventSetting;
	
	EventSetting.EventType = ADS_EVT_DI_INTERRUPT1;
	EventSetting.Enabled   = FALSE;
	EventSetting.Count     = 1;
	ErrCde = DRV_EnableEvent(DriverHandle_1784, &EventSetting );

	return ErrCde;
}

///////////////////////////////////////////////////////////////////////
//查询1784卡对转向回零的事件
///////////////////////////////////////////////////////////////////////
LRESULT CARD::read_1784_EVENT_strori(void)
{
	PT_CheckEvent	ptCheckEvent;
	USHORT	usEventType;

	ptCheckEvent.EventType = &usEventType;
	ptCheckEvent.Milliseconds = 60000;//事件的超时限制为60s

	ErrCde = DRV_CheckEvent(DriverHandle_1784, &ptCheckEvent );
	if ( ErrCde == SUCCESS)
	{
		if( usEventType == ADS_EVT_DI_INTERRUPT1 )
		{
			strzero_evt_count++;//增加事件计数
		}
	}

	return ErrCde;

}


//////////////////////////////////////////////////////////////////////
////读取1784卡的数字输入
//////////////////////////////////////////////////////////////////////
LRESULT CARD::read_1784_DI(void)
{
	ptDioReadPortByte.port = 0;
    ptDioReadPortByte.value = (USHORT far *)&DiState;

	ErrCde = DRV_DioReadPortByte(DriverHandle_1784, (LPT_DioReadPortByte)&ptDioReadPortByte);
    if ( ErrCde != SUCCESS )
    {
		strcpy(szErrMsg,"Device 1784 DI read error !"); //更新返回故障消息	
		return ErrCde;
    }

	//判断转向回零开关是否闭合，闭合为高电平
	if ( ( (DiState) & (1<<CARD_1784_DI_CHL_STRZERO) ) == 0 )
	{
		strzero_sw = 1;//开关断开
	}
	else
	{
		strzero_sw = 0;//开关闭合
	}

	return ErrCde;
}
