/*----------------------------------------------------------------------

			����忨ģ��card

----------------------------------------------------------------------*/

#include "stdafx.h"
#include <windows.h>
#include "string.h"
#include "limits.h"
#include "card.h"
#include "C:\Program Files\Advantech\Adsapi\Include\driver.h"

//////////////////////////////////////////////////////////////////////////
//���ƣ�card
//���TI/AC
//������ģ��Ĺ��캯�����������ڲ��������г�ʼ��
//////////////////////////////////////////////////////////////////////////
CARD::CARD()
{
    Status = CARD_STATUS_CLOSE;			//�忨״̬��0-δ�򿪣�1-1721δ�򿪣�2-1784δ�򿪣�3-ȫ����
	DoState = 0;						//STATUS,�������״̬��ÿһλ��Ӧһ��ͨ����״̬
	AO_Enable = CARD_1721_AO_DISABLE;	//STATUS,ģ�����ʹ�ܣ�0�����ã�1��ʹ�ܣ�

//	LONG        DriverHandle_1721;				//1721�豸���
//	LONG        DriverHandle_1784;              //1784�豸���
	gwChannel = 0;                      // input channel

	autosel_sw = 0;                     //OUTPUT���ֶ�/�Զ�����״̬
	emgstop_sw = 0;                     //OUTPUT����ͣ����״̬
	strplim_sw = 0;                       //OUTPUT��ת��������λ״̬
	strnlim_sw = 0;                       //OUTPUT��ת������λ״̬
	strzero_sw = 0;                       //OUTPUT��ת�����״̬
	brkplim_sw = 0;                       //OUTPUT���ƶ�������λ״̬
	brkzero_sw = 0;                       //OUTPUT���ƶ����״̬


	autosel_count = 0;                     //STATUS���ֶ�/�Զ�����״̬��ȡ����
	emgstop_count = 0;                     //STATUS����ͣ����״̬��ȡ����
	strplim_count = 0;                       //STATUS��ת��������λ״̬��ȡ����
	strnlim_count = 0;                       //STATUS��ת������λ״̬��ȡ����
	strzero_count = 0;                        //STATUS��ת�����״̬�Ķ�ȡ����
	strzero_evt_count = 0;                   //STATUS��ת������¼��Ķ�ȡ����
	brkplim_count = 0;                       //STATUS���ƶ�������λ״̬��ȡ����
	brknlim_count = 0;                       //STATUS���ƶ�������λ״̬��ȡ����

	DeviceNum_1721 = 0;                 //1721�豸�ţ�Ĭ��Ϊ0
    DeviceNum_1784 = 0;                 //1784�豸�ţ�Ĭ��Ϊ1
}

//////////////////////////////////////////////////////////////////////////
//�򿪰忨
//////////////////////////////////////////////////////////////////////////
LRESULT CARD::CARD_ON(void)
{
//    int i;
	WORD cha_sel;//ͨ����
	//1784�ڲ�����
    PT_QCounterConfig  ptQCounterConfig;
    PT_QCounterStart   ptQCounterStart;
//	PT_CounterEventStart ptCounterEventStart;
//PT_CounterEventRead  ptCounterEventRead;

	LONG			pBuffer[5];
	float           pfClock[5];
	ULONG           ulSize;
	
 /*   ע��1721�忨�Ĺ���
 //�򿪰忨
	//-------------------��1721---------------------
    ErrCde = DRV_DeviceOpen(DeviceNum_1721,&DriverHandle_1721);
    if (ErrCde != SUCCESS)
    {
         Status = CARD_STATUS_FAIL_0;	//���°忨״̬
		 strcpy(szErrMsg,"Device 1721 open error !"); //���·��ع�����Ϣ		 

         return ErrCde;
    }
	//1.����1721��ģ�����
	//1.1 �������1:0ͨ��
	cha_sel = CARD_1721_CHL_GAS_1;
	ptAOConfig[cha_sel].chan = cha_sel; //�趨ͨ����
	ptAOConfig[cha_sel].RefSrc = 0;		//ѡȡ�ڲ��ο���ѹ
	ptAOConfig[cha_sel].MaxValue = 5;	//�趨�����ѹ�����ֵ
	ptAOConfig[cha_sel].MinValue = 0;	//�趨�����ѹ����Сֵ
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//����ģ�����
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//���°忨״̬
		 strcpy(szErrMsg,"Device 1721 AO cha-0 config error !"); //���·��ع�����Ϣ		 

		return ErrCde;
	}
		
	//1.2 �������2:1ͨ��
	cha_sel = CARD_1721_CHL_GAS_2;
	ptAOConfig[cha_sel].chan = cha_sel; //�趨ͨ����
	ptAOConfig[cha_sel].RefSrc = 0;		//ѡȡ�ڲ��ο���ѹ
	ptAOConfig[cha_sel].MaxValue = 5;	//�趨�����ѹ�����ֵ
	ptAOConfig[cha_sel].MinValue = 0;	//�趨�����ѹ����Сֵ
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//����ģ�����
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//���°忨״̬
		 strcpy(szErrMsg,"Device 1721 AO cha-1 config error !"); //���·��ع�����Ϣ		 

		return ErrCde;
	}

	//1.3 ת�����:2ͨ��
	cha_sel = CARD_1721_CHL_STR;
	ptAOConfig[cha_sel].chan = cha_sel; //�趨ͨ����
	ptAOConfig[cha_sel].RefSrc = 0;		//ѡȡ�ڲ��ο���ѹ
	ptAOConfig[cha_sel].MaxValue = 5;	//�趨�����ѹ�����ֵ
	ptAOConfig[cha_sel].MinValue = 0;	//�趨�����ѹ����Сֵ
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//����ģ�����
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//���°忨״̬
		 strcpy(szErrMsg,"Device 1721 AO cha-2 config error !"); //���·��ع�����Ϣ		 

		return ErrCde;
	}

	//1.4 �ƶ����:3ͨ��
	cha_sel = CARD_1721_CHL_BRK;
	ptAOConfig[cha_sel].chan = cha_sel; //�趨ͨ����
	ptAOConfig[cha_sel].RefSrc = 0;		//ѡȡ�ڲ��ο���ѹ
	ptAOConfig[cha_sel].MaxValue = 5;	//�趨�����ѹ�����ֵ
	ptAOConfig[cha_sel].MinValue = 0;	//�趨�����ѹ����Сֵ
		
	ErrCde = DRV_AOConfig(DriverHandle_1721,&ptAOConfig[cha_sel]);//����ģ�����
	if ( ErrCde != SUCCESS )
	{
         Status = CARD_STATUS_FAIL_0;	//���°忨״̬
		 strcpy(szErrMsg,"Device 1721 AO cha-3 config error !"); //���·��ع�����Ϣ		 

		return ErrCde;
	}

	//2.�Ը���ģ��ͨ������������㣨������������������������
	//2.�趨����ģ��ͨ�����Ϊ0
	for ( i=0;i<CARD_1721_CHL_NUM;i++ )
	{
		vlt_out[i] = 0;	
		ErrCde = write_1721_AO(i);
	}
*/
	 //-------------------------��1784-------------------------
     ErrCde = DRV_DeviceOpen(DeviceNum_1784,&DriverHandle_1784);
     if (ErrCde != SUCCESS)
     {
         Status = CARD_STATUS_FAIL_1;  //���°忨״̬
		 strcpy(szErrMsg,"Device 1784 open error !"); //���·��ع�����Ϣ		 

         return ErrCde;
     }

	 //-----------------����1784---------------------
	 //1.�������Բ�����ע��ֻ��Ҫ���ò���QEP������1ͨ����2ͨ��������Ҫ���ò�����������������0ͨ����
	 //ÿ��ͨ�����趨ʹ���˲���
	 ulSize = sizeof(LONG) * 5;
	 ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrDigitalFilter, pBuffer, &ulSize);
	 pBuffer[CARD_1784_CHL_CNT] = 0;//0:��ʹ��, 1��ʹ��
 	 pBuffer[CARD_1784_CHL_STR] = 0;//0:��ʹ�ã�1��ʹ�ã�
  	 pBuffer[CARD_1784_CHL_BRK] = 1;//0:��ʹ�ã�1��ʹ�ã�

	 ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrDigitalFilter, pBuffer, ulSize);
	
	//�趨ʱ��Ϊ8MHz
	ulSize = sizeof(float) * 5;
	ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrClockFrequency, pfClock, &ulSize);
	pfClock[CARD_1784_CHL_CNT] = 8000000.0;
	pfClock[CARD_1784_CHL_STR] = 8000000.0;
    pfClock[CARD_1784_CHL_BRK] = 8000000.0;
	ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrClockFrequency, pfClock, ulSize);
					
	//�趨��ֹ����Ϊ0������ֹ��,1(������ֹ����2��������ֹ����3��������������ֹ��
	ulSize = sizeof(LONG) * 5;
	ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrCounterLockControl, pBuffer, &ulSize);
	pBuffer[CARD_1784_CHL_CNT] = 0;
	pBuffer[CARD_1784_CHL_STR] = 0;
	pBuffer[CARD_1784_CHL_BRK] = 0;
	ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrCounterLockControl, pBuffer, ulSize);
					
	// index reset��Ĭ��Ϊ����
	ErrCde = DRV_DeviceGetProperty(DriverHandle_1784, CFG_CntrIndexReset, pBuffer, &ulSize);
	pBuffer[CARD_1784_CHL_CNT] &= ~(0x1 << CARD_1784_CHL_CNT);
	pBuffer[CARD_1784_CHL_STR] &= ~(0x1 << CARD_1784_CHL_STR);
	pBuffer[CARD_1784_CHL_BRK] &= ~(0x1 << CARD_1784_CHL_BRK);
	ErrCde = DRV_DeviceSetProperty(DriverHandle_1784, CFG_CntrIndexReset, pBuffer, ulSize);

	////2.����ͨ��0����ͨ����������ģ����������
	////��������������
	//cha_sel = CARD_1784_CHL_CNT;
	////���ü���������
	//ptQCounterConfig.counter = cha_sel;//����ͨ����
	//ptQCounterConfig.LatchSrc = SWLATCH;//�����ֹ���������û�����Ӹߵ��Ͷ�ȡ������
	//ptQCounterConfig.LatchOverflow = 0;//����������ʱ����ֹ
	//ptQCounterConfig.ResetOnLatch = 1;//��������ֹ��λ
	//ptQCounterConfig.ResetValue = 0;//��λ�������ֵΪ0
	//ErrCde = DRV_QCounterConfig (DriverHandle_1784,&ptQCounterConfig);
	//if ( ErrCde!=SUCCESS )
	//{
	//	Status = CARD_STATUS_FAIL_1;  //���°忨״̬
	//	strcpy(szErrMsg,"Device 1784 QEP-0 Config error !"); //���·��ع�����Ϣ
	//	return ErrCde;
	//}
	////����������
	//ptQCounterStart.counter = cha_sel;//����ͨ����
 //   ptQCounterStart.InputMode = ABPHASEX4;//��������ģʽΪ4��Ƶ��AB����������
 //   ErrCde = DRV_QCounterStart(DriverHandle_1784,(LPT_QCounterStart)&ptQCounterStart);
	//if ( ErrCde!=SUCCESS )
	//{
	//	Status = CARD_STATUS_FAIL_1;  //���°忨״̬
	//	strcpy(szErrMsg,"Device 1784 QEP-0 Start error !"); //���·��ع�����Ϣ
	//	return ErrCde;
	//}

	//3.���ò�����ͨ��1����ͨ������ת��ģ���QEP����
	cha_sel = CARD_1784_CHL_STR;
	//���ü���������
	ptQCounterConfig.counter = cha_sel;//����ͨ����
	ptQCounterConfig.LatchSrc = SWLATCH;//�����ֹ���������û�����Ӹߵ��Ͷ�ȡ������
	ptQCounterConfig.LatchOverflow = 0;//����������ʱ����ֹ
	ptQCounterConfig.ResetOnLatch = 1;//��������ֹ��λ
	ptQCounterConfig.ResetValue = 0;//��λ�������ֵΪ0
	ErrCde = DRV_QCounterConfig (DriverHandle_1784,&ptQCounterConfig);
	if ( ErrCde!=SUCCESS )
	{
		Status = CARD_STATUS_FAIL_1;  //���°忨״̬
		strcpy(szErrMsg,"Device 1784 QEP-1 Config error !"); //���·��ع�����Ϣ
		return ErrCde;
	}
	//����������
	ptQCounterStart.counter = cha_sel;//����ͨ����
    ptQCounterStart.InputMode = ABPHASEX4;//��������ģʽΪ4��Ƶ��AB����������
 //   ErrCde = DRV_QCounterStart(DriverHandle_1784,(LPT_QCounterStart)&ptQCounterStart);
	//if ( ErrCde!=SUCCESS )
	//{
	//	Status = CARD_STATUS_FAIL_1;  //���°忨״̬
	//	strcpy(szErrMsg,"Device 1784 QEP-1 Start error !"); //���·��ع�����Ϣ
	//	return ErrCde;
	//}

	//4.���ò�����ͨ��2����ͨ�������ƶ�ģ���QEP����
	cha_sel = CARD_1784_CHL_BRK;
	//���ü���������
	ptQCounterConfig.counter = cha_sel;//����ͨ����
	ptQCounterConfig.LatchSrc = SWLATCH;//�����ֹ���������û�����Ӹߵ��Ͷ�ȡ������
	ptQCounterConfig.LatchOverflow = 0;//����������ʱ����ֹ
	ptQCounterConfig.ResetOnLatch = 1;//��������ֹ��λ
	ptQCounterConfig.ResetValue = 0;//��λ�������ֵΪ0
	ErrCde = DRV_QCounterConfig (DriverHandle_1784,&ptQCounterConfig);
	if ( ErrCde!=SUCCESS )
	{
		Status = CARD_STATUS_FAIL_1;  //���°忨״̬
		strcpy(szErrMsg,"Device 1784 QEP-2 Config error !"); //���·��ع�����Ϣ
		return ErrCde;
	}
	//����������
	ptQCounterStart.counter = cha_sel;//����ͨ����
    ptQCounterStart.InputMode = ABPHASEX4;//��������ģʽΪ4��Ƶ��AB����������
    ErrCde = DRV_QCounterStart(DriverHandle_1784,(LPT_QCounterStart)&ptQCounterStart);
	if ( ErrCde!=SUCCESS )
	{
		Status = CARD_STATUS_FAIL_1;  //���°忨״̬
		strcpy(szErrMsg,"Device 1784 QEP-2 Start error !"); //���·��ع�����Ϣ
		return ErrCde;
	}

	//5. 0-2ͨ����counter reset���Ը�ͨ���ļ��������г�ʼ���㣨������������������������
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
	//----------------�趨�忨1721���������--------------------
	//3.1 ʹ��ת����
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_STR_EN,1);
	if ( ErrCde!=SUCCESS )
	{
//	 	MessageBox("1721 DO-0 config error");
		strcpy(szErrMsg,"1721 DO-0 config error!"); //���·��ع�����Ϣ

		return ErrCde;
	}

	//3.2 ʹ���ƶ����
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_BRK_EN,1);
	if ( ErrCde!=SUCCESS )
	{
//		MessageBox("1721 DO-4 config error");
		strcpy(szErrMsg,"1721 DO-4 config error!"); //���·��ع�����Ϣ

		return ErrCde;
	}

	//3.3 ʹ��ת�������ƶ�������ƶ��źţ��Ա�����ʧ��
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_STR_BRK,1);
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_BRK_BRK,1);
*/
	
	Status = CARD_STATUS_OPEN; //�趨�忨Ϊ��״̬

	return ErrCde;
}

//////////////////////////////////////////////////////////////////////////
//�رհ忨
//////////////////////////////////////////////////////////////////////////
LRESULT CARD::CARD_OFF(void)
{
//	int i;
/*	
	//1.�رյ��ʹ��
	//�ر�ת����ʹ��
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_STR_EN,0);
	if ( ErrCde!=SUCCESS )
	{
		MessageBox(NULL,"1721 DO-0 write error","",MB_OK);
		return MSG_ERROR;
	}

	//�ر��ƶ����ʹ��
	ErrCde = write_1721_DO(CARD_1721_DO_CHL_BRK_EN,0);
	if ( ErrCde!=SUCCESS )
	{
		MessageBox(NULL,"1721 DO-4 write error","",MB_OK);
		return MSG_ERROR;
	}

	//2.�趨����ģ��ͨ�����Ϊ0��������������������������
	for ( i=0;i<CARD_1721_CHL_NUM;i++ )
	{
		vlt_out[i] = 0;	
		ErrCde = write_1721_AO(i);
	}
	
	
	
	//-------------�ر�1721-----------------
	ErrCde = DRV_DeviceClose( &DriverHandle_1721 ); 
	if ( ErrCde != SUCCESS )
	{
		 return ErrCde;
	}

*/
	//-------------�ر�1784------------------
	//0-2ͨ����counter reset�������������������������������������
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

	Status = CARD_STATUS_CLOSE; //�趨�忨ΪCLOSE״̬

	return ErrCde;
}


//////////////////////////////////////////////////////////////////////////
//����:��ȡ1784�ļ���
//���������cha_sel,ͨ����,0:����PWM��1-ת��QEP��2-�ƶ�QEP;
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
	case CARD_1784_CHL_CNT://��ȡPWM���������
		ptCounterEventRead.counter = CARD_1784_CHL_GAS;//�趨ͨ����
        ptCounterEventRead.overflow = (USHORT *)&gwOverflow;//�趨���״ָ̬�룬1�����
        ptCounterEventRead.count = (ULONG *)&CountRead;//�趨��ȡ������ָ��
		//ע�⣬�˴�û�м���������������Ϊ���صļ���ֵΪULONG�ͣ�ʵ������ʹ���в��������

		//��ȡ����������
		ErrCde = DRV_CounterEventRead(DriverHandle_1784,(LPT_CounterEventRead)&ptCounterEventRead);
		if ( ErrCde != SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-0 read error !"); //���·��ع�����Ϣ
			return ErrCde;
		}
		
		//�������ֵ
		cnt_in[cha_sel] = CountRead;

		//������һ��ֵ
		CountRead_last[cha_sel] = CountRead;

		return SUCCESS;
		
		break;
	case CARD_1784_CHL_STR://��ȡQEP����

		ptQCounterRead.counter = cha_sel;//�趨����ͨ��
		ptQCounterRead.overflow = (USHORT *)&gwOverflow;//�趨���ص����ָ��
		ptQCounterRead.LoCount  = (ULONG *)&CountRead;//�趨���ص�32λ����ָ��
		ptQCounterRead.HiCount  = (ULONG  far *)&ulHiCount;
		
		ErrCde = DRV_QCounterRead(DriverHandle_1784,(LPT_QCounterRead)&ptQCounterRead);
		if ( ErrCde!=SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-1 read error !"); //���·��ع�����Ϣ
			return ErrCde;
		}
		
		//�������ֵ
		if ( CountRead>LONG_MAX )//������Ϊ����
		{
			data = (ULONG_MAX - CountRead) + 1;
			cnt_in[cha_sel] = (int)data;
			cnt_in[cha_sel] *= -1;
		}
		else
		{
			cnt_in[cha_sel] = CountRead;
		}


		//������һ��ֵ
		CountRead_last[cha_sel] = CountRead;
		
		return SUCCESS;

		break;
	case CARD_1784_CHL_BRK://��ȡQEP����
		ptQCounterRead.counter = cha_sel;//�趨����ͨ��
		ptQCounterRead.overflow = (USHORT *)&gwOverflow;//�趨���ص����ָ��
		ptQCounterRead.LoCount  = (ULONG *)&CountRead;//�趨���ص�32λ����ָ��
		ptQCounterRead.HiCount  = (ULONG  far *)&ulHiCount;
		
		ErrCde = DRV_QCounterRead(DriverHandle_1784,(LPT_QCounterRead)&ptQCounterRead);
		if ( ErrCde!=SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-2 read error !"); //���·��ع�����Ϣ
			return ErrCde;
		}
		
		//�������ֵ
		if ( CountRead>LONG_MAX )//������Ϊ����
		{
			data = (ULONG_MAX - CountRead) + 1;
			cnt_in[cha_sel] = (int)data;
			cnt_in[cha_sel] *= -1;
		}
		else
		{
			cnt_in[cha_sel] = CountRead;
		}


		//������һ��ֵ
		CountRead_last[cha_sel] = CountRead;

		return SUCCESS;

		break;
	case 3://��ȡQEP����

		ptQCounterRead.counter = cha_sel;//�趨����ͨ��
		ptQCounterRead.overflow = (USHORT *)&gwOverflow;//�趨���ص����ָ��
		ptQCounterRead.LoCount  = (ULONG *)&CountRead;//�趨���ص�32λ����ָ��
		ptQCounterRead.HiCount  = (ULONG  far *)&ulHiCount;

		ErrCde = DRV_QCounterRead(DriverHandle_1784,(LPT_QCounterRead)&ptQCounterRead);
		if ( ErrCde!=SUCCESS )
		{
			strcpy(szErrMsg,"Device 1784 cha-1 read error !"); //���·��ع�����Ϣ
			return ErrCde;
		}

		//�������ֵ
		if ( CountRead>LONG_MAX )//������Ϊ����
		{
			data = (ULONG_MAX - CountRead) + 1;
			cnt_in[cha_sel] = (int)data;
			cnt_in[cha_sel] *= -1;
		}
		else
		{
			cnt_in[cha_sel] = CountRead;
		}


		//������һ��ֵ
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
//1721���ú���
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
//1721���º����������vlt_cout���
///////////////////////////////////////////////////////////////
LRESULT CARD::write_1721_AO(WORD cha_sel )
{
	//ģ�����δʹ��ʱ��ֻ�������0��ֻ����ʹ��ʱ�������������0��ģ����
	if ( AO_Enable == CARD_1721_AO_DISABLE  )
	{
		vlt_out[cha_sel] = 0;		
	}	
	else
	{
		//����ǰͨ����ģ���������Ϊ������4.9V
		if ( vlt_out[cha_sel]>4.9 )
		{
			vlt_out[cha_sel] = (float)4.9;
		}
		else if ( vlt_out[cha_sel]<0.01 ) //������0.01
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
//����1721���������
//�������Ϊͨ���ż���ߵ͵�ƽ
///////////////////////////////////////////////////
LRESULT CARD::write_1721_DO(WORD cha_sl, WORD cmd_en)
{
    WORD data;

	//���ȶ�ȡ��ǰIO���
	/*
	ptDioGetCurrentDOByte.port  = 0;//1721�����������0��1�����˿�16λ������ֱ��ӦB0-B7,B8-B15λ
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
	case CARD_1721_DO_CHL_STR_EN://ת����ʹ��
		if ( cmd_en ) //ʹ��
		{
			data |= 1<<cha_sl;
		}
		else //ȡ��
		{
			data &= (~(1<<cha_sl));
		}

		break;
	case CARD_1721_DO_CHL_STR_REV://ת������ת
		if ( cmd_en ) //ʹ��
		{
			data |= 1<<cha_sl;
		}
		else //ȡ��
		{
			data &= (~(1<<cha_sl));
		}
		break;
	case CARD_1721_DO_CHL_STR_BRK://ת�����ƶ�
		if ( cmd_en ) //ʹ��
		{
			data |= 1<<cha_sl;
		}
		else //ȡ��
		{
			data &= (~(1<<cha_sl));

		}		
		break;
	case CARD_1721_DO_CHL_BRK_EN://�ƶ����ʹ��
		if ( cmd_en ) //ʹ��
		{
			data |= 1<<cha_sl;
		}
		else //ȡ��
		{
			data &= (~(1<<cha_sl));
		}		
		break;
	case CARD_1721_DO_CHL_BRK_REV://�ƶ������ת
		if ( cmd_en ) //ʹ��
		{
			data |= 1<<cha_sl;
		}
		else //ȡ��
		{
			data &= (~(1<<cha_sl));
		}
		break;
	case CARD_1721_DO_CHL_BRK_BRK://�ƶ�����ƶ�
		if ( cmd_en ) //ʹ��
		{
			data |= 1<<cha_sl;
		}
		else //ȡ��
		{
			data &= (~(1<<cha_sl));
		}
		break;
	default:
		break;
	}
	
	ptDioWritePortByte.port  = CARD_1721_DO_PORT;//1721�����������0��1�����˿�16λ������ֱ��ӦB0-B7,B8-B15λ
    ptDioWritePortByte.mask  = 0xff;
    ptDioWritePortByte.state = data;

    ErrCde = DRV_DioWritePortByte(DriverHandle_1721, (LPT_DioWritePortByte)&ptDioWritePortByte);
	if ( ErrCde != SUCCESS )
	{
		strcpy(szErrMsg,"Device 1721 DO write error !"); //���·��ع�����Ϣ	
		return ErrCde;
	}
	//���µ�ǰ�������״̬
	DoState = data;

	return ErrCde;
}

//����1721��DI�����ȡ
//���ݶ�ȡֵ���������뿪�ص�״̬
LRESULT CARD::read_1721_DI(void)
{
	gwChannel = CARD_1721_DI_PORT;
	ptDioReadPortByte.port = gwChannel;
    ptDioReadPortByte.value = (USHORT far *)&DiState;

	ErrCde = DRV_DioReadPortByte(DriverHandle_1721, (LPT_DioReadPortByte)&ptDioReadPortByte);
    if ( ErrCde != SUCCESS )
    {
		strcpy(szErrMsg,"Device 1721 DI read error !"); //���·��ع�����Ϣ	
		return ErrCde;
    }

	//�ж��Զ���ʻ�����Ƿ�պϣ��պ�Ϊ�ߵ�ƽ
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_AUTOSEL) ) == 0 )
	{
		autosel_sw = 0;//���ضϿ�
	}
	else
	{
		autosel_sw = 1;//���رպ�
	}

	//�жϽ������򿪹��Ƿ�պϣ��պ�Ϊ�ߵ�ƽ
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_EMGSTOP) ) == 0 )
	{
		emgstop_sw = 0;//���ضϿ�
	}
	else
	{
		emgstop_sw = 1;//���رպ�
	}

	
	//�ж�ת������λ�����Ƿ�պϣ��պ�Ϊ�ߵ�ƽ
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_STRPLIM) ) == 0 )
	{
		strplim_sw = 0;//���ضϿ�		
	}
	else
	{
		strplim_sw = 1;//���رպ�
	}

	//�ж�ת����λ�����Ƿ�պϣ��պ�Ϊ�ߵ�ƽ
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_STRNLIM) ) == 0 )
	{
		strnlim_sw = 0;//���ضϿ�
	}
	else
	{
		strnlim_sw = 1;//���رպ�
	}

	//�ж��ƶ���λ�����Ƿ�պϣ��պ�Ϊ�ߵ�ƽ
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_BRKPLIM) ) == 0 )
	{
		brkplim_sw = 0;//���ضϿ�
	}
	else
	{
		brkplim_sw = 1;//���رպ�
	}

	//�ж��ƶ���㿪���Ƿ�պϣ��պ�Ϊ�ߵ�ƽ
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_BRKZERO) ) == 0 )
	{
		brkzero_sw = 0;//���ضϿ�
	}
	else
	{
		brkzero_sw = 1;//���رպ�
	}
	

	//ͨ����ѯ�¼����ж�ת����㿪���Ƿ������պ�
	if ( ( (DiState<<8) & (1<<CARD_1721_DI_CHL_STRNLIM) ) == 0 )
	{
		strzero_sw = 0;//���ضϿ�
	}
	else
	{
		strzero_sw = 1;//���رպ�
	}


	return ErrCde;

}

///////////////////////////////////////////////////////////////////
//1784��λ���������������
///////////////////////////////////////////////////////////////////
LRESULT CARD::reset_1784_CNT(WORD chl)
{
	ErrCde = DRV_CounterReset(DriverHandle_1784,(LPARAM)chl);

	return ErrCde;
}

////////////////////////////////////////////////////////////////////
//����1784����ת�������¼�ʹ��
////////////////////////////////////////////////////////////////////
LRESULT CARD::config_1784_EVENT_strori_start(void)
{
 	PT_EnableEvent	 EventSetting;
	
	EventSetting.EventType = ADS_EVT_DI_INTERRUPT1;
	EventSetting.Enabled   = TRUE;
	EventSetting.Count     = 1;
	ErrCde = DRV_EnableEvent(DriverHandle_1784, &EventSetting );

	strzero_evt_count = 0;//�����¼�����

	return ErrCde;

}

/////////////////////////////////////////////////////////////////////
//ֹͣ1784����ת�������¼�ʹ��
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
//��ѯ1784����ת�������¼�
///////////////////////////////////////////////////////////////////////
LRESULT CARD::read_1784_EVENT_strori(void)
{
	PT_CheckEvent	ptCheckEvent;
	USHORT	usEventType;

	ptCheckEvent.EventType = &usEventType;
	ptCheckEvent.Milliseconds = 60000;//�¼��ĳ�ʱ����Ϊ60s

	ErrCde = DRV_CheckEvent(DriverHandle_1784, &ptCheckEvent );
	if ( ErrCde == SUCCESS)
	{
		if( usEventType == ADS_EVT_DI_INTERRUPT1 )
		{
			strzero_evt_count++;//�����¼�����
		}
	}

	return ErrCde;

}


//////////////////////////////////////////////////////////////////////
////��ȡ1784������������
//////////////////////////////////////////////////////////////////////
LRESULT CARD::read_1784_DI(void)
{
	ptDioReadPortByte.port = 0;
    ptDioReadPortByte.value = (USHORT far *)&DiState;

	ErrCde = DRV_DioReadPortByte(DriverHandle_1784, (LPT_DioReadPortByte)&ptDioReadPortByte);
    if ( ErrCde != SUCCESS )
    {
		strcpy(szErrMsg,"Device 1784 DI read error !"); //���·��ع�����Ϣ	
		return ErrCde;
    }

	//�ж�ת����㿪���Ƿ�պϣ��պ�Ϊ�ߵ�ƽ
	if ( ( (DiState) & (1<<CARD_1784_DI_CHL_STRZERO) ) == 0 )
	{
		strzero_sw = 1;//���ضϿ�
	}
	else
	{
		strzero_sw = 0;//���رպ�
	}

	return ErrCde;
}
