//����忨ģ��
#ifndef _CARD
#define _CARD

#include "datatype.h"
#include "C:\Program Files\Advantech\Adsapi\Include\Driver.h" 

// ------------------------------------------------------------------------
//	��������
// ------------------------------------------------------------------------
#define     MAX_DEVICES     5

//����忨��״̬
#define CARD_STATUS_CLOSE		0   //��δ��
#define CARD_STATUS_FAIL_0		1   //1721��ʧ��
#define CARD_STATUS_FAIL_1		2   //1784��ʧ��
#define CARD_STATUS_OPEN        3   //���Ѵ�

//ģ�����ʹ�ܱ�־
#define CARD_1721_AO_DISABLE    0   //ģ��������ã����Ϊ0
#define CARD_1721_AO_ENABLE     1   //ģ�����ʹ�ܣ����Ϊ��0ֵ

//ģ�����ͨ����
#define CARD_1721_CHL_NUM       4   //1721ģ�����ͨ����
#define CARD_1721_CHL_GAS_1     0   //����0~5V���
#define CARD_1721_CHL_GAS_2     1   //����0~2.5V���
#define CARD_1721_CHL_STR       2   //ת��0-5V���
#define CARD_1721_CHL_BRK       3   //�ƶ�0-5V���

//�������ͨ����
//1721
#define CARD_1721_DO_PORT         0  //DO����˿ں�
#define CARD_1721_DO_CHL_NUM      6  //1721 DO���ͨ����
#define CARD_1721_DO_CHL_STR_EN   0  //ת����ʹ�ܣ��ߵ�ƽ��Ч��
#define CARD_1721_DO_CHL_STR_REV  1  //ת������ת������
#define CARD_1721_DO_CHL_STR_BRK  2  //ת�����ƶ����ߵ�ƽ��Ч��
#define CARD_1721_DO_CHL_BRK_EN   3  //�ƶ����ʹ�ܣ��ߵ�ƽ��Ч��
#define CARD_1721_DO_CHL_BRK_REV  4  //�ƶ������ת������
#define CARD_1721_DO_CHL_BRK_BRK  5  //�ƶ�����ƶ����ߵ�ƽ��Ч��

//��������ͨ����
//1721
#define CARD_1721_DI_PORT          1   //DI����˿ں�
#define CARD_1721_DI_CHL_NUM       6   //1721 DI����ͨ����
#define CARD_1721_DI_CHL_AUTOSEL   8   //�Զ���ʻ���أ��ߵ�ƽ��Ч��
#define CARD_1721_DI_CHL_EMGSTOP   9   //��ͣ����
#define CARD_1721_DI_CHL_STRPLIM   10  //ת����������λ���ߵ�ƽ��Ч��
#define CARD_1721_DI_CHL_STRNLIM   11  //ת����������λ���ߵ�ƽ��Ч��
#define CARD_1721_DI_CHL_BRKPLIM   12  //�ƶ������λ���ߵ�ƽ��Ч��
#define CARD_1721_DI_CHL_BRKZERO   13  //�ƶ������㣨�ߵ�ƽ��Ч��
//1784
#define CARD_1784_DI_CHL_STRZERO    1  //ת������㣨�ߵ�ƽ��Ч��



//������QEP����ͨ����
#define CARD_1784_CHL_NUM       3   //1784����ͨ����
#define CARD_1784_CHL_CNT       0   //���ڼ�����ͨ��
#define CARD_1784_CHL_GAS       0   //��������ͨ���ţ�����/�������������̶�Ϊ0����������
#define CARD_1784_CHL_BRK       1   //�ƶ����QEP����ͨ���ţ�QEP����
#define CARD_1784_CHL_STR       2   //ת����QEP����ͨ���ţ�QEP����



//�忨ģ�鶨��
class CARD {
public:
	//ģ��Ĺ��캯�������ڲ��������г�ʼ��
	CARD();

    LRESULT CARD_ON(void);//�򿪰忨
	LRESULT CARD_OFF(void);//�رհ忨
	

	//1784���ú���
	//LRESULT config_1784_CNT(WORD chl);
	LRESULT config_1784_EVENT_strori_start();//����1784����ת������¼�
	LRESULT config_1784_EVENT_strori_end();//ֹͣ1784����ת������¼�

	//1784���º����������cnt_in�ж�ȡ
	LRESULT read_1784_CNT(WORD chl);
	LRESULT read_1784_EVENT_strori();//��ȡ1784����ת������¼�
	LRESULT read_1784_DI();//��ȡ1784������������

	//1721���ú���
//	LRESULT config_1721_AO(WORD chl);
//	LRESULT config_1721_DO(void);

	//1721���º����������vlt_cout���
	LRESULT write_1721_AO(WORD chl);
    LRESULT write_1721_DO(WORD chl, WORD cmd_en);
	LRESULT read_1721_DI(void);

	//1784��λ���������������
	LRESULT reset_1784_CNT(WORD chl);

public:
	float       vlt_out[CARD_1721_CHL_NUM];     //INPUT�����������ѹ�������ͨ����һ��
	long        cnt_in[CARD_1784_CHL_NUM];		//OUTPUT�����յ���������������ͨ����һ��

	WORD        autosel_sw;                     //OUTPUT���ֶ�/�Զ�����״̬
	WORD        emgstop_sw;                     //OUTPUT����ͣ����״̬
	WORD        strplim_sw;                       //OUTPUT��ת��������λ״̬
	WORD        strnlim_sw;                       //OUTPUT��ת������λ״̬
	WORD        strzero_sw;                       //OUTPUT��ת�����״̬
	WORD        brkplim_sw;                       //OUTPUT���ƶ�������λ״̬
	WORD        brkzero_sw;                       //OUTPUT���ƶ����״̬

	
	WORD        autosel_count;                     //STATUS���ֶ�/�Զ�����״̬��ȡ����
	WORD        emgstop_count;                     //STATUS����ͣ����״̬��ȡ����
	WORD        strplim_count;                       //STATUS��ת��������λ״̬��ȡ����
	WORD        strnlim_count;                       //STATUS��ת������λ״̬��ȡ����
	WORD        strzero_count;                        //STATUS��ת�����״̬�Ķ�ȡ����
	WORD        strzero_evt_count;                   //STATUS��ת������¼��Ķ�ȡ����
	WORD        brkplim_count;                       //STATUS���ƶ�������λ״̬��ȡ����
	WORD        brknlim_count;                       //STATUS���ƶ�������λ״̬��ȡ����

	
	USHORT      DiState;                        //STATUS����������״̬��ÿһλ��Ӧһ��ͨ����״̬
	WORD        DoState;						//STATUS,�������״̬��ÿһλ��Ӧһ��ͨ����״̬
	WORD        AO_Enable;						//STATUS,ģ�����ʹ�ܣ�0�����ã�1��ʹ�ܣ�
	
	WORD        Status;                 //STATUS���忨״̬��0-δ�򿪣�1-1721δ�򿪣�2-1784δ�򿪣�3-ȫ����
	

	LONG        DriverHandle_1721;			//STATUS,1721�豸���
	LONG        DriverHandle_1784;          //STATUS,1784�豸���

	ULONG       DeviceNum_1721;             //CONFIG,1721�豸�ţ�Ĭ��Ϊ0
    ULONG       DeviceNum_1784;             //CONFIG,1784�豸�ţ�Ĭ��Ϊ1

	


	char        szErrMsg[80];           //STATUS,Use for MESSAGEBOX function
	LRESULT     ErrCde;                 //STATUS, Return error code

	USHORT      usChan0;	            //STATUS,1721ͨ���ţ�0-����5V�����1-����2.5V�����2-ת�������3-�ƶ����
	USHORT      usChan1;		        //STATUS,1784ͨ���ţ�0-����PWM������2-ת����QEP������3-�ƶ����QEP����

	WORD        dir_str_type;           //CONFIG,�趨ת�����������ͣ�0��Ϊ������1������ʱʹ�ܷ�ת��ƽ
	WORD        dir_brk_type;           //CONFIG,�趨�ƶ�����������ͣ�0��Ϊ������1������ʱʹ�ܷ�ת��ƽ

	
	ULONG					CountRead_last[CARD_1784_CHL_NUM];//STATUS,��һ�ζ�ȡ�ļ�����ֵ

	
	//1721�忨DLL��ر���
	USHORT      gwChannel;                      // input channel

	PT_AOConfig			ptAOConfig[CARD_1721_CHL_NUM];		//CONFIG,1721���ò���
	PT_AOVoltageOut		ptAOVoltageOut[CARD_1721_CHL_NUM];	//STATUS,1721�����ѹ

	PT_DioWritePortByte	ptDioWritePortByte;                 //STATUS,1721�������
	PT_DioReadPortByte	ptDioReadPortByte;                  //STATUS��1721���������ȡ
	PT_DioGetCurrentDOByte ptDioGetCurrentDOByte;
	PT_EnableEvent		EventSetting;

	//1784�忨DLL��ر���
	//PT_QCounterConfig		ptQCounterConfig[CARD_1784_CHL_NUM];		//STATUS,1784 QEP
	//PT_QCounterConfigSys	ptQCounterConfigSys[CARD_1784_CHL_NUM];	//STATUS,
	//PT_QCounterStart		ptQCounterStart[CARD_1784_CHL_NUM];			//STATUS,
	//PT_QCounterRead			ptQCounterRead[CARD_1784_CHL_NUM];			//STATUS,

	//PT_CounterEventStart	ptCounterEventStart[CARD_1784_CHL_NUM];//1784 COUNTER
	//PT_CounterEventRead		ptCounterEventRead[CARD_1784_CHL_NUM];
	USHORT					gwOverflow[CARD_1784_CHL_NUM];//���״̬

};


#endif