/*******************************************************

		���ļ����ڶ����������͡��ֳ�������ȡֵ��Χ

********************************************************/
#ifndef _DATATYPE
#define _DATATYPE

//�����ֳ�
typedef unsigned long	DWORD;	//�޷���˫��
typedef unsigned short 	WORD;	//�޷��ŵ���;
typedef long			SDWORD;	//�з�����˫��
typedef short			SWORD;	//�з����͵���

//Q15��ʾ�ĳ���
#define Q15_SQRT_2   46341     //(2)^(1/2)��Q15��ʾ,������Ϊ-1.1e-6.
#define Q15_1        32768     //1��Q15��ʾ  
#define Q15_1_APP    32767     //1��Q15��ʾ��Ϊ����int���������������Ϊ��ֵ  
#define Q15_1DIV6    5461      //1/6,������Ϊ-6.1e-5.
#define Q15_1DIV10   3276      //1/10,������Ϊ-2.4e-4.
#define Q15_1DIV100  327       //1/100,������Ϊ-2.1e-3.
#define Q15_1DIV120  273       //1/120,������-2.4e-4.

//������Ϣ
#define MSG_SUCCESS         0   //�ɹ����
#define MSG_ERROR           1   //���ֹ���


#define CTRLREG_INTFRE		    100			//���ƻ����ж�Ƶ�ʣ���λHz
#define CTRLREG_INTPRD          10          //���ƻ����ж����ڣ���λms

/************************************************************
				ϵͳ����
************************************************************/

//ϵͳ�������鳤��
#define DEBUG_ARRAY_SIZE   18000  //��ɱ���3���ӵ�����


/************************************************************
				���ſ��Ƴ�����������ֵ��Χ
************************************************************/
#define GASREG_SPEED_MAX           32       //�������ֵ,m/s
#define GASREG_SPEED_MIN            0       //������Сֵ,m/s
#define GASREG_PIGAIN_MAX       10000       //PI��������P������I��������ֵ
#define GASREG_UOUT_SIZE        12          //PI�����������λ��
#define GASREG_UOUT_PMAX		 4.99          //PI������������������ֵ����
#define GASREG_UOUT_NMAX		 0          //PI����������ĸ������ֵ����
#define GASREG_INTFRE           10          //���ſ��ƻ���Ƶ�ʣ���λHz
#define GASREG_INTPRD          100          //���ſ��ƻ������ڣ���λms
#define GASREG_STALL_COUNT_MAX    50        //����Ķ�ת�������ֵ����λΪ��������

/************************************************************
				������Ƴ�����������ֵ��Χ
************************************************************/
#define STEERREG_ANGLE_RATIO            54                        // ת����ٱȣ�������Ƕ�/�����̽Ƕ�
#define STEERREG_ANGLE_MAX       25000//(1.5*360*STEERREG_ANGLE_RATIO)   //ת�����ֵ����
#define STEERREG_ANGLE_MIN      -25000//(-1.5*360*STEERREG_ANGLE_RATIO)  //ת����Сֵ����
#define STEERREG_PIGAIN_MAX    10000          //PI��������P������I��������ֵΪ2
#define STEERREG_UOUT_PMAX      4.99          //λ�û�PI������������������ֵ����
#define STEERREG_UOUT_NMAX     -4.99          //λ�û�PI����������ĸ������ֵ����
#define STEERREG_INTFRE       (CTRLREG_INTFRE)  //ת��λ�ÿ��ƻ���Ƶ�ʣ���λHz
#define STEERREG_INTPRD       (CTRLREG_INTPRD)  //ת��λ�ÿ��ƻ������ڣ���λms
#define STEERREG_VEL_MAX      ((3000.0*360.0)/(STEERREG_INTFRE*60))
#define STEERREG_VEL_MIN             4     //ϵͳ��С�ٶȣ���λ��/��������
#define STEERREG_STALL_COUNT_MAX    20      //����Ķ�ת�������ֵ����ת��ʱΪ200ms��
#define STEERREG_INTP_ENDREGDIS        20    //Ԥ�����յ����پ��룬��λΪ��
#define STEERREG_ERROR_MAX             200    //����������ֵ����λΪ��
#define STEERREG_ACC_VELERR_MAX          4    //����ʱ���������ٶ����
#define STEERREG_DEC_VELERR_MAX          4    //����ʱ���������ٶ����

/************************************************************
				�ƶ����Ƴ�����������ֵ��Χ
************************************************************/
#define BRAKEREG_ANGLE_RATIO            60       //���ٱȣ�������Ƕ�/ִ�����Ƕ�
#define BRAKEREG_ANGLE_MAX          (180*BRAKEREG_ANGLE_RATIO)       //ת�����ֵ����
#define BRAKEREG_ANGLE_MIN         (-180*BRAKEREG_ANGLE_RATIO)       //ת����Сֵ����
#define BRAKEREG_PIGAIN_MAX    10000          //PI��������P������I��������ֵΪ2
#define BRAKEREG_UOUT_PMAX      4.99          //λ�û�PI������������������ֵ����
#define BRAKEREG_UOUT_NMAX     -4.99          //λ�û�PI����������ĸ������ֵ����
#define BRAKEREG_INTFRE       (CTRLREG_INTFRE)  //ת��λ�ÿ��ƻ���Ƶ�ʣ���λHz
#define BRAKEREG_INTPRD       (CTRLREG_INTPRD)  //ת��λ�ÿ��ƻ������ڣ���λms
#define BRAKEREG_VEL_MAX      ((3000.0*360.0)/(BRAKEREG_INTFRE*60))
#define BRAKEREG_VEL_MIN             4     //ϵͳ��С�ٶȣ���λ��/��������
#define BRAKEREG_STALL_COUNT_MAX    40      //����Ķ�ת�������ֵ����ת��ʱΪ200ms��
#define BRAKEREG_INTP_ENDREGDIS        60    //Ԥ�����յ����پ��룬��λΪ��
#define BRAKEREG_ERROR_MAX             720    //����������ֵ����λΪ��
#define BRAKEREG_ACC_VELERR_MAX          4    //����ʱ���������ٶ����
#define BRAKEREG_DEC_VELERR_MAX          4    //����ʱ���������ٶ����

//ģ�鳣������
#define PIREG_SAT_POS  1   //���򱥺ͱ�־
#define PIREG_SAT_NEG  -1  //���򱥺ͱ�־
#define PIREG_SAT_NON  0   //δ���ͱ�־


/*************************************************************

                     �岹��س����궨��

**************************************************************/
//�岹ָ��ִ��״̬�궨��
//#define INTP_STATUS_END		    0x0000		 //����˶�
//#define INTP_STATUS_STA		    0x0001		 //��ʼ״̬
//#define INTP_STATUS_ERROR       0x0002       //�岹ָ�����
//#define INTP_STATUS_WORK        0x0004       //ִ��״̬

//�Ӽ���״̬�궨��
#define INTP_ACCSTAT_END        1000       //���״̬
#define INTP_ACCSTAT_STA        1001       //��ʼ״̬
#define INTP_ACCSTAT_BREG       1002       //��ʼ����
#define INTP_ACCSTAT_ACC	    1003	   //����״̬
#define INTP_ACCSTAT_REG	    1004	   //����״̬
#define INTP_ACCSTAT_DEC	    1005	   //����״̬
#define INTP_ACCSTAT_EREG       1006       //�յ�����
#define INTP_ACCSTAT_REVDEC     1007       //�������״̬
#define INTP_ACCSTAT_ORIGIN     1008       //����״̬
#define INTP_ACCSTAT_REST       1009       //��ֹ״̬

#define INTP_MAXFRERATIO         2           //����һ����ɵı���

#define VK_OEM_PLUS             0x00BB       //+�����
#define VK_OEM_MINUS            0x00BD       //-�����
#define VK_LEFT_ARROW           0x0025       //<-���ͷ�����
#define VK_UP_ARROW             0x0026       //�ϼ�ͷ�����
#define VK_RIGHT_ARROW          0x0027       //->�Ҽ�ͷ�����
#define VK_Z                    0x005A       //Z�����
#define VK_X                    0x0058       //X�����
#define VK_C                    0x0043       //C�����
#define VK_F                    0x0046       //F�����
#define VK_J                    0x004A       //J�����



//�Զ����һЩС����
void CopyWORD(WORD* wp1,WORD* wp2);//���ָ�������
void CopyDWORD(DWORD* wp1,DWORD* wp2);//��˫�ָ�������


#endif
