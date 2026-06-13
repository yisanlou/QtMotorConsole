#pragma once
#include "string.h"

#include <stddef.h>

#pragma pack(push)
#pragma pack(2)

#define MC_API  extern "C" __declspec(dllimport)

#define MAX_MACRO_CHAR_LENGTH (128)

typedef void (*GAS_IOCallBackFun)(unsigned long,unsigned long);

//函数执行返回值
#define MC_COM_SUCCESS			        (0)	//执行成功
#define MC_COM_ERR_EXEC_FAIL			(1)	//执行失败
#define MC_COM_ERR_LICENSE_WRONG		(2)	//license不支持
#define MC_COM_ERR_DATA_WORRY			(7)	//参数错误
#define MC_COM_ERR_SEND					(-1)//发送失败
#define MC_COM_ERR_CARD_OPEN_FAIL		(-6)//打开失败
#define MC_COM_ERR_TIME_OUT				(-7)//无响应
#define MC_COM_ERR_COM_OPEN_FAIL        (-8)//打开串口失败

//轴状态位定义
#define AXIS_STATUS_ESTOP               (0x00000001)	//急停
#define AXIS_STATUS_SV_ALARM            (0x00000002)	//驱动器报警标志（1-伺服有报警，0-伺服无报警）
#define AXIS_STATUS_POS_SOFT_LIMIT      (0x00000004)	//正软限位触发标志（规划位置大于正向软限位时置1）
#define AXIS_STATUS_NEG_SOFT_LIMIT      (0x00000008)	//负软位触发标志（规划位置小于负向软限位时置1）
#define	AXIS_STATUS_FOLLOW_ERR          (0x00000010)	//轴规划位置和实际位置的误差大于设定极限时置1┅
#define AXIS_STATUS_POS_HARD_LIMIT      (0x00000020)	//正硬限位触发标志（正限位开关电平状态为限位触发电平时置1）
#define AXIS_STATUS_NEG_HARD_LIMIT      (0x00000040)	//负硬限位触发标志（负限位开关电平状态为限位触发电平时置1）
#define AXIS_STATUS_IO_SMS_STOP         (0x00000080)	//IO平滑停止触发标志（正限位开关电平状态为限位触发电平时置1，规划位置大于正向软限位时置1）
#define AXIS_STATUS_IO_EMG_STOP         (0x00000100)	//IO紧急停止触发标志（负限位开关电平状态为限位触发电平时置1，规划位置小于负向软限位时置1）
#define AXIS_STATUS_ENABLE              (0x00000200)	//电机使能标志
#define	AXIS_STATUS_RUNNING             (0x00000400)	//规划运动标志，规划器运动时置1
#define AXIS_STATUS_ARRIVE              (0x00000800)	//电机到位（规划器静止，规划位置和实际位置的误差小于设定误差带，并在误差带内保持设定时间后，置起到位标志）
#define AXIS_STATUS_HOME_RUNNING        (0x00001000)	//正在回零
#define AXIS_STATUS_HOME_SUCESS	        (0x00002000)	//回零成功
#define AXIS_STATUS_HOME_SWITCH			(0x00004000)	//零位信号
#define AXIS_STATUS_INDEX				(0x00008000)    //z索引信号
#define AXIS_STATUS_GEAR_START  		(0x00010000)    //电子齿轮开始啮合
#define AXIS_STATUS_GEAR_FINISH         (0x00020000)    //电子齿轮完成啮合
#define AXIS_STATUS_HOME_FAIL  	        (0x00400000)	//回零失败

//坐标系状态位定义
#define	CRDSYS_STATUS_PROG_RUN						(0x00000001)	//启动中
#define	CRDSYS_STATUS_PROG_STOP						(0x00000002)	//平滑停止中
#define	CRDSYS_STATUS_PROG_ESTOP					(0x00000004)	//紧急停止中

#define	CRDSYS_STATUS_FIFO_FINISH_0		    (0x00000010)	//板卡FIFO-0数据已执行完毕的状态位
#define	CRDSYS_STATUS_FIFO_FINISH_1		    (0x00000020)	//板卡FIFO-1数据已执行完毕的状态位

//输入IO类型宏定义
#define MC_LIMIT_POSITIVE               0
#define MC_LIMIT_NEGATIVE               1
#define MC_ALARM                        2
#define MC_HOME                         3
#define MC_GPI                          4
#define MC_ARRIVE                       5
#define MC_IP_SWITCH                    6
#define MC_MPG                          7

//输出IO类型宏定义
#define MC_ENABLE                       10
#define MC_CLEAR                        11
#define MC_GPO                          12


//高速捕获输入类型宏定义
#define CAPTURE_HOME                    1//HOME捕获
#define CAPTURE_INDEX                   2//INDEX捕获
#define CAPTURE_PROBE1                  3//探针捕获
#define CAPTURE_PROBE2                  4
#define CAPTURE_X0                      100
#define CAPTURE_X1                      101
#define CAPTURE_X2                      102
#define CAPTURE_X3                      103
#define CAPTURE_X4                      104
#define CAPTURE_X5                      105
#define CAPTURE_X6                      106
#define CAPTURE_X7                      107
#define CAPTURE_X8                      108
#define CAPTURE_X9                      109
#define CAPTURE_X10                     110
#define CAPTURE_X11                     111
#define CAPTURE_X12                     112
#define CAPTURE_X13                     113
#define CAPTURE_X14                     114
#define CAPTURE_X15                     115

#define CAPTURE_HOME_STOP               11//HOME捕获，并自动停止
#define CAPTURE_INDEX_STOP              12//INDEX捕获，并自动停止
#define CAPTURE_PROBE1_STOP             13//探针捕获，并自动停止
#define CAPTURE_PROBE2_STOP             14
#define CAPTURE_X0_STOP                 200
#define CAPTURE_X1_STOP                 201
#define CAPTURE_X2_STOP                 202
#define CAPTURE_X3_STOP                 203
#define CAPTURE_X4_STOP                 204
#define CAPTURE_X5_STOP                 205
#define CAPTURE_X6_STOP                 206
#define CAPTURE_X7_STOP                 207
#define CAPTURE_X8_STOP                 208
#define CAPTURE_X9_STOP                 209
#define CAPTURE_X10_STOP                210
#define CAPTURE_X11_STOP                211
#define CAPTURE_X12_STOP                212
#define CAPTURE_X13_STOP                213
#define CAPTURE_X14_STOP                214
#define CAPTURE_X15_STOP                215

//PT模式宏定义
#define PT_MODE_STATIC                  0
#define PT_MODE_DYNAMIC                 1

#define PT_SEGMENT_NORMAL               0
#define PT_SEGMENT_EVEN                 1
#define PT_SEGMENT_STOP                 2

#define GEAR_MASTER_ENCODER             1//电子齿轮，跟随编码器
#define GEAR_MASTER_PROFILE             2//电子齿轮，跟随规划值(脉冲个数)
#define GEAR_MASTER_AXIS                3//保留


//电子齿轮启动事件定义
#define GEAR_EVENT_IMMED                1//立即启动电子齿轮
#define GEAR_EVENT_BIG_EQU              2//主轴规划或者编码器位置大于等于指定数值时启动电子齿轮
#define GEAR_EVENT_SMALL_EQU            3//主轴规划或者编码器位置小于等于指定数值时启动电子齿轮
#define GEAR_EVENT_IO_ON                4//指定IO为ON时启动电子齿轮
#define GEAR_EVENT_IO_OFF               5//指定IO为OFF时启动电子齿轮

#define CAM_EVENT_IMMED                1
#define CAM_EVENT_BIG_EQU              2
#define CAM_EVENT_SMALL_EQU            3
#define CAM_EVENT_IO_ON                4
#define CAM_EVENT_IO_OFF               5

#define FROCAST_LEN (200)                     //前瞻缓冲区深度

#define INTERPOLATION_AXIS_MAX          6
#define CRD_FIFO_MAX                    4096
#define CRD_MAX                         2


//点位模式参数结构体
typedef struct TrapPrm
{
	double acc;//加速度
	double dec;//减速度
	double velStart;//起始速度
	short  smoothTime;//平滑时间
}TTrapPrm;

//JOG模式参数结构体
typedef struct JogPrm
{
	double dAcc;
	double dDec;
	double dSmooth;
}TJogPrm;

//插补数据状态结构体
typedef struct _CrdDataState{
	double dLength[8];             //各个轴向分量的长度
	double dSynLength;                          //本段插补数据合成长度
	double dEndSpeed;                           //本段插补终点速度
}TCrdDataState;

//坐标系参数结构体
typedef struct _CrdPrm
{
    short dimension;                              // 坐标系维数
    short profile[8];                      // 关联profile和坐标轴(从1开始)
    double synVelMax;                             // 最大合成速度
    double synAccMax;                             // 最大合成加速度
    short evenTime;                               // 最小匀速时间
    short setOriginFlag;                          // 设置原点坐标值标志,0:默认当前规划位置为原点位置;1:用户指定原点位置
    long originPos[8];                     // 用户指定的原点位置
}TCrdPrm;

//坐标系参数结构体
typedef struct _CrdPrmEx
{
    short dimension;                              // 坐标系维数
    short profile[32];                             // 关联profile和坐标轴(从1开始)
    double synVelMax;                             // 最大合成速度
    double synAccMax;                             // 最大合成加速度
    short evenTime;                               // 最小匀速时间
    short setOriginFlag;                          // 设置原点坐标值标志,0:默认当前规划位置为原点位置;1:用户指定原点位置
    long originPos[32];                            // 用户指定的原点位置
}TCrdPrmEx;

//命令类型
enum _CMD_TYPE
{
	//命令类型
	CMD_G00=1,		//快速定位
	CMD_G01,		//直线插补
	CMD_G02,		//顺圆弧插补
	CMD_G03,		//逆圆弧插补
	CMD_G04,		//延时,G04 P1000是暂停1秒(单位为ms),G04 X2.0是暂停2秒
	CMD_G05,		//设置自定义插补段段号
	CMD_G54,
	CMD_HELIX_G02,		//顺圆弧螺旋插补
	CMD_HELIX_G03,		//逆圆弧螺旋插补
	CMD_CRD_PRM,        //建立坐标系

	CMD_M00 = 11,        //暂停
	CMD_M30,        //结束
	CMD_M31,        //切换到XY1Z坐标系
	CMD_M32,        //切换到XY2Z坐标系
	CMD_M99,        //循环

	CMD_SET_IO = 101,     //设置IO
	CMD_WAIT_IO,           //等待IO
	CMD_BUFFER_MOVE_SET_POS,      //CMD_BUFFER_MOVE_SET_POS
	CMD_BUFFER_MOVE_SET_VEL,      //CMD_BUFFER_MOVE_SET_VEL
	CMD_BUFFER_MOVE_SET_ACC,      //CMD_BUFFER_MOVE_SET_ACC
	CMD_BUFFER_GEAR,      //BUFFER_GEAR
};


//G00(快速定位)命令参数
struct _G00PARA{
	float synVel; //插补段合成速度
	float synAcc; //插补段合成加速度
    long lX;       //X轴到达位置绝对位置(单位：pluse)
    long lY;       //Y轴到达位置绝对位置(单位：pluse)
    long lZ;       //Z轴到达位置绝对位置(单位：pluse)
    long lA;       //A轴到达位置绝对位置(单位：pluse)
	unsigned char iDimension; //参与插补的轴数量
	unsigned char cFuncFlag; //该位为0X01,代表启用lDisMask
	long segNum;
	long lB;       //B轴到达位置绝对位置(单位：pluse)(放在这里兼容老版本，位置不能随便移动)
	long lDisMask; //屏蔽掩码，对应位为1代表该轴不运动
};
//G01(直线插补)命令参数(任意2到3轴，上位机保证)
struct _G01PARA{
	float synVel;    //插补段合成速度
	float synAcc;    //插补段合成加速度
	float velEnd;   //插补段的终点速度
    long lX;       //X轴到达位置绝对位置(单位：pluse)
    long lY;       //Y轴到达位置绝对位置(单位：pluse)
    long lZ;       //Z轴到达位置绝对位置(单位：pluse)
    long lA;       //A轴到达位置绝对位置(单位：pluse)
	long segNum;
	unsigned char iDimension; //参与插补的轴数量
	unsigned char iPreciseStopFlag;   //精准定位标志位，如果为1，终点按照终点坐标来
	long lB;                //B轴到达位置绝对位置(单位：pluse)(放在这里兼容老版本，位置不能随便移动)
};

//G02_G03(顺圆弧插补)命令参数(任意2轴，上位机保证)
struct _G02_3PARA{
	float synVel;    //插补段合成速度
	float synAcc;    //插补段合成加速度
	float velEnd;   //插补段的终点速度
    int iPlaneSelect;       //平面选择0：XY平面 1：XZ平面 2：YZ平面
    int iEnd1;              //第一轴终点坐标（单位um）
    int iEnd2;              //第二轴终点坐标（单位um）
    int iI;                 //圆心坐标（单位um）(相对于起点)
    int iJ;                 //圆心坐标（单位um）(相对于起点)
	long segNum;
    unsigned char iPreciseStopFlag;   //精准定位标志位，如果为1，终点按照终点坐标来
};

//G04延时
struct _G04PARA{
unsigned long ulDelayTime;       //延时时间,单位MS
long segNum;
};

//G05设置用户自定义段号
struct _G05PARA{
long lUserSegNum;       //用户自定义段号
};

//BufferMove命令参数(最多支持8轴)
struct _BufferMoveGearPARA{
	long lAxis1Pos[8];         //轴目标位置，最大支持8轴。轴的加速度和速度采用点位运动速度和加速度。该轴必须处于点位模式且不是插补轴
	long lUserSegNum;          //用户自定义行号
	unsigned char cAxisMask;   //轴掩码，bit0代表轴1，bit1代表轴2，.......对应位为1代表该轴要bufferMove
	unsigned char cModalMask;  //轴掩码，bit0代表轴1，bit1代表轴2，.......对应位为1代表该轴为阻塞，该轴到位后才进入下一行
};

//BufferMove设置Vel和Acc命令参数(最多支持8轴)
struct _BufferMoveVelAccPARA{
	float dVelAcc[8];          //轴速度及加速度，最大支持8轴。
	long lUserSegNum;          //用户自定义行号
	unsigned char cAxisMask;   //轴掩码，bit0代表轴1，bit1代表轴2，.......对应位为1代表该轴要bufferMove
};

//SetIO设置物理IO
struct _SetIOPara{
	unsigned short nCarkIndex;  //板卡索引，0代表主卡，1代表扩展卡1，2代表扩展卡2......依次类推
	unsigned short nDoMask;
	unsigned short nDoValue;
	long lUserSegNum;
};

//SetIO设置物理IO
struct _SetIOReversePara{
	unsigned short nCarkIndex;  //板卡索引，0代表主卡，1代表扩展卡1，2代表扩展卡2......依次类推
	unsigned short nDoMask;
	unsigned short nDoValue;
	unsigned short nReverseTime;
	long lUserSegNum;
};

//G代码参数
union _CMDPara{
    struct _G00PARA     G00PARA;
    struct _G01PARA     G01PARA;
    struct _G02_3PARA   G02_3PARA;
    struct _G04PARA     G04PARA;
    struct _G05PARA     G05PARA;
	struct _BufferMoveGearPARA  BufferMoveGearPARA;
	struct _BufferMoveVelAccPARA BufferMoveVelAccPARA;
	struct _SetIOPara   SetIOPara;
};

//每一行程序结构体
typedef struct _CrdData{
    unsigned char CMDType;              //指令类型，支持最多255种指令0：GOO 1：G01 2：G02 FF:文件结束
    union _CMDPara CMDPara;         //指令参数，不同命令对应不同参数
}TCrdData;

//前瞻参数结构体
typedef struct _LookAheadPrm
{
	int lookAheadNum;                               //前瞻段数
	double dSpeedMax[INTERPOLATION_AXIS_MAX];	    //各轴的最大速度(p/ms)
	double dAccMax[INTERPOLATION_AXIS_MAX];			//各轴的最大加速度
	double dMaxStepSpeed[INTERPOLATION_AXIS_MAX];   //各轴的最大速度变化量（相当于启动速度）
	double dScale[INTERPOLATION_AXIS_MAX];			//各轴的脉冲当量

	//这个指针变量一定要放到最后，因为指针变量再32位系统下长度是32，在64位系统下长度是64
	TCrdData * pLookAheadBuf;                       //前瞻缓冲区指针
}TLookAheadPrm;

//轴回零参数
typedef struct _AxisHomeParm{
	short		nHomeMode;					//回零方式：0--无 1--HOME回原点	2--HOME加Index回原点3----Z脉冲	
	short		nHomeDir;					//回零方向，1-正向回零，0-负向回零
	long        lOffset;                    //回零偏移，回到零位后再走一个Offset作为零位

	double		dHomeRapidVel;			    //回零快移速度，单位：Pluse/ms
	double		dHomeLocatVel;			    //回零定位速度，单位：Pluse/ms
	double		dHomeIndexVel;			    //回零寻找INDEX速度，单位：Pluse/ms
	double      dHomeAcc;                   //回零使用的加速度

	long ulHomeIndexDis;           //找Index最大距离
	long ulHomeBackDis;            //回零时，第一次碰到零位后的回退距离
	unsigned short nDelayTimeBeforeZero;    //位置清零前，延时时间，单位ms
	unsigned long ulHomeMaxDis;//回零最大寻找范围，单位脉冲
}TAxisHomePrm;

//系统状态结构体
typedef struct _AllSysStatusData
{
	double dAxisEncPos[9];//轴编码器位置，包含一个手轮
	double dAxisPrfPos[8];//轴规划位置
	unsigned long lAxisStatus[8];//轴状态
	short nADCValue[2];//ADC值
	long lUserSegNum[2];//两个坐标系的用户段号
	long lRemainderSegNum[2];//两个坐标系的剩余段号
	short nCrdRunStatus[2];//两个坐标系的坐标系状态
	long lCrdSpace[2];//两个坐标系的剩余空间
	double dCrdVel[2];//两个坐标系的速度
	double dCrdPos[2][5];//两个坐标系的坐标
	long lLimitPosRaw;//正硬限位
	long lLimitNegRaw;//负硬限位
	long lAlarmRaw;//报警输入
	long lHomeRaw;//零位输入
	long lMPG;//手轮信号
	long lGpiRaw[4];//通用IO输入（除主卡外，最大支持3个扩展模块）
}TAllSysStatusData;

//16轴以内系统状态结构体
typedef struct _AllSysStatusDataEX
{
	long lAxisEncPos[16];//轴编码器位置
	long lAxisPrfPos[16];//轴规划位置
	unsigned long lAxisStatus[16];//轴状态
	short nADCValue[2];//ADC值
	long lUserSegNum[2];//两个坐标系的用户段号
	long lRemainderSegNum[2];//两个坐标系的剩余段号
	short nCrdRunStatus[2];//两个坐标系的坐标系状态
	long lCrdSpace[2];//两个坐标系的剩余空间
	double dCrdVel[2];//两个坐标系的速度
	long lCrdPos[2][5];//两个坐标系的坐标
	long lLimitPosRaw;//正硬限位
	long lLimitNegRaw;//负硬限位
	long lAlarmRaw;//报警输入
	long lHomeRaw;//零位输入
	long lMPGEncPos;//手轮编码器
	long lMPG;//IO手轮信号
	long lGpiRaw[8];//通用IO输入（除主卡外，最大支持7个扩展模块）
}TAllSysStatusDataEX;

//16轴以内系统状态结构体(带输出IO)
typedef struct _AllSysStatusDataSX
{
	long lAxisEncPos[16];//轴编码器位置
	long lAxisPrfPos[16];//轴规划位置
	unsigned long lAxisStatus[16];//轴状态
	short nADCValue[2];//ADC值
	long lUserSegNum[2];//两个坐标系的用户段号
	short lRemainderSegNum[2];//两个坐标系的剩余段号
	short nCrdRunStatus[2];//两个坐标系的坐标系状态
	short lCrdSpace[2];//两个坐标系的剩余空间
	float dCrdVel[2];//两个坐标系的速度
	long lCrdPos[2][5];//两个坐标系的坐标
	short lLimitPosRaw;//正硬限位
	short lLimitNegRaw;//负硬限位
	short lAlarmRaw;//报警输入
	short lHomeRaw;//零位输入
	long lMPGEncPos;//手轮编码器
	int lMPG;//手轮IO信号
	long lGpiRaw[8];//通用IO输入（除主卡外，最大支持7个扩展模块）
	long lGpoRaw[8];//通用IO输出（除主卡外，最大支持7个扩展模块）
}TAllSysStatusDataSX;

//通讯帧头
typedef struct _ComDataFrameHead
{
	char nCardNum;//控制卡物理编号
	char nType;   //帧信息类型
	char nSubType;//帧信息子类型
	char nResult; //执行结果
	unsigned long  ulAxisMask;  //轴掩码    （最多支持一块板卡32个轴）
	unsigned char  nCrdMask;    //坐标系掩码（最多支持8个坐标系）
	unsigned char  nFrameCount; //轮询校验位（发送一帧加1）
	unsigned short nDataBufLen; //有效数据域长度
	unsigned long ulCRC;//校验和
}TComDataFrameHead;

//前瞻状态管理
typedef struct _LookAheadState{
	int iFirstTime;                              //第一次前瞻标志
	int iWriteIndex;                             //缓冲区先入先出，写指针可能变化，读指针始终是最后一帧
	int iNeedLookAhead;                          //需要对前瞻缓冲区进行前瞻计算
	int iNeedAutoSendAllDataInBuf;               //需要把前瞻缓冲区的数据全部发送出去（自动发送）
	double dTotalLength;                         //当前在缓冲区中的所有数据段长度总和
	double dStartSpeed;                          //前瞻缓冲区当前速度，第一次为0，后面为最近发送出去那一段的终点速度

	double dStartPos[32];                  //坐标系刚刚建立时，各参与插补轴的当前位置。每压入一条，更新一下
	double dModalPos[32];                  //前瞻时用到，记录假设上一条运动完成后，各插补轴应该所在的位置
	double dEndPos[32];                    //最后压入的插补端结束后各轴位置

	TCrdDataState *pCrdDataState;                //指向插补数据状态的指针
	int iNeedLookAheadFlag;                      //需要前瞻标志位
}TLookAheadState;

//通讯帧
typedef struct _ComDataFrame
{
	TComDataFrameHead Head;
	unsigned char nDataBuf[1100];
}TComDataFrame;

//TXPDO和RXPDO参数
typedef struct _ECatPDOParm{
	unsigned char cTXPDOCount;//TXPDO条目数量
	unsigned char cRXPDOCount;//RXPDO条目数量

	unsigned long lTXPDOItem[10];//TXPDO条目
	unsigned long lRXPDOItem[10];//RXPDO条目
}TECatPDOParm;

#define CRDSYS_MAX_COUNT				    (16)	//最大坐标系数量(最大通道数量)
#define AXIS_MAX                        8

class MultiCard
{
public:
 
	MultiCard();
 
	~MultiCard();

private:
	short m_nCardNum;
	short m_nOpenFlag;//板卡打开成功标志位
	
	TCrdPrm m_LookAheadCrdPrm[CRDSYS_MAX_COUNT];//模块变量，发送坐标系参数时存放一份坐标系参数，主要给前瞻缓冲区用

	//一共两个坐标系，每个坐标系两个前瞻缓冲区
	//定义前瞻缓冲区参数
	TLookAheadPrm m_LookAheadPrm[CRDSYS_MAX_COUNT][2];
	//定义前瞻缓冲区状态
	TLookAheadState mLookAheadState[CRDSYS_MAX_COUNT][2];

	int ComWaitForResponseData(TComDataFrame * pDataFrame,TComDataFrame * pRecFrame);
	int ComSendData(TComDataFrame * pSendFrame,TComDataFrame * pRecFrame);
	int ComSendDataOpen(char* cString,int iLen);
	int WriteFrameToLookAheadBuf(short iCrdIndex,short FifoIndex,TCrdData* pCrdData);
	int InitLookAheadBufCtrlData(short iCrdIndex,short FifoIndex);
	int LookAhead(short iCrdIndex,short FifoIndex);
	int ReadFrameFromLookAheadBuf(short iCrdIndex,short FifoIndex,TCrdData* pCrdData);
	int ClearLookAheadBuf(short iCrdIndex,short FifoIndex);
	double CalConSpeed(short iCrdIndex,short FifoIndex,TCrdDataState *pCurState,TCrdDataState *pNextState,TCrdData* pCrdDataCur,TCrdData* pCrdDataNext);
	int IsLookAheadBufEmpty(short iCrdIndex,short FifoIndex);
	int IsLookAheadBufFull(short iCrdIndex,short FifoIndex);
	float CalculateAngleByRelativePos(double x,double y);
	double CalEndSpeed(double dStartSpeed,double dAccDec,double dLength);
	void GetCoordAfterRotate90(double i,double j,double &iAfterRotate, double &jAfterRotate,int iDir);
	int GetLookAheadBufRemainDataNum(short iCrdIndex,short FifoIndex);
	int ReadPitchErrorTableInfo(short nTableNum,short* pPointNum,long* pStartPos,long* pEndPos);
	int ReadPitchErrorTableValue(short nTableNum,short nStartPointNum,short nLen,short *pErrValue1,short *pErrValue2);

	int MC_GetClockHighPrecision(double *pClock);
	int MC_GetClock(double *pClock);
	int MC_LoadConfig(char *pFile); 
	int MC_GetConfig();
	int MC_GpiSns(unsigned long sense);
	int MC_GetGpiSns(unsigned long *pSense);
	int MC_GetProfileScale(short iAxis,short *pAlpha,short *pBeta);
    int MC_SetMtrLmt(short dac,short limit);
	int MC_GetMtrLmt(short dac,short *pLimit);
	int MC_PrfFollow(short profile,short dir=0);
	int MC_SetFollowMaster(short profile, short masterIndex, short masterType,short masterItem);
	int MC_GetFollowMaster(short profile,short *pMasterIndex,short *pMasterType,short *pMasterItem);
	int MC_SetFollowLoop(short profile,short loop);
	int MC_GetFollowLoop(short profile,long *pLoop);
	int MC_SetFollowEvent(short profile,short nEvent,short masterDir,long pos);
	int MC_GetFollowEvent(short profile,short *pEvent,short *pMasterDir,long *pPos);
	int MC_FollowSpace(short profile,short *pSpace,short FifoIndex);
	int MC_FollowData(short profile,long masterSegment,double slaveSegment,short type,short FifoIndex);
	int MC_FollowClear(short profile, short FifoIndex);
	int MC_FollowStart(long mask, long option);
	int MC_FollowSwitch(long mask);
	int MC_SetFollowMemory(short profile,short memory);
	int MC_GetFollowMemory(short profile,short *pMemory);
	int MC_SetPtLoop(short nAxisNum);
	int MC_GetPtLoop(short nAxisNum);
	int MC_PrfPvt(short profile);
	int MC_SetPvtLoop(short profile,long loop);
	int MC_GetPvtLoop(short profile,long *pLoopCount,long *pLoop);
	int MC_PvtTable(short tableId,long lCount,double *pTime,double *pPos,double *pVel);
	int MC_PvtTableComplete(short tableId,long lCount,double *pTime,double *pPos,double *pA,double *pB,double *pC,double velBegin,double velEnd);
	int MC_PvtTablePercent(short tableId,long lCount,double *pTime,double *pPos,double *pPercent, double velBegin);
	int MC_PvtPercentCalculate(long lCount,double *pTime,double *pPos,double *pPercent, double velBegin,double *pVel);
	int MC_PvtTableContinuous(short tableId,long lCount,double *pPos,double *pVel,double *pPercent, double *pVelMax, double *pAcc, double *pDec,double timeBegin);
	int MC_PvtContinuousCalculate(long lCount,double *pPos,double *pVel,double *pPercent, double *pVelMax, double *pAcc, double *pDec, double *pTime);
	int MC_PvtTableSelect(short profile,short tableId);
	int MC_PvtStart(long mask);
	int MC_PvtStatus(short profile,short *pTableId,double *pTime,short nCount);
	int MC_IntConfig(short nCardIndex,short nBitIndex,short nIntLogic);
	int MC_GetIntConfig(short nCardIndex,short nBitIndex,short *nIntLogic);
	int MC_IntEnable(short nCardIndex,GAS_IOCallBackFun IntCallBack);
	int MC_GetControlInfo(short control);
	int MC_FwUpdate(char *File,unsigned long ulFileLen,int *pProgress);
	int MC_ArcXYR(short nCrdNum,long x,long y,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_ArcYZR(short nCrdNum,long y,long z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_ArcZXR(short nCrdNum,long z,long x,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_BufLaserFollowRatio(short nCrdNum,double dRatio,double dMinPower,double dMaxPower,short nFifoIndex,short nChannel);
	int MC_BufLmtsOn(short nCrdNum,short nAxisNum,short limitType,short FifoIndex=0,long segNum = 0);
	int MC_BufLmtsOff(short nCrdNum,short nAxisNum,short limitType,short FifoIndex=0,long segNum = 0);
	int MC_BufSetStopIo(short nCrdNum,short nAxisNum,short stopType,short inputType,short inputIndex,short FifoIndex=0,long segNum = 0);
	int MC_BufGearPercent(short nCrdNum,short gearAxis,long pos,short accPercent,short decPercent,short FifoIndex=0,long segNum = 0);
	int MC_BufJumpNextSeg(short nCrdNum,short nAxisNum,short limitType,short FifoIndex=0);
	int MC_BufSynchPrfPos(short nCrdNum,short nEncodeNum,short profile,short FifoIndex=0);
	int MC_BufVirtualToActual(short nCrdNum,short FifoIndex=0);
	int MC_BufSetLongVar(short nCrdNum,short index,long value,short FifoIndex=0);
	int MC_BufSetDoubleVar(short nCrdNum,short index,double value,short FifoIndex=0);
	int MC_CrdStartStep(short mask,short option);
	int MC_CrdStepMode(short mask,short option);
	int MC_GetUserTargetVel(short nCrdNum,double *pTargetVel);
	int MC_GetSegTargetPos(short nCrdNum,long *pTargetPos);
	int MC_G001PreData(short nCrdNum,long lC,long* plEnd,short FifoIndex=0,long segNum=-1);
	int MC_HelixXYRZ(short nCrdNum,long x,long y,long z,double radius,short circleDir,double synVel,double synAcc,double velEnd,short FifoIndex=0,long segNum = 0);
	int MC_HelixYZRX(short nCrdNum,long x,long y,long z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_HelixZXRY(short nCrdNum,long x,long y,long z,double radius,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);

	int MC_HomeInit();
	int MC_Home(short nAxisNum,long pos,double vel,double acc,long offset);
	int MC_Index(short nAxisNum,long pos,long offset);
	int MC_HomeSts(short nAxisNum,unsigned short *pStatus);

	int MC_HandwheelInit();
	int MC_SetHandwheelStopDec(short nAxisNum,double decSmoothStop,double decAbruptStop);

	int MC_CmpRpt(short nEncode, short nEncodeType,short nChannel,long lStartPos, long lRptTime, long lInterval, short nTime);
	int MC_CmpRpt(short nCmpEncodeNum,short nChannel,long lStartPos, long lRptTime, long lInterval, short nTime,short nPluseType,short nAbsPosFlag);

	int MC_ECatSendAdoAddr(short nStationNum,short nAdoAddr);
	int MC_ECatSendSdoAddr(short nStationNum,short nSdoIndex,short nSdoSubIndex,short* pPDOFlag,long *pPDOValue);
	int MC_BeginWriteCamTable();
	int MC_EndWriteCamTable();
public:	
	
	//其他指令列表
	int MC_Open(short nCardNum,char* cPCEthernetIP,unsigned short nPCEthernetPort,char* cCardEthernetIP,unsigned short nCardEthernetPort);
	int MC_Close(void);
	int MC_Reset();
	int MC_GetVersion(char *pVersion);
	int MC_SetPrfPos(short profile,long prfPos);
	int MC_SynchAxisPos(long mask);
	int MC_ZeroPos(short nAxisNum,short nCount=1);
	int MC_SetAxisBand(short nAxisNum,long lBand,long lTime);
	int MC_GetAxisBand(short nAxisNum,long *pBand,long *pTime);
	int MC_SetBacklash(short nAxisNum,long lCompValue,double dCompChangeValue,long lCompDir);
	int MC_GetBacklash(short nAxisNum,long *pCompValue,double *pCompChangeValue,long *pCompDir);
	int MC_SendString(char* cString,int iLen,int iOpenFlag=0);

	//系统配置信息
	int MC_AlarmOn(short nAxisNum);
	int MC_AlarmOff(short nAxisNum);
	int MC_GetAlarmOnOff(short nAxisNum,short *pAlarmOnOff);
	int MC_AlarmSns(unsigned short nSense);
	int MC_GetAlarmSns(unsigned short *pSense);
	int MC_HomeSns(unsigned short sense);
	int MC_GetHomeSns(unsigned short *pSense);
	int MC_LmtsOn(short nAxisNum,short limitType=-1);
	int MC_LmtsOff(short nAxisNum,short limitType=-1);
	int MC_GetLmtsOnOff(short nAxisNum,short *pPosLmtsOnOff, short *pNegLmtsOnOff);
	int MC_LmtSns(unsigned short nSense);
	int MC_LmtSnsEX(unsigned long lSense);
	int MC_GetLmtSns(unsigned long *pSense);
	int MC_ProfileScale(short nAxisNum,short alpha,short beta);
	int MC_EncScale(short nAxisNum,short alpha,short beta);
	int MC_GetEncScale(short iAxis,short *pAlpha,short *pBeta);
	int MC_StepDir(short step);
	int MC_StepPulse(short step);
	int MC_GetStep(short nAxisNum,short *pStep);
	int MC_StepSns(unsigned short sense);
	int MC_GetStepSns(short *pSense);
	int MC_SetMtrBias(short dac,short bias);
	int MC_GetMtrBias(short dac,short* pBias);
	int MC_EncSns(unsigned short sense);
	int MC_GetEncSns(short *pSense);
	int MC_EncOn(short nEncoderNum);
	int MC_EncOff(short nEncoderNum);
	int MC_GetEncOnOff(short nAxisNum,short *pEncOnOff);
	int MC_SetPosErr(short nAxisNum,long lError);
	int MC_GetPosErr(short nAxisNum,long *pError);
	int MC_SetStopDec(short nAxisNum,double decSmoothStop,double decAbruptStop);
	int MC_GetStopDec(short nAxisNum,double *pDecSmoothStop,double *pDecAbruptStop);
	int MC_CtrlMode(short nAxisNum,short mode);
	int MC_GetCtrlMode(short nAxisNum,short *pMode);
	int MC_SetStopIo(short nAxisNum,short stopType,short inputType,short inputIndex);
	int MC_SetAdcFilter(short nAdcNum,short nFilterTime);
	int MC_SetSmoothTime(short nAxisNum,short nSmoothTime);
	int MC_SetAdcBias(short nAdcNum,short nBias);
	int MC_GetAdcBias(short nAdcNum,short *pBias);

	//运动状态检测指令列表
	int MC_GetSts(short nAxisNum,long *pSts,short nCount=1,unsigned long *pClock=NULL);
	int MC_ClrSts(short nAxisNum,short nCount=1);
	int MC_GetPrfMode(short profile,long *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetPrfPos(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetPrfVel(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetPrfAcc(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetAxisPrfPos(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetAxisPrfVel(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetAxisPrfAcc(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetAxisEncPos(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetAxisEncVel(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetAxisEncAcc(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetAxisError(short nAxisNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_Stop(long lMask,long lOption);
	int MC_AxisOn(short nAxisNum);
	int MC_AxisOff(short nAxisNum);
	int MC_GetAllSysStatus(TAllSysStatusData *pAllSysStatusData);
	int MC_GetAllSysStatusEX(TAllSysStatusDataEX *pAllSysStatusData);
	int MC_GetAllSysStatusSX(TAllSysStatusDataSX *pAllSysStatusData);

	//点位运动指令列表（包括点位和速度模式）
	int MC_PrfTrap(short nAxisNum);
	int MC_SetTrapPrm(short nAxisNum,TTrapPrm *pPrm);
	int MC_SetTrapPrmSingle(short nAxisNum,double dAcc,double dDec,double dVelStart,short  dSmoothTime);
	int MC_GetTrapPrm(short nAxisNum,TTrapPrm *pPrm);
	int MC_GetTrapPrmSingle(short nAxisNum,double* dAcc,double* dDec,double* dVelStart,short*  dSmoothTime);
	int MC_PrfJog(short nAxisNum);
	int MC_SetJogPrm(short nAxisNum,TJogPrm *pPrm);
	int MC_SetJogPrmSingle(short nAxisNum,double dAcc,double dDec,double dSmooth);
	int MC_GetJogPrm(short nAxisNum,TJogPrm *pPrm);
	int MC_GetJogPrmSingle(short nAxisNum,double* dAcc,double* dDec,double* dSmooth);
	int MC_SetPos(short nAxisNum,long pos);
	int MC_GetPos(short nAxisNum,long *pPos);
	int MC_SetVel(short nAxisNum,double vel);
	int MC_GetVel(short nAxisNum,double *pVel);
	int MC_SetMultiVel(short nAxisNum,double *pVel,short nCount=1);
	int MC_SetMultiPos(short nAxisNum,long *pPos,short nCount=1);
	int MC_Update(long mask);

	//电子齿轮模式指令列表
	int MC_PrfGear(short nAxisNum,short dir=0);
	int MC_SetGearMaster(short nAxisNum,short nMasterAxisNum,short masterType=GEAR_MASTER_PROFILE);
	int MC_GetGearMaster(short nAxisNum,short *nMasterAxisNum,short *pMasterType=NULL);
	int MC_SetGearRatio(short nAxisNum,long masterEven,long slaveEven,long masterSlope=0,long lStopSmoothTime = 200);
	int MC_GetGearRatio(short nAxisNum,long *pMasterEven,long *pSlaveEven,long *pMasterSlope=NULL,long *pStopSmoothTime=NULL);
	int MC_GearStart(long mask);
	int MC_GearStop(long lAxisMask,long lEMGMask);
	int MC_SetGearEvent(short nAxisNum,short nEvent,double startPara0,double startPara1);
	int MC_GetGearEvent(short nAxisNum,short *pEvent,double *pStartPara0,double *pStartPara1);
	int MC_SetGearIntervalTime(short nAxisNum,short nIntervalTime);
	int MC_GetGearIntervalTime(short nAxisNum,short* nIntervalTime);

	//电子凸轮模式指令列表
	int MC_PrfCam(short nAxisNum,short nTableNum);
	int MC_SetCamMaster(short nAxisNum,short nMasterAxisNum,short nMasterType);
	int MC_GetCamMaster(short nAxisNum,short *pnMasterAxisNum,short *pMasterType);
	int MC_SetCamEvent(short nAxisNum,short nEvent,double startPara0,double startPara1);
	int MC_GetCamEvent(short nAxisNum,short *pEvent,double *pStartPara0,double *pStartPara1);
	int MC_SetCamIntervalTime(short nAxisNum,short nIntervalTime);
	int MC_GetCamIntervalTime(short nAxisNum,short *nIntervalTime);
	int MC_SetUpCamTable(short nCamTableNum,long lMasterValueMax, long *plSlaveCamData, long lCamTableLen);
	int MC_SetUpCamTableByKeyPoint(short nCamTableNum,long *plData, long lKeyPointNum,short nDynamicStaticFlag,short nNextCycleFlag);
	int MC_DownCamTable(short nTableNum,int *pProgress);
	int MC_CamStart(long lMask);
	int MC_CamStop(long lAxisMask,long lEMGMask);

	//PT模式指令列表
	int MC_PrfPt(short nAxisNum,short mode=PT_MODE_STATIC);
	int MC_PtSpace(short nAxisNum,long *pSpace,short nCount);
	int MC_PtRemain(short nAxisNum,long *pRemainSpace,short nCount);
	int MC_PtData(short nAxisNum,short* pData,long lLength,double dDataID);
	int MC_PtClear(long lAxisMask);
	int MC_PtStart(long lAxisMask);

	//插补运动模式指令列表
	int MC_StartDebugLog();
	int MC_StopDebugLog();
	int MC_SetCrdPrm(short nCrdNum,TCrdPrm *pCrdPrm);
	int MC_SetCrdPrmEX(short nCrdNum,TCrdPrmEx *pCrdPrm);
	int MC_GetCrdPrm(short nCrdNum,TCrdPrm *pCrdPrm);
	int MC_SetCrdPrmSingle(short nCrdNum,short dimension,short *profile,double synVelMax,double synAccMax,short evenTime,short setOriginFlag,long *originPos);
	int MC_SetAddAxis(short nAxisNum,short nAddAxisNum);
	int MC_SetCrdOffset(short nCrdNum,long lOffsetX,long lOffsetY,long lOffsetZ,long lOffsetA,long lOffsetB,double dOffsetAngle);
	int MC_SetCrdStopDec(short nCrdNum,double decSmoothStop,double decAbruptStop);
	int MC_GetCrdStopDec(short nCrdNum,double *pDecSmoothStop,double *pDecAbruptStop);
	int MC_InitLookAhead(short nCrdNum,short FifoIndex,TLookAheadPrm* plookAheadPara);
	int MC_InitLookAheadSingle(short nCrdNum,short FifoIndex,int lookAheadNum,double* dSpeedMax,double* dAccMax,double *dMaxStepSpeed,double *dScale);
	int MC_CrdClear(short nCrdNum,short FifoIndex);
	int MC_LnXYG0(short nCrdNum,long x,long y,double synVel,double synAcc,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZG0(short nCrdNum,long x,long y,long z,double synVel,double synAcc,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZAG0(short nCrdNum,long x,long y,long z,long a,double synVel,double synAcc,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZABG0(short nCrdNum,long x,long y,long z,long a,long b,double synVel,double synAcc,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZABCG0(short nCrdNum,long x,long y,long z,long a,long b,long c,double synVel,double synAcc,short FifoIndex=0,long segNum = 0);
	int MC_LnAllG0(short nCrdNum,long *pPos,short nDim,double synVel,double synAcc,short FifoIndex=0,long segNum = 0);
	int MC_LnX(short nCrdNum,long x,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_LnXY(short nCrdNum,long x,long y,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZ(short nCrdNum,long x,long y,long z,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZA(short nCrdNum,long x,long y,long z,long a,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZAB(short nCrdNum,long x,long y,long z,long a,long b,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_LnXYZABC(short nCrdNum,long x,long y,long z,long a,long b,long c,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_LnAll(short nCrdNum,long* pPos,short nDim, double synVel,double synAcc,double velEnd,short FifoIndex=0,long segNum = 0);
	int MC_ArcXYC(short nCrdNum,long x,long y,double xCenter,double yCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_ArcYZC(short nCrdNum,long y,long z,double yCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_ArcXZC(short nCrdNum,long x,long z,double xCenter,double zCenter,short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum = 0);
	int MC_HelixPreData(short nCrdNum,long x,long y,long z,short FifoIndex=0,long segNum=-1);
	int MC_HelixXYCZ(short nCrdNum,long x,long y,long z,double xCenter,double yCenter,float k, short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum=-1);
	int MC_HelixYZCX(short nCrdNum,long x,long y,long z,double yCenter,double zCenter,float k, short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum=-1);
	int MC_HelixXZCY(short nCrdNum,long x,long y,long z,double xCenter,double zCenter,float k, short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum=-1);
	int MC_HelixXYCCount(short nCrdNum,double xCenter,double yCenter,float k,float CirlceCount, short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum=-1);
	int MC_HelixXZCCount(short nCrdNum,double xCenter,double zCenter,float k,float CirlceCount, short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum=-1);
	int MC_HelixYZCCount(short nCrdNum,double yCenter,double zCenter,float k,float CirlceCount, short circleDir,double synVel,double synAcc,double velEnd=0,short FifoIndex=0,long segNum=-1);
	int MC_BufIO(short nCrdNum,unsigned short nDoType,unsigned short nCardIndex,unsigned short doMask,unsigned short doValue,short FifoIndex=0,long segNum = 0);
	int MC_BufIOEx(short nCrdNum,unsigned short nDoType,unsigned short nCardIndex,unsigned long doMask,unsigned long doValue,short FifoIndex=0,long segNum = 0);
	int MC_BufIOReverse(short nCrdNum,unsigned short nDoType,unsigned short nCardIndex,unsigned short doMask,unsigned short doValue,unsigned short nReverseTime,short FifoIndex=0,long segNum = 0);
	int MC_BufWaitIO(short nCrdNum,unsigned short nCardIndex,unsigned short nIOPortIndex,unsigned short nLevel,unsigned long lWaitTimeMS,unsigned short nFilterTime,short FifoIndex=0,long segNum=-1);
	int MC_BufDelay(short nCrdNum,unsigned long ulDelayTime,short FifoIndex=0,long segNum = 0);
	int MC_BufPWM(short nCrdNum,short nPwmNum ,double dFreq,double dDuty,short nFifoIndex,long lUserSegNum=-1);
	int MC_BufDA(short nCrdNum,short nDacNum,short nValue,short nFifoIndex,long lUserSegNum=-1);
	int MC_BufZeroPos(short nCrdNum,short nAxisNum,short nFifoIndex,long lUserSegNum=-1);
	int MC_BufSetM(short nCrdNum,int iMAddr,short nMValue,short nFifoIndex,long lUserSegNum=-1);
	int MC_BufWaitM(short nCrdNum,int iMAddr,short nMValue,short nFifoIndex,long lUserSegNum=-1);
	int MC_LnXYZABMaskG0(short nCrdNum,long x,long y,long z,long a,long b,long lEnableMask,double synVel,double synAcc,short FifoIndex=0,long segNum=-1);
	int MC_CrdData(short nCrdNum,void *pCrdData,short FifoIndex=0);
	int MC_CrdStart(short mask,short option);
	int MC_SetOverride(short nCrdNum,double synVelRatio);
	int MC_GetCrdPos(short nCrdNum,double *pPos);
	int MC_GetCrdVel(short nCrdNum,double *pSynVel);
	int MC_CrdSpace(short nCrdNum,long *pSpace,short FifoIndex=0);
	int MC_CrdStatus(short nCrdNum,short *pCrdStatus,long *pSegment,short FifoIndex=0);
	int MC_SetUserSegNum(short nCrdNum,long segNum,short FifoIndex=0);
	int MC_GetUserSegNum(short nCrdNum,long *pSegment,short FifoIndex=0);
	int MC_GetRemainderSegNum(short nCrdNum,long *pSegment,short FifoIndex=0);
	int MC_GetLookAheadSegCount(short nCrdNum,long *pSegCount,short FifoIndex=0);
	int MC_GetLookAheadSpace(short nCrdNum,long *pSpace,short FifoIndex=0);
	int MC_BufCmpData(short nCrdNum,short nCmpEncodeNum,short nPluseType, short nStartLevel, short nTime,long *pBuf, short nBufLen,short nAbsPosFlag,short nTimerFlag,short nFifoIndex,long lSegNum = -1);
	int MC_BufCmpPluse(short nCrdNum,short nChannel,short nPluseType,short nTime,short nTimerFlag,short nFifoIndex,long lSegNum=-1);
	int MC_BufMoveVel(short nCrdNum,short nAxisMask,float* pVel,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveVelSingle(short nCrdNum,short nAxisMask,float dVel0,float dVel1,float dVel2,float dVel3,float dVel4,float dVel5,float dVel6,float dVel7,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveVelEX(short nCrdNum,short nAxisMask,float* pVel,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveAcc(short nCrdNum,short nAxisMask,float* pAcc,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveAccSingle(short nCrdNum,short nAxisMask,float dAcc0,float dAcc1,float dAcc2,float dAcc3,float dAcc4,float dAcc5,float dAcc6,float dAcc7,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveAccEX(short nCrdNum,short nAxisMask,float* pAcc,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveDec(short nCrdNum,short nAxisMask,float* pDec,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveDecEX(short nCrdNum,short nAxisMask,float* pDec,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMove(short nCrdNum,short nAxisMask,long* pPos,short nModalMask,short nFifoIndex,long lSegNum);
	int MC_BufMoveSingle(short nCrdNum,short nAxisMask,long lPos0,long lPos1,long lPos2,long lPos3,long lPos4,long lPos5,long lPos6,long lPos7,short nModalMask,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufMoveEX(short nCrdNum,short nAxisMask,long* pPos,short nModalMask,short nFifoIndex,long lSegNum);
	int MC_BufGear(short nCrdNum,short nAxisMask,long* pPos,short nFifoIndex,long lSegNum);
	int MC_BufGearSingle(short nCrdNum,short nAxisMask,long lPos0,long lPos1,long lPos2,long lPos3,long lPos4,long lPos5,long lPos6,long lPos7,short nFifoIndex=0,long lSegNum=-1);
	int MC_BufJog(short nCrdNum,short nAxisNum,double dAccDec,double dVel,short nBlock,short nFifoIndex,long lUserSegNum);
	int MC_GetCrdErrStep(short nCrdNum,unsigned long *pulErrStep);

	//访问硬件资源指令列表
	int MC_GetDi(short nDiType,long *pValue);
	int MC_GetDiRaw(short nDiType,long *pValue);
	int MC_GetDiReverseCount(short nDiType,short diIndex,unsigned long *pReverseCount,short nCount=1,short nCardIndex=0,short nRiseSinkType=0);
	int MC_SetDiReverseCount(short nDiType,short diIndex,unsigned long ReverseCount,short nCount=1,short nCardIndex=0);
	int MC_SetDo(short nDoType,long value);
	int MC_SetDoBit(short nDoType,short nDoNum,short value);
	int MC_SetDoBitReverse(short nDoType,short nDoNum,short nValue,short nReverseTime);
	int MC_SetDoBitReverseEx(unsigned short nCardIndex,short nDoType,short nDoNum,short nValue,short nReverseTime);
	int MC_GetDo(short nDoType,long *pValue);
	int MC_GetEncPos(short nEncodeNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetEncVel(short nEncodeNum,double *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_SetEncPos(short nEncodeNum,long encPos);
	int MC_SetDac(short nDacNum,short* pValue,short nCount=1);
	int MC_GetAdc(short nADCNum,short *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_SetPwm(short nPwmNum ,double dFreq,double dDuty);
	int MC_GetPwm(short nPwmNum ,double *pFreq,double *pDuty);
	int MC_SetExtDoValue(short nCardIndex,unsigned long *value,short nCount=1);
	int MC_GetExtDiValue(short nCardIndex,unsigned long *pValue,short nCount=1);
	int MC_GetExtDoValue(short nCardIndex,unsigned long *pValue,short nCount=1);
	int MC_SetExtDoBit(short nCardIndex,short nBitIndex,unsigned short nValue);
	int MC_GetExtDiBit(short nCardIndex,short nBitIndex,unsigned short *pValue);
	int MC_GetExtDoBit(short nCardIndex,short nBitIndex,unsigned short *pValue);
	int MC_SetExtDiFilter(short nCardIndex,short FilterTime,long ulIOMask);
	int MC_SendEthToUartString(short nUartNum,unsigned char*pSendBuf, short nLength);
	int MC_ReadUartToEthString(short nUartNum,unsigned char* pRecvBuf, short* pLength);
	int MC_SetExDac(short nCardIndex,short nDacNum,short* pValue,short nCount=1);
	int MC_GetExAdc(short nCardIndex,short nADCNum,short *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_SetIOEventTrigger(short nEventNum,short nIOIndex,short nTriggerSense,long lFilterTimer,short nEventType,double dEventParm1,double dEventParm2);
	int MC_GetIOEventTrigger(short nEventNum,short *pTriggerFlag,short nCount);
	int MC_GetDac(short nDacNum,short *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_GetExDac(short nCardIndex,short nDacNum,short *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_UartConfig(unsigned short nUartNum,	unsigned long uLBaudRate,unsigned short nDataLength,unsigned short nVerifyType,unsigned short nStopBitLen);

	//比较输出指令
	int MC_CmpPluse(short nChannelMask, short nPluseType1, short nPluseType2, short nTime1,short nTime2, short nTimeFlag1, short nTimeFlag2);
	int MC_CmpBufSetChannel(short nBuf1ChannelNum,short nBuf2ChannelNum);
	int MC_CmpBufData(short nCmpEncodeNum, short nPluseType, short nStartLevel, short nTime, long *pBuf1, short nBufLen1, long *pBuf2, short nBufLen2,short nAbsPosFlag=0,short nTimerFlag=0);
	int MC_CmpBufSts(unsigned short *pStatus,unsigned short *pRemainDaga1,unsigned short *pRemainDaga2,unsigned short *pRemainSpace1,unsigned short *pRemainSpace2);
	int MC_CmpBufStop(short nChannelMask);
	int MC_CmpSetHighSpeedIOTrigger(short nChannelNum,short nTriggerSense,short nPluseType,short nStartLevel,short nTime,short nTimerFlag=0);
	int MC_CmpSetTriggerCount(long lTriggerCount1,long lTriggerCount2);
	int MC_CmpGetTriggerCount(long* plTriggerCount1,long* plTriggerCount2);

	//高速硬件捕获指令列表
	int MC_SetCaptureMode(short nEncodeNum,short mode);
	int MC_GetCaptureMode(short nEncodeNum,short *pMode,short nCount=1);
	int MC_GetCaptureStatus(short nEncodeNum,short *pStatus,long *pValue,short nCount=1,unsigned long *pClock=NULL);
	int MC_SetCaptureSense(short nEncodeNum,short mode,short sense);
	int MC_GetCaptureSense(short nEncodeNum,short mode,short *sense);
	int MC_ClearCaptureStatus(short nEncodeNum);
	int MC_SetContinueCaptureMode(short nEncodeNum,short nMode,short nContinueMode,short nFilterTime);
	int MC_GetContinueCaptureData(short nEncodeNum,long *pCapturePos,short* pCaptureCount);

	//安全机制指令列表
	int MC_SetSoftLimit(short nAxisNum,long lPositive,long lNegative);
	int MC_GetSoftLimit(short nAxisNum,long *pPositive,long *pNegative);
	int MC_SetHardLimP(short nAxisNum,short nType ,short nCardIndex,short nIOIndex);
	int MC_SetHardLimN(short nAxisNum,short nType ,short nCardIndex,short nIOIndex);
	int MC_EStopSetIO(short nCardIndex,short nIOIndex,short nEStopSns,unsigned long lFilterTime);
	int MC_EStopOnOff(short nEStopOnOff);
	int MC_EStopGetSts(short *nEStopSts);
	int MC_EStopClrSts();
	int MC_EStopConfig(unsigned int ulEnableMask,unsigned int ulEnableValue,short nAdcMask,short nAdcValue,unsigned int ulIOMask,unsigned int ulIOValue);
	int MC_CrdHlimEnable(short nCrdNum ,short nEnableFlag);

	//自动回零相关API
	int MC_HomeStart(short nAxisNum);
	int MC_HomeStop(short nAxisNum);
	int MC_HomeSetPrm(short nAxisNum,TAxisHomePrm *pAxisHomePrm);
	int MC_HomeSetPrmSingle(short iAxisNum,short nHomeMode,short nHomeDir,long lOffset,double dHomeRapidVel,double dHomeLocatVel,double dHomeIndexVel,double dHomeAcc);
	int MC_HomeGetPrm(short nAxisNum,TAxisHomePrm *pAxisHomePrm);
	int MC_HomeGetPrmSingle(short nAxisNum,short *nHomeMode,short *nHomeDir,long *lOffset,double* dHomeRapidVel,double* dHomeLocatVel,double* dHomeIndexVel,double* dHomeAcc);
	int MC_HomeGetSts(short nAxisNum,unsigned short* pStatus);

	//手轮相关
	int MC_StartHandwheel(short nAxisNum,short nMasterAxisNum = 9,long lMasterEven = 1,long lSlaveEven = 1,short nIntervalTime = 0,double dAcc = 0.1,double dDec = 0.1,double dVel = 50,short nStopWaitTime = 0);
	int MC_EndHandwheel(short nAxisNum);

	//激光相关
	int MC_LaserPowerMode(short nChannelIndex,short nPowerMode,double dMaxValue,double dMinValue,short nDelayMode);
	int MC_LaserSetPower(short nChannelIndex,double dPower);
	int MC_LaserOn(short nChannelIndex);
	int MC_LaserOff(short nChannelIndex);
	int MC_LaserGetPowerAndOnOff(short nChannelIndex,double* dPower,short* pOnOff);
	int MC_LaserFollowRatio(short nChannelIndex,double dMinSpeed,double dMaxSpeed,double dMinPower,double dMaxPower,short nFifoIndex);

	//EtherCAT总线相关
	int MC_ECatInit();
	int MC_ECatGetInitStep(short* pCutInitSlaveNum,short* pMode,short* pModeStep);
	int MC_ECatGetSlaveCount(short* pCount);
	int MC_ECatSetPluseAxisNum(short* pAxisNum,short nCount);
	int MC_ECatSetAdoValue(short nStationNum,short nAdoAddr,short nAdoValue);
	int MC_ECatGetAdoValue(short nStationNum,short nAdoAddr,short *pAdoValue);
	int MC_ECatSetSdoValue(short nStationNum,short nSdoIndex,short nSdoSubIndex,long lSdoValue,short nLen);
	int MC_ECatGetSdoValue(short nStationNum,short nSdoIndex,short nSdoSubIndex,long *pSdoValue);
	int MC_ECatSetProbeCaptureStart(short nStationNum,short nProbeNum,short nProbeSource,short nProbeSense,short nContinueFlag,short nAutoStopFlag);
	int MC_ECatGetProbeCaptureStatus(short nStationNum,short nProbeNum,short* nSatus,long *pValueP,long *pValueF);
	int MC_ECatSetPDOConfig(short nStationNum,short nGroupNum,TECatPDOParm* pEcatPrm);
	int MC_ECatGetPDOConfig(short nStationNum,short nGroupNum,TECatPDOParm* pEcatPrm);
	int MC_ECatResetPDOConfig(short nStationNum);
	int MC_ECatLoadPDOConfig(short nStationNum);
	int MC_ECatSetCtrlBit(short nStationNum,unsigned long ulMask,unsigned long ulValue);
	int MC_ECatSetCtrlMode(short nStationNum,unsigned short nMode);
	int MC_ECatHomeStart(short nStationNum,short nHomeMode,double dHomeRapidVel,double dHomeLocatVel,double dHomeAcc,long lOffset,unsigned short nDelayTime);
	int MC_ECatHomeStop(unsigned long ulAxisMask);
	int MC_ECatGetStatusWord(short nStationNum,short *pEcatStatusValue);
	int MC_ECatSetAllPDOData(short nStationNum,unsigned char* pData,short nLen);
	int MC_ECatGetAllPDOData(short nStationNum,unsigned char* pData,short* pLen);

	int MC_ECatSetOrgPosAbs(short nStationNum,long lOrgPosAbs);
	int MC_ECatSetOrgPosCur(short nStationNum);
	int MC_ECatGetOrgPosAbs(short nStationNum,long* plOrgPosAbs);
	int MC_ECatLoadOrgPosAbs(short nStationNum);
	int MC_ECatGetDCOffset(long *lDCOffset);
	int MC_ECatSetDCOffset(short nStationNum,long lDCOffset);

	//其他API
	int MC_GetIP(unsigned long* pIP);
	int MC_SetIP(unsigned long ulIP);
	int MC_GetID(unsigned long* pID);
	int MC_WriteInterFlash(unsigned char* pData,short nLength);
	int MC_ReadInterFlash(unsigned char*pData,short nLength);
	int MC_SetPLCShortD(long lAdd,short *pData,short nCount);
	int MC_GetPLCShortD(long lAdd,short *pData,short nCount);
	int MC_SetPLCLongD(long lAdd,long *pData,short nCount);
	int MC_GetPLCLongD(long lAdd,long *pData,short nCount);
	int MC_SetPLCFloatD(long lAdd,float *pData,short nCount);
	int MC_GetPLCFloatD(long lAdd,float *pData,short nCount);
	int MC_SetPLCM(long lAdd,char *pData,short nCount);
	int MC_GetPLCM(long lAdd,char *pData,short nCount);
	int MC_ResetAllM();
	int MC_GetCardMessage(char *cMessage);
	int MC_ClrCardMessage();
	int MC_SetCommuTimer(int iCommuTimer);
	int MC_SetKeepAlive(long AliveTime,long ulEnableMask,long ulEnableValue,short* nAdcMask,short* nAdcValue,long* ulIOMask,long* ulIOValue,short nModuleNum);
	int MC_DownPitchErrorTable(short nTableNum,short nPointNum,long lStartPos,long lEndPos,short *pErrValue1,short *pErrValue2);
	int MC_ReadPitchErrorTable(short nTableNum,short* pPointNum,long* pStartPos,long* pEndPos,short *pErrValue1,short *pErrValue2);
	int MC_AxisErrPitchOn(short nAxisNum);
	int MC_AxisErrPitchOff(short nAxisNum);
	int MC_StartWatch(long lAxisMask,long lPackageCountFlag,long lUserSegNumFlag,long lReserve,char *FilePath);
	int MC_StopWatch();
};

#pragma pack(pop)