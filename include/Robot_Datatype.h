#ifndef _ROBOT_DATATYPE_H_
#define _ROBOT_DATATYPE_H_

#include <stdint.h>

#define SIA_NO_ERROR 0
#define SIA_ERR_SNDTIMEOUT -1          /// 发送失败返回值，代表应答超时
#define SIA_ERR_NETDISCNT -2           /// 发送失败返回值，代表网络断开
#define SIA_ERR_PARAMINVALID -3        /// 参数无效
#define SIA_ERR_CHECKCODE -4           /// 返回数据校验码错误
#define SIA_ERR_DATAINVALID -5         /// 返回数据无效
#define SIA_ERR_NOTINITION -6          /// 未初始化
#define SIA_ERR_STATUSFAULT -9         /// 控制器状态错误
#define SIA_ERR_JOBSPACE_SHORTAGE -11  /// 作业空间已满
#define SIA_ERR_RUNNING -13            /// 机器人处于执行状态，禁止该功能  add by tongyan 20131224
#define SIA_ERR_REMOTE -14             /// 远程控制功能未启动  add by ytj 20150409
#define SIA_ERR_NOFUNCTION -15         /// rc不支持该功能 add by tongyan 20150818
#define SIA_ERR_OPENFILERROR -16       /// 配置文件打开失败 add by tongyan 20151119
#define SIA_ERR_RESPACKAGESIZEZERO -17 /// 接收数据包大小为0 add by tongyan 20151228
#define SIA_ERR_MESBODYLENERROR -18    /// 接收数据包消息体长度错误 add by tongyan 20151228
#define SIA_ERR_RESPACKAGENULL -19     /// 接收数据包对象为空 add by tongyan 20151228
#define SIA_ERR_CHILDCODE -20          /// 子功能码错误 add by tongyan 20151228
#define SIA_ERR_SOCKETID -21           /// SOCKEID错误，add by tongyan 20161028
#define SIA_ERR_LICENCEINVALID -22     /// 授权文件无效,add by tongyan 20170217
#define SIA_ERR_LOADSYSCONFIG -25      /// 加载系统配置文件失败
#define SIA_ERR_LOADINSTCONFIG -26     /// 加载指令配置文件失败
#define SIA_ERR_DATALENTH -30          /// 数据长度错误
#define SIA_ERR_JOBCONVERT -31         /// 作业转换失败 add by fjy 2024.3.27
#define SIA_ERR_JOBCONVERTSPACE -32    /// 文件路径不能包含空格 add by fjy 2024.4.10
#define SIA_ERR_JOBCONVERTINIT -33     /// 作业转换软件初始化失败 add by fjy 2024.5.11
#define SIA_ERR_LOGINIT -34            /// 日志文件创建失败 add by fjy 2024.7.12
#define SIA_ERR_NOTALLOWOPERATE -35    /// 机器人禁止此操作 add by tongyan 2024.8.14
#define SIA_ERR_ILLEGALJOBNAME -36     /// 作业名不合法 add by fjy 2025.2.6

#define SYSTEM_AXIS 12 // add by tongyan 20151119 ,兼容9轴和12轴
#define STR_SIZE 512
/**监控自动执行状态的函数指针*/
typedef int (*LPMONITOR_CALLBACK)(int Param1, int Param2);

/**监控机器人参数信息的函数指针，add by tongyan 20160801*/
typedef int (*RSMONITOR_CALLBACK)(unsigned char *dataStream, int streamLenth);

enum API_CODE
{
	API_READJOINT = 1,
	API_READPOSE,
	API_READTOOL,
	API_READALARM,
	API_READSTATUS,
	API_READUO,
	API_READGUO,
	API_READUI,
	API_READGUI,
	API_CLEARBUFFER,
	API_READJOBNAMELIST,
	API_RECEIVEJOB,
	API_READRCPARAM,
	API_DELJOB,
	API_CLEARJOBLIST,
	API_OPENJOB,
	API_WRITEUSERVARBYTE,
	API_WRITEUSERVARINT,
	API_WRITEUSERVARFLOAT,
	API_WRITEUSERVARRPY,
	API_WRITEUSERVARRPYS,
	API_ARCON,
	API_ARCOFF,
	API_SENDJOBNET,
	API_WAIT,
	API_DO_OUT,
	API_DGO_OUT,
	API_DGO_OUT8,
	API_WRITEGUO,
	API_WRITEUO,
	API_SIMWRITEGUI,
	API_SIMWRITEUI,
	API_DELAY,
	API_JOBRUMMODE,
	API_READ_MAINJOPEN,
	API_READ_SAFEDOOR,
	API_READ_ONPATH,
	API_READ_POSIRION1,
	API_READ_POSITION2,
	API_READ_WARNING,
	API_READ_ONLINE,
	API_READ_AUTORUN,
	API_READ_POWERON,
	API_READ_PLAY,
	API_READ_MAINJOBRUN,
	API_READ_TEACH,
	API_MOVL_P2P_RPY,
	API_MOVL_CP_RPY,
	API_READ_POSERPY,
	API_READ_BYTE_USERVAR,
	API_READ_INT_USERVAR,
	API_READ_FLOAT_USERVAR,
	API_READ_RPY_USERVAR,
	API_READ_INT_SYSVAR,
	API_SEND_OPEN,
	API_SEND_RESET,
	API_SEND_PAUSE,
	API_SEND_STOP,
	API_SEND_SAFEDOOROFF,
	API_SEND_POWERON,
	API_SEND_START,
	API_SEND_STACKFLAGDATA,
	API_READ_JOINTBASENOAP,
	API_SEND_STACKPRODATA,
	API_SEND_STACKCOUNT,
	API_SEND_STACKPOINTDATA,
	API_SEND_STACKPOINTDATA2,
	API_SET_AXIS_INTERF_DATA,
	API_SET_CUBE_INTERF_DATA,
	API_SEND_SHIFTPON,
	API_SEND_SHIFTPONOFF,
	API_SET_USERCOORD,
	API_SET_TOOLCOORD,
	API_READ_FROCECONTROLDATA,
	API_READ_VISUALSENSEDATA,
	API_READ_CURSTACKCOUNT,
	API_SEND_STACKFILENUM,
	API_READTORQUE,
	API_READCURRJOBNAME,
	API_WREFP,
	API_WAVEREF,
	API_READPOSE2,
	API_READ_POSERPY2,
	API_READWELDPARA,
	API_READSTARTTIME,
	API_READRUNNINGTIME,
	API_READSPEED,
	API_READUSERCOORD,
	API_READJOGCOORDINATE,
	API_SETJOGCOORDINATE,
	API_READAXIS,
	API_SETAXIS,
	API_READSPEEDSCALE,
	API_SETSPEEDSCALE,
	API_SENDJOGMOTION,
	API_SENDCREATEJOB,
	API_SENDCONVERTJOB,
	API_SENDPOSDATA,
	API_READREMAINOFFLINEBUFFSIZE,
	API_ARCON_UI,
	API_ARCON_UI2,
	API_ARCOFF_UI,
	API_ARCOFF_UI2,
	API_PARA_S_UI,
	API_PARA_S_UI2,
	API_SENDSTEPMOTION,
	API_READCURMODE,
	API_READRUNMODE,
	API_READJOBOPEN,
	API_JOINT2POS,
	API_POS2JOINT,
	API_JUDGEPOS,
	API_READALARMINFO,
	API_CLEARALARM,
	API_READCURJOBINFO,
	API_CLEARJOBBUFFER,
	API_READUOMULTI,
	API_READGUOMULTI,
	API_READUIMULTI,
	API_READGUIMULTI,
	API_READUSERVARBYTEMULTI,
	API_READUSERVARINTMULTI,
	API_READUSERVARFLOATMULTI,
	API_READUSERVARRPYMULTI,
	API_READSYSVARINTMULTI,
	API_WRITEUOMULTI,
	API_WRITEGUOMULTI,
	API_SIMWRITEUIMULTI,
	API_SIMWRITEGUIMULTI,
	API_WRITEUSERVARBYTEMULTI,
	API_WRITEUSERVARINTMULTI,
	API_WRITEUSERVARFLOATMULTI,
	API_WRITEUSERVARRPYMULTI,
	API_WRITEUSERVARRPYSMULTI,
	API_AMENDON,
	API_AMENDOFF,
	API_TRACKON_ARC,
	API_TRACKON_LASER_ABSOLUTE,
	API_TRACKON_LASER_RELATIVE,
	API_TRACKON_ARC_LASER_ABSOLUTE,
	API_TRACKON_ARC_LASER_RELATIVE,
	API_TRACKOFF_ARC,
	API_TRACKOFF_LASER,
	API_TRACKOFF_ARC_LASER,
	API_ARCJ_P,
	API_ARCC_P,
	API_ARCL_P, 
	API_WAVEON_SIN,
	API_WAVEON_MOON,
	API_WAVEON_ANGLE,
	API_WAVEON_L,
	API_WAVEON_CIRCLE,
	API_WAVEON_8,
	API_WAVEON,
	API_WAVEOFF,
	ARI_READCURNUM,
	API_SETCURTOOL,
	API_SETCURUSER,
	API_SENDFWDMOTION,
	API_SETINTERLOCK,
	API_SENDPLAYSTART,
	API_SENDPLAYPAUSE,
	API_READINTERLOCK,
	API_READBGJOB,
	API_MOVL_P,
	API_MOVL_RPY,
	API_MOVJ_P,
	API_MOVJ2,
	API_MOVJ_RPY,
	API_READUSERCOORD_RPY,
	API_MOVC_P,
	API_MOVC_RPY,
	API_ARCL_RPY,
	API_ARCJ_RPY,
	API_ARCC_RPY,
	API_ARCL_P_COORD,
	API_ARCC_P_COORD,
	API_ARCL_RPY_COORD,
	API_ARCC_RPY_COORD,
	API_READUSERVARSTR,
	API_READUSERVARSTRMULTI,
	API_WRITEUSERVARSTR,
	API_WRITEUSERVARSTRMULTI,
	API_SETETOOLCOORDINATE,
	API_SETWORLDCOORDDINATE,
	API_READETCOORD,
	API_READWORLDCOORDINATE,
	API_ARCJ_P_INTERLOCK,
	API_ARCJ_RPY_INTERLOCK,
	API_ARCL_P_INTERLOCK,
	API_ARCL_RPY_INTERLOCK,
	API_ARCL_P_COORD_INTERLOCK,
	API_ARCL_RPY_COORD_INTERLOCK,
	API_ARCC_P_INTERLOCK,
	API_ARCC_RPY_INTERLOCK,
	API_ARCC_P_COORD_INTERLOCK,
	API_ARCC_RPY_COORD_INTERLOCK,
	API_MOVJ_P_INTERLOCK,
	API_MOVJ_RPY_INTERLOCK,
	API_MOVL_P_INTERLOCK,
	API_MOVL_RPY_INTERLOCK,
	API_MOVC_P_INTERLOCK,
	API_MOVC_RPY_INTERLOCK,
	API_MOVL_P_ET_INTERLOCK,
	API_MOVL_RPY_ET_INTERLOCK,
	API_MOVC_P_ET_INTERLOCK,
	API_MOVC_RPY_ET_INTERLOCK,
	API_MOVL_P_ET,
	API_MOVL_RPY_ET,
	API_MOVC_P_ET,
	API_MOVC_RPY_ET, 
	API_READ_WELD_WAVEINFO, 
	API_SEND_WELD_WAVEINFO1, 
	API_SEND_WELD_WAVEINFO2,
	API_SEND_WELD_WAVEINFO3
};

// add by tongyan 20131111
typedef struct t_system_axes
{
    int r_axes;   // robot axes
    int s_axes;   // slider axes
    int p_axes;   // positioner axer
	int sys_axes; // system axes = r_axes+ s_axes+ p_axes
} T_SYSTEM_AXES;

typedef struct toolfile
{
	float Px;
	float Rx;
	float Py;
	float Ry;
	float Pz;
	float Rz;
} TOOLF;

typedef struct vec
{
	float x;
	float y;
	float z;
} VEC;

typedef struct trans
{
	VEC N;
	VEC O;
	VEC A;
	VEC P;
} TRANS;

typedef struct set_trdla
{
    unsigned char enable;      // enable flag
	int pc_trdla[SYSTEM_AXIS]; // RC->PC  pulse control tracks delta
} SET_TRDLA;

// RC参数结构体
typedef struct t_send_robot_para
{
    char VER_DISP_PART1[60];   // 系统号
    char VER_DISP_PART2[30];   // 版本号
    char cToolNo;              // 当前工具坐标系
    unsigned char cUserNo;     // 当前用户坐标系
    T_SYSTEM_AXES system_axes; // 系统轴数配置
    TRANS WPT6;                // 工件坐标系
    TOOLF ToolCFile[32];       // 工具坐标系
    TRANS UserCFile[128];      // 用户坐标系
    int Kepc[SYSTEM_AXIS];              // 码盘限
    int rc_trdla[SYSTEM_AXIS];          // RC跟踪差
    int rc_sp_limit[SYSTEM_AXIS];       // RC速度限
    int max_trdla[SYSTEM_AXIS];         // 设置最大值
    float Kr[SYSTEM_AXIS];              // 减速比
    float JtMin[SYSTEM_AXIS];           // 软限位
    float JtMax[SYSTEM_AXIS];
    float JtVmax[SYSTEM_AXIS];            // 最大速度
    float JtAmax[SYSTEM_AXIS];            // 最大加速度
    SET_TRDLA setTrdlaPara;      // 设置跟踪差参数
    unsigned short wRealDI[272]; // IO输入
    unsigned short wRealDO[272]; // IO输出
    float fc_sensor[6];          // 力传感器反馈值
} T_SEND_ROBOT_PARA;
// add end

// Add by yuantianjian 20140902
// 位置变量RPY
typedef struct rpy
{
	float px; // 位置变量x
	float py; // 位置变量y
	float pz; // 位置变量z
	float rx; // 方向变量x
	float ry; // 方向变量y
	float rz; // 方向变量z
	int mWord;
} RPY;

// 参数类型
enum ParameterType
{
	BYTE_1,
	SHORT_2,
	INT_4,
	FLOAT_4,
	CHAR_1
};

// 指令参数
typedef struct inst_Para
{
	ParameterType paraType;
	int typeLength;
	char val[100];
	char minVal[100];
	char maxVal[100];
} INST_PARA;

// 指令
typedef struct inst
{
	short code;
	int paraLen;
	int paraNum;
	INST_PARA para[20];
} INST;
#pragma pack(push, 4)
typedef struct arcweld_feedback
{
    double welder_current;       // 焊机反馈电流
    double welder_voltage;       // 焊机反馈电压
    double welder_wirespeed;     // 焊机反馈送丝速度
    double welder_wirecurrent;   // 焊机反馈送丝机电流
    float welder_lagtime;        // 接触传感延迟时间
    float welder_current2;       // 双丝焊焊机反馈电流2
    float welder_voltage2;       // 双丝焊焊机反馈电压2
    float welder_wirespeed2;     // 双丝焊焊机反馈送丝速度2
    float welder_wirecurrent2;   // 双丝焊焊机反馈送丝机电流2
    float set_welder_current;    // 设置焊机电流
    float set_welder_voltage;    // 设置焊机电压
    float set_welder_wirespeed;  // 设置焊机送丝速度
    float set_welder_current2;   // 设置焊机电流2
    float set_welder_voltage2;   // 设置焊机电压2
	float set_welder_wirespeed2; // 设置焊机送丝速度2
    int welder_error_code;       // 焊机反馈错误码
    int welder_error_code2;      // 焊机反馈错误码2
    double welder_current_track; // 焊机反馈跟踪电流
    int s_iRcvBatCnt;            //
} ARCWELD_FEEDBACK;

typedef struct wave_file_param_type1
{
	unsigned char wave_type;			//摆动类型（0-7，自定义与默认7种）
	float frequency;			//频率（0-100HZ）
	float amplitude;			//振幅（0-100mm）
	float left_delay_time;		//左停留时间（0-100s）
	float right_delay_time;		//右停留时间（0-100s）
	float Rx;					//倾斜角（0-90°）				
	float Rz;					//方位角（0-90°） 
	unsigned char stay_delay_type;		//停留延迟类型（0：移动；1：停留）
	unsigned char wave_connect_type;	//摆焊连接（0：不过示教点；1：过示教点）
}Wave_File_Param_TYPE1;

typedef struct wave_file_param_type2
{
	unsigned char wave_type;			//摆动类型（0-7，自定义与默认7种）
	float frequency;			//频率（0-100HZ）
	float amplitude;			//振幅（0-100mm）
	float Rx;					//倾斜角（0-90°）				
	float Rz;					//方位角（0-90°） 
	float rad;					//半径（0-100mm）
	unsigned char wave_connect_type;	//摆焊连接（0：不过示教点；1：过示教点）
}Wave_File_Param_TYPE2;



//摆动库结构体
#define NAME_LENGTH 25
#define LIBNUM_SAVE 15
typedef struct wavelib_node
{
	char time;			//时间
	char x;				//前后X
	char y;				//横向Y
	char z;				//垂直Z
	int angle;			//角度
	int stay_time_type;	//0：不停留；1：左停留时间；2：右停留时间
} WAVELIB_NODE;

typedef struct wave_lib
{
	char name[NAME_LENGTH];			//摆动库名称
	unsigned int year;					//年月日
	unsigned int mon;
	unsigned int day;
	WAVELIB_NODE node[LIBNUM_SAVE];	//摆动库节点参数（15个节点）
}WAVE_LIB;

typedef struct wave_file_param_type3
{
	unsigned char wave_type;			//摆动类型（0-7，自定义与默认7种）
	int wavelib_num;			//摆动库号（1-110）
	float frequency;			//频率（0-100HZ）
	float amplitude;			//振幅（0-100mm）
	float left_delay_time;		//左停留时间（0-100s）
	float right_delay_time;		//右停留时间（0-100s）
	float Rx;					//倾斜角（0-90°）				
	float Rz;					//方位角（0-90°）
	float rad;					//半径（0-100mm） 
	unsigned char stay_delay_type;		//停留延迟类型（0：移动；1：停留）
	unsigned char wave_connect_type;	//摆焊连接（0：不过示教点；1：过示教点）
}Wave_File_Param_TYPE3;

#pragma pack(pop)
// 垛型枚举
typedef enum stack_type
{
	stack_type1 = 0,
	stack_type2,
	stack_type3,
	stack_type4,
	stack_type5,
	stack_type6,
} STACK_TYPE;

// 码垛数据结构体
typedef struct pall_file_pro
{
	float product_length; // 产品长
    float product_wide;   // 产品宽
	float product_height; // 产品高
	float product_weight; // 产品重量
	float pallet_length;  // 托盘长
    float pallet_wide;    // 托盘宽
	float pallet_height;  // 托盘高
    float row_space;      // 行间距
    float col_space;      // 列间距
	STACK_TYPE stacktype; // 垛型
    int stacklayernum;    // 垛型层数
    int stackrownum;      // 行数
    int stackcolnum;      // 列数
    int verirownum;       // 型1行数
    int vericolnum;       // 型1列数
    int horirownum;       // 型2行数
    int horicolnum;       // 型2列数
    int startform;        // 起始形式
    int imageflag;        // 镜像开关
} PALL_FILE_PRO;

// 计数结构体
typedef struct pall_file_counter
{
	int flagcountnum; // 个数计数器
    int flagrownum;   // 行数计数器
    int flagcolnum;   // 列数计数器
	int flaglayernum; // 层数计数器
	int flagstepnum;  // 步数计数器
} PALL_FILE_COUNTER;

// 码垛标志结构体
typedef struct pall_flag_msg
{
    short filenum;    // 文件号
	int realtotalnum; // 实际的码垛的总数目
    int breakflag;    // 拆码标志
    int stackcount;   // 码垛初始计数
    int breakcount;   // 拆垛初始计数
} PALL_FLAG_MSG;

// 轴干涉区结构体
typedef struct AxisInterf_Data
{
    unsigned char enable;        // 使能状态1为使能，即干涉区起作用，0为没有使能
	unsigned char reserved[105]; // 预留数据
} AXIS_INTERF_DATA;

// 立体干涉区结构体--更改结构体内部内容 20231019 yhy
typedef struct CubeInterf_Data
{
    unsigned char shape;     // 干涉区形状，1：立方体 2：圆台体  3：球 4：交集（只包括1,2,3） 5：并集（可包括1,2,3,4）
    unsigned char enable;    // 使能标志，0：不使能	1：使能
	unsigned char coordtype; // 坐标系类型，1：世界坐标系  2：用户坐标系
    unsigned char coordnum;  // 坐标系号，世界坐标系默认为1，用户坐标系可配置1-8
    unsigned char tool_num;  // 工具坐标系号
    float Xmin;              // 对应立方体形状的参数
	float Ymin;
	float Zmin;
	float Xmax;
	float Ymax;
	float Zmax;
    float Xorig;       // 下底面圆心x分量
    float Yorig;       // 下底面圆心y分量
    float Zorig;       // 下底面圆心z分量
    float Rdown;       // 下底面半径
    float Rup;         // 上底面半径
    float h;           // 高
    float Xsphere;     // 球心x分量
    float Ysphere;     // 球心y分量
    float Zsphere;     // 球心z分量
    float Rsphere;     // 球体半径
	char sub_area[8];  // 交集和并集的子区域号
	float reserve[12]; // 预留
} CUBE_INTERF_DATA;

// 机器人位姿结构体
typedef struct RobPos
{
	float N[3];
	float O[3];
	float A[3];
	float P[3];
	float exAxis[6];
} ROB_POS;

// 机器人位姿结构体
typedef struct RobPosRPY
{
	float px; // 位置变量x
	float py; // 位置变量y
	float pz; // 位置变量z
	float rx; // 方向变量x
	float ry; // 方向变量y
	float rz; // 方向变量z
	float exAxis[6];
} ROB_POS_RPY;


typedef struct Torque_Data
{
	short Real[12];
	short Ideal[12];
} TORQUE_DATA;

typedef struct Time_Data
{
    int year;   // 年
    int month;  // 月
    int day;    // 天
    int hour;   // 小时
	int minute; // 分
    int sec;    // 秒
} TIME_DATA;

// 运动类型枚举
typedef enum Motion_Type
{
	NOSEL,
	MOVJ,
	MOVL,
	MOVC,
	SMOVL,
	SMOVC,
	MOVSPI,
} MOTION_TYPE;

// 速度数据
typedef struct Speed_Data
{
	MOTION_TYPE type; // 运动类型
    float speed[12];  // 机器人速度
} SPEED_DATA;

// 报警信息
typedef struct AlarmInfo
{
    char time[30];     // 报警时刻
    int code;          // 报警码
	char content[580]; // 报警内容
    int level;         // 报警级别（1：提示，2：报警，3：警告）
	char solution[50]; // 报警对策
	char reason[50];   // 报警原因

} AlarmInfo;

// 作业状态
typedef struct JobInfo
{
    char name[33];       // 作业名
	char parentName[33]; // 父作业名
    int lineNum;         // 执行行号
    int status;          // 作业状态（0：停止，1：暂停，2：运行中：3：报警）
    int posNum;          // 正在运动的P变量编号或者最后一次运动完成的P变量编号
} JobInfo;

#endif
