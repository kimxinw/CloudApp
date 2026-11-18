#ifndef _ROBOTSDK_H_
#define _ROBOTSDK_H_

#ifdef _WIN32
#ifdef ROBOTSDK_EXPORTS
#define ROBOTSDK_API extern "C" __declspec(dllexport)
#else
#define ROBOTSDK_API extern "C" __declspec(dllimport)
#endif
#elif defined(__linux__)
// 设置特定的函数或变量公开，即可从外部链接
#if defined(ROBOTSDK_EXPORTS)
#define ROBOTSDK_API extern "C" __attribute__((visibility("default")))
#else
#define ROBOTSDK_API extern "C"
#endif
#endif

#include "Robot_Datatype.h"

ROBOTSDK_API int RobotInit();
ROBOTSDK_API int RobotRelease();

/*********************网络接口*******************************/

/**连接控制器.
@param StrIP(in) 控制器IP地址
@param port (in) 端口号
@return 若成功返回当前IP的网络连接ID,ID值大于0，否则返回错误码
*/
ROBOTSDK_API int ConnectUDP(const char *StrIP, short port);
/**断开网络连接.
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int Disconnect(int ConnectID);

/*********************运动类指令*******************************/


/*********************查询接口********************************/

/**查询当前关节值.
@param pJoint (out) 关节值存放地址
@param JointNum (in) 要查询的关节数
@param ConnectID(in) 网络连接ID，Connect(const char* StrIP,short port)的返回值
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadJoint(float *pJoint, int JointNum, int ConnectID);

/**查询当前工具末端TCP点的姿态,分别为N,O,A,P，各自由三个浮点数表示.
@param exAxis (out)返回三个外部轴结果
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadPOSE(float *N, float *O, float *A, float *P, float *exAxis, int &mWord, int ConnectID);

/**查询当前工具末端TCP点的姿态,RPY形式
@param rpy (out) 位置变量结构体
@param exAxis (out)返回三个外部轴结果
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadPOSE_RPY(RPY *rpy, float *exAxis, int ConnectID);

/**查询当前工具末端TCP点的姿态,分别为N,O,A,P，各自由三个浮点数表示.
@param exAxis (out)返回三个外部轴结果
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadPOSE2(float *N, float *O, float *A, float *P, float *exAxis, int &mWord, int coordinate, int TOOLNUM, int USERNUM, int ConnectID);

/**查询当前工具末端TCP点的姿态,RPY形式
@param rpy (out) 位置变量结构体
@param exAxis (out)返回三个外部轴结果
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadPOSE_RPY2(RPY *rpy, float *exAxis, int coordinate, int TOOLNUM, int USERNUM, int ConnectID);

/**查询当前使用的工具号
@param ToolIndex (in) 工具号
@param Px (out) 工具坐标参数Px分量
@param Py (out) 工具坐标参数Py分量
@param Pz (out) 工具坐标参数Pz分量
@param Rx (out) 工具坐标参数Rx分量
@param Ry (out) 工具坐标参数Ry分量
@param Rz (out) 工具坐标参数Rz分量
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadTool(unsigned char ToolIndex, float &Px, float &Py, float &Pz, float &Rx, float &Ry, float &Rz, int ConnectID);

/**读报警.
@param ErrCode(out) 报警码
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadAlarm(int &ErrCode, int ConnectID);

/**读单个用户IO输出状态.
@param IOMap (in) IO索引号
@param IOStatus (out) 等待的IO状态
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUO(int IOMap, unsigned char &IOStatus, int ConnectID);

/**读用户IO输出.
@param IOStatus (out) 8位或16位组IO状态
@param Channel (in) IO通道号 1~8
@param IOGroupFlag (in) IO组标识
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadGUO(int &IOStatus, int Channel, int IOGroupFlag, int ConnectID);

/**读单个用户IO输入.
@param IOMap (in) IO索引号
@param IOStatus (out) 等待的IO状态
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUI(int IOMap, unsigned char &IOStatus, int ConnectID);

/**读用户IO输入.
@param IOStatus (out) 8位或16位组IO状态
@param Channel (in) IO通道号 1~8
@param IOGroupFlag (in) IO组标识
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadGUI(int &IOStatus, int Channel, int IOGroupFlag, int ConnectID);

/**查询RC参数
@param rcParam (out) RC参数结构体指针
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadRCParam(T_SEND_ROBOT_PARA *rcParam, int ConnectID);

/**查询RC实际转矩值和理论转矩值
@param torque (out)
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadTorque(TORQUE_DATA *torque, int ConnectID);
/***********************执行接口***********************************************/

/**清空控制器缓存的所有指令.
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ClearBuffer(int ConnectID);

/**网口下载作业 modify by fjy 24.9.6
@param jobStream (in) 作业数据流
@param jobLen (in) 作业数据流长度
@param dataStream (in) 局部变量数据流
@param dataLen (in) 局部变量数据流长度
@param ConnectID（in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendJobNet(unsigned char *jobStream, int jobLen, unsigned char *dataStream, int dataLen, int ConnectID);

/**查询RC现有作业列表
@param jobNameList (out) RC现有作业列表
@param jobNumber (out) 现有作业的个数
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadJobNameList(char *jobNameList, int &jobNumber, int ConnectID);

/**上传作业  modify by fjy 24.9.6
@param jobName (in) 指定的作业名称
@param jobStream (out) 作业数据流
@param jobLen (out) 作业数据流的长度
@param dataStream (out) 局部变量数据流
@param dataLen (out) 局部变量数据流长度
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReceiveJob(char *jobName, unsigned char *jobStream, int &jobLen, unsigned char *dataStream, int &dataLen, int ConnectID);

/**删除指定作业
@param jobName (in) 指定的作业名称
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int DelJob(char *jobName, int ConnectID);

/**清空作业列表
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ClearJobList(int ConnectID);

/**打开指定作业
@param jobName (in) 指定的作业名称
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int OpenJob(char *jobName, int ConnectID);

/**发送字节型用户变量
@param varID(in) 变量编号
@param varBYTE(in) 变量值
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarBYTE(int varID, unsigned char varBYTE, int ConnectID);
/**发送32位整型用户变量
@param varID(in) 变量编号
@param varFloat(in) 变量值
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarINT(int varID, int varINT, int ConnectID);
/**发送浮点型用户变量
@param varID(in) 变量编号
@param varINT(in) 变量值
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarFLOAT(int varID, float varFloat, int ConnectID);

/**发送位置变量（RPY中的mWord起作用）
@param varRPY(in) 位置变量结构体
@param exAxis(in) RC外部轴数据组
@param coordinate(in) 坐标系 1~2
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarRPY(int varID, RPY varRPY, float *exAxis, int coordinate, int ConnectID);

/**发送位置变量（RPY中的mWord不起作用）
@param varRPY(in) 位置变量结构体
@param exAxis(in) RC外部轴数据组
@param coordinate(in) 坐标系 1~2
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarRPYS(int varID, RPY varRPY, float *exAxis, int coordinate, int ConnectID);

/**读状态
@param statusCode(out) 状态码
@param ConnectID(in)网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadStatus(unsigned int &statusCode, int ConnectID);

/**读运动坐标系
@param coordinate(out) 坐标系 1关节坐标系，2世界坐标系，3工具坐标系，4用户坐标系
@param ConnectID(in)网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadJogCoordinate(int &coordinate, int ConnectID);

/**设置运动坐标系
@param coordinate(in) 坐标系 1关节坐标系，2世界坐标系，3工具坐标系，4用户坐标系
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetJogCoordinate(int coordinate, int ConnectID);

/**读轴组
@param axis(out) 轴组 0本体轴，1外部轴
@param ConnectID(in)网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadAxis(int &axis, int ConnectID);

/**设置轴组
@param axis(in) 轴组 0本体轴，1外部轴
@param ConnectID(in)网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetAxis(int axis, int ConnectID);

/**读速度百分比
@param speedScale(out) 速度百分比
@param ConnectID(in)网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadSpeedScale(float &speedScale, int ConnectID);

/**设置速度百分比
@param speedScale(in) 速度百分比
@param ConnectID(in)网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetSpeedScale(float speedScale, int ConnectID);

/**发送Jog运动（示教模式）
@param type(in) 类型 关节坐标系时：1:J1，2:J2，3:3，4:J4，5:J5，6:J6，有外部轴时外部轴从7开始向后顺延；直角坐标系时：1:X，2:Y，3:Z，4:Rx，5:Ry，6:Rz
@param direction(in) 方向 1正向，2反向
@param ConnectID(in)网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SendJogMotion(int type, int direction, int ConnectID);

/**发送寸动运动
@param type(in) 类型 关节坐标系时：1:J1，2:J2，3:3，4:J4，5:J5，6:J6，有外部轴时外部轴从7开始向后顺延；直角坐标系时：1:X，2:Y，3:Z
@param distance(in) 移动距离，符号代表方向（>0正向，<0反向），单位mm或°
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SendStepMotion(int type, float distance, int ConnectID);

/**查询机器人当前工作模式
@param mode(out) 工作模式，1：示教模式T1，2：示教模式T2，3：执行模式，4：远程模式
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadCurMode(int &mode, int ConnectID);

/**查询当前作业运行方式
@param mode(out) 作业运行方式，1：自动，2：单循环，3：单步
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadRunMode(int &mode, int ConnectID);

/**查询指定作业打开状态
@param jobName(in) 指定的作业名称
@param status(out) 作业打开状态 1：打开，0：未打开
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadJobOpen(char *jobName, int &status, int ConnectID);

/**机器人运动学正解
@param joint(in) 关节值
@param rpy(out) 转换后RPY坐标
@param toolCoor(in) 工具坐标系RPY值
@param userCoor(in) 用户坐标系RPY值
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int Joint2Pos(float *joint, RPY *rpy, RPY toolCoor, RPY userCoor, int ConnectID);

/**机器人运动学反解
@param rpy(in) RPY坐标值
@param refJoint(in) 逆解参考关节值
@param joint(out) 转换后关节值
@param toolCoor(in) 工具坐标系RPY值
@param userCoor(in) 用户坐标系RPY值
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int Pos2Joint(RPY rpy, float *refJoint, float *joint, RPY toolCoor, RPY userCoor, int ConnectID);

/**判断位姿是否可达
@param rpy(in) RPY坐标值
@param toolCoor(in) 工具坐标系RPY值
@param userCoor(in) 用户坐标系RPY值
@param refJoint(in) 参考关节值
@param result(out) 机器人是否能到达，1：可以到达，2：无法到达
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int JudgePos(RPY rpy, RPY toolCoor, RPY userCoor, float *refJoint, int &result, int ConnectID);

/**读取报警信息
@param alarm(out) 报警信息结构体数组，长度为报警数量
@param count(in) 报警数量，调用后返回实际查询到的报警数量
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadAlarmInfo(AlarmInfo *alarm, int &count, int ConnectID);

/**清除报警信息
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ClearAlarm(int ConnectID);

/**查询作业状态
@param jobinfo(out) 作业状态结构体
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadCurJobInfo(JobInfo *jobinfo, int ConnectID);

/**清空作业堆栈
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ClearJobBuffer(int ConnectID);

/**批量读取用户IO输出
@param IOMap(in) 变量起始编号
@param count(in) 变量个数
@param IOStatus(out) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUOMulti(int IOMap, int count, unsigned char *IOStatus, int ConnectID);

/**批量读取用户组IO输出
@param channel(in) IO通道号起始编号
@param count(in) 变量个数
@param IOGroupFlag(in) IO组标识
@param IOStatus(out) 组IO状态值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadGUOMulti(int channel, int count, int IOGroupFlag, int *IOStatus, int ConnectID);

/**批量读取用户IO输入
@param IOMap(in) 变量起始编号
@param count(in) 变量个数
@param IOStatus(out) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUIMulti(int IOMap, int count, unsigned char *IOStatus, int ConnectID);

/**批量读取用户组IO输入
@param channel(in) IO通道号起始编号
@param count(in) 变量个数
@param IOGroupFlag(in) IO组标识
@param IOStatus(out) 组IO状态值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadGUIMulti(int cchannel, int count, int IOGroupFlag, int *IOStatus, int ConnectID);

/**批量读字节型用户变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varBYTE(out) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarBYTEMulti(int varID, int count, unsigned char *varBYTE, int ConnectID);

/**批量读整型用户变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varINT(out) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarINTMulti(int varID, int count, int *varINT, int ConnectID);

/**批量读浮点型用户变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varFloat(out) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarFLOATMulti(int varID, int count, float *varFloat, int ConnectID);

/**批量读位置变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varRPY(out) 位置变量结构体数组，长度为变量个数
@param exAxis(out) RC外部轴数据数组，长度为外部轴数*变量个数
@param coordinate(out) 坐标系数组，长度为变量个数（1关节坐标系，2世界坐标系）
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarRPYMulti(int varID, int count, RPY *varRPY, float *exAxis, int *coordinate, int ConnectID);

/**批量读整型系统变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varINT(out) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadSysVarINTMulti(int varID, int count, int *varINT, int ConnectID);

/**批量设置用户IO输出
@param IOMap(in) 变量起始编号
@param count(in) 变量个数
@param IOStatus(in) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUOMulti(int IOMap, int count, unsigned char *IOStatus, int ConnectID);

/**批量设置用户组IO输出
@param channel(in) IO通道号起始编号
@param count(in) 变量个数
@param IOGroupFlag(in) IO组标识
@param IOStatus(in) 组IO状态值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteGUOMulti(int channel, int count, int IOGroupFlag, int *IOStatus, int ConnectID);

/**批量仿真用户IO输入
@param IOMap(in) 变量起始编号
@param count(in) 变量个数
@param IOStatus(in) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SimWriteUIMulti(int IOMap, int count, unsigned char *IOStatus, int ConnectID);

/**批量仿真用户组IO输入
@param channel(in) IO通道号起始编号
@param count(in) 变量个数
@param IOGroupFlag(in) IO组标识
@param IOStatus(in) 组IO状态值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SimWriteGUIMulti(int Channel, int count, int IOGroupFlag, int *IOStatus, int ConnectID);

/**批量发送字节型用户变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varBYTE(in) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarBYTEMulti(int varID, int count, unsigned char *varBYTE, int ConnectID);

/**批量发送整型用户变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varrINT(in) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarINTMulti(int varID, int count, int *varINT, int ConnectID);

/**批量发送浮点型用户变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varFloat(in) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarFLOATMulti(int varID, int count, float *varFloat, int ConnectID);

/**批量发送位置变量（RPY中的mWord起作用）
@param varID(in) 位置变量起始编号
@param count(in) 位置变量数量
@param varRPY(in) 位置变量结构体数组 长度为变量个数
@param exAxis(in) RC外部轴数据数组 长度为外部轴数*变量个数
@param coordinate(in) 坐标系数组，长度为变量个数（1关节坐标系，2世界坐标系）
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarRPYMulti(int varID, int count, RPY *varRPY, float *exAxis, int *coordinate, int ConnectID);

/**批量发送位置变量（RPY中的mWord不起作用）
@param varID(in) 位置变量起始编号
@param count(in) 位置变量数量
@param varRPY(in) 位置变量结构体数组 长度为变量个数
@param exAxis(in) RC外部轴数据数组 长度为外部轴数*变量个数
@param coordinate(in) 坐标系数组，长度为变量个数（1关节坐标系，2世界坐标系）
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarRPYSMulti(int varID, int count, RPY *varRPY, float *exAxis, int *coordinate, int ConnectID);

/**由文本文件创建作业
@param filePath(in) 文件路径
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int CreateJob(char *filePath);

/**作业文件格式转换 txt->job 或 job->txt
@param filePath(in) 文件路径
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ConvertJob(char *filePath);

/***********************IO类指令**********************************************/

/**wait指令.
@param IOMap (in) IO索引号
@param IOStatus (in) 等待的IO状态
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int Wait(int IoMap, unsigned char IoStatus, int ConnectID);

/**IO输出指令,在控制器端需要缓存到指令执行队列.
@param IoMap (in) 用户IO索引号
@param IoStatus (in) IO状态，1:ON，0:OFF
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int DO_Out(int IOMap, unsigned char IOstatus, int ConnectID);

/***16位组IO输出指令,在控制器端需要缓存到指令执行队列.
@param IOStatus (in)  16位IO状态
@param Channel (in) IO通道号 1~64
@param IOGroupFlag (in) IO组标识
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int DGO_Out(int IOStatus, int Channel, int ConnectID);

/***8位组IO输出指令,在控制器端需要缓存到指令执行队列.
@param IOStatus (in)  8位IO状态
@param Channel (in) IO通道号 1~8
@param IOGroupFlag (in) IO组标识
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int DGO_Out8(int IOStatus, int Channel, int ConnectID);

/**设置用户组IO输出.
@brief 控制器收到该命令后立即执行，不缓存
@param IOStatus (in)  8位或者16位IO状态
@param Channel (in) IO通道号 8位：1~8 16位：1~64
@param IOGroupFlag (in) IO组标识
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteGUO(int IOStatus, int Channel, int IOGroupFlag, int ConnectID);

/**设置单个用户IO输出.
@brief 控制器收到该命令后立即执行，不缓存
@param IOMap (in) 用户IO索引号
@param IOStatus (in)   IO状态，1:ON，0:OFF
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUO(int IOMap, unsigned char IOStatus, int ConnectID);

/**仿真用户组IO输入.
@brief 控制器收到该命令后立即执行，不缓存
@param IOStatus (in)  8位IO状态
@param Channel (in) IO通道号 8位：1~8 16位：1~64
@param IOGroupFlag (in) IO组标识
@return 若成功返回0，否则返回-1
*/
ROBOTSDK_API int SimWriteGUI(int IOStatus, int Channel, int IOGroupFlag, int ConnectID);

/**仿真用户IO输入.
@param IOMap (in) 用户IO索引号，见用户IO表
@param IOStatus (in)   IO状态，1:ON，0:OFF
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SimWriteUI(int IOMap, unsigned char IOStatus, int ConnectID);

/****************************设置类指令**********************************/

/**延迟指令.
@param DelayTime (in) 延迟时间，单位：S
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int Delay(float DelayTime, int ConnectID);

/**设置作业运行模式
@param JobRunMode (in) 作业运行模式，1：自动，2：一次，3：单步
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ChangeJobRunMode(unsigned char JobRunMode, int ConnectID);

/*设置当前作业执行状态反馈的回调函数.
@param func (in) 函数指针，由用户实现执行状态的处理
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetPlayMonitor(LPMONITOR_CALLBACK func, int ConnectID);
ROBOTSDK_API int SetSndstreamMonitor(LPMONITOR_CALLBACK func, int ConnectID);
ROBOTSDK_API int SetBackstreamMonitor(LPMONITOR_CALLBACK func, int ConnectID);

/****************************应用类指令(弧焊)**********************************/
/**弧焊直线运动（发送P变量）
@param pVarID (in) P变量编号
@param speed (in) 运动速度
@param cpFlag (in) 连续标识,0:非连续，大于0:连续
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
// ROBOTSDK_API int ARCL_P(short pVarID, float speed, short cpFlag, int ConnectID);

/**弧焊关节运动（发送P变量）
@param pVarID (in) P变量编号
@param speed (in) 运动速度
@param cpFlag (in) 连续标识,0:非连续，大于0:连续
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
// ROBOTSDK_API int ARCJ_P(short pVarID, float speed, short cpFlag, int ConnectID);

/**焊机1焊接开始
@param sARCFNum（in）起弧文件号
@param weldFNum（in）焊接文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCON(short sARCFNum, short weldFNum, int ConnectID);

/**焊机1焊接结束
@param eARCFNum（in）收弧文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCOFF(short eARCFNum, int ConnectID);

/**根据相应的系统环境指定相应的焊接基准面
@param type（in）焊接基准面Z方向类型,Z方向指定：1世界下z+方向；2世界下y+方向；3世界下x+方向；4焊枪（TCP）z-方向
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WREFP(short type, int ConnectID);

/**根据相应的系统环境指定相应的摆弧基准
@param type（in） 摆动基准类型：1 焊接基准面基准；2 焊枪基准（焊枪TCP z-方向）；3 焊枪进入方向基准（起点指向上一安全点方向）
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEREF(short type, int ConnectID);

// /**调用摆动库参数来进行摆动
// @param fileNum（in）摆动库文件号，107 横摆
// @param frequency（in）摆动频率
// @param amplitude（in）摆幅
// @param type（in）是否经过示教点
// @param ConnectID(in) 网络连接ID
// @return 若成功则返回0，否则返回错误码
// */
// ROBOTSDK_API int WVLIBON(short fileNum, float frequency, float amplitude, short type, int ConnectID);

/**摆动库的模式下摆弧停止
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEOFF(int ConnectID);

/**查询焊接参数
@param para（in）焊接参数结构体指针变量
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadWeldPara(ARCWELD_FEEDBACK *para, int ConnectID);

/**焊机1参数方式起弧
@param sARCFNum（in）起弧文件号
@param weldFNum（in）焊接文件号
@param VarI (in) 焊接电流
@param VarU (in) 焊接电压
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCON_UI(short sARCFNum, short weldFNum, float varI, float varU, int ConnectID);

/**焊机2参数方式起弧
@param sARCFNum（in）起弧文件号
@param weldFNum（in）焊接文件号
@param VarI (in) 焊接电流
@param VarU (in) 焊接电压
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCON_UI2(short sARCFNum, short weldFNum, float varI, float varU, int ConnectID);

/**焊机1通过参数方式息弧
@param eARCFNum (in) 收弧文件号
@param VarI (in) 收弧电流
@param VarU (in) 收弧电压
@param varTime (in) 收弧时间
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCOFF_UI(short eARCFNum, float varI, float varU, float varTime, int ConnectID);

/**焊机2通过参数方式息弧
@param eARCFNum (in) 收弧文件号
@param VarI (in) 收弧电流
@param VarU (in) 收弧电压
@param varTime (in) 收弧时间
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCOFF_UI2(short eARCFNum, float varI, float varU, float varTime, int ConnectID);

/**修改焊机1的焊接参数
@param weldFNum (in) 焊接文件号
@param VarI (in) 焊接电流
@param VarU (in) 焊接电压
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int PARA_S_UI(short weldFNum, float varI, float varU, int ConnectID);

/**修改焊机2的焊接参数
@param weldFNum (in) 焊接文件号
@param VarI (in) 焊接电流
@param VarU (in) 焊接电压
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int PARA_S_UI2(short weldFNum, float varI, float varU, int ConnectID);

/****************************远程控制类指令**********************************/

/**获得远程IO——主作业打开的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_MainJobOpen(int ConnectID);
/**获得远程IO——安全门的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_SafeDoor(int ConnectID);
/**获得远程IO——在路径上的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_OnPath(int ConnectID);
/**获得远程IO——原点位置1的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_Position1(int ConnectID);
/**获得远程IO——原点位置2的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_Position2(int ConnectID);
/**获得远程IO——报警中的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_Warning(int ConnectID);
/**获得远程IO——远程模式的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_Online(int ConnectID);
/**获得远程IO——运行中的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_AutoRun(int ConnectID);
/**获得远程IO——伺服上电的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_PowerOn(int ConnectID);
/**获得远程IO——执行模式的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_Play(int ConnectID);
/**获得远程IO——主作业执行中的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_MainJobRun(int ConnectID);
/**获得远程IO——示教模式的状态
@param ConnectID(in) 网络连接ID
@return 1:ON,0:OFF;否则返回错误码
*/
ROBOTSDK_API int Read_Teach(int ConnectID);

/**开启远程IO——主程序调用
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/

ROBOTSDK_API int Send_Open(int ConnectID);
/**开启远程IO——报警复位
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int Send_Reset(int ConnectID);

/**开启远程IO——外部启动
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int Send_Start(int ConnectID);
/**开启远程IO——外部暂停
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int Send_Pause(int ConnectID);
/**开启远程IO——外部急停
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int Send_Stop(int ConnectID);

/**开启远程IO——外部伺服上电
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int Send_PowerOn(int ConnectID);

/**远程IO——关闭安全门
@param ConnectID(in) 网络连接ID
*/
ROBOTSDK_API int Send_SafeDoorOFF(int ConnectID);

/**发送RC指令
@param RCInst(in) RC指令
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int Send_RCInst(INST RCInst, int ConnectID);

/**读字节型用户变量
@param varID(in) 变量编号
@param varBYTE(out) 变量值
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarBYTE(int varID, unsigned char &varBYTE, int ConnectID);

/**读32位整型用户变量
@param varID(in) 变量编号
@param varINT(out) 变量值
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarINT(int varID, int &varINT, int ConnectID);

/**读浮点型用户变量
@param varID(in) 变量编号
@param varFloat(out) 变量值
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarFLOAT(int varID, float &varFloat, int ConnectID);

/**读位置变量
@param varRPY(out) 位置变量结构体
@param exAxis(out) RC外部轴数据组
@param coordinate(out) 坐标系 1~3
@param TOOLNUM(out)  工具坐标系号 1~8
@param USERNUM(out) 用户坐标系号 1~8
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarRPY(int varID, RPY *varRPY, float *exAxis, int &coordinate, int &TOOLNUM, int &USERNUM, int ConnectID);

/**读32位整型系统变量
@param varID(in) 变量编号
@param varINT(out) 变量值
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadSysVarINT(int varID, int &varINT, int ConnectID);

// 码垛应用
/**发送码垛产品相关数据
@param fileNum(in) 文件号
@param stackProData(in) 码垛数据结构体
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendStackProData(short fileNum, PALL_FILE_PRO stackProData, int ConnectID);

/**发送码垛计数
@param fileNum(in) 文件号
@param stackData(int) 码垛计数结构体
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendStackCount(short fileNum, PALL_FILE_COUNTER stackCount, int ConnectID);

/**发送码垛垛点数据(单一产品)
@param fileNum(in) 文件号
@param pointData(in) 码垛垛点数据
@param byteLen(in) 数据长度
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendStackPointData(short fileNum, unsigned char *pointData, int byteLen, int ConnectID);

/**发送码垛垛点数据(多种产品)
@param fileNum(in) 文件号
@param pointData(in) 码垛垛点数据
@param byteLen(in) 数据长度
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendStackPointData2(short fileNum, unsigned char *pointData, int byteLen, int ConnectID);

/*设置监控机器人信息的回调函数.
@param func (in) 函数指针，由用户实现执行状态的处理
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetRobAllDataMonitor(RSMONITOR_CALLBACK func, int ConnectID);

/**发送码垛标志数据
@param stackFlagData(in) 码垛标志数据
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendStackFlagData(PALL_FLAG_MSG stackFlagData, int ConnectID);

/**根据姿态值获得关节值.
@param N (in) 位姿矩阵N分量
@param O (in) 位姿矩阵O分量
@param A (in) 位姿矩阵A分量
@param P (in) 位姿矩阵P分量,位置.
@param jointNum（out）关节个数
@param jointVal（out）关节值
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadJointBaseNOAP(float *N, float *O, float *A, float *P, int jointNum, float *jointVal, int ConnectID);

/**设置轴干涉区.
@param fileNum (in) 轴干涉区文件号
@param data (in) 轴干涉区结构体变量
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetAxisInsertData(int fileNum, AXIS_INTERF_DATA data, int ConnectID);

/**设置立体干涉区.
@param fileNum (in) 立体干涉区文件号
@param data (in) 立体干涉区结构体变量
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetCubeInsertData(int fileNum, CUBE_INTERF_DATA data, int ConnectID);
/**偏移开关.
@param posNum (in) 位置变量号
@param corrdingNum (in) 坐标系号
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int Send_ShiftOnP(int posNum, int corrdingNum, int connectID);
/**偏移关闭.
@param connectID (in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int Send_ShiftOnPOff(int connectID);

/**查询指定码垛文件号的当前码垛计数
@param fileNum (in) 码垛文件号
@param count (out) 当前码垛数量
@param connectID (in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadCurStacCount(short fileNum, int &count, int ConnectID);

/**发送码垛文件号
@param fileNum (in) 码垛文件号
@param connectID (in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SendStackFileNum(short fileNum, int ConnectID);

/**设置工具坐标系
@param coordNum (in) 坐标系文件号
@param coordVal (in) 坐标系值
@param connectID (in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetToolCoordinate(int coordNum, RPY coordVal, int connectID);

/**设置用户坐标系
@param coordNum (in) 坐标系文件号
@param coordVal (in) 坐标系值
@param connectID (in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetUserCoordinate(int coordNum, RPY coordVal, int connectID);


/**读取力控数据
@param Fx（out）x方向的力
@param Fy（out）y方向的力
@param Fz（out）z方向的力
@param Mx（out）x方向的力矩
@param My（out）y方向的力矩
@param Mz（out）z方向的力矩
@param checkout（out）是否拍到照片的校验值
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadForceControlData(double &Fx, double &Fy, double &Fz, double &Mx, double &My, double &Mz, int ConnectID);

/**读取力控数据
@param checkout（out）是否拍到照片的校验值
@param x（out）相机平面坐标值x
@param y（out）相机平面坐标值y
@param Rz（out）偏移角度
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadVisualSenseData(int &checkout, float &x, float &y, float &Rz, int ConnectID);



/**返回当前作业名
@param jobName (in) 当前作业名
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadCurJobName(char *jobName, int ConnectID);

/**读取开机时间
@param startTime（out）开机时间
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadStartTime(TIME_DATA *startTime, int ConnectID);

/**读取运行时间
@param runningTime（out）运行时间
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadRunningTime(double &runningTime, int ConnectID);

/**读取机器人执行速度
@param speed（out）机器人执行速度
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadSpeed(SPEED_DATA *speed, int ConnectID);

/**根据用户坐标系号查询用户坐标系
@param userIndex (in) 用户坐标系号
@param coordVal (out) 用户坐标系NOAP值
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserCoord(unsigned char userIndex, TRANS *coordVal, int ConnectID);

/**发送位置变量数据
@param pDataStream (in) 位置变量数据流
@param byteLen (in) 数据流长度
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendPosData(unsigned char *pDataStream, int byteLen, int ConnectID);

/**查询离线缓存剩余大小
@param remainBuffSize(out) 剩余缓存大小
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadRemainOfflineBuffSize(int &remainBuffSize, int ConnectID);

/**电弧跟踪开启
@param fileNum(in) 文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKON_ARC(short fileNum, int ConnectID);

/**激光绝对跟踪开启
@param fileNum(in) 文件号
@param offY(in) Y方向偏移量
@param offZ(in) Z方向偏移量
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKON_LASER_ABSOLUTE(short fileNum, float offY, float offZ, int ConnectID);

/**激光相对跟踪开启
@param fileNum(in) 文件号
@param offY(in) Y方向偏移量
@param offZ(in) Z方向偏移量
@param posNum(in) 位置变量号
@param mode(in) 偏移的方向，取值范围0-6，0:XYZ,1:X,2:Y,3:Z,4:XY,5:XZ,6:YZ
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKON_LASER_RELATIVE(short fileNum, float offY, float offZ, short posNum, short mode, int ConnectID);

/**（电弧+激光）绝对跟踪开启
@param fileNum(in) 文件号
@param offY(in) Y方向偏移量
@param offZ(in) Z方向偏移量
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKON_ARC_LASER_ABSOLUTE(short fileNum, float offY, float offZ, int ConnectID);

/**（电弧+激光）相对跟踪开启
@param fileNum(in) 文件号
@param offY(in) Y方向偏移量
@param offZ(in) Z方向偏移量
@param posNum(in) 位置变量号
@param mode(in) 偏移的方向，取值范围0-6，0:XYZ,1:X,2:Y,3:Z,4:XY,5:XZ,6:YZ
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKON_ARC_LASER_RELATIVE(short fileNum, float offY, float offZ, short posNum, short mode, int ConnectID);

/**电弧跟踪关闭
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKOFF_ARC(int ConnectID);

/**激光跟踪关闭
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKOFF_LASER(int ConnectID);

/**(电弧+激光)跟踪关闭
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int TRACKOFF_ARC_LASER(int ConnectID);

/**打开实时补偿功能
@param coordType(in) 坐标系类型
@param MType(in) 纠偏方式
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int AMENDON(short coordType, short MType, int ConnectID);

/**关闭实时补偿功能
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int AMENDOFF(int ConnectID);

/**弧焊应用关节运动
@param posNum(in) 全局位置变量编号
@param speed(in) 关节速度（%）
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCJ_P(short posNum, float speed, short acc, short cnt, int ConnectID);

/**弧焊应用圆弧运动
@param mPVarID(in) 中间点全局位置变量编号
@param ePVarID(in) 终点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_P(short mPVarID, short ePVarID, float speed, short acc, short cnt, int ConnectID);

// /**弧焊应用与外部轴联动的圆弧运动
// @param mPVarID(in) 中间点全局位置变量编号
// @param ePVarID(in) 终点全局位置变量编号
// @param speed(in) 圆弧速度(mm/s)
// @param acc(in) 加速度百分比
// @param cnt(in) 逼近等级
// @param ConnectID(in) 网络连接ID
// @return 若成功则返回0，否则返回错误码
// */
// ROBOTSDK_API int SARCC_P(short mPVarID, short ePVarID, float speed, short acc, short cnt, int ConnectID);

/**弧焊应用直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_P(short posNum, float speed, short acc, short cnt, int ConnectID);

// /**弧焊应用与外部轴联动的直线运动
// @param posNum(in) 全局位置变量编号
// @param speed(in) 直线速度(mm/s)
// @param acc(in) 加速度百分比
// @param cnt(in) 逼近等级
// @param ConnectID(in) 网络连接ID
// @return 若成功则返回0，否则返回错误码
// */
// ROBOTSDK_API int SARCL_P(short posNum, float speed, short acc, short cnt, int ConnectID);

/**摆弧开启（正弦型摆动）
@param frequency(in) 频率
@param amplitude(in) 幅值
@param leftTime(in) 左停留时间
@param rightTime(in) 右停留时间
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEON_SIN(float frequency, float amplitude, float leftTime, float rightTime, int ConnectID);

/**摆弧开启（月牙型摆动）
@param frequency(in) 频率
@param amplitude(in) 幅值
@param leftTime(in) 左停留时间
@param rightTime(in) 右停留时间
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEON_MOON(float frequency, float amplitude, float leftTime, float rightTime, int ConnectID);

/**摆弧开启（三角型摆动）
@param frequency(in) 频率
@param amplitude(in) 幅值
@param leftTime(in) 左停留时间
@param rightTime(in) 右停留时间
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEON_ANGLE(float frequency, float amplitude, float leftTime, float rightTime, int ConnectID);

/**摆弧开启（L型摆动）
@param frequency(in) 频率
@param amplitude(in) 幅值
@param leftTime(in) 左停留时间
@param rightTime(in) 右停留时间
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEON_L(float frequency, float amplitude, float leftTime, float rightTime, int ConnectID);

/**摆弧开启（圆型摆动）
@param frequency(in) 频率
@param amplitude(in) 幅值
@param radius(in) 半径
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEON_CIRCLE(float frequency, float amplitude, float radius, int ConnectID);

/**摆弧开启（8字型摆动）
@param frequency(in) 频率
@param amplitude(in) 幅值
@param radius(in) 半径
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEON_8(float frequency, float amplitude, float radius, int ConnectID);

/**摆弧开启（摆动文件）
@param fileNum(in) 摆动文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WAVEON(short fileNum, int ConnectID);

/**查询机器人当前工具坐标系号和当前用户坐标系号
@param curToolNum(out) 当前工具坐标系号
@param curUserNum(out) 当前用户坐标系号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadCurToolNumAndCurUserNum(unsigned char *curToolNum, unsigned char *curUserNum, int ConnectID);

/**设置机器人当前工具坐标系号
@param curToolNum(in) 当前工具坐标系号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SetCurToolNum(unsigned char curToolNum, int ConnectID);

/**设置机器人当前用户坐标系号
@param curUserNum(in) 当前用户坐标系号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SetCurUserNum(unsigned char curUserNum, int ConnectID);

/**发送正反向运动
@param type(in) 方向，1正向，2反向，0停止
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SendFwdMotion(int type, int ConnectID);

/**设置作业联锁模式
@param mode(in) 模式，1正向运动时执行逻辑指令和运动指令，2正向运动时不执行逻辑指令，只执行运动指令
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetInterlock(int mode, int ConnectID);

/**执行模式运行作业
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SendPlayStart(int ConnectID);

/**执行模式暂停作业
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SendPlayPause(int ConnectID);

/**获取作业联锁模式
@param mode(in) 模式，1正向运动时执行逻辑指令和运动指令，2正向运动时不执行逻辑指令，只执行运动指令
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadInterlock(int &mode, int ConnectID);

/**获取后台作业状态
@param status(in) 状态，1运行，2未运行
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadBgJob(int &status, int ConnectID);

/**基础直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_P(short posNum,float speed,short acc,short cnt,int ConnectID);

/**基础直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_RPY(RPY *rpy,float *exAxis,short toolNo,short userNo,float speed,short acc,short cnt,int ConnectID);

/**基础关节运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVJ_P(short posNum,float speed,short acc,short cnt,int ConnectID);

/**基础关节运动
@param joints(in) 机器人关节值
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 关节速度%
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVJ2(float *joints,short toolNo,short userNo,float speed,short acc,short cnt,int ConnectID);

/**基础关节运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 关节速度%
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVJ_RPY(RPY *rpy,float *exAxis,short toolNo,short userNo,float speed,short acc,short cnt,int ConnectID);

/**根据用户坐标系号查询用户坐标系rpy
@param userIndex (in) 用户坐标系号
@param coordVal (out) 用户坐标系RPY值
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserCoord_RPY(unsigned char userIndex, RPY *coordVal, int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点全局位置变量编号
@param posTar (in) 目标点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_P(short posAssNum,short posTarNum,float speed,short acc,short cnt,int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点位姿
@param posTar (in) 目标点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_RPY(ROB_POS_RPY *posAss,ROB_POS_RPY *posTar,short toolNo,short userNo,float speed,short acc,short cnt,int ConnectID);


/**弧焊应用直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_P_COORD(short posNum, float speed, short acc, short cnt, short coord, int ConnectID);

/**弧焊应用圆弧运动
@param mPVarID(in) 中间点全局位置变量编号
@param ePVarID(in) 终点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_P_COORD(short mPVarID, short ePVarID, float speed, short acc, short cnt, short coord, int ConnectID);

/**弧焊应用关节运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 关节速度%
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCJ_RPY(RPY *rpy,float *exAxis,short toolNo,short userNo, float speed, short acc, short cnt, int ConnectID);

/**弧焊应用圆弧运动
@param posM (in) 中间点位姿
@param posE (in) 终点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_RPY(ROB_POS_RPY *posM,ROB_POS_RPY *posE,short toolNo,short userNo, float speed, short acc, short cnt, int ConnectID);


/**弧焊应用直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_RPY(RPY *rpy,float *exAxis,short toolNo,short userNo, float speed, short acc, short cnt, int ConnectID);

/**弧焊应用与变位机联动的直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_RPY_COORD(RPY *rpy,float *exAxis,short toolNo,short userNo, float speed, short acc, short cnt,short coord, int ConnectID);

/**弧焊应用与变位机联动的圆弧运动
@param posM (in) 中间点位姿
@param posE (in) 终点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_RPY_COORD(ROB_POS_RPY *posM,ROB_POS_RPY *posE,short toolNo,short userNo, float speed, short acc, short cnt,short coord, int ConnectID);


/**读128位字符串变量
@param varID(in) 变量编号
@param varStr(out) 变量值
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarStr(int varID, char *varStr, int ConnectID);

/**批量读字符串变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varStr(out) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadUserVarStrMulti(int varID, int count,char varStr[][STR_SIZE], int ConnectID);


/**发送字符串型用户变量
@param varID(in) 变量编号
@param varBYTE(in) 变量值
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarStr(int varID, char *varStr, int ConnectID);


/**批量发送字符串型用户变量
@param varID(in) 变量起始编号
@param count(in) 变量个数
@param varStr(in) 变量值数组，长度为变量个数
@param ConnectID(in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int WriteUserVarStrMulti(int varID, int count, char varStr[][STR_SIZE], int ConnectID);


// /**设置外部工具坐标系
// @param coordNum (in) 坐标系文件号
// @param coordVal (in) 坐标系值
// @param connectID (in) 网络连接ID
// @return 若成功返回0，否则返回错误码
// */
// ROBOTSDK_API int SetEToolCoordinate(int coordNum, RPY *coordVal, int ConnectID);

/**设置世界坐标系
@param coordVal (in) 坐标系值
@param connectID (in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int SetWorldCoordinate(RPY *coordVal, int ConnectID);

// /**根据外部工具坐标系号查询外部工具坐标系rpy
// @param etIndex (in) 外部工具坐标系号
// @param coordVal (out) 用户坐标系RPY值
// @param ConnectID(in) 网络连接ID
// @return 若成功返回0，否则返回错误码
// */
// ROBOTSDK_API int ReadETCoord(unsigned char etIndex, RPY *coordVal, int ConnectID);

/**查询世界坐标系
@param coordVal (out) 坐标系值
@param connectID (in) 网络连接ID
@return 若成功返回0，否则返回错误码
*/
ROBOTSDK_API int ReadWorldCoordinate(RPY* coordVal, int ConnectID);

/**弧焊应用关节运动
@param posNum(in) 全局位置变量编号
@param speed(in) 关节速度（%）
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCJ_P_INTERLOCK(short posNum, float speed, short acc, short cnt, short interlock,int ConnectID);

/**弧焊应用关节运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 关节速度%
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCJ_RPY_INTERLOCK(RPY *rpy,float *exAxis,short toolNo,short userNo, float speed, short acc, short cnt,short interlock,int ConnectID);

/**弧焊应用直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_P_INTERLOCK(short posNum, float speed, short acc, short cnt, short interlock, int ConnectID);

/**弧焊应用直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_RPY_INTERLOCK(RPY *rpy,float *exAxis,short toolNo,short userNo, float speed, short acc, short cnt, short interlock, int ConnectID);



/**弧焊应用直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_P_COORD_INTERLOCK(short posNum, float speed, short acc, short cnt, short coord, short interlock,int ConnectID);

/**弧焊应用与变位机联动的直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCL_RPY_COORD_INTERLOCK(RPY *rpy,float *exAxis,short toolNo,short userNo, float speed, short acc, short cnt, short coord,short interlock, int ConnectID);


/**弧焊应用圆弧运动
@param mPVarID(in) 中间点全局位置变量编号
@param ePVarID(in) 终点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_P_INTERLOCK(short mPVarID, short ePVarID, float speed, short acc, short cnt, short interlock,int ConnectID);


/**弧焊应用圆弧运动
@param posM (in) 中间点位姿
@param posE (in) 终点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_RPY_INTERLOCK(ROB_POS_RPY *posM,ROB_POS_RPY *posE,short toolNo,short userNo, float speed, short acc, short cnt, short interlock,int ConnectID);

/**弧焊应用圆弧运动
@param mPVarID(in) 中间点全局位置变量编号
@param ePVarID(in) 终点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_P_COORD_INTERLOCK(short mPVarID, short ePVarID, float speed, short acc, short cnt, short coord, short interlock,int ConnectID);

/**弧焊应用与变位机联动的圆弧运动
@param posM (in) 中间点位姿
@param posE (in) 终点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param coord(in) 变位机系统号
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ARCC_RPY_COORD_INTERLOCK(ROB_POS_RPY *posM,ROB_POS_RPY *posE,short toolNo,short userNo, float speed, short acc, short cnt,short coord, short interlock,int ConnectID);


/**基础关节运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVJ_P_INTERLOCK(short posNum,float speed,short acc,short cnt, short interlock,int ConnectID);

/**基础关节运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 关节速度%
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVJ_RPY_INTERLOCK(RPY *rpy,float *exAxis,short toolNo,short userNo,float speed,short acc,short cnt,short interlock,int ConnectID);


/**基础直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_P_INTERLOCK(short posNum,float speed,short acc,short cnt,short interlock,int ConnectID);

/**基础直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_RPY_INTERLOCK(RPY *rpy,float *exAxis,short toolNo,short userNo,float speed,short acc,short cnt,short interlock,int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点全局位置变量编号
@param posTar (in) 目标点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_P_INTERLOCK(short posAssNum,short posTarNum,float speed,short acc,short cnt,short interlock,int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点位姿
@param posTar (in) 目标点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_RPY_INTERLOCK(ROB_POS_RPY *posAss,ROB_POS_RPY *posTar,short toolNo,short userNo,float speed,short acc,short cnt,short interlock,int ConnectID);


/**基础直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param eToolNum(in) 外部工具号
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_P_ET_INTERLOCK(short posNum,float speed,short acc,short cnt,short interlock,int ConnectID);

/**基础直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param eToolNum(in) 外部工具号
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_RPY_ET_INTERLOCK(RPY *rpy,float *exAxis,short toolNo,short userNo,float speed,short acc,short cnt,short interlock,int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点全局位置变量编号
@param posTar (in) 目标点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_P_ET_INTERLOCK(short posAssNum,short posTarNum,float speed,short acc,short cnt,short interlock,int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点位姿
@param posTar (in) 目标点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param interlock(in) 干涉区文件号
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_RPY_ET_INTERLOCK(ROB_POS_RPY *posAss,ROB_POS_RPY *posTar,short toolNo,short userNo,float speed,short acc,short cnt,short interlock,int ConnectID);

/**基础直线运动
@param posNum(in) 全局位置变量编号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_P_ET(short posNum,float speed,short acc,short cnt,int ConnectID);

/**基础直线运动
@param rpy(in) 机器人位姿
@param exAxis(in) 外部轴信息
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 直线速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVL_RPY_ET(RPY *rpy,float *exAxis,short toolNo,short userNo,float speed,short acc,short cnt,int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点全局位置变量编号
@param posTar (in) 目标点全局位置变量编号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_P_ET(short posAssNum,short posTarNum,float speed,short acc,short cnt,int ConnectID);

/**基础圆弧运动
@param posAss (in) 辅助点位姿
@param posTar (in) 目标点位姿
@param toolNo(in) 工具坐标系号
@param userNo(in) 用户坐标系号
@param speed(in) 圆弧速度(mm/s)
@param acc(in) 加速度百分比
@param cnt(in) 逼近等级
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int MOVC_RPY_ET(ROB_POS_RPY *posAss,ROB_POS_RPY *posTar,short toolNo,short userNo,float speed,short acc,short cnt,int ConnectID);

/**根据摆动文件号获得摆动文件信息
@param fileNum (in) 摆动文件号1-32
@param waveType (out) 摆动类型
@param waveInfo1(out) 摆动类型为正弦型、月牙型、三角型、L型的摆动文件信息
@param waveInfo2(out) 摆动类型为圆形、8字型的摆动文件信息
@param waveInfo3(out) 摆动类型为自定义类型的摆动文件信息
@param libInfo(out) 摆动类型为自定义类型的摆动库文件信息
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int ReadWeldWaveLibInfo(short fileNum,int *waveType,Wave_File_Param_TYPE1 *waveInfo1,Wave_File_Param_TYPE2 *waveInfo2,Wave_File_Param_TYPE3 *waveInfo3,WAVE_LIB *libInfo,int ConnectID);

/**设置摆动文件信息1(摆动类型为正弦型、月牙型、三角型、L型)
@param fileNum (in) 摆动文件号1-32
@param waveInfo(in) 摆动文件信息
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendWeldWaveLibInfo1(short fileNum,Wave_File_Param_TYPE1 *waveInfo,int ConnectID);

/**设置摆动文件信息2(摆动类型为圆形、8字型)
@param fileNum (in) 摆动文件号1-32
@param waveInfo(in) 摆动文件信息
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendWeldWaveLibInfo2(short fileNum,Wave_File_Param_TYPE2 *waveInfo,int ConnectID);

/**设置摆动文件信息3(摆动类型为自定义类型)
@param fileNum (in) 摆动文件号1-32
@param waveInfo(in) 摆动文件信息
@param libInfo(in) 摆动库信息
@param ConnectID(in) 网络连接ID
@return 若成功则返回0，否则返回错误码
*/
ROBOTSDK_API int SendWeldWaveLibInfo3(short fileNum,Wave_File_Param_TYPE3 *waveInfo,WAVE_LIB *libInfo,int ConnectID);

#endif
