/***********************************************************************************************************************
 *        HEXBOT通信协议头文件                                                                         *
 *        Copyright (c)     VK,2017                                                                                    *
 *        Author :          Pei.He                                                                                     *
 *        Version number :  1.00                                                                                       *
 *        Date :            20170222                                                                                   *
 ***********************************************************************************************************************/

#ifndef _HEXBOT_PROTOCOL_
#define _HEXBOT_PROTOCOL_


/*协议的数据帧格式如下：*/
/*********************************************************************/
/*  |start_flag|mesage_type|mesage_id|payload_len|payload| crc |     */
/*  |    2     |     1     |      1  |      1    |   N   |  1  |     */
/* *******************************************************************/

typedef unsigned char byte;

#define VK_PROTOCOL_STX1  0x56
#define VK_PROTOCOL_STX2  0x4B
#define MIN_MESSGE_LEN    0x05      //start_flag+mesage_type+mesage_id+payload_len
#define CRC_LEN           0x01

#define MAX_PAYLOAD_LEN   255       //payload最长为256个字节

enum MESSAGE_TYPE{
  //4种消息类型
  SET_READ_PARAMETERS_MESSAGE  = 0x01,
  NOT_GAIT_CONTROL_MESSAGE =    0x02,
  GAIT_CONTROL_MESSAGE  =  0x03,
  RETURN_MESSAGE  =   0x04,           
};

enum SET_READ_PARAMETERS_ID{
  READ_SYSTEM_STATUS = 0x01,              //读取系统状态
  SET_CONTROL_MODE = 0x02,                //设置控制模式
  READ_CONTROL_MODE = 0x03,               //读取控制模式
  READ_BATTERY_INFO = 0x04,               //读取电池电量
  READ_FIRMWARE_VERSION = 0x05,           //读取机器人固件版本号
  SET_LED_RGB = 0x06,                     //设置LED颜色
  READ_SERVOS_POSITION = 0x07,            //读取舵机位置
  READ_SERVOS_OFFSET = 0x08,              //读取舵机偏差
  SET_SERVOS_OFFSET = 0x09,               //设置舵机偏差
  RESET_SERVOS_OFFSET = 0x0A,             //复位舵机偏差
  SET_BEE_RING = 0x0B,                    //设置蜂鸣器响铃
  START_FIRMWARE_UPDATE = 0x0C,           //启动固件升级
  SET_GPIO_MODE = 0x0D,                   //设置GPIO模式
  SET_GPIO_OUTPUT = 0x0E,                 //设置GPIO输出
  READ_GPIO_VALUE = 0x0F,                 //读取GPIO的值
  SET_IIC_STATUS = 0x10,                  //设置IIC扩展口状态
  SET_IIC_REGISTER_VALUE = 0x11,          //设置IIC设备寄存器值
  READ_IIC_REGISTER_VALUE = 0x12,         //读取IIC设备寄存器值
  READ_SENSOR_DATA = 0x13                 //读取传感器数据
};

enum NOT_GAIT_CONTROL_ID{
  ACTION_DATA = 0x01,                     //Action数据
  RUN_ACTIONGROUP = 0x02,                 //运行动作组
  STOP_ACTIONGROUP = 0x03,                //停止运行动作组
  DOWNLOAD_ACTIONGROUP = 0x04,            //下载动作组
  ERASE_ACTIONGROUP = 0x05 ,              //擦除动作组
  RESET_SERVOS_POSITION = 0x06,           //复位舵机
  SET_NOT_GAIT_SPEED = 0x07,              //设置非步态模式下的速度
  SET_TORQUE_SWITCH = 0x08                //设置扭矩开关
};

enum GAIT_CONTROL_ID{
  SET_GAIT_TYPE  = 0x01 ,                 //设置步态模式  
  SET_WALK_GAIT_TYPE =  0x02 ,            //设置行走步态模式
  SET_MOVE_DIRECTION =  0x03 ,            //设置运动方向
  SET_GAIT_SPEED = 0x04 ,                 //设置步态模式下的速度
  SET_BODY_HIGHT = 0x05 ,                 //设置机身高度
  SET_LEG_LIFT_HIGHT = 0x06 ,             //设置抬腿高度
  SET_WALK_STEP = 0x07 ,                  //设置行走步长
  SET_CURRENT_LEG = 0x08 ,                //单腿模式下切换控制腿
  SET_BALANCE_MODE = 0x09 ,               //开启/关闭平衡模式
  SET_NAVIGATE_MODE = 0X0A,                //开启/关闭导航模式
  SET_NAVIGATE_VEL = 0X0B                 //设置导航模式下的运行速度
};

enum RECV_RETURN_ID{
  RECV_RETURN_SYSTEM_STATUS = 0x01,                //返回系统状态
  RECV_RETURN_CONTROL_MODE = 0x02,                 //返回控制模式
  RECV_RETURN_BATTERY_INFO = 0x03,                 //返回电池电量
  RECV_RETURN_FIRMWARE_VERSION = 0x04,             //返回机器人固件版本号
  RECV_RETURN_SERVOS_POSITION = 0x05,              //返回舵机位置
  RECV_RETURN_SERVOS_OFFSET = 0x06,                //返回舵机偏差
  RECV_RETURN_FINISH_DOWNLOAD_SERVO_OFFSET = 0x07, //舵机偏差下载是否成功的返回消息
  RECV_RETURN_RUN_ACTION_GROUP = 0x08,             //运行动作组命令的返回信息
  RECV_RETURN_STOP_ACTION_GROUP = 0x09,            //停止动作组命令的返回信息
  RECV_RETURN_COMPLETE_ACTION_GROUP = 0x0A,        //动作组执行完成的返回信息
  RECV_RETURN_FINSH_DOWNLOAD_ACTION_GROUP = 0x0B,   //动作组下载是否成功的返回消息
  RECV_RETURN_ERASE_COMPLETE = 0x0C,                //擦除完成返回消息
  RECV_RETURN_SET_NOT_GAIT_SPEED = 0x0D,            //设置速度成功返回消息
  RECV_RETURN_START_FIMRWARE_UPDATE = 0x0E,         //开启固件升级返回消息
  RECV_RETURN_READ_GPIO_VALUE = 0x0F,               //读取GPIO值返回消息
  RECV_RETURN_SET_IIC_REGISTER_VALUE = 0x10,        //设置IIC设备寄存器返回消息
  RECV_RETURN_SET_READ_REGISTER_VALUE = 0x11,       //读取IIC设备寄存器值返回消息
  RECV_RETURN_READ_SENSOR_DATA = 0x12,               //读取传感器数据返回消息
  RECV_RETURN_NAVIGATE_POS_AND_VEL = 0X013          //导航模式下机器人的位置和速度
};

typedef struct ledRGB{      //LED灯
    byte index;        //LED灯编号
    byte red;         //红灯亮度值
    byte green;         //绿灯亮度值
    byte blue;         //蓝灯亮度值
}LedRGB;

typedef struct servoPosition{        //舵机动作
    byte index;          //舵机编号
    unsigned short pos;          //舵机动作值
}ServoPos;

typedef struct servoOffset{    //舵机偏差
    byte index;
    unsigned short offset;
}ServoOffset;

typedef struct download_action{    //下载动作组
    byte index;
    unsigned short pos;
    unsigned short action_id;  //动作编号
}DownLoadAction;

 enum GPIOMode{ 
    GPIOMode_AIN = 1,     //模拟输入
    GPIOMode_IN_FLOATING = 2,     //浮空输入
    GPIOMode_IPD = 3,      //下拉输入
   GPIOMode_IPU = 4,      //上拉输入
    GPIOMode_Out_OD = 5,     //开漏输出
    GPIOMode_Out_PP = 6,     //推挽输出
   GPIOMode_AF_OD = 7 ,     //复用开漏输出�
   GPIOMode_AF_PP = 8     //复用推挽输出
};

enum GaitMode{               //步态模式类型
    WALK_MODE = 1,         //行走步态模式
    ROTATE_MODE = 2,      //姿态调整步态模式1
    TRANSLATE_MODE = 3,     //姿态调整步态模式2
    SINGLE_LEG_MODE = 4      //单腿步态模式
};

enum WalkType{          //行走步态类型
    RIPPLE = 1,       //涟漪形
    TRIANGLE8 = 2,     //三角（8步）
    TRIANGLE12 = 3,     //三角（12步）
    TRIANGLE16 = 4,     //三角（16步）
    WAVE = 5,           //波浪步
    TRIANGLE = 6      //三角6步
};

enum SpeedSet{        //速度设置
    FASTER=1,     //加快
    SLOWER=0       //减慢
};

enum HightSet{        //高度设置
    HIGHER=1,      //提高
    LOWER=0        //降低
};

enum StepLengthSet{     //默认步长为-128~127mm
    LONG=1,         //减短-64~63.5mm
    SHORT=0           //加长为-128~127mm
};

enum BalanceMode{      //平衡模式
    OPEN=1,        //打开
    CLOSE=0        //关闭
};

class HexbotProtocol{
public:
  HexbotProtocol();
  void setStartFlag1(byte);          //设置信息第一个标志为.....
  byte getStarFlag1();               //获取信息第一个标志
  void setStartFlag2(byte);
  byte getStarFlag2();
  void setMessageType(MESSAGE_TYPE);  //设置信息类型为.....
  byte getMessageType();
  void setMessageId(byte);            //设置信息ID为.....
  byte getMessageId();
  void setPayloadLen(byte);          //设置数据长度为....
  byte getPayloadLen();
  byte getPayloadParam(int index);
  void setPayloadParam(int index,int value);
  byte getMessageTotalLen();
  /*从byte[]数组中，解析成数据帧格式*/
  void initDataFrame(byte data[],int len ,int startPos = 0);
  /*将数据帧格式转换为byte[],为发送数据做准备*/
  void dataFrameToByteArray(byte data[],int& len);
  void addPayload(byte data[],int len,int startPos = 0);
  void fillPayload(byte data[],int len,int pos);
  void getPayload(byte data[],int& len);
  void resetDataFrame();
  void copyFromInstance(HexbotProtocol message);
  byte calCrc(byte* buf,byte len);       
  void calFrameCrc();
  byte getFrameCrc();
private:
  byte startFlag1;
  byte startFlag2;
  byte messageType;
  byte messageId;
  byte payloadLen;
  byte payload[MAX_PAYLOAD_LEN];
  byte crc;
};


#endif


