/***********************************************************************************************************************
 *        HEXBOT通信协议格式封装/解析头文件                                                                                      *
 *        Copyright (c)     VK,2017                                                                                    *
 *        Author :          Pei.He                                                                                     *
 *        Version number :  1.00                                                                                       *
 *        Date :            20170222                                                                                   *
 ***********************************************************************************************************************/

#include "hexbotProtocolPack.h"
#include <cstring>
#include <string.h>
#include <ros/ros.h>

byte K=0;
/** 
   * @Title 短整形以大端模式转为字节数组
   * @param data 短整形数据
   * @return 转换后的字节数组地址
   */
void shortToBytesNet(short data,byte bytes[]) {
    bytes[0] = (byte) ((data & 0xff00) >> 8);  
    bytes[1] = (byte) (data & 0xff);
}

/**
   * @Description 从大端字节序的byte[]中指定位置读出2个字节，转换为short数据
   * @param data 字节数组
   * @param startPos 起始位置
   * @return 返回short数据
   */
short byteNet2short(byte data[],int startPos) {  
    short target = (data[startPos+1] & 0xff) | ((data[startPos] << 8) & 0xff00);  
    return target;  
}


/****************************SET_READ_PARAMETERS_MESSAGE消息封装************************************/
/*
 *函数功能：封装READ_SYSTEM_STATUS消息  获取系统状态
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_system_status_pack(HexbotProtocol* dataFrame)    //封装好的数据可以直接发出
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_SYSTEM_STATUS);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_CONTROL_MODE消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_control_mode_pack(byte mode,HexbotProtocol* dataFrame)        //设置步态1或非步态模式0
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_CONTROL_MODE);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = mode;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}


/*
 *函数功能：封装READ_CONTROL_MODE消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_control_mode_pack(HexbotProtocol* dataFrame)   
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_CONTROL_MODE);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_BATTERY_INFO消息
 *参数描述： dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_battery_info_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_BATTERY_INFO);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_FIRMWARE_VERSION消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_firmware_version_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_FIRMWARE_VERSION);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_FIRMWARE_VERSION消息
 *参数描述：led_nums:LED灯数量
 *          Led：待设置的LED灯（编号、颜色）首地址
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_led_rgb_pack(byte led_nums,LedRGB * Led,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_LED_RGB);
  byte tempBuf[4];
  for(int i=0;i<led_nums;i++)
  {
    memset(tempBuf,0,4);
    tempBuf[0] = Led[i].index;
    tempBuf[1] = Led[i].red;
    tempBuf[2] = Led[i].green;
    tempBuf[3] = Led[i].blue;
    dataFrame->addPayload(tempBuf,4);
  }
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_SERVOS_POSITION消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_servos_position_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_SERVOS_POSITION);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_SERVOS_OFFSET消息
 *参数描述：
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_servos_offset_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_SERVOS_OFFSET);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_SERVOS_OFFSET消息
 *参数描述：servoNums:舵机个数
 *          servo：待设置的舵机偏差（编号、偏差值）首地址
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_servos_offset_pack(byte servoNums,ServoOffset * servo,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_SERVOS_OFFSET);
  byte tempBuf[3];
  memset(tempBuf,0,3);
  tempBuf[0] = servoNums;
  dataFrame->addPayload(tempBuf,1);

  for (int i=0;i<servoNums;i++)
  {
    memset(tempBuf,0,3);
    tempBuf[0] = servo[i].index;
    shortToBytesNet(servo[i].offset,&tempBuf[1]);
    dataFrame->addPayload(tempBuf,3);
  }
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装RESET_SERVOS_OFFSET消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_param_reset_servo_offset_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(RESET_SERVOS_OFFSET);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_BEE_RING消息
 *参数描述：interval：响铃时长
 *          freq：响铃频率
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_bee_ring(short interval,short freq,HexbotProtocol* dataFrame){   
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_BEE_RING);
  byte tempBuf[2];
  memset(tempBuf,0,2);
  shortToBytesNet(interval,&tempBuf[0]);
  dataFrame->addPayload(tempBuf,2);
  memset(tempBuf,0,2);
  shortToBytesNet(freq,&tempBuf[0]);
  dataFrame->addPayload(tempBuf,2);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装START_FIRMWARE_UPDATE消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int start_firmware_update_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(START_FIRMWARE_UPDATE);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_GPIO_MODE消息
 *参数描述：gpio_num：扩展口号
 *          mode：待设置的GPIO模式值
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gpio_mode_pack(byte gpio_num , enum GPIOMode mode , HexbotProtocol* dataFrame){  
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_GPIO_MODE);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0]=gpio_num;
  dataFrame->addPayload(tempBuf,1);
  memset(tempBuf,0,1);
  tempBuf[0]=mode;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_GPIO_OUTPUT消息
 *参数描述：gpio_num：扩展口号
 *          gpio_value：管脚输出值
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gpio_output(byte gpio_num , byte gpio_value , HexbotProtocol* dataFrame){
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_GPIO_OUTPUT);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0]=gpio_num;
  dataFrame->addPayload(tempBuf,1);
  memset(tempBuf,0,1);
  tempBuf[0]=gpio_value;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_GPIO_VALUE消息
 *参数描述：gpio_num：扩展口号
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_gpio_value(byte gpio_num , HexbotProtocol* dataFrame){ 
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_GPIO_VALUE);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0]=gpio_num;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_IIC_STATUS消息
 *参数描述：statu：扩展口状态
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_iic_status(byte statu , HexbotProtocol* dataFrame){
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_IIC_STATUS);
  dataFrame->addPayload(&statu,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_IIC_REGISTER_VALUE消息
 *参数描述：iic_addr：IIC设备地址
 *          register_addr:寄存器地址
 *          data_size：数据长度
 *          register_data[]：待写入数据
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_iic_register_value(short iic_addr , short register_addr , byte data_size , byte register_data[] , HexbotProtocol* dataFrame){     
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(SET_IIC_REGISTER_VALUE);
  byte tempBuf[2];
  memset(tempBuf,0,2);
  shortToBytesNet(iic_addr,&tempBuf[0]);
  dataFrame->addPayload(tempBuf,2);
  memset(tempBuf,0,2);
  shortToBytesNet(register_addr,&tempBuf[0]);
  dataFrame->addPayload(tempBuf,2);
  for(int i=0;i<data_size;i++)dataFrame->addPayload(&register_data[i],1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_IIC_REGISTER_VALUE消息
 *参数描述：iic_addr：IIC设备地址
 *          register_addr:寄存器地址
 *          data_size：数据长度
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_iic_register_value(short iic_addr , short register_addr , byte data_size , HexbotProtocol* dataFrame){
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_IIC_REGISTER_VALUE);
  byte tempBuf[2];
  memset(tempBuf,0,2);
  shortToBytesNet(iic_addr,&tempBuf[0]);
  dataFrame->addPayload(tempBuf,2);
  memset(tempBuf,0,2);
  shortToBytesNet(register_addr,&tempBuf[0]);
  dataFrame->addPayload(tempBuf,2);
  dataFrame->addPayload(&data_size,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装READ_SENSOR_DATA消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int read_sensor(HexbotProtocol* dataFrame){
  dataFrame->setMessageType(SET_READ_PARAMETERS_MESSAGE);
  dataFrame->setMessageId(READ_SENSOR_DATA);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}


/********************************NOT_GAIT_CONTROL_MESSAGE消息封装*********************************/
/*
*函数功能：封装ACTION_DATA消息
*参数描述：servoNums:舵机个数
*          actionTimes：动作持续时间
*          servo：待设置的舵机动作（编号、动作值）首地址
*          dataFrame：【输出参数】返回封装好的协议帧数据
*返回值：  消息的总字节数
*/
unsigned int action_data_pack(byte servoNums , short actionTimes, ServoPos *servo , HexbotProtocol* dataFrame){ 
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);        //获取信息类型
  dataFrame->setMessageId(ACTION_DATA);                  //获取信息ID
  byte tempBuf[3];
  if(K==0){
    memset(tempBuf,0,3);
    tempBuf[0] = servoNums;     //装载舵机个数
    shortToBytesNet(actionTimes,&tempBuf[1]);    // 因时间是短整型，将其拆分字节，从tempbuf[1]开始保存
    dataFrame->addPayload(tempBuf,3);        //给payload装入舵机个数，动作时间的信息  同时更新长度
  }
    memset(tempBuf,0,3);                 //三个为一组
    tempBuf[0] = servo[K].index;             //装入舵机编号
    shortToBytesNet(servo[K].pos,&tempBuf[1]);       //装入动作数据，一个servo.pulse分成两个tempbuf保存
    dataFrame->addPayload(tempBuf,3);     //给payload装入数据  同时更新长度
  K++;
  return dataFrame->getMessageTotalLen();      //返回数据总长度
}

/*
*函数功能：封装RUN_ACTIONGROUP消息
*参数描述：actionGroupId：动作组编号
*          actionNums：动作执行次数
*          dataFrame：【输出参数】返回封装好的协议帧数据
*返回值：  消息的总字节数
*/
unsigned int run_actiongroup_pack(byte actionGroupId ,short actionNums,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(RUN_ACTIONGROUP);
  byte tempBuf[3];
  memset(tempBuf,0,3);
  tempBuf[0] = actionGroupId;
  shortToBytesNet(actionNums,&tempBuf[1]);
  dataFrame->addPayload(tempBuf,3);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装STOP_ACTIONGROUP消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int stop_actiongroup_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(STOP_ACTIONGROUP);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装DOWNLOAD_ACTIONGROUP消息
 *参数描述：servoNums:舵机个数
 *          actionTimes：动作持续时间
 *          action_count：动作组包含的动作数量
 *          actiongroup_id：动作组编号
 *          actionIndex：当前动作在动作组中的序号
 *          action：待设置的下载动作（编号、动作值）首地址
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int download_actiongroup_pack(byte servoNums,short actionTime,short action_count,
             byte actiongroup_id, DownLoadAction *action,HexbotProtocol* dataFrame){ 
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(DOWNLOAD_ACTIONGROUP); 
  byte tempBuf[3];
  if(K==0){
    memset(tempBuf,0,3);
    tempBuf[0]=actiongroup_id;
    shortToBytesNet(action_count,&tempBuf[1]);
    dataFrame->addPayload(tempBuf,3);
    memset(tempBuf,0,3);
    shortToBytesNet(action[0].action_id,&tempBuf[0]);
    dataFrame->addPayload(tempBuf,2);
    memset(tempBuf,0,3);
    tempBuf[0]=servoNums;
    shortToBytesNet(actionTime,&tempBuf[1]);
    dataFrame->addPayload(tempBuf,3);
  }
  memset(tempBuf,0,3);
  tempBuf[0] = action[K].index;
  shortToBytesNet(action[K].pos,&tempBuf[1]);
  dataFrame->addPayload(tempBuf,3);
  K++;
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装ERASE_ACTIONGROUP消息
 *参数描述：actionGroupId:动作组编号
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int erase_actiongroup_pack(byte actionGroupId,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(ERASE_ACTIONGROUP);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = actionGroupId;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装RESET_SERVOS_POSITION消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int reset_servos_position_pack(HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(RESET_SERVOS_POSITION);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_NOT_GAIT_SPEED消息
 *参数描述：percentageSpeed:速度百分比
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_not_gait_speed_pack(short percentageSpeed,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_NOT_GAIT_SPEED);
  byte tempBuf[2];
  memset(tempBuf,0,2);
  shortToBytesNet(percentageSpeed,tempBuf);
  dataFrame->addPayload(tempBuf,2);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_TORQUE_SWITCH消息
 *参数描述：flag:扭矩开关状态，1：开启；0：关闭
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_torque_switch_pack(byte flag,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(NOT_GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_TORQUE_SWITCH);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = flag;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}


/*************************************GAIT_CONTROL_MESSAGE消息封装*********************************/
/*
 *函数功能：封装SET_GAIT_TYPE消息
 *参数描述：gaitMode:步态模式，1：行走步态；2：姿态调整模式1；3：姿态调整模式2；4：单腿模式
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_mode_pack(byte gaitMode,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_GAIT_TYPE);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = gaitMode;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_WALK_GAIT_TYPE消息
 *参数描述：waklMode:行走模式，1：涟漪；2：三角8步；3：三角12步；4：三角16步；5：波浪；6：三角6步
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_walk_mode_pack(byte waklMode,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_WALK_GAIT_TYPE);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = waklMode;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_MOVE_DIRECTION消息
 *参数描述：lx:左摇杆的水平方向的值
 *         ly:左摇杆的垂直方向的值
 *         rx:右摇杆的水平方向的值
 *         ry:右摇杆的垂直方向的值
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_move_dirction_pack(byte lx, byte ly, byte rx, byte ry, HexbotProtocol* dataFrame)
{
   dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId( SET_MOVE_DIRECTION);
  byte tempBuf[4];
  memset(tempBuf,0,4);
  tempBuf[0] = lx;
  tempBuf[1] = ly;
  tempBuf[2] = rx;
  tempBuf[3] = ry;
  dataFrame->addPayload(tempBuf,4);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_GAIT_SPEED消息
 *参数描述：flag:1：加速；0：减速
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_speed_pack(byte flag, HexbotProtocol* dataFrame){
   dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_GAIT_SPEED);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = flag;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
 }
 
 /*
 *函数功能：封装SET_BODY_HIGHT消息
 *参数描述：flag:1：抬高；0：降低
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_body_hight_pack(byte flag,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_BODY_HIGHT);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = flag;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

 /*
 *函数功能：封装SET_LEG_LIFT_HIGHT消息
 *参数描述：flag:1：抬高；0：降低
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_walk_leg_lift_hight_pack(byte flag,HexbotProtocol* dataFrame)
{
 dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_LEG_LIFT_HIGHT);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = flag;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_WALK_STEP消息
 *参数描述：flag:1：加长；0：减短
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_walk_step_pack(byte flag,HexbotProtocol* dataFrame)
{
  dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_WALK_STEP);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = flag;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_WALK_STEP消息
 *参数描述：dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_Single_leg_Selected_Leg(HexbotProtocol* dataFrame)
{
   dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_CURRENT_LEG);
  dataFrame->setPayloadLen(0);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_BALANCE_MODE消息
 *参数描述：flag:1：开启；0：关闭
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_balance_mode_pack(byte flag, HexbotProtocol* dataFrame)
{
   dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
  dataFrame->setMessageId(SET_BALANCE_MODE);
  byte tempBuf[1];
  memset(tempBuf,0,1);
  tempBuf[0] = flag;
  dataFrame->addPayload(tempBuf,1);
  dataFrame->calFrameCrc();
  return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_NAVIGATE_MODE消息
 *参数描述：flag:1：开启；0：关闭
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_navigate_mode_pack(byte flag,HexbotProtocol* dataFrame){
	dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
	dataFrame->setMessageId(SET_NAVIGATE_MODE);
	byte tempBuf[1];
	memset(tempBuf,0,1);
	tempBuf[0] = flag;
	dataFrame->addPayload(tempBuf,1);
	dataFrame->calFrameCrc();
	return dataFrame->getMessageTotalLen();
}

/*
 *函数功能：封装SET_NAVIGATE_VEL消息
 *参数描述：flag:1：开启；0：关闭
 *          dataFrame：【输出参数】返回封装好的协议帧数据
 *返回值：  消息的总字节数
 */
unsigned int set_gait_navigate_vel_pack(float vx, float vy, float vz,HexbotProtocol* dataFrame){
	dataFrame->setMessageType(GAIT_CONTROL_MESSAGE);
	dataFrame->setMessageId(SET_NAVIGATE_VEL);
	byte tempBuf[12];
	memset(tempBuf,0,12);
	memcpy(tempBuf, &vx,4);
	memcpy(tempBuf+4, &vy,4);
	memcpy(tempBuf+8, &vz,4);
	dataFrame->addPayload(tempBuf,12);
	dataFrame->calFrameCrc();
	return dataFrame->getMessageTotalLen();
}

/***********************************RETURN_MESSAGE消息解析******************************************/
/*
 *函数功能：解析RECV_RETURN_SYSTEM_STATUS消息
 *参数描述：dataFrame：接收到的协议帧数据 
 *          elecPercentag：【输出参数】电量百分比
 *          controlMode：【输出参数】控制模式
 *          gaitMode：【输出参数】步态模式
 *返回值：  null
 */
void recv_return_system_status_decode(HexbotProtocol* dataFrame,byte& elecPercentag,byte& controlMode,byte& gaitMode)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  elecPercentag = tempBuf[0];
  controlMode = tempBuf[1];
  gaitMode = tempBuf[2];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_CONTROL_MODE消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          controlMode：【输出参数】控制模式
 *返回值：  null
 */
void recv_return_control_mode_decode(HexbotProtocol* dataFrame,byte& controlMode)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  controlMode = tempBuf[0];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_BATTERY_INFO消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          elecPercentag：【输出参数】电量
 *返回值：  null
 */
void recv_return_battery_info_decode(HexbotProtocol* dataFrame,byte& elecPercentag)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  elecPercentag = tempBuf[0];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_FIRMWARE_VERSION消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          versionF：【输出参数】版本号第一位数字
 *          versionS: 【输出参数】版本号第二位数字
 *          versionT: 【输出参数】版本号第三位数字
 *返回值：  null
 */
void recv_return_firmware_version_decode(HexbotProtocol* dataFrame,byte& versionF,byte& versionS,byte& versionT)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  versionF = tempBuf[0];
  versionS = tempBuf[1];
  versionT = tempBuf[2];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_SERVOS_POSITION消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          servoNums：【输出参数】舵机个数
 *          servoIndex：【输出参数】舵机编号数组
 *          servoPos:【输出参数】舵机位置数组
 *返回值：  null
 */
void recv_return_servos_position_decode(HexbotProtocol* dataFrame,byte& servoNums,byte servoIndex[],short servoPos[])
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  servoNums = tempBuf[0]>SUPPORT_SERVO_COUNT?SUPPORT_SERVO_COUNT:tempBuf[0];
  int nums = servoNums;
  int index = 1;
  for (int i=0;i<nums;i++)
  {
    servoIndex[i] = tempBuf[i*3+index];
    servoPos[i] = byteNet2short(tempBuf,i*3+index+1);
  }
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_SERVOS_OFFSET消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          servoNums：【输出参数】舵机个数
 *          servoIndex：【输出参数】舵机编号数组
 *          servoOffset:【输出参数】舵机偏差数组
 *返回值：  null
 */
void recv_return_servos_offset_decode(HexbotProtocol* dataFrame,byte& servoNums,byte servoIndex[],short servoOffset[])
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  servoNums = tempBuf[0]>SUPPORT_SERVO_COUNT?SUPPORT_SERVO_COUNT:tempBuf[0];
  int nums = servoNums;
  int index = 1;
  for (int i=0;i<nums;i++)
  {
    servoIndex[i] = tempBuf[i*3+index];
    servoOffset[i] = byteNet2short(tempBuf,i*3+index+1);
  }
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_FINISH_DOWNLOAD_SERVO_OFFSET消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          servoNums：【输出参数】舵机个数
 *          result:【输出参数】执行结果，0：成功；1：失败
 *返回值：  null
 */
void recv_return_finish_download_servo_offset_decode(HexbotProtocol* dataFrame,byte& servoNums,byte& result)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  servoNums = tempBuf[0];
  result = tempBuf[1];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_RUN_ACTION_GROUP消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          actionGroupId：【输出参数】动作组编号
 *          actionNums：【输出参数】动作组执行次数
 *返回值：  null
 */
void recv_return_run_action_group_decode(HexbotProtocol* dataFrame,byte& actionGroupId,short& actionNums,byte& result)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  actionGroupId = tempBuf[0];
  actionNums = byteNet2short(tempBuf,1);
  result = tempBuf[3];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_STOP_ACTION_GROUP消息
 *参数描述：dataFrame：接收到的协议帧数据
 *返回值：  null
 */
 void recv_return_stop_action_group_decode(HexbotProtocol* dataFrame)
{
  
}

/*
 *函数功能：解析RECV_RETURN_COMPLETE_ACTION_GROUP消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          actionGroupId：【输出参数】动作组编号
 *          actionNums：【输出参数】动作组执行次数
 *返回值：  null
 */
void recv_return_complete_action_group_decode(HexbotProtocol* dataFrame,byte& actionGroupId,short& actionNums)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  actionGroupId = tempBuf[0];
  actionNums = byteNet2short(tempBuf,1);
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_FINSH_DOWNLOAD_ACTION_GROUP消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          actionGroupId：【输出参数】动作组编号
 *          actionCount：【输出参数】动作组执行次数
 *          result:【输出参数】执行结果，0：成功；1：失败
 *返回值：  null
 */
void recv_return_finish_download_action_group_decode(HexbotProtocol* dataFrame,byte& actionGroupId,short& actionCount,byte& result)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  actionGroupId = tempBuf[0];
  actionCount = byteNet2short(tempBuf,1);
  result = tempBuf[3];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_ERASE_COMPLETE消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          actionGroupId：【输出参数】动作组编号
 *          result:【输出参数】执行结果，0：成功；1：失败
 *返回值：  null
 */
void recv_return_erase_complete_decode(HexbotProtocol* dataFrame,byte& actionGroupId,byte& result)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  actionGroupId = tempBuf[0];
  result = tempBuf[1];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_SET_NOT_GAIT_SPEED消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          speedFactor：【输出参数】速度百分比
 *          result:【输出参数】执行结果，0：成功；1：失败
 *返回值：  null
 */
void recv_return_set_not_gait_speed_decode(HexbotProtocol* dataFrame,short& speedFactor,byte& result)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  speedFactor = byteNet2short(tempBuf,0);
  result = tempBuf[2];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_START_FIMRWARE_UPDATE消息
 *参数描述：dataFrame：接收到的协议帧数据
 *返回值：  null
 */
static inline void recv_return_start_firmware_update_decode(HexbotProtocol* dataFrame)
{
  
}

/*
 *函数功能：解析RECV_RETURN_READ_GPIO_VALUE消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          gpio_num：GPIO管脚号
 *          gpio_value：GPIO的值
 *返回值：  null
 */
void recv_return_read_gpio_value_decode(HexbotProtocol* dataFrame,byte& gpio_num,byte& gpio_value)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  gpio_num = tempBuf[0];
  gpio_value = tempBuf[1];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_SET_IIC_REGISTER_VALUE消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          iic_addr：IIC设备寄存器的地址
 *          result：设置结果  0：成功  1：IIC设备不存在  2：IIC设备存在，写入数据失败
 *返回值：  null
 */
void recv_return_set_iic_register_value_decode(HexbotProtocol* dataFrame,short& iic_addr,byte& result)
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  iic_addr = byteNet2short(tempBuf,0);
  result = tempBuf[2];
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_SET_READ_REGISTER_VALUE消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          iic_addr：IIC设备寄存器的地址
 *          result：设置结果  0：成功  1：IIC设备不存在  2：IIC设备存在，读取数据失败
 *          data[]：IIC设备寄存器数据
 *返回值：  null
 */
void recv_return_read_iic_register_value_decode(HexbotProtocol* dataFrame,short& iic_addr,byte& result,byte data[])
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  iic_addr = byteNet2short(tempBuf,0);
  result = tempBuf[2];
  for (int i=0;i<(payloadLen-3);i++)
  {
    data[i]=tempBuf[3+i];
  }
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_READ_SENSOR_DATA消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          data[]：传感器数据
 *返回值：  null
 */
void recv_return_read_sensor_data_decode(HexbotProtocol* dataFrame,byte data[])
{
  int payloadLen = dataFrame->getPayloadLen();
  byte *tempBuf = new byte[payloadLen];
  memset(tempBuf,0,payloadLen);
  dataFrame->getPayload(tempBuf,payloadLen);
  for (int i=0;i<payloadLen;i++)
  {
    data[i]=tempBuf[i];
  }
  delete tempBuf;
}

/*
 *函数功能：解析RECV_RETURN_NAVIGATE_POS_AND_VEL消息
 *参数描述：dataFrame：接收到的协议帧数据
 *          data[]：传感器数据
 *返回值：  null
 */
void recv_return_navigate_pos_and_vel_decode(HexbotProtocol* dataFrame, float& xPos, float& yPos, float& zPos,float& vx, float& vy, float& vz){
	int payloadLen = dataFrame->getPayloadLen();
	byte *tempBuf = new byte[payloadLen];
	memset(tempBuf,0,payloadLen);
	dataFrame->getPayload(tempBuf,payloadLen);
	memcpy(&xPos,tempBuf,4);
	memcpy(&yPos,tempBuf+4,4);
	memcpy(&zPos,tempBuf+8,4);
	memcpy(&vx,tempBuf+12,4);
	memcpy(&vy,tempBuf+16,4);
	memcpy(&vz,tempBuf+20,4);
}

