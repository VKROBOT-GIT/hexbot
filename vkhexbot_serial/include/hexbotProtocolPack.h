/***********************************************************************************************************************
 *        HEXBOT通信协议格式封装/解析头文件                                                                                      *
 *        Copyright (c)     VK,2017                                                                                    *
 *        Author :          Pei.He                                                                                     *
 *        Version number :  1.00                                                                                       *
 *        Date :            20170222                                                                                   *
 ***********************************************************************************************************************/
#ifndef _HEXBOT_PROTOCOL_PACK_H
#define _HEXBOT_PROTOCOL_PACK_H

#include "hexbotProtocol.h"

#define SUPPORT_SERVO_COUNT 32

//数据格式转换
void shortToBytesNet(short data,byte bytes[]);
short byteNet2short(byte data[],int startPos=0);

//SET_READ_PARAMETERS_MESSAGE消息封装
unsigned int read_system_status_pack(HexbotProtocol* dataFrame);
unsigned int set_control_mode_pack(byte mode,HexbotProtocol* dataFrame);
unsigned int read_control_mode_pack(HexbotProtocol* dataFrame);
unsigned int read_battery_info_pack(HexbotProtocol* dataFrame);
unsigned int read_firmware_version_pack(HexbotProtocol* dataFrame);
unsigned int set_led_rgb_pack(byte led_nums,LedRGB * Led,HexbotProtocol* dataFrame);
unsigned int read_servos_position_pack(HexbotProtocol* dataFrame);
unsigned int read_servos_offset_pack(HexbotProtocol* dataFrame);
unsigned int set_servos_offset_pack(byte servoNums,ServoOffset * servo,HexbotProtocol* dataFrame);
unsigned int set_param_reset_servo_offset_pack(HexbotProtocol* dataFrame);
unsigned int set_bee_ring(short interval,short freq);
unsigned int start_firmware_update_pack(HexbotProtocol* dataFrame);
unsigned int set_gpio_mode_pack(byte gpio_num , enum GPIOMode Mode);
unsigned int set_gpio_output(byte gpio_num , byte gpio_value);
unsigned int read_gpio_value(byte gpio_num);
unsigned int set_iic_status(byte statu);
unsigned int set_iic_register_value(short iic_addr , short register_addr , byte register_data[]);
unsigned int read_iic_register_value(short iic_addr , short register_addr , byte data_size);
unsigned int read_sensor(void);

//NOT_GAIT_CONTROL_MESSAGE消息封装
unsigned int action_data_pack(byte servoNums ,short actionTimes,byte servoIndex[],short servoPulse[],HexbotProtocol* dataFrame);
/*
 * 功能说明;运行动作组
 * actionGroupId:动作组Id
 * actionNums:动作组执行的次数
 * dataFrame: 封装好的消息包，返回值
 * 备注说明：HexBot开始运行动作组之后，将只处理速度设置，停止运行动作组，获取机器人运行状态三种控制命令，
 * 其它控制命令会被丢弃，直到动作组运行完成或被动结束。该特点请用户悉知
 */
unsigned int run_actiongroup_pack(byte actionGroupId ,short actionNums,HexbotProtocol* dataFrame);
unsigned int stop_actiongroup_pack(HexbotProtocol* dataFrame);
unsigned int download_actiongroup_pack(byte actionGroupId,short actionCount,short actionIndex,byte servoNums ,short actionTimes,byte servoIndex[],short servoPulse[],HexbotProtocol* dataFrame);
unsigned int erase_actiongroup_pack(byte actionGroupId,HexbotProtocol* dataFrame);
unsigned int reset_servos_position_pack(HexbotProtocol* dataFrame);
unsigned int set_not_gait_speed_pack(short percentageSpeed,HexbotProtocol* dataFrame);
unsigned int set_torque_switch_pack(byte flag,HexbotProtocol* dataFrame);

//GAIT_CONTROL_MESSAGE消息封装
unsigned int set_gait_mode_pack(byte gaitMode, HexbotProtocol* dataFrame);
unsigned int set_gait_walk_mode_pack(byte waklMode, HexbotProtocol* dataFrame);
unsigned int set_gait_move_dirction_pack(byte lx, byte ly, byte rx, byte ry, HexbotProtocol* dataFrame);
unsigned int set_gait_speed_pack(byte flag, HexbotProtocol* dataFrame);
unsigned int set_gait_body_hight_pack(byte flag, HexbotProtocol* dataFrame);
unsigned int set_gait_walk_leg_lift_hight_pack(byte flag, HexbotProtocol* dataFrame);
unsigned int set_gait_walk_step_pack(byte flag, HexbotProtocol* dataFrame);
unsigned int set_gait_Single_leg_Selected_Leg(HexbotProtocol* dataFrame);
unsigned int set_gait_balance_mode_pack(byte flg, HexbotProtocol* dataFrame);
unsigned int set_gait_navigate_mode_pack(byte flag,HexbotProtocol* dataFrame);
unsigned int set_gait_navigate_vel_pack(float vx, float vy, float vz,HexbotProtocol* dataFrame);


//RETURN_MESSAGE消息解析      
void recv_return_system_status_decode(HexbotProtocol* dataFrame,byte& elecPercentag,byte& controlMode,byte& gaitMode);
void recv_return_control_mode_decode(HexbotProtocol* dataFrame,byte& controlMode);
void recv_return_battery_info_decode(HexbotProtocol* dataFrame,byte& elecPercentag);
void recv_return_firmware_version_decode(HexbotProtocol* dataFrame,byte& versionF,byte& versionS,byte& versionT);
void recv_return_servos_position_decode(HexbotProtocol* dataFrame,byte& servoNums,byte servoIndex[],short servoPos[]);
void recv_return_servos_offset_decode(HexbotProtocol* dataFrame,byte& servoNums,byte servoIndex[],short servoOffset[]);
void recv_return_finish_download_servo_offset_decode(HexbotProtocol* dataFrame,byte& servoNums,byte& result);
void recv_return_run_action_group_decode(HexbotProtocol* dataFrame,byte& actionGroupId,short& actionNums,byte& result);
void recv_return_stop_action_group_decode(HexbotProtocol* dataFrame);
void recv_return_complete_action_group_decode(HexbotProtocol* dataFrame,byte& actionGroupId,short& actionNums);
void recv_return_finish_download_action_group_decode(HexbotProtocol* dataFrame,byte& actionGroupId,short& actionCount,byte& result);
void recv_return_erase_complete_decode(HexbotProtocol* dataFrame,byte& actionGroupId,byte& result);
void recv_return_set_not_gait_speed_decode(HexbotProtocol* dataFrame,short& speedFactor,byte& result);
void recv_return_navigate_pos_and_vel_decode(HexbotProtocol* dataFrame, float& xPos, float& yPos, float& zPos,float& vx, float& vy, float& vz);

#endif

