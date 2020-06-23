/***********************************************************************************************************************
 *        HEXBOT通信协议实现文件                                                                       *
 *        Copyright (c)     VK,2017                                                                                    *
 *        Author :          Pei.He                                                                                     *
 *        Version number :  1.00                                                                                       *
 *        Date :            20170222                                                                                   *
 ***********************************************************************************************************************/
#include "hexbotProtocol.h"
#include <cstring>

HexbotProtocol::HexbotProtocol(){
  resetDataFrame();
}
void HexbotProtocol::setStartFlag1(byte flag1){
  this->startFlag1 = flag1;
}

byte HexbotProtocol::getStarFlag1(){
  return this->startFlag1;
}

void HexbotProtocol::setStartFlag2(byte flag2){
  this->startFlag2 = flag2;
}

byte HexbotProtocol::getStarFlag2(){
  return this->startFlag2;
}
void HexbotProtocol::setMessageType(MESSAGE_TYPE messageType){
  this->messageType = messageType;
}

byte HexbotProtocol::getMessageType(){
  return this->messageType;
}

void HexbotProtocol::setMessageId(byte messageId){
  this->messageId = messageId;
}

byte HexbotProtocol::getMessageId(){
  return this->messageId;
}

void HexbotProtocol::setPayloadLen(byte payloadLen){   
  this->payloadLen = payloadLen;
}

byte HexbotProtocol::getPayloadLen(){   
  return this->payloadLen;
}

byte HexbotProtocol::getPayloadParam(int index){  
  if(index < this->payloadLen){
      return this->payload[index];
   }else                         
      return 0;
}

void HexbotProtocol::setPayloadParam(int index,int value){      
  if(index < MAX_PAYLOAD_LEN)
    this->payload[index] = (unsigned char)value;
}

byte HexbotProtocol::getMessageTotalLen(){
  return this->payloadLen + MIN_MESSGE_LEN + CRC_LEN;
}

/*从byte[]数组中，解析成数据帧格式*/
void HexbotProtocol::initDataFrame(byte data[],int len ,int startPos){

}

/*将数据帧格式转换为byte[],为发送数据做准备*/
void HexbotProtocol::dataFrameToByteArray(byte data[],int& len){  
  len = MIN_MESSGE_LEN + getPayloadLen()+CRC_LEN;   
  int index = 0;
  data[index] = getStarFlag1();     
  index++;
  data[index] = getStarFlag2();                 
  index++;
  data[index] = getMessageType(); 
  index++;
  data[index] = getMessageId();
  index++;
  data[index] = getPayloadLen();
  index++;
  memcpy(&data[index],payload,getPayloadLen());    
  index += getPayloadLen();                         
  data[index] = this->crc;     
}

//在payload尾部添加内容，同时修改Payload长度值，只是数据部分，非整个信息
void HexbotProtocol::addPayload(byte data[],int len,int startPos){
  int payloadIndex = getPayloadLen();           //获取数据长度
  if((payloadIndex+len) > MAX_PAYLOAD_LEN)                //若数据大于允许最大字节255，即返回
    return;
  memcpy(&payload[payloadIndex],&data[startPos],len);   //将data的startPos开始装载len长度到payload的payloadIndex
  payloadIndex += len;                           //数据长度
  setPayloadLen(payloadIndex);         //更新payload的长度 
}


//在payload的指定位置pos开始处添加内容，Payload长度值不变
void HexbotProtocol::fillPayload(byte data[],int len,int pos){
   if((pos+len) > MAX_PAYLOAD_LEN)
     return;
   memcpy(&payload[pos],data,len);
}
void HexbotProtocol::getPayload(byte data[],int &len){
  len = getPayloadLen();     //获取数据长度
  memcpy(data,payload,len);        //将payload的len长度复制到data
}

void HexbotProtocol::resetDataFrame(){                   //数据清零
  startFlag1 = VK_PROTOCOL_STX1;               //信息起始标志1
  startFlag2 = VK_PROTOCOL_STX2;               //信息起始标志2
  messageType = 0x00;     
  messageId = 0x00; 
  payloadLen = 0x00;                 //长度清零
  memset(payload,0,MAX_PAYLOAD_LEN);        //清除payload
  this->crc = 0x00;
}

void HexbotProtocol::copyFromInstance(HexbotProtocol message){     
  this->startFlag1 = message.getStarFlag1();
  this->startFlag2 = message.getStarFlag2();
  this->messageType = message.getMessageType();
  this->messageId = message.getMessageId();
  this->payloadLen = message.getPayloadLen();
  memcpy(this->payload,message.payload,message.getPayloadLen());
  this->crc = message.getFrameCrc();
}

byte HexbotProtocol::calCrc(byte* buf,byte len){
  byte crc = 0;
  byte sum = 0;
  for(int i=0;i<len;i++){        //求和
    sum += buf[i];
  }
  crc =(byte) ~sum;           //按位取反
  return crc;
}

void HexbotProtocol::calFrameCrc(){
  int len = this->getPayloadLen();
  byte *buf = new byte[len+2+1]; 
  int index = 0;
  buf[index] = getMessageType();
  index++;
  buf[index] = getMessageId();
  index++;
  buf[index] = getPayloadLen();
  index++;
  int l=0;
  getPayload(&buf[index],l);     //从buf的index位置开始装载payload
  this->crc = calCrc(buf,len+3);      //获取校验位
  delete buf;
}

byte HexbotProtocol::getFrameCrc(){
  return this->crc;
}
