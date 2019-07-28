#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "fifo.h"
#include "cmsis_os.h"

#include "MecanumKinematics.h"

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
extern void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
  
//USB底层发送函数，直接操作硬件
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//计算式输入数据
communicate_class_input_data_t communicate_input_data;
//计算式输出结果
communicate_class_output_data_t communicate_output_data;

chassis_ctrl_info_t chassis_ctrl_input_data;

extern QueueHandle_t recvMotorQueue;

//实现RM协议的序列化过程
void referee_send_data(uint16_t cmd_id, void* buf, uint16_t len)
{
    //TODO 定义至少128字节大小缓存数组
    static uint8_t send_buf[128];
    uint16_t index = 0;
    //TODO 定义帧头结构体
    frame_header_struct_t referee_send_header;
    
    //TODO 初始化对应帧头结构体
    referee_send_header.SOF = HEADER_SOF;
    referee_send_header.data_length = len;
    referee_send_header.seq++;
    
    //TODO 生成CRC8校验
    Append_CRC8_Check_Sum((uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    
    memcpy(send_buf, (uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(send_buf + index, (void*)&cmd_id, sizeof(uint16_t));
    index += sizeof(uint16_t);

    //TODO 填充数据包
    memcpy(send_buf + index, (void*)buf, len);
    index += len;

    //TODO 生成CRC16校验
    Append_CRC16_Check_Sum(send_buf, REF_HEADER_CRC_CMDID_LEN + len);
    index += sizeof(uint16_t);
    
    //TODO 调用底层发送函数
    CDC_Transmit_FS(send_buf, index);
}

//TODO：实现计算式运算和结果上发
//计算式信息包反序列化已经实现
//通过communicate_input_data结构体可以直接获取计算式信息
//operate '+' '-' '*' '/'
void communicate_class_solve(void) {
	;
}

double topLeft = 0;
double topRight = 0;
double backLeft = 0;
double backRight = 0;

float ab = 2.0f;
float r = 1.0f;

void chassis_command_solve(){
	Twist2D input;
	
	input.vX = chassis_ctrl_input_data.vx;
	input.vY = chassis_ctrl_input_data.vy;
	input.w = chassis_ctrl_input_data.vw;
	
	MecanumWheelValues values;
	
	mecanumInverseKinematics(&input, ab, r, &values);
	
	topLeft = values.topLeft;
	topRight = values.topRight;
	backLeft = values.backLeft;
	backRight = values.backRight;
	
	
	xQueueSend(recvMotorQueue, (void*) (&input), (TickType_t) 100);
}

