#include "pid_tracer.h"

#define PROT_FRAME_LEN_RECV  128

/* 校验数据的长度 */
#define PROT_FRAME_LEN_CHECKSUM    1

/* 数据头结构体 */
typedef __packed struct
{
  uint32_t head;    // 包头
  uint8_t ch;       // 通道
  uint32_t len;     // 包长度
  uint8_t cmd;      // 命令
//  uint8_t sum;      // 校验和
}packet_head_t;

#define FRAME_HEADER     0x59485A53    // 帧头

/* 通道宏定义 */
#define CURVES_CH1      0x01
#define CURVES_CH2      0x02
#define CURVES_CH3      0x03
#define CURVES_CH4      0x04
#define CURVES_CH5      0x05

/* 指令(下位机 -> 上位机) */
#define SEND_TARGET_CMD      0x01     // 发送上位机通道的目标值
#define SEND_FACT_CMD        0x02     // 发送通道实际值
#define SEND_P_I_D_CMD       0x03     // 发送 PID 值（同步上位机显示的值）
#define SEND_START_CMD       0x04     // 发送启动指令（同步上位机按钮状态）
#define SEND_STOP_CMD        0x05     // 发送停止指令（同步上位机按钮状态）
#define SEND_PERIOD_CMD      0x06     // 发送周期（同步上位机显示的值）

/* 指令(上位机 -> 下位机) */
#define SET_P_I_D_CMD        0x10     // 设置 PID 值
#define SET_TARGET_CMD       0x11     // 设置目标值
#define START_CMD            0x12     // 启动指令
#define STOP_CMD             0x13     // 停止指令
#define RESET_CMD            0x14     // 复位指令
#define SET_PERIOD_CMD       0x15     // 设置周期

/* 空指令 */
#define CMD_NONE             0xFF     // 空指令

/* 索引值宏定义 */
#define HEAD_INDEX_VAL       0x3u     // 包头索引值（4字节）
#define CHX_INDEX_VAL        0x4u     // 通道索引值（1字节）
#define LEN_INDEX_VAL        0x5u     // 包长索引值（4字节）
#define CMD_INDEX_VAL        0x9u     // 命令索引值（1字节）

#define EXCHANGE_H_L_BIT(data)      ((((data) << 24) & 0xFF000000) |\
                                     (((data) <<  8) & 0x00FF0000) |\
                                     (((data) >>  8) & 0x0000FF00) |\
                                     (((data) >> 24) & 0x000000FF))     // 交换高低字节

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))      // 合成为一个字
                                     

#include <string.h>

extern float set_point;
struct prot_frame_parser_t
{
    uint8_t *recv_ptr;
    uint16_t r_oft;
    uint16_t w_oft;
    uint16_t frame_len;
    uint16_t found_frame_head;
};

static struct prot_frame_parser_t parser;
static uint8_t recv_buf[PROT_FRAME_LEN_RECV];

/**
  * @brief 计算校验和
  * @param ptr：需要计算的数据
  * @param len：需要计算的长度
  * @retval 校验和
  */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len )
{
  uint8_t sum = init;
  
  while(len--)
  {
    sum += *ptr;
    ptr++;
  }
  
  return sum;
}

/**
 * @brief   得到帧类型（帧命令）
 * @param   *frame:  数据帧
 * @param   head_oft: 帧头的偏移位置
 * @return  帧长度.
 */
static uint8_t get_frame_type(uint8_t *frame, uint16_t head_oft)
{
    return (frame[(head_oft + CMD_INDEX_VAL) % PROT_FRAME_LEN_RECV] & 0xFF);
}

/**
 * @brief   得到帧长度
 * @param   *buf:  数据缓冲区.
 * @param   head_oft: 帧头的偏移位置
 * @return  帧长度.
 */
static uint16_t get_frame_len(uint8_t *frame, uint16_t head_oft)
{
    return ((frame[(head_oft + LEN_INDEX_VAL + 0) % PROT_FRAME_LEN_RECV] <<  0) |
            (frame[(head_oft + LEN_INDEX_VAL + 1) % PROT_FRAME_LEN_RECV] <<  8) |
            (frame[(head_oft + LEN_INDEX_VAL + 2) % PROT_FRAME_LEN_RECV] << 16) |
            (frame[(head_oft + LEN_INDEX_VAL + 3) % PROT_FRAME_LEN_RECV] << 24));    // 合成帧长度
}

/**
 * @brief   获取 crc-16 校验值
 * @param   *frame:  数据缓冲区.
 * @param   head_oft: 帧头的偏移位置
 * @param   head_oft: 帧长
 * @return  帧长度.
 */
static uint8_t get_frame_checksum(uint8_t *frame, uint16_t head_oft, uint16_t frame_len)
{
    return (frame[(head_oft + frame_len - 1) % PROT_FRAME_LEN_RECV]);
}

/**
 * @brief   查找帧头
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   start: 起始位置
 * @param   len: 需要查找的长度
 * @return  -1：没有找到帧头，其他值：帧头的位置.
 */
static int32_t recvbuf_find_header(uint8_t *buf, uint16_t ring_buf_len, uint16_t start, uint16_t len)
{
    uint16_t i = 0;

    for (i = 0; i < (len - 3); i++)
    {
        if (((buf[(start + i + 0) % ring_buf_len] <<  0) |
             (buf[(start + i + 1) % ring_buf_len] <<  8) |
             (buf[(start + i + 2) % ring_buf_len] << 16) |
             (buf[(start + i + 3) % ring_buf_len] << 24)) == FRAME_HEADER)
        {
            return ((start + i) % ring_buf_len);
        }
    }
    return -1;
}

/**
 * @brief   计算为解析的数据长度
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   start: 起始位置
 * @param   end: 结束位置
 * @return  为解析的数据长度
 */
static int32_t recvbuf_get_len_to_parse(uint16_t frame_len, uint16_t ring_buf_len,uint16_t start, uint16_t end)
{
    uint16_t unparsed_data_len = 0;

    if (start <= end)
        unparsed_data_len = end - start;
    else
        unparsed_data_len = ring_buf_len - start + end;

    if (frame_len > unparsed_data_len)
        return 0;
    else
        return unparsed_data_len;
}

/**
 * @brief   接收数据写入缓冲区
 * @param   *buf:  数据缓冲区.
 * @param   ring_buf_len: 缓冲区大小
 * @param   w_oft: 写偏移
 * @param   *data: 需要写入的数据
 * @param   *data_len: 需要写入数据的长度
 * @return  void.
 */
static void recvbuf_put_data(uint8_t *buf, uint16_t ring_buf_len, uint16_t w_oft,
        uint8_t *data, uint16_t data_len)
{
    if ((w_oft + data_len) > ring_buf_len)               // 超过缓冲区尾
    {
        uint16_t data_len_part = ring_buf_len - w_oft;     // 缓冲区剩余长度

        /* 数据分两段写入缓冲区*/
        memcpy(buf + w_oft, data, data_len_part);                         // 写入缓冲区尾
        memcpy(buf, data + data_len_part, data_len - data_len_part);      // 写入缓冲区头
    }
    else
        memcpy(buf + w_oft, data, data_len);    // 数据写入缓冲区
}

/**
 * @brief   查询帧类型（命令）
 * @param   *data:  帧数据
 * @param   data_len: 帧数据的大小
 * @return  帧类型（命令）.
 */
static uint8_t protocol_frame_parse(uint8_t *data, uint16_t *data_len)
{
    uint8_t frame_type = CMD_NONE;
    uint16_t need_to_parse_len = 0;
    int16_t header_oft = -1;
    uint8_t checksum = 0;
    
    need_to_parse_len = recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV, parser.r_oft, parser.w_oft);    // 得到为解析的数据长度
    if (need_to_parse_len < 9)     // 肯定还不能同时找到帧头和帧长度
        return frame_type;

    /* 还未找到帧头，需要进行查找*/
    if (0 == parser.found_frame_head)
    {
        /* 同步头为四字节，可能存在未解析的数据中最后一个字节刚好为同步头第一个字节的情况，
           因此查找同步头时，最后一个字节将不解析，也不会被丢弃*/
        header_oft = recvbuf_find_header(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.r_oft, need_to_parse_len);
        if (0 <= header_oft)
        {
            /* 已找到帧头*/
            parser.found_frame_head = 1;
            parser.r_oft = header_oft;
          
            /* 确认是否可以计算帧长*/
            if (recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV,
                    parser.r_oft, parser.w_oft) < 9)
                return frame_type;
        }
        else 
        {
            /* 未解析的数据中依然未找到帧头，丢掉此次解析过的所有数据*/
            parser.r_oft = ((parser.r_oft + need_to_parse_len - 3) % PROT_FRAME_LEN_RECV);
            return frame_type;
        }
    }
    
    /* 计算帧长，并确定是否可以进行数据解析*/
    if (0 == parser.frame_len) 
    {
        parser.frame_len = get_frame_len(parser.recv_ptr, parser.r_oft);
        if(need_to_parse_len < parser.frame_len)
            return frame_type;
    }

    /* 帧头位置确认，且未解析的数据超过帧长，可以计算校验和*/
    if ((parser.frame_len + parser.r_oft - PROT_FRAME_LEN_CHECKSUM) > PROT_FRAME_LEN_RECV)
    {
        /* 数据帧被分为两部分，一部分在缓冲区尾，一部分在缓冲区头 */
        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft, 
                PROT_FRAME_LEN_RECV - parser.r_oft);
        checksum = check_sum(checksum, parser.recv_ptr, parser.frame_len -
                PROT_FRAME_LEN_CHECKSUM + parser.r_oft - PROT_FRAME_LEN_RECV);
    }
    else 
    {
        /* 数据帧可以一次性取完*/
        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft, parser.frame_len - PROT_FRAME_LEN_CHECKSUM);
    }

    if (checksum == get_frame_checksum(parser.recv_ptr, parser.r_oft, parser.frame_len))
    {
        /* 校验成功，拷贝整帧数据 */
        if ((parser.r_oft + parser.frame_len) > PROT_FRAME_LEN_RECV) 
        {
            /* 数据帧被分为两部分，一部分在缓冲区尾，一部分在缓冲区头*/
            uint16_t data_len_part = PROT_FRAME_LEN_RECV - parser.r_oft;
            memcpy(data, parser.recv_ptr + parser.r_oft, data_len_part);
            memcpy(data + data_len_part, parser.recv_ptr, parser.frame_len - data_len_part);
        }
        else 
        {
            /* 数据帧可以一次性取完*/
            memcpy(data, parser.recv_ptr + parser.r_oft, parser.frame_len);
        }
        *data_len = parser.frame_len;
        frame_type = get_frame_type(parser.recv_ptr, parser.r_oft);

        /* 丢弃缓冲区中的命令帧*/
        parser.r_oft = (parser.r_oft + parser.frame_len) % PROT_FRAME_LEN_RECV;
    }
    else
    {
        /* 校验错误，说明之前找到的帧头只是偶然出现的废数据*/
        parser.r_oft = (parser.r_oft + 1) % PROT_FRAME_LEN_RECV;
    }
    parser.frame_len = 0;
    parser.found_frame_head = 0;

    return frame_type;
}

/**
 * @brief   接收数据处理
 * @param   *data:  要计算的数据的数组.
 * @param   data_len: 数据的大小
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len)
{
    recvbuf_put_data(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.w_oft, data, data_len);    // 接收数据
    parser.w_oft = (parser.w_oft + data_len) % PROT_FRAME_LEN_RECV;                          // 计算写偏移
}

/**
 * @brief   初始化接收协议
 * @param   void
 * @return  初始化结果.
 */
int32_t protocol_init(void)
{
    memset(&parser, 0, sizeof(struct prot_frame_parser_t));
    
    /* 初始化分配数据接收与解析缓冲区*/
    parser.recv_ptr = recv_buf;
  
    return 0;
}

/**
 * @brief   接收的数据处理
 * @param   void
 * @return  -1：没有找到一个正确的命令.
 */
int8_t receiving_process(void)
{
  uint8_t frame_data[128];         // 要能放下最长的帧
  uint16_t frame_len = 0;          // 帧长度
  uint8_t cmd_type = CMD_NONE;     // 命令类型
  
  while(1)
  {
    cmd_type = protocol_frame_parse(frame_data, &frame_len);
    switch (cmd_type)
    {
      case CMD_NONE:
      {
        return -1;
      }

      case SET_P_I_D_CMD:
      {
        uint32_t temp0 = COMPOUND_32BIT(&frame_data[13]);
        uint32_t temp1 = COMPOUND_32BIT(&frame_data[17]);
        uint32_t temp2 = COMPOUND_32BIT(&frame_data[21]);
        
        float p_temp, i_temp, d_temp;
        
        p_temp = *(float *)&temp0;
        i_temp = *(float *)&temp1;
        d_temp = *(float *)&temp2;
        
        set_p_i_d(p_temp, i_temp, d_temp);    // 设置 P I D
      }
      break;

      case SET_TARGET_CMD:
      {
        int actual_temp = COMPOUND_32BIT(&frame_data[13]);    // 得到数据
        
        set_point = (actual_temp);    // 设置目标值
      }
      break;
      
      case START_CMD:
      {
//        set_motor_enable();              // 启动电机
      }
      break;
      
      case STOP_CMD:
      {
//        set_motor_disable();              // 停止电机
      }
      break;
      
      case RESET_CMD:
      {
        HAL_NVIC_SystemReset();          // 复位系统
      }
      break;
      
      case SET_PERIOD_CMD:
      {
        uint32_t temp = COMPOUND_32BIT(&frame_data[13]);     // 周期数
        SET_BASIC_TIM_PERIOD(temp);                             // 设置定时器周期1~1000ms
      }
      break;

      default: 
        return -1;
    }
  }
}

/**
  * @brief 设置上位机的值
  * @param cmd：命令
  * @param ch: 曲线通道
  * @param data：参数指针
  * @param num：参数个数
  * @retval 无
  */
#include "bsp_uart.h"
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num)
{
  uint8_t sum = 0;    // 校验和
  num *= 4;           // 一个参数 4 个字节
  
  static packet_head_t set_packet;
  
  set_packet.head = FRAME_HEADER;     // 包头 0x59485A53
  set_packet.len  = 0x0B + num;      // 包长
  set_packet.ch   = ch;              // 设置通道
  set_packet.cmd  = cmd;             // 设置命令
  
  sum = check_sum(0, (uint8_t *)&set_packet, sizeof(set_packet));       // 计算包头校验和
  sum = check_sum(sum, (uint8_t *)data, num);                           // 计算参数校验和
  
  bsp_uart1_send_data((uint8_t *)&set_packet, sizeof(set_packet));
  bsp_uart1_send_data((uint8_t *)data, num);
  bsp_uart1_send_data((uint8_t *)&sum, sizeof(sum));
}

/**********************************************************************************************/