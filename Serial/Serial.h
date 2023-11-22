/*
*	@Author: PingLin Zhang
*	@Date:	 2020.04.13
*	@Brief:  Serial
*/
#ifndef  _USART_H
#define  _USART_H

//串口相关的头文件
#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/

#if defined(__linux__)  /* DO NOT INCLUDE UNIX HEAEDERS UNDER WINDOWS */
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#endif // _WIN32

#include<string.h>
#include<time.h>
#include<stdint.h>
#include "Platform.h"
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <memory>

/**
* @brief 输出接收数据使用的结构体
* @param mode :1为自瞄，2为吊射
*/
struct SerialReceiveData
{
    float INS_quat1;
    float INS_quat2;
    float INS_quat3;
    float INS_quat4;
    float yaw;
    float pitch;
    float distance;
    bool is_enemy_blue;
    int  mode = 1;
    float bullet_speed; // m/s
};

/**
 * @brief 向下位机发送数据使用的结构体
 */
struct SerialTransmitData
{

};

class Serial
{
public:
    Serial();
    ~Serial();

    static Serial* Get();

    static void ReceiveThreadTask();
    static void TransmitThreadTask();

    /**
    * @brief Serial main function
    * @param fire: Whether to doFire
    * @param find: whether to find armor
    */
    int SerialReceive(SerialReceiveData* receiveBuf, char *serial_device);
    int SerialSend(double yaw, double pitch, double distance, bool find, bool fire, int armorNum, char *serial_device);

    void EnableDebugOutput(bool receiveThread, bool transmitThread) { m_TransmitThreadDbgOutput = transmitThread; m_ReceiveThreadDbgOutput = receiveThread; };

protected:

    /*******************************************************************
    *名称：             UART0_Open
    *功能：             打开串口并返回串口设备文件描述
    *入口参数：         fd      文件描述符
                        port    串口号(ttyTHS0,ttyTHS1,ttyTHS2)
    *出口参数：正确返回为1，错误返回为0
    *******************************************************************/
//      int UART0_Open(int fd,char*port);

    /*******************************************************************
    *名称：             UART0_Set
    *功能：             设置串口数据位，停止位和效验位
    *入口参数：         fd          串口文件描述符
    *                   speed       串口速度
    *                   flow_ctrl   数据流控制
    *                   databits    数据位   取值为 7 或者8
    *                   stopbits    停止位   取值为 1 或者2
    *                   parity      效验类型 取值为N,E,O,,S
    *出口参数：正确返回为1，错误返回为0
    *******************************************************************/
    int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);

    /*******************************************************************
    *名称：                UART0_Init()
    *功能：                串口初始化
    *入口参数：            fd         文件描述符
    *                      speed      串口速度
    *                      flow_ctrl  数据流控制
    *                      databits   数据位   取值为 7 或者8
    *                      stopbits   停止位   取值为 1 或者2
    *                      parity     效验类型 取值为N,E,O,,S
    *
    *出口参数：正确返回为1，错误返回为0
    *******************************************************************/
    int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);

    /*******************************************************************
    * 名称：            UART0_Recv
    * 功能：            接收串口数据
    * 入口参数：        fd         文件描述符
    *                   rcv_buf    接收串口中数据存入rcv_buf缓冲区中
    *                   data_len   一帧数据的长度
    * 出口参数：        正确返回为1，错误返回errno
    *******************************************************************/
    int UART0_Recv(int fd, char *rcv_buf,int data_len);

    /********************************************************************
    * 名称：            UART0_Send
    * 功能：            发送数据
    * 入口参数：        fd           文件描述符
    *                   send_buf     存放串口发送数据
    *                   data_len     一帧数据的个数
    * 出口参数：        正确返回为1，错误返回为0
    *******************************************************************/
    int UART0_Send(int fd, uint8_t *send_buf,int data_len);

    /**
    * @brief 在串口读取到的数据中提取出一个数据包的数据段,转存到infoArray中，供后续解析为CSInfoStructure.
    * @param infoArray 存放一个完整的数据包
    * @param packages 从串口读取到的包含数据包的数据
    * @param sizepackages 从串口读取到的字节数（packages的大小）
    */
    bool get_one_in_packages(uint8_t * infoArray, uint8_t * packages);

    /**
     * @brief  把存有一个数据段的数组解析为一个数据结构体,结果存到参数2对应的地址
     * @param  infoArray 存有一个数据段的uint8类型的数组
     * @param  infoStrc 从串口读取到的字节数（packages的大小）
     * @retval 无
     */
    void buff_to_vision_receive(uint8_t * uart0_recv_buf, SerialReceiveData* receive_buf);

private:
    static Serial* m_instance; // 单例

    // 常量定义
    static constexpr uint8_t ReceiveFrameHead = 0x50, ReceiveFrameEnd = 0x22,
                             SendFrameHead = 0xAA, SendFrameEnd = 0xBB;

    // 接收缓冲区和原子交换
    PACK(struct _RawReceivedFrame {
        uint8_t FrameHead;
        //TODO：四元数现在不是那么需要了，有空把它去掉
        float INS_quat1;
        float INS_quat2;
        float INS_quat3;
        float INS_quat4;
        int32_t yaw;
        int16_t pitch;
        uint8_t EnemyColor;
        float BulletSpeed;
        uint8_t FrameEnd;
    });
    PACK(struct _RawTransmissionFrame {
        uint8_t FrameHead;
        int32_t Yaw;
        int16_t Pitch;
        int16_t Distance;
        uint8_t Found;
        uint8_t Fire;
        uint8_t FrameEnd;
    });
    // 这两个原子整数均为双缓冲的读取者使用的，写入者必须要用1减去它
    std::atomic_int32_t m_ReceiverBufferIndex, m_TransmitBufferIndex;
    std::shared_ptr<_RawReceivedFrame> m_ReceiveBuffer[2];
    std::shared_ptr<_RawTransmissionFrame> m_TransmissionBuffer[2];

    // 接收数据的原始字节存放在此，处理后才写入m_ReceiveBuffer
    char* m_ReceivePreprocessBuffer = nullptr;

    // 条件变量和它拥有的锁，让串口发送只在有新数据时再触发发送
    std::condition_variable m_SendCondVar;
    std::mutex m_SendCondMutex;

    // 设置串口设备名称加的🔓
    std::mutex m_MutexSetDevice;
    std::string m_SerialPortDevice;

    // 串口调试日志开关
    bool m_TransmitThreadDbgOutput = false, m_ReceiveThreadDbgOutput = false;

};

#endif


