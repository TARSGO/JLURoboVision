
#include <mutex>
#include <iostream>
#include "Serial.h"
#include "imgui.h"

Serial *Serial::m_instance = nullptr;

Serial::Serial()
{
    m_ReceiveBuffer[0] = std::make_shared<_RawReceivedFrame>();
    m_ReceiveBuffer[1] = std::make_shared<_RawReceivedFrame>();
    m_TransmissionBuffer[0] = std::make_shared<_RawTransmissionFrame>();
    m_TransmissionBuffer[1] = std::make_shared<_RawTransmissionFrame>();
    m_SerialPortDevice = "/dev/ttyACM1";
}

Serial::~Serial()
{
}

/*
 优先目标数字 - targetNum - 前三位（数字1 - 8——1英雄，2工程，345步兵，6哨兵，7前哨战，8基地）
 模式选择mode - 后五位（0 不处理，1 - 8留作模式选择，1为手动开火，2为自瞄，3为小符，4为大符）
 */
constexpr int INFOSIZE = 13;  //接收max数据bao的大小


int Serial::SerialReceive(SerialReceiveData* receiveBuf, char *serial_device)
{
    return 0;
}

int
Serial::SerialSend(double yaw, double pitch, double distance, bool find, bool fire, int armorNum, char *serial_device)
{

    return 0;
}

int Serial::UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    return true;
}

int Serial::UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    return true;
}

int Serial::UART0_Recv(int fd, char *rcv_buf,int data_len)
{
    return 0;
}

int Serial::UART0_Send(int fd, uint8_t *send_buf,int data_len)
{
    return 0;
}

bool Serial::get_one_in_packages(uint8_t * infoArray, uint8_t * packages)
{

    return false;
}

void Serial::buff_to_vision_receive(uint8_t * uart0_recv_buf, SerialReceiveData* vision_receive)
{

}

Serial *Serial::Get() {
    static std::once_flag once;
    std::call_once(once, [&](){ m_instance = new Serial; }); // 创建串口的单例，这一句是线程安全的
    return m_instance;
}

void Serial::ReceiveThreadTask() {
    std::cerr << "Serial::ReceiveThreadTask stub. Thread will exit now!\n";
}


void Serial::TransmitThreadTask() {
    std::cerr << "Serial::TransmitThreadTask stub. Thread will exit now!\n";
}




