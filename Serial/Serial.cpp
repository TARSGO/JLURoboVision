
#include <mutex>
#include <iostream>
#include <fstream>
#include "General.h"
#include "Serial.h"
#include "imgui.h"

#include <sys/poll.h>

using namespace std;

Serial *Serial::m_instance = nullptr;

fstream receiveLog;

Serial::Serial()
{
    m_ReceiveBuffer[0] = std::make_shared<_RawReceivedFrame>();
    m_ReceiveBuffer[1] = std::make_shared<_RawReceivedFrame>();
    m_TransmissionBuffer[0] = std::make_shared<_RawTransmissionFrame>();
    m_TransmissionBuffer[1] = std::make_shared<_RawTransmissionFrame>();
    m_ReceiverBufferIndex = 0;
    m_TransmitBufferIndex = 0;

    m_ReceivePreprocessBuffer = new char[8192];

    receiveLog.open(CurrentDateTime(), std::ios::out);

    // HACK
    FILE* serialDetect = NULL;
    serialDetect = fopen("/dev/ttyACM0", "r");
    if (serialDetect != NULL) {
        fclose(serialDetect);
        m_SerialPortDevice = "/dev/ttyACM0";
    }
    else
        m_SerialPortDevice = "/dev/ttyACM1";

}

Serial::~Serial()
{
    delete[] m_ReceivePreprocessBuffer;
}

/*
 优先目标数字 - targetNum - 前三位（数字1 - 8——1英雄，2工程，345步兵，6哨兵，7前哨战，8基地）
 模式选择mode - 后五位（0 不处理，1 - 8留作模式选择，1为手动开火，2为自瞄，3为小符，4为大符）
 */
constexpr int INFOSIZE = 13;  //接收max数据bao的大小


int Serial::SerialReceive(SerialReceiveData* receiveBuf, char *serial_device)
{
    {
        // 新开一个作用域，让shared_ptr引用大于1的时间尽量短
        auto serial = Serial::Get();
        auto BufferSharedRef = serial->m_ReceiveBuffer[serial->m_ReceiverBufferIndex.load()];

        receiveBuf->INS_quat1 = BufferSharedRef->INS_quat1 * 0.01;
        receiveBuf->INS_quat2 = BufferSharedRef->INS_quat2 * 0.01;
        receiveBuf->INS_quat3 = BufferSharedRef->INS_quat3 * 0.01;
        receiveBuf->INS_quat4 = BufferSharedRef->INS_quat4 * 0.01;
        //left is positive
        receiveBuf->yaw = BufferSharedRef->yaw * 0.01;
        //down is positive
        receiveBuf->pitch = BufferSharedRef->pitch * 0.01;
        receiveBuf->is_enemy_blue = BufferSharedRef->EnemyColor;
        receiveBuf->bullet_speed = BufferSharedRef->BulletSpeed;
        /*cout << "0:" << receiveBuf->INS_quat1<<endl;
        cout << "1:" << receiveBuf->INS_quat2<<endl;
        cout << "2:" << receiveBuf->INS_quat3<<endl;
        cout << "3:" << receiveBuf->INS_quat4<<endl;
        cout << "bullet speed :" << receiveBuf -> bullet_speed<<endl;*/
    }



/*    ImGui::Begin("SerialReceive");
    ImGui::Text("INS_quat1 : %f", receiveBuf->INS_quat1);
    ImGui::Text("INS_quat2 : %f", receiveBuf->INS_quat2);
    ImGui::Text("INS_quat3 : %f", receiveBuf->INS_quat3);
    ImGui::Text("INS_quat4 : %f", receiveBuf->INS_quat4);
    ImGui::Text("bullet_speed :%f m/s", receiveBuf->bullet_speed);
    ImGui::Text("Enemy Color: ");
    ImGui::SameLine();
    ImGui::TextColored(receiveBuf->is_enemy_blue ? ImVec4(.2, .2, 1, 1) : ImVec4(1, .2, .2, 1),
                       receiveBuf->is_enemy_blue ? "BLUE" : "RED");
    ImGui::End();*/
    return 0;
}

int Serial::SerialSend(double yaw, double pitch, double distance, bool find, bool fire, int armorNum, char *serial_device)
{
    auto serial = Serial::Get();
    ImGui::BeginChild("SerialPortControl");
    ImGui::Text("Yaw: %lf", yaw);
    ImGui::Text("Pitch: %lf", pitch);
    ImGui::Text("Distance: %lf", distance);
    ImGui::EndChild();

    auto BufferIndex = 1 - serial->m_TransmitBufferIndex.load();
    auto txBuffer = serial->m_TransmissionBuffer[BufferIndex];
    txBuffer->FrameHead = 0xAA;
    txBuffer->Yaw = yaw * 100;
    txBuffer->Pitch = pitch * 100;
    txBuffer->Distance = distance * 100;
    txBuffer->Fire = fire;
    txBuffer->Found = find;
    txBuffer->FrameEnd = 0xB0;

    // 尝试交换缓冲区
    if (serial->m_TransmissionBuffer[serial->m_TransmitBufferIndex.load()].use_count() > 1) // 大于1，没传完
        return 1;
    serial->m_TransmitBufferIndex.exchange(BufferIndex); // 交换缓冲区
    serial->m_SendCondVar.notify_all(); // 唤醒写线程

    return 0; // FIXME
}

int Serial::UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200,B57600, B38400,B19200,B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200, 57600,38400, 19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return false;
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

        case 0 ://不使用流控制
            options.c_cflag &= ~CRTSCTS;
            break;

        case 1 ://使用硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2 ://使用软件流控制
            options.c_cflag |= IXON | IXOFF | IXANY;
            break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
        case 5    :
            options.c_cflag |= CS5;
            break;
        case 6    :
            options.c_cflag |= CS6;
            break;
        case 7    :
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n");
            return false;
    }
    //设置校验位
    switch (parity)
    {
        case 'n':
        case 'N': //无奇偶校验位。
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O'://设置为奇校验
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E'://设置为偶校验
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S': //设置为空格
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return false;
    }
    // 设置停止位
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB; break;
        case 2:
            options.c_cflag |= CSTOPB; break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return false;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return false;
    }
    return true;
}

int Serial::UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err=0;
    //设置串口数据帧格式
    if (UART0_Set(fd,115200,0,8,2,'N') == false)
    {
        return false;
    }
    else
    {
        return  true;
    }
}

int Serial::UART0_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);    //如果fd == -1, FD_SET将在此阻塞

    time.tv_sec = 0;
    time.tv_usec = 10000;//

    //串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);

    if(fs_sel != -1)
    {
        len = read(fd,rcv_buf,data_len);
        return (len == -1 ? -errno : len);
    }
    else
    {
        return -errno;
    }
}

int Serial::UART0_Send(int fd, uint8_t *send_buf,int data_len)
{
    int len = 0;
    //uint8_t sdbuf[4] = {0xaa,0xbb,0xcc,0xdd};
    len = write(fd,send_buf,data_len);
    //write(fd,sdbuf,4);
    if (len == data_len )
    {
        return len;
    }
    else
    {
        tcflush(fd,TCOFLUSH);
        return false;
    }
}

bool Serial::get_one_in_packages(uint8_t * infoArray, uint8_t * packages)
{
    int ptr;
    for (ptr = 0; ptr < INFOSIZE * 2; ptr++) {
        if (packages[ptr] == 0xAB){
            for (int i = 0; i < INFOSIZE; i++){
                infoArray[i] = packages[ptr + i];
            }
            return true;
        }
    }
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
    int fd = -1, err = 0;
    auto serial = Get();
    static int errorCounter = 0;

    while(1) {
        // 打开串口
        // FIXME: 怎么锁这个字符串
        fd = open(serial->m_SerialPortDevice.c_str(),O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
        if(fd == -1) continue;

        // 初始化串口
        do{
            err = serial->UART0_Init(fd,115200,0,8,1,'N');
        }while(false == err || false == fd);

        while(true) {
            // 主循环
            int TotalReceived = 0, BytesReceived = 0;
            _RawReceivedFrame* foundFrame = nullptr;

            // 先把系统的缓冲区全读出来，4K 4K地读（防止读空了之后最后一次读出来的不够一帧）
            int bufferHalf = 0;
            auto readBuf = serial->m_ReceivePreprocessBuffer;
            // 等待到数据可用
            pollfd pollstruct {}; pollstruct.fd = fd; pollstruct.events = POLLIN;
            ::poll(&pollstruct, 1, -1);
            while(true) {
                int readBytes = serial->UART0_Recv(fd,
                                                   readBuf + bufferHalf * 4096,
                                                   4096);
                if (readBytes < 0) {
                    // 出错情况，先判断是否是已读完
                    if (readBytes == -EAGAIN) {
                        break;
                    }
                } else {
                    TotalReceived += readBytes;
                    if (readBytes < 4096) {
                        // 已经读完
                        break;
                    }
                }
                bufferHalf = 1 - bufferHalf;
            };

            // 要是还没读够一帧就放弃
            if (TotalReceived < sizeof(_RawReceivedFrame)) {
                continue;
            }

            // 从尾部寻找帧尾
            for (int i = TotalReceived % 4096 + bufferHalf * 4096 - 1;
                 i >= sizeof(_RawReceivedFrame) - 1;
                 i--) {
                if (readBuf[i] == ReceiveFrameEnd &&
                    readBuf[i - sizeof(_RawReceivedFrame) + 1] == ReceiveFrameHead) {
                    // 找到了
                    if (serial->m_ReceiveThreadDbgOutput) {
                        cout << "SerialReceiveThread: Found a seemingly valid frame\n";
                        HexDump(cout, readBuf + (i - sizeof(_RawReceivedFrame) + 1), sizeof(_RawReceivedFrame));
                    }
                    foundFrame = reinterpret_cast<_RawReceivedFrame*>(&readBuf[i - sizeof(_RawReceivedFrame) + 1]);
                }
            }

            if (!foundFrame) {
                if (serial->m_ReceiveThreadDbgOutput) {
                    cout << "SerialReceiveThread: Didn't find a seemingly valid frame!\n";
                }
                continue;
            }

            int BufferIndex = 1 - serial->m_ReceiverBufferIndex.load();
            char* ReadBuffer = reinterpret_cast<char*>(serial->m_ReceiveBuffer[BufferIndex].get());
            auto ReadStruct = serial->m_ReceiveBuffer[BufferIndex].get();

            memcpy(ReadBuffer, foundFrame, sizeof(_RawReceivedFrame));

            // 验证数据
            if(ReadStruct->FrameHead != Serial::ReceiveFrameHead ||
               ReadStruct->FrameEnd != Serial::ReceiveFrameEnd) {
                errorCounter++;
                if ((errorCounter % 10) == 0)
                    errorCounter = 0;
                std::cerr << "Serial::ReceiveThreadTask: Retrying due to invalid data. " << std::endl; // 如果帧头帧尾有问题，跳出去重试
                    HexDump(cout, ReadStruct, sizeof(*ReadStruct));
                break;
            }

            if (serial->m_ReceiveThreadDbgOutput) {
              /*  cout << "Serial::ReceiveThreadTask: yaw=" << ReadStruct->Yaw / 100.0 << "\t pitch="
                     << ReadStruct->Pitch / 100.0 << endl;*/
                HexDump(cout, ReadStruct, sizeof(*ReadStruct));
                //receiveLog << ReadStruct->Yaw << "," << ReadStruct->Pitch << ",\n";
            }

            // 尝试交换缓冲区
            if (serial->m_ReceiveBuffer[serial->m_ReceiverBufferIndex.load()].use_count() > 1) // 大于1，说明还有其他人在读
                continue;
            serial->m_ReceiverBufferIndex = BufferIndex; // 交换缓冲区
        }
ReadReopenSerialPort:
        close(fd);
    }
    assert(false);
}


void Serial::TransmitThreadTask() {
    int fd = -1, err = 0;
    auto serial = Get();

    while(1) {
        // 打开串口
        // FIXME: 怎么锁这个字符串
        fd = open(serial->m_SerialPortDevice.c_str(),O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
        if(fd == -1) continue;

        // 初始化串口
        do{
            err = serial->UART0_Init(fd,115200,0,8,1,'N');
        }while(false == err || false == fd);

        while(true) {
            std::unique_lock<std::mutex> uniqueLock(serial->m_SendCondMutex);
            serial->m_SendCondVar.wait(uniqueLock);

//            cout << "/==== Send Thread activated " << CurrentPreciseTime() << " =====\\\n";
            auto transmissionBuffer = serial->m_TransmissionBuffer[serial->m_TransmitBufferIndex.load()];
            serial->UART0_Send(fd, (uint8_t*)(transmissionBuffer.get()), sizeof(_RawTransmissionFrame));
//            cout << "\\==== Send Thread finished " << CurrentPreciseTime() << " =====/\n";
        }
    }
    assert(false);
}




