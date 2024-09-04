#include "mbed.h"
#include "PID.hpp"

BufferedSerial pc(USBTX, USBRX, 250000); // パソコンとのシリアル通信
CAN can(PA_11, PA_12, (int)1e6);
uint8_t DATA[8] = {};
PID pidControllerRight(0.001, 0.00, 0.00, 0.0000, 501, 0.01);
int targetSpeed = 0;
int currentSpeed = 0;
int16_t outputpicInt16 = 0;

void can_thread()
{
    while (1)
    {
        CANMessage msg1;
        if (can.read(msg1) && msg1.id == 0x201)
        {
            currentSpeed = (msg1.data[2] << 8) | msg1.data[3];
        }

        // PID制御の計算
        float output = pidControllerRight.calculate(targetSpeed, currentSpeed);
        outputpicInt16 = static_cast<int16_t>(output);

        // デバッグ用の出力を追加
        char buffer[80];
        sprintf(buffer, "Target: %d, Speed: %d, Output: %f\n", targetSpeed, currentSpeed, output);
        pc.write(buffer, strlen(buffer));

        ThisThread::sleep_for(10ms);
    }
}

int main()
{
    Thread canThread;
    canThread.start(can_thread);

    while (1)
    {
        if (pc.readable())
        {
            char buf;
            pc.read(&buf, sizeof(buf));
            if (buf == 'w')
            {
                targetSpeed = 1000;
            }
            else if (buf == 's')
            {
                targetSpeed = 8000;
            }
        }

        // CANメッセージの送信
        DATA[0] = outputpicInt16 >> 8;   // MSB
        DATA[1] = outputpicInt16 & 0xFF; // LSB
        CANMessage msg0(0x200, DATA, 8);
        can.write(msg0);

        ThisThread::sleep_for(10ms);
    }
}