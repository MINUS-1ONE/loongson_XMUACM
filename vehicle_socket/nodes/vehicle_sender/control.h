#include "controlcan.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <inttypes.h>

#define MAX_CHANNELS 4
#define msleep(ms) usleep((ms)*1000)

class Control
{
public:
    Control()
    {
        // ----- open device -------------------------------------------------

        gDevType = 4;
        gDevIdx = 0;
        gChMask = 1;
        gBaud = 0x1c00;
        gTxType = 0;
        gTxSleep = 50;
        gTxFrames = 1;
        gTxCount = 1000;
        printf("DevType=%d, DevIdx=%d, ChMask=0x%x, Baud=0x%04x, TxType=%d, TxSleep=%d, TxFrames=0x%08x(%d), TxCount=0x%08x(%d)\n",
               gDevType, gDevIdx, gChMask, gBaud, gTxType, gTxSleep, gTxFrames, gTxFrames, gTxCount, gTxCount);

        if (!VCI_OpenDevice(gDevType, gDevIdx, 0))
        {
            printf("VCI_OpenDevice failed\n");
            return;
        }
        printf("VCI_OpenDevice succeeded\n");

        // ----- init & start -------------------------------------------------

        VCI_INIT_CONFIG config;
        config.AccCode = 0;
        config.AccMask = 0xffffffff;
        config.Filter = 1;
        config.Mode = 0;
        config.Timing0 = gBaud & 0xff;
        config.Timing1 = gBaud >> 8;

        for (int i = 0; i < MAX_CHANNELS; i++)
        {
            if ((gChMask & (1 << i)) == 0)
                continue;

            if (!VCI_InitCAN(gDevType, gDevIdx, i, &config))
            {
                printf("VCI_InitCAN(%d) failed\n", i);
                return;
            }
            printf("VCI_InitCAN(%d) succeeded\n", i);

            if (!VCI_StartCAN(gDevType, gDevIdx, i))
            {
                printf("VCI_StartCAN(%d) failed\n", i);
                return;
            }
            printf("VCI_StartCAN(%d) succeeded\n", i);
        }

        buf_brake = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));
        buf_steer = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));
        buf_throttle = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));
        buf_geer = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ));

        memset(buf_brake, 0, sizeof(VCI_CAN_OBJ));
        memset(buf_steer, 0, sizeof(VCI_CAN_OBJ));
        memset(buf_throttle, 0, sizeof(VCI_CAN_OBJ));
        memset(buf_geer, 0, sizeof(VCI_CAN_OBJ));

        // init control
        disable_all();
        enable_all();
    }

    ~Control()
    {
        VCI_CloseDevice(gDevType, gDevIdx);
    }

    void disable_all()
    {
        int err = 0;
        unsigned tx;

        buf_brake->ID = 0x111;
        buf_brake->SendType = gTxType;
        buf_brake->DataLen = 8;
        buf_brake->Data[0] = 0x00;
        buf_brake->Data[1] = 0x00;

        buf_steer->ID = 0x112;
        buf_steer->SendType = gTxType;
        buf_steer->DataLen = 8;
        buf_steer->Data[0] = 0x00;
        buf_steer->Data[1] = 0x00;

        buf_throttle->ID = 0x110;
        buf_throttle->SendType = gTxType;
        buf_throttle->DataLen = 8;
        buf_throttle->Data[0] = 0x00;
        buf_throttle->Data[1] = 0x00;

        buf_geer->ID = 0x114;
        buf_geer->SendType = gTxType;
        buf_geer->DataLen = 8;
        buf_geer->Data[0] = 0x03;
        buf_geer->Data[1] = 0x00;

        for (tx = 0; !err && tx < 10; tx++)
        {
            for (int i = 0; i < MAX_CHANNELS; i++)
            {
                if ((gChMask & (1 << i)) == 0)
                    continue;

                if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_brake, 1))
                {
                    printf("CAN%d TX failed: ID=%08x\n", i, buf_brake->ID);
                    err = 1;
                    break;
                }
                if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_steer, 1))
                {
                    printf("CAN%d TX failed: ID=%08x\n", i, buf_steer->ID);
                    err = 1;
                    break;
                }
                if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_throttle, 1))
                {
                    printf("CAN%d TX failed: ID=%08x\n", i, buf_throttle->ID);
                    err = 1;
                    break;
                }
                if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_geer, 1))
                {
                    printf("CAN%d TX failed: ID=%08x\n", i, buf_geer->ID);
                    err = 1;
                    break;
                }
            }
            if (gTxSleep)
                msleep(gTxSleep);
        }
    }

    void enable_all()
    {
        int err = 0;
        unsigned tx;

        buf_brake->Data[0] = 0x01;
        buf_brake->Data[1] = 0x00;

        buf_steer->Data[0] = 0x01;
        buf_steer->Data[1] = 0x00;

        buf_throttle->Data[0] = 0x01;
        buf_throttle->Data[1] = 0x00;

        for (tx = 0; !err && tx < 1; tx++)
        {
            for (int i = 0; i < MAX_CHANNELS; i++)
            {
                if ((gChMask & (1 << i)) == 0)
                    continue;

                if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_brake, 1))
                {
                    printf("CAN%d TX failed: ID=%08x\n", i, buf_brake->ID);
                    err = 1;
                    break;
                }
                if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_steer, 1))
                {
                    printf("CAN%d TX failed: ID=%08x\n", i, buf_steer->ID);
                    err = 1;
                    break;
                }
                if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_throttle, 1))
                {
                    printf("CAN%d TX failed: ID=%08x\n", i, buf_throttle->ID);
                    err = 1;
                    break;
                }
            }
            if (gTxSleep)
                msleep(gTxSleep);
        }
    }

    void brake(uint8_t value)
    {
        buf_brake->Data[0] = 0x01;
        buf_brake->Data[1] = value;

        for (int i = 0; i < MAX_CHANNELS; i++)
        {
            if ((gChMask & (1 << i)) == 0) continue;
            if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_brake, 1))
            {
                printf("CAN%d TX failed: ID=%08x\n", i, buf_brake->ID);
                break;
            }
        }
        if (gTxSleep) msleep(gTxSleep);
    }

    void steer(int16_t value)
    {
        buf_steer->Data[0] = 0x01;
        buf_steer->Data[1] = (uint8_t)(value & 0x00ff);
        buf_steer->Data[2] = (uint8_t)((value & 0xff00)>>8);

        for (int i = 0; i < MAX_CHANNELS; i++)
        {
            if ((gChMask & (1 << i)) == 0) continue;
            if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_steer, 1))
            {
                printf("CAN%d TX failed: ID=%08x\n", i, buf_steer->ID);
                break;
            }
        }
        if (gTxSleep) msleep(gTxSleep);
    }

    void throttle(uint8_t value)
    {
        buf_throttle->Data[0] = 0x01;
        buf_throttle->Data[1] = value;

        for (int i = 0; i < MAX_CHANNELS; i++)
        {
            if ((gChMask & (1 << i)) == 0) continue;
            if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_throttle, 1))
            {
                printf("CAN%d TX failed: ID=%08x\n", i, buf_throttle->ID);
                break;
            }
        }
        if (gTxSleep) msleep(gTxSleep);        
    }

    void gear(uint8_t value)
    {
        buf_geer->Data[0] = value;

        for (int i = 0; i < MAX_CHANNELS; i++)
        {
            if ((gChMask & (1 << i)) == 0) continue;
            if (1 != VCI_Transmit(gDevType, gDevIdx, i, buf_geer, 1))
            {
                printf("CAN%d TX failed: ID=%08x\n", i, buf_geer->ID);
                break;
            }
        }
        if (gTxSleep) msleep(gTxSleep);        
    }

    VCI_CAN_OBJ *buf_brake = nullptr;
    VCI_CAN_OBJ *buf_steer = nullptr;
    VCI_CAN_OBJ *buf_throttle = nullptr;
    VCI_CAN_OBJ *buf_geer = nullptr;

    unsigned gDevType = 0;
    unsigned gDevIdx = 0;
    unsigned gChMask = 0;
    unsigned gBaud = 0;
    unsigned gTxType = 0;
    unsigned gTxSleep = 0;
    unsigned gTxFrames = 0;
    unsigned gTxCount = 0;
};
