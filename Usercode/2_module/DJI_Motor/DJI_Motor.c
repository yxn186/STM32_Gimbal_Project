/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    DJI_Motor.c
  * @brief   大疆系电机库（目前只支持单种电机多个使用）（3508 6020）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "DJI_Motor.h"
#include "bsp_can.h"
#include <stdint.h>

/**
 * @brief 基准ID 用3508选0x201 用6020选0x205
 * 
 */
#define Motor_ID_Start 0x201

/**
 * @brief 6020和3508电机输出限幅宏定义
 * 
 */
#define DJI_Motor_6020_Out_Max  25000
#define DJI_Motor_6020_Out_Min  -25000
#define DJI_Motor_3508_Out_Max  16384
#define DJI_Motor_3508_Out_Min  -16384

/**
 * @brief 发送数据数组（static）
 * 
 */
static uint8_t TxData_200[8] = {0};
static uint8_t TxData_1FF[8] = {0};
static uint8_t TxData_2FF[8] = {0};

static DJI_Motor_Data_t *DJI_Motors_Data_Temp = NULL;

/**
 * @brief 大疆电机获取数据函数（回调调用）
 * 
 * @param RxBuffer 
 * @param DJI_Motors_Data 
 */
void DJI_Motor_Get_Data(CAN_Rx_Buffer_t *RxBuffer,DJI_Motor_Data_t *DJI_Motors_Data)
{
    DJI_Motors_Data->RawAngle = (RxBuffer->Data[0] << 8) | RxBuffer->Data[1];
    DJI_Motors_Data->speed_rpm = (int16_t)(((uint16_t)RxBuffer->Data[2] << 8) | RxBuffer->Data[3]);
    DJI_Motors_Data->Torque_Current = (int16_t)(((uint16_t)RxBuffer->Data[4] << 8) | RxBuffer->Data[5]);
    DJI_Motors_Data->Temperature = RxBuffer->Data[6];
}

/**
 * @brief 大疆电机接收回调函数
 * 
 * @param RxBuffer 接收缓冲区
 */
static void DJI_Motor_RxCallBack(CAN_Rx_Buffer_t *RxBuffer)
{
    if (!DJI_Motors_Data_Temp)//防炸
    {
        return;
    }

    uint16_t ID = RxBuffer->Header.StdId;

    if (ID < Motor_ID_Start || ID >= Motor_ID_Start + 8)//防炸
    {
       return;
    }

    DJI_Motor_Get_Data(RxBuffer, &DJI_Motors_Data_Temp[ID - Motor_ID_Start]); 
}

/**
 * @brief 大疆电机CAN初始化函数
 * 
 * @param hcan hcanx
 * @param Object_Para 编号[3:] | FIFOx[2:2] | ID类型[1:1] | 帧类型[0:0]
 * @param ID ID 填写IDx_xxxx
 * @param Mask_ID 屏蔽位
 */
void DJI_Motor_CAN_Init(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_Filter_Mask_Config(hcan,Object_Para,ID,Mask_ID);
    CAN_Init(hcan);
}

/**
 * @brief 大疆电机初始化函数
 * 
 * @param hcan hcanx
 * @param DJI_Motors_Data 电机数据结构体数组[8]（一定要是8！！）
 */
void DJI_Motor_Init(CAN_HandleTypeDef *hcan,DJI_Motor_Data_t DJI_Motors_Data[8])
{
    DJI_Motors_Data_Temp = DJI_Motors_Data;
    CAN_Register_RxCallBack_FIFO0_Function(DJI_Motor_RxCallBack);
    DJI_Motor_CAN_Init(hcan,CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE,0x200, 0x7E0);
}

/**
 * @brief 大疆单电机控制函数
 * 
 * @param hcan hcanx
 * @param DJI_Motor_Type 电机型号 DJI_Motor_6020 / DJI_Motor_3508
 * @param DJI_Motor_ID 电机CAN-ID  6020：1~7   3508：1~8
 * @param Out 输出值 6020电压控制：-25000~0~25000   3508电流控制：-16384~0~16384
 */
void DJI_Motor_Control_Single(CAN_HandleTypeDef *hcan,DJI_Motor_Type_Typedef DJI_Motor_Type,uint16_t DJI_Motor_ID,int16_t Out)
{
    uint8_t *TxData = NULL;

    uint16_t TxID = 0;

    //1 判断电机型号 赋值发送ID和限幅Out
    if(DJI_Motor_Type == DJI_Motor_6020)
    {
        if(DJI_Motor_ID > 0 && DJI_Motor_ID <= 4)
        {
            TxID = 0x1FF;
            TxData = TxData_1FF;
        }
        else if(DJI_Motor_ID > 4 && DJI_Motor_ID <= 7)
        {
            TxID = 0x2FF;
            TxData = TxData_2FF;
            DJI_Motor_ID = DJI_Motor_ID - 4;
        }

        if(Out >= DJI_Motor_6020_Out_Max)
        {
            Out = DJI_Motor_6020_Out_Max;
        }
        else if (Out <= DJI_Motor_6020_Out_Min)
        {
            Out = DJI_Motor_6020_Out_Min;
        }
    }
    else if(DJI_Motor_Type == DJI_Motor_3508)
    {
        if(DJI_Motor_ID > 0 && DJI_Motor_ID <= 4)
        {
            TxID = 0x200;
            TxData = TxData_200;
        }
        else if(DJI_Motor_ID > 4 && DJI_Motor_ID <= 8)
        {
            TxID = 0x1FF;
            TxData = TxData_1FF;
            DJI_Motor_ID = DJI_Motor_ID - 4;
        }

        if(Out >= DJI_Motor_3508_Out_Max)
        {
            Out = DJI_Motor_3508_Out_Max;
        }
        else if (Out <= DJI_Motor_3508_Out_Min)
        {
            Out = DJI_Motor_3508_Out_Min;
        }
    }

    if(TxID == 0)//防炸
    {
        return;
    }

    //2 数据处理 
    uint8_t DJI_Motor_ID_Temp = (uint8_t)(DJI_Motor_ID - 1);
    TxData[2*DJI_Motor_ID_Temp]   = (uint8_t)((uint16_t)Out >> 8);
    TxData[2*DJI_Motor_ID_Temp+1] = (uint8_t)((uint16_t)Out);

    CAN_Send_Data(hcan,TxID,TxData,8);
}

/**
 * @brief 大疆双电机控制函数（针对6020电机）
 * 
 * @param hcan hcanx
 * @param DJI_Motor_Type 电机型号 DJI_Motor_6020
 * @param DJI_Motor_ID1 电机CAN-ID  6020：1~7
 * @param Out1 输出值1 6020电压控制：-25000~0~25000
 * @param DJI_Motor_ID2 电机CAN-ID  6020：1~7
 * @param Out2 输出值2 6020电压控制：-25000~0~25000
 */
void DJI_Motor_Control_Double(CAN_HandleTypeDef *hcan,DJI_Motor_Type_Typedef DJI_Motor_Type,uint8_t DJI_Motor_ID1,int16_t Out1,uint8_t DJI_Motor_ID2,int16_t Out2)
{
    uint8_t *TxData = NULL;

    uint16_t TxID = 0;

    if(Out1 >= DJI_Motor_6020_Out_Max)
    {
         Out1 = DJI_Motor_6020_Out_Max;
    }
    else if (Out1 <= DJI_Motor_6020_Out_Min)
    {
        Out1 = DJI_Motor_6020_Out_Min;
    }

    if(Out2 >= DJI_Motor_6020_Out_Max)
    {
         Out2 = DJI_Motor_6020_Out_Max;
    }
    else if (Out2 <= DJI_Motor_6020_Out_Min)
    {
        Out2 = DJI_Motor_6020_Out_Min;
    }

    //同一区间ID
    if(((DJI_Motor_ID1 >= 1 && DJI_Motor_ID1 <= 4) && (DJI_Motor_ID2 >= 1 && DJI_Motor_ID2 <= 4)) || ((DJI_Motor_ID1 >= 5 && DJI_Motor_ID1 <= 7) && (DJI_Motor_ID2 >= 5 && DJI_Motor_ID2 <= 7)))
    {
        //同1-4
        if((DJI_Motor_ID1 >= 1 && DJI_Motor_ID1 <= 4) && (DJI_Motor_ID2 >= 1 && DJI_Motor_ID2 <= 4))
        {
            TxID = 0x1FF;
            TxData = TxData_1FF;
        }
        else if((DJI_Motor_ID1 >= 5 && DJI_Motor_ID1 <= 7) && (DJI_Motor_ID2 >= 5 && DJI_Motor_ID2 <= 7))
        {
            TxID = 0x2FF;
            TxData = TxData_2FF;
            DJI_Motor_ID1 = DJI_Motor_ID1 - 4;
            DJI_Motor_ID2 = DJI_Motor_ID2 - 4;
        }

        if(TxID == 0)//防炸
        {
            return;
        }
        uint8_t DJI_Motor_ID_Temp = (uint8_t)(DJI_Motor_ID1 - 1);
        TxData[2*DJI_Motor_ID_Temp]   = (uint8_t)((uint16_t)Out1 >> 8);
        TxData[2*DJI_Motor_ID_Temp+1] = (uint8_t)((uint16_t)Out1);

        DJI_Motor_ID_Temp = (uint8_t)(DJI_Motor_ID2 - 1);
        TxData[2*DJI_Motor_ID_Temp]   = (uint8_t)((uint16_t)Out2 >> 8);
        TxData[2*DJI_Motor_ID_Temp+1] = (uint8_t)((uint16_t)Out2);

        CAN_Send_Data(hcan,TxID,TxData,8);
    }
    else//不同区间ID 分开发送
    {
        DJI_Motor_Control_Single(hcan,DJI_Motor_Type,DJI_Motor_ID1,Out1);
        DJI_Motor_Control_Single(hcan,DJI_Motor_Type,DJI_Motor_ID2,Out2);
    }
}

/**
 * @brief 大疆电机获得角度函数
 * 
 * @param DJI_Motor_ID 电机CAN-ID  6020：1~7   3508：1~8
 * @return float 角度（360度）单位：Deg
 */
float DJI_Motor_Get_Angle(uint8_t DJI_Motor_ID)
{
    return (float)DJI_Motors_Data_Temp[DJI_Motor_ID-1].RawAngle * 360.0 / 8192.0;
}

/**
 * @brief 大疆电机获得角速度函数
 * 
 * @param DJI_Motor_ID 电机CAN-ID  6020：1~7   3508：1~8
 * @return float 角速度 rad/s
 */
float DJI_Motor_Get_AngleSpeed(uint8_t DJI_Motor_ID)
{
    return (float)DJI_Motors_Data_Temp[DJI_Motor_ID-1].speed_rpm / 60.0 * 2.0 * (3.1415926);
}
