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
#include <cstring>
#include <stdint.h>
#define motor_gear_ratio_inv 0.063432835820f //减速比倒数 268：17

Class_DJI_Motor_Group *Class_DJI_Motor_Group::Group_FIFO0 = nullptr;

/**
 * @brief 大疆电机组初始化
 * 
 * @param hcan hcanx
 * @param Motor_Type 电机型号 DJI_Motor_6020 / DJI_Motor_3508
 */
void Class_DJI_Motor_Group::Init(CAN_HandleTypeDef *hcan, DJI_Motor_Type_Typedef Motor_Type)
{
    this->hcan = hcan;
    this->Type = Motor_Type;

    memset(Motor_List, 0, sizeof(Motor_List));
    memset(TxData_Low, 0, sizeof(TxData_Low));
    memset(TxData_High, 0, sizeof(TxData_High));

    Group_FIFO0 = this;

    uint8_t CAN_FIlter_BANK = (hcan->Instance == CAN1) ? 0 : 14;

    CAN_Register_RxCallBack_FIFO0_Function(CAN_RxCallback_Entry);
    CAN_Filter_Mask_Config(hcan, CAN_FILTER(CAN_FIlter_BANK) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x200, 0x7E0);
    CAN_Init(hcan);
}

/**
 * @brief 大疆电机初始化函数
 * 
 * @param Motor_Type 电机型号 DJI_Motor_6020 / DJI_Motor_3508
 * @param Motor_ID 电机ID
 * @param group 电机组
 */
void Class_DJI_Motor::Init(DJI_Motor_Type_Typedef Motor_Type, uint8_t Motor_ID, Class_DJI_Motor_Group *group)
{
    Type = Motor_Type;
    ID = Motor_ID;
    Group = group;

    RawAngle = 0;
    Speed_Rpm = 0;
    Torque_Current = 0;
    Temperature = 0;
    Out = 0;

    if (Group != nullptr)
    {
        Group->Register_Motor(this);
    }
}

/**
 * @brief 大疆电机获取电机对应CAN-ID的起始ID
 * 
 * @return uint16_t 0x201--3508 0x205--6020
 */
uint16_t Class_DJI_Motor_Group::Get_Rx_Start_ID(void) const
{
    return (Type == DJI_Motor_3508) ? 0x201 : 0x205;
}

/**
 * @brief 大疆电机获取低位ID发送数据CAN标识符
 * 
 * @return uint16_t 0x200--3508 0x1FF--6020
 */
uint16_t Class_DJI_Motor_Group::Get_Tx_Low_ID(void) const
{
    return (Type == DJI_Motor_3508) ? 0x200 : 0x1FF;
}

/**
 * @brief 大疆电机获取高位ID发送数据CAN标识符
 * 
 * @return uint16_t 0x200--3508 0x1FF--6020
 */
uint16_t Class_DJI_Motor_Group::Get_Tx_High_ID(void) const
{
    return (Type == DJI_Motor_3508) ? 0x1FF : 0x2FF;
}

/**
 * @brief 大疆电机接收回调进入函数
 * 
 * @param RxBuffer 
 */
void Class_DJI_Motor_Group::CAN_RxCallback_Entry(CAN_Rx_Buffer_t *RxBuffer)
{
    if (Group_FIFO0 == nullptr)
    {
        return;
    }


    Group_FIFO0->CAN_RxCallback(RxBuffer);
}

/**
 * @brief 大疆电机回调函数
 * 
 * @param RxBuffer 
 */
void Class_DJI_Motor_Group::CAN_RxCallback(CAN_Rx_Buffer_t *RxBuffer)
{
    uint16_t id = RxBuffer->Header.StdId;
    uint16_t start_id = Get_Rx_Start_ID();

    if (id < start_id || id >= start_id + 8)
    {
        return;
    }

    uint8_t index = (uint8_t)(id - start_id);

    if (Motor_List[index] != nullptr)
    {
        Motor_List[index]->FeedBack_Data(RxBuffer);
    }
}

/**
 * @brief 大疆电机注册电机函数
 * 
 * @param motor Class_DJI_Motor类地址
 */
void Class_DJI_Motor_Group::Register_Motor(Class_DJI_Motor *motor)
{
    if (motor == nullptr)
    {
        return;
    }

    if (motor->Type != Type)
    {
        return;
    }

    uint8_t max_id = (Type == DJI_Motor_3508) ? 8 : 7;

    if (motor->ID == 0 || motor->ID > max_id)
    {
        return;
    }

    Motor_List[motor->ID - 1] = motor;
    motor->Group = this;
}

/**
 * @brief 大疆电机更新反馈数据
 * 
 * @param RxBuffer 
 */
void Class_DJI_Motor::FeedBack_Data(const CAN_Rx_Buffer_t *RxBuffer)
{
    RawAngle = (RxBuffer->Data[0] << 8) | RxBuffer->Data[1];
    Speed_Rpm = (int16_t)(((uint16_t)RxBuffer->Data[2] << 8) | RxBuffer->Data[3]);
    Torque_Current = (int16_t)(((uint16_t)RxBuffer->Data[4] << 8) | RxBuffer->Data[5]);
    Temperature = RxBuffer->Data[6];
}

/**
 * @brief 大疆系电机限幅函数
 * 
 * @param out 
 * @return int16_t 
 */
int16_t Class_DJI_Motor::Limit_Out(int16_t out) const
{
    if (Type == DJI_Motor_6020)
    {
        if (out > 25000)
        {
            return 25000;
        }
        if (out < -25000)
        {
            return -25000;
        }
    }
    else if (Type == DJI_Motor_3508)
    {
        if (out > 16384)
        {
            return 16384;
        }
        if (out < -16384)
        {
            return -16384;
        }
    }

    return out;
}

/**
 * @brief 大疆电机设置输出值
 * 
 * @param out 
 */
void Class_DJI_Motor::Set_Out(int16_t out)
{
    Out = Limit_Out(out);
}

float Class_DJI_Motor::Get_AngleSpeed(void)
{
    if(this->Type == DJI_Motor_3508)
    {
        return Speed_Rpm * 0.006642671f; // 1/60.0 * 2.0 * (3.1415926) * motor_gear_ratio_inv
    }
    return Speed_Rpm * 0.1047197533333f; // 1/60.0 * 2.0 * (3.1415926)
}

/**
 * @brief 大疆电机上传数据给电机进行控制
 * 
 */
void Class_DJI_Motor_Group::Push_Data(void)
{
    memset(TxData_Low,  0, 8);
    memset(TxData_High, 0, 8);

    uint8_t need_send_low = 0;
    uint8_t need_send_high = 0;

    uint8_t max_id = (Type == DJI_Motor_3508) ? 8 : 7;

    for (uint8_t i = 0; i < max_id; i++)
    {
        if (Motor_List[i] == nullptr)
        {
            continue;
        }

        uint8_t ID = i + 1;
        int16_t out = Motor_List[i]->Get_Out();

        uint8_t *TxData = nullptr;
        uint8_t Temp_ID = 0;

        if (ID <= 4)
        {
            TxData = TxData_Low;
            Temp_ID = ID - 1;
            need_send_low = 1;
        }
        else
        {
            TxData = TxData_High;
            Temp_ID = ID - 5;
            need_send_high = 1;
        }

        TxData[2 * Temp_ID]     = (uint8_t)((uint16_t)out >> 8);
        TxData[2 * Temp_ID + 1] = (uint8_t)((uint16_t)out);
    }

    if (need_send_low)
    {
        CAN_Send_Data(hcan, Get_Tx_Low_ID(), TxData_Low, 8);
    }

    if (need_send_high)
    {
        CAN_Send_Data(hcan, Get_Tx_High_ID(), TxData_High, 8);
    }
}


//----------------------------------------------------------
//↓旧版C语言大疆电机库 已弃用 先留存
//----------------------------------------------------------

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

static DJI_Motor_Data_t *DJI_Motors_Data_Global = NULL;

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
    if (!DJI_Motors_Data_Global)//防炸
    {
        return;
    }

    uint16_t ID = RxBuffer->Header.StdId;

    if (ID < Motor_ID_Start || ID >= Motor_ID_Start + 8)//防炸
    {
       return;
    }

    DJI_Motor_Get_Data(RxBuffer, &DJI_Motors_Data_Global[ID - Motor_ID_Start]); 
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
    DJI_Motors_Data_Global = DJI_Motors_Data;
    CAN_Register_RxCallBack_FIFO0_Function(DJI_Motor_RxCallBack);
    DJI_Motor_CAN_Init(hcan,CAN_FILTER(0) | CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE,0x200, 0x7E0);
}

/**
 * @brief 大疆单电机控制数据推送函数
 * 
 * @param hcan 
 * @param DJI_Motor_Type 
 * @param DJI_Motor_ID 
 * @param Out 
 */
void DJI_Motor_Control_Single_Push_Data(DJI_Motor_Type_Typedef DJI_Motor_Type,uint16_t DJI_Motor_ID,int16_t Out)
{
    uint8_t *TxData = NULL;

    if(DJI_Motor_Type == DJI_Motor_6020)
    {
        if(DJI_Motor_ID > 0 && DJI_Motor_ID <= 4)
        {
            TxData = TxData_1FF;
        }
        else if(DJI_Motor_ID > 4 && DJI_Motor_ID <= 7)
        {
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
            TxData = TxData_200;
        }
        else if(DJI_Motor_ID > 4 && DJI_Motor_ID <= 8)
        {
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

    if(TxData == NULL)
    {
        return;
    }

    uint8_t DJI_Motor_ID_Temp = (uint8_t)(DJI_Motor_ID - 1);
    TxData[2*DJI_Motor_ID_Temp]   = (uint8_t)((uint16_t)Out >> 8);
    TxData[2*DJI_Motor_ID_Temp+1] = (uint8_t)((uint16_t)Out);
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
    uint16_t TxID = 0;
    uint8_t *TxData = NULL;

    DJI_Motor_Control_Single_Push_Data(DJI_Motor_Type,DJI_Motor_ID,Out);

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
        }
    }
    if(TxData == NULL)
    {
        return;
    }

    if(TxID == 0)//防炸
    {
        return;
    }

    //2 数据处理 
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
    if(DJI_Motor_Type != DJI_Motor_6020) return;

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
 * @brief 大疆四电机控制函数（针对3508电机）
 * 
 * @param hcan 
 * @param DJI_Motor_Type 
 * @param DJI_Motor_ID1 
 * @param Out1 
 * @param DJI_Motor_ID2 
 * @param Out2 
 * @param DJI_Motor_ID3 
 * @param Out3 
 * @param DJI_Motor_ID4 
 * @param Out4 
 */
void DJI_Motor_Control_Four(CAN_HandleTypeDef *hcan,DJI_Motor_Type_Typedef DJI_Motor_Type,uint8_t DJI_Motor_ID1,int16_t Out1,uint8_t DJI_Motor_ID2,int16_t Out2
                                                                                         ,uint8_t DJI_Motor_ID3,int16_t Out3,uint8_t DJI_Motor_ID4,int16_t Out4)
{
    if(DJI_Motor_Type != DJI_Motor_3508) return;

    uint8_t need_send_200 = 0;
    uint8_t need_send_1FF = 0;

    if((DJI_Motor_ID1 == 0 || DJI_Motor_ID1 > 8) ||
        (DJI_Motor_ID2 == 0 || DJI_Motor_ID2 > 8) ||
        (DJI_Motor_ID3 == 0 || DJI_Motor_ID3 > 8) ||
        (DJI_Motor_ID4 == 0 || DJI_Motor_ID4 > 8))
    {
        return;
    }

    need_send_200 = (DJI_Motor_ID1 <= 4) || (DJI_Motor_ID2 <= 4) || (DJI_Motor_ID3 <= 4) || (DJI_Motor_ID4 <= 4);
    need_send_1FF = (DJI_Motor_ID1 > 4) || (DJI_Motor_ID2 > 4) || (DJI_Motor_ID3 > 4) || (DJI_Motor_ID4 > 4);

    DJI_Motor_Control_Single_Push_Data(DJI_Motor_Type,DJI_Motor_ID1,Out1);
    DJI_Motor_Control_Single_Push_Data(DJI_Motor_Type,DJI_Motor_ID2,Out2);
    DJI_Motor_Control_Single_Push_Data(DJI_Motor_Type,DJI_Motor_ID3,Out3);
    DJI_Motor_Control_Single_Push_Data(DJI_Motor_Type,DJI_Motor_ID4,Out4);

    if(need_send_200)
    {
        CAN_Send_Data(hcan,0x200,TxData_200,8);
    }

    if(need_send_1FF)
    {
        CAN_Send_Data(hcan,0x1FF,TxData_1FF,8);
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
    return (float)DJI_Motors_Data_Global[DJI_Motor_ID-1].RawAngle * 360.0 / 8192.0;
}

/**
 * @brief 大疆电机获得角速度函数
 * 
 * @param DJI_Motor_ID 电机CAN-ID  6020：1~7   3508：1~8
 * @return float 角速度 rad/s
 */
float DJI_Motor_Get_AngleSpeed(uint8_t DJI_Motor_ID)
{
    return (float)DJI_Motors_Data_Global[DJI_Motor_ID-1].speed_rpm / 60.0 * 2.0 * (3.1415926);
}
