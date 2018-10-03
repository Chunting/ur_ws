#ifndef HANDCONTROL_H
#define HANDCONTROL_H

#pragma pack(1)
#pragma pack(push)
#pragma pack(1)

typedef unsigned long       DWORD;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;

class HandControl
{
public:
    HandControl();
};

typedef struct THREEBYTE
{
    BYTE value[3];
}THREEBYTE;

typedef struct MasterSlaveControl
{
    BYTE MasterSlaveControl_BJ;
    WORD MasterSlaveControl_ZH;
    BYTE MasterSlaveControl_YK;
    WORD MasterSlaveControl_S1;
    WORD MasterSlaveControl_S2;
    WORD MasterSlaveControl_S3;
    WORD MasterSlaveControl_S4;
    WORD MasterSlaveControl_S5;
    WORD MasterSlaveControl_S6;
    WORD MasterSlaveControl_S7;
    WORD MasterSlaveControl_S8;
    WORD MasterSlaveControl_S9;
    WORD MasterSlaveControl_S10;
    BYTE MasterSlaveControl_SC_BJ;
    WORD MasterSlaveControl_SC_YKZ;
    BYTE MasterSlaveControl_SC_LL;
    DWORD MasterSlaveControl_SC_TF;
    DWORD MasterSlaveControl_SC_TS;
    DWORD MasterSlaveControl_SC_TZ;
    DWORD MasterSlaveControl_SC_TZS;
    BYTE MasterSlaveControl_SC_TI;
    THREEBYTE MasterSlaveControl_SC_R;
}MasterSlaveControl;

#pragma pack(pop)

#endif // HANDCONTROL_H
