// Copyright 2022 PurplePopo <PurplePopo(at)outlook.com>
#ifndef APP_ANO_TC_PROTOCOLDEF_H_
#define APP_ANO_TC_PROTOCOLDEF_H_

/*
*    Protocol definitions
*   ________________________________________________________________________
*   |  Header | D_Addr | FuncID | dataLen | dataBytes | sumcheck | addCheck |
*   |_________|________|________|_________|___________|__________|__________|
*   |  1 byte | 1 byte | 1 byte | 1 byte  |  n byte   |  1bytes  | 1bytes   |
*   |_________|________|________|_________|___________|__________|__________|
*/

#define ANO_ADDR_BROAD          0xFF
#define ANO_ADDR_GENERAL        0xAF
#define ANO_ADDR_TUOKONGPRO     0x05
#define ANO_ADDR_SHUCHUAN       0x10
#define ANO_ADDR_GUANGLIU       0x22
#define ANO_ADDR_UWB            0x30
#define ANO_ADDR_LINXIAOIMU     0x60
#define ANO_ADDR_LINXIAO        0x61



#endif  // APP_ANO_TC_PROTOCOLDEF_H_
