// Copyright 2022 PurplePopo <PurplePopo(at)outlook.com>
#ifndef APP_ESP01S_AT_CMDTABLE_H_
#define APP_ESP01S_AT_CMDTABLE_H_
// AI-Thinker ESP01-S AT Cmd Table

#define AT_RESPONE_OK                       "OK"
#define AT_RECV_DATA                        "+IPD,"
// Test
#define ATCMD_AT                            "AT"                                        // Test                 --NOLINT
#define ATCMD_RST                           "AT+RST"                                    // Reset                --NOLINT
#define ATCMD_GMR                           "AT+GMR"                                    // Version info         --NOLINT
// WIfi
#define ATCMD_SET_CWMODE_AP                 "AT+CWMODE=2"                               // Setup AP mode        --NOLINT
#define ATCMD_SET_CWSAP(ssid, pwd)          "AT+CWSAP=\"##ssid\",\"##pwd\",5,3,1,0"     // Set AP param         --NOLINT
#define ATCMD_CHK_CWLIF                     "AT+CWLIF"                                  // Check connection     --NOLINT
#define ATCMD_SET_CWQIF                     "AT+CWQIF"                                  // Close all connection --NOLINT
#define ATCMD_SET_CWDHCP                    "AT+CWDHCP=1,2"                             // enable DHCP          --NOLINT
#define ATCMD_CHK_CIPAP                     "AT+CIPAP?"                                 // check ipconfig       --NOLINT
#define ATCMD_SET_MDNS(host, server, port)  "AT+MDNS=1,\"##host\",\"##server\",##port"  // set mdns server      --NOLINT
// UDP
#define ATCMD_CHK_CIPSTATE                  "AT+CIPSTATE?"                              // check udp link state --NOLINT
#define ATCMD_CHK_CIFSR                     "AT+CIFSR"                                  // check local ip/mac   --NOLINT
#define ATCMD_SET_CIPSEND(len)              "AT+CIPSEND=##len"                          // send data            --NOLINT
#define ATCMD_SET_CIPCLOSE                  "AT+CIPCLOSE"                               // close connection     --NOLINT

#endif  // APP_ESP01S_AT_CMDTABLE_H_
