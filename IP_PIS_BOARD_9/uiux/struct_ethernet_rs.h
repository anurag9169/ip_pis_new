/*
 * struct_ethernet_rs.h
 *
 *  Created on: 28-Mar-2022
 *      Author: csemi-win1
 */

#ifndef UIUX_STRUCT_ETHERNET_RS_H_
#define UIUX_STRUCT_ETHERNET_RS_H_

typedef struct config_s
{
    uint32_t header;
    char sno[12];
    char mfr[20];
    char mfr_date[20];
    char hw_v[20];
    char sw_v[20];
    char conf_date[20];
    char prj[20];

}config_t;
typedef struct data_s
{
    char def_msg[100];
    char def_err[100];
    uint32_t def_spd;
    uint32_t def_brgt;
    uint32_t def_wt;
    char def_buff[2048];

}data_t;

typedef struct ethernet_rs485_s
{

    char header[10];
    char ip_addr[20];
    char net_mask[20];
    char gateway[20];
    char dns[20];
    char domain[20];
    char value;

}ethernet_rs485_t;

typedef struct conf_s{

    config_t config;
    data_t data;
    ethernet_rs485_t ethernet_rs485;
    int brt_lo;
    int brt_mid;

}conf_t;
typedef enum conf_stages_e{

    CONF_IDLE,
    CONF_SYNC,
    CONF_SEND,
    CONF_RECV

}conf_stages_t;


typedef enum mode_s{

    OPS_MODE,
    CONFIG_MODE

}mode_t;




#endif /* UIUX_STRUCT_ETHERNET_RS_H_ */
