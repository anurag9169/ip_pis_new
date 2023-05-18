/*
 * ndkConf.c
 *
 *  Created on: 15/02/2020
 *      Author: Sahil
 */

#include <string.h>
#include <stdlib.h>
/* XDCtools Header files */
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
/* TI-RTOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>


#include <ti/ndk/inc/netmain.h>
#include "struct_ethernet_rs.h"
//#include <ti/ndk/inc/nettools/netcfg.h>
//#include <ti/ndk/inc/netctrl/netctrl.h>
extern conf_t conf_data;

static void ti_ndk_config_Global_NetworkOpen();
static void ti_ndk_config_Global_NetworkClose();
static void ti_ndk_config_Global_NetworkIPAddr(IPN IPAddr, uint IfIdx, uint fAdd);
extern char *ti_ndk_config_Global_HostName ;
/* Static IP Address settings */


typedef enum state_e
{
    L_dynamic,
    L_static
}state_t;
 state_t state_1;
extern void llTimerTick();

static void ti_ndk_config_Global_NetworkOpen()
{
}

static void ti_ndk_config_Global_NetworkClose()
{
}
static void ti_ndk_config_Global_NetworkIPAddr(IPN IPAddr, uint IfIdx, uint fAdd)
{
    IPN IPTmp;

    if (fAdd) {
        xdc_runtime_System_printf("Network Added: ");
    }
    else {
        xdc_runtime_System_printf("Network Removed: ");
    }

    // Print a message
    IPTmp = ntohl(IPAddr);
    xdc_runtime_System_printf("If-%d:%d.%d.%d.%d\n", IfIdx,
            (UINT8)(IPTmp>>24)&0xFF, (UINT8)(IPTmp>>16)&0xFF,
            (UINT8)(IPTmp>>8)&0xFF, (UINT8)IPTmp&0xFF);

    {
        extern Void netIPAddrHook();

        /* call user defined network IP address hook */
        netIPAddrHook(IPAddr, IfIdx, fAdd);
    }
    xdc_runtime_System_flush();
}
extern int appl;
extern Void ti_ndk_config_external_dns_init(HANDLE hCfg);
extern Void ti_ndk_config_tcp_init(HANDLE hCfg);
extern Void ti_ndk_config_udp_init(HANDLE hCfg);
extern Void ti_ndk_config_Global_serviceReport(uint Item, uint Status,
        uint Report, HANDLE h);
static Void ti_ndk_config_ip_init(HANDLE hCfg)
{
    /* Add our global hostname to hCfg (to be claimed in all connected domains) */
    CfgAddEntry(hCfg, CFGTAG_SYSINFO, CFGITEM_DHCP_HOSTNAME, 0,
                 strlen(ti_ndk_config_Global_HostName),
                 (UINT8 *)ti_ndk_config_Global_HostName, 0);
    if(conf_data.ethernet_rs485.value==49)
    {
        state_1=L_static;
    }
    else
    {
        state_1=L_dynamic;
    }
    state_1=L_static;
    switch(state_1)
    {
    default:
    case L_dynamic:
    {
        CI_SERVICE_DHCPC dhcpc;
        UINT8 DHCP_OPTIONS[] =
                {
                DHCPOPT_SUBNET_MASK,
                };

        /* Specify DHCP Service on IF specified by "IfIdx" */
        bzero(&dhcpc, sizeof(dhcpc));
        dhcpc.cisargs.Mode   = 1;
        dhcpc.cisargs.IfIdx  = 1;
        dhcpc.cisargs.pCbSrv = &ti_ndk_config_Global_serviceReport;
        dhcpc.param.pOptions = DHCP_OPTIONS;
        dhcpc.param.len = 1;
        CfgAddEntry(hCfg, CFGTAG_SERVICE, CFGITEM_SERVICE_DHCPCLIENT, 0,
                sizeof(dhcpc), (UINT8 *)&dhcpc, 0);
    }
        break;
    case L_static:
    {
            CI_IPNET NA;
            CI_ROUTE RT;
            /* Setup manual IP address */
            bzero(&NA, sizeof(NA));

            NA.IPAddr  = inet_addr(conf_data.ethernet_rs485.ip_addr);
            NA.IPMask  = inet_addr(conf_data.ethernet_rs485.net_mask);
            strcpy(NA.Domain,conf_data.ethernet_rs485.domain);
            NA.NetType = 0;

            CfgAddEntry(hCfg, CFGTAG_IPNET, 1, 0,
                    sizeof(CI_IPNET), (UINT8 *)&NA, 0);

            /*
             *  Add the default gateway. Since it is the default, the
             *  destination address and mask are both zero (we go ahead
             *  and show the assignment for clarity).
             */
            bzero(&RT, sizeof(RT));
            RT.IPDestAddr = 0;
            RT.IPDestMask = 0;
            RT.IPGateAddr = inet_addr(conf_data.ethernet_rs485.gateway);

            CfgAddEntry(hCfg, CFGTAG_ROUTE, 0, 0,
                    sizeof(CI_ROUTE), (UINT8 *)&RT, 0);


    }
        break;
    }
    /* Configure IP address manually on interface 1 */
    {

    }
}


Void userNdkStackThread(UArg arg0, UArg arg1)
{
    int rc;
    HANDLE hCfg;

    ti_sysbios_knl_Clock_Params clockParams;

    /* Create the NDK heart beat */
    ti_sysbios_knl_Clock_Params_init(&clockParams);
    clockParams.startFlag = TRUE;
    clockParams.period = 100;
    ti_sysbios_knl_Clock_create(&llTimerTick, clockParams.period, &clockParams, NULL);


    /* THIS MUST BE THE ABSOLUTE FIRST THING DONE IN AN APPLICATION!! */
    rc = NC_SystemOpen(NC_PRIORITY_LOW, NC_OPMODE_INTERRUPT);
    if (rc) {
        xdc_runtime_System_abort("NC_SystemOpen Failed (%d)\n");
    }

    /* Create and build the system configuration from scratch. */
    hCfg = CfgNew();
    if (!hCfg) {
        xdc_runtime_System_printf("Unable to create configuration\n");
        goto main_exit;
    }

    /* add the Ip module configuration settings. */
    ti_ndk_config_ip_init(hCfg);

    /* add the Tcp module configuration settings. */
    ti_ndk_config_tcp_init(hCfg);

    /* add the Udp module configuration settings. */
    ti_ndk_config_udp_init(hCfg);

    /* add the external DNS server to the configuration. */
    //ti_ndk_config_external_dns_init(hCfg);

    /* add the configuration settings for NDK low priority tasks stack size. */
    rc = 1280;
    CfgAddEntry(hCfg, CFGTAG_OS, CFGITEM_OS_TASKSTKLOW,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );

    /* add the configuration settings for NDK norm priority tasks stack size. */
    rc = 1024;
    CfgAddEntry(hCfg, CFGTAG_OS, CFGITEM_OS_TASKSTKNORM,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );

    /* add the configuration settings for NDK high priority tasks stack size. */
    rc = 1024;
    CfgAddEntry(hCfg, CFGTAG_OS, CFGITEM_OS_TASKSTKHIGH,
                 CFG_ADDMODE_UNIQUE, sizeof(uint), (UINT8 *)&rc, 0 );

    /*
     *  Boot the system using this configuration
     *
     *  We keep booting until the function returns 0. This allows
     *  us to have a "reboot" command.
    */
    do
    {
        rc = NC_NetStart(hCfg, ti_ndk_config_Global_NetworkOpen,
                         ti_ndk_config_Global_NetworkClose,
                         ti_ndk_config_Global_NetworkIPAddr);

    } while( rc > 0 );

    /* Delete Configuration */
    CfgFree(hCfg);

    /* Close the OS */
main_exit:
    NC_SystemClose();
    xdc_runtime_System_flush();


}
