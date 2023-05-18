/*
 * Copyright (c) 2014-2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== tcpEcho.c ========
 *    Contains BSD sockets code.
 */

#include <string.h>
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <stdlib.h>
/* NDK BSD support */
#include <sys/socket.h>

/* Example/Board Header file */
#include "Board.h"

#include <stdio.h>



#include <ti/sysbios/knl/Clock.h>


#include "driverlib/debug.h"
#include "driverlib/flash.h"

#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "driverlib/uart.h"
#include "utils/lwiplib.h"
#include "utils/swupdate.h"
#include "utils/ustdlib.h"
#include "driverlib/adc.h"

#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "inc/hw_gpio.h"
#include "utils/lwiplib.h"
#include "utils/swupdate.h"
#include "utils/ustdlib.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
/* custom headers */
#include "font5x7.h"
#include "font8x12.h"
#include "font9x16.h"
#include "font11x16.h"
#include "struct_ethernet_rs.h"
#include "symbol16x16_font.h"






#define TCPPORT 1000
#define TCPHANDLERSTACK 1024
#define TCPPACKETSIZE 2048
#define NUMTCPWORKERS 3
#define TASKSTACKSIZE 1536
Task_Struct task0Struct,task1Struct,task2Struct,task3Struct,task4Struct,task6Struct;
Char task0Stack[TASKSTACKSIZE],task1Stack[TASKSTACKSIZE],task2Stack[TASKSTACKSIZE],task3Stack[TASKSTACKSIZE],task4Stack[TASKSTACKSIZE],task6Stack[TASKSTACKSIZE];
UART_Handle uart4;
static void uart4_init();

SPI_Handle rowSpi, colSpi;
SPI_Params rowSpiParams;
SPI_Params colSpiParams;
SPI_Transaction rowTransaction, colTransaction;
bool transferOK;
mode_t  mode;
conf_t conf_data;
conf_stages_t conf_stage;
#define PIX_WIDTH   432
#define BYTES_RXD   1024

#define HCD_COLS    1088

unsigned char addr = 31;
int static_blocks=0;

unsigned char sspeed=2;
unsigned char bContrl=0;
typedef enum brght_mode_t
{
    BRGHT_LOW,
    BRGHT_MID,
    BRGHT_HI


}brght_mode;
static void clear_screen(int frames);
//static void default_fb ();
static void adc_init();
static void set_brightness(brght_mode mode);
static void auto_brightness();
void row_latch()
{
    GPIO_write(EK_TM4C1294XL_ROW_LE, EK_TM4C1294XL_HIGH);
    GPIO_write(EK_TM4C1294XL_ROW_LE, EK_TM4C1294XL_LOW);
}

void col_latch()
{
    GPIO_write(EK_TM4C1294XL_COL_LE, EK_TM4C1294XL_HIGH);
    GPIO_write(EK_TM4C1294XL_COL_LE, EK_TM4C1294XL_LOW);
}

void col_output_enable()
{
    GPIO_write(EK_TM4C1294XL_COL_OE, EK_TM4C1294XL_LOW);
}

void col_output_disable()
{
    GPIO_write(EK_TM4C1294XL_COL_OE, EK_TM4C1294XL_HIGH);
}

typedef struct packet_s
{
    char dddbuffer[1024];
    int scroll_enable;
    int scroll_speed;
    int brightness;
    int flash_on;
}packet_t;
packet_t packet;
char dbuffer[2048]={0};
char dbuffer_train[5];
char dddbuffer[2048]={0};
char dddbuffer1[4732]={0};
char dbuffer_new[2048]={0};
int ddd_idx=0;
char *revbuffer;
char rev_buffer[TCPPACKETSIZE]={0};
unsigned char dataReceived=0;
Void farmware_update(UArg arg0, UArg arg1);
unsigned short rowTxBuffer[1]={0x0000};
unsigned char rowRxBuffer[2];

unsigned short colTxBuffer[32]={0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};//first byte is set in 4th nibble sector of display
unsigned char colRxBuffer[64];
uint8_t pui8IP[14];
uint32_t IP=0,IP1;
uint32_t NM=0,NM1;
Clock_Struct clk0Struct, clk1Struct;
Clock_Handle clkHandle;
void matrix_spi_init()
{
    /* Initialize SPI handle with slave mode */
    SPI_Params_init(&rowSpiParams);
    rowSpiParams.bitRate = 20000;
    rowSpiParams.dataSize = 16;
    rowSpiParams.frameFormat = SPI_POL0_PHA0;

    SPI_Params_init(&colSpiParams);
    colSpiParams.bitRate =5000000;
    colSpiParams.dataSize = 16;
    colSpiParams.frameFormat = SPI_POL0_PHA0;

    /* Initialize SPI handle as default master */
    rowSpi = SPI_open(Board_SPI0, &rowSpiParams);
    if (rowSpi == NULL) {
        System_abort("Error initializing Row SPI\n");
    }
    else {
        System_printf("Row SPI initialized\n");
    }

    /* Initialize master SPI transaction structure */
    rowTransaction.count = 1;
    rowTransaction.txBuf = (Ptr)rowTxBuffer;
    rowTransaction.rxBuf = (Ptr)rowRxBuffer;

    /* Initialize SPI handle as default master */
    colSpi = SPI_open(Board_SPI1, &colSpiParams);
    if (colSpi == NULL) {
        System_abort("Error initializing Col SPI\n");
    }
    else {
        System_printf("Col SPI initialized\n");
    }

    /* Initialize master SPI transaction structure */
    colTransaction.count = PIX_WIDTH/16;
    colTransaction.txBuf = (Ptr)colTxBuffer;
    colTransaction.rxBuf = (Ptr)colRxBuffer;

}

void buf_left_bitshift(uint16_t rbuf[], int len )
{
    uint16_t cc = (rbuf[0] & 0x8000) >>15;
    int idx;
    for(idx=0;idx<len-1;idx++)
    {
        rbuf[idx] = rbuf[idx] << 1;
        rbuf[idx] |= (rbuf[idx+1] & 0x8000) >>15;
    }

    rbuf[idx] = rbuf[idx] << 1;
    rbuf[len-1] += cc;
}
void buf_left_bitshift2(uint16_t rbuf[], int len )
{
    uint16_t cc = (rbuf[0] & 0x8000) >>15;
    int idx;
    for(idx=0;idx<len-1;idx++)
    {
        rbuf[idx] = rbuf[idx] << 1;
        rbuf[idx] |= (rbuf[idx+1] & 0x8000) >>15;
    }

    rbuf[idx] = rbuf[idx] << 1;
    rbuf[len-1] += cc;
}
void buf_left_bitshift1(uint16_t rbuf[], int len )
{
    uint16_t cc = (rbuf[0] & 0x8000) >>15;
    int idx;
    for(idx=8;idx<len-1;idx++)
    {
        rbuf[idx] = rbuf[idx] << 1;
        rbuf[idx] |= (rbuf[idx+1] & 0x8000) >>15;
    }
    rbuf[idx] = rbuf[idx] << 1;
    rbuf[len-1] += cc;
}


void buf_right_bitshift(uint16_t rbuf[], int len )
{
    uint16_t cc = (rbuf[len-1] & 0x01)<<15;
    int idx;
    for(idx=len-1;idx>0;idx--)
    {
        rbuf[idx] >>=1;
        rbuf[idx] |= (rbuf[idx-1])<<15;
    }

    rbuf[0] >>=1;
    rbuf[0] += cc;
}
static volatile bool g_bFirmwareUpdate = false;
#define TICKS_PER_SECOND 100
uint32_t g_ui32SysClock;
#define SIZE_MAC_ADDR_BUFFER 32
#define SIZE_IP_ADDR_BUFFER 32
uint32_t g_ui32SysClock;
char g_pcMACAddr[SIZE_MAC_ADDR_BUFFER];
char g_pcIPAddr[SIZE_IP_ADDR_BUFFER];
#ifdef normal
int bufIdx=0;
#endif
int bufIdx=0;
unsigned short jmp = BYTES_RXD/16;
unsigned int ctr = 1;
unsigned char frames=0;
int i,up=0,down=0,right=0,left=0,change=0;
//char buff[TCPPACKETSIZE]={0};
char buffer[TCPPACKETSIZE]={0};
char binary[512];
char rev_binary[512];
unsigned char reverseBits(unsigned char num)
{
    unsigned int  NO_OF_BITS = sizeof(num) * 8;
    unsigned int  i, temp;
    uint8_t reverse_num = 0;

    for (i = 0; i < NO_OF_BITS; i++)
    {
        temp = (num & (1 << i));
        if(temp)
            reverse_num |= (1 << ((NO_OF_BITS - 1) - i));
    }

    return reverse_num;
}
char empty_packet[2048]={0};
void dbuffer_display()
{
    int i,j;
    for(i=0;i<16;i++)
    {
        for(j=0;j<64;j++)
        {
            dbuffer[i*64+j]=packet.dddbuffer[i*64+j];
        }
    }
}
int flash_time=0;
// single line ddb
Void matrixSLScrollTaskFxn (UArg arg0, UArg arg1)
{
    matrix_spi_init();
    while(1)
    {
        rowTxBuffer[0] = ~ctr;        //works for rtv ddb sl
        ctr*=2;
        if(packet.scroll_enable==0)
        {
            if(packet.flash_on==1)
            {

                if(flash_time>=1&&flash_time<1000)
                {
                    colTransaction.txBuf = (Ptr)(dbuffer+/*BYTES_RXD*/ HCD_COLS-(bufIdx*jmp));
                }
                else
                if(flash_time>=1000&&flash_time<2000)
                {
                    colTransaction.txBuf = (Ptr)(dbuffer+/*BYTES_RXD*/ HCD_COLS-(bufIdx*jmp));
                }
                else
                if(flash_time==2000)
                {
                    flash_time=0;
                }
                flash_time++;
            }
            else
            {
                colTransaction.txBuf = (Ptr)(dbuffer+/*BYTES_RXD*/ HCD_COLS-(bufIdx*jmp));
            }
        }
        else
        {
            colTransaction.txBuf = (Ptr)(dbuffer+/*BYTES_RXD*/ HCD_COLS-(bufIdx*jmp));
        }
        transferOK = SPI_transfer(rowSpi, &rowTransaction);
        transferOK = SPI_transfer(colSpi, &colTransaction);
        row_latch();
        col_latch();
        if(ctr==65536)
        {
            ctr=1;
        }
        bufIdx++;
        if(bufIdx==16)
        {
            bufIdx=0;
            if(packet.scroll_enable==1)
            {
                if(packet.scroll_speed==2)
                {
                    sspeed=1;
                }
                else
                if(packet.scroll_speed==1)
                {
                    sspeed=2;
                }
                else
                {
                    sspeed=packet.scroll_speed;
                }

                frames++;
                if(frames==sspeed)
                {
                    frames = 0;
                   for(i=1;i<=16;i++)
                   {
                       buf_left_bitshift2((uint16_t*)dbuffer+ i*32 + static_blocks, 32-static_blocks);
                   }
                }
            }
        }
    }
}

void default_fb (char*string)
{
    int pos=0,k,p;
    int rcnt=0, ccnt=0;
    int cIdx=0, cidx=0;
    unsigned short bit=0;
    //char string[] ={'W','E','L','C',0x7F,'M',0x7F,0x80,0x81}; //{0x57};//"WELCOME TO INDIAN RAILWAYS";
    unsigned char strIdx=0;
    unsigned char strLen = strlen(string);
    unsigned char skip;
    unsigned char f_w=9;
    int idx;
    for(rcnt=0;rcnt<16;rcnt++)
    {
        skip=0;
        for(cIdx=0;cIdx<32;cIdx++)
        {
            for(cidx=15;cidx>=0;cidx--)
            {
                if(ccnt<f_w)
                {
                    bit = Font9x16[ccnt+++(string[strIdx]-0x20)*9]>>(rcnt-2)&0x01;
                    bit = bit<<cidx;
                    colTxBuffer[cIdx]=colTxBuffer[cIdx]|bit;
                }
                else
                {
                    ccnt=0;
                    strIdx++;
                    if(strLen==strIdx)
                    {
                        strIdx=0;
                        skip=1;
                        break;
                    }
                }
            }
            if(skip)
            {
                break;
            }

        }
        for(idx=0;idx<64;idx++)
        {
            dbuffer[rcnt*64+idx]=((unsigned char*)colTxBuffer)[idx];
            ((unsigned char*)colTxBuffer)[idx]=0;
        }
    }

}


int                clientfd;
int rxd=0;
int write_mode=0;
uint8_t buffer12[200];
int appl=0;
Void tcpWorker(UArg arg0, UArg arg1)
{
    int  bytesRcvd;
   int  clientfd = (int)arg0;
   int cidx;
   Task_Params taskParams3;
   System_printf("tcpWorker: start clientfd = 0x%x\n", clientfd);

   while ((bytesRcvd = recv(clientfd, buffer, TCPPACKETSIZE, 0)) > 0) {
       {

           rxd+=bytesRcvd;

           // copy it
            for(cidx=0;cidx<bytesRcvd;cidx++)
            {
                dddbuffer1[ddd_idx++] = buffer[cidx];
            }

            if(rxd == sizeof(packet))
            {
               memcpy(&packet,dddbuffer1,sizeof(packet));
               ddd_idx=0;
               rxd=0;
               bytesRcvd=0;
               dbuffer_display();

            }
            else
            if(rxd==3)
            {
               if(!strncmp("#F*",dddbuffer1,3))
               {
                   rxd=0;
                   ddd_idx=0;
                   Task_Params_init(&taskParams3);
                   taskParams3.priority = 4;
                   taskParams3.stackSize = TASKSTACKSIZE;
                   taskParams3.stack = &task3Stack;
                   Task_construct(&task3Struct, (Task_FuncPtr)farmware_update, &taskParams3, NULL);


               }
            }
//            else
//            {
//                rxd=0;
//                ddd_idx=0;
//            }

           System_printf("\n rxd = %d\n", rxd);
//           //we received data


       }
   }

   System_printf("tcpWorker stop clientfd = 0x%x\n", clientfd);
   close(clientfd);
}

Void tcpHandler(UArg arg0, UArg arg1)
{
    int                status;

    int                server;
    struct sockaddr_in localAddr;
    struct sockaddr_in clientAddr;
    int                optval;
    int                optlen = sizeof(optval);
    socklen_t          addrlen = sizeof(clientAddr);
    Task_Handle        taskHandle;
    Task_Params        taskParams;
    Error_Block        eb;

    server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server == -1) {
        System_printf("Error: socket not created.\n");
        goto shutdown;
    }


    memset(&localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(arg0);

    status = bind(server, (struct sockaddr *)&localAddr, sizeof(localAddr));
    if (status == -1) {
        System_printf("Error: bind failed.\n");
        goto shutdown;
    }

    status = listen(server, NUMTCPWORKERS);
    if (status == -1) {
        System_printf("Error: listen failed.\n");
        goto shutdown;
    }

    optval = 1;
    if (setsockopt(server, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
        System_printf("Error: setsockopt failed\n");
        goto shutdown;
    }

    while ((clientfd =
            accept(server, (struct sockaddr *)&clientAddr, &addrlen)) != -1) {

        System_printf("tcpHandler: Creating thread clientfd = %d\n", clientfd);

        /* Init the Error_Block */
        Error_init(&eb);

        /* Initialize the defaults and set the parameters. */
        Task_Params_init(&taskParams);
        taskParams.arg0 = (UArg)clientfd;
        taskParams.stackSize = 1280;
        taskHandle = Task_create((Task_FuncPtr)tcpWorker, &taskParams, &eb);
        if (taskHandle == NULL) {
            System_printf("Error: Failed to create new Task\n");
            close(clientfd);
        }

        /* addrlen is a value-result param, must reset for next accept call */
        addrlen = sizeof(clientAddr);
    }

    System_printf("Error: accept failed.\n");

shutdown:
    if (server > 0) {
        close(server);
    }
}
// single line ddb

void
SysTickIntHandler(void)
{

    lwIPTimer(1000 / TICKS_PER_SECOND);
}
void
SoftwareUpdateRequestCallback(void)
{
    g_bFirmwareUpdate = true;
}
void default_config()
{
    if(conf_data.config.header== 0)
    {
        conf_data.config.header=0;
        sprintf(conf_data.config.sno,"0\n");
        sprintf(conf_data.config.mfr,"RTV\n");
        sprintf(conf_data.config.mfr_date,"0\n");
        sprintf(conf_data.config.hw_v,"1.0\n");
        sprintf(conf_data.config.sw_v,"1.0\n");
        sprintf(conf_data.config.conf_date,"0\n");
        sprintf(conf_data.config.prj,"NA\n");
    }
    else
    {

    }
    sprintf(conf_data.ethernet_rs485.ip_addr, "192.168.1.110");
    sprintf(conf_data.ethernet_rs485.net_mask, "255.255.254.0");
    sprintf(conf_data.ethernet_rs485.gateway, "192.168.1.1");
    sprintf(conf_data.ethernet_rs485.dns,"192.168.1.1");
    sprintf(conf_data.ethernet_rs485.domain, "rtv.net");


    sprintf(conf_data.data.def_msg, "INDIAN RAILWAYS WELCOMES YOU");
    sprintf(conf_data.data.def_err, "WELCOME TO INDIAN RAILWAYS");
    conf_data.data.def_brgt=2;
    conf_data.data.def_spd=2;
    conf_data.data.def_wt=10;
    sprintf(conf_data.data.def_buff, "");

}

void
SetupForEthernet(void)
{
    uint32_t ui32User0, ui32User1;
    uint8_t pui8MACAddr[6];


    //
    // Configure SysTick for a 100Hz interrupt.
    //
    MAP_SysTickPeriodSet(g_ui32SysClock / TICKS_PER_SECOND);
    MAP_SysTickEnable();
    MAP_SysTickIntEnable();

    //
    // Get the MAC address from the UART0 and UART1 registers in NV ram.
    //
    MAP_FlashUserGet(&ui32User0, &ui32User1);


    //
    // Convert the 24/24 split MAC address from NV ram into a MAC address
    // array.
    //
    pui8MACAddr[0] = ui32User0 & 0xff;
    pui8MACAddr[1] = (ui32User0 >> 8) & 0xff;
    pui8MACAddr[2] = (ui32User0 >> 16) & 0xff;
    pui8MACAddr[3] = ui32User1 & 0xff;
    pui8MACAddr[4] = (ui32User1 >> 8) & 0xff;
    pui8MACAddr[5] = (ui32User1 >> 16) & 0xff;

    //
    // Format this address into the string used by the relevant widget.
    //
    usnprintf(g_pcMACAddr, SIZE_MAC_ADDR_BUFFER,
              "MAC: %02X-%02X-%02X-%02X-%02X-%02X",
              pui8MACAddr[0], pui8MACAddr[1], pui8MACAddr[2], pui8MACAddr[3],
              pui8MACAddr[4], pui8MACAddr[5]);

    //
    // Remember that we don't have an IP address yet.
    //
    usnprintf(g_pcIPAddr, SIZE_IP_ADDR_BUFFER, "IP: Not assigned");

    //
    // Initialize the lwIP TCP/IP stack.
    //
    lwIPInit(g_ui32SysClock, pui8MACAddr, IP, NM, 0, IPADDR_USE_STATIC);
   // lwIPInit(g_ui32SysClock, pui8MACAddr, 3232235886, 4294966784, 3232235777, IPADDR_USE_STATIC);
    //
  //  lwIPInit(g_ui32SysClock, pui8MACAddr, 0, 0, 0, IPADDR_USE_DHCP);
    // Start the remote software update module.
    //
    SoftwareUpdateInit(SoftwareUpdateRequestCallback);
}

Void HCD_farmware_update(UArg arg0, UArg arg1)
{
    MAP_FPULazyStackingEnable();

                  //
              // Run from the PLL at 120 MHz.
              // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
              // later to better reflect the actual VCO speed due to SYSCTL#22.
              //
              g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                                       SYSCTL_OSC_MAIN |
                                                       SYSCTL_USE_PLL |
                                                       SYSCTL_CFG_VCO_240), 120000000);

           SetupForEthernet();

//           firmware_update_ok=1;
//           EEPROMProgram((uint32_t*)&firmware_update_ok,0x600, sizeof(firmware_update_ok));

           SoftwareUpdateBegin(g_ui32SysClock);

}
Void farmware_update(UArg arg0, UArg arg1)
{
    Task_sleep(500);
    Task_sleep(500);
    Task_sleep(500);

  //  Task_sleep(500);
    int wait_firmware=0;
    while(1)
    {
        wait_firmware++;
        if(wait_firmware==10000000)
        {
            wait_firmware=0;
            break;
        }
    }



    Task_sleep(500);
    Task_sleep(500);
    Task_sleep(500);
    Task_sleep(500);
    Task_sleep(500);
    Task_sleep(500);
    snprintf(pui8IP,4,"%s",conf_data.ethernet_rs485.ip_addr);
    IP1=atoi(pui8IP);
    IP=IP+IP1*256*256*256;
    snprintf(pui8IP,4,"%s",conf_data.ethernet_rs485.ip_addr+4);
    IP1=atoi(pui8IP);
    IP=IP+IP1*256*256;
    snprintf(pui8IP,2,"%s",conf_data.ethernet_rs485.ip_addr+8);
    IP1=atoi(pui8IP);
    IP=IP+IP1*256;
    snprintf(pui8IP,4,"%s",conf_data.ethernet_rs485.ip_addr+10);
    IP1=atoi(pui8IP);
    IP=IP+IP1;

    snprintf(pui8IP,4,"%s",conf_data.ethernet_rs485.net_mask);
    NM1=atoi(pui8IP);
    NM=NM+NM1*256*256*256;
    snprintf(pui8IP,4,"%s",conf_data.ethernet_rs485.net_mask+4);
    NM1=atoi(pui8IP);
    NM=NM+NM1*256*256;
    snprintf(pui8IP,4,"%s",conf_data.ethernet_rs485.net_mask+8);
    NM1=atoi(pui8IP);
    NM=NM+NM1*256;
    snprintf(pui8IP,4,"%s",conf_data.ethernet_rs485.net_mask+12);
    NM1=atoi(pui8IP);
    NM=NM+NM1;

    Task_Params taskParams4;

    Task_Params_init(&taskParams4);
    taskParams4.stackSize = TASKSTACKSIZE;
    taskParams4.stack = &task4Stack;
    taskParams4.priority = 5;
    Task_construct(&task4Struct, (Task_FuncPtr)HCD_farmware_update, &taskParams4, NULL);




}


int main(void)
{
    /* Construct BIOS objects */

    Task_Params taskParams;

    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();
    Board_initEMAC();
    Board_initSPI();
    default_config();

    Task_Params_init(&taskParams);
    taskParams.priority = 1;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)matrixSLScrollTaskFxn, &taskParams, NULL);

    BIOS_start();
    return (0);
}




//
//static void uart4_init()
//{
//    UART_Params uartParams;
//
//    /* Create a UART with data processing off. */
//    UART_Params_init(&uartParams);
//    uartParams.writeDataMode = UART_DATA_BINARY;
//    uartParams.readDataMode = UART_DATA_BINARY;
//    uartParams.readReturnMode = UART_RETURN_FULL;
//    uartParams.readEcho = UART_ECHO_OFF;
//    uartParams.baudRate = 115200;
//    uartParams.readTimeout = 5000;
//    uart4 = UART_open(Board_UART4, &uartParams);
//    if (uart4 == NULL) {
//        System_abort("Error opening the UART3");
//    }
//}

void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{



    static Task_Handle taskHandle;
    Task_Params taskParams;
    Error_Block eb;
    /* Create a HTTP task when the IP address is added */
    if (fAdd && !taskHandle) {
        Error_init(&eb);

        Task_Params_init(&taskParams);
        taskParams.stackSize = TCPHANDLERSTACK;
        taskParams.priority = 1;
        taskParams.arg0 = TCPPORT;
        taskHandle = Task_create((Task_FuncPtr)/*httpTask*/tcpHandler, &taskParams, &eb);
        if (taskHandle == NULL) {
            System_printf("netOpenHook: Failed to create tcpHandler Task\n");
        }
    }
}
static void uart4_init()
{
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uartParams.readTimeout = 5000;
    uart4= UART_open(Board_UART4, &uartParams);
    if (uart4 == NULL) {
        System_abort("Error opening the UART3");
    }
}

