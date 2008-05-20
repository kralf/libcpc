/************************************************************************/
/* Library routines for CPC-Interfaces                                  */
/*                                                                      */
/* Copyright 2000,2001,2002 Dr.-Ing. Thomas W?nsche                     */
/*                                                                      */
/* Company:  EMS Dr. Thomas Wuensche                                    */
/*           Sonnenhang 3                                               */
/*           85304 Ilmmuenster                                          */
/*           Phone: +49-8441-490260                                     */
/*           Fax:   +49-8441-81860                                      */
/*           email: support@ems-wuensche.com                            */
/*           WWW:   www.ems-wuensche.com                                */
/*                                                                      */
/* All rights reserved                                                  */
/*                                                                      */
/* This code is provided "as is" without warranty of any kind, either   */
/* expressed or implied, including but not limited to the liability     */
/* concerning the freedom from material defects, the fitness for        */
/* particular purposes or the freedom of proprietary rights of third    */
/* parties.                                                             */
/************************************************************************/
/*
 * Revision : 1.00 - 27.07.99 - gu
 *            - started
 *
 * Revision : 1.01 - 01.08.99 - gu
 *            - small bugfix in length of CPC_CANInit
 *
 * Revision : 1.10 - 27.09.99 - gu
 *            - CPC_GetState implemented
 *
 * Revision : 1.11 - 03.06.00 - gu
 *          : - CPC_CANInit() for CPC-XTI is able to handle init for
 *              standard or extended frames
 *            - if VERBOSE is defined, library prints out ERROR messages
 *
 * Revision : 1.2 - 20.08.00 - gu
 *          : - asynchronous driver model implemented
 *              CPC_AddHandler(), CPC_RemoveHandler(), CPC_Handle()
 *
 * Revision : 1.21 - 15.02.01 - gu
 *          : - CPC-XT1000 added in CPC_CANInit()
 *
 * Revision : 1.22 - 10.08.01 - gu
 *            - bugfix: handler functions did not support more than one
 *              channel correct
 *
 * Revision : 1.23 - 13.11.01 - gu
 *            - length codes corrected
 *
 * Revision : 1.24 - 22.11.01 - gu
 *            - length codes bug fixed
 *
 * Revision : 1.33 - 25.01.02 - gu
 *            - controller specific can init params rather than card specfic
 *
 * Revision : 2.00 - 16.04.02 - gu
 *            - modified to fit unified user API
 *
 * Revision : 2.01 - 26.04.02 - gu
 *            - smaller enhancements
 *
 * Revision : 2.02 - 15.11.02 - gu
 *            - CPC_GetInfo implemented
 *            - CPC_BufferClear implemented
 *
 * Revision : 2.12 - 09.12.02 - gu
 *            - Bugfix: CPC_Open()/CPC_Close() did not correspond
 *
 * Revision : 2.22 - 16.12.02 - gu
 *            - EtherCAN client implemented
 *
 * Revision : 2.23 - 16.12.02 - gu
 *            - small bugfix: "while((retval < 0) && (i<3000));"
 *                            instead of:
 *                            "while((retval != 0) && (i<3000));"
 *                            in CPC_OpenChannel()
 *
 * Revision : 2.24 - 30.06.03 - gu
 *            - check for CAN controller type in CPC_CANInit()
 *            - CPC_RequestCANParams() length of cmd corrected
 *            - msgid and ts set to correct values
 *            - new utility function CPC_DecodeErrorMsg()
 *
 * Revision : 2.25 - 03.06.04 - gu
 *            - bugfix: return values which depends on errno have
 *              always been positive
 *            - some error checking done in CPC_DecodeErrorMsg
 *
 * Revision : 2.32 - 11.06.04 - sh
 *            - support for cpcconf.ini
 *            - bugfix: remove handler, init handler count array
 *
 * Revision : 2.33 - xx.02.05 - sh
 *            - bugfix: ini_parser_get_ ...
 *
 * Revision : 2.34 - 01.04.02.05 - sh
 *            - added CPC-XT1000/200 support
 *            - pass irq to cpc-xt driver
 *
 *************************************************************************
 */

#include <sys/types.h>
#include <sys/dir.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/time.h>
#include <errno.h>
#include <unistd.h>

#include "cpclib.h"
#include "cpc_common.h"

#include "ethercan.h"

#include "ini_parser.h"

#define MAX_CHANNEL_ENTRIES  CAN_MAX_DEVICE

#define CPC_LIB_SERIAL       "not applicable"
#define CPC_CONF_INI_PATH    "/etc/cpcconf.ini"

// debug
unsigned int CPCLIBverbose = 0;

CPC_INIT_PARAMS_T CPCInitParams[CAN_MAX_DEVICE];
CPC_LIB_PARAMS_T  CPCLibParams[CAN_MAX_DEVICE];

unsigned int CPCHandlerCnt[CAN_MAX_DEVICE];

unsigned char cpc_version[256] = "CPC-Version (unknown)";

extern int errno;

// definitions for interface types
#define NONE        0
#define CPCPP       1
#define CPCECO      2
#define CPCXT200    3
#define CPCXT1000   4
#define CPCXT527    5
#define CPCCARD     6
#define CPCXTI      7
#define CPCPCI      8
#define ETHERCAN    9
#define CPCUSB     10

#ifdef _CPC_CONF_INI

/*********************************************************************************/
static int _CPC_DetermineDeviceFile(const char *ucChannel, int intf)
{
    char valueBuffer[50];
    int slotNr, busNr, channelNr, serial, portNr, baseAddress, irqNr;
    int procMinor, procSlot, procBus, procChannel, procSerial, procPortNo, procIrqNr, procBaseAddress;

    FILE *f;

    switch(intf) {
        case CPCECO:
            if(!ini_get_key_value(ucChannel, "PrinterPortNr",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            portNr = atoi(valueBuffer);

            /* open the information file provided by the CPC-USB driver */
            f = fopen("/proc/driver/cpc-eco/info", "r");
            if(!f) return -1;

            /* every line is one channel
             * Example:
             * MINOR PORTNO
             * 0     0
             * 1     2
             */
            while(fgets(valueBuffer, sizeof(valueBuffer), f) != NULL) {
                sscanf(valueBuffer, "%d %d", &procMinor, &procPortNo);
                if(portNr == (procPortNo+1)) {
                    fclose(f);
                    return procMinor;
                }
            }
            fclose(f);
            break;
        case CPCUSB:
            if(!ini_get_key_value(ucChannel, "SerialNumber",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            serial = atoi(valueBuffer);

            /* open the information file provided by the CPC-USB driver */
            f = fopen("/proc/driver/cpc-usb/info", "r");
            if(!f) return -1;

            /* every line is one channel
             * Example:
             * MINOR SERIAL
             * 0     1000087
             * 1     1288832
             */
            while(fgets(valueBuffer, sizeof(valueBuffer), f) != NULL) {
                sscanf(valueBuffer, "%d %d", &procMinor, &procSerial);
                if(procSerial == serial || serial == 9999999) {
                    fclose(f);
                    return procMinor;
                }
            }
            fclose(f);
            break;
        case CPCPCI:
            if(!ini_get_key_value(ucChannel, "SlotNr",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            slotNr = atoi(valueBuffer);
            if(!ini_get_key_value(ucChannel, "BusNr",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            busNr = atoi(valueBuffer);
            if(!ini_get_key_value(ucChannel, "ChannelNr",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            channelNr = atoi(valueBuffer);

            /* open the information file provided by the CPC-PCI driver */
            f = fopen("/proc/driver/cpc-pci/info", "r");
            if(!f) return -1;

            /* every line is one channel
             * Example:
             * MINOR CHANNEL BUS SLOT
             * 0     0       0   0B
             * 2     0       0   11
             * 3     1       0   11
             */
            while(fgets(valueBuffer, sizeof(valueBuffer), f) != NULL) {
                sscanf(valueBuffer, "%d %d %d %d", &procMinor, &procChannel, &procBus, &procSlot);
                if(slotNr == procSlot && busNr == procBus  && channelNr == procChannel) {
                    fclose(f);
                    return procMinor;
                }
            }
            fclose(f);
            break;
        case CPCXT200:
        case CPCXT1000:
            if(!ini_get_key_value(ucChannel, "BaseAddress",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            baseAddress = strtoul(valueBuffer, NULL, 16);
            if(!ini_get_key_value(ucChannel, "IrqNr",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            irqNr = atoi(valueBuffer);

            /* open the information file provided by the CPC-PCI driver */
            f = fopen("/proc/driver/cpc-xt/info", "r");
            if(!f) return -1;

            /* every line is one channel
             * Example:
             * MINOR IRQ BASE
             * 0     5   C0000
             * 2     6   D0000
             */
            while(fgets(valueBuffer, sizeof(valueBuffer), f) != NULL) {
                sscanf(valueBuffer, "%d %d %lX", &procMinor, &procIrqNr, &procBaseAddress);
                if(baseAddress == procBaseAddress) {
                    fclose(f);
                    return procMinor;
                }
            }
            fclose(f);
            break;
        case CPCCARD:
            /* We currently ignoring the PortNr value cause it is also
             * not used under windows
             */
            if(!ini_get_key_value(ucChannel, "SlotNr",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            slotNr = atoi(valueBuffer);
            if(!ini_get_key_value(ucChannel, "ChannelNr",
                CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                return -1;
            }
            channelNr = atoi(valueBuffer);

            /* open the information file provided by the CPC-CARD driver */
            f = fopen("/proc/driver/cpc-card/info", "r");
            if(!f) return -1;

            /* every line is one channel
             * Example:
             * MINOR CHANNEL SLOT
             * 0     0       0
             * 2     0       0
             * 3     1       0
             */
            while(fgets(valueBuffer, sizeof(valueBuffer), f) != NULL) {
                sscanf(valueBuffer, "%d %d %d", &procMinor, &procChannel, &procSlot);
                if(slotNr == procSlot && channelNr == procChannel) {
                    fclose(f);
                    return procMinor;
                }
            }
            fclose(f);
            break;
        default:
            return -1;
    }
    return -1;
}

/*********************************************************************************/
static inline int _CPC_InterfaceStrToNumber(const char *intf)
{
       if(strcmp("CPC-PCI", intf)   == 0) return CPCPCI;
  else if(strcmp("CPC-PP", intf)    == 0) return CPCPP;
  else if(strcmp("CPC-PP/ECO", intf)== 0) return CPCECO;
  else if(strcmp("CPC-XT200", intf) == 0) return CPCXT200;
  else if(strcmp("CPC-XT1000", intf)== 0) return CPCXT1000;
  else if(strcmp("CPC-XTI", intf)   == 0) return CPCXTI;
  else if(strcmp("CPC-CARD", intf)  == 0) return CPCCARD;
  else if(strcmp("ETHERCAN", intf)  == 0) return ETHERCAN;
  else if(strcmp("CPC-USB", intf)   == 0) return CPCUSB;

  return NONE;
}
#endif

/*********************************************************************************/
int CPC_OpenChannel(char * ucChannel)
{
    CPC_MSG_T msg;
    static unsigned int firstCall = 1;
    unsigned int slot, i;
    int retval, fd;
    char *devFile = NULL;
    int   intf    = CPCUSB;

#ifdef _CPC_CONF_INI
    char *sectionList[32];
    char  valueBuffer[50];
    int   sectionCount;
    int   minor   = 0;
    int   tmp     = 0;
    int   found   = 0;

    unsigned char configIrq = 0;
    unsigned int  irqNr;
#endif

    if(firstCall){ //initialize internal structures
        for(i = 0; i < CAN_MAX_DEVICE; i++){
            CPC_SJA1000_PARAMS_T *p = &CPCInitParams[i].canparams.cc_params.sja1000;
            CPCHandlerCnt[i] = 0;
            CPCInitParams[i].chanparams.fd     = -1;
            CPCInitParams[i].canparams.cc_type = SJA1000;
            p->acc_code0 = p->acc_code1 = p->acc_code2 = p->acc_code3 = 0x55;
            p->acc_mask0 = p->acc_mask1 = p->acc_mask2 = p->acc_mask3 = 0xFF;
            p->mode = p->btr0 = p->btr1 = p->outp_contr = 0x00;

            CPCHandlerCnt[i] = 0;
        }
        firstCall = 0;
    }

    /* find free slot */
    for(slot=0;slot<CAN_MAX_DEVICE;slot++){
        if(CPCInitParams[slot].chanparams.fd == -1)
            break;
    }
    if(slot >= CAN_MAX_DEVICE)
        return CPC_ERR_NO_FREE_CHANNEL;

#ifdef _CPC_CONF_INI
    /* Get section list from cpcconf.ini */
    tmp = sectionCount = ini_get_section_list(CPC_CONF_INI_PATH, &sectionList[0], 32);
    /* cpcconf.ini was not found */
    if(sectionCount <= 0) {
        if(BE_INFORMATIVE)
            printf("CPCLIB[%s]: File '%s' not found or empty\n", __PRETTY_FUNCTION__, CPC_CONF_INI_PATH);
        return CPC_ERR_NO_INIFILE_PRESENT;
    }
    retval = 0;
    /* iterate over every section and try to find ucChannel */
    for(--sectionCount; sectionCount >= 0; sectionCount--) {
        if(strcmp(ucChannel, sectionList[sectionCount]) == 0) {
            /* ucChannel was found now search InterfaceType to determine the right settings */
			   if(!ini_get_key_value(sectionList[sectionCount], "InterfaceType", CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
                if(BE_INFORMATIVE)
                    printf("CPCLIB[%s]: 'InterfaceType' was not found in '%s::%s'\n", __PRETTY_FUNCTION__, CPC_CONF_INI_PATH,
                    sectionList[sectionCount]);
                retval = CPC_ERR_WRONG_PARAMETERS;
                break;
            }
            if(BE_DEBUG) printf("CPCLIB[%s]: Channel '%s' and interface '%s' found\n", __PRETTY_FUNCTION__, ucChannel, valueBuffer);
            found = 1;
            intf = _CPC_InterfaceStrToNumber(valueBuffer);
            /* check if interface type is known */
            switch(intf) {
                case CPCCARD: /* CPC-ECO Card */
                    minor = _CPC_DetermineDeviceFile(ucChannel, intf);
                    if(minor < 0) {
                        if(BE_INFORMATIVE) printf("CPCLIB[%s]: ERROR _CPC_DetermineDeviceFile() returned %d\n", __PRETTY_FUNCTION__, minor);
                        retval = CPC_ERR_NO_MATCHING_CHANNEL;
                        break;
                    }
                    sprintf(valueBuffer, "/dev/cpc_card%d", minor);
                    devFile = valueBuffer;
                break;
                case CPCXT200:
                case CPCXT1000: /* CPC-XT Card */
                    minor = _CPC_DetermineDeviceFile(ucChannel, intf);
                    if(minor < 0) {
                        if(BE_INFORMATIVE) printf("CPCLIB[%s]: ERROR _CPC_DetermineDeviceFile() returned %d\n", __PRETTY_FUNCTION__, minor);
                        retval = CPC_ERR_NO_MATCHING_CHANNEL;
                        break;
                    }
                    sprintf(valueBuffer, "/dev/cpc_xt%d", minor);
                    devFile   = valueBuffer;
                    configIrq = 1;
                break;
                case CPCPCI: /* CPC-PCI Card */
                    minor = _CPC_DetermineDeviceFile(ucChannel, intf);
                    if(minor < 0) {
                        if(BE_INFORMATIVE) printf("CPCLIB[%s]: ERROR _CPC_DetermineDeviceFile() returned %d\n", __PRETTY_FUNCTION__, minor);
                        retval = CPC_ERR_NO_MATCHING_CHANNEL;
                        break;
                    }
                    sprintf(valueBuffer, "/dev/cpc_pci%d", minor);
                    devFile = valueBuffer;
                break;
                case CPCUSB: /* CPC-USB device */
                    minor = _CPC_DetermineDeviceFile(ucChannel, intf);
                    if(minor < 0) {
                        if(BE_INFORMATIVE) printf("CPCLIB[%s]: ERROR _CPC_DetermineDeviceFile() returned %d\n", __PRETTY_FUNCTION__, minor);
                        retval = CPC_ERR_NO_MATCHING_CHANNEL;
                        break;
                    }
                    sprintf(valueBuffer, "/dev/usb/cpc_usb%d", minor);
                    devFile = valueBuffer;
                break;
                case CPCECO: /* CPC-ECO device */
                    minor = _CPC_DetermineDeviceFile(ucChannel, intf);
                    if(minor < 0) {
                        if(BE_INFORMATIVE) printf("CPCLIB[%s]: ERROR _CPC_DetermineDeviceFile() returned %d\n", __PRETTY_FUNCTION__, minor);
                        retval = CPC_ERR_NO_MATCHING_CHANNEL;
                        break;
                    }
                    sprintf(valueBuffer, "/dev/cpc_eco%d", minor);
                    devFile = valueBuffer;
                break;
                case ETHERCAN: /* Remote EtherCAN Interface */
                    {
                        char ip[32], port[10];
                        if(ini_get_key_value(sectionList[sectionCount], "IpAddress", CPC_CONF_INI_PATH, ip, sizeof(ip))) {
                            sprintf(valueBuffer, "%s:", ip);
                            if(ini_get_key_value(sectionList[sectionCount], "IpPort", CPC_CONF_INI_PATH, port, sizeof(port))) {
                                strcat(valueBuffer, port);
                                devFile = valueBuffer;
                                break;
                            }
                        }
                        retval = CPC_ERR_WRONG_PARAMETERS;
                    }
                break;
                case NONE:
                default:
                    if(BE_INFORMATIVE) printf("CPCLIB[%s]: Unsupported interface '%s'\n", __PRETTY_FUNCTION__, valueBuffer);
                    retval = CPC_ERR_NO_MATCHING_INTERFACE;
                break;
            }
            break;
        }
    }
/* for the transition from devfile opening to cpcconf.ini using
 * we provide a compat mode of the cpclib
 */
#ifndef _CPC_LIB_COMPAT_MODE
    if(!found) retval = CPC_ERR_NO_MATCHING_INTERFACE;
#else
    /* if the given entry was not found we asume it is a device file or ip address */
    if(!found) {
        devFile = ucChannel;
    }
#endif

    if(retval != 0) {
        for(--tmp; tmp >= 0; tmp--) free(sectionList[tmp]);
        return retval;
    }
#else
    devFile = ucChannel;
#endif
    if((fd = open(devFile, O_RDWR)) < 0) {
        if((fd = setupEtherCANConnection(slot, devFile)) < 0){
            if(BE_INFORMATIVE) perror(devFile);
            return CPC_ERR_CHANNEL_NOT_ACTIVE;
        }
    } else {
        CPCLibParams[slot].read  = read;
        CPCLibParams[slot].write = write;
    }

    if(BE_DEBUG) printf("CPCLIB[%s]: Device %s (%s) opened with handle: %d, fd: %d!\n", __PRETTY_FUNCTION__, devFile, ucChannel, slot, fd);

    CPCInitParams[slot].chanparams.fd = fd;

#ifndef __uClinux__
    /* inquires CPC-Version; used with CPC-XTI, CPC-PP */
    i = 0;
    do{ retval = CPC_RequestInfo(slot, 0, CPC_INFOMSG_T_INTERFACE, CPC_INFOMSG_T_VERSION); } while((retval < 0) && (++i<3000));

    if(retval < 0) {
        if(BE_INFORMATIVE) printf("CPCLIB[%s]: ERROR CPC_RequestInfo() returned %d\n", __PRETTY_FUNCTION__, retval);
        CPC_CloseChannel(slot);
        return CPC_ERR_NO_INTERFACE_PRESENT;
    }

#ifdef _CPC_CONF_INI
	if(configIrq) {
		if(!ini_get_key_value(ucChannel, "IrqNr", CPC_CONF_INI_PATH, valueBuffer, sizeof(valueBuffer))) {
			if(BE_INFORMATIVE) printf("CPCLIB[%s]: Could find IRQ number\n", __PRETTY_FUNCTION__);
			CPC_CloseChannel(slot);
			return CPC_ERR_WRONG_PARAMETERS;
		}
		irqNr = atoi(valueBuffer);
		msg.type   = CPC_CMD_T_OPEN_CHAN;
		msg.length = 2;
		msg.msg.generic[0] = (unsigned char)irqNr;

		retval = CPCLibParams[slot].write(CPCInitParams[slot].chanparams.fd, (void *)&msg, sizeof(CPC_MSG_T));
		if(retval < 0) {
			if(BE_INFORMATIVE) printf("CPCLIB[%s]: Could not configure IRQ line\n", __PRETTY_FUNCTION__);
			CPC_CloseChannel(slot);
			return CPC_ERR_NO_RESOURCES;
		}
	}
#endif

    for(i = 0; i < 6000; i++) {
        retval = CPCLibParams[slot].read(CPCInitParams[slot].chanparams.fd, (void *)&msg, sizeof(CPC_MSG_T));
        if(retval > 0){
            if(msg.type == CPC_MSG_T_INFO) {
                if((msg.msg.info.source == CPC_INFOMSG_T_INTERFACE) && (msg.msg.info.type == CPC_INFOMSG_T_VERSION)){
                    memcpy(cpc_version, msg.msg.info.msg, msg.length-2);
                    cpc_version[msg.length-1] = '\0';
                    break;
                }
            } else { /* something else ?                */
                if(BE_INFORMATIVE) {
                    printf("CPCLIB[%s]: Type: %2.2x, Len: %d, D:", __PRETTY_FUNCTION__, msg.type,msg.length);
                    for(i = 0; i < msg.length; i++) printf("%2.2x ",msg.msg.generic[i]);
                    printf("\n");
                }
            }
        } else if(retval < 0 && BE_DEBUG) { /* only on error */
            printf("CPCLIB[%s]: CPC_RequestInfo() read returned %d, errno: %d\n", __PRETTY_FUNCTION__, retval, -errno);
        }
    }

    if(i > 6000) {
        if(BE_INFORMATIVE) printf("CPCLIB[%s]: ERROR CPC_GetVersion() did not return anything\n", __PRETTY_FUNCTION__);
        CPC_CloseChannel(slot);
        return CPC_ERR_NO_INTERFACE_PRESENT;
    }

#endif
    return slot;
}

/*********************************************************************************/
int CPC_CloseChannel(int handle)
{
  if(BE_DEBUG)
    printf("CPCLIB[%s]: %d]\n", __PRETTY_FUNCTION__, handle);

  if(handle >= CAN_MAX_DEVICE)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;
  if(CPCInitParams[handle].chanparams.fd == -1)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  close(CPCInitParams[handle].chanparams.fd);
  CPCInitParams[handle].chanparams.fd = -1;

  return 0;

}

/*********************************************************************************/
CPC_INIT_PARAMS_T * CPC_GetInitParamsPtr(int handle)
{

  if(BE_DEBUG)
    printf("CPCLIB[%s]: %d]\n", __PRETTY_FUNCTION__, handle);

  if(CPCInitParams[handle].chanparams.fd < 0)
    return NULL;

  return &CPCInitParams[handle];

}

/*********************************************************************************/
int CPC_CANInit(int handle, unsigned char confirm)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  unsigned char len;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  switch(CPCInitParams[handle].canparams.cc_type){
    case PCA82C200:{
     len = 1+sizeof(CPC_PCA82C200_PARAMS_T);
    }
    break;
    case SJA1000:{
     len = 1+sizeof(CPC_SJA1000_PARAMS_T);
    }
    break;
    case AN82527:{
     return CPC_ERR_WRONG_CONTROLLER_TYPE;
    }
    break;
    case M16C_BASIC:{
     len = 1+sizeof(CPC_M16C_BASIC_PARAMS_T);
    }
    break;
  }

  msg.type    = CPC_CMD_T_CAN_PRMS;
  msg.length  = len;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  memcpy(&msg.msg.canparams, &CPCInitParams[handle].canparams, len);

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN + msg.length) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_CANExit(int handle, unsigned char confirm)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;
  unsigned char len = 0;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_CAN_EXIT;
  msg.length  = len;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN + msg.length) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_Control(int handle, unsigned short value)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_CONTROL;
  msg.length  = sizeof(unsigned short);
  msg.msgid   = 0;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  *(unsigned short *)&msg.msg.generic[0] = value;

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN + sizeof(unsigned short)) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_SendMsg(int handle, unsigned char confirm, CPC_CAN_MSG_T * canmsg)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_CAN;
  msg.length  = CPC_CAN_MSG_HEADER_LEN + canmsg->length;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  memcpy(&msg.msg.canmsg, canmsg, sizeof(CPC_CAN_MSG_T));

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T)) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_SendRTR(int handle, unsigned char confirm, CPC_CAN_MSG_T * canmsg)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_RTR;
  msg.length  = CPC_CAN_MSG_HEADER_LEN;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  memcpy(&msg.msg.canmsg, canmsg, sizeof(CPC_CAN_MSG_T));

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T)) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_SendXMsg(int handle, unsigned char confirm, CPC_CAN_MSG_T * canmsg)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_XCAN;
  msg.length  = CPC_CAN_MSG_HEADER_LEN + canmsg->length;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  memcpy(&msg.msg.canmsg, canmsg, sizeof(CPC_CAN_MSG_T));

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T)) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_SendXRTR(int handle, unsigned char confirm, CPC_CAN_MSG_T * canmsg)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_XRTR;
  msg.length  = CPC_CAN_MSG_HEADER_LEN;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  memcpy(&msg.msg.canmsg, canmsg, sizeof(CPC_CAN_MSG_T));

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN + sizeof(CPC_CAN_MSG_T)) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_RequestCANState(int handle, unsigned char confirm)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_CANSTATE;
  msg.length  = 0;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_RequestCANParams(int handle, unsigned char confirm)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_INQ_CAN_PARMS;
  msg.length  = 1;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  msg.msg.canparams.cc_type = CPCInitParams[handle].canparams.cc_type;

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN+msg.length) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
int CPC_RequestInfo(int handle, unsigned char confirm, unsigned char source, unsigned char type)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type    = CPC_CMD_T_INQ_INFO;
  msg.length  = 2;
  msg.msgid   = confirm;
  msg.ts_sec  = 0L;
  msg.ts_nsec = 0L;

  msg.msg.info.source = source;
  msg.msg.info.type   = type;

  if(CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN+2) >= 0)
    return 0;
  else
    return -errno;

}

/*********************************************************************************/
/* Functions to handle CPC messages in synchronous programming model             */
/*********************************************************************************/

/******************************************************************************
* Function.....: CPC_WaitForMType                                             *
*                                                                             *
* Task.........: Waits for a message type specified by the 2. parameter       *
*                NOTE: all other messages are discarded if no handlers are    *
*                      hooked !!                                              *
*                                                                             *
* Parameters...: int                : index of the Channel number             *
*                int                : message type                            *
*                                                                             *
* Return values: CPC_MSG_T          : pointer to a CPC_MSG                    *
*                NULL               : an error or timeout occurred            *
******************************************************************************/
CPC_MSG_T* CPC_WaitForMType(int handle, int mtype)
{
    CPC_MSG_T* pMsg;
    struct timeval tv;
    unsigned long sec;

    gettimeofday(&tv, NULL);
    sec = tv.tv_sec;

    do {
        pMsg = CPC_Handle(handle);
        if ((pMsg != NULL) && (pMsg->type == mtype))
            return pMsg;
        gettimeofday(&tv, NULL);
    } while (tv.tv_sec - sec < 5); /* wait 5 sec. */

  return NULL;
}

/******************************************************************************
* Function.....: CPC_Get_CANState                                             *
*                                                                             *
* Task.........: Get the version of the driver related to the                 *
*                specified channel                                            *
*                                                                             *
* Parameters...: int                : index of the Channel number             *
*                                                                             *
* Return values: int   : the current CANstate                                 *
*                -1    : an error occurred                                    *
******************************************************************************/
int CPC_GetCANState(int handle)
{
    int result;
    CPC_MSG_T* pMsg;

    /* first we send a request to the driver */
    if(!(result = CPC_RequestCANState(handle, DO_NOT_CONFIRM))) {
        /* now we wait for the expected message */
        pMsg = CPC_WaitForMType(handle, CPC_MSG_T_CANSTATE);
        if(pMsg != NULL)
            return (pMsg->msg.canstate);
    }
    return -1;
}

char * cpcLibVersion    = CPC_LIB_VERSION;
char * cpcLibSerial     = CPC_LIB_SERIAL;
char * infoSourceBummer = "unknown source";
char * infoTypeBummer   = "unknown type";
char * noResponseBummer = "no response";
char * noHandleBummer   = "handle not valid";

char * CPC_GetInfo(int handle, unsigned char source, unsigned char type)
{
  CPC_MSG_T * cpc;
  int i;

  switch(source){
/**/case CPC_INFOMSG_T_LIBRARY:{
      switch(type){
        case CPC_INFOMSG_T_VERSION:
          return cpcLibVersion;
        case CPC_INFOMSG_T_SERIAL:
          return cpcLibSerial;
        default:
          return infoTypeBummer;
      }
    }
    break;

/**/case CPC_INFOMSG_T_DRIVER:
    case CPC_INFOMSG_T_INTERFACE:{

      if(CPC_RequestInfo(handle, 0, source, type) < 0)
        return noHandleBummer;

    }
    break;

/**/default:
      return infoSourceBummer;

  }

  cpc = CPC_WaitForMType(handle, CPC_MSG_T_INFO);
  if(cpc) {
        if((cpc->msg.info.source == source) && (cpc->msg.info.type == type))
            return (char *)&cpc->msg.info.msg;
  }

  return noResponseBummer;

}

/*********************************************************************************/
int CPC_ClearCMDQueue(int handle, unsigned char confirm)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type   = CPC_CMD_T_CLEAR_CMD_QUEUE;
  msg.ts_sec = 0UL;
  msg.ts_nsec= 0UL;
  msg.msgid  = confirm;
  msg.length = 0;

  return CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN);
}

/*********************************************************************************/
int CPC_ClearMSGQueue(int handle)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type   = CPC_CMD_T_CLEAR_MSG_QUEUE;
  msg.length = 0;

  return CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN);
}

/*********************************************************************************/
int CPC_GetMSGQueueCnt(int handle)
{
  CPC_MSG_T msg;
  int fd = CPCInitParams[handle].chanparams.fd;

  if(fd < 0)
    return CPC_ERR_CHANNEL_NOT_ACTIVE;

  msg.type   = CPC_CMD_T_INQ_MSG_QUEUE_CNT;
  msg.length = 0;

  return CPCLibParams[handle].write(fd, &msg, CPCMSG_HEADER_LEN);

}

/*********************************************************************************/
/* Utility functions                                                             */
/*********************************************************************************/
char * CPCErrorStr[] = {"SUCCESS",
                     "CPC_ERR_NO_FREE_CHANNEL -> no more free space within the channel array",
                     "CPC_ERR_CHANNEL_ALREADY_OPEN -> the channel is already open",
                     "CPC_ERR_CHANNEL_NOT_ACTIVE -> access to a channel not active failed",
                     "CPC_ERR_NO_DRIVER_PRESENT -> no driver at the location searched by the library",
                     "CPC_ERR_NO_INIFILE_PRESENT -> the library could not find the inifile",
                     "CPC_ERR_WRONG_PARAMETERS -> wrong parameters in the inifile",
                     "CPC_ERR_NO_INTERFACE_PRESENT -> the specified interface is not connected",
                     "CPC_ERR_NO_MATCHING_CHANNEL -> the driver couldn't find a matching channel",
                     "CPC_ERR_NO_BUFFER_AVAILABLE -> the driver couldn't allocate buffer for messages",
                     "CPC_ERR_NO_INTERRUPT -> the requested interrupt couldn't be virtualized",
                     "CPC_ERR_NO_MATCHING_INTERFACE -> no interface type related to this channel was found",
                     "CPC_ERR_NO_RESOURCES -> the requested resources could not be claimed",
                     "CPC_ERR_SOCKET -> error concerning TCP sockets",
// init error codes
                     "CPC_ERR_WRONG_CONTROLLER_TYPE -> wrong CAN controller type within initialization",
                     "CPC_ERR_NO_RESET_MODE -> the controller could not be set into reset mode",
                     "CPC_ERR_NO_CAN_ACCESS -> the CAN controller could not be accessed",
                     "Not defined",
                     "Not defined",
                     "Not defined",
// transmit error codes
                     "CPC_ERR_CAN_WRONG_ID -> the provided CAN id is too big",
                     "CPC_ERR_CAN_WRONG_LENGTH -> the provided CAN length is too long",
                     "CPC_ERR_CAN_NO_TRANSMIT_BUF -> the transmit buffer was occupied",
                     "CPC_ERR_CAN_TRANSMIT_TIMEOUT -> the message could not be sent within a specified time",
                     "Not defined",
                     "Not defined",
                     "Not defined",
                     "Not defined",
                     "Not defined",
                     "Not defined",
// other error codes
                     "CPC_ERR_SERVICE_NOT_SUPPORTED -> the requested service is not supported by the interface",
                     "CPC_ERR_IO_TRANSFER -> a transmission error down to the driver occurred",
                     "CPC_ERR_TRANSMISSION_FAILED -> a transmission error down to the interface occurred",
                     "CPC_ERR_TRANSMISSION_TIMEOUT -> a timeout occurred within transmission to the interface",
                     "Not defined",
                     "CPC_ERR_OP_SYS_NOT_SUPPORTED -> the operating system is not supported",
                     "Not defined",
                     "Not defined",
                     "Not defined",
                     "Not defined",
                     "CPC_ERR_UNKNOWN -> an unknown error ocurred",
                     "Unknown error code"};

char * CPC_DecodeErrorMsg(int error){

  unsigned int index = abs(error);
  static unsigned int sizeOfErr = (sizeof(CPCErrorStr)/sizeof(char *));

  /* No error */
  if(error >= 0) return CPCErrorStr[0];

  if(index < sizeOfErr)
    return CPCErrorStr[index];
  else
    return CPCErrorStr[sizeOfErr];
}

