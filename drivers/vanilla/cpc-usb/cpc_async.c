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
#include <sys/types.h>
#include <sys/dir.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>
#include <sys/time.h>
#include <errno.h>
#include <unistd.h>

#include "cpc.h"
#include "cpclib.h"
#include "cpc_common.h"

/*********************************************************************************/
/* Functions to handle CPC messages in an asynchronous programming model         */
/*********************************************************************************/
extern unsigned int      CPCHandlerCnt[CAN_MAX_DEVICE];
extern CPC_INIT_PARAMS_T CPCInitParams[CAN_MAX_DEVICE];
extern CPC_LIB_PARAMS_T  CPCLibParams [CAN_MAX_DEVICE];

#define CPC_HANDLER_CNT 50

typedef enum {
    HDLR_STANDARD = 0,
    HDLR_EXTENDED
} EN_HDLR_T;

struct CPC_HANDLER{
    EN_HDLR_T type;

    /* standard handler */
    void (*cpc_handler)(int, const CPC_MSG_T *);

    /* extended handler */
    void (*cpc_handlerEx)(int, const CPC_MSG_T*, void *);
    void *customPointer;
};

struct CPC_HANDLER CPCHandlers[CAN_MAX_DEVICE][CPC_HANDLER_CNT];


/*********************************************************************************/
/* add a handler to the list                                                     */
int CPC_AddHandlerEx      (int handle, void (*handlerEx)(int handle, const CPC_MSG_T*, void *customPointer), void *customPointer)
{
    if(CPCInitParams[handle].chanparams.fd < 0)
        return CPC_ERR_CHANNEL_NOT_ACTIVE;

if (CPCHandlerCnt[handle] < CPC_HANDLER_CNT) {
        CPCHandlers[handle][CPCHandlerCnt[handle]].cpc_handlerEx = handlerEx;
        CPCHandlers[handle][CPCHandlerCnt[handle]].customPointer = customPointer;
        CPCHandlers[handle][CPCHandlerCnt[handle]].type          = HDLR_EXTENDED;
        CPCHandlerCnt[handle]++;

        return 0;
    }
    return -1;
}

/*********************************************************************************/
/* remove a handler from the list                                                */
int CPC_RemoveHandlerEx   (int handle, void (*handlerEx)(int handle, const CPC_MSG_T*, void *customPointer))
{
    signed int i;

    if(CPCInitParams[handle].chanparams.fd < 0)
        return CPC_ERR_CHANNEL_NOT_ACTIVE;

    for (i = CPCHandlerCnt[handle]-1; i >= 0; i--) {
        if (CPCHandlers[handle][i].cpc_handlerEx == handlerEx) {
            for (; i < CPCHandlerCnt[handle]; i++){
                CPCHandlers[handle][i].cpc_handler   = CPCHandlers[handle][i+1].cpc_handler;
                CPCHandlers[handle][i].cpc_handlerEx = CPCHandlers[handle][i+1].cpc_handlerEx;
                CPCHandlers[handle][i].type          = CPCHandlers[handle][i+1].type;
                CPCHandlers[handle][i].customPointer = CPCHandlers[handle][i+1].customPointer;
            }
            CPCHandlerCnt[handle]--;
            return 0;
        }
    }
    return -1;
}

/*********************************************************************************/
/* add a handler to the list                                                     */
int CPC_AddHandler(int handle, void (*handler)(int handle, const CPC_MSG_T *))
{
    if(CPCInitParams[handle].chanparams.fd < 0)
        return CPC_ERR_CHANNEL_NOT_ACTIVE;

    if (CPCHandlerCnt[handle] < CPC_HANDLER_CNT) {
        CPCHandlers[handle][CPCHandlerCnt[handle]].cpc_handler = handler;
        CPCHandlers[handle][CPCHandlerCnt[handle]].type        = HDLR_STANDARD;
        CPCHandlerCnt[handle]++;

        return 0;
    }
    return -1;
}

/*********************************************************************************/
/* remove a handler from the list                                                */
int CPC_RemoveHandler(int handle, void (*handler)(int handle, const CPC_MSG_T *))
{
    signed int i;

    if(CPCInitParams[handle].chanparams.fd < 0)
        return CPC_ERR_CHANNEL_NOT_ACTIVE;

    for (i = CPCHandlerCnt[handle]-1; i >= 0; i--) {
        if (CPCHandlers[handle][i].cpc_handler == handler) {
            for (; i < CPCHandlerCnt[handle]; i++){
                CPCHandlers[handle][i].cpc_handler   = CPCHandlers[handle][i+1].cpc_handler;
                CPCHandlers[handle][i].cpc_handlerEx = CPCHandlers[handle][i+1].cpc_handlerEx;
                CPCHandlers[handle][i].type          = CPCHandlers[handle][i+1].type;
                CPCHandlers[handle][i].customPointer = CPCHandlers[handle][i+1].customPointer;
            }
            CPCHandlerCnt[handle]--;
            return 0;
        }
    }
    return -1;
}

/*********************************************************************************/
/* execute all handlers in the list                                              */
CPC_MSG_T * CPC_Handle(int handle)
{
    static CPC_MSG_T msg;
    unsigned int i;
    ssize_t retval;
    int fd = CPCInitParams[handle].chanparams.fd;

    if(fd < 0)
        return NULL;

    errno = 0;

    retval = CPCLibParams[handle].read(fd, &msg, sizeof(CPC_MSG_T));

    if(-errno == CPC_ERR_NO_INTERFACE_PRESENT || retval == CPC_ERR_NO_INTERFACE_PRESENT) {
        msg.type = CPC_MSG_T_DISCONNECTED;
        msg.length = 1;

        /* sh 07.03.2005
	 * device has been disconnected inform application
	 * via handlers
	 */
	retval = sizeof(CPC_MSG_T);
    }

    if(retval > 0){
        for (i = 0; i < CPCHandlerCnt[handle]; i++){
            if(CPCHandlers[handle][i].type == HDLR_STANDARD)
                CPCHandlers[handle][i].cpc_handler(handle, &msg);
            else
                CPCHandlers[handle][i].cpc_handlerEx(handle, &msg, CPCHandlers[handle][i].customPointer);
        }
        return &msg;
    }

    return NULL;
}
