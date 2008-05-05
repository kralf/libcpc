/************************************************************************/
/*            EtherCAN Client Implementation for CPC-LIB                */
/* Copyright 2000-2005 EMS Dr.Thomas Wuensche                           */
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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <netdb.h>

#include <arpa/inet.h>
#include <errno.h>

#include "cpc.h"
#include "cpclib.h"

#include "cpc_common.h"
#include "ethercan.h"

#define TRUE 1
#define FALSE 0

struct sockaddr_in ethercan;

inline HIDE_SYMBOL unsigned char *u8_to_hex (unsigned char  *dest, unsigned value);
inline HIDE_SYMBOL unsigned       hex_to_u8 (unsigned char **src);

void SetDescriptorToNonBlock(int sockfd);

/******************************************************************************
* Function.....: setupEtherCANConnection                                      *
*                                                                             *
* Task.........:                                                              *
*                                                                             *
* Parameters...:                                                              *
*                                                                             *
* Return values: socket or < 0                                                *
******************************************************************************/
int setupEtherCANConnection(int handle, char * ucChannel)
{
  static int sock;
  char *port;
  struct hostent * h;

  port = strstr(ucChannel, ":");

  ethercan.sin_family      = AF_INET;
  ethercan.sin_port        = htons( port == NULL ? ETHERCAN_PORT : atoi(port+1) );

  if(port) *port = '\0';
  if((h = gethostbyname(ucChannel)) == NULL){
    if(BE_INFORMATIVE)
      printf("ERROR[CPCLIB->setupEtherCANConnection()]: gethostbyname failed!\n");
        return CPC_ERR_NO_INTERFACE_PRESENT;
  }
  memcpy(&ethercan.sin_addr.s_addr, h->h_addr, h->h_length);

  if(BE_INFORMATIVE)
    printf("MESSAGE[CPCLIB->setupEtherCANConnection()]: IP address is %x\n", ethercan.sin_addr.s_addr);

  if((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_IP)) < 0){
    if(BE_INFORMATIVE)
  		printf("MESSAGE[CPCLIB->setupEtherCANConnection()]: createSocket\n");
		return CPC_ERR_SOCKET;
	}

  if(BE_INFORMATIVE)
    printf("MESSAGE[CPCLIB->setupEtherCANConnection()]: try to connect\n");

  if(connect(sock, (struct sockaddr *)&ethercan, sizeof(struct sockaddr)) < 0){
    if(BE_INFORMATIVE)
      printf("MESSAGE[CPCLIB->setupEtherCANConnection()]: connect() to EtherCAN failed!\n");
    close(sock);
    return CPC_ERR_NO_INTERFACE_PRESENT;
  }
  if(BE_INFORMATIVE)
    printf("MESSAGE[CPCLIB->setupEtherCANConnection()]: connected to EtherCAN %s!\n", ucChannel);

  memcpy(CPCLibParams[handle].name, "EtherCAN", sizeof("EtherCAN"));

  CPCLibParams[handle].read  = TCP_Read;
  CPCLibParams[handle].write = TCP_Write;

  SetDescriptorToNonBlock(sock);
  return sock;

}

/******************************************************************************
* Function.....: TCP_Write                                                    *
*                                                                             *
* Task.........: sends a CPC_MSG via TCP                                      *
*                                                                             *
* Parameters...:                             *
*                                                                             *
* Return values: 0: controller was initialzed                                 *
*                1: init mode could not be set                                *
******************************************************************************/
ssize_t TCP_Write(int fd, const void * buf, size_t count)
{
  static unsigned char sendBuf[255];
  static unsigned char sendBufFree  = TRUE;
  static unsigned int  bytesWritten = 0;

  unsigned char *ptr;
  unsigned int   i;
  ssize_t        wb;
  unsigned int   len;

  CPC_MSG_T     *cpc = (CPC_MSG_T *)buf;

  /* check if message is pending */
  if(sendBufFree == FALSE) {
  	len = strlen((char *)sendBuf);

  	wb = write(fd, &sendBuf[bytesWritten], len-bytesWritten);

  	if(wb >= 0) {
  		bytesWritten += wb;

  		if(bytesWritten == len) { /* message got completly sent */
  			sendBufFree = TRUE;
  		} else { /* message still incomplete */
  			return CPC_ERR_CAN_NO_TRANSMIT_BUF;
  		}
  	} else {
  		if(BE_INFORMATIVE) {
		      printf("MESSAGE[CPCLIB->TCP_Write()]: Interface not present\n");
    	}
	    return CPC_ERR_NO_INTERFACE_PRESENT;
  	}
  }

  ptr = sendBuf;

  *ptr++ = 'S';
  ptr = u8_to_hex(ptr, cpc->type);
  ptr = u8_to_hex(ptr, cpc->length);
  ptr = u8_to_hex(ptr, cpc->msgid);
  ptr = u8_to_hex(ptr, cpc->ts_sec);
  ptr = u8_to_hex(ptr, (cpc->ts_sec >> 8));
  ptr = u8_to_hex(ptr, (cpc->ts_sec >> 16));
  ptr = u8_to_hex(ptr, (cpc->ts_sec >> 24));
  ptr = u8_to_hex(ptr, cpc->ts_nsec);
  ptr = u8_to_hex(ptr, (cpc->ts_nsec >> 8));
  ptr = u8_to_hex(ptr, (cpc->ts_nsec >> 16));
  ptr = u8_to_hex(ptr, (cpc->ts_nsec >> 24));

  for(i=0; i < cpc->length; i++) {
    ptr = u8_to_hex(ptr, cpc->msg.generic[i]);
  }

  *ptr++ = 'T'; /* we sign the end of the TCP frame */
  *ptr++ = '\0';

  len = strlen((char *)sendBuf);
  wb = write(fd, sendBuf, len);

  if(wb < 0) { /* error? */
    if(BE_INFORMATIVE) {
      printf("MESSAGE[CPCLIB->TCP_Write()]: Interface not present\n");
    }
    return CPC_ERR_NO_INTERFACE_PRESENT;
  } else if(wb != len && wb >= 0) { /* not all data could be sent */
  	sendBufFree  = FALSE;
  	bytesWritten = wb;
  }

  return count;
}


/******************************************************************************
* Function.....: TCP_Read                                                     *
*                                                                             *
* Task.........: reads a CPC_MSG via TCP                                      *
*                                                                             *
* Parameters...:                             *
*                                                                             *
* Return values: 0: controller was initialzed                                 *
*                1: init mode could not be set                                *
******************************************************************************/
#define MAX_CONV_MSG_BUF_SIZE 70
#define MAX_RECV_TCPFRAME_BUF_SIZE 500
#define FIND_BEGIN  0
#define STORE_DATA  1

ssize_t TCP_Read(int remote_socket, void * pCpcMsg, size_t count)
{
  static int i = 0;                                          /* important int ... */
  static int j = 0;                                          /* ... for the state-machine */
  static int tcpframelength;                                 /* length of the received tcp frame */
  static int state  = FIND_BEGIN;                            /* the state for the state-machine */
  static unsigned char convertBuffer[MAX_CONV_MSG_BUF_SIZE]; /* the converted message */
  static unsigned char recvbuf[MAX_RECV_TCPFRAME_BUF_SIZE];  /* the received messages */
  static unsigned char ready = TRUE;
  int result, k;                                                /* a check variable */

  CPC_MSG_T *cpc = (CPC_MSG_T *)pCpcMsg;
  unsigned char * tmpbufptr;

  /* check, if we proceed all messages */
  if(ready == TRUE) {
    /* all data proceeded, get new frames from TCP stack */
    memset(&recvbuf, 0, sizeof(recvbuf)); /* fill the recvbuf with 0 completly */
    errno  = 0;                            /* set errno */
    result = 0;                           /* set result */

    result = recv(remote_socket, &recvbuf[0], MAX_RECV_TCPFRAME_BUF_SIZE, 0); /* read from the TCP stack */

    if(result == 0 || errno == 104) { /* check for result */
      perror("\nClient has disconnected\n");
      return CPC_ERR_NO_INTERFACE_PRESENT;
    } else {
      /* check if there is data available in the buffer */
      if(result > 0){
        tcpframelength = result;
        i=0;
        ready = FALSE;
      } else{
        return 0;
      }
    }
  }

  /* the "start" of the state-machine */
  while(i < tcpframelength){
    switch(state){
      case FIND_BEGIN:
        if(recvbuf[i] == 'S') {
          state = STORE_DATA;
          memset(&convertBuffer, 0, sizeof(convertBuffer));
          recvbuf[i] = '0';
        }
        i++;
        break;
      case STORE_DATA:
        if(recvbuf[i] != 'T') {
          convertBuffer[j] = recvbuf[i];
          recvbuf[i] = '0';
          i++;
          j++;
        } else {
          convertBuffer[j] = '\0';
          recvbuf[i++] = '0';
          j = 0;
          state = FIND_BEGIN;
          ready = FALSE;

          memset(cpc, 0, sizeof(CPC_MSG_T));

          /* As casting doesn't seem to work, we convert manually */
          tmpbufptr = &convertBuffer[0];

          /* CPC_MSG type */
          cpc->type = hex_to_u8(&tmpbufptr);

          /* CPC_MSG length */
          cpc->length = hex_to_u8(&tmpbufptr);

          /* Skip msg_id */
          tmpbufptr += 2;

          /* CPC_MSG ts_sec */
          cpc->ts_sec = hex_to_u8(&tmpbufptr);
          cpc->ts_sec |= hex_to_u8(&tmpbufptr)<<8;
          cpc->ts_sec |= hex_to_u8(&tmpbufptr)<<16;
          cpc->ts_sec |= hex_to_u8(&tmpbufptr)<<24;

          /* CPC_MSG ts_nsec */
          cpc->ts_nsec = hex_to_u8(&tmpbufptr);
          cpc->ts_nsec |= hex_to_u8(&tmpbufptr)<<8;
          cpc->ts_nsec |= hex_to_u8(&tmpbufptr)<<16;
          cpc->ts_nsec |= hex_to_u8(&tmpbufptr)<<24;

          /* CPC_MSG data */
          for(k= 0; k < cpc->length; k++) {
            cpc->msg.generic[k] = hex_to_u8(&tmpbufptr);
          }
          return sizeof(CPC_MSG_T);
        }
        break;
    }
  }
  i = 0;
  tcpframelength = 0;
  recvbuf[0] = '\0';
  ready = TRUE;

  return 0;
}

/******************************************************************************/
/* utilities                                                                  */
/******************************************************************************/
inline HIDE_SYMBOL unsigned char * u8_to_hex(unsigned char * dest, unsigned value)
{
	unsigned high_nib, low_nib;

	low_nib = (value & 0xf) + 0x30;

	if (low_nib > 0x39) low_nib += 0x27;

	high_nib = ((value >> 4) & 0xf) + 0x30;

	if (high_nib > 0x39) high_nib += 0x27;

	*dest++ = high_nib;
	*dest++ = low_nib;

	return dest;
}

/******************************************************************************/
inline HIDE_SYMBOL unsigned hex_to_u8(unsigned char ** src)
{
	unsigned low_nib, high_nib;

	high_nib = *(*src)++ - 0x30;

	if (high_nib > 9){
		high_nib = high_nib | 0x20;
		high_nib -= 0x27;
	}

	low_nib = *(*src)++ - 0x30;

	if (low_nib > 9){
		low_nib = low_nib | 0x20;
		low_nib -= 0x27;
	}

	return (high_nib << 4) + low_nib;
}

/**********************************************
 * Function-name: void SetDeskriptorToNonBlock(int sockfd)
 * Author: Sebastian Haas <haas@ems-wuensche.com>
 * Date: 11.10.2002
 * Description: This function set a socket to non-blocking!
 * Parameter:
 * int sockfd = the socket filedeskriptor
 * Return-value: void
 **********************************************/
void HIDE_SYMBOL SetDescriptorToNonBlock(int sockfd)
{
    int prev_mode;

    if(( prev_mode = fcntl(sockfd, F_GETFL, 0)) != -1)
      fcntl(sockfd, F_SETFL, prev_mode | O_NONBLOCK);
    return;
}

