#ifndef CPC_COMMON_H
#define CPC_COMMON_H

#define CPCMSG_HEADER_LEN      12
#define CPC_CAN_MSG_HEADER_LEN 5

#define HIDE_SYMBOL __attribute__((visibility("hidden")))

/* LIBRARY specific *****************************************************/

typedef struct CPC_LIB_PARAMS { char name[32]; // this is to be nice
                                ssize_t (* read)(int fd, void * buf, size_t count);
                                ssize_t (* write)(int fd, const void * buf, size_t count);

                              }CPC_LIB_PARAMS_T;

extern CPC_LIB_PARAMS_T  CPCLibParams[];

extern unsigned int CPCLIBverbose;

// debug
#define DEBUG_INFORMATIVE  5
#define DEBUG_DEBUG        10
#define BE_INFORMATIVE     (CPCLIBverbose >= DEBUG_INFORMATIVE)
#define BE_DEBUG           (CPCLIBverbose >= DEBUG_DEBUG)

#endif
