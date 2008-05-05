#ifndef ETHERCAN_H
#define ETHERCAN_H

#define ETHERCAN_PORT  1500

#define ETHERCAN_VERSION "1.1"

int setupEtherCANConnection(int handle, char * ucChannel);
ssize_t TCP_Read(int fd, void * buf, size_t count);
ssize_t TCP_Write(int fd, const void * buf, size_t count);


#endif
