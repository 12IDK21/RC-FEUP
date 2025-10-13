// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;
    strcpy(ll.serialPort,serialPort);
    ll.role = (strcmp(role,"tx")==0) ? 0 : 1;
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;
    if(llopen(ll)==0){
        if(ll.role==0){
            const unsigned char buf[256] = {'h','e','l','l','o'};
            llwrite(buf,256);
        }
        else{
            unsigned char packet;
            llread(&packet);
        }
    }
}

