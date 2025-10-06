// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;
    strcpy(serialPort,ll.serialPort);
    if(role == "tx") ll.role ? 0 : 1;
    
    llopen(ll);
}

