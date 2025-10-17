// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;
    strcpy(ll.serialPort, serialPort);
    ll.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    printf("[APP] Starting on port %s as %s...\n", serialPort,
           (ll.role == LlTx ? "Transmitter" : "Receiver"));

    if (llopen(ll) != 0) {
        fprintf(stderr, "[APP] llopen failed.\n");
        return;
    }

    if (ll.role == LlTx) {
        const unsigned char msg[] = "hello link layer";
        int len = sizeof(msg);
        printf("[APP] Sending test message (%d bytes)...\n", len);
        if (llwrite(msg, len) > 0)
            printf("[APP] Message sent successfully.\n");
        else
            fprintf(stderr, "[APP] Error sending message.\n");
    } else {
        unsigned char buffer[256];
        int n = llread(buffer);
        if (n > 0) {
            printf("[APP] Received %d bytes: ", n);
            for (int i = 0; i < n; i++)
                putchar(buffer[i]);
            printf("\n");
        } else {
            fprintf(stderr, "[APP] Error or timeout in llread.\n");
        }
    }

    llclose();
    printf("[APP] Connection closed.\n");
}

