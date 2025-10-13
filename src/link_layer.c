// Link layer protocol implementation

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "link_layer.h"
#include "serial_port.h"


#define FALSE 0
#define TRUE 1

#define BAUDRATE 38400
#define BUF_SIZE 256

#define FLAG 0x7E
#define A_1 0x03
#define A_3 0x01

#define C_Set 0x03
#define C_UA 0x07

typedef enum {
    ST_START = 0,
    ST_FLAG_RCV,
    ST_A_RCV,
    ST_C_RCV,
    ST_BCC_OK,
} RxState;

static volatile sig_atomic_t g_timed_out = 0;

static void alarm_handler(int sig);
static int send_set(void);
static int send_ua(void);
static int send_frame(const unsigned char *buf, int bufSize);
static int receive_frame(void);
static int stateMachineEstablishment(unsigned char Aexintp, unsigned char Cexp, int timeout_s);
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        return -1;
    }
    printf("Serial port %s opened\n", connectionParameters.serialPort);
    if (connectionParameters.role == LlTx) {
        for (int attempt = 1; attempt <= connectionParameters.nRetransmissions; ++attempt) {
            if (send_set() < 0) { perror("[TX] SET not sent"); return -1; }
            printf("[TX] SET sent (try %d/%d), waiting UA (%ds)\n",
                   attempt, connectionParameters.nRetransmissions, connectionParameters.timeout);

            int received = stateMachineEstablishment(A_3, C_UA, connectionParameters.timeout);
            if (received == 1) { printf("[TX] UA recieved\n"); return 0; }
            printf("[TX] Timeout waiting UA\n");
        }
        fprintf(stderr, "[TX] Fail: exceeded retransmissions.\n");
        return -1;

    } else {
        printf("[RX] waiting SET (%ds)...\n", connectionParameters.timeout);
        int received = stateMachineEstablishment(A_1, C_Set, connectionParameters.timeout);
        if (received == 1) {
            printf("[RX] SET received. Sending UA\n");
            if (send_ua() < 0) { perror("[RX] UA not sent"); return -1; }
            return 0;
        } else if (received == 0) {
            fprintf(stderr, "[RX] Timeout waiting SET\n");
            return -1;
        } else {
            perror("[RX] reading SET");
            return -1;
        }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    send_frame(buf,bufSize);
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{   
    int n = receive_frame();
    printf("%d\n",n);
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}

static int send_set(void) {
    unsigned char SET[BUF_SIZE] = { FLAG, A_1, C_Set, (unsigned char)(A_1 ^ C_Set), FLAG };
    int n = writeBytesSerialPort(SET, 5);
    return (n == 5) ? 0 : -1;
}

static int send_ua(void) {
    unsigned char UA[BUF_SIZE] = { FLAG, A_3, C_UA, (unsigned char)(A_3 ^ C_UA), FLAG };
    int n = writeBytesSerialPort(UA, 5);
    return (n == 5) ? 0 : -1;
}

static int send_frame(const unsigned char *buf, int bufSize) {
    int n = writeBytesSerialPort(buf, bufSize);
    return (n == bufSize) ? 0 : -1;
}

static int receive_frame(void) {
    unsigned char byte;
    int n = readByteSerialPort(&byte);
    printf("byte : %c)...\n", byte);
    return n;
}

static int stateMachineEstablishment(unsigned char A, unsigned char C, int timeout_s)
{
    RxState st = ST_START;
    g_timed_out = 0;
    signal(SIGALRM, alarm_handler);
    alarm(timeout_s);

    while (TRUE) {
        if (g_timed_out) { alarm(0); return 0; }

        unsigned char b = 0;
        int received = readByteSerialPort(&b);
        if (received < 0) { alarm(0); return -1; }
        if (received == 0) {
            continue;                           
        }

        switch (st) {
            case ST_START:
                if (b == FLAG) st = ST_FLAG_RCV;
                break;

            case ST_FLAG_RCV:
                if (b == FLAG) {}
                else if (b == A) st = ST_A_RCV;
                else st = ST_START;
                break;

            case ST_A_RCV:
                if (b == FLAG) st = ST_FLAG_RCV;
                else if (b == C) st = ST_C_RCV;
                else st = ST_START;
                break;

            case ST_C_RCV: {
                unsigned char bcc_ok = (unsigned char)(A ^ C);
                if (b == FLAG) st = ST_FLAG_RCV;
                else if (b == bcc_ok) st = ST_BCC_OK;
                else st = ST_START;
            } break;

            case ST_BCC_OK:
                if (b == FLAG) { alarm(0); return 1; }
                else st = ST_START;
                break;
        }
    }
}

static void alarm_handler(int sig) {
    (void)sig;
    g_timed_out = 1;
}
