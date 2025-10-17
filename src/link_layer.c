// Link layer protocol implementation

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
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

#define ESC 0x7D
#define ESC_XOR 0x20

#define C_I(ns) ((unsigned char)((ns) ? 0x80 : 0x00))
#define C_RR(r) ((unsigned char)((r) ? 0xAB : 0xAA))
#define C_REJ(r) ((unsigned char)((r) ? 0x55 : 0x54))

typedef enum
{
    ST_START = 0,
    ST_FLAG_RCV,
    ST_A_RCV,
    ST_C_RCV,
    ST_BCC_OK,
} RxState;

static LinkLayer g_ll;
static unsigned char g_ns = 0;
static volatile sig_atomic_t g_timed_out = 0;

static void alarm_handler(int sig);
static void set_alarm_handler(void);
static int send_set(void);
static int send_ua(void);
static int stateMachineEstablishment(unsigned char Aexintp, unsigned char Cexp, int timeout_s);
static int wait_supervision_resp(unsigned char ns, int timeout_s);
static unsigned char compute_bcc2(const unsigned char *data, int len);
static int stuff_ppp(const unsigned char *in, int inLen, unsigned char *out, int outMax);
static int destuff_ppp(const unsigned char *in, int inLen, unsigned char *out, int outMax);
static int build_i_frame(unsigned char *out, int outMax, const unsigned char *payload, int payloadLen, unsigned char ns);
static int get_frame(unsigned char *frame, int maxLen, int timeout_s);
static int frame_check(const unsigned char *frame, int frameLen);
static int bcc2_check(const unsigned char *stuffed, int stuffedLen, unsigned char *outData, int outMax);
static int send_rr(unsigned char r);
static int send_rej(unsigned char r);
static int read_byte_with_timeout(unsigned char *b, int timeout_s);

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
    {
        perror("openSerialPort");
        return -1;
    }
    g_ll = connectionParameters;
    g_ns = 0;
    printf("Serial port %s opened\n", connectionParameters.serialPort);
    if (connectionParameters.role == LlTx)
    {
        for (int attempt = 1; attempt <= connectionParameters.nRetransmissions; ++attempt)
        {
            if (send_set() < 0)
            {
                perror("[TX] SET not sent");
                return -1;
            }
            printf("[TX] SET sent (try %d/%d), waiting UA (%ds)\n",
                   attempt, connectionParameters.nRetransmissions, connectionParameters.timeout);

            int received = stateMachineEstablishment(A_3, C_UA, connectionParameters.timeout);
            if (received == 1)
            {
                printf("[TX] UA recieved\n");
                return 0;
            }
            printf("[TX] Timeout waiting UA\n");
        }
        fprintf(stderr, "[TX] Fail: exceeded retransmissions.\n");
        return -1;
    }
    else
    {
        printf("[RX] waiting SET (%ds)...\n", connectionParameters.timeout);
        int received = stateMachineEstablishment(A_1, C_Set, connectionParameters.timeout);
        if (received == 1)
        {
            printf("[RX] SET received. Sending UA\n");
            if (send_ua() < 0)
            {
                perror("[RX] UA not sent");
                return -1;
            }
            return 0;
        }
        else if (received == 0)
        {
            fprintf(stderr, "[RX] Timeout waiting SET\n");
            return -1;
        }
        else
        {
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
    if (bufSize < 0 || bufSize > BUF_SIZE)
        return -1;
    unsigned char frame[BUF_SIZE * 2 + 16];
    int frameLen = build_i_frame(frame, sizeof(frame), buf, bufSize, g_ns);
    if (frameLen < 0)
    {
        fprintf(stderr, "[TX] build_i_frame failed\n");
        return -1;
    }
    for (int attempt = 1; attempt <= g_ll.nRetransmissions; ++attempt)
    {
        int sent = writeBytesSerialPort(frame, frameLen);
        if (sent != frameLen)
        {
            perror("[TX] write I frame");
            return -1;
        }
        printf("[TX] I(Ns=%u) sent (try %d/%d), waiting RR/REJ (%ds)\n",
               g_ns, attempt, g_ll.nRetransmissions, g_ll.timeout);

        int resp = wait_supervision_resp(g_ns, g_ll.timeout);
        if (resp == 1)
        {
            printf("[TX] RR ok. Advancing Ns.\n");
            g_ns ^= 0x01;
            return bufSize;
        }
        else if (resp == 0)
        {
            printf("[TX] REJ received. Retransmitting same I(Ns=%u)...\n", g_ns);
            continue;
        }
        else if (resp == -2)
        {
            printf("[TX] Timeout waiting RR/REJ. Retransmitting...\n");
            continue;
        }
        else
        {
            fprintf(stderr, "[TX] Unexpected response (%d). Retransmitting...\n", resp);
            continue;
        }
    }
    fprintf(stderr, "[TX] Fail: exceeded retransmissions in llwrite.\n");
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char frame[BUF_SIZE * 2 + 16];
    int flen = get_frame(frame, sizeof(frame), g_ll.timeout);
    if (flen == 0)
    {
        return 0;
    }
    if (flen < 0)
    {
        perror("[RX] rx_capture_frame");
        return -1;
    }

    if (frame_check(frame, flen) < 0)
    {
        return -1;
    }
    unsigned char C = frame[2];
    unsigned char Ns = (C == C_I(1)) ? 1 : 0;
    const unsigned char *stuffed = &frame[4];
    int stuffedLen = flen - 5;
    int payloadLen = bcc2_check(stuffed, stuffedLen, packet, BUF_SIZE);
    if (payloadLen < 0)
    {
        (void)send_rej(g_ns);
        fprintf(stderr, "[RX] BCC2 error (%d). Sent REJ(r=%u)\n", payloadLen, g_ns);
        return -1;
    }
    if (Ns == g_ns)
    {
        int r = (g_ns ^ 1) & 0x01;
        (void)send_rr(r);
        g_ns = r;
        return payloadLen;
    }
    (void)send_rr(g_ns);
    fprintf(stderr, "[RX] Duplicate I(Ns=%u). Sent RR(r=%u). Payload dropped.\n", Ns, g_ns);
    return -3;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}

// Establishment
static int send_set(void)
{
    unsigned char SET[] = {FLAG, A_1, C_Set, (unsigned char)(A_1 ^ C_Set), FLAG};
    int n = writeBytesSerialPort(SET, 5);
    return (n == 5) ? 0 : -1;
}

static int send_ua(void)
{
    unsigned char UA[BUF_SIZE] = {FLAG, A_3, C_UA, (unsigned char)(A_3 ^ C_UA), FLAG};
    int n = writeBytesSerialPort(UA, 5);
    return (n == 5) ? 0 : -1;
}

// State machines
static int stateMachineEstablishment(unsigned char A, unsigned char C, int timeout_s)
{
    RxState st = ST_START;
    g_timed_out = 0;
    set_alarm_handler();
    alarm(timeout_s);

    while (TRUE)
    {
        if (g_timed_out)
        {
            alarm(0);
            return 0;
        }

        unsigned char b = 0;
        int received = read_byte_with_timeout(&b, timeout_s);
        if (received == 0)
        {
            alarm(0);
            return 0;
        }
        if (received < 0)
        {
            alarm(0);
            return -1;
        }

        switch (st)
        {
        case ST_START:
            if (b == FLAG)
                st = ST_FLAG_RCV;
            break;

        case ST_FLAG_RCV:
            if (b == FLAG)
            {
            }
            else if (b == A)
                st = ST_A_RCV;
            else
                st = ST_START;
            break;

        case ST_A_RCV:
            if (b == FLAG)
                st = ST_FLAG_RCV;
            else if (b == C)
                st = ST_C_RCV;
            else
                st = ST_START;
            break;

        case ST_C_RCV:
        {
            unsigned char bcc_ok = (unsigned char)(A ^ C);
            if (b == FLAG)
                st = ST_FLAG_RCV;
            else if (b == bcc_ok)
                st = ST_BCC_OK;
            else
                st = ST_START;
        }
        break;

        case ST_BCC_OK:
            if (b == FLAG)
            {
                alarm(0);
                return 1;
            }
            else
                st = ST_START;
            break;
        }
    }
}

static int wait_supervision_resp(unsigned char ns, int timeout_s)
{
    RxState st = ST_START;
    unsigned char A = 0, C = 0;
    g_timed_out = 0;
    set_alarm_handler();
    alarm(timeout_s);

    while (TRUE)
    {
         if (g_timed_out)
        {
            alarm(0);
            return 0;
        }

        unsigned char b = 0;
        int received = read_byte_with_timeout(&b, timeout_s);
        if (received == 0)
        {
            alarm(0);
            return 0;
        }
        if (received < 0)
        {
            alarm(0);
            return -1;
        }

        switch (st)
        {
        case ST_START:
            if (b == FLAG)
                st = ST_FLAG_RCV;
            break;

        case ST_FLAG_RCV:
            if (b == FLAG)
            {
            }
            else if (b == A_3)
            {
                A = b;
                st = ST_A_RCV;
            }
            else
                st = ST_START;
            break;

        case ST_A_RCV:
            if (b == FLAG)
                st = ST_FLAG_RCV;
            else
            {
                C = b;
                st = ST_C_RCV;
            }
            break;

        case ST_C_RCV:
        {
            unsigned char bcc_ok = (unsigned char)(A ^ C);
            if (b == FLAG)
                st = ST_FLAG_RCV;
            else if (b == bcc_ok)
                st = ST_BCC_OK;
            else
                st = ST_START;
        }
        break;

        case ST_BCC_OK:
            if (b == FLAG)
            {
                alarm(0);

                if (C == C_RR(0) || C == C_RR(1))
                {
                    unsigned char r = (C == C_RR(0)) ? 0 : 1;
                    if (r == ((ns ^ 1) & 0x01))
                        return 1;
                    else
                        return -4;
                }
                else if (C == C_REJ(0) || C == C_REJ(1))
                {
                    unsigned char rejNs = (C == C_REJ(0)) ? 0 : 1;
                    if (rejNs == ns)
                        return 0;
                    else
                        return -5;
                }
                else
                {
                    return -6;
                }
            }
            else
            {
                st = ST_START;
            }
            break;
        }
    }
}

static int read_byte_with_timeout(unsigned char *b, int timeout_s)
{
    g_timed_out = 0;
    set_alarm_handler();
    alarm(timeout_s);

    while (TRUE)
    {
        if (g_timed_out)
        {
            alarm(0);
            return 0;
        }

        int received = readByteSerialPort(b);
        if (received < 0)
        {
            alarm(0);
            return -1;
        }
        if (received == 0)
        {
            continue;
        }
        alarm(0);
        return 1;
    }
    return -1;
}

static void set_alarm_handler(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = alarm_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGALRM, &sa, NULL);
}

static void alarm_handler(int sig)
{
    (void)sig;
    g_timed_out = 1;
}

static unsigned char compute_bcc2(const unsigned char *data, int len)
{
    unsigned char bcc2 = 0x00;
    for (size_t i = 0; i < len; i++)
    {
        bcc2 ^= data[i];
    }
    return bcc2;
}

static int stuff_ppp(const unsigned char *in, int inLen, unsigned char *out, int outMax)
{
    int j = 0;
    for (int i = 0; i < inLen; i++)
    {
        unsigned char d = in[i];
        if (d == FLAG || d == ESC)
        {
            if (j + 2 > outMax)
                return -1;
            out[j++] = ESC;
            out[j++] = (unsigned char)(d ^ ESC_XOR);
        }
        else
        {
            if (j + 1 > outMax)
                return -1;
            out[j++] = d;
        }
    }
    return j;
}

static int build_i_frame(unsigned char *out, int outMax,
                         const unsigned char *payload, int payloadLen,
                         unsigned char ns)
{
    if (payloadLen < 0 || payloadLen > BUF_SIZE)
        return -1;

    const unsigned char A = A_1;
    const unsigned char C = C_I(ns);
    const unsigned char BCC1 = (unsigned char)(A ^ C);

    unsigned char bcc2 = compute_bcc2(payload, payloadLen);
    unsigned char tmp[BUF_SIZE + 1];
    for (int i = 0; i < payloadLen; ++i)
        tmp[i] = payload[i];
    tmp[payloadLen] = bcc2;

    unsigned char stuffed[(BUF_SIZE + 1) * 2];
    int sLen = stuff_ppp(tmp, payloadLen + 1, stuffed, sizeof(stuffed));
    if (sLen < 0)
        return -1;

    int needed = 1 + 1 + 1 + 1 + sLen + 1;
    if (needed > outMax)
        return -1;

    int k = 0;
    out[k++] = FLAG;
    out[k++] = A;
    out[k++] = C;
    out[k++] = BCC1;
    for (int i = 0; i < sLen; ++i)
        out[k++] = stuffed[i];
    out[k++] = FLAG;

    return k;
}

// Computations llread()

static int send_rr(unsigned char r)
{
    unsigned char out[BUF_SIZE];
    const unsigned char A = A_3;
    const unsigned char C = C_RR(r);
    const unsigned char BCC1 = (unsigned char)(A ^ C);

    out[0] = FLAG;
    out[1] = A;
    out[2] = C;
    out[3] = BCC1;
    out[4] = FLAG;
    int nbytes = writeBytesSerialPort(out, 5);
    if (nbytes == 5)
        return 0;
    return -1;
}

static int send_rej(unsigned char r)
{
    unsigned char out[BUF_SIZE];
    const unsigned char A = A_3;
    const unsigned char C = C_REJ(r);
    const unsigned char BCC1 = (unsigned char)(A ^ C);

    out[0] = FLAG;
    out[1] = A;
    out[2] = C;
    out[3] = BCC1;
    out[4] = FLAG;
    int nbytes = writeBytesSerialPort(out, 5);
    if (nbytes == 5)
        return 0;
    return -1;
}

static int get_frame(unsigned char *frame, int maxLen, int timeout_s)
{
    int started = 0;
    int k = 0;
    unsigned char byte;

    while (TRUE)
    {
        int read = read_byte_with_timeout(&byte, timeout_s);
        if (read == 0)
            return 0;
        if (read < 0)
            return -1;
        if (!started)
        {
            if (byte == FLAG)
            {
                if (k >= maxLen)
                    return -1;
                frame[k++] = FLAG;
                started = 1;
            }
            continue;
        }
        if (k == 1 && byte == FLAG)
        {
            continue;
        }
        if (k >= maxLen)
            return -1;
        frame[k++] = byte;
        if (byte == FLAG && k > 1)
        {
            return k;
        }
    }
}

static int frame_check(const unsigned char *frame, int frameLen)
{
    if (frameLen < 5)
    {
        fprintf(stderr, "[RX] Frame too short for header\n");
        return -1;
    }
    if (frame[0] != FLAG || frame[frameLen - 1] != FLAG)
    {
        fprintf(stderr, "[RX] Missing frame delimiters\n");
        return -1;
    }
    unsigned char A = frame[1];
    unsigned char C = frame[2];
    unsigned char BCC1 = frame[3];
    if (BCC1 != (unsigned char)(A ^ C))
    {
        fprintf(stderr, "[RX] BCC1 mismatch \n");
        return -1;
    }
    if (!(C == C_I(0) || C == C_I(1)))
    {
        fprintf(stderr, "[RX] Not an I-frame \n");
        return -1;
    }
    return 0;
}
static int bcc2_check(const unsigned char *stuffed, int stuffedLen, unsigned char *outData, int outMax)
{
    if (stuffedLen <= 0)
        return -1;
    unsigned char tmp[BUF_SIZE + 1];
    int unLen = destuff_ppp(stuffed, stuffedLen, tmp, sizeof(tmp));
    if (unLen < 0)
        return -2;
    if (unLen < 1)
        return -3;
    unsigned char recv_bcc2 = tmp[unLen - 1];
    int payloadLen = unLen - 1;
    unsigned char calc_bcc2 = compute_bcc2(tmp, payloadLen);
    if (calc_bcc2 != recv_bcc2)
        return -4;
    if (payloadLen > outMax)
        return -5;
    memcpy(outData, tmp, payloadLen);
    return payloadLen;
}
static int destuff_ppp(const unsigned char *in, int inLen, unsigned char *out, int outMax)
{
    int j = 0;
    for (int i = 0; i < inLen; i++)
    {
        unsigned char d = in[i];
        if (d == ESC)
        {
            if (i + 1 >= inLen)
                return -1;
            unsigned char y = (unsigned char)(in[i + 1] ^ ESC_XOR);
            if (j + 1 > outMax)
                return -1;
            out[j++] = y;
            i++;
        }
        else
        {
            if (j + 1 > outMax)
                return -1;
            out[j++] = d;
        }
    }
    return j;
}
