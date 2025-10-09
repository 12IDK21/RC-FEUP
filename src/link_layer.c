// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define BAUDRATE 38400
#define BUF_SIZE 256

#define FLAG 0x7E
#define A_1 0x03
#define A_3 0x01

#define C_Set 0x03
#define C_UA 0x07

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }
    printf("Serial port %s opened\n", connectionParameters.serialPort);
    int wrote;
    int read;
    if (connectionParameters.role == LlTx)
    {
        wrote = writeEstablishment(connectionParameters);
        read = readEstablishment(connectionParameters);
    }
    else
    {
        read = readEstablishment(connectionParameters);
        wrote = writeEstablishment(connectionParameters);
    }

    if (wrote != 1 || read != 1)
    {
        perror("Communication start error");
        exit(-1);
    }

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

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

int writeEstablishment(LinkLayer connectionParameters)
{
    if (connectionParameters.role == LlTx)
    {
        unsigned char SET[BUF_SIZE];

        SET[0] = FLAG;

        SET[1] = A_1;

        SET[2] = C_Set;

        SET[3] = A_1 ^ C_Set;

        SET[4] = FLAG;

        int bytes = writeBytesSerialPort(SET, 5);
        return (bytes == 5) ? 1 : -1;
    }
    else
    {
        unsigned char UA[BUF_SIZE];

        UA[0] = FLAG;
        UA[1] = A_3;
        UA[2] = C_UA;
        UA[3] = A_3 ^ C_UA;
        UA[4] = FLAG;

        int bytes = writeBytesSerialPort(UA, 5);
        return (bytes == 5) ? 1 : -1;
    }
}

int readEstablishment(LinkLayer connectionParameters)
{
    int STOP = FALSE;
    int nBytesBuf = 0;
    volatile int byte_counter = 0;
    if (connectionParameters.role == LlTx)
    {
        while (STOP == FALSE)
        {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            nBytesBuf += bytes;
            byte_counter++;

            printf("byte = 0x%02X\n", byte);
            if (!validateByteUA(byte, byte_counter))
                exit(-1);
            if (byte_counter == 5)
                STOP = TRUE;
        }
        if (STOP == TRUE)
        {
            printf("UA bytes received: %d\n", nBytesBuf);
            return 1;
        }
    }
    else
        while (STOP == FALSE)
        {
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            nBytesBuf += bytes;
            byte_counter++;

            printf("byte = 0x%02X\n", byte);
            if (validateByteSET(byte, byte_counter) == 0)
                exit(-1);
            if (byte_counter == 5)
                STOP = TRUE;
        }
    if (STOP == TRUE)
    {
        printf("UA bytes received: %d\n", nBytesBuf);
    }
}

int validateByteUA(unsigned char byte, int idx)
{
    switch (idx)
    {
    case 1:
        return (byte == FLAG);
    case 2:
        return (byte == A_3);
    case 3:
        return (byte == C_UA);
    case 4:
        return (byte == (unsigned char)(A_3 ^ C_UA));
    case 5:
        return (byte == FLAG);
    default:
        return 0;
    }
}

int validateByteSET(unsigned char byte, int idx)
{
    switch (idx)
    {
    case 1:
        return (byte == FLAG);
    case 2:
        return (byte == A_1);
    case 3:
        return (byte == C_Set);
    case 4:
        return (byte == (unsigned char)(A_1 ^ C_Set));
    case 5:
        return (byte == FLAG);
    default:
        return 0;
    }
}
