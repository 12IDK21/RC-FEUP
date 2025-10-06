// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <cstddef>

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
    if (openSerialPort(serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }
    printf("Serial port %s opened\n", serialPort);
    if(connectionParameters.role==LlTx){
        writeEstablishment(connectionParameters);
        readEstablishment(connectionParameters);
    }
    else{
        readEstablishment(connectionParameters);
        writeEstablishment(connectionParameters);
    }
    
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

int writeEstablishment(LinkLayer connectionParameters){
    if(connectionParameters.role==LlTx){
        unsigned char SET[BUF_SIZE];

        SET[0] = FLAG;

        SET[1] = A_1; 

        SET[2] = C_Set;

        SET[3] = A_1 ^ C_Set;

        SET[4] = FLAG;

        int bytes = writeBytesSerialPort(SET, 5);
        if(bytes==-1) return -1
        return 1
    }    
    else
        unsigned char UA[BUF_SIZE];

        UA[0] = FLAG;
        UA[1] = A_3;
        UA[2] = A_3;
        UA[3] = A_3^C_UA;
        UA[4] = FLAG;

        int bytes = writeBytesSerialPort(UA,5);
        if(bytes==-1) return -1
        return 1
    
}

int readEstablishment(LinkLayer connectionParameters){
    if(connectionParameters.role==LlTx){
        while (STOP == FALSE)
        {   
            unsigned char byte;
            int bytes = readByteSerialPort(&byte);
            nBytesBuf += bytes;
            byte_counter++;

            printf("byte = 0x%02X\n", byte);
            if(validateByteUA(byte,byte_counter)==0) exit(-1);
            if (byte_counter == 5) 
                STOP = TRUE;
        }   
        if (STOP==TRUE)
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
            if(validateByteSET(byte,byte_counter)==0) exit(-1);
            if (byte_counter == 5) 
                STOP = TRUE;
        }   
        if (STOP==TRUE)
        {
            printf("UA bytes received: %d\n", nBytesBuf);
        }
}

int validateByteUA(unsigned char byte,int byte_counter){
    switch (byte_counter)
    {
    case 0:
        if(byte == FLAG) return 1;
        break;
    case 1:
        if(byte == A_3) return 1;
        break;
    case 2:
        if(byte == C_UA) return 1;
        break;
    case 3:
        if(byte == A_3^C_UA) return 1;
        break;
    case 4:
        if(byte == FLAG) return 1;
        break;
    default:
        return 0;
        break;
    }
}

int validateByteSET(unsigned char byte,int byte_counter){
    switch (byte_counter)
    {
    case 0:
        if(byte == FLAG) return 1;
        break;
    case 1:
        if(byte == A_1) return 1;
        break;
    case 2:
        if(byte != C_UA) return 1;
        break;
    case 3:
        if(byte != A_1^C_UA) return 1;
        break;
    case 4:
        if(byte != FLAG) return 1;
        break;
    default:
        return 0;
        break;
    }
}
