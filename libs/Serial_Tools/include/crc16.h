#ifndef CRC16_HPP
#define CRC16_HPP

#include <stdint.h>
#include <string.h>
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

inline uint16_t fletcher16(unsigned char* buf, size_t buflen)
{
    //Compute Fletcher16 CRC
    uint8_t chksm0 = 0, chksm1 = 0;
    for(unsigned i = 0; i < buflen; ++i)
    {
        chksm0 += buf[i];
        chksm1 += chksm0;
    }
    
    return (uint16_t) ((chksm1 << 8) | chksm0);
}
/*
inline uint16_t fletcher16_append(char* buf, size_t buflen, char* dest)
{
    //Compute Fletcher16 CRC
    uint16_t retVal = fletcher16(buf, buflen);

    //Copy the value to 'dest' 
    memcpy(dest, (char*)&retVal, 2);

    return retVal;
}
*/
inline uint16_t fletcher16_append(unsigned char* buf, size_t buflen, unsigned char* dest)
{
    //Compute Fletcher16 CRC
    uint16_t retVal = fletcher16(buf, buflen);

    //Copy the value to 'dest' 
    memcpy(dest, (unsigned char*) &retVal, 2);

    return retVal;
}
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //CRC16_HPP 