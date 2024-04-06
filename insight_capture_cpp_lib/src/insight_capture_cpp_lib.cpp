#include <insight_capture_cpp_lib/insight_capture_cpp_lib.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <stdexcept>


bool string2bytes(int size, char* src, char* dst)
{
    char tmp[3] = "  ";
    uint8_t val = 0u;
    int j = 0;    

    for(int i=0; i<size; i+=2)
    {
        // copy temporary buffer from 2 bytes
        memcpy(tmp, src+i, 2);

        // convert HEX string to integrer
        val = (uint8_t)std::stoi(tmp, nullptr, 16);

        // copy to result 
        memcpy(dst+j, &val, 1);
        j++;
    }

    return true;
}

bool readHeader(char* data, char* type, int* size, int* offset)
{
    memcpy(type, data, 2);
    memcpy(size, data+2, 4);
    memcpy(offset, data+10, 4);

    return true;
}

bool readInformation(char* data, int* width, int* height)
{
    memcpy(width, data+4, 4);
    memcpy(height, data+8, 4);

    return true;
}