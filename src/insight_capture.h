#ifndef __INSIGHT_CAPTURE_H__
#define __INSIGHT_CAPTURE_H__


extern "C" {

bool string2bytes(int size, char* src, char* dst);
bool readHeader(char* data, char* type, int* size, int* offset);
bool readInformation(char* data, int* width, int* height);

}

#endif