#include "insight_capture.h"
#include <cstdio>
#include <string>


int main(int argc, char** argv)
{
    char src[] = "424D";
    char dst[] = "    ";

    bool res = string2bytes(sizeof(src)-1, src, dst);

    printf("%d\n%s\n%s\n", res, src, dst);
}