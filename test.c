/******************************************************************************

                          Testing C code file.
Write your own code in this file to test anything you want

*******************************************************************************/


#include <stdio.h>
#include <string.h>

typedef unsigned char uint8_t;

int append_string_to_buffer(uint8_t *buf, const char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        buf[i] = (uint8_t)str[i];
        ++i;
    }
    return i;
}

int main()
{
    uint8_t TxBuffer[50] = {0};
    int i = 0;
    int j = 0;
    j += append_string_to_buffer(TxBuffer, "SVINFO,1,");
    j += append_string_to_buffer(&TxBuffer[j], "Y,");
    
    for (i = 0; i < 50; ++i)
    {
        printf("%c", TxBuffer[i]);
    }
    printf("\n%s: %d\n\n", TxBuffer, j);

    return 0;
}

