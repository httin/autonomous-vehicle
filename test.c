/******************************************************************************

                          Testing C code file.
Write your own code in this file to test anything that you want to

*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef unsigned char uint8_t;
typedef unsigned long int uint32_t;

void my_itoa(int i, char **b)
{
    char const digit[] = "0123456789";
    char* p = *b;

    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        ++(*b);
        shifter = shifter/10;
    }while(shifter);
    
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
}

char *dtostr(double value, char *out_str, int precision)
{
    int i, adding_zero = 0, meet_minus_flag = 0;
    int num_before_dot, num_after_dot;
    char *p = out_str;
    
    if (value < 0)
    {
        value = -value;
        *p++ = '-';
        meet_minus_flag = 1;
    }

    num_before_dot = (int)value;
    value = value - num_before_dot;
    for (i = 0; i < precision; ++i)
    {
        value *= 10;
        adding_zero += !((int)value);
    }
    num_after_dot = (int)value;

    if (num_after_dot == 0) // see @value as if it is an integer
    {
        if (num_before_dot == 0) {
            if(meet_minus_flag) {
                *--p = '0';
            } else {
                *p = '0';
            }
        } else {
            my_itoa(num_before_dot, &p);
        }
    } 
    else // @value is actually double
    {
        if (num_before_dot == 0) { // example: @value = 0.123
            *p++ = '0';
            *p++ = '.';
            for (i = 0; i < adding_zero; ++i)
                *p++ = '0';
            my_itoa(num_after_dot, &p);
        } else {    // example: @value = 123.123
            my_itoa(num_before_dot, &p);
            *p++ = '.';
            for (i = 0; i < adding_zero; ++i)
                *p++ = '0';
            my_itoa(num_after_dot, &p);
        }
    }
    
    return out_str;
}

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

void TEST_append_string_to_buffer(void) {
    uint8_t TxBuffer[50] = {0};
    int i = 0;
    int j = 0;
    j += append_string_to_buffer(TxBuffer, "SVINFO,1,");
    j += append_string_to_buffer(&TxBuffer[j], "Y,");
    for (i = 0; i < 50; ++i)
        printf("%c", TxBuffer[i]);
    printf("\n%s: %d\n\n", TxBuffer, j);
}

void TEST_dtostr(void) {
    char buf[20] = {0};
    double test_val = -10.132329;
    char *str = dtostr(test_val, buf, 4);
    printf("%lf %s %s\n", test_val, buf, str);
}

int main(int argc, char **argv)
{
    TEST_dtostr();

    return 0;
}

