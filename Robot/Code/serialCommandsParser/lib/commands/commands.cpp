#include "commands.h"

#include <stdio.h>

int32_t example1(int32_t unused, int32_t unused_, int32_t unused__)
{
    if(unused || unused_ || unused__)
    {
        return (int32_t) TOO_MANY_ARGS;
    }
    printf("[Executed] Example 1\n");
    return PARSER_OK;
}

int32_t example2(int32_t x, int32_t y, int32_t theta)
{
    printf("[Executed] Example 2 (args : %d, %d, %d)\n", x, y, theta);
    return PARSER_OK;
}