#ifndef __DEBUG_HPP__
#define __DEBUG_HPP__

#define DEBUG_ERROR
#define DEBUG_MSG_LEVEL1
// #define DEBUG_MSG_LEVEL2
// #define DEBUG_MSG_LEVEL3

#ifdef DEBUG_ERROR

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <libgen.h>

static void err_info(const char *file, const char *func, const int line)
{
    fprintf(stderr,"%s/%s#%s(%d) :: ", basename(dirname(strdup(file))), basename(strdup(file)), func, line);
    //fprintf(stderr,"[%s(%d)] ", func, line);
    fflush(stderr);
}

static void err_msg(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    
    vfprintf(stderr, format, args);
    fprintf(stderr, "\n");
    fflush(stderr);
    
    va_end(args);
}

#define DEBUG_E(x) { err_info(__FILE__, __FUNCTION__, __LINE__); err_msg x; }

#else
#define DEBUG_E(x) {}
#endif

// Debug Message level 1 active
#ifdef DEBUG_MSG_LEVEL1
#define DEBUG_1(x) { x }
#else
#define DEBUG_1(x) {}
#endif

#ifdef DEBUG_MSG_LEVEL2
#define DEBUG_2(x) { x }
#else
#define DEBUG_2(x) {}
#endif

#ifdef DEBUG_MSG_LEVEL3
#define DEBUG_3(x) { x }
#else
#define DEBUG_3(x) {}
#endif



#endif