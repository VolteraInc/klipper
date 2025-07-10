// File: debug_utilities.h
// #################################################################
// Define global debug level (0 = no debug, 1 = error, 2 = warning, 3 = info, 4 = verbose)
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 0
#endif

// Debug level enumeration
#define DEBUG_NONE    0  
#define DEBUG_ERROR   1  
#define DEBUG_WARNING 2  
#define DEBUG_INFO    3  
#define DEBUG_VERBOSE 4  

// Debug macros with keywords
#if DEBUG_LEVEL >= DEBUG_ERROR
    #define DBG_ERR(fmt, ...) \
        output("ERROR: " fmt, ##__VA_ARGS__)
#else
    #define DBG_ERR(fmt, ...) 
#endif

#if DEBUG_LEVEL >= DEBUG_WARNING
    #define DBG_WARN(fmt, ...) \
        output("WARN: " fmt, ##__VA_ARGS__)
#else
    #define DBG_WARN(fmt, ...) 
#endif

#if DEBUG_LEVEL >= DEBUG_INFO
    #define DBG_INFO(fmt, ...) \
        output("INFO: " fmt, ##__VA_ARGS__)
#else
    #define DBG_INFO(fmt, ...) 
#endif

#if DEBUG_LEVEL >= DEBUG_VERBOSE
    #define DBG_VERB(fmt, ...) \
        output("DEBUG: " fmt, ##__VA_ARGS__)
#else
    #define DBG_VERB(fmt, ...) 
#endif


