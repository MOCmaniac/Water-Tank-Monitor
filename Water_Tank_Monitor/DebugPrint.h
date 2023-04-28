// Debug serial option

//#define DEBUG // Comment this line to turn debug prints off
#ifdef DEBUG
#define DBG_PRINT(args) Serial.print(args)
#define DBG_PRINTLN(args) Serial.println(args)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__) // multiple arguments function
#define SERIAL_BEGIN(args) Serial.begin(args)
#else
#define DBG_PRINT(args)
#define DBG_PRINTLN(args)
#define DBG_PRINTF(...)
#define SERIAL_BEGIN(args)
#endif
