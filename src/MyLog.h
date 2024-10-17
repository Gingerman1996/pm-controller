// MyLog.h
#ifndef MY_LOG_H
#define MY_LOG_H

#include <Arduino.h>

// ระดับของ log
#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

// กำหนดระดับ log ที่ต้องการใช้งาน
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG
#endif

class MyLog {
public:
    static void error(const char *format, ...);
    static void warn(const char *format, ...);
    static void info(const char *format, ...);
    static void debug(const char *format, ...);

private:
    static void logPrint(const char *level, const char *format, va_list args);
};

#endif // MY_LOG_H
