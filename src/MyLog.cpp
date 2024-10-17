// MyLog.cpp
#include "MyLog.h"

void MyLog::error(const char *format, ...) {
  if (LOG_LEVEL >= LOG_LEVEL_ERROR) {
    va_list args;
    va_start(args, format);
    logPrint("ERROR", format, args);
    va_end(args);
  }
}

void MyLog::warn(const char *format, ...) {
  if (LOG_LEVEL >= LOG_LEVEL_WARN) {
    va_list args;
    va_start(args, format);
    logPrint("WARN", format, args);
    va_end(args);
  }
}

void MyLog::info(const char *format, ...) {
  if (LOG_LEVEL >= LOG_LEVEL_INFO) {
    va_list args;
    va_start(args, format);
    logPrint("INFO", format, args);
    va_end(args);
  }
}

void MyLog::debug(const char *format, ...) {
  if (LOG_LEVEL >= LOG_LEVEL_DEBUG) {
    va_list args;
    va_start(args, format);
    logPrint("DEBUG", format, args);
    va_end(args);
  }
}

void MyLog::logPrint(const char *level, const char *format, va_list args) {
  Serial.printf("[%s] ", level);
  char buffer[256];
  vsnprintf(buffer, sizeof(buffer), format, args);
  Serial.print(buffer);
  Serial.println();
}
