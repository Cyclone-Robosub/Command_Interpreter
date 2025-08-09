#ifndef MOCK_RPI

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <cstdint>
#include <string>

int serialOpen(const char *device, const int baud);
void serialPuts(const int fd, const char *s);
int serialGetchar (const int fd);
void echoOn(int serial);
bool initializeSerial(int *serial, bool testing);
int getSerialChar(int *serial, bool testing);

#endif