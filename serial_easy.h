/***************************************************************************
                        serial.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : open/close a serial port
 ***************************************************************************/

#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>

#define SERIAL_WRITE(tty,buf,size) (write(tty,buf,size))
#define SERIAL_READ(tty,buf,size)  (read(tty,buf,size))
#define SERIAL_FLUSH_RX(tty)       (tcflush (tty, TCIFLUSH))
#define SERIAL_FLUSH_TX(tty)       (tcflush (tty, TCOFLUSH))

int BaudrateToBitPerSec(int baudrate);
unsigned int ComputeTransTime(unsigned int bytes, int baudrate);
int OpenSerial (char *device, int baudrate, int timeout, int wait_bytes);
int CloseSerial(int ttyVG);

#endif
