
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <strings.h>

#include "Edebug.h"

int BaudrateToBitPerSec(int baudrate)
{
  switch(baudrate)
  {
    case B9600:  return 9600; break;
    case B19200: return 19200; break;
    case B38400: return 38400; break;
    case B57600: return 57600; break;
    case B115200: return 115200; break;
    case B230400: return 230400; break;
    default:
    {
      fprintf(stderr, "Internal error!\n");
      exit(EXIT_FAILURE);
    }
  }
}

unsigned int ComputeTransTime(unsigned int bytes, int baudrate)
{
  /* num of bytes in the packet * 10 bits per transmitted byte / bit per second */
  double time = (bytes * 10.0)/BaudrateToBitPerSec(baudrate);
  return (unsigned int)(1000000.0 * time);
}

/* -1 in case of error else the file descriptor timeout is in ms, set wait_bytes to zero if you want to read only one byte and use the timeout */
int OpenSerial (char *device, int baudrate, int timeout, int wait_bytes)
{
  struct termios newtio;
  char   error[255];

  int ttyVG = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (ttyVG < 0)
  {
    sprintf (error, "OpenSerial > %s", device);
    perror(error);
    return -1;
  }

  /* Blocking behavior for the read function */
  fcntl(ttyVG, F_SETFL, 0);

  /* Erase the termios structure */
  bzero(&newtio, sizeof(newtio));

  /* Set the baud rates */
  cfsetispeed(&newtio, baudrate);
  cfsetospeed(&newtio, baudrate);

  /* c_cflag : 8 bits, no parity, 1 stop bit, no hardware control, */

  newtio.c_cflag &= ~PARENB;
  newtio.c_cflag &= ~CSTOPB;
  newtio.c_cflag &= ~CSIZE;
  newtio.c_cflag |= CS8;
  newtio.c_cflag &= ~CRTSCTS;
  newtio.c_cflag |= CLOCAL;
  newtio.c_cflag |= CREAD;

  /* c_lflag : raw input, no echo non canonical */
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  /* c_iflag : no software flow control, no parity control */
  newtio.c_iflag &= ~(IXON | IXOFF | IXANY | IGNPAR);

  /* c_oflag : raw output */
  newtio.c_oflag &= ~OPOST;

  /* c_cc : timeouts */
  newtio.c_cc[VTIME] = timeout/100;
  EDBG("Port opened with a timeout of : %i ms and wait_bytes : %i", timeout, wait_bytes);
  newtio.c_cc[VMIN]   = wait_bytes;

  if(wait_bytes == 0) { 
    newtio.c_cc[VEOF]   = newtio.c_cc[VMIN];
    newtio.c_cc[VEOL]  = newtio.c_cc[VTIME];
  }

  /* Flush incoming and outgoing data and setup the port now */
  tcflush (ttyVG, TCIFLUSH);
  tcflush (ttyVG, TCOFLUSH);
  tcsetattr(ttyVG, TCSANOW, &newtio);

  return ttyVG;
}

int CloseSerial(int ttyVG)
{
  tcflush (ttyVG, TCIFLUSH);
  tcflush (ttyVG, TCOFLUSH);
  return close(ttyVG);
}
