/*
 * syscalls.c - Minimal newlib hooks. snprintf pulls these in; nothing here
 * uses files. _write goes to the debug UART so printf works if you need it.
 */
#include <errno.h>
#include <sys/stat.h>

#include "board.h"

int _write(int fd, const char *buf, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (const uint8_t *)buf, (uint16_t)len, 100);
    return len;
}

int _read(int fd, char *buf, int len)       { (void)fd; (void)buf; (void)len; errno = ENOSYS; return -1; }
int _close(int fd)                          { (void)fd; return -1; }
int _lseek(int fd, int off, int whence)     { (void)fd; (void)off; (void)whence; return 0; }
int _isatty(int fd)                         { (void)fd; return 1; }
int _fstat(int fd, struct stat *st)         { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _getpid(void)                           { return 1; }
int _kill(int pid, int sig)                 { (void)pid; (void)sig; errno = EINVAL; return -1; }
