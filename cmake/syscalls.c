/**
  ******************************************************************************
  * @file    syscalls.c
  * @brief   newlib 裸机系统调用桩（GCC 链路专用）
  *
  *          本工程不使用标准 I/O，也不需要堆；这些桩用于满足 newlib 的链接需求。
  *          之所以自带而非直接引 --specs=nosys.specs：libnosys 的弱符号带
  *          .gnu.warning 注记，链接时会输出
  *            "_write is not implemented and will always fail"
  *          等 4 条链接告警，破坏 0 Warning 的验收口径。
  *
  *          未使用的桩会被 --gc-sections 回收，不占用 Flash。
  ******************************************************************************
  */

#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* _sbrk 的堆顶/栈顶边界，由链接脚本提供 */
extern char end;      /* 链接脚本 ._user_heap_stack 之前的地址 */
extern char _estack;  /* RAM 最高地址，即初始栈顶 */

int _close(int fildes)
{
    (void)fildes;
    return -1;
}

off_t _lseek(int fildes, off_t offset, int whence)
{
    (void)fildes;
    (void)offset;
    (void)whence;
    return 0;
}

int _read(int fildes, void *buf, size_t nbyte)
{
    (void)fildes;
    (void)buf;
    (void)nbyte;
    errno = ENOSYS;
    return -1;
}

int _write(int fildes, const void *buf, size_t nbyte)
{
    (void)fildes;
    (void)buf;
    (void)nbyte;
    errno = ENOSYS;
    return -1;
}

int _fstat(int fildes, struct stat *st)
{
    (void)fildes;
    if (st == NULL)
    {
        errno = EINVAL;
        return -1;
    }
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fildes)
{
    (void)fildes;
    return 1;
}

/* 堆从链接脚本给出的 end 起向上生长，越过 _estack 即失败 */
void *_sbrk(ptrdiff_t incr)
{
    static char *heap_end = NULL;
    char *prev;

    if (heap_end == NULL)
    {
        heap_end = &end;
    }

    prev = heap_end;

    if (heap_end + incr > &_estack)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return prev;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

void _exit(int status)
{
    (void)status;
    while (1)
    {
    }
}
