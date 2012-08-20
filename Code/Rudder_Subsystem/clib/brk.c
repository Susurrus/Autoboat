/*
** Set the end of the process's data space to endds.
** Return 0 if successful, -1 if not. 
**
**
** brk() and sbrk() are used to change dynamically the amount of space
** allocated for the calling process's data segment.  The change is made
** by resetting the process's break value and allocating the appropriate
** amount of space.  The break value is the address of the first location
** beyond the end of the data segment.  The amount of allocated space
** increases as the break value increases.  Newly allocated space is set to
** zero.  If, however, the same memory space is reallocated to the same
** process its contents are undefined.
**
** sbrk adds incr bytes to the break value and changes the allocated space
** accordingly.  incr can be negative, in which case the amount of allocated
** space is decreased.
**
** brk and sbrk will fail without making any change in the allocated space
** if one or more of the following are true:
**
** ENOMEM       Such a change would result in more space being
**		allocated than is allowed by the system-imposed maximum
**		process size.
**
** EAGAIN       Total amount of system memory available for a read
**		during physical IO is temporarily insufficient.
**		This may occur even though the space requested was less than
**		the system-imposed maximum process size.
*/
extern void _heap, _eheap;
void *__curbrk;
int
__attribute__((__weak__, __section__(".libc")))
brk(void *endds)
{
	int rc = 0;

	if (endds == ((void *)0))
	{
		__curbrk = &_heap;
	}
	else if (endds <= &_eheap)
	{
		__curbrk = endds;
	}
	else
	{
		rc = -1;
	}
	return(rc);
}

