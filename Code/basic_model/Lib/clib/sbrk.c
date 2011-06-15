
/*
** Extend the process's data space by INCREMENT.
** If INCREMENT is negative, shrink data space by - INCREMENT.
** Return start of new space allocated, or -1 for errors.
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
extern int brk(void *endds);
extern void *__curbrk;	/* in brk.c */
void * __attribute__((__weak__, __section__(".libc")))
sbrk(int incr)
{
	void *oldbrk;

	if (__curbrk == ((void *)0))
	{
		if (brk(0) < 0)		/* Initialize the break.  */
		{
			return((void *) -1);
		}
	}
	if (incr == 0)
	{
		return(__curbrk);
	}
	oldbrk = __curbrk;
	if (brk(oldbrk + incr) < 0)
	{
		return((void *) -1);
	}
	return(oldbrk);
}

