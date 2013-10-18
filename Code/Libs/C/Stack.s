; Set the stack to be under address 0x8000, so in regular address space. We choose a stack size of 4K and assume it'll be large enough.
; Add this to any projects utilizing the dsPIC33Es where the warning "Taking the address of VARIABLE may require an extended pointer for this device" occurs.
; With this compiled into a program, you may ignore all the extended pointer warnings.

.section non_eds_stack, stack, address(0x8000 - 8192) ; allocate the stack below 0x8000
.space 8192
