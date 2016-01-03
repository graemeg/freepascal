program main;
{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

// The default exception handler displays the exception type - data abort or undefined instruction
// you can relate the exception to your code using arm-eabi-addr2line -e <elf file> <address>
// assuming you built with debug info this will display a source file and a line number
// The address of the instruction is shown as pc, beside the address which faulted
// the rest of the screen is a dump of the registers.

begin
	// install the default exception handler
	defaultExceptionHandler();

	irqInit();
	irqEnable(IRQ_VBLANK);

	// generate an exception
	pu32(250)^ := 100;

	while true do
    swiWaitForVBlank();

end.
