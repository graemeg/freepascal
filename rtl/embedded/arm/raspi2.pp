unit raspi2;

{$goto on}
{$INLINE ON}

interface

type
    TBitvector32 = bitpacked array[0..31] of 0..1;

const
    PeripheralBase = $3F000000;

    GPFSEL1 = PeripheralBase + $00200004;
    GPSET0 = PeripheralBase + $0020001C;
    GPCLR0 = PeripheralBase + $00200028;
    GPPUD = PeripheralBase + $00200094;
    GPPUDCLK0 = PeripheralBase + $00200098;

    AUX_ENABLES = PeripheralBase + $00215004;
    AUX_MU_IO_REG = PeripheralBase + $00215040;
    AUX_MU_IER_REG = PeripheralBase + $00215044;
    AUX_MU_IIR_REG = PeripheralBase + $00215048;
    AUX_MU_LCR_REG = PeripheralBase + $0021504C;
    AUX_MU_MCR_REG = PeripheralBase + $00215050;
    AUX_MU_LSR_REG = PeripheralBase + $00215054;
    AUX_MU_MSR_REG = PeripheralBase + $00215058;
    AUX_MU_SCRATCH = PeripheralBase + $0021505C;
    AUX_MU_CNTL_REG = PeripheralBase + $00215060;
    AUX_MU_STAT_REG = PeripheralBase + $00215064;
    AUX_MU_BAUD_REG = PeripheralBase + $00215068;

implementation

uses
    consoleio;

procedure _FPC_haltproc; assembler; nostackframe; public name '_haltproc';
asm
.Lhalt:
    wfi
    b .Lhalt
end;

procedure DUMMY(Count: DWord);
var
    i : DWord;
begin
    for i := 0 to Count do
    begin
        asm
            nop
        end;
    end;
end; 

procedure PUT32(Address: DWord; Value: DWord); inline;
VAR
    p: ^DWord;
begin
    p := POINTER (Address);
    p^ := Value;
end;

function GET32(Address: DWord) : DWord; inline;
VAR
    p: ^DWord;
begin
    p := POINTER (Address);
    GET32 := p^;
end;

function UARTLCR(): DWord;
begin
    UARTLCR := GET32(AUX_MU_LCR_REG);
end;

procedure UARTPuts(C: Char);
begin
    while True do
    begin
        if (GET32(AUX_MU_LSR_REG) and $20) > 0 then break;
    end;

    PUT32(AUX_MU_IO_REG, DWord(C));
end;

function UARTGet(): Char;
begin
    while True do
    begin
        if (GET32(AUX_MU_LSR_REG) and $01) > 0 then break;
    end;

    UARTGet := Char(GET32(AUX_MU_IO_REG) and $FF);
end;

procedure UARTFlush();
begin
    while True do
    begin
        if (GET32(AUX_MU_LSR_REG) and $100) > 0 then break;
    end;
end;

function RaspiWrite(ACh: char; AUserData: pointer): boolean;
begin
    UARTPuts(ACh);

    RaspiWrite := true;
end;

function RaspiRead(var ACh: char; AUserData: pointer): boolean;
begin
    if (GET32(AUX_MU_LSR_REG) and $01) > 0 then
    begin
        ACh := UARTGet();
    end else
    begin
        ACh := #0;
    end;

    RaspiRead := true;
end;

procedure UARTInit; public name 'UARTInit';
var
    ra: dword;
begin
    PUT32(AUX_ENABLES, 1);
    PUT32(AUX_MU_IER_REG, 0);
    PUT32(AUX_MU_CNTL_REG, 0);
    PUT32(AUX_MU_LCR_REG, 3);
    PUT32(AUX_MU_MCR_REG, 0);
    PUT32(AUX_MU_IER_REG, 0);
    PUT32(AUX_MU_IIR_REG, $C6);
    PUT32(AUX_MU_BAUD_REG, 270);
    
    ra := GET32(GPFSEL1);
    ra := ra AND (not (7 shl 12)); // gpio14
    ra := ra OR (2 shl 12);  // alt5
    ra := ra AND (not (7 shl 15)); // gpio15
    ra := ra OR (2 shl 15);  // alt5

    PUT32(GPFSEL1, ra);
    PUT32(GPPUD, 0);
    
    Dummy(500);

    PUT32(GPPUDCLK0, ((1 shl 14) OR (1 shl 15)));

    Dummy(500);

    PUT32(GPPUDCLK0, 0);
    PUT32(AUX_MU_CNTL_REG, 3);
end;

{$ifndef CUSTOM_ENTRY}
procedure PASCALMAIN; external name 'PASCALMAIN';

var
    _stack_top: record end; external name '_stack_top';

{ This start makes sure we only execute on core 0 - the others will halt }
procedure _FPC_start; assembler; nostackframe;
label
    _start;
asm
    .init
    .align 16
    .globl _start
_start:
    // enable fpu
    .long 0xee110f50      // mrc p15, 0, r0, c1, c0, 2
    orr r0, r0, #0x300000 // single precision
    orr r0, r0, #0xC00000 // double precision
    .long 0xee010f50      // mcr p15, 0, r0, c1, c0, 2
    mov r0, #0x40000000
    .long 0xeee80a10      // fmxr fpexc, r0

    .long 0xee100fb0      // mrc p15,0,r0,c0,c0,5 - find the core ID
    mov r1, #0xFF
    ands r1, r1, r0
    bne _FPC_haltproc

    ldr r0, .L_stack_top
    mov sp, r0

    bl UARTInit
    bl PASCALMAIN
    bl _FPC_haltproc
.L_stack_top:
    .long _stack_top
    .text
end;
{$endif CUSTOM_ENTRY}

begin
    OpenIO(Input, @RaspiWrite, @RaspiRead, fmInput, nil);
    OpenIO(Output, @RaspiWrite, @RaspiRead, fmOutput, nil);
    OpenIO(ErrOutput, @RaspiWrite, @RaspiRead, fmOutput, nil);
    OpenIO(StdOut, @RaspiWrite, @RaspiRead, fmOutput, nil);
    OpenIO(StdErr, @RaspiWrite, @RaspiRead, fmOutput, nil);
end.