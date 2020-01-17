var
 _stack_top: record end; external name '_stack_top';
 _data: record end; external name '_data';
 _edata: record end; external name '_edata';
 _text_start: record end; external name '_text_start';
 _etext: record end; external name '_etext';
 _bss_start: record end; external name '_bss_start';
 _bss_end: record end; external name '_bss_end';

procedure Pascalmain; external name 'PASCALMAIN';

procedure HaltProc; noreturn; public name'_haltproc';
begin
  while true do;
end;

procedure Startup;
var
  pdest, psrc, pend: PLongWord;
begin
  pdest:=@_data;
  psrc:=@_etext;
  pend:=@_bss_start;

  while pdest<pend do
    begin
      pdest^:=psrc^;
      inc(pdest);
      inc(psrc);
    end;

  pend:=@_bss_end;
  while pdest<pend do
    begin
      pdest^:=0;
      inc(pdest);
    end;

  PASCALMAIN;
  HaltProc;
end;

procedure LowlevelStartup; assembler; nostackframe; [public, alias: '_START'];
asm
  { Initialize global Pointer }
  .option push
  .option norelax
  lui   gp, %hi(_bss_start+0x800)
  addi  gp, gp, %lo(_bss_start+0x800)
  .option pop

  { Initialize Stack Pointer }
.L1:
  auipc sp, %pcrel_hi(_stack_top)
  addi  sp, sp, %pcrel_lo(.L1)

  { Initialise FP to zero }
  addi  fp, x0, 0

  jal x0, Startup
//   { Copy Initialized vars}
//   lui   x10,%hi(_data)
//   addi  x10,x10,%lo(_data)
//   lui   x11,%hi(_edata)
//   addi  x11,x11,%lo(_edata)
//   sltu  x12,x10,x11
//   beq   x12,x0,.L2
//   lui   x12,%hi(_etext)
//   addi  x12,x12,%lo(_etext)
//   addi  x10,x10,-1
//   addi  x12,x12,-1
// .L1:
//   addi  x10,x10,1
//   addi  x12,x12,1
//   lb    x13,0(x10)
//   sb    x13,0(x12)
//   bne   x10,x11,.L1
// .L2:
// { Initialize Memory}
//   lui   x10,%hi(_bss_start)
//   addi  x10,x10,%lo(_bss_start)
//   lui   x11,%hi(_bss_end)
//   addi  x11,x11,%lo(_bss_end)
//   sltu  x12,x10,x11
//   beq   x12,x0,.L4
//   addi  x10,x10,-1
// .L3:
//   addi  x10,x10,1
//   sb    x0,0(x10)
//   bne   x10,x11,.L3
// .L4:
//   jal   x0,PASCALMAIN
//   jal   x0,HaltProc
// .L_bss_start:
//   .long _bss_start
// .L_bss_end:
//   .long _bss_end
// .L_etext:
//   .long _etext
// .L_data:
//   .long _data
// .L_edata:
//   .long _edata
// .text
end;
