var
 _stack_top: record end; external name '_stack_top';
 _data: record end; external name '_data';
 _edata: record end; external name '_edata';
 _text_start: record end; external name '_text_start';
 _etext: record end; external name '_etext';
 _bss_start: record end; external name '_bss_start';
 _bss_end: record end; external name '_bss_end';

procedure Pascalmain; external name 'PASCALMAIN';

procedure HaltProc; assembler; nostackframe; public name'_haltproc';
asm
.Lloop:
   b .Lloop
end;

procedure Startup; assembler; nostackframe; [public, alias: '_START'];
asm
  ldr r1,.L_etext
  ldr r2,.L_data
  ldr r3,.L_edata
.Lcopyloop:
  cmp r2,r3
  ittt ls
  ldrls r0,[r1],#4
  strls r0,[r2],#4
  bls .Lcopyloop

  // clear onboard ram
  ldr r1,.L_bss_start
  ldr r2,.L_bss_end
  mov r0,#0
.Lzeroloop:
  cmp r1,r2
  itt ls
  strls r0,[r1],#4
  bls .Lzeroloop

{$ifdef REMAP_VECTTAB}
  ldr r0, .Lvtor
  ldr r1, .Ltext_start
  str r1, [r0]
{$endif REMAP_VECTTAB}

{$if defined(FPUARM_HAS_VFP_EXTENSION)}
  ldr r0, .Lcpacr
  ldr r1, [r0]
  orr r1, r1, #0xf00000
  str r1, [r0]
  dsb
  isb
{$endif defined(FPUARM_HAS_VFP_EXTENSION)}
  bl PASCALMAIN
  b HaltProc

.L_bss_start:
  .long _bss_start
.L_bss_end:
  .long _bss_end
.L_etext:
  .long _etext
.L_data:
  .long _data
.L_edata:
  .long _edata
{$if defined(FPUARM_HAS_VFP_EXTENSION)}
.Lcpacr:
  .long 0xE000ED88
{$endif defined(FPUARM_HAS_VFP_EXTENSION)}
{$ifdef REMAP_VECTTAB}
.Lvtor:
  .long 0xE000ED08
.Ltext_start:
  .long _text_start
{$endif REMAP_VECTTAB}
end;
