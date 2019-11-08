{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Watcom

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit System;

INTERFACE

{$define FPC_IS_SYSTEM}
{$ifndef NO_EXCEPTIONS_IN_SYSTEM}
{$define EXCEPTIONS_IN_SYSTEM}
{$endif NO_EXCEPTIONS_IN_SYSTEM}

{$define FPC_ANSI_TEXTFILEREC}
{ include system-independent routine headers }

{ wasm does not support SSE3 instructions }
{$define FASTMOVE_DISABLE_SSE3}

{$include systemh.inc}



const
 LineEnding = #13#10;
{ LFNSupport is a variable here, defined below!!! }
 DirectorySeparator = '\';
 DriveSeparator = ':';
 ExtensionSeparator = '.';
 PathSeparator = ';';
 AllowDirectorySeparators : set of char = ['\','/'];
 AllowDriveSeparators : set of char = [':'];
{ FileNameCaseSensitive and FileNameCasePreserving are defined separately below!!! }
 maxExitCode = 255;
 MaxPathLen = 256;

const
{ Default filehandles }
  UnusedHandle    = -1;
  StdInputHandle  = 0;
  StdOutputHandle = 1;
  StdErrorHandle  = 2;

  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = false;
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

{ Default memory segments (Tp7 compatibility) }
  seg0040 = $0040;
  segA000 = $A000;
  segB000 = $B000;
  segB800 = $B800;

var
{ Mem[] support }
  mem  : array[0..$7fffffff-1] of byte absolute $0:$0;
  memw : array[0..($7fffffff div sizeof(word)) -1] of word absolute $0:$0;
  meml : array[0..($7fffffff div sizeof(longint)) -1] of longint absolute $0:$0;
{ C-compatible arguments and environment }
  argc  : longint;
  argv  : ppchar;
  envp  : ppchar;
  dos_argv0 : pchar;

  AllFilesMask: string [3];

{$ifndef RTLLITE}
{ System info }
  LFNSupport : boolean;
{$ELSE RTLLITE}
Const
  LFNSupport = false;
{$endif RTLLITE}

{
  necessary for objects.pas, should be removed (at least from the interface
  to the implementation)
}
  type
    trealregs=record
      realedi,realesi,realebp,realres,
      realebx,realedx,realecx,realeax : longint;
      realflags,
      reales,realds,realfs,realgs,
      realip,realcs,realsp,realss  : word;
    end;
  function  do_write(h:longint;addr:pointer;len : longint) : longint;
  function  do_read(h:longint;addr:pointer;len : longint) : longint;
  procedure syscopyfromdos(addr : sizeuint; len : longint);
  procedure syscopytodos(addr : sizeuint; len : longint);
  procedure sysrealintr(intnr : word;var regs : trealregs);

  var tb:longint;
      tb_segment:word;

  const tb_offset=0;
        tb_size=8192;

IMPLEMENTATION

{ include system independent routines }

{$include system.inc}


{$asmmode ATT}

var psp_selector:word; external name 'PSP_SELECTOR';

procedure setup_arguments;
type
  arrayword = array [0..255] of word;
var
  proxy_s  : string[50];
  proxy_argc,proxy_seg,proxy_ofs,lin : longint;
  rm_argv  : ^arrayword;
  argv0len : longint;
  useproxy : boolean;
  hp       : ppchar;
  doscmd   : string[129];  { Dos commandline copied from PSP, max is 128 chars +1 for terminating zero }
  arglen,
  count   : longint;
  argstart,
  pc,arg  : pchar;
  quote   : char;
  argvlen : longint;

  function atohex(s : pchar) : longint;
  var
    rv : longint;
    v  : byte;
  begin
    rv:=0;
    while (s^<>#0) do
     begin
       v:=byte(s^)-byte('0');
       if (v > 9) then
         dec(v,7);
       v:=v and 15; { in case it's lower case }
       rv:=(rv shl 4) or v;
       inc(longint(s));
     end;
    atohex:=rv;
  end;

  procedure allocarg(idx,len:longint);
  var oldargvlen:longint;
  begin
    if idx>=argvlen then
     begin
       oldargvlen:=argvlen;
       argvlen:=(idx+8) and (not 7);
       sysreallocmem(argv,argvlen*sizeof(pointer));
       fillchar(argv[oldargvlen],(argvlen-oldargvlen)*sizeof(pointer),0);
       argv[idx]:=nil;
     end;
     ArgV [Idx] := SysAllocMem (Succ (Len));
  end;

begin
  count:=0;
  argc:=1;
  argv:=nil;
  argvlen:=0;
  { load commandline from psp }
  sysseg_move(psp_selector, 128, get_ds, longint(@doscmd), 128);
  doscmd[length(doscmd)+1]:=#0;
{$IfDef SYSTEM_DEBUG_STARTUP}
  Writeln(stderr,'Dos command line is #',doscmd,'# size = ',length(doscmd));
{$EndIf }
  { create argv[0] }
  argv0len:=strlen(dos_argv0);
  allocarg(count,argv0len);
  move(dos_argv0^,argv[count]^,argv0len);
  inc(count);
  { setup cmdline variable }
  cmdline:=Getmem(argv0len+length(doscmd)+2);
  move(dos_argv0^,cmdline^,argv0len);
  cmdline[argv0len]:=' ';
  inc(argv0len);
  move(doscmd[1],cmdline[argv0len],length(doscmd));
  cmdline[argv0len+length(doscmd)+1]:=#0;
  { parse dos commandline }
  pc:=@doscmd[1];
  while pc^<>#0 do
   begin
     { skip leading spaces }
     while pc^ in [#1..#32] do
      inc(pc);
     if pc^=#0 then
      break;
     { calc argument length }
     quote:=' ';
     argstart:=pc;
     arglen:=0;
     while (pc^<>#0) do
      begin
        case pc^ of
          #1..#32 :
            begin
              if quote<>' ' then
               inc(arglen)
              else
               break;
            end;
          '"' :
            begin
              if quote<>'''' then
               begin
                 if pchar(pc+1)^<>'"' then
                  begin
                    if quote='"' then
                     quote:=' '
                    else
                     quote:='"';
                  end
                 else
                  inc(pc);
               end
              else
               inc(arglen);
            end;
          '''' :
            begin
              if quote<>'"' then
               begin
                 if pchar(pc+1)^<>'''' then
                  begin
                    if quote=''''  then
                     quote:=' '
                    else
                     quote:='''';
                  end
                 else
                  inc(pc);
               end
              else
               inc(arglen);
            end;
          else
            inc(arglen);
        end;
        inc(pc);
      end;
     { copy argument }
     allocarg(count,arglen);
     quote:=' ';
     pc:=argstart;
     arg:=argv[count];
     while (pc^<>#0) do
      begin
        case pc^ of
          #1..#32 :
            begin
              if quote<>' ' then
               begin
                 arg^:=pc^;
                 inc(arg);
               end
              else
               break;
            end;
          '"' :
            begin
              if quote<>'''' then
               begin
                 if pchar(pc+1)^<>'"' then
                  begin
                    if quote='"' then
                     quote:=' '
                    else
                     quote:='"';
                  end
                 else
                  inc(pc);
               end
              else
               begin
                 arg^:=pc^;
                 inc(arg);
               end;
            end;
          '''' :
            begin
              if quote<>'"' then
               begin
                 if pchar(pc+1)^<>'''' then
                  begin
                    if quote=''''  then
                     quote:=' '
                    else
                     quote:='''';
                  end
                 else
                  inc(pc);
               end
              else
               begin
                 arg^:=pc^;
                 inc(arg);
               end;
            end;
          else
            begin
              arg^:=pc^;
              inc(arg);
            end;
        end;
        inc(pc);
      end;
     arg^:=#0;
 {$IfDef SYSTEM_DEBUG_STARTUP}
     Writeln(stderr,'dos arg ',count,' #',arglen,'#',argv[count],'#');
 {$EndIf SYSTEM_DEBUG_STARTUP}
     inc(count);
   end;
  argc:=count;
  { check for !proxy for long commandlines passed using environment }
  hp:=envp;
  useproxy:=false;
  while assigned(hp^) do
   begin
     if (hp^[0]=' ') then
      begin
        proxy_s:=strpas(hp^);
        if Copy(proxy_s,1,7)=' !proxy' then
         begin
           proxy_s[13]:=#0;
           proxy_s[18]:=#0;
           proxy_s[23]:=#0;
           argv[2]:=@proxy_s[9];
           argv[3]:=@proxy_s[14];
           argv[4]:=@proxy_s[19];
           useproxy:=true;
           break;
         end;
      end;
     inc(hp);
   end;
  { check for !proxy for long commandlines passed using commandline }
  if (not useproxy) and
     (argc > 1) and (far_strlen(get_ds,longint(argv[1])) = 6)  then
   begin
     move(argv[1]^,proxy_s[1],6);
     proxy_s[0] := #6;
     if (proxy_s = '!proxy') then
      useproxy:=true;
   end;
  { use proxy when found }
  if useproxy then
   begin
     proxy_argc:=atohex(argv[2]);
     proxy_seg:=atohex(argv[3]);
     proxy_ofs:=atohex(argv[4]);
{$IfDef SYSTEM_DEBUG_STARTUP}
     Writeln(stderr,'proxy command line found');
     writeln(stderr,'argc: ',proxy_argc,' seg: ',proxy_seg,' ofs: ',proxy_ofs);
{$EndIf SYSTEM_DEBUG_STARTUP}
     rm_argv:=SysGetmem(proxy_argc*sizeof(word));
     sysseg_move(dos_selector,proxy_seg*16+proxy_ofs, get_ds,longint(rm_argv),proxy_argc*sizeof(word));
     for count:=0 to proxy_argc - 1 do
      begin
        lin:=proxy_seg*16+rm_argv^[count];
        arglen:=far_strlen(dos_selector,lin);
        allocarg(count,arglen);
        sysseg_move(dos_selector,lin,get_ds,longint(argv[count]),arglen+1);
{$IfDef SYSTEM_DEBUG_STARTUP}
        Writeln(stderr,'arg ',count,' #',rm_argv^[count],'#',arglen,'#',argv[count],'#');
{$EndIf SYSTEM_DEBUG_STARTUP}
    end;
     SysFreemem(rm_argv);
     argc:=proxy_argc;
   end;
  { create an nil entry }
  allocarg(argc,0);
  { free unused memory }
  sysreallocmem(argv,(argc+1)*sizeof(pointer));
end;

function strcopy(dest,source : pchar) : pchar;assembler;
var
  saveeax,saveesi,saveedi : longint;
asm
        movl    %edi,saveedi
        movl    %esi,saveesi
{$ifdef REGCALL}
        movl    %eax,saveeax
        movl    %edx,%edi
{$else}
        movl    source,%edi
{$endif}
        testl   %edi,%edi
        jz      .LStrCopyDone
        leal    3(%edi),%ecx
        andl    $-4,%ecx
        movl    %edi,%esi
        subl    %edi,%ecx
{$ifdef REGCALL}
        movl    %eax,%edi
{$else}
        movl    dest,%edi
{$endif}
        jz      .LStrCopyAligned
.LStrCopyAlignLoop:
        movb    (%esi),%al
        incl    %edi
        incl    %esi
        testb   %al,%al
        movb    %al,-1(%edi)
        jz      .LStrCopyDone
        decl    %ecx
        jnz     .LStrCopyAlignLoop
        .balign  16
.LStrCopyAligned:
        movl    (%esi),%eax
        movl    %eax,%edx
        leal    0x0fefefeff(%eax),%ecx
        notl    %edx
        addl    $4,%esi
        andl    %edx,%ecx
        andl    $0x080808080,%ecx
        jnz     .LStrCopyEndFound
        movl    %eax,(%edi)
        addl    $4,%edi
        jmp     .LStrCopyAligned
.LStrCopyEndFound:
        testl   $0x0ff,%eax
        jz      .LStrCopyByte
        testl   $0x0ff00,%eax
        jz      .LStrCopyWord
        testl   $0x0ff0000,%eax
        jz      .LStrCopy3Bytes
        movl    %eax,(%edi)
        jmp     .LStrCopyDone
.LStrCopy3Bytes:
        xorb     %dl,%dl
        movw     %ax,(%edi)
        movb     %dl,2(%edi)
        jmp     .LStrCopyDone
.LStrCopyWord:
        movw    %ax,(%edi)
        jmp     .LStrCopyDone
.LStrCopyByte:
        movb    %al,(%edi)
.LStrCopyDone:
{$ifdef REGCALL}
        movl    saveeax,%eax
{$else}
        movl    dest,%eax
{$endif}
        movl    saveedi,%edi
        movl    saveesi,%esi
end;


var
    env_selector:word; external name 'ENV_SELECTOR';
    env_size:longint; external name 'ENV_SIZE';

procedure setup_environment;
var env_count : longint;
    dos_env,cp : pchar;
begin
  env_count:=0;
  dos_env:=getmem(env_size);
  sysseg_move(env_selector,$0, get_ds, longint(dos_env), env_size);
  cp:=dos_env;
  while cp ^ <> #0 do
    begin
    inc(env_count);
    while (cp^ <> #0) do inc(longint(cp)); { skip to NUL }
    inc(longint(cp)); { skip to next character }
    end;
  envp := sysgetmem((env_count+1) * sizeof(pchar));
  if (envp = nil) then exit;
  cp:=dos_env;
  env_count:=0;
  while cp^ <> #0 do
   begin
     envp[env_count] := sysgetmem(strlen(cp)+1);
     strcopy(envp[env_count], cp);
{$IfDef SYSTEM_DEBUG_STARTUP}
     Writeln(stderr,'env ',env_count,' = "',envp[env_count],'"');
{$EndIf SYSTEM_DEBUG_STARTUP}
     inc(env_count);
     while (cp^ <> #0) do
      inc(longint(cp)); { skip to NUL }
     inc(longint(cp)); { skip to next character }
   end;
  envp[env_count]:=nil;
  longint(cp):=longint(cp)+3;
  dos_argv0 := sysgetmem(strlen(cp)+1);
  if (dos_argv0 = nil) then halt;
  strcopy(dos_argv0, cp);
end;


procedure syscopytodos(addr : sizeuint; len : longint);
begin
   if len > tb_size then
     HandleError(217);
   sysseg_move(get_ds,addr,dos_selector,tb,len);
end;


procedure syscopyfromdos(addr : sizeuint; len : longint);
begin
   if len > tb_size then
     HandleError(217);
   sysseg_move(dos_selector,tb,get_ds,addr,len);
end;


procedure sysrealintr(intnr : word;var regs : trealregs);
begin
   regs.realsp:=0;
   regs.realss:=0;
   asm
      pushl %edi
      pushl %ebx
      pushw %fs
      movw  intnr,%bx
      xorl  %ecx,%ecx
      movl  regs,%edi
      movw  $0x300,%ax
      int   $0x31
      popw  %fs
      popl %ebx
      popl %edi
   end;
end;


procedure set_pm_interrupt(vector : byte;const intaddr : tseginfo);
begin
  asm
        pushl %ebx
        movl intaddr,%eax
        movl (%eax),%edx
        movw 4(%eax),%cx
        movl $0x205,%eax
        movb vector,%bl
        int $0x31
        popl %ebx
  end;
end;


procedure get_pm_interrupt(vector : byte;var intaddr : tseginfo);
begin
  asm
        pushl %ebx
        movb    vector,%bl
        movl    $0x204,%eax
        int     $0x31
        movl    intaddr,%eax
        movl    %edx,(%eax)
        movw    %cx,4(%eax)
        popl %ebx
  end;
end;



{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure ___exit(exitcode:longint);cdecl;external name '___exit';


Procedure system_exit;
var
  h : byte;
begin
  for h:=0 to max_files-1 do
    if openfiles[h] then
      begin
{$ifdef SYSTEMDEBUG}
         writeln(stderr,'file ',opennames[h],' not closed at exit');
{$endif SYSTEMDEBUG}
         if h>=5 then
           do_close(h);
      end;
  { halt is not allways called !! }
  { not on normal exit !! PM }
  set_pm_interrupt($00,old_int00);
{$ifndef EXCEPTIONS_IN_SYSTEM}
  set_pm_interrupt($75,old_int75);
{$endif EXCEPTIONS_IN_SYSTEM}
  ___exit(exitcode);
end;


procedure new_int00;
begin
  HandleError(200);
end;


{$ifndef EXCEPTIONS_IN_SYSTEM}
procedure new_int75;
begin
  asm
        xorl    %eax,%eax
        outb    %al,$0x0f0
        movb    $0x20,%al
        outb    %al,$0x0a0
        outb    %al,$0x020
  end;
  HandleError(200);
end;
{$endif EXCEPTIONS_IN_SYSTEM}


var
  __stkbottom : pointer;//###########external name '__stkbottom';



{*****************************************************************************
                              ParamStr/Randomize
*****************************************************************************}

function paramcount : longint;
begin
  paramcount := argc - 1;
end;


function paramstr(l : longint) : string;
begin
  if (l>=0) and (l+1<=argc) then
   paramstr:=strpas(argv[l])
  else
   paramstr:='';
end;


procedure randomize;
var
  hl   : longint;
  regs : trealregs;
begin
  regs.realeax:=$2c00;
  sysrealintr($21,regs);
  hl:=lo(regs.realedx);
  randseed:=hl*$10000+ lo(regs.realecx);
end;


{ include standard heap management }
 { include heap.inc}


(*
{*****************************************************************************
                           UnTyped File Handling
*****************************************************************************}

{$i file.inc}

{*****************************************************************************
                           Typed File Handling
*****************************************************************************}

{$i typefile.inc}

{*****************************************************************************
                           Text File Handling
*****************************************************************************}

{$i text.inc}


{*****************************************************************************
                           Generic Handling
*****************************************************************************}

{$ifdef TEST_GENERIC}
{$i generic.inc}
{$endif TEST_GENERIC}
*)

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

function CheckLFN:boolean;
var
  regs     : TRealRegs;
  RootName : pchar;
begin
{ Check LFN API on drive c:\ }
  RootName:='C:\';
  syscopytodos(longint(RootName),strlen(RootName)+1);
{ Call 'Get Volume Information' ($71A0) }
  regs.realeax:=$71a0;
  regs.reales:=tb_segment;
  regs.realedi:=tb_offset;
  regs.realecx:=32;
  regs.realds:=tb_segment;
  regs.realedx:=tb_offset;
  regs.realflags:=carryflag;
  sysrealintr($21,regs);
{ If carryflag=0 and LFN API bit in ebx is set then use Long file names }
  CheckLFN:=(regs.realflags and carryflag=0) and (regs.realebx and $4000=$4000);
end;

{$ifdef  EXCEPTIONS_IN_SYSTEM}
{$define IN_SYSTEM}
{$i dpmiexcp.pp}
{$endif  EXCEPTIONS_IN_SYSTEM}

procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
end;


function GetProcessID: SizeUInt;
begin
 GetProcessID := 1;
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

var
  temp_int : tseginfo;
Begin
  alloc_tb;
  StackLength := InitialStkLen;
  StackBottom := __stkbottom;
  { To be set if this is a GUI or console application }
  IsConsole := TRUE;
  { To be set if this is a library and not a program  }
  IsLibrary := FALSE;
{ save old int 0 and 75 }
  get_pm_interrupt($00,old_int00);
  get_pm_interrupt($75,old_int75);
  temp_int.segment:=get_cs;
  temp_int.offset:=@new_int00;
  set_pm_interrupt($00,temp_int);
{$ifndef EXCEPTIONS_IN_SYSTEM}
  temp_int.offset:=@new_int75;
  set_pm_interrupt($75,temp_int);
{$endif EXCEPTIONS_IN_SYSTEM}
{ Setup heap }
  InitHeap;
  SysInitExceptions;
  initunicodestringmanager;
{ Setup stdin, stdout and stderr }
  SysInitStdIO;
{ Setup environment and arguments }
  Setup_Environment;
  Setup_Arguments;
{ Use LFNSupport LFN }
  LFNSupport:=CheckLFN;
  if LFNSupport then
   begin
    FileNameCasePreserving:=true;
    AllFilesMask := '*';
   end
  else
   AllFilesMask := '*.*';

{ Reset IO Error }
  InOutRes:=0;
  ThreadID := 1;
{$ifdef  EXCEPTIONS_IN_SYSTEM}
  InitDPMIExcp;
  InstallDefaultHandlers;
{$endif  EXCEPTIONS_IN_SYSTEM}
End.
