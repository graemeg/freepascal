{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;

interface

{ two debug conditionnals can be used
  - SYSTEMDEBUG
    -for STACK checks
    -for non closed files at exit (or at any time with GDB)
  - SYSTEM_DEBUG_STARTUP
    specifically for
    - proxy command line (DJGPP feature)
    - list of args
    - list of env variables  (PM) }

{$define FPC_IS_SYSTEM}
{$ifndef NO_EXCEPTIONS_IN_SYSTEM}
{$define EXCEPTIONS_IN_SYSTEM}
{$endif NO_EXCEPTIONS_IN_SYSTEM}
{$define USE_NOTHREADMANAGER}

{ include system-independent routine headers }

{$I systemh.inc}
{$I portsh.inc}


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
  memw : array[0..($7fffffff div sizeof(word))-1] of word absolute $0:$0;
  meml : array[0..($7fffffff div sizeof(longint))-1] of longint absolute $0:$0;
{ C-compatible arguments and environment }
  argc:longint;public name 'operatingsystem_parameter_argc';
  argv:PPchar;public name 'operatingsystem_parameter_argv';
  envp:PPchar;public name 'operatingsystem_parameter_envp';
  dos_argv0 : pchar; public name 'dos_argv0';

  AllFilesMask: string [3];

{$ifndef RTLLITE}
{ System info }
  LFNSupport : boolean;
{$ELSE RTLLITE}
const
  LFNSupport = false;
{$endif RTLLITE}

type
{ Dos Extender info }
  p_stub_info = ^t_stub_info;
  t_stub_info = packed record
       magic         : array[0..15] of char;
       size          : longint;
       minstack      : longint;
       memory_handle : longint;
       initial_size  : longint;
       minkeep       : word;
       ds_selector   : word;
       ds_segment    : word;
       psp_selector  : word;
       cs_selector   : word;
       env_size      : word;
       basename      : array[0..7] of char;
       argv0         : array [0..15] of char;
       dpmi_server   : array [0..15] of char;
  end;

  p_go32_info_block = ^t_go32_info_block;
  t_go32_info_block = packed record
       size_of_this_structure_in_bytes    : longint; {offset 0}
       linear_address_of_primary_screen   : longint; {offset 4}
       linear_address_of_secondary_screen : longint; {offset 8}
       linear_address_of_transfer_buffer  : longint; {offset 12}
       size_of_transfer_buffer            : longint; {offset 16}
       pid                                : longint; {offset 20}
       master_interrupt_controller_base   : byte; {offset 24}
       slave_interrupt_controller_base    : byte; {offset 25}
       selector_for_linear_memory         : word; {offset 26}
       linear_address_of_stub_info_structure : longint; {offset 28}
       linear_address_of_original_psp     : longint; {offset 32}
       run_mode                           : word; {offset 36}
       run_mode_info                      : word; {offset 38}
  end;

var
  stub_info       : p_stub_info; public name 'operatingsystem_stub_info';
  go32_info_block : t_go32_info_block; public name 'operatingsystem_go32_info_block';
{$ifdef SYSTEMDEBUG}
const
   accept_sbrk : boolean = true;
{$endif}

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
  procedure syscopyfromdos(addr : longint; len : longint);
  procedure syscopytodos(addr : longint; len : longint);
  procedure sysrealintr(intnr : word;var regs : trealregs);
  function  tb : longint;

implementation

{ include system independent routines }

{$I system.inc}
{$I ports.inc}


var
  c_environ : ppchar;external name '__environ';
  _args : ppchar;external name '_args';
  __stubinfo : p_stub_info;external name '__stubinfo';
  ___dos_argv0 : pchar;external name '___dos_argv0';


procedure setup_arguments;
type
  arrayword = array [0..255] of word;
var
  psp      : word;
  proxy_s  : string[50];
  proxy_argc,proxy_seg,proxy_ofs,lin : longint;
  rm_argv  : ^arrayword;
  argv0len : longint;
  useproxy : boolean;
  hp       : ppchar;
  doscmd   : string[129];  { Dos commandline copied from PSP, max is 128 chars +1 for terminating zero }
  arglen,cmdlen,
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
    var
      oldargvlen : longint;
    begin
      if idx>=argvlen then
       begin
         oldargvlen:=argvlen;
         argvlen:=(idx+8) and (not 7);
         sysreallocmem(argv,argvlen*sizeof(pointer));
         fillchar(argv[oldargvlen],(argvlen-oldargvlen)*sizeof(pointer),0);
       end;
      { use realloc to reuse already existing memory }
      { always allocate, even if length is zero, since }
      { the arg. is still present!                     }
      sysreallocmem(argv[idx],len+1);
    end;

begin
  count:=0;
  argc:=1;
  argv:=nil;
  argvlen:=0;
  { load commandline from psp }
  psp:=stub_info^.psp_selector;
  sysseg_move(psp, 128, get_ds, longint(@doscmd), 128);
  doscmd[length(doscmd)+1]:=#0;
{$IfDef SYSTEM_DEBUG_STARTUP}
  Writeln(stderr,'Dos command line is #',doscmd,'# size = ',length(doscmd));
{$EndIf }
  { create argv[0] }
  argv0len:=strlen(dos_argv0);
  allocarg(count,argv0len+1);
  move(dos_argv0^,argv[count]^,argv0len+1);
  inc(count);
  { setup cmdline variable }
  cmdlen:=argv0len+length(doscmd)+2;
  cmdline:=Getmem(cmdlen);
  move(dos_argv0^,cmdline^,argv0len);
  cmdline[argv0len]:=' ';
  inc(argv0len);
  move(doscmd[1],cmdline[argv0len],length(doscmd));
  cmdline[cmdlen-1]:=#0;
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
           { Do not set argv[2..4] to PROXY_S
             values, because PROXY_S is on stack,
             while ARGV[2..4] need to be on heap.
             PM 2011-06-08
           argv[2]:=@proxy_s[9];
           argv[3]:=@proxy_s[14];
           argv[4]:=@proxy_s[19];}
           allocarg(2,4);
           strcopy(argv[2], @proxy_s[9]);
           allocarg(3,4);
           strcopy(argv[3], @proxy_s[14]);
           allocarg(4,4);
           strcopy(argv[4], @proxy_s[19]);
           { We need to change this variable env name
             otherwise it will be used by other DJGPP variables
             if we call them. PM 2011-07-04
             Hide it as '_!proxy' instead of ' !proxy' }
           hp^[0]:='_';
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
  _args:=argv;
end;


procedure setup_environment;
var env_selector : word;
    env_count : longint;
    dos_env,cp : pchar;
begin
   stub_info:=__stubinfo;
   dos_env := sysgetmem(stub_info^.env_size);
   env_count:=0;
   sysseg_move(stub_info^.psp_selector,$2c, get_ds, longint(@env_selector), 2);
   sysseg_move(env_selector, 0, get_ds, longint(dos_env), stub_info^.env_size);
  cp:=dos_env;
  while cp ^ <> #0 do
    begin
    inc(env_count);
    while (cp^ <> #0) do inc(longint(cp)); { skip to NUL }
    inc(longint(cp)); { skip to next character }
    end;
  envp := sysgetmem((env_count+1) * sizeof(pchar));
  if (envp = nil) then HandleError (203);
  c_environ:=envp;
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
  if (dos_argv0 = nil) then HandleError (203);
  strcopy(dos_argv0, cp);
  { update ___dos_argv0 also }
  ___dos_argv0:=dos_argv0
end;



{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure __exit(exitcode:longint);cdecl;external;

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
  __exit(exitcode);
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
  __stkbottom : pointer;external name '__stkbottom';



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
 GetProcessID := SizeUInt (Go32_info_block.pid);
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

var
  temp_int : tseginfo;
Begin
{ v2prt0.as adds 256 bytes to __stkbottom for extra safety during stack
  checking, so we subtract 256 here in order to calculate StackTop correctly
  and to ensure that StackLength = StackTop - StackBottom }
  StackLength := CheckInitialStkLen(InitialStkLen)-256;
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
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitSystemThreads;
{$endif}
{$ifdef  EXCEPTIONS_IN_SYSTEM}
  InitDPMIExcp;
  InstallDefaultHandlers;
{$endif  EXCEPTIONS_IN_SYSTEM}
End.
