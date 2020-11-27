{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl and Pavel Ozerski
    and Yury Sidorov member of the Free Pascal development team.

    FPC Pascal system unit for the WinCE.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;

interface

{$define FPC_IS_SYSTEM}
{$ifdef SYSTEMDEBUG}
  {$define SYSTEMEXCEPTIONDEBUG}
{$endif SYSTEMDEBUG}

{$define WINCE_EXCEPTION_HANDLING}
{$define DISABLE_NO_THREAD_MANAGER}
{$define HAS_CMDLINE}
{$define HAS_MEMORYMANAGER}  // comment this line to switch from wincemm to fpcmm
{$define HAS_WIDESTRINGMANAGER}
{$define DISABLE_NO_DYNLIBS_MANAGER}
{$define FPC_SYSTEM_HAS_SYSDLH}


{ include system-independent routine headers }
{$I systemh.inc}

const
 LineEnding = #13#10;
 LFNSupport = true;
 DirectorySeparator = '\';
 DriveSeparator = ':';
 ExtensionSeparator = '.';
 PathSeparator = ';';
 AllowDirectorySeparators : set of char = ['\','/'];
 AllowDriveSeparators : set of char = [':'];
{ FileNameCaseSensitive and FileNameCasePreserving are defined separately below!!! }
 maxExitCode = 65535;
 MaxPathLen = 260;
 AllFilesMask = '*';

const
{ Default filehandles }
  UnusedHandle    : THandle = THandle(-1);
  StdInputHandle  : THandle = 0;
  StdOutputHandle : THandle = 0;
  StdErrorHandle  : THandle = 0;

  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = true;
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

var
{ WinCE Info }
  hprevinst,
  MainInstance,
  DLLreason,DLLparam:DWord;

type
  TDLL_Entry_Hook = procedure (dllparam : longint);

const
  Dll_Process_Detach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Attach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Detach_Hook : TDLL_Entry_Hook = nil;

{ ANSI <-> Wide }
function AnsiToWideBuf(AnsiBuf: PChar; AnsiBufLen: longint; WideBuf: PWideChar; WideBufLen: longint): longint;
function WideToAnsiBuf(WideBuf: PWideChar; WideCharsLen: longint; AnsiBuf: PChar; AnsiBufLen: longint): longint;
function PCharToPWideChar(str: PChar; strlen: longint = -1; outlen: PLongInt = nil): PWideChar;
function StringToPWideChar(const s: AnsiString; outlen: PLongInt = nil): PWideChar;

{ Wrappers for some WinAPI calls }
function  CreateEvent(lpEventAttributes:pointer;bManualReset:longbool;bInitialState:longbool;lpName:pchar): THandle;
function ResetEvent(h: THandle): LONGBOOL;
function SetEvent(h: THandle): LONGBOOL;
function GetCurrentProcessId:DWORD;
function Win32GetCurrentThreadId:DWORD;
function TlsAlloc : DWord;
function TlsFree(dwTlsIndex : DWord) : LongBool;


function GetFileAttributesW(p : pwidechar) : dword;
    cdecl; external KernelDLL name 'GetFileAttributesW';
function DeleteFileW(p : pwidechar) : longint;
    cdecl; external KernelDLL name 'DeleteFileW';
function MoveFileW(old,_new : pwidechar) : longint;
    cdecl; external KernelDLL name 'MoveFileW';
function CreateFileW(lpFileName:pwidechar; dwDesiredAccess:DWORD; dwShareMode:DWORD;
                   lpSecurityAttributes:pointer; dwCreationDisposition:DWORD;
                   dwFlagsAndAttributes:DWORD; hTemplateFile:DWORD):longint;
    cdecl; external KernelDLL name 'CreateFileW';


{$ifdef CPUARM}
function addd(d1,d2 : double) : double; compilerproc;
   cdecl;external 'coredll' name '__addd';

function subd(d1,d2 : double) : double; compilerproc;
   cdecl;external 'coredll' name '__subd';

function muld(d1,d2 : double) : double; compilerproc;
   cdecl;external 'coredll' name '__muld';

function divd(d1,d2 : double) : double; compilerproc;
   cdecl;external 'coredll' name '__divd';

function eqd(d1,d2 : double) : boolean; compilerproc;
   cdecl;external 'coredll' name '__eqd';

function ned(d1,d2 : double) : boolean; compilerproc;
   cdecl;external 'coredll' name '__ned';

function ltd(d1,d2 : double) : boolean; compilerproc;
   cdecl;external 'coredll' name '__ltd';

function gtd(d1,d2 : double) : boolean; compilerproc;
   cdecl;external 'coredll' name '__gtd';

function ged(d1,d2 : double) : boolean; compilerproc;
   cdecl;external 'coredll' name '__ged';

function led(d1,d2 : double) : boolean; compilerproc;
   cdecl;external 'coredll' name '__led';

{ ***************** single ******************** }

function eqs(d1,d2 : single) : boolean; compilerproc;
   cdecl;external 'coredll' name '__eqs';

function nes(d1,d2 : single) : boolean; compilerproc;
   cdecl;external 'coredll' name '__nes';

function lts(d1,d2 : single) : boolean; compilerproc;
   cdecl;external 'coredll' name '__lts';

function gts(d1,d2 : single) : boolean; compilerproc;
   cdecl;external 'coredll' name '__gts';

function ges(d1,d2 : single) : boolean; compilerproc;
   cdecl;external 'coredll' name '__ges';

function les(d1,d2 : single) : boolean; compilerproc;
   cdecl;external 'coredll' name '__les';

function dtos(d : double) : single; compilerproc;
   cdecl;external 'coredll' name '__dtos';

function stod(d : single) : double; compilerproc;
   cdecl;external 'coredll' name '__stod';

function negs(d : single) : single; compilerproc;
   cdecl;external 'coredll' name '__negs';

function negd(d : double) : double; compilerproc;
   cdecl;external 'coredll' name '__negd';

function utod(i : dword) : double; compilerproc;
   cdecl;external 'coredll' name '__utod';

function itod(i : longint) : double; compilerproc;
   cdecl;external 'coredll' name '__itod';

function ui64tod(i : qword) : double; compilerproc;
   cdecl;external 'coredll' name '__u64tod';

function i64tod(i : int64) : double; compilerproc;
   cdecl;external 'coredll' name '__i64tod';

function utos(i : dword) : single; compilerproc;
   cdecl;external 'coredll' name '__utos';

function itos(i : longint) : single; compilerproc;
   cdecl;external 'coredll' name '__itos';

function ui64tos(i : qword) : single; compilerproc;
   cdecl;external 'coredll' name '__u64tos';

function i64tos(i : int64) : single; compilerproc;
   cdecl;external 'coredll' name '__i64tos';

function adds(s1,s2 : single) : single; compilerproc;
function subs(s1,s2 : single) : single; compilerproc;
function muls(s1,s2 : single) : single; compilerproc;
function divs(s1,s2 : single) : single; compilerproc;
{$endif CPUARM}

function CmdLine: PChar;
{ C compatible arguments }
function argc: longint;
function argv: ppchar;

implementation

var
  SysInstance : Longint;public name '_FPC_SysInstance';

function MessageBox(w1:longint;l1,l2:PWideChar;w2:longint):longint;
   cdecl; external 'coredll' name 'MessageBoxW';

function CreateDirectoryW(name : pwidechar;sec : pointer) : longbool;
    cdecl; external KernelDLL name 'CreateDirectoryW';
function RemoveDirectoryW(name:pwidechar):longbool;
    cdecl; external KernelDLL name 'RemoveDirectoryW';


{*****************************************************************************}

{$define FPC_SYSTEM_HAS_MOVE}
procedure memmove(dest, src: pointer; count: longint);
   cdecl; external 'coredll' name 'memmove';

procedure Move(const source;var dest;count:SizeInt);[public, alias: 'FPC_MOVE'];
begin
  if count > 0 then
    memmove(@dest, @source, count);
end;

{$define FPC_SYSTEM_HAS_COMPAREBYTE}
function memcmp(buf1, buf2: pointer; count: longint): longint;
   cdecl; external 'coredll' name 'memcmp';

function CompareByte(Const buf1,buf2;len:SizeInt):SizeInt;
begin
  CompareByte := memcmp(@buf1, @buf2, len);
end;

{$ifdef CPUARM}

{$define FPC_SYSTEM_HAS_INT}
function floor(d : double) : double;
   cdecl;external 'coredll' name 'floor';

function ceil(d : double) : double;
   cdecl;external 'coredll' name 'ceil';

function fpc_int_real(d: ValReal): ValReal;compilerproc;
begin
  if d > 0 then
    fpc_int_real:=floor(d)
  else
    fpc_int_real:=ceil(d);
end;

{$define FPC_SYSTEM_HAS_TRUNC}
function __dtoi64(d: double) : int64; cdecl; external 'coredll';

function fpc_trunc_real(d : ValReal) : int64; assembler; nostackframe; compilerproc;
asm
	b __dtoi64
end;

{$define FPC_SYSTEM_HAS_ABS}
function fabs(d: double): double; cdecl; external 'coredll';

function fpc_abs_real(d : ValReal) : ValReal; assembler; nostackframe; compilerproc;
asm
  b fabs
end;

{$define FPC_SYSTEM_HAS_SQRT}
function coresqrt(d: double): double; cdecl; external 'coredll' name 'sqrt';

function fpc_sqrt_real(d : ValReal) : ValReal; assembler; nostackframe; compilerproc;
asm
  b coresqrt
end;

function adds(s1,s2 : single) : single;
begin
  adds := double(s1) + double(s2);
end;

function subs(s1,s2 : single) : single;
begin
  subs := double(s1) - double(s2);
end;

function muls(s1,s2 : single) : single;
begin
  muls := double(s1) * double(s2);
end;

function divs(s1,s2 : single) : single;
begin
  divs := double(s1) / double(s2);
end;

{$endif CPUARM}

{*****************************************************************************}

{ include system independent routines }
{$I system.inc}

{*****************************************************************************
                              ANSI <-> Wide
*****************************************************************************}
const
  { MultiByteToWideChar  }
     MB_PRECOMPOSED = 1;
     MB_COMPOSITE = 2;
     MB_ERR_INVALID_CHARS = 8;
     MB_USEGLYPHCHARS = 4;

function MultiByteToWideChar(CodePage:UINT; dwFlags:DWORD; lpMultiByteStr:PChar; cchMultiByte:longint; lpWideCharStr:PWideChar;cchWideChar:longint):longint;
     cdecl; external 'coredll' name 'MultiByteToWideChar';
function WideCharToMultiByte(CodePage:UINT; dwFlags:DWORD; lpWideCharStr:PWideChar; cchWideChar:longint; lpMultiByteStr:PChar;cchMultiByte:longint; lpDefaultChar:PChar; lpUsedDefaultChar:pointer):longint;
     cdecl; external 'coredll' name 'WideCharToMultiByte';
function GetACP:UINT; cdecl; external 'coredll' name 'GetACP';

{ Returns number of characters stored to WideBuf, including null-terminator. }
function AnsiToWideBuf(AnsiBuf: PChar; AnsiBufLen: longint; WideBuf: PWideChar; WideBufLen: longint): longint;
begin
  Result := MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, AnsiBuf, AnsiBufLen, WideBuf, WideBufLen div SizeOf(WideChar));
  if ((AnsiBufLen <> -1) or (Result = 0)) and (WideBuf <> nil) then
  begin
    if (Result + 1)*SizeOf(WideChar) > WideBufLen then
    begin
      Result := 0;
      if WideBufLen < SizeOf(WideChar) then
        exit;
    end;
    WideBuf[Result] := #0;
  end;
  if (AnsiBufLen <> -1) and ((Result <> 0) or (AnsiBufLen = 0)) then
    Inc(Result);
end;

{ Returns number of characters stored to AnsiBuf, including null-terminator. }
function WideToAnsiBuf(WideBuf: PWideChar; WideCharsLen: longint; AnsiBuf: PChar; AnsiBufLen: longint): longint;
begin
  Result := WideCharToMultiByte(CP_ACP, 0, WideBuf, WideCharsLen, AnsiBuf, AnsiBufLen, nil, nil);
  if ((WideCharsLen <> -1) or (Result = 0)) and (AnsiBuf <> nil) then
  begin
    if Result + 1 > AnsiBufLen then
    begin
      Result := 0;
      if AnsiBufLen < 1 then
        exit;
    end;
    AnsiBuf[Result] := #0;
  end;
  if (WideCharsLen <> -1) and ((Result <> 0) or (WideCharsLen = 0)) then
    Inc(Result);
end;

{ Returns dynamic memory block, which contains wide string. This memory should be freed using FreeMem. }
{ outlen will contain number of wide characters stored to result buffer, including null-terminator. }
function PCharToPWideChar(str: PChar; strlen: longint = -1; outlen: PLongInt = nil): PWideChar;
var
  len: longint;
begin
  while True do begin
    if strlen <> -1 then
      len:=strlen + 1
    else
      len:=AnsiToWideBuf(str, -1, nil, 0);
    if len > 0 then
    begin
      len:=len*SizeOf(WideChar);
      GetMem(Result, len);
      len:=AnsiToWideBuf(str, strlen, Result, len);
      if (len = 0) and (strlen <> -1) then
      begin
        FreeMem(Result);
        strlen:=-1;
        continue;
      end;
    end
    else begin
      GetMem(Result, SizeOf(WideChar));
      len:=1;
      Result^:=#0;
    end;
    break;
  end;
  if outlen <> nil then
    outlen^:=len;
end;

{ Returns dynamic memory block, which contains wide string. This memory should be freed using FreeMem. }
{ outlen will contain number of wide characters stored to result buffer, including null-terminator. }
function StringToPWideChar(const s: AnsiString; outlen: PLongInt = nil): PWideChar;
var
  len, wlen: longint;
begin
  len:=Length(s);
  wlen:=(len + 1)*SizeOf(WideChar);
  GetMem(Result, wlen);
  wlen:=AnsiToWideBuf(PChar(s), len, Result, wlen)*SizeOf(WideChar);
  if wlen = 0 then
  begin
    wlen:=AnsiToWideBuf(PChar(s), len, nil, 0)*SizeOf(WideChar);
    if wlen > 0 then
    begin
      ReAllocMem(Result, wlen);
      wlen:=AnsiToWideBuf(PChar(s), len, Result, wlen)*SizeOf(WideChar);
    end
    else
    begin
      Result^:=#0;
      wlen:=SizeOf(WideChar);
    end;
  end;
  if outlen <> nil then
    outlen^:=wlen div SizeOf(WideChar);
end;

{*****************************************************************************
                      WinAPI wrappers implementation
*****************************************************************************}

const
{$ifdef CPUARM}
  UserKData = $FFFFC800;
{$else CPUARM}
  UserKData = $00005800;
{$endif CPUARM}
  SYSHANDLE_OFFSET = $004;
  SYS_HANDLE_BASE  = 64;
  SH_CURTHREAD     = 1;
  SH_CURPROC       = 2;

type
  PHandle = ^THandle;

const
  EVENT_PULSE =     1;
  EVENT_RESET =     2;
  EVENT_SET   =     3;

function CreateEventW(lpEventAttributes:pointer;bManualReset:longbool;bInitialState:longbool;lpName:PWideChar): THandle;
    cdecl; external KernelDLL name 'CreateEventW';

function CreateEvent(lpEventAttributes:pointer;bManualReset:longbool;bInitialState:longbool;lpName:pchar): THandle;
var
  buf: array[0..MaxPathLen] of WideChar;
begin
  if lpName=nil then
    CreateEvent := CreateEventW(lpEventAttributes, bManualReset, bInitialState, nil)
  else begin
    AnsiToWideBuf(lpName, -1, buf, SizeOf(buf));
    CreateEvent := CreateEventW(lpEventAttributes, bManualReset, bInitialState, buf);
  end;
end;

function EventModify(h: THandle; func: DWORD): LONGBOOL;
     cdecl; external KernelDLL name 'EventModify';
function TlsCall(p1, p2: DWORD): DWORD;
     cdecl; external KernelDLL name 'TlsCall';

function ResetEvent(h: THandle): LONGBOOL;
begin
	ResetEvent := EventModify(h,EVENT_RESET);
end;

function SetEvent(h: THandle): LONGBOOL;
begin
	SetEvent := EventModify(h,EVENT_SET);
end;

function GetCurrentProcessId:DWORD;
var
  p: PHandle;
begin
  p:=PHandle(UserKData+SYSHANDLE_OFFSET + SH_CURPROC*SizeOf(THandle));
  GetCurrentProcessId := p^;
end;

function Win32GetCurrentThreadId:DWORD;
var
  p: PHandle;
begin
  p:=PHandle(UserKData+SYSHANDLE_OFFSET + SH_CURTHREAD*SizeOf(THandle));
  Win32GetCurrentThreadId := p^;
end;

const
  TLS_FUNCALLOC = 0;
  TLS_FUNCFREE  = 1;

function TlsAlloc : DWord;
begin
  TlsAlloc := TlsCall(TLS_FUNCALLOC, 0);
end;

function TlsFree(dwTlsIndex : DWord) : LongBool;
begin
  TlsFree := LongBool(TlsCall(TLS_FUNCFREE, dwTlsIndex));
end;

{*****************************************************************************
                              Parameter Handling
*****************************************************************************}

function GetCommandLine : pwidechar;
    cdecl; external KernelDLL name 'GetCommandLineW';

var
  ModuleName : array[0..255] of char;

function GetCommandFile:pchar;
var
  buf: array[0..MaxPathLen] of WideChar;
begin
  if ModuleName[0] = #0 then begin
    GetModuleFileName(0, @buf, SizeOf(buf));
    WideToAnsiBuf(buf, -1, @ModuleName, SizeOf(ModuleName));
  end;
  GetCommandFile:=@ModuleName;
end;

var
  Fargc: longint;
  Fargv: ppchar;
  FCmdLine: PChar;

procedure setup_arguments;
var
  arglen,
  count   : longint;
  argstart,
  pc,arg  : pchar;
  quote   : char;
  argvlen : longint;

  procedure allocarg(idx,len:longint);
    var
      oldargvlen : longint;
    begin
      if idx>=argvlen then
       begin
         oldargvlen:=argvlen;
         argvlen:=(idx+8) and (not 7);
         sysreallocmem(Fargv,argvlen*sizeof(pointer));
         fillchar(Fargv[oldargvlen],(argvlen-oldargvlen)*sizeof(pointer),0);
       end;
      { use realloc to reuse already existing memory }
      { always allocate, even if length is zero, since }
      { the arg. is still present!                     }
      sysreallocmem(Fargv[idx],len+1);
    end;

begin
  { create commandline, it starts with the executed filename which is argv[0] }
  { WinCE passes the command NOT via the args, but via getmodulefilename}
  if FCmdLine <> nil then exit;
  argvlen:=0;
  pc:=getcommandfile;
  Arglen:=0;
  while pc[Arglen] <> #0 do
    Inc(Arglen);
  allocarg(0,arglen);
  move(pc^,Fargv[0]^,arglen+1);
  { Setup FCmdLine variable }
  arg:=PChar(GetCommandLine);
  count:=WideToAnsiBuf(PWideChar(arg), -1, nil, 0);
  FCmdLine:=SysGetMem(arglen + count + 3);
  FCmdLine^:='"';
  move(pc^, (FCmdLine + 1)^, arglen);
  (FCmdLine + arglen + 1)^:='"';
  (FCmdLine + arglen + 2)^:=' ';
  WideToAnsiBuf(PWideChar(arg), -1, FCmdLine + arglen + 3, count);
  { process arguments }
  count:=0;
  pc:=FCmdLine;
{$IfDef SYSTEM_DEBUG_STARTUP}
  Writeln(stderr,'WinCE GetCommandLine is #',pc,'#');
{$EndIf }
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
     { Don't copy the first one, it is already there.}
     If Count<>0 then
      begin
        allocarg(count,arglen);
        quote:=' ';
        pc:=argstart;
        arg:=Fargv[count];
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
      end;
 {$IfDef SYSTEM_DEBUG_STARTUP}
     Writeln(stderr,'dos arg ',count,' #',arglen,'#',Fargv[count],'#');
 {$EndIf SYSTEM_DEBUG_STARTUP}
     inc(count);
   end;
  { get argc and create an nil entry }
  Fargc:=count;
  allocarg(argc,0);
  { free unused memory }
  sysreallocmem(Fargv,(argc+1)*sizeof(pointer));
end;

function CmdLine: PChar;
begin
  setup_arguments;
  Result:=FCmdLine;
end;

function argc: longint;
begin
  setup_arguments;
  Result:=Fargc;
end;

function argv: ppchar;
begin
  setup_arguments;
  Result:=Fargv;
end;

function paramcount : longint;
begin
  paramcount := argc - 1;
end;

function paramstr(l : longint) : string;
begin
  setup_arguments;
  if (l>=0) and (l<Fargc) then
    paramstr:=strpas(Fargv[l])
  else
    paramstr:='';
end;

procedure randomize;
begin
  randseed:=GetTickCount;
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure PascalMain;external name 'PASCALMAIN';
procedure ExitThread(Exitcode : longint); cdecl; external 'coredll';

Procedure system_exit;
begin
  if IsLibrary then
    exit;
  if not IsConsole then
    begin
      Close(stderr);
      Close(stdout);
      Close(erroutput);
      Close(Input);
      Close(Output);
  end;
  ExitThread(exitcode);
end;

{$ifdef cpu386}
var
  { value of the stack segment
    to check if the call stack can be written on exceptions }
  _SS : Cardinal;
{$endif cpu386}

procedure fpc_lib_exit_intern; external name 'FPC_LIB_EXIT';

function Dll_entry : longbool;[public, alias : '_FPC_DLL_Entry'];
begin
   IsLibrary:=true;
   Dll_entry:=false;
   case DLLreason of
     DLL_PROCESS_ATTACH :
       begin
         PASCALMAIN;
         Dll_entry:=true;
       end;
     DLL_THREAD_ATTACH :
       begin
{ Allocate Threadvars ?!}
         if assigned(Dll_Thread_Attach_Hook) then
           Dll_Thread_Attach_Hook(DllParam);
       end;
     DLL_THREAD_DETACH :
       begin
         if assigned(Dll_Thread_Detach_Hook) then
           Dll_Thread_Detach_Hook(DllParam);
{ Release Threadvars ?!}
       end;
     DLL_PROCESS_DETACH :
       begin
         Fpc_Lib_Exit_intern;
         if assigned(Dll_Process_Detach_Hook) then
           Dll_Process_Detach_Hook(DllParam);
       end;
   end;
end;

{$ifdef WINCE_EXCEPTION_HANDLING}

//
// Hardware exception handling
//

{
  Error code definitions for the WinCE API functions


  Values are 32 bit values layed out as follows:
   3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
  +---+-+-+-----------------------+-------------------------------+
  |Sev|C|R|     Facility          |               Code            |
  +---+-+-+-----------------------+-------------------------------+

  where
      Sev - is the severity code
          00 - Success
          01 - Informational
          10 - Warning
          11 - Error

      C - is the Customer code flag
      R - is a reserved bit
      Facility - is the facility code
      Code - is the facility's status code
}

const
  SEVERITY_SUCCESS        = $00000000;
  SEVERITY_INFORMATIONAL  = $40000000;
  SEVERITY_WARNING        = $80000000;
  SEVERITY_ERROR          = $C0000000;

const
  STATUS_SEGMENT_NOTIFICATION             = $40000005;
  DBG_TERMINATE_THREAD                    = $40010003;
  DBG_TERMINATE_PROCESS                   = $40010004;
  DBG_CONTROL_C                           = $40010005;
  DBG_CONTROL_BREAK                       = $40010008;

  STATUS_GUARD_PAGE_VIOLATION             = $80000001;
  STATUS_DATATYPE_MISALIGNMENT            = $80000002;
  STATUS_BREAKPOINT                       = $80000003;
  STATUS_SINGLE_STEP                      = $80000004;
  DBG_EXCEPTION_NOT_HANDLED               = $80010001;

  STATUS_ACCESS_VIOLATION                 = $C0000005;
  STATUS_IN_PAGE_ERROR                    = $C0000006;
  STATUS_INVALID_HANDLE                   = $C0000008;
  STATUS_NO_MEMORY                        = $C0000017;
  STATUS_ILLEGAL_INSTRUCTION              = $C000001D;
  STATUS_NONCONTINUABLE_EXCEPTION         = $C0000025;
  STATUS_INVALID_DISPOSITION              = $C0000026;
  STATUS_ARRAY_BOUNDS_EXCEEDED            = $C000008C;
  STATUS_FLOAT_DENORMAL_OPERAND           = $C000008D;
  STATUS_FLOAT_DIVIDE_BY_ZERO             = $C000008E;
  STATUS_FLOAT_INEXACT_RESULT             = $C000008F;
  STATUS_FLOAT_INVALID_OPERATION          = $C0000090;
  STATUS_FLOAT_OVERFLOW                   = $C0000091;
  STATUS_FLOAT_STACK_CHECK                = $C0000092;
  STATUS_FLOAT_UNDERFLOW                  = $C0000093;
  STATUS_INTEGER_DIVIDE_BY_ZERO           = $C0000094;
  STATUS_INTEGER_OVERFLOW                 = $C0000095;
  STATUS_PRIVILEGED_INSTRUCTION           = $C0000096;
  STATUS_STACK_OVERFLOW                   = $C00000FD;
  STATUS_CONTROL_C_EXIT                   = $C000013A;
  STATUS_FLOAT_MULTIPLE_FAULTS            = $C00002B4;
  STATUS_FLOAT_MULTIPLE_TRAPS             = $C00002B5;
  STATUS_REG_NAT_CONSUMPTION              = $C00002C9;

const
  ExceptionContinueExecution = 0;
  ExceptionContinueSearch = 1;
  ExceptionNestedException = 2;
  ExceptionCollidedUnwind = 3;
  ExceptionExecuteHandler = 4;

  MaxExceptionLevel = 16;
  exceptLevel : Byte = 0;

{$ifdef CPUARM}
const
  CONTEXT_ARM                     = $0000040;
  CONTEXT_CONTROL                 = CONTEXT_ARM or $00000001;
  CONTEXT_INTEGER                 = CONTEXT_ARM or $00000002;
  CONTEXT_SEGMENTS                = CONTEXT_ARM or $00000004;
  CONTEXT_FLOATING_POINT          = CONTEXT_ARM or $00000008;
  CONTEXT_DEBUG_REGISTERS         = CONTEXT_ARM or $00000010;
  CONTEXT_EXTENDED_REGISTERS      = CONTEXT_ARM or $00000020;

  CONTEXT_FULL                    = CONTEXT_CONTROL or CONTEXT_INTEGER or CONTEXT_SEGMENTS;

  NUM_VFP_REGS = 32;
  NUM_EXTRA_CONTROL_REGS = 8;

type
  PContext = ^TContext;
  TContext = record
    ContextFlags : LongWord;
// This section is specified/returned if the ContextFlags word contains
// the flag CONTEXT_INTEGER.
    R0 : LongWord;
    R1 : LongWord;
    R2 : LongWord;
    R3 : LongWord;
    R4 : LongWord;
    R5 : LongWord;
    R6 : LongWord;
    R7 : LongWord;
    R8 : LongWord;
    R9 : LongWord;
    R10 : LongWord;
    R11 : LongWord;
    R12 : LongWord;
// This section is specified/returned if the ContextFlags word contains
// the flag CONTEXT_CONTROL.
    Sp : LongWord;
    Lr : LongWord;
    Pc : LongWord;
    Psr : LongWord;
    Fpscr : LongWord;
    FpExc : LongWord;
// Floating point registers
    S : array[0..(NUM_VFP_REGS + 1)-1] of LongWord;
    FpExtra : array[0..(NUM_EXTRA_CONTROL_REGS)-1] of LongWord;
  end;
{$endif CPUARM}

{$ifdef CPUI386}
const
  CONTEXT_X86                     = $00010000;
  CONTEXT_CONTROL                 = CONTEXT_X86 or $00000001;
  CONTEXT_INTEGER                 = CONTEXT_X86 or $00000002;
  CONTEXT_SEGMENTS                = CONTEXT_X86 or $00000004;
  CONTEXT_FLOATING_POINT          = CONTEXT_X86 or $00000008;
  CONTEXT_DEBUG_REGISTERS         = CONTEXT_X86 or $00000010;
  CONTEXT_EXTENDED_REGISTERS      = CONTEXT_X86 or $00000020;

  MAXIMUM_SUPPORTED_EXTENSION     = 512;

type
  PFloatingSaveArea = ^TFloatingSaveArea;
  TFloatingSaveArea = packed record
    ControlWord : Cardinal;
    StatusWord : Cardinal;
    TagWord : Cardinal;
    ErrorOffset : Cardinal;
    ErrorSelector : Cardinal;
    DataOffset : Cardinal;
    DataSelector : Cardinal;
    RegisterArea : array[0..79] of Byte;
    Cr0NpxState : Cardinal;
  end;

  PContext = ^TContext;
  TContext = packed record
      //
      // The flags values within this flag control the contents of
      // a CONTEXT record.
      //
          ContextFlags : Cardinal;

      //
      // This section is specified/returned if CONTEXT_DEBUG_REGISTERS is
      // set in ContextFlags.  Note that CONTEXT_DEBUG_REGISTERS is NOT
      // included in CONTEXT_FULL.
      //
          Dr0, Dr1, Dr2,
          Dr3, Dr6, Dr7 : Cardinal;

      //
      // This section is specified/returned if the
      // ContextFlags word contains the flag CONTEXT_FLOATING_POINT.
      //
          FloatSave : TFloatingSaveArea;

      //
      // This section is specified/returned if the
      // ContextFlags word contains the flag CONTEXT_SEGMENTS.
      //
          SegGs, SegFs,
          SegEs, SegDs : Cardinal;

      //
      // This section is specified/returned if the
      // ContextFlags word contains the flag CONTEXT_INTEGER.
      //
          Edi, Esi, Ebx,
          Edx, Ecx, Eax : Cardinal;

      //
      // This section is specified/returned if the
      // ContextFlags word contains the flag CONTEXT_CONTROL.
      //
          Ebp : Cardinal;
          Eip : Cardinal;
          SegCs : Cardinal;
          EFlags, Esp, SegSs : Cardinal;

      //
      // This section is specified/returned if the ContextFlags word
      // contains the flag CONTEXT_EXTENDED_REGISTERS.
      // The format and contexts are processor specific
      //
          ExtendedRegisters : array[0..MAXIMUM_SUPPORTED_EXTENSION-1] of Byte;
  end;
{$endif CPUI386}

type
  PExceptionPointers = ^TExceptionPointers;
  TExceptionPointers = packed record
    ExceptionRecord   : PExceptionRecord;
    ContextRecord     : PContext;
  end;

{$ifdef CPUI386}
{**************************** i386 Exception handling *****************************************}

function GetCurrentProcess:DWORD;
begin
  GetCurrentProcess := SH_CURPROC+SYS_HANDLE_BASE;
end;

function ReadProcessMemory(process : dword;address : pointer;dest : pointer;size : dword;bytesread : pdword) :  longbool;
   cdecl; external 'coredll' name 'ReadProcessMemory';

function is_prefetch(p : pointer) : boolean;
var
  a : array[0..15] of byte;
  doagain : boolean;
  instrlo,instrhi,opcode : byte;
  i : longint;
begin
  result:=false;
  { read memory savely without causing another exeception }
  if not(ReadProcessMemory(GetCurrentProcess,p,@a,sizeof(a),nil)) then
    exit;
  i:=0;
  doagain:=true;
  while doagain and (i<15) do
    begin
      opcode:=a[i];
      instrlo:=opcode and $f;
      instrhi:=opcode and $f0;
      case instrhi of
        { prefix? }
        $20,$30:
          doagain:=(instrlo and 7)=6;
        $60:
          doagain:=(instrlo and $c)=4;
        $f0:
          doagain:=instrlo in [0,2,3];
        $0:
          begin
            result:=(instrlo=$f) and (a[i+1] in [$d,$18]);
            exit;
          end;
        else
          doagain:=false;
      end;
      inc(i);
    end;
end;

var
  exceptEip       : array[0..MaxExceptionLevel-1] of Longint;
  exceptError     : array[0..MaxExceptionLevel-1] of Byte;
  resetFPU        : array[0..MaxExceptionLevel-1] of Boolean;

{$ifdef SYSTEMEXCEPTIONDEBUG}
procedure DebugHandleErrorAddrFrame(error, addr, frame : longint);
begin
  if IsConsole then
    begin
      write(stderr,'HandleErrorAddrFrame(error=',error);
      write(stderr,',addr=',hexstr(addr,8));
      writeln(stderr,',frame=',hexstr(frame,8),')');
    end;
  HandleErrorAddrFrame(error,addr,frame);
end;
{$endif SYSTEMEXCEPTIONDEBUG}

procedure JumpToHandleErrorFrame;
var
  eip, ebp, error : Longint;
begin
  // save ebp
  asm
    movl (%ebp),%eax
    movl %eax,ebp
  end;
  if (exceptLevel > 0) then
    dec(exceptLevel);

  eip:=exceptEip[exceptLevel];
  error:=exceptError[exceptLevel];
{$ifdef SYSTEMEXCEPTIONDEBUG}
  if IsConsole then
    writeln(stderr,'In JumpToHandleErrorFrame error=',error);
{$endif SYSTEMEXCEPTIONDEBUG}
  if resetFPU[exceptLevel] then
    SysResetFPU;
  { build a fake stack }
  asm
{$ifdef REGCALL}
    movl   ebp,%ecx
    movl   eip,%edx
    movl   error,%eax
    pushl  eip
    movl   ebp,%ebp // Change frame pointer
{$else}
    movl   ebp,%eax
    pushl  %eax
    movl   eip,%eax
    pushl  %eax
    movl   error,%eax
    pushl  %eax
    movl   eip,%eax
    pushl  %eax
    movl   ebp,%ebp // Change frame pointer
{$endif}

{$ifdef SYSTEMEXCEPTIONDEBUG}
    jmpl   DebugHandleErrorAddrFrame
{$else not SYSTEMEXCEPTIONDEBUG}
    jmpl   HandleErrorAddrFrame
{$endif SYSTEMEXCEPTIONDEBUG}
  end;
end;

function i386_exception_handler(ExceptionRecord: PExceptionRecord;
	    EstablisherFrame: pointer; ContextRecord: PContext;
	    DispatcherContext: pointer): longint; cdecl;
var
  res: longint;
  must_reset_fpu: boolean;
begin
  res := ExceptionContinueSearch;
  if ContextRecord^.SegSs=_SS then begin
    must_reset_fpu := true;
  {$ifdef SYSTEMEXCEPTIONDEBUG}
    if IsConsole then Writeln(stderr,'Exception  ',
            hexstr(excep^.ExceptionRecord^.ExceptionCode, 8));
  {$endif SYSTEMEXCEPTIONDEBUG}
    case cardinal(ExceptionRecord^.ExceptionCode) of
      STATUS_INTEGER_DIVIDE_BY_ZERO,
      STATUS_FLOAT_DIVIDE_BY_ZERO :
        res := 208;
      STATUS_ARRAY_BOUNDS_EXCEEDED :
        begin
          res := 201;
          must_reset_fpu := false;
        end;
      STATUS_STACK_OVERFLOW :
        begin
          res := 202;
          must_reset_fpu := false;
        end;
      STATUS_FLOAT_OVERFLOW :
        res := 205;
      STATUS_FLOAT_DENORMAL_OPERAND,
      STATUS_FLOAT_UNDERFLOW :
        res := 206;
  {excep^.ContextRecord^.FloatSave.StatusWord := excep^.ContextRecord^.FloatSave.StatusWord and $ffffff00;}
      STATUS_FLOAT_INEXACT_RESULT,
      STATUS_FLOAT_INVALID_OPERATION,
      STATUS_FLOAT_STACK_CHECK :
        res := 207;
      STATUS_INTEGER_OVERFLOW :
        begin
          res := 215;
          must_reset_fpu := false;
        end;
      STATUS_ILLEGAL_INSTRUCTION:
        res := 216;
      STATUS_ACCESS_VIOLATION:
        { Athlon prefetch bug? }
        if is_prefetch(pointer(ContextRecord^.Eip)) then
          begin
            { if yes, then retry }
            ExceptionRecord^.ExceptionCode := 0;
            res:=ExceptionContinueExecution;
          end
        else
          res := 216;

      STATUS_CONTROL_C_EXIT:
        res := 217;
      STATUS_PRIVILEGED_INSTRUCTION:
        begin
          res := 218;
          must_reset_fpu := false;
        end;
      else
        begin
          if ((ExceptionRecord^.ExceptionCode and SEVERITY_ERROR) = SEVERITY_ERROR) then
            res := 217
          else
            res := 255;
        end;
    end;

    if (res >= 200) and (exceptLevel < MaxExceptionLevel) then begin
      exceptEip[exceptLevel] := ContextRecord^.Eip;
      exceptError[exceptLevel] := res;
      resetFPU[exceptLevel] := must_reset_fpu;
      inc(exceptLevel);

      ContextRecord^.Eip := Longint(@JumpToHandleErrorFrame);
      ExceptionRecord^.ExceptionCode := 0;

      res := ExceptionContinueExecution;
    {$ifdef SYSTEMEXCEPTIONDEBUG}
      if IsConsole then begin
        writeln(stderr,'Exception Continue Exception set at ',
                hexstr(exceptEip[exceptLevel],8));
        writeln(stderr,'Eip changed to ',
                hexstr(longint(@JumpToHandleErrorFrame),8), ' error=', error);
      end;
    {$endif SYSTEMEXCEPTIONDEBUG}
    end;
  end;
  i386_exception_handler := res;
end;

{$endif CPUI386}

{$ifdef CPUARM}
{**************************** ARM Exception handling *****************************************}

var
  exceptPC        : array[0..MaxExceptionLevel-1] of Longint;
  exceptError     : array[0..MaxExceptionLevel-1] of Byte;

procedure JumpToHandleErrorFrame;
var
  _pc, _fp, _error : Longint;
begin
  // get original fp
  asm
    ldr r0,[r11,#-12]
    str r0,_fp
  end;
  if (exceptLevel > 0) then
    dec(exceptLevel);

  _pc:=exceptPC[exceptLevel];
  _error:=exceptError[exceptLevel];
  asm
    ldr r0,_error
    ldr r1,_pc
    ldr r2,_fp
    mov r11,r2              // Change frame pointer
    b HandleErrorAddrFrame
  end;
end;

function ARM_ExceptionHandler(ExceptionRecord: PExceptionRecord;
	    EstablisherFrame: pointer; ContextRecord: PContext;
	    DispatcherContext: pointer): longint; [public, alias : '_ARM_ExceptionHandler'];
var
  res: longint;
begin
  res := ExceptionContinueSearch;

  case cardinal(ExceptionRecord^.ExceptionCode) of
    STATUS_INTEGER_DIVIDE_BY_ZERO,
    STATUS_FLOAT_DIVIDE_BY_ZERO :
      res := 200;
    STATUS_ARRAY_BOUNDS_EXCEEDED :
      res := 201;
    STATUS_STACK_OVERFLOW :
      res := 202;
    STATUS_FLOAT_OVERFLOW :
      res := 205;
    STATUS_FLOAT_DENORMAL_OPERAND,
    STATUS_FLOAT_UNDERFLOW :
      res := 206;
    STATUS_FLOAT_INEXACT_RESULT,
    STATUS_FLOAT_INVALID_OPERATION,
    STATUS_FLOAT_STACK_CHECK :
      res := 207;
    STATUS_INTEGER_OVERFLOW :
      res := 215;
    STATUS_ILLEGAL_INSTRUCTION:
      res := 216;
    STATUS_ACCESS_VIOLATION:
      res := 216;
    STATUS_DATATYPE_MISALIGNMENT:
      res := 214;
    STATUS_CONTROL_C_EXIT:
      res := 217;
    STATUS_PRIVILEGED_INSTRUCTION:
      res := 218;
    else
      begin
        if ((cardinal(ExceptionRecord^.ExceptionCode) and SEVERITY_ERROR) = SEVERITY_ERROR) then
          res := 217
        else
          res := 255;
      end;
  end;

  if (res <> ExceptionContinueSearch) and (exceptLevel < MaxExceptionLevel) then begin
    exceptPC[exceptLevel] := ContextRecord^.PC;
    exceptError[exceptLevel] := res;
    inc(exceptLevel);

    ContextRecord^.PC := Longint(@JumpToHandleErrorFrame);
    ExceptionRecord^.ExceptionCode := 0;

    res := ExceptionContinueExecution;
  {$ifdef SYSTEMEXCEPTIONDEBUG}
    if IsConsole then begin
      writeln(stderr,'Exception Continue Exception set at ',
              hexstr(exceptEip[exceptLevel],8));
      writeln(stderr,'Eip changed to ',
              hexstr(longint(@JumpToHandleErrorFrame),8), ' error=', error);
    end;
  {$endif SYSTEMEXCEPTIONDEBUG}
  end;
  ARM_ExceptionHandler := res;
end;

{$endif CPUARM}

{$endif WINCE_EXCEPTION_HANDLING}

procedure Exe_entry;[public, alias : '_FPC_EXE_Entry'];
begin
  IsLibrary:=false;
{$ifdef CPUARM}
  asm
    mov fp,#0
    bl PASCALMAIN;
  end;
{$endif CPUARM}

{$ifdef CPUI386}
  asm
  {$ifdef WINCE_EXCEPTION_HANDLING}
    pushl i386_exception_handler
    pushl %fs:(0)
    mov %esp,%fs:(0)
  {$endif WINCE_EXCEPTION_HANDLING}
    pushl %ebp
    xorl %eax,%eax
    movw %ss,%ax
    movl %eax,_SS
    xorl %ebp,%ebp
    call PASCALMAIN
    popl %ebp
 {$ifdef WINCE_EXCEPTION_HANDLING}
    popl %fs:(0)
    addl $4, %esp
  {$endif WINCE_EXCEPTION_HANDLING}
  end;
{$endif CPUI386}
end;

procedure _FPC_mainCRTStartup;public name '_mainCRTStartup';
begin
  IsConsole:=True;
  Exe_entry;
end;

procedure _FPC_WinMainCRTStartup;public name '_WinMainCRTStartup';
begin
  IsConsole:=False;
  Exe_entry;
end;

procedure _FPC_DLLMainCRTStartup(_hinstance,_dllreason,_dllparam:longint);public name '_DLLMainCRTStartup';
begin
  IsConsole:=true;
  sysinstance:=_hinstance;
  dllreason:=_dllreason;
  dllparam:=_dllparam;
  DLL_Entry;
end;


procedure _FPC_DLLWinMainCRTStartup(_hinstance,_dllreason,_dllparam:longint);public name '_DLLWinMainCRTStartup';
begin
  IsConsole:=false;
  sysinstance:=_hinstance;
  dllreason:=_dllreason;
  dllparam:=_dllparam;
  DLL_Entry;
end;

{****************************************************************************
                      OS dependend widestrings
****************************************************************************}

function CharUpperBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD; cdecl; external KernelDLL name 'CharUpperBuffW';
function CharLowerBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD; cdecl; external KernelDLL name 'CharLowerBuffW';


procedure WinCEWide2AnsiMove(source:pwidechar;var dest:RawByteString;cp:TSystemCodePage;len:SizeInt);
  var
    destlen: SizeInt;
  begin
    // retrieve length including trailing #0
    // not anymore, because this must also be usable for single characters
    destlen:=WideCharToMultiByte(cp, 0, source, len, nil, 0, nil, nil);
    // this will null-terminate
    setlength(dest, destlen);
    if destlen>0 then
      begin
        WideCharToMultiByte(cp, 0, source, len, @dest[1], destlen, nil, nil);
        PAnsiRec(pointer(dest)-AnsiFirstOff)^.CodePage:=cp;
      end;
  end;

procedure WinCEAnsi2WideMove(source:pchar;cp:TSystemCodePage;var dest:widestring;len:SizeInt);
  var
    destlen: SizeInt;
    dwFlags: DWORD;
  begin
    if cp=CP_UTF8 then
      dwFlags:=0
    else
      dwFlags:=MB_PRECOMPOSED;
    destlen:=MultiByteToWideChar(cp, dwFlags, source, len, nil, 0);
    // this will null-terminate
    setlength(dest, destlen);
    if destlen>0 then
      MultiByteToWideChar(cp, dwFlags, source, len, @dest[1], destlen);
  end;

function WinCEWideUpper(const s : WideString) : WideString;
  begin
    result:=s;
    UniqueString(result);
    if length(result)>0 then
      CharUpperBuff(LPWSTR(result),length(result));
  end;


function WinCEWideLower(const s : WideString) : WideString;
  begin
    result:=s;
    UniqueString(result);
    if length(result)>0 then
      CharLowerBuff(LPWSTR(result),length(result));
  end;

procedure WinCEUnicode2AnsiMove(source:punicodechar;var dest:RawByteString;cp:TSystemCodePage;len:SizeInt);
  var
    destlen: SizeInt;
  begin
    // retrieve length including trailing #0
    // not anymore, because this must also be usable for single characters
    destlen:=WideCharToMultiByte(cp, 0, source, len, nil, 0, nil, nil);
    // this will null-terminate
    setlength(dest, destlen);
    if destlen>0 then
      begin
        WideCharToMultiByte(cp, 0, source, len, @dest[1], destlen, nil, nil);
        PAnsiRec(pointer(dest)-AnsiFirstOff)^.CodePage:=cp;
      end;
  end;

procedure WinCEAnsi2UnicodeMove(source:pchar;cp : TSystemCodePage;var dest:UnicodeString;len:SizeInt);
  var
    destlen: SizeInt;
    dwflags: DWORD;
  begin
    if cp=CP_UTF8 then
      dwFlags:=0
    else
      dwFlags:=MB_PRECOMPOSED;
    destlen:=MultiByteToWideChar(cp, dwFlags, source, len, nil, 0);
    // this will null-terminate
    setlength(dest, destlen);
    if destlen>0 then
      begin
        MultiByteToWideChar(cp, dwFlags, source, len, @dest[1], destlen);
        PUnicodeRec(pointer(dest)-UnicodeFirstOff)^.CodePage:=CP_UTF16;
      end;
  end;

function WinCEUnicodeUpper(const s : UnicodeString) : UnicodeString;
begin
  Result:=WinCEWideUpper(s);
end;

function WinCEUnicodeLower(const s : UnicodeString) : UnicodeString;
begin
  Result:=WinCEWideLower(s);
end;

function WinCEGetStandardCodePage(const stdcp: TStandardCodePageEnum): TSystemCodePage;
  begin
    case stdcp of
      scpAnsi: Result := GetACP;
      scpConsoleInput: Result := GetACP;
      scpConsoleOutput: Result := GetACP;
      { all of WinCE's file APIs are based on UTF8 -> prevent data loss when using
        single byte strings }
      scpFileSystemSingleByte: Result := CP_UTF8;
    end;
  end;

{ there is a similiar procedure in sysutils which inits the fields which
  are only relevant for the sysutils units }
procedure InitWinCEWidestrings;
  begin
    widestringmanager.Wide2AnsiMoveProc:=@WinCEWide2AnsiMove;
    widestringmanager.Ansi2WideMoveProc:=@WinCEAnsi2WideMove;
    widestringmanager.UpperWideStringProc:=@WinCEWideUpper;
    widestringmanager.LowerWideStringProc:=@WinCEWideLower;
    { Unicode }
    widestringmanager.Unicode2AnsiMoveProc:=@WinCEUnicode2AnsiMove;
    widestringmanager.Ansi2UnicodeMoveProc:=@WinCEAnsi2UnicodeMove;
    widestringmanager.UpperUnicodeStringProc:=@WinCEUnicodeUpper;
    widestringmanager.LowerUnicodeStringProc:=@WinCEUnicodeLower;
    { Codepage }
    widestringmanager.GetStandardCodePageProc:=@WinCEGetStandardCodePage;

    DefaultSystemCodePage:=GetACP;
    DefaultFileSystemCodePage:=WinCEGetStandardCodePage(scpFileSystemSingleByte);
    DefaultRTLFileSystemCodePage:=DefaultFileSystemCodePage;
    DefaultUnicodeCodePage:=CP_UTF16;
  end;


{$IFDEF HAS_MEMORYMANAGER}

{****************************************************************************
                    Memory manager
****************************************************************************}

function malloc(Size : ptruint) : Pointer; cdecl; external 'coredll';
procedure free(P : pointer); cdecl; external 'coredll';
function realloc(P : Pointer; Size : ptruint) : pointer; cdecl; external 'coredll';
function _msize(P : pointer): ptruint; cdecl; external 'coredll';

function SysGetMem (Size : ptruint) : Pointer;
begin
  Result:=malloc(Size);
end;

Function SysFreeMem (P : pointer) : ptruint;
begin
  free(P);
  Result:=0;
end;

Function SysFreeMemSize(p:pointer;Size:ptruint):ptruint;
begin
  Result:=0;
  if (size > 0) and (p <> nil) then
    Result:=SysFreeMem(P);
end;

Function SysAllocMem(Size : ptruint) : Pointer;
begin
  Result:=SysGetMem(Size);
  if Result <> nil then
    FillChar(Result^, Size, 0);
end;

Function SysReAllocMem (var p:pointer;Size:ptruint):Pointer;
begin
  Result:=realloc(p, Size);
  p:=Result;
end;

function SysTryResizeMem(var p:pointer;size : ptruint):boolean;
var
  res: pointer;
begin
  res:=realloc(p, Size);
  Result:=(res <> nil) or (Size = 0);
  if Result then
    p:=res;
end;

function SysMemSize(P : pointer): ptruint;
begin
  Result:=_msize(P);
end;

function SysGetHeapStatus:THeapStatus;
begin
  fillchar(Result,sizeof(Result),0);
end;

function SysGetFPCHeapStatus:TFPCHeapStatus;
begin
  fillchar(Result,sizeof(Result),0);
end;

{$ENDIF HAS_MEMORYMANAGER}

{****************************************************************************
                    Error Message writing using messageboxes
****************************************************************************}

const
  ErrorBufferLength = 1024;
var
  ErrorBuf : array[0..ErrorBufferLength] of char;
  ErrorBufW : array[0..ErrorBufferLength] of widechar;
  ErrorLen : longint;

procedure ErrorWrite(Var F: TextRec);
{
  An error message should always end with #13#10#13#10
}
var
  i : longint;
Begin
  while F.BufPos>0 do
    begin
      begin
        if F.BufPos+ErrorLen>ErrorBufferLength then
          i:=ErrorBufferLength-ErrorLen
        else
          i:=F.BufPos;
        Move(F.BufPtr^,ErrorBuf[ErrorLen],i);
        inc(ErrorLen,i);
        ErrorBuf[ErrorLen]:=#0;
      end;
      if ErrorLen=ErrorBufferLength then
        begin
          AnsiToWideBuf(@ErrorBuf, -1, @ErrorBufW, SizeOf(ErrorBufW));
          MessageBox(0,@ErrorBufW,'Error',$10010); { MB_SETFOREGROUND or ICON_ERROR }
          ErrorLen:=0;
        end;
      Dec(F.BufPos,i);
    end;
End;


procedure ErrorClose(Var F: TextRec);
begin
  if ErrorLen>0 then
   begin
     AnsiToWideBuf(@ErrorBuf, -1, @ErrorBufW, SizeOf(ErrorBufW));
     MessageBox(0,@ErrorBufW,'Error',$10010); { MB_SETFOREGROUND or ICON_ERROR }
     ErrorLen:=0;
   end;
  ErrorLen:=0;
end;


procedure ErrorOpen(Var F: TextRec);
Begin
  TextRec(F).InOutFunc:=@ErrorWrite;
  TextRec(F).FlushFunc:=@ErrorWrite;
  TextRec(F).CloseFunc:=@ErrorClose;
  ErrorLen:=0;
End;


procedure AssignError(Var T: Text);
begin
  Assign(T,'');
  TextRec(T).OpenFunc:=@ErrorOpen;
  Rewrite(T);
end;

function _getstdfilex(fd: integer): pointer; cdecl; external 'coredll';
function _fileno(fd: pointer): THandle; cdecl; external 'coredll';

procedure SysInitStdIO;
begin
  { Setup stdin, stdout and stderr, for GUI apps redirect stderr,stdout to be
    displayed in and messagebox }
  if not IsConsole then
    begin
      AssignError(stderr);
      AssignError(stdout);
      Assign(Output,'');
      Assign(Input,'');
      Assign(ErrOutput,'');
    end
  else
    begin
      StdInputHandle:=_fileno(_getstdfilex(0));
      StdOutputHandle:=_fileno(_getstdfilex(1));
      StdErrorHandle:=_fileno(_getstdfilex(2));

      OpenStdIO(Input,fmInput,StdInputHandle);
      OpenStdIO(Output,fmOutput,StdOutputHandle);
      OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
      OpenStdIO(StdOut,fmOutput,StdOutputHandle);
      OpenStdIO(StdErr,fmOutput,StdErrorHandle);
    end;
end;

(* ProcessID cached to avoid repeated calls to GetCurrentProcess. *)

var
  ProcessID: SizeUInt;

function GetProcessID: SizeUInt;
begin
 GetProcessID := ProcessID;
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

procedure SysCleanup;
var
  i: integer;
begin
  if FCmdLine = nil then
    exit;
  SysFreeMem(FCmdLine);
  for i:=0 to Fargc do
    sysfreemem(Fargv[i]);
  sysfreemem(Fargv);
end;

initialization
  SysResetFPU;
  if not(IsLibrary) then
    SysInitFPU;
  StackLength := CheckInitialStkLen(InitialStkLen);
  StackBottom := Sptr - StackLength;
  { some misc stuff }
  hprevinst:=0;
  if not IsLibrary then
    SysInstance:=GetModuleHandle(nil);
  MainInstance:=SysInstance;
{$IFNDEF HAS_MEMORYMANAGER}
  { Setup Heap }
  InitHeap;
{$ENDIF HAS_MEMORYMANAGER}
  SysInitExceptions;
  initunicodestringmanager;
  InitWinCEWidestrings;
  if not IsLibrary then
    begin
      SysInitStdIO;
    end;
  { Reset IO Error }
  InOutRes:=0;
  ProcessID := GetCurrentProcessID;
  { threading }
  InitSystemThreads;
  InitSystemDynLibs;
  DispCallByIDProc:=@DoDispCallByIDError;

finalization
  SysCleanup;
end.
