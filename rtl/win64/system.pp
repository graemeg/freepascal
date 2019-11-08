{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2006 by Florian Klaempfl and Pavel Ozerski
    member of the Free Pascal development team.

    FPC Pascal system unit for the Win64 API.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;
interface


{$define FPC_IS_SYSTEM}
{ $define SYSTEMEXCEPTIONDEBUG}

{$ifdef SYSTEMDEBUG}
  {$define SYSTEMEXCEPTIONDEBUG}
{$endif SYSTEMDEBUG}

{$define DISABLE_NO_THREAD_MANAGER}
{$define HAS_WIDESTRINGMANAGER}
{$define DISABLE_NO_DYNLIBS_MANAGER}
{$define FPC_SYSTEM_HAS_SYSDLH}
{$define FPC_HAS_SETCTRLBREAKHANDLER}

{$ifdef FPC_USE_WIN64_SEH}
  {$define FPC_SYSTEM_HAS_RAISEEXCEPTION}
  {$define FPC_SYSTEM_HAS_RERAISE}
  {$define FPC_SYSTEM_HAS_CAPTUREBACKTRACE}
{$endif FPC_USE_WIN64_SEH}

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

type
   PEXCEPTION_FRAME = ^TEXCEPTION_FRAME;
   TEXCEPTION_FRAME = record
     next : PEXCEPTION_FRAME;
     handler : pointer;
   end;

const
{ Default filehandles }
  UnusedHandle    : THandle = THandle(-1);
  StdInputHandle  : THandle = 0;
  StdOutputHandle : THandle = 0;
  StdErrorHandle  : THandle = 0;
  System_exception_frame : PEXCEPTION_FRAME =nil;

  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = true;
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

var
{ C compatible arguments }
  argc : longint;
  argv : ppchar;
{ Win32 Info }
  startupinfo : tstartupinfo deprecated;  // Delphi does not have one in interface
  StartupConsoleMode : dword;
  MainInstance : qword;
  cmdshow     : longint;
  DLLreason : dword;
  DLLparam : PtrInt;
const
  hprevinst: qword=0;
type
  TDLL_Entry_Hook = procedure (dllparam : PtrInt);

const
  Dll_Process_Detach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Attach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Detach_Hook : TDLL_Entry_Hook = nil;

Const
  { it can be discussed whether fmShareDenyNone means read and write or read, write and delete, see
    also http://bugs.freepascal.org/view.php?id=8898, this allows users to configure the used
	value
  }
  fmShareDenyNoneFlags : DWord = 3;

implementation

{$asmmode att}

var
{$ifdef VER3_0}
  SysInstance : qword;
  FPCSysInstance: PQWord = @SysInstance; public name '_FPC_SysInstance';
{$else VER3_0}
  FPCSysInstance : PQWord;public name '_FPC_SysInstance';
{$endif VER3_0}

{$define FPC_SYSTEM_HAS_OSSETUPENTRYINFORMATION}
procedure OsSetupEntryInformation(constref info: TEntryInformation); forward;

{$ifdef FPC_USE_WIN64_SEH}
function main_wrapper(arg: Pointer; proc: Pointer): ptrint; assembler; nostackframe;
asm
    subq   $40, %rsp
.seh_stackalloc 40
.seh_endprologue
    call   %rdx             { "arg" is passed in %rcx }
    nop                     { this nop is critical for exception handling }
    addq   $40, %rsp
.seh_handler __FPC_default_handler,@except,@unwind
end;
{$endif FPC_USE_WIN64_SEH}

{$define FPC_SYSTEM_HAS_STACKTOP}
function StackTop: pointer; assembler;nostackframe;
asm
   movq  %gs:(8),%rax
end;

{ include system independent routines }
{$I system.inc}

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

{$ifndef FPC_USE_WIN64_SEH}
procedure install_exception_handlers;forward;
{$endif FPC_USE_WIN64_SEH}
{$ifdef VER3_0}
procedure PascalMain;external name 'PASCALMAIN';
{$endif VER3_0}

{ include code common with win32 }
{$I syswin.inc}

{$ifdef VER3_0}
{ TLS directory code }
{$I systlsdir.inc}
{$endif VER3_0}

procedure OsSetupEntryInformation(constref info: TEntryInformation);
begin
  TlsKey := info.OS.TlsKeyAddr;
  FPCSysInstance := info.OS.SysInstance;
  WStrInitTablesTable := info.OS.WideInitTables;
end;

Procedure system_exit;
begin
  { see comments in win32/system.pp about this logic }
  if IsLibrary then
  begin
    if DllInitState in [DLL_PROCESS_ATTACH,DLL_PROCESS_DETACH] then
      LongJmp(DLLBuf,1)
    else
      MainThreadIDWin32:=0;
  end;
  if not IsConsole then
   begin
     Close(stderr);
     Close(stdout);
     Close(erroutput);
     Close(Input);
     Close(Output);
     { what about Input and Output ?? PM }
     { now handled, FPK }
   end;

  { call exitprocess, with cleanup as required }
  ExitProcess(exitcode);
end;

var
  { old compilers emitted a reference to _fltused if a module contains
    floating type code so the linker could leave away floating point
    libraries or not. VC does this as well so we need to define this
    symbol as well (FK)
  }
  _fltused : int64;cvar;public;
  { value of the stack segment
    to check if the call stack can be written on exceptions }
  _SS : Cardinal;


{$ifdef VER3_0}
procedure Exe_entry;[public,alias:'_FPC_EXE_Entry'];
{$else VER3_0}
procedure Exe_entry(constref info: TEntryInformation);[public,alias:'_FPC_EXE_Entry'];
{$endif VER3_0}
  begin
{$ifndef VER3_0}
     SetupEntryInformation(info);
{$endif VER3_0}
     IsLibrary:=false;
     { install the handlers for exe only ?
       or should we install them for DLL also ? (PM) }
{$ifndef FPC_USE_WIN64_SEH}
     install_exception_handlers;
{$endif FPC_USE_WIN64_SEH}
     ExitCode:=0;
     asm
        xorq %rax,%rax
        movw %ss,%ax
        movl %eax,_SS(%rip)
        movq %rbp,%rsi
        xorq %rbp,%rbp
{$ifdef VER3_0}
{$ifdef FPC_USE_WIN64_SEH}
        xor  %rcx,%rcx
        lea  PASCALMAIN(%rip),%rdx
        call main_wrapper
{$else FPC_USE_WIN64_SEH}
        call PASCALMAIN
{$endif FPC_USE_WIN64_SEH}
{$else VER3_0}
{$ifdef FPC_USE_WIN64_SEH}
        xor  %rcx,%rcx
        lea  EntryInformation(%rip),%rdx
        movq TEntryInformation.PascalMain(%rdx),%rdx
        call main_wrapper
{$else FPC_USE_WIN64_SEH}
        lea  EntryInformation(%rip),%rdx
        call TEntryInformation.PascalMain(%rdx)
{$endif FPC_USE_WIN64_SEH}
{$endif VER3_0}
        movq %rsi,%rbp
     end ['RSI','RBP'];     { <-- specifying RSI allows compiler to save/restore it properly }
     { if we pass here there was no error ! }
     system_exit;
  end;


{$ifdef VER3_0}
procedure _FPC_DLLMainCRTStartup(_hinstance : qword;_dllreason : dword;_dllparam:Pointer);stdcall;public name '_DLLMainCRTStartup';
begin
  IsConsole:=true;
  sysinstance:=_hinstance;
  dllreason:=_dllreason;
  dllparam:=PtrInt(_dllparam);
  DLL_Entry;
end;


procedure _FPC_DLLWinMainCRTStartup(_hinstance : qword;_dllreason : dword;_dllparam:Pointer);stdcall;public name '_DLLWinMainCRTStartup';
begin
  IsConsole:=false;
  sysinstance:=_hinstance;
  dllreason:=_dllreason;
  dllparam:=PtrInt(_dllparam);
  DLL_Entry;
end;
{$endif VER3_0}

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


//
// Hardware exception handling
//
{$I seh64.inc}


type
  TVectoredExceptionHandler = function (excep : PExceptionPointers) : Longint;

function AddVectoredExceptionHandler(FirstHandler : DWORD;VectoredHandler : TVectoredExceptionHandler) : longint;
        external 'kernel32' name 'AddVectoredExceptionHandler';

{$ifndef FPC_USE_WIN64_SEH}
const
  MaxExceptionLevel = 16;
  exceptLevel : Byte = 0;

var
  exceptRip       : array[0..MaxExceptionLevel-1] of Int64;
  exceptError     : array[0..MaxExceptionLevel-1] of Byte;
  resetFPU        : array[0..MaxExceptionLevel-1] of Boolean;

{$ifdef SYSTEMEXCEPTIONDEBUG}
procedure DebugHandleErrorAddrFrame(error : longint; addr, frame : pointer);
begin
  if IsConsole then
    begin
      write(stderr,'HandleErrorAddrFrame(error=',error);
      write(stderr,',addr=',hexstr(int64(addr),16));
      writeln(stderr,',frame=',hexstr(int64(frame),16),')');
    end;
  HandleErrorAddrFrame(error,addr,frame);
end;
{$endif SYSTEMEXCEPTIONDEBUG}

procedure JumpToHandleErrorFrame;
  var
    rip, rbp : int64;
    error : longint;
  begin
    // save ebp
    asm
      movq (%rbp),%rax
      movq %rax,rbp
    end;
    if exceptLevel>0 then
      dec(exceptLevel);

    rip:=exceptRip[exceptLevel];
    error:=exceptError[exceptLevel];
{$ifdef SYSTEMEXCEPTIONDEBUG}
    if IsConsole then
      writeln(stderr,'In JumpToHandleErrorFrame error=',error);
{$endif SYSTEMEXCEPTIONDEBUG}
    if resetFPU[exceptLevel] then
      SysResetFPU;
    { build a fake stack }
    asm
      movq   rbp,%r8
      movq   rip,%rdx
      movl   error,%ecx
      pushq  rip
      movq   rbp,%rbp // Change frame pointer

{$ifdef SYSTEMEXCEPTIONDEBUG}
      jmpl   DebugHandleErrorAddrFrame
{$else not SYSTEMEXCEPTIONDEBUG}
      jmpl   HandleErrorAddrFrame
{$endif SYSTEMEXCEPTIONDEBUG}
    end;
  end;


function syswin64_x86_64_exception_handler(excep : PExceptionPointers) : Longint;public;
  var
    res: longint;
    err: byte;
    must_reset_fpu: boolean;
  begin
    res:=EXCEPTION_CONTINUE_SEARCH;
{$ifdef SYSTEMEXCEPTIONDEBUG}
    if IsConsole then
      Writeln(stderr,'syswin64_x86_64_exception_handler called');
{$endif SYSTEMEXCEPTIONDEBUG}
    if excep^.ContextRecord^.SegSs=_SS then
      begin
        err := 0;
        must_reset_fpu := true;
{$ifdef SYSTEMEXCEPTIONDEBUG}
        if IsConsole then Writeln(stderr,'Exception ',
          hexstr(excep^.ExceptionRecord^.ExceptionCode,8));
{$endif SYSTEMEXCEPTIONDEBUG}
        case cardinal(excep^.ExceptionRecord^.ExceptionCode) of
          STATUS_INTEGER_DIVIDE_BY_ZERO,
          STATUS_FLOAT_DIVIDE_BY_ZERO :
            err := 200;
          STATUS_ARRAY_BOUNDS_EXCEEDED :
            begin
              err := 201;
              must_reset_fpu := false;
            end;
          STATUS_STACK_OVERFLOW :
            begin
              err := 202;
              must_reset_fpu := false;
            end;
          STATUS_FLOAT_OVERFLOW :
            err := 205;
          STATUS_FLOAT_DENORMAL_OPERAND,
          STATUS_FLOAT_UNDERFLOW :
            err := 206;
            { excep^.ContextRecord^.FloatSave.StatusWord := excep^.ContextRecord^.FloatSave.StatusWord and $ffffff00;}
          STATUS_FLOAT_INEXACT_RESULT,
          STATUS_FLOAT_INVALID_OPERATION,
          STATUS_FLOAT_STACK_CHECK :
            err := 207;
          STATUS_INTEGER_OVERFLOW :
            begin
              err := 215;
              must_reset_fpu := false;
            end;
          STATUS_ILLEGAL_INSTRUCTION:
            err := 216;
          STATUS_ACCESS_VIOLATION:
            { Athlon prefetch bug? }
            if is_prefetch(pointer(excep^.ContextRecord^.rip)) then
              begin
                { if yes, then retry }
                excep^.ExceptionRecord^.ExceptionCode := 0;
                res:=EXCEPTION_CONTINUE_EXECUTION;
              end
            else
              err := 216;

          STATUS_CONTROL_C_EXIT:
            err := 217;
          STATUS_PRIVILEGED_INSTRUCTION:
            begin
              err := 218;
              must_reset_fpu := false;
            end;
          else
            begin
              if ((excep^.ExceptionRecord^.ExceptionCode and SEVERITY_ERROR) = SEVERITY_ERROR) then
                err := 217
              else
                { pass through exceptions which aren't an error. The problem is that vectored handlers
                  always are called before structured ones so we see also internal exceptions of libraries.
                  I wonder if there is a better solution (FK)
                }
                res:=EXCEPTION_CONTINUE_SEARCH;
            end;
        end;

        if (err <> 0) and (exceptLevel < MaxExceptionLevel) then
          begin
            exceptRip[exceptLevel] := excep^.ContextRecord^.Rip;
            exceptError[exceptLevel] := err;
            resetFPU[exceptLevel] := must_reset_fpu;
            inc(exceptLevel);

            excep^.ContextRecord^.Rip := Int64(@JumpToHandleErrorFrame);
            excep^.ExceptionRecord^.ExceptionCode := 0;

            res := EXCEPTION_CONTINUE_EXECUTION;
{$ifdef SYSTEMEXCEPTIONDEBUG}
            if IsConsole then begin
              writeln(stderr,'Exception Continue Exception set at ',
                      hexstr(exceptRip[exceptLevel-1],16));
              writeln(stderr,'Rip changed to ',
                      hexstr(int64(@JumpToHandleErrorFrame),16), ' error=', err);
            end;
{$endif SYSTEMEXCEPTIONDEBUG}
        end;
    end;
    syswin64_x86_64_exception_handler := res;
  end;



procedure install_exception_handlers;
  begin
    AddVectoredExceptionHandler(1,@syswin64_x86_64_exception_handler);
  end;
{$endif ndef FPC_USE_WIN64_SEH}

{$ifdef VER3_0}
procedure LinkIn(p1,p2,p3: Pointer); inline;
begin
end;

procedure _FPC_mainCRTStartup;stdcall;public name '_mainCRTStartup';
begin
  IsConsole:=true;
  GetConsoleMode(GetStdHandle((Std_Input_Handle)),@StartupConsoleMode);
{$ifdef FPC_USE_TLS_DIRECTORY}
  LinkIn(@_tls_used,@FreePascal_TLS_callback,@FreePascal_end_of_TLS_callback);
{$endif FPC_USE_TLS_DIRECTORY}
  Exe_entry;
end;


procedure _FPC_WinMainCRTStartup;stdcall;public name '_WinMainCRTStartup';
begin
  IsConsole:=false;
{$ifdef FPC_USE_TLS_DIRECTORY}
  LinkIn(@_tls_used,@FreePascal_TLS_callback,@FreePascal_end_of_TLS_callback);
{$endif FPC_USE_TLS_DIRECTORY}
  Exe_entry;
end;
{$endif VER3_0}

{$ifdef FPC_SECTION_THREADVARS}
function fpc_tls_add(addr: pointer): pointer; assembler; nostackframe;
  [public,alias: 'FPC_TLS_ADD']; compilerproc;
  asm
      sub   $56,%rsp                  { 32 spill area + 16 local vars + 8 misalignment }
  .seh_stackalloc 56
  .seh_endprologue
      lea   tls_data_start(%rip),%rax
      sub   %rax,%rcx
      cmpb  $0,IsLibrary(%rip)
      mov   _tls_index(%rip),%eax
      jnz   .L1
      mov   %gs:(88),%rdx
      add   (%rdx,%rax,8),%rcx
      mov   %rcx,%rax
      jmp   .L3
.L1:
      mov   %rcx,32(%rsp)
      call  GetLastError
      mov   %rax,40(%rsp)             { save LastError }
      mov   _tls_index(%rip),%ecx
      call  TlsGetValue
      test  %rax,%rax
      jnz   .L2
      { This can happen when a thread existed before DLL was loaded,
        or if DisableThreadLibraryCalls was called. }
      call  SysAllocateThreadVars
      mov   $0x1000000,%rcx
      call  InitThread
      mov   _tls_index(%rip),%ecx
      call  TlsGetValue
.L2:
      add   %rax,32(%rsp)
      mov   40(%rsp),%rcx
      call  SetLastError
      mov   32(%rsp),%rax
.L3:
      add   $56,%rsp
  end;
{$endif FPC_SECTION_THREADVARS}

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
  type
    tdosheader = packed record
       e_magic : word;
       e_cblp : word;
       e_cp : word;
       e_crlc : word;
       e_cparhdr : word;
       e_minalloc : word;
       e_maxalloc : word;
       e_ss : word;
       e_sp : word;
       e_csum : word;
       e_ip : word;
       e_cs : word;
       e_lfarlc : word;
       e_ovno : word;
       e_res : array[0..3] of word;
       e_oemid : word;
       e_oeminfo : word;
       e_res2 : array[0..9] of word;
       e_lfanew : longint;
    end;
    tpeheader = packed record
       PEMagic : longint;
       Machine : word;
       NumberOfSections : word;
       TimeDateStamp : longint;
       PointerToSymbolTable : longint;
       NumberOfSymbols : longint;
       SizeOfOptionalHeader : word;
       Characteristics : word;
       Magic : word;
       MajorLinkerVersion : byte;
       MinorLinkerVersion : byte;
       SizeOfCode : longint;
       SizeOfInitializedData : longint;
       SizeOfUninitializedData : longint;
       AddressOfEntryPoint : longint;
       BaseOfCode : longint;
{$ifdef win32}
       BaseOfData : longint;
{$endif win32}
       ImageBase : PtrInt;
       SectionAlignment : longint;
       FileAlignment : longint;
       MajorOperatingSystemVersion : word;
       MinorOperatingSystemVersion : word;
       MajorImageVersion : word;
       MinorImageVersion : word;
       MajorSubsystemVersion : word;
       MinorSubsystemVersion : word;
       Reserved1 : longint;
       SizeOfImage : longint;
       SizeOfHeaders : longint;
       CheckSum : longint;
       Subsystem : word;
       DllCharacteristics : word;
       SizeOfStackReserve : PtrInt;
       SizeOfStackCommit : PtrInt;
       SizeOfHeapReserve : PtrInt;
       SizeOfHeapCommit : PtrInt;
       LoaderFlags : longint;
       NumberOfRvaAndSizes : longint;
       DataDirectory : array[1..$80] of byte;
    end;
  begin
    result:=tpeheader((pointer(getmodulehandle(nil))+(tdosheader(pointer(getmodulehandle(nil))^).e_lfanew))^).SizeOfStackReserve;
  end;

begin
  { pass dummy value }
  StackLength := CheckInitialStkLen($1000000);
  StackBottom := StackTop - StackLength;
  { get some helpful informations }
  GetStartupInfo(@startupinfo);
  { some misc Win32 stuff }
  if not IsLibrary then
    FPCSysInstance^:=getmodulehandle(nil);
  MainInstance:=FPCSysInstance^;
  cmdshow:=startupinfo.wshowwindow;
  { Setup heap and threading, these may be already initialized from TLS callback }
  if not Assigned(CurrentTM.BeginThread) then
  begin
    InitHeap;
    InitSystemThreads;
  end;
  SysInitExceptions;
  initunicodestringmanager;
  InitWin32Widestrings;
  SysInitStdIO;
  { Arguments }
  setup_arguments;
  InitSystemDynLibs;
  { Reset IO Error }
  InOutRes:=0;
  ProcessID := GetCurrentProcessID;
  DispCallByIDProc:=@DoDispCallByIDError;
end.
