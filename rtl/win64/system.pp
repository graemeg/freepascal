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


{ $define SYSTEMEXCEPTIONDEBUG}

{$ifdef SYSTEMDEBUG}
  {$define SYSTEMEXCEPTIONDEBUG}
{$endif SYSTEMDEBUG}

{$define DISABLE_NO_THREAD_MANAGER}
{$define HAS_WIDESTRINGMANAGER}
{$define DISABLE_NO_DYNLIBS_MANAGER}
{$define FPC_SYSTEM_HAS_SYSDLH}

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
<<<<<<< HEAD
<<<<<<< HEAD
{ FileNameCaseSensitive and FileNameCasePreserving are defined separately below!!! }
=======
=======
>>>>>>> origin/fixes_2_2
{ FileNameCaseSensitive is defined separately below!!! }
>>>>>>> graemeg/fixes_2_2
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

<<<<<<< HEAD
  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = true;
=======
  FileNameCaseSensitive : boolean = true;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
type
  TStartupInfo = record
    cb : longint;
    lpReserved : Pointer;
    lpDesktop : Pointer;
    lpTitle : Pointer;
    dwX : longint;
    dwY : longint;
    dwXSize : longint;
    dwYSize : longint;
    dwXCountChars : longint;
    dwYCountChars : longint;
    dwFillAttribute : longint;
    dwFlags : longint;
    wShowWindow : Word;
    cbReserved2 : Word;
    lpReserved2 : Pointer;
    hStdInput : THandle;
    hStdOutput : THandle;
    hStdError : THandle;
  end;

>>>>>>> graemeg/cpstrnew
var
{ C compatible arguments }
  argc : longint;
  argv : ppchar;
{ Win32 Info }
<<<<<<< HEAD
  startupinfo : tstartupinfo deprecated;  // Delphi does not have one in interface
  StartupConsoleMode : dword;
=======
  startupinfo : tstartupinfo;
  StartupConsoleMode : dword;
  hprevinst,
>>>>>>> graemeg/cpstrnew
  MainInstance : qword;
  cmdshow     : longint;
  DLLreason : dword;
  DLLparam : PtrInt;
const
  hprevinst: qword=0;
type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TDLL_Entry_Hook = procedure (dllparam : PtrInt);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  TDLL_Entry_Hook = procedure (dllparam : longint);
>>>>>>> graemeg/cpstrnew

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

Const
  { it can be discussed whether fmShareDenyNone means read and write or read, write and delete, see
    also http://bugs.freepascal.org/view.php?id=8898, this allows users to configure the used
	value
  }
  fmShareDenyNoneFlags : DWord = 3;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

implementation

var
  SysInstance : qword;public;

{ used by wstrings.inc because wstrings.inc is included before sysos.inc
  this is put here (FK) }

function SysAllocStringLen(psz:pointer;len:dword):pointer;stdcall;
 external 'oleaut32.dll' name 'SysAllocStringLen';

procedure SysFreeString(bstr:pointer);stdcall;
 external 'oleaut32.dll' name 'SysFreeString';

function SysReAllocStringLen(var bstr:pointer;psz: pointer;
  len:dword): Integer; stdcall;external 'oleaut32.dll' name 'SysReAllocStringLen';

>>>>>>> graemeg/cpstrnew

implementation

{$asmmode att}

var
<<<<<<< HEAD
  SysInstance : qword;public;
=======
  arglen,
  count   : longint;
  argstart,
  pc,arg  : pchar;
  quote   : char;
  argvlen : longint;
  buf: array[0..259] of char;  // need MAX_PATH bytes, not 256!

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
  { create commandline, it starts with the executed filename which is argv[0] }
  { Win32 passes the command NOT via the args, but via getmodulefilename}
  count:=0;
  argv:=nil;
  argvlen:=0;
  ArgLen := GetModuleFileName(0, @buf[0], sizeof(buf));
  buf[ArgLen] := #0; // be safe
  allocarg(0,arglen);
  move(buf,argv[0]^,arglen+1);
  { Setup cmdline variable }
  cmdline:=GetCommandLine;
  { process arguments }
  pc:=cmdline;
{$IfDef SYSTEM_DEBUG_STARTUP}
  Writeln(stderr,'Win32 GetCommandLine is #',pc,'#');
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
      end;
 {$IfDef SYSTEM_DEBUG_STARTUP}
     Writeln(stderr,'dos arg ',count,' #',arglen,'#',argv[count],'#');
 {$EndIf SYSTEM_DEBUG_STARTUP}
     inc(count);
   end;
  { get argc }
  argc:=count;
  { free unused memory, leaving a nil entry at the end }
  sysreallocmem(argv,(count+1)*sizeof(pointer));
  argv[count] := nil;
end;


function paramcount : longint;
begin
  paramcount := argc - 1;
end;
>>>>>>> graemeg/cpstrnew

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
procedure PascalMain;stdcall;external name 'PASCALMAIN';

{ include code common with win32 }
{$I syswin.inc}

{ TLS directory code }
{$I systlsdir.inc}

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



procedure Exe_entry;[public,alias:'_FPC_EXE_Entry'];
  begin
     IsLibrary:=false;
     { install the handlers for exe only ?
       or should we install them for DLL also ? (PM) }
{$ifndef FPC_USE_WIN64_SEH}
     install_exception_handlers;
{$endif FPC_USE_WIN64_SEH}
     ExitCode:=0;
     asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        { allocate space for an exception frame }
        pushq $0
        pushq %gs:(0)
        { movl  %rsp,%gs:(0)
          but don't insert it as it doesn't
          point to anything yet
          this will be used in signals unit }
        movq %rsp,%rax
{$ifdef FPC_HAS_RIP_RELATIVE}
        movq %rax,System_exception_frame(%rip)
{$else}
        movq %rax,System_exception_frame
{$endif}
        { keep stack aligned }
        pushq $0
        pushq %rbp
        movq %rsp,%rax
        movq %rax,st
     end;
     StackTop:=st;
     asm
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        xorq %rax,%rax
        movw %ss,%ax
        movl %eax,_SS(%rip)
<<<<<<< HEAD
        movq %rbp,%rsi
        xorq %rbp,%rbp
{$ifdef FPC_USE_WIN64_SEH}
        xor  %rcx,%rcx
        lea  PASCALMAIN(%rip),%rdx
        call main_wrapper
{$else FPC_USE_WIN64_SEH}
=======
{$else}
        movl %eax,_SS
<<<<<<< HEAD
<<<<<<< HEAD
{$endif}
        xorq %rbp,%rbp
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
        xorl %rbp,%rbp
>>>>>>> graemeg/fixes_2_2
        call PASCALMAIN
{$endif FPC_USE_WIN64_SEH}
        movq %rsi,%rbp
     end ['RSI','RBP'];     { <-- specifying RSI allows compiler to save/restore it properly }
     { if we pass here there was no error ! }
     system_exit;
  end;

function GetConsoleMode(hConsoleHandle: THandle; var lpMode: DWORD): Boolean; stdcall; external 'kernel32' name 'GetConsoleMode';

<<<<<<< HEAD
procedure _FPC_DLLMainCRTStartup(_hinstance : qword;_dllreason : dword;_dllparam:Pointer);stdcall;public name '_DLLMainCRTStartup';
=======
function Dll_entry{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}(const info : TEntryInformation){$endif FPC_HAS_INDIRECT_MAIN_INFORMATION} : longbool;forward;

procedure _FPC_mainCRTStartup;stdcall;public name '_mainCRTStartup';
begin
  IsConsole:=true;
  GetConsoleMode(GetStdHandle((Std_Input_Handle)),StartupConsoleMode);
  Exe_entry;
end;


procedure _FPC_WinMainCRTStartup;stdcall;public name '_WinMainCRTStartup';
begin
  IsConsole:=false;
  Exe_entry;
end;


procedure _FPC_DLLMainCRTStartup(_hinstance : qword;_dllreason,_dllparam:longint);stdcall;public name '_DLLMainCRTStartup';
>>>>>>> graemeg/cpstrnew
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


procedure LinkIn(p1,p2,p3: Pointer); inline;
begin
end;

<<<<<<< HEAD
procedure _FPC_mainCRTStartup;stdcall;public name '_mainCRTStartup';
begin
  IsConsole:=true;
  GetConsoleMode(GetStdHandle((Std_Input_Handle)),@StartupConsoleMode);
{$ifdef FPC_USE_TLS_DIRECTORY}
  LinkIn(@_tls_used,@FreePascal_TLS_callback,@FreePascal_end_of_TLS_callback);
{$endif FPC_USE_TLS_DIRECTORY}
  Exe_entry;
end;

=======
const
  { MultiByteToWideChar  }
     MB_PRECOMPOSED = 1;
     CP_ACP = 0;
     WC_NO_BEST_FIT_CHARS = $400;

function MultiByteToWideChar(CodePage:UINT; dwFlags:DWORD; lpMultiByteStr:PChar; cchMultiByte:longint; lpWideCharStr:PWideChar;cchWideChar:longint):longint;
    stdcall; external 'kernel32' name 'MultiByteToWideChar';
function WideCharToMultiByte(CodePage:UINT; dwFlags:DWORD; lpWideCharStr:PWideChar; cchWideChar:longint; lpMultiByteStr:PChar;cchMultiByte:longint; lpDefaultChar:PChar; lpUsedDefaultChar:pointer):longint;
    stdcall; external 'kernel32' name 'WideCharToMultiByte';
function CharUpperBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD;
    stdcall; external 'user32' name 'CharUpperBuffW';
function CharLowerBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD;
    stdcall; external 'user32' name 'CharLowerBuffW';


<<<<<<< HEAD
=======
procedure Win32Wide2AnsiMove(source:pwidechar;var dest:ansistring;len:SizeInt);
  var
    destlen: SizeInt;
  begin
    // retrieve length including trailing #0
    // not anymore, because this must also be usable for single characters
    destlen:=WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, source, len, nil, 0, nil, nil);
    // this will null-terminate
    setlength(dest, destlen);
    WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, source, len, @dest[1], destlen, nil, nil);
  end;

>>>>>>> graemeg/fixes_2_2
procedure Win32Ansi2WideMove(source:pchar;var dest:widestring;len:SizeInt);
  var
    destlen: SizeInt;
  begin
    // retrieve length including trailing #0
    // not anymore, because this must also be usable for single characters
    destlen:=MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, source, len, nil, 0);
    // this will null-terminate
    setlength(dest, destlen);
    MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, source, len, @dest[1], destlen);
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  end;


function Win32WideUpper(const s : WideString) : WideString;
  begin
    result:=s;
    UniqueString(result);
    if length(result)>0 then
      CharUpperBuff(LPWSTR(result),length(result));
>>>>>>> graemeg/fixes_2_2
  end;
>>>>>>> graemeg/cpstrnew

procedure _FPC_WinMainCRTStartup;stdcall;public name '_WinMainCRTStartup';
begin
  IsConsole:=false;
{$ifdef FPC_USE_TLS_DIRECTORY}
  LinkIn(@_tls_used,@FreePascal_TLS_callback,@FreePascal_end_of_TLS_callback);
{$endif FPC_USE_TLS_DIRECTORY}
  Exe_entry;
end;

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

<<<<<<< HEAD
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
<<<<<<< HEAD
=======
=======
  { Setup stdin, stdout and stderr, for GUI apps redirect stderr,stdout to be
    displayed in a messagebox }
  StdInputHandle:=THandle(GetStdHandle(STD_INPUT_HANDLE));
  StdOutputHandle:=THandle(GetStdHandle(STD_OUTPUT_HANDLE));
  StdErrorHandle:=THandle(GetStdHandle(STD_ERROR_HANDLE));
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


>>>>>>> origin/fixes_2_2
function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;assembler;
asm
  movq  %gs:(8),%rax
  subq  %gs:(16),%rax
end;


begin
  SysResetFPU;
  if not(IsLibrary) then
    SysInitFPU;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { pass dummy value }
  StackLength := CheckInitialStkLen($1000000);
  StackBottom := StackTop - StackLength;
  { get some helpful informations }
  GetStartupInfo(@startupinfo);
  { some misc Win32 stuff }
  if not IsLibrary then
    SysInstance:=getmodulehandle(nil);
  MainInstance:=SysInstance;
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
