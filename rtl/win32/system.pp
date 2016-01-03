{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl and Pavel Ozerski
    member of the Free Pascal development team.

    FPC Pascal system unit for the Win32 API.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;
interface

{$ifdef SYSTEMDEBUG}
  {$define SYSTEMEXCEPTIONDEBUG}
{$endif SYSTEMDEBUG}

{$define FPC_HAS_INDIRECT_MAIN_INFORMATION}

{$ifdef cpui386}
  {$define Set_i386_Exception_handler}
{$endif cpui386}

{$define DISABLE_NO_THREAD_MANAGER}
{$define HAS_WIDESTRINGMANAGER}
{$define DISABLE_NO_DYNLIBS_MANAGER}
{$define FPC_SYSTEM_HAS_SYSDLH}

{$ifdef FPC_USE_WIN32_SEH}
  {$define FPC_SYSTEM_HAS_RAISEEXCEPTION}
  {$define FPC_SYSTEM_HAS_RERAISE}
  {$define FPC_SYSTEM_HAS_DONEEXCEPTION}
  {$define FPC_SYSTEM_HAS_SAFECALLHANDLER}
{$endif FPC_USE_WIN32_SEH}

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

  System_exception_frame : PEXCEPTION_FRAME =nil;

var
{ C compatible arguments }
  argc : longint; public name 'operatingsystem_parameter_argc';
  argv : ppchar; public name 'operatingsystem_parameter_argv';
{ Win32 Info }
  startupinfo : tstartupinfo deprecated;  // Delphi does not have one in interface
  MainInstance,
  cmdshow     : longint;
<<<<<<< HEAD
  DLLreason : dword; public name 'operatingsystem_dllreason';
  DLLparam : PtrInt; public name 'operatingsystem_dllparam';
  StartupConsoleMode : DWORD;
const
  hprevinst: longint=0;
=======
  DLLreason,DLLparam:longint;
  StartupConsoleMode : DWORD;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
<<<<<<< HEAD
  { it can be discussed whether fmShareDenyNone means read and write or read, write and delete, see
=======
  { it can be discussed whether fmShareDenyNone means read and write or read, write and delete, see 
>>>>>>> origin/fixes_2.4
    also http://bugs.freepascal.org/view.php?id=8898, this allows users to configure the used
	value
  }
  fmShareDenyNoneFlags : DWord = 3;
<<<<<<< HEAD

=======
  
>>>>>>> origin/fixes_2.4
implementation

var
  EntryInformation : TEntryInformation;
  SysInstance : Longint;public name '_FPC_SysInstance';
  InitFinalTable : record end; external name 'INITFINAL';
  ThreadvarTablesTable : record end; external name 'FPC_THREADVARTABLES';
  procedure PascalMain;stdcall;external name 'PASCALMAIN';
  procedure asm_exit;stdcall;external name 'asm_exit';
const
  EntryInformation : TEntryInformation = (
    InitFinalTable : @InitFinalTable;
    ThreadvarTablesTable : @ThreadvarTablesTable;
    asm_exit : @asm_exit;
    PascalMain : @PascalMain;
    valgrind_used : false;
    );

{$ifdef FPC_USE_WIN32_SEH}
function main_wrapper(arg: Pointer; proc: Pointer): ptrint; forward;
procedure OutermostHandler; external name '__FPC_DEFAULT_HANDLER';
{$endif FPC_USE_WIN32_SEH}

{$define FPC_SYSTEM_HAS_STACKTOP}
function StackTop: pointer; assembler;nostackframe;
asm
   movl  %fs:(4),%eax
end;

{ include system independent routines }
{$I system.inc}

<<<<<<< HEAD
{ include code common with win64 }
{$I syswin.inc}
=======
{*****************************************************************************
                              Parameter Handling
*****************************************************************************}

procedure setup_arguments;
var
  arglen,
  count   : longint;
  argstart,
  pc,arg  : pchar;
  quote   : Boolean;
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
     quote:=False;
     argstart:=pc;
     arglen:=0;
     while (pc^<>#0) do
      begin
        case pc^ of
          #1..#32 :
            begin
              if quote then
               inc(arglen)
              else
               break;
            end;
          '"' :
            if pc[1]<>'"' then
              quote := not quote
              else
              inc(pc);
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
        quote:=False;
        pc:=argstart;
        arg:=argv[count];
        while (pc^<>#0) do
         begin
           case pc^ of
             #1..#32 :
               begin
                 if quote then
                  begin
                    arg^:=pc^;
                    inc(arg);
                  end
                 else
                  break;
               end;
             '"' :
               if pc[1]<>'"' then
                 quote := not quote
                  else
                inc(pc);
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

function paramstr(l : longint) : string;
begin
  if (l>=0) and (l<argc) then
    paramstr:=strpas(argv[l])
  else
    paramstr:='';
end;


procedure randomize;
begin
  randseed:=GetTickCount;
end;
>>>>>>> graemeg/cpstrnew


{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

{$ifndef FPC_USE_WIN32_SEH}
procedure install_exception_handlers;forward;
procedure remove_exception_handlers;forward;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_USE_WIN32_SEH}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifndef FPC_HAS_INDIRECT_MAIN_INFORMATION}
procedure PascalMain;stdcall;external name 'PASCALMAIN';
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
procedure fpc_do_exit;stdcall;external name 'FPC_DO_EXIT';
Procedure ExitDLL(Exitcode : longint); forward;
procedure asm_exit;stdcall;external name 'asm_exit';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Procedure system_exit;
begin
  if IsLibrary then
  begin
    { If exiting from DLL_PROCESS_ATTACH/DETACH, unwind to DllMain context. }
    if DllInitState in [DLL_PROCESS_ATTACH,DLL_PROCESS_DETACH] then
      LongJmp(DLLBuf,1)
    else
    { Abnormal termination, Halt has been called from DLL function,
      put down the entire process (DLL_PROCESS_DETACH will still
      occur). At this point RTL has been already finalized in InternalExit
      and shouldn't be finalized another time in DLL_PROCESS_DETACH.
      Indicate this by resetting MainThreadIdWin32. }
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
{$ifndef FPC_USE_WIN32_SEH}
  if not IsLibrary then
    remove_exception_handlers;
{$endif FPC_USE_WIN32_SEH}

  { do cleanup required by the startup code }
  EntryInformation.asm_exit();

  { in 2.0 asm_exit does an exitprocess }
{$ifndef ver2_0}
  { do cleanup required by the startup code }
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
  EntryInformation.asm_exit();
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
  asm_exit;
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
{$endif ver2_0}

  { in 2.0 asm_exit does an exitprocess }
{$ifndef ver2_0}
  { do cleanup required by the startup code }
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
  EntryInformation.asm_exit();
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
  asm_exit;
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
{$endif ver2_0}

  { call exitprocess, with cleanup as required }
  ExitProcess(exitcode);
end;

var
  { value of the stack segment
    to check if the call stack can be written on exceptions }
  _SS : Cardinal;

procedure Exe_entry(const info : TEntryInformation);[public,alias:'_FPC_EXE_Entry'];
  var
    xframe: TEXCEPTION_FRAME;
  begin
     EntryInformation:=info;
     IsLibrary:=false;
     { install the handlers for exe only ?
       or should we install them for DLL also ? (PM) }
{$ifndef FPC_USE_WIN32_SEH}
     install_exception_handlers;
{$endif FPC_USE_WIN32_SEH}
     { This strange construction is needed to solve the _SS problem
       with a smartlinked syswin32 (PFV) }
     asm
        { movl  %esp,%fs:(0)
          but don't insert it as it doesn't
          point to anything yet
          this will be used in signals unit }
        leal xframe,%eax
        movl %fs:(0),%ecx
        movl %ecx,TException_Frame.next(%eax)
        movl %eax,System_exception_frame
{$ifndef FPC_USE_WIN32_SEH}
        movl $0,TException_Frame.handler(%eax)
{$else}
        movl $OutermostHandler,TException_Frame.handler(%eax)
        movl %eax,%fs:(0)
{$endif FPC_USE_WIN32_SEH}
        pushl %ebp
        xorl %eax,%eax
        movw %ss,%ax
        movl %eax,_SS
        xorl %ebp,%ebp
     end;
<<<<<<< HEAD
<<<<<<< HEAD
     EntryInformation.PascalMain();
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
     EntryInformation.PascalMain();
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
     PascalMain;
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     asm
        popl %ebp
     end;
     { if we pass here there was no error ! }
     system_exit;
  end;

<<<<<<< HEAD
=======

Const
  { DllEntryPoint  }
     DLL_PROCESS_ATTACH = 1;
     DLL_THREAD_ATTACH = 2;
     DLL_PROCESS_DETACH = 0;
     DLL_THREAD_DETACH = 3;
Var
     DLLBuf : Jmp_buf;
Const
     DLLExitOK : boolean = true;

function Dll_entry(const info : TEntryInformation) : longbool; [public,alias:'_FPC_DLL_Entry'];
  var
    res : longbool;
  begin
     EntryInformation:=info;
     IsLibrary:=true;
     Dll_entry:=false;
     case DLLreason of
       DLL_PROCESS_ATTACH :
         begin
           If SetJmp(DLLBuf) = 0 then
             begin
               if assigned(Dll_Process_Attach_Hook) then
                 begin
                   res:=Dll_Process_Attach_Hook(DllParam);
                   if not res then
                     exit(false);
                 end;
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
               EntryInformation.PascalMain();
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
               PascalMain;
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
               Dll_entry:=true;
             end
           else
             Dll_entry:=DLLExitOK;
         end;
       DLL_THREAD_ATTACH :
         begin
           inclocked(Thread_count);
{ Allocate Threadvars ?!}
           if assigned(Dll_Thread_Attach_Hook) then
             Dll_Thread_Attach_Hook(DllParam);
           Dll_entry:=true; { return value is ignored }
         end;
       DLL_THREAD_DETACH :
         begin
           declocked(Thread_count);
           if assigned(Dll_Thread_Detach_Hook) then
             Dll_Thread_Detach_Hook(DllParam);
{ Release Threadvars ?!}
           Dll_entry:=true; { return value is ignored }
         end;
       DLL_PROCESS_DETACH :
         begin
           Dll_entry:=true; { return value is ignored }
           If SetJmp(DLLBuf) = 0 then
             FPC_Do_Exit;
           if assigned(Dll_Process_Detach_Hook) then
             Dll_Process_Detach_Hook(DllParam);
         end;
     end;
  end;

Procedure ExitDLL(Exitcode : longint);
begin
    DLLExitOK:=ExitCode=0;
    LongJmp(DLLBuf,1);
end;


function GetCurrentProcess : dword;
 stdcall;external 'kernel32' name 'GetCurrentProcess';

function ReadProcessMemory(process : dword;address : pointer;dest : pointer;size : dword;bytesread : pdword) :  longbool;
 stdcall;external 'kernel32' name 'ReadProcessMemory';

>>>>>>> graemeg/fixes_2_2
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

{$ifdef Set_i386_Exception_handler}

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


  PExceptionPointers = ^TExceptionPointers;
  TExceptionPointers = packed record
          ExceptionRecord   : PExceptionRecord;
          ContextRecord     : PContext;
  end;

{ type of functions that should be used for exception handling }
  TTopLevelExceptionFilter = function (excep : PExceptionPointers) : Longint;stdcall;

{$i seh32.inc}

{$ifndef FPC_USE_WIN32_SEH}
function SetUnhandledExceptionFilter(lpTopLevelExceptionFilter : TTopLevelExceptionFilter) : TTopLevelExceptionFilter;
        stdcall;external 'kernel32' name 'SetUnhandledExceptionFilter';

const
  MaxExceptionLevel = 16;
  exceptLevel : Byte = 0;

var
  exceptEip       : array[0..MaxExceptionLevel-1] of Longint;
  exceptError     : array[0..MaxExceptionLevel-1] of Byte;
  resetFPU        : array[0..MaxExceptionLevel-1] of Boolean;

{$ifdef SYSTEMEXCEPTIONDEBUG}
procedure DebugHandleErrorAddrFrame(error : longint; addr, frame : pointer);
begin
  if IsConsole then
    begin
      write(stderr,'HandleErrorAddrFrame(error=',error);
      write(stderr,',addr=',hexstr(ptruint(addr),8));
      writeln(stderr,',frame=',hexstr(ptruint(frame),8),')');
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
      movl   ebp,%ecx
      movl   eip,%edx
      movl   error,%eax
      pushl  eip
      movl   ebp,%ebp // Change frame pointer

{$ifdef SYSTEMEXCEPTIONDEBUG}
      jmpl   DebugHandleErrorAddrFrame
{$else not SYSTEMEXCEPTIONDEBUG}
      jmpl   HandleErrorAddrFrame
{$endif SYSTEMEXCEPTIONDEBUG}
    end;
  end;

function syswin32_i386_exception_handler(excep : PExceptionPointers) : Longint;stdcall;
  var
    res: longint;
    err: byte;
    must_reset_fpu: boolean;
  begin
    res := EXCEPTION_CONTINUE_SEARCH;
    if excep^.ContextRecord^.SegSs=_SS then begin
      err := 0;
      must_reset_fpu := true;
{$ifdef SYSTEMEXCEPTIONDEBUG}
      if IsConsole then Writeln(stderr,'Exception  ',
              hexstr(excep^.ExceptionRecord^.ExceptionCode, 8));
{$endif SYSTEMEXCEPTIONDEBUG}
      case excep^.ExceptionRecord^.ExceptionCode of
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
    {excep^.ContextRecord^.FloatSave.StatusWord := excep^.ContextRecord^.FloatSave.StatusWord and $ffffff00;}
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
          { if we're testing sse support, simply set the flag and continue }
          if sse_check then
            begin
              os_supports_sse:=false;
              { skip the offending movaps %xmm7, %xmm6 instruction }
              inc(excep^.ContextRecord^.Eip,3);
              excep^.ExceptionRecord^.ExceptionCode := 0;
              res:=EXCEPTION_CONTINUE_EXECUTION;
            end
          else
            err := 216;
        STATUS_ACCESS_VIOLATION:
          { Athlon prefetch bug? }
          if is_prefetch(pointer(excep^.ContextRecord^.Eip)) then
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
              err := 255;
          end;
      end;

      if (err <> 0) and (exceptLevel < MaxExceptionLevel) then begin
        exceptEip[exceptLevel] := excep^.ContextRecord^.Eip;
        exceptError[exceptLevel] := err;
        resetFPU[exceptLevel] := must_reset_fpu;
        inc(exceptLevel);

        excep^.ContextRecord^.Eip := Longint(@JumpToHandleErrorFrame);
        excep^.ExceptionRecord^.ExceptionCode := 0;

        res := EXCEPTION_CONTINUE_EXECUTION;
{$ifdef SYSTEMEXCEPTIONDEBUG}
        if IsConsole then begin
          writeln(stderr,'Exception Continue Exception set at ',
                  hexstr(exceptEip[exceptLevel],8));
          writeln(stderr,'Eip changed to ',
                  hexstr(longint(@JumpToHandleErrorFrame),8), ' error=', err);
        end;
{$endif SYSTEMEXCEPTIONDEBUG}
      end;
    end;
    syswin32_i386_exception_handler := res;
  end;

procedure install_exception_handlers;
  begin
    SetUnhandledExceptionFilter(@syswin32_i386_exception_handler);
  end;

procedure remove_exception_handlers;
  begin
    SetUnhandledExceptionFilter(nil);
  end;
{$endif not FPC_USE_WIN32_SEH}

{$else not cpui386 (Processor specific !!)}
procedure install_exception_handlers;
begin
end;

procedure remove_exception_handlers;
begin
end;
{$endif Set_i386_Exception_handler}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_SECTION_THREADVARS}
function fpc_tls_add(addr: pointer): pointer; assembler; nostackframe;
  [public,alias: 'FPC_TLS_ADD']; compilerproc;
  asm
      sub   $tls_data_start,%eax
      cmpb  $0,IsLibrary
      mov   _tls_index,%ecx
      jnz   .L1
      mov   %fs:(0x2c),%edx
      add   (%edx,%ecx,4),%eax
      ret
.L1:
      push  %ebx
      mov   %eax,%ebx
      call  GetLastError
      push  %eax                      { save LastError }
      push  _tls_index
      call  TlsGetValue
      test  %eax,%eax
      jnz   .L2
      { This can happen when a thread existed before DLL was loaded,
        or if DisableThreadLibraryCalls was called. }
      call  SysAllocateThreadVars
      mov   $0x1000000,%eax
      call  InitThread
      push  _tls_index
      call  TlsGetValue
.L2:
      add   %eax,%ebx
      call  SetLastError              { restore (value is on stack) }
      mov   %ebx,%eax
      pop   %ebx
=======
{****************************************************************************
                      OS dependend widestrings
****************************************************************************}

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
>>>>>>> graemeg/fixes_2_2
=======
  end;


function Win32WideUpper(const s : WideString) : WideString;
  begin
    result:=s;
    UniqueString(result);
    if length(result)>0 then
      CharUpperBuff(LPWSTR(result),length(result));
>>>>>>> origin/fixes_2_2
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
       BaseOfData : longint;
       ImageBase : longint;
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
       SizeOfStackReserve : longint;
       SizeOfStackCommit : longint;
       SizeOfHeapReserve : longint;
       SizeOfHeapCommit : longint;
       LoaderFlags : longint;
       NumberOfRvaAndSizes : longint;
       DataDirectory : array[1..$80] of byte;
    end;
  begin
    result:=tpeheader((pointer(getmodulehandle(nil))+(tdosheader(pointer(getmodulehandle(nil))^).e_lfanew))^).SizeOfStackReserve;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{******************************************************************************}
{ include code common with win64 }

{$I syswin.inc}
{******************************************************************************}


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
	     BaseOfData : longint;
	     ImageBase : longint;
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
	     SizeOfStackReserve : longint;
	     SizeOfStackCommit : longint;
	     SizeOfHeapReserve : longint;
	     SizeOfHeapCommit : longint;
	     LoaderFlags : longint;
	     NumberOfRvaAndSizes : longint;
	     DataDirectory : array[1..$80] of byte;
	  end;
	begin
	  result:=tpeheader((pointer(SysInstance)+(tdosheader(pointer(SysInstance)^).e_lfanew))^).SizeOfStackReserve;
	end;

>>>>>>> graemeg/cpstrnew
=======
{****************************************************************************
                    Error Message writing using messageboxes
****************************************************************************}

function MessageBox(w1:longint;l1,l2:pointer;w2:longint):longint;
   stdcall;external 'user32' name 'MessageBoxA';

const
  ErrorBufferLength = 1024;
var
  ErrorBuf : array[0..ErrorBufferLength] of char;
  ErrorLen : longint;

Function ErrorWrite(Var F: TextRec): Integer;
{
  An error message should always end with #13#10#13#10
}
var
  p : pchar;
  i : longint;
Begin
  if F.BufPos>0 then
   begin
     if F.BufPos+ErrorLen>ErrorBufferLength then
       i:=ErrorBufferLength-ErrorLen
     else
       i:=F.BufPos;
     Move(F.BufPtr^,ErrorBuf[ErrorLen],i);
     inc(ErrorLen,i);
     ErrorBuf[ErrorLen]:=#0;
   end;
  if ErrorLen>3 then
   begin
     p:=@ErrorBuf[ErrorLen];
     for i:=1 to 4 do
      begin
        dec(p);
        if not(p^ in [#10,#13]) then
         break;
      end;
   end;
   if ErrorLen=ErrorBufferLength then
     i:=4;
   if (i=4) then
    begin
      MessageBox(0,@ErrorBuf,pchar('Error'),0);
      ErrorLen:=0;
    end;
  F.BufPos:=0;
  ErrorWrite:=0;
End;


Function ErrorClose(Var F: TextRec): Integer;
begin
  if ErrorLen>0 then
   begin
     MessageBox(0,@ErrorBuf,pchar('Error'),0);
     ErrorLen:=0;
   end;
  ErrorLen:=0;
  ErrorClose:=0;
end;


Function ErrorOpen(Var F: TextRec): Integer;
Begin
  TextRec(F).InOutFunc:=@ErrorWrite;
  TextRec(F).FlushFunc:=@ErrorWrite;
  TextRec(F).CloseFunc:=@ErrorClose;
  ErrorOpen:=0;
End;


procedure AssignError(Var T: Text);
begin
  Assign(T,'');
  TextRec(T).OpenFunc:=@ErrorOpen;
  Rewrite(T);
end;


procedure SysInitStdIO;
begin
  { Setup stdin, stdout and stderr, for GUI apps redirect stderr,stdout to be
    displayed in a messagebox }
  StdInputHandle:=longint(GetStdHandle(cardinal(STD_INPUT_HANDLE)));
  StdOutputHandle:=longint(GetStdHandle(cardinal(STD_OUTPUT_HANDLE)));
  StdErrorHandle:=longint(GetStdHandle(cardinal(STD_ERROR_HANDLE)));
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

{ ProcessID cached to avoid repeated calls to GetCurrentProcess. }

var
  ProcessID: SizeUInt;

function GetProcessID: SizeUInt;
begin
 GetProcessID := ProcessID;
end;

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
     BaseOfData : longint;
     ImageBase : longint;
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
     SizeOfStackReserve : longint;
     SizeOfStackCommit : longint;
     SizeOfHeapReserve : longint;
     SizeOfHeapCommit : longint;
     LoaderFlags : longint;
     NumberOfRvaAndSizes : longint;
     DataDirectory : array[1..$80] of byte;
  end;
begin
  result:=tpeheader((pointer(HInstance)+(tdosheader(pointer(HInstance)^).e_lfanew))^).SizeOfStackReserve;
end;

{
const
   Exe_entry_code : pointer = @Exe_entry;
   Dll_entry_code : pointer = @Dll_entry;
}
>>>>>>> graemeg/fixes_2_2

begin
  { get some helpful informations }
  GetStartupInfo(@startupinfo);

  SysResetFPU;
  if not(IsLibrary) then
    SysInitFPU;

  { some misc Win32 stuff }
  if not IsLibrary then
    SysInstance:=getmodulehandle(nil);

<<<<<<< HEAD
<<<<<<< HEAD
  MainInstance:=SysInstance;
=======
  MainInstance:=HInstance;
>>>>>>> graemeg/fixes_2_2
=======
  MainInstance:=HInstance;
>>>>>>> origin/fixes_2_2

  { pass dummy value }
  StackLength := CheckInitialStkLen($1000000);
  StackBottom := StackTop - StackLength;

  cmdshow:=startupinfo.wshowwindow;
  { Setup heap and threading, these may be already initialized from TLS callback }
  if not Assigned(CurrentTM.BeginThread) then
  begin
    InitHeap;
    InitSystemThreads;
  end;
  SysInitExceptions;
  { setup fastmove stuff }
  fpc_cpucodeinit;
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
