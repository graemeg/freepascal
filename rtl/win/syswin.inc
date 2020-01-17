{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2008 by Florian Klaempfl and Pavel Ozerski
    member of the Free Pascal development team.

    FPC Pascal system unit part shared by win32/win64.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  Error code definitions for the Win32 API functions


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
  SEVERITY_SUCCESS                        = $00000000;
  SEVERITY_INFORMATIONAL                  = $40000000;
  SEVERITY_WARNING                        = $80000000;
  SEVERITY_ERROR                          = $C0000000;

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

  { Exceptions raised by RTL use this code }
  FPC_EXCEPTION_CODE                      = $E0465043;

  EXCEPTION_EXECUTE_HANDLER               = 1;
  EXCEPTION_CONTINUE_EXECUTION            = -1;
  EXCEPTION_CONTINUE_SEARCH               = 0;

  { exception flags (not everything applies to Win32!) }
  EXCEPTION_NONCONTINUABLE  = $01;
  EXCEPTION_UNWINDING       = $02;
  EXCEPTION_EXIT_UNWIND     = $04;
  EXCEPTION_STACK_INVALID   = $08;
  EXCEPTION_NESTED_CALL     = $10;
  EXCEPTION_TARGET_UNWIND   = $20;
  EXCEPTION_COLLIDED_UNWIND = $40;


  CONTEXT_X86                             = $00010000;
  CONTEXT_CONTROL                         = CONTEXT_X86 or $00000001;
  CONTEXT_INTEGER                         = CONTEXT_X86 or $00000002;
  CONTEXT_SEGMENTS                        = CONTEXT_X86 or $00000004;
  CONTEXT_FLOATING_POINT                  = CONTEXT_X86 or $00000008;
  CONTEXT_DEBUG_REGISTERS                 = CONTEXT_X86 or $00000010;
  CONTEXT_EXTENDED_REGISTERS              = CONTEXT_X86 or $00000020;

  CONTEXT_FULL                            = CONTEXT_CONTROL or CONTEXT_INTEGER or CONTEXT_SEGMENTS;

  MAXIMUM_SUPPORTED_EXTENSION             = 512;

type
  EXCEPTION_DISPOSITION=(
    ExceptionContinueExecution,
    ExceptionContinueSearch,
    ExceptionNestedException,
    ExceptionCollidedUnwind
  );

  TUnwindProc=procedure(frame: PtrUInt);

  PFilterRec=^TFilterRec;
  TFilterRec=record
    RvaClass: DWord;
    RvaHandler: DWord;
  end;

  TExceptObjProc=function(code: Longint; const rec: TExceptionRecord): Pointer; { Exception }
  TExceptClsProc=function(code: Longint): Pointer; { ExceptClass }


procedure RaiseException(
  dwExceptionCode: DWORD;
  dwExceptionFlags: DWORD;
  dwArgCount: DWORD;
  lpArguments: Pointer);  // msdn: *ULONG_PTR
  stdcall; external 'kernel32.dll' name 'RaiseException';


function RunErrorCode(const rec: TExceptionRecord): longint;
begin
  { negative result means 'FPU reset required' }
  case rec.ExceptionCode of
    STATUS_INTEGER_DIVIDE_BY_ZERO:      result := 200;    { reDivByZero }
    STATUS_FLOAT_DIVIDE_BY_ZERO:        result := -208;   { !!reZeroDivide }
    STATUS_ARRAY_BOUNDS_EXCEEDED:       result := 201;    { reRangeError }
    STATUS_STACK_OVERFLOW:              result := 202;    { reStackOverflow }
    STATUS_FLOAT_OVERFLOW:              result := -205;   { reOverflow }
    STATUS_FLOAT_DENORMAL_OPERAND,
    STATUS_FLOAT_UNDERFLOW:             result := -206;   { reUnderflow }
    STATUS_FLOAT_INEXACT_RESULT,
    STATUS_FLOAT_INVALID_OPERATION,
    STATUS_FLOAT_STACK_CHECK:           result := -207;   { reInvalidOp }
    STATUS_INTEGER_OVERFLOW:            result := 215;    { reIntOverflow }
    STATUS_ILLEGAL_INSTRUCTION:         result := -216;
    STATUS_ACCESS_VIOLATION:            result := 216;    { reAccessViolation }
    STATUS_CONTROL_C_EXIT:              result := 217;    { reControlBreak }
    STATUS_PRIVILEGED_INSTRUCTION:      result := 218;    { rePrivilegedInstruction }
    STATUS_FLOAT_MULTIPLE_TRAPS,
    STATUS_FLOAT_MULTIPLE_FAULTS:       result := -255;   { indicate FPU reset }
  else
    result := 255;                                        { reExternalException }
  end;
end;

procedure TranslateMxcsr(mxcsr: longword; var code: longint);
begin
  { we can return only one value, further one's are lost }
  { InvalidOp }
  if (mxcsr and 1)<>0 then
    code:=-207
  { Denormal }
  else if (mxcsr and 2)<>0 then
    code:=-206
  { !!reZeroDivide }
  else if (mxcsr and 4)<>0 then
    code:=-208
  { reOverflow }
  else if (mxcsr and 8)<>0 then
    code:=-205
  { Underflow }
  else if (mxcsr and 16)<>0 then
    code:=-206
  { Precision }
  else if (mxcsr and 32)<>0 then
    code:=-207
  else { this should not happen }
    code:=-255
end;

function FilterException(var rec:TExceptionRecord; imagebase: PtrUInt; filterRva: DWord; errcode: Longint): Pointer;
var
  ExClass: TClass;
  i: Longint;
  Filter: Pointer;
  curFilt: PFilterRec;
begin
  result:=nil;
  if rec.ExceptionCode=FPC_EXCEPTION_CODE then
    ExClass:=TObject(rec.ExceptionInformation[1]).ClassType
  else if Assigned(ExceptClsProc) then
    ExClass:=TClass(TExceptClsProc(ExceptClsProc)(errcode))
  else
    Exit; { if we cannot determine type of exception, don't handle it }
  Filter:=Pointer(imagebase+filterRva);
  for i:=0 to PLongint(Filter)^-1 do
  begin
    CurFilt:=@PFilterRec(Filter+sizeof(Longint))[i];
    if (CurFilt^.RvaClass=$FFFFFFFF) or
      { TODO: exception might be coming from another module, need more advanced comparing }
      (ExClass.InheritsFrom({$if not defined(ver3_0) and not defined(ver3_2)}PClass(imagebase+CurFilt^.RvaClass)^{$else}TClass(imagebase+CurFilt^.RvaClass){$endif})) then
    begin
      result:=Pointer(imagebase+CurFilt^.RvaHandler);
      exit;
    end;
  end;
end;

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




Var
  DLLInitState : Longint = -1;
  DLLBuf : Jmp_buf;

{$if defined(FPC_USE_WIN32_SEH) or defined(FPC_USE_WIN64_SEH)}
{$define FPC_USE_SEH}
{$endif}

function Dll_entry{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}(constref info : TEntryInformation){$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION} : longbool; [public,alias:'_FPC_DLL_Entry'];
  begin
{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
     SetupEntryInformation(info);
{$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}
     IsLibrary:=true;
     DllInitState:=DLLreason;
     Dll_entry:=false;  { return value is ignored, except when DLLreason=DLL_PROCESS_ATTACH }
     case DLLreason of
       DLL_PROCESS_ATTACH :
         begin
           MainThreadIdWin32 := Win32GetCurrentThreadId;

           If SetJmp(DLLBuf) = 0 then
             begin
{$ifdef FPC_USE_SEH}
               try
{$endif}
{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
               EntryInformation.PascalMain();
{$else FPC_HAS_INDIRECT_ENTRY_INFORMATION}
               PascalMain;
{$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}
               Dll_entry:=true;
{$ifdef FPC_USE_SEH}
               except
                 DoUnHandledException;
                 Dll_entry:=false;
               end;
{$endif}
             end
           else
             Dll_entry:=(ExitCode=0);
         end;
       DLL_THREAD_ATTACH :
         begin
           { SysInitMultithreading must not be called here,
             see comments in exec_tls_callback below }
           { Allocate Threadvars  }
           SysAllocateThreadVars;

           { NS : no idea what is correct to pass here - pass dummy value for now }
           { passing a dummy is ok, the correct value is read from the coff header of SysInstance (FK) }
           InitThread($1000000); { Assume everything is idempotent there, as the thread could have been created with BeginThread... }

           if assigned(Dll_Thread_Attach_Hook) then
             Dll_Thread_Attach_Hook(DllParam);
        end;
       DLL_THREAD_DETACH :
         begin
           if assigned(Dll_Thread_Detach_Hook) then
             Dll_Thread_Detach_Hook(DllParam);
           { Release Threadvars }
           if TlsGetValue(TLSKey^)<>nil then
             DoneThread; { Assume everything is idempotent there }
         end;
       DLL_PROCESS_DETACH :
         begin
           if MainThreadIDWin32=0 then // already been here.
             exit;
           If SetJmp(DLLBuf) = 0 then
             begin
               if assigned(Dll_Process_Detach_Hook) then
                 Dll_Process_Detach_Hook(DllParam);
               InternalExit;
             end;

           SysReleaseThreadVars;
           { Free TLS resources used by ThreadVars }
           SysFiniMultiThreading;
           MainThreadIDWin32:=0;
         end;
     end;
     DllInitState:=-1;
  end;


{****************************************************************************
                    Error Message writing using messageboxes
****************************************************************************}

function MessageBox(w1:THandle;l1,l2:pointer;w2:longint):longint;
   stdcall;external 'user32' name 'MessageBoxA';

const
  ErrorBufferLength = 1024;
var
  ErrorBuf : array[0..ErrorBufferLength] of char;
  ErrorLen : SizeInt;

procedure ErrorWrite(Var F: TextRec);
{
  An error message should always end with #13#10#13#10
}
var
  i : SizeInt;
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
          if not NoErrMsg then
            MessageBox(0,@ErrorBuf,pchar('Error'),0);
          ErrorLen:=0;
        end;
      Dec(F.BufPos,i);
    end;
End;


procedure ErrorClose(Var F: TextRec);
begin
  if ErrorLen>0 then
   begin
     MessageBox(0,@ErrorBuf,pchar('Error'),0);
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


procedure SysInitStdIO;
begin
  { Setup stdin, stdout and stderr, for GUI apps redirect stderr,stdout to be
    displayed in a messagebox }
  { WARNING: this should be done only once at startup,
    not for DLL entry code, as the standard handles might
    have been redirected }
  if StdInputHandle=0 then
    StdInputHandle:=THandle(GetStdHandle(cardinal(STD_INPUT_HANDLE)));
  if StdOutputHandle=0 then
    StdOutputHandle:=THandle(GetStdHandle(cardinal(STD_OUTPUT_HANDLE)));
  if StdErrorHandle=0 then
    StdErrorHandle:=THandle(GetStdHandle(cardinal(STD_ERROR_HANDLE)));
  if not IsConsole then
   begin
     AssignError(stderr);
     AssignError(StdOut);
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


{******************************************************************************
                              Unicode
 ******************************************************************************}
const
  { MultiByteToWideChar  }
  MB_PRECOMPOSED = 1;
  WC_NO_BEST_FIT_CHARS = $400;

function MultiByteToWideChar(CodePage:UINT; dwFlags:DWORD; lpMultiByteStr:PChar; cchMultiByte:longint; lpWideCharStr:PWideChar;cchWideChar:longint):longint;
    stdcall; external 'kernel32' name 'MultiByteToWideChar';
function WideCharToMultiByte(CodePage:UINT; dwFlags:DWORD; lpWideCharStr:PWideChar; cchWideChar:longint; lpMultiByteStr:PChar;cchMultiByte:longint; lpDefaultChar:PChar; lpUsedDefaultChar:pointer):longint;
    stdcall; external 'kernel32' name 'WideCharToMultiByte';
function CharUpperBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD;
    stdcall; external 'user32' name 'CharUpperBuffW';
function CharLowerBuff(lpsz:LPWSTR; cchLength:DWORD):DWORD;
    stdcall; external 'user32' name 'CharLowerBuffW';

procedure Win32Unicode2AnsiMove(source:punicodechar;var dest:RawByteString;cp : TSystemCodePage;len:SizeInt);
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

procedure Win32Ansi2UnicodeMove(source:pchar;cp : TSystemCodePage;var dest:UnicodeString;len:SizeInt);
  var
    destlen: SizeInt;
    dwflags: DWORD;
  begin
    // retrieve length including trailing #0
    // not anymore, because this must also be usable for single characters
    case cp of
    // Under https://docs.microsoft.com/en-us/windows/desktop/api/stringapiset/nf-stringapiset-multibytetowidechar
      CP_UTF8, CP_UTF7, 50220, 50221, 50222, 50225, 50227, 50229, 57002..57011, 42:
                           dwFlags:=0
      else
        dwFlags:=MB_PRECOMPOSED;
      end;
    destlen:=MultiByteToWideChar(cp, dwFlags, source, len, nil, 0);
    // this will null-terminate
    setlength(dest, destlen);
    if destlen>0 then
      begin
        MultiByteToWideChar(cp, dwFlags, source, len, @dest[1], destlen);
        PUnicodeRec(pointer(dest)-UnicodeFirstOff)^.CodePage:=CP_UTF16;
      end;
  end;


function Win32UnicodeUpper(const s : UnicodeString) : UnicodeString;
  begin
    result:=s;
    UniqueString(result);
    if length(result)>0 then
      CharUpperBuff(LPWSTR(result),length(result));
  end;


function Win32UnicodeLower(const s : UnicodeString) : UnicodeString;
  begin
    result:=s;
    UniqueString(result);
    if length(result)>0 then
      CharLowerBuff(LPWSTR(result),length(result));
  end;

{******************************************************************************
                              Widestring
 ******************************************************************************}

procedure Win32Ansi2WideMove(source:pchar;cp : TSystemCodePage;var dest:widestring;len:SizeInt);
  var
    destlen: SizeInt;
    dwFlags: DWORD;
  begin
    // retrieve length including trailing #0
    // not anymore, because this must also be usable for single characters
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


function Win32WideUpper(const s : WideString) : WideString;
  begin
    result:=s;
    if length(result)>0 then
      CharUpperBuff(LPWSTR(result),length(result));
  end;


function Win32WideLower(const s : WideString) : WideString;
  begin
    result:=s;
    if length(result)>0 then
      CharLowerBuff(LPWSTR(result),length(result));
  end;

type
  PWStrInitEntry = ^TWStrInitEntry;
  TWStrInitEntry = record
    addr: PPointer;
    data: Pointer;
  end;

  PWStrInitTablesTable = ^TWStrInitTablesTable;
  TWStrInitTablesTable = packed record
    count  : {$ifdef VER2_6}longint{$else}sizeint{$endif};
    tables : packed array [1..32767] of PWStrInitEntry;
  end;

var
{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
  WStrInitTablesTable: PWStrInitTablesTable;
{$else FPC_HAS_INDIRECT_ENTRY_INFORMATION}
  WStrInitTablesTableVar: TWStrInitTablesTable; external name 'FPC_WIDEINITTABLES';
  WStrInitTablesTable: PWStrInitTablesTable = @WStrInitTablesTableVar;
{$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}

function GetACP:UINT; stdcall; external 'kernel32' name 'GetACP';
function GetConsoleCP:UINT; stdcall; external 'kernel32' name 'GetConsoleCP';

function Win32GetStandardCodePage(const stdcp: TStandardCodePageEnum): TSystemCodePage;
  begin
    case stdcp of
      scpAnsi,
      scpFileSystemSingleByte: Result := GetACP;
      scpConsoleInput: Result := GetConsoleCP;
      scpConsoleOutput: Result := GetConsoleOutputCP;
    end;
  end;

{ there is a similiar procedure in sysutils which inits the fields which
  are only relevant for the sysutils units }
procedure InitWin32Widestrings;
  var
    i: longint;
    ptable: PWStrInitEntry;
  begin
{$if not(defined(VER2_2) or defined(VER2_4))}
    { assign initial values to global Widestring typed consts }
    for i:=1 to WStrInitTablesTable^.count do
      begin
        ptable:=WStrInitTablesTable^.tables[i];
        while Assigned(ptable^.addr) do
          begin
            fpc_widestr_assign(ptable^.addr^, ptable^.data);
            Inc(ptable);
          end;
      end;
{$endif}

    { Note: since WideChar=UnicodeChar and PWideChar=PUnicodeChar,
      Wide2AnsiMoveProc is identical to Unicode2AnsiStrMoveProc. }

    { Widestring }
    widestringmanager.Wide2AnsiMoveProc:=@Win32Unicode2AnsiMove;
    widestringmanager.Ansi2WideMoveProc:=@Win32Ansi2WideMove;
    widestringmanager.UpperWideStringProc:=@Win32WideUpper;
    widestringmanager.LowerWideStringProc:=@Win32WideLower;
    { Unicode }
    widestringmanager.Unicode2AnsiMoveProc:=@Win32Unicode2AnsiMove;
    widestringmanager.Ansi2UnicodeMoveProc:=@Win32Ansi2UnicodeMove;
    widestringmanager.UpperUnicodeStringProc:=@Win32UnicodeUpper;
    widestringmanager.LowerUnicodeStringProc:=@Win32UnicodeLower;
    { Codepage }
    widestringmanager.GetStandardCodePageProc:=@Win32GetStandardCodePage;

    DefaultSystemCodePage:=GetACP;
    DefaultUnicodeCodePage:=CP_UTF16;
    DefaultFileSystemCodePage:=CP_UTF8;
    DefaultRTLFileSystemCodePage:=DefaultSystemCodePage;
  end;

type
  WINBOOL = longbool;
  PHANDLER_ROUTINE = function (dwCtrlType:DWORD):WINBOOL; stdcall;

function SetConsoleCtrlHandler(HandlerRoutine:PHANDLER_ROUTINE; Add:WINBOOL):WINBOOL; stdcall; 
  external 'kernel32' name 'SetConsoleCtrlHandler';

function WinCtrlBreakHandler(dwCtrlType:DWORD): WINBOOL;stdcall;
const
  CTRL_BREAK_EVENT = 1;
begin
  if Assigned(CtrlBreakHandler) then 
    Result:=CtrlBreakHandler((dwCtrlType and CTRL_BREAK_EVENT > 0))
  else
    Result:=false;
end;

function SysSetCtrlBreakHandler (Handler: TCtrlBreakHandler): TCtrlBreakHandler;
begin
  (* Return either nil or previous handler *)
  if (Assigned(CtrlBreakHandler)) and (not Assigned(Handler)) then   
    SetConsoleCtrlHandler(@WinCtrlBreakHandler, false)
  else if (not Assigned(CtrlBreakHandler)) and (Assigned(Handler)) then
    SetConsoleCtrlHandler(@WinCtrlBreakHandler, true);

  SysSetCtrlBreakHandler := CtrlBreakHandler;
  CtrlBreakHandler := Handler;
end;
