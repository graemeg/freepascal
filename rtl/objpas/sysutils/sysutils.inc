{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

  { MCBS functions }
  {$i sysansi.inc}
  {$i syscodepages.inc}

{$macro on}
{$define PathStr:=UnicodeString}
{$define PathPChar:=PWideChar}
{$define SYSUTILSUNICODE}
  { Read filename handling functions implementation }
  {$i fina.inc}
  { Read disk function implementations }
  {$i disk.inc}
{$undef SYSUTILSUNICODE}
{$define PathStr:=RawByteString}
{$define PathPChar:=PAnsiChar}
  { Read filename handling functions implementation }
  {$i fina.inc}
  { Read disk function implementations }
  {$i disk.inc}
{$undef PathStr}
{$undef PathPChar}

  { Read file utility functions implementation }
  {$i filutil.inc}

  { variant error codes }
  {$i varerror.inc}
<<<<<<< HEAD
<<<<<<< HEAD
  
  { Type helpers}
  {$IFNDEF VER2_6}
  {$i syshelp.inc}
  {$ENDIF}
=======
=======
>>>>>>> origin/fixes_2_2

    Function FileSearch (Const Name, DirList : String; ImplicitCurrentDir : Boolean = True) : String;
    Var
      I : longint;
      Temp : String;

    begin
      Result:=Name;
      temp:=SetDirSeparators(DirList);
      // Start with checking the file in the current directory
      If ImplicitCurrentDir and (Result <> '') and FileExists(Result) Then
        exit;
      while True do begin
        If Temp = '' then
          Break; // No more directories to search - fail
        I:=pos(PathSeparator,Temp);
        If I<>0 then
          begin
            Result:=Copy (Temp,1,i-1);
            system.Delete(Temp,1,I);
          end
        else
          begin
            Result:=Temp;
            Temp:='';
          end;
        If Result<>'' then
          Result:=IncludeTrailingPathDelimiter(Result)+name;
        If (Result <> '') and FileExists(Result) Then
          exit;
      end;
      result:='';
    end;

    Function ExeSearch (Const Name, DirList : String) : String;

    begin
    {$ifdef unix}
      Result := FileSearch(Name, DirList, False);
    {$else unix}
      Result := FileSearch(Name, DirList, True);
    {$endif unix}
    end;
>>>>>>> graemeg/fixes_2_2

  {$ifndef OS_FILEISREADONLY}
  Function FileIsReadOnly(const FileName: RawByteString): Boolean;
  begin
    Result := (FileGetAttr(FileName) and faReadOnly) <> 0;
  end;


  Function FileIsReadOnly(const FileName: UnicodeString): Boolean;
  begin
    Result := (FileGetAttr(FileName) and faReadOnly) <> 0;
  end;
  {$endif OS_FILEISREADONLY}


  {$ifndef OS_FILESETDATEBYNAME}
  Function FileSetDate (Const FileName : RawByteString;Age : Longint) : Longint;
  Var
    fd : THandle;
  begin
    { at least windows requires fmOpenWrite here }
    fd:=FileOpen(FileName,fmOpenWrite);
    If (Fd<>feInvalidHandle) then
      try
        Result:=FileSetDate(fd,Age);
      finally
        FileClose(fd);
      end
    else
  {$ifdef HAS_OSERROR}
      Result:=GetLastOSError;
  {$else}
      Result:=-1;
  {$endif}
  end;


  Function FileSetDate (Const FileName : UnicodeString;Age : Longint) : Longint;
  Var
    fd : THandle;
  begin
    { at least windows requires fmOpenWrite here }
    fd:=FileOpen(FileName,fmOpenWrite);
    If (Fd<>feInvalidHandle) then
      try
        Result:=FileSetDate(fd,Age);
      finally
        FileClose(fd);
      end
    else
  {$ifdef HAS_OSERROR}
      Result:=GetLastOSError;
  {$else}
      Result:=-1;
  {$endif}
  end;
  {$endif}

  { Read String Handling functions implementation }
  {$i sysstr.inc}

  { Read date & Time function implementations }
{$ifndef FPUNONE}
  {$i dati.inc}
{$endif}

{$IFNDEF HAS_GETTICKCOUNT}
function GetTickCount: LongWord;
begin
  Result := LongWord(GetTickCount64);
end;
{$ENDIF}

{$IFNDEF HAS_GETTICKCOUNT64}
function GetTickCount64: QWord;
begin
{$IFDEF FPU_NONE}
  {$IFDEF HAS_SYSTIMERTICK}
  Result := SysTimerTick;
  {$ELSE}
  Result := 0;
  {$ENDIF}
{$ELSE}
  Result := Trunc(Now * 24 * 60 * 60 * 1000);
{$ENDIF}
end;
{$ENDIF}

  { Read pchar handling functions implementation }
  {$i syspch.inc}

  { generic internationalisation code }
  {$i sysint.inc}

  { wide string functions }
  {$i syswide.inc}

{$ifdef FPC_HAS_UNICODESTRING}
  { unicode string functions }
  {$i sysuni.inc}
  {$i sysencoding.inc}
{$endif FPC_HAS_UNICODESTRING}
  
  { threading stuff }
  {$i sysuthrd.inc}

  { OS utility code }
  {$i osutil.inc}

    procedure FreeAndNil(var obj);
      var
        temp: tobject;
      begin
        temp:=tobject(obj);
        pointer(obj):=nil;
        temp.free;
      end;


  { Interfaces support }
  {$i sysuintf.inc}

    constructor Exception.Create(const msg : string);

      begin
         inherited create;
         fmessage:=msg;
      end;


    constructor Exception.CreateFmt(const msg : string; const args : array of const);

      begin
         inherited create;
         fmessage:=Format(msg,args);
      end;


    constructor Exception.CreateRes(ResString: PString);

      begin
         inherited create;
         fmessage:=ResString^;
      end;


    constructor Exception.CreateResFmt(ResString: PString; const Args: array of const);

      begin
         inherited create;
         fmessage:=Format(ResString^,args);
      end;


    constructor Exception.CreateHelp(const Msg: string; AHelpContext: Longint);

      begin
         inherited create;
         fmessage:=Msg;
         fhelpcontext:=AHelpContext;
      end;


    constructor Exception.CreateFmtHelp(const Msg: string; const Args: array of const;
      AHelpContext: Longint);

    begin
       inherited create;
       fmessage:=Format(Msg,args);
       fhelpcontext:=AHelpContext;
    end;


    constructor Exception.CreateResHelp(ResString: PString; AHelpContext: Longint);

    begin
       inherited create;
       fmessage:=ResString^;
       fhelpcontext:=AHelpContext;
    end;


    constructor Exception.CreateResFmtHelp(ResString: PString; const Args: array of const;
      AHelpContext: Longint);

    begin
       inherited create;
       fmessage:=Format(ResString^,args);
       fhelpcontext:=AHelpContext;
    end;

    Function Exception.ToString : String;
    begin
      Result:=ClassName+': '+Message;
    end;

    procedure EHeapMemoryError.FreeInstance;
    begin
       if AllowFree then
        inherited FreeInstance;
    end;


    Constructor EVariantError.CreateCode (Code : longint);
    begin
       case Code of
         VAR_OK:
           Create(SNoError);
         VAR_PARAMNOTFOUND:
           Create(SVarParamNotFound);
         VAR_TYPEMISMATCH:
           Create(SInvalidVarCast);
         VAR_BADVARTYPE:
           Create(SVarBadType);
         VAR_OVERFLOW:
           Create(SVarOverflow);
         VAR_BADINDEX:
           Create(SVarArrayBounds);
         VAR_ARRAYISLOCKED:
           Create(SVarArrayLocked);
         VAR_NOTIMPL:
           Create(SVarNotImplemented);
         VAR_OUTOFMEMORY:
           Create(SVarOutOfMemory);
         VAR_INVALIDARG:
           Create(SVarInvalid);
         VAR_UNEXPECTED,
         VAR_EXCEPTION:
           Create(SVarUnexpected);
         else
           CreateFmt(SUnknownErrorCode,[Code]);
       end;
       ErrCode:=Code;
    end;

{$ifdef windows}
function EExternal.GetExceptionRecord: PExceptionRecord;
begin
  result:=@FExceptionRecord;
end;

{$endif windows}

{$push}
{$S-}
Procedure CatchUnhandledException (Obj : TObject; Addr: CodePointer; FrameCount: Longint; Frames: PCodePointer);[public,alias:'FPC_BREAK_UNHANDLED_EXCEPTION'];
Var
  i : longint;
  hstdout : ^text;
<<<<<<< HEAD
<<<<<<< HEAD
  
begin
  if WriteErrorsToStdErr then
    hstdout:=@stderr
  else
    hstdout:=@stdout;
  Writeln(hstdout^,'An unhandled exception occurred at $',HexStr(Addr),':');
  if Obj is exception then
    Writeln(hstdout^,Obj.ClassName,': ',Exception(Obj).Message)
  else if Obj is TObject then
    Writeln(hstdout^,'Exception object ',Obj.ClassName,' is not of class Exception.')
  else
    Writeln(hstdout^,'Exception object is not a valid class.');
{$IFDEF HAS_OSERROR}
 {$IFDEF DEBUG_EXCEPTIONS_LASTOSERROR}
  WriteLn (HStdOut^, 'Last OS error detected in the RTL: ', GetLastOSError);
 {$ENDIF DEBUG_EXCEPTIONS_LASTOSERROR}
{$ENDIF HAS_OSERROR}
=======
begin
=======
begin
>>>>>>> origin/fixes_2_2
  hstdout:=@stdout;
  Writeln(hstdout^,'An unhandled exception occurred at $',HexStr(PtrUInt(Addr),sizeof(PtrUInt)*2),' :');
  if Obj is exception then
   begin
     Message:=Exception(Obj).ClassName+' : '+Exception(Obj).Message;
     Writeln(hstdout^,Message);
   end
  else
   Writeln(hstdout^,'Exception object ',Obj.ClassName,' is not of class Exception.');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Writeln(hstdout^,BackTraceStrFunc(Addr));
  if (FrameCount>0) then
    begin
      for i:=0 to FrameCount-1 do
        Writeln(hstdout^,BackTraceStrFunc(Frames[i]));
    end;
  Writeln(hstdout^,'');
end;

type
  PExceptMapEntry=^TExceptMapEntry;
  TExceptMapEntry=record
    code: byte;
    cls:  ExceptClass;
{$IFDEF FPC_HAS_FEATURE_RESOURCES}  // This is necessary for 2.4.4, which does not have reasources as a separate feature
    msg:  PResStringRec;
{$else FPC_HAS_FEATURE_RESOURCES}
    msg:  PString;
{$endif FPC_HAS_FEATURE_RESOURCES}
  end;

const
  exceptmap: array[0..29] of TExceptMapEntry = (
    (code: 200; cls: EDivByZero;     msg: @SDivByZero),
    (code: 201; cls: ERangeError;    msg: @SRangeError),
    (code: 202; cls: EStackOverflow; msg: @SStackOverflow),
    (code: 205; cls: EOverflow;      msg: @SOverflow),
    (code: 206; cls: EUnderflow;     msg: @SUnderflow),
    (code: 207; cls: EInvalidOp;     msg: @SInvalidOp),
    { Delphi distinguishes reDivByZero from reZeroDivide, but maps both to code 200. }
    (code: 208; cls: EZeroDivide;    msg: @SZeroDivide),
    (code: 210; cls: EObjectCheck;   msg: @SObjectCheckError),
    (code: 211; cls: EAbstractError; msg: @SAbstractError),
    (code: 212; cls: EExternalException; msg: @SExternalException),
    (code: 214; cls: EBusError;      msg: @SBusError),
    (code: 215; cls: EIntOverflow;   msg: @SIntOverflow),
    (code: 216; cls: EAccessViolation; msg: @SAccessViolation),
    (code: 217; cls: EControlC;      msg: @SControlC),
    (code: 218; cls: EPrivilege;     msg: @SPrivilege),
    (code: 219; cls: EInvalidCast;   msg: @SInvalidCast),
    (code: 220; cls: EVariantError;  msg: @SInvalidVarCast),
    (code: 221; cls: EVariantError;  msg: @SInvalidVarOp),
    (code: 222; cls: EVariantError;  msg: @SDispatchError),
    (code: 223; cls: EVariantError;  msg: @SVarArrayCreate),
    (code: 224; cls: EVariantError;  msg: @SVarNotArray),
    (code: 225; cls: EVariantError;  msg: @SVarArrayBounds),
    (code: 227; cls: EAssertionFailed; msg: @SAssertionFailed),
    (code: 228; cls: EIntfCastError; msg: @SIntfCastError),
    (code: 229; cls: ESafecallException; msg: @SSafecallException),
    (code: 231; cls: EConvertError;  msg: @SiconvError),
    (code: 232; cls: ENoThreadSupport; msg: @SNoThreadSupport),
    (code: 233; cls: ENoWideStringSupport; msg: @SSigQuit),
    (code: 234; cls: ENoWideStringSupport; msg: @SMissingWStringManager),
    (code: 235; cls: ENoDynLibsSupport; msg: @SNoDynLibsSupport)
  );

function FindExceptMapEntry(err: longint): PExceptMapEntry;
var
  i: longint;
begin
  for i:=low(exceptmap) to high(exceptmap) do
    if err=exceptmap[i].code then
    begin
      result:=@exceptmap[i];
      exit;
    end;
  result:=nil;
end;

Var OutOfMemory : EOutOfMemory;
    InValidPointer : EInvalidPointer;


<<<<<<< HEAD
Procedure RunErrorToExcept (ErrNo : Longint; Address : CodePointer; Frame : Pointer);
var
  E: Exception;
  HS: PString;
  Entry: PExceptMapEntry;
=======
Procedure RunErrorToExcept (ErrNo : Longint; Address,Frame : Pointer);

Var E : Exception;
    HS : PString;

>>>>>>> graemeg/fixes_2_2
begin
  Case Errno of
   1,203 : E:=OutOfMemory;
   204 : E:=InvalidPointer;
  else
    Entry:=FindExceptMapEntry(ErrNo);
    if Assigned(Entry) then
      E:=Entry^.cls.CreateRes(Entry^.msg)
    else
    begin
     HS:=nil;
     Case Errno of
       2 : HS:=@SFileNotFound;
       3 : HS:=@SInvalidFileName;
       4 : HS:=@STooManyOpenFiles;
       5 : HS:=@SAccessDenied;
       6 : HS:=@SInvalidFileHandle;
       15 : HS:=@SInvalidDrive;
       100 : HS:=@SEndOfFile;
       101 : HS:=@SDiskFull;
       102 : HS:=@SFileNotAssigned;
       103 : HS:=@SFileNotOpen;
       104 : HS:=@SFileNotOpenForInput;
       105 : HS:=@SFileNotOpenForOutput;
       106 : HS:=@SInvalidInput;
     end;
<<<<<<< HEAD
<<<<<<< HEAD
     if Assigned(HS) then
       E:=EInOutError.CreateRes(HS)
     else
       E:=EInOutError.CreateResFmt(@SUnknownRunTimeError,[errno]);
=======
     E:=EinOutError.Create (HS^);
>>>>>>> graemeg/fixes_2_2
=======
     E:=EinOutError.Create (HS^);
>>>>>>> origin/fixes_2_2
     // this routine can be called from FPC_IOCHECK,
     // which clears inoutres and then passes its
     // original value to HandleErrorFrame() (which calls
     // us). So use errno rather than IOResult, and clear
     // InOutRes explicitly in case we can also be called
     // from a place that does not clear InOutRes explicitly
     EInoutError(E).ErrorCode:=errno;
     inoutres:=0;
     end;
<<<<<<< HEAD
=======
  // We don't set abstracterrorhandler, but we do it here.
  // Unless the use sets another handler we'll get here anyway...
  200 : E:=EDivByZero.Create(SDivByZero);
  201 : E:=ERangeError.Create(SRangeError);
  205 : E:=EOverflow.Create(SOverflow);
  206 : E:=EOverflow.Create(SUnderflow);
  207 : E:=EInvalidOp.Create(SInvalidOp);
  211 : E:=EAbstractError.Create(SAbstractError);
  212 : E:=EExternalException.Create(SExternalException);
  214 : E:=EBusError.Create(SBusError);
  215 : E:=EIntOverflow.Create(SIntOverflow);
  216 : E:=EAccessViolation.Create(SAccessViolation);
  217 : E:=EControlC.Create(SControlC);
  218 : E:=EPrivilege.Create(SPrivilege);
  219 : E:=EInvalidCast.Create(SInvalidCast);
  220 : E:=EVariantError.Create(SInvalidVarCast);
  221 : E:=EVariantError.Create(SInvalidVarOp);
  222 : E:=EVariantError.Create(SDispatchError);
  223 : E:=EVariantError.Create(SVarArrayCreate);
  224 : E:=EVariantError.Create(SVarNotArray);
  225 : E:=EVariantError.Create(SVarArrayBounds);
  227 : E:=EAssertionFailed.Create(SAssertionFailed);
  228 : E:=EIntfCastError.Create(SIntfCastError);
  229 : E:=ESafecallException.Create(SSafecallException);
  231 : E:=EConvertError.Create(SiconvError);
  232 : E:=ENoThreadSupport.Create(SNoThreadSupport);
  233 : E:=ENoWideStringSupport.Create(SMissingWStringManager);
  else
   E:=Exception.CreateFmt (SUnKnownRunTimeError,[Errno]);
>>>>>>> graemeg/fixes_2_2
  end;
  Raise E at Address,Frame;
end;

{$IFDEF HAS_OSERROR}
Procedure RaiseLastOSError;overload;
begin
  RaiseLastOSError(GetLastOSError);
end;

Procedure RaiseLastOSError(LastError: Integer);overload;
var
  E : EOSError;
begin
  If (LastError<>0) then
    E:=EOSError.CreateFmt(SOSError, [LastError, SysErrorMessage(LastError)])
  else
    E:=EOSError.Create(SUnkOSError);
  E.ErrorCode:=LastError;
  Raise E;
end;

{$else}
Procedure RaiseLastOSError;overload;
begin
  Raise Exception.Create('RaiseLastOSError not implemented on this platform.');
end;

Procedure RaiseLastOSError(LastError: Integer);overload;
begin
  RaiseLastOSError;
end;

{$endif}
Procedure AssertErrorHandler (Const Msg,FN : ShortString;LineNo:longint; TheAddr : pointer);
Var
  S : String;
begin
  If Msg='' then
    S:=SAssertionFailed
  else
    S:=Msg;
<<<<<<< HEAD
<<<<<<< HEAD
  Raise EAssertionFailed.Createfmt(SAssertError,[S,Fn,LineNo]) at get_caller_addr(theAddr), get_caller_frame(theAddr);
=======
  Raise EAssertionFailed.Createfmt(SAssertError,[S,Fn,LineNo]) at Pointer(theAddr);
>>>>>>> graemeg/fixes_2_2
=======
  Raise EAssertionFailed.Createfmt(SAssertError,[S,Fn,LineNo]) at Pointer(theAddr);
>>>>>>> origin/fixes_2_2
end;

{$pop} //{$S-} for Error handling functions

Procedure InitExceptions;
{
  Must install uncaught exception handler (ExceptProc)
  and install exceptions for system exceptions or signals.
  (e.g: SIGSEGV -> ESegFault or so.)
}
begin
  ExceptionClass := Exception;
  ExceptProc:=@CatchUnhandledException;
  // Create objects that may have problems when there is no memory.
  OutOfMemory:=EOutOfMemory.Create(SOutOfMemory);
  OutOfMemory.AllowFree:=false;
  InvalidPointer:=EInvalidPointer.Create(SInvalidPointer);
  InvalidPointer.AllowFree:=false;
  AssertErrorProc:=@AssertErrorHandler;
  ErrorProc:=@RunErrorToExcept;
  OnShowException:=Nil;
end;


Procedure DoneExceptions;
begin
  OutOfMemory.AllowFree:=true;
  OutOfMemory.Free;
  InValidPointer.AllowFree:=true;
  InValidPointer.Free;
end;


{ Exception handling routines }

function ExceptObject: TObject;

begin
  If RaiseList=Nil then
    Result:=Nil
  else
    Result:=RaiseList^.FObject;
end;

function ExceptAddr: CodePointer;

begin
  If RaiseList=Nil then
    Result:=Nil
  else
    Result:=RaiseList^.Addr;
end;

function ExceptFrameCount: Longint;

begin
  If RaiseList=Nil then
    Result:=0
  else
    Result:=RaiseList^.Framecount;
end;

function ExceptFrames: PCodePointer;

begin
  If RaiseList=Nil then
    Result:=Nil
  else
    Result:=RaiseList^.Frames;
end;

function ExceptionErrorMessage(ExceptObject: TObject; ExceptAddr: Pointer;
                               Buffer: PChar; Size: Integer): Integer;

Var
  S : AnsiString;
  Len : Integer;

begin
  S:=Format(SExceptionErrorMessage,[ExceptAddr,ExceptObject.ClassName]);
  If ExceptObject is Exception then
    S:=Format('%s:'#10'%s',[S,Exception(ExceptObject).Message]);
  Len:=Length(S);
  If S[Len]<>'.' then
    begin
    S:=S+'.';
    Inc(len);
    end;
  If Len>Size then
    Len:=Size;
  if Len > 0 then
    Move(S[1],Buffer^,Len);
  Result:=Len;
end;

procedure ShowException(ExceptObject: TObject; ExceptAddr: Pointer);

// use shortstring. On exception, the heap may be corrupt.

Var
  Buf : ShortString;

begin
  SetLength(Buf,ExceptionErrorMessage(ExceptObject,ExceptAddr,@Buf[1],255));
  If IsConsole Then
    writeln(Buf)
  else
    If Assigned(OnShowException) Then
      OnShowException (Buf);
end;

procedure Abort;

begin
  Raise EAbort.Create(SAbortError) at CodePointer(Get_Caller_addr(Get_Frame));
end;

procedure OutOfMemoryError;

begin
  Raise OutOfMemory;
end;

{ ---------------------------------------------------------------------
    Initialization/Finalization/exit code
  ---------------------------------------------------------------------}

Type
  PPRecord = ^TPRecord;
  TPRecord = Record
    Func : TTerminateProc;
    NextFunc : PPRecord;
  end;

Const
  TPList : PPRecord = Nil;

procedure AddTerminateProc(TermProc: TTerminateProc);

Var
  TPR : PPRecord;

begin
  New(TPR);
  With TPR^ do
    begin
    NextFunc:=TPList;
    Func:=TermProc;
    end;
  TPList:=TPR;
end;

function CallTerminateProcs: Boolean;

Var
  TPR : PPRecord;

begin
  Result:=True;
  TPR:=TPList;
  While Result and (TPR<>Nil) do
    begin
    Result:=TPR^.Func();
    TPR:=TPR^.NextFunc;
    end;
end;

{ ---------------------------------------------------------------------
    Diskh functions, OS independent.
  ---------------------------------------------------------------------}

<<<<<<< HEAD
Function GetCurrentDir: {$ifdef FPC_UNICODE_RTL}UnicodeString{$else}AnsiString{$endif};
begin
  GetDir(0,Result);
end;

{ ---------------------------------------------------------------------
    Other functions, OS independent.
  ---------------------------------------------------------------------}

Var
  GUIDCalledRandomize : Boolean = False;
=======
function ForceDirectories(Const Dir: string): Boolean;

var
  E: EInOutError;
  ADrv : String;

function DoForceDirectories(Const Dir: string): Boolean;
var
  ADir : String;
  APath: String;
begin
  Result:=True;
  ADir:=ExcludeTrailingPathDelimiter(Dir);
  if (ADir='') then Exit;
  if Not DirectoryExists(ADir) then
    begin
      APath := ExtractFilePath(ADir);
      //this can happen on Windows if user specifies Dir like \user\name/test/
      //and would, if not checked for, cause an infinite recusrsion and a stack overflow
      if (APath = ADir) then Result := False
        else Result:=DoForceDirectories(APath);
    If Result then
      Result := CreateDir(ADir);
    end;
end;

function IsUncDrive(const Drv: String): Boolean;
begin
  Result := (Length(Drv) > 2) and (Drv[1] = PathDelim) and (Drv[2] = PathDelim);
end;

begin
  Result := False;
  ADrv := ExtractFileDrive(Dir);
  if (ADrv<>'') and (not DirectoryExists(ADrv))
  {$IFNDEF FORCEDIR_NO_UNC_SUPPORT} and (not IsUncDrive(ADrv)){$ENDIF} then Exit;
  if Dir='' then
    begin
      E:=EInOutError.Create(SCannotCreateEmptyDir);
      E.ErrorCode:=3;
      Raise E;
    end;
  Result := DoForceDirectories(SetDirSeparators(Dir));
end;
>>>>>>> graemeg/fixes_2_2

Var
  GUIDCalledRandomize : Boolean = False;

Var
  GUIDCalledRandomize : Boolean = False;

Procedure GetRandomBytes(Var Buf; NBytes : Integer);

Var
  I : Integer;
  P : PByte;

begin
  P:=@Buf;
  If Not GUIDCalledRandomize then
    begin
    Randomize;
    GUIDCalledRandomize:=True;
    end;
  For I:=0 to NBytes-1 do
    P[i]:=Random(256);
end;

{$IFDEF HASCREATEGUID}
Function SysCreateGUID(out GUID : TGUID) : Integer; forward;
{$ENDIF}

Function CreateGUID(out GUID : TGUID) : Integer;
begin
  If Assigned(OnCreateGUID) then
    Result:=OnCreateGUID(GUID)
  else
    begin
    {$IFDEF HASCREATEGUID}
    Result:=SysCreateGUID(GUID);
    {$ELSE}
    GetRandomBytes(GUID,SizeOf(Guid));
    guid.clock_seq_hi_and_reserved:=(guid.clock_seq_hi_and_reserved and $3F) + 64;
    guid.time_hi_and_version      :=(guid.time_hi_and_version and $0FFF)+ $4000;
    Result:=0;
    {$ENDIF}
    end;
end;


function SafeLoadLibrary(const FileName: AnsiString;
  ErrorMode: DWord = {$ifdef windows}SEM_NOOPENFILEERRORBOX{$else windows}0{$endif windows}): HMODULE;
{$if defined(cpui386) or defined(cpux86_64)}
  var
    mode : DWord;
    fpucw : Word;
    ssecw : DWord;
{$endif}
  begin
{$if defined(win64) or defined(win32)}
    mode:=SetErrorMode(ErrorMode);
{$endif}
    try
{$if defined(cpui386) or defined(cpux86_64)}
      fpucw:=Get8087CW;
{$ifdef cpui386}
      if has_sse_support then
{$endif cpui386}
        ssecw:=GetMXCSR;
{$endif}
{$if defined(windows) or defined(win32)}
      Result:=LoadLibraryA(PChar(Filename));
{$else}
      Result:=0;
{$endif}
      finally
{$if defined(cpui386) or defined(cpux86_64)}
      Set8087CW(fpucw);
{$ifdef cpui386}
      if has_sse_support then
{$endif cpui386}
        SetMXCSR(ssecw);
{$endif}
{$if defined(win64) or defined(win32)}
      SetErrorMode(mode);
{$endif}
    end;
  end;

function GetModuleName(Module: HMODULE): string;

begin
{$ifdef MSWINDOWS}
  SetLength(Result,MAX_PATH);
<<<<<<< HEAD
<<<<<<< HEAD
  SetLength(Result,GetModuleFileNameA(Module, Pchar(Result),Length(Result)));
=======
  SetLength(Result,GetModuleFileName(Module, Pchar(Result),Length(Result)));
>>>>>>> graemeg/fixes_2_2
=======
  SetLength(Result,GetModuleFileName(Module, Pchar(Result),Length(Result)));
>>>>>>> origin/fixes_2_2
{$ELSE}
  Result:='';
{$ENDIF}
end;
<<<<<<< HEAD
<<<<<<< HEAD

{ Beep support }

procedure Beep;
begin
  If Assigned(OnBeep) then
    OnBeep;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
