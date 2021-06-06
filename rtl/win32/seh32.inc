{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2013 by Free Pascal development team

    Support for 32-bit Windows exception handling

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
  EXCEPTION_UNWIND = EXCEPTION_UNWINDING or EXCEPTION_EXIT_UNWIND;

type
  TDispatcherContext=record
  end;

  PSEHFrame=^TSEHFrame;
  TSEHFrame=record
    Next: PSEHFrame;
    Addr: Pointer;
    _EBP: PtrUint;
    HandlerArg: Pointer;
  end;


procedure RtlUnwind(
  TargetFrame: Pointer;
  TargetIp: Pointer;
  ExceptionRecord: PExceptionRecord;
  ReturnValue: Pointer);
  stdcall; external 'kernel32.dll' name 'RtlUnwind';

{$ifdef FPC_USE_WIN32_SEH}
function NullHandler(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl;
begin
  result:=ExceptionContinueSearch;
end;


function GetBacktrace(Context: TContext; StartingFrame: Pointer; out Frames: PPointer): Longint;
var
  FrameCount: Longint;
  oldebp: Cardinal;
begin
  Frames:=AllocMem(RaiseMaxFrameCount*sizeof(pointer));
  FrameCount:=0;
  repeat
    oldebp:=context.ebp;
    { get_caller_stackinfo checks against StackTop on i386 }
    get_caller_stackinfo(pointer(Context.Ebp),codepointer(Context.Eip));
    if (Context.ebp<=oldebp) or (FrameCount>=RaiseMaxFrameCount) then
      break;
    if (Pointer(Context.ebp)>StartingFrame) or (FrameCount>0) then
    begin
      Frames[FrameCount]:=Pointer(Context.eip);
      Inc(FrameCount);
    end;
  until False;
  result:=FrameCount;
end;


function RunErrorCode386(const rec: TExceptionRecord; const context: TContext): Longint;
begin
  result:=RunErrorCode(rec);
  { deal with SSE exceptions }
  if (result=-255) and ((context.ContextFlags and CONTEXT_EXTENDED_REGISTERS)<>0) then
    TranslateMxcsr(PLongword(@context.ExtendedRegisters[24])^,result);
end;


procedure fpc_RaiseException(Obj: TObject; AnAddr,AFrame: Pointer); [public,alias: 'FPC_RAISEEXCEPTION']; compilerproc;
var
  ctx: TContext;
  args: array[0..4] of PtrUint;
begin
  ctx.Ebp:=Cardinal(AFrame);
  ctx.Eip:=Cardinal(AnAddr);
  args[0]:=PtrUint(AnAddr);
  args[1]:=PtrUint(Obj);
  args[2]:=GetBacktrace(ctx,AFrame,PPointer(args[3]));
  args[4]:=PtrUInt(AFrame);
  RaiseException(FPC_EXCEPTION_CODE,EXCEPTION_NONCONTINUABLE,5,@args[0]);
end;


procedure fpc_reraise; [public,alias:'FPC_RERAISE']; compilerproc;
var
  hp: PExceptObject;
begin
  hp:=ExceptObjectStack;
  ExceptObjectStack:=hp^.next;
  { Since we're going to 'reraise' the original OS exception (or, more exactly, pretend
    it wasn't handled), we must revert action of CommonHandler. }
  if TExceptionRecord(hp^.ExceptRec^).ExceptionCode<>FPC_EXCEPTION_CODE then
  begin
    if assigned(hp^.frames) then
      freemem(hp^.frames);
    if hp^.refcount=0 then
      hp^.FObject.Free;
  end;
  TSEHFrame(hp^.SEHFrame^).Addr:=@NullHandler;
  longjmp(hp^.ReraiseBuf,1);
end;


{ Parameters are dummy and used to force "ret 16" at the end;
  this removes a TSEHFrame record from the stack }
procedure _fpc_leave(a1,a2,a3,a4:pointer); [public,alias:'_FPC_leave']; stdcall; compilerproc; assembler; nostackframe;
asm
     movl   4(%esp),%eax
     movl   %eax,%fs:(0)
     movl   %ebp,%eax
     call   16(%esp)
end;


function PopObjectStack: PExceptObject;
var
  hp: PExceptObject;
begin
  hp:=ExceptObjectStack;
  if hp=nil then
    halt(255)
  else
  begin
    ExceptObjectStack:=hp^.next;
    if assigned(hp^.frames) then
      freemem(hp^.frames);
  end;
  result:=hp;
end;


function __FPC_finally_handler(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl; [public,alias:'__FPC_finally_handler'];
begin
  if (rec.ExceptionFlags and EXCEPTION_UNWIND)<>0 then
  begin
    { prevent endless loop if things go bad in user routine }
    frame.Addr:=@NullHandler;
    TUnwindProc(frame.HandlerArg)(frame._EBP);
  end;
  result:=ExceptionContinueSearch;
end;


function __FPC_default_handler(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl; [public,alias:'__FPC_DEFAULT_HANDLER'];
var
  Exc: TExceptObject;
  code: longint;
begin
  if (rec.ExceptionFlags and EXCEPTION_UNWIND)=0 then
  begin
    { Athlon prefetch bug? }
    if (rec.ExceptionCode=STATUS_ACCESS_VIOLATION) and is_prefetch(pointer(context.eip)) then
    begin
      result:=ExceptionContinueExecution;
      exit;
    end
    else if (rec.ExceptionCode=STATUS_ILLEGAL_INSTRUCTION) and sse_check then
    begin
      os_supports_sse:=False;
      { skip the offending movaps %xmm7,%xmm6 instruction }
      inc(context.eip,3);
      result:=ExceptionContinueExecution;
      exit;
    end;

    RtlUnwind(@frame,nil,@rec,nil);
    asm
      { RtlUnwind destroys nonvolatile registers, this assembler block prevents
        regvar optimizations. }
    end ['ebx','esi','edi'];

    if rec.ExceptionCode<>FPC_EXCEPTION_CODE then
    begin
      code:=RunErrorCode386(rec,context);
      if code<0 then
        SysResetFPU;
      code:=abs(code);
      Exc.Addr:=rec.ExceptionAddress;
      Exc.FObject:=nil;
      if Assigned(ExceptObjProc) then
        Exc.FObject:=TObject(TExceptObjProc(ExceptObjProc)(code,rec));
      if Exc.FObject=nil then
      begin
        { This works because RtlUnwind does not actually unwind the stack on i386
          (and only on i386) }
        errorcode:=word(code);
        errorbase:=pointer(context.Ebp);
        erroraddr:=pointer(context.Eip);
        Halt(code);
      end;
      Exc.FrameCount:=GetBacktrace(context,nil,Exc.Frames);
    end
    else
    begin
      Exc.FObject:=TObject(rec.ExceptionInformation[1]);
      Exc.Addr:=rec.ExceptionInformation[0];
      Exc.Frames:=PCodePointer(rec.ExceptionInformation[3]);
      Exc.FrameCount:=ptruint(rec.ExceptionInformation[2]);
      code:=217;
    end;

    Exc.Refcount:=0;
    Exc.SEHFrame:=@frame;
    Exc.ExceptRec:=@rec;
    { link to ExceptObjectStack }
    Exc.Next:=ExceptObjectStack;
    ExceptObjectStack:=@Exc;

    if Assigned(ExceptProc) then
    begin
      ExceptProc(Exc.FObject,Exc.Addr,Exc.FrameCount,Exc.Frames);
      Halt(217);
    end
    else
    begin
      errorcode:=word(code);
      errorbase:=pointer(rec.ExceptionInformation[4]);
      erroraddr:=pointer(Exc.Addr);
      Halt(code);
    end;
  end;
  result:=ExceptionContinueExecution;
end;


function NestedHandler(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl;
var
  hp: PExceptObject;
begin
  if (rec.ExceptionFlags and EXCEPTION_UNWIND)<>0 then
  begin
    hp:=PopObjectStack;
    if hp^.refcount=0 then
      hp^.FObject.Free;
  end;
  result:=ExceptionContinueSearch;
end;

function __FPC_except_safecall(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl; forward;

procedure CommonHandler(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  TargetAddr: Pointer);
var
  Exc: TExceptObject;
  code: Longint;
begin
  if rec.ExceptionCode<>FPC_EXCEPTION_CODE then
  begin
    Exc.FObject:=nil;
    code:=RunErrorCode386(rec,context);
    if Assigned(ExceptObjProc) then
      Exc.FObject:=TObject(TExceptObjProc(ExceptObjProc)(abs(code),rec));
    if (Exc.FObject=nil) and (frame.Addr<>Pointer(@__FPC_except_safecall)) then
      Exit;
    Exc.Addr:=rec.ExceptionAddress;
    Exc.FrameCount:=GetBacktrace(context,nil,Exc.Frames);
    if code<0 then
      SysResetFPU;
  end
  else
  begin
    Exc.Addr:=rec.ExceptionInformation[0];
    Exc.FObject:=TObject(rec.ExceptionInformation[1]);
    Exc.Framecount:=Longint(PtrUInt(rec.ExceptionInformation[2]));
    Exc.Frames:=rec.ExceptionInformation[3];
  end;

  RtlUnwind(@frame,nil,@rec,nil);

  Exc.Refcount:=0;
  Exc.SEHFrame:=@frame;
  Exc.ExceptRec:=@rec;
  { link to ExceptObjectStack }
  Exc.Next:=ExceptObjectStack;
  ExceptObjectStack:=@Exc;

  frame.Addr:=@NestedHandler;
  if setjmp(Exc.ReraiseBuf)=0 then
  asm
      movl   Exc.FObject,%eax
      movl   frame,%edx
      movl   TargetAddr,%ecx              // load ebp-based var before changing ebp
      movl   TSEHFrame._EBP(%edx),%ebp
      jmpl   *%ecx
  end;
  { control comes here if exception is re-raised }
  rec.ExceptionFlags:=rec.ExceptionFlags and (not EXCEPTION_UNWINDING);
end;


function __FPC_except_handler(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl; [public,alias:'__FPC_except_handler'];
begin
  if (rec.ExceptionFlags and EXCEPTION_UNWIND)=0 then
  begin
    { Athlon prefetch bug? }
    if (rec.ExceptionCode=STATUS_ACCESS_VIOLATION) and
      is_prefetch(pointer(Context.eip)) then
    begin
      result:=ExceptionContinueExecution;
      exit;
    end;
    CommonHandler(rec,frame,context,frame.HandlerArg);
  end;
  result:=ExceptionContinueSearch;
end;

{ Safecall procedures are expected to handle OS exceptions even if they cannot be
  converted to language exceptions. This is indicated by distinct handler address. }
function __FPC_except_safecall(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl; [public,alias:'__FPC_except_safecall']; assembler; nostackframe;
asm
    jmp  __FPC_except_handler
end;


function __FPC_on_handler(
  var rec: TExceptionRecord;
  var frame: TSEHFrame;
  var context: TContext;
  var dispatch: TDispatcherContext): EXCEPTION_DISPOSITION; cdecl; [public,alias:'__FPC_on_handler'];
var
  TargetAddr: Pointer;
begin
  if (rec.ExceptionFlags and EXCEPTION_UNWIND)=0 then
  begin
    { Athlon prefetch bug? }
    if (rec.ExceptionCode=STATUS_ACCESS_VIOLATION) and
      is_prefetch(pointer(Context.eip)) then
    begin
      result:=ExceptionContinueExecution;
      exit;
    end;
    { Are we going to catch it? }
    TargetAddr:=FilterException(rec,0,PtrUInt(frame.HandlerArg),abs(RunErrorCode386(rec,context)));
    if assigned(TargetAddr) then
      CommonHandler(rec,frame,context,TargetAddr);
  end;
  result:=ExceptionContinueSearch;
end;


function fpc_safecallhandler(obj: TObject): HResult; [public,alias:'FPC_SAFECALLHANDLER']; compilerproc;
var
  hp: PExceptObject;
  exc: TObject;
begin
  hp:=PopObjectStack;
  exc:=hp^.FObject;
  if Assigned(obj) and Assigned(exc) then
    result:=obj.SafeCallException(exc,hp^.Addr)
  else
    result:=E_UNEXPECTED;
  if hp^.refcount=0 then
    exc.Free;
  asm
       movl   %ebp,%edx                             // save current frame
       movl   hp,%ecx
       movl   TExceptObject.SEHFrame(%ecx),%ecx     // target ESP minus sizeof(TSEHFrame)
       movl   (%ecx),%eax
       movl   %eax,%fs:(0)                          // restore SEH chain
       movl   __RESULT,%eax
       movl   TSEHFrame._EBP(%ecx),%ebp             // restore EBP
       leal   16(%ecx),%esp                         // restore ESP past the SEH frame
       jmpl   4(%edx)                               // jump to caller
  end;
end;


procedure fpc_doneexception;[public,alias:'FPC_DONEEXCEPTION'] compilerproc;
var
  hp: PExceptObject;
begin
  hp:=PopObjectStack;
  if hp^.refcount=0 then
    hp^.FObject.Free;
  erroraddr:=nil;
    asm
        movl   %ebp,%edx                             // save current frame
        movl   hp,%eax
        movl   TExceptObject.SEHFrame(%eax),%eax     // target ESP minus sizeof(TSEHFrame)
        movl   (%eax),%ecx
        movl   %ecx,%fs:(0)                          // restore SEH chain
        movl   TSEHFrame._EBP(%eax),%ebp             // restore EBP
        leal   16(%eax),%esp                         // restore ESP, removing SEH frame
        jmpl   4(%edx)                               // jump to caller
    end;
end;


function main_wrapper(arg: Pointer; proc: Pointer): ptrint; assembler; nostackframe;
asm
    xorl    %ecx,%ecx
    pushl   $__FPC_default_handler
    pushl   %fs:(%ecx)
    movl    %esp,%fs:(%ecx)
    call    *%edx
    xorl    %ecx,%ecx
    popl    %edx
    movl    %edx,%fs:(%ecx)
    popl    %ecx
end;

{$endif FPC_USE_WIN32_SEH}

