{
    This file is part of the Free Pascal run time library.
    This unit implements unix like signal handling for win32
    Copyright (c) 1999-2006 by the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit signals;

interface

{$PACKRECORDS C}

  { Signals }
  const
    SIGABRT   = 288;
    SIGFPE    = 289;
    SIGILL    = 290;
    SIGSEGV   = 291;
    SIGTERM   = 292;
    SIGALRM   = 293;
    SIGHUP    = 294;
    SIGINT    = 295;
    SIGKILL   = 296;
    SIGPIPE   = 297;
    SIGQUIT   = 298;
    SIGUSR1   = 299;
    SIGUSR2   = 300;
    SIGNOFP   = 301;
    SIGTRAP   = 302;
    SIGTIMR   = 303;    { Internal for setitimer (SIGALRM, SIGPROF) }
    SIGPROF   = 304;
    SIGMAX    = 320;

    SIG_BLOCK   = 1;
    SIG_SETMASK = 2;
    SIG_UNBLOCK = 3;

  function SIG_DFL( x: longint) : longint; cdecl;

  function SIG_ERR( x: longint) : longint; cdecl;

  function SIG_IGN( x: longint) : longint; cdecl;

  type

    SignalHandler  = function (v : longint) : longint;cdecl;

    PSignalHandler = ^SignalHandler; { to be compatible with linux.pp }

  function signal(sig : longint;func : SignalHandler) : SignalHandler;

  const

     EXCEPTION_MAXIMUM_PARAMETERS = 15;


{$define __HASCONTEXT__}
//
// Define 128-bit 16-byte aligned xmm register type.
//

//typedef struct DECLSPEC_ALIGN(16) _M128A {
{$note todo, fix alignment }
  type
     DWORD64 = QWORD;
     ULONGLONG = QWORD;
     LONGLONG = int64;

     M128A = record
          Low: ULONGLONG;
          High: LONGLONG;
       end;
     _M128A = M128A;
     TM128A = M128A;
     PM128A = TM128A;

//
// Format of data for 32-bit fxsave/fxrstor instructions.
//

//typedef struct _XMM_SAVE_AREA32 {
  type
     XMM_SAVE_AREA32 = record
          ControlWord: WORD;
          StatusWord: WORD;
          TagWord: BYTE;
          Reserved1: BYTE;
          ErrorOpcode: WORD;
          ErrorOffset: DWORD;
          ErrorSelector: WORD;
          Reserved2: WORD;
          DataOffset: DWORD;
          DataSelector: WORD;
          Reserved3: WORD;
          MxCsr: DWORD;
          MxCsr_Mask: DWORD;
          FloatRegisters: array[0..7] of M128A;
          XmmRegisters: array[0..16] of M128A;
          Reserved4: array[0..95] of BYTE;
       end;
     _XMM_SAVE_AREA32 = XMM_SAVE_AREA32;
     TXmmSaveArea = XMM_SAVE_AREA32;
     PXmmSaveArea = ^TXmmSaveArea;

  const
     LEGACY_SAVE_AREA_LENGTH = sizeof(XMM_SAVE_AREA32);

//
// Context Frame
//
//  This frame has a several purposes: 1) it is used as an argument to
//  NtContinue, 2) is is used to constuct a call frame for APC delivery,
//  and 3) it is used in the user level thread creation routines.
//
//
// The flags field within this record controls the contents of a CONTEXT
// record.
//
// If the context record is used as an input parameter, then for each
// portion of the context record controlled by a flag whose value is
// set, it is assumed that that portion of the context record contains
// valid context. If the context record is being used to modify a threads
// context, then only that portion of the threads context is modified.
//
// If the context record is used as an output parameter to capture the
// context of a thread, then only those portions of the thread's context
// corresponding to set flags will be returned.
//
// CONTEXT_CONTROL specifies SegSs, Rsp, SegCs, Rip, and EFlags.
//
// CONTEXT_INTEGER specifies Rax, Rcx, Rdx, Rbx, Rbp, Rsi, Rdi, and R8-R15.
//
// CONTEXT_SEGMENTS specifies SegDs, SegEs, SegFs, and SegGs.
//
// CONTEXT_DEBUG_REGISTERS specifies Dr0-Dr3 and Dr6-Dr7.
//
// CONTEXT_MMX_REGISTERS specifies the floating point and extended registers
//     Mm0/St0-Mm7/St7 and Xmm0-Xmm15).
//

//typedef struct DECLSPEC_ALIGN(16) _CONTEXT {
{$packrecords C}
  type

     CONTEXT = record

          //
          // Register parameter home addresses.
          //
          // N.B. These fields are for convience - they could be used to extend the
          //      context record in the future.
          //

          P1Home: DWORD64;
          P2Home: DWORD64;
          P3Home: DWORD64;
          P4Home: DWORD64;
          P5Home: DWORD64;
          P6Home: DWORD64;

          //
          // Control flags.
          //

          ContextFlags: DWORD;
          MxCsr: DWORD;

          //
          // Segment Registers and processor flags.
          //

          SegCs: WORD;
          SegDs: WORD;
          SegEs: WORD;
          SegFs: WORD;
          SegGs: WORD;
          SegSs: WORD;
          EFlags: DWORD;

          //
          // Debug registers
          //

          Dr0: DWORD64;
          Dr1: DWORD64;
          Dr2: DWORD64;
          Dr3: DWORD64;
          Dr6: DWORD64;
          Dr7: DWORD64;

          //
          // Integer registers.
          //

          Rax: DWORD64;
          Rcx: DWORD64;
          Rdx: DWORD64;
          Rbx: DWORD64;
          Rsp: DWORD64;
          Rbp: DWORD64;
          Rsi: DWORD64;
          Rdi: DWORD64;
          R8: DWORD64;
          R9: DWORD64;
          R10: DWORD64;
          R11: DWORD64;
          R12: DWORD64;
          R13: DWORD64;
          R14: DWORD64;
          R15: DWORD64;

          //
          // Program counter.
          //

          Rip: DWORD64;

          //
          // Floating point state.
          //

          FltSave: XMM_SAVE_AREA32; // MWE: only translated the FltSave part of the union
(*
          union  {
              XMM_SAVE_AREA32 FltSave;
              struct {
                  M128A Header[2];
                  M128A Legacy[8];
                  M128A Xmm0;
                  M128A Xmm1;
                  M128A Xmm2;
                  M128A Xmm3;
                  M128A Xmm4;
                  M128A Xmm5;
                  M128A Xmm6;
                  M128A Xmm7;
                  M128A Xmm8;
                  M128A Xmm9;
                  M128A Xmm10;
                  M128A Xmm11;
                  M128A Xmm12;
                  M128A Xmm13;
                  M128A Xmm14;
                  M128A Xmm15;
              };
          };
*)

          //
          // Vector registers.
          //

          VectorRegister: array[0..25] of M128A;
          VectorControl: DWORD64;

          //
          // Special debug control registers.
          //

          DebugControl: DWORD64;
          LastBranchToRip: DWORD64;
          LastBranchFromRip: DWORD64;
          LastExceptionToRip: DWORD64;
          LastExceptionFromRip: DWORD64;
      end;


(*
  type

     FLOATING_SAVE_AREA = record
          ControlWord : DWORD;
          StatusWord : DWORD;
          TagWord : DWORD;
          ErrorOffset : DWORD;
          ErrorSelector : DWORD;
          DataOffset : DWORD;
          DataSelector : DWORD;
          RegisterArea : array[0..79] of BYTE;
          Cr0NpxState : DWORD;
       end;
     _FLOATING_SAVE_AREA = FLOATING_SAVE_AREA;
     TFLOATINGSAVEAREA = FLOATING_SAVE_AREA;
     PFLOATINGSAVEAREA = ^FLOATING_SAVE_AREA;

     CONTEXT = record
          ContextFlags : DWORD;
          Dr0 : DWORD;
          Dr1 : DWORD;
          Dr2 : DWORD;
          Dr3 : DWORD;
          Dr6 : DWORD;
          Dr7 : DWORD;
          FloatSave : FLOATING_SAVE_AREA;
          SegGs : DWORD;
          SegFs : DWORD;
          SegEs : DWORD;
          SegDs : DWORD;
          Edi : DWORD;
          Esi : DWORD;
          Ebx : DWORD;
          Edx : DWORD;
          Ecx : DWORD;
          Eax : DWORD;
          Ebp : DWORD;
          Eip : DWORD;
          SegCs : DWORD;
          EFlags : DWORD;
          Esp : DWORD;
          SegSs : DWORD;
       end;

*)
     LPCONTEXT = ^CONTEXT;
     _CONTEXT = CONTEXT;
     TCONTEXT = CONTEXT;
     PCONTEXT = ^CONTEXT;


  type
     pexception_record = ^exception_record;
     EXCEPTION_RECORD  = record
       ExceptionCode   : cardinal;
       ExceptionFlags  : longint;
       ExceptionRecord : pexception_record;
       ExceptionAddress : pointer;
       NumberParameters : longint;
       ExceptionInformation : array[0..EXCEPTION_MAXIMUM_PARAMETERS-1] of pointer;
     end;

     PEXCEPTION_POINTERS = ^EXCEPTION_POINTERS;
     EXCEPTION_POINTERS = record
       ExceptionRecord   : PEXCEPTION_RECORD ;
       ContextRecord     : PCONTEXT ;
     end;


implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$asmmode att}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

const
     EXCEPTION_ACCESS_VIOLATION = $c0000005;
     EXCEPTION_BREAKPOINT = $80000003;
     EXCEPTION_DATATYPE_MISALIGNMENT = $80000002;
     EXCEPTION_SINGLE_STEP = $80000004;
     EXCEPTION_ARRAY_BOUNDS_EXCEEDED = $c000008c;
     EXCEPTION_FLT_DENORMAL_OPERAND = $c000008d;
     EXCEPTION_FLT_DIVIDE_BY_ZERO = $c000008e;
     EXCEPTION_FLT_INEXACT_RESULT = $c000008f;
     EXCEPTION_FLT_INVALID_OPERATION = $c0000090;
     EXCEPTION_FLT_OVERFLOW = $c0000091;
     EXCEPTION_FLT_STACK_CHECK = $c0000092;
     EXCEPTION_FLT_UNDERFLOW = $c0000093;
     EXCEPTION_INT_DIVIDE_BY_ZERO = $c0000094;
     EXCEPTION_INT_OVERFLOW = $c0000095;
     EXCEPTION_INVALID_HANDLE = $c0000008;
     EXCEPTION_PRIV_INSTRUCTION = $c0000096;
     EXCEPTION_NONCONTINUABLE_EXCEPTION = $c0000025;
     EXCEPTION_NONCONTINUABLE = $1;
     EXCEPTION_STACK_OVERFLOW = $c00000fd;
     EXCEPTION_INVALID_DISPOSITION = $c0000026;
     EXCEPTION_ILLEGAL_INSTRUCTION = $C000001D;
     EXCEPTION_IN_PAGE_ERROR = $C0000006;

     EXCEPTION_EXECUTE_HANDLER = 1;
     EXCEPTION_CONTINUE_EXECUTION = -(1);
     EXCEPTION_CONTINUE_SEARCH = 0;

  type
     { type of functions that should be used for exception handling }
     LPTOP_LEVEL_EXCEPTION_FILTER = function(excep :PEXCEPTION_POINTERS) : longint;stdcall;

     function SetUnhandledExceptionFilter(lpTopLevelExceptionFilter : LPTOP_LEVEL_EXCEPTION_FILTER)
       : LPTOP_LEVEL_EXCEPTION_FILTER;
       stdcall; external 'kernel32' name 'SetUnhandledExceptionFilter';

var
  signal_list : Array[SIGABRT..SIGMAX] of SignalHandler;
{ var
   value of the stack segment
    to check if the call stack can be written on exceptions
    this is without any object on win64
  _SS : cardinal;                      }

const
  Exception_handler_installed : boolean = false;
  MAX_Level = 16;
  except_level : byte = 0;
var
  except_rip   : array[0..Max_level-1] of dword64;
  except_signal : array[0..Max_level-1] of dword64;
  reset_fpu    : array[0..max_level-1] of boolean;

  procedure JumpToHandleSignal;
    var
      res, rip, _rbp, sigtype : dword64;
    begin
      asm
        movq (%rbp),%rax
        movq %rax,_rbp
      end;
{$ifdef SIGNALS_DEBUG}
      if IsConsole then
        Writeln(stderr,'In start of JumpToHandleSignal');
{$endif SIGNALS_DEBUG}

      if except_level>0 then
        dec(except_level)
      else
        RunError(216);
      rip:=except_rip[except_level];

      sigtype:=except_signal[except_level];
      if reset_fpu[except_level] then
        SysResetFPU;
      if assigned(System_exception_frame) then
        { get the handler in front again }
        asm
          movq  System_exception_frame,%rax
          movq  %rax,%gs:(0)
        end;
      if (sigtype>=SIGABRT) and (sigtype<=SIGMAX) and
         (signal_list[sigtype]<>@SIG_DFL) then
        begin
          res:=signal_list[sigtype](sigtype);
        end
      else
        res:=0;

      if res=0 then
        Begin
{$ifdef SIGNALS_DEBUG}
          if IsConsole then
            Writeln(stderr,'In JumpToHandleSignal');
{$endif SIGNALS_DEBUG}
          RunError(sigtype);
        end
      else
        { jump back to old code }
        asm
          movq rip,%rax
          push %rax
          movq _rbp,%rax
          push %rax
          leave
          ret
        end;
    end;



  function Signals_exception_handler
    (excep_exceptionrecord :PEXCEPTION_RECORD;
     excep_frame : PEXCEPTION_FRAME;
     excep_contextrecord : PCONTEXT;
     dispatch : pointer) : longint;stdcall;
    var frame,res  : longint;
        function CallSignal(sigtype,frame : longint;must_reset_fpu : boolean) : longint;
          begin
{$ifdef SIGNALS_DEBUG}
             if IsConsole then
               begin
                 writeln(stderr,'CallSignal called for signal ',sigtype);
                 dump_stack(stderr,pointer(frame));
               end;
{$endif SIGNALS_DEBUG}
            {if frame=0 then
              begin
                CallSignal:=1;
                writeln(stderr,'CallSignal frame is zero');
              end
            else    }
              begin
                 if except_level >= Max_level then
                   exit;
                 except_rip[except_level]:=excep_ContextRecord^.Rip;
                 except_signal[except_level]:=sigtype;
                 reset_fpu[except_level]:=must_reset_fpu;
                 inc(except_level);
                 {dec(excep^.ContextRecord^.Esp,4);
                 plongint (excep^.ContextRecord^.Esp)^ := longint(excep^.ContextRecord^.rip);}
                 excep_ContextRecord^.rip:=longint(@JumpToHandleSignal);
                 excep_ExceptionRecord^.ExceptionCode:=0;
                 CallSignal:=0;
{$ifdef SIGNALS_DEBUG}
                 if IsConsole then
                   writeln(stderr,'Exception_Continue_Execution set');
{$endif SIGNALS_DEBUG}
              end;
          end;

    begin
       frame:=excep_ContextRecord^.rbp;
       { default : unhandled !}
       res:=1;
{$ifdef SIGNALS_DEBUG}
       if IsConsole then
         writeln(stderr,'Signals exception  ',
           hexstr(excep_ExceptionRecord^.ExceptionCode,8));
{$endif SIGNALS_DEBUG}
       case excep_ExceptionRecord^.ExceptionCode of
         EXCEPTION_ACCESS_VIOLATION :
           res:=CallSignal(SIGSEGV,frame,false);
         { EXCEPTION_BREAKPOINT = $80000003;
         EXCEPTION_DATATYPE_MISALIGNMENT = $80000002;
         EXCEPTION_SINGLE_STEP = $80000004; }
         EXCEPTION_ARRAY_BOUNDS_EXCEEDED :
           res:=CallSignal(SIGSEGV,frame,false);
         EXCEPTION_FLT_DENORMAL_OPERAND :
           begin
             res:=CallSignal(SIGFPE,frame,true);
           end;
         EXCEPTION_FLT_DIVIDE_BY_ZERO :
           begin
             res:=CallSignal(SIGFPE,frame,true);
             {excep^.ContextRecord^.FloatSave.StatusWord:=excep^.ContextRecord^.FloatSave.StatusWord and $ffffff00;}
           end;
         {EXCEPTION_FLT_INEXACT_RESULT = $c000008f; }
         EXCEPTION_FLT_INVALID_OPERATION :
           begin
             res:=CallSignal(SIGFPE,frame,true);
           end;
         EXCEPTION_FLT_OVERFLOW :
           begin
             res:=CallSignal(SIGFPE,frame,true);
           end;
         EXCEPTION_FLT_STACK_CHECK :
           begin
             res:=CallSignal(SIGFPE,frame,true);
           end;
         EXCEPTION_FLT_UNDERFLOW :
           begin
             res:=CallSignal(SIGFPE,frame,true); { should be accepted as zero !! }
           end;
         EXCEPTION_INT_DIVIDE_BY_ZERO :
           res:=CallSignal(SIGFPE,frame,false);
         EXCEPTION_INT_OVERFLOW :
           res:=CallSignal(SIGFPE,frame,false);
         {EXCEPTION_INVALID_HANDLE = $c0000008;
         EXCEPTION_PRIV_INSTRUCTION = $c0000096;
         EXCEPTION_NONCONTINUABLE_EXCEPTION = $c0000025;
         EXCEPTION_NONCONTINUABLE = $1;}
         EXCEPTION_STACK_OVERFLOW :
           res:=CallSignal(SIGSEGV,frame,false);
         {EXCEPTION_INVALID_DISPOSITION = $c0000026;}
         EXCEPTION_ILLEGAL_INSTRUCTION,
         EXCEPTION_PRIV_INSTRUCTION,
         EXCEPTION_IN_PAGE_ERROR,
         EXCEPTION_SINGLE_STEP : res:=CallSignal(SIGSEGV,frame,false);
         { Ignore EXCEPTION_INVALID_HANDLE exceptions }
         EXCEPTION_INVALID_HANDLE : res:=0;
         end;
       Signals_exception_handler:=res;
    end;


    function API_signals_exception_handler(exceptptrs : PEXCEPTION_POINTERS) : longint; stdcall;
      begin
        API_signals_exception_handler:=Signals_exception_handler(
          @exceptptrs^.ExceptionRecord,
          nil,
          @exceptptrs^.ContextRecord,
          nil);
      end;


const
  PreviousHandler : LPTOP_LEVEL_EXCEPTION_FILTER = nil;
  Prev_Handler : pointer = nil;
  Prev_fpc_handler : pointer = nil;

  procedure install_exception_handler;
{$ifdef SIGNALS_DEBUG}
    var
      oldexceptaddr,newexceptaddr : longint;
{$endif SIGNALS_DEBUG}
    begin
      if Exception_handler_installed then
        exit;
      if assigned(System_exception_frame) then
        begin
          prev_fpc_handler:=System_exception_frame^.handler;
          System_exception_frame^.handler:=@Signals_exception_handler;
          { get the handler in front again }
          asm
            movq  %gs:(0),%rax
            movq  %rax,prev_handler
            movq  System_exception_frame,%rax
            movq  %rax,%gs:(0)
          end;
          Exception_handler_installed:=true;
          exit;
        end;
{$ifdef SIGNALS_DEBUG}
      asm
        movq $0,%rax
        movq %gs:(%rax),%rax
        movq %rax,oldexceptaddr
      end;
{$endif SIGNALS_DEBUG}
      PreviousHandler:=SetUnhandledExceptionFilter(@API_signals_exception_handler);
{$ifdef SIGNALS_DEBUG}
      asm
        movq $0,%rax
        movq %gs:(%rax),%rax
        movq %rax,newexceptaddr
      end;
      if IsConsole then
        begin
          writeln(stderr,'Old exception  ',hexstr(oldexceptaddr,8),
            ' new exception  ',hexstr(newexceptaddr,8));
          writeln('SetUnhandledExceptionFilter returned ',hexstr(longint(PreviousHandler),8));
        end;
{$endif SIGNALS_DEBUG}
      Exception_handler_installed := true;
    end;

  procedure remove_exception_handler;
    begin
      if not Exception_handler_installed then
        exit;
      if assigned(System_exception_frame) then
        begin
          if assigned(prev_fpc_handler) then
            System_exception_frame^.handler:=prev_fpc_handler;
          prev_fpc_handler:=nil;
          { restore old handler order again }
          if assigned(prev_handler) then
            asm
            movq  prev_handler,%rax
            movq  %rax,%gs:(0)
            end;
          prev_handler:=nil;
          Exception_handler_installed:=false;
          exit;
        end;
      SetUnhandledExceptionFilter(PreviousHandler);
      PreviousHandler:=nil;
      Exception_handler_installed:=false;
    end;


function SIG_ERR(x:longint):longint; cdecl;
begin
  SIG_ERR:=-1;
end;


function SIG_IGN(x:longint):longint; cdecl;
begin
  SIG_IGN:=-1;
end;


function SIG_DFL(x:longint):longint; cdecl;
begin
  SIG_DFL:=0;
end;

function signal(sig : longint;func : SignalHandler) : SignalHandler;
var
  temp : SignalHandler;
begin
  if ((sig < SIGABRT) or (sig > SIGMAX) or (sig = SIGKILL)) then
   begin
     signal:=@SIG_ERR;
     runerror(201);
   end;
  if not Exception_handler_installed then
    install_exception_handler;
  temp := signal_list[sig];
  signal_list[sig] := func;
  signal:=temp;
end;


var
  i : longint;
initialization

  for i:=SIGABRT to SIGMAX do
    signal_list[i]:=@SIG_DFL;

  {install_exception_handler;
   delay this to first use
  as other units also might install their handlers PM }

finalization
  remove_exception_handler;
end.
