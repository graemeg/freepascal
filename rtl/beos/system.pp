Unit System;

interface

// Was needed to bootstrap with our old 2.1 fpc for BeOS
// to define real
{ $define VER2_0}

{$define FPC_IS_SYSTEM}

{$I sysunixh.inc}

  
type
  THeapPointer = ^pointer;
var
  heapstartpointer : THeapPointer;
  heapstart : pointer;//external;//external name 'HEAP';
  myheapsize : longint; //external;//external name 'HEAPSIZE';
  myheaprealsize : longint;
  heap_handle : longint;
implementation

procedure debugger(s : PChar); cdecl; external 'root' name 'debugger';

function disable_debugger(state : integer): integer; external 'root' name 'disable_debugger';
//begin
//end;

{ OS independant parts}

{$I system.inc}

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}
procedure prthaltproc;external name '_haltproc';

procedure system_exit;
begin
  asm
    jmp prthaltproc
  end;
End;


{ OS dependant parts  }

{*****************************************************************************
                              Heap Management
*****************************************************************************}

(*var myheapstart:pointer;
    myheapsize:longint;
    myheaprealsize:longint;
    heap_handle:longint;
    zero:longint;


{ first address of heap }
function getheapstart:pointer;
begin
   getheapstart:=myheapstart;
end;

{ current length of heap }
function getheapsize:longint;
begin
   getheapsize:=myheapsize;
end;
*)


(*function getheapstart:pointer;
assembler;
asm
        leal    HEAP,%eax
end ['EAX'];


function getheapsize:longint;
assembler;
asm
        movl    intern_HEAPSIZE,%eax
end ['EAX'];*)

{ function to allocate size bytes more for the program }
{ must return the first address of new data space or nil if fail }
(*function Sbrk(size : longint):pointer;
var newsize,newrealsize:longint;
  s : string;
begin
  WriteLn('SBRK');
  Str(size, s);
  WriteLn('size : ' + s);
  if (myheapsize+size)<=myheaprealsize then 
  begin
    Sbrk:=pointer(heapstart+myheapsize);
    myheapsize:=myheapsize+size;
    exit;
  end;
  newsize:=myheapsize+size;
  newrealsize:=(newsize and $FFFFF000)+$1000;
  case resize_area(heap_handle,newrealsize) of
    B_OK : 
      begin
        WriteLn('B_OK');
        Sbrk:=pointer(heapstart+myheapsize);
        myheapsize:=newsize;
        myheaprealsize:=newrealsize;
        exit;
      end;
    B_BAD_VALUE : WriteLn('B_BAD_VALUE');
    B_NO_MEMORY : WriteLn('B_NO_MEMORY');
    B_ERROR : WriteLn('B_ERROR');
    else
      begin
        Sbrk:=pointer(heapstart+myheapsize);
        myheapsize:=newsize;
        myheaprealsize:=newrealsize;
        exit;
      end;
  end;

//  Sbrk:=nil;
end;*)

function sys_resize_area (handle:cardinal; size:longint):longint; cdecl; external name 'sys_resize_area';

//function sbrk2 (size : longint):pointer; cdecl; external name 'sbrk';

{ function to allocate size bytes more for the program }
{ must return the first address of new data space or nil if fail }
//function Sbrk(size : longint):pointer;
//var newsize,newrealsize:longint;
//  s : string;
//begin
//  sbrk := sbrk2(size);
(*  sbrk := nil;
  WriteLn('sbrk');
  Str(size, s);
  WriteLn('size : ' + s);
  if (myheapsize+size)<=myheaprealsize then 
  begin
    Sbrk:=heapstart+myheapsize;
    myheapsize:=myheapsize+size;
    exit;
  end;
  newsize:=myheapsize+size;
  newrealsize:=(newsize and $FFFFF000)+$1000;
  if sys_resize_area(heap_handle,newrealsize+$1000)=0 then 
  begin
    WriteLn('sys_resize_area OK');
    Str(longint(newrealsize), s);
    WriteLn('newrealsize : $' + Hexstr(longint(newrealsize), 8));
    Str(longint(heapstartpointer), s);
    WriteLn('heapstart : $' + Hexstr(longint(heapstart), 8));
    Str(myheapsize, s);
    WriteLn('myheapsize : ' + s);
    Str(myheapsize, s);
    WriteLn('Total : ' + s);
    WriteLn('Before fillchar');
    WriteLn('sbrk : $' + Hexstr(longint(heapstart+myheapsize), 8));        
    Sbrk:=heapstart+myheapsize;
    FillChar(sbrk^, size, #0);    
    WriteLn('EndFillChar');
    WriteLn('sbrk : $' + Hexstr(longint(sbrk), 8));
//    ReadLn(s);
    myheapsize:=newsize;
    Str({longint(heapstartpointer) +} myheapsize, s);
    WriteLn('Total : ' + s);    
    myheaprealsize:=newrealsize;
    exit;
  end
  else
  begin
    debugger('Bad resize_area');
    WriteLn('Bad resize_area');
  end;
  Sbrk:=nil;
*)
//end;

{ $I text.inc}

{*****************************************************************************
                           UnTyped File Handling
*****************************************************************************}


{ $i file.inc}

{*****************************************************************************
                           Typed File Handling
*****************************************************************************}

{ $i typefile.inc}

{*****************************************************************************
                       Misc. System Dependent Functions
*****************************************************************************}

Function ParamCount: Longint;
var
  s : string;
Begin
  ParamCount := 0;
  Paramcount:=argc - 1;
End;

 { variable where full path and filename and executable is stored }
 { is setup by the startup of the system unit.                    }
var
 execpathstr : shortstring;

{$ifdef FPC_USE_LIBC}

// private; use the macros, below
function _get_image_info(image : image_id; var info : image_info; size : size_t)
         : status_t; cdecl; external 'root' name '_get_image_info';

function _get_next_image_info(team : team_id; var cookie : Longint; var info : image_info; size : size_t)
         : status_t; cdecl; external 'root' name '_get_next_image_info';

function get_image_info(image : image_id; var info : image_info) : status_t;
begin
  Result := _get_image_info(image, info, SizeOf(info));
end;

function get_next_image_info(team : team_id; var cookie : Longint; var info : image_info) : status_t;
begin
  Result := _get_next_image_info(team, cookie, info, SizeOf(info));
end;

{$endif}

{ this routine sets up the paramstr(0) string at startup }
procedure setupexecname;
var
 cookie: longint;
 image : image_info;
 index : byte;
 s : string;
begin
  cookie:=0;
  fillchar(image, sizeof(image_info), 0);
  if get_next_image_info(0, cookie, image) = B_OK then
  begin
    execpathstr := strpas(@image.name);
  end
  else
    execpathstr := '';
  { problem with Be 4.5 noted... path contains . character }
  { if file is directly executed in CWD                    }
  index:=pos('/./',execpathstr);
  if index <> 0 then
    begin
      { remove the /. characters }
      Delete(execpathstr,index, 2);
    end;
end;

function paramstr(l: longint) : string;
var
  s: string;
  s1: string;
begin
   
  { stricly conforming POSIX applications  }
  { have the executing filename as argv[0] }
  if l = 0 then
  begin
    paramstr := execpathstr;
  end
  else if (l < argc) then
  begin
    paramstr:=strpas(argv[l]);
  end
  else
    paramstr := '';
end;

Procedure Randomize;
Begin
  randseed:=longint(Fptime(nil));
End;

function GetProcessID: SizeUInt;
begin
  GetProcessID := SizeUInt (fpGetPID);
end;

{*****************************************************************************
                         SystemUnit Initialization
*****************************************************************************}

function  reenable_signal(sig : longint) : boolean;
var
  e : TSigSet;
  i,j : byte;
  olderrno: cint;
begin
  fillchar(e,sizeof(e),#0);
  { set is 1 based PM }
  dec(sig);
  i:=sig mod (sizeof(cuLong) * 8);
  j:=sig div (sizeof(cuLong) * 8);
  e[j]:=1 shl i;
  { this routine is called from a signal handler, so must not change errno }
  olderrno:=geterrno;
  fpsigprocmask(SIG_UNBLOCK,@e,nil);
  reenable_signal:=geterrno=0;
  seterrno(olderrno);
end;

// signal handler is arch dependant due to processorexception to language
// exception translation

{$i sighnd.inc}

procedure InstallDefaultSignalHandler(signum: longint; out oldact: SigActionRec); public name '_FPC_INSTALLDEFAULTSIGHANDLER';
var
  act: SigActionRec;
begin
  { Initialize the sigaction structure }
  { all flags and information set to zero }
  FillChar(act, sizeof(SigActionRec),0);
  { initialize handler                    }
  act.sa_handler := SigActionHandler(@SignalToRunError);
  act.sa_flags:=SA_SIGINFO;
  FpSigAction(signum,@act,@oldact);
end;

var
  oldsigfpe: SigActionRec; public name '_FPC_OLDSIGFPE';
  oldsigsegv: SigActionRec; public name '_FPC_OLDSIGSEGV';
  oldsigbus: SigActionRec; public name '_FPC_OLDSIGBUS';
  oldsigill: SigActionRec; public name '_FPC_OLDSIGILL';

Procedure InstallSignals;
begin
  InstallDefaultSignalHandler(SIGFPE,oldsigfpe);
  InstallDefaultSignalHandler(SIGSEGV,oldsigsegv);
  InstallDefaultSignalHandler(SIGBUS,oldsigbus);
  InstallDefaultSignalHandler(SIGILL,oldsigill);
end;

Procedure RestoreOldSignalHandlers;
begin
  FpSigAction(SIGFPE,@oldsigfpe,nil);
  FpSigAction(SIGSEGV,@oldsigsegv,nil);
  FpSigAction(SIGBUS,@oldsigbus,nil);
  FpSigAction(SIGILL,@oldsigill,nil);
end;


procedure SysInitStdIO;
begin
  { Setup stdin, stdout and stderr, for GUI apps redirect stderr,stdout to be
    displayed in and messagebox }
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

var
  s : string;
begin
  IsConsole := TRUE;
  StackLength := CheckInitialStkLen(InitialStkLen);
  StackBottom := Sptr - StackLength;

  { Set up signals handlers (may be needed by init code to test cpu features) }
  InstallSignals;

{$ifdef cpui386}
  fpc_cpucodeinit;
{$endif}

  { Setup heap }
  myheapsize:=4096*1;// $ 20000;
  myheaprealsize:=4096*1;// $ 20000;
  heapstart:=nil;
  heapstartpointer := nil;
  heapstartpointer := Sbrk2(4096*1);
{$IFDEF FPC_USE_LIBC}  
//  heap_handle := create_area('fpcheap',heapstart,0,myheaprealsize,0,3);//!!
{$ELSE}
//  debugger('tata'#0);
//  heap_handle := create_area('fpcheap',longint(heapstartpointer),0,myheaprealsize,0,3);//!!
//  case heap_handle of
//    B_BAD_VALUE : WriteLn('B_BAD_VALUE');
//    B_PAGE_SIZE : WriteLn('B_PAGE_SIZE');
//    B_NO_MEMORY : WriteLn('B_NO_MEMORY');
//    B_ERROR : WriteLn('B_ERROR');
//  end;

  FillChar(heapstartpointer^, myheaprealsize, #0);
//  WriteLn('EndFillChar');
//    WriteLn('P : $' + Hexstr(longint(heapstartpointer), 8));        
//    WriteLn('heapstart : $' + Hexstr(longint(heapstartpointer^), 8));        
  heapstart := heapstartpointer;
{$ENDIF}
//  WriteLn('before InitHeap');
//  case heap_handle of
//    B_BAD_VALUE : WriteLn('B_BAD_VALUE');
//    B_PAGE_SIZE : WriteLn('B_PAGE_SIZE');
//    B_NO_MEMORY : WriteLn('B_NO_MEMORY');
//    B_ERROR : WriteLn('B_ERROR');
//  else
//    begin
//      WriteLn('ok');  
//      WriteLn('P : $' + Hexstr(longint(heapstartpointer), 8));        
//      WriteLn('heapstart : $' + Hexstr(longint(heapstartpointer^), 8));       
//      if heap_handle>0 then 
//      begin
        InitHeap;
//      end;
//    end;
//  end;
//  WriteLn('after InitHeap');
//  end else system_exit;
  SysInitExceptions;
//  WriteLn('after SysInitException');

  initunicodestringmanager;
{ Setup IO }
  SysInitStdIO;
{ Reset IO Error }
  InOutRes:=0;
  InitSystemThreads;
  InitSystemDynLibs;
  setupexecname;
  { restore original signal handlers in case this is a library }
  if IsLibrary then
    RestoreOldSignalHandlers;
end.
