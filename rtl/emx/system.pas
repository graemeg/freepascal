{
 ****************************************************************************

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2002 by Free Pascal development team

    Free Pascal - EMX runtime library

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

****************************************************************************}

unit System;

interface

{Link the startup code.}
{$l prt1.o}

{$define FPC_SYSTEM_HAS_SYSDLH}

{$I systemh.inc}

const
(* Are file sizes > 2 GB (64-bit) supported on the current system? *)
  FSApi64: boolean = false;
(* Is full Unicode support provided by the underlying OS/2 version available *)
(* and successfully initialized (otherwise dummy routines need to be used).  *)
  UniAPI: boolean = false;
  DosCallsHandle: THandle = THandle (-1);

 LineEnding = #13#10;
{ LFNSupport is defined separately below!!! }
 DirectorySeparator = '\';
 DriveSeparator = ':';
 ExtensionSeparator = '.';
 PathSeparator = ';';
 AllowDirectorySeparators : set of char = ['\','/'];
 AllowDriveSeparators : set of char = [':'];
{ FileNameCaseSensitive and FileNameCasePreserving are defined separately below!!! }
 maxExitCode = 255;
 MaxPathLen = 256;
 AllFilesMask = '*';

type    Tos=(osDOS,osOS2,osDPMI);

var     os_mode:Tos;
        first_meg:pointer;

type    TByteArray = array [0..$ffff] of byte;
        PByteArray = ^TByteArray;

        TSysThreadIB = record
            TID,
            Priority,
            Version: cardinal;
            MCCount,
            MCForceFlag: word;
        end;
        PSysThreadIB = ^TSysThreadIB;

        TThreadInfoBlock = record
            PExChain,
            Stack,
            StackLimit: pointer;
            TIB2: PSysThreadIB;
            Version,
            Ordinal: cardinal;
        end;
        PThreadInfoBlock = ^TThreadInfoBlock;
        PPThreadInfoBlock = ^PThreadInfoBlock;

        TProcessInfoBlock = record
            PID,
            ParentPid,
            Handle: cardinal;
            Cmd,
            Env: PByteArray;
            Status,
            ProcType: cardinal;
        end;
        PProcessInfoBlock = ^TProcessInfoBlock;
        PPProcessInfoBlock = ^PProcessInfoBlock;

const   UnusedHandle=-1;
        StdInputHandle=0;
        StdOutputHandle=1;
        StdErrorHandle=2;

        LFNSupport: boolean = true;
        FileNameCaseSensitive: boolean = false;
        FileNameCasePreserving: boolean = false;
        CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

        sLineBreak = LineEnding;
        DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

var
{ C-compatible arguments and environment }
  argc  : longint;external name '_argc';
  argv  : ppchar;external name '_argv';
  envp  : ppchar;external name '_environ';
  EnvC: cardinal; external name '_envc';

(* Pointer to the block of environment variables - used e.g. in unit Dos. *)
  Environment: PChar;

var
(* Type / run mode of the current process: *)
(* 0 .. full screen OS/2 session           *)
(* 1 .. DOS session                        *)
(* 2 .. VIO windowable OS/2 session        *)
(* 3 .. Presentation Manager OS/2 session  *)
(* 4 .. detached (background) OS/2 process *)
  ApplicationType: cardinal;


procedure SetDefaultOS2FileType (FType: ShortString);

procedure SetDefaultOS2Creator (Creator: ShortString);

(* Support for tracking I/O errors returned by OS/2 API calls - emulation *)
(* of GetLastError / fpGetError functionality used e.g. in Sysutils.      *)
type
  TOSErrorWatch = procedure (Error: cardinal);

procedure NoErrorTracking (Error: cardinal);

(* This shall be invoked whenever a non-zero error is returned by OS/2 APIs *)
(* used in the RTL. Direct OS/2 API calls in user programs are not covered! *)
const
  OSErrorWatch: TOSErrorWatch = @NoErrorTracking;

type
  TDosOpenL = function (FileName: PChar; var Handle: THandle;
                        var Action: cardinal; InitSize: int64;
                        Attrib, OpenFlags, FileMode: cardinal;
                                                 EA: pointer): cardinal; cdecl;

  TDosSetFilePtrL = function (Handle: THandle; Pos: int64; Method: cardinal;
                                        var PosActual: int64): cardinal; cdecl;

  TDosSetFileSizeL = function (Handle: THandle; Size: int64): cardinal; cdecl;


var
  Sys_DosOpenL: TDosOpenL;
  Sys_DosSetFilePtrL: TDosSetFilePtrL;
  Sys_DosSetFileSizeL: TDosSetFileSizeL;


implementation

{ EMX cross-assembler is way too old to support 64bit opcodes }
{$define OLD_ASSEMBLER}

{$I system.inc}

var
    heap_base: pointer; external name '__heap_base';
    heap_brk: pointer; external name '__heap_brk';
    heap_end: pointer; external name '__heap_end';

(* Maximum heap size - only used if heap is allocated as continuous block. *)
{$IFDEF CONTHEAP}
    BrkLimit: cardinal;
{$ENDIF CONTHEAP}


{****************************************************************************

                    Miscellaneous related routines.

****************************************************************************}

{$asmmode intel}
procedure system_exit; assembler;
asm
    mov  ah, 04ch
    mov  al, byte ptr exitcode
    call syscall
end {['EAX']};

{$ASMMODE ATT}

function paramcount:longint;assembler;

asm
    movl argc,%eax
    decl %eax
end {['EAX']};

    function args:pointer;assembler;

    asm
        movl argv,%eax
end {['EAX']};


function paramstr(l:longint):string;

var p:^Pchar;

begin
    { There seems to be a problem with EMX for DOS when trying to }
    { access paramstr(0), and to avoid problems between DOS and   }
    { OS/2 they have been separated.                              }
    if os_Mode = OsOs2 then
    begin
    if L = 0 then
        begin
            GetMem (P, 260);
            p[0] := #0;  { in case of error, initialize to empty string }
{$ASMMODE INTEL}
            asm
                mov edx, P
                mov ecx, 260
                mov eax, 7F33h
                call syscall    { error handle already with empty string }
            end ['eax', 'ecx', 'edx'];
            ParamStr := StrPas (PChar (P));
            FreeMem (P, 260);
        end
    else
        if (l>0) and (l<=paramcount) then
            begin
                p:=args;
                paramstr:=strpas(p[l]);
            end
        else paramstr:='';
    end
   else
    begin
      p:=args;
      paramstr:=strpas(p[l]);
    end;
end;


procedure randomize; assembler;
asm
    mov ah, 2Ch
    call syscall
    mov word ptr [randseed], cx
    mov word ptr [randseed + 2], dx
end {['eax', 'ecx', 'edx']};

{$ASMMODE ATT}


{*****************************************************************************

                        System unit initialization.

****************************************************************************}

{****************************************************************************
                    Error Message writing using messageboxes
****************************************************************************}

type
  TWinMessageBox = function (Parent, Owner: cardinal;
         BoxText, BoxTitle: PChar; Identity, Style: cardinal): cardinal; cdecl;
  TWinInitialize = function (Options: cardinal): cardinal; cdecl;
  TWinCreateMsgQueue = function (Handle: cardinal; cmsg: longint): cardinal;
                                                                         cdecl;

const
  ErrorBufferLength = 1024;
  mb_OK = $0000;
  mb_Error = $0040;
  mb_Moveable = $4000;
  MBStyle = mb_OK or mb_Error or mb_Moveable;
  WinInitialize: TWinInitialize = nil;
  WinCreateMsgQueue: TWinCreateMsgQueue = nil;
  WinMessageBox: TWinMessageBox = nil;
  EnvSize: cardinal = 0;

var
  ErrorBuf: array [0..ErrorBufferLength] of char;
  ErrorLen: longint;
  PMWinHandle: cardinal;

function ErrorWrite (var F: TextRec): integer;
{
  An error message should always end with #13#10#13#10
}
var
  P: PChar;
  I: longint;
begin
  if F.BufPos > 0 then
   begin
     if F.BufPos + ErrorLen > ErrorBufferLength then
       I := ErrorBufferLength - ErrorLen
     else
       I := F.BufPos;
     Move (F.BufPtr^, ErrorBuf [ErrorLen], I);
     Inc (ErrorLen, I);
     ErrorBuf [ErrorLen] := #0;
   end;
  if ErrorLen > 3 then
   begin
     P := @ErrorBuf [ErrorLen];
     for I := 1 to 4 do
      begin
        Dec (P);
        if not (P^ in [#10, #13]) then
          break;
      end;
   end;
   if ErrorLen = ErrorBufferLength then
     I := 4;
   if (I = 4) then
    begin
      WinMessageBox (0, 0, @ErrorBuf, PChar ('Error'), 0, MBStyle);
      ErrorLen := 0;
    end;
  F.BufPos := 0;
  ErrorWrite := 0;
end;

function ErrorClose (var F: TextRec): integer;
begin
  if ErrorLen > 0 then
   begin
     WinMessageBox (0, 0, @ErrorBuf, PChar ('Error'), 0, MBStyle);
     ErrorLen := 0;
   end;
  ErrorLen := 0;
  ErrorClose := 0;
end;

function ErrorOpen (var F: TextRec): integer;
begin
  TextRec(F).InOutFunc := @ErrorWrite;
  TextRec(F).FlushFunc := @ErrorWrite;
  TextRec(F).CloseFunc := @ErrorClose;
  ErrorOpen := 0;
end;


procedure AssignError (var T: Text);
begin
  Assign (T, '');
  TextRec (T).OpenFunc := @ErrorOpen;
  Rewrite (T);
end;


procedure DosEnvInit;
var
 Q: PPChar;
 I: cardinal;
begin
(* It's a hack, in fact - DOS stores the environment the same way as OS/2 does,
   but I don't know how to find Program Segment Prefix and thus the environment
   address under EMX, so I'm recreating this structure using EnvP pointer. *)
{$ASMMODE INTEL}
 asm
  cld
  mov ecx, EnvC
  mov esi, EnvP
  xor eax, eax
  xor edx, edx
@L1:
  xchg eax, edx
  push ecx
  mov ecx, -1
  mov edi, [esi]
  repne
  scasb
  neg ecx
  dec ecx
  xchg eax, edx
  add eax, ecx
  pop ecx
  dec ecx
  jecxz @Stop
  inc esi
  inc esi
  inc esi
  inc esi
  jmp @L1
@Stop:
  inc eax
  mov EnvSize, eax
 end ['eax','ecx','edx','esi','edi'];
 Environment := GetMem (EnvSize);
 asm
  cld
  mov ecx, EnvC
  mov edx, EnvP
  mov edi, Environment
@L2:
  mov esi, [edx]
@Copying:
  lodsb
  stosb
  or al, al
  jnz @Copying
  dec ecx
  jecxz @Stop2
  inc edx
  inc edx
  inc edx
  inc edx
  jmp @L2
@Stop2:
  stosb
 end ['eax','ecx','edx','esi','edi'];
end;


procedure SysInitStdIO;
begin
  { Setup stdin, stdout and stderr, for GUI apps redirect stderr,stdout to be
    displayed in a messagebox }
(*
  StdInputHandle := longint(GetStdHandle(cardinal(STD_INPUT_HANDLE)));
  StdOutputHandle := longint(GetStdHandle(cardinal(STD_OUTPUT_HANDLE)));
  StdErrorHandle := longint(GetStdHandle(cardinal(STD_ERROR_HANDLE)));

  if not IsConsole then
    begin
      if (DosLoadModule (nil, 0, 'PMWIN', PMWinHandle) = 0) and
       (DosQueryProcAddr (PMWinHandle, 789, nil, pointer (WinMessageBox)) = 0)
                                                                           and
       (DosQueryProcAddr (PMWinHandle, 763, nil, pointer (WinInitialize)) = 0)
                                                                           and
       (DosQueryProcAddr (PMWinHandle, 716, nil, pointer (WinCreateMsgQueue))
                                                                           = 0)
        then
          begin
            WinInitialize (0);
            WinCreateMsgQueue (0, 0);
          end
        else
          HandleError (2);
     AssignError (StdErr);
     AssignError (StdOut);
     Assign (Output, '');
     Assign (Input, '');
   end
  else
   begin
*)
     OpenStdIO (Input, fmInput, StdInputHandle);
     OpenStdIO (Output, fmOutput, StdOutputHandle);
     OpenStdIO (ErrOutput, fmOutput, StdErrorHandle);
     OpenStdIO (StdOut, fmOutput, StdOutputHandle);
     OpenStdIO (StdErr, fmOutput, StdErrorHandle);
(*
   end;
*)
end;


threadvar
  DefaultCreator: ShortString;
  DefaultFileType: ShortString;


procedure SetDefaultOS2FileType (FType: ShortString);
begin
{$WARNING Not implemented yet!}
  DefaultFileType := FType;
end;


procedure SetDefaultOS2Creator (Creator: ShortString);
begin
{$WARNING Not implemented yet!}
  DefaultCreator := Creator;
end;

(* The default handler does not store the OS/2 API error codes. *)
procedure NoErrorTracking (Error: cardinal);
begin
end;

function GetFileHandleCount: longint;
var L1: longint;
    L2: cardinal;
begin
    L1 := 0; (* Don't change the amount, just check. *)
    if DosSetRelMaxFH (L1, L2) <> 0 then GetFileHandleCount := 50
                                                 else GetFileHandleCount := L2;
end;

function CheckInitialStkLen (StkLen: SizeUInt): SizeUInt;
begin
  CheckInitialStkLen := StkLen;
end;

var TIB: PThreadInfoBlock;
    PIB: PProcessInfoBlock;

const
 FatalHeap: array [0..33] of char = 'FATAL: Cannot initialize heap!!'#13#10'$';

begin
    {Determine the operating system we are running on.}
{$ASMMODE INTEL}
    asm
        push ebx
        mov os_mode, 0
        mov eax, 7F0Ah
        call syscall
        test bx, 512         {Bit 9 is OS/2 flag.}
        setne byte ptr os_mode
        test bx, 4096
        jz @noRSX
        mov os_mode, 2
    @noRSX:
    {Enable the brk area by initializing it with the initial heap size.}
        mov eax, 7F01h
        mov edx, heap_brk
        add edx, heap_base
        call syscall
        cmp eax, -1
        jnz @heapok
        lea edx, FatalHeap
        mov eax, 900h
        call syscall
        pop ebx
        push dword 204
        call HandleError
    @heapok:
{$IFDEF CONTHEAP}
{ Find out brk limit }
        mov eax, 7F02h
        mov ecx, 3
        call syscall
        jcxz @heaplimitknown
        mov eax, 0
    @heaplimitknown:
        mov BrkLimit, eax
{$ELSE CONTHEAP}
{ Change sbrk behaviour to allocate arbitrary (non-contiguous) memory blocks }
        mov eax, 7F0Fh
        mov ecx, 0Ch
        mov edx, 8
        call syscall
{$ENDIF CONTHEAP}
        pop ebx
    end ['eax', 'ecx', 'edx'];
    { in OS/2 this will always be nil, but in DOS mode }
    { this can be changed.                             }
    first_meg := nil;
    {Now request, if we are running under DOS,
     read-access to the first meg. of memory.}
    if os_mode in [osDOS,osDPMI] then
        asm
            push ebx
            mov eax, 7F13h
            xor ebx, ebx
            mov ecx, 0FFFh
            xor edx, edx
            call syscall
            jc @endmem
            mov first_meg, eax
         @endmem:
            pop ebx
        end ['eax', 'ecx', 'edx']
    else
        begin
    (* Initialize the amount of file handles *)
            FileHandleCount := GetFileHandleCount;
        end;
    {At 0.9.2, case for enumeration does not work.}
    case os_mode of
        osDOS:
            begin
                stackbottom:=pointer(heap_brk);     {In DOS mode, heap_brk is
                                                     also the stack bottom.}
                StackLength:=sptr-stackbottom;
{$WARNING To be checked/corrected!}
                ApplicationType := 1;   (* Running under DOS. *)
                IsConsole := true;
                asm
                    mov ax, 7F05h
                    call syscall
                    mov ProcessID, eax
                end ['eax'];
                ThreadID := 1;
            end;
        osOS2:
            begin
                DosGetInfoBlocks (@TIB, @PIB);
                StackLength:=CheckInitialStkLen(InitialStklen);
                { OS/2 has top of stack in TIB^.StackLimit - unlike Windows where it is in TIB^.Stack }
                StackBottom := TIB^.StackLimit - StackLength;

                Environment := pointer (PIB^.Env);
                ApplicationType := PIB^.ProcType;
                ProcessID := PIB^.PID;
                ThreadID := TIB^.TIB2^.TID;
                IsConsole := ApplicationType <> 3;
                FileNameCasePreserving := true;
            end;
        osDPMI:
            begin
                stackbottom:=nil;   {Not sure how to get it, but seems to be
                                     always zero.}
                StackLength:=sptr-stackbottom;
{$WARNING To be checked/corrected!}
                ApplicationType := 1;   (* Running under DOS. *)
                IsConsole := true;
                ThreadID := 1;
            end;
    end;
    exitproc:=nil;

    {Initialize the heap.}
    initheap;

    { ... and exceptions }
    SysInitExceptions;

{$ifdef HASWIDESTRING}
    InitUnicodeStringManager;
{$endif HASWIDESTRING}

    { ... and I/O }
    SysInitStdIO;

    { no I/O-Error }
    inoutres:=0;

    InitSystemThreads;

    InitSystemDynLibs;

    if os_Mode in [osDOS,osDPMI] then
        DosEnvInit;

{$IFDEF DUMPGROW}
 {$IFDEF CONTHEAP}
    WriteLn ('Initial brk size is ', GetHeapSize);
    WriteLn ('Brk limit is ', BrkLimit);
 {$ENDIF CONTHEAP}
{$ENDIF DUMPGROW}
end.
