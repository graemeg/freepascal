{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Karoly Balogh

    exec.library functions for AmigaOS 4.x/PowerPC

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function execAllocMem(byteSize: longword; attributes: longword): Pointer; syscall IExec 104;
function AllocPooled(poolHeader: Pointer;memSize: longword): Pointer; syscall IExec 108;
function CreatePool(memFlags: longword; puddleSize: longword; threshSize: longword): Pointer; syscall IExec 132;

procedure DeletePool(poolHeader: Pointer); syscall IExec 140;

procedure Forbid; syscall IExec 156;

procedure execFreeMem(memoryBlock: Pointer; byteSize: longword); syscall IExec 164;
procedure FreePooled(poolHeader: Pointer;memory: Pointer;memSize: longword); syscall IExec 168;

procedure Permit; syscall IExec 200;

function FindTask(name: PChar): PTask; syscall IExec 260;

function AllocSignal(signalNum: shortint): shortint; syscall IExec 276;
procedure FreeSignal(signalNum: shortint); syscall IExec 280;

function SetSignal(newSignals: longword; signalMask: longword): longword; syscall IExec 288;

procedure AddPort(port: PMsgPort); syscall IExec 300;

function CreateMsgPort(): PMsgPort; syscall IExec 308;

procedure DeleteMsgPort(Port: PMsgPort); syscall IExec 316;

function GetMsg(port: PMsgPort): PMessage; syscall IExec 324;
procedure PutMsg(port: PMsgPort; message: PMessage); syscall IExec 328;
procedure RemPort(port: PMsgPort); syscall IExec 332;
procedure ReplyMsg(message: PMessage); syscall IExec 336;
function WaitPort(port: PMsgPort): PMessage; syscall IExec 340;

function AttemptSemaphore(sigSem: PSignalSemaphore): Cardinal; syscall IExec 352;

procedure InitSemaphore(sigSem: PSignalSemaphore); syscall IExec 364;
procedure ObtainSemaphore(sigSem: PSignalSemaphore); syscall IExec 368;
procedure ObtainSemaphoreShared(sigSem: PSignalSemaphore); syscall IExec 376;

procedure ReleaseSemaphore(sigSem: PSignalSemaphore); syscall IExec 384;

function OpenLibrary(name: PChar; version: longword): PLibrary; syscall IExec 424;
function CloseLibrary(_library: PLibrary): Pointer; syscall IExec 428;

function GetInterface(_library: PLibrary;name: PChar;version: longword;taglist: PTagItem): POS4Interface; syscall IExec 448;
function GetInterfaceTags(_library: PLibrary;name: PChar;version: longword): POS4Interface; {varargs;} syscall IExec 452; {$WARNING varargs should be possible with syscalls too on OS4}
procedure DropInterface(_interface: POS4Interface); syscall IExec 456;

function OpenDevice(devName: PChar; unitNumber: longword;ioRequest: PIORequest; flags: longword): longint; syscall IExec 504;
function CloseDevice(ioRequest: PIORequest): Pointer; syscall IExec 508;
function CreateIORequest(const IOReplyPort: PMsgPort; Size: LongWord): PIORequest; syscall IExec 512;
procedure DeleteIORequest(IORequest: PIORequest); syscall IExec 516;

function DoIO(ioRequest: PIORequest): shortint; syscall IExec 528;

procedure RawPutChar(c: Char); syscall IExec 760;
