{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2014 Free Pascal Development team

    exec functions (V40) for Amiga/m68k

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function Supervisor(userFunction: Pointer location 'a5'): Cardinal;
SysCall AOS_ExecBase 030;

procedure InitCode(startClass: Cardinal location 'd0';
                   version   : Cardinal location 'd1');
SysCall AOS_ExecBase 072;

procedure InitStruct(initTable: Pointer  location 'a1';
                     memory   : Pointer  location 'a2';
                     size     : Cardinal location 'd0');
SysCall AOS_ExecBase 078;

function MakeLibrary(funcInit  : Pointer  location 'a0';
                     structInit: Pointer  location 'a1';
                     libInit   : Pointer  location 'a2';
                     dataSize  : Cardinal location 'd0';
                     segList   : Cardinal location 'd1'): PLibrary;
SysCall AOS_ExecBase 084;

procedure MakeFunctions(target       : Pointer location 'a0';
                        functionArray: Pointer location 'a1';
                        funcDispBase : Pointer location 'a2');
SysCall AOS_ExecBase 090;

function FindResident(name: PChar location 'a1'): PResident;
SysCall AOS_ExecBase 096;

function InitResident(resident: PResident location 'a1';
                      segList : Cardinal  location 'd1'): Pointer;
SysCall AOS_ExecBase 102;

procedure Alert(alertNum: Cardinal location 'd7');
SysCall AOS_ExecBase 108;

procedure Debug(flags: Cardinal location 'd0');
SysCall AOS_ExecBase 114;

procedure Disable;
SysCall AOS_ExecBase 120;

procedure Enable;
SysCall AOS_ExecBase 126;

procedure Forbid;
SysCall AOS_ExecBase 132;

procedure Permit;
SysCall AOS_ExecBase 138;

function SetSR(newSR: Cardinal location 'd0';
               mask : Cardinal location 'd1'): Cardinal;
SysCall AOS_ExecBase 144;

function SuperState : Pointer;
SysCall AOS_ExecBase 150;

procedure UserState(sysStack: Pointer location 'd0');
SysCall AOS_ExecBase 156;

function SetIntVector(intNumber: LongInt    location 'd0';
                      interrupt: PInterrupt location 'a1'): PInterrupt;
SysCall AOS_ExecBase 162;

procedure AddIntServer(intNumber: LongInt    location 'd0';
                       interrupt: PInterrupt location 'a1');
SysCall AOS_ExecBase 168;

procedure RemIntServer(intNumber: LongInt    location 'd0';
                       interrupt: PInterrupt location 'a1');
SysCall AOS_ExecBase 174;

procedure Cause(interrupt: PInterrupt location 'a1');
SysCall AOS_ExecBase 180;

function Allocate(freeList: PMemHeader location 'a0';
                  byteSize: Cardinal   location 'd0'): Pointer;
SysCall AOS_ExecBase 186;

procedure Deallocate(freeList   : PMemHeader location 'a0';
                     memoryBlock: Pointer    location 'a1';
                     byteSize   : Cardinal   location 'd0');
SysCall AOS_ExecBase 192;

{ * Name changed to avoid conflict with heap manager... * }
function execAllocMem(byteSize    : Cardinal location 'd0';
                      requirements: Cardinal location 'd1'): Pointer;
SysCall AOS_ExecBase 198;

function AllocAbs(byteSize: Cardinal location 'd0';
                  location: Pointer  location 'a1'): Pointer;
SysCall AOS_ExecBase 204;

{ * Name changed to avoid conflict with heap manager... * }
procedure execFreeMem(memoryBlock: Pointer  location 'a1';
                      byteSize   : Cardinal location 'd0');
SysCall AOS_ExecBase 210;

function AvailMem(requirements: Cardinal location 'd1'): Cardinal;
SysCall AOS_ExecBase 216;

function AllocEntry(entry: PMemList location 'a0'): PMemList;
SysCall AOS_ExecBase 222;

procedure FreeEntry(entry: PMemList location 'a0');
SysCall AOS_ExecBase 228;

{ * Name changed to avoid conflict with other System unit call... * }
procedure execInsert(list: PList location 'a0';
                     node: PNode location 'a1';
                     pred: PNode location 'a2');
SysCall AOS_ExecBase 234;

procedure AddHead(list: PList location 'a0';
                  node: PNode location 'a1');
SysCall AOS_ExecBase 240;

procedure AddTail(list: PList location 'a0';
                  node: PNode location 'a1');
SysCall AOS_ExecBase 246;

procedure Remove(node: PNode location 'a1');
SysCall AOS_ExecBase 252;

function RemHead(list: PList location 'a0'): PNode;
SysCall AOS_ExecBase 258;

function RemTail(list: PList location 'a0'): PNode;
SysCall AOS_ExecBase 264;

procedure Enqueue(list: PList location 'a0';
                  node: PNode location 'a1');
SysCall AOS_ExecBase 270;

function FindName(list: PList location 'a0';
                  name: PChar location 'a1'): PNode;
SysCall AOS_ExecBase 276;

function AddTask(task   : PTask   location 'a1';
                 initPC : Pointer location 'a2';
                 finalPC: Pointer location 'a3'): Pointer;
SysCall AOS_ExecBase 282;

procedure RemTask(task: PTask location 'a1');
SysCall AOS_ExecBase 288;

function FindTask(name: PChar location 'a1'): PTask;
SysCall AOS_ExecBase 294;

function SetTaskPri(task    : PTask   location 'a1';
                    priority: LongInt location 'd0'): ShortInt;
SysCall AOS_ExecBase 300;

function SetSignal(newSignals: Cardinal location 'd0';
                   signalSet : Cardinal location 'd1'): Cardinal;
SysCall AOS_ExecBase 306;

function SetExcept(newSignals: Cardinal location 'd0';
                   signalSet : Cardinal location 'd1'): Cardinal;
SysCall AOS_ExecBase 312;

function Wait(signalSet: Cardinal location 'd0'): Cardinal;
SysCall AOS_ExecBase 318;

procedure Signal(task     : PTask    location 'a1';
                 signalSet: Cardinal location 'd0');
SysCall AOS_ExecBase 324;

function AllocSignal(signalNum: LongInt location 'd0'): ShortInt;
SysCall AOS_ExecBase 330;

procedure FreeSignal(signalNum: LongInt location 'd0');
SysCall AOS_ExecBase 336;

function AllocTrap(trapNum: LongInt location 'd0'): LongInt;
SysCall AOS_ExecBase 342;

procedure FreeTrap(trapNum: LongInt location 'd0');
SysCall AOS_ExecBase 348;

procedure AddPort(port: PMsgPort location 'a1');
SysCall AOS_ExecBase 354;

procedure RemPort(port: PMsgPort location 'a1');
SysCall AOS_ExecBase 360;

procedure PutMsg(port   : PMsgPort location 'a0';
                 message: PMessage location 'a1');
SysCall AOS_ExecBase 366;

function GetMsg(port: PMsgPort location 'a0'): PMessage;
SysCall AOS_ExecBase 372;

procedure ReplyMsg(message : PMessage location 'a1');
SysCall AOS_ExecBase 378;

function WaitPort(port: PMsgPort location 'a0'): PMessage;
SysCall AOS_ExecBase 384;

function FindPort(name: PChar location 'a1'): PMsgPort;
SysCall AOS_ExecBase 390;

procedure AddLibrary(libHandle: PLibrary location 'a1');
SysCall AOS_ExecBase 396;

procedure RemLibrary(libHandle: PLibrary location 'a1');
SysCall AOS_ExecBase 402;

function OldOpenLibrary(libName: PChar location 'a1'): PLibrary;
SysCall AOS_ExecBase 408;

procedure CloseLibrary(libHandle: PLibrary location 'a1');
SysCall AOS_ExecBase 414;

function SetFunction(libHandle  : PLibrary location 'a1';
                     funcOffset : LongInt  location 'a0';
                     newFunction: Pointer  location 'd0'): Pointer;
SysCall AOS_ExecBase 420;

procedure SumLibrary(libHandle: PLibrary location 'a1');
SysCall AOS_ExecBase 426;

procedure AddDevice(device: PDevice location 'a1');
SysCall AOS_ExecBase 432;

procedure RemDevice(device: PDevice location 'a1');
SysCall AOS_ExecBase 438;

function OpenDevice(devName  : PChar      location 'a0';
                    numunit  : Cardinal   location 'd0';
                    ioRequest: pIORequest location 'a1';
                    flags    : Cardinal   location 'd1'): ShortInt;
SysCall AOS_ExecBase 444;

procedure CloseDevice(ioRequest: PIORequest location 'a1');
SysCall AOS_ExecBase 450;

function DoIO(ioRequest: PIORequest location 'a1'): ShortInt;
SysCall AOS_ExecBase 456;

procedure SendIO(ioRequest: PIORequest location 'a1');
SysCall AOS_ExecBase 462;

function CheckIO(ioRequest: PIORequest location 'a1'): PIORequest;
SysCall AOS_ExecBase 468;

function WaitIO(ioRequest: PIORequest location 'a1'): ShortInt;
SysCall AOS_ExecBase 474;

procedure AbortIO(ioRequest: PIORequest location 'a1');
SysCall AOS_ExecBase 480;

procedure AddResource(resource: Pointer location 'a1');
SysCall AOS_ExecBase 486;

procedure RemResource(resource: Pointer location 'a1');
SysCall AOS_ExecBase 492;

function OpenResource(resName: PChar location 'a1'): Pointer;
SysCall AOS_ExecBase 498;

procedure RawIOInit;
SysCall AOS_ExecBase 504;

function RawMayGetChar: Char;
SysCall AOS_ExecBase 510;

procedure RawPutChar(d0arg: Char location 'd0');
SysCall AOS_ExecBase 516;

function RawDoFmt(formatString: PChar   location 'a0';
                  dataStream  : Pointer location 'a1';
                  putChProc   : Pointer location 'a2';
                  putChData   : Pointer location 'a3'): Pointer;
SysCall AOS_ExecBase 522;

function GetCC: Cardinal;
SysCall AOS_ExecBase 528;

function TypeOfMem(address: Pointer location 'a1'): Cardinal;
SysCall AOS_ExecBase 534;

function Procure(sigSem: PSignalSemaphore  location 'a0';
                 bidMsg: PSemaphoreMessage location 'a1'): Cardinal;
SysCall AOS_ExecBase 540;

procedure Vacate(sigSem: PSignalSemaphore  location 'a0';
                 bidMsg: PSemaphoreMessage location 'a1');
SysCall AOS_ExecBase 546;

function OpenLibrary(libname: PChar    location 'a1';
                     libver : Cardinal location 'd0'): Pointer;
SysCall AOS_ExecBase 552;

// these functions are only available v33+ (OS Release 1.2+)
{$IFNDEF AMIGA_V1_0_ONLY}

procedure InitSemaphore(sigSem: PSignalSemaphore location 'a0');
SysCall AOS_ExecBase 558;

procedure ObtainSemaphore(sigSem: PSignalSemaphore location 'a0');
SysCall AOS_ExecBase 564;

procedure ReleaseSemaphore(sigSem: PSignalSemaphore location 'a0');
SysCall AOS_ExecBase 570;

function AttemptSemaphore(sigSem: PSignalSemaphore location 'a0'): Cardinal;
SysCall AOS_ExecBase 576;

procedure ObtainSemaphoreList(sigSem: PList location 'a0');
SysCall AOS_ExecBase 582;

procedure ReleaseSemaphoreList(sigSem: PList location 'a0');
SysCall AOS_ExecBase 588;

function FindSemaphore(sigSem: PChar location 'a1'): PSignalSemaphore;
SysCall AOS_ExecBase 594;

procedure AddSemaphore(sigSem: PSignalSemaphore location 'a1');
SysCall AOS_ExecBase 600;

procedure RemSemaphore(sigSem: PSignalSemaphore location 'a1');
SysCall AOS_ExecBase 606;

function SumKickData: Cardinal;
SysCall AOS_ExecBase 612;

procedure AddMemList(size      : Cardinal location 'd0';
                     attributes: Cardinal location 'd1';
                     pri       : LongInt  location 'd2';
                     base      : Pointer  location 'a0';
                     name      : PChar    location 'a1');
SysCall AOS_ExecBase 618;

procedure CopyMem(source: Pointer  location 'a0';
                  dest  : Pointer  location 'a1';
                  size  : Cardinal location 'd0');
SysCall AOS_ExecBase 624;

procedure CopyMemQuick(source: Pointer  location 'a0';
                       dest  : Pointer  location 'a1';
                       size  : Cardinal location 'd0');
SysCall AOS_ExecBase 630;

// these functions are only available v36+ (OS Release 2.0+)
{$IFNDEF AMIGA_V1_2_ONLY}

procedure CacheClearU;
SysCall AOS_ExecBase 636;

procedure CacheClearE(address: Pointer  location 'a0';
                      length : Cardinal location 'd0';
                      caches : Cardinal location 'd1');
SysCall AOS_ExecBase 642;

function CacheControl(cacheBits: Cardinal location 'd0';
                      cacheMask: Cardinal location 'd1'): Cardinal;
SysCall AOS_ExecBase 648;

function CreateIORequest(port: PMsgPort location 'a0';
                         size: Cardinal location 'd0'): Pointer;
SysCall AOS_ExecBase 654;

procedure DeleteIORequest(iorequest: Pointer location 'a0');
SysCall AOS_ExecBase 660;

function CreateMsgPort: PMsgPort;
SysCall AOS_ExecBase 666;

procedure DeleteMsgPort(port: PMsgPort location 'a0');
SysCall AOS_ExecBase 672;

procedure ObtainSemaphoreShared(sigSem: PSignalSemaphore location 'a0');
SysCall AOS_ExecBase 678;

function AllocVec(byteSize    : Cardinal location 'd0';
                  requirements: Cardinal location 'd1'): Pointer;
SysCall AOS_ExecBase 684;

procedure FreeVec(memoryBlock: Pointer location 'a1');
SysCall AOS_ExecBase 690;

function CreatePool(requirements: Cardinal location 'd0';
                    puddleSize  : Cardinal location 'd1';
                    threshSize  : Cardinal location 'd2'): Pointer;
SysCall AOS_ExecBase 696;

procedure DeletePool(poolHeader: Pointer location 'a0');
SysCall AOS_ExecBase 702;

function AllocPooled(poolHeader: Pointer  location 'a0';
                     memSize   : Cardinal location 'd0'): Pointer;
SysCall AOS_ExecBase 708;

procedure FreePooled(poolHeader: Pointer  location 'a0';
                      memory    : Pointer  location 'a1';
                      memSize   : Cardinal location 'd0');
SysCall AOS_ExecBase 714;

function AttemptSemaphoreShared(sigSem: pSignalSemaphore location 'a0'): Cardinal;
SysCall AOS_ExecBase 720;

procedure ColdReboot;
SysCall AOS_ExecBase 726;

procedure StackSwap(newStack: PStackSwapStruct location 'a0');
SysCall AOS_ExecBase 732;

function CachePreDMA(address   : Pointer  location 'a0';
                     var length: Cardinal location 'a1';
                     flags     : Cardinal location 'd0'): Pointer;
SysCall AOS_ExecBase 762;

procedure CachePostDMA(address   : Pointer  location 'a0';
                       var length: Cardinal location 'a1';
                       flags     : Cardinal location 'd0');
SysCall AOS_ExecBase 768;

procedure AddMemHandler(memhand: PInterrupt location 'a1');
SysCall AOS_ExecBase 774;

procedure RemMemHandler(memhand: PInterrupt location 'a1');
SysCall AOS_ExecBase 780;

function ObtainQuickVector(interruptCode: Pointer location 'a0'): Cardinal;
SysCall AOS_ExecBase 786;

{$ENDIF AMIGA_V1_2_ONLY}
{$ENDIF AMIGA_V1_0_ONLY}

{ TODO: v45 functions (OS3.9) }
