{
    This file is part of the Free Pascal run time library.

    exec functions (V50) for MorphOS/PowerPC
    Copyright (c) 2002 The MorphOS Development Team, All Rights Reserved.

    Free Pascal conversion
    Copyright (c) 2004 Karoly Balogh for Genesi S.a.r.l. <www.genesi.lu>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function Supervisor(userFunction: Pointer location 'a5'): Cardinal;
SysCall MOS_ExecBase 030;

procedure InitCode(startClass: Cardinal location 'd0';
                   version   : Cardinal location 'd1');
SysCall MOS_ExecBase 072;

procedure InitStruct(initTable: Pointer  location 'a1';
                     memory   : Pointer  location 'a2';
                     size     : Cardinal location 'd0');
SysCall MOS_ExecBase 078;

function MakeLibrary(funcInit  : Pointer  location 'a0';
                     structInit: Pointer  location 'a1';
                     libInit   : Pointer  location 'a2';
                     dataSize  : Cardinal location 'd0';
                     segList   : Cardinal location 'd1'): PLibrary;
SysCall MOS_ExecBase 084;

procedure MakeFunctions(target       : Pointer location 'a0';
                        functionArray: Pointer location 'a1';
                        funcDispBase : Pointer location 'a2');
SysCall MOS_ExecBase 090;

function FindResident(name: PChar location 'a1'): PResident;
SysCall MOS_ExecBase 096;

function InitResident(resident: PResident location 'a1';
                      segList : Cardinal  location 'd1'): Pointer;
SysCall MOS_ExecBase 102;

procedure Alert(alertNum: Cardinal location 'd7');
SysCall MOS_ExecBase 108;

procedure Debug(flags: Cardinal location 'd0');
SysCall MOS_ExecBase 114;

procedure Disable;
SysCall MOS_ExecBase 120;

procedure Enable;
SysCall MOS_ExecBase 126;

procedure Forbid;
SysCall MOS_ExecBase 132;

procedure Permit;
SysCall MOS_ExecBase 138;

function SetSR(newSR: Cardinal location 'd0';
               mask : Cardinal location 'd1'): Cardinal;
SysCall MOS_ExecBase 144;

function SuperState : Pointer;
SysCall MOS_ExecBase 150;

procedure UserState(sysStack: Pointer location 'd0');
SysCall MOS_ExecBase 156;

function SetIntVector(intNumber: LongInt    location 'd0';
                      interrupt: PInterrupt location 'a1'): PInterrupt;
SysCall MOS_ExecBase 162;

procedure AddIntServer(intNumber: LongInt    location 'd0';
                       interrupt: PInterrupt location 'a1');
SysCall MOS_ExecBase 168;

procedure RemIntServer(intNumber: LongInt    location 'd0';
                       interrupt: PInterrupt location 'a1');
SysCall MOS_ExecBase 174;

procedure Cause(interrupt: PInterrupt location 'a1');
SysCall MOS_ExecBase 180;

function Allocate(freeList: PMemHeader location 'a0';
                  byteSize: Cardinal   location 'd0'): Pointer;
SysCall MOS_ExecBase 186;

procedure Deallocate(freeList   : PMemHeader location 'a0';
                     memoryBlock: Pointer    location 'a1';
                     byteSize   : Cardinal   location 'd0');
SysCall MOS_ExecBase 192;

{ * Name changed to avoid conflict with heap manager... * }
function execAllocMem(byteSize    : Cardinal location 'd0';
                      requirements: Cardinal location 'd1'): Pointer;
SysCall MOS_ExecBase 198;

function AllocAbs(byteSize: Cardinal location 'd0';
                  location: Pointer  location 'a1'): Pointer;
SysCall MOS_ExecBase 204;

{ * Name changed to avoid conflict with heap manager... * }
procedure execFreeMem(memoryBlock: Pointer  location 'a1';
                      byteSize   : Cardinal location 'd0');
SysCall MOS_ExecBase 210;

function AvailMem(requirements: Cardinal location 'd1'): Cardinal;
SysCall MOS_ExecBase 216;

function AllocEntry(entry: PMemList location 'a0'): PMemList;
SysCall MOS_ExecBase 222;

procedure FreeEntry(entry: PMemList location 'a0');
SysCall MOS_ExecBase 228;

{ * Name changed to avoid conflict with other System unit call... * }
procedure execInsert(list: PList location 'a0';
                     node: PNode location 'a1';
                     pred: PNode location 'a2');
SysCall MOS_ExecBase 234;

procedure AddHead(list: PList location 'a0';
                  node: PNode location 'a1');
SysCall MOS_ExecBase 240;

procedure AddTail(list: PList location 'a0';
                  node: PNode location 'a1');
SysCall MOS_ExecBase 246;

procedure Remove(node: PNode location 'a1');
SysCall MOS_ExecBase 252;

function RemHead(list: PList location 'a0'): PNode;
SysCall MOS_ExecBase 258;

function RemTail(list: PList location 'a0'): PNode;
SysCall MOS_ExecBase 264;

procedure Enqueue(list: PList location 'a0';
                  node: PNode location 'a1');
SysCall MOS_ExecBase 270;

function FindName(list: PList location 'a0';
                  name: PChar location 'a1'): PNode;
SysCall MOS_ExecBase 276;

function AddTask(task   : PTask   location 'a1';
                 initPC : Pointer location 'a2';
                 finalPC: Pointer location 'a3'): Pointer;
SysCall MOS_ExecBase 282;

procedure RemTask(task: PTask location 'a1');
SysCall MOS_ExecBase 288;

function FindTask(name: PChar location 'a1'): PTask;
SysCall MOS_ExecBase 294;

function SetTaskPri(task    : PTask   location 'a1';
                    priority: LongInt location 'd0'): ShortInt;
SysCall MOS_ExecBase 300;

function SetSignal(newSignals: Cardinal location 'd0';
                   signalSet : Cardinal location 'd1'): Cardinal;
SysCall MOS_ExecBase 306;

function SetExcept(newSignals: Cardinal location 'd0';
                   signalSet : Cardinal location 'd1'): Cardinal;
SysCall MOS_ExecBase 312;

function Wait(signalSet: Cardinal location 'd0'): Cardinal;
SysCall MOS_ExecBase 318;

procedure Signal(task     : PTask    location 'a1';
                 signalSet: Cardinal location 'd0');
SysCall MOS_ExecBase 324;

function AllocSignal(signalNum: LongInt location 'd0'): ShortInt;
SysCall MOS_ExecBase 330;

procedure FreeSignal(signalNum: LongInt location 'd0');
SysCall MOS_ExecBase 336;

function AllocTrap(trapNum: LongInt location 'd0'): LongInt;
SysCall MOS_ExecBase 342;

procedure FreeTrap(trapNum: LongInt location 'd0');
SysCall MOS_ExecBase 348;

procedure AddPort(port: PMsgPort location 'a1');
SysCall MOS_ExecBase 354;

procedure RemPort(port: PMsgPort location 'a1');
SysCall MOS_ExecBase 360;

procedure PutMsg(port   : PMsgPort location 'a0';
                 message: PMessage location 'a1');
SysCall MOS_ExecBase 366;

function GetMsg(port: PMsgPort location 'a0'): PMessage;
SysCall MOS_ExecBase 372;

procedure ReplyMsg(message : PMessage location 'a1');
SysCall MOS_ExecBase 378;

function WaitPort(port: PMsgPort location 'a0'): PMessage;
SysCall MOS_ExecBase 384;

function FindPort(name: PChar location 'a1'): PMsgPort;
SysCall MOS_ExecBase 390;

procedure AddLibrary(libHandle: PLibrary location 'a1');
SysCall MOS_ExecBase 396;

procedure RemLibrary(libHandle: PLibrary location 'a1');
SysCall MOS_ExecBase 402;

function OldOpenLibrary(libName: PChar location 'a1'): PLibrary;
SysCall MOS_ExecBase 408;

procedure CloseLibrary(libHandle: PLibrary location 'a1');
SysCall MOS_ExecBase 414;

function SetFunction(libHandle  : PLibrary location 'a1';
                     funcOffset : LongInt  location 'a0';
                     newFunction: Pointer  location 'd0'): Pointer;
SysCall MOS_ExecBase 420;

procedure SumLibrary(libHandle: PLibrary location 'a1');
SysCall MOS_ExecBase 426;

procedure AddDevice(device: PDevice location 'a1');
SysCall MOS_ExecBase 432;

procedure RemDevice(device: PDevice location 'a1');
SysCall MOS_ExecBase 438;

function OpenDevice(devName  : PChar      location 'a0';
                    numunit  : Cardinal   location 'd0';
                    ioRequest: pIORequest location 'a1';
                    flags    : Cardinal   location 'd1'): ShortInt;
SysCall MOS_ExecBase 444;

procedure CloseDevice(ioRequest: PIORequest location 'a1');
SysCall MOS_ExecBase 450;

function DoIO(ioRequest: PIORequest location 'a1'): ShortInt;
SysCall MOS_ExecBase 456;

procedure SendIO(ioRequest: PIORequest location 'a1');
SysCall MOS_ExecBase 462;

function CheckIO(ioRequest: PIORequest location 'a1'): PIORequest;
SysCall MOS_ExecBase 468;

function WaitIO(ioRequest: PIORequest location 'a1'): ShortInt;
SysCall MOS_ExecBase 474;

procedure AbortIO(ioRequest: PIORequest location 'a1');
SysCall MOS_ExecBase 480;

procedure AddResource(resource: Pointer location 'a1');
SysCall MOS_ExecBase 486;

procedure RemResource(resource: Pointer location 'a1');
SysCall MOS_ExecBase 492;

function OpenResource(resName: PChar location 'a1'): Pointer;
SysCall MOS_ExecBase 498;

procedure RawIOInit;
SysCall MOS_ExecBase 504;

function RawMayGetChar: Char;
SysCall MOS_ExecBase 510;

procedure RawPutChar(d0arg: Char location 'd0');
SysCall MOS_ExecBase 516;

function RawDoFmt(formatString: PChar   location 'a0';
                  dataStream  : Pointer location 'a1';
                  putChProc   : Pointer location 'a2';
                  putChData   : Pointer location 'a3'): Pointer;
SysCall MOS_ExecBase 522;

function GetCC: Cardinal;
SysCall MOS_ExecBase 528;

function TypeOfMem(address: Pointer location 'a1'): Cardinal;
SysCall MOS_ExecBase 534;

function Procure(sigSem: PSignalSemaphore  location 'a0';
                 bidMsg: PSemaphoreMessage location 'a1'): Cardinal;
SysCall MOS_ExecBase 540;

procedure Vacate(sigSem: PSignalSemaphore  location 'a0';
                 bidMsg: PSemaphoreMessage location 'a1');
SysCall MOS_ExecBase 546;

function OpenLibrary(libname: PChar    location 'a1';
                     libver : Cardinal location 'd0'): Pointer;
SysCall MOS_ExecBase 552;

procedure InitSemaphore(sigSem: PSignalSemaphore location 'a0');
SysCall MOS_ExecBase 558;

procedure ObtainSemaphore(sigSem: PSignalSemaphore location 'a0');
SysCall MOS_ExecBase 564;

procedure ReleaseSemaphore(sigSem: PSignalSemaphore location 'a0');
SysCall MOS_ExecBase 570;

function AttemptSemaphore(sigSem: PSignalSemaphore location 'a0'): Cardinal;
SysCall MOS_ExecBase 576;

procedure ObtainSemaphoreList(sigSem: PList location 'a0');
SysCall MOS_ExecBase 582;

procedure ReleaseSemaphoreList(sigSem: PList location 'a0');
SysCall MOS_ExecBase 588;

function FindSemaphore(sigSem: PChar location 'a1'): PSignalSemaphore;
SysCall MOS_ExecBase 594;

procedure AddSemaphore(sigSem: PSignalSemaphore location 'a1');
SysCall MOS_ExecBase 600;

procedure RemSemaphore(sigSem: PSignalSemaphore location 'a1');
SysCall MOS_ExecBase 606;

function SumKickData: Cardinal;
SysCall MOS_ExecBase 612;

procedure AddMemList(size      : Cardinal location 'd0';
                     attributes: Cardinal location 'd1';
                     pri       : LongInt  location 'd2';
                     base      : Pointer  location 'a0';
                     name      : PChar    location 'a1');
SysCall MOS_ExecBase 618;

procedure CopyMem(source: Pointer  location 'a0';
                  dest  : Pointer  location 'a1';
                  size  : Cardinal location 'd0');
SysCall MOS_ExecBase 624;

procedure CopyMemQuick(source: Pointer  location 'a0';
                       dest  : Pointer  location 'a1';
                       size  : Cardinal location 'd0');
SysCall MOS_ExecBase 630;

procedure CacheClearU;
SysCall MOS_ExecBase 636;

procedure CacheClearE(address: Pointer  location 'a0';
                      length : Cardinal location 'd0';
                      caches : Cardinal location 'd1');
SysCall MOS_ExecBase 642;

function CacheControl(cacheBits: Cardinal location 'd0';
                      cacheMask: Cardinal location 'd1'): Cardinal;
SysCall MOS_ExecBase 648;

function CreateIORequest(port: PMsgPort location 'a0';
                         size: Cardinal location 'd0'): Pointer;
SysCall MOS_ExecBase 654;

procedure DeleteIORequest(iorequest: Pointer location 'a0');
SysCall MOS_ExecBase 660;

function CreateMsgPort: PMsgPort;
SysCall MOS_ExecBase 666;

procedure DeleteMsgPort(port: PMsgPort location 'a0');
SysCall MOS_ExecBase 672;

procedure ObtainSemaphoreShared(sigSem: PSignalSemaphore location 'a0');
SysCall MOS_ExecBase 678;

function AllocVec(byteSize    : Cardinal location 'd0';
                  requirements: Cardinal location 'd1'): Pointer;
SysCall MOS_ExecBase 684;

procedure FreeVec(memoryBlock: Pointer location 'a1');
SysCall MOS_ExecBase 690;

function CreatePool(requirements: Cardinal location 'd0';
                    puddleSize  : Cardinal location 'd1';
                    threshSize  : Cardinal location 'd2'): Pointer;
SysCall MOS_ExecBase 696;

procedure DeletePool(poolHeader: Pointer location 'a0');
SysCall MOS_ExecBase 702;

function AllocPooled(poolHeader: Pointer  location 'a0';
                     memSize   : Cardinal location 'd0'): Pointer;
SysCall MOS_ExecBase 708;

procedure FreePooled(poolHeader: Pointer  location 'a0';
                     memory    : Pointer  location 'a1';
                     memSize   : Cardinal location 'd0');
SysCall MOS_ExecBase 714;

function AttemptSemaphoreShared(sigSem: pSignalSemaphore location 'a0'): Cardinal;
SysCall MOS_ExecBase 720;

procedure ColdReboot;
SysCall MOS_ExecBase 726;

procedure StackSwap(newStack: PStackSwapStruct location 'a0');
SysCall MOS_ExecBase 732;

function NewGetTaskAttrsA(Task    : PTask    location 'a0';
                          Data    : Pointer  location 'a1';
                          DataSize: Cardinal location 'd0';
                          TType   : Cardinal location 'd1';
                          Tags    : PTagItem location 'a2'): Cardinal;
SysCall MOS_ExecBase 738;

function NewSetTaskAttrsA(Task    : PTask    location 'a0';
                          Data    : Pointer  location 'a1';
                          DataSize: Cardinal location 'd0';
                          TType   : Cardinal location 'd1';
                          Tags    : PTagItem location 'a2'): Cardinal;
SysCall MOS_ExecBase 744;

function CachePreDMA(address   : Pointer  location 'a0';
                     var length: Cardinal location 'a1';
                     flags     : Cardinal location 'd0'): Pointer;
SysCall MOS_ExecBase 762;

procedure CachePostDMA(address   : Pointer  location 'a0';
                       var length: Cardinal location 'a1';
                       flags     : Cardinal location 'd0');
SysCall MOS_ExecBase 768;

procedure AddMemHandler(memhand: PInterrupt location 'a1');
SysCall MOS_ExecBase 774;

procedure RemMemHandler(memhand: PInterrupt location 'a1');
SysCall MOS_ExecBase 780;

function ObtainQuickVector(interruptCode: Pointer location 'a0'): Cardinal;
SysCall MOS_ExecBase 786;

function NewSetFunction(libHandle  : PLibrary location 'a0';
                        newfunction: Pointer  location 'a1';
                        offset     : LongInt  location 'd0';
                        tags       : PTagItem location 'a2'): Pointer;
SysCall MOS_ExecBase 792;

function NewCreateLibrary(tags: PTagItem location 'a0'): PLibrary;
SysCall MOS_ExecBase 798;

function NewPPCStackSwap(newStack : PStackSwapStruct  location 'a0';
                         sfunction: Pointer           location 'a1';
                         args     : PPPCStackSwapArgs location 'a2'): Cardinal;
SysCall MOS_ExecBase 804;

function TaggedOpenLibrary(d0arg: LongInt location 'd0'): Pointer;
SysCall MOS_ExecBase 810;

function ReadGayle: Cardinal;
SysCall MOS_ExecBase 816;

function VNewRawDoFmt(FmtString: PChar;
                      PutChProc: Pointer;
                      PutChData: PChar;
                      args     : PChar): PChar;
SysCall BaseSysV MOS_ExecBase 822;

procedure CacheFlushDataArea(Address: Pointer  location 'a0';
                             Size   : Cardinal location 'd0');
SysCall MOS_ExecBase 828;

procedure CacheInvalidInstArea(Address: Pointer  location 'a0';
                               Size   : Cardinal location 'd0');
SysCall MOS_ExecBase 834;

procedure CacheInvalidDataArea(Address: Pointer  location 'a0';
                               Size   : Cardinal location 'd0');
SysCall MOS_ExecBase 840;

procedure CacheFlushDataInstArea(Address: Pointer  location 'a0';
                                 Size   : Cardinal location 'd0');
SysCall MOS_ExecBase 846;

procedure CacheTrashCacheArea(Address: Pointer  location 'a0';
                              Size   : Cardinal location 'd0');
SysCall MOS_ExecBase 852;

function AllocTaskPooled(Size: Cardinal location 'd0'): Pointer;
SysCall MOS_ExecBase 858;

procedure FreeTaskPooled(Address: Pointer  location 'a1';
                         Size   : Cardinal location 'd0');
SysCall MOS_ExecBase 864;

function AllocVecTaskPooled(Size: Cardinal location 'd0'): Pointer;
SysCall MOS_ExecBase 870;

procedure FreeVecTaskPooled(Address: Pointer location 'a1');
SysCall MOS_ExecBase 876;

procedure FlushPool(poolHeader: Pointer location 'a0');
SysCall MOS_ExecBase 882;

procedure FlushTaskPool;
SysCall MOS_ExecBase 888;

function AllocVecPooled(poolHeader: Pointer  location 'a0';
                        memSize   : Cardinal location 'd0'): Pointer;
SysCall MOS_ExecBase 894;

function NewGetSystemAttrsA(Data    : Pointer  location 'a0';
                            DataSize: Cardinal location 'd0';
                            TType   : Cardinal location 'd1';
                            Tags    : PTagItem location 'a1'): Cardinal;
SysCall MOS_ExecBase 906;

function NewSetSystemAttrsA(Data    : Pointer  location 'a0';
                            DataSize: Cardinal location 'd0';
                            TType   : Cardinal location 'd1';
                            Tags    : PTagItem location 'a1'): Cardinal;
SysCall MOS_ExecBase 912;

function NewCreateTaskA(Tags: PTagItem location 'a0'): PTask;
SysCall MOS_ExecBase 918;

function AllocateAligned(memHeader  : pMemHeader;
                         byteSize   : Cardinal;
                         alignSize  : Cardinal;
                         alignOffset: Cardinal): Pointer;
SysCall BaseSysV MOS_ExecBase 930;

function AllocMemAligned(byteSize   : Cardinal;
                         attributes : Cardinal;
                         alignSize  : Cardinal;
                         alignOffset: Cardinal): Pointer;
SysCall BaseSysV MOS_ExecBase 936;

function AllocVecAligned(byteSize   : Cardinal;
                         attributes : Cardinal;
                         alignSize  : Cardinal;
                         alignOffset: Cardinal): Pointer;
SysCall BaseSysV MOS_ExecBase 942;

procedure AddExecNotify(hook: PHook);
SysCall BaseSysV MOS_ExecBase 948;

procedure RemExecNotify(hook: PHook);
SysCall BaseSysV MOS_ExecBase 954;

function FindExecNode(ttype: Cardinal location 'd0';
                      name : PChar    location 'a0'): PNode;
SysCall MOS_ExecBase 960;

function AddExecNodeA(innode  : Pointer  location 'a0';
                      TagItems: PTagItem location 'a1'): Pointer;
SysCall MOS_ExecBase 966;

function AllocVecDMA(byteSize    : Cardinal location 'd0';
                     requirements: Cardinal location 'd1'): Pointer;
SysCall MOS_ExecBase 972;

procedure FreeVecDMA(memoryBlock: Pointer location 'a1');
SysCall MOS_ExecBase 978;

function AllocPooledAligned(poolHeader : Pointer;
                            byteSize   : Cardinal;
                            alignSize  : Cardinal;
                            alignOffset: Cardinal): Pointer;
SysCall BaseSysV MOS_ExecBase 984;

function AddResident(resident: pResident location 'd0'): LongInt;
SysCall BaseSysV MOS_ExecBase 990;

function FindTaskByPID(processID: Cardinal): PTask;
SysCall BaseSysV MOS_ExecBase 996;


