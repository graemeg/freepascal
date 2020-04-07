{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 Karoly Balogh, Free Pascal Development team

    Headers for Amiga legacy (OS 1.x/2.x) support functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{* exec.library *}

function AllocVec(byteSize    : Cardinal;
                  requirements: Cardinal): Pointer; external name '_fpc_amiga_allocvec';
procedure FreeVec(memoryBlock: Pointer); external name '_fpc_amiga_freevec';
function CreatePool(requirements: Cardinal;
                    puddleSize  : Cardinal;
                    threshSize  : Cardinal): Pointer; external name '_fpc_amiga_createpool';
function AllocPooled(poolHeader: Pointer;
                     memSize   : Cardinal): Pointer; external name '_fpc_amiga_allocpooled';
procedure FreePooled(poolHeader: Pointer;
                    memory    : Pointer;
                    memSize   : Cardinal); external name '_fpc_amiga_freepooled';
procedure DeletePool(poolHeader: Pointer); external name '_fpc_amiga_deletepool';

procedure StackSwap(newStack: PStackSwapStruct); external name '_fpc_amiga_stackswap';
procedure ObtainSemaphoreShared(sigSem: PSignalSemaphore); external name '_fpc_amiga_obtainsemaphoreshared';
