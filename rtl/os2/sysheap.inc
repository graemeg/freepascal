{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2001-2014 by Free Pascal development team

    This file implements heap management for OS/2.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{****************************************************************************

                    Heap management releated routines.

****************************************************************************}

{Get some memory.
 P          = Pointer to memory will be returned here.
 Size       = Number of bytes to get. The size is rounded up to a multiple
              of 4096. This is probably not the case on non-intel 386
              versions of OS/2.
 Flags      = One or more of the mfXXXX constants.}

function DosAllocMem(var P:pointer;Size,Flag:cardinal): cardinal; cdecl;
external 'DOSCALLS' index 299;

function DosSetMem(P:pointer;Size,Flag:cardinal): cardinal; cdecl;
external 'DOSCALLS' index 305;

function DosFreeMem (P: pointer): cardinal; cdecl;
external 'DOSCALLS' index 304;

{$IFDEF DUMPGROW}
 {$DEFINE EXTDUMPGROW}
{$ENDIF DUMPGROW}

{$IFDEF EXTDUMPGROW}
var
  Int_HeapSize: cardinal;
{$ENDIF EXTDUMPGROW}

{function GetHeapSize: longint; assembler;
asm
  movl Int_HeapSize, %eax
end ['EAX'];
}


function SysOSAlloc (Size: ptruint): pointer;
var
  P: pointer;
  RC: cardinal;
begin
{$IFDEF EXTDUMPGROW}
  if Int_HeapSize <> high (cardinal) then
{
  if Int_HeapSize = high (cardinal) then
   WriteLn ('Trying to allocate first heap of size ', Size)
  else
}
   WriteLn ('Trying to grow heap by ', Size, ' to ', Int_HeapSize);
{$ENDIF}

  RC := DosAllocMem (P, Size, HeapAllocFlags);
  if RC = 0 then
   begin
{$IFDEF EXTDUMPGROW}
    if Int_HeapSize <> high (cardinal) then
     WriteLn ('DosAllocMem returned memory at ', cardinal (P));
{$ENDIF}
    SysOSAlloc := P;
{$IFDEF EXTDUMPGROW}
    if Int_HeapSize = high (cardinal) then
     Int_HeapSize := Size
    else
     Inc (Int_HeapSize, Size);
{$ENDIF EXTDUMPGROW}
   end
  else
   begin
    SysOSAlloc := nil;
    OSErrorWatch (RC);
{$IFDEF EXTDUMPGROW}
    if Int_HeapSize <> high (cardinal) then
     begin
      WriteLn ('Error ', RC, ' during additional memory allocation (DosAllocMem)!');
{     if Int_HeapSize = high (cardinal) then
       WriteLn ('No memory allocated yet!')
      else
}
      WriteLn ('Total allocated memory is ', Int_HeapSize);
     end;
{$ENDIF EXTDUMPGROW}
   end;
end;

{$define HAS_SYSOSFREE}

<<<<<<< HEAD
procedure SysOSFree (P: pointer; Size: ptruint);
=======
procedure SysOSFree (P: pointer; Size: PtrInt);
{$IFDEF EXTDUMPGROW}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  RC: cardinal;
{$ENDIF EXTDUMPGROW}
begin
{$IFDEF EXTDUMPGROW}
  WriteLn ('Trying to free memory!');
  WriteLn ('Total allocated memory is ', Int_HeapSize);
  Dec (Int_HeapSize, Size);
  RC :=
{$ENDIF EXTDUMPGROW}
<<<<<<< HEAD
<<<<<<< HEAD
  RC := DosFreeMem (P);
  if RC <> 0 then
   begin
    OSErrorWatch (RC);
{$IFDEF EXTDUMPGROW}
=======
=======
>>>>>>> origin/fixes_2_2
        DosFreeMem (P);
{$IFDEF EXTDUMPGROW}
  if RC <> 0 then
   begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    WriteLn ('Error ', RC, ' during memory deallocation (DosFreeMem)!');
    WriteLn ('Total allocated memory is ', Int_HeapSize);
{$ENDIF EXTDUMPGROW}
   end;
end;


function ReadUseHighMem: boolean;
begin
 ReadUseHighMem := HeapAllocFlags and $400 = $400;
end;


procedure WriteUseHighMem (B: boolean);
begin
 if B then
  HeapAllocFlags := HeapAllocFlags or $400
 else
  HeapAllocFlags := HeapAllocFlags and not ($400);
end;
