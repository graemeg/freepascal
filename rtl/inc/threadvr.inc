{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt
    member of the Free Pascal development team

    Threadvar support, platform independent part

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{*****************************************************************************
                           Threadvar support
*****************************************************************************}


type
  pltvInitEntry = ^ltvInitEntry;
  ltvInitEntry = packed record
     varaddr : {$ifdef cpu16}pword{$else}pdword{$endif};
     size    : longint;
  end;

  TltvInitTablesTable = packed record
    count  : dword;
    tables : packed array [1..{$ifdef cpu16}16{$else}32767{$endif}] of pltvInitEntry;
  end;
  PltvInitTablesTable = ^TltvInitTablesTable;

{$ifndef FPC_HAS_INDIRECT_MAIN_INFORMATION}
var
  ThreadvarTablesTable : TltvInitTablesTable; external name 'FPC_THREADVARTABLES';
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}

procedure init_unit_threadvars (tableEntry : pltvInitEntry);
begin
  while tableEntry^.varaddr <> nil do
   begin
     CurrentTM.InitThreadvar (tableEntry^.varaddr^, tableEntry^.size);
     inc (pchar (tableEntry), sizeof (tableEntry^));
   end;
end;


procedure init_all_unit_threadvars;
var
  i : longint;
begin
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
  with PltvInitTablesTable(EntryInformation.ThreadvarTablesTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
  with ThreadvarTablesTable do
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
  begin
{$ifdef DEBUG_MT}
<<<<<<< HEAD
<<<<<<< HEAD
    WriteLn ('init_all_unit_threadvars (',count,') units');
{$endif}
    for i := 1 to count do
      init_unit_threadvars (tables[i]);
  end;
=======
  WriteLn ('init_all_unit_threadvars (',
    {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
      ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.count,') units');
{$endif}
=======
  WriteLn ('init_all_unit_threadvars (',
    {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
      ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.count,') units');
{$endif}
>>>>>>> origin/fixes_2_2
  for i := 1 to {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.count do
    init_unit_threadvars ({$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
      ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.tables[i]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


procedure copy_unit_threadvars (tableEntry : pltvInitEntry);
var
  oldp,
  newp : pointer;
begin
  while tableEntry^.varaddr <> nil do
   begin
     newp:=CurrentTM.RelocateThreadVar(tableEntry^.varaddr^);
     oldp:=pointer(pchar(tableEntry^.varaddr)+sizeof(pointer));
     move(oldp^,newp^,tableEntry^.size);
     inc (pchar (tableEntry), sizeof (tableEntry^));
   end;
end;


procedure copy_all_unit_threadvars;
var
  i: longint;
begin
{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}
  with PltvInitTablesTable(EntryInformation.ThreadvarTablesTable)^ do
{$else FPC_HAS_INDIRECT_MAIN_INFORMATION}
  with ThreadvarTablesTable do
{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
  begin
{$ifdef DEBUG_MT}
<<<<<<< HEAD
<<<<<<< HEAD
    WriteLn ('copy_all_unit_threadvars (',count,') units');
{$endif}
    for i := 1 to count do
      copy_unit_threadvars (tables[i]);
  end;
=======
  WriteLn ('copy_all_unit_threadvars (',{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.count,') units');
{$endif}
=======
  WriteLn ('copy_all_unit_threadvars (',{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.count,') units');
{$endif}
>>>>>>> origin/fixes_2_2
  for i := 1 to {$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
    ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.count do
    copy_unit_threadvars ({$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION}PltvInitTablesTable(EntryInformation.{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}
      ThreadvarTablesTable{$ifdef FPC_HAS_INDIRECT_MAIN_INFORMATION})^{$endif FPC_HAS_INDIRECT_MAIN_INFORMATION}.tables[i]);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure InitThreadVars(RelocProc : TRelocateThreadVarHandler);

begin
   { initialize threadvars }
   init_all_unit_threadvars;
   { allocate mem for main thread threadvars }
   CurrentTM.AllocateThreadVars;
   { copy main thread threadvars }
   copy_all_unit_threadvars;
   { install threadvar handler }
   fpc_threadvar_relocate_proc:=RelocProc;
{$ifdef FPC_HAS_FEATURE_HEAP}
{$ifndef HAS_MEMORYMANAGER}
   RelocateHeap;
{$endif HAS_MEMORYMANAGER}
{$endif}
end;


