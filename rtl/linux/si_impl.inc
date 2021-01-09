{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2015 by Sven Barth, member of the Free Pascal development
    team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

procedure PascalMain; external name 'PASCALMAIN';

{$ifdef FPC_HAS_INDIRECT_ENTRY_INFORMATION}
procedure SysEntry(constref info: TEntryInformation); external name 'FPC_SysEntry';
{$ifndef FPC_USE_LIBC}
procedure SysEntry_InitTLS(constref info: TEntryInformation); external name 'FPC_SysEntry_InitTLS';
{$endif FPC_USE_LIBC}

var
  InitFinalTable : record end; external name 'INITFINAL';
  ThreadvarTablesTable : record end; external name 'FPC_THREADVARTABLES';
  {$ifdef FPC_HAS_RESSTRINITS}
  ResStrInitTables : record end; external name 'FPC_RESSTRINITTABLES';
  {$endif}
  ResourceStringTables : record end; external name 'FPC_RESOURCESTRINGTABLES';
  ResLocation : Pointer; external name 'FPC_RESLOCATION';
  StackLength : SizeUInt; external name '__stklen';

const
  SysInitEntryInformation : TEntryInformation = (
    InitFinalTable : @InitFinalTable;
{$ifdef FPC_SECTION_THREADVARS}
    ThreadvarTablesTable : nil;
{$else FPC_SECTION_THREADVARS}
    ThreadvarTablesTable : @ThreadvarTablesTable;
{$endif FPC_SECTION_THREADVARS}
    ResourceStringTables : @ResourceStringTables;
{$ifdef FPC_HAS_RESSTRINITS}
    ResStrInitTables : @ResStrInitTables;
{$else}
    ResStrInitTables : nil;
{$endif}
    ResLocation : @ResLocation;
    PascalMain : @PascalMain;
    valgrind_used : false;
    OS: (
        argc: 0;
        argv: nil;
        envp: nil;
        stkptr: nil;
        stklen: 0;
        haltproc: nil;
      );
    );
{$endif FPC_HAS_INDIRECT_ENTRY_INFORMATION}

