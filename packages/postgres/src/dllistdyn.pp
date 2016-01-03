{
  Call InitialiseDllist before using any of the calls, and call ReleaseDllist
  when finished.
}

unit DllistDyn;

{$mode objfpc}{$H+}

interface

uses
  dynlibs, SysUtils;

{$PACKRECORDS C}

{$IFDEF Unix}
  const
    pqlib = 'libpq.so';
{$ENDIF}
{$IFDEF Windows}
  const
    pqlib = 'libpq.dll';
{$ENDIF}

{$i dllisttypes.inc}

var
  DLNewList : function : PDllist;cdecl;
  DLFreeList : procedure (_para1:PDllist);cdecl;
  DLNewElem : function (val : pointer) :PDlelem;cdecl;
  DLFreeElem : procedure (_para1:PDlelem);cdecl;
  DLGetHead : function (_para1:PDllist):PDlelem;cdecl;
  DLGetTail : function (_para1:PDllist):PDlelem;cdecl;
  DLRemTail : function (l:PDllist):PDlelem;cdecl;
  DLGetPred : function (_para1:PDlelem):PDlelem;cdecl;
  DLGetSucc : function (_para1:PDlelem):PDlelem;cdecl;
  DLRemove : procedure (_para1:PDlelem);cdecl;
  DLAddHead : procedure (list:PDllist; node:PDlelem);cdecl;
  DLAddTail : procedure (list:PDllist; node:PDlelem);cdecl;
  DLRemHead : function (list:PDllist):PDlelem;cdecl;

{ Macro translated }
Function  DLE_VAL(elem : PDlelem) : pointer;

Procedure InitialiseDllist;
Procedure ReleaseDllist;

var DllistLibraryHandle : TLibHandle;

implementation

var RefCount : integer;

Procedure InitialiseDllist;

begin
  inc(RefCount);
  if RefCount = 1 then
    begin
    DllistLibraryHandle := loadlibrary(pqlib);
    if DllistLibraryHandle = nilhandle then
      begin
      RefCount := 0;
      Raise EInOutError.Create('Can not load PosgreSQL client. Is it installed? ('+pqlib+')');
      end;

    pointer(DLNewList) := GetProcedureAddress(DllistLibraryHandle,'DLNewList');
    pointer(DLFreeList) := GetProcedureAddress(DllistLibraryHandle,'DLFreeList');
    pointer( DLNewElem) := GetProcedureAddress(DllistLibraryHandle,' DLNewElem');
    pointer(DLFreeElem) := GetProcedureAddress(DllistLibraryHandle,'DLFreeElem');
    pointer( DLGetHead) := GetProcedureAddress(DllistLibraryHandle,' DLGetHead');
    pointer( DLGetTail) := GetProcedureAddress(DllistLibraryHandle,' DLGetTail');
    pointer( DLRemTail) := GetProcedureAddress(DllistLibraryHandle,' DLRemTail');
    pointer( DLGetPred) := GetProcedureAddress(DllistLibraryHandle,' DLGetPred');
    pointer( DLGetSucc) := GetProcedureAddress(DllistLibraryHandle,' DLGetSucc');
    pointer(DLRemove) := GetProcedureAddress(DllistLibraryHandle,'DLRemove');
    pointer(DLAddHead) := GetProcedureAddress(DllistLibraryHandle,'DLAddHead');
    pointer(DLAddTail) := GetProcedureAddress(DllistLibraryHandle,'DLAddTail');
    pointer( DLRemHead) := GetProcedureAddress(DllistLibraryHandle,' DLRemHead');
    end;
end;

Procedure ReleaseDllist;

begin
  if RefCount > 0 then dec(RefCount);
  if RefCount = 0 then
    begin
    if not UnloadLibrary(DllistLibraryHandle) then inc(RefCount);
    end;
end;

// This function is also defined in Dllist!
Function DLE_VAL(elem : PDlelem) : pointer;
begin
  DLE_VAL:=elem^.dle_val
end;


end.
