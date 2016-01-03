{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program Poolmm1;

{$mode objfpc}{$H+}

uses
  Classes, SysUtils, pooledmm;

type
  PMyRecord = ^TMyRecord;
  TMyRecord = record
    Next: PMyRecord;
    Value1, Value2, Value3: integer;
  end;

type
  TMyRecordMemManager = class(TPooledMemManager)
  protected
    procedure FreeFirstItem; override;
  public
    procedure DisposeMyRecord(aMyRecord: PMyRecord);
    function NewMyRecord: PMyRecord;
  end;

procedure TMyRecordMemManager.FreeFirstItem;
var AMyRecord: PMyRecord;
begin
  AMyRecord:=PMyRecord(FFirstFree);
  PMyRecord(FFirstFree):=AMyRecord^.Next;
  Dispose(AMyRecord);
  {$R-}
  inc(FFreedCount);
  {$IfDef RangeChecksOn}{$R+}{$Endif}
end;

procedure TMyRecordMemManager.DisposeMyRecord(AMyRecord: PMyRecord);
begin
  if (FFreeCount<FMinFree) or (FFreeCount<((FCount shr 3)*FMaxFreeRatio)) then
  begin
    // add AMyRecord to Free list
    AMyRecord^.Next:=PMyRecord(FFirstFree);
    PMyRecord(FFirstFree):=AMyRecord;
    inc(FFreeCount);
  end else begin
    // free list full -> free the item
    Dispose(AMyRecord);
    {$R-}
    inc(FFreedCount);
    {$IfDef RangeChecksOn}{$R+}{$Endif}
  end;
  dec(FCount);
end;

function TMyRecordMemManager.NewMyRecord: PMyRecord;
begin
  if FFirstFree<>nil then begin
    // take from free list
    Result:=PMyRecord(FFirstFree);
    PMyRecord(FFirstFree):=Result^.Next;
    dec(FFreeCount);
  end else begin
    // free list empty -> create new item
    New(Result);
    {$R-}
    inc(FAllocatedCount);
    {$IfDef RangeChecksOn}{$R+}{$Endif}
  end;
  FillChar(Result^, SizeOf(TMyRecord), 0);
  inc(FCount);
end;

const
  MyRecordMemManager: TMyRecordMemManager = nil;

var
  MyRecord: PMyRecord;
begin
  // init the manager for PMyRecord
  MyRecordMemManager:=TMyRecordMemManager.Create;
  MyRecordMemManager.MinimumFreeCount:=1000;

  // allocate one record
  MyRecord:=MyRecordMemManager.NewMyRecord;

  // free the record
  MyRecordMemManager.DisposeMyRecord(MyRecord);
  
  // free the manager
  MyRecordMemManager.Free;
end.

