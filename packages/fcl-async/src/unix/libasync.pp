{

    libasync: Asynchronous event management
    Copyright (C) 2001-2002 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    Unix implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

unit libasync;

{$MODE objfpc}

interface

type

  TAsyncData = record
    IsRunning, DoBreak: Boolean;
    HasCallbacks: Boolean;      // True as long as callbacks are set
    FirstTimer: Pointer;
    FirstIOCallback: Pointer;
    CurIOCallback: Pointer;     // current callback being processed within 'run'
    NextIOCallback: Pointer;    // next callback to get processed within 'run'
    FDData: Pointer;
    HighestHandle: LongInt;
  end;

{$INCLUDE libasynch.inc}



implementation

uses baseunix, Unix;

const
  MaxHandle = SizeOf(TFDSet) * 8 - 1;

type
  PIOCallbackData = ^TIOCallbackData;
  TIOCallbackData = record
    Next: PIOCallbackData;
    IOHandle: LongInt;
    ReadCallback, WriteCallback: TAsyncCallback;
    ReadUserData, WriteUserData: Pointer;
    SavedHandleFlags: LongInt;
  end;

{$INCLUDE libasync.inc}



procedure InternalInit(Handle: TAsyncHandle);
begin
  Handle^.Data.HighestHandle := -1;
end;

procedure InternalFree(Handle: TAsyncHandle);
var
  IOCallback: PIOCallbackData;
begin
  IOCallback := PIOCallbackData(Handle^.Data.FirstIOCallback);
  while Assigned(IOCallback) do
  begin
    if (IOCallback^.SavedHandleFlags and Open_NonBlock) = 0 then
      fpfcntl(IOCallback^.IOHandle, F_SetFl, IOCallback^.SavedHandleFlags);
    IOCallback := IOCallback^.Next;
  end;

  if Assigned(Handle^.Data.FDData) then
    FreeMem(Handle^.Data.FDData);
end;

procedure InternalRun(Handle: TAsyncHandle; TimeOut: Int64);
var
  AsyncResult: Integer;
  CurReadFDSet, CurWriteFDSet: TFDSet;
  CurIOCallback: PIOCallbackData;
begin
  if Handle^.Data.HighestHandle < 0 then
    // No I/O checks to do, so just wait...
    AsyncResult := fpselect(0, nil, nil, nil, TimeOut)
  else
  begin
    CurReadFDSet := PFDSet(Handle^.Data.FDData)[0];
    CurWriteFDSet := PFDSet(Handle^.Data.FDData)[1];
    AsyncResult := fpselect(Handle^.Data.HighestHandle + 1,
      @CurReadFDSet, @CurWriteFDSet, nil, TimeOut);

    if AsyncResult > 0 then
    begin
      // Check for I/O events
      Handle^.Data.CurIOCallback := Handle^.Data.FirstIOCallback;
      while Assigned(Handle^.Data.CurIOCallback) do
      begin
        CurIOCallback := PIOCallbackData(Handle^.Data.CurIOCallback);
        Handle^.Data.NextIOCallback := CurIOCallback^.Next;
        if (fpFD_ISSET(CurIOCallback^.IOHandle,CurReadFDSet) > 0) and
           (fpFD_ISSET(CurIOCallback^.IOHandle, PFDSet(Handle^.Data.FDData)[0]) > 0) and
          Assigned(CurIOCallback^.ReadCallback) then
        begin
          CurIOCallback^.ReadCallback(CurIOCallback^.ReadUserData);
          if Handle^.Data.DoBreak then
            break;
        end;

        CurIOCallback := PIOCallbackData(Handle^.Data.CurIOCallback);
        if Assigned(CurIOCallback) and
           (fpFD_ISSET(CurIOCallback^.IOHandle, CurWriteFDSet) > 0) and
           (fpFD_ISSET(CurIOCallback^.IOHandle, PFDSet(Handle^.Data.FDData)[1]) > 0) and
          Assigned(CurIOCallback^.WriteCallback) then
        begin
          CurIOCallback^.WriteCallback(CurIOCallback^.WriteUserData);
          if Handle^.Data.DoBreak then
            break;
        end;

        Handle^.Data.CurIOCallback := Handle^.Data.NextIOCallback;
      end;
    end;
  end;
end;

procedure InternalInitIOCallback(Handle: TAsyncHandle; Data: PIOCallbackData;
  InitData: Boolean; CallbackTypes: TCallbackTypes);
var
  i: LongInt;
begin
  if InitData then
  begin
    if not Assigned(Handle^.Data.FDData) then
    begin
      GetMem(Handle^.Data.FDData, SizeOf(TFDSet) * 2);
      fpFD_ZERO(PFDSet(Handle^.Data.FDData)[0]);
      fpFD_ZERO(PFDSet(Handle^.Data.FDData)[1]);
    end;
    if Data^.IOHandle > Handle^.Data.HighestHandle then
      Handle^.Data.HighestHandle := Data^.IOHandle;
  end;

  Data^.SavedHandleFlags := fpfcntl(Data^.IOHandle, F_GetFl);
  fpfcntl(Data^.IOHandle, F_SetFl, Data^.SavedHandleFlags or Open_NonBlock);

  case Data^.IOHandle of
    StdInputHandle:
      i := Open_RdOnly;
    StdOutputHandle, StdErrorHandle:
      i := Open_WrOnly;
    else
      i := Data^.SavedHandleFlags and Open_Accmode;
  end;

  case i of
    Open_RdOnly:
      if cbRead in CallbackTypes then
        fpFD_SET(Data^.IOHandle, PFDSet(Handle^.Data.FDData)[0]);
    Open_WrOnly:
      if cbWrite in CallbackTypes then
        fpFD_SET(Data^.IOHandle, PFDSet(Handle^.Data.FDData)[1]);
    Open_RdWr:
      begin
        if cbRead in CallbackTypes then
          fpFD_SET(Data^.IOHandle, PFDSet(Handle^.Data.FDData)[0]);
        if cbWrite in CallbackTypes then
          fpFD_SET(Data^.IOHandle, PFDSet(Handle^.Data.FDData)[1]);
      end;
  end;
end;

procedure InternalClearIOCallback(Handle: TAsyncHandle; IOHandle: LongInt;
  CallbackTypes: TCallbackTypes);
begin
  if not Assigned(Handle) then
    exit;

  if cbRead in CallbackTypes then
    fpFD_CLR(IOHandle, PFDSet(Handle^.Data.FDData)[0]);
  if cbWrite in CallbackTypes then
    fpFD_CLR(IOHandle, PFDSet(Handle^.Data.FDData)[1]);
end;

function asyncGetTicks: Int64; cdecl;
var
  Time: TimeVal;
begin
  fpGetTimeOfDay(@time,nil);
  Result := Int64(Time.tv_Sec) * 1000 + Int64(Time.tv_USec div 1000);
end;


end.
