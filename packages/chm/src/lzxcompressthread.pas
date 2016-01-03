{ Copyright (C) <2009> <Andrew Haines> lzxcompressthread.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2.4
}
{
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
unit lzxcompressthread;

{$mode objfpc}{$H+}

interface

uses
  Classes, paslzxcomp;

type
  TLZXCompressor = class;
  TLZXMasterThread = class;
  TLZXWorkerThread = class;

  TLZXGetDataMethod = function(Sender: TLZXCompressor; WantedByteCount: Integer; Buffer: Pointer): Integer of object;
  TLZXIsEndOfFileMethod = function(Sender: TLZXCompressor): Boolean  of object;
  TLZXChunkDoneMethod = procedure(Sender: TLZXCompressor; CompressedSize: Integer; UncompressedSize: Integer; Buffer: Pointer) of object;
  TLZXMarkFrameMethod = procedure(Sender: TLZXCompressor; CompressedSize: Integer; UncompressedSize: Integer) of object;

  PLZXFinishedBlock = ^TLZXFinishedBlock;
  TLZXFinishedBlock = record
    CompressedSize: Integer;
    UnCompressedSize: Integer;
    Frame1CSize,
    Frame1USize,
    Frame2CSize,
    Frame2USize: Integer;
    Index: Integer;
    Data: Pointer;
  end;

  { TLZXCompressor }

  TLZXCompressor = class(TObject)
  private
    FOnMarkFrame: TLZXMarkFrameMethod;
    FThreadCount: Integer;
    FBlockSize: Integer;
    FTotalCompressedSize: DWord;
    FTotalUnCompressedSize: DWord;
    FOnChunkDone: TLZXChunkDoneMethod;
    FOnGetData: TLZXGetDataMethod;
    FOnIsEndOfFile: TLZXIsEndOfFileMethod;
    FWindowSizeCode: Integer;
    FinishedBlocks: TFPList;
    NextBlockNeeded: Integer;
    FMasterThread: TLZXMasterThread;
    procedure BlockIsFinished(ABlock: PLZXFinishedBlock);
    function GetRunning: Boolean;
  public
    constructor Create(AThreadCount: Integer);
    destructor  Destroy; override;

    procedure   Execute(WaitForFinish: Boolean = True);

    property BlockSize: Integer read FBlockSize write FBlockSize;
    property Running: Boolean read GetRunning;
    property WindowSizeCode: Integer read FWindowSizeCode write FWindowSizeCode;

    // the following properties must all be assigned
    property OnGetData: TLZXGetDataMethod read FOnGetData write FOnGetData;
    property OnChunkDone: TLZXChunkDoneMethod read FOnChunkDone write FOnChunkDone;
    property OnIsEndOfFile: TLZXIsEndOfFileMethod read FOnIsEndOfFile write FOnIsEndOfFile;
    property OnMarkFrame: TLZXMarkFrameMethod read FOnMarkFrame write FOnMarkFrame;
  end;

  { TLZXMasterThread }

  TLZXMasterThread = class(TThread)
    FCompressor: TLZXCompressor;
    FBusyThreads: TFPList;
    FLockData: TRTLCriticalSection;
    FLockQueueThread: TRTLCriticalSection;
    FDataRemains: Boolean;
    FBlockNumber: Integer;
    FRunning: Boolean;
    FMemList: TFPList;

    // only used inside a critical section!!
    // belongs to a Worker thread which will free it
    FTmpData: Pointer;
    FTmpDataSize: Integer;

    procedure UpdateDataRemains;

    function  BlockDone(Worker: TLZXWorkerThread; ABlock: PLZXFinishedBlock): Boolean;
    procedure WorkerFinished(Sender: TObject);
    function GetFreeMemoryChunk: Pointer;


    procedure Lock;
    Procedure UnLock;
    procedure LockTmpData;
    procedure UnLockTmpData;
    function Working: Boolean;
    function DataRemains: Boolean;
    function Running: Boolean;
    function QueueThread(Thread: TLZXWorkerThread): Boolean;

  public
    procedure   Execute; override;
    constructor Create(Compressor: TLZXCompressor);
    destructor  Destroy; override;
  end;

  { TLZXWorkerThread }

  TLZXWorkerThread = class(TThread)
  private
    Data: PByte;
    DataSize: Integer;
    DataCursor: Integer;
    Frame1C,
    Frame1U,
    Frame2C,
    Frame2U: Integer;

    LZXData: Plzx_data;

    CompressedData: PByte;
    CompressedDataSize: Integer; // compressed written size. not the size of the array

    BlockNumber: Integer;

    WindowSizeCode: Integer;
    BlockSize: Integer;

    MasterThread: TLZXMasterThread;
    ShouldSuspend: Boolean;

    // callbacks for lzxcomp
    function    GetBytes(ACount: Longint; ABuffer: Pointer): LongInt; cdecl;
    function    WriteBytes(ACount: LongInt; ABuffer: Pointer): LongInt; cdecl;
    procedure   MarkFrame(UnCompressedSize: DWord; CompressedSize: DWord); cdecl;
    function    IsEndOfFile: LongBool; cdecl;
    // end callbacks
    procedure   NotifyMasterDone;
  protected
    procedure   Execute; override;
  public
    procedure   CompressData(ABlockNumber: Integer);

    constructor Create(AMaster: TLZXMasterThread; AWindowSizeCode: Integer; ABlockSize: Integer);
    destructor  Destroy; override;
  end;

implementation
uses
  Sysutils; // for Sleep()

{ TLZXCompressor }

procedure TLZXCompressor.BlockIsFinished(ABlock: PLZXFinishedBlock);
  procedure SendChunk(AChunk: PLZXFinishedBlock);
  begin
    if Assigned(FOnMarkFrame) then
    begin
      Inc(FTotalCompressedSize, AChunk^.Frame1CSize);
      Inc(FTotalUnCompressedSize, AChunk^.Frame1USize);
      Inc(NextBlockNeeded);

      FOnMarkFrame(Self, FTotalCompressedSize, FTotalUnCompressedSize);

      if AChunk^.Frame2CSize > 0 then
      begin
        Inc(FTotalCompressedSize, AChunk^.Frame2CSize);
        Inc(FTotalUnCompressedSize, AChunk^.Frame2USize);
        FOnMarkFrame(Self, FTotalCompressedSize, FTotalUnCompressedSize);
      end;
    end;

    FOnChunkDone(Self, AChunk^.CompressedSize, AChunk^.UnCompressedSize, AChunk^.Data);
    FMasterThread.FMemList.Add(AChunk^.Data);
    Dispose(AChunk);
  end;
var
  TmpBlock  : PLZXFinishedBlock;
  FoundMatch: Boolean;
  i: Integer;

begin
  if NextBlockNeeded = ABlock^.Index then
    SendChunk(ABlock)
  else
    FinishedBlocks.Add(ABlock);

  repeat
    FoundMatch := False;
    for i := FinishedBlocks.Count-1 downto 0 do
    begin
      TmpBlock := PLZXFinishedBlock(FinishedBlocks.Items[i]);
      if TmpBlock^.Index = NextBlockNeeded then
      begin
        FoundMatch := True;
        SendChunk(TmpBlock);
        FinishedBlocks.Delete(i);
      end;
    end;
  until not FoundMatch;
end;

function TLZXCompressor.GetRunning: Boolean;
begin
  Result := FMasterThread.Running;
end;

constructor TLZXCompressor.Create(AThreadCount: Integer);
begin
  inherited Create;
  FThreadCount    := AThreadCount;
  FBlockSize      := 1 shl 16 ; // $10000;
  FWindowSizeCode := 16;
  FMasterThread := TLZXMasterThread.Create(Self);
  FinishedBlocks := TFPList.Create;
end;

destructor TLZXCompressor.Destroy;
begin
  FMasterThread.Free;
  FinishedBlocks.Free;
  inherited Destroy;
end;

procedure TLZXCompressor.Execute(WaitForFinish: Boolean = True);
begin
    FTotalCompressedSize := 0;
    FTotalUnCompressedSize := 0;
    FMasterThread.FRunning:=True;
    FMasterThread.Resume;
    if WaitForFinish then
      While Running do
<<<<<<< HEAD
        CheckSynchronize(10);
=======
        CheckSynchronize(50);
>>>>>>> origin/fixes_2.4
end;

{ TLZXMasterThread }

procedure TLZXMasterThread.UpdateDataRemains;
begin
  FDataRemains := not FCompressor.FOnIsEndOfFile(FCompressor);
end;

function TLZXMasterThread.BlockDone(Worker: TLZXWorkerThread; ABlock: PLZXFinishedBlock): Boolean;
begin
  Lock;
  REsult := True;
<<<<<<< HEAD

=======
>>>>>>> origin/fixes_2.4
  FCompressor.BlockIsFinished(ABlock);
  if DataRemains then
    QueueThread(Worker)
  else
  begin
    Result := False;
    FBusyThreads.Remove(Worker);
    Worker.Terminate;
    if FBusyThreads.Count = 0 then
      Resume;
  end;
  Unlock;
end;

procedure TLZXMasterThread.WorkerFinished(Sender: TObject);
begin
  FBusyThreads.Remove(Sender);
  if TThread(Sender).FatalException <> nil then
    Raise Exception(TThread(Sender).FatalException);
  Sender.Free;
end;

function TLZXMasterThread.GetFreeMemoryChunk: Pointer;
begin
  if FMemList.Count >0 then
  begin
    Result := FMemList.Items[0];
    FMemList.Delete(0);
  end
  else
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Result := Getmem(FCompressor.BlockSize*2); // it's unlikely but possible for the block to be bigger than the orig size
=======
    Result := Getmem(FCompressor.BlockSize);
>>>>>>> graemeg/cpstrnew
=======
    Result := Getmem(FCompressor.BlockSize);
>>>>>>> graemeg/cpstrnew
=======
    Result := Getmem(FCompressor.BlockSize);
>>>>>>> graemeg/cpstrnew
=======
    Result := Getmem(FCompressor.BlockSize);
>>>>>>> origin/cpstrnew
=======
    Result := Getmem(FCompressor.BlockSize);
>>>>>>> origin/fixes_2.4
end;

procedure TLZXMasterThread.Lock;
begin
  EnterCriticalsection(FLockData);
end;

procedure TLZXMasterThread.UnLock;
begin
  LeaveCriticalsection(FLockData);
end;

procedure TLZXMasterThread.LockTmpData;
begin
  EnterCriticalsection(FLockQueueThread);
end;

procedure TLZXMasterThread.UnLockTmpData;
begin
  LeaveCriticalsection(FLockQueueThread);
end;

function TLZXMasterThread.Working: Boolean;
begin
  Result := FBusyThreads.Count > 0;
end;

function TLZXMasterThread.DataRemains: Boolean;
begin
  UpdateDataRemains;
  Result := FDataRemains;
end;

function TLZXMasterThread.Running: Boolean;
begin
  REsult := FRunning;
end;

function TLZXMasterThread.QueueThread(Thread: TLZXWorkerThread): Boolean;
begin
  LockTmpData;

  Result := DataRemains;
  if Not Result then
  begin
    UnLockTmpData;
    Exit;
  end;

  FDataRemains := False;

  Thread.DataSize := FCompressor.OnGetData(FCompressor, FCompressor.FBlockSize, Thread.Data);

  Thread.CompressData(FBlockNumber);
  Inc(FBlockNumber);
<<<<<<< HEAD
  if Thread.Suspended then
    Thread.Resume;
=======
  Thread.Resume;
>>>>>>> origin/fixes_2.4
  UnLockTmpData;
end;

procedure TLZXMasterThread.Execute;
var
  i: Integer;
  Thread: TLZXWorkerThread;
begin
  FRunning:= True;
  for i := 0 to FCompressor.FThreadCount-1 do
  begin
    Thread := TLZXWorkerThread.Create(Self, FCompressor.WindowSizeCode, FCompressor.BlockSize);
    Thread.FreeOnTerminate := True;
    Thread.OnTerminate:= @WorkerFinished;
    if QueueThread(Thread) then
      FBusyThreads.Add(Thread);
  end;
  //Suspend;
  while Working do
  begin
<<<<<<< HEAD
      Sleep(0);
=======
      Sleep(50);
>>>>>>> origin/fixes_2.4
  end;
  FRunning:= False;
end;

constructor TLZXMasterThread.Create(Compressor: TLZXCompressor);
begin
  Inherited Create(True);
  FCompressor  := Compressor;
  FDataRemains := True;
  FBusyThreads := TFPList.Create;
  FMemList := TFPList.Create;
  InitCriticalSection(FLockData);
  InitCriticalSection(FLockQueueThread);
end;

destructor TLZXMasterThread.Destroy;
var
  i: Integer;
begin
  DoneCriticalsection(FLockData);
  DoneCriticalsection(FLockQueueThread);

  for i := 0 to FBusyThreads.Count-1 do TObject(FBusyThreads.Items[i]).Free;
  for i := 0 to FMemList.Count-1 do Freemem(FMemList.Items[i]);

  FBusyThreads.Free;
  FMemList.Free;

  inherited Destroy;
end;

{ TLZXWorkerThread }

function TLZXWorkerThread.GetBytes(ACount: Longint; ABuffer: Pointer): LongInt; cdecl;
var
  MaxBytes: Integer;
begin
  MaxBytes := DataSize - DataCursor;

  if ACount > MaxBytes then
    ACount := MaxBytes;

  Move(Data[DataCursor], ABuffer^, ACount);
  Inc(DataCursor, ACount);

  Result := ACount;
end;

function TLZXWorkerThread.WriteBytes(ACount: LongInt; ABuffer: Pointer
  ): LongInt; cdecl;
begin
  Move(ABuffer^, CompressedData[CompressedDataSize], ACount);
  Inc(CompressedDataSize, ACount);
  Result := ACount;
end;

procedure TLZXWorkerThread.MarkFrame(UnCompressedSize: DWord;
  CompressedSize: DWord); cdecl;
begin
  if Frame1C = 0 then
  begin
    Frame1C := CompressedSize;
    Frame1U := UnCompressedSize;
  end
  else
  begin
    Frame2C := CompressedSize;
    Frame2U := UnCompressedSize;
  end;
end;

function TLZXWorkerThread.IsEndOfFile: LongBool; cdecl;
begin
  Result := LongBool(DataSize - DataCursor <= 0);
end;

procedure TLZXWorkerThread.NotifyMasterDone;
var
  Block: PLZXFinishedBlock;
begin
  LZXData^.len_compressed_output:=0;
  LZXData^.len_uncompressed_input:=0;

  New(Block);
  Block^.Data            :=  MasterThread.GetFreeMemoryChunk;
  Move(CompressedData^, Block^.Data^, CompressedDataSize);
  Block^.CompressedSize  := CompressedDataSize;
  Block^.UnCompressedSize:= DataSize;
  Block^.Index           := BlockNumber;
  Block^.Frame1CSize      := Frame1C;
  Block^.Frame2CSize      := Frame2C-Frame1C;
  Block^.Frame1USize      := Frame1U;
  Block^.Frame2USize      := Frame2U-Frame1U;

  Frame1C := 0;
  Frame2C := 0;
  Frame1U := 0;
  Frame2U := 0;

  ShouldSuspend := not MasterThread.BlockDone(Self, Block);

end;

procedure TLZXWorkerThread.CompressData(ABlockNumber: Integer);
begin
  BlockNumber := ABlockNumber;
  DataCursor := 0;
  CompressedDataSize := 0;
end;

procedure TLZXWorkerThread.Execute;
var
  WSize: LongInt;
begin
  WSize := 1 shl WindowSizeCode;
  while not Terminated do
  begin
    lzx_reset(LZXdata);
<<<<<<< HEAD

=======
>>>>>>> origin/fixes_2.4
    lzx_compress_block(LZXdata, WSize, True);

    MasterThread.Synchronize(@NotifyMasterDone);

    if ShouldSuspend then
<<<<<<< HEAD
    begin
      Suspend;
    end;

=======
      Suspend;
>>>>>>> origin/fixes_2.4
  end;
end;

constructor TLZXWorkerThread.Create(AMaster: TLZXMasterThread;
  AWindowSizeCode: Integer; ABlockSize: Integer);
begin
  Inherited Create(True);
  MasterThread := AMaster;
  WindowSizeCode := AWindowSizeCode;
  BlockSize     := ABlockSize;
  FreeOnTerminate := True;

  Data  := GetMem(ABlockSize);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  //it's possible to have a chunk be slightly bigger than the data it's compressing
  CompressedData:=GetMem(ABlockSize*2);
=======
  CompressedData:=GetMem(ABlockSize);
>>>>>>> graemeg/cpstrnew
=======
  CompressedData:=GetMem(ABlockSize);
>>>>>>> graemeg/cpstrnew
=======
  CompressedData:=GetMem(ABlockSize);
>>>>>>> graemeg/cpstrnew
=======
  CompressedData:=GetMem(ABlockSize);
>>>>>>> origin/cpstrnew
=======
  CompressedData:=GetMem(ABlockSize);
>>>>>>> origin/fixes_2.4

  lzx_init(@LZXdata, longint(WindowSizeCode),
           TGetBytesFunc(@TLZXWorkerThread.GetBytes), Self,
           TIsEndOfFileFunc(@TLZXWorkerThread.IsEndOfFile),
           TWriteBytesFunc(@TLZXWorkerThread.WriteBytes), Self,
           TMarkFrameFunc(@TLZXWorkerThread.MarkFrame), Self);
end;

destructor TLZXWorkerThread.Destroy;
begin
  lzx_finish(LZXdata, nil);
  Freemem(Data);
  FreeMem(CompressedData);
  inherited Destroy;
end;

end.

