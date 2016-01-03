{

    FPCRes - Free Pascal Resource Converter
    Part of the Free Pascal distribution
    Copyright (C) 2008-2010 by Giulio Bernardi

    Support for file streams that can be temporarily closed

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}

unit closablefilestream;

{$MODE OBJFPC} {$H+}

interface

uses
  Classes, SysUtils;

type
  TClosableFileNotifier = class;

  { TClosableFileStream }

  TClosableFileStream = class(TStream)
  private
    fStream : TFileStream;
    fFileName : string;
    fMode : word;
    fListener : TClosableFileNotifier;
    fPosition : int64;
    procedure EnsureHandleOpen;
  protected
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    procedure SetSize(NewSize: Longint); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure SetSize(NewSize: Longint); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure SetSize(NewSize: Longint); override;
>>>>>>> graemeg/cpstrnew
=======
    procedure SetSize(NewSize: Longint); override;
>>>>>>> origin/cpstrnew
    procedure SetSize(const NewSize: Int64); override;
    function RetryOpen : boolean;
  public
    constructor Create(const AFileName: String; Mode: Word);
    destructor Destroy; override;
    function Read(var Buffer; Count: Longint): Longint; override;
    function Write(const Buffer; Count: Longint): Longint; override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
    function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> graemeg/cpstrnew
=======
    function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> graemeg/cpstrnew
=======
    function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> graemeg/cpstrnew
=======
    function Seek(Offset: Longint; Origin: Word): Longint; override;
>>>>>>> origin/cpstrnew
    function Seek(const Offset: Int64; Origin: TSeekOrigin): Int64; override;
    procedure CloseHandle;
end;

TClosableFileNotifier = class
protected
  procedure NotifyFileOpened(aStream : TClosableFileStream); virtual; abstract;
  function NotifyOpenFailed(aStream: TClosableFileStream) : boolean;
    virtual; abstract;
end;


implementation

type
  PQueueItem = ^TQueueItem;
  TQueueItem = record
    prev : PQueueItem;
    next : PQueueItem;
    data : pointer;
  end;

  { TFIFOQueue }

  TFIFOQueue = class
  private
    fHead : PQueueItem;
    fTail : PQueueItem;
    fCount : integer;
  protected
  public
    constructor Create;
    destructor Destroy; override;
    procedure Add(p : pointer);
    function Remove : Pointer;
    property Count : integer read fCount;
  end;


  { TFileKeeper }

  TFileKeeper = class(TClosableFileNotifier)
  private
    fOpenFiles : TFIFOQueue;
    fMaxOpen : longint;
  private
    procedure CloseFiles(const count : integer);
  protected
    procedure NotifyFileOpened(aStream : TClosableFileStream);  override;
    function NotifyOpenFailed(aStream: TClosableFileStream) : boolean; override;
  public
    constructor Create;
    destructor Destroy; override;
  end;

{ TFIFOQueue }

constructor TFIFOQueue.Create;
begin
  fHead:=nil;
  fTail:=nil;
  fCount:=0;
end;

destructor TFIFOQueue.Destroy;
var
  el : PQueueItem;
begin
  while fHead<>nil do
  begin
    el:=fHead;
    fHead:=fHead^.next;
    FreeMem(el);
  end;
end;

procedure TFIFOQueue.Add(p: pointer);
var
  el : PQueueItem;
begin
  el:=GetMem(sizeof(TQueueItem));
  el^.data:=p;
  el^.prev:=nil;
  el^.next:=fHead;
  if fHead<>nil then
    fHead^.prev:=el;
  fHead:=el;
  if fTail=nil then
    fTail:=fHead;
  inc(fCount);
end;

function TFIFOQueue.Remove: Pointer;
var
  el : PQueueItem;
begin
  if fCount=0 then
  begin
    Result:=nil;
    exit;
  end;
  Result:=fTail^.data;
  el:=fTail;
  fTail:=fTail^.prev;
  if fTail=nil then
    fHead:=nil
  else
    fTail^.next:=nil;
  FreeMem(el);
  dec(fCount);
end;

{ TFileKeeper }

procedure TFileKeeper.CloseFiles(const count: integer);
var
  i,max : integer;
  tmp : TClosableFileStream;
begin
  max:=count;
  if fOpenFiles.Count<max then
    max:=fOpenFiles.Count;
  for i:=0 to max-1 do
  begin
    tmp:=TClosableFileStream(fOpenFiles.Remove);
    tmp.CloseHandle;
  end;
end;

procedure TFileKeeper.NotifyFileOpened(aStream: TClosableFileStream);
begin
   fOpenFiles.Add(aStream);
   if (fOpenFiles.Count>=fMaxOpen) then
     CloseFiles(1);
end;

function TFileKeeper.NotifyOpenFailed(aStream: TClosableFileStream) : boolean;
var
  decrement : longint;
begin
  Result:=false;
  decrement:=round(0.1*fOpenFiles.Count);
  if decrement<=0 then exit;
  CloseFiles(decrement);
  Result:=aStream.RetryOpen;
  if Result then
    fMaxOpen:=fOpenFiles.Count+1;
end;

constructor TFileKeeper.Create;
begin
  fOpenFiles:=TFIFOQueue.Create;
  fMaxOpen:=high(longint);
end;

destructor TFileKeeper.Destroy;
begin
  fOpenFiles.Free;
end;

var
  FileKeeper : TFileKeeper;

{ TClosableFileStream }

procedure TClosableFileStream.EnsureHandleOpen;
begin
  if fStream<>nil then
    exit;
  try
    fStream:=TFileStream.Create(fFileName,fMode);
  except
    if not fListener.NotifyOpenFailed(self) then
      raise;
  end;
  fStream.Position:=fPosition;
  fListener.NotifyFileOpened(self);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TClosableFileStream.SetSize(NewSize: Longint);
begin
  SetSize(int64(NewSize));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TClosableFileStream.SetSize(const NewSize: Int64);
begin
  EnsureHandleOpen;
  fStream.Size:=NewSize;
end;

function TClosableFileStream.RetryOpen: boolean;
begin
  try
    fStream:=TFileStream.Create(fFileName,fMode);
  except
    Result:=false;
    exit;
  end;
  Result:=true;
end;

constructor TClosableFileStream.Create(const AFileName: String; Mode: Word);
begin
  fListener:=FileKeeper;
  fFileName:=aFileName;
  fMode:=Mode;
  fPosition:=0;
  fStream:=nil;
  EnsureHandleOpen;
  //if the file has been created, ensure that further openings don't recreate
  //it again!
  if fMode=fmCreate then
    fMode:=fmOpenReadWrite;
end;

destructor TClosableFileStream.Destroy;
begin
  if fStream<>nil then
    CloseHandle;
end;

function TClosableFileStream.Read(var Buffer; Count: Longint): Longint;
begin
  EnsureHandleOpen;
  Result:=fStream.Read(Buffer,Count);
end;

function TClosableFileStream.Write(const Buffer; Count: Longint): Longint;
begin
  EnsureHandleOpen;
  Result:=fStream.Write(Buffer,Count);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TClosableFileStream.Seek(Offset: Longint; Origin: Word): Longint;
begin
  EnsureHandleOpen;
  Result:=fStream.Seek(Offset,Origin);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TClosableFileStream.Seek(const Offset: Int64; Origin: TSeekOrigin
  ): Int64;
begin
  EnsureHandleOpen;
  Result:=fStream.Seek(Offset,Origin);
end;

procedure TClosableFileStream.CloseHandle;
begin
  fPosition:=fStream.Position;
  fStream.Free;
  fStream:=nil;
end;

initialization
  FileKeeper:=TFileKeeper.Create;

finalization
  FileKeeper.Destroy;

end.

