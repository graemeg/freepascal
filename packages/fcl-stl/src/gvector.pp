{
   This file is part of the Free Pascal FCL library.
   BSD parts (c) 2011 Vlado Boza

   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{$mode objfpc}

unit gvector;

interface

type

  { TVector }

  generic TVector<T> = class
  private
  type
    PT = ^ T;
    TArr = array of T;
  var
    FCapacity:SizeUInt;
    FDataSize:SizeUInt;
    FData:TArr;

    procedure SetValue(Position: SizeUInt; const Value: T); inline;
    function GetValue(Position: SizeUInt): T; inline;
    function GetMutable(Position: SizeUInt): PT; inline;
    function NewCapacity: SizeUInt;
    procedure IncreaseCapacity;

  const
    // todo: move these constants to implementation when
    // mantis #0021310 will be fixed.
    SVectorPositionOutOfRange      = 'Vector position out of range';
    SAccessingElementOfEmptyVector = 'Accessing element of empty vector';

  type
    TVectorEnumerator = class
    private
      FVector: TVector;
      FPosition: SizeUInt;
      FFirstDone: Boolean;
      function GetCurrent: T; inline;
    public
      constructor Create(AVector: TVector);
      function GetEnumerator: TVectorEnumerator; inline;
      function MoveNext: Boolean; inline;
      property Current: T read GetCurrent;
    end;

  public
    constructor Create;
    function Size: SizeUInt; inline;
    procedure PushBack(const Value: T); inline;
    procedure PopBack; inline;
    function IsEmpty: boolean; inline;
    procedure Insert(Position: SizeUInt; const Value: T); inline;
    procedure Erase(Position: SizeUInt); inline;
    procedure Clear; inline;
    function Front: T; inline;
    function Back: T; inline;
    procedure Reserve(Num: SizeUInt);
    procedure Resize(Num: SizeUInt);

    function GetEnumerator: TVectorEnumerator; inline;

    property Items[i : SizeUInt]: T read getValue write setValue; default;
    property Mutable[i : SizeUInt]: PT read getMutable;
end;

implementation

{ TVector.TVectorEnumerator }

constructor TVector.TVectorEnumerator.Create(AVector: TVector);
begin
  FVector := AVector;
end;

function TVector.TVectorEnumerator.GetEnumerator: TVectorEnumerator;
begin
  result:=self;
end;

function TVector.TVectorEnumerator.GetCurrent: T;
begin
  Result := FVector[FPosition];
end;

function TVector.TVectorEnumerator.MoveNext: Boolean;
begin
  if not FFirstDone then begin
    Result := FVector.Size > 0;
    FFirstDone := True;
  end else begin
    Result := FPosition < FVector.Size - 1;
    if Result then
      inc(FPosition);
  end;
end;

{ TVector }

constructor TVector.Create();
begin
  FCapacity:=0;
  FDataSize:=0;
end;

procedure TVector.SetValue(Position: SizeUInt; const Value: T);
begin
  Assert(position < size, SVectorPositionOutOfRange);
  FData[Position]:=Value;
end;

function TVector.GetValue(Position: SizeUInt): T;
begin
  Assert(position < size, SVectorPositionOutOfRange);
  GetValue:=FData[Position];
end;

function TVector.GetMutable(Position: SizeUInt): PT;
begin
  Assert(position < size, SVectorPositionOutOfRange);
  GetMutable:=@FData[Position];
end;

function TVector.Front(): T;
begin
  Assert(size > 0, SAccessingElementOfEmptyVector);
  Front:=FData[0];
end;

function TVector.Back(): T;
begin
  Assert(size > 0, SAccessingElementOfEmptyVector);
  Back:=FData[FDataSize-1];
end;

function TVector.Size(): SizeUInt;
begin
  Size:=FDataSize;
end;

function TVector.IsEmpty(): boolean;
begin
  IsEmpty := (Size() = 0);
end;

procedure TVector.PushBack(const Value: T);
begin
  if FDataSize=FCapacity then
    IncreaseCapacity;
  FData[FDataSize]:=Value;
  inc(FDataSize);
end;

function TVector.NewCapacity: SizeUInt;
const
  // if size is small, multiply by 2;
  // if size bigger but <256M, inc by 1/8*size;
  // otherwise inc by 1/16*size
  cSizeSmall = 1*1024*1024;
  cSizeBig = 256*1024*1024;
var
  DataSize:SizeUInt;
begin
  DataSize:=FCapacity*SizeOf(T);
  if FCapacity=0 then
    Result:=4
  else
  if DataSize<cSizeSmall then
    Result:=FCapacity*2
  else
  if DataSize<cSizeBig then
    Result:=FCapacity+FCapacity div 8
  else
    Result:=FCapacity+FCapacity div 16;
end;

procedure TVector.IncreaseCapacity();
begin
  FCapacity:=NewCapacity;
  SetLength(FData, FCapacity);
end;

function TVector.GetEnumerator: TVectorEnumerator;
begin
  Result := TVectorEnumerator.Create(self);
end;

procedure TVector.PopBack();
begin
  if FDataSize > 0 then
  begin
    dec(FDataSize);
    // if a managed type, decrease the popped element's reference count (see http://bugs.freepascal.org/view.php?id=23938#)
    FData[FDataSize] := Default(T);
  end;
end;

procedure TVector.Insert(Position: SizeUInt; const Value: T);
var
  def: T;
begin
  if Position <= Size then	// allow appending a new element at end of vector (but not beyond)
  begin
    if FDataSize = FCapacity then
      IncreaseCapacity;
    if Position < FDataSize then
      System.Move (FData[Position], FData[Position+1], (FDataSize - Position) * SizeOf(T));
    // update inserted item
    def := Default(T);
    Move(def, FData[Position], SizeOf(T)); // this will clear FData[Position] without changing the reference count
    FData[Position] := Value;
    inc(FDataSize);
  end;
end;

procedure TVector.Erase(Position: SizeUInt);
begin
  if Position < Size then 
  begin
    dec(FDataSize);
    // ensure that the data we want to erase is released
    FData[Position] := Default(T);
    Move(FData[Position+1], FData[Position], (FDataSize - Position) * SizeOf(T));
  end;
end;

procedure TVector.Clear;
begin
  FDataSize:=0;
end;

procedure TVector.Reserve(Num: SizeUInt);
begin
  if(Num < FCapacity) then 
    exit
  else if (Num <= NewCapacity) then 
    IncreaseCapacity
  else begin 
    SetLength(FData, Num);
    FCapacity:=Num;
  end;
end;

procedure TVector.Resize(Num: SizeUInt);
begin
  Reserve(Num);
  FDataSize:=Num;
end;

end.
