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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

  { TVector }

  generic TVector<T> = class
  private
  type
    PT = ^ T;
    TArr = array of T;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  generic TVector<T>=class
  private
  type
    PT=^ T;
    TArr=array of T;
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
  var
    FCapacity:SizeUInt;
    FDataSize:SizeUInt;
    FData:TArr;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetValue(Position: SizeUInt; const Value: T); inline;
    function GetValue(Position: SizeUInt): T; inline;
    function GetMutable(Position: SizeUInt): PT; inline;
    procedure IncreaseCapacity; inline;

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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure SetValue(Position:SizeUInt; Value:T);inline;
    function GetValue(Position:SizeUInt):T;inline;
    function GetMutable(Position:SizeUInt):PT;inline;
    procedure IncreaseCapacity;inline;
  public
    constructor Create;
    function Size:SizeUInt;inline;
    procedure PushBack(Value:T);inline;
    procedure PopBack;inline;
    function IsEmpty:boolean;inline;
    procedure Insert(Position:SizeUInt; Value:T);inline;
    procedure Erase(Position:SizeUInt);inline;
    procedure Clear;inline;
    function Front:T;inline;
    function Back:T;inline;
    procedure Reserve(Num:SizeUInt);inline;
    procedure Resize(Num:SizeUInt);inline;
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

    property Items[i : SizeUInt]: T read getValue write setValue; default;
    property Mutable[i : SizeUInt]: PT read getMutable;
end;

implementation

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ TVector.TVectorEnumerator }

constructor TVector.TVectorEnumerator.Create(AVector: TVector);
begin
  FVector := AVector;
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

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
constructor TVector.Create();
begin
  FCapacity:=0;
  FDataSize:=0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TVector.SetValue(Position:SizeUInt; Value:T);inline;
begin
  Assert(position < size, 'Vector position out of range');
  FData[Position]:=Value;
end;

function TVector.GetValue(Position:SizeUInt):T;inline;
begin
  Assert(position < size, 'Vector position out of range');
  GetValue:=FData[Position];
end;

function TVector.GetMutable(Position:SizeUInt):PT;inline;
begin
  Assert(position < size, 'Vector position out of range');
  GetMutable:=@FData[Position];
end;

function TVector.Front():T;inline;
begin
  Assert(size > 0, 'Accessing element of empty vector');
  Front:=FData[0];
end;

function TVector.Back():T;inline;
begin
  Assert(size > 0, 'Accessing element of empty vector');
  Back:=FData[FDataSize-1];
end;

function TVector.Size():SizeUInt;inline;
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
begin
  Size:=FDataSize;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TVector.IsEmpty(): boolean;
begin
  IsEmpty := (Size() = 0);
end;

procedure TVector.PushBack(const Value: T);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TVector.IsEmpty():boolean;inline;
begin
  if Size()=0 then 
    IsEmpty:=true
  else 
    IsEmpty:=false;
end;

procedure TVector.PushBack(Value:T);inline;
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
begin
  if FDataSize=FCapacity then
    IncreaseCapacity;
  FData[FDataSize]:=Value;
  inc(FDataSize);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVector.IncreaseCapacity();
=======
procedure TVector.IncreaseCapacity();inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.IncreaseCapacity();inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.IncreaseCapacity();inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.IncreaseCapacity();inline;
>>>>>>> origin/cpstrnew
begin
  if FCapacity=0 then
    FCapacity:=1
  else
    FCapacity:=FCapacity*2;
  SetLength(FData, FCapacity);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure TVector.PopBack();inline;
begin
  if FDataSize>0 then
    FDataSize:=FDataSize-1;
end;

procedure TVector.Insert(Position:SizeUInt; Value: T);inline;
var i:SizeUInt;
begin
  pushBack(Value);
  for i:=Size-1 downto Position+1 do 
  begin
    FData[i]:=FData[i-1];
  end;
  FData[Position]:=Value;
end;

procedure TVector.Erase(Position:SizeUInt);inline;
var i:SizeUInt;
begin
  if Position <= Size then 
  begin
    for i:=Position to Size-2 do
    begin
      FData[i]:=FData[i+1];
    end;
    popBack();
  end;
end;

procedure TVector.Clear;inline;
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
begin
  FDataSize:=0;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVector.Reserve(Num: SizeUInt);
=======
procedure TVector.Reserve(Num:SizeUInt);inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.Reserve(Num:SizeUInt);inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.Reserve(Num:SizeUInt);inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.Reserve(Num:SizeUInt);inline;
>>>>>>> origin/cpstrnew
begin
  if(Num < FCapacity) then 
    exit
  else if(Num <= 2*FCapacity) then 
    IncreaseCapacity
  else begin 
    SetLength(FData, Num);
    FCapacity:=Num;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TVector.Resize(Num: SizeUInt);
=======
procedure TVector.Resize(Num:SizeUInt);inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.Resize(Num:SizeUInt);inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.Resize(Num:SizeUInt);inline;
>>>>>>> graemeg/cpstrnew
=======
procedure TVector.Resize(Num:SizeUInt);inline;
>>>>>>> origin/cpstrnew
begin
  Reserve(Num);
  FDataSize:=Num;
end;

end.
