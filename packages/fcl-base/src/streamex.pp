{$mode objfpc}
{$h+}
unit streamex;

Interface

uses Classes;

type

   { TBidirBinaryObjectReader }

   TBidirBinaryObjectReader = class(TBinaryObjectReader)
   protected
      function GetPosition: Longint;
      procedure SetPosition(const AValue: Longint);
   public
      property Position: Longint read GetPosition write SetPosition;
   end;
   
   { TBidirBinaryObjectWriter }

   TBidirBinaryObjectWriter = class(TBinaryObjectWriter)
   protected
      function GetPosition: Longint;
      procedure SetPosition(const AValue: Longint);
   public
      property Position: Longint read GetPosition write SetPosition;
   end;
  
   { TDelphiReader }

   TDelphiReader = class(TReader)
   protected
      function GetPosition: LongInt;
      procedure SetPosition(const AValue: LongInt);
      function CreateDriver(Stream: TStream; BufSize: Integer): TAbstractObjectReader; override;
   public
      function GetDriver: TBidirBinaryObjectReader;
      function ReadStr: string;
      procedure Read(var Buf; Count: LongInt); override;
      property Position: LongInt read GetPosition write SetPosition;
   end;

   { TDelphiWriter }

   TDelphiWriter = class(TWriter)
   protected
      function GetPosition: Longint;
      procedure SetPosition(const AValue: LongInt);
      function CreateDriver(Stream: TStream; BufSize: Integer): TAbstractObjectWriter; override;
   public
      function GetDriver: TBidirBinaryObjectWriter;
      procedure FlushBuffer;
      procedure Write(const Buf; Count: LongInt); override;
      procedure WriteStr(const Value: string);
      procedure WriteValue(Value: TValueType);
      property Position: LongInt read GetPosition write SetPosition;
   end;

Implementation

{ TBidirBinaryObjectReader }

function TBidirBinaryObjectReader.GetPosition: Longint;
begin
   Result := FStream.Position - (FBufEnd - FBufPos);
end;

procedure TBidirBinaryObjectReader.SetPosition(const AValue: Longint);
begin
   FStream.Position := AValue;
   FBufPos := 0;
   FBufEnd := 0;
end;

{ TBidirBinaryObjectWriter }

function TBidirBinaryObjectWriter.GetPosition: Longint;
begin
   Result := FStream.Position - (FBufEnd - FBufPos);
end;

procedure TBidirBinaryObjectWriter.SetPosition(const AValue: Longint);
begin
   FStream.Position := AValue;
   FBufPos := 0;
   FBufEnd := 0;
end;



{ TDelphiReader }

function TDelphiReader.GetDriver: TBidirBinaryObjectReader;
begin
   Result := (Driver as TBidirBinaryObjectReader);
end;

function TDelphiReader.GetPosition: LongInt;
begin
   Result := GetDriver.Position;
end;

procedure TDelphiReader.SetPosition(const AValue: LongInt);
begin
   GetDriver.Position := AValue;
end;

function TDelphiReader.CreateDriver(Stream: TStream; BufSize:
Integer): TAbstractObjectReader;
begin
   Result := TBidirBinaryObjectReader.Create(Stream, BufSize);
end;


function TDelphiReader.ReadStr: string;
begin
   Result := GetDriver.ReadStr;
end;

procedure TDelphiReader.Read(var Buf; Count: LongInt);
begin
   GetDriver.Read(Buf, Count);
end;

{ TDelphiWriter }

function TDelphiWriter.GetDriver: TBidirBinaryObjectWriter;
begin
   Result := (Driver as TBidirBinaryObjectWriter);
end;

function TDelphiWriter.GetPosition: LongInt;
begin
   Result := GetDriver.Position;
end;

procedure TDelphiWriter.SetPosition(const AValue: LongInt);
begin
   GetDriver.Position := AValue;
end;

function TDelphiWriter.CreateDriver(Stream: TStream; BufSize: Integer): TAbstractObjectWriter;
begin
   Result := TBidirBinaryObjectWriter.Create(Stream, BufSize);
end;

procedure TDelphiWriter.FlushBuffer;
begin
   GetDriver.FlushBuffer();
end;

procedure TDelphiWriter.Write(const Buf; Count: Longint);
begin
   GetDriver.Write(Buf, Count);
end;

procedure TDelphiWriter.WriteStr(const Value: string);
begin
   GetDriver.WriteStr(Value);
end;

procedure TDelphiWriter.WriteValue(Value: TValueType);
begin
   GetDriver.WriteValue(Value);
end;

end.
