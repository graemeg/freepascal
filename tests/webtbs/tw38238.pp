program tw38238;

{$mode objfpc}
{$modeswitch advancedrecords}

type
  TCallback = procedure(AValue: longint) of object;

  TRec = record
    Clb: TCallback;
    procedure AddCallback(ACallback: TCallback);
    procedure TriggerCallback(AValue: longint);
  end;

  TRec2 = record
    Value: longint;
    Rec: TRec;
    procedure CLB(AValue: longint);
    procedure InitStuff;
  end;

procedure TRec.AddCallback(ACallback: TCallback);
begin
  Clb:=ACallback;
end;

procedure TRec.TriggerCallback(AValue: longint);
begin
  if assigned(Clb) then
    Clb(AValue);
end;

procedure TRec2.CLB(AValue: longint);
begin
  Value:=AValue;
end;

procedure TRec2.InitStuff;
begin
  Rec.AddCallback(@CLB);
end;

var
  Rec1, Rec2: TRec2;
begin
  Rec1.InitStuff;
  Rec2.InitStuff;

  Rec1.Rec.TriggerCallback(1234);
  Rec2.Rec.TriggerCallback($0943);

  if Rec1.Value<>1234 then
    Halt(1);
  if Rec2.Value<>$0943 then
    Halt(2);
end.
