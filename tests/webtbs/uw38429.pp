unit uw38429;

{$mode objfpc}{$H+}
{$modeswitch advancedrecords}

interface

uses
  SysUtils, Variants;

type
  TMyVar = packed record
    VType: TVarType;
    Dummy1: array[0..2] of Word;
    Dummy2,
    Dummy3: Pointer;
    procedure Init;
  end;

  { TMyVariant }

  TMyVariant = class(TInvokeableVariantType)
    procedure Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean); override;
    procedure Clear(var V: TVarData); override;
    procedure Cast(var Dest: TVarData; const Source: TVarData); override;
    procedure CastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType); override;
  end;

  function MyVarCreate: Variant;

  function varMyVar: TVarType;

implementation

var
  MyVariant: TMyVariant;

function MyVarCreate: Variant;
begin
  VarClear(Result);
  TMyVar(Result).Init;
end;

function VarMyVar: TVarType;
begin
  Result := MyVariant.VarType;
end;

{ TMyVar }

procedure TMyVar.Init;
begin
  VType := VarMyVar;
end;

{ TMyVariant }

procedure TMyVariant.Copy(var Dest: TVarData; const Source: TVarData;
  const Indirect: Boolean);
begin
  Dest := Source;
end;

procedure TMyVariant.Clear(var V: TVarData);
begin
  TMyVar(v).VType := varEmpty;
end;

procedure TMyVariant.Cast(var Dest: TVarData; const Source: TVarData);
begin
  WriteLn('TMyVariant.Cast');
  VarClear(Variant(Dest));
  TMyVar(Dest).Init;
end;

procedure TMyVariant.CastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType);
begin
  WriteLn('TMyVariant.CastTo');
  VarClear(Variant(Dest));
  TVarData(Dest).VType := aVarType;
end;

initialization
  MyVariant := TMyVariant.Create;
finalization
  MyVariant.Free;
end.

