program DestBug;

{$APPTYPE CONSOLE}
{$MODE Delphi}

uses
  Variants, SysUtils;

type
  TSampleVariant = class(TCustomVariantType)
  protected
    procedure Clear(var V: TVarData); override;
    procedure Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean ); override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure DispInvoke(Dest: PVarData; var Source: TVarData; CallDesc: PCallDesc; Params: Pointer); override;
=======
    procedure DispInvoke(Dest: PVarData; const Source: TVarData; CallDesc: PCallDesc; Params: Pointer); override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure DispInvoke(Dest: PVarData; const Source: TVarData; CallDesc: PCallDesc; Params: Pointer); override;
>>>>>>> origin/fixes_2_2
  end;

procedure TSampleVariant.Clear(var V: TVarData);
begin
  V.VType:=varEmpty;
end;

procedure TSampleVariant.Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean);
begin
  if Indirect and VarDataIsByRef(Source) then
    VarDataCopyNoInd(Dest, Source)
  else with Dest do
    VType:=Source.VType;
end;

var
  p : pointer;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TSampleVariant.DispInvoke(Dest: PVarData; var Source: TVarData; CallDesc: PCallDesc; Params: Pointer);
=======
procedure TSampleVariant.DispInvoke(Dest: PVarData; const Source: TVarData; CallDesc: PCallDesc; Params: Pointer);
>>>>>>> graemeg/fixes_2_2
=======
procedure TSampleVariant.DispInvoke(Dest: PVarData; const Source: TVarData; CallDesc: PCallDesc; Params: Pointer);
>>>>>>> origin/fixes_2_2
begin
  Writeln('Dest is 0x', IntToStr(Cardinal(Dest)));
  p:=Dest;
end;

var
  SampleVariant: TSampleVariant;
  v, v1: Variant;

begin
  SampleVariant:=TSampleVariant.Create;
  TVarData(v).VType:=SampleVariant.VarType;
  v.SomeProc;
  if assigned(p) then
    halt(1);
  v1:=v.SomeFunc;
  if not(assigned(p)) then
    halt(1);
  writeln('ok');
end.
