{ %NORUN }

program tw38145b;
{$mode objfpc}{$modeswitch advancedrecords}
type
  generic TMyWrap<T> = record
    Value: T;
    class operator Explicit(const w: TMyWrap): T;
    class operator :=(const w: TMyWrap): T;
  end;

class operator TMyWrap.Explicit(const w: TMyWrap): T;
begin
  Result := w.Value;
end;

class operator TMyWrap.:=(const w: TMyWrap): T;
begin
  Result := w.Value;
end;

type
  //TString = string[255]; //compiles
  TString = string[254]; //not compiles
var
  MySpec: specialize TMyWrap<TString>;
begin
end.
