{ %NORUN }

program tw38145a;
{$mode delphi}
type
  TMyWrap<T> = record
    Value: T;
    class operator Explicit(const w: TMyWrap<T>): T;
    class operator Implicit(const w: TMyWrap<T>): T;
  end;

class operator TMyWrap<T>.Explicit(const w: TMyWrap<T>): T;
begin
  Result := w.Value;
end;

class operator TMyWrap<T>.Implicit(const w: TMyWrap<T>): T;
begin
  Result := w.Value;
end;

type
  //TString = string[255]; //compiles
  TString = string[254]; //not compiles

var
  MySpec: TMyWrap<TString>;
begin
end.
