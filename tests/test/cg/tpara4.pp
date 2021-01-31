{ This test ensures that a "const TVarData" parameter is passed as a reference.
  This is required for Delphi compatibility as implementers of IVarInvokable or
  inheritors of TInvokableVariantType need to modify the variant data by using
  a pointer to the TVarData because it's passed as const and thus not modifyable
  by itself.
  This behavior is documented in so far as the C++ builder documentation shows
  that the same parameter is implemented as "const&". }

program tpara4;

var
  d: TVarData;

procedure Test(const v: TVarData);
begin
  if @d <> @v then
    Halt(1);
end;

begin
  Test(d);
end.
