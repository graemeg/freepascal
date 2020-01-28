{$MODE DELPHI}
{$RANGECHECKS OFF}
{$OVERFLOWCHECKS OFF}

function SplitMix64(var X: UInt64) : UInt64;
var
  Z: UInt64;
begin
  Inc(X, UInt64($9E3779B97F4A7C15));
  Z := (X xor (X shr 30)) * UInt64($BF58476D1CE4E5B9);
  Z := (Z xor (Z shr 27)) * UInt64($94D049BB133111EB);
  Result := Z xor (Z shr 31);
end;

begin
end.
