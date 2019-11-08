program Project1;
type
  TBitSize = -7..7;
  TFpDbgValueSize = bitpacked record
    Size: Int64;
    BitSize: TBitSize;
  end;

const
  gcFpDbgValueSize: TFpDbgValueSize = (Size: $7FFFFFFF; BitSize: 2);

begin
  writeln(hexstr(gcFpDbgValueSize.Size,16));
  writeln(gcFpDbgValueSize.BitSize);
  if gcFpDbgValueSize.Size<>$7fffffff then
    halt(1);
end.
