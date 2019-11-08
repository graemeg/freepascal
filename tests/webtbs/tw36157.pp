program WatchesValuePrg;

type
  TEnum = (EnVal1, EnVal2, EnVal3, EnVal4);
  TSet = set of TEnum;
  TBitPackSetArray2 = bitpacked array [0..1, 0..2] of TSet;
const
  gcBitPackSetArray2 : TBitPackSetArray2 = (([EnVal3, EnVal1], [], [EnVal3]), ([],[EnVal1,EnVal2],[EnVal1]));

begin
  if gcBitPackSetArray2[0,0]<>[EnVal3, EnVal1] then
    halt(1);
  if gcBitPackSetArray2[0,2]<>[EnVal3] then
    halt(2);
  if gcBitPackSetArray2[1,1]<>[EnVal1,EnVal2] then
    halt(3);
end.
