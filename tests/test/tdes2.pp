{ %fail }
{ %cpu=avr }
{ %norun }
uses
  intrinsics;
var
  data,key : array[0..7] of byte;
begin
  avr_des(data,key,true,100);
end.

  