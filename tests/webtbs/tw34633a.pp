{$mode delphi}
type tenum = (b0, b1, b2, bMax = high(dword)); 

begin
  writeln(sizeof(tenum));
end.
