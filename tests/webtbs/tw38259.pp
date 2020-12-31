{$mode objfpc}
{$inline on}

procedure mymove(var src,dst; len: ptrint); inline;
  begin
    if len<=0 then
      exit;
  end;


function concatansistrings(p1,p2 : pchar;length1,length2 : longint) : pchar;
var
  p : pchar;
begin
  getmem(p,length1+length2+1);
  mymove(p1[0],p[0],length1);
  mymove(p2[0],p[length1],length2+1);
  concatansistrings:=p;
end;

begin
end.
