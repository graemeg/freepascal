{ %NORUN }

program tw24283;

{$mode delphi}{$H+}

type
  TA<T> = record
  end;

  TB<T> = class
  end;

  TC<T> = class(TB<TA<T>>)
  end;

{ ensure that specialization works as well }
var
  t: TC<LongInt>;
begin
end.

