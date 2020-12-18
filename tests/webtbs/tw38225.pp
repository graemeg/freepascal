{$inline on}
{$mode objfpc}
uses
  classes;

operator - (const A: TPoint): TPoint; inline;
begin
  Result.X := - A.X;
  Result.Y := - A.Y;
end;

operator div(const A: TPoint; ADivisor: Integer): TPoint; inline;
begin
  Result.X := A.X div ADivisor;
  Result.Y := A.Y div ADivisor;
end;


procedure p;
  var
    i1,i2 : longint;
    q1,q2 : int64;
    d2 : dword;
    p1,p2 : TPoint;

  begin
    p2:=-p2 div 2;
    try
      p2:=-p2 div 2;
    except
      p2:=-p2 div 2;
    end;
  end;

begin
end.

