{$mode objfpc}

type
  generic GList<_T> = class
<<<<<<< HEAD
<<<<<<< HEAD
    private
      var
        i : integer;
=======
    var private
      i : integer;
>>>>>>> graemeg/fixes_2_2
=======
    var private
      i : integer;
>>>>>>> origin/fixes_2_2
    function some_func(): integer;
  end;

function GList.some_func(): integer;
begin
  i := -1;
  Result := -1;
end { some_func };


type
  TA = specialize GList<integer>;
var
  A : TA;

begin
  A:=TA.Create;
  if A.some_func<>-1 then
    halt(1);
  writeln('ok');
end.
