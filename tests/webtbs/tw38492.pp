{ %opt=-Sm -dmydefine:=false }

{$mode macpas}
{$setc def := mydefine}
program setcbug;
begin
{$ifc def}
  writeln( 'mydefine is true')
  halt(1);
{$elsec}
  writeln( 'mydefine is false')
{$endc}
end.
