program tw38122;

{$mode objfpc}
{$modeswitch advancedrecords}
{$modeswitch typehelpers}

uses
  Math;

type float = double;
     pfloat = ^float;

type  TFloatHelper = type helper for float
        procedure sub (const a: float);
      end;

type TMatrix = record
                 sx,sy: sizeint;
                 procedure Init (x,y: sizeint; content: array of float);
                 function GetAdr (x,y: sizeint): pfloat;
                 procedure print;
                 private
                   data: array of float;
               end;

procedure TFloatHelper.sub (const a: float);
begin
  self := self-a;
end;

function TMatrix.GetAdr (x,y: sizeint): pfloat;
begin
  result := @data[x*sy+y];
end;

procedure TMatrix.Init (x,y: sizeint; content: array of float);
var i: sizeint;
begin
  sx :=x;
  sy :=y;
  Data := nil;
  SetLength (data, sx*sy);
  for i := 0 to sx*sy-1 do data[i] := content[i];
end;

procedure TMatrix.print;
var x,y: sizeint;
begin
  for y := 0 to sy-1 do begin
    writeln;
    for x := 0 to sx-1 do begin
      write (GetAdr(x,y)^:2:2,'  ');
    end;
  end;
  writeln;
end;

var A: TMatrix;
    px: pfloat;
begin
  A.Init (2,2,[1,2,3,4]);
  A.print;
  if not SameValue(A.data[3],4,1e-1) then
    Halt(1);

  A.GetAdr(1,1)^ := 0; //I can set an element like this...
  A.Print;
  if not SameValue(A.data[3],0,1e-1) then
    Halt(2);

  px := A.GetAdr(1,1);
  px^.sub(100);  //and this works as well.
  A.Print;
  if not SameValue(A.data[3],-100,1e-1) then
    Halt(3);

  A.GetAdr(1,1)^.sub(1000); //but that does not change the Matrix !?!
  A.print;
  if not SameValue(A.data[3],-1100,1e-1) then
    Halt(4);
end.
