program foo;

{$mode DELPHI}

type
  TRgb = record
    R,G,B : Byte;
  end;
  
  TRgbArray = array of TRgb;
  
  TSomeClass = class
    a: TRgbArray;
    function GetP(Index : integer) : Pointer;
    constructor create;
  public
<<<<<<< HEAD
<<<<<<< HEAD
    property P[Index: Integer]: Pointer read GetP;
=======
    property P[Index: LongInt]: Pointer read GetP;
>>>>>>> graemeg/fixes_2_2
=======
    property P[Index: LongInt]: Pointer read GetP;
>>>>>>> origin/fixes_2_2
  end;
  
var a : TRgbArray;
    c : TSomeClass;

constructor tsomeclass.create;
begin
  setlength(a,2);
  a[0].r:=1;
  a[0].g:=2;
  a[0].b:=3;
  a[1].r:=4;
  a[1].g:=5;
  a[1].b:=6;
end;

function TSomeClass.GetP(Index : integer) : Pointer;
begin
  result := pointer(a);
end;
    
begin
  c := TSomeClass.Create;
  a := TRgbArray(c.P[1]); // Fatal: Internal error 2006111510
  if (length(a)<>2) or
     (a[0].r<>1) or
     (a[0].g<>2) or
     (a[0].b<>3) or
     (a[1].r<>4) or
     (a[1].g<>5) or
     (a[1].b<>6) then
    halt(1);
  c.free;
end.
