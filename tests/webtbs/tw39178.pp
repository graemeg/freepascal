{ %cpu=x86_64,i386 }
{ %opt=-Cpcoreavx2 -O3 }
{$mode objfpc} {$h+} {$modeswitch advancedrecords} {$modeswitch duplicatelocals}
uses
  cpu;
type
    UintVec3 = record
        x, y, z: uint32;
        class function Make(x, y, z: uint32): UintVec3; static;
        function ToString: string;
        class operator shr(const a: UintVec3; b: uint32): UintVec3;
        class operator div(const a: UintVec3; b: uint32): UintVec3;
        class operator =(const a,b: UintVec3): Boolean;
    end;

    class function UintVec3.Make(x, y, z: uint32): UintVec3;
    begin
        result.x := x;
        result.y := y;
        result.z := z;
    end;

    function UintVec3.ToString: string;
    begin
        WriteStr(result, x, ', ', y, ', ', z);
    end;

    class operator UintVec3.shr(const a: UintVec3; b: uint32): UintVec3;
    begin
        result.x := a.x shr b;
        result.y := a.y shr b;
        result.z := a.z shr b;
    end;

    class operator UintVec3.div(const a: UintVec3; b: uint32): UintVec3;
    begin
        result.x := a.x div b;
        result.y := a.y div b;
        result.z := a.z div b;
    end;

    class operator UintVec3.=(const a,b: UintVec3): Boolean;
    begin
        result := (a.x = b.x) and
                  (a.y = b.y) and
                  (a.z = b.z);
    end;
begin
    if BMI2Support then
      begin
        writeln('div 2: ', (UintVec3.Make(100, 50, 30) div 2).ToString);
        writeln('shr 1: ', (UintVec3.Make(100, 50, 30) shr 1).ToString);
        if not((UintVec3.Make(100, 50, 30) div 2)=(UintVec3.Make(100, 50, 30) shr 1)) then
           halt(1);
        writeln('ok');
      end;
end.
