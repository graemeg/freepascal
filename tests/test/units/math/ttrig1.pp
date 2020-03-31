program test03;

uses
{$IFDEF CPU86}
  CMem,
{$ENDIF}
  SysUtils,Math;

const
{$if defined(cpuarm) or defined (cpumips) or defined(cpuaarch64)}
  arctan_res = 1E-13;
{$else}
  arctan_res = 1E-14;
{$endif}


 dim = 36;
 MaxFloat = 1.1000000000000000E+4932;

 // from dcc
 sinus : array[1..dim] of double = (
 1.7364817766693035E-0001, 3.4202014332566873E-0001, 5.0000000000000000E-0001, 6.4278760968653933E-0001,
 7.6604444311897804E-0001, 8.6602540378443865E-0001, 9.3969262078590838E-0001, 9.8480775301220806E-0001,
 1.0000000000000000E+0000, 9.8480775301220806E-0001, 9.3969262078590838E-0001, 8.6602540378443865E-0001,
 7.6604444311897804E-0001, 6.4278760968653933E-0001, 5.0000000000000000E-0001, 3.4202014332566873E-0001,
 1.7364817766693035E-0001,-5.4210108624275222E-0020,-1.7364817766693035E-0001,-3.4202014332566873E-0001,
-5.0000000000000000E-0001,-6.4278760968653933E-0001,-7.6604444311897804E-0001,-8.6602540378443865E-0001,
-9.3969262078590838E-0001,-9.8480775301220806E-0001,-1.0000000000000000E+0000,-9.8480775301220806E-0001,
-9.3969262078590838E-0001,-8.6602540378443865E-0001,-7.6604444311897804E-0001,-6.4278760968653933E-0001,
-5.0000000000000000E-0001,-3.4202014332566873E-0001,-1.7364817766693035E-0001, 1.0842021724855044E-0019
 );

var
  i : integer;
  Delta,Ref,Value : Double;
  Ref2,Value2,Dummy : Float;

begin
for i:=1 to dim do
  begin
  // Generate constant array above output
  write(sin(i*10/float(180.0)*pi),',');
  if i mod 4 = 0 then writeln;
  end;


writeln('Testing SIN');
for i:=1 to dim do
  begin
  Ref:=sinus[i];
  Value:=sin(i*10/float(180.0)*pi);
  Delta := Value - Ref;
  if Abs(Delta) > 1E-15 then
    begin
      writeln('  Error for Sin(',i*10,') was:',Value,' should be:',Ref) ;
      halt(1);
    end;
  end;


writeln('Testing COS');
for i:=1 to dim do
  begin
  Ref := sin(pi/2-i*10/float(180)*pi);
  Value := cos(i*10/float(180)*pi);
  Delta := Value - Ref;
  if Abs(Delta) > 1E-15 then
    begin
      writeln('  Error for Cos(',i*10,') was:',Value,' should be:',Ref) ;
      halt(1);
    end;
  end;


writeln('Testing TAN');
for i:=1 to dim do
  begin
{    if i=9 then Ref := MaxFloat
    else if i=27 then Ref := -Maxfloat
    else } Ref:=sin(i*10/float(180)*pi)/cos(i*10/float(180)*pi);
    Value := tan(i*10/float(180)*pi);
    Delta := Value - Ref;
    { Test modifed to cope with different rounding for Win64 PM }
    if (Abs(Delta) > 1E-15) or
       ((abs(Value)+abs(ref)>1) and 
        (Abs(Delta / (abs(Value)+abs(ref))) > 1E-15)) then
      begin
        writeln('  Error for Tan(',i*10,') was:',Value,' should be:',Ref) ;
        halt(1);
      end;
  end;


writeln('Testing ARCTAN...');
for i:=1 to 8 do
  begin
  Ref := i*10;
  Value := arctan(tan(i*10/float(180)*pi))/pi*float(180);
  Delta := Value - Ref;
  if Abs(Delta) > arctan_res then
    begin
      writeln('  Error for ArcTan(',i*10,') was:',Value,' should be:',Ref);
      halt(1);
    end;
  end;


for i:=-1 downto -8 do
  begin
  Ref := i*10;
  Value := arctan(tan(i*10/float(180)*pi))/pi*float(180);
  Delta := Value - Ref;
  if Abs(Delta) > arctan_res then
    begin
      writeln('  Error for ArcTan(',i*10,') was:',Value,' should be:',Ref);
      halt(1);
    end;
  end;

writeln('Testing SINCOS');
for i:=1 to dim do
  begin
  sincos(pi/2-i*10/float(180)*pi,Ref2,Dummy);
  sincos(i*10/float(180)*pi,Dummy,Value2);
  Delta := Value2 - Ref2;
  if Abs(Delta) > 1E-15 then
    begin
      writeln('  Error for Cos(',i*10,') was:',Value2,' should be:',Ref2) ;
      halt(1);
    end;
  end;


writeln('Tan +/- 90 deg test:');
writeln('tan(89.999):',tan(89.999/float(180)*pi));
writeln('tan(90.000):',tan(90.000/float(180)*pi));

writeln('tan(-89.999):',tan(-89.999/float(180)*pi));
writeln('tan(-90.000):',tan(-90.000/float(180)*pi));

writeln('ArcTan2 kwadrants:');
writeln('Kwadrant 1 ( 1, 1):',arctan2( 1, 1)/pi*float(180):5:1,' deg');
writeln('Kwadrant 2 (-1, 1):',arctan2( 1,-1)/pi*float(180):5:1,' deg');
writeln('Kwadrant 3 (-1,-1):',arctan2(-1,-1)/pi*float(180):5:1,' deg');
writeln('Kwadrant 4 ( 1,-1):',arctan2(-1, 1)/pi*float(180):5:1,' deg');

writeln('ArcTan2 special cases:');
writeln('Kwadrant X ( 0, 0):',arctan2( 0, 0)/pi*float(180):5:1,' deg');
writeln('Kwadrant 1 ( 1, 0):',arctan2( 0, 1)/pi*float(180):5:1,' deg');
writeln('Kwadrant 2 ( 0, 1):',arctan2( 1, 0)/pi*float(180):5:1,' deg');
writeln('Kwadrant 3 (-1, 0):',arctan2( 0,-1)/pi*float(180):5:1,' deg');
writeln('Kwadrant 4 ( 0,-1):',arctan2(-1, 0)/pi*float(180):5:1,' deg');



end.
