{ this tests the round routine }
program tround;

{$ifndef MACOS}
{$APPTYPE CONSOLE}
{$else}
{$APPTYPE TOOL}
{$endif}

const
  RESULT_ONE = 1235;
  VALUE_ONE = 1234.5678;
  RESULT_CONST_ONE = round(VALUE_ONE);
  RESULT_TWO = -1235;
  VALUE_TWO = -1234.5678;
  RESULT_CONST_TWO = round(VALUE_TWO);
  VALUE_LARGE = 1.5000000000000000E+018;
  RESULT_LARGE= 1500000000000000000;
  RESULT_CONST_LARGE = round(1.5000000000000000E+018);
  VALUE_HUGE = 1.500000549755813888E+018;
  RESULT_HUGE = 1500000549755813888;
  RESULT_CONST_HUGE = round(1.500000549755813888E+018);

 procedure fail;
  begin
    WriteLn('Failed!');
    halt(1);
  end;

procedure test_round_real;
var
 r: real;
 _success : boolean;
 l: longint;
 i64: int64;
Begin
 Write('Round() real testing...');
 _success := true;
 r:=VALUE_ONE;
 if round(r)<>RESULT_ONE then
   _success:=false;
 if round(VALUE_ONE)<>RESULT_ONE then
   _success:=false;
 r:=VALUE_ONE;
 if round(r)<>RESULT_CONST_ONE then
   _success := false;
 r:=VALUE_ONE;
 l:=round(r);
 if l<>RESULT_ONE then
   _success:=false;
 l:=round(VALUE_ONE);
 if l<>RESULT_ONE then
   _success:=false;


 r:=VALUE_TWO;
 if round(r)<>RESULT_TWO then
   _success:=false;
 if round(VALUE_TWO)<>RESULT_TWO then
   _success:=false;
 r:=VALUE_TWO;
 if round(r)<>RESULT_CONST_TWO then
   _success := false;
 r:=VALUE_TWO;
 l:=round(r);
 if l<>RESULT_TWO then
   _success:=false;
 l:=round(VALUE_TWO);
 if l<>RESULT_TWO then
   _success:=false;


  r:=VALUE_LARGE;
  if round(r)<>RESULT_LARGE then
    _success:=false;
  if round(VALUE_LARGE)<>RESULT_LARGE then
    _success:=false;
  r:=VALUE_LARGE;
  if round(r)<>RESULT_CONST_LARGE then
    _success := false;
  r:=VALUE_LARGE;
  i64:=round(r);
  if i64<>RESULT_LARGE then
    _success:=false;
  i64:=round(VALUE_LARGE);
  if i64<>RESULT_LARGE then
    _success:=false; 
 
 
  r:=VALUE_HUGE;
  if round(r)<>RESULT_HUGE then
    _success:=false;
  if round(VALUE_HUGE)<>RESULT_HUGE then
    _success:=false;
  r:=VALUE_HUGE;
  if round(r)<>RESULT_CONST_HUGE then
    _success := false;
  r:=VALUE_HUGE;
  i64:=round(r);
  if i64<>RESULT_HUGE then
    _success:=false;
  i64:=round(VALUE_HUGE);
  if i64<>RESULT_HUGE then
    _success:=false;
 
  if not _success then
   fail;
 WriteLn('Success!');
end;


procedure test_round_single;
var
 r: single;
 _success : boolean;
 l: longint;
Begin
 Write('Round() single testing...');
 _success := true;
 r:=VALUE_ONE;
 if round(r)<>RESULT_ONE then
   _success:=false;
 if round(VALUE_ONE)<>RESULT_ONE then
   _success:=false;
 r:=VALUE_ONE;
 if round(r)<>RESULT_CONST_ONE then
   _success := false;
 r:=VALUE_ONE;
 l:=round(r);
 if l<>RESULT_ONE then
   _success:=false;
 l:=round(VALUE_ONE);
 if l<>RESULT_ONE then
   _success:=false;


 r:=VALUE_TWO;
 if round(r)<>RESULT_TWO then
   _success:=false;
 if round(VALUE_TWO)<>RESULT_TWO then
   _success:=false;
 r:=VALUE_TWO;
 if round(r)<>RESULT_CONST_TWO then
   _success := false;
 r:=VALUE_TWO;
 l:=round(r);
 if l<>RESULT_TWO then
   _success:=false;
 l:=round(VALUE_TWO);
 if l<>RESULT_TWO then
   _success:=false;


 if not _success then
   fail;
 WriteLn('Success!');
end;

procedure test_round_double;
var
 r: double;
 _success : boolean;
 l: longint;
 i64: int64;
Begin
 Write('Round() double testing...');
 _success := true;
 r:=VALUE_ONE;
 if round(r)<>RESULT_ONE then
   _success:=false;
 if round(VALUE_ONE)<>RESULT_ONE then
   _success:=false;
 r:=VALUE_ONE;
 if round(r)<>RESULT_CONST_ONE then
   _success := false;
 r:=VALUE_ONE;
 l:=round(r);
 if l<>RESULT_ONE then
   _success:=false;
 l:=round(VALUE_ONE);
 if l<>RESULT_ONE then
   _success:=false;


 r:=VALUE_TWO;
 if round(r)<>RESULT_TWO then
   _success:=false;
 if round(VALUE_TWO)<>RESULT_TWO then
   _success:=false;
 r:=VALUE_TWO;
 if round(r)<>RESULT_CONST_TWO then
   _success := false;
 r:=VALUE_TWO;
 l:=round(r);
 if l<>RESULT_TWO then
   _success:=false;
 l:=round(VALUE_TWO);
 if l<>RESULT_TWO then
   _success:=false;


  r:=VALUE_LARGE;
  if round(r)<>RESULT_LARGE then
    _success:=false;
  if round(VALUE_LARGE)<>RESULT_LARGE then
    _success:=false;
  r:=VALUE_LARGE;
  if round(r)<>RESULT_CONST_LARGE then
    _success := false;
  r:=VALUE_LARGE;
  i64:=round(r);
  if i64<>RESULT_LARGE then
    _success:=false;
  i64:=round(VALUE_LARGE);
  if i64<>RESULT_LARGE then
    _success:=false; 
 
  
  r:=VALUE_HUGE;
  if round(r)<>RESULT_HUGE then
    _success:=false;
  if round(VALUE_HUGE)<>RESULT_HUGE then
    _success:=false;
  r:=VALUE_HUGE;
  if round(r)<>RESULT_CONST_HUGE then
    _success := false;
  r:=VALUE_HUGE;
  i64:=round(r);
  if i64<>RESULT_HUGE then
    _success:=false;
  i64:=round(VALUE_HUGE);
  if i64<>RESULT_HUGE then
    _success:=false;

 if not _success then
   fail;
 WriteLn('Success!');
end;

{$ifndef SKIP_CURRENCY_TEST}
procedure test_round_currency;
var
 r: currency;
 _success : boolean;
 l: longint;
Begin
 Write('Round() currency testing...');
 _success := true;
 r:=VALUE_ONE;
 if round(r)<>RESULT_ONE then
   _success:=false;
 if round(VALUE_ONE)<>RESULT_ONE then
   _success:=false;
 r:=VALUE_ONE;
 if round(r)<>RESULT_CONST_ONE then
   _success := false;
 r:=VALUE_ONE;
 l:=round(r);
 if l<>RESULT_ONE then
   _success:=false;
 l:=round(VALUE_ONE);
 if l<>RESULT_ONE then
   _success:=false;


 r:=VALUE_TWO;
 if round(r)<>RESULT_TWO then
   _success:=false;
 if round(VALUE_TWO)<>RESULT_TWO then
   _success:=false;
 r:=VALUE_TWO;
 if round(r)<>RESULT_CONST_TWO then
   _success := false;
 r:=VALUE_TWO;
 l:=round(r);
 if l<>RESULT_TWO then
   _success:=false;
 l:=round(VALUE_TWO);
 if l<>RESULT_TWO then
   _success:=false;

 if not _success then
   fail;
 WriteLn('Success!');
end;
{$endif SKIP_CURRENCY_TEST}


Begin
  test_round_real;
  test_round_single;
  test_round_double;
{$ifdef SKIP_CURRENCY_TEST}
  Writeln('Skipping currency test because its not supported by theis compiler');
{$else SKIP_CURRENCY_TEST}
  test_round_currency;
{$endif SKIP_CURRENCY_TEST}
end.
