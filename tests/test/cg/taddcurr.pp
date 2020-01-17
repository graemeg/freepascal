{ %OPT=-Oonofastmath }

{****************************************************************}
{  CODE GENERATOR TEST PROGRAM                                   }
{****************************************************************}
{ NODE TESTED : secondadd() FPU currency type code               }
{****************************************************************}
{ PRE-REQUISITES: secondload()                                   }
{                 secondassign()                                 }
{                 secondtypeconv()                               }
{****************************************************************}
{ DEFINES:                                                       }
{            FPC     = Target is FreePascal compiler             }
{****************************************************************}
{ REMARKS:                                                       }
{                                                                }
{                                                                }
{                                                                }
{****************************************************************}

{ Result is either LOC_FPU or LOC_REFERENCE                     }
{ LEFT NODE (operand) (left operator)                           }
{  LOC_REFERENCE / LOC_MEM                                      }
{  LOC_FPU                                                      }
{ RIGHT NODE (operand)                                          }
{  LOC_FPU                                                      }
{  LOC_REFERENCE / LOC_MEM                                      }
procedure fail;
begin
  WriteLn('Failed!');
  halt(1);
end;


 Procedure CurrencyTestSub;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 Begin
  Write('Currency - Currency test...');
  result := true;
  i:=99.9;
  j:=10.0;
  i:=i-j;
  if trunc(i) <> trunc(89.9) then
    result := false;
  WriteLn('Result (89.9) :',i);
  i:=j-i;
  if trunc(i) <> trunc(-79.9) then
    result := false;
  WriteLn('Result (-79.9) :',i);
  j:=j-10.0;
  if j <> 0.0 then
    result := false;
  WriteLn('Result (0.0) :',j);
  if not result then
    Fail
  else
    WriteLn('Success.');
 end;

 procedure CurrencyTestAdd;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 Begin
   WriteLn('Currency + Currency test...');
   result := true;
   i:= 9;
   i:=i+1.5;
   if trunc(i) <> trunc(10.5) then
     result := false;
   WriteLn('Result (10.5) :',i);
   i := 0.0;
   j := 100.0;
   i := i + j + j + 12.5;
   if trunc(i) <> trunc(212.5) then
     result := false;
   WriteLn('Result (212.5) :',i);
   if not result then
    Fail
   else
    WriteLn('Success.');
 end;


 procedure CurrencyTestmul;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 begin
  WriteLn('Currency * Currency test...');
  result := true;
  i:= 0;
  j:= 0;
  i := i * j * i;
  if trunc(i) <> trunc(0.0) then
    result := false;
  WriteLn('Result (0.0) :',i);
  i := 10.0;
  j := -12.0;
  i := i * j * 10.0;
  if trunc(i) <> trunc(-1200.0) then
    result := false;
  WriteLn('Result (-1200.0) :',i);
  if not result then
    Fail
  else
    WriteLn('Success.');
 end;



 Procedure CurrencyTestDiv;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 Begin
  result := true;
  WriteLn('Currency / Currency test...');
  i:=-99.9;
  j:=10.0;
  i:=i / j;
  if trunc(i) <> trunc(-9.99) then
    result := false;
  if abs(frac(i) - frac(-9.99)) > 1e-9 then
    result := false;
  WriteLn('Result (-9.99) :',i);
  i:=j / i;
  if trunc(i) <> trunc(-1.001) then
    result := false;
  if abs (frac(i) - frac(-1.001)) > 1e-9 then
    result := false;
  WriteLN('Result (-1.001) :',i);
  j:=i / 10.0;
  if trunc(j) <> trunc(-0.1001) then
    result := false;
  if abs(frac(j) - frac(-0.1001)) > 1e-9 then
    result := false;
  WriteLn('Result (-0.1001) :',j);
  if not result then
    Fail
  else
    WriteLn('Success.');
 end;



{ Procedure CurrencyTestComplex;
 var
  i : real;
 Begin
   Write('RESULT SHOULD BE 2.09 :');
   i := 4.4;
   WriteLn(Sqrt(i));
   Write('RESULT SHOULD BE PI :');
   WriteLn(Pi);
   Write('RESULT SHOULD BE 4.0 :');
   WriteLn(Round(3.6));
 end;}


 procedure CurrencyTestequal;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 begin
  result := true;
  Write('Currency = Currency test...');
  i := 1000.0;
  j := 1000.0;
  if not (trunc(i) = trunc(j)) then
    result := false;
  if not (trunc(i) = trunc(1000.0)) then
    result := false;
  if not result then
    Fail
  else
    WriteLn('Success.');
 end;

 procedure CurrencyTestnotequal;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 begin
  result := true;
  Write('Currency <> Currency test...');
  i := 1000.0;
  j := 1000.0;
  if (trunc(i) <> trunc(j)) then
    result := false;
  if (trunc(i) <> trunc(1000.0)) then
    result := false;
  if not result then
    Fail
  else
    WriteLn('Success.');
 end;


 procedure CurrencyTestle;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 begin
  result := true;
  Write('Currency <= Currency test...');
  i := 1000.0;
  j := 1000.0;
  if not (trunc(i) <= trunc(j)) then
    result := false;
  if not (trunc(i) <= trunc(1000.0)) then
    result := false;
  i := 10000.0;
  j := 999.0;
  if trunc(i) < trunc(j) then
    result := false;
  if trunc(i) < trunc(999.0) then
    result := false;
  if not result then
    Fail
  else
    WriteLn('Success.');
 end;

 procedure CurrencyTestge;
 var
  i : Currency;
  j : Currency;
  result : boolean;
 begin
  result := true;
  Write('Currency >= Currency test...');
  i := 1000.0;
  j := 1000.0;
  if not (trunc(i) >= trunc(j)) then
    result := false;
  if not (trunc(i) >= trunc(1000.0)) then
    result := false;
  i := 999.0;
  j := 1000.0;
  if trunc(i) > trunc(j) then
    result := false;
  if trunc(i) > trunc(999.0) then
    result := false;
  if not result then
    Fail
  else
    WriteLn('Success.');
 end;


Begin
 CurrencyTestEqual;
 CurrencyTestNotEqual;
 CurrencyTestLE;
 CurrencyTestGE;
 CurrencyTestSub;
 CurrencyTestAdd;
 CurrencyTestDiv;
 CurrencyTestMul;
{ CurrencyTestComplex;}
end.
