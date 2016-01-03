{$IFDEF FPC}
{$MODE OBJFPC}
{$ENDIF}

program testreg1;
uses
<<<<<<< HEAD
<<<<<<< HEAD
   oldregexpr;
=======
   regexpr;
>>>>>>> graemeg/fixes_2_2
=======
   regexpr;
>>>>>>> origin/fixes_2_2

var
   r         : tregexprengine;
   index,len : longint;
   S         : String;
   Dest	     : AnsiString;

procedure do_error(i : longint);

  begin
     writeln('error near ',i,' index: ',index,' len: ',len);
     halt(1);
  end;

var
 initok: boolean;
begin
   writeln('*** Testing unit regexpr ***');

<<<<<<< HEAD
<<<<<<< HEAD
   { runtime error test }
    initok:=GenerateRegExprEngine('[o]{1,2}',[],r);
    if not initok then
      do_error(1);
    if not(RegExprPos(r,'book',index,len)) or
      (index<>1) or (len<>2) then
      do_error(1);
    // if it has bug, error  An unhandled exception when r.Free
    DestroyregExprEngine(r); // bug:Test for rcClear

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   writeln('*** Searching tests ***');
   { basic tests }

   initok:=GenerateRegExprEngine('.*',[],r);
   if not initok then
<<<<<<< HEAD
<<<<<<< HEAD
     do_error(50);
   if not(RegExprPos(r,'CXXXX',index,len)) or
     (index<>0) or (len<>5) then
     do_error(51);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('\t\t',[],r);
   if not initok then
     do_error(52);
   if not(RegExprPos(r,'a'+#9+#9+'b'+'\t\t',index,len)) or
     (index<>1) or (len<>2) then
     do_error(52);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('\t',[],r);
   if not initok then
     do_error(53);
   if not(RegExprPos(r,'a'+#9+#9+'b'+'\t\t',index,len)) or
     (index<>1) or (len<>1) then
     do_error(53);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('\w',[],r);
   if not initok then
     do_error(54);
   if not(RegExprPos(r,'- abc \w',index,len)) or
     (index<>2) or (len<>1) then
     do_error(54);
=======
=======
>>>>>>> origin/fixes_2_2
     do_error(90);
   if not(RegExprPos(r,'CXXXX',index,len)) or
     (index<>0) or (len<>5) then
     do_error(91);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   DestroyregExprEngine(r);

   { java package name }
   initok:=GenerateRegExprEngine('[A-Za-z]+([.][0-9A-Za-z]+)*',[],r);
   if not initok then
     do_error(92);
   if not(RegExprPos(r,'CXXXX',index,len)) or
     (index<>0) or (len<>5) then
     do_error(92);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('[A-Za-z]+([.][0-9A-Za-z]+)*',[],r);
   if not initok then
     do_error(92);
   if not(RegExprPos(r,'CXXXX.A',index,len)) or
     (index<>0) or (len<>7) then
     do_error(92);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('[A-Za-z]+([.][0-9A-Za-z]+)*',[],r);
   if not initok then
     do_error(92);
   if not(RegExprPos(r,'CXXXX.A9Package',index,len)) or
     (index<>0) or (len<>15) then
     do_error(92);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('[A-Za-z]+([.][0-9A-Za-z]+)*',[],r);
   if not initok then
     do_error(92);
   if not(RegExprPos(r,'1CXXXX.A9Package',index,len)) or
     (index<>1) or (len<>15) then
     do_error(92);
   DestroyregExprEngine(r);

   { singleline }

   initok:=GenerateRegExprEngine('^TEST1',[],r);
   if not initok then
     do_error(101);
   if (RegExprPos(r,'THISISATEST1TEST1THIS',index,len)) then
     do_error(101);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('^TEST1(ANOTHER)',[],r);
   if not initok then
     do_error(102);
   if (RegExprPos(r,'THISISATEST1ANOTHER',index,len)) then
     do_error(102);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('^TEST1(ANOTHER)',[],r);
   if not initok then
     do_error(103);
   if not(RegExprPos(r,'TEST1ANOTHER',index,len)) or
     (index<>0) or (len<>12) then
     do_error(103);
   DestroyregExprEngine(r);

   { multiline }

   { UNIX Newline }
   initok:=GenerateRegExprEngine('^TEST1',[ref_multiline],r);
   if not initok then
     do_error(120);
   if not(RegExprPos(r,'THISISATEST1'#10'TEST12',index,len)) or
     (index<>13) or (len<>5) then
     do_error(120);
   DestroyregExprEngine(r);

   { Apple Newline }
   initok:=GenerateRegExprEngine('^TEST1',[ref_multiline],r);
   if not initok then
     do_error(121);
   if not(RegExprPos(r,'THISISATEST1'#13'TEST12',index,len)) or
     (index<>13) or (len<>5) then
     do_error(121);
   DestroyregExprEngine(r);

   { DOS Newline }
   initok:=GenerateRegExprEngine('^TEST1',[ref_multiline],r);
   if not initok then
     do_error(122);
   if not(RegExprPos(r,'THISISATEST1'#13#10'TEST12',index,len)) or
     (index<>14) or (len<>5) then
     do_error(122);
   DestroyregExprEngine(r);

   { IBM Mainframe Newline }
   initok:=GenerateRegExprEngine('^TEST1',[ref_multiline],r);
   if not initok then
     do_error(123);
   if not(RegExprPos(r,'THISISATEST1'#$85'TEST12',index,len)) or
     (index<>13) or (len<>5) then
     do_error(123);
   DestroyregExprEngine(r);

   { Some weird cases }
   initok:=GenerateRegExprEngine('^TEST1',[ref_multiline],r);
   if not initok then
     do_error(124);
   if not(RegExprPos(r,#13#10#13#10'TEST12',index,len)) or
     (index<>4) or (len<>5) then
     do_error(124);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('^TEST1',[ref_multiline],r);
   if not initok then
     do_error(125);
   if RegExprPos(r,#13#10#13#10'F',index,len) then
     do_error(125);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('^TEST1(ANOTHER)',[ref_multiline],r);
   if not initok then
     do_error(102);
   if (RegExprPos(r,'THISISATEST1ANOTHER',index,len)) then
     do_error(102);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('^TEST1(ANOTHER)',[],r);
   if not initok then
     do_error(103);
   if not(RegExprPos(r,'TEST1ANOTHER',index,len)) or
     (index<>0) or (len<>12) then
     do_error(103);
   DestroyregExprEngine(r);

   { END OF LINE CASES }
   initok:=GenerateRegExprEngine('TEST1$',[],r);
   if not initok then
     do_error(101);
   if (RegExprPos(r,'THISISATEST1TEST1THIS',index,len)) then
     do_error(101);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('TEST1(ANOTHER)$',[],r);
   if not initok then
     do_error(102);
   if not(RegExprPos(r,'!TEST1ANOTHER',index,len)) or
     (index<>1) or (len<>12) then
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('TEST1(ANOTHER)$',[],r);
   if not initok then
     do_error(102);
   if not(RegExprPos(r,'!TEST1ANOTHERFOOBARTEST1ANOTHER',index,len)) or
     (index<>19) or (len<>12) then
   DestroyregExprEngine(r);

   { UNIX Newline }
   initok:=GenerateRegExprEngine('TEST1$',[ref_multiline],r);
   if not initok then
     do_error(120);
   if not(RegExprPos(r,'THISISATEST1'#10'TEST12',index,len)) or
     (index<>7) or (len<>5) then
     do_error(120);
   DestroyregExprEngine(r);

   { Apple Newline }
   initok:=GenerateRegExprEngine('TEST1$',[ref_multiline],r);
   if not initok then
     do_error(121);
   if not(RegExprPos(r,'THISISATEST1'#13'TEST12',index,len)) or
     (index<>7) or (len<>5) then
     do_error(121);
   DestroyregExprEngine(r);

   { DOS Newline }
   initok:=GenerateRegExprEngine('TEST1$',[ref_multiline],r);
   if not initok then
     do_error(122);
   if not(RegExprPos(r,'THISISATEST1'#13#10'TEST12',index,len)) or
     (index<>7) or (len<>5) then
     do_error(122);
   DestroyregExprEngine(r);

   { IBM Mainframe Newline }
   initok:=GenerateRegExprEngine('TEST1$',[ref_multiline],r);
   if not initok then
     do_error(123);
   if not(RegExprPos(r,'THISISATEST1'#$85'TEST12',index,len)) or
     (index<>7) or (len<>5) then
     do_error(123);
   DestroyregExprEngine(r);

   { Some weird cases }
   initok:=GenerateRegExprEngine('TEST1$',[ref_multiline],r);
   if not initok then
     do_error(124);
   if not(RegExprPos(r,#13#10#13#10'TEST1'#13#10,index,len)) or
     (index<>4) or (len<>5) then
     do_error(124);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('TEST1$',[ref_multiline],r);
   if not initok then
     do_error(125);
   if RegExprPos(r,#13#10#13#10'F'#13#10#13#10,index,len) then
     do_error(125);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('TEST1(ANOTHER)$',[ref_multiline],r);
   if not initok then
     do_error(102);
   if (RegExprPos(r,'THISISATEST1ANOTHERFOO',index,len)) then
     do_error(102);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('TEST1(ANOTHER)$',[],r);
   if not initok then
     do_error(103);
   if not(RegExprPos(r,'TEST1ANOTHER',index,len)) or
     (index<>0) or (len<>12) then
     do_error(103);
   DestroyregExprEngine(r);

   { start and end of string handling }
   initok:=GenerateRegExprEngine('^TEST1(ANOTHER)$',[],r);
   if not initok then
     do_error(103);
   if not(RegExprPos(r,'TEST1ANOTHER',index,len)) or
     (index<>0) or (len<>12) then
     do_error(103);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('^TEST1(ANOTHER)$',[],r);
   if not initok then
     do_error(103);
   if RegExprPos(r,'FOOTEST1ANOTHER',index,len) then
     do_error(103);
   DestroyregExprEngine(r);


   (* {n,} tests *)
   initok:=GenerateRegExprEngine('(AZ){0,}',[],r);
   if not initok then
     do_error(700);
   if not(RegExprPos(r,'C',index,len)) or
     (index<>0) or (len<>0) then
     do_error(700);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ){0,}',[],r);
   if not initok then
     do_error(701);
   if not(RegExprPos(r,'AZ',index,len)) or
     (index<>0) or (len<>2) then
     do_error(701);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ){0,}',[],r);
   if not initok then
     do_error(702);
   if not(RegExprPos(r,'AZAZ',index,len)) or
     (index<>0) or (len<>4) then
     do_error(702);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ){0,}',[],r);
   if not initok then
     do_error(703);
   if not(RegExprPos(r,'CatAZAZ',index,len)) or
     (index<>0) or (len<>7) then
     do_error(703);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('C(AZ){3,}',[],r);
   if not initok then
     do_error(704);
   if (RegExprPos(r,'AZAZAZ',index,len)) then
     do_error(704);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ){3,}',[],r);
   if not initok then
     do_error(705);
   if not(RegExprPos(r,'BCatAZAZAZDABCD',index,len)) or
     (index<>1) or (len<>9) then
     do_error(705);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ){2,}Q',[],r);
   if not initok then
     do_error(705);
   if not(RegExprPos(r,'BCatAZAZAZAZQDABCD',index,len)) or
     (index<>1) or (len<>12) then
     do_error(705);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('CatAZ{0,}',[],r);
   if not initok then
     do_error(706);
   if RegExprPos(r,'BCatDAZZZBCD',index,len) then
     do_error(706);
   DestroyregExprEngine(r);

   (* {n} tests *)
   initok:=GenerateRegExprEngine('Cat(AZ){3}',[],r);
   if not initok then
     do_error(715);
   if not(RegExprPos(r,'BCatAZAZAZDABCD',index,len)) or
     (index<>1) or (len<>9) then
     do_error(715);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('CatAz{5}',[],r);
   if not initok then
     do_error(716);
   if not(RegExprPos(r,'BCatAzzzzzDABCD',index,len)) or
     (index<>1) or (len<>9) then
     do_error(716);

   initok:=GenerateRegExprEngine('CatAz{5}',[],r);
   if not initok then
     do_error(717);
   if RegExprPos(r,'BCatDAzizzzzHello',index,len) then
     do_error(717);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('CatAz{0}',[],r);
   if not initok then
     do_error(718);
   if RegExprPos(r,'BCatDAzizzzzHello',index,len) then
     do_error(718);
   DestroyregExprEngine(r);

<<<<<<< HEAD
<<<<<<< HEAD
   initok:=GenerateRegExprEngine('o{2}',[],r);
   if not initok then
     do_error(719);
   if not(RegExprPos(r,'book',index,len)) or
     (index<>1) or (len<>2) then
     do_error(719);
   DestroyregExprEngine(r);

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   (* {n,m} tests *)
   initok:=GenerateRegExprEngine('Cat(AZ){1,3}',[],r);
   if not initok then
     do_error(725);
   if not(RegExprPos(r,'BCatAZAZAZDABCD',index,len)) or
     (index<>1) or (len<>9) then
     do_error(725);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ){1,3}',[],r);
   if not initok then
     do_error(725);
   if not(RegExprPos(r,'BCatAZAZDABCD',index,len)) or
     (index<>1) or (len<>7) then
     do_error(725);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('CatAz{1,5}',[],r);
   if not initok then
     do_error(726);
   if not(RegExprPos(r,'BCatAzzzzzzzzzzDABCD',index,len)) or
     (index<>1) or (len<>9) then
     do_error(726);

   initok:=GenerateRegExprEngine('CatAz{1,1}',[],r);
   if not initok then
     do_error(727);
   if not(RegExprPos(r,'BCatAzzzzzzzzzzDABCD',index,len)) or
     (index<>1) or (len<>5) then
     do_error(727);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('CatAz{3,4}',[],r);
   if not initok then
     do_error(728);
   if not(RegExprPos(r,'BCatAzzzzzzzzzzDABCD',index,len)) or
     (index<>1) or (len<>8) then
     do_error(728);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('CatAz{0,0}',[],r);
   if not initok then
     do_error(729);
   if RegExprPos(r,'BCatDAzizzzzHello',index,len) then
     do_error(729);
   DestroyregExprEngine(r);

<<<<<<< HEAD
<<<<<<< HEAD
   initok:=GenerateRegExprEngine('o{2,2}',[],r);
   if not initok then
     do_error(730);
   if not(RegExprPos(r,'book',index,len)) or
     (index<>1) or (len<>2) then
     do_error(730);
   DestroyregExprEngine(r);

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

   { ()* tests }
   initok:=GenerateRegExprEngine('(AZ)*',[],r);
   if not initok then
     do_error(800);
   if not(RegExprPos(r,'C',index,len)) or
     (index<>0) or (len<>0) then
     do_error(800);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ)*',[],r);
   if not initok then
     do_error(801);
   if not(RegExprPos(r,'AZ',index,len)) or
     (index<>0) or (len<>2) then
     do_error(801);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ)*',[],r);
   if not initok then
     do_error(802);
   if not(RegExprPos(r,'AZAZ',index,len)) or
     (index<>0) or (len<>4) then
     do_error(802);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ)*',[],r);
   if not initok then
     do_error(803);
   if not(RegExprPos(r,'CatAZAZ',index,len)) or
     (index<>0) or (len<>7) then
     do_error(803);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('C(AZ)*',[],r);
   if not initok then
     do_error(804);
   if (RegExprPos(r,'AZAZ',index,len)) then
     do_error(804);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ)*',[],r);
   if not initok then
     do_error(805);
   if not(RegExprPos(r,'BCatAZAZDABCD',index,len)) or
     (index<>1) or (len<>7) then
     do_error(805);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ)*',[],r);
   if not initok then
     do_error(806);
   if not(RegExprPos(r,'BCatDABCD',index,len)) or
     (index<>1) or (len<>3) then
     do_error(806);
   DestroyregExprEngine(r);


   { ()+ tests }
   initok:=GenerateRegExprEngine('(AZ)+',[],r);
   if not initok then
     do_error(850);
   if (RegExprPos(r,'C',index,len)) then
     do_error(850);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ)+',[],r);
   if not initok then
     do_error(851);
   if not(RegExprPos(r,'AZ',index,len)) or
     (index<>0) or (len<>2) then
     do_error(851);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ)+',[],r);
   if not initok then
     do_error(852);
   if not(RegExprPos(r,'AZAZ',index,len)) or
     (index<>0) or (len<>4) then
     do_error(852);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ)+',[],r);
   if not initok then
     do_error(853);
   if not(RegExprPos(r,'CatAZAZ',index,len)) or
     (index<>0) or (len<>7) then
     do_error(853);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('C(AZ)+',[],r);
   if not initok then
     do_error(854);
   if (RegExprPos(r,'AZAZ',index,len)) then
     do_error(854);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('Cat(AZ)+',[],r);
   if not initok then
     do_error(855);
   if not(RegExprPos(r,'BCatAZAZDABCD',index,len)) or
     (index<>1) or (len<>7) then
     do_error(855);
   DestroyregExprEngine(r);

   { ()? tests }

   initok:=GenerateRegExprEngine('(AZ)?',[],r);
   if not initok then
     do_error(900);
   if not(RegExprPos(r,'C',index,len)) or
     (index<>0) or (len<>0) then
     do_error(900);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ)?',[],r);
   if not initok then
     do_error(901);
   if not(RegExprPos(r,'AZ',index,len)) or
     (index<>0) or (len<>2) then
     do_error(901);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('(AZ)?',[],r);
   if not initok then
     do_error(902);
   if not(RegExprPos(r,'AZAZ',index,len)) or
     (index<>0) or (len<>2) then
     do_error(902);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('Cat(AZ)?',[],r);
   if not(RegExprPos(r,'CatAZAZ',index,len)) or
     (index<>0) or (len<>5) then
     do_error(903);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('C(AZ)?',[],r);
   if (RegExprPos(r,'AZAZ',index,len)) then
     do_error(904);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('Cat(AZ)?',[],r);
   if not(RegExprPos(r,'BCatAZAZDABCD',index,len)) or
     (index<>1) or (len<>5) then
     do_error(905);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('Cat(AZ)?',[],r);
   if not(RegExprPos(r,'BCatDABCD',index,len)) or
     (index<>1) or (len<>3) then
     do_error(906);
   DestroyregExprEngine(r);


   { Character classes tests }

   GenerateRegExprEngine('[A-Z]',[],r);
   if not(RegExprPos(r,'234578923457823659GHJK38',index,len)) or
     (index<>18) or (len<>1) then
     do_error(1000);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('[A-Z]*',[],r);
   if not(RegExprPos(r,'2345ARTZU38',index,len)) or
     (index<>0) or (len<>0) then
     do_error(1002);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('[A-Z]+',[],r);
   if not(RegExprPos(r,'234578923457823659ARTZU38',index,len)) or
     (index<>18) or (len<>5) then
     do_error(1003);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('[A-Z][A-Z]*',[],r);
   if not(RegExprPos(r,'234578923457823659ARTZU38',index,len)) or
     (index<>18) or (len<>5) then
     do_error(1004);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('[A-Z][A-Z]?',[],r);
   if not(RegExprPos(r,'234578923457823659ARTZU38',index,len)) or
     (index<>18) or (len<>2) then
     do_error(1005);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('[^\d]+',[],r);
   if not(RegExprPos(r,'234578923457823659ARTZU38',index,len)) or
     (index<>18) or (len<>5) then
     do_error(1006);
   DestroyregExprEngine(r);

   { test chaining }

   GenerateRegExprEngine('[A-Z][A-Z]?[A-Z]',[],r);
   if not(RegExprPos(r,'234578923457823659ARTZU38',index,len)) or
     (index<>18) or (len<>3) then
     do_error(1007);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('[A-Z][A-Z]*[0-9]',[],r);
   if not(RegExprPos(r,'234578923457823659ARTZU38',index,len)) or
     (index<>18) or (len<>6) then
     do_error(1008);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('[A-Z]+[0-9]',[],r);
   if not(RegExprPos(r,'234578923457823659ARTZU38',index,len)) or
     (index<>18) or (len<>6) then
     do_error(1009);
   DestroyregExprEngine(r);

   { case insensitive: }

   GenerateRegExprEngine('[A-Z]',[ref_caseinsensitive],r);
   if not(RegExprPos(r,'234578923457823659a38',index,len)) or
     (index<>18) or (len<>1) then
     do_error(1100);
   DestroyregExprEngine(r);

   { case insensitive: }
   GenerateRegExprEngine('[a-z]',[ref_caseinsensitive],r);
   if not(RegExprPos(r,'234578923457823659A38',index,len)) or
     (index<>18) or (len<>1) then
     do_error(1101);
   DestroyregExprEngine(r);

   { with parenthsis }
   GenerateRegExprEngine('(foo)1234',[],r);
   if not(RegExprPos(r,'1234   foo1234XXXX',index,len)) or
     (index<>7) or (len<>7) then
     do_error(1200);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('(((foo)))1234',[],r);
   if not(RegExprPos(r,'1234   foo1234XXXX',index,len)) or
     (index<>7) or (len<>7) then
     do_error(1201);
   DestroyregExprEngine(r);

   GenerateRegExprEngine('(foo)(1234)',[],r);
   if not(RegExprPos(r,'1234   foo1234XXXX',index,len)) or
     (index<>7) or (len<>7) then
     do_error(1202);
   DestroyregExprEngine(r);

   { test real backtracking }

   r:=GenerateRegExprEngine('nofoo|foo',[]);
   if not(RegExprPos(r,'1234   foo1234XXXX',index,len)) or
     (index<>7) or (len<>3) then
     do_error(1300);
   DestroyregExprEngine(r);

  GenerateRegExprEngine('abc\(123\)$',[],r);
  if not (RegExprPos(r,'1234 abc(123)', index, len)) or
         (index <> 5) or (len <> 8) then
    do_error (1400);
  DestroyregExprEngine(r);

  GenerateRegExprEngine('^\t$',[ref_singleline],r);
  if not (RegExprPos(r,#9, index, len)) or
         (index <> 0) or (len <> 1) then
    do_error (1401);
  DestroyregExprEngine(r);

  GenerateRegExprEngine('^\n$',[ref_singleline],r);
  if not (RegExprPos(r,#10, index, len)) or
         (index <> 0) or (len <> 1) then
    do_error (1402);
  DestroyregExprEngine(r);

  GenerateRegExprEngine('^\f$',[ref_singleline],r);
  if not (RegExprPos(r,#12, index, len)) or
         (index <> 0) or (len <> 1) then
    do_error (1403);
  DestroyregExprEngine(r);

  GenerateRegExprEngine('^\r$',[ref_singleline],r);
  if not (RegExprPos(r,#13, index, len)) or
         (index <> 0) or (len <> 1) then
    do_error (1404);
  DestroyregExprEngine(r);

  GenerateRegExprEngine('^\a$',[ref_singleline],r);
  if not (RegExprPos(r,#7, index, len)) or
         (index <> 0) or (len <> 1) then
    do_error (1405);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('^(([^:/?#]+):)',[],r);
  if not (RegExprPos(r,'http:',index, len)) or
         (index <> 0) or (len <> 5) then
    do_error (1406);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('^(([^:/?#]+):)?(//([^/?#]*))?',[],r);
  if not initok then
     do_error(1407);
  if not (RegExprPos(r,'http://www.myurl.com',index, len)) or
         (index <> 0) or (len <> 20) then
    do_error (1407);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\?([^#]*))?(#(.*))?',[],r);
  if not initok then
     do_error(1408);
  if not (RegExprPos(r,'http://www.myurl.com',index, len)) or
         (index <> 0) or (len <> 20) then
    do_error (1408);


   writeln('*** Escaping tests ***');
   s := '^Hello World \.  [a-z] \D { } |() ?a*.*\\ 1 $';
   if RegExprEscapeStr(s)<>'\^Hello World \\\.  \[a\-z\] \\D \{ \} \|\(\) \?a\*\.\*\\\\ 1 \$' then
     do_error(1450);

   writeln('*** More search tests ***');

  initok:=GenerateRegExprEngine('((nofoo)|(foo))1234',[],r);
  if not initok then
     do_error(1501);
   if not(RegExprPos(r,'1234   nofoo1234XXXX',index,len)) or
     (index<>7) or (len<>9) then
     do_error(1501);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('((nofoo)|(foo)|(anotherbar))1234',[],r);
   if not initok then
     do_error(1502);
   if not(RegExprPos(r,'anotherbar1234XXXX',index,len)) or
     (index<>0) or (len<>14) then
     do_error(1502);
   DestroyregExprEngine(r);


   initok:=GenerateRegExprEngine('((nofoo)|(foo)|(anotherfoo))1234',[],r);
   if not initok then
     do_error(1503);
   if not(RegExprPos(r,'1234   anotherfoo1234XXXX',index,len)) or
     (index<>7) or (len<>14) then
     do_error(1503);
   DestroyregExprEngine(r);

   initok:=GenerateRegExprEngine('(nofoo1234)|(foo1234)',[],r);
   if not initok then
     do_error(1504);
   if not(RegExprPos(r,'1234   foo1234XXXX',index,len)) or
     (index<>7) or (len<>7) then
     do_error(1504);
   DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('((foo)|(nofoo))1234',[],r);
  if not initok then
     do_error(1505);
   if not(RegExprPos(r,'1234   nofoo1234XXXX',index,len)) or
     (index<>7) or (len<>9) then
     do_error(1505);
   DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('\.localhost$',[],r);
  if not initok then
     do_error(1506);
   if not(RegExprPos(r,'exsample.localhost',index,len)) or
     (index<>8) or (len<>10) then
     do_error(1506);
   DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('[^e]\.localhost$',[],r);
  if not initok then
     do_error(1507);
   if RegExprPos(r,'exsample.localhost',index,len) then
     do_error(1507);
   DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('.*[^e]\.localhost$',[],r);
  if not initok then
     do_error(1508);
   if RegExprPos(r,'exsample.localhost',index,len) then
     do_error(1508);
   DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('x.*[^e]\.localhost$',[],r);
  if not initok then
     do_error(1509);
   if RegExprPos(r,'exsample.localhost',index,len) then
     do_error(1509);
   DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('(nofoo|foo)1234',[],r);
  if not initok then
     do_error(1500);
   if not(RegExprPos(r,'1234   nofoo1234XXXX',index,len)) or
     (index<>7) or (len<>9) then
     do_error(1500);
   DestroyregExprEngine(r);

   r:=GenerateRegExprEngine('(nofoo|foo|anotherfoo)1234',[]);
   if not(RegExprPos(r,'1234   nofoo1234XXXX',index,len)) or
     (index<>7) or (len<>9) then
     do_error(1009);
   DestroyregExprEngine(r);

   r:=GenerateRegExprEngine('nofoo1234|foo1234',[]);
   if {(r.data=nil) or} not(RegExprPos(r,'1234   foo1234XXXX',index,len)) or
     (index<>7) or (len<>7) then
     do_error(1010);
   DestroyregExprEngine(r);

  { *************************************************************************
                              replacement tests
   ************************************************************************* }
  writeln('*** Replacement tests ***');

  initok:=GenerateRegExprEngine('fa',[],r);
  if not initok then
    do_error(2000);
  if (RegExprReplaceAll(r,'asdfasdf','',Dest)<>1) or
    (Dest<>'asdsdf') then
    do_error(2001);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('fa',[],r);
  if not initok then
    do_error(2002);
  if (RegExprReplaceAll(r,'fa','',Dest)<>1) or
    (Dest<>'') then
    do_error(2003);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('fa',[],r);
  if not initok then
    do_error(2004);
  if RegExprReplaceAll(r,'','',Dest)<>0then
    do_error(2005);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('fa',[],r);
  if not initok then
    do_error(2006);
  if (RegExprReplaceAll(r,'asdfafaasd','',Dest)<>2) or
    (Dest<>'asdasd') then
    do_error(2007);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('fa',[],r);
  if not initok then
    do_error(2008);
  if (RegExprReplaceAll(r,'asdfafaasdasdfafaasd','',Dest)<>4) or
    (Dest<>'asdasdasdasd') then
    do_error(2009);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('fa',[],r);
  if not initok then
    do_error(2000);
  if (RegExprReplaceAll(r,'fasdfasdf','',Dest)<>2) or
    (Dest<>'sdsdf') then
    do_error(2010);
  DestroyregExprEngine(r);

  initok:=GenerateRegExprEngine('fa',[],r);
  if not initok then
    do_error(2011);
  if (RegExprReplaceAll(r,'fasdfafaasdasdfafaasd','',Dest)<>5) or
    (Dest<>'sdasdasdasd') then
    do_error(2012);
  DestroyregExprEngine(r);

   writeln('*** Testing unit regexpr was successful ***');
end.
