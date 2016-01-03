program SetSizes;

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef fpc}
  {$APPTYPE CONSOLE}
  {$mode delphi}
  {$ifdef CPU16}
    {$define _16BITCOMPILER}
  {$endif}
{$else fpc}
  {$ifdef VER70} { Turbo Pascal 7 }
    {$define _16BITCOMPILER}
  {$endif}
  {$ifdef VER80} { Delphi 1 }
    {$define _16BITCOMPILER}
    uses
      WinCrt;
  {$endif}
  {$ifdef WIN32}
    {$APPTYPE CONSOLE}
  {$endif}
  {$ifdef WIN64}
    {$APPTYPE CONSOLE}
  {$endif}
{$endif fpc}

const
  _a= 0;
{$ifdef _16BITCOMPILER}
  three_or_four = 3;
{$else}
  three_or_four = 4;
{$endif}
=======
=======
>>>>>>> origin/fixes_2_2
{$APPTYPE CONSOLE}

{$ifdef fpc}
  {$mode delphi}
  {$packset 1}
{$endif}

const
  _a= 0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type
  TIntRange1_a =  0 + _a.. Pred( 1 * 8) + _a;
  TIntRange2_a =  0 + _a.. Pred( 2 * 8) + _a;
  TIntRange3_a =  0 + _a.. Pred( 3 * 8) + _a;
  TIntRange4_a =  0 + _a.. Pred( 4 * 8) + _a;
  TIntRange5_a =  0 + _a.. Pred( 5 * 8) + _a;
  TIntRange6_a =  0 + _a.. Pred( 6 * 8) + _a;
  TIntRange7_a =  0 + _a.. Pred( 7 * 8) + _a;
  TIntRange8_a =  0 + _a.. Pred( 8 * 8) + _a;
  TIntRange9_a =  0 + _a.. Pred( 9 * 8) + _a;
  TIntRange10_a=  0 + _a.. Pred(10 * 8) + _a;
  TIntRange11_a=  0 + _a.. Pred(11 * 8) + _a;
  TIntRange12_a=  0 + _a.. Pred(12 * 8) + _a;
  TIntRange13_a=  0 + _a.. Pred(13 * 8) + _a;
  TIntRange14_a=  0 + _a.. Pred(14 * 8) + _a;
  TIntRange15_a=  0 + _a.. Pred(15 * 8) + _a;
  TIntRange16_a=  0 + _a.. Pred(16 * 8) + _a;

  TSet1_a = set of TIntRange1_a;
  TSet2_a = set of TIntRange2_a;
  TSet3_a = set of TIntRange3_a;
  TSet4_a = set of TIntRange4_a;
  TSet5_a = set of TIntRange5_a;
  TSet6_a = set of TIntRange6_a;
  TSet7_a = set of TIntRange7_a;
  TSet8_a = set of TIntRange8_a;
  TSet9_a = set of TIntRange9_a;
  TSet10_a= set of TIntRange10_a;
  TSet11_a= set of TIntRange11_a;
  TSet12_a= set of TIntRange12_a;
  TSet13_a= set of TIntRange13_a;
  TSet14_a= set of TIntRange14_a;
  TSet15_a= set of TIntRange15_a;
  TSet16_a= set of TIntRange16_a;

const
  _b= 1;

type
  TIntRange1_b =  0 + _b.. Pred( 1 * 8) + _b;
  TIntRange2_b =  0 + _b.. Pred( 2 * 8) + _b;
  TIntRange3_b =  0 + _b.. Pred( 3 * 8) + _b;
  TIntRange4_b =  0 + _b.. Pred( 4 * 8) + _b;
  TIntRange5_b =  0 + _b.. Pred( 5 * 8) + _b;
  TIntRange6_b =  0 + _b.. Pred( 6 * 8) + _b;
  TIntRange7_b =  0 + _b.. Pred( 7 * 8) + _b;
  TIntRange8_b =  0 + _b.. Pred( 8 * 8) + _b;
  TIntRange9_b =  0 + _b.. Pred( 9 * 8) + _b;
  TIntRange10_b=  0 + _b.. Pred(10 * 8) + _b;
  TIntRange11_b=  0 + _b.. Pred(11 * 8) + _b;
  TIntRange12_b=  0 + _b.. Pred(12 * 8) + _b;
  TIntRange13_b=  0 + _b.. Pred(13 * 8) + _b;
  TIntRange14_b=  0 + _b.. Pred(14 * 8) + _b;
  TIntRange15_b=  0 + _b.. Pred(15 * 8) + _b;
  TIntRange16_b=  0 + _b.. Pred(16 * 8) + _b;

  TSet1_b = set of TIntRange1_b;
  TSet2_b = set of TIntRange2_b;
  TSet3_b = set of TIntRange3_b;
  TSet4_b = set of TIntRange4_b;
  TSet5_b = set of TIntRange5_b;
  TSet6_b = set of TIntRange6_b;
  TSet7_b = set of TIntRange7_b;
  TSet8_b = set of TIntRange8_b;
  TSet9_b = set of TIntRange9_b;
  TSet10_b= set of TIntRange10_b;
  TSet11_b= set of TIntRange11_b;
  TSet12_b= set of TIntRange12_b;
  TSet13_b= set of TIntRange13_b;
  TSet14_b= set of TIntRange14_b;
  TSet15_b= set of TIntRange15_b;
  TSet16_b= set of TIntRange16_b;

const
  _c= 7;

type
  TIntRange1_c =  0 + _c.. Pred( 1 * 8) + _c;
  TIntRange2_c =  0 + _c.. Pred( 2 * 8) + _c;
  TIntRange3_c =  0 + _c.. Pred( 3 * 8) + _c;
  TIntRange4_c =  0 + _c.. Pred( 4 * 8) + _c;
  TIntRange5_c =  0 + _c.. Pred( 5 * 8) + _c;
  TIntRange6_c =  0 + _c.. Pred( 6 * 8) + _c;
  TIntRange7_c =  0 + _c.. Pred( 7 * 8) + _c;
  TIntRange8_c =  0 + _c.. Pred( 8 * 8) + _c;
  TIntRange9_c =  0 + _c.. Pred( 9 * 8) + _c;
  TIntRange10_c=  0 + _c.. Pred(10 * 8) + _c;
  TIntRange11_c=  0 + _c.. Pred(11 * 8) + _c;
  TIntRange12_c=  0 + _c.. Pred(12 * 8) + _c;
  TIntRange13_c=  0 + _c.. Pred(13 * 8) + _c;
  TIntRange14_c=  0 + _c.. Pred(14 * 8) + _c;
  TIntRange15_c=  0 + _c.. Pred(15 * 8) + _c;
  TIntRange16_c=  0 + _c.. Pred(16 * 8) + _c;

  TSet1_c = set of TIntRange1_c;
  TSet2_c = set of TIntRange2_c;
  TSet3_c = set of TIntRange3_c;
  TSet4_c = set of TIntRange4_c;
  TSet5_c = set of TIntRange5_c;
  TSet6_c = set of TIntRange6_c;
  TSet7_c = set of TIntRange7_c;
  TSet8_c = set of TIntRange8_c;
  TSet9_c = set of TIntRange9_c;
  TSet10_c= set of TIntRange10_c;
  TSet11_c= set of TIntRange11_c;
  TSet12_c= set of TIntRange12_c;
  TSet13_c= set of TIntRange13_c;
  TSet14_c= set of TIntRange14_c;
  TSet15_c= set of TIntRange15_c;
  TSet16_c= set of TIntRange16_c;

procedure test(actualsize, wantsize: longint);
begin
  if (actualsize<>wantsize) then
    halt(1);
end;


begin
  WriteLn(Low(TIntRange1_a),'..',High(TIntRange1_a),' -> ', SizeOf(TSet1_a));
  test(SizeOf(TSet1_a),1);
  WriteLn(Low(TIntRange2_a),'..',High(TIntRange2_a),' -> ', SizeOf(TSet2_a));
  test(SizeOf(TSet2_a),2);
  WriteLn(Low(TIntRange3_a),'..',High(TIntRange3_a),' -> ', SizeOf(TSet3_a));
<<<<<<< HEAD
<<<<<<< HEAD
  test(SizeOf(TSet3_a),three_or_four);
=======
  test(SizeOf(TSet3_a),4);
>>>>>>> graemeg/fixes_2_2
=======
  test(SizeOf(TSet3_a),4);
>>>>>>> origin/fixes_2_2
  WriteLn(Low(TIntRange4_a),'..',High(TIntRange4_a),' -> ', SizeOf(TSet4_a));
  test(SizeOf(TSet4_a),4);
  WriteLn(Low(TIntRange5_a),'..',High(TIntRange5_a),' -> ', SizeOf(TSet5_a));
  test(SizeOf(TSet5_a),5);
  WriteLn(Low(TIntRange6_a),'..',High(TIntRange6_a),' -> ', SizeOf(TSet6_a));
  test(SizeOf(TSet6_a),6);
  WriteLn(Low(TIntRange7_a),'..',High(TIntRange7_a),' -> ', SizeOf(TSet7_a));
  test(SizeOf(TSet7_a),7);
  WriteLn(Low(TIntRange8_a),'..',High(TIntRange8_a),' -> ', SizeOf(TSet8_a));
  test(SizeOf(TSet8_a),8);
  WriteLn(Low(TIntRange9_a),'..',High(TIntRange9_a),' -> ', SizeOf(TSet9_a));
  test(SizeOf(TSet9_a),9);
  WriteLn(Low(TIntRange10_a),'..',High(TIntRange10_a),' -> ', SizeOf(TSet10_a));
  test(SizeOf(TSet10_a),10);
  WriteLn(Low(TIntRange11_a),'..',High(TIntRange11_a),' -> ', SizeOf(TSet11_a));
  test(SizeOf(TSet11_a),11);
  WriteLn(Low(TIntRange12_a),'..',High(TIntRange12_a),' -> ', SizeOf(TSet12_a));
  test(SizeOf(TSet12_a),12);
  WriteLn(Low(TIntRange13_a),'..',High(TIntRange13_a),' -> ', SizeOf(TSet13_a));
  test(SizeOf(TSet13_a),13);
  WriteLn(Low(TIntRange14_a),'..',High(TIntRange14_a),' -> ', SizeOf(TSet14_a));
  test(SizeOf(TSet14_a),14);
  WriteLn(Low(TIntRange15_a),'..',High(TIntRange15_a),' -> ', SizeOf(TSet15_a));
  test(SizeOf(TSet15_a),15);
  WriteLn(Low(TIntRange16_a),'..',High(TIntRange16_a),' -> ', SizeOf(TSet16_a));
  test(SizeOf(TSet16_a),16);

  WriteLn;

  WriteLn(Low(TIntRange1_b),'..',High(TIntRange1_b),' -> ', SizeOf(TSet1_b));
  test(SizeOf(TSet1_b),2);
  WriteLn(Low(TIntRange2_b),'..',High(TIntRange2_b),' -> ', SizeOf(TSet2_b));
<<<<<<< HEAD
<<<<<<< HEAD
  test(SizeOf(TSet2_b),three_or_four);
=======
  test(SizeOf(TSet2_b),4);
>>>>>>> graemeg/fixes_2_2
=======
  test(SizeOf(TSet2_b),4);
>>>>>>> origin/fixes_2_2
  WriteLn(Low(TIntRange3_b),'..',High(TIntRange3_b),' -> ', SizeOf(TSet3_b));
  test(SizeOf(TSet3_b),4);
  WriteLn(Low(TIntRange4_b),'..',High(TIntRange4_b),' -> ', SizeOf(TSet4_b));
  test(SizeOf(TSet4_b),5);
  WriteLn(Low(TIntRange5_b),'..',High(TIntRange5_b),' -> ', SizeOf(TSet5_b));
  test(SizeOf(TSet5_b),6);
  WriteLn(Low(TIntRange6_b),'..',High(TIntRange6_b),' -> ', SizeOf(TSet6_b));
  test(SizeOf(TSet6_b),7);
  WriteLn(Low(TIntRange7_b),'..',High(TIntRange7_b),' -> ', SizeOf(TSet7_b));
  test(SizeOf(TSet7_b),8);
  WriteLn(Low(TIntRange8_b),'..',High(TIntRange8_b),' -> ', SizeOf(TSet8_b));
  test(SizeOf(TSet8_b),9);
  WriteLn(Low(TIntRange9_b),'..',High(TIntRange9_b),' -> ', SizeOf(TSet9_b));
  test(SizeOf(TSet9_b),10);
  WriteLn(Low(TIntRange10_b),'..',High(TIntRange10_b),' -> ', SizeOf(TSet10_b));
  test(SizeOf(TSet10_b),11);
  WriteLn(Low(TIntRange11_b),'..',High(TIntRange11_b),' -> ', SizeOf(TSet11_b));
  test(SizeOf(TSet11_b),12);
  WriteLn(Low(TIntRange12_b),'..',High(TIntRange12_b),' -> ', SizeOf(TSet12_b));
  test(SizeOf(TSet12_b),13);
  WriteLn(Low(TIntRange13_b),'..',High(TIntRange13_b),' -> ', SizeOf(TSet13_b));
  test(SizeOf(TSet13_b),14);
  WriteLn(Low(TIntRange14_b),'..',High(TIntRange14_b),' -> ', SizeOf(TSet14_b));
  test(SizeOf(TSet14_b),15);
  WriteLn(Low(TIntRange15_b),'..',High(TIntRange15_b),' -> ', SizeOf(TSet15_b));
  test(SizeOf(TSet15_b),16);
  WriteLn(Low(TIntRange16_b),'..',High(TIntRange16_b),' -> ', SizeOf(TSet16_b));
  test(SizeOf(TSet16_b),17);

  WriteLn;

  WriteLn(Low(TIntRange1_c),'..',High(TIntRange1_c),' -> ', SizeOf(TSet1_c));
  test(SizeOf(TSet1_c),2);
  WriteLn(Low(TIntRange2_c),'..',High(TIntRange2_c),' -> ', SizeOf(TSet2_c));
<<<<<<< HEAD
<<<<<<< HEAD
  test(SizeOf(TSet2_c),three_or_four);
=======
  test(SizeOf(TSet2_c),4);
>>>>>>> graemeg/fixes_2_2
=======
  test(SizeOf(TSet2_c),4);
>>>>>>> origin/fixes_2_2
  WriteLn(Low(TIntRange3_c),'..',High(TIntRange3_c),' -> ', SizeOf(TSet3_c));
  test(SizeOf(TSet3_c),4);
  WriteLn(Low(TIntRange4_c),'..',High(TIntRange4_c),' -> ', SizeOf(TSet4_c));
  test(SizeOf(TSet4_c),5);
  WriteLn(Low(TIntRange5_c),'..',High(TIntRange5_c),' -> ', SizeOf(TSet5_c));
  test(SizeOf(TSet5_c),6);
  WriteLn(Low(TIntRange6_c),'..',High(TIntRange6_c),' -> ', SizeOf(TSet6_c));
  test(SizeOf(TSet6_c),7);
  WriteLn(Low(TIntRange7_c),'..',High(TIntRange7_c),' -> ', SizeOf(TSet7_c));
  test(SizeOf(TSet7_c),8);
  WriteLn(Low(TIntRange8_c),'..',High(TIntRange8_c),' -> ', SizeOf(TSet8_c));
  test(SizeOf(TSet8_c),9);
  WriteLn(Low(TIntRange9_c),'..',High(TIntRange9_c),' -> ', SizeOf(TSet9_c));
  test(SizeOf(TSet9_c),10);
  WriteLn(Low(TIntRange10_c),'..',High(TIntRange10_c),' -> ', SizeOf(TSet10_c));
  test(SizeOf(TSet10_c),11);
  WriteLn(Low(TIntRange11_c),'..',High(TIntRange11_c),' -> ', SizeOf(TSet11_c));
  test(SizeOf(TSet11_c),12);
  WriteLn(Low(TIntRange12_c),'..',High(TIntRange12_c),' -> ', SizeOf(TSet12_c));
  test(SizeOf(TSet12_c),13);
  WriteLn(Low(TIntRange13_c),'..',High(TIntRange13_c),' -> ', SizeOf(TSet13_c));
  test(SizeOf(TSet13_c),14);
  WriteLn(Low(TIntRange14_c),'..',High(TIntRange14_c),' -> ', SizeOf(TSet14_c));
  test(SizeOf(TSet14_c),15);
  WriteLn(Low(TIntRange15_c),'..',High(TIntRange15_c),' -> ', SizeOf(TSet15_c));
  test(SizeOf(TSet15_c),16);
  WriteLn(Low(TIntRange16_c),'..',High(TIntRange16_c),' -> ', SizeOf(TSet16_c));
  test(SizeOf(TSet16_c),17);
end.
