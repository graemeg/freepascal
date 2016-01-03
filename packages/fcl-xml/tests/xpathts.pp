{**********************************************************************

    This file is part of the Free Component Library (FCL)

    Test suite for the xpath.pp unit.
    Largely based on expressions from libxml2 source tree.
    Copyright (c) 2009 by Sergei Gorelkin, sergei_gorelkin@mail.ru

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
program xpathts;
{$mode delphi}{$h+}

uses
  Classes, SysUtils, Math,
  dom, xmlread, xmlwrite, xpath;

type
  TResultType = (rtString, rtNumber, rtBool, rtNodeStr, rtOther);

  TTestRec = record
    data: string;              // UTF-8 encoded
    expr: DOMString;
  case rt: TResultType of
    rtString, rtNodeStr: (s: DOMPChar);   // cannot use DOMString here
    rtNumber: (n: Extended);
    rtBool:   (b: Boolean);
  end;

  TTestRec3 = record
    data: string;                // UTF-8 encoded
    re: string;
    expr: DOMString;
  case rt: TResultType of
    rtString, rtNodeStr: (s: DOMPChar);   // cannot use DOMString here
    rtNumber: (n: Extended);
    rtBool:   (b: Boolean);
  end;

{$warnings off}
const
  BaseTests: array[0..4] of TTestRec = (
    (expr: '1';           rt: rtNumber; n: 1),
    (expr: '1+2';         rt: rtNumber; n: 3),
    (expr: '2*3';         rt: rtNumber; n: 6),
    (expr: '1+2*3+4';     rt: rtNumber; n: 11),
    (expr: '(1+2)*(3+4)'; rt: rtNumber; n: 21)
  );

  CompareTests: array[0..46] of TTestRec = (
    (expr: '0<0';        rt: rtBool; b: False),
    (expr: '0<=0';       rt: rtBool; b: True),
    (expr: '0>0';        rt: rtBool; b: False),
    (expr: '0>=0';       rt: rtBool; b: True),
    (expr: '0<1';        rt: rtBool; b: True),
    (expr: '0<=1';       rt: rtBool; b: True),
    (expr: '0>1';        rt: rtBool; b: False),
    (expr: '0>=1';       rt: rtBool; b: False),
    (expr: '1<0';        rt: rtBool; b: False),
    (expr: '1<=0';       rt: rtBool; b: False),
    (expr: '1>0';        rt: rtBool; b: True),
    (expr: '1>=0';       rt: rtBool; b: True),
    (expr: '1<1';        rt: rtBool; b: False),
    (expr: '1<=1';       rt: rtBool; b: True),
    (expr: '1>1';        rt: rtBool; b: False),
    (expr: '1>=1';       rt: rtBool; b: True),
    (expr: '0>-0';       rt: rtBool; b: False),
    (expr: '"0"<1';      rt: rtBool; b: True),
    (expr: '"0"<=1';     rt: rtBool; b: True),
    (expr: '"0">1';      rt: rtBool; b: False),
    (expr: '"0">=1';     rt: rtBool; b: False),
    (expr: '0<"1.2"';    rt: rtBool; b: True),
    (expr: '0<="1.2"';   rt: rtBool; b: True),
    (expr: '0>"1.2"';    rt: rtBool; b: False),
    (expr: '0>="1.2"';   rt: rtBool; b: False),
    (expr: '0<"-0.2"';   rt: rtBool; b: False),
    (expr: '0<="-0.2"';  rt: rtBool; b: False),
    (expr: '0>"-0.2"';   rt: rtBool; b: True),
    (expr: '0>="-0.2"';  rt: rtBool; b: True),
    (expr: 'false()<1';  rt: rtBool; b: True),
    (expr: 'false()<=1'; rt: rtBool; b: True),
    (expr: '0>true()';   rt: rtBool; b: False),
    (expr: '0>=true()';  rt: rtBool; b: False),
    (expr: '"a" > "a"';  rt: rtBool; b: False),
    (expr: '"a" > "b"';  rt: rtBool; b: False),
    (expr: '"b" > "a"';  rt: rtBool; b: False),
    (expr: '"a" < "a"';  rt: rtBool; b: False),
    (expr: '"a" < "b"';  rt: rtBool; b: False),
    (expr: '"b" < "a"';  rt: rtBool; b: False),
    (expr: '"a" >= "a"'; rt: rtBool; b: False),
    (expr: '"a" >= "b"'; rt: rtBool; b: False),
    (expr: '"b" >= "a"'; rt: rtBool; b: False),
    (expr: '"a" <= "a"'; rt: rtBool; b: False),
    (expr: '"a" <= "b"'; rt: rtBool; b: False),
    (expr: '"b" <= "a"'; rt: rtBool; b: False),
    (expr: '"a" > "0.0"'; rt: rtBool; b: False),
    (expr: '"a" < "0.0"'; rt: rtBool; b: False)
  );

  nscmp = '<doc>'#10+
    '<j l="12" w="33">first</j>'#10+
    '<j l="17" w="45">second</j>'#10+
    '<j l="16" w="78">third</j>'#10+
    '<j l="12" w="33">fourth</j>'#10+
    '</doc>';

  nscmp2 = '<doc>'#10+
    '<j l="12" w="45">first</j>'#10+
    '<j l="17" w="45">second</j>'#10+
    '<j l="16" w="78">third</j>'#10+
    '<j l="12" w="33">fourth</j>'#10+
    '</doc>';

  simple = '<doc>test</doc>';

  bool58 = '<doc>'#10+
  '<av>'#10+
  ' <a>'#10+
  '   <b>b</b>'#10+
  '   <c>c</c>'#10+
  '   <d>d</d>'#10+
  '   <e>e</e>'#10+
  ' </a>'#10+
  ' <v>'#10+
  '   <w>w</w>'#10+
  '   <x>x</x>'#10+
  '   <y>y</y>'#10+
  '   <z>z</z>'#10+
  ' </v>'#10+
  ' <a>'#10+
  '   <b>fe</b>'#10+
  '   <c>fi</c>'#10+
  '   <d>fo</d>'#10+
  '   <e>fu</e>'#10+
  ' </a>'#10+
  ' <v>'#10+
  '   <w>fee</w>'#10+
  '   <x>fii</x>'#10+
  '   <y>foo</y>'#10+
  '   <z>fom</z>'#10+
  ' </v>'#10+
  ' <j>foo</j>'#10+
  ' <j>foo</j>'#10+
  ' <j>foo</j>'#10+
  ' <j>foo</j>'#10+
  '</av>'#10+
  '</doc>';

  bool84='<doc>'#10+
  '<avj>'#10+
  '  <good>'#10+
  '   <b>12</b>'#10+
  '   <c>34</c>'#10+
  '   <d>56</d>'#10+
  '   <e>78</e>'#10+
  ' </good>'#10+
  '</avj>'#10+
  '</doc>';

  bool85='<doc>'#10+
  '<avj>'#10+
  '  <bool>'#10+
  '   <b>true</b>'#10+
  '   <c></c>'#10+
  '   <d>false?</d>'#10+
  '   <e>1</e>'#10+
  '   <f>0</f>'#10+
  ' </bool>'#10+
  '</avj>'#10+
  '</doc>';

  str04='<doc>'#10+
  '<a>Testing this</a>'#10+
  '<b>and this too</b>'#10+
  '</doc>';
  
  NodesetCompareTests: array[0..38] of TTestRec = (
   { same nodeset }
   (data: nscmp; expr: 'j[@l="12"] = j[@w="33"]'; rt: rtBool; b: True),   // #70
   { disjoint nodesets }
   (data: nscmp; expr: 'j[@l="12"] = j[@l="17"]'; rt: rtBool; b: False),  // #71
   { both have one common node }
   (data: nscmp2; expr: 'j[@l="12"] = j[@w="45"]'; rt: rtBool; b: True),  // #72
   { same nodeset - unequal }
   (data: nscmp; expr: 'j[@l="12"] != j[@w="33"]'; rt: rtBool; b: True),  // #73
   { disjoint - unequal }
   (data: nscmp; expr: 'j[@l="12"] != j[@l="17"]'; rt: rtBool; b: True),  // #74
   { one common node - unequal }
   (data: nscmp2; expr: 'j[@l="12"] != j[@w="45"]'; rt: rtBool; b: True), // #75
   { single common node - unequal }
   (data: nscmp2; expr: 'j[@l="16"] != j[@w="78"]'; rt: rtBool; b: False),// #76
   { nodeset vs. string }
   (data: bool58; expr: '/doc/av//*="foo"';       rt: rtBool; b: True),  // #58.1
   (data: bool58; expr: 'not(/doc/av//*!="foo")'; rt: rtBool; b: False), // #58.2
   (data: bool58; expr: '/doc/av//j="foo"';       rt: rtBool; b: True),  // #58.3
   (data: bool58; expr: 'not(/doc/av//j!="foo")'; rt: rtBool; b: True),  // #58.4
   { empty nodeset vs. string. Data differs, but that doesn't matter }
   (data: bool58; expr: '/doc/avj//k="foo"';      rt: rtBool; b: False), // #59.1
   (data: bool58; expr: 'not(/doc/avj//k="foo")'; rt: rtBool; b: True),  // #59.2
   (data: bool58; expr: '/doc/avj//k!="foo"';     rt: rtBool; b: False), // #59.3
   (data: bool58; expr: 'not(/doc/avj//k!="foo")'; rt: rtBool; b: True), // #59.4
   { nodeset vs. number }
   (data: bool84; expr: '/doc/avj/good/*=34';      rt: rtBool; b: True),  // #84.1
   (data: bool84; expr: 'not(/doc/avj/good/*=34)'; rt: rtBool; b: False), // #84.2
   (data: bool84; expr: '/doc/avj/good/*!=34';     rt: rtBool; b: True),  // #84.3
   (data: bool84; expr: 'not(/doc/avj/good/*!=34)'; rt: rtBool; b: False),// #84.4
   { same with reversed order of operands }
   (data: bool84; expr: '34=/doc/avj/good/*';      rt: rtBool; b: True),  // #84.5
   (data: bool84; expr: 'not(34=/doc/avj/good/*)'; rt: rtBool; b: False), // #84.6
   (data: bool84; expr: '34!=/doc/avj/good/*';     rt: rtBool; b: True),  // #84.7
   (data: bool84; expr: 'not(34!=/doc/avj/good/*)'; rt: rtBool; b: False),// #84.8
   { nodeset vs. boolean }
   (data: bool85; expr: '/doc/avj/bool/*=true()';      rt: rtBool; b: True),  // #85.1
   (data: bool85; expr: 'not(/doc/avj/bool/*=true())'; rt: rtBool; b: False), // #85.2
   (data: bool85; expr: '/doc/avj/bool/*!=true()';     rt: rtBool; b: False), // #85.3
   (data: bool85; expr: 'not(/doc/avj/bool/*!=true())'; rt: rtBool; b: True), // #85.4
   { same with reversed order of operands }
   (data: bool85; expr: 'true()=/doc/avj/bool/*';      rt: rtBool; b: True),  // #85.5
   (data: bool85; expr: 'not(true()=/doc/avj/bool/*)'; rt: rtBool; b: False), // #85.6
   (data: bool85; expr: 'true()!=/doc/avj/bool/*';     rt: rtBool; b: False), // #85.7
   (data: bool85; expr: 'not(true()!=/doc/avj/bool/*)'; rt: rtBool; b: True), // #85.8
   { empty nodeset vs. boolean }
   (data: bool85; expr: '/doc/avj/none/*=true()';      rt: rtBool; b: False), // #86.1
   (data: bool85; expr: 'not(/doc/avj/none/*=true())'; rt: rtBool; b: True),  // #86.2
   (data: bool85; expr: '/doc/avj/none/*!=true()';     rt: rtBool; b: True),  // #86.3
   (data: bool85; expr: 'not(/doc/avj/none/*!=true())'; rt: rtBool; b: False),// #86.4
   { same with reversed order of operands }
   (data: bool85; expr: 'true()=/doc/avj/none/*';      rt: rtBool; b: False), // #86.5
   (data: bool85; expr: 'not(true()=/doc/avj/none/*)'; rt: rtBool; b: True),  // #86.6
   (data: bool85; expr: 'true()!=/doc/avj/none/*';     rt: rtBool; b: True),  // #86.7
   (data: bool85; expr: 'not(true()!=/doc/avj/none/*)'; rt: rtBool; b: False) // #86.8

  );

  EqualityTests: array[0..25] of TTestRec = (
    (expr: '1=1';  rt: rtBool; b: True),
    (expr: '1!=1'; rt: rtBool; b: False),
    (expr: '1=0';  rt: rtBool; b: False),
    (expr: '1!=0'; rt: rtBool; b: True),

    (expr: 'true()=true()';   rt: rtBool; b: True),
    (expr: 'true()!=true()';  rt: rtBool; b: False),
    (expr: 'true()=false()';  rt: rtBool; b: False),
    (expr: 'false()!=true()'; rt: rtBool; b: True),

    (expr: '"test"="test"';   rt: rtBool; b: True),
    (expr: '"test"!="test"';  rt: rtBool; b: False),
    (expr: '"test2"="test"';  rt: rtBool; b: False),
    (expr: '"test2"!="test"'; rt: rtBool; b: True),

    (expr: 'false()=0';   rt: rtBool; b: True),
    (expr: 'false()!=0';  rt: rtBool; b: False),
    (expr: 'false()=1';   rt: rtBool; b: False),

    (expr: 'false()!=1';  rt: rtBool; b: True),
    (expr: '0=true()';    rt: rtBool; b: False),
    (expr: '0!=true()';   rt: rtBool; b: True),
    (expr: '1=true()';    rt: rtBool; b: True),
    (expr: '1!=true()';   rt: rtBool; b: False),

    (expr: 'true()="test"';   rt: rtBool; b: True),
    (expr: 'false()="test"';  rt: rtBool; b: False),
    (expr: '"test"!=true()';  rt: rtBool; b: False),
    (expr: '"test"!=false()'; rt: rtBool; b: True),

    (expr: '"a"=0.0'; rt: rtBool; b: False),
    (expr: '"a"!=0.0'; rt: rtBool; b: True)
  );

  math88='<doc>'+
  '<n0>0</n0>'+
  '<n1>1</n1>'+
  '<n2>2</n2>'+
  '<n3>3</n3>'+
  '<n4>4</n4>'+
  '<n5>5</n5>'+
  '<n6>6</n6>'+
  '<n7>2</n7>'+
  '<n8>6</n8>'+
  '<n9>10</n9>'+
  '<n10>3</n10>'+
  '</doc>';

  math85='<doc>'+
  '<n0>0</n0>'+
  '<n1>1</n1>'+
  '<n2>2</n2>'+
  '<n3>3</n3>'+
  '<n4>4</n4>'+
  '<e>five</e>'+
  '</doc>';

  math80='<doc>'+
  '<n1 attrib="10">5</n1>'+
  '<n2 attrib="4">2</n2>'+
  '<div attrib="-5">-5</div>'+
  '<mod attrib="-2">2</mod>'+
  '</doc>';

  math69='<doc>'+
  '<n-1 attrib="9">3</n-1>'+
  '<n-2 attrib="1">7</n-2>'+
  '</doc>';

  FloatTests: array[0..70] of TTestRec = (
    (expr: '1';        rt: rtNumber; n: 1),
    (expr: '123';      rt: rtNumber; n: 123),
    (expr: '1.23';     rt: rtNumber; n: 1.23),
    (expr: '0.123';    rt: rtNumber; n: 0.123),
    (expr: '4.';       rt: rtNumber; n: 4),
    (expr: '.4';       rt: rtNumber; n: 0.4),
    //(expr: '1.23e3';   rt: rtNumber; n: 1230),
    //(expr: '1.23e-3';  rt: rtNumber; n: 0.00123),
    (expr: '1 div 0';  rt: rtNumber; n: Infinity),
    (expr: '-1 div 0'; rt: rtNumber; n: -Infinity),
    (expr: '0 div 0';  rt: rtNumber; n: NaN),
    (expr: '1 div -0'; rt: rtNumber; n: -Infinity),

    (expr: '(1 div 0) > 0';  rt: rtBool; b: True),
    (expr: '(1 div 0) < 0';  rt: rtBool; b: False),
    (expr: '(-1 div 0) > 0'; rt: rtBool; b: False),
    (expr: '(-1 div 0) < 0'; rt: rtBool; b: True),
    (expr: '(0 div 0) > 0';  rt: rtBool; b: False),
    (expr: '(0 div 0) < 0';  rt: rtBool; b: False),
    (expr: '(1 div -0) > 0'; rt: rtBool; b: False),
    (expr: '(1 div -0) < 0'; rt: rtBool; b: True),
    (expr: '0 div 0 = 0 div 0';  rt: rtBool; b: False),
    (expr: '0 div 0 != 0 div 0'; rt: rtBool; b: True),
    (expr: '0 div 0 > 0 div 0';  rt: rtBool; b: False),
    (expr: '0 div 0 < 0 div 0';  rt: rtBool; b: False),
    (expr: '0 div 0 >= 0 div 0'; rt: rtBool; b: False),
    (expr: '0 div 0 <= 0 div 0'; rt: rtBool; b: False),
    (expr: '1 div 0 = -1 div 0'; rt: rtBool; b: False),
    (expr: '1 div 0 != -1 div 0'; rt: rtBool; b: True),
    (expr: '1 div 0 > -1 div 0';  rt: rtBool; b: True),
    (expr: '1 div 0 < -1 div 0';  rt: rtBool; b: False),
    (expr: '1 div 0 >= -1 div 0'; rt: rtBool; b: True),
    (expr: '1 div 0 <= -1 div 0'; rt: rtBool; b: False),
    (expr: '1 div 0 = 1 div 0';   rt: rtBool; b: True),
    (expr: '1 div 0 != 1 div 0';  rt: rtBool; b: False),
    (expr: '1 div 0 > 1 div 0';   rt: rtBool; b: False),
    (expr: '1 div 0 < 1 div 0';   rt: rtBool; b: False),
    (expr: '1 div 0 >= -1 div 0'; rt: rtBool; b: True),
    (expr: '1 div 0 <= -1 div 0'; rt: rtBool; b: False),
    (expr: '-2 div 0 = -1 div 0'; rt: rtBool; b: True),

    (expr: '1 div floor(0.1)';    rt: rtNumber; n: Infinity),
    (expr: '1 div floor(-0.1)';   rt: rtNumber; n: -1),
    (expr: '1 div floor(-0)';     rt: rtNumber; n: -Infinity),
    (expr: '1 div floor(0)';      rt: rtNumber; n: Infinity),
    (expr: '1 div ceiling(0.1)';  rt: rtNumber; n: 1),
    (expr: '1 div ceiling(-0.1)'; rt: rtNumber; n: -Infinity),
    (expr: '1 div ceiling(-0)';   rt: rtNumber; n: -Infinity),
    (expr: '1 div ceiling(0)';    rt: rtNumber; n: Infinity),
    (expr: '1 div round(0.1)';    rt: rtNumber; n: Infinity),
    (expr: '1 div round(-0.1)';   rt: rtNumber; n: -Infinity),
    (expr: '1 div round(-0)';     rt: rtNumber; n: -Infinity),
    (expr: '1 div round(0)';      rt: rtNumber; n: Infinity),
    (expr: '1 div number("f")';   rt: rtNumber; n: NaN),
    (expr: 'number("f") div 1';   rt: rtNumber; n: NaN),
    (expr: '1 div (1 div 0)';     rt: rtNumber; n: 0),
    (expr: '(1 div 0) div 1';     rt: rtNumber; n: Infinity),
    (expr: '-(1 div 0) div 1';    rt: rtNumber; n: -Infinity),

    (expr: '5 mod 2';     rt: rtNumber; n: 1),
    (expr: '5 mod -2';    rt: rtNumber; n: 1),
    (expr: '-5 mod 2';    rt: rtNumber; n: -1),
    (expr: '-5 mod -2';   rt: rtNumber; n: -1),
    (expr: '2 mod number("xxx")'; rt: rtNumber; n: NaN),
    (expr: 'number("xxx") mod 3'; rt: rtNumber; n: NaN),
    (expr: '8 mod 3 = 2'; rt: rtBool; b: True),
    (data: math88; expr: '(n1*n2*n3*n4*n5*n6)div n7 div n8 div n9 div n10'; rt: rtNumber; n: 2),
    (data: math85; expr: '((((((n3+5)*(3)+(((n2)+2)*(n1 - 6)))-(n4 - n2))+(-(4-6)))))'; rt: rtNumber; n: 4),
    (data: math80; expr: 'div mod mod'; rt: rtNumber; n: -1),
    (data: math69; expr: '-(n-2/@attrib) - -(n-1/@attrib)'; rt: rtNumber; n: 8),
    (data: math69; expr: '-n-2/@attrib --n-1/@attrib'; rt: rtNumber; n: 8),
    (data: math69; expr: '-n-2 --n-1'; rt: rtNumber; n: -4),

   // test boolean operator short-circuting; "count(5)" acts as an error
    (expr: '10+30*20 or count(5)';  rt: rtBool; b: True),
    (expr: '75-50-25 and count(5)'; rt: rtBool; b: False),
    (expr: '"1" and "0"';     rt: rtBool; b: True),
    (expr: '0 or ""';         rt: rtBool; b: False)
  );

  math95='<doc>'+
  '<e>1</e>'+
  '<e>2</e>'+
  '<e>3</e>'+
  '<e>4</e>'+
  '<e>five</e>'+
  '</doc>';

  math96='<doc>'+
  '<e>17</e>'+
  '<e>-5</e>'+
  '<e>8</e>'+
  '<e>-37</e>'+
  '</doc>';

  expr01='<doc>'+
  '<para id="1" xml:lang="en">en</para>'+
  '<div xml:lang="en">'+
  '  <para>en</para>'+
  '</div>'+
  '<para id="3" xml:lang="EN">EN</para>'+
  '<para id="4" xml:lang="en-us">en-us</para>'+
  '</doc>';

  id04='<!DOCTYPE t04 ['+
  '<!ELEMENT t04 (a*)>'+
  '<!ELEMENT a EMPTY>'+
  '<!ATTLIST a  id ID #REQUIRED>'+
  ']>'+
  '<t04>'+
  '<a id="a"/>'+
  '<a id="b"/>'+
  '<a id="c"/>'+
  '<a id="d"/>'+
  '</t04>';
  
  pos04='<doc>'+
  '<a test="true"><num>1</num></a>'+
  '<a><num>1191</num></a>'+
  '<a><num>263</num></a>'+
  '<a test="true"><num>2</num></a>'+
  '<a><num>827</num></a>'+
  '<a><num>256</num></a>'+
  '<a test="true"><num>3</num></a>'+
  '<a test="true"><num>4</num></a>'+
  '</doc>';

  FunctionTests: array[0..51] of TTestRec = (
  // last()
  // local-name()
  // namespace-uri()
  // name()

    (expr: 'boolean(0)';        rt: rtBool; b: False),
    (expr: 'boolean(-0)';       rt: rtBool; b: False),
    (expr: 'boolean(1 div 0)';  rt: rtBool; b: True),
    (expr: 'boolean(-1 div 0)'; rt: rtBool; b: True),
    (expr: 'boolean(0 div 0)';  rt: rtBool; b: False),
    (expr: 'boolean("")';       rt: rtBool; b: False),
    (expr: 'boolean("abc")';    rt: rtBool; b: True),

    (data: simple; expr: 'boolean(/doc)'; rt: rtBool; b: True),   // #40
    (data: simple; expr: 'boolean(foo)'; rt: rtBool; b: False),   // #41

    (expr: 'true()';  rt: rtBool; b: True),
    (expr: 'false()'; rt: rtBool; b: False),
    (expr: 'not(true())';  rt: rtBool; b: False),
    (expr: 'not(false())'; rt: rtBool; b: True),
    (expr: 'not("")';      rt: rtBool; b: True),

    // lang() tests. These ones, however, test much more than lang().
    (data: expr01; expr: 'para[@id="1" and lang("en")]'; rt: rtNodeStr; s: 'en'),     // expression01
    (data: expr01; expr: 'para[@id="4" and lang("en")]'; rt: rtNodeStr; s: 'en-us'),  // expression03
    (data: expr01; expr: 'div/para[lang("en")]'; rt: rtNodeStr; s: 'en'),             // expression04
    (data: expr01; expr: 'para[@id="3" and lang("en")]'; rt: rtNodeStr; s: 'EN'),     // expression05
    
    (data: id04; expr: 'id("c")/@id'; rt: rtNodeStr; s: 'c'),  // idkey04
    
    // position() tests
    (data: pos04; expr: '*[@test][position()=4]/num'; rt: rtNodeStr; s: '4'),

    (expr: 'number("1.5")';   rt: rtNumber; n: 1.5),
    (expr: 'number("abc")';   rt: rtNumber; n: NaN),
    (expr: '-number("abc")';  rt: rtNumber; n: NaN),
    (expr: 'number(true())';  rt: rtNumber; n: 1.0),
    (expr: 'number(false())'; rt: rtNumber; n: 0),

    (data: math95; expr: 'sum(e)'; rt: rtNumber; n: NaN),
    (data: math96; expr: 'sum(e)'; rt: rtNumber; n: -17),

    (expr: 'floor(0.1)';     rt: rtNumber; n: 0),
    (expr: 'floor(-0.1)';    rt: rtNumber; n: -1),
    (expr: 'floor(-0)';      rt: rtNumber; n: 0),
    (expr: 'floor(0)';       rt: rtNumber; n: 0),
    (expr: 'floor(5.2)';     rt: rtNumber; n: 5),
    (expr: 'floor(-5.2)';    rt: rtNumber; n: -6),
    (expr: 'floor("NaN")';   rt: rtNumber; n: NaN),

    (expr: 'ceiling(0.1)';   rt: rtNumber; n: 1),
    (expr: 'ceiling(-0.1)';  rt: rtNumber; n: 0),
    (expr: 'ceiling(-0)';    rt: rtNumber; n: 0),
    (expr: 'ceiling(0)';     rt: rtNumber; n: 0),
    (expr: 'ceiling(5.2)';   rt: rtNumber; n: 6),
    (expr: 'ceiling(-5.2)';  rt: rtNumber; n: -5),
    (expr: 'ceiling("NaN")'; rt: rtNumber; n: NaN),

    (expr: 'round(0.1)';     rt: rtNumber; n: 0),
    (expr: 'round(5.2)';     rt: rtNumber; n: 5),
    (expr: 'round(5.5)';     rt: rtNumber; n: 6),
    (expr: 'round(5.6)';     rt: rtNumber; n: 6),
    (expr: 'round(-0.1)';    rt: rtNumber; n: 0),
    (expr: 'round(-5.2)';    rt: rtNumber; n: -5),
    (expr: 'round(-5.5)';    rt: rtNumber; n: -5),
    (expr: 'round(-5.6)';    rt: rtNumber; n: -6),
    (expr: 'round("NaN")';   rt: rtNumber; n: NaN),
    (expr: 'round(1 div 0)'; rt: rtNumber; n: Infinity),
    (expr: 'round(-1 div 0)'; rt: rtNumber; n: -Infinity)
  );

  str14 ='<doc>'#10+
  '  <av>'#10+
  '    <a>'#10+
  '      <b>b</b>'#10+
  '      <c>c</c>'#10+
  '      <d>d</d>'#10+
  '      <e>e</e>'#10+
  '    </a>'#10+
  '    <v>'#10+
  '      <w>w</w>'#10+
  '      <x>x</x>'#10+
  '      <y>y</y>'#10+
  '      <z>z</z>'#10+
  '    </v>'#10+
  '  </av>'#10+
  '</doc>';

  out14 =#10+
'      b'#10+
'      c'#10+
'      d'#10+
'      e'#10+
'    ';

  node08='<docs xmlns:ped="http://www.ped.com"><?MyPI DoesNothing ?><!-- This is a big tree containing all letters of the alphabet -->'#10+
  '<a attr1="This should not be seen">A</a>'#10+
  '<b><c attr1="tsnbs" attr2="tsnbs">B-C</c>'#10+
  '<d><e><f>TextNode_between_F_and_G'#10+
  '<g><h><i><j><k><l><m><n><o><p><q><r><s><t><u><v><w><x><y><z><Yahoo>Yahoo</Yahoo>'#10+
  '</z></y></x></w></v></u></t></s></r></q></p></o></n></m></l></k></j></i></h>SecondNode_after_H</g></f></e></d></b>'#10+
  '</docs>';
  
  out08=#10+
  'A'#10+
  'B-C'#10+
  'TextNode_between_F_and_G'#10+
  'Yahoo'#10+
  'SecondNode_after_H'#10;

  str30='<doc xmlns="http://xsl.lotus.com/ns2" xmlns:ns1="http://xsl.lotus.com/ns1">'#10+
  '<ns1:a attrib1="test" xmlns="http://xsl.lotus.com/ns2" xmlns:ns1="http://xsl.lotus.com/ns1"/>'#10+
  '<b ns1:attrib2="test"/>'#10+
  '</doc>';

  ns11='<doc-one xmlns="http://xsl.lotus.com/ns2" xmlns:ns1="http://xsl.lotus.com/ns1">'+
  '  <ns1:a-two attrib1="Goodbye" xmlns="http://xsl.lotus.com/ns2" xmlns:ns1="http://xsl.lotus.com/ns1">Hello</ns1:a-two>'+
  '  <b-three ns1:attrib2="Ciao">'+
  '    <c-four/>'+
  '  </b-three>'+
  '</doc-one>';

  pidata='<?a-pi data="foo"?><?b-pi data="bar"?><doc/>';

  StringTests: array[0..87] of TTestRec = (             // numbers refer to xalan/string/stringXX
    (expr: 'string(0)';       rt: rtString; s: '0'),
    (expr: 'string(5)';       rt: rtString; s: '5'),    // #38/39
    (expr: 'string(0.5)';     rt: rtString; s: '0.5'),
    (expr: 'string(-0.5)';    rt: rtString; s: '-0.5'),
    (expr: 'string("test")';  rt: rtString; s: 'test'), // #40
    (expr: 'string("")';      rt: rtString; s: ''),     // #41
    (expr: 'string(true())';  rt: rtString; s: 'true'),
    (expr: 'string(false())'; rt: rtString; s: 'false'),
    (expr: 'string(0 div 0)'; rt: rtString; s: 'NaN'),
    (expr: 'string(1 div 0)'; rt: rtString; s: 'Infinity'),
    (expr: 'string(-1 div 0)'; rt: rtString; s: '-Infinity'),
    // maybe other checks for correct numeric formats
    (data: str14; expr: 'string(av//*)'; rt: rtString; s: out14),
    (data: node08; expr: '/'; rt: rtNodeStr; s: out08),

    (expr: 'concat("titi","toto")'; rt: rtString; s: 'tititoto'),
    (expr: 'concat("titi","toto","tata")'; rt: rtString; s: 'tititototata'),
    (expr: 'concat("titi",''toto'')'; rt: rtString; s: 'tititoto'),
    (expr: 'concat("titi",''toto'',"tata","last")'; rt: rtString; s: 'tititototatalast'),
    (expr: 'concat("cd", 34)';      rt: rtString; s: 'cd34'),          // #101
    (expr: 'concat(false(), "ly")'; rt: rtString; s: 'falsely'),       // #104

    (expr: 'starts-with("tititoto","titi")'; rt: rtBool; b: True),
    (expr: 'starts-with("tititoto","to")';   rt: rtBool; b: False),
    (expr: 'starts-with("ab", "abc")';       rt: rtBool; b: False),    // #46
    (expr: 'starts-with("abc", "bc")';       rt: rtBool; b: False),    // #47
    (expr: 'starts-with("abc", "")';         rt: rtBool; b: True),     // #48
    (expr: 'starts-with("", "")';            rt: rtBool; b: True),     // #49
    (expr: 'starts-with(true(), "tr")';      rt: rtBool; b: True),     // #50

    (expr: 'contains("tititototata","titi")'; rt: rtBool; b: True),
    (expr: 'contains("tititototata","toto")'; rt: rtBool; b: True),
    (expr: 'contains("tititototata","tata")'; rt: rtBool; b: True),
    (expr: 'contains("tititototata","tita")'; rt: rtBool; b: False),
    // 'contains(concat(.,'BC'),concat('A','B','C'))' == true          // #57    
    (expr: 'contains("ab", "abc")';           rt: rtBool; b: False),   // #58
    (expr: 'contains("abc", "bcd")';          rt: rtBool; b: False),   // #60
    (expr: 'contains("abc", "")';             rt: rtBool; b: True),    // #61
    (expr: 'contains("", "")';                rt: rtBool; b: True),    // #62
    (expr: 'contains(true(), "e")';           rt: rtBool; b: True),    // #63    

    (expr: 'substring("12345",2,3)'; rt: rtString; s: '234'),
    (expr: 'substring("12345",2)';   rt: rtString; s: '2345'),
    (expr: 'substring("12345",-4)';  rt: rtString; s: '12345'),
    (expr: 'substring("12345",3.4)'; rt: rtString; s: '345'),
    (expr: 'substring("12345",3.6)'; rt: rtString; s: '45'),

    (expr: 'substring("12345",1.5,2.6)'; rt: rtString; s: '234'), // #16
    (expr: 'substring("12345",2.2,2.2)'; rt: rtString; s: '23'),
    (expr: 'substring("12345",0,3)';     rt: rtString; s: '12'),  // #17
    (expr: 'substring("12345",-8,10)';   rt: rtString; s: '1'),
    (expr: 'substring("12345",4,-10)';   rt: rtString; s: ''),

    (expr: 'substring("12345",0 div 0, 3)'; rt: rtString; s: ''), // #18
    (expr: 'substring("12345",1, 0 div 0)'; rt: rtString; s: ''), // #19
    (expr: 'substring("12345",1 div 0, 3)'; rt: rtString; s: ''),
    (expr: 'substring("12345",3,-1 div 0)'; rt: rtString; s: ''),
    (expr: 'substring("12345",-42, 1 div 0)'; rt: rtString; s: '12345'), // #20

    (expr: 'substring("12345",-1 div 0, 1 div 0)'; rt: rtString; s: ''), // #21
    (expr: 'substring("12345",-1 div 0,5)';        rt: rtString; s: ''),

    (expr: 'substring-before("1999/04/01","/")'; rt: rtString; s: '1999'),  // #08
    (expr: 'substring-before("1999/04/01","a")'; rt: rtString; s: ''),      // #68 modified
    (expr: 'substring-after("1999/04/01","/")'; rt: rtString; s: '04/01'),  // #09
    (expr: 'substring-after("1999/04/01","19")'; rt: rtString; s: '99/04/01'),
    (expr: 'substring-after("1999/04/01","a")'; rt: rtString; s: ''),

    (expr: 'string-length("")';     rt: rtNumber; n: 0),
    (expr: 'string-length("titi")'; rt: rtNumber; n: 4),
    (data: simple; expr: 'string-length(.)'; rt: rtNumber; n: 4),    // #02 modified
    (data: str04;  expr: 'string-length(/)'; rt: rtNumber; n:27),    // #04.1 modified
    (data: str04;  expr: 'string-length(/doc/a)'; rt: rtNumber; n: 12), // #04.2
    (data: str04;  expr: 'string-length()';  rt: rtNumber; n: 27),
    (expr: 'normalize-space("'#9#10#13' ab   cd'#10#13#9'ef'#9#10#13'  ")'; rt: rtString; s: 'ab cd ef'), // #10

    (expr: 'translate("bar", "abc", "ABC")'; rt: rtString; s: 'BAr'),  // #11
    (expr: 'translate("--aaa--","abc-","ABC")'; rt: rtString; s: 'AAA'),
    (expr: 'translate("ddaaadddd","abcd","ABCxy")'; rt: rtString; s: 'xxAAAxxxx'),   // #96

    (data: node08; expr: 'name(a/@attr1)';          rt: rtString; s: 'attr1'),  // namespace08 modified
    (data: node08; expr: 'namespace-uri(a/@attr1)'; rt: rtString; s: ''),
    (data: node08; expr: 'local-name(a/@attr1)';    rt: rtString; s: 'attr1'),

    (data: pidata; expr: 'name(/processing-instruction())';          rt: rtString; s: 'a-pi'),       // namespace29 modified
    (data: pidata; expr: 'name(/processing-instruction("b-pi"))';    rt: rtString; s: 'b-pi'),
    (data: pidata; expr: 'local-name(/processing-instruction())';    rt: rtString; s: 'a-pi'),
    (data: pidata; expr: 'namespace-uri(/processing-instruction())'; rt: rtString; s: ''),

    (data: node08; expr: 'name(//comment())';          rt: rtString; s: ''),  // namespace30 modified
    (data: node08; expr: 'local-name(//comment())';    rt: rtString; s: ''),
    (data: node08; expr: 'namespace-uri(//comment())'; rt: rtString; s: ''),

    (data: node08; expr: 'name(//text())';          rt: rtString; s: ''),  // namespace31 modified
    (data: node08; expr: 'local-name(//text())';    rt: rtString; s: ''),
    (data: node08; expr: 'namespace-uri(//text())'; rt: rtString; s: ''),

    // tests for number->string conversions at boundary conditions
    (expr: 'string(123456789012345678)';     rt: rtString; s: '123456789012345680'),    // #132.1
    (expr: 'string(-123456789012345678)';    rt: rtString; s: '-123456789012345680'),   // #132.2
    (expr: 'string(.10123456789234567893)';  rt: rtString; s: '0.10123456789234568'),   // #133.1
    (expr: 'string(-.10123456789234567893)'; rt: rtString; s: '-0.10123456789234568'),  // #133.2
    
    (expr: 'string(9.87654321012345)'; rt: rtString; s: '9.87654321012345'),  // #134.1
    (expr: 'string(98765432101234.5)'; rt: rtString; s: '98765432101234.5'),  // #134.2
    (expr: 'string(.0000000000000000000000000000000000000000123456789)'; rt: rtString;  // #135.1
      s: '0.0000000000000000000000000000000000000000123456789'),
    (expr: 'string(-.0000000000000000000000000000000000000000123456789)'; rt: rtString; // #135.2
      s: '-0.0000000000000000000000000000000000000000123456789')
  );

  res1 = '<foo xmlns:baz1="http://xsl.lotus.com/ns1" xmlns:baz2="http://xsl.lotus.com/ns2"/>';

  nameTests: array[0..17] of TTestRec3 = (
    (data: str30; re: res1; expr: 'namespace-uri(baz1:a/@baz2:attrib1)'; rt: rtString; s: ''), // #30
    (data: str30; re: res1; expr: 'namespace-uri(baz2:b/@baz1:attrib2)'; rt: rtString; s: 'http://xsl.lotus.com/ns1'), // #31
    (data: str30; re: res1; expr: 'name(*)'; rt: rtString; s: 'ns1:a'),       // #32
    (data: str30; re: res1; expr: 'name(baz1:a)'; rt: rtString; s: 'ns1:a'),  // #33
    (data: str30; re: res1; expr: 'name(baz2:b)'; rt: rtString; s: 'b'),      // #34
    (data: str30; re: res1; expr: 'name(baz1:a/@baz2:attrib1)'; rt: rtString; s: ''),            // #35
    (data: str30; re: res1; expr: 'name(baz2:b/@baz1:attrib2)'; rt: rtString; s: 'ns1:attrib2'), // #36

    (data: str30; re: res1; expr: 'local-name(baz2:b)'; rt: rtString; s: 'b'), // namespace07
    (data: str30; re: res1; expr: 'local-name(baz2:b/@baz1:attrib2)'; rt: rtString; s: 'attrib2'), // namespace09
    (data: str30; re: res1; expr: 'local-name()'; rt: rtString; s: 'doc'),      // namespace26
    (data: str30; re: res1; expr: 'namespace-uri()'; rt: rtString; s: 'http://xsl.lotus.com/ns2'), // namespace27

    (data: ns11; re: res1; expr: 'namespace-uri(baz1:a-two)'; rt: rtString; s: 'http://xsl.lotus.com/ns1'), // namespace11
    (data: ns11; re: res1; expr: 'namespace-uri(baz1:a-two/@attrib1)'; rt: rtString; s: ''),
    (data: ns11; re: res1; expr: 'namespace-uri(baz2:b-three)'; rt: rtString; s: 'http://xsl.lotus.com/ns2'),
    (data: ns11; re: res1; expr: 'namespace-uri(baz2:b-three/@baz1:attrib2)'; rt: rtString; s: 'http://xsl.lotus.com/ns1'),
{*} (data: ns11; re: res1; expr: 'namespace-uri(baz2:b-three/c-four)'; rt: rtString; s: ''),
    (data: ns11; re: res1; expr: 'namespace-uri(bogus)'; rt: rtString; s: ''),

    (data: str30; re: res1; expr: 'name(baz1:*)'; rt: rtString; s: 'ns1:a')
  );

  ax114='<doc>'+
  '<foo att1="c">'+
  '  <foo att1="b">'+
  '     <foo att1="a"/>'+
  '  </foo>'+
  '</foo>'+
  '<baz/>'+
  '</doc>';

  ax115='<doc>'+
  '<foo att1="c"/>'+
  '<foo att1="b"/>'+
  '<foo att1="a"/>'+
  '<baz/>'+
  '</doc>';


  ax117='<chapter title="A" x="0">'+
  '<section title="A1" x="1">'+
  '  <subsection title="A1a" x="2">hello</subsection>'+
  '  <subsection title="A1b">ahoy</subsection>'+
  '</section>'+
  '<section title="A2">'+
  '  <subsection title="A2a">goodbye</subsection>'+
  '  <subsection title="A2b">sayonara</subsection>'+
  '  <subsection title="A2c">adios</subsection>'+
  '</section>'+
  '<section title="A3">'+
  '  <subsection title="A3a">aloha</subsection>'+
  '  <subsection title="A3b">'+
  '    <footnote x="3">A3b-1</footnote>'+
  '    <footnote>A3b-2</footnote>'+
  '  </subsection>'+
  '  <subsection title="A3c">shalom</subsection>'+
  '</section>'+
  '</chapter>';

  AxesTests: array[0..15] of TTestRec = (
    (data: ax117; expr: 'count(//@*)';                        rt: rtNumber; n: 16),
    (data: ax117; expr: 'count(//@title)';                    rt: rtNumber; n: 12),
    (data: ax117; expr: 'count(//section//@*)';               rt: rtNumber; n: 14),
    (data: ax117; expr: 'count(//section//@title)';           rt: rtNumber; n: 11),
    (data: ax117; expr: 'count(/chapter/.//@*)';              rt: rtNumber; n: 16),
    (data: ax117; expr: 'count(/chapter/.//@title)';          rt: rtNumber; n: 12),
    (data: ax117; expr: 'count(/chapter/section[1]//@*)';     rt: rtNumber; n: 5),
    (data: ax117; expr: 'count(/chapter/section[1]//@title)'; rt: rtNumber; n: 3),
    (data: ax117; expr: 'count(/chapter/section[2]//@*)';     rt: rtNumber; n: 4),
    (data: ax117; expr: 'count(/chapter/section[2]//@title)'; rt: rtNumber; n: 4),
    (data: ax117; expr: 'count(/chapter/section[3]//@*)';     rt: rtNumber; n: 5),
    (data: ax117; expr: 'count(/chapter/section[3]//@title)'; rt: rtNumber; n: 4),

    (data: simple; expr: 'local-name(namespace::*[1])';     rt: rtString; s: 'xml'), // namespace28a
    (data: simple; expr: 'name(namespace::*[1])';           rt: rtString; s: 'xml'), // namespace28b
    (data: ax117; expr: 'name(//subsection[@title="A3b"]/@title/parent::*)'; rt: rtString; s: 'subsection'),   // axes96 modified
    (data: ax117; expr: 'name(//subsection[@title="A3b"]/@title/ancestor::*[1])'; rt: rtString; s: 'subsection')  // axes97 modified
  );

  AxesTests2: array[0..3] of TTestRec3 = (
    (data: ax114; re: '//baz'; expr: 'preceding::foo[1]/@att1';    rt: rtNodeStr; s: 'a'),
    (data: ax114; re: '//baz'; expr: '(preceding::foo)[1]/@att1';  rt: rtNodeStr; s: 'c'),         // won't parse
    (data: ax115; re: '//baz'; expr: 'preceding-sibling::foo[1]/@att1';    rt: rtNodeStr; s: 'a'),
    (data: ax115; re: '//baz'; expr: '(preceding-sibling::foo)[1]/@att1';  rt: rtNodeStr; s: 'c')  // won't parse
  );

  pred44 = '<doc>'+
  '<element1>'+
    '<child1>Success</child1>'+
    '<child2>child2</child2>'+
  '</element1>'+
  '<element2>'+
    '<child1>Wrong node selected!!</child1>'+
  '</element2>'+
  '<element3>'+
    '<child1>Wrong node selected!!</child1>'+
  '</element3>'+
  '</doc>';

  pred11 = '<doc>'+
  '<a>1</a>'+
  '<a>2'+
  '<achild>target</achild>'+
  '</a>'+
  '<a>3</a>'+
  '<a>target</a>'+
  '</doc>';

  PredicateTests: array [0..4] of TTestRec = (
    (data: pred44; expr: '//child1[parent::element1]'; rt: rtNodeStr; s: 'Success'),  // predicate44
    {should select all but last elements named 'e' }
    (data: math96; expr: 'sum(e[true()=following-sibling::*])'; rt: rtNumber; n: 20), // predicate03
    {should select two first elements}
    (data: math96; expr: 'sum(e[8=following-sibling::*])'; rt: rtNumber; n: 12),      // predicate05
    (data: pred11; expr: 'a["target"=descendant::*]'; rt: rtNodeStr; s: '2target'),    // predicate06
    (data: pred11; expr: 'a[following-sibling::*=descendant::*]'; rt: rtNodeStr; s: '2target')  // predicate11




  );
{$warnings on}

var
  FailCount: Integer = 0;  

procedure CheckResult(const t: TTestRec; r: TXPathVariable); overload;
begin
  case t.rt of
    rtBool:
    begin
      if (r is TXPathBooleanVariable) and (r.AsBoolean = t.b) then
        Exit;
      writeln;
      writeln('Failed: ', t.expr);
      writeln('Expected: ', t.b, ' got: ', r.AsBoolean);
    end;

    rtNumber:
    begin
      if (r is TXPathNumberVariable) then
      begin
        if IsNan(t.n) then
        begin
          if IsNan(r.AsNumber) then
            Exit;
        end
        else
        begin
          if IsInfinite(t.n) and (t.n = r.AsNumber) then
            Exit;
          if not IsNan(TXPathNumberVariable(r).Value) and SameValue(TXPathNumberVariable(r).Value, t.n) then
            Exit;
        end;  
      end;
      writeln;
      writeln('Failed: ', t.expr);
      writeln('Expected: ', t.n, ' got: ', r.AsNumber);
    end;

    rtString:
    begin
      if (r is TXPathStringVariable) and (r.AsText = DOMString(t.s)) then
        Exit;
      writeln;  
      writeln('Failed: ', t.expr);
      writeln('Expected: ', DOMString(t.s), ' got: ', r.AsText);
    end;
    rtNodeStr:
    begin
      if (r is TXPathNodeSetVariable) and (r.AsNodeSet.Count = 1) and (r.AsText = DOMString(t.s)) then
        Exit;
      writeln;  
      writeln('Failed: ', t.expr);
      if r.AsNodeSet.Count > 1 then
        writeln('Result is not a single node');
      writeln('Expected: ', DOMString(t.s), ' got: ', r.AsText);
    end;
  end;
  Inc(FailCount);
end;

procedure CheckResult(const t: TTestRec3; r: TXPathVariable); overload;
var
  temp: TTestRec;
begin
  temp.data := t.data;
  temp.expr := t.expr;
  temp.rt := t.rt;
  temp.n := t.n;
  CheckResult(temp, r);
end;


function ParseString(const data: string): TXMLDocument;
var
  parser: TDOMParser;
  src: TXMLInputSource;
begin
  parser := TDOMParser.Create;
  try
    parser.Options.PreserveWhitespace := True;
    parser.Options.Namespaces := True;
    src := TXMLInputSource.Create(data);
    try
      parser.Parse(src, Result);
    finally
      src.Free;
    end;
  finally
    parser.Free;
  end;
end;

procedure DoSuite(const tests: array of TTestRec);
var
  i: Integer;
  doc: TXMLDocument;
  rslt: TXPathVariable;
begin
  for i := 0 to High(tests) do
  begin
    if tests[i].data <> '' then
      doc := ParseString(tests[i].data)
    else
      doc := TXMLDocument.Create;
    try
      try
        rslt := EvaluateXPathExpression(tests[i].expr, doc.DocumentElement);
        try
          CheckResult(tests[i], rslt);
        finally
          rslt.Free;
        end;
      except
        writeln;
        writeln('Failed: ', tests[i].expr);
        SysUtils.ShowException(ExceptObject, ExceptAddr);
        Inc(FailCount);
      end;
    finally
      doc.Free;
    end;
  end;
end;

procedure DoSuite_WithResolver(const tests: array of TTestRec3);
var
  i: Integer;
  doc: TXMLDocument;
  rslt: TXPathVariable;
  nsdoc: TXMLDocument;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  resolver: TXPathNSResolver;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  for i := 0 to High(tests) do
  begin
    doc := ParseString(tests[i].data);
    try
      nsdoc := ParseString(tests[i].re);
      try
        try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          resolver := TXPathNSResolver.Create(nsdoc.DocumentElement);
          try
            rslt := EvaluateXPathExpression(tests[i].expr, doc.DocumentElement, resolver);
            try
              CheckResult(tests[i], rslt);
            finally
              rslt.Free;
            end;
          finally
            resolver.Free;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          rslt := EvaluateXPathExpression(tests[i].expr, doc.DocumentElement, nsdoc.DocumentElement);
          try
            CheckResult(tests[i], rslt);
          finally
            rslt.Free;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          end;
        except
          writeln;
          writeln('Failed: ', tests[i].expr);
          SysUtils.ShowException(ExceptObject, ExceptAddr);
          Inc(FailCount);
        end;
      finally
        nsdoc.Free;
      end;
    finally
      doc.Free;
    end;
  end;
end;

procedure DoSuite_WithContext(const tests: array of TTestRec3);
var
  i: Integer;
  doc: TXMLDocument;
  rslt: TXPathVariable;
  context: TXPathVariable;
  ctxNs: TNodeSet;
begin
  for i := 0 to High(tests) do
  begin
    doc := ParseString(tests[i].data);
    try
      context := EvaluateXPathExpression(tests[i].re, doc.DocumentElement);
      try
        try
          ctxNs := context.AsNodeSet;
          if ctxNs.Count <> 1 then
            raise Exception.CreateFmt('Context expression "%s" does not evaluate to a single node', [tests[i].re]);
          rslt := EvaluateXPathExpression(tests[i].expr, TDOMNode(ctxNs[0]));
          try
            CheckResult(tests[i], rslt);
          finally
            rslt.Free;
          end;
        except
          writeln;
          writeln('Failed: ', tests[i].expr);
          SysUtils.ShowException(ExceptObject, ExceptAddr);
          Inc(FailCount);
        end;
      finally
        context.Free;
      end;
    finally
      doc.Free;
    end;
  end;
end;

begin
  DoSuite(BaseTests);
  DoSuite(CompareTests);
  DoSuite(NodesetCompareTests);  
  DoSuite(EqualityTests);
  DoSuite(FloatTests);
  DoSuite(FunctionTests);
  DoSuite(StringTests);
  DoSuite(AxesTests);
  DoSuite_WithContext(AxesTests2);
  DoSuite_WithResolver(nameTests);
  DoSuite(PredicateTests);

  writeln;
  writeln('Total failed tests: ', FailCount);
end.
