{ %target=darwin }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{ %cpu=powerpc,powerpc64,i386,x86_64,arm,aarch64 }
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> graemeg/cpstrnew
=======
{ %cpu=powerpc,powerpc64,i386,x86_64,arm }
>>>>>>> origin/cpstrnew

{ Written by Jonas Maebe in 2009, released into the public domain }

{$mode objfpc}
{$modeswitch objectivec1}
{$packenum 1}
{$packset 1}

procedure check(const name,a,b: string);
begin
  if (a<>b) then
    begin
      writeln('For ',name,' got: "',a,'", expected: "',b,'"');
      halt(1);
    end;
end;

procedure checksimpletypes;
type
  tenum = (ea,eb,ec);
  tprocedure = procedure;
var
  p: pointer;
begin
  check('char',objcencode(char),'C');
  check('widechar',objcencode(widechar),'S');
  check('void',objcencode(p^),'v');

  check('tenum',objcencode(tenum),'C');

  check('shortint',objcencode(shortint),'c');
  check('byte',objcencode(byte),'C');
  check('smallint',objcencode(smallint),'s');
  check('word',objcencode(word),'S');
  check('longint',objcencode(longint),'i');
  check('cardinal',objcencode(cardinal),'I');
  check('int64',objcencode(int64),'q');
  check('qword',objcencode(qword),'Q');
  check('shortstring',objcencode(shortstring),'[256C]');

  check('pointer',objcencode(pointer),'^v');
  
  check('single',objcencode(single),'f');
  check('double',objcencode(double),'d');
  
  check('tprocedure',objcencode(tprocedure),'^?');
  
  check('id',objcencode(id),'@');
  check('NSObject',objcencode(NSObject),'@');
  check('pobjc_class',objcencode(pobjc_class),'#');
  check('selector',objcencode(objcselector('alloc')),':');
end;


procedure checkarrays;
type
  ta = array[5..6] of byte;
  tb = array[1..10] of pointer;
  tc = array[0..3] of tb;
begin
  check('ta',objcencode(ta),'[2C]');
  check('tb',objcencode(tb),'[10^v]');
  check('tc',objcencode(tc),'[4[10^v]]');
end;


procedure checkrecords;
type
  tra=record
    a,b: longint;
  end;
  TStrippedVarRec = record
     case VType : shortint of
       vtInteger    : (VInteger: Longint);
       vtBoolean    : (VBoolean: Boolean);
       vtChar       : (VChar: Char);
       vtWideChar   : (VWideChar: WideChar);
       vtString     : (VString: PShortString);
       vtPointer    : (VPointer: Pointer);
       vtPChar      : (VPChar: PChar);
       vtObject     : (VObject: TObject);
       vtClass      : (VClass: TClass);
       vtPWideChar  : (VPWideChar: PWideChar);
       vtAnsiString : (VAnsiString: Pointer);
       vtInterface  : (VInterface: Pointer);
       vtWideString : (VWideString: Pointer);
       vtInt64      : (VInt64: PInt64);
       vtQWord      : (VQWord: PQWord);
   end;
 tnestedvarrechelper1 = record
   case byte of
     1: (f: single);
     2: (d: double);
 end;
 tnestedvarrechelper2 = record
   x: longint;
   y: shortint;
 end;
 tnestedvarrec = record
   a: longint;
   p: ^tra;
   case byte of
     1: (t: tnestedvarrechelper1);
     2: (t2: tnestedvarrechelper2);
     3: (bb: longint);
 end;
begin
  check('tra',objcencode(tra),'{tra=ii}');
  check('TStrippedVarRec',objcencode(TStrippedVarRec),'{TStrippedVarRec=c(?={?=i}{?=B}{?=C}{?=S}{?=^[256C]}{?=^v}{?=*}{?=^{TObject}}{?=^{TClass}}{?=^S}{?=^v}{?=^v}{?=^v}{?=^q}{?=^Q})}');
  check('TObject',objcencode(TObject),'^{TObject}');
  check('tnestedvarrec',objcencode(tnestedvarrec),'{tnestedvarrec=i^{tra}(?={?={tnestedvarrechelper1=(?={?=f}{?=d})}}{?={tnestedvarrechelper2=ic}}{?=i})}');
end;

procedure checksets;
type
  tset1 = set of 0..4;
  tset2 = set of 0..31;
  tset3 = set of 0..128;
begin
  check('tset1',objcencode(tset1),'{?=[1C]}');
  check('tset2',objcencode(tset2),'{?=[4C]}');
{$ifdef cpui386}
  { for some mysterious reason, sets are always passed by value for cdecl on
    i386 }
  check('tset3',objcencode(tset3),'{?=[17C]}');
{$else cpui386}
  check('tset3',objcencode(tset3),'[17C]');
{$endif cpui386}
end;



begin
  checksimpletypes;
  checkarrays;
  checkrecords;
  checksets;
end.