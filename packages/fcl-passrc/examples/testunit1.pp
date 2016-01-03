//This is only for testing the parser, it is not intended to be runable in a real
//program but for checking the contructs to be parsed well.
//All statements are written like testparser would print them out to diff the 
//result with this file again to show differences. 
//Based on /utils/fpdoc/testunit.pp
{$mode objfpc}
{$h+}
unit testunit1;

interface

 uses 
  SysUtils,Classes;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

resourcestring
 SParserErrorAtToken = 'parser error at token';
 
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 const
  AnIntegerConst=1;
  AStringConst='Hello, World!';
  AFLoatconst=1.23;
  ABooleanConst=True;
  ATypedConst: Integer=3;
  AnArrayConst: Array[1..3] of Integer=(1,2,3);
  ARecordConst: TMethod=(Code:nil;Data:nil);
  ASetConst=[true,false];
  ADeprecatedConst=1 deprecated;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ADeprecatedConst2 = 2 deprecated 'use another const';
     
 Type
  TLineEndStr = string [3];

  TDeprecatedType = Integer deprecated;
  TDeprecatedRecord = Record
    x,Y : Integer; 
  end deprecated;
  TDeprecatedFieldsRecord = Record
    x,Y : Integer deprecated; 
  end;
  TDeprecatedFieldsRecord2 = Record
    x,Y : Integer deprecated
  end;
  TAnEnumType=(one,two,three);
  arangetypealias = type 0..$FF;
  TASetType=set of TAnEnumType;
  TIntegerSet = Set of 0..SizeOf(Integer)*8-1;
  TAnArrayType=Array[1..10] of Integer;
  TASubRangeType=one..two;
  TABooleanArrayType=Array[Boolean] of Integer;  
  TDay = (monday,tuesday,wednesday,thursday,friday,saturday,sunday);
  TShortDay = (mon,tue,wed,thu,fri,sat,sun);
  TShortDays = set of TShortDay;
  TDays = set of TDay;
  TMyInteger = Integer;
  ADouble = type double;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   
 Type
  TAnEnumType=(one,two,three);
  TASetType=set of TAnEnumType;
  TAnArrayType=Array[1..10] of Integer;
  TASubRangeType=one..two;
  TABooleanArrayType=Array[Boolean] of Integer;  
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
  TARecordType=record
                   X,Y: Integer;
                   Z: String;
                      end;
  TAVariantRecordType=record
                          A: String;
                          Case Integer of
                        1: (X,Y : Integer);
                        2: (phi,Omega : Real);
                         end; 
  TAVariantRecordType2=record
                          A: String;
                          Case Atype : Integer of
                            1 : (X,Y : Integer);
                            2 : (phi,Omega : Real);
                          end; 
                          
  MyRec = Record  
          X : Longint;  
          Case byte of  
            2 : (Y : Longint;  
                 case byte of  
                 3 : (Z : Longint);  
                 );  
          end;                           
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

TYPE
   PPoint = ^TPoint;
   TPoint = OBJECT
      X, Y: Sw_Integer;
   END;

   PRect = ^TRect;
   TRect = OBJECT
      A, B: TPoint;                                { Corner points }
      FUNCTION Empty: Boolean;
      FUNCTION Equals (R: TRect): Boolean;
      FUNCTION Contains (P: TPoint): Boolean;
      PROCEDURE Copy (R: TRect);
      PROCEDURE Union (R: TRect);
      PROCEDURE Intersect (R: TRect);
      PROCEDURE Move (ADX, ADY: Sw_Integer);
      PROCEDURE Grow (ADX, ADY: Sw_Integer);
      PROCEDURE Assign (XA, YA, XB, YB: Sw_Integer);
   END;
               

  TNotifyEvent = Procedure (Sender : TObject) of object;

  TNestedProcedure = Procedure (Sender : TObject) is nested;

  TNotifyEvent2 = Function (Sender : TObject) : Integer of object;
 
                          
//  TADeprecatedType = Integer deprecated;
  TMyChildClass = Class;
  MyInterface = Interface;
  
=======
                          
//  TADeprecatedType = Integer deprecated;

>>>>>>> graemeg/cpstrnew
=======
                          
//  TADeprecatedType = Integer deprecated;

>>>>>>> graemeg/cpstrnew
=======
                          
//  TADeprecatedType = Integer deprecated;

>>>>>>> graemeg/cpstrnew
=======
                          
//  TADeprecatedType = Integer deprecated;

>>>>>>> origin/cpstrnew
  { TMyParentClass }

  TMyParentClass=Class(TComponent)
  Private 
    FI: Integer;
    Function GetA(AIndex: Integer): String;
    Function GetIP(AIndex: integer): String;
    procedure SetA(AIndex: Integer; const AValue: String);
    procedure SetIP(AIndex: integer; const AValue: String);
    Procedure WriteI(AI: Integer);
    Function ReadI: Integer;
  Protected
    Procedure AProtectedMethod;
    Property AProtectedProp: Integer Read FI Write FI;  
  Public
    Constructor Create(AOwner: TComponent); override;
    Destructor Destroy; override;
    Procedure AVirtualProc; virtual;
    Procedure AnAbstractProc; virtual; abstract;
    Procedure AMessageProc(var Msg);message 123;
    Procedure AStringMessageProc(var Msg);message '123';
    Procedure ADeprecatedProc; deprecated;
    Procedure APlatformProc; Platform;
    Property IntProp: Integer Read FI Write Fi;
    Property IntROProp: Integer Read FI;
    Property GetIntProp: Integer Read ReadI Write WriteI;
    Property AnArrayProp[AIndex: Integer]: String Read GetA Write SetA;
    Property AnIndexProp: String Index 1 Read GetIP Write SetIP;
    Property AnIndexProp2: String Index 2 Read GetIP Write SetIP;
  Published
    Procedure SomePublishedMethod;
  end;
  
  { TMyChildClass }

  TMyChildClass=Class(TMyParentClass)
  Public
    Procedure AVirtualProc; Override;
    Procedure AnAbstractProc; Override;
  Published
    Property AProtectedProp;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TC = TMyChildClass;

  TPasFunctionType=Class(TObject)
  public
    destructor Destroy; override;
    Class Function TypeName: string;
    Function ElementTypeName: string; 
    Function GetDeclaration(Full: boolean): string; 
    Procedure Something;  strict
  Private  
    Procedure SomethingElse;
  public
    ResultEl: TObject;
  end; 

  TPropModifiers = Class(TObject)
  Private
    FB : Integer;
    Function IsStored : Boolean;
    Function GetI(AI : Integer) : Integer;
    Procedure SetI(AI : Integer; AVal : Integer);
  Published
    Property A : Integer Read FB Write FB Stored False;
    Property B : Integer Read FB Write FB Stored True;
    Property C : Integer Read FB Write FB Stored IsStored;
    Property D : Integer Read FB Write FB Default 1;
    Property E : Integer Read FB Write FB Stored True Default 1;
  Public
    Property Ints[AI : Integer] : Integer Read GetI Write SetI; default;
  end;
  
  TPropModifiers2 = class(TPropModifiers)
  Public
    Property Ints[AI : Integer] : Integer Read GetI Write SetI; default; deprecated;
  end;                          
  
  TEdit = Class(TObject)
    Text : String;
  end;
  
var
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  
 TPasFunctionType=Class(TPasProcedureType)
  public
    destructor Destroy; override;
    Class Function TypeName: string; override;
    Function ElementTypeName: string; override;
    Function GetDeclaration(Full: boolean): string; override;
  public
    ResultEl: TPasResultElement;
  end; 
                        
 var
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
  ASimpleVar: Integer;  
  ATypedVar: TMethod;
  ARecordVar: Record
                 A,B: Integer;
               end;
  AnArrayVar: Array[1..10] of Integer;
  ATypedArray: Array[TanEnumType] of Integer;
  AInitVar: Integer=1;
  
  ADeprecatedVar: Integer deprecated;
  ACVarVar: Integer ; cvar;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  AnExternalVar1: Integer; external;
  AnExternalVar2: Integer; external name 'avar';
  AnExternalLibVar: Integer; external 'library' name 'avar';
  APublicVar : String; public;
  APublicVar2 : String; public name 'ANAME';
  APublicVar3 : String; export;
  APublicVar4 : String; export name 'nono';
  APublicVar5 : String; cvar; external;
  APublicVar6 : String; external name 'me';
  APublicVar7 : String deprecated; external name 'me';
=======
  AnExternalVar: Integer ;external name 'avar';
  AnExternalLibVar: Integer ;external 'library' name 'avar';
>>>>>>> graemeg/cpstrnew
=======
  AnExternalVar: Integer ;external name 'avar';
  AnExternalLibVar: Integer ;external 'library' name 'avar';
>>>>>>> graemeg/cpstrnew
=======
  AnExternalVar: Integer ;external name 'avar';
  AnExternalLibVar: Integer ;external 'library' name 'avar';
>>>>>>> graemeg/cpstrnew
=======
  AnExternalVar: Integer ;external name 'avar';
  AnExternalLibVar: Integer ;external 'library' name 'avar';
>>>>>>> origin/cpstrnew
      
 Procedure SimpleProc;
 Procedure OverloadedProc(A: Integer);
 Procedure OverloadedProc(B: String);
 Function SimpleFunc: Integer;
 Function OverloadedFunc(A: Integer): Integer;
 Function OverloadedFunc(B: String): Integer;  

 Procedure ConstArgProc(const A: Integer); 
 Procedure VarArgProc(var A: Integer); 
 Procedure OutArgProc(out A: Integer); 
 Procedure UntypedVarArgProc(var A); 
 Procedure UntypedConstArgProc(const A); 
 Procedure UntypedOutArgProc(out A); 

 Procedure ArrayArgProc(A: TAnArrayType);
 Procedure OpenArrayArgProc(A: Array of string);
 Procedure ConstArrayArgProc(A: Array of const);

 Procedure externalproc; external;
 Procedure externalnameProc; external name 'aname';
 Procedure externallibnameProc; external 'alibrary' name 'aname';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 Function  hi(q : QWord) : DWord;   [INTERNPROC: fpc_in_hi_qword];

{$define extdecl:=cdecl}
Type
  FontEnumProc = function (var ELogFont:TEnumLogFont; var Metric:TNewTextMetric;
      FontType:longint; Data:LParam):longint; extdecl;
      
 
Type
 generic TFPGListEnumerator<T> = class(TObject)
 protected
    FList: TFPList;
    FPosition: Integer;
    function GetCurrent: T;
 end;                 
 TFPGListEnumeratorSpec = specialize TFPGListEnumerator<TPasFunctionType>; 

 
=======

  
>>>>>>> graemeg/cpstrnew
=======

  
>>>>>>> graemeg/cpstrnew
=======

  
>>>>>>> graemeg/cpstrnew
=======

  
>>>>>>> origin/cpstrnew
Implementation


 Procedure SimpleProc;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  procedure  SubProc;
  Var S : String;
  begin
   s:= s+'a';
  end;
 Var
   a,B,c,i : integer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 procedure  SubProc;
  begin
   s:= s+'a';
  end;
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

 begin
  a:= 1;
  c:= a+b;
  for i:= 1 to 10 do 
    write(a);
 end;

 Procedure OverloadedProc(A: Integer);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 Var
   i : integer;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 begin
  if i=1 then ;
 end;

 Procedure OverloadedProc(B: String);
 begin
 end;

 Function SimpleFunc: Integer;
 begin
 end;

 Function OverloadedFunc(A: Integer): Integer; 
 begin
 end;

 Function OverloadedFunc(B: String): Integer;  
 begin
 end;

 Procedure ArrayArgProc(A: TAnArrayType);
 begin
 end;

 Procedure OpenArrayArgProc(A: Array of String);
 begin
 end;

 Procedure ConstArrayArgProc(A: Array of const);
 begin
 end;

 Procedure ConstArgProc(const A: Integer); 
 begin
 end;

 Procedure VarArgProc(var A: Integer); 
 begin
 end;

 Procedure OutArgProc(out A: Integer); 
 begin
 end;

 Procedure UntypedVarArgProc(var A); 
 begin
 end;

 Procedure UntypedConstArgProc(const A); 
 begin
 end;

 Procedure UntypedOutArgProc(out A); 
 begin
 end;

{ TMyChildClass }
 procedure TMyChildClass.AVirtualProc;
 begin
  inherited AVirtualProc;
 end;

 procedure TMyChildClass.AnAbstractProc;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 
 procedure  SubCProc;
 
   Var sc : string;
   
=======
 procedure  SubCProc;
>>>>>>> graemeg/cpstrnew
=======
 procedure  SubCProc;
>>>>>>> graemeg/cpstrnew
=======
 procedure  SubCProc;
>>>>>>> graemeg/cpstrnew
=======
 procedure  SubCProc;
>>>>>>> origin/cpstrnew
  begin
   sc:= sc+'ac';
  end;

 begin
  // Cannot call ancestor
 end;

{ TMyParentClass }
 procedure TMyParentClass.WriteI(AI: Integer);
 begin
 end;

 Function TMyParentClass.GetA(AIndex: Integer): String;
 begin
 end;

 Function TMyParentClass.GetIP(AIndex: integer): String;
 begin
 end;

 procedure TMyParentClass.SetA(AIndex: Integer; const AValue: String);
 begin
 end;

 procedure TMyParentClass.SetIP(AIndex: integer; const AValue: String);
 begin
 end;

 Function TMyParentClass.ReadI: Integer;
 begin
 end;

 procedure TMyParentClass.AProtectedMethod;
 begin
 end;

 constructor TMyParentClass.Create(AOwner: TComponent);
 begin
  inherited Create(AOwner);
 end;

 destructor TMyParentClass.Destroy;
 begin
  inherited Destroy;
 end;

 procedure TMyParentClass.AVirtualProc;
 begin
 end;

 procedure TMyParentClass.AMessageProc(var Msg);
 begin
 end;

 procedure TMyParentClass.AStringMessageProc(var Msg);
 begin
 end;

 procedure TMyParentClass.ADeprecatedProc;
 begin
 end;

 procedure TMyParentClass.APlatformProc;
 begin
 end;

 procedure TMyParentClass.SomePublishedMethod;
 begin
 end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> graemeg/cpstrnew
=======
 
>>>>>>> origin/cpstrnew
 Class Function TPasFunctionType.TypeName: String;
 begin
  Result:= 'Function';
 end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Type
  TI = Class(TComponent)
  Public
    FP : Integer;
    Procedure SetP1(A : Integer); virtual;
    Procedure M1;virtual;
    Function F1  : Integer; virtual;
    procedure test; virtual;
    property P : Integer Read FP Write SetP1;
  end;
  
  Procedure TI.M1;
  begin
  end;
  Procedure TI.Test;
  begin
  end;
  Function TI.F1 : Integer; 
  begin
  Result:=0;
  end;
  Procedure TI.SetP1(A : Integer);
  begin
    FP:=A;
  end;
  
TYpe
  TI2 = Class(TI)
  procedure write(s : string);
  Procedure SetP1(A : Integer); override;
  Procedure M1;override;
  Procedure Test;override;
  Function F1 : integer; override;
  procedure donothing;
  property P : Integer Read F1 Write SetP1;
  end;
  Procedure TI2.M1;
  begin
    Inherited;
  end;
  Procedure TI2.Write(s : string);
  begin
    writeln(s);
  end;
  Function TI2.F1 :Integer; 
  begin
     Result:=0;
  end;
  Procedure TI2.Test;
  begin
  if true then
    Inherited Test
  else
    DoNothing;
    Inherited test;
   if true then
     Inherited
   else
     DoNothing;
  end;
  Procedure TI2.DoNothing;
    function escapetext(s : string) : string;
    begin
    end;
  var
  Atext : string;
  begin
    Self.Write(EscapeText(AText)); 
    TComponent.Create(Self);
  end;
  Procedure TI2.SetP1(A : Integer);
  begin
    FP:=A;
    Inherited P:= 3;
    Inherited SetP1(3);
    Inherited P:= Ord(A);
  end;


 procedure usage;
 begin
 end;
 Procedure DoSomething;
 begin
 end;
 Procedure DoSomethingElse;
 begin
 end;
 procedure stat1;
 begin
 end;
 procedure stat2;
 begin
 end;
 procedure stat3;
 begin
 end;
 procedure stat4;
 begin
 end;
 procedure stat5;
 begin
 end;
 procedure stat6;
 begin
 end;
 procedure stat7;
 begin
 end;
  procedure stat8;
 begin
 end;
 procedure stat9;
 begin
 end;
 procedure doit;
 begin
 end;
 procedure statement;
 begin
 end;
 procedure work;
 begin
 end;
 procedure kissdwarf(i : integer);
 
 begin
   writeln('kiss dwarf',i);
 end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 procedure Statements;
 const
  cint=1;
  cint1=-1;
  creal=3.1415;
  Addi=1+2;
  Subs=2-3;
  Muti=3*3;
  Divi=3/5;
  //Powe=2^3;
  Modu=5 mod 3;
  IDiv=5 div 3;
  fals= not TRUE;
  cand=true and false;
  cor=true or false;
  cxor=true xor false;
  lt=2<3;
  gt=3>2;
  let=2<=3;
  get=3>=2;
  LeftShift=2 shl 3;
  RightShift=2 shr 3;
  ConstString='01'+'ab';

 Type
  Passenger=Record
                Name: String[30];
                Flight: String[10];
  end;

 Type 
  AR=record
      X,Y: LongInt;
     end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  TScanner = record
   currow,curcolumn : integer;
   curfilename : string;
  end;  

  //PAR = Record;
 var
  msg,curtokenname : string;
=======
  //PAR = Record;
 var
>>>>>>> graemeg/cpstrnew
=======
  //PAR = Record;
 var
>>>>>>> graemeg/cpstrnew
=======
  //PAR = Record;
 var
>>>>>>> graemeg/cpstrnew
=======
  //PAR = Record;
 var
>>>>>>> origin/cpstrnew
  TheCustomer: Passenger;
  L: ^LongInt;
  P: PPChar;
  S,T: Ar;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  M, X,Y : Double;
  Done : Boolean;
  Weather,Good: Boolean;  
  c : char;
  j,dwarfs,i,Number,Block : integer;
  exp1,exp2,exp3,exp4,exp5,exp6,exp7,exp8,exp9 : boolean;
  o : Tobject;
  day,today : tday;
  A,B,D : Passenger;
  E : Exception;
  scanner : tscanner;
    
 begin
  O:=Nil;
=======
      
 begin
>>>>>>> graemeg/cpstrnew
=======
      
 begin
>>>>>>> graemeg/cpstrnew
=======
      
 begin
>>>>>>> graemeg/cpstrnew
=======
      
 begin
>>>>>>> origin/cpstrnew
  X:= X+Y;
  //EparserError on C++ style
  //X+=Y;      { Same as X := X+Y, needs -Sc command line switch}
  //x-=y;
  //X/=2;      { Same as X := X/2, needs -Sc command line switch}
  //x*=y;
  Done:= False;
  Weather:= Good;
  //MyPi := 4* Tan(1); warum * ?
  L^:= 3;
  P^^:= 'A';
  Usage;
  WriteLn('Pascal is an easy language !');
  Doit();
  //label jumpto;
  //Jumpto :
  //  Statement;
  //Goto jumpto;

  Case i of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    6: DoSomething;
=======
    3: DoSomething;
>>>>>>> graemeg/cpstrnew
=======
    3: DoSomething;
>>>>>>> graemeg/cpstrnew
=======
    3: DoSomething;
>>>>>>> graemeg/cpstrnew
=======
    3: DoSomething;
>>>>>>> origin/cpstrnew
    1..5: DoSomethingElse;
  end;

  Case C of  
    'a': WriteLn('A pressed');
    'b': WriteLn('B pressed');
    'c': WriteLn('C pressed');
  else  
   WriteLn('unknown letter pressed : ',C);
  end;

  Case C of
    'a','e','i','o','u': WriteLn('vowel pressed');
    'y': WriteLn('This one depends on the language');
  else
   WriteLn('Consonant pressed');
  end;

  Case Number of
    1..10: WriteLn('Small number');
    11..100: WriteLn('Normal, medium number');
  else
   WriteLn('HUGE number');
  end;

  case block of
    1: begin
	writeln('1');
	end;
    2: writeln('2');
  else
    writeln('3');
    writeln('4');
  end;

  If exp1 Then
    If exp2 then
      Stat1
  else
    stat2;

  If exp3 Then
      begin
      If exp4 then
	Stat5
      else
	stat6
      end;

  If exp7 Then
    begin
    If exp8 then
	Stat9
    end
  else
    stat2;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 if o is TObject then
  begin
    write('object');
  end
  else 
    if o is TMyParentClass then 
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 if i is integer then
  begin
    write('integer');
  end
  else 
    if i is real then 
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
  begin
    write('real');
  end
  else 
    write('0'); 

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if Today in [Monday..Friday] then
=======
  if Today in[Monday..Friday] then
>>>>>>> graemeg/cpstrnew
=======
  if Today in[Monday..Friday] then
>>>>>>> graemeg/cpstrnew
=======
  if Today in[Monday..Friday] then
>>>>>>> graemeg/cpstrnew
=======
  if Today in[Monday..Friday] then
>>>>>>> origin/cpstrnew
    WriteLn('Must work harder')
  else
    WriteLn('Take a day off.');

  for Day:= Monday to Friday do 
    Work;
  for I:= 100 downto 1 do
    WriteLn('Counting down : ',i);
  for I:= 1 to 7*dwarfs do 
    KissDwarf(i);

  for i:= 0 to 10 do
    begin
    j:= 2+1;
    write(i,j);
    end;

  repeat
    WriteLn('I =',i);
    I:= I+2;
  until I>100;
    
  repeat
    X:= X/2;
  until x<10e-3;

  I:= I+2;
  while i<=100 do
    begin
     WriteLn('I =',i);
     I:= I+2;
    end;
    X:= X/2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    while i>=10e-3 do 
      dec(i);

    while i>0 do 
    while j>0 do 
      begin
	dec(i);
	dec(j);
      end;

    while i>0 do
    if i>2 then 
     dec(i)
    else 
     dec(i,2);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    while x>=10e-3 do 
      dec(x);

    while x>0 do 
    while y>0 do 
      begin
	dec(x);
	dec(y);
      end;

    while x>0 do
    if x>2 then 
     dec(x)
    else 
     dec(x,2);
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

      X:= 2+3;

    TheCustomer.Name:= 'Michael';
    TheCustomer.Flight:= 'PS901';

    With TheCustomer do
      begin
       Name:= 'Michael';
       Flight:= 'PS901';
      end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  With A,B,D do
=======
  With A,B,C,D do
>>>>>>> graemeg/cpstrnew
=======
  With A,B,C,D do
>>>>>>> graemeg/cpstrnew
=======
  With A,B,C,D do
>>>>>>> graemeg/cpstrnew
=======
  With A,B,C,D do
>>>>>>> origin/cpstrnew
   Statement;

    With A do
     With B do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
      With C do
>>>>>>> graemeg/cpstrnew
=======
      With C do
>>>>>>> graemeg/cpstrnew
=======
      With C do
>>>>>>> graemeg/cpstrnew
=======
      With C do
>>>>>>> origin/cpstrnew
       With D do 
        Statement;

    S.X:= 1;S.Y:= 1;
    T.X:= 2;T.Y:= 2;
    With S,T do
      WriteLn(X,' ',Y);

    {asm
      Movl $1,%ebx
      Movl $0,%eax
      addl %eax,%ebx
    end; ['EAX','EBX'];}

    try
	try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	  M:= Y;
	except
	  on excep: EParserError do
	    begin
	      writeln(excep.message,' : ',excep.classname);
	      raise ;
	  end;
	end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
	  M:= ParseSource(E,cmdl,'linux','i386');
	except
	  on excep: EParserError do
	    begin
	      writeln(excep.message,' line:',excep.row,' column:',excep.column,' file:',excep.filename);
	      raise ;
	  end;
	end;
	Decls:= M.InterfaceSection.Declarations;
	for I:= 0 to Decls.Count-1 do
	  Writeln('Interface item ',I,': ');

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
	FreeAndNil(M);
    finally
	FreeAndNil(E)
   end;
   
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif});
=======
   raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn);
>>>>>>> graemeg/cpstrnew
=======
   raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn);
>>>>>>> graemeg/cpstrnew
=======
   raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn);
>>>>>>> graemeg/cpstrnew
=======
   raise EParserError.Create(Format(SParserErrorAtToken, [Msg, CurTokenName]) {$ifdef addlocation}+' ('+inttostr(scanner.currow)+' '+inttostr(scanner.curcolumn)+')'{$endif},Scanner.CurFilename, Scanner.CurRow, Scanner.CurColumn);
>>>>>>> origin/cpstrnew
    
    // try else
 end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 function addone : integer;
 begin
 end;
  procedure myproc;
  begin
  end;
 procedure Expression;

  Var
    A,b,c,d,e,f,i,j : Integer;
    x : double;
    u : Boolean;
    fu : function : integer;
    ad : boolean;
    z : tdays;
    today,tomorrow : tday;
    bs : set of byte;
    cs : set of char;
    cc : char;  
    W : TShortDays;
    buffer : array[1..10] of byte;
    P : Pointer;
    SErrMultipleSourceFiles,FileName,Dirname,S : string;
    o,co : tobject;
    
 begin
  x:= a+b *c /(-e+f)*(3 div 2) + 4 mod 5 - 2 shl 3 + 3 shr 1 ;
  b:= (a and not b) or c xor d;
  u:= (i<=2) or (a<>b) or (j>=3);
  u:= (i=1) or (a>b) or (b<a) or (i<>2);
  u:= i in [1..2];

 If Fu=@AddOne Then  
  WriteLn('Functions are equal');

 If Fu()=Addone then  
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 procedure Expression;
 begin
  A:= a+b *c /(-e+f)*3 div 2 + 4 mod 5 - 2 shl 3 + 3 shr 1 ;
  b:= (a and not b) or c xor d;
  u:= i<=2 or a<>b or j>=3;
  u:= i=1 or a>b or b<a or i<>2;
  u:= i in [1..2];

 If F=@AddOne Then  
  WriteLn('Functions are equal');

 If F()=Addone then  
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
  WriteLn('Functions return same values ');

 z:= [today,tomorrow];
 z:= [Monday..Friday,Sunday];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 bs:= [2,3*2,6*2,9*2];
 cs:= ['A'..'Z','a'..'z','0'..'9'];

 i:= Byte('A');
 cc:= Char(48);
 ad:= boolean(1);
 i:= longint(@Buffer);
 i:= Integer('A');
 cc:= Char(225);
 i:= Word(@Buffer);

 B:= Byte(C);

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 z:= [2,3*2,6*2,9*2];
 z:= ['A'..'Z','a'..'z','0'..'9'];

 x:= Byte('A');
 x:= Char(48);
 x:= boolean(1);
 x:= longint(@Buffer);
 x:= Integer('A');
 x:= Char(4875);
 x:= Word(@Buffer);

 B:= Byte(C);
 Char(B):= C;

 TWordRec(W).L:= $FF;
 TWordRec(W).H:= 0;
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
 S:= TObject(P).ClassName;

 P:= @MyProc; //warum @ ? fix pparser 769 ?

 Dirname:= Dirname+'\';

 W:= [mon,tue]+[wed,thu,fri]; // equals [mon,tue,wed,thu,fri]
 W:= [mon,tue,wed]-[wed];     // equals [mon,tue]
 W:= [mon,tue,wed]*[wed,thu,fri]; // equals [wed] warum * ?

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
 (Co as TEdit).Text:= 'Some text';
 Co:= O as TComponent;

 if co is TComponent then ;
 If co is TC then ;

=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 (C as TEdit).Text:= 'Some text';
 C:= O as TComponent;

 if A is TComponent then ;
 If A is B then ;

 Inherited ;
 Inherited Test;

  if true then
    Inherited
  else
    DoNothing;

  if true then
    Inherited Test
  else
    DoNothing;

   Inherited P:= 3;  
   Inherited SetP1(3); 
   Result:= Char(P and $FF);  
   Result:= Char((Inherited P) and $FF);  
   Inherited P:= Ord(AValue);
   Result:= Inherited InterPretOption(Cmd,Arg);
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

  raise Exception.Create(SErrMultipleSourceFiles);

  if Filename<>'' then
	  raise Exception.Create(SErrMultipleSourceFiles);

  if Filename<>'' then
	  raise Exception.Create(SErrMultipleSourceFiles)
	else
	  Filename:= s;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  Self.Write(EscapeText(AText)); 
  TObject.Create(Self);
>>>>>>> graemeg/cpstrnew
=======
  Self.Write(EscapeText(AText)); 
  TObject.Create(Self);
>>>>>>> graemeg/cpstrnew
=======
  Self.Write(EscapeText(AText)); 
  TObject.Create(Self);
>>>>>>> graemeg/cpstrnew
=======
  Self.Write(EscapeText(AText)); 
  TObject.Create(Self);
>>>>>>> origin/cpstrnew
 end;

 constructor TPasPackage.Create(const AName: String; AParent: TPasElement);
 begin
  if (Length(AName)>0)and(AName[1]<>'#') then
   Inherited Create('#'+AName,AParent)
  else
   Inherited Create(AName,AParent);
  Modules:= TList.Create;
 end;         

 Function TPascalScanner.FetchToken: TToken;
 var
  IncludeStackItem: TIncludeStackItem;

 begin
  while true do
  begin
    Result:= DoFetchToken;
     if FCurToken=tkEOF then
      if FIncludeStack.Count>0 then
      begin
        CurSourceFile.Free;
        IncludeStackItem:= TIncludeStackItem(FIncludeStack[FIncludeStack.Count-1]);
        FIncludeStack.Delete(FIncludeStack.Count-1);
        FCurSourceFile:= IncludeStackItem.SourceFile;
        FCurFilename:= IncludeStackItem.Filename;
        FCurToken:= IncludeStackItem.Token;
        FCurTokenString:= IncludeStackItem.TokenString;
        FCurLine:= IncludeStackItem.Line;
        FCurRow:= IncludeStackItem.Row;
        TokenStr:= IncludeStackItem.TokenStr;
        IncludeStackItem.Free;
        Result:= FCurToken;
      end 
    else
      break
    else
      if not PPIsSkipping then
        break;
  end;
 end;  

 Procedure IFS;
 begin
  if true then
   repeat
   until false
  else
    Noting;
 end;           


 Procedure IFS(x: integer); overload;
 begin
  if true then
    case x of
     1: writeln;
     2: write;
   else 
    writeln('#');
   end
  else
    Noting;
 end;

 Procedure IFS1; 
 begin
  if true then
    while true do
     Something
  else
    Noting;
 end;

 Procedure IFS3;
 begin
  if true then
   if true then 
    write
   else 
    writeln;
 end; 

Initialization
 
  hallo:= valid;
end.
