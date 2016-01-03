{$mode objfpc}
{$h+}
unit testunit;

interface

uses SysUtils, Classes;

Const
  AnIntegerConst  = 1;
  AStringConst    = 'Hello, World!';
  AFLoatconst     = 1.23;
  ABooleanConst   = True;
  ATypedConst : Integer = 3;
  AnArrayConst : Array[1..3] of Integer = (1,2,3);
  ARecordConst : TMethod = (Code:Nil;Data:Nil);
  ASetConst = [true,false];
  ADeprecatedConst = 1 deprecated;
   
Type
  TAnEnumType         = (one,two,three);
  TASetType           = Set of TAnEnumType;
  TAnArrayType        = Array[1..10] of Integer;
  TASubRangeType      = one..two;
  TABooleanArrayType  = Array[Boolean] of Integer;  
  TARecordType        = Record
                         X,Y : Integer;
                         Z : String;
                       end;
  TAVariantRecordType = Record
                          A : String;
                          Case Integer of
                           1 : (X,Y : Integer);
                           2 : (phi,Omega : Real);
                        end; 
  TAVariantRecordType2 = Record
                          A : String;
                          Case Atype : Integer of
                            1 : (X,Y : Integer);
                            2 : (phi,Omega : Real);
                          end; 
                          
//  TADeprecatedType = Integer deprecated;
                        
Var
  ASimpleVar : Integer;  
  ATypedVar  : TMethod;
  ARecordVar : Record
                 A,B : integer;
               end;
  AnArrayVar : Array[1..10] of Integer;
  ATypedArray : Array[TanEnumType] of Integer;
  AInitVar : Integer = 1;
  
  ADeprecatedVar : Integer deprecated;
  ACVarVar : Integer; cvar;
  AnExternalVar : Integer; external name 'avar';
  AnExternalLibVar : Integer; external 'library' name 'avar';
      
Procedure SimpleProc;
Procedure OverloadedProc(A : Integer);
Procedure OverloadedProc(B : String);
Function SimpleFunc : Integer;
Function OverloadedFunc(A: Integer) : Integer;
Function OverloadedFunc(B : String) : Integer;  

Procedure ConstArgProc(Const A : Integer); 
Procedure VarArgProc(Var A : Integer); 
Procedure OutArgProc(Out A : Integer); 
Procedure UntypedVarArgProc(Var A); 
Procedure UntypedConstArgProc(const A); 
Procedure UntypedOutArgProc(Out A); 

Procedure ArrayArgProc (A : TAnArrayType);
Procedure OpenArrayArgProc(A : Array of string);
Procedure ConstArrayArgProc(A : Array of const);

Procedure externalproc; external;
Procedure externalnameProc; external name 'aname';
Procedure externallibnameProc; external 'alibrary' name 'aname';

Type 

  { TMyParentClass }

  TMyParentClass = Class(TComponent)
  Private 
    FI : Integer;
    function GetA(AIndex : Integer): String;
    function GetIP(AIndex: integer): String;
    procedure SetA(AIndex : Integer; const AValue: String);
    procedure SetIP(AIndex: integer; const AValue: String);
    Procedure WriteI(AI : Integer);
    Function ReadI : Integer;
  Protected
    Procedure AProtectedMethod;
    Property AProtectedProp : Integer Read FI Write FI;  
  Public
    Constructor Create(AOwner : TComponent); override;
    Destructor Destroy; override;
    Procedure AVirtualProc; virtual;
    Procedure AnAbstractProc; virtual; abstract;
    Procedure AMessageProc(Var Msg); Message 123;
    Procedure AStringMessageProc(Var Msg); Message '123';
    Procedure ADeprecatedProc; deprecated;
    Procedure APlatformProc; Platform;
    Property IntProp : Integer Read FI Write Fi;
    Property IntROProp : Integer Read FI;
    Property GetIntProp : Integer Read ReadI Write WriteI;
    Property AnArrayProp[AIndex : Integer] : String Read GetA write SetA;
    Property AnIndexProp : String Index 1 Read GetIP Write SetIP;
    Property AnIndexProp2 : String Index 2 Read GetIP Write SetIP;
  Published
    Procedure SomePublishedMethod;
  end;
  
  { TMyChildClass }

  TMyChildClass = Class(TMyParentClass)
  Public
    Procedure AVirtualProc; Override;
    Procedure AnAbstractProc; Override;
  Published
    Property AProtectedProp;
  end;
  
Implementation

Procedure SimpleProc;
begin
end;

Procedure OverloadedProc(A : Integer);
begin
end;

Procedure OverloadedProc(B : String);
begin
end;

Function SimpleFunc : Integer;
begin
end;

Function OverloadedFunc(A: Integer) : Integer;
begin
end;

Function OverloadedFunc(B : String) : Integer;  
begin
end;

Procedure ArrayArgProc (A : TAnArrayType);
begin
end;

Procedure OpenArrayArgProc(A : Array of string);
begin
end;

Procedure ConstArrayArgProc(A : Array of const);
begin
end;

Procedure ConstArgProc(Const A : Integer); 
begin
end;

Procedure VarArgProc(Var A : Integer); 
begin
end;

Procedure OutArgProc(Out A : Integer); 
begin
end;

Procedure UntypedVarArgProc(Var A); 
begin
end;

Procedure UntypedConstArgProc(const A); 
begin
end;

Procedure UntypedOutArgProc(Out A); 
begin
end;


{ TMyChildClass }

procedure TMyChildClass.AVirtualProc;
begin
  inherited AVirtualProc;
end;

procedure TMyChildClass.AnAbstractProc;
begin
  // Cannot call ancestor
end;

{ TMyParentClass }

procedure TMyParentClass.WriteI(AI: Integer);
begin

end;

function TMyParentClass.GetA(AIndex : Integer): String;
begin

end;

function TMyParentClass.GetIP(AIndex: integer): String;
begin

end;

procedure TMyParentClass.SetA(AIndex : Integer; const AValue: String);
begin

end;

procedure TMyParentClass.SetIP(AIndex: integer; const AValue: String);
begin

end;

function TMyParentClass.ReadI: Integer;
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

end.
