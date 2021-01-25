unit regtcxmlreg;

{$mode objfpc}{$H+}

interface

uses
  fpcunit, testutils, testregistry, testdecorator, Classes, SysUtils, xmlreg;

Type

  { TTestXMLRegistry }

  TTestXMLRegistry = Class(TTestCase)
  private
    FXMLReg: TXmlRegistry;
  Protected
    Procedure Setup; override;
    Procedure TearDown; override;
    Property XMLReg : TXmlRegistry Read FXMLReg;
  Published
    Procedure TestReadBufDataDWord;
    Procedure TestReadBufDataString;
    Procedure TestReadBufDataBinary;
  end;


implementation

{ TTestXMLRegistry }

procedure TTestXMLRegistry.Setup;
begin
  inherited Setup;
  DeleteFile('test.xml');
  FXMLReg:=TXmlRegistry.Create('test.xml');
end;

procedure TTestXMLRegistry.TearDown;
begin
  FreeAndNil(FXMLReg);
  DeleteFile('test.xml');
  inherited TearDown;
end;

procedure TTestXMLRegistry.TestReadBufDataDWord;

Var
  C : Cardinal;
  I : Smallint;
  DS : Integer;
  dt : TDataType;

begin
  XMLReg.SetKey('a',True);
  C:=123456;
  XMLReg.SetValueData('b',dtDWORD,C,SizeOf(C));
  XMLReg.Flush;
  DS:=SizeOf(SmallInt);
  AssertEquals('Cannot read, buffer size too small',False,XMLReg.GetValueData('b',dt,I,ds));
  AssertTrue('Correct data type reported',dt=dtDWord);
  AssertEquals('Correct data buffer size reported',SizeOf(C),DS);

end;

procedure TTestXMLRegistry.TestReadBufDataString;

Var
  S1,S2 : String;
  DS : Integer;
  dt : TDataType;

begin
  XMLReg.SetKey('a',True);
  S1:=StringOfChar('*',100);
  XMLReg.SetValueData('b',dtString,S1[1],Length(S1));
  XMLReg.Flush;
  DS:=SizeOf(S1) div 2;
  S2:=StringOfChar('*',DS);
  AssertEquals('Cannot read, buffer size too small',False,XMLReg.GetValueData('b',dt,S2[1],ds));
  AssertTrue('Correct data type reported',dt=dtString);
  AssertEquals('Correct data buffer size reported',Length(S1),DS);
end;

procedure TTestXMLRegistry.TestReadBufDataBinary;
Var
  S1,S2 : Array of byte;
  I : Smallint;
  DS : Integer;
  dt : TDataType;

begin
  XMLReg.SetKey('a',True);
  SetLength(S1,100);
  For I:=0 to 99 do
    S1[I]:=i;
  XMLReg.SetValueData('b',dtBinary,S1[0],Length(S1));
  XMLReg.Flush;
  DS:=Length(S1) div 4;
  SetLength(S2,DS);
  For I:=0 to DS-1 do
    S2[I]:=i;
  AssertEquals('Cannot read, buffer size too small',False,XMLReg.GetValueData('b',dt,S2[0],DS));
  AssertTrue('Correct data type reported',dt=dtBinary);
  AssertEquals('Correct data buffer size reported',Length(S1),DS);
end;

begin
  RegisterTest(TTestXMLRegistry);
end.

