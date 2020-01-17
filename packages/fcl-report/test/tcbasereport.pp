unit tcbasereport;

{$mode objfpc}{$H+}

{.$define gdebug}

interface

uses
  Classes,
  SysUtils,
  fpcunit,
  testregistry,
  fpexprpars,
  fpCanvas,
  fpReport;

type

  TMyFPReportComponent = class(TFPReportComponent)
  public
    procedure StartLayout; override;
    procedure EndLayout; override;
    procedure StartRender; override;
    procedure EndRender; override;
  end;


  TMyFPReportElement = class(TFPReportElement)
  private
    FChangedCalled: integer;
  public
    procedure CallChange;
    procedure ResetChanged;
    procedure DoChanged; override;
    property ChangedCalled: integer read FChangedCalled;
  end;


  TMyFPReportElementWithChildren = class(TFPReportElementWithChildren)
  private
    FChangedCalled: integer;
  public
    procedure CallChange;
    procedure ResetChanged;
    procedure DoChanged; override;
    property ChangedCalled: integer read FChangedCalled;
  end;


  TMyFPReportPageSize = class(TFPReportPageSize)
  private
    FChangedCalled: integer;
  public
    procedure ResetChanged;
    procedure Changed; override;
    property ChangedCalled: integer read FChangedCalled;
  end;


  TMyFPReportPage = class(TFPReportPage)
  private
    FChangedCalled: integer;
    FPrepareObjectsCalled: integer;
    procedure SetupPage;
  protected
    procedure PrepareObjects(aRTParent: TFPReportElement); override;
  public
    constructor Create(AOwner: TComponent); override;
    procedure ResetChanged;
    procedure DoChanged; override;
    property ChangedCalled: integer read FChangedCalled;
  end;


  TMyReportTitleBand = class(TFPReportCustomTitleBand)
  private
    FPrepareObjectsCalled: integer;
  protected
    procedure PrepareObjects(aRTParent: TFPReportElement); override;
  public
    constructor Create(AOwner: TComponent); override;
  end;


  TMyDataBand = class(TFPReportDataBand)
  private
    FPrepareObjectsCalled: integer;
  protected
    procedure PrepareObjects(aRTParent: TFPReportElement); override;
  public
    constructor Create(AOwner: TComponent); override;
  end;


  TMyCustomReport = class(TFPReport)
  end;


  TMyFPReportData = class(TFPReportData)
  private
    FCC: integer;
    FDFC: integer;
    FEC: integer;
    FFC: integer;
    FNC: integer;
    FOC: integer;
    FOE: boolean;
    FReportEOF: boolean;
  public
    procedure ResetCounts;
    procedure DoInitDataFields; override;
    procedure DoOpen; override;
    procedure DoFirst; override;
    procedure DoNext; override;
    procedure DoClose; override;
    function DoEOF: boolean; override;
    property InitDataFieldsCount: integer read FDFC;
    property OpenCount: integer read FOC;
    property FirstCount: integer read FFC;
    property NextCount: integer read FNC;
    property CloseCount: integer read FCC;
    property EOFCount: integer read FEC;
    property ReportEOF: boolean read FReportEOF write FReportEOF;
    property OldEOF: boolean read FOE;
    property Datafields;
  end;


  TTestFPPageSize = class(TTestCase)
  published
    procedure TestCreate;
  end;


  TTestFPPapers = class(TTestCase)
  protected
    FM: TFPReportPaperManager;
    procedure Setup; override;
    procedure TearDown; override;
    procedure RegisterPapers(ACount: integer; Local: boolean = True);
  end;


  TTestFPPaperManager = class(TTestFPPapers)
  private
    FAccess: integer;
    procedure TestAccess;
  protected
    procedure Setup; override;
  published
    procedure TestCreate;
    procedure TestRegister1;
    procedure TestRegister2;
    procedure TestRegister3;
    procedure TestRegisterDuplicate;
    procedure TestClear;
    procedure TestFind1;
    procedure TestFind2;
    procedure TestFind3;
    procedure IllegalAccess1;
    procedure IllegalAccess2;
    procedure IllegalAccess3;
    procedure IllegalAccess4;
    procedure IllegalAccess5;
    procedure IllegalAccess6;
    procedure IllegalAccess7;
    procedure IllegalAccess8;
    procedure TestWidth;
    procedure TestHeight;
  end;


  TTestFPReportPageSize = class(TTestFPPapers)
  private
    FP: TMyFPReportPageSize;
  protected
    procedure Setup; override;
    procedure TearDown; override;
  published
    procedure TestCreate;
    procedure TestCreateWithPage;
    procedure TestCreateByPage;
    procedure TestChanged1;
    procedure TestChanged2;
    procedure TestChanged3;
    procedure TestPaperName1;
    procedure TestPaperName2;
    procedure TestAssign;
  end;


  TBaseReportComponentTest = class(TTestCase)
  private
    FC: TMyFPReportComponent;
    procedure ExpectState(const aExpected: TFPReportState);
  protected
    procedure AssertEquals(Msg: string; const aExpected, AActual: TFPReportState); overload;
    procedure SetUp; override;
    procedure TearDown; override;
  end;


  TTestReportComponent = class(TBaseReportComponentTest)
  published
    procedure TestCreate;
    procedure TestLayoutState;
    procedure TestRenderState;
  end;


  TBaseReportElementTest = class(TTestCase)
  private
    FC: TMyFPReportElement;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  end;


  TReportElementTest = class(TBaseReportElementTest)
  published
    procedure TestCreate;
    procedure TestDoChange;
    procedure TestChangeCount;
    procedure TestChangeCountNested;
    procedure TestChangeCountNested2;
    procedure TestVisibleChanges;
    procedure TestLayoutChanges;
    procedure TestFrameChanges;
    procedure TestAssign;
    procedure TestEquals1;
    procedure TestEquals2;
    procedure TestEquals3;
    procedure TestEquals4;
    procedure TestEquals5;
  end;


  TTestReportChildren = class(TTestCase)
  private
    FC, FC2: TMyFPReportElementWithChildren;
    FChild: TFPReportElement;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
    procedure WrongParent;
  published
    procedure TestCreate;
    procedure TestSetParent1;
    procedure TestSetParent2;
    procedure TestSetParent3;
    procedure TestSetParent4;
    procedure TestSetParent5;
    procedure TestSetParent6;
  end;


  TTestReportFrame = class(TBaseReportElementTest)
  published
    procedure TestCreate;
    procedure TestWidthChange;
    procedure TestColorChange;
    procedure TestPenStyleChange;
    procedure TestShapeChange;
    procedure TestLinesChange;
    procedure TestAssign;
    procedure TestEquals1;
    procedure TestEquals2;
    procedure TestEquals3;
    procedure TestEquals4;
    procedure TestEquals5;
    procedure TestEquals6;
    procedure TestEquals7;
  end;


  TTestReportLayout = class(TBaseReportElementTest)
  published
    procedure TestCreate;
    procedure TestTopChange;
    procedure TestLeftChange;
    procedure TestWidthChange;
    procedure TestHeightChange;
    procedure TestAssign;
    procedure TestEquals1;
    procedure TestEquals2;
    procedure TestEquals3;
    procedure TestEquals4;
    procedure TestEquals5;
    procedure TestEquals6;
  end;


  TTestCaseWithData = class(TTestCase)
  private
    FData: TFPReportUserData;
    FSL: TStringList;
    procedure InitializeData(const ACount: integer);
    procedure SetReportData(const ADataCount: Byte);
    procedure DoGetDataValue(Sender: TObject; const AValueName: string; var AValue: variant);
    procedure DoGetDataEOF(Sender: TObject; var IsEOF: boolean);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  public
    property Data: TFPReportUserData read FData write FData;
  end;


  TTestCaseWithDataAndReport = class(TTestCaseWithData)
  private
    FReport: TMyCustomReport;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  public
    property Report: TMyCustomReport read FReport write FReport;
  end;


  TTestReportPage = class(TTestCase)
  private
    FP: TMyFPReportPage;
  protected
    procedure Setup; override;
    procedure TearDown; override;
  published
    procedure TestCreate1;
    procedure TestCreate2;
    procedure TestCreate3;
    procedure TestPageSize1;
    procedure TestPageSize2;
    procedure TestPageSize3;
    procedure TestBand1;
    procedure TestBand2;
    procedure TestData;
    procedure TestAssign;
    procedure TestFindBand;
  end;


  TTestReportData = class(TTestCase)
  private
    FD: TMyFPReportData;
    FHandler: boolean;
    procedure AssertField(Prefix: string; F: TFPReportDataField; AFieldName: string;
      AFieldKind: TFPReportFieldKind; ADisplayWidth: integer = 0);
  protected
    procedure DoOpen(Sender: TObject);
    procedure DoNext(Sender: TObject);
    procedure Setup; override;
    procedure TearDown; override;
    procedure CreateFields;
    procedure DoFieldByName;
  published
    procedure TestCreate;
    procedure TestOpen1;
    procedure TestNext;
    procedure TestInitFieldDefs;
    procedure TestInitFieldDefs_OnlyAllowedOnce;
    procedure TestEOF1;
    procedure TestAddDatafield;
    procedure TestDatafieldAdd;
    procedure TestCreateFields;
    procedure TestDatafieldIndexOf1;
    procedure TestDatafieldIndexOf2;
    procedure TestFindField1;
    procedure TestFindField2;
    procedure TestFindByName1;
    procedure TestFindByName2;
    procedure TestFieldAssign;
    procedure TestGetValue;
    procedure TestEasyAccessProperties;
  end;


  { Testing UserData by pulling data from a DataField }
  TTestUserReportData = class(TTestCase)
  private
    FD: TFPReportUserData;
    FExpectName: string;
    FReturnValue: variant;
    procedure DoValue(Sender: TObject; const AValueName: string; var AValue: variant);
  protected
    procedure Setup; override;
    procedure TearDown; override;
  published
    procedure TestGetValue;
  end;


  { Testing UserData by pulling data from a StringList }
  TTestUserReportData2 = class(TTestCase)
  private
    FData: TFPReportUserData;
    FSL: TStringList;
    procedure DoGetValue(Sender: TObject; const AValueName: string; var AValue: variant);
    procedure DoGetEOF(Sender: TObject; var IsEOF: boolean);
  protected
    procedure Setup; override;
    procedure TearDown; override;
  published
    procedure TestGetValue;
    procedure TestOnGetEOF1;
    procedure TestOnGetEOF2;
  end;


  TTestDataBand = class(TTestCaseWithDataAndReport)
  private
    FDataBand: TFPReportDataBand;
  protected
    procedure Setup; override;
    procedure TearDown; override;
  published
    procedure TestData;
    procedure TestDataPropertyAutoSet;
  end;


  TTestCustomReport = class(TTestCase)
  private
    FRpt: TMyCustomReport;
    FBeginReportCount: integer;
    FEndReportCount: integer;
    FSL: TStringList;
    FData: TFPReportUserData;
    procedure HandleOnBeginReport;
    procedure HandleOnEndReport;
    procedure InitializeData(const ACount: integer);
    procedure SetReportData(const ADataCount: Byte);
    procedure DoGetDataValue(Sender: TObject; const AValueName: string; var AValue: variant);
    procedure DoGetDataEOF(Sender: TObject; var IsEOF: boolean);
    procedure DoGetDataFieldNames(Sender: TObject; List: TStrings);
  protected
    procedure Setup; override;
    procedure TearDown; override;
  public
    property Data: TFPReportUserData read FData;
    property Report: TMyCustomReport read FRpt write FRpt;
  published
    procedure TestBeginReportEvent;
    procedure TestEndReportEvent;
    procedure TestPagePrepareObjects;
    procedure TestBandPrepareObjects;
    procedure TestRTObjects1;
    procedure TestRTObjects2;
    procedure TestRTObjects3;
    procedure TestRTObjects4_OneDataItem;
    procedure TestRTObjects5_TwoDataItems;
    procedure TestInternalFunction_Page;
    procedure TestInternalFunction_Page_with_text;
    procedure TestInternalFunction_RecNo;
    procedure TestInternalFunction_Today;
    procedure TestInternalFunction_Today_with_text;
    procedure TestInternalFunction_Author;
    procedure TestInternalFunction_Author_with_text;
    procedure TestInternalFunction_Title;
    procedure TestInternalFunction_Title_with_text;
  end;


  { TTestReportMemo }

  TTestReportMemo = class(TTestCase)
  private
    FMemo: TFPReportMemo;
    procedure CauseFontNotFoundException;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  published
    procedure TestCreate;
    procedure TestPrepareTextBlocks;
    procedure TestPrepareTextBlocks_multiline_data;
    procedure TestPrepareTextBlocks_multiline_wraptext;
    procedure TestPrepareTextBlocks_multiline_wraptext_oneword;
    procedure TestPrepareTextBlocks_multiline_wraptext_oneword_overflow;
    procedure TestPrepareTextBlocks_multiline_wraptext_oneword_split;
    procedure TestRGBToReportColor;
    procedure TestHTMLColorToReportColor_length7;
    procedure TestHTMLColorToReportColor_length6;
    procedure TestHTMLColorToReportColor_length3;
    procedure TestCreateTestBlock;
    procedure TestCreateTestBlock_IsURL;
    procedure TestSubStr;
    procedure TestTokenCount;
    procedure TestToken;
  end;


  TTestBandList = class(TTestCase)
  private
    FList: TBandList;
    b1: TFPReportPageHeaderBand;
    b2: TFPReportTitleBand;
    b3: TFPReportDataBand;
    procedure CreateBands;
    procedure AddAllBandsToList;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  published
    procedure TestAdd;
    procedure TestItems;
    procedure TestClear;
    procedure TestDelete;
    procedure TestFind1;
    procedure TestFind2;
  end;

  { TTestVariableBase }

  TTestVariableBase = Class(TTestCase)
  Public
    Class procedure AssertEquals(Const Msg : String; AExpected,AActual : TResultType); overload;
  end;

  { TTestVariable }

  TTestVariable = Class(TTestVariableBase)
  private
    FVar: TFPReportVariable;
  Protected
    Procedure SetUp; override;
    Procedure TearDown; override;
    Property Variable : TFPReportVariable Read FVar;
  Published
    Procedure TestEmpty;
    Procedure TestName;
    Procedure TestBoolean;
    Procedure TestInteger;
    Procedure TestDateTime;
    Procedure TestFloat;
    Procedure TestString;
    Procedure TestExpressionResult;
  end;

  { TTestVariables }

  TTestVariables  = Class(TTestVariableBase)
  private
    FVar: TFPReportVariables;
    FV : Array[0..2] of TFPReportVariable;
    procedure AddThree;
  Protected
    Procedure SetUp; override;
    Procedure TearDown; override;
    Property Variables : TFPReportVariables Read FVar;
  Published
    Procedure TestEmpty;
    Procedure TestAdd;
    Procedure TestIndexOf;
    Procedure TestFind;
  end;

implementation

uses
  TypInfo,
  DateUtils,
  fpTTF;

type
  TMemoFriend = class(TFPReportMemo);

{ TTestVariables }

procedure TTestVariables.SetUp;
begin
  inherited SetUp;
  FVar:=TFPReportVariables.Create(Nil,TFPReportVariable);
end;

procedure TTestVariables.TearDown;
begin
  FreeAndNil(FVar);
  inherited TearDown;
end;

procedure TTestVariables.TestEmpty;
begin
  AssertNotNull('Have variables',Variables);
  AssertEquals('Variable count',0,Variables.Count);
  AssertTrue('Variable class',Variables.ItemClass.InheritsFrom(TFPReportVariable));
end;

procedure TTestVariables.TestAdd;

Var
  V : TFPReportVariable;

begin
  V:=Variables.addVariable('aName');
  AssertNotNull('Have result',V);
  AssertEquals('Correct name','aName',V.Name);
  AssertEquals('Correct type',rtString,V.DataType);
  AssertEquals('Correct value','',V.AsString);
  AssertEquals('Added to collection',1,Variables.Count);
  AssertSame('In array',V,Variables[0]);
  ExpectException('Cannot add twice',EReportError);
  V:=Variables.addVariable('aName');
end;

procedure TTestVariables.AddThree;

Var
  I: integer;

begin
  For I:=0 to 2 do
    FV[I]:=Variables.Addvariable('aName'+IntToStr(i+1));
end;

procedure TTestVariables.TestIndexOf;
begin
  AddThree;
  AssertEquals('First',0,Variables.IndexOfVariable('aName1'));
  AssertEquals('Second',1,Variables.IndexOfVariable('aName2'));
  AssertEquals('Third',2,Variables.IndexOfVariable('aName3'));
  AssertEquals('NonExisting',-1,Variables.IndexOfVariable('aName4'));
end;

procedure TTestVariables.TestFind;
begin
  AddThree;
  AssertSame('First',FV[0],Variables.FindVariable('aName1'));
  AssertSame('Second',FV[1],Variables.FindVariable('aName2'));
  AssertSame('Third',FV[2],Variables.FindVariable('aName3'));
  AssertNull('NonExisting',Variables.FindVariable('aName4'));
end;

{ TTestVariableBase }

class procedure TTestVariableBase.AssertEquals(const Msg: String; AExpected,
  AActual: TResultType);
begin
  AssertEquals(Msg,GetEnumName(TypeInfo(TResultType),Ord(AExpected)),GetEnumName(TypeInfo(TResultType),Ord(AActual)))
end;

{ TTestVariable }

procedure TTestVariable.SetUp;
begin
  inherited SetUp;
  FVar:=TFPReportVariable.Create(Nil);
end;

procedure TTestVariable.TearDown;
begin
  FreeandNil(FVar);
  inherited TearDown;
end;

procedure TTestVariable.TestEmpty;
begin
  AssertNotNull('Have variable', Variable);
  AssertEquals('Boolean type',rtBoolean,Variable.DataType);
  AssertFalse('Boolean default value',Variable.AsBoolean);
end;

procedure TTestVariable.TestName;
begin
  Variable.Name:='me'; // OK
  Variable.Name:='me.me'; // OK
  ExpectException('Name must be identifier',EReportError);
  Variable.Name:='me me'; // not OK
end;

procedure TTestVariable.TestBoolean;

Var
  R : TFPExpressionResult;

begin
  Variable.DataType:=rtBoolean;
  AssertEquals('Boolean type remains',rtBoolean,Variable.DataType);
  AssertFalse('Boolean default value',Variable.AsBoolean);
  AssertEquals('Boolean as string','False',Variable.Value);
  Variable.DataType:=rtFloat;
  Variable.AsBoolean:=true;
  AssertEquals('Boolean type remains',rtBoolean,Variable.DataType);
  AssertEquals('Boolean as string','True',Variable.Value);
  AssertTrue('Boolean value',Variable.AsBoolean);
  R:=Variable.AsExpressionResult;
  AssertEquals('Correct result',rtBoolean,r.resulttype);
  AssertEquals('Correct value',True,r.resBoolean);
  ExpectException('Cannot fetch as other type',EConvertError);
  Variable.AsString;
end;

procedure TTestVariable.TestInteger;

Var
  R : TFPExpressionResult;

begin
  Variable.DataType:=rtInteger;
  AssertEquals('Integer type remains',rtInteger,Variable.DataType);
  AssertEquals('Integer default value',0,Variable.AsInteger);
  AssertEquals('Integer as string','0',Variable.Value);
  Variable.DataType:=rtFloat;
  Variable.AsInteger:=123;
  AssertEquals('Integer type remains',rtInteger,Variable.DataType);
  AssertEquals('Integer as string','123',Variable.Value);
  AssertEquals('Integer value',123,Variable.AsInteger);
  R:=Variable.AsExpressionResult;
  AssertEquals('Correct result',rtInteger,r.resulttype);
  AssertEquals('Correct value',123,r.resInteger);
  ExpectException('Cannot fetch as other type',EConvertError);
  Variable.AsString;
end;

procedure TTestVariable.TestDateTime;

Var
  R : TFPExpressionResult;

begin
  Variable.DataType:=rtDateTime;
  AssertEquals('DateTime type remains',rtDateTime,Variable.DataType);
  AssertEquals('DateTime default value',0.0,Variable.AsDateTime);
  AssertEquals('DateTime as string','00000000T000000',Variable.Value);
  Variable.DataType:=rtDateTime;
  Variable.AsDateTime:=Date;
  AssertEquals('DateTime type remains',rtDateTime,Variable.DataType);
  AssertEquals('DateTime as string',FormatDateTime('yyyymmdd"T"000000',Date),Variable.Value);
  AssertEquals('DateTime value',Date,Variable.AsDateTime);
  R:=Variable.AsExpressionResult;
  AssertEquals('Correct result',rtDateTime,r.resulttype);
  AssertEquals('Correct value',Date,r.resDateTime);
  ExpectException('Cannot fetch as other type',EConvertError);
  Variable.AsString;
end;

procedure TTestVariable.TestFloat;

Var
  R : TFPExpressionResult;

begin
  Variable.DataType:=rtFloat;
  AssertEquals('Float type remains',rtFloat,Variable.DataType);
  AssertEquals('Float default value',0.0,Variable.AsFloat);
  AssertEquals('Float as string',' 0.0000000000000000E+000',Variable.Value);
  Variable.DataType:=rtBoolean;
  Variable.AsFloat:=1.23;
  AssertEquals('Float type remains',rtFloat,Variable.DataType);
  AssertEquals('Float as string',' 1.2300000000000000E+000',Variable.Value);
  AssertEquals('Float value',1.23,Variable.AsFloat);
  R:=Variable.AsExpressionResult;
  AssertEquals('Correct result',rtFloat,r.resulttype);
  AssertEquals('Correct value',1.23,r.resFloat);
  ExpectException('Cannot fetch as other type',EConvertError);
  Variable.AsString;
end;

procedure TTestVariable.TestString;

Var
  R : TFPExpressionResult;

begin
  Variable.DataType:=rtString;
  AssertEquals('String type remains',rtString,Variable.DataType);
  AssertEquals('String default value','',Variable.AsString);
  AssertEquals('String as string','',Variable.Value);
  Variable.DataType:=rtBoolean;
  Variable.AsString:='abc';
  AssertEquals('String type remains',rtString,Variable.DataType);
  AssertEquals('String as string','abc',Variable.Value);
  AssertEquals('String value','abc',Variable.AsString);
  R:=Variable.AsExpressionResult;
  AssertEquals('Correct result',rtString,r.resulttype);
  AssertEquals('Correct value','abc',r.resString);
  ExpectException('Cannot fetch as other type',EConvertError);
  Variable.AsFloat;
end;

procedure TTestVariable.TestExpressionResult;

Var
  R : TFPExpressionResult;

begin
  R.ResultType:=rtFloat;
  R.ResFloat:=1.23;
  Variable.AsExpressionResult:=R;
  AssertEquals('Correct type ',rtFloat,Variable.DataType);
  AssertEquals('Correct value',1.23,Variable.AsFloat);
  R.ResultType:=rtBoolean;
  R.ResBoolean:=True;
  Variable.AsExpressionResult:=R;
  AssertEquals('Correct type ',rtBoolean,Variable.DataType);
  AssertEquals('Correct value',True,Variable.AsBoolean);
  R.ResultType:=rtString;
  R.ResString:='me';
  Variable.AsExpressionResult:=R;
  AssertEquals('Correct type ',rtString,Variable.DataType);
  AssertEquals('Correct value','me',Variable.AsString);
  R.ResultType:=rtDateTime;
  R.ResDateTime:=Date;
  Variable.AsExpressionResult:=R;
  AssertEquals('Correct type ',rtDateTime,Variable.DataType);
  AssertEquals('Correct value',Date,Variable.AsDateTime);
  R.ResultType:=rtinteger;
  R.ResInteger:=1234;
  Variable.AsExpressionResult:=R;
  AssertEquals('Correct type ',rtinteger,Variable.DataType);
  AssertEquals('Correct value',1234,Variable.AsInteger);
end;

{ TTestCaseWithData }

procedure TTestCaseWithData.InitializeData(const ACount: Integer);
var
  i: integer;
begin
  // data is coming from the stringlist this time
  FSL := TStringList.Create;
  if ACount < 1 then
    Exit;
  for i := 1 to ACount do
    FSL.Add('Item ' + IntToStr(i));
end;

procedure TTestCaseWithData.SetReportData(const ADataCount: Byte);
begin
  if ADataCount < 1 then
    Exit;
  InitializeData(ADataCount);
  FData := TFPReportUserData.Create(nil);
  FData.OnGetValue := @DoGetDataValue;
  FData.OnGetEOF := @DoGetDataEOF;
end;

procedure TTestCaseWithData.DoGetDataValue(Sender: TObject; const AValueName: string; var AValue: variant);
begin
  if AValueName = 'element' then
    AValue := FSL[FData.RecNo - 1];
end;

procedure TTestCaseWithData.DoGetDataEOF(Sender: TObject; var IsEOF: boolean);
begin
  if FData.RecNo > FSL.Count then
    IsEOF := True
  else
    IsEOF := False;
end;

procedure TTestCaseWithData.SetUp;
begin
  inherited SetUp;
end;

procedure TTestCaseWithData.TearDown;
begin
  FreeAndNil(FData);
  FreeAndNil(FSL);
  inherited TearDown;
end;

{ TTestCaseWithDataAndReport }

procedure TTestCaseWithDataAndReport.SetUp;
begin
  inherited SetUp;
  FReport := TMyCustomReport.Create(nil);
end;

procedure TTestCaseWithDataAndReport.TearDown;
begin
  inherited TearDown;
  FreeAndNil(FReport);
end;

{ TBaseReportComponentTest }

procedure TBaseReportComponentTest.ExpectState(const aExpected: TFPReportState);
begin
  AssertEquals('ReportComponent.ReportState: ', AExpected, FC.ReportState);
end;

procedure TBaseReportComponentTest.AssertEquals(Msg: string; const aExpected, AActual: TFPReportState);
begin
  AssertEquals(Msg, GetEnumName(TypeInfo(TFPReportState), Ord(AExpected)),
    GetEnumName(TypeInfo(TFPReportState), Ord(AActual)));
end;

procedure TBaseReportComponentTest.SetUp;
begin
  FC := TMyFPReportComponent.Create(nil);
end;

procedure TBaseReportComponentTest.TearDown;
begin
  FreeAndNil(FC);
end;

{ TTestReportComponent }

procedure TTestReportComponent.TestCreate;
begin
  ExpectState(rsDesign);
end;

procedure TTestReportComponent.TestLayoutState;
begin
  FC.StartLayout;
  ExpectState(rsLayout);
  FC.EndLayout;
  ExpectState(rsDesign);
end;

procedure TTestReportComponent.TestRenderState;
begin
  FC.StartRender;
  ExpectState(rsRender);
  FC.EndRender;
  ExpectState(rsDesign);
end;

{ TMyFPReportComponent }

procedure TMyFPReportComponent.StartLayout;
begin
  inherited StartLayout;
end;

procedure TMyFPReportComponent.EndLayout;
begin
  inherited EndLayout;
end;

procedure TMyFPReportComponent.StartRender;
begin
  inherited StartRender;
end;

procedure TMyFPReportComponent.EndRender;
begin
  inherited EndRender;
end;

{ TMyFPReportElement }

procedure TMyFPReportElement.CallChange;
begin
  Changed;
end;

procedure TMyFPReportElement.ResetChanged;
begin
  FChangedCalled := 0;
end;

procedure TMyFPReportElement.DoChanged;
begin
  inherited DoChanged;
  Inc(FChangedCalled);
end;

{ TBaseReportElementTest }

procedure TBaseReportElementTest.SetUp;
begin
  inherited SetUp;
  FC := TMyFPReportElement.Create(nil);
end;

procedure TBaseReportElementTest.TearDown;
begin
  FreeAndNil(FC);
  inherited TearDown;
end;

{ TReportElementTest }

procedure TReportElementTest.TestCreate;
begin
  AssertEquals('Create does not invoke changed', 0, FC.ChangedCalled);
  AssertNotNull('Create creates frame', FC.Frame);
  AssertEquals('Create creates frame of correct class', TFPReportFrame, FC.Frame.Classtype);
  AssertNotNull('Create creates layout', FC.Layout);
  AssertEquals('Create creates layout of correct class ', TFPReportLayout, FC.Layout.Classtype);
  AssertEquals('Created element is visible', True, FC.Visible);
  AssertNull('No parent at create', FC.Parent);
end;

procedure TReportElementTest.TestDoChange;
begin
  FC.CallChange;
  AssertEquals('Change calls dochange', 1, FC.ChangedCalled);
end;

procedure TReportElementTest.TestChangeCount;
begin
  FC.BeginUpdate;
  try
    FC.CallChange;
    AssertEquals('First Change does notcall dochange', 0, FC.ChangedCalled);
    FC.CallChange;
    AssertEquals('Second Change does not call dochange', 0, FC.ChangedCalled);
  finally
    FC.EndUpdate;
  end;
  AssertEquals('EndUpdate calls dochange once', 1, FC.ChangedCalled);
end;

procedure TReportElementTest.TestChangeCountNested;
begin
  FC.BeginUpdate;
  try
    FC.CallChange;
    AssertEquals('First Change does notcall dochange', 0, FC.ChangedCalled);
    FC.BeginUpdate;
    try
      FC.CallChange;
      AssertEquals('Second Change does not call dochange', 0, FC.ChangedCalled);
    finally
      FC.EndUpdate;
      AssertEquals('First endupdate does not call dochange', 0, FC.ChangedCalled);
    end;
  finally
    FC.EndUpdate;
  end;
  AssertEquals('Second EndUpdate calls dochange once', 1, FC.ChangedCalled);
end;

procedure TReportElementTest.TestChangeCountNested2;
begin
  FC.BeginUpdate;
  try
    FC.CallChange;
    AssertEquals('First Change does notcall dochange', 0, FC.ChangedCalled);
    FC.BeginUpdate;
    try
      FC.CallChange;
      AssertEquals('Second Change does not call dochange', 0, FC.ChangedCalled);
      FC.CallChange;
      AssertEquals('Third Change does not call dochange', 0, FC.ChangedCalled);
    finally
      FC.EndUpdate;
      AssertEquals('First endupdate does not call dochange', 0, FC.ChangedCalled);
    end;
  finally
    FC.EndUpdate;
  end;
  AssertEquals('Second EndUpdate calls dochange once', 1, FC.ChangedCalled);
end;

procedure TReportElementTest.TestVisibleChanges;
begin
  FC.ResetChanged;
  FC.Visible := False;
  AssertEquals('Setting visible calls change', 1, FC.ChangedCalled);
end;

procedure TReportElementTest.TestLayoutChanges;
var
  L: TFPreportLayout;
begin
  L := TFPreportLayout.Create(nil);
  try
    FC.Layout := L;
    AssertEquals('Setting layout calls change', 1, FC.ChangedCalled);
  finally
    L.Free;
  end;
end;

procedure TReportElementTest.TestFrameChanges;
var
  F: TFPreportFrame;
begin
  F := TFPreportFrame.Create(nil);
  try
    FC.Frame := F;
    AssertEquals('Setting frame calls change', 1, FC.ChangedCalled);
  finally
    F.Free;
  end;
end;

procedure TReportElementTest.TestAssign;
var
  E: TFPReportElement;
begin
  E := TMyFPReportElement.Create(nil);
  try
    FC.Layout.Top := 1;
    FC.Frame.Width := 2;
    E.Assign(FC);
    AssertEquals('Assigned frame equal', True, FC.Frame.Equals(E.Frame));
    AssertEquals('Assigned layout equal', True, FC.Layout.Equals(FC.Layout));
  finally
    E.Free;
  end;
end;

procedure TReportElementTest.TestEquals1;
begin
  AssertEquals('Self always returns equal', True, FC.Equals(FC));
end;

procedure TReportElementTest.TestEquals2;
var
  E: TFPReportElement;
begin
  E := TMyFPReportElement.Create(nil);
  try
    E.Assign(FC);
    AssertEquals('Assigned element returns equal', True, FC.Equals(E));
    AssertEquals('Assigned element returns equal', True, E.Equals(FC));
  finally
    E.Free;
  end;
end;

procedure TReportElementTest.TestEquals3;
var
  E: TFPReportElement;
begin
  E := TFPReportElement.Create(nil);
  try
    E.Assign(FC);
    AssertEquals('Different class makes unequal', True, FC.Equals(E));
    AssertEquals('Different class makes unequal', True, E.Equals(FC));
  finally
    E.Free;
  end;
end;

procedure TReportElementTest.TestEquals4;
var
  E: TFPReportElement;
begin
  E := TMyFPReportElement.Create(nil);
  try
    FC.Layout.Top := 1;
    E.Assign(FC);
    E.Layout.Top := 2;
    AssertEquals('Changed layout makes unequal', False, FC.Equals(E));
    AssertEquals('Changed layout makes unequal', False, E.Equals(FC));
  finally
    E.Free;
  end;
end;

procedure TReportElementTest.TestEquals5;
var
  E: TFPReportElement;
begin
  E := TMyFPReportElement.Create(nil);
  try
    FC.Layout.Top := 1;
    E.Assign(FC);
    E.Frame.Lines := [flLeft];
    AssertEquals('Changed frame makes unequal', False, FC.Equals(E));
    AssertEquals('Changed frame makes unequal', False, E.Equals(FC));
  finally
    E.Free;
  end;
end;

{ TTestReportFrame }

procedure TTestReportFrame.TestCreate;
begin
  AssertEquals('Failed on 1', 1, FC.Frame.Width);
  AssertEquals('Failed on 2',
    GetEnumName(TYpeInfo(TFPPenStyle), Ord(psSolid)),
    GetEnumName(TYpeInfo(TFPPenStyle), Ord(FC.Frame.Pen)));
  if not (FC.Frame.Lines = []) then
    Fail('Failed on 3');
  AssertEquals('Failed on 4',
    GetEnumName(TypeInfo(TFPReportFrameShape), Ord(fsNone)),
    GetEnumName(TypeInfo(TFPReportFrameShape), Ord(FC.Frame.Shape)));
end;

procedure TTestReportFrame.TestWidthChange;
begin
  FC.Frame.Width := 2;
  AssertEquals('Setting Width calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportFrame.TestColorChange;
begin
  FC.Frame.Color := 3;
  AssertEquals('Setting Solor calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportFrame.TestPenStyleChange;
begin
  FC.Frame.Pen := psDot;
  AssertEquals('Setting pen calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportFrame.TestShapeChange;
begin
  FC.Frame.Shape := fsRoundedRect;
  AssertEquals('Setting pen calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportFrame.TestLinesChange;
begin
  FC.Frame.Lines := [flBottom];
  AssertEquals('Setting pen calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportFrame.TestAssign;
var
  F: TFPReportFrame;
begin
  F := TFPReportFrame.Create(nil);
  try
    F.Width := 3;
    F.Lines := [flBottom, flTop];
    F.Color := 4;
    F.Pen := psDot;
    F.Shape := fsRoundedRect;
    FC.Frame.Assign(F);
    AssertSame('ReportElement not copied', FC, FC.Frame.ReportElement);
    AssertEquals('Assert calls changed', 1, FC.ChangedCalled);
    AssertEquals('Frame width equals 3', F.Width, FC.Frame.Width);
    AssertEquals('Frame penstyle equals psDot',
      GetEnumName(TYpeInfo(TFPPenStyle), Ord(F.Pen)),
      GetEnumName(TYpeInfo(TFPPenStyle), Ord(FC.Frame.Pen)));
    if not (FC.Frame.Lines = F.Lines) then
      Fail('Frame lines not copied correctly');
    AssertEquals('Frame shape correctly copied',
      GetEnumName(TypeInfo(TFPReportFrameShape), Ord(F.Shape)),
      GetEnumName(TypeInfo(TFPReportFrameShape), Ord(FC.Frame.Shape)));
  finally
    F.Free;
  end;
end;

procedure TTestReportFrame.TestEquals1;
var
  F: TFPReportFrame;
begin
  F := TFPReportFrame.Create(nil);
  try
    FC.Frame.Width := 3;
    FC.Frame.Lines := [flBottom, flTop];
    FC.Frame.Color := 4;
    FC.Frame.Pen := psDot;
    FC.Frame.Shape := fsRoundedRect;
    F.Assign(FC.Frame);
    F.Width := 2;
    AssertEquals('Width changed makes unequal', False, FC.Frame.Equals(F));
    AssertEquals('Width changed makes unequal', False, F.Equals(FC.Frame));
  finally
    F.Free;
  end;
end;

procedure TTestReportFrame.TestEquals2;
var
  F: TFPReportFrame;
begin
  F := TFPReportFrame.Create(nil);
  try
    FC.Frame.Width := 3;
    FC.Frame.Lines := [flBottom, flTop];
    FC.Frame.Color := 4;
    FC.Frame.Pen := psDot;
    FC.Frame.Shape := fsRoundedRect;
    F.Assign(FC.Frame);
    F.Color := 2;
    AssertEquals('Color changed makes unequal', False, FC.Frame.Equals(F));
    AssertEquals('Color changed makes unequal', False, F.Equals(FC.Frame));
  finally
    F.Free;
  end;
end;

procedure TTestReportFrame.TestEquals3;
var
  F: TFPReportFrame;
begin
  F := TFPReportFrame.Create(nil);
  try
    FC.Frame.Width := 3;
    FC.Frame.Lines := [flBottom, flTop];
    FC.Frame.Color := 4;
    FC.Frame.Pen := psDot;
    FC.Frame.Shape := fsRoundedRect;
    F.Assign(FC.Frame);
    F.Pen := psDash;
    AssertEquals('Pen changed makes unequal', False, FC.Frame.Equals(F));
    AssertEquals('Pen changed makes unequal', False, F.Equals(FC.Frame));
  finally
    F.Free;
  end;
end;

procedure TTestReportFrame.TestEquals4;
var
  F: TFPReportFrame;
begin
  F := TFPReportFrame.Create(nil);
  try
    FC.Frame.Width := 3;
    FC.Frame.Lines := [flBottom, flTop];
    FC.Frame.Color := 4;
    FC.Frame.Pen := psDot;
    FC.Frame.Shape := fsRoundedRect;
    F.Assign(FC.Frame);
    F.Shape := fsShadow;
    AssertEquals('Shape changed makes unequal', False, FC.Frame.Equals(F));
    AssertEquals('Shape changed makes unequal', False, F.Equals(FC.Frame));
  finally
    F.Free;
  end;
end;

procedure TTestReportFrame.TestEquals5;
var
  F: TFPReportFrame;
begin
  F := TFPReportFrame.Create(nil);
  try
    FC.Frame.Width := 3;
    FC.Frame.Lines := [flBottom, flTop];
    FC.Frame.Color := 4;
    FC.Frame.Pen := psDot;
    FC.Frame.Shape := fsRoundedRect;
    F.Assign(FC.Frame);
    F.Lines := [flLeft, flRight];
    AssertEquals('Lines changed makes unequal', False, FC.Frame.Equals(F));
    AssertEquals('Lines changed makes unequal', False, F.Equals(FC.Frame));
  finally
    F.Free;
  end;
end;

procedure TTestReportFrame.TestEquals6;
begin
  AssertEquals('Same frame always equals', True, FC.Frame.Equals(FC.Frame));
end;

procedure TTestReportFrame.TestEquals7;
var
  F: TFPReportFrame;
begin
  F := TFPReportFrame.Create(nil);
  try
    FC.Frame.Width := 3;
    FC.Frame.Lines := [flBottom, flTop];
    FC.Frame.Color := 4;
    FC.Frame.Pen := psDot;
    FC.Frame.Shape := fsRoundedRect;
    F.Assign(FC.Frame);
    AssertEquals('Equals after assign', True, FC.Frame.Equals(F));
    AssertEquals('Equals after assign', True, F.Equals(FC.Frame));
  finally
    F.Free;
  end;
end;


{ TTestReportLayout }

procedure TTestReportLayout.TestCreate;
begin
  AssertEquals('Top is zero', 0, FC.Layout.top);
  AssertEquals('Left is zero', 0, FC.Layout.Left);
  AssertEquals('Width is zero', 0, FC.Layout.Width);
  AssertEquals('Height is zero', 0, FC.Layout.Width);
end;

procedure TTestReportLayout.TestTopChange;
begin
  FC.Layout.Top := 2;
  AssertEquals('Setting top calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportLayout.TestLeftChange;
begin
  FC.Layout.Left := 2;
  AssertEquals('Setting left calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportLayout.TestWidthChange;
begin
  FC.Layout.Width := 2;
  AssertEquals('Setting width calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportLayout.TestHeightChange;
begin
  FC.Layout.Height := 2;
  AssertEquals('Setting Height calls onChange', 1, FC.ChangedCalled);
end;

procedure TTestReportLayout.TestAssign;
var
  L: TFPReportLayout;
begin
  L := TFPReportlayout.Create(nil);
  try
    FC.Layout.Top := 1;
    FC.Layout.Left := 1;
    FC.Layout.Width := 10;
    FC.Layout.Height := 10;
    L.Assign(FC.Layout);
    AssertEquals('Top correct', FC.Layout.Top, L.Top);
    AssertEquals('Left correct', FC.Layout.Left, L.Left);
    AssertEquals('Width correct', FC.Layout.Width, L.Width);
    AssertEquals('Height correct', FC.Layout.Height, L.Height);
  finally
    L.Free;
  end;
end;

procedure TTestReportLayout.TestEquals1;
var
  L: TFPReportLayout;
begin
  L := TFPReportlayout.Create(nil);
  try
    FC.Layout.Top := 1;
    FC.Layout.Left := 1;
    FC.Layout.Width := 10;
    FC.Layout.Height := 10;
    L.Assign(FC.Layout);
    FC.Layout.Top := 2;
    AssertEquals('Top changed makes unequal', False, FC.Layout.Equals(L));
    AssertEquals('Top changed makes unequal', False, L.Equals(FC.Layout));
  finally
    L.Free;
  end;
end;

procedure TTestReportLayout.TestEquals2;
var
  L: TFPReportLayout;
begin
  L := TFPReportlayout.Create(nil);
  try
    FC.Layout.Top := 1;
    FC.Layout.Left := 1;
    FC.Layout.Width := 10;
    FC.Layout.Height := 10;
    L.Assign(FC.Layout);
    FC.Layout.Left := 2;
    AssertEquals('Left changed makes unequal', False, FC.Layout.Equals(L));
    AssertEquals('Left changed makes unequal', False, L.Equals(FC.Layout));
  finally
    L.Free;
  end;
end;

procedure TTestReportLayout.TestEquals3;
var
  L: TFPReportLayout;
begin
  L := TFPReportlayout.Create(nil);
  try
    FC.Layout.Top := 1;
    FC.Layout.Left := 1;
    FC.Layout.Width := 10;
    FC.Layout.Height := 10;
    L.Assign(FC.Layout);
    FC.Layout.Width := 2;
    AssertEquals('Width changed makes unequal', False, FC.Layout.Equals(L));
    AssertEquals('Width changed makes unequal', False, L.Equals(FC.Layout));
  finally
    L.Free;
  end;
end;

procedure TTestReportLayout.TestEquals4;
var
  L: TFPReportLayout;
begin
  L := TFPReportlayout.Create(nil);
  try
    FC.Layout.Top := 1;
    FC.Layout.Left := 1;
    FC.Layout.Width := 10;
    FC.Layout.Height := 10;
    L.Assign(FC.Layout);
    FC.Layout.Height := 2;
    AssertEquals('Height changed makes unequal', False, FC.Layout.Equals(L));
    AssertEquals('Height changed makes unequal', False, L.Equals(FC.Layout));
  finally
    L.Free;
  end;
end;

procedure TTestReportLayout.TestEquals5;
var
  L: TFPReportLayout;
begin
  L := TFPReportlayout.Create(nil);
  try
    FC.Layout.Top := 1;
    FC.Layout.Left := 1;
    FC.Layout.Width := 10;
    FC.Layout.Height := 10;
    L.Assign(FC.Layout);
    AssertEquals('Assign results in equal', True, FC.Layout.Equals(L));
    AssertEquals('Assign results in equal', True, L.Equals(FC.Layout));
  finally
    L.Free;
  end;
end;

procedure TTestReportLayout.TestEquals6;
begin
  AssertEquals('Assign results in equal', True, FC.Layout.Equals(FC.Layout));
end;

{ TTestReportChildren }

procedure TTestReportChildren.SetUp;
begin
  FC := TMyFPReportElementWithChildren.Create(nil);
  FC2 := TMyFPReportElementWithChildren.Create(nil);
  FChild := TFPReportElement.Create(nil);
end;

procedure TTestReportChildren.TearDown;
begin
  FreeAndNil(FChild);
  FreeAndNil(FC);
  FreeAndNil(FC2);
end;

procedure TTestReportChildren.WrongParent;
begin
  FC.Parent := FChild;
end;

procedure TTestReportChildren.TestCreate;
begin
  AssertEquals('No children at create', 0, FC.ChildCount);
end;

procedure TTestReportChildren.TestSetParent1;
begin
  AssertException('Cannot set TReportElement as parent', EReportError, @WrongParent);
end;

procedure TTestReportChildren.TestSetParent2;
begin
  FChild.parent := FC;
  AssertSame('Parent was saved', FC, FChild.parent);
  AssertEquals('Changed was called', 1, FC.ChangedCalled);
  AssertEquals('Parent childcount is 1', 1, FC.ChildCount);
  AssertSame('Parent first child is OK', FChild, FC.Child[0]);
end;

procedure TTestReportChildren.TestSetParent3;
var
  E: TFPReportElementWithChildren;
begin
  FChild.parent := FC;
  AssertSame('Parent was saved', FC, FChild.parent);
  AssertEquals('Parent childcount is 1', 1, FC.ChildCount);
  AssertSame('Parent first child is OK', FChild, FC.Child[0]);
  FC.ResetChanged;
  FChild.Parent := FC2;
  AssertSame('Parent was saved', FC2, FChild.parent);
  AssertEquals('Changed was called', 1, FC.ChangedCalled);
  AssertEquals('Old Parent childcount is 0', 0, FC.ChildCount);
  AssertEquals('Parent childcount is 1', 1, FC2.ChildCount);
  AssertSame('Parent first child is OK', FChild, FC2.Child[0]);
end;

procedure TTestReportChildren.TestSetParent4;
begin
  FChild.parent := FC;
  AssertSame('Parent was saved', FC, FChild.parent);
  AssertEquals('Parent childcount is 1', 1, FC.ChildCount);
  AssertSame('Parent first child is OK', FChild, FC.Child[0]);
  FreeAndNil(FC);
  //FChild is freed due to free of parent
  //AssertNull('Child parent was removed when parent is freed', FChild.Parent);
  FChild := Nil;
end;

procedure TTestReportChildren.TestSetParent6;
begin
  FChild.parent := FC;
  AssertSame('Parent was saved', FC, FChild.parent);
  AssertEquals('Parent childcount is 1', 1, FC.ChildCount);
  AssertSame('Parent first child is OK', FChild, FC.Child[0]);
  FChild.parent := nil;
  AssertNull('Child parent was removed when parent is freed', FChild.Parent);
end;

procedure TTestReportChildren.TestSetParent5;
begin
  FChild.parent := FC;
  AssertSame('Parent was saved', FC, FChild.parent);
  AssertEquals('Parent childcount is 1', 1, FC.ChildCount);
  AssertSame('Parent first child is OK', FChild, FC.Child[0]);
  FreeAndNil(FChild);
  AssertEquals('Child removed when freed', 0, FC.ChildCount);
end;

{ TMyFPReportElementWithChildren }

procedure TMyFPReportElementWithChildren.CallChange;
begin
  Changed;
end;

procedure TMyFPReportElementWithChildren.ResetChanged;
begin
  FChangedCalled := 0;
end;

procedure TMyFPReportElementWithChildren.DoChanged;
begin
  inherited DoChanged;
  Inc(FChangedCalled);
end;

{ TTestFPPageSize }

procedure TTestFPPageSize.TestCreate;
var
  F: TFPReportPaperSize;
begin
  F := TFPReportPaperSize.Create(1.23, 3.45);
  try
    AssertEquals('Width stored correctly', 1.23, F.Width, 0.001);
    AssertEquals('Height stored correctly', 3.45, F.Height, 0.001);
  finally
    F.Free;
  end;
end;

{ TTestFPPaperManager }

procedure TTestFPPapers.Setup;
begin
  FM := TFPReportPaperManager.Create(nil);
  AssertNotNull(FM);
end;

procedure TTestFPPapers.TearDown;
begin
  FreeAndNil(FM);
end;

procedure TTestFPPapers.RegisterPapers(ACount: integer; Local: boolean = True);
var
  F: TFPReportPaperManager;
begin
  if local then
    F := FM
  else
    F := PaperManager;
  if (ACount >= 1) then
    F.RegisterPaper('P3', 1.0, 2.0);
  if (ACount >= 2) then
    F.RegisterPaper('P2', 4.0, 8.0);
  if (ACount >= 3) then
    F.RegisterPaper('P1', 16.0, 32.0);
end;

procedure TTestFPPaperManager.TestAccess;
begin
  case FAccess of
    0: FM.PaperNames[-1];
    1: FM.PaperNames[FM.PaperCount];
    2: FM.PaperHeight[-1];
    3: FM.PaperHeight[FM.PaperCount];
    4: FM.PaperWidth[-1];
    5: FM.PaperWidth[FM.PaperCount];
    6: FM.WidthByName['NoPaper'];
    7: FM.HeightByName['NoPaper'];
  end;
end;

procedure TTestFPPaperManager.Setup;
begin
  inherited Setup;
  FAccess := -1;
end;

procedure TTestFPPaperManager.TestCreate;
begin
  AssertEquals('No registered papers', 0, FM.PaperCount);
end;

procedure TTestFPPaperManager.TestRegister1;
begin
  RegisterPapers(1);
  AssertEquals('1 registered paper', 1, FM.PaperCount);
  AssertEquals('Correct name', 'P3', FM.PaperNames[0]);
end;

procedure TTestFPPaperManager.TestRegister2;
begin
  RegisterPapers(2);
  AssertEquals('2 registered papers', 2, FM.PaperCount);
  AssertEquals('Correct name paper 1', 'P2', FM.PaperNames[0]);
  AssertEquals('Correct name paper 2', 'P3', FM.PaperNames[1]);
end;

procedure TTestFPPaperManager.TestRegister3;
begin
  RegisterPapers(3);
  AssertEquals('3 registered papers', 3, FM.PaperCount);
  AssertEquals('Correct name paper 1', 'P1', FM.PaperNames[0]);
  AssertEquals('Correct name paper 2', 'P2', FM.PaperNames[1]);
  AssertEquals('Correct name paper 3', 'P3', FM.PaperNames[2]);
end;

procedure TTestFPPaperManager.TestRegisterDuplicate;
begin
  RegisterPapers(2);
  AssertEquals('2 registered papers', 2, FM.PaperCount);
  AssertEquals('Correct name paper 1', 'P2', FM.PaperNames[0]);
  AssertEquals('Correct name paper 2', 'P3', FM.PaperNames[1]);
  try
    FM.RegisterPaper('P3', 10.0, 20.0);
    Fail('We expected an exception to be raised.');
  except
    on E: Exception do
      begin
        AssertEquals('Exception class', 'EReportError', E.ClassName);
        AssertEquals('Exception message', 'Paper name P3 already exists', E.Message);
      end;
  end;
end;

procedure TTestFPPaperManager.TestClear;
begin
  RegisterPapers(2);
  AssertEquals('2 registered papers', 2, FM.PaperCount);
  AssertEquals('Correct name paper 1', 'P2', FM.PaperNames[0]);
  AssertEquals('Correct name paper 2', 'P3', FM.PaperNames[1]);
  FM.Clear;
  AssertEquals('0 registered papers', 0, FM.PaperCount);
end;

procedure TTestFPPaperManager.TestFind1;
begin
  AssertEquals('No paper registered', -1, FM.IndexOfPaper('P1'));
end;

procedure TTestFPPaperManager.TestFind2;
begin
  RegisterPapers(3);
  AssertEquals('No paper registered', -1, FM.IndexOfPaper('PA1'));
end;

procedure TTestFPPaperManager.TestFind3;
begin
  RegisterPapers(3);
  AssertEquals('3 registered papers', 3, FM.PaperCount);
  AssertEquals('Find P1 OK', 0, FM.IndexOfPaper('P1'));
  AssertEquals('Find P2 OK', 1, FM.IndexOfPaper('P2'));
  AssertEquals('Find P3 OK', 2, FM.IndexOfPaper('P3'));
end;

procedure TTestFPPaperManager.IllegalAccess1;
begin
  RegisterPapers(3);
  FAccess := 0;
  AssertException('Papername[-1]', EStringListError, @TestAccess);
end;

procedure TTestFPPaperManager.IllegalAccess2;
begin
  RegisterPapers(3);
  FAccess := 1;
  AssertException('Papername[3]', EStringListError, @TestAccess);
end;

procedure TTestFPPaperManager.IllegalAccess3;
begin
  RegisterPapers(3);
  FAccess := 2;
  AssertException('PaperHeight[-1]', EStringListError, @TestAccess);
end;

procedure TTestFPPaperManager.IllegalAccess4;
begin
  RegisterPapers(3);
  FAccess := 3;
  AssertException('PaperHeight[3]', EStringListError, @TestAccess);
end;

procedure TTestFPPaperManager.IllegalAccess5;
begin
  RegisterPapers(3);
  FAccess := 4;
  AssertException('PaperWidth[-1]', EStringListError, @TestAccess);
end;

procedure TTestFPPaperManager.IllegalAccess6;
begin
  RegisterPapers(3);
  FAccess := 5;
  AssertException('PaperWidth[3]', EStringListError, @TestAccess);
end;

procedure TTestFPPaperManager.IllegalAccess7;
begin
  RegisterPapers(3);
  FAccess := 6;
  AssertException('WidthByName[NoPaper]', EReportError, @TestAccess);
end;

procedure TTestFPPaperManager.IllegalAccess8;
begin
  RegisterPapers(3);
  FAccess := 7;
  AssertException('WidthByName[NoPaper]', EReportError, @TestAccess);
end;

procedure TTestFPPaperManager.TestWidth;
begin
  RegisterPapers(3);
  AssertEquals('Paper width 0', 16.0, FM.PaperWidth[0]);
  AssertEquals('Paper width 1', 4.0, FM.PaperWidth[1]);
  AssertEquals('Paper width 2', 1.0, FM.PaperWidth[2]);
  AssertEquals('Width[P1]', 16.0, FM.WidthByName['P1']);
  AssertEquals('Width[P2]', 4.0, FM.WidthByName['P2']);
  AssertEquals('Width[P3]', 1, FM.WidthByName['P3']);
end;

procedure TTestFPPaperManager.TestHeight;
begin
  RegisterPapers(3);
  AssertEquals('Paper height 0', 32.0, FM.PaperHeight[0]);
  AssertEquals('Paper height 1', 8.0, FM.PaperHeight[1]);
  AssertEquals('Paper height 2', 2.0, FM.PaperHeight[2]);
  AssertEquals('Height[P1]', 32.0, FM.HeightByName['P1']);
  AssertEquals('Height[P2]', 8.0, FM.HeightByName['P2']);
  AssertEquals('Height[P3]', 2, FM.HeightByName['P3']);
end;

{ TMyFPReportPageSize }

procedure TMyFPReportPageSize.ResetChanged;
begin
  FChangedCalled := 0;
end;

procedure TMyFPReportPageSize.Changed;
begin
  Inc(FChangedCalled);
  inherited Changed;
end;

{ TTestFPReportPageSize }

procedure TTestFPReportPageSize.Setup;
begin
  inherited Setup;
  FP := TMyFPReportPageSize.Create(nil);
end;

procedure TTestFPReportPageSize.TearDown;
begin
  FreeAndNil(FP);
  inherited TearDown;
end;

procedure TTestFPReportPageSize.TestCreate;
begin
  AssertNull('No page', FP.Page);
  AssertEquals('Zero width at create', 0.0, FP.Width);
  AssertEquals('Zero height at create', 0.0, FP.Height);
  AssertEquals('No paper name', '', FP.PaperName);
end;

procedure TTestFPReportPageSize.TestCreateWithPage;
var
  P: TFPReportPage;
  F: TFPReportPageSize;
begin
  P := TFPReportPage.Create(nil);
  try
    F := TFPReportPageSize.Create(P);
    try
      AssertSame('Pagesize created with page has page as page', P, F.Page);
    finally
      F.Free;
    end;
  finally
    P.Free
  end;
end;

procedure TTestFPReportPageSize.TestCreateByPage;
var
  P: TFPReportPage;
begin
  P := TFPReportPage.Create(nil);
  try
    AssertSame('Pagesize created with page has page as page', P, P.PageSize.Page);
  finally
    P.Free
  end;
end;

procedure TTestFPReportPageSize.TestChanged1;
begin
  FP.Width := 1.23;
  AssertEquals('Setting width triggers change', 1, FP.ChangedCalled);
end;

procedure TTestFPReportPageSize.TestChanged2;
begin
  FP.Height := 1.23;
  AssertEquals('Setting height triggers change', 1, FP.ChangedCalled);
end;

procedure TTestFPReportPageSize.TestChanged3;
begin
  FP.PaperName := 'ABC';
  AssertEquals('Setting paper name without associated paper does not trigger change', 0, FP.ChangedCalled);
end;

procedure TTestFPReportPageSize.TestPaperName1;
var
  F: TFPReportPaperManager;
begin
  F := PaperManager;
  if F.PaperCount = 0 then
    Registerpapers(3, False);
  FP.PaperName := F.PaperNames[0];
  AssertEquals('Setting papername sets width', F.PaperWidth[0], FP.Width);
  AssertEquals('Setting papername sets height', F.PaperHeight[0], FP.Height);
  AssertEquals('Setting papername calls changed once', 1, FP.ChangedCalled);
end;

procedure TTestFPReportPageSize.TestPaperName2;
var
  F: TFPReportPaperManager;
begin
  F := PaperManager;
  if F.PaperCount = 0 then
    Registerpapers(3, False);
  FP.PaperName := F.PaperNames[0];
  AssertEquals('Setting papername sets width', F.PaperWidth[0], FP.Width);
  AssertEquals('Setting papername sets height', F.PaperHeight[0], FP.Height);
  FP.ResetChanged;
  FP.PaperName := 'aloha'; // Non existing
  AssertEquals('Setting non-existing papername leaves width', F.PaperWidth[0], FP.Width);
  AssertEquals('Setting non-existing papername leaves height', F.PaperHeight[0], FP.Height);
  AssertEquals('Setting non-existing papername does not call changed', 0, FP.ChangedCalled);
end;

procedure TTestFPReportPageSize.TestAssign;
var
  F: TMyFPreportPageSize;
begin
  F := TMyFPreportPageSize.Create(nil);
  try
    FP.PaperName := 'me';
    FP.Width := 1.23;
    FP.Height := 4.56;
    F.Assign(FP);
    AssertEquals('Assign assigns Width', FP.Width, F.Width);
    AssertEquals('Assign assigns height', FP.Height, F.Height);
    AssertEquals('Assign assigns papername', FP.PaperName, F.PaperName);
    AssertEquals('Assign calls Changed once', 1, F.ChangedCalled);
  finally
    F.Free;
  end;
end;

{ TMyFPReportPage }

procedure TMyFPReportPage.SetupPage;
begin
  Orientation := poPortrait;
  { paper size }
  PageSize.PaperName := 'A4';
  { page margins }
  Margins.Left := 30;
  Margins.Top := 20;
  Margins.Right := 30;
  Margins.Bottom := 20;
end;

procedure TMyFPReportPage.PrepareObjects(aRTParent: TFPReportElement);

begin
  Inc(FPrepareObjectsCalled);
  inherited PrepareObjects(aRTParent);
end;

constructor TMyFPReportPage.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  Font.Name := 'LiberationSerif';
end;

procedure TMyFPReportPage.ResetChanged;
begin
  FChangedCalled := 0;
end;

procedure TMyFPReportPage.DoChanged;
begin
  Inc(FChangedCalled);
  inherited DoChanged;
end;

{ TMyReportTitleBand }

procedure TMyReportTitleBand.PrepareObjects(aRTParent: TFPReportElement);

begin
  Inc(FPrepareObjectsCalled);
  inherited PrepareObjects(aRTParent);
end;

constructor TMyReportTitleBand.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  Layout.Height := 20;
end;

{ TMyDataBand }

procedure TMyDataBand.PrepareObjects(aRTParent: TFPReportElement);

begin
  Inc(FPrepareObjectsCalled);
  inherited PrepareObjects(aRTParent);
end;

constructor TMyDataBand.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  Layout.Height := 10;
end;

{ TTestReportPage }

procedure TTestReportPage.Setup;
begin
  inherited Setup;
  FP := TMyFPReportPage.Create(nil);
end;

procedure TTestReportPage.TearDown;
begin
  FreeAndNil(FP);
  inherited TearDown;
end;

procedure TTestReportPage.TestCreate1;
begin
  AssertNull('Created page without parent has no report', FP.Report);
  AssertNotNull('Created page has margins', FP.Margins);
  AssertNotNull('Created page has pagesize', FP.PageSize);
  AssertEquals('Orientation is portrait', Ord(poPortrait), Ord(FP.Orientation));
  AssertEquals('No bands', 0, FP.BandCount);
end;

procedure TTestReportPage.TestCreate2;
var
  R: TFPReport;
  P: TMyFPReportPage;
begin
  R := TFPReport.Create(nil);
  try
    P := TMyFPReportPage.Create(nil);
    try
      P.Report := R;
      AssertSame('Page owner is report when created', R, P.Report);
      AssertEquals('Report has one page', 1, R.PageCount);
      AssertSame('Page added to pages', P, R.Pages[0]);
    finally
      R.Free;
    end;
    AssertNull('Report has notified page', P.Report);
  finally
    P.Free;
  end;
end;

procedure TTestReportPage.TestCreate3;
var
  R: TFPReport;
  P: TMyFPReportPage;
begin
  R := TFPReport.Create(nil);
  P := TMyFPReportPage.Create(R); // Lets try passing Report as the AOwner in constructor
  try
    AssertSame('Page report is set', R, P.Report);
    AssertSame('Page added to pages', P, R.Pages[0]);
    P.Report := nil;
    AssertEquals('No more pages', 0, R.PageCount);
  finally
    // This will free P as well, because R was set as the owner
    R.Free;
  end;
end;

procedure TTestReportPage.TestPageSize1;
begin
  FP.ResetChanged;
  FP.BeginUpdate;
  try
    FP.PageSize.Width := 10;
    FP.PageSize.Height := 20;
  finally
    FP.EndUpdate;
  end;
  AssertEquals('Changed called', 1, FP.ChangedCalled);
  AssertEquals('Top is zero', 0, FP.Layout.Top);
  AssertEquals('Left is zero', 0, FP.Layout.Left);
  AssertEquals('Width is pagewidth', FP.PageSize.Width, FP.Layout.Width);
  AssertEquals('Height is pageheight', FP.PageSize.Height, FP.Layout.Height);
end;

procedure TTestReportPage.TestPageSize2;
begin
  FP.ResetChanged;
  FP.BeginUpdate;
  try
    FP.PageSize.Width := 10;
    FP.PageSize.Height := 20;
    FP.Margins.Left := 1;
    FP.Margins.Right := 2;
    FP.Margins.Top := 3;
    FP.Margins.Bottom := 4;
  finally
    FP.EndUpdate;
  end;
  AssertEquals('Changed called', 1, FP.ChangedCalled);
  AssertEquals('Top is top margin', 3, FP.Layout.Top);
  AssertEquals('Left is left margin', 1, FP.Layout.Left);
  AssertEquals('Width is pagewidth-rightmargin-leftmargin', 7, FP.Layout.Width);
  AssertEquals('Height is pageheight-topmargin-bottommargin', 13, FP.Layout.Height);
end;

procedure TTestReportPage.TestPageSize3;
begin
  FP.ResetChanged;
  FP.BeginUpdate;
  try
    FP.Orientation := poLandScape;
    FP.PageSize.Width := 10;
    FP.PageSize.Height := 20;
    FP.Margins.Left := 1;
    FP.Margins.Right := 2;
    FP.Margins.Top := 3;
    FP.Margins.Bottom := 4;
  finally
    FP.EndUpdate;
  end;
  AssertEquals('Changed called', 1, FP.ChangedCalled);
  AssertEquals('Top is top margin', 3, FP.Layout.Top);
  AssertEquals('Left is left margin', 1, FP.Layout.Left);
  AssertEquals('Width is pageheight-rightmargin-leftmargin', 17, FP.Layout.Width);
  AssertEquals('Height is pagewidth-topmargin-bottommargin', 3, FP.Layout.Height);
end;

procedure TTestReportPage.TestBand1;
var
  B: TFPReportCustomBand;
begin
  B := TFPReportCustomBand.Create(nil);
  try
    FP.ResetChanged;
    B.Parent := FP;
    AssertEquals('Changed called', 1, FP.ChangedCalled);
    AssertSame('Parent stored correctly', FP, B.Page);
    AssertEquals('Bandcount correct', 1, FP.BandCount);
    AssertSame('Bands[0] correct', B, FP.Bands[0]);
  finally
    B.Free;
  end;
  AssertEquals('Bandcount correct', 0, FP.BandCount);
end;

procedure TTestReportPage.TestBand2;
var
  B: TFPReportCustomBand;
  P: TMyFPReportPage;
begin
  P := TMyFPReportPage.Create(nil);
  try
    B := TFPReportCustomBand.Create(nil);
    try
      B.Parent := P;
      AssertSame('Parent stored correctly', P, B.Page);
      AssertEquals('Bandcount correct', 1, P.BandCount);
      AssertSame('Bands[0] correct', B, P.Bands[0]);
    finally
      B.Free;
    end;
    AssertEquals('Page notified that Band is gone', 0, P.BandCount);
  finally
    P.Free;
  end;
end;

procedure TTestReportPage.TestData;
var
  FData: TFPReportData;
begin
  FData := TFPReportData.Create(nil);
  try
    FP.Data := FData;
  finally
    FData.Free;
  end;
  AssertNull('Data is cleared', FP.Data);
end;

procedure TTestReportPage.TestAssign;
var
  E: TFPReportPage;
begin
  E := TFPReportPage.Create(nil);
  try
    FP.Layout.Top := 1;
    FP.Frame.Width := 2;
    E.Assign(FP);
    AssertEquals('Failed on 1', True, FP.Frame.Equals(E.Frame));
    AssertEquals('Failed on 2', True, FP.Layout.Equals(E.Layout));
    AssertEquals('Failed on 3', Ord(E.Orientation), Ord(FP.Orientation));
    AssertEquals('Failed on 4', True, FP.Margins.Equals(E.Margins));
  finally
    E.Free;
  end;
end;

procedure TTestReportPage.TestFindBand;
var
  t: TFPReportTitleBand;
  h: TFPReportPageHeaderBand;
  f: TFPReportPageFooterBand;
  d: TFPReportDataBand;
begin
  t := TFPReportTitleBand.Create(FP);
  h := TFPReportPageHeaderBand.Create(FP);
  f := TFPReportPageFooterBand.Create(FP);
  d := TFPReportDataBand.Create(FP);
  AssertTrue('failed on 1', h = FP.FindBand(TFPReportPageHeaderBand));
  AssertTrue('failed on 2', t <> FP.FindBand(TFPReportPageHeaderBand));
  AssertTrue('failed on 3', t = FP.FindBand(TFPReportTitleBand));
  AssertTrue('failed on 4', f = FP.FindBand(TFPReportPageFooterBand));
  AssertTrue('failed on 5', d = FP.FindBand(TFPReportDataBand));
  AssertTrue('failed on 6', FP.FindBand(TFPReportChildBand) = nil);
end;

{ TMyFPReportData }

procedure TMyFPReportData.ResetCounts;
begin
  FCC := 0;
  FDFC := 0;
  FEC := 0;
  FFC := 0;
  FNC := 0;
  FOC := 0;
end;

procedure TMyFPReportData.DoInitDataFields;
begin
  inherited DoInitDataFields;
  Inc(FDFC);
end;

procedure TMyFPReportData.DoOpen;
begin
  inherited DoOpen;
  Inc(FOC);
end;

procedure TMyFPReportData.DoFirst;
begin
  inherited DoFirst;
  Inc(FFC);
end;

procedure TMyFPReportData.DoNext;
begin
  inherited DoNext;
  Inc(FNC);
end;

procedure TMyFPReportData.DoClose;
begin
  inherited DoClose;
  Inc(FCC);
end;

function TMyFPReportData.DoEOF: boolean;
begin
  FOE := inherited DoEOF;
  Inc(FEC);
  Result := FReportEOF;
end;

{ TTestReportData }

procedure TTestReportData.DoOpen(Sender: TObject);
begin
  FHandler := True;
  AssertEquals('OnOpen called before DoOpen', 0, FD.OpenCount);
  AssertEquals('OnOpen called before InitFieldDefs', 0, FD.InitDataFieldsCount);
end;

procedure TTestReportData.DoNext(Sender: TObject);
begin
  FHandler := True;
  AssertEquals('DoNext not yet called in handler', 0, FD.NextCount);
  AssertEquals('Recno is already 2 in donext', 2, FD.RecNo);
end;

procedure TTestReportData.Setup;
begin
  inherited Setup;
  FD := TMyFPReportData.Create(nil);
  FHandler := False;
end;

procedure TTestReportData.TearDown;
begin
  FreeAndNil(FD);
  inherited TearDown;
end;

procedure TTestReportData.CreateFields;
begin
  FD.DataFields.AddField('string', rfkString).DisplayWidth := 10;
  FD.DataFields.AddField('boolean', rfkBoolean).DisplayWidth := 20;
  FD.DataFields.AddField('integer', rfkInteger).DisplayWidth := 30;
  FD.DataFields.AddField('float', rfkFloat).DisplayWidth := 40;
  FD.DataFields.AddField('datetime', rfkDateTime).DisplayWidth := 50;
  FD.Datafields.AddField('stream', rfkStream).DisplayWidth := 60;
end;

procedure TTestReportData.DoFieldByName;
var
  F: TFPReportDataField;
begin
  F := FD.Datafields.FieldByName('ohlala');
end;

procedure TTestReportData.TestCreate;
begin
  AssertEquals('Closed recno is 0', 0, FD.RecNo);
  AssertNotNull('DataFields created', FD.DataFields);
  AssertEquals('Closed fieldcount is 0', 0, FD.DataFields.Count);
  AssertSame('Datafields reportdata is self', FD, FD.DataFields.ReportData);
end;

procedure TTestReportData.TestOpen1;
begin
  FD.OnOpen := @DoOpen;
  FD.Open;
  AssertEquals('OnOpen Handler called', True, FHandler);
  AssertEquals('DoOpen called once', 1, FD.OpenCount);
  AssertEquals('InitFieldDefs called once', 1, FD.InitDataFieldsCount);
  AssertEquals('Recno is 1', 1, FD.RecNo);
end;

procedure TTestReportData.TestNext;
begin
  FD.OnNext := @DoNext;
  FD.Open;
  FHandler := False;
  FD.Next;
  AssertEquals('OnNext Handler called', True, FHandler);
  AssertEquals('DoNext Called once', 1, FD.NextCount);
  AssertEquals('Recno is 2 after next', 2, FD.RecNo);
end;

procedure TTestReportData.TestInitFieldDefs;
begin
  FD.InitFieldDefs;
  AssertEquals('InitFieldDefs called once', 1, FD.InitDataFieldsCount);
end;

procedure TTestReportData.TestInitFieldDefs_OnlyAllowedOnce;
begin
  FD.Open;
  AssertEquals('Failed on 1', 1, FD.InitDataFieldsCount);
  try
    FD.InitFieldDefs;
    Fail('Failed on 2. - we should not have reached here.');
  except
    on E: Exception do
    begin
      AssertEquals('Failed on 3', E.ClassName, 'EReportError');
    end;
  end;
  AssertEquals('Failed on 4', 1, FD.InitDataFieldsCount);
end;

procedure TTestReportData.TestEOF1;
begin
  FD.ReportEOF := True;
  AssertEquals('ReportEOF works correctly', True, FD.EOF);
  AssertEquals('Inherited EOF returns false', False, FD.OldEOF);
end;

procedure TTestReportData.TestAddDatafield;
var
  F: TFPReportDataField;
begin
  F := FD.DataFields.AddField('test', rfkBoolean);
  AssertEquals('Boolean field Added', Ord(rfkBoolean), Ord(F.FieldKind));
  AssertEquals('test field name Added', 'test', F.fieldname);
  AssertEquals('0 width field Added', 0, F.DisplayWidth);
end;

procedure TTestReportData.TestDatafieldAdd;
var
  I: TCollectionItem;
  F: TFPReportDataField;
begin
  I := FD.Datafields.Add;
  AssertEquals('add creates TFPReportDataField', TFPReportDataField, I.ClassType);
  F := I as TFPReportDataField;
  AssertEquals('Default field of string kind', Ord(rfkString), Ord(F.FieldKind));
  AssertEquals('Default field name empty', '', F.FieldName);
  AssertEquals('Default field with 0', 0, F.DisplayWidth);
end;

procedure TTestReportData.AssertField(Prefix: string; F: TFPReportDataField; AFieldName: string;
  AFieldKind: TFPReportFieldKind; ADisplayWidth: integer = 0);
var
  S1, S2: string;
begin
  AssertEquals(Prefix + ' has correct field name', AfieldName, F.FieldName);
  S1 := GetEnumName(TypeInfo(TFPReportFieldKind), Ord(AFieldKind));
  S2 := GetEnumName(TypeInfo(TFPReportFieldKind), Ord(F.FieldKind));
  AssertEquals(Prefix + ' has corrrect fieldkind', S1, S2);
  AssertEquals(Prefix + ' has correct fieldwidth', ADisplayWidth, F.DisplayWidth);
end;

procedure TTestReportData.TestCreateFields;
begin
  CreateFields;
  AssertEquals('Correct field count', 6, FD.FieldCount);
  AssertField('Field 0', FD.DataFields[0], 'string', rfkString, 10);
  AssertField('Field 1', FD.DataFields[1], 'boolean', rfkBoolean, 20);
  AssertField('Field 2', FD.DataFields[2], 'integer', rfkInteger, 30);
  AssertField('Field 3', FD.DataFields[3], 'float', rfkFloat, 40);
  AssertField('Field 4', FD.DataFields[4], 'datetime', rfkDateTime, 50);
  AssertField('Field 5', FD.DataFields[5], 'stream', rfkStream, 60);
end;

procedure TTestReportData.TestDatafieldIndexOf1;
begin
  CreateFields;
  AssertEquals('Finds field at pos 0', 0, FD.DataFields.IndexOfField('string'));
  AssertEquals('Finds field at pos 3', 3, FD.DataFields.IndexOfField('float'));
  AssertEquals('Finds field at pos 5', 5, FD.DataFields.IndexOfField('stream'));
  AssertEquals('Finds field (casing) at pos 3', 3, FD.DataFields.IndexOfField('Float'));
end;

procedure TTestReportData.TestDatafieldIndexOf2;
begin
  AssertEquals('No fields returns -1', -1, FD.DataFields.IndexOfField('string'));
  CreateFields;
  AssertEquals('Non-existing field returns -1', -1, FD.DataFields.IndexOfField('stringlslsl'));
end;

procedure TTestReportData.TestFindField1;
begin
  AssertNull('No fields returns Nil', FD.DataFields.FindField('string'));
  CreateFields;
  AssertNull('Non-existing fields returns Nil', FD.DataFields.FindField('stringsss'));
end;

procedure TTestReportData.TestFindField2;
begin
  CreateFields;
  AssertSame('FindField returns correct field', FD.DataFields[0], FD.DataFields.FindField('string'));
  AssertSame('FindField returns correct field', FD.DataFields[3], FD.DataFields.FindField('float'));
  AssertSame('FindField returns correct field (case insensitive)', FD.DataFields[3], FD.DataFields.FindField('floaT'));
end;

procedure TTestReportData.TestFindByName1;
begin
  CreateFields;
  AssertSame('FieldByName returns correct field', FD.DataFields[0], FD.DataFields.FieldByName('string'));
end;

procedure TTestReportData.TestFindByName2;
begin
  CreateFields;
  AssertException('FieldByName (non-existent) raises exception', EReportError, @DoFieldByName);
end;

procedure TTestReportData.TestFieldAssign;
var
  F1, F2: TFPReportDataField;
begin
  F1 := TFPReportDataField.Create(nil);
  try
    f2 := TFPReportDataField.Create(nil);
    try
      F1.FieldKind := rfkBoolean;
      F1.FieldName := 'bool';
      F1.DisplayWidth := 12;
      F2.Assign(F1);
      AssertField('Assigned ', F2, 'bool', rfkBoolean, 12);
    finally
      F2.Free;
    end;
  finally
    F1.Free;
  end;
end;

procedure TTestReportData.TestGetValue;
var
  v: variant;
begin
  CreateFields;
  v := FD.Datafields[0].GetValue;
  AssertTrue('Failed on 1', V = Null);
end;

procedure TTestReportData.TestEasyAccessProperties;
var
  I: integer;
begin
  CreateFields;
  for I := 0 to FD.FieldCount - 1 do
    AssertEquals('FieldNames array OK', FD.DataFields[0].FieldName, FD.FieldNames[0]);
  for I := 0 to FD.FieldCount - 1 do
    AssertEquals('FieldWidth array OK', FD.DataFields[0].DisplayWidth, FD.FieldWidths[FD.FieldNames[0]]);
  for I := 0 to FD.FieldCount - 1 do
    AssertEquals('FieldTypes array OK', Ord(FD.DataFields[0].FieldKind), Ord(FD.FieldTypes[FD.FieldNames[0]]));
end;

{ TTestUserReportData }

procedure TTestUserReportData.Setup;
begin
  FD := TFPReportUserData.Create(nil);
  FD.DataFields.AddField('string', rfkString);
  FD.OnGetValue := @DoValue;
  inherited;
end;

procedure TTestUserReportData.TearDown;
begin
  FreeAndNil(FD);
  inherited TearDown;
end;

procedure TTestUserReportData.DoValue(Sender: TObject; const AValueName: string; var AValue: variant);
begin
  AssertSame('DoValue Sender is reportdata', FD, Sender);
  AssertEquals('DoValue gets correct value name', FExpectName, AValueName);
  AValue := FReturnValue;
end;

procedure TTestUserReportData.TestGetValue;
begin
  FExpectName := 'string';
  FReturnValue := 10;
  AssertEquals('Return value correct', 10, FD.DataFields[0].GetValue);
  AssertEquals('FieldValues array value correct', 10, FD.FieldValues['string']);
end;

{ TTestUserReportData2 }

procedure TTestUserReportData2.DoGetValue(Sender: TObject; const AValueName: string; var AValue: variant);
begin
  if AValueName = 'element' then
    AValue := FSL[FData.RecNo - 1];
end;

procedure TTestUserReportData2.DoGetEOF(Sender: TObject; var IsEOF: boolean);
begin
  if FData.RecNo > FSL.Count then
    IsEOF := True
  else
    IsEOF := False;
end;

procedure TTestUserReportData2.Setup;
begin
  inherited Setup;
  FData := TFPReportUserData.Create(nil);
  FData.OnGetValue := @DoGetValue;
  // data is coming from the stringlist this time
  FSL := TStringList.Create;
  FSL.Add('Item 1');
  FSL.Add('Item 2');
  FSL.Add('Item 3');
  FSL.Add('Item 4');
end;

procedure TTestUserReportData2.TearDown;
begin
  FData.Free;
  FSL.Free;
  inherited TearDown;
end;

procedure TTestUserReportData2.TestGetValue;
begin
  FData.First;
  AssertEquals('Failed on 1', 'Item 1', FData.FieldValues['element']);
  FData.Next;
  AssertEquals('Failed on 2', 'Item 2', FData.FieldValues['element']);
  FData.Next;
  AssertEquals('Failed on 3', 'Item 3', FData.FieldValues['element']);
  FData.Next;
  AssertEquals('Failed on 4', 'Item 4', FData.FieldValues['element']);
  FData.Next;
end;

procedure TTestUserReportData2.TestOnGetEOF1;
var
  i: integer;
begin
  FData.First;
  for i := 1 to FSL.Count do
    FData.Next;
  // Should be False, because we haven't assigned OnGetEOF event handler
  AssertTrue('Failed on 1', FData.EOF = False);
end;

procedure TTestUserReportData2.TestOnGetEOF2;
var
  i: integer;
begin
  FData.OnGetEOF := @DoGetEOF;
  FData.First;
  for i := 1 to FSL.Count do
    FData.Next;
  AssertTrue('Failed on 1', FData.EOF = True);
end;

{ TTestDataBand }

procedure TTestDataBand.Setup;
begin
  FDataBand := TFPReportDataBand.Create(nil);
  inherited Setup;
end;

procedure TTestDataBand.TearDown;
begin
  FreeAndNil(FDataBand);
  inherited TearDown;
end;

procedure TTestDataBand.TestData;
var
  D: TFPReportData;
begin
  D := TFPReportData.Create(nil);
  try
    FDataBand.Data := D;
    AssertSame('Assigned data OK', D, FDataBand.Data)
  finally
    D.Free;
  end;
  AssertNull('Free notification of Data', FDataBand.Data);
end;

procedure TTestDataBand.TestDataPropertyAutoSet;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  D: TFPReportData;
begin
  SetReportData(2);

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page1';
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  // DataBand should have been assigned p.Data automatically
  AssertSame('Failed on 1', TFPReportData(Data), DataBand.Data);
  D := TFPReportData.Create(nil);
  try
    DataBand.Data := D;
    AssertTrue('Failed on 2', p.Data <> DataBand.Data);
  finally
    D.Free;
  end;
end;

{ TTestCustomReport }

procedure TTestCustomReport.HandleOnBeginReport;
begin
  Inc(FBeginReportCount);
end;

procedure TTestCustomReport.HandleOnEndReport;
begin
  Inc(FEndReportCount);
end;

procedure TTestCustomReport.InitializeData(const ACount: integer);
var
  i: integer;
begin
  // data is coming from the stringlist this time
  FSL := TStringList.Create;
  if ACount < 1 then
    Exit;
  for i := 1 to ACount do
    FSL.Add('Item ' + IntToStr(i));
end;

procedure TTestCustomReport.SetReportData(const ADataCount: Byte);
begin
  if ADataCount < 1 then
    Exit;
  InitializeData(ADataCount);
  FData := TFPReportUserData.Create(nil);
  FData.OnGetValue := @DoGetDataValue;
  FData.OnGetEOF := @DoGetDataEOF;
  FData.OnGetNames := @DoGetDataFieldNames;
end;

procedure TTestCustomReport.DoGetDataValue(Sender: TObject; const AValueName: string; var AValue: variant);
begin
  if AValueName = 'element' then
    AValue := FSL[FData.RecNo - 1];
end;

procedure TTestCustomReport.DoGetDataEOF(Sender: TObject; var IsEOF: boolean);
begin
  if FData.RecNo > FSL.Count then
    IsEOF := True
  else
    IsEOF := False;
end;

procedure TTestCustomReport.Setup;
begin
  inherited Setup;
  PaperManager.Clear;
  PaperManager.RegisterStandardSizes;
  Report := TMyCustomReport.Create(nil);
  FBeginReportCount := 0;
  FEndReportCount := 0;

  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Text := 'fonts';
  gTTFontCache.BuildFontCache;
end;

procedure TTestCustomReport.TearDown;
begin
  FreeAndNil(FRpt);
  FreeAndNil(FData);
  FreeAndNil(FSL);
  inherited TearDown;
end;

procedure TTestCustomReport.DoGetDataFieldNames(Sender: TObject; List: TStrings);
begin
  List.Add('element');
end;

procedure TTestCustomReport.TestBeginReportEvent;
begin
  TMyFPReportPage.Create(Report); // add at least one page
  Report.OnBeginReport := @HandleOnBeginReport;
  AssertEquals('Failed on 1', 0, FBeginReportCount);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, FBeginReportCount);
  AssertEquals('Failed on 3', 0, FEndReportCount);
end;

procedure TTestCustomReport.TestEndReportEvent;
begin
  TMyFPReportPage.Create(Report); // add at least one page
  Report.OnEndReport := @HandleOnEndReport;
  AssertEquals('Failed on 1', 0, FEndReportCount);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, FEndReportCount);
  AssertEquals('Failed on 3', 0, FBeginReportCount);
end;

procedure TTestCustomReport.TestPagePrepareObjects;
var
  p: TMyFPReportPage;
begin
  SetReportData(2);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.Data := Data;

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page2';
  p.Data := Data;

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page3';
  p.Data := Data;

  AssertEquals('Failed on 1', 0, TMyFPReportPage(Report.Pages[0]).FPrepareObjectsCalled);
  AssertEquals('Failed on 2', 0, TMyFPReportPage(Report.Pages[1]).FPrepareObjectsCalled);
  AssertEquals('Failed on 3', 0, TMyFPReportPage(Report.Pages[2]).FPrepareObjectsCalled);

  Report.RunReport;
  // due to Re-interpret of Page.Data, page is prepared per record (r38906)
  AssertEquals('Failed on 4', 2, TMyFPReportPage(Report.Pages[0]).FPrepareObjectsCalled);
  AssertEquals('Failed on 5', 2, TMyFPReportPage(Report.Pages[1]).FPrepareObjectsCalled);
  AssertEquals('Failed on 6', 2, TMyFPReportPage(Report.Pages[2]).FPrepareObjectsCalled);
end;

procedure TTestCustomReport.TestBandPrepareObjects;
var
  p: TMyFPReportPage;
  TitleBand: TMyReportTitleBand;
  DataBand: TMyDataBand;
begin
  SetReportData(2);

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page1';
  p.Data := Data;

  TitleBand := TMyReportTitleBand.Create(p);
  DataBand := TMyDataBand.Create(p);
  DataBand.Data := FData;

  AssertEquals('Failed on 1', 0, p.FPrepareObjectsCalled);
  AssertEquals('Failed on 2', 0, TitleBand.FPrepareObjectsCalled);
  AssertEquals('Failed on 3', 0, DataBand.FPrepareObjectsCalled);

  Report.RunReport;
  AssertEquals('Failed on 4', 1, p.FPrepareObjectsCalled);
  AssertEquals('Failed on 5', 1, TitleBand.FPrepareObjectsCalled);
  AssertEquals('Failed on 6', 2, DataBand.FPrepareObjectsCalled);
end;

procedure TTestCustomReport.TestRTObjects1;
var
  p: TMyFPReportPage;
begin
  SetReportData(2);

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page1';
  p.Data := Data;

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page2';
  p.Data := Data;

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page3';
  p.Data := Data;

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);

  Report.RunReport;
  // due to Re-interpret of Page.Data, page is prepared per record (r38906)
  AssertEquals('Failed on 2', 6, Report.RTObjects.Count);
end;

procedure TTestCustomReport.TestRTObjects2;
var
  p: TMyFPReportPage;
  TitleBand: TMyReportTitleBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(2);

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page1';
  p.Data := Data;

  TitleBand := TMyReportTitleBand.Create(p);
  Memo := TFPReportMemo.Create(TitleBand);
  Memo.Text := 'THE REPORT TITLE';
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  // due to Re-interpret of Page.Data, page is prepared per record (r38906)
  AssertEquals('Failed on 2', 2, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount);
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);

  {$IFDEF gdebug}
//  writeln(Report.DebugPreparedPageAsJSON(0));
  {$ENDIF}
end;

procedure TTestCustomReport.TestRTObjects3;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(2);

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page1';
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[element]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 2, rtPage.ChildCount);
  AssertEquals('Failed on 5', 2, rtPage.BandCount);

  {$IFDEF gdebug}
//  writeln(Report.DebugPreparedPageAsJSON(0));
  {$ENDIF}
end;

procedure TTestCustomReport.TestRTObjects4_OneDataItem;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  p := TMyFPReportPage.Create(Report);
  p.SetupPage;
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[element]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount);
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);

  {$IFDEF gdebug}
//    writeln(Report.DebugPreparedPageAsJSON(0));
  {$ENDIF}
end;

procedure TTestCustomReport.TestRTObjects5_TwoDataItems;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(2);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Top := 0;
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[element]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 2, rtPage.ChildCount);
  AssertEquals('Failed on 5', 2, rtPage.BandCount); { each data row has its own data band }
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  AssertEquals('Failed on 7', 1, rtPage.Bands[1].ChildCount);

  {$IFDEF gdebug}
//    writeln(Report.DebugPreparedPageAsJSON(0));
  {$ENDIF}
end;

procedure TTestCustomReport.TestInternalFunction_Page;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[PageNo]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount);
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', '1', Memo.Text);
end;

procedure TTestCustomReport.TestInternalFunction_Page_with_text;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := 'Page [PageNo]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount);
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', 'Page 1', Memo.Text);
end;

procedure TTestCustomReport.TestInternalFunction_RecNo;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
  i: integer;
begin
  SetReportData(5);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[recno('''')]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 5, rtPage.ChildCount);  // 5 rendered data bands because we have 5 data records
  AssertEquals('Failed on 5', 5, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  for i := 0 to 4 do
  begin
    Memo := TFPReportMemo(rtPage.Bands[i].Child[0]);
    AssertEquals('Failed on 7.'+IntToStr(i), IntToStr(i+1), Memo.Text); { recno is 1-based }
  end;
end;

procedure TTestCustomReport.TestInternalFunction_Today;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[today]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount); // 1 rendered data band because we have 1 data record
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', FormatDateTime('yyyy-mm-dd', Today), Memo.Text);
end;

procedure TTestCustomReport.TestInternalFunction_Today_with_text;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := 'Today is [today]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount); // 1 rendered data band because we have 1 data record
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', 'Today is ' + FormatDateTime('yyyy-mm-dd', Today), Memo.Text);
end;

procedure TTestCustomReport.TestInternalFunction_Author;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[author]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount); // 1 rendered data band because we have 1 data record
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', '', Memo.Text); // we never set Report.Author
end;

procedure TTestCustomReport.TestInternalFunction_Author_with_text;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  Report.Author := 'Graeme Geldenhuys';
  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := 'The Author is [author].';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount); // 1 rendered data band because we have 1 data record
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', 'The Author is Graeme Geldenhuys.', Memo.Text);
end;

procedure TTestCustomReport.TestInternalFunction_Title;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := '[title]';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount); // 1 rendered data band because we have 1 data record
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', '', Memo.Text); // we never set Report.Title
end;

procedure TTestCustomReport.TestInternalFunction_Title_with_text;
var
  p: TMyFPReportPage;
  DataBand: TMyDataBand;
  Memo: TFPReportMemo;
  rtPage: TFPReportCustomPage;
begin
  SetReportData(1);

  Report.Title := 'My Test Report';
  p := TMyFPReportPage.Create(Report);
  p.Name := 'Page1';
  p.SetupPage;
  p.Data := Data;

  DataBand := TMyDataBand.Create(p);
  DataBand.Layout.Height := 23;

  Memo := TFPReportMemo.Create(DataBand);
  Memo.Layout.Top := 5;
  Memo.Layout.Left := 10;
  Memo.Text := 'Report Title is "[title]".';

  AssertEquals('Failed on 1', 0, Report.RTObjects.Count);
  Report.RunReport;
  AssertEquals('Failed on 2', 1, Report.RTObjects.Count); // runtime objects adhere to same hierarchy as design time
  AssertEquals('Failed on 3', 'TFPReportCustomPage', TObject(Report.RTObjects[0]).ClassName);
  rtPage := TFPReportCustomPage(Report.RTObjects[0]);
  AssertEquals('Failed on 4', 1, rtPage.ChildCount); // 1 rendered data band because we have 1 data record
  AssertEquals('Failed on 5', 1, rtPage.BandCount);
  AssertEquals('Failed on 6', 1, rtPage.Bands[0].ChildCount);
  Memo := TFPReportMemo(rtPage.Bands[0].Child[0]);
  AssertEquals('Failed on 7', 'Report Title is "My Test Report".', Memo.Text);
end;

{ TTestReportMemo }


procedure TTestReportMemo.CauseFontNotFoundException;
begin
  TMemoFriend(FMemo).RecalcLayout;
end;

procedure TTestReportMemo.SetUp;
begin
  inherited SetUp;
  FMemo := TFPReportMemo.Create(nil);
  FMemo.Layout.SetPosition(0, 0, 60, 5);
end;

procedure TTestReportMemo.TearDown;
begin
  FMemo.Free;
  inherited TearDown;
end;

procedure TTestReportMemo.TestCreate;
var
  m: TFPReportMemo;
begin
  m := TFPReportMemo.Create(nil);
  try
    m.Text := 'abc 123';
    AssertTrue('Failed on 1', m <> nil);
  finally
    m.Free;
  end;
end;

procedure TTestReportMemo.TestPrepareTextBlocks;
begin
  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Text := 'fonts';
  gTTFontCache.BuildFontCache;

  FMemo.Layout.Width := 100;
  FMemo.Text := 'abc 123';
  FMemo.UseParentFont := False;
  FMemo.Font.Name := 'Calibri';
  FMemo.StretchMode := smActualHeight;
  TMemoFriend(FMemo).CreateRTLayout;
  TMemoFriend(FMemo).RecalcLayout;
  AssertEquals('Failed on 2', 1, FMemo.TextLines.Count);
end;

procedure TTestReportMemo.TestPrepareTextBlocks_multiline_data;
begin
  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Text := 'fonts';
  gTTFontCache.BuildFontCache;

  FMemo.Layout.Width := 100;
  FMemo.Text := 'abc'+LineEnding+'123';
  FMemo.UseParentFont := False;
  FMemo.Font.Name := 'Calibri';
  FMemo.StretchMode := smActualHeight;
  TMemoFriend(FMemo).CreateRTLayout;
  TMemoFriend(FMemo).RecalcLayout;
  AssertEquals('Failed on 2', 2, FMemo.TextLines.Count);
end;

procedure TTestReportMemo.TestPrepareTextBlocks_multiline_wraptext;
begin
  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Text := 'fonts';
  gTTFontCache.BuildFontCache;

  FMemo.Layout.Width := 6;
  FMemo.Text := 'abc 123';
  FMemo.UseParentFont := False;
  FMemo.Font.Name := 'Calibri';
  FMemo.StretchMode := smActualHeight;
  FMemo.WordOverflow := woOverflow;
  TMemoFriend(FMemo).CreateRTLayout;
  TMemoFriend(FMemo).RecalcLayout;
  AssertEquals('Failed on 2', 2, FMemo.TextLines.Count);
end;

procedure TTestReportMemo.TestPrepareTextBlocks_multiline_wraptext_oneword;
begin
  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Text := 'fonts';
  gTTFontCache.BuildFontCache;

  FMemo.Layout.Width := 10;
  FMemo.Text := 'abc123';
  FMemo.UseParentFont := False;
  FMemo.Font.Name := 'Calibri';
  FMemo.StretchMode := smActualHeight;
  TMemoFriend(FMemo).CreateRTLayout;
  TMemoFriend(FMemo).RecalcLayout;
  AssertEquals('Failed on 1', 1, FMemo.TextLines.Count);
  // The length of abc1 fits.
  AssertEquals('Failed on 1', 'abc1', FMemo.TextLines[0]);
end;

procedure TTestReportMemo.TestPrepareTextBlocks_multiline_wraptext_oneword_overflow;
begin
  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Text := 'fonts';
  gTTFontCache.BuildFontCache;

  FMemo.Layout.Width := 10;
  FMemo.Text := 'abc123';
  FMemo.UseParentFont := False;
  FMemo.Font.Name := 'Calibri';
  FMemo.StretchMode := smActualHeight;
  TMemoFriend(FMemo).WordOverflow:=woOverflow;
  TMemoFriend(FMemo).CreateRTLayout;
  TMemoFriend(FMemo).RecalcLayout;
  AssertEquals('Failed on 1', 1, FMemo.TextLines.Count);
  AssertEquals('Failed on 1', 'abc123', FMemo.TextLines[0]);
end;

procedure TTestReportMemo.TestPrepareTextBlocks_multiline_wraptext_oneword_split;
begin
  gTTFontCache.Clear;
  gTTFontCache.SearchPath.Text := 'fonts';
  gTTFontCache.BuildFontCache;

  FMemo.Layout.Width := 10;
  FMemo.Text := 'abc123';
  FMemo.UseParentFont := False;
  FMemo.Font.Name := 'Calibri';
  FMemo.StretchMode := smActualHeight;
  TMemoFriend(FMemo).WordOverflow:=woSplit;
  TMemoFriend(FMemo).CreateRTLayout;
  TMemoFriend(FMemo).RecalcLayout;
  AssertEquals('Failed on 1', 2, FMemo.TextLines.Count);
  AssertEquals('Failed on 2', 'abc1', FMemo.TextLines[0]);
  AssertEquals('Failed on 3', '23', FMemo.TextLines[1]);
end;

procedure TTestReportMemo.TestRGBToReportColor;
var
  c: TFPReportColor;
begin
  c := RGBToReportColor(255, 0, 0);
  AssertEquals('failed on 1', IntToHex(clRed, 8), IntToHex(c, 8));
  c := RGBToReportColor(0, 128, 0);
  AssertEquals('failed on 2', IntToHex(clGreen, 8), IntToHex(c, 8));
  c := RGBToReportColor(0, 0, 255);
  AssertEquals('failed on 3', IntToHex(clBlue, 8), IntToHex(c, 8));
end;

procedure TTestReportMemo.TestHTMLColorToReportColor_length7;
var
  c: TFPReportColor;
begin
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('#FF0000', clBlack);
  AssertEquals('failed on 1', IntToHex(clRed, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('#008000', clBlack);
  AssertEquals('failed on 2', IntToHex(clGreen, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('#0000FF', clBlack);
  AssertEquals('failed on 3', IntToHex(clBlue, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('A0000FF', clBlack);
  AssertEquals('failed on 4', IntToHex(clBlack, 8), IntToHex(c, 8));
end;

procedure TTestReportMemo.TestHTMLColorToReportColor_length6;
var
  c: TFPReportColor;
begin
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('FF0000', clBlack);
  AssertEquals('failed on 1', IntToHex(clRed, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('008000', clBlack);
  AssertEquals('failed on 2', IntToHex(clGreen, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('0000FF', clBlack);
  AssertEquals('failed on 3', IntToHex(clBlue, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('A0000FF', clBlack);
  AssertEquals('failed on 4', IntToHex(clBlack, 8), IntToHex(c, 8));
end;

procedure TTestReportMemo.TestHTMLColorToReportColor_length3;
var
  c: TFPReportColor;
begin
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('F00', clBlack);
  AssertEquals('failed on 1', IntToHex(clRed, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('080', clBlack);
  AssertEquals('failed on 2', IntToHex($008800, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('00F', clBlack);
  AssertEquals('failed on 3', IntToHex(clBlue, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('A00F', clDkGray);
  AssertEquals('failed on 4', IntToHex(clDkGray, 8), IntToHex(c, 8));

  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('700', clBlack);
  AssertEquals('failed on 5', IntToHex($770000, 8), IntToHex(c, 8));
  c := TMemoFriend(FMemo).HtmlColorToFPReportColor('006', clBlack);
  AssertEquals('failed on 6', IntToHex($000066, 8), IntToHex(c, 8));
end;

procedure TTestReportMemo.TestCreateTestBlock;
var
  tb: TFPTextBlock;
begin
  tb := TMemoFriend(FMemo).CreateTextBlock(false);
  try
    AssertTrue('failed on 1', tb is TFPTextBlock);
    AssertFalse('failed on 2', tb is TFPHTTPTextBlock);
  finally
    tb.Free;
  end;
end;

procedure TTestReportMemo.TestCreateTestBlock_IsURL;
var
  tb: TFPTextBlock;
begin
  tb := TMemoFriend(FMemo).CreateTextBlock(true);
  try
    AssertTrue('failed on 1', tb is TFPTextBlock);
    AssertTrue('failed on 2', tb is TFPHTTPTextBlock);
  finally
    tb.Free;
  end;
end;

procedure TTestReportMemo.TestSubStr;
var
  m: TMemoFriend;
  lStartPos: integer;
begin
  m := TMemoFriend(FMemo);
  AssertEquals('failed on 1', '', m.SubStr('','','', 1, lStartPos));
  AssertEquals('failed on 1.1', -1, lStartPos);
  AssertEquals('failed on 2', 'abc', m.SubStr('xxxabcyyy','xxx','yyy', 1, lStartPos));
  AssertEquals('failed on 2.1', 4, lStartPos);
  AssertEquals('failed on 3', 'abc', m.SubStr('xxx,abc;xxx',',',';', 1, lStartPos));
  AssertEquals('failed on 3.1', 5, lStartPos);
  AssertEquals('failed on 4', 'abc', m.SubStr('<d>abc</d>','<d>','</d>', 1, lStartPos));
  AssertEquals('failed on 4.1', 4, lStartPos);
  AssertEquals('failed on 5', 'abc1', m.SubStr('<d>abc1</d> <d>abc2</d>','<d>','</d>', 1, lStartPos));
  AssertEquals('failed on 5.1', 4, lStartPos);
  AssertEquals('failed on 6', 'abc2', m.SubStr('<d>abc1</d> <d>abc2</d>','<d>','</d>', 2, lStartPos));
  AssertEquals('failed on 6.1', 16, lStartPos);
  AssertEquals('failed on 7', '', m.SubStr('<d>abc1</d> <d>abc2</d>','<d>','</d>', 3, lStartPos));
  AssertEquals('failed on 7.1', -1, lStartPos);
  AssertEquals('failed on 8', 'abc1', m.SubStr('<d>abc1</d> <d>abc2</d>','<d>','</d>', 0, lStartPos));
  AssertEquals('failed on 8.1', 4, lStartPos);
  AssertEquals('failed on 9', 'abc1', m.SubStr('<d>abc1</d> <d>abc2</d>','<d>','</d>', -1, lStartPos));
  AssertEquals('failed on 9.1', 4, lStartPos);
end;

procedure TTestReportMemo.TestTokenCount;
var
  m: TMemoFriend;
  lStartPos: integer;
begin
  m := TMemoFriend(FMemo);
  AssertEquals('failed on 1', '', m.SubStr('','','', 1, lStartPos));
  AssertEquals('failed on 1.1', -1, lStartPos);
  AssertEquals('failed on 2', 'abc', m.SubStr('xxxabcyyy','xxx','yyy', 1, lStartPos));
  AssertEquals('failed on 2.1', 4, lStartPos);

  AssertEquals('failed on 1', m.TokenCount('', ','), 0);
  AssertEquals('failed on 2', m.TokenCount('adf adf', ','), 1);
  AssertEquals('failed on 3', m.TokenCount('adf,', ','), 2);
  AssertEquals('failed on 4', m.TokenCount('adf,adf', ','), 2);
  AssertEquals('failed on 5', m.TokenCount('adf,adf,adf', ','), 3);
  AssertEquals('failed on 6', m.TokenCount('adf,adf,adf,', ','), 4);
  AssertEquals('failed on 6', m.TokenCount('0mm margin top and bottom.', ' '), 5);
  AssertEquals('failed on 6', m.TokenCount('0mm margin top and bottom. ', ' '), 6);
end;

procedure TTestReportMemo.TestToken;
var
  m: TMemoFriend;
  lStartPos: integer;
begin
  m := TMemoFriend(FMemo);
  AssertEquals('failed on 1', m.Token('', ',', 1), '');
  AssertEquals('failed on 2', m.Token('a,b,c', ',', 1), 'a');
  AssertEquals('failed on 3', m.Token('a,b,c', ',', 2), 'b');
  AssertEquals('failed on 4', m.Token('a,b,c', ',', 3), 'c');
  AssertEquals('failed on 5', m.Token('a,b,c', ',', 4), '');
  AssertEquals('failed on 6', m.Token('aa,bb,cc', ',', 1), 'aa');
  AssertEquals('failed on 7', m.Token('aa,bb,cc', ',', 2), 'bb');
  AssertEquals('failed on 8', m.Token('aa,bb,cc', ',', 3), 'cc');
  AssertEquals('failed on 9', m.Token('aa,bb,cc', ',', 4), '');
  AssertEquals('failed on 10', m.Token('aa,bb,cc,', ',', 4), '');
  AssertEquals('failed on 11', m.Token('0mm margin top and bottom.', ' ', 5), 'bottom.');
  AssertEquals('failed on 12', m.Token('0mm margin top and bottom. ', ' ', 5), 'bottom.');
  AssertEquals('failed on 13', m.Token('0mm margin top and bottom. ', ' ', 6), '');
end;

{ TTestBandList }

procedure TTestBandList.CreateBands;
begin
  b1 := TFPReportPageHeaderBand.Create(nil);
  b2 := TFPReportTitleBand.Create(nil);
  b3 := TFPReportDataBand.Create(nil);
end;

procedure TTestBandList.AddAllBandsToList;
begin
  FList.Add(b1);
  FList.Add(b2);
  FList.Add(b3);
end;

procedure TTestBandList.SetUp;
begin
  inherited SetUp;
  FList := TBandList.Create;
  CreateBands;
end;

procedure TTestBandList.TearDown;
begin
  FreeAndNil(FList);
  FreeAndNil(b3);
  FreeAndNil(b2);
  FreeAndNil(b1);
  inherited TearDown;
end;

procedure TTestBandList.TestAdd;
begin
  AssertEquals('Failed on 1', 0, FList.Count);
  AddAllBandsToList;
  AssertEquals('Failed on 2', 3, FList.Count);
end;

procedure TTestBandList.TestItems;
begin
  AssertEquals('Failed on 1', 0, FList.Count);
  AddAllBandsToList;
  AssertEquals('Failed on 2', 3, FList.Count);

  AssertTrue('failed on 3', FList.Items[0] = b1);
  AssertTrue('failed on 4', FList.Items[1] = b2);
  AssertTrue('failed on 5', FList.Items[1] <> b1);
  AssertTrue('failed on 6', FList.Items[2] = b3);
end;

procedure TTestBandList.TestClear;
begin
  AssertEquals('Failed on 1', 0, FList.Count);
  AddAllBandsToList;
  AssertEquals('Failed on 2', 3, FList.Count);

  FList.Clear;
  AssertEquals('Failed on 3', 0, FList.Count);
  AssertTrue('failed on 4', b1 <> nil); // List.Clear shouldn't free bands
end;

procedure TTestBandList.TestDelete;
begin
  AssertEquals('Failed on 1', 0, FList.Count);
  AddAllBandsToList;
  AssertEquals('Failed on 2', 3, FList.Count);

  FList.Delete(0);
  AssertEquals('Failed on 3', 2, FList.Count);
  AssertTrue('failed on 4', b1 <> nil); // List.Delete shouldn't free bands
  AssertTrue('failed on 5', FList.Items[0] = b2);
  AssertTrue('failed on 6', FList.Items[1] = b3);
end;

procedure TTestBandList.TestFind1;
var
  lBand: TFPReportCustomBand;
  lResult: integer;
begin
  AssertEquals('Failed on 1', 0, FList.Count);
  AddAllBandsToList;
  AssertEquals('Failed on 2', 3, FList.Count);

  AssertTrue('failed on 3', FList.Find(TFPReportPageHeaderBand) <> nil);
  AssertTrue('failed on 4', FList.Find(TFPReportPageHeaderBand) = b1);
  AssertTrue('failed on 5', FList.Find(TFPReportTitleBand) = b2);
  AssertTrue('failed on 6', FList.Find(TFPReportDataBand) = b3);

  FList.Clear;
  AssertTrue('failed on 7', FList.Find(TFPReportTitleBand) = nil);
end;

procedure TTestBandList.TestFind2;
var
  lBand: TFPReportCustomBand;
  lResult: integer;
begin
  AssertEquals('Failed on 1', 0, FList.Count);
  lResult := FList.Find(TFPReportPageHeaderBand, lBand);
  AssertEquals('failed on 2', -1, lResult);
  AssertTrue('failed on 3', lBand = nil);

  AddAllBandsToList;
  AssertEquals('Failed on 4', 3, FList.Count);

  lResult := FList.Find(TFPReportPageHeaderBand, lBand);
  AssertEquals('failed on 5', 0, lResult);
  AssertTrue('failed on 6', lBand <> nil);
  AssertTrue('failed on 7', lBand = b1);

  lResult := FList.Find(TFPReportTitleBand, lBand);
  AssertEquals('failed on 8', 1, lResult);
  AssertTrue('failed on 9', lBand = b2);

  lResult := FList.Find(TFPReportDataBand, lBand);
  AssertEquals('failed on 10', 2, lResult);
  AssertTrue('failed on 11', lBand = b3);

  FList.Clear;
  lResult := FList.Find(TFPReportTitleBand, lBand);
  AssertTrue('failed on 12', lBand = nil);
  AssertTrue('failed on 13', lResult = -1);
end;

initialization
  RegisterTests(
    [TTestReportComponent,
    TReportElementTest,
    TTestReportChildren,
    TTestReportFrame,
    TTestReportLayout,
    TTestFPPageSize,
    TTestFPPaperManager,
    TTestFPReportPageSize,
    TTestReportPage,
    TTestReportData,
    TTestUserReportData,
    TTestUserReportData2,
    TTestDataBand,
    TTestCustomReport,
    TTestReportMemo,
    TTestBandList,
    TTestVariable,
    TTestVariables
    ]);
end.


