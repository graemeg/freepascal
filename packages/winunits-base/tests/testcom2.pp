{$ifdef FPC}
{$mode objfpc}
{$endif FPC}
program excel;

uses variants,Windows,activeX;

Const
  IID_IDISPATCH : TGUID = '{00020400-0000-0000-C000-000000000046}';


Type
  tArguments = array[0..63] of variant;

  ExcelRange = dispinterface ['{00020846-0000-0000-C000-000000000046}']
    property Value: OleVariant dispid 6;
  end;

  WorksheetDisp = dispinterface ['{000208D8-0000-0000-C000-000000000046}']
     property Cells: ExcelRange readonly dispid 238;
  end;

  ExcelWorkbook = interface(IDispatch)
  end;

  WorkbooksDisp = dispinterface ['{000208DB-0000-0000-C000-000000000046}']
    function Add(Template: OleVariant; lcid: Integer): ExcelWorkbook; dispid 181;
  end;

  ExcelApplicationDisp = dispinterface ['{000208D5-0000-0000-C000-000000000046}']
    property ActiveSheet: IDispatch readonly dispid 307;
    property Workbooks: IDispatch readonly dispid 572;
    property Visible[lcid: Integer]: WordBool dispid 558;
  end;

Function CheckOle(Msg : string;hres : HResult) : HResult;

begin
  Result:=hres;
  if Failed(hres) then
    writeln(Msg,' error')
  else if hres=S_OK then
    writeln(Msg,' S_OK')
  else if hres=REGDB_E_CLASSNOTREG then
    writeln(Msg,'CLASSNOTREG')
  else if hres=CLASS_E_NOAGGREGATION then
    writeln(Msg,'NOAGGREGATION')
  else
    writeln(Msg,'other error:',longint(hres));
end;

Var
    hres      : HRESULT;
    aclsID    : TGUID;

    excelapp  : ExcelApplicationDisp;
    WorkBooks : WorkbooksDisp;
    ActiveSheet : WorksheetDisp;
    Cells     : ExcelRange;
    i, j      : longint;

begin
  hres := CheckOle('CoInit',CoInitializeEx(nil,COINIT_MULTITHREADED));
  hres := CheckOle('CLSIDFromProgID',CLSIDFromProgID('Excel.Application', aclsid));
  hres := CheckOle('CoCreate',CoCreateInstance(aclsid, Nil, {CLSCTX_INPROC_SERVER or }CLSCTX_LOCAL_SERVER, IID_IDispatch, excelApp));

  ExcelApp.Visible[0] := true;
  { Following should also be possible as ExcelApp.Workbooks.Add !!}
  WorkBooks := ExcelApp.WorkBooks as WorkBooksDisp;
  WorkBooks.Add(Null,0);
  {
    The following should also work as
      For I:=1 to 5 do
        For J:=1 to 5 do
          ExcelApp.ActiveSheet.Cells[i,j] := i+j;
   }
  ActiveSheet:=ExcelApp.ActiveSheet as WorksheetDisp;
  For I:=1 to 5 do
    for j:=1 to 5 do
      begin
//      Cells:=ActiveSheet.Cells[I,J];
//      Cells.Value:=I+J;
      end;
  // Free everything.
  Cells:=Nil;
  ActiveSheet:=Nil;
  WorkBooks:=Nil;
  excelApp:=Nil;
end.
