{ %OPT=-O2 }
program bdiv;

{$mode objfpc}{$H+}

uses
  SysUtils;

{ Utility functions }
function GetRealTime(const st: TSystemTime): Real;
  begin
    Result := st.Hour*3600.0 + st.Minute*60.0 + st.Second + st.MilliSecond/1000.0;
  end;

{$push}
{$warn 5057 off}
function GetRealTime : Real;
  var
    st:TSystemTime;
  begin
    GetLocalTime(st);
    result:=GetRealTime(st);
  end;
{$pop}

const
  ITERATIONS = 524288;
  INTERNAL_LOOPS = 64;

{ TTestAncestor }
type
  TTestAncestor = class
    private
      FStartTime: Real;
      FEndTime: Real;
      FAvgTime: Real;
      procedure SetStartTime;
      procedure SetEndTime;
    protected
      procedure DoTestIteration(Iteration: Integer); virtual; abstract;
    public
      constructor Create; virtual;
      destructor Destroy; override;
      procedure Run;
      function TestTitle: shortstring; virtual; abstract;
      function WriteResults: Boolean; virtual; abstract;
      property RunTime: Real read FAvgTime;
  end;

  TTestClass = class of TTestAncestor;

  TUInt32DivTest = class(TTestAncestor)
    protected
      FInputArray: array[$00..$FF] of Cardinal;
      FResultArray: array[$00..$FF] of Cardinal;
      function GetDivisor: Cardinal; virtual; abstract;
      function DoVariableDiv(Numerator: Cardinal): Cardinal; inline;
    public
      function WriteResults: Boolean; override;
  end;

  TUInt32ModTest = class(TUInt32DivTest)
    protected
      function DoVariableMod(Numerator: Cardinal): Cardinal; inline;
    public
      function WriteResults: Boolean; override;
  end;

  TSInt32DivTest = class(TTestAncestor)
    protected
      FInputArray: array[$00..$FF] of Integer;
      FResultArray: array[$00..$FF] of Integer;
      function GetDivisor: Integer; virtual; abstract;
      function DoVariableDiv(Numerator: Integer): Integer; inline;
    public
      function WriteResults: Boolean; override;
  end;

  TSInt32ModTest = class(TSInt32DivTest)
    protected
      function DoVariableMod(Numerator: Integer): Integer; inline;
    public
      function WriteResults: Boolean; override;
  end;

  TUInt64DivTest = class(TTestAncestor)
    protected
      FInputArray: array[$00..$FF] of QWord;
      FResultArray: array[$00..$FF] of QWord;
      function GetDivisor: QWord; virtual; abstract;
      function DoVariableDiv(Numerator: QWord): QWord; inline;
    public
      function WriteResults: Boolean; override;
  end;

  TUInt64ModTest = class(TUInt64DivTest)
    protected
      function DoVariableMod(Numerator: QWord): QWord; inline;
    public
      function WriteResults: Boolean; override;
  end;

  TSInt64DivTest = class(TTestAncestor)
    protected
      FInputArray: array[$00..$FF] of Int64;
      FResultArray: array[$00..$FF] of Int64;
      function GetDivisor: Int64; virtual; abstract;
      function DoVariableDiv(Numerator: Int64): Int64; inline;
    public
      function WriteResults: Boolean; override;
  end;

  TSInt64ModTest = class(TSInt64DivTest)
    protected
      function DoVariableMod(Numerator: Int64): Int64; inline;
    public
      function WriteResults: Boolean; override;
  end;

{$I bdiv_u32.inc}
{$I bdiv_u64.inc}
{$I bdiv_s32.inc}
{$I bdiv_s64.inc}

{ TTestAncestor }

constructor TTestAncestor.Create;
  begin
    FStartTime := 0;
    FEndTime := 0;
    FAvgTime := 0;
  end;

destructor TTestAncestor.Destroy;
  begin
    inherited Destroy;
  end;

procedure TTestAncestor.SetStartTime;
  begin
    FStartTime := GetRealTime();
  end;

procedure TTestAncestor.SetEndTime;
  begin
    FEndTime := GetRealTime();
    if FEndTime < FStartTime then { Happens if the test runs past midnight }
      FEndTime := FEndTime + 86400.0;
  end;

procedure TTestAncestor.Run;
  var
    X: Integer;
  begin
    SetStartTime;
    for X := 0 to ITERATIONS - 1 do
      DoTestIteration(X);

    SetEndTime;

    FAvgTime := FEndTime - FStartTime;
  end;

{ TUInt32DivTest }

function TUInt32DivTest.DoVariableDiv(Numerator: Cardinal): Cardinal;
  begin
    Result := Numerator div GetDivisor;
  end;

function TUInt32DivTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: Cardinal;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableDiv(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' div ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ TUInt32ModTest }

function TUInt32ModTest.DoVariableMod(Numerator: Cardinal): Cardinal;
  begin
    Result := Numerator mod GetDivisor;
  end;

function TUInt32ModTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: Cardinal;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableMod(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' mod ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ TSInt32DivTest }

function TSInt32DivTest.DoVariableDiv(Numerator: Integer): Integer;
  begin
    Result := Numerator div GetDivisor;
  end;

function TSInt32DivTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: Integer;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableDiv(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' div ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ TSInt32ModTest }

function TSInt32ModTest.DoVariableMod(Numerator: Integer): Integer;
  begin
    Result := Numerator mod GetDivisor;
  end;

function TSInt32ModTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: Integer;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableMod(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' mod ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ TUInt64DivTest }

function TUInt64DivTest.DoVariableDiv(Numerator: QWord): QWord;
  begin
    Result := Numerator div GetDivisor;
  end;

function TUInt64DivTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: QWord;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableDiv(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' div ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ TUInt64ModTest }

function TUInt64ModTest.DoVariableMod(Numerator: QWord): QWord;
  begin
    Result := Numerator mod GetDivisor;
  end;

function TUInt64ModTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: QWord;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableMod(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' mod ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ TSInt64DivTest }

function TSInt64DivTest.DoVariableDiv(Numerator: Int64): Int64;
  begin
    Result := Numerator div GetDivisor;
  end;

function TSInt64DivTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: Int64;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableDiv(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' div ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ TSInt64ModTest }

function TSInt64ModTest.DoVariableMod(Numerator: Int64): Int64;
  begin
    Result := Numerator mod GetDivisor;
  end;

function TSInt64ModTest.WriteResults: Boolean;
  var
    X: Integer;
    Expected: Int64;
  begin
    Result := True;
    for X := 0 to 255 do
      begin
        Expected := DoVariableMod(FInputArray[X]);
        if FResultArray[X] <> Expected then
          begin
            WriteLn('FAIL - ', FInputArray[X], ' mod ', GetDivisor, '; expected ', Expected, ' got ', FResultArray[X]);
            Result := False;
            Exit;
          end;
      end;
  end;

{ Main function }
const
  TestClasses: array[0..53] of TTestClass = (
    TUInt32Bit1Test,
    TUInt32Bit1ModTest,
    TUInt32Bit2Test,
    TUInt32Bit2ModTest,
    TUInt32Bit3Test,
    TUInt32Bit3ModTest,
    TUInt32Bit10Test,
    TUInt32Bit10ModTest,
    TUInt32Bit100Test,
    TUInt32Bit100ModTest,
    TUInt32Bit1000Test,
    TUInt32Bit1000ModTest,
    TUInt32Bit60000Test,
    TUInt32Bit60000ModTest,
    TUInt32Bit146097Test,
    TUInt32Bit146097ModTest,
    TUInt32Bit3600000Test,
    TUInt32Bit3600000ModTest,
    TUInt64Bit1Test,
    TUInt64Bit1ModTest,
    TUInt64Bit2Test,
    TUInt64Bit2ModTest,
    TUInt64Bit3Test,
    TUInt64Bit3ModTest,
    TUInt64Bit5Test,
    TUInt64Bit5ModTest,
    TUInt64Bit10Test,
    TUInt64Bit10ModTest,
    TUInt64Bit100Test,
    TUInt64Bit100ModTest,
    TUInt64Bit1000000000Test,
    TUInt64Bit1000000000ModTest,
    TSInt32Bit1Test,
    TSInt32Bit1ModTest,
    TSInt32Bit100Test,
    TSInt32Bit100ModTest,
    TSInt64Bit1Test,
    TSInt64Bit1ModTest,
    TSInt64Bit10Test,
    TSInt64Bit10ModTest,
    TSInt64Bit18Test,
    TSInt64Bit18ModTest,
    TSInt64Bit24Test,
    TSInt64Bit24ModTest,
    TSInt64Bit100Test,
    TSInt64Bit100ModTest,
    TSInt64Bit153Test,
    TSInt64Bit153ModTest,
    TSInt64Bit1461Test,
    TSInt64Bit1461ModTest,
    TSInt64Bit10000Test,
    TSInt64Bit10000ModTest,
    TSInt64Bit86400000Test,
    TSInt64Bit86400000ModTest
  );

var
  CurrentObject: TTestAncestor;
  Failed: Boolean;
  X: Integer;
  SummedUpAverageDuration, AverageDuration : Double;
begin
  SummedUpAverageDuration := 0.0;
  Failed := False;
  WriteLn('Division compilation and timing test (using constants from System and Sysutils)');
  WriteLn('-------------------------------------------------------------------------------');
  for X := Low(TestClasses) to High(TestClasses) do
    begin
      try
        CurrentObject := TestClasses[X].Create;
        try
          Write(CurrentObject.TestTitle:43, ' - ');
          CurrentObject.Run;

          if CurrentObject.WriteResults then
            begin
              AverageDuration := ((CurrentObject.RunTime * 1000000000.0) / (ITERATIONS * INTERNAL_LOOPS));
              WriteLn('Pass - average iteration duration: ', AverageDuration:1:3, ' ns');
              SummedUpAverageDuration := SummedUpAverageDuration + AverageDuration;
            end
          else
            { Final average isn't processed if a test failed, so there's no need
              to calculate and add the average duration to it }
            Failed := True;

        finally
          CurrentObject.Free;
        end;
      except on E: Exception do
        begin
          WriteLn('Exception "', E.ClassName, '" raised while running test object of class "', TestClasses[X].ClassName, '"');
          Failed := True;
        end;
      end;
    end;

  if Failed then
    Halt(1);

  WriteLn(#10'ok');
  WriteLn('- Sum of average durations: ', SummedUpAverageDuration:1:3, ' ns');
  WriteLn('- Overall average duration: ', (SummedUpAverageDuration / Length(TestClasses)):1:3, ' ns');
end.
