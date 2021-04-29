type
  { TUInt32Bit1Test }

  TUInt32Bit1Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit1ModTest }

  TUInt32Bit1ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit2Test }

  TUInt32Bit2Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit2ModTest }

  TUInt32Bit2ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit3Test }

  TUInt32Bit3Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit3ModTest }

  TUInt32Bit3ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit10Test }

  TUInt32Bit10Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit10ModTest }

  TUInt32Bit10ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit100Test }

  TUInt32Bit100Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit100ModTest }

  TUInt32Bit100ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit1000Test }
const
  FU32_1000Input: array[$0..$F] of Cardinal =
    (0, 1, 999, 1000, 1001, 1999, 2000, 2001,
    4294958999, 4294959000, 4294959001,
    $7FFFFFFE, $7FFFFFFF, $80000000, $80000001, $FFFFFFFF);

type
  TUInt32Bit1000Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit1000ModTest }

  TUInt32Bit1000ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit60000Test }
const
  FU32_60000Input: array[$0..$F] of Cardinal =
    (0, 1, 59999, 60000, 60001, 119999, 120000, 120001,
    4294919999, 4294920000, 4294920001,
    $7FFFFFFE, $7FFFFFFF, $80000000, $80000001, $FFFFFFFF);

type
  TUInt32Bit60000Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit60000ModTest }

  TUInt32Bit60000ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit146097Test }
const
  FU32_146097Input: array[$0..$F] of Cardinal =
    (0, 1, 146096, 146097, 146098, 292193, 292194, 292195,
    4294959605, 4294959606, 4294959607,    
    $7FFFFFFE, $7FFFFFFF, $80000000, $80000001, $FFFFFFFF);

type
  TUInt32Bit146097Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit146097ModTest }

  TUInt32Bit146097ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;
  
  { TUInt32Bit3600000Test }
const
  FU32_3600000Input: array[$0..$F] of Cardinal =
    (0, 1, 3599999, 3600000, 3600001, 7199999, 7200000, 7200001,
    3600000000, 4294799999, 4294800000, 4294800001,
    $7FFFFFFF, $80000000, $80000001, $FFFFFFFF);

type
  TUInt32Bit3600000Test = class(TUInt32DivTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt32Bit3600000ModTest }

  TUInt32Bit3600000ModTest = class(TUInt32ModTest)
    protected
      function GetDivisor: Cardinal; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;


{ TUInt32Bit1Test }

function TUInt32Bit1Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 1';
  end;

function TUInt32Bit1Test.GetDivisor: Cardinal;
  begin
    Result := 1;
  end;

procedure TUInt32Bit1Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967293;
      254:
        Numerator := 4294967294;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 1;

    FResultArray[Index] := Answer;
  end;
  
{ TUInt32Bit1Test }

function TUInt32Bit1ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 1';
  end;

function TUInt32Bit1ModTest.GetDivisor: Cardinal;
  begin
    Result := 1;
  end;

procedure TUInt32Bit1ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967293;
      254:
        Numerator := 4294967294;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 1;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit2Test }

function TUInt32Bit2Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 2';
  end;

function TUInt32Bit2Test.GetDivisor: Cardinal;
  begin
    Result := 2;
  end;

procedure TUInt32Bit2Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967293;
      254:
        Numerator := 4294967294;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 2;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit2ModTest }

function TUInt32Bit2ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 2';
  end;

function TUInt32Bit2ModTest.GetDivisor: Cardinal;
  begin
    Result := 2;
  end;

procedure TUInt32Bit2ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967293;
      254:
        Numerator := 4294967294;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 2;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit3Test }

function TUInt32Bit3Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 3';
  end;

function TUInt32Bit3Test.GetDivisor: Cardinal;
  begin
    Result := 3;
  end;

procedure TUInt32Bit3Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      254:
        Numerator := 4294967294;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 3;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit3ModTest }

function TUInt32Bit3ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 3';
  end;

function TUInt32Bit3ModTest.GetDivisor: Cardinal;
  begin
    Result := 3;
  end;

procedure TUInt32Bit3ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      254:
        Numerator := 4294967294;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 3;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit10Test }

function TUInt32Bit10Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 10';
  end;

function TUInt32Bit10Test.GetDivisor: Cardinal;
  begin
    Result := 10;
  end;

procedure TUInt32Bit10Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967289;
      254:
        Numerator := 4294967290;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 10;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit10ModTest }

function TUInt32Bit10ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 10';
  end;

function TUInt32Bit10ModTest.GetDivisor: Cardinal;
  begin
    Result := 10;
  end;

procedure TUInt32Bit10ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967289;
      254:
        Numerator := 4294967290;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 10;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit100Test }

function TUInt32Bit100Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 100';
  end;

function TUInt32Bit100Test.GetDivisor: Cardinal;
  begin
    Result := 100;
  end;

procedure TUInt32Bit100Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967199;
      254:
        Numerator := 4294967200;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 100;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit100ModTest }

function TUInt32Bit100ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 100';
  end;

function TUInt32Bit100ModTest.GetDivisor: Cardinal;
  begin
    Result := 100;
  end;

procedure TUInt32Bit100ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := 4294967199;
      254:
        Numerator := 4294967200;
      255:
        Numerator := 4294967295;
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 100;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit1000Test }

function TUInt32Bit1000Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 1,000';
  end;

function TUInt32Bit1000Test.GetDivisor: Cardinal;
  begin
    Result := 1000;
  end;

procedure TUInt32Bit1000Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_1000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 1000;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit1000ModTest }

function TUInt32Bit1000ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 1,000';
  end;

function TUInt32Bit1000ModTest.GetDivisor: Cardinal;
  begin
    Result := 1000;
  end;

procedure TUInt32Bit1000ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_1000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 1000;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit60000Test }

function TUInt32Bit60000Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 60,000';
  end;

function TUInt32Bit60000Test.GetDivisor: Cardinal;
  begin
    Result := 60000;
  end;

procedure TUInt32Bit60000Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_60000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 60000;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit60000ModTest }

function TUInt32Bit60000ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 60,000';
  end;

function TUInt32Bit60000ModTest.GetDivisor: Cardinal;
  begin
    Result := 60000;
  end;

procedure TUInt32Bit60000ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_60000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 60000;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit146097Test }

function TUInt32Bit146097Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 146,097';
  end;

function TUInt32Bit146097Test.GetDivisor: Cardinal;
  begin
    Result := 146097;
  end;

procedure TUInt32Bit146097Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_146097Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 146097;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit146097ModTest }

function TUInt32Bit146097ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 146,097';
  end;

function TUInt32Bit146097ModTest.GetDivisor: Cardinal;
  begin
    Result := 146097;
  end;

procedure TUInt32Bit146097ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_146097Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 146097;

    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit3600000Test }

function TUInt32Bit3600000Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit division by 3,600,000';
  end;

function TUInt32Bit3600000Test.GetDivisor: Cardinal;
  begin
    Result := 3600000;
  end;

procedure TUInt32Bit3600000Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_3600000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 3600000;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt32Bit3600000ModTest }

function TUInt32Bit3600000ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 32-bit modulus by 3,600,000';
  end;

function TUInt32Bit3600000ModTest.GetDivisor: Cardinal;
  begin
    Result := 3600000;
  end;

procedure TUInt32Bit3600000ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Cardinal;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU32_3600000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 3600000;

    FResultArray[Index] := Answer;
  end;
