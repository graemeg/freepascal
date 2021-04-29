type
  { TSInt64Bit1Test }

  TSInt64Bit1Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit1ModTest }

  TSInt64Bit1ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit10Test }

  TSInt64Bit10Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit10ModTest }

  TSInt64Bit10ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit18Test }

  TSInt64Bit18Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit18ModTest }

  TSInt64Bit18ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit24Test }

  TSInt64Bit24Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit24ModTest }

  TSInt64Bit24ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit100Test }

  TSInt64Bit100Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit100ModTest }

  TSInt64Bit100ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit153Test }
const
  FS64_153Input: array[$0..$F] of Int64 =
    (0, 1, 152, 153, 154, -1, -152, -153, -154,
    8000000000000000117, 8000000000000000118, 8000000000000000119, 
    -8000000000000000117, -8000000000000000118, -8000000000000000119,
    Int64($8000000000000000));

type
  TSInt64Bit153Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit153ModTest }

  TSInt64Bit153ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit1461Test }
const
  FS64_1461Input: array[$0..$F] of Int64 =
    (0, 1, 1460, 1461, 1462, -1, -1460, -1461, -1462,
    8000000000000000582, 8000000000000000583, 8000000000000000584, 
    -8000000000000000582, -8000000000000000583, -8000000000000000584,
    Int64($8000000000000000));

type
  TSInt64Bit1461Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit1461ModTest }

  TSInt64Bit1461ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit10000Test }
const
  FS64_10000Input: array[$0..$F] of Int64 =
    (0, 1, 9999, 10000, 10001, -1, -9999, -10000, -10001,
    7999999999999999999, 8000000000000000000, 8000000000000000001, 
    -7999999999999999999, -8000000000000000000, -8000000000000000001,
    Int64($8000000000000000));

type
  TSInt64Bit10000Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit10000ModTest }

  TSInt64Bit10000ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;
  
  { TSInt64Bit86400000Test }
const
  FS64_86400000Input: array[$0..$F] of Int64 =
    (0, 1, 86399999, 86400000, 86400001, -1, -86399999, -86400000, -86400001,
    8639999999999999999, 8640000000000000000, 8640000000000000001, 
    -8639999999999999999, -8640000000000000000, -8640000000000000001,
    Int64($8000000000000000));

type
  TSInt64Bit86400000Test = class(TSInt64DivTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt64Bit86400000ModTest }

  TSInt64Bit86400000ModTest = class(TSInt64ModTest)
    protected
      function GetDivisor: Int64; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

{ TSInt64Bit1Test }

function TSInt64Bit1Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 1';
  end;

function TSInt64Bit1Test.GetDivisor: Int64;
  begin
    Result := 1;
  end;

procedure TSInt64Bit1Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := Int64($8000000000000000);
      1:
        Numerator := Int64($8000000000000006);
      2:
        Numerator := Int64($8000000000000007);
      253:
        Numerator := Int64($7FFFFFFFFFFFFFF9);
      254:
        Numerator := Int64($7FFFFFFFFFFFFFFA);
      255:
        Numerator := Int64($7FFFFFFFFFFFFFFF);
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 1;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit1ModTest }

function TSInt64Bit1ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 1';
  end;

function TSInt64Bit1ModTest.GetDivisor: Int64;
  begin
    Result := 1;
  end;

procedure TSInt64Bit1ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := Int64($8000000000000000);
      1:
        Numerator := Int64($8000000000000006);
      2:
        Numerator := Int64($8000000000000007);
      253:
        Numerator := Int64($7FFFFFFFFFFFFFF9);
      254:
        Numerator := Int64($7FFFFFFFFFFFFFFA);
      255:
        Numerator := Int64($7FFFFFFFFFFFFFFF);
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 1;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit10Test }

function TSInt64Bit10Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 10';
  end;

function TSInt64Bit10Test.GetDivisor: Int64;
  begin
    Result := 10;
  end;

procedure TSInt64Bit10Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := Int64($8000000000000000);
      1:
        Numerator := Int64($8000000000000006);
      2:
        Numerator := Int64($8000000000000007);
      253:
        Numerator := Int64($7FFFFFFFFFFFFFF9);
      254:
        Numerator := Int64($7FFFFFFFFFFFFFFA);
      255:
        Numerator := Int64($7FFFFFFFFFFFFFFF);
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 10;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit10ModTest }

function TSInt64Bit10ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 10';
  end;

function TSInt64Bit10ModTest.GetDivisor: Int64;
  begin
    Result := 10;
  end;

procedure TSInt64Bit10ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := Int64($8000000000000000);
      1:
        Numerator := Int64($8000000000000006);
      2:
        Numerator := Int64($8000000000000007);
      253:
        Numerator := Int64($7FFFFFFFFFFFFFF9);
      254:
        Numerator := Int64($7FFFFFFFFFFFFFFA);
      255:
        Numerator := Int64($7FFFFFFFFFFFFFFF);
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 10;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit18Test }

function TSInt64Bit18Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 18';
  end;

function TSInt64Bit18Test.GetDivisor: Int64;
  begin
    Result := 18;
  end;

procedure TSInt64Bit18Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := Index - 128;
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 18;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit18ModTest }

function TSInt64Bit18ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 18';
  end;

function TSInt64Bit18ModTest.GetDivisor: Int64;
  begin
    Result := 18;
  end;

procedure TSInt64Bit18ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := Index - 128;
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 18;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit24Test }

function TSInt64Bit24Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 24';
  end;

function TSInt64Bit24Test.GetDivisor: Int64;
  begin
    Result := 24;
  end;

procedure TSInt64Bit24Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := Index - 128;
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 24;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit24ModTest }

function TSInt64Bit24ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 24';
  end;

function TSInt64Bit24ModTest.GetDivisor: Int64;
  begin
    Result := 24;
  end;

procedure TSInt64Bit24ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := Index - 128;
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 24;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit100Test }

function TSInt64Bit100Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 100';
  end;

function TSInt64Bit100Test.GetDivisor: Int64;
  begin
    Result := 100;
  end;

procedure TSInt64Bit100Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := Int64($8000000000000000);
      1:
        Numerator := Int64($8000000000000008);
      2:
        Numerator := Int64($8000000000000009);
      253:
        Numerator := Int64($7FFFFFFFFFFFFFF7);
      254:
        Numerator := Int64($7FFFFFFFFFFFFFF8);
      255:
        Numerator := Int64($7FFFFFFFFFFFFFFF);
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 100;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit100ModTest }

function TSInt64Bit100ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 100';
  end;

function TSInt64Bit100ModTest.GetDivisor: Int64;
  begin
    Result := 100;
  end;

procedure TSInt64Bit100ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := Int64($8000000000000000);
      1:
        Numerator := Int64($8000000000000008);
      2:
        Numerator := Int64($8000000000000009);
      253:
        Numerator := Int64($7FFFFFFFFFFFFFF7);
      254:
        Numerator := Int64($7FFFFFFFFFFFFFF8);
      255:
        Numerator := Int64($7FFFFFFFFFFFFFFF);
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 100;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit153Test }

function TSInt64Bit153Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 153';
  end;

function TSInt64Bit153Test.GetDivisor: Int64;
  begin
    Result := 153;
  end;

procedure TSInt64Bit153Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_153Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 153;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit153ModTest }

function TSInt64Bit153ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 153';
  end;

function TSInt64Bit153ModTest.GetDivisor: Int64;
  begin
    Result := 153;
  end;

procedure TSInt64Bit153ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_153Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 153;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit1461Test }

function TSInt64Bit1461Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 1,461';
  end;

function TSInt64Bit1461Test.GetDivisor: Int64;
  begin
    Result := 1461;
  end;

procedure TSInt64Bit1461Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_1461Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 1461;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit1461ModTest }

function TSInt64Bit1461ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 1,461';
  end;

function TSInt64Bit1461ModTest.GetDivisor: Int64;
  begin
    Result := 1461;
  end;

procedure TSInt64Bit1461ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_1461Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 1461;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit10000Test }

function TSInt64Bit10000Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 10,000 (Currency)';
  end;

function TSInt64Bit10000Test.GetDivisor: Int64;
  begin
    Result := 10000;
  end;

procedure TSInt64Bit10000Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_10000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 10000;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit10000ModTest }

function TSInt64Bit10000ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 10,000 (Currency)';
  end;

function TSInt64Bit10000ModTest.GetDivisor: Int64;
  begin
    Result := 10000;
  end;

procedure TSInt64Bit10000ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_10000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 10000;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit86400000Test }

function TSInt64Bit86400000Test.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit division by 86,400,000';
  end;

function TSInt64Bit86400000Test.GetDivisor: Int64;
  begin
    Result := 86400000;
  end;

procedure TSInt64Bit86400000Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_86400000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 86400000;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt64Bit86400000ModTest }

function TSInt64Bit86400000ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 64-bit modulus by 86,400,000';
  end;

function TSInt64Bit86400000ModTest.GetDivisor: Int64;
  begin
    Result := 86400000;
  end;

procedure TSInt64Bit86400000ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Int64;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FS64_86400000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 86400000;
      
    FResultArray[Index] := Answer;
  end;
