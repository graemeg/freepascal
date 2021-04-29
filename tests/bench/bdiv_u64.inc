type
  { TUInt64Bit1Test }

  TUInt64Bit1Test = class(TUInt64DivTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit1ModTest }

  TUInt64Bit1ModTest = class(TUInt64ModTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit2Test }

  TUInt64Bit2Test = class(TUInt64DivTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit2ModTest }

  TUInt64Bit2ModTest = class(TUInt64ModTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit3Test }

  TUInt64Bit3Test = class(TUInt64DivTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit3ModTest }

  TUInt64Bit3ModTest = class(TUInt64ModTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit5Test }

  TUInt64Bit5Test = class(TUInt64DivTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit5ModTest }

  TUInt64Bit5ModTest = class(TUInt64ModTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit10Test }

  TUInt64Bit10Test = class(TUInt64DivTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit10ModTest }

  TUInt64Bit10ModTest = class(TUInt64ModTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit100Test }

  TUInt64Bit100Test = class(TUInt64DivTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit100ModTest }

  TUInt64Bit100ModTest = class(TUInt64ModTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TUInt64Bit1000000000Test }
const
  FU64_1000000000Input: array[$0..$F] of QWord =
    (0, 1, 999999999, 1000000000, 1000000001, 5000000000,
    7999999999999999999, 8000000000000000000, 8000000000000000001,
    QWord(15999999999999999999), QWord(16000000000000000000), QWord(16000000000000000001),
    $7FFFFFFFFFFFFFFF, QWord($8000000000000000), QWord($8000000000000001), QWord($FFFFFFFFFFFFFFFF));

type
  TUInt64Bit1000000000Test = class(TUInt64DivTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  TUInt64Bit1000000000ModTest = class(TUInt64ModTest)
    protected
      function GetDivisor: QWord; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

{ TUInt64Bit1Test }

function TUInt64Bit1Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit division by 1';
  end;

function TUInt64Bit1Test.GetDivisor: QWord;
  begin
    Result := 1;
  end;

procedure TUInt64Bit1Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFFD);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 1;

    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit1ModTest }

function TUInt64Bit1ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit modulus by 1';
  end;

function TUInt64Bit1ModTest.GetDivisor: QWord;
  begin
    Result := 1;
  end;

procedure TUInt64Bit1ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFFD);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 1;

    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit2Test }

function TUInt64Bit2Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit division by 2';
  end;

function TUInt64Bit2Test.GetDivisor: QWord;
  begin
    Result := 2;
  end;

procedure TUInt64Bit2Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFFD);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 2;

    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit2ModTest }

function TUInt64Bit2ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit modulus by 2';
  end;

function TUInt64Bit2ModTest.GetDivisor: QWord;
  begin
    Result := 2;
  end;

procedure TUInt64Bit2ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFFD);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 2;

    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit3Test }

function TUInt64Bit3Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit division by 3';
  end;

function TUInt64Bit3Test.GetDivisor: QWord;
  begin
    Result := 3;
  end;

procedure TUInt64Bit3Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 3;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit3ModTest }

function TUInt64Bit3ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit modulus by 3';
  end;

function TUInt64Bit3ModTest.GetDivisor: QWord;
  begin
    Result := 3;
  end;

procedure TUInt64Bit3ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 3;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit5Test }

function TUInt64Bit5Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit division by 5';
  end;

function TUInt64Bit5Test.GetDivisor: QWord;
  begin
    Result := 5;
  end;

procedure TUInt64Bit5Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 5;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit5ModTest }

function TUInt64Bit5ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit modulus by 5';
  end;

function TUInt64Bit5ModTest.GetDivisor: QWord;
  begin
    Result := 5;
  end;

procedure TUInt64Bit5ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFE);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 5;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit10Test }

function TUInt64Bit10Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit division by 10';
  end;

function TUInt64Bit10Test.GetDivisor: QWord;
  begin
    Result := 10;
  end;

procedure TUInt64Bit10Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFF9);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFA);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 10;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit10ModTest }

function TUInt64Bit10ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit modulus by 10';
  end;

function TUInt64Bit10ModTest.GetDivisor: QWord;
  begin
    Result := 10;
  end;

procedure TUInt64Bit10ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFF9);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFFA);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := QWord(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 10;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit100Test }

function TUInt64Bit100Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit division by 100';
  end;

function TUInt64Bit100Test.GetDivisor: QWord;
  begin
    Result := 100;
  end;

procedure TUInt64Bit100Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFEF);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFF0);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 100;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit100ModTest }

function TUInt64Bit100ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit modulus by 100';
  end;

function TUInt64Bit100ModTest.GetDivisor: QWord;
  begin
    Result := 100;
  end;

procedure TUInt64Bit100ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      253:
        Numerator := QWord($FFFFFFFFFFFFFFEF);
      254:
        Numerator := QWord($FFFFFFFFFFFFFFF0);
      255:
        Numerator := QWord($FFFFFFFFFFFFFFFF);
      else
        Numerator := Cardinal(Index);
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 100;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit1000000000Test }

function TUInt64Bit1000000000Test.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit division by 1,000,000,000';
  end;

function TUInt64Bit1000000000Test.GetDivisor: QWord;
  begin
    Result := 1000000000;
  end;

procedure TUInt64Bit1000000000Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU64_1000000000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 1000000000;
      
    FResultArray[Index] := Answer;
  end;

{ TUInt64Bit1000000000ModTest }

function TUInt64Bit1000000000ModTest.TestTitle: shortstring;
  begin
    Result := 'Unsigned 64-bit modulus by 1,000,000,000';
  end;

function TUInt64Bit1000000000ModTest.GetDivisor: QWord;
  begin
    Result := 1000000000;
  end;

procedure TUInt64Bit1000000000ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: QWord;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    Numerator := FU64_1000000000Input[Index and $F];
    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 1000000000;
      
    FResultArray[Index] := Answer;
  end;
