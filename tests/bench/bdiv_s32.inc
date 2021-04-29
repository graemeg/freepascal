type
  { TSInt32Bit1Test }

  TSInt32Bit1Test = class(TSInt32DivTest)
    protected
      function GetDivisor: Integer; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt32Bit1ModTest }

  TSInt32Bit1ModTest = class(TSInt32ModTest)
    protected
      function GetDivisor: Integer; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt32Bit100Test }

  TSInt32Bit100Test = class(TSInt32DivTest)
    protected
      function GetDivisor: Integer; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

  { TSInt32Bit100ModTest }

  TSInt32Bit100ModTest = class(TSInt32ModTest)
    protected
      function GetDivisor: Integer; override;
      procedure DoTestIteration(Iteration: Integer); override;
    public
      function TestTitle: shortstring; override;
  end;

{ TSInt32Bit1Test }

function TSInt32Bit1Test.TestTitle: shortstring;
  begin
    Result := 'Signed 32-bit division by 1';
  end;

function TSInt32Bit1Test.GetDivisor: Integer;
  begin
    Result := 1;
  end;

procedure TSInt32Bit1Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Integer;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := -2147483648;
      1:
        Numerator := -2147483600;
      2:
        Numerator := -2147483599;
      253:
        Numerator := 2147483599;
      254:
        Numerator := 2147483600;
      255:
        Numerator := 2147483647;
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 1;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt32Bit1ModTest }

function TSInt32Bit1ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 32-bit modulus by 1';
  end;

function TSInt32Bit1ModTest.GetDivisor: Integer;
  begin
    Result := 1;
  end;

procedure TSInt32Bit1ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Integer;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := -2147483648;
      1:
        Numerator := -2147483600;
      2:
        Numerator := -2147483599;
      253:
        Numerator := 2147483599;
      254:
        Numerator := 2147483600;
      255:
        Numerator := 2147483647;
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 1;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt32Bit100Test }

function TSInt32Bit100Test.TestTitle: shortstring;
  begin
    Result := 'Signed 32-bit division by 100';
  end;

function TSInt32Bit100Test.GetDivisor: Integer;
  begin
    Result := 100;
  end;

procedure TSInt32Bit100Test.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Integer;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := -2147483648;
      1:
        Numerator := -2147483600;
      2:
        Numerator := -2147483599;
      253:
        Numerator := 2147483599;
      254:
        Numerator := 2147483600;
      255:
        Numerator := 2147483647;
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator div 100;
      
    FResultArray[Index] := Answer;
  end;

{ TSInt32Bit100ModTest }

function TSInt32Bit100ModTest.TestTitle: shortstring;
  begin
    Result := 'Signed 32-bit modulus by 100';
  end;

function TSInt32Bit100ModTest.GetDivisor: Integer;
  begin
    Result := 100;
  end;

procedure TSInt32Bit100ModTest.DoTestIteration(Iteration: Integer);
  var
    Numerator, Answer: Integer;
    Index, X: Integer;
  begin
    Index := Iteration and $FF;
    case Index of
      0:
        Numerator := -2147483648;
      1:
        Numerator := -2147483600;
      2:
        Numerator := -2147483599;
      253:
        Numerator := 2147483599;
      254:
        Numerator := 2147483600;
      255:
        Numerator := 2147483647;
      else
        Numerator := Index - 128;
    end;

    FInputArray[Index] := Numerator;
    for X := 0 to INTERNAL_LOOPS - 1 do
      Answer := Numerator mod 100;
      
    FResultArray[Index] := Answer;
  end;
