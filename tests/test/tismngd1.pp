program tismngd1;

{$mode objfpc}
{$modeswitch advancedrecords}

uses
  TypInfo;

var
  gError: LongInt = 0;

function NextErrorCode: LongInt; inline;
begin
  Inc(gError);
  Result := gError;
end;

generic procedure TestType<T>(aIsMngd: Boolean); inline;
begin
  if IsManagedType(T) <> aIsMngd then begin
    Writeln('IsManagedType(', PTypeInfo(TypeInfo(T))^.Name, ') failure; expected: ', aIsMngd, ', got: ', IsManagedType(T));
    Halt(NextErrorCode);
  end;
  NextErrorCode;
end;

type
  TTestLongInt = record
    a: LongInt;
  end;

  TTestAnsiString = record
    a: AnsiString;
  end;

  TTestManaged = record
    a: LongInt;
    class operator Initialize(var aTestManaged: TTestManaged);
  end;

  TTestObj = object
    a: LongInt;
  end;

  TTestObjAnsiString = object
    a: AnsiString;
  end;

class operator TTestManaged.Initialize(var aTestManaged: TTestManaged);
begin
  aTestManaged.a := 42;
end;

type
  TProcVar = procedure;
  TMethodVar = procedure of object;

  TDynArrayLongInt = array of LongInt;
  TStaticArrayLongInt = array[0..4] of LongInt;
  TStaticArrayAnsiString = array[0..4] of AnsiString;

  TEnum = (eOne, eTwo, eThree);
  TSet = set of (sOne, sTwo, sThree);

begin
  specialize TestType<LongInt>(False);
  specialize TestType<Boolean>(False);
  specialize TestType<ShortString>(False);
  specialize TestType<AnsiString>(True);
  specialize TestType<UnicodeString>(True);
  specialize TestType<WideString>(True);
  specialize TestType<Single>(False);
  specialize TestType<TProcVar>(False);
  specialize TestType<TMethodVar>(False);
  specialize TestType<Pointer>(False);
  specialize TestType<IInterface>(True);
  specialize TestType<TObject>(False);
  specialize TestType<TTestLongInt>(False);
  specialize TestType<TTestAnsiString>(True);
  specialize TestType<TTestManaged>(True);
  specialize TestType<TTestObj>(False);
  specialize TestType<TTestObjAnsiString>(True);
  specialize TestType<TDynArrayLongInt>(True);
  specialize TestType<TStaticArrayLongInt>(False);
  specialize TestType<TStaticArrayAnsiString>(True);
  specialize TestType<TEnum>(False);
  specialize TestType<TSet>(False);
  Writeln('Ok');
end.
