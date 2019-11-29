program trtti17;

{$mode objfpc}{$H+}

uses
  typinfo, variants;

type
  TEvent = procedure of object;

  TTestObj = object

  end;

  TTestRec = record

  end;

  TArrayDyn = array of LongInt;
  TArrayStatic = array[0..10] of LongInt;

  TSet = set of (alpha, beta, gamma);

var
  gError: LongInt = 0;

function NextErrorCode: LongInt; inline;
begin
  Inc(gError);
  Result := gError;
end;

procedure TestTypeInfo(aTypeInfo: PTypeInfo; aType: TTypeKind);
begin
  if aTypeInfo^.Kind <> aType then begin
    Writeln('TypeInfo failure; expected: ', aType, ', got: ', aTypeInfo^.Kind);
    Halt(NextErrorCode);
  end;
  NextErrorCode;
end;

generic procedure TestTypeKind<T>(aType: TTypeKind); inline;
begin
  if GetTypeKind(T) <> aType then begin
    Writeln('GetTypeKind() failure; expected: ', aType, ', got: ', GetTypeKind(T));
    Halt(NextErrorCode);
  end;
  TestTypeInfo(PTypeInfo(TypeInfo(T)), aType);
end;

begin
  specialize TestTypeKind<TObject>(tkClass);
  specialize TestTypeKind<TClass>(tkClassRef);
  specialize TestTypeKind<TProcedure>(tkProcVar);
  specialize TestTypeKind<TEvent>(tkMethod);
  specialize TestTypeKind<Int8>(tkInteger);
  specialize TestTypeKind<Int16>(tkInteger);
  specialize TestTypeKind<Int32>(tkInteger);
  specialize TestTypeKind<Int64>(tkInt64);
  specialize TestTypeKind<UInt8>(tkInteger);
  specialize TestTypeKind<UInt16>(tkInteger);
  specialize TestTypeKind<UInt32>(tkInteger);
  specialize TestTypeKind<UInt64>(tkQWord);
  specialize TestTypeKind<TTestObj>(tkObject);
  specialize TestTypeKind<TTestRec>(tkRecord);
  specialize TestTypeKind<TTypeKind>(tkEnumeration);
  specialize TestTypeKind<Boolean>(tkBool);
  specialize TestTypeKind<Boolean16>(tkBool);
  specialize TestTypeKind<Boolean32>(tkBool);
  specialize TestTypeKind<Boolean64>(tkBool);
  specialize TestTypeKind<ByteBool>(tkBool);
  specialize TestTypeKind<WordBool>(tkBool);
  specialize TestTypeKind<LongBool>(tkBool);
  specialize TestTypeKind<QWordBool>(tkBool);
  specialize TestTypeKind<Pointer>(tkPointer);
  specialize TestTypeKind<TArrayDyn>(tkDynArray);
  specialize TestTypeKind<TArrayStatic>(tkArray);
  specialize TestTypeKind<IInterface>(tkInterface);
  specialize TestTypeKind<IDispatch>(tkInterface);
  specialize TestTypeKind<ShortString>(tkSString);
  specialize TestTypeKind<AnsiString>(tkAString);
{$ifdef FPC_WIDESTRING_EQUAL_UNICODESTRING}
  specialize TestTypeKind<WideString>(tkUString);
{$else}
  specialize TestTypeKind<WideString>(tkWString);
{$endif}
  specialize TestTypeKind<UnicodeString>(tkUString);
  specialize TestTypeKind<AnsiChar>(tkChar);
  specialize TestTypeKind<WideChar>(tkWChar);
  specialize TestTypeKind<UnicodeChar>(tkWChar);
  specialize TestTypeKind<Single>(tkFloat);
  specialize TestTypeKind<Double>(tkFloat);
  specialize TestTypeKind<Extended>(tkFloat);
  specialize TestTypeKind<Currency>(tkFloat);
{$ifdef FPC_COMP_IS_INT64}
  specialize TestTypeKind<Comp>(tkInt64);
{$else}
  specialize TestTypeKind<Comp>(tkFloat);
{$endif}
  specialize TestTypeKind<TSet>(tkSet);
  specialize TestTypeKind<Variant>(tkVariant);
  {specialize TestTypeKind<file>(tkFile);
  specialize TestTypeKind<TextFile>(tkFile);}
  Writeln('ok');
end.
