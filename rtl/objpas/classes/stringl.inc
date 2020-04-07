{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************}
{*                        TStringsEnumerator                                *}
{****************************************************************************}

constructor TStringsEnumerator.Create(AStrings: TStrings);
begin
  inherited Create;
  FStrings := AStrings;
  FPosition := -1;
end;

function TStringsEnumerator.GetCurrent: String;
begin
  Result := FStrings[FPosition];
end;

function TStringsEnumerator.MoveNext: Boolean;
begin
  Inc(FPosition);
  Result := FPosition < FStrings.Count;
end;

{****************************************************************************}
{*                             TStrings                                     *}
{****************************************************************************}

// Function to quote text. Should move maybe to sysutils !!
// Also, it is not clear at this point what exactly should be done.

{ //!! is used to mark unsupported things. }

Function QuoteString (Const S : String; Const Quote : String) : String;
Var
  I,J : Integer;
begin
  J:=0;
  Result:=S;
  for i:=1 to length(s) do
   begin
     inc(j);
     if S[i]=Quote then
      begin
        System.Insert(Quote,Result,J);
        inc(j);
      end;
   end;
  Result:=Quote+Result+Quote;
end;

{
  For compatibility we can't add a Constructor to TSTrings to initialize
  the special characters. Therefore we add a routine which is called whenever
  the special chars are needed.
}

Procedure Tstrings.CheckSpecialChars;

begin
  If Not FSpecialCharsInited then
    begin
    FQuoteChar:='"';
    FDelimiter:=',';
    FNameValueSeparator:='=';
    FLBS:=DefaultTextLineBreakStyle;
    FSpecialCharsInited:=true;
    FLineBreak:=sLineBreak;
    end;
end;

Function TStrings.GetSkipLastLineBreak : Boolean;

begin
  Result:=not TrailingLineBreak;
end;

procedure TStrings.SetSkipLastLineBreak(const AValue : Boolean);

begin
  TrailingLineBreak:=not AValue;
end;

Function TStrings.GetLBS : TTextLineBreakStyle;
begin
  CheckSpecialChars;
  Result:=FLBS;
end;

Procedure TStrings.SetLBS (AValue : TTextLineBreakStyle);
begin
  CheckSpecialChars;
  FLBS:=AValue;
end;

procedure TStrings.SetDelimiter(c:Char);
begin
  CheckSpecialChars;
  FDelimiter:=c;
end;

Procedure TStrings.SetEncoding(const AEncoding: TEncoding);
begin
  if (FEncoding<>nil) and not TEncoding.IsStandardEncoding(FEncoding) then
    FEncoding.Free;

  if TEncoding.IsStandardEncoding(AEncoding) then
    FEncoding:=AEncoding
  else if AEncoding<>nil then
    FEncoding:=AEncoding.Clone
  else
    FEncoding:=nil;
end;

Function TStrings.GetDelimiter : Char;
begin
  CheckSpecialChars;
  Result:=FDelimiter;
end;

procedure TStrings.SetLineBreak(Const S : String);
begin
  CheckSpecialChars;
  FLineBreak:=S;
end;

Function TStrings.GetLineBreak : String;
begin
  CheckSpecialChars;
  Result:=FLineBreak;
end;


procedure TStrings.SetQuoteChar(c:Char);
begin
  CheckSpecialChars;
  FQuoteChar:=c;
end;

Function TStrings.GetQuoteChar :Char;
begin
  CheckSpecialChars;
  Result:=FQuoteChar;
end;

procedure TStrings.SetNameValueSeparator(c:Char);
begin
  CheckSpecialChars;
  FNameValueSeparator:=c;
end;



Function TStrings.GetNameValueSeparator :Char;
begin
  CheckSpecialChars;
  Result:=FNameValueSeparator;
end;


function TStrings.GetCommaText: string;

Var
  C1,C2 : Char;
  FSD : Boolean;

begin
  CheckSpecialChars;
  FSD:=StrictDelimiter;
  C1:=Delimiter;
  C2:=QuoteChar;
  Delimiter:=',';
  QuoteChar:='"';
  StrictDelimiter:=False;
  Try
    Result:=GetDelimitedText;
  Finally
    Delimiter:=C1;
    QuoteChar:=C2;
    StrictDelimiter:=FSD;
  end;
end;

function TStrings.GetLineBreakCharLBS: string;
begin
  CheckSpecialChars;
  if FLineBreak<>sLineBreak then
    Result:=FLineBreak
  else
    Case FLBS of
      tlbsLF   : Result:=#10;
      tlbsCRLF : Result:=#13#10;
      tlbsCR   : Result:=#13;
    end;
end;

function TStrings.GetMissingNameValueSeparatorAction: TMissingNameValueSeparatorAction;
begin
  CheckSpecialChars;
  Result:=FMissingNameValueSeparatorAction;
end;


Function TStrings.GetDelimitedText: string;

Var
  I : integer;
  p : pchar;
  BreakChars : set of char;
  S : String;
  doQuote : Boolean;
    
begin
  CheckSpecialChars;
  result:='';
  if StrictDelimiter then
    BreakChars:=[#0,QuoteChar,Delimiter]
  else  
    BreakChars:=[#0..' ',QuoteChar,Delimiter];

  // Check for break characters and quote if required.
  For i:=0 to count-1 do
    begin
    S:=Strings[i];
    doQuote:=FAlwaysQuote;
    If not DoQuote then
      begin
      p:=pchar(S);
      //Quote strings that include BreakChars:
      while not(p^ in BreakChars) do
        inc(p);
      DoQuote:=(p<>pchar(S)+length(S));  
      end;
    if DoQuote then
      Result:=Result+QuoteString(S,QuoteChar)
    else
      Result:=Result+S;
    if I<Count-1 then 
      Result:=Result+Delimiter;
    end;
  // Quote empty string:
  If (Length(Result)=0) and (Count=1) then
    Result:=QuoteChar+QuoteChar;
end;

procedure TStrings.GetNameValue(Index : Integer; Out AName,AValue : String);

Var L : longint;

begin
  aName:='';
  CheckSpecialChars;
  AValue:=Strings[Index];
  L:=Pos(FNameValueSeparator,AValue);
  If L<>0 then
    begin
    AName:=Copy(AValue,1,L-1);
    System.Delete(AValue,1,L);
    end
  else
    case FMissingNameValueSeparatorAction of
      mnvaValue : ;
      mnvaName :
        begin
        aName:=aValue;
        aValue:='';
        end;
      mnvaEmpty :
        aValue:='';
      mnvaError :
        Raise EStringListError.CreateFmt(SErrNoNameValuePairAt,[Index]);
    end;
end;

function TStrings.ExtractName(const s:String):String;
var
  L: Longint;
begin
  CheckSpecialChars;
  L:=Pos(FNameValueSeparator,S);
  If L<>0 then
    Result:=Copy(S,1,L-1)
  else
    Result:='';
end;


procedure TStrings.Filter(aFilter: TStringsFilterMethod; aList: TStrings);

var
  S : string;

begin
  for S in self do
    if aFilter(S) then
      aList.Add(S);
end;


procedure TStrings.ForEach(aCallback: TStringsForeachMethod);

var
  S : String;

begin
  for S in self do
    aCallBack(S);
end;


procedure TStrings.ForEach(aCallback: TStringsForeachMethodEx);

var
  i: integer;

begin
  for i:=0 to Count-1 do
    aCallBack(Strings[i],i);
end;


procedure TStrings.ForEach(aCallback: TStringsForeachMethodExObj);

var
  i: integer;

begin
  for i:=0 to Count-1 do
    aCallback(Strings[i],i,Objects[i]);
end;


function TStrings.Filter(aFilter: TStringsFilterMethod): TStrings;

begin
  Result:=TStringsClass(Self.ClassType).Create;
  try
    Filter(aFilter,Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

procedure TStrings.Fill(const aValue: String; aStart, aEnd: Integer);
var
  i: integer;
begin
  if aEnd<0 then
    aEnd:=Self.Count+aEnd;
  if aEnd>=Count then
    aEnd:=Count-1;
  for i:=aStart to aEnd do
    Strings[i]:=aValue;
end;


Procedure TStrings.Map(aMap: TStringsMapMethod; aList : TStrings);

Var
  S : String;

begin
  For S in self do
    aList.Add(aMap(S));
end;


Function TStrings.Map(aMap: TStringsMapMethod) : TStrings;

begin
  Result:=TStringsClass(Self.ClassType).Create;
  try
    Map(aMap,Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;


function TStrings.Reduce(aReduceMethod: TStringsReduceMethod; const startingValue: string): string;

var
  S : String;

begin
  Result:=startingValue;
  for S in self do
    Result:=aReduceMethod(Result, S);
end;


Function TStrings.Reverse : TStrings;

begin
  Result:=TStringsClass(Self.ClassType).Create;
  try
    Reverse(Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;


Procedure TStrings.Reverse(aList : TStrings);

Var
  I : Integer;

begin
  for I:=Count-1 downto 0 do
    aList.Add(Strings[i]);
end;


Procedure TStrings.Slice(fromIndex: integer; aList : TStrings);

var
  i: integer;

begin
  for i:=fromIndex to Count-1 do
    aList.Add(Self[i]);
end;

Function TStrings.Slice(fromIndex: integer) :  TStrings;

begin
  Result:=TStringsClass(Self.ClassType).Create;
  try
    Slice(FromIndex,Result);
  except
    FreeAndNil(Result);
    Raise;
  end;
end;

function TStrings.GetName(Index: Integer): string;

Var
  V : String;

begin
  GetNameValue(Index,Result,V);
end;

function TStrings.GetStrictDelimiter: Boolean;
begin
  Result:=soStrictDelimiter in FOptions;
end;

function TStrings.GetTrailingLineBreak: Boolean;
begin
  Result:=soTrailingLineBreak in FOptions;
end;

function TStrings.GetUseLocale: Boolean;
begin
  Result:=soUseLocale in FOptions;
end;

function TStrings.GetWriteBOM: Boolean;
begin
  Result:=soWriteBOM in FOptions;
end;

Function TStrings.GetValue(const Name: string): string;

Var
  L : longint;
  N : String;

begin
  Result:='';
  L:=IndexOfName(Name);
  If L<>-1 then
    GetNameValue(L,N,Result);
end;

Function TStrings.GetValueFromIndex(Index: Integer): string;

Var
  N : String;

begin
  GetNameValue(Index,N,Result);
end;

Procedure TStrings.SetValueFromIndex(Index: Integer; const Value: string);

begin
  If (Value='') then
    Delete(Index)
  else
    begin
    If (Index<0) then
      Index:=Add('');
    CheckSpecialChars;
    Strings[Index]:=GetName(Index)+FNameValueSeparator+Value;
    end;
end;

procedure TStrings.ReadData(Reader: TReader);
begin
  Reader.ReadListBegin;
  BeginUpdate;
  try
    Clear;
    while not Reader.EndOfList do
      Add(Reader.ReadString);
  finally
    EndUpdate;
  end;
  Reader.ReadListEnd;
end;


Procedure TStrings.SetDelimitedText(const AValue: string);

begin
  CheckSpecialChars;
  DoSetDelimitedText(aValue,True,StrictDelimiter,FQuoteChar,FDelimiter);
end;

Procedure TStrings.DoSetDelimitedText(const AValue: string; DoClear,aStrictDelimiter : Boolean; aQuoteChar,aDelimiter : Char);

var
  len,i,j: SizeInt;
  aNotFirst:boolean;

  Procedure AddQuoted;

  begin
    Add(StringReplace(Copy(AValue,i+1,j-i-1),aQuoteChar+aQuoteChar,aQuoteChar, [rfReplaceAll]));
  end;

begin
 BeginUpdate;

 i:=1;
 j:=1;
 aNotFirst:=false;

 { Paraphrased from Delphi XE2 help:
 Strings must be separated by Delimiter characters or spaces.
 They may be enclosed in QuoteChars.
 QuoteChars in the string must be repeated to distinguish them from the QuoteChars enclosing the string.
 }
 try
  if DoClear then
    Clear;
  len:=length(AValue);
  If aStrictDelimiter then
    begin
    while i<=Len do begin
     // skip delimiter
     if aNotFirst and (i<=len) and (AValue[i]=aDelimiter) then
       inc(i);

     // read next string
     if i<=len then begin
      if AValue[i]=aQuoteChar then begin
       // next string is quoted
       j:=i+1;
       while (j<=len) and
             ((AValue[j]<>aQuoteChar) or
              ((j+1<=len) and (AValue[j+1]=aQuoteChar))) do
        begin
        if (j<=len) and (AValue[j]=aQuoteChar) then
          inc(j,2)
        else
          inc(j);
        end;
       AddQuoted;
       i:=j+1;
      end else begin
       // next string is not quoted; read until delimiter
       j:=i;
       while (j<=len) and
             (AValue[j]<>aDelimiter) do inc(j);
       Add( Copy(AValue,i,j-i));
       i:=j;
      end;
     end else begin
      if aNotFirst then Add('');
     end;
     aNotFirst:=true;
    end;
    end
  else 
    begin
    while i<=len do begin
     // skip delimiter
     if aNotFirst and (i<=len) and (AValue[i]=aDelimiter) then inc(i);

     // skip spaces
     while (i<=len) and (Ord(AValue[i])<=Ord(' ')) do inc(i);
    
     // read next string
     if i<=len then begin
      if AValue[i]=aQuoteChar then begin
       // next string is quoted
       j:=i+1;
       while (j<=len) and
             ( (AValue[j]<>aQuoteChar) or
               ( (j+1<=len) and (AValue[j+1]=aQuoteChar) ) ) do begin
        if (j<=len) and (AValue[j]=aQuoteChar) then inc(j,2)
                                                          else inc(j);
       end;
       AddQuoted;
       i:=j+1;
      end else begin
       // next string is not quoted; read until control character/space/delimiter
       j:=i;
       while (j<=len) and
             (Ord(AValue[j])>Ord(' ')) and
             (AValue[j]<>aDelimiter) do inc(j);
       Add( Copy(AValue,i,j-i));
       i:=j;
      end;
     end else begin
      if aNotFirst then Add('');
     end;

     // skip spaces
     while (i<=len) and (Ord(AValue[i])<=Ord(' ')) do inc(i);

     aNotFirst:=true;
    end;
    end;
 finally
   EndUpdate;
 end;
end;

Procedure TStrings.SetCommaText(const Value: string);

begin
  CheckSpecialChars;
  DoSetDelimitedText(Value,True,StrictDelimiter,'"',',');
end;

procedure TStrings.SetMissingNameValueSeparatorAction(AValue: TMissingNameValueSeparatorAction);
begin
  CheckSpecialChars;
  FMissingNameValueSeparatorAction:=aValue;
end;


Procedure TStrings.SetStringsAdapter(const Value: IStringsAdapter);

begin
end;

procedure TStrings.SetStrictDelimiter(AValue: Boolean);
begin
  if AValue then
    Include(FOptions,soStrictDelimiter)
  else
    Exclude(FOptions,soStrictDelimiter);
end;

procedure TStrings.SetTrailingLineBreak(AValue: Boolean);
begin
  if AValue then
    Include(FOptions,soTrailingLineBreak)
  else
    Exclude(FOptions,soTrailingLineBreak);
end;

procedure TStrings.SetUseLocale(AValue: Boolean);
begin
  if AValue then
    Include(FOptions,soUseLocale)
  else
    Exclude(FOptions,soUseLocale);
end;


procedure TStrings.SetWriteBOM(AValue: Boolean);
begin
  if AValue then
    Include(FOptions,soWriteBOM)
  else
    Exclude(FOptions,soWriteBOM);
end;



Procedure TStrings.SetDefaultEncoding(const ADefaultEncoding: TEncoding);
begin
  if (FDefaultEncoding<>nil) and not TEncoding.IsStandardEncoding(FDefaultEncoding) then
    FDefaultEncoding.Free;

  if TEncoding.IsStandardEncoding(ADefaultEncoding) then
    FDefaultEncoding:=ADefaultEncoding
  else if ADefaultEncoding<>nil then
    FDefaultEncoding:=ADefaultEncoding.Clone
  else
    FDefaultEncoding:=TEncoding.Default;
end;



Procedure TStrings.SetValue(const Name, Value: string);

Var L : longint;

begin
  CheckSpecialChars;
  L:=IndexOfName(Name);
  if L=-1 then
   Add (Name+FNameValueSeparator+Value)
  else
   Strings[L]:=Name+FNameValueSeparator+value;
end;



procedure TStrings.WriteData(Writer: TWriter);
var
  i: Integer;
begin
  Writer.WriteListBegin;
  for i := 0 to Count - 1 do
    Writer.WriteString(Strings[i]);
  Writer.WriteListEnd;
end;



function TStrings.CompareStrings(const s1,s2 : string) : Integer;
begin
  Result := DoCompareText(s1, s2);
end;



procedure TStrings.DefineProperties(Filer: TFiler);
var
  HasData: Boolean;
begin
  if Assigned(Filer.Ancestor) then
    // Only serialize if string list is different from ancestor
    if Filer.Ancestor.InheritsFrom(TStrings) then
      HasData := not Equals(TStrings(Filer.Ancestor))
    else
      HasData := True
  else
    HasData := Count > 0;
  Filer.DefineProperty('Strings', @ReadData, @WriteData, HasData);
end;


Procedure TStrings.Error(const Msg: string; Data: Integer);
begin
  Raise EStringListError.CreateFmt(Msg,[Data]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;


Procedure TStrings.Error(const Msg: pstring; Data: Integer);
begin
  Raise EStringListError.CreateFmt(Msg^,[Data]) at get_caller_addr(get_frame), get_caller_frame(get_frame);
end;


Function TStrings.GetCapacity: Integer;

begin
  Result:=Count;
end;



Function TStrings.GetObject(Index: Integer): TObject;

begin
  Result:=Nil;
end;



Function TStrings.GetTextStr: string;

Var P : Pchar;
    I,L,NLS : SizeInt;
    S,NL : String;

begin
  NL:=GetLineBreakCharLBS;
  // Determine needed place
  L:=0;
  NLS:=Length(NL);
  For I:=0 to count-1 do
    L:=L+Length(Strings[I])+NLS;
  if SkipLastLineBreak then
    Dec(L,NLS);
  Setlength(Result,L);
  P:=Pointer(Result);
  For i:=0 To count-1 do
    begin
    S:=Strings[I];
    L:=Length(S);
    if L<>0 then
      System.Move(Pointer(S)^,P^,L);
    P:=P+L;
    if (I<Count-1) or Not SkipLastLineBreak then
      For L:=1 to NLS do
        begin
        P^:=NL[L];
        inc(P);
        end;
    end;
end;



Procedure TStrings.Put(Index: Integer; const S: string);

Var Obj : TObject;

begin
  Obj:=Objects[Index];
  Delete(Index);
  InsertObject(Index,S,Obj);
end;



Procedure TStrings.PutObject(Index: Integer; AObject: TObject);

begin
  // Empty.
end;



Procedure TStrings.SetCapacity(NewCapacity: Integer);

begin
  // Empty.
end;

Class Function TStrings.GetNextLine (Const Value : String; Var S : String; Var P : SizeInt) : Boolean;

var
  LengthOfValue: SizeInt;
  StartPos, FuturePos: SizeInt;

begin
  LengthOfValue := Length(Value);
  StartPos := P;
  if (StartPos <= 0) or (StartPos > LengthOfValue) then // True for LengthOfValue <= 0
    begin
    S := '';
    Exit(False);
    end;
  FuturePos := StartPos;
  while (FuturePos <= LengthOfValue) and not (Value[FuturePos] in [#10, #13]) do
    Inc(FuturePos);
  // If we use S := Copy(Value, StartPos, FuturePos - StartPos); then compiler
  // generate TempS := Copy(...); S := TempS to eliminate side effects and
  // implicit "try finally" for TempS finalization
  // When we use SetString then no TempS, no try finally generated,
  // but we must check case when Value and S is same (side effects)
  if Pointer(S) = Pointer(Value) then
    System.Delete(S, FuturePos, High(FuturePos))
  else
    begin
    SetString(S, @Value[StartPos], FuturePos - StartPos);
    if (FuturePos <= LengthOfValue) and (Value[FuturePos] = #13) then
      Inc(FuturePos);
    if (FuturePos <= LengthOfValue) and (Value[FuturePos] = #10) then
      Inc(FuturePos);
    end;
  P := FuturePos;
  Result := True;
end;

Function TStrings.GetNextLineBreak (Const Value : String; Var S : String; Var P : SizeInt) : Boolean;

var
  StartPos, FuturePos: SizeInt;
  
begin
  StartPos := P;
  if (StartPos <= 0) or (StartPos > Length(Value)) then // True for Length <= 0
    begin
    S := '';
    Exit(False);
    end;
  FuturePos := Pos(FLineBreak, Value, StartPos); // Use PosEx in old RTL
  // Why we don't use Copy but use SetString read in GetNextLine
  if FuturePos = 0 then // No line breaks
    begin
    FuturePos := Length(Value) + 1;
    if Pointer(S) = Pointer(Value) then
      // Nothing to do
    else
      SetString(S, @Value[StartPos], FuturePos - StartPos)
    end
  else
    if Pointer(S) = Pointer(Value) then
      System.Delete(S, FuturePos, High(FuturePos))
    else
      begin
      SetString(S, @Value[StartPos], FuturePos - StartPos);
      Inc(FuturePos, Length(FLineBreak));
      end;
  P := FuturePos;
  Result := True;
end;

{$IF (SizeOf(Integer) < SizeOf(SizeInt)) }
class function TStrings.GetNextLine(const Value: string; var S: string; var P: Integer) : Boolean;
var
  LP: SizeInt;
begin
  LP := P;
  Result := GetNextLine(Value, S, LP);
  P := LP;
end;

function TStrings.GetNextLineBreak(const Value: string; var S: string; var P: Integer) : Boolean;
var
  LP: SizeInt;
begin
  LP := P;
  Result := GetNextLineBreak(Value, S, LP);
  P := LP;
end;
{$IFEND}

Procedure TStrings.DoSetTextStr(const Value: string; DoClear : Boolean);

Var
  S : String;
  P : SizeInt;

begin
  Try
    beginUpdate;
    if DoClear then
      Clear;
    P:=1;
    if FLineBreak=sLineBreak then
      begin
      While GetNextLine (Value,S,P) do
        Add(S)
      end
    else
      While GetNextLineBreak (Value,S,P) do
        Add(S);
  finally
    EndUpdate;
  end;
end;

Procedure TStrings.SetTextStr(const Value: string);

begin
  CheckSpecialChars;
  DoSetTextStr(Value,True);
end;

Procedure TStrings.AddText(const S: string);

begin
  CheckSpecialChars;
  DoSetTextStr(S,False);
end;

procedure TStrings.AddCommaText(const S: String);

begin
  DoSetDelimitedText(S,False,StrictDelimiter,'"',',');
end;

procedure TStrings.AddDelimitedText(const S: String; ADelimiter: Char; AStrictDelimiter: Boolean);

begin
  CheckSpecialChars;
  DoSetDelimitedText(S,False,AStrictDelimiter,FQuoteChar,ADelimiter);
end;

procedure TStrings.AddDelimitedText(const S: String);
begin
  CheckSpecialChars;
  DoSetDelimitedText(S,False,StrictDelimiter,FQuoteChar,FDelimiter);
end;

Procedure TStrings.SetUpdateState(Updating: Boolean);

begin
  FPONotifyObservers(Self,ooChange,Nil);
end;


destructor TSTrings.Destroy;

begin
  if (FEncoding<>nil) and not TEncoding.IsStandardEncoding(FEncoding) then
    FreeAndNil(FEncoding);
  if (FDefaultEncoding<>nil) and not TEncoding.IsStandardEncoding(FDefaultEncoding) then
    FreeAndNil(FDefaultEncoding);
  inherited destroy;
end;

function TStrings.ToObjectArray: TObjectDynArray;

begin
  Result:=ToObjectArray(0,Count-1);
end;

function TStrings.ToObjectArray(aStart,aEnd : Integer): TObjectDynArray;
Var
  I : Integer;

begin
  Result:=Nil;
  if aStart>aEnd then exit;
  SetLength(Result,aEnd-aStart+1);
  For I:=aStart to aEnd do
    Result[i-aStart]:=Objects[i];
end;

function TStrings.ToStringArray: TStringDynArray;

begin
  Result:=ToStringArray(0,Count-1);
end;

function TStrings.ToStringArray(aStart,aEnd : Integer): TStringDynArray;

Var
  I : Integer;

begin
  Result:=Nil;
  if aStart>aEnd then exit;
  SetLength(Result,aEnd-aStart+1);
  For I:=aStart to aEnd do
    Result[i-aStart]:=Strings[i];
end;


constructor TStrings.Create;
begin
  inherited Create;
  FDefaultEncoding:=TEncoding.Default;
  FEncoding:=nil;
  FOptions := [soTrailingLineBreak,soUseLocale,soPreserveBOM];
  FAlwaysQuote:=False;
end;

Function TStrings.Add(const S: string): Integer;

begin
  Result:=Count;
  Insert (Count,S);
end;

function TStrings.Add(const Fmt : string; const Args : Array of const): Integer;

begin
  Result:=Add(Format(Fmt,Args));
end;


Function TStrings.AddObject(const S: string; AObject: TObject): Integer;

begin
  Result:=Add(S);
  Objects[result]:=AObject;
end;

function TStrings.AddObject(const Fmt: string; Args : Array of const; AObject: TObject): Integer;

begin
  Result:=AddObject(Format(Fmt,Args),AObject);
end;

function TStrings.AddPair(const AName, AValue: string): TStrings;
begin
  Result:=AddPair(AName,AValue,Nil);
end;

function TStrings.AddPair(const AName, AValue: string; AObject: TObject): TStrings;
begin
  Result := Self;
  AddObject(Concat(AName, NameValueSeparator, AValue), AObject);
end;

Procedure TStrings.Append(const S: string);

begin
  Add (S);
end;



Procedure TStrings.AddStrings(TheStrings: TStrings; ClearFirst : Boolean);

Var Runner : longint;
begin
  beginupdate;
  try
    if ClearFirst then
      Clear;
    if Count + TheStrings.Count > Capacity then
      Capacity := Count + TheStrings.Count;
    For Runner:=0 to TheStrings.Count-1 do
      self.AddObject (Thestrings[Runner],TheStrings.Objects[Runner]);
  finally
    EndUpdate;
  end;
end;

Procedure TStrings.AddStrings(TheStrings: TStrings);

begin
  AddStrings(TheStrings, False);
end;

Procedure TStrings.AddStrings(const TheStrings: array of string);

begin
  AddStrings(TheStrings, False);
end;

Procedure TStrings.AddStrings(const TheStrings: array of string; ClearFirst : Boolean);

Var Runner : longint;
begin
  beginupdate;
  try
    if ClearFirst then
      Clear;
    if Count + High(TheStrings)+1 > Capacity then
      Capacity := Count + High(TheStrings)+1;
    For Runner:=Low(TheStrings) to High(TheStrings) do
      self.Add(Thestrings[Runner]);
  finally
    EndUpdate;
  end;
end;

procedure TStrings.SetStrings(TheStrings: TStrings);

begin
  AddStrings(TheStrings,True);
end;

procedure TStrings.SetStrings(TheStrings: array of string);

begin
  AddStrings(TheStrings,True);
end;

Procedure TStrings.Assign(Source: TPersistent);

Var
  S : TStrings;

begin
  If Source is TStrings then
    begin
    S:=TStrings(Source);
    BeginUpdate;
    Try
      clear;
      FSpecialCharsInited:=S.FSpecialCharsInited;
      FQuoteChar:=S.FQuoteChar;
      FDelimiter:=S.FDelimiter;
      FNameValueSeparator:=S.FNameValueSeparator;
      FLBS:=S.FLBS;
      FLineBreak:=S.FLineBreak;
      FOptions:=S.FOptions;
      DefaultEncoding:=S.DefaultEncoding;
      SetEncoding(S.Encoding);
      AddStrings(S);
    finally
      EndUpdate;
    end;
    end
  else
    Inherited Assign(Source);
end;



Procedure TStrings.BeginUpdate;

begin
   if FUpdateCount = 0 then SetUpdateState(true);
   inc(FUpdateCount);
end;



Procedure TStrings.EndUpdate;

begin
  If FUpdateCount>0 then
     Dec(FUpdateCount);
  if FUpdateCount=0 then
    SetUpdateState(False);
end;



Function TStrings.Equals(Obj: TObject): Boolean;

begin
  if Obj is TStrings then
    Result := Equals(TStrings(Obj))
  else
    Result := inherited Equals(Obj);
end;



Function TStrings.Equals(TheStrings: TStrings): Boolean;

Var Runner,Nr : Longint;

begin
  Result:=False;
  Nr:=Self.Count;
  if Nr<>TheStrings.Count then exit;
  For Runner:=0 to Nr-1 do
    If Strings[Runner]<>TheStrings[Runner] then exit;
  Result:=True;
end;



Procedure TStrings.Exchange(Index1, Index2: Integer);

Var
  Obj : TObject;
  Str : String;

begin
  beginUpdate;
  Try
    Obj:=Objects[Index1];
    Str:=Strings[Index1];
    Objects[Index1]:=Objects[Index2];
    Strings[Index1]:=Strings[Index2];
    Objects[Index2]:=Obj;
    Strings[Index2]:=Str;
  finally
    EndUpdate;
  end;
end;


function TStrings.GetEnumerator: TStringsEnumerator;
begin
  Result:=TStringsEnumerator.Create(Self);
end;


Function TStrings.GetText: PChar;
begin
  Result:=StrNew(Pchar(Self.Text));
end;


Function TStrings.DoCompareText(const s1,s2 : string) : PtrInt;
  begin
    if UseLocale then
      result:=AnsiCompareText(s1,s2)
    else
      result:=CompareText(s1,s2);
  end;


Function TStrings.IndexOf(const S: string): Integer;
begin
  Result:=0;
  While (Result<Count) and (DoCompareText(Strings[Result],S)<>0) do Result:=Result+1;
  if Result=Count then Result:=-1;
end;

function TStrings.IndexOf(const S: string; aStart: Integer): Integer;
begin
  if aStart<0 then
    begin
    aStart:=Count+aStart;
    if aStart<0 then
      aStart:=0;
    end;
  Result:=aStart;
  While (Result<Count) and (DoCompareText(Strings[Result],S)<>0) do Result:=Result+1;
  if Result=Count then Result:=-1;
end;


Function TStrings.IndexOfName(const Name: string): Integer;
Var
  len : longint;
  S : String;
begin
  CheckSpecialChars;
  Result:=0;
  while (Result<Count) do
    begin
    S:=Strings[Result];
    len:=pos(FNameValueSeparator,S)-1;
    if (len>=0) and (DoCompareText(Name,Copy(S,1,Len))=0) then
      exit;
    inc(result);
    end;
  result:=-1;
end;


Function TStrings.IndexOfObject(AObject: TObject): Integer;
begin
  Result:=0;
  While (Result<count) and (Objects[Result]<>AObject) do Result:=Result+1;
  If Result=Count then Result:=-1;
end;


Procedure TStrings.InsertObject(Index: Integer; const S: string;
  AObject: TObject);

begin
  Insert (Index,S);
  Objects[Index]:=AObject;
end;

function TStrings.LastIndexOf(const S: string): Integer;

begin
  Result:=LastIndexOf(S,Count-1);
end;

function TStrings.LastIndexOf(const S: string; aStart : Integer): Integer;
begin
  if aStart<0 then
    begin
    aStart:=Count+aStart;
    if aStart<0 then
      aStart:=0;
    end;
  Result:=aStart;
  if Result>=Count-1 then
    Result:=Count-1;
  While (Result>=0) and (DoCompareText(Strings[Result],S)<>0) do
    Result:=Result-1;
end;

Procedure TStrings.LoadFromFile(const FileName: string);

begin
  LoadFromFile(FileName,False)
end;

Procedure TStrings.LoadFromFile(const FileName: string; IgnoreEncoding : Boolean);
Var
  TheStream : TFileStream;
begin
  TheStream:=TFileStream.Create(FileName,fmOpenRead or fmShareDenyWrite);
  try
    LoadFromStream(TheStream, IgnoreEncoding);
  finally
    TheStream.Free;
  end;
end;



Procedure TStrings.LoadFromFile(const FileName: string; AEncoding: TEncoding);
Var
        TheStream : TFileStream;
begin
  TheStream:=TFileStream.Create(FileName,fmOpenRead or fmShareDenyWrite);
  try
    LoadFromStream(TheStream,AEncoding);
  finally
    TheStream.Free;
  end;
end;

Procedure TStrings.LoadFromStream(Stream: TStream);

begin
  LoadFromStream(Stream,False);
end;

Const
  LoadBufSize = 1024;
  LoadMaxGrow = MaxInt Div 2;

Procedure TStrings.LoadFromStream(Stream: TStream; IgnoreEncoding : Boolean);
{
   Borlands method is no good, since a pipe for
   instance doesn't have a size.
   So we must do it the hard way.
}

Var
  Buffer : AnsiString;
  BufLen : SizeInt;
  BytesRead, I, BufDelta : Longint;

begin
  if not IgnoreEncoding then
    begin
    LoadFromStream(Stream,Nil);
    Exit;
    end;
  // reread into a buffer
  beginupdate;
  try
    Buffer:='';
    BufLen:=0;
    I:=1;
    Repeat
      BufDelta:=LoadBufSize*I;
      SetLength(Buffer,BufLen+BufDelta);
      BytesRead:=Stream.Read(Buffer[BufLen+1],BufDelta);
      inc(BufLen,BufDelta);
      If I<LoadMaxGrow then
        I:=I shl 1;
    Until BytesRead<>BufDelta;
    SetLength(Buffer, BufLen-BufDelta+BytesRead);
    SetTextStr(Buffer);
    SetLength(Buffer,0);
  finally
    EndUpdate;
  end;
  if soPreserveBOM in FOptions then
    WriteBOM:=False;
end;


Procedure TStrings.LoadFromStream(Stream: TStream; AEncoding: TEncoding);
{
   Borlands method is no good, since a pipe for
   instance doesn't have a size.
   So we must do it the hard way.
}

Var
  Buffer : TBytes;
  T : string;
  BufLen : SizeInt;
  BytesRead, I, BufDelta, PreambleLength : Longint;

begin
  // reread into a buffer
  beginupdate;
  try
    SetLength(Buffer,0);
    BufLen:=0;
    I:=1;
    Repeat
      BufDelta:=LoadBufSize*I;
      SetLength(Buffer,BufLen+BufDelta);
      BytesRead:=Stream.Read(Buffer[BufLen],BufDelta);
      inc(BufLen,BufDelta);
      If I<LoadMaxGrow then
        I:=I shl 1;
    Until BytesRead<>BufDelta;
    SetLength(Buffer,BufLen-BufDelta+BytesRead);
    PreambleLength:=TEncoding.GetBufferEncoding(Buffer,AEncoding,FDefaultEncoding);
    T:=AEncoding.GetAnsiString(Buffer,PreambleLength,Length(Buffer)-PreambleLength);
    if soPreserveBOM in FOptions then
      WriteBOM:=PreambleLength>0;
    SetEncoding(AEncoding);
    SetLength(Buffer,0);
    SetTextStr(T);
  finally
    EndUpdate;
  end;
end;


Procedure TStrings.Move(CurIndex, NewIndex: Integer);
Var
  Obj : TObject;
  Str : String;
begin
  BeginUpdate;
  Try
    Obj:=Objects[CurIndex];
    Str:=Strings[CurIndex];
    Objects[CurIndex]:=Nil; // Prevent Delete from freeing.
    Delete(Curindex);
    InsertObject(NewIndex,Str,Obj);
  finally
    EndUpdate;
    end;
end;

function TStrings.Pop: string;

var
  C : Integer;

begin
  Result:='';
  C:=Count-1;
  if (C>=0) then
    begin
    Result:=Strings[C];
    Delete(C);
    end;
end;

function TStrings.Shift: String;

begin
  Result:='';
  if (Count > 0) then
    begin
    Result:=Strings[0];
    Delete(0);
    end;
end;

Procedure TStrings.SaveToFile(const FileName: string);

Var TheStream : TFileStream;

begin
  TheStream:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToStream(TheStream);
  finally
    TheStream.Free;
  end;
end;



Procedure TStrings.SaveToFile(const FileName: string; IgnoreEncoding : Boolean);

Var TheStream : TFileStream;

begin
  TheStream:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToStream(TheStream, IgnoreEncoding);
  finally
    TheStream.Free;
  end;
end;



Procedure TStrings.SaveToFile(const FileName: string; AEncoding: TEncoding);

Var TheStream : TFileStream;

begin
  TheStream:=TFileStream.Create(FileName,fmCreate);
  try
    SaveToStream(TheStream,AEncoding);
  finally
    TheStream.Free;
  end;
end;



Procedure TStrings.SaveToStream(Stream: TStream);
begin
  SaveToStream(Stream,False)
end;



Procedure TStrings.SaveToStream(Stream: TStream; IgnoreEncoding: Boolean);
Var
  I,L,NLS : SizeInt;
  S,NL : String;

begin
  if not IgnoreEncoding then
    begin
    SaveToStream(Stream,FEncoding);
    Exit;
    end;
  NL:=GetLineBreakCharLBS;
  NLS:=Length(NL)*SizeOf(Char);
  For i:=0 To count-1 do
    begin
    S:=Strings[I];
    L:=Length(S);
    if L<>0 then
      Stream.WriteBuffer(S[1], L*SizeOf(Char));
    if (I<Count-1) or Not SkipLastLineBreak then
      Stream.WriteBuffer(NL[1], NLS);
    end;
end;




Procedure TStrings.SaveToStream(Stream: TStream; AEncoding: TEncoding);

Var B,BNL : TBytes;
    NL,S: string;
    i,BNLS: SizeInt;

begin
  if AEncoding=nil then
    AEncoding:=FDefaultEncoding;
  if WriteBOM then
    begin
      B:=AEncoding.GetPreamble;
      if Length(B)>0 then
        Stream.WriteBuffer(B[0],Length(B));
    end;

  NL := GetLineBreakCharLBS;
  BNL:=AEncoding.GetAnsiBytes(NL);
  BNLS:=Length(BNL);
  For i:=0 To count-1 do
    begin
    S:=Strings[I];
    if S<>'' then
      begin
      B:=AEncoding.GetAnsiBytes(S);
      Stream.WriteBuffer(B[0],Length(B));
      end;
    if (I<Count-1) or Not SkipLastLineBreak then
      Stream.WriteBuffer(BNL[0],BNLS);
    end;
end;




Procedure TStrings.SetText(TheText: PChar);

Var S : String;

begin
  If TheText<>Nil then
    S:=StrPas(TheText)
  else
    S:='';
  SetTextStr(S);  
end;


{****************************************************************************}
{*                             TStringList                                  *}
{****************************************************************************}

{$if not defined(FPC_TESTGENERICS)}

procedure TStringList.ExchangeItemsInt(Index1, Index2: Integer);

Var P1,P2 : Pointer;

begin
  P1:=Pointer(Flist^[Index1].FString);
  P2:=Pointer(Flist^[Index1].FObject);
  Pointer(Flist^[Index1].Fstring):=Pointer(Flist^[Index2].Fstring);
  Pointer(Flist^[Index1].FObject):=Pointer(Flist^[Index2].FObject);
  Pointer(Flist^[Index2].Fstring):=P1;
  Pointer(Flist^[Index2].FObject):=P2;
end;

function TStringList.GetSorted: Boolean;
begin
  Result:=FSortStyle in [sslUser,sslAuto];
end;


procedure TStringList.ExchangeItems(Index1, Index2: Integer);
begin
  ExchangeItemsInt(Index1, Index2);
end;


procedure TStringList.Grow;

Var
  NC : Integer;

begin
  NC:=FCapacity;
  If NC>=256 then
    NC:=NC+(NC Div 4)
  else if NC=0 then
    NC:=4
  else
    NC:=NC*4;
  SetCapacity(NC);
end;

procedure TStringList.InternalClear(FromIndex: Integer; ClearOnly: Boolean);

Var
  I: Integer;

begin
  if FromIndex < FCount then
    begin
      if FOwnsObjects then
        begin
          For I:=FromIndex to FCount-1 do
            begin
              Flist^[I].FString:='';
              freeandnil(Flist^[i].FObject);
            end;
        end
      else
        begin
          For I:=FromIndex to FCount-1 do
            Flist^[I].FString:='';
        end;
      FCount:=FromIndex;
    end;
  if Not ClearOnly then
    SetCapacity(0);
end;


procedure TStringList.InsertItem(Index: Integer; const S: string);
begin
  InsertItem(Index, S, nil);
end;


procedure TStringList.InsertItem(Index: Integer; const S: string; O: TObject);
begin
  Changing;
  If FCount=Fcapacity then Grow;
  If Index<FCount then
    System.Move (FList^[Index],FList^[Index+1],
                 (FCount-Index)*SizeOf(TStringItem));
  Pointer(Flist^[Index].Fstring):=Nil;  // Needed to initialize...
  Flist^[Index].FString:=S;
  Flist^[Index].FObject:=O;
  Inc(FCount);
  Changed;
end;


procedure TStringList.SetSorted(Value: Boolean);

begin
  If Value then
    SortStyle:=sslAuto
  else
    SortStyle:=sslNone
end;



procedure TStringList.Changed;

begin
  If (FUpdateCount=0) Then
   begin
   If Assigned(FOnChange) then
     FOnchange(Self);
   FPONotifyObservers(Self,ooChange,Nil);
   end;
end;



procedure TStringList.Changing;

begin
  If FUpdateCount=0 then
    if Assigned(FOnChanging) then
      FOnchanging(Self);
end;



function TStringList.Get(Index: Integer): string;

begin
  CheckIndex(Index);
  Result:=Flist^[Index].FString;
end;



function TStringList.GetCapacity: Integer;

begin
  Result:=FCapacity;
end;



function TStringList.GetCount: Integer;

begin
  Result:=FCount;
end;



function TStringList.GetObject(Index: Integer): TObject;

begin
  CheckIndex(Index);
  Result:=Flist^[Index].FObject;
end;



procedure TStringList.Put(Index: Integer; const S: string);

begin
  If Sorted then
    Error(SSortedListError,0);
  CheckIndex(Index);
  Changing;
  Flist^[Index].FString:=S;
  Changed;
end;



procedure TStringList.PutObject(Index: Integer; AObject: TObject);

begin
  CheckIndex(Index);
  Changing;
  Flist^[Index].FObject:=AObject;
  Changed;
end;



procedure TStringList.SetCapacity(NewCapacity: Integer);

Var NewList : Pointer;
    MSize : Longint;

begin
  If (NewCapacity<0) then
     Error (SListCapacityError,NewCapacity);
  If NewCapacity>FCapacity then
    begin
    GetMem (NewList,NewCapacity*SizeOf(TStringItem));
    If NewList=Nil then
      Error (SListCapacityError,NewCapacity);
    If Assigned(FList) then
      begin
      MSize:=FCapacity*Sizeof(TStringItem);
      System.Move (FList^,NewList^,MSize);
      FillWord (Pchar(NewList)[MSize],(NewCapacity-FCapacity)*(SizeOf(TStringItem) div SizeOf(Word)), 0);
      FreeMem (Flist,MSize);
      end;
    Flist:=NewList;
    FCapacity:=NewCapacity;
    end
  else if NewCapacity<FCapacity then
    begin
    if NewCapacity = 0 then
    begin
      if FCount > 0 then
        InternalClear(0,True);
      FreeMem(FList);
      FList := nil;
    end else
    begin
      InternalClear(NewCapacity,True);
      GetMem(NewList, NewCapacity * SizeOf(TStringItem));
      System.Move(FList^, NewList^, NewCapacity * SizeOf(TStringItem));
      FreeMem(FList);
      FList := NewList;
    end;
    FCapacity:=NewCapacity;
    end;
end;



procedure TStringList.SetUpdateState(Updating: Boolean);

begin
  If Updating then
    Changing
  else
    Changed
end;



destructor TStringList.Destroy;

begin
  InternalClear;
  Inherited destroy;
end;



function TStringList.Add(const S: string): Integer;

begin
  If (SortStyle<>sslAuto) then
    Result:=FCount
  else
    If Find (S,Result) then
      Case DUplicates of
        DupIgnore : Exit;
        DupError : Error(SDuplicateString,0)
      end;
   InsertItem (Result,S);
end;

procedure TStringList.Clear;

begin
  if FCount = 0 then Exit;
  Changing;
  InternalClear;
  Changed;
end;

procedure TStringList.Delete(Index: Integer);

begin
  CheckIndex(Index);
  Changing;
  Flist^[Index].FString:='';
  if FOwnsObjects then
    FreeAndNil(Flist^[Index].FObject);
  Dec(FCount);
  If Index<FCount then
    System.Move(Flist^[Index+1],
                Flist^[Index],
                (Fcount-Index)*SizeOf(TStringItem));
  Changed;
end;



procedure TStringList.Exchange(Index1, Index2: Integer);

begin
  CheckIndex(Index1);
  CheckIndex(Index2);
  Changing;
  ExchangeItemsInt(Index1,Index2);
  changed;
end;


procedure TStringList.SetCaseSensitive(b : boolean);
begin
  if b=FCaseSensitive then
    Exit;
  FCaseSensitive:=b;
  if FSortStyle=sslAuto then
    begin
    FForceSort:=True;
    try
      Sort;
    finally
      FForceSort:=False;
    end;
    end;
end;

procedure TStringList.SetSortStyle(AValue: TStringsSortStyle);
begin
  if FSortStyle=AValue then Exit;
  if (AValue=sslAuto) then
    Sort;
  FSortStyle:=AValue;
end;

procedure TStringList.CheckIndex(AIndex: Integer);
begin
  If (AIndex<0) or (AIndex>=FCount) then
    Error(SListIndexError,AIndex);
end;


function TStringList.DoCompareText(const s1, s2: string): PtrInt;
begin
  if FCaseSensitive then
  begin
    if UseLocale then
      result:=AnsiCompareStr(s1,s2)
    else
      result:=CompareStr(s1,s2);
  end else
  begin
    if UseLocale then
      result:=AnsiCompareText(s1,s2)
    else
      result:=CompareText(s1,s2);
  end;
end;


function TStringList.Find(const S: string; out Index: Integer): Boolean;

var
  L, R, I: Integer;
  CompareRes: PtrInt;
begin
  Result := false;
  Index:=-1;
  if Not Sorted then
    Raise EListError.Create(SErrFindNeedsSortedList);
  // Use binary search.
  L := 0;
  R := Count - 1;
  while (L<=R) do
  begin
    I := L + (R - L) div 2;
    CompareRes := DoCompareText(S, Flist^[I].FString);
    if (CompareRes>0) then
      L := I+1
    else begin
      R := I-1;
      if (CompareRes=0) then begin
         Result := true;
         if (Duplicates<>dupAccept) then
            L := I; // forces end of while loop
      end;
    end;
  end;
  Index := L;
end;



function TStringList.IndexOf(const S: string): Integer;

begin
  If Not Sorted then
    Result:=Inherited indexOf(S)
  else
    // faster using binary search...
    If Not Find (S,Result) then
      Result:=-1;
end;



procedure TStringList.Insert(Index: Integer; const S: string);

begin
  If SortStyle=sslAuto then
    Error (SSortedListError,0)
  else
    begin
    If (Index<0) or (Index>FCount) then
      Error(SListIndexError,Index); // Cannot use CheckIndex, because there >= FCount...
    InsertItem (Index,S);
    end;
end;


procedure TStringList.CustomSort(CompareFn: TStringListSortCompare);

begin
  CustomSort(CompareFn, SortBase.DefaultSortingAlgorithm);
end;

type
  PStringList_CustomSort_Context = ^TStringList_CustomSort_Context;
  TStringList_CustomSort_Context = record
    List: TStringList;
    ListStartPtr: Pointer;
    CompareFn: TStringListSortCompare;
  end;

function TStringList_CustomSort_Comparer(Item1, Item2, Context: Pointer): Integer;
begin
  with PStringList_CustomSort_Context(Context)^ do
    Result := CompareFn(List,
                        (Item1 - ListStartPtr) div SizeOf(TStringItem),
                        (Item2 - ListStartPtr) div SizeOf(TStringItem));
end;

procedure TStringList_CustomSort_Exchanger(Item1, Item2, Context: Pointer);
begin
  with PStringList_CustomSort_Context(Context)^ do
    List.ExchangeItems((Item1 - ListStartPtr) div SizeOf(TStringItem),
                       (Item2 - ListStartPtr) div SizeOf(TStringItem));
end;

procedure TStringList.CustomSort(CompareFn: TStringListSortCompare; SortingAlgorithm: PSortingAlgorithm);
var
  Context: TStringList_CustomSort_Context;
begin
  If (FCount>1) and (FForceSort or (FSortStyle<>sslAuto))  then
    begin
    Changing;

    Context.List := Self;
    Context.ListStartPtr := FList;
    Context.CompareFn := CompareFn;

    //if ExchangeItems is overriden call that, else call (faster) ItemListSorter_ContextComparer
    if TMethod(@Self.ExchangeItems).Code = CodePointer(@TStringList.ExchangeItems) then
      SortingAlgorithm^.ItemListSorter_ContextComparer(
        FList, FCount, SizeOf(TStringItem), @TStringList_CustomSort_Comparer,
        @Context)
    else
      SortingAlgorithm^.ItemListSorter_CustomItemExchanger_ContextComparer(
        FList, FCount, SizeOf(TStringItem), @TStringList_CustomSort_Comparer,
        @TStringList_CustomSort_Exchanger, @Context);

    Changed;
    end;
end;

function StringListAnsiCompare(List: TStringList; Index1, Index: Integer): Integer;

begin
  Result := List.DoCompareText(List.FList^[Index1].FString,
    List.FList^[Index].FString);
end;

procedure TStringList.Sort;

begin
  CustomSort(@StringListAnsiCompare);
end;

procedure TStringList.Sort(SortingAlgorithm: PSortingAlgorithm);

begin
  CustomSort(@StringListAnsiCompare, SortingAlgorithm);
end;

{$else}

{ generics based implementation of TStringList follows }

function StringListAnsiCompare(List: TStringList; Index1, Index2: Integer): Integer;
begin
  Result := List.DoCompareText(List.Strings[Index1], List.Strings[Index2]);
end;

constructor TStringList.Create;
begin
  inherited;
  FOwnsObjects:=false;
  FMap := TFPStrObjMap.Create;
  FMap.OnPtrCompare := @MapPtrCompare;
  FOnCompareText := @DefaultCompareText;
  NameValueSeparator:='=';
  CheckSpecialChars;
end;

destructor TStringList.Destroy;
begin
  FMap.Free;
  inherited;
end;

function TStringList.GetDuplicates: TDuplicates;
begin
  Result := FMap.Duplicates;
end;

function TStringList.GetSorted: boolean;
begin
  Result := FMap.Sorted;
end;

procedure TStringList.SetDuplicates(NewDuplicates: TDuplicates);
begin
  FMap.Duplicates := NewDuplicates;
end;

procedure TStringList.SetSorted(NewSorted: Boolean);
begin
  FMap.Sorted := NewSorted;
end;

procedure TStringList.Changed;
begin
  if FUpdateCount = 0 then
   if Assigned(FOnChange) then
     FOnChange(Self);
end;

procedure TStringList.Changing;
begin
  if FUpdateCount = 0 then
    if Assigned(FOnChanging) then
      FOnChanging(Self);
end;

function TStringList.Get(Index: Integer): string;
begin
  Result := FMap.Keys[Index];
end;

function TStringList.GetCapacity: Integer;
begin
  Result := FMap.Capacity;
end;

function TStringList.GetCount: Integer;
begin
  Result := FMap.Count;
end;

function TStringList.GetObject(Index: Integer): TObject;
begin
  Result := FMap.Data[Index];
end;

procedure TStringList.Put(Index: Integer; const S: string);
begin
  Changing;
  FMap.Keys[Index] := S;
  Changed;
end;

procedure TStringList.PutObject(Index: Integer; AObject: TObject);
begin
  Changing;
  FMap.Data[Index] := AObject;
  Changed;
end;

procedure TStringList.SetCapacity(NewCapacity: Integer);
begin
  FMap.Capacity := NewCapacity;
end;

procedure TStringList.SetUpdateState(Updating: Boolean);
begin
  if Updating then
    Changing
  else
    Changed
end;

function TStringList.Add(const S: string): Integer;
begin
  Result := FMap.Add(S);
end;

procedure TStringList.Clear;
begin
  if FMap.Count = 0 then exit;
  Changing;
  FMap.Clear;
  Changed;
end;

procedure TStringList.Delete(Index: Integer);
begin
  if (Index < 0) or (Index >= FMap.Count) then
    Error(SListIndexError, Index);
  Changing;
  FMap.Delete(Index);
  Changed;
end;

procedure TStringList.Exchange(Index1, Index2: Integer);
begin
  if (Index1 < 0) or (Index1 >= FMap.Count) then
    Error(SListIndexError, Index1);
  if (Index2 < 0) or (Index2 >= FMap.Count) then
    Error(SListIndexError, Index2);
  Changing;
  FMap.InternalExchange(Index1, Index2);
  Changed;
end;

procedure TStringList.SetCaseSensitive(NewSensitive: Boolean);
begin
  if NewSensitive <> FCaseSensitive then
  begin
    FCaseSensitive := NewSensitive;
    if Sorted then
      Sort;
  end;
end;

function TStringList.MapPtrCompare(Key1, Key2: Pointer): Integer;
begin
  Result := FOnCompareText(string(Key1^), string(Key2^));
end;

function TStringList.DefaultCompareText(const s1, s2: string): PtrInt;
begin
  if FCaseSensitive then
    Result := AnsiCompareStr(s1, s2)
  else
    Result := AnsiCompareText(s1, s2);
end;

function TStringList.DoCompareText(const s1, s2: string): PtrInt;
begin
  Result := FOnCompareText(s1, s2);
end;

function TStringList.Find(const S: string; var Index: Integer): Boolean;
begin
  Result := FMap.Find(S, Index);
end;

function TStringList.IndexOf(const S: string): Integer;
begin
  Result := FMap.IndexOf(S);
end;

procedure TStringList.Insert(Index: Integer; const S: string);
begin
  if not Sorted and (0 <= Index) and (Index < FMap.Count) then
    Changing;
  FMap.InsertKey(Index, S);
  Changed;
end;

type
  PStringList_CustomSort_Context = ^TStringList_CustomSort_Context;
  TStringList_CustomSort_Context = record
    List: TStringList;
    ListStartPtr: Pointer;
    ItemSize: SizeUInt;
    IndexBase: Integer;
    CompareFn: TStringListSortCompare;
  end;

function TStringList_CustomSort_Comparer(Item1, Item2, Context: Pointer): Integer;
begin
  with PStringList_CustomSort_Context(Context)^ do
    Result := CompareFn(List,
                        ((Item1 - ListStartPtr) div ItemSize) + IndexBase,
                        ((Item2 - ListStartPtr) div ItemSize) + IndexBase);
end;

procedure TStringList_CustomSort_Exchanger(Item1, Item2, Context: Pointer);
begin
  with PStringList_CustomSort_Context(Context)^ do
    List.Exchange(((Item1 - ListStartPtr) div ItemSize) + IndexBase,
                  ((Item2 - ListStartPtr) div ItemSize) + IndexBase);
end;

procedure TStringList.QuickSort(L, R: Integer; CompareFn: TStringListSortCompare);
var
  Context: TStringList_CustomSort_Context;
begin
  if L > R then
    exit;
  Context.List := Self;
  Context.ListStartPtr := FMap.Items[L];
  Context.CompareFn := CompareFn;
  Context.ItemSize := FMap.KeySize + FMap.DataSize;
  Context.IndexBase := L;

  DefaultSortingAlgorithm^.ItemListSorter_CustomItemExchanger_ContextComparer(
    Context.ListStartPtr, R - L + 1, Context.ItemSize, @TStringList_CustomSort_Comparer,
    @TStringList_CustomSort_Exchanger, @Context);
end;

procedure TStringList.CustomSort(CompareFn: TStringListSortCompare);
begin
  if not Sorted and (FMap.Count > 1) then
  begin
    Changing;
    QuickSort(0, FMap.Count-1, CompareFn);
    Changed;
  end;
end;

procedure TStringList.CustomSort(CompareFn: TStringListSortCompare; SortingAlgorithm: PSortingAlgorithm);
var
  Context: TStringList_CustomSort_Context;
begin
  if not Sorted and (FMap.Count > 1) then
  begin
    Changing;

    Context.List := Self;
    Context.ListStartPtr := FMap.Items[0];
    Context.CompareFn := CompareFn;
    Context.ItemSize := FMap.KeySize + FMap.DataSize;
    Context.IndexBase := 0;

    SortingAlgorithm^.ItemListSorter_CustomItemExchanger_ContextComparer(
      Context.ListStartPtr, FMap.Count, Context.ItemSize, @TStringList_CustomSort_Comparer,
      @TStringList_CustomSort_Exchanger, @Context);

    Changed;
  end;
end;

procedure TStringList.Sort;
begin
  if not Sorted and (FMap.Count > 1) then
  begin
    Changing;
    FMap.Sort;
    Changed;
  end;
end;

procedure TStringList.Sort(SortingAlgorithm: PSortingAlgorithm);
begin
  if not Sorted and (FMap.Count > 1) then
  begin
    Changing;
    FMap.Sort(SortingAlgorithm);
    Changed;
  end;
end;

{$endif}

