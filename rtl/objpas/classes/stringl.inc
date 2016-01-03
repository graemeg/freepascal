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
<<<<<<< HEAD
<<<<<<< HEAD
    FLineBreak:=sLineBreak;
    end;
end;

Function TStrings.GetSkipLastLineBreak : Boolean;

begin
  CheckSpecialChars;
  Result:=FSkipLastLineBreak;
end;

procedure TStrings.SetSkipLastLineBreak(const AValue : Boolean);

begin
  CheckSpecialChars;
  FSkipLastLineBreak:=AValue;
end;

=======
=======
>>>>>>> origin/fixes_2_2
    FLBS:=DefaultTextLineBreakStyle;
    end;
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

<<<<<<< HEAD
<<<<<<< HEAD
Function TStrings.GetQuoteChar :Char;
begin
  CheckSpecialChars;
  Result:=FQuoteChar;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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


Function TStrings.GetDelimitedText: string;

Var
  I : integer;
  p : pchar;
<<<<<<< HEAD
<<<<<<< HEAD
  BreakChars : set of char;
=======
  c : set of char;
>>>>>>> graemeg/fixes_2_2
=======
  c : set of char;
>>>>>>> origin/fixes_2_2
  S : String;
  
begin
  CheckSpecialChars;
  result:='';
  if StrictDelimiter then
<<<<<<< HEAD
<<<<<<< HEAD
    BreakChars:=[#0,QuoteChar,Delimiter]
  else  
    BreakChars:=[#0..' ',QuoteChar,Delimiter];

  // Check for break characters and quote if required.
=======
    c:=[#0,Delimiter]
  else  
    c:=[#0..' ',QuoteChar,Delimiter];
>>>>>>> graemeg/fixes_2_2
=======
    c:=[#0,Delimiter]
  else  
    c:=[#0..' ',QuoteChar,Delimiter];
>>>>>>> origin/fixes_2_2
  For i:=0 to count-1 do
    begin
    S:=Strings[i];
    p:=pchar(S);
<<<<<<< HEAD
<<<<<<< HEAD
    //Quote strings that include BreakChars:
    while not(p^ in BreakChars) do
     inc(p);
    if (p<>pchar(S)+length(S)) then
=======
    while not(p^ in c) do
     inc(p);
// strings in list may contain #0
    if (p<>pchar(S)+length(S)) and not StrictDelimiter then
>>>>>>> graemeg/fixes_2_2
=======
    while not(p^ in c) do
     inc(p);
// strings in list may contain #0
    if (p<>pchar(S)+length(S)) and not StrictDelimiter then
>>>>>>> origin/fixes_2_2
      Result:=Result+QuoteString(S,QuoteChar)
    else
      Result:=Result+S;
    if I<Count-1 then 
      Result:=Result+Delimiter;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  // Quote empty string:
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  If (Length(Result)=0) and (Count=1) then
    Result:=QuoteChar+QuoteChar;
end;

procedure TStrings.GetNameValue(Index : Integer; Out AName,AValue : String);

Var L : longint;

begin
  CheckSpecialChars;
  AValue:=Strings[Index];
  L:=Pos(FNameValueSeparator,AValue);
  If L<>0 then
    begin
    AName:=Copy(AValue,1,L-1);
    System.Delete(AValue,1,L);
    end
  else
    AName:='';
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

function TStrings.GetName(Index: Integer): string;

Var
  V : String;

begin
  GetNameValue(Index,Result,V);
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

<<<<<<< HEAD

Procedure TStrings.SetDelimitedText(const AValue: string);
<<<<<<< HEAD
=======

  Procedure  SetStrictDelimited;
  
  Var
    P,PL : Integer;
    
  begin
    PL:=1;
    P:=1;
{
    While Length(S)>0 do
      begin
      P:=Pos(FDelimiter,S);
      If (P=0) then
        P:=Length(S)+1;
      Add(Copy(S,1,P-1));
      System.Delete(S,1,P));
      end;
}  end;  
    

>>>>>>> graemeg/fixes_2_2
var i,j:integer;
    aNotFirst:boolean;
begin
 CheckSpecialChars;
 BeginUpdate;
<<<<<<< HEAD

 i:=1;
 j:=1;
 aNotFirst:=false;

 { Paraphrased from Delphi XE2 help:
 Strings must be separated by Delimiter characters or spaces.
 They may be enclosed in QuoteChars.
 QuoteChars in the string must be repeated to distinguish them from the QuoteChars enclosing the string.
 }
 try
  Clear;
  If StrictDelimiter then
    begin
    while i<=length(AValue) do begin
     // skip delimiter
     if aNotFirst and (i<=length(AValue)) and (AValue[i]=FDelimiter) then inc(i);

     // read next string
     if i<=length(AValue) then begin
      if AValue[i]=FQuoteChar then begin
       // next string is quoted
       j:=i+1;
       while (j<=length(AValue)) and
             ( (AValue[j]<>FQuoteChar) or
               ( (j+1<=length(AValue)) and (AValue[j+1]=FQuoteChar) ) ) do begin
        if (j<=length(AValue)) and (AValue[j]=FQuoteChar) then inc(j,2)
                                                          else inc(j);
       end;
       // j is position of closing quote
       Add( StringReplace (Copy(AValue,i+1,j-i-1),
                           FQuoteChar+FQuoteChar,FQuoteChar, [rfReplaceAll]));
       i:=j+1;
      end else begin
       // next string is not quoted; read until delimiter
       j:=i;
       while (j<=length(AValue)) and
             (AValue[j]<>FDelimiter) do inc(j);
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
    while i<=length(AValue) do begin
     // skip delimiter
     if aNotFirst and (i<=length(AValue)) and (AValue[i]=FDelimiter) then inc(i);

     // skip spaces
     while (i<=length(AValue)) and (Ord(AValue[i])<=Ord(' ')) do inc(i);
    
     // read next string
     if i<=length(AValue) then begin
      if AValue[i]=FQuoteChar then begin
       // next string is quoted
       j:=i+1;
       while (j<=length(AValue)) and
             ( (AValue[j]<>FQuoteChar) or
               ( (j+1<=length(AValue)) and (AValue[j+1]=FQuoteChar) ) ) do begin
        if (j<=length(AValue)) and (AValue[j]=FQuoteChar) then inc(j,2)
                                                          else inc(j);
       end;
       // j is position of closing quote
       Add( StringReplace (Copy(AValue,i+1,j-i-1),
                           FQuoteChar+FQuoteChar,FQuoteChar, [rfReplaceAll]));
       i:=j+1;
      end else begin
       // next string is not quoted; read until control character/space/delimiter
       j:=i;
       while (j<=length(AValue)) and
             (Ord(AValue[j])>Ord(' ')) and
             (AValue[j]<>FDelimiter) do inc(j);
       Add( Copy(AValue,i,j-i));
       i:=j;
      end;
     end else begin
      if aNotFirst then Add('');
     end;

     // skip spaces
     while (i<=length(AValue)) and (Ord(AValue[i])<=Ord(' ')) do inc(i);

=======

 i:=1;
 j:=1;
 aNotFirst:=false;

=======

Procedure TStrings.SetDelimitedText(const AValue: string);

  Procedure  SetStrictDelimited;
  
  Var
    P,PL : Integer;
    
  begin
    PL:=1;
    P:=1;
{
    While Length(S)>0 do
      begin
      P:=Pos(FDelimiter,S);
      If (P=0) then
        P:=Length(S)+1;
      Add(Copy(S,1,P-1));
      System.Delete(S,1,P));
      end;
}  end;  
    

var i,j:integer;
    aNotFirst:boolean;
begin
 CheckSpecialChars;
 BeginUpdate;

 i:=1;
 j:=1;
 aNotFirst:=false;

>>>>>>> origin/fixes_2_2
 try
  Clear;
  If StrictDelimiter then
    begin
    // Easier, faster loop.
    While I<=Length(AValue) do
      begin
      If (AValue[I] in [FDelimiter,#0]) then
        begin
        Add(Copy(AValue,J,I-J));
        J:=I+1;
        end;
      Inc(i);
      end;
    If (Length(AValue)>0) then
      Add(Copy(AValue,J,I-J));  
    end
  else 
    begin
    while i<=length(AValue) do begin
     // skip delimiter
     if aNotFirst and (i<=length(AValue)) and (AValue[i]=FDelimiter) then inc(i);

     // skip spaces
     while (i<=length(AValue)) and (Ord(AValue[i])<=Ord(' ')) do inc(i);
    
     // read next string
     if i<=length(AValue) then begin
      if AValue[i]=FQuoteChar then begin
       // next string is quoted
       j:=i+1;
       while (j<=length(AValue)) and
             ( (AValue[j]<>FQuoteChar) or
               ( (j+1<=length(AValue)) and (AValue[j+1]=FQuoteChar) ) ) do begin
        if (j<=length(AValue)) and (AValue[j]=FQuoteChar) then inc(j,2)
                                                          else inc(j);
       end;
       // j is position of closing quote
       Add( StringReplace (Copy(AValue,i+1,j-i-1),
                           FQuoteChar+FQuoteChar,FQuoteChar, [rfReplaceAll]));
       i:=j+1;
      end else begin
       // next string is not quoted
       j:=i;
       while (j<=length(AValue)) and
             (Ord(AValue[j])>Ord(' ')) and
             (AValue[j]<>FDelimiter) do inc(j);
       Add( Copy(AValue,i,j-i));
       i:=j;
      end;
     end else begin
      if aNotFirst then Add('');
     end;

     // skip spaces
     while (i<=length(AValue)) and (Ord(AValue[i])<=Ord(' ')) do inc(i);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
     aNotFirst:=true;
    end;
    end;
 finally
   EndUpdate;
 end;
end;

Procedure TStrings.SetCommaText(const Value: string);

Var
  C1,C2 : Char;

begin
  CheckSpecialChars;
  C1:=Delimiter;
  C2:=QuoteChar;
  Delimiter:=',';
  QuoteChar:='"';
  Try
    SetDelimitedText(Value);
  Finally
    Delimiter:=C1;
    QuoteChar:=C2;
  end;
end;


Procedure TStrings.SetStringsAdapter(const Value: IStringsAdapter);

begin
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
    I,L,NLS : Longint;
    S,NL : String;

begin
  CheckSpecialChars;
  // Determine needed place
<<<<<<< HEAD
<<<<<<< HEAD
  if FLineBreak<>sLineBreak then
    NL:=FLineBreak
  else
    Case FLBS of
      tlbsLF   : NL:=#10;
      tlbsCRLF : NL:=#13#10;
      tlbsCR   : NL:=#13;
    end;
=======
=======
>>>>>>> origin/fixes_2_2
  Case FLBS of
    tlbsLF   : NL:=#10;
    tlbsCRLF : NL:=#13#10;
    tlbsCR   : NL:=#13; 
  end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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

Class Function TStrings.GetNextLine (Const Value : String; Var S : String; Var P : Integer) : Boolean;

Var 
  PS : PChar;
  IP,L : Integer;
  
begin
  L:=Length(Value);
  S:='';
  Result:=False;
  If ((L-P)<0) then 
    exit;
  if ((L-P)=0) and (not (value[P] in [#10,#13])) Then
    Begin
      s:=value[P];
      inc(P);
      Exit(True);
    End;
  PS:=PChar(Value)+P-1;
  IP:=P;
  While ((L-P)>=0) and (not (PS^ in [#10,#13])) do 
    begin
    P:=P+1;
    Inc(PS);
    end;
  SetLength (S,P-IP);
  System.Move (Value[IP],Pointer(S)^,P-IP);
  If (P<=L) and (Value[P]=#13) then 
    Inc(P);
  If (P<=L) and (Value[P]=#10) then
    Inc(P); // Point to character after #10(#13)
  Result:=True;
end;

Function TStrings.GetNextLineBreak (Const Value : String; Var S : String; Var P : Integer) : Boolean;

Var
  PS,PC,PP : PChar;

begin
  S:='';
  Result:=False;
  If ((Length(Value)-P)<0) then
    exit;
  PS:=@Value[P];
  PC:=PS;
  PP:=AnsiStrPos(PS,PChar(FLineBreak));
  // Stop on #0.
  While (PC^<>#0) and (PC<>PP) do
    Inc(PC);
  P:=P+(PC-PS)+Length(FLineBreak);
  SetString(S,PS,PC-PS);
  Result:=True;
end;

Procedure TStrings.DoSetTextStr(const Value: string; DoClear : Boolean);

Var
  S : String;
  P : Integer;

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

Procedure TStrings.SetUpdateState(Updating: Boolean);

begin
  FPONotifyObservers(Self,ooChange,Nil);
end;



destructor TSTrings.Destroy;

begin
  inherited destroy;
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


Procedure TStrings.Append(const S: string);

begin
  Add (S);
end;



Procedure TStrings.AddStrings(TheStrings: TStrings);

Var Runner : longint;

begin
  beginupdate;
  try
    For Runner:=0 to TheStrings.Count-1 do
      self.AddObject (Thestrings[Runner],TheStrings.Objects[Runner]);
  finally
    EndUpdate;
  end;
end;

Procedure TStrings.AddStrings(const TheStrings: array of string);

Var Runner : longint;

begin
  try
    beginupdate;
    if Count + High(TheStrings)+1 > Capacity then
      Capacity := Count + High(TheStrings)+1;
    For Runner:=Low(TheStrings) to High(TheStrings) do
      self.Add(Thestrings[Runner]);
  finally
    EndUpdate;
  end;
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
<<<<<<< HEAD
<<<<<<< HEAD
      FLineBreak:=S.FLineBreak;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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
    result:=CompareText(s1,s2);
  end;


Function TStrings.IndexOf(const S: string): Integer;
begin
  Result:=0;
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



Procedure TStrings.LoadFromFile(const FileName: string);
Var
        TheStream : TFileStream;
begin
  TheStream:=TFileStream.Create(FileName,fmOpenRead or fmShareDenyWrite);
<<<<<<< HEAD
  try
    LoadFromStream(TheStream);
  finally
    TheStream.Free;
  end;
=======
  LoadFromStream(TheStream);
  TheStream.Free;
>>>>>>> origin/fixes_2.4
end;



Procedure TStrings.LoadFromStream(Stream: TStream);
{
   Borlands method is no good, since a pipe for
   instance doesn't have a size.
   So we must do it the hard way.
}
Const
  BufSize = 1024;
  MaxGrow = 1 shl 29;

Var
  Buffer     : AnsiString;
  BytesRead,
  BufLen,
  I,BufDelta     : Longint;
begin
  // reread into a buffer
  beginupdate;
  try
    Buffer:='';
    BufLen:=0;
    I:=1;
    Repeat
      BufDelta:=BufSize*I;
      SetLength(Buffer,BufLen+BufDelta);
      BytesRead:=Stream.Read(Buffer[BufLen+1],BufDelta);
      inc(BufLen,BufDelta);
      If I<MaxGrow then
        I:=I shl 1;
    Until BytesRead<>BufDelta;
    SetLength(Buffer, BufLen-BufDelta+BytesRead);
    SetTextStr(Buffer);
    SetLength(Buffer,0);
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



Procedure TStrings.SaveToStream(Stream: TStream);
Var
  S : String;
begin
  S:=Text;
<<<<<<< HEAD
<<<<<<< HEAD
  if S = '' then Exit;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Stream.WriteBuffer(Pointer(S)^,Length(S));
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

<<<<<<< HEAD
<<<<<<< HEAD
{$if not defined(FPC_TESTGENERICS)}

Procedure TStringList.ExchangeItemsInt(Index1, Index2: Integer);
=======
=======
>>>>>>> origin/fixes_2_2
{$if defined(VER2_0) or not defined(FPC_TESTGENERICS)}

Procedure TStringList.ExchangeItems(Index1, Index2: Integer);
>>>>>>> graemeg/fixes_2_2

Var P1,P2 : Pointer;

begin
  P1:=Pointer(Flist^[Index1].FString);
  P2:=Pointer(Flist^[Index1].FObject);
  Pointer(Flist^[Index1].Fstring):=Pointer(Flist^[Index2].Fstring);
  Pointer(Flist^[Index1].FObject):=Pointer(Flist^[Index2].FObject);
  Pointer(Flist^[Index2].Fstring):=P1;
  Pointer(Flist^[Index2].FObject):=P2;
end;


Procedure TStringList.ExchangeItems(Index1, Index2: Integer);
begin
  ExchangeItemsInt(Index1, Index2);
end;


Procedure TStringList.Grow;

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

Procedure TStringList.InternalClear(FromIndex : Integer = 0; ClearOnly : Boolean = False);

<<<<<<< HEAD
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

Procedure TStringList.QuickSort(L, R: Integer; CompareFn: TStringListSortCompare);
var
  Pivot, vL, vR: Integer;
<<<<<<< HEAD
  ExchangeProc: procedure(Left, Right: Integer) of object;
begin
  //if ExchangeItems is override call that, else call (faster) ExchangeItemsInt
  if TMethod(@Self.ExchangeItems).Code = CodePointer(@TStringList.ExchangeItems) then
    ExchangeProc := @ExchangeItemsInt
  else
    ExchangeProc := @ExchangeItems;

  if R - L <= 1 then begin // a little bit of time saver
    if L < R then
      if CompareFn(Self, L, R) > 0 then
        ExchangeProc(L, R);
=======
begin
  if R - L <= 1 then begin // a little bit of time saver
    if L < R then
      if CompareFn(Self, L, R) > 0 then
        ExchangeItems(L, R);
>>>>>>> origin/fixes_2_2

    Exit;
  end;

<<<<<<< HEAD
=======

Procedure TStringList.QuickSort(L, R: Integer; CompareFn: TStringListSortCompare);
var
  Pivot, vL, vR: Integer;
begin
  if R - L <= 1 then begin // a little bit of time saver
    if L < R then
      if CompareFn(Self, L, R) > 0 then
        ExchangeItems(L, R);

    Exit;
  end;

>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  vL := L;
  vR := R;

  Pivot := L + Random(R - L); // they say random is best

  while vL < vR do begin
    while (vL < Pivot) and (CompareFn(Self, vL, Pivot) <= 0) do
      Inc(vL);

    while (vR > Pivot) and (CompareFn(Self, vR, Pivot) > 0) do
      Dec(vR);

<<<<<<< HEAD
<<<<<<< HEAD
    ExchangeProc(vL, vR);
=======
    ExchangeItems(vL, vR);
>>>>>>> graemeg/fixes_2_2
=======
    ExchangeItems(vL, vR);
>>>>>>> origin/fixes_2_2

    if Pivot = vL then // swap pivot if we just hit it from one side
      Pivot := vR
    else if Pivot = vR then
      Pivot := vL;
  end;

  if Pivot - 1 >= L then
    QuickSort(L, Pivot - 1, CompareFn);
  if Pivot + 1 <= R then
    QuickSort(Pivot + 1, R, CompareFn);
end;


Procedure TStringList.InsertItem(Index: Integer; const S: string);
begin
  InsertItem(Index, S, nil);
end;


Procedure TStringList.InsertItem(Index: Integer; const S: string; O: TObject);
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


Procedure TStringList.SetSorted(Value: Boolean);

begin
  If FSorted<>Value then
    begin
    If Value then sort;
    FSorted:=VAlue
    end;
end;



Procedure TStringList.Changed;

begin
  If (FUpdateCount=0) Then
   begin
   If Assigned(FOnChange) then
     FOnchange(Self);
   FPONotifyObservers(Self,ooChange,Nil);
   end;
end;



Procedure TStringList.Changing;

begin
  If FUpdateCount=0 then
    if Assigned(FOnChanging) then
      FOnchanging(Self);
end;



Function TStringList.Get(Index: Integer): string;

begin
  If (Index<0) or (INdex>=Fcount)  then
    Error (SListIndexError,Index);
  Result:=Flist^[Index].FString;
end;



Function TStringList.GetCapacity: Integer;

begin
  Result:=FCapacity;
end;



Function TStringList.GetCount: Integer;

begin
  Result:=FCount;
end;



Function TStringList.GetObject(Index: Integer): TObject;

begin
  If (Index<0) or (INdex>=Fcount)  then
    Error (SListIndexError,Index);
  Result:=Flist^[Index].FObject;
end;



Procedure TStringList.Put(Index: Integer; const S: string);

begin
  If Sorted then
    Error(SSortedListError,0);
  If (Index<0) or (INdex>=Fcount)  then
    Error (SListIndexError,Index);
  Changing;
  Flist^[Index].FString:=S;
  Changed;
end;



Procedure TStringList.PutObject(Index: Integer; AObject: TObject);

begin
  If (Index<0) or (INdex>=Fcount)  then
    Error (SListIndexError,Index);
  Changing;
  Flist^[Index].FObject:=AObject;
  Changed;
end;



Procedure TStringList.SetCapacity(NewCapacity: Integer);

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



Procedure TStringList.SetUpdateState(Updating: Boolean);

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



Function TStringList.Add(const S: string): Integer;

begin
  If Not Sorted then
    Result:=FCount
  else
    If Find (S,Result) then
      Case DUplicates of
        DupIgnore : Exit;
        DupError : Error(SDuplicateString,0)
      end;
   InsertItem (Result,S);
end;

Procedure TStringList.Clear;

begin
  if FCount = 0 then Exit;
  Changing;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  InternalClear;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  if FOwnsObjects then
    begin
      For I:=0 to FCount-1 do
        begin
          Flist^[I].FString:='';
          freeandnil(Flist^[i].FObject);
        end;
    end
  else
    begin
      For I:=0 to FCount-1 do
        Flist^[I].FString:='';
    end;
  FCount:=0;
  SetCapacity(0);
>>>>>>> graemeg/cpstrnew
  Changed;
end;

Procedure TStringList.Delete(Index: Integer);

begin
  If (Index<0) or (Index>=FCount) then
    Error(SlistINdexError,Index);
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



Procedure TStringList.Exchange(Index1, Index2: Integer);

begin
  If (Index1<0) or (Index1>=FCount) then
    Error(SListIndexError,Index1);
  If (Index2<0) or (Index2>=FCount) then
    Error(SListIndexError,Index2);
  Changing;
  ExchangeItemsInt(Index1,Index2);
  changed;
end;


procedure TStringList.SetCaseSensitive(b : boolean);
begin
  if b=FCaseSensitive then
    Exit;
  FCaseSensitive:=b;
  if FSorted then
    begin
    FForceSort:=True;
    sort;
    FForceSort:=False;
    end;
end;


Function TStringList.DoCompareText(const s1,s2 : string) : PtrInt;
  begin
        if FCaseSensitive then
          result:=AnsiCompareStr(s1,s2)
        else
          result:=AnsiCompareText(s1,s2);
  end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TStringList.CompareStrings(const s1,s2 : string) : Integer;
begin
  Result := DoCompareText(s1, s2);
end;

<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Function TStringList.Find(const S: string; Out Index: Integer): Boolean;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
var
  L, R, I: Integer;
  CompareRes: PtrInt;
begin
  Result := false;
<<<<<<< HEAD
<<<<<<< HEAD
  if Not Sorted then 
    exit;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
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



Function TStringList.IndexOf(const S: string): Integer;

begin
  If Not Sorted then
    Result:=Inherited indexOf(S)
  else
    // faster using binary search...
    If Not Find (S,Result) then
      Result:=-1;
end;



Procedure TStringList.Insert(Index: Integer; const S: string);

begin
  If Sorted then
    Error (SSortedListError,0)
  else
    If (Index<0) or (Index>FCount) then
      Error (SListIndexError,Index)
    else
      InsertItem (Index,S);
end;


Procedure TStringList.CustomSort(CompareFn: TStringListSortCompare);

begin
  If (FForceSort or (Not Sorted)) and (FCount>1) then
    begin
    Changing;
    QuickSort(0,FCount-1, CompareFn);
    Changed;
    end;
end;

function StringListAnsiCompare(List: TStringList; Index1, Index: Integer): Integer;

begin
  Result := List.DoCompareText(List.FList^[Index1].FString,
    List.FList^[Index].FString);
end;

Procedure TStringList.Sort;

begin
  CustomSort(@StringListAnsiCompare);
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
<<<<<<< HEAD
<<<<<<< HEAD
  FOwnsObjects:=false;
  FMap := TFPStrObjMap.Create;
  FMap.OnPtrCompare := @MapPtrCompare;
  FOnCompareText := @DefaultCompareText;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  NameValueSeparator:='=';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  CheckSpecialChars;
=======
  FMap := TFPStrObjMap.Create;
  FMap.OnPtrCompare := @MapPtrCompare;
  FOnCompareText := @DefaultCompareText;
>>>>>>> graemeg/fixes_2_2
=======
  FMap := TFPStrObjMap.Create;
  FMap.OnPtrCompare := @MapPtrCompare;
  FOnCompareText := @DefaultCompareText;
>>>>>>> origin/fixes_2_2
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

procedure TStringList.QuickSort(L, R: Integer; CompareFn: TStringListSortCompare);
var 
  I, J, Pivot: Integer;
begin
  repeat
    I := L;
    J := R;
    Pivot := (L + R) div 2;
    repeat
      while CompareFn(Self, I, Pivot) < 0 do Inc(I);
      while CompareFn(Self, J, Pivot) > 0 do Dec(J);
      if I <= J then
      begin
        FMap.InternalExchange(I, J); // No check, indices are correct.
        if Pivot = I then
          Pivot := J
        else if Pivot = J then
          Pivot := I;
        Inc(I);
        Dec(j);
      end;
    until I > J;
    if L < J then 
      QuickSort(L,J, CompareFn);
    L := I;
  until I >= R;
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

procedure TStringList.Sort;
begin
  if not Sorted and (FMap.Count > 1) then
  begin
    Changing;
    FMap.Sort;
    Changed;
  end;
end;

{$endif}

