{
    *********************************************************************
    Copyright (C) 1997, 1998 Gertjan Schouten

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

    System Utilities For Free Pascal
}

{   NewStr creates a new PString and assigns S to it
    if length(s) = 0 NewStr returns Nil   }

function NewStr(const S: string): PString;
begin
  if (S='') then
   Result:=nil
  else
   begin
     new(result);
     if (Result<>nil) then
       Result^:=s;
   end;
end;

{$ifdef dummy}
{ declaring this breaks delphi compatibility and e.g. tw3721.pp }
FUNCTION NewStr (Const S: ShortString): PShortString;
VAR P: PShortString;
BEGIN
   If (S = '') Then
     P := Nil
    Else
     Begin               { Return nil }
     GetMem(P, Length(S) + 1);                        { Allocate memory }
     If (P<>Nil) Then P^ := S;                        { Hold string }
     End;
   NewStr := P;                                       { Return result }
END;
{$endif dummy}

{   DisposeStr frees the memory occupied by S   }

procedure DisposeStr(S: PString);
begin
  if S <> Nil then
   begin
     dispose(s);
     S:=nil;
   end;
end;

PROCEDURE DisposeStr (S: PShortString);
BEGIN
   If (S <> Nil) Then FreeMem(S, Length(S^) + 1);     { Release memory }
END;


{   AssignStr assigns S to P^   }

procedure AssignStr(var P: PString; const S: string);
begin
  P^ := s;
end ;

{   AppendStr appends S to Dest   }

procedure AppendStr(var Dest: String; const S: string);
begin
Dest := Dest + S;
end ;

function IsLeadChar(C: AnsiChar): Boolean; inline;

begin
  Result:=C in LeadBytes;
end;

function IsLeadChar(B: Byte): Boolean; inline;


begin
  Result:=Char(B) in LeadBytes;
end;

Function InternalChangeCase(Const S : AnsiString; const Chars: TSysCharSet; const Adjustment: Longint): AnsiString;
  var
    i : Integer;
    P : PChar;
    Unique : Boolean;
  begin
    Result := S;
    if Result='' then
      exit;
    Unique:=false;
    P:=PChar(Result);
    for i:=1 to Length(Result) do
      begin
        if CharInSet(P^,Chars) then
          begin
            if not Unique then
              begin
                UniqueString(Result);
                p:=@Result[i];
                Unique:=true;
              end;
            P^:=Char(Ord(P^)+Adjustment);
          end;
        Inc(P);
      end;
  end;


{   UpperCase returns a copy of S where all lowercase characters ( from a to z )
    have been converted to uppercase   }
Function UpperCase(Const S : AnsiString) : AnsiString;
  begin
    Result:=InternalChangeCase(S,['a'..'z'],-32);
  end;


function UpperCase(const s: string; LocaleOptions: TLocaleOptions): string; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    case LocaleOptions of
      loInvariantLocale: Result:=UpperCase(s);
      loUserLocale: Result:=AnsiUpperCase(s);
    end;
  end;

{   LowerCase returns a copy of S where all uppercase characters ( from A to Z )
    have been converted to lowercase  }
Function Lowercase(Const S : AnsiString) : AnsiString;
  begin
    Result:=InternalChangeCase(S,['A'..'Z'],32);
  end;


function LowerCase(const s: string; LocaleOptions: TLocaleOptions): string; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    case LocaleOptions of
      loInvariantLocale: Result:=LowerCase(s);
      loUserLocale: Result:=AnsiLowerCase(s);
    end;
  end;


function LowerCase(const V: variant): string; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    result:=LowerCase(ansistring(V));
  end;


{   CompareStr compares S1 and S2, the result is the based on
    substraction of the ascii values of the characters in S1 and S2
    case     result
    S1 < S2  < 0
    S1 > S2  > 0
    S1 = S2  = 0     }

{$IF SIZEOF(SIZEINT)>SIZEOF(INTEGER)}
Function DoCapSizeInt(SI : SizeInt) : Integer; inline;

begin
  if (SI<0) then
    result:=-1
  else if (SI>0) then
    result:=1
  else
    result:=0;
end;
{$DEFINE CAPSIZEINT:=DoCapSizeInt}
{$ELSE}
{$DEFINE CAPSIZEINT:=}
{$ENDIF}

function CompareStr(const S1, S2: string): Integer;
var res,count, count1, count2: SizeInt;
begin
  result := 0;
  Count1 := Length(S1);
  Count2 := Length(S2);
  if Count1>Count2 then
    Count:=Count2
  else
    Count:=Count1;
  result := CompareMemRange(Pointer(S1),Pointer(S2), Count);
  if result=0 then
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(Count1-Count2);
end;

function CompareStr(const S1, S2: string; LocaleOptions: TLocaleOptions): Integer; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}
begin
  case LocaleOptions of
    loInvariantLocale: Result:=CompareStr(S1,S2);
    loUserLocale: Result:=AnsiCompareStr(S1,S2);
  end;
end;

{   CompareMemRange returns the result of comparison of Length bytes at P1 and P2
    case       result
    P1 < P2    < 0
    P1 > P2    > 0
    P1 = P2    = 0    }

function CompareMemRange(P1, P2: Pointer; Length: PtrUInt): integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
begin
  If P1=P2 then
    Result:=0
  else
    Result:=CompareByte(P1^,P2^,Length);
end;

function CompareMem(P1, P2: Pointer; Length: PtrUInt): Boolean;{$ifdef SYSUTILSINLINE}inline;{$endif}
begin
  if P1=P2 then
    Result:=True
  else
    Result:=CompareByte(P1^,P2^,Length)=0;
end;


{   CompareText compares S1 and S2, the result is the based on
    substraction of the ascii values of characters in S1 and S2
    comparison is case-insensitive
    case     result
    S1 < S2  < 0
    S1 > S2  > 0
    S1 = S2  = 0     }

function CompareText(const S1, S2: string): Integer; overload;

var
  i, count, count1, count2: sizeint;
  Chr1, Chr2: byte;
  P1, P2: PChar;
begin
  Count1 := Length(S1);
  Count2 := Length(S2);
  if (Count1>Count2) then
    Count := Count2
  else
    Count := Count1;
  i := 0;
  if count>0 then
    begin
      P1 := @S1[1];
      P2 := @S2[1];
      while i < Count do
        begin
          Chr1 := byte(p1^);
          Chr2 := byte(p2^);
          if Chr1 <> Chr2 then
            begin
              if Chr1 in [97..122] then
                dec(Chr1,32);
              if Chr2 in [97..122] then
                dec(Chr2,32);
              if Chr1 <> Chr2 then
                Break;
            end;
          Inc(P1); Inc(P2); Inc(I);
        end;
    end;
  if i < Count then
    result := Chr1-Chr2
  else
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(Count1-Count2);
end;

function CompareText(const S1, S2: string; LocaleOptions: TLocaleOptions): Integer; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}

begin
  case LocaleOptions of
    loInvariantLocale: Result:=CompareText(S1,S2);
    loUserLocale: Result:=AnsiCompareText(S1,S2);
  end;
end;

function SameText(const s1,s2:String):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}

begin
 Result:=CompareText(S1,S2)=0;
end;

function SameText(const s1,s2:String; LocaleOptions: TLocaleOptions):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}

begin
  case LocaleOptions of
    loInvariantLocale: Result:=SameText(S1,S2);
    loUserLocale: Result:=AnsiSameText(S1,S2);
  end;
end;

function SameStr(const s1,s2:String):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}

begin
 Result:=CompareStr(S1,S2)=0;
end;

function SameStr(const s1,s2:String; LocaleOptions: TLocaleOptions):Boolean; overload;{$ifdef SYSUTILSINLINE}inline;{$endif}

begin
  case LocaleOptions of
    loInvariantLocale: Result:=SameStr(S1,S2);
    loUserLocale: Result:=AnsiSameStr(S1,S2);
  end;
end;

{$ifndef FPC_NOGENERICANSIROUTINES}
{==============================================================================}
{   Ansi string functions                                                      }
{   these functions rely on the character set loaded by the OS                 }
{==============================================================================}

type
  TCaseTranslationTable = array[0..255] of char;

var
  { Tables with upper and lowercase forms of character sets.
    MUST be initialized with the correct code-pages }
  UpperCaseTable: TCaseTranslationTable;
  LowerCaseTable: TCaseTranslationTable;

function GenericAnsiUpperCase(const s: string): string;
  var
    len, i: integer;
begin
  len := length(s);
  SetLength(result, len);
  for i := 1 to len do
     result[i] := UpperCaseTable[ord(s[i])];
end;


function GenericAnsiLowerCase(const s: string): string;
  var
    len, i: integer;
begin
  len := length(s);
  SetLength(result, len);
  for i := 1 to len do
     result[i] := LowerCaseTable[ord(s[i])];
end;


function GenericAnsiCompareStr(const S1, S2: string): PtrInt;
  Var
    I,L1,L2 : SizeInt;
begin
  Result:=0;
  L1:=Length(S1);
  L2:=Length(S2);
  I:=1;
  While (Result=0) and ((I<=L1) and (I<=L2)) do
    begin
    Result:=Ord(S1[I])-Ord(S2[I]); //!! Must be replaced by ansi characters !!
    Inc(I);
    end;
  If Result=0 Then
    Result:=L1-L2;
end;

function GenericAnsiCompareText(const S1, S2: string): PtrInt;
  Var
    I,L1,L2 : SizeInt;
begin
  Result:=0;
  L1:=Length(S1);
  L2:=Length(S2);
  I:=1;
  While (Result=0) and ((I<=L1) and (I<=L2)) do
    begin
    Result:=Ord(LowerCaseTable[Ord(S1[I])])-Ord(LowerCaseTable[Ord(S2[I])]); //!! Must be replaced by ansi characters !!
    Inc(I);
    end;
  If Result=0 Then
    Result:=L1-L2;
end;

function GenericAnsiStrComp(S1, S2: PChar): PtrInt;

begin
  Result:=0;
  If S1=Nil then
    begin
      If S2=Nil Then Exit;
      result:=-1;
      exit;
    end;
  If S2=Nil then
    begin
      Result:=1;
      exit;
    end;
  While (Result=0) and (S1^<>#0) and (S2^<>#0) do begin
    Result:=Ord(S1^)-Ord(S2^); //!! Must be replaced by ansi characters !!
    Inc(S1);
    Inc(S2);
  end;
  if (Result=0) and (S1^<>S2^) then // loop ended because exactly one has #0
    if S1^=#0 then // shorter string is smaller
      result:=-1
    else
      result:=1;
end;


function GenericAnsiStrIComp(S1, S2: PChar): PtrInt;

begin
  Result:=0;
  If S1=Nil then
    begin
    If S2=Nil Then Exit;
    result:=-1;
    exit;
    end;
  If S2=Nil then
    begin
    Result:=1;
    exit;
    end;
  While (Result=0) and (S1^<>#0) and (S2^<>#0) do begin
    Result:=Ord(LowerCaseTable[Ord(S1[0])])-Ord(LowerCaseTable[Ord(S2[0])]); //!! Must be replaced by ansi characters !!
    Inc(S1);
    Inc(S2);
  end;
  if (Result=0) and (s1[0]<>s2[0]) then //length(s1)<>length(s2)
    if s1[0]=#0 then
      Result:=-1 //s1 shorter than s2
    else
      Result:=1; //s1 longer than s2
end;


function GenericAnsiStrLComp(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;

Var I : PtrUInt;

begin
  Result:=0;
  If MaxLen=0 then exit;
  If S1=Nil then
    begin
    If S2=Nil Then Exit;
    result:=-1;
    exit;
    end;
  If S2=Nil then
    begin
    Result:=1;
    exit;
    end;
  I:=0;
  Repeat
    Result:=Ord(S1[0])-Ord(S2[0]); //!! Must be replaced by ansi characters !!
    Inc(S1);
    Inc(S2);
    Inc(I);
  Until (Result<>0) or (I=MaxLen)
end;


function GenericAnsiStrLIComp(S1, S2: PChar; MaxLen: PtrUInt): PtrInt;

Var I : PtrUInt;

begin
  Result:=0;
  If MaxLen=0 then exit;
  If S1=Nil then
    begin
    If S2=Nil Then Exit;
    result:=-1;
    exit;
    end;
  If S2=Nil then
    begin
    Result:=1;
    exit;
    end;
  I:=0;
  Repeat
    Result:=Ord(LowerCaseTable[Ord(S1[0])])-Ord(LowerCaseTable[Ord(S2[0])]); //!! Must be replaced by ansi characters !!
    Inc(S1);
    Inc(S2);
    Inc(I);
  Until (Result<>0) or (I=MaxLen)
end;


function GenericAnsiStrLower(Str: PChar): PChar;
begin
result := Str;
if Str <> Nil then begin
   while Str^ <> #0 do begin
      Str^ := LowerCaseTable[byte(Str^)];
      Str := Str + 1;
      end;
   end;
end;


function GenericAnsiStrUpper(Str: PChar): PChar;
begin
result := Str;
if Str <> Nil then begin
   while Str^ <> #0 do begin
      Str^ := UpperCaseTable[byte(Str^)];
      Str := Str + 1;
      end ;
   end ;
end ;
{$endif FPC_NOGENERICANSIROUTINES}

function AnsiSameText(const s1,s2:String):Boolean;{$ifdef SYSUTILSINLINE}inline;{$endif}

begin
 AnsiSameText:=AnsiCompareText(S1,S2)=0;
end;

function AnsiSameStr(const s1,s2:String):Boolean;{$ifdef SYSUTILSINLINE}inline;{$endif}

begin
  AnsiSameStr:=AnsiCompareStr(S1,S2)=0;
end;

function AnsiLastChar(const S: string): PChar;

begin
  //!! No multibyte yet, so we return the last one.
  result:=StrEnd(Pchar(pointer(S)));  // strend checks for nil
  Dec(Result);
end ;

function AnsiStrLastChar(Str: PChar): PChar;
begin
  //!! No multibyte yet, so we return the last one.
  result:=StrEnd(Str);
  Dec(Result);
end ;


function AnsiUpperCase(const s: string): string;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    result:=widestringmanager.UpperAnsiStringProc(s);
  end;


function AnsiLowerCase(const s: string): string;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    result:=widestringmanager.LowerAnsiStringProc(s);
  end;


function AnsiCompareStr(const S1, S2: string): integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(widestringmanager.CompareStrAnsiStringProc(s1,s2));
  end;


function AnsiCompareText(const S1, S2: string): integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(widestringmanager.CompareTextAnsiStringProc(s1,s2));
  end;


function AnsiStrComp(S1, S2: PChar): integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(widestringmanager.StrCompAnsiStringProc(s1,s2));
  end;


function AnsiStrIComp(S1, S2: PChar): integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(widestringmanager.StrICompAnsiStringProc(s1,s2));
  end;


function AnsiStrLComp(S1, S2: PChar; MaxLen: SizeUInt): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(widestringmanager.StrLCompAnsiStringProc(s1,s2,maxlen));
  end;


function AnsiStrLIComp(S1, S2: PChar; MaxLen: SizeUint): Integer;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    // CAPSIZEINT is no-op if Sizeof(Sizeint)<=SizeOF(Integer)
    result:=CAPSIZEINT(widestringmanager.StrLICompAnsiStringProc(s1,s2,maxlen));
  end;


function AnsiStrLower(Str: PChar): PChar;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    result:=widestringmanager.StrLowerAnsiStringProc(Str);
  end;


function AnsiStrUpper(Str: PChar): PChar;{$ifdef SYSUTILSINLINE}inline;{$endif}
  begin
    result:=widestringmanager.StrUpperAnsiStringProc(Str);
  end;


{==============================================================================}
{  End of Ansi functions                                                       }
{==============================================================================}

{   Trim returns a copy of S with blanks characters on the left and right stripped off   }

Const WhiteSpace = [#0..' '];

function Trim(const S: string): string;
var Ofs, Len: integer;
begin
  len := Length(S);
  while (Len>0) and (S[Len] in WhiteSpace) do
   dec(Len);
  Ofs := 1;
  while (Ofs<=Len) and (S[Ofs] in WhiteSpace) do
   Inc(Ofs);
  result := Copy(S, Ofs, 1 + Len - Ofs);
end ;

{   TrimLeft returns a copy of S with all blank characters on the left stripped off  }

function TrimLeft(const S: string): string;
var i,l:integer;
begin
  l := length(s);
  i := 1;
  while (i<=l) and (s[i] in whitespace) do
   inc(i);
  Result := copy(s, i, l);
end ;

{   TrimRight returns a copy of S with all blank characters on the right stripped off  }

function TrimRight(const S: string): string;
var l:integer;
begin
  l := length(s);
  while (l>0) and (s[l] in whitespace) do
   dec(l);
  result := copy(s,1,l);
end ;

{   QuotedStr returns S quoted left and right and every single quote in S
    replaced by two quotes   }

function QuotedStr(const S: string): string;
begin
result := AnsiQuotedStr(s, '''');
end ;

{   AnsiQuotedStr returns S quoted left and right by Quote,
    and every single occurance of Quote replaced by two   }

function AnsiQuotedStr(const S: string; Quote: char): string;
var i, j, count: integer;
begin
result := '' + Quote;
count := length(s);
i := 0;
j := 0;
while i < count do begin
   i := i + 1;
   if S[i] = Quote then begin
      result := result + copy(S, 1 + j, i - j) + Quote;
      j := i;
      end ;
   end ;
if i <> j then
   result := result + copy(S, 1 + j, i - j);
result := result + Quote;
end ;

{   AnsiExtractQuotedStr returns a copy of Src with quote characters
    deleted to the left and right and double occurances
    of Quote replaced by a single Quote   }


function AnsiExtractQuotedStr(var  Src: PChar; Quote: Char): string;
var
  P,Q,R: PChar;
begin
 result:='';
 if Src=Nil then exit;
 P := Src;
 Q := StrEnd(P);
 if P=Q then
   exit;
 if P^<>quote then
   exit(strpas(P));
 inc(p);
 setlength(result,(Q-P)+1);
 R:=@Result[1];
 while P <> Q do
   begin
     R^:=P^;
     inc(R);
     if (P^ = Quote) then
       begin
         P := P + 1;
         if (p^ <> Quote) then
          begin
            dec(R);
            break;
          end;
       end;
     P := P + 1;
   end ;
 src:=p;
 SetLength(result, (R-pchar(@Result[1])));
end ;


{  Change CRLF, CR or LF with the default for the current platform  }

function AdjustLineBreaks(const S: string): string;

begin
  Result:=AdjustLineBreaks(S,DefaultTextLineBreakStyle);
end;

{  Change CRLF, CR or LF with the indicated style }

function AdjustLineBreaks(const S: string; Style: TTextLineBreakStyle): string;
var
  Source,Dest: PChar;
  DestLen: Integer;
  I,J,L: Longint;

begin
  Source:=Pointer(S);
  L:=Length(S);
  DestLen:=L;
  I:=1;
  while (I<=L) do
    begin
    case S[i] of
      #10: if (Style=tlbsCRLF) then
               Inc(DestLen);
      #13: if (Style=tlbsCRLF) then
             if (I<L) and (S[i+1]=#10) then
               Inc(I)
             else
               Inc(DestLen)
             else if (I<L) and (S[I+1]=#10) then
               Dec(DestLen);
    end;
    Inc(I);
    end;
  if (DestLen=L) then
    Result:=S
  else
    begin
    SetLength(Result, DestLen);
    FillChar(Result[1],DestLen,0);
    Dest := Pointer(Result);
    J:=0;
    I:=0;
    While I<L do
      case Source[I] of
        #10: begin
             if Style=tlbsCRLF then
               begin
               Dest[j]:=#13;
               Inc(J);
              end;
             Dest[J] := #10;
             Inc(J);
             Inc(I);
             end;
        #13: begin
             if Style=tlbsCRLF then
               begin
               Dest[j] := #13;
               Inc(J);
               end;
             Dest[j]:=#10;
             Inc(J);
             Inc(I);
             if Source[I]=#10 then
               Inc(I);
             end;
      else
        Dest[j]:=Source[i];
        Inc(J);
        Inc(I);
      end;
    end;
end;


{   IsValidIdent returns true if the first character of Ident is in:
    'A' to 'Z', 'a' to 'z' or '_' and the following characters are
    on of: 'A' to 'Z', 'a' to 'z', '0'..'9' or '_'    }

function IsValidIdent(const Ident: string; AllowDots: Boolean = False; StrictDots: Boolean = False): Boolean;
const
  Alpha = ['A'..'Z', 'a'..'z', '_'];
  AlphaNum = Alpha + ['0'..'9'];
  Dot = '.';
var
  First: Boolean;
  I, Len: Integer;
begin
  Len := Length(Ident);
  if Len < 1 then
    Exit(False);
  First := True;
  for I := 1 to Len do
  begin
    if First then
    begin
      Result := Ident[I] in Alpha;
      First := False;
    end
    else if AllowDots and (Ident[I] = Dot) then
    begin
      if StrictDots then
      begin
        Result := I < Len;
        First := True;
      end;
    end
    else
      Result := Ident[I] in AlphaNum;
    if not Result then
      Break;
  end;
end;

{   IntToStr returns a string representing the value of Value    }

function IntToStr(Value: Longint): string;
begin
 System.Str(Value, result);
end ;


function IntToStr(Value: int64): string;
begin
 System.Str(Value, result);
end ;

function IntToStr(Value: QWord): string;
begin
 System.Str(Value, result);
end ;

function UIntToStr(Value: QWord): string;

begin
  result:=IntTostr(Value);
end;

function UIntToStr(Value: Cardinal): string; 

begin
  System.Str(Value, result);
end;

{   IntToHex returns a string representing the hexadecimal value of Value   }

const
   HexDigits: array[0..15] of char = '0123456789ABCDEF';

function IntToHex(Value: Longint; Digits: integer): string;
var i: integer;
begin
 If Digits=0 then
   Digits:=1;
 SetLength(result, digits);
 for i := 0 to digits - 1 do
  begin
   result[digits - i] := HexDigits[value and 15];
   value := value shr 4;
  end ;
 while value <> 0 do begin
   result := HexDigits[value and 15] + result;
   value := value shr 4;
 end;
end ;

function IntToHex(Value: int64; Digits: integer): string;
var i: integer;
begin
 If Digits=0 then
   Digits:=1;
 SetLength(result, digits);
 for i := 0 to digits - 1 do
  begin
   result[digits - i] := HexDigits[value and 15];
   value := value shr 4;
  end ;
 while value <> 0 do begin
   result := HexDigits[value and 15] + result;
   value := value shr 4;
 end;
end ;

function IntToHex(Value: QWord; Digits: integer): string;
begin
  result:=IntToHex(Int64(Value),Digits);
end;

function IntToHex(Value: Int8): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(Int8));
end;

function IntToHex(Value: UInt8): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(UInt8));
end;

function IntToHex(Value: Int16): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(Int16));
end;

function IntToHex(Value: UInt16): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(UInt16));
end;

function IntToHex(Value: Int32): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(Int32));
end;

function IntToHex(Value: UInt32): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(UInt32));
end;

function IntToHex(Value: Int64): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(Int64));
end;

function IntToHex(Value: UInt64): string;
begin
  Result:=IntToHex(Value, 2*SizeOf(UInt64));
end;

function TryStrToInt(const s: string; out i : Longint) : boolean;
var Error : word;
begin
  Val(s, i, Error);
  TryStrToInt:=Error=0
end;

{   StrToInt converts the string S to an integer value,
    if S does not represent a valid integer value EConvertError is raised  }

function StrToInt(const S: string): Longint;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then raise EConvertError.createfmt(SInvalidInteger,[S]);
end ;


function StrToInt64(const S: string): int64;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then raise EConvertError.createfmt(SInvalidInteger,[S]);
end;


function TryStrToInt64(const s: string; Out i : int64) : boolean;
var Error : word;
begin
  Val(s, i, Error);
  TryStrToInt64:=Error=0
end;


function StrToQWord(const s: string): QWord;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then raise EConvertError.createfmt(SInvalidInteger,[S]);
end;

function StrToUInt64(const s: string): UInt64;
begin
  result:=StrToQWord(s);
end;

function StrToDWord(const s: string): DWord;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then raise EConvertError.createfmt(SInvalidInteger,[S]);
end;

function TryStrToDWord(const s: string; Out D: DWord): boolean;
var Error : word;
begin
  Val(s, D, Error);
  TryStrToDWord:=Error=0
end;

function StrToUInt(const s: string): Cardinal;
begin
  StrToUInt:=StrToDWord(s);
end;

function TryStrToUInt(const s: string; out C: Cardinal): Boolean;
begin
  TryStrToUInt:=TryStrToDWord(s, C);
end;

function TryStrToQWord(const s: string; Out Q: QWord): boolean;
var Error : word;
begin
  Val(s, Q, Error);
  TryStrToQWord:=Error=0
end;

function TryStrToUInt64(const s: string; Out u: UInt64): boolean;
begin
  result:=TryStrToQWord(s,u);
end;

{   StrToIntDef converts the string S to an integer value,
    Default is returned in case S does not represent a valid integer value  }

function StrToIntDef(const S: string; Default: Longint): Longint;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then result := Default;
end ;

{   StrToDWordDef converts the string S to an DWord value,
    Default is returned in case S does not represent a valid DWord value  }

function StrToDWordDef(const S: string; Default: DWord): DWord;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then result := Default;
end;

function StrToUIntDef(const S: string; Default: Cardinal): Cardinal;
begin
  Result:=StrToDWordDef(S, Default);
end;

{   StrToInt64Def converts the string S to an int64 value,
    Default is returned in case S does not represent a valid int64 value  }

function StrToInt64Def(const S: string; Default: int64): int64;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then result := Default;
end ;

{   StrToQWordDef converts the string S to an QWord value,
    Default is returned in case S does not represent a valid QWord value  }

function StrToQWordDef(const S: string; Default: QWord): QWord;
var Error: word;
begin
  Val(S, result, Error);
  if Error <> 0 then result := Default;
end;

function StrToUInt64Def(const S: string; Default: UInt64): UInt64;
begin
  result:=StrToQWordDef(S,Default);
end;

{   LoadStr returns the string resource Ident.   }

function LoadStr(Ident: integer): string;
begin
  result:='';
end ;

{   FmtLoadStr returns the string resource Ident and formats it accordingly   }


function FmtLoadStr(Ident: integer; const Args: array of const): string;
begin
  result:='';
end;

Const
  feInvalidFormat   = 1;
  feMissingArgument = 2;
  feInvalidArgIndex = 3;

{$ifdef fmtdebug}
Procedure Log (Const S: String);
begin
 Writeln (S);
end;
{$endif}


Procedure DoFormatError (ErrCode : Longint;const fmt:ansistring);
Var
  S : String;
begin
  //!! must be changed to contain format string...
  S:=fmt;
  Case ErrCode of
   feInvalidFormat : raise EConvertError.Createfmt(SInvalidFormat,[s]);
   feMissingArgument : raise EConvertError.Createfmt(SArgumentMissing,[s]);
   feInvalidArgIndex : raise EConvertError.Createfmt(SInvalidArgIndex,[s]);
 end;
end;

{ we've no templates, but with includes we can simulate this :) }

{$macro on}
{$define INFORMAT}
{$define TFormatString:=ansistring}
{$define TFormatChar:=char}

Function Format (Const Fmt : AnsiString; const Args : Array of const; const FormatSettings: TFormatSettings) : AnsiString;
{$i sysformt.inc}

{$undef TFormatString}
{$undef TFormatChar}
{$undef INFORMAT}
{$macro off}

Function Format (Const Fmt : AnsiString; const Args : Array of const) : AnsiString;

begin
  Result:=Format(Fmt,Args,DefaultFormatSettings);
end;

Function FormatBuf (Var Buffer; BufLen : Cardinal; Const Fmt; fmtLen : Cardinal; Const Args : Array of const; Const FormatSettings: TFormatSettings) : Cardinal;

Var S,F : String;

begin
  Setlength(F,fmtlen);
  if fmtlen > 0 then
    Move(fmt,F[1],fmtlen);
  S:=Format (F,Args,FormatSettings);
  If Cardinal(Length(S))<Buflen then
    Result:=Length(S)
  else
    Result:=Buflen;
  Move(S[1],Buffer,Result);
end;

Function FormatBuf (Var Buffer; BufLen : Cardinal;
                     Const Fmt; fmtLen : Cardinal;
                     Const Args : Array of const) : Cardinal;

begin
  Result:=FormatBuf(Buffer,BufLen,Fmt,FmtLen,Args,DefaultFormatSettings);
end;

Procedure FmtStr(Var Res: string; const Fmt : string; Const args: Array of const; Const FormatSettings: TFormatSettings);

begin
  Res:=Format(fmt,Args,FormatSettings);
end;

Procedure FmtStr(Var Res: String; Const Fmt : String; Const args: Array of const);

begin
  FmtStr(Res,Fmt,Args,DefaultFormatSettings);
end;


Function StrFmt(Buffer,Fmt : PChar; Const args: Array of const) : Pchar;

begin
  Result:=StrFmt(Buffer,Fmt,Args,DefaultFormatSettings);
end;

Function StrFmt(Buffer,Fmt : PChar; Const Args: Array of const; Const FormatSettings: TFormatSettings): PChar;

begin
  Buffer[FormatBuf(Buffer^,Maxint,Fmt^,strlen(fmt),args,FormatSettings)]:=#0;
  Result:=Buffer;
end;

Function StrLFmt(Buffer : PCHar; Maxlen : Cardinal;Fmt : PChar; Const args: Array of const) : Pchar;

begin
  Result:=StrLFmt(Buffer,MaxLen,Fmt,Args,DefaultFormatSettings);
end;

Function StrLFmt(Buffer : PCHar; Maxlen : Cardinal;Fmt : PChar; Const args: Array of const; Const FormatSettings: TFormatSettings) : Pchar;

begin
  Buffer[FormatBuf(Buffer^,MaxLen,Fmt^,strlen(fmt),args,FormatSettings)]:=#0;
  Result:=Buffer;
end;

{$ifndef FPUNONE}

Function StrToFloat(Const S: String): Extended;

begin
  Result:=StrToFloat(S,DefaultFormatSettings);
end;

Function StrToFloat(Const S : String; Const FormatSettings: TFormatSettings) : Extended;

Begin // texttofloat handles NIL properly
  If Not TextToFloat(Pchar(pointer(S)),Result,FormatSettings) then
    Raise EConvertError.createfmt(SInValidFLoat,[S]);
End;

function StrToFloatDef(const S: string; const Default: Extended): Extended;

begin
  Result:=StrToFloatDef(S,Default,DefaultFormatSettings);
end;

Function StrToFloatDef(Const S: String; Const Default: Extended; Const FormatSettings: TFormatSettings): Extended;

begin
   if not TextToFloat(PChar(pointer(S)),Result,fvExtended,FormatSettings) then
     Result:=Default;
end;

Function TextToFloat(Buffer: PChar; Out Value: Extended; Const FormatSettings: TFormatSettings): Boolean;

Var
  E,P : Integer;
  S : String;

Begin
  S:=StrPas(Buffer);
  //ThousandSeparator not allowed as by Delphi specs
  if (FormatSettings.ThousandSeparator <> FormatSettings.DecimalSeparator) and
     (Pos(FormatSettings.ThousandSeparator, S) <> 0) then
    begin
      Result := False;
      Exit;
    end;
  if (FormatSettings.DecimalSeparator <> '.') and
     (Pos('.', S) <>0) then
    begin
      Result := False;
      Exit;
    end;
  P:=Pos(FormatSettings.DecimalSeparator,S);
  If (P<>0) Then
    S[P] := '.';
  try
    Val(trim(S),Value,E);
  { on x87, a floating point exception may be pending in case of an invalid
    input value -> trigger it now }
{$if defined(cpui386) or (defined(cpux86_64) and not(defined(win64))) or defined(cpui8086)}
    asm
      fwait
    end;
{$endif}
  except
    E:=1;
  end;
  Result:=(E=0);
End;


Function TextToFloat(Buffer: PChar; Out Value: Extended): Boolean;

begin
  Result:=TextToFloat(Buffer,Value,DefaultFormatSettings);
end;

Function TextToFloat(Buffer: PChar; Out Value; ValueType: TFloatValue): Boolean;

begin
  Result:=TextToFloat(Buffer,Value,ValueType,DefaultFormatSettings);
end;

Function TextToFloat(Buffer: PChar; Out Value; ValueType: TFloatValue; Const FormatSettings: TFormatSettings): Boolean;

Var
  E,P : Integer;
  S : String;

Begin
  S:=StrPas(Buffer);
  //ThousandSeparator not allowed as by Delphi specs
  if (FormatSettings.ThousandSeparator <> FormatSettings.DecimalSeparator) and
     (Pos(FormatSettings.ThousandSeparator, S) <> 0) then
    begin
      Result := False;
      Exit;
    end;
  if (FormatSettings.DecimalSeparator <> '.') and
     (Pos('.', S) <>0) then
    begin
      Result := False;
      Exit;
    end;

  P:=Pos(FormatSettings.DecimalSeparator,S);
  If (P<>0) Then
    S[P] := '.';
  s:=Trim(s);
  try
    case ValueType of
      fvCurrency:
        Val(S,Currency(Value),E);
      fvExtended:
        Val(S,Extended(Value),E);
      fvDouble:
        Val(S,Double(Value),E);
      fvSingle:
        Val(S,Single(Value),E);
      fvComp:
        Val(S,Comp(Value),E);
      fvReal:
        Val(S,Real(Value),E);
    end;
  { on x87, a floating point exception may be pending in case of an invalid
    input value -> trigger it now }
{$if defined(cpui386) or (defined(cpux86_64) and not(defined(win64))) or defined(cpui8086)}
    asm
      fwait
    end;
{$endif}
  except
    E:=1;
  end;
  Result:=(E=0);
End;


Function TryStrToFloat(Const S : String; Out Value: Single): Boolean;

begin
  Result:=TryStrToFloat(S,Value,DefaultFormatSettings);
end;

Function TryStrToFloat(Const S : String; Out Value: Single; Const FormatSettings: TFormatSettings): Boolean;
Begin
  Result := TextToFloat(PChar(pointer(S)), Value, fvSingle,FormatSettings);
End;

Function TryStrToFloat(Const S : String; Out Value: Double): Boolean;

begin
  Result:=TryStrToFloat(S,Value,DefaultFormatSettings);
end;

Function TryStrToFloat(Const S : String; Out Value: Double; Const FormatSettings: TFormatSettings): Boolean;
Begin
  Result := TextToFloat(PChar(pointer(S)), Value, fvDouble,FormatSettings);
End;

{$ifdef FPC_HAS_TYPE_EXTENDED}
Function TryStrToFloat(Const S : String; Out Value: Extended): Boolean;

begin
  Result:=TryStrToFloat(S,Value,DefaultFormatSettings);
end;

Function TryStrToFloat(Const S : String; Out Value: Extended; Const FormatSettings: TFormatSettings): Boolean;
Begin
  Result := TextToFloat(PChar(pointer(S)), Value,FormatSettings);
End;
{$endif FPC_HAS_TYPE_EXTENDED}


const
{$ifdef FPC_HAS_TYPE_EXTENDED}
  maxdigits = 17;
{$else}
  maxdigits = 15;
{$endif}

{ deactive aligned function for 2.6 }
{$ifdef VER2_6}
{$macro on}
{$define aligned:= }
{$endif VER2_6}
Function FloatToStrFIntl(const Value; format: TFloatFormat; Precision, Digits: Integer; ValueType: TFloatValue; Const FormatSettings: TFormatSettings): String;
Var
  P, PE, Q, Exponent: Integer;
  Negative: Boolean;
  DS: Char;

  function RemoveLeadingNegativeSign(var AValue: String): Boolean;
  // removes negative sign in case when result is zero eg. -0.00
  var
    i: PtrInt;
    TS: Char;
    StartPos: PtrInt;
  begin
    Result := False;
    if Format = ffCurrency then
      StartPos := 1
    else
      StartPos := 2;
    TS := FormatSettings.ThousandSeparator;
    for i := StartPos to length(AValue) do
    begin
      Result := (AValue[i] in ['0', DS, 'E', '+', TS]);
      if not Result then
        break;
    end;
    if (Result) and (Format <> ffCurrency) then
      Delete(AValue, 1, 1);
  end;

Begin
  DS:=FormatSettings.DecimalSeparator;
  Case format Of

    ffGeneral:

      Begin
        case ValueType of
          fvCurrency:
              If (Precision = -1) Or (Precision > 19) Then Precision := 19;
          else
              If (Precision = -1) Or (Precision > maxdigits) Then Precision := maxdigits;
        end;
        { First convert to scientific format, with correct precision }
        case ValueType of
          fvDouble:
            Str(Double(Extended(Aligned(Value))):precision+7, Result);
          fvSingle:
            Str(Single(Extended(Aligned(Value))):precision+6, Result);
          fvCurrency:
            Str(Currency(Aligned(Value)):precision+6, Result);
          else
            Str(Extended(Aligned(Value)):precision+8, Result);
        end;
        { Delete leading spaces }
        while Result[1] = ' ' do
          System.Delete(Result, 1, 1);
        P := Pos('.', Result);
        if P<>0 then
          Result[P] := DS
        else
          Exit; { NAN or other special case }
        { Consider removing exponent }
        PE:=Pos('E',Result);
        if PE > 0 then begin
          { Read exponent }
          Q := PE+2;
          Exponent := 0;
          while (Q <= Length(Result)) do begin
            Exponent := Exponent*10 + Ord(Result[Q])-Ord('0');
            Inc(Q);
          end;
          if Result[PE+1] = '-' then
            Exponent := -Exponent;
          if (P+Exponent < PE) and (Exponent > -6) then begin
            { OK to remove exponent }
            SetLength(Result,PE-1); { Trim exponent }
            if Exponent >= 0 then begin
              { Shift point to right }
              for Q := 0 to Exponent-1 do begin
                Result[P] := Result[P+1];
                Inc(P);
              end;
              Result[P] := DS;
              P := 1;
              if Result[P] = '-' then
                Inc(P);
              while (Result[P] = '0') and (P < Length(Result)) and (Result[P+1] <> DS) do
                { Trim leading zeros; conversion above should not give any, but occasionally does
                  because of rounding }
                System.Delete(Result,P,1);
            end else begin
              { Add zeros at start }
              Insert(Copy('00000',1,-Exponent),Result,P-1);
              Result[P-Exponent] := Result[P-Exponent-1]; { Copy leading digit }
              Result[P] := DS;
              if Exponent <> -1 then
                Result[P-Exponent-1] := '0';
            end;
            { Remove trailing zeros }
            Q := Length(Result);
            while (Q > 0) and (Result[Q] = '0') do
              Dec(Q);
            if Result[Q] = DS then
              Dec(Q); { Remove trailing decimal point }
            if (Q = 0) or ((Q=1) and (Result[1] = '-')) then
              Result := '0'
            else
              SetLength(Result,Q);
          end else begin
            { Need exponent, but remove superfluous characters }
            { Delete trailing zeros }
            while Result[PE-1] = '0' do begin
              System.Delete(Result,PE-1,1);
              Dec(PE);
            end;
            { If number ends in decimal point, remove it }
            if Result[PE-1] = DS then begin
              System.Delete(Result,PE-1,1);
              Dec(PE);
            end;
            { delete superfluous + in exponent }
            if Result[PE+1]='+' then
              System.Delete(Result,PE+1,1)
            else
              Inc(PE);
            while Result[PE+1] = '0' do
              { Delete leading zeros in exponent }
              System.Delete(Result,PE+1,1)
          end;
        end;
      End;

    ffExponent:

      Begin
        If (Precision = -1) Or (Precision > maxdigits) Then Precision := maxdigits;
        case ValueType of
          fvDouble:
            Str(Double(Extended(Aligned(Value))):Precision+7, Result);
          fvSingle:
            Str(Single(Extended(Aligned(Value))):Precision+6, Result);
          fvCurrency:
            Str(Currency(Aligned(Value)):Precision+6, Result);
          else
            Str(Extended(Aligned(Value)):Precision+8, Result);
        end;
        { Delete leading spaces }
        while Result[1] = ' ' do
          System.Delete(Result, 1, 1);

        if (Result[1]='-') and
          { not Nan etc.? }
          (Result[3]='.') then
          Result[3] := DS
        else if Result[2]='.' then
          Result[2] := DS;

        P:=Pos('E',Result);
        if P <> 0 then
          begin
            Inc(P, 2);
            if Digits > 4 then
              Digits:=4;
            Digits:=Length(Result) - P - Digits + 1;
            if Digits < 0 then
              insert(copy('0000',1,-Digits),Result,P)
            else
              while (Digits > 0) and (Result[P] = '0') do
                begin
                  System.Delete(Result, P, 1);
                  if P > Length(Result) then
                    begin
                      System.Delete(Result, P - 2, 2);
                      break;
                    end;
                  Dec(Digits);
                end;
          end;
      End;

    ffFixed:

      Begin
        If Digits = -1 Then Digits := 2
        Else If Digits > 18 Then Digits := 18;
        case ValueType of
          fvDouble:
            Str(Double(Extended(Aligned(Value))):0:Digits, Result);
          fvSingle:
            Str(Single(Extended(Aligned(Value))):0:Digits, Result);
          fvCurrency:
            Str(Currency(Aligned(Value)):0:Digits, Result);
          else
            Str(Extended(Aligned(Value)):0:Digits, Result);
        end;
        If Result[1] = ' ' Then
          System.Delete(Result, 1, 1);
        P := Pos('.', Result);
        If P <> 0 Then Result[P] := DS;
      End;

    ffNumber:

      Begin
        If Digits = -1 Then Digits := 2
        Else If Digits > maxdigits Then Digits := maxdigits;
        case ValueType of
          fvDouble:
            Str(Double(Extended(Aligned(Value))):0:Digits, Result);
          fvSingle:
            Str(Single(Extended(Aligned(Value))):0:Digits, Result);
          fvCurrency:
            Str(Currency(Aligned(Value)):0:Digits, Result);
          else
            Str(Extended(Aligned(Value)):0:Digits, Result);
        end;
        If Result[1] = ' ' Then System.Delete(Result, 1, 1);
        P := Pos('.', Result);
        If P <> 0 Then
          Result[P] := DS
        else
          P := Length(Result)+1;
        Dec(P, 3);
        While (P > 1) Do
        Begin
          If (Result[P - 1] <> '-') And (FormatSettings.ThousandSeparator <> #0) Then
            Insert(FormatSettings.ThousandSeparator, Result, P);
          Dec(P, 3);
        End;
      End;

    ffCurrency:

      Begin
        If Digits = -1 Then Digits := FormatSettings.CurrencyDecimals
        Else If Digits > 18 Then Digits := 18;
        case ValueType of
          fvDouble:
            Str(Double(Extended(Aligned(Value))):0:Digits, Result);
          fvSingle:
            Str(Single(Extended(Aligned(Value))):0:Digits, Result);
          fvCurrency:
            Str(Currency(Aligned(Value)):0:Digits, Result);
          else
            Str(Extended(Aligned(Value)):0:Digits, Result);
        end;
        Negative:=Result[1] = '-';
        if Negative then
          System.Delete(Result, 1, 1);
        P := Pos('.', Result);
        If P <> 0 Then Result[P] := DS else P := Length(Result)+1;
        Dec(P, 3);
        While (P > 1) Do
        Begin
          If FormatSettings.ThousandSeparator<>#0 Then
            Insert(FormatSettings.ThousandSeparator, Result, P);
          Dec(P, 3);
        End;

        if (length(Result) > 1) and Negative then
          Negative := not RemoveLeadingNegativeSign(Result);

        If Not Negative Then
        Begin
          Case FormatSettings.CurrencyFormat Of
            0: Result := FormatSettings.CurrencyString + Result;
            1: Result := Result + FormatSettings.CurrencyString;
            2: Result := FormatSettings.CurrencyString + ' ' + Result;
            3: Result := Result + ' ' + FormatSettings.CurrencyString;
          End
        End
        Else
        Begin
          Case FormatSettings.NegCurrFormat Of
            0: Result := '(' + FormatSettings.CurrencyString + Result + ')';
            1: Result := '-' + FormatSettings.CurrencyString + Result;
            2: Result := FormatSettings.CurrencyString + '-' + Result;
            3: Result := FormatSettings.CurrencyString + Result + '-';
            4: Result := '(' + Result + FormatSettings.CurrencyString + ')';
            5: Result := '-' + Result + FormatSettings.CurrencyString;
            6: Result := Result + '-' + FormatSettings.CurrencyString;
            7: Result := Result + FormatSettings.CurrencyString + '-';
            8: Result := '-' + Result + ' ' + FormatSettings.CurrencyString;
            9: Result := '-' + FormatSettings.CurrencyString + ' ' + Result;
            10: Result := Result + ' ' + FormatSettings.CurrencyString + '-';
            11: Result := FormatSettings.CurrencyString + ' ' + Result + '-';
            12: Result := FormatSettings.CurrencyString + ' ' + '-' + Result;
            13: Result := Result + '-' + ' ' + FormatSettings.CurrencyString;
            14: Result := '(' + FormatSettings.CurrencyString + ' ' + Result + ')';
            15: Result := '(' + Result + ' ' + FormatSettings.CurrencyString + ')';
          End;
        End;
      End;
  End;
  if not (format in [ffCurrency]) and (length(Result) > 1) and (Result[1] = '-') then
    RemoveLeadingNegativeSign(Result);
End;
{$macro off}

{$ifdef FPC_HAS_TYPE_EXTENDED}
Function FloatToStr(Value: Extended; Const FormatSettings: TFormatSettings): String;
Begin
  Result := FloatToStrFIntl(Value, ffGeneral, 15, 0, fvExtended,FormatSettings);
End;


Function FloatToStr(Value: Extended): String;

begin
  Result:=FloatToStr(Value,DefaultFormatSettings);
end;
{$endif FPC_HAS_TYPE_EXTENDED}


Function FloatToStr(Value: Currency; Const FormatSettings: TFormatSettings): String;
Begin
  Result := FloatToStrFIntl(Value, ffGeneral, 15, 0, fvCurrency,FormatSettings);
End;


Function FloatToStr(Value: Currency): String;

begin
  Result:=FloatToStr(Value,DefaultFormatSettings);
end;


Function FloatToStr(Value: Double; Const FormatSettings: TFormatSettings): String;
var
  e: Extended;
Begin
  e := Value;
  Result := FloatToStrFIntl(e, ffGeneral, 15, 0, fvDouble,FormatSettings);
End;


Function FloatToStr(Value: Double): String;

begin
  Result:=FloatToStr(Value,DefaultFormatSettings);
end;


Function FloatToStr(Value: Single; Const FormatSettings: TFormatSettings): String;
var
  e: Extended;
Begin
  e := Value;
  Result := FloatToStrFIntl(e, ffGeneral, 15, 0, fvSingle,FormatSettings);
End;


Function FloatToStr(Value: Single): String;

begin
  Result:=FloatToStr(Value,DefaultFormatSettings);
end;


Function FloatToStr(Value: Comp; Const FormatSettings: TFormatSettings): String;
var
  e: Extended;
Begin
  e := Value;
  Result := FloatToStrFIntl(e, ffGeneral, 15, 0, fvComp,FormatSettings);
End;


Function FloatToStr(Value: Comp): String;

begin
  Result:=FloatToStr(Value,DefaultFormatSettings);
end;

{$ifndef FPC_COMP_IS_INT64}
Function FloatToStr(Value: Int64): String;

begin
  Result:=FloatToStr(Value,DefaultFormatSettings);
end;

Function FloatToStr(Value: Int64; Const FormatSettings: TFormatSettings): String;

var
  e: Extended;

Begin
  e := Comp(Value);
  Result := FloatToStrFIntl(e, ffGeneral, 15, 0, fvComp,FormatSettings);
End;
{$endif FPC_COMP_IS_INT64}


Function FloatToText(Buffer: PChar; Value: Extended; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): Longint;
Var
  Tmp: String[40];
Begin
  Tmp := FloatToStrF(Value, format, Precision, Digits,FormatSettings);
  Result := Length(Tmp);
  Move(Tmp[1], Buffer[0], Result);
End;


Function FloatToText(Buffer: PChar; Value: Extended; format: TFloatFormat; Precision, Digits: Integer): Longint;

begin
  Result:=FloatToText(Buffer,Value,Format,Precision,Digits,DefaultFormatSettings);
end;


{$ifdef FPC_HAS_TYPE_EXTENDED}
Function FloatToStrF(Value: Extended; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
begin
  Result := FloatToStrFIntl(value,format,precision,digits,fvExtended,FormatSettings);
end;


Function FloatToStrF(Value: Extended; format: TFloatFormat; Precision, Digits: Integer): String;

begin
  Result:=FloatToStrF(Value,Format,Precision,Digits,DefaultFormatSettings);
end;
{$endif}


Function FloatToStrF(Value: Currency; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
begin
  Result := FloatToStrFIntl(value,format,precision,digits,fvCurrency,FormatSettings);
end;


Function FloatToStrF(Value: Currency; format: TFloatFormat; Precision, Digits: Integer): String;

begin
  Result:=FloatToStrF(Value,format,Precision,Digits,DefaultFormatSettings);
end;


Function FloatToStrF(Value: Double; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;
var
  e: Extended;
begin
  e := Value;
  result := FloatToStrFIntl(e,format,precision,digits,fvDouble,FormatSettings);
end;


Function FloatToStrF(Value: Double; format: TFloatFormat; Precision, Digits: Integer): String;

begin
  Result:= FloatToStrF(Value,Format,Precision,Digits,DefaultFormatSettings);
end;


Function FloatToStrF(Value: Single; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;

var
  e: Extended;
begin
  e:=Value;
  result := FloatToStrFIntl(e,format,precision,digits,fvSingle,FormatSettings);
end;


Function FloatToStrF(Value: Single; format: TFloatFormat; Precision, Digits: Integer): String;

begin
  Result:= FloatToStrF(Value,Format,Precision,Digits,DefaultFormatSettings);
end;


Function FloatToStrF(Value: Comp; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;

var
  e: Extended;
begin
  e := Value;
  Result := FloatToStrFIntl(e,format,precision,digits,fvComp,FormatSettings);
end;


Function FloatToStrF(Value: Comp; format: TFloatFormat; Precision, Digits: Integer): String;

begin
  Result:=FloatToStrF(Value,Format,Precision,Digits,DefaultFormatSettings);
end;



{$ifndef FPC_COMP_IS_INT64}
Function FloatToStrF(Value: Int64; format: TFloatFormat; Precision, Digits: Integer; Const FormatSettings: TFormatSettings): String;

var
  e: Extended;
begin
  e := Comp(Value);
  result := FloatToStrFIntl(e,format,precision,digits,fvComp,FormatSettings);
end;


Function FloatToStrF(Value: Int64; format: TFloatFormat; Precision, Digits: Integer): String;

begin
  Result:=FloatToStrF(Value,Format,Precision,Digits,DefaultFormatSettings);
end;
{$endif FPC_COMP_IS_INT64}


Function CurrToStrF(Value: Currency; Format: TFloatFormat; Digits: Integer; Const FormatSettings: TFormatSettings): string;

begin
  result:=FloatToStrF(Value,Format,19,Digits,FormatSettings);
end;


Function CurrToStrF(Value: Currency; Format: TFloatFormat; Digits: Integer): string;

begin
  Result:=CurrToStrF(Value,Format,Digits,DefaultFormatSettings);
end;


Function FloatToDateTime (Const Value : Extended) : TDateTime;
begin
  If (Value<MinDateTime) or (Value>MaxDateTime) then
    Raise EConvertError.CreateFmt (SInvalidDateTimeFloat,[Value]);
  Result:=Value;
end;

function TryFloatToCurr(const Value: Extended; var AResult: Currency): Boolean;

begin
  Result:=(Value>=MinCurrency) and (Value<=MaxCurrency);
  if Result then
    AResult := Value;
end;

function FloatToCurr(const Value: Extended): Currency;

begin
  if not TryFloatToCurr(Value, Result) then
    Raise EConvertError.CreateFmt(SInvalidCurrency, [FloatToStr(Value)]);
end;


Function CurrToStr(Value: Currency): string;
begin
  Result:=FloatToStrF(Value,ffGeneral,-1,0);
end;


Function CurrToStr(Value: Currency; Const FormatSettings: TFormatSettings): string;
begin
  Result:=FloatToStrF(Value,ffGeneral,-1,0,FormatSettings);
end;


function StrToCurr(const S: string): Currency;
begin
  if not TextToFloat(PChar(pointer(S)), Result, fvCurrency) then
    Raise EConvertError.createfmt(SInValidFLoat,[S]);
end;


function StrToCurr(const S: string; Const FormatSettings: TFormatSettings): Currency;
begin
  if not TextToFloat(PChar(pointer(S)), Result, fvCurrency,FormatSettings) then
    Raise EConvertError.createfmt(SInValidFLoat,[S]);
end;


Function TryStrToCurr(Const S : String; Out Value: Currency): Boolean;
Begin
  Result := TextToFloat(PChar(pointer(S)), Value, fvCurrency);
End;


function TryStrToCurr(const S: string;Out Value : Currency; Const FormatSettings: TFormatSettings): Boolean;
Begin
  Result := TextToFloat(PChar(pointer(S)), Value, fvCurrency,FormatSettings);
End;


function StrToCurrDef(const S: string; Default : Currency): Currency;
begin
  if not TextToFloat(PChar(pointer(S)), Result, fvCurrency) then
    Result:=Default;
end;

function StrToCurrDef(const S: string; Default : Currency; Const FormatSettings: TFormatSettings): Currency;
begin
  if not TextToFloat(PChar(pointer(S)), Result, fvCurrency,FormatSettings) then
    Result:=Default;
end;
{$endif FPUNONE}

function AnsiDequotedStr(const S: string; AQuote: Char): string;

var p : pchar;

begin
  p:=pchar(pointer(s)); // work around CONST. Ansiextract is safe for nil
  result:=AnsiExtractquotedStr(p,AQuote);
end;

function StrToBool(const S: string): Boolean;
begin
  if not(TryStrToBool(S,Result,DefaultFormatSettings)) then
    Raise EConvertError.CreateFmt(SInvalidBoolean,[S]);
end;

function StrToBool(const S: string; const FormatSettings: TFormatSettings): Boolean;
begin
  if not(TryStrToBool(S,Result,FormatSettings)) then
    Raise EConvertError.CreateFmt(SInvalidBoolean,[S]);
end;

procedure CheckBoolStrs;
begin
    If Length(TrueBoolStrs)=0 then
      begin
        SetLength(TrueBoolStrs,1);
        TrueBoolStrs[0]:='True';
      end;
    If Length(FalseBoolStrs)=0 then
      begin
        SetLength(FalseBoolStrs,1);
        FalseBoolStrs[0]:='False';
      end;
end;


function BoolToStr(B: Boolean;UseBoolStrs:Boolean=False): string;
begin
 if UseBoolStrs Then
  begin
    CheckBoolStrs;
    if B then
      Result:=TrueBoolStrs[0]
    else
      Result:=FalseBoolStrs[0];
  end
 else
  If B then
    Result:='-1'
  else
    Result:='0';
end;

// from textmode IDE util funcs.
function BoolToStr(B: boolean; const TrueS, FalseS: string): string;
begin
  if B then Result:=TrueS else BoolToStr:=FalseS;
end;

function StrToBoolDef(const S: string; Default: Boolean): Boolean;
begin
  if not(TryStrToBool(S,Result)) then
    Result:=Default;
end;

function StrToBoolDef(const S: string; Default: Boolean; const FormatSettings: TFormatSettings): Boolean;
begin
  if not(TryStrToBool(S,Result,FormatSettings)) then
    Result:=Default;
end;

function TryStrToBool(const S: string; out Value: Boolean): Boolean;

begin
  Result:=TryStrToBool(S,Value,DefaultFormatSettings);
end;

function TryStrToBool(const S: string; out Value: Boolean; const FormatSettings: TFormatSettings): Boolean;
Var
  Temp : String;
  I    : Longint;
{$ifdef FPUNONE}
  D : Longint;
{$else}
  D : Double;
{$endif}
  Code: word;
begin
  Temp:=upcase(S);
  Val(temp,D,code);
  Result:=true;
  If (Code=0) or TryStrToFloat(S,D,FormatSettings) then
{$ifdef FPUNONE}
    Value:=(D<>0)
{$else}
    Value:=(D<>0.0)
{$endif}
  else
    begin
      CheckBoolStrs;
      for I:=low(TrueBoolStrs) to High(TrueBoolStrs) do
        if Temp=upcase(TrueBoolStrs[I]) then
          begin
            Value:=true;
            exit;
          end;
      for I:=low(FalseBoolStrs) to High(FalseBoolStrs) do
        if Temp=upcase(FalseBoolStrs[I]) then
          begin
            Value:=false;
            exit;
          end;
      Result:=false;
    end;
end;

{$ifndef FPUNONE}

Function FloatToTextFmt(Buffer: PChar; Value: Extended; format: PChar): Integer;

begin
  Result:=FloatToTextFmt(Buffer,Value,Format,DefaultFormatSettings);
end;

{$MACRO ON}
{$define FPChar:=PAnsiChar}
{$define FChar:=AnsiChar}
{$define FString:=AnsiString}

{$I fmtflt.inc}

Function FloatToTextFmt(Buffer: PChar; Value: Extended; format: PChar; FormatSettings : TFormatSettings): Integer;

begin
  Result:=IntFloatToTextFmt(Buffer,Value,fvExtended,Format,FormatSettings);
end;

Procedure FloatToDecimal(Out Result: TFloatRec; const Value; ValueType: TFloatValue; Precision, Decimals : integer);
var
  Buffer: String[254];  //Though str func returns only 25 chars, this might change in the future
  InfNan: string[3];
  Error, N, L, Start, C: Integer;
  GotNonZeroBeforeDot, BeforeDot : boolean;

begin
  case ValueType of
    fvExtended:
      Str(Extended(Value):25, Buffer);
    fvDouble,
    fvReal:
      Str(Double(Value):23, Buffer);
    fvSingle:
      Str(Single(Value):16, Buffer);
    fvCurrency:
      Str(Currency(Value):25, Buffer);
    fvComp:
      Str(Currency(Value):23, Buffer);
  end;

  N := 1;
  L := Byte(Buffer[0]);
  while Buffer[N]=' ' do
    Inc(N);
  Result.Negative := (Buffer[N] = '-');
  if Result.Negative then
    Inc(N)
  else if (Buffer[N] = '+') then
    inc(N);
  { special cases for Inf and Nan }
  if (L>=N+2) then
    begin
      InfNan:=copy(Buffer,N,3);
      if (InfNan='Inf') then
        begin
          Result.Digits[0]:=#0;
          Result.Exponent:=32767;
          exit
        end;
      if (InfNan='Nan') then
        begin
          Result.Digits[0]:=#0;
          Result.Exponent:=-32768;
          exit
        end;
    end;
  Start := N;  //Start of digits
  Result.Exponent := 0; BeforeDot := true;
  GotNonZeroBeforeDot := false;
  while (L>=N) and (Buffer[N]<>'E') do
    begin
      if Buffer[N]='.' then
        BeforeDot := false
      else
        begin
          if BeforeDot then
            begin  // Currently this is always 1 char
              Inc(Result.Exponent);
              Result.Digits[N-Start] := Buffer[N];
              if Buffer[N] <> '0' then
                GotNonZeroBeforeDot := true;
            end
          else
            Result.Digits[N-Start-1] := Buffer[N]
        end;
      Inc(N);
    end;
  Inc(N); // Pass through 'E'
  if N<=L then
    begin
      Val(Copy(Buffer, N, L-N+1), C, Error); // Get exponent after 'E'
      Inc(Result.Exponent, C);
    end;
  // Calculate number of digits we have from str
  if BeforeDot then
    N := N - Start - 1
  else
    N := N - Start - 2;
  L := SizeOf(Result.Digits);
  if N<L then
    FillChar(Result.Digits[N], L-N, '0');  //Zero remaining space
  if Decimals + Result.Exponent < Precision Then //After this it is the same as in FloatToDecimal
    N := Decimals + Result.Exponent
  Else
    N := Precision;
  if N >= L Then
    N := L-1;
  if N = 0 Then
    begin
      if Result.Digits[0] >= '5' Then
        begin
          Result.Digits[0] := '1';
          Result.Digits[1] := #0;
          Inc(Result.Exponent);
        end
      Else
        Result.Digits[0] := #0;
    end  //N=0
  Else if N > 0 Then
    begin
      if Result.Digits[N] >= '5' Then
        begin
          Repeat
            Result.Digits[N] := #0;
            Dec(N);
            Inc(Result.Digits[N]);
          Until (N = 0) Or (Result.Digits[N] < ':');
          If Result.Digits[0] = ':' Then
            begin
              Result.Digits[0] := '1';
              Inc(Result.Exponent);
            end;
        end
      Else
        begin
          Result.Digits[N] := '0';
          While (N > -1) And (Result.Digits[N] = '0') Do
            begin
              Result.Digits[N] := #0;
              Dec(N);
            end;
        end;
      end //N>0
  Else
    Result.Digits[0] := #0;
  if (Result.Digits[0] = #0) and
     not GotNonZeroBeforeDot then
    begin
      Result.Exponent := 0;
      Result.Negative := False;
    end;
end;


Procedure FloatToDecimal(Out Result: TFloatRec; Value: Extended; Precision, Decimals : integer);

begin
  FloatToDecimal(Result,Value,fvExtended,Precision,Decimals);
end;

Function FormatFloat(Const Format : String; Value : Extended; Const FormatSettings: TFormatSettings) : String;

Var
  buf : Array[0..1024] of char;

Begin // not changed to pchar(pointer(). Possibly not safe
  Buf[FloatToTextFmt(@Buf[0],Value,Pchar(Format),FormatSettings)]:=#0;
  Result:=StrPas(@Buf[0]);
End;

Function FormatFloat(Const format: String; Value: Extended): String;

begin
  Result:=FormatFloat(Format,Value,DefaultFormatSettings);
end;

Function FormatCurr(const Format: string; Value: Currency; Const FormatSettings: TFormatSettings): string;
begin
  Result := FormatFloat(Format, Value,FormatSettings);
end;

function FormatCurr(const Format: string; Value: Currency): string;

begin
  Result:=FormatCurr(Format,Value,DefaultFormatSettings);
end;

{$endif}

{==============================================================================}
{   extra functions                                                            }
{==============================================================================}

{   LeftStr returns Count left-most characters from S }

function LeftStr(const S: string; Count: integer): string;
begin
  result := Copy(S, 1, Count);
end ;

{ RightStr returns Count right-most characters from S }

function RightStr(const S: string; Count: integer): string;
begin
   If Count>Length(S) then
     Count:=Length(S);
   result := Copy(S, 1 + Length(S) - Count, Count);
end;

{    BCDToInt converts the BCD value Value to an integer   }

function BCDToInt(Value: integer): integer;
var i, j, digit: integer;
begin
result := 0;
j := 1;

for i := 0 to SizeOf(Value) shl 1 - 1 do begin
   digit := Value and 15;

   if digit > $9 then
   begin
       if i = 0 then
       begin
           if digit in [$B, $D] then j := -1
       end
       else raise EConvertError.createfmt(SInvalidBCD,[Value]);
   end
   else
   begin
      result := result + j * digit;
      j := j * 10;
      end ;
   Value := Value shr 4;
   end ;
end ;

Function LastDelimiter(const Delimiters, S: string): SizeInt;
var
  chs: TSysCharSet;
  I: SizeInt;

begin
  chs := [];
  for I := 1 to Length(Delimiters) do
    Include(chs, Delimiters[I]);
  Result:=Length(S);
  While (Result>0) and not (S[Result] in chs) do
    Dec(Result);
end;

{$macro on}
{$define INSTRINGREPLACE}
{$define SRString:=String}
{$define SRUpperCase:=AnsiUppercase}
{$define SRPCHAR:=PChar}
{$define SRCHAR:=Char}

Function StringReplace(const S, OldPattern, NewPattern: string;  Flags: TReplaceFlags): string;

Var
  C : Integer;

begin
  Result:=StringReplace(S,OldPattern,NewPattern,Flags,C);
end;

function StringReplace(const S, OldPattern, NewPattern: string;  Flags: TReplaceFlags; Out aCount : Integer): string;

{$i syssr.inc}

{$undef INSTRINGREPLACE}
{$undef SRString}
{$undef SRUpperCase}
{$undef SRPCHAR}
{$undef SRCHAR}

Function IsDelimiter(const Delimiters, S: string; Index: SizeInt): Boolean;

begin
  Result:=False;
  If (Index>0) and (Index<=Length(S)) then
    Result:=Pos(S[Index],Delimiters)<>0; // Note we don't do MBCS yet
end;

Function ByteToCharLen(const S: string; MaxLen: SizeInt): SizeInt;

begin
  Result:=Length(S);
  If Result>MaxLen then
    Result:=MaxLen;
end;

Function ByteToCharIndex(const S: string; Index: SizeInt): SizeInt;

begin
  Result:=Index;
end;


Function CharToByteLen(const S: string; MaxLen: SizeInt): SizeInt;

begin
  Result:=Length(S);
  If Result>MaxLen then
    Result:=MaxLen;
end;

Function CharToByteIndex(const S: string; Index: SizeInt): SizeInt;

begin
  Result:=Index;
end;

Function ByteType(const S: string; Index: SizeUInt): TMbcsByteType;

begin
  Result:=mbSingleByte;
end;


Function StrByteType(Str: PChar; Index: SizeUInt): TMbcsByteType;
begin
  Result:=mbSingleByte;
end;


Function StrCharLength(const Str: PChar): SizeInt;
begin
  result:=widestringmanager.CharLengthPCharProc(Str);
end;


function StrNextChar(const Str: PChar): PChar;
begin
  result:=Str+StrCharLength(Str);
end;


Function FindCmdLineSwitch(const Switch: string; const Chars: TSysCharSet;IgnoreCase: Boolean): Boolean;

Var
  I,L : Integer;
  S,T : String;

begin
  Result:=False;
  S:=Switch;
  If IgnoreCase then
    S:=UpperCase(S);
  I:=ParamCount;
  While (Not Result) and (I>0) do
    begin
    L:=Length(Paramstr(I));
    If (L>0) and (ParamStr(I)[1] in Chars) then
      begin
      T:=Copy(ParamStr(I),2,L-1);
      If IgnoreCase then
        T:=UpperCase(T);
      Result:=S=T;
      end;
    Dec(i);
    end;
end;

Function FindCmdLineSwitch(const Switch: string; IgnoreCase: Boolean): Boolean;

begin
  Result:=FindCmdLineSwitch(Switch,SwitchChars,IgnoreCase);
end;

Function FindCmdLineSwitch(const Switch: string): Boolean;

begin
  Result:=FindCmdLineSwitch(Switch,SwitchChars,False);
end;

function WrapText(const Line, BreakStr: string; const BreakChars: TSysCharSet;  MaxCol: Integer): string;

const
  Quotes = ['''', '"'];

Var
  L : String;
  C,LQ,BC : Char;
  P,BLen,Len : Integer;
  HB,IBC : Boolean;

begin
  Result:='';
  L:=Line;
  Blen:=Length(BreakStr);
  If (BLen>0) then
    BC:=BreakStr[1]
  else
    BC:=#0;
  Len:=Length(L);
  While (Len>0) do
    begin
    P:=1;
    LQ:=#0;
    HB:=False;
    IBC:=False;
    While ((P<=Len) and ((P<=MaxCol) or not IBC)) and ((LQ<>#0) or Not HB) do
      begin
      C:=L[P];
      If (C=LQ) then
        LQ:=#0
      else If (C in Quotes) then
        LQ:=C;
      If (LQ<>#0) then
        Inc(P)
      else
        begin
        HB:=((C=BC) and (BreakStr=Copy(L,P,BLen)));
        If HB then
          Inc(P,Blen)
        else
          begin
          If (P>=MaxCol) then
            IBC:=C in BreakChars;
          Inc(P);
          end;
        end;
//      Writeln('"',C,'" : IBC : ',IBC,' HB  : ',HB,' LQ  : ',LQ,' P>MaxCol : ',P>MaxCol);
      end;
    Result:=Result+Copy(L,1,P-1);
    Delete(L,1,P-1);
    Len:=Length(L);
    If (Len>0) and Not HB then
      Result:=Result+BreakStr;
    end;
end;

function WrapText(const Line: string; MaxCol: Integer): string;
begin
  Result:=WrapText(Line,sLineBreak, [' ', '-', #9], MaxCol);
end;


{$ifndef FPC_NOGENERICANSIROUTINES}
{
   Case Translation Tables
   Can be used in internationalization support.

   Although these tables can be obtained through system calls
cd    it is better to not use those, since most implementation are not 100%
   WARNING:
   before modifying a translation table make sure that the current codepage
   of the OS corresponds to the one you make changes to
}



const
{$if defined(MSDOS) or defined(GO32V2) or defined(WATCOM) or defined(WIN16) }
   { upper case translation table for character set 850 }
   CP850UCT: array[128..255] of char =
   (#128,#154,#144,#182,#142,#182,#143,#128,#210,#211,#212,#216,#215,#222,#142,#143,
    #144,#146,#146,#226,#153,#227,#234,#235,'Y',#153,#154,#157,#156,#157,#158,#159,
    #181,#214,#224,#233,#165,#165,#166,#167,#168,#169,#170,#171,#172,#173,#174,#175,
    #176,#177,#178,#179,#180,#181,#182,#183,#184,#185,#186,#187,#188,#189,#190,#191,
    #192,#193,#194,#195,#196,#197,#199,#199,#200,#201,#202,#203,#204,#205,#206,#207,
    #208,#209,#210,#211,#212,#213,#214,#215,#216,#217,#218,#219,#220,#221,#222,#223,
    #224,#225,#226,#227,#229,#229,#230,#237,#232,#233,#234,#235,#237,#237,#238,#239,
    #240,#241,#242,#243,#244,#245,#246,#247,#248,#249,#250,#251,#252,#253,#254,#255);

   { lower case translation table for character set 850 }
   CP850LCT: array[128..255] of char =
   (#135,#129,#130,#131,#132,#133,#134,#135,#136,#137,#138,#139,#140,#141,#132,#134,
    #130,#145,#145,#147,#148,#149,#150,#151,#152,#148,#129,#155,#156,#155,#158,#159,
    #160,#161,#162,#163,#164,#164,#166,#167,#168,#169,#170,#171,#172,#173,#174,#175,
    #176,#177,#178,#179,#180,#160,#131,#133,#184,#185,#186,#187,#188,#189,#190,#191,
    #192,#193,#194,#195,#196,#197,#198,#198,#200,#201,#202,#203,#204,#205,#206,#207,
    #208,#209,#136,#137,#138,#213,#161,#140,#139,#217,#218,#219,#220,#221,#141,#223,
    #162,#225,#147,#149,#228,#228,#230,#237,#232,#163,#150,#151,#236,#236,#238,#239,
    #240,#241,#242,#243,#244,#245,#246,#247,#248,#249,#250,#251,#252,#253,#254,#255);
{$endif}

   { upper case translation table for character set ISO 8859/1  Latin 1  }
   CPISO88591UCT: array[192..255] of char =
   ( #192, #193, #194, #195, #196, #197, #198, #199,
     #200, #201, #202, #203, #204, #205, #206, #207,
     #208, #209, #210, #211, #212, #213, #214, #215,
     #216, #217, #218, #219, #220, #221, #222, #223,
     #192, #193, #194, #195, #196, #197, #198, #199,
     #200, #201, #202, #203, #204, #205, #206, #207,
     #208, #209, #210, #211, #212, #213, #214, #247,
     #216, #217, #218, #219, #220, #221, #222, #89 );

   { lower case translation table for character set ISO 8859/1  Latin 1  }
   CPISO88591LCT: array[192..255] of char =
   ( #224, #225, #226, #227, #228, #229, #230, #231,
     #232, #233, #234, #235, #236, #237, #238, #239,
     #240, #241, #242, #243, #244, #245, #246, #215,
     #248, #249, #250, #251, #252, #253, #254, #223,
     #224, #225, #226, #227, #228, #229, #230, #231,
     #232, #233, #234, #235, #236, #237, #238, #239,
     #240, #241, #242, #243, #244, #245, #246, #247,
     #248, #249, #250, #251, #252, #253, #254, #255 );

{$endif FPC_NOGENERICANSIROUTINES}

function sscanf(const s: string; const fmt : string;const Pointers : array of Pointer) : Integer;
  var
    i,j,n,m : SizeInt;
    s1      : string;

  function GetInt(unsigned : boolean=false) : Integer;
    begin
      s1 := '';
      while (Length(s) > n) and (s[n] = ' ') do
        inc(n);
      { read sign }
      if (Length(s)>= n) and (s[n] in ['+', '-']) then
        begin
          { don't accept - when reading unsigned }
          if unsigned and (s[n]='-') then
            begin
              result:=length(s1);
              exit;
            end
          else
            begin
              s1:=s1+s[n];
              inc(n);
            end;
        end;
      { read numbers }
      while (Length(s) >= n) and
            (s[n] in ['0'..'9']) do
        begin
          s1 := s1+s[n];
          inc(n);
        end;
      Result := Length(s1);
    end;


  function GetFloat : Integer;
    begin
      s1 := '';
      while (Length(s) > n) and (s[n] = ' ')  do
        inc(n);
      while (Length(s) >= n) and
            (s[n] in ['0'..'9', '+', '-', FormatSettings.DecimalSeparator, 'e', 'E']) do
        begin
          s1 := s1+s[n];
          inc(n);
        end;
      Result := Length(s1);
    end;


  function GetString : Integer;
    begin
      s1 := '';
      while (Length(s) > n) and (s[n] = ' ') do
        inc(n);
      while (Length(s) >= n) and (s[n] <> ' ')do
        begin
          s1 := s1+s[n];
          inc(n);
        end;
      Result := Length(s1);
    end;


  function ScanStr(c : Char) : Boolean;
    begin
      while (Length(s) > n) and (s[n] <> c) do
        inc(n);
      inc(n);
      If (n <= Length(s)) then
        Result := True
      else
        Result := False;
    end;


  function GetFmt : Integer;
    begin
      Result := -1;
      while true do
        begin

          while (Length(fmt) > m) and (fmt[m] = ' ') do
            inc(m);

          if (m >= Length(fmt)) then
            break;

          if (fmt[m] = '%') then
            begin
              inc(m);
              case fmt[m] of
                'd':
                  Result:=vtInteger;
{$ifndef FPUNONE}
                'f':
                  Result:=vtExtended;
{$endif}
                's':
                  Result:=vtString;
                'c':
                  Result:=vtChar;
                else
                  raise EFormatError.CreateFmt(SInvalidFormat,[fmt]);
              end;
              inc(m);
              break;
            end;

          if not(ScanStr(fmt[m])) then
            break;
          inc(m);
        end;
    end;


  begin
    n := 1;
    m := 1;
    Result := 0;

    for i:=0 to High(Pointers) do
      begin
        j := GetFmt;
        case j of
          vtInteger :
            begin
              if GetInt>0 then
                begin
                  pLongint(Pointers[i])^:=StrToInt(s1);
                  inc(Result);
                end
              else
                break;

            end;

          vtchar :
            begin
              if Length(s)>n then
                begin
                  pchar(Pointers[i])^:=s[n];
                  inc(n);
                  inc(Result);
                end
              else
                break;

            end;

{$ifndef FPUNONE}
          vtExtended :
            begin
              if GetFloat>0 then
                begin
                  pextended(Pointers[i])^:=StrToFloat(s1);
                  inc(Result);
                end
              else
                break;
            end;
{$endif}

          vtString :
            begin
              if GetString > 0 then
                begin
                  pansistring(Pointers[i])^:=s1;
                  inc(Result);
                end
              else
                break;
            end;
          else
            break;
        end;
      end;
   end;

{$macro on}
// Ansi version declaration
{$UNDEF SBUNICODE}
{$define SBChar:=AnsiChar}
{$define SBString:=AnsiString}
{$define TSBCharArray:=Array of SBChar}
{$define PSBChar:=PAnsiChar}
{$define SBRAWString:=RawByteString}
{$define TGenericStringBuilder:=TAnsiStringBuilder}

{$i syssb.inc}
{$undef SBChar}
{$undef SBString}
{$undef TSBCharArray}
{$undef PSBChar}
{$undef SBRAWString}
{$undef TGenericStringBuilder}

// Unicode version declaration

{$define SBUNICODE}
{$define SBChar:=WideChar}
{$define SBString:=UnicodeString}
{$define TSBCharArray:=Array of SBChar}
{$define PSBChar:=PWideChar}
{$define SBRAWString:=UnicodeString}
{$define TGenericStringBuilder:=TUnicodeStringBuilder}
{$i syssb.inc}
{$undef SBChar}
{$undef SBString}
{$undef TSBCharArray}
{$undef PSBChar}
{$undef SBRAWString}
{$undef TGenericStringBuilder}
{$undef SBUNICODE}


