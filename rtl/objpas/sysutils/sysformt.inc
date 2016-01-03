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

Var ChPos,OldPos,ArgPos,DoArg,Len : SizeInt;
    Hs,ToAdd : TFormatString;
    Index : SizeInt;
    Width,Prec : Longint;
    Left : Boolean;
    Fchar : char;
    vq : qword;

  {
    ReadFormat reads the format string. It returns the type character in
    uppercase, and sets index, Width, Prec to their correct values,
    or -1 if not set. It sets Left to true if left alignment was requested.
    In case of an error, DoFormatError is called.
  }

  Function ReadFormat : Char;

  Var Value : longint;

    Procedure ReadInteger;

    var
      Code: Word;
      ArgN: SizeInt;
    begin
      If Value<>-1 then exit; // Was already read.
      OldPos:=ChPos;
      While (ChPos<=Len) and
            (Fmt[ChPos]<='9') and (Fmt[ChPos]>='0') do inc(ChPos);
      If ChPos>len then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        DoFormatError(feInvalidFormat,ansistring(Fmt));
=======
        DoFormatError(feInvalidFormat,Fmt);
>>>>>>> graemeg/cpstrnew
=======
        DoFormatError(feInvalidFormat,Fmt);
>>>>>>> graemeg/cpstrnew
=======
        DoFormatError(feInvalidFormat,Fmt);
>>>>>>> graemeg/cpstrnew
=======
        DoFormatError(feInvalidFormat,Fmt);
>>>>>>> origin/cpstrnew
      If Fmt[ChPos]='*' then
        begin

        if Index=-1 then
          ArgN:=Argpos
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        else
        begin
          ArgN:=Index;
          Inc(Index);
        end;

        If (ChPos>OldPos) or (ArgN>High(Args)) then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          DoFormatError(feInvalidFormat,ansistring(Fmt));

        ArgPos:=ArgN+1;

        case Args[ArgN].Vtype of
          vtInteger: Value := Args[ArgN].VInteger;
          vtInt64: Value := Args[ArgN].VInt64^;
          vtQWord: Value := Args[ArgN].VQWord^;
        else
          DoFormatError(feInvalidFormat,ansistring(Fmt));
=======
        else
        begin
          ArgN:=Index;
          Inc(Index);
        end;

        If (ChPos>OldPos) or (ArgN>High(Args)) then
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          DoFormatError(feInvalidFormat,Fmt);

        ArgPos:=ArgN+1;

        case Args[ArgN].Vtype of
          vtInteger: Value := Args[ArgN].VInteger;
          vtInt64: Value := Args[ArgN].VInt64^;
          vtQWord: Value := Args[ArgN].VQWord^;
        else
          DoFormatError(feInvalidFormat,Fmt);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        end;
        Inc(ChPos);
        end
      else
        begin
        If (OldPos<ChPos) Then
          begin
          Val (Copy(Fmt,OldPos,ChPos-OldPos),value,code);
          // This should never happen !!
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          If Code>0 then DoFormatError (feInvalidFormat,ansistring(Fmt));
=======
          If Code>0 then DoFormatError (feInvalidFormat,Fmt);
>>>>>>> graemeg/cpstrnew
=======
          If Code>0 then DoFormatError (feInvalidFormat,Fmt);
>>>>>>> graemeg/cpstrnew
=======
          If Code>0 then DoFormatError (feInvalidFormat,Fmt);
>>>>>>> graemeg/cpstrnew
=======
          If Code>0 then DoFormatError (feInvalidFormat,Fmt);
>>>>>>> origin/cpstrnew
          end
        else
          Value:=-1;
        end;
    end;

    Procedure ReadIndex;

    begin
      If Fmt[ChPos]<>':' then
        ReadInteger
      else
        value:=0; // Delphi undocumented behaviour, assume 0, #11099
      If Fmt[ChPos]=':' then
        begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        If Value=-1 then DoFormatError(feMissingArgument,ansistring(Fmt));
=======
        If Value=-1 then DoFormatError(feMissingArgument,fmt);
>>>>>>> graemeg/cpstrnew
=======
        If Value=-1 then DoFormatError(feMissingArgument,fmt);
>>>>>>> graemeg/cpstrnew
=======
        If Value=-1 then DoFormatError(feMissingArgument,fmt);
>>>>>>> graemeg/cpstrnew
=======
        If Value=-1 then DoFormatError(feMissingArgument,fmt);
>>>>>>> origin/cpstrnew
        Index:=Value;
        Value:=-1;
        Inc(ChPos);
        end;
{$ifdef fmtdebug}
      Log ('Read index');
{$endif}
    end;

    Procedure ReadLeft;

    begin
      If Fmt[ChPos]='-' then
        begin
        left:=True;
        Inc(ChPos);
        end
      else
        Left:=False;
{$ifdef fmtdebug}
      Log ('Read Left');
{$endif}
    end;

    Procedure ReadWidth;

    begin
      ReadInteger;
      If Value<>-1 then
        begin
        Width:=Value;
        Value:=-1;
        end;
{$ifdef fmtdebug}
      Log ('Read width');
{$endif}
    end;

    Procedure ReadPrec;

    begin
      If Fmt[ChPos]='.' then
        begin
        inc(ChPos);
          ReadInteger;
        If Value=-1 then
         Value:=0;
        prec:=Value;
        end;
{$ifdef fmtdebug}
      Log ('Read precision');
{$endif}
    end;

{$ifdef INWIDEFORMAT}
  var
    FormatChar : TFormatChar;
{$endif INWIDEFORMAT}

  begin
{$ifdef fmtdebug}
    Log ('Start format');
{$endif}
    Index:=-1;
    Width:=-1;
    Prec:=-1;
    Value:=-1;
    inc(ChPos);
    If Fmt[ChPos]='%' then
      begin
        Result:='%';
        exit;                           // VP fix
      end;
    ReadIndex;
    ReadLeft;
    ReadWidth;
    ReadPrec;
{$ifdef INWIDEFORMAT}
    FormatChar:=UpCase(UnicodeChar(Fmt[ChPos]));
    if word(FormatChar)>255 then
      ReadFormat:=#255
    else
      ReadFormat:=FormatChar;
{$else INWIDEFORMAT}
    ReadFormat:=Upcase(Fmt[ChPos]);
{$endif INWIDEFORMAT}
{$ifdef fmtdebug}
    Log ('End format');
{$endif}
end;


{$ifdef fmtdebug}
Procedure DumpFormat (C : char);
begin
  Write ('Fmt : ',fmt:10);
  Write (' Index : ',Index:3);
  Write (' Left  : ',left:5);
  Write (' Width : ',Width:3);
  Write (' Prec  : ',prec:3);
  Writeln (' Type  : ',C);
end;
{$endif}


function Checkarg (AT : SizeInt;err:boolean):boolean;
{
  Check if argument INDEX is of correct type (AT)
  If Index=-1, ArgPos is used, and argpos is augmented with 1
  DoArg is set to the argument that must be used.
}
begin
  result:=false;
  if Index=-1 then
    DoArg:=Argpos
  else
    DoArg:=Index;
  ArgPos:=DoArg+1;
  If (Doarg>High(Args)) or (Args[Doarg].Vtype<>AT) then
   begin
     if err then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      DoFormatError(feInvalidArgindex,ansistring(Fmt));
=======
      DoFormatError(feInvalidArgindex,Fmt);
>>>>>>> graemeg/cpstrnew
=======
      DoFormatError(feInvalidArgindex,Fmt);
>>>>>>> graemeg/cpstrnew
=======
      DoFormatError(feInvalidArgindex,Fmt);
>>>>>>> graemeg/cpstrnew
=======
      DoFormatError(feInvalidArgindex,Fmt);
>>>>>>> origin/cpstrnew
     dec(ArgPos);
     exit;
   end;
  result:=true;
end;

begin
  Result:='';
  Len:=Length(Fmt);
  ChPos:=1;
  OldPos:=1;
  ArgPos:=0;
  While ChPos<=len do
    begin
    While (ChPos<=Len) and (Fmt[ChPos]<>'%') do
      inc(ChPos);
    If ChPos>OldPos Then
      Result:=Result+Copy(Fmt,OldPos,ChPos-Oldpos);
    If ChPos<Len then
      begin
      FChar:=ReadFormat;
{$ifdef fmtdebug}
      DumpFormat(FCHar);
{$endif}
      Case FChar of
        'D' : begin
              if Checkarg(vtinteger,false) then
                Str(Args[Doarg].VInteger,ToAdd)
              else if CheckArg(vtInt64,false) then
                Str(Args[DoArg].VInt64^,toadd)
              else if CheckArg(vtQWord,true) then
                Str(int64(Args[DoArg].VQWord^),toadd);
              Width:=Abs(width);
              Index:=Prec-Length(ToAdd);
              If ToAdd[1]<>'-' then
                ToAdd:=TFormatString(StringOfChar('0',Index))+ToAdd
              else
                // + 1 to accomodate for - sign in length !!
                Insert(TFormatString(StringOfChar('0',Index+1)),toadd,2);
              end;
        'U' : begin
              if Checkarg(vtinteger,false) then
                Str(cardinal(Args[Doarg].VInteger),ToAdd)
              else if CheckArg(vtInt64,false) then
                Str(qword(Args[DoArg].VInt64^),toadd)
              else if CheckArg(vtQWord,true) then
                Str(Args[DoArg].VQWord^,toadd);
              Width:=Abs(width);
              Index:=Prec-Length(ToAdd);
              ToAdd:=TFormatString(StringOfChar('0',Index))+ToAdd
              end;
{$ifndef FPUNONE}
        'E' : begin
              if CheckArg(vtCurrency,false) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VCurrency^,ffexponent,Prec,3,FormatSettings))
              else if CheckArg(vtExtended,true) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VExtended^,ffexponent,Prec,3,FormatSettings));
              end;
        'F' : begin
              if CheckArg(vtCurrency,false) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VCurrency^,ffFixed,9999,Prec,FormatSettings))
              else if CheckArg(vtExtended,true) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VExtended^,ffFixed,9999,Prec,FormatSettings));
              end;
        'G' : begin
              if CheckArg(vtCurrency,false) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VCurrency^,ffGeneral,Prec,3,FormatSettings))
              else if CheckArg(vtExtended,true) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VExtended^,ffGeneral,Prec,3,FormatSettings));
              end;
        'N' : begin
              if CheckArg(vtCurrency,false) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VCurrency^,ffNumber,9999,Prec,FormatSettings))
              else if CheckArg(vtExtended,true) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VExtended^,ffNumber,9999,Prec,FormatSettings));
              end;
        'M' : begin
              if CheckArg(vtExtended,false) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VExtended^,ffCurrency,9999,Prec,FormatSettings))
              else if CheckArg(vtCurrency,true) then
                ToAdd:=TFormatString(FloatToStrF(Args[doarg].VCurrency^,ffCurrency,9999,Prec,FormatSettings));
              end;
{$else}
        'E','F','G','N','M':
              RunError(207);
{$endif}
        'S' : begin
                if CheckArg(vtString,false) then
                  hs:=TFormatString(Args[doarg].VString^)
                else
                  if CheckArg(vtChar,false) then
                    hs:=TFormatString(Args[doarg].VChar)
                else
                  if CheckArg(vtPChar,false) then
                    hs:=TFormatString(Args[doarg].VPChar)
                else
                  if CheckArg(vtPWideChar,false) then
                    hs:=TFormatString(WideString(Args[doarg].VPWideChar))
                else
                  if CheckArg(vtWideChar,false) then
                    hs:=TFormatString(WideString(Args[doarg].VWideChar))
                else
                  if CheckArg(vtWidestring,false) then
                    hs:=TFormatString(WideString(Args[doarg].VWideString))
                else
                  if CheckArg(vtAnsiString,false) then
                    hs:=TFormatString(ansistring(Args[doarg].VAnsiString))
                else
                  if CheckArg(vtUnicodeString,false) then
                    hs:=TFormatString(UnicodeString(Args[doarg].VUnicodeString))
                else
<<<<<<< HEAD
=======
                  if CheckArg(vtAnsiString,false) then
                    hs:=ansistring(Args[doarg].VAnsiString)
                else
                  if CheckArg(vtUnicodeString,false) then
                    hs:=UnicodeString(Args[doarg].VUnicodeString)
                else
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
                  if CheckArg(vtAnsiString,false) then
                    hs:=ansistring(Args[doarg].VAnsiString)
                else
                  if CheckArg(vtUnicodeString,false) then
                    hs:=UnicodeString(Args[doarg].VUnicodeString)
                else
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
                  if CheckArg(vtAnsiString,false) then
                    hs:=ansistring(Args[doarg].VAnsiString)
                else
                  if CheckArg(vtUnicodeString,false) then
                    hs:=UnicodeString(Args[doarg].VUnicodeString)
                else
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
                  if CheckArg(vtAnsiString,false) then
                    hs:=ansistring(Args[doarg].VAnsiString)
                else
                  if CheckArg(vtUnicodeString,false) then
                    hs:=UnicodeString(Args[doarg].VUnicodeString)
                else
>>>>>>> origin/cpstrnew
                  if CheckArg(vtVariant,true) then
                    hs:=Args[doarg].VVariant^;
                Index:=Length(hs);
                If (Prec<>-1) and (Index>Prec) then
                  Index:=Prec;
                ToAdd:=Copy(hs,1,Index);
              end;
        'P' : Begin
              CheckArg(vtpointer,true);
<<<<<<< HEAD
<<<<<<< HEAD
              ToAdd:=TFormatString(HexStr(ptruint(Args[DoArg].VPointer),sizeof(Ptruint)*2));
=======
              ToAdd:=HexStr(ptruint(Args[DoArg].VPointer),sizeof(Ptruint)*2);
>>>>>>> graemeg/fixes_2_2
=======
              ToAdd:=HexStr(ptruint(Args[DoArg].VPointer),sizeof(Ptruint)*2);
>>>>>>> origin/fixes_2_2
              // Insert ':'. Is this needed in 32 bit ? No it isn't.
              // Insert(':',ToAdd,5);
              end;
        'X' : begin
              if Checkarg(vtinteger,false) then
                 begin
                   vq:=Cardinal(Args[Doarg].VInteger);
                   index:=16;
                 end
              else
                 if CheckArg(vtQWord, false) then
                   begin
                     vq:=Qword(Args[DoArg].VQWord^);
                     index:=31;
                   end
              else
                 begin
                   CheckArg(vtInt64,true);
                   vq:=Qword(Args[DoArg].VInt64^);
                   index:=31;
                 end;
              If Prec>index then
<<<<<<< HEAD
<<<<<<< HEAD
                ToAdd:=TFormatString(HexStr(int64(vq),index))
=======
                ToAdd:=HexStr(int64(vq),index)
>>>>>>> graemeg/fixes_2_2
=======
                ToAdd:=HexStr(int64(vq),index)
>>>>>>> origin/fixes_2_2
              else
                begin
                // determine minimum needed number of hex digits.
                Index:=1;
                While (qWord(1) shl (Index*4)<=vq) and (index<16) do
                  inc(Index);
                If Index>Prec then
                  Prec:=Index;
<<<<<<< HEAD
<<<<<<< HEAD
                ToAdd:=TFormatString(HexStr(int64(vq),Prec));
=======
                ToAdd:=HexStr(int64(vq),Prec);
>>>>>>> graemeg/fixes_2_2
=======
                ToAdd:=HexStr(int64(vq),Prec);
>>>>>>> origin/fixes_2_2
                end;
              end;
        '%': ToAdd:='%';
      end;
      If Width<>-1 then
        If Length(ToAdd)<Width then
          If not Left then
            ToAdd:=TFormatString(Space(Width-Length(ToAdd)))+ToAdd
          else
            ToAdd:=ToAdd+TFormatString(space(Width-Length(ToAdd)));
      Result:=Result+ToAdd;
      end;
    inc(ChPos);
    Oldpos:=ChPos;
    end;
end;
