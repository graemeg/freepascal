{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Michael Van Canneyt,
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
  { See symconst.pas tfloattype }
  treal_type = (
    rt_s32real,rt_s64real,rt_s80real,rt_sc80real,
    rt_c64bit,rt_currency,rt_s128real
  );
  { corresponding to single   double   extended   fixed      comp for i386 }

{$if not declared(mul_by_power10)}
  function mul_by_power10 (x : ValReal; power : integer) : ValReal; forward;
{$endif}

Procedure str_real (len,f : longint; d : ValReal; real_type :treal_type; out s : string);
{$ifdef SUPPORT_EXTENDED}
type
  TSplitExtended = packed record
    case byte of
      0: (bytes: Array[0..9] of byte);
      1: (words: Array[0..4] of word);
      2: (cards: Array[0..1] of cardinal; w: word);
  end;
const
  maxDigits = 17;
{$else}
{$ifdef SUPPORT_DOUBLE}
{$ifndef cpujvm}
type
  TSplitDouble = packed record
    case byte of
      0: (bytes: Array[0..7] of byte);
      1: (words: Array[0..3] of word);
      2: (cards: Array[0..1] of cardinal);
  end;
  {$endif}
const
  maxDigits = 15;
{$else}
{$ifdef SUPPORT_SINGLE}
type
  TSplitSingle = packed record
    case byte of
      0: (bytes: Array[0..3] of byte);
      1: (words: Array[0..1] of word);
      2: (cards: Array[0..0] of cardinal);
  end;
const
  maxDigits = 9;
{$endif SUPPORT_SINGLE}
{$endif SUPPORT_DOUBLE}
{$endif SUPPORT_EXTENDED}

type
  { the value in the last position is used for rounding }
  TIntPartStack = array[1..maxDigits+1] of valReal;

var
{$ifdef cpujvm}
  doublebits: int64;
{$endif}
  roundCorr, corrVal, factor : valReal;
  high_exp10_reduced,
  spos, endpos, fracCount: longint;
  correct, currprec: longint;
  temp : string;
  power : string[10];
  sign : boolean;
  dot : byte;
  fraczero, expMaximal: boolean;


  maxlen : longint;   { Maximal length of string for float }
  minlen : longint;   { Minimal length of string for float }
  explen : longint;   { Length of exponent, including E and sign.
                           Must be strictly larger than 2 }
const
  maxexp = 1e+35;   { Maximum value for decimal expressions }
  minexp = 1e-35;   { Minimum value for decimal expressions }
  zero   = '0000000000000000000000000000000000000000';

  procedure RoundStr(var s: string; lastPos: byte);
  var carry: longint;
  begin
    carry := 1;
    repeat
      s[lastPos] := chr(ord(s[lastPos])+carry);
      carry := 0;
      if s[lastPos] > '9' then
        begin
          s[lastPos] := '0';
          carry := 1;
        end;
      dec(lastPos);
    until carry = 0;
  end;

  procedure getIntPart(d: valreal);
  var
    intPartStack: TIntPartStack;
    intPart, stackPtr, endStackPtr, digits: longint;
    overflow: boolean;
  begin
{$ifdef DEBUG_NASM}
    writeln(stderr,'getintpart(d) entry');
{$endif DEBUG_NASM}
    { position in the stack (gets increased before first write) }
    stackPtr := 0;
    { number of digits processed }
    digits := 0;
    { did we wrap around in the stack? Necessary to know whether we should round }
    overflow :=false;
    { generate a list consisting of d, d/10, d/100, ... until d < 1.0 }
    while d > 1.0-roundCorr do
      begin
        inc(stackPtr);
        inc(digits);
        if stackPtr > maxDigits+1 then
          begin
            stackPtr := 1;
            overflow := true;
          end;
        intPartStack[stackPtr] := d;
        d := d / 10.0;
      end;
    { if no integer part, exit }
    if digits = 0 then
      exit;
    endStackPtr := stackPtr+1;
    if endStackPtr > maxDigits + 1 then
      endStackPtr := 1;
 { now, all digits are calculated using trunc(d*10^(-n)-int(d*10^(-n-1))*10) }
    corrVal := 0.0;
 { the power of 10 with which the resulting string has to be "multiplied" }
 { if the decimal point is placed after the first significant digit       }
    correct := digits-1;
{$ifdef DEBUG_NASM}
    writeln(stderr,'endStackPtr = ',endStackPtr);
{$endif DEBUG_NASM}
    repeat
      if (currprec > 0) then
        begin
          intPart:= trunc(intPartStack[stackPtr]-corrVal);
          dec(currPrec);
          inc(spos);
          temp[spos] := chr(intPart+ord('0'));
{$ifdef DEBUG_NASM}
    writeln(stderr,'stackptr =',stackptr,' intpart = ',intpart);
{$endif DEBUG_NASM}
          if temp[spos] > '9' then
            begin
              temp[spos] := chr(ord(temp[spos])-10);
              roundStr(temp,spos-1);
            end;
        end;
      corrVal := int(intPartStack[stackPtr]) * 10.0;
{$ifdef DEBUG_NASM}
    writeln(stderr,'trunc(corrval) = ',trunc(corrval));
{$endif DEBUG_NASM}
      dec(stackPtr);
      if stackPtr = 0 then
        stackPtr := maxDigits+1;
    until (overflow and (stackPtr = endStackPtr)) or
          (not overflow and (stackPtr = maxDigits+1)) or (currPrec = 0);
    { round if we didn't use all available digits yet and if the }
    { remainder is > 5                                           }
    if (overflow or
        (stackPtr < maxDigits+1)) then
      begin
        { we didn't use all available digits of the whole part -> make sure }
        { the fractional part is not used for rounding later                }
        currprec := -1;
        { instead, round based on the next whole digit }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if (int(intPartStack[stackPtr]-corrVal) >= 5.0) then
=======
        if (int(intPartStack[stackPtr]-corrVal+roundcorr) >= 5.0) then
>>>>>>> graemeg/fixes_2_2
=======
        if (int(intPartStack[stackPtr]-corrVal+roundcorr) >= 5.0) then
>>>>>>> origin/fixes_2_2
=======
        if (int(intPartStack[stackPtr]-corrVal) >= 5.0) then
>>>>>>> origin/fixes_2.4
           roundStr(temp,spos);
        end;
{$ifdef DEBUG_NASM}
    writeln(stderr,'temp at getintpart exit is = ',temp);
{$endif DEBUG_NASM}
  end;

  function reduce_exponent (d : ValReal; out scaled : ValReal) : longint;
  { Returns decimal exponent which was used for scaling, and a scaled value out }
  const
    C_LN10 = ln(10);
  var
    log10_d : longint;
  begin
    reduce_exponent := 0;
    if d<>0 then
      begin
       // get exponent approximation ["d" is assumed to be non-negative]
        log10_d:=trunc(ln(d)/C_LN10);
       // trying to stay at least 1 digit away from introducing integer/fractional part
        if log10_d > maxDigits+1 then
          reduce_exponent := log10_d-maxDigits
        else
          if log10_d < -(maxDigits+1) then
            reduce_exponent := log10_d+maxDigits
       // else
       //   the number is already suitable enough
      end;
    // do scaling if needed
    if reduce_exponent<>0
      then scaled := mul_by_power10(d,-reduce_exponent) // denormals should be handled properly by this call
      else scaled := d;
  end;

begin
  case real_type of
    rt_s32real :
      begin
         maxlen:=16;
         minlen:=8;
         explen:=4;
         { correction used with comparing to avoid rounding/precision errors }
         roundCorr := 1.1920928955e-07;
      end;
    rt_s64real :
      begin
         maxlen := 22;
         { correction used with comparing to avoid rounding/precision errors }
         roundCorr := 2.2204460493e-16;
         minlen:=9;
         explen:=5;
      end;
    rt_s80real,
    rt_sc80real:
      begin
         { Different in TP help, but this way the output is the same (JM) }
         maxlen:=25;
         minlen:=10;
         explen:=6;
         { correction used with comparing to avoid rounding/precision errors }
         roundCorr := 1.0842021725e-19;
      end;
    rt_c64bit  :
      begin
         maxlen:=23;
         minlen:=10;
         { according to TP (was 5) (FK) }
         explen:=6;
         { correction used with comparing to avoid rounding/precision errors }
         roundCorr := 2.2204460493e-16;
      end;
    rt_currency :
      begin
         { Different in TP help, but this way the output is the same (JM) }
         maxlen:=25;
         minlen:=10;
         explen:=0;
         { correction used with comparing to avoid rounding/precision errors }
         roundCorr := 1.0842021725e-19;
      end;
    rt_s128real  :
      begin
         { Different in TP help, but this way the output is the same (JM) }
         maxlen:=25;
         minlen:=10;
         explen:=6;
         { correction used with comparing to avoid rounding/precision errors }
         roundCorr := 1.0842021725e-19;
<<<<<<< HEAD
<<<<<<< HEAD
      end;
    else
      begin
        { keep JVM byte code verifier happy }
        maxlen:=0;
        minlen:=0;
        explen:=0;
        roundCorr:=0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;
    end;
  { check parameters }
  { default value for length is -32767 }
  if len=-32767 then
    len:=maxlen;
  { determine sign. before precision, needs 2 less calls to abs() }
{$ifndef endian_big}
{$ifdef SUPPORT_EXTENDED}
  { extended, format (MSB): 1 Sign bit, 15 bit exponent, 64 bit mantissa }
  sign := (TSplitExtended(d).w and $8000) <> 0;
  expMaximal := (TSplitExtended(d).w and $7fff) = 32767;
  fraczero := (TSplitExtended(d).cards[0] = 0) and
                  ((TSplitExtended(d).cards[1] and $7fffffff) = 0);
{$else SUPPORT_EXTENDED}
{$ifdef SUPPORT_DOUBLE}
{$ifdef FPC_DOUBLE_HILO_SWAPPED}
  { double, format (MSB): 1 Sign bit, 11 bit exponent, 52 bit mantissa }
  { high and low dword are swapped when using the arm fpa }
  sign := ((TSplitDouble(d).cards[0] shr 20) and $800) <> 0;
  expMaximal := ((TSplitDouble(d).cards[0] shr 20) and $7ff) = 2047;
  fraczero:= (TSplitDouble(d).cards[0] and $fffff = 0) and
              (TSplitDouble(d).cards[1] = 0);
{$else FPC_DOUBLE_HILO_SWAPPED}
  { double, format (MSB): 1 Sign bit, 11 bit exponent, 52 bit mantissa }
  sign := ((TSplitDouble(d).cards[1] shr 20) and $800) <> 0;
  expMaximal := ((TSplitDouble(d).cards[1] shr 20) and $7ff) = 2047;
  fraczero := (TSplitDouble(d).cards[1] and $fffff = 0) and
              (TSplitDouble(d).cards[0] = 0);
{$endif FPC_DOUBLE_HILO_SWAPPED}
{$else SUPPORT_DOUBLE}
{$ifdef SUPPORT_SINGLE}
  { single, format (MSB): 1 Sign bit, 8 bit exponent, 23 bit mantissa }
  sign := ((TSplitSingle(d).words[1] shr 7) and $100) <> 0;
  expMaximal := ((TSplitSingle(d).words[1] shr 7) and $ff) = 255;
  fraczero := (TSplitSingle(d).cards[0] and $7fffff = 0);
{$else SUPPORT_SINGLE}
  {$error No little endian floating type supported yet in real2str}
{$endif SUPPORT_SINGLE}
{$endif SUPPORT_DOUBLE}
{$endif SUPPORT_EXTENDED}
{$else endian_big}
{$ifdef SUPPORT_EXTENDED}
  {$error sign/NaN/Inf not yet supported for big endian CPU's in str_real}
{$else SUPPORT_EXTENDED}
{$ifdef SUPPORT_DOUBLE}
{$ifdef cpujvm}
  doublebits := JLDouble.doubleToLongBits(d);
  sign := doublebits<0;
  expMaximal := (doublebits shr (32+20)) and $7ff = 2047;
  fraczero:= (((doublebits shr 32) and $fffff) = 0) and
             (longint(doublebits)=0);
{$else cpujvm}
  { double, format (MSB): 1 Sign bit, 11 bit exponent, 52 bit mantissa }
  sign := ((TSplitDouble(d).cards[0] shr 20) and $800) <> 0;
  expMaximal := ((TSplitDouble(d).cards[0] shr 20) and $7ff) = 2047;
  fraczero:= (TSplitDouble(d).cards[0] and $fffff = 0) and
              (TSplitDouble(d).cards[1] = 0);
{$endif cpujvm}
{$else SUPPORT_DOUBLE}
{$ifdef SUPPORT_SINGLE}
  { single, format (MSB): 1 Sign bit, 8 bit exponent, 23 bit mantissa }
  sign := ((TSplitSingle(d).bytes[0] and $80)) <> 0;
  expMaximal := ((TSplitSingle(d).words[0] shr 7) and $ff) = 255;
  fraczero:= (TSplitSingle(d).cards[0] and $7fffff = 0);
{$else SUPPORT_SINGLE}
  {$error No big endian floating type supported yet in real2str}
{$endif SUPPORT_SINGLE}
{$endif SUPPORT_DOUBLE}
{$endif SUPPORT_EXTENDED}
{$endif endian}
  if expMaximal then
    if fraczero then
      if sign then
        temp := '-Inf'
      else temp := '+Inf'
    else temp := 'Nan'
  else
    begin
      {  d:=abs(d); this converts d to double so we loose precision }
      { for the same reason I converted d:=frac(d) to d:=d-int(d); (PM) }
      if sign then
        d:=-d;
      { determine precision : maximal precision is : }
      currPrec := maxlen-explen-2;
      { this is also the maximal number of decimals !!}
      if f>currprec then
        f:=currprec;
      { when doing a fixed-point, we need less characters.}
      if (f<0) {or ((d<>0) and ((d>maxexp) and (d>minexp)))} then
        begin
        { determine maximal number of decimals }
          if (len>=0) and (len<minlen) then
            len:=minlen;
          if (len>0) and (len<maxlen) then
            currprec:=len-explen-2;
        end;

      { leading zero, may be necessary for things like str(9.999:0:2) to }
      { be able to insert an extra character at the start of the string  }
      temp := ' 0';
      { position in the temporary output string }
      spos := 2;

      // workaround to make follow-up things go somewhat faster
      high_exp10_reduced := 0;
      case real_type of
      // blacklist, in order of increasing headache:
//?     rt_s32real :;   
          // ? needs additional testing to ensure any reasonable benefit
          //   without lost of accuracy due to an extra conversion
        rt_c64bit, rt_currency :;
          // no much sense to touch them
      else
      // acceptable:
      // ? rt_s32real [see above]
      //   rt_s64real
      //   rt_s80real, rt_sc80real
      // ? rt_s128real [have not tried]
        high_exp10_reduced := reduce_exponent(d,d);
      end;
  
      { get the integer part }
      correct := 0;
      GetIntPart(d);

      inc(correct,high_exp10_reduced); // end of workaround

      { now process the fractional part }
      if d > 1.0- roundCorr then
        d := frac(d);
      { if we have to round earlier than the amount of available precision, }
      { only calculate digits up to that point                              }
      if (f >= 0) and (currPrec > f) then
        currPrec := f;
      { if integer part was zero, go to the first significant digit of the }
      { fractional part                                                    }
      { make sure we don't get an endless loop if d = 0                    }
      if (spos = 2) and (d <> 0.0) then
        begin
         { take rounding errors into account }
          while d < 0.1-roundCorr do
            begin
              d := d * 10.0;
              dec(correct);
              { adjust the precision depending on how many digits we  }
              { already "processed" by multiplying by 10, but only if }
              { the amount of precision is specified                  }
              if f >= 0 then
                dec(currPrec);
            end;
          dec(correct);
        end;
      { current length of the output string in endPos }
      endPos := spos;
      { always calculate at least 1 fractional digit for rounding }
      if (currPrec >= 0) then
        begin
          corrVal := 0.5;
          factor := 1;
          for fracCount := 1 to currPrec do
            factor := factor * 10.0;
          corrval := corrval / factor;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
          { for single, we may write more significant digits than are available,
            so the rounding correction itself can show up -> don't round in that
            case
          }
          if real_type<>rt_s32real then
            d:=d+d*roundCorr;
=======
          if (d<>0.0) then
            d:=d+roundCorr;
>>>>>>> graemeg/fixes_2_2
=======
          if (d<>0.0) then
            d:=d+roundCorr;
>>>>>>> origin/fixes_2_2
          if d >= corrVal then
            d := d + corrVal;
          if int(d) = 1 then
            begin
              roundStr(temp,spos);
              d := frac(d);
<<<<<<< HEAD
=======
              if (f < 0) then
                begin
                  dec(currprec);
                  if (currprec=0) then
                    begin
                      inc(spos);
                      temp[spos]:='0';
                    end;
                end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
            end;
          { calculate the necessary fractional digits }
          for fracCount := 1 to currPrec do
            begin
              if d > 1.0 then
                d := frac(d) * 10.0
              else d := d * 10.0;
              inc(spos);
              temp[spos] := chr(trunc(d)+ord('0'));
              if temp[spos] > '9' then
                { possible because trunc and the "*10.0" aren't exact :( }
                begin
                  temp[spos] := chr(ord(temp[spos]) - 10);
                  roundStr(temp,spos-1);
                end;
            end;
          { new length of string }
          endPos := spos;
        end;
      setLength(temp,endPos);
      { delete leading zero if we didn't need it while rounding at the }
      { string level                                                   }
      if temp[2] = '0' then
        delete(temp,2,1)
      { the rounding caused an overflow to the next power of 10 }
      else inc(correct);
      if sign then
        temp[1] := '-';
      if (f<0) or (correct>(round(ln(maxexp)/ln(10)))) then
        begin
          insert ('.',temp,3);
          str(abs(correct),power);
          if length(power)<explen-2 then
            power:=copy(zero,1,explen-2-length(power))+power;
          if correct<0 then
            power:='-'+power
          else
            power:='+'+power;
          temp:=temp+'E'+power;
        end
      else
        begin
          if not sign then
            begin
              delete(temp,1,1);
              dot := 2
            end
          else
            dot := 3;
          { set zeroes and dot }
          if correct>=0 then
            begin
              if length(temp)<correct+dot+f-1 then
                temp:=temp+copy(zero,1,correct+dot+f-length(temp));
              insert ('.',temp,correct+dot);
            end
          else
            begin
              correct:=abs(correct);
              insert(copy(zero,1,correct),temp,dot-1);
              insert ('.',temp,dot);
            end;
          { correct length to fit precision }
          if f>0 then
            setlength(temp,pos('.',temp)+f)
          else
            setLength(temp,pos('.',temp)-1);
        end;
    end;
    if length(temp)<len then
      s:=space(len-length(temp))+temp
    else s:=temp;
end;


Procedure str_real_iso (len,f : longint; d : ValReal; real_type :treal_type; out s : string);
  var
   i : Integer;
  begin
    str_real(len,f,d,real_type,s);
    for i:=1 to Length(s) do
      if s[i]='E' then
        s[i]:='e';
  end;


