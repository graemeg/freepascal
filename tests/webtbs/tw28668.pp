{ %cpu=i386 }
{ %opt=-Cg }
{ %norun }

{$mode delphi}

unit tw28668;
interface

type
  TMapRef=class of TBaseMap;
  TBaseMap=class
  public
    class function BinToBase64(b:byte):ansichar;virtual;abstract;
    class function Base64ToBin(ch:ansichar):byte;virtual;abstract;
  end;

  TSafeMap=class(TBaseMap)
  public
    class function BinToBase64(b:byte):ansichar;override;
    class function Base64ToBin(ch:ansichar):byte;override;
  end;

  TpeMap=class(TSafeMap)
    class function BinToBase64(ch:byte):ansichar;override;
    class function Base64ToBin(ch:ansichar):byte;override;
  end;


function simple_encode_safe64(const Buf;cb:integer;const map:TMapRef=nil):string;
function simple_decode_safe64(S:string;const map:TMapRef=nil):string;

implementation

type
	TByteArray=array[0..255]of byte;
	PByteArray=^TByteArray;

class function TSafeMap.BinToBase64(b:byte):ansichar;
const
  Safe64:array[0..63] of ansichar = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_';
begin
  Result:=Safe64[b];
end;

class function TSafeMap.Base64ToBin(ch:ansichar):byte;
begin
  case ch of
  'A'..'Z': Result:=ord(ch)-ord('A');
  'a'..'z': Result:=ord(ch)-ord('a')+26;
  '0'..'9': Result:=ord(ch)-ord('0')+52;
  '-','+': Result:=62;
  '_','/': Result:=63;
  else
    Result:= 255;
  end;
end;

class function TpeMap.BinToBase64(ch:byte):Ansichar;
begin
  Result:=ansichar(ch+ord(';'));
end;

class function TpeMap.Base64ToBin(ch:ansichar):byte;
begin
  Result:=ord(ch)-ord(';');
end;

function simple_encode_safe64(const Buf;cb:integer;const map:TMapRef=nil):string;
var
  stemp:string;
  BufArr:TByteArray absolute Buf;
  i,c:integer;
  AMap:TMapRef;
begin
  if map=nil then
    AMap:=TSafeMap
  else
    AMap:=map;

  setlength(stemp,4);
  Result:='';
  c:=0;
  for i:=0 to cb-1 do
  begin
    if c=0 then fillchar(stemp[1],4,0);
    case c of
      0: begin
        stemp[1]:=Amap.BinToBase64(BufArr[i] shr 2);
        stemp[2]:=char((BufArr[i]and $03) shl 4);
      end;
      1: begin
        stemp[2]:=Amap.BinToBase64(ord(stemp[2]) or BufArr[i] shr 4);
        stemp[3]:=char((BufArr[i]and $0F) shl 2);
      end;
      2: begin
        stemp[3]:=Amap.BinToBase64(ord(stemp[3]) or BufArr[i] shr 6);
        stemp[4]:=Amap.BinToBase64(BufArr[i] and $3F);
      end;
    end;
    inc(c);
    if c=3
    then begin
      Result:=Result+stemp;
      c:=0;
    end;
  end;
  if c<>0 then
  begin
    stemp[c+1]:=Amap.BinToBase64(ord(stemp[c+1]));
    Result:=Result+copy(stemp,1,c+1);
  end;
end;

function simple_decode_safe64(S:string;const map:TMapRef=nil):string;
var
  i,c,o:integer;
  ch:byte;
  AMap:TMapRef;
begin
  if map=nil then
    AMap:=TSafeMap
  else
    AMap:=map;

  setlength(Result,length(S)*3 div 4);
  c:=0;
  o:=1;
  for i:=1 to length(S) do
  begin
    ch:=Amap.Base64ToBin(S[i]);
    if (ch<64) then
    begin
      case c of
        0: Result[o]:=chr(ch shl 2);
        1: begin
          Result[o]:=chr(ord(Result[o]) or ch shr 4);
          inc(o);
          Result[o]:=chr(ch shl 4);
        end;
        2: begin
          Result[o]:=chr(ord(Result[o]) or ch shr 2);
          inc(o);
          Result[o]:=chr(ch shl 6);
        end;
        3: begin
          Result[o]:=chr(ord(Result[o]) or ch);
          inc(o);
        end;
      end;
      inc(c);
      if c>=4 then c:=0;
    end;
  end;
  setlength(result,o-1);
end;

end.
