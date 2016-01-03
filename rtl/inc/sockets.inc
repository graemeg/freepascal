{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{******************************************************************************
                     Text File Writeln/ReadLn Support
******************************************************************************}


Procedure OpenSock(var F:Text);
begin
  if textrec(f).handle=UnusedHandle then
   textrec(f).mode:=fmclosed
  else
   case textrec(f).userdata[1] of
     S_OUT : textrec(f).mode:=fmoutput;
     S_IN : textrec(f).mode:=fminput;
   else
    textrec(f).mode:=fmclosed;
   end;
end;



procedure iosock(var f:text);

var r:sizeint;
    def_error:word;

begin
  with textrec(f) do
    begin
      case mode of
        fmoutput:
          begin
            repeat
{$ifdef use_readwrite}
              r:=fpwrite(handle,bufptr^,bufpos);
{$else}
              r:=fpsend(handle,bufptr,bufpos,0);
{$endif}
            until (r<>-1) or (SocketError <> EsockEINTR);
            bufend:=r;
            def_error:=101; {File write error.}
          end;
        fminput:
          begin
            repeat
{$ifdef use_readwrite}
              r:=fpread(handle,bufptr^,bufsize);
{$else}
              r:=fprecv(handle,bufptr,bufsize,0);
{$endif}
            until (r<>-1) or (SocketError <> EsockEINTR);
            bufend:=r;
            def_error:=100; {File read error.}
          end;
      end;
      if r=-1 then
        case SocketError of
          EsockEBADF:
{          EsysENOTSOCK:}   {Why is this constant not defined? (DM)}
            inoutres:=6;    {Invalid file handle.}
          EsockEFAULT:
            inoutres:=217;
          EsockEINVAL:
            inoutres:=218;
        else
          inoutres:=def_error;
        end;
      bufpos:=0;
    end;
end;



Procedure FlushSock(var F:Text);
begin
  if (textrec(f).mode=fmoutput) and (textrec(f).bufpos<>0) then
   begin
     IOSock(f);
     textrec(f).bufpos:=0;
   end;
end;



Procedure CloseSock(var F:text);
begin
  { Nothing special has to be done here }
end;



Procedure Sock2Text(Sock:Longint;Var SockIn,SockOut:Text);
{
 Set up two Pascal Text file descriptors for reading and writing)
}
begin
{ First the reading part.}
  Assign(SockIn,'.');
  Textrec(SockIn).Handle:=Sock;
  Textrec(Sockin).userdata[1]:=S_IN;
  TextRec(SockIn).OpenFunc:=@OpenSock;
  TextRec(SockIn).InOutFunc:=@IOSock;
  TextRec(SockIn).FlushFunc:=@FlushSock;
  TextRec(SockIn).CloseFunc:=@CloseSock;
  TextRec(SockIn).Mode := fmInput;
  Case DefaultTextLineBreakStyle Of
    tlbsLF: TextRec(sockin).LineEnd := #10;
    tlbsCRLF: TextRec(sockin).LineEnd := #13#10;
    tlbsCR: TextRec(sockin).LineEnd := #13;
   End;
{ Now the writing part. }
  Assign(SockOut,'.');
  Textrec(SockOut).Handle:=Sock;
  Textrec(SockOut).userdata[1]:=S_OUT;
  TextRec(SockOut).OpenFunc:=@OpenSock;
  TextRec(SockOut).InOutFunc:=@IOSock;
  TextRec(SockOut).FlushFunc:=@FlushSock;
  TextRec(SockOut).CloseFunc:=@CloseSock;
  TextRec(SockOut).Mode := fmOutput;

   Case DefaultTextLineBreakStyle Of
    tlbsLF: TextRec(sockout).LineEnd := #10;
    tlbsCRLF: TextRec(sockout).LineEnd := #13#10;
    tlbsCR: TextRec(sockout).LineEnd := #13;
   End;
end;


{******************************************************************************
                                Untyped File
******************************************************************************}

Procedure Sock2File(Sock:Longint;Var SockIn,SockOut:File);
begin
{Input}
  Assign(SockIn,'.');
  FileRec(SockIn).Handle:=Sock;
  FileRec(SockIn).RecSize:=1;
  FileRec(Sockin).userdata[1]:=S_IN;
  FileRec(SockIn).Mode := fmInput;

{Output}
  Assign(SockOut,'.');
  FileRec(SockOut).Handle:=Sock;
  FileRec(SockOut).RecSize:=1;
  FileRec(SockOut).userdata[1]:=S_OUT;
  FileRec(SockOut).Mode := fmOutput;
end;

{******************************************************************************
                               InetSock
******************************************************************************}

Function DoAccept(Sock:longint;Var addr:TInetSockAddr):longint;

Var AddrLen : Longint;

begin
  AddrLEn:=SizeOf(Addr);
  DoAccept:=fpaccept(Sock,@Addr,@AddrLen);
end;

Function DoConnect(Sock:longint;const addr: TInetSockAddr): Boolean;

begin
  DoConnect:=fpconnect(Sock,@Addr,SizeOF(TInetSockAddr)) = 0;
end;

{$warnings off}

Function Connect(Sock:longint;const addr: TInetSockAddr;var SockIn,SockOut:text):Boolean;

begin
  Connect:=DoConnect(Sock,addr);
  If Connect then
     Sock2Text(Sock,SockIn,SockOut);
end;

Function Connect(Sock:longint;const addr:TInetSockAddr;var SockIn,SockOut:file):Boolean;

begin
  Connect:=DoConnect(Sock,addr);
  If Connect then
     Sock2File(Sock,SockIn,SockOut);
end;

Function Accept(Sock:longint;var addr:TInetSockAddr;var SockIn,SockOut:text):Boolean;
var
  s : longint;
begin
  S:=DoAccept(Sock,addr);
  if S>0 then
   begin
     Sock2Text(S,SockIn,SockOut);
     Accept:=true;
   end
  else
   Accept:=false;
end;

Function Accept(Sock:longint;var addr:TInetSockAddr;var SockIn,SockOut:File):Boolean;
var
  s : longint;
begin
  S:=DoAccept(Sock,addr);
  if S>0 then
   begin
     Sock2File(S,SockIn,SockOut);
     Accept:=true;
   end
  else
   Accept:=false;
end;

{$warnings on}

type thostaddr= packed array[1..4] of byte;

function htonl( host : longint):longint; inline;

begin
{$ifdef FPC_BIG_ENDIAN}
  htonl:=host;
{$else}
  htonl:=THostAddr(host)[4];
  htonl:=htonl or ( (THostAddr(host)[3]) shl 8);
  htonl:=htonl or ( (THostAddr(host)[2]) shl 16);
  htonl:=htonl or ( (THostAddr(host)[1]) shl 24);
{$endif}
end;

Function NToHl (Net : Longint) : Longint; inline;

begin
{$ifdef FPC_BIG_ENDIAN}
  ntohl:=net;
{$else}
  ntohl:=THostAddr(Net)[4];
  ntohl:=ntohl or ( (THostAddr(Net)[3]) shl 8);
  ntohl:=ntohl or ( (THostAddr(Net)[2]) shl 16);
  ntohl:=ntohl or ( (THostAddr(Net)[1]) shl 24);
{$endif}
end;

function htons( host : word):word; inline;

begin
{$ifdef FPC_BIG_ENDIAN}
  htons:=host;
{$else}
  htons:=swap(host);
{$endif}
end;

Function NToHs (Net : word):word; inline;

begin
{$ifdef FPC_BIG_ENDIAN}
  ntohs:=net;
{$else}
  ntohs:=swap(net);
{$endif}
end;

Type array4int = array[1..4] of byte;

function NetAddrToStr (Entry : in_addr) : AnsiString;

Var Dummy : Ansistring;
    i,j   : Longint;

begin
  NetAddrToStr:='';
  j:=entry.s_addr;
  For I:=1 to 4 do
   begin
     Str(array4int(j)[i],Dummy);
     NetAddrToStr:=NetAddrToStr+Dummy;
     If I<4 Then
       NetAddrToStr:=NetAddrToStr+'.';
   end;
end;

function HostAddrToStr (Entry : in_addr) : AnsiString;

Var x: in_addr;

begin
  x.s_addr:=htonl(entry.s_addr);
  HostAddrToStr:=NetAddrToStr(x);
end;

function StrToHostAddr(IP : AnsiString) : in_addr ;

Var
    Dummy : AnsiString;
    I,j,k     : Longint;
    Temp : in_addr;

begin
  strtohostaddr.s_addr:=0;              //:=NoAddress;
  For I:=1 to 4 do
    begin
      If I<4 Then
        begin
          J:=Pos('.',IP);
          If J=0 then
            exit;
          Dummy:=Copy(IP,1,J-1);
          Delete (IP,1,J);
        end
       else
         Dummy:=IP;
      Val (Dummy,k,J);
      array4int(temp.s_addr)[i]:=k;
      If J<>0 then Exit;
   end;
   strtohostaddr.s_addr:=ntohl(Temp.s_addr);
end;

function StrToNetAddr(IP : AnsiString) : in_addr;

begin
  StrToNetAddr.s_addr:=htonl(StrToHostAddr(IP).s_addr);
end;

Function HostToNet (Host : in_addr):in_addr;

begin
  HostToNet.s_addr:=htonl(host.s_addr);
end;

Function NetToHost (Net : in_addr) : in_addr;

begin
  NetToHost.s_addr:=ntohl(net.s_addr);
end;

Function HostToNet (Host : Longint) : Longint;

begin
  HostToNet:=htonl(host);
end;

Function NetToHost (Net : Longint) : Longint;

begin
  NetToHost:=ntohl(net);
end;

Function ShortHostToNet (Host : Word) : Word;

begin
  ShortHostToNet:=htons(host);
end;

Function ShortNetToHost (Net : Word) : Word;

begin
  ShortNEtToHost:=ntohs(net);
end;

const digittab : shortstring = ('0123456789ABCDEF');

function lclinttohex (i:integer;digits:longint): ansistring;

begin
  SetLength(lclinttohex,4);
  lclinttohex[4]:=digittab[1+(i and 15)];
  lclinttohex[3]:=digittab[1+((i shr 4) and 15)];
  lclinttohex[2]:=digittab[1+((i shr 8) and 15)];
  lclinttohex[1]:=digittab[1+((i shr 12) and 15)];;
end;

function HostAddrToStr6 (Entry : TIn6_Addr) :ansiString;
var
  i: byte;
  zr1,zr2: set of byte;
  zc1,zc2: byte;
  have_skipped: boolean;
begin
  zr1 := [];
  zr2 := [];
  zc1 := 0;
  zc2 := 0;
  for i := 0 to 7 do begin
    if Entry.u6_addr16[i] = 0 then begin
      include(zr2, i);
      inc(zc2);
    end else begin
      if zc1 < zc2 then begin
        zc1 := zc2;
        zr1 := zr2;
        zc2 := 0; zr2 := [];
      end;
    end;
  end;
  if zc1 < zc2 then begin
    zc1 := zc2;
    zr1 := zr2;
  end;
  SetLength(HostAddrToStr6, 8*5-1);
  SetLength(HostAddrToStr6, 0);
  have_skipped := false;
  for i := 0 to 7 do begin
    if not (i in zr1) then begin
      if have_skipped then begin
        if HostAddrToStr6 = ''
          then HostAddrToStr6 := '::'
          else HostAddrToStr6 := HostAddrToStr6 + ':';
        have_skipped := false;
      end;
      // FIXME: is that shortnettohost really proper there? I wouldn't be too sure...
      HostAddrToStr6 := HostAddrToStr6 +lclIntToHex(ShortNetToHost(Entry.u6_addr16[i]), 1) + ':';
    end else begin
      have_skipped := true;
    end;
  end;
  if have_skipped then
    if HostAddrToStr6 = ''
      then HostAddrToStr6 := '::'
      else HostAddrToStr6 := HostAddrToStr6 + ':';

  if HostAddrToStr6 = '' then HostAddrToStr6 := '::';
  if not (7 in zr1) then
    SetLength(HostAddrToStr6, Length(HostAddrToStr6)-1);
end;


function StrToHostAddr6(IP : String) : TIn6_addr;
Var Part   : String;
    IPv6   : TIn6_addr;
    P,J    : Integer;
    W      : Word;
    Index  : Integer;
    ZeroAt : Integer;
Begin
  FillChar(IPv6,SizeOf(IPv6),0);
  { Every 16-bit block is converted at its own and stored into Result. When }
  { the '::' zero-spacer is found, its location is stored. Afterwards the   }
  { address is shifted and zero-filled.                                     }
  Index := 0; ZeroAt := -1;
  J := 0;
  P := Pos(':',IP);
  While (P > 0) and (Length(IP) > 0) and (Index < 8) do
    Begin
      Part := '$'+Copy(IP,1,P-1);
      Delete(IP,1,P);
      if Length(Part) > 1 then  { is there a digit after the '$'? }
        Val(Part,W,J)
      else W := 0;
      IPv6.u6_addr16[Index] := HtoNS(W);
      if J <> 0 then
        Begin
          FillChar(IPv6,SizeOf(IPv6),0);
          Exit;
        End;
      if IP[1] = ':' then
        Begin
          ZeroAt := Index;
          Delete(IP,1,1);
        End;
      Inc(Index);
      P := Pos(':',IP); if P = 0 then P := Length(IP)+1;
    End;
  { address      a:b:c::f:g:h }
  { Result now   a : b : c : f : g : h : 0 : 0, ZeroAt = 2, Index = 6 }
  { Result after a : b : c : 0 : 0 : f : g : h }
  if ZeroAt >= 0 then
    Begin
      Move(IPv6.u6_addr16[ZeroAt+1],IPv6.u6_addr16[(8-Index)+ZeroAt+1],2*(Index-ZeroAt-1));
      FillChar(IPv6.u6_addr16[ZeroAt+1],2*(8-Index),0);
    End;

  StrToHostAddr6:=IPv6;
End;


function NetAddrToStr6 (Entry : TIn6_Addr) : ansiString;
begin
  netaddrtostr6 := HostAddrToStr6((Entry));
end;

function StrToNetAddr6(IP : ansiString) : TIn6_Addr;
begin
  StrToNetAddr6 := StrToHostAddr6(IP);
end;


