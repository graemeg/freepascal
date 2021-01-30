{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by the Free Pascal development team

    Implement networking routines.
    
    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}

unit netdb;
{
  WARNING
  This unit hardly does any error checking. For example, stringfromlabel
  could easily be exploited by  someone sending malicious UDP packets in
  order to crash  your program.  So if you really want to depend on this
  in critical programs then you'd better fix a lot of code in here.
  Otherwise, it appears to work pretty well.

  When compiling this unit with the FPC_USE_LIBC defined, the warning above
  can be ignored, since the libc implementation should be robust.
}

Interface

Uses Sockets;

{$IFDEF OS2}
(* ETC directory location determined by environment variable ETC *)
 {$DEFINE ETC_BY_ENV}
(* Use names supported also on non-LFN drives like plain FAT-16. *)
 {$DEFINE SFN_VERSION}
{$ENDIF OS2}
{$IFDEF GO32V2}
 {$DEFINE ETC_BY_ENV}
 {$DEFINE SFN_VERSION}
{$ENDIF GO32V2}
{$IFDEF WATCOM}
 {$DEFINE ETC_BY_ENV}
 {$DEFINE SFN_VERSION}
{$ENDIF WATCOM}

{$IFDEF UNIX}
(* ETC directory location hardcoded to /etc/ *)
 {$DEFINE UNIX_ETC}
{$ENDIF UNIX}

{$if defined(android)}
  {$define FPC_USE_LIBC}
{$endif}

Type
  THostAddr = in_addr;		// historical aliases for these.
  THostAddr6= Tin6_addr;
  TNetAddr  = THostAddr;	// but in net order.

Const
  MaxResolveAddr = 10;
{$ifndef FPC_USE_LIBC}
  DNSPort        = 53;
  SServicesFile  = 'services';
  SHostsFile     = 'hosts';
  SNetworksFile  = 'networks';
{$IFDEF SFN_VERSION}
  SProtocolFile  = 'protocol';
  SResolveFile   = 'resolv';
 {$IFDEF OS2}
(* Peculiarity of OS/2 - depending on the used TCP/IP version, *)
(* the file differs slightly in name and partly also content.  *)
   SResolveFile2  = 'resolv2';
 {$ENDIF OS2}
{$ELSE SFN_VERSION}
  SProtocolFile  = 'protocols';
  SResolveFile   = 'resolv.conf';
{$ENDIF SFN_VERSION}

  MaxRecursion = 10;
  MaxIP4Mapped = 10;

  { from http://www.iana.org/assignments/dns-parameters }
  DNSQRY_A     = 1;                     // name to IP address
  DNSQRY_AAAA  = 28;                    // name to IP6 address
  DNSQRY_A6    = 38;                    // name to IP6 (new)
  DNSQRY_PTR   = 12;                    // IP address to name 
  DNSQRY_MX    = 15;                    // name to MX 
  DNSQRY_TXT   = 16;                    // name to TXT
  DNSQRY_CNAME = 5;
  DNSQRY_SOA   = 6;
  DNSQRY_NS    = 2;
  DNSQRY_SRV   = 33;

  // Flags 1
  QF_QR     = $80;
  QF_OPCODE = $78;
  QF_AA     = $04;
  QF_TC     = $02;  // Truncated.
  QF_RD     = $01;

  // Flags 2
  QF_RA     = $80;
  QF_Z      = $70;
  QF_RCODE  = $0F;

var
  EtcPath: string;
{$endif FPC_USE_LIBC}

Type
  TDNSRcode = (rcNoError, rcFormatError,rcServFail,rcNXDomain,
    rcNotImpl,rcRefused,rcReserved,rcInvalid);
  TDNSServerArray = Array of THostAddr;
  TServiceEntry = record
    Name     : String;
    Protocol : String;
    Port     : Word;
    Aliases  : String;
  end;
     
  THostEntry = record
    Name : String;
    Addr : THostAddr;
    Aliases : String;
  end;
  PHostEntry = ^THostEntry;
  THostEntryArray = Array of THostEntry;

  THostEntry6 = record
    Name : String;
    Addr : THostAddr6;
    Aliases : String;
  end;
  PHostEntry6 = ^THostEntry6;
  THostEntry6Array = Array of THostEntry6;
  
  TNetworkEntry = Record
    Name : String;
    Addr : TNetAddr;
    Aliases : String;
  end;  
  PNetworkEntry = ^TNetworkEntry;

  TProtocolEntry = Record
    Name : String;
    Number : integer;
    Aliases : String;
  end;  
  PProtocolEntry = ^TProtocolEntry;

  PHostListEntry = ^THostListEntry;
  THostListEntry = Record
    Entry : THostEntry;
    Next : PHostListEntry;
  end;

{$ifndef FPC_USE_LIBC}

Type 
  TPayLoad  = Array[0..511] of Byte;
  TPayLoadTCP = Array[0 .. 65535] of Byte;

  TDNSHeader = packed Record
    id      : Array[0..1] of Byte;
    flags1  : Byte;
    flags2  : Byte;
    qdcount : word;
    ancount : word;
    nscount : word;
    arcount : word;
  end;

  TQueryData = packed Record
    h: TDNSHeader;
    Payload : TPayLoad;
  end;

  TQueryDataLength = packed record
    length: Word;
    hpl: TQueryData;
  end;

  TQueryDataLengthTCP = packed Record
    length: Word;
    h: TDNSHeader;
    Payload : TPayLoadTCP;
  end;

  PRRData = ^TRRData;
  TRRData = Packed record       // RR record
    Atype    : Word;            // Answer type
    AClass   : Word;
    TTL      : Cardinal;
    RDLength : Word;
  end;

  TRRNameData = packed record
    RRName   : ShortString;
    RRMeta   : TRRData;
    RDataSt  : Word;
  end;
  TRRNameDataArray = array of TRRNameData;

  TDNSDomainName = ShortString;
  TDNSRR_SOA = packed record
    mname, rname: TDNSDomainName;
    serial,refresh,retry,expire,min: Cardinal;
  end;
  TDNSRR_MX = packed record
    preference: Word;
    exchange: TDNSDomainName;
  end;
  TDNSRR_SRV = packed record
    priority, weight, port: Word;
    target: TDNSDomainName;
  end;

Var
  DNSServers            : TDNSServerArray;
  DNSOptions            : String;
  DefaultDomainList     : String;
  CheckResolveFileAge   : Boolean; 
  CheckHostsFileAge     : Boolean; 
  TimeOutS,TimeOutMS    : Longint;
  
{$ifdef android}
Function GetDNSServers : Integer;
{$else}
Function GetDNSServers(FN : String) : Integer;
{$endif android}
{$endif FPC_USE_LIBC}

// Addresses are returned in the net byte order
Function ResolveName(HostName : String; Var Addresses : Array of THostAddr) : Integer;
Function ResolveName6(HostName : String; Var Addresses : Array of THostAddr6) : Integer;

// HostAddr is specified in the host byte order
Function ResolveAddress(HostAddr : THostAddr; Var Addresses : Array of String) : Integer;
Function ResolveAddress6(HostAddr: THostAddr6; var Addresses: Array of string) : Integer;

function IN6_IS_ADDR_V4MAPPED(HostAddr: THostAddr6): boolean;

// H.Addr is returned in the net byte order
Function ResolveHostByName(HostName : String; Var H : THostEntry) : Boolean;
// HostAddr is specified in the host byte order
Function ResolveHostByAddr(HostAddr : THostAddr; Var H : THostEntry) : Boolean;

Function ResolveHostByName6(Hostname : String; Var H : THostEntry6) : Boolean;
Function ResolveHostByAddr6(HostAddr : THostAddr6; Var H : THostEntry6) : Boolean;

// H.Addr is returned in the host byte order
Function GetHostByName(HostName: String;  Var H : THostEntry) : boolean;
// Addr is specified in the host byte order
Function GetHostByAddr(Addr: THostAddr;  Var H : THostEntry) : boolean;

// N.Addr is returned in the net byte order
Function GetNetworkByName(NetName: String; Var N : TNetworkEntry) : boolean;
// Addr is specified in the host byte order
Function GetNetworkByAddr(Addr: THostAddr; Var N : TNetworkEntry) : boolean;

// E.Port is returned in the host byte order
Function GetServiceByName(Const Name,Proto : String; Var E : TServiceEntry) : Boolean;
// Port is specified in the host byte order
Function GetServiceByPort(Port : Word;Const Proto : String; Var E : TServiceEntry) : Boolean;

Function GetProtocolByName(ProtoName: String;  Var H : TProtocolEntry) : boolean;
Function GetProtocolByNumber(proto: Integer;  Var H : TProtocolEntry) : boolean;

{$ifndef FPC_USE_LIBC}
Function ProcessHosts(FileName : String) : PHostListEntry;
Function FreeHostsList(var List : PHostListEntry) : Integer;
Procedure HostsListToArray(var List : PHostListEntry; Var Hosts : THostEntryArray; FreeList : Boolean);

Procedure CheckResolveFile;
Function Query(Resolver : Integer; Var Qry,Ans : TQueryData; QryLen : Integer; Var AnsLen : Integer) : Boolean;
function QueryTCP(Resolver : Integer; Var Qry: TQueryDataLength;
  var Ans: TQueryDataLengthTCP; QryLen : Integer; Var AnsLen : Integer) : Boolean;
Function BuildPayLoad(Var Q : TQueryData; Name : String; RR : Word; QClass : Word) : Integer;
Function BuildPayLoadTCP(Var Q : TQueryDataLength; Name : String; RR : Word; QClass : Word) : Integer;

Function SkipAnsQueries(Var Ans : TQueryData; L : Integer) : integer;
Function SkipAnsQueries(Var Ans : TQueryDataLengthTCP; L : Integer) : integer;

function stringfromlabel(pl: TPayLoad; var start: Integer): string;
function stringfromlabel(pl: TPayLoadTCP; var start: Integer): string;
Function CheckAnswer(Const Qry : TDNSHeader; Var Ans : TDNSHeader) : Boolean;
function IsValidAtype(atype: Word): Boolean;

function IsTruncated(R: TDNSHeader): Boolean;
function GetRcode(R: TDNSHeader): TDNSRcode;
function GetFixlenStr(pl: TPayLoad; startidx: Cardinal; len: Byte;
  out res: ShortString): Byte;
function GetFixlenStr(pl: TPayLoadTCP; startidx: Cardinal; len: Byte;
  out res: ShortString): Byte;

function NextNameRR(const pl: TPayLoadTCP; start: Word;
  out RRName: TRRNameData): Boolean;
function NextNameRR(const pl: TPayLoad; start: Word;
  out RRName: TRRNameData): Boolean;

function GetRRrecords(const pl: TPayloadTCP; var Start: Word; Count: Word):
  TRRNameDataArray;
function GetRRrecords(const pl: TPayload; var Start: Word; Count: Word):
  TRRNameDataArray;

function DnsLookup(dn: String; qtype: Word; out Ans: TQueryData;
  out AnsLen: Longint): Boolean;
function DnsLookup(dn: String; qtype: Word; out Ans: TQueryDataLengthTCP;
  out AnsLen: Longint): Boolean;

function DNSRRGetA(const RR: TRRNameData; const pl: TPayLoadTCP;
  out IP: THostAddr): Boolean;
function DNSRRGetA(const RR: TRRNameData; const pl: TPayLoad;
  out IP: THostAddr): Boolean;
function DNSRRGetCNAME(const RR: TRRNameData; const pl: TPayLoad;
  out cn: TDNSDomainName): Boolean;
function DNSRRGetCNAME(const RR: TRRNameData; const pl: TPayLoadTCP;
  out cn: TDNSDomainName): Boolean;
function DNSRRGetAAAA(const RR: TRRNameData; const pl: TPayLoadTCP;
  out IP: THostAddr6): Boolean;
function DNSRRGetAAAA(const RR: TRRNameData; const pl: TPayLoad;
  out IP: THostAddr6): Boolean;
function DNSRRGetNS(const RR: TRRNameData; const pl: TPayLoadTCP;
  out NSName: TDNSDomainName): Boolean;
function DNSRRGetNS(const RR: TRRNameData; const pl: TPayLoad;
  out NSName: TDNSDomainName): Boolean;
function DNSRRGetSOA(const RR: TRRNameData; const pl: TPayLoadTCP;
  out dnssoa: TDNSRR_SOA): Boolean;
function DNSRRGetSOA(const RR: TRRNameData; const pl: TPayLoad;
  out dnssoa: TDNSRR_SOA): Boolean;
function  DNSRRGetText(const RR: TRRNameData; const pl: TPayLoad;
  out dnstext: AnsiString): Boolean;
function  DNSRRGetText(const RR: TRRNameData; const pl: TPayLoadTCP;
  out dnstext: AnsiString): Boolean;
function DNSRRGetMX(const RR: TRRNameData; const pl: TPayLoadTCP;
  out MX: TDNSRR_MX): Boolean;
function DNSRRGetMX(const RR: TRRNameData; const pl: TPayLoad;
  out MX: TDNSRR_MX): Boolean;
function DNSRRGetPTR(const RR: TRRNameData; const pl: TPayLoadTCP;
  out ptr: TDNSDomainName): Boolean;
function DNSRRGetPTR(const RR: TRRNameData; const pl: TPayLoad;
  out ptr: TDNSDomainName): Boolean;
function DNSRRGetSRV(const RR: TRRNameData; const pl: TPayload;
  out srv: TDNSRR_SRV): Boolean;
function DNSRRGetSRV(const RR: TRRNameData; const pl: TPayloadTCP;
  out srv: TDNSRR_SRV): Boolean;


{$endif FPC_USE_LIBC}

Implementation

uses 
{$ifdef FPC_USE_LIBC}
   cNetDB,
{$endif FPC_USE_LIBC}
   BaseUnix,
   sysutils;

{$ifndef FPC_USE_LIBC}
type
  TTCPSocketResult = (srTimeout,srPartial,srSocketClose,srOK);

var
  DefaultDomainListArr : array of string;
  NDots: Integer;

   

{ ---------------------------------------------------------------------
    Some Parsing routines
  ---------------------------------------------------------------------}

Const 
  Whitespace = [' ',#9];

Function NextWord(Var Line : String) : String;

Var 
  I,J : Integer;

begin
  I:=1;
  While (I<=Length(Line)) and (Line[i] in Whitespace) do
    inc(I);
  J:=I;
  While (J<=Length(Line)) and Not (Line[J] in WhiteSpace) do
    inc(j);
  Result:=Copy(Line,I,J-I);  
  Delete(Line,1,J);  
end;
  
Function StripComment(var L : String) : Boolean;

Var
  i : Integer;

begin
  I:=Pos('#',L);
  If (I<>0) then
    L:=Copy(L,1,I-1)
  else
    begin
      I:=Pos(';',L);
      If (I<>0) then
        L:=Copy(L,1,I-1)
    end;
  Result:=Length(L)>0;
end;

Function MatchNameOrAlias(Const Entry,Name: String; Aliases : String) : Boolean;

Var
  P : Integer;
  A : String;

begin
  Result:=CompareText(Entry,Name)=0;
  If Not Result then
    While (Not Result) and (Length(Aliases)>0) do
      begin
      P:=Pos(',',Aliases);
      If (P=0) then
        P:=Length(Aliases)+1;
      A:=Copy(Aliases,1,P-1);
      Delete(Aliases,1,P);
      Result:=CompareText(A,Entry)=0;
      end;
end;

{ ---------------------------------------------------------------------
    hosts processing
  ---------------------------------------------------------------------}

Function GetAddr(Var L : String; Var Addr : THostAddr) : Boolean;

Var
  S : String;
//  i,p,a : Integer;
  
begin
  Result:=True;
  S:=NextWord(L);
  Addr:=StrToNetAddr(S);
//  Writeln(s,'->',Addr.s_bytes[1],'.',Addr.s_bytes[2],'.',Addr.s_bytes[3],'.',Addr.s_bytes[4]);
  Result:=Addr.s_bytes[1]<>0;
end;


Function FillHostEntry (Var Entry : THostEntry; L: String) : boolean;

Var
  H : String;

begin
  Result := False;
  Repeat
  H:=NextWord(L);
    If (H<>'') then begin
      if (Entry.Name='') then
        Entry.Name:=H
      else  
        begin
        If (Entry.Aliases<>'') then
          Entry.Aliases:=Entry.Aliases+',';
        Entry.Aliases:=Entry.Aliases+H;
        end;
      Result := True;
    end;
  until (H='');
end;

Function ProcessHosts(FileName : String) : PHostListEntry;

Var
  F : Text;
  L : String;
  A : THostAddr;
  T : PHostListEntry;
  B : Array of byte;
  FS : Int64;
  
begin
  Result:=Nil;
  Assign(F,FileName);
  {$push}{$I-}
  Reset(F);
  SetLength(B,65355);
  SetTextBuf(F,B[0],65355);
  {$pop};
  If (IOResult<>0) then
    Exit;
  Try  
    While Not EOF(F) do
      begin
      Readln(F,L);
      If StripComment(L) then
        begin
        If GetAddr(L,A) then
          begin
          T:=New(PHostListEntry);
          T^.Entry.Addr:=A;
          FillHostEntry(T^.Entry,L);
          T^.Next:=Result;
          Result:=T;
          end;
        end;
      end;
  Finally  
    Close(F);
  end;
end;

{ Internal lookup, used in GetHostByName and friends. }

Var
  HostsList : PHostListEntry = Nil;  
  HostsFileAge  : Longint;
//  HostsFileName : String;

Function FreeHostsList(var List : PHostListEntry) : Integer;

Var
  P : PHostListEntry;

begin
  Result:=0;
  While (List<>Nil) do
    begin
    Inc(Result);
    P:=List^.Next;
    Dispose(List);
    List:=P;
    end;
end;

Procedure HostsListToArray(var List : PHostListEntry; Var Hosts : THostEntryArray; FreeList : Boolean);

Var
  P : PHostListEntry;
  Len : Integer;

begin
  Len:=0;
  P:=List;
  While P<> Nil do
    begin
    Inc(Len);
    P:=P^.Next;
    end;
  SetLength(Hosts,Len);
  If (Len>0) then
    begin
    Len:=0;
    P:=List;
    While (P<>Nil) do
      begin
      Hosts[Len]:=P^.Entry;
      P:=P^.Next;
      Inc(Len);
      end;
    end;
  If FreeList then
    FreeHostsList(List);
end;

Procedure CheckHostsFile;

Var
  F : Integer;

begin
  If CheckHostsFileAge then
    begin
    F:=FileAge (EtcPath + SHostsFile);
    If HostsFileAge<F then
      begin
      // Rescan.
      FreeHostsList(HostsList);
      HostsList:=ProcessHosts (EtcPath + SHostsFile);
      HostsFileAge:=F;
      end;
    end;  
end;

Function FindHostEntryInHostsFile(N: String; Addr: THostAddr; Var H : THostEntry) : boolean;

Var
//  F : Text;
  HE : THostEntry;
  P : PHostListEntry;
  
begin
  Result:=False;
  CheckHostsFile;
  P:=HostsList;
  While (Not Result) and (P<>Nil) do
    begin
    HE:=P^.Entry;
    If (N<>'') then
      Result:=MatchNameOrAlias(N,HE.Name,HE.Aliases)
    else
      Result:=Cardinal(hosttonet(Addr))=Cardinal(HE.Addr);
    P:=P^.Next;  
    end; 
 If Result then
   begin
   H.Name:=HE.Name;
   H.Addr:=nettohost(HE.Addr);
   H.Aliases:=HE.Aliases;
   end;
end;

{ ---------------------------------------------------------------------
   Resolve.conf handling
  ---------------------------------------------------------------------}

{$ifdef android}

Function GetDNSServers: Integer;
var
  i: integer;
  s: string;
  H : THostAddr;
begin
  if SystemApiLevel >= 26 then
    begin
      // Since Android 8 the net.dnsX properties can't be read.
      // Use Google Public DNS servers
      Result:=2;
      SetLength(DNSServers, Result);
      DNSServers[0]:=StrToNetAddr('8.8.8.8');
      DNSServers[1]:=StrToNetAddr('8.8.4.4');
      exit;
    end;

  Result:=0;
  SetLength(DNSServers, 9);
  for i:=1 to 9 do
    begin
      s:=GetSystemProperty(PAnsiChar('net.dns' + IntToStr(i)));
      if s = '' then
        break;
      H:=StrToNetAddr(s);
      if H.s_bytes[1] <> 0 then
        begin
          DNSServers[Result]:=H;
          Inc(Result);
        end;
    end;
  SetLength(DNSServers, Result);
end;

var
  LastChangeProp: string;

Procedure CheckResolveFile;
var
  n, v: string;
begin
  if not CheckResolveFileAge then
    exit;

  if (Length(DNSServers) = 0) and (SystemApiLevel >= 26) then
    begin
      GetDNSServers;
      exit;
    end;

  n:=GetSystemProperty('net.change');
  if n <> '' then
    v:=GetSystemProperty(PAnsiChar(n))
  else
    v:='';
  n:=n + '=' + v;
  if LastChangeProp = n then
    exit;
  LastChangeProp:=n;
  GetDNSServers;
end;

{$else}

Var
  ResolveFileAge  : Longint;
  ResolveFileName : String;
  
Function GetDNSServers(Fn : String) : Integer;

Var
  R : Text;
  L : String;
//  I : Integer;
  H : THostAddr;
  E : THostEntry;
  
  Function CheckDirective(Dir : String) : Boolean;
  
  Var
    P : Integer;
  
  begin
    P:=Pos(Dir,L);
    Result:=(P<>0);
    If Result then
      begin
      Delete(L,1,P+Length(Dir));
      L:=Trim(L);
      end;
  end;
   
begin
  Result:=0;
  ResolveFileName:=Fn;
  ResolveFileAge:=FileAge(FN);
  DefaultDomainListArr:=[];
  NDots:=1;
  {$push}{$i-}
  Assign(R,FN);
  Reset(R);
  {$pop}
  If (IOResult<>0) then 
    exit;
  Try  
    While not EOF(R) do
      begin
      Readln(R,L);
      if StripComment(L) then
        If CheckDirective('nameserver') then
          begin
          H:=HostToNet(StrToHostAddr(L));
          If (H.s_bytes[1]<>0) then
            begin
            setlength(DNSServers,Result+1);
            DNSServers[Result]:=H;
            Inc(Result);
            end
          else if FindHostEntryInHostsFile(L,H,E) then
            begin
            setlength(DNSServers,Result+1);
            DNSServers[Result]:=E.Addr;
            Inc(Result);
            end;
          end
        else if CheckDirective('domain') then
          DefaultDomainList:=L
        else if CheckDirective('search') then
          DefaultDomainList:=L
        else if CheckDirective('options') then
          DNSOptions:=L;
      end;
  Finally
    Close(R);
  end;
  L := GetEnvironmentVariable('LOCALDOMAIN');
  if L <> '' then
    DefaultDomainList := L;
end;

Procedure CheckResolveFile;

Var
  F : Integer;
  N : String;

begin
  If CheckResolveFileAge then
    begin
    N:=ResolveFileName;
    if (N='') then
      N:=EtcPath + SResolveFile;
    F:=FileAge(N);
    If ResolveFileAge<F then
      GetDnsServers(N);
    end;  
end;

{$endif android}

{ ---------------------------------------------------------------------
    Payload handling functions.
  ---------------------------------------------------------------------}
  

Procedure DumpPayLoad(Q : TQueryData; L : Integer);

Var 
  i : Integer;

begin
  Writeln('Payload : ',l);
  For I:=0 to L-1 do
    Write(Q.Payload[i],' ');
  Writeln;  
end;
  
Function BuildPayLoad(Var Q : TQueryData; Name : String; RR : Word; QClass : Word) : Integer;

Var
  P : PByte;
  l,S : Integer;
  
begin
  Result:=-1;
  If (Length(Name) = 0) or (length(Name)>506) then
    Exit;

  Result:=0;
  P:=@Q.Payload[0];
  Repeat
    L:=Pos('.',Name);
    If (L=0) then
      S:=Length(Name)
    else
      S:=L-1;
    // empty label is invalid, unless it's a dot at the end.
    if (S = 0) then
    begin
      if (Length(Name) > 0) then
      begin
        Result := -1;
        exit;
      end
      else
        break; // empty label at end, break out for final 0 length byte.
    end;
    P[Result]:=S;
    Move(Name[1],P[Result+1],S);
    Inc(Result,S+1);
    If (L>0) then
      Delete(Name,1,L);
  Until (L=0);
  P[Result]:=0;
  rr := htons(rr);
  Move(rr,P[Result+1],2);
  Inc(Result,3);
  QClass := htons(QClass);
  Move(qclass,P[Result],2);
  Inc(Result,2);
end;

{Construct a TCP query payload from the given name, rr and qclass. The
 principal difference between the TCP and UDP payloads is the two-octet
 length field in the TCP payload. The UDP payload has no length field.

 See RFC-1035, section 4.2.2.

 Returns the length of the constructed payload, which doesn't include
 the header or the length field.}
function BuildPayLoadTCP(var Q: TQueryDataLength; Name: String; RR: Word;
  QClass: Word): Integer;
var
  l: Word;
begin
  l := BuildPayLoad(Q.hpl, Name, RR, QClass);
  Q.length := htons(l + SizeOf(Q.hpl.h));
  Result := l;
end;

Function NextRR(Const PayLoad : TPayLoad;Var Start : LongInt; AnsLen : LongInt; Var RR : TRRData) : Boolean;

Var
  I : Integer;
  HaveName : Boolean;
  PA : PRRData;
  
begin
  Result:=False;
  I:=Start;
  // Skip labels and pointers. At least 1 label or pointer is present.
  Repeat
    HaveName:=True;
    If (Payload[i]>63) then // Pointer, skip
      Inc(I,2)
    else If Payload[i]=0 then // Null termination of label, skip.
      Inc(i)
    else  
      begin
      Inc(I,Payload[i]+1); // Label, continue scan.
      HaveName:=False;
      end;
  Until HaveName or (I>(AnsLen-SizeOf(TRRData)));
  Result:=(I<=(AnsLen-SizeOf(TRRData)));
  // Check RR record.
  PA:=PRRData(@Payload[i]);
  RR:=PA^;
  Start:=I+SizeOf(TRRData);
end;


Function BuildName (Const PayLoad : TPayLoad; Start,len : Integer) : String;

Const
  FIREDNS_POINTER_VALUE = $C000;
  
Var
  I,O : Integer;
  P : Word;
  
begin
  SetLength(Result,512);
  I:=Start;
  O:=1;
  // Copy labels and pointers. At least 1 label or pointer is present.
  Repeat
    If (Payload[i]>63) then // Pointer, move.
      begin
      Move(Payload[i],P,2);
      I:=ntohs(p)-FIREDNS_POINTER_VALUE-12;
      end
    else if Payload[i]<>0 then // Label, copy
      begin
      If O<>1 then
        begin
        Result[O]:='.';
        Inc(O);
        end;
      P:=Payload[i];  
      Move(Payload[i+1],Result[o],P);
      Inc(I,P+1);
      Inc(O,P);
      end;
   Until (Payload[I]=0);
   setlength(result,o-1);
end;


{ ---------------------------------------------------------------------
    QueryData handling functions
  ---------------------------------------------------------------------}

function CheckAnswer(const Qry: TDNSHeader; var Ans: TDNSHeader): Boolean;
begin
  Result:=False;
  With Ans do
    begin
    // Check ID.
    If (ID[1]<>QRY.ID[1]) or (ID[0]<>Qry.ID[0]) then
      exit;  
    // Flags ?
    If (Flags1 and QF_QR)=0 then
      exit;
    if (Flags1 and QF_OPCODE)<>0 then 
      exit;
    if (Flags2 and QF_RCODE)<>0 then
      exit;  
    // Number of answers ?  
    AnCount := htons(Ancount);
    If Ancount<1 then
      Exit;
    Result:=True;
    end;
end;

{
 Check that Atype is valid. These are the DNSQRY_? params we support. See the
 definitions at the top of this unit for the names.
 Deliberately excluding axfr (252), mailb (253), maila (254), and * (255).
}
function IsValidAtype(atype: Word): Boolean;
begin
  Result := False;
  case atype of
    1 .. 16, 28, 33: Result := True;
  end;
end;

function IsTruncated(R: TDNSHeader): Boolean;
begin
  Result := ((R.flags1 and QF_TC) > 0);
end;

function GetRcode(R: TDNSHeader): TDNSRcode;
var
  rcode_n: Byte;
begin
  rcode_n := (R.flags2 and QF_RCODE);
  case rcode_n of
    0: Result := rcNoError;
    1: Result := rcFormatError;
    2: Result := rcServFail;
    3: Result := rcNXDomain;
    4: Result := rcNotImpl;
    5: Result := rcRefused;
    6 .. 15: Result := rcReserved;
  else
    Result := rcInvalid;
  end;
end;

function GetFixlenStr(pl: TPayLoad; startidx: Cardinal; len: Byte; out
  res: ShortString): Byte;
begin
  Result := 0;
  res := '';
  if (startidx + len) > Length(pl) then exit;
  SetLength(res, len);
  Move(pl[startidx], res[1], len);
  Result := len;
end;

function GetFixlenStr(pl: TPayLoadTCP; startidx: Cardinal; len: Byte;
  out res: ShortString): Byte;
begin
  Result := 0;
  res := '';
  if (startidx + len) > Length(pl) then exit;
  SetLength(res, len);
  Move(pl[startidx], res[1], len);
  Result := len;
end;

function NextNameRR(const pl: TPayLoadTCP; start: Word; out RRName: TRRNameData
  ): Boolean;
var
  I : Integer;
  PA : PRRData;

begin
  Result:=False;
  I:=Start;
  if (Length(pl) - I) < (SizeOf(TRRData)+2) then exit;
  RRName.RRName := stringfromlabel(pl, I);
  if (Length(pl) - I) < (SizeOf(TRRData)) then exit;

  PA:=PRRData(@pl[I]);
  RRName.RRMeta := PA^;
  RRName.RRMeta.AClass := NToHs(RRName.RRMeta.AClass);
  RRName.RRMeta.Atype := NToHs(RRName.RRMeta.Atype);
  if not IsValidAtype(RRName.RRMeta.Atype) then
    exit;
  RRName.RRMeta.RDLength := NToHs(RRName.RRMeta.RDLength);
  RRName.RRMeta.TTL := NToHl(RRName.RRMeta.TTL);
  RRName.RDataSt := I+SizeOf(TRRData);
  // verify that start + rdlength is within the buffer boundary.
  if RRName.RDataSt + RRName.RRMeta.RDLength > Length(pl) then exit;
  Result := True;
end;

function NextNameRR(const pl: TPayLoad; start: Word; out RRName: TRRNameData
  ): Boolean;
var
  I : Integer;
  PA : PRRData;

begin
  Result:=False;
  I:=Start;
  if (Length(pl) - I) < (SizeOf(TRRData)+2) then exit;
  RRName.RRName := stringfromlabel(pl, I);
  if (Length(pl) - I) < (SizeOf(TRRData)) then exit;

  PA:=PRRData(@pl[I]);
  RRName.RRMeta := PA^;
  RRName.RRMeta.AClass := NToHs(RRName.RRMeta.AClass);
  RRName.RRMeta.Atype := NToHs(RRName.RRMeta.Atype);
  if not IsValidAtype(RRName.RRMeta.Atype) then
    exit;

  RRName.RRMeta.RDLength := NToHs(RRName.RRMeta.RDLength);
  RRName.RRMeta.TTL := NToHl(RRName.RRMeta.TTL);
  RRName.RDataSt := I+SizeOf(TRRData);
  // verify that start + rdlength is within the buffer boundary.
  if RRName.RDataSt + RRName.RRMeta.RDLength > Length(pl) then exit;
  Result := True;
end;

function GetRRrecords(const pl: TPayloadTCP; var Start: Word; Count: Word
  ): TRRNameDataArray;
var
  I, Total: Word;
  B: Boolean;
  RRN: TRRNameData;

begin
  I:=0;
  Total := 0;
  SetLength(Result,Count);
  while (I < Count) do
  begin
    B := NextNameRR(pl, Start, RRN);
    if not B then break;
    Inc(Total);
    Result[I] := RRN;
    Inc(I);
    Start := RRN.RDataSt+RRN.RRMeta.RDLength;
  end;
  if Total < Count then SetLength(Result,Total);
end;

function GetRRrecords(const pl: TPayload; var Start: Word; Count: Word
  ): TRRNameDataArray;
var
  I, Total: Word;
  B: Boolean;
  RRN: TRRNameData;

begin
  I:=0;
  Total := 0;
  SetLength(Result,Count);
  while (I < Count) do
  begin
    B := NextNameRR(pl, Start, RRN);
    if not B then break;
    Inc(Total);
    Result[I] := RRN;
    Inc(I);
    Start := RRN.RDataSt+RRN.RRMeta.RDLength;
  end;
  if Total < Count then SetLength(Result,Total);
end;

function DnsLookup(dn: String; qtype: Word; out Ans: TQueryData; out
  AnsLen: Longint): Boolean;
var
  Qry: TQueryData;
  QryLen: Longint;
  idx: Word;
begin
  Result := False;
  AnsLen := -2;

  CheckResolveFile;
  if Length(DNSServers) = 0 then
    exit;

  QryLen := BuildPayLoad(Qry, dn, qtype, 1);
  if QryLen <= 0 then exit;

  AnsLen := -1;
  { Try the query at each configured resolver in turn, until one of them
   returns an answer. We check for AnsLen > -1 because we need to distinguish
   between failure to connect and the server saying it doesn't know or can't
   answer. If AnsLen = -1 then we failed to connect. If AnsLen >= 0 but qr
   = False, then we connected but the server returned an error code.}
  idx := 0;
  repeat
    Result := Query(idx,Qry,Ans,QryLen,AnsLen);
    Inc(idx);
  until (idx > High(DNSServers)) or (Result = True) or (AnsLen >= 0);
end;

function DnsLookup(dn: String; qtype: Word; out Ans: TQueryDataLengthTCP; out
  AnsLen: Longint): Boolean;
var
  Qry: TQueryDataLength;
  QryLen: Longint;
  idx: Word;

begin
  Result := False;
  AnsLen := -2;

  CheckResolveFile;
  if Length(DNSServers) = 0 then
    exit;

  QryLen:=BuildPayLoadTCP(Qry, dn, qtype, 1);
  if QryLen <= 0 then exit;
  AnsLen := -1;

  { Try the query at each configured resolver in turn, until one of them
   returns an answer. We check for AnsLen > -1 because we need to distinguish
   between failure to connect and the server saying it doesn't know or can't
   answer. If AnsLen = -1 then we failed to connect. If AnsLen >= 0 but qr
   = False, then we connected but the server returned an error code.}
  idx := 0;
  repeat
    Result := QueryTCP(idx,Qry,Ans,QryLen,AnsLen);
    Inc(idx);
  until (idx > High(DNSServers)) or (Result = True) or (AnsLen >= 0);
end;

function DNSRRGetA(const RR: TRRNameData; const pl: TPayLoadTCP; out
  IP: THostAddr): Boolean;
begin
  IP.s_addr := 0;
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_A then exit;
  if (Length(pl) - RR.RDataSt) < 4 then exit;
  Move(pl[RR.RDataSt], IP, SizeOf(THostAddr));
  IP.s_addr := NToHl(IP.s_addr);
  Result := True;
end;

function DNSRRGetA(const RR: TRRNameData; const pl: TPayLoad; out IP: THostAddr
  ): Boolean;
begin
  IP.s_addr := 0;
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_A then exit;
  if (Length(pl) - RR.RDataSt) < 4 then exit;
  Move(pl[RR.RDataSt], IP, SizeOf(THostAddr));
  IP.s_addr := NToHl(IP.s_addr);
  Result := True;
end;

function DNSRRGetCNAME(const RR: TRRNameData; const pl: TPayLoad; out
  cn: TDNSDomainName): Boolean;
var
  n: Integer;
begin
  Result := False;
  cn := '';
  if RR.RRMeta.Atype <> DNSQRY_CNAME then exit;
  n := RR.RDataSt;
  if (RR.RDataSt + RR.RRMeta.RDLength) > Length(pl) then exit;
  cn := stringfromlabel(pl, n);
  Result := True;
end;

function DNSRRGetCNAME(const RR: TRRNameData; const pl: TPayLoadTCP; out
  cn: TDNSDomainName): Boolean;
var
  n: Integer;
begin
  Result := False;
  cn := '';
  if RR.RRMeta.Atype <> DNSQRY_CNAME then exit;
  n := RR.RDataSt;
  if (n + RR.RRMeta.rdlength) > Length(pl) then exit;
  cn := stringfromlabel(pl, n);
  Result := True;
end;

function DNSRRGetAAAA(const RR: TRRNameData; const pl: TPayLoadTCP; out
  IP: THostAddr6): Boolean;
begin
  IP.s6_addr32[0] := 0;
  IP.s6_addr32[1] := 0;
  IP.s6_addr32[2] := 0;
  IP.s6_addr32[3] := 0;
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_AAAA then exit;
  if (RR.RDataSt + SizeOf(THostAddr6)) > Length(pl) then exit;
  Move(pl[RR.RDataSt],IP,SizeOf(THostAddr6));
  Result := True;
end;

function DNSRRGetAAAA(const RR: TRRNameData; const pl: TPayLoad; out
  IP: THostAddr6): Boolean;
begin
  IP.s6_addr32[0] := 0;
  IP.s6_addr32[1] := 0;
  IP.s6_addr32[2] := 0;
  IP.s6_addr32[3] := 0;
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_AAAA then exit;
  if (RR.RDataSt + SizeOf(THostAddr6)) > Length(pl) then exit;
  Move(pl[RR.RDataSt],IP,SizeOf(THostAddr6));
  Result := True;
end;

function DNSRRGetNS(const RR: TRRNameData; const pl: TPayLoadTCP; out
  NSName: TDNSDomainName): Boolean;
var
  n: LongInt;
begin
  NSName := '';
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_NS then exit;
  if (RR.RDataSt + RR.RRMeta.RDLength) > Length(pl) then exit;
  n := RR.RDataSt;
  NSName := stringfromlabel(pl, n);
  Result := True;
end;

function DNSRRGetNS(const RR: TRRNameData; const pl: TPayLoad; out
  NSName: TDNSDomainName): Boolean;
var
  n: LongInt;
begin
  NSName := '';
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_NS then exit;
  if (RR.RDataSt + RR.RRMeta.RDLength) > Length(pl) then exit;
  n := RR.RDataSt;
  NSName := stringfromlabel(pl, n);
  Result := True;
end;

function DNSRRGetSOA(const RR: TRRNameData; const pl: TPayLoadTCP; out
  dnssoa: TDNSRR_SOA): Boolean;
var
  idx: Integer;
begin
  // can't trust the counts we've been given, so check that we never
  // exceed the end of the payload buffer.
  idx := RR.RDataSt;
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_SOA then exit;
  dnssoa.mname := stringfromlabel(pl, idx);
  if idx >= Length(pl) then exit;

  dnssoa.rname := stringfromlabel(pl, idx);

  if (idx + (SizeOf(Cardinal) * 5)) > Length(pl) then exit;
  Move(pl[idx],dnssoa.serial,SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.refresh, SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.retry, SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.expire, SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.min, SizeOf(Cardinal));
  Result := True;
  dnssoa.serial := NToHl(dnssoa.serial);
  dnssoa.min := NToHl(dnssoa.min);
  dnssoa.expire := NToHl(dnssoa.expire);
  dnssoa.refresh := NToHl(dnssoa.refresh);
  dnssoa.retry := NToHl(dnssoa.retry);
end;

function DNSRRGetSOA(const RR: TRRNameData; const pl: TPayLoad; out
  dnssoa: TDNSRR_SOA): Boolean;
var
  idx: Integer;
begin
  // can't trust the counts we've been given, so check that we never
  // exceed the end of the payload buffer.
  idx := RR.RDataSt;
  Result := False;
  if RR.RRMeta.Atype <> DNSQRY_SOA then exit;
  dnssoa.mname := stringfromlabel(pl, idx);
  if idx >= Length(pl) then exit;

  dnssoa.rname := stringfromlabel(pl, idx);

  if (idx + (SizeOf(Cardinal) * 5)) > Length(pl) then exit;
  Move(pl[idx],dnssoa.serial,SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.refresh, SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.retry, SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.expire, SizeOf(Cardinal));
  Inc(idx, SizeOf(Cardinal));
  Move(pl[idx], dnssoa.min, SizeOf(Cardinal));
  Result := True;
  dnssoa.serial := NToHl(dnssoa.serial);
  dnssoa.min := NToHl(dnssoa.min);
  dnssoa.expire := NToHl(dnssoa.expire);
  dnssoa.refresh := NToHl(dnssoa.refresh);
  dnssoa.retry := NToHl(dnssoa.retry);
end;

function DNSRRGetText(const RR: TRRNameData; const pl: TPayLoad; out
  dnstext: AnsiString): Boolean;
var
  wrk: ShortString;
  idx: LongInt;
  l: Byte;
begin
  Result := False;
  dnstext := '';
  if RR.RRMeta.Atype <> DNSQRY_TXT then exit;
  wrk := '';

  idx := RR.RDataSt;
  if (Length(pl) - idx)  < 2 then exit;

  repeat
    l := GetFixlenStr(pl, idx+1, pl[idx], wrk);
    if l = 0 then exit; // count would send us past end of buffer
    dnstext := dnstext + wrk;
    Inc(idx, l+1);
  until (idx >= (RR.RDataSt + RR.RRMeta.RDLength)) or ((Length(pl) - idx) < 2);
  Result := True;
end;

function DNSRRGetText(const RR: TRRNameData; const pl: TPayLoadTCP; out
  dnstext: AnsiString): Boolean;
var
  wrk: ShortString;
  idx: LongInt;
  l: Byte;
begin
  Result := False;
  dnstext := '';
  if RR.RRMeta.Atype <> DNSQRY_TXT then exit;
  wrk := '';

  idx := RR.RDataSt;
  if (Length(pl) - idx)  < 2 then exit;

  repeat
    l := GetFixlenStr(pl, idx+1, pl[idx], wrk);
    if l = 0 then exit; // count would send us past end of buffer
    dnstext := dnstext + wrk;
    Inc(idx, l+1);
  until (idx >= (RR.RDataSt + RR.RRMeta.RDLength)) or ((Length(pl) - idx) < 2);
  Result := True;
end;

function DNSRRGetMX(const RR: TRRNameData; const pl: TPayLoadTCP; out
  MX: TDNSRR_MX): Boolean;
var
  idx: Integer;
begin
  Result := False;
  MX.preference := 0;
  MX.exchange := '';
  if RR.RRMeta.Atype <> DNSQRY_MX then exit;
  idx := RR.RDataSt;
  if idx + SizeOf(Word) >= Length(pl) then exit;
  Move(pl[idx],MX.preference, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;
  MX.exchange := stringfromlabel(pl, idx);
  MX.preference := NToHs(MX.preference);
  Result := True;
end;

function DNSRRGetMX(const RR: TRRNameData; const pl: TPayLoad; out MX: TDNSRR_MX
  ): Boolean;
var
  idx: Integer;
begin
  Result := False;
  MX.preference := 0;
  MX.exchange := '';
  if RR.RRMeta.Atype <> DNSQRY_MX then exit;
  idx := RR.RDataSt;
  if idx + SizeOf(Word) >= Length(pl) then exit;
  Move(pl[idx],MX.preference, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;
  MX.exchange := stringfromlabel(pl, idx);
  MX.preference := NToHs(MX.preference);
  Result := True;
end;

function DNSRRGetPTR(const RR: TRRNameData; const pl: TPayLoadTCP; out
  ptr: TDNSDomainName): Boolean;
var
  n: Integer;
begin
  Result := False;
  ptr := '';
  if RR.RRMeta.Atype <> DNSQRY_PTR then exit;
  n := RR.RDataSt;
  if (n + RR.RRMeta.RDLength) > Length(pl) then exit;
  ptr := stringfromlabel(pl, n);
  Result := True;
end;

function DNSRRGetPTR(const RR: TRRNameData; const pl: TPayLoad; out
  ptr: TDNSDomainName): Boolean;
var
  n: Integer;
begin
  Result := False;
  ptr := '';
  if RR.RRMeta.Atype <> DNSQRY_PTR then exit;
  n := RR.RDataSt;
  if (n + RR.RRMeta.RDLength) > Length(pl) then exit;
  ptr := stringfromlabel(pl, n);
  Result := True;
end;

function DNSRRGetSRV(const RR: TRRNameData; const pl: TPayload; out
  srv: TDNSRR_SRV): Boolean;
var
  idx: Integer;
begin
  Result := False;
  srv.priority := 0;
  srv.weight := 0;
  srv.port := 0;
  srv.target := '';
  if RR.RRMeta.Atype <> DNSQRY_SRV then exit;

  idx := RR.RDataSt;
  if idx +  RR.RRMeta.RDLength > Length(pl) then exit;

  Move(pl[idx], srv.priority, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;

  Move(pl[idx], srv.weight, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;

  Move(pl[idx], srv.port, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;

  srv.target := stringfromlabel(pl, idx);

  srv.priority := NToHs(srv.priority);
  srv.weight := NToHs(srv.weight);
  srv.port := NToHs(srv.port);

  Result := True;
end;

function DNSRRGetSRV(const RR: TRRNameData; const pl: TPayloadTCP; out
  srv: TDNSRR_SRV): Boolean;
var
  idx: Integer;
begin
  Result := False;
  srv.priority := 0;
  srv.weight := 0;
  srv.port := 0;
  srv.target := '';
  if RR.RRMeta.Atype <> DNSQRY_SRV then exit;

  idx := RR.RDataSt;
  if idx +  RR.RRMeta.RDLength > Length(pl) then exit;

  Move(pl[idx], srv.priority, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;

  Move(pl[idx], srv.weight, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;

  Move(pl[idx], srv.port, SizeOf(Word));
  Inc(idx, SizeOf(Word));
  if (Length(pl) - idx) < 2 then exit;

  srv.target := stringfromlabel(pl, idx);

  srv.priority := NToHs(srv.priority);
  srv.weight := NToHs(srv.weight);
  srv.port := NToHs(srv.port);

  Result := True;
end;

Function SkipAnsQueries(Var Ans : TQueryData; L : Integer) : integer;

Var
  Q,I : Integer;

begin
  Result:=0;
  With Ans do
    begin
    h.qdcount := htons(h.qdcount);
    i:=0;
    q:=0;
    While (Q<h.qdcount) and (i<l) do  
      begin
      If Payload[i]>63 then
        begin
        Inc(I,6);
        Inc(Q);
        end
      else
        begin
        If Payload[i]=0 then
          begin
          inc(q);
          Inc(I,5);
          end
        else
          Inc(I,Payload[i]+1);  
        end;  
      end;
    Result:=I;  
    end;  
end;

function SkipAnsQueries(var Ans: TQueryDataLengthTCP; L: Integer): integer;
var
  Q,I : Integer;

begin
  Result:=0;
  With Ans do
  begin
    h.qdcount := htons(h.qdcount);
    i:=0;
    q:=0;
    While (Q<h.qdcount) and (i<l) do
    begin
      If Payload[i]>63 then
      begin
        Inc(I,6);
        Inc(Q);
      end
      else
      begin
        If Payload[i]=0 then
        begin
          inc(q);
          Inc(I,5);
        end
        else
          Inc(I,Payload[i]+1);
      end;
    end;
    Result:=I;
  end;
end;

{ ---------------------------------------------------------------------
    DNS Query functions.
  ---------------------------------------------------------------------}
  

Function Query(Resolver : Integer; Var Qry,Ans : TQueryData; QryLen : Integer; Var AnsLen : Integer) : Boolean;

Var
  SA : TInetSockAddr;
  Sock,L : Longint;
  Al,RTO : Longint;
  ReadFDS : TFDSet;
  
begin
  Result:=False;
  With Qry.h do
    begin
    ID[0]:=Random(256);
    ID[1]:=Random(256);
    Flags1:=QF_RD;
    Flags2:=0;
    qdcount:=htons(1); // was 1 shl 8;
    ancount:=0;
    nscount:=0;
    arcount:=0;
    end;
  Sock:=FpSocket(PF_INET,SOCK_DGRAM,0);
  If Sock=-1 then 
    exit;
  With SA do
    begin
    sin_family:=AF_INET;
    sin_port:=htons(DNSport);
    sin_addr.s_addr:=cardinal(DNSServers[Resolver]); // dnsservers already in net order
    end;
  fpsendto(sock,@qry,qrylen+12,0,@SA,SizeOf(SA));
  // Wait for answer.
  RTO:=TimeOutS*1000+TimeOutMS;
  fpFD_ZERO(ReadFDS);
  fpFD_Set(sock,readfds);
  if fpSelect(Sock+1,@readfds,Nil,Nil,RTO)<=0 then
    begin
    fpclose(Sock);
    exit;
    end;
  AL:=SizeOf(SA);
  L:=fprecvfrom(Sock,@ans,SizeOf(Ans),0,@SA,@AL);
  fpclose(Sock);

  if L < 12 then exit;
  // Return Payload length.
  Anslen:=L-12;
  // even though we may still return false to indicate an error, if AnsLen
  // is >= 0 then the caller knows the dns server responded.
  If not CheckAnswer(Qry.h,Ans.h) Then
    exit;
  Result:=True;
  //end;
end;

function FetchDNSResponse(sock: Cint; out len: ssize_t;
  out Ans: TQueryDataLengthTCP): TTCPSocketResult;
var
  respsize: Word;
  L: ssize_t;

begin
  Result := srOK;
  len := 0;

  // peek into the socket buffer and see if a full message is waiting.
  L := fprecv(sock, @Ans, SizeOf(Ans), MSG_PEEK);
  if L = 0 then
  begin
    Result := srSocketClose;
    exit;
  end;
  // The first two bytes of a DNS TCP payload is the number of octets in the
  // response, excluding the two bytes of length. This lets us see if we've
  // received the full response.
  respsize := NToHs(Ans.length);
  if (L < 2) or (L < (respsize + SizeOf(Ans.length))) then
  begin
    Result := srPartial;
    exit;
  end;

  // The full DNS response is waiting in the buffer. Get it now.
  len := fprecv(sock, @Ans, SizeOf(Ans), 0);
end;

function QueryTCP(Resolver: Integer; var Qry: TQueryDataLength;
  var Ans: TQueryDataLengthTCP; QryLen: Integer; var AnsLen: Integer): Boolean;
Var
  SA : TInetSockAddr;
  Sock : cint;
  L: ssize_t;
  RTO : Longint;
  ReadFDS : TFDSet;
  count: Integer;
  sendsize: ssize_t;
  respsize: Word;
  resp: TTCPSocketResult;
  tstart: QWord;

begin
  tstart := GetTickCount64;
  Result:=False;
  With Qry.hpl.h do
  begin
    ID[0]:=Random(256);
    ID[1]:=Random(256);
    Flags1:=QF_RD;
    Flags2:=0;
    qdcount:=htons(1); // was 1 shl 8;
    ancount:=0;
    nscount:=0;
    arcount:=0;
  end;
  Sock:=FpSocket(AF_INET,SOCK_STREAM,0);
  If Sock=-1 then
    exit;
  With SA do
  begin
    sin_family:=AF_INET;
    sin_port:=htons(DNSport);
    sin_addr.s_addr:=cardinal(DNSServers[Resolver]); // octets already in net order
  end;

  // connect to the resolver
  if (fpconnect(Sock, @SA, SizeOf(SA)) <> 0) then
    exit;

  // send the query to the resolver
  sendsize := QryLen + SizeOf(Qry.hpl.h) + SizeOf(Qry.length);
  count := fpsend(Sock,@Qry,sendsize,0);
  if count < sendsize then
  begin
    fpclose(Sock);
    exit;
  end;

  // tell other side we're done writing.
  fpshutdown(Sock, SHUT_WR);

  RTO := 5000;
  fpFD_ZERO(ReadFDS);
  fpFD_Set(sock,ReadFDS);

  // select to wait for data
  if fpSelect(sock+1, @ReadFDS, Nil, Nil,  RTO)<=0 then
  begin
    // timed out, nothing received.
    fpclose(sock);
    exit;
  end;

  // for partial responses, keep trying until all data received or the
  // timeout period has elapsed. the timeout period includes the time
  // spent waiting on select.
  resp := FetchDNSResponse(Sock, L, Ans);
  while (resp = srPartial) and ((GetTickCount64 - tstart) < RTO) do
  begin
    // need to sleep to avoid high cpu. 50ms means a 5 second timeout will
    // make up to 100 calls to FetchDNSResponse.
    Sleep(50);
    resp := FetchDNSResponse(Sock, L, Ans);
  end;

  fpclose(sock);
  if resp <> srOK then exit;

  // Set AnsLen to be the size of the payload minus the header.
  Anslen := L-SizeOf(Qry.hpl.h);
  // if the final check finds problems with the answer, we'll return false
  // but AnsLen being >=0 will let the caller know that the server did
  // respond, but either declined to answer or couldn't.
  If not CheckAnswer(Qry.hpl.h,Ans.h) then
    exit;
  Result:=True;
end;

{
Read a string from the payload buffer. Handles compressed as well as
regular labels. On termination start points to the character after the
end of the str.
}

function stringfromlabel(pl: TPayLoad; var start: Integer): string;
var
  l,i,n,lc: integer;
  ptr: Word;
  ptrseen: Boolean = False;
begin
  result := '';
  l := 0;
  i := 0;
  n := start;
  // Label counter. Per rfc1035, s. 3.1, each label is at least 2 bytes and the
  // max length for a domain is 255, so there can't be more than 127 labels.
  // This helps to short-circuit loops in label pointers.
  lc := 0;
  repeat
    // each iteration of this loop is for one label. whether a pointer or a
    // regular label, we need 2 bytes headroom minimum.
    if n > (Length(pl) - 2) then break;
    l := ord(pl[n]);
    { compressed reply }
    while (l >= 192) do
      begin
        if not ptrseen then start := n + 2;
        ptrseen := True;
        ptr := (l and not(192)) shl 8 + ord(pl[n+1]);
        {ptr must point backward and be >= 12 (for the dns header.}
        if (ptr >= (n+12)) or (ptr < 12) then l := 0 // l=0 causes loop to exit
        else
        begin
          { the -12 is because of the reply header length. we do the decrement
          here to avoid overflowing if ptr < 12.}
          n := ptr - 12;
          l := ord(pl[n]);
        end;
      end;
    // check we point inside the buffer
    if (n+l+1) > Length(pl) then l := 0;
    if l <> 0 then begin
      setlength(result,length(result)+l);
      move(pl[n+1],result[i+1],l);
      result := result + '.';
      inc(n,l); inc(n);
      inc(i,l); inc(i);
      if n > start then start := n;
    end;
    Inc(lc); // label count
  until (l = 0) or (lc > 127);
  // per rfc1035, section 4.1.4, a domain name may be represented by
  // either a sequence of labels followed by 0, or a pointer, or a series
  // of labels followed by a pointer. If there's a pointer there's no 0 to
  // skip over when calculating the final index.
  if not ptrseen then Inc(start); // jump past the 0.
  if (Length(result) > 0) and (result[length(result)] = '.') then
    setlength(result,length(result)-1);
end;

function stringfromlabel(pl: TPayLoadTCP; var start: Integer): string;
var
  l,i,n,lc: integer;
  ptr: Word;
  ptrseen: Boolean = False;
begin
  result := '';
  l := 0;
  i := 0;
  n := start;
  // Label counter. Per rfc1035, s. 3.1, each label is at least 2 bytes and the
  // max length for a domain is 255, so there can't be more than 127 labels.
  // This helps to short-circuit loops in label pointers.
  lc := 0;
  repeat
    // each iteration of this loop is for one label. whether a pointer or a
    // regular label, we need 2 bytes headroom minimum.
    if n > (Length(pl) - 2) then break;
    l := ord(pl[n]);
    { compressed reply }
    while (l >= 192) do
      begin
        if not ptrseen then start := n + 2;
        ptrseen := True;
        ptr := (l and not(192)) shl 8 + ord(pl[n+1]);
        {ptr must point backward and be >= 12 (for the dns header.}
        if (ptr >= (n+12)) or (ptr < 12) then l := 0 // l=0 causes loop to exit
        else
        begin
          { the -12 is because of the reply header length. we do the decrement
          here to avoid overflowing if ptr < 12.}
          n := ptr - 12;
          l := ord(pl[n]);
        end;
      end;
    // check we point inside the buffer
    if (n+l+1) > Length(pl) then l := 0;
    if l <> 0 then begin
      setlength(result,length(result)+l);
      move(pl[n+1],result[i+1],l);
      result := result + '.';
      inc(n,l); inc(n);
      inc(i,l); inc(i);
      if n > start then start := n;
    end;
    Inc(lc); // label count
  until (l = 0) or (lc > 127);
  // per rfc1035, section 4.1.4, a domain name may be represented by
  // either a sequence of labels followed by 0, or a pointer, or a series
  // of labels followed by a pointer. If there's a pointer there's no 0 to
  // skip over when calculating the final index.
  if not ptrseen then Inc(start); // jump past the 0.
  if (Length(result) > 0) and (result[length(result)] = '.') then
    setlength(result,length(result)-1);
end;

Function ResolveNameAt(Resolver : Integer; HostName : String; Var Addresses : Array of THostAddr; Recurse: Integer) : Integer;

Var
  Qry, Ans            : TQueryData;
  MaxAnswer,I,QryLen,
  AnsLen,AnsStart     : Longint;
  RR                  : TRRData;
  cname               : string;
begin
  Result:=0;
  QryLen:=BuildPayLoad(Qry,HostName,DNSQRY_A,1);
  If Not Query(Resolver,Qry,Ans,QryLen,AnsLen) then
    Result:=-1
  else  
    begin
    AnsStart:=SkipAnsQueries(Ans,AnsLen);
    MaxAnswer:=Ans.h.AnCount-1;
    If MaxAnswer>High(Addresses) then
      MaxAnswer:=High(Addresses);
    I:=0;
    While (I<=MaxAnswer) and NextRR(Ans.Payload,AnsStart,AnsLen,RR) do
      begin
      if htons(rr.AClass) = 1 then
        case ntohs(rr.AType) of
          DNSQRY_A: begin
            Move(Ans.PayLoad[AnsStart],Addresses[i],SizeOf(THostAddr));
            inc(Result);
            Inc(AnsStart,htons(RR.RDLength));
          end;
          DNSQRY_CNAME: begin
            if Recurse >= MaxRecursion then begin
              Result := -1;
              exit;
            end;
            rr.rdlength := ntohs(rr.rdlength);
            setlength(cname, rr.rdlength);
            cname := stringfromlabel(ans.payload, ansstart);
            Result := ResolveNameAt(Resolver, cname, Addresses, Recurse+1);
            exit; // FIXME: what about other servers?!
          end;
        end;
        Inc(I);
      end;  
    end;
end;

Function ResolveName(HostName : String; Var Addresses : Array of THostAddr) : Integer;

Var
  I : Integer;

begin
  CheckResolveFile;
  I:=0;
  Result:=0;
  While (Result<=0) and (I<=high(DNSServers)) do
    begin
    Result:=ResolveNameAt(I,HostName,Addresses,0);
    Inc(I);
    end;
end;

//const NoAddress6 : array[0..7] of word = (0,0,0,0,0,0,0,0);

Function ResolveNameAt6(Resolver : Integer; HostName : String; Var Addresses : Array of THostAddr6; Recurse: Integer) : Integer;
                                                                                                                                        
Var
  Qry, Ans            : TQueryData;
  MaxAnswer,I,QryLen,
  AnsLen,AnsStart     : Longint;
  RR                  : TRRData;
  cname               : string;
  LIP4mapped: array[0..MaxIP4Mapped-1] of THostAddr;
  LIP4count: Longint;
                                                                                                                                        
begin
  Result:=0;
  QryLen:=BuildPayLoad(Qry,HostName,DNSQRY_AAAA,1);
  If Not Query(Resolver,Qry,Ans,QryLen,AnsLen) then begin
    // no answer? try IPv4 mapped addresses, maybe that will generate one
    LIP4Count := ResolveName(HostName, LIP4Mapped);
    if LIP4Count > 0 then begin
      inc(LIP4Count); // we loop to LIP4Count-1 later
      if LIP4Count > MaxIP4Mapped then LIP4Count := MaxIP4Mapped;
      if LIP4Count > Length(Addresses) then LIP4Count := Length(Addresses);
      for i := 0 to LIP4Count-2 do begin
        Addresses[i] := NoAddress6;
        Addresses[i].u6_addr16[5] := $FFFF;
        Move(LIP4Mapped[i], Addresses[i].u6_addr16[6], 4);
      end;
      Result := LIP4Count;
    end else begin
      Result:=-1
    end;
  end else
    begin
    AnsStart:=SkipAnsQueries(Ans,AnsLen);
    MaxAnswer:=Ans.h.AnCount-1;
    If MaxAnswer>High(Addresses) then
      MaxAnswer:=High(Addresses);
    I:=0;
    While (I<=MaxAnswer) and NextRR(Ans.Payload,AnsStart,AnsLen,RR) do
      begin
      if (1=NtoHS(RR.AClass)) then
      case ntohs(rr.atype) of
        DNSQRY_AAAA: begin
            Move(Ans.PayLoad[AnsStart],Addresses[i],SizeOf(THostAddr6));
            inc(Result);
            rr.rdlength := ntohs(rr.rdlength);
            Inc(AnsStart,RR.RDLength);
          end;
        DNSQRY_CNAME: begin
          if Recurse >= MaxRecursion then begin
            Result := -1;
            exit;
          end;
          rr.rdlength := ntohs(rr.rdlength);
          setlength(cname, rr.rdlength);
          cname := stringfromlabel(ans.payload, ansstart);
          Result := ResolveNameAt6(Resolver, cname, Addresses, Recurse+1);
          exit; // FIXME: what about other servers?!
        end;
      end;
      Inc(I);
      end;
    end;
end;
                                                                                                                                        


Function ResolveName6(HostName: String; Var Addresses: Array of THostAddr6) : Integer;
var
  i: Integer;
begin
  CheckResolveFile;
  i := 0;
  Result := 0;
  while (Result <= 0) and (I<= high(DNSServers)) do begin
    Result := ResolveNameAt6(I, Hostname, Addresses, 0);
    Inc(i);
  end;
end;

Function ResolveAddressAt(Resolver : Integer; Address : String; Var Names : Array of String; Recurse: Integer) : Integer;


Var
  Qry, Ans            : TQueryData;
  MaxAnswer,I,QryLen,
  AnsLen,AnsStart     : Longint;
  RR                  : TRRData;

begin
  Result:=0;
  QryLen:=BuildPayLoad(Qry,Address,DNSQRY_PTR,1);
  If Not Query(Resolver,Qry,Ans,QryLen,AnsLen) then
    Result:=-1
  else  
    begin
    AnsStart:=SkipAnsQueries(Ans,AnsLen);
    MaxAnswer:=Ans.h.AnCount-1;
    If MaxAnswer>High(Names) then
      MaxAnswer:=High(Names);
    I:=0;
    While (I<=MaxAnswer) and NextRR(Ans.Payload,AnsStart,AnsLen,RR) do
      begin
      Case Ntohs(RR.AType) of
        DNSQRY_PTR:
          if (1=NtoHS(RR.AClass)) then
            begin
            Names[i]:=BuildName(Ans.Payload,AnsStart,AnsLen);
            inc(Result);
            RR.RDLength := ntohs(RR.RDLength);
            Inc(AnsStart,RR.RDLength);
            end;
        DNSQRY_CNAME:
          begin
          if Recurse >= MaxRecursion then
            begin
            Result := -1;
            exit;
            end;
          rr.rdlength := ntohs(rr.rdlength);
          setlength(Address, rr.rdlength);
          address := stringfromlabel(ans.payload, ansstart);
          Result := ResolveAddressAt(Resolver, Address, Names, Recurse+1);
          exit;
          end;
      end;
      Inc(I);
      end;  
    end;
end;


Function ResolveAddress(HostAddr : THostAddr; Var Addresses : Array of String) : Integer;

Var
  I : Integer;
  S : String;
  nt : tnetaddr;
  
begin
  CheckResolveFile;
  I:=0;
  Result:=0;
  nt:=hosttonet(hostaddr);
  S:=Format('%d.%d.%d.%d.in-addr.arpa',[nt.s_bytes[4],nt.s_bytes[3],nt.s_bytes[2],nt.s_bytes[1]]);
  While (Result=0) and (I<=high(DNSServers)) do
    begin
    Result:=ResolveAddressAt(I,S,Addresses,1);
    Inc(I);
    end;
end;

Function ResolveAddress6(HostAddr : THostAddr6; Var Addresses : Array of String) : Integer;

const
  hexdig: string[16] = '0123456789abcdef';
                                                                                
Var
  I : Integer;
  S : ShortString;
                                                                                
begin
  CheckResolveFile;
  Result:=0;
  S := '0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.0.ip6.arpa';
  for i := 7 downto 0 do begin
    S[5+(7-i)*8] := hexdig[1+(HostAddr.u6_addr16[i] and $000F) shr 00];
    S[7+(7-i)*8] := hexdig[1+(HostAddr.u6_addr16[i] and $00F0) shr 04];
    S[1+(7-i)*8] := hexdig[1+(HostAddr.u6_addr16[i] and $0F00) shr 08];
    S[3+(7-i)*8] := hexdig[1+(HostAddr.u6_addr16[i] and $F000) shr 12];
  end;
  I := 0;
  While (Result=0) and (I<=high(DNSServers)) do
    begin
    Result:=ResolveAddressAt(I,S,Addresses,1);
    Inc(I);
    end;
end;

Function HandleAsFullyQualifiedName(const HostName: String) : Boolean;
var
  I,J : Integer;
begin
  Result := False;
  J := 0;
  for I := 1 to Length(HostName) do
    if HostName[I] = '.' then
      begin
      Inc(J);
      if J >= NDots then
        begin
        Result := True;
        Break;
        end;
      end;
end;

Function ResolveHostByName(HostName : String; Var H : THostEntry) : Boolean;

Var
  Address : Array[1..MaxResolveAddr] of THostAddr;
  AbsoluteQueryFirst : Boolean;
  L : Integer;
  K : Integer;

begin
  // Use domain or search-list to append to the searched hostname.
  // When the amount of dots in hostname is higher or equal to ndots,
  // do the query without adding any search-domain first.
  // See the resolv.conf manual for more info.
  if (DefaultDomainList<>'') then
    begin
    // Fill the cached DefaultDomainListArr and NDots
    if (Length(DefaultDomainListArr) = 0) then
      begin
      DefaultDomainListArr := DefaultDomainList.Split(' ',Char(9));
      L := Pos('ndots:', DNSOptions);
      if L > 0 then
        NDots := StrToIntDef(Trim(Copy(DNSOptions, L+6, 2)), 1);
      end;

    AbsoluteQueryFirst := HandleAsFullyQualifiedName(HostName);
    if AbsoluteQueryFirst then
      L:=ResolveName(HostName,Address)
    else
      L := -1;

    K := 0;
    while (L < 1) and (K < Length(DefaultDomainListArr)) do
      begin
      L:=ResolveName(HostName + '.' + DefaultDomainListArr[K],Address);
      Inc(K);
      end;
    end
  else
    begin
    AbsoluteQueryFirst := False;
    L := -1;
    end;

  if (L<1) and not AbsoluteQueryFirst then
    L:=ResolveName(HostName,Address);

  Result:=(L>0);
  If Result then
    begin
    // We could add a reverse call here to get the real name and aliases.
    H.Name:=HostName;
    H.Addr:=Address[1];
    H.aliases:='';
    end;
end;

Function ResolveHostByName6(HostName : String; Var H : THostEntry6) : Boolean;

Var
  Address : Array[1..MaxResolveAddr] of THostAddr6;
  L : Integer;
  
begin
  L:=ResolveName6(HostName,Address);
  Result:=(L>0);
  If Result then
    begin
    // We could add a reverse call here to get the real name and aliases.
    H.Name:=HostName;
    H.Addr:=Address[1];
    H.aliases:='';
    end;
end;


Function ResolveHostByAddr(HostAddr : THostAddr; Var H : THostEntry) : Boolean;

Var
  Names : Array[1..MaxResolveAddr] of String;
  I,L : Integer;
  
begin
  L:=ResolveAddress(HostAddr,Names);
  Result:=(L>0);
  If Result then
    begin
    H.Name:=Names[1];
    H.Addr:=HostAddr;
    H.Aliases:='';
    If (L>1) then
      For I:=2 to L do
        If (I=2) then
          H.Aliases:=Names[i]
        else  
          H.Aliases:=H.Aliases+','+Names[i];
    end;
end;

Function ResolveHostByAddr6(HostAddr : THostAddr6; Var H : THostEntry6) : Boolean;

Var
  Names : Array[1..MaxResolveAddr] of String;
  I,L : Integer;
  
begin
  L:=ResolveAddress6(HostAddr,Names);
  Result:=(L>0);
  If Result then
    begin
    H.Name:=Names[1];
    H.Addr:=HostAddr;
    H.Aliases:='';
    If (L>1) then
      For I:=2 to L do
        If (I=2) then
          H.Aliases:=Names[i]
        else  
          H.Aliases:=H.Aliases+','+Names[i];
    end;
end;




//const NoAddress : in_addr = (s_addr: 0);

Function GetHostByName(HostName: String;  Var H : THostEntry) : boolean;

begin
  Result:=FindHostEntryInHostsFile(HostName,NoAddress,H);
end;


Function GetHostByAddr(Addr: THostAddr;  Var H : THostEntry) : boolean;

begin
  Result:=FindHostEntryInHostsFile('',Addr,H);
end;


{ ---------------------------------------------------------------------
    /etc/protocols handling.
  ---------------------------------------------------------------------}

Function GetNextProtoEntry(var F : Text; Var H : TProtocolEntry): boolean;

Var
  Line,S : String;
  I      : integer;

begin
  Result:=False;
  Repeat
    ReadLn(F,Line);
    StripComment(Line);
    S:=NextWord(Line);
    If (S<>'') then
      begin
        H.Name:=S;
        S:=NextWord(Line);
	i:=strtointdef(s,-1);
        If (i<>-1) then
          begin
          H.number:=i;
          Result:=True;
          H.Aliases:='';
          Repeat
            S:=NextWord(line);
            If (S<>'') then
              If (H.Aliases='') then
                H.Aliases:=S
              else
                H.Aliases:=H.Aliases+','+S;
          until (S='');
          end;
      end;
  until Result or EOF(F);
end;

Function FindProtoEntryInProtoFile(N: String; prot: integer; Var H : TProtocolEntry) : boolean;

Var
  F : Text;
  HE : TProtocolEntry;

begin
  Result:=False;
  If FileExists (EtcPath + SProtocolFile) then
    begin
    Assign (F, EtcPath + SProtocolFile);
    {$push}{$i-}
    Reset(F);
    {$pop}
    If (IOResult=0) then
      begin
      While Not Result and GetNextProtoEntry(F,HE) do
        begin
        If (N<>'') then
          Result:=MatchNameOrAlias(N,HE.Name,HE.Aliases)
        else
          Result:=prot=he.number;
        end;
      Close(f);
      If Result then
        begin
        H.Name:=HE.Name;
        H.number:=he.number;
        H.Aliases:=HE.Aliases;
        end;
      end;
    end;
end;

Function GetProtocolByName(ProtoName: String;  Var H : TProtocolEntry) : boolean;

begin
  Result:=FindProtoEntryInProtoFile(ProtoName,0,H);
end;


Function GetProtocolByNumber(proto: Integer;  Var H : TProtocolEntry) : boolean;

begin
  Result:=FindProtoEntryInProtoFile('',Proto,H);
end;

{ ---------------------------------------------------------------------
    /etc/networks handling
  ---------------------------------------------------------------------}

function StrTonetpartial( IP : AnsiString) : in_addr ;

Var
    Dummy : AnsiString;
    I,j,k     : Longint;
//    Temp : in_addr;

begin
  strtonetpartial.s_addr:=0;              //:=NoAddress;
  i:=0; j:=0;
  while (i<4) and (j=0) do
   begin
     J:=Pos('.',IP);
     if j=0 then j:=length(ip)+1;
     Dummy:=Copy(IP,1,J-1);
     Delete (IP,1,J);
     Val (Dummy,k,J);
     if j=0 then
      strtonetpartial.s_bytes[i+1]:=k;
     inc(i);
   end;
   if (i=0) then strtonetpartial.s_addr:=0;
end;

Function GetNextNetworkEntry(var F : Text; Var N : TNetworkEntry): boolean;

Var
  NN,Line,S : String;
  A : TNetAddr;

begin
  Result:=False;
  Repeat
    ReadLn(F,Line);
    StripComment(Line);
    S:=NextWord(Line);
    If (S<>'') then
      begin
      NN:=S;
      A:=StrTonetpartial(NextWord(Line));
      Result:=(NN<>'') and (A.s_bytes[1]<>0); // Valid addr.
      If result then
        begin
        N.Addr.s_addr:=A.s_addr; // keep it host.
        N.Name:=NN;
        N.Aliases:='';
        end;
      end;
  until Result or EOF(F);
end;

Function FindNetworkEntryInNetworksFile(Net: String; Addr: TNetAddr; Var N : TNetworkEntry) : boolean;

Var
  F : Text;
  NE : TNetworkEntry;

begin
  Result:=False;
  If FileExists (EtcPath + SNetworksFile) then
    begin
    Assign (F, EtcPath + SNetworksFile);
    {$push}{$i-}
    Reset(F);
    {$pop}
    If (IOResult=0) then
      begin
      While Not Result and GetNextNetworkEntry(F,NE) do
        begin
        If (Net<>'') then
          Result:=MatchNameOrAlias(Net,NE.Name,NE.Aliases)
        else
          Result:=Cardinal(Addr)=Cardinal(NE.Addr);
        end;
      Close(f);
      If Result then
        begin
        N.Name:=NE.Name;
        N.Addr:=nettohost(NE.Addr);
        N.Aliases:=NE.Aliases;
        end;
      end;
    end;
end;

Const NoNet : in_addr = (s_addr:0);

Function GetNetworkByName(NetName: String; Var N : TNetworkEntry) : boolean;

begin
  Result:=FindNetworkEntryInNetworksFile(NetName,NoNet,N);
end;

Function GetNetworkByAddr(Addr: THostAddr; Var N : TNetworkEntry) : boolean;

begin
  Result:=FindNetworkEntryInNetworksFile('',Addr,N);
end;

{ ---------------------------------------------------------------------
    /etc/services section
  ---------------------------------------------------------------------}

Function GetNextServiceEntry(Var F : Text; Var E : TServiceEntry) : Boolean;


Var
  Line,S : String;
  P : INteger;

begin
  Result:=False;
  Repeat
    ReadLn(F,Line);
    StripComment(Line);
    S:=NextWord(Line);
    If (S<>'') then
      begin
      E.Name:=S;
      S:=NextWord(Line);
      P:=Pos('/',S);
      If (P<>0) then
        begin
        E.Port:=StrToIntDef(Copy(S,1,P-1),0);
        If (E.Port<>0) then
          begin
          E.Protocol:=Copy(S,P+1,Length(S)-P);
          Result:=length(E.Protocol)>0;
          E.Aliases:='';
          Repeat
            S:=NextWord(Line);
            If (S<>'') then
              If (Length(E.Aliases)=0) then
                E.aliases:=S
              else
                E.Aliases:=E.Aliases+','+S;
          until (S='');
          end;
        end;
      end;
  until Result or EOF(F);
end;


Function FindServiceEntryInFile(Const Name,Proto : String; Port : Integer; Var E : TServiceEntry) : Boolean;

Var
  F : Text;
  TE : TServiceEntry;

begin
  Result:=False;
  If FileExists (EtcPath + SServicesFile) then
    begin
    Assign (F, EtcPath + SServicesFile);
    {$push}{$i-}
    Reset(F);
    {$pop}
    If (IOResult=0) then
      begin
      While Not Result and GetNextServiceEntry(F,TE) do
        begin
        If (Port=-1) then
          Result:=MatchNameOrAlias(Name,TE.Name,TE.Aliases)
        else
          Result:=(Port=TE.Port);
        If Result and (Proto<>'') then
          Result:=(Proto=TE.Protocol);
        end;
      Close(f);
      If Result then
        begin
        E.Name:=TE.Name;
        E.Port:=TE.Port;
        E.Protocol:=TE.Protocol;
        E.Aliases:=TE.Aliases;
        end;
      end;
    end;
end;

Function GetServiceByName(Const Name,Proto : String; Var E : TServiceEntry) : Boolean;

begin
  Result:=FindServiceEntryInFile(Name,Proto,-1,E);
end;

Function GetServiceByPort(Port : Word;Const Proto : String; Var E : TServiceEntry) : Boolean;

begin
  Result:=FindServiceEntryInFile('',Proto,Port,E);
end;

{ ---------------------------------------------------------------------
    Initialization section
  ---------------------------------------------------------------------}

Procedure InitResolver;

begin
  TimeOutS :=5;
  TimeOutMS:=0;
  CheckHostsFileAge:=False;
{$IFDEF UNIX_ETC}
  EtcPath := '/etc/';
{$ELSE UNIX_ETC}
 {$IFDEF ETC_BY_ENV}
  EtcPath := GetEnvironmentVariable ('ETC');
  if (EtcPath <> '') and (EtcPath [Length (EtcPath)] <> DirectorySeparator) then
   EtcPath := EtcPath + DirectorySeparator;
 {$ELSE ETC_BY_ENV}
{$WARNING Support for finding /etc/ directory not implemented for this platform!}

 {$ENDIF ETC_BY_ENV}
{$ENDIF UNIX_ETC}
  If FileExists (EtcPath + SHostsFile) then
    HostsList := ProcessHosts (EtcPath + SHostsFile);
{$ifdef android}
  CheckResolveFileAge:=True;
  CheckResolveFile;
{$else}
  CheckResolveFileAge:=False;
  If FileExists(EtcPath + SResolveFile) then
    GetDNsservers(EtcPath + SResolveFile)
{$endif android}
{$IFDEF OS2}
  else if FileExists(EtcPath + SResolveFile2) then
    GetDNsservers(EtcPath + SResolveFile2)
{$ENDIF OS2}
                                         ;
end;

Procedure DoneResolver;

begin
  FreeHostsList(HostsList);
end;

{$else FPC_USE_LIBC}

{ ---------------------------------------------------------------------
    Implementation based on libc
  ---------------------------------------------------------------------}

Function ResolveName(const HostName : String; Addresses: pointer; MaxAddresses, Family: integer) : Integer;
var
  h: TAddrInfo;
  res, ai: PAddrInfo;
begin
  Result:=-1;
  if MaxAddresses = 0 then
    exit;
  FillChar(h, SizeOf(h), 0);
  h.ai_family:=Family;
  h.ai_socktype:=SOCK_STREAM;
  res:=nil;
  if (getaddrinfo(PChar(HostName), nil, @h, @res) <> 0) or (res = nil) then
    exit;
  Result:=0;
  ai:=res;
  repeat
    if ai^.ai_family = Family then begin
      if Family = AF_INET then begin
        Move(PInetSockAddr(ai^.ai_addr)^.sin_addr, Addresses^, SizeOf(TInAddr));
        Inc(PInAddr(Addresses));
      end
      else begin
        Move(PInetSockAddr6(ai^.ai_addr)^.sin6_addr, Addresses^, SizeOf(TIn6Addr));
        Inc(PIn6Addr(Addresses));
      end;
      Inc(Result);
    end;
    ai:=ai^.ai_next;
  until (ai = nil) or (Result >= MaxAddresses);
  freeaddrinfo(res);
end;

Function ResolveName(HostName : String; Var Addresses : Array of THostAddr) : Integer;
begin
  Result:=ResolveName(HostName, @Addresses, Length(Addresses), AF_INET);
end;

Function ResolveName6(HostName : String; Var Addresses : Array of THostAddr6) : Integer;
begin
  Result:=ResolveName(HostName, @Addresses, Length(Addresses), AF_INET6);
end;

Function ResolveAddress(Addr : pointer; AddrLen: integer; Var Names : Array of String) : Integer;
var
  n: ansistring;
begin
  Result:=-1;
  if Length(Names) = 0 then
    exit;
  n:='';
  SetLength(n, NI_MAXHOST);
  if getnameinfo(Addr, AddrLen, @n[1], Length(n), nil, 0, 0) = 0 then begin
    Names[Low(Names)]:=PAnsiChar(n);
    Result:=1;
  end;
end;

Function ResolveAddress(HostAddr : THostAddr; Var Addresses : Array of String) : Integer;
var
  a: TInetSockAddr;
begin
  FillChar(a, SizeOf(a), 0);
  a.sin_family:=AF_INET;
  a.sin_addr.s_addr:=htonl(HostAddr.s_addr);
  Result:=ResolveAddress(@a, SizeOf(a), Addresses);
end;

Function ResolveAddress6(HostAddr: THostAddr6; var Addresses: Array of string) : Integer;
var
  a: TInetSockAddr6;
begin
  FillChar(a, SizeOf(a), 0);
  a.sin6_family:=AF_INET6;
  Move(HostAddr, a.sin6_addr, SizeOf(TInetSockAddr6));
  Result:=ResolveAddress(@a, SizeOf(a), Addresses);
end;

Function ResolveHostByName(HostName : String; Var H : THostEntry) : Boolean;
Var
  Address : Array[1..1] of THostAddr;
begin
  Result:=ResolveName(HostName,Address) > 0;
  if Result then begin
    H.Name:=HostName;
    H.Addr:=Address[1];
    H.aliases:='';
  end;
end;

Function ResolveHostByName6(Hostname : String; Var H : THostEntry6) : Boolean;
Var
  Address : Array[1..1] of THostAddr6;
begin
  Result:=ResolveName6(HostName,Address) > 0;
  if Result then begin
    H.Name:=HostName;
    H.Addr:=Address[1];
    H.aliases:='';
  end;
end;

Function ResolveHostByAddr(HostAddr : THostAddr; Var H : THostEntry) : Boolean;
Var
  Names : Array[1..MaxResolveAddr] of String;
  I,L : Integer;
begin
  L:=ResolveAddress(HostAddr,Names);
  Result:=(L>0);
  If Result then
    begin
    H.Name:=Names[1];
    H.Addr:=HostAddr;
    H.Aliases:='';
    If (L>1) then
      For I:=2 to L do
        If (I=2) then
          H.Aliases:=Names[i]
        else
          H.Aliases:=H.Aliases+','+Names[i];
    end;
end;

Function ResolveHostByAddr6(HostAddr : THostAddr6; Var H : THostEntry6) : Boolean;
Var
  Names : Array[1..MaxResolveAddr] of String;
  I,L : Integer;
begin
  L:=ResolveAddress6(HostAddr,Names);
  Result:=(L>0);
  If Result then
    begin
    H.Name:=Names[1];
    H.Addr:=HostAddr;
    H.Aliases:='';
    If (L>1) then
      For I:=2 to L do
        If (I=2) then
          H.Aliases:=Names[i]
        else
          H.Aliases:=H.Aliases+','+Names[i];
    end;
end;

Function GetHostByName(HostName: String;  Var H : THostEntry) : boolean;
begin
  Result:=False;
end;

Function GetHostByAddr(Addr: THostAddr;  Var H : THostEntry) : boolean;
begin
  Result:=False;
end;

function PPCharToString(list: PPChar): string;
begin
  Result:='';
  if list = nil then
    exit;
  while list^ <> nil do begin
    if Length(Result) = 0 then
      Result:=list^
    else
      Result:=Result + ',' + list^;
    Inc(list);
  end;
end;

Function GetNetworkByName(NetName: String; Var N : TNetworkEntry) : boolean;
var
  ne: PNetEnt;
begin
  ne:=getnetbyname(PAnsiChar(NetName));
  Result:=ne <> nil;
  if Result then begin
    N.Name:=ne^.n_name;
    N.Addr.s_addr:=ne^.n_net;
    N.Aliases:=PPCharToString(ne^.n_aliases);
  end;
end;

Function GetNetworkByAddr(Addr: THostAddr; Var N : TNetworkEntry) : boolean;
var
  ne: PNetEnt;
begin
  ne:=getnetbyaddr(htonl(Addr.s_addr), AF_INET);
  Result:=ne <> nil;
  if Result then begin
    N.Name:=ne^.n_name;
    N.Addr.s_addr:=ne^.n_net;
    N.Aliases:=PPCharToString(ne^.n_aliases);
  end;
end;

Function GetServiceByName(Const Name,Proto : String; Var E : TServiceEntry) : Boolean;
var
  se: PServEnt;
begin
  se:=getservbyname(PAnsiChar(Name), PAnsiChar(Proto));
  Result:=se <> nil;
  if Result then begin
    E.Name:=se^.s_name;
    E.Port:=NToHs(se^.s_port);
    E.Protocol:=se^.s_proto;
    E.Aliases:=PPCharToString(se^.s_aliases);
  end;
end;

Function GetServiceByPort(Port : Word;Const Proto : String; Var E : TServiceEntry) : Boolean;
var
  se: PServEnt;
begin
  se:=getservbyport(htons(Port), PAnsiChar(Proto));
  Result:=se <> nil;
  if Result then begin
    E.Name:=se^.s_name;
    E.Port:=NToHs(se^.s_port);
    E.Protocol:=se^.s_proto;
    E.Aliases:=PPCharToString(se^.s_aliases);
  end;
end;

Function GetProtocolByName(ProtoName: String;  Var H : TProtocolEntry) : boolean;
var
  pe: PProtoEnt;
begin
  pe:=getprotobyname(PAnsiChar(ProtoName));
  Result:=pe <> nil;
  if Result then begin
    H.Name:=pe^.p_name;
    H.Number:=pe^.p_proto;
    h.Aliases:=PPCharToString(pe^.p_aliases);
  end;
end;

Function GetProtocolByNumber(proto: Integer;  Var H : TProtocolEntry) : boolean;
var
  pe: PProtoEnt;
begin
  pe:=getprotobynumber(proto);
  Result:=pe <> nil;
  if Result then begin
    H.Name:=pe^.p_name;
    H.Number:=pe^.p_proto;
    h.Aliases:=PPCharToString(pe^.p_aliases);
  end;
end;

Procedure InitResolver; inline;
begin
end;

Procedure DoneResolver; inline;
begin
end;

{$endif FPC_USE_LIBC}

{ ---------------------------------------------------------------------
    Common routines
  ---------------------------------------------------------------------}

function IN6_IS_ADDR_V4MAPPED(HostAddr: THostAddr6): boolean;
begin
  Result :=
   (HostAddr.u6_addr16[0] = 0) and
   (HostAddr.u6_addr16[1] = 0) and
   (HostAddr.u6_addr16[2] = 0) and
   (HostAddr.u6_addr16[3] = 0) and
   (HostAddr.u6_addr16[4] = 0) and
   (HostAddr.u6_addr16[5] = $FFFF);
end;

Initialization
  InitResolver;
Finalization
  DoneResolver;  
end.

