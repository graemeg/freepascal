unit netdbtest;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fpcunit, testregistry, Sockets, math, netdb;

const
  FAKETLD = 'doesnotexist';
  FAKEDOMAIN = 'fakedomain';

  FAKEFQDN=FAKEDOMAIN+'.'+FAKETLD;

type
  TDomainCompressionOffset = packed record
    nm: String;
    offset: Word;
  end;
  TDomainCompressionTable = Array of TDomainCompressionOffset;

  TTwoByteArr = array[0 .. 1] of Byte;
  TDNSDomainPointer = packed record
    case b: boolean of
      true: (ba: TTwoByteArr);
      false: (b1,b2: Byte);
  end;

  TDNSDomainByteStream = packed record
    ulabels: Array of byte;
    cptr: Word;
  end;

  TBuffer = Array of Byte;

  // can't use dynamic arrays in variant records, so fudge things by
  // having between 1 and 5 subsstrings per text RR. it's good enough
  // for these tests.
  TTextArray = array [1 .. 5] of ShortString;

  TFakeQuery = record
    nm: ShortString;
    qtype, qclass: Word;
  end;

  TFakeSOA = record
    mn,rn: ShortString;
    serial,refresh,retry,expire,min: Cardinal;
  end;
  TFakeMX = record
    pref: Word;
    exch: ShortString;
  end;
  TFakeSRV = record
    priority, weight, port: Word;
    target: ShortString;
  end;

  TFakeRR = record
    RRName   : ShortString;
    AClass   : Word;
    TTL      : Cardinal;
    RDLength : Word;
    case Atype: Word of
      DNSQRY_A: (ip: THostAddr);
      DNSQRY_AAAA: (ip6: THostAddr6);
      DNSQRY_CNAME: (cn: ShortString);
      DNSQRY_MX: (fmx: TFakeMX);
      DNSQRY_NS: (nsh: ShortString);
      DNSQRY_PTR: (ptr: ShortString);
      DNSQRY_SOA: (fsoa: TFakeSoa);
      DNSQRY_TXT: (sstrcount: Byte; txtarr: TTextArray);
      DNSQRY_SRV: (fsrv: TFakeSRV);
  end;

  TRRSection = Array of TFakeRR;

  TFakeDNSResponse = record
    strtable: TDomainCompressionTable;
    compresslabels: Boolean;
    hdr: TDNSHeader;
    qry: TFakeQuery;
    answers, authority, additional: TRRSection;
  end;

  TRDataWriteRes = packed record
    bw, etw: Word;
  end;

  { TNetDbTest }

  TNetDbTest= class(TTestCase)
  strict private
    tsl: TStringList;
  protected
    procedure SetUp; override;
    procedure TearDown; override;

    procedure BuildFakeRR_A(out RR: TFakeRR; nm: String; ttl: Cardinal;
      val: String);
    procedure BuildFakeRR_AAAA(out RR: TFakeRR; nm: String; ttl: Cardinal;
      val: String);
    procedure BuildFakeRR_MX(out RR: TFakeRR; nm: String; ttl: Cardinal;
      pref: Word; exch: ShortString );
    procedure BuildFakeRR_NS(out RR: TFakeRR; nm: String; ttl: Cardinal;
      val: String);
    procedure BuildFakeRR_PTR(out RR: TFakeRR; nm: String; ttl: Cardinal;
      val: String);
    procedure BuildFakeRR_CNAME(out RR: TFakeRR; nm: String; ttl: Cardinal;
      val: String);
    procedure BuildFakeRR_SOA(out RR: TFakeRR; nm: String; ttl: Cardinal;
      mn,rn: ShortString; serial,refresh,retry,expire,min: Cardinal);
    procedure BuildFakeRR_TXT(out RR: TFakeRR; nm: String; ttl: Cardinal;
      n: Byte; txt: TTextArray);
    procedure BuildFakeRR_SRV(out RR: TFakeRR; nm: String; ttl: Cardinal;
       priority, weight, port: Word; target: ShortString);

    procedure CopyBytesTo(var buf: TPayLoad; startidx,destidx,count: Word);
    procedure CopyBytesTo(var buf: TPayLoadTCP; startidx,destidx,count: Word);

    function WriteNumToBuffer(var buf: TBuffer; var offset: Cardinal;
       val: Word): Word;
    function WriteNumToBuffer(var buf: TBuffer; var offset: Cardinal;
       val: Cardinal): Word;

    function WriteNumToBufferN(var buf: TBuffer; var offset: Cardinal;
       val: Word): Word;
    function WriteNumToBufferN(var buf: TBuffer; var offset: Cardinal;
       val: Cardinal): Word;

    function WriteMXAsRData(var buf: TBuffer; var offset: Cardinal;
      fmx: TFakeMX): TRDataWriteRes;
    function WriteSOAasRData(var buf: TBuffer; var offset: Cardinal;
      fsoa: TFakeSOA): TRDataWriteRes;
    function WriteSOAasRData(var buf: TBuffer; var offset: Cardinal;
      fsoa: TFakeSOA; var ctbl: TDomainCompressionTable): TRDataWriteRes;
    function WriteAAAAasRData(var buf: TBuffer; var offset: Cardinal;
      ip6: THostAddr6): TRDataWriteRes;
    function WriteAasRData(var buf: TBuffer; var offset: Cardinal;
      ip: THostAddr): TRDataWriteRes;

    function WriteSRVasRData(var buf: TBuffer; var offset: Cardinal;
      fsrv: TFakeSRV): TRDataWriteRes;
    function WriteSRVasRData(var buf: TBuffer; var offset: Cardinal;
      fsrv: TFakeSRV; var ctbl: TDomainCompressionTable): TRDataWriteRes;

    function WriteMXAsRData(var buf: TBuffer; var offset: Cardinal;
      fmx: TFakeMX; var ctbl: TDomainCompressionTable): TRDataWriteRes;

    function CalcRdLength(o: TDNSDomainByteStream): Word;
    function CalcRdLength(o: TTextArray): Word;
    function WriteTextRecAsRData(var buf: TBuffer; var offset: Cardinal;
      tt: TTextArray): TRDataWriteRes;

    function DomainNameToByteStream(nm: ShortString;
      var ctbl: TDomainCompressionTable): TDNSDomainByteStream;
    function DomainNameToByteStream(nm: ShortString): TDNSDomainByteStream;

    function WriteDNSDomainByteStreamToBuffer(var buf: TBuffer;
      var offset: Cardinal; dbs: TDNSDomainByteStream): Word;

    function WriteDomainAsRdata(var buf: TBuffer; var offset: Cardinal;
      dbs: TDNSDomainByteStream): TRDataWriteRes;

    function WriteRRToBuffer(var buf: TBuffer; var offset: Cardinal;
      rr: TFakeRR): Word;
    function WriteRRToBuffer(var buf: TBuffer; var offset: Cardinal;
      rr: TFakeRR; var ctbl: TDomainCompressionTable): Word;
    function FakeDNSResponseToByteBuffer(fdr: TFakeDNSResponse;
      out buf: TBuffer; compress: Boolean = False): Cardinal;
    function BufferToPayload(const buf: TBuffer; out pl: TPayload): Boolean;
    function BufferToPayload(const buf: TBuffer; out pl: TPayLoadTCP): Boolean;

    function BuildQueryData(fdr: TFakeDNSResponse; out qd: TQueryData;
      out qlen: Word; Compress: Boolean = False): Boolean;

    function BuildQueryData(fdr: TFakeDNSResponse;
      out qd: TQueryDataLengthTCP; out qlen: Word;
        Compress: Boolean = False): Boolean;

    function BuildTruncatedQueryData(fdr: TFakeDNSResponse; out qd: TQueryData;
      out qlen: Word; truncoffset: Word): Boolean;

    procedure BuildFakeResponseA(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponseAAAA(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponseMX(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponseSOA(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponseCNAME(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponseNS(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponsePTR(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponseTXT(nm: ShortString; out fr: TFakeDNSResponse);
    procedure BuildFakeResponseSRV(nm: ShortString; out fr: TFakeDNSResponse);

  published
    procedure TestBuildPayloadSimple;
    procedure TestBuildPayloadSimpleEmpty;
    procedure TestBuildPayloadSimpleEndDot;
    procedure TestBuildPayloadSimpleStartDot;
    procedure TestBuildPayloadSimpleMultipleDot;

    { * straightforward tests for the api with valid data. Have to test each
      * known RR type with both TCP and UDP buffer functions, and with and
      * without compression of domain names.
      * No network calls will be made. These tests hit all functions for
      * processing dns requests except network functions.}
    procedure TestDnsQueryUDP_A;
    procedure TestDnsQueryTCP_A;
    procedure TestDnsQueryCompressUDP_A;
    procedure TestDnsQueryCompressTCP_A;

    procedure TestDnsQueryUDP_AAAA;
    procedure TestDnsQueryTCP_AAAA;
    procedure TestDnsQueryCompressUDP_AAAA;
    procedure TestDnsQueryCompressTCP_AAAA;

    procedure TestDnsQueryUDP_MX;
    procedure TestDnsQueryTCP_MX;
    procedure TestDnsQueryCompressUDP_MX;
    procedure TestDnsQueryCompressTCP_MX;

    procedure TestDnsQueryUDP_SOA;
    procedure TestDnsQueryTCP_SOA;
    procedure TestDnsQueryCompressUDP_SOA;
    procedure TestDnsQueryCompressTCP_SOA;

    procedure TestDnsQueryUDP_CNAME;
    procedure TestDnsQueryTCP_CNAME;
    procedure TestDnsQueryCompressUDP_CNAME;
    procedure TestDnsQueryCompressTCP_CNAME;

    procedure TestDnsQueryUDP_NS;
    procedure TestDnsQueryTCP_NS;
    procedure TestDnsQueryCompressUDP_NS;
    procedure TestDnsQueryCompressTCP_NS;

    procedure TestDnsQueryUDP_PTR;
    procedure TestDnsQueryTCP_PTR;
    procedure TestDnsQueryCompressUDP_PTR;
    procedure TestDnsQueryCompressTCP_PTR;

    procedure TestDnsQueryUDP_TXT;
    procedure TestDnsQueryTCP_TXT;
    procedure TestDnsQueryCompressUDP_TXT;
    procedure TestDnsQueryCompressTCP_TXT;

    procedure TestDnsQueryUDP_SRV;
    procedure TestDnsQueryTCP_SRV;
    procedure TestDnsQueryCompressUDP_SRV;
    procedure TestDnsQueryCompressTCP_SRV;

    {
      * Tests with invalid input data. These attempt to simulate a hostile
      * dns server returning deliberately invalid data in an attempt to
      * cause a buffer overflow, memory corruption, or DDOS.
    }

    // buffer truncated so RRs have invalid types.
    procedure TestDnsQueryTruncateRR_UDP_A;

    {
     * Tests of DNSRRGet* functions where RR is near the end of the buffer,
     * testing both when the RR just fits, and when it doesn't.
     }
    procedure TestDnsRRBufferEdgeA;
    procedure TestDnsRRBufferPastEdgeA;
    procedure TestDnsRRBufferEdgeAAAA;
    procedure TestDNsRRBufferPastEdgeAAAA;
    procedure TestDnsRRBufferEdgeMX;
    procedure TestDnsRRBufferPastEdgeMX;
    procedure TestDnsRRBufferEdgeSOA;
    procedure TestDnsRRBufferPastEdgeSOA;
    procedure TestDnsRRBufferEdgeSRV;
    procedure TestDnsRRBufferPastEdgeSRV;
    procedure TestDnsRRBufferEdgeCNAME;
    procedure TestDnsRRBufferPastEdgeCNAME;
    procedure TestDnsRRBufferEdgeNS;
    procedure TestDnsRRBufferPastEdgeNS;
    procedure TestDnsRRBufferEdgePTR;
    procedure TestDnsRRBufferPastEdgePTR;
    procedure TestDnsRRBufferEdgeTXT;
    procedure TestDnsRRBufferPastEdgeTXT;


    {
     * the TCP variants. identical code, but qd variable is a different type
     * and so different paths get followed in netdb.
    }
    procedure TestDnsRRBufferEdgeTCPA;
    procedure TestDnsRRBufferPastEdgeTCPA;
    procedure TestDnsRRBufferEdgeTCPAAAA;
    procedure TestDNsRRBufferPastEdgeTCPAAAA;
    procedure TestDnsRRBufferEdgeTCPMX;
    procedure TestDnsRRBufferPastEdgeTCPMX;
    procedure TestDnsRRBufferEdgeTCPSOA;
    procedure TestDnsRRBufferPastEdgeTCPSOA;
    procedure TestDnsRRBufferEdgeTCPSRV;
    procedure TestDnsRRBufferPastEdgeTCPSRV;
    procedure TestDnsRRBufferEdgeTCPCNAME;
    procedure TestDnsRRBufferPastEdgeTCPCNAME;
    procedure TestDnsRRBufferEdgeTCPNS;
    procedure TestDnsRRBufferPastEdgeTCPNS;
    procedure TestDnsRRBufferEdgeTCPPTR;
    procedure TestDnsRRBufferPastEdgeTCPPTR;
    procedure TestDnsRRBufferEdgeTCPTXT;
    procedure TestDnsRRBufferPastEdgeTCPTXT;

    // Testing of NextNameRR at buffer edge and beyond. this differs from
    // the above tests in that they tests DNSGet* at the edge, but NextNameRR
    // is never called to read at the edge in those functions.
    // Because NextNameRR does nothing that is specific to RR types it's
    // not necessary to test with each type of RR.

    procedure TestNextNameRREdgeA;
    procedure TestNextNameRRPastEdgeA;
    procedure TestNextNameRREdgeTCPA;
    procedure TestNextNameRRPastEdgeTCPA;

    {
     * Test GetRRrecords at and beyond buffer boundaries.
    }
    procedure TestGetRRrecordsInvalidStart;
    procedure TestGetRRrecordsInvalidStartTCP;

    {
    Tests for GetFixlenStr
    }
    procedure TestGetFixLenStrSimple;
    procedure TestGetFixLenStrSimpleTCP;
    procedure TestGetFixLenStrSimpleAtEdge;
    procedure TestGetFixLenStrSimpleTCPAtEdge;
    procedure TestGetFixLenStrSimplePastEdge;
    procedure TestGetFixLenStrSimpleTCPPastEdge;


    {
     * Test stringfromlabel with buffer edges and beyond. Its behaviour
     * at present is to drop any label that would exceed the buffer boundary
     * but still return any other labels successfully received.

     * Some of the previous tests already verify what happens with a label
     * that occurs on the edge. See the tests for TestDnsRRBufferEdgeSRV
     * and TestDnsRRBufferEdgeTCPSRV, TestDnsRRBufferEdgeCNAME, etc.
    }

    // read a label starting at the end of the buffer where the count is
    // greater than 0.
    procedure TestStringFromLabelCountAsLastByte;
    procedure TestStringFromLabelCountAsLastByteTCP;

    // compressed label
    procedure TestStringFromLabelCompress;
    procedure TestStringFromLabelCompressTCP;
    // another compressed label test, this time with one uncompressed label
    procedure TestStringFromLabelCompressWithUncompressedLabel;
    // as above, but on the tcp payload buffer
    procedure TestStringFromLabelCompressWithUncompressedLabelTCP;
    // compressed label at the edge of the buffer
    procedure TestStringFromLabelCompressEndBuffer;
    // compressed label at the edge of the tcp buffer
    procedure TestStringFromLabelCompressEndBufferTCP;
    // test stringfromlabel when last byte is 192. 192 is the signal
    // that the next byte is a pointer offset, but of course there's
    // no next byte.
    procedure TestStringFromLabelCompressSplit;
    // repeat using TCP buffer variant
    procedure TestStringFromLabelCompressSplitTCP;
    // test that stringfromlabel rejects pointers that go forward. per
    // rfc 1035, pointers must go backward.
    procedure TestStringFromLabelCompressPtrFwd;
    procedure TestStringFromLabelCompressPtrFwdTCP;
    // fill buffer with 192, pointer marker, then try stringfromlabel on it.
    procedure TestStringFromLabelCompressAllPtrStart;
    procedure TestStringFromLabelCompressAllPtrStartTCP;

    // test string from label where second byte is 0.
    procedure TestStringFromLabelCompressedZero;
    procedure TestStringFromLabelCompressedZeroTCP;

    // test whether an infinite loop can be triggered.
    procedure TestStringFromLabelInfiniteLoop;
    procedure TestStringFromLabelInfiniteLoopTCP;

    // test short domain less than 12 chars. this tests that dns pointer
    // calculations in stringfromlabel are correct
    procedure TestCompressShortDomain;
    procedure TestCompressShortDomainTCP;
  end;

implementation

procedure dump_payload(const pl: TBuffer);
var
  idx,llen: Cardinal;
begin
  idx := 0;
  llen := 0;
  for idx := 0 to Length(pl) - 1 do
  begin
    write('['+inttostr(idx)+'] '+IntToHex(pl[idx],2));
    if (pl[idx] > 48) and (pl[idx] < 123) then
      write(' ' + chr(pl[idx]))
    else
      write(' .');
    write(' ');
    Inc(llen);
    if llen >= 6 then
    begin
      llen := 0;
      writeln();
    end;
  end;
  if llen > 0 then
  begin
    writeln();
  end;
end;

procedure dump_payload(const pl: TPayload; count: Word);
var
  idx,llen: Cardinal;
begin
  idx := 0;
  llen := 0;
  for idx := 0 to count - 1 do
  begin
    write('['+inttostr(idx)+'] '+IntToHex(pl[idx],2));
    if (pl[idx] > 48) and (pl[idx] < 123) then
      write(' ' + chr(pl[idx]))
    else
      write(' .');
    write(' ');
    Inc(llen);
    if llen >= 6 then
    begin
      llen := 0;
      writeln();
    end;
  end;
  if llen > 0 then
  begin
    writeln();
  end;
end;

function LookupStr(ls: String; stt: TDomainCompressionTable; out idx: Word): Boolean;
var
  so: TDomainCompressionOffset;
begin
  Result := False;
  for so in stt do
  begin
    if ls = so.nm then
    begin
      Result := True;
      idx := so.offset;
      exit;
    end;
  end;
end;

function AddStr(ls: String; var stt: TDomainCompressionTable; idx: Word): Boolean;
var
  so: TDomainCompressionOffset;
begin
  so.nm := ls;
  so.offset := idx;
  SetLength(stt, Length(stt)+1);
  stt[Length(stt)-1] := so;
  Result := True;
end;

function GetDnsDomainPointer(offset: Word): TDNSDomainPointer;
begin
  Result.b1 := 0;
  Result.b2 := 0;
  // dns comp. ptr can't be > 2 ** 14 or 16383
  if offset > 16383 then exit;
  Result.b1 := (offset SHR 8) OR 192;
  Result.b2 := (offset AND $00FF);
end;

procedure DomainNameToLabels(const dmn: String; var labels: TStringList);
begin
  labels.Clear;
  labels.Delimiter := '.';
  labels.StrictDelimiter := True;
  labels.DelimitedText := dmn;
end;

procedure TNetDbTest.BuildFakeRR_A(out RR: TFakeRR; nm: String; ttl: Cardinal;
  val: String);
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_A;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.ip := StrToNetAddr(val);
  RR.RDLength := 4;
end;

procedure TNetDbTest.BuildFakeRR_AAAA(out RR: TFakeRR; nm: String;
  ttl: Cardinal; val: String);
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_AAAA;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.ip6 := StrToNetAddr6(val);
  RR.RDLength := 16;
end;

procedure TNetDbTest.BuildFakeRR_MX(out RR: TFakeRR; nm: String; ttl: Cardinal;
  pref: Word; exch: ShortString );
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_MX;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.fmx.pref := pref;
  RR.fmx.exch := exch;
end;

procedure TNetDbTest.BuildFakeRR_NS(out RR: TFakeRR; nm: String; ttl: Cardinal;
  val: String);
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_NS;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.nsh := val;
end;

procedure TNetDbTest.BuildFakeRR_PTR(out RR: TFakeRR; nm: String; ttl: Cardinal;
  val: String);
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_PTR;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.ptr := val;
end;

procedure TNetDbTest.BuildFakeRR_CNAME(out RR: TFakeRR; nm: String;
  ttl: Cardinal; val: String);
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_CNAME;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.cn := val;
end;

procedure TNetDbTest.BuildFakeRR_SOA(out RR: TFakeRR; nm: String; ttl: Cardinal;
  mn,rn: ShortString; serial,refresh,retry,expire,min: Cardinal);
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_SOA;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.fsoa.mn := mn;
  RR.fsoa.rn := rn;
  RR.fsoa.serial := serial;
  RR.fsoa.refresh := refresh;
  RR.fsoa.retry := retry;
  RR.fsoa.expire := expire;
  RR.fsoa.min := min;
end;

procedure TNetDbTest.BuildFakeRR_TXT(out RR: TFakeRR; nm: String; ttl: Cardinal;
  n: Byte; txt: TTextArray);
var
  idx: Byte;
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_TXT;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.sstrcount := n;
  RR.txtarr[1] := '';
  RR.txtarr[2] := '';
  RR.txtarr[3] := '';
  RR.txtarr[4] := '';
  RR.txtarr[5] := '';
  for idx := Low(txt) to Min(n, High(txt)) do
    RR.txtarr[idx] := txt[idx];
end;

procedure TNetDbTest.BuildFakeRR_SRV(out RR: TFakeRR; nm: String; ttl: Cardinal;
   priority, weight, port: Word; target: ShortString);
begin
  RR.RRName := nm;
  RR.Atype := DNSQRY_SRV;
  RR.AClass := 1;
  RR.TTL := ttl;
  RR.fsrv.priority := priority;
  RR.fsrv.weight := weight;
  RR.fsrv.port := port;
  RR.fsrv.target := target;
end;

function TNetDbTest.CalcRdLength(o: TTextArray): Word;
var
  tmps: ShortString;
begin
  Result := 0;
  for tmps in o do
  begin
    if tmps = '' then break;
    Result := Result + Length(tmps)+1; // don't forget length byte!
  end;
end;

function TNetDbTest.WriteAasRData(var buf: TBuffer; var offset: Cardinal;
  ip: THostAddr): TRDataWriteRes;
var
  s,l: Word;
begin
  s := offset;
  l := SizeOf(ip.s_addr);
  Result.etw := l + 2; //rdlength +2 for length itself
  // rdlength
  WriteNumToBuffer(buf, offset, l);
  // rr data
  WriteNumToBufferN(buf, offset, ip.s_addr);
  Result.bw := offset - s;
end;

function TNetDbTest.WriteSRVasRData(var buf: TBuffer; var offset: Cardinal;
  fsrv: TFakeSRV): TRDataWriteRes;
var
  s, l: Word;
  dmbs: TDNSDomainByteStream;
begin
  s := offset;
  dmbs := DomainNameToByteStream(fsrv.target);
  l := CalcRdLength(dmbs) + SizeOf(Word) * 3;
  Result.etw := l + 2; //rdlength +2 for length byte

  // rdlength
  WriteNumToBuffer(buf, offset, l);

  // RR data
  WriteNumToBuffer(buf, offset, fsrv.priority);
  WriteNumToBuffer(buf, offset, fsrv.weight);
  WriteNumToBuffer(buf, offset, fsrv.port);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  Result.bw := offset - s;
end;

function TNetDbTest.WriteSRVasRData(var buf: TBuffer; var offset: Cardinal;
  fsrv: TFakeSRV; var ctbl: TDomainCompressionTable): TRDataWriteRes;
var
  s, l: Word;
  dmbs: TDNSDomainByteStream;
begin
  s := offset;
  dmbs := DomainNameToByteStream(fsrv.target, ctbl);
  l := CalcRdLength(dmbs) + SizeOf(Word) * 3;
  Result.etw := l + 2; //rdlength +2 for length byte

  // rdlength
  WriteNumToBuffer(buf, offset, l);

  // RR data
  WriteNumToBuffer(buf, offset, fsrv.priority);
  WriteNumToBuffer(buf, offset, fsrv.weight);
  WriteNumToBuffer(buf, offset, fsrv.port);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  Result.bw := offset - s;
end;

function TNetDbTest.WriteMXAsRData(var buf: TBuffer; var offset: Cardinal;
  fmx: TFakeMX; var ctbl: TDomainCompressionTable): TRDataWriteRes;
var
  s, l: Word;
  dmbs: TDNSDomainByteStream;
begin
  s := offset;
  dmbs := DomainNameToByteStream(fmx.exch, ctbl);
  l := SizeOf(fmx.pref) + CalcRdLength(dmbs);
  Result.etw := l + 2; // we'll write rdlength bytes+2 bytes for length itself.

  // rdlength
  WriteNumToBuffer(buf, offset, l);

  // RR data
  // pref
  WriteNumToBuffer(buf, offset, fmx.pref);
  // exchange
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  Result.bw := offset - s;
end;

function TNetDbTest.CalcRdLength(o: TDNSDomainByteStream): Word;
begin
  Result := Length(o.ulabels);
  if o.cptr > 0 then Inc(Result,2);
end;

function TNetDbTest.WriteAAAAasRData(var buf: TBuffer; var offset: Cardinal;
  ip6: THostAddr6): TRDataWriteRes;
var
  s,l: Word;
begin
  s := offset;
  l := SizeOf(ip6.u6_addr32);
  Result.etw := l + 2; //rdlength + 2 for length itself
  // rdlength
  WriteNumToBuffer(buf, offset, l);
  // rr data
  Move(ip6.s6_addr, buf[offset], l);
  Inc(offset, l);
  Result.bw := offset - s;
end;

function TNetDbTest.WriteSOAasRData(var buf: TBuffer; var offset: Cardinal;
  fsoa: TFakeSOA): TRDataWriteRes;
var
  s, l: Word;
  dmbsmn, dmbsrn: TDNSDomainByteStream;
begin
  s := offset;
  dmbsmn := DomainNameToByteStream(fsoa.mn);
  dmbsrn := DomainNameToByteStream(fsoa.rn);
  l := CalcRdLength(dmbsmn) + CalcRdLength(dmbsrn) + (SizeOf(Cardinal) * 5);

  Result.etw := l + 2; // rdlength bytes + 2 for length itself

  // rdlength
  WriteNumToBuffer(buf, offset, l);

  // rr data
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbsmn);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbsrn);

  WriteNumToBuffer(buf, offset, fsoa.serial);
  WriteNumToBuffer(buf, offset, fsoa.refresh);
  WriteNumToBuffer(buf, offset, fsoa.retry);
  WriteNumToBuffer(buf, offset, fsoa.expire);
  WriteNumToBuffer(buf, offset, fsoa.min);

  Result.bw := offset - s;
end;

function TNetDbTest.WriteSOAasRData(var buf: TBuffer; var offset: Cardinal;
  fsoa: TFakeSOA; var ctbl: TDomainCompressionTable): TRDataWriteRes;
var
  s, l: Word;
  dmbsmn, dmbsrn: TDNSDomainByteStream;
begin
  s := offset;
  dmbsmn := DomainNameToByteStream(fsoa.mn, ctbl);
  dmbsrn := DomainNameToByteStream(fsoa.rn, ctbl);
  l := CalcRdLength(dmbsmn) + CalcRdLength(dmbsrn) + (SizeOf(Cardinal) * 5);
  Result.etw := l + 2; // rdlength bytes + 2 for length itself

  // rdlength
  WriteNumToBuffer(buf, offset, l);

  // rr data
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbsmn);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbsrn);

  WriteNumToBuffer(buf, offset, fsoa.serial);
  WriteNumToBuffer(buf, offset, fsoa.refresh);
  WriteNumToBuffer(buf, offset, fsoa.retry);
  WriteNumToBuffer(buf, offset, fsoa.expire);
  WriteNumToBuffer(buf, offset, fsoa.min);

  Result.bw := offset - s;
end;

function TNetDbTest.WriteMXAsRData(var buf: TBuffer; var offset: Cardinal;
  fmx: TFakeMX): TRDataWriteRes;
var
  s, l: Word;
  dmbs: TDNSDomainByteStream;
begin
  Result.bw := 0;
  s := offset;
  dmbs := DomainNameToByteStream(fmx.exch);
  l := SizeOf(fmx.pref) + CalcRdLength(dmbs);
  Result.etw := l + 2; // we'll write rdlength + 2 bytes for the length itself.

  // rdlength
  WriteNumToBuffer(buf, offset, l);

  // RR data
  // pref
  WriteNumToBuffer(buf, offset, fmx.pref);
  // exchange
  dmbs := DomainNameToByteStream(fmx.exch);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  Result.bw := offset - s;
end;

function TNetDbTest.WriteTextRecAsRData(var buf: TBuffer; var offset: Cardinal;
  tt: TTextArray): TRDataWriteRes;
var
  s, l: Word;
  ws: ShortString;
begin
  s := offset;
  l := CalcRdLength(tt);
  Result.etw := l + 2; // rdlength +2 for length itself
  // rdlength
  WriteNumToBuffer(buf, offset, l);

  for ws in tt do
  begin
    if  ws = '' then break;
    Move(ws, buf[offset], Length(ws)+1);
    Inc(offset,Length(ws)+1);
  end;

  Result.bw := offset - s;
end;

{
Convert a domain name into a byte stream. Compression is supported using the
supplied compression table.
}
function TNetDbTest.DomainNameToByteStream(nm: ShortString;
  var ctbl: TDomainCompressionTable): TDNSDomainByteStream;
var
  dmn: ShortString;
  offset,cmpoffset: Word;
  ptrseen: Boolean = False;
begin
  SetLength(Result.ulabels, 0);
  Result.cptr := 0;
  offset := 0;

  if nm = '' then exit;
  DomainNameToLabels(nm, tsl);
  if tsl.Count = 0 then exit;

  dmn := '';
  cmpoffset := 0;

  {
  for a domain a.b.c, using the lookup table,
   -> lookup (a.b.c), if not found, add to table,
   ->   lookup (b.c), if not found, add to table,
   ->   lookup   (c), if not found, add to table,

   buf if any label domain is found, add the pointer to the buffer and stop.
  }
  repeat
    dmn := tsl.DelimitedText;
    ptrseen := LookupStr(dmn, ctbl, cmpoffset);
    if ptrseen then
    begin
      // found the domain name. add a pointer, then we're done. Per RFC1035,
      // section 4.1.4, a domain name is either a series of labels, a pointer,
      // or a series of labels ending with a pointer. There's just one pointer
      // for a domain name.
      Result.cptr := cmpoffset;
      break;
    end
    else
    begin
      // add the last full domain we looked up, not the working label,
      // to the compression lookup table. E.g, add a.b.c rather than a.
      // Add 12 for the dns header, which our buffer doesn't include, but
      // api methods like stringfromlabel adjust offsets to account for it.
      if Length(dmn) > 0 then AddStr(dmn, ctbl, offset+12);
      // write the label to the buffer
      dmn := tsl[0];
      tsl.Delete(0);
      SetLength(Result.ulabels, (Length(Result.ulabels) + Length(dmn)+1));
      Result.ulabels[offset] := Length(dmn);
      Inc(offset);
      Move(dmn[1], Result.ulabels[offset], Length(dmn));
      Inc(offset, Length(dmn));
    end;
  until tsl.Count = 0;

  // if we didn't see a pointer then we have to write a 0. see rfc1035, s4.1.4.
  if not ptrseen then
  begin
    SetLength(Result.ulabels, Length(Result.ulabels) + 1);
    Result.ulabels[offset] := 0;
    Inc(offset);
  end;
end;

{
This version of DomainNameToByteStream doesn't compress.
}
function TNetDbTest.DomainNameToByteStream(nm: ShortString
  ): TDNSDomainByteStream;
var
  dmn: ShortString;
  offset: Word;
begin
  SetLength(Result.ulabels, 0);
  Result.cptr := 0;
  offset := 0;

  if nm = '' then exit;
  DomainNameToLabels(nm, tsl);
  if tsl.Count = 0 then exit;

  for dmn in tsl do
  begin
    SetLength(Result.ulabels, (Length(Result.ulabels) + Length(dmn)+1));
    Result.ulabels[offset] := Length(dmn);
    Inc(offset);
    Move(dmn[1], Result.ulabels[offset], Length(dmn));
    Inc(offset, Length(dmn));
  end;

  SetLength(Result.ulabels, Length(Result.ulabels) + 1);
  Result.ulabels[offset] := 0;
end;

function TNetDbTest.WriteDNSDomainByteStreamToBuffer(var buf: TBuffer;
  var offset: Cardinal; dbs: TDNSDomainByteStream): Word;
var
  p: TDNSDomainPointer;
  so: Word;
begin
  Result := 0;
  // no label, no pointer, no write for you.
  if (Length(dbs.ulabels) = 0) and (dbs.cptr = 0) then exit;
  if (offset + CalcRdLength(dbs)) > Length(buf) then exit;

  so := offset;
  // labels can be empty, in which case we're writing just a pointer.
  if Length(dbs.ulabels) > 0 then
  begin
    Move(dbs.ulabels[0], buf[offset], Length(dbs.ulabels));
    Inc(offset, Length(dbs.ulabels));
  end;
  if dbs.cptr > 0 then
  begin
    p := GetDnsDomainPointer(dbs.cptr);
    Move(p.ba, buf[offset], Length(p.ba));
    Inc(offset, Min(Length(p.ba), (Length(buf) - offset)));
  end;
  Result := offset - so;
end;

{
Write a domain name as RDATA. This means an RDLength (Word) and the
domain labels.
}
function TNetDbTest.WriteDomainAsRdata(var buf: TBuffer; var offset: Cardinal;
  dbs: TDNSDomainByteStream): TRDataWriteRes;
var
  s,l: Word;
begin
  l := CalcRdLength(dbs);
  Result.etw := l + 2;
  s := offset;
  WriteNumToBuffer(buf, offset,l);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dbs);
  Result.bw := offset - s;
end;


procedure TNetDbTest.BuildFakeResponseA(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 2;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_A;

  // now the answer RRs
  SetLength(fr.answers,2);
  BuildFakeRR_A(fr.answers[0], nm, 300, '127.0.0.1');
  BuildFakeRR_A(fr.answers[1], nm, 215, '127.0.5.1');
end;

procedure TNetDbTest.BuildFakeResponseAAAA(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 2;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_AAAA;

  // now the answer RRs
  SetLength(fr.answers,2);
  BuildFakeRR_AAAA(fr.answers[0], nm, 300, 'fe80::3b92:3429:ff16:a3e4');
  BuildFakeRR_AAAA(fr.answers[1], nm, 215, 'fe80::92e6:baff:fe44:ffbb');
end;

procedure TNetDbTest.BuildFakeResponseMX(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 1;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 2;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_MX;

  // now the answer RRs
  SetLength(fr.answers,1);
  BuildFakeRR_MX(fr.answers[0], nm, 0, 10, 'mailer.'+FAKEFQDN);
  // now an additional rr with the A record for the above.
  SetLength(fr.additional, 2);
  BuildFakeRR_A(fr.additional[0], 'mailer.'+FAKEFQDN, 0,
    '172.16.27.238');
  BuildFakeRR_AAAA(fr.additional[1], 'mailer.'+FAKEFQDN, 0,
    'fe80::3b92:3429:ff16:a3e4');
end;

procedure TNetDbTest.BuildFakeResponseSOA(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 1;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_SOA;
  // now the answer RRs
  SetLength(fr.answers,1);
  BuildFakeRR_SOA(fr.answers[0],FAKEFQDN,33,
    'mn.'+FAKEFQDN,'rn.'+FAKEFQDN,76543210,
      123,456,789,60);
end;

procedure TNetDbTest.BuildFakeResponseCNAME(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 1;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_CNAME;

  // now the answer RRs
  SetLength(fr.answers,1);
  BuildFakeRR_CNAME(fr.answers[0], nm, 300, 'fakecname.'+FAKEFQDN);
end;

procedure TNetDbTest.BuildFakeResponseNS(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 1;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_NS;

  // now the answer RRs
  SetLength(fr.answers,1);
  BuildFakeRR_NS(fr.answers[0], nm, 300, 'fakens.'+FAKEFQDN);
end;

procedure TNetDbTest.BuildFakeResponsePTR(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 1;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_PTR;

  // now the answer RRs
  SetLength(fr.answers,1);
  BuildFakeRR_PTR(fr.answers[0], nm, 300, 'fakeptrans.'+FAKEFQDN);
end;

procedure TNetDbTest.BuildFakeResponseTXT(nm: ShortString; out
  fr: TFakeDNSResponse);
var
  txtarr: TTextArray;
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 1;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_TXT;

  txtarr[1] := 'v=spf1 mx a:lists.'+FAKEFQDN;
  txtarr[2] := 'Always look on the bright side of life!';
  // now the answer RRs
  SetLength(fr.answers,1);
  BuildFakeRR_TXT(fr.answers[0], nm, 300, 2, txtarr);
end;

procedure TNetDbTest.BuildFakeResponseSRV(nm: ShortString; out
  fr: TFakeDNSResponse);
begin
  // metadata
  SetLength(fr.strtable, 0);

  // start by building a fake header.
  fr.hdr.ID[0] := 12;
  fr.hdr.ID[1] := 34;
  fr.hdr.flags1 := QF_QR or QF_RD;
  fr.hdr.flags2 := 0;
  fr.hdr.qdcount := 1;
  fr.hdr.ancount := 1;
  fr.hdr.nscount := 0;
  fr.hdr.arcount := 0;

  // Next is the query part
  fr.qry.nm := nm;
  fr.qry.qclass := 1;
  fr.qry.qtype := DNSQRY_SRV;
  // now the answer RRs
  SetLength(fr.answers,1);
  BuildFakeRR_SRV(fr.answers[0],FAKEFQDN,3300,22,44,2201,'_this._that._other');
end;

{
Test that BuildPayload puts the right values into the payload buffer.
}
procedure TNetDbTest.TestBuildPayloadSimple;
var
  Q: TQueryData;
  R, I,J,el: Integer;
  S: String;
begin
  R := BuildPayLoad(Q, FAKEFQDN, DNSQRY_A, 1);
  // this is the expected length. Essentially, for each label, len(label)+1,
  // then 4 bytes for the qclass and qtype, and 1 more for a 0 byte.
  // rather than hardwire the length we calculate it so that no matter
  // what the fake domain the test passes.
  el := (Length(FAKEDOMAIN)+1)+(Length(FAKETLD)+1)+5;
  AssertEquals('Payload byte count wrong:', el, R);
  I := 0;
  J := 0;
  S := stringfromlabel(Q.Payload,I);
  AssertEquals('Wrong domain name returned:',FAKEFQDN, S);
  Move(Q.Payload[I],J,SizeOf(Word));
  AssertEquals('Wrong query type', DNSQRY_A, NToHs(J));
  Inc(I,2);
  Move(Q.Payload[I],J,SizeOf(Word));
  AssertEquals('Wrong class', 1, NToHs(J));
end;

{
Test building a payload with an empty str.
}
procedure TNetDbTest.TestBuildPayloadSimpleEmpty;
var
  Q: TQueryData;
  R: Integer;
begin
  R := BuildPayLoad(Q, '', DNSQRY_A, 1);
  AssertEquals('Payload byte count wrong:',-1, R);
end;

{
Test BuildQuery with a label that ends in a dot. This should be allowed.
A dot at the end is an empty label but we must not count its 0 byte twice.
}
procedure TNetDbTest.TestBuildPayloadSimpleEndDot;
var
  Q: TQueryData;
  R,el: Integer;
begin
  // this is the expected length. Essentially, for each label, len(label)+1,
  // then 4 bytes for the qclass and qtype, and 1 more for a 0 byte.
  // rather than hardwire the length we calculate it so that no matter
  // what the fake domain the test passes.
  el := (Length(FAKEDOMAIN)+1)+(Length(FAKETLD)+1)+5;
  R := BuildPayLoad(Q, FAKEFQDN+'.', DNSQRY_A, 1);
  AssertEquals('Payload byte count wrong:',el, R);
end;

{
Test BuildPayload with a label that starts with a dot. This should be
rejected outright.
}
procedure TNetDbTest.TestBuildPayloadSimpleStartDot;
var
  Q: TQueryData;
  R: Integer;
begin
  R := BuildPayLoad(Q, '.'+FAKEFQDN, DNSQRY_A, 1);
  AssertEquals('Payload byte count wrong:',-1, R);
end;

{
Test BuildPayload with multiple dots (empty labels) in the middle of the domain
name. This should be rejected outright.
}
procedure TNetDbTest.TestBuildPayloadSimpleMultipleDot;
var
  Q: TQueryData;
  R: Integer;
begin
  R := BuildPayLoad(Q, FAKEDOMAIN+'.....'+FAKETLD, DNSQRY_A, 1);
  AssertEquals('Payload byte count wrong:',-1, R);
end;

procedure TNetDbTest.TestDnsQueryUDP_A;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an A RR.', DNSQRY_A, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong A record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong A record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.0.1', HostAddrToStr(ip));
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[1], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.5.1', HostAddrToStr(ip));
end;

procedure TNetDbTest.TestDnsQueryTCP_A;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an A RR.', DNSQRY_A, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong A record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong A record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.0.1', HostAddrToStr(ip));
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[1], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.5.1', HostAddrToStr(ip));
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_A;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an A RR.', DNSQRY_A, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong A record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong A record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.0.1', HostAddrToStr(ip));
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[1], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.5.1', HostAddrToStr(ip));
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_A;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an A RR.', DNSQRY_A, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong A record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong A record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.0.1', HostAddrToStr(ip));
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[1], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.5.1', HostAddrToStr(ip));
end;

procedure TNetDbTest.TestDnsQueryUDP_AAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an AAAA RR.',DNSQRY_AAAA, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong AAAA record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong AAAA record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[0], qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4', HostAddrToStr6(ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1], qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::92E6:BAFF:FE44:FFBB', HostAddrToStr6(ip));
end;

procedure TNetDbTest.TestDnsQueryTCP_AAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an AAAA RR.',DNSQRY_AAAA, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong AAAA record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong AAAA record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[0],
    qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4',
    HostAddrToStr6(ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1],
    qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::92E6:BAFF:FE44:FFBB',
    HostAddrToStr6(ip));
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_AAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an AAAA RR.',DNSQRY_AAAA, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong AAAA record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong AAAA record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[0],
    qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4',
    HostAddrToStr6(ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1],
    qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::92E6:BAFF:FE44:FFBB',
    HostAddrToStr6(ip));
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_AAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an AAAA RR.',DNSQRY_AAAA, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertEquals('Wrong AAAA record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertEquals('Wrong AAAA record name for RR 1', FAKEFQDN,
    RRArr[1].RRName);
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[0],
    qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4',
    HostAddrToStr6(ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1],
    qd.Payload, ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::92E6:BAFF:FE44:FFBB',
    HostAddrToStr6(ip));
end;

procedure TNetDbTest.TestDnsQueryUDP_MX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  mxrec: TDNSRR_MX;
  ip: THostAddr;
  ip6: THostAddr6;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an MX RR.',DNSQRY_MX, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, mxrec));
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEFQDN,
    mxrec.exchange);
  AssertEquals('Wrong MX preference', 10, mxrec.preference);

  AssertEquals('Should be 2 additional RR records.',2,NToHs(qd.h.arcount));
  RRArr := GetRRrecords(qd.Payload, ansstart, NToHs(qd.h.arcount));
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1], qd.Payload, ip6));

  AssertEquals('Wrong ip for A.', '172.16.27.238', HostAddrToStr(ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4',
    HostAddrToStr6(ip6));
end;

procedure TNetDbTest.TestDnsQueryTCP_MX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  mxrec: TDNSRR_MX;
  ip: THostAddr;
  ip6: THostAddr6;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an MX RR.',DNSQRY_MX, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, mxrec));
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEFQDN,
    mxrec.exchange);
  AssertEquals('Wrong MX preference', 10, mxrec.preference);

  AssertEquals('Should be 2 additional RR records.',2,NToHs(qd.h.arcount));
  RRArr := GetRRrecords(qd.Payload, ansstart, NToHs(qd.h.arcount));
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1], qd.Payload, ip6));

  AssertEquals('Wrong ip for A.', '172.16.27.238', HostAddrToStr(ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4',
    HostAddrToStr6(ip6));
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_MX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  mxrec: TDNSRR_MX;
  ip: THostAddr;
  ip6: THostAddr6;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an MX RR.',DNSQRY_MX, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, mxrec));
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEFQDN,
    mxrec.exchange);
  AssertEquals('Wrong MX preference', 10, mxrec.preference);

  AssertEquals('Should be 2 additional RR records.',2,NToHs(qd.h.arcount));
  RRArr := GetRRrecords(qd.Payload, ansstart, NToHs(qd.h.arcount));
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1], qd.Payload,
    ip6));

  AssertEquals('Wrong ip for A.', '172.16.27.238', HostAddrToStr(ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4',
    HostAddrToStr6(ip6));
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_MX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  mxrec: TDNSRR_MX;
  ip: THostAddr;
  ip6: THostAddr6;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an MX RR.',DNSQRY_MX, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, mxrec));
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEFQDN,
    mxrec.exchange);
  AssertEquals('Wrong MX preference', 10, mxrec.preference);

  AssertEquals('Should be 2 additional RR records.',2,NToHs(qd.h.arcount));
  RRArr := GetRRrecords(qd.Payload, ansstart, NToHs(qd.h.arcount));
  AssertEquals('Wrong number of resource records.', 2, Length(RRArr));
  AssertEquals('RR 0 is not an A RR.',DNSQRY_A, RRarr[0].RRMeta.Atype);
  AssertEquals('RR 1 is not an AAAA RR.', DNSQRY_AAAA, RRarr[1].RRMeta.Atype);
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertTrue('Did not get RR AAAA data.', DNSRRGetAAAA(RRArr[1], qd.Payload,
    ip6));

  AssertEquals('Wrong ip for A.', '172.16.27.238', HostAddrToStr(ip));
  AssertEquals('Wrong ip for AAAA.', 'FE80::3B92:3429:FF16:A3E4',
    HostAddrToStr6(ip6));
end;

procedure TNetDbTest.TestDnsQueryUDP_SOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
  AssertEquals('Wrong mname hostname', 'mn.'+FAKEFQDN,
    soarec.mname);
  AssertEquals('Wrong rname hostname', 'rn.'+FAKEFQDN,
    soarec.rname);
  AssertEquals('Wrong SOA serial', 76543210, soarec.serial);
  AssertEquals('Wrong SOA refresh', 123, soarec.refresh);
  AssertEquals('Wrong SOA retry', 456, soarec.retry);
  AssertEquals('Wrong SOA expire', 789, soarec.expire);
  AssertEquals('Wrong SOA min', 60, soarec.min);
end;

procedure TNetDbTest.TestDnsQueryTCP_SOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
  AssertEquals('Wrong mname hostname', 'mn.'+FAKEFQDN,
    soarec.mname);
  AssertEquals('Wrong rname hostname', 'rn.'+FAKEFQDN,
    soarec.rname);
  AssertEquals('Wrong SOA serial', 76543210, soarec.serial);
  AssertEquals('Wrong SOA refresh', 123, soarec.refresh);
  AssertEquals('Wrong SOA retry', 456, soarec.retry);
  AssertEquals('Wrong SOA expire', 789, soarec.expire);
  AssertEquals('Wrong SOA min', 60, soarec.min);
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_SOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
  AssertEquals('Wrong mname hostname', 'mn.'+FAKEFQDN,
    soarec.mname);
  AssertEquals('Wrong rname hostname', 'rn.'+FAKEFQDN,
    soarec.rname);
  AssertEquals('Wrong SOA serial', 76543210, soarec.serial);
  AssertEquals('Wrong SOA refresh', 123, soarec.refresh);
  AssertEquals('Wrong SOA retry', 456, soarec.retry);
  AssertEquals('Wrong SOA expire', 789, soarec.expire);
  AssertEquals('Wrong SOA min', 60, soarec.min);
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_SOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
  AssertEquals('Wrong mname hostname', 'mn.'+FAKEFQDN,
    soarec.mname);
  AssertEquals('Wrong rname hostname', 'rn.'+FAKEFQDN,
    soarec.rname);
  AssertEquals('Wrong SOA serial', 76543210, soarec.serial);
  AssertEquals('Wrong SOA refresh', 123, soarec.refresh);
  AssertEquals('Wrong SOA retry', 456, soarec.retry);
  AssertEquals('Wrong SOA expire', 789, soarec.expire);
  AssertEquals('Wrong SOA min', 60, soarec.min);
end;

procedure TNetDbTest.TestDnsQueryUDP_CNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong CNAME record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryTCP_CNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong CNAME record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_CNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong CNAME record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_CNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong CNAME record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload,
    s));
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryUDP_NS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong NS record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong NS.', 'fakens.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryTCP_NS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong NS record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong NS.', 'fakens.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_NS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong NS record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong NS.', 'fakens.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_NS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong NS record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong NS.', 'fakens.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryUDP_PTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);
  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryTCP_PTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);
  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_PTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);
  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_PTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);
  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsQueryUDP_TXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR TXT data.', DNSRRGetText(RRArr[0], qd.Payload, s));
  AssertEquals(
    'v=spf1 mx a:lists.'+FAKEFQDN+'Always look on the bright side of life!',
      s);
end;

procedure TNetDbTest.TestDnsQueryTCP_TXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR TXT data.', DNSRRGetText(RRArr[0], qd.Payload, s));
  AssertEquals(
    'v=spf1 mx a:lists.'+FAKEFQDN+'Always look on the bright side of life!',
      s);
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_TXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR TXT data.', DNSRRGetText(RRArr[0], qd.Payload, s));
  AssertEquals(
    'v=spf1 mx a:lists.'+FAKEFQDN+'Always look on the bright side of life!',
      s);
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_TXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);
  AssertTrue('Did not get RR TXT data.', DNSRRGetText(RRArr[0], qd.Payload, s));
  AssertEquals(
    'v=spf1 mx a:lists.'+FAKEFQDN+'Always look on the bright side of life!',
      s);
end;

procedure TNetDbTest.TestDnsQueryUDP_SRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SRV data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
  AssertEquals('Wrong SRV priority', 22, srvrec.priority);
  AssertEquals('Wrong SRV weight', 44, srvrec.weight);
  AssertEquals('Wrong SRV port', 2201, srvrec.port);

  AssertEquals('Wrong SRV hostname', '_this._that._other', srvrec.target);
end;

procedure TNetDbTest.TestDnsQueryTCP_SRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
  AssertEquals('Wrong SRV priority', 22, srvrec.priority);
  AssertEquals('Wrong SRV weight', 44, srvrec.weight);
  AssertEquals('Wrong SRV port', 2201, srvrec.port);

  AssertEquals('Wrong SRV hostname', '_this._that._other', srvrec.target);
end;

procedure TNetDbTest.TestDnsQueryCompressUDP_SRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SRV data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
  AssertEquals('Wrong SRV priority', 22, srvrec.priority);
  AssertEquals('Wrong SRV weight', 44, srvrec.weight);
  AssertEquals('Wrong SRV port', 2201, srvrec.port);

  AssertEquals('Wrong SRV hostname', '_this._that._other', srvrec.target);
end;

procedure TNetDbTest.TestDnsQueryCompressTCP_SRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to compressed querydata',
    BuildQueryData(fakeresp, qd, anslen, True));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  AssertEquals('Wrong record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
  AssertEquals('Wrong SRV priority', 22, srvrec.priority);
  AssertEquals('Wrong SRV weight', 44, srvrec.weight);
  AssertEquals('Wrong SRV port', 2201, srvrec.port);

  AssertEquals('Wrong SRV hostname', '_this._that._other', srvrec.target);
end;

{
This test is of debatable value, as it only detects truncation if the buffer
contents are zeroed which gives an invalid RR type.
}
procedure TNetDbTest.TestDnsQueryTruncateRR_UDP_A;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildTruncatedQueryData(fakeresp, qd, anslen,40));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  // the header says there are 2 A records, but it's a trap!
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  // truncation of buffer means this call returns 0 RRs.
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of RRs', 0, Length(RRArr));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // Change start position for RR[0] to end of buffer - 4
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  qd.Payload[Length(qd.Payload)-1] := $AA; // sentinel marker we can look for

  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '0.0.0.170', HostAddrToStr(ip));
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // Change start position for RR[0] to end of buffer - 3
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 3);
  AssertFalse('Got RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
end;

{
Test that we read the AAAA right at the buffer edge, with the last byte
being a special value we can test for.
}
procedure TNetDbTest.TestDnsRRBufferEdgeAAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  // Change start position for RR[0]
  RRArr[0].RDataSt := Length(qd.Payload) - SizeOf(THostAddr6);
  qd.Payload[Length(qd.Payload)-1] := $AA;
  AssertTrue('Got RR AAAA data.', DNSRRGetAAAA(RRArr[0], qd.Payload, ip));
  AssertEquals($AA, ip.u6_addr8[15]);
end;

{
Attempt to read an AAAA that goes past the end of the buffer.
}
procedure TNetDbTest.TestDNsRRBufferPastEdgeAAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  // Change start position for RR[0]. attempting to read 16 bytes
  // from this position will pass the end of the buffer.
  RRArr[0].RDataSt := Length(qd.Payload) - (SizeOf(THostAddr6)-1);
  qd.Payload[Length(qd.Payload)-1] := $AA;
  AssertFalse('Got RR AAAA data.', DNSRRGetAAAA(RRArr[0], qd.Payload, ip));
end;

{
Test reading an MX RR that terminates on the last byte of the buffer.
}
procedure TNetDbTest.TestDnsRRBufferEdgeMX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  fmx: TDNSRR_MX;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // move the MX RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Got RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, fmx));
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEFQDN,
    fmx.exchange);
  AssertEquals('Wrong MX preference', 10, fmx.preference);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeMX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  fmx: TDNSRR_MX;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // move the MX RR bytes to the end of the payload buffer. We omit the last
  // 2 bytes of the MX to attempt to trick the code into reading past the buffer
  // edge.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  AssertTrue('Got RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, fmx));
  // stringfromlabel should drop the last label, so the result should be just
  // missing the tld.
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEDOMAIN,
    fmx.exchange);
  AssertEquals('Wrong MX preference', 10, fmx.preference);
end;

procedure TNetDbTest.TestDnsRRBufferEdgeSOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);

  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  // move the SOA RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
  AssertEquals('Wrong mname hostname', 'mn.'+FAKEFQDN,
    soarec.mname);
  AssertEquals('Wrong rname hostname', 'rn.'+FAKEFQDN,
    soarec.rname);
  AssertEquals('Wrong SOA serial', 76543210, soarec.serial);
  AssertEquals('Wrong SOA refresh', 123, soarec.refresh);
  AssertEquals('Wrong SOA retry', 456, soarec.retry);
  AssertEquals('Wrong SOA expire', 789, soarec.expire);
  AssertEquals('Wrong SOA min', 60, soarec.min);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeSOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);

  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  // move the SOA RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-1);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-1));

  AssertFalse('Got RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeSRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  // move the SRV RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR SRV data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
  AssertEquals('Wrong SRV priority', 22, srvrec.priority);
  AssertEquals('Wrong SRV weight', 44, srvrec.weight);
  AssertEquals('Wrong SRV port', 2201, srvrec.port);

  AssertEquals('Wrong SRV hostname', '_this._that._other', srvrec.target);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeSRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  // move the SRV RR bytes to the end of the payload buffer. ensure that
  // we're one byte short to try and trick the code into reading past the
  // end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 1);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength - 1));

  AssertFalse('Got RR SRV data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeCNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);

  // move the cname to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEFQDN, s);
end;

{
Test retrieving a cname when the actual string is longer than rdlength says it
is. The bytes in the payload buffer try to point past the end of the buffer.
}
procedure TNetDbTest.TestDnsRRBufferPastEdgeCNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);

  // move the cname to the end of the buffer. we drop two bytes off the end of
  // the cname, because there's a 0 byte at the end of a label if not a ptr.
  // now, the last label's size is greater than the number of bytes left in
  // the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  // lie about the rdlength too!
  Dec(RRArr[0].RRMeta.RDLength,2);
  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload, s));
  // last label will get removed, leaving just the domain part.
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEDOMAIN, s);
end;

{
Test retrieving an NS RR when it's at the end of the payload buffer.
}
procedure TNetDbTest.TestDnsRRBufferEdgeNS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong NS record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  // move the ns to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong NS.', 'fakens.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeNS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);

  // move the ns to the end of the buffer. we drop two bytes off the end of
  // the ns, because there's a 0 byte at the end of a label if not a ptr.
  // now, the last label's size is greater than the number of bytes left in
  // the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  // lie about the rdlength too!
  Dec(RRArr[0].RRMeta.RDLength,2);
  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  // last label will get removed, leaving just the domain part.
  AssertEquals('Wrong NS.', 'fakens.'+FAKEDOMAIN, s);
end;

procedure TNetDbTest.TestDnsRRBufferEdgePTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);

  // move the ptr to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgePTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);

  // move the ns to the end of the buffer. we drop two bytes off the end of
  // the ns, because there's a 0 byte at the end of a label if not a ptr.
  // now, the last label's size is greater than the number of bytes left in
  // the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  // lie about the rdlength too!
  Dec(RRArr[0].RRMeta.RDLength,2);
  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  // last label will get removed, leaving just the domain part.
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEDOMAIN, s);
end;

{
Test reading a text record right at the edge of the payload buffer.
}
procedure TNetDbTest.TestDnsRRBufferEdgeTXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart,oldstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  // Move the text record to the end of the buffer
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);
  AssertTrue('Did not get RR TXT data.', DNSRRGetText(RRArr[0], qd.Payload, s));
  AssertEquals(
    'v=spf1 mx a:lists.'+FAKEFQDN+'Always look on the bright side of life!',
      s);
end;

{
Try reading a TXT record that points past the end of the payload buffer.
}
procedure TNetDbTest.TestDnsRRBufferPastEdgeTXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart,oldstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  // Move the text record to the end of the buffer, cutting off the last
  // 2 bytes. this means the length byte for the second string will point
  // past the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength - 2));
  AssertFalse('Did not get RR TXT data.',
    DNSRRGetText(RRArr[0], qd.Payload, s));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // Change start position for RR[0] to end of buffer - 4
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  qd.Payload[Length(qd.Payload)-1] := $AA; // sentinel marker we can look for
  AssertTrue('Did not get RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '0.0.0.170', HostAddrToStr(ip));
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // Change start position for RR[0] to end of buffer - 3
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 2);
  AssertFalse('Got RR A data.', DNSRRGetA(RRArr[0], qd.Payload, ip));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPAAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  // Change start position for RR[0]
  RRArr[0].RDataSt := Length(qd.Payload) - SizeOf(THostAddr6);
  qd.Payload[Length(qd.Payload)-1] := $AA;
  AssertTrue('Got RR AAAA data.', DNSRRGetAAAA(RRArr[0], qd.Payload, ip));
  AssertEquals($AA, ip.u6_addr8[15]);
end;

procedure TNetDbTest.TestDNsRRBufferPastEdgeTCPAAAA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
  ip: THostAddr6;
begin
  BuildFakeResponseAAAA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of AAAA records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  // Change start position for RR[0]. attempting to read 16 bytes
  // from this position will pass the end of the buffer.
  RRArr[0].RDataSt := Length(qd.Payload) - (SizeOf(THostAddr6)-1);
  AssertFalse('Got RR AAAA data.', DNSRRGetAAAA(RRArr[0], qd.Payload, ip));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPMX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  fmx: TDNSRR_MX;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // move the MX RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Got RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, fmx));
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEFQDN,
    fmx.exchange);
  AssertEquals('Wrong MX preference', 10, fmx.preference);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPMX;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  fmx: TDNSRR_MX;
begin
  BuildFakeResponseMX(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of MX records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);

  // move the MX RR bytes to the end of the payload buffer. We omit the last
  // 2 bytes of the MX to attempt to trick the code into reading past the buffer
  // edge.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  AssertTrue('Got RR MX data.', DNSRRGetMX(RRArr[0], qd.Payload, fmx));
  // stringfromlabel should drop the last label, so the result should be just
  // missing the tld.
  AssertEquals('Wrong MX hostname', 'mailer.'+FAKEDOMAIN,
    fmx.exchange);
  AssertEquals('Wrong MX preference', 10, fmx.preference);
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPSOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);

  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  // move the SOA RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
  AssertEquals('Wrong mname hostname', 'mn.'+FAKEFQDN,
    soarec.mname);
  AssertEquals('Wrong rname hostname', 'rn.'+FAKEFQDN,
    soarec.rname);
  AssertEquals('Wrong SOA serial', 76543210, soarec.serial);
  AssertEquals('Wrong SOA refresh', 123, soarec.refresh);
  AssertEquals('Wrong SOA retry', 456, soarec.retry);
  AssertEquals('Wrong SOA expire', 789, soarec.expire);
  AssertEquals('Wrong SOA min', 60, soarec.min);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPSOA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  soarec: TDNSRR_SOA;
begin
  BuildFakeResponseSOA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SOA records.', 1, qd.h.ancount);

  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SOA RR.',DNSQRY_SOA, RRarr[0].RRMeta.Atype);

  // move the SOA RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-1);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-1));

  AssertFalse('Got RR SOA data.', DNSRRGetSOA(RRArr[0], qd.Payload,
    soarec));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPSRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  // move the SRV RR bytes to the end of the payload buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR SRV data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
  AssertEquals('Wrong SRV priority', 22, srvrec.priority);
  AssertEquals('Wrong SRV weight', 44, srvrec.weight);
  AssertEquals('Wrong SRV port', 2201, srvrec.port);

  AssertEquals('Wrong SRV hostname', '_this._that._other', srvrec.target);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPSRV;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  srvrec: TDNSRR_SRV;
begin
  BuildFakeResponseSRV(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of SRV records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an SRV RR.',DNSQRY_SRV, RRarr[0].RRMeta.Atype);

  // move the SRV RR bytes to the end of the payload buffer. ensure that
  // we're one byte short to try and trick the code into reading past the
  // end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 1);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength - 1));

  AssertFalse('Got RR SRV data.', DNSRRGetSRV(RRArr[0], qd.Payload,
    srvrec));
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPCNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);

  // move the cname to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPCNAME;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseCNAME(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of CNAME records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an CNAME RR.',DNSQRY_CNAME, RRarr[0].RRMeta.Atype);

  // move the cname to the end of the buffer. we drop two bytes off the end of
  // the cname, because there's a 0 byte at the end of a label if not a ptr.
  // now, the last label's size is greater than the number of bytes left in
  // the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  // lie about the rdlength too!
  Dec(RRArr[0].RRMeta.RDLength,2);
  AssertTrue('Did not get RR CNAME data.', DNSRRGetCNAME(RRArr[0], qd.Payload, s));
  // last label will get removed, leaving just the domain part.
  AssertEquals('Wrong CNAME.', 'fakecname.'+FAKEDOMAIN, s);
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPNS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong NS record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  // move the ns to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong NS.', 'fakens.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPNS;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);

  // move the ns to the end of the buffer. we drop two bytes off the end of
  // the ns, because there's a 0 byte at the end of a label if not a ptr.
  // now, the last label's size is greater than the number of bytes left in
  // the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  // lie about the rdlength too!
  Dec(RRArr[0].RRMeta.RDLength,2);
  AssertTrue('Did not get RR NS data.', DNSRRGetNS(RRArr[0], qd.Payload, s));
  // last label will get removed, leaving just the domain part.
  AssertEquals('Wrong NS.', 'fakens.'+FAKEDOMAIN, s);
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPPTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);

  // move the ptr to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEFQDN, s);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPPTR;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
begin
  // the str passed in to this function doesn't really matter, but using
  // a proper in-addr.arpa domain helps keep it clear what we're testing.
  BuildFakeResponsePTR('0.5.0.127.in-addr.arpa', fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of PTR records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an PTR RR.',DNSQRY_PTR, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong PTR record name for RR 0', '0.5.0.127.in-addr.arpa',
    RRArr[0].RRName);

  // move the ns to the end of the buffer. we drop two bytes off the end of
  // the ns, because there's a 0 byte at the end of a label if not a ptr.
  // now, the last label's size is greater than the number of bytes left in
  // the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength-2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength-2));

  // lie about the rdlength too!
  Dec(RRArr[0].RRMeta.RDLength,2);
  AssertTrue('Did not get RR PTR data.', DNSRRGetPTR(RRArr[0], qd.Payload, s));
  // last label will get removed, leaving just the domain part.
  AssertEquals('Wrong PTR.', 'fakeptrans.'+FAKEDOMAIN, s);
end;

procedure TNetDbTest.TestDnsRRBufferEdgeTCPTXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart,oldstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  // Move the text record to the end of the buffer
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);
  AssertTrue('Did not get RR TXT data.', DNSRRGetText(RRArr[0], qd.Payload, s));
  AssertEquals(
    'v=spf1 mx a:lists.'+FAKEFQDN+'Always look on the bright side of life!',
      s);
end;

procedure TNetDbTest.TestDnsRRBufferPastEdgeTCPTXT;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart,oldstart: Word;
  RRArr: TRRNameDataArray;
  s: AnsiString;
begin
  s := '';
  BuildFakeResponseTXT(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of TXT records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not a TXT RR.',DNSQRY_TXT, RRarr[0].RRMeta.Atype);
  AssertEquals('Wrong TXT record name for RR 0', FAKEFQDN,
    RRArr[0].RRName);

  // Move the text record to the end of the buffer, cutting off the last
  // 2 bytes. this means the length byte for the second string will point
  // past the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - (RRArr[0].RRMeta.RDLength - 2);
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt,
    (RRArr[0].RRMeta.RDLength - 2));
  AssertFalse('Did not get RR TXT data.',
    DNSRRGetText(RRArr[0], qd.Payload, s));
end;

{
Test that NextNameRR correctly reads an RR on the edge of the buffer.
}
procedure TNetDbTest.TestNextNameRREdgeA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  rrn: TRRNameData;
  ip: THostAddr;
  t: Cardinal;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);

  // get an RR from its normal position. need this to calculate the length.
  AssertTrue('NextNameRR should succeed.',
    NextNameRR(qd.Payload, ansstart,  rrn));

  // calculate the size in bytes of the rr so we can copy it to the end
  // of the payload buffer
  t := (rrn.RDataSt + rrn.RRMeta.RDLength) - ansstart;
  CopyBytesTo(qd.Payload,ansstart,Length(qd.Payload)-t, t);
  AssertTrue('NextNameRR should succeed.',
    NextNameRR(qd.Payload, Length(qd.Payload)-t,  rrn));
  AssertEquals(DNSQRY_A, rrn.RRMeta.Atype);
  AssertEquals(300, rrn.RRMeta.TTL);
  AssertTrue(DNSRRGetA(rrn, qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.0.1', HostAddrToStr(ip));
end;

{
Try to trick NextNameRR into reading past the end of the payload buffer.
}
procedure TNetDbTest.TestNextNameRRPastEdgeA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  rrn: TRRNameData;
  t: Cardinal;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);

  // get an RR from its normal position. need this to calculate the length.
  AssertTrue('NextNameRR should succeed.',
    NextNameRR(qd.Payload, ansstart,  rrn));

  // calculate the size in bytes of the rr so we can copy it to the end
  // of the payload buffer
  t := (rrn.RDataSt + rrn.RRMeta.RDLength) - ansstart;
  // copy the bytes, but leave off the last one. leave the rdlength unchanged.
  CopyBytesTo(qd.Payload,ansstart,Length(qd.Payload)-(t-1), t-1);
  AssertFalse('NextNameRR should fail.',
    NextNameRR(qd.Payload, Length(qd.Payload)-(t-1),  rrn));
end;

procedure TNetDbTest.TestNextNameRREdgeTCPA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  rrn: TRRNameData;
  ip: THostAddr;
  t: Cardinal;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);

  // get an RR from its normal position. need this to calculate the length.
  AssertTrue('NextNameRR should succeed.',
    NextNameRR(qd.Payload, ansstart,  rrn));

  // calculate the size in bytes of the rr so we can copy it to the end
  // of the payload buffer
  t := (rrn.RDataSt + rrn.RRMeta.RDLength) - ansstart;
  CopyBytesTo(qd.Payload,ansstart,Length(qd.Payload)-t, t);
  AssertTrue('NextNameRR should succeed.',
    NextNameRR(qd.Payload, Length(qd.Payload)-t,  rrn));
  AssertEquals(DNSQRY_A, rrn.RRMeta.Atype);
  AssertEquals(300, rrn.RRMeta.TTL);
  AssertTrue(DNSRRGetA(rrn, qd.Payload, ip));
  AssertEquals('Wrong ip for A.', '127.0.0.1', HostAddrToStr(ip));
end;

procedure TNetDbTest.TestNextNameRRPastEdgeTCPA;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  rrn: TRRNameData;
  t: Cardinal;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);

  // get an RR from its normal position. need this to calculate the length.
  AssertTrue('NextNameRR should succeed.',
    NextNameRR(qd.Payload, ansstart,  rrn));

  // calculate the size in bytes of the rr so we can copy it to the end
  // of the payload buffer
  t := (rrn.RDataSt + rrn.RRMeta.RDLength) - ansstart;
  // copy the bytes, but leave off the last one. leave the rdlength unchanged.
  CopyBytesTo(qd.Payload,ansstart,Length(qd.Payload)-(t-1), t-1);
  AssertFalse('NextNameRR should fail.',
    NextNameRR(qd.Payload, Length(qd.Payload)-(t-1),  rrn));
end;

{
Call GetRRrecords with a start position past the end of the buffer.
}

procedure TNetDbTest.TestGetRRrecordsInvalidStart;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := High(Word);
  anslen := High(Word);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, anslen);
  AssertEquals(0, Length(RRArr));
end;

procedure TNetDbTest.TestGetRRrecordsInvalidStartTCP;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart: Word;
  RRArr: TRRNameDataArray;
begin
  BuildFakeResponseA(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := High(Word);
  anslen := High(Word);
  AssertEquals('Wrong number of A records.', 2, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, anslen);
  AssertEquals(0, Length(RRArr));
end;

procedure TNetDbTest.TestGetFixLenStrSimple;
const
  s = 'another fine mess';
var
  buf: TBuffer;
  pl: TPayload;
  tr: TTextArray;
  offset: Cardinal;
  res: ShortString;
begin
  tr[1] := s;
  tr[2] := '';
  tr[3] := '';
  tr[4] := '';
  tr[5] := '';
  SetLength(buf, 1024);
  offset := 0;
  WriteTextRecAsRData(buf, offset, tr);
  SetLength(buf, offset);
  BufferToPayload(buf, pl);
  // rdlength is word, so len byte for str is at offset 2 and str starts
  // at offset 3.
  GetFixlenStr(pl, 3, pl[2], res);
  AssertEquals(s, res);
end;

procedure TNetDbTest.TestGetFixLenStrSimpleTCP;
const
  s = 'another fine mess';
var
  buf: TBuffer;
  pl: TPayLoadTCP;
  tr: TTextArray;
  offset: Cardinal;
  res: ShortString;
begin
  tr[1] := s;
  tr[2] := '';
  tr[3] := '';
  tr[4] := '';
  tr[5] := '';
  SetLength(buf, 1024);
  offset := 0;
  WriteTextRecAsRData(buf, offset, tr);
  SetLength(buf, offset);
  BufferToPayload(buf, pl);
  // rdlength is word, so len byte for str is at offset 2 and str starts
  // at offset 3.
  GetFixlenStr(pl, 3, pl[2], res);
  AssertEquals(s, res);
end;

procedure TNetDbTest.TestGetFixLenStrSimpleAtEdge;
const
  s = 'another fine mess';
var
  buf: TBuffer;
  pl: TPayload;
  tr: TTextArray;
  offset,n: Cardinal;
  res: ShortString;
begin
  tr[1] := s;
  tr[2] := '';
  tr[3] := '';
  tr[4] := '';
  tr[5] := '';
  SetLength(buf, Length(pl));
  offset := Length(pl) - (Length(s)+3);
  n := offset+2;
  WriteTextRecAsRData(buf, offset, tr);
  SetLength(buf, offset);
  BufferToPayload(buf, pl);
  GetFixlenStr(pl, n+1, pl[n], res);
  AssertEquals(s, res);
end;

procedure TNetDbTest.TestGetFixLenStrSimpleTCPAtEdge;
const
  s = 'another fine mess';
var
  buf: TBuffer;
  pl: TPayLoadTCP;
  tr: TTextArray;
  offset,n: Cardinal;
  res: ShortString;
begin
  tr[1] := s;
  tr[2] := '';
  tr[3] := '';
  tr[4] := '';
  tr[5] := '';
  SetLength(buf, Length(pl));
  offset := Length(pl) - (Length(s)+3);
  n := offset+2;
  WriteTextRecAsRData(buf, offset, tr);
  SetLength(buf, offset);
  BufferToPayload(buf, pl);
  GetFixlenStr(pl, n+1, pl[n], res);
  AssertEquals(s, res);
end;

{
Test GetFixLenStr where len would take string past edge of buffer.
}
procedure TNetDbTest.TestGetFixLenStrSimplePastEdge;
var
  pl: TPayLoadTCP;
  res: ShortString;
begin
  pl[Length(pl) - 2] := 30;
  pl[Length(pl) - 1] := Ord('a');
  GetFixlenStr(pl, Length(pl)-1, pl[Length(pl)-2], res);
  AssertEquals('', res);
end;

procedure TNetDbTest.TestGetFixLenStrSimpleTCPPastEdge;
var
  pl: TPayLoadTCP;
  res: ShortString;
begin
  pl[Length(pl) - 2] := 30;
  pl[Length(pl) - 1] := Ord('a');
  GetFixlenStr(pl, Length(pl)-1, pl[Length(pl)-2], res);
  AssertEquals('', res);
end;

{
 read a label at the end of the buffer where the last byte is a count
 greater than 0. this is to try and trick stringfromlabel into reading past
 the end of the buffer.
}
procedure TNetDbTest.TestStringFromLabelCountAsLastByte;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryData;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
  startpos: Longint;
begin
  // we can use any of CNAME, NS or PTR because these RRs are just a single
  // domain name or series of labels.
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);

  // move the ns to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  // Set the last byte in the buffer to a high count
  qd.Payload[Length(qd.Payload)-1] := 63; // must be less than 64

  // need this var because stringfromlabel expects a longint that's a var type.
  startpos := RRarr[0].RDataSt;
  s := stringfromlabel(qd.Payload, startpos);
  AssertEquals('fakens.'+FAKEFQDN, s);
  AssertEquals(Length(qd.Payload), startpos);
end;

procedure TNetDbTest.TestStringFromLabelCountAsLastByteTCP;
var
  fakeresp: TFakeDNSResponse;
  qd: TQueryDataLengthTCP;
  anslen, ansstart, oldstart: Word;
  RRArr: TRRNameDataArray;
  s: TDNSDomainName;
  startpos: Longint;
begin
  // we can use any of CNAME, NS or PTR because these RRs are just a single
  // domain name or series of labels.
  BuildFakeResponseNS(FAKEFQDN, fakeresp);
  AssertTrue('Unable to convert fake dns response to querydata',
    BuildQueryData(fakeresp, qd, anslen));
  AssertTrue('CheckAnswer should return true.', CheckAnswer(qd.h,qd.h));
  ansstart := SkipAnsQueries(qd, anslen);
  AssertEquals('Wrong number of NS records.', 1, qd.h.ancount);
  RRArr := GetRRrecords(qd.Payload, ansstart, qd.h.ancount);
  AssertEquals('Wrong number of resource records.', 1, Length(RRArr));
  AssertEquals('RR 0 is not an NS RR.',DNSQRY_NS, RRarr[0].RRMeta.Atype);

  // move the ns to the end of the buffer.
  oldstart := RRArr[0].RDataSt;
  RRArr[0].RDataSt := Length(qd.Payload) - RRArr[0].RRMeta.RDLength;
  CopyBytesTo(qd.Payload, oldstart, RRArr[0].RDataSt, RRArr[0].RRMeta.RDLength);

  // Set the last byte in the buffer to a high count
  qd.Payload[Length(qd.Payload)-1] := 63; // must be less than 64

  // need this var because stringfromlabel expects a longint that's a var type.
  startpos := RRarr[0].RDataSt;
  s := stringfromlabel(qd.Payload, startpos);
  AssertEquals('fakens.'+FAKEFQDN, s);
  AssertEquals(Length(qd.Payload), startpos);
end;

procedure TNetDbTest.TestStringFromLabelCompress;
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayload;
  s: String;
  dmbs: TDNSDomainByteStream;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  offset := 0;
  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  offset2 := offset;
  // write same domain, this time we get compression.
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  BufferToPayload(buf,pl);
  s := stringfromlabel(pl, offset2);
  AssertEquals(FAKEFQDN,s);
end;

procedure TNetDbTest.TestStringFromLabelCompressTCP;
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayLoadTCP;
  s: String;
  dmbs: TDNSDomainByteStream;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  offset := 0;
  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  offset2 := offset;
  // write same domain, this time we get compression.
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  BufferToPayload(buf,pl);
  s := stringfromlabel(pl, offset2);
  AssertEquals(FAKEFQDN,s);
end;

procedure TNetDbTest.TestStringFromLabelCompressWithUncompressedLabel;
var
  buf: TBuffer;
  dmbs: TDNSDomainByteStream;
  offset: Cardinal;
  so: Longint;
  stt: TDomainCompressionTable;
  len: Word;
  pl: TPayload;
  s: String;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  // compress table empty so no compression here.
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  offset := 0;
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);

  so := offset;
  // should get compression on FAKEFQDN but label "foo" is written as full label.
  dmbs := DomainNameToByteStream('foo.' + FAKEFQDN, stt);
  len := CalcRdLength(dmbs);
  // len is 4 for 'foo' (including its length byte) and 2 for the pointer.
  AssertEquals(6, len);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  BufferToPayload(buf,pl);
  s := stringfromlabel(pl, so);
  AssertEquals('foo.'+FAKEFQDN,s);
end;

procedure TNetDbTest.TestStringFromLabelCompressWithUncompressedLabelTCP;
var
  buf: TBuffer;
  dmbs: TDNSDomainByteStream;
  offset: Cardinal;
  so: Longint;
  stt: TDomainCompressionTable;
  len: Word;
  pl: TPayLoadTCP;
  s: String;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  // compress table empty so no compression here.
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  offset := 0;
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);

  so := offset;
  // should get compression on FAKEFQDN but label "foo" is written as full label.
  dmbs := DomainNameToByteStream('foo.' + FAKEFQDN, stt);
  len := CalcRdLength(dmbs);
  // len is 4 for 'foo' (including its length byte) and 2 for the pointer.
  AssertEquals(6, len);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  BufferToPayload(buf,pl);
  s := stringfromlabel(pl, so);
  AssertEquals('foo.'+FAKEFQDN,s);
end;

{
Test stringfromlabel with a compressed label at the end of the buffer.
}
procedure TNetDbTest.TestStringFromLabelCompressEndBuffer;
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayload;
  s: String;
  dmbs: TDNSDomainByteStream;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  offset := 0;

  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  offset2 := offset;
  // write same domain, this time we get compression.
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);

  // write the pointer at the end of the payload buffer
  offset := Length(pl) - 2;
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  BufferToPayload(buf,pl);

  // read back the label.
  offset2 := Length(pl) - 2;
  s := stringfromlabel(pl, offset2);
  AssertEquals(FAKEFQDN,s);
end;

procedure TNetDbTest.TestStringFromLabelCompressEndBufferTCP;
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayLoadTCP;
  s: String;
  dmbs: TDNSDomainByteStream;
begin
  SetLength(buf, Length(pl));
  SetLength(stt,0);
  offset := 0;

  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  offset2 := offset;
  // write same domain, this time we get compression.
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);

  // write the pointer at the end of the payload buffer
  offset := Length(pl) - 2;
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  BufferToPayload(buf,pl);

  // read back the label.
  offset2 := Length(pl) - 2;
  s := stringfromlabel(pl, offset2);
  AssertEquals(FAKEFQDN,s);
end;

procedure TNetDbTest.TestStringFromLabelCompressSplit;
var
  pl: TPayload;
  s: String;
  offset: Longint;
begin
  // fill the buffer with 'A' so that we'll know if stringfromlabel read any
  // of it.
  FillByte(pl, Length(pl), 65);
  offset := Length(pl) - 1;
  pl[offset] := 192;
  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

procedure TNetDbTest.TestStringFromLabelCompressSplitTCP;
var
  pl: TPayLoadTCP;
  s: String;
  offset: Longint;
begin
  // fill the buffer with 'A' so that we'll know if stringfromlabel read any
  // of it.
  FillByte(pl, Length(pl), 65);
  offset := Length(pl) - 1;
  pl[offset] := 192;
  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

procedure TNetDbTest.TestStringFromLabelCompressPtrFwd;
var
  pl: TPayload;
  s: String;
  offset: Longint;
  ptr: TDNSDomainPointer;
begin
  FillByte(pl, Length(pl), 0);
  Move('foo', pl[21], 3);
  pl[20] := 3;

  ptr := GetDnsDomainPointer(32); // offset 20 + 12 for the header
  offset := 0;
  pl[offset] := ptr.b1;
  pl[offset+1] := ptr.b2;
  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

procedure TNetDbTest.TestStringFromLabelCompressPtrFwdTCP;
var
  pl: TPayLoadTCP;
  s: String;
  offset: Longint;
  ptr: TDNSDomainPointer;
begin
  FillByte(pl, Length(pl), 0);
  Move('foo', pl[21], 3);
  pl[20] := 3;

  ptr := GetDnsDomainPointer(32); // offset 20 + 12 for the header
  offset := 0;
  pl[offset] := ptr.b1;
  pl[offset+1] := ptr.b2;
  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

procedure TNetDbTest.TestStringFromLabelCompressAllPtrStart;
var
  pl: TPayload;
  s: String;
  offset: Longint;
begin
  FillByte(pl, Length(pl), 192);
  offset := 0;
  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

procedure TNetDbTest.TestStringFromLabelCompressAllPtrStartTCP;
var
  pl: TPayLoadTCP;
  s: String;
  offset: Longint;
begin
  FillByte(pl, Length(pl), 192);
  offset := 0;
  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

{
Test what happens when pointer is 0.
}
procedure TNetDbTest.TestStringFromLabelCompressedZero;
var
  pl: TPayLoad;
  s: String;
  offset: Longint;
  ptr: TDNSDomainPointer;
begin
  FillByte(pl, Length(pl), 0);
  pl[0] := 1;
  pl[1] := Ord('a');
  ptr := GetDnsDomainPointer(0);
  offset := 5;
  pl[offset] := ptr.b1;
  pl[offset+1] := ptr.b2;

  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

{
Test what happens when pointer is 0.
}
procedure TNetDbTest.TestStringFromLabelCompressedZeroTCP;
var
  pl: TPayLoadTCP;
  s: String;
  offset: Longint;
  ptr: TDNSDomainPointer;
begin
  FillByte(pl, Length(pl), 0);
  pl[0] := 1;
  pl[1] := Ord('a');
  ptr := GetDnsDomainPointer(0);
  offset := 5;
  pl[offset] := ptr.b1;
  pl[offset+1] := ptr.b2;

  s := stringfromlabel(pl, offset);
  AssertEquals('', s);
end;

procedure TNetDbTest.TestStringFromLabelInfiniteLoop;
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayload;
  s: String;
  dmbs: TDNSDomainByteStream;
  ptr: TDNSDomainPointer;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  offset := 0;
  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  ptr := GetDnsDomainPointer(12);

  // offset now points to 0 byte at end of label. We're overwriting that
  // 0 so that stringfromlabel will be tricked into a loop.
  Dec(offset);
  Move(ptr.ba, buf[offset], 2);

  BufferToPayload(buf,pl);
  offset2 := 0;
  s := stringfromlabel(pl, offset2);
  // if stringfromlabel returns at all then the test passed.
end;

procedure TNetDbTest.TestStringFromLabelInfiniteLoopTCP;
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayLoadTCP;
  s: String;
  dmbs: TDNSDomainByteStream;
  ptr: TDNSDomainPointer;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  offset := 0;
  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(FAKEFQDN, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  ptr := GetDnsDomainPointer(12);

  // offset now points to 0 byte at end of label. We're overwriting that
  // 0 so that stringfromlabel will be tricked into a loop.
  Dec(offset);
  Move(ptr.ba, buf[offset], 2);

  BufferToPayload(buf,pl);
  offset2 := 0;
  s := stringfromlabel(pl, offset2);
  // if stringfromlabel returns at all then the test passed.
end;

procedure TNetDbTest.TestCompressShortDomain;
const
  shortdomain = 'a.b';
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayload;
  s: String;
  dmbs: TDNSDomainByteStream;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  offset := 0;
  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(shortdomain, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  offset2 := offset;
  // second str is compressed
  dmbs := DomainNameToByteStream(shortdomain, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);

  BufferToPayload(buf,pl);
  s := stringfromlabel(pl, offset2);
  AssertEquals(shortdomain, s);
end;

procedure TNetDbTest.TestCompressShortDomainTCP;
const
  shortdomain = 'a.b';
var
  buf: TBuffer;
  stt: TDomainCompressionTable;
  offset: Cardinal;
  offset2: Longint;
  pl: TPayLoadTCP;
  s: String;
  dmbs: TDNSDomainByteStream;
begin
  SetLength(buf, 1024);
  SetLength(stt,0);
  offset := 0;
  // initial str is uncompressed because compress table empty
  dmbs := DomainNameToByteStream(shortdomain, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);
  offset2 := offset;
  // second str is compressed
  dmbs := DomainNameToByteStream(shortdomain, stt);
  WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs);

  BufferToPayload(buf,pl);
  s := stringfromlabel(pl, offset2);
  AssertEquals(shortdomain, s);
end;

procedure TNetDbTest.SetUp;
begin
  tsl := TStringList.Create;
end;

procedure TNetDbTest.TearDown;
begin
  tsl.Free;
end;

procedure TNetDbTest.CopyBytesTo(var buf: TPayLoad; startidx, destidx,
  count: Word);
begin
  // no tests for overlapping source and dest.
  if ((startidx+count) > Length(buf)) or ((destidx+count) > Length(buf)) then
    exit;
  Move(buf[startidx], buf[destidx], count);
end;

procedure TNetDbTest.CopyBytesTo(var buf: TPayLoadTCP; startidx, destidx,
  count: Word);
begin
  // no tests for overlapping source and dest.
  if ((startidx+count) > Length(buf)) or ((destidx+count) > Length(buf)) then
    exit;
  Move(buf[startidx], buf[destidx], count);
end;

function TNetDbTest.WriteNumToBuffer(var buf: TBuffer; var offset: Cardinal;
  val: Word): Word;
begin
  Result := 0;
  if (offset + SizeOf(val)) > Length(buf) then exit;
  Move(HToNs(val), buf[offset], SizeOf(val));
  Inc(offset, SizeOf(val));
  Result := SizeOf(val);
end;

function TNetDbTest.WriteNumToBuffer(var buf: TBuffer; var offset: Cardinal;
  val: Cardinal): Word;
begin
  Result := 0;
  if (offset + SizeOf(val)) > Length(buf) then exit;
  Move(HToNl(val), buf[offset], SizeOf(val));
  Inc(offset, SizeOf(val));
  Result := SizeOf(val);
end;

{
Write a number to the buffer without converting it to network byte order.
}
function TNetDbTest.WriteNumToBufferN(var buf: TBuffer; var offset: Cardinal;
  val: Word): Word;
begin
  Result := 0;
  if (offset + SizeOf(val)) > Length(buf) then exit;
  Move(val, buf[offset], SizeOf(val));
  Inc(offset, SizeOf(val));
  Result := SizeOf(val);
end;

{
Write a number to the buffer without converting it to network byte order.
}
function TNetDbTest.WriteNumToBufferN(var buf: TBuffer; var offset: Cardinal;
  val: Cardinal): Word;
begin
  Result := 0;
  if (offset + SizeOf(val)) > Length(buf) then exit;
  Move(val, buf[offset], SizeOf(val));
  Inc(offset, SizeOf(val));
  Result := SizeOf(val);
end;

{
Write an RR to the byte buffer. No compression of domain names will occur.
}
function TNetDbTest.WriteRRToBuffer(var buf: TBuffer; var offset: Cardinal;
  rr: TFakeRR): Word;
var
  s,etw: Word;
  dmbs: TDNSDomainByteStream;
  res: TRDataWriteRes;
begin
  etw := 0;
  s := offset;
  // write the RR Name
  dmbs := DomainNameToByteStream(rr.RRName);
  etw := CalcRdLength(dmbs);
  if WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs) < etw
  then
    Fail('Cannot write RR name to buffer at offset '+ inttostr(offset));

  if (offset + SizeOf(rr.Atype) + SizeOf(rr.AClass) + SizeOf(rr.TTL)) >
    Length(buf)
  then
    Fail('Not enough space to add RR type,class,ttl at offset '+ inttostr(offset));

  // Write the RR type, class and TTL.
  WriteNumToBuffer(buf, offset,rr.Atype);
  WriteNumToBuffer(buf, offset, rr.AClass);
  WriteNumToBuffer(buf, offset, rr.TTL);

  // now the RR data, which is type specific. Each type-specific method
  // also writes the RDLength word, so we have to account for 2 additional
  // bytes.
  case rr.Atype of
    DNSQRY_A:
        res := WriteAasRData(buf, offset, rr.ip);
    DNSQRY_AAAA:
        res := WriteAAAAasRData(buf, offset, rr.ip6);
    DNSQRY_SOA:
        res := WriteSOAasRData(buf, offset, rr.fsoa);
    DNSQRY_MX:
        res := WriteMXAsRData(buf, offset, rr.fmx);
    DNSQRY_NS:
      begin
        dmbs := DomainNameToByteStream(rr.nsh);
        res := WriteDomainAsRdata(buf,offset,dmbs);
      end;
    DNSQRY_PTR:
      begin
        dmbs := DomainNameToByteStream(rr.nsh);
        res := WriteDomainAsRdata(buf,offset,dmbs);
      end;
    DNSQRY_CNAME:
      begin
        dmbs := DomainNameToByteStream(rr.cn);
        res := WriteDomainAsRdata(buf,offset,dmbs);
      end;
    DNSQRY_TXT:
        res := WriteTextRecAsRData(buf, offset, rr.txtarr);
    DNSQRY_SRV:
        res := WriteSRVasRData(buf, offset, rr.fsrv);
  else
    Fail('Called to handle RR type '+inttostr(rr.Atype)+
      ' but no code to handle it.');
  end;

  if res.bw < res.etw then
    Fail('Unable to write RR of type ' +inttostr(RR.Atype) +
      ', name "'  + rr.RRName + '" to buffer at offset '+inttostr(offset)+
      '. Wrote '+inttostr(res.bw)+' bytes, expected to write '+
        inttostr(res.etw)+' bytes.');

  Result := offset - s;
end;

{
Write an RR to the output buffer, with compression of domain names turned on.
}
function TNetDbTest.WriteRRToBuffer(var buf: TBuffer; var offset: Cardinal;
  rr: TFakeRR; var ctbl: TDomainCompressionTable): Word;
var
  s, etw: Word;
  dmbs: TDNSDomainByteStream;
  res: TRDataWriteRes;
begin
  etw := 0;
  s := offset;
  // write the RR Name
  dmbs := DomainNameToByteStream(rr.RRName, ctbl);
  etw := CalcRdLength(dmbs);
  if WriteDNSDomainByteStreamToBuffer(buf, offset, dmbs) < etw
  then
    Fail('Cannot write RR name to buffer at offset '+ inttostr(offset));

  if (offset + SizeOf(rr.Atype) + SizeOf(rr.AClass) + SizeOf(rr.TTL)) >
    Length(buf)
  then
    Fail('Not enough space to add RR type,class,ttl at offset '+ inttostr(offset));

  // Write the RR type, class and TTL.
  WriteNumToBuffer(buf, offset,rr.Atype);
  WriteNumToBuffer(buf, offset, rr.AClass);
  WriteNumToBuffer(buf, offset, rr.TTL);

  // now the RR data, which is type specific. Each type-specific method
  // also writes the RDLength word, so we have to account for 2 additional
  // bytes.
  case rr.Atype of
    DNSQRY_A:
      begin
        res := WriteAasRData(buf, offset, rr.ip);
      end;
    DNSQRY_AAAA:
      begin
        res := WriteAAAAasRData(buf, offset, rr.ip6);
      end;
    DNSQRY_SOA:
      begin
        res := WriteSOAasRData(buf, offset, rr.fsoa);
      end;
    DNSQRY_MX:
      begin
        res := WriteMXAsRData(buf, offset, rr.fmx, ctbl);
      end;
    DNSQRY_NS:
      begin
        dmbs := DomainNameToByteStream(rr.nsh, ctbl);
        res := WriteDomainAsRdata(buf,offset,dmbs);
      end;
    DNSQRY_PTR:
      begin
        dmbs := DomainNameToByteStream(rr.nsh, ctbl);
        res := WriteDomainAsRdata(buf,offset,dmbs);
      end;
    DNSQRY_CNAME:
      begin
        dmbs := DomainNameToByteStream(rr.cn, ctbl);
        res := WriteDomainAsRdata(buf,offset,dmbs);
      end;
    DNSQRY_TXT:
      begin
        res := WriteTextRecAsRData(buf, offset, rr.txtarr);
      end;
    DNSQRY_SRV:
      begin
        res := WriteSRVasRData(buf, offset, rr.fsrv);
      end;
  else
    Fail('Called to handle RR type '+inttostr(rr.Atype)+
      ' but no code to handle it.');
  end;

  if res.bw < res.etw then
    Fail('Unable to write RR of type ' +inttostr(RR.Atype) +
      ', name "'  + rr.RRName + '" to buffer at offset '+inttostr(offset)+
      '. Wrote '+inttostr(res.bw)+' bytes, expected to write '+
        inttostr(res.etw)+' bytes.');

  Result := offset - s;
end;

{
Turn a fake DNS response into a payload buffer. This is a byte buffer minus the
DNS header. That is, the buffer begins with the question part of the response,
after which comes the RRs of the answers, authority, and additional sections.
}
function TNetDbTest.FakeDNSResponseToByteBuffer(fdr: TFakeDNSResponse; out
  buf: TBuffer; compress: Boolean): Cardinal;
var
  offset: Cardinal;
  rr: TFakeRR;
  dbs: TDNSDomainByteStream;
begin
  // plenty of room for our test responses. could precalculate this, but there's
  // no benefit. The return value of this function is the length of the
  // DNS reply, which we get for free since we have to track our offset into
  // the buffer as we write it.
  SetLength(buf, 2048);
  offset := 0;

  if compress then
    dbs := DomainNameToByteStream(fdr.qry.nm,fdr.strtable)
  else
    dbs := DomainNameToByteStream(fdr.qry.nm);

  // The question section consists of the dns query name, the qtype and
  // qclass.
  if WriteDNSDomainByteStreamToBuffer(buf, offset, dbs) < CalcRdLength(dbs)
  then
    Fail('Cannot write name to buffer at offset '+ inttostr(offset));

  WriteNumToBuffer(buf, offset, fdr.qry.qtype);
  WriteNumToBuffer(buf, offset, fdr.qry.qclass);

  // Now the answer sections.
  for rr in fdr.answers do
    if compress then
      WriteRRToBuffer(buf, offset, rr, fdr.strtable)
    else
      WriteRRToBuffer(buf, offset, rr);
  for rr in fdr.authority do
    if compress then
      WriteRRToBuffer(buf, offset, rr, fdr.strtable)
    else
      WriteRRToBuffer(buf, offset, rr);
  for rr in fdr.additional do
    if compress then
      WriteRRToBuffer(buf, offset, rr, fdr.strtable)
    else
      WriteRRToBuffer(buf, offset, rr);

  SetLength(buf, offset);
  Result := offset;
end;

{
Generate a TPayload buffer, a fixed-length array of byte, from the TBuffer
type, which is a variable-length array of byte.
}
function TNetDbTest.BufferToPayload(const buf: TBuffer;
  out pl: TPayload): Boolean;
begin
  Result := False;
  FillChar(pl,Length(pl),0);
  Move(buf[0], pl[0], Min(Length(pl),Length(buf)));
  Result := True;
end;

function TNetDbTest.BufferToPayload(const buf: TBuffer;
  out pl: TPayLoadTCP): Boolean;
begin
  Result := False;
  FillChar(pl,Length(pl),0);
  Move(buf[0], pl[0], Min(Length(pl),Length(buf)));
  Result := True;
end;

function TNetDbTest.BuildQueryData(fdr: TFakeDNSResponse; out qd: TQueryData;
  out qlen: Word; Compress: Boolean = False): Boolean;
var
  buf: TBuffer;
begin
  qlen := FakeDNSResponseToByteBuffer(fdr, buf, Compress);
  qd.h.ancount := HToNs(fdr.hdr.ancount);
  qd.h.arcount := HToNs(fdr.hdr.arcount);
  qd.h.nscount := HToNs(fdr.hdr.nscount);
  qd.h.qdcount := HToNs(fdr.hdr.qdcount);
  qd.h.flags1 := fdr.hdr.flags1;
  qd.h.flags2 := fdr.hdr.flags2;
  qd.h.id[0] := fdr.hdr.id[0];
  qd.h.id[1] := fdr.hdr.id[1];
  Result := BufferToPayload(buf, qd.Payload);
end;

function TNetDbTest.BuildQueryData(fdr: TFakeDNSResponse; out
  qd: TQueryDataLengthTCP; out qlen: Word; Compress: Boolean = False): Boolean;
var
  buf: TBuffer;
begin
  qlen := FakeDNSResponseToByteBuffer(fdr, buf, Compress);
  qd.h.ancount := HToNs(fdr.hdr.ancount);
  qd.h.arcount := HToNs(fdr.hdr.arcount);
  qd.h.nscount := HToNs(fdr.hdr.nscount);
  qd.h.qdcount := HToNs(fdr.hdr.qdcount);
  qd.h.flags1 := fdr.hdr.flags1;
  qd.h.flags2 := fdr.hdr.flags2;
  qd.h.id[0] := fdr.hdr.id[0];
  qd.h.id[1] := fdr.hdr.id[1];
  Result := BufferToPayload(buf, qd.Payload);
end;

{
Create a deliberately invalid DNS response to test our API's ability to cope
with invalid data without causing memory corruption.

After building a valid DNS response as normal, we truncate it at the given
offset.}
function TNetDbTest.BuildTruncatedQueryData(fdr: TFakeDNSResponse; out
  qd: TQueryData; out qlen: Word; truncoffset: Word): Boolean;
var
  buf: TBuffer;
begin
  qlen := FakeDNSResponseToByteBuffer(fdr, buf);
  qd.h.ancount := HToNs(fdr.hdr.ancount);
  qd.h.arcount := HToNs(fdr.hdr.arcount);
  qd.h.nscount := HToNs(fdr.hdr.nscount);
  qd.h.qdcount := HToNs(fdr.hdr.qdcount);
  qd.h.flags1 := fdr.hdr.flags1;
  qd.h.flags2 := fdr.hdr.flags2;
  qd.h.id[0] := fdr.hdr.id[0];
  qd.h.id[1] := fdr.hdr.id[1];
  SetLength(buf, truncoffset);
  Result := BufferToPayload(buf, qd.Payload);
end;

initialization

  RegisterTest(TNetDbTest);
end.

