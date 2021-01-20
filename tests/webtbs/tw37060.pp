program fp37060;

{$mode objfpc}{$H+}

uses sockets, Classes, SysUtils;

procedure BuildBadAddrs4(var bad_addrs: TStringList);
begin
  bad_addrs.Add('1.1.1.1.1'); // too many octets
  bad_addrs.Add('0xa.3.4.5'); //hex in octets
  bad_addrs.Add('9.$c.4.5'); //hex in octets
  bad_addrs.Add('12.$2.1.1.1'); //hex in octets
  bad_addrs.Add('000001.2.3.4'); // too many digits in octet
  bad_addrs.Add('1.000002.3.4'); // ditto second octet
  bad_addrs.Add('1.2.000003.4'); // ditto third octet
  bad_addrs.Add('1.2.3.000004'); // ditto fourth octet
  bad_addrs.Add('1.2.3.+4'); // math sign in octet
  bad_addrs.Add('1.2.-3.4'); // math sign in octet
  bad_addrs.Add('1.+2.3.4'); // math sign in octet
  bad_addrs.Add('-1.2.3.4'); // math sign in octet
  bad_addrs.Add('192.168.1'); // too few octets
  bad_addrs.Add('192.168'); // fewer still
  bad_addrs.Add('192'); // just one octet
  bad_addrs.Add('172.16.231.25.'); // ends with dot
  bad_addrs.Add('.172.16.231.25'); // starts with dot
  bad_addrs.Add('10...'); // dots but no digits
  bad_addrs.Add('310.259.1.1'); // overflow in octet 1
  bad_addrs.Add('10.259.1.1'); // ditto octet 2
  bad_addrs.Add('10.25.300.1'); // ditto octet 3
  bad_addrs.Add('10.25.30.301'); //ditto octet 4
  bad_addrs.Add('%1111.%11001110.30.4'); // binary
  bad_addrs.Add('&7.&5.30.4'); // octal
end;

procedure BuildGoodAddrs4(var good_addrs: TStringList);
begin
  good_addrs.Add('127.0.0.1|127.0.0.1');
  good_addrs.Add('0.0.0.0|0.0.0.0');
  good_addrs.Add('172.16.31.241|172.16.31.241');
  good_addrs.Add('255.255.255.255|255.255.255.255');
end;

procedure BuildBadAddrs6(var bad_addrs: TStringList);
begin
  // start with some obviously bad formats.
  bad_addrs.Add('');
  bad_addrs.Add(':');
  bad_addrs.Add(':::');
  bad_addrs.Add('::.');
  bad_addrs.Add('::::');
  bad_addrs.Add('fe80:');
  bad_addrs.Add('x:');
  bad_addrs.Add('.');
  bad_addrs.Add('....');

  // invalid chars in all 8 hextets.
  bad_addrs.Add('fe@0:b46c:c2a1:a202:9*6e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b4%c:c2a1:a202:9*6e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2#1:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a2^2:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:9*6e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9!d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:a=20:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:a520:41+2');

  // $ sign in hextets.
  bad_addrs.Add('$fe80:b46c:c2a1:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b$46c:c2a1:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1$:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926$e:9d2f:a520:4172');

  // last char is :
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:a520:4172:');

  // first char is :
  bad_addrs.Add(':fe80:b46c:c2a1:a202:926e:9d2f:a520:4172');

  // two sequences of collapsed zeroes. :: is the Highlander
  // sequence. There can be only one.
  bad_addrs.Add('fe80::c2a1:a202::9d2f:a520:4172');

  // 8 hextets plus collapsed zeroes, which means at least 1 hextet
  // of all zeroes, equaling 9 hextets.
  bad_addrs.Add('fe80:b46c:c2a1::a202:926e:9d2f:a520:4172');

  // try the same with the :: at the start.
  bad_addrs.Add('::b46c:c2a1:8fcb:a202:926e:9d2f:a520:4172');

  // and now try the same with the :: at the ned.
  bad_addrs.Add('b46c:a771:8fcb:a202:926e:9d2f:a520:4172::');

  // too many hextets
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:a520:4172:1211');

  // too few hextets
  bad_addrs.Add('fe80:b46c:9d2f:a520:4172:1211');

  // too many digits in each of the 8 hextets
  bad_addrs.Add('fe801:b46c:c2a1:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46cb:c2a1:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:0c2ad:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a2022:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e6:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:09d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:a5209:4172');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:a520:04172');

  // math signs in hextets. naive parsing of hextets with
  // math signs produces a positive result, but these are not
  // valid.
  bad_addrs.Add('fe80:-b46c:c2a1:a202:926e:9d2f:a520:4172');
  bad_addrs.Add('fe80:b46c:-c2a1:a202:926e:9d2f:a520:4172');

  // Hybrid 6 and 4 addresses.

  // ipv4 octet can't contain hex
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:19F.168.1.2');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.1A8.1.2');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.B.2');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.1.C');

  // ipv4 octets can't contain math signs
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:-192.168.1.1');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.-168.1.1');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.-1.1');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.1.-1');

  // invalid hybrid ipv6/ipv4 address, because there are 7
  // hextets before the ipv4. there can be only 6, as the 4
  // octets make up 2 hextets, and there can be no more than 8
  // hextets in total.
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:a520:192.168.1.2');

  // 5 hextets plus 2 hextets (4 octets) = 7. Must be 8.
  bad_addrs.Add('fe80:b46c:926e:9d2f:a520:192.168.1.2');

  // too few octets in ipv4 bit.
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.1');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192');

  // 7 hextets plus two octets of ipv4 = 8 hextets, but still must
  // not be parsed as valid because there must be 4 octets.
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:e2fc:192.168');

  // too many ipv4 octets
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.1.17.32');

  // addr starts with .
  bad_addrs.Add('.fe80:b46c:c2a1:a202:926e:9d2f:192.168.1.17');
  // addr ends with .
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.1.17.');

  // 6 hextets + 2 hextets (4 octets) plus collapsed zero sequence = 9 hextets.
  bad_addrs.Add('fe80:b46c::c2a1:a202:926e:9d2f:72.16.32.1');

  // repeat with :: at start
  bad_addrs.Add('::fe80:b46c:c2a1:a202:926e:9d2f:72.16.32.1');
  // and at end
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f::72.16.32.1');

  // ipv4 octets > 255
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:351.16.32.1');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:72.123216.32.1');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:72.16.9999999999999999999.1');
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:72.16.32.5e21');

  // dot sequence
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192..168.1.17');

  // start with dot
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:.192.168.1.17');

  // end with dot
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:9d2f:192.168.1.17.');

  // ipv4 octets followed by hextet
  bad_addrs.Add('fe80:b46c:c2a1:a202:926e:192.168.1.1:a2c1');

  // all zeroes but for mathematical operator.
  bad_addrs.Add('::-');

  // end on colon, but with earlier collapsed section.
  bad_addrs.Add('fe80::a202:926e:a2c1:');

  // too many digits in ipv4 octet
  bad_addrs.Add('21ac:d349:07c4:198e:6fab:df5a:0192.168.1.17');
  bad_addrs.Add('21ac:d349:07c4:198e:6fab:df5a:192.0168.1.17');
  bad_addrs.Add('21ac:d349:07c4:198e:6fab:df5a:192.168.0001.17');
  bad_addrs.Add('21ac:d349:07c4:198e:6fab:df5a:192.0168.1.0017');

  // just ipv4 address
  bad_addrs.Add('127.0.0.2');
end;

procedure BuildGoodAddrs6(var addrlist: TStringList);
begin
  // Each str is two parts, separated by a pipe. The left part is the input
  // address to be parsed, and the right is the expected result of taking the
  // resulting address and converting back to a string. This provides an
  // easy way to verify that the StrToHostAddr6 function parsed the address
  // correctly.
  // The values on the right have been double-checked with libc's inet_pton.
  addrlist.Add('::1|::0001');
  addrlist.Add('::|::');
  addrlist.Add('2001:4860:4000::|2001:4860:4000::');
  addrlist.Add('21ac:d349:07c4:198e:6fab:df5a:192.168.1.17|21AC:D349:07C4:198E:6FAB:DF5A:C0A8:0111');
  addrlist.Add('21ac:d349:07c4:198e:6fab:df5a:0.0.0.0|21AC:D349:07C4:198E:6FAB:DF5A::');
  addrlist.Add('::213.41.35.14|::D529:230E');
  addrlist.Add('fe80:b46c:c2a1:a202:926e:9d2f:a520:4172|FE80:B46C:C2A1:A202:926E:9D2F:A520:4172');
  addrlist.Add('a:b:c:d:e:f:0:1|000A:000B:000C:000D:000E:000F::0001');
  addrlist.Add('a:B:c:D:e:f:9:1|000A:000B:000C:000D:000E:000F:0009:0001');
  addrlist.Add('::0.0.0.0|::');
end;

function TestAddrs4(al: TStringList): Cardinal;
var
  bad_addr: String;
  i4: in_addr;
  res: Boolean;
begin
  Result := 0;
  for bad_addr in al do
  begin
    res := TryStrToHostAddr(bad_addr, i4);
    if res = True then
    begin
      writeln('  [x] '+bad_addr);
      Inc(Result);
    end;
  end;
end;

function TestGoodAddrs4(al: TStringList): Cardinal;
var
  addr,instr,parsed_addr,expected: String;
  i4: in_addr;
  idx: Cardinal;
  res: Boolean;
begin
  Result := 0;
  for addr in al do
  begin
    idx := Pos('|', addr);
    if idx > 0 then
    begin
      instr := Copy(addr,1,idx-1);
      res := TryStrToHostAddr(instr, i4);
      if res = False then
      begin
        writeln(' [x] '+instr);
        exit;
      end;
      expected := Copy(addr, idx+1, Length(addr)-idx);
      parsed_addr := HostAddrToStr(i4);
      if parsed_addr <> expected then
        writeln(' [X] "'+instr+' -> '+parsed_addr)
      else
        Inc(Result);
    end;
  end;
end;

function TestAddrs6(al: TStringList): Cardinal;
var
  bad_addr: String;
  i6: in6_addr;
  res: Boolean;
begin
  Result := 0;
  for bad_addr in al do
  begin
    res := TryStrToHostAddr6(bad_addr, i6);
    if res = True then
    begin
      writeln('  [x] '+bad_addr);
      Inc(Result);
    end;
  end;
end;

function TestGoodAddrs6(al: TStringList): Cardinal;
var
  addr,instr,parsed_addr,expected: String;
  i6: in6_addr;
  idx: Cardinal;
  res: Boolean;
begin
  Result := 0;
  for addr in al do
  begin
    idx := Pos('|', addr);
    if idx > 0 then
    begin
      instr := Copy(addr,1,idx-1);
      res := TryStrToHostAddr6(instr, i6);
      if res = False then
      begin
        writeln(' [x] '+instr);
        exit;
      end;
      expected := Copy(addr, idx+1, Length(addr)-idx);
      parsed_addr := HostAddrToStr6(i6);
      if parsed_addr <> expected then
        writeln(' [X] "'+instr+' -> '+parsed_addr)
      else
        Inc(Result);
    end;
  end;
end;

var
  addrlist: TStringList;
  count: Cardinal;
begin
  ExitCode := 0;
  addrlist := TStringList.Create;

  // IPv4 Tests

  writeln('IPv4');
  BuildBadAddrs4(addrlist);
  count := TestAddrs4(addrlist);
  writeln('Got True result for '+inttostr(count)+' out of '+
    inttostr(addrlist.Count)+' ip4 bad addresses.');
  // if we successfully parsed any bad addresses
  if count > 0 then ExitCode := 1;

  addrlist.Clear;
  BuildGoodAddrs4(addrlist);
  count := TestGoodAddrs4(addrlist);
  writeln('Got True result for '+inttostr(count)+' out of '+
    inttostr(addrlist.Count)+' good ip4 addresses.');
  writeln();

  // if we didn't parse all the good addresses.
  if count < addrlist.Count then
    ExitCode := 1;

  // IPv6 Tests

  writeln('IPv6');
  BuildBadAddrs6(addrlist);
  count := TestAddrs6(addrlist);
  writeln('Got True result for '+inttostr(count)+' out of '+
    inttostr(addrlist.Count)+' ip6 bad addresses.');
  // if we successfully parsed any bad addresses
  if count > 0 then ExitCode := 1;

  addrlist.Clear;
  BuildGoodAddrs6(addrlist);
  count := TestGoodAddrs6(addrlist);
  writeln('Got True result for '+inttostr(count)+' out of '+
    inttostr(addrlist.Count)+' good ip6 addresses.');

  // if we didn't parse all the good addresses.
  if count < addrlist.Count then
    ExitCode := 1;

  addrlist.Free;
end.

