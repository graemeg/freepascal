uses
  baseunix,
  unix,
  lincd;

Function ReadCDTOC(Device : String; Var CDTOC : Array of TTocEntry) : Integer;

Var
  I,Drive    : Integer;
  tochdr   : Tcdrom_tochdr;
  tocentry : tcdrom_tocentry;

begin
  drive:=fpOpen(Device, Open_RDONLY or Open_NONBLOCK);
  if drive<0 then
    begin
    Result:=-1;
    Exit;
    end;
  if fpioctl(drive, CDROMREADTOCHDR, @tochdr)<>0 then
    begin
    Result:=-1;
    Exit;
    end;
  If (tochdr.cdth_trk1-tochdr.cdth_trk0)>High(CDToc) then
    Result:=-2
  else
    begin
    Result:=0;
    for i := tochdr.cdth_trk0 to tochdr.cdth_trk1 do
      begin
      tocentry.cdte_track := i;
      tocentry.cdte_format := CDROM_MSF;
      fpIOCtl(drive, CDROMREADTOCENTRY, @tocentry);
      // We should do some error checking here actually.
      With cdtoc[result] do
        begin
        min := tocentry.cdte_addr.msf.minute;
        sec := tocentry.cdte_addr.msf.second;
        frame := tocentry.cdte_addr.msf.frame;
        inc(frame,min*60*75);
        inc(frame,sec*75);
        end;
      Inc(result);
      end;
    tocentry.cdte_track := $AA;
    tocentry.cdte_format := CDROM_MSF;
    fpIOCtl(drive, CDROMREADTOCENTRY, @tocentry);
    With cdtoc[Result] do
      begin
      Min := tocentry.cdte_addr.msf.minute;
      sec := tocentry.cdte_addr.msf.second;
      frame := tocentry.cdte_addr.msf.frame;
      inc(frame, min*60*75);
      inc(frame, sec*75);
      end;
    end;
  fpClose(drive);
end;

{ ---------------------------------------------------------------------
    /etc/fstab scanning.
  ---------------------------------------------------------------------}

Function ExtractDevice(S : String) : String;

Var
  P,L : Integer;

begin
  Result:='';
  P:=Pos('#',S);
  If P<>0 then
    S:=Copy(S,1,P-1);
  If Length(S)>0 then
    begin
    P:=1;
    While (P<=Length(S)) and (S[P] in [#9,' ']) do
      Inc(p);
    L:=P;
    While (L<=Length(S)) and (Not (S[L] in [#9,' '])) do
      Inc(L);
    If L>P then
      Result:=Copy(S,P,L-P);
    end;
end;

Function TestFSTab(var Devices : Array of String) : Integer;

Var
  fstab : text;
  Line : String;

begin
  Result:=0;
  Assign(FSTab,'/etc/fstab');
  {$i-}
  Reset(fstab);
  {$i+}
  If IOResult=0 then
    begin
    While Not EOF(fstab) do
      begin
      ReadLn(fsTab,Line);
      Line:=ExtractDevice(Line);
      If IsCdDevice(Line) and (Result<=High(Devices)) then
        begin
        Devices[Result]:=Line;
        inc(Result);
        end;
      end;
    Close(fstab);
    end
  else
    Result:=-1;
end;

Function GetCDRomDevices(Var Devices : Array of string) : Integer;

Var
  S : String;

begin
  Result:=TestFSTab(Devices);
  If (Result<1) then
    begin
    S:=DetectCD;
    If (S<>'') then
      begin
      Devices[0]:=S;
      Result:=1;
      end;
    end
end;


