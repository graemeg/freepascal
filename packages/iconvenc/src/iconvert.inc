
{$ifndef Unix}
 {$define noerrnoiconv}
{$endif}

function Iconvert(S: string; var Res: string; const FromEncoding, ToEncoding: string): cint;
var
  InLen, OutLen, Offset: size_t;
  Src, Dst: pchar;
  H: iconv_t;
  {$ifndef noerrnoiconv}
  lerr: cint;
  {$endif}
  iconvres: size_t;
begin
  H := iconv_open(PChar(ToEncoding), PChar(FromEncoding));
  if h=Iconv_t(-1) then
  begin
    Res := S;
    exit(-1);
  end;

  try
    InLen:=Length(s);
    {$ifdef noerrnoiconv}
      // guestimate based on http://svn.php.net/viewvc/php/php-src/trunk/ext/iconv/iconv.c?revision=280602&view=markup
      outlen:=InLen*Sizeof(longint) + 15;
    {$else}
      outlen:=InLen;
    {$endif}
    setlength(res,outlen);

    Src := PChar(S);
    Dst := PChar(Res);


    {$ifdef noerrnoiconv}
      iconvres := iconv(H, @Src, @InLen, @Dst, @OutLen);
      if iconvres = size_t(-1) then
        begin
          res:=s;
          exit(-1);
        end;
       if outlen<8 then    // From PHP, URL above, see also  PHP bug 55042 (they didn't recalc their "dest")
        begin
          Offset:=Dst-PChar(Res);
          SetLength(Res, Length(Res)+8); // 5 is minimally one utf-8 char
          Dst:=PChar(Res)+Offset;
          OutLen:=Length(Res)-Offset;
        end;
       InLen=0;
       iconvres:=iconv(H, nil, @InLen, @Dst, @Outlen);
       if iconvres = size_t(-1) then
         begin
           res:=s;
           exit(-1);
         end;
    {$else}    
    while InLen > 0 do
    begin
      iconvres := iconv(H, @Src, @InLen, @Dst, @OutLen);
      if iconvres = size_t(-1) then
      begin
        lerr := cerrno;
        if lerr = ESysEILSEQ then // unknown char, skip
          begin
            Dst^ := Src^;
            Inc(Src);
            Inc(Dst);
            Dec(InLen);
            Dec(OutLen);
          end
        else
          if lerr = ESysE2BIG then
            begin
              Offset := Dst - PChar(Res);
              SetLength(Res, Length(Res)+InLen*2+5); // 5 is minimally one utf-8 char
              Dst := PChar(Res) + Offset;
              OutLen := Length(Res) - Offset;
            end
          else
            exit(-1)
      end;
    end;
   
    // iconv has a buffer that needs flushing, specially if the last char is not #0
    iconvres:=iconv(H, nil, nil, @Dst, @Outlen);
    lerr:=cerrno;
    if (iconvres=size_t(-1)) and (lerr=ESysE2BIG) then
      begin
        Offset:=Dst-PChar(Res);
        SetLength(Res, Length(Res)+InLen*2+5); // 5 is minimally one utf-8 char
        Dst:=PChar(Res)+Offset;
        OutLen:=Length(Res)-Offset;
        InLen:=0;
        iconv(H, nil, @InLen, @Dst, @Outlen);
      end;
   {$endif}
    // trim output buffer
    SetLength(Res, Length(Res) - Outlen);
  finally 
        Offset:=Dst-PChar(Res);
        SetLength(Res, Length(Res)+InLen*2+5); // 5 is minimally one utf-8 char
        Dst:=PChar(Res)+Offset;
        OutLen:=Length(Res)-Offset;
        InLen:=0;
        iconvres:=iconv(H, nil, @InLen, @Dst, @Outlen);
        setlength(Res,Length(Res) - Outlen);
    iconv_close(H);
  end;
  
  Result := 0;
end;
