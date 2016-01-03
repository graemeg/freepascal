{$mode objfpc}
{$h+}
program ppxview;

uses sysutils,pxlib, ctypes;


procedure DumpInfo (Doc : PPX_Doc);

Var
  I : Integer;
  S : String;
  pxf : Ppxfield_t;
  
begin
  I:=1;
  pxf:=PX_get_fields(Doc);
  While I<=PX_get_num_fields(Doc) do
    begin
    Write('Field ',I:3,': ',strpas(pxf^.px_fname):18,' : ');
    S:='';
    Case (pxf^.px_ftype) of
      pxfAlpha:    S:=Format('char(%d)',[pxf^.px_flen]);
      pxfDate:     S:=Format('date(%d)',[pxf^.px_flen]);
      pxfShort:    S:=Format('int(%d)',[pxf^.px_flen]);
      pxfLong:     S:=Format('int(%d)',[pxf^.px_flen]);
      pxfCurrency: S:=Format('currency(%d)',[pxf^.px_flen]);
      pxfNumber:   S:=Format('double(%d)',[pxf^.px_flen]);
      pxfLogical:  S:=Format('boolean(%d)',[pxf^.px_flen]);
      pxfMemoBLOb: S:=Format('memoblob(%d)',[pxf^.px_flen]);
      pxfBLOb:     S:=Format('blob(%d)',[pxf^.px_flen]);
      pxfFmtMemoBLOb: S:=Format('fmtmemoblob(%d)',[pxf^.px_flen]);
      pxfOLE:      S:=Format('ole(%d)',[pxf^.px_flen]);
      pxfGraphic:  S:=Format('graphic(%d)',[pxf^.px_flen]);
      pxfTime:     S:=Format('time(%d)',[pxf^.px_flen]);
      pxfTimestamp: S:=Format('timestamp(%d)',[pxf^.px_flen]);
      pxfAutoInc:  S:=Format('autoinc(%d)',[pxf^.px_flen]);       
      pxfBCD:      S:=Format('decimal(%d,%d)',[pxf^.px_flen*2, pxf^.px_fdc]);
      pxfBytes:    S:=Format('bytes(%d)',[pxf^.px_flen]);
    else
      S:=Format('Unknnown type (%d) (%d)',[pxf^.px_ftype, pxf^.px_flen]);
    end;
    Writeln(S);
    Inc(I);
    Inc(pxf);
    end;
end;

Procedure DumpRecords(Doc : PPX_Doc);

var
  I,J,K,flen : Integer;
  pxf : Ppxfield_t;
  s : string;          
  buf, fbuf,value : Pchar;
  longv : clong;
  y,m,d : cint;
<<<<<<< HEAD
<<<<<<< HEAD
  si: shortint;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  R : Double;
  c : char;
  
begin
  I:=0;
  Buf:=GetMem(PX_get_recordSize(Doc));
  For I:=0 to px_get_num_records(Doc)-1 do
    begin
    Writeln('Record : ',I+1:4);
    Writeln('=============');
    PX_get_record(Doc,I, Buf);
    pxf:=PX_get_fields(Doc);
    fbuf:=Buf;
    For J:=0 to PX_get_num_fields(Doc)-1 do
      begin
      flen:=pxf^.px_flen;
      Case (pxf^.px_ftype) of
        pxfAlpha: 
          if PX_get_data_alpha(Doc,fbuf,flen,@value)>0 then
            begin
            S:=Strpas(value);
            doc^.free(doc,value);
            end;
        pxfDate:
           if PX_get_data_long(Doc,fbuf,flen,@longv)>0 then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
              {$ifdef windows}
              S:=DateToStr(Longv+1721425-2415019);
              {$else}
              PX_SdnToGregorian(longv+1721425,@Y,@M,@D);
              S:=DateToStr(EncodeDate(Y,M,D));
              {$endif}
              end;
        pxfShort:
          if PX_get_data_short(Doc,fbuf, flen, @si)>0 then
            S:=IntToStr(si);
=======
=======
>>>>>>> origin/fixes_2_2
              PX_SdnToGregorian(longv+1721425,@Y,@M,@D);
              S:=DateToStr(EncodeDate(Y,M,D));
              end;
        pxfShort:
          if PX_get_data_short(Doc,fbuf, flen, @D)>0 then
            S:=IntToStr(D);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        pxfAutoInc,
        pxfLong:
          if (PX_get_data_long(Doc,fbuf,flen,@longv)>0) then
            S:=IntToStr(Longv);
        pxfCurrency,
        pxfNumber:
          If (PX_get_data_double(Doc,FBuf,Flen,@R)>0) then
            S:=Format('%f',[R]);
        pxfLogical:
          if (PX_get_data_byte(Doc,FBuf,flen,@C)>0) then
            S:=BoolToStr(C<>#0);
        pxfMemoBLOb,
        pxfBLOb,
        pxfFmtMemoBLOb,
        pxfOLE,
        pxfGraphic,
        pxfBytes:
          begin
          S:='';
          if (pxf^.px_ftype=pxfGraphic) then
            Y:=PX_get_data_graphic(Doc,FBuf,FLen,@M,@D,@Value)
          else if (pxf^.px_ftype=pxfBytes) then
            Y:=PX_get_data_bytes(Doc,FBuf,FLen,@Value)
          else  
            Y:=PX_get_data_blob(Doc,FBuf,FLen,@M,@D,@Value);
          If (Y>0) then
            If Assigned(Value) then
              begin
              S:='';
              If pxf^.px_ftype in [pxfMemoBLOb,pxfFmtMemoBLOb] then
                begin
                // Not null terminated.
                SetLength(S,D);
                Move(Value^,S[1],D);
                end
              else  
                For K:=0 to D-1 do
                  S:=S+' '+HexStr(Ord(Value[K]),2);
              doc^.free(doc,value);
              end
            else
              S:='<Null>';
          end;     
        pxfTime:   
          if (PX_get_data_long(Doc,fbuf,flen,@longv)>0) then
            S:=TimeToStr(longv/MSecsPerDay);
        pxfTimestamp:
          if (PX_get_data_double(Doc,fbuf,flen,@R)>0) then
            begin
            value:=PX_timestamp2string(Doc,R,'Y-m-d H:i:s');
            S:=Strpas(Value);
            doc^.free(doc,value);
            end; 
        pxfBCD:
          if (PX_get_data_bcd(Doc,pcuchar(FBuf),pxf^.px_fdc,@Value)>0) then
            begin
            S:=strpas(value);
            doc^.free(doc,value);
            end;
      else
        S:=Format('Unknnown type (%d) (%d)',[pxf^.px_ftype, pxf^.px_flen]);
      end;
      WriteLn(strpas(pxf^.px_fname):18,' = ',S);
      Inc(fbuf,Flen);
      Inc(Pxf);
      end;
    end;  
<<<<<<< HEAD
<<<<<<< HEAD
  FreeMem(Buf);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

Var
  Doc : PPX_Doc;
  FN,BFN : String;

begin
  LoadPXlib(pxlibraryname);
  PX_Boot;
  try
    Doc:=px_new();
    Try
      FN:=ParamStr(1);
      BFN:=ChangeFileExt(FN,'.mb');
      px_open_file(Doc,Pchar(FN));
      try
        if FileExists(BFN) then
          PX_set_blob_file(Doc,PChar(BFN));
        DumpInfo(Doc);
        DumpRecords(Doc);
      Finally  
        PX_close(Doc);
      end;  
    Finally
      PX_Delete(Doc);
    end;  
  finally
    PX_Shutdown;
  end;
end.