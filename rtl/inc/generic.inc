{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by the Free Pascal development team.

    Processor independent implementation for the system unit
    (adapted for intel i386.inc file)

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function align(addr : PtrUInt;alignment : PtrUInt) : PtrUInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  var
    tmp: PtrUInt;
  begin
    tmp:=addr+PtrUInt(alignment-1);
    result:=tmp-(tmp mod alignment)
  end;


{$ifndef cpujvm}
function align(addr : Pointer;alignment : PtrUInt) : Pointer;{$ifdef SYSTEMINLINE}inline;{$endif}
  var
    tmp: PtrUInt;
  begin
    tmp:=PtrUInt(addr)+(alignment-1);
    result:=pointer(ptruint(tmp-(tmp mod alignment)));
  end;
{$endif}

{****************************************************************************
                               Primitives
****************************************************************************}
type
  pstring = ^shortstring;

{$ifndef FPC_HAS_SHORTSTR_SHORTSTR_INTERN_CHARMOVE}
{$define FPC_HAS_SHORTSTR_SHORTSTR_INTERN_CHARMOVE}
procedure fpc_shortstr_shortstr_intern_charmove(const src: shortstring; const srcindex: byte; var dst: shortstring; const dstindex, len: byte); {$ifdef SYSTEMINLINE}inline;{$endif}
begin
  move(src[srcindex],dst[dstindex],len);
end;
{$endif FPC_HAS_SHORTSTR_SHORTSTR_INTERN_CHARMOVE}

{$ifndef FPC_HAS_SHORTSTR_CHARARRAY_INTERN_CHARMOVE}
{$define FPC_HAS_SHORTSTR_CHARARRAY_INTERN_CHARMOVE}
procedure fpc_shortstr_chararray_intern_charmove(const src: shortstring; out dst: array of char; const len: sizeint);
begin
  move(src[1],pchar(@dst)^,len);
end;
{$endif FPC_HAS_SHORTSTR_CHARARRAY_INTERN_CHARMOVE}


{$ifndef FPC_SYSTEM_HAS_MOVE}
procedure Move(const source;var dest;count:SizeInt);[public, alias: 'FPC_MOVE'];
var
  aligncount : sizeint;
  pdest,psrc,pend : pbyte;
begin
  if (@dest=@source) or (count<=0) then
    exit;
  if (@dest<@source) or (@source+count<@dest) then
    begin
      { Forward Move }
      psrc:=@source;
      pdest:=@dest;
      if (Count>4*sizeof(ptruint)-11)
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
        and ((PtrUInt(pdest) and (sizeof(PtrUInt)-1))=(PtrUInt(psrc) and (sizeof(PtrUInt)-1)))
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
        then
        begin
          { Align on native pointer size }
          aligncount:=(sizeof(PtrUInt)-PtrInt(pdest)) and (sizeof(PtrUInt)-1);
          dec(count,aligncount);
          pend:=psrc+aligncount;
          while psrc<pend do
            begin
              pdest^:=psrc^;
              inc(pdest);
              inc(psrc);
            end;
          { use sizeuint typecast to force shr optimization }
          pptruint(pend):=pptruint(psrc)+(sizeuint(count) div sizeof(ptruint));
          while psrc<pend do
            begin
              pptruint(pdest)^:=pptruint(psrc)^;
              inc(pptruint(pdest));
              inc(pptruint(psrc));
            end;
          count:=count and (sizeof(PtrUInt)-1);
        end;
      pend:=psrc+count;
      while psrc<pend do
        begin
          pdest^:=psrc^;
          inc(pdest);
          inc(psrc);
        end;
    end
  else
    begin
      { Backward Move }
      psrc:=@source+count;
      pdest:=@dest+count;
      if (Count>4*sizeof(ptruint)-11)
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
        and ((PtrUInt(pdest) and (sizeof(PtrUInt)-1))=(PtrUInt(psrc) and (sizeof(PtrUInt)-1)))
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
        then
        begin
          { Align on native pointer size }
          aligncount:=(PtrUInt(pdest) and (sizeof(PtrUInt)-1));
          dec(count,aligncount);
          pend:=psrc-aligncount;
          while psrc>pend do
            begin
              dec(pdest);
              dec(psrc);
              pdest^:=psrc^;
            end;
          { use sizeuint typecast to force shr optimization }
          pptruint(pend):=pptruint(psrc)-(sizeuint(count) div sizeof(ptruint));
          while psrc>pend do
            begin
              dec(pptruint(pdest));
              dec(pptruint(psrc));
              pptruint(pdest)^:=pptruint(psrc)^;
            end;
          count:=count and (sizeof(PtrUInt)-1);
        end;
      pend:=psrc-count;
      while psrc>pend do
        begin
          dec(pdest);
          dec(psrc);
          pdest^:=psrc^;
        end;
    end;
end;
{$endif not FPC_SYSTEM_HAS_MOVE}


{$ifndef FPC_SYSTEM_HAS_FILLCHAR}
Procedure FillChar(var x;count:SizeInt;value:byte);
var
  pdest,pend : pbyte;
  v : ALUUInt;
begin
  if count <= 0 then
    exit;
  pdest:=@x;
  if Count>4*sizeof(ptruint)-1 then
    begin
{$if sizeof(v)>=2}
      v:=(value shl 8) or value;
{$endif sizeof(v)>=2}
{$if sizeof(v)>=4}
      v:=(v shl 16) or v;
{$endif sizeof(v)>=4}
{$if sizeof(v)=8}
      v:=(v shl 32) or v;
{$endif sizeof(v)=8}
      { Align on native pointer size }
      pend:=pbyte(align(pdest,sizeof(PtrUInt)));
      dec(count,pend-pdest);
      while pdest<pend do
        begin
          pdest^:=value;
          inc(pdest);
        end;
      { use sizeuint typecast to force shr optimization }
      pptruint(pend):=pptruint(pdest)+(sizeuint(count) div sizeof(ptruint));
      while pdest<pend do
        begin
          pptruint(pdest)^:=v;
          inc(pptruint(pdest));
        end;
      count:=count and (sizeof(ptruint)-1);
    end;
  pend:=pdest+count;
  while pdest<pend do
    begin
      pdest^:=value;
      inc(pdest);
    end;
end;
{$endif FPC_SYSTEM_HAS_FILLCHAR}


{$ifndef FPC_SYSTEM_HAS_FILLWORD}
procedure fillword(var x;count : SizeInt;value : word);
var
  aligncount : sizeint;
  pdest,pend : pword;
  v : ALUUInt;
begin
  if count <= 0 then
    exit;
  pdest:=@x;
  if Count>4*sizeof(ptruint)-1 then
    begin
{$if sizeof(v)>=4}
      v:=(value shl 16) or value;
{$endif sizeof(v)>=4}
{$if sizeof(v)=8}
      v:=(v shl 32) or v;
{$endif sizeof(v)=8}
      { Align on native pointer size }
      aligncount:=(PtrUInt(pdest) and (sizeof(PtrUInt)-1)) shr 1;
      dec(count,aligncount);
      pend:=pdest+aligncount;
      while pdest<pend do
        begin
          pdest^:=value;
          inc(pdest);
        end;
      { use sizeuint typecast to force shr optimization }
      pptruint(pend):=pptruint(pdest)+((sizeuint(count)*2) div sizeof(ptruint));
      while pdest<pend do
        begin
          pptruint(pdest)^:=v;
          inc(pptruint(pdest));
        end;
      count:=((count*2) and (sizeof(ptruint)-1)) shr 1;
    end;
  pend:=pdest+count;
  while pdest<pend do
    begin
      pdest^:=value;
      inc(pdest);
    end;
end;
{$endif not FPC_SYSTEM_HAS_FILLWORD}


{$ifndef FPC_SYSTEM_HAS_FILLDWORD}
procedure filldword(var x;count : SizeInt;value : dword);
var
  aligncount : sizeint;
  pdest,pend : pdword;
  v : ALUUInt;
begin
  if count <= 0 then
    exit;
  pdest:=@x;
  if Count>4*sizeof(ptruint)-1 then
    begin
      v:=value;
{$if sizeof(v)=8}
      v:=(v shl 32) or v;
{$endif sizeof(v)=8}
      { Align on native pointer size }
      aligncount:=(PtrUInt(pdest) and (sizeof(PtrUInt)-1)) shr 2;
      dec(count,aligncount);
      pend:=pdest+aligncount;
      while pdest<pend do
        begin
          pdest^:=value;
          inc(pdest);
        end;
      { use sizeuint typecast to force shr optimization }
      pptruint(pend):=pptruint(pdest)+((sizeuint(count)*4) div sizeof(ptruint));
      while pdest<pend do
        begin
          pptruint(pdest)^:=v;
          inc(pptruint(pdest));
        end;
      count:=((count*4) and (sizeof(ptruint)-1)) shr 2;
    end;
  pend:=pdest+count;
  while pdest<pend do
    begin
      pdest^:=value;
      inc(pdest);
    end;
end;
{$endif FPC_SYSTEM_HAS_FILLDWORD}


{$ifndef FPC_SYSTEM_HAS_FILLQWORD}
procedure fillqword(var x;count : SizeInt;value : qword);
var
  pdest,pend : pqword;
begin
  if count <= 0 then
    exit;
  pdest:=@x;
  pend:=pdest+count;
  while pdest<pend do
    begin
      pdest^:=value;
      inc(pdest);
    end;
end;
{$endif FPC_SYSTEM_HAS_FILLQWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXBYTE}
function IndexByte(Const buf;len:SizeInt;b:byte):SizeInt;
var
  psrc,pend : pbyte;
begin
  psrc:=@buf;
  { simulate assembler implementations behaviour, which is expected }
  { fpc_pchar_to_ansistr in astrings.inc                            }
  if (len < 0) or
     (psrc+len < psrc) then
    pend:=pbyte(high(PtrUInt)-sizeof(byte))
  else
    pend:=psrc+len;
  while (psrc<pend) do
    begin
      if psrc^=b then
        begin
          result:=psrc-pbyte(@buf);
          exit;
        end;
      inc(psrc);
    end;
  result:=-1;
end;
{$endif not FPC_SYSTEM_HAS_INDEXBYTE}


{$ifndef FPC_SYSTEM_HAS_INDEXWORD}
function Indexword(Const buf;len:SizeInt;b:word):SizeInt;
var
  psrc,pend : pword;
begin
  psrc:=@buf;
  { simulate assembler implementations behaviour, which is expected }
  { fpc_pchar_to_ansistr in astrings.inc                            }
  if (len < 0) or
     { is this ever true? }
     (len > high(PtrInt)) or
     (psrc+len < psrc) then
    pend:=pword(high(PtrUInt)-sizeof(word))
  else
    pend:=psrc+len;
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
  if (ptruint(psrc) mod 2)<>0 then
    while psrc<pend do
      begin
        if unaligned(psrc^)=b then
          begin
            result:=psrc-pword(@buf);
            exit;
          end;
        inc(psrc);
      end
  else
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    while psrc<pend do
      begin
        if psrc^=b then
          begin
            result:=psrc-pword(@buf);
            exit;
          end;
        inc(psrc);
      end;
  result:=-1;
end;
{$endif not FPC_SYSTEM_HAS_INDEXWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXDWORD}
function IndexDWord(Const buf;len:SizeInt;b:DWord):SizeInt;
var
  psrc,pend : pdword;
begin
  psrc:=@buf;
  { simulate assembler implementations behaviour, which is expected }
  { fpc_pchar_to_ansistr in astrings.inc                            }
  if (len < 0) or
     (len > high(PtrInt) div 2) or
     (psrc+len < psrc) then
    pend:=pdword(high(PtrUInt)-PtrUInt(sizeof(dword)))
  else
    pend:=psrc+len;
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
  if (ptruint(psrc) mod 4)<>0 then
    while psrc<pend do
      begin
        if unaligned(psrc^)=b then
          begin
            result:=psrc-pdword(@buf);
            exit;
          end;
        inc(psrc);
      end
  else
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    while psrc<pend do
      begin
        if psrc^=b then
          begin
            result:=psrc-pdword(@buf);
            exit;
          end;
        inc(psrc);
      end;
  result:=-1;
end;
{$endif not FPC_SYSTEM_HAS_INDEXDWORD}


{$ifndef FPC_SYSTEM_HAS_INDEXQWORD}
function IndexQWord(Const buf;len:SizeInt;b:QWord):SizeInt;
var
  psrc,pend : pqword;
begin
  psrc:=@buf;
  { simulate assembler implementations behaviour, which is expected }
  { fpc_pchar_to_ansistr in astrings.inc                            }
  if (len < 0) or
     (len > high(PtrInt) div 4) or
     (psrc+len < psrc) then
    pend:=pqword(high(PtrUInt)-PtrUInt(sizeof(qword)))
  else
    pend:=psrc+len;
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
  if (ptruint(psrc) mod 8)<>0 then
    while psrc<pend do
      begin
        if unaligned(psrc^)=b then
          begin
            result:=psrc-pqword(@buf);
            exit;
          end;
        inc(psrc);
      end
  else
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    while psrc<pend do
      begin
        if psrc^=b then
          begin
            result:=psrc-pqword(@buf);
            exit;
          end;
        inc(psrc);
      end;
  result:=-1;
end;
{$endif not FPC_SYSTEM_HAS_INDEXQWORD}


{$ifndef FPC_SYSTEM_HAS_COMPAREBYTE}
function CompareByte(Const buf1,buf2;len:SizeInt):SizeInt;
var
  aligncount : sizeint;
  psrc,pdest,pend : pbyte;
  b : ptrint;
begin
  b:=0;
  psrc:=@buf1;
  pdest:=@buf2;
  if (len>4*sizeof(ptruint)-1)
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
    and ((PtrUInt(pdest) and (sizeof(PtrUInt)-1))=(PtrUInt(psrc) and (sizeof(PtrUInt)-1)))
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    then
    begin
      { Align on native pointer size }
      aligncount:=(sizeof(PtrUInt)-(PtrUInt(pdest) and (sizeof(PtrUInt)-1))) and (sizeof(PtrUInt)-1);
      dec(len,aligncount);
      pend:=psrc+aligncount;
      while psrc<pend do
        begin
          b:=(ptrint(psrc^)-ptrint(pdest^));
          if b<>0 then
            begin
              if b<0 then
                exit(-1)
              else
                exit(1);
            end;
          inc(pdest);
          inc(psrc);
        end;
      { use sizeuint typecast to force shr optimization }
      pptruint(pend):=pptruint(psrc)+(sizeuint(len) div sizeof(ptruint));
      len:=len and (sizeof(PtrUInt)-1);
      while psrc<pend do
        begin
          b:=(pptrint(psrc)^-pptrint(pdest)^);
          if b<>0 then
            begin
              len:=sizeof(ptruint);
              break;
            end;
          inc(pptruint(pdest));
          inc(pptruint(psrc));
        end;
    end;
  if (psrc+len >= psrc) then
    pend:=psrc+len
  else
    pend:=pbyte(high(ptruint)-1);
  while psrc<pend do
    begin
      b:=(ptrint(psrc^)-ptrint(pdest^));
      if b<>0 then
        begin
          if b<0 then
            exit(-1)
          else
            exit(1);
        end;
      inc(pdest);
      inc(psrc);
    end;
  result:=0;
end;
{$endif not FPC_SYSTEM_HAS_COMPAREBYTE}


{$ifndef FPC_SYSTEM_HAS_COMPAREWORD}
function CompareWord(Const buf1,buf2;len:SizeInt):SizeInt;
var
  aligncount : sizeint;
  psrc,pdest,pend : pword;
  b : ptrint;
begin
  b:=0;
  psrc:=@buf1;
  pdest:=@buf2;
  if (len>4*sizeof(ptruint)-1)
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
    and ((PtrUInt(pdest) and (sizeof(PtrUInt)-1))=(PtrUInt(psrc) and (sizeof(PtrUInt)-1)))
    and (((PtrUInt(pdest) and 1) or (PtrUInt(psrc) and 1))=0)
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    then
     begin
      { Align on native pointer size }
      aligncount:=((sizeof(PtrUInt)-(PtrUInt(pdest) and (sizeof(PtrUInt)-1))) and (sizeof(PtrUInt)-1)) shr 1;
      dec(len,aligncount);
      pend:=psrc+aligncount;
      while psrc<pend do
        begin
          b:=(ptrint(psrc^)-ptrint(pdest^));
          if b<>0 then
            begin
              if b<0 then
                exit(-1)
              else
                exit(1);
            end;
          inc(pdest);
          inc(psrc);
        end;
      { use sizeuint typecast to force shr optimization }
      pptruint(pend):=pptruint(psrc)+(sizeuint(len)*2 div sizeof(ptruint));
      len:=((len*2) and (sizeof(PtrUInt)-1)) shr 1;
      while psrc<pend do
        begin
          b:=(pptrint(psrc)^-pptrint(pdest)^);
          if b<>0 then
            begin
              len:=sizeof(ptruint) shr 1;
              break;
            end;
          inc(pptruint(pdest));
          inc(pptruint(psrc));
        end;
    end;
  if (psrc+len >= psrc) then
    pend:=psrc+len
  else
    pend:=pword(high(ptruint)-2);
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
  if ((PtrUInt(pdest) and 1) or (PtrUInt(psrc) and 1))<>0 then
    while psrc<pend do
      begin
        b:=(ptrint(unaligned(psrc^))-ptrint(unaligned(pdest^)));
        if b<>0 then
          begin
            if b<0 then
              exit(-1)
            else
              exit(1);
          end;
        inc(pdest);
        inc(psrc);
      end
  else
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    while psrc<pend do
      begin
        b:=(ptrint(psrc^)-ptrint(pdest^));
        if b<>0 then
          begin
            if b<0 then
              exit(-1)
            else
              exit(1);
          end;
        inc(pdest);
        inc(psrc);
      end;
  result:=0;
end;
{$endif not FPC_SYSTEM_HAS_COMPAREWORD}


{$ifndef FPC_SYSTEM_HAS_COMPAREDWORD}
function CompareDWord(Const buf1,buf2;len:SizeInt):SizeInt;
var
  aligncount : sizeint;
  psrc,pdest,pend : pdword;
begin
  psrc:=@buf1;
  pdest:=@buf2;
  if (len>4*sizeof(ptruint)-11)
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
    and ((PtrUInt(pdest) and (sizeof(PtrUInt)-1))=(PtrUInt(psrc) and (sizeof(PtrUInt)-1)))
    and (((PtrUInt(pdest) and 3) or (PtrUInt(psrc) and 3))=0)
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    then
    begin
      { Align on native pointer size }
      aligncount:=((sizeof(PtrUInt)-(PtrUInt(pdest) and (sizeof(PtrUInt)-1))) and (sizeof(PtrUInt)-1)) shr 2;
      dec(len,aligncount);
      pend:=psrc+aligncount;
      while psrc<pend do
        begin
          if psrc^<>pdest^ then
            if psrc^>pdest^ then
              exit(1)
            else
              exit(-1);
          inc(pdest);
          inc(psrc);
        end;
      { use sizeuint typecast to force shr optimization }
      pptruint(pend):=pptruint(psrc)+(sizeuint(len)*4 div sizeof(ptruint));
      len:=((len*4) and (sizeof(PtrUInt)-1)) shr 2;
      while psrc<pend do
        begin
          if pptrint(psrc)^<>pptrint(pdest)^ then
            begin
              len:=sizeof(ptruint) shr 2;
              break;
            end;
          inc(pptruint(pdest));
          inc(pptruint(psrc));
        end;
    end;
  if (len <= high(ptrint) div 2) and
     (psrc+len >= psrc) then
    pend:=psrc+len
  else
    pend:=pdword(high(ptruint)-4);
{$ifdef FPC_REQUIRES_PROPER_ALIGNMENT}
  if ((PtrUInt(pdest) and 3) or (PtrUInt(psrc) and 3))<>0 then
    while psrc<pend do
      begin
        if unaligned(psrc^)<>unaligned(pdest^) then
          if unaligned(psrc^)>unaligned(pdest^) then
            exit(1)
          else
            exit(-1);
        inc(pdest);
        inc(psrc);
      end
  else
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
    while psrc<pend do
      begin
        if psrc^<>pdest^ then
          if psrc^>pdest^ then
            exit(1)
          else
            exit(-1);
        inc(pdest);
        inc(psrc);
      end;
  result:=0;
end;
{$endif ndef FPC_SYSTEM_HAS_COMPAREDWORD}


{$ifndef FPC_SYSTEM_HAS_MOVECHAR0}
procedure MoveChar0(Const buf1;var buf2;len:SizeInt);
var
  I : SizeInt;
begin
  if Len = 0 then
    exit;
  I:=IndexByte(Buf1,Len,0);
  if I<>-1 then
    Move(Buf1,Buf2,I)
  else
    Move(Buf1,Buf2,len);
end;
{$endif ndef FPC_SYSTEM_HAS_MOVECHAR0}


{$ifndef FPC_SYSTEM_HAS_INDEXCHAR0}
function IndexChar0(Const buf;len:SizeInt;b:Char):SizeInt;
var
  psrc,pend : pbyte;
begin
  psrc:=@buf;
  { simulate assembler implementations behaviour, which is expected }
  { fpc_pchar_to_ansistr in astrings.inc                            }
  if (len < 0) then
    pend:=pbyte(high(PtrUInt)-PtrUInt(sizeof(byte)))
  else
    pend:=psrc+len;
  while (psrc<pend) and (psrc^<>0) do
    begin
      if (psrc^=byte(b)) then
        begin
          result:=psrc-pbyte(@buf);
          exit;
        end;
      inc(psrc);
    end;
  result:=-1;
end;
{$endif ndef FPC_SYSTEM_HAS_INDEXCHAR0}


{$ifndef FPC_SYSTEM_HAS_COMPARECHAR0}
function CompareChar0(Const buf1,buf2;len:SizeInt):SizeInt;
var
  psrc,pdest,pend : pbyte;
  b : ptrint;
begin
  b:=0;
  psrc:=@buf1;
  pdest:=@buf2;
  pend:=psrc+len;
  while psrc<pend do
    begin
      b:=(ptrint(psrc^)-ptrint(pdest^));
      if b<0 then
        exit(-1)
      else if b>0 then
        exit(1);
      if (psrc^=0) or (pdest^=0) then
        exit(0);
      inc(pdest);
      inc(psrc);
    end;
  result:=0;
end;
{$endif not FPC_SYSTEM_HAS_COMPARECHAR0}


{****************************************************************************
                              Object Helpers
****************************************************************************}

{$ifdef FPC_HAS_FEATURE_OBJECTS}
type
  pobjectvmt=^tobjectvmt;
  tobjectvmt=record
    size,msize:sizeuint;
    parent:{$ifdef VER3_0}pointer{$else}ppointer{$endif};
  end;

{$ifndef FPC_SYSTEM_HAS_FPC_HELP_CONSTRUCTOR}
{ Note: _vmt will be reset to -1 when memory is allocated,
  this is needed for fpc_help_fail }
function fpc_help_constructor(_self:pointer;var _vmt:pointer;_vmt_pos:cardinal):pointer;[public,alias:'FPC_HELP_CONSTRUCTOR'];compilerproc;
var
  vmtcopy : pobjectvmt;
begin
  vmtcopy:=pobjectvmt(_vmt);
  { Inherited call? }
  if vmtcopy=nil then
    begin
      fpc_help_constructor:=_self;
      exit;
    end;

  if (_self=nil) and
     (vmtcopy^.size>0) then
    begin
      getmem(_self,vmtcopy^.size);
      { reset vmt needed for fail }
      _vmt:=pointer(-1);
    end;
  if _self<>nil then
    begin
      fillchar(_self^,vmtcopy^.size,0);
      ppointer(_self+_vmt_pos)^:=vmtcopy;
    end;
  fpc_help_constructor:=_self;
end;
{$endif FPC_SYSTEM_HAS_FPC_HELP_CONSTRUCTOR}


{$ifndef FPC_SYSTEM_HAS_FPC_HELP_DESTRUCTOR}
{ Note: _self will not be reset, the compiler has to generate the reset }
procedure fpc_help_destructor(_self,_vmt:pointer;vmt_pos:cardinal);[public,alias:'FPC_HELP_DESTRUCTOR'];  compilerproc;
begin
   { already released? }
   if (_self=nil) or
      (_vmt<>pointer(-1)) or
      (ppointer(_self+vmt_pos)^=nil) then
     exit;
   if (pobjectvmt(ppointer(_self+vmt_pos)^)^.size=0) or
      (pobjectvmt(ppointer(_self+vmt_pos)^)^.size+pobjectvmt(ppointer(_self+vmt_pos)^)^.msize<>0) then
     HandleErrorAddrFrameInd(210,get_pc_addr,get_frame);
   { reset vmt to nil for protection }
   ppointer(_self+vmt_pos)^:=nil;
   freemem(_self);
end;
{$endif FPC_SYSTEM_HAS_FPC_HELP_DESTRUCTOR}


{$ifndef FPC_SYSTEM_HAS_FPC_HELP_FAIL}
{ Note: _self will not be reset, the compiler has to generate the reset }
procedure fpc_help_fail(_self:pointer;var _vmt:pointer;vmt_pos:cardinal);[public,alias:'FPC_HELP_FAIL'];compilerproc;
begin
   if (_self=nil) or (_vmt=nil) then
     exit;
   { vmt=$ffffffff when memory was allocated }
   if ptruint(_vmt)=high(ptruint) then
     begin
       if (_self=nil) or (ppointer(_self+vmt_pos)^=nil) then
         HandleError(210)
       else
         begin
           ppointer(_self+vmt_pos)^:=nil;
           freemem(_self);
           { reset _vmt to nil so it will not be freed a
             second time }
           _vmt:=nil;
         end;
     end
   else
     ppointer(_self+vmt_pos)^:=nil;
end;
{$endif FPC_SYSTEM_HAS_FPC_HELP_FAIL}

{$ifndef FPC_SYSTEM_HAS_FPC_CHECK_OBJECT}
procedure fpc_check_object(_vmt : pointer); [public,alias:'FPC_CHECK_OBJECT'];  compilerproc;
begin
  if (_vmt=nil) or
     (pobjectvmt(_vmt)^.size=0) or
     (pobjectvmt(_vmt)^.size+pobjectvmt(_vmt)^.msize<>0) then
    HandleErrorAddrFrameInd(210,get_pc_addr,get_frame);
end;

{$endif ndef FPC_SYSTEM_HAS_FPC_CHECK_OBJECT}


{$ifndef FPC_SYSTEM_HAS_FPC_CHECK_OBJECT_EXT}
{ checks for a correct vmt pointer }
{ deeper check to see if the current object is }
{ really related to the true }
procedure fpc_check_object_ext(vmt, expvmt : pointer); [public,alias:'FPC_CHECK_OBJECT_EXT']; compilerproc;
begin
   if (vmt=nil) or
      (pobjectvmt(vmt)^.size=0) or
      (pobjectvmt(vmt)^.size+pobjectvmt(vmt)^.msize<>0) then
        HandleErrorAddrFrameInd(210,get_pc_addr,get_frame);
   while assigned(vmt) do
     if vmt=expvmt then
       exit
     else
       {$ifdef VER3_0}
       vmt:=pobjectvmt(vmt)^.parent;
       {$else VER3_0}
       if assigned(pobjectvmt(vmt)^.parent) then
         vmt:=pobjectvmt(vmt)^.parent^
       else
         vmt:=nil;
       {$endif}
   HandleErrorAddrFrameInd(219,get_pc_addr,get_frame);
end;
{$endif not FPC_SYSTEM_HAS_FPC_CHECK_OBJECT_EXT}


{$endif FPC_HAS_FEATURE_OBJECTS}

{****************************************************************************
                                 String
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_ASSIGN}

procedure fpc_shortstr_to_shortstr(out res:shortstring; const sstr: shortstring);[public,alias:'FPC_SHORTSTR_TO_SHORTSTR']; compilerproc;
var
  slen : byte;
begin
  slen:=length(sstr);
  if slen>high(res) then
    slen:=high(res);
  move(sstr[0],res[0],slen+1);
  res[0]:=chr(slen);
end;

procedure fpc_shortstr_assign(len:{$ifdef cpu16}smallint{$else}longint{$endif};sstr,dstr:pointer);[public,alias:'FPC_SHORTSTR_ASSIGN']; compilerproc;
var
  slen : byte;
begin
  slen:=length(pshortstring(sstr)^);
  if slen<len then
    len:=slen;
  move(sstr^,dstr^,len+1);
  if slen>len then
    pchar(dstr)^:=chr(len);
end;

{$endif ndef FPC_SYSTEM_HAS_FPC_SHORTSTR_ASSIGN}

{$push}
{ ensure that comparing addresses of openshortstrings with regular shortstrings
  doesn't cause errors }
{$t-}

{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_CONCAT}
procedure fpc_shortstr_concat(var dests:shortstring;const s1,s2:shortstring);compilerproc;
var
  s1l, s2l : ObjpasInt;
begin
  s1l:=length(s1);
  s2l:=length(s2);
  if s1l+s2l>high(dests) then
    begin
      if s1l>high(dests) then
        s1l:=high(dests);
      s2l:=high(dests)-s1l;
    end;
  if @dests=@s1 then
    fpc_shortstr_shortstr_intern_charmove(s2,1,dests,s1l+1,s2l)
  else
    if @dests=@s2 then
      begin
        fpc_shortstr_shortstr_intern_charmove(dests,1,dests,s1l+1,s2l);
        fpc_shortstr_shortstr_intern_charmove(s1,1,dests,1,s1l);
      end
  else
    begin
      fpc_shortstr_shortstr_intern_charmove(s1,1,dests,1,s1l);
      fpc_shortstr_shortstr_intern_charmove(s2,1,dests,s1l+1,s2l);
    end;
  dests[0]:=chr(s1l+s2l);
end;

procedure fpc_shortstr_concat_multi(var dests:shortstring;const sarr:array of pshortstring);compilerproc;
var
  s2l : byte;
  LowStart,i,
  Len : ObjpasInt;
  needtemp : boolean;
  tmpstr  : shortstring;
  p,pdest  : pshortstring;
begin
  if high(sarr)=0 then
    begin
      DestS:='';
      exit;
    end;
  lowstart:=low(sarr);
  if Pointer(@DestS)=Pointer(sarr[lowstart]) then
    inc(lowstart);
  { Check for another reuse, then we can't use
    the append optimization and need to use a temp }
  needtemp:=false;
  for i:=lowstart to high(sarr) do
    begin
      if Pointer(@DestS)=Pointer(sarr[i]) then
        begin
          needtemp:=true;
          break;
        end;
    end;
  if needtemp then
    begin
      lowstart:=low(sarr);
      tmpstr:='';
      pdest:=@tmpstr
    end
  else
    begin
      { Start with empty DestS if we start with concatting
        the first array element }
      if lowstart=low(sarr) then
        DestS:='';
      pdest:=@DestS;
    end;
  { Concat all strings, except the string we already
    copied in DestS }
  Len:=length(pdest^);
  for i:=lowstart to high(sarr) do
    begin
      p:=sarr[i];
      if assigned(p) then
        begin
          s2l:=length(p^);
          if Len+s2l>high(dests) then
            s2l:=high(dests)-Len;
          fpc_shortstr_shortstr_intern_charmove(p^,1,pdest^,Len+1,s2l);
          inc(Len,s2l);
        end;
    end;
  pdest^[0]:=Chr(Len);
  if needtemp then
    DestS:=TmpStr;
end;
{$endif ndef FPC_SYSTEM_HAS_FPC_SHORTSTR_CONCAT}

{$pop}

{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_APPEND_SHORTSTR}
procedure fpc_shortstr_append_shortstr(var s1:shortstring;const s2:shortstring);compilerproc;
    [public,alias:'FPC_SHORTSTR_APPEND_SHORTSTR'];
var
  s1l, s2l : sizeint;
begin
  s1l:=length(s1);
  s2l:=length(s2);
  if s1l+s2l>high(s1) then
    s2l:=high(s1)-s1l;
  move(s2[1],s1[s1l+1],s2l);
  s1[0]:=chr(s1l+s2l);
end;
{$endif ndef FPC_SYSTEM_HAS_FPC_SHORTSTR_APPEND_SHORTSTR}


{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_COMPARE}
function fpc_shortstr_compare(const left,right:shortstring) : longint;[public,alias:'FPC_SHORTSTR_COMPARE']; compilerproc;
var
   s1,s2,max,i : byte;
   d : ObjpasInt;
begin
  s1:=length(left);
  s2:=length(right);
  if s1<s2 then
    max:=s1
  else
    max:=s2;
  for i:=1 to max do
    begin
     d:=byte(left[i])-byte(right[i]);
     if d>0 then
       exit(1)
     else if d<0 then
       exit(-1);
    end;
  if s1>s2 then
    exit(1)
  else if s1<s2 then
    exit(-1)
  else
    exit(0);
end;
{$endif ndef FPC_SYSTEM_HAS_FPC_SHORTSTR_COMPARE}


{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_COMPARE_EQUAL}
function fpc_shortstr_compare_equal(const left,right:shortstring): longint; [public,alias:'FPC_SHORTSTR_COMPARE_EQUAL']; compilerproc;
begin
  Result := ObjpasInt(left[0]) - ObjpasInt(right[0]);
  if Result = 0 then
    Result := CompareByte(left[1],right[1], ObjpasInt(left[0]));
end;
{$endif ndef FPC_SYSTEM_HAS_FPC_SHORTSTR_COMPARE_EQUAL}


{$ifndef FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}


procedure fpc_pchar_to_shortstr(out res : shortstring;p:pchar);[public,alias:'FPC_PCHAR_TO_SHORTSTR']; compilerproc;
var
  l : ObjpasInt;
  s: shortstring;
begin
  if p=nil then
    l:=0
  else
    l:=strlen(p);
  if l>high(res) then
    l:=high(res);
  if l>0 then
    move(p^,s[1],l);
  s[0]:=chr(l);
  res:=s;
end;


{$endif ndef FPC_SYSTEM_HAS_FPC_PCHAR_TO_SHORTSTR}

{$ifndef cpujvm}

{ also define alias which can be used inside the system unit }
procedure fpc_pchar_to_shortstr(out res : shortstring;p:pchar);[external name 'FPC_PCHAR_TO_SHORTSTR'];


function strpas(p:pchar):shortstring;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    fpc_pchar_to_shortstr(result,p);
  end;

{$endif not cpujvm}


function Utf8CodePointLen(P: PAnsiChar; MaxLookAhead: SizeInt; IncludeCombiningDiacriticalMarks: Boolean): SizeInt;
  var
    bytes: sizeint;
    firstzerobit: byte;
  begin
    { see https://en.wikipedia.org/wiki/UTF-8#Description for details }

    if maxlookahead<=0 then
      begin
        { incomplete }
        result:=0;
        exit;
      end;
    { include the first byte }
    result:=1;
    { multiple byte utf-8 code point? }
    if p[0]>#127 then
      begin
        { bsr searches for the leftmost 1 bit. We are interested in the
          leftmost 0 bit, so first invert the value
        }
        firstzerobit:=bsrbyte(not(byte(p[0])));
        { if there is no zero bit or the first zero bit is the rightmost bit
          (bit 0), this is an invalid UTF-8 byte ($ff cannot appear in an
          UTF-8-encoded string, and in the worst case bit 1 has to be zero)
          Additionally, 5-byte UTF-8 sequences don't exist either, so bit 1
          cannot be the first zero-bit either. And bits 6 and 7 can't be 0
          either in the first byte.
        }
        if (firstzerobit<=1) or (firstzerobit>=6)  then
          begin
            result:=-result;
            exit;
          end;
        { the number of bytes belonging to this code point is
          7-(pos first 0-bit). Subtract 1 since we're already at the first
          byte. All subsequent bytes of the same sequence must have their
          highest bit set and the next one unset. We stop when we detect an
          invalid sequence.
        }
        bytes:=6-firstzerobit;
        while (result<maxlookahead) and
              (bytes>0) and
              ((ord(p[result]) and %11000000)=%10000000) do
          begin
            inc(result);
            dec(bytes);
          end;
        { stopped because of invalid/incomplete sequence -> exit }
        if bytes<>0 then
          begin
            if result>=maxlookahead then
              result:=0
            else
              result:=-result;
            exit;
          end;
      end;
    if includecombiningdiacriticalmarks then
      begin
        { combining diacritical marks?
            1) U+0300 - U+036F in UTF-8 = %11001100 10000000 - %11001101 10101111
            2) U+1AB0 - U+1AFF in UTF-8 = %11100001 10101010 10110000 - %11100001 10101011 10111111
            3) U+1DC0 - U+1DFF in UTF-8 = %11100001 10110111 10000000 - %11100001 10110111 10111111
            4) U+20D0 - U+20FF in UTF-8 = %11100010 10000011 10010000 - %11100010 10000011 10111111
            5) U+FE20 - U+FE2F in UTF-8 = %11101111 10111000 10100000 - %11101111 10111000 10101111
        }
        repeat
          bytes:=result;
          if result+1<maxlookahead then
            begin
              { case 1) }
              if ((ord(p[result]) and %11001100=%11001100)) and
                  (ord(p[result+1])>=%10000000) and
                  (ord(p[result+1])<=%10101111) then
                inc(result,2)
                  { case 2), 3), 4), 5) }
              else if (result+2<maxlookahead) and
                 (ord(p[result])>=%11100001) then
                begin
                     { case 2) }
                  if ((ord(p[result])=%11100001) and
                      (ord(p[result+1])=%10101010) and
                      (ord(p[result+2])>=%10110000) and
                      (ord(p[result+2])<=%10111111)) or
                     { case 3) }
                     ((ord(p[result])=%11100001) and
                      (ord(p[result+1])=%10110111) and
                      (ord(p[result+2])>=%10000000) and
                      (ord(p[result+2])<=%10111111)) or
                     { case 4) }
                     ((ord(p[result])=%11100010) and
                      (ord(p[result+1])=%10000011) and
                      (ord(p[result+2])>=%10010000) and
                      (ord(p[result+2])<=%10111111)) or
                     { case 5) }
                     ((ord(p[result])=%11101111) and
                      (ord(p[result+1])=%10111000) and
                      (ord(p[result+2])>=%10100000) and
                      (ord(p[result+2])<=%10101111)) then
                    inc(result,3);
                end;
            end;
        until bytes=result;
        { is there an incomplete diacritical mark? (invalid makes little sense:
          either a sequence is a combining diacritical mark, or it's not ; if
          it's invalid, it may also not have been a combining diacritical mark)
        }
        if result<maxlookahead then
          begin
               { case 1) }
            if (((ord(p[result]) and %11001100=%11001100)) and
                (result+1>=maxlookahead)) or
               { case 2) and 3)}
               ((ord(p[result])=%11100001) and
                ((result+1>=maxlookahead) or
                 (((ord(p[result+1])=%10101010) or
                   (ord(p[result+1])=%10110111)) and
                  (result+2>=maxlookahead)))) or
               { case 4 }
               ((ord(p[result])=%11100010) and
                ((result+1>=maxlookahead) or
                 ((ord(p[result+1])=%10000011) and
                  (result+2>=maxlookahead)))) or
               { case 5 }
               ((ord(p[result])=%11101111) and
                ((result+1>=maxlookahead) or
                 ((ord(p[result+1])=%10111000) and
                  (result+2>=maxlookahead)))) then
              begin
                result:=0;
                exit;
              end;
          end;
      end;
  end;

{$ifndef FPC_SYSTEM_HAS_FPC_CHARARRAY_TO_SHORTSTR}

procedure fpc_chararray_to_shortstr(out res : shortstring;const arr: array of char; zerobased: boolean = true);[public,alias:'FPC_CHARARRAY_TO_SHORTSTR']; compilerproc;
var
 l: ObjpasInt;
 index: ObjpasInt;
 len: byte;
begin
  l:=high(arr)+1;
  if l>=high(res)+1 then
    l:=high(res)
  else if l<0 then
    l:=0;
  if zerobased then
    begin
      index:=IndexByte(arr[0],l,0);
      if index<0 then
        len:=l
      else
        len:=index;
    end
  else
    len:=l;
  move(arr[0],res[1],len);
  res[0]:=chr(len);
end;

{$endif ndef FPC_SYSTEM_HAS_FPC_CHARARRAY_TO_SHORTSTR}


{$ifndef FPC_SYSTEM_HAS_FPC_SHORTSTR_TO_CHARARRAY}

procedure fpc_shortstr_to_chararray(out res: array of char; const src: ShortString); compilerproc;
var
  len: ObjpasInt;
begin
  len := length(src);
  if len > length(res) then
    len := length(res);
{$push}{$r-}
  { make sure we don't access char 1 if length is 0 (JM) }
  if len > 0 then
    move(src[1],res[0],len);
  fillchar(res[len],length(res)-len,0);
{$pop}
end;

{$endif FPC_SYSTEM_HAS_FPC_SHORTSTR_TO_CHARARRAY}

{$ifndef FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}

function fpc_pchar_length(p:pchar):sizeint;[public,alias:'FPC_PCHAR_LENGTH']; compilerproc;
begin
  if assigned(p) then
    Result:=IndexByte(p^,high(Result),0)
  else
    Result:=0;
end;

{$endif ndef FPC_SYSTEM_HAS_FPC_PCHAR_LENGTH}

{$ifndef FPC_SYSTEM_HAS_FPC_PWIDECHAR_LENGTH}

function fpc_pwidechar_length(p:pwidechar):sizeint;[public,alias:'FPC_PWIDECHAR_LENGTH']; compilerproc;
var i : sizeint;
begin
  i:=0;
  if assigned(p) then
    while p[i]<>#0 do
      inc(i);
  exit(i);
end;

{$endif ndef FPC_SYSTEM_HAS_FPC_PWIDECHAR_LENGTH}

{****************************************************************************
                       Caller/StackFrame Helpers
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_GET_FRAME}
{_$error Get_frame must be defined for each processor }
{$endif ndef FPC_SYSTEM_HAS_GET_FRAME}

{$ifndef FPC_SYSTEM_HAS_GET_CALLER_ADDR}
{_$error Get_caller_addr must be defined for each processor }
{$endif ndef FPC_SYSTEM_HAS_GET_CALLER_ADDR}

{$ifndef FPC_SYSTEM_HAS_GET_CALLER_FRAME}
{_$error Get_caller_frame must be defined for each processor }
{$endif ndef FPC_SYSTEM_HAS_GET_CALLER_FRAME}

{****************************************************************************
                                 Math
****************************************************************************}

{****************************************************************************
                          Software multiplication
****************************************************************************}
{$ifdef FPC_INCLUDE_SOFTWARE_MUL}

{$ifdef VER3_0}

{$ifndef FPC_SYSTEM_HAS_MUL_INTEGER}
    function fpc_mul_integer(f1,f2 : integer;checkoverflow : boolean) : integer;[public,alias: 'FPC_MUL_INTEGER']; compilerproc;
      var
        sign : boolean;
        q1,q2,q3 : word;
      begin
        { there's no difference between signed and unsigned multiplication,
          when the destination size is equal to the source size and overflow
          checking is off }
        if not checkoverflow then
          { word(f1)*word(f2) is coded as a call to mulword }
          fpc_mul_integer:=integer(word(f1)*word(f2))
        else
          begin
            sign:=false;
            if f1<0 then
              begin
                sign:=not(sign);
                q1:=word(-f1);
              end
            else
              q1:=f1;
            if f2<0 then
              begin
                sign:=not(sign);
                q2:=word(-f2);
              end
            else
              q2:=f2;
            { the q1*q2 is coded as call to mulword }
            q3:=q1*q2;

            if (q1 <> 0) and (q2 <>0) and
              ((q1>q3) or (q2>q3) or
              { the bit 63 can be only set if we have $8000 }
              { and sign is true                            }
              (q3 shr 15<>0) and
               ((q3<>word(word(1) shl 15)) or not(sign))
              ) then
              HandleErrorAddrFrameInd(215,get_pc_addr,get_frame);

            if sign then
              fpc_mul_integer:=-q3
            else
              fpc_mul_integer:=q3;
          end;
      end;
{$endif FPC_SYSTEM_HAS_MUL_INTEGER}


{$ifndef FPC_SYSTEM_HAS_MUL_WORD}
    function fpc_mul_word(f1,f2 : word;checkoverflow : boolean) : word;[public,alias: 'FPC_MUL_WORD']; compilerproc;
      var
        _f1,bitpos : word;
        b : byte;
        f1overflowed : boolean;
      begin
        fpc_mul_word:=0;
        bitpos:=1;
        f1overflowed:=false;

        for b:=0 to 15 do
          begin
            if (f2 and bitpos)<>0 then
              begin
                _f1:=fpc_mul_word;
                fpc_mul_word:=fpc_mul_word+f1;

                { if one of the operands is greater than the result an
                  overflow occurs                                      }
                if checkoverflow and (f1overflowed or ((_f1<>0) and (f1<>0) and
                  ((_f1>fpc_mul_word) or (f1>fpc_mul_word)))) then
                  HandleErrorAddrFrameInd(215,get_pc_addr,get_frame);
              end;
            { when bootstrapping, we forget about overflow checking for qword :) }
            f1overflowed:=f1overflowed or ((f1 and (1 shl 15))<>0);
            f1:=f1 shl 1;
            bitpos:=bitpos shl 1;
          end;
      end;
{$endif FPC_SYSTEM_HAS_MUL_WORD}


{$ifndef FPC_SYSTEM_HAS_MUL_LONGINT}
    function fpc_mul_longint(f1,f2 : longint;checkoverflow : boolean) : longint;[public,alias: 'FPC_MUL_LONGINT']; compilerproc;
      var
        sign : boolean;
        q1,q2,q3 : dword;
      begin
        { there's no difference between signed and unsigned multiplication,
          when the destination size is equal to the source size and overflow
          checking is off }
        if not checkoverflow then
          { dword(f1)*dword(f2) is coded as a call to muldword }
          fpc_mul_longint:=longint(dword(f1)*dword(f2))
        else
          begin
            sign:=false;
            if f1<0 then
              begin
                sign:=not(sign);
                q1:=dword(-f1);
              end
            else
              q1:=f1;
            if f2<0 then
              begin
                sign:=not(sign);
                q2:=dword(-f2);
              end
            else
              q2:=f2;
            { the q1*q2 is coded as call to muldword }
            q3:=q1*q2;

            if (q1 <> 0) and (q2 <>0) and
              ((q1>q3) or (q2>q3) or
              { the bit 31 can be only set if we have $8000 0000 }
              { and sign is true                                 }
              (q3 shr 15<>0) and
               ((q3<>dword(dword(1) shl 31)) or not(sign))
              ) then
              HandleErrorAddrFrameInd(215,get_pc_addr,get_frame);

            if sign then
              fpc_mul_longint:=-q3
            else
              fpc_mul_longint:=q3;
          end;
      end;
{$endif FPC_SYSTEM_HAS_MUL_INTEGER}


{$ifndef FPC_SYSTEM_HAS_MUL_DWORD}
    { multiplies two dwords
      the longbool for checkoverflow avoids a misaligned stack
    }
    function fpc_mul_dword(f1,f2 : dword;checkoverflow : boolean) : dword;[public,alias: 'FPC_MUL_DWORD']; compilerproc;
      var
        _f1,bitpos : dword;
        b : byte;
        f1overflowed : boolean;
      begin
        fpc_mul_dword:=0;
        bitpos:=1;
        f1overflowed:=false;

        for b:=0 to 31 do
          begin
            if (f2 and bitpos)<>0 then
              begin
                _f1:=fpc_mul_dword;
                fpc_mul_dword:=fpc_mul_dword+f1;

                { if one of the operands is greater than the result an
                  overflow occurs                                      }
                if checkoverflow and (f1overflowed or ((_f1<>0) and (f1<>0) and
                  ((_f1>fpc_mul_dword) or (f1>fpc_mul_dword)))) then
                  HandleErrorAddrFrameInd(215,get_pc_addr,get_frame);
              end;
            { when bootstrapping, we forget about overflow checking for qword :) }
            f1overflowed:=f1overflowed or ((f1 and (dword(1) shl 31))<>0);
            f1:=f1 shl 1;
            bitpos:=bitpos shl 1;
          end;
      end;
{$endif FPC_SYSTEM_HAS_MUL_DWORD}

{$else VER3_0}

{$ifndef FPC_SYSTEM_HAS_MUL_SHORTINT}
    function fpc_mul_shortint(f1,f2 : shortint) : shortint;[public,alias: 'FPC_MUL_SHORTINT']; compilerproc;
      begin
        { there's no difference between signed and unsigned multiplication,
          when the destination size is equal to the source size and overflow
          checking is off }
        { byte(f1) * byte(f2) is coded as a call to mul_byte }
        fpc_mul_shortint := shortint(byte(f1) * byte(f2));
      end;

    function fpc_mul_shortint_checkoverflow(f1,f2 : shortint) : shortint;[public,alias: 'FPC_MUL_SHORTINT_CHECKOVERFLOW']; compilerproc;
      var
        sign : boolean;
        q1,q2,q3 : byte;
      begin
        sign:=false;
        if f1 < 0 then
          begin
            sign := not(sign);
            q1 := byte(-f1);
          end
        else
          q1 := f1;
        if f2 < 0 then
          begin
            sign := not(sign);
            q2 := byte(-f2);
          end
        else
          q2 := f2;
        { the q1*q2 is coded as call to mul_byte }
{$push}
{$Q+}
        q3 := q1 * q2;
{$pop}

        if (q1 <> 0) and (q2 <> 0) and
          ((q1 > q3) or (q2 > q3) or
          { the bit 7 can be only set if we have $80 }
          { and sign is true                            }
          (q3 shr 7 <> 0) and
           ((q3 <> byte(byte(1) shl 7)) or not(sign))
          ) then
          FPC_Overflow();

        if sign then
          fpc_mul_shortint_checkoverflow := -q3
        else
          fpc_mul_shortint_checkoverflow := q3;
      end;
{$endif FPC_SYSTEM_HAS_MUL_SHORTINT}

{$ifndef FPC_SYSTEM_HAS_MUL_BYTE}
    function fpc_mul_byte(f1,f2 : byte) : byte;[public,alias: 'FPC_MUL_BYTE']; compilerproc;
      var
        bitpos : byte;
        b : byte;
      begin
        fpc_mul_byte := 0;
        bitpos := 1;

        for b := 0 to 7 do
          begin
            if (f2 and bitpos) <> 0 then
              fpc_mul_byte := fpc_mul_byte + f1;
            f1 := f1 shl 1;
            bitpos := bitpos shl 1;
          end;
      end;

    function fpc_mul_byte_checkoverflow(f1,f2 : byte) : byte;[public,alias: 'FPC_MUL_BYTE_CHECKOVERFLOW']; compilerproc;
      var
        _f1, bitpos : byte;
        b : byte;
        f1overflowed : boolean;
      begin
        fpc_mul_byte_checkoverflow := 0;
        bitpos := 1;
        f1overflowed := false;

        for b := 0 to 7 do
          begin
            if (f2 and bitpos) <> 0 then
              begin
                _f1 := fpc_mul_byte_checkoverflow;
                fpc_mul_byte_checkoverflow := fpc_mul_byte_checkoverflow + f1;

                { if one of the operands is greater than the result an
                  overflow occurs                                      }
                if f1overflowed or ((_f1 <> 0) and (f1 <> 0) and
                  ((_f1 > fpc_mul_byte_checkoverflow) or (f1 > fpc_mul_byte_checkoverflow))) then
                  FPC_Overflow();
              end;
            { when bootstrapping, we forget about overflow checking for qword :) }
            f1overflowed := f1overflowed or ((f1 and (1 shl 7)) <> 0);
            f1 := f1 shl 1;
            bitpos := bitpos shl 1;
          end;
      end;
{$endif FPC_SYSTEM_HAS_MUL_BYTE}

{$ifndef FPC_SYSTEM_HAS_MUL_INTEGER}
    function fpc_mul_integer(f1,f2 : integer) : integer;[public,alias: 'FPC_MUL_INTEGER']; compilerproc;
      begin
        { there's no difference between signed and unsigned multiplication,
          when the destination size is equal to the source size and overflow
          checking is off }
        { word(f1)*word(f2) is coded as a call to mulword }
        fpc_mul_integer:=integer(word(f1)*word(f2));
      end;

    function fpc_mul_integer_checkoverflow(f1,f2 : integer) : integer;[public,alias: 'FPC_MUL_INTEGER_CHECKOVERFLOW']; compilerproc;
      var
        sign : boolean;
        q1,q2,q3 : word;
      begin
        sign:=false;
        if f1<0 then
          begin
            sign:=not(sign);
            q1:=word(-f1);
          end
        else
          q1:=f1;
        if f2<0 then
          begin
            sign:=not(sign);
            q2:=word(-f2);
          end
        else
          q2:=f2;
        { the q1*q2 is coded as call to mulword }
{$push}
{$Q+}
        q3:=q1*q2;
{$pop}

        if (q1 <> 0) and (q2 <>0) and
          ((q1>q3) or (q2>q3) or
          { the bit 63 can be only set if we have $8000 }
          { and sign is true                            }
          (q3 shr 15<>0) and
           ((q3<>word(word(1) shl 15)) or not(sign))
          ) then
          FPC_Overflow();

        if sign then
          fpc_mul_integer_checkoverflow:=-q3
        else
          fpc_mul_integer_checkoverflow:=q3;
      end;
{$endif FPC_SYSTEM_HAS_MUL_INTEGER}


{$ifndef FPC_SYSTEM_HAS_MUL_WORD}
    function fpc_mul_word(f1,f2 : word) : word;[public,alias: 'FPC_MUL_WORD']; compilerproc;
      var
        bitpos : word;
        b : byte;
      begin
        fpc_mul_word:=0;
        bitpos:=1;

        for b:=0 to 15 do
          begin
            if (f2 and bitpos)<>0 then
              fpc_mul_word:=fpc_mul_word+f1;
            f1:=f1 shl 1;
            bitpos:=bitpos shl 1;
          end;
      end;

    function fpc_mul_word_checkoverflow(f1,f2 : word) : word;[public,alias: 'FPC_MUL_WORD_CHECKOVERFLOW']; compilerproc;
      var
        _f1,bitpos : word;
        b : byte;
        f1overflowed : boolean;
      begin
        fpc_mul_word_checkoverflow:=0;
        bitpos:=1;
        f1overflowed:=false;

        for b:=0 to 15 do
          begin
            if (f2 and bitpos)<>0 then
              begin
                _f1:=fpc_mul_word_checkoverflow;
                fpc_mul_word_checkoverflow:=fpc_mul_word_checkoverflow+f1;

                { if one of the operands is greater than the result an
                  overflow occurs                                      }
                if f1overflowed or ((_f1<>0) and (f1<>0) and
                  ((_f1>fpc_mul_word_checkoverflow) or (f1>fpc_mul_word_checkoverflow))) then
                  FPC_Overflow();
              end;
            { when bootstrapping, we forget about overflow checking for qword :) }
            f1overflowed:=f1overflowed or ((f1 and (1 shl 15))<>0);
            f1:=f1 shl 1;
            bitpos:=bitpos shl 1;
          end;
      end;
{$endif FPC_SYSTEM_HAS_MUL_WORD}


{$ifndef FPC_SYSTEM_HAS_MUL_LONGINT}
    function fpc_mul_longint(f1,f2 : longint) : longint;[public,alias: 'FPC_MUL_LONGINT']; compilerproc;
      begin
        { there's no difference between signed and unsigned multiplication,
          when the destination size is equal to the source size and overflow
          checking is off }
        { dword(f1)*dword(f2) is coded as a call to muldword }
        fpc_mul_longint:=longint(dword(f1)*dword(f2));
      end;

    function fpc_mul_longint_checkoverflow(f1,f2 : longint) : longint;[public,alias: 'FPC_MUL_LONGINT_CHECKOVERFLOW']; compilerproc;
      var
        sign : boolean;
        q1,q2,q3 : dword;
      begin
        sign:=false;
        if f1<0 then
          begin
            sign:=not(sign);
            q1:=dword(-f1);
          end
        else
          q1:=f1;
        if f2<0 then
          begin
            sign:=not(sign);
            q2:=dword(-f2);
          end
        else
          q2:=f2;
        { the q1*q2 is coded as call to muldword }
{$push}
{$Q+}
        q3:=q1*q2;
{$pop}

        if (q1 <> 0) and (q2 <>0) and
          ((q1>q3) or (q2>q3) or
          { the bit 31 can be only set if we have $8000 0000 }
          { and sign is true                                 }
          (q3 shr 31<>0) and
           ((q3<>dword(dword(1) shl 31)) or not(sign))
          ) then
          FPC_Overflow();

        if sign then
          fpc_mul_longint_checkoverflow:=-q3
        else
          fpc_mul_longint_checkoverflow:=q3;
      end;
{$endif FPC_SYSTEM_HAS_MUL_INTEGER}


{$ifndef FPC_SYSTEM_HAS_MUL_DWORD}
    function fpc_mul_dword(f1,f2 : dword) : dword;[public,alias: 'FPC_MUL_DWORD']; compilerproc;
      var
        bitpos : dword;
        b : byte;
      begin
        fpc_mul_dword:=0;
        bitpos:=1;

        for b:=0 to 31 do
          begin
            if (f2 and bitpos)<>0 then
              fpc_mul_dword:=fpc_mul_dword+f1;
            f1:=f1 shl 1;
            bitpos:=bitpos shl 1;
          end;
      end;

    function fpc_mul_dword_checkoverflow(f1,f2 : dword) : dword;[public,alias: 'FPC_MUL_DWORD_CHECKOVERFLOW']; compilerproc;
      var
        _f1,bitpos : dword;
        b : byte;
        f1overflowed : boolean;
      begin
        fpc_mul_dword_checkoverflow:=0;
        bitpos:=1;
        f1overflowed:=false;

        for b:=0 to 31 do
          begin
            if (f2 and bitpos)<>0 then
              begin
                _f1:=fpc_mul_dword_checkoverflow;
                fpc_mul_dword_checkoverflow:=fpc_mul_dword_checkoverflow+f1;

                { if one of the operands is greater than the result an
                  overflow occurs                                      }
                if f1overflowed or ((_f1<>0) and (f1<>0) and
                  ((_f1>fpc_mul_dword_checkoverflow) or (f1>fpc_mul_dword_checkoverflow))) then
                  FPC_Overflow();
              end;
            { when bootstrapping, we forget about overflow checking for qword :) }
            f1overflowed:=f1overflowed or ((f1 and (dword(1) shl 31))<>0);
            f1:=f1 shl 1;
            bitpos:=bitpos shl 1;
          end;
      end;
{$endif FPC_SYSTEM_HAS_MUL_DWORD}

{$endif VER3_0}

{$endif FPC_INCLUDE_SOFTWARE_MUL}

{****************************************************************************
                          Software longint/dword division
****************************************************************************}
{$ifdef FPC_INCLUDE_SOFTWARE_MOD_DIV}

{$ifndef FPC_SYSTEM_HAS_DIV_DWORD}
function fpc_div_dword(n,z : dword) : dword; [public,alias: 'FPC_DIV_DWORD']; compilerproc;
  var
     shift,lzz,lzn : ObjpasInt;
  begin
     result:=0;
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     if z=0 then
       exit;
     lzz:=BsrDWord(z);
     lzn:=BsrDWord(n);
     { if the denominator contains less zeros
       then the numerator
       then d is greater than the n }
     if lzn>lzz then
       exit;
     shift:=lzz-lzn;
     n:=n shl shift;
     for shift:=shift downto 0 do
       begin
         if z>=n then
           begin
              z:=z-n;
              result:=result+dword(dword(1) shl shift);
           end;
         n:=n shr 1;
       end;
  end;
{$endif FPC_SYSTEM_HAS_DIV_DWORD}


{$ifndef FPC_SYSTEM_HAS_MOD_DWORD}
function fpc_mod_dword(n,z : dword) : dword; [public,alias: 'FPC_MOD_DWORD']; compilerproc;
  var
     shift,lzz,lzn : ObjpasInt;
  begin
     result:=0;
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     if z=0 then
       exit;
     lzz:=BsrDWord(z);
     lzn:=BsrDWord(n);
     { if the denominator contains less zeros
       then the numerator
       then d is greater than the n }
     if lzn>lzz then
      begin
         result:=z;
         exit;
      end;
     shift:=lzz-lzn;
     n:=n shl shift;
     for shift:=shift downto 0 do
       begin
         if z>=n then
           z:=z-n;
         n:=n shr 1;
       end;
    result:=z;
  end;
{$endif FPC_SYSTEM_HAS_MOD_DWORD}


{$ifndef FPC_SYSTEM_HAS_DIV_WORD}
function fpc_div_word(n,z : word) : word; [public,alias: 'FPC_DIV_WORD']; compilerproc;
  var
     shift,lzz,lzn : Byte;
  begin
     result:=0;
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     if z=0 then
       exit;
     lzz:=BsrWord(z);
     lzn:=BsrWord(n);
     { if the denominator contains less zeros
       then the numerator
       then d is greater than the n }
     if lzn>lzz then
       exit;
     shift:=lzz-lzn;
     n:=n shl shift;
     for shift:=shift downto 0 do
       begin
         if z>=n then
           begin
              z:=z-n;
              result:=result+word(word(1) shl shift);
           end;
         n:=n shr 1;
       end;
  end;
{$endif FPC_SYSTEM_HAS_DIV_WORD}


{$ifndef FPC_SYSTEM_HAS_MOD_WORD}
function fpc_mod_word(n,z : word) : word; [public,alias: 'FPC_MOD_WORD']; compilerproc;
  var
     shift,lzz,lzn : Byte;
  begin
     result:=0;
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     if z=0 then
       exit;
     lzz:=BsrWord(z);
     lzn:=BsrWord(n);
     { if the denominator contains less zeros
       then the numerator
       then d is greater than the n }
     if lzn>lzz then
      begin
         result:=z;
         exit;
      end;
     shift:=lzz-lzn;
     n:=n shl shift;
     for shift:=shift downto 0 do
       begin
         if z>=n then
           z:=z-n;
         n:=n shr 1;
       end;
    result:=z;
  end;
{$endif FPC_SYSTEM_HAS_MOD_WORD}


{$ifndef FPC_SYSTEM_HAS_DIV_BYTE}
function fpc_div_byte(n,z : byte) : byte; [public,alias: 'FPC_DIV_BYTE']; compilerproc;
  var
     shift,lzz,lzn : Byte;
  begin
     result:=0;
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     if z=0 then
       exit;
     lzz:=BsrByte(z);
     lzn:=BsrByte(n);
     { if the denominator contains less zeros
       then the numerator
       then d is greater than the n }
     if lzn>lzz then
       exit;
     shift:=lzz-lzn;
     n:=n shl shift;
     for shift:=shift downto 0 do
       begin
         if z>=n then
           begin
              z:=z-n;
              result:=result+byte(byte(1) shl shift);
           end;
         n:=n shr 1;
       end;
  end;
{$endif FPC_SYSTEM_HAS_DIV_BYTE}


{$ifndef FPC_SYSTEM_HAS_MOD_BYTE}
function fpc_mod_byte(n,z : byte) : byte; [public,alias: 'FPC_MOD_BYTE']; compilerproc;
  var
     shift,lzz,lzn : Byte;
  begin
     result:=0;
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     if z=0 then
       exit;
     lzz:=BsrByte(z);
     lzn:=BsrByte(n);
     { if the denominator contains less zeros
       then the numerator
       then d is greater than the n }
     if lzn>lzz then
      begin
         result:=z;
         exit;
      end;
     shift:=lzz-lzn;
     n:=n shl shift;
     for shift:=shift downto 0 do
       begin
         if z>=n then
           z:=z-n;
         n:=n shr 1;
       end;
    result:=z;
  end;
{$endif FPC_SYSTEM_HAS_MOD_BYTE}


{$ifndef FPC_SYSTEM_HAS_DIV_LONGINT}
function fpc_div_longint(n,z : longint) : longint; [public,alias: 'FPC_DIV_LONGINT']; compilerproc;
  var
     sign : boolean;
     d1,d2 : dword;
  begin
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     sign:=false;
     if z<0 then
       begin
          sign:=not(sign);
          d1:=dword(-z);
       end
     else
       d1:=z;
     if n<0 then
       begin
          sign:=not(sign);
          d2:=dword(-n);
       end
     else
       d2:=n;

     { the div is coded by the compiler as call to divdword }
     if sign then
       result:=-(d1 div d2)
     else
       result:=d1 div d2;
  end;
{$endif FPC_SYSTEM_HAS_DIV_LONGINT}


{$ifndef FPC_SYSTEM_HAS_MOD_LONGINT}
function fpc_mod_longint(n,z : longint) : longint; [public,alias: 'FPC_MOD_LONGINT']; compilerproc;
  var
     signed : boolean;
     r,nq,zq : dword;
  begin
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     nq:=abs(n);

     if z<0 then
       begin
          zq:=dword(-z);
          signed:=true;
       end
     else
       begin
         zq:=z;
         signed:=false;
       end;

     r:=zq mod nq;
     if signed then
       result:=-longint(r)
     else
       result:=r;
  end;
{$endif FPC_SYSTEM_HAS_MOD_LONGINT}


{$ifndef FPC_SYSTEM_HAS_DIV_SMALLINT}
function fpc_div_smallint(n,z : smallint) : smallint; [public,alias: 'FPC_DIV_SMALLINT']; compilerproc;
  var
     sign : boolean;
     w1,w2 : word;
  begin
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     sign:=false;
     if z<0 then
       begin
          sign:=not(sign);
          w1:=word(-z);
       end
     else
       w1:=z;
     if n<0 then
       begin
          sign:=not(sign);
          w2:=word(-n);
       end
     else
       w2:=n;

     { the div is coded by the compiler as call to divdword }
     if sign then
       result:=-(w1 div w2)
     else
       result:=w1 div w2;
  end;
{$endif FPC_SYSTEM_HAS_DIV_SMALLINT}


{$ifndef FPC_SYSTEM_HAS_MOD_SMALLINT}
function fpc_mod_smallint(n,z : smallint) : smallint; [public,alias: 'FPC_MOD_SMALLINT']; compilerproc;
  var
     signed : boolean;
     r,nq,zq : word;
  begin
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     nq:=abs(n);

     if z<0 then
       begin
          zq:=word(-z);
          signed:=true;
       end
     else
       begin
         zq:=z;
         signed:=false;
       end;

     r:=zq mod nq;
     if signed then
       result:=-smallint(r)
     else
       result:=r;
  end;
{$endif FPC_SYSTEM_HAS_MOD_SMALLINT}


{$ifndef FPC_SYSTEM_HAS_DIV_SHORTINT}
function fpc_div_shortint(n,z : shortint) : shortint; [public,alias: 'FPC_DIV_SHORTINT']; compilerproc;
  var
     sign : boolean;
     b1,b2 : byte;
  begin
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     sign:=false;
     if z<0 then
       begin
          sign:=not(sign);
          b1:=byte(-z);
       end
     else
       b1:=z;
     if n<0 then
       begin
          sign:=not(sign);
          b2:=byte(-n);
       end
     else
       b2:=n;

     { the div is coded by the compiler as call to divdword }
     if sign then
       result:=-(b1 div b2)
     else
       result:=b1 div b2;
  end;
{$endif FPC_SYSTEM_HAS_DIV_SHORTINT}


{$ifndef FPC_SYSTEM_HAS_MOD_SHORTINT}
function fpc_mod_shortint(n,z : shortint) : shortint; [public,alias: 'FPC_MOD_SHORTINT']; compilerproc;
  var
     signed : boolean;
     r,nq,zq : byte;
  begin
     if n=0 then
       HandleErrorAddrFrameInd(200,get_pc_addr,get_frame);
     nq:=abs(n);

     if z<0 then
       begin
          zq:=byte(-z);
          signed:=true;
       end
     else
       begin
         zq:=z;
         signed:=false;
       end;

     r:=zq mod nq;
     if signed then
       result:=-shortint(r)
     else
       result:=r;
  end;
{$endif FPC_SYSTEM_HAS_MOD_SHORTINT}

{$endif FPC_INCLUDE_SOFTWARE_MOD_DIV}

{****************************************************************************}

{$if defined(CPUINT8)}
{$ifndef FPC_SYSTEM_HAS_ABS_SHORTINT}
function abs(l:shortint):shortint;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   if l<0 then
     abs:=-l
   else
     abs:=l;
end;
{$endif not FPC_SYSTEM_HAS_ABS_SMALLINT}
{$endif CPUINT8}

{$if defined(CPUINT16) or defined(CPUINT8)}
{$ifndef FPC_SYSTEM_HAS_ABS_SMALLINT}
function abs(l:smallint):smallint;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   if l<0 then
     abs:=-l
   else
     abs:=l;
end;
{$endif not FPC_SYSTEM_HAS_ABS_SMALLINT}
{$endif CPUINT16 or CPUINT8}

{$ifndef FPC_SYSTEM_HAS_ABS_LONGINT}
{ This is only needed to bootstrap on SPARC targets
  (MIPS and m68k too, but they have no releases, so bootstrapping is not an issue) }
function abs(l:longint):longint;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   if l<0 then
     abs:=-l
   else
     abs:=l;
end;

{$endif not FPC_SYSTEM_HAS_ABS_LONGINT}

{$if defined(CPUINT8)}
{$ifndef FPC_SYSTEM_HAS_ODD_SHORTINT}
function odd(l:shortint):Boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(l and 1);
end;
{$endif ndef FPC_SYSTEM_HAS_ODD_SHORTINT}
{$ifndef FPC_SYSTEM_HAS_ODD_BYTE}
function odd(l:byte):Boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(l and 1);
end;
{$endif ndef FPC_SYSTEM_HAS_ODD_BYTE}
{$endif CPUINT8}

{$if defined(CPUINT16) or defined(CPUINT8)}
{$ifndef FPC_SYSTEM_HAS_ODD_SMALLINT}
function odd(l:smallint):Boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(l and 1);
end;
{$endif ndef FPC_SYSTEM_HAS_ODD_SMALLINT}
{$ifndef FPC_SYSTEM_HAS_ODD_WORD}
function odd(l:word):Boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(l and 1);
end;
{$endif ndef FPC_SYSTEM_HAS_ODD_WORD}
{$endif CPUINT16 or CPUINT8}

{$ifndef FPC_SYSTEM_HAS_ODD_LONGINT}

function odd(l:longint):boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(l and 1);
end;

{$endif ndef FPC_SYSTEM_HAS_ODD_LONGINT}

{$ifndef FPC_SYSTEM_HAS_ODD_LONGWORD}

function odd(l:longword):boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(l and 1);
end;

{$endif ndef FPC_SYSTEM_HAS_ODD_LONGWORD}


{$ifndef FPC_SYSTEM_HAS_ODD_INT64}

function odd(l:int64):boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(longint(l) and 1);
end;

{$endif ndef FPC_SYSTEM_HAS_ODD_INT64}

{$ifndef FPC_SYSTEM_HAS_ODD_QWORD}

function odd(l:qword):boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   odd:=boolean(longint(l) and 1);
end;

{$endif ndef FPC_SYSTEM_HAS_ODD_QWORD}

{$if defined(CPUINT8)}
{$ifndef FPC_SYSTEM_HAS_SQR_SHORTINT}
function sqr(l:shortint):shortint;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   sqr:=l*l;
end;
{$endif ndef FPC_SYSTEM_HAS_SQR_SHORTINT}
{$endif CPUINT8}

{$if defined(CPUINT16) or defined(CPUINT8)}
{$ifndef FPC_SYSTEM_HAS_SQR_SMALLINT}
function sqr(l:smallint):smallint;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   sqr:=l*l;
end;
{$endif ndef FPC_SYSTEM_HAS_SQR_SMALLINT}
{$endif CPUINT16 or CPUINT8}

{$ifndef FPC_SYSTEM_HAS_SQR_LONGINT}

function sqr(l:longint):longint;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
   sqr:=l*l;
end;

{$endif ndef FPC_SYSTEM_HAS_SQR_LONGINT}


{$ifndef FPC_SYSTEM_HAS_ABS_INT64}

function abs(l: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  if l < 0 then
    abs := -l
  else
    abs := l;
end;

{$endif ndef FPC_SYSTEM_HAS_ABS_INT64}


{$ifndef FPC_SYSTEM_HAS_SQR_INT64}

function sqr(l: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  sqr := l*l;
end;

{$endif ndef FPC_SYSTEM_HAS_SQR_INT64}


{$ifndef FPC_SYSTEM_HAS_SQR_QWORD}

function sqr(l: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  sqr := l*l;
end;

{$endif ndef FPC_SYSTEM_HAS_SQR_INT64}

{$ifdef CPU16}
{$ifndef FPC_SYSTEM_HAS_DECLOCKED_SMALLINT}
function declocked(var l:smallint):boolean;
  begin
    Dec(l);
    declocked:=(l=0);
  end;
{$endif FPC_SYSTEM_HAS_DECLOCKED_SMALLINT}
{$endif CPU16}

{$ifndef FPC_SYSTEM_HAS_DECLOCKED_LONGINT}
function declocked(var l:longint):boolean;
  begin
    Dec(l);
    declocked:=(l=0);
  end;
{$endif FPC_SYSTEM_HAS_DECLOCKED_LONGINT}


{$ifndef FPC_SYSTEM_HAS_DECLOCKED_INT64}
function declocked(var l:int64):boolean;
  begin
    Dec(l);
    declocked:=(l=0);
  end;
{$endif FPC_SYSTEM_HAS_DECLOCKED_INT64}


{$ifdef CPU16}
{$ifndef FPC_SYSTEM_HAS_INCLOCKED_SMALLINT}
procedure inclocked(var l:smallint);
  begin
    Inc(l);
  end;
{$endif FPC_SYSTEM_HAS_INCLOCKED_SMALLINT}
{$endif CPU16}

{$ifndef FPC_SYSTEM_HAS_INCLOCKED_LONGINT}
procedure inclocked(var l:longint);
  begin
    Inc(l);
  end;
{$endif FPC_SYSTEM_HAS_INCLOCKED_LONGINT}


{$ifndef FPC_SYSTEM_HAS_INCLOCKED_INT64}
procedure inclocked(var l:int64);
  begin
    Inc(l);
  end;
{$endif FPC_SYSTEM_HAS_INCLOCKED_INT64}


{$ifndef FPC_SYSTEM_HAS_SPTR}
{_$error Sptr must be defined for each processor }
{$endif ndef FPC_SYSTEM_HAS_SPTR}

{****************************************************************************
                                 Str()
****************************************************************************}

{$ifndef FPC_SYSTEM_HAS_INT_STR_LONGINT}

procedure int_str(l:longint;out s:shortstring);
var
  m,m1 : longword;
  pcstart,
  pc2start,
  pc,pc2 : pchar;
  hs : string[32];
  overflow : longint;
begin
  pc2start:=@s[1];
  pc2:=pc2start;
  if (l<0) then
    begin
      pc2^:='-';
      inc(pc2);
      m:=longword(-l);
    end
  else
    m:=longword(l);
  pcstart:=pchar(@hs[0]);
  pc:=pcstart;
  repeat
    m1:=m div 10;
    inc(pc);
    pc^:=char(m-(m1*10)+byte('0'));
    m:=m1;
  until m=0;
  overflow:=(pc-pcstart)+(pc2-pc2start)-high(s);
  if overflow>0 then
    inc(pcstart,overflow);
  while (pc>pcstart) do
    begin
      pc2^:=pc^;
      inc(pc2);
      dec(pc);
    end;
  s[0]:=char(pc2-pc2start);
end;

{$endif ndef FPC_SYSTEM_HAS_INT_STR_LONGINT}

{$ifndef FPC_SYSTEM_HAS_INT_STR_LONGWORD}

procedure int_str_unsigned(l:longword;out s:shortstring);
var
  m1 : longword;
  pcstart,
  pc2start,
  pc,pc2 : pchar;
  hs : string[32];
  overflow : longint;
begin
  pc2start:=@s[1];
  pc2:=pc2start;
  pcstart:=pchar(@hs[0]);
  pc:=pcstart;
  repeat
    inc(pc);
    m1:=l div 10;
    pc^:=char(l-(m1*10)+byte('0'));
    l:=m1;
  until l=0;
  overflow:=(pc-pcstart)-high(s);
  if overflow>0 then
    inc(pcstart,overflow);
  while (pc>pcstart) do
    begin
      pc2^:=pc^;
      inc(pc2);
      dec(pc);
    end;
  s[0]:=char(pc2-pc2start);
end;

{$endif ndef FPC_SYSTEM_HAS_INT_STR_LONGWORD}

{$ifndef FPC_SYSTEM_HAS_INT_STR_INT64}

procedure int_str(l:int64;out s:shortstring);
{$ifdef EXCLUDE_COMPLEX_PROCS}
begin
  runerror(217);
end;
{$else EXCLUDE_COMPLEX_PROCS}
var
  m,m1 : qword;
  pcstart,
  pc2start,
  pc,pc2 : pchar;
  hs : string[32];
  overflow : longint;
begin
  pc2start:=@s[1];
  pc2:=pc2start;
  if (l<0) then
    begin
      pc2^:='-';
      inc(pc2);
      m:=qword(-l);
    end
  else
    m:=qword(l);
  pcstart:=pchar(@hs[0]);
  pc:=pcstart;
  repeat
    m1:=m div 10;
    inc(pc);
    pc^:=char(m-(m1*10)+byte('0'));
    m:=m1;
  until m=0;
  overflow:=(pc-pcstart)+(pc2-pc2start)-high(s);
  if overflow>0 then
    inc(pcstart,overflow);
  while (pc>pcstart) do
    begin
      pc2^:=pc^;
      inc(pc2);
      dec(pc);
    end;
  s[0]:=char(pc2-pc2start);
end;
{$endif EXCLUDE_COMPLEX_PROCS}

{$endif ndef FPC_SYSTEM_HAS_INT_STR_INT64}

{$ifndef FPC_SYSTEM_HAS_INT_STR_QWORD}

procedure int_str_unsigned(l:qword;out s:shortstring);
{$ifdef EXCLUDE_COMPLEX_PROCS}
begin
  runerror(217);
end;
{$else EXCLUDE_COMPLEX_PROCS}
var
  m1 : qword;
  pcstart,
  pc2start,
  pc,pc2 : pchar;
  hs : string[64];
  overflow : longint;
begin
  pc2start:=@s[1];
  pc2:=pc2start;
  pcstart:=pchar(@hs[0]);
  pc:=pcstart;
  repeat
    inc(pc);
    m1:=l div 10;
    pc^:=char(l-(m1*10)+byte('0'));
    l:=m1;
  until l=0;
  overflow:=(pc-pcstart)-high(s);
  if overflow>0 then
    inc(pcstart,overflow);
  while (pc>pcstart) do
    begin
      pc2^:=pc^;
      inc(pc2);
      dec(pc);
    end;
  s[0]:=char(pc2-pc2start);
end;
{$endif EXCLUDE_COMPLEX_PROCS}

{$endif ndef FPC_SYSTEM_HAS_INT_STR_QWORD}

{$ifndef FPUNONE}
{$ifndef FPC_SYSTEM_HAS_SYSRESETFPU}

procedure SysResetFpu;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  softfloat_exception_flags:=[];
end;

{$endif FPC_SYSTEM_HAS_SYSRESETFPU}

{$ifndef FPC_SYSTEM_HAS_SYSINITFPU}

procedure SysInitFpu;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
  softfloat_exception_mask:=[float_flag_underflow,float_flag_inexact,float_flag_denormal];
end;

{$endif FPC_SYSTEM_HAS_SYSINITFPU}
{$endif}

{$ifndef FPC_SYSTEM_HAS_SWAPENDIAN}
function SwapEndian(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    { the extra Word type cast is necessary because the "AValue shr 8" }
    { is turned into "longint(AValue) shr 8", so if AValue < 0 then    }
    { the sign bits from the upper 16 bits are shifted in rather than  }
    { zeroes.                                                          }
    Result := SmallInt((Word(AValue) shr 8) or (Word(AValue) shl 8));
  end;

{$ifndef cpujvm}
function SwapEndian(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result := Word((AValue shr 8) or (AValue shl 8));
  end;
{$endif}

function SwapEndian(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result := ((AValue shl 8) and $FF00FF00) or ((AValue shr 8) and $00FF00FF);
    Result := (Result shl 16) or (Result shr 16);
  end;

{$ifndef cpujvm}
function SwapEndian(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result := ((AValue shl 8) and $FF00FF00) or ((AValue shr 8) and $00FF00FF);
    Result := (Result shl 16) or (Result shr 16);
  end;
{$endif}

function SwapEndian(const AValue: Int64): Int64;
  begin
    Result := ((AValue shl 8) and $FF00FF00FF00FF00) or
            ((AValue shr 8) and $00FF00FF00FF00FF);
    Result := ((Result shl 16) and $FFFF0000FFFF0000) or
            ((Result shr 16) and $0000FFFF0000FFFF);
    Result := (Result shl 32) or ((Result shr 32));
  end;

{$ifndef cpujvm}
function SwapEndian(const AValue: QWord): QWord;
  begin
    Result := ((AValue shl 8) and $FF00FF00FF00FF00) or
            ((AValue shr 8) and $00FF00FF00FF00FF);
    Result := ((Result shl 16) and $FFFF0000FFFF0000) or
            ((Result shr 16) and $0000FFFF0000FFFF);
    Result := (Result shl 32) or ((Result shr 32));
  end;
{$endif}
{$endif FPC_SYSTEM_HAS_SWAPENDIAN}

function BEtoN(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function BEtoN(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function BEtoN(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function BEtoN(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function BEtoN(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function BEtoN(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}



function LEtoN(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function LEtoN(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function LEtoN(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function LEtoN(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function LEtoN(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function LEtoN(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}



function NtoBE(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function NtoBE(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function NtoBE(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function NtoBE(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function NtoBE(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function NtoBE(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_BIG}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function NtoLE(const AValue: SmallInt): SmallInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function NtoLE(const AValue: Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function NtoLE(const AValue: LongInt): LongInt;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function NtoLE(const AValue: DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}


function NtoLE(const AValue: Int64): Int64;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;


{$ifndef cpujvm}
function NtoLE(const AValue: QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    {$IFDEF ENDIAN_LITTLE}
      Result := AValue;
    {$ELSE}
      Result := SwapEndian(AValue);
    {$ENDIF}
  end;
{$endif not cpujvm}

{$ifndef FPC_SYSTEM_HAS_MEM_BARRIER}

procedure ReadBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
end;

procedure ReadDependencyBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
end;

procedure ReadWriteBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
end;

procedure WriteBarrier;{$ifdef SYSTEMINLINE}inline;{$endif}
begin
end;

{$endif FPC_SYSTEM_HAS_MEM_BARRIER}

{$ifndef FPC_HAS_INTERNAL_ROX_BYTE}
{$ifndef FPC_SYSTEM_HAS_ROX_BYTE}

function RorByte(Const AValue : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr 1) or (AValue shl 7);
  end;


function RorByte(Const AValue : Byte;const Dist : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr (Dist and 7)) or (AValue shl (8-(Dist and 7)));
  end;


function RolByte(Const AValue : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl 1) or (AValue shr 7);
  end;


function RolByte(Const AValue : Byte;const Dist : Byte): Byte;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl (Dist and 7)) or (AValue shr (8-(Dist and 7)));
  end;

{$endif FPC_SYSTEM_HAS_ROX_BYTE}
{$endif FPC_HAS_INTERNAL_ROX_BYTE}

{$ifndef FPC_HAS_INTERNAL_ROX_WORD}
{$ifndef FPC_SYSTEM_HAS_ROX_WORD}

function RorWord(Const AValue : Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr 1) or (AValue shl 15);
  end;


function RorWord(Const AValue : Word;const Dist : Byte): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr (Dist and 15)) or (AValue shl (16-(Dist and 15)));
  end;


function RolWord(Const AValue : Word): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl 1) or (AValue shr 15);
  end;


function RolWord(Const AValue : Word;const Dist : Byte): Word;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl (Dist and 15)) or (AValue shr (16-(Dist and 15)));
  end;

{$endif FPC_SYSTEM_HAS_ROX_WORD}
{$endif FPC_HAS_INTERNAL_ROX_WORD}

{$ifndef FPC_HAS_INTERNAL_ROX_DWORD}
{$ifndef FPC_SYSTEM_HAS_ROX_DWORD}

function RorDWord(Const AValue : DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr 1) or (AValue shl 31);
  end;


function RorDWord(Const AValue : DWord;const Dist : Byte): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr (Dist and 31)) or (AValue shl (32-(Dist and 31)));
  end;


function RolDWord(Const AValue : DWord): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl 1) or (AValue shr 31);
  end;


function RolDWord(Const AValue : DWord;const Dist : Byte): DWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl (Dist and 31)) or (AValue shr (32-(Dist and 31)));
  end;

{$endif FPC_SYSTEM_HAS_ROX_DWORD}
{$endif FPC_HAS_INTERNAL_ROX_DWORD}

{$ifndef FPC_HAS_INTERNAL_ROX_QWORD}
{$ifndef FPC_SYSTEM_HAS_ROX_QWORD}

function RorQWord(Const AValue : QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr 1) or (AValue shl 63);
  end;


function RorQWord(Const AValue : QWord;const Dist : Byte): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shr (Dist and 63)) or (AValue shl (64-(Dist and 63)));
  end;


function RolQWord(Const AValue : QWord): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl 1) or (AValue shr 63);
  end;


function RolQWord(Const AValue : QWord;const Dist : Byte): QWord;{$ifdef SYSTEMINLINE}inline;{$endif}
  begin
    Result:=(AValue shl (Dist and 63)) or (AValue shr (64-(Dist and 63)));
  end;

{$endif FPC_SYSTEM_HAS_ROX_QWORD}
{$endif FPC_HAS_INTERNAL_ROX_QWORD}

{$ifndef FPC_HAS_INTERNAL_ROX_ASSIGN_QWORD}
{$ifndef FPC_SYSTEM_HAS_ROX_ASSIGN_QWORD}
procedure fpc_ror_assign_int64(var AValue : int64;const Dist : Byte); [Public,Alias:'FPC_ROR_ASSIGN_INT64']; compilerproc;
  begin
    AValue:=(AValue shr (Dist and 63)) or (AValue shl (64-(Dist and 63)));
  end;


procedure fpc_ror_assign_qword(var AValue : QWord;const Dist : Byte); [Public,Alias:'FPC_ROR_ASSIGN_QWORD']; compilerproc;
  begin
    AValue:=(AValue shr (Dist and 63)) or (AValue shl (64-(Dist and 63)));
  end;


procedure fpc_rol_assign_int64(var AValue : int64;const Dist : Byte); [Public,Alias:'FPC_ROL_ASSIGN_INT64']; compilerproc;
  begin
    AValue:=(AValue shl (Dist and 63)) or (AValue shr (64-(Dist and 63)));
  end;


procedure fpc_rol_assign_qword(var AValue : QWord;const Dist : Byte); [Public,Alias:'FPC_ROL_ASSIGN_QWORD']; compilerproc;
  begin
    AValue:=(AValue shl (Dist and 63)) or (AValue shr (64-(Dist and 63)));
  end;
{$endif FPC_SYSTEM_HAS_ROX_ASSIGN_QWORD}
{$endif FPC_HAS_INTERNAL_ROX_ASSIGN_QWORD}

{$ifndef FPC_HAS_INTERNAL_SAR_BYTE}
{$ifndef FPC_SYSTEM_HAS_SAR_BYTE}
function SarShortint(Const AValue : Shortint;const Shift : Byte): Shortint;
  begin
    Result:=shortint(byte(byte(byte(AValue) shr (Shift and 7)) or (byte(shortint(byte(0-byte(byte(AValue) shr 7)) and byte(shortint(0-(ord((Shift and 7)<>0){ and 1}))))) shl (8-(Shift and 7)))));
  end;
{$endif FPC_HAS_INTERNAL_SAR_BYTE}
{$endif FPC_SYSTEM_HAS_SAR_BYTE}

{$ifndef FPC_HAS_INTERNAL_SAR_WORD}
{$ifndef FPC_SYSTEM_HAS_SAR_WORD}
function SarSmallint(Const AValue : Smallint;const Shift : Byte): Smallint;
  begin
    Result:=smallint(word(word(word(AValue) shr (Shift and 15)) or (word(smallint(word(0-word(word(AValue) shr 15)) and word(smallint(0-(ord((Shift and 15)<>0){ and 1}))))) shl (16-(Shift and 15)))));
  end;
{$endif FPC_HAS_INTERNAL_SAR_WORD}
{$endif FPC_SYSTEM_HAS_SAR_WORD}

{$ifndef FPC_HAS_INTERNAL_SAR_DWORD}
{$ifndef FPC_SYSTEM_HAS_SAR_DWORD}
function SarLongint(Const AValue : Longint;const Shift : Byte): Longint;
  begin
    Result:=longint(dword(dword(dword(AValue) shr (Shift and 31)) or (dword(longint(dword(0-dword(dword(AValue) shr 31)) and dword(longint(0-(ord((Shift and 31)<>0){ and 1}))))) shl (32-(Shift and 31)))));
  end;
{$endif FPC_HAS_INTERNAL_SAR_DWORD}
{$endif FPC_SYSTEM_HAS_SAR_DWORD}

{$ifndef FPC_HAS_INTERNAL_SAR_QWORD}
{$ifndef FPC_SYSTEM_HAS_SAR_QWORD}
function fpc_SarInt64(Const AValue : Int64;const Shift : Byte): Int64; [Public,Alias:'FPC_SARINT64']; compilerproc;
  begin
    Result:=int64(qword(qword(qword(AValue) shr (Shift and 63)) or (qword(int64(qword(0-qword(qword(AValue) shr 63)) and qword(int64(0-(ord((Shift and 63)<>0){ and 1}))))) shl (64-(Shift and 63)))));
  end;
{$endif FPC_HAS_INTERNAL_SAR_QWORD}
{$endif FPC_SYSTEM_HAS_SAR_QWORD}

{$ifndef FPC_HAS_INTERNAL_SAR_ASSIGN_QWORD}
{$ifndef FPC_SYSTEM_HAS_SAR_ASSIGN_QWORD}
procedure fpc_sar_assign_int64(var AValue : Int64;const Shift : Byte); [Public,Alias:'FPC_SAR_ASSIGN_INT64']; compilerproc;
  begin
    AValue:=int64(qword(qword(qword(AValue) shr (Shift and 63)) or (qword(int64(qword(0-qword(qword(AValue) shr 63)) and qword(int64(0-(ord((Shift and 63)<>0){ and 1}))))) shl (64-(Shift and 63)))));
  end;
procedure fpc_sar_assign_qword(var AValue : QWord;const Shift : Byte); [Public,Alias:'FPC_SAR_ASSIGN_QWORD']; compilerproc;
  begin
    AValue:=qword(qword(qword(qword(AValue) shr (Shift and 63)) or (qword(int64(qword(0-qword(qword(AValue) shr 63)) and qword(int64(0-(ord((Shift and 63)<>0){ and 1}))))) shl (64-(Shift and 63)))));
  end;
{$endif FPC_HAS_INTERNAL_SAR_ASSIGN_QWORD}
{$endif FPC_SYSTEM_HAS_SAR_ASSIGN_QWORD}

{$ifndef FPC_HAS_INTERNAL_BSF_BYTE}
{$ifndef FPC_SYSTEM_HAS_BSF_BYTE}
function BsfByte(Const AValue: Byte): Byte;
  const bsf8bit: array [Byte] of Byte = (
	  $ff,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,
	  5,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,
	  6,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,
	  5,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,
	  7,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,
	  5,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,
	  6,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,
	  5,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0,4,0,1,0,2,0,1,0,3,0,1,0,2,0,1,0
  );
  begin
    result:=bsf8bit[AValue];
  end;
{$endif}
{$endif}

{$ifndef FPC_HAS_INTERNAL_BSR_BYTE}
{$ifndef FPC_SYSTEM_HAS_BSR_BYTE}
function BsrByte(Const AValue: Byte): Byte;
  const bsr8bit: array [Byte] of Byte = (
    $ff,0,1,1,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
	  5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
	  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	  6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
	  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7
  );
  begin
    result:=bsr8bit[AValue];
  end;
{$endif}
{$endif}

{$ifndef FPC_SYSTEM_HAS_BSF_WORD}
{$ifndef FPC_HAS_INTERNAL_BSF_WORD}
function BsfWord(Const AValue: Word): {$ifdef CPU16}byte{$else}cardinal{$endif};
  begin
    result:=ord(lo(AValue)=0)*8;
    result:=result or BsfByte(byte(AValue shr result));
  end;
{$endif}
{$endif}

{$ifndef FPC_SYSTEM_HAS_BSR_WORD}
{$ifndef FPC_HAS_INTERNAL_BSR_WORD}
function BsrWord(Const AValue: Word): {$ifdef CPU16}byte{$else}cardinal{$endif};
  begin
    result:=ord(AValue>255)*8;
    result:=result or BsrByte(byte(AValue shr result));
  end;
{$endif}
{$endif}

{$ifndef FPC_HAS_INTERNAL_BSF_DWORD}
{$ifndef FPC_SYSTEM_HAS_BSF_DWORD}
function BsfDWord(Const AValue : DWord): {$ifdef CPU16}byte{$else}cardinal{$endif};
  var
    tmp: DWord;
  begin
    result:=ord(lo(AValue)=0)*16;
    tmp:=AValue shr result;
    result:=result or (ord((tmp and $FF)=0)*8);
    tmp:=tmp shr (result and 8);
    result:=result or BsfByte(byte(tmp));
  end;
{$endif}
{$endif}

{$ifndef FPC_HAS_INTERNAL_BSR_DWORD}
{$ifndef FPC_SYSTEM_HAS_BSR_DWORD}
function BsrDWord(Const AValue : DWord): {$ifdef CPU16}byte{$else}cardinal{$endif};
  var
    tmp: DWord;
  begin
    result:=ord(AValue>$FFFF)*16;
    tmp:=AValue shr result;
    result:=result or (ord(tmp>$FF)*8);
    tmp:=tmp shr (result and 8);
    result:=result or BsrByte(byte(tmp));
  end;
{$endif}
{$endif}

{$ifndef FPC_HAS_INTERNAL_BSF_QWORD}
{$ifndef FPC_SYSTEM_HAS_BSF_QWORD}
function BsfQWord(Const AValue : QWord): {$ifdef CPU16}byte{$else}cardinal{$endif};
  var
    tmp: DWord;
  begin
    result:=0;
    tmp:=lo(AValue);
    if (tmp=0) then
      begin
        tmp:=hi(AValue);
        result:=32;
      end;
    result:=result or BsfDword(tmp);
  end;
{$endif}
{$endif}

{$ifndef FPC_HAS_INTERNAL_BSR_QWORD}
{$ifndef FPC_SYSTEM_HAS_BSR_QWORD}
function BsrQWord(Const AValue : QWord): {$ifdef CPU16}byte{$else}cardinal{$endif};
  var
    tmp: DWord;
  begin
    result:=32;
    tmp:=hi(AValue);
    if (tmp=0) then
      begin
        tmp:=lo(AValue);
        result:=0;
      end;
    result:=result or BsrDword(tmp);
  end;
{$endif}
{$endif}

const
  PopCntData : array[0..15] of byte = (0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4);

function fpc_PopCnt_byte(AValue : Byte): Byte;[Public,Alias:'FPC_POPCNT_BYTE'];compilerproc;
  begin
    Result:=PopCntData[AValue and $f]+PopCntData[(AValue shr 4) and $f];
  end;


function fpc_PopCnt_word(AValue : Word): Word;[Public,Alias:'FPC_POPCNT_WORD'];compilerproc;
  var
    i : SizeInt;
  begin
    Result:=0;
    for i:=0 to 3 do
      begin
        inc(Result,PopCntData[AValue and $f]);
        AValue:=AValue shr 4;
      end;
  end;


function fpc_PopCnt_dword(AValue : DWord): DWord;[Public,Alias:'FPC_POPCNT_DWORD'];compilerproc;
  var
    i : SizeInt;
  begin
    Result:=0;
    for i:=0 to 7 do
      begin
        inc(Result,PopCntData[AValue and $f]);
        AValue:=AValue shr 4;
      end;
  end;


{$ifndef FPC_SYSTEM_HAS_POPCNT_QWORD}
function fpc_PopCnt_qword(AValue : QWord): QWord;[Public,Alias:'FPC_POPCNT_QWORD'];compilerproc;
{$ifdef VER2_6}
  var
    i : SizeInt;
  begin
    Result:=0;
    for i:=0 to 15 do
      begin
        inc(Result,PopCntData[AValue and $f]);
        AValue:=AValue shr 4;
      end;
{$else VER2_6}
  begin
    Result:=PopCnt(lo(AValue))+PopCnt(hi(AValue))
{$endif VER2_6}
  end;
{$endif}

