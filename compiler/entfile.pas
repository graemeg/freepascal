{
    Copyright (c) 1998-2013 by Free Pascal development team

    Routines to read/write entry based files (ppu, pcp)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}
unit entfile;

{$i fpcdefs.inc}

interface

  uses
    systems,globtype,constexp,cstreams;

const
{ buffer sizes }
  maxentrysize = 1024;
  // Unused, and wrong as there are entries that are larger then 1024 bytes

  entryfilebufsize   = 16384;

{ppu entries}
  mainentryid         = 1;
  subentryid          = 2;
  {special}
  iberror             = 0;
  ibextraheader       = 242;
  ibpputable          = 243;
  ibstartrequireds    = 244;
  ibendrequireds      = 245;
  ibstartcontained    = 246;
  ibendcontained      = 247;
  ibstartdefs         = 248;
  ibenddefs           = 249;
  ibstartsyms         = 250;
  ibendsyms           = 251;
  ibendinterface      = 252;
  ibendimplementation = 253;
//  ibendbrowser        = 254;
  ibend               = 255;
  {general}
  ibmodulename           = 1;
  ibsourcefiles          = 2;
  ibloadunit             = 3;
  ibinitunit             = 4;
  iblinkunitofiles       = 5;
  iblinkunitstaticlibs   = 6;
  iblinkunitsharedlibs   = 7;
  iblinkotherofiles      = 8;
  iblinkotherstaticlibs  = 9;
  iblinkothersharedlibs  = 10;
  ibImportSymbols        = 11;
  ibsymref               = 12;
  ibdefref               = 13;
  ibfeatures             = 14;
{$IFDEF MACRO_DIFF_HINT}
  ibusedmacros           = 16;
{$ENDIF}
  ibderefdata            = 17;
  ibexportedmacros       = 18;
  ibderefmap             = 19;

  {syms}
  ibtypesym        = 20;
  ibprocsym        = 21;
  ibstaticvarsym   = 22;
  ibconstsym       = 23;
  ibenumsym        = 24;
//  ibtypedconstsym  = 25;
  ibabsolutevarsym = 26;
  ibpropertysym    = 27;
  ibfieldvarsym    = 28;
  ibunitsym        = 29;
  iblabelsym       = 30;
  ibsyssym         = 31;
  ibnamespacesym   = 32;
  iblocalvarsym    = 33;
  ibparavarsym     = 34;
  ibmacrosym       = 35;
  {definitions}
  iborddef         = 40;
  ibpointerdef     = 41;
  ibarraydef       = 42;
  ibprocdef        = 43;
  ibshortstringdef = 44;
  ibrecorddef      = 45;
  ibfiledef        = 46;
  ibformaldef      = 47;
  ibobjectdef      = 48;
  ibenumdef        = 49;
  ibsetdef         = 50;
  ibprocvardef     = 51;
  ibfloatdef       = 52;
  ibclassrefdef    = 53;
  iblongstringdef  = 54;
  ibansistringdef  = 55;
  ibwidestringdef  = 56;
  ibvariantdef     = 57;
  ibundefineddef   = 58;
  ibunicodestringdef = 59;
  {implementation/ObjData}
  ibnodetree       = 80;
  ibasmsymbols     = 81;
  ibresources      = 82;
  ibcreatedobjtypes = 83;
  ibwpofile         = 84;
  ibmoduleoptions   = 85;
  ibunitimportsyms  = 86;
  iborderedsymbols  = 87;

  ibmainname       = 90;
  ibsymtableoptions = 91;
  ibpackagefiles   = 92;
  ibpackagename    = 93;
  ibrecsymtableoptions = 94;
  { target-specific things }
  iblinkotherframeworks = 100;
  ibjvmnamespace = 101;

{$ifdef generic_cpu}
{ We need to use the correct size of aint and pint for
  the target CPU }
const
  CpuAddrBitSize : array[tsystemcpu] of longint =
    (
    {  0 } 32 {'none'},
    {  1 } 32 {'i386'},
    {  2 } 32 {'m68k'},
    {  3 } 32 {'alpha'},
    {  4 } 32 {'powerpc'},
    {  5 } 32 {'sparc'},
    {  6 } 32 {'vis'},
    {  7 } 64 {'ia64'},
    {  8 } 64 {'x86_64'},
    {  9 } 32 {'mipseb'},
    { 10 } 32 {'arm'},
    { 11 } 64 {'powerpc64'},
    { 12 } 16 {'avr'},
    { 13 } 32 {'mipsel'},
    { 14 } 32 {'jvm'},
    { 15 } 16 {'i8086'},
    { 16 } 64 {'aarch64'},
    { 17 } 32 {'wasm'},
    { 18 } 64 {'sparc64'},
    { 19 } 32 {'riscv32'},
    { 20 } 64 {'riscv64'}
    );
  CpuAluBitSize : array[tsystemcpu] of longint =
    (
    {  0 } 32 {'none'},
    {  1 } 32 {'i386'},
    {  2 } 32 {'m68k'},
    {  3 } 32 {'alpha'},
    {  4 } 32 {'powerpc'},
    {  5 } 32 {'sparc'},
    {  6 } 32 {'vis'},
    {  7 } 64 {'ia64'},
    {  8 } 64 {'x86_64'},
    {  9 } 32 {'mipseb'},
    { 10 } 32 {'arm'},
    { 11 } 64 {'powerpc64'},
    { 12 }  8 {'avr'},
    { 13 } 32 {'mipsel'},
    { 14 } 64 {'jvm'},
    { 15 } 16 {'i8086'},
    { 16 } 64 {'aarch64'},
    { 17 } 64 {'wasm'},
    { 18 } 64 {'sparc64'},
    { 19 } 32 {'riscv32'},
    { 20 } 64 {'riscv64'}
    );
{$endif generic_cpu}

type
  { bestreal is defined based on the target architecture }
  entryreal=bestreal;

  { common part of the header for all kinds of entry files }
  tentryheader=record
    id       : array[1..3] of char;
    ver      : array[1..3] of char;
    compiler : word;
    cpu      : word;
    target   : word;
    flags    : dword;
    size     : dword; { size of the ppufile without header }
  end;
  pentryheader=^tentryheader;

  tentry=packed record
    size : longint;
    id   : byte;
    nr   : byte;
  end;

  tentryfile=class
  private
    function getposition:longint;
    procedure setposition(value:longint);
  protected
    buf      : pchar;
    bufstart,
    bufsize,
    bufidx   : integer;
    entrybufstart,
    entrystart,
    entryidx : integer;
    entry    : tentry;
    closed,
    tempclosed : boolean;
    closepos : integer;
  protected
    f        : TCStream;
{$ifdef DEBUG_PPU}
    flog     : text;
    flog_open : boolean;
    ppu_log_level : longint;
    ppu_log_idx : integer;
{$endif}
    mode     : byte; {0 - Closed, 1 - Reading, 2 - Writing}
    fisfile  : boolean;
    fname    : string;
    fsize    : integer;
    procedure newheader;virtual;abstract;
    function readheader:longint;virtual;abstract;
    function outputallowed:boolean;virtual;
    procedure resetfile;virtual;abstract;
    function getheadersize:longint;virtual;abstract;
    function getheaderaddr:pentryheader;virtual;abstract;
    procedure RaiseAssertion(Code: Longint); virtual;
  public
    entrytyp : byte;
    size             : integer;
    change_endian    : boolean; { Used in ppudump util }
{$ifdef generic_cpu}
    has_more,
{$endif not generic_cpu}
    error         : boolean;
    constructor create(const fn:string);
    destructor  destroy;override;
    function getversion:integer;
    procedure flush; {$ifdef USEINLINE}inline;{$endif}
    procedure closefile;virtual;
    procedure newentry;
    property position:longint read getposition write setposition;
    { Warning: don't keep the stream open during a tempclose! }
    function substream(ofs,len:longint):TCStream;
    { Warning: don't use the put* or write* functions anymore when writing through this }
    property stream:TCStream read f;
{$ifdef DEBUG_PPU}
    procedure ppu_log(st :string);virtual;
    procedure ppu_log_val(st :string);virtual;
    procedure inc_log_level;
    procedure dec_log_level;
{$endif}
  {read}
    function  openfile:boolean;
    function  openstream(strm:TCStream):boolean;
    procedure reloadbuf;
    procedure readdata(out b;len:integer);
    procedure skipdata(len:integer);
    function  readentry:byte;
    function  EndOfEntry:boolean; {$ifdef USEINLINE}inline;{$endif}
    function  entrysize:longint; {$ifdef USEINLINE}inline;{$endif}
    function  entryleft:longint; {$ifdef USEINLINE}inline;{$endif}
    procedure getdatabuf(out b;len:integer;out res:integer);
    procedure getdata(out b;len:integer);
    function  getbyte:byte;
    function  getword:word;
    function  getdword:dword;
    function  getlongint:longint;
    function getint64:int64;
    function  getqword:qword;
    function getaint:{$ifdef generic_cpu}int64{$else}aint{$ifdef USEINLINE}; inline{$endif}{$endif};
    function getasizeint:{$ifdef generic_cpu}int64{$else}asizeint{$ifdef USEINLINE}; inline{$endif}{$endif};
    function getpuint:{$ifdef generic_cpu}qword{$else}puint{$ifdef USEINLINE}; inline{$endif}{$endif};
    function getptruint:{$ifdef generic_cpu}qword{$else}TConstPtrUInt{$ifdef USEINLINE}; inline{$endif}{$endif};
    function getaword:{$ifdef generic_cpu}qword{$else}aword{$ifdef USEINLINE}; inline{$endif}{$endif};
    function  getreal:entryreal;
    function  getrealsize(sizeofreal : longint):entryreal;
    function  getboolean:boolean; {$ifdef USEINLINE}inline;{$endif}
    function  getstring:string;
    function  getpshortstring:pshortstring;
    function  getansistring:ansistring;
    procedure getset(out arr: array of byte);
    function  skipuntilentry(untilb:byte):boolean;
  {write}
    function  createfile:boolean;virtual;
    function  createstream(strm:TCStream):boolean;
    procedure writeheader;virtual;abstract;
    procedure writebuf;
    procedure writedata(const b;len:integer);
    procedure writeentry(ibnr:byte);
    procedure putdata(const b;len:integer);virtual;
    procedure putbyte(b:byte); {$ifdef USEINLINE}inline;{$endif}
    procedure putword(w:word); {$ifdef USEINLINE}inline;{$endif}
    procedure putdword(w:dword); {$ifdef USEINLINE}inline;{$endif}
    procedure putlongint(l:longint); {$ifdef USEINLINE}inline;{$endif}
    procedure putint64(i:int64); {$ifdef USEINLINE}inline;{$endif}
    procedure putqword(q:qword); {$ifdef USEINLINE}inline;{$endif}
    procedure putaint(i:aint); {$ifdef USEINLINE}inline;{$endif}
    procedure putasizeint(i:asizeint); {$ifdef USEINLINE}inline;{$endif}
    procedure putpuint(i:puint); {$ifdef USEINLINE}inline;{$endif}
    procedure putptruint(v:TConstPtrUInt); {$ifdef USEINLINE}inline;{$endif}
    procedure putaword(i:aword); {$ifdef USEINLINE}inline;{$endif}
    procedure putreal(d:entryreal);
    procedure putboolean(b:boolean); {$ifdef USEINLINE}inline;{$endif}
    procedure putstring(const s:string); {$ifdef USEINLINE}inline;{$endif}
    procedure putansistring(const s:ansistring);

    procedure putset(const arr: array of byte); {$ifdef USEINLINE}inline;{$endif}
    procedure tempclose;        // MG: not used, obsolete?
    function  tempopen:boolean; // MG: not used, obsolete?
  end;

implementation

  uses
    cutils;


function swapendian_entryreal(d:entryreal):entryreal;
type
  entryreal_bytes=array[0..sizeof(d)-1] of byte;
var
  i:0..sizeof(d)-1;
begin
  for i:=low(entryreal_bytes) to high(entryreal_bytes) do
    entryreal_bytes(result)[i]:=entryreal_bytes(d)[high(entryreal_bytes)-i];
end;

{*****************************************************************************
                              tentryfile
*****************************************************************************}

function tentryfile.outputallowed: boolean;
begin
  result:=true;
end;


constructor tentryfile.create(const fn:string);
begin
  fname:=fn;
  fisfile:=false;
  change_endian:=false;
  mode:=0;
  newheader;
  error:=false;
  closed:=true;
  tempclosed:=false;
  getmem(buf,entryfilebufsize);
{$ifdef DEBUG_PPU}
  assign(flog,fn+'.debug-log');
  flog_open:=false;
{$endif DEBUG_PPU}
end;


destructor tentryfile.destroy;
begin
  closefile;
  if assigned(buf) then
    freemem(buf,entryfilebufsize);
end;

{$ifdef DEBUG_PPU}

function entryid_name(nr : byte) : string;
begin
  case nr of
  {ppu entries}
  mainentryid: entryid_name:='main_entry_id';
  subentryid: entryid_name:='sub_entry_id';
  else
    entryid_name:='unknown entryid '+tostr(nr);
  end;
end;

function entry_name(nr : byte) : string;
begin
  case nr of
  {special}
  iberror: entry_name:='iberror';
  ibextraheader: entry_name:='ibextraheader';
  ibpputable: entry_name:='ibpputable';
  ibstartrequireds: entry_name:='ibstartrequireds';
  ibendrequireds: entry_name:='ibendrequireds';
  ibstartcontained: entry_name:='ibstartcontained';
  ibendcontained: entry_name:='ibendcontained';
  ibstartdefs: entry_name:='ibstartdefs';
  ibenddefs: entry_name:='ibenddefs';
  ibstartsyms: entry_name:='ibstartsyms';
  ibendsyms: entry_name:='ibendsyms';
  ibendinterface: entry_name:='ibendinterface';
  ibendimplementation: entry_name:='ibendimplementation';
  // ibendbrowser: entry_name:='ibendbrowser';
  ibend: entry_name:='ibend';
  {general}
  ibmodulename: entry_name:='ibmodulename';
  ibsourcefiles: entry_name:='ibsourcefiles';
  ibloadunit: entry_name:='ibloadunit';
  ibinitunit: entry_name:='ibinitunit';
  iblinkunitofiles: entry_name:='iblinkunitofiles';
  iblinkunitstaticlibs: entry_name:='iblinkunitstaticlibs';
  iblinkunitsharedlibs: entry_name:='iblinkunitsharedlibs';
  iblinkotherofiles: entry_name:='iblinkotherofiles';
  iblinkotherstaticlibs: entry_name:='iblinkotherstaticlibs';
  iblinkothersharedlibs: entry_name:='iblinkothersharedlibs';
  ibImportSymbols: entry_name:='ibImportSymbols';
  ibsymref: entry_name:='ibsymref';
  ibdefref: entry_name:='ibdefref';
  ibfeatures: entry_name:='ibfeatures';
{$IFDEF MACRO_DIFF_HINT}
  ibusedmacros: entry_name:='ibusedmacros';
{$ENDIF}
  ibderefdata: entry_name:='ibderefdata';
  ibexportedmacros: entry_name:='ibexportedmacros';
  ibderefmap: entry_name:='ibderefmap';

  {syms}
  ibtypesym: entry_name:='ibtypesym';
  ibprocsym: entry_name:='ibprocsym';
  ibstaticvarsym: entry_name:='ibstaticvarsym';
  ibconstsym: entry_name:='ibconstsym';
  ibenumsym: entry_name:='ibenumsym';
  // ibtypedconstsym: entry_name:='ibtypedconstsym';
  ibabsolutevarsym: entry_name:='ibabsolutevarsym';
  ibpropertysym: entry_name:='ibpropertysym';
  ibfieldvarsym: entry_name:='ibfieldvarsym';
  ibunitsym: entry_name:='ibunitsym';
  iblabelsym: entry_name:='iblabelsym';
  ibsyssym: entry_name:='ibsyssym';
  ibnamespacesym: entry_name:='ibnamespacesym';
  iblocalvarsym: entry_name:='iblocalvarsym';
  ibparavarsym: entry_name:='ibparavarsym';
  ibmacrosym: entry_name:='ibmacrosym';
  {definitions}
  iborddef: entry_name:='iborddef';
  ibpointerdef: entry_name:='ibpointerdef';
  ibarraydef: entry_name:='ibarraydef';
  ibprocdef: entry_name:='ibprocdef';
  ibshortstringdef: entry_name:='ibshortstringdef';
  ibrecorddef: entry_name:='ibrecorddef';
  ibfiledef: entry_name:='ibfiledef';
  ibformaldef: entry_name:='ibformaldef';
  ibobjectdef: entry_name:='ibobjectdef';
  ibenumdef: entry_name:='ibenumdef';
  ibsetdef: entry_name:='ibsetdef';
  ibprocvardef: entry_name:='ibprocvardef';
  ibfloatdef: entry_name:='ibfloatdef';
  ibclassrefdef: entry_name:='ibclassrefdef';
  iblongstringdef: entry_name:='iblongstringdef';
  ibansistringdef: entry_name:='ibansistringdef';
  ibwidestringdef: entry_name:='ibwidestringdef';
  ibvariantdef: entry_name:='ibvariantdef';
  ibundefineddef: entry_name:='ibundefineddef';
  ibunicodestringdef: entry_name:='ibunicodestringdef';
  {implementation/ObjData}
  ibnodetree: entry_name:='ibnodetree';
  ibasmsymbols: entry_name:='ibasmsymbols';
  ibresources: entry_name:='ibresources';
  ibcreatedobjtypes: entry_name:='ibcreatedobjtypes';
  ibwpofile: entry_name:='ibwpofile';
  ibmoduleoptions: entry_name:='ibmoduleoptions';
  ibunitimportsyms: entry_name:='ibunitimportsyms';
  iborderedsymbols: entry_name:='iborderedsymbols';

  ibmainname: entry_name:='ibmainname';
  ibsymtableoptions: entry_name:='ibsymtableoptions';
  // ibrecsymtableoptions: entry_name:='ibrecsymtableoptions';
  ibpackagefiles: entry_name:='ibpackagefiles';
  ibpackagename: entry_name:='ibpackagename';
  { target-specific things }
  iblinkotherframeworks: entry_name:='iblinkotherframeworks';
  ibjvmnamespace: entry_name:='ibjvmnamespace';
  else
    entry_name:='unknown entry '+tostr(nr);
  end;
end;

procedure tentryfile.ppu_log(st :string);
begin
  if flog_open then
    begin
      writeln(flog,bufstart+bufidx,': ',st);
    end;
{$ifdef IN_PPUDUMP}
  writeln(bufstart+bufidx,': ',st);
{$endif}
end;

procedure tentryfile.inc_log_level;
begin
  inc(ppu_log_level);
end;

procedure tentryfile.ppu_log_val(st :string);
begin
  if flog_open then
    begin
      writeln(flog,'(',ppu_log_level,') value: ',st);
    end;
{$ifdef IN_PPUDUMP}
  writeln('(',ppu_log_level,') value: ',st);
{$endif}
end;

procedure tentryfile.dec_log_level;
begin
  dec(ppu_log_level);
end;
{$endif}

function tentryfile.getversion:integer;
  var
    l    : integer;
    code : integer;
    header : pentryheader;
  begin
    header:=getheaderaddr;
    Val(header^.ver[1]+header^.ver[2]+header^.ver[3],l,code);
    if code=0 then
     result:=l
    else
     result:=0;
  end;

procedure tentryfile.flush;
begin
  if mode=2 then
   writebuf;
end;


procedure tentryfile.RaiseAssertion(Code: Longint);
begin
  { It's down to descendent classes to raise an internal error as desired. [Kit] }
  error := true;
end;


procedure tentryfile.closefile;
begin
  if mode<>0 then
   begin
     flush;
{$ifdef DEBUG_PPU}
     if (entry.nr<>0) and (mode=1) then
       ppu_log('writeentry, id='+entryid_name(entry.id)+' nr='+entry_name(entry.nr)+' size='+tostr(entry.size));
{$endif}
     if fisfile then
       f.Free;
     mode:=0;
     closed:=true;
{$ifdef DEBUG_PPU}
     if flog_open then
       close(flog);
     flog_open:=false;
{$endif DEBUG_PPU}
   end;
end;


procedure tentryfile.setposition(value:longint);
begin
  if assigned(f) then
    f.Position:=value
  else
    if tempclosed then
      closepos:=value;
end;


function tentryfile.getposition:longint;
begin
  if assigned(f) then
    result:=f.Position
  else
    if tempclosed then
      result:=closepos
    else
      result:=0;
end;


function tentryfile.substream(ofs,len:longint):TCStream;
begin
  result:=nil;
  if assigned(f) then
    result:=TCRangeStream.Create(f,ofs,len);
end;

{*****************************************************************************
                              tentryfile Reading
*****************************************************************************}

function tentryfile.openfile:boolean;
var
  strm : TCStream;
begin
  openfile:=false;
  try
    strm:=CFileStreamClass.Create(fname,fmOpenRead)
  except
    exit;
  end;
  openfile:=openstream(strm);
  fisfile:=result;
end;


function tentryfile.openstream(strm:TCStream):boolean;
var
  i : longint;
begin
  openstream:=false;
  f:=strm;
  closed:=false;
{$ifdef DEBUG_PPU}
  {$push}
  {$I-}
  assign(flog,fname+'.debug-read-log');
  rewrite(flog);
  if InOutRes=0 then
    flog_open:=true;
  {$pop}
{$endif DEBUG_PPU}
{read ppuheader}
  fsize:=f.Size;
  i:=readheader;
  if i<0 then
    exit;
{reset buffer}
  bufstart:=i;
  bufsize:=0;
  bufidx:=0;
  mode:=1;
  FillChar(entry,sizeof(tentry),0);
  entryidx:=0;
  entrystart:=0;
  entrybufstart:=0;
  error:=false;
  openstream:=true;
end;


procedure tentryfile.reloadbuf;
begin
  inc(bufstart,bufsize);
  bufsize:=f.Read(buf^,entryfilebufsize);
  bufidx:=0;
end;


procedure tentryfile.readdata(out b;len:integer);
var
  p,pbuf : pchar;
  left : integer;
{$ifdef DEBUG_PPU}
  i : integer;
{$endif DEBUG_PPU}
begin
  p:=pchar(@b);
  pbuf:=@buf[bufidx];
{$ifdef DEBUG_PPU}
  if ppu_log_level <= 0 then
    begin
      ppu_log('writedata, length='+tostr(len)+' level='+tostr(ppu_log_level));
      for i:=0 to len-1 do
        ppu_log_val('p['+tostr(i)+']=$'+hexstr(byte(p[i]),2));
    end;
{$endif DEBUG_PPU}
  repeat
    left:=bufsize-bufidx;
    if len<left then
      break;
    move(pbuf^,p^,left);
    dec(len,left);
    inc(p,left);
    reloadbuf;
    pbuf:=@buf[bufidx];
    if bufsize=0 then
      exit;
  until false;
  move(pbuf^,p^,len);
  inc(bufidx,len);
end;


procedure tentryfile.skipdata(len:integer);
var
  left : integer;
begin
{$ifdef DEBUG_PPU}
  if len>0 then
    ppu_log('explicit skipdata '+tostr(len));
{$endif}
  while len>0 do
   begin
     left:=bufsize-bufidx;
     if len>left then
      begin
        dec(len,left);
        reloadbuf;
        if bufsize=0 then
         exit;
      end
     else
      begin
        inc(bufidx,len);
        exit;
      end;
   end;
end;


function tentryfile.readentry:byte;
begin
  if entryidx<entry.size then
    begin
{$ifdef generic_cpu}
     has_more:=true;
{$endif not generic_cpu}
{$ifdef DEBUG_PPU}
     if entry.size-entryidx>0 then
       ppu_log('skipdata '+tostr(entry.size-entryidx));
{$endif}
     skipdata(entry.size-entryidx);
    end;
{$ifdef DEBUG_PPU}
  if entry.nr<>0 then
    ppu_log('writeentry, id='+entryid_name(entry.id)+' nr='+entry_name(entry.nr)+' size='+tostr(entry.size));
  ppu_log('entrystart');
{$endif}
  readdata(entry,sizeof(tentry));
  if change_endian then
    entry.size:=swapendian(entry.size);
  entrystart:=bufstart+bufidx;
  entryidx:=0;
{$ifdef generic_cpu}
  has_more:=false;
{$endif not generic_cpu}
  if not(entry.id in [mainentryid,subentryid]) then
   begin
     readentry:=iberror;
     error:=true;
     exit;
   end;
  readentry:=entry.nr;
end;


function tentryfile.endofentry:boolean;
begin
{$ifdef generic_cpu}
  endofentry:=(entryidx=entry.size);
{$else not generic_cpu}
  endofentry:=(entryidx>=entry.size);
{$endif not generic_cpu}
end;


function tentryfile.entrysize:longint;
begin
  entrysize:=entry.size;
end;

function tentryfile.entryleft:longint;
begin
  entryleft:=entry.size-entryidx;
end;


procedure tentryfile.getdatabuf(out b;len:integer;out res:integer);
begin
  if entryidx+len>entry.size then
   res:=entry.size-entryidx
  else
   res:=len;
  readdata(b,res);
  inc(entryidx,res);
end;


procedure tentryfile.getdata(out b;len:integer);
begin
  if entryidx+len>entry.size then
   begin
     error:=true;
     exit;
   end;
  readdata(b,len);
  inc(entryidx,len);
end;


function tentryfile.getbyte:byte;
begin
  if entryidx>=entry.size then
   begin
     error:=true;
     result:=0;
     exit;
   end;
{$ifdef DEBUG_PPU}
  ppu_log('putbyte');
  inc_log_level;
{$endif}
  if bufidx<bufsize then
    begin
      result:=pbyte(@buf[bufidx])^;
      inc(bufidx);
    end
  else
    readdata(result,1);
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
  inc(entryidx);
end;


function tentryfile.getword:word;
begin
  if entryidx+2>entry.size then
   begin
     error:=true;
     result:=0;
     exit;
   end;
{$ifdef DEBUG_PPU}
  ppu_log('putword');
  inc_log_level;
{$endif}
  if bufsize-bufidx>=sizeof(word) then
    begin
      result:=Unaligned(pword(@buf[bufidx])^);
      inc(bufidx,sizeof(word));
    end
  else
    readdata(result,sizeof(word));
  if change_endian then
   result:=swapendian(result);
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
  inc(entryidx,2);
end;


function tentryfile.getlongint:longint;
begin
  if entryidx+4>entry.size then
   begin
     error:=true;
     result:=0;
     exit;
   end;
{$ifdef DEBUG_PPU}
  ppu_log('putlongint');
  inc_log_level;
{$endif}
  if bufsize-bufidx>=sizeof(longint) then
    begin
      result:=Unaligned(plongint(@buf[bufidx])^);
      inc(bufidx,sizeof(longint));
    end
  else
    readdata(result,sizeof(longint));
  if change_endian then
   result:=swapendian(result);
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
  inc(entryidx,4);
end;


function tentryfile.getdword:dword;
begin
  if entryidx+4>entry.size then
   begin
     error:=true;
     result:=0;
     exit;
   end;
{$ifdef DEBUG_PPU}
  ppu_log('putdword');
  inc_log_level;
{$endif}
  if bufsize-bufidx>=sizeof(dword) then
    begin
      result:=Unaligned(pdword(@buf[bufidx])^);
      inc(bufidx,sizeof(longint));
    end
  else
    readdata(result,sizeof(dword));
  if change_endian then
   result:=swapendian(result);
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
  inc(entryidx,4);
end;


function tentryfile.getint64:int64;
begin
  if entryidx+8>entry.size then
   begin
     error:=true;
     result:=0;
     exit;
   end;
{$ifdef DEBUG_PPU}
  ppu_log('putint64');
  inc_log_level;
{$endif}
  if bufsize-bufidx>=sizeof(int64) then
    begin
      result:=Unaligned(pint64(@buf[bufidx])^);
      inc(bufidx,sizeof(int64));
    end
  else
    readdata(result,sizeof(int64));
  if change_endian then
   result:=swapendian(result);
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
  inc(entryidx,8);
end;


function tentryfile.getqword:qword;
begin
  if entryidx+8>entry.size then
   begin
     error:=true;
     result:=0;
     exit;
   end;
{$ifdef DEBUG_PPU}
  ppu_log('putqword');
  inc_log_level;
{$endif}
  if bufsize-bufidx>=sizeof(qword) then
    begin
      result:=Unaligned(pqword(@buf[bufidx])^);
      inc(bufidx,sizeof(qword));
    end
  else
    readdata(result,sizeof(qword));
  if change_endian then
   result:=swapendian(result);
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
  inc(entryidx,8);
end;


function tentryfile.getaint:{$ifdef generic_cpu}int64{$else}aint{$endif};
{$ifdef generic_cpu}
var
  header : pentryheader;
{$endif generic_cpu}
begin
{$ifdef DEBUG_PPU}
  ppu_log('putaint');
  inc_log_level;
{$endif}
{$ifdef generic_cpu}
  header:=getheaderaddr;
  if CpuAluBitSize[tsystemcpu(header^.cpu)]=64 then
    result:=getint64
  else if CpuAluBitSize[tsystemcpu(header^.cpu)]=32 then
    result:=getlongint
  else if CpuAluBitSize[tsystemcpu(header^.cpu)]=16 then
    result:=smallint(getword)
  else if CpuAluBitSize[tsystemcpu(header^.cpu)]=8 then
    result:=shortint(getbyte)
  else
    begin
      error:=true;
      result:=0;
    end;
{$else not generic_cpu}
  case sizeof(aint) of
    8: result:=getint64;
    4: result:=getlongint;
    2: result:=smallint(getword);
    1: result:=shortint(getbyte);
  else
    begin
      RaiseAssertion(2019041801);
      result:=0;
    end;
  end;
{$endif not generic_cpu}
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
end;


function tentryfile.getasizeint:{$ifdef generic_cpu}int64{$else}asizeint{$endif};
{$ifdef generic_cpu}
var
  header : pentryheader;
{$endif generic_cpu}
begin
{$ifdef DEBUG_PPU}
  ppu_log('putasizeint');
  inc_log_level;
{$endif}
{$ifdef generic_cpu}
  header:=getheaderaddr;
  if CpuAddrBitSize[tsystemcpu(header^.cpu)]=64 then
    result:=getint64
  else if CpuAddrBitSize[tsystemcpu(header^.cpu)]=32 then
    result:=getlongint
  else if CpuAddrBitSize[tsystemcpu(header^.cpu)]=16 then
    begin
      { result:=smallint(getword);
        would have been logical, but it contradicts
	definition of asizeint in globtype unit,
	which uses 32-bit lngint type even for 16-bit
	address size, to be able to cope with
	I8086 seg:ofs huge addresses }
      result:=getlongint;
    end
  else
    begin
      error:=true;
      result:=0;
    end;
{$else not generic_cpu}
  case sizeof(asizeint) of
    8: result:=asizeint(getint64);
    4: result:=asizeint(getlongint);
    2: result:=asizeint(getword);
    1: result:=asizeint(getbyte);
  else
    begin
      RaiseAssertion(2019041802);
      result:=0;
    end;
  end;
{$endif not generic_cpu}
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
end;


function tentryfile.getpuint:{$ifdef generic_cpu}qword{$else}puint{$endif};
{$ifdef generic_cpu}
var
header : pentryheader;
{$endif generic_cpu}
begin
{$ifdef DEBUG_PPU}
  ppu_log('putpuint');
  inc_log_level;
{$endif}
{$ifdef generic_cpu}
  header:=getheaderaddr;
  if CpuAddrBitSize[tsystemcpu(header^.cpu)]=64 then
    result:=getqword
  else if CpuAddrBitSize[tsystemcpu(header^.cpu)]=32 then
    result:=getdword
  else if CpuAddrBitSize[tsystemcpu(header^.cpu)]=16 then
    result:=getword
  else
    begin
      error:=true;
      result:=0;
    end;
{$else not generic_cpu}
  case sizeof(puint) of
    8: result:=getqword;
    4: result:=getdword;
    2: result:=getword;
    1: result:=getbyte;
  else
    begin
      RaiseAssertion(2019041803);
      result:=0;
    end;
  end;
{$endif not generic_cpu}
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
end;


function tentryfile.getptruint:{$ifdef generic_cpu}qword{$else}TConstPtrUInt{$endif};
{$ifdef generic_cpu}
var
header : pentryheader;
{$endif generic_cpu}
begin
{$ifdef DEBUG_PPU}
  ppu_log('putptruint');
  inc_log_level;
{$endif}
{$ifdef generic_cpu}
  header:=getheaderaddr;
  if CpuAddrBitSize[tsystemcpu(header^.cpu)]=64 then
    result:=getqword
  else result:=getdword;
{$else not generic_cpu}
  {$if sizeof(TConstPtrUInt)=8}
  result:=tconstptruint(getint64);
  {$else}
  result:=TConstPtrUInt(getlongint);
  {$endif}
{$endif not generic_cpu}
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
end;


function tentryfile.getaword:{$ifdef generic_cpu}qword{$else}aword{$endif};
{$ifdef generic_cpu}
var
header : pentryheader;
{$endif generic_cpu}
begin
{$ifdef DEBUG_PPU}
  ppu_log('putaword');
  inc_log_level;
{$endif}
{$ifdef generic_cpu}
  header:=getheaderaddr;
  if CpuAluBitSize[tsystemcpu(header^.cpu)]=64 then
    result:=getqword
  else if CpuAluBitSize[tsystemcpu(header^.cpu)]=32 then
    result:=getdword
  else if CpuAluBitSize[tsystemcpu(header^.cpu)]=16 then
    result:=getword
  else if CpuAluBitSize[tsystemcpu(header^.cpu)]=8 then
    result:=getbyte
  else
    begin
      error:=true;
      result:=0;
    end;
{$else not generic_cpu}
  case sizeof(aword) of
    8: result:=getqword;
    4: result:=getdword;
    2: result:=getword;
    1: result:=getbyte;
  else
    begin
      RaiseAssertion(2019041804);
      result:=0;
    end;
  end;
{$endif not generic_cpu}
{$ifdef DEBUG_PPU}
  ppu_log_val(tostr(result));
  dec_log_level;
{$endif}
end;

function tentryfile.getrealsize(sizeofreal : longint):entryreal;
var
  e : entryreal;
  d : double;
  di : qword;{ integer of same size as double }
  s : single;
  si : dword; { integer of same size as single }
begin
  if sizeofreal=sizeof(e) then
    begin
{$ifdef DEBUG_PPU}
      ppu_log('putreal,size='+tostr(sizeof(e)));
      inc_log_level;
{$endif}
      if entryidx+sizeof(e)>entry.size then
       begin
         error:=true;
         result:=0;
         exit;
       end;
      readdata(e,sizeof(e));
      if change_endian then
        result:=swapendian_entryreal(e)
      else
        result:=e;
      inc(entryidx,sizeof(e));
{$ifdef DEBUG_PPU}
      ppu_log_val(realtostr(result));
      dec_log_level;
{$endif}
      exit;
    end;
  if sizeofreal=sizeof(d) then
    begin
{$ifdef DEBUG_PPU}
      ppu_log('putreal,size='+tostr(sizeof(d)));
      inc_log_level;
{$endif}
      if entryidx+sizeof(d)>entry.size then
       begin
         error:=true;
         result:=0;
         exit;
       end;
      readdata(d,sizeof(d));
      if change_endian then
        begin
          di:=swapendian(pqword(@d)^);
          d:=pdouble(@di)^;
        end;
      result:=d;
      inc(entryidx,sizeof(d));
      result:=d;
{$ifdef DEBUG_PPU}
      ppu_log_val(realtostr(result));
      dec_log_level;
{$endif}
      exit;
    end;
  if sizeofreal=sizeof(s) then
    begin
{$ifdef DEBUG_PPU}
      ppu_log('putreal,size='+tostr(sizeof(s)));
      inc_log_level;
{$endif}
      if entryidx+sizeof(s)>entry.size then
       begin
         error:=true;
         result:=0;
         exit;
       end;
      readdata(s,sizeof(s));
      if change_endian then
        begin
          si:=swapendian(pdword(@s)^);
          s:=psingle(@si)^;
        end;
      result:=s;
      inc(entryidx,sizeof(s));
      result:=s;
{$ifdef DEBUG_PPU}
      ppu_log_val(realtostr(result));
      dec_log_level;
{$endif}
      exit;
    end;
  error:=true;
  result:=0.0;
end;


function tentryfile.getreal:entryreal;
var
  d : entryreal;
  hd : double;
begin
  if target_info.system=system_x86_64_win64 then
    begin
      hd:=getrealsize(sizeof(hd));
      getreal:=hd;
    end
  else
    begin
      d:=getrealsize(sizeof(d));
      getreal:=d;
    end;
end;


function tentryfile.getboolean:boolean;
begin
{$ifdef DEBUG_PPU}
  ppu_log('putboolean');
{$endif}
  result:=boolean(getbyte);
end;


function tentryfile.getstring:string;
begin
  result[0]:=chr(getbyte);
{$ifdef DEBUG_PPU}
  ppu_log('putstring,size='+tostr(length(result)+1));
  inc_log_level;
{$endif}
  if entryidx+length(result)>entry.size then
   begin
     error:=true;
     exit;
   end;
  ReadData(result[1],length(result));
{$ifdef DEBUG_PPU}
  ppu_log_val(result);
  dec_log_level;
{$endif}
  inc(entryidx,length(result));
end;

function tentryfile.getpshortstring:pshortstring;
var
  len: char;
begin
  result:=nil;
  len:=chr(getbyte);
{$ifdef DEBUG_PPU}
  ppu_log('putstring,size='+tostr(ord(len)+1));
  inc_log_level;
{$endif}
  if entryidx+ord(len)>entry.size then
   begin
     error:=true;
     exit;
   end;
  getmem(result,ord(len)+1);
  result^[0]:=len;
  ReadData(result^[1],ord(len));
  inc(entryidx,ord(len));
{$ifdef DEBUG_PPU}
  ppu_log_val(result^);
  dec_log_level;
{$endif}
end;

function tentryfile.getansistring:ansistring;
var
  len: longint;
begin
{$ifdef DEBUG_PPU}
  ppu_log('putansistring');
  inc_log_level;
{$endif}
  len:=getlongint;
  if entryidx+len>entry.size then
   begin
     error:=true;
     result:='';
     exit;
   end;
  setlength(result,len);
  if len>0 then
    getdata(result[1],len);
{$ifdef DEBUG_PPU}
  ppu_log_val(result);
  dec_log_level;
{$endif}
end;


procedure tentryfile.getset(out arr: array of byte);
var
  i : longint;
begin
{$ifdef DEBUG_PPU}
  ppu_log('putset');
  inc_log_level;
{$endif}
  getdata(arr,sizeof(arr));
  if change_endian then
    for i:=low(arr) to high(arr) do
      arr[i]:=reverse_byte(arr[i]);
{$ifdef DEBUG_PPU}
  for i:=0 to 3 do
    ppu_log_val('byte['+tostr(i)+']=$'+hexstr(arr[i],2));
  dec_log_level;
{$endif}
end;


function tentryfile.skipuntilentry(untilb:byte):boolean;
var
  b : byte;
begin
{$ifdef DEBUG_PPU}
  ppu_log('skipuntilentry '+tostr(untilb));
{$endif}
  repeat
    b:=readentry;
  until (b in [ibend,iberror]) or ((b=untilb) and (entry.id=mainentryid));
  skipuntilentry:=(b=untilb);
end;


{*****************************************************************************
                              tentryfile Writing
*****************************************************************************}

function tentryfile.createfile:boolean;
var
  ok: boolean;
  strm : TCStream;
begin
  createfile:=false;
  strm:=nil;
  if outputallowed then
    begin
      {$ifdef MACOS}
      {FPas is FreePascal's creator code on MacOS. See systems/mac_crea.txt}
      SetDefaultMacOSCreator('FPas');
      SetDefaultMacOSFiletype('FPPU');
      {$endif}
      ok:=false;
      try
        strm:=CFileStreamClass.Create(fname,fmCreate);
        ok:=true;
      except
      end;
      {$ifdef MACOS}
      SetDefaultMacOSCreator('MPS ');
      SetDefaultMacOSFiletype('TEXT');
      {$endif}
      if not ok then
       exit;
    end;
  createfile:=createstream(strm);
  fisfile:=result;
end;

function tentryfile.createstream(strm:TCStream):boolean;
begin
  createstream:=false;
  if outputallowed then
    begin
      f:=strm;
      mode:=2;
      {write header for sure}
      f.Write(getheaderaddr^,getheadersize);
    end;
  bufsize:=entryfilebufsize;
  bufstart:=getheadersize;
  bufidx:=0;
{reset}
  resetfile;
  error:=false;
  size:=0;
  entrytyp:=mainentryid;
{$ifdef DEBUG_PPU}
  {$push}
  {$I-}
  assign(flog,fname+'.debug-write-log');
  rewrite(flog);
  if InOutRes=0 then
    flog_open:=true;
  {$pop}
{$endif DEBUG_PPU}
{start}
  newentry;
  createstream:=true;
end;


procedure tentryfile.writebuf;
begin
  if outputallowed and
     (bufidx <> 0) then
    f.Write(buf^,bufidx);
  inc(bufstart,bufidx);
  bufidx:=0;
end;


procedure tentryfile.writedata(const b;len:integer);
var
  p   : pchar;
  left,
  idx : integer;
{$ifdef DEBUG_PPU}
  start_len : integer;
{$endif}
begin
  if not outputallowed then
    exit;
{$ifdef DEBUG_PPU}
  start_len:=len;
{$endif}
  p:=pchar(@b);
  idx:=0;
  while len>0 do
   begin
     left:=bufsize-bufidx;
     if len>left then
      begin
        move(p[idx],buf[bufidx],left);
        dec(len,left);
        inc(idx,left);
        inc(bufidx,left);
        writebuf;
      end
     else
      begin
        move(p[idx],buf[bufidx],len);
        inc(bufidx,len);
{$ifdef DEBUG_PPU}
        len:=0;
{$else}
        exit;
{$endif}
      end;
   end;
{$ifdef DEBUG_PPU}
  if (start_len > 0) and (ppu_log_level <= 0) then
    begin
      ppu_log('writedata, length='+tostr(start_len)+' level='+tostr(ppu_log_level));
      for idx:=0 to start_len-1 do
        ppu_log_val('p['+tostr(idx)+']=$'+hexstr(byte(p[idx]),2));
    end;
{$endif DEBUG_PPU}
end;


procedure tentryfile.newentry;
begin
  with entry do
   begin
     id:=entrytyp;
     nr:=ibend;
     size:=0;
   end;
{Reset Entry State}
  entryidx:=0;
  entrybufstart:=bufstart;
  entrystart:=bufstart+bufidx;
{$ifdef DEBUG_PPU}
  ppu_log('entrystart');
{$endif}
{Alloc in buffer}
  writedata(entry,sizeof(tentry));
end;


procedure tentryfile.writeentry(ibnr:byte);
var
  opos : integer;
begin
{create entry}
  entry.id:=entrytyp;
  entry.nr:=ibnr;
  entry.size:=entryidx;
{it's already been sent to disk ?}
  if entrybufstart<>bufstart then
   begin
    if outputallowed then
      begin
      {flush to be sure}
        WriteBuf;
      {write entry}
        opos:=f.Position;
        f.Position:=entrystart;
        f.write(entry,sizeof(tentry));
        f.Position:=opos;
      end;
     entrybufstart:=bufstart;
   end
  else
   move(entry,buf[entrystart-bufstart],sizeof(entry));
{$ifdef DEBUG_PPU}
  ppu_log('writeentry, id='+entryid_name(entry.id)+' nr='+entry_name(entry.nr)+' size='+tostr(entry.size));
{$endif}
{Add New Entry, which is ibend by default}
  entrystart:=bufstart+bufidx; {next entry position}
  newentry;
end;


procedure tentryfile.putdata(const b;len:integer);
begin
  if outputallowed then
    writedata(b,len);
  inc(entryidx,len);
end;


procedure tentryfile.putbyte(b:byte);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putbyte');
  inc_log_level;
  ppu_log_val(tostr(b));
{$endif}
  putdata(b,1);
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putword(w:word);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putword');
  inc_log_level;
  ppu_log_val(tostr(w));
{$endif}
  putdata(w,2);
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putdword(w:dword);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putdword');
  inc_log_level;
  ppu_log_val(tostr(w));
{$endif}
  putdata(w,4);
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putlongint(l:longint);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putlongint');
  inc_log_level;
  ppu_log_val(tostr(l));
{$endif}
  putdata(l,4);
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putint64(i:int64);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putint64');
  inc_log_level;
  ppu_log_val(tostr(i));
{$endif}
  putdata(i,8);
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putqword(q:qword);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putqword');
  inc_log_level;
  ppu_log_val(tostr(q));
{$endif}
  putdata(q,sizeof(qword));
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putaint(i:aint);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putaint');
  inc_log_level;
  case sizeof(aint) of
    8: ppu_log('putint64');
    4: ppu_log('putlongint');
    2: ppu_log('putword');
    1: ppu_log('putbyte');
  end;
  ppu_log_val(tostr(i));
{$endif}
  putdata(i,sizeof(aint));
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putasizeint(i: asizeint);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putasizeint');
  inc_log_level;
  case sizeof(asizeint) of
    8: ppu_log('putint64');
    4: ppu_log('putlongint');
    2: ppu_log('putword');
    1: ppu_log('putbyte');
  end;
  ppu_log_val(tostr(i));
{$endif}
  putdata(i,sizeof(asizeint));
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putpuint(i : puint);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putpuint');
  inc_log_level;
  ppu_log_val(tostr(i));
{$endif}
  putdata(i,sizeof(puint));
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;

procedure tentryfile.putptruint(v:TConstPtrUInt);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putptruint');
  inc_log_level;
{$endif}
  {$if sizeof(TConstPtrUInt)=8}
  putint64(int64(v));
  {$else}
  putlongint(longint(v));
  {$endif}
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;




procedure tentryfile.putaword(i:aword);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putaword');
  inc_log_level;
  ppu_log_val(tostr(i));
{$endif}
  putdata(i,sizeof(aword));
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putreal(d:entryreal);
var
  hd : double;
begin
  if target_info.system=system_x86_64_win64 then
    begin
{$ifdef DEBUG_PPU}
      ppu_log('putreal,size='+tostr(sizeof(hd)));
      inc_log_level;
      ppu_log_val(realtostr(d));
{$endif}
      hd:=d;
      putdata(hd,sizeof(hd));
    end
  else
    begin
{$ifdef DEBUG_PPU}
      ppu_log('putreal,size='+tostr(sizeof(d)));
      inc_log_level;
      ppu_log_val(realtostr(d));
{$endif}
      putdata(d,sizeof(entryreal));
    end;
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putboolean(b:boolean);
begin
{$ifdef DEBUG_PPU}
  ppu_log('putboolean');
  inc_log_level;
{$endif}
  putbyte(byte(b));
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putstring(const s:string);
begin
{$ifdef DEBUG_PPU}
  { The reading method uses getbyte, so fake it here }
  ppu_log('putbyte');
  inc_log_level;
  inc(bufidx);
  ppu_log('putstring,size='+tostr(length(s)+1));
  dec(bufidx);
  ppu_log_val(s);
{$endif}
  putdata(s,length(s)+1);
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
end;


procedure tentryfile.putansistring(const s:ansistring);
  var
    len: longint;
  begin
    len:=length(s);
{$ifdef DEBUG_PPU}
  ppu_log('putansistring');
  inc_log_level;
  ppu_log_val(s);
{$endif}
    putlongint(len);
    if len>0 then
      putdata(s[1],len);
{$ifdef DEBUG_PPU}
  dec_log_level;
{$endif}
  end;


procedure tentryfile.putset(const arr: array of byte);
{$ifdef DEBUG_PPU}
  var
    i : byte;
{$endif}
  begin
{$ifdef DEBUG_PPU}
  ppu_log('putset');
  inc_log_level;
{$endif}
    putdata(arr,sizeof(arr));
{$ifdef DEBUG_PPU}
  for i:=0 to 31 do
    ppu_log_val('byte['+tostr(i)+']=$'+hexstr(arr[i],2));
  dec_log_level;
{$endif}
  end;


procedure tentryfile.tempclose;
  begin
    if not closed then
     begin
       closepos:=f.Position;
       f.Free;
       f:=nil;
       closed:=true;
       tempclosed:=true;
     end;
  end;


function tentryfile.tempopen:boolean;
  begin
    tempopen:=false;
    if not closed or not tempclosed then
     exit;
   { MG: not sure, if this is correct
     f.position:=0;
       No, f was freed in tempclose above, we need to
       recreate it.  PM 2011/06/06 }
    try
      f:=CFileStreamClass.Create(fname,fmOpenRead);
    except
      exit;
    end;
    closed:=false;
    tempclosed:=false;

  { restore state }
    f.Position:=closepos;
    tempopen:=true;
  end;

end.
