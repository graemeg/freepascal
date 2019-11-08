{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by Peter Vreman

    Executable file reading functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
  This unit should not be compiled in objfpc mode, since this would make it
  dependent on objpas unit.
}

{ Disable checks of pointers explictly,
  as we are dealing here with special pointer that
  might be seen as invalid by heaptrc unit CheckPointer function }

{$checkpointer off}

unit exeinfo;
interface

{$S-}

type
  TExeFile=record
    f : file;
    // cached filesize
    size      : int64;
    isopen    : boolean;
    nsects    : longint;
    sechdrofs,
    secstrofs : {$ifdef cpui8086}longword{$else}ptruint{$endif};
    processaddress : {$ifdef cpui8086}word{$else}ptruint{$endif};
{$ifdef cpui8086}
    processsegment : word;
{$endif cpui8086}
    FunctionRelative: boolean;
    // Offset of the binary image forming permanent offset to all retrieved values
    ImgOffset: {$ifdef cpui8086}longword{$else}ptruint{$endif};
    filename  : string;
    // Allocate static buffer for reading data
    buf       : array[0..4095] of byte;
    bufsize,
    bufcnt    : longint;
  end;

function OpenExeFile(var e:TExeFile;const fn:string):boolean;
function FindExeSection(var e:TExeFile;const secname:string;var secofs,seclen:longint):boolean;
function CloseExeFile(var e:TExeFile):boolean;
function ReadDebugLink(var e:TExeFile;var dbgfn:string):boolean;

{$ifdef CPUI8086}
procedure GetModuleByAddr(addr: farpointer; var baseaddr: farpointer; var filename: string);
{$else CPUI8086}
procedure GetModuleByAddr(addr: pointer; var baseaddr: pointer; var filename: string);
{$endif CPUI8086}

implementation

uses
  strings{$ifdef windows},windows{$endif windows};

{$if defined(unix) and not defined(beos) and not defined(haiku)}

  procedure GetModuleByAddr(addr: pointer; var baseaddr: pointer; var filename: string);
    begin
      if assigned(UnixGetModuleByAddrHook) then
        UnixGetModuleByAddrHook(addr,baseaddr,filename)
      else
        begin
          baseaddr:=nil;
          filename:=ParamStr(0);
        end;
    end;

{$elseif defined(windows)}

  var
    Tmm: TMemoryBasicInformation;
{$ifdef FPC_OS_UNICODE}
    TST: array[0..Max_Path] of WideChar;
{$else}
    TST: array[0..Max_Path] of Char;
{$endif FPC_OS_UNICODE}
  procedure GetModuleByAddr(addr: pointer; var baseaddr: pointer; var filename: string);
    begin
      baseaddr:=nil;
      if VirtualQuery(addr, @Tmm, SizeOf(Tmm))<>sizeof(Tmm) then
        filename:=ParamStr(0)
      else
        begin
          baseaddr:=Tmm.AllocationBase;
          TST[0]:= #0;
          if baseaddr <> nil then
            begin
              GetModuleFileName(THandle(Tmm.AllocationBase), TST, Length(TST));
{$ifdef FPC_OS_UNICODE}
              filename:= String(PWideChar(@TST));
{$else}
              filename:= String(PChar(@TST));
{$endif FPC_OS_UNICODE}
            end;
        end;
    end;

{$elseif defined(morphos) or defined(aros) or defined(amigaos4)}

  procedure startsymbol; external name '_start';

  procedure GetModuleByAddr(addr: pointer; var baseaddr: pointer; var filename: string);
    begin
      baseaddr:= @startsymbol;
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
      filename:=ParamStr(0);
{$else FPC_HAS_FEATURE_COMMANDARGS}
      filename:='';
{$endif FPC_HAS_FEATURE_COMMANDARGS}
    end;

{$elseif defined(msdos)}

  procedure GetModuleByAddr(addr: farpointer; var baseaddr: farpointer; var filename: string);
    begin
      baseaddr:=Ptr(PrefixSeg+16,0);
      filename:=ParamStr(0);
    end;

{$elseif defined(beos) or defined(haiku)}

{$i ptypes.inc}
{$i ostypes.inc}

  function get_next_image_info(team: team_id; var cookie:longint; var info:image_info; size: size_t) : status_t;cdecl; external 'root' name '_get_next_image_info';

  procedure GetModuleByAddr(addr: pointer; var baseaddr: pointer; var filename: string);
    const
      B_OK = 0;
    var
      cookie    : longint;
      info      : image_info;
    begin
      filename:='';
      baseaddr:=nil;

      cookie:=0;
      fillchar(info, sizeof(image_info), 0);

      while get_next_image_info(0,cookie,info,sizeof(info))=B_OK do
        begin
          if (info._type = B_APP_IMAGE) and
             (addr >= info.text) and (addr <= (info.text + info.text_size)) then
            begin
              baseaddr:=info.text;
              filename:=PChar(@info.name);
            end;
        end;
    end;

{$else}

{$ifdef CPUI8086}
  procedure GetModuleByAddr(addr: farpointer; var baseaddr: farpointer; var filename: string);
{$else CPUI8086}
  procedure GetModuleByAddr(addr: pointer; var baseaddr: pointer; var filename: string);
{$endif CPUI8086}
    begin
      baseaddr:= nil;
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
      filename:=ParamStr(0);
{$else FPC_HAS_FEATURE_COMMANDARGS}
      filename:='';
{$endif FPC_HAS_FEATURE_COMMANDARGS}
    end;

{$endif}

{****************************************************************************
                             Executable Loaders
****************************************************************************}

{$if defined(freebsd) or defined(netbsd) or defined (openbsd) or defined(linux) or defined(sunos) or defined(android) or defined(dragonfly)}
  {$ifdef cpu64}
    {$define ELF64}
    {$define FIND_BASEADDR_ELF}
  {$else}
    {$define ELF32}
    {$define FIND_BASEADDR_ELF}
  {$endif}
{$endif}

{$if defined(beos) or defined(haiku)}
  {$ifdef cpu64}
    {$define ELF64}
  {$else}
    {$define ELF32}
  {$endif}
{$endif}

{$if defined(morphos) or defined(aros) or defined(amigaos4)}
  {$ifdef cpu64}
    {$define ELF64}
  {$else}
    {$define ELF32}
  {$endif}
{$endif}

{$if defined(msdos)}
  {$define ELF32}
{$endif}

{$if defined(win32) or defined(wince)}
  {$define PE32}
{$endif}

{$if defined(win64)}
  {$define PE32PLUS}
{$endif}

{$ifdef netwlibc}
  {$define netware}
{$endif}

{$IFDEF OS2}
  {$DEFINE EMX}
{$ENDIF OS2}


{****************************************************************************
                              DOS Stub
****************************************************************************}

{$if defined(EMX) or defined(PE32) or defined(PE32PLUS) or defined(GO32V2) or defined(MSDOS)}
type
  tdosheader = packed record
     e_magic : word;
     e_cblp : word;
     e_cp : word;
     e_crlc : word;
     e_cparhdr : word;
     e_minalloc : word;
     e_maxalloc : word;
     e_ss : word;
     e_sp : word;
     e_csum : word;
     e_ip : word;
     e_cs : word;
     e_lfarlc : word;
     e_ovno : word;
     e_res : array[0..3] of word;
     e_oemid : word;
     e_oeminfo : word;
     e_res2 : array[0..9] of word;
     e_lfanew : longint;
  end;
{$endif EMX or PE32 or PE32PLUS or GO32v2}


{****************************************************************************
                                  NLM
****************************************************************************}

{$ifdef netware}

function getByte(var f:file):byte;
  begin
    BlockRead (f,getByte,1);
  end;

  procedure Skip (var f:file; bytes : longint);
  var i : longint;
  begin
    for i := 1 to bytes do getbyte(f);
  end;

  function get0String (var f:file) : string;
  var c : char;
  begin
    get0String := '';
    c := char (getbyte(f));
    while (c <> #0) do
    begin
      get0String := get0String + c;
      c := char (getbyte(f));
    end;
  end;

  function getint32 (var f:file): longint;
  begin
    blockread (F, getint32, 4);
  end;


const SIZE_OF_NLM_INTERNAL_FIXED_HEADER = 130;
      SIZE_OF_NLM_INTERNAL_VERSION_HEADER = 32;
      SIZE_OF_NLM_INTERNAL_EXTENDED_HEADER = 124;

function openNetwareNLM(var e:TExeFile):boolean;
var valid : boolean;
    name  : string;
    hdrLength,
    dataOffset,
    dataLength : longint;


  function getLString : String;
  var Res:string;
  begin
    blockread (e.F, res, 1);
    if length (res) > 0 THEN
      blockread (e.F, res[1], length (res));
    getbyte(e.f);
    getLString := res;
  end;

  function getFixString (Len : byte) : string;
  var i : byte;
  begin
    getFixString := '';
    for I := 1 to Len do
      getFixString := getFixString + char (getbyte(e.f));
  end;


  function getword : word;
  begin
    blockread (e.F, getword, 2);
  end;



begin
  e.sechdrofs := 0;
  openNetwareNLM:=false;

  // read and check header
  Skip (e.f,SIZE_OF_NLM_INTERNAL_FIXED_HEADER);
  getLString;  // NLM Description
  getInt32(e.f);    // Stacksize
  getInt32(e.f);    // Reserved
  skip(e.f,5);     // old Thread Name
  getLString;  // Screen Name
  getLString;  // Thread Name
  hdrLength := -1;
  dataOffset := -1;
  dataLength := -1;
  valid := true;
  repeat
    name := getFixString (8);
    if (name = 'VeRsIoN#') then
    begin
      Skip (e.f,SIZE_OF_NLM_INTERNAL_VERSION_HEADER-8);
    end else
    if (name = 'CoPyRiGh') then
    begin
      getword;     // T=
      getLString;  // Copyright String
    end else
    if (name = 'MeSsAgEs') then
    begin
      skip (e.f,SIZE_OF_NLM_INTERNAL_EXTENDED_HEADER - 8);
    end else
    if (name = 'CuStHeAd') then
    begin
      hdrLength := getInt32(e.f);
      dataOffset := getInt32(e.f);
      dataLength := getInt32(e.f);
      Skip (e.f,8); // dateStamp
      Valid := false;
    end else
      Valid := false;
  until not valid;
  if (hdrLength = -1) or (dataOffset = -1) or (dataLength = -1) then
    exit;

  Seek (e.F, dataOffset);
  e.sechdrofs := dataOffset;
  openNetwareNLM := (e.sechdrofs > 0);
end;

function FindSectionNetwareNLM(var e:TExeFile;const asecname:string;var secofs,seclen:longint):boolean;
var name : string;
    alignAmount : longint;
begin
  seek(e.f,e.sechdrofs);
    (* The format of the section information is:
       null terminated section name
       zeroes to adjust to 4 byte boundary
       4 byte section data file pointer
       4 byte section size *)
  Repeat
    Name := Get0String(e.f);
    alignAmount := 4 - ((length (Name) + 1) MOD 4);
    Skip (e.f,AlignAmount);
    if (Name = asecname) then
    begin
      secOfs := getInt32(e.f);
      secLen := getInt32(e.f);
    end else
      Skip(e.f,8);
  until (Name = '') or (Name = asecname);
  FindSectionNetwareNLM := (Name=asecname);
end;

{$endif}


{****************************************************************************
                               COFF
****************************************************************************}

{$if defined(PE32) or defined(PE32PLUS) or defined(GO32V2)}
type
  tcoffsechdr=packed record
    name     : array[0..7] of char;
    vsize    : longint;
    rvaofs   : longint;
    datalen  : longint;
    datapos  : longint;
    relocpos : longint;
    lineno1  : longint;
    nrelocs  : word;
    lineno2  : word;
    flags    : longint;
  end;
  coffsymbol=packed record
    name    : array[0..3] of char; { real is [0..7], which overlaps the strofs ! }
    strofs  : longint;
    value   : longint;
    section : smallint;
    empty   : word;
    typ     : byte;
    aux     : byte;
  end;

function FindSectionCoff(var e:TExeFile;const asecname:string;var secofs,seclen:longint):boolean;
var
  i : longint;
  sechdr     : tcoffsechdr;
  secname    : string;
  secnamebuf : array[0..255] of char;
  code,
  oldofs,
  bufsize    : longint;
  strofs     : cardinal;
begin
  FindSectionCoff:=false;
  { read section info }
  seek(e.f,e.sechdrofs);
  for i:=1 to e.nsects do
   begin
     blockread(e.f,sechdr,sizeof(sechdr),bufsize);
     move(sechdr.name,secnamebuf,8);
     secnamebuf[8]:=#0;
     secname:=strpas(secnamebuf);
     if secname[1]='/' then
       begin
         Val(Copy(secname,2,8),strofs,code);
         if code=0 then
           begin
             fillchar(secnamebuf,sizeof(secnamebuf),0);
             oldofs:=filepos(e.f);
             seek(e.f,e.secstrofs+strofs);
             blockread(e.f,secnamebuf,sizeof(secnamebuf),bufsize);
             seek(e.f,oldofs);
             secname:=strpas(secnamebuf);
           end
         else
           secname:='';
       end;
     if asecname=secname then
       begin
         secofs:=cardinal(sechdr.datapos) + E.ImgOffset;
{$ifdef GO32V2}
         seclen:=sechdr.datalen;
{$else GO32V2}
         { In PECOFF, datalen includes file padding up to the next section.
           vsize is the actual payload size if it does not exceed datalen,
           otherwise it is .bss (or alike) section that we should ignore.  }
         if sechdr.vsize<=sechdr.datalen then
           seclen:=sechdr.vsize
         else
           exit;
{$endif GO32V2}
         FindSectionCoff:=true;
         exit;
       end;
   end;
end;
{$endif PE32 or PE32PLUS or GO32V2}


{$ifdef go32v2}
function OpenGo32Coff(var e:TExeFile):boolean;
type
  tgo32coffheader=packed record
    mach   : word;
    nsects : word;
    time   : longint;
    sympos : longint;
    syms   : longint;
    opthdr : word;
    flag   : word;
    other  : array[0..27] of byte;
  end;
const
  ParagraphSize = 512;
var
  coffheader : tgo32coffheader;
  DosHeader: TDosHeader;
  BRead: cardinal;
begin
  OpenGo32Coff:=false;
  { read and check header }
  if E.Size < SizeOf (DosHeader) then
   Exit;
  BlockRead (E.F, DosHeader, SizeOf (DosHeader), BRead);
  if BRead <> SizeOf (DosHeader) then
   Exit;
  if DosHeader.E_Magic = $5A4D then
  begin
   E.ImgOffset := DosHeader.e_cp * ParagraphSize;
   if DosHeader.e_cblp > 0 then
    E.ImgOffset := E.ImgOffset + DosHeader.e_cblp - ParagraphSize;
  end;
  if e.size < E.ImgOffset + sizeof(coffheader) then
   exit;
  seek(e.f,E.ImgOffset);
  blockread(e.f,coffheader,sizeof(coffheader));
  if coffheader.mach<>$14c then
    exit;
  e.sechdrofs:=filepos(e.f);
  e.nsects:=coffheader.nsects;
  e.secstrofs:=coffheader.sympos+coffheader.syms*sizeof(coffsymbol)+4;
  if e.secstrofs>e.size then
    exit;
  OpenGo32Coff:=true;
end;
{$endif Go32v2}


{$ifdef PE32}
function OpenPeCoff(var e:TExeFile):boolean;
type
  tpeheader = packed record
     PEMagic : longint;
     Machine : word;
     NumberOfSections : word;
     TimeDateStamp : longint;
     PointerToSymbolTable : longint;
     NumberOfSymbols : longint;
     SizeOfOptionalHeader : word;
     Characteristics : word;
     Magic : word;
     MajorLinkerVersion : byte;
     MinorLinkerVersion : byte;
     SizeOfCode : longint;
     SizeOfInitializedData : longint;
     SizeOfUninitializedData : longint;
     AddressOfEntryPoint : longint;
     BaseOfCode : longint;
     BaseOfData : longint;
     ImageBase : longint;
     SectionAlignment : longint;
     FileAlignment : longint;
     MajorOperatingSystemVersion : word;
     MinorOperatingSystemVersion : word;
     MajorImageVersion : word;
     MinorImageVersion : word;
     MajorSubsystemVersion : word;
     MinorSubsystemVersion : word;
     Reserved1 : longint;
     SizeOfImage : longint;
     SizeOfHeaders : longint;
     CheckSum : longint;
     Subsystem : word;
     DllCharacteristics : word;
     SizeOfStackReserve : longint;
     SizeOfStackCommit : longint;
     SizeOfHeapReserve : longint;
     SizeOfHeapCommit : longint;
     LoaderFlags : longint;
     NumberOfRvaAndSizes : longint;
     DataDirectory : array[1..$80] of byte;
  end;
var
  dosheader  : tdosheader;
  peheader   : tpeheader;
begin
  OpenPeCoff:=false;
  { read and check header }
  if e.size<sizeof(dosheader) then
    exit;
  blockread(e.f,dosheader,sizeof(tdosheader));
  seek(e.f,dosheader.e_lfanew);
  blockread(e.f,peheader,sizeof(tpeheader));
  if peheader.pemagic<>$4550 then
    exit;
  e.sechdrofs:=filepos(e.f);
  e.nsects:=peheader.NumberOfSections;
  e.secstrofs:=peheader.PointerToSymbolTable+peheader.NumberOfSymbols*sizeof(coffsymbol);
  if e.secstrofs>e.size then
    exit;
  e.processaddress:=peheader.ImageBase;
  OpenPeCoff:=true;
end;
{$endif PE32}


{$ifdef PE32PLUS}
function OpenPePlusCoff(var e:TExeFile):boolean;
type
  tpeheader = packed record
     PEMagic : longint;
     Machine : word;
     NumberOfSections : word;
     TimeDateStamp : longint;
     PointerToSymbolTable : longint;
     NumberOfSymbols : longint;
     SizeOfOptionalHeader : word;
     Characteristics : word;
     Magic : word;
     MajorLinkerVersion : byte;
     MinorLinkerVersion : byte;
     SizeOfCode : longint;
     SizeOfInitializedData : longint;
     SizeOfUninitializedData : longint;
     AddressOfEntryPoint : longint;
     BaseOfCode : longint;
     ImageBase : qword;
     SectionAlignment : longint;
     FileAlignment : longint;
     MajorOperatingSystemVersion : word;
     MinorOperatingSystemVersion : word;
     MajorImageVersion : word;
     MinorImageVersion : word;
     MajorSubsystemVersion : word;
     MinorSubsystemVersion : word;
     Reserved1 : longint;
     SizeOfImage : longint;
     SizeOfHeaders : longint;
     CheckSum : longint;
     Subsystem : word;
     DllCharacteristics : word;
     SizeOfStackReserve : qword;
     SizeOfStackCommit : qword;
     SizeOfHeapReserve : qword;
     SizeOfHeapCommit : qword;
     LoaderFlags : longint;
     NumberOfRvaAndSizes : longint;
     DataDirectory : array[1..$80] of byte;
  end;
var
  dosheader  : tdosheader;
  peheader   : tpeheader;
begin
  OpenPePlusCoff:=false;
  { read and check header }
  if E.Size<sizeof(dosheader) then
   exit;
  blockread(E.F,dosheader,sizeof(tdosheader));
  seek(E.F,dosheader.e_lfanew);
  blockread(E.F,peheader,sizeof(tpeheader));
  if peheader.pemagic<>$4550 then
   exit;
  e.sechdrofs:=filepos(e.f);
  e.nsects:=peheader.NumberOfSections;
  e.secstrofs:=peheader.PointerToSymbolTable+peheader.NumberOfSymbols*sizeof(coffsymbol);
  if e.secstrofs>e.size then
    exit;
  e.processaddress:=peheader.ImageBase;
  OpenPePlusCoff:=true;
end;
{$endif PE32PLUS}


{****************************************************************************
                                 AOUT
****************************************************************************}

{$IFDEF EMX}
type
  TEmxHeader = packed record
     Version: array [1..16] of char;
     Bound: word;
     AoutOfs: longint;
     Options: array [1..42] of char;
  end;

  TAoutHeader = packed record
     Magic: word;
     Machine: byte;
     Flags: byte;
     TextSize: longint;
     DataSize: longint;
     BssSize: longint;
     SymbSize: longint;
     EntryPoint: longint;
     TextRelocSize: longint;
     DataRelocSize: longint;
  end;

const
 PageSizeFill = $FFF;

var
 DosHeader: TDosHeader;
 EmxHeader: TEmxHeader;
 AoutHeader: TAoutHeader;
 StabOfs: PtrUInt;
 S4: string [4];

function OpenEMXaout (var E: TExeFile): boolean;
begin
 OpenEMXaout := false;
{ GDB after 4.18 uses offset to function begin
  in text section but OS/2 version still uses 4.16 PM }
 E.FunctionRelative := false;
{ read and check header }
 if E.Size > SizeOf (DosHeader) then
 begin
  BlockRead (E.F, DosHeader, SizeOf (TDosHeader));
{$IFDEF DEBUG_LINEINFO}
  WriteLn (StdErr, 'DosHeader.E_CParHdr = ', DosHeader.E_cParHdr);
{$ENDIF DEBUG_LINEINFO}
  if E.Size > DosHeader.e_cparhdr shl 4 + SizeOf (TEmxHeader) then
  begin
   Seek (E.F, DosHeader.e_cparhdr shl 4);
   BlockRead (E.F, EmxHeader, SizeOf (TEmxHeader));
  S4 [0] := #4;
  Move (EmxHeader.Version, S4 [1], 4);
   if (S4 = 'emx ') and
                       (E.Size > EmxHeader.AoutOfs + SizeOf (TAoutHeader)) then
   begin
{$IFDEF DEBUG_LINEINFO}
    WriteLn (StdErr, 'EmxHeader.AoutOfs = ', EmxHeader.AoutOfs, '/', HexStr (pointer (EmxHeader.AoutOfs)));
{$ENDIF DEBUG_LINEINFO}
    Seek (E.F, EmxHeader.AoutOfs);
    BlockRead (E.F, AoutHeader, SizeOf (TAoutHeader));
{$IFDEF DEBUG_LINEINFO}
    WriteLn (StdErr, 'AoutHeader.Magic = ', AoutHeader.Magic);
{$ENDIF DEBUG_LINEINFO}
{    if AOutHeader.Magic = $10B then}
    StabOfs := (EmxHeader.AoutOfs or PageSizeFill) + 1
                 + AoutHeader.TextSize
                 + AoutHeader.DataSize
                 + AoutHeader.TextRelocSize
                 + AoutHeader.DataRelocSize;
{$IFDEF DEBUG_LINEINFO}
    WriteLn (StdErr, 'AoutHeader.TextSize = ', AoutHeader.TextSize, '/', HexStr (pointer (AoutHeader.TextSize)));
    WriteLn (StdErr, 'AoutHeader.DataSize = ', AoutHeader.DataSize, '/', HexStr (pointer (AoutHeader.DataSize)));
    WriteLn (StdErr, 'AoutHeader.TextRelocSize = ', AoutHeader.TextRelocSize, '/', HexStr (pointer (AoutHeader.TextRelocSize)));
    WriteLn (StdErr, 'AoutHeader.DataRelocSize = ', AoutHeader.DataRelocSize, '/', HexStr (pointer (AoutHeader.DataRelocSize)));
    WriteLn (StdErr, 'AoutHeader.SymbSize = ', AoutHeader.SymbSize, '/', HexStr (pointer (AoutHeader.SymbSize)));
    WriteLn (StdErr, 'StabOfs = ', StabOfs, '/', HexStr (pointer (StabOfs)));
{$ENDIF DEBUG_LINEINFO}
    if E.Size > StabOfs + AoutHeader.SymbSize then
     OpenEMXaout := true;
   end;
  end;
 end;
end;


function FindSectionEMXaout (var E: TExeFile; const ASecName: string;
                                         var SecOfs, SecLen: longint): boolean;
begin
 FindSectionEMXaout := false;
 if ASecName = '.stab' then
 begin
  SecOfs := StabOfs;
  SecLen := AoutHeader.SymbSize;
  FindSectionEMXaout := true;
 end else
 if ASecName = '.stabstr' then
 begin
  SecOfs := StabOfs + AoutHeader.SymbSize;
  SecLen := E.Size - Pred (SecOfs);
  FindSectionEMXaout := true;
 end;
end;
{$ENDIF EMX}


{****************************************************************************
                                 ELF
****************************************************************************}

{$if defined(ELF32)}
type
  telfheader=packed record
      magic0123         : longint;
      file_class        : byte;
      data_encoding     : byte;
      file_version      : byte;
      padding           : array[$07..$0f] of byte;
      e_type            : word;
      e_machine         : word;
      e_version         : longword;
      e_entry           : longword;                  // entrypoint
      e_phoff           : longword;                  // program header offset
      e_shoff           : longword;                  // sections header offset
      e_flags           : longword;
      e_ehsize          : word;             // elf header size in bytes
      e_phentsize       : word;             // size of an entry in the program header array
      e_phnum           : word;             // 0..e_phnum-1 of entrys
      e_shentsize       : word;             // size of an entry in sections header array
      e_shnum           : word;             // 0..e_shnum-1 of entrys
      e_shstrndx        : word;             // index of string section header
  end;
  telfsechdr=packed record
      sh_name           : longword;
      sh_type           : longword;
      sh_flags          : longword;
      sh_addr           : longword;
      sh_offset         : longword;
      sh_size           : longword;
      sh_link           : longword;
      sh_info           : longword;
      sh_addralign      : longword;
      sh_entsize        : longword;
    end;
  telfproghdr=packed record
    p_type            : longword;
    p_offset          : longword;
    p_vaddr           : longword;
    p_paddr           : longword;
    p_filesz          : longword;
    p_memsz           : longword;
    p_flags           : longword;
    p_align           : longword;
  end;
{$endif ELF32}
{$ifdef ELF64}
type
  telfheader=packed record
      magic0123         : longint;
      file_class        : byte;
      data_encoding     : byte;
      file_version      : byte;
      padding           : array[$07..$0f] of byte;
      e_type            : word;
      e_machine         : word;
      e_version         : longword;
      e_entry           : int64;                  // entrypoint
      e_phoff           : int64;                  // program header offset
      e_shoff           : int64;                  // sections header offset
      e_flags           : longword;
      e_ehsize          : word;             // elf header size in bytes
      e_phentsize       : word;             // size of an entry in the program header array
      e_phnum           : word;             // 0..e_phnum-1 of entrys
      e_shentsize       : word;             // size of an entry in sections header array
      e_shnum           : word;             // 0..e_shnum-1 of entrys
      e_shstrndx        : word;             // index of string section header
  end;
type
  telfsechdr=packed record
      sh_name           : longword;
      sh_type           : longword;
      sh_flags          : int64;
      sh_addr           : int64;
      sh_offset         : int64;
      sh_size           : int64;
      sh_link           : longword;
      sh_info           : longword;
      sh_addralign      : int64;
      sh_entsize        : int64;
    end;

  telfproghdr=packed record
    p_type            : longword;
    p_flags           : longword;
    p_offset          : qword;
    p_vaddr           : qword;
    p_paddr           : qword;
    p_filesz          : qword;
    p_memsz           : qword;
    p_align           : qword;
  end;
{$endif ELF64}


{$if defined(ELF32) or defined(ELF64)}

{$ifdef FIND_BASEADDR_ELF}
var
  LocalJmpBuf : Jmp_Buf;
procedure LocalError;
begin
  Longjmp(LocalJmpBuf,1);
end;

procedure GetExeInMemoryBaseAddr(addr : pointer; var BaseAddr : pointer;
                                 var filename : openstring);
type
  AT_HDR = record
    typ : ptruint;
    value : ptruint;
  end;
  P_AT_HDR = ^AT_HDR;

{ Values taken from /usr/include/linux/auxvec.h }
const
  AT_HDR_COUNT = 5;{ AT_PHNUM }
  AT_HDR_SIZE = 4; { AT_PHENT }
  AT_HDR_Addr = 3; { AT_PHDR }
  AT_EXE_FN = 31;  {AT_EXECFN }

var
  pc : ppchar;
  pat_hdr : P_AT_HDR;
  i, phdr_count : ptrint;
  phdr_size : ptruint;
  phdr :  ^telfproghdr;
  found_addr : ptruint;
  SavedExitProc : pointer;
begin
  filename:=ParamStr(0);
  SavedExitProc:=ExitProc;
  ExitProc:=@LocalError;
  if SetJmp(LocalJmpBuf)=0 then
  begin
  { Try, avoided in order to remove exception installation }
    pc:=envp;
    phdr_count:=-1;
    phdr_size:=0;
    phdr:=nil;
    found_addr:=ptruint(-1);
    while (assigned(pc^)) do
      inc (pointer(pc), sizeof(ptruint));
    inc(pointer(pc), sizeof(ptruint));
    pat_hdr:=P_AT_HDR(pc);
    while assigned(pat_hdr) do
      begin
        if (pat_hdr^.typ=0) and (pat_hdr^.value=0) then
          break;
        if pat_hdr^.typ = AT_HDR_COUNT then
          phdr_count:=pat_hdr^.value;
        if pat_hdr^.typ = AT_HDR_SIZE then
          phdr_size:=pat_hdr^.value;
        if pat_hdr^.typ = AT_HDR_Addr then
          phdr := pointer(pat_hdr^.value);
        if pat_hdr^.typ = AT_EXE_FN then
          filename:=strpas(pchar(pat_hdr^.value));
        inc (pointer(pat_hdr),sizeof(AT_HDR));
      end;
    if (phdr_count>0) and (phdr_size = sizeof (telfproghdr))
       and  assigned(phdr) then
      begin
        for i:=0 to phdr_count -1 do
          begin
            if (phdr^.p_type = 1 {PT_LOAD}) and (ptruint(phdr^.p_vaddr) < found_addr) then
              found_addr:=phdr^.p_vaddr;
            inc(pointer(phdr), phdr_size);
          end;
      {$ifdef DEBUG_LINEINFO}
      end
    else
      begin
        if (phdr_count=-1) then
           writeln(stderr,'AUX entry AT_PHNUM not found');
        if (phdr_size=0) then
           writeln(stderr,'AUX entry AT_PHENT not found');
        if (phdr=nil) then
           writeln(stderr,'AUX entry AT_PHDR not found');
      {$endif DEBUG_LINEINFO}
      end;

     if found_addr<>ptruint(-1) then
       begin
          {$ifdef DEBUG_LINEINFO}
          Writeln(stderr,'Found addr = $',hexstr(found_addr,2 * sizeof(ptruint)));
          {$endif}
          BaseAddr:=pointer(found_addr);
       end
  {$ifdef DEBUG_LINEINFO}
     else
    writeln(stderr,'Error parsing stack');
  {$endif DEBUG_LINEINFO}
  end
  else
  begin
  {$ifdef DEBUG_LINEINFO}
    writeln(stderr,'Exception parsing stack');
  {$endif DEBUG_LINEINFO}
  end;
  ExitProc:=SavedExitProc;
end;
{$endif FIND_BASEADDR_ELF}

function OpenElf(var e:TExeFile):boolean;
{$ifdef MSDOS}
const
  ParagraphSize = 512;
{$endif MSDOS}
var
  elfheader : telfheader;
  elfsec    : telfsechdr;
  phdr      : telfproghdr;
  i         : longint;
{$ifdef MSDOS}
  DosHeader : tdosheader;
  BRead     : cardinal;
{$endif MSDOS}
begin
  OpenElf:=false;
{$ifdef MSDOS}
  { read and check header }
  if E.Size < SizeOf (DosHeader) then
   Exit;
  BlockRead (E.F, DosHeader, SizeOf (DosHeader), BRead);
  if BRead <> SizeOf (DosHeader) then
   Exit;
  if DosHeader.E_Magic = $5A4D then
  begin
   E.ImgOffset := LongWord(DosHeader.e_cp) * ParagraphSize;
   if DosHeader.e_cblp > 0 then
    E.ImgOffset := E.ImgOffset + DosHeader.e_cblp - ParagraphSize;
  end;
{$endif MSDOS}
  { read and check header }
  if e.size<(sizeof(telfheader)+e.ImgOffset) then
   exit;
  seek(e.f,e.ImgOffset);
  blockread(e.f,elfheader,sizeof(telfheader));
 if elfheader.magic0123<>{$ifdef ENDIAN_LITTLE}$464c457f{$else}$7f454c46{$endif} then
   exit;
  if elfheader.e_shentsize<>sizeof(telfsechdr) then
   exit;
  { read section names }
  seek(e.f,e.ImgOffset+elfheader.e_shoff+elfheader.e_shstrndx*cardinal(sizeof(telfsechdr)));
  blockread(e.f,elfsec,sizeof(telfsechdr));
  e.secstrofs:=elfsec.sh_offset;
  e.sechdrofs:=elfheader.e_shoff;
  e.nsects:=elfheader.e_shnum;

{$ifdef MSDOS}
  { e.processaddress is already initialized to 0 }
  e.processsegment:=PrefixSeg+16;
{$else MSDOS}
  { scan program headers to find the image base address }
  e.processaddress:=High(e.processaddress);
  seek(e.f,e.ImgOffset+elfheader.e_phoff);
  for i:=1 to elfheader.e_phnum do
    begin
      blockread(e.f,phdr,sizeof(phdr));
      if (phdr.p_type = 1 {PT_LOAD}) and (ptruint(phdr.p_vaddr) < e.processaddress) then
        e.processaddress:=phdr.p_vaddr;
    end;

  if e.processaddress = High(e.processaddress) then
    e.processaddress:=0;
{$endif MSDOS}

  OpenElf:=true;
end;


function FindSectionElf(var e:TExeFile;const asecname:string;var secofs,seclen:longint):boolean;
var
  elfsec     : telfsechdr;
  secname    : string;
  secnamebuf : array[0..255] of char;
  oldofs,
  bufsize,i  : longint;
begin
  FindSectionElf:=false;
  seek(e.f,e.ImgOffset+e.sechdrofs);
  for i:=1 to e.nsects do
   begin
     blockread(e.f,elfsec,sizeof(telfsechdr));
     fillchar(secnamebuf,sizeof(secnamebuf),0);
     oldofs:=filepos(e.f);
     seek(e.f,e.ImgOffset+e.secstrofs+elfsec.sh_name);
     blockread(e.f,secnamebuf,sizeof(secnamebuf)-1,bufsize);
     seek(e.f,oldofs);
     secname:=strpas(secnamebuf);
     if asecname=secname then
       begin
         secofs:=e.ImgOffset+elfsec.sh_offset;
         seclen:=elfsec.sh_size;
         FindSectionElf:=true;
         exit;
       end;
   end;
end;
{$endif ELF32 or ELF64}


{****************************************************************************
                                 MACHO
****************************************************************************}

{$ifdef darwin}
type
  MachoFatHeader= packed record
    magic: longint;
    nfatarch: longint;
  end;
  MachoHeader=packed record
    magic: longword;
    cpu_type_t: longint;
    cpu_subtype_t: longint;
    filetype: longint;
    ncmds: longint;
    sizeofcmds: longint;
    flags: longint;
  end;
  cmdblock=packed record
    cmd: longint;
    cmdsize: longint;
  end;
  symbSeg=packed record
    symoff :      longint;
    nsyms  :      longint;
    stroff :      longint;
    strsize:      longint;
  end;
  tstab=packed record
    strpos  : longint;
    ntype   : byte;
    nother  : byte;
    ndesc   : word;
    nvalue  : dword;
  end;


function OpenMachO32PPC(var e:TExeFile):boolean;
var
   mh:MachoHeader;
begin
  OpenMachO32PPC:= false;
  E.FunctionRelative:=false;
  if e.size<sizeof(mh) then
    exit;
  blockread (e.f, mh, sizeof(mh));
  e.sechdrofs:=filepos(e.f);
  e.nsects:=mh.ncmds;
  OpenMachO32PPC:=true;
end;


function FindSectionMachO32PPC(var e:TExeFile;const asecname:string;var secofs,seclen:longint):boolean;
var
   i: longint;
   block:cmdblock;
   symbolsSeg: symbSeg;
begin
  FindSectionMachO32PPC:=false;
  seek(e.f,e.sechdrofs);
  for i:= 1 to e.nsects do
    begin
      {$I-}
      blockread (e.f, block, sizeof(block));
      {$I+}
      if IOResult <> 0 then
        Exit;
      if block.cmd = $2   then
      begin
          blockread (e.f, symbolsSeg, sizeof(symbolsSeg));
          if asecname='.stab' then
            begin
              secofs:=symbolsSeg.symoff;
              { the caller will divide again by sizeof(tstab) }
              seclen:=symbolsSeg.nsyms*sizeof(tstab);
              FindSectionMachO32PPC:=true;
            end
          else if asecname='.stabstr' then
            begin
              secofs:=symbolsSeg.stroff;
              seclen:=symbolsSeg.strsize;
              FindSectionMachO32PPC:=true;
            end;
          exit;
      end;
      Seek(e.f, FilePos (e.f) + block.cmdsize - sizeof(block));
    end;
end;
{$endif darwin}


{****************************************************************************
                                   CRC
****************************************************************************}

var
  Crc32Tbl : array[0..255] of cardinal;

procedure MakeCRC32Tbl;
var
  crc : cardinal;
  i,n : integer;
begin
  for i:=0 to 255 do
   begin
     crc:=i;
     for n:=1 to 8 do
      if (crc and 1)<>0 then
       crc:=(crc shr 1) xor cardinal($edb88320)
      else
       crc:=crc shr 1;
     Crc32Tbl[i]:=crc;
   end;
end;


Function UpdateCrc32(InitCrc:cardinal;const InBuf;InLen:LongInt):cardinal;
var
  i : LongInt;
  p : pchar;
begin
  if Crc32Tbl[1]=0 then
   MakeCrc32Tbl;
  p:=@InBuf;
  UpdateCrc32:=not InitCrc;
  for i:=1 to InLen do
   begin
     UpdateCrc32:=Crc32Tbl[byte(UpdateCrc32) xor byte(p^)] xor (UpdateCrc32 shr 8);
     inc(p);
   end;
  UpdateCrc32:=not UpdateCrc32;
end;


{****************************************************************************
                         Generic Executable Open/Close
****************************************************************************}

type
  TOpenProc=function(var e:TExeFile):boolean;
  TFindSectionProc=function(var e:TExeFile;const asecname:string;var secofs,seclen:longint):boolean;

  TExeProcRec=record
    openproc : TOpenProc;
    findproc : TFindSectionProc;
  end;

const
  ExeProcs : TExeProcRec = (
{$ifdef go32v2}
     openproc : @OpenGo32Coff;
     findproc : @FindSectionCoff;
{$endif}
{$ifdef PE32}
     openproc : @OpenPeCoff;
     findproc : @FindSectionCoff;
{$endif}
{$ifdef PE32PLUS}
     openproc : @OpenPePlusCoff;
     findproc : @FindSectionCoff;
{$endif PE32PLUS}
{$if defined(ELF32) or defined(ELF64)}
     openproc : @OpenElf;
     findproc : @FindSectionElf;
{$endif ELF32 or ELF64}
{$ifdef darwin}
     openproc : @OpenMachO32PPC;
     findproc : @FindSectionMachO32PPC;
{$endif darwin}
{$IFDEF EMX}
     openproc : @OpenEMXaout;
     findproc : @FindSectionEMXaout;
{$ENDIF EMX}
{$ifdef netware}
     openproc : @OpenNetwareNLM;
     findproc : @FindSectionNetwareNLM;
{$endif}
   );

function OpenExeFile(var e:TExeFile;const fn:string):boolean;
var
  ofm : word;
begin
  OpenExeFile:=false;
  fillchar(e,sizeof(e),0);
  e.bufsize:=sizeof(e.buf);
  e.filename:=fn;
  if fn='' then   // we don't want to read stdin
    exit;
  assign(e.f,fn);
  {$I-}
   ofm:=filemode;
   filemode:=$40;
   reset(e.f,1);
   filemode:=ofm;
  {$I+}
  if ioresult<>0 then
   exit;
  e.isopen:=true;
  // cache filesize
  e.size:=filesize(e.f);

  E.FunctionRelative := true;
  E.ImgOffset := 0;
  if ExeProcs.OpenProc<>nil then
    OpenExeFile:=ExeProcs.OpenProc(e);
end;


function CloseExeFile(var e:TExeFile):boolean;
begin
  CloseExeFile:=false;
  if not e.isopen then
    exit;
  e.isopen:=false;
  close(e.f);
  CloseExeFile:=true;
end;


function FindExeSection(var e:TExeFile;const secname:string;var secofs,seclen:longint):boolean;
begin
  FindExeSection:=false;
  if not e.isopen then
    exit;
  if ExeProcs.FindProc<>nil then
    FindExeSection:=ExeProcs.FindProc(e,secname,secofs,seclen);
end;



function CheckDbgFile(var e:TExeFile;const fn:string;dbgcrc:cardinal):boolean;
var
  c      : cardinal;
  ofm    : word;
  g      : file;
begin
  CheckDbgFile:=false;
  assign(g,fn);
  {$I-}
   ofm:=filemode;
   filemode:=$40;
   reset(g,1);
   filemode:=ofm;
  {$I+}
  if ioresult<>0 then
   exit;
  { We reuse the buffer from e here to prevent too much stack allocation }
  c:=0;
  repeat
    blockread(g,e.buf,e.bufsize,e.bufcnt);
    c:=UpdateCrc32(c,e.buf,e.bufcnt);
  until e.bufcnt<e.bufsize;
  close(g);
  CheckDbgFile:=(dbgcrc=c);
end;


function ReadDebugLink(var e:TExeFile;var dbgfn:string):boolean;
var
  dbglink : array[0..255] of char;
  i,
  dbglinklen,
  dbglinkofs : longint;
  dbgcrc     : cardinal;
begin
  ReadDebugLink:=false;
  if not FindExeSection(e,'.gnu_debuglink',dbglinkofs,dbglinklen) then
    exit;
  if dbglinklen>sizeof(dbglink)-1 then
    exit;
  fillchar(dbglink,sizeof(dbglink),0);
  seek(e.f,dbglinkofs);
  blockread(e.f,dbglink,dbglinklen);
  dbgfn:=strpas(dbglink);
  if length(dbgfn)=0 then
    exit;
  i:=align(length(dbgfn)+1,4);
  if (i+4)>dbglinklen then
    exit;
  move(dbglink[i],dbgcrc,4);
  { current dir }
  if CheckDbgFile(e,dbgfn,dbgcrc) then
    begin
      ReadDebugLink:=true;
      exit;
    end;
  { executable dir }
  i:=length(e.filename);
  while (i>0) and not(e.filename[i] in AllowDirectorySeparators) do
    dec(i);
  if i>0 then
    begin
      dbgfn:=copy(e.filename,1,i)+dbgfn;
      if CheckDbgFile(e,dbgfn,dbgcrc) then
        begin
          ReadDebugLink:=true;
          exit;
        end;
    end;
end;


begin
{$ifdef FIND_BASEADDR_ELF}
  UnixGetModuleByAddrHook:=@GetExeInMemoryBaseAddr;
{$endif FIND_BASEADDR_ELF}
end.
