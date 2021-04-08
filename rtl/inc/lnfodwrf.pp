{
    This file is part of the Free Pascal run time library.

    Copyright (c) 2006 by Thomas Schatzl, member of the FreePascal
    Development team
    Parts (c) 2000 Peter Vreman (adapted from original dwarfs line
    reader)

    Dwarf LineInfo Retriever

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
unit lnfodwrf;

interface

{$S-}

{$IF FPC_VERSION<3}
type
  CodePointer = Pointer;
{$ENDIF}

function GetLineInfo(addr:codeptruint;var func,source:string;var line:longint) : boolean;
function DwarfBackTraceStr(addr: CodePointer): string;
procedure CloseDwarf;

var
  // Allows more efficient operation by reusing previously loaded debug data
  // when the target module filename is the same. However, if an invalid memory
  // address is supplied then further calls may result in an undefined behaviour.
  // In summary: enable for speed, disable for resilience.
  AllowReuseOfLineInfoData: Boolean = True;


implementation

uses
  exeinfo;

{ Current issues:

  - ignores DW_LNS_SET_FILE
}

{$MACRO ON}

{ $DEFINE DEBUG_DWARF_PARSER}
{$ifdef DEBUG_DWARF_PARSER}
  {$define DEBUG_WRITELN := WriteLn}
  {$define DEBUG_COMMENT :=  }
{$else}
  {$define DEBUG_WRITELN := //}
  {$define DEBUG_COMMENT := //}
{$endif}

{ some type definitions }
type
  Bool8 = ByteBool;
{$ifdef CPUI8086}
  TOffset = Word;
{$else CPUI8086}
  TOffset = PtrUInt;
{$endif CPUI8086}
  TSegment = Word;

const
  EBUF_SIZE = 100;

{$WARNING This code is not thread-safe, and needs improvement}
var
  { the input file to read DWARF debug info from, i.e. paramstr(0) }
  e : TExeFile;
  EBuf: Array [0..EBUF_SIZE-1] of Byte;
  EBufCnt, EBufPos: Integer;
  { the offset and size of the DWARF debug_line section in the file }
  Dwarf_Debug_Line_Section_Offset,
  Dwarf_Debug_Line_Section_Size,
  { the offset and size of the DWARF debug_info section in the file }
  Dwarf_Debug_Info_Section_Offset,
  Dwarf_Debug_Info_Section_Size,
  { the offset and size of the DWARF debug_aranges section in the file }
  Dwarf_Debug_Aranges_Section_Offset,
  Dwarf_Debug_Aranges_Section_Size,
  { the offset and size of the DWARF debug_abbrev section in the file }
  Dwarf_Debug_Abbrev_Section_Offset,
  Dwarf_Debug_Abbrev_Section_Size : longint;

{ DWARF 2 default opcodes}
const
  { Extended opcodes }
  DW_LNE_END_SEQUENCE = 1;
  DW_LNE_SET_ADDRESS = 2;
  DW_LNE_DEFINE_FILE = 3;
{$ifdef CPUI8086}
  { non-standard Open Watcom extension; might conflict with future versions of
    the DWARF standard }
  DW_LNE_SET_SEGMENT = 4;
{$endif CPUI8086}
  { Standard opcodes }
  DW_LNS_COPY = 1;
  DW_LNS_ADVANCE_PC = 2;
  DW_LNS_ADVANCE_LINE = 3;
  DW_LNS_SET_FILE = 4;
  DW_LNS_SET_COLUMN = 5;
  DW_LNS_NEGATE_STMT = 6;
  DW_LNS_SET_BASIC_BLOCK = 7;
  DW_LNS_CONST_ADD_PC = 8;
  DW_LNS_FIXED_ADVANCE_PC = 9;
  DW_LNS_SET_PROLOGUE_END = 10;
  DW_LNS_SET_EPILOGUE_BEGIN = 11;
  DW_LNS_SET_ISA = 12;

  DW_FORM_addr = $1;
  DW_FORM_block2 = $3;
  DW_FORM_block4 = $4;
  DW_FORM_data2 = $5;
  DW_FORM_data4 = $6;
  DW_FORM_data8 = $7;
  DW_FORM_string = $8;
  DW_FORM_block = $9;
  DW_FORM_block1 = $a;
  DW_FORM_data1 = $b;
  DW_FORM_flag = $c;
  DW_FORM_sdata = $d;
  DW_FORM_strp = $e;
  DW_FORM_udata = $f;
  DW_FORM_ref_addr = $10;
  DW_FORM_ref1 = $11;
  DW_FORM_ref2 = $12;
  DW_FORM_ref4 = $13;
  DW_FORM_ref8 = $14;
  DW_FORM_ref_udata = $15;
  DW_FORM_indirect = $16;
  DW_FORM_sec_offset = $17;
  DW_FORM_exprloc = $18;
  DW_FORM_flag_present = $19;

type
  { state record for the line info state machine }
  TMachineState = record
    address : QWord;
    segment : TSegment;
    file_id : DWord;
    line : QWord;
    column : DWord;
    is_stmt : Boolean;
    basic_block : Boolean;
    end_sequence : Boolean;
    prolouge_end : Boolean;
    epilouge_begin : Boolean;
    isa : DWord;
    append_row : Boolean;
  end;

{ DWARF line number program header preceding the line number program, 64 bit version }
  TLineNumberProgramHeader64 = packed record
    magic : DWord;
    unit_length : QWord;
    version : Word;
    length : QWord;
    minimum_instruction_length : Byte;
    default_is_stmt : Bool8;
    line_base : ShortInt;
    line_range : Byte;
    opcode_base : Byte;
  end;

{ DWARF line number program header preceding the line number program, 32 bit version }
  TLineNumberProgramHeader32 = packed record
    unit_length : DWord;
    version : Word;
    length : DWord;
    minimum_instruction_length : Byte;
    default_is_stmt : Bool8;
    line_base : ShortInt;
    line_range : Byte;
    opcode_base : Byte;
  end;

  TDebugInfoProgramHeader64 = packed record
    magic : DWord;
    unit_length : QWord;
    version : Word;
    debug_abbrev_offset : QWord;
    address_size : Byte;
  end;

  TDebugInfoProgramHeader32= packed record
    unit_length : DWord;
    version : Word;
    debug_abbrev_offset : DWord;
    address_size : Byte;
  end;

  TDebugArangesHeader64 = packed record
    magic : DWord;
    unit_length : QWord;
    version : Word;
    debug_info_offset : QWord;
    address_size : Byte;
    segment_size : Byte;
{$ifndef CPUI8086}
    padding : DWord;
{$endif CPUI8086}
  end;

  TDebugArangesHeader32= packed record
    unit_length : DWord;
    version : Word;
    debug_info_offset : DWord;
    address_size : Byte;
    segment_size : Byte;
{$ifndef CPUI8086}
    padding : DWord;
{$endif CPUI8086}
  end;

{---------------------------------------------------------------------------
 I/O utility functions
---------------------------------------------------------------------------}

type
{$ifdef cpui8086}
  TFilePos = LongInt;
{$else cpui8086}
  TFilePos = SizeInt;
{$endif cpui8086}

var
  base, limit : TFilePos;
  index : TFilePos;
  baseaddr : {$ifdef cpui8086}farpointer{$else}pointer{$endif};
  filename,
  dbgfn : string;
  lastfilename: string;   { store last processed file }
  lastopendwarf: Boolean; { store last result of processing a file }

{$ifdef cpui8086}
function tofar(fp: FarPointer): FarPointer; inline;
begin
  tofar:=fp;
end;

function tofar(cp: NearCsPointer): FarPointer; inline;
begin
  tofar:=Ptr(CSeg,Word(cp));
end;

function tofar(cp: NearPointer): FarPointer; inline;
begin
  tofar:=Ptr(DSeg,Word(cp));
end;
{$else cpui8086}
type
  tofar=Pointer;
{$endif cpui8086}

function OpenDwarf(addr : codepointer) : boolean;
var
  oldprocessaddress: TExeProcessAddress;
begin
  // False by default
  OpenDwarf:=false;

  // Empty so can test if GetModuleByAddr has worked
  filename := '';

  // Get filename by address using GetModuleByAddr
  GetModuleByAddr(tofar(addr),baseaddr,filename);
{$ifdef DEBUG_LINEINFO}
  writeln(stderr,filename,' Baseaddr: ',hexstr(baseaddr));
{$endif DEBUG_LINEINFO}

  // Check if GetModuleByAddr has worked
  if filename = '' then
    exit;

  // If target filename same as previous, then re-use previous result
  if AllowReuseOfLineInfoData and (filename = lastfilename) then
  begin
    {$ifdef DEBUG_LINEINFO}
    writeln(stderr,'Reusing debug data');
    {$endif DEBUG_LINEINFO}
    OpenDwarf:=lastopendwarf;
    exit;
  end;

  // Close previously opened Dwarf
  CloseDwarf;

  // Reset last open dwarf result
  lastopendwarf := false;

  // Save newly processed filename
  lastfilename := filename;

  // Open exe file or debug link
  if not OpenExeFile(e,filename) then
    exit;
  if ReadDebugLink(e,dbgfn) then
    begin
      oldprocessaddress:=e.processaddress;
      CloseExeFile(e);
      if not OpenExeFile(e,dbgfn) then
        exit;
      e.processaddress:=oldprocessaddress;
    end;

  // Find debug data section
  e.processaddress:=ptruint(baseaddr)-e.processaddress;
  if FindExeSection(e,'.debug_line',Dwarf_Debug_Line_Section_offset,dwarf_Debug_Line_Section_size) and
    FindExeSection(e,'.debug_info',Dwarf_Debug_Info_Section_offset,dwarf_Debug_Info_Section_size) and
    FindExeSection(e,'.debug_abbrev',Dwarf_Debug_Abbrev_Section_offset,dwarf_Debug_Abbrev_Section_size) and
    FindExeSection(e,'.debug_aranges',Dwarf_Debug_Aranges_Section_offset,dwarf_Debug_Aranges_Section_size) then
  begin
    lastopendwarf:=true;
    OpenDwarf:=true;
    DEBUG_WRITELN('.debug_line starts at offset $',hexstr(Dwarf_Debug_Line_Section_offset,8),' with a size of ',Dwarf_Debug_Line_Section_Size,' Bytes');
    DEBUG_WRITELN('.debug_info starts at offset $',hexstr(Dwarf_Debug_Info_Section_offset,8),' with a size of ',Dwarf_Debug_Info_Section_Size,' Bytes');
    DEBUG_WRITELN('.debug_abbrev starts at offset $',hexstr(Dwarf_Debug_Abbrev_Section_offset,8),' with a size of ',Dwarf_Debug_Abbrev_Section_Size,' Bytes');
    DEBUG_WRITELN('.debug_aranges starts at offset $',hexstr(Dwarf_Debug_Aranges_Section_offset,8),' with a size of ',Dwarf_Debug_Aranges_Section_Size,' Bytes');
  end
  else
    CloseExeFile(e);
end;


procedure CloseDwarf;
begin
  if e.isopen then
    CloseExeFile(e);

  // Reset last processed filename
  lastfilename := '';
end;


function Init(aBase, aLimit : Int64) : Boolean;
begin
  base := aBase;
  limit := aLimit;
  Init := (aBase + limit) <= e.size;
  seek(e.f, base);
  EBufCnt := 0;
  EBufPos := 0;
  index := 0;
end;

function Init(aBase : Int64) : Boolean;
begin
  Init := Init(aBase, limit - (aBase - base));
end;


function Pos() : TFilePos;
begin
  Pos := index;
end;


procedure Seek(const newIndex : Int64);
begin
  index := newIndex;
  system.seek(e.f, base + index);
  EBufCnt := 0;
  EBufPos := 0;
end;


{ Returns the next Byte from the input stream, or -1 if there has been
  an error }
function ReadNext() : Longint; inline;
var
  bytesread : SizeInt;
begin
  ReadNext := -1;
  if EBufPos >= EBufCnt then begin
    EBufPos := 0;
    EBufCnt := EBUF_SIZE;
    if EBufCnt > limit - index then
      EBufCnt := limit - index;
    blockread(e.f, EBuf, EBufCnt, bytesread);
    EBufCnt := bytesread;
  end;
  if EBufPos < EBufCnt then begin
    ReadNext := EBuf[EBufPos];
    inc(EBufPos);
    inc(index);
  end
  else
    ReadNext := -1;
end;

{ Reads the next size bytes into dest. Returns true if successful,
  false otherwise. Note that dest may be partially overwritten after
  returning false. }
function ReadNext(var dest; size : SizeInt) : Boolean;
var
  bytesread, totalread : SizeInt;
  r: Boolean;
  d: PByte;
begin
  d := @dest;
  totalread := 0;
  r := True;
  while (totalread < size) and r do begin;
    if EBufPos >= EBufCnt then begin
      EBufPos := 0;
      EBufCnt := EBUF_SIZE;
      if EBufCnt > limit - index then
        EBufCnt := limit - index;
      blockread(e.f, EBuf, EBufCnt, bytesread);
      EBufCnt := bytesread;
      if bytesread <= 0 then
        r := False;
    end;
    if EBufPos < EBufCnt then begin
      bytesread := EBufCnt - EBufPos;
      if bytesread > size - totalread then bytesread := size - totalread;
      System.Move(EBuf[EBufPos], d[totalread], bytesread);
      inc(EBufPos, bytesread);
      inc(index, bytesread);
      inc(totalread, bytesread);
    end;
  end;
  ReadNext := r;
end;


{ Reads an unsigned LEB encoded number from the input stream }
function ReadULEB128() : QWord;
var
  shift : Byte;
  data : PtrInt;
  val : QWord;
begin
  shift := 0;
  ReadULEB128 := 0;
  data := ReadNext();
  while (data <> -1) do begin
    val := data and $7f;
    ReadULEB128 := ReadULEB128 or (val shl shift);
    inc(shift, 7);
    if ((data and $80) = 0) then
      break;
    data := ReadNext();
  end;
end;

{ Reads a signed LEB encoded number from the input stream }
function ReadLEB128() : Int64;
var
  shift : Byte;
  data : PtrInt;
  val : Int64;
begin
  shift := 0;
  ReadLEB128 := 0;
  data := ReadNext();
  while (data <> -1) do begin
    val := data and $7f;
    ReadLEB128 := ReadLEB128 or (val shl shift);
    inc(shift, 7);
    if ((data and $80) = 0) then
      break;
    data := ReadNext();
  end;
  { extend sign. Note that we can not use shl/shr since the latter does not
    translate to arithmetic shifting for signed types }
  ReadLEB128 := (not ((ReadLEB128 and (Int64(1) shl (shift-1)))-1)) or ReadLEB128;
end;


{$ifdef CPUI8086}
{ Reads an address from the current input stream }
function ReadAddress(addr_size: smallint) : LongWord;
begin
  if addr_size = 4 then
    ReadNext(ReadAddress, 4)
  else if addr_size = 2 then begin
    ReadAddress := 0;
    ReadNext(ReadAddress, 2);
  end
  else
    ReadAddress := 0;
end;

{ Reads a segment from the current input stream }
function ReadSegment() : Word;
begin
  ReadNext(ReadSegment, sizeof(ReadSegment));
end;
{$else CPUI8086}
{ Reads an address from the current input stream }
function ReadAddress(addr_size: smallint) : PtrUInt;
begin
  ReadNext(ReadAddress, sizeof(ReadAddress));
end;
{$endif CPUI8086}


{ Reads a zero-terminated string from the current input stream. If the
  string is larger than 255 chars (maximum allowed number of elements in
  a ShortString, excess characters will be chopped off. }
function ReadString() : ShortString;
var
  temp : PtrInt;
  i : PtrUInt;
begin
  i := 1;
  temp := ReadNext();
  while (temp > 0) do begin
    ReadString[i] := char(temp);
    if (i = 255) then begin
      { skip remaining characters }
      repeat
        temp := ReadNext();
      until (temp <= 0);
      break;
    end;
    inc(i);
    temp := ReadNext();
  end;
  { unexpected end of file occurred? }
  if (temp = -1) then
    ReadString := ''
  else
    Byte(ReadString[0]) := i-1;
end;


{ Reads an unsigned Half from the current input stream }
function ReadUHalf() : Word;
begin
  ReadNext(ReadUHalf, sizeof(ReadUHalf));
end;


{---------------------------------------------------------------------------

 Generic Dwarf lineinfo reader

 The line info reader is based on the information contained in

   DWARF Debugging Information Format Version 3
   Chapter 6.2 "Line Number Information"

 from the

   DWARF Debugging Information Format Workgroup.

 For more information on this document see also

   http://dwarf.freestandards.org/

---------------------------------------------------------------------------}

{ initializes the line info state to the default values }
procedure InitStateRegisters(var state : TMachineState; const aIs_Stmt : Bool8);
begin
  with state do begin
    address := 0;
    segment := 0;
    file_id := 1;
    line := 1;
    column := 0;
    is_stmt := aIs_Stmt;
    basic_block := false;
    end_sequence := false;
    prolouge_end := false;
    epilouge_begin := false;
    isa := 0;
    append_row := false;
  end;
end;


{ Skips all line info directory entries }
procedure SkipDirectories();
var s : ShortString;
begin
  while (true) do begin
    s := ReadString();
    if (s = '') then break;
    DEBUG_WRITELN('Skipping directory : ', s);
  end;
end;

{ Skips an LEB128 }
procedure SkipLEB128();
{$ifdef DEBUG_DWARF_PARSER}
var temp : QWord;
{$endif}
begin
  {$ifdef DEBUG_DWARF_PARSER}temp := {$endif}ReadLEB128();
  DEBUG_WRITELN('Skipping LEB128 : ', temp);
end;

{ Skips the filename section from the current file stream }
procedure SkipFilenames();
var s : ShortString;
begin
  while (true) do begin
    s := ReadString();
    if (s = '') then break;
    DEBUG_WRITELN('Skipping filename : ', s);
    SkipLEB128(); { skip the directory index for the file }
    SkipLEB128(); { skip last modification time for file }
    SkipLEB128(); { skip length of file }
  end;
end;

function CalculateAddressIncrement(opcode : Byte; const header : TLineNumberProgramHeader64) : Int64;
begin
  CalculateAddressIncrement := (Int64(opcode) - header.opcode_base) div header.line_range * header.minimum_instruction_length;
end;

function GetFullFilename(const filenameStart, directoryStart : Int64; const file_id : DWord) : ShortString;
var
  i : DWord;
  filename, directory : ShortString;
  dirindex : Int64;
begin
  filename := '';
  directory := '';
  i := 1;
  Seek(filenameStart);
  while (i <= file_id) do begin
    filename := ReadString();
    DEBUG_WRITELN('Found "', filename, '"');
    if (filename = '') then break;
    dirindex := ReadLEB128(); { read the directory index for the file }
    SkipLEB128(); { skip last modification time for file }
    SkipLEB128(); { skip length of file }
    inc(i);
  end;
  { if we could not find the file index, exit }
  if (filename = '') then begin
    GetFullFilename := '(Unknown file)';
    exit;
  end;

  Seek(directoryStart);
  i := 1;
  while (i <= dirindex) do begin
    directory := ReadString();
    if (directory = '') then break;
    inc(i);
  end;
  if (directory<>'') and (directory[length(directory)]<>'/') then
    directory:=directory+'/';
  GetFullFilename := directory + filename;
end;


function ParseCompilationUnit(const addr : TOffset; const segment : TSegment; const file_offset : QWord;
  var source : String; var line : longint; var found : Boolean) : QWord;
var
  state : TMachineState;
  { we need both headers on the stack, although we only use the 64 bit one internally }
  header64 : TLineNumberProgramHeader64;
  header32 : TLineNumberProgramHeader32;

  adjusted_opcode : Int64;

  opcode : PtrInt;
  extended_opcode : PtrInt;
  extended_opcode_length : PtrInt;
  i, addrIncrement, lineIncrement : PtrInt;

  {$ifdef DEBUG_DWARF_PARSER}
  s : ShortString;
  {$endif}

  numoptable : array[1..255] of Byte;
  { the offset into the file where the include directories are stored for this compilation unit }
  include_directories : QWord;
  { the offset into the file where the file names are stored for this compilation unit }
  file_names : Int64;

  temp_length : DWord;
  unit_length : QWord;
  header_length : SizeInt;

  first_row : Boolean;

  prev_line : QWord;
  prev_file : DWord;

begin
  prev_line := 0;
  prev_file := 0;
  first_row := true;

  found := false;

  ReadNext(temp_length, sizeof(temp_length));
  if (temp_length <> $ffffffff) then begin
    unit_length := temp_length + sizeof(temp_length)
  end else begin
    ReadNext(unit_length, sizeof(unit_length));
    inc(unit_length, 12);
  end;

  ParseCompilationUnit := file_offset + unit_length;

  Init(file_offset, unit_length);

  DEBUG_WRITELN('Unit length: ', unit_length);
  if (temp_length <> $ffffffff) then begin
    DEBUG_WRITELN('32 bit DWARF detected');
    ReadNext(header32, sizeof(header32));
    header64.magic := $ffffffff;
    header64.unit_length := header32.unit_length;
    header64.version := header32.version;
    header64.length := header32.length;
    header64.minimum_instruction_length := header32.minimum_instruction_length;
    header64.default_is_stmt := header32.default_is_stmt;
    header64.line_base := header32.line_base;
    header64.line_range := header32.line_range;
    header64.opcode_base := header32.opcode_base;
    header_length :=
      sizeof(header32.length) + sizeof(header32.version) +
      sizeof(header32.unit_length);
  end else begin
    DEBUG_WRITELN('64 bit DWARF detected');
    ReadNext(header64, sizeof(header64));
    header_length :=
      sizeof(header64.magic) + sizeof(header64.version) +
      sizeof(header64.length) + sizeof(header64.unit_length);
  end;

  inc(header_length, header64.length);

  fillchar(numoptable, sizeof(numoptable), #0);
  ReadNext(numoptable, header64.opcode_base-1);
  DEBUG_WRITELN('Opcode parameter count table');
  for i := 1 to header64.opcode_base-1 do begin
    DEBUG_WRITELN('Opcode[', i, '] - ', numoptable[i], ' parameters');
  end;

  DEBUG_WRITELN('Reading directories...');
  include_directories := Pos();
  SkipDirectories();
  DEBUG_WRITELN('Reading filenames...');
  file_names := Pos();
  SkipFilenames();

  Seek(header_length);

  with header64 do begin
    InitStateRegisters(state, default_is_stmt);
  end;
  opcode := ReadNext();
  while (opcode <> -1) and (not found) do begin
    DEBUG_WRITELN('Next opcode: ');
    case (opcode) of
      { extended opcode }
      0 : begin
        extended_opcode_length := ReadULEB128();
        extended_opcode := ReadNext();
        case (extended_opcode) of
          -1: begin
            exit;
          end;
          DW_LNE_END_SEQUENCE : begin
            state.end_sequence := true;
            state.append_row := true;
            DEBUG_WRITELN('DW_LNE_END_SEQUENCE');
          end;
          DW_LNE_SET_ADDRESS : begin
            state.address := ReadAddress(extended_opcode_length-1);
            DEBUG_WRITELN('DW_LNE_SET_ADDRESS (', hexstr(state.address, sizeof(state.address)*2), ')');
          end;
{$ifdef CPUI8086}
          DW_LNE_SET_SEGMENT : begin
            state.segment := ReadSegment();
            DEBUG_WRITELN('DW_LNE_SET_SEGMENT (', hexstr(state.segment, sizeof(state.segment)*2), ')');
          end;
{$endif CPUI8086}
          DW_LNE_DEFINE_FILE : begin
            {$ifdef DEBUG_DWARF_PARSER}s := {$endif}ReadString();
            SkipLEB128();
            SkipLEB128();
            SkipLEB128();
            DEBUG_WRITELN('DW_LNE_DEFINE_FILE (', s, ')');
          end;
          else begin
            DEBUG_WRITELN('Unknown extended opcode (opcode ', extended_opcode, ' length ', extended_opcode_length, ')');
            for i := 0 to extended_opcode_length-2 do
              if ReadNext() = -1 then
                exit;
          end;
        end;
      end;
      DW_LNS_COPY : begin
        state.basic_block := false;
        state.prolouge_end := false;
        state.epilouge_begin := false;
        state.append_row := true;
        DEBUG_WRITELN('DW_LNS_COPY');
      end;
      DW_LNS_ADVANCE_PC : begin
        inc(state.address, ReadULEB128() * header64.minimum_instruction_length);
        DEBUG_WRITELN('DW_LNS_ADVANCE_PC (', hexstr(state.address, sizeof(state.address)*2), ')');
      end;
      DW_LNS_ADVANCE_LINE : begin
        // inc(state.line, ReadLEB128()); negative values are allowed
        // but those may generate a range check error
        state.line := state.line + ReadLEB128();
        DEBUG_WRITELN('DW_LNS_ADVANCE_LINE (', state.line, ')');
      end;
      DW_LNS_SET_FILE : begin
        state.file_id := ReadULEB128();
        DEBUG_WRITELN('DW_LNS_SET_FILE (', state.file_id, ')');
      end;
      DW_LNS_SET_COLUMN : begin
        state.column := ReadULEB128();
        DEBUG_WRITELN('DW_LNS_SET_COLUMN (', state.column, ')');
      end;
      DW_LNS_NEGATE_STMT : begin
        state.is_stmt := not state.is_stmt;
        DEBUG_WRITELN('DW_LNS_NEGATE_STMT (', state.is_stmt, ')');
      end;
      DW_LNS_SET_BASIC_BLOCK : begin
        state.basic_block := true;
        DEBUG_WRITELN('DW_LNS_SET_BASIC_BLOCK');
      end;
      DW_LNS_CONST_ADD_PC : begin
        inc(state.address, CalculateAddressIncrement(255, header64));
        DEBUG_WRITELN('DW_LNS_CONST_ADD_PC (', hexstr(state.address, sizeof(state.address)*2), ')');
      end;
      DW_LNS_FIXED_ADVANCE_PC : begin
        inc(state.address, ReadUHalf());
        DEBUG_WRITELN('DW_LNS_FIXED_ADVANCE_PC (', hexstr(state.address, sizeof(state.address)*2), ')');
      end;
      DW_LNS_SET_PROLOGUE_END : begin
        state.prolouge_end := true;
        DEBUG_WRITELN('DW_LNS_SET_PROLOGUE_END');
      end;
      DW_LNS_SET_EPILOGUE_BEGIN : begin
        state.epilouge_begin := true;
        DEBUG_WRITELN('DW_LNS_SET_EPILOGUE_BEGIN');
      end;
      DW_LNS_SET_ISA : begin
        state.isa := ReadULEB128();
        DEBUG_WRITELN('DW_LNS_SET_ISA (', state.isa, ')');
      end;
      else begin { special opcode }
        if (opcode < header64.opcode_base) then begin
          DEBUG_WRITELN('Unknown standard opcode $', hexstr(opcode, 2), '; skipping');
          for i := 1 to numoptable[opcode] do
            SkipLEB128();
        end else begin
          adjusted_opcode := opcode - header64.opcode_base;
          addrIncrement := CalculateAddressIncrement(opcode, header64);
          inc(state.address, addrIncrement);
          lineIncrement := header64.line_base + (adjusted_opcode mod header64.line_range);
          inc(state.line, lineIncrement);
          DEBUG_WRITELN('Special opcode $', hexstr(opcode, 2), ' address increment: ', addrIncrement, ' new line: ', lineIncrement);
          state.basic_block := false;
          state.prolouge_end := false;
          state.epilouge_begin := false;
          state.append_row := true;
        end;
      end;
    end;

    if (state.append_row) then begin
      DEBUG_WRITELN('Current state : address = ', hexstr(state.address, sizeof(state.address) * 2),
{$ifdef CPUI8086}
      DEBUG_COMMENT ' segment = ', hexstr(state.segment, sizeof(state.segment) * 2),
{$endif CPUI8086}
      DEBUG_COMMENT ' file_id = ', state.file_id, ' line = ', state.line, ' column = ', state.column,
      DEBUG_COMMENT  ' is_stmt = ', state.is_stmt, ' basic_block = ', state.basic_block,
      DEBUG_COMMENT  ' end_sequence = ', state.end_sequence, ' prolouge_end = ', state.prolouge_end,
      DEBUG_COMMENT  ' epilouge_begin = ', state.epilouge_begin, ' isa = ', state.isa);

      if (first_row) then begin
        if (state.segment > segment) or
           ((state.segment = segment) and
            (state.address > addr)) then
          break;
        first_row := false;
      end;

      { when we have found the address we need to return the previous
        line because that contains the call instruction }
      if (state.segment > segment) or
         ((state.segment = segment) and
          (state.address >= addr)) then
        found:=true
      else
        begin
          { save line information }
          prev_file := state.file_id;
          prev_line := state.line;
        end;

      state.append_row := false;
      if (state.end_sequence) then begin
        InitStateRegisters(state, header64.default_is_stmt);
        first_row := true;
      end;
    end;

    opcode := ReadNext();
  end;

  if (found) then begin
    line := prev_line;
    source := GetFullFilename(file_names, include_directories, prev_file);
  end;
end;


var
  Abbrev_Offsets : array of QWord;
  Abbrev_Tags : array of QWord;
  Abbrev_Children : array of Byte;
  Abbrev_Attrs : array of array of record attr,form : QWord; end;

procedure ReadAbbrevTable;
  var
   i : PtrInt;
   tag,
   nr,
   attr,
   form,
   PrevHigh : Int64;
  begin
    DEBUG_WRITELN('Starting to read abbrev. section at $',hexstr(Dwarf_Debug_Abbrev_Section_Offset+Pos,16));
    repeat
      nr:=ReadULEB128;
      if nr=0 then
        break;

      if nr>high(Abbrev_Offsets) then
        begin
          SetLength(Abbrev_Offsets,nr+1024);
          SetLength(Abbrev_Tags,nr+1024);
          SetLength(Abbrev_Attrs,nr+1024);
          SetLength(Abbrev_Children,nr+1024);
        end;

      Abbrev_Offsets[nr]:=Pos;

      { read tag }
      tag:=ReadULEB128;
      Abbrev_Tags[nr]:=tag;
      DEBUG_WRITELN('Abbrev ',nr,' at offset ',Pos,' has tag $',hexstr(tag,4));
      { read flag for children }
      Abbrev_Children[nr]:=ReadNext;
      i:=0;
      { ensure that length(Abbrev_Attrs)=0 if an entry is overwritten (not sure if this will ever happen) and
        the new entry has no attributes }
      Abbrev_Attrs[nr]:=nil;
      repeat
        attr:=ReadULEB128;
        form:=ReadULEB128;
        if attr<>0 then
          begin
            SetLength(Abbrev_Attrs[nr],i+1);
            Abbrev_Attrs[nr][i].attr:=attr;
            Abbrev_Attrs[nr][i].form:=form;
          end;
        inc(i);
      until attr=0;
      DEBUG_WRITELN('Abbrev ',nr,' has ',Length(Abbrev_Attrs[nr]),' attributes');
    until false;
  end;


function ParseCompilationUnitForDebugInfoOffset(const addr : TOffset; const segment : TSegment; const file_offset : QWord;
  var debug_info_offset : QWord; var found : Boolean) : QWord;
{$ifndef CPUI8086}
const
  arange_segment = 0;
{$endif CPUI8086}
var
  { we need both headers on the stack, although we only use the 64 bit one internally }
  header64 : TDebugArangesHeader64;
  header32 : TDebugArangesHeader32;
  isdwarf64 : boolean;
  temp_length : DWord;
  unit_length : QWord;
{$ifdef CPUI8086}
  arange_start, arange_size: DWord;
  arange_segment: Word;
{$else CPUI8086}
  arange_start, arange_size: PtrUInt;
{$endif CPUI8086}
begin
  found := false;

  ReadNext(temp_length, sizeof(temp_length));
  if (temp_length <> $ffffffff) then begin
    unit_length := temp_length + sizeof(temp_length)
  end else begin
    ReadNext(unit_length, sizeof(unit_length));
    inc(unit_length, 12);
  end;

  ParseCompilationUnitForDebugInfoOffset := file_offset + unit_length;

  Init(file_offset, unit_length);

  DEBUG_WRITELN('Unit length: ', unit_length);
  if (temp_length <> $ffffffff) then
    begin
      DEBUG_WRITELN('32 bit DWARF detected');
      ReadNext(header32, sizeof(header32));
      header64.magic := $ffffffff;
      header64.unit_length := header32.unit_length;
      header64.version := header32.version;
      header64.debug_info_offset := header32.debug_info_offset;
      header64.address_size := header32.address_size;
      header64.segment_size := header32.segment_size;
      isdwarf64:=false;
    end
  else
    begin
      DEBUG_WRITELN('64 bit DWARF detected');
      ReadNext(header64, sizeof(header64));
      isdwarf64:=true;
    end;

  DEBUG_WRITELN('debug_info_offset: ',header64.debug_info_offset);
  DEBUG_WRITELN('address_size: ', header64.address_size);
  DEBUG_WRITELN('segment_size: ', header64.segment_size);
  arange_start:=ReadAddress(header64.address_size);
{$ifdef CPUI8086}
  arange_segment:=ReadSegment();
{$endif CPUI8086}
  arange_size:=ReadAddress(header64.address_size);

  while not((arange_start=0) and (arange_segment=0) and (arange_size=0)) and (not found) do
    begin
      if (segment=arange_segment) and (addr>=arange_start) and (addr<=arange_start+arange_size) then
        begin
          found:=true;
          debug_info_offset:=header64.debug_info_offset;
          DEBUG_WRITELN('Matching aranges entry $',hexStr(arange_start,header64.address_size*2),', $',hexStr(arange_size,header64.address_size*2));
        end;

      arange_start:=ReadAddress(header64.address_size);
{$ifdef CPUI8086}
      arange_segment:=ReadSegment();
{$endif CPUI8086}
      arange_size:=ReadAddress(header64.address_size);
    end;
end;

function ParseCompilationUnitForFunctionName(const addr : TOffset; const segment : TSegment; const file_offset : QWord;
  var func : String; var found : Boolean) : QWord;
var
  { we need both headers on the stack, although we only use the 64 bit one internally }
  header64 : TDebugInfoProgramHeader64;
  header32 : TDebugInfoProgramHeader32;
  isdwarf64 : boolean;
  abbrev,
  high_pc,
  low_pc : QWord;
  temp_length : DWord;
  unit_length : QWord;
  name : String;
  level : Integer;

procedure SkipAttr(form : QWord);
  var
    dummy : array[0..7] of byte;
    bl : byte;
    wl : word;
    dl : dword;
    ql : qword;
    i : PtrUInt;
  begin
    case form of
      DW_FORM_addr:
        ReadNext(dummy,header64.address_size);
      DW_FORM_block2:
        begin
          ReadNext(wl,SizeOf(wl));
          for i:=1 to wl do
            ReadNext;
        end;
      DW_FORM_block4:
        begin
          ReadNext(dl,SizeOf(dl));
          for i:=1 to dl do
            ReadNext;
        end;
      DW_FORM_data2:
        ReadNext(dummy,2);
      DW_FORM_data4:
        ReadNext(dummy,4);
      DW_FORM_data8:
        ReadNext(dummy,8);
      DW_FORM_string:
        ReadString;
      DW_FORM_block,
      DW_FORM_exprloc:
        begin
          ql:=ReadULEB128;
          for i:=1 to ql do
            ReadNext;
        end;
      DW_FORM_block1:
        begin
          bl:=ReadNext;
          for i:=1 to bl do
            ReadNext;
        end;
      DW_FORM_data1,
      DW_FORM_flag:
        ReadNext(dummy,1);
      DW_FORM_sdata:
        ReadLEB128;
      DW_FORM_ref_addr:
        { the size of DW_FORM_ref_addr changed between DWAWRF2 and later versions:
          in DWARF2 it depends on the architecture address size, in later versions on the DWARF type (32 bit/64 bit)
        }
        if header64.version>2 then
          begin
            if isdwarf64 then
              ReadNext(dummy,8)
            else
              ReadNext(dummy,4);
          end
        else
          begin
            { address size for DW_FORM_ref_addr must be at least 32 bits }
            { this is compatible with Open Watcom on i8086 }
            if header64.address_size<4 then
              ReadNext(dummy,4)
            else
              ReadNext(dummy,header64.address_size);
          end;
      DW_FORM_strp,
      DW_FORM_sec_offset:
        if isdwarf64 then
          ReadNext(dummy,8)
        else
          ReadNext(dummy,4);
      DW_FORM_udata:
        ReadULEB128;
      DW_FORM_ref1:
        ReadNext(dummy,1);
      DW_FORM_ref2:
        ReadNext(dummy,2);
      DW_FORM_ref4:
        ReadNext(dummy,4);
      DW_FORM_ref8:
        ReadNext(dummy,8);
      DW_FORM_ref_udata:
        ReadULEB128;
      DW_FORM_indirect:
        SkipAttr(ReadULEB128);
      DW_FORM_flag_present: {none};
      else
        begin
          writeln(stderr,'Internal error: unknown dwarf form: $',hexstr(form,2));
          ReadNext;
          exit;
        end;
    end;
  end;

var
  i : PtrInt;
  prev_base,prev_limit : TFilePos;
  prev_pos : TFilePos;

begin
  found := false;

  ReadNext(temp_length, sizeof(temp_length));
  if (temp_length <> $ffffffff) then begin
    unit_length := temp_length + sizeof(temp_length)
  end else begin
    ReadNext(unit_length, sizeof(unit_length));
    inc(unit_length, 12);
  end;

  ParseCompilationUnitForFunctionName := file_offset + unit_length;

  Init(file_offset, unit_length);

  DEBUG_WRITELN('Unit length: ', unit_length);
  if (temp_length <> $ffffffff) then begin
    DEBUG_WRITELN('32 bit DWARF detected');
    ReadNext(header32, sizeof(header32));
    header64.magic := $ffffffff;
    header64.unit_length := header32.unit_length;
    header64.version := header32.version;
    header64.debug_abbrev_offset := header32.debug_abbrev_offset;
    header64.address_size := header32.address_size;
    isdwarf64:=false;
  end else begin
    DEBUG_WRITELN('64 bit DWARF detected');
    ReadNext(header64, sizeof(header64));
    isdwarf64:=true;
  end;

  DEBUG_WRITELN('debug_abbrev_offset: ',header64.debug_abbrev_offset);
  DEBUG_WRITELN('address_size: ',header64.address_size);

  { not nice, but we have to read the abbrev section after the start of the debug_info section has been read }
  prev_limit:=limit;
  prev_base:=base;
  prev_pos:=Pos;
  Init(Dwarf_Debug_Abbrev_Section_Offset+header64.debug_abbrev_offset,Dwarf_Debug_Abbrev_Section_Size);
  ReadAbbrevTable;

  { restore previous reading state and position }
  Init(prev_base,prev_limit);
  Seek(prev_pos);

  abbrev:=ReadULEB128;
  level:=0;
  while (abbrev <> 0) and (not found) do
    begin
      DEBUG_WRITELN('Next abbrev: ',abbrev);
      if Abbrev_Children[abbrev]<>0 then
        inc(level);
      { DW_TAG_subprogram? }
      if Abbrev_Tags[abbrev]=$2e then
        begin
          low_pc:=1;
          high_pc:=0;
          name:='';
          for i:=0 to high(Abbrev_Attrs[abbrev]) do
            begin
              { DW_AT_low_pc }
              if (Abbrev_Attrs[abbrev][i].attr=$11) and
               (Abbrev_Attrs[abbrev][i].form=DW_FORM_addr) then
                begin
                  low_pc:=0;
                  ReadNext(low_pc,header64.address_size);
                end
              { DW_AT_high_pc }
              else if (Abbrev_Attrs[abbrev][i].attr=$12) and
               (Abbrev_Attrs[abbrev][i].form=DW_FORM_addr) then
                begin
                  high_pc:=0;
                  ReadNext(high_pc,header64.address_size);
                end
              { DW_AT_name }
              else if (Abbrev_Attrs[abbrev][i].attr=$3) and
                { avoid that we accidently read an DW_FORM_strp entry accidently }
                (Abbrev_Attrs[abbrev][i].form=DW_FORM_string) then
                begin
                  name:=ReadString;
                end
              else
                SkipAttr(Abbrev_Attrs[abbrev][i].form);
            end;
          DEBUG_WRITELN('Got DW_TAG_subprogram with low pc = $',hexStr(low_pc,header64.address_size*2),', high pc = $',hexStr(high_pc,header64.address_size*2),', name = ',name);
          if (addr>low_pc) and (addr<high_pc) then
            begin
              found:=true;
              func:=name;
            end;
        end
      else
        begin
          for i:=0 to high(Abbrev_Attrs[abbrev]) do
            SkipAttr(Abbrev_Attrs[abbrev][i].form);
        end;
      abbrev:=ReadULEB128;
      { skip entries signaling that no more child entries are following }
      while (level>0) and (abbrev=0) do
        begin
          dec(level);
          abbrev:=ReadULEB128;
        end;
    end;
end;

const
{ 64 bit and 32 bit CPUs tend to have more memory }
{$if defined(CPU64)}
  LineInfoCacheLength = 2039;
{$elseif defined(CPU32)}
  LineInfoCacheLength = 251;
{$else}
  LineInfoCacheLength = 1;
{$endif CPU64}

var
  LineInfoCache : array[0..LineInfoCacheLength-1] of
                    record
                      addr : codeptruint;
                      func, source : string;
                      line : longint;
                    end;

function GetLineInfo(addr : codeptruint; var func, source : string; var line : longint) : boolean;
var
  current_offset,
  end_offset, debug_info_offset_from_aranges : QWord;
  segment : Word = 0;

  found, found_aranges : Boolean;
  CacheIndex: CodePtrUInt;

begin
  func := '';
  source := '';
  GetLineInfo:=false;

  CacheIndex:=addr mod LineInfoCacheLength;

  if LineInfoCache[CacheIndex].addr=addr then
    begin
      func:=LineInfoCache[CacheIndex].func;
      source:=LineInfoCache[CacheIndex].source;
      line:=LineInfoCache[CacheIndex].line;
      GetLineInfo:=true;
      exit;
    end;

  if not OpenDwarf(codepointer(addr)) then
    exit;

{$ifdef CPUI8086}
  {$if defined(FPC_MM_MEDIUM) or defined(FPC_MM_LARGE) or defined(FPC_MM_HUGE)}
    segment := (addr shr 16) - e.processsegment;
    addr := Word(addr);
  {$else}
    segment := CSeg - e.processsegment;
  {$endif}
{$endif CPUI8086}

  addr := addr - e.processaddress;

  current_offset := Dwarf_Debug_Line_Section_Offset;
  end_offset := Dwarf_Debug_Line_Section_Offset + Dwarf_Debug_Line_Section_Size;

  found := false;
  while (current_offset < end_offset) and (not found) do begin
    Init(current_offset, end_offset - current_offset);
    current_offset := ParseCompilationUnit(addr, segment, current_offset,
      source, line, found);
  end;

  current_offset := Dwarf_Debug_Aranges_Section_Offset;
  end_offset := Dwarf_Debug_Aranges_Section_Offset + Dwarf_Debug_Aranges_Section_Size;

  found_aranges := false;
  while (current_offset < end_offset) and (not found_aranges) do begin
    Init(current_offset, end_offset - current_offset);
    current_offset := ParseCompilationUnitForDebugInfoOffset(addr, segment, current_offset, debug_info_offset_from_aranges, found_aranges);
  end;

  { no function name found yet }
  found := false;

  if found_aranges then
    begin
      DEBUG_WRITELN('Found .debug_info offset $',hexstr(debug_info_offset_from_aranges,8),' from .debug_aranges');
      current_offset := Dwarf_Debug_Info_Section_Offset + debug_info_offset_from_aranges;
      end_offset := Dwarf_Debug_Info_Section_Offset + debug_info_offset_from_aranges + Dwarf_Debug_Info_Section_Size;

      DEBUG_WRITELN('Reading .debug_info at section offset $',hexStr(current_offset-Dwarf_Debug_Info_Section_Offset,16));

      Init(current_offset, end_offset - current_offset);
      current_offset := ParseCompilationUnitForFunctionName(addr, segment, current_offset, func, found);
      if found then
        DEBUG_WRITELN('Found .debug_info entry by using .debug_aranges information');
    end
  else
    DEBUG_WRITELN('No .debug_info offset found from .debug_aranges');

  current_offset := Dwarf_Debug_Info_Section_Offset;
  end_offset := Dwarf_Debug_Info_Section_Offset + Dwarf_Debug_Info_Section_Size;

  while (current_offset < end_offset) and (not found) do begin
    DEBUG_WRITELN('Reading .debug_info at section offset $',hexStr(current_offset-Dwarf_Debug_Info_Section_Offset,16));

    Init(current_offset, end_offset - current_offset);
    current_offset := ParseCompilationUnitForFunctionName(addr, segment, current_offset, func, found);
  end;

  if not AllowReuseOfLineInfoData then
    CloseDwarf;

  LineInfoCache[CacheIndex].addr:=addr;
  LineInfoCache[CacheIndex].func:=func;
  LineInfoCache[CacheIndex].source:=source;
  LineInfoCache[CacheIndex].line:=line;

  GetLineInfo:=true;
end;


function DwarfBackTraceStr(addr: CodePointer): string;
var
  func,
  source : string;
  hs     : string;
  line   : longint;
  Store  : TBackTraceStrFunc;
  Success : boolean;
begin
  {$ifdef DEBUG_LINEINFO}
  writeln(stderr,'DwarfBackTraceStr called');
  {$endif DEBUG_LINEINFO}
  { reset to prevent infinite recursion if problems inside the code }
  Success:=false;
  Store := BackTraceStrFunc;
  BackTraceStrFunc := @SysBackTraceStr;
  Success:=GetLineInfo(codeptruint(addr), func, source, line);
  { create string }
  DwarfBackTraceStr :='  $' + HexStr(addr);
  if Success then
  begin
    if func<>'' then
      DwarfBackTraceStr := DwarfBackTraceStr + '  ' + func;
    if source<>'' then
    begin
      if func<>'' then
        DwarfBackTraceStr := DwarfBackTraceStr + ', ';
      if line<>0 then
      begin
        str(line, hs);
        DwarfBackTraceStr := DwarfBackTraceStr + ' line ' + hs;
      end;
      DwarfBackTraceStr := DwarfBackTraceStr + ' of ' + source;
    end;
  end;
  BackTraceStrFunc := Store;
end;


initialization
  lastfilename := '';
  lastopendwarf := false;
  BackTraceStrFunc := @DwarfBacktraceStr;

finalization
  CloseDwarf;

end.
