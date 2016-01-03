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

function GetLineInfo(addr:ptruint;var func,source:string;var line:longint) : boolean;

implementation

uses
  exeinfo;

{ Current issues:

  - ignores DW_LNS_SET_FILE
}

{$MACRO ON}

//{$DEFINE DEBUG_DWARF_PARSER}
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
<<<<<<< HEAD
<<<<<<< HEAD

const
  EBUF_SIZE = 100;

{$WARNING This code is not thread-safe, and needs improvement}
var
  { the input file to read DWARF debug info from, i.e. paramstr(0) }
  e : TExeFile;
  EBuf: Array [0..EBUF_SIZE-1] of Byte;
  EBufCnt, EBufPos: Integer;
=======
=======
>>>>>>> origin/fixes_2_2

var
  { the input file to read DWARF debug info from, i.e. paramstr(0) }
  e : TExeFile;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  DwarfErr : boolean;
  { the offset and size of the DWARF debug_line section in the file }
  DwarfOffset : longint;
  DwarfSize : longint;

{ DWARF 2 default opcodes}
const
  { Extended opcodes }
  DW_LNE_END_SEQUENCE = 1;
  DW_LNE_SET_ADDRESS = 2;
  DW_LNE_DEFINE_FILE = 3;
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

type
  { state record for the line info state machine }
  TMachineState = record
    address : QWord;
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

{---------------------------------------------------------------------------
 I/O utility functions
---------------------------------------------------------------------------}

var
  base, limit : SizeInt;
  index : SizeInt;
  baseaddr : pointer;
  filename,
  dbgfn : string;

function Opendwarf(addr : pointer) : boolean;
begin
  Opendwarf:=false;
  if dwarferr then
    exit;

  GetModuleByAddr(addr,baseaddr,filename);
{$ifdef DEBUG_LINEINFO}
  writeln(stderr,filename,' Baseaddr: ',hexstr(ptruint(baseaddr),sizeof(baseaddr)*2));
{$endif DEBUG_LINEINFO}

  if not OpenExeFile(e,filename) then
    exit;
  if ReadDebugLink(e,dbgfn) then
    begin
      CloseExeFile(e);
      if not OpenExeFile(e,dbgfn) then
        exit;
    end;

<<<<<<< HEAD
<<<<<<< HEAD
  e.processaddress:=ptruint(baseaddr)-e.processaddress;
=======
  e.processaddress:=e.processaddress+dword(baseaddr);
>>>>>>> graemeg/fixes_2_2
=======
  e.processaddress:=e.processaddress+dword(baseaddr);
>>>>>>> origin/fixes_2_2

  if FindExeSection(e,'.debug_line',dwarfoffset,dwarfsize) then
    Opendwarf:=true
  else
    begin
      dwarferr:=true;
      exit;
    end;
end;


procedure Closedwarf;
begin
  CloseExeFile(e);
end;


function Init(aBase, aLimit : Int64) : Boolean;
begin
  base := aBase;
  limit := aLimit;
  Init := (aBase + limit) <= e.size;
  seek(e.f, base);
<<<<<<< HEAD
<<<<<<< HEAD
  EBufCnt := 0;
  EBufPos := 0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  index := 0;
end;

function Init(aBase : Int64) : Boolean;
begin
  Init := Init(aBase, limit - (aBase - base));
end;


function Pos() : Int64;
begin
  Pos := index;
end;


procedure Seek(const newIndex : Int64);
begin
  index := newIndex;
  system.seek(e.f, base + index);
<<<<<<< HEAD
<<<<<<< HEAD
  EBufCnt := 0;
  EBufPos := 0;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


{ Returns the next Byte from the input stream, or -1 if there has been
  an error }
<<<<<<< HEAD
<<<<<<< HEAD
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
function ReadNext(var dest; size : SizeInt) : Boolean; inline;
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

=======
function ReadNext() : Longint;
var
  bytesread : SizeInt;
  b : Byte;
begin
  ReadNext := -1;
  if (index < limit) then begin
    blockread(e.f, b, 1, bytesread);
    ReadNext := b;
    inc(index);
  end;
  if (bytesread <> 1) then
    ReadNext := -1;
end;

=======
function ReadNext() : Longint;
var
  bytesread : SizeInt;
  b : Byte;
begin
  ReadNext := -1;
  if (index < limit) then begin
    blockread(e.f, b, 1, bytesread);
    ReadNext := b;
    inc(index);
  end;
  if (bytesread <> 1) then
    ReadNext := -1;
end;

>>>>>>> origin/fixes_2_2
{ Reads the next size bytes into dest. Returns true if successful,
  false otherwise. Note that dest may be partially overwritten after
  returning false. }
function ReadNext(var dest; size : SizeInt) : Boolean;
var
  bytesread : SizeInt;
begin
  bytesread := 0;
  if ((index + size) < limit) then begin
    blockread(e.f, dest, size, bytesread);
    inc(index, size);
  end;
  ReadNext := (bytesread = size);
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

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
  ReadLEB128 := (not ((ReadLEB128 and (1 shl (shift-1)))-1)) or ReadLEB128;
end;


{ Reads an address from the current input stream }
function ReadAddress() : PtrUInt;
begin
  ReadNext(ReadAddress, sizeof(ReadAddress));
end;


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
<<<<<<< HEAD
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


=======
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


>>>>>>> origin/fixes_2_2
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


function ParseCompilationUnit(const addr : PtrUInt; const file_offset : QWord;
  var source : String; var line : longint; var found : Boolean) : QWord;
var
  state : TMachineState;
  { we need both headers on the stack, although we only use the 64 bit one internally }
  header64 : TLineNumberProgramHeader64;
  header32 : TLineNumberProgramHeader32;

  adjusted_opcode : Int64;

  opcode : PtrInt;
<<<<<<< HEAD
<<<<<<< HEAD
  extended_opcode : PtrInt;
=======
  extended_opcode : Byte;
>>>>>>> graemeg/fixes_2_2
=======
  extended_opcode : Byte;
>>>>>>> origin/fixes_2_2
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
            state.address := ReadAddress();
            DEBUG_WRITELN('DW_LNE_SET_ADDRESS (', hexstr(state.address, sizeof(state.address)*2), ')');
          end;
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
      DEBUG_COMMENT ' file_id = ', state.file_id, ' line = ', state.line, ' column = ', state.column,
      DEBUG_COMMENT  ' is_stmt = ', state.is_stmt, ' basic_block = ', state.basic_block,
      DEBUG_COMMENT  ' end_sequence = ', state.end_sequence, ' prolouge_end = ', state.prolouge_end,
      DEBUG_COMMENT  ' epilouge_begin = ', state.epilouge_begin, ' isa = ', state.isa);

      if (first_row) then begin
        if (state.address > addr) then
          break;
        first_row := false;
      end;

      { when we have found the address we need to return the previous
        line because that contains the call instruction }
      if (state.address >= addr) then
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

function GetLineInfo(addr : ptruint; var func, source : string; var line : longint) : boolean;
var
  current_offset : QWord;
  end_offset : QWord;

  found : Boolean;

begin
  func := '';
  source := '';
  found := false;
  GetLineInfo:=false;
  if DwarfErr then
    exit;
  if not e.isopen then
   begin
     if not OpenDwarf(pointer(addr)) then
      exit;
   end;

  addr := addr - e.processaddress;

  current_offset := DwarfOffset;
  end_offset := DwarfOffset + DwarfSize;

  while (current_offset < end_offset) and (not found) do begin
    Init(current_offset, end_offset - current_offset);
    current_offset := ParseCompilationUnit(addr, current_offset,
      source, line, found);
  end;
  if e.isopen then
    CloseDwarf;
  GetLineInfo:=true;
end;


function DwarfBackTraceStr(addr : CodePointer) : shortstring;
var
  func,
  source : string;
  hs     : string[32];
  line   : longint;
  Store  : TBackTraceStrFunc;
  Success : boolean;
begin
  { reset to prevent infinite recursion if problems inside the code }
  Success:=false;
  Store := BackTraceStrFunc;
  BackTraceStrFunc := @SysBackTraceStr;
  Success:=GetLineInfo(ptruint(addr), func, source, line);
  { create string }
  DwarfBackTraceStr :='  $' + HexStr(ptruint(addr), sizeof(ptruint) * 2);
  if func<>'' then
   DwarfBackTraceStr := DwarfBackTraceStr + '  ' + func;

  if source<>'' then begin
    if func<>'' then
      DwarfBackTraceStr := DwarfBackTraceStr + ', ';
    if line<>0 then begin
      str(line, hs);
      DwarfBackTraceStr := DwarfBackTraceStr + ' line ' + hs;
    end;
    DwarfBackTraceStr := DwarfBackTraceStr + ' of ' + source;
  end;
  if Success then
    BackTraceStrFunc := Store;
end;


initialization
  BackTraceStrFunc := @DwarfBacktraceStr;

finalization
  if e.isopen then
    CloseDwarf();
end.
