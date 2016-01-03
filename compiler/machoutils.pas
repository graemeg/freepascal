{
    Copyright (c) 2009-2010 by Dmitry Boyarintsev

    Contains utility routines and types for handling mach-o structure and types.

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

unit machoutils;

interface

{$mode objfpc}{$h+}

uses
  macho;

  type
    TRawWriter=class(TObject)
      public
        procedure WriteRaw(const data; datasize: Integer); virtual; abstract;
      end;

    TRawReader=class(TObject)
      public
        function ReadRaw(var data; datasize: Integer): Integer; virtual; abstract;
        function Seek(pos: qword): Boolean; virtual; abstract;
        function ReadPos: qword; virtual; abstract;
      end;

    TMachHeader=record
      cputype     : cpu_type_t;
      cpusubtype  : cpu_subtype_t;
      filetype    : longword;
      ncmds       : longword;
      sizeofcmds  : longword;
      flags       : longword;
    end;

    TSegmentName=string[16];
    TSectionName=TSegmentName;

    TMachoSegment=record
      segname    : TSegmentName;
      vmaddr     : qword;
      vmsize     : qword;
      fileoff    : qword;
      filesize   : qword;
      maxprot    : vm_prot_t;
      initprot   : vm_prot_t;
      nsects     : longword;
      flags      : longword;
    end;

    TMachoSection=record
      sectname  : TSectionName;
      segname   : TSegmentName;
      addr      : uint64_t;
      size      : uint64_t;
      offset    : uint32_t;
      align     : uint32_t;
      reloff    : uint32_t;
      nreloc    : uint32_t;
      flags     : uint32_t;

      indirectIndex  : Integer; // reserved1 for LAZY and NON_LAZY pointers
      stubSize       : Integer; // reserved2 for S_SYMBOL_STUBS
    end;

    TMachoRoutine=record
      init_address : uint64_t;    { address of initialization routine  }
      init_module  : uint64_t;    { index into the module table that  }
    end;

    { TMachoWriter }

    TMachoWriter=class(TObject)
      private
        fwriter : TRawWriter;
        fown    : Boolean;
      protected
      public
        constructor Create(ARawWriter: TRawWriter; AllowFreeWriter: Boolean);
        destructor Destroy; override;

        { non platform specific writer }
        procedure WriteData(const data; dataSize: Integer);
        procedure WriteUint8(i: uint8_t);

        { endian specific writer }
        procedure WriteUint16(i: uint16_t); virtual; abstract;
        procedure WriteUint32(i: uint32_t); virtual; abstract;
        procedure WriteUint64(i: uint64_t); virtual; abstract;

        { endian and ptr-size specific writer }
        procedure WritePtr(ofs: QWord); virtual; abstract; // ptr is 32 bit for 32-bit platforms

        { macro utility methods }

        procedure WriteHeader(const hdr: TMachHeader); virtual; abstract;
        procedure WriteSegmentCmd(const seg: TMachoSegment; cmdSize: LongWord); virtual; abstract;
        procedure WriteSection(const sec: TMachoSection); virtual; abstract;
        procedure WriteRoutineCmd(const rt: TMachoRoutine); virtual; abstract;
        procedure WriteLoadCommand(const cmd: load_command); virtual; abstract; overload;
        procedure WriteLoadCommand(cmd, cmdsize: Integer); overload;
        procedure WriteRelocation(const ri: relocation_info); virtual; abstract;
        procedure WriteScatterReloc(const ri: scattered_relocation_info); virtual; abstract;
        procedure WriteNList(const list: nlist_64); virtual; abstract;
      end;

    { TLE32MachoWriter }

    TLE32MachoWriter=class(TMachoWriter)
      public
        procedure WriteUint16(i: uint16_t); override;
        procedure WriteUint32(i: uint32_t); override;
        procedure WriteUint64(i: uint64_t); override;
        procedure WritePtr(ofs: QWord); override;

        procedure WriteHeader(const hdr: TMachHeader); override;
        procedure WriteSegmentCmd(const seg: TMachoSegment; ACmdSize: LongWord); override;
        procedure WriteSection(const sec: TMachoSection); override;
        procedure WriteRoutineCmd(const rt: TMachoRoutine); override;
        procedure WriteLoadCommand(const cmd: load_command); override;
        procedure WriteRelocation(const ri: relocation_info); override;
        procedure WriteScatterReloc(const ri: scattered_relocation_info); override;
        procedure WriteNList(const list: nlist_64); override;
      end;

    { TLE64MachoWriter }

    TLE64MachoWriter=class(TLE32MachoWriter)
      public
        procedure WritePtr(ofs: QWord); override;

        procedure WriteHeader(const hdr: TMachHeader); override;
        procedure WriteSegmentCmd(const seg: TMachoSegment; acmdSize: LongWord); override;
        procedure WriteSection(const sec: TMachoSection); override;
        procedure WriteRoutineCmd(const rt: TMachoRoutine); override;
        procedure WriteNList(const list: nlist_64); override;
      end;

    { TBE32MachoWriter }

    TBE32MachoWriter=class(TMachoWriter)
      public
        procedure WriteUint16(i: uint16_t); override;
        procedure WriteUint32(i: uint32_t); override;
        procedure WriteUint64(i: uint64_t); override;
        procedure WritePtr(ofs: QWord); override;

        procedure WriteHeader(const hdr: TMachHeader); override;
        procedure WriteSegmentCmd(const seg: TMachoSegment; acmdSize: LongWord); override;
        procedure WriteSection(const sec: TMachoSection); override;
        procedure WriteRoutineCmd(const rt: TMachoRoutine); override;
        procedure WriteLoadCommand(const cmd: load_command); override;
        procedure WriteRelocation(const ri: relocation_info); override;
        procedure WriteScatterReloc(const ri: scattered_relocation_info); override;
        procedure WriteNList(const list: nlist_64); override;
      end;

      { TBE64MachoWriter }

    TBE64MachoWriter=class(TBE32MachoWriter)
      public
        procedure WritePtr(ofs: QWord); override;

        procedure WriteHeader(const hdr: TMachHeader); override;
        procedure WriteSegmentCmd(const seg: TMachoSegment; acmdSize: LongWord); override;
        procedure WriteSection(const sec: TMachoSection); override;
        procedure WriteRoutineCmd(const rt: TMachoRoutine); override;
        procedure WriteNList(const list: nlist_64); override;
      end;


    { TLEMachoStructConverter }

    { converter for Little-endian structures to Host }
    TLEMachoStructConverter = class(TObject)
      public
        procedure ConvertMachoHeader(const mh: mach_header; var hdr: TMachHeader); virtual;
        procedure ConvertMachoHeader64(const mh: mach_header_64; var hdr: TMachHeader); virtual;
        procedure ConvertLoadCommand(var cmd: load_command); virtual;
        procedure ConvertSegment(const segcmd: segment_command; var segment: TMachoSegment); virtual;
        procedure ConvertSegment64(const segcmd: segment_command_64; var segment: TMachoSegment); virtual;
        procedure ConvertSection(const sec: section; var section: TMachoSection); virtual;
        procedure ConvertSection64(const sec: section_64; var section: TMachoSection); virtual;

        procedure ConvertUInt16(var v: Word); virtual;
        procedure ConvertUInt32(var v: LongWord); virtual;
        procedure ConvertUInt64(var v: qWord); virtual;
      end;

    { converter for Big-endian structures to Host }
    TBEMachoStructConverter = class(TLEMachoStructConverter);

    {common}
    TMachoStructConverter = TLEMachoStructConverter;

    { TMachoReader }

    TMachoReader=class(TObject)
      private
        fReader : TRawReader;
        HdrOfs  : qword;
        fCnv    : TMachoStructConverter;
        fHdr    : TMachHeader;
        is64    : Boolean;
        cmdofs  : array of qword;
        cmds    : array of load_command;
      protected
        function IntReadStruct: Boolean;
      public
        constructor Create(ARawReader: TRawReader; StartOfs: QWord=0);
        function ReadHeader(var hdr: TMachHeader): Boolean;
        function ReadCommandID(index: LongWord; var cmd: load_command): Boolean;
        function GetCmdOfs(index: LongWord): qword;

        function ReadSegmentCommand(cmdindex: LongWord; var segment: TMachoSegment): Boolean;
        function ReadSection(segindex, secindex: LongWord; var machsection: TMachoSection): Boolean;

        function ReadSymTabCmd(var symcmd: symtab_command): Boolean;

        function ReadUInt32(var v: LongWord): Boolean;
        function ReadData(var data; dataSize: Integer): Integer;

        {todo: ReadNList - using index of symbol, instead of file offset?}
        function GetNListSize: Integer;
        function ReadNList(fileofs: qword; var nsym: nlist_64): Boolean;

        procedure Seek(apos: qword);
      end;

    const
      seg_TEXT   : TSegmentName = '__TEXT';
      seg_DATA   : TSegmentName = '__DATA';
      seg_OBJC   : TSegmentName = '__OBJC';
      seg_IMPORT : TSegmentName = '__IMPORT';
      seg_DWARF  : TSegmentName = '__DWARF';

    function AllocMachoWriter(cputarget: cpu_type_t; ARawWriter: TRawWriter; AllowFreeWriter: Boolean): TMachoWriter;

    function sizeMachHeader(cputarget: cpu_type_t): integer; inline;
    function sizeSegment(cputarget: cpu_type_t): integer; inline;
    function sizeSection(cputarget: cpu_type_t): integer; inline;
    function sizeNList(cputarget: cpu_type_t): integer; inline;

    function AlignAddr(cputarget: cpu_type_t; addr: qword): qword;

    procedure InitSegment(var seg: TMachoSegment);

    function MakeSectionName(const segName, secName: shortstring): shortstring;
    function GetSegmentSectionName(const objSecName: shortstring; var segName, secName: shortstring): shortstring;

    function GetSectionFlags(const segName, secName: shortstring): LongWord;

    type
      TRelocInfoLength = (ril_byte = 0, ril_word = 1, ril_long = 2, ril_quad = 3);

    procedure RelocInfo(addr, symnum, reltype: integer; len: TRelocInfoLength; pcrel, extern: Boolean; var info: relocation_info);
    procedure ScatterRelocInfo(value, addr, reltype: integer; len: TRelocInfoLength; pcrel: Boolean; var info: scattered_relocation_info);

    function GetReserved1(const macho: TMachoSection): integer;
    function GetReserved2(const macho: TMachoSection): integer;

    function GetStubSize(cputarget: Integer; Pic: Boolean): Integer;
    function MachoAlign(al: integer): integer;

implementation

  function MachoAlign(al: integer): integer;
    begin
      Result:=0;
      al:=al shr 1;
      while al>0 do
        begin
          inc(Result);
          al:=al shr 1;
        end;
    end;


  function AllocConverter(magic: LongWord): TMachoStructConverter;
    begin
      {$ifdef ENDIAN_BIG}
      if magic=MH_MAGIC then
        Result:=TBEMachoStructConverter.Create
      else
        Result:=TLEMachoStructConverter.Create;
      {$else}
      if magic=MH_MAGIC then
        Result:=TLEMachoStructConverter.Create
      else
        Result:=TBEMachoStructConverter.Create;
      {$endif}
    end;


  {result values are used from aggas.pas, see TGNUAssembler.WriteSection }
  function GetStubSize(cputarget: Integer; Pic: Boolean): Integer;
    begin
      case cputarget of
        CPU_TYPE_I386,  CPU_TYPE_X86_64:
          Result:=5;
        CPU_TYPE_POWERPC, CPU_TYPE_POWERPC64:
          if Pic then
            Result:=32
          else
            Result:=16;
        CPU_TYPE_ARM:
          if Pic then
            Result:=16
          else
            Result:=12;
      else
        Result:=-1;
      end;
    end;


  function GetReserved1(const macho: TMachoSection): integer;
    begin
      case macho.flags and SECTION_TYPE of
        S_NON_LAZY_SYMBOL_POINTERS, S_LAZY_SYMBOL_POINTERS:
          Result:=macho.indirectIndex;
      else
        Result:=0;
      end;
    end;


  function GetReserved2(const macho: TMachoSection): integer;
    begin
      case macho.flags and SECTION_TYPE of
        S_SYMBOL_STUBS:
          Result:=macho.stubSize
      else
        Result:=0;
      end;
    end;


  procedure RelocInfo(addr, symnum, reltype: integer; len: TRelocInfoLength; pcrel, extern: Boolean; var info: relocation_info);
    {$ifdef ENDIAN_BIG}
    const
      relbit : array [Boolean] of Integer = (0, 1 shl 7);
      extbit : array [Boolean] of Integer = (0, 1 shl 4);
      ri_len_mask : array [TRelocInfoLength] of Integer = (0 shl 5, 1 shl 5, 2 shl 5, 3 shl 5);
    begin
      info.r_address:=addr;
      info.r_info:=((symnum and $FFFFFF) shl 8) or // r_symbolnum:24
                    relbit[pcrel] or               // r_pcrel:1;
                    ri_len_mask[len] or            // r_length:2;
                    extbit[extern] or              // r_extern:1;
                    (reltype and $F);              // r_type:4;
    end;
    {$else}
    const
      relbit : array [Boolean] of Integer = (0, 1 shl 24);
      extbit : array [Boolean] of Integer = (0, 1 shl 27);
      ri_len_mask : array [TRelocInfoLength] of Integer = (0 shl 25, 1 shl 25, 2 shl 25, 3 shl 25);
    begin
      info.r_address:=addr;
      info.r_info:=(symnum and $FFFFFF) or // r_symbolnum:24
                   relbit[pcrel] or        // r_pcrel:1;
                   extbit[extern] or       // r_length:2;
                   ri_len_mask[len] or     // r_extern:1;
                   (reltype shl 28);       // r_type:4;
    end;
    {$endif}


const
    si_len_mask: array [TRelocInfoLength] of Integer = (0 shl 28, 1 shl 28, 2 shl 28, 3 shl 28);
    si_type_ofs    = 24;
    si_pcrel_bit   = 1 shl 30;
    si_scatter_bit = 1 shl 31;
    si_addr_ofs    = 0;


  procedure ScatterRelocInfo(value, addr, reltype: integer; len: TRelocInfoLength; pcrel: Boolean; var info: scattered_relocation_info);
    const
      relbit : array [Boolean] of Integer = (0, si_pcrel_bit);
    begin
                                                          // big endian
      info.r_info:=si_scatter_bit or                      // r_scattered:1,	/* 1=scattered, 0=non-scattered (see above) */
                   relbit[pcrel] or                       // r_pcrel:1,   	/* was relocated pc relative already */
                   si_len_mask[len] or                    // r_length:2,    /* 0=byte, 1=word, 2=long, 3=quad */
                   ((reltype and $F) shl si_type_ofs) or  // r_type:4,   	  /* if not 0, machine specific relocation type */
                   ((addr and $FFFFFF) shl si_addr_ofs);  // r_address:24;	/* offset in the section to what is being relocated */}
      info.r_value:=value;
                                                          // little endian
                                                          // r_address:24,	  /* offset in the section to what is being relocated */
                                                          // r_type:4,	      /* if not 0, machine specific relocation type */
                                                          // r_length:2,	    /* 0=byte, 1=word, 2=long, 3=quad */
                                                          // r_pcrel:1, 	    /* was relocated pc relative already */
                                                          // r_scattered:1;	/* 1=scattered, 0=non-scattered (see above) */   *}
    end;


  function GetSectionFlags(const segName, secName: shortstring): LongWord;
    begin
      Result:=0;
      if segName = seg_DATA then
        begin
          if secName = '__nl_symbol_ptr' then
            Result:=Result or S_NON_LAZY_SYMBOL_POINTERS
          else if secName = '__la_symbol_ptr' then
            Result:=Result or S_LAZY_SYMBOL_POINTERS
          else if secName = '__common' then
            Result:=Result or S_ZEROFILL
        end
      else if segName = seg_TEXT then
        begin
          if (secName = '__text') then
            Result:=Result or S_ATTR_PURE_INSTRUCTIONS or S_ATTR_SOME_INSTRUCTIONS
          else if secName = '__textcoal_nt' then
            Result:=Result or S_ATTR_PURE_INSTRUCTIONS or S_ATTR_SOME_INSTRUCTIONS or S_COALESCED
          else if secName = '.fpc' then
            Result:=Result or S_ATTR_NO_DEAD_STRIP
          else if secName = '__cstring' then
            Result:=Result or S_CSTRING_LITERALS;
        end
      else if (segName = seg_IMPORT) then
        begin
          if (secName = '__jump_table') then
            Result:=Result or S_SYMBOL_STUBS or S_ATTR_SELF_MODIFYING_CODE or S_ATTR_SOME_INSTRUCTIONS
        end
      else if (segName=seg_OBJC) then
        begin
          Result:=S_ATTR_NO_DEAD_STRIP;
          if secName='__message_refs' then
            Result:=Result or S_ATTR_NO_DEAD_STRIP or S_LITERAL_POINTERS
          else if secName='__cls_refs' then
            Result:=Result or S_ATTR_NO_DEAD_STRIP or S_LITERAL_POINTERS;
        end;
    end;


  function MakeSectionName(const segName, secName: shortstring): shortstring;
    begin
      if segName = '' then
        Result:=secName
      else
        Result:=segName+' '+secName;
    end;


  function GetSegmentSectionName(const objSecName: shortstring; var segName, secName: shortstring): shortstring;
    var
      i : integer;
    begin
      i:=Pos(' ', objSecName);
      if i>0 then
        begin
          segName:=copy(objsecName, 1, i-1);
          secName:=copy(objsecName, i+1, length(objsecName)-i);
        end
      else
        begin
          segName:='';
          secName:=objSecName;
        end;
      Result:=objSecName;
    end;


  procedure InitSegment(var seg: TMachoSegment);
    begin
      FillChar(seg, sizeof(seg), 0);
      seg.initprot:=VM_PROT_ALL;
      seg.maxprot:=VM_PROT_ALL;
    end;


  const
    is64MachHeaderSize : array [Boolean] of Integer = ( sizeof(mach_header),  sizeof(mach_header_64));
    is64SectionSize : array [Boolean] of Integer = ( sizeof(section),  sizeof(section_64));
    is64SegmentSize : array [Boolean] of Integer = ( sizeof(segment_command),  sizeof(segment_command_64));
    is64NListSize : array [Boolean] of Integer = (sizeof(nlist), sizeof(nlist_64));

  function AlignAddr(cputarget: cpu_type_t; addr: qword): qword;
    var
      md  : array [Boolean] of integer = (4, 8);
      p   : PtrUInt;
    begin
      p:=addr;
      p:=align(p, md[cputarget and CPU_ARCH_ABI64 > 0]);
      Result:=qword(p);
    end;


  function sizeMachHeader(cputarget: cpu_type_t): integer;
    begin
      Result:=is64MachHeaderSize[ cputarget and CPU_ARCH_ABI64 > 0];
    end;


  function sizeSegment(cputarget: cpu_type_t): integer;
    begin
      Result:=is64SegmentSize[ cputarget and CPU_ARCH_ABI64 > 0];
    end;


  function sizeSection(cputarget: cpu_type_t): integer;
    begin
      Result:=is64SectionSize[ cputarget and CPU_ARCH_ABI64 > 0];
    end;


  function sizeNList(cputarget: cpu_type_t): integer; inline;
    begin
      Result:=is64NlistSize[ cputarget and CPU_ARCH_ABI64 > 0];
    end;


  function AllocMachoWriter(cputarget: cpu_type_t; ARawWriter: TRawWriter; AllowFreeWriter: Boolean): TMachoWriter;
    begin
      case cputarget of
        CPU_TYPE_I386,
        CPU_TYPE_ARM: Result:=TLE32MachoWriter.Create(ARawWriter, AllowFreeWriter);
        CPU_TYPE_X86_64: Result:=TLE64MachoWriter.Create(ARawWriter, AllowFreeWriter);
        CPU_TYPE_POWERPC:  Result:=TBE32MachoWriter.Create(ARawWriter, AllowFreeWriter);
        CPU_TYPE_POWERPC64: Result:=TBE64MachoWriter.Create(ARawWriter, AllowFreeWriter);
      else
        begin
          if AllowFreeWriter then
            ARawWriter.Free;
          Result:=nil;
        end;
      end;
    end;


  { TMachoWriter }

  procedure TMachoWriter.WriteData(const data; dataSize: Integer);
    begin
      if not assigned(fwriter) then
        Exit;
      fwriter.WriteRaw(data, dataSize);
    end;


  procedure TMachoWriter.WriteUint8(i: uint8_t);
    begin
      WriteData(i, sizeof(i));
    end;


  procedure TMachoWriter.WriteLoadCommand(cmd, cmdsize: Integer);
    var
      m : load_command;
    begin
      m.cmd:=cmd;
      m.cmdsize:=cmdsize;
      WriteLoadCommand(m);
    end;


  constructor TMachoWriter.Create(ARawWriter: TRawWriter; AllowFreeWriter: Boolean);
    begin
      inherited Create;
      fwriter:=ARawWriter;
      fown:=AllowFreeWriter;
    end;


  destructor TMachoWriter.Destroy;
    begin
      if fown then
        fwriter.Free;
      inherited Destroy;
    end;


  { TLE32MachoWriter }

  procedure TLE32MachoWriter.WriteHeader(const hdr: TMachHeader);
    var
      m : mach_header;
    begin
      with m do
        begin
          magic:=NtoLE(MH_MAGIC);
          cputype:=NtoLE(hdr.cputype);
          cpusubtype:=NtoLE(hdr.cpusubtype);
          filetype:=NtoLE(hdr.filetype);
          ncmds:=NtoLE(hdr.ncmds);
          sizeofcmds:=NtoLE(hdr.sizeofcmds);
          flags:=NtoLE(hdr.flags);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteSegmentCmd(const seg: TMachoSegment; ACmdSize: LongWord);
    var
      m : segment_command;
    begin
      with m do
        begin
          cmd:=NtoLE(LC_SEGMENT);
          cmdsize:=NtoLE(ACmdSize);
          segname:=seg.segname;
          vmaddr:=NtoLE(uint32_t(seg.vmaddr));
          vmsize:=NtoLE(uint32_t(seg.vmsize));
          fileoff:=NtoLE(uint32_t(seg.fileoff));
          filesize:=NtoLE(uint32_t(seg.filesize));
          maxprot:=NtoLE(seg.maxprot);
          initprot:=NtoLE(seg.initprot);
          nsects:=NtoLE(seg.nsects);
          flags:=NtoLE(seg.flags);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteSection(const sec: TMachoSection);
    var
      m : section;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          sectname:=sec.sectname;
          segname:=sec.segname;
          addr:=NtoLE(sec.addr);
          size:=NtoLE(sec.size);
          offset:=NtoLE(sec.offset);
          align:=NtoLE(sec.align);
          reloff:=NtoLE(sec.reloff);
          nreloc:=NtoLE(sec.nreloc);
          flags:=NtoLE(sec.flags);
          reserved1:=NtoLE( GetReserved1(sec));
          reserved2:=NtoLE( GetReserved2(sec));
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteRoutineCmd(const rt: TMachoRoutine);
    var
      m : routines_command;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          cmd:=NtoLE(LC_ROUTINES);
          cmdsize:=NtoLE(sizeof(m));
          init_address:=NtoLE(rt.init_address);
          init_module:=NtoLE(rt.init_module);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteLoadCommand(const cmd: load_command);
    var
      m : load_command;
    begin
      m.cmd:=NtoLE(cmd.cmd);
      m.cmdsize:=NtoLE(cmd.cmdsize);
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteRelocation(const ri: relocation_info);
    var
      m : relocation_info;
    begin
      m.r_address:=NtoLE(ri.r_address);
      m.r_info:=NtoLE(ri.r_info);
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteScatterReloc(const ri: scattered_relocation_info);
    var
      m : LongWord;
    begin
      m:=LongWord(ri.r_info);
      WriteUint32(NtoLE(m));

      m:=LongWord(ri.r_value);
      WriteUint32(NtoLE(m));
    end;


  procedure TLE32MachoWriter.WriteNList(const list: nlist_64);
    var
      m : nlist;
    begin
      m.n_un.n_strx:=NtoLe(list.n_un.n_strx);
      m.n_type:=NtoLe(list.n_type);
      m.n_sect:=NtoLe(list.n_sect);
      m.n_desc:=NtoLe(list.n_desc);
      m.n_value:=NtoLe(list.n_value);
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteUint16(i: uint16_t);
    var
      m: uint16_t;
    begin
      m:=NtoLE(i);
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteUint32(i: uint32_t);
    var
      m: uint32_t;
    begin
      m:=NtoLE(i);
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WriteUint64(i: uint64_t);
    var
      m: uint64_t;
    begin
      m:=NtoLE(i);
      WriteData(m, sizeof(m));
    end;


  procedure TLE32MachoWriter.WritePtr(ofs: QWord);
    var
      m: uint32_t;
    begin
      m:=NtoLE(ofs);
      WriteData(m, sizeof(m));
    end;


  { TLE64MachoWriter }

  procedure TLE64MachoWriter.WritePtr(ofs: QWord);
    var
      m : uint64_t;
    begin
      m:=NtoLE(ofs);
      Writedata(m, sizeof(m));
    end;


  procedure TLE64MachoWriter.WriteHeader(const hdr: TMachHeader);
    var
      m : mach_header_64;
    begin
      with m do
        begin
          magic:=NtoLE(MH_MAGIC_64);
          cputype:=NtoLE(hdr.cputype);
          cpusubtype:=NtoLE(hdr.cpusubtype);
          filetype:=NtoLE(hdr.filetype);
          ncmds:=NtoLE(hdr.ncmds);
          sizeofcmds:=NtoLE(hdr.sizeofcmds);
          flags:=NtoLE(hdr.flags);
          reserved:=0;
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE64MachoWriter.WriteSegmentCmd(const seg: TMachoSegment; acmdSize: LongWord);
    var
      m : segment_command_64;
    begin
      with m do
        begin
          cmd:=NtoLE(LC_SEGMENT_64);
          cmdsize:=NtoLE(acmdSize);
          segname:=seg.segname;
          vmaddr:=NtoLE(seg.vmaddr);
          vmsize:=NtoLE(seg.vmsize);
          fileoff:=NtoLE(seg.fileoff);
          filesize:=NtoLE(seg.filesize);
          maxprot:=NtoLE(seg.maxprot);
          initprot:=NtoLE(seg.initprot);
          nsects:=NtoLE(seg.nsects);
          flags:=NtoLE(seg.flags);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE64MachoWriter.WriteSection(const sec: TMachoSection);
    var
      m : section_64;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          sectname:=sec.sectname;
          segname:=sec.segname;
          addr:=NtoLE(sec.addr);
          size:=NtoLE(sec.size);
          offset:=NtoLE(sec.offset);
          align:=NtoLE(sec.align);
          reloff:=NtoLE(sec.reloff);
          nreloc:=NtoLE(sec.nreloc);
          flags:=NtoLE(sec.flags);
          reserved1:=NtoLE( GetReserved1(sec));
          reserved2:=NtoLE( GetReserved2(sec));
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE64MachoWriter.WriteRoutineCmd(const rt: TMachoRoutine);
    var
      m : routines_command_64;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          cmd:=NtoLE(LC_ROUTINES);
          cmdsize:=NtoLE(sizeof(m));
          init_address:=NtoLE(rt.init_address);
          init_module:=NtoLE(rt.init_module);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TLE64MachoWriter.WriteNList(const list: nlist_64);
    var
      m : nlist_64;
    begin
      m.n_un.n_strx:=NtoLe(list.n_un.n_strx);
      m.n_type:=NtoLe(list.n_type);
      m.n_sect:=NtoLe(list.n_sect);
      m.n_desc:=NtoLe(list.n_desc);
      m.n_value:=NtoLe(list.n_value);
      WriteData(m, sizeof(m));
    end;


  { TBE32MachoWriter }

  procedure TBE32MachoWriter.WriteHeader(const hdr: TMachHeader);
    var
      m : mach_header;
    begin
      with m do
        begin
          magic:=NtoBE(MH_MAGIC);
          cputype:=NtoBE(hdr.cputype);
          cpusubtype:=NtoBE(hdr.cpusubtype);
          filetype:=NtoBE(hdr.filetype);
          ncmds:=NtoBE(hdr.ncmds);
          sizeofcmds:=NtoBE(hdr.sizeofcmds);
          flags:=NtoBE(hdr.flags);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteSegmentCmd(const seg: TMachoSegment; acmdSize: LongWord);
    var
      m : segment_command;
    begin
      with m do
        begin
          cmd:=NtoBE(LC_SEGMENT);
          cmdsize:=NtoBE(acmdSize);
          segname:=seg.segname;
          vmaddr:=NtoBE(uint32_t(seg.vmaddr));
          vmsize:=NtoBE(uint32_t(seg.vmsize));
          fileoff:=NtoBE(uint32_t(seg.fileoff));
          filesize:=NtoBE(uint32_t(seg.filesize));
          maxprot:=NtoBE(seg.maxprot);
          initprot:=NtoBE(seg.initprot);
          nsects:=NtoBE(seg.nsects);
          flags:=NtoBE(seg.flags);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteSection(const sec: TMachoSection);
    var
      m : section;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          sectname:=sec.sectname;
          segname:=sec.segname;
          addr:=NtoBE(sec.addr);
          size:=NtoBE(sec.size);
          offset:=NtoBE(sec.offset);
          align:=NtoBE(sec.align);
          reloff:=NtoBE(sec.reloff);
          nreloc:=NtoBE(sec.nreloc);
          flags:=NtoBE(sec.flags);
          reserved1:=NtoBE( GetReserved1(sec));
          reserved2:=NtoBE( GetReserved2(sec));
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteRoutineCmd(const rt: TMachoRoutine);
    var
      m : routines_command;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          cmd:=NtoBE(LC_ROUTINES);
          cmdsize:=NtoBE(sizeof(m));
          init_address:=NtoBE(rt.init_address);
          init_module:=NtoBE(rt.init_module);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteLoadCommand(const cmd: load_command);
    var
      m : load_command;
    begin
      m.cmd:=NtoBE(cmd.cmd);
      m.cmdsize:=NtoBE(cmd.cmdsize);
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteRelocation(const ri: relocation_info);
    var
      m : relocation_info;
    begin
      m.r_address:=NtoBE(ri.r_address);
      m.r_info:=NtoBE(ri.r_info);
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteScatterReloc(const ri: scattered_relocation_info);
    var
      m : LongWord;
    begin
      m:=LongWord(ri.r_info);
      WriteUint32(NtoBE(m));

      m:=LongWord(ri.r_value);
      WriteUint32(NtoBE(m));
    end;


  procedure TBE32MachoWriter.WriteNList(const list: nlist_64);
    var
      m : nlist;
    begin
      m.n_un.n_strx:=NtoBe(list.n_un.n_strx);
      m.n_type:=NtoBe(list.n_type);
      m.n_sect:=NtoBe(list.n_sect);
      m.n_desc:=NtoBe(list.n_desc);
      m.n_value:=NtoBe(list.n_value);
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteUint16(i: uint16_t);
    var
      m: uint16_t;
    begin
      m:=NtoBE(i);
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteUint32(i: uint32_t);
    var
      m: uint32_t;
    begin
      m:=NtoBE(i);
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WriteUint64(i: uint64_t);
    var
      m: uint64_t;
    begin
      m:=NtoBE(i);
      WriteData(m, sizeof(m));
    end;


  procedure TBE32MachoWriter.WritePtr(ofs: QWord);
    var
      m: uint32_t;
    begin
      m:=NtoBE(ofs);
      WriteData(m, sizeof(m));
    end;


  { TBE64MachoWriter }

  procedure TBE64MachoWriter.WritePtr(ofs: QWord);
    var
      m : uint64_t;
    begin
      m:=NtoBE(ofs);
      Writedata(m, sizeof(m));
    end;


  procedure TBE64MachoWriter.WriteHeader(const hdr: TMachHeader);
    var
      m : mach_header_64;
    begin
      with m do
        begin
          magic:=NtoBE(MH_MAGIC_64);
          cputype:=NtoBE(hdr.cputype);
          cpusubtype:=NtoBE(hdr.cpusubtype);
          filetype:=NtoBE(hdr.filetype);
          ncmds:=NtoBE(hdr.ncmds);
          sizeofcmds:=NtoBE(hdr.sizeofcmds);
          flags:=NtoBE(hdr.flags);
          reserved:=0;
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE64MachoWriter.WriteSegmentCmd(const seg: TMachoSegment; acmdSize: LongWord);
    var
      m : segment_command_64;
    begin
      with m do
        begin
          cmd:=NtoBE(LC_SEGMENT_64);
          cmdsize:=NtoBE(acmdSize);
          segname:=seg.segname;
          vmaddr:=NtoBE(seg.vmaddr);
          vmsize:=NtoBE(seg.vmsize);
          fileoff:=NtoBE(seg.fileoff);
          filesize:=NtoBE(seg.filesize);
          maxprot:=NtoBE(seg.maxprot);
          initprot:=NtoBE(seg.initprot);
          nsects:=NtoBE(seg.nsects);
          flags:=NtoBE(seg.flags);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE64MachoWriter.WriteSection(const sec: TMachoSection);
    var
      m : section_64;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          sectname:=sec.sectname;
          segname:=sec.segname;
          addr:=NtoBE(sec.addr);
          size:=NtoBE(sec.size);
          offset:=NtoBE(sec.offset);
          align:=NtoBE(sec.align);
          reloff:=NtoBE(sec.reloff);
          nreloc:=NtoBE(sec.nreloc);
          flags:=NtoBE(sec.flags);
          reserved1:=NtoBE( GetReserved1(sec));
          reserved2:=NtoBE( GetReserved2(sec));
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE64MachoWriter.WriteRoutineCmd(const rt: TMachoRoutine);
    var
      m : routines_command_64;
    begin
      FillChar(m, sizeof(m), 0);
      with m do
        begin
          cmd:=NtoBE(LC_ROUTINES);
          cmdsize:=NtoBE(sizeof(m));
          init_address:=NtoBE(rt.init_address);
          init_module:=NtoBE(rt.init_module);
        end;
      WriteData(m, sizeof(m));
    end;


  procedure TBE64MachoWriter.WriteNList(const list: nlist_64);
    var
      m : nlist_64;
    begin
      m.n_un.n_strx:=NtoBe(list.n_un.n_strx);
      m.n_type:=NtoBe(list.n_type);
      m.n_sect:=NtoBe(list.n_sect);
      m.n_desc:=NtoBe(list.n_desc);
      m.n_value:=NtoBe(list.n_value);
      WriteData(m, sizeof(m));
    end;


  { TMachoReader }

  constructor TMachoReader.Create(ARawReader: TRawReader; StartOfs: QWord=0);
    begin
      inherited Create;
      fReader:=ARawReader;
      hdrofs:=StartOfs;
    end;


  function TMachoReader.IntReadStruct: Boolean;
    var
      m   : mach_header_64;
      i   : Integer;
      p   : qword;
    begin
      Result:=false;
      if not fReader.Seek(hdrofs) then
        Exit;
      //todo:
      fReader.ReadRaw(m, sizeof(mach_header_64));
      fCnv:=AllocConverter(m.magic);
      fCnv.ConvertMachoHeader(pmach_header(@m)^, fhdr);
      is64:=fhdr.cputype and CPU_ARCH_ABI64>0;
      Result:=true;

      SetLength(cmds, fHdr.ncmds);
      if fHdr.ncmds>0 then
        begin
          if is64 then
            p:=sizeof(mach_header_64)
          else
            p:=sizeof(mach_header);

          SetLength(cmdofs, fHdr.ncmds);
          for i:=0 to fHdr.ncmds - 1 do
            begin
              cmdofs[i]:=p;
              fReader.Seek(p);
              fReader.ReadRaw(cmds[i], sizeof(cmds[i]));
              fCnv.ConvertLoadCommand(cmds[i]);
              inc(p, cmds[i].cmdsize);
            end;
        end;
    end;


  function TMachoReader.ReadHeader(var hdr: TMachHeader): Boolean;
    begin
      if not Assigned(fCnv) then
        Result:=IntReadStruct
      else
        Result:=true;
      hdr:=fhdr;
    end;


  function TMachoReader.ReadCommandID(index: LongWord; var cmd: load_command): Boolean;
    begin
      if not Assigned(fCnv) then
        IntReadStruct;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Result:={(index>=0) and }(index<fHdr.ncmds);
=======
      Result:=(index>=0) and (index<fHdr.ncmds);
>>>>>>> graemeg/cpstrnew
=======
      Result:=(index>=0) and (index<fHdr.ncmds);
>>>>>>> graemeg/cpstrnew
=======
      Result:=(index>=0) and (index<fHdr.ncmds);
>>>>>>> graemeg/cpstrnew
=======
      Result:=(index>=0) and (index<fHdr.ncmds);
>>>>>>> origin/cpstrnew
      if not Result then
        Exit;
      Result:=true;
      cmd:=cmds[index];
    end;


  function TMachoReader.ReadSegmentCommand(cmdindex: LongWord; var segment: TMachoSegment): Boolean;
    var
      seg64 : segment_command_64;
      seg32 : segment_command;
    begin
      if not Assigned(fCnv) then
        IntReadStruct;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Result:={(cmdindex>=0) and }
=======
      Result:=(cmdindex>=0) and
>>>>>>> graemeg/cpstrnew
=======
      Result:=(cmdindex>=0) and
>>>>>>> graemeg/cpstrnew
=======
      Result:=(cmdindex>=0) and
>>>>>>> graemeg/cpstrnew
=======
      Result:=(cmdindex>=0) and
>>>>>>> origin/cpstrnew
              (cmdindex<fHdr.ncmds) and
              (cmds[cmdindex].cmd in [LC_SEGMENT, LC_SEGMENT_64]);

      if Result then
      begin
        fReader.Seek(cmdofs[cmdindex]);
        if is64 then
          begin
            Result:=fReader.ReadRaw(seg64, sizeof(seg64))=sizeof(seg64);
            if Result then
              fCnv.ConvertSegment64(seg64, segment);
          end
        else
          begin
            Result:=fReader.ReadRaw(seg32, sizeof(seg32))=sizeof(seg32);
            if Result then
              fCnv.ConvertSegment(seg32, segment);
          end;
      end;
    end;


  function TMachoReader.GetCmdOfs(index: LongWord): qword;
    begin
      if not Assigned(fCnv) then
        IntReadStruct;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      if {(index<0) or}
=======
      if (index<0) or
>>>>>>> graemeg/cpstrnew
=======
      if (index<0) or
>>>>>>> graemeg/cpstrnew
=======
      if (index<0) or
>>>>>>> graemeg/cpstrnew
=======
      if (index<0) or
>>>>>>> origin/cpstrnew
         (index>=longword(length(cmdofs))) then
        Result:=0
      else
        Result:=cmdofs[index];
    end;


  function TMachoReader.ReadSection(segindex, secindex: LongWord; var machsection: TMachoSection): Boolean;
    var
      ofs       : qword;
      is64bit   : Boolean;
      buf       : array [0..sizeof(section_64)-1] of byte;
    const
      sectsize : array[Boolean] of LongWord = ( sizeof(macho.section), sizeof(macho.section_64));
      segsize  : array[Boolean] of LongWord = ( sizeof(macho.segment_command), sizeof(macho.segment_command_64));
    begin
      if not Assigned(fCnv) then
        IntReadStruct;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      Result:={(secindex>=0) and (segindex>=0) and }(segindex<fHdr.ncmds) and (cmds[segindex].cmd in [LC_SEGMENT, LC_SEGMENT_64]);
=======
      Result:=(secindex>=0) and (segindex>=0) and (segindex<fHdr.ncmds) and (cmds[segindex].cmd in [LC_SEGMENT, LC_SEGMENT_64]);
>>>>>>> graemeg/cpstrnew
=======
      Result:=(secindex>=0) and (segindex>=0) and (segindex<fHdr.ncmds) and (cmds[segindex].cmd in [LC_SEGMENT, LC_SEGMENT_64]);
>>>>>>> graemeg/cpstrnew
=======
      Result:=(secindex>=0) and (segindex>=0) and (segindex<fHdr.ncmds) and (cmds[segindex].cmd in [LC_SEGMENT, LC_SEGMENT_64]);
>>>>>>> graemeg/cpstrnew
=======
      Result:=(secindex>=0) and (segindex>=0) and (segindex<fHdr.ncmds) and (cmds[segindex].cmd in [LC_SEGMENT, LC_SEGMENT_64]);
>>>>>>> origin/cpstrnew
      if not Result then
        Exit;

      is64bit:=cmds[segindex].cmd=LC_SEGMENT_64;
      Result:=secindex<(cmds[segindex].cmdsize-segsize[is64bit]) div sectsize[is64bit];
      if not Result then
        Exit;

      ofs:=cmdofs[segindex]+segsize[is64bit]+sectsize[is64]*secindex;
      fReader.Seek(ofs);
      fReader.ReadRaw(buf, segsize[is64bit]);
      if is64bit then
        fCnv.ConvertSection64( psection_64(@buf)^, machsection)
      else
        fCnv.ConvertSection( psection(@buf)^, machsection);
    end;


  function TMachoReader.ReadUInt32(var v: LongWord): Boolean;
    begin
      if not Assigned(fCnv) then
        IntReadStruct;
      Result:=Assigned(fCnv) and (fReader.ReadRaw(v, sizeof(v))=sizeof(v));
      if Result then
        fCnv.ConvertUint32(v);
    end;


  function TMachoReader.ReadData(var data; dataSize: Integer): Integer;
    begin
      Result:=fReader.ReadRaw(data, dataSize);
    end;


  function TMachoReader.GetNListSize: Integer;
    begin
      if is64 then
        Result:=sizeof(nlist_64)
      else
        Result:=sizeof(nlist);
    end;


  function TMachoReader.ReadNList(fileofs: qword; var nsym: nlist_64): Boolean;
    var
      n32 : nlist;
    begin
      fReader.Seek(fileofs);
      if is64 then
        Result:=fReader.ReadRaw(nsym, sizeof(nlist_64))=sizeof(nlist_64)
      else
        begin
          Result:=fReader.ReadRaw(n32, sizeof(nlist))=sizeof(nlist);
          nsym.n_un.n_strx:=n32.n_un.n_strx;
          nsym.n_desc:=n32.n_desc;
          nsym.n_sect:=n32.n_sect;
          nsym.n_type:=n32.n_type;
          nsym.n_value:=n32.n_value;
        end;
      fCnv.ConvertUInt32(nsym.n_un.n_strx);
      fCnv.ConvertUInt16(nsym.n_desc);
      fCnv.ConvertUInt64(nsym.n_value);
    end;


  function TMachoReader.ReadSymTabCmd(var symcmd: symtab_command): Boolean;
    var
      i   : Integer;
      p   : qword;
    begin
      if not Assigned(fCnv) then
        IntReadStruct;

      for i:=0 to length(cmds)-1 do
        if cmds[i].cmd=LC_SYMTAB then
          begin
            p:=fReader.ReadPos;
            fReader.Seek(cmdofs[i]);
            fReader.ReadRaw(symcmd, sizeof(symcmd));
            fCnv.ConvertUInt32(symcmd.cmd);
            fCnv.ConvertUInt32(symcmd.cmdsize);
            fCnv.ConvertUInt32(symcmd.symoff);
            fCnv.ConvertUInt32(symcmd.nsyms);
            fCnv.ConvertUInt32(symcmd.stroff);
            fCnv.ConvertUInt32(symcmd.strsize);
            fReader.Seek(p);
            Result:=true;
            Exit;
          end;
      Result:=false;
    end;


  procedure TMachoReader.Seek(apos: qword);
    begin
      fReader.Seek(apos);
    end;


  { TLEMachoStructConverter }

  procedure TLEMachoStructConverter.ConvertMachoHeader(const mh: mach_header; var hdr: TMachHeader);
    begin
      hdr.cputype:=LEToN(mh.cputype);
      hdr.cpusubtype:=LEtoN(mh.cpusubtype);
      hdr.filetype:=LEToN(mh.filetype);
      hdr.ncmds:=LEToN(mh.ncmds);
      hdr.sizeofcmds:=LEToN(mh.ncmds);
      hdr.flags:=LEToN(mh.flags);
    end;


  procedure TLEMachoStructConverter.ConvertMachoHeader64(const mh: mach_header_64; var hdr: TMachHeader);
    begin
      hdr.cputype:=LEToN(mh.cputype);
      hdr.cpusubtype:=LEtoN(mh.cpusubtype);
      hdr.filetype:=LEToN(mh.filetype);
      hdr.ncmds:=LEToN(mh.ncmds);
      hdr.sizeofcmds:=LEToN(mh.ncmds);
      hdr.flags:=LEToN(mh.flags);
    end;


  procedure TLEMachoStructConverter.ConvertLoadCommand(var cmd: load_command);
    begin
      cmd.cmd:=LEToN(cmd.cmd);
      cmd.cmdsize:=LEToN(cmd.cmdsize);
    end;


  procedure TLEMachoStructConverter.ConvertSegment(const segcmd: segment_command; var segment: TMachoSegment);
    begin
      FillChar(segment, sizeof(segment), 0);
      segment.segname:=segcmd.segname;
      segment.vmaddr:=LEToN(segcmd.vmaddr);
      segment.vmsize:=LEToN(segcmd.vmsize);
      segment.fileoff:=LEToN(segcmd.fileoff);
      segment.filesize:=LEToN(segcmd.filesize);
      segment.maxprot:=LEToN(segcmd.maxprot);
      segment.initprot:=LEToN(segcmd.initprot);
      writelN('segcmd.nsects = ', segcmd.nsects);
      segment.nsects:=LEToN(segcmd.nsects);
      segment.flags:=LEToN(segcmd.flags);
      //todo: reserved!?
    end;


  procedure TMachoStructConverter.ConvertSegment64(const segcmd: segment_command_64; var segment: TMachoSegment);
    begin
      FillChar(segment, sizeof(segment), 0);
      segment.segname:=segcmd.segname;
      segment.vmaddr:=LEToN(segcmd.vmaddr);
      segment.vmsize:=LEToN(segcmd.vmsize);
      segment.fileoff:=LEToN(segcmd.fileoff);
      segment.filesize:=LEToN(segcmd.filesize);
      segment.maxprot:=LEToN(segcmd.maxprot);
      segment.initprot:=LEToN(segcmd.initprot);
      segment.nsects:=LEToN(segcmd.nsects);
      segment.flags:=LEToN(segcmd.flags);
      //todo: reserved!?
    end;


  procedure TMachoStructConverter.ConvertSection(const sec: section; var section: TMachoSection);
    begin
      FillChar(section, sizeof(section), 0);
      section.sectname:=sec.sectname;
      section.segname:=sec.segname;
      section.addr:=LEToN(sec.addr);
      section.size:=LEToN(sec.size);
      section.offset:=LEToN(sec.offset);
      section.align:=LEToN(sec.align);
      section.reloff:=LEToN(sec.reloff);
      section.nreloc:=LEToN(sec.nreloc);
      section.flags:=LEToN(sec.flags);
      //todo:
      //section.indirectIndex  : Integer; // reserved1 for LAZY and NON_LAZY pointers
      //section.stubSize       : Integer; // reserved2 for S_SYMBOL_STUBS
    end;


  procedure TMachoStructConverter.ConvertSection64(const sec: section_64; var section: TMachoSection);
    begin
      FillChar(section, sizeof(section), 0);
      section.sectname:=sec.sectname;
      section.segname:=sec.segname;
      section.addr:=LEToN(sec.addr);
      section.size:=LEToN(sec.size);
      section.offset:=LEToN(sec.offset);
      section.align:=LEToN(sec.align);
      section.reloff:=LEToN(sec.reloff);
      section.nreloc:=LEToN(sec.nreloc);
      section.flags:=LEToN(sec.flags);
      //todo:
      //section.indirectIndex  : Integer; // reserved1 for LAZY and NON_LAZY pointers
      //section.stubSize       : Integer; // reserved2 for S_SYMBOL_STUBS
    end;


  procedure TMachoStructConverter.ConvertUInt32(var v: LongWord);
    begin
      v:=LEtoN(v);
    end;


  procedure TMachoStructConverter.ConvertUInt64(var v: qword);
    begin
      v:=LEtoN(v);
    end;


  procedure TMachoStructConverter.ConvertUInt16(var v: Word);
    begin
      v:=LEToN(v);
    end;

end.

