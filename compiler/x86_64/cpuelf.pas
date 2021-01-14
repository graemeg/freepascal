{
    Copyright (c) 1998-2006 by Peter Vreman

    Includes ELF-related code specific to x86_64

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
unit cpuelf;

{$i fpcdefs.inc}

interface

implementation

  uses
    globtype,cutils,cclasses,
    verbose,elfbase,
    systems,aasmbase,ogbase,ogelf,assemble;

  type
    TElfExeOutputx86_64=class(TElfExeOutput)
    private
      procedure MaybeWriteGOTEntry(relocval:aint;objsym:TObjSymbol);
      procedure MaybeWriteTLSIEGotEntry(relocval:aint;objsym:TObjSymbol);
    protected
      procedure WriteFirstPLTEntry;override;
      procedure WritePLTEntry(exesym:TExeSymbol);override;
      procedure WriteIndirectPLTEntry(exesym:TExeSymbol);override;
      procedure GOTRelocPass1(objsec:TObjSection;var idx:longint);override;
      procedure DoRelocationFixup(objsec:TObjSection);override;
    end;

  const
    { Relocation types }
    R_X86_64_NONE = 0;
    R_X86_64_64 = 1;                 { Direct 64 bit   }
    R_X86_64_PC32 = 2;               { PC relative 32 bit signed  }
    R_X86_64_GOT32 = 3;              { 32 bit GOT entry  }
    R_X86_64_PLT32 = 4;              { 32 bit PLT address  }
    R_X86_64_COPY = 5;               { Copy symbol at runtime  }
    R_X86_64_GLOB_DAT = 6;           { Create GOT entry  }
    R_X86_64_JUMP_SLOT = 7;          { Create PLT entry  }
    R_X86_64_RELATIVE = 8;           { Adjust by program base  }
    R_X86_64_GOTPCREL = 9;           { 32 bit signed PC relative offset to GOT  }
    R_X86_64_32 = 10;                { Direct 32 bit zero extended  }
    R_X86_64_32S = 11;               { Direct 32 bit sign extended  }
    R_X86_64_16 = 12;                { Direct 16 bit zero extended  }
    R_X86_64_PC16 = 13;              { 16 bit sign extended PC relative  }
    R_X86_64_8 = 14;                 { Direct 8 bit sign extended   }
    R_X86_64_PC8 = 15;               { 8 bit sign extended PC relative  }
    R_X86_64_DTPMOD64 = 16;          { ID of module containing symbol  }
    R_X86_64_DTPOFF64 = 17;          { Offset in module's TLS block  }
    R_X86_64_TPOFF64 = 18;           { Offset in initial TLS block  }
    { 32 bit signed PC relative offset to two GOT entries for GD symbol  }
    R_X86_64_TLSGD = 19;
    { 32 bit signed PC relative offset to two GOT entries for LD symbol  }
    R_X86_64_TLSLD = 20;
    R_X86_64_DTPOFF32 = 21;          { Offset in TLS block  }
    { 32 bit signed PC relative offset to GOT entry for IE symbol  }
    R_X86_64_GOTTPOFF = 22;
    R_X86_64_TPOFF32 = 23;           { Offset in initial TLS block  }
    R_X86_64_PC64 = 24;              { PC relative 64-bit signed }
    R_X86_64_GOTOFF64 = 25;          { 64-bit offset from GOT base }
    R_X86_64_GOTPC32 = 26;           { PC-relative offset GOT }
    R_X86_64_GOT64  = 27;            { 64-bit GOT entry offset }
    R_X86_64_GOTPCREL64 = 28;        { 64-bit PC relative offset to GOT entry }
    R_X86_64_GOTPC64 = 29;           { 64-bit PC relative offset to GOT }
    R_X86_64_GOTPLT64 = 30;          { Like GOT64, indicates that PLT entry needed }
    R_X86_64_PLTOFF64 = 31;          { 64-bit GOT relative offset to PLT entry }
    R_X86_64_SIZE32 = 32;
    R_X86_64_SIZE64 = 33;
    R_X86_64_GOTPC32_TLSDESC = 34;
    R_X86_64_TLSDESC_CALL = 35;
    R_X86_64_TLSDESC = 36;
    R_X86_64_IRELATIVE = 37;
    R_X86_64_GOTPCRELX =41;
    R_X86_64_REX_GOTPCRELX =42;
    R_X86_64_GNU_VTINHERIT = 250;    { GNU extension to record C++ vtable hierarchy }
    R_X86_64_GNU_VTENTRY = 251;      { GNU extension to record C++ vtable member usage }

  type
    TRelocProp=record
      name: PChar;
      size: byte;
    end;

  const
    relocprops: array[0..37] of TRelocProp=(
      (name: 'R_X86_64_NONE';  size:0),
      (name: 'R_X86_64_64';    size:8),
      (name: 'R_X86_64_PC32';  size:4),
      (name: 'R_X86_64_GOT32'; size:4),
      (name: 'R_X86_64_PLT32'; size:4),
      (name: 'R_X86_64_COPY';  size:0),
      (name: 'R_X86_64_GLOB_DAT'; size:8),
      (name: 'R_X86_64_JUMP_SLOT';size:8),
      (name: 'R_X86_64_RELATIVE'; size:8),
      (name: 'R_X86_64_GOTPCREL'; size:4),
      (name: 'R_X86_64_32';    size:4),
      (name: 'R_X86_64_32S';   size:4),
      (name: 'R_X86_64_16';    size:2),
      (name: 'R_X86_64_PC16';  size:2),
      (name: 'R_X86_64_8';     size:1),
      (name: 'R_X86_64_PC8';   size:1),
      (name: 'R_X86_64_DTPMOD64'; size:8),
      (name: 'R_X86_64_DTPOFF64'; size:8),
      (name: 'R_X86_64_TPOFF64';  size:8),
      (name: 'R_X86_64_TLSGD';    size:4),
      (name: 'R_X86_64_TLSLD';    size:4),
      (name: 'R_X86_64_DTPOFF32'; size:4),
      (name: 'R_X86_64_GOTTPOFF'; size:4),
      (name: 'R_X86_64_TPOFF32';  size:4),
      (name: 'R_X86_64_PC64';     size:8),
      (name: 'R_X86_64_GOTOFF64'; size:8),
      (name: 'R_X86_64_GOTPC32';  size:4),
      (name: 'R_X86_64_GOT64';    size:8),
      (name: 'R_X86_64_GOTPCREL64'; size:8),
      (name: 'R_X86_64_GOTPC64';  size:8),
      (name: 'R_X86_64_GOTPLT64'; size:8),
      (name: 'R_X86_64_PLTOFF64'; size:8),
      (name: 'R_X86_64_SIZE32';   size:4),
      (name: 'R_X86_64_SIZE64';   size:8),
      (name: 'R_X86_64_GOTPC32_TLSDESC'; size:4),
      (name: 'R_X86_64_TLSDESC_CALL';    size:0),
      (name: 'R_X86_64_TLSDESC';   size:8),
      (name: 'R_X86_64_IRELATIVE'; size:8)
    );


{****************************************************************************
                              ELF Target methods
****************************************************************************}

  function elf_x86_64_encodereloc(objrel:TObjRelocation):byte;
    begin
      case objrel.typ of
        RELOC_NONE :
          result:=R_X86_64_NONE;
      { Note: 8 and 16-bit relocations are known to be non-conformant with
        AMD64 ABI, so they aren't handled. }
        RELOC_RELATIVE :
          if objrel.size=8 then
            result:=R_X86_64_PC64
          else if objrel.size=4 then
            result:=R_X86_64_PC32
          else
            InternalError(2012061900);
        RELOC_ABSOLUTE :
          if objrel.size=8 then
            result:=R_X86_64_64
          else if objrel.size=4 then
            result:=R_X86_64_32
          else
            InternalError(2012061902);
        RELOC_ABSOLUTE32 :
          result:=R_X86_64_32S;
        RELOC_GOTPCREL :
          result:=R_X86_64_GOTPCREL;
        RELOC_GOTPCRELX :
          result:=R_X86_64_GOTPCRELX;
        RELOC_REX_GOTPCRELX :
          result:=R_X86_64_REX_GOTPCRELX;
        RELOC_PLT32 :
          result:=R_X86_64_PLT32;
        RELOC_TPOFF:
          if objrel.size=8 then
            result:=R_X86_64_TPOFF64
          else if objrel.size=4 then
            result:=R_X86_64_TPOFF32
          else
            InternalError(2019091701);
        RELOC_TLSGD:
          result:=R_X86_64_TLSGD;
        RELOC_DTPOFF:
          if objrel.size=8 then
            result:=R_X86_64_DTPOFF64
          else if objrel.size=4 then
            result:=R_X86_64_DTPOFF32
          else
            InternalError(2019091702);
        else
          result:=0;
          InternalError(2012082302);
      end;
    end;


  procedure elf_x86_64_loadreloc(objrel:TObjRelocation);
    begin
    end;


  function elf_x86_64_relocname(reltyp:byte):string;
    begin
      if reltyp<=high(relocprops) then
        result:=relocprops[reltyp].name
      else
        result:='unknown ('+tostr(reltyp)+')';
    end;

{****************************************************************************
                               TELFExeOutputx86_64
****************************************************************************}

  procedure TElfExeOutputx86_64.GOTRelocPass1(objsec:TObjSection;var idx:longint);
    var
      objsym:TObjSymbol;
      objreloc:TObjRelocation;
      reltyp:byte;
      externsym,fromtext:boolean;
    begin
      objreloc:=TObjRelocation(objsec.ObjRelocations[idx]);
      if (ObjReloc.flags and rf_raw)=0 then
        reltyp:=ElfTarget.encodereloc(ObjReloc)
      else
        reltyp:=ObjReloc.ftype;

      case reltyp of
        R_X86_64_PLT32,
        R_X86_64_PLTOFF64,
        R_X86_64_GOTPLT64:
          begin
            if assigned(ObjReloc.symbol) and assigned(ObjReloc.symbol.exesymbol) then
              begin
                objsym:=ObjReloc.symbol.exesymbol.ObjSymbol;
                objsym.refs:=objsym.refs or symref_plt;
              end;
          end;
      end;

      case reltyp of
        R_X86_64_GOTTPOFF:
          begin
            { TLS IE to locally defined symbol, convert into LE so GOT entry isn't needed
              (Is TLS IE allowed in shared libs at all? Yes it is, when lib is accessing
               a threadvar in main program or in other *statically* loaded lib; TLS IE access to
               own threadvars may render library not loadable dynamically) }
(*
            if not (IsSharedLibrary or (sym.dynindex>0)) then
              begin
                if not IsValidIEtoLE(objsec,ObjReloc) then
                  Comment(v_error,'Cannot transform TLS IE to LE');
                TranslateIEtoLE(objsec,ObjReloc);
                ObjReloc.ftype:=R_X86_64_TPOFF32;
                exit;
              end;
*)
            AllocGOTSlot(objreloc.symbol);
          end;

        R_X86_64_GOT32,
        R_X86_64_GOT64,
        R_X86_64_GOTPCREL,
        R_X86_64_GOTPCREL64:
          begin
            if AllocGOTSlot(objreloc.symbol) then
              if IsSharedLibrary and (objreloc.symbol.exesymbol.dynindex=0) then
                Inc(relative_reloc_count);
          end;

        //R_X86_64_TLSGD,
        //R_X86_64_TLSLD:  { TODO: allocate two GOT slots }

        R_X86_64_TPOFF32,
        { R_X86_64_32S cannot be used in DSOs at all }
        R_X86_64_32S:
          if IsSharedLibrary then
            ReportNonDSOReloc(reltyp,objsec,objreloc);

        { R_X86_64_32 is processed by rtld, but binutils accept it in data sections only.
          Relocating against local symbols is tricky: changing into RELATIVE is not possible,
          so it is changed into relocation against section symbol. This requires adding
          the appropriate section symbol to dynamic symtable. BFD also has some obscure logic
          behind, e.g. it uses .text section for symbols from .data section.

          For now, leave this situation unhandled, as 32-bit relocations aren't commonly
          used in 64-bit code. }

        R_X86_64_32:
          if IsSharedLibrary then
            begin
              if (oso_executable in objsec.SecOptions) or
                not (oso_write in objsec.SecOptions) then
                ReportNonDSOReloc(reltyp,objsec,objreloc)
              else
                InternalError(2012092601);
            end;

        R_X86_64_64:
          begin
            fromtext:=(oso_executable in objsec.SecOptions) or
              not (oso_write in objsec.SecOptions);
            externsym:=assigned(objreloc.symbol) and
              assigned(objreloc.symbol.exesymbol) and
              (objreloc.symbol.exesymbol.dynindex<>0);

            if IsSharedLibrary then
              begin
                if fromtext then
                  hastextrelocs:=True;
                dynrelocsec.alloc(dynrelocsec.shentsize);
                objreloc.flags:=objreloc.flags or rf_dynamic;
                if (not externsym) then
                  Inc(relative_reloc_count);
              end
            else if externsym then
              // TODO: R_X86_64_32 and R_X86_64_32S here?
              begin
                objsym:=objreloc.symbol.ExeSymbol.ObjSymbol;
                { If symbol has non-GOT references from readonly sections, then it needs a
                  copy reloc, which eliminates any dynamic relocations to this symbol from
                  writable sections as well. OTOH if it is referenced *only* from writable
                  sections, then it's better not to generate a copy reloc and keep dynamic
                  relocations. The following code is based on assumption that all readonly
                  sections are processed before writable ones (which is true for current
                  segment mapping).
                  For arbitrary segment mapping, this will probably require a separate pass. }
                if fromtext then
                  objsym.refs:=objsym.refs or symref_from_text
                else if (objsym.refs and symref_from_text)=0 then
                  begin
                    dynrelocsec.alloc(dynrelocsec.shentsize);
                    objreloc.flags:=objreloc.flags or rf_dynamic;
                  end;
              end;
          end;
      end;
    end;


  procedure TElfExeOutputx86_64.MaybeWriteGOTEntry(relocval:aint;objsym:TObjSymbol);
    var
      gotoff,tmp:aword;
    begin
      gotoff:=objsym.exesymbol.gotoffset;
      if gotoff=0 then
        InternalError(2012060902);

      { the GOT slot itself, and a dynamic relocation for it }
      { TODO: only data symbols must get here }
      if gotoff=gotobjsec.Data.size+sizeof(pint) then
        begin
          gotobjsec.write(relocval,sizeof(pint));

          tmp:=gotobjsec.mempos+gotoff-sizeof(pint);
          if (objsym.exesymbol.dynindex>0) then
            dynreloclist.Add(TObjRelocation.CreateRaw(tmp,objsym,R_X86_64_GLOB_DAT))
          else
            if IsSharedLibrary then
              WriteDynRelocEntry(tmp,R_X86_64_RELATIVE,0,relocval);
        end;
    end;


  procedure TElfExeOutputx86_64.MaybeWriteTLSIEGotEntry(relocval:aint;objsym:TObjSymbol);
    var
      gotoff,tmp: aword;
      objrel: TObjRelocation;
    begin
      gotoff:=objsym.exesymbol.gotoffset;
      if gotoff=0 then
        InternalError(2012060903);

      if gotoff=gotobjsec.Data.size+sizeof(pint) then
        begin
          tmp:=gotobjsec.mempos+gotoff-sizeof(pint);
          if (objsym.exesymbol.dynindex>0) then
            begin
              gotobjsec.writezeros(sizeof(pint));
              dynreloclist.Add(TObjRelocation.CreateRaw(tmp,objsym,R_X86_64_TPOFF64));
            end
          else
            begin
              gotobjsec.write(relocval,sizeof(pint));
              if IsSharedLibrary then
                begin
                  objrel:=TObjRelocation.CreateRaw(tmp,nil,R_X86_64_TPOFF64);
                  objrel.orgsize:=relocval;
                  dynreloclist.Add(objrel);
                end;
            end;
        end;
    end;

  procedure TElfExeOutputx86_64.DoRelocationFixup(objsec:TObjSection);
    var
      i,zero:longint;
      objreloc: TObjRelocation;
      address,
      relocval : aint;
      relocsec : TObjSection;
      data: TDynamicArray;
      reltyp,relsize: byte;
      PC: aword;
    begin
      data:=objsec.data;
      for i:=0 to objsec.ObjRelocations.Count-1 do
        begin
          objreloc:=TObjRelocation(objsec.ObjRelocations[i]);
          case objreloc.typ of
            RELOC_NONE:
              continue;
            RELOC_ZERO:
              begin
                data.Seek(objreloc.dataoffset);
                zero:=0;
                data.Write(zero,4);
                continue;
              end;
            else
              ;
          end;

          if (objreloc.flags and rf_raw)=0 then
            reltyp:=ElfTarget.encodereloc(objreloc)
          else
            reltyp:=objreloc.ftype;

          if reltyp<=high(relocprops) then
            relsize:=relocprops[reltyp].size
          else
            InternalError(2012092103);

          if ElfTarget.relocs_use_addend then
            address:=objreloc.orgsize
          else
            begin
              data.Seek(objreloc.dataoffset);
              data.Read(address,relsize);
            end;
          if assigned(objreloc.symbol) then
            begin
              relocsec:=objreloc.symbol.objsection;
              relocval:=objreloc.symbol.address;
            end
          else if assigned(objreloc.objsection) then
            begin
              relocsec:=objreloc.objsection;
              relocval:=objreloc.objsection.mempos
            end
          else
            internalerror(2012060702);

          { Only debug sections are allowed to have relocs pointing to unused sections }
          if assigned(relocsec) and not (relocsec.used and assigned(relocsec.exesection)) and
             not (oso_debug in objsec.secoptions) then
            begin
              writeln(objsec.fullname,' references ',relocsec.fullname);
              internalerror(2012060703);
            end;

          PC:=objsec.mempos+objreloc.dataoffset;
          { TODO: if relocsec=nil, relocations must be copied to .rela.dyn section }
          if (relocsec=nil) or (relocsec.used) then
            case reltyp of
              R_X86_64_PC32,
              R_X86_64_PC64:
                begin
                  // TODO: ld rejects PC32 relocations to dynamic symbols, they must use @PLT
                  address:=address+relocval-PC;
                end;

              R_X86_64_PLT32:
                begin
                  { If target is in current object, treat as RELOC_RELATIVE }
                  address:=address+relocval-PC;
                end;

              //R_X86_64_DTPOFF64 is possible in data??
              R_X86_64_DTPOFF32:
                begin
                  { In executable it behaves as TPOFF32 (i.e. generates negative offset),
                    but data expressions like ".long foo@dtpoff" resolve to positive offset }
                  if IsSharedLibrary or not (oso_executable in objsec.SecOptions) then
                    address:=address+relocval-tlsseg.MemPos
                  else
                    address:=address+relocval-(tlsseg.MemPos+tlsseg.MemSize);
                end;

              R_X86_64_TPOFF32,
              R_X86_64_TPOFF64:
                address:=address+relocval-(tlsseg.MemPos+tlsseg.MemSize);

              R_X86_64_GOTTPOFF:
                begin
                  if IsSharedLibrary then
                    relocval:=relocval-tlsseg.MemPos
                  else
                    relocval:=relocval-(tlsseg.MemPos+tlsseg.MemSize);

                  MaybeWriteTLSIEGotEntry(relocval,objreloc.symbol);

                  { resolves to PC-relative offset to GOT slot }
                  relocval:=gotobjsec.mempos+objreloc.symbol.exesymbol.gotoffset-sizeof(pint);
                  address:=address+relocval-PC;
                end;

              R_X86_64_GOTPCREL,
              R_X86_64_GOTPCREL64:
                begin
                  MaybeWriteGOTEntry(relocval,objreloc.symbol);

                  { resolves to PC-relative offset to GOT slot }
                  relocval:=gotobjsec.mempos+objreloc.symbol.exesymbol.gotoffset-sizeof(pint);
                  address:=address+relocval-PC;
                end;

              R_X86_64_32S,
              R_X86_64_32:
                inc(address,relocval);

              R_X86_64_64:
                begin
                  inc(address,relocval);

                  if (objreloc.flags and rf_dynamic)<>0 then
                    begin
                      if (objreloc.symbol=nil) or
                         (objreloc.symbol.exesymbol=nil) or
                         (objreloc.symbol.exesymbol.dynindex=0) then
                        WriteDynRelocEntry(PC,R_X86_64_RELATIVE,0,address)
                      else
                        dynreloclist.add(TObjRelocation.CreateRaw(PC,objreloc.symbol,R_X86_64_64));
                    end;
                end;

              R_X86_64_GOTPC32,
              R_X86_64_GOTPC64:
                begin
                  address:=address+gotsymbol.address-PC;
                end;

              R_X86_64_GOT32,
              R_X86_64_GOT64:
                begin
                  MaybeWriteGOTEntry(relocval,objreloc.symbol);

                  relocval:=gotobjsec.mempos+objreloc.symbol.exesymbol.gotoffset-sizeof(pint)-gotsymbol.address;
                  address:=address+relocval;
                end;

              R_X86_64_GOTOFF64,
              R_X86_64_PLTOFF64:
                begin
                  address:=address+relocval-gotsymbol.address;
                end;

              else
                begin
                  writeln(objreloc.typ);
                  internalerror(2006040105);
                end;
            end
          else           { not relocsec.Used }
            address:=0;  { Relocation in debug section points to unused section, which is eliminated by linker }

          case relsize of
            8: ;
            4:
              begin
                case reltyp of
                  R_X86_64_32:
                    if qword(address)>qword($FFFFFFFF) then
                      ReportRelocOverflow(reltyp,objsec,objreloc);
                else
                  if (address>high(longint)) or (address<low(longint)) then
                    ReportRelocOverflow(reltyp,objsec,objreloc);
                end;
              end;
          else
            InternalError(2012101103);
          end;

          data.Seek(objreloc.dataoffset);
          data.Write(address,relsize);
        end;
    end;


  procedure TElfExeOutputx86_64.WriteFirstPLTEntry;
    begin
      pltobjsec.writeBytes(#$FF#$35);         // push got+8(%rip)
      pltobjsec.writeReloc_internal(gotpltobjsec,sizeof(pint),4,RELOC_RELATIVE);
      pltobjsec.writeBytes(#$FF#$25);         // jmp  *got+16(%rip)
      pltobjsec.writeReloc_internal(gotpltobjsec,2*sizeof(pint),4,RELOC_RELATIVE);
      pltobjsec.writeBytes(#$0F#$1F#$40#$00); // nopl  0(%rax)
    end;


  procedure TElfExeOutputx86_64.WritePLTEntry(exesym:TExeSymbol);
    var
      got_offset: aword;
      tmp: pint;
    begin
      pltobjsec.writeBytes(#$FF#$25);       // jmp  *got+x(%rip)
      pltobjsec.writeReloc_internal(gotpltobjsec,gotpltobjsec.size,4,RELOC_RELATIVE);
      pltobjsec.writeBytes(#$68);           // push  $index
      tmp:=pltrelocsec.size div pltrelocsec.shentsize;
      pltobjsec.write(tmp,4);

      pltobjsec.writeBytes(#$E9);           // jmp   .plt
      tmp:=-(4+pltobjsec.Size);
      pltobjsec.write(tmp,4);

      { write a .got.plt slot pointing back to the 'push' instruction }
      gotpltobjsec.writeReloc_internal(pltobjsec,pltobjsec.size-(16-6),sizeof(pint),RELOC_ABSOLUTE);

      { write a .rela.plt entry (Elf64_rela record) }
      pltrelocsec.writeReloc_internal(gotpltobjsec,gotpltobjsec.size-sizeof(pint),sizeof(pint),RELOC_ABSOLUTE);
      got_offset:=(qword(exesym.dynindex) shl 32) or R_X86_64_JUMP_SLOT;
      pltrelocsec.write(got_offset,sizeof(pint));
      if ElfTarget.relocs_use_addend then
        pltrelocsec.writezeros(sizeof(pint));
    end;


  procedure TElfExeOutputx86_64.WriteIndirectPLTEntry(exesym:TExeSymbol);
    var
      tmp: pint;
      objsym:TObjSymbol;
      targetsym:TObjSymbol;
    begin
      targetsym:=exesym.ObjSymbol;
      objsym:=internalobjdata.CreateSymbol(exesym.name);
      objsym.typ:=AT_FUNCTION;
      objsym.bind:=exesym.ObjSymbol.bind;  { AB_EXTERNAL or AB_WEAK_EXTERNAL }
      objsym.offset:=pltobjsec.size;
      objsym.objsection:=pltobjsec;
      exesym.ObjSymbol:=objsym;

      pltobjsec.writeBytes(#$FF#$25);       // jmp  *got+x(%rip)
      pltobjsec.writeReloc_internal(gotpltobjsec,gotpltobjsec.size,4,RELOC_RELATIVE);
      { TODO: Are these entries relevant when linking dynamic?
        (for static linking, they don't matter) }
      pltobjsec.writeBytes(#$68);           // push  $index
      tmp:=pltrelocsec.size div pltrelocsec.shentsize;
      pltobjsec.write(tmp,4);

      pltobjsec.writeBytes(#$E9);           // jmp   .plt
      tmp:=-(4+pltobjsec.Size);
      pltobjsec.write(tmp,4);

      { write a .got.plt slot pointing back to the 'push' instruction }
      gotpltobjsec.writeReloc_internal(pltobjsec,pltobjsec.size-(16-6),sizeof(pint),RELOC_ABSOLUTE);

      { write a .rela.iplt entry (Elf64_rela record) }
      ipltrelocsec.writeReloc_internal(gotpltobjsec,gotpltobjsec.size-sizeof(pint),sizeof(pint),RELOC_ABSOLUTE);
      tmp:=R_X86_64_IRELATIVE;
      ipltrelocsec.write(tmp,sizeof(pint));
      if ElfTarget.relocs_use_addend then
        ipltrelocsec.writeReloc_internal(targetsym.objsection,targetsym.offset,sizeof(pint),RELOC_ABSOLUTE);
    end;

{*****************************************************************************
                                    Initialize
*****************************************************************************}

  const
    elf_target_x86_64: TElfTarget =
      (
        max_page_size:     $200000;
        exe_image_base:    $400000;
        machine_code:      EM_X86_64;
        relocs_use_addend: true;
        dyn_reloc_codes: (
          R_X86_64_RELATIVE,
          R_X86_64_GLOB_DAT,
          R_X86_64_JUMP_SLOT,
          R_X86_64_COPY,
          R_X86_64_IRELATIVE
        );
        relocname:         @elf_x86_64_relocName;
        encodereloc:       @elf_x86_64_encodeReloc;
        loadreloc:         @elf_x86_64_loadReloc;
        loadsection:       nil;
        encodeflags:       nil;
      );


    as_x86_64_elf64_info : tasminfo =
      (
        id     : as_x86_64_elf64;
        idtxt  : 'ELF';
        asmbin : '';
        asmcmd : '';
        supported_targets : [system_x86_64_linux,system_x86_64_freebsd,
                             system_x86_64_openbsd,system_x86_64_netbsd,
                             system_x86_64_dragonfly,system_x86_64_solaris,
                             system_x86_64_aros,system_x86_64_android,
                             system_x86_64_haiku];
        flags : [af_outputbinary,af_smartlink_sections,af_supports_dwarf];
        labelprefix : '.L';
        labelmaxlen : -1;
        comment : '';
        dollarsign: '$';
      );

initialization
  RegisterAssembler(as_x86_64_elf64_info,TElfAssembler);
  ElfTarget:=elf_target_x86_64;
  ElfExeOutputClass:=TElfExeOutputx86_64;

end.

