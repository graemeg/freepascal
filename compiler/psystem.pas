{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Load the system unit, create required defs for systemunit

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
unit psystem;

{$i fpcdefs.inc}

interface

    uses
      symbase;

    procedure create_intern_symbols;
    procedure create_intern_types;

    procedure load_intern_types;

    procedure registernodes;
    procedure registertais;


implementation

    uses
      globals,globtype,verbose,constexp,cpuinfo,compinnr,
      systems,
      symconst,symtype,symsym,symdef,symcpu,symtable,
      aasmtai,aasmcpu,
      fmodule,
      node,nbas,nflw,nset,ncon,ncnv,nld,nmem,ncal,nmat,nadd,ninl;


    procedure create_intern_symbols;
      {
        all intern procedures for the system unit
      }
      begin
        systemunit.insert(csyssym.create('Concat',in_concat_x));
        systemunit.insert(csyssym.create('Write',in_write_x));
        systemunit.insert(csyssym.create('WriteLn',in_writeln_x));
        systemunit.insert(csyssym.create('WriteStr',in_writestr_x));
        systemunit.insert(csyssym.create('Assigned',in_assigned_x));
        systemunit.insert(csyssym.create('Read',in_read_x));
        systemunit.insert(csyssym.create('ReadLn',in_readln_x));
        systemunit.insert(csyssym.create('ReadStr',in_readstr_x));
        systemunit.insert(csyssym.create('Ofs',in_ofs_x));
        systemunit.insert(csyssym.create('SizeOf',in_sizeof_x));
        systemunit.insert(csyssym.create('BitSizeOf',in_bitsizeof_x));
        systemunit.insert(csyssym.create('TypeOf',in_typeof_x));
        systemunit.insert(csyssym.create('Low',in_low_x));
        systemunit.insert(csyssym.create('High',in_high_x));
        systemunit.insert(csyssym.create('Slice',in_slice_x));
        systemunit.insert(csyssym.create('Seg',in_seg_x));
        systemunit.insert(csyssym.create('Ord',in_ord_x));
        systemunit.insert(csyssym.create('Pred',in_pred_x));
        systemunit.insert(csyssym.create('Succ',in_succ_x));
        systemunit.insert(csyssym.create('Exclude',in_exclude_x_y));
        systemunit.insert(csyssym.create('Include',in_include_x_y));
        systemunit.insert(csyssym.create('Pack',in_pack_x_y_z));
        systemunit.insert(csyssym.create('Unpack',in_unpack_x_y_z));
        systemunit.insert(csyssym.create('Break',in_break));
        systemunit.insert(csyssym.create('Exit',in_exit));
        systemunit.insert(csyssym.create('Continue',in_continue));
        systemunit.insert(csyssym.create('Leave',in_leave)); {macpas only}
        systemunit.insert(csyssym.create('Cycle',in_cycle)); {macpas only}
        systemunit.insert(csyssym.create('Dec',in_dec_x));
        systemunit.insert(csyssym.create('Inc',in_inc_x));
        systemunit.insert(csyssym.create('Str',in_str_x_string));
        systemunit.insert(csyssym.create('Assert',in_assert_x_y));
        systemunit.insert(csyssym.create('Val',in_val_x));
        systemunit.insert(csyssym.create('Addr',in_addr_x));
{$ifdef i8086}
        systemunit.insert(csyssym.create('FarAddr',in_faraddr_x));
{$endif i8086}
        systemunit.insert(csyssym.create('TypeInfo',in_typeinfo_x));
        systemunit.insert(csyssym.create('SetLength',in_setlength_x));
        systemunit.insert(csyssym.create('Copy',in_copy_x));
        systemunit.insert(csyssym.create('Initialize',in_initialize_x));
        systemunit.insert(csyssym.create('Finalize',in_finalize_x));
        systemunit.insert(csyssym.create('Length',in_length_x));
        systemunit.insert(csyssym.create('New',in_new_x));
        systemunit.insert(csyssym.create('Dispose',in_dispose_x));
{$ifdef SUPPORT_GET_FRAME}
        systemunit.insert(csyssym.create('Get_Frame',in_get_frame));
{$endif SUPPORT_GET_FRAME}
        systemunit.insert(csyssym.create('Unaligned',in_unaligned_x));
        systemunit.insert(csyssym.create('Aligned',in_aligned_x));
        systemunit.insert(csyssym.create('Volatile',in_volatile_x));
        systemunit.insert(csyssym.create('ObjCSelector',in_objc_selector_x)); { objc only }
        systemunit.insert(csyssym.create('ObjCEncode',in_objc_encode_x)); { objc only }
        systemunit.insert(csyssym.create('Default',in_default_x));
        systemunit.insert(csyssym.create('SetString',in_setstring_x_y_z));
        systemunit.insert(csyssym.create('Insert',in_insert_x_y_z));
        systemunit.insert(csyssym.create('Delete',in_delete_x_y_z));
        systemunit.insert(csyssym.create('GetTypeKind',in_gettypekind_x));
        systemunit.insert(csyssym.create('IsManagedType',in_ismanagedtype_x));
        systemunit.insert(csyssym.create('fpc_eh_return_data_regno', in_const_eh_return_data_regno));
        systemunit.insert(cconstsym.create_ord('False',constord,0,pasbool1type));
        systemunit.insert(cconstsym.create_ord('True',constord,1,pasbool1type));
      end;


    procedure set_default_int_types;
      begin
{$ifdef cpu64bitalu}
        aluuinttype:=u64inttype;
        alusinttype:=s64inttype;
{$endif cpu64bitalu}
{$ifdef cpu64bitaddr}
        sizeuinttype:=u64inttype;
        sizesinttype:=s64inttype;
        uinttype:=u64inttype;
        sinttype:=s64inttype;
{$endif cpu64bitaddr}
{$ifdef cpu32bitaddr}
        sizeuinttype:=u32inttype;
        sizesinttype:=s32inttype;
        uinttype:=u32inttype;
        sinttype:=s32inttype;
{$endif cpu32bitaddr}
{$ifdef cpu32bitalu}
        uinttype:=u32inttype;
        sinttype:=s32inttype;
        aluuinttype:=u32inttype;
        alusinttype:=s32inttype;
{$endif cpu32bitalu}
{$ifdef cpu16bitaddr}
        sizeuinttype:=u16inttype;
        sizesinttype:=s16inttype;
{$endif cpu16bitaddr}
{$ifdef cpu16bitalu}
        uinttype:=u16inttype;
        sinttype:=s16inttype;
        aluuinttype:=u16inttype;
        alusinttype:=s16inttype;
{$endif cpu16bitalu}
{$ifdef cpu8bitalu}
        uinttype:=u8inttype;
        sinttype:=s8inttype;
        aluuinttype:=u8inttype;
        alusinttype:=s8inttype;
{$endif cpu8bitalu}

        osuinttype:=uinttype;
        ossinttype:=sinttype;
      end;


    procedure set_default_ptr_types;
      begin
{$ifdef i8086}
        if current_settings.x86memorymodel in x86_far_code_models then
          voidcodepointertype:=voidfarpointertype
        else if current_settings.x86memorymodel=mm_tiny then
          voidcodepointertype:=voidnearpointertype
        else
          voidcodepointertype:=voidnearcspointertype;
        voidstackpointertype:=voidnearsspointertype;
{$else i8086}
        voidcodepointertype:=voidpointertype;
        voidstackpointertype:=voidpointertype;
{$endif i8086}
        case voidcodepointertype.size of
          2:
            begin
              codeptruinttype:=u16inttype;
              codeptrsinttype:=s16inttype;
            end;
          4:
            begin
              codeptruinttype:=u32inttype;
              codeptrsinttype:=s32inttype;
            end;
          8:
            begin
              codeptruinttype:=u64inttype;
              codeptrsinttype:=s64inttype;
            end;
          else
            Internalerror(2015112106);
        end;
        case voidpointertype.size of
          2:
            begin
              ptruinttype:=u16inttype;
              ptrsinttype:=s16inttype;
            end;
          4:
            begin
              ptruinttype:=u32inttype;
              ptrsinttype:=s32inttype;
            end;
          8:
            begin
              ptruinttype:=u64inttype;
              ptrsinttype:=s64inttype;
            end;
          else
            Internalerror(2016100301);
        end;
      end;

    procedure create_intern_types;
      {
        all the types inserted into the system unit
      }

        function addtype(const s:string;def:tdef):ttypesym;
        begin
          result:=ctypesym.create(s,def);
          systemunit.insert(result);
        end;

        procedure addfield(recst:tabstractrecordsymtable;sym:tfieldvarsym);
        begin
          recst.insert(sym);
          recst.addfield(sym,vis_hidden);
        end;

        procedure create_fpu_types;
        begin
          if init_settings.fputype<>fpu_none then
            begin
              s32floattype:=cfloatdef.create(s32real,true);
              s64floattype:=cfloatdef.create(s64real,true);
              s80floattype:=cfloatdef.create(s80real,true);
              sc80floattype:=cfloatdef.create(sc80real,true);
            end else begin
              s32floattype:=nil;
              s64floattype:=nil;
              s80floattype:=nil;
              sc80floattype:=nil;
            end;
        end;

      var
        hrecst : trecordsymtable;
      begin
        symtablestack.push(systemunit);
        cundefinedtype:=cundefineddef.create(true);
        cformaltype:=cformaldef.create(false);
        ctypedformaltype:=cformaldef.create(true);
        voidtype:=corddef.create(uvoid,0,0,true);
        voidpointertype:=cpointerdef.create(voidtype);
        u8inttype:=corddef.create(u8bit,0,255,true);
        s8inttype:=corddef.create(s8bit,int64(-128),127,true);
        u16inttype:=corddef.create(u16bit,0,65535,true);
        s16inttype:=corddef.create(s16bit,int64(-32768),32767,true);
        s24inttype:=corddef.create(customint,-(int64(1) shl 23),1 shl 23 - 1,true);
        u24inttype:=corddef.create(customint,0,1 shl 24 - 1,true);
        u32inttype:=corddef.create(u32bit,0,high(longword),true);
        s32inttype:=corddef.create(s32bit,int64(low(longint)),int64(high(longint)),true);
        s40inttype:=corddef.create(customint,-(int64(1) shl 39),int64(1) shl 39 - 1,true);
        u40inttype:=corddef.create(customint,0,int64(1) shl 40 - 1,true);
        s48inttype:=corddef.create(customint,-(int64(1) shl 47),int64(1) shl 47 - 1,true);
        u48inttype:=corddef.create(customint,0,int64(1) shl 48 - 1,true);
        s56inttype:=corddef.create(customint,-(int64(1) shl 55),int64(1) shl 55 - 1,true);
        u56inttype:=corddef.create(customint,0,int64(1) shl 56 - 1,true);
        u64inttype:=corddef.create(u64bit,low(qword),high(qword),true);
        s64inttype:=corddef.create(s64bit,low(int64),high(int64),true);
        { upper/lower bound not yet properly set for 128 bit types, as we don't
          support them yet at the Pascal level (nor for tconstexprint); they're
          only used internally by the high level code generator for LLVM to
          implement overflow checking }
        u128inttype:=corddef.create(u128bit,0,0,true);
        s128inttype:=corddef.create(s128bit,0,0,true);
        pasbool1type:=corddef.create(pasbool1,0,1,true);
        pasbool8type:=corddef.create(pasbool8,0,1,true);
        pasbool16type:=corddef.create(pasbool16,0,1,true);
        pasbool32type:=corddef.create(pasbool32,0,1,true);
        pasbool64type:=corddef.create(pasbool64,0,1,true);
        bool8type:=corddef.create(bool8bit,low(int64),high(int64),true);
        bool16type:=corddef.create(bool16bit,low(int64),high(int64),true);
        bool32type:=corddef.create(bool32bit,low(int64),high(int64),true);
        bool64type:=corddef.create(bool64bit,low(int64),high(int64),true);
{$ifdef llvm}
        llvmbool1type:=corddef.create(pasbool1,0,1,true);
{$endif llvm}
        cansichartype:=corddef.create(uchar,0,255,true);
        cwidechartype:=corddef.create(uwidechar,0,65535,true);
        cshortstringtype:=cstringdef.createshort(255,true);
        { should we give a length to the default long and ansi string definition ?? }
        clongstringtype:=cstringdef.createlong(-1,true);
        cansistringtype:=cstringdef.createansi(0,true);
        if target_info.system in systems_windows then
          cwidestringtype:=cstringdef.createwide(true)
        else
          cwidestringtype:=cstringdef.createunicode(true);
        cunicodestringtype:=cstringdef.createunicode(true);
        { length=0 for shortstring is open string (needed for readln(string) }
        openshortstringtype:=cstringdef.createshort(0,true);
{$ifdef x86}
        create_fpu_types;
{$ifndef FPC_SUPPORT_X87_TYPES_ON_WIN64}
        if target_info.system=system_x86_64_win64 then
          begin
            s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
            pbestrealtype:=@s64floattype;
          end
        else
{$endif FPC_SUPPORT_X87_TYPES_ON_WIN64}
          s64currencytype:=cfloatdef.create(s64currency,true);
{$endif x86}
{$ifdef powerpc}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif powerpc}
{$ifdef POWERPC64}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif POWERPC64}
{$ifdef sparc}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif sparc}
{$ifdef sparc64}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif sparc64}
{$ifdef m68k}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif}
{$ifdef arm}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif arm}
{$ifdef aarch64}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif aarch64}
{$ifdef avr}
        s32floattype:=cfloatdef.create(s32real,true);
        s64floattype:=cfloatdef.create(s64real,true);
        s80floattype:=cfloatdef.create(s80real,true);
        sc80floattype:=cfloatdef.create(sc80real,true);
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif avr}
{$ifdef mips}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif mips}
{$ifdef riscv32}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif riscv32}
{$ifdef riscv64}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif riscv64}
{$ifdef jvm}
        create_fpu_types;
        s64currencytype:=corddef.create(scurrency,low(int64),high(int64),true);
{$endif jvm}
        set_default_int_types;
        { some other definitions }
        charpointertype:=cpointerdef.create(cansichartype);
        widecharpointertype:=cpointerdef.create(cwidechartype);
{$ifdef i8086}
        parentfpvoidpointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near_ss);
{$else i8086}
        parentfpvoidpointertype:=cpointerdef.create(voidtype);
{$endif i8086}
{$ifdef x86}
        voidnearpointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near);
        voidnearcspointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near_cs);
        voidneardspointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near_ds);
        voidnearsspointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near_ss);
        voidnearespointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near_es);
        voidnearfspointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near_fs);
        voidneargspointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_near_gs);
  {$ifdef i8086}
        voidfarpointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_far);
        voidhugepointertype:=tcpupointerdefclass(cpointerdef).createx86(voidtype,x86pt_huge);
        charnearpointertype:=tcpupointerdefclass(cpointerdef).createx86(cansichartype,x86pt_near);
        charfarpointertype:=tcpupointerdefclass(cpointerdef).createx86(cansichartype,x86pt_far);
        charhugepointertype:=tcpupointerdefclass(cpointerdef).createx86(cansichartype,x86pt_huge);
        bytefarpointertype:=tcpupointerdefclass(cpointerdef).createx86(u8inttype,x86pt_far);
        wordfarpointertype:=tcpupointerdefclass(cpointerdef).createx86(u16inttype,x86pt_far);
        longintfarpointertype:=tcpupointerdefclass(cpointerdef).createx86(s32inttype,x86pt_far);
  {$endif i8086}
{$endif x86}
        set_default_ptr_types;
        openchararraytype:=carraydef.create_openarray;
        tarraydef(openchararraytype).elementdef:=cansichartype;
        cfiletype:=cfiledef.createuntyped;
        if f_variants in features then
          begin
            cvarianttype:=cvariantdef.create(vt_normalvariant);
            colevarianttype:=cvariantdef.create(vt_olevariant);
          end;

{$ifdef cpufpemu}
        { Normal types }
        (* we use the same types as without emulator, the only
          difference is that direct calls to the emulator are generated
        if (cs_fp_emulation in current_settings.moduleswitches) then
          begin
            addtype('Single',s32floattype);
            { extended size is the best real type for the target }
            addtype('Real',s32floattype);
            pbestrealtype:=@s32floattype;
            { extended size is the best real type for the target }
            addtype('Extended',pbestrealtype^);
          end
        else
        *)
{$endif cpufpemu}
        if init_settings.fputype<>fpu_none then
          begin
            addtype('Single',s32floattype);
            addtype('Double',s64floattype);
            { extended size is the best real type for the target }
            addtype('Extended',pbestrealtype^);
            { CExtended corresponds to the C version of the Extended type
              (either "long double" or "double") }
            if target_info.system in systems_android then
              { Android has "long double"="double" even for x86 }
              addtype('CExtended',s64floattype)
            else
              if tfloatdef(pbestrealtype^).floattype=s80real then
                addtype('CExtended',sc80floattype)
              else
                addtype('CExtended',pbestrealtype^);
          end;
{$ifdef x86}
{$ifndef FPC_SUPPORT_X87_TYPES_ON_WIN64}
        if target_info.system<>system_x86_64_win64 then
{$endif FPC_SUPPORT_X87_TYPES_ON_WIN64}
          addtype('Comp',cfloatdef.create(s64comp,true));
{$endif x86}
        addtype('Currency',s64currencytype);
        addtype('Pointer',voidpointertype);
{$ifdef x86}
        addtype('NearPointer',voidnearpointertype);
        addtype('NearCsPointer',voidnearcspointertype);
        addtype('NearDsPointer',voidneardspointertype);
        addtype('NearSsPointer',voidnearsspointertype);
        addtype('NearEsPointer',voidnearespointertype);
        addtype('NearFsPointer',voidnearfspointertype);
        addtype('NearGsPointer',voidneargspointertype);
  {$ifdef i8086}
        addtype('FarPointer',voidfarpointertype);
        addtype('HugePointer',voidhugepointertype);
  {$endif i8086}
{$endif x86}
        addtype('ShortString',cshortstringtype);
{$ifdef support_longstring}
        addtype('LongString',clongstringtype);
{$endif support_longstring}
        addtype('AnsiString',cansistringtype);
        addtype('WideString',cwidestringtype);
        addtype('UnicodeString',cunicodestringtype);

        addtype('OpenString',openshortstringtype);
        addtype('Boolean',pasbool1type);
        addtype('Boolean8',pasbool8type);
        addtype('Boolean16',pasbool16type);
        addtype('Boolean32',pasbool32type);
        addtype('Boolean64',pasbool64type);
        addtype('ByteBool',bool8type);
        addtype('WordBool',bool16type);
        addtype('LongBool',bool32type);
        addtype('QWordBool',bool64type);
{$ifdef llvm}
        addtype('LLVMBool1',llvmbool1type);
{$endif llvm}
        addtype('Byte',u8inttype);
        addtype('ShortInt',s8inttype);
        addtype('Word',u16inttype);
        addtype('SmallInt',s16inttype);
        addtype('LongWord',u32inttype);
        addtype('LongInt',s32inttype);
        addtype('QWord',u64inttype);
        addtype('Int64',s64inttype);
        addtype('Char',cansichartype);
        addtype('WideChar',cwidechartype);
        addtype('Text',cfiledef.createtext);
        addtype('TypedFile',cfiledef.createtyped(voidtype));
        if f_variants in features then
          begin
            addtype('Variant',cvarianttype);
            addtype('OleVariant',colevarianttype);
          end;
        { Internal types }
        addtype('$undefined',cundefinedtype);
        addtype('$formal',cformaltype);
        addtype('$typedformal',ctypedformaltype);
        addtype('$void',voidtype);
        addtype('$void_pointer',voidpointertype);
        addtype('$byte',u8inttype);
        addtype('$shortint',s8inttype);
        addtype('$word',u16inttype);
        addtype('$smallint',s16inttype);
        addtype('$sint24',s24inttype);
        addtype('$uint24',u24inttype);
        addtype('$ulong',u32inttype);
        addtype('$longint',s32inttype);
        addtype('$sint40',s40inttype);
        addtype('$uint40',u40inttype);
        addtype('$sint48',s48inttype);
        addtype('$uint48',u48inttype);
        addtype('$sint56',s56inttype);
        addtype('$uint56',u56inttype);
        addtype('$qword',u64inttype);
        addtype('$int64',s64inttype);
        addtype('$uint128',u128inttype);
        addtype('$int128',s128inttype);
        addtype('$char',cansichartype);
        addtype('$widechar',cwidechartype);
        addtype('$shortstring',cshortstringtype);
        addtype('$longstring',clongstringtype);
        addtype('$ansistring',cansistringtype);
        addtype('$widestring',cwidestringtype);
        addtype('$unicodestring',cunicodestringtype);
        addtype('$openshortstring',openshortstringtype);
        addtype('$boolean',pasbool1type);
        addtype('$boolean8',pasbool8type);
        addtype('$boolean16',pasbool16type);
        addtype('$boolean32',pasbool32type);
        addtype('$boolean64',pasbool64type);
        addtype('$bytebool',bool8type);
        addtype('$wordbool',bool16type);
        addtype('$longbool',bool32type);
        addtype('$qwordbool',bool64type);
{$ifdef llvm}
        addtype('$llvmbool1',llvmbool1type);
{$endif llvm}
        addtype('$char_pointer',charpointertype);
        addtype('$widechar_pointer',widecharpointertype);
        addtype('$parentfp_void_pointer',parentfpvoidpointertype);
{$ifdef x86}
        addtype('$void_nearpointer',voidnearpointertype);
        addtype('$void_nearcspointer',voidnearcspointertype);
        addtype('$void_neardspointer',voidneardspointertype);
        addtype('$void_nearsspointer',voidnearsspointertype);
        addtype('$void_nearespointer',voidnearespointertype);
        addtype('$void_nearfspointer',voidnearfspointertype);
        addtype('$void_neargspointer',voidneargspointertype);
  {$ifdef i8086}
        addtype('$void_farpointer',voidfarpointertype);
        addtype('$void_hugepointer',voidhugepointertype);
        addtype('$char_nearpointer',charnearpointertype);
        addtype('$char_farpointer',charfarpointertype);
        addtype('$char_hugepointer',charhugepointertype);
        addtype('$byte_farpointer',bytefarpointertype);
        addtype('$word_farpointer',wordfarpointertype);
        addtype('$longint_farpointer',longintfarpointertype);
  {$endif i8086}
{$endif x86}
        addtype('$openchararray',openchararraytype);
        addtype('$file',cfiletype);
        if f_variants in features then
          begin
            addtype('$variant',cvarianttype);
            addtype('$olevariant',colevarianttype);
          end;
        if init_settings.fputype<>fpu_none then
          begin
            addtype('$s32real',s32floattype);
            addtype('$s64real',s64floattype);
            addtype('$s80real',s80floattype);
            addtype('$sc80real',sc80floattype);
          end;
        addtype('$s64currency',s64currencytype);
        if not(target_info.system in systems_managed_vm) then
          begin
            { Add a type for virtual method tables }
            hrecst:=trecordsymtable.create('',current_settings.packrecords,current_settings.alignment.recordalignmin);
            vmttype:=crecorddef.create('',hrecst);
            pvmttype:=cpointerdef.create(vmttype);
            { can't use addtype for pvmt because the rtti of the pointed
              type is not available. The rtti for pvmt will be written implicitly
              by thev tblarray below }
            systemunit.insert(ctypesym.create('$pvmt',pvmttype));
            addfield(hrecst,cfieldvarsym.create('$length',vs_value,sizesinttype,[]));
            addfield(hrecst,cfieldvarsym.create('$mlength',vs_value,sizesinttype,[]));
            addfield(hrecst,cfieldvarsym.create('$parent',vs_value,pvmttype,[]));
            { it seems vmttype is used both for TP objects and Delphi classes,
              so the next entry could either be the first virtual method (vm1)
              (object) or the class name (class). We can't easily create separate
              vtable formats for both, as gdb is hard coded to search for
              __vtbl_ptr_type in all cases (JM) }
            addfield(hrecst,cfieldvarsym.create('$vm1_or_classname',vs_value,cpointerdef.create(cshortstringtype),[]));
            vmtarraytype:=carraydef.create(0,0,s32inttype);
            tarraydef(vmtarraytype).elementdef:=voidpointertype;
            addfield(hrecst,cfieldvarsym.create('$__pfn',vs_value,vmtarraytype,[]));
            addtype('$__vtbl_ptr_type',vmttype);
            vmtarraytype:=carraydef.create(0,1,s32inttype);
            tarraydef(vmtarraytype).elementdef:=pvmttype;
            addtype('$vtblarray',vmtarraytype);
          end;
        { Add a type for methodpointers }
        hrecst:=trecordsymtable.create('',1,current_settings.alignment.recordalignmin);
        addfield(hrecst,cfieldvarsym.create('$proc',vs_value,voidcodepointertype,[]));
        addfield(hrecst,cfieldvarsym.create('$self',vs_value,voidpointertype,[]));
        methodpointertype:=crecorddef.create('',hrecst);
        addtype('$methodpointer',methodpointertype);
        { Add a type for nested proc pointers }
        hrecst:=trecordsymtable.create('',1,current_settings.alignment.recordalignmin);
        addfield(hrecst,cfieldvarsym.create('$proc',vs_value,voidcodepointertype,[]));
        addfield(hrecst,cfieldvarsym.create('$parentfp',vs_value,parentfpvoidpointertype,[]));
        nestedprocpointertype:=crecorddef.create('',hrecst);
        addtype('$nestedprocpointer',nestedprocpointertype);
{$ifdef llvm}
        llvm_metadatatype:=cpointerdef.create(voidtype);
        { if this gets renamed, also adjust agllvm so it still writes the identifier of this type as "metadata" }
        addtype('$metadata',llvm_metadatatype);
{$endif}
        symtablestack.pop(systemunit);
      end;


    procedure load_intern_types;
      {
        Load all default definitions for consts from the system unit
      }

        procedure loadtype(const s:string;var def:tdef);
        var
          srsym : ttypesym;
        begin
          srsym:=search_system_type(s);
          def:=srsym.typedef;
        end;

      var
        oldcurrentmodule : tmodule;
      begin
{$ifndef FPC_SUPPORT_X87_TYPES_ON_WIN64}
        if target_info.system=system_x86_64_win64 then
          pbestrealtype:=@s64floattype;
{$endif FPC_SUPPORT_X87_TYPES_ON_WIN64}

        oldcurrentmodule:=current_module;
        set_current_module(nil);
        loadtype('byte',u8inttype);
        loadtype('shortint',s8inttype);
        loadtype('word',u16inttype);
        loadtype('smallint',s16inttype);
        loadtype('uint24',u24inttype);
        loadtype('sint24',s24inttype);
        loadtype('ulong',u32inttype);
        loadtype('longint',s32inttype);
        loadtype('uint40',u40inttype);
        loadtype('sint40',s40inttype);
        loadtype('uint48',u48inttype);
        loadtype('sint48',s48inttype);
        loadtype('uint56',u56inttype);
        loadtype('sint56',s56inttype);
        loadtype('qword',u64inttype);
        loadtype('int64',s64inttype);
        loadtype('uint128',u128inttype);
        loadtype('int128',s128inttype);
        loadtype('undefined',cundefinedtype);
        loadtype('formal',cformaltype);
        loadtype('typedformal',ctypedformaltype);
        loadtype('void',voidtype);
        loadtype('void_pointer',voidpointertype);
        loadtype('char',cansichartype);
        loadtype('widechar',cwidechartype);
        loadtype('shortstring',cshortstringtype);
        loadtype('longstring',clongstringtype);
        loadtype('ansistring',cansistringtype);
        loadtype('widestring',cwidestringtype);
        loadtype('unicodestring',cunicodestringtype);
        loadtype('openshortstring',openshortstringtype);
        loadtype('openchararray',openchararraytype);
        if init_settings.fputype <> fpu_none then
          begin
            loadtype('s32real',s32floattype);
            loadtype('s64real',s64floattype);
            loadtype('s80real',s80floattype);
            loadtype('sc80real',sc80floattype);
          end;
        loadtype('s64currency',s64currencytype);
        loadtype('boolean',pasbool1type);
        loadtype('boolean8',pasbool8type);
        loadtype('boolean16',pasbool16type);
        loadtype('boolean32',pasbool32type);
        loadtype('boolean64',pasbool64type);
        loadtype('bytebool',bool8type);
        loadtype('wordbool',bool16type);
        loadtype('longbool',bool32type);
        loadtype('qwordbool',bool64type);
        loadtype('char_pointer',charpointertype);
        loadtype('widechar_pointer',widecharpointertype);
        loadtype('parentfp_void_pointer',parentfpvoidpointertype);
{$ifdef x86}
        loadtype('void_nearpointer',voidnearpointertype);
        loadtype('void_nearcspointer',voidnearcspointertype);
        loadtype('void_neardspointer',voidneardspointertype);
        loadtype('void_nearsspointer',voidnearsspointertype);
        loadtype('void_nearespointer',voidnearespointertype);
        loadtype('void_nearfspointer',voidnearfspointertype);
        loadtype('void_neargspointer',voidneargspointertype);
  {$ifdef i8086}
        loadtype('void_farpointer',voidfarpointertype);
        loadtype('void_hugepointer',voidhugepointertype);
        loadtype('char_nearpointer',charnearpointertype);
        loadtype('char_farpointer',charfarpointertype);
        loadtype('char_hugepointer',charhugepointertype);
        loadtype('byte_farpointer',bytefarpointertype);
        loadtype('word_farpointer',wordfarpointertype);
        loadtype('longint_farpointer',longintfarpointertype);
  {$endif i8086}
{$endif x86}
{$ifdef llvm}
        loadtype('llvmbool1',llvmbool1type);
{$endif llvm}
        loadtype('file',cfiletype);
        if not(target_info.system in systems_managed_vm) then
          begin
            loadtype('pvmt',pvmttype);
            loadtype('vtblarray',vmtarraytype);
            loadtype('__vtbl_ptr_type',vmttype);
          end;
        if f_variants in features then
          begin
            loadtype('variant',cvarianttype);
            loadtype('olevariant',colevarianttype);
          end;
        loadtype('methodpointer',methodpointertype);
        loadtype('nestedprocpointer',nestedprocpointertype);
{$ifdef llvm}
        loadtype('metadata',llvm_metadatatype);
{$endif}
        loadtype('HRESULT',hresultdef);
        loadtype('TTYPEKIND',typekindtype);
        set_default_int_types;
        set_default_ptr_types;
        set_current_module(oldcurrentmodule);
      end;


    procedure registernodes;
      {
        Register all possible nodes in the nodeclass array that
        will be used for loading the nodes from a ppu
      }
      begin
        nodeclass[addn]:=caddnode;
        nodeclass[muln]:=caddnode;
        nodeclass[subn]:=caddnode;
        nodeclass[divn]:=cmoddivnode;
        nodeclass[symdifn]:=caddnode;
        nodeclass[modn]:=cmoddivnode;
        nodeclass[assignn]:=cassignmentnode;
        nodeclass[loadn]:=cloadnode;
        nodeclass[rangen]:=crangenode;
        nodeclass[ltn]:=caddnode;
        nodeclass[lten]:=caddnode;
        nodeclass[gtn]:=caddnode;
        nodeclass[gten]:=caddnode;
        nodeclass[equaln]:=caddnode;
        nodeclass[unequaln]:=caddnode;
        nodeclass[inn]:=cinnode;
        nodeclass[orn]:=caddnode;
        nodeclass[xorn]:=caddnode;
        nodeclass[shrn]:=cshlshrnode;
        nodeclass[shln]:=cshlshrnode;
        nodeclass[slashn]:=caddnode;
        nodeclass[andn]:=caddnode;
        nodeclass[subscriptn]:=csubscriptnode;
        nodeclass[derefn]:=cderefnode;
        nodeclass[addrn]:=caddrnode;
        nodeclass[ordconstn]:=cordconstnode;
        nodeclass[typeconvn]:=ctypeconvnode;
        nodeclass[calln]:=ccallnode;
        nodeclass[callparan]:=ccallparanode;
        nodeclass[realconstn]:=crealconstnode;
        nodeclass[unaryminusn]:=cunaryminusnode;
        nodeclass[unaryplusn]:=cunaryplusnode;
        nodeclass[asmn]:=casmnode;
        nodeclass[vecn]:=cvecnode;
        nodeclass[pointerconstn]:=cpointerconstnode;
        nodeclass[stringconstn]:=cstringconstnode;
        nodeclass[notn]:=cnotnode;
        nodeclass[inlinen]:=cinlinenode;
        nodeclass[niln]:=cnilnode;
        nodeclass[errorn]:=cerrornode;
        nodeclass[typen]:=ctypenode;
        nodeclass[setelementn]:=csetelementnode;
        nodeclass[setconstn]:=csetconstnode;
        nodeclass[blockn]:=cblocknode;
        nodeclass[statementn]:=cstatementnode;
        nodeclass[ifn]:=cifnode;
        nodeclass[breakn]:=cbreaknode;
        nodeclass[continuen]:=ccontinuenode;
        nodeclass[whilerepeatn]:=cwhilerepeatnode;
        nodeclass[forn]:=cfornode;
        nodeclass[exitn]:=cexitnode;
        nodeclass[casen]:=ccasenode;
        nodeclass[labeln]:=clabelnode;
        nodeclass[goton]:=cgotonode;
        nodeclass[tryexceptn]:=ctryexceptnode;
        nodeclass[raisen]:=craisenode;
        nodeclass[tryfinallyn]:=ctryfinallynode;
        nodeclass[onn]:=connode;
        nodeclass[isn]:=cisnode;
        nodeclass[asn]:=casnode;
        nodeclass[starstarn]:=caddnode;
        nodeclass[arrayconstructorn]:=carrayconstructornode;
        nodeclass[arrayconstructorrangen]:=carrayconstructorrangenode;
        nodeclass[tempcreaten]:=ctempcreatenode;
        nodeclass[temprefn]:=ctemprefnode;
        nodeclass[tempdeleten]:=ctempdeletenode;
        nodeclass[addoptn]:=caddnode;
        nodeclass[nothingn]:=cnothingnode;
        nodeclass[loadvmtaddrn]:=cloadvmtaddrnode;
        nodeclass[guidconstn]:=cguidconstnode;
        nodeclass[rttin]:=crttinode;
        nodeclass[loadparentfpn]:=cloadparentfpnode;
      end;


    procedure registertais;
      {
        Register all possible tais in the taiclass array that
        will be used for loading the tais from a ppu
      }
      begin
        aiclass[ait_none]:=nil;
        aiclass[ait_align]:=tai_align;
        aiclass[ait_section]:=tai_section;
        aiclass[ait_comment]:=tai_comment;
        aiclass[ait_string]:=tai_string;
        aiclass[ait_instruction]:=taicpu;
        aiclass[ait_datablock]:=tai_datablock;
        aiclass[ait_symbol]:=tai_symbol;
        aiclass[ait_symbol_end]:=tai_symbol_end;
        aiclass[ait_directive]:=tai_directive;
        aiclass[ait_label]:=tai_label;
        aiclass[ait_const]:=tai_const;
        aiclass[ait_realconst]:=tai_realconst;
        aiclass[ait_stab]:=tai_stab;
        aiclass[ait_force_line]:=tai_force_line;
        aiclass[ait_function_name]:=tai_function_name;
        aiclass[ait_symbolpair]:=tai_symbolpair;
        aiclass[ait_cutobject]:=tai_cutobject;
        aiclass[ait_regalloc]:=tai_regalloc;
        aiclass[ait_tempalloc]:=tai_tempalloc;
        aiclass[ait_marker]:=tai_marker;
        aiclass[ait_seh_directive]:=tai_seh_directive;
{$ifdef JVM}
        aiclass[ait_jvar]:=tai_jvar;
        aiclass[ait_jcatch]:=tai_jcatch;
{$endif JVM}
      end;

end.
