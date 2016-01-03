{
    Copyright (c) 1998-2002 by Florian Klaempfl

    This unit implements an asmoutput class for i386 AT&T syntax

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
{ This unit implements an asmoutput class for i386 AT&T syntax
}
unit agx86att;

{$i fpcdefs.inc}

interface

    uses
      cclasses,cpubase,systems,
      globals,globtype,cgutils,
      aasmbase,aasmtai,aasmdata,assemble,aggas;

    type
      Tx86ATTAssembler=class(TGNUassembler)
        constructor create(info: pasminfo; smart: boolean); override;
        function MakeCmdLine: TCmdStr; override;
      end;

      Tx86AppleGNUAssembler=class(TAppleGNUassembler)
        constructor create(info: pasminfo; smart: boolean); override;
      end;

      Tx86AoutGNUAssembler=class(TAoutGNUassembler)
        constructor create(info: pasminfo; smart: boolean); override;
      end;

      Tx86AoutGNUAssembler=class(TAoutGNUassembler)
        constructor create(smart: boolean); override;
      end;

      Tx86AoutGNUAssembler=class(TAoutGNUassembler)
        constructor create(smart: boolean); override;
      end;


     Tx86InstrWriter=class(TCPUInstrWriter)
       private
        procedure WriteReference(var ref : treference);
        procedure WriteOper(const o:toper);
        procedure WriteOper_jmp(const o:toper);
       protected
        fskipPopcountSuffix: boolean;
        { http://gcc.gnu.org/bugzilla/show_bug.cgi?id=56656 }
        fNoInterUnitMovQ: boolean;
       public
        procedure WriteInstruction(hp: tai);override;
     end;



  implementation

    uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      cutils,
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      globtype,
      cutils,systems,
>>>>>>> graemeg/cpstrnew
      verbose,
      itcpugas,
      cgbase,
      aasmcpu;


{****************************************************************************
                            Tx86ATTAssembler
 ****************************************************************************}

    constructor Tx86ATTAssembler.create(info: pasminfo; smart: boolean);
      begin
        inherited;
        InstrWriter := Tx86InstrWriter.create(self);
      end;

    function TX86ATTAssembler.MakeCmdLine: TCmdStr;
      var
        FormatName : string;
      begin
        result:=Inherited MakeCmdLine;
{$ifdef i386}
        case target_info.system of
          system_i386_go32v2:
            FormatName:='coff';
          system_i386_wdosx,
          system_i386_win32:
            FormatName:='win32';
          system_i386_embedded:
            FormatName:='obj';
          system_i386_linux,
          system_i386_beos:
            FormatName:='elf';
          system_i386_darwin:
            FormatName:='macho32';
        else
          FormatName:='elf';
        end;
{$endif i386}
{$ifdef x86_64}
        case target_info.system of
          system_x86_64_win64:
            FormatName:='win64';
          system_x86_64_darwin:
            FormatName:='macho64';
          system_x86_64_linux:
            FormatName:='elf64';
        else
          FormatName:='elf64';
        end;
{$endif x86_64}
        Replace(result,'$FORMAT',FormatName);
      end;

{****************************************************************************
                          Tx86AppleGNUAssembler
 ****************************************************************************}

    constructor Tx86AppleGNUAssembler.create(info: pasminfo; smart: boolean);
      begin
        inherited;
        InstrWriter := Tx86InstrWriter.create(self);
        { Apple's assembler does not support a size suffix for popcount }
        Tx86InstrWriter(InstrWriter).fskipPopcountSuffix := true;
        { Apple's assembler is broken regarding some movq suffix handling }
        Tx86InstrWriter(InstrWriter).fNoInterUnitMovQ := true;
      end;

{****************************************************************************
                          Tx86AoutGNUAssembler
 ****************************************************************************}

    constructor Tx86AoutGNUAssembler.create(info: pasminfo; smart: boolean);
      begin
        inherited;
        InstrWriter := Tx86InstrWriter.create(self);
      end;

{****************************************************************************
                          Tx86AoutGNUAssembler
 ****************************************************************************}

    constructor Tx86AoutGNUAssembler.create(smart: boolean);
      begin
        inherited create(smart);
        InstrWriter := Tx86InstrWriter.create(self);
      end;

{****************************************************************************
                          Tx86AoutGNUAssembler
 ****************************************************************************}

    constructor Tx86AoutGNUAssembler.create(smart: boolean);
      begin
        inherited create(smart);
        InstrWriter := Tx86InstrWriter.create(self);
      end;

{****************************************************************************
                            Tx86InstrWriter
 ****************************************************************************}

    procedure Tx86InstrWriter.WriteReference(var ref : treference);
      begin
        with ref do
         begin
           { do we have a segment prefix ? }
           { These are probably not correctly handled under GAS }
           { should be replaced by coding the segment override  }
           { directly! - DJGPP FAQ                              }
           if segment<>NR_NO then
             owner.writer.AsmWrite(gas_regname(segment)+':');
           if assigned(symbol) then
<<<<<<< HEAD
             owner.writer.AsmWrite(symbol.name);
           if assigned(relsymbol) then
             owner.writer.AsmWrite('-'+relsymbol.name);
=======
             owner.AsmWrite(symbol.name);
           if assigned(relsymbol) then
             owner.AsmWrite('-'+relsymbol.name);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           if ref.refaddr=addr_pic then
             begin
               { @GOT and @GOTPCREL references are only allowed for symbol alone,
                 indexing, relsymbol or offset cannot be present. }
               if assigned(relsymbol) or (offset<>0) or (index<>NR_NO) then
                 InternalError(2015011801);
{$ifdef x86_64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               if (base<>NR_RIP) then
                 InternalError(2015011802);
               owner.writer.AsmWrite('@GOTPCREL');
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
             begin
               { local symbols don't have to (and in case of Mac OS X: cannot)
                 be accessed via the GOT
               }
               if not assigned(ref.symbol) or
                  (ref.symbol.bind<>AB_LOCAL) then
                 owner.AsmWrite('@GOTPCREL');
             end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$else x86_64}
               owner.writer.AsmWrite('@GOT');
{$endif x86_64}
             end;
           if offset<0 then
             owner.writer.AsmWrite(tostr(offset))
           else
            if (offset>0) then
             begin
               if assigned(symbol) then
                owner.writer.AsmWrite('+'+tostr(offset))
               else
                owner.writer.AsmWrite(tostr(offset));
             end
           else if (index=NR_NO) and (base=NR_NO) and (not assigned(symbol)) then
             owner.writer.AsmWrite('0');
           if (index<>NR_NO) and (base=NR_NO) then
            begin
              if scalefactor in [0,1] then
                { Switching index to base position gives shorter
                  assembler instructions }
                begin
                  owner.writer.AsmWrite('('+gas_regname(index)+')');
                end
              else
                begin
                  owner.writer.AsmWrite('(,'+gas_regname(index));
                  if scalefactor<>0 then
                   owner.writer.AsmWrite(','+tostr(scalefactor)+')')
                  else
                   owner.writer.AsmWrite(')');
                end;
            end
           else
            if (index=NR_NO) and (base<>NR_NO) then
              owner.writer.AsmWrite('('+gas_regname(base)+')')
            else
             if (index<>NR_NO) and (base<>NR_NO) then
              begin
                owner.writer.AsmWrite('('+gas_regname(base)+','+gas_regname(index));
                if scalefactor<>0 then
                 owner.writer.AsmWrite(','+tostr(scalefactor));
                owner.writer.AsmWrite(')');
              end;
         end;
      end;


    procedure Tx86InstrWriter.WriteOper(const o:toper);
      begin
        case o.typ of
          top_reg :
            { Solaris assembler does not accept %st instead of %st(0) }
            if (owner.asminfo^.id=as_solaris_as) and (o.reg=NR_ST) then
              owner.writer.AsmWrite(gas_regname(NR_ST0))
            else
              owner.writer.AsmWrite(gas_regname(o.reg));
          top_ref :
            if o.ref^.refaddr in [addr_no,addr_pic,addr_pic_no_got] then
              WriteReference(o.ref^)
            else
              begin
                owner.writer.AsmWrite('$');
                if assigned(o.ref^.symbol) then
                 owner.writer.AsmWrite(o.ref^.symbol.name);
                if o.ref^.offset>0 then
                 owner.writer.AsmWrite('+'+tostr(o.ref^.offset))
                else
                 if o.ref^.offset<0 then
                  owner.writer.AsmWrite(tostr(o.ref^.offset))
                else
                 if not(assigned(o.ref^.symbol)) then
                   owner.writer.AsmWrite('0');
              end;
          top_const :
              owner.writer.AsmWrite('$'+tostr(o.val));
          else
            internalerror(10001);
        end;
      end;


    procedure Tx86InstrWriter.WriteOper_jmp(const o:toper);
      begin
        case o.typ of
          top_reg :
            owner.writer.AsmWrite('*'+gas_regname(o.reg));
          top_ref :
            begin
              if o.ref^.refaddr in [addr_no,addr_pic_no_got] then
                begin
                  owner.writer.AsmWrite('*');
                  WriteReference(o.ref^);
                end
              else
                begin
                  owner.writer.AsmWrite(o.ref^.symbol.name);
                  if o.ref^.refaddr=addr_pic then
                    owner.writer.AsmWrite('@PLT');
                  if o.ref^.offset>0 then
                   owner.writer.AsmWrite('+'+tostr(o.ref^.offset))
                  else
                   if o.ref^.offset<0 then
                    owner.writer.AsmWrite(tostr(o.ref^.offset));
                end;
            end;
          top_const :
            owner.writer.AsmWrite(tostr(o.val));
          else
            internalerror(10001);
        end;
      end;


    procedure Tx86InstrWriter.WriteInstruction(hp: tai);
      var
       op       : tasmop;
       val      : aint;
       calljmp  : boolean;
       need_second_mov : boolean;
       i        : integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       sreg     : string;
=======
       comment  : tai_comment;
>>>>>>> graemeg/cpstrnew
=======
       comment  : tai_comment;
>>>>>>> graemeg/cpstrnew
=======
       comment  : tai_comment;
>>>>>>> graemeg/cpstrnew
=======
       comment  : tai_comment;
>>>>>>> origin/cpstrnew
      begin
        if hp.typ <> ait_instruction then
          exit;
        taicpu(hp).SetOperandOrder(op_att);
        op:=taicpu(hp).opcode;
        calljmp:=is_calljmp(op);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

        // BUGFIX GAS-assembler
        // Intel "Intel 64 and IA-32 Architectures Software Developers manual 12/2011"
        // Intel:       VCVTDQ2PD  YMMREG, YMMREG/mem128 ((intel syntax))
        // GAS:         VCVTDQ2PD  YMMREG, XMMREG/mem128 ((intel syntax))
        if (op = A_VCVTDQ2PD) and
           (taicpu(hp).ops = 2) and
           (taicpu(hp).oper[0]^.typ = top_reg) and
           (taicpu(hp).oper[1]^.typ = top_reg) then
        begin
          if ((taicpu(hp).oper[0]^.ot and OT_YMMREG) = OT_YMMREG) and
             ((taicpu(hp).oper[1]^.ot and OT_YMMREG) = OT_YMMREG) then
          begin
            // change registertype in oper[0] from OT_YMMREG to OT_XMMREG
            taicpu(hp).oper[0]^.ot := taicpu(hp).oper[0]^.ot and not(OT_YMMREG) or OT_XMMREG;

            sreg := gas_regname(taicpu(hp).oper[0]^.reg);
            if (copy(sreg, 1, 2) = '%y') or
               (copy(sreg, 1, 2) = '%Y') then
              taicpu(hp).oper[0]^.reg := gas_regnum_search('%x' + copy(sreg, 3, length(sreg) - 2));
          end;
        end;
        { see fNoInterUnitMovQ declaration comment }
        if fNoInterUnitMovQ then
          begin
            if ((op=A_MOVQ) or
                (op=A_VMOVQ)) and
               (((taicpu(hp).oper[0]^.typ=top_reg) and
                 (getregtype(taicpu(hp).oper[0]^.reg)=R_INTREGISTER)) or
                ((taicpu(hp).oper[1]^.typ=top_reg) and
                 (getregtype(taicpu(hp).oper[1]^.reg)=R_INTREGISTER))) then
              begin
                if op=A_MOVQ then
                  op:=A_MOVD
                else
                  op:=A_VMOVD;
                taicpu(hp).opcode:=op;
              end;
          end;
        owner.writer.AsmWrite(#9);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        { constant values in the 32 bit range are sign-extended to
          64 bits, but this is not what we want.  PM 2010-09-02
          the fix consists of simply setting only the 4-byte register
          as the upper 4-bytes will be zeroed at the same time. }
        need_second_mov:=false;
        if (op=A_MOV) and (taicpu(hp).opsize=S_Q) and
           (taicpu(hp).oper[0]^.typ = top_const) then
           begin
             val := taicpu(hp).oper[0]^.val;
	     if (val > int64($7fffffff)) and (val < int64($100000000)) then
               begin
                 owner.AsmWrite(target_asm.comment);
                 owner.AsmWritePChar('Fix for Win64-GAS bug');
                 owner.AsmLn;
                 taicpu(hp).opsize:=S_L;
                 if taicpu(hp).oper[1]^.typ = top_reg then
                   setsubreg(taicpu(hp).oper[1]^.reg,R_SUBD)
                 else if taicpu(hp).oper[1]^.typ = top_ref then
                   need_second_mov:=true
                 else
                   internalerror(2010090211);
               end;
           end;
        owner.AsmWrite(#9);
>>>>>>> graemeg/cpstrnew
        { movsd should not be translated to movsl when there
          are (xmm) arguments }
        if (op=A_MOVSD) and (taicpu(hp).ops>0) then
          owner.writer.AsmWrite('movsd')
        else
          owner.writer.AsmWrite(gas_op2str[op]);
        owner.writer.AsmWrite(cond2str[taicpu(hp).condition]);
        { suffix needed ?  fnstsw,fldcw don't support suffixes
          with binutils 2.9.5 under linux }
{        if (Taicpu(hp).oper[0]^.typ=top_reg) and
            (Taicpu(hp).oper[0]^.reg.enum>lastreg) then
          internalerror(200301081);}

        if (not calljmp) and
           (gas_needsuffix[op]<>AttSufNONE) and
           (op<>A_FNSTSW) and
           (op<>A_FSTSW) and
           (op<>A_FNSTCW) and
           (op<>A_FSTCW) and
           (op<>A_FLDCW) and
           (not fskipPopcountSuffix or
            (op<>A_POPCNT)) and
           ((owner.asminfo^.id<>as_solaris_as) or (op<>A_Jcc) and (op<>A_SETcc)) and
           not(
               (taicpu(hp).ops<>0) and
               (taicpu(hp).oper[0]^.typ=top_reg) and
               (getregtype(taicpu(hp).oper[0]^.reg)=R_FPUREGISTER)
              ) then
        begin
          if (gas_needsuffix[op] = AttSufMM)then
          begin
            for i:=0 to taicpu(hp).ops-1 do
            begin

              if (taicpu(hp).oper[i]^.typ = top_ref) then
              begin
                case taicpu(hp).oper[i]^.ot and OT_SIZE_MASK of
                   OT_BITS32: begin
                                owner.writer.AsmWrite(gas_opsize2str[S_L]);
                                break;
                              end;
                   OT_BITS64: begin
                                owner.writer.AsmWrite(gas_opsize2str[S_Q]);
                                break;
                              end;
                  OT_BITS128: begin
                                owner.writer.AsmWrite(gas_opsize2str[S_XMM]);
                                break;
                              end;
                  OT_BITS256: begin
                                owner.writer.AsmWrite(gas_opsize2str[S_YMM]);
                                break;
                              end;
                           0: begin
                                owner.writer.AsmWrite(gas_opsize2str[taicpu(hp).opsize]);
                                break;
                              end;
                end;
              end;
            end;
          end
          else owner.writer.AsmWrite(gas_opsize2str[taicpu(hp).opsize]);
        end;

        { process operands }
        if taicpu(hp).ops<>0 then
          begin
            if calljmp then
             begin
               owner.writer.AsmWrite(#9);
               WriteOper_jmp(taicpu(hp).oper[0]^);
             end
            else
             begin
               for i:=0 to taicpu(hp).ops-1 do
                 begin
                   if i=0 then
                     owner.writer.AsmWrite(#9)
                   else
                     owner.writer.AsmWrite(',');
                   WriteOper(taicpu(hp).oper[i]^);
                 end;
             end;
          end;
<<<<<<< HEAD
        owner.writer.AsmLn;
=======
        owner.AsmLn;
        if need_second_mov then
          begin
            taicpu(hp).oper[0]^.val:=0;
            inc(taicpu(hp).oper[1]^.ref^.offset,4);
            WriteInstruction(hp);
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      end;


{*****************************************************************************
                                  Initialize
*****************************************************************************}

    const
{$ifdef x86_64}
       as_x86_64_as_info : tasminfo =
          (
            id     : as_gas;
            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '--64 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_linux,system_x86_64_freebsd,
                                 system_x86_64_win64,system_x86_64_embedded,
                                 system_x86_64_openbsd,system_x86_64_netbsd,
                                 system_x86_64_dragonfly,system_x86_64_aros];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );

       as_x86_64_yasm_info : tasminfo =
          (
            id     : as_yasm;
            idtxt  : 'YASM';
            asmbin : 'yasm';
            asmcmd : '-a x86 -p gas -f $FORMAT -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_linux,system_x86_64_freebsd,system_x86_64_win64,system_x86_64_embedded];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );

       as_x86_64_gas_info : tasminfo =
          (
            id     : as_ggas;
            idtxt  : 'GAS';
            asmbin : 'gas';
            asmcmd : '--64 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_solaris];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );

       as_x86_64_gas_info : tasminfo =
          (
            id     : as_ggas;
            idtxt  : 'GAS';
            asmbin : 'gas';
            asmcmd : '--64 -o $OBJ $ASM';
            supported_targets : [system_x86_64_solaris];
            flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
          );

       as_x86_64_gas_info : tasminfo =
          (
            id     : as_ggas;
            idtxt  : 'GAS';
            asmbin : 'gas';
            asmcmd : '--64 -o $OBJ $ASM';
            supported_targets : [system_x86_64_solaris];
            flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
          );

       as_x86_64_gas_info : tasminfo =
          (
            id     : as_ggas;
            idtxt  : 'GAS';
            asmbin : 'gas';
            asmcmd : '--64 -o $OBJ $ASM';
            supported_targets : [system_x86_64_solaris];
            flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
          );


<<<<<<< HEAD



       as_x86_64_solaris_info : tasminfo =
          (
            id     : as_solaris_as;
            idtxt  : 'AS-SOL';
            asmbin : 'as';
            asmcmd : ' -m64 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_x86_64_solaris];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );


=======
>>>>>>> origin/cpstrnew

       as_x86_64_gas_darwin_info : tasminfo =
          (
            id     : as_darwin;
            idtxt  : 'AS-DARWIN';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM -arch x86_64';
            supported_targets : [system_x86_64_darwin,system_x86_64_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );

       as_x86_64_clang_darwin_info : tasminfo =
          (
            id     : as_clang;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-c -o $OBJ $EXTRAOPT -arch x86_64 $DARWINVERSION -x assembler $ASM';
            supported_targets : [system_x86_64_darwin,system_x86_64_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );

{$else x86_64}
       as_i386_as_info : tasminfo =
          (
            id     : as_gas;
            idtxt  : 'AS';
            asmbin : 'as';
            asmcmd : '--32 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_GO32V2,system_i386_linux,system_i386_Win32,system_i386_freebsd,system_i386_solaris,system_i386_beos,
                                system_i386_netbsd,system_i386_Netware,system_i386_qnx,system_i386_wdosx,system_i386_openbsd,
                                system_i386_netwlibc,system_i386_wince,system_i386_embedded,system_i386_symbian,system_i386_haiku,system_x86_6432_linux,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                                system_i386_nativent,system_i386_android,system_i386_aros];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                                system_i386_nativent];
            flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf];
>>>>>>> graemeg/cpstrnew
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_yasm_info : tasminfo =
          (
            id     : as_yasm;
            idtxt  : 'YASM';
            asmbin : 'yasm';
            asmcmd : '-a x86 -p gas -f $FORMAT -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_GO32V2,system_i386_linux,system_i386_Win32,system_i386_freebsd,system_i386_solaris,system_i386_beos,
                                system_i386_netbsd,system_i386_Netware,system_i386_qnx,system_i386_wdosx,system_i386_openbsd,
                                system_i386_netwlibc,system_i386_wince,system_i386_embedded,system_i386_symbian,system_i386_haiku,system_x86_6432_linux,
                                system_i386_nativent];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );


       as_i386_as_aout_info : tasminfo =
          (
            id           : as_i386_as_aout;
            idtxt  : 'AS_AOUT';
            asmbin : 'as';
            asmcmd : '-o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_linux,system_i386_OS2,system_i386_freebsd,system_i386_netbsd,system_i386_openbsd,system_i386_EMX,system_i386_embedded];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            flags : [af_needar,af_stabs_use_function_absolute_addresses];
=======
            flags : [af_allowdirect,af_needar,af_stabs_use_function_absolute_addresses];
>>>>>>> graemeg/cpstrnew
=======
            flags : [af_allowdirect,af_needar,af_stabs_use_function_absolute_addresses];
>>>>>>> graemeg/cpstrnew
=======
            flags : [af_allowdirect,af_needar,af_stabs_use_function_absolute_addresses];
>>>>>>> graemeg/cpstrnew
=======
            flags : [af_allowdirect,af_needar,af_stabs_use_function_absolute_addresses];
>>>>>>> origin/cpstrnew
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );


       as_i386_gas_darwin_info : tasminfo =
          (
            id     : as_darwin;
            idtxt  : 'AS-DARWIN';
            asmbin : 'as';
<<<<<<< HEAD
            asmcmd : '-o $OBJ $EXTRAOPT $ASM -arch i386';
            supported_targets : [system_i386_darwin,system_i386_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
=======
            asmcmd : '-o $OBJ $ASM -arch i386';
            supported_targets : [system_i386_darwin,system_i386_iphonesim];
            flags : [af_allowdirect,af_needar,af_smartlink_sections,af_supports_dwarf,af_stabs_use_function_absolute_addresses];
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_clang_darwin_info : tasminfo =
          (
            id     : as_clang;
            idtxt  : 'CLANG';
            asmbin : 'clang';
            asmcmd : '-c -o $OBJ $EXTRAOPT -arch i386 $DARWINVERSION -x assembler $ASM';
            supported_targets : [system_i386_darwin,system_i386_iphonesim];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
=======
>>>>>>> origin/cpstrnew
            labelprefix : 'L';
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_gas_info : tasminfo =
          (
            id     : as_ggas;
            idtxt  : 'GAS';
            asmbin : 'gas';
            asmcmd : '--32 -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_GO32V2,system_i386_linux,system_i386_Win32,system_i386_freebsd,system_i386_solaris,system_i386_beos,
                                system_i386_netbsd,system_i386_Netware,system_i386_qnx,system_i386_wdosx,system_i386_openbsd,
                                system_i386_netwlibc,system_i386_wince,system_i386_embedded,system_i386_symbian,system_i386_haiku,
                                system_x86_6432_linux,system_i386_android];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );

       as_i386_solaris_info : tasminfo =
          (
            id     : as_solaris_as;
            idtxt  : 'AS-SOL';
            asmbin : 'as';
            asmcmd : ' -o $OBJ $EXTRAOPT $ASM';
            supported_targets : [system_i386_solaris];
            flags : [af_needar,af_smartlink_sections,af_supports_dwarf];
            labelprefix : '.L';
            comment : '# ';
            dollarsign: '$';
          );


{$endif x86_64}

initialization
{$ifdef x86_64}
  RegisterAssembler(as_x86_64_as_info,Tx86ATTAssembler);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  RegisterAssembler(as_x86_64_yasm_info,Tx86ATTAssembler);
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  RegisterAssembler(as_x86_64_gas_info,Tx86ATTAssembler);
  RegisterAssembler(as_x86_64_gas_darwin_info,Tx86AppleGNUAssembler);
  RegisterAssembler(as_x86_64_clang_darwin_info,Tx86AppleGNUAssembler);
  RegisterAssembler(as_x86_64_solaris_info,Tx86ATTAssembler);
{$else x86_64}
  RegisterAssembler(as_i386_as_info,Tx86ATTAssembler);
  RegisterAssembler(as_i386_gas_info,Tx86ATTAssembler);
  RegisterAssembler(as_i386_yasm_info,Tx86ATTAssembler);
  RegisterAssembler(as_i386_gas_darwin_info,Tx86AppleGNUAssembler);
<<<<<<< HEAD
<<<<<<< HEAD
  RegisterAssembler(as_i386_clang_darwin_info,Tx86AppleGNUAssembler);
  RegisterAssembler(as_i386_as_aout_info,Tx86AoutGNUAssembler);
  RegisterAssembler(as_i386_solaris_info,Tx86ATTAssembler);
=======
  RegisterAssembler(as_i386_as_aout_info,Tx86AoutGNUAssembler);
>>>>>>> graemeg/fixes_2_2
=======
  RegisterAssembler(as_i386_as_aout_info,Tx86AoutGNUAssembler);
>>>>>>> origin/fixes_2_2
{$endif x86_64}
end.
