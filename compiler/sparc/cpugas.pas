{
    Copyright (c) 1999-2003 by Florian Klaempfl

    This unit implements an asmoutput class for SPARC AT&T syntax

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
unit cpugas;

{$i fpcdefs.inc}

interface

    uses
      cpubase,systems,
      aasmtai,aasmdata,aasmcpu,assemble,aggas,
      cgutils,globtype;

    type
      TGasSPARC=class(TGnuAssembler)
        constructor create(info: pasminfo; smart: boolean); override;
        {# Constructs the command line for calling the assembler }
        function MakeCmdLine: TCmdStr; override;
      end;

     TSPARCInstrWriter=class(TCPUInstrWriter)
       procedure WriteInstruction(hp:Tai);override;
       function GetReferenceString(var ref : TReference):string;
       function getopstr(const Oper:TOper):string;
     end;

implementation

    uses
      cutils,globals,cpuinfo,procinfo,
      verbose,itcpugas,cgbase;


{****************************************************************************}
{                         GNU PPC Assembler writer                           }
{****************************************************************************}

    constructor TGasSPARC.create(info: pasminfo; smart: boolean);
      begin
        inherited;
        InstrWriter := TSPARCInstrWriter.create(self);
      end;


    function TGasSPARC.MakeCmdLine: TCmdStr;
      begin
         result := Inherited MakeCmdLine;

         { ARCH selection }
         // Note for casual readers: gas (GNU as) uses -Av7, -Av8, -Av9 etc. on SPARC,
         // rather than variants of the -m option used by most other CPUs. Solaris as
         // uses -xarch=v7, -xarch=v8 etc., that form is not supported here since there
         // are probably other incompatibilties between the GNU and Solaris binutils
         // that need to be reviewed.
         //
         // v9 is required as the default since the RTL started using membar at 2.2.2.
         case current_settings.cputype of
           cpu_SPARC_V7: Replace(result,'$ARCH','-Av7');
           cpu_SPARC_V8: Replace(result,'$ARCH','-Av8')
         else
           Replace(result,'$ARCH','-Av9')
         end
      end;


    function TSPARCInstrWriter.GetReferenceString(var ref:TReference):string;
      begin
        result:='';
        if assigned(ref.symbol) then
          begin
            result:=ref.symbol.name;
            if assigned(ref.relsymbol) then
              result:=result+'-'+ref.relsymbol.name;
          end;
        if (ref.offset<0) then
          result:=result+tostr(ref.offset)
        else if (ref.offset>0) then
          begin
            if assigned(ref.symbol) then
              result:=result+'+';
            result:=result+tostr(ref.offset);
          end
        { asmreader appears to treat literal numbers as references }
        else if (ref.symbol=nil) and (ref.base=NR_NO) and (ref.index=NR_NO) then
          result:='0';

        case ref.refaddr of
          addr_high:
            result:='%hi('+result+')';
          addr_low:
            result:='%lo('+result+')';
        end;

        if assigned(ref.symbol) or (ref.offset<>0) then
          begin
            if (ref.base<>NR_NO) then
              begin
<<<<<<< HEAD
                if (ref.index<>NR_NO) then
                  InternalError(2013013001);
                if (result[1]='-') then
                  result:=gas_regname(ref.base)+result
                else
                  result:=gas_regname(ref.base)+'+'+result;
=======
                 if assigned(symbol) then
                   GetReferenceString:=symbol.name;
                 if offset>0 then
                   GetReferenceString:=GetReferenceString+'+'+ToStr(offset)
                 else if offset<0 then
                   GetReferenceString:=GetReferenceString+ToStr(offset);
                 case refaddr of
                   addr_high:
                     GetReferenceString:='%hi('+GetReferenceString+')';
                   addr_low:
                     GetReferenceString:='%lo('+GetReferenceString+')';
                 end;
>>>>>>> graemeg/fixes_2_2
              end
            else if (ref.index<>NR_NO) then
              InternalError(2013122501);
          end
        else
          begin
            if (ref.base<>NR_NO) then
              begin
<<<<<<< HEAD
                result:=gas_regname(ref.base);
                if (ref.index<>NR_NO) then
                  result:=result+'+'+gas_regname(ref.index);
              end
            else if (ref.index<>NR_NO) then
              result:=gas_regname(ref.index);
=======
{$ifdef extdebug}
                if assigned(symbol) and
                  not(refaddr in [addr_pic,addr_low]) then
                  internalerror(2003052601);
{$endif extdebug}
                if base<>NR_NO then
                  GetReferenceString:=GetReferenceString+gas_regname(base);
                if index=NR_NO then
                  begin
                    { if (Offset<simm13lo) or (Offset>simm13hi) then
                      internalerror(2003053008); }
                    if offset>0 then
                      GetReferenceString:=GetReferenceString+'+'+ToStr(offset)
                    else if offset<0 then
                      GetReferenceString:=GetReferenceString+ToStr(offset);
                    {
                    else if (offset=0) and not(assigned(symbol)) then
                      GetReferenceString:=GetReferenceString+ToStr(offset);
                    }
                    if assigned(symbol) then
                      begin
                        if refaddr=addr_low then
                          GetReferenceString:='%lo('+symbol.name+')+'+GetReferenceString
                        else
                          GetReferenceString:=symbol.name+'+'+GetReferenceString;
                      end;
                  end
                else
                  begin
{$ifdef extdebug}
                    if (Offset<>0) or assigned(symbol) then
                      internalerror(2003052603);
{$endif extdebug}
                    GetReferenceString:=GetReferenceString+'+'+gas_regname(index);
                  end;
              end;
>>>>>>> graemeg/fixes_2_2
          end;
      end;


    function TSPARCInstrWriter.getopstr(const Oper:TOper):string;
      begin
        with Oper do
          case typ of
            top_reg:
              getopstr:=gas_regname(reg);
            top_const:
              getopstr:=tostr(longint(val));
            top_ref:
<<<<<<< HEAD
<<<<<<< HEAD
              if (oper.ref^.refaddr in [addr_no,addr_pic]) or
                 ((oper.ref^.refaddr=addr_low) and ((oper.ref^.base<>NR_NO) or
                  (oper.ref^.index<>NR_NO))) then
=======
=======
>>>>>>> origin/fixes_2_2
              if (oper.ref^.refaddr in [addr_no,addr_pic]) or ((oper.ref^.refaddr=addr_low) and ((oper.ref^.base<>NR_NO) or
                (oper.ref^.index<>NR_NO))) then
>>>>>>> graemeg/fixes_2_2
                getopstr:='['+getreferencestring(ref^)+']'
              else
                getopstr:=getreferencestring(ref^);
            else
              internalerror(10001);
          end;
        end;


    procedure TSPARCInstrWriter.WriteInstruction(hp:Tai);

      procedure writePseudoInstruction(opc: TAsmOp);
        begin
          if (taicpu(hp).ops<>2) or
             (taicpu(hp).oper[0]^.typ<>top_reg) or
             (taicpu(hp).oper[1]^.typ<>top_reg) then
            internalerror(200401045);
          { Fxxxs %f<even>,%f<even> }
          owner.writer.AsmWriteln(#9+std_op2str[opc]+#9+getopstr(taicpu(hp).oper[0]^)+','+getopstr(taicpu(hp).oper[1]^));
          { FMOVs %f<odd>,%f<odd> }
          inc(taicpu(hp).oper[0]^.reg);
          inc(taicpu(hp).oper[1]^.reg);
          owner.writer.AsmWriteln(#9+std_op2str[A_FMOVs]+#9+getopstr(taicpu(hp).oper[0]^)+','+getopstr(taicpu(hp).oper[1]^));
          dec(taicpu(hp).oper[0]^.reg);
          dec(taicpu(hp).oper[1]^.reg);
        end;

      var
        Op:TAsmOp;
        s:String;
        i:Integer;
      begin
        if hp.typ<>ait_instruction then
          exit;
        op:=taicpu(hp).opcode;
        if (op=A_Bxx) and (taicpu(hp).condition in floatAsmConds) then
          op:=A_FBxx;
        { translate pseudoops, this should be move to a separate pass later, so it's done before
          peephole optimization }
        case op of
          A_FABSd:
            writePseudoInstruction(A_FABSs);

          A_FMOVd:
            writePseudoInstruction(A_FMOVs);

          A_FNEGd:
            writePseudoInstruction(A_FNEGs);

          else
            begin
              { call maybe not translated to call }
              s:=#9+std_op2str[op]+cond2str[taicpu(hp).condition];
              if taicpu(hp).delayslot_annulled then
                s:=s+',a';
              if taicpu(hp).ops>0 then
                begin
                  s:=s+#9+getopstr(taicpu(hp).oper[0]^);
                  for i:=1 to taicpu(hp).ops-1 do
                    s:=s+','+getopstr(taicpu(hp).oper[i]^);
                end;
              owner.writer.AsmWriteLn(s);
            end;
        end;
      end;


    const
      as_sparc_as_info : tasminfo =
         (
           id     : as_gas;
           idtxt  : 'AS';
           asmbin : 'as';
{$ifdef FPC_SPARC_V8_ONLY}
<<<<<<< HEAD
<<<<<<< HEAD
           asmcmd : '$PIC -o $OBJ $EXTRAOPT $ASM';
{$else}
           asmcmd : '$ARCH $PIC -o $OBJ $EXTRAOPT $ASM';
{$endif}
           supported_targets : [system_sparc_solaris,system_sparc_linux,system_sparc_embedded];
           flags : [af_needar,af_smartlink_sections];
=======
=======
>>>>>>> origin/fixes_2_2
           asmcmd : '-o $OBJ $ASM';
{$else}
           asmcmd : '-Av9 -o $OBJ $ASM';
{$endif}
           supported_target : system_any;
           flags : [af_allowdirect,af_needar,af_smartlink_sections];
>>>>>>> graemeg/fixes_2_2
           labelprefix : '.L';
           comment : '# ';
           dollarsign: '$';
         );

      as_sparc_gas_info : tasminfo =
         (
           id     : as_ggas;
           idtxt  : 'GAS';
           asmbin : 'gas';
<<<<<<< HEAD
<<<<<<< HEAD
           asmcmd : '$ARCH $PIC -o $OBJ $EXTRAOPT $ASM';
           supported_targets : [system_sparc_solaris,system_sparc_linux,system_sparc_embedded];
           flags : [af_needar,af_smartlink_sections];
=======
=======
>>>>>>> origin/fixes_2_2
           asmcmd : '-Av9 -o $OBJ $ASM';
           supported_target : system_any;
           flags : [af_allowdirect,af_needar,af_smartlink_sections];
>>>>>>> graemeg/fixes_2_2
           labelprefix : '.L';
           comment : '# ';
           dollarsign: '$';
         );

begin
  RegisterAssembler(as_SPARC_as_info,TGasSPARC);
  RegisterAssembler(as_SPARC_gas_info,TGasSPARC);
end.
