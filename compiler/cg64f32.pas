{
    Copyright (c) 1998-2002 by Florian Klaempfl
    Member of the Free Pascal development team

    This unit implements the code generation for 64 bit int
    arithmethics on 32 bit processors

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
{# This unit implements the code generation for 64 bit int arithmethics on
   32 bit processors.
}
unit cg64f32;

  {$i fpcdefs.inc}

  interface

    uses
       aasmbase,aasmtai,aasmdata,aasmcpu,
       cpubase,cpupara,
       cgbase,cgobj,parabase,cgutils,
       symtype
       ;

    type
      {# Defines all the methods required on 32-bit processors
         to handle 64-bit integers.
      }
      tcg64f32 = class(tcg64)
        procedure a_load64_const_ref(list : TAsmList;value : int64;const ref : treference);override;
        procedure a_load64_reg_ref(list : TAsmList;reg : tregister64;const ref : treference);override;
        procedure a_load64_ref_reg(list : TAsmList;const ref : treference;reg : tregister64);override;
        procedure a_load64_reg_reg(list : TAsmList;regsrc,regdst : tregister64);override;
        procedure a_load64_const_reg(list : TAsmList;value: int64;reg : tregister64);override;

        procedure a_load64_subsetref_reg(list : TAsmList; const sref: tsubsetreference; destreg: tregister64);override;
        procedure a_load64_reg_subsetref(list : TAsmList; fromreg: tregister64; const sref: tsubsetreference);override;
        procedure a_load64_const_subsetref(list: TAsmlist; a: int64; const sref: tsubsetreference);override;
        procedure a_load64_ref_subsetref(list : TAsmList; const fromref: treference; const sref: tsubsetreference);override;
        procedure a_load64_subsetref_subsetref(list: TAsmlist; const fromsref, tosref: tsubsetreference);override;
        procedure a_load64_subsetref_ref(list : TAsmList; const sref: tsubsetreference; const destref: treference);override;

        procedure a_load64_loc_reg(list : TAsmList;const l : tlocation;reg : tregister64);override;
        procedure a_load64_loc_ref(list : TAsmList;const l : tlocation;const ref : treference);override;
        procedure a_load64_const_loc(list : TAsmList;value : int64;const l : tlocation);override;
        procedure a_load64_reg_loc(list : TAsmList;reg : tregister64;const l : tlocation);override;


        procedure a_load64high_reg_ref(list : TAsmList;reg : tregister;const ref : treference);override;
        procedure a_load64low_reg_ref(list : TAsmList;reg : tregister;const ref : treference);override;
        procedure a_load64high_ref_reg(list : TAsmList;const ref : treference;reg : tregister);override;
        procedure a_load64low_ref_reg(list : TAsmList;const ref : treference;reg : tregister);override;
        procedure a_load64high_loc_reg(list : TAsmList;const l : tlocation;reg : tregister);override;
        procedure a_load64low_loc_reg(list : TAsmList;const l : tlocation;reg : tregister);override;

        procedure a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);override;
        procedure a_op64_reg_ref(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64; const ref: treference);override;
        procedure a_op64_const_loc(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const l: tlocation);override;
        procedure a_op64_reg_loc(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64;const l : tlocation);override;
        procedure a_op64_loc_reg(list : TAsmList;op:TOpCG;size : tcgsize;const l : tlocation;reg : tregister64);override;
        procedure a_op64_const_ref(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const ref : treference);override;

        procedure a_load64_reg_cgpara(list : TAsmList;reg : tregister64;const paraloc : tcgpara);override;
        procedure a_load64_const_cgpara(list : TAsmList;value : int64;const paraloc : tcgpara);override;
        procedure a_load64_ref_cgpara(list : TAsmList;const r : treference;const paraloc : tcgpara);override;
        procedure a_load64_loc_cgpara(list : TAsmList;const l : tlocation;const paraloc : tcgpara);override;

        procedure a_loadmm_intreg64_reg(list: TAsmList; mmsize: tcgsize; intreg: tregister64; mmreg: tregister);override;
        procedure a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64);override;
        {# This routine tries to optimize the a_op64_const_reg operation, by
           removing superfluous opcodes. Returns TRUE if normal processing
           must continue in op64_const_reg, otherwise, everything is processed
           entirely in this routine, by emitting the appropriate 32-bit opcodes.
        }
        function optimize64_op_const_reg(list: TAsmList; var op: topcg; var a : int64; var reg: tregister64): boolean;override;

        procedure g_rangecheck64(list: TAsmList; const l:tlocation;fromdef,todef: tdef); override;
      end;


  implementation

    uses
       globtype,systems,constexp,
       verbose,cutils,
       symbase,symconst,symdef,symtable,defutil,paramgr,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       tgobj,hlcgobj;
=======
       tgobj;
>>>>>>> graemeg/cpstrnew
=======
       tgobj;
>>>>>>> graemeg/cpstrnew
=======
       tgobj;
>>>>>>> graemeg/cpstrnew
=======
       tgobj;
>>>>>>> origin/cpstrnew

{****************************************************************************
                                     Helpers
****************************************************************************}

    procedure swap64(var q : int64);
      begin
         q:=(int64(lo(q)) shl 32) or hi(q);
      end;


    procedure splitparaloc64(const cgpara:tcgpara;var cgparalo,cgparahi:tcgpara);
      var
        paraloclo,paraloclo2,paraloclo3,paraloclo4,
        paralochi,paralochi2,paralochi3,paralochi4 : pcgparalocation;
        curparaloc : PCGParaLocation;
      begin
        if not(cgpara.size in [OS_64,OS_S64]) then
          internalerror(200408231);
        if not assigned(cgpara.location) then
          internalerror(200408201);
        { init lo/hi para }
        cgparahi.reset;
        if cgpara.size=OS_S64 then
          cgparahi.size:=OS_S32
        else
          cgparahi.size:=OS_32;
        cgparahi.intsize:=4;
        cgparahi.alignment:=cgpara.alignment;
        paralochi:=cgparahi.add_location;
        cgparalo.reset;
        cgparalo.size:=OS_32;
        cgparalo.intsize:=4;
        cgparalo.alignment:=cgpara.alignment;
        paraloclo:=cgparalo.add_location;
        case cgpara.locations_count of
          8:
            begin
              { 8 parameter fields? }
              { Order for multiple locations is always
                  paraloc^ -> high
                  paraloc^.next -> low }
              if (target_info.endian=ENDIAN_BIG) then
                begin
                  { is there any big endian 8 bit ALU/16 bit Addr CPU? }
                  internalerror(2015041001);
                  { paraloc^ -> high }
                  move(cgpara.location^,paralochi^,sizeof(paralochi^));
                  paralochi^.next:=nil;
                  paralochi2:=cgparahi.add_location;
                  move(cgpara.location^.next,paralochi2^,sizeof(paralochi2^));

                  { paraloc^.next^.next^ -> low }
                  move(cgpara.location^.next^.next^,paraloclo^,sizeof(paraloclo^));
                  paraloclo^.next:=nil;
                  paraloclo2:=cgparalo.add_location;
                  move(cgpara.location^.next^.next^.next^,paraloclo2^,sizeof(paraloclo2^));
                end
              else
                begin
                  { paraloc^ -> low }
                  move(cgpara.location^,paraloclo^,sizeof(paraloclo^));
                  paraloclo^.next:=nil;
                  paraloclo2:=cgparalo.add_location;
                  move(cgpara.location^.next^,paraloclo2^,sizeof(paraloclo2^));
                  paraloclo2^.next:=nil;
                  paraloclo3:=cgparalo.add_location;
                  move(cgpara.location^.next^.next^,paraloclo3^,sizeof(paraloclo3^));
                  paraloclo3^.next:=nil;
                  paraloclo4:=cgparalo.add_location;
                  move(cgpara.location^.next^.next^.next^,paraloclo4^,sizeof(paraloclo4^));

                  { paraloc^.next^.next^.next^.next^ -> high }
                  curparaloc:=cgpara.location^.next^.next^.next^.next;
                  move(curparaloc^,paralochi^,sizeof(paralochi^));
                  paralochi^.next:=nil;
                  paralochi2:=cgparahi.add_location;
                  move(curparaloc^.next^,paralochi2^,sizeof(paralochi2^));
                  paralochi2^.next:=nil;
                  paralochi3:=cgparahi.add_location;
                  move(curparaloc^.next^.next^,paralochi3^,sizeof(paralochi3^));
                  paralochi3^.next:=nil;
                  paralochi4:=cgparahi.add_location;
                  move(curparaloc^.next^.next^.next^,paralochi4^,sizeof(paralochi4^));
                end;

              { fix size }
              paraloclo^.size:=OS_8;
              paraloclo2^.size:=OS_8;
              paraloclo3^.size:=OS_8;
              paraloclo4^.size:=OS_8;
              paraloclo4^.next:=nil;
              paralochi^.size:=OS_8;
              paralochi2^.size:=OS_8;
              paralochi3^.size:=OS_8;
              paralochi4^.size:=OS_8;
              paralochi4^.next:=nil;
              if cgpara.size=OS_S64 then
                if target_info.endian=ENDIAN_BIG then
                  paralochi^.size:=OS_S8
                else
                  paraloclo4^.size:=OS_S8;
            end;
          4:
            begin
              { 4 parameter fields? }
              { Order for multiple locations is always
                  paraloc^ -> high
                  paraloc^.next -> low }
              if (target_info.endian=ENDIAN_BIG) then
                begin
                  { paraloc^ -> high }
                  move(cgpara.location^,paralochi^,sizeof(paralochi^));
                  paralochi^.next:=nil;
                  paralochi2:=cgparahi.add_location;
                  move(cgpara.location^.next,paralochi2^,sizeof(paralochi2^));

                  { paraloc^.next^.next^ -> low }
                  move(cgpara.location^.next^.next^,paraloclo^,sizeof(paraloclo^));
                  paraloclo^.next:=nil;
                  paraloclo2:=cgparalo.add_location;
                  move(cgpara.location^.next^.next^.next^,paraloclo2^,sizeof(paraloclo2^));
                end
              else
                begin
                  { paraloc^ -> low }
                  move(cgpara.location^,paraloclo^,sizeof(paraloclo^));
                  paraloclo^.next:=nil;
                  paraloclo2:=cgparalo.add_location;
                  move(cgpara.location^.next^,paraloclo2^,sizeof(paraloclo2^));

                  { paraloc^.next^.next -> high }
                  move(cgpara.location^.next^.next^,paralochi^,sizeof(paralochi^));
                  paralochi^.next:=nil;
                  paralochi2:=cgparahi.add_location;
                  move(cgpara.location^.next^.next^.next^,paralochi2^,sizeof(paralochi2^));
                end;

              { fix size }
              paraloclo^.size:=OS_16;
              paraloclo2^.size:=OS_16;
              paraloclo2^.next:=nil;
              paralochi^.size:=OS_16;
              paralochi2^.size:=OS_16;
              paralochi2^.next:=nil;
              if cgpara.size=OS_S64 then
                if target_info.endian=ENDIAN_BIG then
                  paralochi^.size:=OS_S16
                else
                  paraloclo2^.size:=OS_S16;
            end;
          2:
            begin
              { 2 parameter fields? }
              { Order for multiple locations is always
                  paraloc^ -> high
                  paraloc^.next -> low }
              if (target_info.endian=ENDIAN_BIG) then
                begin
                  { paraloc^ -> high
                    paraloc^.next -> low }
                  move(cgpara.location^,paralochi^,sizeof(paralochi^));
                  move(cgpara.location^.next^,paraloclo^,sizeof(paraloclo^));
                end
              else
                begin
                  { paraloc^ -> low
                    paraloc^.next -> high }
                  move(cgpara.location^,paraloclo^,sizeof(paraloclo^));
                  move(cgpara.location^.next^,paralochi^,sizeof(paralochi^));
                end;

              { fix size }
              paraloclo^.size:=cgparalo.size;
              paraloclo^.next:=nil;
              paralochi^.size:=cgparahi.size;
              paralochi^.next:=nil;
            end;
          1:
            begin
              { single parameter, this can only be in memory }
              if cgpara.location^.loc<>LOC_REFERENCE then
                internalerror(200408282);
              move(cgpara.location^,paraloclo^,sizeof(paraloclo^));
              move(cgpara.location^,paralochi^,sizeof(paralochi^));
              { for big endian low is at +4, for little endian high }
              if target_info.endian = endian_big then
                begin
                  inc(cgparalo.location^.reference.offset,4);
                  cgparalo.alignment:=newalignment(cgparalo.alignment,4);
                end
              else
                begin
                  inc(cgparahi.location^.reference.offset,4);
                  cgparahi.alignment:=newalignment(cgparahi.alignment,4);
                end;

              { fix size }
              paraloclo^.size:=cgparalo.size;
              paraloclo^.next:=nil;
              paralochi^.size:=cgparahi.size;
              paralochi^.next:=nil;
            end;
          else
            internalerror(2013051901);
        end;
      end;


{****************************************************************************
                                   TCG64F32
****************************************************************************}

    procedure tcg64f32.a_load64_reg_ref(list : TAsmList;reg : tregister64;const ref : treference);
      var
        tmpreg: tregister;
        tmpref: treference;
      begin
        if target_info.endian = endian_big then
          begin
            tmpreg:=reg.reglo;
            reg.reglo:=reg.reghi;
            reg.reghi:=tmpreg;
          end;
        cg.a_load_reg_ref(list,OS_32,OS_32,reg.reglo,ref);
        tmpref := ref;
        inc(tmpref.offset,4);
        cg.a_load_reg_ref(list,OS_32,OS_32,reg.reghi,tmpref);
      end;


    procedure tcg64f32.a_load64_const_ref(list : TAsmList;value : int64;const ref : treference);
      var
        tmpref: treference;
      begin
        if target_info.endian = endian_big then
          swap64(value);
        cg.a_load_const_ref(list,OS_32,longint(lo(value)),ref);
        tmpref := ref;
        inc(tmpref.offset,4);
        cg.a_load_const_ref(list,OS_32,longint(hi(value)),tmpref);
      end;


    procedure tcg64f32.a_load64_ref_reg(list : TAsmList;const ref : treference;reg : tregister64);
      var
        tmpreg: tregister;
        tmpref: treference;
      begin
        if target_info.endian = endian_big then
          begin
            tmpreg := reg.reglo;
            reg.reglo := reg.reghi;
            reg.reghi := tmpreg;
          end;
        tmpref := ref;
        if (tmpref.base=reg.reglo) then
         begin
           tmpreg:=cg.getaddressregister(list);
           cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,tmpref.base,tmpreg);
           tmpref.base:=tmpreg;
         end
        else
         { this works only for the i386, thus the i386 needs to override  }
         { this method and this method must be replaced by a more generic }
         { implementation FK                                              }
         if (tmpref.index=reg.reglo) then
          begin
            tmpreg:=cg.getaddressregister(list);
            cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,tmpref.index,tmpreg);
            tmpref.index:=tmpreg;
          end;
        cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg.reglo);
        inc(tmpref.offset,4);
        cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg.reghi);
      end;


    procedure tcg64f32.a_load64_reg_reg(list : TAsmList;regsrc,regdst : tregister64);

      begin
        cg.a_load_reg_reg(list,OS_32,OS_32,regsrc.reglo,regdst.reglo);
        cg.a_load_reg_reg(list,OS_32,OS_32,regsrc.reghi,regdst.reghi);
      end;


    procedure tcg64f32.a_load64_const_reg(list : TAsmList;value : int64;reg : tregister64);

      begin
        cg.a_load_const_reg(list,OS_32,longint(lo(value)),reg.reglo);
        cg.a_load_const_reg(list,OS_32,longint(hi(value)),reg.reghi);
      end;


    procedure tcg64f32.a_load64_subsetref_reg(list : TAsmList; const sref: tsubsetreference; destreg: tregister64);

      var
        tmpreg: tregister;
        tmpsref: tsubsetreference;
      begin
        if (sref.bitindexreg <> NR_NO) or
           (sref.bitlen <> 64) then
          internalerror(2006082310);
        if (sref.startbit = 0) then
          begin
            a_load64_ref_reg(list,sref.ref,destreg);
            exit;
          end;

        if target_info.endian = endian_big then
          begin
            tmpreg := destreg.reglo;
            destreg.reglo := destreg.reghi;
            destreg.reghi := tmpreg;
          end;
        tmpsref:=sref;
        if (tmpsref.ref.base=destreg.reglo) then
         begin
           tmpreg:=cg.getaddressregister(list);
           cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,tmpsref.ref.base,tmpreg);
           tmpsref.ref.base:=tmpreg;
         end
        else
         if (tmpsref.ref.index=destreg.reglo) then
          begin
            tmpreg:=cg.getaddressregister(list);
            cg.a_load_reg_reg(list,OS_ADDR,OS_ADDR,tmpsref.ref.index,tmpreg);
            tmpsref.ref.index:=tmpreg;
          end;
        tmpsref.bitlen:=32;
        hlcg.a_load_subsetref_reg(list,u32inttype,u32inttype,tmpsref,destreg.reglo);
        inc(tmpsref.ref.offset,4);
        hlcg.a_load_subsetref_reg(list,u32inttype,u32inttype,tmpsref,destreg.reghi);
      end;


    procedure tcg64f32.a_load64_reg_subsetref(list : TAsmList; fromreg: tregister64; const sref: tsubsetreference);

      var
        tmpreg: tregister;
        tmpsref: tsubsetreference;
      begin
        if (sref.bitindexreg <> NR_NO) or
           (sref.bitlen <> 64) then
          internalerror(2006082311);
        if (sref.startbit = 0) then
          begin
            a_load64_reg_ref(list,fromreg,sref.ref);
            exit;
          end;

        if target_info.endian = endian_big then
          begin
            tmpreg:=fromreg.reglo;
            fromreg.reglo:=fromreg.reghi;
            fromreg.reghi:=tmpreg;
          end;
        tmpsref:=sref;
        tmpsref.bitlen:=32;
        hlcg.a_load_reg_subsetref(list,u32inttype,u32inttype,fromreg.reglo,tmpsref);
        inc(tmpsref.ref.offset,4);
        hlcg.a_load_reg_subsetref(list,u32inttype,u32inttype,fromreg.reghi,tmpsref);
      end;


    procedure tcg64f32.a_load64_const_subsetref(list: TAsmlist; a: int64; const sref: tsubsetreference);

      var
        tmpsref: tsubsetreference;
      begin
        if (sref.bitindexreg <> NR_NO) or
           (sref.bitlen <> 64) then
          internalerror(2006082312);
        if target_info.endian = endian_big then
          swap64(a);
        tmpsref := sref;
        tmpsref.bitlen := 32;
        hlcg.a_load_const_subsetref(list,u32inttype,longint(lo(a)),tmpsref);
        inc(tmpsref.ref.offset,4);
        hlcg.a_load_const_subsetref(list,u32inttype,longint(hi(a)),tmpsref);
      end;


    procedure tcg64f32.a_load64_subsetref_subsetref(list: TAsmlist; const fromsref, tosref: tsubsetreference);

      var
        tmpreg64 : tregister64;
      begin
        tmpreg64.reglo:=cg.getintregister(list,OS_32);
        tmpreg64.reghi:=cg.getintregister(list,OS_32);
        a_load64_subsetref_reg(list,fromsref,tmpreg64);
        a_load64_reg_subsetref(list,tmpreg64,tosref);
      end;


    procedure tcg64f32.a_load64_subsetref_ref(list : TAsmList; const sref: tsubsetreference; const destref: treference);

      var
        tmpreg64 : tregister64;
      begin
        tmpreg64.reglo:=cg.getintregister(list,OS_32);
        tmpreg64.reghi:=cg.getintregister(list,OS_32);
        a_load64_subsetref_reg(list,sref,tmpreg64);
        a_load64_reg_ref(list,tmpreg64,destref);
      end;


    procedure tcg64f32.a_load64_ref_subsetref(list : TAsmList; const fromref: treference; const sref: tsubsetreference);

      var
        tmpreg64 : tregister64;
      begin
        tmpreg64.reglo:=cg.getintregister(list,OS_32);
        tmpreg64.reghi:=cg.getintregister(list,OS_32);
        a_load64_ref_reg(list,fromref,tmpreg64);
        a_load64_reg_subsetref(list,tmpreg64,sref);
      end;


    procedure tcg64f32.a_load64_loc_reg(list : TAsmList;const l : tlocation;reg : tregister64);

      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_load64_ref_reg(list,l.reference,reg);
          LOC_REGISTER,LOC_CREGISTER:
            a_load64_reg_reg(list,l.register64,reg);
          LOC_CONSTANT :
            a_load64_const_reg(list,l.value64,reg);
          LOC_SUBSETREF, LOC_CSUBSETREF:
            a_load64_subsetref_reg(list,l.sref,reg);
          else
            internalerror(200112292);
        end;
      end;


    procedure tcg64f32.a_load64_loc_ref(list : TAsmList;const l : tlocation;const ref : treference);
      begin
        case l.loc of
          LOC_REGISTER,LOC_CREGISTER:
            a_load64_reg_ref(list,l.register64,ref);
          LOC_CONSTANT :
            a_load64_const_ref(list,l.value64,ref);
          LOC_SUBSETREF, LOC_CSUBSETREF:
            a_load64_subsetref_ref(list,l.sref,ref);
          else
            internalerror(200203288);
        end;
      end;


    procedure tcg64f32.a_load64_const_loc(list : TAsmList;value : int64;const l : tlocation);

      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_load64_const_ref(list,value,l.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_load64_const_reg(list,value,l.register64);
          LOC_SUBSETREF, LOC_CSUBSETREF:
            a_load64_const_subsetref(list,value,l.sref);
          else
            internalerror(200112293);
        end;
      end;


    procedure tcg64f32.a_load64_reg_loc(list : TAsmList;reg : tregister64;const l : tlocation);

      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_load64_reg_ref(list,reg,l.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_load64_reg_reg(list,reg,l.register64);
          LOC_SUBSETREF, LOC_CSUBSETREF:
            a_load64_reg_subsetref(list,reg,l.sref);
          LOC_MMREGISTER, LOC_CMMREGISTER:
            a_loadmm_intreg64_reg(list,l.size,reg,l.register);
          else
            internalerror(200112294);
        end;
      end;


    procedure tcg64f32.a_load64high_reg_ref(list : TAsmList;reg : tregister;const ref : treference);
      var
        tmpref: treference;
      begin
        if target_info.endian = endian_big then
          cg.a_load_reg_ref(list,OS_32,OS_32,reg,ref)
        else
          begin
            tmpref := ref;
            inc(tmpref.offset,4);
            cg.a_load_reg_ref(list,OS_32,OS_32,reg,tmpref)
          end;
      end;

    procedure tcg64f32.a_load64low_reg_ref(list : TAsmList;reg : tregister;const ref : treference);
      var
        tmpref: treference;
      begin
        if target_info.endian = endian_little then
          cg.a_load_reg_ref(list,OS_32,OS_32,reg,ref)
        else
          begin
            tmpref := ref;
            inc(tmpref.offset,4);
            cg.a_load_reg_ref(list,OS_32,OS_32,reg,tmpref)
          end;
      end;


    procedure tcg64f32.a_load64high_ref_reg(list : TAsmList;const ref : treference;reg : tregister);
      var
        tmpref: treference;
      begin
        if target_info.endian = endian_big then
          cg.a_load_ref_reg(list,OS_32,OS_32,ref,reg)
        else
          begin
            tmpref := ref;
            inc(tmpref.offset,4);
            cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg)
          end;
      end;


    procedure tcg64f32.a_load64low_ref_reg(list : TAsmList;const ref : treference;reg : tregister);
      var
        tmpref: treference;
      begin
        if target_info.endian = endian_little then
          cg.a_load_ref_reg(list,OS_32,OS_32,ref,reg)
        else
          begin
            tmpref := ref;
            inc(tmpref.offset,4);
            cg.a_load_ref_reg(list,OS_32,OS_32,tmpref,reg)
          end;
      end;


    procedure tcg64f32.a_load64low_loc_reg(list : TAsmList;const l : tlocation;reg : tregister);
      begin
        case l.loc of
          LOC_REFERENCE,
          LOC_CREFERENCE :
            a_load64low_ref_reg(list,l.reference,reg);
          LOC_REGISTER,
          LOC_CREGISTER :
            cg.a_load_reg_reg(list,OS_32,OS_32,l.register64.reglo,reg);
          LOC_CONSTANT :
            cg.a_load_const_reg(list,OS_32,longint(lo(l.value64)),reg);
          else
            internalerror(200203244);
        end;
      end;


    procedure tcg64f32.a_load64high_loc_reg(list : TAsmList;const l : tlocation;reg : tregister);
      begin
        case l.loc of
          LOC_REFERENCE,
          LOC_CREFERENCE :
            a_load64high_ref_reg(list,l.reference,reg);
          LOC_REGISTER,
          LOC_CREGISTER :
            cg.a_load_reg_reg(list,OS_32,OS_32,l.register64.reghi,reg);
          LOC_CONSTANT :
            cg.a_load_const_reg(list,OS_32,longint(hi(l.value64)),reg);
          else
            internalerror(200203244);
        end;
      end;


    procedure tcg64f32.a_op64_const_loc(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const l: tlocation);
      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op64_const_ref(list,op,size,value,l.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_op64_const_reg(list,op,size,value,l.register64);
          else
            internalerror(200203292);
        end;
      end;


    procedure tcg64f32.a_op64_reg_loc(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64;const l : tlocation);
      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op64_reg_ref(list,op,size,reg,l.reference);
          LOC_REGISTER,LOC_CREGISTER:
            a_op64_reg_reg(list,op,size,reg,l.register64);
          else
            internalerror(2002032422);
        end;
      end;



    procedure tcg64f32.a_op64_loc_reg(list : TAsmList;op:TOpCG;size : tcgsize;const l : tlocation;reg : tregister64);
      begin
        case l.loc of
          LOC_REFERENCE, LOC_CREFERENCE:
            a_op64_ref_reg(list,op,size,l.reference,reg);
          LOC_REGISTER,LOC_CREGISTER:
            a_op64_reg_reg(list,op,size,l.register64,reg);
          LOC_CONSTANT :
            a_op64_const_reg(list,op,size,l.value64,reg);
          else
            internalerror(200203242);
        end;
      end;


    procedure tcg64f32.a_op64_ref_reg(list : TAsmList;op:TOpCG;size : tcgsize;const ref : treference;reg : tregister64);
      var
        tempreg: tregister64;
      begin
        tempreg.reghi:=cg.getintregister(list,OS_32);
        tempreg.reglo:=cg.getintregister(list,OS_32);
        a_load64_ref_reg(list,ref,tempreg);
        a_op64_reg_reg(list,op,size,tempreg,reg);
      end;


    procedure tcg64f32.a_op64_reg_ref(list : TAsmList;op:TOpCG;size : tcgsize;reg : tregister64; const ref: treference);
      var
        tempreg: tregister64;
      begin
        tempreg.reghi:=cg.getintregister(list,OS_32);
        tempreg.reglo:=cg.getintregister(list,OS_32);
        a_load64_ref_reg(list,ref,tempreg);
        a_op64_reg_reg(list,op,size,reg,tempreg);
        a_load64_reg_ref(list,tempreg,ref);
      end;


    procedure tcg64f32.a_op64_const_ref(list : TAsmList;op:TOpCG;size : tcgsize;value : int64;const ref : treference);
      var
        tempreg: tregister64;
      begin
        tempreg.reghi:=cg.getintregister(list,OS_32);
        tempreg.reglo:=cg.getintregister(list,OS_32);
        a_load64_ref_reg(list,ref,tempreg);
        a_op64_const_reg(list,op,size,value,tempreg);
        a_load64_reg_ref(list,tempreg,ref);
      end;


    procedure tcg64f32.a_load64_reg_cgpara(list : TAsmList;reg : tregister64;const paraloc : tcgpara);
      var
        tmplochi,tmploclo: tcgpara;
      begin
        tmploclo.init;
        tmplochi.init;
        splitparaloc64(paraloc,tmploclo,tmplochi);
<<<<<<< HEAD
        if target_info.endian=endian_big then
          begin
            { Keep this order of first lo before hi to have
              the correct push order for m68k }
            cg.a_load_reg_cgpara(list,OS_32,reg.reglo,tmploclo);
            cg.a_load_reg_cgpara(list,OS_32,reg.reghi,tmplochi);
          end
        else
          begin
            { Keep this order of first hi before lo to have
              the correct push order for i386 }
            cg.a_load_reg_cgpara(list,OS_32,reg.reghi,tmplochi);
            cg.a_load_reg_cgpara(list,OS_32,reg.reglo,tmploclo);
          end;
=======
        { Keep this order of first hi before lo to have
          the correct push order for i386 }
        cg.a_load_reg_cgpara(list,OS_32,reg.reghi,tmplochi);
        cg.a_load_reg_cgpara(list,OS_32,reg.reglo,tmploclo);
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
        tmploclo.done;
        tmplochi.done;
      end;


    procedure tcg64f32.a_load64_const_cgpara(list : TAsmList;value : int64;const paraloc : tcgpara);
      var
        tmplochi,tmploclo: tcgpara;
      begin
        tmploclo.init;
        tmplochi.init;
        splitparaloc64(paraloc,tmploclo,tmplochi);
<<<<<<< HEAD
        if target_info.endian=endian_big then
          begin
            { Keep this order of first lo before hi to have
              the correct push order for m68k }
            cg.a_load_const_cgpara(list,OS_32,longint(lo(value)),tmploclo);
            cg.a_load_const_cgpara(list,OS_32,longint(hi(value)),tmplochi);
          end
        else
          begin
            { Keep this order of first hi before lo to have
              the correct push order for i386 }
            cg.a_load_const_cgpara(list,OS_32,longint(hi(value)),tmplochi);
            cg.a_load_const_cgpara(list,OS_32,longint(lo(value)),tmploclo);
          end;
=======
        { Keep this order of first hi before lo to have
          the correct push order for i386 }
        cg.a_load_const_cgpara(list,OS_32,aint(hi(value)),tmplochi);
        cg.a_load_const_cgpara(list,OS_32,aint(lo(value)),tmploclo);
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
        tmploclo.done;
        tmplochi.done;
      end;


    procedure tcg64f32.a_load64_ref_cgpara(list : TAsmList;const r : treference;const paraloc : tcgpara);
      var
        tmprefhi,tmpreflo : treference;
        tmploclo,tmplochi : tcgpara;
      begin
        tmploclo.init;
        tmplochi.init;
        splitparaloc64(paraloc,tmploclo,tmplochi);
        tmprefhi:=r;
        tmpreflo:=r;
        if target_info.endian=endian_big then
          begin
            { Keep this order of first lo before hi to have
              the correct push order for m68k }
            inc(tmpreflo.offset,4);
            cg.a_load_ref_cgpara(list,OS_32,tmpreflo,tmploclo);
            cg.a_load_ref_cgpara(list,OS_32,tmprefhi,tmplochi);
          end
        else
<<<<<<< HEAD
          begin
            { Keep this order of first hi before lo to have
              the correct push order for i386 }
            inc(tmprefhi.offset,4);
            cg.a_load_ref_cgpara(list,OS_32,tmprefhi,tmplochi);
            cg.a_load_ref_cgpara(list,OS_32,tmpreflo,tmploclo);
          end;
=======
          inc(tmprefhi.offset,4);
        { Keep this order of first hi before lo to have
          the correct push order for i386 }
        cg.a_load_ref_cgpara(list,OS_32,tmprefhi,tmplochi);
        cg.a_load_ref_cgpara(list,OS_32,tmpreflo,tmploclo);
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
        tmploclo.done;
        tmplochi.done;
      end;


    procedure tcg64f32.a_load64_loc_cgpara(list : TAsmList;const l:tlocation;const paraloc : tcgpara);
      begin
        case l.loc of
          LOC_REGISTER,
          LOC_CREGISTER :
            a_load64_reg_cgpara(list,l.register64,paraloc);
          LOC_CONSTANT :
            a_load64_const_cgpara(list,l.value64,paraloc);
          LOC_CREFERENCE,
          LOC_REFERENCE :
            a_load64_ref_cgpara(list,l.reference,paraloc);
          else
            internalerror(200203287);
        end;
      end;


    procedure tcg64f32.a_loadmm_intreg64_reg(list: TAsmList; mmsize: tcgsize; intreg: tregister64; mmreg: tregister);
      var
        tmpref: treference;
      begin
        if (tcgsize2size[mmsize]<>8) then
          internalerror(2009112501);
        tg.gettemp(list,8,8,tt_normal,tmpref);
        a_load64_reg_ref(list,intreg,tmpref);
        cg.a_loadmm_ref_reg(list,mmsize,mmsize,tmpref,mmreg,mms_movescalar);
        tg.ungettemp(list,tmpref);
      end;


    procedure tcg64f32.a_loadmm_reg_intreg64(list: TAsmList; mmsize: tcgsize; mmreg: tregister; intreg: tregister64);
      var
        tmpref: treference;
      begin
        if (tcgsize2size[mmsize]<>8) then
          internalerror(2009112502);
        tg.gettemp(list,8,8,tt_normal,tmpref);
        cg.a_loadmm_reg_ref(list,mmsize,mmsize,mmreg,tmpref,mms_movescalar);
        a_load64_ref_reg(list,tmpref,intreg);
        tg.ungettemp(list,tmpref);
      end;


    procedure tcg64f32.g_rangecheck64(list : TAsmList;const l:tlocation;fromdef,todef:tdef);

      var
        neglabel,
        poslabel,
        endlabel: tasmlabel;
        hreg   : tregister;
        hdef   :  torddef;
        opsize   : tcgsize;
        from_signed,to_signed: boolean;
        temploc : tlocation;

      begin
         from_signed := is_signed(fromdef);
         to_signed := is_signed(todef);

         if not is_64bit(todef) then
           begin
             { get the high dword in a register }
             if l.loc in [LOC_REGISTER,LOC_CREGISTER] then
               begin
                 hreg := l.register64.reghi;
               end
             else
               begin
                 hreg:=cg.getintregister(list,OS_32);
                 a_load64high_ref_reg(list,l.reference,hreg);
               end;
             current_asmdata.getjumplabel(poslabel);

             { check high dword, must be 0 (for positive numbers) }
             cg.a_cmp_const_reg_label(list,OS_32,OC_EQ,0,hreg,poslabel);

             { It can also be $ffffffff, but only for negative numbers }
             if from_signed and to_signed then
               begin
                 current_asmdata.getjumplabel(neglabel);
                 cg.a_cmp_const_reg_label(list,OS_32,OC_EQ,-1,hreg,neglabel);
               end
             else
               { we do not have dynamic dfa, so avoid a warning below about the unused
                 neglabel }
               neglabel:=nil;
             { For all other values we have a range check error }
             cg.a_call_name(list,'fpc_rangeerror',false);

             { if the high dword = 0, the low dword can be considered a }
             { simple cardinal                                          }
             cg.a_label(list,poslabel);
             hdef:=corddef.create(u32bit,0,$ffffffff,false);

             location_copy(temploc,l);
             temploc.size:=OS_32;

             if (temploc.loc in [LOC_REFERENCE,LOC_CREFERENCE]) and
                (target_info.endian = endian_big) then
               begin
                 inc(temploc.reference.offset,4);
                 temploc.reference.alignment:=newalignment(temploc.reference.alignment,4);
               end;

             hlcg.g_rangecheck(list,temploc,hdef,todef);
             hdef.free;

             if from_signed and to_signed then
               begin
                 current_asmdata.getjumplabel(endlabel);
                 cg.a_jmp_always(list,endlabel);
                 { if the high dword = $ffffffff, then the low dword (when }
                 { considered as a longint) must be < 0                    }
                 cg.a_label(list,neglabel);
                 if l.loc in [LOC_REGISTER,LOC_CREGISTER] then
                   begin
                     hreg := l.register64.reglo;
                   end
                 else
                   begin
                     hreg:=cg.getintregister(list,OS_32);
                     a_load64low_ref_reg(list,l.reference,hreg);
                   end;
                 { get a new neglabel (JM) }
                 current_asmdata.getjumplabel(neglabel);
                 cg.a_cmp_const_reg_label(list,OS_32,OC_LT,0,hreg,neglabel);

                 cg.a_call_name(list,'fpc_rangeerror',false);

                 { if we get here, the 64bit value lies between }
                 { longint($80000000) and -1 (JM)               }
                 cg.a_label(list,neglabel);
                 hdef:=corddef.create(s32bit,int64(longint($80000000)),int64(-1),false);
                 location_copy(temploc,l);
                 temploc.size:=OS_32;
                 hlcg.g_rangecheck(list,temploc,hdef,todef);
                 hdef.free;
                 cg.a_label(list,endlabel);
               end;
           end
         else
           { todef = 64bit int }
           { no 64bit subranges supported, so only a small check is necessary }

           { if both are signed or both are unsigned, no problem! }
           if (from_signed xor to_signed) and
              { also not if the fromdef is unsigned and < 64bit, since that will }
              { always fit in a 64bit int (todef is 64bit)                       }
              (from_signed or
               (torddef(fromdef).ordtype = u64bit)) then
             begin
               { in all cases, there is only a problem if the higest bit is set }
               if l.loc in [LOC_REGISTER,LOC_CREGISTER] then
                 begin
                   if is_64bit(fromdef) then
                     begin
                       hreg := l.register64.reghi;
                       opsize := OS_32;
                     end
                   else
                     begin
                       hreg := l.register;
                       opsize := def_cgsize(fromdef);
                     end;
                 end
               else
                 begin
                   hreg:=cg.getintregister(list,OS_32);
                   opsize:=OS_32;

                   if l.size in [OS_64,OS_S64] then
                     a_load64high_ref_reg(list,l.reference,hreg)
                   else
                     cg.a_load_ref_reg(list,l.size,OS_32,l.reference,hreg);
                 end;
               current_asmdata.getjumplabel(poslabel);
               cg.a_cmp_const_reg_label(list,opsize,OC_GTE,0,hreg,poslabel);

               cg.a_call_name(list,'fpc_rangeerror',false);
               cg.a_label(list,poslabel);
             end;
      end;


    function tcg64f32.optimize64_op_const_reg(list: TAsmList; var op: topcg; var a : int64; var reg: tregister64): boolean;
      var
        lowvalue, highvalue : longint;
        hreg: tregister;
      begin
        lowvalue := longint(a);
        highvalue:= longint(a shr 32);
        { assume it will be optimized out }
        optimize64_op_const_reg := true;
        case op of
        OP_ADD:
           begin
             if a = 0 then
                exit;
           end;
        OP_AND:
           begin
              if lowvalue <> -1 then
                cg.a_op_const_reg(list,op,OS_32,lowvalue,reg.reglo);
              if highvalue <> -1 then
                cg.a_op_const_reg(list,op,OS_32,highvalue,reg.reghi);
              { already emitted correctly }
              exit;
           end;
        OP_OR:
           begin
              if lowvalue <> 0 then
                cg.a_op_const_reg(list,op,OS_32,lowvalue,reg.reglo);
              if highvalue <> 0 then
                cg.a_op_const_reg(list,op,OS_32,highvalue,reg.reghi);
              { already emitted correctly }
              exit;
           end;
        OP_SUB:
           begin
             if a = 0 then
                exit;
           end;
        OP_XOR:
           begin
           end;
        OP_SHL:
           begin
             if a = 0 then
                 exit;
             { simply clear low-register
               and shift the rest and swap
               registers.
             }
             if (a > 31) then
               begin
                 cg.a_load_const_reg(list,OS_32,0,reg.reglo);
                 cg.a_op_const_reg(list,OP_SHL,OS_32,a mod 32,reg.reghi);
                 { swap the registers }
                 hreg := reg.reghi;
                 reg.reghi := reg.reglo;
                 reg.reglo := hreg;
                 exit;
               end;
           end;
        OP_SHR:
           begin
             if a = 0 then exit;
             { simply clear high-register
               and shift the rest and swap
               registers.
             }
             if (a > 31) then
               begin
                 cg.a_load_const_reg(list,OS_32,0,reg.reghi);
                 cg.a_op_const_reg(list,OP_SHL,OS_32,a mod 32,reg.reglo);
                 { swap the registers }
                 hreg := reg.reghi;
                 reg.reghi := reg.reglo;
                 reg.reglo := hreg;
                 exit;
               end;
           end;
        OP_IMUL,OP_MUL:
           begin
             if a = 1 then exit;
           end;
        OP_IDIV,OP_DIV:
            begin
             if a = 1 then exit;
            end;
        else
           internalerror(20020817);
        end;
        optimize64_op_const_reg := false;
      end;

end.
