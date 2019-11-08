{
    Copyright (c) 1998-2011 by Florian Klaempfl, Jonas Maebe

    Generates code/nodes for typed constant declarations

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
unit ngtcon;

{$i fpcdefs.inc}

interface

    uses
      globtype,cclasses,constexp,
      aasmbase,aasmdata,aasmtai,aasmcnst,
      node,nbas,
      symconst, symtype, symbase, symdef,symsym;


    type
      ttypedconstbuilder = class
       protected
        current_old_block_type : tblock_type;
        tcsym: tstaticvarsym;

        { this procedure reads typed constants }
        procedure read_typed_const_data(def:tdef);

        procedure parse_orddef(def: torddef);
        procedure parse_floatdef(def: tfloatdef);
        procedure parse_classrefdef(def: tclassrefdef);
        procedure parse_pointerdef(def: tpointerdef);
        procedure parse_setdef(def: tsetdef);
        procedure parse_enumdef(def: tenumdef);
        procedure parse_stringdef(def: tstringdef);
        procedure parse_arraydef(def:tarraydef);virtual;abstract;
        procedure parse_procvardef(def:tprocvardef);virtual;abstract;
        procedure parse_recorddef(def:trecorddef);virtual;abstract;
        procedure parse_objectdef(def:tobjectdef);virtual;abstract;

        procedure tc_emit_orddef(def: torddef; var node: tnode);virtual;abstract;
        procedure tc_emit_floatdef(def: tfloatdef; var node: tnode);virtual;abstract;
        procedure tc_emit_classrefdef(def: tclassrefdef; var node: tnode);virtual;abstract;
        procedure tc_emit_pointerdef(def: tpointerdef; var node: tnode);virtual;abstract;
        procedure tc_emit_setdef(def: tsetdef; var node: tnode);virtual;abstract;
        procedure tc_emit_enumdef(def: tenumdef; var node: tnode);virtual;abstract;
        procedure tc_emit_stringdef(def: tstringdef; var node: tnode);virtual;abstract;
       public
        constructor create(sym: tstaticvarsym);
      end;
      ttypedconstbuilderclass = class of ttypedconstbuilder;


      { should be changed into nested type of tasmlisttypedconstbuilder when
        possible }
      tbitpackedval = record
        curval, nextval: aword;
        curbitoffset: smallint;
        packedbitsize: byte;
      end;

      tasmlisttypedconstbuilder = class(ttypedconstbuilder)
       private
        fsym: tstaticvarsym;
        curoffset: asizeint;

        function parse_single_packed_const(def: tdef; var bp: tbitpackedval): boolean;
        procedure flush_packed_value(var bp: tbitpackedval);
       protected
        ftcb: ttai_typedconstbuilder;
        fdatalist: tasmlist;

        procedure parse_packed_array_def(def: tarraydef);
        procedure parse_arraydef(def:tarraydef);override;
        procedure parse_procvardef(def:tprocvardef);override;
        procedure parse_recorddef(def:trecorddef);override;
        procedure parse_objectdef(def:tobjectdef);override;

        procedure tc_emit_orddef(def: torddef; var node: tnode);override;
        procedure tc_emit_floatdef(def: tfloatdef; var node: tnode); override;
        procedure tc_emit_classrefdef(def: tclassrefdef; var node: tnode);override;
        procedure tc_emit_pointerdef(def: tpointerdef; var node: tnode);override;
        procedure tc_emit_setdef(def: tsetdef; var node: tnode);override;
        procedure tc_emit_enumdef(def: tenumdef; var node: tnode);override;
        procedure tc_emit_stringdef(def: tstringdef; var node: tnode);override;
       public
        constructor create(sym: tstaticvarsym);virtual;
        destructor Destroy; override;
        procedure parse_into_asmlist;
        { the asmlist containing the definition of the parsed entity and another
          one containing the data generated for that same entity (e.g. the
          string data referenced by an ansistring constant) }
        procedure get_final_asmlists(out reslist, datalist: tasmlist);
      end;
      tasmlisttypedconstbuilderclass = class of tasmlisttypedconstbuilder;

      tnodetreetypedconstbuilder = class(ttypedconstbuilder)
       protected
        resultblock: tblocknode;
        statmnt: tstatementnode;

        { when parsing a record, the base nade becomes a loadnode of the record,
          etc. }
        basenode: tnode;

        procedure parse_arraydef(def:tarraydef);override;
        procedure parse_procvardef(def:tprocvardef);override;
        procedure parse_recorddef(def:trecorddef);override;
        procedure parse_objectdef(def:tobjectdef);override;

        procedure tc_emit_orddef(def: torddef; var node: tnode);override;
        procedure tc_emit_floatdef(def: tfloatdef; var node: tnode); override;
        procedure tc_emit_classrefdef(def: tclassrefdef; var node: tnode);override;
        procedure tc_emit_pointerdef(def: tpointerdef; var node: tnode);override;
        procedure tc_emit_setdef(def: tsetdef; var node: tnode);override;
        procedure tc_emit_enumdef(def: tenumdef; var node: tnode);override;
        procedure tc_emit_stringdef(def: tstringdef; var node: tnode);override;
       public
        constructor create(sym: tstaticvarsym; previnit: tnode);virtual;
        destructor destroy;override;
        function parse_into_nodetree: tnode;
      end;
      tnodetreetypedconstbuilderclass = class of tnodetreetypedconstbuilder;

   var
     ctypedconstbuilder: ttypedconstbuilderclass;

implementation

uses
   SysUtils,
   systems,tokens,verbose,compinnr,
   cutils,globals,widestr,scanner,
   symtable,
   defutil,defcmp,
   { pass 1 }
   htypechk,procinfo,
   nmem,ncnv,ninl,ncon,nld,
   { parser specific stuff }
   pbase,pexpr,
   { codegen }
   cpuinfo,cgbase,
   wpobase
   ;

{$maxfpuregisters 0}

function get_next_varsym(def: tabstractrecorddef; const SymList:TFPHashObjectList; var symidx:longint):tsym;inline;
  begin
    while symidx<SymList.Count do
      begin
        result:=tsym(def.symtable.SymList[symidx]);
        inc(symidx);
        if (result.typ=fieldvarsym) and
           not(sp_static in result.symoptions) then
          exit;
      end;
    result:=nil;
  end;


{*****************************************************************************
                             read typed const
*****************************************************************************}

      procedure ttypedconstbuilder.parse_orddef(def:torddef);
        var
          n : tnode;
        begin
           n:=comp_expr([ef_accept_equal]);
           { for C-style booleans, true=-1 and false=0) }
           if is_cbool(def) then
             inserttypeconv(n,def);
           tc_emit_orddef(def,n);
           n.free;
        end;

      procedure ttypedconstbuilder.parse_floatdef(def:tfloatdef);
        var
          n : tnode;
        begin
          n:=comp_expr([ef_accept_equal]);
          tc_emit_floatdef(def,n);
          n.free;
        end;

      procedure ttypedconstbuilder.parse_classrefdef(def:tclassrefdef);
        var
          n : tnode;
        begin
          n:=comp_expr([ef_accept_equal]);
          case n.nodetype of
            loadvmtaddrn:
              begin
                { update wpo info }
                if not assigned(current_procinfo) or
                   (po_inline in current_procinfo.procdef.procoptions) or
                   wpoinfomanager.symbol_live(current_procinfo.procdef.mangledname) then
                  tobjectdef(tclassrefdef(n.resultdef).pointeddef).register_maybe_created_object_type;
              end;
            else
              ;
          end;
          tc_emit_classrefdef(def,n);
          n.free;
        end;

      procedure ttypedconstbuilder.parse_pointerdef(def:tpointerdef);
        var
          p: tnode;
        begin
          p:=comp_expr([ef_accept_equal]);
          tc_emit_pointerdef(def,p);
          p.free;
        end;

      procedure ttypedconstbuilder.parse_setdef(def:tsetdef);
        var
          p : tnode;
        begin
          p:=comp_expr([ef_accept_equal]);
          tc_emit_setdef(def,p);
          p.free;
        end;

      procedure ttypedconstbuilder.parse_enumdef(def:tenumdef);
        var
          p : tnode;
        begin
          p:=comp_expr([ef_accept_equal]);
          tc_emit_enumdef(def,p);
          p.free;
        end;

      procedure ttypedconstbuilder.parse_stringdef(def:tstringdef);
        var
          n : tnode;
        begin
          n:=comp_expr([ef_accept_equal]);
          tc_emit_stringdef(def,n);
          n.free;
        end;

    { ttypedconstbuilder }

    procedure ttypedconstbuilder.read_typed_const_data(def:tdef);
      var
       prev_old_block_type,
       old_block_type: tblock_type;
      begin
        old_block_type:=block_type;
        prev_old_block_type:=current_old_block_type;
        current_old_block_type:=old_block_type;
        block_type:=bt_const;
        case def.typ of
          orddef :
            parse_orddef(torddef(def));
          floatdef :
            parse_floatdef(tfloatdef(def));
          classrefdef :
            parse_classrefdef(tclassrefdef(def));
          pointerdef :
            parse_pointerdef(tpointerdef(def));
          setdef :
            parse_setdef(tsetdef(def));
          enumdef :
            parse_enumdef(tenumdef(def));
          stringdef :
            parse_stringdef(tstringdef(def));
          arraydef :
            parse_arraydef(tarraydef(def));
          procvardef:
            parse_procvardef(tprocvardef(def));
          recorddef:
            parse_recorddef(trecorddef(def));
          objectdef:
            parse_objectdef(tobjectdef(def));
          errordef:
            begin
               { try to consume something useful }
               if token=_LKLAMMER then
                 consume_all_until(_RKLAMMER)
               else
                 consume_all_until(_SEMICOLON);
            end;
          else
            Message(parser_e_type_const_not_possible);
        end;
        block_type:=old_block_type;
        current_old_block_type:=prev_old_block_type;
      end;


    constructor ttypedconstbuilder.create(sym: tstaticvarsym);
      begin
        tcsym:=sym;
      end;


{*****************************************************************************
                          Bitpacked value helpers
*****************************************************************************}

    procedure initbitpackval(out bp: tbitpackedval; packedbitsize: byte);
      begin
        bp.curval:=0;
        bp.nextval:=0;
        bp.curbitoffset:=0;
        bp.packedbitsize:=packedbitsize;
      end;


{$push}
{$r-}
{$q-}
    { (values between quotes below refer to fields of bp; fields not         }
    {  mentioned are unused by this routine)                                 }
    { bitpacks "value" as bitpacked value of bitsize "packedbitsize" into    }
    { "curval", which has already been filled up to "curbitoffset", and      }
    { stores the spillover if any into "nextval". It also updates            }
    { curbitoffset to reflect how many bits of currval are now used (can be  }
    { > AIntBits in case of spillover)                                       }
    procedure bitpackval(value: aword; var bp: tbitpackedval);
      var
        shiftcount: longint;
      begin
        if (target_info.endian=endian_big) then
          begin
            { bitpacked format: left-aligned (i.e., "big endian bitness") }
            { work around broken x86 shifting }
            if (AIntBits<>bp.packedbitsize) and
               (bp.curbitoffset<AIntBits) then
              bp.curval:=bp.curval or ((value shl (AIntBits-bp.packedbitsize)) shr bp.curbitoffset);
            shiftcount:=((AIntBits-bp.packedbitsize)-bp.curbitoffset);
            { carry-over to the next element? }
            if (shiftcount<0) then
              begin
                if shiftcount>=AIntBits then
                  bp.nextval:=(value and ((aword(1) shl (-shiftcount))-1)) shl
                              (AIntBits+shiftcount)
                else
                  bp.nextval:=0
              end
          end
        else
          begin
            { bitpacked format: right aligned (i.e., "little endian bitness") }
            { work around broken x86 shifting }
            if bp.curbitoffset<AIntBits then
              bp.curval:=bp.curval or (value shl bp.curbitoffset);
            { carry-over to the next element? }
            if (bp.curbitoffset+bp.packedbitsize>AIntBits) then
              if bp.curbitoffset>0 then
                bp.nextval:=value shr (AIntBits-bp.curbitoffset)
              else
                bp.nextval:=0;
          end;
        inc(bp.curbitoffset,bp.packedbitsize);
      end;

    procedure tasmlisttypedconstbuilder.flush_packed_value(var bp: tbitpackedval);
      var
        bitstowrite: longint;
        writeval : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
      begin
        if (bp.curbitoffset < AIntBits) then
          begin
            { forced flush -> write multiple of a byte }
            bitstowrite:=align(bp.curbitoffset,8);
            bp.curbitoffset:=0;
          end
        else
          begin
            bitstowrite:=AIntBits;
            dec(bp.curbitoffset,AIntBits);
          end;
        while (bitstowrite>=8) do
          begin
            if (target_info.endian=endian_little) then
              begin
                { write lowest byte }
                writeval:=byte(bp.curval);
                bp.curval:=bp.curval shr 8;
              end
            else
              begin
                { write highest byte }
                writeval:=bp.curval shr (AIntBits-8);
{$push}{$r-,q-}
                bp.curval:=bp.curval shl 8;
{$pop}
              end;
            ftcb.emit_tai(tai_const.create_8bit(writeval),u8inttype);
            dec(bitstowrite,8);
          end;
        bp.curval:=bp.nextval;
        bp.nextval:=0;
      end;

    {$pop}


    { parses a packed array constant }
    procedure tasmlisttypedconstbuilder.parse_packed_array_def(def: tarraydef);
      var
        i  : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
        bp : tbitpackedval;
      begin
        if not(def.elementdef.typ in [orddef,enumdef]) then
          internalerror(2007022010);
        ftcb.maybe_begin_aggregate(def);
        { begin of the array }
        consume(_LKLAMMER);
        initbitpackval(bp,def.elepackedbitsize);
        i:=def.lowrange;
        { can't use for-loop, fails when cross-compiling from }
        { 32 to 64 bit because i is then 64 bit               }
        while (i<def.highrange) do
          begin
            { get next item of the packed array }
            if not parse_single_packed_const(def.elementdef,bp) then
              exit;
            consume(_COMMA);
            inc(i);
          end;
        { final item }
        if not parse_single_packed_const(def.elementdef,bp) then
          exit;
        { flush final incomplete value if necessary }
        if (bp.curbitoffset <> 0) then
          flush_packed_value(bp);
        ftcb.maybe_end_aggregate(def);
        consume(_RKLAMMER);
      end;



    constructor tasmlisttypedconstbuilder.create(sym: tstaticvarsym);
      begin
        inherited;
        fsym:=sym;
        ftcb:=ctai_typedconstbuilder.create([tcalo_make_dead_strippable,tcalo_apply_constalign]);
        fdatalist:=tasmlist.create;
        curoffset:=0;
      end;


    destructor tasmlisttypedconstbuilder.Destroy;
      begin
        fdatalist.free;
        ftcb.free;
        inherited Destroy;
      end;


    procedure tasmlisttypedconstbuilder.tc_emit_stringdef(def: tstringdef; var node: tnode);
      var
        strlength : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
        strval    : pchar;
        ll        : tasmlabofs;
        ca        : pchar;
        winlike   : boolean;
        hsym      : tconstsym;
      begin
        strval:='';
        { load strval and strlength of the constant tree }
        if (node.nodetype=stringconstn) or is_wide_or_unicode_string(def) or is_constwidecharnode(node) or
          ((node.nodetype=typen) and is_interfacecorba(ttypenode(node).typedef)) or
          is_constcharnode(node) then
          begin
            { convert to the expected string type so that
              for widestrings strval is a pcompilerwidestring }
            inserttypeconv(node,def);
            if (not codegenerror) and
               (node.nodetype=stringconstn) then
              begin
                strlength:=tstringconstnode(node).len;
                strval:=tstringconstnode(node).value_str;
                { the def may have changed from e.g. RawByteString to
                  AnsiString(CP_ACP) }
                if node.resultdef.typ=stringdef then
                  def:=tstringdef(node.resultdef)
                else
                  internalerror(2014010501);
              end
            else
              begin
                { an error occurred trying to convert the result to a string }
                strlength:=-1;
                { it's possible that the type conversion could not be
                  evaluated at compile-time }
                if not codegenerror then
                  CGMessage(parser_e_widestring_to_ansi_compile_time);
              end;
          end
        else if is_constresourcestringnode(node) then
          begin
            hsym:=tconstsym(tloadnode(node).symtableentry);
            strval:=pchar(hsym.value.valueptr);
            strlength:=hsym.value.len;
            { Delphi-compatible (mis)feature:
              Link AnsiString constants to their initializing resourcestring,
              enabling them to be (re)translated at runtime.
              Wide/UnicodeString are currently rejected above (with incorrect error message).
              ShortStrings cannot be handled unless another table is built for them;
              considering this acceptable, because Delphi rejects them altogether.
            }
            if (not is_shortstring(def)) and
               ((tcsym.owner.symtablelevel<=main_program_level) or
                (current_old_block_type=bt_const)) then
              begin
                current_asmdata.ResStrInits.Concat(
                  TTCInitItem.Create(tcsym,curoffset,
                  current_asmdata.RefAsmSymbol(make_mangledname('RESSTR',hsym.owner,hsym.name),AT_DATA),charpointertype)
                );
                Include(tcsym.varoptions,vo_force_finalize);
              end;
          end
        else
          begin
            Message(parser_e_illegal_expression);
            strlength:=-1;
          end;
        if strlength>=0 then
          begin
            case def.stringtype of
              st_shortstring:
                begin
                  ftcb.maybe_begin_aggregate(def);
                  if strlength>=def.size then
                   begin
                     message2(parser_w_string_too_long,strpas(strval),tostr(def.size-1));
                     strlength:=def.size-1;
                   end;
                  ftcb.emit_tai(Tai_const.Create_8bit(strlength),cansichartype);
                  { room for the string data + terminating #0 }
                  getmem(ca,def.size);
                  move(strval^,ca^,strlength);
                  { zero-terminate and fill with spaces if size is shorter }
                  fillchar(ca[strlength],def.size-strlength-1,' ');
                  ca[strlength]:=#0;
                  ca[def.size-1]:=#0;
                  ftcb.emit_tai(Tai_string.Create_pchar(ca,def.size-1),carraydef.getreusable(cansichartype,def.size-1));
                  ftcb.maybe_end_aggregate(def);
                end;
              st_ansistring:
                begin
                   { an empty ansi string is nil! }
                   if (strlength=0) then
                     begin
                       ll.lab:=nil;
                       ll.ofs:=0;
                     end
                   else
                     ll:=ftcb.emit_ansistring_const(fdatalist,strval,strlength,def.encoding);
                   ftcb.emit_string_offset(ll,strlength,def.stringtype,false,charpointertype);
                end;
              st_unicodestring,
              st_widestring:
                begin
                   { an empty wide/unicode string is nil! }
                   if (strlength=0) then
                     begin
                       ll.lab:=nil;
                       ll.ofs:=0;
                       winlike:=false;
                     end
                   else
                     begin
                       winlike:=(def.stringtype=st_widestring) and (tf_winlikewidestring in target_info.flags);
                       ll:=ftcb.emit_unicodestring_const(fdatalist,
                              strval,
                              def.encoding,
                              winlike);

                       { Collect Windows widestrings that need initialization at startup.
                         Local initialized vars are excluded because they are initialized
                         at function entry instead. }
                       if winlike and
                          ((tcsym.owner.symtablelevel<=main_program_level) or
                           (current_old_block_type=bt_const)) then
                         begin
                           if ll.ofs<>0 then
                             internalerror(2012051704);
                           current_asmdata.WideInits.Concat(
                              TTCInitItem.Create(tcsym,curoffset,ll.lab,widecharpointertype)
                           );
                           ll.lab:=nil;
                           ll.ofs:=0;
                           Include(tcsym.varoptions,vo_force_finalize);
                         end;
                     end;
                  ftcb.emit_string_offset(ll,strlength,def.stringtype,winlike,widecharpointertype);
                end;
              else
                internalerror(200107081);
            end;
          end;
      end;


    procedure tasmlisttypedconstbuilder.tc_emit_orddef(def: torddef; var node: tnode);
      var
        intvalue: tconstexprint;

      procedure do_error;
        begin
          if is_constnode(node) then
            IncompatibleTypes(node.resultdef, def)
          else if not(parse_generic) then
            Message(parser_e_illegal_expression);
        end;

      begin
        case def.ordtype of
           pasbool1,
           pasbool8,
           bool8bit,
           pasbool16,
           bool16bit,
           pasbool32,
           bool32bit,
           pasbool64,
           bool64bit:
             begin
                if is_constboolnode(node) then
                  begin
                    adaptrange(def,tordconstnode(node).value,rc_default);
                    ftcb.emit_ord_const(tordconstnode(node).value.svalue,def)
                  end
                else
                  do_error;
             end;
           uchar :
             begin
                if is_constwidecharnode(node) then
                  inserttypeconv(node,cansichartype);
                if is_constcharnode(node) or
                  ((m_delphi in current_settings.modeswitches) and
                   is_constwidecharnode(node) and
                   (tordconstnode(node).value <= 255)) then
                  ftcb.emit_ord_const(byte(tordconstnode(node).value.svalue),def)
                else
                  do_error;
             end;
           uwidechar :
             begin
                if is_constcharnode(node) then
                  inserttypeconv(node,cwidechartype);
                if is_constwidecharnode(node) then
                  ftcb.emit_ord_const(word(tordconstnode(node).value.svalue),def)
                else
                  do_error;
             end;
           s8bit,u8bit,
           u16bit,s16bit,
           s32bit,u32bit,
           s64bit,u64bit :
             begin
                if is_constintnode(node) then
                  begin
                    adaptrange(def,tordconstnode(node).value,rc_default);
                    ftcb.emit_ord_const(tordconstnode(node).value.svalue,def);
                  end
                else
                  do_error;
             end;
           scurrency:
             begin
                if is_constintnode(node) then
                  intvalue:=tordconstnode(node).value*10000
                { allow bootstrapping }
                else if is_constrealnode(node) then
                  intvalue:=PInt64(@trealconstnode(node).value_currency)^
                else
                  begin
                    intvalue:=0;
                    IncompatibleTypes(node.resultdef, def);
                  end;
               ftcb.emit_ord_const(intvalue,def);
             end;
           else
             internalerror(200611052);
        end;
      end;


    procedure tasmlisttypedconstbuilder.tc_emit_floatdef(def: tfloatdef; var node: tnode);
      var
        value : bestreal;
      begin
        value:=0.0;
        if is_constrealnode(node) then
          value:=trealconstnode(node).value_real
        else if is_constintnode(node) then
          value:=tordconstnode(node).value
        else if is_constnode(node) then
          IncompatibleTypes(node.resultdef, def)
        else
          Message(parser_e_illegal_expression);

        case def.floattype of
           s32real :
             ftcb.emit_tai(tai_realconst.create_s32real(ts32real(value)),def);
           s64real :
{$ifdef ARM}
             if is_double_hilo_swapped then
               ftcb.emit_tai(tai_realconst.create_s64real_hiloswapped(ts64real(value)),def)
             else
{$endif ARM}
               ftcb.emit_tai(tai_realconst.create_s64real(ts64real(value)),def);
           s80real :
             ftcb.emit_tai(tai_realconst.create_s80real(value,s80floattype.size),def);
           sc80real :
             ftcb.emit_tai(tai_realconst.create_s80real(value,sc80floattype.size),def);
           s64comp :
             { the round is necessary for native compilers where comp isn't a float }
             ftcb.emit_tai(tai_realconst.create_s64compreal(round(value)),def);
           s64currency:
             ftcb.emit_tai(tai_realconst.create_s64compreal(round(value*10000)),def);
           s128real:
             ftcb.emit_tai(tai_realconst.create_s128real(value),def);
        end;
      end;


    procedure tasmlisttypedconstbuilder.tc_emit_classrefdef(def: tclassrefdef; var node: tnode);
      begin
        case node.nodetype of
          loadvmtaddrn:
            begin
              if not def_is_related(tobjectdef(tclassrefdef(node.resultdef).pointeddef),tobjectdef(def.pointeddef)) then
                IncompatibleTypes(node.resultdef, def);
              ftcb.emit_tai(Tai_const.Create_sym(current_asmdata.RefAsmSymbol(Tobjectdef(tclassrefdef(node.resultdef).pointeddef).vmt_mangledname,AT_DATA)),def);
            end;
           niln:
             ftcb.emit_tai(Tai_const.Create_sym(nil),def);
           else if is_constnode(node) then
             IncompatibleTypes(node.resultdef, def)
           else
             Message(parser_e_illegal_expression);
        end;
      end;


    procedure tasmlisttypedconstbuilder.tc_emit_pointerdef(def: tpointerdef; var node: tnode);
      var
        hp        : tnode;
        srsym     : tsym;
        pd        : tprocdef;
        ca        : pchar;
        pw        : pcompilerwidestring;
        i,len     : longint;
        ll        : tasmlabel;
        varalign  : shortint;
        datadef   : tdef;
        datatcb   : ttai_typedconstbuilder;
      begin
        { remove equal typecasts for pointer/nil addresses }
        if (node.nodetype=typeconvn) then
          with Ttypeconvnode(node) do
            if (left.nodetype in [addrn,niln]) and equal_defs(def,node.resultdef) then
              begin
                hp:=left;
                left:=nil;
                node.free;
                node:=hp;
              end;
        { allows horrible ofs(typeof(TButton)^) code !! }
        if (node.nodetype=typeconvn) then
          with Ttypeconvnode(node) do
            if (left.nodetype=addrn) and equal_defs(uinttype,node.resultdef) then
              begin
                hp:=left;
                left:=nil;
                node.free;
                node:=hp;
              end;
        if (node.nodetype=addrn) then
          with Taddrnode(node) do
            if left.nodetype=derefn then
              begin
                hp:=tderefnode(left).left;
                tderefnode(left).left:=nil;
                node.free;
                node:=hp;
             end;
        { const pointer ? }
        if (node.nodetype = pointerconstn) then
          begin
            ftcb.queue_init(def);
            ftcb.queue_typeconvn(ptrsinttype,def);
            {$if sizeof(TConstPtrUInt)=8}
              ftcb.queue_emit_ordconst(int64(tpointerconstnode(node).value),ptrsinttype);
            {$else}
              {$if sizeof(TConstPtrUInt)=4}
                ftcb.queue_emit_ordconst(longint(tpointerconstnode(node).value),ptrsinttype);
              {$else}
                {$if sizeof(TConstPtrUInt)=2}
                  ftcb.queue_emit_ordconst(smallint(tpointerconstnode(node).value),ptrsinttype);
                {$else}
                  {$if sizeof(TConstPtrUInt)=1}
                    ftcb.queue_emit_ordconst(shortint(tpointerconstnode(node).value),ptrsinttype);
                  {$else}
                    internalerror(200404122);
            {$endif} {$endif} {$endif} {$endif}
          end
        { nil pointer ? }
        else if node.nodetype=niln then
          ftcb.emit_tai(Tai_const.Create_sym(nil),def)
        { maybe pchar ? }
        else
          if is_char(def.pointeddef) and
             (node.nodetype<>addrn) then
            begin
              { create a tcb for the string data (it's placed in a separate
                asmlist) }
              ftcb.start_internal_data_builder(fdatalist,sec_rodata_norel,'',datatcb,ll);
              if node.nodetype=stringconstn then
                varalign:=size_2_align(tstringconstnode(node).len)
              else
                varalign:=1;
              varalign:=const_align(varalign);
              { represent the string data as an array }
              if node.nodetype=stringconstn then
                begin
                  len:=tstringconstnode(node).len;
                  { For tp7 the maximum lentgh can be 255 }
                  if (m_tp7 in current_settings.modeswitches) and
                     (len>255) then
                   len:=255;
                  getmem(ca,len+1);
                  move(tstringconstnode(node).value_str^,ca^,len+1);
                  datadef:=carraydef.getreusable(cansichartype,len+1);
                  datatcb.maybe_begin_aggregate(datadef);
                  datatcb.emit_tai(Tai_string.Create_pchar(ca,len+1),datadef);
                  datatcb.maybe_end_aggregate(datadef);
                end
              else if is_constcharnode(node) then
                begin
                  datadef:=carraydef.getreusable(cansichartype,2);
                  datatcb.maybe_begin_aggregate(datadef);
                  datatcb.emit_tai(Tai_string.Create(char(byte(tordconstnode(node).value.svalue))+#0),datadef);
                  datatcb.maybe_end_aggregate(datadef);
                end
              else
                begin
                  IncompatibleTypes(node.resultdef, def);
                  datadef:=carraydef.getreusable(cansichartype,1);
                end;
              ftcb.finish_internal_data_builder(datatcb,ll,datadef,varalign);
              { we now emit the address of the first element of the array
                containing the string data }
              ftcb.queue_init(def);
              { the first element ... }
              ftcb.queue_vecn(datadef,0);
              { ... of the string array }
              ftcb.queue_emit_asmsym(ll,datadef);
            end
        { maybe pwidechar ? }
        else
          if is_widechar(def.pointeddef) and
             (node.nodetype<>addrn) then
            begin
              if (node.nodetype in [stringconstn,ordconstn]) then
                begin
                  { convert to unicodestring stringconstn }
                  inserttypeconv(node,cunicodestringtype);
                  if (node.nodetype=stringconstn) and
                     (tstringconstnode(node).cst_type in [cst_widestring,cst_unicodestring]) then
                   begin
                     { create a tcb for the string data (it's placed in a separate
                       asmlist) }
                     ftcb.start_internal_data_builder(fdatalist,sec_rodata,'',datatcb,ll);
                     datatcb:=ctai_typedconstbuilder.create([tcalo_is_lab,tcalo_make_dead_strippable,tcalo_apply_constalign]);
                     pw:=pcompilerwidestring(tstringconstnode(node).value_str);
                     { include terminating #0 }
                     datadef:=carraydef.getreusable(cwidechartype,tstringconstnode(node).len+1);
                     datatcb.maybe_begin_aggregate(datadef);
                     for i:=0 to tstringconstnode(node).len-1 do
                       datatcb.emit_tai(Tai_const.Create_16bit(pw^.data[i]),cwidechartype);
                     { ending #0 }
                     datatcb.emit_tai(Tai_const.Create_16bit(0),cwidechartype);
                     datatcb.maybe_end_aggregate(datadef);
                     { concat add the string data to the fdatalist }
                     ftcb.finish_internal_data_builder(datatcb,ll,datadef,const_align(sizeof(pint)));
                     { we now emit the address of the first element of the array
                       containing the string data }
                     ftcb.queue_init(def);
                     { the first element ... }
                     ftcb.queue_vecn(datadef,0);
                     { ... of the string array }
                     ftcb.queue_emit_asmsym(ll,datadef);
                   end;
                end
              else
                IncompatibleTypes(node.resultdef, def);
          end
        else
          if (node.nodetype=addrn) or
             is_proc2procvar_load(node,pd) then
            begin
              { insert typeconv }
              inserttypeconv(node,def);
              hp:=node;
              while assigned(hp) and (hp.nodetype in [addrn,typeconvn,subscriptn,vecn]) do
                hp:=tunarynode(hp).left;
              if (hp.nodetype=loadn) then
                begin
                  hp:=node;
                  ftcb.queue_init(def);
                  while assigned(hp) and (hp.nodetype<>loadn) do
                    begin
                       case hp.nodetype of
                         vecn :
                           begin
                             if (is_constintnode(tvecnode(hp).right) or
                                 is_constenumnode(tvecnode(hp).right) or
                                 is_constcharnode(tvecnode(hp).right) or
                                 is_constboolnode(tvecnode(hp).right)) and
                                not is_implicit_array_pointer(tvecnode(hp).left.resultdef) then
                               ftcb.queue_vecn(tvecnode(hp).left.resultdef,get_ordinal_value(tvecnode(hp).right))
                             else
                               Message(parser_e_illegal_expression);
                           end;
                         subscriptn :
                           ftcb.queue_subscriptn(tabstractrecorddef(tsubscriptnode(hp).left.resultdef),tsubscriptnode(hp).vs);
                         typeconvn :
                           begin
                             if not(ttypeconvnode(hp).convtype in [tc_equal,tc_proc_2_procvar]) then
                               Message(parser_e_illegal_expression)
                             else
                               ftcb.queue_typeconvn(ttypeconvnode(hp).left.resultdef,hp.resultdef);
                           end;
                         addrn :
                           { nothing, is implicit };
                         else
                           Message(parser_e_illegal_expression);
                       end;
                       hp:=tunarynode(hp).left;
                    end;
                  srsym:=tloadnode(hp).symtableentry;
                  case srsym.typ of
                    procsym :
                      begin
                        pd:=tprocdef(tprocsym(srsym).ProcdefList[0]);
                        if Tprocsym(srsym).ProcdefList.Count>1 then
                          Message(parser_e_no_overloaded_procvars);
                        if po_abstractmethod in pd.procoptions then
                          Message(type_e_cant_take_address_of_abstract_method)
                        else
                          ftcb.queue_emit_proc(pd);
                      end;
                    staticvarsym :
                      ftcb.queue_emit_staticvar(tstaticvarsym(srsym));
                    labelsym :
                      ftcb.queue_emit_label(tlabelsym(srsym));
                    constsym :
                      if tconstsym(srsym).consttyp=constresourcestring then
                        ftcb.queue_emit_const(tconstsym(srsym))
                      else
                        Message(type_e_variable_id_expected);
                    else
                      Message(type_e_variable_id_expected);
                  end;
                end
              else
                Message(parser_e_illegal_expression);
            end
        else
        { allow typeof(Object type)}
          if (node.nodetype=inlinen) and
             (tinlinenode(node).inlinenumber=in_typeof_x) then
            begin
              if (tinlinenode(node).left.nodetype=typen) then
                begin
                  // TODO correct type?
                  ftcb.emit_tai(Tai_const.createname(
                    tobjectdef(tinlinenode(node).left.resultdef).vmt_mangledname,AT_DATA,0),
                    voidpointertype);
                end
              else
                Message(parser_e_illegal_expression);
            end
        else
          Message(parser_e_illegal_expression);
      end;


    procedure tasmlisttypedconstbuilder.tc_emit_setdef(def: tsetdef; var node: tnode);
      type
         setbytes = array[0..31] of byte;
         Psetbytes = ^setbytes;
      var
        i: longint;
        setval: cardinal;
      begin
        if node.nodetype=setconstn then
          begin
            { be sure to convert to the correct result, else
              it can generate smallset data instead of normalset (PFV) }
            inserttypeconv(node,def);
            { we only allow const sets }
            if (node.nodetype<>setconstn) or
               assigned(tsetconstnode(node).left) then
              Message(parser_e_illegal_expression)
            else
              begin
                ftcb.maybe_begin_aggregate(def);
                tsetconstnode(node).adjustforsetbase;
                { this writing is endian-dependant   }
                if not is_smallset(def) then
                  begin
                    if source_info.endian=target_info.endian then
                      begin
                        for i:=0 to node.resultdef.size-1 do
                          ftcb.emit_tai(tai_const.create_8bit(Psetbytes(tsetconstnode(node).value_set)^[i]),u8inttype);
                      end
                    else
                      begin
                        for i:=0 to node.resultdef.size-1 do
                          ftcb.emit_tai(tai_const.create_8bit(reverse_byte(Psetbytes(tsetconstnode(node).value_set)^[i])),u8inttype);
                      end;
                  end
                else
                  begin
                    { emit the set as a single constant (would be nicer if we
                      could automatically merge the bytes inside the
                      typed const builder, but it's not easy :/ ) }
                    setval:=0;
                    if source_info.endian=target_info.endian then
                      begin
                        for i:=0 to node.resultdef.size-1 do
                          setval:=setval or (Psetbytes(tsetconstnode(node).value_set)^[i] shl (i*8));
                      end
                    else
                      begin
                        for i:=0 to node.resultdef.size-1 do
                          setval:=setval or (reverse_byte(Psetbytes(tsetconstnode(node).value_set)^[i]) shl (i*8));
                      end;
                    case def.size of
                      1:
                        ftcb.emit_tai(tai_const.create_8bit(setval),def);
                      2:
                        begin
                          if target_info.endian=endian_big then
                            setval:=swapendian(word(setval));
                          ftcb.emit_tai(tai_const.create_16bit(setval),def);
                        end;
                      4:
                        begin
                          if target_info.endian=endian_big then
                            setval:=swapendian(cardinal(setval));
                          ftcb.emit_tai(tai_const.create_32bit(longint(setval)),def);
                        end;
                      else
                        internalerror(2015112207);
                    end;
                  end;
                ftcb.maybe_end_aggregate(def);
              end;
          end
        else
          Message(parser_e_illegal_expression);
      end;


    procedure tasmlisttypedconstbuilder.tc_emit_enumdef(def: tenumdef; var node: tnode);
      begin
        if node.nodetype=ordconstn then
          begin
            if equal_defs(node.resultdef,def) or
               is_subequal(node.resultdef,def) then
              begin
                adaptrange(def,tordconstnode(node).value,rc_default);
                case longint(node.resultdef.size) of
                  1 : ftcb.emit_tai(Tai_const.Create_8bit(Byte(tordconstnode(node).value.svalue)),def);
                  2 : ftcb.emit_tai(Tai_const.Create_16bit(Word(tordconstnode(node).value.svalue)),def);
                  4 : ftcb.emit_tai(Tai_const.Create_32bit(Longint(tordconstnode(node).value.svalue)),def);
                end;
              end
            else
              IncompatibleTypes(node.resultdef,def);
          end
        else
          Message(parser_e_illegal_expression);
      end;


    { parse a single constant and add it to the packed const info  }
    { represented by curval etc (see explanation of bitpackval for }
    { what the different parameters mean)                          }
    function tasmlisttypedconstbuilder.parse_single_packed_const(def: tdef; var bp: tbitpackedval): boolean;
      var
        node: tnode;
      begin
        result:=true;
        node:=comp_expr([ef_accept_equal]);
        if (node.nodetype <> ordconstn) or
           (not equal_defs(node.resultdef,def) and
            not is_subequal(node.resultdef,def)) then
          begin
            incompatibletypes(node.resultdef,def);
            node.free;
            consume_all_until(_SEMICOLON);
            result:=false;
            exit;
          end;
        if (Tordconstnode(node).value<qword(low(Aword))) or (Tordconstnode(node).value>qword(high(Aword))) then
          message3(type_e_range_check_error_bounds,tostr(Tordconstnode(node).value),tostr(low(Aword)),tostr(high(Aword)))
        else
          bitpackval(Tordconstnode(node).value.uvalue,bp);
        if (bp.curbitoffset>=AIntBits) then
          flush_packed_value(bp);
        node.free;
      end;

    procedure tasmlisttypedconstbuilder.get_final_asmlists(out reslist, datalist: tasmlist);
      var
        asmsym: tasmsymbol;
        addstabx: boolean;
        sec: TAsmSectiontype;
        secname: ansistring;
      begin
        addstabx:=false;
        if fsym.globalasmsym then
          begin
            if (target_dbg.id=dbg_stabx) and
               (cs_debuginfo in current_settings.moduleswitches) and
               not assigned(current_asmdata.GetAsmSymbol(fsym.name)) then
              addstabx:=true;
            asmsym:=current_asmdata.DefineAsmSymbol(fsym.mangledname,AB_GLOBAL,AT_DATA,tcsym.vardef)
          end
        else if tf_supports_hidden_symbols in target_info.flags then
          asmsym:=current_asmdata.DefineAsmSymbol(fsym.mangledname,AB_PRIVATE_EXTERN,AT_DATA,tcsym.vardef)
        else
          asmsym:=current_asmdata.DefineAsmSymbol(fsym.mangledname,AB_LOCAL,AT_DATA,tcsym.vardef);
        if vo_has_section in fsym.varoptions then
          begin
            sec:=sec_user;
            secname:=fsym.section;
          end
        else
          begin
            { Certain types like windows WideString are initialized at runtime and cannot
              be placed into readonly memory }
            if (fsym.varspez=vs_const) and
               not (vo_force_finalize in fsym.varoptions) then
              sec:=sec_rodata
            else
              sec:=sec_data;
            secname:=asmsym.Name;
          end;
        reslist:=ftcb.get_final_asmlist(asmsym,fsym.vardef,sec,secname,fsym.vardef.alignment);
        if addstabx then
          begin
            { see same code in ncgutil.insertbssdata }
            reslist.insert(tai_directive.Create(asd_reference,fsym.name));
            reslist.insert(tai_symbol.Create(current_asmdata.DefineAsmSymbol(fsym.name,AB_LOCAL,AT_DATA,tcsym.vardef),0));
          end;
        datalist:=fdatalist;
      end;


    procedure tasmlisttypedconstbuilder.parse_arraydef(def:tarraydef);
      const
        LKlammerToken: array[Boolean] of TToken = (_LKLAMMER, _LECKKLAMMER);
        RKlammerToken: array[Boolean] of TToken = (_RKLAMMER, _RECKKLAMMER);
      var
        n : tnode;
        i : longint;
        len : asizeint;
        ch  : array[0..1] of char;
        ca  : pbyte;
        int_const: tai_const;
        char_size: integer;
        dyncount,
        oldoffset: asizeint;
        dummy : byte;
        sectype : tasmsectiontype;
        oldtcb,
        datatcb : ttai_typedconstbuilder;
        ll : tasmlabel;
        dyncountloc : ttypedconstplaceholder;
        llofs : tasmlabofs;
        dynarrdef : tdef;
      begin
        { dynamic array }
        if is_dynamic_array(def) then
          begin
            if try_to_consume(_NIL) then
              begin
                ftcb.emit_tai(Tai_const.Create_sym(nil),def);
              end
            else if try_to_consume(LKlammerToken[m_delphi in current_settings.modeswitches]) then
              begin
                if try_to_consume(RKlammerToken[m_delphi in current_settings.modeswitches]) then
                  begin
                    ftcb.emit_tai(tai_const.create_sym(nil),def);
                  end
                else
                  begin
                    if fsym.varspez=vs_const then
                      sectype:=sec_rodata
                    else
                      sectype:=sec_data;
                    ftcb.start_internal_data_builder(fdatalist,sectype,'',datatcb,ll);

                    llofs:=datatcb.begin_dynarray_const(def,ll,dyncountloc);

                    dyncount:=0;

                    oldtcb:=ftcb;
                    ftcb:=datatcb;
                    while true do
                      begin
                        read_typed_const_data(def.elementdef);
                        inc(dyncount);
                        if try_to_consume(RKlammerToken[m_delphi in current_settings.modeswitches]) then
                          break
                        else
                          consume(_COMMA);
                      end;
                    ftcb:=oldtcb;

                    dynarrdef:=datatcb.end_dynarray_const(def,dyncount,dyncountloc);

                    ftcb.finish_internal_data_builder(datatcb,ll,dynarrdef,sizeof(pint));

                    ftcb.emit_dynarray_offset(llofs,dyncount,def,trecorddef(dynarrdef));
                  end;
              end
            else
              consume(_LKLAMMER);
          end
        { packed array constant }
        else if is_packed_array(def) and
                (def.elementdef.typ in [orddef,enumdef]) and
                ((def.elepackedbitsize mod 8 <> 0) or
                 not ispowerof2(def.elepackedbitsize div 8,i)) then
          begin
            parse_packed_array_def(def);
          end
        { normal array const between brackets }
        else if try_to_consume(_LKLAMMER) then
          begin
            ftcb.maybe_begin_aggregate(def);
            oldoffset:=curoffset;
            curoffset:=0;
            { in case of a generic subroutine, it might be we cannot
              determine the size yet }
            if assigned(current_procinfo) and (df_generic in current_procinfo.procdef.defoptions) then
              begin
                while true do
                  begin
                    read_typed_const_data(def.elementdef);
                    if token=_RKLAMMER then
                      begin
                        consume(_RKLAMMER);
                        break;
                      end
                    else
                      consume(_COMMA);
                  end;
              end
            else
              begin
                for i:=def.lowrange to def.highrange-1 do
                  begin
                    read_typed_const_data(def.elementdef);
                    Inc(curoffset,def.elementdef.size);
                    if token=_RKLAMMER then
                      begin
                        Message1(parser_e_more_array_elements_expected,tostr(def.highrange-i));
                        consume(_RKLAMMER);
                        exit;
                      end
                    else
                      consume(_COMMA);
                  end;
                read_typed_const_data(def.elementdef);
                consume(_RKLAMMER);
              end;
            curoffset:=oldoffset;
            ftcb.maybe_end_aggregate(def);
          end
        { if array of char then we allow also a string }
        else if is_anychar(def.elementdef) then
          begin
             ftcb.maybe_begin_aggregate(def);
             char_size:=def.elementdef.size;
             n:=comp_expr([ef_accept_equal]);
             if n.nodetype=stringconstn then
               begin
                 len:=tstringconstnode(n).len;
                  case char_size of
                    1:
                     begin
                       if (tstringconstnode(n).cst_type in [cst_unicodestring,cst_widestring]) then
                         inserttypeconv(n,getansistringdef);
                       if n.nodetype<>stringconstn then
                         internalerror(2010033003);
                       ca:=pointer(tstringconstnode(n).value_str);
                     end;
                    2:
                      begin
                        inserttypeconv(n,cunicodestringtype);
                        if n.nodetype<>stringconstn then
                          internalerror(2010033003);
                        ca:=pointer(pcompilerwidestring(tstringconstnode(n).value_str)^.data)
                      end;
                    else
                      internalerror(2010033005);
                  end;
                 { For tp7 the maximum lentgh can be 255 }
                 if (m_tp7 in current_settings.modeswitches) and
                    (len>255) then
                  len:=255;
               end
             else if is_constcharnode(n) then
                begin
                  case char_size of
                    1:
                      ch[0]:=chr(tordconstnode(n).value.uvalue and $ff);
                    2:
                      begin
                        inserttypeconv(n,cwidechartype);
                        if not is_constwidecharnode(n) then
                          internalerror(2010033001);
                        widechar(ch):=widechar(tordconstnode(n).value.uvalue and $ffff);
                      end;
                    else
                      internalerror(2010033002);
                  end;
                  ca:=@ch;
                  len:=1;
                end
             else if is_constwidecharnode(n) and (current_settings.sourcecodepage<>CP_UTF8) then
                begin
                  case char_size of
                    1:
                      begin
                        inserttypeconv(n,cansichartype);
                        if not is_constcharnode(n) then
                          internalerror(2010033001);
                        ch[0]:=chr(tordconstnode(n).value.uvalue and $ff);
                      end;
                    2:
                      widechar(ch):=widechar(tordconstnode(n).value.uvalue and $ffff);
                    else
                      internalerror(2010033002);
                  end;
                  ca:=@ch;
                  len:=1;
                end
             else
               begin
                 Message(parser_e_illegal_expression);
                 len:=0;
                 { avoid crash later on }
                 dummy:=0;
                 ca:=@dummy;
               end;
             if len>(def.highrange-def.lowrange+1) then
               Message(parser_e_string_larger_array);
             for i:=0 to def.highrange-def.lowrange do
               begin
                 if i<len then
                   begin
                     case char_size of
                       1:
                        int_const:=Tai_const.Create_char(char_size,pbyte(ca)^);
                       2:
                        int_const:=Tai_const.Create_char(char_size,pword(ca)^);
                       else
                         internalerror(2010033004);
                     end;
                     inc(ca, char_size);
                   end
                 else
                   {Fill the remaining positions with #0.}
                   int_const:=Tai_const.Create_char(char_size,0);
                 ftcb.emit_tai(int_const,def.elementdef)
               end;
             ftcb.maybe_end_aggregate(def);
             n.free;
          end
        else
          begin
            { we want the ( }
            consume(_LKLAMMER);
          end;
      end;


    procedure tasmlisttypedconstbuilder.parse_procvardef(def:tprocvardef);
      var
        tmpn,n : tnode;
        pd : tprocdef;
        procaddrdef: tprocvardef;
        havepd,
        haveblock: boolean;
      begin
        { Procvars and pointers are no longer compatible.  }
        { under tp:  =nil or =var under fpc: =nil or =@var }
        if try_to_consume(_NIL) then
          begin
             ftcb.maybe_begin_aggregate(def);
             { we need the procdef type called by the procvar here, not the
               procvar record }
             ftcb.emit_tai_procvar2procdef(Tai_const.Create_sym(nil),def);
             if not def.is_addressonly then
               ftcb.emit_tai(Tai_const.Create_sym(nil),voidpointertype);
             ftcb.maybe_end_aggregate(def);
             exit;
          end;
        { you can't assign a value other than NIL to a typed constant  }
        { which is a "procedure of object", because this also requires }
        { address of an object/class instance, which is not known at   }
        { compile time (JM)                                            }
        if (po_methodpointer in def.procoptions) then
          Message(parser_e_no_procvarobj_const);
        { parse the rest too, so we can continue with error checking }
        getprocvardef:=def;
        n:=comp_expr([ef_accept_equal]);
        getprocvardef:=nil;
        if codegenerror then
          begin
            n.free;
            exit;
          end;
        { let type conversion check everything needed }
        inserttypeconv(n,def);
        if codegenerror then
          begin
            n.free;
            exit;
          end;
        { in case of a nested procdef initialised with a global routine }
        ftcb.maybe_begin_aggregate(def);
        { get the address of the procedure, except if it's a C-block (then we
          we will end up with a record that represents the C-block) }
        if not is_block(def) then
          procaddrdef:=cprocvardef.getreusableprocaddr(def)
        else
          procaddrdef:=def;
        ftcb.queue_init(procaddrdef);
        { remove typeconvs, that will normally insert a lea
          instruction which is not necessary for us }
        while n.nodetype=typeconvn do
          begin
            ftcb.queue_typeconvn(ttypeconvnode(n).left.resultdef,n.resultdef);
            tmpn:=ttypeconvnode(n).left;
            ttypeconvnode(n).left:=nil;
            n.free;
            n:=tmpn;
          end;
        { remove addrn which we also don't need here }
        if n.nodetype=addrn then
          begin
            tmpn:=taddrnode(n).left;
            taddrnode(n).left:=nil;
            n.free;
            n:=tmpn;
          end;
        pd:=nil;
        { we now need to have a loadn with a procsym }
        havepd:=
          (n.nodetype=loadn) and
          (tloadnode(n).symtableentry.typ=procsym);
        { or a staticvarsym representing a block }
        haveblock:=
          (n.nodetype=loadn) and
          (tloadnode(n).symtableentry.typ=staticvarsym) and
          (sp_internal in tloadnode(n).symtableentry.symoptions);
        if havepd or
           haveblock then
          begin
            if havepd then
              begin
                pd:=tloadnode(n).procdef;
                ftcb.queue_emit_proc(pd);
              end
            else
              begin
                ftcb.queue_emit_staticvar(tstaticvarsym(tloadnode(n).symtableentry));
              end;
            { nested procvar typed consts can only be initialised with nil
              (checked above) or with a global procedure (checked here),
              because in other cases we need a valid frame pointer }
            if is_nested_pd(def) then
              begin
                if haveblock or
                   is_nested_pd(pd) then
                  Message(parser_e_no_procvarnested_const);
                ftcb.emit_tai(Tai_const.Create_sym(nil),voidpointertype);
              end;
          end
        else if n.nodetype=pointerconstn then
          begin
            ftcb.queue_emit_ordconst(tpointerconstnode(n).value,procaddrdef);
            if not def.is_addressonly then
              ftcb.emit_tai(Tai_const.Create_sym(nil),voidpointertype);
          end
        else
          Message(parser_e_illegal_expression);
        ftcb.maybe_end_aggregate(def);
        n.free;
      end;


    procedure tasmlisttypedconstbuilder.parse_recorddef(def:trecorddef);
      var
        n       : tnode;
        symidx  : longint;
        recsym,
        srsym   : tsym;
        hs      : string;
        sorg,s  : TIDString;
        tmpguid : tguid;
        recoffset,
        fillbytes  : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
        bp   : tbitpackedval;
        error,
        is_packed: boolean;
        startoffset: {$ifdef CPU8BITALU}word{$else}aword{$endif};

      procedure handle_stringconstn;
        begin
          hs:=strpas(tstringconstnode(n).value_str);
          if string2guid(hs,tmpguid) then
            ftcb.emit_guid_const(tmpguid)
          else
            Message(parser_e_improper_guid_syntax);
        end;

      var
        i : longint;
        SymList:TFPHashObjectList;
      begin
        { GUID }
        if (def=rec_tguid) and (token=_ID) then
          begin
            n:=comp_expr([ef_accept_equal]);
            if n.nodetype=stringconstn then
              handle_stringconstn
            else
              begin
                inserttypeconv(n,rec_tguid);
                if n.nodetype=guidconstn then
                  ftcb.emit_guid_const(tguidconstnode(n).value)
                else
                  Message(parser_e_illegal_expression);
              end;
            n.free;
            exit;
          end;
        if (def=rec_tguid) and ((token=_CSTRING) or (token=_CCHAR)) then
          begin
            n:=comp_expr([ef_accept_equal]);
            inserttypeconv(n,cshortstringtype);
            if n.nodetype=stringconstn then
              handle_stringconstn
            else
              Message(parser_e_illegal_expression);
            n.free;
            exit;
          end;
        ftcb.maybe_begin_aggregate(def);
        { bitpacked record? }
        is_packed:=is_packed_record_or_object(def);
        if (is_packed) then
          { packedbitsize will be set separately for each field }
          initbitpackval(bp,0);
        { normal record }
        consume(_LKLAMMER);
        recoffset:=0;
        sorg:='';
        symidx:=0;
        symlist:=def.symtable.SymList;
        srsym:=get_next_varsym(def,symlist,symidx);
        recsym := nil;
        startoffset:=curoffset;
        while token<>_RKLAMMER do
          begin
            s:=pattern;
            sorg:=orgpattern;
            consume(_ID);
            consume(_COLON);
            error := false;
            recsym := tsym(def.symtable.Find(s));
            if not assigned(recsym) then
              begin
                Message1(sym_e_illegal_field,sorg);
                error := true;
              end;
            if (not error) and
               (not assigned(srsym) or
                (s <> srsym.name)) then
              { possible variant record (JM) }
              begin
                { All parts of a variant start at the same offset      }
                { Also allow jumping from one variant part to another, }
                { as long as the offsets match                         }
                if (assigned(srsym) and
                    (tfieldvarsym(recsym).fieldoffset = tfieldvarsym(srsym).fieldoffset)) or
                   { srsym is not assigned after parsing w2 in the      }
                   { typed const in the next example:                   }
                   {   type tr = record case byte of                    }
                   {          1: (l1,l2: dword);                        }
                   {          2: (w1,w2: word);                         }
                   {        end;                                        }
                   {   const r: tr = (w1:1;w2:1;l2:5);                  }
                   (tfieldvarsym(recsym).fieldoffset = recoffset) then
                  begin
                    srsym:=recsym;
                    { symidx should contain the next symbol id to search }
                    symidx:=SymList.indexof(srsym)+1;
                  end
                { going backwards isn't allowed in any mode }
                else if (tfieldvarsym(recsym).fieldoffset<recoffset) then
                  begin
                    Message(parser_e_invalid_record_const);
                    error := true;
                  end
                { Delphi allows you to skip fields }
                else if (m_delphi in current_settings.modeswitches) then
                  begin
                    Message1(parser_w_skipped_fields_before,sorg);
                    srsym := recsym;
                  end
                { FPC and TP don't }
                else
                  begin
                    Message1(parser_e_skipped_fields_before,sorg);
                    error := true;
                  end;
              end;
            if error then
              consume_all_until(_SEMICOLON)
            else
              begin
                { if needed fill (alignment) }
                if tfieldvarsym(srsym).fieldoffset>recoffset then
                  begin
                    if not(is_packed) then
                      fillbytes:=0
                    else
                      begin
                        flush_packed_value(bp);
                        { curoffset is now aligned to the next byte }
                        recoffset:=align(recoffset,8);
                        { offsets are in bits in this case }
                        fillbytes:=(tfieldvarsym(srsym).fieldoffset-recoffset) div 8;
                      end;
                    for i:=1 to fillbytes do
                      ftcb.emit_tai(Tai_const.Create_8bit(0),u8inttype)
                  end;

                { new position }
                recoffset:=tfieldvarsym(srsym).fieldoffset;
                if not(is_packed) then
                  inc(recoffset,tfieldvarsym(srsym).vardef.size)
                 else
                   inc(recoffset,tfieldvarsym(srsym).vardef.packedbitsize);

                { read the data }
                ftcb.next_field:=tfieldvarsym(srsym);
                if not(is_packed) or
                   { only orddefs and enumdefs are bitpacked, as in gcc/gpc }
                   not(tfieldvarsym(srsym).vardef.typ in [orddef,enumdef]) then
                  begin
                    if is_packed then
                      begin
                        flush_packed_value(bp);
                        recoffset:=align(recoffset,8);
                      end;
                    curoffset:=startoffset+tfieldvarsym(srsym).fieldoffset;
                    read_typed_const_data(tfieldvarsym(srsym).vardef);
                  end
                else
                  begin
                    bp.packedbitsize:=tfieldvarsym(srsym).vardef.packedbitsize;
                    parse_single_packed_const(tfieldvarsym(srsym).vardef,bp);
                  end;

                { keep previous field for checking whether whole }
                { record was initialized (JM)                    }
                recsym := srsym;
                { goto next field }
                srsym:=get_next_varsym(def,SymList,symidx);

                if token=_SEMICOLON then
                  consume(_SEMICOLON)
                else if (token=_COMMA) and (m_mac in current_settings.modeswitches) then
                  consume(_COMMA)
                else
                  break;
              end;
          end;
        curoffset:=startoffset;

        { are there any fields left, but don't complain if there only
          come other variant parts after the last initialized field }
        if assigned(srsym) and
           (
            (recsym=nil) or
            (tfieldvarsym(srsym).fieldoffset > tfieldvarsym(recsym).fieldoffset)
           ) then
          Message1(parser_w_skipped_fields_after,sorg);

        if not error then
          begin
            if not(is_packed) then
              fillbytes:=0
            else
              begin
                flush_packed_value(bp);
                recoffset:=align(recoffset,8);
                fillbytes:=def.size-(recoffset div 8);
              end;
            for i:=1 to fillbytes do
              ftcb.emit_tai(Tai_const.Create_8bit(0),u8inttype);
          end;

        ftcb.maybe_end_aggregate(def);
        consume(_RKLAMMER);
      end;


    procedure tasmlisttypedconstbuilder.parse_objectdef(def:tobjectdef);
      var
        n      : tnode;
        obj    : tobjectdef;
        srsym  : tsym;
        st     : tsymtable;
        objoffset : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
        s,sorg : TIDString;
        vmtwritten : boolean;
        startoffset : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
      begin
        { no support for packed object }
        if is_packed_record_or_object(def) then
          begin
            Message(type_e_no_const_packed_record);
            exit;
          end;

        { only allow nil for implicit pointer object types }
        if is_implicit_pointer_object_type(def) then
          begin
            n:=comp_expr([ef_accept_equal]);
            if n.nodetype<>niln then
              begin
                Message(parser_e_type_const_not_possible);
                consume_all_until(_SEMICOLON);
              end
            else
              ftcb.emit_tai(Tai_const.Create_sym(nil),def);
            n.free;
            exit;
          end;

        { for objects we allow it only if it doesn't contain a vmt }
        if (oo_has_vmt in def.objectoptions) and
           (m_fpc in current_settings.modeswitches) then
          begin
            Message(parser_e_type_object_constants);
            exit;
          end;

        ftcb.maybe_begin_aggregate(def);

        consume(_LKLAMMER);
        startoffset:=curoffset;
        objoffset:=0;
        vmtwritten:=false;
        while token<>_RKLAMMER do
          begin
            s:=pattern;
            sorg:=orgpattern;
            consume(_ID);
            consume(_COLON);
            srsym:=nil;
            obj:=tobjectdef(def);
            st:=obj.symtable;
            while (srsym=nil) and assigned(st) do
              begin
                srsym:=tsym(st.Find(s));
                if assigned(obj) then
                  obj:=obj.childof;
                if assigned(obj) then
                  st:=obj.symtable
                else
                  st:=nil;
              end;

            if (srsym=nil) or
               (srsym.typ<>fieldvarsym) then
              begin
                if (srsym=nil) then
                  Message1(sym_e_id_not_found,sorg)
                else
                  Message1(sym_e_illegal_field,sorg);
                consume_all_until(_RKLAMMER);
                break;
              end
            else
              with tfieldvarsym(srsym) do
                begin
                  { check position }
                  if fieldoffset<objoffset then
                    message(parser_e_invalid_record_const);

                  { check in VMT needs to be added for TP mode }
                  if not(vmtwritten) and
                     not(m_fpc in current_settings.modeswitches) and
                     (oo_has_vmt in def.objectoptions) and
                     (def.vmt_offset<fieldoffset) then
                    begin
                      ftcb.next_field:=tfieldvarsym(def.vmt_field);
                      ftcb.emit_tai(tai_const.createname(def.vmt_mangledname,AT_DATA,0),tfieldvarsym(def.vmt_field).vardef);
                      objoffset:=def.vmt_offset+tfieldvarsym(def.vmt_field).vardef.size;
                      vmtwritten:=true;
                    end;

                  ftcb.next_field:=tfieldvarsym(srsym);

                  { new position }
                  objoffset:=fieldoffset+vardef.size;

                  { read the data }
                  curoffset:=startoffset+fieldoffset;
                  read_typed_const_data(vardef);

                  if not try_to_consume(_SEMICOLON) then
                    break;
                end;
          end;
        curoffset:=startoffset;
        { add VMT pointer if we stopped writing fields before the VMT was
          written }
        if not(m_fpc in current_settings.modeswitches) and
           (oo_has_vmt in def.objectoptions) and
           (def.vmt_offset>=objoffset) then
          begin
            ftcb.next_field:=tfieldvarsym(def.vmt_field);
            ftcb.emit_tai(tai_const.createname(def.vmt_mangledname,AT_DATA,0),tfieldvarsym(def.vmt_field).vardef);
            objoffset:=def.vmt_offset+tfieldvarsym(def.vmt_field).vardef.size;
          end;
        ftcb.maybe_end_aggregate(def);
        consume(_RKLAMMER);
      end;


    procedure tasmlisttypedconstbuilder.parse_into_asmlist;
      begin
        read_typed_const_data(tcsym.vardef);
      end;


    { tnodetreetypedconstbuilder }

    procedure tnodetreetypedconstbuilder.parse_arraydef(def: tarraydef);
      var
        n : tnode;
        i : longint;
        orgbase: tnode;
      begin
        { dynamic array nil }
        if is_dynamic_array(def) then
          begin
            { Only allow nil initialization }
            consume(_NIL);
            addstatement(statmnt,cassignmentnode.create_internal(basenode,cnilnode.create));
            basenode:=nil;
          end
        { array const between brackets }
        else if try_to_consume(_LKLAMMER) then
          begin
            orgbase:=basenode;
            for i:=def.lowrange to def.highrange-1 do
              begin
                basenode:=cvecnode.create(orgbase.getcopy,ctypeconvnode.create_explicit(genintconstnode(i),tarraydef(def).rangedef));
                read_typed_const_data(def.elementdef);
                if token=_RKLAMMER then
                  begin
                    Message1(parser_e_more_array_elements_expected,tostr(def.highrange-i));
                    consume(_RKLAMMER);
                    exit;
                  end
                else
                  consume(_COMMA);
              end;
            basenode:=cvecnode.create(orgbase,ctypeconvnode.create_explicit(genintconstnode(def.highrange),tarraydef(def).rangedef));
            read_typed_const_data(def.elementdef);
            consume(_RKLAMMER);
          end
        { if array of char then we allow also a string }
        else if is_anychar(def.elementdef) then
          begin
             n:=comp_expr([ef_accept_equal]);
             addstatement(statmnt,cassignmentnode.create_internal(basenode,n));
             basenode:=nil;
          end
        else
          begin
            { we want the ( }
            consume(_LKLAMMER);
          end;
      end;


    procedure tnodetreetypedconstbuilder.parse_procvardef(def: tprocvardef);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,comp_expr([ef_accept_equal])));
        basenode:=nil;
      end;


    procedure tnodetreetypedconstbuilder.parse_recorddef(def: trecorddef);
      var
        n,n2    : tnode;
        SymList:TFPHashObjectList;
        orgbasenode : tnode;
        symidx  : longint;
        recsym,
        srsym   : tsym;
        sorg,s  : TIDString;
        recoffset : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
        error,
        is_packed: boolean;

      procedure handle_stringconstn;
        begin
          addstatement(statmnt,cassignmentnode.create_internal(basenode,n));
          basenode:=nil;
          n:=nil;
        end;

      begin
        { GUID }
        if (def=rec_tguid) and (token=_ID) then
          begin
            n:=comp_expr([ef_accept_equal]);
            if n.nodetype=stringconstn then
              handle_stringconstn
            else
              begin
                inserttypeconv(n,rec_tguid);
                if n.nodetype=guidconstn then
                  begin
                    n2:=cstringconstnode.createstr(guid2string(tguidconstnode(n).value));
                    n.free;
                    n:=n2;
                    handle_stringconstn;
                  end
                else
                  Message(parser_e_illegal_expression);
              end;
            n.free;
            exit;
          end;
        if (def=rec_tguid) and ((token=_CSTRING) or (token=_CCHAR)) then
          begin
            n:=comp_expr([ef_accept_equal]);
            inserttypeconv(n,cshortstringtype);
            if n.nodetype=stringconstn then
              handle_stringconstn
            else
              Message(parser_e_illegal_expression);
            n.free;
            exit;
          end;
        { bitpacked record? }
        is_packed:=is_packed_record_or_object(def);
        { normal record }
        consume(_LKLAMMER);
        recoffset:=0;
        sorg:='';
        symidx:=0;
        symlist:=def.symtable.SymList;
        srsym:=get_next_varsym(def,symlist,symidx);
        recsym := nil;
        orgbasenode:=basenode;
        basenode:=nil;
        while token<>_RKLAMMER do
          begin
            s:=pattern;
            sorg:=orgpattern;
            consume(_ID);
            consume(_COLON);
            error := false;
            recsym := tsym(def.symtable.Find(s));
            if not assigned(recsym) then
              begin
                Message1(sym_e_illegal_field,sorg);
                error := true;
              end;
            if (not error) and
               (not assigned(srsym) or
                (s <> srsym.name)) then
              { possible variant record (JM) }
              begin
                { All parts of a variant start at the same offset      }
                { Also allow jumping from one variant part to another, }
                { as long as the offsets match                         }
                if (assigned(srsym) and
                    (tfieldvarsym(recsym).fieldoffset = tfieldvarsym(srsym).fieldoffset)) or
                   { srsym is not assigned after parsing w2 in the      }
                   { typed const in the next example:                   }
                   {   type tr = record case byte of                    }
                   {          1: (l1,l2: dword);                        }
                   {          2: (w1,w2: word);                         }
                   {        end;                                        }
                   {   const r: tr = (w1:1;w2:1;l2:5);                  }
                   (tfieldvarsym(recsym).fieldoffset = recoffset) then
                  begin
                    srsym:=recsym;
                    { symidx should contain the next symbol id to search }
                    symidx:=SymList.indexof(srsym)+1;
                  end
                { going backwards isn't allowed in any mode }
                else if (tfieldvarsym(recsym).fieldoffset<recoffset) then
                  begin
                    Message(parser_e_invalid_record_const);
                    error := true;
                  end
                { Delphi allows you to skip fields }
                else if (m_delphi in current_settings.modeswitches) then
                  begin
                    Message1(parser_w_skipped_fields_before,sorg);
                    srsym := recsym;
                  end
                { FPC and TP don't }
                else
                  begin
                    Message1(parser_e_skipped_fields_before,sorg);
                    error := true;
                  end;
              end;
            if error then
              consume_all_until(_SEMICOLON)
            else
              begin
                { skipping fill bytes happens automatically, since we only
                  initialize the defined fields }
                { new position }
                recoffset:=tfieldvarsym(srsym).fieldoffset;
                if not(is_packed) then
                  inc(recoffset,tfieldvarsym(srsym).vardef.size)
                 else
                   inc(recoffset,tfieldvarsym(srsym).vardef.packedbitsize);

                { read the data }
                if is_packed and
                   { only orddefs and enumdefs are bitpacked, as in gcc/gpc }
                   not(tfieldvarsym(srsym).vardef.typ in [orddef,enumdef]) then
                  recoffset:=align(recoffset,8);
                basenode:=csubscriptnode.create(srsym,orgbasenode.getcopy);
                read_typed_const_data(tfieldvarsym(srsym).vardef);

                { keep previous field for checking whether whole }
                { record was initialized (JM)                    }
                recsym := srsym;
                { goto next field }
                srsym:=get_next_varsym(def,SymList,symidx);
                if token=_SEMICOLON then
                  consume(_SEMICOLON)
                else if (token=_COMMA) and (m_mac in current_settings.modeswitches) then
                  consume(_COMMA)
                else
                  break;
              end;
          end;

        { are there any fields left, but don't complain if there only
          come other variant parts after the last initialized field }
        if assigned(srsym) and
           (
            (recsym=nil) or
            (tfieldvarsym(srsym).fieldoffset > tfieldvarsym(recsym).fieldoffset)
           ) then
          Message1(parser_w_skipped_fields_after,sorg);
        orgbasenode.free;
        basenode:=nil;

        consume(_RKLAMMER);
      end;


    procedure tnodetreetypedconstbuilder.parse_objectdef(def: tobjectdef);
      var
        n,
        orgbasenode : tnode;
        obj    : tobjectdef;
        srsym  : tsym;
        st     : tsymtable;
        objoffset : {$ifdef CPU8BITALU}smallint{$else}aint{$endif};
        s,sorg : TIDString;
      begin
        { no support for packed object }
        if is_packed_record_or_object(def) then
          begin
            Message(type_e_no_const_packed_record);
            exit;
          end;

        { only allow nil for implicit pointer object types }
        if is_implicit_pointer_object_type(def) then
          begin
            n:=comp_expr([ef_accept_equal]);
            if n.nodetype<>niln then
              begin
                Message(parser_e_type_const_not_possible);
                consume_all_until(_SEMICOLON);
              end
            else
              begin
                addstatement(statmnt,cassignmentnode.create_internal(basenode,n));
                n:=nil;
                basenode:=nil;
              end;
            n.free;
            exit;
          end;

        { for objects we allow it only if it doesn't contain a vmt }
        if (oo_has_vmt in def.objectoptions) and
           (m_fpc in current_settings.modeswitches) then
          begin
            Message(parser_e_type_object_constants);
            exit;
          end;

        consume(_LKLAMMER);
        objoffset:=0;
        orgbasenode:=basenode;
        basenode:=nil;
        while token<>_RKLAMMER do
          begin
            s:=pattern;
            sorg:=orgpattern;
            consume(_ID);
            consume(_COLON);
            srsym:=nil;
            obj:=tobjectdef(def);
            st:=obj.symtable;
            while (srsym=nil) and assigned(st) do
              begin
                srsym:=tsym(st.Find(s));
                if assigned(obj) then
                  obj:=obj.childof;
                if assigned(obj) then
                  st:=obj.symtable
                else
                  st:=nil;
              end;

            if (srsym=nil) or
               (srsym.typ<>fieldvarsym) then
              begin
                if (srsym=nil) then
                  Message1(sym_e_id_not_found,sorg)
                else
                  Message1(sym_e_illegal_field,sorg);
                consume_all_until(_RKLAMMER);
                break;
              end
            else
              with tfieldvarsym(srsym) do
                begin
                  { check position }
                  if fieldoffset<objoffset then
                    message(parser_e_invalid_record_const);

                  { new position }
                  objoffset:=fieldoffset+vardef.size;

                  { read the data }
                  basenode:=csubscriptnode.create(srsym,orgbasenode.getcopy);
                  read_typed_const_data(vardef);

                  if not try_to_consume(_SEMICOLON) then
                    break;
                end;
          end;
        consume(_RKLAMMER);
      end;


    procedure tnodetreetypedconstbuilder.tc_emit_orddef(def: torddef; var node: tnode);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,node));
        basenode:=nil;
        node:=nil;
      end;


    procedure tnodetreetypedconstbuilder.tc_emit_floatdef(def: tfloatdef; var node: tnode);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,node));
        basenode:=nil;
        node:=nil;
      end;


    procedure tnodetreetypedconstbuilder.tc_emit_classrefdef(def: tclassrefdef; var node: tnode);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,node));
        basenode:=nil;
        node:=nil;
      end;


    procedure tnodetreetypedconstbuilder.tc_emit_pointerdef(def: tpointerdef; var node: tnode);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,node));
        basenode:=nil;
        node:=nil;
      end;


    procedure tnodetreetypedconstbuilder.tc_emit_setdef(def: tsetdef; var node: tnode);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,node));
        basenode:=nil;
        node:=nil;
      end;


    procedure tnodetreetypedconstbuilder.tc_emit_enumdef(def: tenumdef; var node: tnode);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,node));
        basenode:=nil;
        node:=nil;
      end;


    procedure tnodetreetypedconstbuilder.tc_emit_stringdef(def: tstringdef; var node: tnode);
      begin
        addstatement(statmnt,cassignmentnode.create_internal(basenode,node));
        basenode:=nil;
        node:=nil;
      end;


    constructor tnodetreetypedconstbuilder.create(sym: tstaticvarsym; previnit: tnode);
      begin
        inherited create(sym);
        basenode:=cloadnode.create(sym,sym.owner);
        resultblock:=internalstatements(statmnt);
        if assigned(previnit) then
          addstatement(statmnt,previnit);
      end;


    destructor tnodetreetypedconstbuilder.destroy;
      begin
        freeandnil(basenode);
        freeandnil(resultblock);
        inherited destroy;
      end;


    function tnodetreetypedconstbuilder.parse_into_nodetree: tnode;
      begin
        read_typed_const_data(tcsym.vardef);
        result:=self.resultblock;
        self.resultblock:=nil;
      end;

begin
  { default to asmlist version, best for most targets }
  ctypedconstbuilder:=tasmlisttypedconstbuilder;
end.
