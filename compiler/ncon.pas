{
    Copyright (c) 2000-2002 by Florian Klaempfl

    Type checking and register allocation for constants

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
unit ncon;

{$i fpcdefs.inc}

interface

    uses
      globtype,widestr,constexp,
      node,
      aasmbase,aasmcnst,cpuinfo,globals,
      symconst,symtype,symdef,symsym;

    type
       tconstnode = class abstract(tnode)
         { directly emit a node's constant data as a constant and return the
           amount of data written }
         function emit_data(tcb:ttai_typedconstbuilder):sizeint;virtual;abstract;
       end;

       trealconstnode = class(tconstnode)
          typedef : tdef;
          typedefderef : tderef;
          value_real : bestreal;
          value_currency : currency;
          lab_real : tasmlabel;
          constructor create(v : bestreal;def:tdef);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode) : boolean; override;
          procedure printnodedata(var t:text);override;
          function emit_data(tcb:ttai_typedconstbuilder):sizeint; override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;
       trealconstnodeclass = class of trealconstnode;

       tordconstnode = class(tconstnode)
          typedef : tdef;
          typedefderef : tderef;
          value : TConstExprInt;
          rangecheck : boolean;
          { create an ordinal constant node of the specified type and value.
            _rangecheck determines if the value of the ordinal should be checked
            against the ranges of the type definition.
          }
          constructor create(const v : tconstexprint;def:tdef; _rangecheck : boolean);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode) : boolean; override;
          procedure printnodedata(var t:text);override;
          function emit_data(tcb:ttai_typedconstbuilder):sizeint; override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeInfo(var T: Text); override;
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;
       tordconstnodeclass = class of tordconstnode;

       tpointerconstnode = class(tconstnode)
          typedef : tdef;
          typedefderef : tderef;
          value   : TConstPtrUInt;
          constructor create(v : TConstPtrUInt;def:tdef);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode) : boolean; override;
          procedure printnodedata(var t : text); override;
          function emit_data(tcb:ttai_typedconstbuilder):sizeint; override;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;
       tpointerconstnodeclass = class of tpointerconstnode;

       tconststringtype = (
         cst_conststring,
         cst_shortstring,
         cst_longstring,
         cst_ansistring,
         cst_widestring,
         cst_unicodestring
       );

       tstringconstnode = class(tconstnode)
          value_str : pchar;
          len     : longint;
          lab_str : tasmlabel;
          astringdef : tdef;
          astringdefderef : tderef;
          cst_type : tconststringtype;
          constructor createstr(const s : string);virtual;
          constructor createpchar(s: pchar; l: longint; def: tdef);virtual;
          constructor createunistr(w : pcompilerwidestring);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          destructor destroy;override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function getpcharcopy : pchar;
          function docompare(p: tnode) : boolean; override;
          procedure changestringtype(def:tdef);
          function fullcompare(p: tstringconstnode): longint;
          function emit_data(tcb:ttai_typedconstbuilder):sizeint; override;
          { returns whether this platform uses the nil pointer to represent
            empty dynamic strings }
          class function emptydynstrnil: boolean; virtual;
{$ifdef DEBUG_NODE_XML}
          procedure XMLPrintNodeData(var T: Text); override;
{$endif DEBUG_NODE_XML}
       end;
       tstringconstnodeclass = class of tstringconstnode;

       tsetconstnode = class(tunarynode)
          typedef : tdef;
          typedefderef : tderef;
          value_set : pconstset;
          lab_set : tasmsymbol;
          constructor create(s : pconstset;def:tdef);virtual;
          destructor destroy;override;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          procedure adjustforsetbase;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode) : boolean; override;
          function elements : AInt;
          function emit_data(tcb:ttai_typedconstbuilder):sizeint;
       end;
       tsetconstnodeclass = class of tsetconstnode;

       tnilnode = class(tconstnode)
          constructor create;virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function emit_data(tcb:ttai_typedconstbuilder):sizeint; override;
       end;
       tnilnodeclass = class of tnilnode;

       tguidconstnode = class(tconstnode)
          value : tguid;
          lab_set : tasmsymbol;
          constructor create(const g:tguid);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode) : boolean; override;
          function emit_data(tcb:ttai_typedconstbuilder):sizeint; override;
       end;
       tguidconstnodeclass = class of tguidconstnode;

    var
       crealconstnode : trealconstnodeclass = trealconstnode;
       cordconstnode : tordconstnodeclass = tordconstnode;
       cpointerconstnode : tpointerconstnodeclass = tpointerconstnode;
       cstringconstnode : tstringconstnodeclass = tstringconstnode;
       csetconstnode : tsetconstnodeclass = tsetconstnode;
       cguidconstnode : tguidconstnodeclass = tguidconstnode;
       cnilnode : tnilnodeclass=tnilnode;

    function genintconstnode(const v : TConstExprInt) : tordconstnode;
    function genenumnode(v : tenumsym) : tordconstnode;

    { some helper routines }
    function get_ordinal_value(p : tnode) : TConstExprInt;
    function get_string_value(p : tnode; def: tstringdef) : tstringconstnode;
    function is_constresourcestringnode(p : tnode) : boolean;
    function is_emptyset(p : tnode):boolean;
    function genconstsymtree(p : tconstsym) : tnode;

    function getbooleanvalue(p : tnode) : boolean;

implementation

    uses
      cutils,
      verbose,systems,sysutils,ppu,
      defcmp,defutil,procinfo,
      aasmdata,aasmtai,
      cgbase,
      nld;

    function genintconstnode(const v : TConstExprInt) : tordconstnode;
      var
        htype : tdef;
      begin
         int_to_type(v,htype);
         genintconstnode:=cordconstnode.create(v,htype,true);
      end;


    function genenumnode(v : tenumsym) : tordconstnode;
      var
        htype : tdef;
      begin
         htype:=v.definition;
         genenumnode:=cordconstnode.create(int64(v.value),htype,true);
      end;


    function get_ordinal_value(p : tnode) : TConstExprInt;
      begin
        get_ordinal_value:=0;
        if is_constnode(p) then
          begin
            if p.nodetype=ordconstn then
              get_ordinal_value:=tordconstnode(p).value
            else
              Message(type_e_ordinal_expr_expected);
          end
        else
          Message(type_e_constant_expr_expected);
      end;

    function get_string_value(p: tnode; def: tstringdef): tstringconstnode;
      var
        stringVal: string;
        pWideStringVal: pcompilerwidestring;
      begin
        stringVal:='';
        if is_constcharnode(p) then
          begin
            SetLength(stringVal,1);
            stringVal[1]:=char(tordconstnode(p).value.uvalue);
            result:=cstringconstnode.createstr(stringVal);
          end
        else if is_constwidecharnode(p) then
          begin
            initwidestring(pWideStringVal);
            concatwidestringchar(pWideStringVal, tcompilerwidechar(tordconstnode(p).value.uvalue));
            result:=cstringconstnode.createunistr(pWideStringVal);
          end
        else if p.nodetype=stringconstn then
          result:=tstringconstnode(p.getcopy)
        else
          begin
            Message(type_e_string_expr_expected);
            stringVal:='';
            result:=cstringconstnode.createstr(stringVal);
          end;
        result.changestringtype(def);
      end;


    function is_constresourcestringnode(p : tnode) : boolean;
      begin
        is_constresourcestringnode:=(p.nodetype=loadn) and
          (tloadnode(p).symtableentry.typ=constsym) and
          (tconstsym(tloadnode(p).symtableentry).consttyp=constresourcestring);
      end;


    function is_emptyset(p : tnode):boolean;
      begin
        is_emptyset:=(p.nodetype=setconstn) and
                     (Tsetconstnode(p).value_set^=[]);
      end;


    function genconstsymtree(p : tconstsym) : tnode;
      var
        p1  : tnode;
        len : longint;
        pc  : pchar;
      begin
        p1:=nil;
        case p.consttyp of
          constord :
            begin
              if p.constdef=nil then
                internalerror(200403232);
              p1:=cordconstnode.create(p.value.valueord,p.constdef,true);
            end;
          conststring :
            begin
              len:=p.value.len;
              if not(cs_refcountedstrings in current_settings.localswitches) and (len>255) then
                begin
                  message(parser_e_string_const_too_long);
                  len:=255;
                end;
              getmem(pc,len+1);
              move(pchar(p.value.valueptr)^,pc^,len);
              pc[len]:=#0;
              p1:=cstringconstnode.createpchar(pc,len,p.constdef);
            end;
          constwstring :
            p1:=cstringconstnode.createunistr(pcompilerwidestring(p.value.valueptr));
          constreal :
            p1:=crealconstnode.create(pbestreal(p.value.valueptr)^,p.constdef);
          constset :
            p1:=csetconstnode.create(pconstset(p.value.valueptr),p.constdef);
          constpointer :
            p1:=cpointerconstnode.create(p.value.valueordptr,p.constdef);
          constnil :
            p1:=cnilnode.create;
          constguid :
            p1:=cguidconstnode.create(pguid(p.value.valueptr)^);
          else
            internalerror(200205103);
        end;
        genconstsymtree:=p1;
      end;


    function getbooleanvalue(p : tnode) : boolean;
      begin
        if is_constboolnode(p) then
          result:=tordconstnode(p).value<>0
        else
          internalerror(2013111601);
      end;

{*****************************************************************************
                             TREALCONSTNODE
*****************************************************************************}

    { generic code     }
    { overridden by:   }
    {   i386           }
    constructor trealconstnode.create(v : bestreal;def:tdef);
      begin
         if current_settings.fputype=fpu_none then
            internalerror(2008022401);
         inherited create(realconstn);
         typedef:=def;
         case tfloatdef(def).floattype of
           s32real:
             v:=single(v);
           s64real:
             v:=double(v);
           s80real,
           sc80real,
           s64comp,
           s64currency:
             v:=extended(v);
           s128real:
             internalerror(2013102701);
         end;
         value_real:=v;
         value_currency:=v;
         lab_real:=nil;
      end;

    constructor trealconstnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      var
        i : int64;
      begin
        inherited ppuload(t,ppufile);
        ppufile.getderef(typedefderef);
        value_real:=ppufile.getreal;
        i:=ppufile.getint64;
        value_currency:=PCurrency(@i)^;
        lab_real:=tasmlabel(ppufile.getasmsymbol);
      end;


    procedure trealconstnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(typedefderef);
        ppufile.putreal(value_real);
        ppufile.putint64(PInt64(@value_currency)^);
        ppufile.putasmsymbol(lab_real);
      end;


    procedure trealconstnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        typedefderef.build(typedef);
      end;


    procedure trealconstnode.derefimpl;
      begin
        inherited derefimpl;
        typedef:=tdef(typedefderef.resolve);
      end;


    function trealconstnode.dogetcopy : tnode;
      var
         n : trealconstnode;
      begin
         n:=trealconstnode(inherited dogetcopy);
         n.typedef:=typedef;
         n.value_real:=value_real;
         n.value_currency:=value_currency;
         n.lab_real:=lab_real;
         dogetcopy:=n;
      end;


    function trealconstnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=typedef;

        { range checking? }
        if floating_point_range_check_error or
           (tfloatdef(resultdef).floattype in [s64comp,s64currency]) then
          begin
            { use CGMessage so that the resultdef will get set to errordef
              by pass1.typecheckpass_internal if a range error was triggered,
              which in turn will prevent any potential parent type conversion
              node from creating a new realconstnode with this exact same value
              and hence trigger the same error again }
            case tfloatdef(resultdef).floattype of
              s32real :
                begin
                  if ts32real(value_real)=MathInf.Value then
                    CGMessage(parser_e_range_check_error);
                end;
              s64real:
                begin
                  if ts64real(value_real)=MathInf.Value then
                    CGMessage(parser_e_range_check_error);
                end;
              s80real,
              sc80real:
                begin
                  if ts80real(value_real)=MathInf.Value then
                    CGMessage(parser_e_range_check_error);
                end;
              s64comp,
              s64currency:
                begin
                  if (value_real>9223372036854775807.0) or
                     (value_real<-9223372036854775808.0) then
                    CGMessage(parser_e_range_check_error)
                end;
              s128real:
                begin
                  if ts128real(value_real)=MathInf.Value then
                    CGMessage(parser_e_range_check_error);
                end;
            end;
          end;
      end;


    function trealconstnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_CREFERENCE;
      end;


    function trealconstnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          { this should be always true }
          (trealconstnode(p).typedef.typ=floatdef) and (typedef.typ=floatdef) and
          (tfloatdef(typedef).floattype = tfloatdef(trealconstnode(p).typedef).floattype) and
          (
           (
            (tfloatdef(typedef).floattype=s64currency) and
            (value_currency=trealconstnode(p).value_currency)
           )
           or
           (
            (tfloatdef(typedef).floattype<>s64currency) and
            (value_real = trealconstnode(p).value_real) and
            { floating point compares for non-numbers give strange results usually }
            is_number_float(value_real) and
            is_number_float(trealconstnode(p).value_real)
           )
          );
      end;


    procedure trealconstnode.printnodedata(var t: text);
      begin
        inherited printnodedata(t);
        writeln(t,printnodeindention,'value = ',value_real);
      end;

    function trealconstnode.emit_data(tcb:ttai_typedconstbuilder):sizeint;
      begin
        case tfloatdef(typedef).floattype of
          s32real:
            tcb.emit_tai(tai_realconst.create_s32real(value_real),typedef);
          s64real:
            tcb.emit_tai(tai_realconst.create_s64real(value_real),typedef);
          s80real:
            tcb.emit_tai(tai_realconst.create_s80real(value_real,s80floattype.size),typedef);
          sc80real:
            tcb.emit_tai(tai_realconst.create_s80real(value_real,sc80floattype.size),typedef);
          s64comp:
            { the round is necessary for native compilers where comp isn't a float }
            tcb.emit_tai(tai_realconst.create_s64compreal(round(value_real)),typedef);
          s64currency:
            { we don't need to multiply by 10000 here }
            tcb.emit_tai(tai_realconst.create_s64compreal(round(value_real)),typedef);
          s128real:
            internalerror(2019070804);
        end;
        result:=resultdef.size;
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TRealConstNode.XMLPrintNodeData(var T: Text);
      begin
        inherited XMLPrintNodeData(T);
        WriteLn(T, printnodeindention, '<value>', value_real, '</value>');
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                              TORDCONSTNODE
*****************************************************************************}

    constructor tordconstnode.create(const v : tconstexprint;def:tdef;_rangecheck : boolean);

      begin
         inherited create(ordconstn);
         value:=v;
         typedef:=def;
         rangecheck := _rangecheck;
      end;


    constructor tordconstnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getderef(typedefderef);
        value:=ppufile.getexprint;
        { normally, the value is already compiled, so we don't need
          to do once again a range check
        }
        rangecheck := false;
      end;


    procedure tordconstnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(typedefderef);
        ppufile.putexprint(value);
      end;


    procedure tordconstnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        typedefderef.build(typedef);
      end;


    procedure tordconstnode.derefimpl;
      begin
        inherited derefimpl;
        typedef:=tdef(typedefderef.resolve);
      end;


    function tordconstnode.dogetcopy : tnode;

      var
         n : tordconstnode;

      begin
         n:=tordconstnode(inherited dogetcopy);
         n.value:=value;
         n.typedef := typedef;
         dogetcopy:=n;
      end;

    function tordconstnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=typedef;
        { only do range checking when explicitly asked for it
          and if the type can be range checked, see tests/tbs/tb0539.pp }
        if (resultdef.typ in [orddef,enumdef]) then
          adaptrange(resultdef,value,nf_internal in flags, not rangecheck)
      end;

    function tordconstnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_CONSTANT;
      end;

    function tordconstnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (value = tordconstnode(p).value) and
          equal_defs(typedef,tordconstnode(p).typedef);
      end;


        procedure tordconstnode.printnodedata(var t: text);
      begin
        inherited printnodedata(t);
        writeln(t,printnodeindention,'value = ',tostr(value));
      end;

    function tordconstnode.emit_data(tcb:ttai_typedconstbuilder):sizeint;
      begin
        tcb.emit_ord_const(value,resultdef);
        result:=resultdef.size;
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TOrdConstNode.XMLPrintNodeInfo(var T: Text);
      begin
        inherited XMLPrintNodeInfo(T);
        Write(T, ' rangecheck="', rangecheck, '"');
      end;


    procedure TOrdConstNode.XMLPrintNodeData(var T: Text);
      begin
        inherited XMLPrintNodeData(T);
        WriteLn(T, printnodeindention, '<value>', tostr(value), '</value>');
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                            TPOINTERCONSTNODE
*****************************************************************************}

    constructor tpointerconstnode.create(v : TConstPtrUInt;def:tdef);

      begin
         inherited create(pointerconstn);
         value:=v;
         typedef:=def;
      end;


    constructor tpointerconstnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getderef(typedefderef);
        value:=ppufile.getptruint;
      end;


    procedure tpointerconstnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(typedefderef);
        ppufile.putptruint(value);
      end;


    procedure tpointerconstnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        typedefderef.build(typedef);
      end;


    procedure tpointerconstnode.derefimpl;
      begin
        inherited derefimpl;
        typedef:=tdef(typedefderef.resolve);
      end;


    function tpointerconstnode.dogetcopy : tnode;

      var
         n : tpointerconstnode;

      begin
         n:=tpointerconstnode(inherited dogetcopy);
         n.value:=value;
         n.typedef := typedef;
         dogetcopy:=n;
      end;

    function tpointerconstnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=typedef;
      end;

    function tpointerconstnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_CONSTANT;
      end;

    function tpointerconstnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (value = tpointerconstnode(p).value);
      end;


    procedure tpointerconstnode.printnodedata(var t : text);
      begin
        inherited printnodedata(t);
        writeln(t,printnodeindention,'value = $',hexstr(PUInt(value),sizeof(PUInt)*2));
      end;

    function tpointerconstnode.emit_data(tcb: ttai_typedconstbuilder): sizeint;
      begin
        if tpointerdef(resultdef).compatible_with_pointerdef_size(tpointerdef(voidpointertype)) then
          tcb.emit_tai(tai_const.Create_int_dataptr(value),voidpointertype)
        else
          tcb.emit_tai(tai_const.Create_int_codeptr(value),voidcodepointertype);
        result:=resultdef.size;
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TPointerConstNode.XMLPrintNodeData(var T: Text);
      begin
        inherited XMLPrintNodeData(T);
        WriteLn(T, PrintNodeIndention, '<value>$', hexstr(PUInt(value),sizeof(PUInt)*2), '</value>');
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                             TSTRINGCONSTNODE
*****************************************************************************}

    constructor tstringconstnode.createstr(const s : string);
      var
         l : longint;
      begin
         inherited create(stringconstn);
         l:=length(s);
         len:=l;
         { stringdup write even past a #0 }
         getmem(value_str,l+1);
         move(s[1],value_str^,l);
         value_str[l]:=#0;
         lab_str:=nil;
         cst_type:=cst_conststring;
      end;


    constructor tstringconstnode.createunistr(w : pcompilerwidestring);
      begin
         inherited create(stringconstn);
         len:=getlengthwidestring(w);
         initwidestring(pcompilerwidestring(value_str));
         copywidestring(w,pcompilerwidestring(value_str));
         lab_str:=nil;
         cst_type:=cst_unicodestring;
      end;


    constructor tstringconstnode.createpchar(s: pchar; l: longint; def: tdef);
      begin
         inherited create(stringconstn);
         len:=l;
         value_str:=s;
         if assigned(def) and
            is_ansistring(def) then
           begin
             cst_type:=cst_ansistring;
             astringdef:=def;
           end
         else
           cst_type:=cst_conststring;
         lab_str:=nil;
      end;


    destructor tstringconstnode.destroy;
      begin
        if cst_type in [cst_widestring,cst_unicodestring] then
          donewidestring(pcompilerwidestring(value_str))
        else
          ansistringdispose(value_str,len);
        inherited destroy;
      end;


    constructor tstringconstnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      var
        pw : pcompilerwidestring;
        i : longint;
      begin
        inherited ppuload(t,ppufile);
        cst_type:=tconststringtype(ppufile.getbyte);
        len:=ppufile.getlongint;
        if cst_type in [cst_widestring,cst_unicodestring] then
          begin
            initwidestring(pw);
            setlengthwidestring(pw,len);
            { don't use getdata, because the compilerwidechars may have to
              be byteswapped
            }
{$if sizeof(tcompilerwidechar) = 2}
            for i:=0 to pw^.len-1 do
              pw^.data[i]:=ppufile.getword;
{$elseif sizeof(tcompilerwidechar) = 4}
            for i:=0 to pw^.len-1 do
              pw^.data[i]:=cardinal(ppufile.getlongint);
{$else}
           {$error Unsupported tcompilerwidechar size}
{$endif}
            pcompilerwidestring(value_str):=pw
          end
        else
          begin
            getmem(value_str,len+1);
            ppufile.getdata(value_str^,len);
            value_str[len]:=#0;
          end;
        lab_str:=tasmlabel(ppufile.getasmsymbol);
        if cst_type=cst_ansistring then
          ppufile.getderef(astringdefderef);
      end;


    procedure tstringconstnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putbyte(byte(cst_type));
        ppufile.putlongint(len);
        if cst_type in [cst_widestring,cst_unicodestring] then
          ppufile.putdata(pcompilerwidestring(value_str)^.data^,len*sizeof(tcompilerwidechar))
        else
          ppufile.putdata(value_str^,len);
        ppufile.putasmsymbol(lab_str);
        if cst_type=cst_ansistring then
          ppufile.putderef(astringdefderef);
      end;


    procedure tstringconstnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        if cst_type=cst_ansistring then
          astringdefderef.build(astringdef);
      end;


    procedure tstringconstnode.derefimpl;
      begin
        inherited derefimpl;
        if cst_type=cst_ansistring then
          astringdef:=tdef(astringdefderef.resolve);
      end;


    function tstringconstnode.dogetcopy : tnode;

      var
         n : tstringconstnode;

      begin
         n:=tstringconstnode(inherited dogetcopy);
         n.cst_type:=cst_type;
         n.len:=len;
         n.lab_str:=lab_str;
         if cst_type in [cst_widestring,cst_unicodestring] then
           begin
             initwidestring(pcompilerwidestring(n.value_str));
             copywidestring(pcompilerwidestring(value_str),pcompilerwidestring(n.value_str));
           end
         else
           n.value_str:=getpcharcopy;
         n.astringdef:=astringdef;
         dogetcopy:=n;
      end;

    function tstringconstnode.pass_typecheck:tnode;
      var
        l : aint;
      begin
        result:=nil;
        case cst_type of
          cst_conststring :
            begin
              { handle and store as array[0..len-1] of char }
              if len>0 then
                l:=len-1
              else
                l:=0;
              resultdef:=carraydef.create(0,l,s32inttype);
              tarraydef(resultdef).elementdef:=cansichartype;
              include(tarraydef(resultdef).arrayoptions,ado_IsConstString);
            end;
          cst_shortstring :
            resultdef:=cshortstringtype;
          cst_ansistring :
            if not assigned(astringdef) then
              resultdef:=getansistringdef
            else
              resultdef:=astringdef;
          cst_unicodestring :
            resultdef:=cunicodestringtype;
          cst_widestring :
            resultdef:=cwidestringtype;
          cst_longstring :
            resultdef:=clongstringtype;
        end;
      end;

    function tstringconstnode.pass_1 : tnode;
      begin
        result:=nil;
        if (cst_type in [cst_ansistring,cst_widestring,cst_unicodestring]) then
          begin
            if len=0 then
              expectloc:=LOC_CONSTANT
            else
              expectloc:=LOC_REGISTER
          end
        else
          expectloc:=LOC_CREFERENCE;
      end;


    function tstringconstnode.getpcharcopy : pchar;
      var
         pc : pchar;
      begin
         pc:=nil;
         getmem(pc,len+1);
         if pc=nil then
           Message(general_f_no_memory_left);
         move(value_str^,pc^,len+1);
         getpcharcopy:=pc;
      end;

    function tstringconstnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (len = tstringconstnode(p).len) and
          (lab_str = tstringconstnode(p).lab_str) and
          { This is enough as soon as labels are allocated, otherwise }
          { fall back to content compare.                             }
          (assigned(lab_str) or
            (cst_type = tstringconstnode(p).cst_type) and
            (fullcompare(tstringconstnode(p)) = 0))
          ;
      end;


    procedure tstringconstnode.changestringtype(def:tdef);
      const
        st2cst : array[tstringtype] of tconststringtype = (
          cst_shortstring,cst_longstring,cst_ansistring,cst_widestring,cst_unicodestring);
      var
        pw : pcompilerwidestring;
        pc : pchar;
        cp1 : tstringencoding;
        cp2 : tstringencoding;
        l,l2 : longint;
      begin
        if def.typ<>stringdef then
          internalerror(200510011);
        { convert ascii 2 unicode }
        if (tstringdef(def).stringtype in [st_widestring,st_unicodestring]) and
           not(cst_type in [cst_widestring,cst_unicodestring]) then
          begin
            initwidestring(pw);
            ascii2unicode(value_str,len,current_settings.sourcecodepage,pw);
            ansistringdispose(value_str,len);
            pcompilerwidestring(value_str):=pw;
          end
        else
          { convert unicode 2 ascii }
          if (cst_type in [cst_widestring,cst_unicodestring]) and
            not(tstringdef(def).stringtype in [st_widestring,st_unicodestring]) then
            begin
              cp1:=tstringdef(def).encoding;
              if (cp1=globals.CP_NONE) or (cp1=0) then
                cp1:=current_settings.sourcecodepage;
              if (cp1=CP_UTF8) then
                begin
                  pw:=pcompilerwidestring(value_str);
                  l2:=len;
                  l:=UnicodeToUtf8(nil,0,PUnicodeChar(pw^.data),l2);
                  getmem(pc,l);
                  UnicodeToUtf8(pc,l,PUnicodeChar(pw^.data),l2);
                  len:=l-1;
                  donewidestring(pw);
                  value_str:=pc;
                end
              else
                begin
                  pw:=pcompilerwidestring(value_str);
                  getmem(pc,getlengthwidestring(pw)+1);
                  unicode2ascii(pw,pc,cp1);
                  donewidestring(pw);
                  value_str:=pc;
                end;
            end
        else
          if (tstringdef(def).stringtype = st_ansistring) and
             not(cst_type in [cst_widestring,cst_unicodestring]) then
            begin
              cp1:=tstringdef(def).encoding;
              if cp1=0 then
                cp1:=current_settings.sourcecodepage;
              if (cst_type = cst_ansistring) then
                begin
                  cp2:=tstringdef(resultdef).encoding;
                  if cp2=0 then
                    cp2:=current_settings.sourcecodepage;
                end
              else if (cst_type in [cst_shortstring,cst_conststring,cst_longstring]) then
                cp2:=current_settings.sourcecodepage
              else
                internalerror(2013112916);
              { don't change string if codepages are equal or string length is 0 }
              if (cp1<>cp2) and (len>0) then
                begin
                  if cpavailable(cp1) and cpavailable(cp2) then
                    changecodepage(value_str,len,cp2,value_str,cp1)
                  else if (cp1 <> globals.CP_NONE) and (cp2 <> globals.CP_NONE) then
                    begin
                      { if source encoding is UTF8 convert using UTF8->UTF16->destination encoding }
                      if (cp2=CP_UTF8) then
                        begin
                          if not cpavailable(cp1) then
                            Message1(option_code_page_not_available,IntToStr(cp1));
                          initwidestring(pw);
                          setlengthwidestring(pw,len);
                          { returns room for terminating 0 }
                          l:=Utf8ToUnicode(PUnicodeChar(pw^.data),len,value_str,len);
                          if (l<>getlengthwidestring(pw)) then
                            begin
                              setlengthwidestring(pw,l);
                              ReAllocMem(value_str,l);
                            end;
                          unicode2ascii(pw,value_str,cp1);
                          len:=l-1;
                          donewidestring(pw);
                        end
                      else
                      { if destination encoding is UTF8 convert using source encoding->UTF16->UTF8 }
                      if (cp1=CP_UTF8) then
                        begin
                          if not cpavailable(cp2) then
                            Message1(option_code_page_not_available,IntToStr(cp2));
                          initwidestring(pw);
                          setlengthwidestring(pw,len);
                          ascii2unicode(value_str,len,cp2,pw);
                          { returns room for terminating 0 }
                          l:=UnicodeToUtf8(nil,0,PUnicodeChar(pw^.data),len);
                          if l<>len then
                            ReAllocMem(value_str,l);
                          len:=l-1;
                          UnicodeToUtf8(value_str,PUnicodeChar(pw^.data),l);
                          donewidestring(pw);
                        end
                      else
                        begin
                          { output error message that encoding is not available for the compiler }
                          if not cpavailable(cp1) then
                            Message1(option_code_page_not_available,IntToStr(cp1));
                          if not cpavailable(cp2) then
                            Message1(option_code_page_not_available,IntToStr(cp2));
                        end;
                    end;
                end;
            end;
        cst_type:=st2cst[tstringdef(def).stringtype];
        resultdef:=def;
      end;

    function tstringconstnode.fullcompare(p: tstringconstnode): longint;
      begin
        if cst_type<>p.cst_type then
          InternalError(2009121701);
        if cst_type in [cst_widestring,cst_unicodestring] then
          result:=comparewidestrings(pcompilerwidestring(value_str),pcompilerwidestring(p.value_str))
        else
          result:=compareansistrings(value_str,p.value_str,len,p.len);
      end;

    function tstringconstnode.emit_data(tcb:ttai_typedconstbuilder):sizeint;
      var
        ss : shortstring;
        labofs : tasmlabofs;
        winlikewidestring : boolean;
      begin
        case tstringdef(resultdef).stringtype of
          st_shortstring:
            begin
              setlength(ss,len);
              move(value_str^,ss[1],len);
              tcb.emit_shortstring_const(ss);
              result:=len+1;
            end;
          st_longstring:
            internalerror(2019070801);
          st_ansistring:
            begin
              labofs:=tcb.emit_ansistring_const(current_asmdata.asmlists[al_typedconsts],value_str,len,tstringdef(resultdef).encoding);
              tcb.emit_string_offset(labofs,len,tstringdef(resultdef).stringtype,false,charpointertype);
              result:=voidpointertype.size;
            end;
          st_widestring,
          st_unicodestring:
            begin
              winlikewidestring:=(cst_type=cst_widestring) and (tf_winlikewidestring in target_info.flags);
              labofs:=tcb.emit_unicodestring_const(current_asmdata.asmlists[al_typedconsts],value_str,tstringdef(resultdef).encoding,winlikewidestring);
              tcb.emit_string_offset(labofs,len,tstringdef(resultdef).stringtype,false,widecharpointertype);
              result:=voidpointertype.size;
            end;
        end;
      end;

    class function tstringconstnode.emptydynstrnil: boolean;
      begin
        result:=true;
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TStringConstNode.XMLPrintNodeData(var T: Text);
      var
        OutputStr: ansistring;
      begin
        inherited XMLPrintNodeData(T);
        Write(T, printnodeindention, '<stringtype>');
        case cst_type of
        cst_conststring:
          Write(T, 'conststring');
        cst_shortstring:
          Write(T, 'shortstring');
        cst_longstring:
          Write(T, 'longstring');
        cst_ansistring:
          Write(T, 'ansistring');
        cst_widestring:
          Write(T, 'widestring');
        cst_unicodestring:
          Write(T, 'unicodestring');
        end;
        WriteLn(T, '</stringtype>');
        WriteLn(T, printnodeindention, '<length>', len, '</length>');

        if len = 0 then
          begin
            WriteLn(T, printnodeindention, '<value />');
            Exit;
          end;

        case cst_type of
        cst_widestring, cst_unicodestring:
          begin
            { value_str is of type PCompilerWideString }
            SetLength(OutputStr, len);
            UnicodeToUtf8(PChar(OutputStr), PUnicodeChar(PCompilerWideString(value_str)^.data), len + 1); { +1 for the null terminator }
          end;
        else
          OutputStr := ansistring(value_str);
          SetLength(OutputStr, len);
        end;

        WriteLn(T, printnodeindention, '<value>', SanitiseXMLString(OutputStr), '</value>');
      end;
{$endif DEBUG_NODE_XML}

{*****************************************************************************
                             TSETCONSTNODE
*****************************************************************************}

    constructor tsetconstnode.create(s : pconstset;def:tdef);

      begin
         inherited create(setconstn,nil);
         typedef:=def;
         if assigned(s) then
           begin
              new(value_set);
              value_set^:=s^;
           end
         else
           value_set:=nil;
      end;


    destructor tsetconstnode.destroy;
      begin
        if assigned(value_set) then
         dispose(value_set);
        inherited destroy;
      end;


    constructor tsetconstnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getderef(typedefderef);
        new(value_set);
        ppufile.getset(tppuset32(value_set^));
      end;


    procedure tsetconstnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(typedefderef);
        ppufile.putset(tppuset32(value_set^));
      end;


    procedure tsetconstnode.buildderefimpl;
      begin
        inherited buildderefimpl;
        typedefderef.build(typedef);
      end;


    procedure tsetconstnode.derefimpl;
      begin
        inherited derefimpl;
        typedef:=tdef(typedefderef.resolve);
      end;

    type
       setbytes = array[0..31] of byte;
       Psetbytes = ^setbytes;

    procedure tsetconstnode.adjustforsetbase;
      var
        i, diff: longint;
      begin
        { Internally, the compiler stores all sets with setbase 0, so we have }
        { to convert the set to its actual format in case setbase<>0 when     }
        { writing it out                                                      }
        if (tsetdef(resultdef).setbase<>0) then
          begin
            if (tsetdef(resultdef).setbase and 7)<>0 then
              internalerror(2007091501);
            diff:=tsetdef(resultdef).setbase div 8;
            { This is endian-neutral in the new set format: in both cases, }
            { the first byte contains the first elements of the set.       }
            { Since the compiler/base rtl cannot contain packed sets before }
            { they work for big endian, it's no problem that the code below }
            { is wrong for the old big endian set format (setbase cannot be }
            { <>0 with non-packed sets).                                    }
            for i:=0 to tsetdef(resultdef).size-1 do
              begin
                Psetbytes(value_set)^[i]:=Psetbytes(value_set)^[i+diff];
                Psetbytes(value_set)^[i+diff]:=0;
              end;
          end;
      end;


    function tsetconstnode.dogetcopy : tnode;
      var
         n : tsetconstnode;
      begin
         n:=tsetconstnode(inherited dogetcopy);
         if assigned(value_set) then
           begin
              new(n.value_set);
              n.value_set^:=value_set^
           end
         else
           n.value_set:=nil;
         n.typedef := typedef;
         n.lab_set:=lab_set;
         dogetcopy:=n;
      end;


    function tsetconstnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=typedef;
      end;


    function tsetconstnode.pass_1 : tnode;
      begin
         result:=nil;
         if is_smallset(resultdef) then
          expectloc:=LOC_CONSTANT
         else
          expectloc:=LOC_CREFERENCE;
      end;


    function tsetconstnode.docompare(p: tnode): boolean;
      begin
        docompare:=(inherited docompare(p)) and
                   (value_set^=Tsetconstnode(p).value_set^);
      end;


    function tsetconstnode.elements : AInt;
      var
        i : longint;
      begin
        result:=0;
        if not(assigned(value_set)) then
          exit;
        for i:=0 to tsetdef(resultdef).size-1 do
          result:=result+ PopCnt(Psetbytes(value_set)^[i]);
      end;

    function tsetconstnode.emit_data(tcb:ttai_typedconstbuilder):sizeint;
      type
        setbytes=array[0..31] of byte;
        Psetbytes=^setbytes;
      var
        setval : aint;
        i : sizeint;
      begin
        if is_smallset(resultdef) then
          begin
            if (source_info.endian=target_info.endian) then
              begin
                { not plongint, because that will "sign extend" the set on 64 bit platforms }
                { if changed to "paword", please also modify "32-resultdef.size*8" and      }
                { cross-endian code below                                                   }
                { Extra aint type cast to avoid range errors                                }
                 setval:=aint(pCardinal(value_set)^)
              end
            else
              begin
                setval:=aint(swapendian(Pcardinal(value_set)^));
                setval:=aint(
                                   reverse_byte (setval         and $ff)         or
                                  (reverse_byte((setval shr  8) and $ff) shl  8) or
                                  (reverse_byte((setval shr 16) and $ff) shl 16) or
                                  (reverse_byte((setval shr 24) and $ff) shl 24)
                                );
              end;
            if (target_info.endian=endian_big) then
              setval:=setval shr (32-resultdef.size*8);
            case resultdef.size of
              1:
                tcb.emit_ord_const(byte(setval),u8inttype);
              2:
                tcb.emit_ord_const(word(setval),u16inttype);
              4:
                tcb.emit_ord_const(longword(setval),u32inttype);
              8:
                tcb.emit_ord_const(qword(setval),u64inttype);
              else
                internalerror(2019070802);
            end;
          end
        else
          begin
            if (source_info.endian=target_info.endian) then
              for i:=0 to resultdef.size-1 do
                tcb.emit_tai(tai_const.create_8bit(Psetbytes(value_set)^[i]),u8inttype)
            else
              for i:=0 to resultdef.size-1 do
                tcb.emit_tai(tai_const.create_8bit(reverse_byte(Psetbytes(value_set)^[i])),u8inttype);
          end;
        result:=resultdef.size;
      end;


{*****************************************************************************
                               TNILNODE
*****************************************************************************}

    constructor tnilnode.create;

      begin
        inherited create(niln);
      end;

    function tnilnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidpointertype;
      end;

    function tnilnode.emit_data(tcb: ttai_typedconstbuilder): sizeint;
      begin
        if tpointerdef(resultdef).compatible_with_pointerdef_size(tpointerdef(voidpointertype)) then
          tcb.emit_tai(tai_const.Create_nil_dataptr,voidpointertype)
        else
          tcb.emit_tai(tai_const.Create_nil_codeptr,voidcodepointertype);
        result:=resultdef.size;
      end;

    function tnilnode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_CONSTANT;
      end;

{*****************************************************************************
                            TGUIDCONSTNODE
*****************************************************************************}

    constructor tguidconstnode.create(const g:tguid);

      begin
         inherited create(guidconstn);
         value:=g;
      end;

    constructor tguidconstnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      begin
        inherited ppuload(t,ppufile);
        ppufile.getguid(value);
      end;


    procedure tguidconstnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putguid(value);
      end;


    function tguidconstnode.dogetcopy : tnode;
      var
         n : tguidconstnode;
      begin
         n:=tguidconstnode(inherited dogetcopy);
         n.value:=value;
         n.lab_set:=lab_set;
         dogetcopy:=n;
      end;


    function tguidconstnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=rec_tguid;
      end;


    function tguidconstnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_CREFERENCE;
      end;


    function tguidconstnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (guid2string(value) = guid2string(tguidconstnode(p).value));
      end;

    function tguidconstnode.emit_data(tcb: ttai_typedconstbuilder): sizeint;
      begin
        tcb.emit_guid_const(value);
        result:=resultdef.size;
      end;

end.
