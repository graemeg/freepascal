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
      cclasses,
      node,
      aasmbase,aasmtai,aasmdata,cpuinfo,globals,
      symconst,symtype,symdef,symsym;

    type
       tdataconstnode = class(tnode)
         data : tdynamicarray;
         maxalign : word;
         constructor create;virtual;
         constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
         destructor destroy;override;
         procedure ppuwrite(ppufile:tcompilerppufile);override;
         function dogetcopy : tnode;override;
         function pass_1 : tnode;override;
         function pass_typecheck:tnode;override;
         function docompare(p: tnode) : boolean; override;
         procedure printnodedata(var t:text);override;
         procedure append(const d;len : aint);
         procedure appendbyte(b : byte);
         procedure align(value : word);
       end;
       tdataconstnodeclass = class of tdataconstnode;

       trealconstnode = class(tnode)
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
       end;
       trealconstnodeclass = class of trealconstnode;

       tordconstnode = class(tnode)
          typedef : tdef;
          typedefderef : tderef;
          value : TConstExprInt;
          rangecheck : boolean;
          { create an ordinal constant node of the specified type and value.
            _rangecheck determines if the value of the ordinal should be checked
            against the ranges of the type definition.
          }
          constructor create(v : tconstexprint;def:tdef; _rangecheck : boolean);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderefimpl;override;
          procedure derefimpl;override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode) : boolean; override;
          procedure printnodedata(var t:text);override;
       end;
       tordconstnodeclass = class of tordconstnode;

       tpointerconstnode = class(tnode)
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

       tstringconstnode = class(tnode)
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
       end;
       tsetconstnodeclass = class of tsetconstnode;

       tnilnode = class(tnode)
          constructor create;virtual;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
       end;
       tnilnodeclass = class of tnilnode;

       tguidconstnode = class(tnode)
          value : tguid;
          lab_set : tasmsymbol;
          constructor create(const g:tguid);virtual;
          constructor ppuload(t:tnodetype;ppufile:tcompilerppufile);override;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          function dogetcopy : tnode;override;
          function pass_1 : tnode;override;
          function pass_typecheck:tnode;override;
          function docompare(p: tnode) : boolean; override;
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
       cdataconstnode : tdataconstnodeclass = tdataconstnode;

    function genintconstnode(v : TConstExprInt) : tordconstnode;
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
      verbose,systems,sysutils,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      defcmp,defutil,procinfo,
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      defutil,
>>>>>>> graemeg/cpstrnew
      cpubase,cgbase,
      nld;

    function genintconstnode(v : TConstExprInt) : tordconstnode;
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            result:=cstringconstnode.createunistr(pWideStringVal);
=======
            result:=cstringconstnode.createwstr(pWideStringVal);
>>>>>>> graemeg/cpstrnew
=======
            result:=cstringconstnode.createwstr(pWideStringVal);
>>>>>>> graemeg/cpstrnew
=======
            result:=cstringconstnode.createwstr(pWideStringVal);
>>>>>>> graemeg/cpstrnew
=======
            result:=cstringconstnode.createwstr(pWideStringVal);
>>>>>>> origin/cpstrnew
          end
        else if is_conststringnode(p) then
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
              if not(cs_refcountedstrings in current_settings.localswitches) and (len>255) then
=======
              if not(cs_ansistrings in current_settings.localswitches) and (len>255) then
>>>>>>> graemeg/cpstrnew
=======
              if not(cs_ansistrings in current_settings.localswitches) and (len>255) then
>>>>>>> graemeg/cpstrnew
=======
              if not(cs_ansistrings in current_settings.localswitches) and (len>255) then
>>>>>>> graemeg/cpstrnew
=======
              if not(cs_ansistrings in current_settings.localswitches) and (len>255) then
>>>>>>> origin/cpstrnew
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            p1:=cstringconstnode.createunistr(pcompilerwidestring(p.value.valueptr));
=======
            p1:=cstringconstnode.createwstr(pcompilerwidestring(p.value.valueptr));
>>>>>>> graemeg/cpstrnew
=======
            p1:=cstringconstnode.createwstr(pcompilerwidestring(p.value.valueptr));
>>>>>>> graemeg/cpstrnew
=======
            p1:=cstringconstnode.createwstr(pcompilerwidestring(p.value.valueptr));
>>>>>>> graemeg/cpstrnew
=======
            p1:=cstringconstnode.createwstr(pcompilerwidestring(p.value.valueptr));
>>>>>>> origin/cpstrnew
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
                             TDATACONSTNODE
*****************************************************************************}

    constructor tdataconstnode.create;
      begin
         inherited create(dataconstn);
         data:=tdynamicarray.create(128);
      end;


    constructor tdataconstnode.ppuload(t:tnodetype;ppufile:tcompilerppufile);
      var
        len : tcgint;
        buf : array[0..255] of byte;
      begin
        inherited ppuload(t,ppufile);
        len:=ppufile.getaint;
        if len<4096 then
          data:=tdynamicarray.create(len)
        else
          data:=tdynamicarray.create(4096);
        while len>0 do
          begin
            if len>sizeof(buf) then
              begin
                ppufile.getdata(buf,sizeof(buf));
                data.write(buf,sizeof(buf));
                dec(len,sizeof(buf));
              end
            else
              begin
                ppufile.getdata(buf,len);
                data.write(buf,len);
                len:=0;
              end;
          end;
      end;


    destructor tdataconstnode.destroy;
      begin
        data.free;
        inherited destroy;
      end;


    procedure tdataconstnode.ppuwrite(ppufile:tcompilerppufile);
      var
        len : tcgint;
        buf : array[0..255] of byte;
      begin
        inherited ppuwrite(ppufile);
        len:=data.size;
        ppufile.putaint(len);
        data.seek(0);
        while len>0 do
          begin
            if len>sizeof(buf) then
              begin
                data.read(buf,sizeof(buf));
                ppufile.putdata(buf,sizeof(buf));
                dec(len,sizeof(buf));
              end
            else
              begin
                data.read(buf,len);
                ppufile.putdata(buf,len);
                len:=0;
              end;
          end;
      end;


    function tdataconstnode.dogetcopy : tnode;
      var
        n : tdataconstnode;
        len : tcgint;
        buf : array[0..255] of byte;
      begin
        n:=tdataconstnode(inherited dogetcopy);
        len:=data.size;
        if len<4096 then
          n.data:=tdynamicarray.create(len)
        else
          n.data:=tdynamicarray.create(4096);
        data.seek(0);
        while len>0 do
          begin
            if len>sizeof(buf) then
              begin
                data.read(buf,sizeof(buf));
                n.data.write(buf,sizeof(buf));
                dec(len,sizeof(buf));
              end
            else
              begin
                data.read(buf,len);
                n.data.write(buf,len);
                len:=0;
              end;
          end;
          dogetcopy := n;
      end;


    function tdataconstnode.pass_1 : tnode;
      begin
        result:=nil;
        expectloc:=LOC_CREFERENCE;
      end;


    function tdataconstnode.pass_typecheck:tnode;
      begin
        result:=nil;
        resultdef:=voidpointertype;
      end;


    function tdataconstnode.docompare(p: tnode) : boolean;
      var
        b1,b2 : byte;
        I : longint;
      begin
        docompare :=
          inherited docompare(p) and (data.size=tdataconstnode(p).data.size);
        if docompare then
          begin
            data.seek(0);
            tdataconstnode(p).data.seek(0);
            for i:=0 to data.size-1 do
              begin
                data.read(b1,1);
                tdataconstnode(p).data.read(b2,1);
                if b1<>b2 then
                  begin
                    docompare:=false;
                    exit;
                  end;
              end;
          end;
      end;


    procedure tdataconstnode.printnodedata(var t:text);
      var
        i : longint;
        b : byte;
      begin
        inherited printnodedata(t);
        write(t,printnodeindention,'data size = ',data.size,' data = ');
        data.seek(0);
        for i:=0 to data.size-1 do
          begin
            data.read(b,1);
            if i=data.size-1 then
              writeln(t,b)
            else
              write(t,b,',');
          end;
      end;


    procedure tdataconstnode.append(const d;len : aint);
      begin
        data.seek(data.size);
        data.write(d,len);
      end;

    procedure tdataconstnode.appendbyte(b : byte);
      begin
        data.seek(data.size);
        data.write(b,1);
      end;

    procedure tdataconstnode.align(value : word);
      begin
        if value>maxalign then
          maxalign:=value;
        data.align(value);
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
      end;

    function trealconstnode.pass_1 : tnode;
      begin
         result:=nil;
         expectloc:=LOC_CREFERENCE;
         if (cs_create_pic in current_settings.moduleswitches) then
           include(current_procinfo.flags,pi_needs_got);
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


    procedure Trealconstnode.printnodedata(var t:text);
      begin
        inherited printnodedata(t);
        writeln(t,printnodeindention,'value = ',value_real);
      end;


{*****************************************************************************
                              TORDCONSTNODE
*****************************************************************************}

    constructor tordconstnode.create(v : tconstexprint;def:tdef;_rangecheck : boolean);

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
<<<<<<< HEAD
<<<<<<< HEAD
        if (resultdef.typ in [orddef,enumdef]) then
           testrange(resultdef,value,not rangecheck,false)
=======
        if rangecheck and (resultdef.typ in [orddef,enumdef]) then
           testrange(resultdef,resultdef,value,false);
>>>>>>> graemeg/fixes_2_2
=======
        if rangecheck and (resultdef.typ in [orddef,enumdef]) then
           testrange(resultdef,resultdef,value,false);
>>>>>>> origin/fixes_2_2
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


    procedure Tordconstnode.printnodedata(var t:text);
      begin
        inherited printnodedata(t);
        writeln(t,printnodeindention,'value = ',tostr(value));
      end;


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
        if (cs_create_pic in current_settings.moduleswitches) and
           (expectloc <> LOC_CONSTANT) then
          include(current_procinfo.flags,pi_needs_got);
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              if (tstringdef(def).encoding=CP_UTF8) or
                 (current_settings.sourcecodepage='utf8') then
                begin
                  pw:=pcompilerwidestring(value_str);
                  l:=(getlengthwidestring(pw)*4)+1;
                  getmem(pc,l);   
                  l2:=UnicodeToUtf8(pc,l,PUnicodeChar(pw^.data),getlengthwidestring(pw));
                  if (l<>l2) then
                    ReAllocMem(pc,l2);
                  len:=l2-1;
                  donewidestring(pw);
                  value_str:=pc;
                  if (tstringdef(def).encoding<>CP_UTF8) then
                    tstringdef(def).encoding:=CP_UTF8;
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
                end
              else
                begin
                  pw:=pcompilerwidestring(value_str);
                  getmem(pc,getlengthwidestring(pw)+1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                  unicode2ascii(pw,pc,cp1);
=======
                  unicode2ascii(pw,pc,tstringdef(def).encoding);
>>>>>>> graemeg/cpstrnew
=======
                  unicode2ascii(pw,pc,tstringdef(def).encoding);
>>>>>>> graemeg/cpstrnew
=======
                  unicode2ascii(pw,pc,tstringdef(def).encoding);
>>>>>>> graemeg/cpstrnew
=======
                  unicode2ascii(pw,pc,tstringdef(def).encoding);
>>>>>>> origin/cpstrnew
                  donewidestring(pw);
                  value_str:=pc;
                end;
            end
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        else
=======
        else 
>>>>>>> graemeg/cpstrnew
=======
        else 
>>>>>>> graemeg/cpstrnew
=======
        else 
>>>>>>> graemeg/cpstrnew
=======
        else 
>>>>>>> origin/cpstrnew
          if (tstringdef(def).stringtype = st_ansistring) and
             not(cst_type in [cst_widestring,cst_unicodestring]) then
            begin
              cp1:=tstringdef(def).encoding;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
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
                          l:=Utf8ToUnicode(PUnicodeChar(pw^.data),len,value_str,len);
                          if (l<>getlengthwidestring(pw)) then
                            begin
                              setlengthwidestring(pw,l);
                              ReAllocMem(value_str,l);
                            end;
                          unicode2ascii(pw,value_str,cp1);
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
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              if (cst_type = cst_ansistring) then
                cp2:=tstringdef(resultdef).encoding
              else if (cst_type in [cst_shortstring,cst_conststring,cst_longstring]) then
                cp2:=codepagebyname(current_settings.sourcecodepage);
              if cpavailable(cp1) and cpavailable(cp2) then
                changecodepage(value_str,len,cp1,value_str,cp2);
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
        ppufile.getnormalset(value_set^);
      end;


    procedure tsetconstnode.ppuwrite(ppufile:tcompilerppufile);
      begin
        inherited ppuwrite(ppufile);
        ppufile.putderef(typedefderef);
        ppufile.putnormalset(value_set^);
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


    procedure tsetconstnode.adjustforsetbase;
      type
         setbytes = array[0..31] of byte;
         Psetbytes = ^setbytes;
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2

    

    function tsetconstnode.dogetcopy : tnode;
>>>>>>> graemeg/fixes_2_2


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
        if (cs_create_pic in current_settings.moduleswitches) and
           (expectloc <> LOC_CONSTANT) then
          include(current_procinfo.flags,pi_needs_got);
      end;


    function tsetconstnode.docompare(p: tnode): boolean;
      begin
        docompare:=(inherited docompare(p)) and
                   (value_set^=Tsetconstnode(p).value_set^);
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
        if (cs_create_pic in current_settings.moduleswitches) and
          (tf_pic_uses_got in target_info.flags) then
          include(current_procinfo.flags,pi_needs_got);
      end;


    function tguidconstnode.docompare(p: tnode): boolean;
      begin
        docompare :=
          inherited docompare(p) and
          (guid2string(value) = guid2string(tguidconstnode(p).value));
      end;

end.
