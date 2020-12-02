{
    Copyright (c) 1998-2002 by Florian Klaempfl, Pierre Muller

    Implementation for the symbols types of the symtable

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
unit symsym;

{$i fpcdefs.inc}

interface

    uses
       { common }
       cutils,compinnr,
       { target }
       globtype,globals,widestr,constexp,
       { symtable }
       symconst,symbase,symtype,symdef,defcmp,
       cclasses,
       { aasm }
       aasmbase,
       cpuinfo,cgbase,cgutils,parabase
       ;

    type
       { this class is the base for all symbol objects }
       tstoredsym = class(tsym)
       private
          procedure writeentry(ppufile: tcompilerppufile; ibnr: byte);
       protected
          procedure ppuwrite_platform(ppufile: tcompilerppufile);virtual;
          procedure ppuload_platform(ppufile: tcompilerppufile);virtual;
       public
          { this is Nil if the symbol has no RTTI attributes }
          rtti_attribute_list : trtti_attribute_list;
          constructor create(st:tsymtyp;const n : TSymStr);
          constructor ppuload(st:tsymtyp;ppufile:tcompilerppufile);
          destructor destroy;override;
          procedure ppuwrite(ppufile:tcompilerppufile);virtual;
          { this is called directly after ppuload }
          procedure ppuload_subentries(ppufile:tcompilerppufile);virtual;
          { this is called directly after ppuwrite }
          procedure ppuwrite_subentries(ppufile:tcompilerppufile);virtual;
          procedure deref; override;
          procedure buildderef; override;
          procedure register_sym; override;
       end;

       tlabelsym = class(tstoredsym)
          used,
          defined,
          nonlocal : boolean;
          { points to the matching node, only valid resultdef pass is run and
            the goto<->label relation in the node tree is created, should
            be a tnode }
          code : pointer;

          { points to the jump buffer }
          jumpbuf : tstoredsym;

          { when the label is defined in an asm block, this points to the
            generated asmlabel }
          asmblocklabel : tasmlabel;
          constructor create(const n : TSymStr);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function mangledname:TSymStr;override;
       end;
       tlabelsymclass = class of tlabelsym;

       tunitsym = class(Tstoredsym)
          module : tobject; { tmodule }
          constructor create(const n : TSymStr;amodule : tobject);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
       end;
       tunitsymclass = class of tunitsym;

       tprogramparasym = class(Tstoredsym)
          isoindex : dword;
          constructor create(const n : TSymStr;i : dword);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
       end;
       tprogramparasymclass = class of tprogramparasym;

       tnamespacesym = class(Tstoredsym)
          unitsym:tsym;
          unitsymderef:tderef;
          constructor create(const n : TSymStr);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
       end;
       tnamespacesymclass = class of tnamespacesym;

       terrorsym = class(Tsym)
          constructor create;
          procedure register_sym; override;
       end;

       { tprocsym }

       tprocsym = class(tstoredsym)
       protected
          FProcdefList   : TFPObjectList;
          FProcdefDerefList : TFPList;
       public
          constructor create(const n : TSymStr);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy;override;
          { writes all declarations except the specified one }
          procedure write_parameter_lists(skipdef:tprocdef);
          { tests, if all procedures definitions are defined and not }
          { only forward                                             }
          procedure check_forward; virtual;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function find_procdef_bytype(pt:Tproctypeoption):Tprocdef;
          function find_bytype_parameterless(pt:Tproctypeoption):Tprocdef;
          function find_procdef_bypara(para:TFPObjectList;retdef:tdef;cpoptions:tcompare_paras_options):Tprocdef;
          function find_procdef_bytype_and_para(pt:Tproctypeoption;para:TFPObjectList;retdef:tdef;cpoptions:tcompare_paras_options):Tprocdef;
          function find_procdef_byoptions(ops:tprocoptions): Tprocdef;
          function find_procdef_byprocvardef(d:Tprocvardef):Tprocdef;
          function find_procdef_assignment_operator(fromdef,todef:tdef;var besteq:tequaltype;isexplicit:boolean):Tprocdef;
          function find_procdef_enumerator_operator(fromdef,todef:tdef;var besteq:tequaltype):Tprocdef;
          property ProcdefList:TFPObjectList read FProcdefList;
       end;
       tprocsymclass = class of tprocsym;

       ttypesym = class(Tstoredsym)
       public
          typedef      : tdef;
          typedefderef : tderef;
          fprettyname : ansistring;
          constructor create(const n : TSymStr;def:tdef);virtual;
          destructor destroy;override;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function prettyname : string;override;
       end;
       ttypesymclass = class of ttypesym;

       tabstractvarsym = class(tstoredsym)
          varoptions    : tvaroptions;
          varspez       : tvarspez;  { sets the type of access }
          varregable    : tvarregable;
          varstate      : tvarstate;
          {could also be part of tabstractnormalvarsym, but there's
           one byte left here till the next 4 byte alignment        }
          varsymaccess  : tvarsymaccessflags;
          constructor create(st:tsymtyp;const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);
          constructor ppuload(st:tsymtyp;ppufile:tcompilerppufile);
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderef;override;
          procedure deref;override;
          function  getsize : asizeint;
          function  getpackedbitsize : longint;
          function  is_regvar(refpara: boolean):boolean;
        private
          _vardef     : tdef;
          vardefderef : tderef;

          function get_addr_taken: boolean;
          function get_different_scope: boolean;
          procedure setregable;
          procedure setvardef(const def: tdef);
          procedure setvardef_and_regable(def:tdef);
          procedure set_addr_taken(AValue: boolean);
          procedure set_different_scope(AValue: boolean);
        public
          property vardef: tdef read _vardef write setvardef_and_regable;
          property addr_taken: boolean read get_addr_taken write set_addr_taken;
          property different_scope: boolean read get_different_scope write set_different_scope;
      end;

      tfieldvarsym = class(tabstractvarsym)
          { offset in record/object, for bitpacked fields the offset is
            given in bit, else in bytes }
          fieldoffset   : asizeint;
{$ifdef llvm}
          { the llvm version of the record does not support variants,   }
          { so the llvm equivalent field may not be at the exact same   }
          { offset -> store the difference (bits for bitpacked records, }
          { bytes otherwise)                                            }
          offsetfromllvmfield : aint;
          { number of the closest field in the llvm definition }
          llvmfieldnr         : longint;
{$endif llvm}
          externalname  : pshortstring;
{$ifdef symansistr}
          cachedmangledname: TSymStr; { mangled name for ObjC or Java }
{$else symansistr}
          cachedmangledname: pshortstring; { mangled name for ObjC or Java }
{$endif symansistr}
          constructor create(const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure set_externalname(const s:string);virtual;
          function mangledname:TSymStr;override;
          destructor destroy;override;
{$ifdef DEBUG_NODE_XML}
        public
          procedure XMLPrintFieldData(var T: Text);
{$endif DEBUG_NODE_XML}
      end;
      tfieldvarsymclass = class of tfieldvarsym;

      tabstractnormalvarsym = class(tabstractvarsym)
          defaultconstsym : tsym;
          defaultconstsymderef : tderef;
          { register/reference for local var }
          localloc      : TLocation;
          { initial location so it can still be initialized later after the location was changed by SSA }
          initialloc    : TLocation;
          { migrated to a parentfpstruct because of nested access (not written to ppu, because not important and would change interface crc) }
          inparentfpstruct : boolean;
          { the variable is not living at entry of the scope, so it does not need to be initialized if it is a reg. var
            (not written to ppu, because not important and would change interface crc) }
          noregvarinitneeded : boolean;
          constructor create(st:tsymtyp;const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);
          constructor ppuload(st:tsymtyp;ppufile:tcompilerppufile);
          function globalasmsym: boolean;
          procedure ppuwrite(ppufile:tcompilerppufile);override;
          procedure buildderef;override;
          procedure deref;override;
      end;

      tlocalvarsym = class(tabstractnormalvarsym)
          constructor create(const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
      end;
      tlocalvarsymclass = class of tlocalvarsym;

      tparavarsym = class(tabstractnormalvarsym)
          paraloc       : array[callerside..calleeside] of TCGPara;
          paranr        : word; { position of this parameter }
          { in MacPas mode, "univ" parameters mean that type checking should
            be disabled, except that the size of the passed parameter must
            match the size of the formal parameter }
          univpara      : boolean;
{$ifdef EXTDEBUG}
          eqval         : tequaltype;
{$endif EXTDEBUG}
          constructor create(const n : TSymStr;nr:word;vsp:tvarspez;def:tdef;vopts:tvaroptions);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          function needs_finalization: boolean;
          function is_used: boolean;
      end;
      tparavarsymclass = class of tparavarsym;

      tstaticvarsym = class(tabstractnormalvarsym)
      protected
{$ifdef symansistr}
          _mangledbasename,
          _mangledname : TSymStr;
{$else symansistr}
          _mangledbasename,
          _mangledname : pshortstring;
{$endif symansistr}
      public
          section : ansistring;
          { if a text buffer has been defined as being initialized from command line
            parameters as it is done by iso pascal with the program symbols,
            isoindex contains the parameter number }
          isoindex : dword;
          { if this static variable was created based on a class field variable then this is set
            to the symbol of the corresponding class field }
          fieldvarsym : tfieldvarsym;
          fieldvarsymderef : tderef;
          constructor create(const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);virtual;
          constructor create_dll(const n : TSymStr;vsp:tvarspez;def:tdef);virtual;
          constructor create_C(const n: TSymStr; const mangled : TSymStr;vsp:tvarspez;def:tdef);virtual;
          constructor create_from_fieldvar(const n: TSymStr;fieldvar:tfieldvarsym);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor destroy;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function mangledname:TSymStr;override;
          procedure set_mangledbasename(const s: TSymStr);
          function mangledbasename: TSymStr;
          procedure set_mangledname(const s:TSymStr);virtual;
          procedure set_raw_mangledname(const s:TSymStr);
      end;
      tstaticvarsymclass = class of tstaticvarsym;

      tabsolutevarsym = class(tabstractvarsym)
      public
         abstyp  : absolutetyp;
         asmname : pshortstring;
         addroffset : PUint;
         ref     : tpropaccesslist;
         constructor create(const n : TSymStr;def:tdef);virtual;
         constructor create_ref(const n : TSymStr;def:tdef;_ref:tpropaccesslist);virtual;
         destructor  destroy;override;
         constructor ppuload(ppufile:tcompilerppufile);
         procedure buildderef;override;
         procedure deref;override;
         function  mangledname : TSymStr;override;
         { do not override this routine in platform-specific subclasses,
           override ppuwrite_platform instead }
         procedure ppuwrite(ppufile:tcompilerppufile);override;final;
      end;
      tabsolutevarsymclass = class of tabsolutevarsym;

       tpropaccesslisttypes=(palt_none,palt_read,palt_write,palt_stored);

       tpropertysym = class(Tstoredsym)
         protected
           procedure finalize_getter_or_setter_for_sym(getset: tpropaccesslisttypes; sym: tsym; fielddef: tdef; accessordef: tprocdef); virtual;
         public
          propoptions   : tpropertyoptions;
          overriddenpropsym : tpropertysym;
          overriddenpropsymderef : tderef;
          propdef       : tdef;
          propdefderef  : tderef;
          indexdef      : tdef;
          indexdefderef : tderef;
          index,
          default       : longint;
          dispid        : longint;
          propaccesslist: array[tpropaccesslisttypes] of tpropaccesslist;
          parast : tsymtable;
          constructor create(const n : TSymStr);virtual;
          destructor  destroy;override;
          constructor ppuload(ppufile:tcompilerppufile);
          function  getsize : asizeint;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
          function getpropaccesslist(pap:tpropaccesslisttypes;out plist:tpropaccesslist):boolean;
          { copies the settings of the current propertysym to p; a bit like
            a form of getcopy, but without the name }
          procedure makeduplicate(p: tpropertysym; readprocdef, writeprocdef: tprocdef; out paranr: word);
          procedure add_accessor_parameters(readprocdef, writeprocdef: tprocdef);
          procedure add_index_parameter(var paranr: word; readprocdef, writeprocdef: tprocdef);
          { set up the accessors for this property }
          procedure add_getter_or_setter_for_sym(getset: tpropaccesslisttypes; sym: tsym; fielddef: tdef; accessordef: tprocdef);
          procedure register_override(overriddenprop: tpropertysym);
          { inherit the read/write property }
          procedure inherit_accessor(getset: tpropaccesslisttypes); virtual;
       end;
       tpropertysymclass = class of tpropertysym;

       tconstvalue = record
         case integer of
         0: (valueord : tconstexprint);
         1: (valueordptr : tconstptruint);
         2: (valueptr : pointer; len : longint);
       end;

       tconstsym = class(tstoredsym)
          constdef    : tdef;
          constdefderef : tderef;
          consttyp    : tconsttyp;
          value       : tconstvalue;
          constructor create_ord(const n : TSymStr;t : tconsttyp;v : tconstexprint;def:tdef);virtual;
          constructor create_ordptr(const n : TSymStr;t : tconsttyp;v : tconstptruint;def:tdef);virtual;
          constructor create_ptr(const n : TSymStr;t : tconsttyp;v : pointer;def:tdef);virtual;
          constructor create_string(const n : TSymStr;t : tconsttyp;str:pchar;l:longint;def:tdef);virtual;
          constructor create_wstring(const n : TSymStr;t : tconsttyp;pw:pcompilerwidestring);virtual;
          constructor create_undefined(const n : TSymStr;def:tdef);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor  destroy;override;
          procedure buildderef;override;
          procedure deref;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
{$ifdef DEBUG_NODE_XML}
        public
          procedure XMLPrintConstData(var T: Text);
{$endif DEBUG_NODE_XML}
       end;
       tconstsymclass = class of tconstsym;

       tenumsym = class(Tstoredsym)
          value      : longint;
          definition : tenumdef;
          definitionderef : tderef;
          constructor create(const n : TSymStr;def : tenumdef;v : longint);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          procedure buildderef;override;
          procedure deref;override;
       end;
       tenumsymclass = class of tenumsym;

       tsyssym = class(Tstoredsym)
          number : tinlinenumber;
          constructor create(const n : TSymStr;l : tinlinenumber);virtual;
          constructor ppuload(ppufile:tcompilerppufile);
          destructor  destroy;override;
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          class function find_by_number(l:longint):tsyssym;
       end;
       tsyssymclass = class of tsyssym;

    const
       maxmacrolen=16*1024;

    type
       pmacrobuffer = ^tmacrobuffer;
       tmacrobuffer = array[0..maxmacrolen-1] of char;

       tmacro = class(tstoredsym)
          {Normally true, but false when a previously defined macro is undef-ed}
          defined : boolean;
          {True if this is a mac style compiler variable, in which case no macro
           substitutions shall be done.}
          is_compiler_var : boolean;
          {Whether the macro was used. NOTE: A use of a macro which was never defined}
          {e. g. an IFDEF which returns false, will not be registered as used,}
          {since there is no place to register its use. }
          is_used : boolean;
          buftext : pchar;
          buflen  : longint;
          constructor create(const n : TSymStr);
          constructor ppuload(ppufile:tcompilerppufile);
          { do not override this routine in platform-specific subclasses,
            override ppuwrite_platform instead }
          procedure ppuwrite(ppufile:tcompilerppufile);override;final;
          destructor  destroy;override;
          function GetCopy:tmacro;
       end;

    var
       generrorsym : tsym;

       clabelsym: tlabelsymclass;
       cunitsym: tunitsymclass;
       cprogramparasym: tprogramparasymclass;
       cnamespacesym: tnamespacesymclass;
       cprocsym: tprocsymclass;
       ctypesym: ttypesymclass;
       cfieldvarsym: tfieldvarsymclass;
       clocalvarsym: tlocalvarsymclass;
       cparavarsym: tparavarsymclass;
       cstaticvarsym: tstaticvarsymclass;
       cabsolutevarsym: tabsolutevarsymclass;
       cpropertysym: tpropertysymclass;
       cconstsym: tconstsymclass;
       cenumsym: tenumsymclass;
       csyssym: tsyssymclass;

    { generate internal static field name based on regular field name }
    function internal_static_field_name(const fieldname: TSymStr): TSymStr;
    function get_high_value_sym(vs: tparavarsym):tsym; { marking it as inline causes IE 200311075 during loading from ppu file }

    procedure check_hints(const srsym: tsym; const symoptions: tsymoptions; const deprecatedmsg : pshortstring);inline;
    procedure check_hints(const srsym: tsym; const symoptions: tsymoptions; const deprecatedmsg : pshortstring;filepos:tfileposinfo);

    function same_constvalue(consttyp:tconsttyp;const value1,value2:tconstvalue):boolean;

implementation

    uses
       { global }
       verbose,
       { target }
       systems,
       { symtable }
       defutil,symtable,
       fmodule,
       { tree }
       node,
       { aasm }
       aasmdata,
       { codegen }
       paramgr,
       procinfo,
       { ppu }
       entfile,ppu
       ;

{****************************************************************************
                               Helpers
****************************************************************************}

    function internal_static_field_name(const fieldname: TSymStr): TSymStr;
      begin
        result:='$_static_'+fieldname;
      end;


    function get_high_value_sym(vs: tparavarsym):tsym;
      begin
        result := tsym(vs.owner.Find('high'+vs.name));
      end;


    function same_constvalue(consttyp:tconsttyp;const value1,value2:tconstvalue):boolean;
      begin
        case consttyp of
          constnone,
          constnil:
            result:=true;
          constord:
            result:=value1.valueord=value2.valueord;
          constpointer:
            result:=value1.valueordptr=value2.valueordptr;
          conststring,
          constreal,
          constset,
          constresourcestring,
          constwstring,
          constguid: begin
            if value1.len<>value2.len then
              exit(false);
            result:=CompareByte(value1.valueptr^,value2.valueptr^,value1.len)=0;
          end;
        end;
      end;


    procedure check_hints(const srsym: tsym; const symoptions: tsymoptions; const deprecatedmsg : pshortstring);
      begin
        check_hints(srsym,symoptions,deprecatedmsg,current_filepos);
      end;


    procedure check_hints(const srsym: tsym; const symoptions: tsymoptions; const deprecatedmsg : pshortstring;filepos:tfileposinfo);
      begin
        if not assigned(srsym) then
          internalerror(200602051);
        if sp_hint_deprecated in symoptions then
          if (sp_has_deprecated_msg in symoptions) and (deprecatedmsg <> nil) then
            MessagePos2(filepos,sym_w_deprecated_symbol_with_msg,srsym.realname,deprecatedmsg^)
          else
            MessagePos1(filepos,sym_w_deprecated_symbol,srsym.realname);
        if sp_hint_experimental in symoptions then
          MessagePos1(filepos,sym_w_experimental_symbol,srsym.realname);
        if sp_hint_platform in symoptions then
          MessagePos1(filepos,sym_w_non_portable_symbol,srsym.realname);
        if sp_hint_library in symoptions then
          MessagePos1(filepos,sym_w_library_symbol,srsym.realname);
        if sp_hint_unimplemented in symoptions then
          MessagePos1(filepos,sym_w_non_implemented_symbol,srsym.realname);
      end;

{****************************************************************************
                          TSYM (base for all symtypes)
****************************************************************************}

    constructor tstoredsym.create(st:tsymtyp;const n : TSymStr);
      begin
         inherited create(st,n);
      end;


    constructor tstoredsym.ppuload(st:tsymtyp;ppufile:tcompilerppufile);
      begin
{$ifdef symansistr}
         inherited Create(st,ppufile.getansistring);
{$else symansistr}
         inherited Create(st,ppufile.getstring);
{$endif symansistr}
         SymId:=ppufile.getlongint;
         current_module.symlist[SymId]:=self;
         ppufile.getposinfo(fileinfo);
         visibility:=tvisibility(ppufile.getbyte);
         ppufile.getset(tppuset2(symoptions));
         if sp_has_deprecated_msg in symoptions then
           deprecatedmsg:=ppufile.getpshortstring
         else
           deprecatedmsg:=nil;
         rtti_attribute_list:=trtti_attribute_list.ppuload(ppufile);
      end;


    procedure tstoredsym.ppuwrite(ppufile:tcompilerppufile);
      var
        oldintfcrc : boolean;
      begin
{$ifdef symansistr}
         ppufile.putansistring(realname);
{$else}
         ppufile.putstring(realname);
{$endif}
         ppufile.putlongint(SymId);
         ppufile.putposinfo(fileinfo);
         ppufile.putbyte(byte(visibility));
         { symoptions can differ between interface and implementation, except
           for overload (this is checked in pdecsub.proc_add_definition() )

           These differences can lead to compiler crashes, so ignore them.
           This does mean that changing e.g. the "deprecated" state of a symbol
           by itself will not trigger a recompilation of dependent units.
         }
         oldintfcrc:=ppufile.do_interface_crc;
         ppufile.do_interface_crc:=false;
         ppufile.putset(tppuset2(symoptions));
         if sp_has_deprecated_msg in symoptions then
           ppufile.putstring(deprecatedmsg^);
         ppufile.do_interface_crc:=oldintfcrc;
         trtti_attribute_list.ppuwrite(rtti_attribute_list,ppufile);
      end;


    procedure tstoredsym.ppuload_subentries(ppufile: tcompilerppufile);
      begin
        trtti_attribute_list.ppuload_subentries(rtti_attribute_list,ppufile);
      end;


    procedure tstoredsym.ppuwrite_subentries(ppufile: tcompilerppufile);
      begin
        trtti_attribute_list.ppuwrite_subentries(rtti_attribute_list,ppufile);
      end;


    procedure tstoredsym.deref;
      begin
        inherited;
        if assigned(rtti_attribute_list) then
          rtti_attribute_list.deref;
      end;


    procedure tstoredsym.buildderef;
      begin
        inherited;
        if not registered then
          register_sym;
        if assigned(rtti_attribute_list) then
          rtti_attribute_list.buildderef;
      end;


    procedure tstoredsym.writeentry(ppufile: tcompilerppufile; ibnr: byte);
      begin
        ppuwrite_platform(ppufile);
        ppufile.writeentry(ibnr);
      end;


    procedure tstoredsym.ppuwrite_platform(ppufile: tcompilerppufile);
      begin
        { by default: do nothing }
      end;

    procedure tstoredsym.ppuload_platform(ppufile: tcompilerppufile);
      begin
        { by default: do nothing }
      end;


    destructor tstoredsym.destroy;
      begin
        rtti_attribute_list.free;
        inherited destroy;
      end;


    procedure tstoredsym.register_sym;
      begin
        if registered then
          exit;
        { Register in current_module }
        if assigned(current_module) then
          begin
            current_module.symlist.Add(self);
            SymId:=current_module.symlist.Count-1;
          end
        else
          SymId:=symid_registered_nost;
      end;

{****************************************************************************
                                 TLABELSYM
****************************************************************************}

    constructor tlabelsym.create(const n : TSymStr);
      begin
         inherited create(labelsym,n);
         used:=false;
         defined:=false;
         nonlocal:=false;
         code:=nil;
      end;


    constructor tlabelsym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(labelsym,ppufile);
         code:=nil;
         used:=false;
         nonlocal:=false;
         defined:=true;
         ppuload_platform(ppufile);
      end;


    procedure tlabelsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         if owner.symtabletype=globalsymtable then
           Message(sym_e_ill_label_decl)
         else
           begin
              inherited ppuwrite(ppufile);
              writeentry(ppufile,iblabelsym);
           end;
      end;


   function tlabelsym.mangledname:TSymStr;
     begin
       if (asmblocklabel=nil) then
         begin
           if nonlocal then
             current_asmdata.getglobaljumplabel(asmblocklabel)
           else
             current_asmdata.getjumplabel(asmblocklabel);
         end;
       result:=asmblocklabel.name;
     end;

{****************************************************************************
                                  TUNITSYM
****************************************************************************}

    constructor tunitsym.create(const n : TSymStr;amodule : tobject);
      begin
         inherited create(unitsym,n);
         module:=amodule;
      end;

    constructor tunitsym.ppuload(ppufile:tcompilerppufile);

      begin
         inherited ppuload(unitsym,ppufile);
         module:=nil;
         ppuload_platform(ppufile);
      end;

    destructor tunitsym.destroy;
      begin
         inherited destroy;
      end;

    procedure tunitsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         writeentry(ppufile,ibunitsym);
      end;

{****************************************************************************
                             TPROGRAMPARASYM
****************************************************************************}

    constructor tprogramparasym.create(const n : TSymStr; i : dword);
      begin
         inherited create(programparasym,n);
         isoindex:=i;
      end;

    constructor tprogramparasym.ppuload(ppufile : tcompilerppufile);
      begin
        { program parameter syms (iso pascal style) might be never written to a ppu }
        internalerror(2015050102);
      end;

    destructor tprogramparasym.destroy;
      begin
       inherited destroy;
      end;

    procedure tprogramparasym.ppuwrite(ppufile : tcompilerppufile);
      begin
        { program parameter syms (iso pascal style) might be never written to a ppu }
        internalerror(2015050101);
      end;


{****************************************************************************
                                TNAMESPACESYM
****************************************************************************}

    constructor tnamespacesym.create(const n : TSymStr);
      begin
         inherited create(namespacesym,n);
         unitsym:=nil;
         unitsymderef.reset;
      end;

    constructor tnamespacesym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(namespacesym,ppufile);
         ppufile.getderef(unitsymderef);
         ppuload_platform(ppufile);
      end;

    procedure tnamespacesym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putderef(unitsymderef);
         writeentry(ppufile,ibnamespacesym);
      end;

    procedure tnamespacesym.buildderef;
      begin
        inherited buildderef;
        unitsymderef.build(unitsym);
      end;

    procedure tnamespacesym.deref;
      begin
        inherited deref;
        unitsym:=tsym(unitsymderef.resolve);
      end;


{****************************************************************************
                                  TPROCSYM
****************************************************************************}

    constructor tprocsym.create(const n : TSymStr);
      var
        i: longint;
      begin
         if not(ts_lowercase_proc_start in current_settings.targetswitches) or
            (n='') then
           inherited create(procsym,n)
         else
           begin
             { YToX -> yToX
               RC64Encode -> rc64Encode
               Test -> test
             }
             i:=2;
             while i<=length(n) do
               begin
                 if not(n[i] in ['A'..'Z']) then
                   begin
                     if (i>2) and
                        (n[i] in ['a'..'z']) then
                       dec(i);
                     break;
                   end;
                 inc(i);
               end;
             inherited create(procsym,lower(copy(n,1,i-1))+copy(n,i,length(n)));
           end;
         FProcdefList:=TFPObjectList.Create(false);
         FProcdefderefList:=nil;
         { the tprocdef have their own symoptions, make the procsym
           always visible }
         visibility:=vis_public;
      end;


    constructor tprocsym.ppuload(ppufile:tcompilerppufile);
      var
         pdderef : tderef;
         i,
         pdcnt : longint;
      begin
         inherited ppuload(procsym,ppufile);
         FProcdefList:=TFPObjectList.Create(false);
         FProcdefDerefList:=TFPList.Create;
         pdcnt:=ppufile.getword;
         for i:=1 to pdcnt do
          begin
            ppufile.getderef(pdderef);
            FProcdefDerefList.Add(Pointer(PtrInt(pdderef.dataidx)));
          end;
         ppuload_platform(ppufile);
      end;


    destructor tprocsym.destroy;
      begin
        FProcdefList.Free;
        if assigned(FProcdefDerefList) then
          FProcdefDerefList.Free;
        inherited destroy;
      end;


    procedure tprocsym.ppuwrite(ppufile:tcompilerppufile);
      var
         i : longint;
         d : tderef;
      begin
         inherited ppuwrite(ppufile);
         if fprocdefdereflist=nil then
           internalerror(2013121801);
         ppufile.putword(FProcdefDerefList.Count);
         for i:=0 to FProcdefDerefList.Count-1 do
           begin
             d.dataidx:=PtrInt(FProcdefDerefList[i]);
             ppufile.putderef(d);
           end;
         writeentry(ppufile,ibprocsym);
      end;


    procedure tprocsym.write_parameter_lists(skipdef:tprocdef);
      var
        i  : longint;
        pd : tprocdef;
      begin
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if pd<>skipdef then
              MessagePos1(pd.fileinfo,sym_e_param_list,pd.fullprocname(false));
           end;
      end;


    procedure tprocsym.check_forward;
      var
        i  : longint;
        pd : tprocdef;
      begin
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if (pd.owner=owner) and (pd.forwarddef) then
              begin
                { For mode macpas. Make implicit externals (procedures declared in the interface
                  section which do not have a counterpart in the implementation)
                  to be an imported procedure }
                if (m_mac in current_settings.modeswitches) and
                   (pd.interfacedef) then
                  begin
                    pd.setmangledname(target_info.CPrefix+tprocdef(pd).procsym.realname);
                    if (not current_module.interface_only) then
                      MessagePos1(pd.fileinfo,sym_w_forward_not_resolved,pd.fullprocname(false));
                  end
                else
                  begin
                    MessagePos1(pd.fileinfo,sym_e_forward_not_resolved,pd.fullprocname(false));
                  end;
                { Turn further error messages off }
                pd.forwarddef:=false;
              end;
          end;
      end;


    procedure tprocsym.buildderef;
      var
        i  : longint;
        pd : tprocdef;
        d  : tderef;
      begin
        inherited;
        if not assigned(FProcdefDerefList) then
          FProcdefDerefList:=TFPList.Create
        else
          FProcdefDerefList.Clear;
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            { only write the proc definitions that belong
              to this procsym and are in the global symtable }
            if pd.owner=owner then
              begin
                d.build(pd);
                FProcdefDerefList.Add(Pointer(PtrInt(d.dataidx)));
              end;
          end;
      end;


    procedure tprocsym.deref;
      var
        i  : longint;
        pd : tprocdef;
        d  : tderef;
      begin
        { Clear all procdefs }
        ProcdefList.Clear;
        if not assigned(FProcdefDerefList) then
          internalerror(200611031);
        for i:=0 to FProcdefDerefList.Count-1 do
          begin
            d.dataidx:=PtrInt(FProcdefDerefList[i]);
            pd:=tprocdef(d.resolve);
            ProcdefList.Add(pd);
          end;
      end;


    function Tprocsym.Find_procdef_bytype(pt:Tproctypeoption):Tprocdef;
      var
        i  : longint;
        pd : tprocdef;
      begin
        result:=nil;
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if pd.proctypeoption=pt then
              begin
                result:=pd;
                exit;
              end;
          end;
      end;


    function tprocsym.find_bytype_parameterless(pt: Tproctypeoption): Tprocdef;
      var
        i,j : longint;
        pd  : tprocdef;
        found : boolean;
      begin
        result:=nil;
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if (pd.proctypeoption=pt) then
              begin
                found:=true;
                for j:=0 to pd.paras.count-1 do
                  begin
                    if not(vo_is_hidden_para in tparavarsym(pd.paras[j]).varoptions) then
                      begin
                        found:=false;
                        break;
                      end;
                  end;
                if found then
                  begin
                    result:=pd;
                    exit;
                  end;
              end;
          end;
      end;


    function check_procdef_paras(pd:tprocdef;para:TFPObjectList;retdef:tdef;
                                            cpoptions:tcompare_paras_options): tprocdef;
      var
        eq: tequaltype;
      begin
        result:=nil;
        if assigned(retdef) then
          eq:=compare_defs(retdef,pd.returndef,nothingn)
        else
          eq:=te_equal;
        if (eq>=te_equal) or
           ((cpo_allowconvert in cpoptions) and (eq>te_incompatible)) then
          begin
            eq:=compare_paras(para,pd.paras,cp_value_equal_const,cpoptions);
            if (eq>=te_equal) or
               ((cpo_allowconvert in cpoptions) and (eq>te_incompatible)) then
              begin
                result:=pd;
                exit;
              end;
          end;
      end;


    function Tprocsym.Find_procdef_bypara(para:TFPObjectList;retdef:tdef;
                                            cpoptions:tcompare_paras_options):Tprocdef;
      var
        i  : longint;
        pd : tprocdef;
      begin
        result:=nil;
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            result:=check_procdef_paras(pd,para,retdef,cpoptions);
            if assigned(result) then
              exit;
          end;
      end;


    function Tprocsym.find_procdef_bytype_and_para(pt:Tproctypeoption;
               para:TFPObjectList;retdef:tdef;cpoptions:tcompare_paras_options):Tprocdef;
      var
        i  : longint;
        pd : tprocdef;
      begin
        result:=nil;
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if pd.proctypeoption=pt then
              begin
                result:=check_procdef_paras(pd,para,retdef,cpoptions);
                if assigned(result) then
                  exit;
              end;
          end;
      end;


    function tprocsym.find_procdef_byoptions(ops: tprocoptions): Tprocdef;
      var
        i  : longint;
        pd : tprocdef;
      begin
        result:=nil;
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if ops * pd.procoptions = ops then
              begin
                result:=pd;
                exit;
              end;
          end;
      end;

    function Tprocsym.Find_procdef_byprocvardef(d:Tprocvardef):Tprocdef;
      var
        i  : longint;
        bestpd,
        pd : tprocdef;
        eq,besteq : tequaltype;
        sym: tsym;
        ps: tprocsym;
      begin
        { This function will return the pprocdef of pprocsym that
          is the best match for procvardef. When there are multiple
          matches it returns nil.}
        result:=nil;
        bestpd:=nil;
        besteq:=te_incompatible;
        ps:=self;
        repeat
          for i:=0 to ps.ProcdefList.Count-1 do
            begin
              pd:=tprocdef(ps.ProcdefList[i]);
              eq:=proc_to_procvar_equal(pd,d,false);
              if eq>=te_convert_l1 then
                begin
                  { multiple procvars with the same equal level }
                  if assigned(bestpd) and
                     (besteq=eq) then
                    exit;
                  if eq>besteq then
                    begin
                      besteq:=eq;
                      bestpd:=pd;
                    end;
                end;
            end;
          { maybe TODO: also search class helpers? -- this code is similar to
            what happens in htypechk in
            tcallcandidates.collect_overloads_in_struct: keep searching in
            parent types in case the currently found procdef is marked as
            "overload" and we haven't found a proper match yet }
          if assigned(ps.owner.defowner) and
             (ps.owner.defowner.typ=objectdef) and
             assigned(tobjectdef(ps.owner.defowner).childof) and
             (not assigned(bestpd) or
              (po_overload in bestpd.procoptions)) then
            begin
              sym:=tsym(tobjectdef(ps.owner.defowner).childof.symtable.find(ps.name));
              if assigned(sym) and
                 (sym.typ=procsym) then
                ps:=tprocsym(sym)
              else
                ps:=nil;
            end
          else
            ps:=nil;
        until (besteq>=te_equal) or
              not assigned(ps);
        result:=bestpd;
      end;


    function Tprocsym.Find_procdef_assignment_operator(fromdef,todef:tdef;var besteq:tequaltype;isexplicit:boolean):Tprocdef;
      var
        paraidx, realparamcount,
        i, j : longint;
        bestpd,
        hpd,
        pd : tprocdef;
        convtyp : tconverttype;
        eq      : tequaltype;
        shortstringcount : longint;
        checkshortstring,
        isgenshortstring : boolean;
      begin
        { This function will return the pprocdef of pprocsym that
          is the best match for fromdef and todef. }
        result:=nil;
        bestpd:=nil;
        besteq:=te_incompatible;
        { special handling for assignment operators overloads to shortstring:
          for implicit assignment we pick the ShortString one if available and
          only pick one with specific length if it is the *only* one }
        shortstringcount:=0;
        checkshortstring:=not isexplicit and
                          is_shortstring(todef) and
                          (tstringdef(todef).len<>255);
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if (pd.owner.symtabletype=staticsymtable) and not pd.owner.iscurrentunit then
              continue;
            if (equal_defs(todef,pd.returndef) or
                { shortstrings of different lengths are ok as result }
                (not isexplicit and is_shortstring(todef) and is_shortstring(pd.returndef))) and
               { the result type must be always really equal and not an alias,
                 if you mess with this code, check tw4093 }
               ((todef=pd.returndef) or
                (
                  not(df_unique in todef.defoptions) and
                  not(df_unique in pd.returndef.defoptions)
                )
               ) then
              begin
                paraidx:=0;
                { ignore vs_hidden parameters }
                while (paraidx<pd.paras.count) and
                      assigned(pd.paras[paraidx]) and
                      (vo_is_hidden_para in tparavarsym(pd.paras[paraidx]).varoptions) do
                  inc(paraidx);
                realparamcount:=0;
                for j := 0 to pd.paras.Count-1 do
                  if assigned(pd.paras[j]) and not (vo_is_hidden_para in tparavarsym(pd.paras[j]).varoptions) then
                    inc(realparamcount);
                if (paraidx<pd.paras.count) and
                   assigned(pd.paras[paraidx]) and
                   (realparamcount = 1) then
                  begin
                    eq:=compare_defs_ext(fromdef,tparavarsym(pd.paras[paraidx]).vardef,nothingn,convtyp,hpd,[]);

                    { alias? if yes, only l1 choice,
                      if you mess with this code, check tw4093 }
                    if (eq=te_exact) and
                       (fromdef<>tparavarsym(pd.paras[paraidx]).vardef) and
                       ((df_unique in fromdef.defoptions) or
                       (df_unique in tparavarsym(pd.paras[paraidx]).vardef.defoptions)) then
                      eq:=te_convert_l1;

                    isgenshortstring:=false;
                    if checkshortstring and is_shortstring(pd.returndef) then
                      if tstringdef(pd.returndef).len<>255 then
                        inc(shortstringcount)
                      else
                        isgenshortstring:=true;

                    if (eq=te_exact) and (not checkshortstring or isgenshortstring) then
                      begin
                        besteq:=eq;
                        result:=pd;
                        exit;
                      end;
                    if eq>besteq then
                      begin
                        bestpd:=pd;
                        besteq:=eq;
                      end;
                  end;
              end;
          end;
        if checkshortstring and (shortstringcount>1) then
          begin
            besteq:=te_incompatible;
            bestpd:=nil;
          end;
        result:=bestpd;
      end;

      function Tprocsym.find_procdef_enumerator_operator(fromdef,todef:tdef;var besteq:tequaltype):Tprocdef;
      var
        paraidx, realparamcount,
        i, j : longint;
        bestpd,
        hpd,
        pd : tprocdef;
        current : tpropertysym;
        convtyp : tconverttype;
        eq      : tequaltype;
      begin
        { This function will return the pprocdef of pprocsym that
          is the best match for fromdef and todef. }
        result:=nil;
        bestpd:=nil;
        besteq:=te_incompatible;
        for i:=0 to ProcdefList.Count-1 do
          begin
            pd:=tprocdef(ProcdefList[i]);
            if (pd.owner.symtabletype=staticsymtable) and not pd.owner.iscurrentunit then
              continue;
            if not (is_class_or_interface_or_object(pd.returndef) or is_record(pd.returndef)) then
              continue;
            current := tpropertysym(tabstractrecorddef(pd.returndef).search_enumerator_current);
            if (current = nil) then
              continue;
            // compare current result def with the todef
            if (equal_defs(todef, current.propdef) or
                { shortstrings of different lengths are ok as result }
                (is_shortstring(todef) and is_shortstring(current.propdef))) and
               { the result type must be always really equal and not an alias,
                 if you mess with this code, check tw4093 }
               ((todef=current.propdef) or
                (
                  not(df_unique in todef.defoptions) and
                  not(df_unique in current.propdef.defoptions)
                )
               ) then
              begin
                paraidx:=0;
                { ignore vs_hidden parameters }
                while (paraidx<pd.paras.count) and
                      assigned(pd.paras[paraidx]) and
                      (vo_is_hidden_para in tparavarsym(pd.paras[paraidx]).varoptions) do
                  inc(paraidx);
                realparamcount:=0;
                for j := 0 to pd.paras.Count-1 do
                  if assigned(pd.paras[j]) and not (vo_is_hidden_para in tparavarsym(pd.paras[j]).varoptions) then
                    inc(realparamcount);
                if (paraidx<pd.paras.count) and
                   assigned(pd.paras[paraidx]) and
                   (realparamcount = 1) then
                  begin
                    eq:=compare_defs_ext(fromdef,tparavarsym(pd.paras[paraidx]).vardef,nothingn,convtyp,hpd,[]);

                    { alias? if yes, only l1 choice,
                      if you mess with this code, check tw4093 }
                    if (eq=te_exact) and
                       (fromdef<>tparavarsym(pd.paras[paraidx]).vardef) and
                       ((df_unique in fromdef.defoptions) or
                       (df_unique in tparavarsym(pd.paras[paraidx]).vardef.defoptions)) then
                      eq:=te_convert_l1;

                    if eq=te_exact then
                      begin
                        besteq:=eq;
                        result:=pd;
                        exit;
                      end;
                    if eq>besteq then
                      begin
                        bestpd:=pd;
                        besteq:=eq;
                      end;
                  end;
              end;
          end;
        result:=bestpd;
      end;


{****************************************************************************
                                  TERRORSYM
****************************************************************************}

    constructor terrorsym.create;
      begin
        inherited create(errorsym,'');
      end;


    procedure terrorsym.register_sym;
      begin
        { these should never be written to a ppu file, since they don't
          derive from tstoredsym }
        Internalerror(2015101801);
      end;

{****************************************************************************
                                TPROPERTYSYM
****************************************************************************}

    procedure tpropertysym.finalize_getter_or_setter_for_sym(getset: tpropaccesslisttypes; sym: tsym; fielddef: tdef; accessordef: tprocdef);
      begin
        { do nothing by default }
      end;


    constructor tpropertysym.create(const n : TSymStr);
      var
        pap : tpropaccesslisttypes;
      begin
         inherited create(propertysym,n);
         propoptions:=[];
         index:=0;
         default:=0;
         propdef:=nil;
         propdefderef.reset;
         indexdef:=nil;
         indexdefderef.reset;
         parast:=nil;
         for pap:=low(tpropaccesslisttypes) to high(tpropaccesslisttypes) do
           propaccesslist[pap]:=tpropaccesslist.create;
      end;


    constructor tpropertysym.ppuload(ppufile:tcompilerppufile);
      var
        pap : tpropaccesslisttypes;
      begin
         inherited ppuload(propertysym,ppufile);
         ppufile.getset(tppuset2(propoptions));
         if ppo_overrides in propoptions then
           ppufile.getderef(overriddenpropsymderef);
         ppufile.getderef(propdefderef);
         index:=ppufile.getlongint;
         default:=ppufile.getlongint;
         ppufile.getderef(indexdefderef);
         for pap:=low(tpropaccesslisttypes) to high(tpropaccesslisttypes) do
           propaccesslist[pap]:=ppufile.getpropaccesslist;
         ppuload_platform(ppufile);
         if [ppo_hasparameters,ppo_overrides]*propoptions=[ppo_hasparameters] then
           begin
             parast:=tparasymtable.create(nil,0);
             tparasymtable(parast).ppuload(ppufile);
           end
         else
           parast:=nil;
      end;


    destructor tpropertysym.destroy;
      var
        pap : tpropaccesslisttypes;
      begin
         for pap:=low(tpropaccesslisttypes) to high(tpropaccesslisttypes) do
           propaccesslist[pap].free;
         parast.free;
         inherited destroy;
      end;


    procedure tpropertysym.buildderef;
      var
        pap : tpropaccesslisttypes;
      begin
        inherited;
        propdefderef.build(propdef);
        indexdefderef.build(indexdef);
        for pap:=low(tpropaccesslisttypes) to high(tpropaccesslisttypes) do
          propaccesslist[pap].buildderef;
        if ppo_overrides in propoptions then
          overriddenpropsymderef.build(overriddenpropsym)
        else
        if ppo_hasparameters in propoptions then
          tparasymtable(parast).buildderef;
      end;


    procedure tpropertysym.deref;
      var
        pap : tpropaccesslisttypes;
      begin
        indexdef:=tdef(indexdefderef.resolve);
        propdef:=tdef(propdefderef.resolve);
        for pap:=low(tpropaccesslisttypes) to high(tpropaccesslisttypes) do
          propaccesslist[pap].resolve;

        if ppo_overrides in propoptions then
          begin
            overriddenpropsym:=tpropertysym(overriddenpropsymderef.resolve);
            if ppo_hasparameters in propoptions then
              parast:=overriddenpropsym.parast.getcopy;
          end
        else
        if ppo_hasparameters in propoptions then
          tparasymtable(parast).deref(false)
      end;


    function tpropertysym.getpropaccesslist(pap:tpropaccesslisttypes;out plist:tpropaccesslist):boolean;
    var
      hpropsym : tpropertysym;
    begin
      result:=false;
      { find property in the overridden list }
      hpropsym:=self;
      repeat
        plist:=hpropsym.propaccesslist[pap];
        if not plist.empty then
          begin
            result:=true;
            exit;
          end;
        hpropsym:=hpropsym.overriddenpropsym;
      until not assigned(hpropsym);
    end;


    procedure tpropertysym.add_accessor_parameters(readprocdef, writeprocdef: tprocdef);
      var
        i: integer;
        orig, hparavs: tparavarsym;
      begin
        for i := 0 to parast.SymList.Count - 1 do
          begin
            orig:=tparavarsym(parast.SymList[i]);
            if assigned(readprocdef) then
              begin
                hparavs:=cparavarsym.create(orig.RealName,orig.paranr,orig.varspez,orig.vardef,[]);
                readprocdef.parast.insert(hparavs);
              end;
            if assigned(writeprocdef) then
              begin
                hparavs:=cparavarsym.create(orig.RealName,orig.paranr,orig.varspez,orig.vardef,[]);
                writeprocdef.parast.insert(hparavs);
              end;
          end;
      end;


    procedure tpropertysym.add_index_parameter(var paranr: word; readprocdef, writeprocdef: tprocdef);
      var
        hparavs: tparavarsym;
      begin
        inc(paranr);
        if assigned(readprocdef) then
          begin
            hparavs:=cparavarsym.create('$index',10*paranr,vs_value,indexdef,[]);
            readprocdef.parast.insert(hparavs);
          end;
        if assigned(writeprocdef) then
          begin
            hparavs:=cparavarsym.create('$index',10*paranr,vs_value,indexdef,[]);
            writeprocdef.parast.insert(hparavs);
          end;
      end;


    procedure tpropertysym.add_getter_or_setter_for_sym(getset: tpropaccesslisttypes; sym: tsym; fielddef: tdef; accessordef: tprocdef);
      var
        cpo: tcompare_paras_options;
      begin
        case sym.typ of
          procsym :
            begin
              { search procdefs matching accessordef }
              { we ignore hidden stuff here because the property access symbol might have
                non default calling conventions which might change the hidden stuff;
                see tw3216.pp (FK) }
              cpo:=[cpo_allowdefaults,cpo_ignorehidden];
              { allow var-parameters for setters in case of VARPROPSETTER+ }
              if (getset=palt_write) and
                 (cs_varpropsetter in current_settings.localswitches) then
                include(cpo,cpo_ignorevarspez);
              propaccesslist[getset].procdef:=tprocsym(sym).find_procdef_bypara(accessordef.paras,accessordef.returndef,cpo);
              if not assigned(propaccesslist[getset].procdef) or
                 { because of cpo_ignorehidden we need to compare if it is a static class method and we have a class property }
                 ((sp_static in symoptions)<>tprocdef(propaccesslist[getset].procdef).no_self_node) then
                Message(parser_e_ill_property_access_sym)
              else
                finalize_getter_or_setter_for_sym(getset,sym,fielddef,accessordef);
            end;
          fieldvarsym :
            begin
              if not assigned(fielddef) then
                internalerror(200310071);
              if compare_defs(fielddef,propdef,nothingn)>=te_equal then
               begin
                 { property parameters are allowed if this is
                   an indexed property, because the index is then
                   the parameter.
                   Note: In the help of Kylix it is written
                   that it isn't allowed, but the compiler accepts it (PFV) }
                 if (ppo_hasparameters in propoptions) or
                    ((sp_static in symoptions) <> (sp_static in sym.symoptions)) then
                   Message(parser_e_ill_property_access_sym)
                 else
                   finalize_getter_or_setter_for_sym(getset,sym,fielddef,accessordef);
               end
              else
               IncompatibleTypes(fielddef,propdef);
            end;
          else
            Message(parser_e_ill_property_access_sym);
        end;
      end;


    procedure tpropertysym.register_override(overriddenprop: tpropertysym);
      begin
        overriddenpropsym:=tpropertysym(overriddenprop);
        include(propoptions,ppo_overrides);
      end;


    procedure tpropertysym.inherit_accessor(getset: tpropaccesslisttypes);
      begin
        { nothing to do by default }
      end;


    procedure tpropertysym.makeduplicate(p: tpropertysym; readprocdef, writeprocdef: tprocdef; out paranr: word);
      begin
        { inherit all type related entries }
        p.indexdef:=indexdef;
        p.propdef:=propdef;
        p.index:=index;
        p.default:=default;
        p.propoptions:=propoptions;
        paranr:=0;
        if ppo_hasparameters in propoptions then
          begin
            p.parast:=parast.getcopy;
            p.add_accessor_parameters(readprocdef,writeprocdef);
            paranr:=p.parast.SymList.Count;
          end;
        if ppo_indexed in p.propoptions then
          p.add_index_parameter(paranr,readprocdef,writeprocdef);
      end;


    function tpropertysym.getsize : asizeint;
      begin
         getsize:=0;
      end;


    procedure tpropertysym.ppuwrite(ppufile:tcompilerppufile);
      var
        pap : tpropaccesslisttypes;
      begin
        inherited ppuwrite(ppufile);
        ppufile.putset(tppuset2(propoptions));
        if ppo_overrides in propoptions then
          ppufile.putderef(overriddenpropsymderef);
        ppufile.putderef(propdefderef);
        ppufile.putlongint(index);
        ppufile.putlongint(default);
        ppufile.putderef(indexdefderef);
        for pap:=low(tpropaccesslisttypes) to high(tpropaccesslisttypes) do
          ppufile.putpropaccesslist(propaccesslist[pap]);
        writeentry(ppufile,ibpropertysym);
        if [ppo_hasparameters,ppo_overrides]*propoptions=[ppo_hasparameters] then
          tparasymtable(parast).ppuwrite(ppufile);
      end;

{****************************************************************************
                            TABSTRACTVARSYM
****************************************************************************}

    constructor tabstractvarsym.create(st:tsymtyp;const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);
      begin
         inherited create(st,n);
         vardef:=def;
         vardefderef.reset;
         varspez:=vsp;
         varstate:=vs_declared;
         varoptions:=vopts;
      end;


    constructor tabstractvarsym.ppuload(st:tsymtyp;ppufile:tcompilerppufile);
      begin
         inherited ppuload(st,ppufile);
         varstate:=vs_readwritten;
         varspez:=tvarspez(ppufile.getbyte);
         varregable:=tvarregable(ppufile.getbyte);
         ppufile.getset(tppuset1(varsymaccess));
         ppufile.getderef(vardefderef);
         ppufile.getset(tppuset4(varoptions));
      end;


    procedure tabstractvarsym.buildderef;
      begin
        inherited;
        vardefderef.build(vardef);
      end;


    procedure tabstractvarsym.deref;
      begin
        { assigning vardef also updates varregable. We just loaded this   }
        { value from a ppu, so it must not be changed (e.g. tw7817a.pp/   }
        { tw7817b.pp: the address is taken of a local variable in an      }
        { inlined procedure -> must remain non-regable when inlining)     }
        setvardef(tdef(vardefderef.resolve));
      end;


    procedure tabstractvarsym.ppuwrite(ppufile:tcompilerppufile);
      var
        oldintfcrc : boolean;
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(varspez));
         oldintfcrc:=ppufile.do_crc;
         ppufile.do_crc:=false;
         ppufile.putbyte(byte(varregable));
         ppufile.putset(tppuset1(varsymaccess));
         ppufile.do_crc:=oldintfcrc;
         ppufile.putderef(vardefderef);
         ppufile.putset(tppuset4(varoptions));
      end;


    function tabstractvarsym.getsize : asizeint;
      begin
        if assigned(vardef) and
           ((vardef.typ<>arraydef) or
            is_dynamic_array(vardef) or
            (tarraydef(vardef).highrange>=tarraydef(vardef).lowrange)) then
          result:=vardef.size
        else
          result:=0;
      end;


    function  tabstractvarsym.getpackedbitsize : longint;
      begin
        { bitpacking is only done for ordinals }
        if not is_ordinal(vardef) then
          internalerror(2006082010);
        result:=vardef.packedbitsize;
      end;


    function tabstractvarsym.is_regvar(refpara: boolean):boolean;
      var
        tempdef : tdef;
      begin
        { Register variables are not allowed in the following cases:
           - regvars are disabled
           - exceptions are used (after an exception is raised the contents of the
               registers is not valid anymore)
           - it has a local copy
           - the value needs to be in memory (i.e. reference counted) }
        result:=(cs_opt_regvar in current_settings.optimizerswitches) and
                not(pi_has_assembler_block in current_procinfo.flags) and
                not(pi_uses_exceptions in current_procinfo.flags) and
                not(pi_has_interproclabel in current_procinfo.flags) and
                ((refpara and
                  (varregable <> vr_none)) or
                 (not refpara and
                  not(varregable in [vr_none,vr_addr])))
{$if not defined(powerpc) and not defined(powerpc64)}
                and ((vardef.typ <> recorddef) or
                     (varregable = vr_addr) or
                     tabstractrecordsymtable(tabstractrecorddef(vardef).symtable).has_single_field(tempdef) or
                     not(varstate in [vs_written,vs_readwritten]));
{$endif}
      end;


    procedure tabstractvarsym.setvardef_and_regable(def:tdef);
      begin
        setvardef(def);
        setregable;
      end;


    procedure tabstractvarsym.set_addr_taken(AValue: boolean);
      begin
        if AValue then
          include(varsymaccess, vsa_addr_taken)
        else
          exclude(varsymaccess, vsa_addr_taken);
      end;


    procedure tabstractvarsym.set_different_scope(AValue: boolean);
      begin
        if AValue then
          include(varsymaccess, vsa_different_scope)
        else
          exclude(varsymaccess, vsa_different_scope);
      end;


    procedure tabstractvarsym.setregable;
      begin
        if vo_volatile in varoptions then
          exit;
         { can we load the value into a register ? }
        if not assigned(owner) or
           (owner.symtabletype in [localsymtable, parasymtable]) or
           (
            (owner.symtabletype=staticsymtable) and
            not(cs_create_pic in current_settings.moduleswitches)
           ) then
          begin
            if (tstoreddef(vardef).is_intregable and
              { we could keep all aint*2 records in registers, but this causes
                too much spilling for CPUs with 8-16 registers so keep only
                parameters and function results of this type in register because they are normally
                passed by register anyways

                This can be changed, as soon as we have full ssa (FK) }
              ((typ=paravarsym) or
                (vo_is_funcret in varoptions) or
                (tstoreddef(vardef).typ<>recorddef) or
                (tstoreddef(vardef).size<=sizeof(aint)))) or

               { const parameters can be put into registers if the def fits into a register }
               (tstoreddef(vardef).is_const_intregable and
                (typ=paravarsym) and
                (varspez=vs_const)) then
              varregable:=vr_intreg
            else if tstoreddef(vardef).is_fpuregable then
              begin
                if use_vectorfpu(vardef) then
                  varregable:=vr_mmreg
                else
                  varregable:=vr_fpureg;
              end
            else if is_vector(vardef) and
              fits_in_mm_register(vardef) then
              begin
                varregable:=vr_mmreg;
              end;
          end;
      end;


    function tabstractvarsym.get_addr_taken: boolean;
      begin
        result:=vsa_addr_taken in varsymaccess;
      end;


    function tabstractvarsym.get_different_scope: boolean;
      begin
        result:=vsa_different_scope in varsymaccess;
      end;


    procedure tabstractvarsym.setvardef(const def: tdef);
      begin
        _vardef := def;
      end;


{****************************************************************************
                               TFIELDVARSYM
****************************************************************************}

    constructor tfieldvarsym.create(const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);
      begin
         inherited create(fieldvarsym,n,vsp,def,vopts);
         fieldoffset:=-1;
      end;


    constructor tfieldvarsym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(fieldvarsym,ppufile);
         fieldoffset:=ppufile.getasizeint;
         if (vo_has_mangledname in varoptions) then
           externalname:=ppufile.getpshortstring
         else
           externalname:=nil;
         ppuload_platform(ppufile);
      end;


    procedure tfieldvarsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putasizeint(fieldoffset);
         if (vo_has_mangledname in varoptions) then
           ppufile.putstring(externalname^);
         writeentry(ppufile,ibfieldvarsym);
      end;


    procedure tfieldvarsym.set_externalname(const s: string);
      begin
        internalerror(2014033001);
      end;


    function tfieldvarsym.mangledname:TSymStr;
      var
        srsym : tsym;
        srsymtable : tsymtable;
      begin
        if sp_static in symoptions then
          begin
            if searchsym(lower(owner.name^)+'_'+name,srsym,srsymtable) then
              result:=srsym.mangledname
            { when generating the debug info for the module in which the }
            { symbol is defined, the localsymtable of that module is     }
            { already popped from the symtablestack                      }
            else if searchsym_in_module(current_module,lower(owner.name^)+'_'+name,srsym,srsymtable) then
              result:=srsym.mangledname
            else
              internalerror(2007012501);
          end
        else if is_objcclass(tdef(owner.defowner)) then
          begin
{$ifdef symansistr}
            if cachedmangledname<>'' then
              result:=cachedmangledname
{$else symansistr}
            if assigned(cachedmangledname) then
              result:=cachedmangledname^
{$endif symansistr}
            else
              begin
                result:=target_info.cprefix+'OBJC_IVAR_$_'+tobjectdef(owner.defowner).objextname^+'.'+RealName;
{$ifdef symansistr}
                cachedmangledname:=result;
{$else symansistr}
                cachedmangledname:=stringdup(result);
{$endif symansistr}
              end;
          end
        else
          result:=inherited mangledname;
      end;


    destructor tfieldvarsym.destroy;
      begin
{$ifndef symansistr}
        stringdispose(cachedmangledname);
{$endif symansistr}
        stringdispose(externalname);
        inherited destroy;
      end;

{$ifdef DEBUG_NODE_XML}
      procedure TFieldVarSym.XMLPrintFieldData(var T: Text);
        begin
          WriteLn(T, PrintNodeIndention, '<type>', SanitiseXMLString(vardef.GetTypeName), '</type>');
          WriteLn(T, PrintNodeIndention, '<visibility>', visibility, '</visibility>');
          WriteLn(T, PrintNodeIndention, '<offset>', fieldoffset, '</offset>');
          WriteLn(T, PrintNodeIndention, '<size>', vardef.size, '</size>');
        end;
{$endif DEBUG_NODE_XML}

{****************************************************************************
                        TABSTRACTNORMALVARSYM
****************************************************************************}

    constructor tabstractnormalvarsym.create(st:tsymtyp;const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);
      begin
         inherited create(st,n,vsp,def,vopts);
         fillchar(localloc,sizeof(localloc),0);
         fillchar(initialloc,sizeof(initialloc),0);
         defaultconstsym:=nil;
         defaultconstsymderef.reset;
      end;


    constructor tabstractnormalvarsym.ppuload(st:tsymtyp;ppufile:tcompilerppufile);
      begin
         inherited ppuload(st,ppufile);
         fillchar(localloc,sizeof(localloc),0);
         fillchar(initialloc,sizeof(initialloc),0);
         ppufile.getderef(defaultconstsymderef);
      end;

    function tabstractnormalvarsym.globalasmsym: boolean;
      begin
        result:=
          (owner.symtabletype=globalsymtable) or
          (create_smartlink and
           not(tf_smartlink_sections in target_info.flags)) or
          current_module.islibrary or
          (assigned(current_procinfo) and
           ((po_inline in current_procinfo.procdef.procoptions) or
            { globalasmsym is called normally before the body of a subroutine is parsed
              so we cannot know if it will be auto inlined, so make all symbols of it
              global if asked }
            (not(po_noinline in current_procinfo.procdef.procoptions) and
             (cs_opt_autoinline in current_settings.optimizerswitches)))
          ) or
          (vo_is_public in varoptions);
      end;


    procedure tabstractnormalvarsym.buildderef;
      begin
        inherited buildderef;
        defaultconstsymderef.build(defaultconstsym);
      end;


    procedure tabstractnormalvarsym.deref;
      begin
        inherited deref;
        defaultconstsym:=tsym(defaultconstsymderef.resolve);
      end;


    procedure tabstractnormalvarsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putderef(defaultconstsymderef);
      end;


{****************************************************************************
                             Tstaticvarsym
****************************************************************************}

    constructor tstaticvarsym.create(const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);
      begin
         inherited create(staticvarsym,n,vsp,def,vopts);
         fieldvarsymderef.reset;
{$ifdef symansistr}
         _mangledname:='';
{$else symansistr}
         _mangledname:=nil;
{$endif symansistr}
      end;


    constructor tstaticvarsym.create_dll(const n : TSymStr;vsp:tvarspez;def:tdef);
      begin
         tstaticvarsym(self).create(n,vsp,def,[vo_is_dll_var]);
      end;


    constructor tstaticvarsym.create_C(const n: TSymStr; const mangled : TSymStr;vsp:tvarspez;def:tdef);
      begin
         tstaticvarsym(self).create(n,vsp,def,[]);
         set_mangledname(mangled);
      end;


    constructor tstaticvarsym.create_from_fieldvar(const n: TSymStr;fieldvar:tfieldvarsym);
      begin
        create(internal_static_field_name(n),fieldvar.varspez,fieldvar.vardef,[]);
        fieldvarsym:=fieldvar;
      end;


    constructor tstaticvarsym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(staticvarsym,ppufile);
{$ifdef symansistr}
         if vo_has_mangledname in varoptions then
           _mangledname:=ppufile.getansistring
         else
           _mangledname:='';
{$else symansistr}
         if vo_has_mangledname in varoptions then
           _mangledname:=ppufile.getpshortstring
         else
           _mangledname:=nil;
{$endif symansistr}
         if vo_has_section in varoptions then
           section:=ppufile.getansistring;
         ppufile.getderef(fieldvarsymderef);
         ppuload_platform(ppufile);
      end;


    destructor tstaticvarsym.destroy;
      begin
{$ifndef symansistr}
        if assigned(_mangledname) then
          begin
{$ifdef MEMDEBUG}
            memmanglednames.start;
{$endif MEMDEBUG}
            stringdispose(_mangledname);
{$ifdef MEMDEBUG}
            memmanglednames.stop;
{$endif MEMDEBUG}
          end;
        stringdispose(_mangledbasename);
{$endif}
        inherited destroy;
      end;


    procedure tstaticvarsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         { write mangledname rather than _mangledname in case the mangledname
           has not been calculated yet (can happen in case only the
           mangledbasename has been set) }
         if vo_has_mangledname in varoptions then
{$ifdef symansistr}
           ppufile.putansistring(mangledname);
{$else symansistr}
           ppufile.putstring(mangledname);
{$endif symansistr}
         if vo_has_section in varoptions then
           ppufile.putansistring(section);
         ppufile.putderef(fieldvarsymderef);
         writeentry(ppufile,ibstaticvarsym);
      end;


    procedure tstaticvarsym.buildderef;
      begin
        inherited buildderef;
        fieldvarsymderef.build(fieldvarsym);
      end;


    procedure tstaticvarsym.deref;
      begin
        inherited deref;
        fieldvarsym:=tfieldvarsym(fieldvarsymderef.resolve);
      end;

    function tstaticvarsym.mangledname:TSymStr;
      var
        usename,
        prefix : TSymStr;
      begin
{$ifdef symansistr}
        if _mangledname='' then
{$else symansistr}
        if not assigned(_mangledname) then
{$endif symansistr}
          begin
            if (vo_is_typed_const in varoptions) then
              prefix:='TC'
            else
              prefix:='U';
{$ifdef symansistr}
            if _mangledbasename='' then
              usename:=name
            else
              usename:=_mangledbasename;
            _mangledname:=make_mangledname(prefix,owner,usename);
{$else symansistr}
            if not assigned(_mangledbasename) then
              usename:=name
            else
              usename:=_mangledbasename^;
            _mangledname:=stringdup(make_mangledname(prefix,owner,usename));
{$endif symansistr}
          end;
{$ifdef symansistr}
        result:=_mangledname;
{$else symansistr}
        result:=_mangledname^;
{$endif symansistr}
      end;


    procedure tstaticvarsym.set_mangledbasename(const s: TSymStr);
      begin
{$ifdef symansistr}
        _mangledbasename:=s;
        _mangledname:='';
{$else symansistr}
        stringdispose(_mangledname);
        stringdispose(_mangledbasename);
        _mangledbasename:=stringdup(s);
{$endif symansistr}
        include(varoptions,vo_has_mangledname);
      end;


    function tstaticvarsym.mangledbasename: TSymStr;
      begin
{$ifdef symansistr}
        result:=_mangledbasename;
{$else symansistr}
        if assigned(_mangledbasename) then
          result:=_mangledbasename^
        else
          result:='';
{$endif symansistr}
      end;


    procedure tstaticvarsym.set_mangledname(const s:TSymStr);
      begin
{$ifdef symansistr}
        _mangledname:=s;
{$else symansistr}
        stringdispose(_mangledname);
        _mangledname:=stringdup(s);
{$endif symansistr}
        include(varoptions,vo_has_mangledname);
      end;


    procedure tstaticvarsym.set_raw_mangledname(const s: TSymStr);
      begin
{$ifndef symansistr}
        stringdispose(_mangledname);
        _mangledname:=stringdup(s);
{$else}
        _mangledname:=s;
{$endif}
        include(varoptions,vo_has_mangledname);
      end;


{****************************************************************************
                               TLOCALVARSYM
****************************************************************************}

    constructor tlocalvarsym.create(const n : TSymStr;vsp:tvarspez;def:tdef;vopts:tvaroptions);
      begin
         inherited create(localvarsym,n,vsp,def,vopts);
      end;


    constructor tlocalvarsym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(localvarsym,ppufile);
         ppuload_platform(ppufile);
      end;


    procedure tlocalvarsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         writeentry(ppufile,iblocalvarsym);
      end;


{****************************************************************************
                              TPARAVARSYM
****************************************************************************}

    constructor tparavarsym.create(const n : TSymStr;nr:word;vsp:tvarspez;def:tdef;vopts:tvaroptions);
      begin
         inherited create(paravarsym,n,vsp,def,vopts);
         if (vsp in [vs_var,vs_value,vs_const,vs_constref]) and
            not(vo_is_funcret in vopts) then
           varstate := vs_initialised;
         paranr:=nr;
         paraloc[calleeside].init;
         paraloc[callerside].init;
      end;


    destructor tparavarsym.destroy;
      begin
        paraloc[calleeside].done;
        paraloc[callerside].done;
        inherited destroy;
      end;


    constructor tparavarsym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(paravarsym,ppufile);
         paranr:=ppufile.getword;
         univpara:=ppufile.getboolean;

         { The var state of parameter symbols is fixed after writing them so
           we write them to the unit file.
           This enables constant folding for inline procedures loaded from units
         }
         varstate:=tvarstate(ppufile.getbyte);

         { read usage info }
         refs:=ppufile.getbyte;

         paraloc[calleeside].init;
         paraloc[callerside].init;
         if vo_has_explicit_paraloc in varoptions then
           paraloc[callerside].ppuload(ppufile);

         ppuload_platform(ppufile);
      end;


    procedure tparavarsym.ppuwrite(ppufile:tcompilerppufile);
      var
        oldintfcrc : boolean;
      begin
         inherited ppuwrite(ppufile);
         ppufile.putword(paranr);
         ppufile.putboolean(univpara);

         { The var state of parameter symbols is fixed after writing them so
           we write them to the unit file.
           This enables constant folding for inline procedures loaded from units
         }
         oldintfcrc:=ppufile.do_crc;
         ppufile.do_crc:=false;
         ppufile.putbyte(ord(varstate));
         { write also info about the usage of parameters,
           the absolute usage does not matter }
         ppufile.putbyte(min(1,refs));
         ppufile.do_crc:=oldintfcrc;

         if vo_has_explicit_paraloc in varoptions then
           begin
             paraloc[callerside].check_simple_location;
             paraloc[callerside].ppuwrite(ppufile);
           end;
         writeentry(ppufile,ibparavarsym);
      end;

    function tparavarsym.needs_finalization:boolean;
      begin
        result:=(varspez=vs_value) and
          (is_managed_type(vardef) or
            (
              (not (tabstractprocdef(owner.defowner).proccalloption in cdecl_pocalls)) and
              (not paramanager.use_stackalloc) and
              (is_open_array(vardef) or is_array_of_const(vardef))
            )
          );
      end;


    function tparavarsym.is_used: boolean;
      begin
        { Only the $parentfp parameter is supported for now }
        result:=not (vo_is_parentfp in varoptions) or (varstate>vs_initialised);
      end;

{****************************************************************************
                               TABSOLUTEVARSYM
****************************************************************************}

    constructor tabsolutevarsym.create(const n : TSymStr;def:tdef);
      begin
        inherited create(absolutevarsym,n,vs_value,def,[]);
        ref:=nil;
      end;


    constructor tabsolutevarsym.create_ref(const n : TSymStr;def:tdef;_ref:tpropaccesslist);
      begin
        inherited create(absolutevarsym,n,vs_value,def,[]);
        ref:=_ref;
      end;


    destructor tabsolutevarsym.destroy;
      begin
        if assigned(ref) then
          ref.free;
        inherited destroy;
      end;


    constructor tabsolutevarsym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(absolutevarsym,ppufile);
         ref:=nil;
         asmname:=nil;
         abstyp:=absolutetyp(ppufile.getbyte);
         case abstyp of
           tovar :
             ref:=ppufile.getpropaccesslist;
           toasm :
             asmname:=ppufile.getpshortstring;
           toaddr :
             addroffset:=ppufile.getpuint;
         end;
         ppuload_platform(ppufile);
      end;


    procedure tabsolutevarsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(abstyp));
         case abstyp of
           tovar :
             ppufile.putpropaccesslist(ref);
           toasm :
             ppufile.putstring(asmname^);
           toaddr :
             ppufile.putpuint(addroffset);
         end;
         writeentry(ppufile,ibabsolutevarsym);
      end;


    procedure tabsolutevarsym.buildderef;
      begin
        inherited buildderef;
        if (abstyp=tovar) then
          ref.buildderef;
      end;


    procedure tabsolutevarsym.deref;
      begin
         inherited deref;
         { own absolute deref }
         if (abstyp=tovar) then
           ref.resolve;
      end;


    function tabsolutevarsym.mangledname : TSymStr;
      begin
         case abstyp of
           toasm :
             mangledname:=asmname^;
           toaddr :
             mangledname:='$'+tostr(addroffset);
           else
             internalerror(200411062);
         end;
      end;


{****************************************************************************
                                  TCONSTSYM
****************************************************************************}

    constructor tconstsym.create_ord(const n : TSymStr;t : tconsttyp;v : tconstexprint;def:tdef);
      begin
         inherited create(constsym,n);
         fillchar(value, sizeof(value), #0);
         consttyp:=t;
         value.valueord:=v;
         constdef:=def;
         constdefderef.reset;
      end;


    constructor tconstsym.create_ordptr(const n : TSymStr;t : tconsttyp;v : tconstptruint;def:tdef);
      begin
         inherited create(constsym,n);
         fillchar(value, sizeof(value), #0);
         consttyp:=t;
         value.valueordptr:=v;
         constdef:=def;
         constdefderef.reset;
      end;


    constructor tconstsym.create_ptr(const n : TSymStr;t : tconsttyp;v : pointer;def:tdef);
      begin
         inherited create(constsym,n);
         fillchar(value, sizeof(value), #0);
         consttyp:=t;
         value.valueptr:=v;
         constdef:=def;
         constdefderef.reset;
      end;


    constructor tconstsym.create_string(const n : TSymStr;t : tconsttyp;str:pchar;l:longint;def: tdef);
      begin
         inherited create(constsym,n);
         fillchar(value, sizeof(value), #0);
         consttyp:=t;
         value.valueptr:=str;
         if assigned(def) then
           constdef:=def
         else
           constdef:=carraydef.getreusable(cansichartype,l);
         constdefderef.reset;
         value.len:=l;
      end;


    constructor tconstsym.create_wstring(const n : TSymStr;t : tconsttyp;pw:pcompilerwidestring);
      begin
         inherited create(constsym,n);
         fillchar(value, sizeof(value), #0);
         consttyp:=t;
         pcompilerwidestring(value.valueptr):=pw;
         constdef:=carraydef.getreusable(cwidechartype,getlengthwidestring(pw));
         constdefderef.reset;
         value.len:=getlengthwidestring(pw);
      end;


    constructor tconstsym.create_undefined(const n : TSymStr;def: tdef);
      begin
        inherited create(constsym,n);
        fillchar(value,sizeof(value),#0);
        consttyp:=constnone;
        constdef:=def;
      end;


    constructor tconstsym.ppuload(ppufile:tcompilerppufile);
      var
         pd : pbestreal;
         ps : pnormalset;
         pc : pchar;
         pw : pcompilerwidestring;
         i  : longint;
      begin
         inherited ppuload(constsym,ppufile);
         constdef:=nil;
         consttyp:=tconsttyp(ppufile.getbyte);
         fillchar(value, sizeof(value), #0);
         case consttyp of
           constord :
             begin
               ppufile.getderef(constdefderef);
               value.valueord:=ppufile.getexprint;
             end;
           constpointer :
             begin
               ppufile.getderef(constdefderef);
               value.valueordptr:=ppufile.getptruint;
             end;
           constwstring :
             begin
               initwidestring(pw);
               setlengthwidestring(pw,ppufile.getlongint);
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
               pcompilerwidestring(value.valueptr):=pw;
             end;
           conststring,
           constresourcestring :
             begin
               ppufile.getderef(constdefderef);
               value.len:=ppufile.getlongint;
               getmem(pc,value.len+1);
               ppufile.getdata(pc^,value.len);
               pc[value.len]:=#0;
               value.valueptr:=pc;
             end;
           constreal :
             begin
               ppufile.getderef(constdefderef);
               new(pd);
               pd^:=ppufile.getreal;
               value.valueptr:=pd;
             end;
           constset :
             begin
               ppufile.getderef(constdefderef);
               new(ps);
               ppufile.getset(tppuset32(ps^));
               value.valueptr:=ps;
             end;
           constguid :
             begin
               ppufile.getderef(constdefderef);
               new(pguid(value.valueptr));
               ppufile.getdata(value.valueptr^,sizeof(tguid));
             end;
           constnil :
             ppufile.getderef(constdefderef);
           else
             Message1(unit_f_ppu_invalid_entry,tostr(ord(consttyp)));
         end;
         ppuload_platform(ppufile);
      end;


    destructor tconstsym.destroy;
      begin
        case consttyp of
          constnone,
          constord,
          constpointer,
          constnil:
            ;
          conststring,
          constresourcestring :
            freemem(pchar(value.valueptr),value.len+1);
          constwstring :
            donewidestring(pcompilerwidestring(value.valueptr));
          constreal :
            dispose(pbestreal(value.valueptr));
          constset :
            dispose(pnormalset(value.valueptr));
          constguid :
            dispose(pguid(value.valueptr));
        end;
        inherited destroy;
      end;


    procedure tconstsym.buildderef;
      begin
        inherited;
        case consttyp  of
          constnil,constord,constreal,constpointer,constset,conststring,constresourcestring,constguid:
            constdefderef.build(constdef);
          constwstring:
            ;
          else
            internalerror(2015120802);
        end;
      end;


    procedure tconstsym.deref;
      begin
        case consttyp of
          constnil,constord,constreal,constpointer,constset,conststring,constresourcestring,constguid:
            constdef:=tdef(constdefderef.resolve);
          constwstring:
            constdef:=carraydef.getreusable(cwidechartype,getlengthwidestring(pcompilerwidestring(value.valueptr)));
          else
            internalerror(2015120801);
        end
      end;


    procedure tconstsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putbyte(byte(consttyp));
         case consttyp of
           constnil :
             ppufile.putderef(constdefderef);
           constord :
             begin
               ppufile.putderef(constdefderef);
               ppufile.putexprint(value.valueord);
             end;
           constpointer :
             begin
               ppufile.putderef(constdefderef);
               ppufile.putptruint(value.valueordptr);
             end;
           constwstring :
             begin
               { no need to store the def, we can reconstruct it }
               ppufile.putlongint(getlengthwidestring(pcompilerwidestring(value.valueptr)));
               ppufile.putdata(pcompilerwidestring(value.valueptr)^.data^,pcompilerwidestring(value.valueptr)^.len*sizeof(tcompilerwidechar));
             end;
           conststring,
           constresourcestring :
             begin
               ppufile.putderef(constdefderef);
               ppufile.putlongint(value.len);
               ppufile.putdata(pchar(value.valueptr)^,value.len);
             end;
           constreal :
             begin
               ppufile.putderef(constdefderef);
               ppufile.putreal(pbestreal(value.valueptr)^);
             end;
           constset :
             begin
               ppufile.putderef(constdefderef);
               ppufile.putset(tppuset32(value.valueptr^));
             end;
           constguid :
             begin
               ppufile.putderef(constdefderef);
               ppufile.putdata(value.valueptr^,sizeof(tguid));
             end;
         else
           internalerror(13);
         end;
        writeentry(ppufile,ibconstsym);
      end;

{$ifdef DEBUG_NODE_XML}
    procedure TConstSym.XMLPrintConstData(var T: Text);
      begin
        WriteLn(T, PrintNodeIndention, '<type>', SanitiseXMLString(constdef.GetTypeName), '</type>');

        case consttyp of
          constnone:
            ;
          conststring,
          constresourcestring,
          constwstring:
            begin
              WriteLn(T, PrintNodeIndention, '<length>', value.len, '</length>');
              if value.len = 0 then
                WriteLn(T, PrintNodeIndention, '<value />')
              else
                WriteLn(T, PrintNodeIndention, '<value>', SanitiseXMLString(PChar(value.valueptr)), '</value>');
            end;
          constord,
          constset:
            WriteLn(T, PrintNodeIndention, '<value>', tostr(value.valueord), '</value>');
          constpointer:
            WriteLn(T, PrintNodeIndention, '<value>', WriteConstPUInt(value.valueordptr), '</value>');
          constreal:
            WriteLn(T, PrintNodeIndention, '<value>', PBestReal(value.valueptr)^, '</value>');
          constnil:
            WriteLn(T, PrintNodeIndention, '<value>nil</value>');
          constguid:
            WriteLn(T, PrintNodeIndention, '<value>', WriteGUID(PGUID(value.valueptr)^), '</value>');
        end;

        WriteLn(T, PrintNodeIndention, '<visibility>', visibility, '</visibility>');

        if not (consttyp in [conststring, constresourcestring, constwstring]) then
          { constdef.size will return an internal error for string
            constants because constdef is an open array internally }
          WriteLn(T, PrintNodeIndention, '<size>', constdef.size, '</size>');

//        WriteLn(T, PrintNodeIndention, '<const_type>', consttyp, '</const_type>');
      end;
{$endif DEBUG_NODE_XML}

{****************************************************************************
                                  TENUMSYM
****************************************************************************}

    constructor tenumsym.create(const n : TSymStr;def : tenumdef;v : longint);
      begin
         inherited create(enumsym,n);
         definition:=def;
         definitionderef.reset;
         value:=v;
      end;


    constructor tenumsym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(enumsym,ppufile);
         ppufile.getderef(definitionderef);
         value:=ppufile.getlongint;
         ppuload_platform(ppufile);
      end;


    procedure tenumsym.buildderef;
      begin
        inherited;
        definitionderef.build(definition);
      end;


    procedure tenumsym.deref;
      begin
         definition:=tenumdef(definitionderef.resolve);
      end;

    procedure tenumsym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putderef(definitionderef);
         ppufile.putlongint(value);
         writeentry(ppufile,ibenumsym);
      end;


{****************************************************************************
                                  TTYPESYM
****************************************************************************}


    constructor ttypesym.create(const n : TSymStr;def:tdef);

      begin
        inherited create(typesym,n);
        typedef:=def;
        typedefderef.reset;
        { register the typesym for the definition }
        if assigned(typedef) and
           (typedef.typ<>errordef) and
           not(assigned(typedef.typesym)) then
         typedef.typesym:=self;
      end;

    destructor ttypesym.destroy;
      begin
        inherited destroy;
      end;


    constructor ttypesym.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(typesym,ppufile);
         ppufile.getderef(typedefderef);
         fprettyname:=ppufile.getansistring;
         ppuload_platform(ppufile);
      end;


    procedure ttypesym.buildderef;
      begin
        inherited;
        typedefderef.build(typedef);
      end;


    procedure ttypesym.deref;
      begin
        typedef:=tdef(typedefderef.resolve);
      end;


    procedure ttypesym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putderef(typedefderef);
         ppufile.putansistring(fprettyname);
         writeentry(ppufile,ibtypesym);
      end;


    function ttypesym.prettyname : string;
      begin
        if fprettyname<>'' then
          result:=fprettyname
        else
          result:=inherited prettyname;
      end;


{****************************************************************************
                                  TSYSSYM
****************************************************************************}


    var
      syssym_list : TFPHashObjectList;


    constructor tsyssym.create(const n : TSymStr;l : tinlinenumber);
      var
        s : shortstring;
      begin
         inherited create(syssym,n);
         number:=l;
         str(longint(l),s);
         if assigned(syssym_list.find(s)) then
           internalerror(2016060303);
         syssym_list.add(s,self);
      end;

    constructor tsyssym.ppuload(ppufile:tcompilerppufile);
      var
        s : shortstring;
      begin
         inherited ppuload(syssym,ppufile);
         number:=tinlinenumber(ppufile.getlongint);
         ppuload_platform(ppufile);
         str(longint(number),s);
         if assigned(syssym_list.find(s)) then
           internalerror(2016060304);
         syssym_list.add(s,self);
      end;

    destructor tsyssym.destroy;
      begin
        inherited destroy;
      end;

    procedure tsyssym.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putlongint(longint(number));
         writeentry(ppufile,ibsyssym);
      end;


    class function tsyssym.find_by_number(l:longint):tsyssym;
      var
        s : shortstring;
      begin
        str(l,s);
        result:=tsyssym(syssym_list.find(s));
      end;

{*****************************************************************************
                                 TMacro
*****************************************************************************}

    constructor tmacro.create(const n : TSymStr);
      begin
         inherited create(macrosym,n);
         owner:=nil;
         defined:=false;
         is_used:=false;
         is_compiler_var:=false;
         buftext:=nil;
         buflen:=0;
      end;

    constructor tmacro.ppuload(ppufile:tcompilerppufile);
      begin
         inherited ppuload(macrosym,ppufile);
         defined:=ppufile.getboolean;
         is_compiler_var:=ppufile.getboolean;
         is_used:=false;
         buflen:= ppufile.getlongint;
         if buflen > 0 then
           begin
             getmem(buftext, buflen);
             ppufile.getdata(buftext^, buflen)
           end
         else
           buftext:=nil;
      end;

    destructor tmacro.destroy;
      begin
         if assigned(buftext) then
           freemem(buftext);
         inherited destroy;
      end;

    procedure tmacro.ppuwrite(ppufile:tcompilerppufile);
      begin
         inherited ppuwrite(ppufile);
         ppufile.putboolean(defined);
         ppufile.putboolean(is_compiler_var);
         ppufile.putlongint(buflen);
         if buflen > 0 then
           ppufile.putdata(buftext^,buflen);
         writeentry(ppufile,ibmacrosym);
      end;


    function tmacro.GetCopy:tmacro;
      var
        p : tmacro;
      begin
        p:=tmacro.create(realname);
        p.defined:=defined;
        p.is_used:=is_used;
        p.is_compiler_var:=is_compiler_var;
        p.buflen:=buflen;
        if assigned(buftext) then
          begin
            getmem(p.buftext,buflen);
            move(buftext^,p.buftext^,buflen);
          end;
        Result:=p;
      end;


    procedure init_symsym;
      begin
        syssym_list:=tfphashobjectlist.create(false);
      end;


    procedure done_symsym;
      begin
        syssym_list.free;
      end;


initialization
  register_initdone_proc(@init_symsym,@done_symsym);
end.
