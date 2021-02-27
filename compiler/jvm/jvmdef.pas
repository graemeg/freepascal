{
    Copyright (c) 2010 by Jonas Maebe

    This unit implements some JVM type helper routines (minimal
    unit dependencies, usable in symdef).

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

{$i fpcdefs.inc}

unit jvmdef;

interface

    uses
      globtype,
      node,
      symbase,symtype,symdef;

    { returns whether a def can make use of an extra type signature (for
      Java-style generics annotations; not use for FPC-style generics or their
      translations, but to annotate the kind of classref a java.lang.Class is
      and things like that) }
    function jvmtypeneedssignature(def: tdef): boolean;
    { create a signature encoding of a particular type; requires that
      jvmtypeneedssignature returned "true" for this type }
    procedure jvmaddencodedsignature(def: tdef; bpacked: boolean; var encodedstr: TSymStr);

    { Encode a type into the internal format used by the JVM (descriptor).
      Returns false if a type is not representable by the JVM,
      and in that case also the failing definition.  }
    function jvmtryencodetype(def: tdef; out encodedtype: TSymStr; forcesignature: boolean; out founderror: tdef): boolean;

    { same as above, but throws an internal error on failure }
    function jvmencodetype(def: tdef; withsignature: boolean): TSymStr;

    { Check whether a type can be used in a JVM methom signature or field
      declaration.  }
    function jvmchecktype(def: tdef; out founderror: tdef): boolean;

    { incremental version of jvmtryencodetype() }
    function jvmaddencodedtype(def: tdef; bpacked: boolean; var encodedstr: TSymStr; forcesignature: boolean; out founderror: tdef): boolean;

    { add type prefix (package name) to a type }
    procedure jvmaddtypeownerprefix(owner: tsymtable; var name: TSymStr);

    { returns type string for a single-dimensional array (different from normal
      typestring in case of a primitive type) }
    function jvmarrtype(def: tdef; out primitivetype: boolean): TSymStr;
    function jvmarrtype_setlength(def: tdef): char;

    { returns whether a def is emulated using an implicit pointer type on the
      JVM target (e.g., records, regular arrays, ...) }
    function jvmimplicitpointertype(def: tdef): boolean;

    { returns the mangled base name for a tsym (type + symbol name, no
      visibility etc); also adds signature attribute if requested and
      appropriate }
    function jvmmangledbasename(sym: tsym; withsignature: boolean): TSymStr;
    function jvmmangledbasename(sym: tsym; const usesymname: TSymStr; withsignature: boolean): TSymStr;

    { sometimes primitive types have to be boxed/unboxed via class types. This
      routine returns the appropriate box type for the passed primitive type }
    procedure jvmgetboxtype(def: tdef; out objdef, paradef: tdef; mergeints: boolean);
    function jvmgetunboxmethod(def: tdef): string;

    function jvmgetcorrespondingclassdef(def: tdef): tdef;

    function get_para_push_size(def: tdef): tdef;

    { threadvars are wrapped via descendents of java.lang.ThreadLocal }
    function jvmgetthreadvardef(def: tdef): tdef;

    { gets the number of dimensions and the final element type of a normal
      array }
    procedure jvmgetarraydimdef(arrdef: tdef; out eledef: tdef; out ndim: longint);

    { the JVM specs require that you add a default parameterless
      constructor in case the programmer hasn't specified any }
    procedure maybe_add_public_default_java_constructor(obj: tabstractrecorddef);


implementation

  uses
    cutils,cclasses,constexp,
    verbose,systems,
    fmodule,
    symtable,symconst,symsym,symcpu,symcreat,
    pparautl,
    defutil,paramgr;

{******************************************************************
                          Type encoding
*******************************************************************}

    function jvmtypeneedssignature(def: tdef): boolean;
      var
        i: longint;
      begin
        result:=false;
        case def.typ of
          classrefdef,
          setdef:
            begin
              result:=true;
            end;
          arraydef :
            begin
              result:=jvmtypeneedssignature(tarraydef(def).elementdef);
            end;
          procvardef :
            begin
              { may change in the future }
            end;
          procdef :
            begin
              for i:=0 to tprocdef(def).paras.count-1 do
                begin
                  result:=jvmtypeneedssignature(tparavarsym(tprocdef(def).paras[i]).vardef);
                  if result then
                    exit;
                end;
            end
          else
            result:=false;
        end;
      end;


    procedure jvmaddencodedsignature(def: tdef; bpacked: boolean; var encodedstr: TSymStr);
      var
        founderror: tdef;
      begin
        case def.typ of
          pointerdef :
            begin
              { maybe one day }
              internalerror(2011051403);
            end;
          classrefdef :
            begin
              { Ljava/lang/Class<+SomeClassType> means
                "Ljava/lang/Class<SomeClassType_or_any_of_its_descendents>" }
              encodedstr:=encodedstr+'Ljava/lang/Class<+';
              jvmaddencodedtype(tclassrefdef(def).pointeddef,false,encodedstr,true,founderror);
              encodedstr:=encodedstr+'>;';
            end;
          setdef :
            begin
              if tsetdef(def).elementdef.typ=enumdef then
                begin
                  encodedstr:=encodedstr+'Ljava/util/EnumSet<';
                  jvmaddencodedtype(tenumdef(tsetdef(def).elementdef).getbasedef,false,encodedstr,true,founderror);
                  encodedstr:=encodedstr+'>;';
                end
              else
                internalerror(2011051404);
            end;
          arraydef :
            begin
              if is_array_of_const(def) then
                begin
                  internalerror(2011051405);
                end
              else if is_packed_array(def) then
                begin
                  internalerror(2011051406);
                end
              else
                begin
                  encodedstr:=encodedstr+'[';
                  jvmaddencodedsignature(tarraydef(def).elementdef,false,encodedstr);
                end;
            end;
          procvardef :
            begin
              { maybe one day }
              internalerror(2011051407);
            end;
          objectdef :
            begin
              { maybe one day }
            end;
          undefineddef,
          errordef :
            begin
              internalerror(2011051408);
            end;
          procdef :
            { must be done via jvmencodemethod() }
            internalerror(2011051401);
        else
          internalerror(2011051402);
        end;
      end;


    function jvmaddencodedtype(def: tdef; bpacked: boolean; var encodedstr: TSymStr; forcesignature: boolean; out founderror: tdef): boolean;
      var
        c: char;
      begin
        result:=true;
        case def.typ of
          stringdef :
            begin
              case tstringdef(def).stringtype of
                { translated into java.lang.String }
                st_widestring,
                st_unicodestring:
                  result:=jvmaddencodedtype(java_jlstring,false,encodedstr,forcesignature,founderror);
                st_ansistring:
                  result:=jvmaddencodedtype(java_ansistring,false,encodedstr,forcesignature,founderror);
                st_shortstring:
                  result:=jvmaddencodedtype(java_shortstring,false,encodedstr,forcesignature,founderror);
                else
                  { May be handled via wrapping later  }
                  result:=false;
              end;
            end;
          enumdef:
            begin
              result:=jvmaddencodedtype(tcpuenumdef(tenumdef(def).getbasedef).classdef,false,encodedstr,forcesignature,founderror);
            end;
          orddef :
            begin
              { for procedure "results" }
              if is_void(def) then
                c:='V'
              { only Pascal-style booleans conform to Java's definition of
                Boolean }
              else if is_pasbool(def) and
                      (def.size=1) then
                c:='Z'
              else if is_widechar(def) then
                c:='C'
              else
                begin
                  case def.size of
                    1:
                      c:='B';
                    2:
                      c:='S';
                    4:
                      c:='I';
                    8:
                      c:='J';
                    else
                      internalerror(2010121905);
                  end;
                end;
              encodedstr:=encodedstr+c;
            end;
          pointerdef :
            begin
              if is_voidpointer(def) then
                result:=jvmaddencodedtype(java_jlobject,false,encodedstr,forcesignature,founderror)
              else if jvmimplicitpointertype(tpointerdef(def).pointeddef) then
                result:=jvmaddencodedtype(tpointerdef(def).pointeddef,false,encodedstr,forcesignature,founderror)
              else
                begin
                  { all pointer types are emulated via arrays }
                  encodedstr:=encodedstr+'[';
                  result:=jvmaddencodedtype(tpointerdef(def).pointeddef,false,encodedstr,forcesignature,founderror);
                end
            end;
          floatdef :
            begin
              case tfloatdef(def).floattype of
                s32real:
                  c:='F';
                s64real:
                  c:='D';
                else
                  begin
                    result:=false;
                    c:=' ';
                  end;
              end;
              encodedstr:=encodedstr+c;
            end;
          filedef :
            begin
              case tfiledef(def).filetyp of
                ft_text:
                  result:=jvmaddencodedtype(search_system_type('TEXTREC').typedef,false,encodedstr,forcesignature,founderror);
                ft_typed,
                ft_untyped:
                  result:=jvmaddencodedtype(search_system_type('FILEREC').typedef,false,encodedstr,forcesignature,founderror);
              end;
            end;
          recorddef :
            begin
              encodedstr:=encodedstr+'L'+trecorddef(def).jvm_full_typename(true)+';'
            end;
          variantdef :
            begin
              { will be hanlded via wrapping later, although wrapping may
                happen at higher level }
              result:=false;
            end;
          classrefdef :
            begin
              if not forcesignature then
                { unfortunately, java.lang.Class is final, so we can't create
                  different versions for difference class reference types }
                encodedstr:=encodedstr+'Ljava/lang/Class;'
              { we can however annotate it with extra signature information in
                using Java's generic annotations }
              else
                jvmaddencodedsignature(def,false,encodedstr);
              result:=true;
            end;
          setdef :
            begin
              if tsetdef(def).elementdef.typ=enumdef then
                begin
                  if forcesignature then
                    jvmaddencodedsignature(def,false,encodedstr)
                  else
                    result:=jvmaddencodedtype(java_juenumset,false,encodedstr,forcesignature,founderror)
                end
              else
                result:=jvmaddencodedtype(java_jubitset,false,encodedstr,forcesignature,founderror)
            end;
          formaldef :
            begin
              { var/const/out x: JLObject }
              result:=jvmaddencodedtype(java_jlobject,false,encodedstr,forcesignature,founderror);
            end;
          arraydef :
            begin
              if is_array_of_const(def) then
                begin
                  encodedstr:=encodedstr+'[';
                  result:=jvmaddencodedtype(search_system_type('TVARREC').typedef,false,encodedstr,forcesignature,founderror);
                end
              else if is_packed_array(def) then
                result:=false
              else
                begin
                  encodedstr:=encodedstr+'[';
                  if not jvmaddencodedtype(tarraydef(def).elementdef,false,encodedstr,forcesignature,founderror) then
                    begin
                      result:=false;
                      { report the exact (nested) error defintion }
                      exit;
                    end;
                end;
            end;
          procvardef :
            begin
              result:=jvmaddencodedtype(tcpuprocvardef(def).classdef,false,encodedstr,forcesignature,founderror);
            end;
          objectdef :
            case tobjectdef(def).objecttype of
              odt_javaclass,
              odt_interfacejava:
                begin
                  def:=maybe_find_real_class_definition(def,false);
                  encodedstr:=encodedstr+'L'+tobjectdef(def).jvm_full_typename(true)+';'
                end
              else
                result:=false;
            end;
          undefineddef,
          errordef :
            result:=false;
          procdef :
            { must be done via jvmencodemethod() }
            internalerror(2010121903);
        else
          internalerror(2010121904);
        end;
        if not result then
          founderror:=def;
      end;


    function jvmtryencodetype(def: tdef; out encodedtype: TSymStr; forcesignature: boolean; out founderror: tdef): boolean;
      begin
        encodedtype:='';
        result:=jvmaddencodedtype(def,false,encodedtype,forcesignature,founderror);
      end;


    procedure jvmaddtypeownerprefix(owner: tsymtable; var name: TSymStr);
      var
        owningcontainer: tsymtable;
        tmpresult: TSymStr;
        module: tmodule;
        nameendpos: longint;
      begin
        { see tprocdef.jvmmangledbasename for description of the format }
        owningcontainer:=owner;
        while (owningcontainer.symtabletype=localsymtable) do
          owningcontainer:=owningcontainer.defowner.owner;
        case owningcontainer.symtabletype of
          globalsymtable,
          staticsymtable:
            begin
              module:=find_module_from_symtable(owningcontainer);
              tmpresult:='';
              if assigned(module.namespace) then
                tmpresult:=module.namespace^+'/';
              tmpresult:=tmpresult+module.realmodulename^+'/';
            end;
          objectsymtable:
            case tobjectdef(owningcontainer.defowner).objecttype of
              odt_javaclass,
              odt_interfacejava:
                begin
                  tmpresult:=tobjectdef(owningcontainer.defowner).jvm_full_typename(true)+'/'
                end
              else
                internalerror(2010122606);
            end;
          recordsymtable:
            tmpresult:=trecorddef(owningcontainer.defowner).jvm_full_typename(true)+'/'
          else
            internalerror(2010122605);
        end;
        name:=tmpresult+name;
        nameendpos:=pos(' ',name);
        if nameendpos=0 then
          nameendpos:=length(name)+1;
        insert('''',name,nameendpos);
        name:=''''+name;
      end;


    function jvmarrtype(def: tdef; out primitivetype: boolean): TSymStr;
      var
        errdef: tdef;
      begin
        if not jvmtryencodetype(def,result,false,errdef) then
          internalerror(2011012201);
        primitivetype:=false;
        if length(result)=1 then
          begin
            case result[1] of
              'Z': result:='boolean';
              'C': result:='char';
              'B': result:='byte';
              'S': result:='short';
              'I': result:='int';
              'J': result:='long';
              'F': result:='float';
              'D': result:='double';
              else
                internalerror(2011012203);
              end;
            primitivetype:=true;
          end
        else if (result[1]='L') then
          begin
            { in case of a class reference, strip the leading 'L' and the
              trailing ';' }
            setlength(result,length(result)-1);
            delete(result,1,1);
          end;
        { for arrays, use the actual reference type }
      end;


    function jvmarrtype_setlength(def: tdef): char;
      var
        errdef: tdef;
        res: TSymStr;
      begin
        { keep in sync with rtl/java/jdynarrh.inc and usage in njvminl }
        if is_record(def) then
          result:='R'
        else if is_shortstring(def) then
          result:='T'
        else if def.typ=setdef then
          begin
            if tsetdef(def).elementdef.typ=enumdef then
              result:='E'
            else
              result:='L'
          end
        else if (def.typ=procvardef) and
                not tprocvardef(def).is_addressonly then
          result:='P'
        else
          begin
            if not jvmtryencodetype(def,res,false,errdef) then
              internalerror(2011012209);
            if length(res)=1 then
              result:=res[1]
            else
              result:='A';
          end;
      end;


    function jvmimplicitpointertype(def: tdef): boolean;
      begin
        case def.typ of
          arraydef:
            result:=(tarraydef(def).highrange>=tarraydef(def).lowrange) or
                is_open_array(def) or
                is_array_of_const(def) or
                is_array_constructor(def) or
                is_conststring_array(def);
          filedef,
          recorddef,
          setdef:
            result:=true;
          objectdef:
            result:=is_object(def);
          stringdef :
            result:=tstringdef(def).stringtype in [st_shortstring,st_longstring];
          procvardef:
            result:=not tprocvardef(def).is_addressonly;
          else
            result:=false;
        end;
      end;


    { mergeints = true means that all integer types are mapped to jllong,
      otherwise they are mapped to the closest corresponding type }
    procedure jvmgetboxtype(def: tdef; out objdef, paradef: tdef; mergeints: boolean);
      begin
        case def.typ of
          orddef:
            begin
              case torddef(def).ordtype of
                pasbool1,
                pasbool8:
                  begin
                    objdef:=tobjectdef(search_system_type('JLBOOLEAN').typedef);
                    paradef:=pasbool8type;
                  end;
                uwidechar:
                  begin
                    objdef:=tobjectdef(search_system_type('JLCHARACTER').typedef);
                    paradef:=cwidechartype;
                  end;
                else
                  begin
                    { wrap all integer types into a JLLONG, so that we don't get
                      errors after returning a byte assigned to a long etc }
                    if mergeints or
                       (torddef(def).ordtype in [s64bit,u64bit,scurrency,bool64bit,pasbool64]) then
                      begin
                        objdef:=tobjectdef(search_system_type('JLLONG').typedef);
                        paradef:=s64inttype;
                      end
                    else
                      begin
                        case torddef(def).ordtype of
                          s8bit,
                          u8bit,
                          uchar,
                          bool8bit:
                            begin
                              objdef:=tobjectdef(search_system_type('JLBYTE').typedef);
                              paradef:=s8inttype;
                            end;
                          s16bit,
                          u16bit,
                          bool16bit,
                          pasbool16:
                            begin
                              objdef:=tobjectdef(search_system_type('JLSHORT').typedef);
                              paradef:=s16inttype;
                            end;
                          s32bit,
                          u32bit,
                          bool32bit,
                          pasbool32:
                            begin
                              objdef:=tobjectdef(search_system_type('JLINTEGER').typedef);
                              paradef:=s32inttype;
                            end;
                          else
                            internalerror(2011052101);
                        end;
                      end;
                  end;
              end;
            end;
          floatdef:
            begin
              case tfloatdef(def).floattype of
                s32real:
                  begin
                    objdef:=tobjectdef(search_system_type('JLFLOAT').typedef);
                    paradef:=s32floattype;
                  end;
                s64real:
                  begin
                    objdef:=tobjectdef(search_system_type('JLDOUBLE').typedef);
                    paradef:=s64floattype;
                  end;
                else
                  internalerror(2011052102);
              end;
            end;
          else
            internalerror(2011052103);
        end;
      end;


    function jvmgetunboxmethod(def: tdef): string;
      begin
        case def.typ of
          orddef:
            begin
              case torddef(def).ordtype of
                pasbool1,
                pasbool8:
                  result:='BOOLEANVALUE';
                s8bit,
                u8bit,
                uchar,
                bool8bit:
                  result:='BYTEVALUE';
                s16bit,
                u16bit,
                bool16bit,
                pasbool16:
                  result:='SHORTVALUE';
                s32bit,
                u32bit,
                bool32bit,
                pasbool32:
                  result:='INTVALUE';
                s64bit,
                u64bit,
                scurrency,
                bool64bit,
                pasbool64:
                  result:='LONGVALUE';
                uwidechar:
                  result:='CHARVALUE';
                else
                  internalerror(2011071702);
              end;
            end;
          floatdef:
            begin
              case tfloatdef(def).floattype of
                s32real:
                  result:='FLOATVALUE';
                s64real:
                  result:='DOUBLEVALUE';
                else
                  internalerror(2011071703);
              end;
            end;
          else
            internalerror(2011071704);
        end;
      end;


    function jvmgetcorrespondingclassdef(def: tdef): tdef;
      var
        paradef: tdef;
      begin
        if def.typ in [orddef,floatdef] then
          jvmgetboxtype(def,result,paradef,false)
        else
          begin
            case def.typ of
              stringdef :
                begin
                  case tstringdef(def).stringtype of
                    { translated into java.lang.String }
                    st_widestring,
                    st_unicodestring:
                      result:=java_jlstring;
                    st_ansistring:
                      result:=java_ansistring;
                    st_shortstring:
                      result:=java_shortstring;
                    else
                      internalerror(2011072409);
                  end;
                end;
              enumdef:
                begin
                  result:=tcpuenumdef(tenumdef(def).getbasedef).classdef;
                end;
              pointerdef :
                begin
                  if def=voidpointertype then
                    result:=java_jlobject
                  else if jvmimplicitpointertype(tpointerdef(def).pointeddef) then
                    result:=tpointerdef(def).pointeddef
                  else
                    internalerror(2011072410);
                end;
              recorddef :
                begin
                  result:=def;
                end;
              variantdef :
                begin
                  result:=cvarianttype;
                end;
              classrefdef :
                begin
                  result:=search_system_type('JLCLASS').typedef;
                end;
              setdef :
                begin
                  if tsetdef(def).elementdef.typ=enumdef then
                    result:=java_juenumset
                  else
                    result:=java_jubitset;
                end;
              formaldef :
                begin
                  result:=java_jlobject;
                end;
              arraydef :
                begin
                  { cannot represent statically }
                  internalerror(2011072411);
                end;
              procvardef :
                begin
                  result:=tcpuprocvardef(def).classdef;
                end;
              objectdef :
                case tobjectdef(def).objecttype of
                  odt_javaclass,
                  odt_interfacejava:
                    result:=def
                  else
                    internalerror(2011072412);
                end;
              else
                internalerror(2011072413);
            end;
          end;
      end;


  function get_para_push_size(def: tdef): tdef;
    begin
      result:=def;
      if def.typ=orddef then
        case torddef(def).ordtype of
          u8bit,uchar:
            if torddef(def).high>127 then
              result:=s8inttype;
          u16bit:
            begin
              if torddef(def).high>32767 then
                result:=s16inttype;
            end
          else
            ;
        end;
    end;


    function jvmgetthreadvardef(def: tdef): tdef;
      begin
        if (def.typ=arraydef) and
           not is_dynamic_array(def) then
          begin
            result:=search_system_type('FPCNORMALARRAYTHREADVAR').typedef;
            exit;
          end;
        if jvmimplicitpointertype(def) then
          begin
            result:=search_system_type('FPCIMPLICITPTRTHREADVAR').typedef;
            exit;
          end;
        case def.typ of
          orddef:
            begin
              case torddef(def).ordtype of
                pasbool1,
                pasbool8:
                  begin
                    result:=tobjectdef(search_system_type('FPCBOOLEANTHREADVAR').typedef);
                  end;
                uwidechar:
                  begin
                    result:=tobjectdef(search_system_type('FPCCHARTHREADVAR').typedef);
                  end;
                s8bit,
                u8bit,
                uchar,
                bool8bit:
                  begin
                    result:=tobjectdef(search_system_type('FPCBYTETHREADVAR').typedef);
                  end;
                s16bit,
                u16bit,
                bool16bit,
                pasbool16:
                  begin
                    result:=tobjectdef(search_system_type('FPCSHORTTHREADVAR').typedef);
                  end;
                s32bit,
                u32bit,
                bool32bit,
                pasbool32:
                  begin
                    result:=tobjectdef(search_system_type('FPCINTTHREADVAR').typedef);
                  end;
                s64bit,
                u64bit,
                scurrency,
                bool64bit,
                pasbool64:
                  begin
                    result:=tobjectdef(search_system_type('FPCLONGTHREADVAR').typedef);
                  end
                else
                  internalerror(2011082101);
              end;
            end;
          floatdef:
            begin
              case tfloatdef(def).floattype of
                s32real:
                  begin
                    result:=tobjectdef(search_system_type('FPCFLOATTHREADVAR').typedef);
                  end;
                s64real:
                  begin
                    result:=tobjectdef(search_system_type('FPCDOUBLETHREADVAR').typedef);
                  end;
                else
                  internalerror(2011082102);
              end;
            end
          else
            begin
              result:=search_system_type('FPCPOINTERTHREADVAR').typedef
            end;
        end;
      end;


    procedure jvmgetarraydimdef(arrdef: tdef; out eledef: tdef; out ndim: longint);
      begin
        eledef:=arrdef;
        ndim:=0;
        repeat
          eledef:=tarraydef(eledef).elementdef;
          inc(ndim);
        until (eledef.typ<>arraydef) or
              is_dynamic_array(eledef);
      end;



    function jvmmangledbasename(sym: tsym; const usesymname: TSymStr; withsignature: boolean): TSymStr;
      var
        container: tsymtable;
        vsym: tabstractvarsym;
        csym: tconstsym;
        usedef: tdef;
      begin
        case sym.typ of
          staticvarsym,
          paravarsym,
          localvarsym,
          fieldvarsym:
            begin
              vsym:=tabstractvarsym(sym);
              { for local and paravarsyms that are unsigned 8/16 bit, change the
                outputted type to signed 16/32 bit:
                  a) the stack slots are all 32 bit anyway, so the storage allocation
                     is still correct
                  b) since at the JVM level all types are signed, this makes sure
                     that the values in the stack slots are valid for the specified
                     types
              }
              usedef:=vsym.vardef;
              if vsym.typ in [localvarsym,paravarsym] then
                begin
                  if (usedef.typ=orddef) then
                    case torddef(usedef).ordtype of
                      u8bit,uchar:
                        usedef:=s16inttype;
                      u16bit:
                        usedef:=s32inttype;
                      else
                        ;
                    end;
                end;
              result:=jvmencodetype(usedef,false);
              if withsignature and
                 jvmtypeneedssignature(usedef) then
                begin
                  result:=result+' signature "';
                  result:=result+jvmencodetype(usedef,true)+'"';
                end;
              if (vsym.typ=paravarsym) and
                 (vo_is_self in tparavarsym(vsym).varoptions) then
                result:='''this'' ' +result
              else if (vsym.typ in [paravarsym,localvarsym]) and
                      ([vo_is_funcret,vo_is_result] * tabstractnormalvarsym(vsym).varoptions <> []) then
                result:='''result'' '+result
              else
                begin
                  { add array indirection if required }
                  if (vsym.typ=paravarsym) and
                     ((usedef.typ=formaldef) or
                      ((vsym.varspez in [vs_var,vs_out,vs_constref]) and
                       not jvmimplicitpointertype(usedef))) then
                    result:='['+result;
                  { single quotes for definitions to prevent clashes with Java
                    opcodes }
                  if withsignature then
                    result:=usesymname+''' '+result
                  else
                    result:=usesymname+' '+result;
                  { we have to mangle staticvarsyms in localsymtables to
                    prevent name clashes... }
                  if (vsym.typ=staticvarsym) then
                    begin
                      container:=sym.Owner;
                      while (container.symtabletype=localsymtable) do
                        begin
                          if tdef(container.defowner).typ<>procdef then
                            internalerror(2011040303);
                          { unique_id_str is added to prevent problem with overloads }
                          result:=tprocdef(container.defowner).procsym.realname+'$$'+tprocdef(container.defowner).unique_id_str+'$'+result;
                          container:=container.defowner.owner;
                        end;
                    end;
                  if withsignature then
                    result:=''''+result
                end;
            end;
          constsym:
            begin
              csym:=tconstsym(sym);
              { some constants can be untyped }
              if assigned(csym.constdef) and
                 not(csym.consttyp in [constwstring,conststring]) then
                begin
                  result:=jvmencodetype(csym.constdef,false);
                  if withsignature and
                     jvmtypeneedssignature(csym.constdef) then
                    begin
                      result:=result+' signature "';
                      result:=result+jvmencodetype(csym.constdef,true)+'"';
                    end;
                end
              else
                begin
                  case csym.consttyp of
                    constord:
                      result:=jvmencodetype(s32inttype,withsignature);
                    constreal:
                      result:=jvmencodetype(s64floattype,withsignature);
                    constset:
                      internalerror(2011040701);
                    constpointer,
                    constnil:
                      result:=jvmencodetype(java_jlobject,withsignature);
                    constwstring,
                    conststring:
                      result:=jvmencodetype(java_jlstring,withsignature);
                    constresourcestring:
                      internalerror(2011040702);
                    else
                      internalerror(2011040703);
                  end;
                end;
              if withsignature then
                result:=''''+usesymname+''' '+result
              else
                result:=usesymname+' '+result
            end;
          else
            internalerror(2011021703);
        end;
      end;


    function jvmmangledbasename(sym: tsym; withsignature: boolean): TSymStr;
      begin
        if (sym.typ=fieldvarsym) and
           assigned(tfieldvarsym(sym).externalname) then
          result:=jvmmangledbasename(sym,tfieldvarsym(sym).externalname^,withsignature)
        else if (sym.typ=staticvarsym) and
           (tstaticvarsym(sym).mangledbasename<>'') then
          result:=jvmmangledbasename(sym,tstaticvarsym(sym).mangledbasename,withsignature)
        else
          result:=jvmmangledbasename(sym,sym.RealName,withsignature);
      end;

{******************************************************************
                    jvm type validity checking
*******************************************************************}

   function jvmencodetype(def: tdef; withsignature: boolean): TSymStr;
     var
       errordef: tdef;
     begin
       if not jvmtryencodetype(def,result,withsignature,errordef) then
         internalerror(2011012305);
     end;


   function jvmchecktype(def: tdef; out founderror: tdef): boolean;
      var
        encodedtype: TSymStr;
      begin
        { don't duplicate the code like in objcdef, since the resulting strings
          are much shorter here so it's not worth it }
        result:=jvmtryencodetype(def,encodedtype,false,founderror);
      end;



{******************************************************************
                   Adding extra methods
*******************************************************************}
    procedure maybe_add_public_default_java_constructor(obj: tabstractrecorddef);
      var
        sym: tsym;
        ps: tprocsym;
        pd: tprocdef;
        topowner: tdefentry;
        i: longint;
        sstate: tscannerstate;
        needclassconstructor: boolean;
      begin
        ps:=nil;
        { if there is at least one constructor for a class, do nothing (for
           records, we'll always also need a parameterless constructor) }
        if not is_javaclass(obj) or
           not (oo_has_constructor in obj.objectoptions) then
          begin
            { check whether the parent has a parameterless constructor that we can
              call (in case of a class; all records will derive from
              java.lang.Object or a shim on top of that with a parameterless
              constructor) }
            if is_javaclass(obj) then
              begin
                pd:=nil;
                { childof may not be assigned in case of a parser error }
                if not assigned(tobjectdef(obj).childof) then
                  exit;
                sym:=tsym(tobjectdef(obj).childof.symtable.find('CREATE'));
                if assigned(sym) and
                   (sym.typ=procsym) then
                  pd:=tprocsym(sym).find_bytype_parameterless(potype_constructor);
                if not assigned(pd) then
                  begin
                    Message(sym_e_no_matching_inherited_parameterless_constructor);
                    exit
                  end;
              end;
            { we call all constructors CREATE, because they don't have a name in
              Java and otherwise we can't determine whether multiple overloads
              are created with the same parameters }
            sym:=tsym(obj.symtable.find('CREATE'));
            if assigned(sym) then
              begin
                { does another, non-procsym, symbol already exist with that name? }
                if (sym.typ<>procsym) then
                  begin
                    Message1(sym_e_duplicate_id_create_java_constructor,sym.realname);
                    exit;
                  end;
                ps:=tprocsym(sym);
                { is there already a parameterless function/procedure create? }
                pd:=ps.find_bytype_parameterless(potype_function);
                if not assigned(pd) then
                  pd:=ps.find_bytype_parameterless(potype_procedure);
                if assigned(pd) then
                  begin
                    Message1(sym_e_duplicate_id_create_java_constructor,pd.fullprocname(false));
                    exit;
                  end;
              end;
            if not assigned(sym) then
              begin
                ps:=cprocsym.create('Create');
                obj.symtable.insert(ps);
              end;
            { determine symtable level }
            topowner:=obj;
            while not(topowner.owner.symtabletype in [staticsymtable,globalsymtable]) do
              topowner:=topowner.owner.defowner;
            { create procdef }
            pd:=cprocdef.create(topowner.owner.symtablelevel+1,true);
            if df_generic in obj.defoptions then
              include(pd.defoptions,df_generic);
            { method of this objectdef }
            pd.struct:=obj;
            { associated procsym }
            pd.procsym:=ps;
            { constructor }
            pd.proctypeoption:=potype_constructor;
            { needs to be exported }
            include(pd.procoptions,po_global);
            { by default do not include this routine when looking for overloads }
            include(pd.procoptions,po_ignore_for_overload_resolution);
            { generate anonymous inherited call in the implementation }
            pd.synthetickind:=tsk_anon_inherited;
            { public }
            pd.visibility:=vis_public;
            { result type }
            pd.returndef:=obj;
            { calling convention }
            if assigned(current_structdef) or
               (assigned(pd.owner.defowner) and
                (pd.owner.defowner.typ=recorddef)) then
              handle_calling_convention(pd,hcc_default_actions_intf_struct)
            else
              handle_calling_convention(pd,hcc_default_actions_intf);
            { register forward declaration with procsym }
            proc_add_definition(pd);
          end;

        { also add class constructor if class fields that need wrapping, and
          if none was defined }
        if obj.find_procdef_bytype(potype_class_constructor)=nil then
          begin
            needclassconstructor:=false;
            for i:=0 to obj.symtable.symlist.count-1 do
              begin
                if (tsym(obj.symtable.symlist[i]).typ=staticvarsym) and
                   jvmimplicitpointertype(tstaticvarsym(obj.symtable.symlist[i]).vardef) then
                  begin
                    needclassconstructor:=true;
                    break;
                  end;
              end;
            if needclassconstructor then
              begin
                replace_scanner('custom_class_constructor',sstate);
                if str_parse_method_dec('constructor fpc_jvm_class_constructor;',potype_class_constructor,true,obj,pd) then
                  pd.synthetickind:=tsk_empty
                else
                  internalerror(2011040501);
                restore_scanner(sstate);
              end;
          end;
      end;




end.
