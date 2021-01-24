{
    Copyright (c) 2014 by Florian Klaempfl

    Symbol table overrides for i8086

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
unit symcpu;

{$i fpcdefs.inc}

interface

uses
  globtype,
  symconst,symtype,symdef,symsym,symx86,symi86;

type
  { defs }
  tcpufiledef = class(tfiledef)
  end;
  tcpufiledefclass = class of tcpufiledef;

  tcpuvariantdef = class(tvariantdef)
  end;
  tcpuvariantdefclass = class of tcpuvariantdef;

  tcpuformaldef = class(tformaldef)
  end;
  tcpuformaldefclass = class of tcpuformaldef;

  tcpuforwarddef = class(tforwarddef)
  end;
  tcpuforwarddefclass = class of tcpuforwarddef;

  tcpuundefineddef = class(tundefineddef)
  end;
  tcpuundefineddefclass = class of tcpuundefineddef;

  tcpuerrordef = class(terrordef)
  end;
  tcpuerrordefclass = class of tcpuerrordef;

  tcpupointerdef = class(tx86pointerdef)
    class function default_x86_data_pointer_type: tx86pointertyp; override;
    function alignment:shortint;override;
    function pointer_arithmetic_int_type:tdef; override;
    function pointer_arithmetic_uint_type:tdef; override;
    function pointer_subtraction_result_type:tdef; override;
    function converted_pointer_to_array_range_type: tdef; override;
  end;
  tcpupointerdefclass = class of tcpupointerdef;

  tcpurecorddef = class(trecorddef)
  end;
  tcpurecorddefclass = class of tcpurecorddef;

  tcpuimplementedinterface = class(timplementedinterface)
  end;
  tcpuimplementedinterfaceclass = class of tcpuimplementedinterface;

  tcpuobjectdef = class(tobjectdef)
  end;
  tcpuobjectdefclass = class of tcpuobjectdef;

  tcpuclassrefdef = class(tclassrefdef)
    function alignment:shortint;override;
  end;
  tcpuclassrefdefclass = class of tcpuclassrefdef;

  { tcpuarraydef }

  tcpuarraydef = class(tarraydef)
   private
    huge: Boolean;
   protected
    procedure ppuload_platform(ppufile: tcompilerppufile); override;
    procedure ppuwrite_platform(ppufile: tcompilerppufile); override;
   public
    constructor create_from_pointer(def:tpointerdef);override;
    function getcopy: tstoreddef; override;
    function GetTypeName:string;override;
    property is_huge: Boolean read huge write huge;
  end;
  tcpuarraydefclass = class of tcpuarraydef;

  tcpuorddef = class(torddef)
  end;
  tcpuorddefclass = class of tcpuorddef;

  tcpufloatdef = class(tfloatdef)
  end;
  tcpufloatdefclass = class of tcpufloatdef;

  { tcpuprocvardef }

  tcpuprocvardef = class(ti86procvardef)
    constructor create(level:byte);override;
    function getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp;const paraprefix:string):tstoreddef;override;
    function address_type:tdef;override;
    function ofs_address_type:tdef;override;
    function size:asizeint;override;
    procedure declared_far;override;
    procedure declared_near;override;
    function is_far:boolean;
  end;
  tcpuprocvardefclass = class of tcpuprocvardef;

  { tcpuprocdef }

  tcpuprocdef = class(ti86procdef)
   private
    { returns whether the function is far by default, i.e. whether it would be
      far if _all_ of the following conditions are true:
      - we're in a far code memory model
      - it has no 'near' or 'far' specifiers
      - it is compiled in a $F- state }
    function default_far:boolean;
   protected
    procedure Setinterfacedef(AValue: boolean);override;
   public
    constructor create(level:byte;doregister:boolean);override;
    function getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp;const paraprefix:string):tstoreddef;override;
    function address_type:tdef;override;
    function ofs_address_type:tdef;override;
    function size:asizeint;override;
    procedure declared_far;override;
    procedure declared_near;override;
    function is_far:boolean;
  end;
  tcpuprocdefclass = class of tcpuprocdef;

  tcpustringdef = class(tstringdef)
  end;
  tcpustringdefclass = class of tcpustringdef;

  tcpuenumdef = class(tenumdef)
  end;
  tcpuenumdefclass = class of tcpuenumdef;

  tcpusetdef = class(tsetdef)
  end;
  tcpusetdefclass = class of tcpusetdef;

  { syms }
  tcpulabelsym = class(tlabelsym)
  end;
  tcpulabelsymclass = class of tcpulabelsym;

  tcpuunitsym = class(tunitsym)
  end;
  tcpuunitsymclass = class of tcpuunitsym;

  tcpuprogramparasym = class(tprogramparasym)
  end;
  tcpuprogramparasymclass = class(tprogramparasym);

  tcpunamespacesym = class(tnamespacesym)
  end;
  tcpunamespacesymclass = class of tcpunamespacesym;

  tcpuprocsym = class(tprocsym)
  end;
  tcpuprocsymclass = class of tcpuprocsym;

  tcputypesym = class(ttypesym)
  end;
  tcpuypesymclass = class of tcputypesym;

  tcpufieldvarsym = class(tfieldvarsym)
  end;
  tcpufieldvarsymclass = class of tcpufieldvarsym;

  tcpulocalvarsym = class(tlocalvarsym)
  end;
  tcpulocalvarsymclass = class of tcpulocalvarsym;

  tcpuparavarsym = class(tparavarsym)
  end;
  tcpuparavarsymclass = class of tcpuparavarsym;

  tcpustaticvarsym = class(tstaticvarsym)
  end;
  tcpustaticvarsymclass = class of tcpustaticvarsym;

  tcpuabsolutevarsym = class(ti86absolutevarsym)
   protected
    procedure ppuload_platform(ppufile: tcompilerppufile); override;
    procedure ppuwrite_platform(ppufile: tcompilerppufile); override;
   public
    addrsegment : aword;
  end;
  tcpuabsolutevarsymclass = class of tcpuabsolutevarsym;

  tcpupropertysym = class(tpropertysym)
  end;
  tcpupropertysymclass = class of tcpupropertysym;

  tcpuconstsym = class(tconstsym)
  end;
  tcpuconstsymclass = class of tcpuconstsym;

  tcpuenumsym = class(tenumsym)
  end;
  tcpuenumsymclass = class of tcpuenumsym;

  tcpusyssym = class(tsyssym)
  end;
  tcpusyssymclass = class of tcpusyssym;


const
   pbestrealtype : ^tdef = @s80floattype;


  function is_proc_far(p: tabstractprocdef): boolean;

  {# Returns true if p is a far proc var }
  function is_farprocvar(p : tdef): boolean;

  {# Returns true if p is a far pointer def }
  function is_farpointer(p : tdef) : boolean;

  {# Returns true if p is a huge pointer def }
  function is_hugepointer(p : tdef) : boolean;

implementation

  uses
    globals, cpuinfo, verbose, fmodule;


  function is_proc_far(p: tabstractprocdef): boolean;
  begin
    if p is tcpuprocdef then
      result:=tcpuprocdef(p).is_far
    else if p is tcpuprocvardef then
      result:=tcpuprocvardef(p).is_far
    else
      internalerror(2014041303);
  end;

  { true if p is a far proc var }
  function is_farprocvar(p : tdef): boolean;
    begin
      result:=(p.typ=procvardef) and tcpuprocvardef(p).is_far;
    end;

  { true if p is a far pointer def }
  function is_farpointer(p : tdef) : boolean;
    begin
      result:=(p.typ=pointerdef) and (tcpupointerdef(p).x86pointertyp=x86pt_far);
    end;

  { true if p is a huge pointer def }
  function is_hugepointer(p : tdef) : boolean;
    begin
      result:=(p.typ=pointerdef) and (tcpupointerdef(p).x86pointertyp=x86pt_huge);
    end;

  procedure handle_procdef_copyas(src: tabstractprocdef; is_far: boolean; copytyp:tproccopytyp; var result: tabstractprocdef);
    begin
      if is_far then
        include(result.procoptions,po_far)
      else
        exclude(result.procoptions,po_far);
      case copytyp of
        pc_far_address:
          begin
            include(result.procoptions,po_addressonly);
            include(result.procoptions,po_far);
          end;
        pc_offset:
          begin
            include(result.procoptions,po_addressonly);
            exclude(result.procoptions,po_far);
          end;
        else
          ; {none}
      end;
    end;

{****************************************************************************
                               tcpuclassrefdef
****************************************************************************}

  function tcpuclassrefdef.alignment:shortint;
    begin
      Result:=2;
    end;

{****************************************************************************
                               tcpuarraydef
****************************************************************************}

  constructor tcpuarraydef.create_from_pointer(def: tpointerdef);
    begin
      huge:=tcpupointerdef(def).x86pointertyp=x86pt_huge;
      inherited create_from_pointer(def);
    end;


  function tcpuarraydef.getcopy: tstoreddef;
    begin
      result:=inherited;
      tcpuarraydef(result).huge:=huge;
    end;


  function tcpuarraydef.GetTypeName: string;
    begin
      Result:=inherited;
      if is_huge then
        Result:='Huge '+Result;
    end;


  procedure tcpuarraydef.ppuload_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      huge:=(ppufile.getbyte<>0);
    end;


  procedure tcpuarraydef.ppuwrite_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      ppufile.putbyte(byte(huge));
    end;


{****************************************************************************
                             tcpuprocdef
****************************************************************************}

  constructor tcpuprocdef.create(level: byte;doregister:boolean);
    begin
      inherited create(level,doregister);
      if (current_settings.x86memorymodel in x86_far_code_models) and
         ((cs_huge_code in current_settings.moduleswitches) or
          (cs_force_far_calls in current_settings.localswitches)) then
        procoptions:=procoptions+[po_far];
    end;


  function tcpuprocdef.getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp;const paraprefix:string):tstoreddef;
    begin
      result:=inherited;
      handle_procdef_copyas(self,is_far,copytyp,tabstractprocdef(result));
    end;


  function tcpuprocdef.address_type: tdef;
    begin
      if is_far then
        result:=voidfarpointertype
      else
        result:=voidnearpointertype;
    end;


  function tcpuprocdef.ofs_address_type:tdef;
    begin
      result:=voidnearpointertype;
    end;


  function tcpuprocdef.size: asizeint;
    begin
      result:=address_type.size;
    end;


  procedure tcpuprocdef.declared_far;
    begin
      include(procoptions,po_far);
      include(procoptions,po_hasnearfarcallmodel);
    end;


  procedure tcpuprocdef.declared_near;
    begin
      if not (cs_huge_code in current_settings.moduleswitches) then
        begin
          exclude(procoptions,po_far);
          include(procoptions,po_hasnearfarcallmodel);
        end
      else
        inherited declared_near;
    end;


  function tcpuprocdef.default_far: boolean;
    begin
      if proctypeoption in [potype_proginit,potype_unitinit,potype_unitfinalize,
                            potype_constructor,potype_destructor,
                            potype_class_constructor,potype_class_destructor,
                            potype_propgetter,potype_propsetter] then
        exit(true);
      if (procoptions*[po_classmethod,po_virtualmethod,po_abstractmethod,
                       po_finalmethod,po_staticmethod,po_overridingmethod,
                       po_external,po_public,po_interrupt])<>[] then
        exit(true);
      if is_methodpointer then
        exit(true);
      result:=not (visibility in [vis_private,vis_hidden]);
    end;


  procedure tcpuprocdef.Setinterfacedef(AValue: boolean);
    begin
      inherited;
      if (current_settings.x86memorymodel in x86_far_code_models) and AValue then
        include(procoptions,po_far);
    end;


  function tcpuprocdef.is_far: boolean;
    begin
      result:=(po_exports in procoptions) or
              (po_far in procoptions) or
              ((current_settings.x86memorymodel in x86_far_code_models) and default_far);
    end;

{****************************************************************************
                             tcpuprocvardef
****************************************************************************}

  constructor tcpuprocvardef.create(level: byte);
    begin
      inherited create(level);
      if current_settings.x86memorymodel in x86_far_code_models then
        procoptions:=procoptions+[po_far];
    end;


  function tcpuprocvardef.getcopyas(newtyp:tdeftyp;copytyp:tproccopytyp;const paraprefix:string):tstoreddef;
    begin
      result:=inherited;
      handle_procdef_copyas(self,is_far,copytyp,tabstractprocdef(result));
    end;


  function tcpuprocvardef.address_type:tdef;
    begin
      if is_addressonly then
        if is_far then
          result:=voidfarpointertype
        else
          begin
            { near }
            if current_settings.x86memorymodel=mm_tiny then
              result:=voidnearpointertype
            else
              result:=voidnearcspointertype;
          end
      else
        result:=inherited;
    end;


  function tcpuprocvardef.ofs_address_type:tdef;
    begin
      result:=voidnearpointertype;
    end;


  function tcpuprocvardef.size:asizeint;
    begin
      if is_addressonly then
        if is_far then
          result:=4
        else
          result:=2
      else
        result:=inherited;
    end;


  procedure tcpuprocvardef.declared_far;
    begin
      if is_addressonly then
        begin
          include(procoptions,po_far);
          include(procoptions,po_hasnearfarcallmodel);
        end
      else
        inherited;
    end;


  procedure tcpuprocvardef.declared_near;
    begin
      if is_addressonly then
        begin
          exclude(procoptions,po_far);
          include(procoptions,po_hasnearfarcallmodel);
        end
      else
        inherited;
    end;


  function tcpuprocvardef.is_far: boolean;
    begin
      if is_addressonly then
        result:=po_far in procoptions
      else
        result:=current_settings.x86memorymodel in x86_far_code_models;
    end;

{****************************************************************************
                             tcpupointerdef
****************************************************************************}

    class function tcpupointerdef.default_x86_data_pointer_type: tx86pointertyp;
      begin
        if current_settings.x86memorymodel in x86_far_data_models then
          result:=x86pt_far
        else
          result:=inherited;
      end;


    function tcpupointerdef.alignment:shortint;
      begin
        { on i8086, we use 16-bit alignment for all pointer types, even far and
          huge (which are 4 bytes long) }
        result:=2;
      end;


    function tcpupointerdef.pointer_arithmetic_int_type:tdef;
      begin
        case x86pointertyp of
          x86pt_huge:
            result:=s32inttype;
          x86pt_far,
          x86pt_near,
          x86pt_near_cs,
          x86pt_near_ds,
          x86pt_near_ss,
          x86pt_near_es,
          x86pt_near_fs,
          x86pt_near_gs:
            result:=s16inttype;
        end;
      end;


    function tcpupointerdef.pointer_arithmetic_uint_type:tdef;
      begin
        case x86pointertyp of
          x86pt_huge:
            result:=u32inttype;
          x86pt_far,
          x86pt_near,
          x86pt_near_cs,
          x86pt_near_ds,
          x86pt_near_ss,
          x86pt_near_es,
          x86pt_near_fs,
          x86pt_near_gs:
            result:=u16inttype;
        end;
      end;


    function tcpupointerdef.pointer_subtraction_result_type:tdef;
      begin
        case x86pointertyp of
          x86pt_huge:
            result:=s32inttype;
          x86pt_far:
            result:=u16inttype;
          x86pt_near,
          x86pt_near_cs,
          x86pt_near_ds,
          x86pt_near_ss,
          x86pt_near_es,
          x86pt_near_fs,
          x86pt_near_gs:
            result:=s16inttype;
        end;
      end;


    function tcpupointerdef.converted_pointer_to_array_range_type: tdef;
      begin
        case x86pointertyp of
          x86pt_huge:
            result:=s32inttype;
          x86pt_far,
          x86pt_near,
          x86pt_near_cs,
          x86pt_near_ds,
          x86pt_near_ss,
          x86pt_near_es,
          x86pt_near_fs,
          x86pt_near_gs:
            result:=s16inttype;
        end;
      end;


{****************************************************************************
                             tcpuabsolutevarsym
****************************************************************************}

  procedure tcpuabsolutevarsym.ppuload_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      if absseg then
        addrsegment:=ppufile.getaword;
    end;


  procedure tcpuabsolutevarsym.ppuwrite_platform(ppufile: tcompilerppufile);
    begin
      inherited;
      if absseg then
        ppufile.putaword(addrsegment);
    end;

begin
  { used tdef classes }
  cfiledef:=tcpufiledef;
  cvariantdef:=tcpuvariantdef;
  cformaldef:=tcpuformaldef;
  cforwarddef:=tcpuforwarddef;
  cundefineddef:=tcpuundefineddef;
  cerrordef:=tcpuerrordef;
  cpointerdef:=tcpupointerdef;
  crecorddef:=tcpurecorddef;
  cimplementedinterface:=tcpuimplementedinterface;
  cobjectdef:=tcpuobjectdef;
  cclassrefdef:=tcpuclassrefdef;
  carraydef:=tcpuarraydef;
  corddef:=tcpuorddef;
  cfloatdef:=tcpufloatdef;
  cprocvardef:=tcpuprocvardef;
  cprocdef:=tcpuprocdef;
  cstringdef:=tcpustringdef;
  cenumdef:=tcpuenumdef;
  csetdef:=tcpusetdef;

  { used tsym classes }
  clabelsym:=tcpulabelsym;
  cunitsym:=tcpuunitsym;
  cprogramparasym:=tcpuprogramparasym;
  cnamespacesym:=tcpunamespacesym;
  cprocsym:=tcpuprocsym;
  ctypesym:=tcputypesym;
  cfieldvarsym:=tcpufieldvarsym;
  clocalvarsym:=tcpulocalvarsym;
  cparavarsym:=tcpuparavarsym;
  cstaticvarsym:=tcpustaticvarsym;
  cabsolutevarsym:=tcpuabsolutevarsym;
  cpropertysym:=tcpupropertysym;
  cconstsym:=tcpuconstsym;
  cenumsym:=tcpuenumsym;
  csyssym:=tcpusyssym;
end.

