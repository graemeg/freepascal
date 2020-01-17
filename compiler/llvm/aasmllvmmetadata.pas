{
    Copyright (c) 2019 by Jonas Maebe,
    member of the Free Pascal Compiler development team

    Support for LLVM metadata

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
unit aasmllvmmetadata;

{$i fpcdefs.inc}

interface

  uses
    globtype,cclasses,
    cgbase,
    aasmtai,aasmcnst,
    symtype;

  type
    tspecialisedmetadatanodekind = (
      smeta_DIFile,
      smeta_DIBasicType,
      smeta_DISubroutineType,
      smeta_DIDerivedType,
      smeta_DICompositeType,
      smeta_DISubrange,
      smeta_DIEnumerator,
      smeta_DITemplateTypeParameter,
      smeta_DITemplateValueParameter,
      smeta_DINamespace,
      smeta_DIGlobalVariable,
      smeta_DISubprogram,
      smeta_DILexicalBlock,
      smeta_DILexicalBlockFile,
      smeta_DILocation,
      smeta_DILocalVariable,
      smeta_DIExpression,
      smeta_DIObjCProperty,
      smeta_DIImportedEntity,
      smeta_DIMacro,
      smeta_DIMacroFile
    );

//   represented by a tai_simpletypedconst() with inside a metadata struct,
//   or as a metadata register (for parameters)
//    tai_llvmmetadatastring = class

    tai_llvmbasemetadatanode = class abstract(tai_aggregatetypedconst)
     strict protected
      function getname: ansistring; virtual; abstract;
     public
      procedure addvalue(val: tai_abstracttypedconst); override;
      property name: ansistring read getname;
      constructor create; reintroduce;
    end;

    (* !0 = !{ type1 value1, ... } *)
    tai_llvmunnamedmetadatanode = class(tai_llvmbasemetadatanode)
     strict private class var
      snextid: TSuperRegister;
      class function getnextid: TSuperRegister;
     strict protected
      fnameval: TSuperRegister;
     public
      constructor create; reintroduce;
      function getname: ansistring; override;
    end;

    (* !name = !{ type1 value1, ... } *)
    tai_llvmnamedmetadatanode = class(tai_llvmbasemetadatanode)
     strict protected
      fname: ansistring;
      function getname: ansistring; override;
     public
      constructor create(const aName: ansistring);
    end;

    tai_llvmmetadatareftypedconst = class(tai_simple)
     strict private
      fval: tai_llvmbasemetadatanode;
     public
      constructor create(_val: tai_llvmbasemetadatanode);
      property val: tai_llvmbasemetadatanode read fval;
    end;

    { @g1 = global i32 0, *!id !value.name* }
    tai_llvmmetadatareferenceoperand = class(tai_simple)
     strict private
      fid: ansistring;
      fvalue: tai_llvmbasemetadatanode;
     public
      constructor create(const anID: ansistring; aValue: tai_llvmbasemetadatanode);
      property id: ansistring read fid;
      property value: tai_llvmbasemetadatanode read fvalue;
    end;

    { !name = !kindname(field1: value1, ...) }
    tai_llvmspecialisedmetadatanode = class(tai_llvmunnamedmetadatanode)
      { identifies name and fieldnames }
      kind: tspecialisedmetadatanodekind;
    end;

    tllvmmetadata = class
     strict private
       class function addstring(const s: TSymstr): TSuperRegister;
       class function regtostring(reg: TRegister): TSymStr;
     public

      class function getstringreg(const s: TSymstr): TRegister;
      class function getpcharreg(p: pchar; len: longint): TRegister;
      class function getregstring(reg: TRegister): TSymStr;
    end;

    function llvm_getmetadatareftypedconst(metadata: tai_llvmbasemetadatanode): tai_simpletypedconst;

    function llvm_constrainedexceptmodestring: ansistring;

implementation

  uses
    verbose,globals,
    fmodule,
    symdef;

  function llvm_getmetadatareftypedconst(metadata: tai_llvmbasemetadatanode): tai_simpletypedconst;
    begin
      result:=tai_simpletypedconst.create(llvm_metadatatype, tai_llvmmetadatareftypedconst.create(metadata));
    end;

  function llvm_constrainedexceptmodestring: ansistring;
    begin
      if not(cs_opt_fastmath in current_settings.optimizerswitches) then
        result:='fpexcept.maytrap'
      else
        result:='fpexcept.ignore'
    end;

  procedure tai_llvmbasemetadatanode.addvalue(val: tai_abstracttypedconst);
    begin
      { bypass string merging attempts, as we add tai_strings directly here }
      fvalues.add(val);
    end;

  constructor tai_llvmbasemetadatanode.create;
    begin
      inherited create(tck_array, llvm_metadatatype);
      typ:=ait_llvmmetadatanode;
    end;


  class function tai_llvmunnamedmetadatanode.getnextid: TSuperRegister;
    begin
      result:=snextid;
      inc(snextid);
    end;


  function tai_llvmunnamedmetadatanode.getname: ansistring;
    begin
      str(fnameval,result);
    end;


  constructor tai_llvmunnamedmetadatanode.create;
    begin
      inherited;
      fnameval:=getnextid;
    end;


  function tai_llvmnamedmetadatanode.getname: ansistring;
    begin
      result:=fname;
    end;


  constructor tai_llvmnamedmetadatanode.create(const aName: ansistring);
    begin
      inherited create;
      fname:=aName;
    end;


  constructor tai_llvmmetadatareftypedconst.create(_val: tai_llvmbasemetadatanode);
    begin
      inherited create(ait_llvmmetadatareftypedconst);
      fval:=_val;
    end;


  constructor tai_llvmmetadatareferenceoperand.create(const anID: ansistring; aValue: tai_llvmbasemetadatanode);
    begin
      inherited create(ait_llvmmetadatarefoperand);
      fid:=anID;
      fvalue:=aValue;
    end;


/////////////////////////////////////////////////

  class function tllvmmetadata.addstring(const s: TSymStr): TSuperRegister;
    var
      index: longint;
    begin
      index:=current_module.llvmmetadatastrings.Add(s,nil);
      if index>high(result) then
        internalerror(2019122806);
      result:=index;
    end;


  class function tllvmmetadata.regtostring(reg: TRegister): TSymStr;
    begin
      if getregtype(reg)<>R_METADATAREGISTER then
        internalerror(2019122807);
      if getsubreg(reg)<>R_SUBMETASTRING then
        internalerror(2019122808);
      result:=current_module.llvmmetadatastrings.NameOfIndex(getsupreg(reg));
    end;


  class function tllvmmetadata.getstringreg(const s: TSymstr): TRegister;
    var
      supreg: TSuperRegister;
      index: longint;
    begin
      index:=current_module.llvmmetadatastrings.FindIndexOf(s);
      if index<>-1 then
        supreg:=index
      else
        supreg:=addstring(s);
      result:=newreg(R_METADATAREGISTER,supreg,R_SUBMETASTRING);
    end;


  class function tllvmmetadata.getpcharreg(p: pchar; len: longint): TRegister;
    var
      str: TSymStr;
    begin
      if len>0 then
        begin
          setlength(str,len);
          move(p[0],str[1],len);
          result:=getstringreg(str);
        end
      else
        result:=getstringreg('');
    end;


  class function tllvmmetadata.getregstring(reg: TRegister): TSymStr;
    begin
      result:=regtostring(reg);
    end;


end.

