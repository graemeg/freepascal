{
    Copyright (c) 1998-2006 by Peter Vreman
    Copyright (c) 2011 by Armin Diehl

    Contains the binary netware nlm executable writer

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
unit ognlm;

{$i fpcdefs.inc}

interface

    uses
       { common }
       cclasses,globtype,
       { target }
       systems,
       aasmbase,assemble,link,
       { output }
       ogbase,
       owbase,
       ogcoff;

{*****************************************************************************
                    NLM File structures and constants
*****************************************************************************}

{

LString0 -> 1 byte Length, Text, #0
LString  -> 1 byte length, Text

Basic NLM File Structure:

FixedHeader
  nlm32_i386_external_fixed_header     130 bytes

VarHdr1
  NLM Description: LString0              2+n bytes
  Stacksize                              4 bytes
  reserved = 0                           4 bytes
  ' LONG'                                5 bytes
  NLM screen name: LString0              2+n bytes
  NLM thread name: LString0              2+n bytes

Optional Headers beginning with stamp (without '')
'VeRsIoN#':   nlm32_i386_external_version_header     32 bytes
'CoPyRiGhT=': LString0                              2+n bytes
'MeSsAgEs':   nlm32_i386_external_extended_header   124 bytes
'CuStHeAd':   nlm32_i386_external_custom_header
'CyGnUsEx':   nlm32_i386_external_cygnus_ext_header 16 bytes

.text
.data
.relocs=
  addr(32),addr(32),...
  addr and $80000000 > 0 -> FixupToSeg=.text else .data
  addr and $40000000 > 0 -> FixupInSeg=.text else .data
.importedSymbols
  name   LString                                   1+n bytes
  number of references r                             4 bytes
  addresses                                        r*4 bytes
.exportedSymbols
  name   LString                                   1+n bytes
  addr                                               4 bytes
    addr and $80000000 > 0 -> .text else .data
  ...
.modules

.nlmdebugrecs
  type (0=.data,1=.code,2,..=????)                   1 byte
  addr                                               4 bytes
  name LString                                     1+n bytes
  ...

}


const NLM_MAX_DESCRIPTION_LENGTH = 127;
      NLM_MAX_SCREEN_NAME_LENGTH = 71;
      NLM_MAX_THREAD_NAME_LENGTH = 71;   // some netware docs limit this to 12 ?
      NLM_OLD_THREAD_NAME_LENGTH = 5;
      NLM_HEADER_VERSION         = 4;
      NLM_DEFAULT_STACKSIZE      = (32 * 1024);
      NLM_VERSION_STAMP          = 'VeRsIoN#';
      NLM_COPYRIGHT_STAMP        = 'CoPyRiGhT=';
      NLM_CYGNUS_STAMP           = 'CyGnUsEx';
      NLM_MESSAGES_STAMP         = 'MeSsAgEs';
      NLM_CUSTOM_STAMP           = 'CuStHeAd';
      NLM_SIGNATURE              = 'NetWare Loadable Module'#$1A;
      NLM_FLAGS_REENTRANT        = 1;
      NLM_FLAGS_MULTILOAD        = 2;
      NLM_FLAGS_SYNCHRONIZE      = 4;
      NLM_FLAGS_PSEUDOPREEMPTION = 8;
      NLM_FLAGS_OSDOMAIN         = $10;
      NLM_FLAGS_AUTOUNLOAD       = $40;


  type
    uint32 = longword;

    nlm32_i386_external_fixed_header = packed record
        signature                  : array[0..23] of char;
        version                    : uint32;
        (* The name of the module, which must be a DOS name (1-8 characters followed
           by a period and a 1-3 character extension).  The first byte is the byte
           length of the name and the last byte is a null terminator byte.  This
           field is fixed length, and any unused bytes should be null bytes.  The
           value is set by the OUTPUT keyword to NLMLINK. *)
        moduleName                 : string[13]; //array[0..13] of byte;
        codeImageOffset            : uint32;     // The byte offset of the code image from the start of the file.
        codeImageSize              : uint32;     // The size of the code image, in bytes.
        dataImageOffset            : uint32;     // The byte offset of the data image from the start of the file.
        dataImageSize              : uint32;     // The size of the data image, in bytes.
        uninitializedDataSize      : uint32;     // The size of the uninitialized data region that the loader has to be
                                                 // allocated at load time.  Uninitialized data follows the initialized
                                                 // data in the NLM address space.
        customDataOffset           : uint32;     // The byte offset of the custom data from the start of the file.  The
                                                 // custom data is set by the CUSTOM keyword to NLMLINK.  It is possible
                                                 // for this to be EOF if there is no custom data.
        customDataSize             : uint32;     // The size of the custom data, in bytes.
        moduleDependencyOffset     : uint32;     // The byte offset of the module dependencies from the start of the file.
                                                 // The module dependencies are determined by the MODULE keyword in NLMLINK.
        numberOfModuleDependencies : uint32;     // he number of module dependencies at the moduleDependencyOffset.
        relocationFixupOffset      : uint32;     // The byte offset of the relocation fixup data from the start of the file
        numberOfRelocationFixups   : uint32;
        externalReferencesOffset   : uint32;
        numberOfExternalReferences : uint32;
        publicsOffset              : uint32;
        numberOfPublics            : uint32;
        debugInfoOffset            : uint32;     // The byte offset of the internal debug info from the start of the file.
                                                 // It is possible for this to be EOF if there is no debug info.
        numberOfDebugRecords       : uint32;
        codeStartOffset            : uint32;
        exitProcedureOffset        : uint32;
        checkUnloadProcedureOffset : uint32;
        moduleType                 : uint32;
        flags                      : uint32;
      end;


  { The version header is one of the optional auxiliary headers and
     follows the fixed length and variable length NLM headers.  }
  { The header is recognized by "VeRsIoN#" in the stamp field.  }

     nlm32_i386_external_version_header = packed record
          stamp           : array[0..7] of char;  // VeRsIoN#
          majorVersion,
          minorVersion,
          revision,
          year,
          month,
          day             : uint32;
       end;
  { The header is recognized by "MeSsAgEs" in the stamp field.  }

     nlm32_i386_external_extended_header = packed record
          stamp                        : array[0..7] of char;  // MeSsAgEs
          languageID                   : uint32;
          messageFileOffset            : uint32;
          messageFileLength            : uint32;
          messageCount                 : uint32;
          helpFileOffset               : uint32;
          helpFileLength               : uint32;
          RPCDataOffset                : uint32;
          RPCDataLength                : uint32;
          sharedCodeOffset             : uint32;
          sharedCodeLength             : uint32;
          sharedDataOffset             : uint32;
          sharedDataLength             : uint32;
          sharedRelocationFixupOffset  : uint32;
          sharedRelocationFixupCount   : uint32;
          sharedExternalReferenceOffset: uint32;
          sharedExternalReferenceCount : uint32;
          sharedPublicsOffset          : uint32;
          sharedPublicsCount           : uint32;
          sharedDebugRecordOffset      : uint32;
          sharedDebugRecordCount       : uint32;
          SharedInitializationOffset   : uint32;
          SharedExitProcedureOffset    : uint32;
          productID : longint;
          reserved0 : longint;
          reserved1 : longint;
          reserved2 : longint;
          reserved3 : longint;
          reserved4 : longint;
          reserved5 : longint;
       end;

     nlm32_i386_external_custom_header = packed record
          stamp       : array[0..7] of char;  // CuStHeAd
          hdrLength   : uint32;
          dataOffset  : uint32;
          dataLength  : uint32;
          //dataStamp   : array[0..7] of char;
          //hdr         : uint32;
       end;
  { The internal Cygnus header is written out externally as a custom
     header.  We don't try to replicate that structure here.   }
  { The header is recognized by "CyGnUsEx" in the stamp field.  }
  { File location of debugging information.   }
  { Length of debugging information.   }

     nlm32_i386_external_cygnus_ext_header = packed record
          stamp       : array[0..7] of char;  // CyGnUsEx
          offset      : uint32;
          length      : uint32;
       end;


//------------------


       TNLMExeSection = class(TExeSection)
       public
         constructor createnw(AList:TFPHashObjectList;const n:string);
       end;

       TsecType = (Section_text,Section_data,Section_other);

       TNLMexeoutput = class(texeoutput)
       private
         FRelocsGenerated,FImportsGenerated : boolean;
         FNumRelocs         : longword;
         FNumExternals      : longword;
         FNumModules        : longword;
         FNumDebugSymbols   : longword;
         fSizeWoDebugSyms   : longword;
         FnumExports        : longword;
         NlmSymbols         : TDynamicArray;
         ExeSecsListSize    : longint;
         nlmImpNames,                            // name of import. module name as import
         nlmImports         : TFPHashObjectList; // name of import, list of relocs as object
         headerAlignBytes   : longint;
         FexportFunctionOffsets:TFPList;    // offsets in .exports for function addresses, an offset of $80000000 is needed

         nlmHeader          : nlm32_i386_external_fixed_header;
         nlmVersionHeader   : nlm32_i386_external_version_header;
         nlmExtHeader       : nlm32_i386_external_extended_header;
         nlmCustHeader      : nlm32_i386_external_custom_header;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         //nlmHelpFileName    : TCmdStr;
         //nlmMessagesFileName: TCmdStr;
         //nlmXdcFileName     : TCmdStr;
=======
         nlmHelpFileName    : TCmdStr;
         nlmMessagesFileName: TCmdStr;
         nlmXdcFileName     : TCmdStr;
>>>>>>> graemeg/cpstrnew
=======
         nlmHelpFileName    : TCmdStr;
         nlmMessagesFileName: TCmdStr;
         nlmXdcFileName     : TCmdStr;
>>>>>>> graemeg/cpstrnew
=======
         nlmHelpFileName    : TCmdStr;
         nlmMessagesFileName: TCmdStr;
         nlmXdcFileName     : TCmdStr;
>>>>>>> graemeg/cpstrnew
=======
         nlmHelpFileName    : TCmdStr;
         nlmMessagesFileName: TCmdStr;
         nlmXdcFileName     : TCmdStr;
>>>>>>> origin/cpstrnew
         nlmCopyright       : string;
         nlmThreadname      : string;
         nlmScreenname      : string;
         nlmDescription     : string;

         function  totalheadersize:longword;
         procedure createNlm_symbol(const name:shortstring;value:longword;secType:TSecType);
         procedure globalsyms_create_symbol(p:TObject;arg:pointer);
         procedure ExeSectionList_write_header(p:TObject;arg:pointer);
         procedure ExeSectionList_calc_size(p:TObject;arg:pointer);
         procedure ExeSectionList_write_data(p:TObject;arg:pointer);
         procedure GenerateImports;
         procedure GenerateExports;
         procedure GenerateRelocs;
         procedure ExeSectionList_pass2_header(p:TObject;arg:pointer);
       protected
         function writedata:boolean;override;
       public
         constructor create; override;
         destructor destroy; override;
         procedure MemPos_Header;override;
         procedure DataPos_Header;override;
         procedure fillNlmVersionHeader;
         procedure GenerateLibraryImports(ImportLibraryList:TFPHashObjectList);override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         procedure MemPos_Start;override;
         procedure MemPos_ExeSection(const aname:string);override;
=======
         procedure Order_End;override;
         procedure MemPos_ExeSection(const aname:string);override;
         procedure DataPos_ExeSection(const aname:string);override;
>>>>>>> graemeg/cpstrnew
=======
         procedure Order_End;override;
         procedure MemPos_ExeSection(const aname:string);override;
         procedure DataPos_ExeSection(const aname:string);override;
>>>>>>> graemeg/cpstrnew
=======
         procedure Order_End;override;
         procedure MemPos_ExeSection(const aname:string);override;
         procedure DataPos_ExeSection(const aname:string);override;
>>>>>>> graemeg/cpstrnew
=======
         procedure Order_End;override;
         procedure MemPos_ExeSection(const aname:string);override;
         procedure DataPos_ExeSection(const aname:string);override;
>>>>>>> origin/cpstrnew
         procedure NLMwriteString (const s : string; terminateWithZero : boolean);
         procedure objNLMwriteString (const s : string; terminateWithZero : boolean);
         procedure ParseScript (linkscript:TCmdStrList); override;
       end;

    var
      {for symbols defined in linker script. To generate a fixup we
       need to know the segment (.text,.bss or .code) of the symbol
       Pointer in list is used as TsecType
       Filled by TInternalLinkerNetware.DefaultLinkScript }
      nlmSpecialSymbols_Segments : TFPHashList;

    type

      TNLMCoffObjInput = class(TCoffObjInput)
         constructor create;override;
       end;

       TNLMCoffassembler = class(tinternalassembler)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         constructor create(info: pasminfo; smart:boolean);override;
=======
         constructor create(smart:boolean);override;
>>>>>>> graemeg/cpstrnew
=======
         constructor create(smart:boolean);override;
>>>>>>> graemeg/cpstrnew
=======
         constructor create(smart:boolean);override;
>>>>>>> graemeg/cpstrnew
=======
         constructor create(smart:boolean);override;
>>>>>>> origin/cpstrnew
       end;

      TNLMCoffObjData = class(TCoffObjData)
         constructor create(const n:string);override;
       end;

      TNLMCoffObjOutput = class(TCoffObjOutput)
         constructor create(AWriter:TObjectWriter);override;
       end;

      TNLMCoffObjSection = class(TCoffObjSection)
         constructor create(AList:TFPHashObjectList;const Aname:string;Aalign:shortint;Aoptions:TObjSectionOptions);override;
       end;

implementation

    uses
{$ifdef win32}
       Windows,
{$endif win32}
       SysUtils,
       cutils,verbose,globals,
       fmodule,aasmdata,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       ogmap,export,owar
=======
       ogmap,export
>>>>>>> graemeg/cpstrnew
=======
       ogmap,export
>>>>>>> graemeg/cpstrnew
=======
       ogmap,export
>>>>>>> graemeg/cpstrnew
=======
       ogmap,export
>>>>>>> origin/cpstrnew
       ;


{****************************************************************************
                                 Helpers
****************************************************************************}
type
  TStringObj = class (TObject)
     fValue : string;
     constructor create (value:string);
     property value : string read fValue write fValue;
  end;

  constructor TStringObj.create(value:string);
  begin
    inherited create;
    fValue := value;
  end;



function SectionType (aName : string) : TSecType;
var s : string;
    seg: ptruint;
begin
  s := copy(aName,1,5);
  if s = '.text' then result := Section_text else
    if (s = '.data') or (copy(s,1,4)='.bss') then result := Section_data else
      if s[1] <> '.' then
        begin
          seg := ptruint(nlmSpecialSymbols_Segments.Find(aName));
          if seg <> 0 then
            result := TSecType(seg)
          else
            result := Section_other;
        end else
      result := Section_other;
end;

{****************************************************************************
                              TNLMexesection
****************************************************************************}


    constructor TNLMExeSection.createnw(AList:TFPHashObjectList;const n:string);
      begin
        inherited create(AList,n);
      end;


{****************************************************************************
                              TNLMexeoutput
****************************************************************************}

    constructor TNLMexeoutput.create;
      begin
        inherited create;
        CExeSection:=TNLMExeSection;
        CObjData:=TNLMCoffObjData;
        MaxMemPos:=$7FFFFFFF;
        SectionMemAlign:=$0;
        SectionDataAlign:=0;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
        RelocSection := true;  // always needed for NLM's
>>>>>>> graemeg/cpstrnew
=======
        RelocSection := true;  // always needed for NLM's
>>>>>>> graemeg/cpstrnew
=======
        RelocSection := true;  // always needed for NLM's
>>>>>>> graemeg/cpstrnew
=======
        RelocSection := true;  // always needed for NLM's
>>>>>>> origin/cpstrnew
        nlmImports := TFPHashObjectList.create(true);
        nlmImpNames := TFPHashObjectList.create(false);
        NlmSymbols := TDynamicArray.create(4096);
        FexportFunctionOffsets := TFPList.Create;
      end;

    destructor TNLMexeoutput.destroy;
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        nlmImports.Free;
        nlmImpNames.Free;
        nlmSymbols.Free;
        FexportFunctionOffsets.Free;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if assigned(nlmImports) then
          nlmImports.Free;
        if assigned(nlmImpNames) then
          nlmImpNames.Free;
        if assigned(nlmSymbols) then
          nlmSymbols.Free;
        if assigned(FexportFunctionOffsets) then
          FexportFunctionOffsets.Free;
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
        inherited destroy;
      end;

    procedure TNLMexeoutput.createNlm_symbol(const name:shortstring;value:longword;secType:TSecType);
      var
        b:byte;
      begin
        //Comment (V_Debug,'TNLMexeoutput.write_symbol '+name);
        {  type (0=.data,1=.code,2,..=????)                   1 byte
           addr                                               4 bytes
           name LString                                     1+n bytes }
        case secType of
          Section_Text : b := 1;
          Section_Data : b := 0
        else
          exit;
        end;
        nlmSymbols.write(b,sizeof(b));
        assert (sizeof(value)<>4);
        nlmSymbols.write(value,sizeof(value));
        nlmSymbols.write(name[0],length(name)+1);
        inc(FNumDebugSymbols);
      end;


    procedure TNLMexeoutput.globalsyms_create_symbol(p:TObject;arg:pointer);
      var
        value  : longword;
        exesec : TExeSection;
        i : integer;
        secType : TsecType;
      begin
        if not assigned(texesymbol(p).objsymbol) then
          internalerror(200603053);
        with texesymbol(p).objsymbol do
          begin
            exesec:=TExeSection(objsection.exesection);
            { There is no exesection defined for special internal symbols
              like __image_base__ }
            if assigned(exesec) then
              begin
                //secval:=exesec.secsymidx;
                value:=address-exesec.mempos;
              end
            else
              begin
                value:=address;
              end;
            { reloctype address to the section in the executable }
            secType := SectionType(objsection.Name);
            if (secType = Section_Text) or (secType = Section_Data) then
              begin
                i := nlmImports.FindIndexOf(texesymbol(p).name);
                if i < 0 then
                  createNlm_symbol(name,value,secType);
              end;
          end;
      end;



(*
function SecOpts(SecOptions:TObjSectionOptions):string;
    begin
      result := '[';
      if oso_Data in SecOptions then result := result + 'oso_Data ';
       { Is loaded into memory }
      if oso_load in SecOptions then result := result + 'oso_load ';
       { Not loaded into memory }
      if oso_noload in SecOptions then result := result + 'oso_noload ';
       { Read only }
      if oso_readonly in SecOptions then result := result + 'oso_readonly ';
       { Read/Write }
      if oso_write in SecOptions then result := result + 'oso_write ';
       { Contains executable instructions }
      if oso_executable in SecOptions then result := result + 'oso_executable ';
       { Never discard section }
      if oso_keep in SecOptions then result := result + 'oso_keep ';
       { Special common symbols }
      if oso_common in SecOptions then result := result + 'oso_common ';
       { Contains debug info and can be stripped }
      if oso_debug in SecOptions then result := result + 'oso_debug ';
       { Contains only strings }
      if oso_strings in SecOptions then result := result + 'oso_strings ';
      result := result + ']';
    end;
*)

    procedure TNLMexeoutput.ExeSectionList_calc_size(p:TObject;arg:pointer);
      var
        objsec : TObjSection;
        i    : longint;
      begin
        with texesection(p) do
          begin
            { don't write normal section if writing only debug info }
            if (ExeWriteMode=ewm_dbgonly) and
               not(oso_debug in SecOptions) then
              exit;

            if oso_data in secoptions then
              begin
                inc (fSizeWoDebugSyms,(Align(fSizeWoDebugSyms,SectionDataAlign)-fSizeWoDebugSyms));
                for i:=0 to ObjSectionList.Count-1 do
                  begin
                    objsec:=TObjSection(ObjSectionList[i]);
                    if oso_data in objsec.secoptions then
                      begin
                        inc(fSizeWoDebugSyms,objsec.size);
                        inc(fSizeWoDebugSyms,objsec.dataalignbytes);
                      end;
                  end;
              end;
          end;
      end;



    procedure TNLMexeoutput.ExeSectionList_write_Data(p:TObject;arg:pointer);
      var
        objsec : TObjSection;
        i,j    : longint;
        b      : byte;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        dpos,pad: aword;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      begin

        with texesection(p) do
          begin
            { don't write normal section if writing only debug info }
            if (ExeWriteMode=ewm_dbgonly) and
               not(oso_debug in SecOptions) then
              exit;

            if oso_data in secoptions then
              begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                if DataPos<FWriter.Size then
                  InternalError(2012103001);
                //if Align(FWriter.Size,SectionDataAlign)-FWriter.Size>0 then
                //  writeln (name,' align ',Align(FWriter.Size,SectionDataAlign)-FWriter.Size,' SectionDataAlign:',SectionDataAlign);
                FWriter.Writezeros(DataPos-FWriter.Size);
=======
                //if Align(FWriter.Size,SectionDataAlign)-FWriter.Size>0 then
                //  writeln (name,' align ',Align(FWriter.Size,SectionDataAlign)-FWriter.Size,' SectionDataAlign:',SectionDataAlign);
                FWriter.Writezeros(Align(FWriter.Size,SectionDataAlign)-FWriter.Size);
>>>>>>> graemeg/cpstrnew
=======
                //if Align(FWriter.Size,SectionDataAlign)-FWriter.Size>0 then
                //  writeln (name,' align ',Align(FWriter.Size,SectionDataAlign)-FWriter.Size,' SectionDataAlign:',SectionDataAlign);
                FWriter.Writezeros(Align(FWriter.Size,SectionDataAlign)-FWriter.Size);
>>>>>>> graemeg/cpstrnew
=======
                //if Align(FWriter.Size,SectionDataAlign)-FWriter.Size>0 then
                //  writeln (name,' align ',Align(FWriter.Size,SectionDataAlign)-FWriter.Size,' SectionDataAlign:',SectionDataAlign);
                FWriter.Writezeros(Align(FWriter.Size,SectionDataAlign)-FWriter.Size);
>>>>>>> graemeg/cpstrnew
=======
                //if Align(FWriter.Size,SectionDataAlign)-FWriter.Size>0 then
                //  writeln (name,' align ',Align(FWriter.Size,SectionDataAlign)-FWriter.Size,' SectionDataAlign:',SectionDataAlign);
                FWriter.Writezeros(Align(FWriter.Size,SectionDataAlign)-FWriter.Size);
>>>>>>> origin/cpstrnew
                for i:=0 to ObjSectionList.Count-1 do
                  begin
                    objsec:=TObjSection(ObjSectionList[i]);
                    if oso_data in objsec.secoptions then
                      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                        { objsection must be within SecAlign bytes from the previous one }
                        dpos:=objsec.MemPos-MemPos+DataPos;
                        pad:=dpos-FWriter.Size;
                        if (dpos<FWriter.Size) or
                         (pad>=max(objsec.SecAlign,1)) then
                          internalerror(200602251);
                        if assigned(exemap) then
                          if objsec.data.size > 0 then
                            exemap.Add('  0x'+hexstr(dpos,8)+': '+objsec.name);
=======
                        if assigned(exemap) then
                          if objsec.data.size > 0 then
                            exemap.Add('  0x'+hexstr(objsec.DataPos,8)+': '+objsec.name);
>>>>>>> graemeg/cpstrnew
=======
                        if assigned(exemap) then
                          if objsec.data.size > 0 then
                            exemap.Add('  0x'+hexstr(objsec.DataPos,8)+': '+objsec.name);
>>>>>>> graemeg/cpstrnew
=======
                        if assigned(exemap) then
                          if objsec.data.size > 0 then
                            exemap.Add('  0x'+hexstr(objsec.DataPos,8)+': '+objsec.name);
>>>>>>> graemeg/cpstrnew
=======
                        if assigned(exemap) then
                          if objsec.data.size > 0 then
                            exemap.Add('  0x'+hexstr(objsec.DataPos,8)+': '+objsec.name);
>>>>>>> origin/cpstrnew
                        //writeln ('   ',objsec.name,'  size:',objsec.size,'  relocs:',objsec.ObjRelocations.count,'  DataPos:',objsec.DataPos,' MemPos:',objsec.MemPos);
                        {for j := 0 to objsec.ObjRelocations.count-1 do
                          begin
                            objreloc := TObjRelocation(objsec.ObjRelocations[j]);
                            with objreloc do
                            begin
                              write('        reloc DataOffset: ',DataOffset,'  OrgSize:',OrgSize,' typ:',typ);
                              if assigned(symbol) then
                                write(' Name: '#39,symbol.Name,#39' bind:',symbol.bind,' address:',symbol.address,' Size:',symbol.size);
                              writeln;
                            end;
                          end;}
                        if not assigned(objsec.data) then
                          internalerror(200603042);
                        if copy (objsec.Name,1,5) = '.text' then
                          begin        // write NOP's instead of zero's for .text, makes disassemble possible
                            b := $90;  // NOP
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                            for j := 1 to pad do
                                FWriter.write(b,1);
                          end else
                            FWriter.writezeros(pad);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
                            if objsec.DataAlignBytes > 0 then
                              for j := 1 to objsec.DataAlignBytes do
                                FWriter.write(b,1);
                          end else
                            FWriter.writezeros(objsec.dataalignbytes);
                        //if objsec.dataalignbytes>0 then
                        //  writeln ('  ',name,'  alignbytes: ',objsec.dataalignbytes);
                        if objsec.DataPos<>FWriter.Size then
                            internalerror(200602251);
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
                        FWriter.writearray(objsec.data);
                      end else
                      begin
                        if assigned(exemap) then //TExeMap
                          exemap.Add('                  skipping: '+objsec.name);
                      end;
                  end;
              end;
          end;
      end;


    function TNLMexeoutput.totalheadersize:longword;
      var
        varHdrSize,
        optHdrSize,
        hdrSize: longword;
      begin
        optHdrSize := 0;
        inc(optHdrSize,2+length(nlmDescription));
        inc(optHdrSize,8); // Stacksize+reserved
        inc(optHdrSize,NLM_OLD_THREAD_NAME_LENGTH);
        inc(optHdrSize,2+length(nlmScreenname));
        inc(optHdrSize,2+length(nlmThreadname));

        varHdrSize := 0;
        if nwcopyright <> '' then
          inc(varHdrSize,sizeof(NLM_COPYRIGHT_STAMP)+2+length(nlmCopyright));
        hdrSize := sizeof(nlm32_i386_external_fixed_header)+
                   sizeof(nlm32_i386_external_extended_header)+
                   sizeof(nlm32_i386_external_custom_header)+
                   sizeof(nlm32_i386_external_version_header)+     // always
                   sizeof(nlm32_i386_external_cygnus_ext_header)+  // CyGnUsEx
                   varHdrSize+optHdrSize+
                   8;  // empty stamp
        result := hdrSize;
      end;


    procedure TNLMexeoutput.MemPos_Header;
      begin
        { calculate start positions after the headers }
        currmempos:=0;
      end;


  procedure TNLMexeoutput.ExeSectionList_write_header(p:TObject;arg:pointer);
  var
    nam : string;
    u32,al : longword;
    alignAmount:longint;
      begin
        with tExeSection(p) do
          begin
            //comment (v_debug,'ExeSectionList_write_header: '+name);
            nam := name;
            alignAmount := 4 - ((length (nam) + 1) MOD 4);
            FWriter.write(nam[1],length(nam));
            FWriter.WriteZeros(1+alignAmount);
            al := 0;
            // for .stab we have to ignore leading zeros due to alignment in file
            if nam='.stab' then
              if assigned(ObjSectionList[0]) then
                al := TObjSection(ObjSectionList[0]).dataalignbytes;
            u32 := dataPos+al; FWriter.write(u32,sizeof(u32));
            u32 := size-al; FWriter.write(u32,sizeof(u32));
          end;
      end;



    procedure TNLMexeoutput.ExeSectionList_pass2_header(p:TObject;arg:pointer);
    var len,alignAmount:longint;
      begin
        {list of sections, extension of binutils,CuStHeAd points to this list
          The format of the section information is:
           null terminated section name
           zeroes to adjust to 4 byte boundary
           4 byte section data file pointer
           4 byte section size }

        with TExeSection(p) do
          begin
            alignAmount := 4 - ((length (Name) + 1) MOD 4);
            len := length(name) + 1 + alignAmount + 8;
            if ObjSectionList.Count>0 then
              inc(len,TObjSection(ObjSectionList[0]).dataalignbytes);
            inc(plongint(arg)^,len);
          end;
      end;

    procedure TNLMexeoutput.DataPos_Header;
      begin
        ExeSecsListSize:=0;
        ExeSectionList.ForEachCall(@ExeSectionList_pass2_header,@ExeSecsListSize);

        headerAlignBytes := align(totalheadersize+ExeSecsListSize,16)-(totalheadersize+ExeSecsListSize);  // align as in TObjData.sectiontype2align
        currdatapos:=totalheadersize+ExeSecsListSize+headerAlignBytes;
      end;


    procedure TNLMexeoutput.fillNlmVersionHeader;
    var
        hour,min,sec,hsec,Year,Month,Day : word;
    begin
      DecodeTime(Time,hour,min,sec,hsec);
      DecodeDate(Date,year,month,day);
      nlmVersionHeader.stamp := NLM_VERSION_STAMP;
      if nlmVersionHeader.year = 0 then
        begin
          nlmVersionHeader.year := Year;
          nlmVersionHeader.month := Month;
          nlmVersionHeader.day := Day;
        end;
    end;



    function TNLMexeoutput.writedata:boolean;
      var
        dummyLong       : array[0..4] of char;
        textExeSec,
        dataExeSec,
        bssExeSec,
        relocsExeSec,
        exportsExeSec,
        importsExeSec,
        xdcExeSec,
        messagesExeSec,
        helpExeSec,
        customExeSec    : TExeSection;
        hassymbols      : boolean;
        nlmCygnusHeader : nlm32_i386_external_cygnus_ext_header;
        ModuleName      : string;
        exesym          : TExeSymbol;
        expOffset       : PtrUInt;
        expAddr         : longword;
        i               : integer;

      begin
        result:=false;
        textExeSec:=FindExeSection('.text');
        dataExeSec:=FindExeSection('.data');
        bssExeSec:=FindExeSection('.bss');
        relocsExeSec:=FindExeSection('.reloc');
        importsExeSec:=FindExeSection('.imports');
        exportsExeSec:=FindExeSection('.exports');
        xdcExeSec:=FindExeSection('.xdc');
        messagesExeSec:=FindExeSection('.messages');
        helpExeSec:=FindExeSection('.help');
        customExeSec:=FindExeSection('.custom');

        // exported function need the upper bit in the address
        // to be set (=CODE), do this here to avoid another
        // reloc type. The ExportFunctionOffsets list was
        // filled in GenerateExports
        if FexportFunctionOffsets.Count>0 then
          begin
            if not assigned(exportsExeSec) then
              internalerror(201103201);   // we have to have a .export section
            if not assigned(exportsExeSec.ObjSectionList[0]) then
              internalerror(201103202);   // nothing in the .exports section but we have data in FexportFunctionOffsets
            for i := 0 to FexportFunctionOffsets.Count-1 do
              begin
                expOffset := PtrUint(FexportFunctionOffsets[i]);
                if TObjSection(exportsExeSec.ObjSectionList[0]).Data.size < expOffset+3 then
                  internalerror(201103203);  // offset in FexportFunctionOffsets out of range
                with TObjSection(exportsExeSec.ObjSectionList[0]) do
                begin  // set the upper bit of address to indicate .text
                  Data.seek(expOffset);
                  Data.read(expAddr,4);
                  Data.seek(expOffset);
                  expAddr := expAddr or $80000000;
                  Data.write(expAddr,4);
                end;
              end;
           end;

        if not assigned(TextExeSec) or
           not assigned(RelocsExeSec) or
           not assigned(DataExeSec) then
          internalerror(200602231);   // we have to have .data, .text and .reloc
        { do we need to write symbols? }
        hassymbols:=(ExeWriteMode=ewm_dbgonly) or
                    (
                     (ExeWriteMode=ewm_exefull) and
                     not(cs_link_strip in current_settings.globalswitches)
                    );

        { Initial header, will be updated later }
        nlmHeader.signature := NLM_SIGNATURE;
        nlmHeader.version := NLM_HEADER_VERSION;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        moduleName := upperCase(current_module.exefilename);
=======
        moduleName := upperCase(current_module.exefilename^);
>>>>>>> graemeg/cpstrnew
=======
        moduleName := upperCase(current_module.exefilename^);
>>>>>>> graemeg/cpstrnew
=======
        moduleName := upperCase(current_module.exefilename^);
>>>>>>> graemeg/cpstrnew
=======
        moduleName := upperCase(current_module.exefilename^);
>>>>>>> origin/cpstrnew
        nlmHeader.moduleName := moduleName;
        nlmHeader.codeImageOffset := TextExeSec.DataPos+TObjSection(TextExeSec.ObjSectionList[0]).dataalignbytes; // ??? may be that align has to be moved to fixups/imports
        nlmHeader.codeImageSize := TextExeSec.Size;
        nlmHeader.dataImageOffset := DataExeSec.DataPos;
        nlmHeader.dataImageSize := DataExeSec.Size;
        if assigned(BSSExeSec) then
          nlmHeader.uninitializedDataSize:=BSSExeSec.Size;
        if assigned(customExeSec) then
          begin
            nlmHeader.customDataOffset := customExeSec.DataPos;
            nlmHeader.customDataSize := customExeSec.Size;
          end;
        if FNumModules > 0 then
          begin
            nlmHeader.moduleDependencyOffset := FindExeSection('.modules').DataPos+4;  // 4 bytes dummy
            nlmHeader.numberOfModuleDependencies := FNumModules;
          end;
        nlmHeader.relocationFixupOffset := relocsExeSec.DataPos;
        nlmHeader.numberOfRelocationFixups := FNumRelocs;
        nlmHeader.externalReferencesOffset := importsExeSec.DataPos+4;  // 4 bytes dummy
        nlmHeader.numberOfExternalReferences := FNumExternals;
        if assigned(exportsExeSec) then
          if exportsExeSec.Size>0 then
          begin
            nlmHeader.publicsOffset := exportsExeSec.dataPos;
            nlmHeader.numberOfPublics := FnumExports;
          end;
        nlmHeader.codeStartOffset := EntrySym.Address;

        {exit function}
        exesym:=texesymbol(ExeSymbolList.Find('_Stop'));
        if assigned(exesym) then
          nlmHeader.exitProcedureOffset := exesym.ObjSymbol.address;

        {check exit function}
        exesym:=texesymbol(ExeSymbolList.Find('FPC_NW_CHECKFUNCTION'));
        if assigned(exesym) then
          nlmHeader.checkUnloadProcedureOffset := exesym.ObjSymbol.address;

        // calc file pos after all exesections
        fSizeWoDebugSyms := totalheadersize + ExeSecsListSize + headerAlignBytes;
        ExeSectionList.ForEachCall(@ExeSectionList_calc_size,nil);

        nlmExtHeader.stamp := NLM_MESSAGES_STAMP;
        //extHeader.languageID    // TODO: where to get this from ?
        if assigned(messagesExeSec) then
          begin
           nlmExtHeader.messageFileOffset := messagesExeSec.DataPos;
           nlmExtHeader.messageFileLength := messagesExeSec.Size;
          end;
        //nlmExtHeader.messageCount  // TODO: how is messageCount set ?
        if assigned(helpExeSec) then
          begin
           nlmExtHeader.helpFileOffset := helpExeSec.DataPos;
           nlmExtHeader.helpFileLength := helpExeSec.Size;
          end;
        //nlmExtHeader.productID     // TODO: were does this came from ?
        if assigned(xdcExeSec) then
          begin
            nlmExtHeader.RPCDataOffset  := xdcExeSec.DataPos;
            nlmExtHeader.RPCDataLength  := xdcExeSec.Size;
          end;

        if hassymbols then
        begin
          nlmHeader.debugInfoOffset := fSizeWoDebugSyms;
          ExeSymbolList.ForEachCall(@globalsyms_create_symbol,nil);
          nlmHeader.numberOfDebugRecords := FNumDebugSymbols;
        end;

        fillNlmVersionHeader;
        FWriter.write(nlmHeader,sizeof(nlmHeader));

        { variable header }
        NLMWriteString(nlmDescription,true);
        if stacksize < NLM_DEFAULT_STACKSIZE then stacksize := NLM_DEFAULT_STACKSIZE;
        FWriter.Write(stacksize,4);
        FWriter.writezeros(4);
        dummyLong := ' LONG';
        FWriter.Write(dummyLong,sizeof(dummyLong));  // old thread name
        NLMWriteString(nlmScreenname,true);
        NLMWriteString(nlmThreadname,true);

        {version}
        FWriter.Write(nlmVersionHeader,sizeof(nlmVersionHeader));
        {copyright}
        if nlmCopyright <> '' then
        begin
          FWriter.write(NLM_COPYRIGHT_STAMP,sizeof(NLM_COPYRIGHT_STAMP));
          NLMWriteString(nlmCopyright,true);
        end;
        {messages}
        FWriter.write(nlmExtHeader,sizeof(nlmExtHeader));

        {custhead}
        nlmCustHeader.stamp := NLM_CUSTOM_STAMP;
        nlmCustHeader.dataLength := ExeSecsListSize;
        nlmCustHeader.dataOffset := totalheadersize;
        nlmCustHeader.hdrLength := $10;               // why 16 ?, this is what binutils write
        FWriter.write(nlmCustHeader,sizeof(nlmCustHeader));

        {CyGnUsEx}
        // bfd has a strange way to read the sections:
        // the section directory is written under CuStHeAd
        // when bfd finds the neader "CyGnUsEx", it uses the
        // offset and size from CuStHeAd to read the section table

        nlmCygnusHeader.stamp  := NLM_CYGNUS_STAMP;  // CyGnUsEx
        // ld writes some unknown values here, bfd irgnores the values at all
        // lets write the offset and length of the segment table
        nlmCygnusHeader.offset := nlmCustHeader.dataLength;
        nlmCygnusHeader.length := nlmCustHeader.dataOffset;
        FWriter.write(nlmCygnusHeader,sizeof(nlmCygnusHeader));
        FWriter.WriteZeros(8);   // empty stamp + align next to 16 bytes

        if FWriter.Size<>totalheadersize then
          internalerror(201103061);               // headersize <> header written

        { Section headers, CuStHeAd points to this section, not needed by
          netware. Can be used to find the section in the nlm file, binutils
          will use this section }
        ExeSectionList.ForEachCall(@ExeSectionList_write_header,nil);
        FWriter.WriteZeros(headerAlignBytes);
        if FWriter.Size<>totalheadersize+ExeSecsListSize+headerAlignBytes then
          internalerror(201103062);
        { Section data }
        if assigned(exemap) then
          begin
            exemap.Add('');
            exemap.Add('NLM file offsets:');
          end;
        ExeSectionList.ForEachCall(@ExeSectionList_write_data,nil);

        if hassymbols then
          FWriter.writearray(NlmSymbols);  // specific symbols for the internal netware debugger

        result:=true;
      end;



    procedure TNLMexeoutput.GenerateLibraryImports(ImportLibraryList:TFPHashObjectList);
      var
        idata5objsection : TObjSection;
        basedllname : string;

        function AddImport(const afuncname,amangledname:string; isvar:boolean):TObjSymbol;
        var
          secname:string;
        begin
          //Comment (V_Debug,'TNLMexeoutput.GenerateLibraryImports.AddImport '+afuncName);
          result:=nil;
          if assigned(exemap) then
            exemap.Add(' Importing Function '+afuncname);

          if not isvar then
            with internalobjdata do
            begin
              secname:=basedllname+'_i_'+amangledname;
              idata5objsection:=createsection(sec_idata5, secname);
              internalobjdata.SetSection(idata5objsection);
              result:=internalobjdata.SymbolDefine('_'+amangledname,AB_IMPORT,AT_FUNCTION);
            end;
        end;

      var
        i,j           : longint;
        ImportLibrary : TImportLibrary;
        ImportSymbol  : TImportSymbol;
        exesym        : TExeSymbol;
        importAddressList : TFPObjectList;
      begin
        if ImportLibraryList.Count > 0 then
          begin
            {objsec:=}internalObjData.createsection('.imports',0,[oso_data,oso_keep]);
            i := 0;
            internalobjdata.writebytes(i,4);  // dummy to avoid deletion
            {objsec:=}internalObjData.createsection('.modules',0,[oso_data,oso_keep]);
            internalobjdata.writebytes(i,4);  // dummy to avoid deletion
          end;
        for i:=0 to ImportLibraryList.Count-1 do
          begin
            ImportLibrary:=TImportLibrary(ImportLibraryList[i]);
            idata5objsection:=nil;
            for j:=0 to ImportLibrary.ImportSymbolList.Count-1 do
              begin
                ImportSymbol:=TImportSymbol(ImportLibrary.ImportSymbolList[j]);
                exesym:=TExeSymbol(ExeSymbolList.Find(ImportSymbol.MangledName));
                if assigned(exesym) and
                   (exesym.State<>symstate_defined) then
                  begin
                    basedllname:=ExtractFileName(ImportLibrary.Name);
                    exesym.objsymbol:=AddImport(ImportSymbol.Name,ImportSymbol.MangledName,ImportSymbol.IsVar);
                    exesym.State:=symstate_defined;
                    importAddressList := TFPObjectList.create(false);
                    nlmImports.Add(ImportSymbol.Name,importAddressList);
                    if pos('.',basedllname) = 0 then
                      basedllname := basedllname + '.nlm';
                    nlmImpNames.Add(ImportSymbol.Name,TStringObj.create(lower(basedllname)));
                  end;
              end;
          end;

        PackUnresolvedExeSymbols('after DLL imports');
        GenerateExports;
      end;



    procedure TNLMexeoutput.GenerateImports;
    var
        exesec,
        impexesec  : TExeSection;
        objsec     : TObjSection;
        objreloc   : TObjRelocation;
        i,j,k      : integer;
        importAddressList : TFPObjectList;
        name,mName : string;
        b          : byte;
        modules    : string;
        modName    : TStringObj;
    begin
      if FImportsGenerated then exit;
      FImportsGenerated := true;
      impexesec:=FindExeSection('.imports');
      if impexesec=nil then exit;

      for i:=0 to ExeSectionList.Count-1 do
        begin
          exesec:=TExeSection(ExeSectionList[i]);
          for j:=0 to exesec.ObjSectionList.count-1 do
            begin
              objsec:=TObjSection(exesec.ObjSectionList[j]);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
              if j=0 then
                begin
                  exesec.DataPos:=objSec.DataPos;
                  exesec.MemPos:=objSec.MemPos;
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
              if (copy(objsec.name,1,5) <> '.text') and (copy(objsec.name,1,4) <> '.bss') and (copy(objsec.name,1,5) <> '.data') then
                  continue;
              for k:=0 to objsec.ObjRelocations.Count-1 do
                begin
                  objreloc := TObjRelocation(objsec.ObjRelocations[k]);
                  if assigned(objreloc.symbol) then
                    begin
                      //writeln (objreloc.symbol.name,' ',objreloc.symbol.bind);
                      if objreloc.symbol.bind = AB_IMPORT then
                        begin
                          importAddressList := TFPObjectList(nlmImports.Find(objreloc.symbol.name));
                          if assigned(importAddressList) then
                            begin
                              objreloc.objsection := objsec;   // points to idata5
                              importAddressList.Add(objreloc);
                            end else
                            begin
                              comment(v_error,objreloc.symbol.name+' is external but not defined in nlm imports');
                            end;
                        end;
                    end
                end;
            end;
        end;

      modules := '';
      for i := 0 to nlmImports.count-1 do
        begin
          importAddressList := TFPObjectList(nlmImports.Items[i]);
          if importAddressList.Count > 0 then
            begin
              name := nlmImports.NameOfIndex(i);

              // find the module to be imported and add it to the list
              // of modules to be auto loaded
              modName := TStringObj(nlmImpNames.Find(name));
              if assigned(modName) then
                begin
                  mName := modName.Value;
                  if mName <> '' then
                    if copy(mName,1,1) <> '!' then  // special, with ! only the imp will be included but no module is autoloaded, needed i.e. for netware.imp
                      begin
                        if pos(mName+';',modules) < 1 then
                          begin
                            modules := modules + mName + ';';
                            inc(FNumModules);
                          end;
                      end;
                end;
              internalobjdata.SetSection(TObjSection(impexesec.ObjSectionList[0]));
              objNLMwriteString (name,false);           // name of symbol
              k := importAddressList.Count;
              internalobjdata.writebytes(k,sizeof(k));    // number of references
              inc(FNumExternals);
              for j := 0 to importAddressList.Count-1 do
                begin
                  objreloc := TObjRelocation(importAddressList[j]);
                  objsec := objreloc.objsection;
                  if oso_executable in objreloc.objsection.SecOptions then
                    begin
                      if objreloc.typ <> RELOC_RELATIVE then comment(v_error,'reference to external symbols must be RELOC_RELATIVE');
                      // TODO: how to check if size is 4 ????

                      k := objsec.MemPos + objreloc.DataOffset;
                      k := k or $40000000;
                      // TODO: data|code if we support importing data symbols
                      //       i do not know if this is possible with netware
                      internalobjdata.writebytes(k,sizeof(k));    // address

                      // the netware loader requires an offset at the import address
                      // for call = E8 this is -4
                      // TODO: how can we check the needed offset ??
                      if objreloc.DataOffset > 0 then
                        begin
                          objsec.Data.seek(objreloc.DataOffset-1);
                          objsec.data.read(b,1);
                          if b <> $E8 then
                            comment(v_error,'no rcall (E8) before imported symbol target address');
                          k := -4;
                          objsec.Data.write(k,sizeof(k));
                        end else
                        begin
                          objsec.Data.seek(objreloc.DataOffset);
                          k := 0;
                          objsec.Data.write(k,sizeof(k));
                        end;
                        objreloc.typ := RELOC_NONE;  // to avoid that TCoffObjSection.fixuprelocs changes the address again
                    end else
                      comment(v_error,'Importing of symbols only supported for .text');
                end;
            end;
        end;

      exesec := FindExeSection('.modules');
      if not assigned(exesec) then internalerror(201103272);  // exe section .modules does not exist ???
      internalobjdata.SetSection(TObjSection(exesec.ObjSectionList[0]));
      for i := 1 to FNumModules do
        begin
          name := GetToken(modules,';');
          objNLMwriteString (name,false);
        end;
    end;


    procedure TNLMexeoutput.GenerateExports;
    var
        hp  : texported_item;  { for exports }
        len : byte;
        addr: longword;
        exesym : texesymbol;
    begin
      internalObjData.createsection('.exports',0,[oso_data,oso_keep]);
      {name   LString                                   1+n bytes
      addr                                               4 bytes
      addr and $80000000 > 0 -> .text else .data}
      hp:=texported_item(current_module._exports.first);
      if assigned(hp) then
        if assigned(exemap) then
           exemap.Add('');
      while assigned(hp) do
        begin
          { Export the Symbol }
          if assigned(exemap) then
            exemap.Add(' Exporting Function '+hp.sym.prettyname+' as '+hp.name^);
          len := length(hp.name^);
          internalobjdata.writebytes(len,1);
          internalobjdata.writebytes(hp.name^[1],len);
          exesym:=texesymbol(ExeSymbolList.Find(hp.sym.prettyname));
          if not assigned(exesym) then
          begin
            comment(v_error,'exported symbol '+hp.sym.prettyname+' not found');
            exit;
          end;
          // for exported functions we have to set the upper bit
          // this will be done in .writedata
          if not hp.is_var then
            FexportFunctionOffsets.Add(pointer(PtrUInt(internalobjdata.CurrObjSec.Size)));
          internalobjdata.writereloc(0,4,exesym.ObjSymbol,RELOC_ABSOLUTE32);

          addr := 0;
          internalobjdata.writebytes(addr,4);
          inc(FnumExports);
          hp:=texported_item(hp.next);
        end;
    end;

    procedure TNLMexeoutput.GenerateRelocs;

      var
        exesec : TExeSection;
        objsec : TObjSection;
        objreloc : TObjRelocation;
        i,j,k : longint;
        offset : longword;
        inSec,toSec : TsecType;
        targetSectionName : string;

      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        if FRelocsGenerated then
=======
        if not RelocSection or FRelocsGenerated then
>>>>>>> graemeg/cpstrnew
=======
        if not RelocSection or FRelocsGenerated then
>>>>>>> graemeg/cpstrnew
=======
        if not RelocSection or FRelocsGenerated then
>>>>>>> graemeg/cpstrnew
=======
        if not RelocSection or FRelocsGenerated then
>>>>>>> origin/cpstrnew
          exit;
        exesec:=FindExeSection('.reloc');
        if exesec=nil then
          exit;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        objsec:=internalObjData.createsection('.reloc',0,[oso_data,oso_load,oso_keep]);
=======
        objsec:=internalObjData.createsection('.reloc',0,exesec.SecOptions+[oso_data]);
>>>>>>> graemeg/cpstrnew
=======
        objsec:=internalObjData.createsection('.reloc',0,exesec.SecOptions+[oso_data]);
>>>>>>> graemeg/cpstrnew
=======
        objsec:=internalObjData.createsection('.reloc',0,exesec.SecOptions+[oso_data]);
>>>>>>> graemeg/cpstrnew
=======
        objsec:=internalObjData.createsection('.reloc',0,exesec.SecOptions+[oso_data]);
>>>>>>> origin/cpstrnew
        exesec.AddObjSection(objsec);
        for i:=0 to ExeSectionList.Count-1 do
          begin
            exesec:=TExeSection(ExeSectionList[i]);
            for j:=0 to exesec.ObjSectionList.count-1 do
              begin
                objsec:=TObjSection(exesec.ObjSectionList[j]);
                //writeln ('Relocs for ',exesec.name,' - ',objsec.name);
                { create relocs only for sections which are loaded in memory }
                if not (oso_load in objsec.SecOptions) then
                  continue;
                { create relocs only for .text and .data }
                inSec := SectionType (objsec.name);
                if (inSec <> Section_Text) and (inSec <> Section_Data) then
                  continue;

                for k:=0 to objsec.ObjRelocations.Count-1 do
                  begin
                    objreloc:=TObjRelocation(objsec.ObjRelocations[k]);
                    if objreloc.typ <> RELOC_ABSOLUTE then
                      continue;
                    offset:=objsec.MemPos+objreloc.dataoffset;
                    targetSectionName := '';
                    if objreloc.symbol <> nil then
                    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                      // writeln ('  MemPos',objsec.MemPos,
                      // ' dataOfs:',objreloc.dataoffset,' ',objsec.name,
                      // '   objreloc.symbol: ',objreloc.symbol.name,
                      // '  objreloc.symbol.objsection.name: ',objreloc.symbol.objsection.name,
                      // ' ',objreloc.symbol.Typ,' ',objrel
                      // oc.symbol.bind,' ',objreloc.Typ);
=======
                      // writeln ('  MemPos',objsec.MemPos,' dataOfs:',objreloc.dataoffset,' ',objsec.name,'   objreloc.symbol: ',objreloc.symbol.name,'  objreloc.symbol.objsection.name: ',objreloc.symbol.objsection.name,' ',objreloc.symbol.Typ,' ',objreloc.symbol.bind,' ',objreloc.Typ);
>>>>>>> graemeg/cpstrnew
=======
                      // writeln ('  MemPos',objsec.MemPos,' dataOfs:',objreloc.dataoffset,' ',objsec.name,'   objreloc.symbol: ',objreloc.symbol.name,'  objreloc.symbol.objsection.name: ',objreloc.symbol.objsection.name,' ',objreloc.symbol.Typ,' ',objreloc.symbol.bind,' ',objreloc.Typ);
>>>>>>> graemeg/cpstrnew
=======
                      // writeln ('  MemPos',objsec.MemPos,' dataOfs:',objreloc.dataoffset,' ',objsec.name,'   objreloc.symbol: ',objreloc.symbol.name,'  objreloc.symbol.objsection.name: ',objreloc.symbol.objsection.name,' ',objreloc.symbol.Typ,' ',objreloc.symbol.bind,' ',objreloc.Typ);
>>>>>>> graemeg/cpstrnew
=======
                      // writeln ('  MemPos',objsec.MemPos,' dataOfs:',objreloc.dataoffset,' ',objsec.name,'   objreloc.symbol: ',objreloc.symbol.name,'  objreloc.symbol.objsection.name: ',objreloc.symbol.objsection.name,' ',objreloc.symbol.Typ,' ',objreloc.symbol.bind,' ',objreloc.Typ);
>>>>>>> origin/cpstrnew
                      if objreloc.symbol.objsection.name[1] <> '.' then
                        targetSectionName := objreloc.symbol.name                       // specials like __bss_start__
                      else                                                              // dont use objsection.name because it begins with *
                        targetSectionName := copy(objreloc.symbol.objsection.name,1,5); // all others begin with .segment, we only have to check for .text, .data or .bss
                    end else
                      internalerror(2011030603);

                    toSec := SectionType(targetSectionName);

                    if (toSec = Section_Text) or (toSec = Section_Data) then
                    begin
                      if (inSec = Section_text) then offset := offset or $40000000;
                      if (toSec = Section_text) then offset := offset or $80000000;
                      internalObjData.writebytes(offset,4);
                      inc(FNumRelocs);
                    end;
                  end;
              end;
          end;
        FRelocsGenerated:=true;
      end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure TNLMexeoutput.MemPos_Start;
      var
        exesec : TExeSection;
      begin
        exesec:=FindExeSection('.reloc');
        if exesec=nil then
          InternalError(2012072602);
        exesec.Disabled:=false;
        inherited;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    procedure TNLMexeoutput.Order_End;
      var
        exesec : TExeSection;
      begin
        inherited;
        exesec:=FindExeSection('.reloc');
        if exesec=nil then
          exit;
        exesec.SecOptions:=exesec.SecOptions + [oso_Data,oso_keep,oso_load];
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


      procedure TNLMexeoutput.MemPos_ExeSection(const aname:string);
        begin
          if aname='.reloc' then
            GenerateRelocs;
          if aname='.imports' then
            GenerateImports;
          if aname='.data' then
            currMemPos := 0;  // both, data and code in the nlm have a start offset of 0
          inherited;
        end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      procedure TNLMexeoutput.DataPos_ExeSection(const aname:string);
        begin
          inherited;
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
      procedure TNLMexeoutput.NLMwriteString (const s : string; terminateWithZero : boolean);
      var len : byte;
        begin
          if length(s) > 254 then len := 254 else len := length(s);
          FWriter.Write(len,1);
          if len > 0 then
            FWriter.write(s[1],len);
          if terminateWithZero then
            FWriter.writeZeros(1);
        end;


      procedure TNLMexeoutput.objNLMwriteString (const s : string; terminateWithZero : boolean);
      var len : byte;
        begin
          if length(s) > 254 then len := 254 else len := length(s);
          Internalobjdata.writebytes(len,1);
          if len > 0 then
            Internalobjdata.writebytes(s[1],len);
          if terminateWithZero then
          begin
            len := 0;
            Internalobjdata.writebytes(s[1],len);
          end;
        end;

      { parse netware specific linker options }
      procedure TNLMexeoutput.ParseScript (linkscript:TCmdStrList);
      var
        hp : TCmdStrListItem;
        opt,keyword,s : string;
        i : integer;

          function toInteger(s:string; min,max:integer; var res:integer):boolean;
          var
            code:word;
          begin
            result := false;
            val (s,res,code);
            if code<>0 then exit;
            if (res < min) or (res > max) then exit;
            result := true;
          end;


          procedure loadFile (const secName, fileName, Desc : string);
          var
            fileBuf : array [0..4095] of char;
            bytesRead : longint;
            fileH : THandle;
            fn : TCmdStr;

            begin
              fn := fileName;
              if not fileExists(fn) then
               if not unitsearchpath.FindFile(fileName,true,fn) then
                 begin
                   comment(v_error,'can not find '+desc+' file '+fileName);
                   exit;
                 end;
               fileH := fileOpen (fn,fmOpenRead);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
               if fileH = THandle(-1) then
=======
               if fileH = -1 then
>>>>>>> graemeg/cpstrnew
=======
               if fileH = -1 then
>>>>>>> graemeg/cpstrnew
=======
               if fileH = -1 then
>>>>>>> graemeg/cpstrnew
=======
               if fileH = -1 then
>>>>>>> origin/cpstrnew
                 begin
                   comment(v_error,'can not open '+desc+' file '+fn);
                   exit;
                  end;
               { load file into section  }
               internalObjData.createsection(secName,0,[oso_data,oso_keep]);
               repeat
                 bytesRead := fileRead(fileH,fileBuf,sizeof(fileBuf));
                 if bytesRead > 0 then
                   internalobjdata.writebytes(fileBuf,bytesRead);
               until bytesRead < sizeof(fileBuf);
               fileClose(fileH);
            end;

        begin
          hp:=TCmdStrListItem(linkscript.first);
          while assigned(hp) do
            begin
              opt:=hp.str;
              if (opt='') or (opt[1]='#') then
                continue;
              keyword:=Upper(GetToken(opt,' '));
              if keyword = 'AUTOUNLOAD' then
                begin
                  nlmHeader.flags:=nlmHeader.flags or NLM_FLAGS_AUTOUNLOAD;
                end else
              if keyword = 'COPYRIGHT' then
                begin
                  nlmCopyright := GetToken(opt,' ');
                end else
              if keyword = 'CUSTOM' then
                begin
                  loadFile ('.custom',GetToken(opt,' '),'custom data');
                end;
              if keyword = 'DATE' then  // month day 4-digit-year
              begin
                if not toInteger(GetToken(opt,' '),1,12,i) then comment(v_error,'DATE: invalid month')
                  else nlmVersionHeader.month := i;
                if not toInteger(GetToken(opt,' '),1,31,i) then comment(v_error,'DATE: invalid day')
                  else nlmVersionHeader.day := i;
                if not toInteger(GetToken(opt,' '),1900,3000,i) then comment(v_error,'DATE: invalid year')
                  else nlmVersionHeader.year := i;
              end else
              if keyword = 'DEBUG' then
              begin
                // ignore
              end else
              if keyword = 'DESCRIPTION' then
                begin
                  nlmDescription := GetToken(opt,' ');
                  if length (nlmDescription) > NLM_MAX_DESCRIPTION_LENGTH then
                    nlmDescription := copy (nlmDescription,1,NLM_MAX_DESCRIPTION_LENGTH);
                end else
              if keyword = 'FLAG' then
                begin
                  s := upper(GetToken(opt,' '));
                  if (not toInteger(GetToken(opt,' '),1,$FFFFFFF,i)) or ((s <> 'ON') and (S <> 'OFF')) then comment(v_error,'FLAG: invalid') else
                    if (s='ON') then
                      nlmHeader.flags:=nlmHeader.flags or i else
                    nlmHeader.flags:=nlmHeader.flags and ($FFFFFFF-i);
                end else
              if keyword = 'HELP' then
                begin
                  loadFile ('.help',GetToken(opt,' '),'help');
                end else
              if keyword = 'MESSAGES' then
                begin
                  loadFile ('.messages',GetToken(opt,' '),'message');
                end else
              if keyword = 'MULTIPLE' then
                begin
                  nlmHeader.flags:=nlmHeader.flags or NLM_FLAGS_MULTILOAD;
                end else
              if keyword = 'OS_DOMAIN' then
                begin
                  nlmHeader.flags:=nlmHeader.flags or NLM_FLAGS_OSDOMAIN;
                end else
              if keyword = 'PSEUDOPREEMPTION' then
                begin
                  nlmHeader.flags:=nlmHeader.flags or NLM_FLAGS_PSEUDOPREEMPTION;
                end else
              if keyword = 'REENTRANT' then
                begin
                  nlmHeader.flags:=nlmHeader.flags or NLM_FLAGS_REENTRANT;
                end else
              if keyword = 'SCREENNAME' then
                begin
                  nlmScreenname := GetToken(opt,' ');
                  if length(nlmScreenname) > NLM_MAX_SCREEN_NAME_LENGTH then
                   nlmScreenName := copy (nlmScreenName,1,NLM_MAX_SCREEN_NAME_LENGTH);
                end else
              if (keyword = 'STACK') or (keyword = 'STACKSIZE') then
                begin
                   if (not toInteger(GetToken(opt,' '),1,$FFFFFFF,i)) then comment(v_error,'invalid stacksize') else
                     stacksize := i;
                end else
              if keyword = 'SYNCHRONIZE' then
                begin
                  nlmHeader.flags:=nlmHeader.flags or NLM_FLAGS_SYNCHRONIZE;
                end else
              if keyword = 'THREADNAME' then
                begin
                  nlmThreadname := GetToken(opt,' ');
                  if length(nlmThreadname) > NLM_MAX_THREAD_NAME_LENGTH then
                    nlmThreadname := copy (nlmThreadname,1,NLM_MAX_THREAD_NAME_LENGTH);
                end else
              if keyword = 'TYPE' then
                begin
                   if (not toInteger(GetToken(opt,' '),1,16,i)) then comment(v_error,'invalid TYPE') else
                     nlmHeader.moduleType := i;  // TODO: set executable extension (.DSK, .LAN, ...)
                end else
              if keyword = 'VERSION' then
                begin
                   if (not toInteger(GetToken(opt,' '),0,$FFFFFFF,i)) then comment(v_error,'invalid major version') else
                     nlmVersionHeader.majorVersion := i;
                   if (not toInteger(GetToken(opt,' '),0,99,i)) then comment(v_error,'invalid minor version') else
                     nlmVersionHeader.minorVersion := i;
                   if (not toInteger(GetToken(opt,' '),0,$FFFFFFF,i)) then comment(v_error,'invalid minor version') else
                     if i > 26 then
                       nlmVersionHeader.revision := 0 else
                       nlmVersionHeader.revision := i;
                end else
              if keyword = 'XDCDATA' then
                begin
                  loadFile ('.xdc',GetToken(opt,' '),'xdc');
                end;
                { TODO: check for unknown options. This means all handled option
                  (also in link.pas) have to be flagged if processed }
              hp:=TCmdStrListItem(hp.next);
            end;
        end;

{****************************************************************************
                                TNLMCoffObjData
****************************************************************************}

    constructor TNLMCoffObjData.create(const n:string);
      begin
        inherited createcoff(n,true,TNLMCoffObjSection);
      end;


{****************************************************************************
                               TNLMoffObjSection
****************************************************************************}

    constructor TNLMCoffObjSection.create(AList:TFPHashObjectList;const aname:string;aalign:shortint;aoptions:TObjSectionOptions);
      begin
        inherited create(alist,aname,aalign,aoptions);
      end;


    constructor TNLMCoffObjOutput.create(AWriter:TObjectWriter);
      begin
        // ??????
        // if win32=false, .stabs and .stabstr will be written without oso_debug
        // Without oso_debug the sections will be removed by the linker
        inherited createcoff(AWriter,{win32}true);
        cobjdata:=TNLMCoffObjData;
      end;

{****************************************************************************
                                 TDJCoffAssembler
****************************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    constructor TNLMCoffAssembler.Create(info: pasminfo; smart:boolean);
      begin
        inherited;
        CObjOutput:=TNLMCoffObjOutput;
        CInternalAr:=tarobjectwriter;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    constructor TNLMCoffAssembler.Create(smart:boolean);
      begin
        inherited Create(smart);
        CObjOutput:=TNLMCoffObjOutput;
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

    constructor TNLMCoffObjInput.create;
      begin
        inherited createcoff(true);
        cobjdata:=TNLMCoffObjData;
      end;

{*****************************************************************************
                                  Initialize
*****************************************************************************}
const
    as_i386_nlmcoff_info : tasminfo =
          (
            id     : as_i386_nlmcoff;
            idtxt  : 'NLMCOFF';
            asmbin : '';
            asmcmd : '';
            supported_targets : [system_i386_Netware,system_i386_netwlibc];
            flags : [af_outputbinary,af_smartlink_sections];
            labelprefix : '.L';
            comment : '';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            dollarsign: '$';
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
          );



initialization
{$ifdef i386}
  RegisterAssembler(as_i386_nlmcoff_info,TNLMCoffAssembler);
{$endif i386}
end.
