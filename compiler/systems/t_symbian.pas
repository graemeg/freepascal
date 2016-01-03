{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by contributors of the Free Pascal Compiler

    This unit implements support import,export,link routines
    for the Symbian OS target

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
unit t_symbian;

{$i fpcdefs.inc}

interface

    uses
       cutils,cclasses,
       aasmbase,aasmtai,aasmdata,aasmcpu,fmodule,globtype,globals,systems,verbose,
       symconst,symdef,symsym,
       script,gendef,
       cpubase,
       import,export,link,cgobj, i_symbian;

    type
      TInternalLinkerSymbian = class(TInternalLinker)
        constructor create; override;
        procedure DefaultLinkScript; override;
        procedure InitSysInitUnitName; override;
      end;

implementation

  uses
    SysUtils,
    cfileutl,
    cpuinfo,cgutils,dbgbase,
    owar,ogbase,ogcoff, t_win;

{****************************************************************************
                            TInternalLinkerSymbian
****************************************************************************}

    constructor TInternalLinkerSymbian.Create;
      begin
        inherited Create;
        CArObjectReader:=TArObjectReader;
        CExeoutput:=TPECoffexeoutput;
        CObjInput:=TPECoffObjInput;
      end;


    procedure TInternalLinkerSymbian.DefaultLinkScript;
    var
      s,s2,
      ibase : TCmdStr;
    begin
        with LinkScript do
        begin
            while not ObjectFiles.Empty do
            begin
                s:=ObjectFiles.GetFirst;
                if s<>'' then
                  Concat('READOBJECT '+MaybeQuoted(s));
            end;
            while not StaticLibFiles.Empty do
            begin
                s:=StaticLibFiles.GetFirst;
                if s<>'' then
                  Concat('READSTATICLIBRARY '+MaybeQuoted(s));
            end;
            While not SharedLibFiles.Empty do
            begin
                S:=SharedLibFiles.GetFirst;
                if FindLibraryFile(s,target_info.staticClibprefix,target_info.staticClibext,s2) then
                  Concat('READSTATICLIBRARY '+MaybeQuoted(s2))
                else
                  Comment(V_Error,'Import library not found for '+S);
            end;
            if IsSharedLibrary then
            begin
                Concat('ISSHAREDLIBRARY');
                Concat('ENTRYNAME _E32DLL');
            end
            else
            begin
                Concat('ENTRYNAME _E32Startup')
            end;
            if IsSharedLibrary then
              ibase:='10000000'
            else
                ibase:='400000';
            Concat('IMAGEBASE $' + ibase);
            Concat('HEADER');
            Concat('EXESECTION .text');
            Concat('  OBJSECTION .text*');
            Concat('  SYMBOL ___CTOR_LIST__');
            Concat('  SYMBOL __CTOR_LIST__');
            Concat('  LONG -1');
            Concat('  OBJSECTION .ctor*');
            Concat('  LONG 0');
            Concat('  SYMBOL ___DTOR_LIST__');
            Concat('  SYMBOL __DTOR_LIST__');
            Concat('  LONG -1');
            Concat('  OBJSECTION .dtor*');
            Concat('  LONG 0');
            Concat('  SYMBOL etext');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .data');
            Concat('  SYMBOL __data_start__');
            Concat('  OBJSECTION .data*');
            Concat('  OBJSECTION .fpc*');
            Concat('  SYMBOL edata');
            Concat('  SYMBOL __data_end__');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .rdata');
            Concat('  SYMBOL ___RUNTIME_PSEUDO_RELOC_LIST__');
            Concat('  SYMBOL __RUNTIME_PSEUDO_RELOC_LIST__');
            Concat('  OBJSECTION .rdata_runtime_pseudo_reloc');
            Concat('  SYMBOL ___RUNTIME_PSEUDO_RELOC_LIST_END__');
            Concat('  SYMBOL __RUNTIME_PSEUDO_RELOC_LIST_END__');
            Concat('  OBJSECTION .rdata*');
            Concat('  OBJSECTION .rodata*');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .pdata');
            Concat('  OBJSECTION .pdata');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .bss');
            Concat('  SYMBOL __bss_start__');
            Concat('  OBJSECTION .bss*');
            Concat('  SYMBOL __bss_end__');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .idata');
            Concat('  OBJSECTION .idata$2*');
            Concat('  OBJSECTION .idata$3*');
            Concat('  ZEROS 20');
            Concat('  OBJSECTION .idata$4*');
            Concat('  OBJSECTION .idata$5*');
            Concat('  OBJSECTION .idata$6*');
            Concat('  OBJSECTION .idata$7*');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .edata');
            Concat('  OBJSECTION .edata*');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .rsrc');
            Concat('  OBJSECTION .rsrc*');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .reloc');
            Concat('  OBJSECTION .reloc');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .stab');
            Concat('  OBJSECTION .stab');
            Concat('ENDEXESECTION');
            Concat('EXESECTION .stabstr');
            Concat('  OBJSECTION .stabstr');
            Concat('ENDEXESECTION');
            Concat('STABS');
            Concat('SYMBOLS');
          end;
      end;


    procedure TInternalLinkerSymbian.InitSysInitUnitName;
    begin
      sysinitunit := 'sysinitpas';
    end;

{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
  { Using external linker as internal doesn't look correct... }
  RegisterLinker(ld_windows,TExternalLinkerWin);
  RegisterLinker(ld_int_windows,TInternalLinkerWin);
{$ifdef i386}
<<<<<<< HEAD
<<<<<<< HEAD
  ///RegisterInternalLinker(system_i386_symbian_info,TExternalLinkerWin);
=======
  RegisterInternalLinker(system_i386_symbian_info,TExternalLinkerWin);
>>>>>>> graemeg/fixes_2_2
=======
  RegisterInternalLinker(system_i386_symbian_info,TExternalLinkerWin);
>>>>>>> origin/fixes_2_2
  RegisterImport(system_i386_symbian,TImportLibWin);
  RegisterExport(system_i386_symbian,TExportLibWin);
  RegisterDLLScanner(system_i386_symbian,TDLLScannerWin);
//  RegisterRes(res_gnu_windres_info);
  RegisterTarget(system_i386_symbian_info);
{$endif i386}
{$ifdef arm}
//  RegisterExternalLinker(system_arm_symbian_info,TExternalLinkerWin);
  ///RegisterInternalLinker(system_arm_symbian_info,TInternalLinkerWin);
  RegisterImport(system_arm_symbian,TImportLibWin);
  RegisterExport(system_arm_symbian,TExportLibWin);
  RegisterTarget(system_arm_symbian_info);
{$endif arm}
end.
