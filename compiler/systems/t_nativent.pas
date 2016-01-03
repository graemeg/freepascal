{
    Copyright (c) 2009 by Sven Barth

    This unit implements support import,export,link routines
    for the Native NT Target

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
unit t_nativent;

{$i fpcdefs.inc}

interface


implementation

  uses
    SysUtils,
    cutils,
    ogbase,ogcoff,
    globtype,globals,systems,verbose,
    import,export,link,t_win,i_nativent;


    type
      TImportLibNativeNT=class(TImportLibWin)
      end;

      TExportLibNativeNT=class(TExportLibWin)
      end;

      TInternalLinkerNativeNT = class(TInternalLinkerWin)
        constructor create;override;
        procedure ConcatEntryName; override;
      end;

{****************************************************************************
                            TInternalLinkerNativeNT
****************************************************************************}

    constructor TInternalLinkerNativeNT.create;
      begin
        inherited create;
        CExeoutput:=TPECoffexeoutput;
        CObjInput:=TPECoffObjInput;
      end;

    procedure TInternalLinkerNativeNT.ConcatEntryName;
      begin
        with LinkScript do
          begin
            if IsSharedLibrary then
              begin
                // for now we use {$apptype native} for kernel mode code
                if apptype=app_native then
                  Concat('ENTRYNAME _NtDriverEntry')
                else
                  Concat('ENTRYNAME _DLLMainStartup')
              end
                else
                  Concat('ENTRYNAME _NtProcessStartup');
          end;
      end;



{*****************************************************************************
                                     Initialize
*****************************************************************************}

initialization
{$ifdef i386}
  { NativeNT }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  RegisterLinker(ld_int_nativent,TInternalLinkerNativeNT);
=======
  RegisterInternalLinker(system_i386_nativent_info,TInternalLinkerNativeNT);
>>>>>>> graemeg/cpstrnew
=======
  RegisterInternalLinker(system_i386_nativent_info,TInternalLinkerNativeNT);
>>>>>>> graemeg/cpstrnew
=======
  RegisterInternalLinker(system_i386_nativent_info,TInternalLinkerNativeNT);
>>>>>>> graemeg/cpstrnew
=======
  RegisterInternalLinker(system_i386_nativent_info,TInternalLinkerNativeNT);
>>>>>>> origin/cpstrnew
  RegisterImport(system_i386_nativent,TImportLibNativeNT);
  RegisterExport(system_i386_nativent,TExportLibNativeNT);
//  RegisterRes(res_gnu_windres_info,TWinLikeResourceFile);
  RegisterTarget(system_i386_nativent_info);
{$endif i386}
end.
