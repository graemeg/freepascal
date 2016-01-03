<<<<<<< HEAD
<<<<<<< HEAD
{
constants.pas

Constants

Copyright (C) 2006-2007 Felipe Monteiro de Carvalho

This file is part of MkSymbian build tool.

MkSymbian is free software;
you can redistribute it and/or modify it under the
terms of the GNU General Public License version 2
as published by the Free Software Foundation.

MkSymbian is distributed in the hope
that it will be useful, but WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

Please note that the General Public License version 2 does not permit
incorporating MkSymbian into proprietary programs.
}
unit constants;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils; 

type

  { Options from the command line }

  TMkSymbianTask = (stNothing, stBuildApp, stBuildBindings);

  TMkSymbianOptions = record
    Task: TMkSymbianTask;
    ProjectFile: string;
  end;

{ Constants and types for the SDK version }
type
  TSDKVersion = (sdkUIQ21, sdkUIQ3);

const
  Str_UIQ21 = 'UIQ 2.1';
  Str_UIQ3  = 'UIQ 3';

{ Commands }

const
  paramBuild = 'build';
  paramBindings = 'bindings';
  paramShowPath = 'showpath';

{ Paths on the SDK }
const
  { UIQ 2 Paths }
  Str_Path_UIQ2_ARM_BINUTILS = 'epoc32\gcc\bin\';
  Str_Path_UIQ2_Makmake = 'epoc32\tools\makmake.pl';

  { UIQ 3 Paths }
  Str_Path_CWTools = 'epoc32\tools\nokia_compiler\Symbian_Tools\Command_Line_Tools\';
  Str_Path_RComp = 'epoc32\tools\rcomp.exe';
  Str_Path_Cpp = 'epoc32\tools\scpp.exe';
  Str_Path_Resource_Files = 'epoc32\release\winscw\udeb\Z\resource\apps\';
  Str_Path_Emulator_Registration = 'epoc32\release\winscw\udeb\Z\private\10003a3f\apps\';

{ Other constants }
const
  UID_SOURCE_FILENAME = 'QUID.cpp';
  UID_OBJECT_FILENAME = 'QUID.o';
  
  STR_ASSEMBLER_EXT = '.S';
  STR_OBJECT_EXT = '.o';

  STR_RESOURCE_TMP_EXT = '.pprsc';
  STR_RESOURCE_EXT = '.rsc';

{ Strings from the project file }
const
  { Sections }
  STR_PRJ_Main = 'Main';
  STR_PRJ_FPC = 'FPC';
  STR_PRJ_UIDs = 'UIDs';
  STR_PRJ_Files = 'Files';
  STR_PRJ_Objects = 'Objects';
  STR_PRJ_RTLObjects = 'RTLObjects';

  { Identifiers }
  STR_PRJ_EXEName = 'EXEName';
  STR_PRJ_Language = 'Language';
  STR_PRJ_CompilerDir = 'CompilerDir';
  STR_PRJ_ProjectType = 'ProjectType';
  STR_PRJ_SDK = 'SDK';
  STR_PRJ_SDKVersion = 'SDKVersion';
  STR_PRJ_Emulator = 'Emulator';

  STR_PRJ_CompilerPath = 'CompilerPath';
  STR_PRJ_AssemblerPath = 'AssemblerPath';
  STR_PRJ_RTLUnitsDir = 'RTLUnitsDir';

  STR_PRJ_UID2 = 'UID2';
  STR_PRJ_UID3 = 'UID3';

  STR_PRJ_MainSource = 'mainsource';
  STR_PRJ_MainResource = 'mainresource';
  STR_PRJ_RegResource = 'regresource';

  STR_PRJ_File = 'file';

  { Options }
  
  STR_OPT_Cpp = 'C++';
  STR_OPT_Pascal = 'Pascal';

implementation

end.

=======
=======
>>>>>>> origin/fixes_2_2
{
constants.pas

Constants

Copyright (C) 2006-2007 Felipe Monteiro de Carvalho

This file is part of MkSymbian build tool.

MkSymbian is free software;
you can redistribute it and/or modify it under the
terms of the GNU General Public License version 2
as published by the Free Software Foundation.

MkSymbian is distributed in the hope
that it will be useful, but WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

Please note that the General Public License version 2 does not permit
incorporating MkSymbian into proprietary programs.
}
unit constants;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils; 

type

  { Options from the command line }

  TMkSymbianTask = (stNothing, stBuildApp, stBuildBindings);

  TMkSymbianOptions = record
    Task: TMkSymbianTask;
    ProjectFile: string;
  end;

{ Constants and types for the SDK version }
type
  TSDKVersion = (sdkUIQ21, sdkUIQ3);

const
  Str_UIQ21 = 'UIQ 2.1';
  Str_UIQ3  = 'UIQ 3';

{ Commands }

const
  paramBuild = 'build';
  paramBindings = 'bindings';
  paramShowPath = 'showpath';

{ Paths on the SDK }
const
  { UIQ 2 Paths }
  Str_Path_UIQ2_ARM_BINUTILS = 'epoc32\gcc\bin\';
  Str_Path_UIQ2_Makmake = 'epoc32\tools\makmake.pl';

  { UIQ 3 Paths }
  Str_Path_CWTools = 'epoc32\tools\nokia_compiler\Symbian_Tools\Command_Line_Tools\';
  Str_Path_RComp = 'epoc32\tools\rcomp.exe';
  Str_Path_Cpp = 'epoc32\tools\scpp.exe';
  Str_Path_Resource_Files = 'epoc32\release\winscw\udeb\Z\resource\apps\';
  Str_Path_Emulator_Registration = 'epoc32\release\winscw\udeb\Z\private\10003a3f\apps\';

{ Other constants }
const
  UID_SOURCE_FILENAME = 'QUID.cpp';
  UID_OBJECT_FILENAME = 'QUID.o';
  
  STR_ASSEMBLER_EXT = '.S';
  STR_OBJECT_EXT = '.o';

  STR_RESOURCE_TMP_EXT = '.pprsc';
  STR_RESOURCE_EXT = '.rsc';

{ Strings from the project file }
const
  { Sections }
  STR_PRJ_Main = 'Main';
  STR_PRJ_FPC = 'FPC';
  STR_PRJ_UIDs = 'UIDs';
  STR_PRJ_Files = 'Files';
  STR_PRJ_Objects = 'Objects';
  STR_PRJ_RTLObjects = 'RTLObjects';

  { Identifiers }
  STR_PRJ_EXEName = 'EXEName';
  STR_PRJ_Language = 'Language';
  STR_PRJ_CompilerDir = 'CompilerDir';
  STR_PRJ_ProjectType = 'ProjectType';
  STR_PRJ_SDK = 'SDK';
  STR_PRJ_SDKVersion = 'SDKVersion';
  STR_PRJ_Emulator = 'Emulator';

  STR_PRJ_CompilerPath = 'CompilerPath';
  STR_PRJ_AssemblerPath = 'AssemblerPath';
  STR_PRJ_RTLUnitsDir = 'RTLUnitsDir';

  STR_PRJ_UID2 = 'UID2';
  STR_PRJ_UID3 = 'UID3';

  STR_PRJ_MainSource = 'mainsource';
  STR_PRJ_MainResource = 'mainresource';
  STR_PRJ_RegResource = 'regresource';

  STR_PRJ_File = 'file';

  { Options }
  
  STR_OPT_Cpp = 'C++';
  STR_OPT_Pascal = 'Pascal';

implementation

end.

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
