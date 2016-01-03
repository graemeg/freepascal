<<<<<<< HEAD
<<<<<<< HEAD
{
projectparser.pas

Parses the project file

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
unit projectparser;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils, IniFiles,
  constants;

type

  { TProject }

  TProject = class(TObject)
  public
    opts: TMkSymbianOptions;
    { Main section }
    ExeName, Language, ProjectType, SDK, SDKVersion: string;
    Emulator: Boolean;
    { FPC section }
    CompilerPath, AssemblerPath, RTLUnitsDir: string;
    { UIDs section }
    UID2, UID3: string;
    { Files section }
    MainSource, MainSourceNoExt, MainSourceAsm, MainSourceObj,
     MainResource, RegResource: string;
    { Objects section }
    ObjectFiles: TStringList;
  public
    constructor Create;
    destructor Destroy; override;
    procedure ParseFile;
  end;

var
  vProject: TProject;

implementation

{ TProject }

constructor TProject.Create;
begin
  inherited Create;
  
  ObjectFiles := TStringList.Create;
end;

destructor TProject.Destroy;
begin
  ObjectFiles.Free;

  inherited Destroy;
end;

{*******************************************************************
*  TProject.ParseFile ()
*
*  DESCRIPTION:    Parses the project file
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TProject.ParseFile;
var
  IniFile: TIniFile;
begin
  IniFile := TIniFile.Create(opts.ProjectFile);
  try
    ExeName := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_ExeName, 'default.exe');
    Language := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_Language, 'Pascal');
    ProjectType := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_ProjectType, 'EXE');
    SDK := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_SDK, 'UIQ');
    SDKVersion := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_SDKVersion, '2.1');
    Emulator := IniFile.ReadBool(STR_PRJ_Files, STR_PRJ_Emulator, False);

    CompilerPath := IniFile.ReadString(STR_PRJ_FPC, STR_PRJ_CompilerPath, 'C:\Programas\fpc21\compiler\ppc386.exe');
    AssemblerPath := IniFile.ReadString(STR_PRJ_FPC, STR_PRJ_AssemblerPath, 'C:\Programas\lazarus20\fpc\2.1.5\bin\i386-win32\as.exe');
    RTLUnitsDir := IniFile.ReadString(STR_PRJ_FPC, STR_PRJ_RTLUnitsDir, 'C:\Programas\fpc21\rtl\units\i386-symbian\');

    UID2 := IniFile.ReadString(STR_PRJ_UIDs, STR_PRJ_UID2, '0x100039CE');
    UID3 := IniFile.ReadString(STR_PRJ_UIDs, STR_PRJ_UID3, '0xE1000002');

    MainSource := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_MainSource, 'default.pas');
    MainSourceNoExt := ExtractFileExt(MainSource);
    MainSourceAsm := ChangeFileExt(MainSource, STR_ASSEMBLER_EXT);
    MainSourceObj := ChangeFileExt(MainSource, STR_OBJECT_EXT);
    MainResource := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_MainResource, 'default.rss');
    RegResource :=  IniFile.ReadString(STR_PRJ_Files, STR_PRJ_RegResource, 'default_reg.rss');
    
    IniFile.ReadSection(STR_PRJ_Objects, ObjectFiles);
  finally
    IniFile.Free;
  end;
end;

end.

=======
=======
>>>>>>> origin/fixes_2_2
{
projectparser.pas

Parses the project file

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
unit projectparser;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils, IniFiles,
  constants;

type

  { TProject }

  TProject = class(TObject)
  public
    opts: TMkSymbianOptions;
    { Main section }
    ExeName, Language, ProjectType, SDK, SDKVersion: string;
    Emulator: Boolean;
    { FPC section }
    CompilerPath, AssemblerPath, RTLUnitsDir: string;
    { UIDs section }
    UID2, UID3: string;
    { Files section }
    MainSource, MainSourceNoExt, MainSourceAsm, MainSourceObj,
     MainResource, RegResource: string;
    { Objects section }
    ObjectFiles: TStringList;
  public
    constructor Create;
    destructor Destroy; override;
    procedure ParseFile;
  end;

var
  vProject: TProject;

implementation

{ TProject }

constructor TProject.Create;
begin
  inherited Create;
  
  ObjectFiles := TStringList.Create;
end;

destructor TProject.Destroy;
begin
  ObjectFiles.Free;

  inherited Destroy;
end;

{*******************************************************************
*  TProject.ParseFile ()
*
*  DESCRIPTION:    Parses the project file
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TProject.ParseFile;
var
  IniFile: TIniFile;
begin
  IniFile := TIniFile.Create(opts.ProjectFile);
  try
    ExeName := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_ExeName, 'default.exe');
    Language := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_Language, 'Pascal');
    ProjectType := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_ProjectType, 'EXE');
    SDK := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_SDK, 'UIQ');
    SDKVersion := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_SDKVersion, '2.1');
    Emulator := IniFile.ReadBool(STR_PRJ_Files, STR_PRJ_Emulator, False);

    CompilerPath := IniFile.ReadString(STR_PRJ_FPC, STR_PRJ_CompilerPath, 'C:\Programas\fpc21\compiler\ppc386.exe');
    AssemblerPath := IniFile.ReadString(STR_PRJ_FPC, STR_PRJ_AssemblerPath, 'C:\Programas\lazarus20\fpc\2.1.5\bin\i386-win32\as.exe');
    RTLUnitsDir := IniFile.ReadString(STR_PRJ_FPC, STR_PRJ_RTLUnitsDir, 'C:\Programas\fpc21\rtl\units\i386-symbian\');

    UID2 := IniFile.ReadString(STR_PRJ_UIDs, STR_PRJ_UID2, '0x100039CE');
    UID3 := IniFile.ReadString(STR_PRJ_UIDs, STR_PRJ_UID3, '0xE1000002');

    MainSource := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_MainSource, 'default.pas');
    MainSourceNoExt := ExtractFileExt(MainSource);
    MainSourceAsm := ChangeFileExt(MainSource, STR_ASSEMBLER_EXT);
    MainSourceObj := ChangeFileExt(MainSource, STR_OBJECT_EXT);
    MainResource := IniFile.ReadString(STR_PRJ_Files, STR_PRJ_MainResource, 'default.rss');
    RegResource :=  IniFile.ReadString(STR_PRJ_Files, STR_PRJ_RegResource, 'default_reg.rss');
    
    IniFile.ReadSection(STR_PRJ_Objects, ObjectFiles);
  finally
    IniFile.Free;
  end;
end;

end.

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
