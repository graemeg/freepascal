<<<<<<< HEAD
<<<<<<< HEAD
{
cmdline.pas

Command line parsing methods

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
unit cmdline;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils,
  constants;

type

  { TCmdLine }

  TCmdLine = class(TObject)
  public
    procedure Usage;
    procedure ShowPath;
    procedure ParseCmdLineOptions(var opts: TMkSymbianOptions);
  end;
  
var
  vCmdLine: TCmdLine;

implementation

uses sdkutil, projectparser;

{ TCmdLine }

{*******************************************************************
*  TCmdLine.Usage ()
*
*  DESCRIPTION:    Shows a usage message for the tool
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TCmdLine.Usage;
begin
  WriteLn('mksymbian - Build tool for Free Pascal for SymbianOS');
  WriteLn('');
  WriteLn('The parameters you specifyed are wrong.');
  WriteLn('');
  WriteLn('Usage: mksymbian [command] [project file]');
  WriteLn('');
  WriteLn('Possible commands: ');
  WriteLn('');
  WriteLn('build    - Builds an application');
  WriteLn('bindings - Builds the c++ bindings for pascal');
  WriteLn('showpath - Show the paths the tool is using');
  WriteLn('');
end;

{*******************************************************************
*  TCmdLine.ShowPath ()
*
*  DESCRIPTION:    Shows in which paths (sdk, fpc, etc) mksymbian is using
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TCmdLine.ShowPath;
begin
  WriteLn('mksymbian - Build tool for Free Pascal for SymbianOS');
  WriteLn('');
  WriteLn('SDK Version: ' + vSDKUtil.StrSDKVersion);
  WriteLn('Location of SDK: ' + vSDKUtil.SDKFolder);
  WriteLn('Location of Free Pascal Compiler: ' + vProject.CompilerPath);
  WriteLn('');
end;

{*******************************************************************
*  TCmdLine.ParseCmdLineOptions ()
*
*  DESCRIPTION:    Parses the command line options utilized to call mksymbian
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TCmdLine.ParseCmdLineOptions(var opts: TMkSymbianOptions);
begin
  FillChar(opts, SizeOf(TMkSymbianOptions), #0);

  if (ParamCount = 0) then
  begin
    Usage;
    Exit;
  end;

  opts.ProjectFile := ParamStr(2);

  if CompareText(ParamStr(1), paramBuild) = 0 then opts.task := stBuildApp
  else if CompareText(ParamStr(1), paramBindings) = 0 then opts.task := stBuildBindings
  else if CompareText(ParamStr(1), paramShowPath) = 0 then
  begin
    opts.task := stNothing;
    vProject.ParseFile;
    ShowPath;
  end
  else
  begin
    opts.task := stNothing;
    Usage;
  end;
end;

end.

=======
=======
>>>>>>> origin/fixes_2_2
{
cmdline.pas

Command line parsing methods

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
unit cmdline;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils,
  constants;

type

  { TCmdLine }

  TCmdLine = class(TObject)
  public
    procedure Usage;
    procedure ShowPath;
    procedure ParseCmdLineOptions(var opts: TMkSymbianOptions);
  end;
  
var
  vCmdLine: TCmdLine;

implementation

uses sdkutil, projectparser;

{ TCmdLine }

{*******************************************************************
*  TCmdLine.Usage ()
*
*  DESCRIPTION:    Shows a usage message for the tool
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TCmdLine.Usage;
begin
  WriteLn('mksymbian - Build tool for Free Pascal for SymbianOS');
  WriteLn('');
  WriteLn('The parameters you specifyed are wrong.');
  WriteLn('');
  WriteLn('Usage: mksymbian [command] [project file]');
  WriteLn('');
  WriteLn('Possible commands: ');
  WriteLn('');
  WriteLn('build    - Builds an application');
  WriteLn('bindings - Builds the c++ bindings for pascal');
  WriteLn('showpath - Show the paths the tool is using');
  WriteLn('');
end;

{*******************************************************************
*  TCmdLine.ShowPath ()
*
*  DESCRIPTION:    Shows in which paths (sdk, fpc, etc) mksymbian is using
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TCmdLine.ShowPath;
begin
  WriteLn('mksymbian - Build tool for Free Pascal for SymbianOS');
  WriteLn('');
  WriteLn('SDK Version: ' + vSDKUtil.StrSDKVersion);
  WriteLn('Location of SDK: ' + vSDKUtil.SDKFolder);
  WriteLn('Location of Free Pascal Compiler: ' + vProject.CompilerPath);
  WriteLn('');
end;

{*******************************************************************
*  TCmdLine.ParseCmdLineOptions ()
*
*  DESCRIPTION:    Parses the command line options utilized to call mksymbian
*
*  PARAMETERS:     None
*
*  RETURNS:        Nothing
*
*******************************************************************}
procedure TCmdLine.ParseCmdLineOptions(var opts: TMkSymbianOptions);
begin
  FillChar(opts, SizeOf(TMkSymbianOptions), #0);

  if (ParamCount = 0) then
  begin
    Usage;
    Exit;
  end;

  opts.ProjectFile := ParamStr(2);

  if CompareText(ParamStr(1), paramBuild) = 0 then opts.task := stBuildApp
  else if CompareText(ParamStr(1), paramBindings) = 0 then opts.task := stBuildBindings
  else if CompareText(ParamStr(1), paramShowPath) = 0 then
  begin
    opts.task := stNothing;
    vProject.ParseFile;
    ShowPath;
  end
  else
  begin
    opts.task := stNothing;
    Usage;
  end;
end;

end.

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
