<<<<<<< HEAD
<<<<<<< HEAD
{
mksymbian.pas

Main program file

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
program mksymbian;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

{$apptype console}

uses
  Classes, SysUtils,
  cmdline, constants, cfgfile, sdkutil, compiler, projectparser;

var
  opts: TMkSymbianOptions;
begin

  vProject := TProject.Create;
  vSDKUtil := TSDKUtil.Create;
  vCmdLine := TCmdLine.Create;
  vCompiler := TCompiler.Create;

  try
    vCmdLine.ParseCmdLineOptions(opts);
    
    vCompiler.opts := opts;
    vProject.opts := opts;

    case opts.task of

      stBuildApp:
      begin
        vProject.ParseFile;
        
        { compilation }
        
        if CompareText(vProject.Language, STR_OPT_Cpp) = 0 then
         vCompiler.MakeBuildCpp
        else
         vCompiler.MakeBuildPascal;

        if vSDKUtil.SDKVersion = sdkUIQ3 then
        begin
          { Main resource file }
        
          vCompiler.BuildResource(vProject.MainResource);

          vCompiler.InstallResource(vProject.MainResource);

          { Registration resource file }
        
          vCompiler.BuildResource(vProject.RegResource);

          vCompiler.RegisterInEmulator;
        end;
      end;

      stBuildBindings:
      begin
        vProject.ParseFile;

        vCompiler.MakeBuildBindings;
      end;
    end;
    
  finally
    vCmdLine.Free;
    vSDKUtil.Free;
    vCompiler.Free;
    vProject.Free;
  end;
end.

=======
=======
>>>>>>> origin/fixes_2_2
{
mksymbian.pas

Main program file

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
program mksymbian;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

{$apptype console}

uses
  Classes, SysUtils,
  cmdline, constants, cfgfile, sdkutil, compiler, projectparser;

var
  opts: TMkSymbianOptions;
begin

  vProject := TProject.Create;
  vSDKUtil := TSDKUtil.Create;
  vCmdLine := TCmdLine.Create;
  vCompiler := TCompiler.Create;

  try
    vCmdLine.ParseCmdLineOptions(opts);
    
    vCompiler.opts := opts;
    vProject.opts := opts;

    case opts.task of

      stBuildApp:
      begin
        vProject.ParseFile;
        
        { compilation }
        
        if CompareText(vProject.Language, STR_OPT_Cpp) = 0 then
         vCompiler.MakeBuildCpp
        else
         vCompiler.MakeBuildPascal;

        if vSDKUtil.SDKVersion = sdkUIQ3 then
        begin
          { Main resource file }
        
          vCompiler.BuildResource(vProject.MainResource);

          vCompiler.InstallResource(vProject.MainResource);

          { Registration resource file }
        
          vCompiler.BuildResource(vProject.RegResource);

          vCompiler.RegisterInEmulator;
        end;
      end;

      stBuildBindings:
      begin
        vProject.ParseFile;

        vCompiler.MakeBuildBindings;
      end;
    end;
    
  finally
    vCmdLine.Free;
    vSDKUtil.Free;
    vCompiler.Free;
    vProject.Free;
  end;
end.

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
