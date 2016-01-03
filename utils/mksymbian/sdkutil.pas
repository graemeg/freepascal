<<<<<<< HEAD
<<<<<<< HEAD
{
sdkutil.pas

SDK utility methods

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
unit sdkutil;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils, registry, constants;

type

  { TSDKUtil }

  TSDKUtil = class(TObject)
  public
    SDKFolder, SDKPartialFolder, StrSDKVersion: string;
    SDKVersion: TSDKVersion;
    constructor Create;
    procedure LocateUIQ2SDK;
    procedure LocateUIQ3SDK;
  end;

var
  vSDKUtil: TSDKUtil;

implementation

uses projectparser;

{ TSDKUtil }

constructor TSDKUtil.Create;
begin
  StrSDKVersion := vProject.SDK + ' ' + vProject.SDKVersion;

  if StrSDKVersion = Str_UIQ21 then SDKVersion := sdkUIQ21
  else if StrSDKVersion = Str_UIQ3 then SDKVersion := sdkUIQ3;

  case SDKVersion of
   sdkUIQ21: LocateUIQ2SDK;
   sdkUIQ3:  LocateUIQ3SDK;
  end;
end;

procedure TSDKUtil.LocateUIQ2SDK;
begin
  SDKPartialFolder := '\Programas\UIQ21\';
  SDKFolder := 'C:' + SDKPartialFolder;
end;

procedure TSDKUtil.LocateUIQ3SDK;
var
  Reg: TRegistry;
  BufferStr: string;
begin
  Reg := TRegistry.Create;
 
  try
    Reg.RootKey := HKEY_LOCAL_MACHINE;
    if Reg.OpenKey('\SOFTWARE\Symbian\UIQ\SDK\UIQ3SDK', False) then
    begin
      BufferStr := Reg.ReadString('InstallPath');
      SDKFolder := IncludeTrailingBackslash(BufferStr);
      SDKPartialFolder := Copy(SDKFolder, 3, Length(SDKFolder) - 2);
    end
    else
    begin
      WriteLn('  ERROR: Could not locate the SDK, using default values');
      SDKPartialFolder := '\Symbian\UIQ3SDK\';
      SDKFolder := 'C:' + SDKPartialFolder;
    end;
  finally
    Reg.Free;
  end;
end;

end.

=======
=======
>>>>>>> origin/fixes_2_2
{
sdkutil.pas

SDK utility methods

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
unit sdkutil;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}

interface

uses
  Classes, SysUtils, registry, constants;

type

  { TSDKUtil }

  TSDKUtil = class(TObject)
  public
    SDKFolder, SDKPartialFolder, StrSDKVersion: string;
    SDKVersion: TSDKVersion;
    constructor Create;
    procedure LocateUIQ2SDK;
    procedure LocateUIQ3SDK;
  end;

var
  vSDKUtil: TSDKUtil;

implementation

uses projectparser;

{ TSDKUtil }

constructor TSDKUtil.Create;
begin
  StrSDKVersion := vProject.SDK + ' ' + vProject.SDKVersion;

  if StrSDKVersion = Str_UIQ21 then SDKVersion := sdkUIQ21
  else if StrSDKVersion = Str_UIQ3 then SDKVersion := sdkUIQ3;

  case SDKVersion of
   sdkUIQ21: LocateUIQ2SDK;
   sdkUIQ3:  LocateUIQ3SDK;
  end;
end;

procedure TSDKUtil.LocateUIQ2SDK;
begin
  SDKPartialFolder := '\Programas\UIQ21\';
  SDKFolder := 'C:' + SDKPartialFolder;
end;

procedure TSDKUtil.LocateUIQ3SDK;
var
  Reg: TRegistry;
  BufferStr: string;
begin
  Reg := TRegistry.Create;
 
  try
    Reg.RootKey := HKEY_LOCAL_MACHINE;
    if Reg.OpenKey('\SOFTWARE\Symbian\UIQ\SDK\UIQ3SDK', False) then
    begin
      BufferStr := Reg.ReadString('InstallPath');
      SDKFolder := IncludeTrailingBackslash(BufferStr);
      SDKPartialFolder := Copy(SDKFolder, 3, Length(SDKFolder) - 2);
    end
    else
    begin
      WriteLn('  ERROR: Could not locate the SDK, using default values');
      SDKPartialFolder := '\Symbian\UIQ3SDK\';
      SDKFolder := 'C:' + SDKPartialFolder;
    end;
  finally
    Reg.Free;
  end;
end;

end.

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
