{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2003, 2006, 2007, 2009-2013  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
type
  TPTCConsole = class(TPTCBaseConsole)
  private
    FConsole: IPTCConsole;
    FModes: array of IPTCMode;
    FOptionsQueue: array of string;
    FHackyOptionConsoleFlag: Boolean;
    FUseOpenGL: Boolean;

    function ConsoleCreate(AIndex: Integer): IPTCConsole;
    function ConsoleCreate(const AName: string): IPTCConsole;
    procedure Check;

    procedure AddOptionToOptionsQueue(const AOption: string);
    procedure ExecuteOptionsFromOptionsQueue;
    procedure ClearOptionsQueue;

    procedure PassOpenGLOptionsToInnerConsole;

    function GetOpenGL_Enabled: Boolean; override;
    procedure SetOpenGL_Enabled(AValue: Boolean); override;
  public
    constructor Create; override;
    destructor Destroy; override;
    procedure Configure(const AFile: string); override;
    function Option(const AOption: string): Boolean; override;
    function Modes: TPTCModeList; override;
    procedure Open(const ATitle: string; APages: Integer = 0); overload; override;
    procedure Open(const ATitle: string; AFormat: IPTCFormat;
                   APages: Integer = 0); overload; override;
    procedure Open(const ATitle: string; AWidth, AHeight: Integer;
                   AFormat: IPTCFormat; APages: Integer = 0); overload; override;
    procedure Open(const ATitle: string; AMode: IPTCMode;
                   APages: Integer = 0); overload; override;

    procedure Close; override;
    procedure InternalResize(AWidth, AHeight: Integer); override;
    procedure Flush; override;
    procedure Finish; override;
    procedure Update; override;
    procedure Update(AArea: IPTCArea); override;
    procedure Copy(ASurface: IPTCSurface); override;
    procedure Copy(ASurface: IPTCSurface;
                   ASource, ADestination: IPTCArea); override;
    function Lock: Pointer; override;
    procedure Unlock; override;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette); override;
    procedure Load(const APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea); override;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette); override;
    procedure Save(APixels: Pointer;
                   AWidth, AHeight, APitch: Integer;
                   AFormat: IPTCFormat;
                   APalette: IPTCPalette;
                   ASource, ADestination: IPTCArea); override;
    procedure Clear; override;
    procedure Clear(AColor: IPTCColor); override;
    procedure Clear(AColor: IPTCColor;
                    AArea: IPTCArea); override;
    procedure Palette(APalette: IPTCPalette); override;
    function Palette: IPTCPalette; override;
    procedure Clip(AArea: IPTCArea); override;
    function GetWidth: Integer; override;
    function GetHeight: Integer; override;
    function GetPitch: Integer; override;
    function GetPages: Integer; override;
    function GetArea: IPTCArea; override;
    function Clip: IPTCArea; override;
    function GetFormat: IPTCFormat; override;
    function GetName: string; override;
    function GetTitle: string; override;
    function GetInformation: string; override;
    function NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean; override;
    function PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent; override;
    procedure OpenGL_SwapBuffers; override;
    procedure OpenGL_SetSwapInterval(AInterval: Integer); override;
    function OpenGL_GetSwapInterval: Integer; override;
  end;

class function TPTCConsoleFactory.CreateNew: IPTCConsole;
begin
  Result := TPTCConsole.Create;
end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
 {$IFDEF GO32V2}
  ConsoleTypesNumber = 4;
 {$ENDIF GO32V2}
 {$IF defined(Win32) OR defined(Win64)}
  ConsoleTypesNumber = 2;
 {$ENDIF defined(Win32) OR defined(Win64)}
 {$IFDEF WinCE}
  ConsoleTypesNumber = 2;
 {$ENDIF WinCE}
 {$IFDEF UNIX}
  ConsoleTypesNumber = 1;
 {$ENDIF UNIX}
  ConsoleTypes: array [0..ConsoleTypesNumber - 1] of
    record
      ConsoleClass: class of TPTCBaseConsole;
      Names: array [1..2] of string;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      OpenGL: Boolean;
    end =
  (
  {$IFDEF GO32V2}
   (ConsoleClass: TVESAConsole;      Names: ('VESA', '');        OpenGL: False),
   (ConsoleClass: TVGAConsole;       Names: ('VGA', 'Fakemode'); OpenGL: False),
   (ConsoleClass: TCGAConsole;       Names: ('CGA', '');         OpenGL: False),
   (ConsoleClass: TTEXTFX2Console;   Names: ('TEXTFX2', 'Text'); OpenGL: False)
  {$ENDIF GO32V2}

  {$IF defined(Win32) OR defined(Win64)}
   (ConsoleClass: TDirectXConsole;   Names: ('DirectX', '');     OpenGL: False),
   (ConsoleClass: TGDIConsole;       Names: ('GDI', '');         OpenGL: True)
  {$ENDIF defined(Win32) OR defined(Win64)}

  {$IFDEF WinCE}
   (ConsoleClass: TWinCEGAPIConsole; Names: ('GAPI', '');        OpenGL: False),
   (ConsoleClass: TWinCEGDIConsole;  Names: ('GDI', '');         OpenGL: False)
  {$ENDIF WinCE}

  {$IFDEF UNIX}
   (ConsoleClass: TX11Console;       Names: ('X11', '');         OpenGL: {$IFDEF ENABLE_X11_EXTENSION_GLX}True{$ELSE}False{$ENDIF})
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end =
  (
  {$IFDEF GO32V2}
   (ConsoleClass: TVESAConsole;      Names: ('VESA', '')),
   (ConsoleClass: TVGAConsole;       Names: ('VGA', 'Fakemode')),
   (ConsoleClass: TCGAConsole;       Names: ('CGA', '')),
   (ConsoleClass: TTEXTFX2Console;   Names: ('TEXTFX2', 'Text'))
  {$ENDIF GO32V2}

  {$IF defined(Win32) OR defined(Win64)}
   (ConsoleClass: TDirectXConsole;   Names: ('DirectX', '')),
   (ConsoleClass: TGDIConsole;       Names: ('GDI', ''))
  {$ENDIF defined(Win32) OR defined(Win64)}

  {$IFDEF WinCE}
   (ConsoleClass: TWinCEGAPIConsole; Names: ('GAPI', '')),
   (ConsoleClass: TWinCEGDIConsole;  Names: ('GDI', ''))
  {$ENDIF WinCE}

  {$IFDEF UNIX}
   (ConsoleClass: TX11Console;       Names: ('X11', ''))
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
  {$ENDIF UNIX}
  );

constructor TPTCConsole.Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF UNIX}
var
  s: AnsiString;
{$ENDIF UNIX}
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  I: Integer;
  {$IFDEF UNIX}
  s: AnsiString;
  {$ENDIF UNIX}
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
begin
  inherited Create;
  FConsole := nil;
  FHackyOptionConsoleFlag := False;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  FillChar(FModes, SizeOf(FModes), 0);
  for I := Low(FModes) to High(FModes) do
    FModes[I] := TPTCMode.Create;
>>>>>>> graemeg/cpstrnew
=======
  FillChar(FModes, SizeOf(FModes), 0);
  for I := Low(FModes) to High(FModes) do
    FModes[I] := TPTCMode.Create;
>>>>>>> graemeg/cpstrnew
=======
  FillChar(FModes, SizeOf(FModes), 0);
  for I := Low(FModes) to High(FModes) do
    FModes[I] := TPTCMode.Create;
>>>>>>> graemeg/cpstrnew
=======
  FillChar(FModes, SizeOf(FModes), 0);
  for I := Low(FModes) to High(FModes) do
    FModes[I] := TPTCMode.Create;
>>>>>>> origin/cpstrnew

  {$IFDEF UNIX}
    Configure('/usr/share/ptcpas/ptcpas.conf');
    s := fpgetenv('HOME');
    if s = '' then
      s := '/';
    if s[Length(s)] <> '/' then
      s := s + '/';
    s := s + '.ptcpas.conf';
    Configure(s);
  {$ENDIF UNIX}

  {$IFDEF Win32}
    Configure('ptcpas.cfg');
  {$ENDIF Win32}

  {$IFDEF GO32V2}
    Configure('ptcpas.cfg');
  {$ENDIF GO32V2}

  {$IFDEF WinCE}
  {todo: configure WinCE}
  {$ENDIF WinCE}
end;

destructor TPTCConsole.Destroy;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
begin
  Close;
  FConsole := nil;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
var
  I: Integer;
begin
  close;
  FConsole.Free;
  for I := Low(FModes) to High(FModes) do
    FModes[I].Free;
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
  inherited Destroy;
end;

procedure TPTCConsole.Configure(const AFile: string);
var
  F: TextFile;
  S: string;
begin
  AssignFile(F, AFile);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$push}{$I-}
  Reset(F);
  {$pop}
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> graemeg/cpstrnew
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> graemeg/cpstrnew
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> graemeg/cpstrnew
=======
  {$I-}
  Reset(F);
  {$I+}
>>>>>>> origin/cpstrnew
  if IOResult <> 0 then
    exit;
  while not EoF(F) do
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    {$push}{$I-}
    Readln(F, S);
    {$pop}
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> graemeg/cpstrnew
=======
    {$I-}
    Readln(F, S);
    {$I+}
>>>>>>> origin/cpstrnew
    if IOResult <> 0 then
      Break;
    Option(S);
  end;
  CloseFile(F);
end;

procedure TPTCConsole.AddOptionToOptionsQueue(const AOption: string);
begin
  SetLength(FOptionsQueue, Length(FOptionsQueue) + 1);
  FOptionsQueue[High(FOptionsQueue)] := AOption;
end;

procedure TPTCConsole.ExecuteOptionsFromOptionsQueue;
var
  I: Integer;
begin
  for I := Low(FOptionsQueue) to High(FOptionsQueue) do
    FConsole.Option(FOptionsQueue[I]);
end;

procedure TPTCConsole.ClearOptionsQueue;
begin
  SetLength(FOptionsQueue, 0);
end;

function TPTCConsole.Option(const AOption: String): Boolean;
begin
  if AOption = 'enable logging' then
  begin
    LOG_enabled := True;
    Result := True;
    exit;
  end;
  if AOption = 'disable logging' then
  begin
    LOG_enabled := False;
    Result := True;
    exit;
  end;

  if Assigned(FConsole) then
    Result := FConsole.Option(AOption)
  else
  begin
    FConsole := ConsoleCreate(AOption);
    if Assigned(FConsole) then
    begin
      FHackyOptionConsoleFlag := True;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      PassOpenGLOptionsToInnerConsole;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      ExecuteOptionsFromOptionsQueue;
{      ClearOptionsQueue;}
      Result := True;
    end
    else
    begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      { TODO: check if the option is supported by at least one console and return false otherwise... }

      AddOptionToOptionsQueue(AOption);
      Result := True;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      { TODO: check if the option is supported by at least one console... }
      if {OptionSupported}True then
      begin
        AddOptionToOptionsQueue(AOption);
        Result := True;
      end
      else
        Result := False;
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
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.Modes: TPTCModeList;
var
  _console: IPTCConsole;
  index, mode: Integer;
  local: Integer;
  _modes: TPTCModeList;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TPTCConsole.Modes: PPTCMode;
var
  _console: TPTCBaseConsole;
  index, mode: Integer;
  local: Integer;
  _modes: PPTCMode;
  tmp: TPTCMode;
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
begin
  if Assigned(FConsole) then
    Result := FConsole.Modes
  else
  begin
    _console := nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    SetLength(FModes, 0);
    index := -1;
    mode := 0;
    repeat
      Inc(index);
      try
        _console := ConsoleCreate(index);
      except
        on TPTCError do begin
          _console := nil;
          continue;
        end;
      end;
      if _console = nil then
        break;
      _modes := _console.modes;
      SetLength(FModes, Length(FModes) + Length(_modes));
      for local := Low(_modes) to High(_modes) do
      begin
        FModes[mode] := _modes[local];
        Inc(mode);
      end;
    until False;
    { todo: strip duplicate modes from list? }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    index := -1;
    mode := 0;
    try
      repeat
        Inc(index);
        try
          _console := ConsoleCreate(index);
        except
          on TPTCError do begin
            FreeAndNil(_console);
            Continue;
          end;
        end;
        if _console = nil then
          Break;
        _modes := _console.modes;
        local := 0;
        while _modes[local].valid do
        begin
          FModes[mode].Assign(_modes[local]);
          Inc(local);
          Inc(mode);
        end;
        FreeAndNil(_console);
      until False;
    finally
      _console.Free;
    end;
    { todo: strip duplicate modes from list? }
    tmp := TPTCMode.Create;
    try
      FModes[mode].Assign(tmp);
    finally
      tmp.Free;
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
    Result := FModes;
  end;
end;

procedure TPTCConsole.Open(const ATitle: string; APages: Integer);
var
  composite, tmp: TPTCError;
  index: Integer;
  success: Boolean;
begin
  if Assigned(FConsole) then
  begin
    try
      FConsole.open(ATitle, APages);
      exit;
    except
      on error: TPTCError do begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FConsole := nil;
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
        if FHackyOptionConsoleFlag then
        begin
          FHackyOptionConsoleFlag := False;
          raise TPTCError.Create('could not open console', error);
        end;
      end;
    end;
  end;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  try
    repeat
      Inc(index);
      try
        FConsole := ConsoleCreate(index);
        if FConsole = nil then
          break;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        PassOpenGLOptionsToInnerConsole;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        ExecuteOptionsFromOptionsQueue;
        FConsole.Open(ATitle, APages);
{        ClearOptionsQueue;}
        success := True;
        exit;
      except
        on error: TPTCError do begin
          tmp := TPTCError.Create(error.message, composite);
          try
            composite.Assign(tmp);
          finally
            tmp.Free;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          FConsole := nil;
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
          continue;
        end;
      end;
    until False;
    FConsole := nil;
    raise TPTCError.Create(composite);
  finally
    composite.Free;
    if not success then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FConsole := nil;
  end;
end;

procedure TPTCConsole.Open(const ATitle: string; AFormat: IPTCFormat;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      FreeAndNil(FConsole);
  end;
end;

procedure TPTCConsole.Open(const ATitle: string; const AFormat: TPTCFormat;
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
                           APages: Integer);
var
  composite, tmp: TPTCError;
  index: Integer;
  success: Boolean;
begin
  if Assigned(FConsole) then
  begin
    try
      FConsole.open(ATitle, AFormat, APages);
      exit;
    except
      on error: TPTCError do begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FConsole := nil;
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
        if FHackyOptionConsoleFlag then
        begin
          FHackyOptionConsoleFlag := False;
          raise TPTCError.Create('could not open console', error);
        end;
      end;
    end;
  end;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  try
    repeat
      Inc(index);
      try
        FConsole := ConsoleCreate(index);
        if FConsole = nil then
          break;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        PassOpenGLOptionsToInnerConsole;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        ExecuteOptionsFromOptionsQueue;
        FConsole.open(ATitle, AFormat, APages);
{        ClearOptionsQueue;}
        success := True;
        exit;
      except
        on error: TPTCError do begin
          tmp := TPTCError.Create(error.message, composite);
          try
            composite.Assign(tmp);
          finally
            tmp.Free;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          FConsole := nil;
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
          Continue;
        end;
      end;
    until False;
    FConsole := nil;
    raise TPTCError.Create(composite);
  finally
    composite.Free;
    if not success then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FConsole := nil;
=======
      FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
      FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
      FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
      FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
  end;
end;

procedure TPTCConsole.Open(const ATitle: string; AWidth, AHeight: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat; APages: Integer);
=======
                           const AFormat: TPTCFormat; APages: Integer);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; APages: Integer);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; APages: Integer);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat; APages: Integer);
>>>>>>> origin/cpstrnew
var
  composite, tmp: TPTCError;
  index: Integer;
  success: Boolean;
begin
  if Assigned(FConsole) then
  begin
    try
      FConsole.Open(ATitle, AWidth, AHeight, AFormat, APages);
      exit;
    except
      on error: TPTCError do begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FConsole := nil;
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
        if FHackyOptionConsoleFlag then
        begin
          FHackyOptionConsoleFlag := False;
          raise TPTCError.Create('could not open console', error);
        end;
      end;
    end;
  end;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  try
    repeat
      Inc(index);
      try
        FConsole := ConsoleCreate(index);
        if FConsole = nil then
          Break;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        PassOpenGLOptionsToInnerConsole;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        ExecuteOptionsFromOptionsQueue;
        FConsole.Open(ATitle, AWidth, AHeight, AFormat, APages);
{        ClearOptionsQueue;}
        success := True;
        exit;
      except
        on error: TPTCError do begin
          tmp := TPTCError.Create(error.message, composite);
          try
            composite.Assign(tmp);
          finally
            tmp.Free;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          FConsole := nil;
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
          Continue;
        end;
      end;
    until False;
    FConsole := nil;
    raise TPTCError.Create(composite);
  finally
    composite.Free;
    if not success then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FConsole := nil;
  end;
end;

procedure TPTCConsole.Open(const ATitle: string; AMode: IPTCMode;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      FreeAndNil(FConsole);
  end;
end;

procedure TPTCConsole.Open(const ATitle: string; const AMode: TPTCMode;
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
                           APages: Integer);
var
  composite, tmp: TPTCError;
  index: Integer;
  success: Boolean;
begin
  if Assigned(FConsole) then
  begin
    try
      FConsole.Open(ATitle, AMode, APages);
      exit;
    except
      on error: TPTCError do begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FConsole := nil;
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
        FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
        if FHackyOptionConsoleFlag then
        begin
          FHackyOptionConsoleFlag := False;
          raise TPTCError.Create('could not open console', error);
        end;
      end;
    end;
  end;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  try
    repeat
      Inc(index);
      try
        FConsole := ConsoleCreate(index);
        if FConsole = nil then
          Break;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        PassOpenGLOptionsToInnerConsole;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        ExecuteOptionsFromOptionsQueue;
        FConsole.Open(ATitle, AMode, APages);
{        ClearOptionsQueue;}
        success := True;
        exit;
      except
        on error: TPTCError do begin
          tmp := TPTCError.Create(error.message, composite);
          try
            composite.Assign(tmp);
          finally
            tmp.Free;
          end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          FConsole := nil;
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
          FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
          Continue;
        end;
      end;
    until False;
    FConsole := nil;
    raise TPTCError.Create(composite);
  finally
    composite.Free;
    if not success then
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      FConsole := nil;
=======
      FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
      FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
      FreeAndNil(FConsole);
>>>>>>> graemeg/cpstrnew
=======
      FreeAndNil(FConsole);
>>>>>>> origin/cpstrnew
  end;
end;

procedure TPTCConsole.Close;
begin
  if Assigned(FConsole) then
    FConsole.Close;
  FHackyOptionConsoleFlag := False;
end;

procedure TPTCConsole.Flush;
begin
  Check;
  FConsole.Flush;
end;

procedure TPTCConsole.Finish;
begin
  Check;
  FConsole.Finish;
end;

procedure TPTCConsole.Update;
begin
  Check;
  FConsole.Update;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCConsole.Update(AArea: IPTCArea);
=======
procedure TPTCConsole.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Update(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Update(const AArea: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Update(AArea);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCConsole.Copy(ASurface: IPTCSurface);
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Copy(ASurface);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCConsole.Copy(ASurface: IPTCSurface;
                           ASource, ADestination: IPTCArea);
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Copy(ASurface: TPTCBaseSurface;
                           const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Copy(ASurface, ASource, ADestination);
end;

function TPTCConsole.Lock: Pointer;
begin
  Check;
  Result := FConsole.Lock;
end;

procedure TPTCConsole.Unlock;
begin
  Check;
  FConsole.Unlock;
end;

procedure TPTCConsole.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette);
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette);
end;

procedure TPTCConsole.Load(const APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Load(APixels, AWidth, AHeight, APitch, AFormat, APalette,
                ASource, ADestination);
end;

procedure TPTCConsole.Save(Apixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette);
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Save(APixels, AWidth, AHeight, APitch, AFormat, APalette);
end;

procedure TPTCConsole.Save(APixels: Pointer;
                           AWidth, AHeight, APitch: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                           AFormat: IPTCFormat;
                           APalette: IPTCPalette;
                           ASource, ADestination: IPTCArea);
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
                           const AFormat: TPTCFormat;
                           const APalette: TPTCPalette;
                           const ASource, ADestination: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Save(APixels, AWidth, AHeight, APitch, AFormat, APalette,
                ASource, ADestination);
end;

procedure TPTCConsole.Clear;
begin
  Check;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FConsole.Clear;
end;

procedure TPTCConsole.Clear(AColor: IPTCColor);
begin
  Check;
  FConsole.Clear(AColor);
end;

procedure TPTCConsole.Clear(AColor: IPTCColor;
                            AArea: IPTCArea);
begin
  Check;
  FConsole.Clear(AColor, AArea);
end;

procedure TPTCConsole.Palette(APalette: IPTCPalette);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FConsole.clear;
end;

procedure TPTCConsole.Clear(const AColor: TPTCColor);
begin
  Check;
  FConsole.clear(AColor);
end;

procedure TPTCConsole.Clear(const AColor: TPTCColor;
                           const AArea: TPTCArea);
begin
  Check;
  FConsole.clear(AColor, AArea);
end;

procedure TPTCConsole.Palette(const APalette: TPTCPalette);
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
begin
  Check;
  FConsole.Palette(APalette);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.Palette: IPTCPalette;
=======
function TPTCConsole.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.Palette: TPTCPalette;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.Palette: TPTCPalette;
>>>>>>> origin/cpstrnew
begin
  Check;
  Result := FConsole.Palette;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TPTCConsole.Clip(AArea: IPTCArea);
=======
procedure TPTCConsole.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Clip(const AArea: TPTCArea);
>>>>>>> graemeg/cpstrnew
=======
procedure TPTCConsole.Clip(const AArea: TPTCArea);
>>>>>>> origin/cpstrnew
begin
  Check;
  FConsole.Clip(AArea);
end;

function TPTCConsole.GetWidth: Integer;
begin
  Check;
  Result := FConsole.GetWidth;
end;

function TPTCConsole.GetHeight: Integer;
begin
  Check;
  Result := FConsole.GetHeight;
end;

function TPTCConsole.GetPitch: Integer;
begin
  Check;
  Result := FConsole.GetPitch;
end;

function TPTCConsole.GetPages: Integer;
begin
  Check;
  Result := FConsole.GetPages;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.GetArea: IPTCArea;
=======
function TPTCConsole.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.GetArea: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.GetArea: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  Check;
  Result := FConsole.GetArea;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.Clip: IPTCArea;
=======
function TPTCConsole.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.Clip: TPTCArea;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.Clip: TPTCArea;
>>>>>>> origin/cpstrnew
begin
  Check;
  Result := FConsole.Clip;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.GetFormat: IPTCFormat;
=======
function TPTCConsole.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.GetFormat: TPTCFormat;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.GetFormat: TPTCFormat;
>>>>>>> origin/cpstrnew
begin
  Check;
  Result := FConsole.GetFormat;
end;

function TPTCConsole.GetName: string;
begin
  Result := '';
  if Assigned(FConsole) then
    Result := FConsole.GetName
  else
{$IFDEF GO32V2}
    Result := 'DOS';
{$ENDIF GO32V2}
{$IFDEF WIN32}
    Result := 'Win32';
{$ENDIF WIN32}
{$IFDEF WIN64}
    Result := 'Win64';
{$ENDIF WIN64}
{$IFDEF LINUX}
    Result := 'Linux';
{$ENDIF LINUX}
end;

function TPTCConsole.GetTitle: string;
begin
  Check;
  Result := FConsole.GetTitle;
end;

function TPTCConsole.GetInformation: string;
begin
  Check;
  Result := FConsole.GetInformation;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.NextEvent(out AEvent: IPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
=======
function TPTCConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.NextEvent(var AEvent: TPTCEvent; AWait: Boolean; const AEventMask: TPTCEventMask): Boolean;
>>>>>>> origin/cpstrnew
begin
  Check;
  Result := FConsole.NextEvent(AEvent, AWait, AEventMask);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): IPTCEvent;
=======
function TPTCConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.PeekEvent(AWait: Boolean; const AEventMask: TPTCEventMask): TPTCEvent;
>>>>>>> origin/cpstrnew
begin
  Check;
  Result := FConsole.PeekEvent(AWait, AEventMask);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.ConsoleCreate(AIndex: Integer): IPTCConsole;
var
  ResultObj: TPTCBaseConsole;
begin
  Result := nil;
  if (AIndex >= Low(ConsoleTypes)) and (AIndex <= High(ConsoleTypes)) then
  begin
    if OpenGL_Enabled and not ConsoleTypes[AIndex].OpenGL then
      raise TPTCError.Create('Console does not support OpenGL');

    ResultObj := ConsoleTypes[AIndex].ConsoleClass.Create;
    Result := ResultObj;
    ResultObj.FOpenGLAttributes := FOpenGLAttributes;
  end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TPTCConsole.ConsoleCreate(AIndex: Integer): TPTCBaseConsole;
begin
  Result := nil;
  if (AIndex >= Low(ConsoleTypes)) and (AIndex <= High(ConsoleTypes)) then
    Result := ConsoleTypes[AIndex].ConsoleClass.Create;
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

  if Result <> nil then
    Result.KeyReleaseEnabled := KeyReleaseEnabled;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPTCConsole.ConsoleCreate(const AName: string): IPTCConsole;
=======
function TPTCConsole.ConsoleCreate(const AName: string): TPTCBaseConsole;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.ConsoleCreate(const AName: string): TPTCBaseConsole;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.ConsoleCreate(const AName: string): TPTCBaseConsole;
>>>>>>> graemeg/cpstrnew
=======
function TPTCConsole.ConsoleCreate(const AName: string): TPTCBaseConsole;
>>>>>>> origin/cpstrnew
var
  I, J: Integer;
begin
  Result := nil;

  if AName = '' then
    exit;

  for I := Low(ConsoleTypes) to High(ConsoleTypes) do
    for J := Low(ConsoleTypes[I].Names) to High(ConsoleTypes[I].Names) do
      if AName = ConsoleTypes[I].Names[J] then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        Result := ConsoleCreate(I);
        exit;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        Result := ConsoleTypes[I].ConsoleClass.Create;

        if Result <> nil then
        begin
          Result.KeyReleaseEnabled := KeyReleaseEnabled;
          exit;
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
      end;
end;

procedure TPTCConsole.Check;
begin
  if FConsole = nil then
    raise TPTCError.Create('console is not open (core)');
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

procedure TPTCConsole.PassOpenGLOptionsToInnerConsole;
begin
  FConsole.OpenGL_Enabled := FUseOpenGL;
end;

function TPTCConsole.GetOpenGL_Enabled: Boolean;
begin
  if FConsole <> nil then
    Result := FConsole.OpenGL_Enabled
  else
    Result := FUseOpenGL;
end;

procedure TPTCConsole.SetOpenGL_Enabled(AValue: Boolean);
begin
  if FConsole <> nil then
    FConsole.OpenGL_Enabled := AValue
  else
    FUseOpenGL := AValue;
end;

procedure TPTCConsole.OpenGL_SwapBuffers;
begin
  Check;
  FConsole.OpenGL_SwapBuffers;
end;

procedure TPTCConsole.OpenGL_SetSwapInterval(AInterval: Integer);
begin
  Check;
  FConsole.OpenGL_SetSwapInterval(AInterval);
end;

function TPTCConsole.OpenGL_GetSwapInterval: Integer;
begin
  Check;
  Result := FConsole.OpenGL_GetSwapInterval;
end;

procedure TPTCConsole.InternalResize(AWidth, AHeight: Integer);
begin
  Check;
  FConsole.InternalResize(AWidth, AHeight);
end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
