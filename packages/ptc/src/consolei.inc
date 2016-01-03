{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

Const
 {$IFDEF GO32V2}
  ConsoleTypesNumber = 4;
 {$ENDIF GO32V2}
 {$IFDEF Win32}
  ConsoleTypesNumber = 2;
 {$ENDIF Win32}
 {$IFDEF WinCE}
  ConsoleTypesNumber = 2;
 {$ENDIF WinCE}
 {$IFDEF UNIX}
  ConsoleTypesNumber = 1;
 {$ENDIF UNIX}
  ConsoleTypes : Array[0..ConsoleTypesNumber - 1] Of
    Record
      ConsoleClass : Class Of TPTCBaseConsole;
      Names : Array[1..2] Of String;
    End =
  (
  {$IFDEF GO32V2}
   (ConsoleClass : TVESAConsole;      Names : ('VESA', '')),
   (ConsoleClass : TVGAConsole;       Names : ('VGA', 'Fakemode')),
   (ConsoleClass : TCGAConsole;       Names : ('CGA', '')),
   (ConsoleClass : TTEXTFX2Console;   Names : ('TEXTFX2', 'Text'))
  {$ENDIF GO32V2}

  {$IFDEF Win32}
   (ConsoleClass : TDirectXConsole;   Names : ('DirectX', '')),
   (ConsoleClass : TGDIConsole;       Names : ('GDI', ''))
  {$ENDIF Win32}

  {$IFDEF WinCE}
   (ConsoleClass : TWinCEGAPIConsole; Names : ('GAPI', '')),
   (ConsoleClass : TWinCEGDIConsole;  Names : ('GDI', ''))
  {$ENDIF WinCE}

  {$IFDEF UNIX}
   (ConsoleClass : TX11Console;       Names : ('X11', ''))
  {$ENDIF UNIX}
  );

Constructor TPTCConsole.Create;

Var
  I : Integer;
  {$IFDEF UNIX}
  s : AnsiString;
  {$ENDIF UNIX}

Begin
  Inherited Create;
  console := Nil;
  hacky_option_console_flag := False;
  FillChar(m_modes, SizeOf(m_modes), 0);
  For I := Low(m_modes) To High(m_modes) Do
    m_modes[I] := TPTCMode.Create;

  {$IFDEF UNIX}
    configure('/usr/share/ptcpas/ptcpas.conf');
    s := fpgetenv('HOME');
    If s = '' Then
      s := '/';
    If s[Length(s)] <> '/' Then
      s := s + '/';
    s := s + '.ptcpas.conf';
    configure(s);
  {$ENDIF UNIX}

  {$IFDEF Win32}
    configure('ptcpas.cfg');
  {$ENDIF Win32}

  {$IFDEF GO32V2}
    configure('ptcpas.cfg');
  {$ENDIF GO32V2}

  {$IFDEF WinCE}
  {todo: configure WinCE}
  {$ENDIF WinCE}
End;

Destructor TPTCConsole.Destroy;

Var
  I : Integer;

Begin
  close;
  console.Free;
  For I := Low(m_modes) To High(m_modes) Do
    m_modes[I].Free;
  Inherited Destroy;
End;

Procedure TPTCConsole.configure(Const _file : String);

Var
  F : Text;
  S : String;

Begin
  AssignFile(F, _file);
  {$I-}
  Reset(F);
  {$I+}
  If IOResult <> 0 Then
    Exit;
  While Not EoF(F) Do
  Begin
    {$I-}
    Readln(F, S);
    {$I+}
    If IOResult <> 0 Then
      Break;
    option(S);
  End;
  CloseFile(F);
End;

Function TPTCConsole.option(Const _option : String) : Boolean;

Begin
  If _option = 'enable logging' Then
  Begin
    LOG_enabled := True;
    option := True;
    Exit;
  End;
  If _option = 'disable logging' Then
  Begin
    LOG_enabled := False;
    option := True;
    Exit;
  End;

  If Assigned(console) Then
    option := console.option(_option)
  Else
  Begin
    console := ConsoleCreate(_option);
    If Assigned(console) Then
    Begin
      hacky_option_console_flag := True;
      option := True;
    End
    Else
      option := False;
  End;
End;

Function TPTCConsole.modes : PPTCMode;

Var
  _console : TPTCBaseConsole;
  index, mode : Integer;
  local : Integer;
  _modes : PPTCMode;
  tmp : TPTCMode;

Begin
  If Assigned(console) Then
    modes := console.modes
  Else
  Begin
    _console := Nil;
    index := -1;
    mode := 0;
    Try
      Repeat
        Inc(index);
        Try
          _console := ConsoleCreate(index);
        Except
          On TPTCError Do Begin
            FreeAndNil(_console);
            Continue;
          End;
        End;
        If _console = Nil Then
          Break;
        _modes := _console.modes;
        local := 0;
        While _modes[local].valid Do
        Begin
          m_modes[mode].Assign(_modes[local]);
          Inc(local);
          Inc(mode);
        End;
        FreeAndNil(_console);
      Until False;
    Finally
      _console.Free;
    End;
    { todo: strip duplicate modes from list? }
    tmp := TPTCMode.Create;
    Try
      m_modes[mode].Assign(tmp);
    Finally
      tmp.Free;
    End;
    modes := m_modes;
  End;
End;

Procedure TPTCConsole.open(Const _title : String; _pages : Integer);{ Overload;}

Var
  composite, tmp : TPTCError;
  index : Integer;
  success : Boolean;

Begin
  If Assigned(console) Then
  Begin
    Try
      console.open(_title, _pages);
      Exit;
    Except
      On error : TPTCError Do Begin
        FreeAndNil(console);
        If hacky_option_console_flag Then
        Begin
          hacky_option_console_flag := False;
          Raise TPTCError.Create('could not open console', error);
        End;
      End;
    End;
  End;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  Try
    Repeat
      Inc(index);
      Try
        console := ConsoleCreate(index);
        If console = Nil Then
          Break;
        console.open(_title, _pages);
        success := True;
        Exit;
      Except
        On error : TPTCError Do Begin
          tmp := TPTCError.Create(error.message, composite);
          Try
            composite.Assign(tmp);
          Finally
            tmp.Free;
          End;
          FreeAndNil(console);
          Continue;
        End;
      End;
    Until False;
    console := Nil;
    Raise TPTCError.Create(composite);
  Finally
    composite.Free;
    If Not success Then
      FreeAndNil(console);
  End;
End;

Procedure TPTCConsole.open(Const _title : String; Const _format : TPTCFormat;
                           _pages : Integer);{ Overload;}

Var
  composite, tmp : TPTCError;
  index : Integer;
  success : Boolean;

Begin
  If Assigned(console) Then
  Begin
    Try
      console.open(_title, _format, _pages);
      Exit;
    Except
      On error : TPTCError Do Begin
        FreeAndNil(console);
        If hacky_option_console_flag Then
        Begin
          hacky_option_console_flag := False;
          Raise TPTCError.Create('could not open console', error);
        End;
      End;
    End;
  End;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  Try
    Repeat
      Inc(index);
      Try
        console := ConsoleCreate(index);
        If console = Nil Then
          Break;
        console.open(_title, _format, _pages);
        success := True;
        Exit;
      Except
        On error : TPTCError Do Begin
          tmp := TPTCError.Create(error.message, composite);
          Try
            composite.Assign(tmp);
          Finally
            tmp.Free;
          End;
          FreeAndNil(console);
          Continue;
        End;
      End;
    Until False;
    console := Nil;
    Raise TPTCError.Create(composite);
  Finally
    composite.Free;
    If Not success Then
      FreeAndNil(console);
  End;
End;

Procedure TPTCConsole.open(Const _title : String; _width, _height : Integer;
                           Const _format : TPTCFormat; _pages : Integer);{ Overload;}

Var
  composite, tmp : TPTCError;
  index : Integer;
  success : Boolean;

Begin
  If Assigned(console) Then
  Begin
    Try
      console.open(_title, _width, _height, _format, _pages);
      Exit;
    Except
      On error : TPTCError Do Begin
        FreeAndNil(console);
        If hacky_option_console_flag Then
        Begin
          hacky_option_console_flag := False;
          Raise TPTCError.Create('could not open console', error);
        End;
      End;
    End;
  End;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  Try
    Repeat
      Inc(index);
      Try
        console := ConsoleCreate(index);
        If console = Nil Then
          Break;
        console.open(_title, _width, _height, _format, _pages);
        success := True;
        Exit;
      Except
        On error : TPTCError Do Begin
          tmp := TPTCError.Create(error.message, composite);
          Try
            composite.Assign(tmp);
          Finally
            tmp.Free;
          End;
          FreeAndNil(console);
          Continue;
        End;
      End;
    Until False;
    console := Nil;
    Raise TPTCError.Create(composite);
  Finally
    composite.Free;
    If Not success Then
      FreeAndNil(console);
  End;
End;

Procedure TPTCConsole.open(Const _title : String; Const _mode : TPTCMode;
                           _pages : Integer);{ Overload;}

Var
  composite, tmp : TPTCError;
  index : Integer;
  success : Boolean;

Begin
  If Assigned(console) Then
  Begin
    Try
      console.open(_title, _mode, _pages);
      Exit;
    Except
      On error : TPTCError Do Begin
        FreeAndNil(console);
        If hacky_option_console_flag Then
        Begin
          hacky_option_console_flag := False;
          Raise TPTCError.Create('could not open console', error);
        End;
      End;
    End;
  End;
  index := -1;
  composite := TPTCError.Create;
  success := False;
  Try
    Repeat
      Inc(index);
      Try
        console := ConsoleCreate(index);
        If console = Nil Then
          Break;
        console.open(_title, _mode, _pages);
        success := True;
        Exit;
      Except
        On error : TPTCError Do Begin
          tmp := TPTCError.Create(error.message, composite);
          Try
            composite.Assign(tmp);
          Finally
            tmp.Free;
          End;
          FreeAndNil(console);
          Continue;
        End;
      End;
    Until False;
    console := Nil;
    Raise TPTCError.Create(composite);
  Finally
    composite.Free;
    If Not success Then
      FreeAndNil(console);
  End;
End;

Procedure TPTCConsole.close;

Begin
  If Assigned(console) Then
    console.close;
  hacky_option_console_flag := False;
End;

Procedure TPTCConsole.flush;

Begin
  check;
  console.flush;
End;

Procedure TPTCConsole.finish;

Begin
  check;
  console.finish;
End;

Procedure TPTCConsole.update;

Begin
  check;
  console.update;
End;

Procedure TPTCConsole.update(Const _area : TPTCArea);

Begin
  check;
  console.update(_area);
End;

Procedure TPTCConsole.copy(Var surface : TPTCBaseSurface);

Begin
  check;
  console.copy(surface);
End;

Procedure TPTCConsole.copy(Var surface : TPTCBaseSurface;
                           Const source, destination : TPTCArea);

Begin
  check;
  console.copy(surface, source, destination);
End;

Function TPTCConsole.lock : Pointer;

Begin
  check;
  lock := console.lock;
End;

Procedure TPTCConsole.unlock;

Begin
  check;
  console.unlock;
End;

Procedure TPTCConsole.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);

Begin
  check;
  console.load(pixels, _width, _height, _pitch, _format, _palette);
End;

Procedure TPTCConsole.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);

Begin
  check;
  console.load(pixels, _width, _height, _pitch, _format, _palette,
               source, destination);
End;

Procedure TPTCConsole.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);

Begin
  check;
  console.save(pixels, _width, _height, _pitch, _format, _palette);
End;

Procedure TPTCConsole.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);

Begin
  check;
  console.save(pixels, _width, _height, _pitch, _format, _palette,
               source, destination);
End;

Procedure TPTCConsole.clear;

Begin
  check;
  console.clear;
End;

Procedure TPTCConsole.clear(Const color : TPTCColor);

Begin
  check;
  console.clear(color);
End;

Procedure TPTCConsole.clear(Const color : TPTCColor;
                           Const _area : TPTCArea);

Begin
  check;
  console.clear(color, _area);
End;

Procedure TPTCConsole.palette(Const _palette : TPTCPalette);

Begin
  check;
  console.palette(_palette);
End;

Function TPTCConsole.Palette : TPTCPalette;

Begin
  check;
  Result := console.Palette;
End;

Procedure TPTCConsole.Clip(Const _area : TPTCArea);

Begin
  check;
  console.clip(_area);
End;

Function TPTCConsole.GetWidth : Integer;

Begin
  check;
  Result := console.GetWidth;
End;

Function TPTCConsole.GetHeight : Integer;

Begin
  check;
  Result := console.GetHeight;
End;

Function TPTCConsole.GetPitch : Integer;

Begin
  check;
  Result := console.GetPitch;
End;

Function TPTCConsole.GetPages : Integer;

Begin
  check;
  Result := console.GetPages;
End;

Function TPTCConsole.GetArea : TPTCArea;

Begin
  check;
  Result := console.GetArea;
End;

Function TPTCConsole.Clip : TPTCArea;

Begin
  check;
  Result := console.Clip;
End;

Function TPTCConsole.GetFormat : TPTCFormat;

Begin
  check;
  Result := console.GetFormat;
End;

Function TPTCConsole.GetName : String;

Begin
  Result := '';
  If Assigned(console) Then
    Result := console.GetName
  Else
{$IFDEF GO32V2}
    Result := 'DOS';
{$ENDIF GO32V2}
{$IFDEF WIN32}
    Result := 'Win32';
{$ENDIF WIN32}
{$IFDEF LINUX}
    Result := 'Linux';
{$ENDIF LINUX}
End;

Function TPTCConsole.GetTitle : String;

Begin
  check;
  Result := console.GetTitle;
End;

Function TPTCConsole.GetInformation : String;

Begin
  check;
  Result := console.GetInformation;
End;

Function TPTCConsole.NextEvent(Var event : TPTCEvent; wait : Boolean; Const EventMask : TPTCEventMask) : Boolean;

Begin
  check;
  Result := console.NextEvent(event, wait, EventMask);
End;

Function TPTCConsole.PeekEvent(wait : Boolean; Const EventMask : TPTCEventMask) : TPTCEvent;

Begin
  check;
  Result := console.PeekEvent(wait, EventMask);
End;

Function TPTCConsole.ConsoleCreate(index : Integer) : TPTCBaseConsole;

Begin
  Result := Nil;
  If (index >= Low(ConsoleTypes)) And (index <= High(ConsoleTypes)) Then
    Result := ConsoleTypes[index].ConsoleClass.Create;

  If Result <> Nil Then
    Result.KeyReleaseEnabled := KeyReleaseEnabled;
End;

Function TPTCConsole.ConsoleCreate(Const AName : String) : TPTCBaseConsole;

Var
  I, J : Integer;

Begin
  Result := Nil;

  If AName = '' Then
    Exit;

  For I := Low(ConsoleTypes) To High(ConsoleTypes) Do
    For J := Low(ConsoleTypes[I].Names) To High(ConsoleTypes[I].Names) Do
      If AName = ConsoleTypes[I].Names[J] Then
      Begin
        Result := ConsoleTypes[I].ConsoleClass.Create;

        If Result <> Nil Then
        Begin
          Result.KeyReleaseEnabled := KeyReleaseEnabled;
          Exit;
        End;
      End;
End;

Procedure TPTCConsole.check;

Begin
  { $IFDEF DEBUG}
  If console = Nil Then
    Raise TPTCError.Create('console is not open (core)');
  { $ENDIF DEBUG}
End;
