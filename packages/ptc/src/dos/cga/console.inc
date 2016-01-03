{$MACRO ON}

{$DEFINE DEFAULT_WIDTH:=320}
{$DEFINE DEFAULT_HEIGHT:=200}
{$DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF)}

Constructor CGAConsole.Create;

Var
  I : Integer;

Begin
{  m_160x100buffer := Nil;}
  m_primary := Nil;
  m_keyboard := Nil;
  m_copy := Nil;
  m_default_format := Nil;
  m_open := False;
  m_locked := False;
  FillChar(m_modes, SizeOf(m_modes), 0);
  m_title[0] := #0;
  m_information[0] := #0;
  m_default_width := DEFAULT_WIDTH;
  m_default_height := DEFAULT_HEIGHT;
  m_default_format := DEFAULT_FORMAT;


  For I := 0 To 255 Do
    m_modes[I] := TPTCMode.Create;

  calcpal := @calcpal_colorbase;
  use_charset := @charset_b7asc;
  build_colormap(0);
  m_copy := TPTCCopy.Create;
  configure('ptc.cfg');
End;

Destructor CGAConsole.Destroy;

Var
  I : Integer;

Begin
  close;
  For I := 0 To 255 Do
    If m_modes[I] <> Nil Then
      m_modes[I].Destroy;
  If m_keyboard <> Nil Then
    m_keyboard.Destroy;
  If m_copy <> Nil Then
    m_copy.Destroy;
  If m_default_format <> Nil Then
    m_default_format.Destroy;
  Inherited Destroy;
End;

Procedure CGAConsole.configure(Const _file : String);

Var
  F : Text;
  S : String;

Begin
  ASSign(F, _file);
  Try
    Reset(F);
  Except
    Exit;
  End;
  Try
    While Not EoF(F) Do
    Begin
      Readln(F, S);
      option(S);
    End;
  Finally
    CloseFile(F);
  End;
End;

Function CGAConsole.option(Const _option : String) : Boolean;

Begin
  {...}
  option := m_copy.option(_option);
End;

Function CGAConsole.modes : PPTCMode;

Begin
  {todo...}
  modes := @m_modes;
End;

Procedure CGAConsole.open(Const _title : String; _pages : Integer); Overload;

Begin
  open(_title, m_default_format, _pages);
End;

Procedure CGAConsole.open(Const _title : String; Const _format : TPTCFormat;
                           _pages : Integer); Overload;

Begin
  open(_title, m_default_width, m_default_height, _format, _pages);
End;

Procedure CGAConsole.open(Const _title : String; _width, _height : Integer;
                           Const _format : TPTCFormat; _pages : Integer); Overload;

Var
  m : TPTCMode;

Begin
  m := TPTCMode.Create(_width, _height, _format);
  open(_title, m, _pages);
  m.Destroy;
End;

Procedure CGAConsole.open(Const _title : String; Const _mode : TPTCMode;
                           _pages : Integer); Overload;

Var
  _width, _height : Integer;
  _format : TPTCFormat;

Begin
  If Not _mode.valid Then
    Raise TPTCError.Create('invalid mode');
  _width := _mode.width;
  _height := _mode.height;
  _format := _mode.format;
  internal_pre_open_setup(_title);
  internal_open_fullscreen_start;
  internal_open_fullscreen(_width, _height, _format);
  internal_open_fullscreen_finish(_pages);
  internal_post_open_setup;
End;

Procedure CGAConsole.close;

Begin
  If m_open Then
  Begin
    If m_locked Then
      Raise TPTCError.Create('console is still locked');
    {flush all key presses}
    While KeyPressed Do ReadKey;
    internal_close;
    m_open := False;
  End;
End;

Procedure CGAConsole.flush;

Begin
  check_open;
  check_unlocked;
End;

Procedure CGAConsole.finish;

Begin
  check_open;
  check_unlocked;
End;

Procedure CGAConsole.update;

Var
  framebuffer : PByte;

Begin
  check_open;
  check_unlocked;
  framebuffer := m_primary.lock;
{  vrc;}
  CGADump(framebuffer);
  m_primary.unlock;
End;

Procedure CGAConsole.update(Const _area : TPTCArea);

Begin
  update;
End;

Procedure CGAConsole.internal_ReadKey(k : TPTCKey);

Begin
  check_open;
  m_keyboard.internal_ReadKey(k);
End;

Function CGAConsole.internal_PeekKey(k : TPTCKey) : Boolean;

Begin
  check_open;
  Result := m_keyboard.internal_PeekKey(k);
End;

Procedure CGAConsole.copy(Var surface : TPTCBaseSurface);

Var
  pixels : Pointer;

Begin
  check_open;
  check_unlocked;
  pixels := lock;
  Try
    surface.load(pixels, width, height, pitch, format, palette);
    unlock;
  Except
    On error : TPTCError Do
    Begin
      unlock;
      Raise TPTCError.Create('failed to copy console to surface', error);
    End;
  End;
End;

Procedure CGAConsole.copy(Var surface : TPTCBaseSurface;
                           Const source, destination : TPTCArea);

Begin
End;

Function CGAConsole.lock : Pointer;

Var
  pixels : Pointer;

Begin
  check_open;
  If m_locked Then
    Raise TPTCError.Create('console is already locked');
  pixels := m_primary.lock;
  m_locked := True;
  lock := pixels;
End;

Procedure CGAConsole.unlock;

Begin
  check_open;
  If Not m_locked Then
    Raise TPTCError.Create('console is not locked');
  m_primary.unlock;
  m_locked := False;
End;

Procedure CGAConsole.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;
  c, a : TPTCArea;

Begin
  c := clip; a := area;
  If (c.left = a.left) And
     (c.top = a.top) And
     (c.right = a.right) And
     (c.bottom = a.bottom) Then
  Begin
    check_open;
    check_unlocked;
    console_pixels := lock;
    Try
      m_copy.request(_format, format);
      m_copy.palette(_palette, palette);
      m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
                  width, height, pitch);
      unlock;
    Except
      On error : TPTCError Do
      Begin
        unlock;
        Raise TPTCError.Create('failed to load pixels to console', error);
      End;
    End;
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    load(pixels, _width, _height, _pitch, _format, _palette, Area_, area);
    Area_.Destroy;
  End;
End;

Procedure CGAConsole.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);

Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
  check_open;
  check_unlocked;
  console_pixels := lock;
  clipped_source := TPTCArea.Create;
  clipped_destination := TPTCArea.Create;
  Try
    tmp := TPTCArea.Create(0, 0, _width, _height);
    TPTCClipper.clip(source, tmp, clipped_source, destination, clip, clipped_destination);
    tmp.Destroy;
    m_copy.request(_format, format);
    m_copy.palette(_palette, palette);
    m_copy.copy(pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, _pitch,
                console_pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, pitch);
    unlock;
  Except
    On error:TPTCError Do
    Begin
      clipped_source.Destroy;
      clipped_destination.Destroy;
      unlock;
      Raise TPTCError.Create('failed to load pixels to console area', error);
    End;
  End;
  clipped_source.Destroy;
  clipped_destination.Destroy;
End;

Procedure CGAConsole.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;
  c, a : TPTCArea;

Begin
  c := clip; a := area;
  If (c.left = a.left) And
     (c.top = a.top) And
     (c.right = a.right) And
     (c.bottom = a.bottom) Then
  Begin
    check_open;
    check_unlocked;
    console_pixels := lock;
    Try
      m_copy.request(format, _format);
      m_copy.palette(palette, _palette);
      m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
                  _width, _height, _pitch);
      unlock;
    Except
      On error : TPTCError Do
      Begin
        unlock;
        Raise TPTCError.Create('failed to save console pixels', error);
      End;
    End;
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    save(pixels, _width, _height, _pitch, _format, _palette, area, Area_);
    Area_.Destroy;
  End;
End;

Procedure CGAConsole.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette;
                           Const source, destination : TPTCArea);
Var
  console_pixels : Pointer;
  clipped_source, clipped_destination : TPTCArea;
  tmp : TPTCArea;

Begin
  check_open;
  check_unlocked;
  console_pixels := lock;
  clipped_source := TPTCArea.Create;
  clipped_destination := TPTCArea.Create;
  Try
    tmp := TPTCArea.Create(0, 0, _width, _height);
    TPTCClipper.clip(source, clip, clipped_source, destination, tmp, clipped_destination);
    tmp.Destroy;
    m_copy.request(format, _format);
    m_copy.palette(palette, _palette);
    m_copy.copy(console_pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, pitch,
                pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, _pitch);
    unlock;
  Except
    On error:TPTCError Do
    Begin
      clipped_source.Destroy;
      clipped_destination.Destroy;
      unlock;
      Raise TPTCError.Create('failed to save console area pixels', error);
    End;
  End;
  clipped_source.Destroy;
  clipped_destination.Destroy;
End;

Procedure CGAConsole.clear;

Begin
End;

Procedure CGAConsole.clear(Const color : TPTCColor);

Begin
End;

Procedure CGAConsole.clear(Const color : TPTCColor;
                           Const _area : TPTCArea);

Begin
End;

Procedure CGAConsole.palette(Const _palette : TPTCPalette);

Begin
  check_open;
  m_primary.palette(_palette);
End;

Function CGAConsole.palette : TPTCPalette;

Begin
  check_open;
  palette := m_primary.palette;
End;

Procedure CGAConsole.clip(Const _area : TPTCArea);

Begin
  check_open;
  m_primary.clip(_area);
End;

Function CGAConsole.width : Integer;

Begin
  check_open;
  width := m_primary.width;
End;

Function CGAConsole.height : Integer;

Begin
  check_open;
  height := m_primary.height;
End;

Function CGAConsole.pitch : Integer;

Begin
  check_open;
  pitch := m_primary.pitch;
End;

Function CGAConsole.pages : Integer;

Begin
  check_open;
  pages := 1;{m_primary.pages;}
End;

Function CGAConsole.area : TPTCArea;

Begin
  check_open;
  area := m_primary.area;
End;

Function CGAConsole.clip : TPTCArea;

Begin
  check_open;
  clip := m_primary.clip;
End;

Function CGAConsole.format : TPTCFormat;

Begin
  check_open;
  format := m_primary.format;
End;

Function CGAConsole.name : String;

Begin
End;

Function CGAConsole.title : String;

Begin
End;

Function CGAConsole.information : String;

Begin
End;

Procedure CGAConsole.internal_pre_open_setup(Const _title : String);

Begin

End;

Procedure CGAConsole.internal_open_fullscreen_start;

Var
  f : TPTCFormat;

Begin
  CGAPrecalc;
  f := TPTCFormat.Create(32, $FF0000, $00FF00, $0000FF);
  m_primary := TPTCSurface.Create(320, 200, f);
  f.Destroy;
{  set80x50;}
  CGA320;
End;

Procedure CGAConsole.internal_open_fullscreen(_width, _height : Integer; Const _format : TPTCFormat);

Begin
{  m_primary := TPTCSurface.Create(_width, _height, _format);}
End;

Procedure CGAConsole.internal_open_fullscreen_finish(_pages : Integer);

Begin
End;

Procedure CGAConsole.internal_post_open_setup;

Begin
  If m_keyboard <> Nil Then
    m_keyboard.Destroy;
  m_keyboard := TDosKeyboard.Create;
  { create win32 keyboard
  m_keyboard = new DosKeyboard();//m_window->handle(),m_window->thread(),false);}

  { temporary platform dependent information fudge }
  {sprintf(m_information,"dos version x.xx.x\nvesa version x.xx\nvesa driver name xxxxx\ndisplay driver vendor xxxxx\ncertified driver? x\n");}

  { set open flag }
  m_open := True;
End;

Procedure CGAConsole.internal_reset;

Begin
  If m_primary <> Nil Then
    m_primary.Destroy;
{  m_keyboard.Destroy;}
  m_primary := Nil;
{  m_keyboard := Nil;}
End;

Procedure CGAConsole.internal_close;

Begin
  If m_primary <> Nil Then
    m_primary.Destroy;
  m_primary := Nil;
{  If m_160x100buffer <> Nil Then
    m_160x100buffer.Destroy;
  m_160x100buffer := Nil;}
  CGAText;
{  m_keyboard.Destroy;
  m_keyboard := Nil;}
End;

Procedure CGAConsole.check_open;

Begin
  {$IFDEF DEBUG}
    If Not m_open Then
      Raise TPTCError.Create('console is not open');
  {$ENDIF}
End;

Procedure CGAConsole.check_unlocked;

Begin
  {$IFDEF DEBUG}
    If m_locked Then
      Raise TPTCError.Create('console is not unlocked');
  {$ENDIF}
End;
