{$MACRO ON}

{$DEFINE DEFAULT_WIDTH:=320}
{$DEFINE DEFAULT_HEIGHT:=200}
{$DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF)}

{$ASMMODE intel}

Constructor VGAConsole.Create;

Var
{  I, J : Integer;
  r, g, b, a : DWord;
  tmpbpp : Integer;}
  tmp : TPTCFormat;

Begin
  m_area := Nil;
  m_clip := Nil;
  m_keyboard := Nil;
  m_copy := Nil;
  m_palette := Nil;
  m_default_format := Nil;
  m_open := False;
  m_locked := False;
  m_title[0] := #0;
  m_information[0] := #0;
  m_default_width := DEFAULT_WIDTH;
  m_default_height := DEFAULT_HEIGHT;
  m_default_format := DEFAULT_FORMAT;

{  InitVESA;}
  m_primary := Nil;
{  m_modes[0].Create;}

  m_area := TPTCArea.Create;
  m_clip := TPTCArea.Create;
  m_copy := TPTCCopy.Create;
  m_palette := TPTCPalette.Create;

  tmp := TPTCFormat.Create(8);
  m_modes[0] := TPTCMode.Create(320, 200, tmp);
  tmp.Destroy;
  tmp := TPTCFormat.Create(8, $E0, $1C, $03);
  m_modes[1] := TPTCMode.Create(320, 200, tmp);
  tmp.Destroy;
  tmp := TPTCFormat.Create(16, $F800, $7E0, $1F);
  m_modes[2] := TPTCMode.Create(320, 200, tmp);
  tmp.Destroy;
  m_modes[3] := TPTCMode.Create;
  m_faketype := FAKEMODE2A;

  configure('ptc.cfg');
End;

Destructor VGAConsole.Destroy;

Begin
  close;
  internal_clear_mode_list;
  If m_keyboard <> Nil Then
    m_keyboard.Destroy;
  If m_copy <> Nil Then
    m_copy.Destroy;
  If m_default_format <> Nil Then
    m_default_format.Destroy;
  If m_palette <> Nil Then
    m_palette.Destroy;
  If m_clip <> Nil Then
    m_clip.Destroy;
  If m_area <> Nil Then
    m_area.Destroy;
  Inherited Destroy;
End;

Procedure VGAConsole.configure(Const _file : String);

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

Function VGAConsole.option(Const _option : String) : Boolean;

Begin
  {...}
  If (System.Copy(_option, 1, 8) = 'FAKEMODE') And (Length(_option) = 10) And
    (_option[9] >= '1') And (_option[9] <= '3') And
    (_option[10] >= 'A') And (_option[10] <= 'C') Then
  Begin
    Case _option[9] Of
      '1' : Case _option[10] Of
        'A' : m_faketype := FAKEMODE1A;
        'B' : m_faketype := FAKEMODE1B;
        'C' : m_faketype := FAKEMODE1C;
      End;
      '2' : Case _option[10] Of
        'A' : m_faketype := FAKEMODE2A;
        'B' : m_faketype := FAKEMODE2B;
        'C' : m_faketype := FAKEMODE2C;
      End;
      '3' : Case _option[10] Of
        'A' : m_faketype := FAKEMODE3A;
        'B' : m_faketype := FAKEMODE3B;
        'C' : m_faketype := FAKEMODE3C;
      End;
    End;
    option := True;
    Exit;
  End;
  option := m_copy.option(_option);
End;

Procedure VGAConsole.internal_clear_mode_list;

Var
  I : Integer;
  Done : Boolean;

Begin
  I := 0;
  Done := False;
  Repeat
    Done := Not m_modes[I].valid;
    m_modes[I].Destroy;
    Inc(I);
  Until Done;
End;

Function VGAConsole.modes : PPTCMode;

Begin
{  internal_clear_mode_list;}

  modes := m_modes;
End;

Procedure VGAConsole.open(Const _title : String; _pages : Integer); Overload;

Begin
  open(_title, m_default_format, _pages);
End;

Procedure VGAConsole.open(Const _title : String; Const _format : TPTCFormat;
                           _pages : Integer); Overload;

Begin
  open(_title, m_default_width, m_default_height, _format, _pages);
End;

Procedure VGAConsole.open(Const _title : String; _width, _height : Integer;
                           Const _format : TPTCFormat; _pages : Integer); Overload;

Var
  m : TPTCMode;

Begin
  m := TPTCMode.Create(_width, _height, _format);
  Try
    open(_title, m, _pages);
  Finally
    m.Destroy;
  End;
End;

Procedure VGAConsole.open(Const _title : String; Const _mode : TPTCMode;
                           _pages : Integer); Overload;

Var
{  _width, _height : Integer;
  _format : TPTCFormat;}
  I : Integer;
{  modefound : Integer;}
  modetype : Integer;

Begin
  If Not _mode.valid Then
    Raise TPTCError.Create('invalid mode');
  If _mode.format.indexed Then
    modetype := INDEX8
  Else
    If _mode.format.bits = 8 Then
      modetype := RGB332
    Else
      modetype := FAKEMODE;
  internal_pre_open_setup(_title);
  internal_open_fullscreen_start;
  internal_open_fullscreen(modetype);
  internal_open_fullscreen_finish(_pages);
  internal_post_open_setup;
End;

Procedure VGAConsole.close;

Begin
  If m_open Then
  Begin
    If m_locked Then
      Raise TPTCError.Create('console is still locked');
    { flush all key presses }
    While KeyPressed Do ReadKey;
    internal_close;
    m_open := False;
  End;
End;

Procedure VGAConsole.flush;

Begin
  check_open;
  check_unlocked;
End;

Procedure VGAConsole.finish;

Begin
  check_open;
  check_unlocked;
End;

Procedure VGAConsole.vga_load(data : Pointer); ASSembler;

Asm
  push es
  mov ax, fs
  mov es, ax
  mov ecx, 64000/4
  mov esi, [data]
  mov edi, 0A0000h
  cld
  rep movsd
  pop es
End;

Procedure VGAConsole.update;

Var
  framebuffer : PInteger;

Begin
  check_open;
  check_unlocked;
  Case m_CurrentMode Of
    0, 1 : Begin
      While (inportb($3DA) And 8) <> 0 Do;
      While (inportb($3DA) And 8) = 0 Do;
      vga_load(m_primary);
    End;
    2 : fakemode_load(m_primary, True);
  End;
{  WriteToVideoMemory(m_primary, 0, m_pitch * m_height);}
End;

Procedure VGAConsole.update(Const _area : TPTCArea);

Begin
  update;
End;

Procedure VGAConsole.internal_ReadKey(k : TPTCKey);

Begin
  check_open;
  m_keyboard.internal_ReadKey(k);
End;

Function VGAConsole.internal_PeekKey(k : TPTCKey) : Boolean;

Begin
  check_open;
  Result := m_keyboard.internal_PeekKey(k);
End;

Procedure VGAConsole.copy(Var surface : TPTCBaseSurface);

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

Procedure VGAConsole.copy(Var surface : TPTCBaseSurface;
                           Const source, destination : TPTCArea);

Var
  pixels : Pointer;

Begin
  check_open;
  check_unlocked;
  pixels := lock;
  Try
    surface.load(pixels, width, height, pitch, format, palette, source, destination);
    unlock;
  Except
    On error : TPTCError Do
    Begin
      unlock;
      Raise TPTCError.Create('failed to copy console to surface', error);
    End;
  End;
End;

Function VGAConsole.lock : Pointer;

Var
  pixels : Pointer;

Begin
  check_open;
  If m_locked Then
    Raise TPTCError.Create('console is already locked');
  pixels := m_primary;
  m_locked := True;
  lock := pixels;
End;

Procedure VGAConsole.unlock;

Begin
  check_open;
  If Not m_locked Then
    Raise TPTCError.Create('console is not locked');
  m_locked := False;
End;

Procedure VGAConsole.load(Const pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
  check_open;
  check_unlocked;
  If clip.Equals(area) Then
  Begin
    console_pixels := lock;
    Try
      Try
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, 0, 0, _width, _height, _pitch, console_pixels, 0, 0,
                    width, height, pitch);
      Except
        On error : TPTCError Do
        Begin
          Raise TPTCError.Create('failed to load pixels to console', error);
        End;
      End;
    Finally
      unlock;
    End;
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    Try
      load(pixels, _width, _height, _pitch, _format, _palette, Area_, area);
    Finally
      Area_.Destroy;
    End;
  End;
End;

Procedure VGAConsole.load(Const pixels : Pointer;
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
  clipped_destination := Nil;
  clipped_source := TPTCArea.Create;
  Try
    clipped_destination := TPTCArea.Create;
    console_pixels := lock;
    Try
      Try
        tmp := TPTCArea.Create(0, 0, _width, _height);
        Try
          TPTCClipper.clip(source, tmp, clipped_source, destination, clip, clipped_destination);
        Finally
          tmp.Destroy;
        End;
        m_copy.request(_format, format);
        m_copy.palette(_palette, palette);
        m_copy.copy(pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, _pitch,
                    console_pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, pitch);
      Except
        On error:TPTCError Do
        Begin
          Raise TPTCError.Create('failed to load pixels to console area', error);
        End;
      End;
    Finally
      unlock;
    End;
  Finally
    clipped_source.Destroy;
    If clipped_destination <> Nil Then
      clipped_destination.Destroy;
  End;
End;

Procedure VGAConsole.save(pixels : Pointer;
                           _width, _height, _pitch : Integer;
                           Const _format : TPTCFormat;
                           Const _palette : TPTCPalette);
Var
  Area_ : TPTCArea;
  console_pixels : Pointer;

Begin
  check_open;
  check_unlocked;
  If clip.Equals(area) Then
  Begin
    console_pixels := lock;
    Try
      Try
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, 0, 0, width, height, pitch, pixels, 0, 0,
                    _width, _height, _pitch);
      Except
        On error : TPTCError Do
        Begin
          Raise TPTCError.Create('failed to save console pixels', error);
        End;
      End;
    Finally
      unlock;
    End;
  End
  Else
  Begin
    Area_ := TPTCArea.Create(0, 0, width, height);
    Try
      save(pixels, _width, _height, _pitch, _format, _palette, area, Area_);
    Finally
      Area_.Destroy;
    End;
  End;
End;

Procedure VGAConsole.save(pixels : Pointer;
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
  clipped_destination := Nil;
  clipped_source := TPTCArea.Create;
  Try
    clipped_destination := TPTCArea.Create;
    console_pixels := lock;
    Try
      Try
        tmp := TPTCArea.Create(0, 0, _width, _height);
        Try
          TPTCClipper.clip(source, clip, clipped_source, destination, tmp, clipped_destination);
        Finally
          tmp.Destroy;
        End;
        m_copy.request(format, _format);
        m_copy.palette(palette, _palette);
        m_copy.copy(console_pixels, clipped_source.left, clipped_source.top, clipped_source.width, clipped_source.height, pitch,
                    pixels, clipped_destination.left, clipped_destination.top, clipped_destination.width, clipped_destination.height, _pitch);
      Except
        On error:TPTCError Do
        Begin
          Raise TPTCError.Create('failed to save console area pixels', error);
        End;
      End;
    Finally
      unlock;
    End;
  Finally
    clipped_source.Destroy;
    If clipped_destination <> Nil Then
      clipped_destination.Destroy;
  End;
End;

Procedure VGAConsole.clear;

Var
  tmp : TPTCColor;

Begin
  check_open;
  check_unlocked;
  If format.direct Then
    tmp := TPTCColor.Create(0, 0, 0, 0)
  Else
    tmp := TPTCColor.Create(0);
  Try
    clear(tmp);
  Finally
    tmp.Destroy;
  End;
End;

Procedure VGAConsole.clear(Const color : TPTCColor);

Var
  tmp : TPTCArea;

Begin
  check_open;
  check_unlocked;
  tmp := TPTCArea.Create;
  Try
    clear(color, tmp);
  Finally
    tmp.Destroy;
  End;
End;

Procedure VGAConsole.clear(Const color : TPTCColor;
                           Const _area : TPTCArea);

Begin
  {...}
End;

Procedure VGAConsole.palette(Const _palette : TPTCPalette);

Begin
  check_open;
  If format.indexed Then
  Begin
    m_palette.load(_palette.data);
    internal_SetPalette(_palette.data);
  End;
End;

Function VGAConsole.palette : TPTCPalette;

Begin
  check_open;
  palette := m_palette;
End;

Procedure VGAConsole.clip(Const _area : TPTCArea);

Var
  tmp : TPTCArea;

Begin
  check_open;
  tmp := TPTCClipper.clip(_area, m_area);
  Try
    m_clip.ASSign(tmp);
  Finally
    tmp.Destroy;
  End;
End;

Function VGAConsole.width : Integer;

Begin
  check_open;
  width := m_width;
End;

Function VGAConsole.height : Integer;

Begin
  check_open;
  height := m_height;
End;

Function VGAConsole.pitch : Integer;

Begin
  check_open;
  pitch := m_pitch;
End;

Function VGAConsole.pages : Integer;

Begin
  check_open;
  pages := 2;{m_primary.pages;}
End;

Function VGAConsole.area : TPTCArea;

Begin
  check_open;
  area := m_area;
End;

Function VGAConsole.clip : TPTCArea;

Begin
  check_open;
  clip := m_clip;
End;

Function VGAConsole.format : TPTCFormat;

Begin
  check_open;
  format := m_modes[m_CurrentMode].format;
End;

Function VGAConsole.name : String;

Begin
  name := 'VGA';
End;

Function VGAConsole.title : String;

Begin
End;

Function VGAConsole.information : String;

Begin
End;

Procedure VGAConsole.internal_pre_open_setup(Const _title : String);

Begin

End;

Procedure VGAConsole.internal_open_fullscreen_start;

{Var
  f : TPTCFormat;}

Begin
{  f := TPTCFormat.Create(32, $0000FF, $00FF00, $FF0000);}
{  m_160x100buffer := TPTCSurface.Create(160, 100, f);}
{  f.Destroy;}
{  set80x50;}
End;

Procedure VGAConsole.internal_open_fullscreen(ModeType : Integer);

Var
  tmp : TPTCArea;

Begin
  VGASetMode(320, 200, ModeType, m_faketype);
  Case ModeType Of
    INDEX8 : Begin
      m_CurrentMode := 0;
      m_pitch := 320;
    End;
    RGB332 : Begin
      m_CurrentMode := 1;
      m_pitch := 320;
    End;
    FAKEMODE : Begin
      m_CurrentMode := 2;
      m_pitch := 640;
    End;
  End;
  m_width := 320;
  m_height := 200;

  tmp := TPTCArea.Create(0, 0, width, height);
  Try
    m_area.ASSign(tmp);
    m_clip.ASSign(tmp);
  Finally
    tmp.Destroy;
  End;
End;

Procedure VGAConsole.internal_open_fullscreen_finish(_pages : Integer);

Begin
  If m_primary <> Nil Then
    FreeMem(m_primary);
  m_primary := GetMem(m_height * m_pitch);
End;

Procedure VGAConsole.internal_post_open_setup;

Begin
  If m_keyboard <> Nil Then
    m_keyboard.Destroy;
  m_keyboard := TDosKeyboard.Create;

  { temporary platform dependent information fudge }
  {sprintf(m_information,"dos version x.xx.x\nvesa version x.xx\nvesa driver name xxxxx\ndisplay driver vendor xxxxx\ncertified driver? x\n");}

  { set open flag }
  m_open := True;
End;

Procedure VGAConsole.internal_reset;

Begin
  If m_primary <> Nil Then
    FreeMem(m_primary);
  m_primary := Nil;
  If m_keyboard <> Nil Then
    m_keyboard.Destroy;
  m_keyboard := Nil;
{  m_primary.Destroy;}
{  m_keyboard.Destroy;}
{  m_primary := Nil;}
{  m_keyboard := Nil;}
End;

Procedure VGAConsole.internal_close;

Begin
  If m_primary <> Nil Then
  Begin
    FreeMem(m_primary);
    m_primary := Nil;
  End;
  RestoreTextMode;
End;

Procedure VGAConsole.internal_SetPalette(data : Pint32);

Var
  i : Integer;
  c : DWord;

Begin
  outportb($3C8, 0);
  For i := 0 To 255 Do
  Begin
    c := (data^ Shr 2) And $003F3F3F;
    outportb($3C9, c Shr 16);
    outportb($3C9, c Shr 8);
    outportb($3C9, c);
    Inc(data);
  End;
End;

Procedure VGAConsole.check_open;

Begin
  {$IFDEF DEBUG}
    If Not m_open Then
      Raise TPTCError.Create('console is not open');
  {$ELSE}
  {$ENDIF}
End;

Procedure VGAConsole.check_unlocked;

Begin
  {$IFDEF DEBUG}
    If m_locked Then
      Raise TPTCError.Create('console is not unlocked');
  {$ELSE}
  {$ENDIF}
End;
