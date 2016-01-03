{$MACRO ON}

{$DEFINE DEFAULT_WIDTH:=320}
{$DEFINE DEFAULT_HEIGHT:=200}
{$DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(32, $00FF0000, $0000FF00, $000000FF)}
{ $DEFINE DEFAULT_FORMAT:=TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF)}

Constructor VESAConsole.Create;

Var
  I, J : Integer;
  r, g, b, a : DWord;
  tmpbpp : Integer;
  tmp : TPTCFormat;

Begin
  m_modes := Nil;
  m_modes_n := Nil;
  m_keyboard := Nil;
  m_open := False;
  m_locked := False;
  m_default_format := Nil;
  m_palette := Nil;
  m_copy := Nil;
  m_area := Nil;
  m_clip := Nil;
  m_title := '';
  m_information := '';
  m_default_width := DEFAULT_WIDTH;
  m_default_height := DEFAULT_HEIGHT;
  m_default_format := DEFAULT_FORMAT;

  InitVESA;
  m_primary := Nil;

  m_modes_last := -1;
  For I := 0 To NrOfModes Do
    With ModeInfo[I].VesaModeInfo Do
      If (MemoryModel = 6) And
         ((BitsPerPixel = 8) Or
          (BitsPerPixel = 15) Or
          (BitsPerPixel = 16) Or
          (BitsPerPixel = 24) Or
          (BitsPerPixel = 32)) Then
        Inc(m_modes_last)
      Else
        If (MemoryModel = 4) And (BitsPerPixel = 8) Then
          Inc(m_modes_last, 2);
  GetMem(m_modes, (m_modes_last + 2) * SizeOf(TPTCMode));
  FillChar(m_modes^, (m_modes_last + 2) * SizeOf(TPTCMode), 0);
  GetMem(m_modes_n, (m_modes_last + 1) * SizeOf(Integer));
//  Writeln(m_modes_last, ' ', NrOfModes);
  m_modes[m_modes_last + 1] := TPTCMode.Create; {mark end of list!}
  J := -1;
  For I := 0 To NrOfModes Do
    With ModeInfo[I].VesaModeInfo Do
      If (MemoryModel = 6) And
         ((BitsPerPixel = 8) Or
          (BitsPerPixel = 15) Or
          (BitsPerPixel = 16) Or
          (BitsPerPixel = 24) Or
          (BitsPerPixel = 32)) Then
      Begin
        Inc(J);
        r := MakeMask(RedMaskSize, RedFieldPosition);
        g := MakeMask(GreenMaskSize, GreenFieldPosition);
        b := MakeMask(BlueMaskSize, BlueFieldPosition);
        {a := MakeMask(RsvdMaskSize, RsvdFieldPosition);}
        a := 0;
        If BitsPerPixel = 15 Then
          tmpbpp := 16
        Else
          tmpbpp := BitsPerPixel;
        tmp := TPTCFormat.Create(tmpbpp, r, g, b, a);
        Try
          m_modes[J] := TPTCMode.Create(XResolution, YResolution, tmp);
          m_modes_n[J] := I;
        Finally
          tmp.Destroy;
        End;
{        Inc(m_modes_last)}
      End
      Else
        If (MemoryModel = 4) And (BitsPerPixel = 8) Then
        Begin
          Inc(J);
          tmp := TPTCFormat.Create(8);
          Try
            m_modes[J] := TPTCMode.Create(XResolution, YResolution, tmp);
            m_modes_n[J] := I;
          Finally
            tmp.Destroy;
          End;
          Inc(J);
          tmp := TPTCFormat.Create(8, $E0, $1C, $03); {RGB 332}
          Try
            m_modes[J] := TPTCMode.Create(XResolution, YResolution, tmp);
            m_modes_n[J] := I;
          Finally
            tmp.Destroy;
          End;
{         Inc(m_modes_last, 2);}
        End;

  m_clip := TPTCArea.Create;
  m_area := TPTCArea.Create;
  m_copy := TPTCCopy.Create;
  m_palette := TPTCPalette.Create;
  configure('ptc.cfg');
End;

Destructor VESAConsole.Destroy;

Var
  I : Integer;

Begin
  close;
  If m_modes <> Nil Then
    For I := 0 To m_modes_last + 1 Do
      If m_modes[I] <> Nil Then
        m_modes[I].Destroy;
  If m_modes <> Nil Then
    FreeMem(m_modes);
  If m_modes_n <> Nil Then
    FreeMem(m_modes_n);
  If m_keyboard <> Nil Then
    m_keyboard.Destroy;
  If m_copy <> Nil Then
    m_copy.Destroy;
  If m_default_format <> Nil Then
    m_default_format.Destroy;
  If m_palette <> Nil Then
    m_palette.Destroy;
  If m_area <> Nil Then
    m_area.Destroy;
  If m_clip <> Nil Then
    m_clip.Destroy;
  Inherited Destroy;
End;

Procedure VESAConsole.configure(Const _file : String);

Var
  F : Text;
  S : String;

Begin
  ASSignFile(F, _file);
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

Function VESAConsole.option(Const _option : String) : Boolean;

Begin
  {...}
  option := m_copy.option(_option);
End;

Function VESAConsole.modes : PPTCMode;

Begin
  {todo...}
  modes := m_modes;
End;

Procedure VESAConsole.open(Const _title : String; _pages : Integer); Overload;

Begin
  open(_title, m_default_format, _pages);
End;

Procedure VESAConsole.open(Const _title : String; Const _format : TPTCFormat;
                           _pages : Integer); Overload;

Begin
  open(_title, m_default_width, m_default_height, _format, _pages);
End;

Procedure VESAConsole.open(Const _title : String; _width, _height : Integer;
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

Procedure VESAConsole.open(Const _title : String; Const _mode : TPTCMode;
                           _pages : Integer); Overload;

Var
{  _width, _height : Integer;
  _format : TPTCFormat;}
  I : Integer;
  modefound, bestmodefound : Integer;
  x, y, bpp : Integer;

Begin
  If Not _mode.valid Then
    Raise TPTCError.Create('invalid mode');
  modefound := -1;
  For I := 0 To m_modes_last Do
    If m_modes[I].Equals(_mode) Then
    Begin
      modefound := I;
      Break;
    End;
{  If modefound = -1 Then
    Raise TPTCError.Create('mode not found >:)');}
  bestmodefound := -1;
  If (modefound = -1) And (_mode.format.direct) Then
  Begin
    x := 100000000;
    y := x;
    bpp := -1;
    For I := 0 To m_modes_last Do
      If (m_modes[i].width >= _mode.width) And
         (m_modes[i].height >= _mode.height) And
         (m_modes[i].width <= x) And
         (m_modes[i].height <= y) And
         (((m_modes[i].format.bits >= bpp) And
           (bpp < _mode.format.bits)) Or
          ((m_modes[i].format.bits < bpp) And
           (m_modes[i].format.bits >= _mode.format.bits) And
           (bpp > _mode.format.bits))) Then
      Begin
        bestmodefound := I;
        x := m_modes[i].width;
        y := m_modes[i].height;
        bpp := m_modes[i].format.bits;
      End;
{      If m_modes[I].bpp >=  Then
      Begin
        modefound := I;
        Break;
      End;}
  End;
  If (modefound = -1) And (_mode.format.indexed) Then
  Begin
    x := 100000000;
    y := x;
    bpp := -1;
    For I := 0 To m_modes_last Do
      If (m_modes[i].width >= _mode.width) And
         (m_modes[i].height >= _mode.height) And
         (m_modes[i].width <= x) And
         (m_modes[i].height <= y) { And
         (((m_modes[i].format.bits >= bpp) And
           (bpp < _mode.format.bits)) Or
          ((m_modes[i].format.bits < bpp) And
           (m_modes[i].format.bits >= _mode.format.bits) And
           (bpp > _mode.format.bits)))} Then
      Begin
        If (m_modes[i].width <> x) Or (m_modes[i].height <> y) Then
          bpp := -1;
        If m_modes[i].format.indexed Or
           (m_modes[i].format.bits > bpp) Then
        Begin
          bestmodefound := I;
          x := m_modes[i].width;
          y := m_modes[i].height;
          bpp := m_modes[i].format.bits;
          If m_modes[i].format.indexed Then
            bpp := 1000;
        End;
      End;
{      If m_modes[I].bpp >=  Then
      Begin
        modefound := I;
        Break;
      End;}
  End;
  If bestmodefound <> -1 Then
    modefound := bestmodefound;
//  Writeln('mf', modefound);
//  Readln;
  If modefound = -1 Then
    Raise TPTCError.Create('mode not found >:)');
{  _width := _mode.width;
  _height := _mode.height;
  _format := _mode.format;}
{  m_CurrentMode := modefound;}
{  m_VESACurrentMode := m_modes_n[modefound];}
  internal_pre_open_setup(_title);
  internal_open_fullscreen_start;
  internal_open_fullscreen(modefound{m_CurrentMode});
  internal_open_fullscreen_finish(_pages);
  internal_post_open_setup;
End;

Procedure VESAConsole.close;

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

Procedure VESAConsole.flush;

Begin
  check_open;
  check_unlocked;
End;

Procedure VESAConsole.finish;

Begin
  check_open;
  check_unlocked;
End;

Procedure VESAConsole.update;

Var
  framebuffer : PInteger;

Begin
  check_open;
  check_unlocked;
  WriteToVideoMemory(m_primary, 0, m_pitch * m_height);
{  m_primary.clear;}
{  m_primary.copy(m_160x100buffer);
  framebuffer := m_160x100buffer.lock;
  dump_160x(0, 50, framebuffer);
  m_160x100buffer.unlock;}
End;

Procedure VESAConsole.update(Const _area : TPTCArea);

Begin
  update;
End;

Procedure VESAConsole.internal_ReadKey(k : TPTCKey);

Begin
  check_open;
  m_keyboard.internal_ReadKey(k);
End;

Function VESAConsole.internal_PeekKey(k : TPTCKey) : Boolean;

Begin
  check_open;
  Result := m_keyboard.internal_PeekKey(k);
End;

Procedure VESAConsole.copy(Var surface : TPTCBaseSurface);

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

Procedure VESAConsole.copy(Var surface : TPTCBaseSurface;
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

Function VESAConsole.lock : Pointer;

Var
  pixels : Pointer;

Begin
  check_open;
  If m_locked Then
    Raise TPTCError.Create('console is already locked');
{  pixels := m_primary.lock;}
  pixels := m_primary;
  m_locked := True;
  lock := pixels;
End;

Procedure VESAConsole.unlock;

Begin
  check_open;
  If Not m_locked Then
    Raise TPTCError.Create('console is not locked');
{  m_primary.unlock;}
  m_locked := False;
End;

Procedure VESAConsole.load(Const pixels : Pointer;
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

Procedure VESAConsole.load(Const pixels : Pointer;
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

Procedure VESAConsole.save(pixels : Pointer;
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

Procedure VESAConsole.save(pixels : Pointer;
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

Procedure VESAConsole.clear;

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

Procedure VESAConsole.clear(Const color : TPTCColor);

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

Procedure VESAConsole.clear(Const color : TPTCColor;
                           Const _area : TPTCArea);

Begin
  check_open;
  check_unlocked;
  {...}
End;

Procedure VESAConsole.palette(Const _palette : TPTCPalette);

Begin
  check_open;
{  m_primary.palette(_palette);}
  If format.indexed Then
  Begin
    m_palette.load(_palette.data);
    SetPalette(_palette.data, 0, 256);
  End;
End;

Function VESAConsole.palette : TPTCPalette;

Begin
  check_open;
  palette := m_palette;
{  palette := m_primary.palette;}
End;

Procedure VESAConsole.clip(Const _area : TPTCArea);

Var
  tmp : TPTCArea;

Begin
  check_open;
  tmp := TPTCClipper.clip(_area, m_area);
  Try
    m_clip.Assign(tmp);
  Finally
    tmp.Destroy;
  End;
End;

Function VESAConsole.width : Integer;

Begin
  check_open;
  width := m_width;
End;

Function VESAConsole.height : Integer;

Begin
  check_open;
  height := m_height;
End;

Function VESAConsole.pitch : Integer;

Begin
  check_open;
  pitch := m_pitch;
End;

Function VESAConsole.pages : Integer;

Begin
  check_open;
  pages := 2;{m_primary.pages;}
End;

Function VESAConsole.area : TPTCArea;

Begin
  check_open;
  area := m_area;
{  area := m_primary.area;}
End;

Function VESAConsole.clip : TPTCArea;

Begin
  check_open;
  clip := m_clip;
{  clip := m_primary.clip;}
End;

Function VESAConsole.format : TPTCFormat;

Begin
  check_open;
  format := m_modes[m_CurrentMode].format;
{  format := m_primary.format;}
End;

Function VESAConsole.name : String;

Begin
  name := 'VESA';
End;

Function VESAConsole.title : String;

Begin
  title := m_title;
End;

Function VESAConsole.information : String;

Begin
  information := m_information;
End;

Procedure VESAConsole.internal_pre_open_setup(Const _title : String);

Begin
  internal_close;
  m_title := _title;
End;

Procedure VESAConsole.internal_open_fullscreen_start;

{Var
  f : TPTCFormat;}

Begin
{  f := TPTCFormat.Create(32, $0000FF, $00FF00, $FF0000);}
{  m_160x100buffer := TPTCSurface.Create(160, 100, f);}
{  f.Destroy;}
{  set80x50;}
End;

Procedure VESAConsole.internal_open_fullscreen(ModeNr : Integer);

Var
  tmp : TPTCFormat;
  tmpa : TPTCArea;
  I : Integer;
  plt : Array[0..255] Of Packed Record
    B, G, R, A : Byte;
  End;

Begin
  m_CurrentMode := ModeNr;
  m_VESACurrentMode := m_modes_n[ModeNr];
  SetVESAMode(m_VESACurrentMode);
  tmp := TPTCFormat.Create(8, $E0, $1C, $03);
  If m_modes[m_CurrentMode].m_format.Equals(tmp) Then
  Begin
    For I := 0 To 255 Do
      With plt[I] Do
      Begin
        Case I Shr 5 Of
          0 : R := 0;
          1 : R := 36;
          2 : R := 73;
          3 : R := 109;
          4 : R := 146;
          5 : R := 182;
          6 : R := 219;
          7 : R := 255;
        End;
        Case (I Shr 2) And 7 Of
          0 : G := 0;
          1 : G := 36;
          2 : G := 73;
          3 : G := 109;
          4 : G := 146;
          5 : G := 182;
          6 : G := 219;
          7 : G := 255;
        End;
        Case I And 3 Of
          0 : B := 0;
          1 : B := 85;
          2 : B := 170;
          3 : B := 255;
        End;
        A := 0;
      End;
    SetPalette(@plt, 0, 256);
  End;
  tmp.Destroy;
{  m_primary := TPTCSurface.Create(_width, _height, _format);}
  With ModeInfo[m_VESACurrentMode].VesaModeInfo Do
  Begin
    m_width := XResolution;
    m_height := YResolution;
    m_pitch := BytesPerScanline;
  End;
  tmpa := TPTCArea.Create(0, 0, width, height);
  Try
    m_area.ASSign(tmpa);
    m_clip.ASSign(tmpa);
  Finally
    tmpa.Destroy;
  End;
End;

Procedure VESAConsole.internal_open_fullscreen_finish(_pages : Integer);

Begin
  m_primary := GetMem(m_height * m_pitch);
End;

Procedure VESAConsole.internal_post_open_setup;

Begin
  If m_keyboard <> Nil Then
    m_keyboard.Destroy;
  m_keyboard := TDosKeyboard.Create;

  { temporary platform dependent information fudge }
  m_information := 'dos version x.xx.x'+#13+#10+'vesa version x.xx'+#13+#10+'vesa driver name xxxxx'+#13+#10+'display driver vendor xxxxx'+#13+#10+'certified driver? x'+#13+#10;

  { set open flag }
  m_open := True;
End;

Procedure VESAConsole.internal_reset;

Begin
  If m_keyboard <> Nil Then
  Begin
    m_keyboard.Destroy;
    m_keyboard := Nil;
  End;
End;

Procedure VESAConsole.internal_close;

Begin
  If m_primary <> Nil Then
  Begin
    FreeMem(m_primary);
    m_primary := Nil;
  End;
  If m_keyboard <> Nil Then
  Begin
    m_keyboard.Destroy;
    m_keyboard := Nil;
  End;
  RestoreTextMode;
End;

Procedure VESAConsole.check_open;

Begin
  {$IFDEF DEBUG}
    If Not m_open Then
      Raise TPTCError.Create('console is not open');
  {$ELSE}
  {$ENDIF}
End;

Procedure VESAConsole.check_unlocked;

Begin
  {$IFDEF DEBUG}
    If m_locked Then
      Raise TPTCError.Create('console is not unlocked');
  {$ELSE}
  {$ENDIF}
End;
