Function ptc_console_create : TPTC_CONSOLE;

Begin
  Try
    ptc_console_create := TPTC_CONSOLE(TPTCConsole.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_create := Nil;
    End;
  End;
End;

Procedure ptc_console_destroy(obj : TPTC_CONSOLE);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCBaseConsole(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_configure(obj : TPTC_CONSOLE; _file : String);

Begin
  Try
    TPTCBaseConsole(obj).configure(_file);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_console_option(obj : TPTC_CONSOLE; _option : String) : Boolean;

Begin
  Try
    ptc_console_option := TPTCBaseConsole(obj).option(_option);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_option := False;
    End;
  End;
End;

Function ptc_console_mode(obj : TPTC_CONSOLE; index : Integer) : TPTC_MODE;

Begin
  Try
    ptc_console_mode := TPTC_MODE(TPTCBaseConsole(obj).modes[index]);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_mode := Nil;
    End;
  End;
End;

Procedure ptc_console_open(obj : TPTC_CONSOLE; title : String; pages : Integer);

Begin
  Try
    TPTCBaseConsole(obj).open(title, pages);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_open_format(obj : TPTC_CONSOLE; title : String; format : TPTC_FORMAT; pages : Integer);

Begin
  Try
    TPTCBaseConsole(obj).open(title, TPTCFormat(format), pages);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_open_resolution(obj : TPTC_CONSOLE; title : String; width, height : Integer; format : TPTC_FORMAT; pages : Integer);

Begin
  Try
    TPTCBaseConsole(obj).open(title, width, height, TPTCFormat(format), pages);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_open_mode(obj : TPTC_CONSOLE; title : String; mode : TPTC_MODE; pages : Integer);

Begin
  Try
    TPTCBaseConsole(obj).open(title, TPTCMode(mode), pages);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_close(obj : TPTC_CONSOLE);

Begin
  Try
    TPTCBaseConsole(obj).close;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_flush(obj : TPTC_CONSOLE);

Begin
  Try
    TPTCBaseConsole(obj).flush;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_finish(obj : TPTC_CONSOLE);

Begin
  Try
    TPTCBaseConsole(obj).finish;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_update(obj : TPTC_CONSOLE);

Begin
  Try
    TPTCBaseConsole(obj).update;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_update_area(obj : TPTC_CONSOLE; area : TPTC_AREA);

Begin
  Try
    TPTCBaseConsole(obj).update(TPTCArea(area));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_console_key(obj : TPTC_CONSOLE) : Boolean;

Begin
  Try
    ptc_console_key := TPTCBaseConsole(obj).key;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_key := False;
    End;
  End;
End;

Procedure ptc_console_read(obj : TPTC_CONSOLE; key : TPTC_KEY);

Var
  tmp : TPTCKey;

Begin
  Try
    tmp := TPTCBaseConsole(obj).read;
    Try
      TPTCKey(key).ASSign(tmp);
    Finally
      tmp.Destroy;
    End;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_copy(obj : TPTC_CONSOLE; surface : TPTC_SURFACE);

Begin
  Try
    TPTCBaseConsole(obj).copy(TPTCBaseSurface(surface));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_copy_area(obj : TPTC_CONSOLE; surface : TPTC_SURFACE; source, destination : TPTC_AREA);

Begin
  Try
    TPTCBaseConsole(obj).copy(TPTCBaseSurface(surface), TPTCArea(source), TPTCArea(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_console_lock(obj : TPTC_CONSOLE) : Pointer;

Begin
  Try
    ptc_console_lock := TPTCBaseConsole(obj).lock;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_lock := Nil;
    End;
  End;
End;

Procedure ptc_console_unlock(obj : TPTC_CONSOLE);

Begin
  Try
    TPTCBaseConsole(obj).unlock;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_load(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);

Begin
  Try
    TPTCBaseConsole(obj).load(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_load_area(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

Begin
  Try
    TPTCBaseConsole(obj).load(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette), TPTCArea(source), TPTCArea(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_save(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);

Begin
  Try
    TPTCBaseConsole(obj).save(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_save_area(obj : TPTC_CONSOLE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

Begin
  Try
    TPTCBaseConsole(obj).save(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette), TPTCArea(source), TPTCArea(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_clear(obj : TPTC_CONSOLE);

Begin
  Try
    TPTCBaseConsole(obj).clear;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_clear_color(obj : TPTC_CONSOLE; color : TPTC_COLOR);

Begin
  Try
    TPTCBaseConsole(obj).clear(TPTCColor(color));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_clear_color_area(obj : TPTC_CONSOLE; color : TPTC_COLOR; area : TPTC_AREA);

Begin
  Try
    TPTCBaseConsole(obj).clear(TPTCColor(color), TPTCArea(area));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_console_palette_set(obj : TPTC_CONSOLE; palette : TPTC_PALETTE);

Begin
  Try
    TPTCBaseConsole(obj).palette(TPTCPalette(palette));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_console_palette_get(obj : TPTC_CONSOLE) : TPTC_PALETTE;

Begin
  Try
    ptc_console_palette_get := TPTC_PALETTE(TPTCBaseConsole(obj).palette);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_palette_get := Nil;
    End;
  End;
End;

Procedure ptc_console_clip_set(obj : TPTC_CONSOLE; area : TPTC_AREA);

Begin
  Try
    TPTCBaseConsole(obj).clip(TPTCArea(area));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_console_width(obj : TPTC_CONSOLE) : Integer;

Begin
  Try
    ptc_console_width := TPTCBaseConsole(obj).width;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_width := 0;
    End;
  End;
End;

Function ptc_console_height(obj : TPTC_CONSOLE) : Integer;

Begin
  Try
    ptc_console_height := TPTCBaseConsole(obj).height;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_height := 0;
    End;
  End;
End;

Function ptc_console_pages(obj : TPTC_CONSOLE) : Integer;

Begin
  Try
    ptc_console_pages := TPTCBaseConsole(obj).pages;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_pages := 0;
    End;
  End;
End;

Function ptc_console_pitch(obj : TPTC_CONSOLE) : Integer;

Begin
  Try
    ptc_console_pitch := TPTCBaseConsole(obj).pitch;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_pitch := 0;
    End;
  End;
End;

Function ptc_console_area(obj : TPTC_CONSOLE) : TPTC_AREA;

Begin
  Try
    ptc_console_area := TPTC_AREA(TPTCBaseConsole(obj).area);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_area := Nil;
    End;
  End;
End;

Function ptc_console_clip(obj : TPTC_CONSOLE) : TPTC_AREA;

Begin
  Try
    ptc_console_clip := TPTC_AREA(TPTCBaseConsole(obj).clip);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_clip := Nil;
    End;
  End;
End;

Function ptc_console_format(obj : TPTC_CONSOLE) : TPTC_FORMAT;

Begin
  Try
    ptc_console_format := TPTC_FORMAT(TPTCBaseConsole(obj).format);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_format := Nil;
    End;
  End;
End;

Function ptc_console_name(obj : TPTC_CONSOLE) : String;

Begin
  Try
    ptc_console_name := TPTCBaseConsole(obj).name;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_name := '';
    End;
  End;
End;

Function ptc_console_title(obj : TPTC_CONSOLE) : String;

Begin
  Try
    ptc_console_title := TPTCBaseConsole(obj).title;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_title := '';
    End;
  End;
End;

Function ptc_console_information(obj : TPTC_CONSOLE) : String;

Begin
  Try
    ptc_console_information := TPTCBaseConsole(obj).information;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_console_information := '';
    End;
  End;
End;
