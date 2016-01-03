Function ptc_surface_create(width, height : Integer; format : TPTC_FORMAT) : TPTC_SURFACE;

Begin
  Try
    ptc_surface_create := TPTC_SURFACE(TPTCSurface.Create(width, height, TPTCFormat(format)));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_create := Nil;
    End;
  End;
End;

Procedure ptc_surface_destroy(obj : TPTC_SURFACE);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCBaseSurface(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_copy(obj : TPTC_SURFACE; surface : TPTC_SURFACE);

Begin
  Try
    TPTCBaseSurface(obj).copy(TPTCBaseSurface(surface));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_copy_area(obj : TPTC_SURFACE; surface : TPTC_SURFACE; source, destination : TPTC_AREA);

Begin
  Try
    TPTCBaseSurface(obj).copy(TPTCBaseSurface(surface), TPTCArea(source), TPTCArea(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_surface_lock(obj : TPTC_SURFACE) : Pointer;

Begin
  Try
    ptc_surface_lock := TPTCBaseSurface(obj).lock;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_lock := Nil;
    End;
  End;
End;

Procedure ptc_surface_unlock(obj : TPTC_SURFACE);

Begin
  Try
    TPTCBaseSurface(obj).unlock;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_load(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);

Begin
  Try
    TPTCBaseSurface(obj).load(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_load_area(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

Begin
  Try
    TPTCBaseSurface(obj).load(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette), TPTCArea(source), TPTCArea(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_save(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE);

Begin
  Try
    TPTCBaseSurface(obj).save(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_save_area(obj : TPTC_SURFACE; pixels : Pointer; width, height, pitch : Integer; format : TPTC_FORMAT; palette : TPTC_PALETTE; source, destination : TPTC_AREA);

Begin
  Try
    TPTCBaseSurface(obj).save(pixels, width, height, pitch, TPTCFormat(format), TPTCPalette(palette), TPTCArea(source), TPTCArea(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_clear(obj : TPTC_SURFACE);

Begin
  Try
    TPTCBaseSurface(obj).clear;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_clear_color(obj : TPTC_SURFACE; color : TPTC_COLOR);

Begin
  Try
    TPTCBaseSurface(obj).clear(TPTCColor(color));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_clear_color_area(obj : TPTC_SURFACE; color : TPTC_COLOR; area : TPTC_AREA);

Begin
  Try
    TPTCBaseSurface(obj).clear(TPTCColor(color), TPTCArea(area));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_surface_palette_set(obj : TPTC_SURFACE; palette : TPTC_PALETTE);

Begin
  Try
    TPTCBaseSurface(obj).palette(TPTCPalette(palette));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_surface_palette_get(obj : TPTC_SURFACE) : TPTC_PALETTE;

Begin
  Try
    ptc_surface_palette_get := TPTC_PALETTE(TPTCBaseSurface(obj).palette);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_palette_get := Nil;
    End;
  End;
End;

Procedure ptc_surface_clip_set(obj : TPTC_SURFACE; area : TPTC_AREA);

Begin
  Try
    TPTCBaseSurface(obj).clip(TPTCArea(area));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_surface_width(obj : TPTC_SURFACE) : Integer;

Begin
  Try
    ptc_surface_width := TPTCBaseSurface(obj).width;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_width := 0;
    End;
  End;
End;

Function ptc_surface_height(obj : TPTC_SURFACE) : Integer;

Begin
  Try
    ptc_surface_height := TPTCBaseSurface(obj).height;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_height := 0;
    End;
  End;
End;

Function ptc_surface_pitch(obj : TPTC_SURFACE) : Integer;

Begin
  Try
    ptc_surface_pitch := TPTCBaseSurface(obj).pitch;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_pitch := 0;
    End;
  End;
End;

Function ptc_surface_area(obj : TPTC_SURFACE) : TPTC_AREA;

Begin
  Try
    ptc_surface_area := TPTC_AREA(TPTCBaseSurface(obj).area);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_area := Nil;
    End;
  End;
End;

Function ptc_surface_clip(obj : TPTC_SURFACE) : TPTC_AREA;

Begin
  Try
    ptc_surface_clip := TPTC_AREA(TPTCBaseSurface(obj).clip);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_clip := Nil;
    End;
  End;
End;

Function ptc_surface_format(obj : TPTC_SURFACE) : TPTC_FORMAT;

Begin
  Try
    ptc_surface_format := TPTC_FORMAT(TPTCBaseSurface(obj).format);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_format := Nil;
    End;
  End;
End;

Function ptc_surface_option(obj : TPTC_SURFACE; _option : String) : Boolean;

Begin
  Try
    ptc_surface_option := TPTCBaseSurface(obj).option(_option);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_surface_option := False;
    End;
  End;
End;
