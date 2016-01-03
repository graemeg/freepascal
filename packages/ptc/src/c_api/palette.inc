Function ptc_palette_create : TPTC_PALETTE;

Begin
  Try
    ptc_palette_create := TPTC_PALETTE(TPTCPalette.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_palette_create := Nil;
    End;
  End;
End;

{Function ptc_palette_create_data(data : Pint32) : TPTC_PALETTE;

Begin
  Try
    ptc_palette_create_data := TPTC_PALETTE(TPTCPalette.Create(data));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_palette_create_data := Nil;
    End;
  End;
End;}

Procedure ptc_palette_destroy(obj : TPTC_PALETTE);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCPalette(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_palette_lock(obj : TPTC_PALETTE) : Pint32;

Begin
  Try
    ptc_palette_lock := TPTCPalette(obj).lock;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_palette_lock := Nil;
    End;
  End;
End;

Procedure ptc_palette_unlock(obj : TPTC_PALETTE);

Begin
  Try
    TPTCPalette(obj).unlock;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_palette_load(obj : TPTC_PALETTE; data : Pint32);

Begin
  Try
    TPTCPalette(obj).load(data);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_palette_save(obj : TPTC_PALETTE; data : Pint32);

Begin
  Try
    TPTCPalette(obj).save(data);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_palette_data(obj : TPTC_PALETTE) : Pint32;

Begin
  Try
    ptc_palette_data := TPTCPalette(obj).data;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_palette_data := Nil;
    End;
  End;
End;

Procedure ptc_palette_assign(obj, palette : TPTC_PALETTE);

Begin
  Try
    TPTCPalette(obj).ASSign(TPTCPalette(palette));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_palette_equals(obj, palette : TPTC_PALETTE) : Boolean;

Begin
  Try
    ptc_palette_equals := TPTCPalette(obj).Equals(TPTCPalette(palette));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_palette_equals := False;
    End;
  End;
End;
