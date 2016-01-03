Function ptc_clear_create : TPTC_CLEAR;

Begin
  Try
    ptc_clear_create := TPTC_CLEAR(TPTCClear.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_clear_create := Nil;
    End;
  End;
End;

Procedure ptc_clear_destroy(obj : TPTC_CLEAR);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCClear(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_clear_request(obj : TPTC_CLEAR; format : TPTC_FORMAT);

Begin
  Try
    TPTCClear(obj).request(TPTCFormat(format));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_clear_clear(obj : TPTC_CLEAR; pixels : Pointer; x, y, width, height, pitch : Integer; color : TPTC_COLOR);

Begin
  Try
    TPTCClear(obj).clear(pixels, x, y, width, height, pitch, TPTCColor(color));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;
