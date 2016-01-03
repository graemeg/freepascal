Function ptc_mode_create(width, height : Integer; format : TPTC_FORMAT) : TPTC_MODE;

Begin
  Try
    ptc_mode_create := TPTC_MODE(TPTCMode.Create(width, height, TPTCFormat(format)));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_mode_create := Nil;
    End;
  End;
End;

Function ptc_mode_create_invalid : TPTC_MODE;

Begin
  Try
    ptc_mode_create_invalid := TPTC_MODE(TPTCMode.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_mode_create_invalid := Nil;
    End;
  End;
End;

Procedure ptc_mode_destroy(obj : TPTC_MODE);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCMode(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_mode_valid(obj : TPTC_MODE) : Boolean;

Begin
  Try
    ptc_mode_valid := TPTCMode(obj).valid;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_mode_valid := False;
    End;
  End;
End;

Function ptc_mode_width(obj : TPTC_MODE) : Integer;

Begin
  Try
    ptc_mode_width := TPTCMode(obj).width;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_mode_width := 0;
    End;
  End;
End;

Function ptc_mode_height(obj : TPTC_MODE) : Integer;

Begin
  Try
    ptc_mode_height := TPTCMode(obj).height;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_mode_height := 0;
    End;
  End;
End;

Function ptc_mode_format(obj : TPTC_MODE) : TPTC_FORMAT;

Begin
  Try
    ptc_mode_format := TPTC_FORMAT(TPTCMode(obj).format);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_mode_format := Nil;
    End;
  End;
End;

Procedure ptc_mode_assign(obj, mode : TPTC_MODE);

Begin
  Try
    TPTCMode(obj).ASSign(TPTCMode(mode));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_mode_equals(obj, mode : TPTC_MODE) : Boolean;

Begin
  Try
    ptc_mode_equals := TPTCMode(obj).Equals(TPTCMode(mode));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_mode_equals := False;
    End;
  End;
End;
