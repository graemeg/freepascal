Function ptc_key_create(code : Integer; alt, shift, control : Boolean) : TPTC_KEY;

Begin
  Try
    ptc_key_create := TPTC_KEY(TPTCKey.Create(code, alt, shift, control));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_key_create := Nil;
    End;
  End;
End;

Procedure ptc_key_destroy(obj : TPTC_KEY);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCKey(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_key_code(obj : TPTC_KEY) : Integer;

Begin
  Try
    ptc_key_code := Integer(TPTCKey(obj).code);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_key_code := 0;
    End;
  End;
End;

Function ptc_key_alt(obj : TPTC_KEY) : Boolean;

Begin
  Try
    ptc_key_alt := TPTCKey(obj).alt;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_key_alt := False;
    End;
  End;
End;

Function ptc_key_shift(obj : TPTC_KEY) : Boolean;

Begin
  Try
    ptc_key_shift := TPTCKey(obj).shift;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_key_shift := False;
    End;
  End;
End;

Function ptc_key_control(obj : TPTC_KEY) : Boolean;

Begin
  Try
    ptc_key_control := TPTCKey(obj).control;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_key_control := False;
    End;
  End;
End;

Procedure ptc_key_assign(obj : TPTC_KEY; key : TPTC_KEY);

Begin
  Try
    TPTCKey(obj).ASSign(TPTCKey(key));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_key_equals(obj : TPTC_KEY; key : TPTC_KEY) : Boolean;

Begin
  Try
    ptc_key_equals := TPTCKey(obj).Equals(TPTCKey(key));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_key_equals := False;
    End;
  End;
End;
