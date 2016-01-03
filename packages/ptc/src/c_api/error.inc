Function ptc_error_create(message : String) : TPTC_ERROR;

Begin
  Try
    ptc_error_create := TPTC_ERROR(TPTCError.Create(message));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_error_create := Nil;
    End;
  End;
End;

Function ptc_error_create_composite(message : String; error : TPTC_ERROR) : TPTC_ERROR;

Begin
  Try
    ptc_error_create_composite := TPTC_ERROR(TPTCError.Create(message, TPTCError(error)));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_error_create_composite := Nil;
    End;
  End;
End;

Procedure ptc_error_destroy(obj : TPTC_ERROR);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCError(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_error_report(obj : TPTC_ERROR);

Begin
  Try
    TPTCError(obj).report;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_error_message(obj : TPTC_ERROR) : String;

Begin
  Try
    ptc_error_message := TPTCError(obj).message;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_error_message := '';
    End;
  End;
End;

Procedure ptc_error_assign(obj, error : TPTC_ERROR);

Begin
  Try
    TPTCError(obj).ASSign(TPTCError(error));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_error_equals(obj, error : TPTC_ERROR) : Boolean;

Begin
  Try
    ptc_error_equals := TPTCError(obj).Equals(TPTCError(error));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_error_equals := False;
    End;
  End;
End;

Procedure ptc_error_handler(handler : TPTC_ERROR_HANDLER);

Begin
  ptc_exception_handler(handler);
End;
