Var
  ptc_error_handler_function : TPTC_ERROR_HANDLER;

Procedure ptc_error_handler_default(error : TPTC_ERROR);

Begin
  TPTCError(error).report;
End;

Procedure ptc_exception_handler(handler : TPTC_ERROR_HANDLER);

Begin
  If handler = Nil Then
    ptc_error_handler_function := @ptc_error_handler_default
  Else
    ptc_error_handler_function := handler;
End;

Procedure ptc_exception_handle(error : TPTCError);

Begin
  ptc_error_handler_function(TPTC_ERROR(error));
End;
