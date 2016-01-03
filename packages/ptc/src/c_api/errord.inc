Type
  TPTC_ERROR_HANDLER = Procedure(error : TPTC_ERROR);

Function ptc_error_create(message : String) : TPTC_ERROR;
Function ptc_error_create_composite(message : String; error : TPTC_ERROR) : TPTC_ERROR;
Procedure ptc_error_destroy(obj : TPTC_ERROR);

Procedure ptc_error_report(obj : TPTC_ERROR);

Function ptc_error_message(obj : TPTC_ERROR) : String;

Procedure ptc_error_assign(obj, error : TPTC_ERROR);
Function ptc_error_equals(obj, error : TPTC_ERROR) : Boolean;

Procedure ptc_error_handler(handler : TPTC_ERROR_HANDLER);
