{ setup }
Function ptc_mode_create(width, height : Integer; format : TPTC_FORMAT) : TPTC_MODE;
Function ptc_mode_create_invalid : TPTC_MODE;
Procedure ptc_mode_destroy(obj : TPTC_MODE);

{ valid mode flag }
Function ptc_mode_valid(obj : TPTC_MODE) : Boolean;

{ data access }
Function ptc_mode_width(obj : TPTC_MODE) : Integer;
Function ptc_mode_height(obj : TPTC_MODE) : Integer;
Function ptc_mode_format(obj : TPTC_MODE) : TPTC_FORMAT;

{ operators }
Procedure ptc_mode_assign(obj, mode : TPTC_MODE);
Function ptc_mode_equals(obj, mode : TPTC_MODE) : Boolean;
