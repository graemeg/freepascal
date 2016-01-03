{ setup }
Function ptc_clear_create : TPTC_CLEAR;
Procedure ptc_clear_destroy(obj : TPTC_CLEAR);

{ request clear }
Procedure ptc_clear_request(obj : TPTC_CLEAR; format : TPTC_FORMAT);

{ clear pixels }
Procedure ptc_clear_clear(obj : TPTC_CLEAR; pixels : Pointer; x, y, width, height, pitch : Integer; color : TPTC_COLOR);
