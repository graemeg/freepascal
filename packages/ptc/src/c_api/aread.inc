{ setup }
Function ptc_area_create(left, top, right, bottom : Integer) : TPTC_AREA;
Procedure ptc_area_destroy(obj : TPTC_AREA);

{ data access }
Function ptc_area_left(obj : TPTC_AREA) : Integer;
Function ptc_area_top(obj : TPTC_AREA) : Integer;
Function ptc_area_right(obj : TPTC_AREA) : Integer;
Function ptc_area_bottom(obj : TPTC_AREA) : Integer;
Function ptc_area_width(obj : TPTC_AREA) : Integer;
Function ptc_area_height(obj : TPTC_AREA) : Integer;

{ operators }
Procedure ptc_area_assign(obj, area : TPTC_AREA);
Function ptc_area_equals(obj, area : TPTC_AREA) : Boolean;
