{ setup }
Function ptc_color_create : TPTC_COLOR;
Function ptc_color_create_indexed(index : Integer) : TPTC_COLOR;
Function ptc_color_create_direct(r, g, b, a : Single) : TPTC_COLOR;
Procedure ptc_color_destroy(obj : TPTC_COLOR);

{ data access }
Function ptc_color_index(obj : TPTC_COLOR) : Integer;
Function ptc_color_r(obj : TPTC_COLOR) : Single;
Function ptc_color_g(obj : TPTC_COLOR) : Single;
Function ptc_color_b(obj : TPTC_COLOR) : Single;
Function ptc_color_a(obj : TPTC_COLOR) : Single;
Function ptc_color_direct(obj : TPTC_COLOR) : Boolean;
Function ptc_color_indexed(obj : TPTC_COLOR) : Boolean;

{ operators }
Procedure ptc_color_assign(obj, color : TPTC_COLOR);
Function ptc_color_equals(obj, color : TPTC_COLOR) : Boolean;
