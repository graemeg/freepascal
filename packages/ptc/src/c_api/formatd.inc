{ setup }
Function ptc_format_create : TPTC_FORMAT;
Function ptc_format_create_indexed(bits : Integer) : TPTC_FORMAT;
Function ptc_format_create_direct(bits : Integer; r, g, b, a : int32) : TPTC_FORMAT;
Procedure ptc_format_destroy(obj : TPTC_FORMAT);

{ data access }
Function ptc_format_r(obj : TPTC_FORMAT) : int32;
Function ptc_format_g(obj : TPTC_FORMAT) : int32;
Function ptc_format_b(obj : TPTC_FORMAT) : int32;
Function ptc_format_a(obj : TPTC_FORMAT) : int32;
Function ptc_format_bits(obj : TPTC_FORMAT) : Integer;
Function ptc_format_bytes(obj : TPTC_FORMAT) : Integer;
Function ptc_format_direct(obj : TPTC_FORMAT) : Boolean;
Function ptc_format_indexed(obj : TPTC_FORMAT) : Boolean;

{ operators }
Procedure ptc_format_assign(obj, format : TPTC_FORMAT);
Function ptc_format_equals(obj, format : TPTC_FORMAT) : Boolean;
