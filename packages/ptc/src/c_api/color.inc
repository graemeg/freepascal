Function ptc_color_create : TPTC_COLOR;

Begin
  Try
    ptc_color_create := TPTC_COLOR(TPTCColor.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_create := Nil;
    End;
  End;
End;

Function ptc_color_create_indexed(index : Integer) : TPTC_COLOR;

Begin
  Try
    ptc_color_create_indexed := TPTC_COLOR(TPTCColor.Create(index));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_create_indexed := Nil;
    End;
  End;
End;

Function ptc_color_create_direct(r, g, b, a : Single) : TPTC_COLOR;

Begin
  Try
    ptc_color_create_direct := TPTC_COLOR(TPTCColor.Create(r, g, b, a));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_create_direct := Nil;
    End;
  End;
End;

Procedure ptc_color_destroy(obj : TPTC_COLOR);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCColor(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_color_index(obj : TPTC_COLOR) : Integer;

Begin
  Try
    ptc_color_index := TPTCColor(obj).index;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_index := 0;
    End;
  End;
End;

Function ptc_color_r(obj : TPTC_COLOR) : Single;

Begin
  Try
    ptc_color_r := TPTCColor(obj).r;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_r := 0;
    End;
  End;
End;

Function ptc_color_g(obj : TPTC_COLOR) : Single;

Begin
  Try
    ptc_color_g := TPTCColor(obj).g;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_g := 0;
    End;
  End;
End;

Function ptc_color_b(obj : TPTC_COLOR) : Single;

Begin
  Try
    ptc_color_b := TPTCColor(obj).b;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_b := 0;
    End;
  End;
End;

Function ptc_color_a(obj : TPTC_COLOR) : Single;

Begin
  Try
    ptc_color_a := TPTCColor(obj).a;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_a := 0;
    End;
  End;
End;

Function ptc_color_direct(obj : TPTC_COLOR) : Boolean;

Begin
  Try
    ptc_color_direct := TPTCColor(obj).direct;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_direct := False;
    End;
  End;
End;

Function ptc_color_indexed(obj : TPTC_COLOR) : Boolean;

Begin
  Try
    ptc_color_indexed := TPTCColor(obj).indexed;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_indexed := False;
    End;
  End;
End;

Procedure ptc_color_assign(obj, color : TPTC_COLOR);

Begin
  Try
    TPTCColor(obj).ASSign(TPTCColor(color));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_color_equals(obj, color : TPTC_COLOR) : Boolean;

Begin
  Try
    ptc_color_equals := TPTCColor(obj).Equals(TPTCColor(color));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_color_equals := False;
    End;
  End;
End;
