Function ptc_area_create(left, top, right, bottom : Integer) : TPTC_AREA;

Begin
  Try
    ptc_area_create := TPTC_AREA(TPTCArea.Create(left, top, right, bottom));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_create := Nil;
    End;
  End;
End;

Procedure ptc_area_destroy(obj : TPTC_AREA);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCArea(obj).Destroy;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
    End;
  End;
End;

Function ptc_area_left(obj : TPTC_AREA) : Integer;

Begin
  Try
    ptc_area_left := TPTCArea(obj).left;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_left := 0;
    End;
  End;
End;

Function ptc_area_top(obj : TPTC_AREA) : Integer;

Begin
  Try
    ptc_area_top := TPTCArea(obj).top;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_top := 0;
    End;
  End;
End;

Function ptc_area_right(obj : TPTC_AREA) : Integer;

Begin
  Try
    ptc_area_right := TPTCArea(obj).right;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_right := 0;
    End;
  End;
End;

Function ptc_area_bottom(obj : TPTC_AREA) : Integer;

Begin
  Try
    ptc_area_bottom := TPTCArea(obj).bottom;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_bottom := 0;
    End;
  End;
End;

Function ptc_area_width(obj : TPTC_AREA) : Integer;

Begin
  Try
    ptc_area_width := TPTCArea(obj).width;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_width := 0;
    End;
  End;
End;

Function ptc_area_height(obj : TPTC_AREA) : Integer;

Begin
  Try
    ptc_area_height := TPTCArea(obj).height;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_height := 0;
    End;
  End;
End;

Procedure ptc_area_assign(obj, area : TPTC_AREA);

Begin
  Try
    TPTCArea(obj).ASSign(TPTCArea(area));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
    End;
  End;
End;

Function ptc_area_equals(obj, area : TPTC_AREA) : Boolean;

Begin
  Try
    ptc_area_equals := TPTCArea(obj).Equals(TPTCArea(area));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_area_equals := False;
    End;
  End;
End;

