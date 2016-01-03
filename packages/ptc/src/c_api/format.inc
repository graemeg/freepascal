Function ptc_format_create : TPTC_FORMAT;

Begin
  Try
    ptc_format_create := TPTC_FORMAT(TPTCFormat.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_create := Nil;
    End;
  End;
End;

Function ptc_format_create_indexed(bits : Integer) : TPTC_FORMAT;

Begin
  Try
    ptc_format_create_indexed := TPTC_FORMAT(TPTCFormat.Create(bits));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_create_indexed := Nil;
    End;
  End;
End;

Function ptc_format_create_direct(bits : Integer; r, g, b, a : int32) : TPTC_FORMAT;

Begin
  Try
    ptc_format_create_direct := TPTC_FORMAT(TPTCFormat.Create(bits, r, g, b, a));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_create_direct := Nil;
    End;
  End;
End;

Procedure ptc_format_destroy(obj : TPTC_FORMAT);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCFormat(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_format_r(obj : TPTC_FORMAT) : int32;

Begin
  Try
    ptc_format_r := TPTCFormat(obj).r;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_r := 0;
    End;
  End;
End;

Function ptc_format_g(obj : TPTC_FORMAT) : int32;

Begin
  Try
    ptc_format_g := TPTCFormat(obj).g;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_g := 0;
    End;
  End;
End;

Function ptc_format_b(obj : TPTC_FORMAT) : int32;

Begin
  Try
    ptc_format_b := TPTCFormat(obj).b;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_b := 0;
    End;
  End;
End;

Function ptc_format_a(obj : TPTC_FORMAT) : int32;

Begin
  Try
    ptc_format_a := TPTCFormat(obj).a;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_a := 0;
    End;
  End;
End;

Function ptc_format_bits(obj : TPTC_FORMAT) : Integer;

Begin
  Try
    ptc_format_bits := TPTCFormat(obj).bits;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_bits := 0;
    End;
  End;
End;

Function ptc_format_bytes(obj : TPTC_FORMAT) : Integer;

Begin
  Try
    ptc_format_bytes := TPTCFormat(obj).bytes;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_bytes := 0;
    End;
  End;
End;

Function ptc_format_direct(obj : TPTC_FORMAT) : Boolean;

Begin
  Try
    ptc_format_direct := TPTCFormat(obj).direct;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_direct := False;
    End;
  End;
End;

Function ptc_format_indexed(obj : TPTC_FORMAT) : Boolean;

Begin
  Try
    ptc_format_indexed := TPTCFormat(obj).indexed;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_indexed := False;
    End;
  End;
End;

Procedure ptc_format_assign(obj, format : TPTC_FORMAT);

Begin
  Try
    TPTCFormat(obj).ASSign(TPTCFormat(format));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_format_equals(obj, format : TPTC_FORMAT) : Boolean;

Begin
  Try
    ptc_format_equals := TPTCFormat(obj).Equals(TPTCFormat(format));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_format_equals := False;
    End;
  End;
End;
