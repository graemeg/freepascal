Function ptc_copy_create : TPTC_COPY;

Begin
  Try
    ptc_copy_create := TPTC_COPY(TPTCCopy.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_copy_create := Nil;
    End;
  End;
End;

Procedure ptc_copy_destroy(obj : TPTC_COPY);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCCopy(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_copy_request(obj : TPTC_COPY; source, destination : TPTC_FORMAT);

Begin
  Try
    TPTCCopy(obj).request(TPTCFormat(source), TPTCFormat(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_copy_palette(obj : TPTC_COPY; source, destination : TPTC_PALETTE);

Begin
  Try
    TPTCCopy(obj).palette(TPTCPalette(source), TPTCPalette(destination));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_copy_copy(obj : TPTC_COPY; source_pixels : Pointer; source_x, source_y, source_width, source_height, source_pitch : Integer;
                        destination_pixels : Pointer; destination_x, destination_y, destination_width, destination_height, destination_pitch : Integer);

Begin
  Try
    TPTCCopy(obj).copy(source_pixels, source_x, source_y, source_width, source_height, source_pitch, destination_pixels, destination_x, destination_y, destination_width, destination_height, destination_pitch);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_copy_option(obj : TPTC_COPY; option : String) : Boolean;

Begin
  Try
    TPTCCopy(obj).option(option);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_copy_option := False;
    End;
  End;
End;
