Procedure ptc_clipper_clip(area, clip, clipped : TPTC_AREA);

Var
  tmp : TPTCArea;

Begin
  Try
    tmp := TPTCClipper.clip(TPTCArea(area), TPTCArea(clip));
    Try
      TPTCArea(clipped).ASSign(tmp);
    Finally
      tmp.Destroy;
    End;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
    End;
  End;
End;

Procedure ptc_clipper_clip_complex(source, clip_source, clipped_source, destination, clip_destination, clipped_destination : TPTC_AREA);

Begin
  Try
    TPTCClipper.clip(TPTCArea(source), TPTCArea(clip_source), TPTCArea(clipped_source), TPTCArea(destination), TPTCArea(clip_destination), TPTCArea(clipped_destination));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
    End;
  End;
End;
