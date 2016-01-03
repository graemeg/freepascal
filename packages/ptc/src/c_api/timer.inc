Function ptc_timer_create : TPTC_TIMER;

Begin
  Try
    ptc_timer_create := TPTC_TIMER(TPTCTimer.Create);
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_timer_create := Nil;
    End;
  End;
End;

Procedure ptc_timer_destroy(obj : TPTC_TIMER);

Begin
  If obj = Nil Then
    Exit;
  Try
    TPTCTimer(obj).Destroy;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_timer_set(obj : TPTC_TIMER; time : Double);

Begin
  Try
    TPTCTimer(obj).settime(time);
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_timer_start(obj : TPTC_TIMER);

Begin
  Try
    TPTCTimer(obj).start;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Procedure ptc_timer_stop(obj : TPTC_TIMER);

Begin
  Try
    TPTCTimer(obj).stop;
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_timer_time(obj : TPTC_TIMER) : Double;

Begin
  Try
    ptc_timer_time := TPTCTimer(obj).time;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_timer_time := 0;
    End;
  End;
End;

Function ptc_timer_delta(obj : TPTC_TIMER) : Double;

Begin
  Try
    ptc_timer_delta := TPTCTimer(obj).delta;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_timer_delta := 0;
    End;
  End;
End;

Function ptc_timer_resolution(obj : TPTC_TIMER) : Double;

Begin
  Try
    ptc_timer_resolution := TPTCTimer(obj).resolution;
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_timer_resolution := 0;
    End;
  End;
End;

Procedure ptc_timer_assign(obj, timer : TPTC_TIMER);

Begin
  Try
    TPTCTimer(obj).ASSign(TPTCTimer(timer));
  Except
    On error : TPTCError Do
      ptc_exception_handle(error);
  End;
End;

Function ptc_timer_equals(obj, timer : TPTC_TIMER) : Boolean;

Begin
  Try
    ptc_timer_equals := TPTCTimer(obj).equals(TPTCTimer(timer));
  Except
    On error : TPTCError Do
    Begin
      ptc_exception_handle(error);
      ptc_timer_equals := False;
    End;
  End;
End;
