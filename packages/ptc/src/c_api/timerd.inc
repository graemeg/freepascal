{ setup }
Function ptc_timer_create : TPTC_TIMER;
Procedure ptc_timer_destroy(obj : TPTC_TIMER);

{ set time }
Procedure ptc_timer_set(obj : TPTC_TIMER; time : Double);

{ control }
Procedure ptc_timer_start(obj : TPTC_TIMER);
Procedure ptc_timer_stop(obj : TPTC_TIMER);

{ time data }
Function ptc_timer_time(obj : TPTC_TIMER) : Double;
Function ptc_timer_delta(obj : TPTC_TIMER) : Double;
Function ptc_timer_resolution(obj : TPTC_TIMER) : Double;

{ operators }
Procedure ptc_timer_assign(obj, timer : TPTC_TIMER);
Function ptc_timer_equals(obj, timer : TPTC_TIMER) : Boolean;
