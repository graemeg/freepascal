{ %skiptarget=go32v2 }
{ %OPT=-CE }

{$mode objfpc}

uses
  sysutils;

const
  Program_has_errors : boolean = false;
  exception_called   : boolean = false;
  TestNumber : longint = 10000;

procedure test_exception(const s : string);
  begin
    if not(exception_called) then
      begin
        Writeln('Exception not called : ',s);
        Program_has_errors := true;
      end;
  end;

var
   i,j : longint;
   e : extended;
   exception_count,level : longint;
begin
   j:=0;
   i:=100;
   try
   exception_called:=false;
   j := i div j;
   except
     on e : exception do
       begin
         Writeln('First integer exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('First division by zero for integers');
   try
   exception_called:=false;
   j := i div j;
   except
     on e : exception do
       begin
         Writeln('Second integer exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('Second division by zero for integers');
   try
   exception_called:=false;
   e:=i/j;
   except
     on e : exception do
       begin
         Writeln('First real exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('First division by zero for reals');
   try
   exception_called:=false;
   e:=i/j;
   except
     on e : exception do
       begin
         Writeln('Second real exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('Second division by zero for reals');
   try
   exception_called:=false;
   j := i div j;
   except
     on e : exception do
       begin
         Writeln('exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('third division by zero for integers');
   exception_count:=0;
   level:=0;
   for j:=1 to TestNumber do
     begin
       try
         i:=0;
         inc(level);
         e:=j/i;
       except
         on e : exception do
           begin
             inc(exception_count);
             if level>1 then
               Writeln('exception overrun');
             dec(level);
           end;
       end;

     end;
   if exception_count<>TestNumber then
     begin
       program_has_errors:=true;
       Writeln('Could not generate ',TestNumber,' consecutive exceptions');
       Writeln('Only ',exception_count,' exceptions were generated');
     end
   else
     begin
       Writeln(TestNumber,' consecutive exceptions generated successfully');
     end;
   try
   exception_called:=false;
   i := -1;
   e := ln(i);
   except
     on e : exception do
       begin
         Writeln('exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('ln(-1)');
   try
   exception_called:=false;
   i := 0;
   e := ln(i);
   except
     on e : exception do
       begin
         Writeln('exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('ln(0)');
   try
   exception_called:=false;
   i := -1;
   e := sqrt(i);
   except
     on e : exception do
       begin
         Writeln('exception called ',e.message);
         exception_called:=true;
       end;
   end;
   test_exception('sqrt(-1)');
   if program_has_errors then
     Halt(1);
end.
