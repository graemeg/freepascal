{$mode objfpc}

uses
{$ifdef unix}
 cthreads,
{$endif}
  sysutils,
  classes;

Const
        wrSignaled = 0;
        wrTimeout  = 1;
        wrAbandoned= 2;
        wrError    = 3;

type
  tc = class(tthread)
    procedure execute; override;
  end;

  torder = (o_destroy, o_post, o_sleeppost, o_waittimeoutabandon, o_waittimeoutsignal);
  thelper = class(tthread)
   private
    forder: torder;
   public
    constructor create(order: torder);
    procedure execute; override;
  end;

var
  event: pEventState;
  waiting: boolean;

constructor thelper.create(order: torder);
  begin
    forder:=order;
    inherited create(false);
  end;

procedure thelper.execute;
var
  res: longint;
begin
  case forder of
    o_destroy:
      basiceventdestroy(event);
    o_post:
      basiceventsetevent(event);
    o_sleeppost:
      begin
        sleep(1000);
        basiceventsetevent(event);
      end;
    o_waittimeoutabandon:
      begin
        res:=basiceventWaitFor(1000,event);
        if (res<>wrAbandoned) then
          begin
            writeln('error 1');
            halt(1);
          end;
      end;
    o_waittimeoutsignal:
      begin
        res:=basiceventWaitFor(1000,event);
        if (res<>wrSignaled) then
          begin
            writeln('error 2');
            halt(2);
          end;
      end;
  end;
end;

procedure tc.execute;
begin
  { make sure we don't exit before this thread has initialised, since    }
  { it can allocate memory in its initialisation, which would cause      }
  { problems for heaptrc as it goes over the memory map in its exit code }
  waiting:=true;
  { avoid deadlocks/bugs from causing this test to never quit }
  sleep(1000*10);
  writeln('error 3');
  halt(3);
end;

var
  help: thelper;
begin
  waiting:=false;
  tc.create(false);
  event := BasicEventCreate(nil,false,false,'bla');
<<<<<<< HEAD
<<<<<<< HEAD
  basiceventSetEvent(event);
  if (basiceventWaitFor(cardinal(-1),event) <> wrSignaled) then
    begin
      writeln('error 4');
      halt(4);
    end;
  basiceventSetEvent(event);
  if (basiceventWaitFor(1000,event) <> wrSignaled) then
    begin
=======
=======
>>>>>>> origin/fixes_2_2
  basiceventSetEvent(event);
  if (basiceventWaitFor(cardinal(-1),event) <> wrSignaled) then
    begin
      writeln('error 4');
      halt(4);
    end;
  basiceventSetEvent(event);
  if (basiceventWaitFor(1000,event) <> wrSignaled) then
    begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      writeln('error 5');
      halt(5);
    end;
  { shouldn't change anything }
  basiceventResetEvent(event);
  basiceventSetEvent(event);
  { shouldn't change anything }
  basiceventSetEvent(event);
  if (basiceventWaitFor(cardinal(-1),event) <> wrSignaled) then
<<<<<<< HEAD
<<<<<<< HEAD
    begin
      writeln('error 6');
      halt(6);
    end;

  { make sure the two BasicSetEvents aren't cumulative }
  if (basiceventWaitFor(1000,event) <> wrTimeOut) then
    begin
      writeln('error 7');
      halt(7);
    end;

{$ifdef windows}
  { On windows event can not be "abandoned". Skipping this test }
  basiceventdestroy(event);
{$else}
 {$IFDEF os2}
  { On OS/2 event can not be "abandoned" either. Skipping this test }
  basiceventdestroy(event);
 {$ELSE OS2}
  help:=thelper.create(o_waittimeoutabandon);
  sleep(100); // make sure that thread has been started
  basiceventdestroy(event);
  help.waitfor;
  help.free;
 {$ENDIF OS2}
{$endif}

  event := BasicEventCreate(nil,false,false,'bla');
  help:=thelper.create(o_waittimeoutsignal);
  basiceventSetEvent(event);
  help.waitfor;
  help.free;
  basiceventdestroy(event);
=======
    begin
      writeln('error 6');
      halt(6);
    end;

  { make sure the two BasicSetEvents aren't cumulative }
  if (basiceventWaitFor(1000,event) <> wrTimeOut) then
    begin
      writeln('error 7');
      halt(7);
    end;

  help:=thelper.create(o_waittimeoutabandon);
  basiceventdestroy(event);
  help.waitfor;
  help.free;

  event := BasicEventCreate(nil,false,false,'bla');
  help:=thelper.create(o_waittimeoutsignal);
  basiceventSetEvent(event);
  help.waitfor;
  help.free;
  basiceventdestroy(event);
>>>>>>> graemeg/fixes_2_2
=======
    begin
      writeln('error 6');
      halt(6);
    end;

  { make sure the two BasicSetEvents aren't cumulative }
  if (basiceventWaitFor(1000,event) <> wrTimeOut) then
    begin
      writeln('error 7');
      halt(7);
    end;

  help:=thelper.create(o_waittimeoutabandon);
  basiceventdestroy(event);
  help.waitfor;
  help.free;

  event := BasicEventCreate(nil,false,false,'bla');
  help:=thelper.create(o_waittimeoutsignal);
  basiceventSetEvent(event);
  help.waitfor;
  help.free;
  basiceventdestroy(event);
>>>>>>> origin/fixes_2_2

  while not waiting do
    sleep(20);
end.
