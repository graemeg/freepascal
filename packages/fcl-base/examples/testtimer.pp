{$mode objfpc}
{$H+}
program testtimer;

uses 
{$ifdef unix}
  cthreads,
{$endif}
  sysutils,classes,custapp,fptimer;

Type
  TTestTimerApp = Class(TCustomApplication)
  Private
    FTimer : TFPTimer;
    FCount : Integer;
    FTick : Integer;
<<<<<<< HEAD
<<<<<<< HEAD
    N : TDateTime;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public  
    Procedure DoRun; override;
    Procedure DoTick(Sender : TObject);
  end;

Procedure TTestTimerApp.DoRun; 

begin
  FTimer:=TFPTimer.Create(Self);
  FTimer.Interval:=100;
  FTimer.OnTimer:=@DoTick;
  FTimer.Enabled:=True;
  Try
    FTick:=0;
    FCount:=0;
<<<<<<< HEAD
<<<<<<< HEAD
    N:=Now;
    While (FCount<10) do
      begin
      Inc(FTick);
      Sleep(1);
      CheckSynchronize; // Needed, because we are not running in a GUI loop.
=======
=======
>>>>>>> origin/fixes_2_2
    While (FCount<10) do
      begin
      Inc(FTick);
      CheckSynchronize; // Needed, because we are not running in a GUI loop.
      Sleep(1);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      end;
  Finally
    FTimer.Enabled:=False;
    FreeAndNil(FTimer);
  end;
  Terminate;
end;

Procedure TTestTimerApp.DoTick(Sender : TObject);

<<<<<<< HEAD
<<<<<<< HEAD
Var
  D : TDateTime;

begin
  Inc(FCount);
  D:=Now-N;
  Writeln('Received timer event ',FCount,' after ',FTick,' ticks. (Elapsed time: ',FormatDateTime('ss.zzz',D),')');
  FTick:=0;
  N:=Now;
=======
=======
>>>>>>> origin/fixes_2_2
begin
  Inc(FCount);
  Writeln('Received timer event ',FCount,' after ',FTick,' ticks.');
  FTick:=0;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;
        

begin
  With TTestTimerApp.Create(Nil) do
    Try
      Run
    finally
      Free;
    end;  
end.