{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by Michael Van Canneyt.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  A generic timer component. Can be used in GUI and non-GUI apps.
  Based heavily on an idea by Graeme Geldenhuys, extended so
  the tick mechanism is pluggable.
  
  Note that the system implementation will only work for timers
  in the main thread, as it uses synchronize to do the job.
  You need to enable threads in your application for the system
  implementation to work.
  
  A nice improvement would be an implementation that works
  in all threads, such as the threadedtimer of IBX for linux.
}

unit fpTimer;

{$mode objfpc}{$H+}

interface

uses
  Classes;

type
  TFPTimerDriver = Class;
  
  TFPCustomTimer = class(TComponent)
  private
    FInterval: Integer;
    FDriver : TFPTimerDriver;
    FOnTimer: TNotifyEvent;
    FContinue: Boolean;
    FRunning: Boolean;
    FEnabled: Boolean;
    procedure   SetEnabled(Value: Boolean );
  protected
    property  Continue: Boolean read FContinue write FContinue;
    procedure Timer; virtual;
    Function CreateTimerDriver : TFPTimerDriver;
  public
    Constructor Create(AOwner: TComponent); override;
    Destructor Destroy; override;
    procedure StartTimer; virtual;
    procedure StopTimer; virtual;
  protected
    property Enabled: Boolean read FEnabled write SetEnabled;
    property Interval: Integer read FInterval write FInterval;
    property OnTimer: TNotifyEvent read FOnTimer write FOnTimer;
  end;

  TFPTimer = Class(TFPCustomTimer)
  Published
    Property Enabled;
    Property Interval;
    Property OnTimer;
  end;  

  TFPTimerDriver = Class(TObject)
<<<<<<< HEAD
<<<<<<< HEAD
  Protected
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FTimer : TFPCustomTimer;
  Public
    Constructor Create(ATimer : TFPCustomTimer); virtual;
    Procedure StartTimer; virtual; abstract;
    Procedure StopTimer; virtual; abstract;
    Property Timer : TFPCustomTimer Read FTimer;
  end;
  TFPTimerDriverClass = Class of TFPTimerDriver;

Var
  DefaultTimerDriverClass : TFPTimerDriverClass = Nil;

implementation

uses
  SysUtils;

{ ---------------------------------------------------------------------
    TFPTimer
  ---------------------------------------------------------------------}

constructor TFPCustomTimer.Create(AOwner: TComponent);
begin
  inherited;
  FDriver:=CreateTimerDriver;
end;

destructor TFPCustomTimer.Destroy;

begin
  If FEnabled then
    StopTimer;
  FDriver.FTimer:=Nil;  
  FreeAndNil(FDriver);
  Inherited;
end;


Function TFPCustomTimer.CreateTimerDriver : TFPTimerDriver;

begin
  Result:=DefaultTimerDriverClass.Create(Self);
end;

procedure TFPCustomTimer.SetEnabled(Value: Boolean);
begin
  if Value <> FEnabled then
    begin
    if Value then
      StartTimer
    else
      StopTimer;
    end;
end;

procedure TFPCustomTimer.StartTimer;
begin
  If FEnabled then
    Exit;
  FEnabled:=True;
  FContinue:=True;  
  If Not (csDesigning in ComponentState) then  
    FDriver.StartTimer;
end;

procedure TFPCustomTimer.StopTimer;
begin
  If Not FEnabled then 
    Exit;
  FEnabled:=False;
  FContinue:=False;  
  FDriver.StopTimer;
end;

procedure TFPCustomTimer.Timer;

begin
  { We check on FEnabled: If by any chance a tick comes in after it was
    set to false, the user won't notice, since no event is triggered.}
  If FEnabled and Assigned(FOnTimer) then
    FOnTimer(Self);
end;

{ ---------------------------------------------------------------------
  TFPTimerDriver
  ---------------------------------------------------------------------}
  

Constructor TFPTimerDriver.Create(ATimer : TFPCustomTimer);

begin
  FTimer:=ATimer;
end;


{ ---------------------------------------------------------------------
    Default implementation. Threaded timer, one thread per timer.
  ---------------------------------------------------------------------}
  
Type
  TFPTimerThread = class(TThread)
  private
    FTimerDriver: TFPTimerDriver;
    Function Timer : TFPCustomTimer;
  public
    procedure Execute; override;
    constructor CreateTimerThread(ATimerDriver: TFPTimerDriver);
  end;

  TFPThreadedTimerDriver = Class(TFPTimerDriver)
  Private
    FThread : TFPTimerThread;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure DoNilTimer(Sender : TObject);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  Public
    Procedure StartTimer; override;
    Procedure StopTimer; override;
  end;

function _GetTickCount: Cardinal;
begin
  Result := Cardinal(Trunc(Now * 24 * 60 * 60 * 1000));
end;

{ ---------------------------------------------------------------------
    TFPTimerThread
  ---------------------------------------------------------------------}
  
constructor TFPTimerThread.CreateTimerThread(ATimerDriver: TFPTimerDriver);
begin
  inherited Create(True);
  FTimerDriver:=ATimerDriver;
  FreeOnTerminate := True;
end;

Function TFPTimerThread.Timer : TFPCustomTimer;

begin
  If Assigned(FTimerDriver) Then
    Result:=FTimerDriver.FTimer
  else
    Result:=Nil;  
end;

procedure TFPTimerThread.Execute;
var
  SleepTime: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
  S,Last: Cardinal;
=======
  Last: Cardinal;
>>>>>>> graemeg/fixes_2_2
=======
  Last: Cardinal;
>>>>>>> origin/fixes_2_2
  T : TFPCustomTimer;
  
begin
  while Not Terminated do
    begin
    Last := _GetTickCount;
    T:=Timer;
    If Assigned(T) then
      begin
      SleepTime := T.FInterval - (_GetTickCount - Last);
      if SleepTime < 10 then
        SleepTime := 10;
<<<<<<< HEAD
<<<<<<< HEAD
      Repeat  
        S:=5;
        If S>SleepTime then
          S:=SleepTime;
        Sleep(S);
        Dec(Sleeptime,S);
      until (SleepTime<=0) or Terminated;
      T:=Timer;
      If Assigned(T) and not terminated then
=======
      Sleep(SleepTime);
      T:=Timer;
      If Assigned(T) then
>>>>>>> graemeg/fixes_2_2
=======
      Sleep(SleepTime);
      T:=Timer;
      If Assigned(T) then
>>>>>>> origin/fixes_2_2
        Synchronize(@T.Timer);
      end
    else
      Terminate;  
    end;
end;

{ ---------------------------------------------------------------------
    TFPThreadedTimerDriver
  ---------------------------------------------------------------------}

<<<<<<< HEAD
<<<<<<< HEAD
Procedure TFPThreadedTimerDriver.DoNilTimer(Sender : TObject);

begin
  FThread:=Nil;
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Procedure TFPThreadedTimerDriver.StartTimer; 

begin
  FThread:=TFPTimerThread.CreateTimerThread(Self);
<<<<<<< HEAD
<<<<<<< HEAD
  FThread.OnTerminate:=@DoNilTimer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FThread.Start;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  FThread.Resume;
>>>>>>> graemeg/cpstrnew
=======
  FThread.Resume;
>>>>>>> graemeg/fixes_2_2
=======
  FThread.Resume;
>>>>>>> origin/fixes_2_2
end;

Procedure TFPThreadedTimerDriver.StopTimer;
begin
  FThread.FTimerDriver:=Nil;
  FThread.Terminate; // Will free itself.
<<<<<<< HEAD
<<<<<<< HEAD
  CheckSynchronize; // make sure thread is not stuck at synchronize call.
  If Assigned(FThread) then
    Fthread.WaitFor;  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;


Initialization
  DefaultTimerDriverClass:=TFPThreadedTimerDriver;
end.

