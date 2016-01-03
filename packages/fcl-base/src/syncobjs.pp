{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1998 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$h+}
unit syncobjs;

interface

uses
  sysutils;

type
  PSecurityAttributes = Pointer;
  TEventHandle = Pointer;

const
  INFINITE = Cardinal(-1);

type
   TWaitResult = (wrSignaled, wrTimeout, wrAbandoned, wrError);

   TSynchroObject = class(TObject)
      procedure Acquire;virtual;
      procedure Release;virtual;
   end;

   TCriticalSection = class(TSynchroObject)
   private
      CriticalSection : TRTLCriticalSection;
   public
      procedure Acquire;override;
      procedure Release;override;
      procedure Enter;
      procedure Leave;
      constructor Create;
      destructor Destroy;override;
   end;

   THandleObject = class(TSynchroObject)
   protected
      FHandle : TEventHandle;
      FLastError : Integer;
   public
      destructor destroy;override;
      property Handle : TEventHandle read FHandle;
      property LastError : Integer read FLastError;
   end;

   TEventObject = class(THandleObject)
   private
      FManualReset: Boolean;
   public
      constructor Create(EventAttributes : PSecurityAttributes;
        AManualReset,InitialState : Boolean;const Name : string);
      destructor destroy; override;
      procedure ResetEvent;
      procedure SetEvent;
      function WaitFor(Timeout : Cardinal) : TWaitResult;
      Property ManualReset : Boolean read FManualReset;
   end;

   TEvent = TEventObject;

   TSimpleEvent = class(TEventObject)
      constructor Create;
   end;

implementation

{ ---------------------------------------------------------------------
    Real syncobjs implementation
  ---------------------------------------------------------------------}

procedure TSynchroObject.Acquire;
begin
end;

procedure TSynchroObject.Release;
begin
end;

procedure TCriticalSection.Enter;
begin
  Acquire;
end;

procedure TCriticalSection.Leave;
begin
  Release;
end;


procedure TCriticalSection.Acquire;

begin
  EnterCriticalSection(CriticalSection);
end;

procedure TCriticalSection.Release;

begin
  LeaveCriticalSection(CriticalSection);
end;

constructor TCriticalSection.Create;

begin
  Inherited Create;
  InitCriticalSection(CriticalSection);
end;

destructor TCriticalSection.Destroy;

begin
  DoneCriticalSection(CriticalSection);
end;

destructor THandleObject.destroy;

begin
end;

constructor TEventObject.Create(EventAttributes : PSecurityAttributes;
  AManualReset,InitialState : Boolean;const Name : string);

begin
  FHandle := BasicEventCreate(EventAttributes, AManualReset, InitialState, Name);
  FManualReset:=AManualReset;
end;

destructor TEventObject.destroy;

begin
  BasicEventDestroy(Handle);
end;

procedure TEventObject.ResetEvent;

begin
  BasicEventResetEvent(Handle);
end;

procedure TEventObject.SetEvent;

begin
  BasicEventSetEvent(Handle);
end;


function TEventObject.WaitFor(Timeout : Cardinal) : TWaitResult;

begin
  Result := TWaitResult(basiceventWaitFor(Timeout, Handle));
  if Result = wrError then
    FLastError := GetLastOSError;
end;

constructor TSimpleEvent.Create;

begin
  inherited Create(nil, True, False, '');
end;

end.
