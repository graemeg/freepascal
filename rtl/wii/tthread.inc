{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2002 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{****************************************************************************}
{*                             TThread                                      *}
{****************************************************************************}

{$WARNING This file is only a stub, and will not work!}

const
 ThreadCount: longint = 0;

(* Implementation of exported functions *)

procedure AddThread (T: TThread);
begin
 Inc (ThreadCount);
end;


procedure RemoveThread (T: TThread);
begin
 Dec (ThreadCount);
end;


procedure TThread.CallOnTerminate;
begin
 FOnTerminate (Self);
end;


function TThread.GetPriority: TThreadPriority;
begin
  result := tpNormal;
end;


procedure TThread.SetPriority(Value: TThreadPriority);
begin

end;


procedure TThread.SetSuspended(Value: Boolean);
begin
 if Value <> FSuspended then
 begin
  if Value then Suspend else Resume;
 end;
end;


procedure TThread.DoTerminate;
begin
 if Assigned (FOnTerminate) then Synchronize (@CallOnTerminate);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TThread.SysCreate(CreateSuspended: Boolean;
                            const StackSize: SizeUInt);
var
  Flags: cardinal;
begin
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
constructor TThread.Create(CreateSuspended: Boolean;
                           const StackSize: SizeUInt = DefaultStackSize);
var
  Flags: cardinal;
begin
  inherited Create;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  AddThread (Self);
end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TThread.SysDestroy;
=======
destructor TThread.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TThread.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TThread.Destroy;
>>>>>>> graemeg/cpstrnew
=======
destructor TThread.Destroy;
>>>>>>> origin/cpstrnew
begin
 if not FFinished and not Suspended then
 begin
  Terminate;
  WaitFor;
 end;
end;

procedure TThread.Resume;
begin

end;


procedure TThread.Suspend;
begin

end;


procedure TThread.Terminate;
begin
 FTerminated := true;
end;


function TThread.WaitFor: Integer;
begin
  result := -1;
end;


