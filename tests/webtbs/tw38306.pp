{ %OPT=-gh }
{$mode objfpc}
program gqueue_test;

uses
  gqueue;

type
  TIntQueue = specialize TQueue<Integer>;

var
  IntQueue: TIntQueue;
  PushCnt: Integer;

procedure Push2Pop1;
var
  i: Integer;
begin
  for i:= 0 to 1000000 do begin
    IntQueue.Push(PushCnt);
    inc(PushCnt);
    IntQueue.Push(PushCnt);
    inc(PushCnt);
    IntQueue.Pop();
  end;
end;

var
  i: Integer;
begin
  try
    IntQueue:= TIntQueue.Create;
    Push2Pop1;
    WriteLn('Ready');
  finally
    IntQueue.Free;
  end;
end.

