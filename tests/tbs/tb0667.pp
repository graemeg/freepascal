{ %OPT=-Ootailrec }

program tb0667;

var
  stack: Pointer = Nil;

function Test1(var Values: array of LongInt; Res: LongInt): LongInt;
begin
  if not Assigned(stack) then
    stack := get_frame
  else if stack <> get_frame then
    Halt(1);
  if Length(Values) = 0 then
    Test1 := Res
  else
    Test1 := Test1(Values[0..High(Values) - 1], Res + Values[High(Values)]);
end;

function Test1(Values: array of LongInt): LongInt;
begin
  Test1 := Test1(Values, 0);
end;

function Test2(constref Values: array of LongInt; Res: LongInt): LongInt;
begin
  if not Assigned(stack) then
    stack := get_frame
  else if stack <> get_frame then
    Halt(3);
  if Length(Values) = 0 then
    Test2 := Res
  else
    Test2 := Test2(Values[0..High(Values) - 1], Res + Values[High(Values)]);
end;

function Test2(constref Values: array of LongInt): LongInt;
begin
  Test2 := Test2(Values, 0);
end;

function Test3(const Values: array of LongInt; Res: LongInt): LongInt;
begin
  if not Assigned(stack) then
    stack := get_frame
  else if stack <> get_frame then
    Halt(5);
  if Length(Values) = 0 then
    Test3 := Res
  else
    Test3 := Test3(Values[0..High(Values) - 1], Res + Values[High(Values)]);
end;

function Test3(const Values: array of LongInt): LongInt;
begin
  Test3 := Test3(Values, 0);
end;

function Test4(Values: array of LongInt; Res: LongInt): LongInt;
begin
  if not Assigned(stack) then
    stack := get_frame
  else if stack <> get_frame then
    Halt(7);
  if Length(Values) = 0 then
    Test4 := Res
  else
    Test4 := Test4(Values[0..High(Values) - 1], Res + Values[High(Values)]);
end;

function Test4(Values: array of LongInt): LongInt;
begin
  Test4 := Test4(Values, 0);
end;

begin
  if Test1([1, 2, 3, 4]) <> 10 then
    Halt(2);
  stack := Nil;
  if Test2([1, 2, 3, 4]) <> 10 then
    Halt(4);
  stack := Nil;
  if Test3([1, 2, 3, 4]) <> 10 then
    Halt(6);
  stack := Nil;
  if Test4([1, 2, 3, 4]) <> 10 then
    Halt(8);
  writeln('ok');
end.
