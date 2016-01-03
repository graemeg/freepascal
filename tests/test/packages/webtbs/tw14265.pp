{ %target=linux }
{ %opt=-Xt }

program phello;
{$linklib c}

{$packrecords c}

uses
  ctypes, unixtype, pthreads;

const N = 2;
var
  res:array[1..N] of Integer;

<<<<<<< HEAD
function Hello(arg: pointer): pointer; cdecl;
begin
//  writeln('Hello from thread #', PInteger(arg)^);
  res[PInteger(arg)^] := PInteger(arg)^;
  Hello := nil;
=======
function Hello(arg: pointer): longint; cdecl;
begin
//  writeln('Hello from thread #', PInteger(arg)^);
  res[PInteger(arg)^] := PInteger(arg)^;
  Hello := 0;
>>>>>>> origin/fixes_2.4
  pthread_exit(pointer(Hello));
end;

var
  i: Integer;
  ret: Pointer;
  arg: array[1..N] of Integer;
  threads: array[1..N] of TThreadID;
  attr: TThreadAttr;
begin
  Writeln('Testing simple thread creation');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  pthread_attr_init(@attr);
=======
  pthread_attr_init(attr);
>>>>>>> graemeg/cpstrnew
=======
  pthread_attr_init(attr);
>>>>>>> graemeg/cpstrnew
=======
  pthread_attr_init(attr);
>>>>>>> graemeg/cpstrnew
=======
  pthread_attr_init(attr);
>>>>>>> origin/cpstrnew
=======
  pthread_attr_init(attr);
>>>>>>> origin/fixes_2.4
  for i := 1 to N do
  begin
    Writeln('Creating thread #',i);
    arg[i] := i;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    if pthread_create(@threads[i], @attr, @Hello, @arg[i]) <> 0 then
=======
    if pthread_create(threads[i], attr, @Hello, @arg[i]) <> 0 then
>>>>>>> graemeg/cpstrnew
=======
    if pthread_create(threads[i], attr, @Hello, @arg[i]) <> 0 then
>>>>>>> graemeg/cpstrnew
=======
    if pthread_create(threads[i], attr, @Hello, @arg[i]) <> 0 then
>>>>>>> graemeg/cpstrnew
=======
    if pthread_create(threads[i], attr, @Hello, @arg[i]) <> 0 then
>>>>>>> origin/cpstrnew
=======
    if pthread_create(threads[i], attr, @Hello, @arg[i]) <> 0 then
>>>>>>> origin/fixes_2.4
      Writeln('Failed to create thread');
  end;
  for i := 1 to N do
  begin
    Write('Waiting for thread #',i, ' ... ');
    pthread_join(threads[i], ret);
    Writeln('result: ', res[i]);
  end;
end.


