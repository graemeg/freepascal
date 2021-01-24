{%skiptarget=$nothread }
{ %TIMEOUT=105 }
{$mode objfpc}{$h+}

uses
{$ifdef UNIX}
  cthreads,
{$endif}
  sysutils,
  classes;

const
  fifolength = 1024;
type
  tpair = class;

  tproducethread = class(tthread)
    running: boolean;
    pair: tpair;
    constructor create(apair: tpair);
    procedure execute; override;
  end;

  tconsumethread = class(tthread)
    running: boolean;
    pair: tpair;
    constructor create(apair: tpair);
    procedure execute; override;
  end;

  tpair = class(tobject)
  public
    readindex: integer;
    writeindex: integer;
    fifo: array[0..fifolength-1] of pointer;
    shared: pointer;
    freefifolock: trtlcriticalsection;
    produce_thread: tproducethread;
    consume_thread: tconsumethread;

    constructor create;
    destructor destroy; override;

    procedure resume;
    procedure waitfor;
  end;

var
  done: boolean;

constructor tproducethread.create(apair: tpair);
begin
  pair := apair;
  inherited create(false);
end;

constructor tconsumethread.create(apair: tpair);
begin
  pair := apair;
  inherited create(false);
end;

constructor tpair.create;
begin
  filldword(fifo, sizeof(fifo) div sizeof(dword), 0);
  readindex := 0;
  writeindex := 0;
  initcriticalsection(freefifolock);
  produce_thread := tproducethread.create(self);
  consume_thread := tconsumethread.create(self);
end;

destructor tpair.destroy;
begin
  produce_thread.free;
  consume_thread.free;
  donecriticalsection(freefifolock);
end;

procedure tpair.resume;
begin
  produce_thread.resume;
  consume_thread.resume;
end;

procedure tpair.waitfor;
begin
  produce_thread.waitfor;
  consume_thread.waitfor;
end;

type
  ttestarray = array[0..31] of pointer;

procedure exercise_heap(var p: ttestarray; var i, j: integer);
begin
  if p[i] = nil then
    p[i] := getmem(((j*11) mod 532)+8)
  else begin
    freemem(p[i]);
    p[i] := nil;
  end;
  inc(i);
  if i >= 32 then
    dec(i, 32);
  inc(j, 13);
  if j >= 256 then
    dec(j, 256);
end;

procedure freearray(p: ppointer; count: integer);
var
  i: integer;
begin
  for i := 0 to count-1 do
  begin
    freemem(p[i]);
    p[i] := nil;
  end;
end;

procedure producer(pair: tpair);
var
  p: ttestarray;
  i, j, k: longint;
begin
  done := false;
  filldword(p, sizeof(p) div sizeof(dword), 0);
  i := 0;
  j := 0;
  k := 0;
  while not done do
  begin
    if ((pair.writeindex+1) mod fifolength) <> pair.readindex then
    begin
      { counterpart for the writebarrier in the consumer: ensure that we see
        the write to pair.fifo[pair.readindex] now that we've seen the write
        to pair.readindex }
      readbarrier;
      freemem(pair.fifo[pair.writeindex]);
      pair.fifo[pair.writeindex] := getmem(((pair.writeindex*17) mod 520)+8);
      writebarrier;
      pair.writeindex := (pair.writeindex + 1) mod 1024;
    end else begin
      exercise_heap(p,i,j);
      inc(k);
      if k = 100 then
      begin
        k := 0;
        ThreadSwitch;
      end;
    end;
  end;
  freearray(p, sizeof(p) div sizeof(pointer));
  entercriticalsection(pair.freefifolock);
  sleep(200);
  freearray(pair.fifo, sizeof(pair.fifo) div sizeof(pointer));
  freemem(pair.shared);
  leavecriticalsection(pair.freefifolock);
end;

procedure consumer(pair: tpair);
var
  p: ttestarray;
  i, j, k: longint;
begin
  done := false;
  filldword(p, sizeof(p) div sizeof(dword), 0);
  i := 0;
  j := 0;
  k := 0;
  entercriticalsection(pair.freefifolock);
  while not done do
  begin
    if pair.readindex <> pair.writeindex then
    begin
      { counterpart for the writebarrier in the producer: ensure that we see
        the write to pair.fifo[pair.writeindex] now that we've seen the write
        to pair.writeindex }
      readbarrier;
      freemem(pair.fifo[pair.readindex]);
      pair.fifo[pair.readindex] := getmem(((pair.writeindex*17) mod 520)+8);
      writebarrier;
      pair.readindex := (pair.readindex + 1) mod fifolength;
    end else begin
      exercise_heap(p,i,j);
      inc(k);
      if k = 100 then
      begin
        k := 0;
        ThreadSwitch;
      end;
    end;
  end;
  pair.shared := getmem(12);
  leavecriticalsection(pair.freefifolock);
  freearray(p, sizeof(p) div sizeof(pointer));
end;

procedure tproducethread.execute;
begin
  running:=true;
  producer(pair);
end;

procedure tconsumethread.execute;
begin
  running:=true;
  consumer(pair);
end;

const
  numpairs = 2;
var
  pairs: array[1..numpairs] of tpair;
  i, iter, num_iterations: integer;
begin
  num_iterations := 20;
  if paramcount > 0 then
    num_iterations := strtointdef(paramstr(1), num_iterations);
  for iter := 1 to num_iterations do
  begin
    done := false;
    for i := low(pairs) to high(pairs) do
      pairs[i] := tpair.create;
    for i := low(pairs) to high(pairs) do
      pairs[i].resume;

    { wait till all threads are really resumed }
    for i :=  low(pairs) to high(pairs) do
      while not(pairs[i].produce_thread.running) or not(pairs[i].consume_thread.running) do
        sleep(100);

    done := true;
    for i := low(pairs) to high(pairs) do
    begin
      pairs[i].waitfor;
      pairs[i].free;
    end;
  end;
end.
