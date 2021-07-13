{$mode objfpc} {$h+} {$coperators on}

var
    a: array[0 .. 3] of uint32;
    whatToIncrementNext: SizeUint;

    procedure Reset;
    begin
        whatToIncrementNext := 0;
        FillChar(pUint32(a)^, length(a) * sizeof(a[0]), 0);
        writeln('Before: ', a[0], ' ', a[1], ' ', a[2], ' ', a[3], LineEnding);
    end;

    function NextIndex: SizeUint;
    begin
        result := whatToIncrementNext;
        writeln('Incrementing ', whatToIncrementNext, 'th element');
        whatToIncrementNext := (whatToIncrementNext + 1) mod length(a);
    end;

    function NextPtr: pUint32;
    begin
        result := @a[whatToIncrementNext];
        writeln('Incrementing ', whatToIncrementNext, 'th element');
        whatToIncrementNext := (whatToIncrementNext + 1) mod length(a);
    end;

var
    incr: uint32;

begin
    Reset;
    for incr in specialize TArray<uint32>.Create(1, 2, 4, 8) do
    begin
        writeln('a[NextIndex()] += ', incr, '...');
        a[NextIndex] += incr;
        writeln(a[0], ' ', a[1], ' ', a[2], ' ', a[3], LineEnding);
    end;

    if (a[0]<>1) or (a[1]<>2) or (a[2]<>4) or (a[3]<>8) then
      halt(1);

    Reset;
    for incr in specialize TArray<uint32>.Create(1, 2, 4, 8) do
    begin
        writeln('NextPtr()^ += ', incr, '...');
        NextPtr^ += incr;
        writeln(a[0], ' ', a[1], ' ', a[2], ' ', a[3], LineEnding);
    end;

    if (a[0]<>1) or (a[1]<>2) or (a[2]<>4) or (a[3]<>8) then
      halt(1);

    writeln('ok');
end.
