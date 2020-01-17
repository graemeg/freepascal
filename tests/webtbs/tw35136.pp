{ %opt=-Miso }
program p;
var f: text;
begin
    rewrite(f);
    f^ := 'a';
    put(f);
    reset(f);
    if eof(f) then writeln('premature eof');
    writeln(f^);
    if eof(f) then writeln('premature eof');
    writeln(f^);
    if eof(f) then writeln('premature eof');
    get(f);
    if eof(f) then writeln('eof correctly set') else begin writeln('eof should be set, but isn''t'); halt(1); end;
end.
