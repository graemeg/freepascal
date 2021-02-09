{ %fail }
{ %opt=-Sew }
{$mode iso}
program string1(input, output);
    var
        c: packed array [1..5] of char;
        inline: packed array [1..10] of char;
begin
    c := '1234567890';
    writeln(c);
    inline := '12345';
    writeln(inline);
end.
