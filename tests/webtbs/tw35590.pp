{$mode objfpc}
{$inline on}
{$h+}
program project1;

  function sLow: integer; inline;
  begin result := 1; end;

  function sHigh( const s: string): integer; inline;
  begin result := Length(s); end;

  procedure insert2( const substr: string; var s: string; index: integer);
  begin insert( substr, s, index); end;

  function replaceChars(const s, subStr: string): string;
  var i: integer;
  begin
    result := s;
    // ok with sHigh(s) or with non-inlined sHigh(result)
    for i := sHigh(result) downto sLow() do begin
      delete( result, i, 1);
      insert2( subStr, result, i); // ok with (unwrapped) insert( subStr, result, i)
    end;
  end; // Error: Internal error 200405231

  procedure test1;
  var s, newChar, r: ansistring;
  begin
    s := 'old'; newChar := 'Replace';
    r := replaceChars( s, newChar);
  end;

begin
test1;
end.
