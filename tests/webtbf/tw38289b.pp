{ %FAIL }

library tw38289b;
procedure Test; begin end;
exports
  Test index 'abc' 3;
  //------------^^^
end.
