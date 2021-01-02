{ %FAIL }

library tw38289a;
procedure Test; begin end;
exports
  Test index 3 'abc';
  //------------^^^
end.
