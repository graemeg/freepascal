{$mode objfpc}
{$interfaces corba}
type
  generic IList<TElem> = interface
  end;

  generic ISet<TElem> = interface
  end;

  generic IMap<TKey, TValue> = interface

    type
      TKeySet = specialize ISet<TKey>; // wrong syntax?
      TValueSet = specialize IList<TValue>; // wrong syntax?

    function Keys : TKeySet;
    function Values : TValueSet;
  end;

begin
end.

