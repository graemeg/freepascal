{$mode objfpc}
unit tw11431;

interface

uses sysutils;

type

  generic IGenericCollection<_T> = interface
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  generic CGenericCollection<_T> = class(TInterfacedObject, specialize IGenericCollection<_T>)
=======
  generic CGenericCollection<_T> = class( IGenericCollection)
>>>>>>> graemeg/fixes_2_2
=======
  generic CGenericCollection<_T> = class( IGenericCollection)
>>>>>>> origin/fixes_2_2
  end;

implementation


end.
