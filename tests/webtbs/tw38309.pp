program c;

{$mode objfpc}

uses
  Math;

type
  generic TBase<T> = class
  private const
    AConst = 1;
  private
    GenVarA: T;
    GenVarB: T;
    function Foo: Boolean;
  end;

  function TBase.Foo: Boolean;
  begin
    //Fails with trunk win-64 if TCur type is defined (e.g. not commented out) (*)
    Result := SameValue(AConst, GenVarB);

    //Fails with trunk win-64, EVEN if TCur definition is commented out
    //Fails with 3.2.0 win-32, EVEN if TCur definition is commented out
    //Fails with 3.2.0 win-64, EVEN if TCur definition is commented out, if it is defined it gives the errormesage twice for this line
    Result := SameValue(GenVarA, GenVarB);

    //Fails with trunk win-64 if TCur type is defined (e.g. not commented out)
    Result := SameValue(GenVarA, AConst);
  end;

type
  TCur = specialize TBase<Currency>;

const
  CurConst = 1;
var
  CurVarA: Currency = 1;
  CurVarB: Currency = 2;

begin
  //Fails with trunk win-64
  SameValue(CurConst, CurVarA);

  //Fails with 3.2.0 win-64
  SameValue(Currency(CurConst), CurVarA);

  //Fails with 3.2.0 win-64
  SameValue(CurVarA, CurVarB);

  //Fails with trunk win-64
  SameValue(CurVarA, CurConst);

  //Fails with 3.2.0 win-64
  SameValue(CurVarA, Currency(CurConst));
end.
