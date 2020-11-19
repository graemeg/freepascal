{$mode objfpc}
program Project1;
type

  TElem=(a,b,c,d,e,f);
  TmyElem=(my_a,my_c,my_e);
  TElems=set of TElem;//Output set, need convert my_a->a, my_c->c, my_e->e остальное скипаем
  TmyElems=set of TmyElem;//Input set


  generic TSetConverter<TGEnumIn,TGSetIn,TGEnumOut,TGSetOut,TGEnumConverter>=class
    class function Convert(value:TGSetIn):TGSetOut;
  end;

  TmyElem2TElem_Converter=class
    class function Convert(valueIn:TmyElem;out valueOut:TElem):boolean;
  end;

  TConverter=specialize TSetConverter<TmyElem,TmyElems,TElem,TElems,TmyElem2TElem_Converter>;

  class function TmyElem2TElem_Converter.Convert(valueIn:TmyElem;out valueOut:TElem):boolean;
  begin
    result:=true;
    case valueIn of
      my_a:valueOut:=a;
      my_c:valueOut:=c;
      my_e:valueOut:=e;
      else result:=false;
    end;
  end;

  {//Variant 1
  class function TSetConverter.Convert(value:TGSetIn):TGSetOut;
  var
   CurrentEnumIn:TGEnumIn;
   CurrentEnumOut:TGEnumOut;
   tvalue:TGSetIn;
  begin
    result:=[];
    for CurrentEnumIn:=low(TGEnumIn) to high(TGEnumIn) do begin
      tvalue:=value-[CurrentEnumIn];
      if tvalue<>value then begin
        if TGEnumConverter.convert(CurrentEnumIn,CurrentEnumOut) then
          result:=result+[CurrentEnumOut];
        if tvalue=[] then exit;
        value:=tvalue;
      end;
    end;
  end;
  }
  //Variant 2
  class function TSetConverter.Convert(value:TGSetIn):TGSetOut;
  var
   CurrentEnumIn:TGEnumIn;
   CurrentEnumOut:TGEnumOut;
  begin
    result:=[];
    for CurrentEnumIn in value do
        if TGEnumConverter.convert(CurrentEnumIn,CurrentEnumOut) then
          result:=result+[CurrentEnumOut];
  end;

var
 Elems:TElems;
 Elem:TElem;
begin
  Elems:=TConverter.Convert([my_a,my_c,my_e]);
  for Elem in Elems do
    write(Elem);
end.
