{$mode objfpc}{$h+}
uses classes, sysutils;
type
<<<<<<< HEAD
<<<<<<< HEAD
  generic TNode<T> = class
  public
    type
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      TAlias = T;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      PT = ^T;
  private
    var
      Data: T;
  public
    constructor Create;
    destructor Destroy; override;
  end;

  generic TContainer<T> = class
  public
    type
      TTNode = specialize TNode<T>;
  private
    var
      Data: TTNode;
  public
    constructor Create;
    destructor Destroy; override;

    function GetAddr: TTNode.PT;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SetV(v: TTNode.TAlias);
=======
    procedure SetV(v: TTNode.T);
>>>>>>> graemeg/cpstrnew
=======
    procedure SetV(v: TTNode.T);
>>>>>>> graemeg/cpstrnew
=======
    procedure SetV(v: TTNode.T);
>>>>>>> graemeg/cpstrnew
=======
    procedure SetV(v: TTNode.T);
>>>>>>> origin/cpstrnew
  end;
=======
=======
>>>>>>> origin/fixes_2_2
        generic TNode<T> = class
        type public
                PT = ^T;
        var private
                Data: T;
        public
                constructor Create;
                destructor Destroy; override;
        end;

        generic TContainer<T> = class
        type public
                TTNode = specialize TNode<T>;
        var
        private
                Data: TTNode;
        public
                constructor Create;
                destructor Destroy; override;

                function GetAddr: TTNode.PT;
                procedure SetV(v: TTNode.T);
        end;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

constructor TNode.Create;
begin
end;

destructor TNode.Destroy;
begin
<<<<<<< HEAD
<<<<<<< HEAD
  inherited Destroy;
=======
        inherited Destroy;
>>>>>>> graemeg/fixes_2_2
=======
        inherited Destroy;
>>>>>>> origin/fixes_2_2
end;

constructor TContainer.Create;
begin
  Data:=TTNode.Create;
end;

destructor TContainer.Destroy;
begin
  Data.Free;
        inherited Destroy;
end;

function TContainer.GetAddr: TTNode.PT;
begin
        result := @Data.Data;
end;


<<<<<<< HEAD
<<<<<<< HEAD
procedure TContainer.SetV(v: TTNode.TAlias);
=======
procedure TContainer.SetV(v: TTNode.T);
>>>>>>> graemeg/fixes_2_2
=======
procedure TContainer.SetV(v: TTNode.T);
>>>>>>> origin/fixes_2_2
begin
  Data.Data:=v;
end;

type
  TStringContainer=specialize TContainer<String>;
var
  c : TStringContainer;
begin
  c:=TStringContainer.Create;
<<<<<<< HEAD
<<<<<<< HEAD
  c.SetV('abc');
  Writeln(HexStr(c.GetAddr));
=======
  c.Set('abc');
  Writeln(HexStr(c.Get));
>>>>>>> graemeg/fixes_2_2
=======
  c.Set('abc');
  Writeln(HexStr(c.Get));
>>>>>>> origin/fixes_2_2
end.
