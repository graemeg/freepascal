program Project1;

{$mode objfpc}{$H+}
<<<<<<< HEAD
<<<<<<< HEAD
{$apptype console}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type

  { TStack }

  generic TStack<T> = class(TObject)
<<<<<<< HEAD
<<<<<<< HEAD
  public
=======
   public
>>>>>>> graemeg/fixes_2_2
=======
   public
>>>>>>> origin/fixes_2_2
    procedure Clear; virtual;
    destructor Destroy; override;
  end;

<<<<<<< HEAD
<<<<<<< HEAD
  { TAdvStack }

  generic TAdvStack<T> = class(specialize TStack<T>)
=======
  { TIntegerStack }

  TAdvStack = class(specialize TStack<T>)
>>>>>>> graemeg/fixes_2_2
=======
  { TIntegerStack }

  TAdvStack = class(specialize TStack<T>)
>>>>>>> origin/fixes_2_2
  public
    procedure Clear; override;
  end;

  { TIntegerStack }

  TIntegerStack = specialize TAdvStack<Integer>;

  { TIntegerStack2 }

<<<<<<< HEAD
<<<<<<< HEAD
  TIntegerStack2 = class(specialize TAdvStack<Integer>)
=======
  TIntegerStack2 = class(specialize TAdvStack<Integer>);
>>>>>>> graemeg/fixes_2_2
=======
  TIntegerStack2 = class(specialize TAdvStack<Integer>);
>>>>>>> origin/fixes_2_2
  public
    procedure Clear; override;
  end;

var
  Idx : Longint;

{ TAdvStack }

procedure TAdvStack.Clear;
begin
  Writeln('new clear');
  Idx:=Idx or 1;
end;

{ TIntegerStack2 }

procedure TIntegerStack2.Clear;
begin
  Writeln('new clear2');
  Idx:=Idx or 2;
end;

{ TStack }

procedure TStack.Clear;
begin
  Writeln('old clear');
end;

destructor TStack.Destroy;
begin
  Writeln('old destroy');
  Clear;
end;


var
  s : TIntegerStack;
  s2 : TIntegerStack2;
begin
  Idx:=0;

  s := TIntegerStack.Create;
  Writeln(s.ClassName);
  s.Free;

  s2 := TIntegerStack2.Create;
  Writeln(s2.ClassName);
  s2.Free;

  if Idx<>3 then
    halt(1);
end.
