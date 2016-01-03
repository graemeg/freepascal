program tclass13;

{$ifdef fpc}
  {$mode delphi}{$H+}
{$endif}
{$apptype console}
uses
  typinfo;
type
  TRootClass = class
  public
    type
      TNode = class
      private
        FTest: Integer;
        type
          TNode = class
            FNode: TRootClass.TNode.TNode;
          end;
          en = (e1,e2);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          TOtherNode = class
          public
            type
              TNestedInOtherNode = class
                ffield: longint;
              end;
          end;
          TFinalNode = class
            fx: TRootClass.TNode.TOtherNode.TNestedInOtherNode;
          end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      TOtherNode = class
      public
        type
          TNestedInOtherNode = class
            ffield: longint;
          end;
      end;
      TFinalNode = class
        fx: TRootClass.TOtherNode.TNestedInOtherNode;
      end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      published
        property Test: Integer read FTest write FTest;
      end;
    class procedure DoTest;
  end;

class procedure TRootClass.DoTest;
var
  Test: TNode;
  Test1: TNode.TNode;
begin
  Test := TNode.Create;
  Test.Test := 1;
  if Test.ClassName <> 'TRootClass.TNode' then
    halt(1);
  Test.Free;
  Test1 := TNode.TNode.Create;
  if Test1.ClassName <> 'TRootClass.TNode.TNode' then
    halt(2);
  Test1.FNode:=Test1;
  Test1.Free;
end;

begin
  TRootClass.DoTest;
  if GetEnumName(TypeInfo(TRootClass.TNode.en), ord(e1))<>'e1' then
    halt(3);
end.
