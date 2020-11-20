{
   This file is part of the Free Pascal FCL library.
   Copyright 2013 Mario Ray Mahardhika
 
   Implements a generic Tree.
 
   See the file COPYING.FPC, included in this distribution,
   for details about the copyright.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY;without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
unit gtree;

{$mode objfpc}{$H+}

interface

uses
  gvector,gstack,gqueue;

type

  { TTreeNode }

  generic TTreeNode<T> = class
  public type
    TTreeNodeList = specialize TVector<TTreeNode>;
  protected
    FData: T;
    FChildren: TTreeNodeList;
  public
    constructor Create;
    constructor Create(const AData: T);
    destructor Destroy; override;
    property Data: T read FData write FData;
    property Children: TTreeNodeList read FChildren;
  end;

  generic TDepthFirstCallback<T> = procedure (const AData: T);
  generic TBreadthFirstCallback<T> = procedure (const AData: T);

  generic TTree<T> = class
  public type
    TTreeNodeType = specialize TTreeNode<T>;
    TDepthFirstCallbackType = specialize TDepthFirstCallback<T>;
    TBreadthFirstCallbackType = specialize TBreadthFirstCallback<T>;
  private type
    TStackType = specialize TStack<TTreeNodeType>;
    TQueueType = specialize TQueue<TTreeNodeType>;
  private
    FRoot: TTreeNodeType;
  public
    constructor Create;
    destructor Destroy; override;
    procedure DepthFirstTraverse(Callback: TDepthFirstCallbackType);
    procedure BreadthFirstTraverse(Callback: TBreadthFirstCallbackType);
    property Root: TTreeNodeType read FRoot write FRoot;
  end;

implementation


{ TTreeNode }

constructor TTreeNode.Create;
begin
  FChildren := TTreeNodeList.Create;
end;

constructor TTreeNode.Create(const AData: T);
begin
  FData := AData;
  FChildren := TTreeNodeList.Create;
end;

destructor TTreeNode.Destroy;
var
  Child: TTreeNode;
begin
  for Child in FChildren do begin
    Child.Free;
  end;
  FChildren.Free;
end;

{ TTree }

constructor TTree.Create;
begin
  FRoot := nil;
end;

destructor TTree.Destroy;
begin
  FRoot.Free;
end;

procedure TTree.DepthFirstTraverse(Callback: TDepthFirstCallbackType);
var
  Stack: TStackType;
  Node,Child: TTreeNodeType;
begin
  if Assigned(FRoot) then begin
    Stack := TStackType.Create;
    Stack.Push(FRoot);
    while Stack.Size > 0 do begin
      Node := Stack.Top;
      Stack.Pop;
      Callback(Node.Data);
      for Child in Node.Children do Stack.Push(Child);
    end;
    Stack.Free;
  end;
end;

procedure TTree.BreadthFirstTraverse(Callback: TBreadthFirstCallbackType);
var
  Queue: TQueueType;
  Node,Child: TTreeNodeType;
begin
  if Assigned(FRoot) then begin
    Queue := TQueueType.Create;
    Queue.Push(FRoot);
    while Queue.Size > 0 do begin
      Node := Queue.Front;
      Queue.Pop;
      Callback(Node.Data);
      for Child in Node.Children do Queue.Push(Child);
    end;
    Queue.Free;
  end;
end;

end.

