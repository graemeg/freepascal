{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

unit linklist;

{
   A unit for an easy way to use exec linked lists
   for Amiga. Can also be used for other platforms
   as it is. I hope.
   27 Oct 1998.

   Added the define use_amiga_smartlink.
   13 Jan 2003.

   nils.sjoholm@mailbox.swipnet.se
}

interface

uses
{$ifdef Amiga}
   Exec,amigalib,
{$endif}
   strings;

{ $define showall}

{$ifndef Amiga}
type

    pNode = ^tNode;
    tNode = record
        ln_Succ: pNode;
        ln_Pred: pNode;
        ln_Type: byte;
        ln_Pri : shortint;
        ln_Name: pchar;
        end;

    pList = ^tList;
    tList = record
        lh_Head: pNode;
        lh_Tail: pNode;
        lh_TailPred: pNode;
        lh_Type: byte;
        l_pad: byte;
        end;

{$endif}

type
    pFPCNode = ^tFPCNode;
    tFPCNode = record
        ln_Succ   : pNode;
        ln_Pred   : pNode;
        ln_Type   : Byte;
        ln_Pri    : Shortint;
        ln_Name   : PChar;
{
   Increase this record if you need more information
   just add your own to the record. Don't forget to
   change the functions or add your own functions.
}
        ln_Size   : Longint;
        end;

{$ifndef Amiga}
procedure NewList (list: pList);
procedure AddHead(list : pList; node : pNode);
procedure AddTail(list : pList; node : pNode);
procedure Insert(list : pList; node, lnode: pNode);
procedure Remove(node : pNode);
function RemHead(list : pList): pNode;
function RemTail(list : pList): pNode;
{$endif}

FUNCTION AddNewNode(VAR fpclist : pList; Str : PChar): pFPCNode;
FUNCTION AddNewNode(VAR fpclist : pList; Str : String): pFPCNode;
PROCEDURE ClearList(VAR fpclist : pList);
PROCEDURE CreateList(VAR fpclist : pList);
FUNCTION CopyList(fpclist : pList): pList;
PROCEDURE DeleteNode(ANode : pFPCNode);
PROCEDURE DestroyList(VAR fpclist : pList);
FUNCTION FindNodeData(fpclist : pList; data : PChar): pFPCNode;
FUNCTION FindNodeData(fpclist : pList; data : String): pFPCNode;
FUNCTION GetFirstNode(fpclist : pList): pFPCNode;
FUNCTION GetLastNode(fpclist : pList): pFPCNode;
FUNCTION GetNextNode( ANode : pFPCNode): pFPCNode;
FUNCTION GetNodeData(Anode : pFPCNode): PChar;
FUNCTION GetNodeNumber(fpclist : pList; num : Longint): pFPCNode;
FUNCTION GetPrevNode( ANode : pFPCNode): pFPCNode;
FUNCTION InsertNewNode(var fpclist : pList; data : PChar; Anode : pFPCNode): pFPCNode;
FUNCTION InsertNewNode(var fpclist : pList; data : String; Anode : pFPCNode): pFPCNode;
PROCEDURE ListToBuffer(fpclist : pList; VAR buf : PChar);
FUNCTION MergeLists(firstlist , secondlist : pList): pList;
PROCEDURE MoveNodeBottom(var fpclist: pList; ANode : pFPCNode);
PROCEDURE MoveNodeDown(VAR fpclist : pList; ANode : pFPCNode);
PROCEDURE MoveNodeTop(VAR fpclist: pList; ANode : pFPCNode);
PROCEDURE MoveNodeUp(VAR fpclist : pList; ANode : pFPCNode);
FUNCTION NodesInList(fpclist :  pList): Longint;
PROCEDURE PrintList(fpclist : pList);
PROCEDURE RemoveDupNode( VAR fpclist :  pList);
PROCEDURE RemoveLastNode(VAR fpclist : pList);
FUNCTION SizeOfList(fpclist : pList): Longint;
PROCEDURE SortList(VAR fpclist: pList);
FUNCTION UpDateNode(ANode : pFPCNode; data : PChar): BOOLEAN;
FUNCTION UpDateNode(ANode : pFPCNode; data : String): BOOLEAN;

function FileToList(thefile : PChar; var thelist : pList): boolean;
function FileToList(thefile : String; var thelist : pList): boolean;
function ListToFile(TheFile : PChar; thelist : pList): Boolean;
function ListToFile(TheFile : String; thelist : pList): Boolean;

implementation

{$ifndef Amiga}
procedure NewList (list: pList);
begin
    list^.lh_Head     := pNode(@list^.lh_Tail);
    list^.lh_Tail     := NIL;
    list^.lh_TailPred := pNode(@list^.lh_Head)
end;


procedure AddHead(list : pList; node : pNode);
begin
    node^.ln_Succ := list^.lh_Head;
    node^.ln_Pred := pNode(@list^.lh_Head);
    list^.lh_Head^.ln_Pred := node;
    list^.lh_Head := node;
end;

procedure AddTail(list : pList; node : pNode);
begin
    node^.ln_Succ := pNode(@list^.lh_Tail);
    node^.ln_Pred := list^.lh_TailPred;
    list^.lh_TailPred^.ln_Succ := node;
    list^.lh_TailPred := node;
end;

procedure Insert(list : pList; node : pNode; lnode: pNode);
begin
    {*
     *  Insert node after lnode.  If lnode = NIL then insert
     *  at head of list.
     *}

    if (lnode = NIL) then lnode := pNode(@list^.lh_Head);
    node^.ln_Pred := lnode;
    node^.ln_Succ := lnode^.ln_Succ;
    lnode^.ln_Succ := node;
    node^.ln_Succ^.ln_Pred := node;
end;

procedure Remove(node : pNode);
begin
    node^.ln_Succ^.ln_Pred := node^.ln_Pred;
    node^.ln_Pred^.ln_Succ := node^.ln_Succ;
    node^.ln_Succ := NIL;
    node^.ln_Pred := NIL;
end;

function RemHead(list : pList): pNode;
var
    node : pNode;
begin
    node := list^.lh_Head;
    if (node^.ln_Succ <> NIL) then begin
        node^.ln_Succ^.ln_Pred := node^.ln_Pred;
        node^.ln_Pred^.ln_Succ := node^.ln_Succ;
        node^.ln_Succ := NIL;
        node^.ln_Pred := NIL;
    end else node := NIL;
    RemHead := node;
end;

function RemTail(list : pList): pNode;
var
    node : pNode;
begin
    node := list^.lh_TailPred;
    if (node^.ln_Pred <> NIL) then Remove(node)
       else node := NIL;
    RemTail := node;
end;

{$endif}

FUNCTION AddNewNode(VAR fpclist : pList; Str : PChar): pFPCNode;
VAR
   tempnode : pFPCNode;
BEGIN
   New(tempnode);
   tempnode^.ln_Name := StrAlloc(StrLen(Str)+1);
   IF tempnode^.ln_Name <>  NIL THEN BEGIN
      StrCopy(tempnode^.ln_Name,Str);
      tempnode^.ln_Size := 0;
      tempnode^.ln_Type := 0;
      tempnode^.ln_Pri  := 0;
      AddTail(fpclist,pNode(tempnode));
      AddNewNode := tempnode;
   END ELSE BEGIN
      AddNewNode := NIL;
   END;
END;

FUNCTION AddNewNode(VAR fpclist : pList; Str : String): pFPCNode;
VAR
   tempnode : pFPCNode;
BEGIN
   New(tempnode);
   tempnode^.ln_Name := StrAlloc(Length(Str)+1);
   IF tempnode^.ln_Name <>  NIL THEN BEGIN
      StrPCopy(tempnode^.ln_Name,Str);
      tempnode^.ln_Size := 0;
      tempnode^.ln_Type := 0;
      tempnode^.ln_Pri  := 0;
      AddTail(fpclist,pNode(tempnode));
      AddNewNode := tempnode;
   END ELSE BEGIN
      AddNewNode := NIL;
   END;
END;

PROCEDURE ClearList(VAR fpclist : pList);
VAR
   tempnode : pFPCNode;
   dummy    : pNode;
BEGIN
   WHILE fpclist^.lh_Head <> @fpclist^.lh_Tail DO BEGIN
       tempnode := pFPCNode(fpclist^.lh_Head);
       if tempnode <> nil then begin
           if tempnode^.ln_Name <> nil then begin
              StrDispose(tempnode^.ln_Name);
           end;
           dummy := RemHead(fpclist);
           Dispose(tempnode);
       end;
   END;
END;

FUNCTION CopyList(fpclist : pList): pList;
VAR
    templist : pList;
    tempnode : pFPCNode;
    i, dummy : Longint;
    added    : pFPCNode;
BEGIN
    CreateList(templist);
    i := NodesInList(fpclist);
    tempnode := pFPCNode(fpclist^.lh_Head);
    FOR dummy := 1 TO i DO BEGIN
       added := AddNewNode(templist,tempnode^.ln_Name);
       tempnode := pFPCNode(tempnode^.ln_Succ);
    END;
    IF added = NIL THEN BEGIN
       CopyList := NIL;
    END ELSE BEGIN
       CopyList := templist;
    END;
END;

PROCEDURE CreateList(VAR fpclist : pList);
BEGIN
    New(fpclist);
    NewList(fpclist);
END;

PROCEDURE DeleteNode(ANode : pFPCNode);
BEGIN
   IF Assigned(ANode)THEN BEGIN
       IF Assigned(ANode^.ln_Name)THEN BEGIN
            StrDispose(ANode^.ln_Name);
       END;
       Remove(pNode(ANode));
       Dispose(ANode);
   END;
END;

{ remove all nodes, list is killed }
PROCEDURE DestroyList(VAR fpclist : pList);
VAR
   tempnode : pFPCNode;
   dummy    : pNode;
BEGIN
 WHILE fpclist^.lh_Head <> @fpclist^.lh_Tail DO BEGIN
       tempnode := pFPCNode(fpclist^.lh_Head);
       if Assigned(tempnode) then begin
           if Assigned(tempnode^.ln_Name) then begin
              {$ifdef showall}
                  write('releasing ');
                  writeln(tempnode^.ln_Name);
              {$endif}
              StrDispose(tempnode^.ln_Name);
           end;
           dummy := RemHead(fpclist);
           {$ifdef showall}
              writeln('Disposing node');
           {$endif}
           Dispose(tempnode);
       end;
   END;
   if Assigned(fpclist) then begin
      {$ifdef showall}
          writeln('Disposing of list');
      {$endif}
      Dispose(fpclist);
      fpclist := nil;
   end;
END;

FUNCTION FindNodeData(fpclist : pList; data : PChar): pFPCNode;
VAR
    temp : pFPCNode;
    result : pFPCNode;
BEGIN
    result := NIL;
    IF fpclist^.lh_Head^.ln_Succ <> NIL THEN BEGIN
        temp := pFPCNode(fpclist^.lh_Head);
        WHILE (temp^.ln_Succ <> NIL) DO BEGIN
            IF (StrIComp(temp^.ln_Name,data)=0) THEN BEGIN
                result := temp;
                break;
            END;
            temp := pFPCNode(temp^.ln_Succ);
        END;
    END;
    FindNodeData := result;
END;

FUNCTION FindNodeData(fpclist : pList; data : String): pFPCNode;
VAR
    temp : pFPCNode;
    result : pFPCNode;
    p : PChar;
BEGIN
    result := NIL;
    p := StrAlloc(length(data)+1);
    StrPCopy(p,data);
    IF fpclist^.lh_Head^.ln_Succ <> NIL THEN BEGIN
        temp := pFPCNode(fpclist^.lh_Head);
        WHILE (temp^.ln_Succ <> NIL) DO BEGIN
            IF (StrIComp(temp^.ln_Name,p)=0) THEN BEGIN
                result := temp;
                break;
            END;
            temp := pFPCNode(temp^.ln_Succ);
        END;
    END;
    StrDispose(p);
    FindNodeData := result;
END;

FUNCTION GetFirstNode(fpclist : pList): pFPCNode;
var
    head : pFPCNode;
BEGIN
    head := pFPCNode(fpclist^.lh_Head);
    if head^.ln_Succ <> nil then begin
        GetFirstNode := pFPCNode(head);
    end else GetFirstNode := nil;
END;

FUNCTION GetLastNode(fpclist : pList): pFPCNode;
var
    tail : pFPCNode;
BEGIN
    tail := pFPCNode(fpclist^.lh_TailPred);
    if tail^.ln_Pred <> nil then begin
        GetLastNode := pFPCNode(tail);
    end else GetLastNode := nil;
END;

FUNCTION GetNextNode( ANode : pFPCNode): pFPCNode;
var
    next : pFPCNode;
BEGIN
    next := pFPCNode(ANode^.ln_Succ);
    if next^.ln_Succ <> nil then begin
       GetNextNode := pFPCNode(next);
    end else GetNextNode := nil;
END;

FUNCTION GetNodeData(Anode : pFPCNode): PChar;
BEGIN
   IF ANode <> NIL THEN BEGIN
       IF ANode^.ln_Name <> NIL THEN BEGIN
           GetNodeData := ANode^.ln_Name;
       END ELSE BEGIN
           GetNodeData := NIL;
       END;
   END;
END;

FUNCTION GetNodeNumber(fpclist : pList; num : Longint): pFPCNode;
VAR
   dummy : Longint;
   tempnode : pFPCNode;
BEGIN
    IF num <= NodesInList(fpclist) then begin
       tempnode := pFPCNode(fpclist^.lh_Head);
       FOR dummy := 1 TO num DO BEGIN
          tempnode := pFPCNode(tempnode^.ln_Succ);
       END;
       GetNodeNumber := pFPCNode(tempnode);
    END ELSE BEGIN
       GetNodeNumber := NIL;
    END;
END;

FUNCTION GetPrevNode( ANode : pFPCNode): pFPCNode;
var
    prev : pFPCNode;
BEGIN
    prev := pFPCNode(ANode^.ln_Pred);
    if prev^.ln_Pred <> nil then begin
       GetPrevNode := pFPCNode(prev);
    end else GetPrevNode := nil;
END;

FUNCTION InsertNewNode(var fpclist : pList; data : PChar; Anode : pFPCNode): pFPCNode;
VAR
    dummy    : pFPCNode;
BEGIN
    dummy := AddNewNode(fpclist,data);
    IF dummy <> NIL THEN BEGIN
        IF (ANode <> NIL) THEN BEGIN
            Remove(pNode(dummy));
{$ifdef Amiga}
            ExecInsert(fpclist,pNode(dummy),pNode(Anode));
{$else}
            Insert(fpclist,pNode(dummy),pNode(Anode));
{$endif}
        END;
        InsertNewNode := dummy;
    END ELSE begin
        InsertNewNode := NIL;
    END;
END;

FUNCTION InsertNewNode(var fpclist : pList; data : String; Anode : pFPCNode): pFPCNode;
VAR
    dummy    : pFPCNode;
BEGIN
    dummy := AddNewNode(fpclist,data);
    IF dummy <> NIL THEN BEGIN
        IF (ANode <> NIL) THEN BEGIN
            Remove(pNode(dummy));
{$ifdef Amiga}
            ExecInsert(fpclist,pNode(dummy),pNode(Anode));
{$else}
            Insert(fpclist,pNode(dummy),pNode(Anode));
{$endif}
        END;
        InsertNewNode := dummy;
    END ELSE begin
        InsertNewNode := NIL;
    END;
END;

PROCEDURE ListToBuffer(fpclist : pList; VAR buf : PChar);
VAR
   i     : Longint;
   dummy : Longint;
   tempnode : pFPCNode;
BEGIN
   buf[0] := #0;
   i := NodesInList(fpclist);
   tempnode := pFPCNode(fpclist^.lh_Head);
   FOR dummy := 1 TO i DO BEGIN
      IF tempnode^.ln_Name <> NIL THEN BEGIN
         strcat(buf,tempnode^.ln_Name);
         IF dummy < i THEN BEGIN
            StrCat(buf,PChar(';'#0));
         END;
      END;
      tempnode := pFPCNode(tempnode^.ln_Succ);
   END;
END;

FUNCTION MergeLists(firstlist , secondlist : pList): pList;
VAR
    templist : pList;
    tempnode : pFPCNode;
    i, dummy : Longint;
    added    : pFPCNode;
BEGIN
    CreateList(templist);
    i := NodesInList(firstlist);
    tempnode := pFPCNode(firstlist^.lh_Head);
    FOR dummy := 0 TO i DO BEGIN
       added := AddNewNode(templist,tempnode^.ln_Name);
       tempnode := pFPCNode(tempnode^.ln_Succ);
    END;
    IF added = NIL THEN BEGIN
       MergeLists := NIL;
    END ELSE BEGIN
       i := NodesInList(secondlist);
       tempnode := pFPCNode(secondlist^.lh_Head);
       FOR dummy := 0 TO i DO BEGIN
          added := AddNewNode(templist,tempnode^.ln_Name);
          tempnode := pFPCNode(tempnode^.ln_Succ);
       END;
       IF added = NIL THEN BEGIN
          MergeLists := NIL;
       END ELSE BEGIN
          MergeLists := templist;
       END;
    END;
END;

{ move a node to the bottom of the list }
PROCEDURE MoveNodeBottom(var fpclist: pList; ANode : pFPCNode);

BEGIN
    IF ANode^.ln_Succ <> NIL THEN BEGIN
        Remove(pNode(ANode));
        AddTail(fpclist,pNode(ANode));
    END;
END;

{ move a node down the list }
PROCEDURE MoveNodeDown(VAR fpclist : pList; ANode : pFPCNode);
VAR
    suc : pFPCNode;
BEGIN
    suc := pFPCNode(ANode^.ln_Succ);
    IF (ANode <> NIL) AND (suc <> NIL) THEN BEGIN
        Remove(pNode(ANode));
{$ifdef Amiga}
        ExecInsert(fpclist,pNode(ANode),pNode(suc));
{$else}
        Insert(fpclist,pNode(ANode),pNode(suc));
{$endif}
    END;
END;

{ move a node up to the top of the list }
PROCEDURE MoveNodeTop(VAR fpclist: pList; ANode : pFPCNode);
BEGIN
    IF ANode^.ln_Pred <> NIL THEN BEGIN
        Remove(pNode(ANode));
        AddHead(fpclist,pNode(ANode));
    END;
END;

{ move a node up the list }
PROCEDURE MoveNodeUp(VAR fpclist : pList; ANode : pFPCNode);
VAR
    prev : pFPCNode;
BEGIN
    prev := pFPCNode(Anode^.ln_Pred);
    IF (ANode <> NIL) AND (prev <> NIL) THEN BEGIN
        prev := pFPCNode(prev^.ln_Pred);
        Remove(pNode(ANode));
{$ifdef Amiga}
        ExecInsert(fpclist,pNode(ANode),pNode(prev));
{$else}
        Insert(fpclist,pNode(ANode),pNode(prev));
{$endif}
    END;
END;

FUNCTION NodesInList(fpclist :  pList): Longint;
VAR
   tempnode : pFPCNode;
   i        : Longint;
BEGIN
    i := 0;
    tempnode := pFPCNode(fpclist^.lh_Head);
    WHILE tempnode^.ln_Succ <> NIL DO BEGIN
        tempnode := pFPCNode(tempnode^.ln_Succ);
        INC(i);
    END;
    NodesInList := i;
END;

PROCEDURE PrintList(fpclist : pList);
VAR
   i     : Longint;
   dummy : Longint;
   tempnode : pFPCNode;
BEGIN

   i := NodesInList(fpclist);

   tempnode := pFPCNode(fpclist^.lh_Head);
   FOR dummy := 1 TO i DO BEGIN
       IF tempnode^.ln_Name <> NIL THEN BEGIN
          WriteLN(tempnode^.ln_Name);
       END;
       tempnode := pFPCNode(tempnode^.ln_Succ);
   END;
END;

PROCEDURE RemoveDupNode( VAR fpclist :  pList);
VAR
   tempnode : pFPCNode;
   nextnode : pFPCNode;
BEGIN
    tempnode := pFPCNode(fpclist^.lh_Head);

    WHILE tempnode^.ln_Succ <> NIL DO BEGIN
         nextnode := pFPCNode(tempnode^.ln_Succ);
        IF (StrIComp(tempnode^.ln_Name,nextnode^.ln_Name)=0) THEN BEGIN
            DeleteNode(tempnode);
        END;
        tempnode := nextnode;
    END;
END;

PROCEDURE RemoveLastNode(VAR fpclist : pList);
VAR
   tempnode : pFPCNode;
   dummy    : pNode;
BEGIN
   tempnode := pFPCNode(fpclist^.lh_TailPred);
   if tempnode^.ln_Name <> nil then begin
      StrDispose(tempnode^.ln_Name);
   end;
   dummy := RemTail(fpclist);
   Dispose(tempnode);
END;

{ get the total size allocated by list }
{ size is WITH ';' between the strings }
FUNCTION SizeOfList(fpclist : pList): Longint;
VAR
   i     : Longint;
   dummy : Longint;
   tempnode : pFPCNode;
   tsize    : Longint;
BEGIN
   tsize := 0;
   i := NodesInList(fpclist);

    tempnode := pFPCNode(fpclist^.lh_Head);
    FOR dummy := 1 TO i DO BEGIN
        IF tempnode^.ln_Name <> NIL THEN BEGIN
            tsize := tsize + (StrLen(tempnode^.ln_Name)+1)
        END;
        tempnode := pFPCNode(tempnode^.ln_Succ);
    END;
    SizeOfList := tsize;
END;

{ sort the list using a bubble sort }
PROCEDURE SortList(VAR fpclist: pList);

VAR
    notfinished : BOOLEAN;
    first, second : pFPCNode;
    n,i : Longint;

BEGIN
    IF fpclist^.lh_Head^.ln_Succ <> NIL THEN BEGIN
        notfinished := True;
        i := NodesInList(fpclist);
        WHILE (notfinished) DO BEGIN
            notfinished := FALSE;
            first := pFPCNode(fpclist^.lh_Head);
            IF first <> NIL THEN BEGIN
                n := 1;
                second := pFPCNode(first^.ln_Succ);
                WHILE n <> i DO BEGIN
                    n := n + 1;
                    IF (StrIComp(first^.ln_Name,second^.ln_Name)>0) THEN BEGIN
                        Remove(pNode(first));
{$ifdef Amiga}
                        ExecInsert(fpclist,pNode(first),pNode(second));
{$else}
                        Insert(fpclist,pNode(first),pNode(second));
{$endif}
                        notfinished := True;
                    END ELSE
                        first := second;
                    second := pFPCNode(first^.ln_Succ);
                END;
            END;
        END;
    END;
END;

FUNCTION UpDateNode(ANode : pFPCNode; data : PChar): BOOLEAN;
VAR
   result : BOOLEAN;
BEGIN
    IF ANode^.ln_Succ <> NIL THEN BEGIN
        IF ANode^.ln_Name <> NIL THEN BEGIN
            StrDispose(ANode^.ln_Name);
            ANode^.ln_Name := StrAlloc(StrLen(data)+1);
            IF ANode^.ln_Name <> NIL THEN BEGIN
                StrCopy(ANode^.ln_Name,data);
                result := True;
            END ELSE BEGIN
                result := FALSE;
            END;
         END;
     END;
     UpDateNode := result;
END;

FUNCTION UpDateNode(ANode : pFPCNode; data : String): BOOLEAN;
VAR
   result : BOOLEAN;
BEGIN
    IF ANode^.ln_Succ <> NIL THEN BEGIN
        IF ANode^.ln_Name <> NIL THEN BEGIN
            StrDispose(ANode^.ln_Name);
            ANode^.ln_Name := StrAlloc(Length(data)+1);
            IF ANode^.ln_Name <> NIL THEN BEGIN
                StrPCopy(ANode^.ln_Name,data);
                result := True;
            END ELSE BEGIN
                result := FALSE;
            END;
         END;
     END;
     UpDateNode := result;
END;

function FileToList(thefile : PChar; var thelist : pList): boolean;
begin
    FileToList := FileToList(strpas(thefile), thelist);
end;

function FileToList(thefile : String; var thelist : pList): boolean;
var
   Inf : Text;
   tempnode : pFPCNode;
   buffer : PChar;
   buf : Array [0..500] of Char;
begin
   buffer := @buf;
   Assign(Inf, thefile);
   {$I-}
   Reset(Inf);
   {$I+}
   if IOResult = 0 then begin
      while not eof(Inf) do begin
      { I don't want end of lines here (for use with amiga listviews)
        just change this if you need newline characters.
      }
         Read(Inf, buffer);
         tempnode := AddNewNode(thelist,buffer);
         Readln(inf, buffer);
      end;
      CLose(Inf);
      FileToList := true;
   end else FileToList := false;
end;

function ListToFile(TheFile : PChar; thelist : pList): Boolean;
begin
    ListToFile := ListToFile(strpas(TheFile), thelist);
end;

function ListToFile(TheFile : String; thelist : pList): Boolean;
VAR
    Out      : Text;
    i        : Longint;
    dummy    : Longint;
    tempnode : pFPCNode;
begin
    Assign(Out, TheFile);
    {$I-}
    Rewrite(Out);
    {$I+}
    if IOResult = 0 then begin
       i := NodesInList(thelist);
       IF i > 0 THEN BEGIN
          tempnode := pFPCNode(thelist^.lh_Head);
          FOR dummy := 1 TO i DO BEGIN
             IF tempnode^.ln_Name <> NIL THEN BEGIN
                {
                  Have to check the strlen here, if it's an
                  empty pchar fpc will write out a #0
                }
                if strlen(tempnode^.ln_Name) > 0 then
                   WriteLN(Out,tempnode^.ln_Name)
                else writeln(Out);
             END;
             tempnode := pFPCNode(tempnode^.ln_Succ);
          END;
        END;
        Close(Out);
        ListToFile := True;
    END Else ListToFile := False;
END;


end.
