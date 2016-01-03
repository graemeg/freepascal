{$mode objfpc}
{$H+}
{
    This file is part of Free Pascal build tools
    Copyright (c) 2005 by Michael Van Canneyt

    Implements string substitutions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit usubst;

interface

uses SysUtils,Classes;

// Add N=V pair to list.
Procedure AddToList(List : TStrings; Const N,V : String);
// Split NV to N/V and call AddToList
Function  AddPair(List : TStrings; Const NV : String) : Boolean;
// Perform substitutions in S, from List.
Function  DoSubStitutions(List : TStrings; Var S : String) : Integer;

implementation

Procedure AddToList(List : TStrings; Const N,V : String);

var
  I : Integer;

begin
  I:=List.IndexOfName(N);
  If (V='') then
    begin
    If (I<>-1) then
      List.Delete(I)
    end
  else
    begin
    If (I=-1) then
      List.Add(N+'='+V)
    else
      List[I]:=N+'='+V;
    end;
end;

Function AddPair(List : TStrings; Const NV : String) : Boolean;

Var
  P : Integer;
  N,V : string;

begin
  P:=Pos('=',NV);
  Result:=(P<>0);
  If Result then
    begin
    V:=NV;
    N:=Copy(V,1,P-1);
    Delete(V,1,P);
    AddToList(List,N,V);
    end;
end;

Function DoSubstitutions(List : TStrings; Var S : String) : Integer;

Var
  N,T : String;
  P : Integer;

begin
  Result:=0;
  T:=S;
  S:='';
  P:=Pos('%',T);
  While (P>0) do
    begin
    S:=S+Copy(T,1,P-1);
    Delete(T,1,P);
    If (Length(T)>0) then
      if (T[1]='%') then
        begin
        S:=S+'%';
        Delete(T,1,1);
        end
      else
        begin
        P:=Pos('%',T);
        If (P=0) then
          S:=S+'%'
        else
          begin
          N:=Copy(T,1,P-1);
          Delete(T,1,P);
          S:=S+List.Values[N];
          end;
        end;
    P:=Pos('%',T);
    end;
  S:=S+T;
end;

end.
