{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit whtml;

{$ifdef fpc}
{$mode objfpc}
{$endif}

interface

uses wformat,Classes,SysUtils;

Type
  THTMLWriter=Class(TFormattingWriter)
  Public
    Constructor Create (AStream : TStream); override;
    Procedure TagStart(Const Name, Attrs : String);
    Procedure TagEnd(Const Name : String);
    Function EscapeText (AText : String) : String; override;
    Procedure DocumentStart(Const Title : String); override;
    Procedure DocumentEnd; override;
    Procedure HeaderStart(Alevel : Integer); override;
    Procedure HeaderEnd(Alevel : Integer); override;
    Procedure ParagraphStart; override;
    Procedure ParagraphEnd; override;
    Procedure LineBreak; override;
    Procedure Rule; override;
    Procedure BoldStart; override;
    Procedure BoldEnd;override;
    Procedure ItalicStart;override;
    Procedure ItalicEnd;override;
    Procedure UnderlineStart;override;
    Procedure UnderlineEnd;override;
    Procedure PreformatStart; override;
    Procedure PreformatEnd; override;
    Procedure TableStart( NoCols: Integer; Border : Boolean); override;
    Procedure TableEnd; override;
    Procedure RowStart; override;
    Procedure RowEnd; override;
    Procedure CellStart; override;
    Procedure CellEnd; override;
    Procedure HeaderCellStart; override;
    Procedure HeaderCellEnd; override;
    Procedure ListStart(ListType : TListType); override;
    Procedure ListEnd(ListType : TListType); override;
    Procedure ListItemStart; override;
    Procedure ListItemEnd; override;
    Procedure DefinitionItem(Const Aname,AText : String); override;
    // Form support
    Procedure FormStart(Const Action,Method : String);
    Procedure FormEnd;
    Procedure EmitInput(Const Name,Value : String);
    Procedure EmitInput(Const Name,Value, Attrs : String);
    Procedure EmitPasswordInput(Const Name,Value : String);
    Procedure EmitCheckBox(Const Name,Value : String);
    Procedure EmitCheckBox(Const Name,Value : String; Checked : Boolean);
    Procedure EmitRadioButton(Const Name,Value : String);
    Procedure EmitRadioButton(Const Name,Value : String; Checked : Boolean);
    Procedure EmitArea(Const Name,Value : String; Rows,Cols : Integer);
    Procedure EmitComboBox(Const Name, Value : String; Items : TStrings; UseValues : Boolean);
    Procedure EmitComboBox(Const Name, Value : String; Items : TStrings);
    Procedure EmitButton(Const Name,ButtonType,Value : String);
    Procedure EmitSubmitButton(Const Name,Value : String);
    Procedure EmitResetButton(Const Name,Value : String);
    Procedure EmitHiddenVar(Const Name,Value: String);
  end;

Const
  ListTags : Array[TListType] of string[2] = ('OL','UL','DL');

implementation

{ THTMLWriter }

procedure THTMLWriter.BoldEnd;
begin
  TagEnd('B');
end;

procedure THTMLWriter.BoldStart;
begin
  TagStart('B','');
end;

procedure THTMLWriter.CellEnd;
begin
  TagEnd('TD');
end;

procedure THTMLWriter.CellStart;
begin
  TagStart('TD','');
end;

constructor THTMLWriter.Create(AStream: TStream);
begin
  inherited;
end;

procedure THTMLWriter.DefinitionItem(const Aname, AText: String);
begin
  TagStart('DT','');
  Write(Aname);
  TagEnd('DT');
  TagStart('DD','');
  Write(AText);
  TagEnd('DD');
end;

procedure THTMLWriter.DocumentEnd;
begin
  TagEnd('BODY');
  TagEnd('HTML');
end;

procedure THTMLWriter.DocumentStart(const Title: String);
begin
  inherited;
  TagStart('HTML','');
  TagStart('TITLE','');
  Write(Title);
  TagEnd('TITLE');
  TagStart('BODY','');
end;

function THTMLWriter.EscapeText(AText: String): String;
begin
  // replace by a more sensitive method.
  Result:=StringReplace(AText,'&','&amp;',[rfReplaceAll]);
  Result:=StringReplace(Result,'<','&lt;',[rfReplaceAll]);
  Result:=StringReplace(Result,'>','&gt;',[rfReplaceAll]);
  Result:=StringReplace(Result,#10,'<BR>',[rfreplaceAll]);
end;

procedure THTMLWriter.HeaderCellEnd;
begin
  TagEnd('TH');
end;

procedure THTMLWriter.HeaderCellStart;
begin
  TagStart('TH','');
end;

procedure THTMLWriter.HeaderEnd(Alevel: Integer);
begin
  TagEnd(Format('H%d',[ALevel]));
end;

procedure THTMLWriter.HeaderStart(Alevel: Integer);
begin
  TagStart(Format('H%d',[ALevel]),'');
end;

procedure THTMLWriter.ItalicEnd;
begin
  TagEnd('I');
end;

procedure THTMLWriter.ItalicStart;
begin
  TagStart('I','');
end;

procedure THTMLWriter.LineBreak;
begin
  TagStart('BR','');
end;

procedure THTMLWriter.ListEnd(ListType: TListType);
begin
  TagEnd(ListTags[ListType]);
end;


procedure THTMLWriter.ListItemEnd;
begin
  TagEnd('LI');

end;

procedure THTMLWriter.ListItemStart;
begin
  TagStart('LI','');
end;

procedure THTMLWriter.ListStart(ListType: TListType);
begin
  TagStart(ListTags[ListType],'');
end;

procedure THTMLWriter.ParagraphEnd;
begin
  TagEnd('P')
end;

procedure THTMLWriter.ParagraphStart;
begin
  TagStart('P','')
end;

procedure THTMLWriter.PreformatEnd;
begin
  TagEnd('PRE')
end;

procedure THTMLWriter.PreformatStart;
begin
  TagStart('PRE','');
end;

procedure THTMLWriter.RowEnd;
begin
  TagEnd('TR')
end;

procedure THTMLWriter.RowStart;
begin
  TagStart('TR','')
end;

procedure THTMLWriter.Rule;
begin
  TagStart('HR','');
end;

procedure THTMLWriter.TableStart(NoCols: Integer; Border: Boolean);

Var
  Attr : string;
begin
  if Border then
    Attr:='BORDER=1'
  else
    Attr:='';
  TagStart('TABLE',Attr);
end;

procedure THTMLWriter.TableEnd;

begin
  TagEnd('TABLE');
end;

procedure THTMLWriter.TagEnd(const Name : String);
begin
  Dump('</'+Name+'>');
end;

procedure THTMLWriter.TagStart(const Name, Attrs: String);
begin
  Dump('<'+Name);
  If Attrs<>'' then
    begin
    Dump(' ');
    Dump(Attrs);
    end;
  Dump('>');
end;

procedure THTMLWriter.UnderlineEnd;
begin
  TagEnd('U');
end;

procedure THTMLWriter.UnderlineStart;
begin
  TagStart('U','');
end;

// Form support.

Procedure THTMLWriter.FormStart(Const Action,Method : String);

Var
  A : String;

begin
  A:='ACTION="'+Action+'"';
  If (Method<>'') then
    A:=A+' METHOD="'+Method+'"';
  TagStart('FORM',A);
end;

Procedure THTMLWriter.FormEnd;

begin
  Tagend('FORM');
end;

Procedure THTMLWriter.EmitInput(Const Name,Value : String);

begin
  EmitInput(Name,Value,'');
end;

Procedure THTMLWriter.EmitPasswordInput(Const Name,Value : String);

begin
  EmitInput(Name,Value,'TYPE="password"');
end;


Procedure THTMLWriter.EmitInput(Const Name,Value, Attrs : String);

Var
  A : String;

begin
  A:='NAME="'+Name+'"';
  If (Value<>'') then
    A:=A+' VALUE="'+Value+'"';
  If (Attrs<>'') then
    A:=A+' '+Attrs;
  TagStart('INPUT',A);
end;

Procedure THTMLWriter.EmitCheckBox(Const Name,Value : String);

begin
  EmitCheckBox(Name,Value,False);
end;

Procedure THTMLWriter.EmitCheckBox(Const Name,Value : String; Checked : Boolean);

Var
  A : String;

begin
  A:='NAME="'+Name+'" TYPE="checkbox" VALUE="'+Value+'"';
  If Checked then
<<<<<<< HEAD
<<<<<<< HEAD
    A:=A+' CHECKED="checked"';
=======
    A:=A+' CHECKED=1';
>>>>>>> graemeg/fixes_2_2
=======
    A:=A+' CHECKED=1';
>>>>>>> origin/fixes_2_2
  TagStart('INPUT',A);
end;

Procedure THTMLWriter.EmitRadioButton(Const Name,Value : String);

begin
  EmitRadioButton(Name,Value,False);
end;

Procedure THTMLWriter.EmitRadioButton(Const Name,Value : String; Checked : Boolean);

Var
  A : String;

begin
  A:='NAME="'+Name+'" TYPE="checkbox" VALUE="'+Value+'"';
  If Checked then
<<<<<<< HEAD
<<<<<<< HEAD
    A:=A+' CHECKED="checked"';
=======
    A:=A+' CHECKED=1';
>>>>>>> graemeg/fixes_2_2
=======
    A:=A+' CHECKED=1';
>>>>>>> origin/fixes_2_2
  TagStart('INPUT',A);

end;

Procedure THTMLWriter.EmitArea(Const Name,Value : String; Rows,Cols : Integer);

Var
  A : String;

begin
  A:='NAME="'+Name+'"';
  If (Rows<>0) and (cols<>0) then
    A:=A+Format(' ROWS=%d COLS=%d',[Rows,Cols]);
  TagStart('TEXTAREA',A);
  Write(Value);
  TagEnd('TEXTAREA');
end;

Procedure THTMLWriter.EmitComboBox(Const Name, Value : String; Items : TStrings);

begin
  EmitComboBox(Name,Value,Items,False);
end;

Procedure THTMLWriter.EmitComboBox(Const Name, Value : String; Items : TStrings; UseValues : Boolean);

Var
  A,S,V : String;
  I,P : Integer;

begin
  TagStart('SELECT','NAME='+Name+'"');
  A:='';
  For I:=0 to Items.Count-1 do
    begin
    S:=Items[I];
    If UseValues then
      begin
      P:=Pos('=',S);
      If P>0 then
        begin
        V:=Copy(S,1,P-1);
        Delete(S,1,P);
        A:='VALUE="'+Copy(S,1,P-1)+'"';
        end
      else
        begin
        A:='';
        V:=S;
        end;
      end;
    If (Value<>'') and (V=Value) then
<<<<<<< HEAD
<<<<<<< HEAD
      A:=A+' SELECTED="selected"';
=======
      A:=A+' SELECTED';
>>>>>>> graemeg/fixes_2_2
=======
      A:=A+' SELECTED';
>>>>>>> origin/fixes_2_2
    TagStart('OPTION',A);
    end;
  TagEnd('SELECT')
end;

Procedure THTMLWriter.EmitSubmitButton(Const Name,Value : String);

begin
  EmitButton(Name,'submit',Value)
end;

Procedure THTMLWriter.EmitResetButton(Const Name,Value : String);

begin
  EmitButton(Name,'reset',Value)
end;

Procedure THTMLWriter.EmitButton(Const Name,ButtonType,Value : String);

Var
  A : String;

begin
  A:='TYPE="'+ButtonType+'"';
  If (Value<>'') then
    A:=A+' VALUE="'+Value+'"';
  If (Name<>'')  then
    A:=A+' NAME="'+Name+'"';
  TagStart('INPUT',A)
end;

Procedure THTMLWriter.EmitHiddenVar(Const Name,Value: String);

Var
  A : String;

begin
  A:='TYPE="hidden" NAME="'+Name+'" VALUE="'+Value+'"';
  TagStart('INPUT',A);
end;


end.
