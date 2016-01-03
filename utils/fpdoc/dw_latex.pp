{

    FPDoc  -  Free Pascal Documentation Tool
    Copyright (C) 2000 - 2003 by
      Areca Systems GmbH / Sebastian Guenther, sg@freepascal.org

    * LaTeX output generator

    See the file COPYING, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
}
{$mode objfpc}
{$H+}
unit dw_LaTeX;

interface

uses DOM, dGlobals, PasTree;

const
  LateXHighLight : Boolean = False;
  TexExtension   : String = '.tex';

Procedure CreateLaTeXDocForPackage(APackage: TPasPackage; AEngine: TFPDocEngine);

implementation

uses SysUtils, Classes, dwLinear, dwriter;


Type
 { TLaTeXWriter }

  TLaTeXWriter = class(TLinearWriter)
  protected
    FLink: String;
    FImageDir: String;
    FTableCount : Integer;
    FInVerbatim : Boolean;
    Inlist,
    TableRowStartFlag,
    TableCaptionWritten: Boolean;
    // Linear documentation methods overrides;
    procedure WriteLabel(Const S : String); override;
    procedure WriteIndex(Const S : String); override;
    Procedure WriteExampleFile(FN : String); override;
    Procedure StartProcedure; override;
    Procedure EndProcedure; override;
    Procedure StartProperty; override;
    Procedure EndProperty; override;
    Procedure StartSynopsis; override;
    Procedure StartDeclaration; override;
    Procedure StartVisibility; override;
    Procedure StartDescription; override;
    Procedure StartAccess; override;
    Procedure StartErrors; override;
    Procedure StartVersion; override;
    Procedure StartSeealso; override;
    Procedure EndSeealso; override;
    procedure StartUnitOverview(AModuleName,AModuleLabel : String);override;
    procedure WriteUnitEntry(UnitRef : TPasType); override;
    Procedure EndUnitOverview; override;
    function  GetLabel(AElement: TPasElement): String; override;
    procedure StartListing(Frames: Boolean; const name: String); override;
    procedure EndListing; override;
    Function  EscapeText(S : String) : String; override;
    Function  StripText(S : String) : String; override;
    procedure WriteCommentLine; override;
    procedure WriteComment(Comment : String);override;
    procedure StartSection(SectionName : String);override;
    procedure StartSubSection(SubSectionName : String);override;
    procedure StartSubSubSection(SubSubSectionName : String);override;
    procedure StartChapter(ChapterName : String); override;
    procedure StartOverview(Const What : String; WithAccess : Boolean); override;
    procedure EndOverview; override;
    procedure WriteOverviewMember(const ALabel,AName,Access,ADescr : String); override;
    procedure WriteOverviewMember(const ALabel,AName,ADescr : String); override;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    Class Function FileNameExtension : String; override;
>>>>>>> graemeg/cpstrnew
    // Description node conversion
    procedure DescrBeginBold; override;
    procedure DescrEndBold; override;
    procedure DescrBeginItalic; override;
    procedure DescrEndItalic; override;
    procedure DescrBeginEmph; override;
    procedure DescrEndEmph; override;
    procedure DescrWriteImageEl(const AFileName, ACaption, ALinkName : DOMString); override;
    procedure DescrWriteFileEl(const AText: DOMString); override;
    procedure DescrWriteKeywordEl(const AText: DOMString); override;
    procedure DescrWriteVarEl(const AText: DOMString); override;
    procedure DescrBeginLink(const AId: DOMString); override;
    procedure DescrEndLink; override;
    procedure DescrBeginURL(const AURL: DOMString); override; // Provides a default implementation
    procedure DescrEndURL; override;
    procedure DescrWriteLinebreak; override;
    procedure DescrBeginParagraph; override;
    procedure DescrBeginCode(HasBorder: Boolean; const AHighlighterName: String); override;
    procedure DescrWriteCodeLine(const ALine: String); override;
    procedure DescrEndCode; override;
    procedure DescrEndParagraph; override;
    procedure DescrBeginOrderedList; override;
    procedure DescrEndOrderedList; override;
    procedure DescrBeginUnorderedList; override;
    procedure DescrEndUnorderedList; override;
    procedure DescrBeginDefinitionList; override;
    procedure DescrEndDefinitionList; override;
    procedure DescrBeginListItem; override;
    procedure DescrEndListItem; override;
    procedure DescrBeginDefinitionTerm; override;
    procedure DescrEndDefinitionTerm; override;
    procedure DescrBeginDefinitionEntry; override;
    procedure DescrEndDefinitionEntry; override;
    procedure DescrBeginSectionTitle; override;
    procedure DescrBeginSectionBody; override;
    procedure DescrEndSection; override;
    procedure DescrBeginRemark; override;
    procedure DescrEndRemark; override;
    procedure DescrBeginTable(ColCount: Integer; HasBorder: Boolean); override;
    procedure DescrEndTable; override;
    procedure DescrBeginTableCaption; override;
    procedure DescrEndTableCaption; override;
    procedure DescrBeginTableHeadRow; override;
    procedure DescrEndTableHeadRow; override;
    procedure DescrBeginTableRow; override;
    procedure DescrEndTableRow; override;
    procedure DescrBeginTableCell; override;
    procedure DescrEndTableCell; override;
    // TFPDocWriter class methods
    Property ImageDir : String Read FImageDir Write FImageDir;
  public
    Function InterPretOption(Const Cmd,Arg : String) : boolean; override;
<<<<<<< HEAD
<<<<<<< HEAD
    Class Function FileNameExtension : String; override;
=======
    Property ImageDir : String Read FImageDir Write FImageDir;
>>>>>>> graemeg/fixes_2_2
=======
    Property ImageDir : String Read FImageDir Write FImageDir;
>>>>>>> origin/fixes_2_2
  end;




function TLaTeXWriter.GetLabel(AElement: TPasElement): String;
var
  i: Integer;
begin
  if AElement.ClassType = TPasUnresolvedTypeRef then
    Result := Engine.ResolveLink(Module, AElement.Name)
  else
  begin
    Result := AElement.PathName;
    Result := LowerCase(Copy(Result, 2, Length(Result) - 1));
  end;
  for i := 1 to Length(Result) do
    if Result[i] = '.' then
      Result[i] := ':';
end;


function TLaTeXWriter.EscapeText(S: String): String;

var
  i: Integer;

begin
  if FInVerBatim=True then
    Result:=S
  else
    begin
    SetLength(Result, 0);
    for i := 1 to Length(S) do
      case S[i] of
        '&','{','}','#','_','$','%':            // Escape these characters
          Result := Result + '\' + S[i];
        '~','^':
          Result := Result + '\'+S[i]+' ';
        '\':
          Result:=Result+'$\backslash$'
        else
          Result := Result + S[i];
      end;
    end;
end;

function TLaTeXWriter.StripText(S: String): String;

var
  I: Integer;

begin
  SetLength(Result, 0);
  for i := 1 to Length(S) do
    If not (S[i] in ['&','{','}','#','_','$','%','''','~','^', '\']) then
      Result := Result + S[i]
    else
      Result:=result+'!'  
end;


procedure TLaTeXWriter.DescrBeginBold;
begin
  Write('\textbf{');
end;

procedure TLaTeXWriter.DescrEndBold;
begin
  Write('}');
end;

procedure TLaTeXWriter.DescrBeginItalic;
begin
  Write('\textit{');
end;

procedure TLaTeXWriter.DescrEndItalic;
begin
  Write('}');
end;

procedure TLaTeXWriter.DescrBeginEmph;
begin
  Write('\emph{');
end;

procedure TLaTeXWriter.DescrEndEmph;
begin
  Write('}');
end;

procedure TLaTeXWriter.DescrWriteImageEl(const AFileName, ACaption, ALinkName : DOMString); 

Var
  FN : String;
  L : Integer;
  
begin
  Writeln('\begin{figure}[ht]%');
  Writeln('\begin{center}');
  If (ACaption<>ACaption) then
    Writeln(Format('\caption{%s}',[EscapeText(ACaption)]));
  If (ALinkName<>'') then
    WriteLabel('fig:'+ALinkName);
  FN:=ImageDir;
  L:=Length(FN);
  If (L>0) and (FN[l]<>'/')  then
    FN:=FN+'/';
  FN:=FN+AFileName;
  Writeln('\epsfig{file='+FN+'}');
  Writeln('\end{center}');
  Writeln('\end{figure}');
end;

procedure TLaTeXWriter.DescrWriteFileEl(const AText: DOMString);
begin
  Write('\file{');
  DescrWriteText(AText);
  Write('}');
end;

procedure TLaTeXWriter.DescrWriteKeywordEl(const AText: DOMString);
begin
  Write('\textbf{\\ttfamily ');
  DescrWriteText(AText);
  Write('}');
end;

procedure TLaTeXWriter.DescrWriteVarEl(const AText: DOMString);
begin
  Write('\var{');
  DescrWriteText(AText);
  Write('}');
end;

procedure TLaTeXWriter.DescrBeginLink(const AId: DOMString);
begin
  FLink := Engine.ResolveLink(Module, AId);
//  System.WriteLn('Link "', AId, '" => ', FLink);
end;

procedure TLaTeXWriter.DescrEndLink;
begin
  WriteF(' (\pageref{%s})',[StripText(Flink)]);
end;

procedure TLaTeXWriter.DescrBeginURL(const AURL: DOMString);
begin
  Inherited; //  Save link
  Write('\htmladdnormallink{');
end;

procedure TLaTeXWriter.DescrEndURL;
begin
  WriteF('}{%s}',[LastURL]);
  LastURL:='';
end;

procedure TLaTeXWriter.DescrWriteLinebreak;
begin
  WriteLn('\\');
end;

procedure TLaTeXWriter.DescrBeginParagraph;
begin
  // Do nothing
end;

procedure TLaTeXWriter.DescrEndParagraph;
begin
  WriteLn('');
  WriteLn('');
end;

procedure TLaTeXWriter.DescrBeginCode(HasBorder: Boolean;
  const AHighlighterName: String);
begin
  StartListing(HasBorder,'');
end;

procedure TLaTeXWriter.DescrWriteCodeLine(const ALine: String);
begin
  WriteLn(ALine);
end;

procedure TLaTeXWriter.DescrEndCode;
begin
  EndListing
end;

procedure TLaTeXWriter.DescrBeginOrderedList;
begin
  WriteLn('\begin{enumerate}');
end;

procedure TLaTeXWriter.DescrEndOrderedList;
begin
  WriteLn('\end{enumerate}');
end;

procedure TLaTeXWriter.DescrBeginUnorderedList;
begin
  WriteLn('\begin{itemize}');
end;

procedure TLaTeXWriter.DescrEndUnorderedList;
begin
  WriteLn('\end{itemize}');
end;

procedure TLaTeXWriter.DescrBeginDefinitionList;
begin
  WriteLn('\begin{description}');
end;

procedure TLaTeXWriter.DescrEndDefinitionList;
begin
  WriteLn('\end{description}');
end;

procedure TLaTeXWriter.DescrBeginListItem;
begin
  Write('\item ');
end;

procedure TLaTeXWriter.DescrEndListItem;
begin
  WriteLn('');
end;

procedure TLaTeXWriter.DescrBeginDefinitionTerm;
begin
  Write('\item[');
end;

procedure TLaTeXWriter.DescrEndDefinitionTerm;
begin
  WriteLn(']');
end;

procedure TLaTeXWriter.DescrBeginDefinitionEntry;
begin
  // Do nothing
end;

procedure TLaTeXWriter.DescrEndDefinitionEntry;
begin
  WriteLn('');
end;

procedure TLaTeXWriter.DescrBeginSectionTitle;
begin
  Write('\subsection{');
end;

procedure TLaTeXWriter.DescrBeginSectionBody;
begin
  WriteLn('}');
end;

procedure TLaTeXWriter.DescrEndSection;
begin
  // Do noting
end;

procedure TLaTeXWriter.DescrBeginRemark;
begin
  WriteLn('\begin{remark}');
end;

procedure TLaTeXWriter.DescrEndRemark;
begin
  WriteLn('\end{remark}');
end;

procedure TLaTeXWriter.DescrBeginTable(ColCount: Integer; HasBorder: Boolean);
var
  i: Integer;
begin
  // !!!: How do we set the border?
  Write('\begin{FPCltable}{');
  for i := 1 to ColCount do
    Write('l');
  write('}{');
  TableCaptionWritten:=False;
end;

procedure TLaTeXWriter.DescrEndTable;
begin
  WriteLn('\end{FPCltable}');
end;

procedure TLaTeXWriter.DescrBeginTableCaption;
begin
  // Do nothing.
end;

procedure TLaTeXWriter.DescrEndTableCaption;
begin
  Write('}{table');
  Inc(FTableCount);
  Write(IntToStr(FTableCount));
  Writeln('}');
  TableCaptionWritten := True;
end;

procedure TLaTeXWriter.DescrBeginTableHeadRow;
begin
  if not TableCaptionWritten then
    DescrEndTableCaption;
  TableRowStartFlag := True;
end;

procedure TLaTeXWriter.DescrEndTableHeadRow;
begin
  WriteLn('\\ \hline');
end;

procedure TLaTeXWriter.DescrBeginTableRow;
begin
  if not TableCaptionWritten then
    DescrEndTableCaption;
  TableRowStartFlag := True;
end;

procedure TLaTeXWriter.DescrEndTableRow;
begin
  WriteLn('\\');
end;

procedure TLaTeXWriter.DescrBeginTableCell;
begin
  if TableRowStartFlag then
    TableRowStartFlag := False
  else
    Write(' & ');
end;

procedure TLaTeXWriter.DescrEndTableCell;
begin
  // Do nothing
end;

procedure TLaTeXWriter.WriteLabel(const S: String);
begin
  WriteLnF('\label{%s}', [LowerCase(StripText(s))]);
end;

procedure TLaTeXWriter.WriteIndex(const S: String);
begin
  Write('\index{');
  Write(EscapeText(s));
  Writeln('}');
end;

procedure TLaTeXWriter.StartListing(Frames: Boolean; const name: String);
begin
  FInVerbatim:=True;
  if Not LaTexHighLight then
    begin
    Writeln('');
    Writeln('\begin{verbatim}');
    end
  else
    if Frames then
      Writelnf('\begin{lstlisting}{%s}',[StripText(Name)])
    else
      Writelnf('\begin{lstlisting}[frame=]{%s}',[StripText(Name)]);
end;

procedure TLaTeXWriter.EndListing;
begin
  FInVerbatim:=False;
  If LaTexHighLight then
    Writeln('\end{lstlisting}')
  else
    Writeln('\end{verbatim}')
end;

procedure TLaTeXWriter.WriteCommentLine;
const
  CommentLine =
    '%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%';
begin
  Writeln(CommentLine);
end;

procedure TLaTeXWriter.WriteComment(Comment: String);
begin
  Write('% ');
  Writeln(Comment);
end;

procedure TLaTeXWriter.StartChapter(ChapterName: String);
begin
  WriteCommentLine;
  WriteComment(ChapterName);
  WriteCommentLine;
  Writeln('\chapter{'+EscapeText(ChapterName)+'}');
end;

procedure TLaTeXWriter.StartSection(SectionName: String);
begin
  WriteCommentLine;
  WriteComment(SectionName);
  Writeln('\section{'+EscapeText(SectionName)+'}');
end;

procedure TLaTeXWriter.StartSubSection(SubSectionName: String);
begin
  WriteComment(SubSectionName);
  Writeln('\subsection{'+EscapeText(SubSectionName)+'}');
end;

procedure TLaTeXWriter.StartSubSubSection(SubSubSectionName: String);
begin
  Writeln('\subsubsection{'+EscapeText(SubSubSectionName)+'}');
end;

procedure CreateLaTeXDocForPackage(APackage: TPasPackage; AEngine: TFPDocEngine);
var
  Writer: TLaTeXWriter;
begin
  Writer := TLaTeXWriter.Create(APackage, AEngine);
  try
    Writer.WriteDoc;
  finally
    Writer.Free;
  end;
end;

procedure TLaTeXWriter.StartProcedure;

begin
  Writeln('\begin{FPCList}');
  InList:=True;
end;

procedure TLaTeXWriter.StartSynopsis;

begin
  Writeln('\Synopsis');
end;

procedure TLaTeXWriter.StartDeclaration;

begin
  Writeln('\Declaration ');
end;

procedure TLaTeXWriter.StartVisibility;

begin
  Writeln('\Visibility');
end;

procedure TLaTeXWriter.StartDescription;

begin
  Writeln('\Description');
end;

procedure TLaTeXWriter.StartErrors;

begin
  Writeln('\Errors');
end;

procedure TLaTeXWriter.StartVersion;
begin
  Writeln('\VersionInfo');
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TLaTeXWriter.StartAccess;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Procedure TLatexWriter.StartAccess;
>>>>>>> graemeg/cpstrnew

begin
  Writeln('\Access')
end;

procedure TLaTeXWriter.EndProcedure;

begin
  InList:=False;
  Writeln('\end{FPCList}');
end;
procedure TLaTeXWriter.StartProperty;

begin
  Writeln('\begin{FPCList}');
  InList:=True;
end;

procedure TLaTeXWriter.EndProperty;

begin
  InList:=False;
  Writeln('\end{FPCList}');
end;

procedure TLaTeXWriter.WriteExampleFile(FN: String);

begin
  If (FN<>'') then
    WritelnF('\FPCexample{%s}', [ChangeFileExt(FN,'')]);
end;

procedure TLaTeXWriter.StartOverview(const What: String; WithAccess: Boolean);

begin
  If WithAccess then
    begin
    WriteLn('\begin{tabularx}{\textwidth}{lllX}');
    WriteLnF('%s & %s & %s & %s \\ \hline',[EscapeText(SDocPage), EscapeText(What), EscapeText(SDocAccess), EscapeText(SDocDescription)])
    end
  else
    begin
    WriteLn('\begin{tabularx}{\textwidth}{llX}');
    WriteLnF('%s & %s & %s  \\ \hline',[EscapeText(SDocPage), EscapeText(What), EscapeText(SDocDescription)])
    end;
end;

procedure TLaTeXWriter.EndOverview;

begin
  WriteLn('\hline');
  WriteLn('\end{tabularx}');
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TLaTeXWriter.WriteOverviewMember(const ALabel, AName, Access,
  ADescr: String);
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,Access,ADescr : String);
>>>>>>> graemeg/cpstrnew
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,Access,ADescr : String);
>>>>>>> graemeg/cpstrnew
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,Access,ADescr : String);
>>>>>>> graemeg/cpstrnew
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,Access,ADescr : String);
>>>>>>> origin/cpstrnew
=======
procedure TLatexWriter.WriteOverviewMember(ALabel,AName,Access,ADescr : String);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  WriteLnF('\pageref{%s} & %s & %s & %s \\',[ALabel,EscapeText(AName),Access,ADescr]);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure TLaTeXWriter.WriteOverviewMember(const ALabel, AName, ADescr: String);
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,ADescr : String);
>>>>>>> graemeg/cpstrnew
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,ADescr : String);
>>>>>>> graemeg/cpstrnew
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,ADescr : String);
>>>>>>> graemeg/cpstrnew
=======
procedure TLatexWriter.WriteOverviewMember(const ALabel,AName,ADescr : String);
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
procedure TLatexWriter.WriteOverviewMember(ALabel,AName,ADescr : String);
>>>>>>> graemeg/fixes_2_2

begin
  WriteLnF('\pageref{%s} & %s  & %s \\',[ALabel,EscapeText(AName),ADescr]);
end;

class function TLaTeXWriter.FileNameExtension: String;
begin
  Result:=TexExtension;
end;

procedure TLaTeXWriter.StartSeealso;

begin
  If not InList then
    begin
    Writeln('');
    Writeln('\begin{FPCList}');
    end;
  Writeln('\SeeAlso');
end;

procedure TLaTeXWriter.EndSeealso;
begin
  If Not InList then
    Writeln('\end{FPCList}');
end;

procedure TLaTeXWriter.StartUnitOverview(AModuleName, AModuleLabel: String);

begin
  WriteLnF('\begin{FPCltable}{lr}{%s}{%s:0units}',
    [Format(SDocUsedUnitsByUnitXY, [EscapeText(AModuleName)]), StripText(AModuleName)]);
  WriteLn('Name & Page \\ \hline');
end;

procedure TLaTeXWriter.WriteUnitEntry(UnitRef: TPasType);

begin
  WriteLnF('%s\index{unit!%s} & \pageref{%s} \\',
     [EscapeText(UnitRef.Name), EscapeText(UnitRef.Name), StripText(GetLabel(UnitRef))]);
end;

procedure TLaTeXWriter.EndUnitOverview;

begin
  WriteLn('\end{FPCltable}');
end;

function TLaTeXWriter.InterPretOption(const Cmd, Arg: String): boolean;

begin
  Result:=True;
  if (cmd= '--latex-highlight') then
    LatexHighLight:=True
  else if Cmd = '--latex-extension' then
     TexExtension:=Arg
  else if Cmd = '--image-dir' then
     ImageDir:=Arg
  else
    Result:=False;
end;

initialization
  // Do not localize.
  RegisterWriter(TLaTeXWriter,'latex','Latex output using fpc.sty class.');
finalization
  UnRegisterWriter('latex');
end.
