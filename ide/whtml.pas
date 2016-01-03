{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit WHTML;

{$I globdir.inc}

interface

uses Objects;

type
    PTextFile = ^TTextFile;
    TTextFile = object(TObject)
      function GetLine(Idx: sw_integer; var S: string): boolean; virtual;
      function GetFileName : string; virtual;
    end;

    PMemoryTextFile = ^TMemoryTextFile;
    TMemoryTextFile = object(TTextFile)
      constructor Init;
      procedure   AddLine(const S: string); virtual;
      function    GetLine(Idx: sw_integer; var S: string): boolean; virtual;
      function    GetFileName : string; virtual;
      function    GetLineCount : sw_integer;
      destructor  Done; virtual;
    private
      Lines : PUnsortedStrCollection;
    end;

    PDOSTextFile = ^TDOSTextFile;
    TDOSTextFile = object(TMemoryTextFile)
      constructor Init(AFileName: string);
      function GetFileName : string; virtual;
    private
      DosFileName : string;
    end;

    PSGMLParser = ^TSGMLParser;
    TSGMLParser = object(TObject)
      constructor Init;
      function    Process(HTMLFile: PTextFile): boolean; virtual;
      function    ProcessLine(LineText: string): boolean; virtual;
      destructor  Done; virtual;
    public
      Line,LinePos: sw_integer;
      procedure   DocSoftBreak; virtual;
      function    GetFileName : string;
      function    DocAddTextChar(C: char): boolean; virtual;
      procedure   DocAddText(S: string); virtual;
      procedure   DocProcessTag(Tag: string); virtual;
      procedure   DocProcessComment(Comment: string); virtual;
      function    DocDecodeNamedEntity(Name: string; var Entity: string): boolean; virtual;
    private
      CurTag: string;
      FileName : string;
      InTag,InComment,InString: boolean;
    end;

    PHTMLParser = ^THTMLParser;
    THTMLParser = object(TSGMLParser)
      procedure   DocSoftBreak; virtual;
      function    DocAddTextChar(C: char): boolean; virtual;
      procedure   DocProcessTag(Tag: string); virtual;
      function    DocGetTagParam(Name: string; var Value: string): boolean; virtual;
      procedure   DocProcessComment(Comment: string); virtual;
      function    DocDecodeNamedEntity(Name: string; var E: string): boolean; virtual;
    public
      TagName,TagParams: string;
      DisableCrossIndexing : boolean;
      procedure   DocUnknownTag; virtual;
      procedure   DocTYPE; virtual;
      procedure   DocHTML(Entered: boolean); virtual;
      procedure   DocHEAD(Entered: boolean); virtual;
      procedure   DocMETA; virtual;
      procedure   DocTITLE(Entered: boolean); virtual;
      procedure   DocBODY(Entered: boolean); virtual;
      procedure   DocAnchor(Entered: boolean); virtual;
      procedure   DocHeading(Level: integer; Entered: boolean); virtual;
      procedure   DocParagraph(Entered: boolean); virtual;
      procedure   DocBreak; virtual;
      procedure   DocImage; virtual;
      procedure   DocBold(Entered: boolean); virtual;
      procedure   DocCite(Entered: boolean); virtual;
      procedure   DocCode(Entered: boolean); virtual;
      procedure   DocEmphasized(Entered: boolean); virtual;
      procedure   DocItalic(Entered: boolean); virtual;
      procedure   DocKbd(Entered: boolean); virtual;
      procedure   DocPreformatted(Entered: boolean); virtual;
      procedure   DocSample(Entered: boolean); virtual;
      procedure   DocStrong(Entered: boolean); virtual;
      procedure   DocTeleType(Entered: boolean); virtual;
      procedure   DocVariable(Entered: boolean); virtual;
      procedure   DocSpan(Entered: boolean); virtual;
      procedure   DocDiv(Entered: boolean); virtual;
      procedure   DocList(Entered: boolean); virtual;
      procedure   DocOrderedList(Entered: boolean); virtual;
      procedure   DocListItem(Entered: boolean); virtual;
      procedure   DocDefList(Entered: boolean); virtual;
      procedure   DocDefTerm(Entered: boolean); virtual;
      procedure   DocDefExp(Entered: boolean); virtual;
      procedure   DocTable(Entered: boolean); virtual;
      procedure   DocTableRow(Entered: boolean); virtual;
      procedure   DocTableHeaderItem(Entered: boolean); virtual;
      procedure   DocTableItem(Entered: boolean); virtual;
      procedure   DocHorizontalRuler; virtual;
    end;

Type
    PTopicLinkCollection = ^TTopicLinkCollection;
    TTopicLinkCollection = object(TStringCollection)
      procedure   Insert(Item: Pointer); virtual;
      function    At(Index: sw_Integer): PString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      function    AddItem(Item: string): sw_integer;
=======
      function    AddItem(Item: string): integer;
>>>>>>> graemeg/fixes_2_2
=======
      function    AddItem(Item: string): integer;
>>>>>>> origin/fixes_2_2
=======
      function    AddItem(Item: string): sw_integer;
>>>>>>> origin/fixes_2.4
    end;

function EncodeHTMLCtx(FileID: integer; LinkNo: word): longint;
procedure DecodeHTMLCtx(Ctx: longint; var FileID: word; var LinkNo: word);


implementation

uses
  WUtils;

function TTextFile.GetLine(Idx: sw_integer; var S: string): boolean;
begin
  Abstract;
  GetLine:=false;
end;

function TTextFile.GetFileName : string;
begin
  GetFileName:='unknown';
end;

constructor TMemoryTextFile.Init;
begin
  inherited Init;
  New(Lines, Init(500,500));
end;


function TMemoryTextFile.GetFileName : string;
begin
  GetFileName:='unknown';
end;

function TMemoryTextFile.GetLineCount : sw_integer;
begin
  GetLineCount:=Lines^.Count;
end;

procedure TMemoryTextFile.AddLine(const S: string);
begin
  Lines^.Insert(NewStr(S));
end;

function TMemoryTextFile.GetLine(Idx: sw_integer; var S: string): boolean;
var OK: boolean;
    PS: PString;
begin
  OK:=(Lines<>nil) and (Idx<Lines^.Count);
  if OK then
    begin
      PS:=Lines^.At(Idx);
      if PS=nil then S:='' else S:=PS^;
    end;
  GetLine:=OK;
end;

destructor TMemoryTextFile.Done;
begin
  if Lines<>nil then
    Dispose(Lines, Done);
  Lines:=nil;
  inherited Done;
end;

constructor TDOSTextFile.Init(AFileName: string);
var f: file;
    linecomplete,hasCR: boolean;
    S: string;
    OldFMode : Integer;
begin
  inherited Init;
  if AFileName='' then Fail;
{$I-}
  Assign(f,AFileName);
  OldFMode:= FileMode;
  FileMode:= 0;
  Reset(f,1);
  FileMode:= OldFMode;
  if IOResult<>0 then Fail;
  DosFileName:=AFileName;
  Dispose(Lines,Done);
  New(Lines, Init(500,2000));
  while (Eof(f)=false) and (IOResult=0) do
    begin
      ReadlnFromFile(f,S,linecomplete,hasCR,true);
      AddLine(S);
    end;
  Close(f);
{$I+}
end;

function TDosTextFile.GetFileName : string;
begin
  GetFileName:=DosFileName;
end;


constructor TSGMLParser.Init;
begin
  inherited Init;
  FileName:='';
end;

function TSGMLParser.GetFileName : string;
begin
  GetFileName:=FileName;
end;

function TSGMLParser.Process(HTMLFile: PTextFile): boolean;
var S: string;
    OK,LineOK: boolean;
begin
  if HTMLFile=nil then Exit;
  InTag:=false; InComment:=false; InString:=false; CurTag:='';
  Line:=0; OK:=true;
  FileName:=HTMLFile^.GetFileName;
  repeat
    LineOK:=HTMLFile^.GetLine(Line,S);
    if LineOK then
      begin
        Inc(Line);
        OK:=ProcessLine(S);
      end;
  until (LineOK=false) or (OK=false);
  Process:=OK;
end;

function TSGMLParser.ProcessLine(LineText: string): boolean;
var OK: boolean;
    C: char;
    NewInString: boolean;
    OldInComment: boolean;
    WasThereAnyText: boolean;
    Pos2: integer;
    Name,Entity: string;
    LiteralCode: boolean;
    LiteralStart,LiteralEnd,P: integer;
const TabSize : integer = 8;
      Tab = #9;
begin
  WasThereAnyText:=false;
  OK:=true; LinePos:=1;
  LiteralStart:=0; LiteralEnd:=0;
  repeat
    P:=Pos(TAB,LineText);
    if P>0 then
      LineText:=copy(LineText,1,P-1)+CharStr(' ',TabSize)+copy(LineText,P+1,255);
  until P=0;
  while (LinePos<=length(LineText)) and OK do
    begin
      LiteralCode:=false;
      NewInString:=InString; OldInComment:=InComment;
      C:=LineText[LinePos];

      LiteralCode:=(LiteralStart<=LinePos) and (LinePos<=LiteralEnd);

      if (LiteralCode=false) and (C='&') then
        begin
          LiteralStart:=0; LiteralEnd:=0;
          Name:=''; Pos2:=LinePos+1;
          while (Pos2<=length(LineText)) and (LineText[Pos2]<>';') do
            begin
              Name:=Name+LineText[Pos2];
              Inc(Pos2);
            end;
          Inc(Pos2);
          if DocDecodeNamedEntity(Name,Entity) then
            begin
              LineText:=copy(LineText,1,LinePos-1)+Entity+copy(LineText,Pos2,255);
              LiteralStart:=LinePos; LiteralEnd:=LiteralStart+length(Entity)-1;
              C:=LineText[LinePos];
            end;
        end;

      LiteralCode:=(LiteralStart<=LinePos) and (LinePos<=LiteralEnd);

      if (LiteralCode=false) and (C='"') and (InTag=true) and (InString=false) then
        NewInString:=true;
      if (LiteralCode=false) and (C='<') and (InTag=false) then
        InTag:=true;

      if InTag then CurTag:=CurTag+C else
        WasThereAnyText:=DocAddTextChar(C);
      if (LiteralCode=false) and InTag and (InString=false) and (CurTag='<!--') then
        InComment:=true;
      { A comment can be longer than 255 chars
        move the test to LineText string,
        This is why the Previous, Next and Up Tags where not working ... PM
      if (LiteralCode=false) and InTag and InComment and (InString=false) and (length(CurTag)>=3) and
         (copy(CurTag,length(CurTag)-2,3)='-->') then
           InComment:=false; }
      if (LiteralCode=false) and InTag and InComment and (InString=false) and (LinePos>=3) and
         (copy(LineText,LinePos-2,3)='-->') then
           InComment:=false;

      if (LiteralCode=false) and (C='"') and (InTag=true) and (InString=true) then
        NewInString:=false;
      if (LiteralCode=false) and (C='>') and (InTag=true) then
        begin
          InTag:=false;
          if OldInComment then
            DocProcessComment(CurTag)
          else
            DocProcessTag(CurTag);
          CurTag:='';
        end;

      InString:=NewInString;
      Inc(LinePos);

    end;
  { whtml does not depend on whelp,
    so I can not use hscLineBreak here. PM }
  if InTag then
    begin
      if InString then
        CurTag:=CurTag+#0;
    end
  else if WasThereAnyText then
    DocSoftBreak;

  ProcessLine:=true;
end;

procedure TSGMLParser.DocSoftBreak;
begin
  Abstract;
end;

function TSGMLParser.DocAddTextChar(C: char): boolean;
begin
  Abstract;
  DocAddTextChar:=false;
end;

procedure TSGMLParser.DocAddText(S: string);
var I: sw_integer;
begin
  for I:=1 to length(S) do
    DocAddTextChar(S[I]);
end;

function TSGMLParser.DocDecodeNamedEntity(Name: string; var Entity: string): boolean;
begin
  DocDecodeNamedEntity:=false;
end;

procedure TSGMLParser.DocProcessTag(Tag: string);
begin
  Abstract;
end;

procedure TSGMLParser.DocProcessComment(Comment: string);
begin
  Abstract;
end;

destructor TSGMLParser.Done;
begin
  inherited Done;
end;

procedure THTMLParser.DocSoftBreak;
begin
end;

function THTMLParser.DocAddTextChar(C: char): boolean;
begin
  { Abstract }
  DocAddTextChar:=false;
end;

function THTMLParser.DocDecodeNamedEntity(Name: string; var E: string): boolean;
var Found: boolean;
    Code: word;
    CC: word;
begin
  Found:=true; Code:=$ffff;
  Name:=LowCaseStr(Name);
  if copy(Name,1,1)='#' then
    begin
      if Name[2]='x' then
        Val('$'+copy(Name,3,255),Code,CC)
      else
        Val(copy(Name,2,255),Code,CC);
      if CC<>0 then
        begin
{$ifdef DEBUG}
          DebugMessage(FileName,'NamedEntity '+Name+' not converted',1,1);
{$endif DEBUG}
          Code:=$ffff;
        end;
    end;
  { #0 to #127 is same for Unicode and Code page 437 }
  if (code<=127) then
    begin
      E:=chr(code);
      DocDecodeNamedEntity:=true;
      exit;
    end;
  if (Code=$22{34}) or (Name='quot')   then E:='"'   else { double quote sign             }
  if (Code=$26{38}) or (Name='amp')    then E:='&'   else { ampersand                     }
  if (Code=$27{39}) or (Name='apos')    then E:='''' else { apostrophe  }
  if (Code=$3C{60}) or (Name='lt')     then E:='<'   else { less-than sign                }
  if (Code=$3E{62}) or (Name='gt')     then E:='>'   else { greater-than sign              }
  if (Code=$5B)                    then E:='['   else { [ }
  if (Code=$5C)                    then E:='\'   else { \ }
  if (Code=$5D)                    then E:=']'   else { ] }
  if (Code=$5E)                    then E:='^'   else { ^ }
  if (Code=$5F)                    then E:='_'   else { _ }
  if (Code=160) or (Name='nbsp')   then E:=#255  else { no-break space                }
  if (Code=161) or (Name='iexcl')  then E:='­'   else { inverted exclamation mark    }
  if (Code=162) or (Name='cent')   then E:='›'   else { cent sign                     }
  if (Code=163) or (Name='pound')  then E:='œ'   else { pound sterling sign           }
  if (Code=164) or (Name='curren') then E:='$'   else { general currency sign         }
  if (Code=165) or (Name='yen')    then E:=''   else { yen sign                      }
  if (Code=166) or (Name='brvbar') then E:='|'   else { broken vertical bar           }
  if (Code=167) or (Name='sect')   then E:=''   else { section sign                  }
  if (Code=168) or (Name='uml')    then E:='"'   else { umlaut  (dieresis)            }
  if (Code=169) or (Name='copy')   then E:='(C)' else { copyright sign                }
(*  if (Code=170) or (Name='ordf')   then E:=#255  else { ordinal indicator, feminine   }*)
  if (Code=171) or (Name='laquo')  then E:='"'   else { angle quotation mark -left    }
  if (Code=172) or (Name='not')    then E:='!'   else { not sign                      }
  if (Code=173) or (Name='shy')    then E:='-'   else { soft hypen                    }
  if (Code=174) or (Name='reg')    then E:='(R)' else { registered sign               }
(*  if (Code=175) or (Name='macr')   then E:='?'   else { macron                        }*)
  if (Code=176) or (Name='deg')    then E:='ø'   else { degree sign                   }
  if (Code=177) or (Name='plusmn') then E:='ñ'   else { plus-or-minus sign            }
  if (Code=178) or (Name='sup2')   then E:='ý'   else { superscript 2                 }
  if (Code=179) or (Name='sup3')   then E:='^3'  else { superscript 3                 }
  if (Code=180) or (Name='acute')  then E:=''''  else { acute accent                  }
  if (Code=181) or (Name='micro')  then E:='æ'   else { micro sign                    }
(*  if (Code=182) or (Name='para')   then E:='?'   else { paragraph sign                }*)
  if (Code=183) or (Name='middot') then E:='ù'   else { middle dot                    }
(*  if (Code=184) or (Name='cedil')  then E:='?'   else { cedilla                       }*)
  if (Code=185) or (Name='sup1')   then E:='^1'  else { superscript 1                 }
(*  if (Code=186) or (Name='ordm')   then E:='?'   else { ordinal indicator, masculine  }*)
  if (Code=187) or (Name='raquo')  then E:='"'   else { angle quoatation mark -right  }
  if (Code=188) or (Name='frac14') then E:='¬'   else { fraction one-quarter          }
  if (Code=189) or (Name='frac12') then E:='«'   else { fraction one-half             }
  if (Code=190) or (Name='frac34') then E:='3/4' else { fraction three-quarters       }
  if (Code=191) or (Name='iquest') then E:='¨'   else { inverted question mark        }
  if (Code=192) or (Name='Agrave') then E:='A'   else { capital A, grave accent       }
  if (Code=193) or (Name='Aacute') then E:='A'   else { capital A, acute accent       }
  if (Code=194) or (Name='Acirc')  then E:='A'   else { capital A, circumflex accent  }
  if (Code=195) or (Name='Atilde') then E:='A'   else { capital A, tilde accent       }
  if (Code=196) or (Name='Auml')   then E:='Ž'   else { capital A, dieresis or umlaut }
  if (Code=197) or (Name='Aring')  then E:=''   else { capital A, ring               }
  if (Code=198) or (Name='AElig')  then E:='’'   else { capital AE diphthong          }
  if (Code=199) or (Name='Ccedil') then E:='€'   else { capital C, cedilla            }
  if (Code=200) or (Name='Egrave') then E:=''   else { capital E, grave accent       }
  if (Code=201) or (Name='Eacute') then E:=''   else { capital E, acute accent       }
  if (Code=202) or (Name='Ecirc')  then E:='E'   else { capital E, circumflex accent  }
  if (Code=203) or (Name='Euml')   then E:='E'   else { capital E, dieresis or umlaut }
  if (Code=204) or (Name='Igrave') then E:='I'   else { capital I, grave accent       }
  if (Code=205) or (Name='Iacute') then E:='I'   else { capital I, acute accent       }
  if (Code=206) or (Name='Icirc')  then E:='I'   else { capital I, circumflex accent  }
  if (Code=207) or (Name='Iuml')   then E:='I'   else { capital I, dieresis or umlaut }
(*  if (Code=208) or (Name='ETH')    then E:='?'   else { capital Eth, Icelandic        }*)
  if (Code=209) or (Name='Ntidle') then E:='¥'   else { capital N, tilde              }
  if (Code=210) or (Name='Ograve') then E:='O'   else { capital O, grave accent       }
  if (Code=211) or (Name='Oacute') then E:='O'   else { capital O, acute accent       }
  if (Code=212) or (Name='Ocirc')  then E:='O'   else { capital O, circumflex accent  }
  if (Code=213) or (Name='Otilde') then E:='O'   else { capital O, tilde              }
  if (Code=214) or (Name='Ouml')   then E:='™'   else { capital O, dieresis or umlaut }
  if (Code=215) or (Name='times')  then E:='*'   else { multiply sign                 }
  if (Code=216) or (Name='Oslash') then E:='O'   else { capital O, slash              }
  if (Code=217) or (Name='Ugrave') then E:='U'   else { capital U, grave accent       }
  if (Code=218) or (Name='Uacute') then E:='U'   else { capital U, acute accent       }
  if (Code=219) or (Name='Ucirc')  then E:='U'   else { capital U, circumflex accent  }
  if (Code=220) or (Name='Uuml')   then E:='š'   else { capital U, dieresis or umlaut }
  if (Code=221) or (Name='Yacute') then E:='Y'   else { capital Y, acute accent       }
(*  if (Code=222) or (Name='THORN')  then E:='?'   else { capital THORN, Icelandic      }*)
  if (Code=223) or (Name='szlig')  then E:='á'   else { small sharp S, German         }
  if (Code=224) or (Name='agrave') then E:='…'   else { small a, grave accent         }
  if (Code=225) or (Name='aacute') then E:=' '   else { small a, acute accent         }
  if (Code=226) or (Name='acirc')  then E:='ƒ'   else { small a, circumflex accent    }
  if (Code=227) or (Name='atilde') then E:='ƒ'   else { small a, tilde                }
  if (Code=228) or (Name='auml')   then E:='„'   else { small a, dieresis or umlaut   }
  if (Code=229) or (Name='aring')  then E:='†'   else { small a, ring                 }
  if (Code=230) or (Name='aelig')  then E:='ae'  else { small ae, diphthong           }
  if (Code=231) or (Name='ccedil') then E:='‡'   else { small c, cedilla              }
  if (Code=232) or (Name='egrave') then E:='Š'   else { small e, grave accent         }
  if (Code=233) or (Name='eacute') then E:='‚'   else { small e, acute accent         }
  if (Code=234) or (Name='ecirc')  then E:='ˆ'   else { small e, circumflex accent    }
  if (Code=235) or (Name='euml')   then E:='‰'   else { small e, dieresis or umlaut   }
  if (Code=236) or (Name='igrave') then E:=''   else { small i, grave accent         }
  if (Code=237) or (Name='iacute') then E:='¡'   else { small i, acute accent         }
  if (Code=238) or (Name='icirc')  then E:='Œ'   else { small i, circumflex accent    }
  if (Code=239) or (Name='iuml')   then E:='‹'   else { small i, dieresis or umlaut   }
(*  if (Code=240) or (Name='eth')    then E:='?'   else { small eth, Icelandic          }*)
  if (Code=241) or (Name='ntilde') then E:='¤'   else { small n, tilde                }
  if (Code=242) or (Name='ograve') then E:='•'   else { small o, grave accent         }
  if (Code=243) or (Name='oacute') then E:='¢'   else { small o, acute accent         }
  if (Code=244) or (Name='ocirc')  then E:='“'   else { small o, circumflex accent    }
  if (Code=245) or (Name='otilde') then E:='“'   else { small o, tilde                }
  if (Code=246) or (Name='ouml')   then E:='”'   else { small o, dieresis or umlaut   }
  if (Code=247) or (Name='divide') then E:='/'   else { divide sign                   }
  if (Code=248) or (Name='oslash') then E:='"'   else { small o, slash                }
  if (Code=249) or (Name='ugrave') then E:='—'   else { small u, grave accent         }
  if (Code=250) or (Name='uacute') then E:='£'   else { small u, acute accent         }
  if (Code=251) or (Name='ucirc')  then E:='–'   else { small u, circumflex accent    }
  if (Code=252) or (Name='uuml')   then E:=''   else { small u, dieresis or umlaut   }
  if (Code=253) or (Name='yacute') then E:='y'   else { small y, acute accent         }
(*  if (Code=254) or (Name='thorn')  then E:='?'   else { small thorn, Icelandic        }*)
  if (Code=255) or (Name='yuml')   then E:='y'   else { small y, dieresis or umlaut   }
  { Special codes appearing in TeXH generated files }
  if (code=$2c6{710}) or (Name='circ')  then E:='^' else      { Modifier Letter Circumflex Accent }
  if (code=$2dc{732}) or (Name='tilde') then E:='~' else      { Small tilde }
  if (code=$2013{8211}) or (Name='endash') then E:='-' else   { En dash }
  if (code=$2014{8212}) or (Name='emdash') then E:='--' else  { Em dash }
  if (Code=$2018{8216}) or (Name='lsquo') then E:='`'  else   { Acute accent as generated by TeXH   }
  if (Code=$2019{8217}) or (Name='rsquo') then E:='''' else   { acute accent as generated by TeXH   }
  if (code=$201C{8220}) or (Name='ldquo') then E:='''''' else { left double quotation marks }
  if (code=$201D{8221}) or (Name='rdquo') then E:='``' else   { right double quotation marks }
  if (code=$2026{8230}) or (Name='hellip') then E:='...' else { horizontal ellipsis }
  if (Code=$FB00) then E:='ff'  else                  { ff together }
  if (Code=$FB01) then E:='fi'  else                  { fi together }
  if (Code=$FB02) then E:='fl'  else                  { fl together }
  if (Code=$FB03) then E:='ffi' else                  { ffi together }
  if (Code=$FB04) then E:='ffl' else                  { ffl together }
  Found:=false;
  DocDecodeNamedEntity:=Found;
{$ifdef DEBUG}
  if (Code<>$ffff) and not found then
    begin
      DebugMessage(FileName,'NamedEntity '+Name+' not handled',1,1);
    end;
{$endif DEBUG}
end;

procedure THTMLParser.DocProcessTag(Tag: string);
var UTagName,ETagName: string[30];
    P: byte;
    NotEndTag: boolean;
begin
  if copy(Tag,1,1)='<' then Delete(Tag,1,1);
  if copy(Tag,length(Tag),1)='>' then Delete(Tag,length(Tag),1);
  Tag:=Trim(Tag);
  P:=Pos(' ',Tag); if P=0 then P:=length(Tag)+1;
  TagName:=copy(Tag,1,P-1); TagParams:=copy(Tag,P+1,255);
  UTagName:=UpcaseStr(TagName);
  NotEndTag:=copy(TagName,1,1)<>'/';
  if NotEndTag then ETagName:=UTagName else ETagName:=copy(UTagName,2,255);
  { <BR/> is also a Break tag... }
  if Copy(ETagName,Length(ETagName),1)='/' then
    begin
      ETagName:=copy(ETagName,1,Length(ETagName)-1);
      NotEndTag:=false;
    end;

  if (UTagName='!DOCTYPE') then DocTYPE else
  { Section tags }
  if (ETagName='HTML') then DocHTML(NotEndTag) else
  if (ETagName='HEAD') then DocHEAD(NotEndTag) else
  if (ETagName='TITLE') then DocTITLE(NotEndTag) else
  if (ETagName='BODY') then DocBODY(NotEndTag) else
  { Anchor tags }
  if (ETagName='A') then DocAnchor(NotEndTag) else
  { Direct formatting directives }
  if (ETagName='H1') then DocHeading(1,NotEndTag) else
  if (ETagName='H2') then DocHeading(2,NotEndTag) else
  if (ETagName='H3') then DocHeading(3,NotEndTag) else
  if (ETagName='H4') then DocHeading(4,NotEndTag) else
  if (ETagName='H5') then DocHeading(5,NotEndTag) else
  if (ETagName='H6') then DocHeading(6,NotEndTag) else
  if (ETagName='P') then DocParagraph(NotEndTag) else
  if (ETagName='BR') then DocBreak else
  if (ETagName='B') then DocBold(NotEndTag) else
  if (ETagName='CITE') then DocCite(NotEndTag) else
  if (ETagName='CODE') then DocCode(NotEndTag) else
  if (ETagName='EM') then DocEmphasized(NotEndTag) else
  if (ETagName='I') then DocItalic(NotEndTag) else
  if (ETagName='KBD') then DocKbd(NotEndTag) else
  if (ETagName='PRE') then DocPreformatted(NotEndTag) else
  if (ETagName='SAMP') then DocSample(NotEndTag) else
  if (ETagName='STRONG') then DocStrong(NotEndTag) else
  if (ETagName='TT') then DocTeleType(NotEndTag) else
  if (ETagName='VAR') then DocVariable(NotEndTag) else
  if (ETagName='SPAN') then DocSpan(NotEndTag) else
  if (ETagName='DIV') then DocDiv(NotEndTag) else
  { Unordered & ordered lists }
  if (ETagName='UL') then DocList(NotEndTag) else
  if (ETagName='OL') then DocOrderedList(NotEndTag) else
  if (ETagName='LI') then DocListItem(NotEndTag) else
  { Definition list }
  if (ETagName='DL') then DocDefList(NotEndTag) else
  if (ETagName='DT') then DocDefTerm(NotEndTag) else
  if (ETagName='DD') then DocDefExp(NotEndTag) else
  { Table }
  if (ETagName='TABLE') then DocTable(NotEndTag) else
  if (ETagName='TR') then DocTableRow(NotEndTag) else
  if (ETagName='TH') then DocTableHeaderItem(NotEndTag) else
  if (ETagName='TD') then DocTableItem(NotEndTag) else
  { Misc. tags }
  if (UTagName='META') then DocMETA else
  if (UTagName='IMG') then DocImage else
  if (UTagName='HR') then DocHorizontalRuler else
  DocUnknownTag;
end;

function THTMLParser.DocGetTagParam(Name: string; var Value: string): boolean;
var Found: boolean;
    S: string;
    ParamName,ParamValue: string;
    InStr: boolean;
    I: sw_integer;
begin
  Found:=false;
  Name:=UpcaseStr(Name);
  Value:='';
  S:=TagParams;
  repeat
    InStr:=false;
    ParamName:=''; ParamValue:='';
    S:=Trim(S); I:=1;
    while (I<=length(S)) and (S[I]<>'=') do
      begin
        if S[I]=' ' then
          ParamName:=''
        else
          ParamName:=ParamName+S[I];
        Inc(I);
      end;
    ParamName:=Trim(ParamName);
    if S[I]='=' then
    begin
      Inc(I); InStr:=false;
      while (I<=length(S)) and (S[I]=' ') do
        Inc(I);
      if (I<=length(S)) and (S[I]='"') then
        begin
          InStr:=true;
          Inc(I);
        end;
      while (I<=length(S)) and ((InStr=true) or (S[I]<>' ')) do
        begin
          if S[I]='"' then
            begin
              InStr:=not InStr;
              if InStr=false then Break;
            end
          else
            ParamValue:=ParamValue+S[I];
          Inc(I);
        end;
    end;
    Found:=(Name=UpcaseStr(ParamName));
    if Found then Value:=ParamValue;
    Delete(S,1,I);
  until Found or (S='');
  DocGetTagParam:=Found;
end;

procedure THTMLParser.DocProcessComment(Comment: string);
begin
end;

procedure THTMLParser.DocUnknownTag;
begin
end;

procedure THTMLParser.DocTYPE;
begin
end;

procedure THTMLParser.DocHTML(Entered: boolean);
begin
end;

procedure THTMLParser.DocHEAD(Entered: boolean);
begin
end;

procedure THTMLParser.DocMETA;
begin
end;

procedure THTMLParser.DocTITLE(Entered: boolean);
begin
end;

procedure THTMLParser.DocBODY(Entered: boolean);
begin
end;

procedure THTMLParser.DocAnchor(Entered: boolean);
begin
end;

procedure THTMLParser.DocHeading(Level: integer; Entered: boolean);
begin
end;

procedure THTMLParser.DocParagraph(Entered: boolean);
begin
end;

procedure THTMLParser.DocBreak;
begin
end;

procedure THTMLParser.DocImage;
begin
end;

procedure THTMLParser.DocBold(Entered: boolean);
begin
end;

procedure THTMLParser.DocCite(Entered: boolean);
begin
end;

procedure THTMLParser.DocCode(Entered: boolean);
begin
end;

procedure THTMLParser.DocEmphasized(Entered: boolean);
begin
end;

procedure THTMLParser.DocItalic(Entered: boolean);
begin
end;

procedure THTMLParser.DocKbd(Entered: boolean);
begin
end;

procedure THTMLParser.DocPreformatted(Entered: boolean);
begin
end;

procedure THTMLParser.DocSample(Entered: boolean);
begin
end;

procedure THTMLParser.DocStrong(Entered: boolean);
begin
end;

procedure THTMLParser.DocTeleType(Entered: boolean);
begin
end;

procedure THTMLParser.DocVariable(Entered: boolean);
begin
end;

procedure THTMLParser.DocSpan(Entered: boolean);
begin
end;

procedure THTMLParser.DocDiv(Entered: boolean);
var
  S: String;
begin
  if Entered then
    begin
      if DocGetTagParam('CLASS',S) then
        if S='crosslinks' then
          begin
            DisableCrossIndexing:=true;
{$ifdef DEBUG}
          DebugMessage(GetFileName,'Crosslinks found',Line,LinePos);
{$endif DEBUG}
          end;
    end
  else
    begin
{$ifdef DEBUG}
      if DisableCrossIndexing then
        begin
          DebugMessage(GetFileName,'Crosslinks end found',Line,LinePos);
        end;
{$endif DEBUG}
      DisableCrossIndexing:=false;
    end;
end;

procedure THTMLParser.DocList(Entered: boolean);
begin
end;

procedure THTMLParser.DocOrderedList(Entered: boolean);
begin
end;

procedure THTMLParser.DocListItem(Entered: boolean);
begin
end;

procedure THTMLParser.DocDefList(Entered: boolean);
begin
end;

procedure THTMLParser.DocDefTerm(Entered: boolean);
begin
end;

procedure THTMLParser.DocDefExp(Entered: boolean);
begin
end;

procedure THTMLParser.DocTable(Entered: boolean);
var
  S: String;
begin
  if Entered then
    begin
      if DocGetTagParam('CLASS',S) then
        if S='bar' then
          begin
            DisableCrossIndexing:=true;
{$ifdef DEBUG}
          DebugMessage(GetFileName,'Bar table found, cross indexing disabled ',Line,LinePos);
{$endif DEBUG}
          end;
    end
  else
    begin
{$ifdef DEBUG}
      if DisableCrossIndexing then
        begin
          DebugMessage(GetFileName,'Bar table end found',Line,LinePos);
        end;
{$endif DEBUG}
      DisableCrossIndexing:=false;
    end;
end;


procedure THTMLParser.DocTableRow(Entered: boolean);
begin
end;

procedure THTMLParser.DocTableHeaderItem(Entered: boolean);
begin
end;

procedure THTMLParser.DocTableItem(Entered: boolean);
begin
end;

procedure THTMLParser.DocHorizontalRuler;
begin
end;

function EncodeHTMLCtx(FileID: integer; LinkNo: word): longint;
var Ctx: longint;
begin
  Ctx:=(longint(FileID) shl 16)+LinkNo;
  EncodeHTMLCtx:=Ctx;
end;

procedure DecodeHTMLCtx(Ctx: longint; var FileID: word; var LinkNo: word);
begin
  if (Ctx shr 16)=0 then
    begin
      FileID:=$ffff; LinkNo:=0;
    end
  else
    begin
      FileID:=Ctx shr 16; LinkNo:=Ctx and $ffff;
    end;
end;


procedure TTopicLinkCollection.Insert(Item: Pointer);
begin
  AtInsert(Count,Item);
end;

function TTopicLinkCollection.At(Index: sw_Integer): PString;
begin
  At:=inherited At(Index);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TTopicLinkCollection.AddItem(Item: string): sw_integer;
=======
function TTopicLinkCollection.AddItem(Item: string): integer;
>>>>>>> graemeg/fixes_2_2
=======
function TTopicLinkCollection.AddItem(Item: string): integer;
>>>>>>> origin/fixes_2_2
=======
function TTopicLinkCollection.AddItem(Item: string): sw_integer;
>>>>>>> origin/fixes_2.4
var Idx: sw_integer;
begin
  if Item='' then Idx:=-1 else
  if Search(@Item,Idx)=false then
    begin
      AtInsert(Count,NewStr(Item));
      Idx:=Count-1;
    end;
  AddItem:=Idx;
end;


END.
