{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2014 by Michael Van Canneyt

    Pascal to Javascript converter class.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************
}(*
Abstract:
  Pascal to JavaScript source map.


*)
unit FPPJsSrcMap;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, math,
  jswriter, jstree, JSSrcMap;

type
  { TPas2JSSrcMap }

  TPas2JSSrcMap = class(TSourceMap)
  private
    fRefCount: integer;
  public
    LocalFilename: string;
    procedure AddRef;
    procedure Release;
  end;

  { TPas2JSMapper }

  TPas2JSMapper = class(TBufferWriter)
  private
    FDestFileName: String;
    FSrcMap: TPas2JSSrcMap;
    procedure SetSrcMap(const AValue: TPas2JSSrcMap);
  protected
    FNeedMapping: boolean;
    FGeneratedStartLine: integer; // first line where CurElement was set or a line was written
    // last valid CurElement position
    FSrcFilename: String;
    FSrcLine: integer;
    FSrcColumn: integer;
    procedure SetCurElement(const AValue: TJSElement); override;
    procedure Writing; override;
  public
    property SrcMap: TPas2JSSrcMap read FSrcMap write SetSrcMap;
    destructor Destroy; override;
    procedure WriteFile(Src, Filename: string);
    // Final destination filename. Usually unit, unless combining javascript in single file.
    Property DestFileName : String Read FDestFileName Write FDestFileName;
  end;

implementation

{ TPas2JSSrcMap }

procedure TPas2JSSrcMap.AddRef;
begin
  inc(fRefCount);
end;

procedure TPas2JSSrcMap.Release;
begin
  if fRefCount<0 then
    raise Exception.Create('TPas2JSSrcMap.Release');
  dec(fRefCount);
  if fRefCount<0 then
    Free;
end;

{ TPas2JSMapper }

procedure TPas2JSMapper.SetSrcMap(const AValue: TPas2JSSrcMap);
begin
  if FSrcMap=AValue then Exit;
  if FSrcMap<>nil then
    FSrcMap.Release;
  FSrcMap:=AValue;
  if FSrcMap<>nil then
    FSrcMap.AddRef;
end;

procedure TPas2JSMapper.SetCurElement(const AValue: TJSElement);
var
  C: TClass;
begin
  {$IFDEF VerboseSrcMap}
  system.write('TPas2JSMapper.SetCurElement ',CurLine,',',CurColumn);
  if AValue<>nil then
    system.writeln(' ',AValue.ClassName,' src=',ExtractFileName(AValue.Source),' ',AValue.Line,',',AValue.Column)
  else
    system.writeln(' NIL');
  {$ENDIF}
  inherited SetCurElement(AValue);
  C:=AValue.ClassType;
  if (C=TJSStatementList)
      or (C=TJSEmptyBlockStatement)
      or (C=TJSEmptyStatement) then
    exit; // do not switch position on brackets

  if (AValue<>nil) and (AValue.Source<>'') then
    begin
    if (FSrcFilename<>AValue.Source)
        or (FSrcLine<>AValue.Line)
        or (FSrcColumn<>AValue.Column) then
      begin
      FNeedMapping:=true;
      FSrcFilename:=AValue.Source;
      FSrcLine:=AValue.Line;
      FSrcColumn:=AValue.Column;
      end;
    end;
  if FGeneratedStartLine<1 then
    FGeneratedStartLine:=CurLine;
end;

procedure TPas2JSMapper.Writing;
var
  S: TJSString;
  p, l, Line, CurSrcLine, CurSrcColumn: Integer;
  CurSrcFilename: String;
begin
  inherited Writing;
  if SrcMap=nil then exit;

  if FGeneratedStartLine<1 then
    FGeneratedStartLine:=CurLine;

  if not FNeedMapping then exit;
  if FSrcFilename='' then
    exit; // built-in element -> do not add a mapping

  CurSrcFilename:=FSrcFilename;
  CurSrcLine:=FSrcLine;
  CurSrcColumn:=FSrcColumn;
  //system.writeln('TPas2JSMapper.Writing ',FSrcFilename);
  if ExtractFileExt(CurSrcFilename)='.pju' then
    begin
    // precompiled js -> map to js
    exit;
    end;

  FNeedMapping:=false;
  //system.writeln('TPas2JSMapper.Writing Generated.Line=',CurLine,',Col=',CurColumn-1,
  //  ' Orig:',ExtractFileName(FSrcFilename),',Line=',FSrcLine,',Col=',FSrcColumn-1);

  SrcMap.AddMapping(CurLine,Max(0,CurColumn-1),
    CurSrcFilename,Max(0,CurSrcLine),Max(0,CurSrcColumn-1));

  if (CurElement is TJSLiteral)
      and (TJSLiteral(CurElement).Value.CustomValue<>'') then
    begin
    // possible multi line value, e.g. asm-block
    S:=TJSLiteral(CurElement).Value.CustomValue;
    l:=length(S);
    p:=1;
    Line:=0;
    while p<=l do
      case S[p] of
      #10,#13:
        begin
        if (p<l) and (S[p+1] in [#10,#13]) and (S[p]<>S[p+1]) then
          inc(p,2)
        else
          inc(p);
        inc(Line);
        // add a mapping for each line
        //system.writeln('TPas2JSMapper.Writing Generated.Line=',CurLine+Line,',Col=',0,
        //  ' Orig:',ExtractFileName(FSrcFilename),',Line=',FSrcLine+Line,',Col=',0);
        SrcMap.AddMapping(CurLine+Line,0,
          CurSrcFilename,CurSrcLine+Line,0);
        end;
      else
        inc(p);
      end;
    end;
end;

destructor TPas2JSMapper.Destroy;
begin
  SrcMap:=nil;
  inherited Destroy;
end;

procedure TPas2JSMapper.WriteFile(Src, Filename: string);
var
  l, p, LineStart: integer;
begin
  if Src='' then exit;
  FSrcFilename:=Filename;
  FSrcLine:=1;
  FSrcColumn:=1;
  l:=length(Src);
  p:=1;
  repeat
    LineStart:=p;
    while (p<=l) do
      case Src[p] of
      #10,#13:
        begin
        if (p<l) and (Src[p+1] in [#10,#13]) and (Src[p]<>Src[p+1]) then
          inc(p,2)
        else
          inc(p);
        break;
        end;
      else
        inc(p);
      end;
    FNeedMapping:=true;
    Write(copy(Src,LineStart,p-LineStart));
    inc(FSrcLine);
  until p>l;
end;

end.

