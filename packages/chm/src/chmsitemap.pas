{ Copyright (C) <2005> <Andrew Haines> chmsitemap.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
<<<<<<< HEAD
<<<<<<< HEAD
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/fixes_2_2
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/fixes_2_2
}
{
  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.
}
<<<<<<< HEAD
<<<<<<< HEAD
unit chmsitemap;
=======
unit chmsitemap; 
>>>>>>> graemeg/fixes_2_2
=======
unit chmsitemap; 
>>>>>>> origin/fixes_2_2

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, fasthtmlparser;
<<<<<<< HEAD
<<<<<<< HEAD

type
  TChmSiteMapItems = class; // forward
  TChmSiteMap = class;

=======
=======
>>>>>>> origin/fixes_2_2
  
type
  TChmSiteMapItems = class; // forward
  TChmSiteMap = class;
  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  { TChmSiteMapItem }

  TChmSiteMapItem = class(TPersistent)
  private
    FChildren: TChmSiteMapItems;
    FComment: String;
    FImageNumber: Integer;
    FIncreaseImageIndex: Boolean;
    FKeyWord: String;
    FLocal: String;
    FOwner: TChmSiteMapItems;
    FSeeAlso: String;
    FText: String;
    FURL: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FMerge : String;
    FFrameName : String;
    FWindowName : String;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure SetChildren(const AValue: TChmSiteMapItems);
  public
    constructor Create(AOwner: TChmSiteMapItems);
    destructor Destroy; override;
  published
    property Children: TChmSiteMapItems read FChildren write SetChildren;
    property Text: String read FText write FText; // Name for TOC; KeyWord for index
    property KeyWord: String read FKeyWord write FKeyWord;
    property Local: String read FLocal write FLocal;
    property URL: String read FURL write FURL;
    property SeeAlso: String read FSeeAlso write FSeeAlso;
    property ImageNumber: Integer read FImageNumber write FImageNumber default -1;
    property IncreaseImageIndex: Boolean read FIncreaseImageIndex write FIncreaseImageIndex;
    property Comment: String read FComment write FComment;
    property Owner: TChmSiteMapItems read FOwner;
<<<<<<< HEAD
<<<<<<< HEAD

    property FrameName: String read FFrameName write FFrameName;
    property WindowName: String read FWindowName write FWindowName;
//    property Type_: Integer read FType_ write FType_; either Local or URL
    property Merge: String read FMerge write FMerge;
=======
=======
>>>>>>> origin/fixes_2_2
    //property FrameName: String read FFrameName write FFrameName;
    //property WindowName: String read FWindowName write FWindowName;
    //property Type_: Integer read FType_ write FType_; either Local or URL
    //property Merge: Boolean read FMerge write FMerge;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;

  { TChmSiteMapItems }

  TChmSiteMapItems = class(TPersistent)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    FInternalData: Dword;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    FInternalData: Dword;
>>>>>>> origin/fixes_2.4
    FList: TList;
    FOwner: TChmSiteMap;
    FParentItem: TChmSiteMapItem;
    function GetCount: Integer;
    function GetItem(AIndex: Integer): TChmSiteMapItem;
    procedure SetItem(AIndex: Integer; const AValue: TChmSiteMapItem);
  public
    constructor Create(AOwner: TChmSiteMap; AParentItem: TChmSiteMapItem);
    destructor Destroy; override;
    procedure Delete(AIndex: Integer);
    function Add(AItem: TChmSiteMapItem): Integer;
    function NewItem: TChmSiteMapItem;
    function Insert(AItem: TChmSiteMapItem; AIndex: Integer): Integer;
    procedure Clear;
    procedure Sort(Compare: TListSortCompare);
    property Item[AIndex: Integer]: TChmSiteMapItem read GetItem write SetItem;
    property Count: Integer read GetCount;
    property ParentItem: TChmSiteMapItem read FParentItem;
    property Owner: TChmSiteMap read FOwner;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    property InternalData: Dword read FInternalData write FInternalData;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
    property InternalData: Dword read FInternalData write FInternalData;
>>>>>>> origin/fixes_2.4
  end;
  

  { TChmSiteMapTree }
  TSiteMapType = (stTOC, stIndex);
  
  TSiteMapTag = (smtUnknown, smtNone, smtHTML, smtHEAD, smtBODY);
  TSiteMapTags = set of TSiteMapTag;

  TSiteMapBodyTag = (smbtUnknown, smbtNone, smbtUL, smbtLI, smbtOBJECT, smbtPARAM);
  TSiteMapBodyTags = set of TSiteMapBodyTag;
  
  TLIObjectParamType = (ptName, ptLocal, ptKeyword);

  TChmSiteMap = class
  private
    FAutoGenerated: Boolean;
    FBackgroundColor: LongInt;
    FCurrentItems: TChmSiteMapItems;
    FExWindowStyles: LongInt;
    FFont: String;
    FForegroundColor: LongInt;
    FFrameName: String;
    FImageList: String;
    FImageWidth: Integer;
    FSiteMapTags: TSiteMapTags;
    FSiteMapBodyTags: TSiteMapBodyTags;
    FHTMLParser: THTMLParser;
    FItems: TChmSiteMapItems;
    FSiteMapType: TSiteMapType;
    FUseFolderImages: Boolean;
    FWindowName: String;
    FLevel: Integer;
<<<<<<< HEAD
<<<<<<< HEAD
    FLevelForced: Boolean;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    FWindowStyles: LongInt;
    procedure SetItems(const AValue: TChmSiteMapItems);
  protected
    procedure FoundTag (ACaseInsensitiveTag, AActualTag: string);
    procedure FoundText(AText: string);
  public
    constructor Create(AType: TSiteMapType);
    destructor Destroy; override;
    procedure LoadFromFile(AFileName: String);
    procedure LoadFromStream(AStream: TStream);
<<<<<<< HEAD
<<<<<<< HEAD
    procedure SaveToFile(AFileName:String);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure SaveToStream(AStream: TStream);
    property Items: TChmSiteMapItems read FItems write SetItems;
    property SiteMapType: TSiteMapType read FSiteMapType;
    // SiteMap properties. most of these are invalid for the index
    property FrameName: String read FFrameName write FFrameName;
    property WindowName: String read FWindowName write FWindowName;
    property ImageList: String read FImageList write FImageList;
    property ImageWidth: Integer read FImageWidth write FImageWidth;
    property BackgroundColor: LongInt read FBackgroundColor write FBackgroundColor;
    property ForegroundColor: LongInt read FForegroundColor write FForegroundColor;
    property ExWindowStyles: LongInt read FExWindowStyles write FExWindowStyles;
    property WindowStyles: LongInt read FWindowStyles write FWindowStyles;
    property UseFolderImages: Boolean read FUseFolderImages write FUseFolderImages;
    property Font: String read FFont write FFont;
    property AutoGenerated: Boolean read FAutoGenerated write FAutoGenerated;
  end;

implementation
uses HTMLUtil;

{ TChmSiteMapTree }

procedure TChmSiteMap.SetItems(const AValue: TChmSiteMapItems);
begin
  if FItems=AValue then exit;
  FItems:=AValue;
end;

procedure TChmSiteMap.FoundTag(ACaseInsensitiveTag, AActualTag: string);
    function ActiveItem: TChmSiteMapItem;
    begin
      Result := FCurrentItems.Item[FCurrentItems.Count-1]
    end;
    procedure IncreaseULevel;
    begin
      if FCurrentItems = nil then FCurrentItems := Items
      else begin
        //WriteLn('NewLevel. Count = ', FCurrentItems.Count, ' Index = ',Items.Count-1);
        FCurrentItems := ActiveItem.Children;
      end;
      Inc(FLevel);
    end;
    procedure DecreaseULevel;
    begin
      if Assigned(FCurrentItems) and Assigned(FCurrentItems.ParentItem) then
        FCurrentItems := FCurrentItems.ParentItem.Owner
      else FCurrentItems := nil;
      Dec(FLevel);
    end;
    procedure NewSiteMapItem;
    begin
      FCurrentItems.Add(TChmSiteMapItem.Create(FCurrentItems));
    end;
var
  TagName,
  //TagAttribute,
  TagAttributeName,
  TagAttributeValue: String;
<<<<<<< HEAD
<<<<<<< HEAD
  isParam,IsMerged : string;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  //WriteLn('TAG:', AActualTag);
  TagName := GetTagName(ACaseInsensitiveTag);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  if not (smtHTML in FSiteMapTags) then begin
    if (TagName = 'HTML') or (TagName = '/HTML') then Include(FSiteMapTags, smtHTML);
  end
  else begin // looking for /HTML
    if TagName = '/HTML' then Exclude(FSiteMapTags, smtHTML);
  end;}

  //if (smtHTML in FSiteMapTags) then begin
=======
=======
>>>>>>> origin/fixes_2_2
  if not (smtHTML in FSiteMapTags) then begin
    if TagName = 'HTML' then Include(FSiteMapTags, smtHTML);
=======
{  if not (smtHTML in FSiteMapTags) then begin
    if (TagName = 'HTML') or (TagName = '/HTML') then Include(FSiteMapTags, smtHTML);
>>>>>>> origin/fixes_2.4
  end
  else begin // looking for /HTML
    if TagName = '/HTML' then Exclude(FSiteMapTags, smtHTML);
  end;}
  
<<<<<<< HEAD
  if (smtHTML in FSiteMapTags) then begin
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  //if (smtHTML in FSiteMapTags) then begin
>>>>>>> origin/fixes_2.4
     if not (smtBODY in FSiteMapTags) then begin
       if TagName = 'BODY' then Include(FSiteMapTags, smtBODY);
     end
     else begin
       if TagName = '/BODY' then Exclude(FSiteMapTags, smtBODY);
     end;
<<<<<<< HEAD
<<<<<<< HEAD

=======
     
>>>>>>> graemeg/fixes_2_2
=======
     
>>>>>>> origin/fixes_2_2
     if (smtBODY in FSiteMapTags) then begin
       //WriteLn('GOT TAG: ', AActualTag);
       if TagName = 'UL' then begin
         //WriteLN('Inc Level');
         IncreaseULevel;
       end
       else if TagName = '/UL' then begin
         //WriteLN('Dec Level');
         DecreaseULevel;
       end
<<<<<<< HEAD
<<<<<<< HEAD
       else if (TagName = 'LI') and (FLevel = 0) then
         FLevelForced := True
       else if TagName = 'OBJECT' then begin
         Include(FSiteMapBodyTags, smbtOBJECT);
         if FLevelForced then
           IncreaseULevel;
=======
       else if TagName = 'OBJECT' then begin
         Include(FSiteMapBodyTags, smbtOBJECT);
>>>>>>> graemeg/fixes_2_2
=======
       else if TagName = 'OBJECT' then begin
         Include(FSiteMapBodyTags, smbtOBJECT);
>>>>>>> origin/fixes_2_2
         If FLevel > 0 then // if it is zero it is the site properties
           NewSiteMapItem;
       end
       else if TagName = '/OBJECT' then begin
         Exclude(FSiteMapBodyTags, smbtOBJECT);
<<<<<<< HEAD
<<<<<<< HEAD
         if FLevelForced then
         begin
           DecreaseULevel;
           FLevelForced := False;
         end;
       end
       else begin // we are the properties of the object tag
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         if (smbtOBJECT in FSiteMapBodyTags) then
           begin
            if (FLevel > 0 ) then 
             begin

              if LowerCase(GetTagName(AActualTag)) = 'param' then begin

                TagAttributeName := GetVal(AActualTag, 'name');
                TagAttributeValue := GetVal(AActualTag, 'value');
                //writeln('name,value',tagattributename, ' ',tagattributevalue);
                if TagAttributeName <> '' then begin
                  if CompareText(TagAttributeName, 'keyword') = 0 then begin
                    ActiveItem.Text := TagAttributeValue;
                  end
                  else if CompareText(TagAttributeName, 'name') = 0 then begin
                    if ActiveItem.Text = '' then ActiveItem.Text := TagAttributeValue;
                  end
                  else if CompareText(TagAttributeName, 'local') = 0 then begin
                    ActiveItem.Local := TagAttributeValue;
                  end
                  else if CompareText(TagAttributeName, 'URL') = 0 then begin
                    ActiveItem.URL := TagAttributeValue;
                  end
                  else if CompareText(TagAttributeName, 'ImageNumber') = 0 then begin
                    ActiveItem.ImageNumber := StrToInt(TagAttributeValue);
                  end
                  else if CompareText(TagAttributeName, 'New') = 0 then begin
                    ActiveItem.IncreaseImageIndex := (LowerCase(TagAttributeValue) = 'yes');
                  end
                  else if CompareText(TagAttributeName, 'Comment') = 0 then begin
                    ActiveItem.Comment := TagAttributeValue
                  end
                  else if CompareText(TagAttributeName, 'Merge') = 0 then begin
                    ActiveItem.Merge:= TagAttributeValue
                  end;
                  //else if CompareText(TagAttributeName, '') = 0 then begin
                  //end;
                end;
              end;
            end
           else
             begin // object and level is zero?
               if LowerCase(GetTagName(AActualTag)) = 'param' then begin
                 begin
                   TagAttributeName := uppercase(GetVal(AActualTag, 'name'));
                   TagAttributeValue := GetVal(AActualTag, 'value');
                   if TagAttributeName = 'FRAMENAME' then
                     framename:=TagAttributeValue
                   else
                     if TagAttributeName = 'WINDOWNAME' then
                       WINDOWname:=TagAttributeValue
                   else
                     if TagAttributeName = 'WINDOW STYLES' then
                       WindowStyles:=StrToIntDef(TagAttributeValue,0)
                   else
                     if TagAttributeName = 'EXWINDOW STYLES' then
                       ExWindowStyles:=StrToIntDef(TagAttributeValue,0)
                   else
                     if TagAttributeName = 'FONT' then
                       FONT:=TagAttributeValue
                   else
                     if TagAttributeName = 'IMAGELIST' then
                      IMAGELIST:=TagAttributeValue
                    else
                     if TagAttributeName = 'IMAGETYPE' then
                      UseFolderImages:=uppercase(TagAttributeValue)='FOLDER';
                  // writeln('0:',flevel,' ' ,aactualtag,' ',tagname,' ' ,tagattributename, ' ' ,tagattributevalue);
                 end;
                 end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         if (FLevel > 0 ) and (smbtOBJECT in FSiteMapBodyTags) then begin

=======
       end
       else begin // we are the properties of the object tag
         if (smbtOBJECT in FSiteMapBodyTags) then begin
>>>>>>> graemeg/fixes_2_2
=======
       end
       else begin // we are the properties of the object tag
         if (smbtOBJECT in FSiteMapBodyTags) then begin
>>>>>>> origin/fixes_2_2
           if LowerCase(GetTagName(AActualTag)) = 'param' then begin

             TagAttributeName := GetVal(AActualTag, 'name');
             TagAttributeValue := GetVal(AActualTag, 'value');

             if TagAttributeName <> '' then begin
               if CompareText(TagAttributeName, 'keyword') = 0 then begin
                 ActiveItem.Text := TagAttributeValue;
               end
               else if CompareText(TagAttributeName, 'name') = 0 then begin
                 if ActiveItem.Text = '' then ActiveItem.Text := TagAttributeValue;
               end
               else if CompareText(TagAttributeName, 'local') = 0 then begin
                 ActiveItem.Local := TagAttributeValue;
               end
               else if CompareText(TagAttributeName, 'URL') = 0 then begin
                 ActiveItem.URL := TagAttributeValue;
               end
               else if CompareText(TagAttributeName, 'ImageNumber') = 0 then begin
                 ActiveItem.ImageNumber := StrToInt(TagAttributeValue);
               end
               else if CompareText(TagAttributeName, 'New') = 0 then begin
                 ActiveItem.IncreaseImageIndex := (LowerCase(TagAttributeValue) = 'yes');
               end
               else if CompareText(TagAttributeName, 'Comment') = 0 then begin
                 ActiveItem.Comment := TagAttributeValue;
               end;
               //else if CompareText(TagAttributeName, '') = 0 then begin
               //end;
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
             end;
          end;
       end;
     end;
  //end
=======
=======
>>>>>>> origin/fixes_2_2
             end;
           end;
         end;
       end;
     end;
<<<<<<< HEAD
  end
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  //end
>>>>>>> origin/fixes_2.4
end;

procedure TChmSiteMap.FoundText(AText: string);
begin
  //WriteLn('TEXT:', AText);
end;

constructor TChmSiteMap.Create(AType: TSiteMapType);
begin
  Inherited Create;
  FSiteMapType := AType;
  FSiteMapTags := [smtNone];
  FSiteMapBodyTags := [smbtNone];
  FHTMLParser:=nil;
  FItems := TChmSiteMapItems.Create(Self, nil);  ;
end;

destructor TChmSiteMap.Destroy;
begin
  if Assigned(FHTMLParser) then FHTMLParser.Free;
  FItems.Free;
  Inherited Destroy;
end;

procedure TChmSiteMap.LoadFromFile(AFileName: String);
var
  Buffer: String;
  TmpStream: TMemoryStream;
begin
  if Assigned(FHTMLParser) then FHTMLParser.Free;
  TmpStream := TMemoryStream.Create;
<<<<<<< HEAD
<<<<<<< HEAD
  try
    TmpStream.LoadFromFile(AFileName);
    SetLength(Buffer, TmpStream.Size);
    TmpStream.Position := 0;
    TmpStream.Read(Buffer[1], TmpStream.Size);
  finally
    TmpStream.Free;
  end;
  FHTMLParser := THTMLParser.Create(Buffer);
  try
    FHTMLParser.OnFoundTag := @FoundTag;
    FHTMLParser.OnFoundText := @FoundText;
    FHTMLParser.Exec;
  finally
    FreeAndNil(FHTMLParser);
  end;
=======
=======
>>>>>>> origin/fixes_2_2
  TmpStream.LoadFromFile(AFileName);
  SetLength(Buffer, TmpStream.Size);
  TmpStream.Position := 0;
  TmpStream.Read(Buffer[1], TmpStream.Size);
  FHTMLParser := THTMLParser.Create(Buffer);
  FHTMLParser.OnFoundTag := @FoundTag;
  FHTMLParser.OnFoundText := @FoundText;
  FHTMLParser.Exec;
  FreeAndNil(FHTMLParser);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TChmSiteMap.LoadFromStream(AStream: TStream);
var
  Buffer: String;
begin
  if Assigned(FHTMLParser) then FHTMLParser.Free;
  SetLength(Buffer, AStream.Size-AStream.Position);
  if AStream.Read(Buffer[1], AStream.Size-AStream.Position) > 0 then begin;
    FHTMLParser := THTMLParser.Create(Buffer);
    FHTMLParser.OnFoundTag := @FoundTag;
    FHTMLParser.OnFoundText := @FoundText;
    FHTMLParser.Exec;
    FreeAndNil(FHTMLParser);
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TChmSiteMap.SaveToFile(AFileName:String);
var
  fs : TFileStream;
begin
  fs:=TFileStream.Create(AFileName,fmcreate);
  try
    SaveToStream(fs);
  finally
    fs.free;
    end;
end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
                    
>>>>>>> graemeg/cpstrnew
=======
                    
>>>>>>> graemeg/cpstrnew
=======
                    
>>>>>>> graemeg/cpstrnew
=======
                    
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TChmSiteMap.SaveToStream(AStream: TStream);
var
  Indent: Integer;
  procedure WriteString(AString: String);
  var
    I: Integer;
  begin
     for I := 0 to Indent-1 do AStream.WriteByte(Byte(' '));
     AStream.Write(AString[1], Length(AString));
     AStream.WriteByte(10);
  end;
  procedure WriteParam(AName: String; AValue: String);
  begin
    WriteString('<param name="'+AName+'" value="'+AValue+'">');
  end;
  procedure WriteEntries(AItems: TChmSiteMapItems);
  var
    I : Integer;
    Item: TChmSiteMapItem;
  begin
    for I := 0 to AItems.Count-1 do begin
      Item := AItems.Item[I];
      WriteString('<LI> <OBJECT type="text/sitemap">');
      Inc(Indent, 8);

      if (SiteMapType = stIndex) and (Item.Children.Count > 0) then
         WriteParam('Keyword', Item.Text);
      //if Item.KeyWord <> '' then WriteParam('Keyword', Item.KeyWord);
      if Item.Text <> '' then WriteParam('Name', Item.Text);
      if (Item.Local <> '') or (SiteMapType = stIndex) then WriteParam('Local', StringReplace(Item.Local, '\', '/', [rfReplaceAll]));
      if Item.URL <> '' then WriteParam('URL', StringReplace(Item.URL, '\', '/', [rfReplaceAll]));
      if (SiteMapType = stIndex) and (Item.SeeAlso <> '') then WriteParam('See Also', Item.SeeAlso);
      //if Item.FrameName <> '' then WriteParam('FrameName', Item.FrameName);
      //if Item.WindowName <> '' then WriteParam('WindowName', Item.WindowName);
      if Item.Comment <> '' then WriteParam('Comment', Item.Comment);
      if (SiteMapType = stTOC) and (Item.IncreaseImageIndex) then WriteParam('New', 'yes'); // is this a correct value?
      if (SiteMapType = stTOC) and (Item.ImageNumber <> -1) then WriteParam('ImageNumber', IntToStr(Item.ImageNumber));

      Dec(Indent, 3);
      WriteString('</OBJECT>');
      Dec(Indent, 5);

      // Now Sub Entries
      if Item.Children.Count > 0 then begin
        WriteString('<UL>');
        Inc(Indent, 8);
        WriteEntries(Item.Children);
        Dec(Indent, 8);
        WriteString('</UL>');
      end;
    end;
  end;
begin
  Indent := 0;
  WriteString('<!DOCTYPE HTML PUBLIC "-//IETF//DTD HTML//EN">');
  WriteString('<HTML>');
  WriteString('<HEAD>');
  WriteString('<meta name="GENERATOR" content="Microsoft&reg; HTML Help Workshop 4.1">');  // Should we change this?
  WriteString('<!-- Sitemap 1.0 -->');
  WriteString('</HEAD><BODY>');
<<<<<<< HEAD
<<<<<<< HEAD

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  // Site Properties
  WriteString('<OBJECT type="text/site properties">');
  Inc(Indent, 8);
    if SiteMapType = stTOC then begin
      if FrameName <> '' then WriteParam('FrameName', FrameName);
      if WindowName <> '' then WriteParam('WindowName', WindowName);
      if ImageList <> '' then WriteParam('ImageList', ImageList);
      if ImageWidth > 0 then WriteParam('Image Width', IntToStr(ImageWidth));
      if BackgroundColor <> 0 then WriteParam('Background', hexStr(BackgroundColor, 4));
      if ForegroundColor <> 0 then WriteParam('Foreground', hexStr(ForegroundColor, 4));
      if ExWindowStyles <> 0 then WriteParam('ExWindow Styles', hexStr(ExWindowStyles, 4));
      if WindowStyles <> 0 then WriteParam('Window Styles', hexStr(WindowStyles, 4));
      if UseFolderImages then WriteParam('ImageType', 'Folder');
    end;
    // both TOC and Index have font
    if Font <> '' then
      WriteParam('Font', Font);
    Dec(Indent, 8);
  WriteString('</OBJECT>');
  
  // And now the items
  if Items.Count > 0 then begin
    WriteString('<UL>');
    Inc(Indent, 8);
    // WriteEntries
    WriteEntries(Items);
    Dec(Indent, 8);
    WriteString('</UL>');
  end;
  
  WriteString('</BODY></HTML>');
  
  AStream.Size := AStream.Position;
end;

{ TChmSiteMapItem }

procedure TChmSiteMapItem.SetChildren(const AValue: TChmSiteMapItems);
begin
  if FChildren = AValue then exit;
  FChildren := AValue;
end;

constructor TChmSiteMapItem.Create(AOwner: TChmSiteMapItems);
begin
  Inherited Create;
  FOwner := AOwner;
  FChildren := TChmSiteMapItems.Create(Owner.Owner, Self);
end;

destructor TChmSiteMapItem.Destroy;
begin
  FChildren.Free;
  Inherited Destroy;
end;

{ TChmSiteMapItems }

function TChmSiteMapItems.GetItem(AIndex: Integer): TChmSiteMapItem;
begin
  Result := TChmSiteMapItem(FList.Items[AIndex]);
end;

function TChmSiteMapItems.GetCount: Integer;
begin
  Result := FList.Count;
end;

procedure TChmSiteMapItems.SetItem(AIndex: Integer; const AValue: TChmSiteMapItem);
begin
  FList.Items[AIndex] := AValue;
end;

constructor TChmSiteMapItems.Create(AOwner: TChmSiteMap; AParentItem: TChmSiteMapItem);
begin
  FList := TList.Create;
  FParentItem := AParentItem;
  FOwner := AOwner;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  FInternalData := maxLongint;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  FInternalData := maxLongint;
>>>>>>> origin/fixes_2.4
end;

destructor TChmSiteMapItems.Destroy;
begin
  Clear;
  FList.Free;
  inherited Destroy;
end;

procedure TChmSiteMapItems.Delete(AIndex: Integer);
begin
  Item[AIndex].Free;
  FList.Delete(AIndex);
end;

function TChmSiteMapItems.Add(AItem: TChmSiteMapItem): Integer;
begin
  Result := FList.Add(AItem);
end;

function TChmSiteMapItems.NewItem: TChmSiteMapItem;
begin
  Result := TChmSiteMapItem.Create(Self);
  Add(Result);
end;

function TChmSiteMapItems.Insert(AItem: TChmSiteMapItem; AIndex: Integer): Integer;
begin
  Result := AIndex;
  FList.Insert(AIndex, AItem);
end;

procedure TChmSiteMapItems.Clear;
var
  I: LongInt;
begin
  for I := Count-1 downto 0 do Delete(I);
end;

procedure TChmSiteMapItems.Sort(Compare: TListSortCompare);
begin
  FList.Sort(Compare);
end;

end.

