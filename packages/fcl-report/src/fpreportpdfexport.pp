{
    This file is part of the Free Component Library.
    Copyright (c) WISA b.v.b.a

    FPReport PDF export filter.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpreportpdfexport;

{$mode objfpc}{$H+}

interface

uses
  Classes,
  SysUtils,
  fpImage,
  fpreport,
  fpPDF;

{$IF FPC_FULLVERSION>=30101}
{$DEFINE PDF_HASISSTANDARDPDFFONT}
{$DEFINE PDF_HASEXTERNALLINK}
{$ENDIF}

type

  TFPReportExportPDF = class(TFPReportExporter)
  private
    FCurrentPage: TPDFPage;
    FOptions: TPDFOptions;
    FPageLayout: TPDFPageLayout;
    FFileName: string;
    FDocument: TPDFDocument;
    FAutoSave: boolean;
  protected
    procedure   RenderElement(pg: TPDFPage; ABand: TFPReportCustomBand; el: TFPReportElement); virtual;
    Procedure   RenderImage(aRect : TFPReportRect; var AImage: TFPCustomImage) ; override;
    procedure   DoExecute(const ARTObjects: TFPList); override;
    procedure   SetupPDFDocument; virtual;
    procedure   RenderFrame(const APage: TPDFPage; const ABand: TFPReportCustomBand; const AFrame: TFPReportFrame; const APos: TPDFCoord; const AWidth, AHeight: TFPReportUnits); virtual;
    procedure   RenderMemo(const APage: TPDFPage; const ABand: TFPReportCustomBand; const AMemo: TFPReportCustomMemo); virtual;
    procedure   RenderShape(const APage: TPDFPage; const ABand: TFPReportCustomBand; const AShape: TFPReportCustomShape); virtual;
    procedure   RenderImage(const APage: TPDFPage; const ABand: TFPReportCustomBand; const AImage: TFPReportCustomImage); virtual;
    procedure   RenderCheckbox(const APage: TPDFPage; const ABand: TFPReportCustomBand; const ACheckbox: TFPReportCustomCheckbox); virtual;
    procedure   RenderShape(const APage: TPDFPage; const AOrigin: TPDFCoord; const AShape: TFPReportCustomShape); virtual;
    procedure   RenderShapeCircle(const APage: TPDFPage; const lpt1: TPDFCoord; const ALayout: TFPReportLayout);
    procedure   RenderShapeEllipse(const APage: TPDFPage; const lpt1: TPDFCoord; const ALayout: TFPReportLayout);
    procedure   RenderShapeLine(const APage: TPDFPage; lpt1: TPDFCoord;  const AOrientation: TFPReportOrientation; const ALayout: TFPReportLayout);
    procedure   RenderShapeRect(const APage: TPDFPage; const lpt1: TPDFCoord; const ALayout: TFPReportLayout);
    procedure   RenderShapeTriangle(const APage: TPDFPage; Alpt: TPDFCoord; const AOrientation: TFPReportOrientation; const ALayout: TFPReportLayout);
    procedure   RenderShapeRoundedRect(const APage: TPDFPage; const lpt1: TPDFCoord; const ARadius: TFPReportUnits; const ALayout: TFPReportLayout);
  public
    constructor Create(AOwner: TComponent); override;
    destructor  Destroy; override;
    Class Function Name : String; override;
    Class Function Description : String; override;
    Class Function DefaultExtension: String; override;
    Procedure   SetFileName(const aFileName: String); override;
    function    FindFontIndex(const ADoc: TPDFDocument; const AFontName: string): integer;
    procedure   SaveToFile;
    property    Document: TPDFDocument read FDocument;
    Property    CurrentPage: TPDFPage Read FCurrentPage;
  published
    property    AutoSave: boolean read FAutoSave write FAutoSave default True;
    property    FileName: string read FFileName write FFileName;
    Property    Options : TPDFOptions Read FOptions Write FOptions;
    property    PageLayout : TPDFPageLayout read FPageLayout write FPageLayout default lSingle;
  end;

implementation

uses
  FPCanvas,
  fpTTF,
  fpparsettf;


{ TFPReportExportPDF }

function TFPReportExportPDF.FindFontIndex(const ADoc: TPDFDocument; const AFontName: string): integer;

    function isStandardPdfFont: Boolean;

    begin
{$IFDEF PDF_HASISSTANDARDPDFFONT}
     Result:=aDoc.IsStandardPDFFont(aFontName)
{$ELSE}
      Result:=(AFontName='Courier') or (AFontName='Courier-Bold') or (AFontName='Courier-Oblique') or (AFontName='Courier-BoldOblique')
              or (AFontName='Helvetica') or (AFontName='Helvetica-Bold') or (AFontName='Helvetica-Oblique') or (AFontName='Helvetica-BoldOblique')
              or (AFontName='Times-Roman') or (AFontName='Times-Bold') or (AFontName='Times-Italic') or (AFontName='Times-BoldItalic')
              or (AFontName='Symbol')
              or (AFontName='ZapfDingbats');
{$ENDIF}
    end;

var
  i: integer;
  fnt: TFPFontCacheItem;
begin
  Result := -1;
  for i := 0 to Document.Fonts.Count-1 do
  begin
    if Document.Fonts.FontDefs[i].Name = AFontName then
    begin
      Result := i;
      break;
    end;
  end;  { for i ... }

  if Result = -1 then
  begin
    if IsStandardPDFFont then
    begin
      Result := Document.AddFont(AFontName);
    end
    else
    begin
      fnt := gTTFontCache.Find(AFontName); // we are doing a PostScript Name lookup (it contains Bold, Italic info)
      if Assigned(fnt) then
        Result := Document.AddFont(fnt.FileName, AFontName)
      else 
        begin
        fnt:=gTTFontCache.FindFont(AFontName);
        if fnt=Nil then
          raise Exception.CreateFmt('fpreport: Could not find the font <%s> in the font cache.', [AFontName]);
        end;
    end;
  end;
end;

procedure TFPReportExportPDF.SetupPDFDocument;
begin
  if Assigned(FDocument) then
    FDocument.Free;
  FDocument := TPDFDocument.Create(Nil);
  FDocument.Infos.Title := TFPReport(Report).Title;
  FDocument.Infos.Author := TFPReport(Report).Author;
  FDocument.Infos.ApplicationName := ApplicationName;
  FDocument.Infos.CreationDate := Now;
  FDocument.Options:=Self.Options;
  FDocument.PageLayout:=Self.PageLayout;
  FDocument.StartDocument;
  { we always need at least one section }
  FDocument.Sections.AddSection;
end;

procedure TFPReportExportPDF.SaveToFile;
var
  F: TFileStream;
begin
  if not Assigned(FDocument) then
    Exit;
  F := TFileStream.Create(FFileName, fmCreate);
  try
    FDocument.SaveToStream(F);
  finally
    F.Free;
  end;
end;

procedure TFPReportExportPDF.RenderFrame(const APage: TPDFPage; const ABand: TFPReportCustomBand;
  const AFrame: TFPReportFrame; const APos: TPDFCoord; const AWidth, AHeight: TFPReportUnits);
var
  bStroke, bFill: boolean;
begin
  bStroke := AFrame.Color <> clNone;
  bFill := AFrame.BackgroundColor <> clNone;

  // We only support TPDFPenStyle types. (ppsSolid,ppsDash,ppsDot,ppsDashDot,ppsDashDotDot)
  case AFrame.Pen of
    psSolid:      APage.SetPenStyle(ppsSolid);
    psDash:       APage.SetPenStyle(ppsDash);
    psDot:        APage.SetPenStyle(ppsDot);
    psDashDot:    APage.SetPenStyle(ppsDashDot);
    psDashDotDot: APage.SetPenStyle(ppsDashDotDot);
    // These FPCanvas pen styles are unsupported
//    psInsideFrame: ;
//    psPattern: ;
//    psClear: ;
    else
      // give a sane fallback for now
      APage.SetPenStyle(ppsSolid);
  end;

  if (AFrame.Shape = fsRectangle) and (bStroke or bFill) then
  begin
    APage.SetColor(AFrame.BackgroundColor, False); // fill color
    APage.SetColor(AFrame.Color, True); // stroke color
    APage.DrawRect(APos.X, APos.Y, AWidth, AHeight, AFrame.Width, bFill, bStroke);
  end;

  if AFrame.Shape = fsNone then
  begin
    if AFrame.Lines <> [] then
    begin
      APage.SetColor(AFrame.Color, True);
      APage.SetColor(AFrame.Color, False);
    end;

    { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left, so adjust them }
    if flTop in AFrame.Lines then
      APage.DrawLine(APos.X, APos.Y-AHeight, APos.X+AWidth, APos.Y-AHeight, AFrame.Width);

    if flBottom in AFrame.Lines then
      APage.DrawLine(APos.X, APos.Y, APos.X+AWidth, APos.Y, AFrame.Width);

    if flLeft in AFrame.Lines then
        APage.DrawLine(APos.X, APos.Y, APos.X, APos.Y-AHeight, AFrame.Width);

    if flRight in AFrame.Lines then
      APage.DrawLine(APos.X+AWidth, APos.Y, APos.X+AWidth, APos.Y-AHeight, AFrame.Width);
  end;  { Frame.Shape = fsNone }
end;

procedure TFPReportExportPDF.RenderMemo(const APage: TPDFPage; const ABand: TFPReportCustomBand;
  const AMemo: TFPReportCustomMemo);
var
  lPt1: TPDFCoord;  // original Report point
  lFontIdx: integer;
  lMemo: TFPReportMemo;
  i: integer;
  lYPos: TPDFFloat;
  txtblk: TFPTextBlock;
begin
  lMemo := TFPReportMemo(AMemo);

  { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
  lPt1.X := ABand.RTLayout.Left + AMemo.RTLayout.Left;
  lPt1.Y := ABand.RTLayout.Top + AMemo.RTLayout.Top + AMemo.RTLayout.Height;

  { Frame must be drawn before the text as it could have a fill color. }
  RenderFrame(APage, ABand, AMemo.Frame, lPt1, AMemo.RTLayout.Width, AMemo.RTLayout.Height);

  { Store the Top-Left coordinate of the Memo. We will be reusing this info. }
  lPt1.X := ABand.RTLayout.Left + AMemo.RTLayout.Left;
  lPt1.Y := ABand.RTLayout.Top + AMemo.RTLayout.Top;

  { render the TextBlocks as-is. }
  for i := 0 to lMemo.TextBlockList.Count-1 do
  begin
    txtblk := lMemo.TextBlockList[i];
    lFontIdx := FindFontIndex(Document, txtblk.FontName);
    APage.SetFont(lFontIdx, lMemo.Font.Size);
    { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
    lYPos := lPt1.Y + txtblk.Pos.Top + txtblk.Height;

    if txtblk.BGColor <> clNone then
    begin
      { draw highlighting background rectangle }
      APage.SetColor(txtblk.BGColor, false);
      APage.DrawRect(lPt1.X + txtblk.Pos.Left, lYPos+txtblk.Descender, txtblk.Width, txtblk.Height+(txtblk.Descender*2), 1.0, True, False);
    end;

    { Text color is always a fill color, hence the False parameter. }
    APage.SetColor(txtblk.FGColor, false);
    APage.WriteText(lPt1.X + txtblk.Pos.Left, lYPos, txtblk.Text);

    // process hyperlink if available
    if txtblk is TFPHTTPTextBlock then
    begin
{$IFDEF PDF_HASEXTERNALLINK}
      APage.AddExternalLink(lPt1.X + txtblk.Pos.Left, lYPos, txtblk.Width, txtblk.Height, TFPHTTPTextBlock(txtblk).URL);
{$ENDIF}
    end;
  end;
end;

procedure TFPReportExportPDF.RenderShape(const APage: TPDFPage; const ABand: TFPReportCustomBand;
    const AShape: TFPReportCustomShape);
var
  lPt1: TPDFCoord;  // original Report point
begin
  APage.SetColor(clblack, True);
  APage.SetColor(clblack, False);

  { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
  lPt1.X := ABand.RTLayout.Left + AShape.RTLayout.Left;
  lPt1.Y := ABand.RTLayout.Top + AShape.RTLayout.Top + AShape.RTLayout.Height;

  { Frame must be drawn before the shape as it could have a fill color. }
  RenderFrame(APage, ABand, AShape.Frame, lPt1, AShape.RTLayout.Width, AShape.RTLayout.Height);
  { Only render shape when color is set and color is different to frame background color. }
  if (TFPReportShape(AShape).Color <> clNone) and
  (TFPReportShape(AShape).Color <> AShape.Frame.BackgroundColor) then
    RenderShape(APage, lPt1, AShape);
end;

type
  { for access to Protected methods }
  TReportImageFriend = class(TFPReportCustomImage);

procedure TFPReportExportPDF.RenderImage(const APage: TPDFPage; const ABand: TFPReportCustomBand;
  const AImage: TFPReportCustomImage);
var
  lPt: TPDFCoord;
  img: TReportImageFriend;
  idx, i: integer;
  pdfimg: TPDFImageItem;
begin
  img := TReportImageFriend(AImage);  { for access to Protected methods }
  lPt.X := ABand.RTLayout.Left + AImage.RTLayout.Left;
  { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
  lPt.Y := ABand.RTLayout.Top + AImage.RTLayout.Top + AImage.RTLayout.Height;

  { Frame must be drawn before the Image as it could have a fill color. }
  RenderFrame(APage, ABand, AImage.Frame, lPt, AImage.RTLayout.Width, AImage.RTLayout.Height);

  if not Assigned(img.Image) then
    Exit; { nothing further to do }

  idx := -1;
  for i := 0 to Document.Images.Count-1 do
  begin
    if Document.Images.Images[i].Equals(img.Image) then
    begin
      idx := i;
      break;
    end;
  end;

  if idx = -1 then
  begin
    pdfimg := Document.Images.AddImageItem;
    pdfimg.Image := img.Image;
    idx := Document.Images.Count-1;
  end;

  if img.Stretched then
  begin
    case APage.UnitOfMeasure of
      uomMillimeters:
        begin
          APage.DrawImage(lPt, AImage.RTLayout.Width, AImage.RTLayout.Height, idx);
        end;
      uomCentimeters:
        begin
          APage.DrawImage(lPt, AImage.RTLayout.Width, AImage.RTLayout.Height, idx);
        end;
      uomInches:
        begin
          APage.DrawImage(lPt, AImage.RTLayout.Width, AImage.RTLayout.Height, idx);
        end;
      uomPixels:
        begin
          APage.DrawImage(lPt, Integer(round(AImage.RTLayout.Width)), Integer(round(AImage.RTLayout.Height)), idx);
        end;
    end;  { case UnitOfMeasure }
  end
  else
    APage.DrawImage(lPt, img.Image.Width, img.Image.Height, idx);
end;

procedure TFPReportExportPDF.RenderCheckbox(const APage: TPDFPage; const ABand: TFPReportCustomBand;
  const ACheckbox: TFPReportCustomCheckbox);
var
  lPt: TPDFCoord;
  idx: integer;
  pdfimg: TPDFImageItem;
  lImage: TFPCustomImage;
  i: integer;
begin
  lPt.X := ABand.RTLayout.Left + ACheckbox.RTLayout.Left;
  { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
  lPt.Y := ABand.RTLayout.Top + ACheckbox.RTLayout.Top + ACheckbox.RTLayout.Height;

//  { Frame must be drawn before the Image as it could have a fill color. }
//  RenderFrame(Document, APage, ABand, AImage.Frame, lPt, AImage.RTLayout.Width, AImage.RTLayout.Height);

  lImage:=ACheckBox.GetRTImage;
  idx := -1;
  for i := 0 to Document.Images.Count-1 do
  begin
    if Document.Images.Images[i].Equals(lImage) then
    begin
      idx := i;
      break;
    end;
  end;

  if idx = -1 then
  begin
    pdfimg := Document.Images.AddImageItem;
    pdfimg.Image := lImage;
    idx := Document.Images.Count-1;
  end;

  case APage.UnitOfMeasure of
    uomMillimeters:
      begin
        APage.DrawImage(lPt, ACheckBox.RTLayout.Width, ACheckBox.RTLayout.Height, idx);
      end;
    uomCentimeters:
      begin
        APage.DrawImage(lPt, ACheckBox.RTLayout.Width, ACheckBox.RTLayout.Height, idx);
      end;
    uomInches:
      begin
        APage.DrawImage(lPt, ACheckBox.RTLayout.Width, ACheckBox.RTLayout.Height, idx);
      end;
    uomPixels:
      begin
        APage.DrawImage(lPt, Integer(round(ACheckBox.RTLayout.Width)), Integer(round(ACheckBox.RTLayout.Height)), idx);
      end;
  end;  { case UnitOfMeasure }
end;

procedure TFPReportExportPDF.RenderShape(const APage: TPDFPage; const AOrigin: TPDFCoord;
  const AShape: TFPReportCustomShape);
begin
  APage.SetColor(TFPReportShape(AShape).Color, True);
  APage.SetColor(TFPReportShape(AShape).Color, False);

  case TFPReportShape(AShape).ShapeType of
    stEllipse:      RenderShapeEllipse(APage, AOrigin, AShape.RTLayout);
    stCircle:       RenderShapeCircle(APage, AOrigin, AShape.RTLayout);
    stLine:         RenderShapeLine(APage, AOrigin, TFPReportShape(AShape).Orientation, AShape.RTLayout);
    stSquare:       RenderShapeRect(APage, AOrigin, AShape.RTLayout);
    stTriangle:     RenderShapeTriangle(APage, AOrigin, TFPReportShape(AShape).Orientation, AShape.RTLayout);
    stRoundedRect:  RenderShapeRoundedRect(APage, AOrigin, TFPReportShape(AShape).CornerRadius, AShape.RTLayout);
  end;
end;

procedure TFPReportExportPDF.RenderShapeCircle(const APage: TPDFPage; const lpt1: TPDFCoord;
    const ALayout: TFPReportLayout);
var
  lPt2: TPDFCoord;
  ldx, ldy, lw: TFPReportUnits;
begin
  if ALayout.Width = ALayout.Height then
  begin
    ldx := 0;
    ldy := 0;
    lw := ALayout.Width;
  end
  else if ALayout.Width > ALayout.Height then
  begin
    ldx := (ALayout.Width - ALayout.Height) / 2;
    ldy := 0;
    lw := ALayout.Height;
  end
  else if ALayout.Width < ALayout.Height then
  begin
    ldx := 0;
    ldy := (ALayout.Height - ALayout.Width) / 2;
    lw := ALayout.Width;
  end;
  { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
  lPt2.X := lPt1.X + ldx;
  lPt2.Y := lPt1.Y - ldy;
  APage.DrawEllipse(lPt2, lw, lw, 1, False, True);
end;

procedure TFPReportExportPDF.RenderShapeEllipse(const APage: TPDFPage; const lpt1: TPDFCoord;
  const ALayout: TFPReportLayout);
begin
  APage.DrawEllipse(lPt1, ALayout.Width, ALayout.Height, 1, False, True);
end;

procedure TFPReportExportPDF.RenderShapeLine(const APage: TPDFPage; lpt1: TPDFCoord;
    const AOrientation: TFPReportOrientation; const ALayout: TFPReportLayout);
var
  lPt2: TPDFCoord;
begin
  case AOrientation of
  orNorth, orSouth:
   begin                                           //   | (1)
     lPt1.X := lPt1.X + (ALayout.Width / 2);       //   |
     lPt2.X := lPt1.X ;                            //   |
     lPt2.Y := lPt1.Y;                             //   | (2)
     lPt1.Y := lPt1.Y - ALayout.Height;
   end;
  orNorthEast, orSouthWest:
   begin                                           //    / (1)
     lPt2.X := lPt1.X;                             //   /
     lPt1.X := lPt1.X + ALayout.Width;             //  /
     lPt2.Y := lPt1.Y;                             // / (2)
     lPt1.Y := lPt1.Y - ALayout.Height;
   end;
  orEast, orWest:
   begin                                           // (1)    (2)
     lPt2.X := lPt1.X + ALayout.Width;             // ----------
     lPt1.Y := lPt1.Y - (ALayout.Height / 2);      //
     lPt2.Y := lPt1.Y;                             //
   end;
  orSouthEast, orNorthWest:
   begin
     lPt1.Y := lPt1.Y - ALayout.Height;            // \ (1)
     lPt2.X := lPt1.X + ALayout.Width;             //  \
     lPt2.Y := lPt1.Y + ALayout.Height;            //   \
   end;                                            //    \ (2)
  end;
  APage.DrawLine(lPt1, lPt2, 1);
end;

procedure TFPReportExportPDF.RenderShapeRect(const APage: TPDFPage; const lpt1: TPDFCoord;
    const ALayout: TFPReportLayout);
var
  ldx, ldy, lw: TFPReportUnits;
  P: TPDFCoord;
begin
  if ALayout.Width = ALayout.Height then
  begin
    ldx := 0;
    ldy := 0;
    lw := ALayout.Width;
  end
  else if ALayout.Width > ALayout.Height then
  begin
    ldx := (ALayout.Width - ALayout.Height) / 2;
    ldy := 0;
    lw := ALayout.Height;
  end
  else if ALayout.Width < ALayout.Height then
  begin
    ldx := 0;
    ldy := (ALayout.Height - ALayout.Width) / 2;
    lw := ALayout.Width;
  end;
  P.X := lPt1.X + ldx;
  { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
  P.Y := lPt1.Y - ldy;
  APage.DrawRect(P, lw, lw, 1, False, True);
end;

procedure TFPReportExportPDF.RenderShapeTriangle(const APage: TPDFPage; Alpt: TPDFCoord;
    const AOrientation: TFPReportOrientation; const ALayout: TFPReportLayout);
var
  lPt1, lPt2, lPt3: TPDFCoord;  // original Report point
  lOrigin: TPDFCoord;
  W, H: TFPReportUnits;
begin
  lOrigin.X := AlPt.X;
  lOrigin.Y := ALPT.Y - ALayout.Height;
    W:=ALayout.Width;
    H:=ALayout.Height;
    case AOrientation of
    orNorth:
      begin
      lPt1.X := lOrigin.X + (W / 2); //      1
      lPt1.Y := lOrigin.Y;           //      /\
      lPt2.X := lOrigin.X;           //     /  \
      lPt2.Y := lOrigin.Y + H;       //    /____\
      lPt3.X := lOrigin.X + W;       //  2       3
      lPt3.Y := lPt2.Y;
      end;
    orNorthEast:
      begin
      lPt1.X := lOrigin.X + (W );    //   +-------1
      lPt1.Y  := lOrigin.Y;          //   |       |
      lPt2.X := lOrigin.X;           //   2       |
      lPt2.Y  := lOrigin.Y + H/2;    //   |       |
      lPt3.X := lOrigin.X + W/2;     //   +---3---+
      lPt3.Y  := lPt1.Y + H;
      end;
    orSouth:
      begin
      lPt1.X := lOrigin.X;            //  1 ------ 2
      lPt1.Y  := lOrigin.Y;           //    \    /
      lPt2.X := lOrigin.X+ W;         //     \  /
      lPt2.Y  := lOrigin.Y;           //      \/
      lPt3.X := lOrigin.X + (W / 2);  //      3
      lPt3.Y  := lOrigin.Y+H;
      end;
    orSouthEast:
      begin
      lPt1.X := lOrigin.X + (W/2);   //   +---1---+
      lPt1.Y  := lOrigin.Y;          //   |       |
      lPt2.X := lOrigin.X;           //   2       |
      lPt2.Y  := lOrigin.Y + H/2;    //   |       |
      lPt3.X := lOrigin.X + W;       //   +-------3
      lPt3.Y  := lPt1.Y + H;
      end;
    orEast:
      begin
      lPt1.X := lOrigin.X;            //   1
      lPt1.Y  := lOrigin.Y ;          //   |\
      lPt2.X := lOrigin.X + W;        //   | \ 2
      lPt2.Y  := lOrigin.Y + (H / 2); //   | /
      lPt3.X := lOrigin.X;            //   |/
      lPt3.Y  := lOrigin.Y + H;       //   3
      end;
    orNorthWest:
      begin
      lPt1.X := lOrigin.X;           //   1-------+
      lPt1.Y  := lOrigin.Y;          //   |       |
      lPt2.X := lOrigin.X+W;         //   |       2
      lPt2.Y  := lOrigin.Y + H/2;    //   |       |
      lPt3.X := lOrigin.X + W/2;     //   +---3---+
      lPt3.Y  := lPt1.Y + H;
      end;
    orWest:
      begin
      lPt1.X := lOrigin.X + W;       //       1
      lPt1.Y  := lOrigin.Y;          //      /|
      lPt2.X := lOrigin.X;           //   2 / |
      lPt2.Y  := lOrigin.Y + H / 2;  //     \ |
      lPt3.X := lOrigin.X + W;       //      \|
      lPt3.Y  := lOrigin.Y+ H;       //       3
      end;
    orSouthWest:
      begin
      lPt1.X := lOrigin.X+ H/2;      //   +---1---+
      lPt1.Y  := lOrigin.Y;          //   |       |
      lPt2.X := lOrigin.X+W;         //   |       2
      lPt2.Y  := lOrigin.Y + H/2;    //   |       |
      lPt3.X := lOrigin.X ;          //   3-------+
      lPt3.Y  := lPt1.Y + H;
      end;
    end;
  APage.DrawLine(lPt1, lPt2, 1);
  APage.DrawLine(lPt2, lPt3, 1);
  APage.DrawLine(lPt3, lPt1, 1);
end;

procedure TFPReportExportPDF.RenderShapeRoundedRect(const APage: TPDFPage; const lpt1: TPDFCoord;
  const ARadius: TFPReportUnits; const ALayout: TFPReportLayout);
begin

end;

constructor TFPReportExportPDF.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FDocument := nil;
  FFileName :=  ApplicationName + '.pdf';
  FAutoSave := True;
end;

destructor TFPReportExportPDF.Destroy;
begin
  FDocument.Free;
  inherited Destroy;
end;

class function TFPReportExportPDF.Name: String;
begin
  Result:='PDF';
end;

class function TFPReportExportPDF.Description: String;
begin
  Result:='PDF file';
end;

class function TFPReportExportPDF.DefaultExtension: String;
begin
  Result:='.pdf';
end;

procedure TFPReportExportPDF.SetFileName(const aFileName: String);
begin
  Filename:=aFileName;
end;


procedure TFPReportExportPDF.RenderImage(aRect: TFPReportRect; var AImage: TFPCustomImage);

var
  LPT : TPDFCoord;
  Idx : Integer;
  pdfimg: TPDFImageItem;

begin
  LPT.X:=aRect.Left;
  LPT.Y:=aRect.Top;
  idx:=Document.Images.Count-1;
  While (Idx>=0) and not Document.Images.Images[idx].Equals(AImage) do
    Dec(Idx);
  if idx = -1 then
    begin
    pdfimg := Document.Images.AddImageItem;
    pdfimg.Image := AImage;
    pdfimg.OwnsImage:=True;
    idx := Document.Images.Count-1;
    end;
  CurrentPage.DrawImage(lPt, aRect.Width, ARect.Height, idx);
  aImage:=Nil; // PDF now owns the image
end;

procedure TFPReportExportPDF.DoExecute(const ARTObjects: TFPList);
var
  pg: TPDFPage;
  p, b, m: integer;
  rpage: TFPReportPage;
  rband: TFPReportCustomBand;
  lPt1: TPDFCoord;  // original Report point
  lPDFPaper: TPDFPaper;

begin
  SetupPDFDocument;

  for p := 0 to (ARTObjects.Count - 1) do
  begin
    rpage := TFPReportPage(ARTObjects[p]);

    pg := FDocument.Pages.AddPage;
    FCurrentPage:=pg;
    case rpage.PageSize.PaperName of
      'A4':     pg.PaperType := ptA4;
      'A5':     pg.PaperType := ptA5;
      'Letter': pg.PaperType := ptLetter;
      'Legal':  pg.PaperType := ptLegal;
      'DL':     pg.PaperType := ptDL;
      'C5':     pg.PaperType := ptC5;
      'B5':     pg.PaperType := ptB5
      else
      begin
        lPDFPaper.W := Round(mmToPDF(rpage.PageSize.Width));
        lPDFPaper.H := Round(mmToPDF(rpage.PageSize.Height));
        pg.Paper := lPDFPaper;
        pg.PaperType := ptCustom;
      end;
    end;  { case PaperName }
    pg.UnitOfMeasure := uomMillimeters; { report measurements are always in millimeter units }

    // This must appear before configuring the pg.Matrix
    if rpage.Orientation = poLandscape then
      pg.Orientation := ppoLandscape;

    // Convert from the Cartesian coordinate system to the Screen coordinate system
    pg.Matrix.SetYScalation(-1);
    pg.Matrix.SetYTranslation(pg.GetPaperHeight);

    for b := 0 to (rpage.BandCount - 1) do
      begin
      rband := rpage.Bands[b];
      lPt1.X := rband.RTLayout.Left;
      { PDF origin coordinate is Bottom-Left, and Report Layout is Top-Left }
      lPt1.Y := rband.RTLayout.Top + rband.RTLayout.Height;
      RenderFrame(pg, rband, rband.Frame, lPt1, rband.RTLayout.Width, rband.RTLayout.Height);
      for m := 0 to (rband.ChildCount - 1) do
        RenderElement(pg, rband, rband.Child[m]);
      end;
    Document.Sections[0].AddPage(pg);
  end;
  if FAutoSave then
    SaveToFile;
end;

procedure TFPReportExportPDF.RenderElement(pg : TPDFPage; ABand : TFPReportCustomBand; el : TFPReportElement);

Var
  C : TFPReportPoint;
  lpt : TPDFCoord;

begin
  if (el is TFPReportCustomMemo) then
    RenderMemo(pg, aband, TFPReportCustomMemo(el))
  else if (el is TFPReportCustomShape) then
    RenderShape(pg, aband, TFPReportCustomShape(el))
  else if (el is TFPReportCustomImage) then
    RenderImage(pg, aband, TFPReportCustomImage(el))
  else if (el is TFPReportCustomCheckbox) then
    RenderCheckbox(pg, aband, TFPReportCustomCheckbox(el))
  else
    begin
    // PDF coords
    lPt.X := ABand.RTLayout.Left + el.RTLayout.Left;
    lPt.Y := ABand.RTLayout.Top + el.RTLayout.Top + el.RTLayout.Height;
    RenderFrame(pg, ABand, el.Frame, lPt, el.RTLayout.Width, el.RTLayout.Height);
    C.Left:=aband.RTLayout.Left;
    // Compensate for add of height which RenderUnknownElement will do
    C.Top:=aband.RTLayout.Top + el.RTLayout.Height;
    RenderUnknownElement(C,El,72);
    end;
end;

initialization
  TFPReportExportPDF.RegisterExporter;
end.

