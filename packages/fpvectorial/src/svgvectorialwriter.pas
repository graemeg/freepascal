{
Writes an SVG Document

License: The same modified LGPL as the Free Pascal RTL
         See the file COPYING.modifiedLGPL for more details

AUTHORS: Felipe Monteiro de Carvalho
}
unit svgvectorialwriter;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, math, fpvectorial;

type
  { TvSVGVectorialWriter }

  TvSVGVectorialWriter = class(TvCustomVectorialWriter)
  private
    FPointSeparator, FCommaSeparator: TFormatSettings;
    procedure WriteDocumentSize(AStrings: TStrings; AData: TvVectorialDocument);
    procedure WriteDocumentName(AStrings: TStrings; AData: TvVectorialDocument);
    procedure WritePaths(AStrings: TStrings; AData: TvVectorialDocument);
    procedure WriteTexts(AStrings: TStrings; AData: TvVectorialDocument);
    procedure ConvertFPVCoordinatesToSVGCoordinates(
      const AData: TvVectorialDocument;
      const ASrcX, ASrcY: Double; var ADestX, ADestY: double);
  public
    { General reading methods }
    procedure WriteToStrings(AStrings: TStrings; AData: TvVectorialDocument); override;
  end;

implementation

const
  // SVG requires hardcoding a DPI value

  // The Opera Browser and Inkscape use 90 DPI, so we follow that

  // 1 Inch = 25.4 milimiters
  // 90 inches per pixel = (1 / 90) * 25.4 = 0.2822
  // FLOAT_MILIMETERS_PER_PIXEL = 0.3528; // DPI 72 = 1 / 72 inches per pixel

  FLOAT_MILIMETERS_PER_PIXEL = 0.2822; // DPI 90 = 1 / 90 inches per pixel
  FLOAT_PIXELS_PER_MILIMETER = 3.5433; // DPI 90 = 1 / 90 inches per pixel

{ TvSVGVectorialWriter }

procedure TvSVGVectorialWriter.WriteDocumentSize(AStrings: TStrings; AData: TvVectorialDocument);
begin
  AStrings.Add('  width="' + FloatToStr(AData.Width, FPointSeparator) + 'mm"');
  AStrings.Add('  height="' + FloatToStr(AData.Height, FPointSeparator) + 'mm"');
end;

procedure TvSVGVectorialWriter.WriteDocumentName(AStrings: TStrings; AData: TvVectorialDocument);
begin
  AStrings.Add('  sodipodi:docname="New document 1">');
end;

{@@
  SVG Coordinate system measures things only in pixels, so that we have to
  hardcode a DPI value for the screen, which is usually 72.
  FPVectorial uses only milimeters (mm).

  The initial point in FPVectorial is in the bottom-left corner of the document
  and it grows to the top and to the right. In SVG, on the other hand, the
  initial point is in the top-left corner, growing to the bottom and right.
  Besides that, coordinates in SVG are also lengths in comparison to the
  previous point and not absolute coordinates.

  SVG uses commas "," to separate the X,Y coordinates, so it always uses points
  "." as decimal separators and uses no thousand separators
}
procedure TvSVGVectorialWriter.WritePaths(AStrings: TStrings; AData: TvVectorialDocument);
var
  i, j: Integer;
  PathStr: string;
  lPath: TPath;
  PtX, PtY, OldPtX, OldPtY: double;
  BezierCP1X, BezierCP1Y, BezierCP2X, BezierCP2Y: double;
  segment: TPathSegment;
  l2DSegment: T2DSegment absolute segment;
  l2DBSegment: T2DBezierSegment absolute segment;
begin
  for i := 0 to AData.GetPathCount() - 1 do
  begin
    OldPtX := 0;
    OldPtY := 0;

    PathStr := '';
    lPath := AData.GetPath(i);
    lPath.PrepareForSequentialReading;

    for j := 0 to lPath.Len - 1 do
    begin
      segment := TPathSegment(lPath.Next());

      if (segment.SegmentType <> st2DLine)
        and (segment.SegmentType <> stMoveTo)
        and (segment.SegmentType <> st2DBezier)
        then Break; // unsupported line type

      // Coordinate conversion from fpvectorial to SVG
      ConvertFPVCoordinatesToSVGCoordinates(
        AData, l2DSegment.X, l2DSegment.Y, PtX, PtY);
      PtX := PtX - OldPtX;
      PtY := PtY - OldPtY;

      if (segment.SegmentType = stMoveTo) then
      begin
        PathStr := PathStr + 'm '
          + FloatToStr(PtX, FPointSeparator) + ','
          + FloatToStr(PtY, FPointSeparator) + ' ';
      end
      else if (segment.SegmentType = st2DLine) then
      begin
        PathStr := PathStr + 'l '
          + FloatToStr(PtX, FPointSeparator) + ','
          + FloatToStr(PtY, FPointSeparator) + ' ';
      end
      else if (segment.SegmentType = st2DBezier) then
      begin
        // Converts all coordinates to absolute values
        ConvertFPVCoordinatesToSVGCoordinates(
          AData, l2DBSegment.X2, l2DBSegment.Y2, BezierCP1X, BezierCP1Y);
        ConvertFPVCoordinatesToSVGCoordinates(
          AData, l2DBSegment.X3, l2DBSegment.Y3, BezierCP2X, BezierCP2Y);

        // Transforms them into values relative to the initial point
        BezierCP1X := BezierCP1X - OldPtX;
        BezierCP1Y := BezierCP1Y - OldPtY;
        BezierCP2X := BezierCP2X - OldPtX;
        BezierCP2Y := BezierCP2Y - OldPtY;

        // PtX and PtY already contains the destination point

        // Now render our 2D cubic bezier
        PathStr := PathStr + 'c '
          + FloatToStr(BezierCP1X, FPointSeparator) + ','
          + FloatToStr(BezierCP1Y, FPointSeparator) + ' '
          + FloatToStr(BezierCP2X, FPointSeparator) + ','
          + FloatToStr(BezierCP2Y, FPointSeparator) + ' '
          + FloatToStr(PtX, FPointSeparator) + ','
          + FloatToStr(PtY, FPointSeparator) + ' '
          ;
      end;

      // Store the current position for future points
      OldPtX := OldPtX + PtX;
      OldPtY := OldPtY + PtY;
    end;

    AStrings.Add('  <path');
    AStrings.Add('    style="fill:none;stroke:#000000;stroke-width:1px;stroke-linecap:butt;stroke-linejoin:miter;stroke-opacity:1"');
    AStrings.Add('    d="' + PathStr + '"');
    AStrings.Add('  id="path' + IntToStr(i) + '" />');
  end;
end;

procedure TvSVGVectorialWriter.ConvertFPVCoordinatesToSVGCoordinates(
  const AData: TvVectorialDocument; const ASrcX, ASrcY: Double; var ADestX,
  ADestY: double);
begin
  ADestX := ASrcX / FLOAT_MILIMETERS_PER_PIXEL;
  ADestY := (AData.Height - ASrcY) / FLOAT_MILIMETERS_PER_PIXEL;
end;

procedure TvSVGVectorialWriter.WriteToStrings(AStrings: TStrings;
  AData: TvVectorialDocument);
begin
  // Format seetings to convert a string to a float
  FPointSeparator := DefaultFormatSettings;
  FPointSeparator.DecimalSeparator := '.';
  FPointSeparator.ThousandSeparator := '#';// disable the thousand separator
  FCommaSeparator := DefaultFormatSettings;
  FCommaSeparator.DecimalSeparator := ',';
  FCommaSeparator.ThousandSeparator := '#';// disable the thousand separator

  // Headers
  AStrings.Add('<?xml version="1.0" encoding="UTF-8" standalone="no"?>');
  AStrings.Add('<!-- Created with fpVectorial (http://wiki.lazarus.freepascal.org/fpvectorial) -->');
  AStrings.Add('');
  AStrings.Add('<svg');
  AStrings.Add('  xmlns:dc="http://purl.org/dc/elements/1.1/"');
  AStrings.Add('  xmlns:cc="http://creativecommons.org/ns#"');
  AStrings.Add('  xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"');
  AStrings.Add('  xmlns:svg="http://www.w3.org/2000/svg"');
  AStrings.Add('  xmlns="http://www.w3.org/2000/svg"');
  AStrings.Add('  xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"');
  WriteDocumentSize(AStrings, AData);
  AStrings.Add('  id="svg2"');
  AStrings.Add('  version="1.1"');
  WriteDocumentName(AStrings, AData);

  // Now data
  AStrings.Add('  <g id="layer1">');
  WritePaths(AStrings, AData);
  WriteTexts(AStrings, AData);
  AStrings.Add('  </g>');

  // finalization
  AStrings.Add('</svg>');
end;

procedure TvSVGVectorialWriter.WriteTexts(AStrings: TStrings; AData: TvVectorialDocument);
var
  i, j, FontSize: Integer;
  TextStr, FontName, SVGFontFamily: string;
  lText: TvText;
  PtX, PtY: double;
begin
  for i := 0 to AData.GetTextCount() - 1 do
  begin
    TextStr := '';
    lText := AData.GetText(i);

    ConvertFPVCoordinatesToSVGCoordinates(
        AData, lText.X, lText.Y, PtX, PtY);

    TextStr := lText.Value;
    FontSize:= ceil(lText.FontSize / FLOAT_MILIMETERS_PER_PIXEL);
    SVGFontFamily := 'Arial, sans-serif';//lText.FontName;

    AStrings.Add('  <text ');
    AStrings.Add('    x="' + FloatToStr(PtX, FPointSeparator) + '"');
    AStrings.Add('    y="' + FloatToStr(PtY, FPointSeparator) + '"');
//    AStrings.Add('    font-size="' + IntToStr(FontSize) + '"'); Doesn't seam to work, we need to use the tspan
    AStrings.Add('    font-family="' + SVGFontFamily + '">');
    AStrings.Add('    <tspan ');
    AStrings.Add('      style="font-size:' + IntToStr(FontSize) + '" ');
//    AStrings.Add('      id="tspan2828" ');
    AStrings.Add('    >');
    AStrings.Add(TextStr + '</tspan></text>');
  end;
end;

initialization

  RegisterVectorialWriter(TvSVGVectorialWriter, vfSVG);

end.

