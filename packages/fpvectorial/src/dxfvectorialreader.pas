{
Reads DXF files

License: The same modified LGPL as the Free Pascal RTL
         See the file COPYING.modifiedLGPL for more details

AUTHORS: Felipe Monteiro de Carvalho

DXF is composed by records written in ASCII with the following structure:

0
SECTION
section_number
SECTION_NAME
<data>
0
ENDSEC
0

after all sections there is:

EOF

}
unit dxfvectorialreader;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, Math,
  fpvectorial, fpimage;

type
  { Used by tcutils.SeparateString }
  T10Strings = array[0..9] of shortstring;

  TDXFToken = class;

  TDXFTokens = TFPList;// TDXFToken;

  TDXFToken = class
    GroupCode: Integer;
    StrValue: string;
    FloatValue: double;
    IntValue: Integer;
    Childs: TDXFTokens;
    constructor Create;
    Destructor Destroy; override;
  end;

  TPolylineElement = record
    X, Y: Double;
    Color: TvColor;
  end;

  TSPLineElement = record
    X, Y: Double;
    KnotValue: Integer;
  end;

  TLWPOLYLINEElement = record
    X, Y: Double;
  end;

  { TDXFTokenizer }

  TDXFTokenizer = class
  public
    Tokens: TDXFTokens;
    constructor Create;
    Destructor Destroy; override;
    procedure ReadFromStrings(AStrings: TStrings);
    function  IsENTITIES_Subsection(AStr: string): Boolean;
  end;

  { TvDXFVectorialReader }

  TvDXFVectorialReader = class(TvCustomVectorialReader)
  private
    FPointSeparator: TFormatSettings;
    // HEADER data
    ANGBASE: Double;
    ANGDIR: Integer;
    INSBASE, EXTMIN, EXTMAX, LIMMIN, LIMMAX: T3DPoint;
    // Calculated HEADER data
    DOC_OFFSET: T3DPoint; // The DOC_OFFSET compensates for documents with huge coordinates
    // For building the POLYLINE objects which is composed of multiple records
    IsReadingPolyline: Boolean;
    Polyline: array of TPolylineElement;
    //
    function  SeparateString(AString: string; ASeparator: Char): T10Strings;
    procedure ReadHEADER(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_LINE(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_ARC(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_CIRCLE(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_DIMENSION(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_ELLIPSE(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_TEXT(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_LWPOLYLINE(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_SPLINE(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_POLYLINE(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_VERTEX(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_SEQEND(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_MTEXT(ATokens: TDXFTokens; AData: TvVectorialDocument);
    procedure ReadENTITIES_POINT(ATokens: TDXFTokens; AData: TvVectorialDocument);
    function  GetCoordinateValue(AStr: shortstring): Double;
    //
    function DXFColorIndexToVColor(AColorIndex: Integer): TvColor;
  public
    { General reading methods }
    Tokenizer: TDXFTokenizer;
    constructor Create; override;
    Destructor Destroy; override;
    procedure ReadFromStrings(AStrings: TStrings; AData: TvVectorialDocument); override;
  end;

implementation

{$ifndef Windows}
{$define FPVECTORIALDEBUG}
{$endif}

const
  // Items in the HEADER section

  // $ACADVER
  DXF_AUTOCAD_2010        = 'AC1024'; // AutoCAD 2011 and 2012 too
  DXF_AUTOCAD_2007        = 'AC1021'; // AutoCAD 2008 and 2009 too
  DXF_AUTOCAD_2004        = 'AC1018'; // AutoCAD 2005 and 2006 too
  DXF_AUTOCAD_2000        = 'AC1015'; // 1999  In some docs it is proposed as AC1500, but in practice I found AC1015
                                      // http://www.autodesk.com/techpubs/autocad/acad2000/dxf/
                                      // AutoCAD 2000i and 2002 too
  DXF_AUTOCAD_R14         = 'AC1014'; // 1997  http://www.autodesk.com/techpubs/autocad/acadr14/dxf/index.htm
  DXF_AUTOCAD_R13         = 'AC1012'; // 1994
  DXF_AUTOCAD_R11_and_R12 = 'AC1009'; // 1990
  DXF_AUTOCAD_R10         = 'AC1006'; // 1988
  DXF_AUTOCAD_R9          = 'AC1004';

  // Group Codes for ENTITIES
  DXF_ENTITIES_TYPE = 0;
  DXF_ENTITIES_HANDLE = 5;
  DXF_ENTITIES_LINETYPE_NAME = 6;
  DXF_ENTITIES_APPLICATION_GROUP = 102;
  DXF_ENTITIES_AcDbEntity = 100;
  DXF_ENTITIES_MODEL_OR_PAPER_SPACE = 67; // default=0=model, 1=paper
  DXF_ENTITIES_VISIBILITY = 60; // default=0 = Visible, 1 = Invisible

  // Obtained from http://www.generalcadd.com/pdf/LivingWithAutoCAD_v4.pdf
  // Valid for DXF up to AutoCad 2004, after that RGB is available
  AUTOCAD_COLOR_PALETTE: array[0..15] of TvColor =
  (
    (Red: $00; Green: $00; Blue: $00; Alpha: FPValphaOpaque), // 0 - Black
    (Red: $00; Green: $00; Blue: $80; Alpha: FPValphaOpaque), // 1 - Dark blue
    (Red: $00; Green: $80; Blue: $00; Alpha: FPValphaOpaque), // 2 - Dark green
    (Red: $00; Green: $80; Blue: $80; Alpha: FPValphaOpaque), // 3 - Dark cyan
    (Red: $80; Green: $00; Blue: $00; Alpha: FPValphaOpaque), // 4 - Dark red
    (Red: $80; Green: $00; Blue: $80; Alpha: FPValphaOpaque), // 5 - Dark Magenta
    (Red: $80; Green: $80; Blue: $00; Alpha: FPValphaOpaque), // 6 - Dark
    (Red: $c0; Green: $c0; Blue: $c0; Alpha: FPValphaOpaque), // 7 - Light Gray
    (Red: $80; Green: $80; Blue: $80; Alpha: FPValphaOpaque), // 8 - Medium Gray
    (Red: $00; Green: $00; Blue: $ff; Alpha: FPValphaOpaque), // 9 - Light blue
    (Red: $00; Green: $ff; Blue: $00; Alpha: FPValphaOpaque), // 10 - Light green
    (Red: $00; Green: $ff; Blue: $ff; Alpha: FPValphaOpaque), // 11 - Light cyan
    (Red: $ff; Green: $00; Blue: $00; Alpha: FPValphaOpaque), // 12 - Light red
    (Red: $ff; Green: $00; Blue: $ff; Alpha: FPValphaOpaque), // 13 - Light Magenta
    (Red: $ff; Green: $ff; Blue: $00; Alpha: FPValphaOpaque), // 14 - Light Yellow
    (Red: $ff; Green: $ff; Blue: $ff; Alpha: FPValphaOpaque)  // 15 - White
  );

{ TDXFToken }

constructor TDXFToken.Create;
begin
  inherited Create;

  Childs := TDXFTokens.Create;
end;

destructor TDXFToken.Destroy;
begin
  Childs.Free;

  inherited Destroy;
end;

{ TDXFTokenizer }

constructor TDXFTokenizer.Create;
begin
  inherited Create;

  Tokens := TDXFTokens.Create;
end;

destructor TDXFTokenizer.Destroy;
begin
  Tokens.Free;

  inherited Destroy;
end;

procedure TDXFTokenizer.ReadFromStrings(AStrings: TStrings);
var
  i: Integer;
  StrSectionGroupCode, StrSectionName: string;
  IntSectionGroupCode: Integer;
  CurTokenBase, NextTokenBase, SectionTokenBase: TDXFTokens;
  NewToken: TDXFToken;
  ParserState: Integer;
begin
  //  Tokens.ForEachCall(); deletecallback
  Tokens.Clear;

  CurTokenBase := Tokens;
  NextTokenBase := Tokens;
  i := 0;
  ParserState := 0;

  while i < AStrings.Count - 1 do
  begin
    CurTokenBase := NextTokenBase;

    // Now read and process the section name
    StrSectionGroupCode := AStrings.Strings[i];
    IntSectionGroupCode := StrToInt(Trim(StrSectionGroupCode));
    StrSectionName := AStrings.Strings[i+1];

    NewToken := TDXFToken.Create;
    NewToken.GroupCode := IntSectionGroupCode;
    NewToken.StrValue := StrSectionName;

    // Waiting for a section
    if ParserState = 0 then
    begin
      if (StrSectionName = 'SECTION') then
      begin
        ParserState := 1;
        NextTokenBase := NewToken.Childs;
      end
      else if (StrSectionName = 'EOF') then
      begin
        Exit;
      end
      else
      begin
        raise Exception.Create(Format(
          'TDXFTokenizer.ReadFromStrings: Expected SECTION, but got: %s', [StrSectionname]));
      end;
    end
    // Processing the section name
    else if ParserState = 1 then
    begin
      if (StrSectionName = 'HEADER') or
        (StrSectionName = 'CLASSES') or
        (StrSectionName = 'TABLES') or
        (StrSectionName = 'BLOCKS') or
        (StrSectionName = 'OBJECTS') or
        (StrSectionName = 'THUMBNAILIMAGE') then
      begin
        ParserState := 2;
        SectionTokenBase := CurTokenBase;
      end
      else if (StrSectionName = 'ENTITIES') then
      begin
        ParserState := 3;
        SectionTokenBase := CurTokenBase;
      end
      else
      begin
        raise Exception.Create(Format(
          'TDXFTokenizer.ReadFromStrings: Invalid section name: %s', [StrSectionname]));
      end;
    end
    // Reading a generic section
    else if ParserState = 2 then
    begin
      if StrSectionName = 'ENDSEC' then
      begin
        ParserState := 0;
        CurTokenBase := SectionTokenBase;
        NextTokenBase := Tokens;
      end;
    end
    // Reading the ENTITIES section
    else if ParserState = 3 then
    begin
      if IsENTITIES_Subsection(StrSectionName) then
      begin
        CurTokenBase := SectionTokenBase;
        NextTokenBase := NewToken.Childs;
      end
      else if StrSectionName = 'ENDSEC' then
      begin
        ParserState := 0;
        CurTokenBase := SectionTokenBase;
        NextTokenBase := Tokens;
      end;
    end;

    CurTokenBase.Add(NewToken);

    Inc(i, 2);
  end;
end;

function TDXFTokenizer.IsENTITIES_Subsection(AStr: string): Boolean;
begin
  Result :=
    (AStr = '3DFACE') or
    (AStr = '3DSOLID') or
    (AStr = 'ACAD_PROXY_ENTITY') or
    (AStr = 'ARC') or
    (AStr = 'ATTDEF') or
    (AStr = 'ATTRIB') or
    (AStr = 'BODY') or
    (AStr = 'CIRCLE') or
    (AStr = 'DIMENSION') or
    (AStr = 'ELLIPSE') or
    (AStr = 'HATCH') or
    (AStr = 'IMAGE') or
    (AStr = 'INSERT') or
    (AStr = 'LEADER') or
    (AStr = 'LINE') or
    (AStr = 'LWPOLYLINE') or
    (AStr = 'MLINE') or
    (AStr = 'MTEXT') or
    (AStr = 'OLEFRAME') or
    (AStr = 'OLE2FRAME') or
    (AStr = 'POINT') or
    (AStr = 'POLYLINE') or
    (AStr = 'RAY') or
    (AStr = 'REGION') or
    (AStr = 'SEQEND') or
    (AStr = 'SHAPE') or
    (AStr = 'SOLID') or
    (AStr = 'SPLINE') or
    (AStr = 'TEXT') or
    (AStr = 'TOLERANCE') or
    (AStr = 'TRACE') or
    (AStr = 'VERTEX') or
    (AStr = 'VIEWPORT') or
    (AStr = 'XLINE');
end;

{ TvDXFVectorialReader }

{@@
  Reads a string and separates it in substring
  using ASeparator to delimite them.

  Limits:

  Number of substrings: 10 (indexed 0 to 9)
  Length of each substring: 255 (they are shortstrings)
}
function TvDXFVectorialReader.SeparateString(AString: string; ASeparator: Char): T10Strings;
var
  i, CurrentPart: Integer;
begin
  CurrentPart := 0;

  { Clears the result }
  for i := 0 to 9 do Result[i] := '';

  { Iterates througth the string, filling strings }
  for i := 1 to Length(AString) do
  begin
    if Copy(AString, i, 1) = ASeparator then
    begin
      Inc(CurrentPart);

      { Verifies if the string capacity wasn't exceeded }
      if CurrentPart > 9 then Exit;
    end
    else
      Result[CurrentPart] := Result[CurrentPart] + Copy(AString, i, 1);
  end;
end;

procedure TvDXFVectorialReader.ReadHEADER(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  i, j: Integer;
  CurToken: TDXFToken;
  CurField: P3DPoint;
begin
  i := 0;
  while i < ATokens.Count do
  begin
    CurToken := TDXFToken(ATokens.Items[i]);
    if CurToken.StrValue = '$ANGBASE' then
    begin
      CurToken := TDXFToken(ATokens.Items[i+1]);
      ANGBASE := StrToFloat(CurToken.StrValue, FPointSeparator);
      Inc(i);
    end
    else if CurToken.StrValue = '$ANGDIR' then
    begin
      CurToken := TDXFToken(ATokens.Items[i+1]);
      ANGDIR := StrToInt(CurToken.StrValue);
      Inc(i);
    end
    // This indicates the size of the document
    else if (CurToken.StrValue = '$INSBASE') or
      (CurToken.StrValue = '$EXTMIN') or (CurToken.StrValue = '$EXTMAX') or
      (CurToken.StrValue = '$LIMMIN') or (CurToken.StrValue = '$LIMMAX') then
    begin
      if (CurToken.StrValue = '$INSBASE') then CurField := @INSBASE
      else if (CurToken.StrValue = '$EXTMIN') then CurField := @EXTMIN
      else if (CurToken.StrValue = '$EXTMAX') then CurField := @EXTMAX
      else if (CurToken.StrValue = '$LIMMIN') then CurField := @LIMMIN
      else if (CurToken.StrValue = '$LIMMAX') then CurField := @LIMMAX;

      // Check the next 2 items and verify if they are the values of the size of the document
      for j := 0 to 1 do
      begin
        CurToken := TDXFToken(ATokens.Items[i+1]);
        case CurToken.GroupCode of
        10:
        begin;
          CurField^.X := StrToFloat(CurToken.StrValue, FPointSeparator);
          Inc(i);
        end;
        20:
        begin
          CurField^.Y := StrToFloat(CurToken.StrValue, FPointSeparator);
          Inc(i);
        end;
        end;
      end;
    end;

    Inc(i);
  end;

  // After getting all the data, we can try to make some sense out of it

  // Sometimes EXTMIN comes as 10^20 and EXTMAX as -10^20, which makes no sence
  // In these cases we need to ignore them.
  if (EXTMIN.X > 10000000000) or (EXTMIN.X < -10000000000)
  or (EXTMAX.X > 10000000000) or (EXTMAX.X < -10000000000) then
  begin
    DOC_OFFSET.X := 0;
    DOC_OFFSET.Y := 0;

    AData.Width := LIMMAX.X;
    AData.Height := LIMMAX.Y;
  end
  else
  begin
    // The size of the document seams to be given by:
    // DOC_SIZE = min(EXTMAX, LIMMAX) - DOC_OFFSET;
    // if EXTMIN is <> -infinite then DOC_OFFSET = EXTMIN else DOC_OFFSET = (0, 0)
    // We will shift the whole document so that it has only positive coordinates and
    // DOC_OFFSET will be utilized for that

    if EXTMIN.X > -100 then
    begin
      DOC_OFFSET.X := EXTMIN.X;
      DOC_OFFSET.Y := EXTMIN.Y;
    end
    else FillChar(DOC_OFFSET, sizeof(T3DPoint), #0);

    AData.Width := min(EXTMAX.X, LIMMAX.X) - DOC_OFFSET.X;
    AData.Height := min(EXTMAX.Y, LIMMAX.Y) - DOC_OFFSET.Y;
  end;
end;

procedure TvDXFVectorialReader.ReadENTITIES(ATokens: TDXFTokens; AData: TvVectorialDocument);
var
  i: Integer;
  CurToken: TDXFToken;
begin
  IsReadingPolyline := False;

  for i := 0 to ATokens.Count - 1 do
  begin
    CurToken := TDXFToken(ATokens.Items[i]);
    if CurToken.StrValue = 'ARC' then ReadENTITIES_ARC(CurToken.Childs, AData)
    else if CurToken.StrValue = 'CIRCLE' then ReadENTITIES_CIRCLE(CurToken.Childs, AData)
    else if CurToken.StrValue = 'DIMENSION' then ReadENTITIES_DIMENSION(CurToken.Childs, AData)
    else if CurToken.StrValue = 'ELLIPSE' then ReadENTITIES_ELLIPSE(CurToken.Childs, AData)
    else if CurToken.StrValue = 'LINE' then ReadENTITIES_LINE(CurToken.Childs, AData)
    else if CurToken.StrValue = 'TEXT' then ReadENTITIES_TEXT(CurToken.Childs, AData)
    else if CurToken.StrValue = 'LWPOLYLINE' then ReadENTITIES_LWPOLYLINE(CurToken.Childs, AData)
    else if CurToken.StrValue = 'SPLINE' then ReadENTITIES_SPLINE(CurToken.Childs, AData)
    else if CurToken.StrValue = 'POINT' then ReadENTITIES_POINT(CurToken.Childs, AData)
    else if CurToken.StrValue = 'MTEXT' then ReadENTITIES_MTEXT(CurToken.Childs, AData)
    // A Polyline can have multiple child objects
    else if CurToken.StrValue = 'POLYLINE' then
    begin
      IsReadingPolyline := True;
      ReadENTITIES_POLYLINE(CurToken.Childs, AData);
    end
    else if CurToken.StrValue = 'VERTEX' then ReadENTITIES_VERTEX(CurToken.Childs, AData)
    else if CurToken.StrValue = 'SEQEND' then
    begin
      ReadENTITIES_SEQEND(CurToken.Childs, AData);
      IsReadingPolyline := False;
    end
    else
    begin
      // ...
    end;
  end;
end;

procedure TvDXFVectorialReader.ReadENTITIES_LINE(ATokens: TDXFTokens; AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  // LINE
  LineStartX, LineStartY, LineStartZ: Double;
  LineEndX, LineEndY, LineEndZ: Double;
  LLineColor: TvColor;
begin
  // Initial values
  LineStartX := 0;
  LineStartY := 0;
  LineStartZ := 0;
  LineEndX := 0;
  LineEndY := 0;
  LineEndZ := 0;
  LLineColor := clvBlack;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 11, 21, 31, 62] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      10: LineStartX := CurToken.FloatValue;
      20: LineStartY := CurToken.FloatValue;
      30: LineStartZ := CurToken.FloatValue;
      11: LineEndX := CurToken.FloatValue;
      21: LineEndY := CurToken.FloatValue;
      31: LineEndZ := CurToken.FloatValue;
      62: LLineColor := DXFColorIndexToVColor(Trunc(CurToken.FloatValue));
    end;
  end;

  // Position fixing for documents with negative coordinates
  LineStartX := LineStartX - DOC_OFFSET.X;
  LineStartY := LineStartY - DOC_OFFSET.Y;
  LineEndX := LineEndX - DOC_OFFSET.X;
  LineEndY := LineEndY - DOC_OFFSET.Y;

  // And now write it
  {$ifdef FPVECTORIALDEBUG}
 // WriteLn(Format('Adding Line from %f,%f to %f,%f', [LineStartX, LineStartY, LineEndX, LineEndY]));
  {$endif}
  AData.StartPath(LineStartX, LineStartY);
  AData.AddLineToPath(LineEndX, LineEndY, LLineColor);
  AData.EndPath();
end;

{
Arcs are always counter-clockwise in DXF

100 Subclass marker (AcDbCircle)
39 Thickness (optional; default = 0)
10 Center point (in OCS) DXF: X value; APP: 3D point
20, 30 DXF: Y and Z values of center point (in OCS)
40 Radius
100 Subclass marker (AcDbArc)
50 Start angle (degrees)
51 End angle (degrees)
210 Extrusion direction. (optional; default = 0, 0, 1) DXF: X value; APP: 3D vector
220, 230 DXF: Y and Z values of extrusion direction (optional)
}
procedure TvDXFVectorialReader.ReadENTITIES_ARC(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  CenterX, CenterY, CenterZ, Radius, StartAngle, EndAngle: Double;
  LColor: TvColor;
begin
  CenterX := 0.0;
  CenterY := 0.0;
  CenterZ := 0.0;
  Radius := 0.0;
  StartAngle := 0.0;
  EndAngle := 0.0;
  LColor := clvBlack;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 40, 50, 51, 62] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      10: CenterX := CurToken.FloatValue;
      20: CenterY := CurToken.FloatValue;
      30: CenterZ := CurToken.FloatValue;
      40: Radius := CurToken.FloatValue;
      50: StartAngle := CurToken.FloatValue;
      51: EndAngle := CurToken.FloatValue;
      62: LColor := DXFColorIndexToVColor(Trunc(CurToken.FloatValue));
    end;
  end;

  // In DXF the EndAngle is always greater then the StartAngle.
  // If it isn't then sum 360 to it to make sure we don't get wrong results
  if EndAngle < StartAngle then EndAngle := EndAngle + 360;

  // Position fixing for documents with negative coordinates
  CenterX := CenterX - DOC_OFFSET.X;
  CenterY := CenterY - DOC_OFFSET.Y;

  {$ifdef FPVECTORIALDEBUG}
  WriteLn(Format('Adding Arc Center=%f,%f Radius=%f StartAngle=%f EndAngle=%f',
    [CenterX, CenterY, Radius, StartAngle, EndAngle]));
  {$endif}
  AData.AddCircularArc(CenterX, CenterY, CenterZ, Radius, StartAngle, EndAngle, LColor);
end;

{
Group codes	Description
100 Subclass marker (AcDbCircle)
39 Thickness (optional; default = 0)
10 Center point (in OCS) DXF: X value; APP: 3D point
20, 30 DXF: Y and Z values of center point (in OCS)
40 Radius
210 Extrusion direction (optional; default = 0, 0, 1) DXF: X value; APP: 3D vector
220, 230 DXF: Y and Z values of extrusion direction  (optional)
}
procedure TvDXFVectorialReader.ReadENTITIES_CIRCLE(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  CircleCenterX, CircleCenterY, CircleCenterZ, CircleRadius: Double;
begin
  CircleCenterX := 0.0;
  CircleCenterY := 0.0;
  CircleCenterZ := 0.0;
  CircleRadius := 0.0;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 40] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      10: CircleCenterX := CurToken.FloatValue;
      20: CircleCenterY := CurToken.FloatValue;
      30: CircleCenterZ := CurToken.FloatValue;
      40: CircleRadius := CurToken.FloatValue;
    end;
  end;

  // Position fixing for documents with negative coordinates
  CircleCenterX := CircleCenterX - DOC_OFFSET.X;
  CircleCenterY := CircleCenterY - DOC_OFFSET.Y;

  AData.AddCircle(CircleCenterX, CircleCenterY,
    CircleCenterZ, CircleRadius);
end;

{
Group codes Description
100 Subclass marker (AcDbDimension)
2 Name of the block that contains the entities that make up the dimension picture
10 Definition point (in WCS) DXF: X value; APP: 3D point
20, 30 DXF: Y and Z values of definition point (in WCS)
11 Middle point of dimension text (in OCS) DXF: X value; APP: 3D point
21, 31 DXF: Y and Z values of middle point of dimension text (in OCS)
70 Dimension type.
  Values 0-6 are integer values that represent the dimension type.
  Values 32, 64, and 128 are bit values, which are added to the integer values
  (value 32 is always set in R13 and later releases).
  0 = Rotated, horizontal, or vertical; 1 = Aligned;
  2 = Angular; 3 = Diameter; 4 = Radius;
  5 = Angular 3 point; 6 = Ordinate;
  32 = Indicates that the block reference (group code 2) is referenced by this dimension only.
  64 = Ordinate type. This is a bit value (bit 7) used only with integer value 6.
    If set, ordinate is X-type; if not set, ordinate is Y-type.
  128 = This is a bit value (bit 8) added to the other group 70 values
    if the dimension text has been positioned at a user-defined location
    rather than at the default location.
71 Attachment point:
  1 = Top left; 2 = Top center; 3 = Top right;
  4 = Middle left; 5 = Middle center; 6 = Middle right;
  7 = Bottom left; 8 = Bottom center; 9 = Bottom right
72 Dimension text line spacing style (optional):
  1(or missing) = At least (taller characters will override)
  2 = Exact (taller characters will not override)
41 Dimension text line spacing factor (optional):
  Percentage of default (3-on-5) line spacing to be applied. Valid values range from 0.25 to 4.00.
42 Actual measurement (optional; read-only value)
1 Dimension text explicitly entered by the user. Optional; default is the measurement.
  If null or "<>", the dimension measurement is drawn as the text,
  if " " (one blank space), the text is suppressed. Anything else is drawn as the text.
53 The optional group code 53 is the rotation angle of the dimension
  text away from its default orientation (the direction of the dimension line)  (optional).
51 All dimension types have an optional 51 group code, which indicates the
  horizontal direction for the dimension entity. The dimension entity determines
  the orientation of dimension text and lines for horizontal, vertical, and
  rotated linear dimensions.
  This group value is the negative of the angle between the OCS X axis
  and the UCS X axis. It is always in the XY plane of the OCS.
210 Extrusion direction (optional; default = 0, 0, 1) DXF: X value; APP: 3D vector
220, 230 DXF: Y and Z values of extrusion direction  (optional)
3 Dimension style name

Aligned Dimension Group Codes

100 Subclass marker (AcDbAlignedDimension)
12 Insertion point for clones of a dimension-Baseline and Continue (in OCS) DXF: X value; APP: 3D point
22, 32 DXF: Y and Z values of insertion point for clones of a dimension-Baseline and Continue (in OCS)
13 Definition point for linear and angular dimensions (in WCS) DXF: X value; APP: 3D point
23, 33 DXF: Y and Z values of definition point for linear and angular dimensions (in WCS)
14 Definition point for linear and angular dimensions (in WCS) DXF: X value; APP: 3D point
24, 34 DXF: Y and Z values of definition point for linear and angular dimensions (in WCS)

  |--text--|->10,20
  |        |
  |        |
  X->14,24 X->13,23
}
procedure TvDXFVectorialReader.ReadENTITIES_DIMENSION(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  // DIMENSION
  BaseLeft, BaseRight, DimensionRight, DimensionLeft, TmpPoint: T3DPoint;
  IsAlignedDimension: Boolean = False;
begin
  // Initial values
  BaseLeft.X := 0;
  BaseLeft.Y := 0;
  BaseRight.X := 0;
  BaseRight.X := 0;
  DimensionRight.X := 0;
  DimensionRight.Y := 0;
  DimensionLeft.X := 0;
  DimensionLeft.Y := 0;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 11, 21, 31, 13, 23, 33, 14, 24, 34] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      10: DimensionRight.X := CurToken.FloatValue;
      20: DimensionRight.Y := CurToken.FloatValue;
      30: DimensionRight.Z := CurToken.FloatValue;
      13: BaseRight.X := CurToken.FloatValue;
      23: BaseRight.Y := CurToken.FloatValue;
      33: BaseRight.Z := CurToken.FloatValue;
      14: BaseLeft.X := CurToken.FloatValue;
      24: BaseLeft.Y := CurToken.FloatValue;
      34: BaseLeft.Z := CurToken.FloatValue;
      100:
      begin
        if CurToken.StrValue = 'AcDbAlignedDimension' then IsAlignedDimension := True;
      end;
    end;
  end;

  // And now write it
  {$ifdef FPVECTORIALDEBUG}
//  WriteLn(Format('Adding Line from %f,%f to %f,%f', [LineStartX, LineStartY, LineEndX, LineEndY]));
  {$endif}
  if IsAlignedDimension then
  begin
    // Now make sure that we actually that BaseLeft is to the left of BaseRight
    if BaseRight.X < BaseLeft.X then
    begin
      TmpPoint := BaseRight;
      BaseRight := BaseLeft;
      BaseLeft := TmpPoint;
    end;

    // Now check if we are a horizontal or vertical dimension

    // horizontal
    //
    //DL____ DR
    //  |  |
    //  |  |
    // BL  BR
    if DimensionRight.X = BaseRight.X then
    begin
      DimensionLeft.X := BaseLeft.X;
      DimensionLeft.Y := DimensionRight.Y;
    end
    // vertical
    //
    // BL ----|DR
    //  BR  --|DL
    //
    // In this case we invert then DR and DL
    else if DimensionRight.Y = BaseLeft.Y then
    begin
      DimensionLeft := DimensionRight;
      DimensionRight.Y := BaseRight.Y;
    end
    // vertical
    //
    // BL ----|DL
    //  BR  --|DR
    //
    else if DimensionRight.Y = BaseRight.Y then
    begin
      DimensionLeft.X := DimensionRight.X;
      DimensionLeft.Y := BaseLeft.Y;
    end;

    AData.AddAlignedDimension(BaseLeft, BaseRight, DimensionLeft, DimensionRight);
  end;
end;

{
100 Subclass marker (AcDbEllipse)
10 Center point (in WCS) DXF: X value; APP: 3D point
20, 30 DXF: Y and Z values of center point (in WCS)
11 Endpoint of major axis, relative to the center (in WCS) DXF: X value; APP: 3D point
21, 31 DXF: Y and Z values of endpoint of major axis, relative to the center (in WCS)
210 Extrusion direction (optional; default = 0, 0, 1) DXF: X value; APP: 3D vector
220, 230 DXF: Y and Z values of extrusion direction  (optional)
40 Ratio of minor axis to major axis
41 Start parameter (this value is 0.0 for a full ellipse)
42 End parameter (this value is 2pi for a full ellipse)
}
procedure TvDXFVectorialReader.ReadENTITIES_ELLIPSE(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  CenterX, CenterY, CenterZ, MajorHalfAxis, MinorHalfAxis, Angle: Double;
begin
  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      10: CenterX := CurToken.FloatValue;
      20: CenterY := CurToken.FloatValue;
      30: CenterZ := CurToken.FloatValue;
    end;
  end;

  // Position fixing for documents with negative coordinates
  CenterX := CenterX - DOC_OFFSET.X;
  CenterY := CenterY - DOC_OFFSET.Y;

  //
  AData.AddEllipse(CenterX, CenterY, CenterZ, MajorHalfAxis, MinorHalfAxis, Angle);
end;

{
100 Subclass marker (AcDbText)
39 Thickness (optional; default = 0)
10 First alignment point (in OCS) DXF: X value; APP: 3D point
20, 30 DXF: Y and Z values of first alignment point (in OCS)
40 Text height
1 Default value (the string itself)
50 Text rotation (optional; default = 0)
41 Relative X scale factor-width (optional; default = 1)
  This value is also adjusted when fit-type text is used.
51 Oblique angle (optional; default = 0)
7 Text style name (optional, default = STANDARD)
71 Text generation flags (optional, default = 0):
  2 = Text is backward (mirrored in X).
  4 = Text is upside down (mirrored in Y).
72 Horizontal text justification type (optional, default = 0) integer codes (not bit-coded)
  0 = Left; 1= Center; 2 = Right
  3 = Aligned (if vertical alignment = 0)
  4 = Middle (if vertical alignment = 0)
  5 = Fit (if vertical alignment = 0)
  See the Group 72 and 73 integer codes table for clarification.
11 Second alignment point (in OCS) (optional)
  DXF: X value; APP: 3D point
  This value is meaningful only if the value of a 72 or 73 group is nonzero (if the justification is anything other than baseline/left).
21, 31 DXF: Y and Z values of second alignment point (in OCS) (optional)
210 Extrusion direction (optional; default = 0, 0, 1)
  DXF: X value; APP: 3D vector
220, 230 DXF: Y and Z values of extrusion direction (optional)
73 Vertical text justification type (optional, default = 0): integer codes (not bit- coded):
  0 = Baseline; 1 = Bottom; 2 = Middle; 3 = Top
  See the Group 72 and 73 integer codes table for clarification.
}
procedure TvDXFVectorialReader.ReadENTITIES_TEXT(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  PosX: Double = 0.0;
  PosY: Double = 0.0;
  PosZ: Double = 0.0;
  FontSize: Double = 10.0;
  Str: string = '';
begin
  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 40] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      1:  Str := CurToken.StrValue;
      10: PosX := CurToken.FloatValue;
      20: PosY := CurToken.FloatValue;
      30: PosZ := CurToken.FloatValue;
      40: FontSize := CurToken.FloatValue;
    end;
  end;

  // Position fixing for documents with negative coordinates
  PosX := PosX - DOC_OFFSET.X;
  PosY := PosY - DOC_OFFSET.Y;

  //
  AData.AddText(PosX, PosY, PosZ, '', Round(FontSize), Str);
end;

{.$define FPVECTORIALDEBUG_LWPOLYLINE}
procedure TvDXFVectorialReader.ReadENTITIES_LWPOLYLINE(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i, curPoint: Integer;
  // LINE
  LWPolyline: array of TLWPOLYLINEElement;
begin
  curPoint := -1;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 11, 21, 31] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    // Loads the coordinates
    // With Position fixing for documents with negative coordinates
    case CurToken.GroupCode of
      10:
      begin
        // Starting a new point
        Inc(curPoint);
        SetLength(LWPolyline, curPoint+1);

        LWPolyline[curPoint].X := CurToken.FloatValue - DOC_OFFSET.X;
      end;
      20: LWPolyline[curPoint].Y := CurToken.FloatValue - DOC_OFFSET.Y;
    end;
  end;

  // And now write it
  if curPoint >= 0 then // otherwise the polyline is empty of points
  begin
    AData.StartPath(LWPolyline[0].X, LWPolyline[0].Y);
    {$ifdef FPVECTORIALDEBUG_LWPOLYLINE}
    Write(Format('LWPOLYLINE ID=%d %f,%f', [AData.PathCount-1, LWPolyline[0].X, LWPolyline[0].Y]));
    {$endif}
    for i := 1 to curPoint do
    begin
      AData.AddLineToPath(LWPolyline[i].X, LWPolyline[i].Y);
      {$ifdef FPVECTORIALDEBUG_LWPOLYLINE}
       Write(Format(' %f,%f', [LWPolyline[i].X, LWPolyline[i].Y]));
      {$endif}
    end;
    {$ifdef FPVECTORIALDEBUG_LWPOLYLINE}
     WriteLn('');
    {$endif}
    AData.EndPath();
  end;
end;

{.$define FPVECTORIALDEBUG_SPLINE}
procedure TvDXFVectorialReader.ReadENTITIES_SPLINE(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i, curPoint: Integer;
  // LINE
  SPLine: array of TSPLineElement;
begin
  curPoint := -1;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 11, 21, 31] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    // Loads the coordinates
    // With Position fixing for documents with negative coordinates
    case CurToken.GroupCode of
      10:
      begin
        // Starting a new point
        Inc(curPoint);
        SetLength(SPLine, curPoint+1);

        SPLine[curPoint].X := CurToken.FloatValue - DOC_OFFSET.X;
      end;
      20: SPLine[curPoint].Y := CurToken.FloatValue - DOC_OFFSET.Y;
    end;
  end;

  // And now write it
  if curPoint >= 0 then // otherwise the polyline is empty of points
  begin
    AData.StartPath(SPLine[0].X, SPLine[0].Y);
    {$ifdef FPVECTORIALDEBUG_SPLINE}
    Write(Format('SPLINE ID=%d %f,%f', [AData.PathCount-1, SPLine[0].X, SPLine[0].Y]));
    {$endif}
    for i := 1 to curPoint do
    begin
      AData.AddLineToPath(SPLine[i].X, SPLine[i].Y);
      {$ifdef FPVECTORIALDEBUG_SPLINE}
       Write(Format(' %f,%f', [SPLine[i].X, SPLine[i].Y]));
      {$endif}
    end;
    {$ifdef FPVECTORIALDEBUG_SPLINE}
     WriteLn('');
    {$endif}
    AData.EndPath();
  end;
end;

procedure TvDXFVectorialReader.ReadENTITIES_POLYLINE(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
begin
  SetLength(Polyline, 0);
end;

procedure TvDXFVectorialReader.ReadENTITIES_VERTEX(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i, curPoint: Integer;
begin
  if not IsReadingPolyline then raise Exception.Create('[TvDXFVectorialReader.ReadENTITIES_VERTEX] Unexpected record: VERTEX before a POLYLINE');

  curPoint := Length(Polyline);
  SetLength(Polyline, curPoint+1);
  Polyline[curPoint].X := 0;
  Polyline[curPoint].Y := 0;
  Polyline[curPoint].Color := clvBlack;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 62] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    // Loads the coordinates
    // With Position fixing for documents with negative coordinates
    case CurToken.GroupCode of
      10: Polyline[curPoint].X := CurToken.FloatValue - DOC_OFFSET.X;
      20: Polyline[curPoint].Y := CurToken.FloatValue - DOC_OFFSET.Y;
      62: Polyline[curPoint].Color := DXFColorIndexToVColor(Trunc(CurToken.FloatValue));
    end;
  end;
end;

{$define FPVECTORIALDEBUG_POLYLINE}
procedure TvDXFVectorialReader.ReadENTITIES_SEQEND(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  i: Integer;
begin
  if not IsReadingPolyline then raise Exception.Create('[TvDXFVectorialReader.ReadENTITIES_SEQEND] Unexpected record: SEQEND before a POLYLINE');

  // Write the Polyline to the document
  if Length(Polyline) >= 0 then // otherwise the polyline is empty of points
  begin
    AData.StartPath(Polyline[0].X, Polyline[0].Y);
    {$ifdef FPVECTORIALDEBUG_POLYLINE}
     Write(Format('POLYLINE %f,%f', [Polyline[0].X, Polyline[0].Y]));
    {$endif}
    for i := 1 to Length(Polyline)-1 do
    begin
      AData.AddLineToPath(Polyline[i].X, Polyline[i].Y, Polyline[i].Color);
      {$ifdef FPVECTORIALDEBUG_POLYLINE}
       Write(Format(' %f,%f', [Polyline[i].X, Polyline[i].Y]));
      {$endif}
    end;
    {$ifdef FPVECTORIALDEBUG_POLYLINE}
     WriteLn('');
    {$endif}
    AData.EndPath();
  end;
end;

procedure TvDXFVectorialReader.ReadENTITIES_MTEXT(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  PosX: Double = 0.0;
  PosY: Double = 0.0;
  PosZ: Double = 0.0;
  FontSize: Double = 10.0;
  Str: string = '';
begin
  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 40] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      1:  Str := CurToken.StrValue;
      10: PosX := CurToken.FloatValue;
      20: PosY := CurToken.FloatValue;
      30: PosZ := CurToken.FloatValue;
      40: FontSize := CurToken.FloatValue;
    end;
  end;

  // Position fixing for documents with negative coordinates
  PosX := PosX - DOC_OFFSET.X;
  PosY := PosY - DOC_OFFSET.Y;

  //
  AData.AddText(PosX, PosY, PosZ, '', Round(FontSize), Str);
end;

procedure TvDXFVectorialReader.ReadENTITIES_POINT(ATokens: TDXFTokens;
  AData: TvVectorialDocument);
var
  CurToken: TDXFToken;
  i: Integer;
  CircleCenterX, CircleCenterY, CircleCenterZ, CircleRadius: Double;
begin
  CircleCenterX := 0.0;
  CircleCenterY := 0.0;
  CircleCenterZ := 0.0;
  CircleRadius := 1.0;

  for i := 0 to ATokens.Count - 1 do
  begin
    // Now read and process the item name
    CurToken := TDXFToken(ATokens.Items[i]);

    // Avoid an exception by previously checking if the conversion can be made
    if CurToken.GroupCode in [10, 20, 30, 40] then
    begin
      CurToken.FloatValue :=  StrToFloat(Trim(CurToken.StrValue), FPointSeparator);
    end;

    case CurToken.GroupCode of
      10: CircleCenterX := CurToken.FloatValue;
      20: CircleCenterY := CurToken.FloatValue;
      30: CircleCenterZ := CurToken.FloatValue;
//      40: CircleRadius := CurToken.FloatValue;
    end;
  end;

  // Position fixing for documents with negative coordinates
  CircleCenterX := CircleCenterX - DOC_OFFSET.X;
  CircleCenterY := CircleCenterY - DOC_OFFSET.Y;

  AData.AddCircle(CircleCenterX, CircleCenterY,
    CircleCenterZ, CircleRadius);
end;

function TvDXFVectorialReader.GetCoordinateValue(AStr: shortstring): Double;
begin
  Result := 0.0;

{  if Length(AStr) <= 1 then Exit;

  Result := StrToFloat(Copy(AStr, 2, Length(AStr) - 1));}
end;

function TvDXFVectorialReader.DXFColorIndexToVColor(AColorIndex: Integer
  ): TvColor;
begin
  if (AColorIndex >= 0) and (AColorIndex <= 15) then
    Result := AUTOCAD_COLOR_PALETTE[AColorIndex]
  else
    raise Exception.Create(Format('[TvDXFVectorialReader.DXFColorIndexToFPVColor] Invalid DXF Color Index: %d', [AColorIndex]));
end;

constructor TvDXFVectorialReader.Create;
begin
  inherited Create;

  FPointSeparator := DefaultFormatSettings;
  FPointSeparator.DecimalSeparator := '.';
  FPointSeparator.ThousandSeparator := '#';// disable the thousand separator

  // Default HEADER data
  ANGBASE := 0.0; // Starts pointing to the right / east
  ANGDIR := 0; // counter-clock wise

  Tokenizer := TDXFTokenizer.Create;
end;

destructor TvDXFVectorialReader.Destroy;
begin
  Tokenizer.Free;

  inherited Destroy;
end;

{@@
  The information of each separate path is lost in G-Code files
  Only one path uniting all of them is created when reading G-Code
}
procedure TvDXFVectorialReader.ReadFromStrings(AStrings: TStrings;
  AData: TvVectorialDocument);
var
  i: Integer;
  CurToken, CurTokenFirstChild: TDXFToken;
begin
  Tokenizer.ReadFromStrings(AStrings);

  for i := 0 to Tokenizer.Tokens.Count - 1 do
  begin
    CurToken := TDXFToken(Tokenizer.Tokens.Items[i]);
    CurTokenFirstChild := TDXFToken(CurToken.Childs.Items[0]);

    if CurTokenFirstChild.StrValue = 'HEADER' then
      ReadHEADER(CurToken.Childs, AData)
    else if CurTokenFirstChild.StrValue = 'ENTITIES' then
      ReadENTITIES(CurToken.Childs, AData);
  end;
end;

initialization

  RegisterVectorialReader(TvDXFVectorialReader, vfDXF);

end.

