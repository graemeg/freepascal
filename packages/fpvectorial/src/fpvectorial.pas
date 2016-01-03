{
fpvectorial.pas

Vector graphics document

License: The same modified LGPL as the Free Pascal RTL
         See the file COPYING.modifiedLGPL for more details

AUTHORS: Felipe Monteiro de Carvalho
         Pedro Sol Pegorini L de Lima
}
unit fpvectorial;

{$ifdef fpc}
  {$mode delphi}
{$endif}

interface

uses
  Classes, SysUtils, Math,
  fpcanvas;

type
  TvVectorialFormat = (
    { Multi-purpose document formats }
    vfPDF, vfPostScript, vfSVG, vfCorelDrawCDR, vfWindowsMetafileWMF,
    { CAD formats }
    vfDXF,
    { GCode formats }
    vfGCodeAvisoCNCPrototipoV5, vfGCodeAvisoCNCPrototipoV6);

const
  { Default extensions }
  { Multi-purpose document formats }
  STR_PDF_EXTENSION = '.pdf';
  STR_POSTSCRIPT_EXTENSION = '.ps';
  STR_SVG_EXTENSION = '.svg';
  STR_CORELDRAW_EXTENSION = '.cdr';
  STR_WINMETAFILE_EXTENSION = '.wmf';

type
  {@@ We need our own format because TFPColor is too big for our needs and TColor has no Alpha }
  TvColor = packed record
    Red, Green, Blue, Alpha: Byte;
  end;

const
  FPValphaTransparent = $00;
  FPValphaOpaque = $FF;

  clvBlack: TvColor = (Red: $00; Green: $00; Blue: $00; Alpha: FPValphaOpaque);

type
  T3DPoint = record
    X, Y, Z: Double;
  end;

  P3DPoint = ^T3DPoint;

  TSegmentType = (
    st2DLine, st2DBezier,
    st3DLine, st3DBezier, stMoveTo);

  {@@
    The coordinates in fpvectorial are given in millimiters and
    the starting point is in the bottom-left corner of the document.
    The X grows to the right and the Y grows to the top.
  }
  { TPathSegment }

  TPathSegment = class
  public
    SegmentType: TSegmentType;
    // Fields for linking the list
    Previous: TPathSegment;
    Next: TPathSegment;
    // Data fields
    PenColor: TvColor;
    PenStyle: TFPPenStyle;
    PenWidth: Integer;
  end;

  {@@
    In a 2D segment, the X and Y coordinates represent usually the
    final point of the segment, being that it starts where the previous
    segment ends. The exception is for the first segment of all, which simply
    holds the starting point for the drawing and should always be of the type
    stMoveTo.
  }
  T2DSegment = class(TPathSegment)
  public
    X, Y: Double;
  end;

  {@@
    In Bezier segments, we remain using the X and Y coordinates for the ending point.
    The starting point is where the previous segment ended, so that the intermediary
    bezier control points are [X2, Y2] and [X3, Y3].
  }
  T2DBezierSegment = class(T2DSegment)
  public
    X2, Y2: Double;
    X3, Y3: Double;
  end;

  T3DSegment = class(TPathSegment)
  public
    {@@
      Coordinates of the end of the segment.
      For the first segment, this is the starting point.
    }
    X, Y, Z: Double;
  end;

  T3DBezierSegment = class(T3DSegment)
  public
    X2, Y2, Z2: Double;
    X3, Y3, Z3: Double;
  end;

  TPath = class
    Len: Integer;
    Points: TPathSegment; // Beginning of the double-linked list
    PointsEnd: TPathSegment; // End of the double-linked list
    CurPoint: TPathSegment; // Used in PrepareForSequentialReading and Next
    procedure Assign(APath: TPath);
    function Count(): TPathSegment;
    procedure PrepareForSequentialReading;
    function Next(): TPathSegment;
  end;

  {@@
    TvText represents a text in memory.

    At the moment fonts are unsupported, only simple texts
    up to 255 chars are supported.
  }
  TvText = class
  public
    X, Y, Z: Double; // Z is ignored in 2D formats
    FontSize: integer;
    FontName: utf8string;
    Value: utf8string;
    Color: TvColor;
  end;

  {@@
  }
  TvEntity = class
  public
    // Pen
    PenColor: TvColor;
    PenStyle: TFPPenStyle;
    PenWidth: Integer;
    // Brush
    BrushStyle: TFPBrushStyle;
    BrushColor: TvColor;
  end;

  {@@
  }
  TvCircle = class(TvEntity)
  public
    CenterX, CenterY, CenterZ, Radius: Double;
  end;

  {@@
  }
  TvCircularArc = class(TvEntity)
  public
    CenterX, CenterY, CenterZ, Radius: Double;
    {@@ The Angle is measured in degrees in relation to the positive X axis }
    StartAngle, EndAngle: Double;
  end;

  {@@
  }

  { TvEllipse }

  TvEllipse = class(TvEntity)
  public
    // Mandatory fields
    CenterX, CenterY, CenterZ, MajorHalfAxis, MinorHalfAxis: Double;
    {@@ The Angle is measured in degrees in relation to the positive X axis }
    Angle: Double;
    // Calculated fields
    BoundingRect: TRect;
    procedure CalculateBoundingRectangle;
  end;

  {@@
  }

  { TvAlignedDimension }

  TvAlignedDimension = class(TvEntity)
  public
    // Mandatory fields
    BaseLeft, BaseRight, DimensionLeft, DimensionRight: T3DPoint;
  end;

type

  TvCustomVectorialWriter = class;
  TvCustomVectorialReader = class;

  { TvVectorialDocument }

  TvVectorialDocument = class
  private
    FPaths: TFPList;
    FTexts: TFPList;
    FEntities: TFPList;
    FTmpPath: TPath;
    FTmpText: TvText;
    procedure RemoveCallback(data, arg: pointer);
    function CreateVectorialWriter(AFormat: TvVectorialFormat): TvCustomVectorialWriter;
    function CreateVectorialReader(AFormat: TvVectorialFormat): TvCustomVectorialReader;
    procedure ClearTmpPath();
    procedure AppendSegmentToTmpPath(ASegment: TPathSegment);
  public
    Name: string;
    Width, Height: Double; // in millimeters
    { Base methods }
    constructor Create;
    destructor Destroy; override;
    procedure WriteToFile(AFileName: string; AFormat: TvVectorialFormat);
    procedure WriteToStream(AStream: TStream; AFormat: TvVectorialFormat);
    procedure WriteToStrings(AStrings: TStrings; AFormat: TvVectorialFormat);
    procedure ReadFromFile(AFileName: string; AFormat: TvVectorialFormat);
    procedure ReadFromStream(AStream: TStream; AFormat: TvVectorialFormat);
    procedure ReadFromStrings(AStrings: TStrings; AFormat: TvVectorialFormat);
    class function GetFormatFromExtension(AFileName: string): TvVectorialFormat;
    function  GetDetailedFileFormat(): string;
    { Data reading methods }
    function  GetPath(ANum: Cardinal): TPath;
    function  GetPathCount: Integer;
    function  GetText(ANum: Cardinal): TvText;
    function  GetTextCount: Integer;
    function  GetEntity(ANum: Cardinal): TvEntity;
    function  GetEntityCount: Integer;
    { Data removing methods }
    procedure Clear;
    procedure RemoveAllPaths;
    procedure RemoveAllTexts;
    { Data writing methods }
    procedure AddPath(APath: TPath);
    procedure StartPath(AX, AY: Double);
    procedure AddLineToPath(AX, AY: Double); overload;
    procedure AddLineToPath(AX, AY: Double; AColor: TvColor); overload;
    procedure AddLineToPath(AX, AY, AZ: Double); overload;
    procedure AddBezierToPath(AX1, AY1, AX2, AY2, AX3, AY3: Double); overload;
    procedure AddBezierToPath(AX1, AY1, AZ1, AX2, AY2, AZ2, AX3, AY3, AZ3: Double); overload;
    procedure EndPath();
    procedure AddText(AX, AY, AZ: Double; FontName: string; FontSize: integer; AText: utf8string); overload;
    procedure AddText(AX, AY, AZ: Double; AStr: utf8string); overload;
    procedure AddCircle(ACenterX, ACenterY, ACenterZ, ARadius: Double);
    procedure AddCircularArc(ACenterX, ACenterY, ACenterZ, ARadius, AStartAngle, AEndAngle: Double; AColor: TvColor);
    procedure AddEllipse(CenterX, CenterY, CenterZ, MajorHalfAxis, MinorHalfAxis, Angle: Double);
    // Dimensions
    procedure AddAlignedDimension(BaseLeft, BaseRight, DimLeft, DimRight: T3DPoint);
    { properties }
    property PathCount: Integer read GetPathCount;
    property Paths[Index: Cardinal]: TPath read GetPath;
  end;

  {@@ TvVectorialReader class reference type }

  TvVectorialReaderClass = class of TvCustomVectorialReader;

  { TvCustomVectorialReader }

  TvCustomVectorialReader = class
  public
    { General reading methods }
    constructor Create; virtual;
    procedure ReadFromFile(AFileName: string; AData: TvVectorialDocument); virtual;
    procedure ReadFromStream(AStream: TStream; AData: TvVectorialDocument); virtual;
    procedure ReadFromStrings(AStrings: TStrings; AData: TvVectorialDocument); virtual;
  end;

  {@@ TvVectorialWriter class reference type }

  TvVectorialWriterClass = class of TvCustomVectorialWriter;

  {@@ TvCustomVectorialWriter }

  { TvCustomVectorialWriter }

  TvCustomVectorialWriter = class
  public
    { General writing methods }
    constructor Create; virtual;
    procedure WriteToFile(AFileName: string; AData: TvVectorialDocument); virtual;
    procedure WriteToStream(AStream: TStream; AData: TvVectorialDocument); virtual;
    procedure WriteToStrings(AStrings: TStrings; AData: TvVectorialDocument); virtual;
  end;

  {@@ List of registered formats }

  TvVectorialFormatData = record
    ReaderClass: TvVectorialReaderClass;
    WriterClass: TvVectorialWriterClass;
    ReaderRegistered: Boolean;
    WriterRegistered: Boolean;
    Format: TvVectorialFormat;
  end;

var
  GvVectorialFormats: array of TvVectorialFormatData;

procedure RegisterVectorialReader(
  AReaderClass: TvVectorialReaderClass;
  AFormat: TvVectorialFormat);
procedure RegisterVectorialWriter(
  AWriterClass: TvVectorialWriterClass;
  AFormat: TvVectorialFormat);

implementation

const
  Str_Error_Nil_Path = ' The program attempted to add a segment before creating a path';

{@@
  Registers a new reader for a format
}
procedure RegisterVectorialReader(
  AReaderClass: TvVectorialReaderClass;
  AFormat: TvVectorialFormat);
var
  i, len: Integer;
  FormatInTheList: Boolean;
begin
  len := Length(GvVectorialFormats);
  FormatInTheList := False;

  { First search for the format in the list }
  for i := 0 to len - 1 do
  begin
    if GvVectorialFormats[i].Format = AFormat then
    begin
      if GvVectorialFormats[i].ReaderRegistered then
       raise Exception.Create('RegisterVectorialReader: Reader class for format ' {+ AFormat} + ' already registered.');

      GvVectorialFormats[i].ReaderRegistered := True;
      GvVectorialFormats[i].ReaderClass := AReaderClass;

      FormatInTheList := True;
      Break;
    end;
  end;

  { If not already in the list, then add it }
  if not FormatInTheList then
  begin
    SetLength(GvVectorialFormats, len + 1);

    GvVectorialFormats[len].ReaderClass := AReaderClass;
    GvVectorialFormats[len].WriterClass := nil;
    GvVectorialFormats[len].ReaderRegistered := True;
    GvVectorialFormats[len].WriterRegistered := False;
    GvVectorialFormats[len].Format := AFormat;
  end;
end;

{@@
  Registers a new writer for a format
}
procedure RegisterVectorialWriter(
  AWriterClass: TvVectorialWriterClass;
  AFormat: TvVectorialFormat);
var
  i, len: Integer;
  FormatInTheList: Boolean;
begin
  len := Length(GvVectorialFormats);
  FormatInTheList := False;

  { First search for the format in the list }
  for i := 0 to len - 1 do
  begin
    if GvVectorialFormats[i].Format = AFormat then
    begin
      if GvVectorialFormats[i].WriterRegistered then
       raise Exception.Create('RegisterVectorialWriter: Writer class for format ' + {AFormat +} ' already registered.');

      GvVectorialFormats[i].WriterRegistered := True;
      GvVectorialFormats[i].WriterClass := AWriterClass;

      FormatInTheList := True;
      Break;
    end;
  end;

  { If not already in the list, then add it }
  if not FormatInTheList then
  begin
    SetLength(GvVectorialFormats, len + 1);

    GvVectorialFormats[len].ReaderClass := nil;
    GvVectorialFormats[len].WriterClass := AWriterClass;
    GvVectorialFormats[len].ReaderRegistered := False;
    GvVectorialFormats[len].WriterRegistered := True;
    GvVectorialFormats[len].Format := AFormat;
  end;
end;

{ TvEllipse }

procedure TvEllipse.CalculateBoundingRectangle;
var
  t, tmp: Double;
begin
  {
    To calculate the bounding rectangle we can do this:

    Ellipse equations:You could try using the parametrized equations for an ellipse rotated at an arbitrary angle:

    x = CenterX + MajorHalfAxis*cos(t)*cos(Angle) - MinorHalfAxis*sin(t)*sin(Angle)
    y = CenterY + MinorHalfAxis*sin(t)*cos(Angle) + MajorHalfAxis*cos(t)*sin(Angle)

    You can then differentiate and solve for gradient = 0:
    0 = dx/dt = -MajorHalfAxis*sin(t)*cos(Angle) - MinorHalfAxis*cos(t)*sin(Angle)
    =>
    tan(t) = -MinorHalfAxis*tan(Angle)/MajorHalfAxis
    =>
    t = cotang(-MinorHalfAxis*tan(Angle)/MajorHalfAxis)

    On the other axis:

    0 = dy/dt = b*cos(t)*cos(phi) - a*sin(t)*sin(phi)
    =>
    tan(t) = b*cot(phi)/a
  }
  t := cotan(-MinorHalfAxis*tan(Angle)/MajorHalfAxis);
  tmp := CenterX + MajorHalfAxis*cos(t)*cos(Angle) - MinorHalfAxis*sin(t)*sin(Angle);
  BoundingRect.Right := Round(tmp);
end;

{ TsWorksheet }

{@@
  Helper method for clearing the records in a spreadsheet.
}
procedure TvVectorialDocument.RemoveCallback(data, arg: pointer);
begin
{  if data <> nil then
  begin
    ldata := PObject(data);
    ldata^.Free;
  end;}
end;

{@@
  Constructor.
}
constructor TvVectorialDocument.Create;
begin
  inherited Create;

  FPaths := TFPList.Create;
  FTexts := TFPList.Create;
  FEntities := TFPList.Create;
  FTmpPath := TPath.Create;
end;

{@@
  Destructor.
}
destructor TvVectorialDocument.Destroy;
begin
  Clear;

  FPaths.Free;
  FTexts.Free;
  FEntities.Free;

  inherited Destroy;
end;

{@@
  Clears the list of Vectors and releases their memory.
}
procedure TvVectorialDocument.RemoveAllPaths;
begin
//  FPaths.ForEachCall(RemoveCallback, nil);
  FPaths.Clear;
end;

procedure TvVectorialDocument.RemoveAllTexts;
begin
//  FTexts.ForEachCall(RemoveCallback, nil);
  FTexts.Clear;
end;

procedure TvVectorialDocument.AddPath(APath: TPath);
var
  lPath: TPath;
  Len: Integer;
begin
  lPath := TPath.Create;
  lPath.Assign(APath);
  FPaths.Add(Pointer(lPath));
  //WriteLn(':>TvVectorialDocument.AddPath 1 Len = ', Len);
  //WriteLn(':>TvVectorialDocument.AddPath 2');
  //WriteLn(':>TvVectorialDocument.AddPath 3');
  //WriteLn(':>TvVectorialDocument.AddPath 4');
end;

{@@
  Starts writing a Path in multiple steps.
  Should be followed by zero or more calls to AddPointToPath
  and by a call to EndPath to effectively add the data.

  @see    StartPath, AddPointToPath
}
procedure TvVectorialDocument.StartPath(AX, AY: Double);
var
  segment: T2DSegment;
begin
  ClearTmpPath();

  FTmpPath.Len := 1;
  segment := T2DSegment.Create;
  segment.SegmentType := stMoveTo;
  segment.X := AX;
  segment.Y := AY;

  FTmpPath.Points := segment;
  FTmpPath.PointsEnd := segment;
end;

{@@
  Adds one more point to the end of a Path being
  writing in multiple steps.

  Does nothing if not called between StartPath and EndPath.

  Can be called multiple times to add multiple points.

  @see    StartPath, EndPath
}
procedure TvVectorialDocument.AddLineToPath(AX, AY: Double);
var
  segment: T2DSegment;
begin
  segment := T2DSegment.Create;
  segment.SegmentType := st2DLine;
  segment.X := AX;
  segment.Y := AY;
  segment.PenColor := clvBlack;

  AppendSegmentToTmpPath(segment);
end;

procedure TvVectorialDocument.AddLineToPath(AX, AY: Double; AColor: TvColor);
var
  segment: T2DSegment;
begin
  segment := T2DSegment.Create;
  segment.SegmentType := st2DLine;
  segment.X := AX;
  segment.Y := AY;
  segment.PenColor := AColor;

  AppendSegmentToTmpPath(segment);
end;

procedure TvVectorialDocument.AddLineToPath(AX, AY, AZ: Double);
var
  segment: T3DSegment;
begin
  segment := T3DSegment.Create;
  segment.SegmentType := st3DLine;
  segment.X := AX;
  segment.Y := AY;
  segment.Z := AZ;

  AppendSegmentToTmpPath(segment);
end;

{@@
  Adds a bezier element to the path. It starts where the previous element ended
  and it goes throw the control points [AX1, AY1] and [AX2, AY2] and ends
  in [AX3, AY3].
}
procedure TvVectorialDocument.AddBezierToPath(AX1, AY1, AX2, AY2, AX3,
  AY3: Double);
var
  segment: T2DBezierSegment;
begin
  segment := T2DBezierSegment.Create;
  segment.SegmentType := st2DBezier;
  segment.X := AX3;
  segment.Y := AY3;
  segment.X2 := AX1;
  segment.Y2 := AY1;
  segment.X3 := AX2;
  segment.Y3 := AY2;

  AppendSegmentToTmpPath(segment);
end;

procedure TvVectorialDocument.AddBezierToPath(AX1, AY1, AZ1, AX2, AY2, AZ2,
  AX3, AY3, AZ3: Double);
var
  segment: T3DBezierSegment;
begin
  segment := T3DBezierSegment.Create;
  segment.SegmentType := st3DBezier;
  segment.X := AX3;
  segment.Y := AY3;
  segment.Z := AZ3;
  segment.X2 := AX1;
  segment.Y2 := AY1;
  segment.Z2 := AZ1;
  segment.X3 := AX2;
  segment.Y3 := AY2;
  segment.Z3 := AZ2;

  AppendSegmentToTmpPath(segment);
end;

{@@
  Finishes writing a Path, which was created in multiple
  steps using StartPath and AddPointToPath,
  to the document.

  Does nothing if there wasn't a previous correspondent call to
  StartPath.

  @see    StartPath, AddPointToPath
}
procedure TvVectorialDocument.EndPath();
begin
  if FTmPPath.Len = 0 then Exit;
  AddPath(FTmPPath);
  ClearTmpPath();
end;

procedure TvVectorialDocument.AddText(AX, AY, AZ: Double; FontName: string; FontSize: integer; AText: utf8string);
var
  lText: TvText;
begin
  lText := TvText.Create;
  lText.Value := AText;
  lText.X := AX;
  lText.Y := AY;
  lText.Z := AZ;
  lText.FontName := FontName;
  lText.FontSize := FontSize;
  FTexts.Add(lText);
end;

procedure TvVectorialDocument.AddText(AX, AY, AZ: Double; AStr: utf8string);
begin
  AddText(AX, AY, AZ, '', 10, AStr);
end;

procedure TvVectorialDocument.AddCircle(ACenterX, ACenterY, ACenterZ, ARadius: Double);
var
  lCircle: TvCircle;
begin
  lCircle := TvCircle.Create;
  lCircle.CenterX := ACenterX;
  lCircle.CenterY := ACenterY;
  lCircle.CenterZ := ACenterZ;
  lCircle.Radius := ARadius;
  FEntities.Add(lCircle);
end;

procedure TvVectorialDocument.AddCircularArc(ACenterX, ACenterY, ACenterZ,
  ARadius, AStartAngle, AEndAngle: Double; AColor: TvColor);
var
  lCircularArc: TvCircularArc;
begin
  lCircularArc := TvCircularArc.Create;
  lCircularArc.CenterX := ACenterX;
  lCircularArc.CenterY := ACenterY;
  lCircularArc.CenterZ := ACenterZ;
  lCircularArc.Radius := ARadius;
  lCircularArc.StartAngle := AStartAngle;
  lCircularArc.EndAngle := AEndAngle;
  lCircularArc.PenColor := AColor;
  FEntities.Add(lCircularArc);
end;

procedure TvVectorialDocument.AddEllipse(CenterX, CenterY, CenterZ,
  MajorHalfAxis, MinorHalfAxis, Angle: Double);
var
  lEllipse: TvEllipse;
begin
  lEllipse := TvEllipse.Create;
  lEllipse.CenterX := CenterX;
  lEllipse.CenterY := CenterY;
  lEllipse.CenterZ := CenterZ;
  lEllipse.MajorHalfAxis := MajorHalfAxis;
  lEllipse.MinorHalfAxis := MinorHalfAxis;
  lEllipse.Angle := Angle;
  FEntities.Add(lEllipse);
end;

procedure TvVectorialDocument.AddAlignedDimension(BaseLeft, BaseRight,
  DimLeft, DimRight: T3DPoint);
var
  lDim: TvAlignedDimension;
begin
  lDim := TvAlignedDimension.Create;
  lDim.BaseLeft := BaseLeft;
  lDim.BaseRight := BaseRight;
  lDim.DimensionLeft := DimLeft;
  lDim.DimensionRight := DimRight;
  FEntities.Add(lDim);
end;

{@@
  Convenience method which creates the correct
  writer object for a given vector graphics document format.
}
function TvVectorialDocument.CreateVectorialWriter(AFormat: TvVectorialFormat): TvCustomVectorialWriter;
var
  i: Integer;
begin
  Result := nil;

  for i := 0 to Length(GvVectorialFormats) - 1 do
    if GvVectorialFormats[i].Format = AFormat then
    begin
      Result := GvVectorialFormats[i].WriterClass.Create;

      Break;
    end;

  if Result = nil then raise Exception.Create('Unsuported vector graphics format.');
end;

{@@
  Convenience method which creates the correct
  reader object for a given vector graphics document format.
}
function TvVectorialDocument.CreateVectorialReader(AFormat: TvVectorialFormat): TvCustomVectorialReader;
var
  i: Integer;
begin
  Result := nil;

  for i := 0 to Length(GvVectorialFormats) - 1 do
    if GvVectorialFormats[i].Format = AFormat then
    begin
      Result := GvVectorialFormats[i].ReaderClass.Create;

      Break;
    end;

  if Result = nil then raise Exception.Create('Unsuported vector graphics format.');
end;

procedure TvVectorialDocument.ClearTmpPath();
var
  segment, oldsegment: TPathSegment;
begin
//  segment := FTmpPath.Points;
// Don't free segments, because they are used when the path is added
//  while segment <> nil do
//  begin
//    oldsegment := segment;
//    segment := segment^.Next;
//    oldsegment^.Free;
//  end;

  FTmpPath.Points := nil;
  FTmpPath.PointsEnd := nil;
  FTmpPath.Len := 0;
end;

procedure TvVectorialDocument.AppendSegmentToTmpPath(ASegment: TPathSegment);
var
  L: Integer;
begin
  if FTmpPath.PointsEnd = nil then
    Exception.Create('[TvVectorialDocument.AppendSegmentToTmpPath]' + Str_Error_Nil_Path);

  L := FTmpPath.Len;
  Inc(FTmpPath.Len);

  // Adds the element to the end of the list
  FTmpPath.PointsEnd.Next := ASegment;
  ASegment.Previous := FTmpPath.PointsEnd;
  FTmpPath.PointsEnd := ASegment;
end;

{@@
  Writes the document to a file.

  If the file doesn't exist, it will be created.
}
procedure TvVectorialDocument.WriteToFile(AFileName: string; AFormat: TvVectorialFormat);
var
  AWriter: TvCustomVectorialWriter;
begin
  AWriter := CreateVectorialWriter(AFormat);

  try
    AWriter.WriteToFile(AFileName, Self);
  finally
    AWriter.Free;
  end;
end;

{@@
  Writes the document to a stream
}
procedure TvVectorialDocument.WriteToStream(AStream: TStream; AFormat: TvVectorialFormat);
var
  AWriter: TvCustomVectorialWriter;
begin
  AWriter := CreateVectorialWriter(AFormat);

  try
    AWriter.WriteToStream(AStream, Self);
  finally
    AWriter.Free;
  end;
end;

procedure TvVectorialDocument.WriteToStrings(AStrings: TStrings;
  AFormat: TvVectorialFormat);
var
  AWriter: TvCustomVectorialWriter;
begin
  AWriter := CreateVectorialWriter(AFormat);

  try
    AWriter.WriteToStrings(AStrings, Self);
  finally
    AWriter.Free;
  end;
end;

{@@
  Reads the document from a file.

  Any current contents will be removed.
}
procedure TvVectorialDocument.ReadFromFile(AFileName: string;
  AFormat: TvVectorialFormat);
var
  AReader: TvCustomVectorialReader;
begin
  Self.Clear;

  AReader := CreateVectorialReader(AFormat);
  try
    AReader.ReadFromFile(AFileName, Self);
  finally
    AReader.Free;
  end;
end;

{@@
  Reads the document from a stream.

  Any current contents will be removed.
}
procedure TvVectorialDocument.ReadFromStream(AStream: TStream;
  AFormat: TvVectorialFormat);
var
  AReader: TvCustomVectorialReader;
begin
  Self.Clear;

  AReader := CreateVectorialReader(AFormat);
  try
    AReader.ReadFromStream(AStream, Self);
  finally
    AReader.Free;
  end;
end;

procedure TvVectorialDocument.ReadFromStrings(AStrings: TStrings;
  AFormat: TvVectorialFormat);
var
  AReader: TvCustomVectorialReader;
begin
  Self.Clear;

  AReader := CreateVectorialReader(AFormat);
  try
    AReader.ReadFromStrings(AStrings, Self);
  finally
    AReader.Free;
  end;
end;

class function TvVectorialDocument.GetFormatFromExtension(AFileName: string
  ): TvVectorialFormat;
var
  lExt: string;
begin
  lExt := ExtractFileExt(AFileName);
  if AnsiCompareText(lExt, STR_PDF_EXTENSION) = 0 then Result := vfPDF
  else if AnsiCompareText(lExt, STR_POSTSCRIPT_EXTENSION) = 0 then Result := vfPostScript
  else if AnsiCompareText(lExt, STR_SVG_EXTENSION) = 0 then Result := vfSVG
  else if AnsiCompareText(lExt, STR_CORELDRAW_EXTENSION) = 0 then Result := vfCorelDrawCDR
  else if AnsiCompareText(lExt, STR_WINMETAFILE_EXTENSION) = 0 then Result := vfWindowsMetafileWMF
  else
    raise Exception.Create('TvVectorialDocument.GetFormatFromExtension: The extension (' + lExt + ') doesn''t match any supported formats.');
end;

function  TvVectorialDocument.GetDetailedFileFormat(): string;
begin

end;

function TvVectorialDocument.GetPath(ANum: Cardinal): TPath;
begin
  if ANum >= FPaths.Count then raise Exception.Create('TvVectorialDocument.GetPath: Path number out of bounds');

  if FPaths.Items[ANum] = nil then raise Exception.Create('TvVectorialDocument.GetPath: Invalid Path number');

  Result := TPath(FPaths.Items[ANum]);
end;

function TvVectorialDocument.GetPathCount: Integer;
begin
  Result := FPaths.Count;
end;

function TvVectorialDocument.GetText(ANum: Cardinal): TvText;
begin
  if ANum >= FTexts.Count then raise Exception.Create('TvVectorialDocument.GetText: Text number out of bounds');

  if FTexts.Items[ANum] = nil then raise Exception.Create('TvVectorialDocument.GetText: Invalid Text number');

  Result := TvText(FTexts.Items[ANum]);
end;

function TvVectorialDocument.GetTextCount: Integer;
begin
  Result := FTexts.Count;
end;

function TvVectorialDocument.GetEntity(ANum: Cardinal): TvEntity;
begin
  if ANum >= FEntities.Count then raise Exception.Create('TvVectorialDocument.GetEntity: Entity number out of bounds');

  if FEntities.Items[ANum] = nil then raise Exception.Create('TvVectorialDocument.GetEntity: Invalid Entity number');

  Result := TvEntity(FEntities.Items[ANum]);
end;

function TvVectorialDocument.GetEntityCount: Integer;
begin
  Result := FEntities.Count;
end;

{@@
  Clears all data in the document
}
procedure TvVectorialDocument.Clear;
begin
  RemoveAllPaths();
  RemoveAllTexts();
end;

{ TvCustomVectorialReader }

constructor TvCustomVectorialReader.Create;
begin
  inherited Create;
end;

procedure TvCustomVectorialReader.ReadFromFile(AFileName: string; AData: TvVectorialDocument);
var
  FileStream: TFileStream;
begin
  FileStream := TFileStream.Create(AFileName, fmOpenRead or fmShareDenyNone);
  try
    ReadFromStream(FileStream, AData);
  finally
    FileStream.Free;
  end;
end;

procedure TvCustomVectorialReader.ReadFromStream(AStream: TStream;
  AData: TvVectorialDocument);
var
  AStringStream: TStringStream;
  AStrings: TStringList;
begin
  AStringStream := TStringStream.Create('');
  AStrings := TStringList.Create;
  try
    AStringStream.CopyFrom(AStream, AStream.Size);
    AStringStream.Seek(0, soFromBeginning);
    AStrings.Text := AStringStream.DataString;
    ReadFromStrings(AStrings, AData);
  finally
    AStringStream.Free;
    AStrings.Free;
  end;
end;

procedure TvCustomVectorialReader.ReadFromStrings(AStrings: TStrings;
  AData: TvVectorialDocument);
var
  AStringStream: TStringStream;
begin
  AStringStream := TStringStream.Create('');
  try
    AStringStream.WriteString(AStrings.Text);
    AStringStream.Seek(0, soFromBeginning);
    ReadFromStream(AStringStream, AData);
  finally
    AStringStream.Free;
  end;
end;

{ TsCustomSpreadWriter }

constructor TvCustomVectorialWriter.Create;
begin
  inherited Create;
end;

{@@
  Default file writting method.

  Opens the file and calls WriteToStream

  @param  AFileName The output file name.
                   If the file already exists it will be replaced.
  @param  AData     The Workbook to be saved.

  @see    TsWorkbook
}
procedure TvCustomVectorialWriter.WriteToFile(AFileName: string; AData: TvVectorialDocument);
var
  OutputFile: TFileStream;
begin
  OutputFile := TFileStream.Create(AFileName, fmCreate or fmOpenWrite);
  try
    WriteToStream(OutputFile, AData);
  finally
    OutputFile.Free;
  end;
end;

{@@
  The default stream writer just uses WriteToStrings
}
procedure TvCustomVectorialWriter.WriteToStream(AStream: TStream;
  AData: TvVectorialDocument);
var
  lStringList: TStringList;
begin
  lStringList := TStringList.Create;
  try
    WriteToStrings(lStringList, AData);
    lStringList.SaveToStream(AStream);
  finally
    lStringList.Free;
  end;
end;

procedure TvCustomVectorialWriter.WriteToStrings(AStrings: TStrings;
  AData: TvVectorialDocument);
begin

end;

{ TPath }

procedure TPath.Assign(APath: TPath);
begin
  Len := APath.Len;
  Points := APath.Points;
  PointsEnd := APath.PointsEnd;
  CurPoint := APath.CurPoint;
end;

function TPath.Count(): TPathSegment;
begin

end;

procedure TPath.PrepareForSequentialReading;
begin
  CurPoint := nil;
end;

function TPath.Next(): TPathSegment;
begin
  if CurPoint = nil then Result := Points
  else Result := CurPoint.Next;

  CurPoint := Result;
end;

finalization

  SetLength(GvVectorialFormats, 0);

end.

