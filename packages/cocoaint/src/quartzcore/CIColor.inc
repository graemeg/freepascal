{ Parsed from Quartzcore.framework CIColor.h }

{$ifdef TYPES}
{$ifndef CICOLOR_PAS_T}
{$define CICOLOR_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef CICOLOR_PAS_R}
{$define CICOLOR_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef CICOLOR_PAS_F}
{$define CICOLOR_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef CICOLOR_PAS_S}
{$define CICOLOR_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  CIColor = objcclass;
  CIColorPointer = ^CIColor;
  CIColorPtr = CIColorPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef CICOLOR_PAS_C}
{$define CICOLOR_PAS_C}

{ CIColor }
  CIColor = objcclass external (NSObject, NSCodingProtocol, NSCopyingProtocol)
  private
    _priv: Pointer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _pad: array[0..(3)-1] of Pointer;
=======
        _pad: array[0..2] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _pad: array[0..2] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _pad: array[0..2] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _pad: array[0..2] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class function colorWithCGColor(c: CGColorRef): CIColor; message 'colorWithCGColor:';
    class function colorWithRed_green_blue_alpha(r: CGFloat; g: CGFloat; b: CGFloat; a: CGFloat): CIColor; message 'colorWithRed:green:blue:alpha:';
    class function colorWithRed_green_blue(r: CGFloat; g: CGFloat; b: CGFloat): CIColor; message 'colorWithRed:green:blue:';
    class function colorWithString(representation: NSString): CIColor; message 'colorWithString:';
    function initWithCGColor(c: CGColorRef): id; message 'initWithCGColor:';
    function numberOfComponents: size_t; message 'numberOfComponents';
    function components: CGFloatPtr; message 'components';
    function alpha: CGFloat; message 'alpha';
    function colorSpace: CGColorSpaceRef; message 'colorSpace';
    function red: CGFloat; message 'red';
    function green: CGFloat; message 'green';
    function blue: CGFloat; message 'blue';
    function stringRepresentation: NSString; message 'stringRepresentation';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
    function copyWithZone(zone_: NSZonePtr): id;
  end;

{$endif}
{$endif}
