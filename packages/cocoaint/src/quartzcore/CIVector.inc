{ Parsed from Quartzcore.framework CIVector.h }

{$ifdef TYPES}
{$ifndef CIVECTOR_PAS_T}
{$define CIVECTOR_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef CIVECTOR_PAS_R}
{$define CIVECTOR_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef CIVECTOR_PAS_F}
{$define CIVECTOR_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef CIVECTOR_PAS_S}
{$define CIVECTOR_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  CIVector = objcclass;
  CIVectorPointer = ^CIVector;
  CIVectorPtr = CIVectorPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef CIVECTOR_PAS_C}
{$define CIVECTOR_PAS_C}

{ CIVector }
  CIVector = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _count: size_t;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    vec: array[0..(4)-1] of CGFloat;
=======
        vec: array[0..3] of CGFloat;
>>>>>>> graemeg/cpstrnew
=======
        vec: array[0..3] of CGFloat;
>>>>>>> graemeg/cpstrnew
=======
        vec: array[0..3] of CGFloat;
>>>>>>> graemeg/cpstrnew
=======
        vec: array[0..3] of CGFloat;
>>>>>>> origin/cpstrnew
    ptr: CGFloatPtr;
    __u: record
      end;
    
  public
    class function vectorWithValues_count(values: CGFloatPtr; count_: size_t): CIVector; message 'vectorWithValues:count:';
    class function vectorWithX(x_: CGFloat): CIVector; message 'vectorWithX:';
    class function vectorWithX_Y(x_: CGFloat; y_: CGFloat): CIVector; message 'vectorWithX:Y:';
    class function vectorWithX_Y_Z(x_: CGFloat; y_: CGFloat; z_: CGFloat): CIVector; message 'vectorWithX:Y:Z:';
    class function vectorWithX_Y_Z_W(x_: CGFloat; y_: CGFloat; z_: CGFloat; w_: CGFloat): CIVector; message 'vectorWithX:Y:Z:W:';
    class function vectorWithString(representation: NSString): CIVector; message 'vectorWithString:';
    function initWithValues_count(values: CGFloatPtr; count_: size_t): id; message 'initWithValues:count:';
    function initWithX(x_: CGFloat): id; message 'initWithX:';
    function initWithX_Y(x_: CGFloat; y_: CGFloat): id; message 'initWithX:Y:';
    function initWithX_Y_Z(x_: CGFloat; y_: CGFloat; z_: CGFloat): id; message 'initWithX:Y:Z:';
    function initWithX_Y_Z_W(x_: CGFloat; y_: CGFloat; z_: CGFloat; w_: CGFloat): id; message 'initWithX:Y:Z:W:';
    function initWithString(representation: NSString): id; message 'initWithString:';
    function valueAtIndex(index: size_t): CGFloat; message 'valueAtIndex:';
    function count: size_t; message 'count';
    function X: CGFloat; message 'X';
    function Y: CGFloat; message 'Y';
    function Z: CGFloat; message 'Z';
    function W: CGFloat; message 'W';
    function stringRepresentation: NSString; message 'stringRepresentation';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
