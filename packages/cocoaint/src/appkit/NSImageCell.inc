{ Parsed from Appkit.framework NSImageCell.h }

{$ifdef TYPES}
{$ifndef NSIMAGECELL_PAS_T}
{$define NSIMAGECELL_PAS_T}

{ Constants }

const
  NSImageAlignCenter = 0;
  NSImageAlignTop = 1;
  NSImageAlignTopLeft = 2;
  NSImageAlignTopRight = 3;
  NSImageAlignLeft = 4;
  NSImageAlignBottom = 5;
  NSImageAlignBottomLeft = 6;
  NSImageAlignBottomRight = 7;
  NSImageAlignRight = 8;

const
  NSImageFrameNone = 0;
  NSImageFramePhoto = 1;
  NSImageFrameGrayBezel = 2;
  NSImageFrameGroove = 3;
  NSImageFrameButton = 4;

{ Types }
type
  NSImageAlignment = NSUInteger;
  NSImageAlignmentPtr = ^NSImageAlignment;
  NSImageFrameStyle = NSUInteger;
  NSImageFrameStylePtr = ^NSImageFrameStyle;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSIMAGECELL_PAS_R}
{$define NSIMAGECELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSIMAGECELL_PAS_F}
{$define NSIMAGECELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSIMAGECELL_PAS_S}
{$define NSIMAGECELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSImageCell = objcclass;
  NSImageCellPointer = ^NSImageCell;
  NSImageCellPtr = NSImageCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSIMAGECELL_PAS_C}
{$define NSIMAGECELL_PAS_C}

{ NSImageCell }
  NSImageCell = objcclass external (NSCell, NSCopyingProtocol, NSCodingProtocol)
  private
    _controlView: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __icFlags: record
      case byte of
       0: (_anonbitfield_NSImageCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __icFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSImageCell0: cuint);
       1: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        _unused: 0..((1 shl 22)-1);
        _animates: 0..1;
        _align: 0..((1 shl 4)-1);
        _scale: 0..((1 shl 2)-1);
        _style: 0..((1 shl 3)-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    __animationState: Pointer;
    _scaledImage: NSImage;
    
  public
    function imageAlignment: NSImageAlignment; message 'imageAlignment';
    procedure setImageAlignment(newAlign: NSImageAlignment); message 'setImageAlignment:';
    function imageScaling: NSImageScaling; message 'imageScaling';
    procedure setImageScaling(newScaling: NSImageScaling); message 'setImageScaling:';
    function imageFrameStyle: NSImageFrameStyle; message 'imageFrameStyle';
    procedure setImageFrameStyle(newStyle: NSImageFrameStyle); message 'setImageFrameStyle:';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
