{ Parsed from Appkit.framework NSFont.h }

{$ifdef TYPES}
{$ifndef NSFONT_PAS_T}
{$define NSFONT_PAS_T}

{ Types }
type
  NSGlyph = cuint;
  NSGlyphPtr = ^NSGlyph;
  NSMultibyteGlyphPacking = NSUInteger;
  NSMultibyteGlyphPackingPtr = ^NSMultibyteGlyphPacking;
  NSFontRenderingMode = NSUInteger;
  NSFontRenderingModePtr = ^NSFontRenderingMode;
  NSGlyphRelation = NSUInteger;
  NSGlyphRelationPtr = ^NSGlyphRelation;

{ Constants }

const
  NSControlGlyph = $00FFFFFF;
  NSNullGlyph = $0;

const
  NSNativeShortGlyphPacking = 5;

const
  NSFontDefaultRenderingMode = 0;
  NSFontAntialiasedRenderingMode = 1;
  NSFontIntegerAdvancementsRenderingMode = 2;
  NSFontAntialiasedIntegerAdvancementsRenderingMode = 3;

const
  NSOneByteGlyphPacking = 0;
  NSJapaneseEUCGlyphPacking = 1;
  NSAsciiWithDoubleByteEUCGlyphPacking = 2;
  NSTwoByteGlyphPacking = 3;
  NSFourByteGlyphPacking = 4;

const
  NSGlyphBelow = 1;
  NSGlyphAbove = 2;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSFONT_PAS_R}
{$define NSFONT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSFONT_PAS_F}
{$define NSFONT_PAS_F}

{ Functions }
function NSConvertGlyphsToPackedGlyphs(glBuf: NSGlyphPtr; count: NSInteger; packing: NSMultibyteGlyphPacking; packedGlyphs: PChar): NSInteger; cdecl; external;

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSFONT_PAS_S}
{$define NSFONT_PAS_S}

{ External string constants }
var
  NSAntialiasThresholdChangedNotification: NSString; cvar; external;
  NSFontSetChangedNotification: NSString; cvar; external;
  NSAFMFamilyName: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMFontName: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMFormatVersion: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMFullName: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMNotice: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMVersion: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMWeight: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMEncodingScheme: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMCharacterSet: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMCapHeight: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMXHeight: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMAscender: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMDescender: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMUnderlinePosition: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMUnderlineThickness: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMItalicAngle: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;
  NSAFMMappingScheme: NSString deprecated 'in Mac OS X 10.4 and later'; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSFont = objcclass;
  NSFontPointer = ^NSFont;
  NSFontPtr = NSFontPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSFONT_PAS_C}
{$define NSFONT_PAS_C}

{ NSFont }
  NSFont = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _name: NSString;
    _size: CGFloat;
    _retainCount: NSInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __fFlags: record
      case byte of
       0: (_anonbitfield_NSFont0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __fFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSFont0: cuint);
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
        _isScreenFont: 0..1;
        _systemFontType: 0..((1 shl 8)-1);
        _reserved1: 0..((1 shl 4)-1);
        _matrixIsIdentity: 0..1;
        _renderingMode: 0..((1 shl 3)-1);
        _reserved2: 0..((1 shl 15)-1);
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
    _private: id;
    
  public
    class function fontWithName_size(fontName_: NSString; fontSize: CGFloat): NSFont; message 'fontWithName:size:';
    class function fontWithName_matrix(fontName_: NSString; fontMatrix: CGFloatPtr): NSFont; message 'fontWithName:matrix:';
    class function fontWithDescriptor_size(fontDescriptor_: NSFontDescriptor; fontSize: CGFloat): NSFont; message 'fontWithDescriptor:size:';
    class function fontWithDescriptor_textTransform(fontDescriptor_: NSFontDescriptor; textTransform_: NSAffineTransform): NSFont; message 'fontWithDescriptor:textTransform:';
    class function userFontOfSize(fontSize: CGFloat): NSFont; message 'userFontOfSize:';
    class function userFixedPitchFontOfSize(fontSize: CGFloat): NSFont; message 'userFixedPitchFontOfSize:';
    class procedure setUserFont(aFont: NSFont); message 'setUserFont:';
    class procedure setUserFixedPitchFont(aFont: NSFont); message 'setUserFixedPitchFont:';
    class function systemFontOfSize(fontSize: CGFloat): NSFont; message 'systemFontOfSize:';
    class function boldSystemFontOfSize(fontSize: CGFloat): NSFont; message 'boldSystemFontOfSize:';
    class function labelFontOfSize(fontSize: CGFloat): NSFont; message 'labelFontOfSize:';
    class function titleBarFontOfSize(fontSize: CGFloat): NSFont; message 'titleBarFontOfSize:';
    class function menuFontOfSize(fontSize: CGFloat): NSFont; message 'menuFontOfSize:';
    class function menuBarFontOfSize(fontSize: CGFloat): NSFont; message 'menuBarFontOfSize:';
    class function messageFontOfSize(fontSize: CGFloat): NSFont; message 'messageFontOfSize:';
    class function paletteFontOfSize(fontSize: CGFloat): NSFont; message 'paletteFontOfSize:';
    class function toolTipsFontOfSize(fontSize: CGFloat): NSFont; message 'toolTipsFontOfSize:';
    class function controlContentFontOfSize(fontSize: CGFloat): NSFont; message 'controlContentFontOfSize:';
    class function systemFontSize: CGFloat; message 'systemFontSize';
    class function smallSystemFontSize: CGFloat; message 'smallSystemFontSize';
    class function labelFontSize: CGFloat; message 'labelFontSize';
    class function systemFontSizeForControlSize(controlSize: NSControlSize): CGFloat; message 'systemFontSizeForControlSize:';
    function fontName: NSString; message 'fontName';
    function pointSize: CGFloat; message 'pointSize';
    function matrix: CGFloatPtr; message 'matrix';
    function familyName: NSString; message 'familyName';
    function displayName: NSString; message 'displayName';
    function fontDescriptor: NSFontDescriptor; message 'fontDescriptor';
    function textTransform: NSAffineTransform; message 'textTransform';
    function numberOfGlyphs: NSUInteger; message 'numberOfGlyphs';
    function mostCompatibleStringEncoding: NSStringEncoding; message 'mostCompatibleStringEncoding';
    function glyphWithName(aName: NSString): NSGlyph; message 'glyphWithName:';
    function coveredCharacterSet: NSCharacterSet; message 'coveredCharacterSet';
    function boundingRectForFont: NSRect; message 'boundingRectForFont';
    function maximumAdvancement: NSSize; message 'maximumAdvancement';
    function ascender: CGFloat; message 'ascender';
    function descender: CGFloat; message 'descender';
    function leading: CGFloat; message 'leading';
    function underlinePosition: CGFloat; message 'underlinePosition';
    function underlineThickness: CGFloat; message 'underlineThickness';
    function italicAngle: CGFloat; message 'italicAngle';
    function capHeight: CGFloat; message 'capHeight';
    function xHeight: CGFloat; message 'xHeight';
    function isFixedPitch: Boolean; message 'isFixedPitch';
    function boundingRectForGlyph(aGlyph: NSGlyph): NSRect; message 'boundingRectForGlyph:';
    function advancementForGlyph(ag: NSGlyph): NSSize; message 'advancementForGlyph:';
    procedure getBoundingRects_forGlyphs_count(bounds: NSRectArray; glyphs: NSGlyphPtr; glyphCount: NSUInteger); message 'getBoundingRects:forGlyphs:count:';
    procedure getAdvancements_forGlyphs_count(advancements: NSSizeArray; glyphs: NSGlyphPtr; glyphCount: NSUInteger); message 'getAdvancements:forGlyphs:count:';
    procedure getAdvancements_forPackedGlyphs_length(advancements: NSSizeArray; packedGlyphs: Pointer; length: NSUInteger); message 'getAdvancements:forPackedGlyphs:length:';
    procedure set_; message 'set';
    procedure setInContext(graphicsContext: NSGraphicsContext); message 'setInContext:';
    function printerFont: NSFont; message 'printerFont';
    function screenFont: NSFont; message 'screenFont';
    function screenFontWithRenderingMode(renderingMode_: NSFontRenderingMode): NSFont; message 'screenFontWithRenderingMode:';
    function renderingMode: NSFontRenderingMode; message 'renderingMode';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSFontDeprecatedCategory }
  NSFontDeprecatedCategory = objccategory external (NSFont)
    class procedure useFont(fontName_: NSString); message 'useFont:'; deprecated 'in Mac OS X 10.4 and later';
    function widthOfString(string_: NSString): CGFloat; message 'widthOfString:'; deprecated 'in Mac OS X 10.4 and later';
    function isBaseFont: Boolean; message 'isBaseFont'; deprecated 'in Mac OS X 10.4 and later';
    function afmDictionary: NSDictionary; message 'afmDictionary'; deprecated 'in Mac OS X 10.4 and later';
    function glyphIsEncoded(aGlyph: NSGlyph): Boolean; message 'glyphIsEncoded:'; deprecated 'in Mac OS X 10.4 and later';
    function defaultLineHeightForFont: CGFloat; message 'defaultLineHeightForFont'; deprecated 'in Mac OS X 10.4 and later';
    class function preferredFontNames: NSArray; message 'preferredFontNames'; deprecated 'in Mac OS X 10.4 and later';
    class procedure setPreferredFontNames(fontNameArray: NSArray); message 'setPreferredFontNames:'; deprecated 'in Mac OS X 10.4 and later';
    function encodingScheme: NSString; message 'encodingScheme'; deprecated 'in Mac OS X 10.4 and later';
    function glyphPacking: NSMultibyteGlyphPacking; message 'glyphPacking'; deprecated 'in Mac OS X 10.4 and later';
    function positionOfGlyph_precededByGlyph_isNominal(curGlyph: NSGlyph; prevGlyph: NSGlyph; nominal: pboolean): NSPoint; message 'positionOfGlyph:precededByGlyph:isNominal:'; deprecated 'in Mac OS X 10.4 and later';
    function positionsForCompositeSequence_numberOfGlyphs_pointArray(someGlyphs: NSGlyphPtr; numGlyphs: NSInteger; points: NSPointArray): NSInteger; message 'positionsForCompositeSequence:numberOfGlyphs:pointArray:'; deprecated 'in Mac OS X 10.4 and later';
    function positionOfGlyph_struckOverGlyph_metricsExist(curGlyph: NSGlyph; prevGlyph: NSGlyph; exist: pboolean): NSPoint; message 'positionOfGlyph:struckOverGlyph:metricsExist:'; deprecated 'in Mac OS X 10.4 and later';
    function positionOfGlyph_struckOverRect_metricsExist(aGlyph: NSGlyph; aRect: NSRect; exist: pboolean): NSPoint; message 'positionOfGlyph:struckOverRect:metricsExist:'; deprecated 'in Mac OS X 10.4 and later';
    function positionOfGlyph_forCharacter_struckOverRect(aGlyph: NSGlyph; aChar: unichar; aRect: NSRect): NSPoint; message 'positionOfGlyph:forCharacter:struckOverRect:'; deprecated 'in Mac OS X 10.4 and later';
    function positionOfGlyph_withRelation_toBaseGlyph_totalAdvancement_metricsExist(thisGlyph: NSGlyph; rel: NSGlyphRelation; baseGlyph: NSGlyph; adv: NSSizePointer; exist: pboolean): NSPoint; message 'positionOfGlyph:withRelation:toBaseGlyph:totalAdvancement:metricsExist:'; deprecated 'in Mac OS X 10.4 and later';
  end;

{$endif}
{$endif}
