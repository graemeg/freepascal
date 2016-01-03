{ Parsed from Appkit.framework NSParagraphStyle.h }

{$ifdef TYPES}
{$ifndef NSPARAGRAPHSTYLE_PAS_T}
{$define NSPARAGRAPHSTYLE_PAS_T}

{ Constants }

const
  NSLeftTabStopType = 0;
  NSRightTabStopType = 1;
  NSCenterTabStopType = 2;
  NSDecimalTabStopType = 3;

const
  NSLineBreakByWordWrapping = 0;
  NSLineBreakByCharWrapping = 1;
  NSLineBreakByClipping = 2;
  NSLineBreakByTruncatingHead = 3;
  NSLineBreakByTruncatingTail = 4;
  NSLineBreakByTruncatingMiddle = 5;

{ Types }
type
  NSTextTabType = NSUInteger;
  NSTextTabTypePtr = ^NSTextTabType;
  NSLineBreakMode = NSUInteger;
  NSLineBreakModePtr = ^NSLineBreakMode;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPARAGRAPHSTYLE_PAS_R}
{$define NSPARAGRAPHSTYLE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPARAGRAPHSTYLE_PAS_F}
{$define NSPARAGRAPHSTYLE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPARAGRAPHSTYLE_PAS_S}
{$define NSPARAGRAPHSTYLE_PAS_S}

{ External string constants }
var
  NSTabColumnTerminatorsAttributeName: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTextTab = objcclass;
  NSTextTabPointer = ^NSTextTab;
  NSTextTabPtr = NSTextTabPointer;
  NSParagraphStyle = objcclass;
  NSParagraphStylePointer = ^NSParagraphStyle;
  NSParagraphStylePtr = NSParagraphStylePointer;
  NSMutableParagraphStyle = objcclass;
  NSMutableParagraphStylePointer = ^NSMutableParagraphStyle;
  NSMutableParagraphStylePtr = NSMutableParagraphStylePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPARAGRAPHSTYLE_PAS_C}
{$define NSPARAGRAPHSTYLE_PAS_C}

{ NSTextTab }
  NSTextTab = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSParagraphStyle1: NSTextAlignment);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSParagraphStyle1: NSTextAlignment);
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
        _alignment: 0..((1 shl 4)-1);
        refCount: 0..((1 shl 24)-1);
        unused: 0..((1 shl 4)-1);
        {$ifdef cpu64}
        _anonbitfield_NSParagraphStyle0: 0..((1 shl 32)-1);
        {$endif}
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
    _location: CGFloat;
    _reserved: id;
    
  public
    function initWithTextAlignment_location_options(alignment_: NSTextAlignment; loc: CGFloat; options_: NSDictionary): id; message 'initWithTextAlignment:location:options:';
    function alignment: NSTextAlignment; message 'alignment';
    function options: NSDictionary; message 'options';
    function initWithType_location(type_: NSTextTabType; loc: CGFloat): id; message 'initWithType:location:';
    function location: CGFloat; message 'location';
    function tabStopType: NSTextTabType; message 'tabStopType';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSParagraphStyle }
  NSParagraphStyle = objcclass external (NSObject, NSCopyingProtocol, NSMutableCopyingProtocol, NSCodingProtocol)
  private
    _lineSpacing: CGFloat;
    _paragraphSpacing: CGFloat;
    _headIndent: CGFloat;
    _tailIndent: CGFloat;
    _firstLineHeadIndent: CGFloat;
    _minimumLineHeight: CGFloat;
    _maximumLineHeight: CGFloat;
    _tabStops: NSArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSParagraphStyle3: NSTextAlignment);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSParagraphStyle3: NSTextAlignment);
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
        _alignment: 0..((1 shl 4)-1);
        _lineBreakMode: 0..((1 shl 4)-1);
        tabStopsIsMutable: 0..1;
        isNaturalDirection: 0..1;
        rightToLeftDirection: 0..1;
        fixedMultiple: 0..((1 shl 2)-1);
        refCount: 0..((1 shl 19)-1);
        {$ifdef cpu64}
        _anonbitfield_NSParagraphStyle2: 0..((1 shl 32)-1);
        {$endif}
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
    _defaultTabInterval: CGFloat;
    _extraData: id;
    
  public
    class function defaultParagraphStyle: NSParagraphStyle; message 'defaultParagraphStyle';
    class function defaultWritingDirectionForLanguage(languageName: NSString): NSWritingDirection; message 'defaultWritingDirectionForLanguage:';
    function lineSpacing: CGFloat; message 'lineSpacing';
    function paragraphSpacing: CGFloat; message 'paragraphSpacing';
    function alignment: NSTextAlignment; message 'alignment';
    function headIndent: CGFloat; message 'headIndent';
    function tailIndent: CGFloat; message 'tailIndent';
    function firstLineHeadIndent: CGFloat; message 'firstLineHeadIndent';
    function tabStops: NSArray; message 'tabStops';
    function minimumLineHeight: CGFloat; message 'minimumLineHeight';
    function maximumLineHeight: CGFloat; message 'maximumLineHeight';
    function lineBreakMode: NSLineBreakMode; message 'lineBreakMode';
    function baseWritingDirection: NSWritingDirection; message 'baseWritingDirection';
    function lineHeightMultiple: CGFloat; message 'lineHeightMultiple';
    function paragraphSpacingBefore: CGFloat; message 'paragraphSpacingBefore';
    function defaultTabInterval: CGFloat; message 'defaultTabInterval';
    function textBlocks: NSArray; message 'textBlocks';
    function textLists: NSArray; message 'textLists';
    function hyphenationFactor: single; message 'hyphenationFactor';
    function tighteningFactorForTruncation: single; message 'tighteningFactorForTruncation';
    function headerLevel: NSInteger; message 'headerLevel';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    function mutableCopyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSMutableParagraphStyle }
  NSMutableParagraphStyle = objcclass external (NSParagraphStyle)
    
  public
    procedure setLineSpacing(aFloat: CGFloat); message 'setLineSpacing:';
    procedure setParagraphSpacing(aFloat: CGFloat); message 'setParagraphSpacing:';
    procedure setAlignment(alignment_: NSTextAlignment); message 'setAlignment:';
    procedure setFirstLineHeadIndent(aFloat: CGFloat); message 'setFirstLineHeadIndent:';
    procedure setHeadIndent(aFloat: CGFloat); message 'setHeadIndent:';
    procedure setTailIndent(aFloat: CGFloat); message 'setTailIndent:';
    procedure setLineBreakMode(mode: NSLineBreakMode); message 'setLineBreakMode:';
    procedure setMinimumLineHeight(aFloat: CGFloat); message 'setMinimumLineHeight:';
    procedure setMaximumLineHeight(aFloat: CGFloat); message 'setMaximumLineHeight:';
    procedure addTabStop(anObject: NSTextTab); message 'addTabStop:';
    procedure removeTabStop(anObject: NSTextTab); message 'removeTabStop:';
    procedure setTabStops(array_: NSArray); message 'setTabStops:';
    procedure setParagraphStyle(obj: NSParagraphStyle); message 'setParagraphStyle:';
    procedure setBaseWritingDirection(writingDirection: NSWritingDirection); message 'setBaseWritingDirection:';
    procedure setLineHeightMultiple(aFloat: CGFloat); message 'setLineHeightMultiple:';
    procedure setParagraphSpacingBefore(aFloat: CGFloat); message 'setParagraphSpacingBefore:';
    procedure setDefaultTabInterval(aFloat: CGFloat); message 'setDefaultTabInterval:';
    procedure setTextBlocks(array_: NSArray); message 'setTextBlocks:';
    procedure setTextLists(array_: NSArray); message 'setTextLists:';
    procedure setHyphenationFactor(aFactor: single); message 'setHyphenationFactor:';
    procedure setTighteningFactorForTruncation(aFactor: single); message 'setTighteningFactorForTruncation:';
    procedure setHeaderLevel(level: NSInteger); message 'setHeaderLevel:';
  end;

{$endif}
{$endif}
