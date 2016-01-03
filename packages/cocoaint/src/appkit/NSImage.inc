{ Parsed from Appkit.framework NSImage.h }

{$ifdef TYPES}
{$ifndef NSIMAGE_PAS_T}
{$define NSIMAGE_PAS_T}

{ Constants }

const
  NSImageLoadStatusCompleted = 0;
  NSImageLoadStatusCancelled = 1;
  NSImageLoadStatusInvalidData = 2;
  NSImageLoadStatusUnexpectedEOF = 3;
  NSImageLoadStatusReadError = 4;

const
  NSImageCacheDefault = 0;
  NSImageCacheAlways = 1;
  NSImageCacheBySize = 2;
  NSImageCacheNever = 3;

{ Types }
type
  NSImageLoadStatus = NSUInteger;
  NSImageLoadStatusPtr = ^NSImageLoadStatus;
  NSImageCacheMode = NSUInteger;
  NSImageCacheModePtr = ^NSImageCacheMode;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSIMAGE_PAS_R}
{$define NSIMAGE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSIMAGE_PAS_F}
{$define NSIMAGE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSIMAGE_PAS_S}
{$define NSIMAGE_PAS_S}

{ External string constants }
var
  NSImageHintCTM: NSString; cvar; external;
  NSImageHintInterpolation: NSString; cvar; external;
  NSImageNameQuickLookTemplate: NSString; cvar; external;
  NSImageNameBluetoothTemplate: NSString; cvar; external;
  NSImageNameIChatTheaterTemplate: NSString; cvar; external;
  NSImageNameSlideshowTemplate: NSString; cvar; external;
  NSImageNameActionTemplate: NSString; cvar; external;
  NSImageNameSmartBadgeTemplate: NSString; cvar; external;
  NSImageNameIconViewTemplate: NSString; cvar; external;
  NSImageNameListViewTemplate: NSString; cvar; external;
  NSImageNameColumnViewTemplate: NSString; cvar; external;
  NSImageNameFlowViewTemplate: NSString; cvar; external;
  NSImageNamePathTemplate: NSString; cvar; external;
  NSImageNameInvalidDataFreestandingTemplate: NSString; cvar; external;
  NSImageNameLockLockedTemplate: NSString; cvar; external;
  NSImageNameLockUnlockedTemplate: NSString; cvar; external;
  NSImageNameGoRightTemplate: NSString; cvar; external;
  NSImageNameGoLeftTemplate: NSString; cvar; external;
  NSImageNameRightFacingTriangleTemplate: NSString; cvar; external;
  NSImageNameLeftFacingTriangleTemplate: NSString; cvar; external;
  NSImageNameAddTemplate: NSString; cvar; external;
  NSImageNameRemoveTemplate: NSString; cvar; external;
  NSImageNameRevealFreestandingTemplate: NSString; cvar; external;
  NSImageNameFollowLinkFreestandingTemplate: NSString; cvar; external;
  NSImageNameEnterFullScreenTemplate: NSString; cvar; external;
  NSImageNameExitFullScreenTemplate: NSString; cvar; external;
  NSImageNameStopProgressTemplate: NSString; cvar; external;
  NSImageNameStopProgressFreestandingTemplate: NSString; cvar; external;
  NSImageNameRefreshTemplate: NSString; cvar; external;
  NSImageNameRefreshFreestandingTemplate: NSString; cvar; external;
  NSImageNameBonjour: NSString; cvar; external;
  NSImageNameComputer: NSString; cvar; external;
  NSImageNameFolderBurnable: NSString; cvar; external;
  NSImageNameFolderSmart: NSString; cvar; external;
  NSImageNameFolder: NSString; cvar; external;
  NSImageNameNetwork: NSString; cvar; external;
  NSImageNameDotMac: NSString; cvar; external;
  NSImageNameMobileMe: NSString; cvar; external;
  NSImageNameMultipleDocuments: NSString; cvar; external;
  NSImageNameUserAccounts: NSString; cvar; external;
  NSImageNamePreferencesGeneral: NSString; cvar; external;
  NSImageNameAdvanced: NSString; cvar; external;
  NSImageNameInfo: NSString; cvar; external;
  NSImageNameFontPanel: NSString; cvar; external;
  NSImageNameColorPanel: NSString; cvar; external;
  NSImageNameUser: NSString; cvar; external;
  NSImageNameUserGroup: NSString; cvar; external;
  NSImageNameEveryone: NSString; cvar; external;
  NSImageNameUserGuest: NSString; cvar; external;
  NSImageNameMenuOnStateTemplate: NSString; cvar; external;
  NSImageNameMenuMixedStateTemplate: NSString; cvar; external;
  NSImageNameApplicationIcon: NSString; cvar; external;
  NSImageNameTrashEmpty: NSString; cvar; external;
  NSImageNameTrashFull: NSString; cvar; external;
  NSImageNameHomeTemplate: NSString; cvar; external;
  NSImageNameBookmarksTemplate: NSString; cvar; external;
  NSImageNameCaution: NSString; cvar; external;
  NSImageNameStatusAvailable: NSString; cvar; external;
  NSImageNameStatusPartiallyAvailable: NSString; cvar; external;
  NSImageNameStatusUnavailable: NSString; cvar; external;
  NSImageNameStatusNone: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSImageDelegateProtocol = objcprotocol;
  NSImage = objcclass;
  NSImagePointer = ^NSImage;
  NSImagePtr = NSImagePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSIMAGE_PAS_C}
{$define NSIMAGE_PAS_C}

{ NSImage }
  NSImage = objcclass external (NSObject)
  private
    _name: NSString;
    _size: NSSize;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSImage0: cuint);
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
       0: (_anonbitfield_NSImage0: cuint);
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
        scalable: 0..1;
        dataRetained: 0..1;
        uniqueWindow: 0..1;
        sizeWasExplicitlySet: 0..1;
        builtIn: 0..1;
        needsToExpand: 0..1;
        useEPSOnResolutionMismatch: 0..1;
        colorMatchPreferred: 0..1;
        multipleResolutionMatching: 0..1;
        focusedWhilePrinting: 0..1;
        archiveByName: 0..1;
        unboundedCacheDepth: 0..1;
        flipped: 0..1;
        aliased: 0..1;
        dirtied: 0..1;
        _cacheMode: 0..((1 shl 2)-1);
        sampleMode: 0..((1 shl 3)-1);
        reserved2: 0..1;
        _isTemplate: 0..1;
        failedToExpand: 0..1;
        reserved1: 0..((1 shl 9)-1);
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
    _reps: id; {garbage collector: volatile }
    _imageAuxiliary: _NSImageAuxiliary;
    
  public
    class function imageNamed(name_: NSString): id; message 'imageNamed:';
    function initWithSize(aSize: NSSize): id; message 'initWithSize:';
    function initWithData(data: NSData): id; message 'initWithData:';
    function initWithContentsOfFile(fileName: NSString): id; message 'initWithContentsOfFile:';
    function initWithContentsOfURL(url: NSURL): id; message 'initWithContentsOfURL:';
    function initByReferencingFile(fileName: NSString): id; message 'initByReferencingFile:';
    function initByReferencingURL(url: NSURL): id; message 'initByReferencingURL:';
    function initWithIconRef(iconRef_: IconRef): id; message 'initWithIconRef:';
    function initWithPasteboard(pasteboard: NSPasteboard): id; message 'initWithPasteboard:';
    function initWithDataIgnoringOrientation(data: NSData): id; message 'initWithDataIgnoringOrientation:';
    procedure setSize(aSize: NSSize); message 'setSize:';
    function size: NSSize; message 'size';
    function setName(string_: NSString): Boolean; message 'setName:';
    function name: NSString; message 'name';
    procedure setBackgroundColor(aColor: NSColor); message 'setBackgroundColor:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure setUsesEPSOnResolutionMismatch(flag: Boolean); message 'setUsesEPSOnResolutionMismatch:';
    function usesEPSOnResolutionMismatch: Boolean; message 'usesEPSOnResolutionMismatch';
    procedure setPrefersColorMatch(flag: Boolean); message 'setPrefersColorMatch:';
    function prefersColorMatch: Boolean; message 'prefersColorMatch';
    procedure setMatchesOnMultipleResolution(flag: Boolean); message 'setMatchesOnMultipleResolution:';
    function matchesOnMultipleResolution: Boolean; message 'matchesOnMultipleResolution';
    procedure drawAtPoint_fromRect_operation_fraction(point: NSPoint; fromRect: NSRect; op: NSCompositingOperation; delta: CGFloat); message 'drawAtPoint:fromRect:operation:fraction:';
    procedure drawInRect_fromRect_operation_fraction(rect: NSRect; fromRect: NSRect; op: NSCompositingOperation; delta: CGFloat); message 'drawInRect:fromRect:operation:fraction:';
    procedure drawInRect_fromRect_operation_fraction_respectFlipped_hints(dstSpacePortionRect: NSRect; srcSpacePortionRect: NSRect; op: NSCompositingOperation; requestedAlpha: CGFloat; respectContextIsFlipped: Boolean; hints: NSDictionary); message 'drawInRect:fromRect:operation:fraction:respectFlipped:hints:';
    function drawRepresentation_inRect(imageRep: NSImageRep; rect: NSRect): Boolean; message 'drawRepresentation:inRect:';
    procedure recache; message 'recache';
    function TIFFRepresentation: NSData; message 'TIFFRepresentation';
    function TIFFRepresentationUsingCompression_factor(comp: NSTIFFCompression; aFloat: single): NSData; message 'TIFFRepresentationUsingCompression:factor:';
    function representations: NSArray; message 'representations';
    procedure addRepresentations(imageReps: NSArray); message 'addRepresentations:';
    procedure addRepresentation(imageRep: NSImageRep); message 'addRepresentation:';
    procedure removeRepresentation(imageRep: NSImageRep); message 'removeRepresentation:';
    function isValid: Boolean; message 'isValid';
    procedure lockFocus; message 'lockFocus';
    procedure lockFocusFlipped(flipped: Boolean); message 'lockFocusFlipped:';
    procedure unlockFocus; message 'unlockFocus';
    function bestRepresentationForDevice(deviceDescription: NSDictionary): NSImageRep; message 'bestRepresentationForDevice:'; deprecated 'in Mac OS X 10.6 and later';
    procedure setDelegate(anObject: NSImageDelegateProtocol); message 'setDelegate:';
    function delegate: NSImageDelegateProtocol; message 'delegate';
    class function imageUnfilteredFileTypes: NSArray; message 'imageUnfilteredFileTypes';
    class function imageUnfilteredPasteboardTypes: NSArray; message 'imageUnfilteredPasteboardTypes';
    class function imageFileTypes: NSArray; message 'imageFileTypes';
    class function imagePasteboardTypes: NSArray; message 'imagePasteboardTypes';
    class function imageTypes: NSArray; message 'imageTypes';
    class function imageUnfilteredTypes: NSArray; message 'imageUnfilteredTypes';
    class function canInitWithPasteboard(pasteboard: NSPasteboard): Boolean; message 'canInitWithPasteboard:';
    procedure cancelIncrementalLoad; message 'cancelIncrementalLoad';
    procedure setCacheMode(mode: NSImageCacheMode); message 'setCacheMode:';
    function cacheMode: NSImageCacheMode; message 'cacheMode';
    function alignmentRect: NSRect; message 'alignmentRect';
    procedure setAlignmentRect(rect: NSRect); message 'setAlignmentRect:';
    function isTemplate: Boolean; message 'isTemplate';
    procedure setTemplate(isTemplate_: Boolean); message 'setTemplate:';
    function accessibilityDescription: NSString; message 'accessibilityDescription';
    procedure setAccessibilityDescription(description_: NSString); message 'setAccessibilityDescription:';
    function initWithCGImage_size(cgImage: CGImageRef; size_: NSSize): id; message 'initWithCGImage:size:';
    function CGImageForProposedRect_context_hints(proposedDestRect: NSRectPtr; referenceContext: NSGraphicsContext; hints: NSDictionary): CGImageRef; message 'CGImageForProposedRect:context:hints:';
    function bestRepresentationForRect_context_hints(rect: NSRect; referenceContext: NSGraphicsContext; hints: NSDictionary): NSImageRep; message 'bestRepresentationForRect:context:hints:';
    function hitTestRect_withImageDestinationRect_context_hints_flipped(testRectDestSpace: NSRect; imageRectDestSpace: NSRect; context: NSGraphicsContext; hints: NSDictionary; flipped: Boolean): Boolean; message 'hitTestRect:withImageDestinationRect:context:hints:flipped:';
  end;

{ NSBundleImageExtensionCategory }
  NSBundleImageExtensionCategory = objccategory external (NSBundle)
    function pathForImageResource(name: NSString): NSString; message 'pathForImageResource:';
    function URLForImageResource(name: NSString): NSURL; message 'URLForImageResource:';
  end;

{ NSDeprecated_NSImageCategory }
  NSDeprecated_NSImageCategory = objccategory external name 'NSDeprecated' (NSImage)
    procedure setFlipped(flag: Boolean); message 'setFlipped:';
    function isFlipped: Boolean; message 'isFlipped';
    procedure dissolveToPoint_fraction(point: NSPoint; aFloat: CGFloat); message 'dissolveToPoint:fraction:';
    procedure dissolveToPoint_fromRect_fraction(point: NSPoint; rect: NSRect; aFloat: CGFloat); message 'dissolveToPoint:fromRect:fraction:';
    procedure compositeToPoint_operation(point: NSPoint; op: NSCompositingOperation); message 'compositeToPoint:operation:';
    procedure compositeToPoint_fromRect_operation(point: NSPoint; rect: NSRect; op: NSCompositingOperation); message 'compositeToPoint:fromRect:operation:';
    procedure compositeToPoint_operation_fraction(point: NSPoint; op: NSCompositingOperation; delta: CGFloat); message 'compositeToPoint:operation:fraction:';
    procedure compositeToPoint_fromRect_operation_fraction(point: NSPoint; rect: NSRect; op: NSCompositingOperation; delta: CGFloat); message 'compositeToPoint:fromRect:operation:fraction:';
    procedure lockFocusOnRepresentation(imageRepresentation: NSImageRep); message 'lockFocusOnRepresentation:';
    procedure setScalesWhenResized(flag: Boolean); message 'setScalesWhenResized:';
    function scalesWhenResized: Boolean; message 'scalesWhenResized';
    procedure setDataRetained(flag: Boolean); message 'setDataRetained:';
    function isDataRetained: Boolean; message 'isDataRetained';
    procedure setCachedSeparately(flag: Boolean); message 'setCachedSeparately:';
    function isCachedSeparately: Boolean; message 'isCachedSeparately';
    procedure setCacheDepthMatchesImageDepth(flag: Boolean); message 'setCacheDepthMatchesImageDepth:';
    function cacheDepthMatchesImageDepth: Boolean; message 'cacheDepthMatchesImageDepth';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSIMAGE_PAS_P}
{$define NSIMAGE_PAS_P}
  
{ NSImageDelegate Protocol }
  NSImageDelegateProtocol = objcprotocol external name 'NSImageDelegate'
  optional
    function imageDidNotDraw_inRect(sender: id; aRect: NSRect): NSImage; message 'imageDidNotDraw:inRect:';
    procedure image_willLoadRepresentation(image: NSImage; rep: NSImageRep); message 'image:willLoadRepresentation:';
    procedure image_didLoadRepresentationHeader(image: NSImage; rep: NSImageRep); message 'image:didLoadRepresentationHeader:';
    procedure image_didLoadPartOfRepresentation_withValidRows(image: NSImage; rep: NSImageRep; rows: NSInteger); message 'image:didLoadPartOfRepresentation:withValidRows:';
    procedure image_didLoadRepresentation_withStatus(image: NSImage; rep: NSImageRep; status: NSImageLoadStatus); message 'image:didLoadRepresentation:withStatus:';
  end;
{$endif}
{$endif}
