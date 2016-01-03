{ Parsed from Appkit.framework NSFontManager.h }

{$ifdef TYPES}
{$ifndef NSFONTMANAGER_PAS_T}
{$define NSFONTMANAGER_PAS_T}

{ Types }
type
  NSFontTraitMask = NSUInteger;
  NSFontTraitMaskPtr = ^NSFontTraitMask;
  NSFontAction = NSUInteger;
  NSFontActionPtr = ^NSFontAction;

{ Constants }

const
  NSItalicFontMask = $00000001;
  NSBoldFontMask = $00000002;
  NSUnboldFontMask = $00000004;
  NSNonStandardCharacterSetFontMask = $00000008;
  NSNarrowFontMask = $00000010;
  NSExpandedFontMask = $00000020;
  NSCondensedFontMask = $00000040;
  NSSmallCapsFontMask = $00000080;
  NSPosterFontMask = $00000100;
  NSCompressedFontMask = $00000200;
  NSFixedPitchFontMask = $00000400;
  NSUnitalicFontMask = $01000000;

const
  NSFontCollectionApplicationOnlyMask = 1 shl 0;

const
  NSNoFontChangeAction = 0;
  NSViaPanelFontAction = 1;
  NSAddTraitFontAction = 2;
  NSSizeUpFontAction = 3;
  NSSizeDownFontAction = 4;
  NSHeavierFontAction = 5;
  NSLighterFontAction = 6;
  NSRemoveTraitFontAction = 7;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSFONTMANAGER_PAS_R}
{$define NSFONTMANAGER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSFONTMANAGER_PAS_F}
{$define NSFONTMANAGER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSFONTMANAGER_PAS_S}
{$define NSFONTMANAGER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSFontManager = objcclass;
  NSFontManagerPointer = ^NSFontManager;
  NSFontManagerPtr = NSFontManagerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSFONTMANAGER_PAS_C}
{$define NSFONTMANAGER_PAS_C}

{ NSFontManager }
  NSFontManager = objcclass external (NSObject)
  private
    _panel: NSFontPanel;
    _fmReserved1: cuint;
    _action: SEL;
    _actionOrigin: id;
    _target: id;
    _selFont: NSFont;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __fmFlags: record
      case byte of
       0: (_anonbitfield_NSFontManager0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __fmFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSFontManager0: cuint);
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
        multipleFont: 0..1;
        disabled: 0..1;
        senderTagMode: 0..((1 shl 2)-1);
        _RESERVED: 0..((1 shl 12)-1);
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
    _fmReserved3: cushort;
    _delegate: id;
    _collections: id;
    _hiddenCollections: id;
    _fmReserved4: NSUInteger;
    
  public
    class procedure setFontPanelFactory(factoryId: Pobjc_class); message 'setFontPanelFactory:';
    class procedure setFontManagerFactory(factoryId: Pobjc_class); message 'setFontManagerFactory:';
    class function sharedFontManager: NSFontManager; message 'sharedFontManager';
    function isMultiple: Boolean; message 'isMultiple';
    function selectedFont: NSFont; message 'selectedFont';
    procedure setSelectedFont_isMultiple(fontObj: NSFont; flag: Boolean); message 'setSelectedFont:isMultiple:';
    procedure setFontMenu(newMenu: NSMenu); message 'setFontMenu:';
    function fontMenu(create_: Boolean): NSMenu; message 'fontMenu:';
    function fontPanel(create_: Boolean): NSFontPanel; message 'fontPanel:';
    function fontWithFamily_traits_weight_size(family: NSString; traits: NSFontTraitMask; weight: NSInteger; size: CGFloat): NSFont; message 'fontWithFamily:traits:weight:size:';
    function traitsOfFont(fontObj: NSFont): NSFontTraitMask; message 'traitsOfFont:';
    function weightOfFont(fontObj: NSFont): NSInteger; message 'weightOfFont:';
    function availableFonts: NSArray; message 'availableFonts';
    function availableFontFamilies: NSArray; message 'availableFontFamilies';
    function availableMembersOfFontFamily(fam: NSString): NSArray; message 'availableMembersOfFontFamily:';
    function convertFont(fontObj: NSFont): NSFont; message 'convertFont:';
    function convertFont_toSize(fontObj: NSFont; size: CGFloat): NSFont; message 'convertFont:toSize:';
    function convertFont_toFace(fontObj: NSFont; typeface: NSString): NSFont; message 'convertFont:toFace:';
    function convertFont_toFamily(fontObj: NSFont; family: NSString): NSFont; message 'convertFont:toFamily:';
    function convertFont_toHaveTrait(fontObj: NSFont; trait: NSFontTraitMask): NSFont; message 'convertFont:toHaveTrait:';
    function convertFont_toNotHaveTrait(fontObj: NSFont; trait: NSFontTraitMask): NSFont; message 'convertFont:toNotHaveTrait:';
    function convertWeight_ofFont(upFlag: Boolean; fontObj: NSFont): NSFont; message 'convertWeight:ofFont:';
    function isEnabled: Boolean; message 'isEnabled';
    procedure setEnabled(flag: Boolean); message 'setEnabled:';
    function action: SEL; message 'action';
    procedure setAction(aSelector: SEL); message 'setAction:';
    function sendAction: Boolean; message 'sendAction';
    procedure setDelegate(anObject: id); message 'setDelegate:';
    function delegate: id; message 'delegate';
    function localizedNameForFamily_face(family: NSString; faceKey: NSString): NSString; message 'localizedNameForFamily:face:';
    procedure setSelectedAttributes_isMultiple(attributes: NSDictionary; flag: Boolean); message 'setSelectedAttributes:isMultiple:';
    function convertAttributes(attributes: NSDictionary): NSDictionary; message 'convertAttributes:';
    function availableFontNamesMatchingFontDescriptor(descriptor: NSFontDescriptor): NSArray; message 'availableFontNamesMatchingFontDescriptor:';
    function collectionNames: NSArray; message 'collectionNames';
    function fontDescriptorsInCollection(collectionNames_: NSString): NSArray; message 'fontDescriptorsInCollection:';
    function addCollection_options(collectionName: NSString; collectionOptions: NSInteger): Boolean; message 'addCollection:options:';
    function removeCollection(collectionName: NSString): Boolean; message 'removeCollection:';
    procedure addFontDescriptors_toCollection(descriptors: NSArray; collectionName: NSString); message 'addFontDescriptors:toCollection:';
    procedure removeFontDescriptor_fromCollection(descriptor: NSFontDescriptor; collection: NSString); message 'removeFontDescriptor:fromCollection:';
    function currentFontAction: NSFontAction; message 'currentFontAction';
    function convertFontTraits(traits: NSFontTraitMask): NSFontTraitMask; message 'convertFontTraits:';
    procedure setTarget(aTarget: id); message 'setTarget:';
    function target: id; message 'target';
  end;

{ NSFontManagerMenuActionMethodsCategory }
  NSFontManagerMenuActionMethodsCategory = objccategory external (NSFontManager)
    function fontNamed_hasTraits(fName: NSString; someTraits: NSFontTraitMask): Boolean; message 'fontNamed:hasTraits:';
    function availableFontNamesWithTraits(someTraits: NSFontTraitMask): NSArray; message 'availableFontNamesWithTraits:';
    procedure addFontTrait(sender: id); message 'addFontTrait:';
    procedure removeFontTrait(sender: id); message 'removeFontTrait:';
    procedure modifyFontViaPanel(sender: id); message 'modifyFontViaPanel:';
    procedure modifyFont(sender: id); message 'modifyFont:';
    procedure orderFrontFontPanel(sender: id); message 'orderFrontFontPanel:';
    procedure orderFrontStylesPanel(sender: id); message 'orderFrontStylesPanel:';
  end;

{ NSFontManagerDelegateCategory }
  NSFontManagerDelegateCategory = objccategory external (NSObject)
    function fontManager_willIncludeFont(sender: id; fontName: NSString): Boolean; message 'fontManager:willIncludeFont:';
  end;

{ NSFontManagerResponderMethodCategory }
  NSFontManagerResponderMethodCategory = objccategory external (NSObject)
    procedure changeFont(sender: id); message 'changeFont:';
  end;

{$endif}
{$endif}
