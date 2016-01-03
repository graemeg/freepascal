{ Parsed from Appkit.framework NSController.h }

{$ifdef TYPES}
{$ifndef NSCONTROLLER_PAS_T}
{$define NSCONTROLLER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCONTROLLER_PAS_R}
{$define NSCONTROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCONTROLLER_PAS_F}
{$define NSCONTROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCONTROLLER_PAS_S}
{$define NSCONTROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSController = objcclass;
  NSControllerPointer = ^NSController;
  NSControllerPtr = NSControllerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCONTROLLER_PAS_C}
{$define NSCONTROLLER_PAS_C}

{ NSController }
  NSController = objcclass external (NSObject, NSCodingProtocol)
  private
    _reserved: Pointer;
    _reserved2: Pointer;
    _specialPurposeType: cint;
    _bindingAdaptor: id;
    _editors: NSMutableArray;
    _declaredKeys: NSMutableArray;
    _dependentKeyToModelKeyTable: NSMutableDictionary;
    _modelKeyToDependentKeyTable: NSMutableDictionary;
    _modelKeysToRefreshEachTime: NSMutableArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __bindingsControllerFlags: record
      case byte of
       0: (_anonbitfield_NSController0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __bindingsControllerFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSController0: cuint);
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
        _alwaysPresentsApplicationModalAlerts: 0..1;
        _refreshesAllModelKeys: 0..1;
        _multipleObservedModelObjects: 0..1;
        _isEditing: 0..1;
        _reservedController: 0..((1 shl 28)-1);
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
    _reservedOther: NSMutableDictionary;
    _modelObservingTracker: id;
    _expectedObservingInfo: id;
    _singleValueAccessor: id;
    
  public
    procedure objectDidBeginEditing(editor: id); message 'objectDidBeginEditing:';
    procedure objectDidEndEditing(editor: id); message 'objectDidEndEditing:';
    procedure discardEditing; message 'discardEditing';
    function commitEditing: Boolean; message 'commitEditing';
    procedure commitEditingWithDelegate_didCommitSelector_contextInfo(delegate: id; didCommitSelector: SEL; contextInfo: Pointer); message 'commitEditingWithDelegate:didCommitSelector:contextInfo:';
    function isEditing: Boolean; message 'isEditing';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
