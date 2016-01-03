{ Parsed from Appkit.framework NSUserDefaultsController.h }

{$ifdef TYPES}
{$ifndef NSUSERDEFAULTSCONTROLLER_PAS_T}
{$define NSUSERDEFAULTSCONTROLLER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSUSERDEFAULTSCONTROLLER_PAS_R}
{$define NSUSERDEFAULTSCONTROLLER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSUSERDEFAULTSCONTROLLER_PAS_F}
{$define NSUSERDEFAULTSCONTROLLER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSUSERDEFAULTSCONTROLLER_PAS_S}
{$define NSUSERDEFAULTSCONTROLLER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSUserDefaultsController = objcclass;
  NSUserDefaultsControllerPointer = ^NSUserDefaultsController;
  NSUserDefaultsControllerPtr = NSUserDefaultsControllerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSUSERDEFAULTSCONTROLLER_PAS_C}
{$define NSUSERDEFAULTSCONTROLLER_PAS_C}

{ NSUserDefaultsController }
  NSUserDefaultsController = objcclass external (NSController)
  private
    _reserved3: Pointer;
    _reserved4: Pointer;
    _defaults: NSUserDefaults;
    _valueBuffer: NSMutableDictionary;
    _initialValues: NSDictionary;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __userDefaultsControllerFlags: record
      case byte of
       0: (_anonbitfield_NSUserDefaultsController0: cuint);
       1: (data: bitpacked record
        _sharedInstance: 0..1;
        _appliesImmediately: 0..1;
        _reservedUserDefaultsController: 0..((1 shl 30)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __userDefaultsControllerFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSUserDefaultsController0: cuint);
       1: (
        _sharedInstance: 0..1;
        _appliesImmediately: 0..1;
        _reservedUserDefaultsController: 0..((1 shl 30)-1);
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
      );
      end;
    
  public
    class function sharedUserDefaultsController: id; message 'sharedUserDefaultsController';
    function initWithDefaults_initialValues(defaults_: NSUserDefaults; initialValues_: NSDictionary): id; message 'initWithDefaults:initialValues:';
    function defaults: NSUserDefaults; message 'defaults';
    procedure setInitialValues(initialValues_: NSDictionary); message 'setInitialValues:';
    function initialValues: NSDictionary; message 'initialValues';
    procedure setAppliesImmediately(flag: Boolean); message 'setAppliesImmediately:';
    function appliesImmediately: Boolean; message 'appliesImmediately';
    function hasUnappliedChanges: Boolean; message 'hasUnappliedChanges';
    function values: id; message 'values';
    procedure revert(sender: id); message 'revert:';
    procedure save(sender: id); message 'save:';
    procedure revertToInitialValues(sender: id); message 'revertToInitialValues:';
  end;

{$endif}
{$endif}
