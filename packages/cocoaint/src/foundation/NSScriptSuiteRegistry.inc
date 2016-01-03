{ Parsed from Foundation.framework NSScriptSuiteRegistry.h }

{$ifdef TYPES}
{$ifndef NSSCRIPTSUITEREGISTRY_PAS_T}
{$define NSSCRIPTSUITEREGISTRY_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSCRIPTSUITEREGISTRY_PAS_R}
{$define NSSCRIPTSUITEREGISTRY_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSCRIPTSUITEREGISTRY_PAS_F}
{$define NSSCRIPTSUITEREGISTRY_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSCRIPTSUITEREGISTRY_PAS_S}
{$define NSSCRIPTSUITEREGISTRY_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSScriptSuiteRegistry = objcclass;
  NSScriptSuiteRegistryPointer = ^NSScriptSuiteRegistry;
  NSScriptSuiteRegistryPtr = NSScriptSuiteRegistryPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSCRIPTSUITEREGISTRY_PAS_C}
{$define NSSCRIPTSUITEREGISTRY_PAS_C}

{ NSScriptSuiteRegistry }
  NSScriptSuiteRegistry = objcclass external (NSObject)
  private
    _isLoadingSDEFFiles: Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved1: array[0..(3)-1] of char;
=======
        _reserved1: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved1: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved1: array[0..2] of char;
>>>>>>> graemeg/cpstrnew
=======
        _reserved1: array[0..2] of char;
>>>>>>> origin/cpstrnew
    _seenBundles: NSMutableSet;
    _suiteDescriptionsBeingCollected: NSMutableArray;
    _classDescriptionNeedingRegistration: NSScriptClassDescription;
    _suiteDescriptions: NSMutableArray;
    _commandDescriptionNeedingRegistration: NSScriptCommandDescription;
    _cachedClassDescriptionsByAppleEventCode: NSMutableDictionary;
    _cachedCommandDescriptionsByAppleEventCodes: NSMutableDictionary;
    _cachedSuiteDescriptionsByName: NSDictionary;
    _complexTypeDescriptionsByName: NSMutableDictionary;
    _listTypeDescriptionsByName: NSMutableDictionary;
    _nextComplexTypeAppleEventCode: cuint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved2: array[0..(4)-1] of Pointer;
=======
        _reserved2: array[0..3] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved2: array[0..3] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved2: array[0..3] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved2: array[0..3] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class function sharedScriptSuiteRegistry: NSScriptSuiteRegistry; message 'sharedScriptSuiteRegistry';
    class procedure setSharedScriptSuiteRegistry(registry: NSScriptSuiteRegistry); message 'setSharedScriptSuiteRegistry:';
    procedure loadSuitesFromBundle(bundle: NSBundle); message 'loadSuitesFromBundle:';
    procedure loadSuiteWithDictionary_fromBundle(suiteDeclaration: NSDictionary; bundle: NSBundle); message 'loadSuiteWithDictionary:fromBundle:';
    procedure registerClassDescription(classDescription: NSScriptClassDescription); message 'registerClassDescription:';
    procedure registerCommandDescription(commandDescription: NSScriptCommandDescription); message 'registerCommandDescription:';
    function suiteNames: NSArray; message 'suiteNames';
    function appleEventCodeForSuite(suiteName: NSString): FourCharCode; message 'appleEventCodeForSuite:';
    function bundleForSuite(suiteName: NSString): NSBundle; message 'bundleForSuite:';
    function classDescriptionsInSuite(suiteName: NSString): NSDictionary; message 'classDescriptionsInSuite:';
    function commandDescriptionsInSuite(suiteName: NSString): NSDictionary; message 'commandDescriptionsInSuite:';
    function suiteForAppleEventCode(appleEventCode: FourCharCode): NSString; message 'suiteForAppleEventCode:';
    function classDescriptionWithAppleEventCode(appleEventCode: FourCharCode): NSScriptClassDescription; message 'classDescriptionWithAppleEventCode:';
    function commandDescriptionWithAppleEventClass_andAppleEventCode(appleEventClassCode: FourCharCode; appleEventIDCode: FourCharCode): NSScriptCommandDescription; message 'commandDescriptionWithAppleEventClass:andAppleEventCode:';
    function aeteResource(languageName: NSString): NSData; message 'aeteResource:';
  end;

{$endif}
{$endif}
