{ Parsed from Foundation.framework NSUserDefaults.h }

{$ifdef TYPES}
{$ifndef NSUSERDEFAULTS_PAS_T}
{$define NSUSERDEFAULTS_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSUSERDEFAULTS_PAS_R}
{$define NSUSERDEFAULTS_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSUSERDEFAULTS_PAS_F}
{$define NSUSERDEFAULTS_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSUSERDEFAULTS_PAS_S}
{$define NSUSERDEFAULTS_PAS_S}

{ External string constants }
var
  NSGlobalDomain: NSString; cvar; external;
  NSArgumentDomain: NSString; cvar; external;
  NSRegistrationDomain: NSString; cvar; external;
  NSUserDefaultsDidChangeNotification: NSString; cvar; external;
  NSWeekDayNameArray: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSShortWeekDayNameArray: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSMonthNameArray: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSShortMonthNameArray: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSTimeFormatString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSDateFormatString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSTimeDateFormatString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSShortTimeDateFormatString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSCurrencySymbol: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSDecimalSeparator: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSThousandsSeparator: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSDecimalDigits: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSAMPMDesignation: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSHourNameDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSYearMonthWeekDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSEarlierTimeDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSLaterTimeDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSThisDayDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSNextDayDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSNextNextDayDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSPriorDayDesignations: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSDateTimeOrdering: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSInternationalCurrencyString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSShortDateFormatString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSPositiveCurrencyFormatString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;
  NSNegativeCurrencyFormatString: NSString deprecated 'in Mac OS X 10.5 and later'; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSUserDefaults = objcclass;
  NSUserDefaultsPointer = ^NSUserDefaults;
  NSUserDefaultsPtr = NSUserDefaultsPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSUSERDEFAULTS_PAS_C}
{$define NSUSERDEFAULTS_PAS_C}

{ NSUserDefaults }
  NSUserDefaults = objcclass external (NSObject)
  private
    _private: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(4)-1] of Pointer;
=======
        _reserved: array[0..3] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..3] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..3] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..3] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class function standardUserDefaults: NSUserDefaults; message 'standardUserDefaults';
    class procedure resetStandardUserDefaults; message 'resetStandardUserDefaults';
    function init: id; message 'init';
    function initWithUser(username: NSString): id; message 'initWithUser:';
    function objectForKey(defaultName: NSString): id; message 'objectForKey:';
    procedure setObject_forKey(value: id; defaultName: NSString); message 'setObject:forKey:';
    procedure removeObjectForKey(defaultName: NSString); message 'removeObjectForKey:';
    function stringForKey(defaultName: NSString): NSString; message 'stringForKey:';
    function arrayForKey(defaultName: NSString): NSArray; message 'arrayForKey:';
    function dictionaryForKey(defaultName: NSString): NSDictionary; message 'dictionaryForKey:';
    function dataForKey(defaultName: NSString): NSData; message 'dataForKey:';
    function stringArrayForKey(defaultName: NSString): NSArray; message 'stringArrayForKey:';
    function integerForKey(defaultName: NSString): NSInteger; message 'integerForKey:';
    function floatForKey(defaultName: NSString): single; message 'floatForKey:';
    function doubleForKey(defaultName: NSString): double; message 'doubleForKey:';
    function boolForKey(defaultName: NSString): Boolean; message 'boolForKey:';
    function URLForKey(defaultName: NSString): NSURL; message 'URLForKey:';
    procedure setInteger_forKey(value: NSInteger; defaultName: NSString); message 'setInteger:forKey:';
    procedure setFloat_forKey(value: single; defaultName: NSString); message 'setFloat:forKey:';
    procedure setDouble_forKey(value: double; defaultName: NSString); message 'setDouble:forKey:';
    procedure setBool_forKey(value: Boolean; defaultName: NSString); message 'setBool:forKey:';
    procedure setURL_forKey(url: NSURL; defaultName: NSString); message 'setURL:forKey:';
    procedure registerDefaults(registrationDictionary: NSDictionary); message 'registerDefaults:';
    procedure addSuiteNamed(suiteName: NSString); message 'addSuiteNamed:';
    procedure removeSuiteNamed(suiteName: NSString); message 'removeSuiteNamed:';
    function dictionaryRepresentation: NSDictionary; message 'dictionaryRepresentation';
    function volatileDomainNames: NSArray; message 'volatileDomainNames';
    function volatileDomainForName(domainName: NSString): NSDictionary; message 'volatileDomainForName:';
    procedure setVolatileDomain_forName(domain: NSDictionary; domainName: NSString); message 'setVolatileDomain:forName:';
    procedure removeVolatileDomainForName(domainName: NSString); message 'removeVolatileDomainForName:';
    function persistentDomainNames: NSArray; message 'persistentDomainNames';
    function persistentDomainForName(domainName: NSString): NSDictionary; message 'persistentDomainForName:';
    procedure setPersistentDomain_forName(domain: NSDictionary; domainName: NSString); message 'setPersistentDomain:forName:';
    procedure removePersistentDomainForName(domainName: NSString); message 'removePersistentDomainForName:';
    function synchronize: Boolean; message 'synchronize';
    function objectIsForcedForKey(key: NSString): Boolean; message 'objectIsForcedForKey:';
    function objectIsForcedForKey_inDomain(key: NSString; domain: NSString): Boolean; message 'objectIsForcedForKey:inDomain:';
  end;

{$endif}
{$endif}
