{ Parsed from Foundation.framework NSNumberFormatter.h }

{$ifdef TYPES}
{$ifndef NSNUMBERFORMATTER_PAS_T}
{$define NSNUMBERFORMATTER_PAS_T}

{ Constants }

const
  NSNumberFormatterNoStyle = kCFNumberFormatterNoStyle;
  NSNumberFormatterDecimalStyle = kCFNumberFormatterDecimalStyle;
  NSNumberFormatterCurrencyStyle = kCFNumberFormatterCurrencyStyle;
  NSNumberFormatterPercentStyle = kCFNumberFormatterPercentStyle;
  NSNumberFormatterScientificStyle = kCFNumberFormatterScientificStyle;
  NSNumberFormatterSpellOutStyle = kCFNumberFormatterSpellOutStyle;

const
  NSNumberFormatterBehaviorDefault = 0;
  NSNumberFormatterBehavior10_0 = 1000;
  NSNumberFormatterBehavior10_4 = 1040;

const
  NSNumberFormatterPadBeforePrefix = kCFNumberFormatterPadBeforePrefix;
  NSNumberFormatterPadAfterPrefix = kCFNumberFormatterPadAfterPrefix;
  NSNumberFormatterPadBeforeSuffix = kCFNumberFormatterPadBeforeSuffix;
  NSNumberFormatterPadAfterSuffix = kCFNumberFormatterPadAfterSuffix;

const
  NSNumberFormatterRoundCeiling = kCFNumberFormatterRoundCeiling;
  NSNumberFormatterRoundFloor = kCFNumberFormatterRoundFloor;
  NSNumberFormatterRoundDown = kCFNumberFormatterRoundDown;
  NSNumberFormatterRoundUp = kCFNumberFormatterRoundUp;
  NSNumberFormatterRoundHalfEven = kCFNumberFormatterRoundHalfEven;
  NSNumberFormatterRoundHalfDown = kCFNumberFormatterRoundHalfDown;
  NSNumberFormatterRoundHalfUp = kCFNumberFormatterRoundHalfUp;

{ Types }
type
  NSNumberFormatterStyle = NSUInteger;
  NSNumberFormatterStylePtr = ^NSNumberFormatterStyle;
  NSNumberFormatterBehavior = NSUInteger;
  NSNumberFormatterBehaviorPtr = ^NSNumberFormatterBehavior;
  NSNumberFormatterPadPosition = NSUInteger;
  NSNumberFormatterPadPositionPtr = ^NSNumberFormatterPadPosition;
  NSNumberFormatterRoundingMode = NSUInteger;
  NSNumberFormatterRoundingModePtr = ^NSNumberFormatterRoundingMode;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSNUMBERFORMATTER_PAS_R}
{$define NSNUMBERFORMATTER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSNUMBERFORMATTER_PAS_F}
{$define NSNUMBERFORMATTER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSNUMBERFORMATTER_PAS_S}
{$define NSNUMBERFORMATTER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSNumberFormatter = objcclass;
  NSNumberFormatterPointer = ^NSNumberFormatter;
  NSNumberFormatterPtr = NSNumberFormatterPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSNUMBERFORMATTER_PAS_C}
{$define NSNUMBERFORMATTER_PAS_C}

{ NSNumberFormatter }
  NSNumberFormatter = objcclass external (NSFormatter)
  private
    _attributes: NSMutableDictionary;
    _formatter: CFNumberFormatterRef; {garbage collector: __strong }
    _counter: NSUInteger;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(12)-1] of Pointer;
=======
        _reserved: array[0..11] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..11] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..11] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..11] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    function getObjectValue_forString_range_error(obj: idPtr; string_: NSString; rangep: NSRangePtr; error: NSErrorPointer): Boolean; message 'getObjectValue:forString:range:error:';
    function stringFromNumber(number: NSNumber): NSString; message 'stringFromNumber:';
    function numberFromString(string_: NSString): NSNumber; message 'numberFromString:';
    class function localizedStringFromNumber_numberStyle(num: NSNumber; nstyle: NSNumberFormatterStyle): NSString; message 'localizedStringFromNumber:numberStyle:';
    function numberStyle: NSNumberFormatterStyle; message 'numberStyle';
    procedure setNumberStyle(style: NSNumberFormatterStyle); message 'setNumberStyle:';
    function locale: NSLocale; message 'locale';
    procedure setLocale(locale_: NSLocale); message 'setLocale:';
    function generatesDecimalNumbers: Boolean; message 'generatesDecimalNumbers';
    procedure setGeneratesDecimalNumbers(b: Boolean); message 'setGeneratesDecimalNumbers:';
    function formatterBehavior: NSNumberFormatterBehavior; message 'formatterBehavior';
    procedure setFormatterBehavior(behavior: NSNumberFormatterBehavior); message 'setFormatterBehavior:';
    class function defaultFormatterBehavior: NSNumberFormatterBehavior; message 'defaultFormatterBehavior';
    class procedure setDefaultFormatterBehavior(behavior: NSNumberFormatterBehavior); message 'setDefaultFormatterBehavior:';
    function negativeFormat: NSString; message 'negativeFormat';
    procedure setNegativeFormat(format_: NSString); message 'setNegativeFormat:';
    function textAttributesForNegativeValues: NSDictionary; message 'textAttributesForNegativeValues';
    procedure setTextAttributesForNegativeValues(newAttributes: NSDictionary); message 'setTextAttributesForNegativeValues:';
    function positiveFormat: NSString; message 'positiveFormat';
    procedure setPositiveFormat(format_: NSString); message 'setPositiveFormat:';
    function textAttributesForPositiveValues: NSDictionary; message 'textAttributesForPositiveValues';
    procedure setTextAttributesForPositiveValues(newAttributes: NSDictionary); message 'setTextAttributesForPositiveValues:';
    function allowsFloats: Boolean; message 'allowsFloats';
    procedure setAllowsFloats(flag: Boolean); message 'setAllowsFloats:';
    function decimalSeparator: NSString; message 'decimalSeparator';
    procedure setDecimalSeparator(string_: NSString); message 'setDecimalSeparator:';
    function alwaysShowsDecimalSeparator: Boolean; message 'alwaysShowsDecimalSeparator';
    procedure setAlwaysShowsDecimalSeparator(b: Boolean); message 'setAlwaysShowsDecimalSeparator:';
    function currencyDecimalSeparator: NSString; message 'currencyDecimalSeparator';
    procedure setCurrencyDecimalSeparator(string_: NSString); message 'setCurrencyDecimalSeparator:';
    function usesGroupingSeparator: Boolean; message 'usesGroupingSeparator';
    procedure setUsesGroupingSeparator(b: Boolean); message 'setUsesGroupingSeparator:';
    function groupingSeparator: NSString; message 'groupingSeparator';
    procedure setGroupingSeparator(string_: NSString); message 'setGroupingSeparator:';
    function zeroSymbol: NSString; message 'zeroSymbol';
    procedure setZeroSymbol(string_: NSString); message 'setZeroSymbol:';
    function textAttributesForZero: NSDictionary; message 'textAttributesForZero';
    procedure setTextAttributesForZero(newAttributes: NSDictionary); message 'setTextAttributesForZero:';
    function nilSymbol: NSString; message 'nilSymbol';
    procedure setNilSymbol(string_: NSString); message 'setNilSymbol:';
    function textAttributesForNil: NSDictionary; message 'textAttributesForNil';
    procedure setTextAttributesForNil(newAttributes: NSDictionary); message 'setTextAttributesForNil:';
    function notANumberSymbol: NSString; message 'notANumberSymbol';
    procedure setNotANumberSymbol(string_: NSString); message 'setNotANumberSymbol:';
    function textAttributesForNotANumber: NSDictionary; message 'textAttributesForNotANumber';
    procedure setTextAttributesForNotANumber(newAttributes: NSDictionary); message 'setTextAttributesForNotANumber:';
    function positiveInfinitySymbol: NSString; message 'positiveInfinitySymbol';
    procedure setPositiveInfinitySymbol(string_: NSString); message 'setPositiveInfinitySymbol:';
    function textAttributesForPositiveInfinity: NSDictionary; message 'textAttributesForPositiveInfinity';
    procedure setTextAttributesForPositiveInfinity(newAttributes: NSDictionary); message 'setTextAttributesForPositiveInfinity:';
    function negativeInfinitySymbol: NSString; message 'negativeInfinitySymbol';
    procedure setNegativeInfinitySymbol(string_: NSString); message 'setNegativeInfinitySymbol:';
    function textAttributesForNegativeInfinity: NSDictionary; message 'textAttributesForNegativeInfinity';
    procedure setTextAttributesForNegativeInfinity(newAttributes: NSDictionary); message 'setTextAttributesForNegativeInfinity:';
    function positivePrefix: NSString; message 'positivePrefix';
    procedure setPositivePrefix(string_: NSString); message 'setPositivePrefix:';
    function positiveSuffix: NSString; message 'positiveSuffix';
    procedure setPositiveSuffix(string_: NSString); message 'setPositiveSuffix:';
    function negativePrefix: NSString; message 'negativePrefix';
    procedure setNegativePrefix(string_: NSString); message 'setNegativePrefix:';
    function negativeSuffix: NSString; message 'negativeSuffix';
    procedure setNegativeSuffix(string_: NSString); message 'setNegativeSuffix:';
    function currencyCode: NSString; message 'currencyCode';
    procedure setCurrencyCode(string_: NSString); message 'setCurrencyCode:';
    function currencySymbol: NSString; message 'currencySymbol';
    procedure setCurrencySymbol(string_: NSString); message 'setCurrencySymbol:';
    function internationalCurrencySymbol: NSString; message 'internationalCurrencySymbol';
    procedure setInternationalCurrencySymbol(string_: NSString); message 'setInternationalCurrencySymbol:';
    function percentSymbol: NSString; message 'percentSymbol';
    procedure setPercentSymbol(string_: NSString); message 'setPercentSymbol:';
    function perMillSymbol: NSString; message 'perMillSymbol';
    procedure setPerMillSymbol(string_: NSString); message 'setPerMillSymbol:';
    function minusSign: NSString; message 'minusSign';
    procedure setMinusSign(string_: NSString); message 'setMinusSign:';
    function plusSign: NSString; message 'plusSign';
    procedure setPlusSign(string_: NSString); message 'setPlusSign:';
    function exponentSymbol: NSString; message 'exponentSymbol';
    procedure setExponentSymbol(string_: NSString); message 'setExponentSymbol:';
    function groupingSize: NSUInteger; message 'groupingSize';
    procedure setGroupingSize(number: NSUInteger); message 'setGroupingSize:';
    function secondaryGroupingSize: NSUInteger; message 'secondaryGroupingSize';
    procedure setSecondaryGroupingSize(number: NSUInteger); message 'setSecondaryGroupingSize:';
    function multiplier: NSNumber; message 'multiplier';
    procedure setMultiplier(number: NSNumber); message 'setMultiplier:';
    function formatWidth: NSUInteger; message 'formatWidth';
    procedure setFormatWidth(number: NSUInteger); message 'setFormatWidth:';
    function paddingCharacter: NSString; message 'paddingCharacter';
    procedure setPaddingCharacter(string_: NSString); message 'setPaddingCharacter:';
    function paddingPosition: NSNumberFormatterPadPosition; message 'paddingPosition';
    procedure setPaddingPosition(position: NSNumberFormatterPadPosition); message 'setPaddingPosition:';
    function roundingMode: NSNumberFormatterRoundingMode; message 'roundingMode';
    procedure setRoundingMode(mode: NSNumberFormatterRoundingMode); message 'setRoundingMode:';
    function roundingIncrement: NSNumber; message 'roundingIncrement';
    procedure setRoundingIncrement(number: NSNumber); message 'setRoundingIncrement:';
    function minimumIntegerDigits: NSUInteger; message 'minimumIntegerDigits';
    procedure setMinimumIntegerDigits(number: NSUInteger); message 'setMinimumIntegerDigits:';
    function maximumIntegerDigits: NSUInteger; message 'maximumIntegerDigits';
    procedure setMaximumIntegerDigits(number: NSUInteger); message 'setMaximumIntegerDigits:';
    function minimumFractionDigits: NSUInteger; message 'minimumFractionDigits';
    procedure setMinimumFractionDigits(number: NSUInteger); message 'setMinimumFractionDigits:';
    function maximumFractionDigits: NSUInteger; message 'maximumFractionDigits';
    procedure setMaximumFractionDigits(number: NSUInteger); message 'setMaximumFractionDigits:';
    function minimum: NSNumber; message 'minimum';
    procedure setMinimum(number: NSNumber); message 'setMinimum:';
    function maximum: NSNumber; message 'maximum';
    procedure setMaximum(number: NSNumber); message 'setMaximum:';
    function currencyGroupingSeparator: NSString; message 'currencyGroupingSeparator';
    procedure setCurrencyGroupingSeparator(string_: NSString); message 'setCurrencyGroupingSeparator:';
    function isLenient: Boolean; message 'isLenient';
    procedure setLenient(b: Boolean); message 'setLenient:';
    function usesSignificantDigits: Boolean; message 'usesSignificantDigits';
    procedure setUsesSignificantDigits(b: Boolean); message 'setUsesSignificantDigits:';
    function minimumSignificantDigits: NSUInteger; message 'minimumSignificantDigits';
    procedure setMinimumSignificantDigits(number: NSUInteger); message 'setMinimumSignificantDigits:';
    function maximumSignificantDigits: NSUInteger; message 'maximumSignificantDigits';
    procedure setMaximumSignificantDigits(number: NSUInteger); message 'setMaximumSignificantDigits:';
    function isPartialStringValidationEnabled: Boolean; message 'isPartialStringValidationEnabled';
    procedure setPartialStringValidationEnabled(b: Boolean); message 'setPartialStringValidationEnabled:';
  end;

{ NSNumberFormatterCompatibilityCategory }
  NSNumberFormatterCompatibilityCategory = objccategory external (NSNumberFormatter)
    function hasThousandSeparators: Boolean; message 'hasThousandSeparators';
    procedure setHasThousandSeparators(flag: Boolean); message 'setHasThousandSeparators:';
    function thousandSeparator: NSString; message 'thousandSeparator';
    procedure setThousandSeparator(newSeparator: NSString); message 'setThousandSeparator:';
    function localizesFormat: Boolean; message 'localizesFormat';
    procedure setLocalizesFormat(flag: Boolean); message 'setLocalizesFormat:';
    function format: NSString; message 'format';
    procedure setFormat(string_: NSString); message 'setFormat:';
    function attributedStringForZero: NSAttributedString; message 'attributedStringForZero';
    procedure setAttributedStringForZero(newAttributedString: NSAttributedString); message 'setAttributedStringForZero:';
    function attributedStringForNil: NSAttributedString; message 'attributedStringForNil';
    procedure setAttributedStringForNil(newAttributedString: NSAttributedString); message 'setAttributedStringForNil:';
    function attributedStringForNotANumber: NSAttributedString; message 'attributedStringForNotANumber';
    procedure setAttributedStringForNotANumber(newAttributedString: NSAttributedString); message 'setAttributedStringForNotANumber:';
    function roundingBehavior: NSDecimalNumberHandler; message 'roundingBehavior';
    procedure setRoundingBehavior(newRoundingBehavior: NSDecimalNumberHandler); message 'setRoundingBehavior:';
  end;

{$endif}
{$endif}
