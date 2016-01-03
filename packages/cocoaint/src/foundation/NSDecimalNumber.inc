{ Parsed from Foundation.framework NSDecimalNumber.h }

{$ifdef TYPES}
{$ifndef NSDECIMALNUMBER_PAS_T}
{$define NSDECIMALNUMBER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSDECIMALNUMBER_PAS_R}
{$define NSDECIMALNUMBER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSDECIMALNUMBER_PAS_F}
{$define NSDECIMALNUMBER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSDECIMALNUMBER_PAS_S}
{$define NSDECIMALNUMBER_PAS_S}

{ External string constants }
var
  NSDecimalNumberExactnessException: NSString; cvar; external;
  NSDecimalNumberOverflowException: NSString; cvar; external;
  NSDecimalNumberUnderflowException: NSString; cvar; external;
  NSDecimalNumberDivideByZeroException: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSDecimalNumberBehaviorsProtocol = objcprotocol;
  NSDecimalNumber = objcclass;
  NSDecimalNumberPointer = ^NSDecimalNumber;
  NSDecimalNumberPtr = NSDecimalNumberPointer;
  NSDecimalNumberHandler = objcclass;
  NSDecimalNumberHandlerPointer = ^NSDecimalNumberHandler;
  NSDecimalNumberHandlerPtr = NSDecimalNumberHandlerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSDECIMALNUMBER_PAS_C}
{$define NSDECIMALNUMBER_PAS_C}

{ NSDecimalNumber }
  NSDecimalNumber = objcclass external (NSNumber)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _anoninternstruct_NSDecimalNumber0: record
      case byte of
       0: (_anonbitfield_NSDecimalNumber0: cint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    _anoninternstruct_NSDecimalNumber0: bitpacked record
      case byte of
       0: (_anonbitfield_NSDecimalNumber0: cint);
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
        _exponent: 0..((1 shl 8)-1);
        _length: 0..((1 shl 4)-1);
        _isNegative: 0..1;
        _isCompact: 0..1;
        _reserved: 0..1;
        _hasExternalRefCount: 0..1;
        _refs: 0..((1 shl 16)-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
      );
      end;
    _mantissa: record end; { array of 0 elements in C, does not allocate space }
=======
      );
      end;
    _mantissa: cushort;
>>>>>>> graemeg/cpstrnew
=======
      );
      end;
    _mantissa: cushort;
>>>>>>> graemeg/cpstrnew
=======
      );
      end;
    _mantissa: cushort;
>>>>>>> graemeg/cpstrnew
=======
      );
      end;
    _mantissa: cushort;
>>>>>>> origin/cpstrnew
    
  public
    function initWithMantissa_exponent_isNegative(mantissa: culonglong; exponent: cshort; flag: Boolean): id; message 'initWithMantissa:exponent:isNegative:';
    function initWithDecimal(dcm: NSDecimal): id; message 'initWithDecimal:';
    function initWithString(numberValue: NSString): id; message 'initWithString:';
    function initWithString_locale(numberValue: NSString; locale_: id): id; message 'initWithString:locale:';
    function descriptionWithLocale(locale_: id): NSString; message 'descriptionWithLocale:';
    function decimalValue: NSDecimal; message 'decimalValue';
    class function decimalNumberWithMantissa_exponent_isNegative(mantissa: culonglong; exponent: cshort; flag: Boolean): NSDecimalNumber; message 'decimalNumberWithMantissa:exponent:isNegative:';
    class function decimalNumberWithDecimal(dcm: NSDecimal): NSDecimalNumber; message 'decimalNumberWithDecimal:';
    class function decimalNumberWithString(numberValue: NSString): NSDecimalNumber; message 'decimalNumberWithString:';
    class function decimalNumberWithString_locale(numberValue: NSString; locale_: id): NSDecimalNumber; message 'decimalNumberWithString:locale:';
    class function zero: NSDecimalNumber; message 'zero';
    class function one: NSDecimalNumber; message 'one';
    class function minimumDecimalNumber: NSDecimalNumber; message 'minimumDecimalNumber';
    class function maximumDecimalNumber: NSDecimalNumber; message 'maximumDecimalNumber';
    class function notANumber: NSDecimalNumber; message 'notANumber';
    function decimalNumberByAdding(decimalNumber: NSDecimalNumber): NSDecimalNumber; message 'decimalNumberByAdding:';
    function decimalNumberByAdding_withBehavior(decimalNumber: NSDecimalNumber; behavior: NSDecimalNumberBehaviorsProtocol): NSDecimalNumber; message 'decimalNumberByAdding:withBehavior:';
    function decimalNumberBySubtracting(decimalNumber: NSDecimalNumber): NSDecimalNumber; message 'decimalNumberBySubtracting:';
    function decimalNumberBySubtracting_withBehavior(decimalNumber: NSDecimalNumber; behavior: NSDecimalNumberBehaviorsProtocol): NSDecimalNumber; message 'decimalNumberBySubtracting:withBehavior:';
    function decimalNumberByMultiplyingBy(decimalNumber: NSDecimalNumber): NSDecimalNumber; message 'decimalNumberByMultiplyingBy:';
    function decimalNumberByMultiplyingBy_withBehavior(decimalNumber: NSDecimalNumber; behavior: NSDecimalNumberBehaviorsProtocol): NSDecimalNumber; message 'decimalNumberByMultiplyingBy:withBehavior:';
    function decimalNumberByDividingBy(decimalNumber: NSDecimalNumber): NSDecimalNumber; message 'decimalNumberByDividingBy:';
    function decimalNumberByDividingBy_withBehavior(decimalNumber: NSDecimalNumber; behavior: NSDecimalNumberBehaviorsProtocol): NSDecimalNumber; message 'decimalNumberByDividingBy:withBehavior:';
    function decimalNumberByRaisingToPower(power: NSUInteger): NSDecimalNumber; message 'decimalNumberByRaisingToPower:';
    function decimalNumberByRaisingToPower_withBehavior(power: NSUInteger; behavior: NSDecimalNumberBehaviorsProtocol): NSDecimalNumber; message 'decimalNumberByRaisingToPower:withBehavior:';
    function decimalNumberByMultiplyingByPowerOf10(power: cshort): NSDecimalNumber; message 'decimalNumberByMultiplyingByPowerOf10:';
    function decimalNumberByMultiplyingByPowerOf10_withBehavior(power: cshort; behavior: NSDecimalNumberBehaviorsProtocol): NSDecimalNumber; message 'decimalNumberByMultiplyingByPowerOf10:withBehavior:';
    function decimalNumberByRoundingAccordingToBehavior(behavior: NSDecimalNumberBehaviorsProtocol): NSDecimalNumber; message 'decimalNumberByRoundingAccordingToBehavior:';
    function compare(decimalNumber: NSNumber): NSComparisonResult; message 'compare:';
    class procedure setDefaultBehavior(behavior: NSDecimalNumberBehaviorsProtocol); message 'setDefaultBehavior:';
    class function defaultBehavior: NSDecimalNumberBehaviorsProtocol; message 'defaultBehavior';
    function objCType: PChar; message 'objCType';
    function doubleValue: double; message 'doubleValue';
  end;

{ NSDecimalNumberHandler }
  NSDecimalNumberHandler = objcclass external (NSObject, NSDecimalNumberBehaviorsProtocol, NSCodingProtocol)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _anoninternstruct_NSDecimalNumber1: record
      case byte of
       0: (_anonbitfield_NSDecimalNumber1: cint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    _anoninternstruct_NSDecimalNumber1: bitpacked record
      case byte of
       0: (_anonbitfield_NSDecimalNumber1: cint);
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
        _scale: 0..((1 shl 16)-1);
        _roundingMode: 0..((1 shl 3)-1);
        _raiseOnExactness: 0..1;
        _raiseOnOverflow: 0..1;
        _raiseOnUnderflow: 0..1;
        _raiseOnDivideByZero: 0..1;
        _unused: 0..((1 shl 9)-1);
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
    _reserved2: Pointer;
    _reserved: Pointer;
    
  public
    class function defaultDecimalNumberHandler: id; message 'defaultDecimalNumberHandler';
    function initWithRoundingMode_scale_raiseOnExactness_raiseOnOverflow_raiseOnUnderflow_raiseOnDivideByZero(roundingMode: NSRoundingMode; scale: cshort; exact: Boolean; overflow: Boolean; underflow: Boolean; divideByZero: Boolean): id; message 'initWithRoundingMode:scale:raiseOnExactness:raiseOnOverflow:raiseOnUnderflow:raiseOnDivideByZero:';
    class function decimalNumberHandlerWithRoundingMode_scale_raiseOnExactness_raiseOnOverflow_raiseOnUnderflow_raiseOnDivideByZero(roundingMode: NSRoundingMode; scale: cshort; exact: Boolean; overflow: Boolean; underflow: Boolean; divideByZero: Boolean): id; message 'decimalNumberHandlerWithRoundingMode:scale:raiseOnExactness:raiseOnOverflow:raiseOnUnderflow:raiseOnDivideByZero:';

    { Adopted Protocols }
    function roundingMode: NSRoundingMode;
    function scale: cshort;
    function exceptionDuringOperation_error_leftOperand_rightOperand(operation: SEL; error: NSCalculationError; leftOperand: NSDecimalNumber; rightOperand: NSDecimalNumber): NSDecimalNumber;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSDecimalNumberExtensionsCategory }
  NSDecimalNumberExtensionsCategory = objccategory external (NSNumber)
    function decimalValue: NSDecimal; message 'decimalValue';
  end;

{ NSDecimalNumberScanningCategory }
  NSDecimalNumberScanningCategory = objccategory external (NSScanner)
    function scanDecimal(dcm: NSDecimalPtr): Boolean; message 'scanDecimal:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSDECIMALNUMBER_PAS_P}
{$define NSDECIMALNUMBER_PAS_P}
  
{ NSDecimalNumberBehaviors Protocol }
  NSDecimalNumberBehaviorsProtocol = objcprotocol external name 'NSDecimalNumberBehaviors'
    function roundingMode: NSRoundingMode; message 'roundingMode';
    function scale: cshort; message 'scale';
    function exceptionDuringOperation_error_leftOperand_rightOperand(operation: SEL; error: NSCalculationError; leftOperand: NSDecimalNumber; rightOperand: NSDecimalNumber): NSDecimalNumber; message 'exceptionDuringOperation:error:leftOperand:rightOperand:';
  end;
{$endif}
{$endif}
