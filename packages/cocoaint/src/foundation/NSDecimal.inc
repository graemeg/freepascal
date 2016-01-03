{ Parsed from Foundation.framework NSDecimal.h }

{$ifdef TYPES}
{$ifndef NSDECIMAL_PAS_T}
{$define NSDECIMAL_PAS_T}

{ Constants }

const
  NSRoundPlain = 0;
  NSRoundDown = 1;
  NSRoundUp = 2;
  NSRoundBankers = 3;

const
  NSCalculationNoError = 0;
  NSCalculationLossOfPrecision = 1;
  NSCalculationUnderflow = 2;
  NSCalculationOverflow = 3;
  NSCalculationDivideByZero = 4;

{ Types }
type
  NSRoundingMode = NSUInteger;
  NSRoundingModePtr = ^NSRoundingMode;
  NSCalculationError = NSUInteger;
  NSCalculationErrorPtr = ^NSCalculationError;

{ Defines }
const
  NSDecimalMaxSize = 8;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSDECIMAL_PAS_R}
{$define NSDECIMAL_PAS_R}

{ Records }
type
  NSDecimal = record
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    case byte of
    0: (_anonbitfield_NSDecimal0: cint);
    1: (data: bitpacked record
          _exponent: 0..((1 shl 8)-1);
          _length: 0..((1 shl 4)-1);
          _isNegative: 0..1;
          _isCompact: 0..1;
          _reserved: 0..((1 shl 18)-1);
          _mantissa: array[0..(NSDecimalMaxSize)-1] of cushort;
         end;
       );
=======
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
    _reserved: 0..((1 shl 18)-1);
    _mantissa: cushort;
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
  end;
NSDecimalPtr = ^NSDecimal;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSDECIMAL_PAS_F}
{$define NSDECIMAL_PAS_F}

{ Functions }
procedure NSDecimalCopy(destination: NSDecimalPtr; source: NSDecimalPtr); cdecl; external;
procedure NSDecimalCompact(number: NSDecimalPtr); cdecl; external;
function NSDecimalCompare(leftOperand: NSDecimalPtr; rightOperand: NSDecimalPtr): NSComparisonResult; cdecl; external;
procedure NSDecimalRound(result_: NSDecimalPtr; number: NSDecimalPtr; scale: NSInteger; roundingMode: NSRoundingMode); cdecl; external;
function NSDecimalNormalize(number1: NSDecimalPtr; number2: NSDecimalPtr; roundingMode: NSRoundingMode): NSCalculationError; cdecl; external;
function NSDecimalAdd(result_: NSDecimalPtr; leftOperand: NSDecimalPtr; rightOperand: NSDecimalPtr; roundingMode: NSRoundingMode): NSCalculationError; cdecl; external;
function NSDecimalSubtract(result_: NSDecimalPtr; leftOperand: NSDecimalPtr; rightOperand: NSDecimalPtr; roundingMode: NSRoundingMode): NSCalculationError; cdecl; external;
function NSDecimalMultiply(result_: NSDecimalPtr; leftOperand: NSDecimalPtr; rightOperand: NSDecimalPtr; roundingMode: NSRoundingMode): NSCalculationError; cdecl; external;
function NSDecimalDivide(result_: NSDecimalPtr; leftOperand: NSDecimalPtr; rightOperand: NSDecimalPtr; roundingMode: NSRoundingMode): NSCalculationError; cdecl; external;
function NSDecimalPower(result_: NSDecimalPtr; number: NSDecimalPtr; power: NSUInteger; roundingMode: NSRoundingMode): NSCalculationError; cdecl; external;
function NSDecimalMultiplyByPowerOf10(result_: NSDecimalPtr; number: NSDecimalPtr; power: cshort; roundingMode: NSRoundingMode): NSCalculationError; cdecl; external;
function NSDecimalString(dcm: NSDecimalPtr; locale: id): NSString; cdecl; external;

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSDECIMAL_PAS_S}
{$define NSDECIMAL_PAS_S}

{$endif}
{$endif}
