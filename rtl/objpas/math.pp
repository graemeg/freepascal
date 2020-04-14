{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{-------------------------------------------------------------------------
 Using functions from AMath/DAMath libraries, which are covered by the
 following license:

 (C) Copyright 2009-2013 Wolfgang Ehrhardt

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from
 the use of this software.

 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it
 freely, subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software in
    a product, an acknowledgment in the product documentation would be
    appreciated but is not required.

 2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.

 3. This notice may not be removed or altered from any source distribution.
----------------------------------------------------------------------------}
{
  This unit is an equivalent to the Delphi Math unit
  (with some improvements)

  What's to do:
    o some statistical functions
    o optimizations
}

{$MODE objfpc}
{$inline on }
{$GOTO on}
unit Math;
interface


{$ifndef FPUNONE}
    uses
       sysutils;

{$IFDEF FPDOC_MATH}
{$DEFINE FPC_HAS_TYPE_SINGLE}
{$DEFINE FPC_HAS_TYPE_DOUBLE}
{$DEFINE FPC_HAS_TYPE_EXTENDED}
{$DEFINE FPC_HAS_TYPE_COMP}
Type
  Float = MaxFloatType;

Const
  MinFloat = 0;
  MaxFloat = 0;
{$ENDIF}

    { Ranges of the IEEE floating point types, including denormals }
{$ifdef FPC_HAS_TYPE_SINGLE}
    const
      { values according to
        https://en.wikipedia.org/wiki/Single-precision_floating-point_format#Single-precision_examples
      }
      MinSingle    =  1.1754943508e-38;
      MaxSingle    =  3.4028234664e+38;
{$endif FPC_HAS_TYPE_SINGLE}
{$ifdef FPC_HAS_TYPE_DOUBLE}
    const
      { values according to
        https://en.wikipedia.org/wiki/Double-precision_floating-point_format#Double-precision_examples
      }
      MinDouble    =  2.2250738585072014e-308;
      MaxDouble    =  1.7976931348623157e+308;
{$endif FPC_HAS_TYPE_DOUBLE}
{$ifdef FPC_HAS_TYPE_EXTENDED}
    const
      MinExtended  =  3.4e-4932;
      MaxExtended  =  1.1e+4932;
{$endif FPC_HAS_TYPE_EXTENDED}
{$ifdef FPC_HAS_TYPE_COMP}
    const
      MinComp      = -9.223372036854775807e+18;
      MaxComp      =  9.223372036854775807e+18;
{$endif FPC_HAS_TYPE_COMP}

       { the original delphi functions use extended as argument, }
       { but I would prefer double, because 8 bytes is a very    }
       { natural size for the processor                          }
       { WARNING : changing float type will                      }
       { break all assembler code  PM                            }
{$if defined(FPC_HAS_TYPE_FLOAT128)}
      type
         Float = Float128;

      const
         MinFloat = MinFloat128;
         MaxFloat = MaxFloat128;
{$elseif defined(FPC_HAS_TYPE_EXTENDED)}
      type
         Float = extended;

      const
         MinFloat = MinExtended;
         MaxFloat = MaxExtended;
{$elseif defined(FPC_HAS_TYPE_DOUBLE)}
      type
         Float = double;

      const
         MinFloat = MinDouble;
         MaxFloat = MaxDouble;
{$elseif defined(FPC_HAS_TYPE_SINGLE)}
      type
         Float = single;

      const
         MinFloat = MinSingle;
         MaxFloat = MaxSingle;
{$else}
        {$fatal At least one floating point type must be supported}
{$endif}

    type
       PFloat = ^Float;
       PInteger = ObjPas.PInteger;

       TPaymentTime = (ptEndOfPeriod,ptStartOfPeriod);

       EInvalidArgument = class(ematherror);

       TValueRelationship = -1..1;

    const
       EqualsValue = 0;
       LessThanValue = Low(TValueRelationship);
       GreaterThanValue = High(TValueRelationship);
       

       
{$push}
{$R-}
{$Q-}
       NaN = 0.0/0.0;
       Infinity = 1.0/0.0;
       NegInfinity = -1.0/0.0;
{$pop}

{ Min/max determination }
function MinIntValue(const Data: array of Integer): Integer;
function MaxIntValue(const Data: array of Integer): Integer;

{ Extra, not present in Delphi, but used frequently  }
function Min(a, b: Integer): Integer;inline; overload;
function Max(a, b: Integer): Integer;inline; overload;
{ this causes more trouble than it solves
function Min(a, b: Cardinal): Cardinal; overload;
function Max(a, b: Cardinal): Cardinal; overload;
}
function Min(a, b: Int64): Int64;inline; overload;
function Max(a, b: Int64): Int64;inline; overload;
function Min(a, b: QWord): QWord;inline; overload;
function Max(a, b: QWord): QWord;inline; overload;
{$ifdef FPC_HAS_TYPE_SINGLE}
function Min(a, b: Single): Single;inline; overload;
function Max(a, b: Single): Single;inline; overload;
{$endif FPC_HAS_TYPE_SINGLE}
{$ifdef FPC_HAS_TYPE_DOUBLE}
function Min(a, b: Double): Double;inline; overload;
function Max(a, b: Double): Double;inline; overload;
{$endif FPC_HAS_TYPE_DOUBLE}
{$ifdef FPC_HAS_TYPE_EXTENDED}
function Min(a, b: Extended): Extended;inline; overload;
function Max(a, b: Extended): Extended;inline; overload;
{$endif FPC_HAS_TYPE_EXTENDED}

function InRange(const AValue, AMin, AMax: Integer): Boolean;inline; overload;
function InRange(const AValue, AMin, AMax: Int64): Boolean;inline; overload;
{$ifdef FPC_HAS_TYPE_DOUBLE}
function InRange(const AValue, AMin, AMax: Double): Boolean;inline;  overload;
{$endif FPC_HAS_TYPE_DOUBLE}

function EnsureRange(const AValue, AMin, AMax: Integer): Integer;inline;  overload;
function EnsureRange(const AValue, AMin, AMax: Int64): Int64;inline;  overload;
{$ifdef FPC_HAS_TYPE_DOUBLE}
function EnsureRange(const AValue, AMin, AMax: Double): Double;inline;  overload;
{$endif FPC_HAS_TYPE_DOUBLE}


procedure DivMod(Dividend: LongInt; Divisor: Word;  var Result, Remainder: Word);
procedure DivMod(Dividend: LongInt; Divisor: Word; var Result, Remainder: SmallInt);
procedure DivMod(Dividend: DWord; Divisor: DWord; var Result, Remainder: DWord);
procedure DivMod(Dividend: LongInt; Divisor: LongInt; var Result, Remainder: LongInt);

{ Floating point modulo}
{$ifdef FPC_HAS_TYPE_SINGLE}
function FMod(const a, b: Single): Single;inline;overload;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function FMod(const a, b: Double): Double;inline;overload;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function FMod(const a, b: Extended): Extended;inline;overload;
{$endif FPC_HAS_TYPE_EXTENDED}

operator mod(const a,b:float) c:float;inline;

// Sign functions
Type
  TValueSign = -1..1;

const
  NegativeValue = Low(TValueSign);
  ZeroValue = 0;
  PositiveValue = High(TValueSign);

function Sign(const AValue: Integer): TValueSign;inline; overload;
function Sign(const AValue: Int64): TValueSign;inline; overload;
{$ifdef FPC_HAS_TYPE_SINGLE}
function Sign(const AValue: Single): TValueSign;inline; overload;
{$endif}
function Sign(const AValue: Double): TValueSign;inline; overload;
{$ifdef FPC_HAS_TYPE_EXTENDED}
function Sign(const AValue: Extended): TValueSign;inline; overload;
{$endif}

function IsZero(const A: Single; Epsilon: Single): Boolean; overload;
function IsZero(const A: Single): Boolean;inline; overload;
{$ifdef FPC_HAS_TYPE_DOUBLE}
function IsZero(const A: Double; Epsilon: Double): Boolean; overload;
function IsZero(const A: Double): Boolean;inline; overload;
{$endif FPC_HAS_TYPE_DOUBLE}
{$ifdef FPC_HAS_TYPE_EXTENDED}
function IsZero(const A: Extended; Epsilon: Extended): Boolean; overload;
function IsZero(const A: Extended): Boolean;inline; overload;
{$endif FPC_HAS_TYPE_EXTENDED}

function IsNan(const d : Single): Boolean; overload;
{$ifdef FPC_HAS_TYPE_DOUBLE}
function IsNan(const d : Double): Boolean; overload;
{$endif FPC_HAS_TYPE_DOUBLE}
{$ifdef FPC_HAS_TYPE_EXTENDED}
function IsNan(const d : Extended): Boolean; overload;
{$endif FPC_HAS_TYPE_EXTENDED}

function IsInfinite(const d : Single): Boolean; overload;
{$ifdef FPC_HAS_TYPE_DOUBLE}
function IsInfinite(const d : Double): Boolean; overload;
{$endif FPC_HAS_TYPE_DOUBLE}
{$ifdef FPC_HAS_TYPE_EXTENDED}
function IsInfinite(const d : Extended): Boolean; overload;
{$endif FPC_HAS_TYPE_EXTENDED}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function SameValue(const A, B: Extended): Boolean;inline; overload;
{$endif}
{$ifdef FPC_HAS_TYPE_DOUBLE}
function SameValue(const A, B: Double): Boolean;inline; overload;
{$endif}
function SameValue(const A, B: Single): Boolean;inline; overload;
{$ifdef FPC_HAS_TYPE_EXTENDED}
function SameValue(const A, B: Extended; Epsilon: Extended): Boolean; overload;
{$endif}
{$ifdef FPC_HAS_TYPE_DOUBLE}
function SameValue(const A, B: Double; Epsilon: Double): Boolean; overload;
{$endif}
function SameValue(const A, B: Single; Epsilon: Single): Boolean; overload;

type
  TRoundToRange = -37..37;

{$ifdef FPC_HAS_TYPE_DOUBLE}
function RoundTo(const AValue: Double; const Digits: TRoundToRange): Double;
{$endif}
{$ifdef FPC_HAS_TYPE_EXTENDED}
function RoundTo(const AVAlue: Extended; const Digits: TRoundToRange): Extended;
{$endif}
{$ifdef FPC_HAS_TYPE_SINGLE}
function RoundTo(const AValue: Single; const Digits: TRoundToRange): Single;
{$endif}
{$ifdef FPC_HAS_TYPE_SINGLE}
function SimpleRoundTo(const AValue: Single; const Digits: TRoundToRange = -2): Single;
{$endif}
{$ifdef FPC_HAS_TYPE_DOUBLE}
function SimpleRoundTo(const AValue: Double; const Digits: TRoundToRange = -2): Double;
{$endif}
{$ifdef FPC_HAS_TYPE_EXTENDED}
function SimpleRoundTo(const AValue: Extended; const Digits: TRoundToRange = -2): Extended;
{$endif}


{ angle conversion }

function DegToRad(deg : float) : float;inline;
function RadToDeg(rad : float) : float;inline;
function GradToRad(grad : float) : float;inline;
function RadToGrad(rad : float) : float;inline;
function DegToGrad(deg : float) : float;inline;
function GradToDeg(grad : float) : float;inline;
{ one cycle are 2*Pi rad }
function CycleToRad(cycle : float) : float;inline;
function RadToCycle(rad : float) : float;inline;
{$ifdef FPC_HAS_TYPE_SINGLE}
Function DegNormalize(deg : single) : single; inline;
{$ENDIF}
{$ifdef FPC_HAS_TYPE_DOUBLE}
Function DegNormalize(deg : double) : double; inline;
{$ENDIF}
{$ifdef FPC_HAS_TYPE_EXTENDED}
Function DegNormalize(deg : extended) : extended; inline;
{$ENDIF}

{ trigoniometric functions }

function Tan(x : float) : float;
function Cotan(x : float) : float;
function Cot(x : float) : float; inline;
{$ifdef FPC_HAS_TYPE_SINGLE}
procedure SinCos(theta : single;out sinus,cosinus : single);
{$endif}
{$ifdef FPC_HAS_TYPE_DOUBLE}
procedure SinCos(theta : double;out sinus,cosinus : double);
{$endif}
{$ifdef FPC_HAS_TYPE_EXTENDED}
procedure SinCos(theta : extended;out sinus,cosinus : extended);
{$endif}


function Secant(x : float) : float; inline;
function Cosecant(x : float) : float; inline;
function Sec(x : float) : float; inline;
function Csc(x : float) : float; inline;

{ inverse functions }

function ArcCos(x : float) : float;
function ArcSin(x : float) : float;

{ calculates arctan(y/x) and returns an angle in the correct quadrant }
function ArcTan2(y,x : float) : float;

{ hyperbolic functions }

function CosH(x : float) : float;
function SinH(x : float) : float;
function TanH(x : float) : float;

{ area functions }

{ delphi names: }
function ArcCosH(x : float) : float;inline;
function ArcSinH(x : float) : float;inline;
function ArcTanH(x : float) : float;inline;
{ IMHO the function should be called as follows (FK) }
function ArCosH(x : float) : float;
function ArSinH(x : float) : float;
function ArTanH(x : float) : float;

{ triangle functions }

{ returns the length of the hypotenuse of a right triangle }
{ if x and y are the other sides                           }
function Hypot(x,y : float) : float;

{ logarithm functions }

function Log10(x : float) : float;
function Log2(x : float) : float;
function LogN(n,x : float) : float;

{ returns natural logarithm of x+1, accurate for x values near zero }
function LnXP1(x : float) : float;

{ exponential functions }

function Power(base,exponent : float) : float;
{ base^exponent }
function IntPower(base : float;const exponent : Integer) : float;

operator ** (bas,expo : float) e: float; inline;
operator ** (bas,expo : int64) i: int64; inline;

{ number converting }

{ rounds x towards positive infinity }
function Ceil(x : float) : Integer;
function Ceil64(x: float): Int64;
{ rounds x towards negative infinity }
function Floor(x : float) : Integer;
function Floor64(x: float): Int64;

{ misc. functions }

{ splits x into mantissa and exponent (to base 2) }
procedure Frexp(X: float; var Mantissa: float; var Exponent: integer);
{ returns x*(2^p) }
function Ldexp(x : float; const p : Integer) : float;

{ statistical functions }

{$ifdef FPC_HAS_TYPE_SINGLE}
function Mean(const data : array of Single) : float;
function Sum(const data : array of Single) : float;inline;
function Mean(const data : PSingle; Const N : longint) : float;
function Sum(const data : PSingle; Const N : Longint) : float;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function Mean(const data : array of double) : float;inline;
function Sum(const data : array of double) : float;inline;
function Mean(const data : PDouble; Const N : longint) : float;
function Sum(const data : PDouble; Const N : Longint) : float;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function Mean(const data : array of Extended) : float;
function Sum(const data : array of Extended) : float;inline;
function Mean(const data : PExtended; Const N : longint) : float;
function Sum(const data : PExtended; Const N : Longint) : float;
{$endif FPC_HAS_TYPE_EXTENDED}

function SumInt(const data : PInt64;Const N : longint) : Int64;
function SumInt(const data : array of Int64) : Int64;inline;
function Mean(const data : PInt64; const N : Longint):Float;
function Mean(const data: array of Int64):Float;
function SumInt(const data : PInteger; Const N : longint) : Int64;
function SumInt(const data : array of Integer) : Int64;inline;
function Mean(const data : PInteger; const N : Longint):Float;
function Mean(const data: array of Integer):Float;


{$ifdef FPC_HAS_TYPE_SINGLE}
function SumOfSquares(const data : array of Single) : float;inline;
function SumOfSquares(const data : PSingle; Const N : Integer) : float;
{ calculates the sum and the sum of squares of data }
procedure SumsAndSquares(const data : array of Single;
  var sum,sumofsquares : float);inline;
procedure SumsAndSquares(const data : PSingle; Const N : Integer;
  var sum,sumofsquares : float);
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function SumOfSquares(const data : array of double) : float;
function SumOfSquares(const data : PDouble; Const N : Integer) : float;
{ calculates the sum and the sum of squares of data }
procedure SumsAndSquares(const data : array of Double;
  var sum,sumofsquares : float);inline;
procedure SumsAndSquares(const data : PDouble; Const N : Integer;
  var sum,sumofsquares : float);
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function SumOfSquares(const data : array of Extended) : float;inline;
function SumOfSquares(const data : PExtended; Const N : Integer) : float;
{ calculates the sum and the sum of squares of data }
procedure SumsAndSquares(const data : array of Extended;
  var sum,sumofsquares : float);inline;
procedure SumsAndSquares(const data : PExtended; Const N : Integer;
  var sum,sumofsquares : float);
{$endif FPC_HAS_TYPE_EXTENDED}

{$ifdef FPC_HAS_TYPE_SINGLE}
function MinValue(const data : array of Single) : Single;inline;
function MinValue(const data : PSingle; Const N : Integer) : Single;
function MaxValue(const data : array of Single) : Single;inline;
function MaxValue(const data : PSingle; Const N : Integer) : Single;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function MinValue(const data : array of Double) : Double;inline;
function MinValue(const data : PDouble; Const N : Integer) : Double;
function MaxValue(const data : array of Double) : Double;inline;
function MaxValue(const data : PDouble; Const N : Integer) : Double;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function MinValue(const data : array of Extended) : Extended;inline;
function MinValue(const data : PExtended; Const N : Integer) : Extended;
function MaxValue(const data : array of Extended) : Extended;inline;
function MaxValue(const data : PExtended; Const N : Integer) : Extended;
{$endif FPC_HAS_TYPE_EXTENDED}

function MinValue(const data : array of integer) : Integer;inline;
function MinValue(const Data : PInteger; Const N : Integer): Integer;

function MaxValue(const data : array of integer) : Integer;inline;
function MaxValue(const data : PInteger; Const N : Integer) : Integer;

{ returns random values with gaussian distribution }
function RandG(mean,stddev : float) : float;
function RandomRange(const aFrom, aTo: Integer): Integer;
function RandomRange(const aFrom, aTo: Int64): Int64;

{$ifdef FPC_HAS_TYPE_SINGLE}
{ calculates the standard deviation }
function StdDev(const data : array of Single) : float;inline;
function StdDev(const data : PSingle; Const N : Integer) : float;
{ calculates the mean and stddev }
procedure MeanAndStdDev(const data : array of Single;
  var mean,stddev : float);inline;
procedure MeanAndStdDev(const data : PSingle;
  Const N : Longint;var mean,stddev : float);
function Variance(const data : array of Single) : float;inline;
function TotalVariance(const data : array of Single) : float;inline;
function Variance(const data : PSingle; Const N : Integer) : float;
function TotalVariance(const data : PSingle; Const N : Integer) : float;

{ Population (aka uncorrected) variance and standard deviation }
function PopnStdDev(const data : array of Single) : float;inline;
function PopnStdDev(const data : PSingle; Const N : Integer) : float;
function PopnVariance(const data : PSingle; Const N : Integer) : float;
function PopnVariance(const data : array of Single) : float;inline;
procedure MomentSkewKurtosis(const data : array of Single;
  out m1,m2,m3,m4,skew,kurtosis : float);inline;
procedure MomentSkewKurtosis(const data : PSingle; Const N : Integer;
  out m1,m2,m3,m4,skew,kurtosis : float);

{ geometrical function }

{ returns the euclidean L2 norm }
function Norm(const data : array of Single) : float;inline;
function Norm(const data : PSingle; Const N : Integer) : float;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
{ calculates the standard deviation }
function StdDev(const data : array of Double) : float;inline;
function StdDev(const data : PDouble; Const N : Integer) : float;
{ calculates the mean and stddev }
procedure MeanAndStdDev(const data : array of Double;
  var mean,stddev : float);inline;
procedure MeanAndStdDev(const data : PDouble;
  Const N : Longint;var mean,stddev : float);
function Variance(const data : array of Double) : float;inline;
function TotalVariance(const data : array of Double) : float;inline;
function Variance(const data : PDouble; Const N : Integer) : float;
function TotalVariance(const data : PDouble; Const N : Integer) : float;

{ Population (aka uncorrected) variance and standard deviation }
function PopnStdDev(const data : array of Double) : float;inline;
function PopnStdDev(const data : PDouble; Const N : Integer) : float;
function PopnVariance(const data : PDouble; Const N : Integer) : float;
function PopnVariance(const data : array of Double) : float;inline;
procedure MomentSkewKurtosis(const data : array of Double;
  out m1,m2,m3,m4,skew,kurtosis : float);inline;
procedure MomentSkewKurtosis(const data : PDouble; Const N : Integer;
  out m1,m2,m3,m4,skew,kurtosis : float);

{ geometrical function }

{ returns the euclidean L2 norm }
function Norm(const data : array of double) : float;inline;
function Norm(const data : PDouble; Const N : Integer) : float;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
{ calculates the standard deviation }
function StdDev(const data : array of Extended) : float;inline;
function StdDev(const data : PExtended; Const N : Integer) : float;
{ calculates the mean and stddev }
procedure MeanAndStdDev(const data : array of Extended;
  var mean,stddev : float);inline;
procedure MeanAndStdDev(const data : PExtended;
  Const N : Longint;var mean,stddev : float);
function Variance(const data : array of Extended) : float;inline;
function TotalVariance(const data : array of Extended) : float;inline;
function Variance(const data : PExtended; Const N : Integer) : float;
function TotalVariance(const data : PExtended; Const N : Integer) : float;

{ Population (aka uncorrected) variance and standard deviation }
function PopnStdDev(const data : array of Extended) : float;inline;
function PopnStdDev(const data : PExtended; Const N : Integer) : float;
function PopnVariance(const data : PExtended; Const N : Integer) : float;
function PopnVariance(const data : array of Extended) : float;inline;
procedure MomentSkewKurtosis(const data : array of Extended;
  out m1,m2,m3,m4,skew,kurtosis : float);inline;
procedure MomentSkewKurtosis(const data : PExtended; Const N : Integer;
  out m1,m2,m3,m4,skew,kurtosis : float);

{ geometrical function }

{ returns the euclidean L2 norm }
function Norm(const data : array of Extended) : float;inline;
function Norm(const data : PExtended; Const N : Integer) : float;
{$endif FPC_HAS_TYPE_EXTENDED}

{ Financial functions }

function FutureValue(ARate: Float; NPeriods: Integer;
  APayment, APresentValue: Float; APaymentTime: TPaymentTime): Float;

function InterestRate(NPeriods: Integer; APayment, APresentValue, AFutureValue: Float;
  APaymentTime: TPaymentTime): Float;

function NumberOfPeriods(ARate, APayment, APresentValue, AFutureValue: Float;
  APaymentTime: TPaymentTime): Float;

function Payment(ARate: Float; NPeriods: Integer;
  APresentValue, AFutureValue: Float; APaymentTime: TPaymentTime): Float;

function PresentValue(ARate: Float; NPeriods: Integer;
  APayment, AFutureValue: Float; APaymentTime: TPaymentTime): Float;

{ Misc functions }

function IfThen(val:boolean;const iftrue:integer; const iffalse:integer= 0) :integer; inline; overload;
function IfThen(val:boolean;const iftrue:int64  ; const iffalse:int64 = 0)  :int64;   inline; overload;
function IfThen(val:boolean;const iftrue:double ; const iffalse:double =0.0):double;  inline; overload;

function CompareValue ( const A, B  : Integer) : TValueRelationship; inline;
function CompareValue ( const A, B  : Int64) : TValueRelationship; inline;
function CompareValue ( const A, B  : QWord) : TValueRelationship; inline;

{$ifdef FPC_HAS_TYPE_SINGLE}
function CompareValue ( const A, B : Single; delta : Single = 0.0 ) : TValueRelationship; inline;
{$endif}
{$ifdef FPC_HAS_TYPE_DOUBLE}
function CompareValue ( const A, B : Double; delta : Double = 0.0) : TValueRelationship; inline;
{$endif}
{$ifdef FPC_HAS_TYPE_EXTENDED}
function CompareValue ( const A, B : Extended; delta : Extended = 0.0 ) : TValueRelationship; inline;
{$endif}

function RandomFrom(const AValues: array of Double): Double; overload;
function RandomFrom(const AValues: array of Integer): Integer; overload;
function RandomFrom(const AValues: array of Int64): Int64; overload;
{$if FPC_FULLVERSION >=30101}
generic function RandomFrom<T>(const AValues:array of T):T;
{$endif}

{ cpu specific stuff }

type
  TFPURoundingMode = system.TFPURoundingMode;
  TFPUPrecisionMode = system.TFPUPrecisionMode;
  TFPUException = system.TFPUException;
  TFPUExceptionMask = system.TFPUExceptionMask;

function GetRoundMode: TFPURoundingMode;
function SetRoundMode(const RoundMode: TFPURoundingMode): TFPURoundingMode;
function GetPrecisionMode: TFPUPrecisionMode;
function SetPrecisionMode(const Precision: TFPUPrecisionMode): TFPUPrecisionMode;
function GetExceptionMask: TFPUExceptionMask;
function SetExceptionMask(const Mask: TFPUExceptionMask): TFPUExceptionMask;
procedure ClearExceptions(RaisePending: Boolean =true);


implementation

function copysign(x,y: float): float; forward;    { returns abs(x)*sign(y) }

{ include cpu specific stuff }
{$i mathu.inc}

ResourceString
  SMathError = 'Math Error : %s';
  SInvalidArgument = 'Invalid argument';

Procedure DoMathError(Const S : String);
begin
  Raise EMathError.CreateFmt(SMathError,[S]);
end;

Procedure InvalidArgument;

begin
  Raise EInvalidArgument.Create(SInvalidArgument);
end;


function Sign(const AValue: Integer): TValueSign;inline;

begin
  result:=TValueSign(
    SarLongint(AValue,sizeof(AValue)*8-1) or            { gives -1 for negative values, 0 otherwise }
    (longint(-AValue) shr (sizeof(AValue)*8-1))         { gives 1 for positive values, 0 otherwise }
  );
end;

function Sign(const AValue: Int64): TValueSign;inline;

begin
{$ifdef cpu64}
  result:=TValueSign(
    SarInt64(AValue,sizeof(AValue)*8-1) or
    (-AValue shr (sizeof(AValue)*8-1))
  );
{$else cpu64}
  If Avalue<0 then
    Result:=NegativeValue
  else If Avalue>0 then
    Result:=PositiveValue
  else
    Result:=ZeroValue;
{$endif}
end;

{$ifdef FPC_HAS_TYPE_SINGLE}
function Sign(const AValue: Single): TValueSign;inline;

begin
  Result:=ord(AValue>0.0)-ord(AValue<0.0);
end;
{$endif}


function Sign(const AValue: Double): TValueSign;inline;

begin
  Result:=ord(AValue>0.0)-ord(AValue<0.0);
end;

{$ifdef FPC_HAS_TYPE_EXTENDED}
function Sign(const AValue: Extended): TValueSign;inline;

begin
  Result:=ord(AValue>0.0)-ord(AValue<0.0);
end;
{$endif}

function degtorad(deg : float) : float;inline;
  begin
     degtorad:=deg*(pi/180.0);
  end;

function radtodeg(rad : float) : float;inline;
  begin
     radtodeg:=rad*(180.0/pi);
  end;

function gradtorad(grad : float) : float;inline;
  begin
     gradtorad:=grad*(pi/200.0);
  end;

function radtograd(rad : float) : float;inline;
  begin
     radtograd:=rad*(200.0/pi);
  end;

function degtograd(deg : float) : float;inline;
  begin
     degtograd:=deg*(200.0/180.0);
  end;

function gradtodeg(grad : float) : float;inline;
  begin
     gradtodeg:=grad*(180.0/200.0);
  end;

function cycletorad(cycle : float) : float;inline;
  begin
     cycletorad:=(2*pi)*cycle;
  end;

function radtocycle(rad : float) : float;inline;
  begin
     { avoid division }
     radtocycle:=rad*(1/(2*pi));
  end;

{$ifdef FPC_HAS_TYPE_SINGLE}
Function DegNormalize(deg : single) : single;

begin
  Result:=Deg-Int(Deg/360)*360;
  If Result<0 then Result:=Result+360;
end;
{$ENDIF}
{$ifdef FPC_HAS_TYPE_DOUBLE}
Function DegNormalize(deg : double) : double; inline;

begin
  Result:=Deg-Int(Deg/360)*360;
  If (Result<0) then Result:=Result+360;
end;
{$ENDIF}
{$ifdef FPC_HAS_TYPE_EXTENDED}
Function DegNormalize(deg : extended) : extended; inline;

begin
  Result:=Deg-Int(Deg/360)*360;
  If Result<0 then Result:=Result+360;
end;
{$ENDIF}

{$ifndef FPC_MATH_HAS_TAN}
function tan(x : float) : float;
  var
    _sin,_cos : float;
  begin
    sincos(x,_sin,_cos);
    tan:=_sin/_cos;
  end;
{$endif FPC_MATH_HAS_TAN}


{$ifndef FPC_MATH_HAS_COTAN}
function cotan(x : float) : float;
  var
    _sin,_cos : float;
  begin
    sincos(x,_sin,_cos);
    cotan:=_cos/_sin;
  end;
{$endif FPC_MATH_HAS_COTAN}

function cot(x : float) : float; inline;
begin
  cot := cotan(x);
end;


{$ifndef FPC_MATH_HAS_SINCOS}
{$ifdef FPC_HAS_TYPE_SINGLE}
procedure sincos(theta : single;out sinus,cosinus : single);
  begin
    sinus:=sin(theta);
    cosinus:=cos(theta);
  end;
{$endif}


{$ifdef FPC_HAS_TYPE_DOUBLE}
procedure sincos(theta : double;out sinus,cosinus : double);
  begin
    sinus:=sin(theta);
    cosinus:=cos(theta);
  end;
{$endif}


{$ifdef FPC_HAS_TYPE_EXTENDED}
procedure sincos(theta : extended;out sinus,cosinus : extended);
  begin
    sinus:=sin(theta);
    cosinus:=cos(theta);
  end;
{$endif}
{$endif FPC_MATH_HAS_SINCOS}


function secant(x : float) : float; inline;
begin
  secant := 1 / cos(x);
end;


function cosecant(x : float) : float; inline;
begin
  cosecant := 1 / sin(x);
end;


function sec(x : float) : float; inline;
begin
  sec := secant(x);
end;


function csc(x : float) : float; inline;
begin
  csc := cosecant(x);
end;

{ arcsin and arccos functions from AMath library (C) Copyright 2009-2013 Wolfgang Ehrhardt }
function arcsin(x : float) : float;
begin
  arcsin:=arctan2(x,sqrt((1.0-x)*(1.0+x)));
end;

function Arccos(x : Float) : Float;
begin
  if abs(x)=1.0 then
    if x<0.0 then
      arccos:=Pi
    else
      arccos:=0
  else
    arccos:=arctan2(sqrt((1.0-x)*(1.0+x)),x);
end;


{$ifndef FPC_MATH_HAS_ARCTAN2}
function arctan2(y,x : float) : float;
  begin
    if (x=0) then
      begin
        if y=0 then
          arctan2:=0.0
        else if y>0 then
          arctan2:=pi/2
        else if y<0 then
          arctan2:=-pi/2;
      end
    else
      ArcTan2:=ArcTan(y/x);
    if x<0.0 then
      ArcTan2:=ArcTan2+pi;
    if ArcTan2>pi then
      ArcTan2:=ArcTan2-2*pi;
  end;
{$endif FPC_MATH_HAS_ARCTAN2}


function cosh(x : float) : float;
  var
     temp : float;
  begin
     temp:=exp(x);
     cosh:=0.5*(temp+1.0/temp);
  end;

function sinh(x : float) : float;
  var
     temp : float;
  begin
     temp:=exp(x);
     { copysign ensures that sinh(-0.0)=-0.0 }
     sinh:=copysign(0.5*(temp-1.0/temp),x);
  end;

Const MaxTanh = 5678.22249441322; // Ln(MaxExtended)/2

function tanh(x : float) : float;
  var Temp : float;
  begin
     if x>MaxTanh then exit(1.0)
     else if x<-MaxTanh then exit (-1.0);
     temp:=exp(-2*x);
     tanh:=(1-temp)/(1+temp)
  end;

function arccosh(x : float) : float; inline;
  begin
     arccosh:=arcosh(x);
  end;

function arcsinh(x : float) : float;inline;
  begin
     arcsinh:=arsinh(x);
  end;

function arctanh(x : float) : float;inline;
  begin
     arctanh:=artanh(x);
  end;

function arcosh(x : float) : float;
  begin
    { Provides accuracy about 4*eps near 1.0 }
    arcosh:=Ln(x+Sqrt((x-1.0)*(x+1.0)));
  end;

function arsinh(x : float) : float;
  var
    z: float;
  begin
    z:=abs(x);
    z:=Ln(z+Sqrt(1+z*z));
    { copysign ensures that arsinh(-Inf)=-Inf and arsinh(-0.0)=-0.0 }
    arsinh:=copysign(z,x);
  end;

function artanh(x : float) : float;
  begin
    artanh:=(lnxp1(x)-lnxp1(-x))*0.5;
  end;

{ hypot function from AMath library (C) Copyright 2009-2013 Wolfgang Ehrhardt }
function hypot(x,y : float) : float;
  begin
    x:=abs(x);
    y:=abs(y);
    if (x>y) then
      hypot:=x*sqrt(1.0+sqr(y/x))
    else if (x>0.0) then
      hypot:=y*sqrt(1.0+sqr(x/y))
    else
      hypot:=y;
  end;

function log10(x : float) : float;
  begin
    log10:=ln(x)*0.43429448190325182765;  { 1/ln(10) }
  end;

{$ifndef FPC_MATH_HAS_LOG2}
function log2(x : float) : float;
  begin
    log2:=ln(x)*1.4426950408889634079;    { 1/ln(2) }
  end;
{$endif FPC_MATH_HAS_LOG2}

function logn(n,x : float) : float;
  begin
     logn:=ln(x)/ln(n);
  end;

{ lnxp1 function from AMath library (C) Copyright 2009-2013 Wolfgang Ehrhardt }
function lnxp1(x : float) : float;
  var
    y: float;
  begin
    if (x>=4.0) then
      lnxp1:=ln(1.0+x)
    else
      begin
        y:=1.0+x;
        if (y=1.0) then
          lnxp1:=x
        else
          begin
            lnxp1:=ln(y);     { lnxp1(-1) = ln(0) = -Inf }
            if y>0.0 then
              lnxp1:=lnxp1+(x-(y-1.0))/y;
          end;
      end;
  end;


function power(base,exponent : float) : float;
  begin
    if Exponent=0.0 then
      result:=1.0
    else if (base=0.0) and (exponent>0.0) then
      result:=0.0
    else if (abs(exponent)<=maxint) and (frac(exponent)=0.0) then
      result:=intpower(base,trunc(exponent))
    else
      result:=exp(exponent * ln (base));
  end;


function intpower(base : float;const exponent : Integer) : float;
  var
     i : longint;
  begin
     if (base = 0.0) and (exponent = 0) then
       result:=1
     else
       begin
         if exponent<0 then
           base:=1.0/base;
         i:=abs(exponent);
         intpower:=1.0;
         while i>0 do
           begin
              while (i and 1)=0 do
                begin
                   i:=i shr 1;
                   base:=sqr(base);
                end;
              i:=i-1;
              intpower:=intpower*base;
           end;
       end;
  end;


operator ** (bas,expo : float) e: float; inline;
  begin
    e:=power(bas,expo);
  end;


operator ** (bas,expo : int64) i: int64; inline;
  begin
    i:=round(intpower(bas,expo));
  end;


function ceil(x : float) : integer;
  begin
    Result:=Trunc(x)+ord(Frac(x)>0);
  end;


function ceil64(x: float): Int64;
  begin
    Result:=Trunc(x)+ord(Frac(x)>0);
  end;


function floor(x : float) : integer;
  begin
    Result:=Trunc(x)-ord(Frac(x)<0);
  end;


function floor64(x: float): Int64;
  begin
    Result:=Trunc(x)-ord(Frac(x)<0);
  end;


procedure Frexp(X: float; var Mantissa: float; var Exponent: integer);
begin
  Exponent:=0;
  if (X<>0) then
    if (abs(X)<0.5) then
      repeat
        X:=X*2;
        Dec(Exponent);
      until (abs(X)>=0.5)
    else
      while (abs(X)>=1) do
        begin
        X:=X/2;
        Inc(Exponent);
        end;
  Mantissa:=X;
end;

function ldexp(x : float;const p : Integer) : float;
  begin
     ldexp:=x*intpower(2.0,p);
  end;

{$ifdef FPC_HAS_TYPE_SINGLE}
function mean(const data : array of Single) : float;

  begin
     Result:=Mean(PSingle(@data[0]),High(Data)+1);
  end;

function mean(const data : PSingle; Const N : longint) : float;
  begin
     mean:=sum(Data,N);
     mean:=mean/N;
  end;

function sum(const data : array of Single) : float;inline;
  begin
     Result:=Sum(PSingle(@Data[0]),High(Data)+1);
  end;

function sum(const data : PSingle;Const N : longint) : float;
  var
     i : longint;
  begin
     sum:=0.0;
     for i:=0 to N-1 do
       sum:=sum+data[i];
  end;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function mean(const data : array of Double) : float; inline;
  begin
     Result:=Mean(PDouble(@data[0]),High(Data)+1);
  end;

function mean(const data : PDouble; Const N : longint) : float;
  begin
     mean:=sum(Data,N);
     mean:=mean/N;
  end;

function sum(const data : array of Double) : float; inline;
  begin
     Result:=Sum(PDouble(@Data[0]),High(Data)+1);
  end;

function sum(const data : PDouble;Const N : longint) : float;
  var
     i : longint;
  begin
     sum:=0.0;
     for i:=0 to N-1 do
       sum:=sum+data[i];
  end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function mean(const data : array of Extended) : float;
  begin
     Result:=Mean(PExtended(@data[0]),High(Data)+1);
  end;

function mean(const data : PExtended; Const N : longint) : float;
  begin
     mean:=sum(Data,N);
     mean:=mean/N;
  end;

function sum(const data : array of Extended) : float; inline;
  begin
     Result:=Sum(PExtended(@Data[0]),High(Data)+1);
  end;

function sum(const data : PExtended;Const N : longint) : float;
  var
     i : longint;
  begin
     sum:=0.0;
     for i:=0 to N-1 do
       sum:=sum+data[i];
  end;
{$endif FPC_HAS_TYPE_EXTENDED}

function sumInt(const data : PInt64;Const N : longint) : Int64;
  var
     i : longint;
  begin
     sumInt:=0;
     for i:=0 to N-1 do
       sumInt:=sumInt+data[i];
  end;

function sumInt(const data : array of Int64) : Int64; inline;
  begin
     Result:=SumInt(PInt64(@Data[0]),High(Data)+1);
  end;

function mean(const data : PInt64; const N : Longint):Float;
  begin
     mean:=sumInt(Data,N);
     mean:=mean/N;
  end;

function mean(const data: array of Int64):Float;
  begin
     mean:=mean(PInt64(@data[0]),High(Data)+1);
  end;

function sumInt(const data : PInteger; Const N : longint) : Int64;
var
   i : longint;
  begin
     sumInt:=0;
     for i:=0 to N-1 do
       sumInt:=sumInt+data[i];
  end;

function sumInt(const data : array of Integer) : Int64;inline;
  begin
     Result:=sumInt(PInteger(@Data[0]),High(Data)+1);
  end;

function mean(const data : PInteger; const N : Longint):Float;
  begin
     mean:=sumInt(Data,N);
     mean:=mean/N;
  end;

function mean(const data: array of Integer):Float;
  begin
     mean:=mean(PInteger(@data[0]),High(Data)+1);
  end;

{$ifdef FPC_HAS_TYPE_SINGLE}
 function sumofsquares(const data : array of Single) : float; inline;
 begin
   Result:=sumofsquares(PSingle(@data[0]),High(Data)+1);
 end;

 function sumofsquares(const data : PSingle; Const N : Integer) : float;
  var
     i : longint;
  begin
     sumofsquares:=0.0;
     for i:=0 to N-1 do
       sumofsquares:=sumofsquares+sqr(data[i]);
  end;

procedure sumsandsquares(const data : array of Single;
  var sum,sumofsquares : float); inline;
begin
  sumsandsquares (PSingle(@Data[0]),High(Data)+1,Sum,sumofsquares);
end;

procedure sumsandsquares(const data : PSingle; Const N : Integer;
  var sum,sumofsquares : float);
  var
     i : Integer;
     temp : float;
  begin
     sumofsquares:=0.0;
     sum:=0.0;
     for i:=0 to N-1 do
       begin
          temp:=data[i];
          sumofsquares:=sumofsquares+sqr(temp);
          sum:=sum+temp;
       end;
  end;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
 function sumofsquares(const data : array of Double) : float; inline;
 begin
   Result:=sumofsquares(PDouble(@data[0]),High(Data)+1);
 end;

 function sumofsquares(const data : PDouble; Const N : Integer) : float;
  var
     i : longint;
  begin
     sumofsquares:=0.0;
     for i:=0 to N-1 do
       sumofsquares:=sumofsquares+sqr(data[i]);
  end;

procedure sumsandsquares(const data : array of Double;
  var sum,sumofsquares : float);
begin
  sumsandsquares (PDouble(@Data[0]),High(Data)+1,Sum,sumofsquares);
end;

procedure sumsandsquares(const data : PDouble; Const N : Integer;
  var sum,sumofsquares : float);
  var
     i : Integer;
     temp : float;
  begin
     sumofsquares:=0.0;
     sum:=0.0;
     for i:=0 to N-1 do
       begin
          temp:=data[i];
          sumofsquares:=sumofsquares+sqr(temp);
          sum:=sum+temp;
       end;
  end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
 function sumofsquares(const data : array of Extended) : float; inline;
 begin
   Result:=sumofsquares(PExtended(@data[0]),High(Data)+1);
 end;

 function sumofsquares(const data : PExtended; Const N : Integer) : float;
  var
     i : longint;
  begin
     sumofsquares:=0.0;
     for i:=0 to N-1 do
       sumofsquares:=sumofsquares+sqr(data[i]);
  end;

procedure sumsandsquares(const data : array of Extended;
  var sum,sumofsquares : float); inline;
begin
  sumsandsquares (PExtended(@Data[0]),High(Data)+1,Sum,sumofsquares);
end;

procedure sumsandsquares(const data : PExtended; Const N : Integer;
  var sum,sumofsquares : float);
  var
     i : Integer;
     temp : float;
  begin
     sumofsquares:=0.0;
     sum:=0.0;
     for i:=0 to N-1 do
       begin
          temp:=data[i];
          sumofsquares:=sumofsquares+sqr(temp);
          sum:=sum+temp;
       end;
  end;
{$endif FPC_HAS_TYPE_EXTENDED}

function randg(mean,stddev : float) : float;
  Var U1,S2 : Float;
  begin
     repeat
       u1:= 2*random-1;
       S2:=Sqr(U1)+sqr(2*random-1);
     until s2<1;
     randg:=Sqrt(-2*ln(S2)/S2)*u1*stddev+Mean;
  end;


function RandomRange(const aFrom, aTo: Integer): Integer;
begin
  Result:=Random(Abs(aFrom-aTo))+Min(aTo,AFrom);
end;


function RandomRange(const aFrom, aTo: Int64): Int64;
begin
  Result:=Random(Abs(aFrom-aTo))+Min(aTo,AFrom);
end;


{$ifdef FPC_HAS_TYPE_SINGLE}
procedure MeanAndTotalVariance
  (const data: PSingle; N: LongInt; var mu, variance: float);
var i: LongInt;
begin
  mu := Mean( data, N );
  variance := 0;
  for i := 0 to N - 1 do
    variance := variance + Sqr( data[i] - mu );
end;

function stddev(const data : array of Single) : float; inline;
begin
  Result:=Stddev(PSingle(@Data[0]),High(Data)+1);
end;

function stddev(const data : PSingle; Const N : Integer) : float;
  begin
     StdDev:=Sqrt(Variance(Data,N));
  end;

procedure meanandstddev(const data : array of Single;
  var mean,stddev : float); inline;
begin
  Meanandstddev(PSingle(@Data[0]),High(Data)+1,Mean,stddev);
end;

procedure meanandstddev
( const data:   PSingle;
  const N:      Longint;
  var   mean,
        stdDev: Float
);
var totalVariance: float;
begin
  MeanAndTotalVariance( data, N, mean, totalVariance );
  if N < 2 then stdDev := 0
  else stdDev := Sqrt( totalVariance / ( N - 1 ) );
end;

function variance(const data : array of Single) : float; inline;
  begin
     Variance:=Variance(PSingle(@Data[0]),High(Data)+1);
  end;

function variance(const data : PSingle; Const N : Integer) : float;
  begin
     If N=1 then
       Result:=0
     else
       Result:=TotalVariance(Data,N)/(N-1);
  end;

function totalvariance(const data : array of Single) : float; inline;
begin
  Result:=TotalVariance(PSingle(@Data[0]),High(Data)+1);
end;

function totalvariance(const data : PSingle; const N : Integer) : float;
var mu: float;
begin
  MeanAndTotalVariance( data, N, mu, result );
end;

function popnstddev(const data : array of Single) : float;
  begin
     PopnStdDev:=Sqrt(PopnVariance(PSingle(@Data[0]),High(Data)+1));
  end;

function popnstddev(const data : PSingle; Const N : Integer) : float;
  begin
     PopnStdDev:=Sqrt(PopnVariance(Data,N));
  end;

function popnvariance(const data : array of Single) : float; inline;

begin
  popnvariance:=popnvariance(PSingle(@data[0]),high(Data)+1);
end;

function popnvariance(const data : PSingle; Const N : Integer) : float;

  begin
     PopnVariance:=TotalVariance(Data,N)/N;
  end;

procedure momentskewkurtosis(const data : array of single;
  out m1,m2,m3,m4,skew,kurtosis : float); inline;
begin
  momentskewkurtosis(PSingle(@Data[0]),High(Data)+1,m1,m2,m3,m4,skew,kurtosis);
end;

procedure momentskewkurtosis(
  const data: pSingle;
  Const N: integer;
  out m1: float;
  out m2: float;
  out m3: float;
  out m4: float;
  out skew: float;
  out kurtosis: float
);
var
  i: integer;
  value : psingle;
  deviation, deviation2: single;
  reciprocalN: float;
begin
  m1 := 0;
  reciprocalN := 1/N;
  value := data;
  for i := 0 to N-1 do
  begin
    m1 := m1 + value^;
    inc(value);
  end;
  m1 := reciprocalN * m1;

  m2 := 0;
  m3 := 0;
  m4 := 0;
  value := data;
  for i := 0 to N-1 do
  begin
    deviation := (value^-m1);
    deviation2 := deviation * deviation;
    m2 := m2 + deviation2;
    m3 := m3 + deviation2 * deviation;
    m4 := m4 + deviation2 * deviation2;
    inc(value);
  end;
  m2 := reciprocalN * m2;
  m3 := reciprocalN * m3;
  m4 := reciprocalN * m4;

  skew := m3 / (sqrt(m2)*m2);
  kurtosis := m4 / (m2 * m2);
end;

function norm(const data : array of Single) : float; inline;
  begin
     norm:=Norm(PSingle(@data[0]),High(Data)+1);
  end;

function norm(const data : PSingle; Const N : Integer) : float;

  begin
     norm:=sqrt(sumofsquares(data,N));
  end;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
procedure MeanAndTotalVariance
  (const data: PDouble; N: LongInt; var mu, variance: float);
var i: LongInt;
begin
  mu := Mean( data, N );
  variance := 0;
  for i := 0 to N - 1 do
    variance := variance + Sqr( data[i] - mu );
end;

function stddev(const data : array of Double) : float; inline;
begin
  Result:=Stddev(PDouble(@Data[0]),High(Data)+1)
end;

function stddev(const data : PDouble; Const N : Integer) : float;
  begin
     StdDev:=Sqrt(Variance(Data,N));
  end;

procedure meanandstddev(const data : array of Double;
  var mean,stddev : float);

begin
  Meanandstddev(PDouble(@Data[0]),High(Data)+1,Mean,stddev);
end;

procedure meanandstddev
( const data:   PDouble;
  const N:      Longint;
  var   mean,
        stdDev: Float
);
var totalVariance: float;
begin
  MeanAndTotalVariance( data, N, mean, totalVariance );
  if N < 2 then stdDev := 0
  else stdDev := Sqrt( totalVariance / ( N - 1 ) );
end;

function variance(const data : array of Double) : float; inline;
  begin
     Variance:=Variance(PDouble(@Data[0]),High(Data)+1);
  end;

function variance(const data : PDouble; Const N : Integer) : float;

  begin
     If N=1 then
       Result:=0
     else
       Result:=TotalVariance(Data,N)/(N-1);
  end;

function totalvariance(const data : array of Double) : float; inline;
begin
  Result:=TotalVariance(PDouble(@Data[0]),High(Data)+1);
end;

function totalvariance(const data : PDouble; const N : Integer) : float;
var mu: float;
begin
  MeanAndTotalVariance( data, N, mu, result );
end;

function popnstddev(const data : array of Double) : float;

  begin
     PopnStdDev:=Sqrt(PopnVariance(PDouble(@Data[0]),High(Data)+1));
  end;

function popnstddev(const data : PDouble; Const N : Integer) : float;

  begin
     PopnStdDev:=Sqrt(PopnVariance(Data,N));
  end;

function popnvariance(const data : array of Double) : float; inline;

begin
  popnvariance:=popnvariance(PDouble(@data[0]),high(Data)+1);
end;

function popnvariance(const data : PDouble; Const N : Integer) : float;

  begin
     PopnVariance:=TotalVariance(Data,N)/N;
  end;

procedure momentskewkurtosis(const data : array of Double;
  out m1,m2,m3,m4,skew,kurtosis : float);
begin
  momentskewkurtosis(PDouble(@Data[0]),High(Data)+1,m1,m2,m3,m4,skew,kurtosis);
end;

procedure momentskewkurtosis(
  const data: pdouble;
  Const N: integer;
  out m1: float;
  out m2: float;
  out m3: float;
  out m4: float;
  out skew: float;
  out kurtosis: float
);
var
  i: integer;
  value : pdouble;
  deviation, deviation2: double;
  reciprocalN: float;
begin
  m1 := 0;
  reciprocalN := 1/N;
  value := data;
  for i := 0 to N-1 do
  begin
    m1 := m1 + value^;
    inc(value);
  end;
  m1 := reciprocalN * m1;

  m2 := 0;
  m3 := 0;
  m4 := 0;
  value := data;
  for i := 0 to N-1 do
  begin
    deviation := (value^-m1);
    deviation2 := deviation * deviation;
    m2 := m2 + deviation2;
    m3 := m3 + deviation2 * deviation;
    m4 := m4 + deviation2 * deviation2;
    inc(value);
  end;
  m2 := reciprocalN * m2;
  m3 := reciprocalN * m3;
  m4 := reciprocalN * m4;

  skew := m3 / (sqrt(m2)*m2);
  kurtosis := m4 / (m2 * m2);
end;


function norm(const data : array of Double) : float; inline;
  begin
     norm:=Norm(PDouble(@data[0]),High(Data)+1);
  end;

function norm(const data : PDouble; Const N : Integer) : float;
  begin
     norm:=sqrt(sumofsquares(data,N));
  end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
procedure MeanAndTotalVariance
  (const data: PExtended; N: LongInt; var mu, variance: float);
var i: LongInt;
begin
  mu := Mean( data, N );
  variance := 0;
  for i := 0 to N - 1 do
    variance := variance + Sqr( data[i] - mu );
end;

function stddev(const data : array of Extended) : float; inline;
begin
  Result:=Stddev(PExtended(@Data[0]),High(Data)+1)
end;

function stddev(const data : PExtended; Const N : Integer) : float;
  begin
     StdDev:=Sqrt(Variance(Data,N));
  end;

procedure meanandstddev(const data : array of Extended;
  var mean,stddev : float); inline;
begin
  Meanandstddev(PExtended(@Data[0]),High(Data)+1,Mean,stddev);
end;

procedure meanandstddev
( const data:   PExtended;
  const N:      Longint;
  var   mean,
        stdDev: Float
);
var totalVariance: float;
begin
  MeanAndTotalVariance( data, N, mean, totalVariance );
  if N < 2 then stdDev := 0
  else stdDev := Sqrt( totalVariance / ( N - 1 ) );
end;

function variance(const data : array of Extended) : float; inline;
  begin
     Variance:=Variance(PExtended(@Data[0]),High(Data)+1);
  end;

function variance(const data : PExtended; Const N : Integer) : float;

  begin
     If N=1 then
       Result:=0
     else
       Result:=TotalVariance(Data,N)/(N-1);
  end;

function totalvariance(const data : array of Extended) : float; inline;
begin
  Result:=TotalVariance(PExtended(@Data[0]),High(Data)+1);
end;

function totalvariance(const data : PExtended;Const N : Integer) : float;
var mu: float;
begin
  MeanAndTotalVariance( data, N, mu, result );
end;

function popnstddev(const data : array of Extended) : float;

  begin
     PopnStdDev:=Sqrt(PopnVariance(PExtended(@Data[0]),High(Data)+1));
  end;

function popnstddev(const data : PExtended; Const N : Integer) : float;

  begin
     PopnStdDev:=Sqrt(PopnVariance(Data,N));
  end;

function popnvariance(const data : array of Extended) : float; inline;
begin
  popnvariance:=popnvariance(PExtended(@data[0]),high(Data)+1);
end;

function popnvariance(const data : PExtended; Const N : Integer) : float;

  begin
     PopnVariance:=TotalVariance(Data,N)/N;
  end;

procedure momentskewkurtosis(const data : array of Extended;
  out m1,m2,m3,m4,skew,kurtosis : float); inline;
begin
  momentskewkurtosis(PExtended(@Data[0]),High(Data)+1,m1,m2,m3,m4,skew,kurtosis);
end;

procedure momentskewkurtosis(
  const data: pExtended;
  Const N: integer;
  out m1: float;
  out m2: float;
  out m3: float;
  out m4: float;
  out skew: float;
  out kurtosis: float
);
var
  i: integer;
  value : pextended;
  deviation, deviation2: extended;
  reciprocalN: float;
begin
  m1 := 0;
  reciprocalN := 1/N;
  value := data;
  for i := 0 to N-1 do
  begin
    m1 := m1 + value^;
    inc(value);
  end;
  m1 := reciprocalN * m1;

  m2 := 0;
  m3 := 0;
  m4 := 0;
  value := data;
  for i := 0 to N-1 do
  begin
    deviation := (value^-m1);
    deviation2 := deviation * deviation;
    m2 := m2 + deviation2;
    m3 := m3 + deviation2 * deviation;
    m4 := m4 + deviation2 * deviation2;
    inc(value);
  end;
  m2 := reciprocalN * m2;
  m3 := reciprocalN * m3;
  m4 := reciprocalN * m4;

  skew := m3 / (sqrt(m2)*m2);
  kurtosis := m4 / (m2 * m2);
end;

function norm(const data : array of Extended) : float; inline;
  begin
     norm:=Norm(PExtended(@data[0]),High(Data)+1);
  end;

function norm(const data : PExtended; Const N : Integer) : float;

  begin
     norm:=sqrt(sumofsquares(data,N));
  end;
{$endif FPC_HAS_TYPE_EXTENDED}


function MinIntValue(const Data: array of Integer): Integer;
var
  I: Integer;
begin
  Result := Data[Low(Data)];
  For I := Succ(Low(Data)) To High(Data) Do
    If Data[I] < Result Then Result := Data[I];
end;

function MaxIntValue(const Data: array of Integer): Integer;
var
  I: Integer;
begin
  Result := Data[Low(Data)];
  For I := Succ(Low(Data)) To High(Data) Do
    If Data[I] > Result Then Result := Data[I];
end;

function MinValue(const Data: array of Integer): Integer; inline;
begin
  Result:=MinValue(Pinteger(@Data[0]),High(Data)+1)
end;

function MinValue(const Data: PInteger; Const N : Integer): Integer;
var
  I: Integer;
begin
  Result := Data[0];
  For I := 1 To N-1 do
    If Data[I] < Result Then Result := Data[I];
end;

function MaxValue(const Data: array of Integer): Integer; inline;
begin
  Result:=MaxValue(PInteger(@Data[0]),High(Data)+1)
end;

function maxvalue(const data : PInteger; Const N : Integer) : Integer;
var
   i : longint;
begin
   { get an initial value }
   maxvalue:=data[0];
   for i:=1 to N-1 do
     if data[i]>maxvalue then
       maxvalue:=data[i];
end;

{$ifdef FPC_HAS_TYPE_SINGLE}
function minvalue(const data : array of Single) : Single; inline;
begin
   Result:=minvalue(PSingle(@data[0]),High(Data)+1);
end;

function minvalue(const data : PSingle; Const N : Integer) : Single;
var
   i : longint;
begin
   { get an initial value }
   minvalue:=data[0];
   for i:=1 to N-1 do
     if data[i]<minvalue then
       minvalue:=data[i];
end;


function maxvalue(const data : array of Single) : Single; inline;
begin
   Result:=maxvalue(PSingle(@data[0]),High(Data)+1);
end;

function maxvalue(const data : PSingle; Const N : Integer) : Single;
var
   i : longint;
begin
   { get an initial value }
   maxvalue:=data[0];
   for i:=1 to N-1 do
     if data[i]>maxvalue then
       maxvalue:=data[i];
end;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function minvalue(const data : array of Double) : Double; inline;
begin
   Result:=minvalue(PDouble(@data[0]),High(Data)+1);
end;

function minvalue(const data : PDouble; Const N : Integer) : Double;
var
   i : longint;
begin
   { get an initial value }
   minvalue:=data[0];
   for i:=1 to N-1 do
     if data[i]<minvalue then
       minvalue:=data[i];
end;


function maxvalue(const data : array of Double) : Double; inline;
begin
   Result:=maxvalue(PDouble(@data[0]),High(Data)+1);
end;

function maxvalue(const data : PDouble; Const N : Integer) : Double;
var
   i : longint;
begin
   { get an initial value }
   maxvalue:=data[0];
   for i:=1 to N-1 do
     if data[i]>maxvalue then
       maxvalue:=data[i];
end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function minvalue(const data : array of Extended) : Extended; inline;
begin
   Result:=minvalue(PExtended(@data[0]),High(Data)+1);
end;

function minvalue(const data : PExtended; Const N : Integer) : Extended;
var
   i : longint;
begin
   { get an initial value }
   minvalue:=data[0];
   for i:=1 to N-1 do
     if data[i]<minvalue then
       minvalue:=data[i];
end;


function maxvalue(const data : array of Extended) : Extended; inline;
begin
   Result:=maxvalue(PExtended(@data[0]),High(Data)+1);
end;

function maxvalue(const data : PExtended; Const N : Integer) : Extended;
var
   i : longint;
begin
   { get an initial value }
   maxvalue:=data[0];
   for i:=1 to N-1 do
     if data[i]>maxvalue then
       maxvalue:=data[i];
end;
{$endif FPC_HAS_TYPE_EXTENDED}


function Min(a, b: Integer): Integer;inline;
begin
  if a < b then
    Result := a
  else
    Result := b;
end;

function Max(a, b: Integer): Integer;inline;
begin
  if a > b then
    Result := a
  else
    Result := b;
end;

{
function Min(a, b: Cardinal): Cardinal;inline;
begin
  if a < b then
    Result := a
  else
    Result := b;
end;

function Max(a, b: Cardinal): Cardinal;inline;
begin
  if a > b then
    Result := a
  else
    Result := b;
end;
}

function Min(a, b: Int64): Int64;inline;
begin
  if a < b then
    Result := a
  else
    Result := b;
end;

function Max(a, b: Int64): Int64;inline;
begin
  if a > b then
    Result := a
  else
    Result := b;
end;

function Min(a, b: QWord): QWord; inline;
begin
  if a < b then
    Result := a
  else
    Result := b;
end;

function Max(a, b: QWord): Qword;inline;
begin
  if a > b then
    Result := a
  else
    Result := b;
end;

{$ifdef FPC_HAS_TYPE_SINGLE}
function Min(a, b: Single): Single;inline;
begin
  if a < b then
    Result := a
  else
    Result := b;
end;

function Max(a, b: Single): Single;inline;
begin
  if a > b then
    Result := a
  else
    Result := b;
end;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function Min(a, b: Double): Double;inline;
begin
  if a < b then
    Result := a
  else
    Result := b;
end;

function Max(a, b: Double): Double;inline;
begin
  if a > b then
    Result := a
  else
    Result := b;
end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function Min(a, b: Extended): Extended;inline;
begin
  if a < b then
    Result := a
  else
    Result := b;
end;

function Max(a, b: Extended): Extended;inline;
begin
  if a > b then
    Result := a
  else
    Result := b;
end;
{$endif FPC_HAS_TYPE_EXTENDED}

function InRange(const AValue, AMin, AMax: Integer): Boolean;inline;

begin
  Result:=(AValue>=AMin) and (AValue<=AMax);
end;

function InRange(const AValue, AMin, AMax: Int64): Boolean;inline;
begin
  Result:=(AValue>=AMin) and (AValue<=AMax);
end;

{$ifdef FPC_HAS_TYPE_DOUBLE}
function InRange(const AValue, AMin, AMax: Double): Boolean;inline;

begin
  Result:=(AValue>=AMin) and (AValue<=AMax);
end;
{$endif FPC_HAS_TYPE_DOUBLE}

function EnsureRange(const AValue, AMin, AMax: Integer): Integer;inline;

begin
  Result:=AValue;
  If Result<AMin then
    Result:=AMin;
  if Result>AMax then
    Result:=AMax;
end;

function EnsureRange(const AValue, AMin, AMax: Int64): Int64;inline;

begin
  Result:=AValue;
  If Result<AMin then
    Result:=AMin;
  if Result>AMax then
    Result:=AMax;
end;

{$ifdef FPC_HAS_TYPE_DOUBLE}
function EnsureRange(const AValue, AMin, AMax: Double): Double;inline;

begin
  Result:=AValue;
  If Result<AMin then
    Result:=AMin;
  if Result>AMax then
    Result:=AMax;
end;
{$endif FPC_HAS_TYPE_DOUBLE}

Const
  EZeroResolution = 1E-16;
  DZeroResolution = 1E-12;
  SZeroResolution = 1E-4;


function IsZero(const A: Single; Epsilon: Single): Boolean;

begin
  if (Epsilon=0) then
    Epsilon:=SZeroResolution;
  Result:=Abs(A)<=Epsilon;
end;

function IsZero(const A: Single): Boolean;inline;

begin
  Result:=IsZero(A,single(SZeroResolution));
end;

{$ifdef FPC_HAS_TYPE_DOUBLE}
function IsZero(const A: Double; Epsilon: Double): Boolean;

begin
  if (Epsilon=0) then
    Epsilon:=DZeroResolution;
  Result:=Abs(A)<=Epsilon;
end;

function IsZero(const A: Double): Boolean;inline;

begin
  Result:=IsZero(A,DZeroResolution);
end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function IsZero(const A: Extended; Epsilon: Extended): Boolean;

begin
  if (Epsilon=0) then
    Epsilon:=EZeroResolution;
  Result:=Abs(A)<=Epsilon;
end;

function IsZero(const A: Extended): Boolean;inline;

begin
  Result:=IsZero(A,EZeroResolution);
end;
{$endif FPC_HAS_TYPE_EXTENDED}


type
  TSplitDouble = packed record
    cards: Array[0..1] of cardinal;
  end;

  TSplitExtended = packed record
    cards: Array[0..1] of cardinal;
    w: word;
  end;

function IsNan(const d : Single): Boolean; overload;
  begin
    result:=(longword(d) and $7fffffff)>$7f800000;
  end;

{$ifdef FPC_HAS_TYPE_DOUBLE}
function IsNan(const d : Double): Boolean;
  var
    fraczero, expMaximal: boolean;
  begin
{$if defined(FPC_BIG_ENDIAN) or defined(FPC_DOUBLE_HILO_SWAPPED)}
    expMaximal := ((TSplitDouble(d).cards[0] shr 20) and $7ff) = 2047;
    fraczero:= (TSplitDouble(d).cards[0] and $fffff = 0) and
                (TSplitDouble(d).cards[1] = 0);
{$else FPC_BIG_ENDIAN}
    expMaximal := ((TSplitDouble(d).cards[1] shr 20) and $7ff) = 2047;
    fraczero := (TSplitDouble(d).cards[1] and $fffff = 0) and
                (TSplitDouble(d).cards[0] = 0);
{$endif FPC_BIG_ENDIAN}
    Result:=expMaximal and not(fraczero);
  end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function IsNan(const d : Extended): Boolean; overload;
  var
    fraczero, expMaximal: boolean;
  begin
{$ifdef FPC_BIG_ENDIAN}
  {$error no support for big endian extended type yet}
{$else FPC_BIG_ENDIAN}
    expMaximal := (TSplitExtended(d).w and $7fff) = 32767;
    fraczero := (TSplitExtended(d).cards[0] = 0) and
                    ((TSplitExtended(d).cards[1] and $7fffffff) = 0);
{$endif FPC_BIG_ENDIAN}
    Result:=expMaximal and not(fraczero);
  end;
{$endif FPC_HAS_TYPE_EXTENDED}

function IsInfinite(const d : Single): Boolean; overload;
  begin
    result:=(longword(d) and $7fffffff)=$7f800000;
  end;

{$ifdef FPC_HAS_TYPE_DOUBLE}
function IsInfinite(const d : Double): Boolean; overload;
  var
    fraczero, expMaximal: boolean;
  begin
{$if defined(FPC_BIG_ENDIAN) or defined(FPC_DOUBLE_HILO_SWAPPED)}
    expMaximal := ((TSplitDouble(d).cards[0] shr 20) and $7ff) = 2047;
    fraczero:= (TSplitDouble(d).cards[0] and $fffff = 0) and
                (TSplitDouble(d).cards[1] = 0);
{$else FPC_BIG_ENDIAN}
    expMaximal := ((TSplitDouble(d).cards[1] shr 20) and $7ff) = 2047;
    fraczero := (TSplitDouble(d).cards[1] and $fffff = 0) and
                (TSplitDouble(d).cards[0] = 0);
{$endif FPC_BIG_ENDIAN}
    Result:=expMaximal and fraczero;
  end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function IsInfinite(const d : Extended): Boolean; overload;
  var
    fraczero, expMaximal: boolean;
  begin
{$ifdef FPC_BIG_ENDIAN}
  {$error no support for big endian extended type yet}
{$else FPC_BIG_ENDIAN}
    expMaximal := (TSplitExtended(d).w and $7fff) = 32767;
    fraczero := (TSplitExtended(d).cards[0] = 0) and
                    ((TSplitExtended(d).cards[1] and $7fffffff) = 0);
{$endif FPC_BIG_ENDIAN}
    Result:=expMaximal and fraczero;
  end;
{$endif FPC_HAS_TYPE_EXTENDED}

function copysign(x,y: float): float;
begin
{$if defined(FPC_HAS_TYPE_FLOAT128)}
  {$error copysign not yet implemented for float128}
{$elseif defined(FPC_HAS_TYPE_EXTENDED)}
  TSplitExtended(x).w:=(TSplitExtended(x).w and $7fff) or (TSplitExtended(y).w and $8000);
{$elseif defined(FPC_HAS_TYPE_DOUBLE)}
  {$if defined(FPC_BIG_ENDIAN) or defined(FPC_DOUBLE_HILO_SWAPPED)}
  TSplitDouble(x).cards[0]:=(TSplitDouble(x).cards[0] and $7fffffff) or (TSplitDouble(y).cards[0] and longword($80000000));
  {$else}
  TSplitDouble(x).cards[1]:=(TSplitDouble(x).cards[1] and $7fffffff) or (TSplitDouble(y).cards[1] and longword($80000000));
  {$endif}
{$else}
  longword(x):=longword(x and $7fffffff) or (longword(y) and longword($80000000));
{$endif}
  result:=x;
end;

{$ifdef FPC_HAS_TYPE_EXTENDED}
function SameValue(const A, B: Extended; Epsilon: Extended): Boolean;

begin
  if (Epsilon=0) then
    Epsilon:=Max(Min(Abs(A),Abs(B))*EZeroResolution,EZeroResolution);
  if (A>B) then
    Result:=((A-B)<=Epsilon)
  else
    Result:=((B-A)<=Epsilon);
end;

function SameValue(const A, B: Extended): Boolean;inline;

begin
  Result:=SameValue(A,B,0.0);
end;
{$endif FPC_HAS_TYPE_EXTENDED}


{$ifdef FPC_HAS_TYPE_DOUBLE}
function SameValue(const A, B: Double): Boolean;inline;

begin
  Result:=SameValue(A,B,0.0);
end;

function SameValue(const A, B: Double; Epsilon: Double): Boolean;

begin
  if (Epsilon=0) then
    Epsilon:=Max(Min(Abs(A),Abs(B))*DZeroResolution,DZeroResolution);
  if (A>B) then
    Result:=((A-B)<=Epsilon)
  else
    Result:=((B-A)<=Epsilon);
end;
{$endif FPC_HAS_TYPE_DOUBLE}

function SameValue(const A, B: Single): Boolean;inline;

begin
  Result:=SameValue(A,B,0);
end;

function SameValue(const A, B: Single; Epsilon: Single): Boolean;

begin
  if (Epsilon=0) then
    Epsilon:=Max(Min(Abs(A),Abs(B))*SZeroResolution,SZeroResolution);
  if (A>B) then
    Result:=((A-B)<=Epsilon)
  else
    Result:=((B-A)<=Epsilon);
end;

// Some CPUs probably allow a faster way of doing this in a single operation...
// There weshould define  FPC_MATH_HAS_CPUDIVMOD in the header mathuh.inc and implement it using asm.
{$ifndef FPC_MATH_HAS_DIVMOD}
procedure DivMod(Dividend: LongInt; Divisor: Word; var Result, Remainder: Word);
begin
  if Dividend < 0 then
    begin
      { Use DivMod with >=0 dividend }
	  Dividend:=-Dividend;
      { The documented behavior of Pascal's div/mod operators and DivMod
        on negative dividends is to return Result closer to zero and
        a negative Remainder. Which means that we can just negate both
        Result and Remainder, and all it's Ok. }
      Result:=-(Dividend Div Divisor);
      Remainder:=-(Dividend+(Result*Divisor));
    end
  else
    begin
	  Result:=Dividend Div Divisor;
      Remainder:=Dividend-(Result*Divisor);
	end;
end;


procedure DivMod(Dividend: LongInt; Divisor: Word; var Result, Remainder: SmallInt);
begin
  if Dividend < 0 then
    begin
      { Use DivMod with >=0 dividend }
	  Dividend:=-Dividend;
      { The documented behavior of Pascal's div/mod operators and DivMod
        on negative dividends is to return Result closer to zero and
        a negative Remainder. Which means that we can just negate both
        Result and Remainder, and all it's Ok. }
      Result:=-(Dividend Div Divisor);
      Remainder:=-(Dividend+(Result*Divisor));
    end
  else
    begin
	  Result:=Dividend Div Divisor;
      Remainder:=Dividend-(Result*Divisor);
	end;
end;


procedure DivMod(Dividend: DWord; Divisor: DWord; var Result, Remainder: DWord);
begin
  Result:=Dividend Div Divisor;
  Remainder:=Dividend-(Result*Divisor);
end;


procedure DivMod(Dividend: LongInt; Divisor: LongInt; var Result, Remainder: LongInt);
begin
  if Dividend < 0 then
    begin
      { Use DivMod with >=0 dividend }
      Dividend:=-Dividend;
      { The documented behavior of Pascal's div/mod operators and DivMod
        on negative dividends is to return Result closer to zero and
        a negative Remainder. Which means that we can just negate both
        Result and Remainder, and all it's Ok. }
      Result:=-(Dividend Div Divisor);
      Remainder:=-(Dividend+(Result*Divisor));
    end
  else
    begin
      Result:=Dividend Div Divisor;
      Remainder:=Dividend-(Result*Divisor);
    end;
end;
{$endif FPC_MATH_HAS_DIVMOD}

{ Floating point modulo}
{$ifdef FPC_HAS_TYPE_SINGLE}
function FMod(const a, b: Single): Single;inline;overload;
begin
  result:= a-b * Int(a/b);
end;
{$endif FPC_HAS_TYPE_SINGLE}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function FMod(const a, b: Double): Double;inline;overload;
begin
  result:= a-b * Int(a/b);
end;
{$endif FPC_HAS_TYPE_DOUBLE}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function FMod(const a, b: Extended): Extended;inline;overload;
begin
  result:= a-b * Int(a/b);
end;
{$endif FPC_HAS_TYPE_EXTENDED}

operator mod(const a,b:float) c:float;inline;
begin
  c:= a-b * Int(a/b);
  if SameValue(abs(c),abs(b)) then
    c:=0.0;
end;

function ifthen(val:boolean;const iftrue:integer; const iffalse:integer= 0) :integer;
begin
  if val then result:=iftrue else result:=iffalse;
end;

function ifthen(val:boolean;const iftrue:int64  ; const iffalse:int64 = 0)  :int64;
begin
  if val then result:=iftrue else result:=iffalse;
end;

function ifthen(val:boolean;const iftrue:double ; const iffalse:double =0.0):double;
begin
  if val then result:=iftrue else result:=iffalse;
end;

// dilemma here. asm can do the two comparisons in one go?
// but pascal is portable and can be inlined. Ah well, we need purepascal's anyway:
function CompareValue(const A, B  : Integer): TValueRelationship;

begin
  result:=GreaterThanValue;
  if a=b then
    result:=EqualsValue
  else
   if a<b then
     result:=LessThanValue;
end;

function CompareValue(const A, B: Int64): TValueRelationship;

begin
  result:=GreaterThanValue;
  if a=b then
    result:=EqualsValue
  else
   if a<b then
     result:=LessThanValue;
end;

function CompareValue(const A, B: QWord): TValueRelationship;

begin
  result:=GreaterThanValue;
  if a=b then
    result:=EqualsValue
  else
   if a<b then
     result:=LessThanValue;
end;

{$ifdef FPC_HAS_TYPE_SINGLE}
function CompareValue(const A, B: Single; delta: Single = 0.0): TValueRelationship;
begin
  result:=GreaterThanValue;
  if abs(a-b)<=delta then
    result:=EqualsValue
  else
   if a<b then
     result:=LessThanValue;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function CompareValue(const A, B: Double; delta: Double = 0.0): TValueRelationship;
begin
  result:=GreaterThanValue;
  if abs(a-b)<=delta then
    result:=EqualsValue
  else
   if a<b then
     result:=LessThanValue;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function CompareValue (const A, B: Extended; delta: Extended = 0.0): TValueRelationship;
begin
  result:=GreaterThanValue;
  if abs(a-b)<=delta then
    result:=EqualsValue
  else
   if a<b then
     result:=LessThanValue;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function RoundTo(const AValue: Double; const Digits: TRoundToRange): Double;

var
  RV : Double;

begin
  RV:=IntPower(10,Digits);
  Result:=Round(AValue/RV)*RV;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function RoundTo(const AVAlue: Extended; const Digits: TRoundToRange): Extended;

var
  RV : Extended;

begin
  RV:=IntPower(10,Digits);
  Result:=Round(AValue/RV)*RV;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_SINGLE}
function RoundTo(const AValue: Single; const Digits: TRoundToRange): Single;

var
  RV : Single;

begin
  RV:=IntPower(10,Digits);
  Result:=Round(AValue/RV)*RV;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_SINGLE}
function SimpleRoundTo(const AValue: Single; const Digits: TRoundToRange = -2): Single;

var
  RV : Single;

begin
  RV := IntPower(10, -Digits);
  if AValue < 0 then
    Result := Int((AValue*RV) - 0.5)/RV
  else
    Result := Int((AValue*RV) + 0.5)/RV;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_DOUBLE}
function SimpleRoundTo(const AValue: Double; const Digits: TRoundToRange = -2): Double;

var
  RV : Double;

begin
  RV := IntPower(10, -Digits);
  if AValue < 0 then
    Result := Int((AValue*RV) - 0.5)/RV
  else
    Result := Int((AValue*RV) + 0.5)/RV;
end;
{$endif}

{$ifdef FPC_HAS_TYPE_EXTENDED}
function SimpleRoundTo(const AValue: Extended; const Digits: TRoundToRange = -2): Extended;

var
  RV : Extended;

begin
  RV := IntPower(10, -Digits);
  if AValue < 0 then
    Result := Int((AValue*RV) - 0.5)/RV
  else
    Result := Int((AValue*RV) + 0.5)/RV;
end;
{$endif}

function RandomFrom(const AValues: array of Double): Double; overload;
begin
  result:=AValues[random(High(AValues)+1)];
end;

function RandomFrom(const AValues: array of Integer): Integer; overload;
begin
  result:=AValues[random(High(AValues)+1)];
end;

function RandomFrom(const AValues: array of Int64): Int64; overload;
begin
  result:=AValues[random(High(AValues)+1)];
end;

{$if FPC_FULLVERSION >=30101}
generic function RandomFrom<T>(const AValues:array of T):T;
begin
  result:=AValues[random(High(AValues)+1)];
end;
{$endif}

function FutureValue(ARate: Float; NPeriods: Integer;
  APayment, APresentValue: Float; APaymentTime: TPaymentTime): Float;
var
  q, qn, factor: Float;
begin
  if ARate = 0 then
    Result := -APresentValue - APayment * NPeriods
  else begin
    q := 1.0 + ARate;
    qn := power(q, NPeriods);
    factor := (qn - 1) / (q - 1);
    if APaymentTime = ptStartOfPeriod then
      factor := factor * q;
    Result := -(APresentValue * qn + APayment*factor);
  end;
end;

function InterestRate(NPeriods: Integer; APayment, APresentValue, AFutureValue: Float;
  APaymentTime: TPaymentTime): Float;
{ The interest rate cannot be calculated analytically. We solve the equation
  numerically by means of the Newton method:
  - guess value for the interest reate
  - calculate at which interest rate the tangent of the curve fv(rate)
    (straight line!) has the requested future vale.
  - use this rate for the next iteration. }
const
  DELTA = 0.001;
  EPS = 1E-9;   // required precision of interest rate (after typ. 6 iterations)
  MAXIT = 20;   // max iteration count to protect agains non-convergence
var
  r1, r2, dr: Float;
  fv1, fv2: Float;
  iteration: Integer;
begin
  iteration := 0;
  r1 := 0.05;  // inital guess
  repeat
    r2 := r1 + DELTA;
    fv1 := FutureValue(r1, NPeriods, APayment, APresentValue, APaymentTime);
    fv2 := FutureValue(r2, NPeriods, APayment, APresentValue, APaymentTime);
    dr := (AFutureValue - fv1) / (fv2 - fv1) * delta;  // tangent at fv(r)
    r1 := r1 + dr;      // next guess
    inc(iteration);
  until (abs(dr) < EPS) or (iteration >= MAXIT);
  Result := r1;
end;

function NumberOfPeriods(ARate, APayment, APresentValue, AFutureValue: Float;
  APaymentTime: TPaymentTime): Float;
{ Solve the cash flow equation (1) for q^n and take the logarithm }
var
  q, x1, x2: Float;
begin
  if ARate = 0 then
    Result := -(APresentValue + AFutureValue) / APayment
  else begin
    q := 1.0 + ARate;
    if APaymentTime = ptStartOfPeriod then
      APayment := APayment * q;
    x1 := APayment - AFutureValue * ARate;
    x2 := APayment + APresentValue * ARate;
    if   (x2 = 0)                    // we have to divide by x2
      or (sign(x1) * sign(x2) < 0)   // the argument of the log is negative
    then
      Result := Infinity
    else begin
      Result := ln(x1/x2) / ln(q);
    end;
  end;
end;

function Payment(ARate: Float; NPeriods: Integer;
  APresentValue, AFutureValue: Float; APaymentTime: TPaymentTime): Float;
var
  q, qn, factor: Float;
begin
  if ARate = 0 then
    Result := -(AFutureValue + APresentValue) / NPeriods
  else begin
    q := 1.0 + ARate;
    qn := power(q, NPeriods);
    factor := (qn - 1) / (q - 1);
    if APaymentTime = ptStartOfPeriod then
      factor := factor * q;
    Result := -(AFutureValue + APresentValue * qn) / factor;
  end;
end;

function PresentValue(ARate: Float; NPeriods: Integer;
  APayment, AFutureValue: Float; APaymentTime: TPaymentTime): Float;
var
  q, qn, factor: Float;
begin
  if ARate = 0.0 then
    Result := -AFutureValue - APayment * NPeriods
  else begin
    q := 1.0 + ARate;
    qn := power(q, NPeriods);
    factor := (qn - 1) / (q - 1);
    if APaymentTime = ptStartOfPeriod then
      factor := factor * q;
    Result := -(AFutureValue + APayment*factor) / qn;
  end;
end;

{$else}
implementation
{$endif FPUNONE}

end.
