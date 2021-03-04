{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005-2006 by the Free Pascal development team
    and Gehard Scholz

    It contains the Free Pascal BCD implementation

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS for A PARTICULAR PURPOSE.

 **********************************************************************}
{ "Programming is the time between two bugs" }
{     (last words of the unknown programmer) }

(* this program was a good test for the compiler: some bugs have been found.

  1. WITH in inline funcs produces a compiler error AFTER producing an .exe file
     (was already known; I didn't see it in the bug list)

  2. macro names were checked for being a keyword, even when starting with
     an '_' (produces range check when compiler is compiled with { $r+ }-mode
     fixed.

  3. { $define program } was not possible in { $macro on } mode
     (keywords not allowed: doesn't make sense here)
     fixed.

  4. the Inc/Dec ( unsigned, signed ) problem (has been similar in the
     bug list already)

  5. when the result of an overloaded (inline) operator is ABSOLUTEd:
     compiler error 200110205
     happens only when operator is defined in a unit.

  6. two range check errors in scanner.pas
     a) array subscripting
     b) value out ouf range
*)

{ $define debug_version}

// Dont use s+ (Stack checking on) because it crashes libraries, see bug 21208
{$r+,q+,s-}

{$mode objfpc}
{$h-}

{$inline on}

{$macro on}

{$define BCDMaxDigits := 64 } { should be even }

{ the next defines must be defined by hand,
  unless someone shows me a way how to to it with macros }

{$define BCDgr4}   { define this if MCDMaxDigits is greater 4,   else undefine! }
{$define BCDgr9}   { define this if MCDMaxDigits is greater 9,   else undefine! }
{$define BCDgr18}  { define this if MCDMaxDigits is greater 18,  else undefine! }
{ $define BCDgr64}  { define this if MCDMaxDigits is greater 64,  else undefine! }
{ $define BCDgr180} { define this if MCDMaxDigits is greater 180, else undefine! }

{$ifdef BCDgr4}
 {$hint BCD Digits > 4}
{$endif}

{$ifdef BCDgr9}
 {$hint BCD Digits > 9}
{$endif}

{$ifdef BCDgr18}
 {$hint BCD Digits > 18}
{$endif}

{$ifdef BCDgr64}
 {$hint BCD Digits > 64}
{$endif}

{$ifdef BCDgr180}
 {$hint BCD Digits > 180}
{$endif}

{$ifndef NO_SMART_LINK}
{ $smartlink on}
{$endif}

{$define some_packed} { enable this to keep some local structures PACKED }

{ $define as_object} { to define the tBCD record as object instead;
                      fields then are private  }
                     { not done yet! }

{$define additional_routines} { to create additional routines and operators }

(* only define one of them! *)
{ $define integ32}
{$define integ64}

(* only define one of them! *)
{ $define real8}
{$define real10}

{check}
{$ifndef integ32}
  {$ifndef integ64}
    {$define integ64}
  {$endif}
{$endif}

{$ifdef integ32}
  {$ifdef integ64}
    {$undef integ32}
  {$endif}
{$endif}

{check}
{$ifndef real8}
  {$ifndef real10}
    {$define real8}
  {$endif}
{$endif}

{$ifdef real8}
  {$ifdef real10}
    {$undef real10}
  {$endif}
{$endif}

{$ifdef some_packed}
  {$define maybe_packed := packed}
{$else}
  {$define maybe_packed := (**)}
{$endif}

UNIT FmtBCD;

INTERFACE

  USES
    SysUtils,
    Variants;

  const
    MaxStringDigits = 100;          { not used ! }
    _NoDecimal = -255;              { not used ! }
    _DefaultDecimals = 10;          { not used ! }

  { From DB.pas }
  { Max supported by Midas }               { must be EVEN }
    MaxFmtBCDFractionSize = BCDMaxDigits + Ord ( Odd ( BCDMaxDigits ) );
  { Max supported by Midas }
    MaxFmtBCDDigits =   32;         { not used ! }
    DefaultFmtBCDScale = 6;         { not used ! }
    MaxBCDPrecision =   18;         { not used ! }
    MaxBCDScale     =   4;          { not used ! }

{$ifdef BCDgr64}
{ $fatal big 1}
  {$define bigger_BCD}  { must be defined
                          if MaxFmtBCDFractionSize > 64 }
                        { not usable in the moment }
{$endif}

{$ifdef BCDgr180}
{ $fatal big 2}
  type
    FmtBCDStringtype = AnsiString;
  {$define use_Ansistring}
{$else}
  type
    FmtBCDStringtype = string [ 255 ];
  {$undef use_Ansistring}
{$endif}

{$ifdef use_ansistring}
  {$hint ansi}
{$else}
  {$hint -ansi}
{$endif}

{$ifdef integ32}
  {$define myInttype := LongInt}
{$endif}
{$ifdef integ64}
  {$define myInttype := int64}
{$endif}

{$ifndef FPUNONE}
{$ifdef real8}
  {$define myRealtype := double}
{$endif}
{$ifdef real10}
  {$define myRealtype := extended}
{$endif}
{$endif}

{$ifdef SUPPORT_COMP}
    {$define comproutines}
{$endif SUPPORT_COMP}

{$define __low_Fraction := 0 }
{$define __high_Fraction := ( ( MaxFmtBCDFractionSize DIV 2 ) - 1 ) }

  type
    pBCD = ^ tBCD;
    tBCD = packed {$ifdef as_object} OBJECT {$else} record {$endif}
            {$ifdef as_object} PRIVATE {$endif}
             Precision : 0..maxfmtbcdfractionsize;  { 1 (joke?)..64 }
{$ifndef bigger_BCD}
             SignSpecialPlaces : Byte;      { Sign:1, Special:1, Places:6 }
{$else}
             Negativ : Boolean;
{
             Special : Boolean;
}
             Places : 0..maxfmtbcdfractionsize - 1;
{$endif}
             Fraction : packed array [ __low_Fraction..__high_Fraction ] of Byte;
                            { BCD Nibbles, 00..99 per Byte, high Nibble 1st }
            end;

{ Exception classes }
  type
    eBCDException = CLASS ( Exception );
    eBCDOverflowException = CLASS ( eBCDException );
    eBCDNotImplementedException = CLASS ( eBCDException );


{ Utility functions for TBCD access }

  function BCDPrecision ( const BCD : tBCD ) : Word; Inline;

  function BCDScale ( const BCD : tBCD ) : Word; Inline;

  function IsBCDNegative ( const BCD : tBCD ) : Boolean; Inline;

{ BCD Arithmetic}

  procedure BCDNegate ( var BCD : tBCD ); Inline;

{ !!!!!!!!!! most routines are intentionally NOT inline !!!!!!!!!! }

{ Returns True if successful, False if Int Digits needed to be truncated }
  function NormalizeBCD ( const InBCD : tBCD;
                            var OutBCD : tBCD;
                          const Precision,
                                Places : Integer ) : Boolean;

  procedure BCDAdd ( const BCDin1,
                           BCDin2 : tBCD;
                       var BCDout : tBCD );

  procedure BCDSubtract ( const BCDin1,
                                BCDin2 : tBCD;
                            var BCDout : tBCD );

  procedure BCDMultiply ( const BCDin1,
                                BCDin2 : tBCD;
                            var BCDout : tBCD );

{$ifndef FPUNONE}
  procedure BCDMultiply ( const BCDIn : tBCD;
                          const DoubleIn : myRealtype;
                            var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDMultiply ( const BCDIn : tBCD;
                        const StringIn : FmtBCDStringtype;
                          var BCDout : tBCD ); Inline;

{ !!! params changed to const, shouldn't give a problem }
  procedure BCDMultiply ( const StringIn1,
                                StringIn2 : FmtBCDStringtype;
                          var BCDout : tBCD ); Inline;

  procedure BCDDivide ( const Dividend,
                              Divisor : tBCD;
                          var BCDout : tBCD );

{$ifndef FPUNONE}
  procedure BCDDivide ( const Dividend : tBCD;
                        const Divisor : myRealtype;
                          var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDDivide ( const Dividend : tBCD;
                        const Divisor : FmtBCDStringtype;
                            var BCDout : tBCD ); Inline;

{ !!! params changed to const, shouldn't give a problem }
  procedure BCDDivide ( const Dividend,
                              Divisor : FmtBCDStringtype;
                          var BCDout : tBCD ); Inline;

{ TBCD variant creation utils }
  procedure VarFmtBCDCreate (   var aDest : Variant;
                              const aBCD : tBCD );

  function VarFmtBCDCreate : Variant;

  function VarFmtBCDCreate ( const aValue : FmtBCDStringtype;
                                   Precision,
                                   Scale : Word ) : Variant;

{$ifndef FPUNONE}
  function VarFmtBCDCreate ( const aValue : myRealtype;
                                   Precision : Word = 18;
                                   Scale : Word = 4 ) : Variant;
{$endif}

  function VarFmtBCDCreate ( const aBCD : tBCD ) : Variant;

  function VarIsFmtBCD ( const aValue : Variant ) : Boolean;

  function VarFmtBCD : TVartype;

{ Convert string/Double/Integer to BCD struct }
  function StrToBCD ( const aValue : FmtBCDStringtype ) : tBCD;

  function StrToBCD ( const aValue : FmtBCDStringtype;
                            const Format : TFormatSettings ) : tBCD;

  function TryStrToBCD ( const aValue : FmtBCDStringtype;
                           var BCD : tBCD ) : Boolean;

  function TryStrToBCD ( const aValue : FmtBCDStringtype;
                           var BCD : tBCD;
                               const Format : TFormatSettings) : Boolean;

{$ifndef FPUNONE}
  function DoubleToBCD ( const aValue : myRealtype ) : tBCD; Inline;

  procedure DoubleToBCD ( const aValue : myRealtype;
                            var BCD : tBCD );
{$endif}

  function IntegerToBCD ( const aValue : myInttype ) : tBCD;

  function VarToBCD ( const aValue : Variant ) : tBCD;

{ From DB.pas }
  function CurrToBCD ( const Curr : currency;
                         var BCD : tBCD;
                             Precision : Integer = 32;
                             Decimals : Integer = 4 ) : Boolean;

{ Convert BCD struct to string/Double/Integer }
  function BCDToStr ( const BCD : tBCD ) : FmtBCDStringtype;

  function BCDToStr ( const BCD : tBCD;
                            const Format : TFormatSettings ) : FmtBCDStringtype;

{$ifndef FPUNONE}
  function BCDToDouble ( const BCD : tBCD ) : myRealtype;
{$endif}

  function BCDToInteger ( const BCD : tBCD;
                                Truncate : Boolean = False ) : myInttype;

{ From DB.pas }
  function BCDToCurr ( const BCD : tBCD;
                         var Curr : currency ) : Boolean;

{ Formatting BCD as string }
  function BCDToStrF ( const BCD : tBCD;
                             Format : TFloatFormat;
                       const Precision,
                             Digits : Integer ) : FmtBCDStringtype;

  function FormatBCD ( const Format : string;
                             BCD : tBCD ) : FmtBCDStringtype;

{ returns -1 if BCD1 < BCD2, 0 if BCD1 = BCD2, 1 if BCD1 > BCD2 }
  function BCDCompare ( const BCD1,
                              BCD2 : tBCD ) : Integer;

{$ifdef additional_routines}

  function CurrToBCD ( const Curr : currency ) : tBCD; Inline;

{$ifdef comproutines}
  function CompToBCD ( const Curr : Comp ) : tBCD; Inline;

  function BCDToComp ( const BCD : tBCD ) : Comp; Inline;
{$endif}

  procedure BCDAdd ( const BCDIn : tBCD;
                     const IntIn : myInttype;
                       var BCDout : tBCD );

  procedure BCDAdd ( const IntIn : myInttype;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

{$ifndef FPUNONE}
  procedure BCDAdd ( const BCDIn : tBCD;
                     const DoubleIn : myRealtype;
                       var BCDout : tBCD ); Inline;

  procedure BCDAdd ( const DoubleIn : myRealtype;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDAdd ( const BCDIn : tBCD;
                     const Currin : currency;
                       var BCDout : tBCD ); Inline;

  procedure BCDAdd ( const Currin : currency;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

{$ifdef comproutines}
  procedure BCDAdd ( const BCDIn : tBCD;
                     const Compin : Comp;
                       var BCDout : tBCD ); Inline;

  procedure BCDAdd ( const Compin : Comp;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDAdd ( const BCDIn : tBCD;
                     const StringIn : FmtBCDStringtype;
                       var BCDout : tBCD ); Inline;

  procedure BCDAdd ( const StringIn : FmtBCDStringtype;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

  procedure BCDAdd ( const StringIn1,
                           StringIn2 : FmtBCDStringtype;
                       var BCDout : tBCD ); Inline;

  procedure BCDSubtract ( const BCDIn : tBCD;
                          const IntIn : myInttype;
                            var BCDout : tBCD );

  procedure BCDSubtract ( const IntIn : myInttype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

{$ifndef FPUNONE}
  procedure BCDSubtract ( const BCDIn : tBCD;
                          const DoubleIn : myRealtype;
                            var BCDout : tBCD ); Inline;

  procedure BCDSubtract ( const DoubleIn : myRealtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDSubtract ( const BCDIn : tBCD;
                          const Currin : currency;
                            var BCDout : tBCD ); Inline;

  procedure BCDSubtract ( const Currin : currency;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

{$ifdef comproutines}
  procedure BCDSubtract ( const BCDIn : tBCD;
                          const Compin : Comp;
                            var BCDout : tBCD ); Inline;

  procedure BCDSubtract ( const Compin : Comp;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDSubtract ( const BCDIn : tBCD;
                          const StringIn : FmtBCDStringtype;
                            var BCDout : tBCD ); Inline;

  procedure BCDSubtract ( const StringIn : FmtBCDStringtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

  procedure BCDSubtract ( const StringIn1,
                                StringIn2 : FmtBCDStringtype;
                          var BCDout : tBCD ); Inline;

  procedure BCDMultiply ( const BCDIn : tBCD;
                          const IntIn : myInttype;
                            var BCDout : tBCD );

  procedure BCDMultiply ( const IntIn : myInttype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

{$ifndef FPUNONE}
  procedure BCDMultiply ( const DoubleIn : myRealtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDMultiply ( const BCDIn : tBCD;
                          const Currin : currency;
                            var BCDout : tBCD ); Inline;

  procedure BCDMultiply ( const Currin : currency;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

{$ifdef comproutines}
  procedure BCDMultiply ( const BCDIn : tBCD;
                          const Compin : Comp;
                            var BCDout : tBCD ); Inline;

  procedure BCDMultiply ( const Compin : Comp;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDMultiply ( const StringIn : FmtBCDStringtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

  procedure BCDDivide ( const Dividend : tBCD;
                        const Divisor : myInttype;
                          var BCDout : tBCD ); Inline;

  procedure BCDDivide ( const Dividend : myInttype;
                        const Divisor : tBCD;
                          var BCDout : tBCD ); Inline;

{$ifndef FPUNONE}
  procedure BCDDivide ( const Dividend : myRealtype;
                        const Divisor : tBCD;
                            var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDDivide ( const BCDIn : tBCD;
                        const Currin : currency;
                          var BCDout : tBCD ); Inline;

  procedure BCDDivide ( const Currin : currency;
                        const BCDIn : tBCD;
                          var BCDout : tBCD ); Inline;

{$ifdef comproutines}
  procedure BCDDivide ( const BCDIn : tBCD;
                        const Compin : Comp;
                          var BCDout : tBCD ); Inline;

  procedure BCDDivide ( const Compin : Comp;
                        const BCDIn : tBCD;
                          var BCDout : tBCD ); Inline;
{$endif}

  procedure BCDDivide ( const Dividend : FmtBCDStringtype;
                        const Divisor : tBCD;
                            var BCDout : tBCD ); Inline;

  operator = ( const BCD1,
                     BCD2 : tBCD ) z : Boolean; Inline;

  operator < ( const BCD1,
                     BCD2 : tBCD ) z : Boolean; Inline;

  operator > ( const BCD1,
                     BCD2 : tBCD ) z : Boolean; Inline;

  operator <= ( const BCD1,
                      BCD2 : tBCD ) z : Boolean; Inline;
  operator >= ( const BCD1,
                      BCD2 : tBCD ) z : Boolean; Inline;

(* ########################            not allowed: why?
  operator + ( const BCD : tBCD ) z : tBCD; make_Inline
##################################################### *)

  operator - ( const BCD : tBCD ) z : tBCD; Inline;

  operator + ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

  operator + ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

  operator + ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifndef FPUNONE}
  operator + ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

  operator + ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator + ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

  operator + ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifdef comproutines}
  operator + ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

  operator + ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator + ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

  operator + ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

  operator - ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

  operator - ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

  operator - ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifndef FPUNONE}
  operator - ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

  operator - ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator - ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

  operator - ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifdef comproutines}
  operator - ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

  operator - ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator - ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

  operator - ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

  operator * ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

  operator * ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

  operator * ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifndef FPUNONE}
  operator * ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

  operator * ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator * ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

  operator * ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifdef comproutines}
  operator * ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

  operator * ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator * ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

  operator * ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

  operator / ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

  operator / ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

  operator / ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifndef FPUNONE}
  operator / ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

  operator / ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator / ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

  operator / ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

{$ifdef comproutines}
  operator / ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

  operator / ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;
{$endif}

  operator / ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

  operator / ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

  operator := ( const i : Byte ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : Byte; Inline;

  operator := ( const i : Word ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : Word; Inline;

  operator := ( const i : longword ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : longword; Inline;

{$if declared ( qword ) }
  operator := ( const i : qword ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : qword; Inline;
{$endif}

  operator := ( const i : ShortInt ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : ShortInt; Inline;

  operator := ( const i : smallint ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : smallint; Inline;

  operator := ( const i : LongInt ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : LongInt; Inline;

{$if declared ( int64 ) }
  operator := ( const i : int64 ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : int64; Inline;
{$endif}

{$ifndef FPUNONE}
  operator := ( const r : Single ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : Single; Inline;

  operator := ( const r : Double ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : Double; Inline;

{$if sizeof ( extended ) <> sizeof ( double )}
  operator := ( const r : Extended ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : Extended; Inline;
{$endif}
{$endif}

  operator := ( const c : currency ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : currency; Inline;

{$ifdef comproutines}
  operator := ( const c : Comp ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : Comp; Inline;
{$endif}

  operator := ( const s : string ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : string; Inline;

  operator := ( const s : AnsiString ) z : tBCD; Inline;

  operator := ( const BCD : tBCD ) z : AnsiString; Inline;

{$endif}

  function __get_null : tBCD; Inline;
  function __get_zero : tBCD; Inline;
  function __get_one : tBCD; Inline;

  PROPERTY
    NullBCD : tBCD Read __get_null;
    ZeroBCD : tBCD Read __get_zero;
    OneBCD : tBCD Read __get_one;

//{$define __lo_bh := 1 * ( -( MaxFmtBCDFractionSize * 1 + 2 ) ) }
//{$define __hi_bh := 1 * ( MaxFmtBCDFractionSize * 1 + 1 ) }

{$define helper_declarations :=

  const
    __lo_bh = -( MaxFmtBCDFractionSize + 2 );
    __hi_bh =  ( MaxFmtBCDFractionSize + 1 );

  type
    tBCD_helper = Maybe_Packed record
                    Prec : {$ifopt r+} 0..( __hi_bh - __lo_bh + 1 ) {$else} Integer {$endif};
                    Plac : {$ifopt r+} 0..( __hi_bh - __lo_bh + 1 ) {$else} Integer {$endif};
                    FDig,
                    LDig : {$ifopt r+} __lo_bh..__hi_bh {$else} Integer {$endif};
                    Singles : Maybe_packed array [ __lo_bh..__hi_bh ]
                                of {$ifopt r+} 0..9 {$else} Byte {$endif};
                    Neg : Boolean;
                   end;
    { in the tBCD_helper the bcd is stored for computations,
      shifted to the right position }

// {$define __lo_bhb := 1 * ( __lo_bh + __lo_bh ) }
// {$define __hi_bhb := 1 * ( __hi_bh + __hi_bh + 1 ) }
  const
    __lo_bhb = __lo_bh + __lo_bh - 1;
    __hi_bhb = __hi_bh + __hi_bh;

  type
    tBCD_helper_big = Maybe_Packed record
                        Prec : {$ifopt r+} 0.. ( __hi_bhb - __lo_bhb + 1 ) {$else} Integer {$endif};
                        Plac : {$ifopt r+} 0.. ( __hi_bhb - __lo_bhb + 1 ) {$else} Integer {$endif};
                        FDig,
                        LDig : {$ifopt r+} __lo_bhb..__hi_bhb {$else} Integer {$endif};
                        Singles : Maybe_packed array [ __lo_bhb..__hi_bhb ]
                                    of {$ifopt r+} 0 * 0..9 * 9 * Pred ( MaxFmtBCDDigits ) {$else} Integer {$endif};
                        Neg : Boolean;
                   end;
}

{$ifdef debug_version}
  helper_declarations

  procedure unpack_BCD ( const BCD : tBCD;
                           var bh : tBCD_helper );
  function pack_BCD ( var bh : tBCD_helper;
                      var BCD : tBCD ) : Boolean;

  procedure dumpBCD ( const v : tBCD );
{$endif}

IMPLEMENTATION

  USES
    classes {$ifopt r+}, sysconst {$endif};

  type
    TFMTBcdFactory = CLASS(TPublishableVarianttype)
    PROTECTED
      function GetInstance(const v : TVarData): tObject; OVERRIDE;
    PUBLIC
      procedure BinaryOp(var Left: TVarData; const Right: TVarData; const Operation: TVarOp); override;
      procedure Clear(var V: TVarData); override;
      procedure Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean); override;
      function CompareOp(const Left, Right: TVarData; const Operation: TVarOp): Boolean; override;
      procedure Compare(const Left, Right: TVarData; var Relationship: TVarCompareResult); override;
      procedure Cast(var Dest: TVarData; const Source: TVarData); override;
      procedure CastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType); override;
    end;

    TFMTBcdVarData = CLASS(TPersistent)
    PRIVATE
      FBcd : tBCD;
    PUBLIC
      constructor create;
      constructor create(const BCD : tBCD);
      PROPERTY BCD : tBCD Read FBcd Write FBcd;
    end;

  var
    NullBCD_ : tBCD;
    OneBCD_ : tBCD;

  function __get_null : tBCD; Inline;
    begin
      __get_null := NullBCD_;
    end;

  function __get_zero : tBCD; Inline;
    begin
      __get_zero := NullBCD_;
      __get_zero.Precision := 1;
    end;

  function __get_one : tBCD; Inline;
    begin
      __get_one := OneBCD_;
    end;

  type
    range_digits = 1..maxfmtbcdfractionsize;
    range_digits0 = 0..maxfmtbcdfractionsize;
    range_fracdigits = 0..pred ( MaxFmtBCDFractionSize );

{$ifopt r+}
  procedure RangeError;
    begin
      raise ERangeError.Create(SRangeError);
    end;
{$endif}

{$ifndef debug_version}
  helper_declarations
{$endif}

  var
    null_ : record
              case Boolean of
                False: ( bh : tBCD_helper );
                True: ( bhb : tBCD_helper_big );
             end;

    FMTBcdFactory : TFMTBcdFactory = NIL;

{$ifndef bigger_BCD}
  const
    NegBit = 1 SHL 7;
    SpecialBit = 1 SHL 6;
    PlacesMask = $ff XOR ( NegBit OR SpecialBit );
{$endif}

{$define _select := {$define _when := if {$define _when := end else if } }
                    {$define _then := then begin }
                    {$define _whenother := end else begin }
                    {$define _endselect := end }  }

{$ifdef debug_version}
  procedure dumpBCD ( const v : tBCD );

    var
      i,
      j : Integer;

    const
      ft : ARRAY [ Boolean ] of Char = ( 'f', 't' );

    begin
{$ifndef bigger_BCD}
      Write ( 'Prec:', v.Precision, ' ',
              'Neg:', ft[( v.SignSpecialPlaces AND NegBit ) <> 0], ' ',
              'Special:', ft[( v.SignSpecialPlaces AND SpecialBit ) <> 0], ' ',
              'Places:', v.SignSpecialPlaces AND PlacesMask, ' ' );
{$else}
      Write ( 'Prec:', v.Precision, ' ',
              'Neg:', ft[v.Negativ], ' ',
              'Places:', v.Places, ' ' );
{$endif}
      j := 0;
      for i := 1 TO v.Precision do
        if Odd ( i )
          then Write ( ( v.Fraction[j] AND $f0 ) SHR 4 )
          else begin
            Write ( v.Fraction[j] AND $0f );
            Inc ( j );
           end;
      WriteLn;
     end;

  procedure dumpbh ( const v : tBCD_helper );

    var
      i : Integer;

    const
      ft : ARRAY [ Boolean ] of Char = ( 'f', 't' );

    begin
      Write ( 'Prec:', v.Prec, ' ',
              'Neg:', ft[v.Neg], ' ',
              'Places:', v.Plac, ' ',
              'FDig:', v.FDig, ' ',
              'LDig:', v.LDig, ' ',
              'Digits:', v.LDig - v.FDig + 1, ' ' );
      for i := v.FDig TO v.LDig do
        Write ( v.Singles[i] );
      WriteLn;
     end;
{$endif}

{$if sizeof ( integer ) = 2 }
  {$ifdef BCDgr4 }
                                  var
                                    myMinIntBCD : tBCD;
  {$endif}
{$else}
  {$if sizeof ( integer ) = 4 }
    {$ifdef BCDgr9 }
                                  var
                                    myMinIntBCD : tBCD;
    {$endif}
  {$else}
    {$if sizeof ( integer ) = 8 }
      {$ifdef BCDgr18 }
                                  var
                                    myMinIntBCD : tBCD;
      {$endif}
    {$else}
      {$fatal You have an interesting integer type! Sorry, not supported}
    {$endif}
  {$endif}
{$endif}

  procedure not_implemented;

    begin
      RAISE eBCDNotImplementedException.create ( 'not implemented' );
     end;

  procedure unpack_BCD ( const BCD : tBCD;
                           var bh : tBCD_helper );

    var
      i : {$ifopt r+} __lo_bh + 1 ..__hi_bh {$else} Integer {$endif};
      j : {$ifopt r+} __low_fraction..__high_fraction+1 {$else} Integer {$endif};
      vv : {$ifopt r+} $00..$99 {$else} Integer {$endif};

    begin
      bh := null_.bh;
      WITH bh,
           BCD do
        begin
          Prec := Precision;
          if Prec > 0
            then begin
{$ifndef bigger_BCD}
              Plac := SignSpecialPlaces AND PlacesMask;
              Neg := ( SignSpecialPlaces AND NegBit ) <> 0;
{$else}
              Plac := Places;
              Neg := Negativ;
{$endif}
              LDig := Plac;
              FDig := LDig - Prec + 1;
              j := 0;
              i := FDig;
              while i <= LDig do
                begin
                  vv := Fraction[j];
                  Singles[i] := ( vv {AND $f0} ) SHR 4;
                  if i < LDig
                    then Singles[i+1] := vv AND $0f;
                  Inc ( j );
                  Inc ( i, 2 );
                end;
             end;
         end;
     end;

  function pack_BCD ( var bh : tBCD_helper;
                      var BCD : tBCD ) : Boolean;
  { return TRUE if successful (BCD valid) }

    var
      pre :    {$ifopt r+} 0..__hi_bh - __lo_bh + 1 {$else} Integer {$endif};
      fra :    {$ifopt r+} -1 * ( __hi_bh - __lo_bh + 1 )..__hi_bh - __lo_bh + 1 {$else} Integer {$endif};
      tm :     {$ifopt r+} 0..__hi_bh - __lo_bh + 1 - Pred ( MaxFmtBCDFractionSize ) {$else} Integer {$endif};
      i :      {$ifopt r+} low ( bh.FDig ) - 1..high ( bh.LDig ) {$else} Integer {$endif};
      rp :     {$ifopt r+} low ( BCD.Fraction )..high ( BCD.Fraction ) + 1 {$else} Integer {$endif};
      ue :     {$ifopt r+} 0..1 {$else} Integer {$endif};
      v :      {$ifopt r+} 0..10 {$else} Integer {$endif};
      lnz :    {$ifopt r+} low ( bh.FDig )..high ( bh.LDig ) {$else} Integer {$endif};
      doround,
      lnzf : Boolean;

    begin
      pack_BCD := False;
      BCD := NullBCD;
      WITH BCD,
           bh do
        begin
          lnzf := FDig <= 0;
          while lnzf do // skip leading 0
            if Singles[FDig] = 0
              then begin
                Inc ( FDig );
                if FDig > 0
                  then lnzf := False;
               end
              else lnzf := False;
          if FDig > 1 then FDig := 1;
          pre := LDig - FDig + 1;
          fra := Plac;
          doround := False;
          if fra >= MaxFmtBCDFractionSize
            then begin
              doround := True;
              tm := fra - Pred ( MaxFmtBCDFractionSize );
{             dec ( pre, tm );   Dec/Inc error? }
              pre := pre - tm;
{             Dec ( fra, tm );   Dec/Inc error? }
              fra := fra - tm;
{             Dec ( LDig, tm );   Dec/Inc error? }
              LDig := LDig - tm;
             end;
          if pre > MaxFmtBCDFractionSize
            then begin
              doround := True;
              tm := pre - MaxFmtBCDFractionSize;
{             Dec ( pre, tm );   Dec/Inc error? }
              pre := pre - tm;
{             Dec ( fra, tm );   Dec/Inc error? }
              fra := fra - tm;
{             Dec ( LDig, tm );   Dec/Inc error? }
              LDig := LDig - tm;
             end;
          if fra < 0
            then EXIT;

          if doround
            then begin
              v := Singles[fra + 1];
              if v > 4
                then begin
                  ue := 1;
                  i := LDig;
                  while ( i >= FDig ) AND ( ue <> 0 ) do
                    begin
                      v := Singles[i] + ue;
                      ue := v DIV 10;
                      Singles[i] := v MOD 10;
                      Dec ( i );
                     end;
                  if ue <> 0
                    then begin
                      Dec ( FDig );
                      Singles[FDig] := ue;
                      Dec ( LDig );
                      Dec ( fra );
                      if fra < 0
                        then EXIT;
                     end;
                 end;
             end;

          lnzf := False;
          i := LDig;
          while ( i >= FDig ) AND ( NOT lnzf ) do // skip trailing 0
            begin
              if Singles[i] <> 0
                then begin
                  lnz := i;
                  lnzf := True;
                 end;
              Dec ( i );
             end;
          if lnzf
            then begin
              tm := LDig - lnz;
              if tm <> 0
                then begin
{                 Dec ( pre, tm );   Dec/Inc error? }
                  pre := pre - tm;
{                 Dec ( fra, tm );   Dec/Inc error? }
                  fra := fra - tm;
{                 Dec ( LDig, tm );   Dec/Inc error? }
                  LDig := LDig - tm;
                  if fra < 0
                    then begin
{                     Dec ( pre, fra );    Dec/Inc error? }
                      pre := pre - fra;
{                     Dec ( LDig, fra );   Dec/Inc error? }
                      LDig := LDig - fra;
                      fra := 0;
                     end;
                 end;
             end
            else begin
              LDig := FDig;
              fra := 0;
              pre := 0;
              Neg := False;
             end;
          if pre <> 0
            then begin
              Precision := pre;
              rp := 0;
              i := FDig;
              while i <= LDig do
                begin
                  if i < LDig
                    then Fraction[rp] := ( Singles[i] SHL 4 ) OR Singles[i + 1]
                    else Fraction[rp] := Singles[i] SHL 4;
                  Inc ( rp );
                  Inc ( i, 2 );
                 end;
{$ifndef bigger_BCD}
              if Neg
                then SignSpecialPlaces := NegBit;
              SignSpecialPlaces := SignSpecialPlaces OR fra;
{$else}
              Negativ := Neg;
              Places := fra;
{$endif}
             end;
         end;
      pack_BCD := True;
     end;

  function BCDPrecision ( const BCD : tBCD ) : Word; Inline;

    begin
      BCDPrecision := BCD.Precision;
     end;

  function BCDScale ( const BCD : tBCD ) : Word; Inline;

    begin
{$ifndef bigger_BCD}
      BCDScale := BCD.SignSpecialPlaces AND PlacesMask;
{$else}
      BCDScale := BCD.Places;
{$endif}
     end;

  function IsBCDNegative ( const BCD : tBCD ) : Boolean; Inline;

    begin
{$ifndef bigger_BCD}
      IsBCDNegative := ( BCD.SignSpecialPlaces AND NegBit ) <> 0;
{$else}
      IsBCDNegative := BCD.Negativ;
{$endif}
     end;

{ BCD Arithmetic}

  procedure BCDNegate ( var BCD : tBCD ); Inline;

    begin
{ with-statement geht nicht !!
      with bcd do
        if precision <> 0
          then signspecialplaces := signspecialplaces xor negbit;
}
        if BCD.Precision <> 0
          then
{$ifndef bigger_BCD}
            BCD.SignSpecialPlaces := BCD.SignSpecialPlaces XOR NegBit;
{$else}
            BCD.Negativ := NOT BCD.Negativ;
{$endif}
     end;

{ returns -1 if BCD1 < BCD2, 0 if BCD1 = BCD2, 1 if BCD1 > BCD2 }
  function BCDCompare ( const BCD1,
                              BCD2 : tBCD ) : Integer;

    var
      pl1 :   {$ifopt r+} 0..maxfmtbcdfractionsize - 1 {$else} Integer {$endif};
      pl2 :   {$ifopt r+} 0..maxfmtbcdfractionsize - 1 {$else} Integer {$endif};
      pr1 :   {$ifopt r+} 0..maxfmtbcdfractionsize {$else} Integer {$endif};
      pr2 :   {$ifopt r+} 0..maxfmtbcdfractionsize {$else} Integer {$endif};
      pr :    {$ifopt r+} 0..maxfmtbcdfractionsize {$else} Integer {$endif};
      idig1 : {$ifopt r+} 0..maxfmtbcdfractionsize {$else} Integer {$endif};
      idig2 : {$ifopt r+} 0..maxfmtbcdfractionsize {$else} Integer {$endif};
      i :     {$ifopt r+} __low_Fraction..__high_Fraction + 1 {$else} Integer {$endif};
      f1 :    {$ifopt r+} $00..$99 {$else} Integer {$endif};
      f2 :    {$ifopt r+} $00..$99 {$else} Integer {$endif};
      res :   {$ifopt r+} -1..1 {$else} Integer {$endif};
      neg1,
      neg2 : Boolean;

    begin
{$ifndef bigger_BCD}
      neg1 := ( BCD1.SignSpecialPlaces AND NegBit ) <> 0;
      neg2 := ( BCD2.SignSpecialPlaces AND NegBit ) <> 0;
{$else}
      neg1 := BCD1.Negativ;
      neg2 := BCD2.Negativ;
{$endif}
      _SELECT
        _WHEN neg1 AND ( NOT neg2 )
          _THEN result := -1;
        _WHEN ( NOT neg1 ) AND neg2
          _THEN result := +1;
        _WHENOTHER
          pr1 := BCD1.Precision;
          pr2 := BCD2.Precision;
{$ifndef bigger_BCD}
          pl1 := BCD1.SignSpecialPlaces AND PlacesMask;
          pl2 := BCD2.SignSpecialPlaces AND PlacesMask;
{$else}
          pl1 := BCD1.Places;
          pl2 := BCD2.Places;
{$endif}
          idig1 := pr1 - pl1;
          idig2 := pr2 - pl2;
          if idig1 <> idig2
            then begin
              if ( idig1 > idig2 ) = neg1
                then result := -1
                else result := +1;
             end
            else begin
              if pr1 < pr2
                then pr := pr1
                else pr := pr2;

              res := 0;
              i := __low_Fraction;
              while ( res = 0 ) AND ( i < ( __low_Fraction + ( pr DIV 2 ) ) ) do
                begin
                  _SELECT
                    _WHEN BCD1.Fraction[i] < BCD2.Fraction[i]
                      _THEN res := -1
                    _WHEN BCD1.Fraction[i] > BCD2.Fraction[i]
                      _THEN res := +1;
                    _WHENOTHER
                   _endSELECT;
                  Inc ( i );
                 end;

              if res = 0
                then begin
                  if Odd ( pr )
                    then begin
                      f1 := BCD1.Fraction[i] AND $f0;
                      f2 := BCD2.Fraction[i] AND $f0;
                      _SELECT
                        _WHEN f1 < f2
                          _THEN res := -1
                        _WHEN f1 > f2
                          _THEN res := +1;
                      _endSELECT;
                     end;

                  if res = 0 then
                    if pr1 > pr2 then
                      res := +1
                    else if pr1 < pr2 then
                      res := -1;
                 end;

              if neg1
                then result := 0 - res
                else result := res;
             end;
       _endSELECT
     end;

{ Convert string/Double/Integer to BCD struct }

  function TryStrToBCD ( const aValue : FmtBCDStringtype;
                           var BCD : tBCD ) : Boolean;
  begin
    Result := TryStrToBCD(aValue, BCD, DefaultFormatSettings);
  end;

  function TryStrToBCD ( const aValue : FmtBCDStringtype;
                           var BCD : tBCD;
                               Const Format : TFormatSettings) : Boolean;
    var
{$ifndef use_ansistring}
      lav : {$ifopt r+} 0..high ( aValue ) {$else} Integer {$endif};
      i   : {$ifopt r+} 0..high ( aValue ) {$else} Integer {$endif};
{$else}
      lav : {$ifopt r+} longword {$else} longword {$endif};
      i   : {$ifopt r+} longword {$else} longword {$endif};
{$endif}
      ch : Char;

    type
      ife = ( inint, infrac, inexp );

{$define max_exp_scanned := 9999 }
    var
      inife : ife;
      lvars : record
                fp,
                lp : ARRAY [ ife ]
{$ifndef use_ansistring}
                       of {$ifopt r+} 0..high ( aValue ) {$else} Integer {$endif};
                pfnb : {$ifopt r+} 0..high ( aValue ) {$else} Integer {$endif};
                ps :   {$ifopt r+} 0..high ( aValue ) {$else} Integer {$endif};
                pse :  {$ifopt r+} 0..high ( aValue ) {$else} Integer {$endif};
                errp : {$ifopt r+} 0..high ( aValue ) {$else} Integer {$endif};
{$else}
                       of {$ifopt r+} longword {$else} longword {$endif};
                pfnb : {$ifopt r+} longword {$else} longword {$endif};
                ps :   {$ifopt r+} longword {$else} longword {$endif};
                pse :  {$ifopt r+} longword {$else} longword {$endif};
                errp : {$ifopt r+} longword {$else} longword {$endif};
{$endif}
                exp :  {$ifopt r+} -max_exp_scanned..max_exp_scanned {$else} Integer {$endif};
                p :    {$ifopt r+} -max_exp_scanned..max_exp_scanned {$else} Integer {$endif};
                bh : tBCD_helper;
                nbf : Boolean;
               end;

    begin
      result := True;
      FillChar ( lvars, SizeOf ( lvars ), #0 );
      BCD := NullBCD;
      lav := Length ( aValue );
      if lav <> 0
        then
          WITH lvars,
               bh do
            begin
              while ( pfnb < lav ) AND ( NOT nbf ) do // skip leading spaces
                begin
                  Inc ( pfnb );
                  nbf := aValue[pfnb] <> ' ';
                 end;
              if nbf
                then begin
                  if aValue[pfnb] IN [ '+', '-' ]
                    then begin
                      ps := pfnb; // position of sign
                      Inc ( pfnb );
                     end;
                  inife := low ( inife );
                  for i := pfnb TO lav do
                    begin
                      ch := aValue[i];
                      case ch of
                        '0'..'9': begin
                                    case inife of
                                      inint,
                                      inexp: if fp[inife] = 0
                                               then begin
                                                 if ch <> '0'
                                                   then begin
                                                     fp[inife] := i;
                                                     lp[inife] := i;
                                                    end;
                                                end
                                               else lp[inife] := i;
                                      infrac: begin
                                                if fp[infrac] = 0
                                                  then fp[infrac] := i;
                                                if ch <> '0'
                                                  then lp[infrac] := i;
                                               end;
                                     end;
                                   end;
                        ',',
                        '.': if ch = Format.DecimalSeparator then
                             begin
                               if inife <> inint then result := False
                               else inife := infrac;
                             end;
                        'e',
                        'E': if inife = inexp
                               then result := False
                               else inife := inexp;
                        '+',
                        '-': if ( inife = inexp ) AND ( fp[inexp] = 0 )
                               then pse := i // position of exponent sign
                               else result := False;
                        else begin
                          result := False;
                          errp := i;
                         end;
                       end;
                     end;
                  if not result
                    then begin
                      result := True;
                      for i := errp TO lav do // skip trailing spaces
                        if aValue[i] <> ' '
                          then result := False;
                     end;
                  if not result
                    then EXIT;

                  if ps <> 0
                    then Neg := aValue[ps] = '-';
                  if lp[infrac] = 0
                    then fp[infrac] := 0;
                  if fp[inexp] <> 0
                    then begin
                      exp := 0;
                      for i := fp[inexp] TO lp[inexp] do
                        if result
                          then
                            if aValue[i] <> Format.ThousandSeparator
                              then begin
                                exp := exp * 10 + ( Ord ( aValue[i] ) - Ord ( '0' ) );
                                if exp > 999
                                  then result := False;
                               end;
                      if not result
                        then EXIT;

                      if pse <> 0
                        then
                          if aValue[pse] = '-'
                            then exp := -exp;
                     end;

                  p := -exp;
                  if fp[infrac] <> 0
                    then begin
                      for i := fp[infrac] TO lp[infrac] do
                        if aValue[i] <> Format.ThousandSeparator
                          then begin
                            if p < ( MaxFmtBCDFractionSize + 2 )
                              then begin
                                Inc ( p );
                                Singles[p] := Ord ( aValue[i] ) - Ord ( '0' );
                               end;
                           end;
                     end;
                  LDig := p;
                  p := 1 - exp;
                  if fp[inint] <> 0
                    then
                      for i := lp[inint] DOWNTO fp[inint] do
                        if aValue[i] <> Format.ThousandSeparator
                          then begin
                            if p > - ( MaxFmtBCDFractionSize + 2 )
                              then begin
                                Dec ( p );
                                Singles[p] := Ord ( aValue[i] ) - Ord ( '0' );
                               end
                              else result := False;
                           end;
                  if not result
                    then EXIT;

                  FDig := p;
                  if LDig < 0
                    then LDig := 0;
                  Plac := LDig;
                  result := pack_BCD ( bh, BCD );
                 end;
             end;
     end;

  function StrToBCD ( const aValue : FmtBCDStringtype ) : tBCD;
  begin
    Result := StrToBCD(aValue, DefaultFormatSettings);
  end;

  function StrToBCD ( const aValue : FmtBCDStringtype;
                            Const Format : TFormatSettings ) : tBCD;
    begin
      if not TryStrToBCD ( aValue, Result, Format ) then
        raise eBCDOverflowException.create ( 'in StrToBCD' );
    end;

{$ifndef FPUNONE}
  procedure DoubleToBCD ( const aValue : myRealtype;
                            var BCD : tBCD );

    var
      s : string [ 30 ];
      f : TFormatSettings;

    begin
      Str ( aValue : 25, s );
      f.DecimalSeparator := '.';
      f.ThousandSeparator := #0;
      BCD := StrToBCD ( s, f );
     end;

  function DoubleToBCD ( const aValue : myRealtype ) : tBCD; Inline;

    begin
      DoubleToBCD ( aValue, result );
     end;
{$endif}

  function IntegerToBCD ( const aValue : myInttype ) : tBCD;

    var
      bh : tBCD_helper;
      v : {$ifopt r+} 0..high ( myInttype ) {$else} Integer {$endif};
      p : {$ifopt r+} low ( bh.Singles ) - 1..0 {$else} Integer {$endif};
      exitloop : Boolean;

    begin
      _SELECT
        _WHEN aValue = 0
          _THEN result := ZeroBCD;
        _WHEN aValue = 1
          _THEN result := OneBCD;
        _WHEN aValue = low ( myInttype )
          _THEN
{$if declared ( myMinIntBCD ) }
            result := myMinIntBCD;
{$else}
            RAISE eBCDOverflowException.create ( 'in IntegerToBCD' );
{$endif}
        _WHENOTHER
          bh := null_.bh;
          WITH bh do
            begin
              Neg := aValue < 0;
              if Neg
                then v := -aValue
                else v := +aValue;
              LDig := 0;
              p := 0;
              REPEAT
                Singles[p] := v MOD 10;
                v := v DIV 10;
                exitloop := v = 0;
                Dec ( p );
                if p < low ( Singles )
                  then begin
                    exitloop := True;
(* what to do if error occurred? *)
                    RAISE eBCDOverflowException.create ( 'in IntegerToBCD' );
                   end;
              UNTIL exitloop;
              FDig := p + 1;
             end;
          pack_BCD ( bh, result );
       _endSELECT;
     end;

  function CurrToBCD ( const Curr : currency;
                         var BCD : tBCD;
                             Precision : Integer = 32;
                             Decimals : Integer = 4 ) : Boolean;

{
  this works under the assumption that a currency is an int64,
  except for scale of 10000
}

    var
      i : int64 absolute Curr;

    begin
      BCD := IntegerToBCD ( i );
{$ifndef bigger_BCD}
      BCD.SignSpecialPlaces := 4 OR ( BCD.SignSpecialPlaces AND NegBit );
{$else}
      BCD.Places := 4;
{$endif}
      if (Decimals <> 4) or (Decimals > BCD.Precision) then
        Result := NormalizeBCD ( BCD, BCD, Precision, Decimals )
      else
        Result := True;
     end;

{$ifdef comproutines}
  function CompToBCD ( const Curr : Comp ) : tBCD; Inline;

    var
      cc : int64 absolute Curr;

    begin
      result := IntegerToBCD ( cc );
     end;

  function BCDToComp ( const BCD : tBCD ) : Comp; Inline;

    var
      zz : record
             case Boolean of
               False: ( i : int64 );
               True: ( c : Comp );
            end;

    begin
      zz.i := BCDToInteger ( BCD );
      BCDToComp := zz.c;
     end;
{$endif}

{ Convert BCD struct to string/Double/Integer }
  function BCDToStr ( const BCD : tBCD ) : FmtBCDStringtype;
  begin
    Result := BCDToStr(BCD, DefaultFormatSettings);
  end;

  function BCDToStr ( const BCD : tBCD;
                            Const Format : TFormatSettings ) : FmtBCDStringtype;
    var
      bh : tBCD_helper;
      l :  {$ifopt r+} 0..maxfmtbcdfractionsize + 1 + 1 {$else} Integer {$endif};
      i :  {$ifopt r+} low ( bh.FDig )..high ( bh.LDig ) {$else} Integer {$endif};
      pp : {$ifopt r+} low ( bh.FDig ) - 1..1 {$else} Integer {$endif};

    begin
{$ifdef use_ansistring}
      result := '';
{$endif}
      unpack_BCD ( BCD, bh );
      WITH bh do
        begin
          l := 0;
          if Neg then
            begin
{$ifndef use_ansistring}
            Inc ( l );
            result[l] := '-';
{$else}
            result := result + '-';
{$endif}
            end;
          if Plac >= Prec then
            begin
            // insert leading 0 before decimal point
{$ifndef use_ansistring}
            Inc ( l );
            result[l] := '0';
{$else}
            result := result + '0';
{$endif}
            end;
          if Prec > 0 then
            begin
            if Plac > 0 then
              begin
              if Plac > Prec then FDig := 1;
              pp := 1;
              end
            else
              pp := low ( bh.FDig ) - 1; // there is no decimal point
            for i := FDig TO LDig do
              begin
              if i = pp then
                begin
{$ifndef use_ansistring}
                Inc ( l );
                result[l] := Format.DecimalSeparator;
{$else}
                result := result + Format.DecimalSeparator;
{$endif}
                end;
{$ifndef use_ansistring}
              Inc ( l );
              result[l] := Chr ( Singles[i] + Ord ( '0' ) );
{$else}
              result := result + Chr ( Singles[i] + Ord ( '0' ) );
{$endif}
              end;
            end;
        end;
{$ifndef use_ansistring}
      result[0] := Chr ( l );
{$endif}
    end;

{$ifndef FPUNONE}
  function BCDToDouble ( const BCD : tBCD ) : myRealtype;

    var
      bh : tBCD_helper;
      i : {$ifopt r+} low ( bh.FDig )..high ( bh.LDig ) {$else} Integer {$endif};
      r,
      e : myRealtype;

    begin
      unpack_BCD ( BCD, bh );
      WITH bh do
        begin
          r := 0;
          e := 1;
          for i := 0 DOWNTO FDig do
            begin
              r := r + Singles[i] * e;
              e := e * 10;
             end;
          e := 1;
          for i := 1 TO LDig do
            begin
              e := e / 10;
              r := r + Singles[i] * e;
             end;
          if Neg
            then BCDToDouble := -r
            else BCDToDouble := +r;
         end;
     end;
{$endif}

  function BCDToInteger ( const BCD : tBCD;
                                Truncate : Boolean = False ) : myInttype;

    var
      bh : tBCD_helper;
      res : myInttype;
      i : {$ifopt r+} low ( bh.FDig )..0 {$else} Integer {$endif};

{
 unclear: behaviour if overflow: abort? return 0? return something?

 so: checks are missing yet
}

    begin
      unpack_BCD ( BCD, bh );
      res := 0;
      WITH bh do
        begin
          for i := FDig TO 0 do
            res := res * 10 - Singles[i];
          if NOT Truncate
            then
              if Plac > 0
                then
                  if Singles[1] > 4
                    then Dec ( res );
          if Neg
            then BCDToInteger := +res
            else BCDToInteger := -res;
         end;
     end;

{ From DB.pas }
  function BCDToCurr ( const BCD : tBCD;
                         var Curr : currency ) : Boolean;

    const
      MaxCurr: array[boolean] of QWord = (QWord($7FFFFFFFFFFFFFFF), QWord($8000000000000000));
    var
      bh : tBCD_helper;
      res : QWord;
      c : currency absolute res;
      i : {$ifopt r+} low ( bh.FDig )..4 {$else} Integer {$endif};

{
 unclear: behaviour if overflow: abort? return 0? return something?
}

    begin
      BCDToCurr := False;
      if BCDPrecision(BCD) - BCDScale(BCD) > 15 then
        Exit;
      unpack_BCD ( BCD, bh );
      res := 0;
      WITH bh do
        begin
          for i := FDig TO 4 do
            res := res * 10 + Singles[i];
          if Plac > 4
            then
              if Singles[5] > 4
                then Inc ( res );
          if res > MaxCurr[Neg] then
            Exit;
          if Neg then
            begin
            res := not res;
            inc(res);
            end;
          Curr := c;
          BCDToCurr := True;
        end;
     end;

  procedure BCDAdd ( const BCDin1,
                           BCDin2 : tBCD;
                       var BCDout : tBCD );

    var
      bhr,
      bh1,
      bh2 : tBCD_helper;
      ue :    {$ifopt r+} 0..1 {$else} Integer {$endif};
      i :     {$ifopt r+} low ( bh1.FDig )..high ( bh1.LDig ) {$else} Integer {$endif};
      v :     {$ifopt r+} 0..9 + 9 + 1 {$else} Integer {$endif};
      BCD : tBCD;
      negate : Boolean;

    begin
      negate := IsBCDNegative ( BCDin1 );
      if negate <> IsBCDNegative ( BCDin2 )
        then begin
          if negate
            then begin
              BCD := BCDin1;
              BCDNegate ( BCD );
              BCDSubtract ( BCDin2, BCD, BCDout );
              EXIT;
             end;

          BCD := BCDin2;
          BCDNegate ( BCD );
          BCDSubtract ( BCDin1, BCD, BCDout );
          EXIT;
         end;

      bhr := null_.bh;
      WITH bhr do
        begin
          unpack_BCD ( BCDin1, bh1 );
          unpack_BCD ( BCDin2, bh2 );
          if bh1.FDig < bh2.FDig
            then FDig := bh1.FDig
            else FDig := bh2.FDig;
          if bh1.LDig > bh2.LDig
            then LDig := bh1.LDig
            else LDig := bh2.LDig;
          Plac := LDig;
          ue := 0;
          for i := LDig DOWNTO FDig do
            begin
              v := bh1.Singles[i] + bh2.Singles[i] + ue;
              ue := v DIV 10;
              Singles[i] := v MOD 10;
             end;
          if ue <> 0
            then begin
              Dec ( FDig );
              Singles[FDig] := ue;
             end;
          Neg := negate;
         end;
      if NOT pack_BCD ( bhr, BCDout )
        then begin
          RAISE eBCDOverflowException.create ( 'in BCDAdd' );
         end;
     end;

  procedure BCDSubtract ( const BCDin1,
                                BCDin2 : tBCD;
                            var BCDout : tBCD );

    var
      bhr,
      bh1,
      bh2 : tBCD_helper;
      cmp : {$ifopt r+} -1..1 {$else} Integer {$endif};
      ue :  {$ifopt r+} 0..1 {$else} Integer {$endif};
      i :   {$ifopt r+} low ( bh1.FDig )..high ( bh1.LDig ) {$else} Integer {$endif};
      v :   {$ifopt r+} 0 - 9 - 1..9 - 0 - 0 {$else} Integer {$endif};
      negate : Boolean;
      BCD : tBCD;

    begin
      negate := IsBCDNegative ( BCDin1 );
      if negate <> IsBCDNegative ( BCDin2 )
        then begin
          if negate
            then begin
              BCD := BCDin1;
              BCDNegate ( BCD );
              BCDAdd ( BCDin2, BCD, BCDout );
              BCDNegate ( BCDout );
              EXIT;
             end;

          BCD := BCDin2;
          BCDNegate ( BCD );
          BCDAdd ( BCDin1, BCD, BCDout );
          EXIT;
         end;

      cmp := BCDCompare ( BCDin1, BCDin2 );
      if cmp = 0
        then begin
          BCDout := NullBCD;
          EXIT;
         end;

      bhr := null_.bh;                    {                      n      n }
      WITH bhr do                          {      >       <       >      < }
        begin                              {                               }
          if ( cmp > 0 ) = negate          {   +123     +12     -12   -123 }
            then begin                     {  - +12  - +123  - -123  - -12 }
              unpack_BCD ( BCDin1, bh2 ); {              x       x        }
              unpack_BCD ( BCDin2, bh1 ); {      s       s       s      s }
              negate := NOT negate;       {     nn       n      nn      n }
             end
            else begin
              unpack_BCD ( BCDin1, bh1 );
              unpack_BCD ( BCDin2, bh2 );
             end;
          if bh1.FDig < bh2.FDig
            then FDig := bh1.FDig
            else FDig := bh2.FDig;
          if bh1.LDig > bh2.LDig
            then LDig := bh1.LDig
            else LDig := bh2.LDig;
          Plac := LDig;
          ue := 0;
          for i := LDig DOWNTO FDig do
            begin
              v := Integer ( bh1.Singles[i] ) - bh2.Singles[i] - ue;
              ue := 0;
              if v < 0
                then begin
                  ue := 1;
                  Inc ( v, 10 );
                 end;
              Singles[i] := v;
             end;
          Neg := negate;
          if NOT pack_BCD ( bhr, BCDout )
            then begin
{should never occur!}
              RAISE eBCDOverflowException.create ( 'in BCDSubtract' );
             end;
         end;
     end;

{ Returns True if successful, False if Int Digits needed to be truncated }
  function NormalizeBCD ( const InBCD : tBCD;
                            var OutBCD : tBCD;
                          const Precision,
                                Places : Integer ) : Boolean;

    var
      bh : tBCD_helper;
      tm : {$ifopt r+} __lo_bh..__hi_bh {$else} Integer {$endif};

    begin
{$ifopt r+}
      if ( Precision < 0 ) OR ( Precision > MaxFmtBCDFractionSize ) then RangeError;
      if ( Places < 0 ) OR ( Precision >= MaxFmtBCDFractionSize ) then RangeError;
{$endif}
      if (BCDScale(InBCD) > Places) or (BCDPrecision(InBCD) < Places) then
        begin
        unpack_BCD ( InBCD, bh );
        tm := bh.Plac - Places;
        bh.Plac := Places;
{       dec ( prec, tm );   Dec/Inc error? }
        bh.Prec := bh.Prec - tm;
{       dec ( LDig, tm );   Dec/Inc error? }
        bh.LDig := bh.LDig - tm;
        NormalizeBCD := tm <= 0;
        if NOT pack_BCD ( bh, OutBCD ) then
          RAISE eBCDOverflowException.Create ( 'in NormalizeBCD' );
        end
      else
        begin
        OutBCD := InBCD;
        NormalizeBCD := True;
        end
    end;

  procedure BCDMultiply ( const BCDin1,
                                BCDin2 : tBCD;
                            var BCDout : tBCD );

    var
      bh1,
      bh2,
      bhr : tBCD_helper;
      bhrr : tBCD_helper_big;
      i1 : {$ifopt r+} low ( bh1.FDig )..high ( bh1.LDig ) {$else} Integer {$endif};
      i2 : {$ifopt r+} low ( bh2.FDig )..high ( bh2.LDig ) {$else} Integer {$endif};
      i3 : {$ifopt r+} low ( bhrr.FDig )..high ( bhrr.LDig ) {$else} Integer {$endif};
      v : {$ifopt r+} low ( bhrr.Singles[0] )..high ( bhrr.Singles[0] ) {$else} Integer {$endif};
      ue : {$ifopt r+} low ( bhrr.Singles[0] ) DIV 10..high ( bhrr.Singles[0] ) DIV 10 {$else} Integer {$endif};

    begin
      unpack_BCD ( BCDin1, bh1 );
      unpack_BCD ( BCDin2, bh2 );
      if ( bh1.Prec = 0 ) OR ( bh2.Prec = 0 )
        then begin
          BCDout := NullBCD;
          EXIT;
         end;

      bhr := null_.bh;
      bhrr := null_.bhb;
      WITH bhrr do
        begin
          Neg := bh1.Neg XOR bh2.Neg;
{
writeln ( __lo_bhb, ' ', __hi_bhb, ' ', bh1.fdig, ' ', bh2.fdig, ' ', low ( fdig ), ' ', low ( ldig ) );
}
          FDig := bh1.FDig + bh2.FDig;
          LDig := bh1.LDig + bh2.LDig;
          for i1 := bh1.FDig TO bh1.LDig do
            for i2 := bh2.FDig TO bh2.LDig do
begin
              Inc ( Singles[i1 + i2],
                    bh1.Singles[i1]
                    * bh2.Singles[i2] );
{
write ( Singles[i1 + i2], ' ', bh1.Singles[i1], ' ', bh2.Singles[i2], ' : ' );
writeln ( Singles[i1 + i2] + bh1.Singles[i1] + bh2.Singles[i2] );
}
{
              Singles[i1 + i2] := Singles[i1 + i2]
                                       + bh1.Singles[i1]
                                         * bh2.Singles[i2];
}
end;
{
for i3 := fdig to ldig do
  write ( ' ', singles[i3] );
writeln;
}
          if FDig < low ( bhr.Singles )
            then RAISE eBCDOverflowException.create ( 'in BCDMultiply' );
          ue := 0;
          for i3 := LDig DOWNTO FDig do
            begin
              v := Singles[i3] + ue;
              ue := v DIV 10;
              v := v MOD 10;
              bhr.Singles[i3] := v;
             end;
          while ue <> 0 do
            begin
              Dec ( FDig );
              if FDig < low ( bhr.Singles )
                then RAISE eBCDOverflowException.create ( 'in BCDMultiply' );
              bhr.Singles[FDig] := ue MOD 10;
              ue := ue DIV 10;
             end;
          bhr.neg := bh1.Neg XOR bh2.Neg;
          bhr.Plac := LDig;
          bhr.FDig := FDig;
          if LDig > high ( bhr.Singles )
            then bhr.LDig := high ( bhr.Singles )
            else bhr.LDig := LDig;
         end;
      if NOT pack_BCD ( bhr, BCDout )
        then begin
          RAISE eBCDOverflowException.create ( 'in BCDMultiply' );
         end;
     end;

{$ifndef FPUNONE}
  procedure BCDMultiply ( const BCDIn : tBCD;
                          const DoubleIn : myRealtype;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( BCDIn, DoubleToBCD ( DoubleIn ), BCDout );
     end;
{$endif}

  procedure BCDMultiply ( const BCDIn : tBCD;
                          const StringIn : FmtBCDStringtype;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( BCDIn, StrToBCD ( StringIn ), BCDout );
     end;

  procedure BCDMultiply ( const StringIn1,
                                StringIn2 : FmtBCDStringtype;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( StrToBCD ( StringIn1 ), StrToBCD ( StringIn2 ), BCDout );
     end;

  procedure BCDDivide ( const Dividend,
                              Divisor : tBCD;
                          var BCDout : tBCD );

    var
      bh1 : ARRAY [ Boolean ] of tBCD_helper;
      bh2,
      bh : tBCD_helper;
      p :     {$ifopt r+} low ( bh.FDig ) - high ( bh.FDig )..high ( bh.FDig ) - low ( bh.FDig ) {$else} Integer {$endif};
      v1 :    {$ifopt r+} low ( bh.Singles[0] )..high ( bh.Singles[0] ) {$else} Integer {$endif};
      v2 :    {$ifopt r+} low ( bh.Singles[0] )..high ( bh.Singles[0] ) {$else} Integer {$endif};
      lFDig : {$ifopt r+} low ( bh.FDig )..high ( bh.FDig ) {$else} Integer {$endif};
      d1 :    {$ifopt r+} low ( bh.LDig ) - high ( bh.FDig )..high ( bh.LDig ) - low ( bh.FDig ) {$else} Integer {$endif};
      d2 :    {$ifopt r+} low ( bh.LDig ) - high ( bh.FDig )..high ( bh.LDig ) - low ( bh.FDig ) {$else} Integer {$endif};
      d :     {$ifopt r+} low ( bh.LDig ) - high ( bh.FDig )..high ( bh.LDig ) - low ( bh.FDig ) {$else} Integer {$endif};
      lLdig : {$ifopt r+} low ( lFDig ) + low ( d )..high ( lFDig ) + high ( d ) {$else} Integer {$endif};
      tm :    {$ifopt r+} low ( lLdig ) - high ( bh2.Singles )..high ( lLdig ) - high ( bh2.Singles ) {$else} Integer {$endif};
      i2 :    {$ifopt r+} low ( lFDig )..high ( lLdig ) {$else} Integer {$endif};
      i3 :    {$ifopt r+} low ( lFDig )..high ( lLdig ) {$else} Integer {$endif};
      ie :    {$ifopt r+} low ( lFDig )..high ( lLdig ) {$else} Integer {$endif};
      i4 :    {$ifopt r+} low ( lFDig )..high ( lLdig ) {$else} Integer {$endif};
      nFDig : {$ifopt r+} low ( i2 )..high ( i2 ) {$else} Integer {$endif};
      nLDig : {$ifopt r+} low ( i2 )..high ( i2 ) {$else} Integer {$endif};
      dd :    {$ifopt r+} 0..9 {$else} Integer {$endif};
      Add :   {$ifopt r+} 0..99 {$else} Integer {$endif};
      ue :    {$ifopt r+} 0..99 {$else} Integer {$endif};
      v3 :    {$ifopt r+} low ( bh.Singles[0] ) - high ( bh2.singles[9] ) * high ( dd ) - high ( ue )..high ( bh.Singles[0] ) - low ( bh2.singles[9] ) * low ( dd ) - low ( ue ) {$else} Integer {$endif};
      v4 :    {$ifopt r+} low ( bh.Singles[0] ) + low ( add )..high ( bh.Singles[0] ) + high ( add ) {$else} Integer {$endif};
      FlipFlop,
      nz,
      sf,
      sh,
      fdset : Boolean;
{
      bh1p : ARRAY [ Boolean ] of ^ tBCD_helper;
}

    begin
{ test:
      bh1p[false] := @ bh1[false];
      bh1p[true] := @ bh1[true];
      v := bh1[false].singles[0];
      v := bh1[true].singles[0];
      v := bh1p[false]^.singles[0];
      v := bh1p[true]^.singles[0];
      v := bh1[nz].singles[0];
      v := bh1p[nz]^.singles[0];
}
      unpack_BCD ( Divisor, bh2 );
      unpack_BCD ( Dividend, bh1[False] );
      p := bh1[False].FDig - bh2.FDig;
      _SELECT
        _WHEN bh2.Prec = 0
          _THEN RAISE eBCDException.create ( 'Division by zero' );
        _WHEN bh1[False].Prec = 0
          _THEN BCDout := NullBCD;
        _WHEN p < low ( bh2.Singles )
          _THEN RAISE eBCDOverflowException.create ( 'in BCDDivide' );
        _WHENOTHER
          bh := null_.bh;
          bh.Neg := bh1[False].Neg XOR bh2.Neg;
          if p <= high ( bh.Singles )
            then begin
              bh1[True] := null_.bh;
              FlipFlop := False;
              fdset := p > 0;
              Add := 0;
              nz := True;
              while nz do
                WITH bh1[FlipFlop] do
                  begin
{
WriteLn('#####');
dumpbh ( bh1[flipflop] );
dumpbh ( bh2 );
dumpbh ( bh );
}
                    if ( Singles[FDig] + bh2.Singles[bh2.FDig] ) = 0
                      then begin
                        if ( FDig >= LDig )
                           OR ( bh2.FDig >= bh2.LDig )
                          then nz := False
                          else begin
                            Inc ( FDig );
                            Inc ( bh2.FDig );
                           end;
                       end
                      else begin
                        v1 := Singles[FDig];
                        v2 := bh2.Singles[bh2.FDig];
                        sh := v1 < v2;
                        if ( v1 = v2 )
                          then begin
                            nz := False;
                            i3 := Succ ( FDig );
                            ie := LDig;
                            while ( i3 <= ie ) AND ( NOT nz ) AND ( NOT sh ) do
                              begin
                                v1 := Singles[i3];
                                v2 := bh2.Singles[i3 - p];
                                if v1 <> v2
                                  then begin
                                    nz := True;
                                    if v1 < v2
                                      then sh := True;
                                   end;
                                Inc ( i3 );
                               end;
                           end;
                        if NOT nz
                          then Add := 1
                          else begin
                            if sh
                              then begin
                                Inc ( p );
{
if p > 3 then halt;
}
                                if p > high ( bh.Singles )
                                  then nz := False
                                  else Dec ( bh2.FDig );
                               end
                              else begin
                                lFDig := FDig;
                                d1 := LDig - FDig;
                                d2 := bh2.LDig - bh2.FDig;
                                if d1 > d2
                                  then d := d1
                                  else d := d2;
                                lLdig := lFDig + d;
                                if lLdig > high ( bh2.Singles )
                                  then begin
                                    tm := ( lLdig ) - high ( bh2.Singles );
                                    d := d - tm;
                                    lLdig := lLdig - tm;
        {runden?}
                                   end;
                                sf := True;
                                Add := 0;
                                nFDig := 0;
                                nLDig := 0;
                                ue := 0;
                                dd := Singles[lFDig] DIV ( bh2.Singles[lFDig - p] + 1 );
                                if dd < 1
                                  then dd := 1;
{
writeln ( 'p=', p, ' dd=', dd, ' lFdig=', lfdig, ' lldig=', lldig );
}
                                                                  
                                for i2 := lLdig DOWNTO lFDig do
                                  begin
                                    // Typecase needed on 64-bit because evaluation happens using qword... 
                                    v3 := Longint(Singles[i2]) - Longint(bh2.Singles[i2 - p] * dd) - Longint(ue);
                                    ue := 0;
                                    while v3 < 0 do
                                      begin
                                        Inc ( ue );;
                                        v3 := v3 + 10;
                                       end;
{
                                    if v3 <> 0
                                      then begin
}
                                        bh1[NOT FlipFlop].Singles[i2] := v3;
{
                                        nFDig := i2;
                                        if sf
                                          then begin
                                            nLDig := i2;
                                            sf := False;
                                           end;
                                       end;
}
                                   end;
                                sf := False;
                                nFDig := lFDig;
                                nLDig := lLDig;
                                Inc ( Add, dd );
                                if sf
                                  then nz := False
                                  else begin
                                    FillChar ( bh1[FlipFlop], SizeOf ( bh1[FlipFlop] ), #0 );
                                    FlipFlop := NOT FlipFlop;
                                    WITH bh1[FlipFlop] do
                                      begin
                                        FDig := nFDig;
                                        LDig := nLDig;
                                       end;
                                   end;
                               end;
                           end;

                        if Add <> 0
                          then begin

                            if NOT fdset
                              then begin
                                bh.FDig := p;
                                fdset := True;
                               end;
                            if bh.LDig < p
                              then begin
                                bh.LDig := p;
                                if ( bh.LDig - bh.FDig ) > Succ ( MaxFmtBCDFractionSize )
                                  then nz := False;
                               end;

                            i4 := p;
                            while ( Add <> 0 ) AND ( i4 >= bh.FDig ) do
                              begin
{
writeln ( '> ', i4, ' ', bh.Singles[i4], ' ', Add );
}
                                v4 := bh.Singles[i4] + Add;
                                Add := v4 DIV 10;
                                bh.Singles[i4] := v4 MOD 10;
                                Dec ( i4 );
                               end;
                            if Add <> 0
                              then begin
                                Dec ( bh.FDig );
                                bh.Singles[bh.FDig] := Add;
                                Add := 0;
                               end;
                           end;
                       end;
                   end;
             end;
          WITH bh do
            begin
              if LDig < 0
                then LDig := 0;
              if LDig > 0
                then Plac := LDig
                else Plac := 0;
             end;
          if NOT pack_BCD ( bh, BCDout )
            then begin
              RAISE eBCDOverflowException.create ( 'in BCDDivide' );
             end;
       _endSELECT
     end;

  procedure BCDDivide ( const Dividend,
                              Divisor : FmtBCDStringtype;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( StrToBCD ( Dividend ), StrToBCD ( Divisor ), BCDout );
     end;

{$ifndef FPUNONE}
  procedure BCDDivide ( const Dividend : tBCD;
                        const Divisor : myRealtype;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( Dividend, DoubleToBCD ( Divisor ), BCDout );
     end;
{$endif}

  procedure BCDDivide ( const Dividend : tBCD;
                        const Divisor : FmtBCDStringtype;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( Dividend, StrToBCD ( Divisor ), BCDout );
     end;

{ TBCD variant creation utils }
  procedure VarFmtBCDCreate (   var aDest : Variant;
                              const aBCD : tBCD );
    begin
      VarClear(aDest);
      TVarData(aDest).Vtype:=FMTBcdFactory.Vartype;
      TVarData(aDest).VPointer:=TFMTBcdVarData.create(aBCD);
    end;

  function VarFmtBCDCreate : Variant;
    begin
      VarFmtBCDCreate ( result, NullBCD );
    end;

  function VarFmtBCDCreate ( const aValue : FmtBCDStringtype;
                                   Precision,
                                   Scale : Word ) : Variant;
    begin
      VarFmtBCDCreate ( result, StrToBCD ( aValue ) );
    end;

{$ifndef FPUNONE}
  function VarFmtBCDCreate ( const aValue : myRealtype;
                                   Precision : Word = 18;
                                   Scale : Word = 4 ) : Variant;

    begin
      VarFmtBCDCreate ( result, DoubleToBCD ( aValue ) );
     end;
{$endif}

  function VarFmtBCDCreate ( const aBCD : tBCD ) : Variant;

    begin
      VarFmtBCDCreate ( result, aBCD );
     end;


  function VarIsFmtBCD ( const aValue : Variant ) : Boolean;
    begin
      Result:=TVarData(aValue).VType=FMTBcdFactory.VarType;
    end;


  function VarFmtBCD : TVartype;
    begin
      Result:=FMTBcdFactory.VarType;
    end;


  { Formatting BCD as string }
  function BCDToStrF ( const BCD : tBCD;
                             Format : TFloatFormat;
                       const Precision,
                             Digits : Integer ) : FmtBCDStringtype;
    var P, E: integer;
        Negative: boolean;
        DS, TS: char;

    procedure RoundDecimalDigits(const d: integer);
    var i,j: integer;
    begin
      j:=P+d;
      if (Length(Result) > j) and (Result[j+1] >= '5') then
        for i:=j downto 1+ord(Negative) do
        begin
          if Result[i] = '9' then
          begin
            Result[i] := '0';
            if i = 1+ord(Negative) then
            begin
              Insert('1', Result, i);
              inc(P);
              inc(j);
            end;
          end
          else if Result[i] <> DS then
          begin
            inc(Result[i]);
            break;
          end;
        end;
      if d = 0 then dec(j); // if decimal separator is last char then do not copy them
      Result := copy(Result, 1, j);
    end;

    procedure AddDecimalDigits(d: integer);
    var n: integer;
    begin
      if P > Length(Result) then // there isn't decimal separator
        if d = 0 then
          Exit
        else
          Result := Result + DS;

      n := d + P - Length(Result);

      if n > 0 then
        Result := Result + StringOfChar('0', n)
      else if n < 0 then
        RoundDecimalDigits(d);
    end;

    procedure AddThousandSeparators;
    begin
      Dec(P, 3);
      While (P > 1) Do
      Begin
        If (Result[P - 1] <> '-') And (TS <> #0) Then
          Insert(TS, Result, P);
        Dec(P, 3);
      End;
    end;

    begin
      Result := BCDToStr(BCD);
      if Format = ffGeneral then Exit;

      DS:=DefaultFormatSettings.DecimalSeparator;
      TS:=DefaultFormatSettings.ThousandSeparator;

      Negative := Result[1] = '-';
      P := Pos(DS, Result);
      if P = 0 then
        P := Length(Result) + 1;

      Case Format Of
        ffExponent:
        Begin
          E := P - 2 - ord(Negative);

          if (E = 0) and (Result[P-1] = '0') then // 0.###
            repeat
              dec(E);
            until (Length(Result) <= P-E) or (Result[P-E] <> '0');

          if E <> 0 then
          begin
            System.Delete(Result, P, 1);
            dec(P, E);
            Insert(DS, Result, P);
          end;

          AddDecimalDigits(Precision-1);

          if E < 0 then
          begin
            System.Delete(Result, P+E-1, -E);
            Result := Result + SysUtils.Format('E%.*d' , [Digits,E])
          end
          else
            Result := Result + SysUtils.Format('E+%.*d', [Digits,E]);
        End;

        ffFixed:
        Begin
          AddDecimalDigits(Digits);
        End;

        ffNumber:
        Begin
          AddDecimalDigits(Digits);
          AddThousandSeparators;
        End;

        ffCurrency:
        Begin
          //implementation based on FloatToStrFIntl()
          if Negative then System.Delete(Result, 1, 1);

          AddDecimalDigits(Digits);
          AddThousandSeparators;

          If Not Negative Then
          Begin
            Case FormatSettings.CurrencyFormat Of
              0: Result := FormatSettings.CurrencyString + Result;
              1: Result := Result + FormatSettings.CurrencyString;
              2: Result := FormatSettings.CurrencyString + ' ' + Result;
              3: Result := Result + ' ' + FormatSettings.CurrencyString;
            End
          End
          Else
          Begin
            Case FormatSettings.NegCurrFormat Of
              0: Result := '(' + FormatSettings.CurrencyString + Result + ')';
              1: Result := '-' + FormatSettings.CurrencyString + Result;
              2: Result := FormatSettings.CurrencyString + '-' + Result;
              3: Result := FormatSettings.CurrencyString + Result + '-';
              4: Result := '(' + Result + FormatSettings.CurrencyString + ')';
              5: Result := '-' + Result + FormatSettings.CurrencyString;
              6: Result := Result + '-' + FormatSettings.CurrencyString;
              7: Result := Result + FormatSettings.CurrencyString + '-';
              8: Result := '-' + Result + ' ' + FormatSettings.CurrencyString;
              9: Result := '-' + FormatSettings.CurrencyString + ' ' + Result;
              10: Result := FormatSettings.CurrencyString + ' ' + Result + '-';
            End;
          End;
        End;
      End;
    end;


  function FormatBCD ( const Format : string;
                             BCD : tBCD ) : FmtBCDStringtype;
    // Tests: tests/test/units/fmtbcd/
    type
      TSection=record
        FmtStart, FmtEnd,      // positions in Format string,
        Fmt1Dig,               // position of 1st digit placeholder,
        FmtDS: PChar;          // position of decimal point
        Digits: integer;       // number of all digit placeholders
        DigDS: integer;        // number of digit placeholders after decimal separator
        HasTS, HasDS: boolean; // has thousand or decimal separator?
      end;

    var
      PFmt: PChar;
      i, j, j1, je, ReqSec, Sec, Scale: integer;
      Section: TSection;
      FF: TFloatFormat;
      BCDStr: string;                   // BCDToStrF of given BCD parameter
      Buf: array [0..85] of char;       // output buffer

    // Parses Format parameter, their sections (positive;negative;zero) and
    //  builds Section information for requested section
    procedure ParseFormat;
    var C,Q: Char;
        PFmtEnd: PChar;
        Section1: TSection;
    begin
      PFmt:=@Format[1];
      PFmtEnd:=PFmt+length(Format);
      Section.FmtStart:=PFmt;
      Section.Fmt1Dig:=nil;
      Section.Digits:=0;
      Section.HasTS:=false; // has thousand separator?
      Section.HasDS:=false; // has decimal separator?
      Sec:=1;
      while true do begin
        if PFmt>=PFmtEnd then
          C:=#0 // hack if short strings used
        else
          C:=PFmt^;
        case C of
          '''', '"':
            begin
            Q:=PFmt^;
            inc(PFmt);
            while (PFmt<PFmtEnd-1) and (PFmt^<>Q) do
              inc(PFmt);
            end;
          #0, ';': // end of Format string or end of section
            begin
            if Sec > 1 then
              Section.FmtStart:=Section.FmtEnd+1;
            Section.FmtEnd:=PFmt;
            if not assigned(Section.Fmt1Dig) then
              Section.Fmt1Dig:=Section.FmtEnd;
            if not Section.HasDS then
              begin
              Section.FmtDS := Section.FmtEnd;
              Section.DigDS := 0;
              end;
            if Sec = 1 then
              Section1 := Section;
            if (C = #0) or (Sec=ReqSec) then
              break;
            Section.Fmt1Dig:=nil;
            Section.Digits:=0;
            Section.HasTS:=false;
            Section.HasDS:=false;
        		inc(Sec);
            end;
          '.':     // decimal point
            begin
            Section.HasDS:=true;
            Section.FmtDS:=PFmt;
            Section.DigDS:=0;
            end;
          ',':     // thousand separator
            Section.HasTS:=true;
          '0','#': // digits placeholders
            begin
            if not assigned(Section.Fmt1Dig) then Section.Fmt1Dig:=PFmt;
            inc(Section.Digits);
            inc(Section.DigDS);
            end;
        end;
        inc(PFmt);
      end;

      // if requested section does not exists or is empty use first section
      if (ReqSec > Sec) or (Section.FmtStart=Section.FmtEnd) then
      begin
        Section := Section1;
        Sec := 1;
      end;
    end;

    procedure PutFmtDigit(var AFmt: PChar; var iBCDStr, iBuf: integer; MoveBy: integer);
    var ADig, Q: Char;
    begin
      if (iBuf < low(Buf)) or (iBuf > high(Buf)) then
        raise eBCDOverflowException.Create ( 'in FormatBCD' );

      if (iBCDStr < 1) or (iBCDStr > length(BCDStr)) then
        ADig:=#0
      else
        ADig:=BCDStr[iBCDStr];

      // write remaining leading part of BCDStr if there are no more digit placeholders in Format string
      if ((AFmt < Section.Fmt1Dig) and (AFmt < Section.FmtDS) and (ADig <> #0)) or
         (ADig = DefaultFormatSettings.ThousandSeparator) then
      begin
        Buf[iBuf] := BCDStr[iBCDStr];
        inc(iBCDStr, MoveBy);
        inc(iBuf, MoveBy);
        Exit;
      end;

      case AFmt^ of
        '''','"':
          begin
          Q:=AFmt^;
          inc(AFmt, MoveBy);
          // write all characters between quotes
          while (AFmt>Section.FmtStart) and (AFmt<Section.FmtEnd) and (AFmt^ <> Q) do
            begin
            Buf[iBuf] := AFmt^;
            inc(AFmt, MoveBy);
            inc(iBuf, MoveBy);
            end;
          end;
        '0','.':
          begin
          if AFmt^ = '.' then
            Buf[iBuf] := DefaultFormatSettings.DecimalSeparator
          else if ADig = #0 then
            Buf[iBuf] := '0'
          else
            Buf[iBuf] := ADig;
          inc(AFmt, MoveBy);
          inc(iBCDStr, MoveBy);
          inc(iBuf, MoveBy);
          end;
        '#':
          begin
          if ADig = #0 then
            inc(AFmt, MoveBy)
          else if (ADig = '0') and (iBCDStr = 1) then // skip leading zero
            begin
            inc(AFmt, MoveBy);
            inc(iBCDStr, MoveBy);
            end
          else
            begin
            Buf[iBuf] := ADig;
            inc(AFmt, MoveBy);
            inc(iBCDStr, MoveBy);
            inc(iBuf, MoveBy);
            end;
          end;
        ',':
          begin
          inc(AFmt, MoveBy); // thousand separators are already in BCDStr
          end;
        else                 // write character what is in Format as is
          begin
          Buf[iBuf] := AFmt^;
          inc(AFmt, MoveBy);
          inc(iBuf, MoveBy);
          end;
      end;
    end;

  begin
    case BCDCompare(BCD, NullBCD) of
       1: ReqSec := 1;
       0: ReqSec := 3;
      -1: ReqSec := 2;
    end;

    // remove sign for negative value
    if ReqSec = 2 then
      BCDNegate(BCD);

    // parse Format into Section
    ParseFormat;

    if Section.FmtStart=Section.FmtEnd then // empty section
      FF := ffGeneral
    else if Section.HasTS then
      FF := ffNumber
    else
      FF := ffFixed;

    Scale := BCDScale(BCD);
    if (FF <> ffGeneral) and (Scale > Section.DigDS) then // we need rounding
      Scale := Section.DigDS;

    BCDStr := BCDToStrF(BCD, FF, 64, Scale);
    if (FF = ffGeneral) then
    begin
      Result:=BCDStr;
      Exit;
    end;

    // write to output buffer
    j1 := high(Buf);   // position of 1st number before decimal point in output buffer
    je := length(Buf); // position after last digit in output buffer
    // output decimal part of BCDStr
    if Section.HasDS and (Section.FmtEnd-Section.FmtDS>1) then // is there something after decimal point?
    begin
      PFmt := Section.FmtDS; // start from decimal point until end
      i := length(BCDStr) - Scale + ord(Scale=0);
      dec(j1, Section.FmtEnd-Section.FmtDS);
      j := j1 + 1;
      while PFmt < Section.FmtEnd do
        PutFmtDigit(PFmt, i, j, 1);
      je := j; // store position after last decimal digit
    end;

    // output whole number part of BCDStr
    PFmt := Section.FmtDS - 1;
    i := length(BCDStr) - Scale - ord(Scale<>0);
    j := j1;
    while (i>0) and (j>0) do
      PutFmtDigit(PFmt, i, j, -1);

    // output leading '0' (f.e. '001.23')
    while (PFmt >= Section.FmtStart) and (PFmt^ = '0') do
      PutFmtDigit(PFmt, i, j, -1);

    // output sign (-), if value is negative, and does not exists 2nd section
    if (ReqSec = 2) and (Sec = 1) then
      begin
      Buf[j]:='-';
      dec(j);
      end;

    // output remaining chars from begining of Format (f.e. 'abc' if given Format is 'abc0.00')
    while PFmt >= Section.FmtStart do
      PutFmtDigit(PFmt, i, j, -1);

    inc(j);
    if j > high(Buf) then
      Result := ''
    else
      SetString(Result, @Buf[j], je-j);
  end;

{$ifdef additional_routines}

  function CurrToBCD ( const Curr : currency ) : tBCD; Inline;

    begin
      CurrToBCD ( Curr, result );
     end;

  procedure BCDAdd ( const BCDIn : tBCD;
                     const IntIn : myInttype;
                       var BCDout : tBCD );

    var
      BCD : tBCD;
      bhr : tBCD_helper;
      p :  {$ifopt r+} low ( bhr.FDig ) - 1..0 {$else} Integer {$endif};
      ue : {$ifopt r+} 0..high ( IntIn ) - 9 {$else} Integer {$endif};
      v :  {$ifopt r+} 0..{high ( ue ) + 9} high ( IntIn ) {$else} Integer {$endif};
      nz : Boolean;

    begin
      if IntIn = 0
        then begin
          BCDout := BCDIn;
          EXIT;
         end;

      if IntIn = low ( myInttype )
        then begin
{$if declared ( myMinIntBCD ) }
          BCDAdd ( BCDIn, myMinIntBCD, BCDout );
          EXIT;
{$else}
          RAISE eBCDOverflowException.create ( 'in BCDAdd' );
{$endif}
         end;

      if IsBCDNegative ( BCDIn )
        then begin
          BCD := BCDIn;
          BCDNegate ( BCD );
          if IntIn < 0
            then BCDAdd ( BCD, -IntIn, BCDout )
            else BCDSubtract ( BCD, IntIn, BCDout );
          BCDNegate ( BCDout );
          EXIT;
         end;

      if IntIn < 0
        then begin
          BCDSubtract ( BCDIn, -IntIn, BCDout );
          EXIT;
         end;

      if IntIn > ( high ( IntIn ) - 9 )
        then begin
          BCDAdd ( BCDIn, IntegerToBCD ( IntIn ), BCDout );
          EXIT;
         end;

      unpack_BCD ( BCDIn, bhr );
      p := 0;
      nz := True;
      ue := IntIn;
      while nz do
        begin
          v := bhr.Singles[p] + ue;
          bhr.Singles[p] := v MOD 10;
          ue := v DIV 10;
          if ue = 0
            then nz := False
            else Dec ( p );
         end;
      if p < bhr.FDig
        then begin
          bhr.FDig := p;
          bhr.Prec := bhr.Prec + ( bhr.FDig - p );
         end;
      if NOT pack_BCD ( bhr, BCDout )
        then begin
          RAISE eBCDOverflowException.create ( 'in BCDAdd' );
         end;
     end;

  procedure BCDSubtract ( const BCDIn : tBCD;
                          const IntIn : myInttype;
                            var BCDout : tBCD );

{}
    var
      BCD : tBCD;
      bhr : tBCD_helper;
      p  : {$ifopt r+} low ( bhr.FDig ) - 1..0 {$else} Integer {$endif};
      ue : {$ifopt r+} 0..pred ( 100000000 ) {$else} Integer {$endif};
      v  : {$ifopt r+} -9..9 {$else} Integer {$endif};
      direct : Boolean;
{}

    begin
      if IntIn = 0
        then begin
          BCDout := BCDIn;
          EXIT;
         end;

      if IntIn = low ( myInttype )
        then begin
{$if declared ( myMinIntBCD ) }
          BCDSubtract ( BCDIn, myMinIntBCD, BCDout );
          EXIT;
{$else}
          RAISE eBCDOverflowException.create ( 'in BCDSubtract' );
{$endif}
         end;

      if IsBCDNegative ( BCDIn )
        then begin
          BCD := BCDIn;
          BCDNegate ( BCD );
          if IntIn < 0
            then BCDSubtract ( BCD, -IntIn, BCDout )
            else BCDAdd ( BCD, IntIn, BCDout );
          BCDNegate ( BCDout );
          EXIT;
         end;

      if IntIn < 0
        then begin
          BCDAdd ( BCDIn, -IntIn, BCDout );
          EXIT;
         end;

      direct := False;
      case BCDIn.Precision
           -
{$ifndef bigger_BCD}
           ( BCDIn.SignSpecialPlaces AND PlacesMask )
{$else}
           BCDIn.Places
{$endif}
          of
        2: direct := IntIn < 10;
        3: direct := IntIn < 100;
        4: direct := IntIn < 1000;
        5: direct := IntIn < 10000;
        6: direct := IntIn < 100000;
        7: direct := IntIn < 1000000;
        8: direct := IntIn < 10000000;
        9: direct := IntIn < 100000000;
       end;
{
write(direct);dumpbcd(bcdin);write('[',intin,']');
}
      if direct
        then begin
          unpack_BCD ( BCDIn, bhr );
          WITH bhr do
            begin
              p := 0;
              ue := IntIn;
              while p >= FDig do
                begin
                  v := Singles[p] - ue MOD 10;
                  ue := ue DIV 10;
                  if v < 0
                    then begin
                      v := v + 10;
                      ue := ue + 1;
                     end;
                  Singles[p] := v;
                  Dec ( p );
                 end;
             end;
          if NOT pack_BCD ( bhr, BCDout )
            then begin
              RAISE eBCDOverflowException.create ( 'in BCDSubtract' );
             end;
         end
        else
{}
        BCDSubtract ( BCDIn, IntegerToBCD ( IntIn ), BCDout );
     end;

  procedure BCDAdd ( const IntIn : myInttype;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( BCDIn, IntIn, BCDout );
     end;

{$ifndef FPUNONE}
  procedure BCDAdd ( const BCDIn : tBCD;
                     const DoubleIn : myRealtype;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( BCDIn, DoubleToBCD ( DoubleIn ), BCDout );
     end;

  procedure BCDAdd ( const DoubleIn : myRealtype;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( DoubleToBCD ( DoubleIn ), BCDIn, BCDout );
     end;
{$endif}

  procedure BCDAdd ( const BCDIn : tBCD;
                     const Currin : currency;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( BCDIn, CurrToBCD ( Currin ), BCDout );
     end;

  procedure BCDAdd ( const Currin : currency;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( CurrToBCD ( Currin ), BCDIn, BCDout );
     end;

{$ifdef comproutines}
  procedure BCDAdd ( const BCDIn : tBCD;
                     const Compin : Comp;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( BCDIn, CompToBCD ( Compin ), BCDout );
     end;

  procedure BCDAdd ( const Compin : Comp;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( CompToBCD ( Compin ), BCDIn, BCDout );
     end;
{$endif}

  procedure BCDAdd ( const BCDIn : tBCD;
                     const StringIn : FmtBCDStringtype;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( BCDIn, StrToBCD ( StringIn ), BCDout );
     end;

  procedure BCDAdd ( const StringIn : FmtBCDStringtype;
                     const BCDIn : tBCD;
                       var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( StrToBCD ( StringIn ), BCDIn, BCDout );
     end;

  procedure BCDAdd ( const StringIn1,
                           StringIn2 : FmtBCDStringtype;
                     var BCDout : tBCD ); Inline;

    begin
      BCDAdd ( StrToBCD ( StringIn1 ), StrToBCD ( StringIn2 ), BCDout );
     end;

  procedure BCDSubtract ( const IntIn : myInttype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( BCDIn, IntIn, BCDout );
      BCDNegate ( BCDout );
     end;

{$ifndef FPUNONE}
  procedure BCDSubtract ( const BCDIn : tBCD;
                          const DoubleIn : myRealtype;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( BCDIn, DoubleToBCD ( DoubleIn ), BCDout );
     end;

  procedure BCDSubtract ( const DoubleIn : myRealtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( DoubleToBCD ( DoubleIn ), BCDIn, BCDout );
     end;
{$endif}

  procedure BCDSubtract ( const BCDIn : tBCD;
                          const Currin : currency;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( BCDIn, CurrToBCD ( Currin ), BCDout );
     end;

  procedure BCDSubtract ( const Currin : currency;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( CurrToBCD ( Currin ), BCDIn, BCDout );
     end;

{$ifdef comproutines}
  procedure BCDSubtract ( const BCDIn : tBCD;
                          const Compin : Comp;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( BCDIn, CompToBCD ( Compin ), BCDout );
     end;

  procedure BCDSubtract ( const Compin : Comp;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( CompToBCD ( Compin ), BCDIn, BCDout );
     end;
{$endif}

  procedure BCDSubtract ( const BCDIn : tBCD;
                          const StringIn : FmtBCDStringtype;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( BCDIn, StrToBCD ( StringIn ), BCDout );
     end;

  procedure BCDSubtract ( const StringIn : FmtBCDStringtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( StrToBCD ( StringIn ), BCDIn, BCDout );
     end;

  procedure BCDSubtract ( const StringIn1,
                                StringIn2 : FmtBCDStringtype;
                            var BCDout : tBCD ); Inline;

    begin
      BCDSubtract ( StrToBCD ( StringIn1 ), StrToBCD ( StringIn2 ), BCDout );
     end;

  procedure BCDMultiply ( const BCDIn : tBCD;
                          const IntIn : myInttype;
                            var BCDout : tBCD );

    var
      bh : tBCD_helper;
      bhr : tBCD_helper;
      bhrr : tBCD_helper_big;
      int : {$ifopt r+} 0..high ( bhrr.Singles[0] ) DIV 10 {$else} Integer {$endif};
      i1 :  {$ifopt r+} low ( bh.Singles )..high ( bh.Singles ) {$else} Integer {$endif};
      i3 :  {$ifopt r+} low ( bhr.Singles )..high ( bhr.Singles ) {$else} Integer {$endif};
      v :   {$ifopt r+} low ( bhrr.Singles[0] ) + low ( bhrr.Singles[0] ) DIV 10..high ( bhrr.Singles[0] ) + high ( bhrr.Singles[0] ) DIV 10 {$else} Integer {$endif};
      ue :  {$ifopt r+} 1 * ( low ( bhrr.Singles[0] ) + low ( bhrr.Singles[0] ) DIV 10 ) DIV 10
                       ..( high ( bhrr.Singles[0] ) + high ( bhrr.Singles[0] ) DIV 10 ) DIV 10 {$else} Integer {$endif};

    begin
      if IntIn = 0
        then begin
          BCDout := NullBCD;
          EXIT;
         end;

      if IntIn = 1
        then begin
          BCDout := BCDIn;
          EXIT;
         end;

      if IntIn = -1
        then begin
          BCDout := BCDIn;
          BCDNegate ( BCDout );
          EXIT;
         end;

      if IntIn = low ( myInttype )
        then begin
{$if declared ( myMinIntBCD ) }
          BCDMultiply ( BCDIn, myMinIntBCD, BCDout );
          EXIT;
{$else}
          RAISE eBCDOverflowException.create ( 'in BCDmultiply' );
{$endif}
         end;

      if Abs ( IntIn ) > low ( bhrr.Singles[0] ) DIV 10
        then begin
          BCDMultiply ( BCDIn, IntegerToBCD ( IntIn ), BCDout );
          EXIT;
         end;

      unpack_BCD ( BCDIn, bh );
      if bh.Prec = 0
        then begin
          BCDout := NullBCD;
          EXIT;
         end;

      bhr := null_.bh;
      bhrr := null_.bhb;
      int := Abs ( IntIn );
      WITH bhrr do
        begin
          Neg := bh.Neg XOR ( IntIn < 0 );
          FDig := bh.FDig;
          LDig := bh.LDig;
          for i1 := bh.FDig TO bh.LDig do
              Singles[i1] := bh.Singles[i1] * int;
{
for i3 := fdig to ldig do
  write ( ' ', singles[i3] );
writeln;
}
          ue := 0;
          for i3 := LDig DOWNTO FDig do
            begin
              v := Singles[i3] + ue;
              ue := v DIV 10;
              v := v MOD 10;
              bhr.Singles[i3] := v;
             end;
          while ue <> 0 do
            begin
              Dec ( FDig );
              if FDig < low ( bhr.Singles )
                then RAISE eBCDOverflowException.create ( 'in BCDMultiply' );
              bhr.Singles[FDig] := ue MOD 10;
              ue := ue DIV 10;
             end;
          bhr.Plac := LDig;
          bhr.FDig := FDig;
          if LDig > high ( bhr.Singles )
            then bhr.LDig := high ( bhr.Singles )
            else bhr.LDig := LDig;
         end;
      if NOT pack_BCD ( bhr, BCDout )
        then begin
          RAISE eBCDOverflowException.create ( 'in BCDMultiply' );
         end;
     end;

  procedure BCDMultiply ( const IntIn : myInttype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( BCDIn, IntIn, BCDout );
     end;

{$ifndef FPUNONE}
  procedure BCDMultiply ( const DoubleIn : myRealtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( DoubleToBCD ( DoubleIn ), BCDIn, BCDout );
     end;
{$endif}

  procedure BCDMultiply ( const BCDIn : tBCD;
                          const Currin : currency;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( BCDIn, CurrToBCD ( Currin ), BCDout );
     end;

  procedure BCDMultiply ( const Currin : currency;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( CurrToBCD ( Currin ), BCDIn, BCDout );
     end;

{$ifdef comproutines}
  procedure BCDMultiply ( const BCDIn : tBCD;
                          const Compin : Comp;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( BCDIn, CompToBCD ( Compin ), BCDout );
     end;

  procedure BCDMultiply ( const Compin : Comp;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( CompToBCD ( Compin ), BCDIn, BCDout );
     end;
{$endif}

  procedure BCDMultiply ( const StringIn : FmtBCDStringtype;
                          const BCDIn : tBCD;
                            var BCDout : tBCD ); Inline;

    begin
      BCDMultiply ( StrToBCD ( StringIn ), BCDIn, BCDout );
     end;

  procedure BCDDivide ( const Dividend : tBCD;
                        const Divisor : myInttype;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( Dividend, IntegerToBCD ( Divisor ), BCDout );
     end;

  procedure BCDDivide ( const Dividend : myInttype;
                        const Divisor : tBCD;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( IntegerToBCD ( Dividend ), Divisor, BCDout );
     end;

{$ifndef FPUNONE}
  procedure BCDDivide ( const Dividend : myRealtype;
                        const Divisor : tBCD;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( DoubleToBCD ( Dividend ), Divisor, BCDout );
     end;
{$endif}

  procedure BCDDivide ( const BCDIn : tBCD;
                        const Currin : currency;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( BCDIn, CurrToBCD ( Currin ), BCDout );
     end;

  procedure BCDDivide ( const Currin : currency;
                        const BCDIn : tBCD;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( CurrToBCD ( Currin ), BCDIn, BCDout );
     end;

{$ifdef comproutines}
  procedure BCDDivide ( const BCDIn : tBCD;
                        const Compin : Comp;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( BCDIn, CompToBCD ( Compin ), BCDout );
     end;

  procedure BCDDivide ( const Compin : Comp;
                        const BCDIn : tBCD;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( CompToBCD ( Compin ), BCDIn, BCDout );
     end;
{$endif}

  procedure BCDDivide ( const Dividend : FmtBCDStringtype;
                        const Divisor : tBCD;
                          var BCDout : tBCD ); Inline;

    begin
      BCDDivide ( StrToBCD ( Dividend ), Divisor, BCDout );
     end;

  operator = ( const BCD1,
                     BCD2 : tBCD ) z : Boolean; Inline;

    begin
      z := BCDCompare ( BCD1, BCD2 ) = 0;
     end;

  operator < ( const BCD1,
                     BCD2 : tBCD ) z : Boolean; Inline;

    begin
      z := BCDCompare ( BCD1, BCD2 ) < 0;
     end;

  operator > ( const BCD1,
                     BCD2 : tBCD ) z : Boolean; Inline;

    begin
      z := BCDCompare ( BCD1, BCD2 ) > 0;
     end;

  operator <= ( const BCD1,
                      BCD2 : tBCD ) z : Boolean; Inline;

    begin
      z := BCDCompare ( BCD1, BCD2 ) <= 0;
     end;

  operator >= ( const BCD1,
                      BCD2 : tBCD ) z : Boolean; Inline;

    begin
      z := BCDCompare ( BCD1, BCD2 ) >= 0;
     end;

(* ########################            not allowed: why?
  operator + ( const BCD : tBCD ) z : tBCD; Inline;

    begin
      z := bcd;
     end;
##################################################### *)

  operator - ( const BCD : tBCD ) z : tBCD; Inline;

    begin
      z := BCD;
      BCDNegate ( z );
     end;

  operator + ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

    begin
      BCDAdd ( BCD1, BCD2, z );
     end;

  operator + ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

    begin
      BCDAdd ( BCD, i, z );
     end;

  operator + ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDAdd ( i, BCD, z );
     end;

{$ifndef FPUNONE}
  operator + ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

    begin
      BCDAdd ( BCD, DoubleToBCD ( r ), z );
     end;

  operator + ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDAdd ( DoubleToBCD ( r ), BCD, z );
     end;
{$endif}

  operator + ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

    begin
      BCDAdd ( BCD, CurrToBCD ( c ), z );
     end;

  operator + ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDAdd ( CurrToBCD ( c ), BCD, z );
     end;

{$ifdef comproutines}
  operator + ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

    begin
      BCDAdd ( BCD, CompToBCD ( c ), z );
     end;

  operator + ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDAdd ( CompToBCD ( c ), BCD, z );
     end;
{$endif}

  operator + ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

    begin
      BCDAdd ( BCD, StrToBCD ( s ), z );
     end;

  operator + ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDAdd ( StrToBCD ( s ), BCD, z );
     end;

  operator - ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

    begin
      BCDSubtract ( BCD1, BCD2, z );
     end;

  operator - ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

    begin
      BCDSubtract ( BCD, i, z );
     end;

  operator - ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDSubtract ( BCD, i, z );
      BCDNegate ( z );
     end;

{$ifndef FPUNONE}
  operator - ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

    begin
      BCDSubtract ( BCD, DoubleToBCD ( r ), z );
     end;

  operator - ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDSubtract ( DoubleToBCD ( r ), BCD, z );
     end;
{$endif}

  operator - ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

    begin
      BCDSubtract ( BCD, CurrToBCD ( c ), z );
     end;

  operator - ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDSubtract ( CurrToBCD ( c ), BCD, z );
     end;

{$ifdef comproutines}
  operator - ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

    begin
      BCDSubtract ( BCD, CompToBCD ( c ), z );
     end;

  operator - ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDSubtract ( CompToBCD ( c ), BCD, z );
     end;
{$endif}

  operator - ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

    begin
      BCDSubtract ( BCD, StrToBCD ( s ), z );
     end;

  operator - ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDSubtract ( StrToBCD ( s ), BCD, z );
     end;

  operator * ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

    begin
      BCDMultiply ( BCD1, BCD2, z );
     end;

  operator * ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

    begin
      BCDMultiply ( BCD, i, z );
     end;

  operator * ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDMultiply ( BCD, i, z );
     end;

{$ifndef FPUNONE}
  operator * ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

    begin
      BCDMultiply ( BCD, DoubleToBCD ( r ), z );
     end;

  operator * ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDMultiply ( DoubleToBCD ( r ), BCD, z );
     end;
{$endif}

  operator * ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

    begin
      BCDMultiply ( BCD, CurrToBCD ( c ), z );
     end;

  operator * ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDMultiply ( CurrToBCD ( c ), BCD, z );
     end;

{$ifdef comproutines}
  operator * ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

    begin
      BCDMultiply ( BCD, CompToBCD ( c ), z );
     end;

  operator * ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDMultiply ( CompToBCD ( c ), BCD, z );
     end;
{$endif}

  operator * ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

    begin
      BCDMultiply ( BCD, StrToBCD ( s ), z );
     end;

  operator * ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDMultiply ( StrToBCD ( s ), BCD, z );
     end;

  operator / ( const BCD1,
                     BCD2 : tBCD ) z : tBCD; Inline;

    begin
      BCDDivide ( BCD1, BCD2, z );
     end;

  operator / ( const BCD : tBCD;
               const i : myInttype ) z : tBCD; Inline;

    begin
      BCDDivide ( BCD, i, z );
     end;

  operator / ( const i : myInttype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDDivide ( IntegerToBCD ( i ), BCD, z );
     end;

{$ifndef FPUNONE}
  operator / ( const BCD : tBCD;
               const r : myRealtype ) z : tBCD; Inline;

    begin
      BCDDivide ( BCD, DoubleToBCD ( r ), z );
     end;

  operator / ( const r : myRealtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDDivide ( DoubleToBCD ( r ), BCD, z );
     end;
{$endif}

  operator / ( const BCD : tBCD;
               const c : currency ) z : tBCD; Inline;

    begin
      BCDDivide ( BCD, CurrToBCD ( c ), z );
     end;

  operator / ( const c : currency;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDDivide ( CurrToBCD ( c ), BCD, z );
     end;

{$ifdef comproutines}
  operator / ( const BCD : tBCD;
               const c : Comp ) z : tBCD; Inline;

    begin
      BCDDivide ( BCD, CompToBCD ( c ), z );
     end;

  operator / ( const c : Comp;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDDivide ( CompToBCD ( c ), BCD, z );
     end;
{$endif}

  operator / ( const BCD : tBCD;
               const s : FmtBCDStringtype ) z : tBCD; Inline;

    begin
      BCDDivide ( BCD, StrToBCD ( s ), z );
     end;

  operator / ( const s : FmtBCDStringtype;
               const BCD : tBCD ) z : tBCD; Inline;

    begin
      BCDDivide ( StrToBCD ( s ), BCD, z );
     end;

  operator := ( const i : Byte ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : Byte; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;

  operator := ( const i : Word ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : Word; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;

  operator := ( const i : longword ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : longword; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;

{$if declared ( qword ) }
  operator := ( const i : qword ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : qword; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;
{$endif}

  operator := ( const i : ShortInt ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : ShortInt; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;

  operator := ( const i : smallint ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : smallint; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;

  operator := ( const i : LongInt ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : LongInt; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;

{$if declared ( int64 ) }
  operator := ( const i : int64 ) z : tBCD; Inline;

    begin
      z := IntegerToBCD ( myInttype ( i ) );
     end;

  operator := ( const BCD : tBCD ) z : int64; Inline;

    begin
      z := BCDToInteger ( BCD );
     end;
{$endif}

{$ifndef FPUNONE}
  operator := ( const r : Single ) z : tBCD; Inline;

    begin
      z := DoubleToBCD ( myRealtype ( r ) );
     end;

  operator := ( const BCD : tBCD ) z : Single; Inline;

    begin
      z := BCDToDouble ( BCD );
     end;

  operator := ( const r : Double ) z : tBCD; Inline;

    begin
      z := DoubleToBCD ( myRealtype ( r ) );
     end;

  operator := ( const BCD : tBCD ) z : Double; Inline;

    begin
      z := BCDToDouble ( BCD );
     end;

{$if sizeof ( extended ) <> sizeof ( double )}
  operator := ( const r : Extended ) z : tBCD; Inline;

    begin
      z := DoubleToBCD ( {myRealtype (} r {)} );
     end;

  operator := ( const BCD : tBCD ) z : Extended; Inline;

    begin
      z := BCDToDouble ( BCD );
     end;
{$endif}
{$endif}

  operator := ( const c : currency ) z : tBCD; Inline;

    begin
      CurrToBCD ( c, z );
     end;

  operator := ( const BCD : tBCD ) z : currency; Inline;

    begin
      BCDToCurr ( BCD, z );
     end;

{$ifdef comproutines}

{$undef makedirect}

{$ifdef makedirect}
  operator := ( const c : Comp ) z : tBCD; Inline;

    var
      cc : int64 absolute c;

    begin
      z := IntegerToBCD ( cc );
     end;

{ $define version1}          { only one of these may be defined! }
{ $define version2}         { version 1 produces a compiler error (with INLINE only!)}
{$define version3}         { I wasn't able to reduce the problem, sorry }

{$ifdef version1}
  operator := ( const BCD : tBCD ) z : Comp; Inline;

    var
      zz : Comp absolute z;

    begin
      zz := BCDToInteger ( BCD );
     end;
{$endif}

{$ifdef version2}
  operator := ( const BCD : tBCD ) z : Comp; Inline;

    var
      zz : int64;
      zzz : Comp absolute zz;

    begin
      zz := BCDToInteger ( BCD );
      z := zzz;
     end;
{$endif}

{$ifdef version3}
  operator := ( const BCD : tBCD ) z : Comp; Inline;

    var
      zz : record
             case Boolean of
               False: ( i : int64 );
               True: ( c : Comp );
            end;

    begin
      zz.i := BCDToInteger ( BCD );
      z := zz.c;
     end;
{$endif}

{$else}
  operator := ( const c : Comp ) z : tBCD; Inline;

    begin
      z := CompToBCD ( c );
     end;

  operator := ( const BCD : tBCD ) z : Comp; Inline;

    begin
      z := BCDToComp ( BCD );
     end;
{$endif}

{$endif}

  operator := ( const s : string ) z : tBCD; Inline;

    begin
      z := StrToBCD ( s );
     end;

  operator := ( const BCD : tBCD ) z : string; Inline;

    begin
      z := BCDToStr ( BCD );
     end;

  operator := ( const s : AnsiString ) z : tBCD; Inline;

    begin
      z := StrToBCD ( s );
     end;

  operator := ( const BCD : tBCD ) z : AnsiString; Inline;

    begin
      z := BCDToStr ( BCD );
     end;

{$endif}


Function VariantToBCD(const VargSrc : TVarData) : TBCD;
begin
  with VargSrc do
    case vType and not varTypeMask of
      0: case vType of
        varEmpty    : Result := 0;
        varSmallInt : Result := vSmallInt;
        varShortInt : Result := vShortInt;
        varInteger  : Result := vInteger;
        varSingle   : Result := vSingle;
        varDouble   : Result := vDouble;
        varCurrency : Result := vCurrency;
        varDate     : Result := vDate;
        varBoolean  : Result := Integer(vBoolean);
        varVariant  : Result := VariantToBCD(PVarData(vPointer)^);
        varByte     : Result := vByte;
        varWord     : Result := vWord;
        varLongWord : Result := vLongWord;
        varInt64    : Result := vInt64;
        varQword    : Result := vQWord;
        varString   : Result := AnsiString(vString);
        varOleStr   : Result := WideString(vOleStr);
        varUString  : Result := UnicodeString(vString);
        else
          if vType=VarFmtBCD then
            Result := TFMTBcdVarData(vPointer).BCD
          else
            not_implemented;
      end;
      varByRef: if Assigned(vPointer) then case vType and varTypeMask of
        varSmallInt : Result := PSmallInt(vPointer)^;
        varShortInt : Result := PShortInt(vPointer)^;
        varInteger  : Result := PInteger(vPointer)^;
        varSingle   : Result := PSingle(vPointer)^;
        varDouble   : Result := PDouble(vPointer)^;
        varCurrency : Result := PCurrency(vPointer)^;
        varDate     : Result := PDate(vPointer)^;
        varBoolean  : Result := SmallInt(PWordBool(vPointer)^);
        varVariant  : Result := VariantToBCD(PVarData(vPointer)^);
        varByte     : Result := PByte(vPointer)^;
        varWord     : Result := PWord(vPointer)^;
        varLongWord : Result := PLongWord(vPointer)^;
        varInt64    : Result := PInt64(vPointer)^;
        varQword    : Result := PQWord(vPointer)^;
      else { other vtype }
        not_implemented;
      end else { pointer is nil }
        not_implemented;
    else { array or something like that }
        not_implemented;
    end;
end;

function VarToBCD ( const aValue : Variant ) : tBCD;
  begin
    Result:=VariantToBCD(TVarData(aValue));
  end;


constructor TFMTBcdVarData.create;
  begin
    inherited create;
    FBcd:=NullBCD;
  end;

constructor TFMTBcdVarData.create(const BCD : tBCD);
  begin
    inherited create;
    FBcd:=BCD;
  end;

function TFMTBcdFactory.GetInstance(const v : TVarData): tObject;
  begin
    result:=tObject(v.VPointer);
  end;


procedure TFMTBcdFactory.BinaryOp(var Left: TVarData; const Right: TVarData; const Operation: TVarOp);
  var l, r: TBCD;
  begin
    l:=VariantToBCD(Left);
    r:=VariantToBCD(Right);

    case Operation of
      opAdd:
        l:=l+r;
      opSubtract:
        l:=l-r;
      opMultiply:
        l:=l*r;
      opDivide:
        l:=l/r;
    else
      RaiseInvalidOp;
    end;

    if Left.vType = VarType then
      TFMTBcdVarData(Left.VPointer).BCD := l
    else if Left.vType = varDouble then
      Left.vDouble := l
    else
      RaiseInvalidOp;
  end;

procedure TFMTBcdFactory.Compare(const Left, Right: TVarData; var Relationship: TVarCompareResult);
  var l, r: TBCD;
      CmpRes: integer;
  begin
    l:=VariantToBCD(Left);
    r:=VariantToBCD(Right);

    CmpRes := BCDCompare(l,r);
    if CmpRes=0 then
      Relationship := crEqual
    else if CmpRes<0 then
      Relationship := crLessThan
    else
      Relationship := crGreaterThan;
  end;

function TFMTBcdFactory.CompareOp(const Left, Right: TVarData; const Operation: TVarOp): Boolean;
  var l, r: TBCD;
  begin
    l:=VariantToBCD(Left);
    r:=VariantToBCD(Right);

    case Operation of
      opCmpEq:
        Result := l=r;
      opCmpNe:
        Result := l<>r;
      opCmpLt:
        Result := l<r;
      opCmpLe:
        Result := l<=r;
      opCmpGt:
        Result := l>r;
      opCmpGe:
        Result := l>=r;
    else
      RaiseInvalidOp;
    end;
  end;
  
procedure TFMTBcdFactory.Clear(var V: TVarData);
  begin
    FreeAndNil(tObject(V.VPointer));
    V.VType:=varEmpty;
  end;

procedure TFMTBcdFactory.Copy(var Dest: TVarData; const Source: TVarData; const Indirect: Boolean);
  begin
    if Indirect then
      Dest.VPointer:=Source.VPointer
    else
      Dest.VPointer:=TFMTBcdVarData.Create(TFMTBcdVarData(Source.VPointer).BCD);
    Dest.VType:=VarType;
  end;

procedure TFMTBcdFactory.Cast(var Dest: TVarData; const Source: TVarData);
begin
  not_implemented;
end;

procedure TFMTBcdFactory.CastTo(var Dest: TVarData; const Source: TVarData; const aVarType: TVarType);
var v: TVarData;
begin
  if Source.vType=VarType then
    if aVarType = varString then
      VarDataFromStr(Dest, BCDToStr(TFMTBcdVarData(Source.vPointer).BCD))
    else
    begin
      VarDataInit(v);
      v.vType:=varDouble;
      v.vDouble:=BCDToDouble(TFMTBcdVarData(Source.vPointer).BCD);
      VarDataCastTo(Dest, v, aVarType); //now cast Double to any requested type
      { finalizing v is not necessary here (Double is a simple type) }
    end
  else
    inherited;
end;

{$if declared ( myMinIntBCD ) }
(*
  {$if sizeof ( integer ) = 2 }
    {$ifdef BCDgr4 }

  const
    myMinIntBCDValue : packed array [ 1..3 ] of Char = #$32#$76#$80;

    {$endif}
  {$else}
    {$if sizeof ( integer ) = 4 }
*)
      {$ifdef BCDgr9 }

  const
    myMinIntBCDValue : packed array [ 1..10 ] of Char = #$21#$47#$48#$36#$48;

      {$endif}
(*
    {$else}
      {$if sizeof ( integer ) = 8 }
        {$ifdef BCDgr18 }

  const
    myMinIntBCDValue : packed array [ 1..19 ] of Char = #$92#$23#$37#$20#$36#$85#$47#$75#$80#$80;

        {$endif}
      {$else}
        {$fatal You have an interesting integer type! Sorry, not supported}
      {$endif}
    {$endif}
  {$endif}
*)
{$endif}

initialization
  FillChar ( null_, SizeOf ( null_ ), #0 );
  FillChar ( NullBCD_, SizeOf ( NullBCD_ ), #0 );
  FillChar ( OneBCD_, SizeOf ( OneBCD_ ), #0 );
  OneBCD_.Precision := 1;
  OneBCD_.Fraction[low ( OneBCD_.Fraction )] := $10;

{$if declared ( myMinIntBCD ) }

  FillChar ( myMinIntBCD, SizeOf ( myMinIntBCD ), #0 );
{$ifndef bigger_BCD}
  myMinIntBCD.SignSpecialPlaces := NegBit;
{$else}
  myMinIntBCD.Negativ := True;
{$endif}

  {$if sizeof ( integer ) = 2 }
    {$ifdef BCDgr4 }

  myMinIntBCD.Precision := 5;
  Move ( myMinIntBCDValue, myMinIntBCD.Fraction, SizeOf ( myMinIntBCDValue ) );

    {$endif}
  {$else}
    {$if sizeof ( integer ) = 4 }
      {$ifdef BCDgr9 }

  myMinIntBCD.Precision := 10;
  Move ( myMinIntBCDValue, myMinIntBCD.Fraction, SizeOf ( myMinIntBCDValue ) );

      {$endif}
    {$else}
      {$if sizeof ( integer ) = 8 }
        {$ifdef BCDgr18 }

  myMinIntBCD.Precision := 19;
  Move ( myMinIntBCDValue, myMinIntBCD.Fraction, SizeOf ( myMinIntBCDValue ) );

        {$endif}
      {$else}
        {$fatal You have an interesting integer type! Sorry, not supported}
      {$endif}
    {$endif}
  {$endif}
{$endif}

  FMTBcdFactory:=TFMTBcdFactory.create;
finalization
  FreeAndNil(FMTBcdFactory)
end.
