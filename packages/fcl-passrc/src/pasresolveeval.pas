{
    This file is part of the Free Component Library

    Pascal source parser
    Copyright (c) 2017 by Mattias Gaertner, mattias@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************

Abstract:
  Evaluation of Pascal constants.

Works:
- Emitting range check warnings
- Error on overflow
- bool:
  - not, =, <>, and, or, xor, low(), high(), pred(), succ(), ord()
  - boolean(0), boolean(1)
- int/uint
  - unary +, -
  - binary: +, -, *, div, mod, ^^, =, <>, <, >, <=, >=, and, or, xor, not, shl, shr
  - Low(), High(), Pred(), Succ(), Ord(), Lo(), Hi()
  - typecast longint(-1), word(-2), intsingle(-1), uintsingle(1)
- float:
  - typecast single(double), double(single), float(integer)
  - +, -, /, *, =, <>, <, >, <=, >=
- string:
  - #65, '', 'a', 'ab'
  - +, =, <>, <, >, <=, >=
  - pred(), succ(), chr(), ord(), low(char), high(char)
  - s[]
  - length(string)
  - #$DC00
  - unicodestring
- enum
  - ord(), low(), high(), pred(), succ()
  - typecast enumtype(integer)
- set of enum, set of char, set of bool, set of int
  - [a,b,c..d]
  - +, -, *, ><, =, <>, >=, <=, in
  - error on duplicate in const set
- arrays
  - length()
  - array of int, charm enum, bool

ToDo:
- arrays
  - [], [a..b], multi dim [a,b], concat with +
  - array of record
  - array of string
  - error on: array[1..2] of longint = (1,2,3);
- anonymous enum range: type f=(a,b,c,d); g=b..c;
}
unit PasResolveEval;

{$mode objfpc}{$H+}

{$ifdef fpc}
  {$define UsePChar}
{$endif}

{$IFOPT Q+}{$DEFINE OverflowCheckOn}{$ENDIF}
{$IFOPT R+}{$DEFINE RangeCheckOn}{$ENDIF}

interface

uses
  Sysutils, Classes, Math, PasTree, PScanner;

// message numbers
const
  nIdentifierNotFound = 3001;
  nNotYetImplemented = 3002;
  nIllegalQualifier = 3003;
  nSyntaxErrorExpectedButFound = 3004;
  nWrongNumberOfParametersForCallTo = 3005;
  nIncompatibleTypeArgNo = 3006;
  nIncompatibleTypeArgNoVarParamMustMatchExactly = 3007;
  nVariableIdentifierExpected = 3008;
  nDuplicateIdentifier = 3009;
  nXExpectedButYFound = 3010;
  nAncestorCycleDetected = 3011;
  nCantUseForwardDeclarationAsAncestor = 3012;
  nCantDetermineWhichOverloadedFunctionToCall = 3013;
  nForwardTypeNotResolved = 3014;
  nForwardProcNotResolved = 3015;
  nInvalidXModifierY = 3016;
  nAbstractMethodsMustNotHaveImplementation = 3017;
  nCallingConventionMismatch = 3018;
  nResultTypeMismatchExpectedButFound = 3019;
  nFunctionHeaderMismatchForwardVarName = 3020;
  nFunctionHidesIdentifier_NonVirtualMethod = 3021;
  nNoMethodInAncestorToOverride = 3022;
  nInheritedOnlyWorksInMethods = 3023;
  nInheritedNeedsAncestor = 3024;
  nNoPropertyFoundToOverride = 3025;
  nExprTypeMustBeClassOrRecordTypeGot = 3026;
  nPropertyNotWritable = 3027;
  nIncompatibleTypesGotExpected = 3028;
  nTypesAreNotRelatedXY = 3029;
  nAbstractMethodsCannotBeCalledDirectly = 3030;
  nMissingParameterX = 3031;
  nInstanceMemberXInaccessible = 3032;
  nInOperatorExpectsSetElementButGot = 3033;
  nWrongNumberOfParametersForTypeCast = 3034;
  nIllegalTypeConversionTo = 3035;
  nConstantExpressionExpected = 3036;
  nLeftSideOfIsOperatorExpectsAClassButGot = 3037;
  nNotReadable = 3038;
  nClassPropertyAccessorMustBeStatic = 3039;
  nClassPropertyAccessorMustNotBeStatic = 3040;
  nOnlyOneDefaultPropertyIsAllowed = 3041;
  nWrongNumberOfParametersForArray = 3042;
  nCantAssignValuesToAnAddress = 3043;
  nIllegalExpression = 3044;
  nCantAccessXMember = 3045;
  nMustBeInsideALoop = 3046;
  nExpectXArrayElementsButFoundY = 3047;
  nCannotCreateADescendantOfTheSealedXY = 3048;
  nAncestorIsNotExternal = 3049;
  nPropertyMustHaveReadOrWrite = 3050;
  nExternalClassInstanceCannotAccessStaticX = 3051;
  nXModifierMismatchY = 3052;
  nSymbolCannotBePublished = 3053;
  nCannotTypecastAType = 3054;
  nTypeIdentifierExpected = 3055;
  nCannotNestAnonymousX = 3056;
  nFoundCallCandidateX = 3057;
  nTextAfterFinalIgnored = 3058;
  nNoMemberIsProvidedToAccessProperty = 3059;
  nTheUseOfXisNotAllowedInARecord = 3060;
  nParameterlessConstructorsNotAllowedInRecords = 3061;
  nMultipleXinTypeYNameZCAandB = 3062;
  nXCannotHaveParameters = 3063;
  nRangeCheckError = 3064;
  nHighRangeLimitLTLowRangeLimit = 3065;
  nRangeCheckEvaluatingConstantsVMinMax = 3066;
  nIllegalChar = 3067;
  nOverflowInArithmeticOperation = 3068;
  nDivByZero = 3069;
  nRangeCheckInSetConstructor = 3070;
  nIncompatibleTypesGotParametersExpected = 3071;
  nAddingIndexSpecifierRequiresNewX = 3072;
  nCantFindUnitX = 3073;
  nCannotFindEnumeratorForType = 3074;
  nPreviousDeclMissesOverload = 3075;
  nOverloadedProcMissesOverload = 3076;
  nMethodHidesMethodOfBaseType = 3077;
  nContextExpectedXButFoundY = 3078;
  nContextXInvalidY = 3079;
  nIdentifierXIsNotAnInstanceField = 3080;
  nXIsNotSupported = 3081;
  nOperatorIsNotOverloadedAOpB = 3082;
  nIllegalQualifierAfter = 3084;
  nIllegalQualifierInFrontOf = 3085;
  nIllegalQualifierWithin = 3086;
  nClassXNotFoundInThisModule = 3087;
  nClassMethodsMustBeStaticInX = 3088;
  nCannotMixMethodResolutionAndDelegationAtX = 3089;
  nImplementsDoesNotSupportArrayProperty = 3101;
  nImplementsDoesNotSupportIndex = 3102;
  nImplementsUsedOnUnimplIntf = 3103;
  nDuplicateImplementsForIntf = 3103;
  nImplPropMustHaveReadSpec = 3104;
  nDoesNotImplementInterface = 3105;
  nTypeCycleFound = 3106;
  nTypeXIsNotYetCompletelyDefined = 3107;
  nDuplicateCaseValueXatY = 3108;
  nMissingFieldsX = 3109;
  nCantAssignValuesToConstVariable = 3110;
  nIllegalAssignmentToForLoopVar = 3111;
  nFunctionHidesIdentifier_NonProc = 3112;
  nTypeXCannotBeExtendedByATypeHelper = 3113;
  nTypeXCannotBeExtendedByARecordHelper = 3114;
  nDerivedXMustExtendASubClassY = 3115;
  nDefaultPropertyNotAllowedInHelperForX = 3116;
  nHelpersCannotBeUsedAsTypes = 3117;
  nMessageHandlersInvalidParams = 3118;
  nImplictConversionUnicodeToAnsi = 3119;
  nWrongTypeXInArrayConstructor = 3120;
  nUnknownCustomAttributeX = 3121;
  nAttributeIgnoredBecauseAbstractX = 3122;
  nCreatingAnInstanceOfAbstractClassY = 3123;
  nIllegalExpressionAfterX = 3124;
  nMethodHidesNonVirtualMethodExactly = 3125;
  nDuplicatePublishedMethodXAtY = 3126;
  nConstraintXSpecifiedMoreThanOnce = 3127;
  nConstraintXAndConstraintYCannotBeTogether = 3128;
  nXIsNotAValidConstraint = 3129;
  nWrongNumberOfParametersForGenericX = 3130;
  nGenericsWithoutSpecializationAsType = 3131;
  nDeclOfXDiffersFromPrevAtY = 3132;
  nTypeParamXIsMissingConstraintY = 3133;
  nTypeParamXIsNotCompatibleWithY = 3134;
  nTypeParamXMustSupportIntfY = 3135;
  nTypeParamsNotAllowedOnX = 3136;
  nXMethodsCannotHaveTypeParams = 3137;
  nImplMustNotRepeatConstraints = 3138;
  nCouldNotInferTypeArgXForMethodY = 3139;
  nInferredTypeXFromDiffArgsMismatchFromMethodY = 3140;
  nParamOfThisTypeCannotHaveDefVal = 3141;

  // using same IDs as FPC
  nVirtualMethodXHasLowerVisibility = 3250; // was 3050
  nConstructingClassXWithAbstractMethodY = 4046; // was 3080
  nNoMatchingImplForIntfMethodXFound = 5042;  // was 3088
  nSymbolXIsDeprecated = 5043;   // was 3062
  nSymbolXBelongsToALibrary = 5065; // was 3061
  nSymbolXIsDeprecatedY = 5066;   // 3063
  nSymbolXIsNotPortable = 5076; // was 3058
  nSymbolXIsNotImplemented = 5078; // was  3060
  nSymbolXIsExperimental = 5079;  // was 3059

// resourcestring patterns of messages
resourcestring
  sIdentifierNotFound = 'identifier not found "%s"';
  sNotYetImplemented = 'not yet implemented: %s';
  sIllegalQualifier = 'illegal qualifier "%s"';
  sSyntaxErrorExpectedButFound = 'Syntax error, "%s" expected but "%s" found';
  sWrongNumberOfParametersForCallTo = 'Wrong number of parameters specified for call to "%s"';
  sIncompatibleTypeArgNo = 'Incompatible type arg no. %s: Got "%s", expected "%s"';
  sIncompatibleTypeArgNoVarParamMustMatchExactly = 'Incompatible type arg no. %s: Got "%s", expected "%s". Var param must match exactly.';
  sVariableIdentifierExpected = 'Variable identifier expected';
  sDuplicateIdentifier = 'Duplicate identifier "%s" at %s';
  sXExpectedButYFound = '%s expected, but %s found';
  sAncestorCycleDetected = 'Ancestor cycle detected';
  sCantUseForwardDeclarationAsAncestor = 'Can''t use forward declaration "%s" as ancestor';
  sCantDetermineWhichOverloadedFunctionToCall = 'Can''t determine which overloaded function to call';
  sForwardTypeNotResolved = 'Forward type not resolved "%s"';
  sForwardProcNotResolved = 'Forward %s not resolved "%s"';
  sInvalidXModifierY = 'Invalid %s modifier %s';
  sAbstractMethodsMustNotHaveImplementation = 'Abstract method must not have an implementation.';
  sCallingConventionMismatch = 'Calling convention mismatch';
  sResultTypeMismatchExpectedButFound = 'Result type mismatch, expected %s, but found %s';
  sFunctionHeaderMismatchForwardVarName = 'function header "%s" doesn''t match forward : var name changes %s => %s';
  sFunctionHidesIdentifier = 'function hides identifier at "%s". Use overload or reintroduce';
  sNoMethodInAncestorToOverride = 'There is no method in an ancestor class to be overridden "%s"';
  sInheritedOnlyWorksInMethods = 'Inherited works only in methods';
  sInheritedNeedsAncestor = 'inherited needs an ancestor';
  sNoPropertyFoundToOverride = 'No property found to override';
  sExprTypeMustBeClassOrRecordTypeGot = 'Expression type must be class or record type, got %s';
  sPropertyNotWritable = 'No member is provided to access property';
  sIncompatibleTypesGotExpected = 'Incompatible types: got "%s" expected "%s"';
  sTypesAreNotRelatedXY = 'Types are not related: "%s" and "%s"';
  sAbstractMethodsCannotBeCalledDirectly = 'Abstract methods cannot be called directly';
  sMissingParameterX = 'Missing parameter %s';
  sInstanceMemberXInaccessible = 'Instance member "%s" inaccessible here';
  sInOperatorExpectsSetElementButGot = 'the in-operator expects a set element, but got %s';
  sWrongNumberOfParametersForTypeCast = 'wrong number of parameters for type cast to %s';
  sIllegalTypeConversionTo = 'Illegal type conversion: "%s" to "%s"';
  sConstantExpressionExpected = 'Constant expression expected';
  sLeftSideOfIsOperatorExpectsAClassButGot = 'left side of is-operator expects a class, but got "%s"';
  sNotReadable = 'not readable';
  sClassPropertyAccessorMustBeStatic = 'class property accessor must be static';
  sClassPropertyAccessorMustNotBeStatic = 'class property accessor must not be static';
  sOnlyOneDefaultPropertyIsAllowed = 'Only one default property is allowed';
  sWrongNumberOfParametersForArray = 'Wrong number of parameters for array';
  sCantAssignValuesToAnAddress = 'Can''t assign values to an address';
  sIllegalExpression = 'Illegal expression';
  sCantAccessXMember = 'Can''t access %s member %s';
  sMustBeInsideALoop = '%s must be inside a loop';
  sExpectXArrayElementsButFoundY = 'Expect %s array elements, but found %s';
  sCannotCreateADescendantOfTheSealedXY = 'Cannot create a descendant of the sealed %s "%s"';
  sAncestorIsNotExternal = 'Ancestor "%s" is not external';
  sPropertyMustHaveReadOrWrite = 'Property must have read or write accessor';
  sVirtualMethodXHasLowerVisibility = 'Virtual method "%s" has a lower visibility (%s) than parent class %s (%s)';
  sExternalClassInstanceCannotAccessStaticX = 'External class instance cannot access static %s';
  sXModifierMismatchY = '%s modifier "%s" mismatch';
  sSymbolCannotBePublished = 'Symbol cannot be published';
  sCannotTypecastAType = 'Cannot type cast a type';
  sTypeIdentifierExpected = 'Type identifier expected';
  sCannotNestAnonymousX = 'Cannot nest anonymous %s';
  sFoundCallCandidateX = 'Found call candidate %s';
  sTextAfterFinalIgnored = 'Text after final ''end.''. ignored by compiler';
  sNoMemberIsProvidedToAccessProperty = 'No member is provided to access property';
  sTheUseOfXisNotAllowedInARecord = 'The use of "%s" is not allowed in a record';
  sParameterlessConstructorsNotAllowedInRecords = 'Parameterless constructors are not allowed in records or record/type helpers';
  sMultipleXinTypeYNameZCAandB = 'Multiple %s in %s %s: %s and %s';
  sXCannotHaveParameters = '%s cannot have parameters';
  sSymbolXIsNotPortable = 'Symbol "%s" is not portable';
  sSymbolXIsExperimental = 'Symbol "%s" is experimental';
  sSymbolXIsNotImplemented = 'Symbol "%s" is not implemented';
  sSymbolXBelongsToALibrary = 'Symbol "%s" belongs to a library';
  sSymbolXIsDeprecated = 'Symbol "%s" is deprecated';
  sSymbolXIsDeprecatedY = 'Symbol "%s" is deprecated: %s';
  sRangeCheckError = 'Range check error';
  sHighRangeLimitLTLowRangeLimit = 'High range limit < low range limit';
  sRangeCheckEvaluatingConstantsVMinMax = 'range check error while evaluating constants (%s is not between %s and %s)';
  sIllegalChar = 'Illegal character';
  sOverflowInArithmeticOperation = 'Overflow in arithmetic operation';
  sDivByZero = 'Division by zero';
  sRangeCheckInSetConstructor = 'range check error in set constructor or duplicate set element';
  sIncompatibleTypesGotParametersExpected = 'Incompatible types, got %s parameters, expected %s';
  sAddingIndexSpecifierRequiresNewX = 'adding index specifier requires new "%s" specifier';
  sCantFindUnitX = 'can''t find unit "%s"';
  sCannotFindEnumeratorForType = 'Cannot find an enumerator for the type "%s"';
  sPreviousDeclMissesOverload = 'Previous declaration of "%s" at %s was not marked with "overload" directive';
  sOverloadedProcMissesOverload = 'Overloaded procedure misses "overload" directive. Previous declaration is at %s';
  sMethodHidesMethodOfBaseType = 'Method "%s" hides method of base type "%s" at %s';
  sContextExpectedXButFoundY = '%s: expected "%s", but found "%s"';
  sContextXInvalidY = '%s: invalid %s';
  sIdentifierXIsNotAnInstanceField = 'Identifier "%s" is not an instance field';
  sConstructingClassXWithAbstractMethodY = 'Constructing a class "%s" with abstract method "%s"';
  sXIsNotSupported = '%s is not supported';
  sOperatorIsNotOverloadedAOpB = 'Operator is not overloaded: "%s" %s "%s"';
  sIllegalQualifierAfter = 'illegal qualifier "%s" after "%s"';
  sIllegalQualifierInFrontOf = 'illegal qualifier "%s" in front of "%s"';
  sIllegalQualifierWithin = 'illegal qualifier "%s" within "%s"';
  sClassXNotFoundInThisModule = 'class "%s" not found in this module';
  sNoMatchingImplForIntfMethodXFound = 'No matching implementation for interface method "%s" found';
  sClassMethodsMustBeStaticInX = 'Class methods must be static in %s';
  sCannotMixMethodResolutionAndDelegationAtX = 'Cannot mix method resolution and delegation at %s';
  sImplementsDoesNotSupportArrayProperty = '"implements" does dot support array property';
  sImplementsDoesNotSupportIndex = '"implements" does not support "index"';
  sImplementsUsedOnUnimplIntf = 'Implements-property used on unimplemented interface: "%"';
  sDuplicateImplementsForIntf = 'Duplicate implements for interface "%s" at %s';
  sImplPropMustHaveReadSpec = 'Implements-property must have read specifier';
  sDoesNotImplementInterface = '"%s" does not implement interface "%s"';
  sTypeCycleFound = 'Type cycle found';
  sTypeXIsNotYetCompletelyDefined = 'type "%s" is not yet completely defined';
  sDuplicateCaseValueXatY = 'Duplicate case value "%s", other at %s';
  sMissingFieldsX = 'Missing fields: "%s"';
  sCantAssignValuesToConstVariable = 'Can''t assign values to const variable';
  sIllegalAssignmentToForLoopVar = 'Illegal assignment to for-loop variable "%s"';
  sTypeXCannotBeExtendedByATypeHelper = 'Type "%s" cannot be extended by a type helper';
  sTypeXCannotBeExtendedByARecordHelper = 'Type "%s" cannot be extended by a record helper';
  sDerivedXMustExtendASubClassY = 'Derived %s must extend a subclass of "%s" or the class itself';
  sDefaultPropertyNotAllowedInHelperForX = 'Default property not allowed in helper for %s';
  sHelpersCannotBeUsedAsTypes = 'helpers cannot be used as types';
  sMessageHandlersInvalidParams = 'Message handlers can take only one call by ref. parameter';
  sImplictConversionUnicodeToAnsi = 'Implicit string type conversion with potential data loss from "UnicodeString" to "AnsiString"';
  sWrongTypeXInArrayConstructor = 'Wrong type "%s" in array constructor';
  sUnknownCustomAttributeX = 'Unknown custom attribute "%s"';
  sAttributeIgnoredBecauseAbstractX = 'attribute ignored because abstract %s';
  sCreatingAnInstanceOfAbstractClassY = 'Creating an instance of abstract class "%s"';
  sIllegalExpressionAfterX = 'illegal expression after %s';
  sMethodHidesNonVirtualMethodExactly = 'method hides identifier at "%s". Use reintroduce';
  sDuplicatePublishedMethodXAtY = 'Duplicate published method "%s" at %s';
  sConstraintXSpecifiedMoreThanOnce = 'Constraint "%s" specified more than once';
  sConstraintXAndConstraintYCannotBeTogether = '"%s" constraint and "%s" constraint cannot be specified together';
  sXIsNotAValidConstraint = '"%s" is not a valid constraint';
  sWrongNumberOfParametersForGenericX = 'wrong number of parameters for generic %s';
  sGenericsWithoutSpecializationAsType = 'Generics without specialization cannot be used as a type for a %s';
  sDeclOfXDiffersFromPrevAtY = 'Declaration of "%s" differs from previous declaration at %s';
  sTypeParamXIsMissingConstraintY = 'Type parameter "%s" is missing constraint "%s"';
  sTypeParamXIsNotCompatibleWithY = 'Type parameter "%s" is not compatible with type "%s"';
  sTypeParamXMustSupportIntfY = 'Type parameter "%s" must support interface "%s"';
  sTypeParamsNotAllowedOnX = 'Type parameters not allowed on %s';
  sXMethodsCannotHaveTypeParams = '%s methods cannot have type parameters';
  sImplMustNotRepeatConstraints = 'Implementations must not repeat constraints';
  sCouldNotInferTypeArgXForMethodY = 'Could not infer generic type argument "%s" for method "%s"';
  sInferredTypeXFromDiffArgsMismatchFromMethodY = 'Inferred type "%s" from different arguments mismatch for method "%s"';
  sParamOfThisTypeCannotHaveDefVal = 'Parameters of this type cannot have default values';

type
  { TResolveData - base class for data stored in TPasElement.CustomData }

  TResolveData = Class(TPasElementBase)
  private
    FElement: TPasElement;
    procedure SetElement(AValue: TPasElement);
  public
    Owner: TObject; // e.g. a TPasResolver
    Next: TResolveData; // TPasResolver uses this for its memory chain
    constructor Create; virtual;
    destructor Destroy; override;
    property Element: TPasElement read FElement write SetElement;// Element.CustomData=Self
  end;
  TResolveDataClass = class of TResolveData;

type
  {$ifdef pas2js}
  TMaxPrecInt = nativeint;
  TMaxPrecUInt = NativeUInt;
  TMaxPrecFloat = double;
  {$else}
  TMaxPrecInt = int64;
  TMaxPrecUInt = qword;
  TMaxPrecFloat = extended;
  {$endif}
  TMaxPrecCurrency = currency;

  {$ifdef fpc}
  PMaxPrecInt = ^TMaxPrecInt;
  PMaxPrecUInt = ^TMaxPrecUInt;
  PMaxPrecFloat = ^TMaxPrecFloat;
  PMaxPrecCurrency = ^TMaxPrecCurrency;
  {$endif}
const
  // Note: when FPC compares int64 with qword it converts the qword to an int64,
  //       possibly resulting in a range check error -> using a qword const instead
  HighIntAsUInt = TMaxPrecUInt(High(TMaxPrecInt));

const
  MinSafeIntCurrency = -922337203685477; // .5808
  MaxSafeIntCurrency =  922337203685477; // .5807
  MinSafeIntSingle = -16777216;
  MaxSafeIntSingle =  16777216;
  MaskUIntSingle = $3fffff;
  MinSafeIntDouble = -$1fffffffffffff; // -9007199254740991 54 bits (52 plus signed bit plus implicit highest bit)
  MaxSafeIntDouble =  $1fffffffffffff; //  9007199254740991
  MaskUIntDouble = $1fffffffffffff;

type
  { TResEvalValue }

  TREVKind = (
    revkNone,
    revkCustom,
    revkNil,  // TResEvalValue
    revkBool, // TResEvalBool
    revkInt,  // TResEvalInt
    revkUInt, // TResEvalUInt
    revkFloat, // TResEvalFloat
    revkCurrency, // TResEvalCurrency
    {$ifdef FPC_HAS_CPSTRING}
    revkString, // TResEvalString  rawbytestring
    {$endif}
    revkUnicodeString, // TResEvalUTF16
    revkEnum,     // TResEvalEnum
    revkRangeInt, // TResEvalRangeInt: range of enum, int, char, widechar, e.g. 1..2
    revkRangeUInt, // TResEvalRangeUInt: range of uint, e.g. 1..2
    revkSetOfInt,  // set of enum, int, char, widechar, e.g. [1,2..3]
    revkExternal // TResEvalExternal: an external const
    );
const
  revkAllStrings = [{$ifdef FPC_HAS_CPSTRING}revkString,{$endif}revkUnicodeString];
type
  TResEvalValue = class(TResolveData)
  public
    Kind: TREVKind;
    IdentEl: TPasElement;
    // Note: "Element" is used when the TResEvalValue is stored as CustomData of an Element
    constructor CreateKind(const aKind: TREVKind);
    function Clone: TResEvalValue; virtual;
    function AsDebugString: string; virtual;
    function AsString: string; virtual;
  end;
  TResEvalValueClass = class of TResEvalValue;

  { TResEvalBool }

  TResEvalBool = class(TResEvalValue)
  public
    B: boolean;
    constructor Create; override;
    constructor CreateValue(const aValue: boolean);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
  end;

  TResEvalTypedInt = (
    reitNone,
    reitByte,
    reitShortInt,
    reitWord,
    reitSmallInt,
    reitUIntSingle,
    reitIntSingle,
    reitLongWord,
    reitLongInt,
    reitUIntDouble,
    reitIntDouble);
  TResEvalTypedInts = set of TResEvalTypedInt;

const
  reitDefaults = [reitNone,reitByte,reitShortInt,reitWord,reitSmallInt,reitLongWord,reitLongInt];
  reitAllSigned = [reitNone,reitShortInt,reitSmallInt,reitIntSingle,reitLongInt,reitIntDouble];
  reitAllUnsigned = [reitByte,reitWord,reitUIntSingle,reitLongWord,reitUIntDouble];

  reitLow: array[TResEvalTypedInt] of TMaxPrecInt = (
    low(TMaxPrecInt), // reitNone,
    low(Byte), // reitByte,
    low(ShortInt), // reitShortInt,
    low(Word), // reitWord,
    low(SmallInt), // reitSmallInt,
    0, // reitUIntSingle,
    MinSafeIntSingle, // reitIntSingle,
    low(LongWord), // reitLongWord,
    low(LongInt), // reitLongInt,
    0, // reitUIntDouble,
    MinSafeIntDouble // reitIntDouble)
    );
  reitHigh: array[TResEvalTypedInt] of TMaxPrecInt = (
    high(TMaxPrecInt), // reitNone,
    high(Byte), // reitByte,
    high(ShortInt), // reitShortInt,
    high(Word), // reitWord,
    high(SmallInt), // reitSmallInt,
    MaxSafeIntSingle, // reitUIntSingle,
    MaxSafeIntSingle, // reitIntSingle,
    high(LongWord), // reitLongWord,
    high(LongInt), // reitLongInt,
    MaxSafeIntDouble, // reitUIntDouble,
    MaxSafeIntDouble // reitIntDouble)
    );

type
  { TResEvalInt }

  TResEvalInt = class(TResEvalValue)
  public
    Int: TMaxPrecInt;
    Typed: TResEvalTypedInt;
    constructor Create; override;
    constructor CreateValue(const aValue: TMaxPrecInt);
    constructor CreateValue(const aValue: TMaxPrecInt; aTyped: TResEvalTypedInt);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
    function AsDebugString: string; override;
  end;

  { TResEvalUInt }

  TResEvalUInt = class(TResEvalValue)
  public
    UInt: TMaxPrecUInt;
    constructor Create; override;
    constructor CreateValue(const aValue: TMaxPrecUInt);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
  end;

  { TResEvalFloat }

  TResEvalFloat = class(TResEvalValue)
  public
    FloatValue: TMaxPrecFloat;
    constructor Create; override;
    constructor CreateValue(const aValue: TMaxPrecFloat);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
    function IsInt(out Int: TMaxPrecInt): boolean;
  end;

  { TResEvalCurrency }

  TResEvalCurrency = class(TResEvalValue)
  public
    Value: TMaxPrecCurrency;
    constructor Create; override;
    constructor CreateValue(const aValue: TMaxPrecCurrency);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
    function IsInt(out Int: TMaxPrecInt): boolean;
    function AsInt: TMaxPrecInt; // value * 10.000
  end;

  {$ifdef FPC_HAS_CPSTRING}
  { TResEvalString - Kind=revkString }

  TResEvalString = class(TResEvalValue)
  public
    S: RawByteString;
    constructor Create; override;
    constructor CreateValue(const aValue: RawByteString);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
  end;
  {$endif}

  { TResEvalUTF16 - Kind=revkUnicodeString }

  TResEvalUTF16 = class(TResEvalValue)
  public
    S: UnicodeString;
    constructor Create; override;
    constructor CreateValue(const aValue: UnicodeString);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
  end;

  { TResEvalEnum - Kind=revkEnum, Value.Int }

  TResEvalEnum = class(TResEvalValue)
  public
    Index: integer; // Beware: might be outside TPasEnumType
    ElType: TPasEnumType; // TPasEnumType
    constructor Create; override;
    constructor CreateValue(const aValue: integer; aIdentEl: TPasEnumValue);
    function GetEnumValue: TPasEnumValue;
    function GetEnumName: String;
    function Clone: TResEvalValue; override;
    function AsDebugString: string; override;
    function AsString: string; override;
  end;

  TRESetElKind = (
    revskNone,
    revskEnum, // ElType is TPasEnumType
    revskInt,
    revskChar,
    revskBool
    );

  { TResEvalRangeInt - Kind=revkRangeInt }

  TResEvalRangeInt = class(TResEvalValue)
  public
    ElKind: TRESetElKind;
    RangeStart, RangeEnd: TMaxPrecInt;
    ElType: TPasType; // revskEnum: TPasEnumType
    constructor Create; override;
    constructor CreateValue(const aElKind: TRESetElKind; aElType: TPasType;
      const aRangeStart, aRangeEnd: TMaxPrecInt); virtual;
    function Clone: TResEvalValue; override;
    function AsString: string; override;
    function AsDebugString: string; override;
    function ElementAsString(El: TMaxPrecInt): string; virtual;
  end;

  { TResEvalRangeUInt }

  TResEvalRangeUInt = class(TResEvalValue)
  public
    RangeStart, RangeEnd: TMaxPrecUInt;
    constructor Create; override;
    constructor CreateValue(const aRangeStart, aRangeEnd: TMaxPrecUInt);
    function Clone: TResEvalValue; override;
    function AsString: string; override;
  end;

  { TResEvalSet - Kind=revkSetOfInt }

  TResEvalSet = class(TResEvalRangeInt)
  public
    const MaxCount = $ffff;
    type
      TItem = record
        RangeStart, RangeEnd: TMaxPrecInt;
      end;
      TItems = array of TItem;
  public
    Ranges: TItems; // disjunct, sorted ascending
    constructor Create; override;
    constructor CreateEmpty(const aElKind: TRESetElKind; aElType: TPasType = nil);
    constructor CreateEmptySameKind(aSet: TResEvalSet);
    constructor CreateValue(const aElKind: TRESetElKind; aElType: TPasType;
      const aRangeStart, aRangeEnd: TMaxPrecInt); override;
    function Clone: TResEvalValue; override;
    function AsString: string; override;
    function Add(aRangeStart, aRangeEnd: TMaxPrecInt): boolean; // false if duplicate ignored
    function IndexOfRange(Index: TMaxPrecInt; FindInsertPos: boolean = false): integer;
    function Intersects(aRangeStart, aRangeEnd: TMaxPrecInt): integer; // returns index of first intersecting range
    procedure ConsistencyCheck;
  end;

  { TResEvalExternal }

  TResEvalExternal = class(TResEvalValue)
  public
    constructor Create; override;
    function Clone: TResEvalValue; override;
    function AsString: string; override;
  end;

  TResEvalFlag = (
    refConst, // computing a const, error if a value is not const
    refConstExt, // as refConst, except allow external const
    refAutoConst, // set refConst if in a const
    refAutoConstExt // set refConstExt if in a const
    );
  TResEvalFlags = set of TResEvalFlag;

  TResExprEvaluator = class;

  TPasResEvalLogHandler = procedure(Sender: TResExprEvaluator; const id: TMaxPrecInt;
    MsgType: TMessageType; MsgNumber: integer;
    const Fmt: String; Args: Array of {$ifdef pas2js}jsvalue{$else}const{$endif}; PosEl: TPasElement) of object;
  TPasResEvalIdentHandler = function(Sender: TResExprEvaluator;
    Expr: TPrimitiveExpr; Flags: TResEvalFlags): TResEvalValue of object;
  TPasResEvalParamsHandler = function(Sender: TResExprEvaluator;
    Params: TParamsExpr; Flags: TResEvalFlags): TResEvalValue of object;
  TPasResEvalRangeCheckElHandler = procedure(Sender: TResExprEvaluator;
    El: TPasElement; var MsgType: TMessageType) of object;

  { TResExprEvaluator }

  TResExprEvaluator = class
  private
    FAllowedInts: TResEvalTypedInts;
    {$ifdef FPC_HAS_CPSTRING}
    FDefaultEncoding: TSystemCodePage;
    {$endif}
    FOnEvalIdentifier: TPasResEvalIdentHandler;
    FOnEvalParams: TPasResEvalParamsHandler;
    FOnLog: TPasResEvalLogHandler;
    FOnRangeCheckEl: TPasResEvalRangeCheckElHandler;
  protected
    procedure LogMsg(const id: TMaxPrecInt; MsgType: TMessageType; MsgNumber: integer;
      const Fmt: String; Args: Array of {$ifdef pas2js}jsvalue{$else}const{$endif}; PosEl: TPasElement); overload;
    procedure RaiseMsg(const Id: TMaxPrecInt; MsgNumber: integer; const Fmt: String;
      Args: Array of {$ifdef pas2js}jsvalue{$else}const{$endif}; ErrorPosEl: TPasElement);
    procedure RaiseNotYetImplemented(id: TMaxPrecInt; El: TPasElement; Msg: string = ''); virtual;
    procedure RaiseInternalError(id: TMaxPrecInt; const Msg: string = '');
    procedure RaiseConstantExprExp(id: TMaxPrecInt; ErrorEl: TPasElement);
    procedure RaiseRangeCheck(id: TMaxPrecInt; ErrorEl: TPasElement);
    procedure RaiseOverflowArithmetic(id: TMaxPrecInt; ErrorEl: TPasElement);
    procedure RaiseDivByZero(id: TMaxPrecInt; ErrorEl: TPasElement);
    function EvalUnaryExpr(Expr: TUnaryExpr; Flags: TResEvalFlags): TResEvalValue;
    function EvalBinaryExpr(Expr: TBinaryExpr; Flags: TResEvalFlags): TResEvalValue;
    function EvalBinaryRangeExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryAddExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinarySubExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryMulExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryDivideExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryDivExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryModExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryPowerExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryShiftExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryBoolOpExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryNEqualExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryLessGreaterExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinaryInExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalBinarySymmetricaldifferenceExpr(Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
    function EvalParamsExpr(Expr: TParamsExpr; Flags: TResEvalFlags): TResEvalValue;
    function EvalArrayParamsExpr(Expr: TParamsExpr; Flags: TResEvalFlags): TResEvalValue;
    function EvalSetParamsExpr(Expr: TParamsExpr; Flags: TResEvalFlags): TResEvalSet;
    function EvalSetExpr(Expr: TPasExpr; ExprArray: TPasExprArray; Flags: TResEvalFlags): TResEvalSet;
    function EvalArrayValuesExpr(Expr: TArrayValues; Flags: TResEvalFlags): TResEvalSet;
    function EvalPrimitiveExprString(Expr: TPrimitiveExpr): TResEvalValue; virtual;
    procedure PredBool(Value: TResEvalBool; ErrorEl: TPasElement);
    procedure SuccBool(Value: TResEvalBool; ErrorEl: TPasElement);
    procedure PredInt(Value: TResEvalInt; ErrorEl: TPasElement);
    procedure SuccInt(Value: TResEvalInt; ErrorEl: TPasElement);
    procedure PredUInt(Value: TResEvalUInt; ErrorEl: TPasElement);
    procedure SuccUInt(Value: TResEvalUInt; ErrorEl: TPasElement);
    {$ifdef FPC_HAS_CPSTRING}
    procedure PredString(Value: TResEvalString; ErrorEl: TPasElement);
    procedure SuccString(Value: TResEvalString; ErrorEl: TPasElement);
    {$endif}
    procedure PredUnicodeString(Value: TResEvalUTF16; ErrorEl: TPasElement);
    procedure SuccUnicodeString(Value: TResEvalUTF16; ErrorEl: TPasElement);
    procedure PredEnum(Value: TResEvalEnum; ErrorEl: TPasElement);
    procedure SuccEnum(Value: TResEvalEnum; ErrorEl: TPasElement);
    function CreateResEvalInt(UInt: TMaxPrecUInt): TResEvalValue; virtual;
  public
    constructor Create;
    function Eval(Expr: TPasExpr; Flags: TResEvalFlags): TResEvalValue;
    function IsInRange(Expr, RangeExpr: TPasExpr; EmitHints: boolean): boolean;
    function IsInRange(Value: TResEvalValue; ValueExpr: TPasExpr;
      RangeValue: TResEvalValue; RangeExpr: TPasExpr; EmitHints: boolean): boolean;
    function IsSetCompatible(Value: TResEvalValue; ValueExpr: TPasExpr;
      RangeValue: TResEvalValue; EmitHints: boolean): boolean;
    function IsConst(Expr: TPasExpr): boolean;
    function IsSimpleExpr(Expr: TPasExpr): boolean; // true = no need to store result
    procedure EmitRangeCheckConst(id: TMaxPrecInt; const aValue, MinVal, MaxVal: String;
      PosEl: TPasElement; MsgType: TMessageType = mtWarning); virtual;
    procedure EmitRangeCheckConst(id: TMaxPrecInt; const aValue: String;
      MinVal, MaxVal: TMaxPrecInt; PosEl: TPasElement; MsgType: TMessageType = mtWarning);
    function ChrValue(Value: TResEvalValue; ErrorEl: TPasElement): TResEvalValue; virtual;
    function OrdValue(Value: TResEvalValue; ErrorEl: TPasElement): TResEvalValue; virtual;
    function StringToOrd(Value: TResEvalValue; PosEl: TPasElement): longword; virtual;
    procedure PredValue(Value: TResEvalValue; ErrorEl: TPasElement); virtual;
    procedure SuccValue(Value: TResEvalValue; ErrorEl: TPasElement); virtual;
    function EvalStrFunc(Params: TParamsExpr; Flags: TResEvalFlags): TResEvalValue; virtual;
    function EvalStringAddExpr(Expr, LeftExpr, RightExpr: TPasExpr;
      LeftValue, RightValue: TResEvalValue): TResEvalValue; virtual;
    function LoHiValue(Value: TResEvalValue; ShiftSize: Integer; Mask: LongWord;
      ErrorEl: TPasElement): TResEvalValue; virtual;
    function EnumTypeCast(EnumType: TPasEnumType; Expr: TPasExpr;
      Flags: TResEvalFlags): TResEvalEnum; virtual;
    {$ifdef FPC_HAS_CPSTRING}
    function CheckValidUTF8(const s: RawByteString; ErrorEl: TPasElement): boolean;
    function GetCodePage(const s: RawByteString): TSystemCodePage;
    function GetRawByteString(const s: UnicodeString; CodePage: TSystemCodePage; ErrorEl: TPasElement): RawByteString;
    function GetUTF8Str(const s: RawByteString; ErrorEl: TPasElement): String;
    function GetUnicodeStr(const s: RawByteString; ErrorEl: TPasElement): UnicodeString;
    function GetWideChar(const s: RawByteString; out w: WideChar): boolean;
    {$endif}
    property OnLog: TPasResEvalLogHandler read FOnLog write FOnLog;
    property OnEvalIdentifier: TPasResEvalIdentHandler read FOnEvalIdentifier write FOnEvalIdentifier;
    property OnEvalParams: TPasResEvalParamsHandler read FOnEvalParams write FOnEvalParams;
    property OnRangeCheckEl: TPasResEvalRangeCheckElHandler read FOnRangeCheckEl write FOnRangeCheckEl;
    property AllowedInts: TResEvalTypedInts read FAllowedInts write FAllowedInts;
    {$ifdef FPC_HAS_CPSTRING}
    property DefaultStringCodePage: TSystemCodePage read FDefaultEncoding write FDefaultEncoding;
    {$endif}
  end;
  TResExprEvaluatorClass = class of TResExprEvaluator;

procedure ReleaseEvalValue(var Value: TResEvalValue);
function NumberIsFloat(const Value: string): boolean;

{$ifdef FPC_HAS_CPSTRING}
function RawStrToCaption(const r: RawByteString; MaxLength: integer): string;
{$endif}
function UnicodeStrToCaption(const u: UnicodeString; MaxLength: integer): Unicodestring;
function CodePointToString(CodePoint: longword): String;
function CodePointToUnicodeString(u: longword): UnicodeString;

function GetObjName(o: TObject): string;
function GetObjPath(o: TObject): string;
function GetGenericParamCommas(Cnt: integer): string;
function GetElementNameAndParams(El: TPasElement; MaxLvl: integer = 3): string;
function GetTypeParamNames(Templates: TFPList; MaxLvl: integer = 3): string;
function dbgs(const Flags: TResEvalFlags): string; overload;
function dbgs(v: TResEvalValue): string; overload;
function LastPos(c: char; const s: string): sizeint;

implementation

procedure ReleaseEvalValue(var Value: TResEvalValue);
begin
  if Value=nil then exit;
  if Value.Element<>nil then exit;
  Value.{$ifdef pas2js}Destroy{$else}Free{$endif};
  Value:=nil;
end;

function NumberIsFloat(const Value: string): boolean;
var
  i: Integer;
begin
  if Value='' then exit(false);
  if Value[1] in ['$','%','&'] then exit(false);
  for i:=2 to length(Value) do
    if Value[i] in ['.','E','e'] then exit(true);
  Result:=false;
end;

{$ifdef FPC_HAS_CPSTRING}
function RawStrToCaption(const r: RawByteString; MaxLength: integer): string;
var
  s: RawByteString;
  p: PAnsiChar;
  InLit: boolean;
  Len: integer;

  procedure AddHash(o: integer);
  var
    h: String;
  begin
    if (Result<>'') and InLit then
      begin
      Result:=Result+'''';
      inc(Len);
      InLit:=false;
      end;
    h:='#'+IntToStr(o);
    inc(Len,length(h));
    if Len<=MaxLength then
      Result:=Result+h;
  end;

  procedure AddLit(const Lit: string; CaptionLen: integer);
  begin
    if not InLit then
      begin
      Result:=Result+'''';
      inc(Len);
      InLit:=true;
      end;
    Result:=Result+Lit;
    inc(Len,CaptionLen);
  end;

var
  l: SizeInt;
  CP: TSystemCodePage;
  EndP: PAnsiChar;
begin
  Result:='';
  s:=r;
  CP:=StringCodePage(s);
  if (CP<>CP_ACP) and (CP<>CP_UTF8) then
    SetCodePage(s, CP_ACP, true);
  p:=PAnsiChar(s);
  EndP:=p+length(s);
  Len:=0;
  InLit:=false;
  while Len<MaxLength do
    case p^ of
    #0:
      begin
      if p-PAnsiChar(s)=length(s) then
        break;
      AddHash(0);
      inc(p);
      end;
    '''':
      begin
      AddLit('''''',2);
      inc(p);
      end;
    #1..#31,#127..#192:
      begin
      AddHash(ord(p^));
      inc(p);
      end
    else
      begin
      l:=Utf8CodePointLen(p,EndP-p,true);
      if l<=0 then
        begin
        // invalid
        AddHash(ord(p^));
        inc(p);
        end
      else
        begin
        AddLit(copy(s,p-PAnsiChar(s)+1,l),1);
        inc(p,l);
        end;
      end;
    end;
  if InLit then
    Result:=Result+'''';
end;
{$endif}

function UnicodeStrToCaption(const u: UnicodeString; MaxLength: integer
  ): Unicodestring;
var
  InLit: boolean;
  Len: integer;

  procedure AddHash(o: integer);
  var
    h: UnicodeString;
  begin
    if (Result<>'') and InLit then
      begin
      Result:=Result+'''';
      inc(Len);
      InLit:=false;
      end;
    h:='#'+UnicodeString(IntToStr(o));
    inc(Len,length(h));
    if Len<=MaxLength then
      Result:=Result+h;
  end;

  procedure AddLit(const Lit: Unicodestring; CaptionLen: integer);
  begin
    if not InLit then
      begin
      Result:=Result+'''';
      inc(Len);
      InLit:=true;
      end;
    Result:=Result+Lit;
    inc(Len,CaptionLen);
  end;

var
  p: integer;
begin
  Result:='';
  p:=1;
  Len:=0;
  InLit:=false;
  while (Len<MaxLength) and (p<=length(u)) do
    case u[p] of
    '''':
      begin
      AddLit('''''',2);
      inc(p);
      end;
    #0..#31,#127..#255,#$D800..#$ffff:
      begin
      AddHash(ord(u[p]));
      inc(p);
      end
    else
      begin
      AddLit(u[p],1);
      inc(p);
      end;
    end;
  if InLit then
    Result:=Result+'''';
end;

function CodePointToString(CodePoint: longword): String;
begin
  case CodePoint of
    0..$7f:
      begin
        Result:=char(byte(CodePoint));
      end;
    $80..$7ff:
      begin
        Result:=char(byte($c0 or (CodePoint shr 6)))
               +char(byte($80 or (CodePoint and $3f)));
      end;
    $800..$ffff:
      begin
        Result:=char(byte($e0 or (CodePoint shr 12)))
               +char(byte((CodePoint shr 6) and $3f) or $80)
               +char(byte(CodePoint and $3f) or $80);
      end;
    $10000..$10ffff:
      begin
        Result:=char(byte($f0 or (CodePoint shr 18)))
               +char(byte((CodePoint shr 12) and $3f) or $80)
               +char(byte((CodePoint shr 6) and $3f) or $80)
               +char(byte(CodePoint and $3f) or $80);
      end;
  else
    Result:='';
  end;
end;

function CodePointToUnicodeString(u: longword): UnicodeString;
begin
  if u < $10000 then
    // Note: codepoints $D800 - $DFFF are reserved
    Result:=WideChar(u)
  else
    Result:=WideChar($D800+((u - $10000) shr 10))+WideChar($DC00+((u - $10000) and $3ff));
end;

function GetObjName(o: TObject): string;
var
  GenType: TPasGenericType;
begin
  if o=nil then
    Result:='nil'
  else if o is TPasElement then
    begin
    Result:=TPasElement(o).Name;
    if o is TPasGenericType then
      begin
      GenType:=TPasGenericType(o);
      if (GenType.GenericTemplateTypes<>nil)
          and (GenType.GenericTemplateTypes.Count>0) then
        Result:=Result+GetGenericParamCommas(GenType.GenericTemplateTypes.Count);
      end;
    Result:=Result+':'+o.ClassName;
    end
  else
    Result:=o.ClassName;
end;

function GetObjPath(o: TObject): string;
var
  El: TPasElement;
  GenType: TPasGenericType;
begin
  if o is TPasElement then
    begin
    El:=TPasElement(o);
    Result:=':'+El.ClassName;
    while El<>nil do
      begin
      if El<>o then
        Result:='.'+Result;
      if El is TPasGenericType then
        begin
        GenType:=TPasGenericType(El);
        if (GenType.GenericTemplateTypes<>nil)
            and (GenType.GenericTemplateTypes.Count>0) then
          Result:=GetGenericParamCommas(GenType.GenericTemplateTypes.Count)+Result;
        end;
      if El.Name<>'' then
        begin
        if IsValidIdent(El.Name) then
          Result:=El.Name+Result
        else
          Result:='"'+El.Name+'"'+Result;
        end
      else
        Result:='['+El.ClassName+']'+Result;
      El:=El.Parent;
      end;
    end
  else
    Result:=GetObjName(o);
end;

function GetGenericParamCommas(Cnt: integer): string;
begin
  if Cnt<=0 then
    Result:=''
  else
    Result:='<'+StringOfChar(',',Cnt-1)+'>';
end;

function GetElementNameAndParams(El: TPasElement; MaxLvl: integer): string;
begin
  if El=nil then
    exit('(nil)');
  Result:=El.Name;
  if El is TPasGenericType then
    Result:=Result+GetTypeParamNames(TPasGenericType(El).GenericTemplateTypes,MaxLvl-1);
end;

function GetTypeParamNames(Templates: TFPList; MaxLvl: integer): string;
var
  i: Integer;
  El: TPasElement;
begin
  if (Templates=nil) or (Templates.Count=0) then
    exit('');
  if MaxLvl<=0 then
    exit('...');
  Result:='<';
  for i:=0 to Templates.Count-1 do
    begin
    if i>0 then
      Result:=Result+',';
    El:=TPasElement(Templates[i]);
    if El.Name<>'' then
      Result:=Result+GetElementNameAndParams(El,MaxLvl-1)
    else if El is TPasArrayType then
      Result:=Result+'array...'
    else
      Result:=Result+'...';
    end;
  Result:=Result+'>';
end;

function dbgs(const Flags: TResEvalFlags): string;
var
  s: string;
  f: TResEvalFlag;
begin
  Result:='';
  for f in Flags do
    if f in Flags then
      begin
      if Result<>'' then Result:=Result+',';
      str(f,s);
      Result:=Result+s;
      end;
  Result:='['+Result+']';
end;

function dbgs(v: TResEvalValue): string;
begin
  if v=nil then
    Result:='nil'
  else
    Result:=v.AsDebugString;
end;

function LastPos(c: char; const s: string): sizeint;
var
  i: SizeInt;
begin
  for i:=length(s) downto 1 do
    if s[i]=c then exit(i);
  Result:=-1;
end;

{ TResEvalExternal }

constructor TResEvalExternal.Create;
begin
  inherited Create;
  Kind:=revkExternal;
end;

function TResEvalExternal.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
end;

function TResEvalExternal.AsString: string;
begin
  Result:=inherited AsString;
end;

{ TResEvalCurrency }

constructor TResEvalCurrency.Create;
begin
  inherited Create;
  Kind:=revkCurrency;
end;

constructor TResEvalCurrency.CreateValue(const aValue: TMaxPrecCurrency);
begin
  Create;
  Value:=aValue;
end;

function TResEvalCurrency.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalCurrency(Result).Value:=Value;
end;

function TResEvalCurrency.AsString: string;
begin
  str(Value,Result);
end;

function TResEvalCurrency.IsInt(out Int: TMaxPrecInt): boolean;
var
  i: TMaxPrecInt;
begin
  i:=AsInt;
  Result:=(i mod 10000)=0;
  Int:=i div 10000;
end;

function TResEvalCurrency.AsInt: TMaxPrecInt;
begin
  {$ifdef pas2js}
  Result:=NativeInt(Value); // pas2js stores currency as a double with factor 10.000
  {$else}
  Result:=PInt64(@Value)^; // fpc stores currency as an int64 with factor 10.000
  {$endif};
end;

{ TResEvalBool }

constructor TResEvalBool.Create;
begin
  inherited Create;
  Kind:=revkBool;
end;

constructor TResEvalBool.CreateValue(const aValue: boolean);
begin
  Create;
  B:=aValue;
end;

function TResEvalBool.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalBool(Result).B:=B;
end;

function TResEvalBool.AsString: string;
begin
  if B then
    Result:='true'
  else
    Result:='false';
end;

{ TResEvalRangeUInt }

constructor TResEvalRangeUInt.Create;
begin
  inherited Create;
  Kind:=revkRangeInt;
end;

constructor TResEvalRangeUInt.CreateValue(const aRangeStart,
  aRangeEnd: TMaxPrecUInt);
begin
  Create;
  RangeStart:=aRangeStart;
  RangeEnd:=aRangeEnd;
end;

function TResEvalRangeUInt.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalRangeUInt(Result).RangeStart:=RangeStart;
  TResEvalRangeUInt(Result).RangeEnd:=RangeEnd;
end;

function TResEvalRangeUInt.AsString: string;
begin
  Result:=IntToStr(RangeStart)+'..'+IntToStr(RangeEnd);
end;

{ TResExprEvaluator }

procedure TResExprEvaluator.LogMsg(const id: TMaxPrecInt; MsgType: TMessageType;
  MsgNumber: integer; const Fmt: String;
  Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif};
  PosEl: TPasElement);
begin
  OnLog(Self,id,MsgType,MsgNumber,Fmt,Args,PosEl);
end;

procedure TResExprEvaluator.RaiseMsg(const Id: TMaxPrecInt; MsgNumber: integer;
  const Fmt: String; Args: array of {$ifdef pas2js}jsvalue{$else}const{$endif};
  ErrorPosEl: TPasElement);
begin
  LogMsg(id,mtError,MsgNumber,Fmt,Args,ErrorPosEl);
  raise Exception.Create('['+IntToStr(id)+'] ('+IntToStr(MsgNumber)+') '+SafeFormat(Fmt,Args));
end;

procedure TResExprEvaluator.RaiseNotYetImplemented(id: TMaxPrecInt; El: TPasElement;
  Msg: string);
var
  s: String;
begin
  s:=sNotYetImplemented+' ['+IntToStr(id)+']';
  if Msg<>'' then
    s:=s+' '+Msg;
  {$IFDEF VerbosePasResolver}
  writeln('TResExprEvaluator.RaiseNotYetImplemented s="',s,'" El=',GetObjName(El));
  {$ENDIF}
  RaiseMsg(id,nNotYetImplemented,s,[GetObjName(El)],El);
end;

procedure TResExprEvaluator.RaiseInternalError(id: TMaxPrecInt; const Msg: string);
begin
  raise Exception.Create('Internal error: ['+IntToStr(id)+'] '+Msg);
end;

procedure TResExprEvaluator.RaiseConstantExprExp(id: TMaxPrecInt; ErrorEl: TPasElement
  );
begin
  RaiseMsg(id,nConstantExpressionExpected,sConstantExpressionExpected,[],ErrorEl);
end;

procedure TResExprEvaluator.RaiseRangeCheck(id: TMaxPrecInt; ErrorEl: TPasElement);
begin
  RaiseMsg(id,nRangeCheckError,sRangeCheckError,[],ErrorEl);
end;

procedure TResExprEvaluator.RaiseOverflowArithmetic(id: TMaxPrecInt;
  ErrorEl: TPasElement);
begin
  RaiseMsg(id,nOverflowInArithmeticOperation,sOverflowInArithmeticOperation,[],ErrorEl);
end;

procedure TResExprEvaluator.RaiseDivByZero(id: TMaxPrecInt; ErrorEl: TPasElement);
begin
  RaiseMsg(id,nDivByZero,sDivByZero,[],ErrorEl);
end;

function TResExprEvaluator.EvalUnaryExpr(Expr: TUnaryExpr; Flags: TResEvalFlags
  ): TResEvalValue;
var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
begin
  Result:=Eval(Expr.Operand,Flags);
  if Result=nil then exit;
  {$IFDEF VerbosePasResEval}
  writeln('TResExprEvaluator.EvalUnaryExpr ',OpcodeStrings[Expr.OpCode],' Value=',Result.AsDebugString);
  {$ENDIF}
  case Expr.OpCode of
    eopAdd: ;
    eopSubtract:
      case Result.Kind of
      revkInt:
        begin
        Int:=TResEvalInt(Result).Int;
        if Int=0 then exit;
        if Result.Element<>nil then
          Result:=Result.Clone;
        if (TResEvalInt(Result).Typed in reitAllSigned) then
          begin
          if Int=reitLow[TResEvalInt(Result).Typed] then
            begin
            // need higher precision
            if TResEvalInt(Result).Typed<>reitNone then
              // unsigned -> switch to untyped
              TResEvalInt(Result).Typed:=reitNone
            else
              begin
              // switch to float
              ReleaseEvalValue(Result);
              Result:=TResEvalFloat.CreateValue(-TMaxPrecFloat(low(TMaxPrecInt)));
              exit;
              end;
            end;
          end
        else
          begin
          // unsigned -> switch to untyped
          TResEvalInt(Result).Typed:=reitNone;
          end ;
        // negate
        TResEvalInt(Result).Int:=-Int;
        end;
      revkUInt:
        begin
        UInt:=TResEvalUInt(Result).UInt;
        if UInt=0 then exit;
        if UInt<=High(TMaxPrecInt) then
          begin
          ReleaseEvalValue(Result);
          Result:=TResEvalInt.CreateValue(-TMaxPrecInt(UInt));
          end
        else
          begin
          // switch to float
          ReleaseEvalValue(Result);
          Result:=TResEvalFloat.CreateValue(-TMaxPrecFloat(UInt));
          end;
        end;
      revkFloat:
        begin
        if TResEvalFloat(Result).FloatValue=0 then exit;
        if Result.Element<>nil then
          Result:=Result.Clone;
        TResEvalFloat(Result).FloatValue:=-TResEvalFloat(Result).FloatValue;
        end;
      revkCurrency:
        begin
        if TResEvalCurrency(Result).Value=0 then exit;
        if Result.Element<>nil then
          Result:=Result.Clone;
        TResEvalCurrency(Result).Value:=-TResEvalCurrency(Result).Value;
        end;
      revkExternal:
        exit;
      else
        begin
        if Result.Element=nil then
          Result.Free;
        RaiseNotYetImplemented(20170518230738,Expr);
        end;
      end;
    eopNot:
      case Result.Kind of
      revkBool:
        begin
        if Result.Element<>nil then
          Result:=Result.Clone;
        TResEvalBool(Result).B:=not TResEvalBool(Result).B;
        end;
      revkInt:
        begin
        if Result.Element<>nil then
          Result:=Result.Clone;
        case TResEvalInt(Result).Typed of
          reitByte: TResEvalInt(Result).Int:=not byte(TResEvalInt(Result).Int);
          reitShortInt: TResEvalInt(Result).Int:=not shortint(TResEvalInt(Result).Int);
          reitWord: TResEvalInt(Result).Int:=not word(TResEvalInt(Result).Int);
          reitSmallInt: TResEvalInt(Result).Int:=not smallint(TResEvalInt(Result).Int);
          reitUIntSingle: TResEvalInt(Result).Int:=(not TResEvalInt(Result).Int) and $3fffff;
          reitIntSingle: TResEvalInt(Result).Int:=(not TResEvalInt(Result).Int) and $7fffff;
          reitLongWord: TResEvalInt(Result).Int:=not longword(TResEvalInt(Result).Int);
          reitLongInt: TResEvalInt(Result).Int:=not longint(TResEvalInt(Result).Int);
          reitUIntDouble: TResEvalInt(Result).Int:=(not TResEvalInt(Result).Int) and $fffffffffffff;
          reitIntDouble: {$ifdef fpc}TResEvalInt(Result).Int:=(not TResEvalInt(Result).Int) and $1fffffffffffff{$endif};
          else TResEvalInt(Result).Int:=not TResEvalInt(Result).Int;
        end;
        end;
      revkUInt:
        begin
        if Result.Element<>nil then
          Result:=Result.Clone;
        TResEvalUInt(Result).UInt:=not TResEvalUInt(Result).UInt;
        end;
      revkExternal:
        exit;
      else
        begin
        if Result.Element=nil then
          Result.Free;
        RaiseNotYetImplemented(20170518232804,Expr);
        end;
      end;
    eopAddress:
      begin
      if Result.Element=nil then
        Result.Free;
      // @ operator requires a compiler (not just a resolver) -> return nil
      Result:=TResEvalValue.CreateKind(revkNil);
      end
    else
      RaiseNotYetImplemented(20170518232823,Expr,'operator='+OpcodeStrings[Expr.OpCode]);
    end;
end;

function TResExprEvaluator.EvalBinaryExpr(Expr: TBinaryExpr;
  Flags: TResEvalFlags): TResEvalValue;
var
  LeftValue, RightValue: TResEvalValue;
begin
  Result:=nil;
  if (Expr.Kind=pekBinary) and (Expr.OpCode=eopSubIdent) then
    begin
    Result:=Eval(Expr.right,Flags);
    exit;
    end;
  LeftValue:=nil;
  RightValue:=nil;
  try
    LeftValue:=Eval(Expr.left,Flags);
    if LeftValue=nil then exit;
    RightValue:=Eval(Expr.right,Flags);
    if RightValue=nil then exit;

    if LeftValue.Kind=revkExternal then
      begin
      if [refConst,refConstExt]*Flags=[refConst] then
        RaiseConstantExprExp(20181024134508,Expr.left);
      Result:=LeftValue;
      LeftValue:=nil;
      exit;
      end;
    if RightValue.Kind=revkExternal then
      begin
      if [refConst,refConstExt]*Flags=[refConst] then
        RaiseConstantExprExp(20181024134545,Expr.right);
      Result:=RightValue;
      RightValue:=nil;
      exit;
      end;

    case Expr.Kind of
    pekRange:
      // leftvalue..rightvalue
      Result:=EvalBinaryRangeExpr(Expr,LeftValue,RightValue);
    pekBinary:
      case Expr.OpCode of
      eopAdd:
        Result:=EvalBinaryAddExpr(Expr,LeftValue,RightValue);
      eopSubtract:
        Result:=EvalBinarySubExpr(Expr,LeftValue,RightValue);
      eopMultiply:
        Result:=EvalBinaryMulExpr(Expr,LeftValue,RightValue);
      eopDivide:
        Result:=EvalBinaryDivideExpr(Expr,LeftValue,RightValue);
      eopDiv:
        Result:=EvalBinaryDivExpr(Expr,LeftValue,RightValue);
      eopMod:
        Result:=EvalBinaryModExpr(Expr,LeftValue,RightValue);
      eopPower:
        Result:=EvalBinaryPowerExpr(Expr,LeftValue,RightValue);
      eopShl,eopShr:
        Result:=EvalBinaryShiftExpr(Expr,LeftValue,RightValue);
      eopAnd,eopOr,eopXor:
        Result:=EvalBinaryBoolOpExpr(Expr,LeftValue,RightValue);
      eopEqual,eopNotEqual:
        Result:=EvalBinaryNEqualExpr(Expr,LeftValue,RightValue);
      eopLessThan,eopGreaterThan, eopLessthanEqual,eopGreaterThanEqual:
        Result:=EvalBinaryLessGreaterExpr(Expr,LeftValue,RightValue);
      eopIn:
        Result:=EvalBinaryInExpr(Expr,LeftValue,RightValue);
      eopSymmetricaldifference:
        Result:=EvalBinarySymmetricaldifferenceExpr(Expr,LeftValue,RightValue);
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryExpr Kind=',Expr.Kind,' Opcode=',OpcodeStrings[Expr.OpCode],' Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        RaiseNotYetImplemented(20170530100823,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryExpr Kind=',Expr.Kind,' Opcode=',OpcodeStrings[Expr.OpCode]);
      {$ENDIF}
      RaiseNotYetImplemented(20170530100827,Expr);
    end;
    {$IFDEF VerbosePasResEval}
    {AllowWriteln}
    if Result<>nil then
      writeln('TResExprEvaluator.EvalBinaryExpr Left=',LeftValue.AsDebugString,' Opcode=',OpcodeStrings[Expr.OpCode],' Right=',RightValue.AsDebugString,' Result=',Result.AsDebugString)
    else
      writeln('TResExprEvaluator.EvalBinaryExpr Left=',LeftValue.AsDebugString,' Opcode=',OpcodeStrings[Expr.OpCode],' Right=',RightValue.AsDebugString,' Result not set');
    {AllowWriteln-}
    {$ENDIF}
  finally
    ReleaseEvalValue(LeftValue);
    ReleaseEvalValue(RightValue);
  end;
end;

function TResExprEvaluator.EvalBinaryRangeExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
// LeftValue..RightValue
var
  LeftInt, RightInt: TMaxPrecInt;
begin
  case LeftValue.Kind of
  revkBool:
    if RightValue.Kind<>revkBool then
      RaiseRangeCheck(20170714133017,Expr.Right)
    else
      begin
      LeftInt:=ord(TResEvalBool(LeftValue).B);
      RightInt:=ord(TResEvalBool(RightValue).B);
      if LeftInt>RightInt then
        RaiseMsg(20170714133540,nHighRangeLimitLTLowRangeLimit,
           sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
      Result:=TResEvalRangeInt.CreateValue(revskBool,nil,LeftInt,RightInt);
      exit;
      end;
  revkInt:
    if RightValue.Kind=revkInt then
      begin
      LeftInt:=TResEvalInt(LeftValue).Int;
      RightInt:=TResEvalInt(RightValue).Int;
      if LeftInt>RightInt then
        RaiseMsg(20170518222939,nHighRangeLimitLTLowRangeLimit,
          sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
      Result:=TResEvalRangeInt.CreateValue(revskInt,nil,LeftInt,RightInt);
      exit;
      end
    else if RightValue.Kind=revkUInt then
      begin
      // Note: when FPC compares int64 with qword it converts the qword to an int64
      if TResEvalUInt(RightValue).UInt<=HighIntAsUInt then
        begin
        if TResEvalInt(LeftValue).Int>TResEvalUInt(RightValue).UInt then
          RaiseMsg(20170519000235,nHighRangeLimitLTLowRangeLimit,
            sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
        Result:=TResEvalRangeInt.CreateValue(revskInt,nil,
           TResEvalInt(LeftValue).Int,TMaxPrecInt(TResEvalUInt(RightValue).UInt));
        exit;
        end
      else if TResEvalInt(LeftValue).Int<0 then
        RaiseRangeCheck(20170522151629,Expr.Right)
      else if TMaxPrecUInt(TResEvalInt(LeftValue).Int)>TResEvalUInt(RightValue).UInt then
        RaiseMsg(20170522151708,nHighRangeLimitLTLowRangeLimit,
          sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
      Result:=TResEvalRangeUInt.CreateValue(TMaxPrecUInt(TResEvalInt(LeftValue).Int),
         TResEvalUInt(RightValue).UInt);
      exit;
      end
    else
      RaiseRangeCheck(20170518222812,Expr.Right);
  revkUInt:
    if RightValue.Kind=revkInt then
      begin
      // Note: when FPC compares int64 with qword it converts the qword to an int64
      if TResEvalUInt(LeftValue).UInt>HighIntAsUInt then
        begin
        if TResEvalInt(RightValue).Int<0 then
          RaiseRangeCheck(20170522152608,Expr.Right)
        else if TResEvalUInt(LeftValue).UInt>TMaxPrecUInt(TResEvalInt(RightValue).Int) then
          RaiseMsg(20170522152648,nHighRangeLimitLTLowRangeLimit,
            sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
        Result:=TResEvalRangeUInt.CreateValue(TResEvalUInt(LeftValue).UInt,
          TMaxPrecUInt(TResEvalInt(RightValue).Int));
        exit;
        end
      else if TResEvalUInt(LeftValue).UInt>TResEvalInt(RightValue).Int then
        RaiseMsg(20170522152804,nHighRangeLimitLTLowRangeLimit,
          sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
      Result:=TResEvalRangeInt.CreateValue(revskInt,nil,
        TMaxPrecInt(TResEvalUInt(LeftValue).UInt),TResEvalInt(RightValue).Int);
      exit;
      end
    else if RightValue.Kind=revkUInt then
      begin
      if TResEvalUInt(LeftValue).UInt>TResEvalUInt(RightValue).UInt then
        RaiseMsg(20170519000240,nHighRangeLimitLTLowRangeLimit,
          sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
      Result:=TResEvalRangeUInt.CreateValue(TResEvalUInt(LeftValue).UInt,
        TResEvalUInt(RightValue).UInt);
      exit;
      end
    else
      RaiseRangeCheck(20170522123106,Expr.Right);
  revkEnum:
    if (RightValue.Kind<>revkEnum) then
      RaiseRangeCheck(20170522153003,Expr.Right)
    else if (TResEvalEnum(LeftValue).ElType<>TResEvalEnum(RightValue).ElType) then
      begin
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryRangeExpr LeftValue=',dbgs(LeftValue),',',GetObjName(TResEvalEnum(LeftValue).ElType),' RightValue=',dbgs(RightValue),',',GetObjName(TResEvalEnum(RightValue).ElType));
      {$ENDIF}
      RaiseRangeCheck(20170522123241,Expr.Right) // mismatch enumtype
      end
    else if TResEvalEnum(LeftValue).Index>TResEvalEnum(RightValue).Index then
      RaiseMsg(20170522123320,nHighRangeLimitLTLowRangeLimit,
        sHighRangeLimitLTLowRangeLimit,[],Expr.Right)
    else
      begin
      Result:=TResEvalRangeInt.CreateValue(revskEnum,
        TResEvalEnum(LeftValue).ElType as TPasEnumType,
        TResEvalEnum(LeftValue).Index,TResEvalEnum(RightValue).Index);
      exit;
      end;
  {$ifdef FPC_HAS_CPSTRING}
  revkString,
  {$endif}
  revkUnicodeString:
    begin
    LeftInt:=StringToOrd(LeftValue,Expr.left);
    if RightValue.Kind in revkAllStrings then
      begin
      RightInt:=StringToOrd(RightValue,Expr.right);
      if LeftInt>RightInt then
        RaiseMsg(20170523151508,nHighRangeLimitLTLowRangeLimit,
          sHighRangeLimitLTLowRangeLimit,[],Expr.Right);
      Result:=TResEvalRangeInt.CreateValue(revskChar,nil,LeftInt,RightInt);
      exit;
      end
    else
      RaiseRangeCheck(20170522123106,Expr.Right);
    end
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryRangeExpr Left=',GetObjName(Expr.Left),' LeftValue.Kind=',LeftValue.Kind);
    RaiseNotYetImplemented(20170518221103,Expr.Left);
    {$ENDIF}
  end;
end;

function TResExprEvaluator.EvalBinaryAddExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;

  procedure IntAddUInt(const i: TMaxPrecInt; const u: TMaxPrecUInt);
  var
    Int: TMaxPrecInt;
    UInt: TMaxPrecUInt;
  begin
    if (i>=0) then
      begin
      UInt:=TMaxPrecUInt(i)+u;
      Result:=CreateResEvalInt(UInt);
      end
    else if u<=HighIntAsUInt then
      begin
      Int:=i + TMaxPrecInt(u);
      Result:=TResEvalInt.CreateValue(Int);
      end
    else
      RaiseRangeCheck(20170601140523,Expr);
  end;

var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
  Flo: TMaxPrecFloat;
  aCurrency: TMaxPrecCurrency;
  LeftSet, RightSet: TResEvalSet;
  i: Integer;
begin
  Result:=nil;
  try
    {$Q+} // enable overflow and range checks
    {$R+}
    case LeftValue.Kind of
    revkInt:
      begin
      Int:=TResEvalInt(LeftValue).Int;
      case RightValue.Kind of
      revkInt: // int + int
        if (Int>0) and (TResEvalInt(RightValue).Int>0) then
          begin
          UInt:=TMaxPrecUInt(Int)+TMaxPrecUInt(TResEvalInt(RightValue).Int);
          Result:=CreateResEvalInt(UInt);
          end
        else
          begin
          Int:=Int + TResEvalInt(RightValue).Int;
          Result:=TResEvalInt.CreateValue(Int);
          end;
      revkUInt: // int + uint
        IntAddUInt(Int,TResEvalUInt(RightValue).UInt);
      revkFloat: // int + float
        Result:=TResEvalFloat.CreateValue(Int + TResEvalFloat(RightValue).FloatValue);
      revkCurrency: // int + currency
        Result:=TResEvalCurrency.CreateValue(Int + TResEvalCurrency(RightValue).Value);
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryAddExpr int+? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        RaiseNotYetImplemented(20170525115537,Expr);
      end;
      end;
    revkUInt:
      begin
      UInt:=TResEvalUInt(LeftValue).UInt;
      case RightValue.Kind of
      revkInt: // uint + int
        IntAddUInt(UInt,TResEvalInt(RightValue).Int);
      revkUInt: // uint + uint
        begin
        UInt:=UInt+TResEvalUInt(RightValue).UInt;
        Result:=TResEvalUInt.CreateValue(UInt);
        end;
      revkFloat: // uint + float
        Result:=TResEvalFloat.CreateValue(UInt + TResEvalFloat(RightValue).FloatValue);
      revkCurrency: // uint + currency
        Result:=TResEvalCurrency.CreateValue(UInt + TResEvalCurrency(RightValue).Value);
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryAddExpr uint+? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        RaiseNotYetImplemented(20170601141031,Expr);
      end;
      end;
    revkFloat:
      begin
      Flo:=TResEvalFloat(LeftValue).FloatValue;
      case RightValue.Kind of
      revkInt: // float + int
        Result:=TResEvalFloat.CreateValue(Flo + TResEvalInt(RightValue).Int);
      revkUInt: // float + uint
        Result:=TResEvalFloat.CreateValue(Flo + TResEvalUInt(RightValue).UInt);
      revkFloat: // float + float
        Result:=TResEvalFloat.CreateValue(Flo + TResEvalFloat(RightValue).FloatValue);
      revkCurrency: // float + Currency
        Result:=TResEvalCurrency.CreateValue(Flo + TResEvalCurrency(RightValue).Value);
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryAddExpr float+? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        RaiseNotYetImplemented(20170711145637,Expr);
      end;
      end;
    revkCurrency:
      begin
      aCurrency:=TResEvalCurrency(LeftValue).Value;
      case RightValue.Kind of
      revkInt: // currency + int
        Result:=TResEvalFloat.CreateValue(aCurrency + TResEvalInt(RightValue).Int);
      revkUInt: // currency + uint
        Result:=TResEvalFloat.CreateValue(aCurrency + TResEvalUInt(RightValue).UInt);
      revkFloat: // currency + float
        Result:=TResEvalFloat.CreateValue(aCurrency + TResEvalFloat(RightValue).FloatValue);
      revkCurrency: // currency + currency
        Result:=TResEvalCurrency.CreateValue(aCurrency + TResEvalCurrency(RightValue).Value);
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryAddExpr currency+? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        RaiseNotYetImplemented(20180421163819,Expr);
      end;
      end;
    {$ifdef FPC_HAS_CPSTRING}
    revkString,
    {$endif}
    revkUnicodeString:
      Result:=EvalStringAddExpr(Expr,Expr.left,Expr.right,LeftValue,RightValue);
    revkSetOfInt:
      case RightValue.Kind of
      revkSetOfInt:
        begin
        // union
        LeftSet:=TResEvalSet(LeftValue);
        RightSet:=TResEvalSet(RightValue);
        if LeftSet.ElKind=revskNone then
          Result:=RightSet.Clone
        else if RightSet.ElKind=revskNone then
          Result:=LeftSet.Clone
        else
          begin
          Result:=RightSet.Clone;
          // add elements of left
          for i:=0 to length(LeftSet.Ranges)-1 do
            begin
            Int:=LeftSet.Ranges[i].RangeStart;
            while Int<=LeftSet.Ranges[i].RangeEnd do
              begin
              TResEvalSet(Result).Add(Int,Int);
              inc(Int);
              end;
            end;
          end;
        end;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryMulExpr add set+? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        RaiseNotYetImplemented(20170714114055,Expr);
      end
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryAddExpr ?+ Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170525115548,Expr);
    end;
  except
    on EOverflow do
      RaiseOverflowArithmetic(20170601140130,Expr);
    on ERangeError do
      RaiseRangeCheck(20170601140132,Expr);
  end;
end;

function TResExprEvaluator.EvalBinarySubExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
  Flo: TMaxPrecFloat;
  aCurrency: TMaxPrecCurrency;
  LeftSet, RightSet: TResEvalSet;
  i: Integer;
begin
  Result:=nil;
  case LeftValue.Kind of
  revkInt:
    begin
    Int:=TResEvalInt(LeftValue).Int;
    case RightValue.Kind of
    revkInt:
      // int - int
      try
        {$Q+}
        Int:=Int - TResEvalInt(RightValue).Int;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        on E: EOverflow do
          if (Int>0) and (TResEvalInt(RightValue).Int<0) then
            begin
            UInt:=TMaxPrecUInt(Int)+TMaxPrecUInt(-TResEvalInt(RightValue).Int);
            Result:=CreateResEvalInt(UInt);
            end
          else
            RaiseOverflowArithmetic(20170525230247,Expr);
      end;
    revkUInt:
      // int - uint
      try
        {$Q+}
        Int:=Int - TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151201,Expr);
      end;
    revkFloat:
      // int - float
      try
        {$Q+}
        Flo:=TMaxPrecFloat(Int) - TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151313,Expr);
      end;
    revkCurrency:
      // int - currency
      try
        {$Q+}
        aCurrency:=TMaxPrecCurrency(Int) - TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164011,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinarySubExpr sub int-? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170525230028,Expr);
    end;
    end;
  revkUInt:
    begin
    UInt:=TResEvalUInt(LeftValue).UInt;
    case RightValue.Kind of
    revkInt:
      // uint - int
      try
        {$Q+}
        UInt:=UInt - TResEvalInt(RightValue).Int;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalUInt.CreateValue(UInt);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151405,Expr);
      end;
    revkUInt:
      // uint - uint
      try
        {$Q+}
        UInt:=UInt - TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalUInt.CreateValue(UInt);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151419,Expr);
      end;
    revkFloat:
      // uint - float
      try
        {$Q+}
        Flo:=TMaxPrecFloat(UInt) - TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151428,Expr);
      end;
    revkCurrency:
      // uint - currency
      try
        {$Q+}
        aCurrency:=TMaxPrecCurrency(UInt) - TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164005,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinarySubExpr sub uint-? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711151435,Expr);
    end;
    end;
  revkFloat:
    begin
    Flo:=TResEvalFloat(LeftValue).FloatValue;
    case RightValue.Kind of
    revkInt:
      // float - int
      try
        {$Q+}
        Flo:=Flo - TResEvalInt(RightValue).Int;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151519,Expr);
      end;
    revkUInt:
      // float - uint
      try
        {$Q+}
        Flo:=Flo - TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151538,Expr);
      end;
    revkFloat:
      // float - float
      try
        {$Q+}
        Flo:=Flo - TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711151552,Expr);
      end;
    revkCurrency:
      // float - currency
      try
        {$Q+}
        aCurrency:=Flo - TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164054,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinarySubExpr sub float-? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711151600,Expr);
    end;
    end;
  revkCurrency:
    begin
    aCurrency:=TResEvalCurrency(LeftValue).Value;
    case RightValue.Kind of
    revkInt:
      // currency - int
      try
        {$Q+}
        aCurrency:=aCurrency - TResEvalInt(RightValue).Int;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164200,Expr);
      end;
    revkUInt:
      // currency - uint
      try
        {$Q+}
        aCurrency:=aCurrency - TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164218,Expr);
      end;
    revkFloat:
      // currency - float
      try
        {$Q+}
        aCurrency:=aCurrency - TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164250,Expr);
      end;
    revkCurrency:
      // currency - currency
      try
        {$Q+}
        aCurrency:=aCurrency - TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164258,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinarySubExpr sub currency-? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20180421164312,Expr);
    end;
    end;
  revkSetOfInt:
    case RightValue.Kind of
    revkSetOfInt:
      begin
      // difference
      LeftSet:=TResEvalSet(LeftValue);
      RightSet:=TResEvalSet(RightValue);
      if LeftSet.ElKind=revskNone then
        Result:=TResEvalSet.CreateEmptySameKind(RightSet)
      else
        begin
        Result:=TResEvalSet.CreateEmptySameKind(LeftSet);
        // add elements, which exists only in LeftSet
        for i:=0 to length(LeftSet.Ranges)-1 do
          begin
          Int:=LeftSet.Ranges[i].RangeStart;
          while Int<=LeftSet.Ranges[i].RangeEnd do
            begin
            if RightSet.IndexOfRange(Int)<0 then
              TResEvalSet(Result).Add(Int,Int);
            inc(Int);
            end;
          end;
        end;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinarySubExpr sub set-? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170714114101,Expr);
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinarySubExpr sub ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170525225946,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryMulExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
  Flo: TMaxPrecFloat;
  aCurrency: TMaxPrecCurrency;
  LeftSet, RightSet: TResEvalSet;
  i: Integer;
begin
  Result:=nil;
  case LeftValue.Kind of
  revkInt:
    begin
    Int:=TResEvalInt(LeftValue).Int;
    case RightValue.Kind of
    revkInt:
      // int * int
      try
        {$Q+}
        Int:=Int * TResEvalInt(RightValue).Int;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        on E: EOverflow do
          if (Int>0) and (TResEvalInt(RightValue).Int>0) then
            try
              // try uint*uint
              {$Q+}
              UInt:=TMaxPrecUInt(Int) * TMaxPrecUInt(TResEvalInt(RightValue).Int);
              {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
              Result:=CreateResEvalInt(UInt);
            except
              on E: EOverflow do
                RaiseOverflowArithmetic(20170530101616,Expr);
            end
          else
            RaiseOverflowArithmetic(20170525230247,Expr);
      end;
    revkUInt:
      // int * uint
      try
        {$Q+}
        Int:=Int * TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        RaiseOverflowArithmetic(20170711164445,Expr);
      end;
    revkFloat:
      // int * float
      try
        {$Q+}
        Flo:=Int * TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170711164541,Expr);
      end;
    revkCurrency:
      // int * currency
      try
        {$Q+}
        aCurrency:=Int * TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421164426,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryMulExpr mul int*? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170525230028,Expr);
    end;
    end;
  revkUInt:
    begin
    UInt:=TResEvalUInt(LeftValue).UInt;
    case RightValue.Kind of
    revkInt:
      // uint * int
      if TResEvalInt(RightValue).Int>=0 then
        try
          {$Q+}
          UInt:=UInt * TResEvalInt(RightValue).Int;
          {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
          Result:=TResEvalUInt.CreateValue(UInt);
        except
          on E: EOverflow do
            RaiseOverflowArithmetic(20170711164714,Expr);
        end
      else
        try
          {$Q+}
          Int:=UInt * TResEvalInt(RightValue).Int;
          {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
          Result:=TResEvalInt.CreateValue(Int);
        except
          on E: EOverflow do
            RaiseOverflowArithmetic(20170711164736,Expr);
        end;
    revkUInt:
      // uint * uint
      try
        {$Q+}
        UInt:=UInt * TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalUInt.CreateValue(UInt);
      except
        RaiseOverflowArithmetic(20170711164751,Expr);
      end;
    revkFloat:
      // uint * float
      try
        {$Q+}
        Flo:=UInt * TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170711164800,Expr);
      end;
    revkCurrency:
      // uint * currency
      try
        {$Q+}
        aCurrency:=UInt * TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421164500,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryMulExpr mul uint*? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711164810,Expr);
    end;
    end;
  revkFloat:
    begin
    Flo:=TResEvalFloat(LeftValue).FloatValue;
    case RightValue.Kind of
    revkInt:
      // float * int
      try
        {$Q+}
        Flo:=Flo * TResEvalInt(RightValue).Int;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20170711164920,Expr);
      end;
    revkUInt:
      // float * uint
      try
        {$Q+}
        Flo:=Flo * TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170711164940,Expr);
      end;
    revkFloat:
      // float * float
      try
        {$Q+}
        Flo:=Flo * TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170711164955,Expr);
      end;
    revkCurrency:
      // float * currency
      try
        {$Q+}
        Flo:=Flo * TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20180421164542,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryMulExpr mul float*? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711165004,Expr);
    end;
    end;
  revkCurrency:
    begin
    aCurrency:=TResEvalCurrency(LeftValue).Value;
    case RightValue.Kind of
    revkInt:
      // currency * int
      try
        {$Q+}
        aCurrency:=aCurrency * TResEvalInt(RightValue).Int;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        on E: EOverflow do
          RaiseOverflowArithmetic(20180421164636,Expr);
      end;
    revkUInt:
      // currency * uint
      try
        {$Q+}
        aCurrency:=aCurrency * TResEvalUInt(RightValue).UInt;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421164654,Expr);
      end;
    revkFloat:
      // currency * float
      try
        {$Q+}
        Flo:=aCurrency * TResEvalFloat(RightValue).FloatValue;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20180421164718,Expr);
      end;
    revkCurrency:
      // currency * currency
      try
        {$Q+}
        aCurrency:=aCurrency * TResEvalCurrency(RightValue).Value;
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421164806,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryMulExpr mul currency*? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20180421164817,Expr);
    end;
    end;
  revkSetOfInt:
    case RightValue.Kind of
    revkSetOfInt:
      begin
      // intersect
      LeftSet:=TResEvalSet(LeftValue);
      RightSet:=TResEvalSet(RightValue);
      if LeftSet.ElKind=revskNone then
        Result:=TResEvalSet.CreateEmptySameKind(RightSet)
      else
        begin
        Result:=TResEvalSet.CreateEmptySameKind(LeftSet);
        // add elements, which exists in both
        for i:=0 to length(LeftSet.Ranges)-1 do
          begin
          Int:=LeftSet.Ranges[i].RangeStart;
          while Int<=LeftSet.Ranges[i].RangeEnd do
            begin
            if RightSet.IndexOfRange(Int)>=0 then
              TResEvalSet(Result).Add(Int,Int);
            inc(Int);
            end;
          end;
        end;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryMulExpr mul set*? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170714110420,Expr);
    end
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryMulExpr mul ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170525225946,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryDivideExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
  Flo: TMaxPrecFloat;
  aCurrency: TMaxPrecCurrency;
begin
  Result:=nil;
  case LeftValue.Kind of
  revkInt:
    begin
    Int:=TResEvalInt(LeftValue).Int;
    case RightValue.Kind of
    revkInt:
      // int / int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170711143925,Expr)
      else
        Result:=TResEvalFloat.CreateValue(Int / TResEvalInt(RightValue).Int);
    revkUInt:
      // int / uint
      if TResEvalUInt(RightValue).UInt=0 then
        RaiseDivByZero(20170711144013,Expr)
      else
        Result:=TResEvalFloat.CreateValue(Int / TResEvalUInt(RightValue).UInt);
    revkFloat:
      begin
      // int / float
      try
        Flo:=Int / TResEvalFloat(RightValue).FloatValue;
      except
        RaiseMsg(20170711144525,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalFloat.CreateValue(Flo);
      end;
    revkCurrency:
      begin
      // int / currency
      try
        aCurrency:=Int / TResEvalCurrency(RightValue).Value;
      except
        RaiseMsg(20180421164915,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalCurrency.CreateValue(aCurrency);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryDivideExpr int / ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711144057,Expr);
    end;
    end;
  revkUInt:
    begin
    UInt:=TResEvalUInt(LeftValue).UInt;
    case RightValue.Kind of
    revkInt:
      // uint / int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170711144103,Expr)
      else
        Result:=TResEvalFloat.CreateValue(UInt / TResEvalInt(RightValue).Int);
    revkUInt:
      // uint / uint
      if TResEvalUInt(RightValue).UInt=0 then
        RaiseDivByZero(20170711144203,Expr)
      else
        Result:=TResEvalFloat.CreateValue(UInt / TResEvalUInt(RightValue).UInt);
    revkFloat:
      begin
      // uint / float
      try
        Flo:=UInt / TResEvalFloat(RightValue).FloatValue;
      except
        RaiseMsg(20170711144912,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalFloat.CreateValue(Flo);
      end;
    revkCurrency:
      begin
      // uint / currency
      try
        aCurrency:=UInt / TResEvalCurrency(RightValue).Value;
      except
        RaiseMsg(20180421164959,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalCurrency.CreateValue(aCurrency);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryDivideExpr uint / ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711144239,Expr);
    end;
    end;
  revkFloat:
    begin
    Flo:=TResEvalFloat(LeftValue).FloatValue;
    case RightValue.Kind of
    revkInt:
      // float / int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170711144954,Expr)
      else
        Result:=TResEvalFloat.CreateValue(Flo / TResEvalInt(RightValue).Int);
    revkUInt:
      // float / uint
      if TResEvalUInt(RightValue).UInt=0 then
        RaiseDivByZero(20170711145023,Expr)
      else
        Result:=TResEvalFloat.CreateValue(Flo / TResEvalUInt(RightValue).UInt);
    revkFloat:
      begin
      // float / float
      try
        Flo:=Flo / TResEvalFloat(RightValue).FloatValue;
      except
        RaiseMsg(20170711145040,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalFloat.CreateValue(Flo);
      end;
    revkCurrency:
      begin
      // float / currency
      try
        aCurrency:=Flo / TResEvalCurrency(RightValue).Value;
      except
        RaiseMsg(20180421165058,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalCurrency.CreateValue(aCurrency);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryDivideExpr float / ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711145050,Expr);
    end;
    end;
  revkCurrency:
    begin
    aCurrency:=TResEvalCurrency(LeftValue).Value;
    case RightValue.Kind of
    revkInt:
      // currency / int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20180421165154,Expr)
      else
        Result:=TResEvalCurrency.CreateValue(aCurrency / TResEvalInt(RightValue).Int);
    revkUInt:
      // currency / uint
      if TResEvalUInt(RightValue).UInt=0 then
        RaiseDivByZero(20180421165205,Expr)
      else
        Result:=TResEvalCurrency.CreateValue(aCurrency / TResEvalUInt(RightValue).UInt);
    revkFloat:
      begin
      // currency / float
      try
        aCurrency:=aCurrency / TResEvalFloat(RightValue).FloatValue;
      except
        RaiseMsg(20180421165237,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalCurrency.CreateValue(aCurrency);
      end;
    revkCurrency:
      begin
      // currency / currency
      try
        aCurrency:=aCurrency / TResEvalCurrency(RightValue).Value;
      except
        RaiseMsg(20180421165252,nDivByZero,sDivByZero,[],Expr);
      end;
      Result:=TResEvalCurrency.CreateValue(aCurrency);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryDivideExpr currency / ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20180421165301,Expr);
    end;
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryDivExpr div ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170530102352,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryDivExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
begin
  Result:=nil;
  case LeftValue.Kind of
  revkInt:
    case RightValue.Kind of
    revkInt:
      // int div int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170530102619,Expr)
      else
        begin
        Int:=TResEvalInt(LeftValue).Int div TResEvalInt(RightValue).Int;
        Result:=TResEvalInt.CreateValue(Int);
        end;
    revkUInt:
      // int div uint
      if TResEvalUInt(RightValue).UInt=0 then
        RaiseDivByZero(20170530102745,Expr)
      else
        begin
        if TResEvalUInt(RightValue).UInt>HighIntAsUInt then
          Int:=0
        else
          Int:=TResEvalInt(LeftValue).Int div TResEvalUInt(RightValue).UInt;
        Result:=TResEvalInt.CreateValue(Int);
        end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryDivExpr int div ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530102403,Expr);
    end;
  revkUInt:
    case RightValue.Kind of
    revkInt:
      // uint div int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170530103026,Expr)
      else if TResEvalUInt(LeftValue).UInt<=HighIntAsUInt then
        begin
        Int:=TMaxPrecInt(TResEvalUInt(LeftValue).UInt) div TResEvalInt(RightValue).Int;
        Result:=TResEvalInt.CreateValue(Int);
        end
      else if TResEvalInt(RightValue).Int>0 then
        begin
        UInt:=TResEvalUInt(LeftValue).UInt div TMaxPrecUInt(TResEvalInt(RightValue).Int);
        Result:=CreateResEvalInt(UInt);
        end
      else
        RaiseOverflowArithmetic(20170530104315,Expr);
    revkUInt:
      // uint div uint
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170530103026,Expr)
      else
        begin
        UInt:=TResEvalUInt(LeftValue).UInt div TResEvalUInt(RightValue).UInt;
        Result:=CreateResEvalInt(UInt);
        end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryDivExpr uint div ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530102403,Expr);
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryDivExpr div ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170530102352,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryModExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
begin
  Result:=nil;
  case LeftValue.Kind of
  revkInt:
    case RightValue.Kind of
    revkInt:
      // int mod int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170530104638,Expr)
      else
        begin
        Int:=TResEvalInt(LeftValue).Int mod TResEvalInt(RightValue).Int;
        Result:=TResEvalInt.CreateValue(Int);
        end;
    revkUInt:
      // int mod uint
      if TResEvalUInt(RightValue).UInt=0 then
        RaiseDivByZero(20170530104758,Expr)
      else
        begin
        if TResEvalInt(LeftValue).Int<0 then
          UInt:=TMaxPrecUInt(-TResEvalInt(LeftValue).Int) mod TResEvalUInt(RightValue).UInt
        else
          UInt:=TMaxPrecUInt(TResEvalInt(LeftValue).Int) mod TResEvalUInt(RightValue).UInt;
        Result:=CreateResEvalInt(UInt);
        end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryModExpr int mod ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530110057,Expr);
    end;
  revkUInt:
    case RightValue.Kind of
    revkInt:
      // uint mod int
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170530110110,Expr)
      else if TResEvalUInt(LeftValue).UInt<=HighIntAsUInt then
        begin
        Int:=TMaxPrecInt(TResEvalUInt(LeftValue).UInt) mod TResEvalInt(RightValue).Int;
        Result:=TResEvalInt.CreateValue(Int);
        end
      else if TResEvalInt(RightValue).Int>0 then
        begin
        UInt:=TResEvalUInt(LeftValue).UInt mod TMaxPrecUInt(TResEvalInt(RightValue).Int);
        Result:=CreateResEvalInt(UInt);
        end
      else
        RaiseOverflowArithmetic(20170530110602,Expr);
    revkUInt:
      // uint div uint
      if TResEvalInt(RightValue).Int=0 then
        RaiseDivByZero(20170530110609,Expr)
      else
        begin
        UInt:=TResEvalUInt(LeftValue).UInt mod TResEvalUInt(RightValue).UInt;
        Result:=CreateResEvalInt(UInt);
        end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryModExpr uint mod ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530110633,Expr);
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryModExpr mod ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170530110644,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryShiftExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
  ShiftLeft: Boolean;
begin
  Result:=nil;
  ShiftLeft:=Expr.OpCode=eopShl;
  case LeftValue.Kind of
  revkInt:
    case RightValue.Kind of
    revkInt:
      // int shl int
      begin
      if (TResEvalInt(RightValue).Int<0) or (TResEvalInt(RightValue).Int>63) then
        EmitRangeCheckConst(20170530203840,IntToStr(TResEvalInt(RightValue).Int),0,63,Expr);
      if ShiftLeft then
        Int:=TResEvalInt(LeftValue).Int shl byte(TResEvalInt(RightValue).Int)
      else
        Int:=TResEvalInt(LeftValue).Int shr byte(TResEvalInt(RightValue).Int);
      Result:=TResEvalInt.CreateValue(Int);
      end;
    revkUInt:
      // int shl uint
      begin
      if (TResEvalUInt(RightValue).UInt>63) then
        EmitRangeCheckConst(20170530203840,IntToStr(TResEvalUInt(RightValue).UInt),0,63,Expr);
      if ShiftLeft then
        Int:=TResEvalInt(LeftValue).Int shl byte(TResEvalUInt(RightValue).UInt)
      else
        Int:=TResEvalInt(LeftValue).Int shr byte(TResEvalUInt(RightValue).UInt);
      Result:=TResEvalInt.CreateValue(Int);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryModExpr int shl/shr ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530205332,Expr);
    end;
  revkUInt:
    case RightValue.Kind of
    revkInt:
      // uint shl int
      begin
      if (TResEvalInt(RightValue).Int<0) or (TResEvalInt(RightValue).Int>63) then
        EmitRangeCheckConst(20170530205414,IntToStr(TResEvalInt(RightValue).Int),0,63,Expr);
      if ShiftLeft then
        UInt:=TResEvalUInt(LeftValue).UInt shl byte(TResEvalInt(RightValue).Int)
      else
        UInt:=TResEvalUInt(LeftValue).UInt shr byte(TResEvalInt(RightValue).Int);
      Result:=CreateResEvalInt(UInt);
      end;
    revkUInt:
      // uint shl uint
      begin
      if (TResEvalUInt(RightValue).UInt>63) then
        EmitRangeCheckConst(20170530205601,IntToStr(TResEvalUInt(RightValue).UInt),0,63,Expr);
      if ShiftLeft then
        UInt:=TResEvalUInt(LeftValue).UInt shl byte(TResEvalUInt(RightValue).UInt)
      else
        UInt:=TResEvalUInt(LeftValue).UInt shr byte(TResEvalUInt(RightValue).UInt);
      Result:=CreateResEvalInt(UInt);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryShiftExpr uint shl/shr ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530205640,Expr);
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryShiftExpr shl/shr ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170530205646,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryBoolOpExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
// AND, OR, XOR
begin
  Result:=nil;
  case LeftValue.Kind of
  revkBool:
    case RightValue.Kind of
    revkBool:
      begin
      // logical and/or/xor
      Result:=TResEvalBool.Create;
      case Expr.OpCode of
      eopAnd: TResEvalBool(Result).B:=TResEvalBool(LeftValue).B and TResEvalBool(RightValue).B;
      eopOr: TResEvalBool(Result).B:=TResEvalBool(LeftValue).B or TResEvalBool(RightValue).B;
      eopXor: TResEvalBool(Result).B:=TResEvalBool(LeftValue).B xor TResEvalBool(RightValue).B;
      end;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryBoolOpExpr bool ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170531011502,Expr);
    end;
  revkInt:
    case RightValue.Kind of
    revkInt:
      begin
      // bitwise and/or/xor
      Result:=TResEvalInt.Create;
      case Expr.OpCode of
      eopAnd: TResEvalInt(Result).Int:=TResEvalInt(LeftValue).Int and TResEvalInt(RightValue).Int;
      eopOr: TResEvalInt(Result).Int:=TResEvalInt(LeftValue).Int or TResEvalInt(RightValue).Int;
      eopXor: TResEvalInt(Result).Int:=TResEvalInt(LeftValue).Int xor TResEvalInt(RightValue).Int;
      end;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryBoolOpExpr int ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530211140,Expr);
    end;
  revkUInt:
    case RightValue.Kind of
    revkUInt:
      begin
      // bitwise and/or/xor
      Result:=TResEvalUInt.Create;
      case Expr.OpCode of
      eopAnd: TResEvalUInt(Result).UInt:=TResEvalUInt(LeftValue).UInt and TResEvalUInt(RightValue).UInt;
      eopOr: TResEvalUInt(Result).UInt:=TResEvalUInt(LeftValue).UInt or TResEvalUInt(RightValue).UInt;
      eopXor: TResEvalUInt(Result).UInt:=TResEvalUInt(LeftValue).UInt xor TResEvalUInt(RightValue).UInt;
      end;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryBoolOpExpr int ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530211140,Expr);
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryBoolOpExpr ',Expr.OpCode,' ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170530205938,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryNEqualExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  UInt: TMaxPrecUInt;
  LeftSet, RightSet: TResEvalSet;
  i: Integer;
begin
  Result:=TResEvalBool.Create;
  try
    {$Q+}
    {$R+}
    case LeftValue.Kind of
    revkBool:
      case RightValue.Kind of
      revkBool:
        TResEvalBool(Result).B:=TResEvalBool(LeftValue).B=TResEvalBool(RightValue).B;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr bool ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170531011937,Expr);
      end;
    revkInt:
      case RightValue.Kind of
      revkInt:
        TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int=TResEvalInt(RightValue).Int;
      revkUInt:
        TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int=TResEvalUInt(RightValue).UInt;
      revkFloat:
        TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int=TResEvalFloat(RightValue).FloatValue;
      revkCurrency:
        TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int=TResEvalCurrency(RightValue).Value;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr int ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170531012412,Expr);
      end;
    revkUInt:
      begin
      UInt:=TResEvalUInt(LeftValue).UInt;
      case RightValue.Kind of
      revkInt:
        TResEvalBool(Result).B:=(UInt<=HighIntAsUInt)
                               and (TMaxPrecInt(UInt)=TResEvalInt(RightValue).Int);
      revkUInt:
        TResEvalBool(Result).B:=UInt=TResEvalUInt(RightValue).UInt;
      revkFloat:
        TResEvalBool(Result).B:=UInt=TResEvalFloat(RightValue).FloatValue;
      revkCurrency:
        TResEvalBool(Result).B:=UInt=TResEvalCurrency(RightValue).Value;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr uint ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170601122803,Expr);
      end;
      end;
    revkFloat:
      case RightValue.Kind of
      revkInt:
        TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue=TResEvalInt(RightValue).Int;
      revkUInt:
        TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue=TResEvalUInt(RightValue).UInt;
      revkFloat:
        TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue=TResEvalFloat(RightValue).FloatValue;
      revkCurrency:
        TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue=TResEvalCurrency(RightValue).Value;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr float ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170601122806,Expr);
      end;
    revkCurrency:
      case RightValue.Kind of
      revkInt:
        TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value=TResEvalInt(RightValue).Int;
      revkUInt:
        TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value=TResEvalUInt(RightValue).UInt;
      revkFloat:
        TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value=TResEvalFloat(RightValue).FloatValue;
      revkCurrency:
        TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value=TResEvalCurrency(RightValue).Value;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr currency ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20180421165438,Expr);
      end;
    {$ifdef FPC_HAS_CPSTRING}
    revkString:
      case RightValue.Kind of
      revkString:
        if GetCodePage(TResEvalString(LeftValue).S)=GetCodePage(TResEvalString(RightValue).S) then
          TResEvalBool(Result).B:=TResEvalString(LeftValue).S=TResEvalString(RightValue).S
        else
          TResEvalBool(Result).B:=GetUnicodeStr(TResEvalString(LeftValue).S,Expr.left)
                                 =GetUnicodeStr(TResEvalString(RightValue).S,Expr.right);
      revkUnicodeString:
        TResEvalBool(Result).B:=GetUnicodeStr(TResEvalString(LeftValue).S,Expr.left)
                               =TResEvalUTF16(RightValue).S;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr string ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170711175409,Expr);
      end;
    {$endif}
    revkUnicodeString:
      case RightValue.Kind of
      {$ifdef FPC_HAS_CPSTRING}
      revkString:
        TResEvalBool(Result).B:=TResEvalUTF16(LeftValue).S
                               =GetUnicodeStr(TResEvalString(RightValue).S,Expr.right);
      {$endif}
      revkUnicodeString:
        TResEvalBool(Result).B:=TResEvalUTF16(LeftValue).S
                               =TResEvalUTF16(RightValue).S;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr string ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170711175409,Expr);
      end;
    revkSetOfInt:
      case RightValue.Kind of
      revkSetOfInt:
        begin
        LeftSet:=TResEvalSet(LeftValue);
        RightSet:=TResEvalSet(RightValue);
        if LeftSet.ElKind=revskNone then
          TResEvalBool(Result).B:=length(RightSet.Ranges)=0
        else if RightSet.ElKind=revskNone then
          TResEvalBool(Result).B:=length(LeftSet.Ranges)=0
        else if length(LeftSet.Ranges)<>length(RightSet.Ranges) then
          TResEvalBool(Result).B:=false
        else
          begin
          TResEvalBool(Result).B:=true;
          for i:=0 to length(LeftSet.Ranges)-1 do
            if (LeftSet.Ranges[i].RangeStart<>RightSet.Ranges[i].RangeStart)
                or (LeftSet.Ranges[i].RangeEnd<>RightSet.Ranges[i].RangeEnd) then
              begin
              TResEvalBool(Result).B:=false;
              break;
              end;
          end;
        end;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryNEqualExpr ',Expr.OpCode,' set=? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        RaiseNotYetImplemented(20170714120756,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryNEqualExpr ',Expr.OpCode,' ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      Result.Free;
      RaiseNotYetImplemented(20170531011931,Expr);
    end;
    {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
    {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
  except
    on EOverflow do
      RaiseOverflowArithmetic(20170601132729,Expr);
    on ERangeError do
      RaiseRangeCheck(20170601132740,Expr);
  end;
  if Expr.OpCode=eopNotEqual then
    TResEvalBool(Result).B:=not TResEvalBool(Result).B;
end;

function TResExprEvaluator.EvalBinaryLessGreaterExpr(Expr: TBinaryExpr;
  LeftValue, RightValue: TResEvalValue): TResEvalValue;

  procedure CmpUnicode(const LeftUnicode, RightUnicode: UnicodeString);
  begin
    case Expr.OpCode of
    eopLessThan:
      TResEvalBool(Result).B:=LeftUnicode < RightUnicode;
    eopGreaterThan:
      TResEvalBool(Result).B:=LeftUnicode > RightUnicode;
    eopLessthanEqual:
      TResEvalBool(Result).B:=LeftUnicode <= RightUnicode;
    eopGreaterThanEqual:
      TResEvalBool(Result).B:=LeftUnicode >= RightUnicode;
    end;
  end;

var
  LeftSet, RightSet: TResEvalSet;
  i: Integer;
  Int: TMaxPrecInt;
begin
  Result:=TResEvalBool.Create;
  try
    {$Q+}
    {$R+}
    case LeftValue.Kind of
    revkInt:
      case RightValue.Kind of
      revkInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int < TResEvalInt(RightValue).Int;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int > TResEvalInt(RightValue).Int;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int <= TResEvalInt(RightValue).Int;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int >= TResEvalInt(RightValue).Int;
        end;
      revkUInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int < TResEvalUInt(RightValue).UInt;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int > TResEvalUInt(RightValue).UInt;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int <= TResEvalUInt(RightValue).UInt;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int >= TResEvalUInt(RightValue).UInt;
        end;
      revkFloat:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int < TResEvalFloat(RightValue).FloatValue;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int > TResEvalFloat(RightValue).FloatValue;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int <= TResEvalFloat(RightValue).FloatValue;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int >= TResEvalFloat(RightValue).FloatValue;
        end;
      revkCurrency:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int < TResEvalCurrency(RightValue).Value;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int > TResEvalCurrency(RightValue).Value;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int <= TResEvalCurrency(RightValue).Value;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalInt(LeftValue).Int >= TResEvalCurrency(RightValue).Value;
        end;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr int ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170601122512,Expr);
      end;
    revkUInt:
      case RightValue.Kind of
      revkInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt < TResEvalInt(RightValue).Int;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt > TResEvalInt(RightValue).Int;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt <= TResEvalInt(RightValue).Int;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt >= TResEvalInt(RightValue).Int;
        end;
      revkUInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt < TResEvalUInt(RightValue).UInt;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt > TResEvalUInt(RightValue).UInt;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt <= TResEvalUInt(RightValue).UInt;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt >= TResEvalUInt(RightValue).UInt;
        end;
      revkFloat:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt < TResEvalFloat(RightValue).FloatValue;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt > TResEvalFloat(RightValue).FloatValue;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt <= TResEvalFloat(RightValue).FloatValue;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt >= TResEvalFloat(RightValue).FloatValue;
        end;
      revkCurrency:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt < TResEvalCurrency(RightValue).Value;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt > TResEvalCurrency(RightValue).Value;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt <= TResEvalCurrency(RightValue).Value;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalUInt(LeftValue).UInt >= TResEvalCurrency(RightValue).Value;
        end;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr uint ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170601133222,Expr);
      end;
    revkFloat:
      case RightValue.Kind of
      revkInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue < TResEvalInt(RightValue).Int;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue > TResEvalInt(RightValue).Int;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue <= TResEvalInt(RightValue).Int;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue >= TResEvalInt(RightValue).Int;
        end;
      revkUInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue < TResEvalUInt(RightValue).UInt;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue > TResEvalUInt(RightValue).UInt;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue <= TResEvalUInt(RightValue).UInt;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue >= TResEvalUInt(RightValue).UInt;
        end;
      revkFloat:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue < TResEvalFloat(RightValue).FloatValue;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue > TResEvalFloat(RightValue).FloatValue;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue <= TResEvalFloat(RightValue).FloatValue;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue >= TResEvalFloat(RightValue).FloatValue;
        end;
      revkCurrency:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue < TResEvalCurrency(RightValue).Value;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue > TResEvalCurrency(RightValue).Value;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue <= TResEvalCurrency(RightValue).Value;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalFloat(LeftValue).FloatValue >= TResEvalCurrency(RightValue).Value;
        end;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr float ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170601133421,Expr);
      end;
    revkCurrency:
      case RightValue.Kind of
      revkInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value < TResEvalInt(RightValue).Int;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value > TResEvalInt(RightValue).Int;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value <= TResEvalInt(RightValue).Int;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value >= TResEvalInt(RightValue).Int;
        end;
      revkUInt:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value < TResEvalUInt(RightValue).UInt;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value > TResEvalUInt(RightValue).UInt;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value <= TResEvalUInt(RightValue).UInt;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value >= TResEvalUInt(RightValue).UInt;
        end;
      revkFloat:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value < TResEvalFloat(RightValue).FloatValue;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value > TResEvalFloat(RightValue).FloatValue;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value <= TResEvalFloat(RightValue).FloatValue;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value >= TResEvalFloat(RightValue).FloatValue;
        end;
      revkCurrency:
        case Expr.OpCode of
        eopLessThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value < TResEvalCurrency(RightValue).Value;
        eopGreaterThan:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value > TResEvalCurrency(RightValue).Value;
        eopLessthanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value <= TResEvalCurrency(RightValue).Value;
        eopGreaterThanEqual:
          TResEvalBool(Result).B:=TResEvalCurrency(LeftValue).Value >= TResEvalCurrency(RightValue).Value;
        end;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr currency ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20180421165752,Expr);
      end;
    {$ifdef FPC_HAS_CPSTRING}
    revkString:
      case RightValue.Kind of
      revkString:
        if GetCodePage(TResEvalString(LeftValue).S)=GetCodePage(TResEvalString(RightValue).S) then
          case Expr.OpCode of
          eopLessThan:
            TResEvalBool(Result).B:=TResEvalString(LeftValue).S < TResEvalString(RightValue).S;
          eopGreaterThan:
            TResEvalBool(Result).B:=TResEvalString(LeftValue).S > TResEvalString(RightValue).S;
          eopLessthanEqual:
            TResEvalBool(Result).B:=TResEvalString(LeftValue).S <= TResEvalString(RightValue).S;
          eopGreaterThanEqual:
            TResEvalBool(Result).B:=TResEvalString(LeftValue).S >= TResEvalString(RightValue).S;
          end
        else
          CmpUnicode(GetUnicodeStr(TResEvalString(LeftValue).S,Expr.left),
                     GetUnicodeStr(TResEvalString(RightValue).S,Expr.right));
      revkUnicodeString:
        CmpUnicode(GetUnicodeStr(TResEvalString(LeftValue).S,Expr.left),
                   TResEvalUTF16(RightValue).S);
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr string ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170711175629,Expr);
      end;
    {$endif}
    revkUnicodeString:
      case RightValue.Kind of
      {$ifdef FPC_HAS_CPSTRING}
      revkString:
        CmpUnicode(TResEvalUTF16(LeftValue).S,
                   GetUnicodeStr(TResEvalString(RightValue).S,Expr.right));
      {$endif}
      revkUnicodeString:
        CmpUnicode(TResEvalUTF16(LeftValue).S,TResEvalUTF16(RightValue).S);
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr unicodestring ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170711210730,Expr);
      end;
    revkSetOfInt:
      case RightValue.Kind of
      revkSetOfInt:
        begin
        LeftSet:=TResEvalSet(LeftValue);
        RightSet:=TResEvalSet(RightValue);
        case Expr.OpCode of
        eopGreaterThanEqual:
          begin
          // >=  ->  true if all elements of RightSet are in LeftSet
          TResEvalBool(Result).B:=true;
          for i:=0 to length(RightSet.Ranges)-1 do
            begin
            Int:=RightSet.Ranges[i].RangeStart;
            while Int<=RightSet.Ranges[i].RangeEnd do
              begin
              if LeftSet.IndexOfRange(Int)<0 then
                begin
                TResEvalBool(Result).B:=false;
                break;
                end;
              inc(Int);
              end;
            end;
          end;
        eopLessthanEqual:
          begin
          // <=  ->  true if all elements of LeftSet are in RightSet
          TResEvalBool(Result).B:=true;
          for i:=0 to length(LeftSet.Ranges)-1 do
            begin
            Int:=LeftSet.Ranges[i].RangeStart;
            while Int<=LeftSet.Ranges[i].RangeEnd do
              begin
              if RightSet.IndexOfRange(Int)<0 then
                begin
                TResEvalBool(Result).B:=false;
                break;
                end;
              inc(Int);
              end;
            end;
          end
        else
          {$IFDEF VerbosePasResolver}
          writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr set ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
          {$ENDIF}
          Result.Free;
          RaiseNotYetImplemented(20170714122121,Expr);
        end;
        end;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr set ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
        {$ENDIF}
        Result.Free;
        RaiseNotYetImplemented(20170714121925,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryLowerGreaterExpr ? ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      Result.Free;
      RaiseNotYetImplemented(20170601122529,Expr);
    end;
  except
    on EOverflow do
      RaiseOverflowArithmetic(20170601132956,Expr);
    on ERangeError do
      RaiseRangeCheck(20170601132958,Expr);
  end;
end;

function TResExprEvaluator.EvalBinaryInExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  RightSet: TResEvalSet;
  Int: TMaxPrecInt;
begin
  Result:=nil;
  case RightValue.Kind of
  revkSetOfInt:
    begin
    RightSet:=TResEvalSet(RightValue);
    case LeftValue.Kind of
    revkBool:
      Int:=ord(TResEvalBool(LeftValue).B);
    revkInt:
      Int:=TResEvalInt(LeftValue).Int;
    revkUInt:
      // Note: when FPC compares int64 with qword it converts the qword to an int64
      if TResEvalUInt(LeftValue).UInt>HighIntAsUInt then
        RaiseMsg(20170714123700,nRangeCheckError,sRangeCheckError,[],Expr)
      else
        Int:=TResEvalUInt(LeftValue).UInt;
    {$ifdef FPC_HAS_CPSTRING}
    revkString,
    {$endif}
    revkUnicodeString:
      Int:=StringToOrd(LeftValue,Expr);
    revkEnum:
      Int:=TResEvalEnum(LeftValue).Index;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryInExpr ? in Set Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170714123412,Expr);
    end;
    Result:=TResEvalBool.CreateValue(RightSet.IndexOfRange(Int)>=0);
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryInExpr ? ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170714123409,Expr);
  end;
end;

function TResExprEvaluator.EvalBinarySymmetricaldifferenceExpr(
  Expr: TBinaryExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
var
  LeftSet, RightSet: TResEvalSet;
  i: Integer;
  Int: TMaxPrecInt;
begin
  case LeftValue.Kind of
  revkSetOfInt:
    case RightValue.Kind of
    revkSetOfInt:
      begin
      // sym diff
      LeftSet:=TResEvalSet(LeftValue);
      RightSet:=TResEvalSet(RightValue);
      // elements, which exists in either, but not both
      if LeftSet.ElKind=revskNone then
        Result:=RightSet.Clone
      else
        begin
        Result:=TResEvalSet.CreateEmptySameKind(LeftSet);
        for i:=0 to length(LeftSet.Ranges)-1 do
          begin
          Int:=LeftSet.Ranges[i].RangeStart;
          while Int<=LeftSet.Ranges[i].RangeEnd do
            begin
            if RightSet.IndexOfRange(Int)<0 then
              TResEvalSet(Result).Add(Int,Int);
            inc(Int);
            end;
          end;
        for i:=0 to length(RightSet.Ranges)-1 do
          begin
          Int:=RightSet.Ranges[i].RangeStart;
          while Int<=RightSet.Ranges[i].RangeEnd do
            begin
            if LeftSet.IndexOfRange(Int)<0 then
              TResEvalSet(Result).Add(Int,Int);
            inc(Int);
            end;
          end;
        end;
      end
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinarySymmetricaldifferenceExpr Set><? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170714114144,Expr);
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinarySymmetricaldifferenceExpr ? ',Expr.OpCode,' ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170714114119,Expr);
  end;
end;

function TResExprEvaluator.EvalParamsExpr(Expr: TParamsExpr;
  Flags: TResEvalFlags): TResEvalValue;
begin
  Result:=OnEvalParams(Self,Expr,Flags);
  if Result<>nil then exit;
  case Expr.Kind of
  pekArrayParams: Result:=EvalArrayParamsExpr(Expr,Flags);
  pekSet: Result:=EvalSetParamsExpr(Expr,Flags);
  end;
  if Result=nil then
    begin
    if [refConst,refConstExt]*Flags<>[] then
      RaiseConstantExprExp(20170713124038,Expr);
    exit;
    end;
end;

function TResExprEvaluator.EvalArrayParamsExpr(Expr: TParamsExpr;
  Flags: TResEvalFlags): TResEvalValue;
var
  ArrayValue, IndexValue: TResEvalValue;
  Int: TMaxPrecInt;
  Param0: TPasExpr;
  MaxIndex: Integer;
begin
  Result:=nil;
  ArrayValue:=Eval(Expr.Value,Flags);
  if ArrayValue=nil then
    begin
    if [refConst,refConstExt]*Flags<>[] then
      RaiseConstantExprExp(20170711181321,Expr.Value);
    exit;
    end;
  IndexValue:=nil;
  try
    case ArrayValue.Kind of
    {$ifdef FPC_HAS_CPSTRING}
    revkString,
    {$endif}
    revkUnicodeString:
      begin
      // string[index]
      Param0:=Expr.Params[0];
      IndexValue:=Eval(Param0,Flags);
      if IndexValue=nil then
        begin
        if [refConst,refConstExt]*Flags<>[] then
          RaiseConstantExprExp(20170711181603,Param0);
        exit;
        end;
      case IndexValue.Kind of
        revkInt: Int:=TResEvalInt(IndexValue).Int;
        revkUInt:
          // Note: when FPC compares int64 with qword it converts the qword to an int64
          if TResEvalUInt(IndexValue).UInt>HighIntAsUInt then
            RaiseRangeCheck(20170711182006,Param0)
          else
            Int:=TResEvalUInt(IndexValue).UInt;
      else
        {$IFDEF VerbosePasResolver}
        writeln('TResExprEvaluator.EvalParamsExpr string[',IndexValue.AsDebugString,']');
        {$ENDIF}
        RaiseNotYetImplemented(20170711182100,Expr);
      end;
      {$ifdef FPC_HAS_CPSTRING}
      if ArrayValue.Kind=revkString then
        MaxIndex:=length(TResEvalString(ArrayValue).S)
      else
      {$endif}
        MaxIndex:=length(TResEvalUTF16(ArrayValue).S);
      if (Int<1) or (Int>MaxIndex) then
        EmitRangeCheckConst(20170711183058,IntToStr(Int),'1',IntToStr(MaxIndex),Param0,mtError);
      {$ifdef FPC_HAS_CPSTRING}
      if ArrayValue.Kind=revkString then
        Result:=TResEvalString.CreateValue(TResEvalString(ArrayValue).S[Int])
      else
      {$endif}
        Result:=TResEvalUTF16.CreateValue(TResEvalUTF16(ArrayValue).S[Int]);
      exit;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalParamsExpr Array=',ArrayValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170711181507,Expr);
    end;

    if [refConst,refConstExt]*Flags<>[] then
      RaiseConstantExprExp(20170522173150,Expr);
  finally
    ReleaseEvalValue(ArrayValue);
    ReleaseEvalValue(IndexValue);
  end;
end;

function TResExprEvaluator.EvalSetParamsExpr(Expr: TParamsExpr;
  Flags: TResEvalFlags): TResEvalSet;
begin
  {$IFDEF VerbosePasResEval}
  writeln('TResExprEvaluator.EvalSetParamsExpr length(Expr.Params)=',length(Expr.Params));
  {$ENDIF}
  Result:=EvalSetExpr(Expr,Expr.Params,Flags);
end;

function TResExprEvaluator.EvalSetExpr(Expr: TPasExpr;
  ExprArray: TPasExprArray; Flags: TResEvalFlags): TResEvalSet;
var
  i: Integer;
  RangeStart, RangeEnd: TMaxPrecInt;
  Value: TResEvalValue;
  ok, OnlyConstElements: Boolean;
  El: TPasExpr;
begin
  {$IFDEF VerbosePasResEval}
  writeln('TResExprEvaluator.EvalSetExpr Expr=',GetObjName(Expr),' length(ExprArray)=',length(ExprArray));
  {$ENDIF}
  Result:=TResEvalSet.Create;
  if Expr=nil then ;
  Value:=nil;
  OnlyConstElements:=true;
  ok:=false;
  try
    for i:=0 to length(ExprArray)-1 do
      begin
      El:=ExprArray[i];
      {$IFDEF VerbosePasResEval}
      writeln('TResExprEvaluator.EvalSetExpr ',i,' of ',length(ExprArray),' El=',GetObjName(El));
      {$ENDIF}
      Value:=Eval(El,Flags);
      if Value=nil then
        begin
        // element is not a const -> the set is not a const
        OnlyConstElements:=false;
        continue;
        end;
      {$IFDEF VerbosePasResEval}
      //writeln('TResExprEvaluator.EvalSetExpr ',i,' of ',length(ExprArray),' Value=',Value.AsDebugString);
      {$ENDIF}
      case Value.Kind of
      revkBool:
        begin
        if Result.ElKind=revskNone then
          Result.ElKind:=revskBool
        else if Result.ElKind<>revskBool then
          RaiseNotYetImplemented(20170714132843,El);
        RangeStart:=ord(TResEvalBool(Value).B);
        RangeEnd:=RangeStart;
        end;
      revkInt:
        begin
        if Result.ElKind=revskNone then
          Result.ElKind:=revskInt
        else if Result.ElKind<>revskInt then
          RaiseNotYetImplemented(20170713201208,El);
        RangeStart:=TResEvalInt(Value).Int;
        RangeEnd:=RangeStart;
        end;
      revkUInt:
        begin
        if Result.ElKind=revskNone then
          Result.ElKind:=revskInt
        else if Result.ElKind<>revskInt then
          RaiseNotYetImplemented(20170713201230,El)
        // Note: when FPC compares int64 with qword it converts the qword to an int64
        else if TResEvalUInt(Value).UInt>HighIntAsUInt then
          EmitRangeCheckConst(20170713201306,Value.AsString,
            '0',IntToStr(High(TMaxPrecInt)),El,mtError);
        RangeStart:=TResEvalUInt(Value).UInt;
        RangeEnd:=RangeStart;
        end;
      {$ifdef FPC_HAS_CPSTRING}
      revkString:
        begin
        if Result.ElKind=revskNone then
          Result.ElKind:=revskChar
        else if Result.ElKind<>revskChar then
          RaiseNotYetImplemented(20170713201456,El);
        RangeStart:=StringToOrd(Value,nil);
        if RangeStart>$ffff then
          begin
          // set of string (not of char)
          ReleaseEvalValue(TResEvalValue(Result));
          exit;
          end;
        RangeEnd:=RangeStart;
        end;
      {$endif}
      revkUnicodeString:
        begin
        if Result.ElKind=revskNone then
          Result.ElKind:=revskChar
        else if Result.ElKind<>revskChar then
          RaiseNotYetImplemented(20170713201516,El);
        if length(TResEvalUTF16(Value).S)<>1 then
          begin
          // set of string (not of char)
          ReleaseEvalValue(TResEvalValue(Result));
          exit;
          end;
        RangeStart:=ord(TResEvalUTF16(Value).S[1]);
        RangeEnd:=RangeStart;
        end;
      revkEnum:
        begin
        if Result.ElKind=revskNone then
          begin
          Result.ElKind:=revskEnum;
          Result.ElType:=Value.IdentEl.Parent as TPasEnumType;
          end
        else if Result.ElKind<>revskEnum then
          RaiseNotYetImplemented(20170713143559,El)
        else if Result.ElType<>TResEvalEnum(Value).ElType then
          RaiseNotYetImplemented(20170713201021,El);
        RangeStart:=TResEvalEnum(Value).Index;
        RangeEnd:=RangeStart;
        end;
      revkRangeInt:
        begin
        if Result.ElKind=revskNone then
          begin
          Result.ElKind:=TResEvalRangeInt(Value).ElKind;
          if Result.ElKind=revskEnum then
            Result.ElType:=TResEvalRangeInt(Value).ElType;
          end
        else if Result.ElKind<>TResEvalRangeInt(Value).ElKind then
          RaiseNotYetImplemented(20170714101910,El);
        RangeStart:=TResEvalRangeInt(Value).RangeStart;
        RangeEnd:=TResEvalRangeInt(Value).RangeEnd;
        end;
      revkRangeUInt:
        begin
        if Result.ElKind=revskNone then
          Result.ElKind:=revskInt
        else if Result.ElKind<>revskInt then
          RaiseNotYetImplemented(20170713202934,El)
        // Note: when FPC compares int64 with qword it converts the qword to an int64
        else if TResEvalRangeUInt(Value).RangeEnd>HighIntAsUInt then
          EmitRangeCheckConst(20170713203034,Value.AsString,
            '0',IntToStr(High(TMaxPrecInt)),El,mtError);
        RangeStart:=TResEvalRangeUInt(Value).RangeStart;
        RangeEnd:=TResEvalRangeUInt(Value).RangeEnd;
        end
      else
        {$IF defined(VerbosePasResEval) or defined(VerbosePasResolver)}
        writeln('TResExprEvaluator.EvalSetExpr Result.ElKind=',Result.ElKind,' Value.Kind=',Value.Kind);
        {$ENDIF}
        RaiseNotYetImplemented(20170713143422,El);
      end;

      if Result.Intersects(RangeStart,RangeEnd)>=0 then
        begin
        {$IF defined(VerbosePasResEval) or defined(VerbosePasResolver)}
        writeln('TResExprEvaluator.EvalSetExpr Value=',Value.AsDebugString,' Range=',RangeStart,'..',RangeEnd,' Result=',Result.AsDebugString);
        {$ENDIF}
        RaiseMsg(20170714141326,nRangeCheckInSetConstructor,
          sRangeCheckInSetConstructor,[],El);
        end;
      Result.Add(RangeStart,RangeEnd);
      ReleaseEvalValue(Value);
      end;
    ok:=OnlyConstElements;
  finally
    ReleaseEvalValue(Value);
    if not ok then
      ReleaseEvalValue(TResEvalValue(Result));
  end;
end;

function TResExprEvaluator.EvalArrayValuesExpr(Expr: TArrayValues;
  Flags: TResEvalFlags): TResEvalSet;
begin
  {$IFDEF VerbosePasResEval}
  writeln('TResExprEvaluator.EvalArrayValuesExpr length(Expr.Values)=',length(Expr.Values));
  {$ENDIF}
  Result:=EvalSetExpr(Expr,Expr.Values,Flags);
end;

function TResExprEvaluator.EvalBinaryPowerExpr(Expr: TBinaryExpr; LeftValue,
  RightValue: TResEvalValue): TResEvalValue;
var
  Int: TMaxPrecInt;
  Flo: TMaxPrecFloat;
  aCurrency: TMaxPrecCurrency;
begin
  Result:=nil;
  case LeftValue.Kind of
  revkInt:
    case RightValue.Kind of
    revkInt:
      // int^^int
      try
        {$Q+}{$R+}
        Int:=trunc(Math.power(TResEvalInt(LeftValue).Int,TResEvalInt(RightValue).Int));
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        RaiseOverflowArithmetic(20170530210533,Expr);
      end;
    revkUInt:
      // int^^uint
      try
        {$Q+}{$R+}
        Int:=trunc(Math.power(TResEvalInt(LeftValue).Int,TResEvalUInt(RightValue).UInt));
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        RaiseOverflowArithmetic(20170530211028,Expr);
      end;
    revkFloat:
      // int^^float
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalInt(LeftValue).Int,TResEvalFloat(RightValue).FloatValue);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170816154223,Expr);
      end;
    revkCurrency:
      // int^^currency
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalInt(LeftValue).Int,TResEvalCurrency(RightValue).Value);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20180421165906,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryPowerExpr int ^^ ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530205640,Expr);
    end;
  revkUInt:
    case RightValue.Kind of
    revkInt:
      // uint^^int
      try
        {$Q+}{$R+}
        Int:=trunc(Math.power(TResEvalUInt(LeftValue).UInt,TResEvalInt(RightValue).Int));
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        RaiseOverflowArithmetic(20170530211102,Expr);
      end;
    revkUInt:
      // uint^^uint
      try
        {$Q+}{$R+}
        Int:=trunc(Math.power(TResEvalUInt(LeftValue).UInt,TResEvalUInt(RightValue).UInt));
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalInt.CreateValue(Int);
      except
        RaiseOverflowArithmetic(20170530211121,Expr);
      end;
    revkFloat:
      // uint^^float
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalUInt(LeftValue).UInt,TResEvalFloat(RightValue).FloatValue);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170816154241,Expr);
      end;
    revkCurrency:
      // uint^^currency
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalUInt(LeftValue).UInt,TResEvalCurrency(RightValue).Value);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20180421165948,Expr);
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryPowerExpr uint ^^ ? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170530211140,Expr);
    end;
  revkFloat:
    case RightValue.Kind of
    revkInt:
      // float ^^ int
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalFloat(LeftValue).FloatValue,TResEvalInt(RightValue).Int);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170816153950,Expr);
      end;
    revkUInt:
      // float ^^ uint
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalFloat(LeftValue).FloatValue,TResEvalUInt(RightValue).UInt);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170816154012,Expr);
      end;
    revkFloat:
      // float ^^ float
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalFloat(LeftValue).FloatValue,TResEvalFloat(RightValue).FloatValue);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20170816154012,Expr);
      end;
    revkCurrency:
      // float ^^ currency
      try
        {$Q+}{$R+}
        Flo:=Math.power(TResEvalFloat(LeftValue).FloatValue,TResEvalCurrency(RightValue).Value);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalFloat.CreateValue(Flo);
      except
        RaiseOverflowArithmetic(20180421170016,Expr);
      end;
    end;
  revkCurrency:
    case RightValue.Kind of
    revkInt:
      // currency ^^ int
      try
        {$Q+}{$R+}
        aCurrency:=Math.power(TResEvalCurrency(LeftValue).Value,TResEvalInt(RightValue).Int);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421170235,Expr);
      end;
    revkUInt:
      // currency ^^ uint
      try
        {$Q+}{$R+}
        aCurrency:=Math.power(TResEvalCurrency(LeftValue).Value,TResEvalUInt(RightValue).UInt);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421170240,Expr);
      end;
    revkFloat:
      // currency ^^ float
      try
        {$Q+}{$R+}
        aCurrency:=Math.power(TResEvalCurrency(LeftValue).Value,TResEvalFloat(RightValue).FloatValue);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421170254,Expr);
      end;
    revkCurrency:
      // currency ^^ currency
      try
        {$Q+}{$R+}
        aCurrency:=Math.power(TResEvalCurrency(LeftValue).Value,TResEvalCurrency(RightValue).Value);
        {$IFNDEF OverflowCheckOn}{$Q-}{$ENDIF}
        {$IFNDEF RangeCheckOn}{$R-}{$ENDIF}
        Result:=TResEvalCurrency.CreateValue(aCurrency);
      except
        RaiseOverflowArithmetic(20180421170311,Expr);
      end;
    end;
  else
    {$IFDEF VerbosePasResolver}
    writeln('TResExprEvaluator.EvalBinaryPowerExpr ^^ ?- Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170816153813,Expr);
  end;
end;

function TResExprEvaluator.StringToOrd(Value: TResEvalValue;
  PosEl: TPasElement): longword;
const
  Invalid = $12345678; // bigger than $ffff and smaller than $8000000
var
  {$ifdef FPC_HAS_CPSTRING}
  S: RawByteString;
  {$endif}
  U: UnicodeString;
begin
  case Value.Kind of
  {$ifdef FPC_HAS_CPSTRING}
  revkString:
    begin
    // ord(ansichar)
    S:=TResEvalString(Value).S;
    if length(S)=1 then
      Result:=ord(S[1])
    else if (length(S)=0) or (length(S)>4) then
      begin
      if PosEl<>nil then
        RaiseMsg(20170522221143,nXExpectedButYFound,sXExpectedButYFound,
          ['char','string'],PosEl)
      else
        exit(Invalid);
      end
    else
      begin
      U:=GetUnicodeStr(S,nil);
      if length(U)<>1 then
        begin
        if PosEl<>nil then
          RaiseMsg(20190124180407,nXExpectedButYFound,sXExpectedButYFound,
            ['char','string'],PosEl)
        else
          exit(Invalid);
        end;
      Result:=ord(U[1]);
      end;
    end;
  {$endif}
  revkUnicodeString:
    begin
    // ord(widechar)
    U:=TResEvalUTF16(Value).S;
    if length(U)<>1 then
      begin
      if PosEl<>nil then
        RaiseMsg(20170522221358,nXExpectedButYFound,sXExpectedButYFound,
          ['char','string'],PosEl)
      else
        exit(Invalid);
      end
    else
      Result:=ord(U[1]);
    end;
  else
    RaiseNotYetImplemented(20170522220959,PosEl);
  end;
end;

function TResExprEvaluator.EvalPrimitiveExprString(Expr: TPrimitiveExpr
  ): TResEvalValue;
{ Extracts the value from a Pascal string literal

  S is a Pascal string literal e.g. 'Line'#10
    ''  empty string
    '''' => "'"
    #decimal
    #$hex
    ^l  l is a letter a-z
}

  procedure RangeError(id: TMaxPrecInt);
  begin
    Result.Free;
    RaiseRangeCheck(id,Expr);
  end;

  procedure Add(h: String);
  begin
    {$ifdef FPC_HAS_CPSTRING}
    if Result.Kind=revkString then
      TResEvalString(Result).S:=TResEvalString(Result).S+h
    else
      TResEvalUTF16(Result).S:=TResEvalUTF16(Result).S+GetUnicodeStr(h,Expr);
    {$else}
    TResEvalUTF16(Result).S:=TResEvalUTF16(Result).S+h;
    {$endif}
  end;

  procedure AddHash(u: longword; ForceUTF16: boolean);
  {$ifdef FPC_HAS_CPSTRING}
  var
    h: RawByteString;
  begin
    if ((u>255) or (ForceUTF16)) and (Result.Kind=revkString) then
      begin
      // switch to unicodestring
      h:=TResEvalString(Result).S;
      Result.Free;
      Result:=nil; // in case of exception in GetUnicodeStr
      Result:=TResEvalUTF16.CreateValue(GetUnicodeStr(h,Expr));
      end;
    if Result.Kind=revkString then
      TResEvalString(Result).S:=TResEvalString(Result).S+Chr(u)
    else
      TResEvalUTF16(Result).S:=TResEvalUTF16(Result).S+WideChar(u);
  end;
  {$else}
  begin
    TResEvalUTF16(Result).S:=TResEvalUTF16(Result).S+WideChar(u);
    if ForceUTF16 then ;
  end;
  {$endif}

var
  p, StartP, l: integer;
  c: Char;
  u: longword;
  S: String;
begin
  Result:=nil;
  S:=Expr.Value;
  {$IFDEF VerbosePasResEval}
  //writeln('TResExprEvaluator.EvalPrimitiveExprString (',S,')');
  {$ENDIF}
  l:=length(S);
  if l=0 then
    RaiseInternalError(20170523113809);
  {$ifdef FPC_HAS_CPSTRING}
  Result:=TResEvalString.Create;
  {$else}
  Result:=TResEvalUTF16.Create;
  {$endif}
  p:=1;
  while p<=l do
    case S[p] of
    {$ifdef UsePChar}
    #0: break;
    {$endif}
    '''':
      begin
      inc(p);
      StartP:=p;
      repeat
        if p>l then
          RaiseInternalError(20170523113938);
        c:=S[p];
        case c of
        '''':
          begin
          if p>StartP then
            Add(copy(S,StartP,p-StartP));
          inc(p);
          StartP:=p;
          if (p>l) or (S[p]<>'''') then
            break;
          Add('''');
          inc(p);
          StartP:=p;
          end;
        else
          inc(p);
        end;
      until false;
      if p>StartP then
        Add(copy(S,StartP,p-StartP));
      end;
    '#':
      begin
      inc(p);
      if p>l then
        RaiseInternalError(20181016121354);
      if S[p]='$' then
        begin
        // #$hexnumber
        inc(p);
        StartP:=p;
        u:=0;
        while p<=l do
          begin
          c:=S[p];
          case c of
          '0'..'9': u:=u*16+longword(ord(c)-ord('0'));
          'a'..'f': u:=u*16+longword(ord(c)-ord('a'))+10;
          'A'..'F': u:=u*16+longword(ord(c)-ord('A'))+10;
          else break;
          end;
          if u>$10FFFF then
            RangeError(20170523115712);
          inc(p);
          end;
        if p=StartP then
          RaiseInternalError(20170207164956);
        if u>$ffff then
          begin
          // split into two
          dec(u,$10000);
          AddHash($D800+(u shr 10),true);
          AddHash($DC00+(u and $3ff),true);
          end
        else
          AddHash(u,p-StartP>2);
        end
      else
        begin
        // #decimalnumber
        StartP:=p;
        u:=0;
        while p<=l do
          begin
          c:=S[p];
          case c of
          '0'..'9': u:=u*10+longword(ord(c)-ord('0'));
          else break;
          end;
          if u>$ffff then
            RangeError(20170523123137);
          inc(p);
          end;
        if p=StartP then
          RaiseInternalError(20170523123806);
        AddHash(u,false);
        end;
      end;
    '^':
      begin
      // ^A is #1
      inc(p);
      if p>l then
        RaiseInternalError(20181016121520);
      c:=S[p];
      case c of
      'a'..'z': AddHash(ord(c)-ord('a')+1,false);
      'A'..'Z': AddHash(ord(c)-ord('A')+1,false);
      else RaiseInternalError(20170523123809);
      end;
      inc(p);
      end;
    else
      RaiseNotYetImplemented(20170523123815,Expr,'ord='+IntToStr(ord(S[p])));
    end;
  {$IFDEF VerbosePasResEval}
  //writeln('TResExprEvaluator.EvalPrimitiveExprString Result=',Result.AsString);
  {$ENDIF}
end;

function TResExprEvaluator.CreateResEvalInt(UInt: TMaxPrecUInt): TResEvalValue;
begin
  if UInt<=HighIntAsUInt then
    Result:=TResEvalInt.CreateValue(TMaxPrecInt(UInt))
  else
    Result:=TResEvalUInt.CreateValue(UInt);
end;

constructor TResExprEvaluator.Create;
begin
  inherited Create;
  FAllowedInts:=ReitDefaults;
  {$ifdef FPC_HAS_CPSTRING}
  FDefaultEncoding:=CP_ACP;
  {$endif}
end;

function TResExprEvaluator.Eval(Expr: TPasExpr; Flags: TResEvalFlags
  ): TResEvalValue;
var
  C: TClass;
  Code: integer;
  Int: TMaxPrecInt;
  UInt: TMaxPrecUInt;
  Flo: TMaxPrecFloat;
begin
  Result:=nil;
  if Expr.CustomData is TResEvalValue then
    begin
    Result:=TResEvalValue(Expr.CustomData);
    exit;
    end;
  {$IFDEF VerbosePasResEval}
  writeln('TResExprEvaluator.Eval Expr=',GetObjName(Expr),' Flags=',dbgs(Flags));
  {$ENDIF}
  if refAutoConst in Flags then
    begin
    Exclude(Flags,refAutoConst);
    if IsConst(Expr) then
      Include(Flags,refConst);
    end;
  if refAutoConstExt in Flags then
    begin
    Exclude(Flags,refAutoConstExt);
    if IsConst(Expr) then
      Include(Flags,refConstExt);
    end;

  C:=Expr.ClassType;
  if C=TPrimitiveExpr then
    begin
    case TPrimitiveExpr(Expr).Kind of
      pekIdent:
        begin
        Result:=OnEvalIdentifier(Self,TPrimitiveExpr(Expr),Flags);
        //writeln('TResExprEvaluator.Eval primitiv result=',Result<>nil,' ',dbgs(Result));
        end;
      pekNumber:
        begin
        // try TMaxPrecInt
        val(TPrimitiveExpr(Expr).Value,Int,Code);
        if Code=0 then
          begin
          {$IFDEF VerbosePasResEval}
          writeln('TResExprEvaluator.Eval Int=',Int,' Value="',TPrimitiveExpr(Expr).Value,'"');
          {$ENDIF}
          if (Int<0) and (Pos('-',TPrimitiveExpr(Expr).Value)<1) then
            // FPC str() converts $8000000000000000 to a negative int64 -> ignore
          else
            begin
            Result:=TResEvalInt.CreateValue(Int);
            exit;
            end;
          end;
        // try TMaxPrecUInt
        val(TPrimitiveExpr(Expr).Value,UInt,Code);
        if Code=0 then
          begin
          Result:=TResEvalUInt.CreateValue(UInt);
          {$IFDEF VerbosePasResEval}
          writeln('TResExprEvaluator.Eval UInt=',UInt,' Value="',TPrimitiveExpr(Expr).Value,'"');
          {$ENDIF}
          exit;
          end;
        // try TMaxPrecFloat
        val(TPrimitiveExpr(Expr).Value,Flo,Code);
        if Code=0 then
          begin
          Result:=TResEvalFloat.CreateValue(Flo);
          {$IFDEF VerbosePasResEval}
          writeln('TResExprEvaluator.Eval Float=',Flo,' Value="',TPrimitiveExpr(Expr).Value,'"');
          {$ENDIF}
          exit;
          end;
        {$IFDEF VerbosePasResEval}
        writeln('TResExprEvaluator.Eval Value="',TPrimitiveExpr(Expr).Value,'"');
        {$ENDIF}
        RaiseRangeCheck(20170518202252,Expr);
        end;
      pekString:
        begin
        Result:=EvalPrimitiveExprString(TPrimitiveExpr(Expr));
        exit;
        end;
    else
      RaiseNotYetImplemented(20170518200951,Expr);
    end;
    {$IFDEF VerbosePasResEval}
    writeln('TResExprEvaluator.Eval primitiv end result=',Result<>nil,' ',dbgs(Result));
    {$ENDIF}
    end
  else if C=TNilExpr then
    Result:=TResEvalValue.CreateKind(revkNil)
  else if C=TBoolConstExpr then
    Result:=TResEvalBool.CreateValue(TBoolConstExpr(Expr).Value)
  else if C=TUnaryExpr then
    Result:=EvalUnaryExpr(TUnaryExpr(Expr),Flags)
  else if C=TBinaryExpr then
    Result:=EvalBinaryExpr(TBinaryExpr(Expr),Flags)
  else if C=TParamsExpr then
    Result:=EvalParamsExpr(TParamsExpr(Expr),Flags)
  else if C=TArrayValues then
    Result:=EvalArrayValuesExpr(TArrayValues(Expr),Flags)
  else if [refConst,refConstExt]*Flags<>[] then
    RaiseConstantExprExp(20170518213800,Expr);
  {$IFDEF VerbosePasResEval}
  writeln('TResExprEvaluator.Eval END ',Expr.ClassName,' result=',Result<>nil,' ',dbgs(Result));
  {$ENDIF}
end;

function TResExprEvaluator.IsInRange(Expr, RangeExpr: TPasExpr;
  EmitHints: boolean): boolean;
var
  Value, RangeValue: TResEvalValue;
begin
  Value:=Eval(Expr,[refAutoConst]);
  if Value=nil then
    exit(true); // a variable -> ok
  RangeValue:=nil;
  try
    RangeValue:=Eval(RangeExpr,[]);
    if RangeValue=nil then
      RaiseNotYetImplemented(20170522171226,RangeExpr);
    Result:=IsInRange(Value,Expr,RangeValue,RangeExpr,EmitHints);
  finally
    ReleaseEvalValue(Value);
    ReleaseEvalValue(RangeValue);
  end;
end;

function TResExprEvaluator.IsInRange(Value: TResEvalValue; ValueExpr: TPasExpr;
  RangeValue: TResEvalValue; RangeExpr: TPasExpr; EmitHints: boolean): boolean;
var
  RgInt: TResEvalRangeInt;
  RgUInt: TResEvalRangeUInt;
  CharIndex: LongWord;
begin
  Result:=false;
  {$IFDEF VerbosePasResEval}
  //writeln('TResExprEvaluator.IsInRange Value=',dbgs(Value),' RangeValue=',dbgs(RangeValue));
  {$ENDIF}
  case RangeValue.Kind of
  revkRangeInt:
    begin
    RgInt:=TResEvalRangeInt(RangeValue);
    case RgInt.ElKind of
      revskBool:
        if Value.Kind=revkBool then
          exit(true)
        else
          RaiseNotYetImplemented(20170522220104,ValueExpr);
      revskEnum:
        begin
        if Value.Kind<>revkEnum then
          RaiseInternalError(20170522172754)
        else if TResEvalEnum(Value).ElType<>RgInt.ElType then
          RaiseInternalError(20170522174028)
        else if (TResEvalEnum(Value).Index<RgInt.RangeStart)
            or (TResEvalEnum(Value).Index>RgInt.RangeEnd) then
          begin
          if EmitHints then
            EmitRangeCheckConst(20170522174406,Value.AsString,
              RgInt.ElementAsString(RgInt.RangeStart),
              RgInt.ElementAsString(RgInt.RangeEnd),
              ValueExpr);
          exit(false);
          end
        else
          exit(true);
        end;
      revskInt: // int..int
        if Value.Kind=revkInt then
          begin
          // int in int..int
          if (TResEvalInt(Value).Int<RgInt.RangeStart)
              or (TResEvalInt(Value).Int>RgInt.RangeEnd) then
            begin
            if EmitHints then
              EmitRangeCheckConst(20170522174958,Value.AsString,
                RgInt.ElementAsString(RgInt.RangeStart),
                RgInt.ElementAsString(RgInt.RangeEnd),
                ValueExpr);
            exit(false);
            end
          else
            exit(true);
          end
        else if Value.Kind=revkUInt then
          begin
          // uint in int..int
          if (TResEvalUInt(Value).UInt>HighIntAsUInt)
              or (TMaxPrecInt(TResEvalUInt(Value).UInt)<RgInt.RangeStart)
              or (TMaxPrecInt(TResEvalUInt(Value).UInt)>RgInt.RangeEnd) then
            begin
            if EmitHints then
              EmitRangeCheckConst(20170522215852,Value.AsString,
                RgInt.ElementAsString(RgInt.RangeStart),
                RgInt.ElementAsString(RgInt.RangeEnd),
                ValueExpr);
            exit(false);
            end
          else
            exit(true);
          end
        else
          begin
          {$IF defined(VerbosePasResEval) or defined(VerbosePasResolver)}
          writeln('TResExprEvaluator.IsInRange Kind=',Value.Kind,' ',Value.AsDebugString);
          {$ENDIF}
          RaiseNotYetImplemented(20170522215906,ValueExpr);
          end;
      revskChar:
        if Value.Kind in revkAllStrings then
          begin
          // string in char..char
          CharIndex:=StringToOrd(Value,ValueExpr);
          if (CharIndex<RgInt.RangeStart) or (CharIndex>RgInt.RangeEnd) then
            begin
            if EmitHints then
              EmitRangeCheckConst(20170522221709,Value.AsString,
                RgInt.ElementAsString(RgInt.RangeStart),
                RgInt.ElementAsString(RgInt.RangeEnd),
                ValueExpr);
            exit(false);
            end
          else
            exit(true);
          end
        else
          RaiseNotYetImplemented(20170522220210,ValueExpr);
    else
      RaiseInternalError(20170522172630);
    end;
    end;
  revkRangeUInt:
    if Value.Kind=revkInt then
      begin
      // int in uint..uint
      RgUInt:=TResEvalRangeUInt(RangeValue);
      if (TResEvalInt(Value).Int<0)
          or (TMaxPrecUInt(TResEvalInt(Value).Int)<RgUInt.RangeStart)
          or (TMaxPrecUInt(TResEvalInt(Value).Int)>RgUInt.RangeEnd) then
        begin
        if EmitHints then
          EmitRangeCheckConst(20170522172250,Value.AsString,
            IntToStr(RgUInt.RangeStart),
            IntToStr(RgUInt.RangeEnd),ValueExpr);
        exit(false);
        end
      else
        exit(true);
      end
    else if Value.Kind=revkUInt then
      begin
      // uint in uint..uint
      RgUInt:=TResEvalRangeUInt(RangeValue);
      if (TResEvalUInt(Value).UInt<RgUInt.RangeStart)
          or (TResEvalUInt(Value).UInt>RgUInt.RangeEnd) then
        begin
        if EmitHints then
          EmitRangeCheckConst(20170522172544,IntToStr(TResEvalUInt(Value).UInt),
            IntToStr(RgUInt.RangeStart),
            IntToStr(RgUInt.RangeEnd),ValueExpr);
        exit(false);
        end
      else
        exit(true);
      end
    else
      begin
      {$IFDEF VerbosePasResEval}
      writeln('TResExprEvaluator.IsInRange Value=',dbgs(Value),' RangeValue=',dbgs(RangeValue));
      {$ENDIF}
      RaiseNotYetImplemented(20170522171551,ValueExpr);
      end;
  else
    {$IFDEF VerbosePasResEval}
    writeln('TResExprEvaluator.IsInRange Value=',dbgs(Value),' RangeValue=',dbgs(RangeValue));
    {$ENDIF}
    RaiseNotYetImplemented(20170522171307,RangeExpr);
  end;
end;

function TResExprEvaluator.IsSetCompatible(Value: TResEvalValue;
  ValueExpr: TPasExpr; RangeValue: TResEvalValue; EmitHints: boolean): boolean;
// checks if Value fits into a set of RangeValue
var
  RightSet: TResEvalSet;
  LeftRange: TResEvalRangeInt;
  MinVal, MaxVal: TMaxPrecInt;
begin
  Result:=true;
  case Value.Kind of
  revkSetOfInt:
    begin
    RightSet:=TResEvalSet(Value);
    if RightSet.ElKind=revskNone then
      exit(true); // empty set always fits
    case RangeValue.Kind of
    revkRangeInt:
      begin
      LeftRange:=TResEvalRangeInt(RangeValue);
      if (LeftRange.ElKind<>RightSet.ElKind)
          or (LeftRange.ElType<>RightSet.ElType) then
        begin
        {$IF defined(VerbosePasResEval) or defined(VerbosePasResolver)}
        writeln('TResExprEvaluator.IsSetCompatible Value=',dbgs(Value),' RangeValue=',dbgs(RangeValue));
        {$ENDIF}
        RaiseNotYetImplemented(20170714201425,ValueExpr);
        end;
      if length(RightSet.Ranges)=0 then
        exit(true); // empty typed set fits
      MinVal:=RightSet.Ranges[0].RangeStart;
      MaxVal:=RightSet.Ranges[length(RightSet.Ranges)-1].RangeEnd;
      {$IFDEF VerbosePasResEval}
      writeln('TResExprEvaluator.IsSetCompatible Value=',dbgs(Value),' MinVal=',MinVal,' MaxVal=',MaxVal,' RangeValue=',dbgs(RangeValue));
      {$ENDIF}
      if (MinVal<LeftRange.RangeStart) then
        if EmitHints then
          EmitRangeCheckConst(20170714202813,RightSet.ElementAsString(MinVal),
            LeftRange.ElementAsString(LeftRange.RangeStart),
            LeftRange.ElementAsString(LeftRange.RangeEnd),ValueExpr,mtError)
        else
          exit(false);
      if (MaxVal>LeftRange.RangeEnd) then
        if EmitHints then
          EmitRangeCheckConst(20170714203134,RightSet.ElementAsString(MaxVal),
            LeftRange.ElementAsString(LeftRange.RangeStart),
            LeftRange.ElementAsString(LeftRange.RangeEnd),ValueExpr,mtError)
        else
          exit(false);
      end;
    else
      {$IF defined(VerbosePasResEval) or defined(VerbosePasResolver)}
      writeln('TResExprEvaluator.IsSetCompatible Value=',dbgs(Value),' RangeValue=',dbgs(RangeValue));
      {$ENDIF}
      RaiseNotYetImplemented(20170714201121,ValueExpr);
    end;
    end
  else
    {$IF defined(VerbosePasResEval) or defined(VerbosePasResolver)}
    writeln('TResExprEvaluator.IsSetCompatible Value=',Value.Kind,' ',dbgs(RangeValue));
    {$ENDIF}
    RaiseNotYetImplemented(20170714195815,ValueExpr);
  end;
end;

function TResExprEvaluator.IsConst(Expr: TPasExpr): boolean;
var
  El: TPasElement;
  C: TClass;
begin
  El:=Expr;
  while El<>nil do
    begin
    C:=El.ClassType;
    if C.InheritsFrom(TPasProcedure) then exit(true);
    if C.InheritsFrom(TPasImplBlock) then exit(false);
    El:=El.Parent;
    end;
  Result:=true;
end;

function TResExprEvaluator.IsSimpleExpr(Expr: TPasExpr): boolean;
var
  C: TClass;
begin
  C:=Expr.ClassType;
  Result:=(C=TNilExpr)
       or (C=TBoolConstExpr)
       or (C=TPrimitiveExpr);
end;

procedure TResExprEvaluator.EmitRangeCheckConst(id: TMaxPrecInt; const aValue,
  MinVal, MaxVal: String; PosEl: TPasElement; MsgType: TMessageType);
begin
  if Assigned(OnRangeCheckEl) then
    OnRangeCheckEl(Self,PosEl,MsgType);
  LogMsg(id,MsgType,nRangeCheckEvaluatingConstantsVMinMax,
    sRangeCheckEvaluatingConstantsVMinMax,[aValue,MinVal,MaxVal],PosEl);
end;

procedure TResExprEvaluator.EmitRangeCheckConst(id: TMaxPrecInt;
  const aValue: String; MinVal, MaxVal: TMaxPrecInt; PosEl: TPasElement;
  MsgType: TMessageType);
begin
  EmitRangeCheckConst(id,aValue,IntToStr(MinVal),IntToStr(MaxVal),PosEl,MsgType);
end;

function TResExprEvaluator.ChrValue(Value: TResEvalValue; ErrorEl: TPasElement
  ): TResEvalValue;
var
  Int: TMaxPrecInt;
begin
  Result:=nil;
  case Value.Kind of
    revkInt,revkUInt:
      begin
      if Value.Kind=revkUInt then
        begin
        if TResEvalUInt(Value).UInt>$ffff then
          EmitRangeCheckConst(20170711195605,Value.AsString,0,$ffff,ErrorEl,mtError)
        else
          Int:=TResEvalUInt(Value).UInt;
        end
      else
        Int:=TResEvalInt(Value).Int;
      if (Int<0) or (Int>$ffff) then
        EmitRangeCheckConst(20170711195747,Value.AsString,0,$ffff,ErrorEl,mtError);
      {$ifdef FPC_HAS_CPSTRING}
      if Int<=$ff then
        Result:=TResEvalString.CreateValue(chr(Int))
      else
      {$endif}
        Result:=TResEvalUTF16.CreateValue(WideChar(Int))
      end;
  else
    {$IFDEF VerbosePasResEval}
    writeln('TResExprEvaluator.ChrValue ',Value.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170711195440,ErrorEl);
  end;
end;

function TResExprEvaluator.OrdValue(Value: TResEvalValue; ErrorEl: TPasElement
  ): TResEvalValue;
var
  v: longword;
begin
  Result:=nil;
  v:=0;
  case Value.Kind of
    revkBool:
      if TResEvalBool(Value).B then
        v:=1
      else
        v:=0;
    revkInt,revkUInt:
      exit(Value);
    {$ifdef FPC_HAS_CPSTRING}
    revkString,
    {$endif}
    revkUnicodeString:
      v:=StringToOrd(Value,ErrorEl);
    revkEnum:
      v:=TResEvalEnum(Value).Index;
  else
    {$IFDEF VerbosePasResEval}
    writeln('TResExprEvaluator.OrdValue ',Value.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20170624155932,ErrorEl);
  end;
  if v>$ffff then exit;
  Result:=TResEvalInt.CreateValue(v);
end;

procedure TResExprEvaluator.PredValue(Value: TResEvalValue; ErrorEl: TPasElement
  );
begin
  case Value.Kind of
    revkBool:
      PredBool(TResEvalBool(Value),ErrorEl);
    revkInt:
      PredInt(TResEvalInt(Value),ErrorEl);
    revkUInt:
      PredUInt(TResEvalUInt(Value),ErrorEl);
    {$ifdef FPC_HAS_CPSTRING}
    revkString:
      PredString(TResEvalString(Value),ErrorEl);
    {$endif}
    revkUnicodeString:
      PredUnicodeString(TResEvalUTF16(Value),ErrorEl);
    revkEnum:
      PredEnum(TResEvalEnum(Value),ErrorEl);
  else
    {$IFDEF VerbosePasResEval}
    writeln('TResExprEvaluator.PredValue ',Value.AsDebugString);
    {$ENDIF}
    ReleaseEvalValue(Value);
    RaiseNotYetImplemented(20170624135738,ErrorEl);
  end;
end;

procedure TResExprEvaluator.SuccValue(Value: TResEvalValue; ErrorEl: TPasElement
  );
begin
  case Value.Kind of
    revkBool:
      SuccBool(TResEvalBool(Value),ErrorEl);
    revkInt:
      SuccInt(TResEvalInt(Value),ErrorEl);
    revkUInt:
      SuccUInt(TResEvalUInt(Value),ErrorEl);
    {$ifdef FPC_HAS_CPSTRING}
    revkString:
      SuccString(TResEvalString(Value),ErrorEl);
    {$endif}
    revkUnicodeString:
      SuccUnicodeString(TResEvalUTF16(Value),ErrorEl);
    revkEnum:
      SuccEnum(TResEvalEnum(Value),ErrorEl);
  else
    {$IFDEF VerbosePasResEval}
    writeln('TResExprEvaluator.SuccValue ',Value.AsDebugString);
    {$ENDIF}
    ReleaseEvalValue(Value);
    RaiseNotYetImplemented(20170624151252,ErrorEl);
  end;
end;

function TResExprEvaluator.EvalStrFunc(Params: TParamsExpr; Flags: TResEvalFlags
  ): TResEvalValue;
var
  AllConst: Boolean;

  function EvalFormat(Expr: TPasExpr; MinVal, MaxVal: TMaxPrecInt): TMaxPrecInt;
  var
    Value: TResEvalValue;
  begin
    Value:=Eval(Expr,Flags);
    if Value=nil then
      begin
      AllConst:=false;
      exit(-1);
      end;
    if Value.Kind<>revkInt then
      RaiseNotYetImplemented(20170717144010,Expr);
    Result:=TResEvalInt(Value).Int;
    if (Result<MinVal) or (Result>MaxVal) then
      EmitRangeCheckConst(20170717144609,IntToStr(Result),MinVal,MaxVal,Expr,mtError);
  end;

var
  i: Integer;
  Param: TPasExpr;
  S, ValStr: String;
  Value: TResEvalValue;
  Format1, Format2: TMaxPrecInt;
begin
  Result:=nil;
  Value:=nil;
  AllConst:=true;
  S:='';
  for i:=0 to length(Params.Params)-1 do
    begin
    Param:=Params.Params[i];
    {$IFDEF VerbosePasResEval}
    writeln('TPasResolver.BI_StrFunc_OnEval i=',i,' of ',length(Params.Params),' Param=',GetObjName(Param));
    {$ENDIF}
    Value:=Eval(Param,Flags);
    if Value=nil then
      begin
      AllConst:=false;
      continue;
      end;
    Format1:=-1;
    Format2:=-1;
    try
      ValStr:='';
      if Param.format1<>nil then
        begin
        Format1:=EvalFormat(Param.format1,1,255);
        if Format1<0 then
          continue;
        if Param.format2<>nil then
          begin
          Format2:=EvalFormat(Param.format2,0,255);
          if Format2<0 then
            continue;
          end;
        end;
      case Value.Kind of
      revkBool:
        if Format1<0 then
          str(TResEvalBool(Value).B,ValStr)
        else
          str(TResEvalBool(Value).B:Format1,ValStr);
      revkInt:
        if Format1<0 then
          str(TResEvalInt(Value).Int,ValStr)
        else
          str(TResEvalInt(Value).Int:Format1,ValStr);
      revkUInt:
        if Format1<0 then
          str(TResEvalUInt(Value).UInt,ValStr)
        else
          str(TResEvalUInt(Value).UInt:Format1,ValStr);
      revkFloat:
        if Format1<0 then
          str(TResEvalFloat(Value).FloatValue,ValStr)
        else if Format2<0 then
          str(TResEvalFloat(Value).FloatValue:Format1,ValStr)
        else
          str(TResEvalFloat(Value).FloatValue:Format1:Format2,ValStr);
      revkCurrency:
        if Format1<0 then
          str(TResEvalCurrency(Value).Value,ValStr)
        else if Format2<0 then
          str(TResEvalCurrency(Value).Value:Format1,ValStr)
        else
          str(TResEvalCurrency(Value).Value:Format1:Format2,ValStr);
      revkEnum:
        begin
        ValStr:=TResEvalEnum(Value).AsString;
        if Format1>0 then
          ValStr:=StringOfChar(' ',Format1)+ValStr;
        end;
      else
        AllConst:=false;
        continue;
      end;
    finally
      ReleaseEvalValue(Value);
      ReleaseEvalValue(Value);
      ReleaseEvalValue(Value);
    end;
    S:=S+ValStr;
  end;
  if AllConst then
    {$ifdef FPC_HAS_CPSTRING}
    Result:=TResEvalString.CreateValue(S);
    {$else}
    Result:=TResEvalUTF16.CreateValue(S);
    {$endif}
end;

function TResExprEvaluator.EvalStringAddExpr(Expr, LeftExpr,
  RightExpr: TPasExpr; LeftValue, RightValue: TResEvalValue): TResEvalValue;
{$ifdef FPC_HAS_CPSTRING}
var
  LeftCP, RightCP: TSystemCodePage;
{$endif}
begin
  case LeftValue.Kind of
  {$ifdef FPC_HAS_CPSTRING}
  revkString:
    case RightValue.Kind of
    revkString:
      begin
      LeftCP:=GetCodePage(TResEvalString(LeftValue).S);
      RightCP:=GetCodePage(TResEvalString(RightValue).S);
      if (LeftCP=RightCP) then
        begin
        Result:=TResEvalString.Create;
        TResEvalString(Result).S:=TResEvalString(LeftValue).S+TResEvalString(RightValue).S;
        end
      else
        begin
        Result:=TResEvalUTF16.Create;
        TResEvalUTF16(Result).S:=GetUnicodeStr(TResEvalString(LeftValue).S,LeftExpr)
                                +GetUnicodeStr(TResEvalString(RightValue).S,RightExpr);
        end;
      end;
    revkUnicodeString:
      begin
      Result:=TResEvalUTF16.Create;
      TResEvalUTF16(Result).S:=GetUnicodeStr(TResEvalString(LeftValue).S,LeftExpr)
                              +TResEvalUTF16(RightValue).S;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryAddExpr string+? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170601141834,Expr);
    end;
  {$endif}
  revkUnicodeString:
    case RightValue.Kind of
    {$ifdef FPC_HAS_CPSTRING}
    revkString:
      begin
      Result:=TResEvalUTF16.Create;
      TResEvalUTF16(Result).S:=TResEvalUTF16(LeftValue).S
                              +GetUnicodeStr(TResEvalString(RightValue).S,RightExpr);
      end;
    {$endif}
    revkUnicodeString:
      begin
      Result:=TResEvalUTF16.Create;
      TResEvalUTF16(Result).S:=TResEvalUTF16(LeftValue).S+TResEvalUTF16(RightValue).S;
      end;
    else
      {$IFDEF VerbosePasResolver}
      writeln('TResExprEvaluator.EvalBinaryAddExpr utf16+? Left=',LeftValue.AsDebugString,' Right=',RightValue.AsDebugString);
      {$ENDIF}
      RaiseNotYetImplemented(20170601141811,Expr);
    end;
  else
    {$ifndef FPC_HAS_CPSTRING}
    if LeftExpr=nil then ; // no Parameter "LeftExpr" not used
    if RightExpr=nil then ; // no Parameter "RightExpr" not used
    {$endif}
    RaiseNotYetImplemented(20181219233139,Expr);
  end;
end;

function TResExprEvaluator.LoHiValue(Value: TResEvalValue; ShiftSize: Integer;
  Mask: LongWord; ErrorEl: TPasElement): TResEvalValue;
var
  uint: LongWord;
begin
  case Value.Kind of
    revkInt:
      {$IFDEF Pas2js}
      if ShiftSize=32 then
        uint := longword(TResEvalInt(Value).Int div $100000000)
      else
      {$ENDIF}
        uint := (TResEvalInt(Value).Int shr ShiftSize) and Mask;
    revkUInt:
      {$IFDEF Pas2js}
      if ShiftSize=32 then
        uint := longword(TResEvalUInt(Value).UInt div $100000000)
      else
      {$ENDIF}
        uint := (TResEvalUInt(Value).UInt shr ShiftSize) and Mask;
  else
    {$IFDEF VerbosePasResEval}
    writeln('TResExprEvaluator.LoHiValue ',Value.AsDebugString);
    {$ENDIF}
    RaiseNotYetImplemented(20190129012100,ErrorEl);
  end;
  Result := TResEvalInt.CreateValue(uint);
end;

function TResExprEvaluator.EnumTypeCast(EnumType: TPasEnumType; Expr: TPasExpr;
  Flags: TResEvalFlags): TResEvalEnum;
var
  Value: TResEvalValue;
  MaxIndex, Index: Integer;
begin
  Result:=nil;
  Value:=Eval(Expr,Flags);
  if Value=nil then exit;
  try
    MaxIndex:=EnumType.Values.Count-1;
    case Value.Kind of
    revkInt:
      if TResEvalInt(Value).Int>High(Index) then
        EmitRangeCheckConst(20170713105944,
          IntToStr(TResEvalInt(Value).Int),'0',IntToStr(MaxIndex),Expr,mtError)
      else
        Index:=TResEvalInt(Value).Int;
    revkUInt:
      if TResEvalUInt(Value).UInt>MaxIndex then
        EmitRangeCheckConst(20170713105944,
          IntToStr(TResEvalUInt(Value).UInt),'0',IntToStr(MaxIndex),Expr,mtError)
      else
        Index:=TResEvalUInt(Value).UInt;
    else
      RaiseNotYetImplemented(20170713105625,Expr);
    end;
    if (Index<0) or (Index>MaxIndex) then
      EmitRangeCheckConst(20170713110232,
        IntToStr(Index),'0',IntToStr(MaxIndex),Expr,mtError);
    Result:=TResEvalEnum.CreateValue(Index,TPasEnumValue(EnumType.Values[Index]));
  finally
    ReleaseEvalValue(Value);
  end;
end;

{$ifdef FPC_HAS_CPSTRING}
function TResExprEvaluator.CheckValidUTF8(const s: RawByteString;
  ErrorEl: TPasElement): boolean;
var
  p, EndP: PChar;
  l: SizeInt;
begin
  p:=PChar(s);
  EndP:=p+length(s);
  while p<EndP do
    begin
    l:=Utf8CodePointLen(p,EndP-p,false);
    if l<=0 then
      if ErrorEl<>nil then
        RaiseMsg(20170711211841,nIllegalChar,sIllegalChar,[],ErrorEl)
      else
        exit(false);
    inc(p,l);
    end;
  Result:=true;
end;

function TResExprEvaluator.GetCodePage(const s: RawByteString): TSystemCodePage;
begin
  if s='' then exit(DefaultStringCodePage);
  Result:=StringCodePage(s);
  if (Result=CP_ACP) or (Result=CP_NONE) then
    begin
    Result:=DefaultStringCodePage;
    if (Result=CP_ACP) or (Result=CP_NONE) then
      begin
      Result:=System.DefaultSystemCodePage;
      if Result=CP_NONE then
        Result:=CP_ACP;
      end;
    end;
end;

function TResExprEvaluator.GetRawByteString(const s: UnicodeString;
  CodePage: TSystemCodePage; ErrorEl: TPasElement): RawByteString;
var
  ok: Boolean;
begin
  Result:=UTF8Encode(s);
  if (CodePage=CP_UTF8)
      or ((DefaultSystemCodePage=CP_UTF8) and ((CodePage=CP_ACP) or (CodePage=CP_NONE))) then
    begin
    // to UTF-8
    SetCodePage(Result,CodePage,false);
    end
  else
    begin
    // to non UTF-8 -> possible loss
    ok:=false;
    try
      SetCodePage(Result,CodePage,true);
      ok:=true;
    except
    end;
    if (not ok) or (GetUnicodeStr(Result,ErrorEl)<>s) then
      LogMsg(20190204165110,mtWarning,nImplictConversionUnicodeToAnsi,
        sImplictConversionUnicodeToAnsi,[],ErrorEl);
    end;
end;

function TResExprEvaluator.GetUTF8Str(const s: RawByteString;
  ErrorEl: TPasElement): String;
var
  CP: TSystemCodePage;
begin
  if s='' then exit('');
  CP:=GetCodePage(s);
  if CP=CP_UTF8 then
    begin
    if ErrorEl<>nil then
      CheckValidUTF8(s,ErrorEl);
    Result:=s;
    end
  else
    // use default conversion
    Result:=UTF8Encode(UnicodeString(s));
end;

function TResExprEvaluator.GetUnicodeStr(const s: RawByteString;
  ErrorEl: TPasElement): UnicodeString;
var
  CP: TSystemCodePage;
begin
  if s='' then exit('');
  CP:=GetCodePage(s);
  if CP=CP_UTF8 then
    begin
    if ErrorEl<>nil then
      CheckValidUTF8(s,ErrorEl);
    Result:=UTF8Decode(s);
    end
  else
    // use default conversion
    Result:=UnicodeString(s);
end;

function TResExprEvaluator.GetWideChar(const s: RawByteString; out w: WideChar
  ): boolean;
var
  CP: TSystemCodePage;
  u: UnicodeString;
begin
  w:=#0;
  Result:=false;
  if s='' then exit;
  CP:=GetCodePage(s);
  if CP=CP_UTF8 then
    begin
    if length(s)>4 then exit;
    u:=UTF8Decode(s);
    if length(u)<>1 then exit;
    w:=u[1];
    Result:=true;
    end
  else if length(s)=1 then
    begin
    w:=s[1];
    Result:=true;
    end;
end;
{$endif}

procedure TResExprEvaluator.PredBool(Value: TResEvalBool; ErrorEl: TPasElement);
begin
  if Value.B=false then
    EmitRangeCheckConst(20170624140251,Value.AsString,
      'true','true',ErrorEl);
  Value.B:=not Value.B;
end;

procedure TResExprEvaluator.SuccBool(Value: TResEvalBool; ErrorEl: TPasElement);
begin
  if Value.B=true then
    EmitRangeCheckConst(20170624142316,Value.AsString,
      'false','false',ErrorEl);
  Value.B:=not Value.B;
end;

procedure TResExprEvaluator.PredInt(Value: TResEvalInt; ErrorEl: TPasElement);
begin
  if Value.Int=low(TMaxPrecInt) then
    begin
    EmitRangeCheckConst(20170624142511,IntToStr(Value.Int),
      IntToStr(succ(low(TMaxPrecInt))),IntToStr(high(TMaxPrecInt)),ErrorEl);
    Value.Int:=high(Value.Int);
    end
  else
    dec(Value.Int);
end;

procedure TResExprEvaluator.SuccInt(Value: TResEvalInt; ErrorEl: TPasElement);
begin
  if Value.Int=high(TMaxPrecInt) then
    begin
    EmitRangeCheckConst(20170624142920,IntToStr(Value.Int),
      IntToStr(low(TMaxPrecInt)),IntToStr(pred(high(TMaxPrecInt))),ErrorEl);
    Value.Int:=low(Value.Int);
    end
  else
    inc(Value.Int);
end;

procedure TResExprEvaluator.PredUInt(Value: TResEvalUInt; ErrorEl: TPasElement);
begin
  if Value.UInt=low(TMaxPrecUInt) then
    begin
    EmitRangeCheckConst(20170624143122,IntToStr(Value.UInt),
      IntToStr(succ(low(TMaxPrecUInt))),IntToStr(high(TMaxPrecUInt)),ErrorEl);
    Value.UInt:=high(Value.UInt);
    end
  else
    dec(Value.UInt);
end;

procedure TResExprEvaluator.SuccUInt(Value: TResEvalUInt; ErrorEl: TPasElement);
begin
  // Note: when FPC compares int64 with qword it converts the qword to an int64
  if Value.UInt=HighIntAsUInt then
    begin
    EmitRangeCheckConst(20170624142921,IntToStr(Value.UInt),
      IntToStr(low(TMaxPrecUInt)),IntToStr(pred(high(TMaxPrecUInt))),ErrorEl);
    Value.UInt:=low(Value.UInt);
    end
  else
    inc(Value.UInt);
end;

{$ifdef FPC_HAS_CPSTRING}
procedure TResExprEvaluator.PredString(Value: TResEvalString;
  ErrorEl: TPasElement);
begin
  if length(Value.S)<>1 then
    RaiseRangeCheck(20170624150138,ErrorEl);
  if Value.S[1]=#0 then
    begin
    EmitRangeCheckConst(20170624150220,Value.AsString,'#1','#255',ErrorEl);
    Value.S:=#255;
    end
  else
    Value.S:=pred(Value.S[1]);
end;

procedure TResExprEvaluator.SuccString(Value: TResEvalString;
  ErrorEl: TPasElement);
begin
  if length(Value.S)<>1 then
    RaiseRangeCheck(20170624150432,ErrorEl);
  if Value.S[1]=#255 then
    begin
    EmitRangeCheckConst(20170624150441,Value.AsString,'#0','#254',ErrorEl);
    Value.S:=#0;
    end
  else
    Value.S:=succ(Value.S[1]);
end;
{$endif}

procedure TResExprEvaluator.PredUnicodeString(Value: TResEvalUTF16;
  ErrorEl: TPasElement);
begin
  if length(Value.S)<>1 then
    RaiseRangeCheck(20170624150703,ErrorEl);
  if Value.S[1]=#0 then
    begin
    EmitRangeCheckConst(20170624150710,Value.AsString,'#1','#65535',ErrorEl);
    Value.S:=WideChar(#65535);
    end
  else
    Value.S:=pred(Value.S[1]);
end;

procedure TResExprEvaluator.SuccUnicodeString(Value: TResEvalUTF16;
  ErrorEl: TPasElement);
begin
  if length(Value.S)<>1 then
    RaiseRangeCheck(20170624150849,ErrorEl);
  if Value.S[1]=#65535 then
    begin
    EmitRangeCheckConst(20170624150910,Value.AsString,'#0','#65534',ErrorEl);
    Value.S:=#0;
    end
  else
    Value.S:=succ(Value.S[1]);
end;

procedure TResExprEvaluator.PredEnum(Value: TResEvalEnum; ErrorEl: TPasElement);
var
  EnumType: TPasEnumType;
begin
  EnumType:=Value.ElType as TPasEnumType;
  if EnumType=nil then
    RaiseInternalError(20170821174038,dbgs(Value));
  if Value.Index<=0 then
    begin
    EmitRangeCheckConst(20170624144332,Value.AsString,
      TPasEnumValue(EnumType.Values[Min(1,EnumType.Values.Count-1)]).Name,
      TPasEnumValue(EnumType.Values[EnumType.Values.Count-1]).Name,ErrorEl);
    Value.Index:=EnumType.Values.Count-1;
    end
  else
    dec(Value.Index);
  Value.IdentEl:=TPasEnumValue(EnumType.Values[Value.Index]);
end;

procedure TResExprEvaluator.SuccEnum(Value: TResEvalEnum; ErrorEl: TPasElement);
var
  EnumType: TPasEnumType;
begin
  EnumType:=Value.ElType as TPasEnumType;
  if EnumType=nil then
    RaiseInternalError(20170821174058,dbgs(Value));
  if Value.Index>=EnumType.Values.Count-1 then
    begin
    EmitRangeCheckConst(20170624145013,Value.AsString,
      TPasEnumValue(EnumType.Values[0]).Name,
      TPasEnumValue(EnumType.Values[Max(0,EnumType.Values.Count-2)]).Name,ErrorEl);
    Value.Index:=0;
    end
  else
    inc(Value.Index);
  Value.IdentEl:=TPasEnumValue(EnumType.Values[Value.Index]);
end;

{ TResolveData }

procedure TResolveData.SetElement(AValue: TPasElement);
begin
  if FElement=AValue then Exit;
  if Element<>nil then
    Element.Release{$IFDEF CheckPasTreeRefCount}(ClassName+'.SetElement'){$ENDIF};
  FElement:=AValue;
  if Element<>nil then
    Element.AddRef{$IFDEF CheckPasTreeRefCount}(ClassName+'.SetElement'){$ENDIF};
end;

constructor TResolveData.Create;
begin

end;

destructor TResolveData.Destroy;
begin
  {$IFDEF VerbosePasResolverMem}
  writeln('TResolveData.Destroy START ',ClassName);
  {$ENDIF}
  Element:=nil;
  Owner:=nil;
  Next:=nil;
  inherited Destroy;
  {$IFDEF VerbosePasResolverMem}
  writeln('TResolveData.Destroy END ',ClassName);
  {$ENDIF}
end;

{ TResEvalValue }

constructor TResEvalValue.CreateKind(const aKind: TREVKind);
begin
  Create;
  Kind:=aKind;
end;

function TResEvalValue.Clone: TResEvalValue;
begin
  Result:=TResEvalValueClass(ClassType).Create;
  Result.Kind:=Kind;
  Result.IdentEl:=IdentEl;
end;

function TResEvalValue.AsDebugString: string;
begin
  str(Kind,Result);
  Result:=Result+'='+AsString;
end;

function TResEvalValue.AsString: string;
begin
  case Kind of
    revkNone: Result:='<None>';
    revkNil: Result:='nil';
  else
    str(Kind,Result);
  end;
end;

{ TResEvalUInt }

constructor TResEvalUInt.Create;
begin
  inherited Create;
  Kind:=revkUInt;
end;

constructor TResEvalUInt.CreateValue(const aValue: TMaxPrecUInt);
begin
  Create;
  UInt:=aValue;
end;

function TResEvalUInt.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalUInt(Result).UInt:=UInt;
end;

function TResEvalUInt.AsString: string;
begin
  Result:=IntToStr(UInt);
end;

{ TResEvalInt }

constructor TResEvalInt.Create;
begin
  inherited Create;
  Kind:=revkInt;
end;

constructor TResEvalInt.CreateValue(const aValue: TMaxPrecInt);
begin
  Create;
  Int:=aValue;
end;

constructor TResEvalInt.CreateValue(const aValue: TMaxPrecInt; aTyped: TResEvalTypedInt
  );
begin
  Create;
  Int:=aValue;
  Typed:=aTyped;
end;

function TResEvalInt.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalInt(Result).Int:=Int;
  TResEvalInt(Result).Typed:=Typed;
end;

function TResEvalInt.AsString: string;
begin
  Result:=IntToStr(Int);
end;

function TResEvalInt.AsDebugString: string;
begin
  if Typed=reitNone then
    Result:=inherited AsDebugString
  else
    begin
    str(Kind,Result);
    case Typed of
      reitByte: Result:=Result+':byte';
      reitShortInt: Result:=Result+':shortint';
      reitWord: Result:=Result+':word';
      reitSmallInt: Result:=Result+':smallint';
      reitUIntSingle: Result:=Result+':uintsingle';
      reitIntSingle: Result:=Result+':intsingle';
      reitLongWord: Result:=Result+':longword';
      reitLongInt: Result:=Result+':longint';
      reitUIntDouble: Result:=Result+':uintdouble';
      reitIntDouble: Result:=Result+':intdouble';
    end;
    Result:=Result+'='+AsString;
    end;
end;

{ TResEvalFloat }

constructor TResEvalFloat.Create;
begin
  inherited Create;
  Kind:=revkFloat;
end;

constructor TResEvalFloat.CreateValue(const aValue: TMaxPrecFloat);
begin
  Create;
  FloatValue:=aValue;
end;

function TResEvalFloat.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalFloat(Result).FloatValue:=FloatValue;
end;

function TResEvalFloat.AsString: string;
begin
  str(FloatValue,Result);
end;

function TResEvalFloat.IsInt(out Int: TMaxPrecInt): boolean;
begin
  Int:=0;
  if Frac(FloatValue)<>0 then exit(false);
  if FloatValue<TMaxPrecFloat(low(TMaxPrecInt)) then exit(false);
  if FloatValue>TMaxPrecFloat(high(TMaxPrecInt)) then exit(false);
  Int:=Trunc(FloatValue);
  Result:=true;
end;

{$ifdef FPC_HAS_CPSTRING}
{ TResEvalString }

constructor TResEvalString.Create;
begin
  inherited Create;
  Kind:=revkString;
end;

constructor TResEvalString.CreateValue(const aValue: RawByteString);
begin
  Create;
  S:=aValue;
end;

function TResEvalString.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalString(Result).S:=S;
end;

function TResEvalString.AsString: string;
begin
  Result:=RawStrToCaption(S,60);
end;
{$endif}

{ TResEvalUTF16 }

constructor TResEvalUTF16.Create;
begin
  inherited Create;
  Kind:=revkUnicodeString;
end;

constructor TResEvalUTF16.CreateValue(const aValue: UnicodeString);
begin
  Create;
  S:=aValue;
end;

function TResEvalUTF16.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalUTF16(Result).S:=S;
end;

function TResEvalUTF16.AsString: string;
begin
  Result:=String(UnicodeStrToCaption(S,60));
end;

{ TResEvalEnum }

constructor TResEvalEnum.Create;
begin
  inherited Create;
  Kind:=revkEnum;
end;

constructor TResEvalEnum.CreateValue(const aValue: integer;
  aIdentEl: TPasEnumValue);
begin
  Create;
  Index:=aValue;
  IdentEl:=aIdentEl;
  ElType:=IdentEl.Parent as TPasEnumType;
  if ElType=nil then
    raise Exception.Create('');
end;

function TResEvalEnum.GetEnumValue: TPasEnumValue;
begin
  Result:=nil;
  if ElType<>nil then
    if (Index>=0) and (Index<ElType.Values.Count) then
      Result:=TObject(ElType.Values[Index]) as TPasEnumValue;
end;

function TResEvalEnum.GetEnumName: String;
var
  V: TPasEnumValue;
begin
  V:=GetEnumValue;
  if V<>nil then
    Result:=V.Name
  else
    Result:='';
end;

function TResEvalEnum.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalEnum(Result).Index:=Index;
  TResEvalEnum(Result).ElType:=ElType;
end;

function TResEvalEnum.AsDebugString: string;
begin
  str(Kind,Result);
  Result:=Result+'='+AsString+'='+IntToStr(Index);
end;

function TResEvalEnum.AsString: string;
begin
  if IdentEl<>nil then
    begin
    Result:=IdentEl.Name;
    if Result<>'' then exit;
    end;
  Result:=GetEnumName;
  if Result<>'' then exit;
  Result:=ElType.Name+'('+IntToStr(Index)+')';
end;

{ TResEvalRangeInt }

constructor TResEvalRangeInt.Create;
begin
  inherited Create;
  Kind:=revkRangeInt;
end;

constructor TResEvalRangeInt.CreateValue(const aElKind: TRESetElKind;
  aElType: TPasType; const aRangeStart, aRangeEnd: TMaxPrecInt);
begin
  Create;
  ElKind:=aElKind;
  ElType:=aElType;
  RangeStart:=aRangeStart;
  RangeEnd:=aRangeEnd;
end;

function TResEvalRangeInt.Clone: TResEvalValue;
begin
  Result:=inherited Clone;
  TResEvalRangeInt(Result).ElKind:=ElKind;
  TResEvalRangeInt(Result).RangeStart:=RangeStart;
  TResEvalRangeInt(Result).RangeEnd:=RangeEnd;
end;

function TResEvalRangeInt.AsString: string;
begin
  Result:=ElementAsString(RangeStart)+'..'+ElementAsString(RangeEnd);
end;

function TResEvalRangeInt.AsDebugString: string;
var
  s: string;
begin
  str(Kind,Result);
  str(ElKind,s);
  Result:=Result+'/'+s+':'+GetObjName(ElType)+'='+AsString;
end;

function TResEvalRangeInt.ElementAsString(El: TMaxPrecInt): string;
var
  EnumValue: TPasEnumValue;
  EnumType: TPasEnumType;
begin
  case ElKind of
    revskBool:
      if El=0 then
        Result:='false'
      else
        Result:='true';
    revskEnum:
      begin
      EnumType:=ElType as TPasEnumType;
      if (El>=0) and (El<EnumType.Values.Count) then
        begin
        EnumValue:=TPasEnumValue(EnumType.Values[El]);
        Result:=EnumValue.Name;
        end
      else
        Result:=ElType.Name+'('+IntToStr(El)+')';
      end;
    revskInt: Result:=IntToStr(El);
    revskChar:
      if ((El>=32) and (El<=38)) or ((El>=40) and (El<=126)) then
        Result:=''''+Chr(El)+''''
      else
        Result:='#'+IntToStr(El);
  end;
end;

{ TResEvalSet }

constructor TResEvalSet.Create;
begin
  inherited Create;
  Kind:=revkSetOfInt;
end;

constructor TResEvalSet.CreateEmpty(const aElKind: TRESetElKind;
  aElType: TPasType);
begin
  Create;
  ElKind:=aElKind;
  ElType:=aElType;
end;

constructor TResEvalSet.CreateEmptySameKind(aSet: TResEvalSet);
begin
  Create;
  IdentEl:=aSet.IdentEl;
  ElKind:=aSet.ElKind;
  ElType:=aSet.ElType;
end;

constructor TResEvalSet.CreateValue(const aElKind: TRESetElKind;
  aElType: TPasType; const aRangeStart, aRangeEnd: TMaxPrecInt);
begin
  inherited CreateValue(aElKind, aElType, aRangeStart, aRangeEnd);
  Add(aRangeStart,aRangeEnd);
end;

function TResEvalSet.Clone: TResEvalValue;
var
  RS: TResEvalSet;
  i: Integer;
begin
  Result:=inherited Clone;
  RS:=TResEvalSet(Result);
  RS.ElKind:=ElKind;
  RS.ElType:=ElType;
  SetLength(RS.Ranges,length(Ranges));
  for i:=0 to length(Ranges)-1 do
    RS.Ranges[i]:=Ranges[i];
end;

function TResEvalSet.AsString: string;
var
  i: Integer;
begin
  Result:='[';
  for i:=0 to length(Ranges)-1 do
    begin
    if i>0 then Result:=Result+',';
    Result:=Result+ElementAsString(Ranges[i].RangeStart);
    if Ranges[i].RangeStart<>Ranges[i].RangeEnd then
      Result:=Result+'..'+ElementAsString(Ranges[i].RangeEnd);
    end;
  Result:=Result+']';
end;

function TResEvalSet.Add(aRangeStart, aRangeEnd: TMaxPrecInt): boolean;

  {$IF FPC_FULLVERSION<30101}
  procedure Insert(const Item: TItem; var Items: TItems; Index: integer);
  var
    i: Integer;
  begin
    Setlength(Items,length(Items)+1);
    for i:=length(Items)-1 downto Index+1 do
      Items[i]:=Items[i-1];
    Items[Index]:=Item;
  end;

  procedure Delete(var Items: TItems; Start, Size: integer);
  var
    i: Integer;
  begin
    if Size=0 then exit;
    for i:=Start+Size to length(Items)-1 do
      Items[i-Size]:=Items[i];
    Setlength(Items,length(Items)-Size);
  end;
  {$ENDIF}

var
  StartIndex, l, EndIndex: Integer;
  Item: TItem;
begin
  Result:=false;
  {$IFDEF VerbosePasResEval}
  writeln('TResEvalSetInt.Add ',aRangeStart,'..',aRangeEnd);
  {$ENDIF}
  if aRangeStart>aRangeEnd then
    raise Exception.Create('');
  if ElKind=revskNone then
    raise Exception.Create('');

  l:=length(Ranges);
  if l=0 then
    begin
    // first range
    RangeStart:=aRangeStart;
    RangeEnd:=aRangeEnd;
    SetLength(Ranges,1);
    Ranges[0].RangeStart:=aRangeStart;
    Ranges[0].RangeEnd:=aRangeEnd;
    exit(true);
    end;
  if RangeStart>aRangeStart then
    RangeStart:=aRangeStart;
  if RangeEnd<aRangeEnd then
    RangeEnd:=aRangeEnd;

  // find insert position
  StartIndex:=IndexOfRange(aRangeStart,true);
  if (StartIndex>0) and (Ranges[StartIndex-1].RangeEnd=aRangeStart-1) then
    dec(StartIndex);
  if StartIndex=l then
    begin
    // add new range
    Item.RangeStart:=aRangeStart;
    Item.RangeEnd:=aRangeEnd;
    Insert(Item,Ranges,StartIndex);
    Result:=true;
    end
  else
    begin
    // StartIndex is now the first affected range
    EndIndex:=IndexOfRange(aRangeEnd,true);
    if (EndIndex>StartIndex) then
      if (EndIndex=l) or (Ranges[EndIndex].RangeStart>aRangeEnd+1) then
        dec(EndIndex);
    // EndIndex is now the last affected range
    if StartIndex>EndIndex then
      raise Exception.Create('');
    if StartIndex=EndIndex then
      begin
      if (Ranges[StartIndex].RangeStart>aRangeEnd) then
        begin
        // range in front
        if (Ranges[StartIndex].RangeStart>aRangeEnd+1) then
          begin
          // insert new range
          Item.RangeStart:=aRangeStart;
          Item.RangeEnd:=aRangeEnd;
          Insert(Item,Ranges,StartIndex);
          Result:=true;
          end
        else
          begin
          // enlarge range at its start
          Ranges[StartIndex].RangeStart:=aRangeStart;
          Result:=true;
          end;
        end
      else if Ranges[StartIndex].RangeEnd<aRangeStart then
        begin
        // range behind
        if Ranges[StartIndex].RangeEnd+1<aRangeStart then
          begin
          // insert new range
          Item.RangeStart:=aRangeStart;
          Item.RangeEnd:=aRangeEnd;
          Insert(Item,Ranges,StartIndex+1);
          Result:=true;
          end
        else
          begin
          // enlarge range at its end
          Ranges[StartIndex].RangeEnd:=aRangeEnd;
          Result:=true;
          end;
        end
      else
        begin
        // intersection -> enlarge to union range
        Result:=false;
        if (Ranges[StartIndex].RangeStart>aRangeStart) then
          Ranges[StartIndex].RangeStart:=aRangeStart;
        if (Ranges[StartIndex].RangeEnd<aRangeEnd) then
          Ranges[StartIndex].RangeEnd:=aRangeEnd;
        end;
      end
    else
      begin
      // multiple ranges are merged to one
      Result:=false;
      if Ranges[StartIndex].RangeStart>aRangeStart then
        Ranges[StartIndex].RangeStart:=aRangeStart;
      if aRangeEnd<Ranges[EndIndex].RangeEnd then
        aRangeEnd:=Ranges[EndIndex].RangeEnd;
      Ranges[StartIndex].RangeEnd:=aRangeEnd;
      Delete(Ranges,StartIndex+1,EndIndex-StartIndex);
      end;
    end;
  {$IFDEF VerbosePasResEval}
  writeln('TResEvalSetInt.Add END ',AsDebugString);
  ConsistencyCheck;
  {$ENDIF}
end;

function TResEvalSet.IndexOfRange(Index: TMaxPrecInt; FindInsertPos: boolean
  ): integer;
var
  l, r, m: Integer;
begin
  l:=0;
  r:=length(Ranges)-1;
  while l<=r do
    begin
    m:=(l+r) div 2;
    if Ranges[m].RangeStart>Index then
      r:=m-1
    else if Ranges[m].RangeEnd<Index then
      l:=m+1
    else
      exit(m);
    end;
  if not FindInsertPos then
    exit(-1);
  // find insert position
  if length(Ranges)=0 then
    exit(0)
  else if l>m then
    exit(l)
  else
    exit(m);
  Result:=-1;
end;

function TResEvalSet.Intersects(aRangeStart, aRangeEnd: TMaxPrecInt): integer;
var
  Index: Integer;
begin
  Index:=IndexOfRange(aRangeStart,true);
  if (Index=length(Ranges)) or (Ranges[Index].RangeStart>aRangeEnd) then
    Result:=-1
  else
    Result:=Index;
end;

procedure TResEvalSet.ConsistencyCheck;

  procedure E(Msg: string);
  begin
    raise Exception.Create(Msg);
  end;

var
  i: Integer;
begin
  if (ElKind=revskNone) and (length(Ranges)>0) then
    E('');
  for i:=0 to length(Ranges)-1 do
    begin
    if Ranges[i].RangeStart>Ranges[i].RangeEnd then
      E('');
    if (i>0) and (Ranges[i-1].RangeEnd+1>=Ranges[i].RangeStart) then
      E('missing gap');
    if RangeStart>Ranges[i].RangeStart then
      E('wrong RangeStart='+IntToStr(RangeStart));
    if RangeEnd<Ranges[i].RangeEnd then
      E('wrong RangeEnd='+IntToStr(RangeEnd));
    end;
end;

end.

