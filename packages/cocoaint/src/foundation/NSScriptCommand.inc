{ Parsed from Foundation.framework NSScriptCommand.h }

{$ifdef TYPES}
{$ifndef NSSCRIPTCOMMAND_PAS_T}
{$define NSSCRIPTCOMMAND_PAS_T}

{ Constants }

const
  NSNoScriptError = 0;
  NSReceiverEvaluationScriptError = 1;
  NSKeySpecifierEvaluationScriptError = 2;
  NSArgumentEvaluationScriptError = 3;
  NSReceiversCantHandleCommandScriptError = 4;
  NSRequiredArgumentsMissingScriptError = 5;
  NSArgumentsWrongScriptError = 6;
  NSUnknownKeyScriptError = 7;
  NSInternalScriptError = 8;
  NSOperationNotSupportedForKeyScriptError = 9;
  NSCannotCreateScriptCommandError = 10;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSCRIPTCOMMAND_PAS_R}
{$define NSSCRIPTCOMMAND_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSCRIPTCOMMAND_PAS_F}
{$define NSSCRIPTCOMMAND_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSCRIPTCOMMAND_PAS_S}
{$define NSSCRIPTCOMMAND_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSScriptCommand = objcclass;
  NSScriptCommandPointer = ^NSScriptCommand;
  NSScriptCommandPtr = NSScriptCommandPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSCRIPTCOMMAND_PAS_C}
{$define NSSCRIPTCOMMAND_PAS_C}

{ NSScriptCommand }
  NSScriptCommand = objcclass external (NSObject, NSCodingProtocol)
  private
    _commandDescription: NSScriptCommandDescription;
    _directParameter: id;
    _receiversSpecifier: NSScriptObjectSpecifier;
    _evaluatedReceivers: id;
    _arguments: NSDictionary;
    _evaluatedArguments: NSMutableDictionary;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSScriptCommand0: cuint);
       1: (data: bitpacked record
        hasEvaluatedReceivers: 0..1;
        hasEvaluatedArguments: 0..1;
        RESERVED: 0..((1 shl 30)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSScriptCommand0: cuint);
       1: (
        hasEvaluatedReceivers: 0..1;
        hasEvaluatedArguments: 0..1;
        RESERVED: 0..((1 shl 30)-1);
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
    _moreVars: id;
    _reserved: Pointer;
    
  public
    function initWithCommandDescription(commandDef: NSScriptCommandDescription): id; message 'initWithCommandDescription:';
    function commandDescription: NSScriptCommandDescription; message 'commandDescription';
    procedure setDirectParameter(directParameter_: id); message 'setDirectParameter:';
    function directParameter: id; message 'directParameter';
    procedure setReceiversSpecifier(receiversRef: NSScriptObjectSpecifier); message 'setReceiversSpecifier:';
    function receiversSpecifier: NSScriptObjectSpecifier; message 'receiversSpecifier';
    function evaluatedReceivers: id; message 'evaluatedReceivers';
    procedure setArguments(args: NSDictionary); message 'setArguments:';
    function arguments: NSDictionary; message 'arguments';
    function evaluatedArguments: NSDictionary; message 'evaluatedArguments';
    function isWellFormed: Boolean; message 'isWellFormed';
    function performDefaultImplementation: id; message 'performDefaultImplementation';
    function executeCommand: id; message 'executeCommand';
    procedure setScriptErrorNumber(errorNumber: cint); message 'setScriptErrorNumber:';
    procedure setScriptErrorOffendingObjectDescriptor(errorOffendingObjectDescriptor: NSAppleEventDescriptor); message 'setScriptErrorOffendingObjectDescriptor:';
    procedure setScriptErrorExpectedTypeDescriptor(errorExpectedTypeDescriptor: NSAppleEventDescriptor); message 'setScriptErrorExpectedTypeDescriptor:';
    procedure setScriptErrorString(errorString: NSString); message 'setScriptErrorString:';
    function scriptErrorNumber: cint; message 'scriptErrorNumber';
    function scriptErrorOffendingObjectDescriptor: NSAppleEventDescriptor; message 'scriptErrorOffendingObjectDescriptor';
    function scriptErrorExpectedTypeDescriptor: NSAppleEventDescriptor; message 'scriptErrorExpectedTypeDescriptor';
    function scriptErrorString: NSString; message 'scriptErrorString';
    class function currentCommand: NSScriptCommand; message 'currentCommand';
    function appleEvent: NSAppleEventDescriptor; message 'appleEvent';
    procedure suspendExecution; message 'suspendExecution';
    procedure resumeExecutionWithResult(result_: id); message 'resumeExecutionWithResult:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
