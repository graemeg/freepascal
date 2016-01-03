{ Parsed from Appkit.framework NSTextInputContext.h }

{$ifdef TYPES}
{$ifndef NSTEXTINPUTCONTEXT_PAS_T}
{$define NSTEXTINPUTCONTEXT_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTEXTINPUTCONTEXT_PAS_R}
{$define NSTEXTINPUTCONTEXT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTEXTINPUTCONTEXT_PAS_F}
{$define NSTEXTINPUTCONTEXT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTEXTINPUTCONTEXT_PAS_S}
{$define NSTEXTINPUTCONTEXT_PAS_S}

{ External string constants }
var
  NSTextInputContextKeyboardSelectionDidChangeNotification: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTextInputContext = objcclass;
  NSTextInputContextPointer = ^NSTextInputContext;
  NSTextInputContextPtr = NSTextInputContextPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTEXTINPUTCONTEXT_PAS_C}
{$define NSTEXTINPUTCONTEXT_PAS_C}

{ NSTextInputContext }
  NSTextInputContext = objcclass external (NSObject)
  private
    _client: id;
    _inputLocales: NSArray;
    _documentID: Pointer;
    _keyBindings: id;
    _keyBindingManager: id;
    _commandCharacters: NSString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(3)-1] of id;
    _auxiliary: id;
    __ticFlags: record
      case byte of
       0: (_anonbitfield_NSTextInputContext0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        _reserved: array[0..2] of id;
    _auxiliary: id;
    __ticFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTextInputContext0: cuint);
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
        _acceptsGlyphInfo: 0..1;
        
        _attributedString: 0..1;
        _fractionOfDistance: 0..1;
        _baselineDelta: 0..1;
        _windowLevel: 0..1;
        _drawsVertically: 0..1;
        
        _compatibilityMode: 0..1;
        _hasTS: 0..1;
        _hasLM: 0..1;
        
        _inputLocaleGeneration: 0..((1 shl 16)-1);
        _blockTSM: 0..1;
        _keyProcess: 0..1;
        _preflight: 0..1;
        _bindingFound: 0..1;
        
        __reserved: 0..((1 shl 3)-1);
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
    
  public
    class function currentInputContext: NSTextInputContext; message 'currentInputContext';
    function initWithClient(theClient: NSTextInputClientProtocol): id; message 'initWithClient:';
    function client: id; message 'client';
    procedure setAcceptsGlyphInfo (newValue: Boolean); message 'setAcceptsGlyphInfo:';
    function acceptsGlyphInfo: Boolean; message 'acceptsGlyphInfo';
    procedure setAllowedInputSourceLocales (newValue: NSArray); message 'setAllowedInputSourceLocales:';
    function allowedInputSourceLocales: NSArray; message 'allowedInputSourceLocales';
    procedure activate; message 'activate';
    procedure deactivate; message 'deactivate';
    function handleEvent(theEvent: NSEvent): Boolean; message 'handleEvent:';
    procedure discardMarkedText; message 'discardMarkedText';
    procedure invalidateCharacterCoordinates; message 'invalidateCharacterCoordinates';
    function keyboardInputSources: NSArray; message 'keyboardInputSources';
    procedure setSelectedKeyboardInputSource (newValue: NSString); message 'setSelectedKeyboardInputSource:';
    function selectedKeyboardInputSource: NSString; message 'selectedKeyboardInputSource';
    class function localizedNameForInputSource(inputSourceIdentifier: NSString): NSString; message 'localizedNameForInputSource:';
  end;

{$endif}
{$endif}
