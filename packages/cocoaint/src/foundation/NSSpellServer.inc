{ Parsed from Foundation.framework NSSpellServer.h }

{$ifdef TYPES}
{$ifndef NSSPELLSERVER_PAS_T}
{$define NSSPELLSERVER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSPELLSERVER_PAS_R}
{$define NSSPELLSERVER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSPELLSERVER_PAS_F}
{$define NSSPELLSERVER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSPELLSERVER_PAS_S}
{$define NSSPELLSERVER_PAS_S}

{ External string constants }
var
  NSGrammarRange: NSString; cvar; external;
  NSGrammarUserDescription: NSString; cvar; external;
  NSGrammarCorrections: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSSpellServerDelegateProtocol = objcprotocol;
  NSSpellServer = objcclass;
  NSSpellServerPointer = ^NSSpellServer;
  NSSpellServerPtr = NSSpellServerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSPELLSERVER_PAS_C}
{$define NSSPELLSERVER_PAS_C}

{ NSSpellServer }
  NSSpellServer = objcclass external (NSObject)
  private
    _delegate: id;
    _caseSensitive: NSInteger;
    _spellServerConnection: id;
    _dictionaries: id;
    _learnedDictionaries: NSArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __ssFlags: record
      case byte of
       0: (_anonbitfield_NSSpellServer0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __ssFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSSpellServer0: cuint);
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
        delegateLearnsWords: 0..1;
        delegateForgetsWords: 0..1;
        busy: 0..1;
        _reserved: 0..((1 shl 29)-1);
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
    _reservedSpellServer1: Pointer;
    _reservedSpellServer2: Pointer;
    
  public
    procedure setDelegate(anObject: NSSpellServerDelegateProtocol); message 'setDelegate:';
    function delegate: NSSpellServerDelegateProtocol; message 'delegate';
    function registerLanguage_byVendor(language: NSString; vendor: NSString): Boolean; message 'registerLanguage:byVendor:';
    function isWordInUserDictionaries_caseSensitive(word: NSString; flag: Boolean): Boolean; message 'isWordInUserDictionaries:caseSensitive:';
    procedure run; message 'run';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSSPELLSERVER_PAS_P}
{$define NSSPELLSERVER_PAS_P}
  
{ NSSpellServerDelegate Protocol }
  NSSpellServerDelegateProtocol = objcprotocol external name 'NSSpellServerDelegate'
  optional
    function spellServer_findMisspelledWordInString_language_wordCount_countOnly(sender: NSSpellServer; stringToCheck: NSString; language: NSString; wordCount: NSIntegerPtr; countOnly: Boolean): NSRange; message 'spellServer:findMisspelledWordInString:language:wordCount:countOnly:';
    function spellServer_suggestGuessesForWord_inLanguage(sender: NSSpellServer; word: NSString; language: NSString): NSArray; message 'spellServer:suggestGuessesForWord:inLanguage:';
    procedure spellServer_didLearnWord_inLanguage(sender: NSSpellServer; word: NSString; language: NSString); message 'spellServer:didLearnWord:inLanguage:';
    procedure spellServer_didForgetWord_inLanguage(sender: NSSpellServer; word: NSString; language: NSString); message 'spellServer:didForgetWord:inLanguage:';
    function spellServer_suggestCompletionsForPartialWordRange_inString_language(sender: NSSpellServer; range: NSRange; string_: NSString; language: NSString): NSArray; message 'spellServer:suggestCompletionsForPartialWordRange:inString:language:';
    function spellServer_checkGrammarInString_language_details(sender: NSSpellServer; stringToCheck: NSString; language: NSString; details: NSArrayPointer): NSRange; message 'spellServer:checkGrammarInString:language:details:';
    function spellServer_checkString_offset_types_options_orthography_wordCount(sender: NSSpellServer; stringToCheck: NSString; offset: NSUInteger; checkingTypes: NSTextCheckingTypes; options: NSDictionary; orthography: NSOrthography; wordCount: NSIntegerPtr): NSArray; message 'spellServer:checkString:offset:types:options:orthography:wordCount:';
  end;
{$endif}
{$endif}
