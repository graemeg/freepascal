{ Parsed from Appkit.framework NSSpellChecker.h }

{$ifdef TYPES}
{$ifndef NSSPELLCHECKER_PAS_T}
{$define NSSPELLCHECKER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSPELLCHECKER_PAS_R}
{$define NSSPELLCHECKER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSPELLCHECKER_PAS_F}
{$define NSSPELLCHECKER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSPELLCHECKER_PAS_S}
{$define NSSPELLCHECKER_PAS_S}

{ External string constants }
var
  NSTextCheckingOrthographyKey: NSString; cvar; external;
  NSTextCheckingQuotesKey: NSString; cvar; external;
  NSTextCheckingReplacementsKey: NSString; cvar; external;
  NSTextCheckingReferenceDateKey: NSString; cvar; external;
  NSTextCheckingReferenceTimeZoneKey: NSString; cvar; external;
  NSTextCheckingDocumentURLKey: NSString; cvar; external;
  NSTextCheckingDocumentTitleKey: NSString; cvar; external;
  NSTextCheckingDocumentAuthorKey: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  NSSpellChecker = objcclass;
  NSSpellCheckerPointer = ^NSSpellChecker;
  NSSpellCheckerPtr = NSSpellCheckerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSPELLCHECKER_PAS_C}
{$define NSSPELLCHECKER_PAS_C}

{ NSSpellChecker }
  NSSpellChecker = objcclass external (NSObject)
  private
    _guessesBrowser: id;
    _wordField: id;
    _languagePopUp: id;
    _reserved1: id;
    _panel: id;
    _reserved2: id;
    _correctButton: id;
    _guessButton: id;
    _ignoreButton: id;
    _reserved3: id;
    _languagesBrowser: id;
    _quotesBrowser: id;
    _replacementsBrowser: id;
    _defineButton: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __scFlags: record
      case byte of
       0: (_anonbitfield_NSSpellChecker0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __scFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSSpellChecker0: cuint);
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
        autoShowGuesses: 0..1;
        needDelayedGuess: 0..1;
        unignoreInProgress: 0..1;
        wordFieldEdited: 0..1;
        inSpelling: 0..1;
        reconnectSpelling: 0..1;
        inGrammar: 0..1;
        reconnectGrammar: 0..1;
        languageIdentification: 0..1;
        languagesHidden: 0..1;
        quotesByLanguage: 0..1;
        _reserved: 0..((1 shl 21)-1);
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
    _substitutionsPanel: id;
    _reserved4: id;
    _learnButton: id;
    _infoField: id;
    _grammarControl: id;
    
  public
    class function sharedSpellChecker: NSSpellChecker; message 'sharedSpellChecker';
    class function sharedSpellCheckerExists: Boolean; message 'sharedSpellCheckerExists';
    class function uniqueSpellDocumentTag: NSInteger; message 'uniqueSpellDocumentTag';
    function checkSpellingOfString_startingAt_language_wrap_inSpellDocumentWithTag_wordCount(stringToCheck: NSString; startingOffset: NSInteger; language_: NSString; wrapFlag: Boolean; tag: NSInteger; wordCount: NSIntegerPtr): NSRange; message 'checkSpellingOfString:startingAt:language:wrap:inSpellDocumentWithTag:wordCount:';
    function checkSpellingOfString_startingAt(stringToCheck: NSString; startingOffset: NSInteger): NSRange; message 'checkSpellingOfString:startingAt:';
    function countWordsInString_language(stringToCount: NSString; language_: NSString): NSInteger; message 'countWordsInString:language:';
    function checkGrammarOfString_startingAt_language_wrap_inSpellDocumentWithTag_details(stringToCheck: NSString; startingOffset: NSInteger; language_: NSString; wrapFlag: Boolean; tag: NSInteger; details: NSArrayPointer): NSRange; message 'checkGrammarOfString:startingAt:language:wrap:inSpellDocumentWithTag:details:';
    function checkString_range_types_options_inSpellDocumentWithTag_orthography_wordCount(stringToCheck: NSString; range: NSRange; checkingTypes: NSTextCheckingTypes; options: NSDictionary; tag: NSInteger; orthography: NSOrthographyPointer; wordCount: NSIntegerPtr): NSArray; message 'checkString:range:types:options:inSpellDocumentWithTag:orthography:wordCount:';
    function menuForResult_string_options_atLocation_inView(result_: NSTextCheckingResult; checkedString: NSString; options: NSDictionary; location: NSPoint; view: NSView): NSMenu; message 'menuForResult:string:options:atLocation:inView:';
    function userQuotesArrayForLanguage(language_: NSString): NSArray; message 'userQuotesArrayForLanguage:';
    function userReplacementsDictionary: NSDictionary; message 'userReplacementsDictionary';
    procedure updateSpellingPanelWithMisspelledWord(word: NSString); message 'updateSpellingPanelWithMisspelledWord:';
    procedure updateSpellingPanelWithGrammarString_detail(string_: NSString; detail: NSDictionary); message 'updateSpellingPanelWithGrammarString:detail:';
    function spellingPanel: NSPanel; message 'spellingPanel';
    function accessoryView: NSView; message 'accessoryView';
    procedure setAccessoryView(aView: NSView); message 'setAccessoryView:';
    function substitutionsPanel: NSPanel; message 'substitutionsPanel';
    function substitutionsPanelAccessoryViewController: NSViewController; message 'substitutionsPanelAccessoryViewController';
    procedure setSubstitutionsPanelAccessoryViewController(accessoryController: NSViewController); message 'setSubstitutionsPanelAccessoryViewController:';
    procedure updatePanels; message 'updatePanels';
    procedure ignoreWord_inSpellDocumentWithTag(wordToIgnore: NSString; tag: NSInteger); message 'ignoreWord:inSpellDocumentWithTag:';
    function ignoredWordsInSpellDocumentWithTag(tag: NSInteger): NSArray; message 'ignoredWordsInSpellDocumentWithTag:';
    procedure setIgnoredWords_inSpellDocumentWithTag(words: NSArray; tag: NSInteger); message 'setIgnoredWords:inSpellDocumentWithTag:';
    function guessesForWordRange_inString_language_inSpellDocumentWithTag(range: NSRange; string_: NSString; language_: NSString; tag: NSInteger): NSArray; message 'guessesForWordRange:inString:language:inSpellDocumentWithTag:';
    function completionsForPartialWordRange_inString_language_inSpellDocumentWithTag(range: NSRange; string_: NSString; language_: NSString; tag: NSInteger): NSArray; message 'completionsForPartialWordRange:inString:language:inSpellDocumentWithTag:';
    procedure closeSpellDocumentWithTag(tag: NSInteger); message 'closeSpellDocumentWithTag:';
    function availableLanguages: NSArray; message 'availableLanguages';
    function userPreferredLanguages: NSArray; message 'userPreferredLanguages';
    function automaticallyIdentifiesLanguages: Boolean; message 'automaticallyIdentifiesLanguages';
    procedure setAutomaticallyIdentifiesLanguages(flag: Boolean); message 'setAutomaticallyIdentifiesLanguages:';
    function language: NSString; message 'language';
    function setLanguage(language_: NSString): Boolean; message 'setLanguage:';
    procedure setWordFieldStringValue(aString: NSString); message 'setWordFieldStringValue:';
    procedure learnWord(word: NSString); message 'learnWord:';
    function hasLearnedWord(word: NSString): Boolean; message 'hasLearnedWord:';
    procedure unlearnWord(word: NSString); message 'unlearnWord:';
  end;

{ NSDeprecated_NSSpellCheckerCategory }
  NSDeprecated_NSSpellCheckerCategory = objccategory external name 'NSDeprecated' (NSSpellChecker)
    function guessesForWord(word: NSString): NSArray; message 'guessesForWord:';
    procedure forgetWord(word: NSString); message 'forgetWord:'; deprecated 'in Mac OS X 10.5 and later';
  end;

{$endif}
{$endif}
