{ Parsed from Appkit.framework NSATSTypesetter.h }

{$ifdef TYPES}
{$ifndef NSATSTYPESETTER_PAS_T}
{$define NSATSTYPESETTER_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSATSTYPESETTER_PAS_R}
{$define NSATSTYPESETTER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSATSTYPESETTER_PAS_F}
{$define NSATSTYPESETTER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSATSTYPESETTER_PAS_S}
{$define NSATSTYPESETTER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSATSTypesetter = objcclass;
  NSATSTypesetterPointer = ^NSATSTypesetter;
  NSATSTypesetterPtr = NSATSTypesetterPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSATSTYPESETTER_PAS_C}
{$define NSATSTYPESETTER_PAS_C}

{ NSATSTypesetter }
  NSATSTypesetter = objcclass external (NSTypesetter)
  private
    _attributedString: NSAttributedString;
    _paragraphGlyphRange: NSRange;
    _paragraphSeparatorGlyphRange: NSRange;
    _lineFragmentPadding: CGFloat;
    _layoutManager: NSLayoutManager;
    _textContainers: NSArray;
    _currentTextContainer: NSTextContainer;
    currentTextContainerIndex: NSUInteger;
    currentTextContainerSize: NSSize;
    _currentParagraphStyle: NSParagraphStyle;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _atsReserved: array[0..(8)-1] of Pointer;
=======
        _atsReserved: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _atsReserved: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _atsReserved: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _atsReserved: array[0..7] of Pointer;
>>>>>>> origin/cpstrnew
    _private: id;
    
  public
    class function sharedTypesetter: id; message 'sharedTypesetter';
  end;

{ NSPantherCompatibilityCategory }
  NSPantherCompatibilityCategory = objccategory external (NSATSTypesetter)
    function lineFragmentRectForProposedRect_remainingRect(proposedRect: NSRect; remainingRect: NSRectPointer): NSRect; message 'lineFragmentRectForProposedRect:remainingRect:'; deprecated 'in Mac OS X 10.4 and later';
  end;

{ NSPrimitiveInterfaceCategory }
  NSPrimitiveInterfaceCategory = objccategory external (NSATSTypesetter)
    function usesFontLeading: Boolean; message 'usesFontLeading';
    procedure setUsesFontLeading(flag: Boolean); message 'setUsesFontLeading:';
    function typesetterBehavior: NSTypesetterBehavior; message 'typesetterBehavior';
    procedure setTypesetterBehavior(behavior: NSTypesetterBehavior); message 'setTypesetterBehavior:';
    function hyphenationFactor: single; message 'hyphenationFactor';
    procedure setHyphenationFactor(factor: single); message 'setHyphenationFactor:';
    function lineFragmentPadding: CGFloat; message 'lineFragmentPadding';
    procedure setLineFragmentPadding(padding: CGFloat); message 'setLineFragmentPadding:';
    function substituteFontForFont(originalFont: NSFont): NSFont; message 'substituteFontForFont:';
    function textTabForGlyphLocation_writingDirection_maxLocation(glyphLocation: CGFloat; direction: NSWritingDirection; maxLocation: CGFloat): NSTextTab; message 'textTabForGlyphLocation:writingDirection:maxLocation:';
    function bidiProcessingEnabled: Boolean; message 'bidiProcessingEnabled';
    procedure setBidiProcessingEnabled(flag: Boolean); message 'setBidiProcessingEnabled:';
    procedure setAttributedString(attrString: NSAttributedString); message 'setAttributedString:';
    function attributedString: NSAttributedString; message 'attributedString';
    procedure setParagraphGlyphRange_separatorGlyphRange(paragraphRange: NSRange; paragraphSeparatorRange: NSRange); message 'setParagraphGlyphRange:separatorGlyphRange:';
    function paragraphGlyphRange: NSRange; message 'paragraphGlyphRange';
    function paragraphSeparatorGlyphRange: NSRange; message 'paragraphSeparatorGlyphRange';
    function layoutParagraphAtPoint(lineFragmentOrigin: NSPointPtr): NSUInteger; message 'layoutParagraphAtPoint:';
    function lineSpacingAfterGlyphAtIndex_withProposedLineFragmentRect(glyphIndex: NSUInteger; rect: NSRect): CGFloat; message 'lineSpacingAfterGlyphAtIndex:withProposedLineFragmentRect:';
    function paragraphSpacingBeforeGlyphAtIndex_withProposedLineFragmentRect(glyphIndex: NSUInteger; rect: NSRect): CGFloat; message 'paragraphSpacingBeforeGlyphAtIndex:withProposedLineFragmentRect:';
    function paragraphSpacingAfterGlyphAtIndex_withProposedLineFragmentRect(glyphIndex: NSUInteger; rect: NSRect): CGFloat; message 'paragraphSpacingAfterGlyphAtIndex:withProposedLineFragmentRect:';
    function layoutManager: NSLayoutManager; message 'layoutManager';
    function currentTextContainer: NSTextContainer; message 'currentTextContainer';
    procedure setHardInvalidation_forGlyphRange(flag: Boolean; glyphRange: NSRange); message 'setHardInvalidation:forGlyphRange:';
    procedure getLineFragmentRect_usedRect_forParagraphSeparatorGlyphRange_atProposedOrigin(lineFragmentRect: NSRectPtr; lineFragmentUsedRect: NSRectPtr; paragraphSeparatorGlyphRange_: NSRange; lineOrigin: NSPoint); message 'getLineFragmentRect:usedRect:forParagraphSeparatorGlyphRange:atProposedOrigin:';
  end;

{ NSLayoutPhaseInterface_NSATSTypesetterCategory }
  NSLayoutPhaseInterface_NSATSTypesetterCategory = objccategory external name 'NSLayoutPhaseInterface' (NSATSTypesetter)
    procedure willSetLineFragmentRect_forGlyphRange_usedRect_baselineOffset(lineRect: NSRectPtr; glyphRange: NSRange; usedRect: NSRectPtr; baselineOffset: CGFloatPtr); message 'willSetLineFragmentRect:forGlyphRange:usedRect:baselineOffset:';
    function shouldBreakLineByWordBeforeCharacterAtIndex(charIndex: NSUInteger): Boolean; message 'shouldBreakLineByWordBeforeCharacterAtIndex:';
    function shouldBreakLineByHyphenatingBeforeCharacterAtIndex(charIndex: NSUInteger): Boolean; message 'shouldBreakLineByHyphenatingBeforeCharacterAtIndex:';
    function hyphenationFactorForGlyphAtIndex(glyphIndex: NSUInteger): single; message 'hyphenationFactorForGlyphAtIndex:';
    function hyphenCharacterForGlyphAtIndex(glyphIndex: NSUInteger): UTF32Char; message 'hyphenCharacterForGlyphAtIndex:';
    function boundingBoxForControlGlyphAtIndex_forTextContainer_proposedLineFragment_glyphPosition_characterIndex(glyphIndex: NSUInteger; textContainer: NSTextContainer; proposedRect: NSRect; glyphPosition: NSPoint; charIndex: NSUInteger): NSRect; message 'boundingBoxForControlGlyphAtIndex:forTextContainer:proposedLineFragment:glyphPosition:characterIndex:';
  end;

{ NSGlyphStorageInterface_NSATSTypesetterCategory }
  NSGlyphStorageInterface_NSATSTypesetterCategory = objccategory external name 'NSGlyphStorageInterface' (NSATSTypesetter)
    function characterRangeForGlyphRange_actualGlyphRange(glyphRange: NSRange; actualGlyphRange: NSRangePointer): NSRange; message 'characterRangeForGlyphRange:actualGlyphRange:';
    function glyphRangeForCharacterRange_actualCharacterRange(charRange: NSRange; actualCharRange: NSRangePointer): NSRange; message 'glyphRangeForCharacterRange:actualCharacterRange:';
    function getGlyphsInRange_glyphs_characterIndexes_glyphInscriptions_elasticBits(glyphsRange: NSRange; glyphBuffer: NSGlyphPtr; charIndexBuffer: NSUIntegerPtr; inscribeBuffer: NSGlyphInscriptionPtr; elasticBuffer: pboolean): NSUInteger; message 'getGlyphsInRange:glyphs:characterIndexes:glyphInscriptions:elasticBits:';
    procedure setLineFragmentRect_forGlyphRange_usedRect_baselineOffset(fragmentRect: NSRect; glyphRange: NSRange; usedRect: NSRect; baselineOffset: CGFloat); message 'setLineFragmentRect:forGlyphRange:usedRect:baselineOffset:';
    procedure substituteGlyphsInRange_withGlyphs(glyphRange: NSRange; glyphs: NSGlyphPtr); message 'substituteGlyphsInRange:withGlyphs:';
    procedure insertGlyph_atGlyphIndex_characterIndex(glyph: NSGlyph; glyphIndex: NSUInteger; characterIndex: NSUInteger); message 'insertGlyph:atGlyphIndex:characterIndex:';
    procedure deleteGlyphsInRange(glyphRange: NSRange); message 'deleteGlyphsInRange:';
    procedure setNotShownAttribute_forGlyphRange(flag: Boolean; glyphRange: NSRange); message 'setNotShownAttribute:forGlyphRange:';
    procedure setDrawsOutsideLineFragment_forGlyphRange(flag: Boolean; glyphRange: NSRange); message 'setDrawsOutsideLineFragment:forGlyphRange:';
    procedure setLocation_withAdvancements_forStartOfGlyphRange(location: NSPoint; advancements: CGFloatPtr; glyphRange: NSRange); message 'setLocation:withAdvancements:forStartOfGlyphRange:';
    procedure setAttachmentSize_forGlyphRange(attachmentSize: NSSize; glyphRange: NSRange); message 'setAttachmentSize:forGlyphRange:';
    procedure setBidiLevels_forGlyphRange(levels: pbyte; glyphRange: NSRange); message 'setBidiLevels:forGlyphRange:';
  end;

{$endif}
{$endif}
