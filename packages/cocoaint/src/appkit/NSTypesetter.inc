{ Parsed from Appkit.framework NSTypesetter.h }

{$ifdef TYPES}
{$ifndef NSTYPESETTER_PAS_T}
{$define NSTYPESETTER_PAS_T}

{ Constants }

const
  NSTypesetterZeroAdvancementAction = 1 shl 0;
  NSTypesetterWhitespaceAction = 1 shl 1;
  NSTypesetterHorizontalTabAction = 1 shl 2;
  NSTypesetterLineBreakAction = 1 shl 3;
  NSTypesetterParagraphBreakAction = 1 shl 4;
  NSTypesetterContainerBreakAction = 1 shl 5;

{ Types }
type
  NSTypesetterControlCharacterAction = NSUInteger;
  NSTypesetterControlCharacterActionPtr = ^NSTypesetterControlCharacterAction;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTYPESETTER_PAS_R}
{$define NSTYPESETTER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTYPESETTER_PAS_F}
{$define NSTYPESETTER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTYPESETTER_PAS_S}
{$define NSTYPESETTER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTypesetter = objcclass;
  NSTypesetterPointer = ^NSTypesetter;
  NSTypesetterPtr = NSTypesetterPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTYPESETTER_PAS_C}
{$define NSTYPESETTER_PAS_C}

{ NSTypesetter }
  NSTypesetter = objcclass external (NSObject)
  private
    {$ifdef cpu64}
    _reserved: Pointer;
    {$else}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(2)-1] of cuint;
=======
        _reserved: array[0..1] of cuint;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of cuint;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of cuint;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..1] of cuint;
>>>>>>> origin/cpstrnew
    {$endif}
    
  public
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
    function paragraphCharacterRange: NSRange; message 'paragraphCharacterRange';
    function paragraphSeparatorCharacterRange: NSRange; message 'paragraphSeparatorCharacterRange';
    function layoutParagraphAtPoint(lineFragmentOrigin: NSPointPointer): NSUInteger; message 'layoutParagraphAtPoint:';
    procedure beginParagraph; message 'beginParagraph';
    procedure endParagraph; message 'endParagraph';
    procedure beginLineWithGlyphAtIndex(glyphIndex: NSUInteger); message 'beginLineWithGlyphAtIndex:';
    procedure endLineWithGlyphRange(lineGlyphRange: NSRange); message 'endLineWithGlyphRange:';
    function lineSpacingAfterGlyphAtIndex_withProposedLineFragmentRect(glyphIndex: NSUInteger; rect: NSRect): CGFloat; message 'lineSpacingAfterGlyphAtIndex:withProposedLineFragmentRect:';
    function paragraphSpacingBeforeGlyphAtIndex_withProposedLineFragmentRect(glyphIndex: NSUInteger; rect: NSRect): CGFloat; message 'paragraphSpacingBeforeGlyphAtIndex:withProposedLineFragmentRect:';
    function paragraphSpacingAfterGlyphAtIndex_withProposedLineFragmentRect(glyphIndex: NSUInteger; rect: NSRect): CGFloat; message 'paragraphSpacingAfterGlyphAtIndex:withProposedLineFragmentRect:';
    procedure getLineFragmentRect_usedRect_forParagraphSeparatorGlyphRange_atProposedOrigin(lineFragmentRect: NSRectPointer; lineFragmentUsedRect: NSRectPointer; paragraphSeparatorGlyphRange_: NSRange; lineOrigin: NSPoint); message 'getLineFragmentRect:usedRect:forParagraphSeparatorGlyphRange:atProposedOrigin:';
    function attributesForExtraLineFragment: NSDictionary; message 'attributesForExtraLineFragment';
    function actionForControlCharacterAtIndex(charIndex: NSUInteger): NSTypesetterControlCharacterAction; message 'actionForControlCharacterAtIndex:';
    function layoutManager: NSLayoutManager; message 'layoutManager';
    function textContainers: NSArray; message 'textContainers';
    function currentTextContainer: NSTextContainer; message 'currentTextContainer';
    function currentParagraphStyle: NSParagraphStyle; message 'currentParagraphStyle';
    procedure setHardInvalidation_forGlyphRange(flag: Boolean; glyphRange: NSRange); message 'setHardInvalidation:forGlyphRange:';
    procedure layoutGlyphsInLayoutManager_startingAtGlyphIndex_maxNumberOfLineFragments_nextGlyphIndex(layoutManager_: NSLayoutManager; startGlyphIndex: NSUInteger; maxNumLines: NSUInteger; nextGlyph: NSUIntegerPtr); message 'layoutGlyphsInLayoutManager:startingAtGlyphIndex:maxNumberOfLineFragments:nextGlyphIndex:';
    function layoutCharactersInRange_forLayoutManager_maximumNumberOfLineFragments(characterRange: NSRange; layoutManager_: NSLayoutManager; maxNumLines: NSUInteger): NSRange; message 'layoutCharactersInRange:forLayoutManager:maximumNumberOfLineFragments:';
    class function printingAdjustmentInLayoutManager_forNominallySpacedGlyphRange_packedGlyphs_count(layoutMgr: NSLayoutManager; nominallySpacedGlyphsRange: NSRange; packedGlyphs: PByte; packedGlyphsCount: NSUInteger): NSSize; message 'printingAdjustmentInLayoutManager:forNominallySpacedGlyphRange:packedGlyphs:count:';
    function baselineOffsetInLayoutManager_glyphIndex(layoutMgr: NSLayoutManager; glyphIndex: NSUInteger): CGFloat; message 'baselineOffsetInLayoutManager:glyphIndex:';
    class function sharedSystemTypesetter: id; message 'sharedSystemTypesetter';
    class function sharedSystemTypesetterForBehavior(theBehavior: NSTypesetterBehavior): id; message 'sharedSystemTypesetterForBehavior:';
    class function defaultTypesetterBehavior: NSTypesetterBehavior; message 'defaultTypesetterBehavior';
  end;

{ NSLayoutPhaseInterfaceCategory }
  NSLayoutPhaseInterfaceCategory = objccategory external (NSTypesetter)
    procedure willSetLineFragmentRect_forGlyphRange_usedRect_baselineOffset(lineRect: NSRectPointer; glyphRange: NSRange; usedRect: NSRectPointer; baselineOffset: CGFloatPtr); message 'willSetLineFragmentRect:forGlyphRange:usedRect:baselineOffset:';
    function shouldBreakLineByWordBeforeCharacterAtIndex(charIndex: NSUInteger): Boolean; message 'shouldBreakLineByWordBeforeCharacterAtIndex:';
    function shouldBreakLineByHyphenatingBeforeCharacterAtIndex(charIndex: NSUInteger): Boolean; message 'shouldBreakLineByHyphenatingBeforeCharacterAtIndex:';
    function hyphenationFactorForGlyphAtIndex(glyphIndex: NSUInteger): single; message 'hyphenationFactorForGlyphAtIndex:';
    function hyphenCharacterForGlyphAtIndex(glyphIndex: NSUInteger): UTF32Char; message 'hyphenCharacterForGlyphAtIndex:';
    function boundingBoxForControlGlyphAtIndex_forTextContainer_proposedLineFragment_glyphPosition_characterIndex(glyphIndex: NSUInteger; textContainer: NSTextContainer; proposedRect: NSRect; glyphPosition: NSPoint; charIndex: NSUInteger): NSRect; message 'boundingBoxForControlGlyphAtIndex:forTextContainer:proposedLineFragment:glyphPosition:characterIndex:';
  end;

{ NSGlyphStorageInterfaceCategory }
  NSGlyphStorageInterfaceCategory = objccategory external (NSTypesetter)
    function characterRangeForGlyphRange_actualGlyphRange(glyphRange: NSRange; actualGlyphRange: NSRangePointer): NSRange; message 'characterRangeForGlyphRange:actualGlyphRange:';
    function glyphRangeForCharacterRange_actualCharacterRange(charRange: NSRange; actualCharRange: NSRangePointer): NSRange; message 'glyphRangeForCharacterRange:actualCharacterRange:';
    function getGlyphsInRange_glyphs_characterIndexes_glyphInscriptions_elasticBits_bidiLevels(glyphsRange: NSRange; glyphBuffer: NSGlyphPtr; charIndexBuffer: NSUIntegerPtr; inscribeBuffer: NSGlyphInscriptionPtr; elasticBuffer: pboolean; bidiLevelBuffer: PByte): NSUInteger; message 'getGlyphsInRange:glyphs:characterIndexes:glyphInscriptions:elasticBits:bidiLevels:';
    procedure getLineFragmentRect_usedRect_remainingRect_forStartingGlyphAtIndex_proposedRect_lineSpacing_paragraphSpacingBefore_paragraphSpacingAfter(lineFragmentRect: NSRectPointer; lineFragmentUsedRect: NSRectPointer; remainingRect: NSRectPointer; startingGlyphIndex: NSUInteger; proposedRect: NSRect; lineSpacing: CGFloat; paragraphSpacingBefore: CGFloat; paragraphSpacingAfter: CGFloat); message 'getLineFragmentRect:usedRect:remainingRect:forStartingGlyphAtIndex:proposedRect:lineSpacing:paragraphSpacingBefore:paragraphSpacingAfter:';
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
