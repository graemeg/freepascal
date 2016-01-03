{ Parsed from Appkit.framework NSFontPanel.h }

{$ifdef TYPES}
{$ifndef NSFONTPANEL_PAS_T}
{$define NSFONTPANEL_PAS_T}

{ Constants }

const
  NSFPPreviewButton = 131;
  NSFPRevertButton = 130;
  NSFPSetButton = 132;
  NSFPPreviewField = 128;
  NSFPSizeField = 129;
  NSFPSizeTitle = 133;
  NSFPCurrentField = 134;

const
  NSFontPanelFaceModeMask = 1 shl 0;
  NSFontPanelSizeModeMask = 1 shl 1;
  NSFontPanelCollectionModeMask = 1 shl 2;
  NSFontPanelUnderlineEffectModeMask = 1 shl 8;
  NSFontPanelStrikethroughEffectModeMask = 1 shl 9;
  NSFontPanelTextColorEffectModeMask = 1 shl 10;
  NSFontPanelDocumentColorEffectModeMask = 1 shl 11;
  NSFontPanelShadowEffectModeMask = 1 shl 12;
  NSFontPanelAllEffectsModeMask = $FFF00;
  NSFontPanelStandardModesMask = $FFFF;
  NSFontPanelAllModesMask = $FFFFFFFF;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSFONTPANEL_PAS_R}
{$define NSFONTPANEL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSFONTPANEL_PAS_F}
{$define NSFONTPANEL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSFONTPANEL_PAS_S}
{$define NSFONTPANEL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSFontPanel = objcclass;
  NSFontPanelPointer = ^NSFontPanel;
  NSFontPanelPtr = NSFontPanelPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSFONTPANEL_PAS_C}
{$define NSFONTPANEL_PAS_C}

{ NSFontPanel }
  NSFontPanel = objcclass external (NSPanel)
  private
    _manager: NSFontManager;
    _collectionNames: NSArray;
    _selection: id;
    _carbonNotification: Pointer;
    _targetObject: id;
    _familyList: id;
    _faceList: id;
    _sizeList: id;
    _mainCollectionList: id;
    _sizeField: id;
    _sizeSlider: id;
    _sizeSliderBox: id;
    _preview: id;
    _previewCaption: id;
    _mainSplitView: id;
    _mmCollectionList: id;
    _mmFamilyList: id;
    _mmFaceList: id;
    _mmSizeList: id;
    _extrasPopup: id;
    _searchField: id;
    _fixedListButton: id;
    _sliderButton: id;
    _accessoryView: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __fpFlags: record
      case byte of
       0: (_anonbitfield_NSFontPanel0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __fpFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSFontPanel0: cuint);
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
        setFontChange: 0..1;
        setFontAttributeChange: 0..1;
        _delRespFamily: 0..1;
        _delRespFace: 0..1;
        _delRespSize: 0..1;
        _delRespColl: 0..1;
        _collectionDisabled: 0..1;
        _sizeDisabled: 0..1;
        _faceDisabled: 0..1;
        showEffects: 0..1;
        _uiMode: 0..((1 shl 8)-1);
        _reserved: 0..((1 shl 14)-1);
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
    _regularModeBox: id;
    _miniModeBox: id;
    _modeBoxSuperview: id;
    _collectionLabel: id;
    _sizeLabel: id;
    _faceLabel: id;
    _familyLabel: id;
    _sizeStyleButton: id;
    _newSizeField: id;
    _editSizeList: id;
    _editSizeListBox: id;
    _editSizeSliderBox: id;
    _editSizeSliderMaxField: id;
    _editSizeSliderMinField: id;
    _sizeEditWindow: id;
    _availableSizes: id;
    _addCollectionButton: id;
    _removeCollectionButton: id;
    _fontPanelPreviewHeight: CGFloat;
    _typographyPanel: id;
    _actionButton: id;
    _fontEffectsBox: id;
    _sizeStyle: cint;
    {$ifndef cpu64}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _fpUnused: array[0..(72)-1] of id;
=======
        _fpUnused: array[0..71] of id;
>>>>>>> graemeg/cpstrnew
=======
        _fpUnused: array[0..71] of id;
>>>>>>> graemeg/cpstrnew
=======
        _fpUnused: array[0..71] of id;
>>>>>>> graemeg/cpstrnew
=======
        _fpUnused: array[0..71] of id;
>>>>>>> origin/cpstrnew
    {$endif}
    
  public
    class function sharedFontPanel: NSFontPanel; message 'sharedFontPanel';
    class function sharedFontPanelExists: Boolean; message 'sharedFontPanelExists';
    function accessoryView: NSView; message 'accessoryView';
    procedure setAccessoryView(aView: NSView); message 'setAccessoryView:';
    procedure setPanelFont_isMultiple(fontObj: NSFont; flag: Boolean); message 'setPanelFont:isMultiple:';
    function panelConvertFont(fontObj: NSFont): NSFont; message 'panelConvertFont:';
    function worksWhenModal: Boolean; message 'worksWhenModal';
    function isEnabled: Boolean; message 'isEnabled';
    procedure setEnabled(flag: Boolean); message 'setEnabled:';
    procedure reloadDefaultFontFamilies; message 'reloadDefaultFontFamilies';
  end;

{ NSFontPanelValidationAdditionsCategory }
  NSFontPanelValidationAdditionsCategory = objccategory external (NSObject)
    function validModesForFontPanel(fontPanel: NSFontPanel): NSUInteger; message 'validModesForFontPanel:';
  end;

{$endif}
{$endif}
