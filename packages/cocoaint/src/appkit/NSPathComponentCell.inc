{ Parsed from Appkit.framework NSPathComponentCell.h }

{$ifdef TYPES}
{$ifndef NSPATHCOMPONENTCELL_PAS_T}
{$define NSPATHCOMPONENTCELL_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPATHCOMPONENTCELL_PAS_R}
{$define NSPATHCOMPONENTCELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPATHCOMPONENTCELL_PAS_F}
{$define NSPATHCOMPONENTCELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPATHCOMPONENTCELL_PAS_S}
{$define NSPATHCOMPONENTCELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPathComponentCell = objcclass;
  NSPathComponentCellPointer = ^NSPathComponentCell;
  NSPathComponentCellPtr = NSPathComponentCellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPATHCOMPONENTCELL_PAS_C}
{$define NSPATHCOMPONENTCELL_PAS_C}

{ NSPathComponentCell }
  NSPathComponentCell = objcclass external (NSTextFieldCell)
  private
    _image: NSImage;
    _fullWidth: CGFloat;
    _resizedWidth: CGFloat;
    _currentWidth: CGFloat;
    _url: NSURL;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSPathComponentCell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSPathComponentCell0: cuint);
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
        shouldDrawArrow: 0..1;
        drawsAsNavigationBar: 0..1;
        isFirstItem: 0..1;
        isLastItem: 0..1;
        isDropTarget: 0..1;
        reserved: 0..((1 shl 27)-1);
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
    _aux: id;
    
  public
    function image: NSImage; message 'image';
    procedure setImage(value: NSImage); message 'setImage:';
    function URL: NSURL; message 'URL';
    procedure setURL(url_: NSURL); message 'setURL:';
  end;

{$endif}
{$endif}
