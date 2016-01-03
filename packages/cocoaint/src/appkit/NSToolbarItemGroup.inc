{ Parsed from Appkit.framework NSToolbarItemGroup.h }

{$ifdef TYPES}
{$ifndef NSTOOLBARITEMGROUP_PAS_T}
{$define NSTOOLBARITEMGROUP_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTOOLBARITEMGROUP_PAS_R}
{$define NSTOOLBARITEMGROUP_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTOOLBARITEMGROUP_PAS_F}
{$define NSTOOLBARITEMGROUP_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTOOLBARITEMGROUP_PAS_S}
{$define NSTOOLBARITEMGROUP_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSToolbarItemGroup = objcclass;
  NSToolbarItemGroupPointer = ^NSToolbarItemGroup;
  NSToolbarItemGroupPtr = NSToolbarItemGroupPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTOOLBARITEMGROUP_PAS_C}
{$define NSTOOLBARITEMGROUP_PAS_C}

{ NSToolbarItemGroup }
  NSToolbarItemGroup = objcclass external (NSToolbarItem)
  private
    _groupItems: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __giFlags: record
      case byte of
       0: (_anonbitfield_NSToolbarItemGroup0: cuint);
       1: (data: bitpacked record
        dirtiedLayout: 0..1;
        reserved: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __giFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSToolbarItemGroup0: cuint);
       1: (
        dirtiedLayout: 0..1;
        reserved: 0..((1 shl 31)-1);
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
    _giReserved: id;
    
  public
    procedure setSubitems(subitems_: NSArray); message 'setSubitems:';
    function subitems: NSArray; message 'subitems';
  end;

{$endif}
{$endif}
