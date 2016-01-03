{ Parsed from Appkit.framework NSTableColumn.h }

{$ifdef TYPES}
{$ifndef NSTABLECOLUMN_PAS_T}
{$define NSTABLECOLUMN_PAS_T}

{ Constants }

const
  NSTableColumnNoResizing = 0;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSTABLECOLUMN_PAS_R}
{$define NSTABLECOLUMN_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSTABLECOLUMN_PAS_F}
{$define NSTABLECOLUMN_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSTABLECOLUMN_PAS_S}
{$define NSTABLECOLUMN_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSTableColumn = objcclass;
  NSTableColumnPointer = ^NSTableColumn;
  NSTableColumnPtr = NSTableColumnPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSTABLECOLUMN_PAS_C}
{$define NSTABLECOLUMN_PAS_C}

{ NSTableColumn }
  NSTableColumn = objcclass external (NSObject, NSCodingProtocol)
  private
    _identifier: id;
    _width: CGFloat;
    _minWidth: CGFloat;
    _maxWidth: CGFloat;
    _tableView: NSTableView;
    _headerCell: NSCell;
    _dataCell: NSCell;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __cFlags: record
      case byte of
       0: (_anonbitfield_NSTableColumn0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __cFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSTableColumn0: cuint);
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
        oldIsResizable: 0..1;
        _isEditable: 0..1;
        resizedPostingDisableCount: 0..((1 shl 8)-1);
        canUseReorderResizeImageCache: 0..1;
        userResizingAllowed: 0..1;
        autoResizingAllowed: 0..1;
        hidden: 0..1;
        RESERVED: 0..((1 shl 18)-1);
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
    _tcAuxiliaryStorage: id;
    
  public
    function initWithIdentifier(identifier_: id): id; message 'initWithIdentifier:';
    procedure setIdentifier(identifier_: id); message 'setIdentifier:';
    function identifier: id; message 'identifier';
    procedure setTableView(tableView_: NSTableView); message 'setTableView:';
    function tableView: NSTableView; message 'tableView';
    procedure setWidth(width_: CGFloat); message 'setWidth:';
    function width: CGFloat; message 'width';
    procedure setMinWidth(minWidth_: CGFloat); message 'setMinWidth:';
    function minWidth: CGFloat; message 'minWidth';
    procedure setMaxWidth(maxWidth_: CGFloat); message 'setMaxWidth:';
    function maxWidth: CGFloat; message 'maxWidth';
    procedure setHeaderCell(cell: NSCell); message 'setHeaderCell:';
    function headerCell: id; message 'headerCell';
    procedure setDataCell(cell: NSCell); message 'setDataCell:';
    function dataCell: id; message 'dataCell';
    function dataCellForRow(row: NSInteger): id; message 'dataCellForRow:';
    procedure setEditable(flag: Boolean); message 'setEditable:';
    function isEditable: Boolean; message 'isEditable';
    procedure sizeToFit; message 'sizeToFit';
    procedure setSortDescriptorPrototype(sortDescriptor: NSSortDescriptor); message 'setSortDescriptorPrototype:';
    function sortDescriptorPrototype: NSSortDescriptor; message 'sortDescriptorPrototype';
    procedure setResizingMask(resizingMask_: NSUInteger); message 'setResizingMask:';
    function resizingMask: NSUInteger; message 'resizingMask';
    procedure setHeaderToolTip(string_: NSString); message 'setHeaderToolTip:';
    function headerToolTip: NSString; message 'headerToolTip';
    function isHidden: Boolean; message 'isHidden';
    procedure setHidden(hidden: Boolean); message 'setHidden:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ NSDeprecated_NSTableColumnCategory }
  NSDeprecated_NSTableColumnCategory = objccategory external name 'NSDeprecated' (NSTableColumn)
    procedure setResizable(flag: Boolean); message 'setResizable:'; deprecated 'in Mac OS X 10.4 and later';
    function isResizable: Boolean; message 'isResizable'; deprecated 'in Mac OS X 10.4 and later';
  end;

{$endif}
{$endif}
