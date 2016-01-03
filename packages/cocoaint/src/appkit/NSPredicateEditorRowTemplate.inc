{ Parsed from Appkit.framework NSPredicateEditorRowTemplate.h }

{$ifdef TYPES}
{$ifndef NSPREDICATEEDITORROWTEMPLATE_PAS_T}
{$define NSPREDICATEEDITORROWTEMPLATE_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSPREDICATEEDITORROWTEMPLATE_PAS_R}
{$define NSPREDICATEEDITORROWTEMPLATE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSPREDICATEEDITORROWTEMPLATE_PAS_F}
{$define NSPREDICATEEDITORROWTEMPLATE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSPREDICATEEDITORROWTEMPLATE_PAS_S}
{$define NSPREDICATEEDITORROWTEMPLATE_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSPredicateEditorRowTemplate = objcclass;
  NSPredicateEditorRowTemplatePointer = ^NSPredicateEditorRowTemplate;
  NSPredicateEditorRowTemplatePtr = NSPredicateEditorRowTemplatePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSPREDICATEEDITORROWTEMPLATE_PAS_C}
{$define NSPREDICATEEDITORROWTEMPLATE_PAS_C}

{ NSPredicateEditorRowTemplate }
  NSPredicateEditorRowTemplate = objcclass external (NSObject, NSCodingProtocol, NSCopyingProtocol)
  private
    _templateType: NSInteger;
    _predicateOptions: NSUInteger;
    _predicateModifier: NSUInteger;
    _leftAttributeType: NSUInteger;
    _rightAttributeType: NSUInteger;
    _views: NSArray;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __ptFlags: record
      case byte of
       0: (_anonbitfield_NSPredicateEditorRowTemplate0: cuint);
       1: (data: bitpacked record
        leftIsWildcard: 0..1;
        rightIsWildcard: 0..1;
        reserved: 0..((1 shl 30)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __ptFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSPredicateEditorRowTemplate0: cuint);
       1: (
        leftIsWildcard: 0..1;
        rightIsWildcard: 0..1;
        reserved: 0..((1 shl 30)-1);
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
    _patternReserved: id;
    
  public
    function matchForPredicate(predicate: NSPredicate): double; message 'matchForPredicate:';
    function templateViews: NSArray; message 'templateViews';
    procedure setPredicate(predicate: NSPredicate); message 'setPredicate:';
    function predicateWithSubpredicates(subpredicates: NSArray): NSPredicate; message 'predicateWithSubpredicates:';
    function displayableSubpredicatesOfPredicate(predicate: NSPredicate): NSArray; message 'displayableSubpredicatesOfPredicate:';
    function initWithLeftExpressions_rightExpressions_modifier_operators_options(leftExpressions_: NSArray; rightExpressions_: NSArray; modifier_: NSComparisonPredicateModifier; operators_: NSArray; options_: NSUInteger): id; message 'initWithLeftExpressions:rightExpressions:modifier:operators:options:';
    function initWithLeftExpressions_rightExpressionAttributeType_modifier_operators_options(leftExpressions_: NSArray; attributeType: NSAttributeType; modifier_: NSComparisonPredicateModifier; operators_: NSArray; options_: NSUInteger): id; message 'initWithLeftExpressions:rightExpressionAttributeType:modifier:operators:options:';
    function initWithCompoundTypes(compoundTypes_: NSArray): id; message 'initWithCompoundTypes:';
    function leftExpressions: NSArray; message 'leftExpressions';
    function rightExpressions: NSArray; message 'rightExpressions';
    function rightExpressionAttributeType: NSAttributeType; message 'rightExpressionAttributeType';
    function modifier: NSComparisonPredicateModifier; message 'modifier';
    function operators: NSArray; message 'operators';
    function options: NSUInteger; message 'options';
    function compoundTypes: NSArray; message 'compoundTypes';
    class function templatesWithAttributeKeyPaths_inEntityDescription(keyPaths: NSArray; entityDescription: NSEntityDescription): NSArray; message 'templatesWithAttributeKeyPaths:inEntityDescription:';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
    function copyWithZone(zone_: NSZonePtr): id;
  end;

{$endif}
{$endif}
