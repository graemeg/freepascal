{
     File:       HIToolbox/HITextViews.h
 
     Contains:   Definitions of text-display and text-editing views provided by HIToolbox.
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     Version:    HIToolbox-624~3
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> graemeg/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/cpstrnew
=======
     Version:    HIToolbox-437~1
>>>>>>> origin/fixes_2.4
 
     Copyright:  © 2006-2008 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                     http://bugs.freepascal.org
 
}
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
{       Pascal Translation Updated:  Jonas Maebe, <jonas@freepascal.org>, October 2012 }
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
                     http://www.freepascal.org/bugs.html
 
}
{       Initial Pascal Translation:  Jonas Maebe, <jonas@freepascal.org>, October 2009 }
<<<<<<< HEAD
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
=======
>>>>>>> origin/fixes_2.4
{
    Modified for use with Free Pascal
    Version 308
    Please report any bugs to <gpc@microbizz.nl>
}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}
{$mode macpas}
{$packenum 1}
{$macro on}
{$inline on}
{$calling mwpascal}

unit HITextViews;
interface
{$setc UNIVERSAL_INTERFACES_VERSION := $0400}
{$setc GAP_INTERFACES_VERSION := $0308}

{$ifc not defined USE_CFSTR_CONSTANT_MACROS}
    {$setc USE_CFSTR_CONSTANT_MACROS := TRUE}
{$endc}

{$ifc defined CPUPOWERPC and defined CPUI386}
	{$error Conflicting initial definitions for CPUPOWERPC and CPUI386}
{$endc}
{$ifc defined FPC_BIG_ENDIAN and defined FPC_LITTLE_ENDIAN}
	{$error Conflicting initial definitions for FPC_BIG_ENDIAN and FPC_LITTLE_ENDIAN}
{$endc}

{$ifc not defined __ppc__ and defined CPUPOWERPC32}
	{$setc __ppc__ := 1}
{$elsec}
	{$setc __ppc__ := 0}
{$endc}
{$ifc not defined __ppc64__ and defined CPUPOWERPC64}
	{$setc __ppc64__ := 1}
{$elsec}
	{$setc __ppc64__ := 0}
{$endc}
{$ifc not defined __i386__ and defined CPUI386}
	{$setc __i386__ := 1}
{$elsec}
	{$setc __i386__ := 0}
{$endc}
{$ifc not defined __x86_64__ and defined CPUX86_64}
	{$setc __x86_64__ := 1}
{$elsec}
	{$setc __x86_64__ := 0}
{$endc}
{$ifc not defined __arm__ and defined CPUARM}
	{$setc __arm__ := 1}
{$elsec}
	{$setc __arm__ := 0}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifc not defined __arm64__ and defined CPUAARCH64}
  {$setc __arm64__ := 1}
{$elsec}
  {$setc __arm64__ := 0}
{$endc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4

{$ifc defined cpu64}
  {$setc __LP64__ := 1}
{$elsec}
  {$setc __LP64__ := 0}
{$endc}


{$ifc defined __ppc__ and __ppc__ and defined __i386__ and __i386__}
	{$error Conflicting definitions for __ppc__ and __i386__}
{$endc}

{$ifc defined __ppc__ and __ppc__}
	{$setc TARGET_CPU_PPC := TRUE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __ppc64__ and __ppc64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := TRUE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __i386__ and __i386__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := TRUE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{$elifc defined __x86_64__ and __x86_64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := TRUE}
	{$setc TARGET_CPU_ARM := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
{$ifc defined(iphonesim)}
 	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := TRUE}
{$elsec}
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
{$endc}
	{$setc TARGET_OS_EMBEDDED := FALSE}
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> graemeg/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/cpstrnew
=======
	{$setc TARGET_OS_MAC := TRUE}
	{$setc TARGET_OS_IPHONE := FALSE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
>>>>>>> origin/fixes_2.4
{$elifc defined __arm__ and __arm__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := TRUE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_CPU_ARM64 := FALSE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elifc defined __arm64__ and __arm64__}
	{$setc TARGET_CPU_PPC := FALSE}
	{$setc TARGET_CPU_PPC64 := FALSE}
	{$setc TARGET_CPU_X86 := FALSE}
	{$setc TARGET_CPU_X86_64 := FALSE}
	{$setc TARGET_CPU_ARM := FALSE}
	{$setc TARGET_CPU_ARM64 := TRUE}
=======
>>>>>>> graemeg/cpstrnew
	{ will require compiler define when/if other Apple devices with ARM cpus ship }
	{$setc TARGET_OS_MAC := FALSE}
	{$setc TARGET_OS_IPHONE := TRUE}
	{$setc TARGET_IPHONE_SIMULATOR := FALSE}
<<<<<<< HEAD
	{$setc TARGET_OS_EMBEDDED := TRUE}
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ nor __arm64__ is defined.}
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> graemeg/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/cpstrnew
=======
{$elsec}
	{$error __ppc__ nor __ppc64__ nor __i386__ nor __x86_64__ nor __arm__ is defined.}
>>>>>>> origin/fixes_2.4
{$endc}

{$ifc defined __LP64__ and __LP64__ }
  {$setc TARGET_CPU_64 := TRUE}
{$elsec}
  {$setc TARGET_CPU_64 := FALSE}
{$endc}

{$ifc defined FPC_BIG_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := TRUE}
	{$setc TARGET_RT_LITTLE_ENDIAN := FALSE}
{$elifc defined FPC_LITTLE_ENDIAN}
	{$setc TARGET_RT_BIG_ENDIAN := FALSE}
	{$setc TARGET_RT_LITTLE_ENDIAN := TRUE}
{$elsec}
	{$error Neither FPC_BIG_ENDIAN nor FPC_LITTLE_ENDIAN are defined.}
{$endc}
{$setc ACCESSOR_CALLS_ARE_FUNCTIONS := TRUE}
{$setc CALL_NOT_IN_CARBON := FALSE}
{$setc OLDROUTINENAMES := FALSE}
{$setc OPAQUE_TOOLBOX_STRUCTS := TRUE}
{$setc OPAQUE_UPP_TYPES := TRUE}
{$setc OTCARBONAPPLICATION := TRUE}
{$setc OTKERNEL := FALSE}
{$setc PM_USE_SESSION_APIS := TRUE}
{$setc TARGET_API_MAC_CARBON := TRUE}
{$setc TARGET_API_MAC_OS8 := FALSE}
{$setc TARGET_API_MAC_OSX := TRUE}
{$setc TARGET_CARBON := TRUE}
{$setc TARGET_CPU_68K := FALSE}
{$setc TARGET_CPU_MIPS := FALSE}
{$setc TARGET_CPU_SPARC := FALSE}
{$setc TARGET_OS_UNIX := FALSE}
{$setc TARGET_OS_WIN32 := FALSE}
{$setc TARGET_RT_MAC_68881 := FALSE}
{$setc TARGET_RT_MAC_CFM := FALSE}
{$setc TARGET_RT_MAC_MACHO := TRUE}
{$setc TYPED_FUNCTION_POINTERS := TRUE}
{$setc TYPE_BOOL := FALSE}
{$setc TYPE_EXTENDED := FALSE}
{$setc TYPE_LONGLONG := TRUE}
uses MacTypes,Appearance,CarbonEvents,Controls,MacTextEditor,QuickdrawTypes,TextCommon, CFBase,CGBase,HIGeometry,HIObject,HIView;
{$endc} {not MACOSALLINCLUDE}


{$ifc TARGET_OS_MAC}

{$ALIGN MAC68K}


{
 *  HITextViews.h
 *  
 *  Discussion:
 *    API definitions for the standard text-display and text-editing
 *    views: Static text view, HITextView, EditUnicodeText view, and
 *    (deprecated) classic EditText view.
 }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ Static Text (CDEF 18)                                                             }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{ Static Text proc IDs }
const
	kControlStaticTextProc = 288;

{ Control Kind Tag }
const
	kControlKindStaticText = FourCharCode('stxt');

{ The HIObject class ID for the HIStaticTextView class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHIStaticTextViewClassID CFSTRP('com.apple.HIStaticTextView')}
{$endc}
{ Creation API: Carbon only }
{$ifc not TARGET_CPU_64}
{
 *  CreateStaticTextControl()
 *  
 *  Summary:
 *    Creates a new static text control.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control.
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        in CarbonLib 1.1 and later
 *    Non-Carbon CFM:   not available
 }
function CreateStaticTextControl( window: WindowRef { can be NULL }; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateStaticTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{$endc} {not TARGET_CPU_64}


{
 *  Summary:
 *    Tagged data supported by the static text control
 }
const
{
   * Used to get or set the control's current text style. Data is of
   * type ControlFontStyleRec. Available with Appearance Manager 1.0
   * (Mac OS 8.0) and later.
   }
	kControlStaticTextStyleTag = kControlFontStyleTag;

  {
   * Used to get or set the control's current text. Data is an array of
   * chars. Generally you should used GetControlDataSize to determine
   * the length of the text, and allocate a buffer of that length,
   * before calling GetControlData with this selector. Deprecated in
   * Carbon in favor of kControlStaticTextCFStringTag. Available with
   * Appearance Manager 1.0 (Mac OS 8.0) and later.
   }
	kControlStaticTextTextTag = FourCharCode('text');

  {
   * Used to get the height of the control's text. May not be used with
   * SetControlData. Data is of type SInt16. Available with Appearance
   * Manager 1.0 (Mac OS 8.0) and later.
   }
	kControlStaticTextTextHeightTag = FourCharCode('thei');

  {
   * Used to get or set the control's text truncation style. Data is of
   * type TruncCode; pass a truncation code of -1 to indication no
   * truncation. Available with Appearance Manager 1.1 (Mac OS 8.5) and
   * later. Truncation will not occur unless
   * kControlStaticTextIsMultilineTag is set to false.
   }
	kControlStaticTextTruncTag = FourCharCode('trun');

  {
   * Used to get or set the control's current text. Data is of type
   * CFStringRef. When setting the text, the control will retain the
   * string, so you may release the string after calling
   * SetControlData; if the string is mutable, the control will make a
   * copy of the string, so any changes to the string after calling
   * SetControlData will not affect the control. When getting the text,
   * the control retains the string before returning it to you, so you
   * must release the string after you are done with it. Available in
   * CarbonLib 1.5 and Mac OS X 10.0 and later.
   }
	kControlStaticTextCFStringTag = FourCharCode('cfst');

  {
   * Used to get or set whether the control draws its text in multiple
   * lines if the text is too wide for the control bounds. If false,
   * then the control always draws the text in a single line. Data is
   * of type Boolean. Default is true. Available in Mac OS X 10.1 and
   * later.
   }
	kControlStaticTextIsMultilineTag = FourCharCode('stim');


{==============================================================================}
{  Text field events                                                           }
{  A text field is the part of some views that you can enter text into.        }
{  A text field is common to the EditText, EditUnicodeText, ComboBox,          }
{  HISearchField, and HITextView views. The kEventClassTextField event allows  }
{  you to receive notifications when the text has been accepted by the user.   }
{  For example, you can install a handler for a kEventClassTextField /         }
{  kEventTextAccepted event on a HISearchField view to receive a notification  }
{  that the user has initiated a search by hitting the return or enter key.    }
{  You can also filter the text that will replace a selection before the       }
{  change has been made to either accept or reject the replacement.            }
{==============================================================================}
{
    kEventClassTextField quick reference:
    
    kEventTextAccepted              = 1,
    kEventTextShouldChangeInRange   = 2,
    kEventTextDidChange             = 3
}
const
	kEventClassTextField = FourCharCode('txfd');

const
	kEventParamTextSelection = FourCharCode('txsl'); { typeCFRange}
	kEventParamCandidateText = FourCharCode('tstx'); { typeCFStringRef}
	kEventParamReplacementText = FourCharCode('trtx'); { typeCFStringRef}
	kEventParamUnconfirmedRange = FourCharCode('tunr'); { typeCFRange}
	kEventParamUnconfirmedText = FourCharCode('txun'); { typeCFStringRef}

{
 *  kEventClassTextField / kEventTextAccepted
 *  
 *  Summary:
 *    Notification that the text in a view's editable text field has
 *    been accepted.
 *  
 *  Discussion:
 *    This event is sent as a notification when the text contained in a
 *    view's editable text field has been accepted by the user. Text is
 *    accepted when the user presses return or enter on the keyboard
 *    for the EditUnicodeText, HIComboBox, and HISearchField views, or
 *    when the text has changed in the field and the field loses focus
 *    for the EditUnicodeText, HIComboBox, HISearchField and HITextView
 *    views. 
 *    
 *    This event is sent to the view containing the text field only, it
 *    will not propagate. It is sent to all handlers installed on the
 *    view containing the text field.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeControlRef)
 *          The editable text field that has sent the notification.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventTextAccepted = 1;

{
 *  kEventClassTextField / kEventTextShouldChangeInRange
 *  
 *  Summary:
 *    Returns whether the text should be changed in editable text
 *    fields.
 *  
 *  Discussion:
 *    There are several editable text field views, such as the
 *    HIComboBox, HISearchField, HITextView, and EditUnicodeText views.
 *    There are times when you may require fine-grained control over
 *    what text is inserted into the text field and either accept the
 *    changes, reject them or modify what is to be entered. This event
 *    is sent whenever the text is about to be modified in a text
 *    field, either by user input or in other scenarios such as a paste
 *    from the clipboard, spell-checking word correction, or Mac OS X
 *    Service operation. You can change what text is inserted by
 *    providing a replacement string as a parameter to this event. This
 *    event is only sent for Unicode text views; it is not sent for the
 *    classic non-Unicode EditText control. 
 *    
 *    This event is not sent prior to programmatic modification of the
 *    text field contents using SetControlData. 
 *    
 *    This event is not sent while an active inline editing session is
 *    in progress. Once the inline text has been confirmed, this event
 *    will be sent prior to the confirmed text being inserted into the
 *    text field. If you need control over keystrokes during an inline
 *    editing session, you can use the kEventTextInputFilterText event.
 *    
 *    
 *    This event is sent to the view containing the text field only; it
 *    will not propagate.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeControlRef)
 *          The editable text field that has sent the notification.
 *          Available in Mac OS X 10.5 and later.
 *    
 *    --> kEventParamTextSelection (in, typeCFRange)
 *          The range of the selection that is about to be changed. The
 *          units of the selection are in the same units that are
 *          returned in a EditTextSelectionRec, when called with
 *          GetControlData using kControlEditTextSelectionTag.
 *    
 *    --> kEventParamCandidateText (in, typeCFStringRef)
 *          The text that is going to replace the selection. Note that
 *          this string was originally created with
 *          CFStringCreateWithCharactersNoCopy, and the original text
 *          has a limited lifespan. If for some reason you need to
 *          retain the text past the end of your event handler, you
 *          should extract the characters from the string with
 *          CFStringGetCharacters, and then store those characters or
 *          create a new CFString from them.
 *    
 *    <-- kEventParamReplacementText (out, typeCFStringRef)
 *          On output, can contain optional replacement text.
 *  
 *  Result:
 *    If noErr is returned from your handler and the
 *    kEventParamReplacementText parameter is added to the event, then
 *    the contents of that parameter, rather than the candidate text,
 *    will be added to the text field. 
 *    
 *    If noErr is returned from your handler and the
 *    kEventParamReplacementText parameter is _not_ added to the event,
 *    then the candidate text will be filtered out and no text will be
 *    entered in the text field. The current selection will be deleted,
 *    however. 
 *    
 *    If userCanceledErr is returned from your handler, then no text
 *    will be entered in the text field and the current selection will
 *    remain unchanged. Effectively, the editing operation will be
 *    ignored. Note that when an inline text editing session is active,
 *    you will only receive this event when the user attempts to
 *    confirm the inline input. You should not return userCanceledErr
 *    at this point; doing so will not really prevent the text from
 *    being committed. 
 *    
 *    If eventNotHandledErr is returned from your handler, the contents
 *    of the kEventParamReplacementText parameter are ignored, and the
 *    candidate text will replace the selection. 
 *    
 *    Any other return value will result in the default behavior, as if
 *    eventNotHandledErr had been returned.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventTextShouldChangeInRange = 2;

{
 *  kEventClassTextField / kEventTextDidChange
 *  
 *  Summary:
 *    Indicates that the contents of an editable text field have
 *    changed.
 *  
 *  Discussion:
 *    This event is sent by all of the Unicode-based editable text
 *    views: HIComboBox, HISearchField, HITextView and EditUnicodeText.
 *    This event is not sent for the classic non-Unicode EditText view.
 *    
 *    
 *    Note that this event is sent after inline editing operations,
 *    such as pressing a dead key, or using a input method that creates
 *    an inline editing hole. Most clients of this event should ignore
 *    the event during inline editing, and only respond to changes to
 *    the text after inline editing completes. A client can check for
 *    the presence of the kEventParamUnconfirmedRange parameter to
 *    determine whether inline editing is currently active; if this
 *    parameter is present, the client may wish to ignore the event.
 *    
 *    
 *    This event is not sent after programmatic modification of the
 *    text field contents using SetControlData. 
 *    
 *    This event is sent only to the view containing the text field; it
 *    will not propagate. It is sent to all handlers registered for it.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    --> kEventParamDirectObject (in, typeControlRef)
 *          The editable text field that has sent the notification.
 *          Available in Mac OS X 10.5 and later.
 *    
 *    --> kEventParamUnconfirmedRange (in, typeCFRange)
 *          If the text field currently has an open inline hole, this
 *          parameter contains the range of text inside the hole. This
 *          parameter is optional and is only present during inline
 *          editing.
 *    
 *    --> kEventParamUnconfirmedText (in, typeCFStringRef)
 *          If the text field currently has an open inline hole, this
 *          parameter contains the non-confirmed text currently being
 *          edited inside the hole. This parameter is optional and is
 *          only present during inline editing. Note that this string
 *          was originally created with
 *          CFStringCreateWithCharactersNoCopy, and the original text
 *          has a limited lifespan. If for some reason you need to
 *          retain the text past the end of your event handler, you
 *          should extract the characters from the string with
 *          CFStringGetCharacters, and then store those characters or
 *          create a new CFString from them.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.4 and later in Carbon.framework
 *    CarbonLib:        not available
 }
const
	kEventTextDidChange = 3;


{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}
{  ¥ HITextView                                                                                        }
{ÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑÑ}

{==============================================================================}
{ HITextView is a MLTE view that can be embedded in the HIView hierarchy. The  }
{ view can be embedded in an HIScrollView if scroll bars are desired and can   }
{ be used in a composited window. On creation, a TXNObject is created to back  }
{ the view. You can extract the TXNObject at any time and use a subset of the  }
{ MLTE API with that object as an argument.                                    }
{==============================================================================}
{
    In Mac OS X 10.4 and later, HITextView supports these tags previously defined for the EditUnicodeText control:
    
        kControlEditTextCharCount
        kControlEditTextSelectionTag
        kControlEditTextCFStringTag
        kControlEditTextInsertCFStringRefTag
}

{ The HIObject class ID for the HITextView class. }
{
 *  kHITextViewClassID
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
var kHITextViewClassID: CFStringRef; external name '_kHITextViewClassID'; (* attribute const *)
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)
{ ControlKind}
const
	kControlKindHITextView = FourCharCode('hitx');


{$ifc not TARGET_CPU_64}
{
 *  HITextViewCreate()
 *  
 *  Summary:
 *    Creates a text view. The new view is initially invisible.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inBoundsRect:
 *      The bounding box of the view. If NULL, the bounds of the view
 *      will be initialized to 0.
 *    
 *    inOptions:
 *      There are currently no options. This must be 0.
 *    
 *    inTXNFrameOptions:
 *      Any frame options desired for the TXN object creation.
 *    
 *    outTextView:
 *      On exit, contains the new view.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HITextViewCreate( {const} inBoundsRect: HIRectPtr { can be NULL }; inOptions: OptionBits; inTXNFrameOptions: TXNFrameOptions; var outTextView: HIViewRef ): OSStatus; external name '_HITextViewCreate';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HITextViewGetTXNObject()
 *  
 *  Summary:
 *    Obtains the TXNObject that backs the text view.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTextView:
 *      The text view that contains the TXNObject you wish to retrieve.
 *  
 *  Result:
 *    The TXNObject backing the given view.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HITextViewGetTXNObject( inTextView: HIViewRef ): TXNObject; external name '_HITextViewGetTXNObject';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HITextViewSetBackgroundColor()
 *  
 *  Summary:
 *    Sets the background color of the view. This allows you to provide
 *    alpha as well. If inColor is NULL, the background of the text
 *    view will not draw.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTextView:
 *      The text view that you are modifying the background color of.
 *    
 *    inColor:
 *      A CGColorRef representing the color or pattern that will fill
 *      the background of the text view. The CGColorRef will be
 *      retained by this API. If the text view already contains a
 *      background color, it will be released prior to the new color
 *      being retained. If inColor is NULL, the background of the text
 *      view will not draw.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HITextViewSetBackgroundColor( inTextView: HIViewRef; inColor: CGColorRef { can be NULL } ): OSStatus; external name '_HITextViewSetBackgroundColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{
 *  HITextViewCopyBackgroundColor()
 *  
 *  Summary:
 *    Gets the background color of the view. If the background color
 *    returned is NULL, the background does not draw.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    inTextView:
 *      The text view from which you want to obtain the background
 *      color.
 *    
 *    outColor:
 *      A CGColorRef representing the color or pattern that is used for
 *      drawing the background of the text view. If the returned value
 *      is NULL, the background does not draw. If the returned
 *      CGColorRef is not NULL, it will be retained on return. You are
 *      responsible for releasing this CGColorRef when you are no
 *      longer referencing it.
 *  
 *  Result:
 *    An operating system status code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.3 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.3 and later
 *    Non-Carbon CFM:   not available
 }
function HITextViewCopyBackgroundColor( inTextView: HIViewRef; var outColor: CGColorRef ): OSStatus; external name '_HITextViewCopyBackgroundColor';
(* AVAILABLE_MAC_OS_X_VERSION_10_3_AND_LATER *)


{---------------------------------------------------------------------------------------}
{ EditUnicodeText                                                                       }
{---------------------------------------------------------------------------------------}
{ This view is only available in Mac OS X.  It is super similar to Edit Text control    }
{ Use all the same Get/Set tags.  But don't ask for the TEHandle.                       }
{---------------------------------------------------------------------------------------}
{$endc} {not TARGET_CPU_64}

const
	kControlEditUnicodeTextProc = 912;
	kControlEditUnicodeTextPasswordProc = 914;

{ Control Kind Tag }
const
	kControlKindEditUnicodeText = FourCharCode('eutx');

{ The HIObject class ID for the HITextField class. }
{$ifc USE_CFSTR_CONSTANT_MACROS}
{$definec kHITextFieldClassID CFSTRP('com.apple.HITextField')}
{$endc}
{$ifc not TARGET_CPU_64}
{
 *  CreateEditUnicodeTextControl()
 *  
 *  Summary:
 *    Creates a new edit text control.
 *  
 *  Discussion:
 *    This is the preferred edit text control. Use it instead of the
 *    EditText control. This control handles Unicode and draws its text
 *    using antialiasing, which the other control does not.
 *  
 *  Mac OS X threading:
 *    Not thread safe
 *  
 *  Parameters:
 *    
 *    window:
 *      The window in which the control should be placed. May be NULL
 *      in 10.3 and later.
 *    
 *    boundsRect:
 *      The bounds of the control, in local coordinates of the window.
 *    
 *    text:
 *      The text of the control. May be NULL.
 *    
 *    isPassword:
 *      A Boolean indicating whether the field is to be used as a
 *      password field. Passing false indicates that the field is to
 *      display entered text normally. True means that the field will
 *      be used as a password field and any text typed into the field
 *      will be displayed only as bullets.
 *    
 *    style:
 *      The control's font style, size, color, and so on. May be NULL.
 *    
 *    outControl:
 *      On exit, contains the new control (if noErr is returned as the
 *      result code).
 *  
 *  Result:
 *    An operating system result code.
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework [32-bit only]
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function CreateEditUnicodeTextControl( window: WindowRef; const (*var*) boundsRect: Rect; text: CFStringRef { can be NULL }; isPassword: Boolean; {const} style: ControlFontStyleRecPtr { can be NULL }; var outControl: ControlRef ): OSStatus; external name '_CreateEditUnicodeTextControl';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)


{ Tagged data supported by edit text }
{$endc} {not TARGET_CPU_64}

const
	kControlEditTextStyleTag = kControlFontStyleTag; { ControlFontStyleRec}
	kControlEditTextTextTag = FourCharCode('text'); { Buffer of chars - you supply the buffer}
	kControlEditTextKeyFilterTag = kControlKeyFilterTag;
	kControlEditTextSelectionTag = FourCharCode('sele'); { ControlEditTextSelectionRec}
	kControlEditTextPasswordTag = FourCharCode('pass'); { The clear text password text}
	kControlEditTextCharCount = FourCharCode('chrc'); { [UInt32] Count of characters in the control's text}

{ tags available with Appearance 1.1 or later }
const
	kControlEditTextKeyScriptBehaviorTag = FourCharCode('kscr'); { ControlKeyScriptBehavior. Defaults to "PrefersRoman" for password fields,}
                                        {       or "AllowAnyScript" for non-password fields.}
	kControlEditTextLockedTag = FourCharCode('lock'); { Boolean. Locking disables editability.}
	kControlEditTextFixedTextTag = FourCharCode('ftxt'); { Like the normal text tag, but fixes inline input first}
	kControlEditTextValidationProcTag = FourCharCode('vali'); { ControlEditTextValidationUPP. Called when a key filter can't be: after cut, paste, etc.}


{
 *  Discussion:
 *    EditText ControlData tags available with Mac OS X and later.
 }
const
{
   * Extract the content of the edit text field as a CFString.  Don't
   * forget that you own the returned CFStringRef and are responsible
   * for CFReleasing it.
   }
	kControlEditTextCFStringTag = FourCharCode('cfst'); { CFStringRef (Also available on CarbonLib 1.5)}

  {
   * Extract the content of the edit text field as a CFString, if it is
   * a password field.  Don't forget that you own the returned
   * CFStringRef and are responsible for CFReleasing it.
   }
	kControlEditTextPasswordCFStringTag = FourCharCode('pwcf'); { CFStringRef}

const
	kControlEditTextSingleLineTag = FourCharCode('sglc'); { data is a Boolean; indicates whether the control should always be single-line}
	kControlEditTextInsertTextBufferTag = FourCharCode('intx'); { data is an array of char; get or set the control's text as WorldScript-encoded text}
	kControlEditTextInsertCFStringRefTag = FourCharCode('incf'); { data is a CFStringRef; get or set the control's text as a CFStringRef. Caller should release CFString if getting.}
	kControlEditUnicodeTextPostUpdateProcTag = FourCharCode('upup'); { data is a UnicodePostUpdateUPP; get or set the post-update proc}
	kControlEditTextSpellCheckingTag = FourCharCode('spck'); { data is a Boolean; indicates whether the control wants to have spell checking support. Available in Leopard and later.}
	kControlEditTextSpellCheckAsYouTypeTag = FourCharCode('scat'); { data is a Boolean; indicates whether you want to support spell-checking-as-you-type. Available in Leopard and later.}

{ Structure for getting the edit text selection. Used with kControlEditTextSelectionTag. }
type
	ControlEditTextSelectionRec = record
		selStart: SInt16;
		selEnd: SInt16;
	end;
type
	ControlEditTextSelectionPtr = ^ControlEditTextSelectionRec;
	ControlEditTextValidationProcPtr = procedure( control: ControlRef );
{ This callback supplies the functionality of the TSMTEPostUpdateProcPtr that is used }
{ in the EditText control.  A client should supply this call if they want to look at  }
{ inline text that has been fixed before it is included in the actual body text       }
{ if the new text (i.e. the text in the handle) should be included in the body text    }
{ the client should return true.  If the client wants to block the inclusion of the    }
{ text they should return false.                                                       }
type
	EditUnicodePostUpdateProcPtr = function( uniText: UniCharArrayHandle; uniTextLength: UniCharCount; iStartOffset: UniCharArrayOffset; iEndOffset: UniCharArrayOffset; refcon: UnivPtr ): Boolean;
	ControlEditTextValidationUPP = ControlEditTextValidationProcPtr;
	EditUnicodePostUpdateUPP = EditUnicodePostUpdateProcPtr;
{
 *  NewControlEditTextValidationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
function NewControlEditTextValidationUPP( userRoutine: ControlEditTextValidationProcPtr ): ControlEditTextValidationUPP; external name '_NewControlEditTextValidationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  NewEditUnicodePostUpdateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function NewEditUnicodePostUpdateUPP( userRoutine: EditUnicodePostUpdateProcPtr ): EditUnicodePostUpdateUPP; external name '_NewEditUnicodePostUpdateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeControlEditTextValidationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure DisposeControlEditTextValidationUPP( userUPP: ControlEditTextValidationUPP ); external name '_DisposeControlEditTextValidationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  DisposeEditUnicodePostUpdateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
procedure DisposeEditUnicodePostUpdateUPP( userUPP: EditUnicodePostUpdateUPP ); external name '_DisposeEditUnicodePostUpdateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeControlEditTextValidationUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        in CarbonLib 1.0 and later
 *    Non-Carbon CFM:   available as macro/inline
 }
procedure InvokeControlEditTextValidationUPP( control: ControlRef; userUPP: ControlEditTextValidationUPP ); external name '_InvokeControlEditTextValidationUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{
 *  InvokeEditUnicodePostUpdateUPP()
 *  
 *  Availability:
 *    Mac OS X:         in version 10.0 and later in Carbon.framework
 *    CarbonLib:        not available in CarbonLib 1.x, is available on Mac OS X version 10.0 and later
 *    Non-Carbon CFM:   not available
 }
function InvokeEditUnicodePostUpdateUPP( uniText: UniCharArrayHandle; uniTextLength: UniCharCount; iStartOffset: UniCharArrayOffset; iEndOffset: UniCharArrayOffset; refcon: UnivPtr; userUPP: EditUnicodePostUpdateUPP ): Boolean; external name '_InvokeEditUnicodePostUpdateUPP';
(* AVAILABLE_MAC_OS_X_VERSION_10_0_AND_LATER *)

{$endc} {TARGET_OS_MAC}

{$ifc not defined MACOSALLINCLUDE or not MACOSALLINCLUDE}

end.
{$endc} {not MACOSALLINCLUDE}
