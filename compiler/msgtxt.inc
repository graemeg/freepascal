{$ifdef Delphi}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const msgtxt : array[0..000318] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000318,1..240] of char=(
=======
const msgtxt : array[0..000245] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000245,1..240] of char=(
>>>>>>> graemeg/cpstrnew
=======
const msgtxt : array[0..000245] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000245,1..240] of char=(
>>>>>>> graemeg/cpstrnew
=======
const msgtxt : array[0..000245] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000245,1..240] of char=(
>>>>>>> graemeg/cpstrnew
=======
const msgtxt : array[0..000245] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000245,1..240] of char=(
>>>>>>> origin/cpstrnew
=======
const msgtxt : array[0..000192] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000192,1..240] of char=(
>>>>>>> graemeg/fixes_2_2
=======
const msgtxt : array[0..000192] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000192,1..240] of char=(
>>>>>>> origin/fixes_2_2
=======
const msgtxt : array[0..000215] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000215,1..240] of char=(
>>>>>>> origin/fixes_2.4
{$endif Delphi}
  '01000_T_Compiler: $1'#000+
  '01001_D_Compiler OS: $1'#000+
  '01002_I_Target OS: $1'#000+
  '01003_T_Using executable path: $1'#000+
  '01004_T_Using unit path: $1'#000+
  '01005_T_Using include path: $1'#000+
  '01006_T_Using library path: $1'#000+
  '01007_T_Using object path: $1'#000+
  '01008_I_$1 lines co','mpiled, $2 sec $3'#000+
  '01009_F_No memory left'#000+
  '01010_I_Writing Resource String Table file: $1'#000+
  '01011_E_Writing Resource String Table file: $1'#000+
  '01012_I_Fatal:'#000+
  '01013_I_Error:'#000+
  '01014_I_Warning:'#000+
  '01015_I_Note:'#000+
  '01016_I_Hint:'#000+
  '01017_E_Path "$1" does not exi','st'#000+
  '01018_F_Compilation aborted'#000+
  '01019_bytes code'#000+
  '01020_bytes data'#000+
  '01021_I_$1 warning(s) issued'#000+
  '01022_I_$1 hint(s) issued'#000+
  '01023_I_$1 note(s) issued'#000+
  '01024_F_I/O error: $1'#000+
  '01025_F_Operating system error: $1'#000+
  '01026_E_Compilation raised exception i','nternally'#000+
  '02000_F_Unexpected end of file'#000+
  '02001_F_String exceeds line'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02002_F_illegal character "$1" ($2)'#000+
=======
  '02002_F_illegal character "$1" ($2)',#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02002_F_illegal character "$1" ($2)',#000+
>>>>>>> origin/fixes_2_2
  '02003_F_Syntax error, "$1" expected but "$2" found'#000+
  '02004_TL_Start reading includefile $1'#000+
  '02005_W_Comment level $1 found'#000+
  '02008_N_Ignored',' compiler switch "$1"'#000+
  '02009_W_Illegal compiler switch "$1"'#000+
<<<<<<< HEAD
  '02010_W_Misplaced global compiler switch, ignored'#000+
  '02011_E_Illegal char constant'#000+
  '02012_F_Cannot open file "$1"'#000+
  '02013_F_Cannot open include file "$1"'#000+
  '02015_E_Illegal record alignment ','specifier "$1"'#000+
  '02016_E_Illegal enum minimum-size specifier "$1"'#000+
  '02017_E_$ENDIF expected for $1 $2 defined in $3 line $4'#000+
=======
  '02010_W_Misplaced global compiler switch'#000+
  '0201','1_E_Illegal char constant'#000+
  '02012_F_Can'#039't open file "$1"'#000+
  '02013_F_Can'#039't open include file "$1"'#000+
  '02015_E_Illegal record alignment specifier "$1"'#000+
  '02016_E_Illegal enum minimum-size specifier "$1"'#000+
  '02017_E_$ENDIF expected for $1 $2 defined in $3 lin','e $4'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '02018_E_Syntax error while parsing a conditional compiling expression'#000+
  '02019_E_Evaluating a conditional compiling express','ion'#000+
  '02020_W_Macro contents are limited to 255 characters in length'#000+
  '02021_E_ENDIF without IF(N)DEF'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02022_F_User defined: $1'#000+
=======
  '02022_F_User defi','ned: $1'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02022_F_User defi','ned: $1'#000+
>>>>>>> origin/fixes_2_2
  '02023_E_User defined: $1'#000+
  '02024_W_User defined: $1'#000+
  '02025_N_User defined: $1'#000+
  '02026_H_User defined: $1'#000+
  '02027_I_User defi','ned: $1'#000+
  '02028_E_Keyword redefined as macro has no effect'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02029_F_Macro buffer overflow while reading or expanding a macro'#000+
=======
  '02029_F_Macro buffer overflow while reading or expanding a',' macro'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02029_F_Macro buffer overflow while reading or expanding a',' macro'#000+
>>>>>>> origin/fixes_2_2
  '02030_W_Expanding of macros exceeds a depth of 16.'#000+
  '02031_W_compiler switches are not supported in // styled comments'#000+
  '0','2032_DL_Handling switch "$1"'#000+
  '02033_CL_ENDIF $1 found'#000+
  '02034_CL_IFDEF $1 found, $2'#000+
  '02035_CL_IFOPT $1 found, $2'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02036_CL_IF $1 found, $2'#000+
=======
  '02036_C','L_IF $1 found, $2'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02036_C','L_IF $1 found, $2'#000+
>>>>>>> origin/fixes_2_2
  '02037_CL_IFNDEF $1 found, $2'#000+
  '02038_CL_ELSE $1 found, $2'#000+
  '02039_CL_Skipping until...'#000+
  '02040_I_Press <return> ','to continue'#000+
  '02041_W_Unsupported switch "$1"'#000+
  '02042_W_Illegal compiler directive "$1"'#000+
  '02043_TL_Back in $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02044_W_Unsupported application type: "$1"'#000+
=======
  '02044_W_Unsu','pported application type: "$1"'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02044_W_Unsu','pported application type: "$1"'#000+
>>>>>>> origin/fixes_2_2
  '02045_W_APPTYPE is not supported by the target OS'#000+
  '02046_W_DESCRIPTION is not supported by the',' target OS'#000+
  '02047_N_VERSION is not supported by target OS'#000+
  '02048_N_VERSION only for exes or DLLs'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02049_W_Wrong format for VERSION directive "$1"'#000+
=======
  '02049_W_Wrong format ','for VERSION directive "$1"'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02049_W_Wrong format ','for VERSION directive "$1"'#000+
>>>>>>> origin/fixes_2_2
  '02050_E_Illegal assembler style specified "$1"'#000+
  '02051_W_ASM reader switch is not possible inside a','sm statement, "$1"'+
  ' will be effective only for next'#000+
  '02052_E_Wrong switch toggle, use ON/OFF or +/-'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02053_E_Resource files are not supported for this target'#000+
=======
  '02053_E_Resource f','iles are not supported for this target'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02053_E_Resource f','iles are not supported for this target'#000+
>>>>>>> origin/fixes_2_2
  '02054_W_Include environment "$1" not found in environment'#000+
  '02055_E_Illegal value for F','PU register limit'#000+
  '02056_W_Only one resource file is supported for this target'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02057_W_Macro support has been turned off'#000+
  '02058_E_Illegal interface type specified. Valids are COM, CORBA or DEF'+
  'AULT.'#000+
  '02059_W_APPID is only supported for PalmOS'#000+
  '0','2060_W_APPNAME is only supported for PalmOS'#000+
  '02061_E_Constant strings cannot be longer than 255 chars'#000+
  '02062_F_Including include files exceeds a depth of 16.'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '02057_W_Macro support has been turned ','off'#000+
  '02058_E_Illegal interface type specified. Valids are COM, CORBA or DEF'+
  'AULT.'#000+
  '02059_W_APPID is only supported for PalmOS'#000+
  '02060_W_APPNAME is only supported for PalmOS'#000+
  '02061_E_Constant strings can'#039't be longer than 255 chars'#000+
  '02062_F_Includin','g include files exceeds a depth of 16.'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '02063_F_Too many levels of PUSH'#000+
  '02064_E_A POP without a preceding PUSH'#000+
  '02065_E_Macro',' or compile time variable "$1" does not have any value'#000+
  '02066_E_Wrong switch toggle, use ON/OFF/DEFAULT or +/-/*'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02067_E_Mode switch "$1" not allowed here'#000+
=======
  '02067','_E_Mode switch "$1" not allowed here'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02067','_E_Mode switch "$1" not allowed here'#000+
>>>>>>> origin/fixes_2_2
  '02068_E_Compile time variable or macro "$1" is not defined.'#000+
  '02069_E_UTF-8 code greater',' than 65535 found'#000+
  '02070_E_Malformed UTF-8 string'#000+
  '02071_C_UTF-8 signature found, using UTF-8 encoding'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02072_E_Compile time expression: Wanted $1 but got $2 at $3'#000+
=======
  '02072_E_Compile ','time expression: Wanted $1 but got $2 at $3'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02072_E_Compile ','time expression: Wanted $1 but got $2 at $3'#000+
>>>>>>> origin/fixes_2_2
  '02073_N_APPTYPE is not supported by the target OS'#000+
  '02074_E_Illegal optimization ','specified "$1"'#000+
  '02075_W_SETPEFLAGS is not supported by the target OS'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02076_W_IMAGEBASE is not supported by the target OS'#000+
=======
  '02076_W_IMAGEBASE is not supported by the target ','OS'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '02076_W_IMAGEBASE is not supported by the target ','OS'#000+
>>>>>>> origin/fixes_2_2
  '02077_W_MINSTACKSIZE is not supported by the target OS'#000+
  '02078_W_MAXSTACKSIZE is not supported by the target OS'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '02079_E_Il','legal state "$1" for $WARN directive'#000+
  '02080_E_Illegal set packing value'#000+
  '02081_W_PIC directive or switch ignored'#000+
  '02082_W_The switch "$1" is not supported by the currently selected tar'+
  'get'#000+
  '02084_W_Framework-related options are only supported fo','r Darwin/Mac '+
  'OS X'#000+
  '02085_E_Illegal minimal floating point constant precision "$1"'#000+
  '02086_W_Overriding name of "main" procedure multiple times, was previo'+
  'usly set to "$1"'#000+
  '02087_W_Illegal identifier "$1" for $WARN directive'#000+
  '02088_E_Illegal alig','nment directive'#000+
  '02089_F_It is not possible to include a file that starts with an UTF-8'+
  ' BOM in a module that uses a different code page'#000+
  '02090_W_Directive "$1" is ignored for the current target platform'#000+
  '02091_W_Current system codepage "$1" is',' not available for the compil'+
  'er. Switching default codepage back to "$2".'#000+
  '02092_W_SETPEOPTFLAGS is not supported by the target OS'#000+
  '02093_E_Illegal argument for SETPEFLAGS'#000+
  '02094_E_Illegal argument for SETPEOPTFLAGS'#000+
  '02095_E_Directive $1 is not',' supported on this target'#000+
  '02096_W_The specified stack size is not within the valid range for the'+
  ' platform. Setting the stack size ignored.'#000+
  '02097_W_The specified HeapMax value is smaller than the HeapMin value.'+
  ' Setting HeapMax ignored.'#000+
  '02098','_E_Illegal argument for HUGEPOINTERNORMALIZATION'#000+
=======
  '02079_E_Illegal state "$1" for $WARN directive'#000+
  '02080_E_Illegal set packing value'#000+
  '02081_W_PIC directive or switch ignored'#000+
=======
  '02079_E_Illegal state "$1" for $WARN directive'#000+
  '02080_E_Illegal set packing value'#000+
  '02081_W_PIC directive or switch ignored'#000+
>>>>>>> graemeg/cpstrnew
=======
  '02079_E_Illegal state "$1" for $WARN directive'#000+
  '02080_E_Illegal set packing value'#000+
  '02081_W_PIC directive or switch ignored'#000+
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  '02079_E_Illegal state "$1" for $WARN directive'#000+
  '02080_E_Illegal set packing value'#000+
  '02081_W_PIC directive or switch ignored'#000+
>>>>>>> origin/cpstrnew
  '02082_','W_The switch "$1" is not supported by the currently selected t'+
=======
  '02082_W_The',' switch "$1" is not supported by the currently selected t'+
>>>>>>> graemeg/fixes_2_2
=======
  '02082_W_The',' switch "$1" is not supported by the currently selected t'+
>>>>>>> origin/fixes_2_2
  'arget'#000+
  '02084_W_Framework-related options are only supported for Darwin/Mac OS'+
  ' X'#000+
  '02085_E_Illegal minimal floating point constant precision "$1"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '02086_W_Overriding name of "main" pr','ocedure multiple times, was prev'+
  'iously set to "$1"'#000+
  '02087_W_Illegal identifier "$1" for $WARN directive'#000+
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
  '03000_E_Parser - Syntax Error'#000+
  '03004_E_INTERRUPT procedure cannot be nested'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '02086_W_Overriding name of "main" procedu','re multiple times, was prev'+
  'iously set to "$1"'#000+
  '03000_E_Parser - Syntax Error'#000+
  '03004_E_INTERRUPT procedure can'#039't be nested'#000+
>>>>>>> graemeg/fixes_2_2
  '03005_W_Procedure type "$1" ignored'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03006_E_Not all declarations of "$1" are declared with OVERLOAD'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03008_E_Duplicat','e exported function name "$1"'#000+
=======
  '03008_E_Duplicate ex','ported function name "$1"'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03008_E_Duplicate ex','ported function name "$1"'#000+
>>>>>>> origin/fixes_2_2
  '03009_E_Duplicate exported function index $1'#000+
  '03010_E_Invalid index for exported function'#000+
  '03011_W_Relocatable DLL or executable $1 debug info does not work, dis'+
  'abled.'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03012_W_To allow debugging for win32 code yo','u need to disable reloca'+
  'tion with -WN option'#000+
=======
  '03006_E_Not all declaration','s of "$1" are declared with OVERLOAD'#000+
  '03008_E_Duplicate exported function name "$1"'#000+
  '03009_E_Duplicate exported function index $1'#000+
  '03010_E_Invalid index for exported function'#000+
=======
  '03006_E_Not all declaration','s of "$1" are declared with OVERLOAD'#000+
  '03008_E_Duplicate exported function name "$1"'#000+
  '03009_E_Duplicate exported function index $1'#000+
  '03010_E_Invalid index for exported function'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03006_E_Not all declaration','s of "$1" are declared with OVERLOAD'#000+
  '03008_E_Duplicate exported function name "$1"'#000+
  '03009_E_Duplicate exported function index $1'#000+
  '03010_E_Invalid index for exported function'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03006_E_Not all declaration','s of "$1" are declared with OVERLOAD'#000+
  '03008_E_Duplicate exported function name "$1"'#000+
  '03009_E_Duplicate exported function index $1'#000+
  '03010_E_Invalid index for exported function'#000+
>>>>>>> origin/cpstrnew
  '03011_W_Relocatable DLL or executable $1 debug info does not work, d','i'+
  'sabled.'#000+
  '03012_W_To allow debugging for win32 code you need to disable relocati'+
  'on with -WN option'#000+
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
  '03012_W_To allow debugging for win32 code you ne','ed to disable reloca'+
  'tion with -WN option'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03012_W_To allow debugging for win32 code you ne','ed to disable reloca'+
  'tion with -WN option'#000+
>>>>>>> origin/fixes_2_2
  '03013_E_Constructor name must be INIT'#000+
  '03014_E_Destructor name must be DONE'#000+
  '03016_E_Procedure type INLINE not supported'#000+
  '03018_W_Constructor sho','uld be public'#000+
  '03019_W_Destructor should be public'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '030','20_N_Class should have one destructor only'#000+
=======
  '03020_N','_Class should have one destructor only'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03020_N','_Class should have one destructor only'#000+
>>>>>>> origin/fixes_2_2
  '03021_E_Local class definitions are not allowed'#000+
  '03022_F_Anonymous class definitions are not allowed'#000+
  '03023_N_The object "$1" has no VMT'#000+
  '03024_E_Illegal parameter list'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03026_E_Wrong number of paramet','ers specified for call to "$1"'#000+
=======
  '03026_E_Wrong number of parameters ','specified for call to "$1"'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03026_E_Wrong number of parameters ','specified for call to "$1"'#000+
>>>>>>> origin/fixes_2_2
  '03027_E_overloaded identifier "$1" isn'#039't a function'#000+
  '03028_E_overloaded functions have the same parameter list'#000+
  '03029_E_function header doesn'#039't match the previous declaration "$1'+
  '"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03030_E_function header "$1" do','esn'#039't match forward : var name ch'+
  'anges $2 => $3'#000+
  '03031_N_Values in enumeration types have to be ascending'#000+
  '03033_E_With cannot be used for variables in a different segment'#000+
  '03034_E_function nesting > 31'#000+
  '03035_E_range check error while evaluati','ng constants'#000+
  '03036_W_range check error while evaluating constants'#000+
  '03037_E_duplicate case label'#000+
  '03038_E_Upper bound of case range is less than lower bound'#000+
  '03039_E_typed constants of classes or interfaces are not allowed'#000+
  '03040_E_functions var','iables of overloaded functions are not allowed'#000+
  '03041_E_string length must be a value from 1 to 255'#000+
  '03042_W_use extended syntax of NEW and DISPOSE for instances of object'+
  's'#000+
  '03043_W_use of NEW or DISPOSE for untyped pointers is meaningless'#000+
  '030','44_E_use of NEW or DISPOSE is not possible for untyped pointers'#000+
  '03045_E_class identifier expected'#000+
  '03046_E_type identifier not allowed here'#000+
  '03047_E_method identifier expected'#000+
  '03048_E_function header doesn'#039't match any method of this class "$1',
  '"'#000+
  '03049_DL_procedure/function $1'#000+
  '03050_E_Illegal floating point constant'#000+
  '03051_E_FAIL can be used in constructors only'#000+
  '03052_E_Destructors cannot have parameters'#000+
  '03053_E_Only class methods, class properties and class variables can b'+
  'e referr','ed with class references'#000+
  '03054_E_Only class methods, class properties and class variables can b'+
  'e accessed in class methods'#000+
  '03055_E_Constant and CASE types do not match'#000+
  '03056_E_The symbol cannot be exported from a library'#000+
  '03057_W_An inherite','d method is hidden by "$1"'#000+
  '03058_E_There is no method in an ancestor class to be overridden: "$1"'+
  #000+
  '03059_E_No member is provided to access property'#000+
  '03060_W_Stored property directive is not yet implemented'#000+
  '03061_E_Illegal symbol for property ','access'#000+
  '03062_E_Cannot access a protected field of an object here'#000+
  '03063_E_Cannot access a private field of an object here'#000+
  '03066_E_Overridden methods must have the same return type: "$2" is ove'+
  'rridden by "$1" which has another return type'#000+
  '030','67_E_EXPORT declared functions cannot be nested'#000+
  '03068_E_Methods cannot be EXPORTed'#000+
  '03069_E_Call by var for arg no. $1 has to match exactly: Got "$2" expe'+
  'cted "$3"'#000+
  '03070_E_Class isn'#039't a parent class of the current class'#000+
  '03071_E_SELF is only ','allowed in methods'#000+
  '03072_E_Methods can be only in other methods called direct with type i'+
  'dentifier of the class'#000+
  '03073_E_Illegal use of '#039':'#039#000+
  '03074_E_range check error in set constructor or duplicate set element'#000+
  '03075_E_Pointer to object expec','ted'#000+
  '03076_E_Expression must be constructor call'#000+
  '03077_E_Expression must be destructor call'#000+
  '03078_E_Illegal order of record elements'#000+
  '03079_E_Expression type must be class or record type, got $1'#000+
  '03080_E_Procedures cannot return a value'#000+
  '03081_','E_constructors, destructors and class operators must be method'+
  's'#000+
  '03082_E_Operator is not overloaded'#000+
  '03083_E_Impossible to overload assignment for equal types'#000+
  '03084_E_Impossible operator overload'#000+
  '03085_E_Re-raise isn'#039't possible there'#000+
  '03086_E_','The extended syntax of new or dispose isn'#039't allowed for '+
  'a class'#000+
  '03088_E_Procedure overloading is switched off'#000+
  '03089_E_It is not possible to overload this operator. Related overload'+
  'able operators (if any) are: $1'#000+
  '03090_E_Comparative operator',' must return a boolean value'#000+
  '03091_E_Only virtual methods can be abstract'#000+
  '03092_F_Use of unsupported feature: "$1".'#000+
  '03093_E_The mix of different kind of objects (class, object, interface'+
  ', etc) isn'#039't allowed'#000+
  '03094_W_Unknown procedure directi','ve had to be ignored: "$1"'#000+
  '03095_E_$1 can be associated with only one variable'#000+
  '03096_E_absolute can only be associated with a var or const'#000+
  '03097_E_Only one variable can be initialized'#000+
  '03098_E_Abstract methods shouldn'#039't have any definition (','with fun'+
  'ction body)'#000+
  '03099_E_This overloaded function cannot be local (must be exported)'#000+
=======
  '03020_N_Class should have one destructor only'#000+
  '03021_E_Local class definitions are not allowed'#000+
  '03022_F_Anonymous class definitions are not allowed'#000+
  '03023_N_The object "$1" has no VMT'#000+
  '03024_E_I','llegal parameter list'#000+
  '03026_E_Wrong number of parameters specified for call to "$1"'#000+
  '03027_E_overloaded identifier "$1" isn'#039't a function'#000+
  '03028_E_overloaded functions have the same parameter list'#000+
  '03029_E_function header doesn'#039't match the prev','ious declaration "'+
  '$1"'#000+
  '03030_E_function header "$1" doesn'#039't match forward : var name chan'+
  'ges $2 => $3'#000+
  '03031_N_Values in enumeration types have to be ascending'#000+
  '03033_E_With cannot be used for variables in a different segment'#000+
  '03034_E_function ','nesting > 31'#000+
  '03035_E_range check error while evaluating constants'#000+
  '03036_W_range check error while evaluating constants'#000+
  '03037_E_duplicate case label'#000+
  '03038_E_Upper bound of case range is less than lower bound'#000+
  '03039_E_typed constants of classe','s or interfaces are not allowed'#000+
  '03040_E_functions variables of overloaded functions are not allowed'#000+
  '03041_E_string length must be a value from 1 to 255'#000+
  '03042_W_use extended syntax of NEW and DISPOSE for instances of object'+
  's'#000+
  '03043_W_use of N','EW or DISPOSE for untyped pointers is meaningless'#000+
  '03044_E_use of NEW or DISPOSE is not possible for untyped pointers'#000+
  '03045_E_class identifier expected'#000+
  '03046_E_type identifier not allowed here'#000+
  '03047_E_method identifier expected'#000+
=======
  '03020_N_Class should have one destructor only'#000+
  '03021_E_Local class definitions are not allowed'#000+
  '03022_F_Anonymous class definitions are not allowed'#000+
  '03023_N_The object "$1" has no VMT'#000+
  '03024_E_I','llegal parameter list'#000+
  '03026_E_Wrong number of parameters specified for call to "$1"'#000+
  '03027_E_overloaded identifier "$1" isn'#039't a function'#000+
  '03028_E_overloaded functions have the same parameter list'#000+
  '03029_E_function header doesn'#039't match the prev','ious declaration "'+
  '$1"'#000+
  '03030_E_function header "$1" doesn'#039't match forward : var name chan'+
  'ges $2 => $3'#000+
  '03031_N_Values in enumeration types have to be ascending'#000+
  '03033_E_With cannot be used for variables in a different segment'#000+
  '03034_E_function ','nesting > 31'#000+
  '03035_E_range check error while evaluating constants'#000+
  '03036_W_range check error while evaluating constants'#000+
  '03037_E_duplicate case label'#000+
  '03038_E_Upper bound of case range is less than lower bound'#000+
  '03039_E_typed constants of classe','s or interfaces are not allowed'#000+
  '03040_E_functions variables of overloaded functions are not allowed'#000+
  '03041_E_string length must be a value from 1 to 255'#000+
  '03042_W_use extended syntax of NEW and DISPOSE for instances of object'+
  's'#000+
  '03043_W_use of N','EW or DISPOSE for untyped pointers is meaningless'#000+
  '03044_E_use of NEW or DISPOSE is not possible for untyped pointers'#000+
  '03045_E_class identifier expected'#000+
  '03046_E_type identifier not allowed here'#000+
  '03047_E_method identifier expected'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03020_N_Class should have one destructor only'#000+
  '03021_E_Local class definitions are not allowed'#000+
  '03022_F_Anonymous class definitions are not allowed'#000+
  '03023_N_The object "$1" has no VMT'#000+
  '03024_E_I','llegal parameter list'#000+
  '03026_E_Wrong number of parameters specified for call to "$1"'#000+
  '03027_E_overloaded identifier "$1" isn'#039't a function'#000+
  '03028_E_overloaded functions have the same parameter list'#000+
  '03029_E_function header doesn'#039't match the prev','ious declaration "'+
  '$1"'#000+
  '03030_E_function header "$1" doesn'#039't match forward : var name chan'+
  'ges $2 => $3'#000+
  '03031_N_Values in enumeration types have to be ascending'#000+
  '03033_E_With cannot be used for variables in a different segment'#000+
  '03034_E_function ','nesting > 31'#000+
  '03035_E_range check error while evaluating constants'#000+
  '03036_W_range check error while evaluating constants'#000+
  '03037_E_duplicate case label'#000+
  '03038_E_Upper bound of case range is less than lower bound'#000+
  '03039_E_typed constants of classe','s or interfaces are not allowed'#000+
  '03040_E_functions variables of overloaded functions are not allowed'#000+
  '03041_E_string length must be a value from 1 to 255'#000+
  '03042_W_use extended syntax of NEW and DISPOSE for instances of object'+
  's'#000+
  '03043_W_use of N','EW or DISPOSE for untyped pointers is meaningless'#000+
  '03044_E_use of NEW or DISPOSE is not possible for untyped pointers'#000+
  '03045_E_class identifier expected'#000+
  '03046_E_type identifier not allowed here'#000+
  '03047_E_method identifier expected'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03020_N_Class should have one destructor only'#000+
  '03021_E_Local class definitions are not allowed'#000+
  '03022_F_Anonymous class definitions are not allowed'#000+
  '03023_N_The object "$1" has no VMT'#000+
  '03024_E_I','llegal parameter list'#000+
  '03026_E_Wrong number of parameters specified for call to "$1"'#000+
  '03027_E_overloaded identifier "$1" isn'#039't a function'#000+
  '03028_E_overloaded functions have the same parameter list'#000+
  '03029_E_function header doesn'#039't match the prev','ious declaration "'+
  '$1"'#000+
  '03030_E_function header "$1" doesn'#039't match forward : var name chan'+
  'ges $2 => $3'#000+
  '03031_N_Values in enumeration types have to be ascending'#000+
  '03033_E_With cannot be used for variables in a different segment'#000+
  '03034_E_function ','nesting > 31'#000+
  '03035_E_range check error while evaluating constants'#000+
  '03036_W_range check error while evaluating constants'#000+
  '03037_E_duplicate case label'#000+
  '03038_E_Upper bound of case range is less than lower bound'#000+
  '03039_E_typed constants of classe','s or interfaces are not allowed'#000+
  '03040_E_functions variables of overloaded functions are not allowed'#000+
  '03041_E_string length must be a value from 1 to 255'#000+
  '03042_W_use extended syntax of NEW and DISPOSE for instances of object'+
  's'#000+
  '03043_W_use of N','EW or DISPOSE for untyped pointers is meaningless'#000+
  '03044_E_use of NEW or DISPOSE is not possible for untyped pointers'#000+
  '03045_E_class identifier expected'#000+
  '03046_E_type identifier not allowed here'#000+
  '03047_E_method identifier expected'#000+
>>>>>>> origin/cpstrnew
  '03048_E_funct','ion header doesn'#039't match any method of this class "'+
  '$1"'#000+
  '03049_DL_procedure/function $1'#000+
  '03050_E_Illegal floating point constant'#000+
  '03051_E_FAIL can be used in constructors only'#000+
  '03052_E_Destructors can'#039't have parameters'#000+
  '03053_E_Only class methods',', class properties and class variables can'+
  ' be referred with class references'#000+
  '03054_E_Only class class methods, class properties and class variables'+
  ' can be accessed in class methods'#000+
  '03055_E_Constant and CASE types do not match'#000+
  '03056_E_The sy','mbol can'#039't be exported from a library'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '03030_E_function header "$1" doesn'#039,'t match forward : var name cha'+
  'nges $2 => $3'#000+
  '03031_N_Values in enumeration types have to be ascending'#000+
  '03033_E_With can not be used for variables in a different segment'#000+
  '03034_E_function nesting > 31'#000+
  '03035_E_range check error while evaluating ','constants'#000+
  '03036_W_range check error while evaluating constants'#000+
  '03037_E_duplicate case label'#000+
  '03038_E_Upper bound of case range is less than lower bound'#000+
  '03039_E_typed constants of classes or interfaces are not allowed'#000+
  '03040_E_functions variab','les of overloaded functions are not allowed'#000+
  '03041_E_string length must be a value from 1 to 255'#000+
  '03042_W_use extended syntax of NEW and DISPOSE for instances of object'+
  's'#000+
  '03043_W_use of NEW or DISPOSE for untyped pointers is meaningless'#000+
  '03044_','E_use of NEW or DISPOSE is not possible for untyped pointers'#000+
  '03045_E_class identifier expected'#000+
  '03046_E_type identifier not allowed here'#000+
  '03047_E_method identifier expected'#000+
  '03048_E_function header doesn'#039't match any method of this class "$1'+
  '"'#000+
  '0','3049_DL_procedure/function $1'#000+
  '03050_E_Illegal floating point constant'#000+
  '03051_E_FAIL can be used in constructors only'#000+
  '03052_E_Destructors can'#039't have parameters'#000+
  '03053_E_Only class methods can be referred with class references'#000+
  '03054_E_Only clas','s methods can be accessed in class methods'#000+
  '03055_E_Constant and CASE types do not match'#000+
  '03056_E_The symbol can'#039't be exported from a library'#000+
>>>>>>> graemeg/fixes_2_2
  '03057_W_An inherited method is hidden by "$1"'#000+
  '03058_E_There is no method in an ancestor class to be ','overridden: "$'+
  '1"'#000+
  '03059_E_No member is provided to access property'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03060_W_Stored property directive is',' not yet implemented'#000+
  '03061_E_Illegal symbol for property access'#000+
  '03062_E_Cannot access a protected field of an object here'#000+
  '03063_E_Cannot access a private field of an object here'#000+
  '03066_E_Overridden methods must have the same return type: "$2','" is o'+
  'verridden by "$1" which has another return type'#000+
  '03067_E_EXPORT declared functions can'#039't be nested'#000+
  '03068_E_Methods can'#039't be EXPORTed'#000+
  '03069_E_Call by var for arg no. $1 has to match exactly: Got "$2" expe'+
  'cted "$3"'#000+
  '03070_E_Class isn'#039't a ','parent class of the current class'#000+
=======
  '03060_W_Stored property directive is not yet implemented'#000+
  '03061_E_Illegal symbol for property access'#000+
  '03062_E_Cannot access a protected field of an object here'#000+
  '03063_E_Cannot a','ccess a private field of an object here'#000+
  '03066_E_Overridden methods must have the same return type: "$2" is ove'+
  'rriden by "$1" which has another return type'#000+
  '03067_E_EXPORT declared functions can'#039't be nested'#000+
  '03068_E_Methods can'#039't be EXPORTed'#000+
=======
  '03060_W_Stored property directive is not yet implemented'#000+
  '03061_E_Illegal symbol for property access'#000+
  '03062_E_Cannot access a protected field of an object here'#000+
  '03063_E_Cannot a','ccess a private field of an object here'#000+
  '03066_E_Overridden methods must have the same return type: "$2" is ove'+
  'rriden by "$1" which has another return type'#000+
  '03067_E_EXPORT declared functions can'#039't be nested'#000+
  '03068_E_Methods can'#039't be EXPORTed'#000+
>>>>>>> origin/fixes_2_2
  '0','3069_E_Call by var for arg no. $1 has to match exactly: Got "$2" ex'+
  'pected "$3"'#000+
  '03070_E_Class isn'#039't a parent class of the current class'#000+
>>>>>>> graemeg/fixes_2_2
  '03071_E_SELF is only allowed in methods'#000+
  '03072_E_Methods can be only in other methods called direct with t','ype'+
  ' identifier of the class'#000+
  '03073_E_Illegal use of '#039':'#039#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03074_E_range check error in set constructor o','r duplicate set elemen'+
  't'#000+
=======
  '03074_E_range check error in set constructor or duplicate set element'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03074_E_range check error in set constructor or duplicate set element'#000+
>>>>>>> origin/fixes_2_2
  '03075_E_Pointer to object expected'#000+
  '03076_E_Expression must be constructor call'#000+
  '03077_E_Expression must be destructo','r call'#000+
  '03078_E_Illegal order of record elements'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03079_E_Expression type must be class or record type'#000,
=======
  '03079_E_Expression type must be class or record type'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03079_E_Expression type must be class or record type'#000+
>>>>>>> origin/fixes_2_2
  '03080_E_Procedures can'#039't return a value'#000+
  '03081_E_constructors, destructors and class operators must be methods'#000+
  '03082_E_Operator is not overloaded'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03083_E_Impossible to overload assignment for equal types'#000+
  '03084_E_Impossible operator overload'#000,
  '03085_E_Re-raise isn'#039't possible there'#000+
  '03086_E_The extended syntax of new or dispose isn'#039't allowed for a '+
  'class'#000+
  '03088_E_Procedure overloading is switched off'#000+
  '03089_E_It is not possible to overload this operator. Related overload'+
  'able operators',' (if any) are: $1'#000+
=======
  '03083_E_Imp','ossible to overload assignment for equal types'#000+
  '03084_E_Impossible operator overload'#000+
  '03085_E_Re-raise isn'#039't possible there'#000+
  '03086_E_The extended syntax of new or dispose isn'#039't allowed for a '+
  'class'#000+
  '03088_E_Procedure overloading is switched off'#000,
  '03089_E_It is not possible to overload this operator. Related overload'+
  'able operators (if any) are: $1'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03083_E_Imp','ossible to overload assignment for equal types'#000+
  '03084_E_Impossible operator overload'#000+
  '03085_E_Re-raise isn'#039't possible there'#000+
  '03086_E_The extended syntax of new or dispose isn'#039't allowed for a '+
  'class'#000+
  '03088_E_Procedure overloading is switched off'#000,
  '03089_E_It is not possible to overload this operator. Related overload'+
  'able operators (if any) are: $1'#000+
>>>>>>> origin/fixes_2_2
  '03090_E_Comparative operator must return a boolean value'#000+
  '03091_E_Only virtual methods can be abstract'#000+
  '03092_F_Use of unsupported feature!'#000,
  '03093_E_The mix of different kind of objects (class, object, interface'+
<<<<<<< HEAD
  ', etc) isn'#039't a','llowed'#000+
=======
  ', etc) isn'#039't allowed'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '03094_W_Unknown procedure directive had to be ignored: "$1"'#000+
  '03095_E_absolute can only be associated to one variable'#000+
  '03096_E_absolute can only be asso','ciated with a var or const'#000+
  '03097_E_Only one variable can be initialized'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03098_E_Abst','ract methods shouldn'#039't have any definition (with fun'+
  'ction body)'#000+
  '03099_E_This overloaded function can'#039't be local (must be exported)'+
  #000+
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
  '03100_W_Virtual methods are used without a constructor in "$1"'#000+
=======
  '03098_E_Abstract methods shouldn'#039't have any definition (with funct'+
  'ion body)'#000+
  '03099_E_This overloaded function can'#039't be local (must be exported)'+
  #000+
  '03100_W_Virtual methods a','re used without a constructor in "$1"'#000+
>>>>>>> origin/fixes_2_2
  '03101_CL_Macro defined: $1'#000+
  '03102_CL_Macro unde','fined: $1'#000+
  '03103_CL_Macro $1 set to $2'#000+
  '03104','_I_Compiling $1'#000+
=======
  '03098_E_Abstract methods shouldn'#039't have any definition (with funct'+
  'ion body)'#000+
  '03099_E_This overloaded function can'#039't be local (must be exported)'+
  #000+
  '03100_W_Virtual methods a','re used without a constructor in "$1"'#000+
  '03101_CL_Macro defined: $1'#000+
  '03102_CL_Macro undefined: $1'#000+
  '03103_CL_Macro $1 set to $2'#000+
  '03104_I_Compiling $1'#000+
>>>>>>> graemeg/fixes_2_2
  '03105_UL_Parsing interface of unit $1'#000+
  '03106_UL_Parsing implementation of $1'#000+
  '03107_DL_Compiling $1',' for the second time'#000+
  '03109_E_No property found to override'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03110_E_Only one default property is allowed'#000+
  '03111_E_The default pro','perty must be an array property'#000+
  '03112_E_Virtual constructors are only supported in class object model'#000+
  '03113_E_No default property available'#000+
  '03114_E_The class cannot have a published section, use the {$M+} switc'+
  'h'#000+
  '03115_E_Forward declaration ','of class "$1" must be resolved here to u'+
  'se the class as ancestor'#000+
  '03116_E_Local operators not supported'#000+
  '03117_E_Procedure directive "$1" not allowed in interface section'#000+
  '03118_E_Procedure directive "$1" not allowed in implementation section'+
  #000,
  '03119_E_Procedure directive "$1" not allowed in procvar declaration'#000+
  '03120_E_Function is already declared Public/Forward "$1"'#000+
  '03121_E_Cannot use both EXPORT and EXTERNAL'#000+
  '03123_H_"$1" not yet supported inside inline procedure/function'#000+
  '03124_H','_Inlining disabled'#000+
  '03125_I_Writing Browser log $1'#000+
  '03126_H_may be pointer dereference is missing'#000+
  '03127_F_Selected assembler reader not supported'#000+
  '03128_E_Procedure directive "$1" has conflicts with other directives'#000+
  '03129_E_Calling convention ','doesn'#039't match forward'#000+
  '03131_E_Property cannot have a default value'#000+
  '03132_E_The default value of a property must be constant'#000+
  '03133_E_Symbol cannot be published, can be only a class'#000+
  '03134_E_This kind of property cannot be published'#000+
  '03136_E_An',' import name is required'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '03110_E_Only one default ','property is allowed'#000+
  '03111_E_The default property must be an array property'#000+
  '03112_E_Virtual constructors are only supported in class object model'#000+
  '03113_E_No default property available'#000+
  '03114_E_The class can'#039't have a published section, use the',' {$M+} '+
  'switch'#000+
  '03115_E_Forward declaration of class "$1" must be resolved here to use'+
  ' the class as ancestor'#000+
  '03116_E_Local operators not supported'#000+
  '03117_E_Procedure directive "$1" not allowed in interface section'#000+
  '03118_E_Procedure directive "','$1" not allowed in implementation secti'+
  'on'#000+
=======
  '03110_E_Only one default property is allowed'#000+
  '03111_E_The default property must be an array property'#000+
  '03112_E_Virtual constructors are only supported in class object model'#000+
  '03113_E_No ','default property available'#000+
  '03114_E_The class can'#039't have a published section, use the {$M+} sw'+
  'itch'#000+
  '03115_E_Forward declaration of class "$1" must be resolved here to use'+
  ' the class as ancestor'#000+
  '03116_E_Local operators not supported'#000+
  '03117_E_Pro','cedure directive "$1" not allowed in interface section'#000+
  '03118_E_Procedure directive "$1" not allowed in implementation section'+
  #000+
>>>>>>> graemeg/fixes_2_2
  '03119_E_Procedure directive "$1" not allowed in procvar declaration'#000+
  '03120_E_Function is already declared Public/Fo','rward "$1"'#000+
  '03121_E_Can'#039't use both EXPORT and EXTERNAL'#000+
<<<<<<< HEAD
  '03123_W_"$1" not yet supported',' inside inline procedure/function'#000+
  '03124_W_Inlining disabled'#000+
  '03125_I_Writing Browser log $1'#000+
  '03126_H_may be pointer dereference is missing'#000+
  '03127_F_Selected assembler reader not supported'#000+
  '03128_E_Procedure directive "$1" has conflicts with oth','er directives'+
  #000+
=======
  '03123_W_"$1" not yet supported inside inline procedure/function'#000+
  '03124_W_Inlining disabled'#000+
  '03125_I_Writing Browser log $1'#000+
  '03126_H_may be pointer dereference is missing'#000+
  '03127_F_Selected as','sembler reader not supported'#000+
  '03128_E_Procedure directive "$1" has conflicts with other directives'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03110_E_Only one default property is allowed'#000+
  '03111_E_The default property must be an array property'#000+
  '03112_E_Virtual constructors are only supported in class object model'#000+
  '03113_E_No ','default property available'#000+
  '03114_E_The class can'#039't have a published section, use the {$M+} sw'+
  'itch'#000+
  '03115_E_Forward declaration of class "$1" must be resolved here to use'+
  ' the class as ancestor'#000+
  '03116_E_Local operators not supported'#000+
  '03117_E_Pro','cedure directive "$1" not allowed in interface section'#000+
  '03118_E_Procedure directive "$1" not allowed in implementation section'+
  #000+
  '03119_E_Procedure directive "$1" not allowed in procvar declaration'#000+
  '03120_E_Function is already declared Public/Fo','rward "$1"'#000+
  '03121_E_Can'#039't use both EXPORT and EXTERNAL'#000+
  '03123_W_"$1" not yet supported inside inline procedure/function'#000+
  '03124_W_Inlining disabled'#000+
  '03125_I_Writing Browser log $1'#000+
  '03126_H_may be pointer dereference is missing'#000+
  '03127_F_Selected as','sembler reader not supported'#000+
  '03128_E_Procedure directive "$1" has conflicts with other directives'#000+
>>>>>>> origin/fixes_2_2
  '03129_E_Calling convention doesn'#039't match forward'#000+
  '03131_E_Property can'#039't have a default value'#000+
  '03132_E_The default value of a property must be c','onstant'#000+
  '03133_E_Symbol can'#039't be published, can be only a class'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03134_E_This kind of ','property can'#039't be published'#000+
=======
  '03134_E_This kind of property can'#039't be published'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '03134_E_This kind of property can'#039't be published'#000+
>>>>>>> origin/fixes_2_2
  '03136_E_An import name is required'#000+
>>>>>>> graemeg/cpstrnew
  '03138_E_Division by zero'#000+
  '03139_E_Invalid floating point operation'#000+
  '03140_E_Upper bound of rang','e is less than lower bound'#000+
  '03141_W_string "$1" is longer than "$2"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03142_E_string length is larger than array of char leng','th'#000+
  '03143_E_Illegal expression after message directive'#000+
  '03144_E_Message handlers can take only one call by ref. parameter'#000+
  '03145_E_Duplicate message label: "$1"'#000+
  '03146_E_Self can only be an explicit parameter in methods which are me'+
  'ssage handle','rs'#000+
  '03147_E_Threadvars can be only static or global'#000+
  '03148_F_Direct assembler not supported for binary output format'#000+
  '03149_W_Don'#039't load OBJPAS unit manually, use {$mode objfpc} or {$m'+
  'ode delphi} instead'#000+
  '03150_E_OVERRIDE cannot be used in obje','cts'#000+
  '03151_E_Data types which require initialization/finalization cannot be'+
  ' used in variant records'#000+
  '03152_E_Resourcestrings can be only static or global'#000+
  '03153_E_Exit with argument cannot be used here'#000+
  '03154_E_The type of the storage symbol mu','st be boolean'#000+
  '03155_E_This symbol isn'#039't allowed as storage symbol'#000+
  '03156_E_Only classes which are compiled in $M+ mode can be published'#000+
  '03157_E_Procedure directive expected'#000+
  '03158_E_The value for a property index must be of an ordinal type'#000+
  '03','159_E_Procedure name too short to be exported'#000+
  '03160_E_No DEFFILE entry can be generated for unit global vars'#000+
  '03161_E_Compile without -WD option'#000+
  '03162_F_You need ObjFpc (-S2) or Delphi (-Sd) mode to compile this mod'+
  'ule'#000+
  '03163_E_Cannot export ','with index under $1'#000+
=======
  '03142_E_string le','ngth is larger than array of char length'#000+
=======
  '03142_E_string length is larger than array of char length'#000+
>>>>>>> graemeg/fixes_2_2
  '03143_E_Illegal expression after message directive'#000+
  '03144_E_Message handlers can take only one call by ref. paramete','r'#000+
  '03145_E_Duplicate message label: "$1"'#000+
<<<<<<< HEAD
=======
  '03142_E_string le','ngth is larger than array of char length'#000+
=======
  '03142_E_string length is larger than array of char length'#000+
>>>>>>> origin/fixes_2_2
  '03143_E_Illegal expression after message directive'#000+
  '03144_E_Message handlers can take only one call by ref. paramete','r'#000+
  '03145_E_Duplicate message label: "$1"'#000+
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  '03142_E_string le','ngth is larger than array of char length'#000+
  '03143_E_Illegal expression after message directive'#000+
  '03144_E_Message handlers can take only one call by ref. parameter'#000+
  '03145_E_Duplicate message label: "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03142_E_string le','ngth is larger than array of char length'#000+
  '03143_E_Illegal expression after message directive'#000+
  '03144_E_Message handlers can take only one call by ref. parameter'#000+
  '03145_E_Duplicate message label: "$1"'#000+
>>>>>>> origin/cpstrnew
  '03146_E_Self can only be an explicit paramet','er in methods which are '+
  'message handlers'#000+
  '03147_E_Threadvars can be only static or global'#000+
  '03148_F_Direct assembler not supported for binary output format'#000+
  '03149_W_Don'#039't load OBJPAS unit manually, use \{\$mode objfpc\} or '+
  '\{\$mode delphi\} ins','tead'#000+
  '03150_E_OVERRIDE can'#039't be used in objects'#000+
  '03151_E_Data types which require initialization/finalization can'#039't'+
  ' be used in variant records'#000+
  '03152_E_Resourcestrings can be only static or global'#000+
  '03153_E_Exit with argument can'#039't be used here'#000,
  '03154_E_The type of the storage symbol must be boolean'#000+
  '03155_E_This symbol isn'#039't allowed as storage symbol'#000+
  '03156_E_Only classes which are compiled in $M+ mode can be published'#000+
  '03157_E_Procedure directive expected'#000+
  '03158_E_The value for a pro','perty index must be of an ordinal type'#000+
  '03159_E_Procedure name too short to be exported'#000+
  '03160_E_No DEFFILE entry can be generated for unit global vars'#000+
  '03161_E_Compile without -WD option'#000+
  '03162_F_You need ObjFpc (-S2) or Delphi (-Sd) mode to c','ompile this m'+
  'odule'#000+
=======
  '03146_E_Self can only be an explicit parameter in methods which are me'+
  'ssage handlers'#000+
  '03147_E_Threadvars can be only static or global'#000+
  '03148_F_Direct assembler not supported for binary output format'#000+
  '031','49_W_Don'#039't load OBJPAS unit manually, use \{\$mode objfpc\} o'+
  'r \{\$mode delphi\} instead'#000+
  '03150_E_OVERRIDE can'#039't be used in objects'#000+
  '03151_E_Data types which require initialization/finalization can'#039't'+
  ' be used in variant records'#000+
  '03152_E_Resourc','estrings can be only static or global'#000+
  '03153_E_Exit with argument can'#039't be used here'#000+
  '03154_E_The type of the storage symbol must be boolean'#000+
  '03155_E_This symbol isn'#039't allowed as storage symbol'#000+
  '03156_E_Only classes which are compiled in $M+ mo','de can be published'+
  #000+
  '03157_E_Procedure directive expected'#000+
  '03158_E_The value for a property index must be of an ordinal type'#000+
  '03159_E_Procedure name too short to be exported'#000+
  '03160_E_No DEFFILE entry can be generated for unit global vars'#000+
=======
  '03146_E_Self can only be an explicit parameter in methods which are me'+
  'ssage handlers'#000+
  '03147_E_Threadvars can be only static or global'#000+
  '03148_F_Direct assembler not supported for binary output format'#000+
  '031','49_W_Don'#039't load OBJPAS unit manually, use \{\$mode objfpc\} o'+
  'r \{\$mode delphi\} instead'#000+
  '03150_E_OVERRIDE can'#039't be used in objects'#000+
  '03151_E_Data types which require initialization/finalization can'#039't'+
  ' be used in variant records'#000+
  '03152_E_Resourc','estrings can be only static or global'#000+
  '03153_E_Exit with argument can'#039't be used here'#000+
  '03154_E_The type of the storage symbol must be boolean'#000+
  '03155_E_This symbol isn'#039't allowed as storage symbol'#000+
  '03156_E_Only classes which are compiled in $M+ mo','de can be published'+
  #000+
  '03157_E_Procedure directive expected'#000+
  '03158_E_The value for a property index must be of an ordinal type'#000+
  '03159_E_Procedure name too short to be exported'#000+
  '03160_E_No DEFFILE entry can be generated for unit global vars'#000+
>>>>>>> origin/fixes_2_2
  '03161_','E_Compile without -WD option'#000+
  '03162_F_You need ObjFpc (-S2) or Delphi (-Sd) mode to compile this mod'+
  'ule'#000+
>>>>>>> graemeg/fixes_2_2
  '03163_E_Can'#039't export with index under $1'#000+
>>>>>>> graemeg/cpstrnew
  '03164_E_Exporting of variables is not supported under $1'#000+
  '03165_E_Improper GUID syntax'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03168_W_Procedure named "$1" not found that is suitable for implementi'+
  'ng the $2.$3'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03169_E_interface identifier expected'#000+
  '03170_E_Type ','"$1" cannot be used as array index type'#000+
  '03171_E_Con- and destructors are not allowed in interfaces'#000+
  '03172_E_Access specifiers cannot be used in INTERFACEs and OBJCPROTOCO'+
  'Ls'#000+
  '03173_E_An interface, helper or Objective-C protocol or category can','n'+
  'ot contain fields'#000+
  '03174_E_Cannot declare local procedure as EXTERNAL'#000+
  '03175_W_Some fields coming before "$1" were not initialized'#000+
  '03176_E_Some fields coming before "$1" were not initialized'#000+
  '03177_W_Some fields coming after "$1" were not ini','tialized'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '03169_E_int','erface identifier expected'#000+
  '03170_E_Type "$1" can'#039't be used as array index type'#000+
  '03171_E_Con- and destructors aren'#039't allowed in interfaces'#000+
  '03172_E_Access specifiers can'#039't be used in INTERFACEs and OBJCPROT'+
  'OCOLs'#000+
  '03173_E_An interface or Objecti','ve-C protocol or category cannot cont'+
  'ain fields'#000+
  '03174_E_Can'#039't declare local procedure as EXTERNAL'#000+
  '03175_W_Some fields coming before "$1" weren'#039't initialized'#000+
  '03176_E_Some fields coming before "$1" weren'#039't initialized'#000+
  '03177_W_Some fields comi','ng after "$1" weren'#039't initialized'#000+
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
  '03178_E_VarArgs directive (or '#039'...'#039' in MacPas) without CDecl/C'+
  'PPDecl/MWPascal/StdCall and External'#000+
  '03179_E_Self must be a normal (call-by-value) parameter'#000+
  '03180_E_Interface "$1" has no interface identification'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03181_E_Unknown class',' field or method identifier "$1"'#000+
  '03182_W_Overriding calling convention "$1" with "$2"'#000+
  '03183_E_Typed constants of the type "procedure of object" can only be '+
  'initialized with NIL'#000+
  '03184_E_Default value can only be assigned to one parameter'#000+
  '031','85_E_Default parameter required for "$1"'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
  '0318','1_E_Unknown class field or method identifier "$1"'#000+
  '03182_W_Overriding calling convention "$1" with "$2"'#000+
  '03183_E_Typed constants of the type "procedure of object" can only be '+
  'initialized with NIL'#000+
=======
  '0318','1_E_Unknown class field or method identifier "$1"'#000+
  '03182_W_Overriding calling convention "$1" with "$2"'#000+
  '03183_E_Typed constants of the type "procedure of object" can only be '+
  'initialized with NIL'#000+
>>>>>>> origin/cpstrnew
  '03184_E_Default value can only be assigned to ','one parameter'#000+
  '03185_E_Default parameter required for "$1"'#000+
>>>>>>> graemeg/cpstrnew
  '03186_W_Use of unsupported feature!'#000+
  '03187_H_C arrays are passed by reference'#000+
  '03188_E_C array of const must be the last argument'#000+
  '03189_H_Type "$1" redefinition'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03190_W_cdecl'#039'ared functions have no hig','h parameter'#000+
=======
  '03190_W_cdecl'#039'ared func','tions have no high parameter'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03190_W_cdecl'#039'ared func','tions have no high parameter'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03190_W_cdecl'#039'ared func','tions have no high parameter'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03190_W_cdecl'#039'ared func','tions have no high parameter'#000+
>>>>>>> origin/cpstrnew
  '03191_W_cdecl'#039'ared functions do not support open strings'#000+
  '03192_E_Cannot initialize variables declared as threadvar'#000+
  '03193_E_Message directive is only allowed in Classes'#000+
  '03194_E_Procedure or Function expected'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03195_W_Calling conve','ntion directive ignored: "$1"'#000+
  '03196_E_REINTRODUCE cannot be used in objects'#000+
  '03197_E_Each argument must have its own location'#000+
  '03198_E_Each argument must have an explicit location'#000+
  '03199_E_Unknown argument location'#000+
  '03200_E_32 Bit-Integer or po','inter variable expected'#000+
  '03201_E_Goto statements are not allowed between different procedures'#000+
  '03202_F_Procedure too complex, it requires too many registers'#000+
  '03203_E_Illegal expression'#000+
  '03204_E_Invalid integer expression'#000+
  '03205_E_Illegal qualifi','er'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '0319','5_W_Calling convention directive ignored: "$1"'#000+
  '03196_E_REINTRODUCE can'#039't be used in objects'#000+
  '03197_E_Each argument must have its own location'#000+
  '03198_E_Each argument must have an explicit location'#000+
  '03199_E_Unknown argument location'#000+
  '03200_E_32 B','it-Integer or pointer variable expected'#000+
  '03201_E_Goto statements aren'#039't allowed between different procedure'+
  's'#000+
  '03202_F_Procedure too complex, it requires too many registers'#000+
  '03203_E_Illegal expression'#000+
  '03204_E_Invalid integer expression'#000+
  '03205_E_','Illegal qualifier'#000+
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
  '03206_E_High range limit < low range limit'#000+
  '03207_E_Exit'#039's parameter must be the name of the procedure it is u'+
  'sed in or of a surrounding procedure'#000+
  '03208_E_Illegal assignment to for-loop variable "$1"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03209_E_Cannot declare local variable ','as EXTERNAL'#000+
  '03210_E_Procedure is already declared EXTERNAL'#000+
=======
  '03209_E_Can'#039't declare local variable as EXTERNAL'#000+
  '0321','0_E_Procedure is already declared EXTERNAL'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03209_E_Can'#039't declare local variable as EXTERNAL'#000+
  '0321','0_E_Procedure is already declared EXTERNAL'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03209_E_Can'#039't declare local variable as EXTERNAL'#000+
  '0321','0_E_Procedure is already declared EXTERNAL'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03209_E_Can'#039't declare local variable as EXTERNAL'#000+
  '0321','0_E_Procedure is already declared EXTERNAL'#000+
>>>>>>> origin/cpstrnew
  '03211_W_Implicit uses of Variants unit'#000+
  '03212_E_Class and static methods cannot be used in INTERFACES'#000+
  '03213_E_Overflow in arithmetic operation'#000+
  '03214_E_Protected or private expected'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '0','3215_E_SLICE cannot be used outside of parameter list'#000+
  '03216_E_A DISPINTERFACE cannot have a parent class'#000+
  '03217_E_A DISPINTERFACE needs a guid'#000+
  '03218_W_Overridden methods must have a related return type. This code '+
  'may crash, it depends on a D','elphi parser bug ("$2" is overridden by '+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '03215_E_SLICE can'#039,'t be used outside of parameter list'#000+
  '03216_E_A DISPINTERFACE can'#039't have a parent class'#000+
  '03217_E_A DISPINTERFACE needs a guid'#000+
  '03218_W_Overridden methods must have a related return type. This code '+
  'may crash, it depends on a Delphi parser bug ("','$2" is overridden by '+
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
  '"$1" which has another return type)'#000+
  '03219_E_Dispatch IDs must be ordinal constants'#000+
  '03220_E_The range of the array is too large'#000+
  '03221_E_The address cannot be taken of bit packed array elements and r'+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  'eco','rd fields'#000+
  '03222_E_Dynamic arrays cannot be packed'#000+
=======
  'ecord fields'#000+
  '03222_E_D','ynamic arrays cannot be packed'#000+
>>>>>>> graemeg/cpstrnew
=======
  'ecord fields'#000+
  '03222_E_D','ynamic arrays cannot be packed'#000+
>>>>>>> graemeg/cpstrnew
=======
  'ecord fields'#000+
  '03222_E_D','ynamic arrays cannot be packed'#000+
>>>>>>> graemeg/cpstrnew
=======
  'ecord fields'#000+
  '03222_E_D','ynamic arrays cannot be packed'#000+
>>>>>>> origin/cpstrnew
  '03223_E_Bit packed array elements and record fields cannot be used as '+
  'loop variables'#000+
  '03224_E_VAR, TYPE and CONST are allowed only in records, objects and c'+
  'lasses'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03225_E_This type cannot be ','a generic'#000+
  '03226_W_Don'#039't load LINEINFO unit manually, Use the -gl compiler sw'+
  'itch instead'#000+
  '03227_E_No function result type specified for function "$1"'#000+
  '03228_E_Specialization is only supported for generic types'#000+
  '03229_E_Generics cannot be used ','as parameters when specializing gene'+
  'rics'#000+
  '03230_E_Constants of objects containing a VMT are not allowed'#000+
  '03231_E_Taking the address of labels defined outside the current scope'+
  ' isn'#039't allowed'#000+
  '03233_E_Cannot initialize variables declared as exte','rnal'#000+
  '03234_E_Illegal function result type'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '03225_E_This type can'#039't be a generic'#000+
  '03226_W_Do','n'#039't load LINEINFO unit manually, Use the -gl compiler '+
  'switch instead'#000+
  '03227_E_No function result type specified for function "$1"'#000+
  '03228_E_Specialization is only supported for generic types'#000+
  '03229_E_Generics can'#039't be used as parameters when sp','ecializing g'+
=======
  '03168_W_Pr','ocedure named "$1" not found that is suitable for implemen'+
  'ting the $2.$3'#000+
  '03169_E_interface identifier expected'#000+
  '03170_E_Type "$1" can'#039't be used as array index type'#000+
  '03171_E_Con- and destructors aren'#039't allowed in interfaces'#000+
  '03172_E_Access spec','ifiers can'#039't be used in INTERFACES'#000+
  '03173_E_An interface can'#039't contain fields'#000+
  '03174_E_Can'#039't declare local procedure as EXTERNAL'#000+
  '03175_W_Some fields coming before "$1" weren'#039't initialized'#000+
  '03176_E_Some fields coming before "$1" weren'#039't initial','ized'#000+
  '03177_W_Some fields coming after "$1" weren'#039't initialized'#000+
  '03178_E_VarArgs directive (or '#039'...'#039' in MacPas) without CDecl/C'+
  'PPDecl/MWPascal and External'#000+
  '03179_E_Self must be a normal (call-by-value) parameter'#000+
  '03180_E_Interface "$1" has no ','interface identification'#000+
  '03181_E_Unknown class field or method identifier "$1"'#000+
  '03182_W_Overriding calling convention "$1" with "$2"'#000+
  '03183_E_Typed constants of the type "procedure of object" can only be '+
  'initialized with NIL'#000+
  '03184_E_Default v','alue can only be assigned to one parameter'#000+
  '03185_E_Default parameter required for "$1"'#000+
  '03186_W_Use of unsupported feature!'#000+
  '03187_H_C arrays are passed by reference'#000+
  '03188_E_C array of const must be the last argument'#000+
  '03189_H_Type "$1" redefin','ition'#000+
  '03190_W_cdecl'#039'ared functions have no high parameter'#000+
  '03191_W_cdecl'#039'ared functions do not support open strings'#000+
  '03192_E_Cannot initialize variables declared as threadvar'#000+
  '03193_E_Message directive is only allowed in Classes'#000+
  '03194_E_Proced','ure or Function expected'#000+
  '03195_W_Calling convention directive ignored: "$1"'#000+
  '03196_E_REINTRODUCE can'#039't be used in objects'#000+
  '03197_E_Each argument must have its own location'#000+
  '03198_E_Each argument must have an explicit location'#000+
  '03199_E_Unknown a','rgument location'#000+
  '03200_E_32 Bit-Integer or pointer variable expected'#000+
  '03201_E_Goto statements aren'#039't allowed between different procedure'+
  's'#000+
  '03202_F_Procedure too complex, it requires too many registers'#000+
  '03203_E_Illegal expression'#000+
  '03204_E_Invali','d integer expression'#000+
  '03205_E_Illegal qualifier'#000+
  '03206_E_High range limit < low range limit'#000+
  '03207_E_Exit'#039's parameter must be the name of the procedure it is u'+
  'sed in'#000+
  '03208_E_Illegal assignment to for-loop variable "$1"'#000+
  '03209_E_Can'#039't declare lo','cal variable as EXTERNAL'#000+
  '03210_E_Procedure is already declared EXTERNAL'#000+
  '03211_W_Implicit uses of Variants unit'#000+
  '03212_E_Class and static methods can'#039't be used in INTERFACES'#000+
  '03213_E_Overflow in arithmetic operation'#000+
  '03214_E_Protected or privat','e expected'#000+
  '03215_E_SLICE can'#039't be used outside of parameter list'#000+
  '03216_E_A DISPINTERFACE can'#039't have a parent class'#000+
  '03217_E_A DISPINTERFACE needs a guid'#000+
  '03218_W_Overridden methods must have a related return type. This code '+
  'may crash, it depe','nds on a Delphi parser bug ("$2" is overridden by '+
  '"$1" which has another return type)'#000+
  '03219_E_Dispatch IDs must be ordinal constants'#000+
  '03220_E_The range of the array is too large'#000+
=======
  '03168_W_Pr','ocedure named "$1" not found that is suitable for implemen'+
  'ting the $2.$3'#000+
  '03169_E_interface identifier expected'#000+
  '03170_E_Type "$1" can'#039't be used as array index type'#000+
  '03171_E_Con- and destructors aren'#039't allowed in interfaces'#000+
  '03172_E_Access spec','ifiers can'#039't be used in INTERFACES'#000+
  '03173_E_An interface can'#039't contain fields'#000+
  '03174_E_Can'#039't declare local procedure as EXTERNAL'#000+
  '03175_W_Some fields coming before "$1" weren'#039't initialized'#000+
  '03176_E_Some fields coming before "$1" weren'#039't initial','ized'#000+
  '03177_W_Some fields coming after "$1" weren'#039't initialized'#000+
  '03178_E_VarArgs directive (or '#039'...'#039' in MacPas) without CDecl/C'+
  'PPDecl/MWPascal and External'#000+
  '03179_E_Self must be a normal (call-by-value) parameter'#000+
  '03180_E_Interface "$1" has no ','interface identification'#000+
  '03181_E_Unknown class field or method identifier "$1"'#000+
  '03182_W_Overriding calling convention "$1" with "$2"'#000+
  '03183_E_Typed constants of the type "procedure of object" can only be '+
  'initialized with NIL'#000+
  '03184_E_Default v','alue can only be assigned to one parameter'#000+
  '03185_E_Default parameter required for "$1"'#000+
  '03186_W_Use of unsupported feature!'#000+
  '03187_H_C arrays are passed by reference'#000+
  '03188_E_C array of const must be the last argument'#000+
  '03189_H_Type "$1" redefin','ition'#000+
  '03190_W_cdecl'#039'ared functions have no high parameter'#000+
  '03191_W_cdecl'#039'ared functions do not support open strings'#000+
  '03192_E_Cannot initialize variables declared as threadvar'#000+
  '03193_E_Message directive is only allowed in Classes'#000+
  '03194_E_Proced','ure or Function expected'#000+
  '03195_W_Calling convention directive ignored: "$1"'#000+
  '03196_E_REINTRODUCE can'#039't be used in objects'#000+
  '03197_E_Each argument must have its own location'#000+
  '03198_E_Each argument must have an explicit location'#000+
  '03199_E_Unknown a','rgument location'#000+
  '03200_E_32 Bit-Integer or pointer variable expected'#000+
  '03201_E_Goto statements aren'#039't allowed between different procedure'+
  's'#000+
  '03202_F_Procedure too complex, it requires too many registers'#000+
  '03203_E_Illegal expression'#000+
  '03204_E_Invali','d integer expression'#000+
  '03205_E_Illegal qualifier'#000+
  '03206_E_High range limit < low range limit'#000+
  '03207_E_Exit'#039's parameter must be the name of the procedure it is u'+
  'sed in'#000+
  '03208_E_Illegal assignment to for-loop variable "$1"'#000+
  '03209_E_Can'#039't declare lo','cal variable as EXTERNAL'#000+
  '03210_E_Procedure is already declared EXTERNAL'#000+
  '03211_W_Implicit uses of Variants unit'#000+
  '03212_E_Class and static methods can'#039't be used in INTERFACES'#000+
  '03213_E_Overflow in arithmetic operation'#000+
  '03214_E_Protected or privat','e expected'#000+
  '03215_E_SLICE can'#039't be used outside of parameter list'#000+
  '03216_E_A DISPINTERFACE can'#039't have a parent class'#000+
  '03217_E_A DISPINTERFACE needs a guid'#000+
  '03218_W_Overridden methods must have a related return type. This code '+
  'may crash, it depe','nds on a Delphi parser bug ("$2" is overridden by '+
  '"$1" which has another return type)'#000+
  '03219_E_Dispatch IDs must be ordinal constants'#000+
  '03220_E_The range of the array is too large'#000+
>>>>>>> origin/fixes_2_2
  '03221_E_The address cannot be taken of bit packed array element','s and'+
  ' record fields'#000+
  '03222_E_Dynamic arrays cannot be packed'#000+
  '03223_E_Bit packed array elements and record fields cannot be used as '+
  'loop variables'#000+
  '03224_E_VAR and TYPE are allowed only in generics'#000+
  '03225_E_This type can'#039't be a generic'#000+
  '03226_W_','Don'#039't load LINEINFO unit manually, Use the -gl compiler '+
  'switch instead'#000+
  '03227_E_No function result type specified for function "$1"'#000+
  '03228_E_Specialization is only supported for generic types'#000+
  '03229_E_Generics can'#039't be used as parameters when ','specializing g'+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'enerics'#000+
  '03230_E_Constants of objects containing a VMT aren'#039't allowed'#000+
  '03231_E_Taking the address of labels defined outside the current scope'+
  ' isn'#039't allowed'#000+
  '03233_E_Cannot initialize variables declared as external'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '03234_E_Illegal f','unction result type'#000+
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
  '03235_E_No common type possible between "$1" and "$2"'#000+
  '03236_E_Generics without specialization cannot be used as a type for a'+
  ' variable'#000+
  '03237_W_Register list is ignored for pure assembler routines'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '032','38_E_Implements property must have class or interface type'#000+
  '03239_E_Implements-property must implement interface of correct type, '+
  'found "$1" expected "$2"'#000+
  '03240_E_Implements-property must have read specifier'#000+
  '03241_E_Implements-property must ','not have write-specifier'#000+
  '03242_E_Implements-property must not have stored-specifier'#000+
  '03243_E_Implements-property used on unimplemented interface: "$1"'#000+
  '03244_E_Floating point not supported for this target'#000+
  '03245_E_Class "$1" does not implement',' interface "$2"'#000+
  '03246_E_Type used by implements must be an interface'#000+
  '03247_E_Variables cannot be exported with a different name on this tar'+
  'get, add the name to the declaration using the "export" directive (var'+
  'iable name: $1, declared export',' name: $2)'#000+
  '03248_E_Weak external symbols are not supported for the current target'+
  #000+
  '03249_E_Forward type definition does not match'#000+
  '03250_N_Virtual method "$1" has a lower visibility ($2) than parent cl'+
  'ass $3 ($4)'#000+
  '03251_E_Fields cannot appear ','after a method or property definition, '+
=======
  '03238_E_Implements proper','ty must have class or interface type'#000+
  '03239_E_Implements-property must implement interface of correct type, '+
  'found "$1" expected "$2"'#000+
  '03240_E_Implements-property must have read specifier'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
  '03238_E_Implements proper','ty must have class or interface type'#000+
  '03239_E_Implements-property must implement interface of correct type, '+
  'found "$1" expected "$2"'#000+
  '03240_E_Implements-property must have read specifier'#000+
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  '03238_E_Implements proper','ty must have class or interface type'#000+
  '03239_E_Implements-property must implement interface of correct type, '+
  'found "$1" expected "$2"'#000+
  '03240_E_Implements-property must have read specifier'#000+
>>>>>>> origin/cpstrnew
  '03241_E_Implements-property must not have write-specifi','er'#000+
  '03242_E_Implements-property must not have stored-specifier'#000+
  '03243_E_Implements-property used on unimplemented interface: "$1"'#000+
  '03244_E_Floating point not supported for this target'#000+
  '03245_E_Class "$1" does not implement interface "$2"'#000+
  '03246_','E_Type used by implements must be an interface'#000+
  '03247_E_Variables cannot be exported with a different name on this tar'+
  'get, add the name to the declaration using the "export" directive (var'+
  'iable name: $1, declared export name: $2)'#000+
  '03248_E_Wea','k external symbols are not supported for the current targ'+
  'et'#000+
  '03249_E_Forward type definition does not match'#000+
  '03250_N_Virtual method "$1" has a lower visibility ($2) than parent cl'+
  'ass $3 ($4)'#000+
  '03251_E_Fields cannot appear after a method or prop','erty definition, '+
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
  'start a new visibility section first'#000+
  '03252_E_Parameters or result types cannot contain local type definitio'+
  'ns. Use a separate type definition in a type block.'#000+
  '03253_E_ABSTRACT and SEALED conflict'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03254','_E_Cannot create a descendant of the sealed class "$1"'#000+
  '03255_E_SEALED class cannot have an ABSTRACT method'#000+
  '03256_E_Only virtual methods can be final'#000+
  '03257_E_Final method cannot be overridden: "$1"'#000+
  '03258_E_Only one message can be used per me','thod.'#000+
  '03259_E_Invalid enumerator identifier: "$1"'#000+
  '03260_E_Enumerator identifier required'#000+
  '03261_E_Enumerator MoveNext pattern method is not valid. Method must b'+
  'e a function with the Boolean return type and no required arguments.'#000+
  '03262_E_Enum','erator Current pattern property is not valid. Property m'+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '03254_E_Cannot create a des','cendant of the sealed class "$1"'#000+
  '03255_E_SEALED class cannot have an ABSTRACT method'#000+
  '03256_E_Only virtual methods can be final'#000+
  '03257_E_Final method cannot be overridden: "$1"'#000+
  '03258_E_Only one message can be used per method.'#000+
  '03259_E_Invalid ','enumerator identifier: "$1"'#000+
  '03260_E_Enumerator identifier required'#000+
  '03261_E_Enumerator MoveNext pattern method is not valid. Method must b'+
  'e a function with the Boolean return type and no required arguments.'#000+
  '03262_E_Enumerator Current pattern',' property is not valid. Property m'+
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
  'ust have a getter.'#000+
  '03263_E_Only one enumerator MoveNext method is allowed per class/objec'+
  't'#000+
  '03264_E_Only one enumerator Current property is allowed per class/obje'+
  'ct'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '03265_E_For in loop ','cannot be used for the type "$1"'#000+
  '03266_E_Objective-C messages require their Objective-C selector name t'+
  'o be specified using the "message" directive.'#000+
  '03267_E_Objective-C does not have formal constructors nor destructors.'+
  ' Use the alloc, initX','XX and dealloc messages.'#000+
  '03268_E_Message name is too long (max. 255 characters)'#000+
  '03269_E_Objective-C message symbol name for "$1" is too long'#000+
  '03270_H_Defining a new Objective-C root class. To derive from another '+
  'root class (e.g., NSObject), ','specify it as the parent class.'#000+
  '03271_E_Objective-C classes cannot have published sections.'#000+
  '03272_F_This module requires an Objective-C mode switch to be compiled'+
  #000+
  '03273_E_Inherited methods can only be overridden in Objective-C and Ja'+
  'va, add',' "override" (inherited method defined in $1)'#000+
  '03274_H_Inherited methods can only be overridden in Objective-C and Ja'+
  'va, add "override" (inherited method defined in $1).'#000+
  '03275_E_Message name "$1" in inherited class is different from message'+
  ' n','ame "$2" in current class.'#000+
  '03276_E_It is not yet possible to make unique copies of Objective-C or'+
  ' Java types'#000+
  '03277_E_Objective-C categories and Object Pascal class helpers cannot '+
  'be used as types'#000+
  '03278_E_Categories do not override, but repl','ace methods. Use "reintr'+
  'oduce" instead.'#000+
  '03279_E_Replaced methods can only be reintroduced in Objective-C, add '+
  '"reintroduce" (replaced method defined in $1).'#000+
  '03280_H_Replaced methods can only be reintroduced in Objective-C, add '+
  '"reintroduce"',' (replaced method defined in $1).'#000+
  '03281_E_Getter for implements interface must use the target'#039's defa'+
  'ult calling convention.'#000+
  '03282_E_Typed files cannot contain reference-counted types.'#000+
  '03283_E_Operator is not overloaded: $2 "$1"'#000+
  '03284_E_Oper','ator is not overloaded: "$1" $2 "$3"'#000+
  '03285_E_Expected another $1 array elements'#000+
  '03286_E_String constant too long while ansistrings are disabled'#000+
  '03287_E_Type cannot be used as univ parameter because its size is unkn'+
  'own at compile time: "$1"'#000,
  '03288_E_Only one class constructor can be declared in class: "$1"'#000+
  '03289_E_Only one class destructor can be declared in class: "$1"'#000+
  '03290_E_Class constructors cannot have parameters'#000+
  '03291_E_Class destructors cannot have parameters'#000+
  '03292_F_Th','is construct requires the \{\$modeswitch objectivec1\} mod'+
  'e switch to be active'#000+
  '03293_E_Unicodechar/string constants cannot be converted to ansi/short'+
  'string at compile-time'#000+
  '03294_E_For-in Objective-Pascal loops require \{\$modeswitch Object','i'+
  'veC2\} to be active'#000+
  '03295_E_The compiler cannot find the NSFastEnumerationProtocol or NSFa'+
  'stEnumerationState type in the CocoaAll unit'#000+
  '03296_E_Typed constants of the type '#039'procedure is nested'#039' can '+
  'only be initialized with NIL and global pr','ocedures/functions'#000+
  '03297_F_Declaration of generic class inside another generic class is n'+
  'ot allowed'#000+
  '03298_E_Forward declaration "$1" must be resolved before a class can c'+
  'onform to or implement it'#000+
  '03299_E_Record types cannot have published s','ections'#000+
  '03300_E_Destructors are not allowed in records or helpers'#000+
  '03301_E_Class methods must be static in records'#000+
  '03302_E_Parameterless constructors are not allowed in records or recor'+
  'd/type helpers'#000+
  '03303_E_Either the result or at least one',' parameter must be of type '+
  '"$1"'#000+
  '03304_E_Type parameters may require initialization/finalization - cann'+
  'ot be used in variant records'#000+
  '03305_E_Variables being declared as external cannot be in a custom sec'+
  'tion'#000+
  '03306_E_Non-static and non-global',' variables cannot have a section di'+
  'rective'#000+
  '03307_E_"$1" is not allowed in helper types'#000+
  '03308_E_Class constructors are not allowed in helpers'#000+
  '03309_E_The use of "inherited" is not allowed in a record'#000+
  '03310_E_Type declarations are not allowed',' in local or anonymous reco'+
  'rds'#000+
  '03311_E_Duplicate implements clause for interface "$1"'#000+
  '03312_E_Interface "$1" cannot be delegated by "$2", it already has met'+
  'hod resolutions'#000+
  '03313_E_Interface "$1" cannot have method resolutions, "$2" already ','d'+
  'elegates it'#000+
  '03314_E_Invalid codepage'#000+
  '03315_E_Only fields (var-sections) and constants can be final in objec'+
  't types'#000+
  '03316_E_Final fields are currently only supported for external classes'+
  #000+
  '03317_E_Typed constants are not allowed here, only fo','rmal constants '+
  'are'#000+
  '03318_E_Constructors are not automatically inherited in the JVM; expli'+
  'citly add a constructor that calls the inherited one if you need it'#000+
  '03319_D_Parsing internally generated code: $1'#000+
  '03320_E_This language feature is not ','supported on managed VM targets'+
  #000+
  '03321_E_Calling a virtual constructor for the current instance inside '+
  'another constructor is not possible on the JVM target'#000+
  '03322_E_Overriding method "$1" cannot have a lower visibility ($2) tha'+
  'n in parent cl','ass $3 ($4)'#000+
  '03323_E_Procedure/Function declared with call option NOSTACKFRAME but '+
  'without ASSEMBLER'#000+
  '03324_E_Procedure/Function declared with call option NOSTACKFRAME but '+
  'local stack size is $1'#000+
  '03325_E_Cannot generate property getter/setter ','$1 because its name c'+
  'lashes with existing identifier $2'#000+
  '03326_W_Automatically generated property getter/setter $1 overrides th'+
  'e same-named getter/setter in class $2'#000+
  '03327_W_Case mismatch between declared property getter/setter $1 and a'+
  'utoma','tically constructed name $2, not changing declared name'#000+
  '03328_E_Constants declarations are not allowed in local or anonymous r'+
  'ecords'#000+
  '03329_E_Method declarations are not allowed in local or anonymous reco'+
  'rds'#000+
  '03330_E_Property declarations are',' not allowed in local or anonymous '+
  'records'#000+
  '03331_E_Class member declarations are not allowed in local or anonymou'+
  's records'#000+
  '03332_E_Visibility section "$1" not allowed in records'#000+
  '03333_E_Directive "$1" not allowed here'#000+
  '03334_E_Assembler bloc','ks not allowed inside generics'#000+
  '03335_E_Properties can be only static, global or inside structured typ'+
  'es'#000+
  '03336_E_Overloaded routines have the same mangled name'#000+
  '03337_E_Default values can only be specified for value, const and cons'+
  'tref parame','ters'#000+
  '03338_W_Pointer type "$1" ignored'#000+
  '03339_E_Global Generic template references static symtable'#000+
  '03340_UL_Unit $1 has been already compiled meanwhile.'#000+
  '03341_E_Explicit implementation of methods for specializations of gene'+
  'rics is not allowe','d'#000+
  '03342_E_Generic methods are not allowed in interfaces'#000+
  '03343_E_Generic methods can not be virtual'#000+
  '04000_E_Type mismatch'#000+
  '04001_E_Incompatible types: got "$1" expected "$2"'#000+
  '04002_E_Type mismatch between "$1" and "$2"'#000+
  '04003_E_Type identifier ','expected'#000+
  '04004_E_Variable identifier expected'#000+
  '04005_E_Integer expression expected, but got "$1"'#000+
  '04006_E_Boolean expression expected, but got "$1"'#000+
  '04007_E_Ordinal expression expected'#000+
=======
  '03265_E_For in loop cannot be used for the',' type "$1"'#000+
  '03266_E_Objective-C messages require their Objective-C selector name t'+
  'o be specified using the "message" directive.'#000+
  '03267_E_Objective-C does not have formal constructors nor destructors.'+
  ' Use the alloc, initXXX and dealloc message','s.'#000+
  '03268_E_Message name is too long (max. 255 characters)'#000+
  '03269_E_Objective-C message symbol name for "$1" is too long'#000+
  '03270_H_Defining a new Objective-C root class. To derive from another '+
  'root class (e.g., NSObject), specify it as the pare','nt class.'#000+
  '03271_E_Objective-C classes cannot have published sections.'#000+
  '03272_F_This module requires an Objective-C mode switch to be compiled'+
  #000+
  '03273_E_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method d','efined in $1)'#000+
  '03274_H_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method defined in $1).'#000+
  '03275_E_Message name "$1" in inherited class is different from message'+
  ' name "$2" in current class.'#000+
  '03276_E_It is',' not yet possible to make unique copies of Objective-C '+
  'types'#000+
  '03277_E_Objective-C categories cannot be used as types'#000+
  '03278_E_Categories do not override, but replace methods. Use "reintrod'+
  'uce" instead.'#000+
  '03279_E_Replaced methods can only be rei','ntroduced in Objective-C, ad'+
  'd "reintroduce" (replaced method defined in $1).'#000+
  '03280_H_Replaced methods can only be reintroduced in Objective-C, add '+
  '"reintroduce" (replaced method defined in $1).'#000+
  '03281_E_Getter for implements interface must u','se the target'#039's de'+
  'fault calling convention.'#000+
  '03282_E_Typed files cannot contain reference-counted types.'#000+
  '03283_E_Operator is not overloaded: $2 "$1"'#000+
  '03284_E_Operator is not overloaded: "$1" $2 "$3"'#000+
  '03285_E_Expected another $1 array elements'#000,
  '03286_E_String constant too long while ansistrings are disabled'#000+
  '03287_E_Type cannot be used as univ parameter because its size is unkn'+
  'own at compile time: "$1"'#000+
  '03288_E_Only one class constructor can be declared in class: "$1"'#000+
  '03289_E_Only o','ne class destructor can be declared in class: "$1"'#000+
  '03290_E_Class constructors can'#039't have parameters'#000+
  '03291_E_Class destructors can'#039't have parameters'#000+
  '03293_F_This construct requires the \{\$modeswitch objectivec1\} mode '+
  'switch to be active'#000+
  '03','294_E_For-in Objective-Pascal loops require \{\$modeswitch Objecti'+
  'veC2\} to be active'#000+
  '03295_E_The compiler cannot find the NSFastEnumerationProtocol or NSFa'+
  'stEnumerationState type in the CocoaAll unit'#000+
  '03296_E_Typed constants of the type '#039'pr','ocedure is nested'#039' ca'+
  'n only be initialized with NIL and global procedures/functions'#000+
  '03297_F_Declaration of generic class inside another generic class is n'+
  'ot allowed'#000+
  '03298_E_Forward declaration of objcprotocol "$1" must be resolved befo'+
  're an',' objcclass can conform to it'#000+
  '03299_E_Record types cannot have published sections'#000+
  '03300_E_Destructors aren'#039't allowed in records'#000+
  '03301_E_Class methods must be static in records'#000+
  '03302_E_Constructors aren'#039't allowed in records'#000+
  '03303_E_Either the',' result or at least one parameter must be of type '+
  '"$1"'#000+
  '03304_E_Type parameters may require initialization/finalization - can'#039+
  't be used in variant records'#000+
  '03305_E_Variables being declared as external cannot be in a custom sec'+
  'tion'#000+
  '03306_E_Non','-static and non-global variables cannot have a section di'+
  'rective'#000+
  '03307_E_Invalid codepage'#000+
  '03308_E_Unicodechar/string constants cannot be converted to ansi/short'+
  'string at compile-time'#000+
  '04000_E_Type mismatch'#000+
=======
  '03265_E_For in loop cannot be used for the',' type "$1"'#000+
  '03266_E_Objective-C messages require their Objective-C selector name t'+
  'o be specified using the "message" directive.'#000+
  '03267_E_Objective-C does not have formal constructors nor destructors.'+
  ' Use the alloc, initXXX and dealloc message','s.'#000+
  '03268_E_Message name is too long (max. 255 characters)'#000+
  '03269_E_Objective-C message symbol name for "$1" is too long'#000+
  '03270_H_Defining a new Objective-C root class. To derive from another '+
  'root class (e.g., NSObject), specify it as the pare','nt class.'#000+
  '03271_E_Objective-C classes cannot have published sections.'#000+
  '03272_F_This module requires an Objective-C mode switch to be compiled'+
  #000+
  '03273_E_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method d','efined in $1)'#000+
  '03274_H_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method defined in $1).'#000+
  '03275_E_Message name "$1" in inherited class is different from message'+
  ' name "$2" in current class.'#000+
  '03276_E_It is',' not yet possible to make unique copies of Objective-C '+
  'types'#000+
  '03277_E_Objective-C categories cannot be used as types'#000+
  '03278_E_Categories do not override, but replace methods. Use "reintrod'+
  'uce" instead.'#000+
  '03279_E_Replaced methods can only be rei','ntroduced in Objective-C, ad'+
  'd "reintroduce" (replaced method defined in $1).'#000+
  '03280_H_Replaced methods can only be reintroduced in Objective-C, add '+
  '"reintroduce" (replaced method defined in $1).'#000+
  '03281_E_Getter for implements interface must u','se the target'#039's de'+
  'fault calling convention.'#000+
  '03282_E_Typed files cannot contain reference-counted types.'#000+
  '03283_E_Operator is not overloaded: $2 "$1"'#000+
  '03284_E_Operator is not overloaded: "$1" $2 "$3"'#000+
  '03285_E_Expected another $1 array elements'#000,
  '03286_E_String constant too long while ansistrings are disabled'#000+
  '03287_E_Type cannot be used as univ parameter because its size is unkn'+
  'own at compile time: "$1"'#000+
  '03288_E_Only one class constructor can be declared in class: "$1"'#000+
  '03289_E_Only o','ne class destructor can be declared in class: "$1"'#000+
  '03290_E_Class constructors can'#039't have parameters'#000+
  '03291_E_Class destructors can'#039't have parameters'#000+
  '03293_F_This construct requires the \{\$modeswitch objectivec1\} mode '+
  'switch to be active'#000+
  '03','294_E_For-in Objective-Pascal loops require \{\$modeswitch Objecti'+
  'veC2\} to be active'#000+
  '03295_E_The compiler cannot find the NSFastEnumerationProtocol or NSFa'+
  'stEnumerationState type in the CocoaAll unit'#000+
  '03296_E_Typed constants of the type '#039'pr','ocedure is nested'#039' ca'+
  'n only be initialized with NIL and global procedures/functions'#000+
  '03297_F_Declaration of generic class inside another generic class is n'+
  'ot allowed'#000+
  '03298_E_Forward declaration of objcprotocol "$1" must be resolved befo'+
  're an',' objcclass can conform to it'#000+
  '03299_E_Record types cannot have published sections'#000+
  '03300_E_Destructors aren'#039't allowed in records'#000+
  '03301_E_Class methods must be static in records'#000+
  '03302_E_Constructors aren'#039't allowed in records'#000+
  '03303_E_Either the',' result or at least one parameter must be of type '+
  '"$1"'#000+
  '03304_E_Type parameters may require initialization/finalization - can'#039+
  't be used in variant records'#000+
  '03305_E_Variables being declared as external cannot be in a custom sec'+
  'tion'#000+
  '03306_E_Non','-static and non-global variables cannot have a section di'+
  'rective'#000+
  '03307_E_Invalid codepage'#000+
  '03308_E_Unicodechar/string constants cannot be converted to ansi/short'+
  'string at compile-time'#000+
  '04000_E_Type mismatch'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03265_E_For in loop cannot be used for the',' type "$1"'#000+
  '03266_E_Objective-C messages require their Objective-C selector name t'+
  'o be specified using the "message" directive.'#000+
  '03267_E_Objective-C does not have formal constructors nor destructors.'+
  ' Use the alloc, initXXX and dealloc message','s.'#000+
  '03268_E_Message name is too long (max. 255 characters)'#000+
  '03269_E_Objective-C message symbol name for "$1" is too long'#000+
  '03270_H_Defining a new Objective-C root class. To derive from another '+
  'root class (e.g., NSObject), specify it as the pare','nt class.'#000+
  '03271_E_Objective-C classes cannot have published sections.'#000+
  '03272_F_This module requires an Objective-C mode switch to be compiled'+
  #000+
  '03273_E_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method d','efined in $1)'#000+
  '03274_H_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method defined in $1).'#000+
  '03275_E_Message name "$1" in inherited class is different from message'+
  ' name "$2" in current class.'#000+
  '03276_E_It is',' not yet possible to make unique copies of Objective-C '+
  'types'#000+
  '03277_E_Objective-C categories cannot be used as types'#000+
  '03278_E_Categories do not override, but replace methods. Use "reintrod'+
  'uce" instead.'#000+
  '03279_E_Replaced methods can only be rei','ntroduced in Objective-C, ad'+
  'd "reintroduce" (replaced method defined in $1).'#000+
  '03280_H_Replaced methods can only be reintroduced in Objective-C, add '+
  '"reintroduce" (replaced method defined in $1).'#000+
  '03281_E_Getter for implements interface must u','se the target'#039's de'+
  'fault calling convention.'#000+
  '03282_E_Typed files cannot contain reference-counted types.'#000+
  '03283_E_Operator is not overloaded: $2 "$1"'#000+
  '03284_E_Operator is not overloaded: "$1" $2 "$3"'#000+
  '03285_E_Expected another $1 array elements'#000,
  '03286_E_String constant too long while ansistrings are disabled'#000+
  '03287_E_Type cannot be used as univ parameter because its size is unkn'+
  'own at compile time: "$1"'#000+
  '03288_E_Only one class constructor can be declared in class: "$1"'#000+
  '03289_E_Only o','ne class destructor can be declared in class: "$1"'#000+
  '03290_E_Class constructors can'#039't have parameters'#000+
  '03291_E_Class destructors can'#039't have parameters'#000+
  '03293_F_This construct requires the \{\$modeswitch objectivec1\} mode '+
  'switch to be active'#000+
  '03','294_E_For-in Objective-Pascal loops require \{\$modeswitch Objecti'+
  'veC2\} to be active'#000+
  '03295_E_The compiler cannot find the NSFastEnumerationProtocol or NSFa'+
  'stEnumerationState type in the CocoaAll unit'#000+
  '03296_E_Typed constants of the type '#039'pr','ocedure is nested'#039' ca'+
  'n only be initialized with NIL and global procedures/functions'#000+
  '03297_F_Declaration of generic class inside another generic class is n'+
  'ot allowed'#000+
  '03298_E_Forward declaration of objcprotocol "$1" must be resolved befo'+
  're an',' objcclass can conform to it'#000+
  '03299_E_Record types cannot have published sections'#000+
  '03300_E_Destructors aren'#039't allowed in records'#000+
  '03301_E_Class methods must be static in records'#000+
  '03302_E_Constructors aren'#039't allowed in records'#000+
  '03303_E_Either the',' result or at least one parameter must be of type '+
  '"$1"'#000+
  '03304_E_Type parameters may require initialization/finalization - can'#039+
  't be used in variant records'#000+
  '03305_E_Variables being declared as external cannot be in a custom sec'+
  'tion'#000+
  '03306_E_Non','-static and non-global variables cannot have a section di'+
  'rective'#000+
  '03307_E_Invalid codepage'#000+
  '03308_E_Unicodechar/string constants cannot be converted to ansi/short'+
  'string at compile-time'#000+
  '04000_E_Type mismatch'#000+
>>>>>>> graemeg/cpstrnew
=======
  '03265_E_For in loop cannot be used for the',' type "$1"'#000+
  '03266_E_Objective-C messages require their Objective-C selector name t'+
  'o be specified using the "message" directive.'#000+
  '03267_E_Objective-C does not have formal constructors nor destructors.'+
  ' Use the alloc, initXXX and dealloc message','s.'#000+
  '03268_E_Message name is too long (max. 255 characters)'#000+
  '03269_E_Objective-C message symbol name for "$1" is too long'#000+
  '03270_H_Defining a new Objective-C root class. To derive from another '+
  'root class (e.g., NSObject), specify it as the pare','nt class.'#000+
  '03271_E_Objective-C classes cannot have published sections.'#000+
  '03272_F_This module requires an Objective-C mode switch to be compiled'+
  #000+
  '03273_E_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method d','efined in $1)'#000+
  '03274_H_Inherited methods can only be overridden in Objective-C, add "'+
  'override" (inherited method defined in $1).'#000+
  '03275_E_Message name "$1" in inherited class is different from message'+
  ' name "$2" in current class.'#000+
  '03276_E_It is',' not yet possible to make unique copies of Objective-C '+
  'types'#000+
  '03277_E_Objective-C categories cannot be used as types'#000+
  '03278_E_Categories do not override, but replace methods. Use "reintrod'+
  'uce" instead.'#000+
  '03279_E_Replaced methods can only be rei','ntroduced in Objective-C, ad'+
  'd "reintroduce" (replaced method defined in $1).'#000+
  '03280_H_Replaced methods can only be reintroduced in Objective-C, add '+
  '"reintroduce" (replaced method defined in $1).'#000+
  '03281_E_Getter for implements interface must u','se the target'#039's de'+
  'fault calling convention.'#000+
  '03282_E_Typed files cannot contain reference-counted types.'#000+
  '03283_E_Operator is not overloaded: $2 "$1"'#000+
  '03284_E_Operator is not overloaded: "$1" $2 "$3"'#000+
  '03285_E_Expected another $1 array elements'#000,
  '03286_E_String constant too long while ansistrings are disabled'#000+
  '03287_E_Type cannot be used as univ parameter because its size is unkn'+
  'own at compile time: "$1"'#000+
  '03288_E_Only one class constructor can be declared in class: "$1"'#000+
  '03289_E_Only o','ne class destructor can be declared in class: "$1"'#000+
  '03290_E_Class constructors can'#039't have parameters'#000+
  '03291_E_Class destructors can'#039't have parameters'#000+
  '03293_F_This construct requires the \{\$modeswitch objectivec1\} mode '+
  'switch to be active'#000+
  '03','294_E_For-in Objective-Pascal loops require \{\$modeswitch Objecti'+
  'veC2\} to be active'#000+
  '03295_E_The compiler cannot find the NSFastEnumerationProtocol or NSFa'+
  'stEnumerationState type in the CocoaAll unit'#000+
  '03296_E_Typed constants of the type '#039'pr','ocedure is nested'#039' ca'+
  'n only be initialized with NIL and global procedures/functions'#000+
  '03297_F_Declaration of generic class inside another generic class is n'+
  'ot allowed'#000+
  '03298_E_Forward declaration of objcprotocol "$1" must be resolved befo'+
  're an',' objcclass can conform to it'#000+
  '03299_E_Record types cannot have published sections'#000+
  '03300_E_Destructors aren'#039't allowed in records'#000+
  '03301_E_Class methods must be static in records'#000+
  '03302_E_Constructors aren'#039't allowed in records'#000+
  '03303_E_Either the',' result or at least one parameter must be of type '+
  '"$1"'#000+
  '03304_E_Type parameters may require initialization/finalization - can'#039+
  't be used in variant records'#000+
  '03305_E_Variables being declared as external cannot be in a custom sec'+
  'tion'#000+
  '03306_E_Non','-static and non-global variables cannot have a section di'+
  'rective'#000+
  '03307_E_Invalid codepage'#000+
  '03308_E_Unicodechar/string constants cannot be converted to ansi/short'+
  'string at compile-time'#000+
  '04000_E_Type mismatch'#000+
>>>>>>> origin/cpstrnew
  '04001_E_Incompatible types: got "$1','" expected "$2"'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '03234_E_Illegal',' function result type'#000+
  '03235_E_No common type possible between "$1" and "$2"'#000+
  '03236_E_Generics without specialization can not be used as a type for '+
  'a variable'#000+
  '03237_W_Register list is ignored for pure assembler routines'#000+
  '03247_E_Variables cann','ot be exported with a different name on this t'+
  'arget, add the name to the declaration using the "export" directive (v'+
  'ariable name: $1, declared export name: $2)'#000+
  '03249_E_Forward type definition does not match'#000+
  '04000_E_Type mismatch'#000+
  '04001_E_Inc','ompatible types: got "$1" expected "$2"'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '04002_E_Type mismatch between "$1" and "$2"'#000+
  '04003_E_Type identifier expected'#000+
  '04004_E_Variable identifier expected'#000+
  '04005_E_Integer expression expected, but got "$1"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04006_E_Boolean expression expected, but got "$1"'#000+
  '04007_E_Or','dinal expression expected'#000+
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
  '04006_E_Boolean expression expected,',' but got "$1"'#000+
  '04007_E_Ordinal expression expected'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '04006_E_Boolean expression expected,',' but got "$1"'#000+
  '04007_E_Ordinal expression expected'#000+
>>>>>>> origin/fixes_2_2
  '04008_E_pointer type expected, but got "$1"'#000+
  '04009_E_class ','type expected, but got "$1"'#000+
  '04011_E_Can'#039't evaluate constant expression'#000+
  '04012_E_Set elements are not compatible'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04013_E_Operation not implemented for sets'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '04014_W_Automatic type conversion from floating type to COMP which is '+
  'an integer type'#000,
  '04015_H_use DIV instead to get an integer result'#000+
  '04016_E_String types have to match exactly in $V+ mode'#000+
  '04017_E_succ or pred on enums with assignments not possible'#000+
  '04018_E_Can'#039't read or write variables of this type'#000+
  '04019_E_Can'#039't use readln ','or writeln on typed file'#000+
  '04020_E_Can'#039't use read or write on untyped file.'#000+
  '04021_E_Type conflict between set elements'#000+
  '04022_W_lo/hi(dword/qword) returns the upper/lower word/dword'#000+
=======
  '04','014_W_Automatic type conversion from floating type to COMP which i'+
  's an integer type'#000+
  '04015_H_use DIV instead to get an integer result'#000+
  '04016_E_String types have to match exactly in $V+ mode'#000+
=======
  '04','014_W_Automatic type conversion from floating type to COMP which i'+
  's an integer type'#000+
  '04015_H_use DIV instead to get an integer result'#000+
  '04016_E_String types have to match exactly in $V+ mode'#000+
>>>>>>> graemeg/cpstrnew
=======
  '04','014_W_Automatic type conversion from floating type to COMP which i'+
  's an integer type'#000+
=======
  '04013_E_Operation not',' implemented for sets'#000+
  '04014_W_Automatic type conversion from floating type to COMP which is '+
  'an integer type'#000+
>>>>>>> origin/fixes_2_2
  '04015_H_use DIV instead to get an integer result'#000+
  '04016_E_String types have to match exactly in $V+ mode'#000+
>>>>>>> graemeg/cpstrnew
=======
  '04','014_W_Automatic type conversion from floating type to COMP which i'+
  's an integer type'#000+
  '04015_H_use DIV instead to get an integer result'#000+
  '04016_E_String types have to match exactly in $V+ mode'#000+
>>>>>>> origin/cpstrnew
  '04017_E_succ or pred on enums with assignments not p','ossible'#000+
=======
  '04013_E_Operation not',' implemented for sets'#000+
  '04014_W_Automatic type conversion from floating type to COMP which is '+
  'an integer type'#000+
  '04015_H_use DIV instead to get an integer result'#000+
  '04016_E_string types doesn'#039't match, because of $V+ mode'#000+
  '04017_E_succ or pred on enu','ms with assignments not possible'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '04018_E_Can'#039't read or write variables of this type'#000+
  '04019_E_Can'#039't use readln or writeln on typed file'#000+
  '04020_E_Can'#039't use read or write on untyped file.'#000+
  '04021_E_Type conflict between set elements'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04022_W_lo/hi(dword/qword) returns the ','upper/lower word/dword'#000+
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
  '04022_W_lo/hi(','dword/qword) returns the upper/lower word/dword'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '04022_W_lo/hi(','dword/qword) returns the upper/lower word/dword'#000+
>>>>>>> origin/fixes_2_2
  '04023_E_Integer or real expression expected'#000+
  '04024_E_Wrong typ','e "$1" in array constructor'#000+
  '04025_E_Incompatible type for arg no. $1: Got "$2", expected "$3"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '04026_E_Method (variable) and Procedure (variable) are not compatible'#000+
  '04027_E_Illegal constant passed to internal math function'#000+
  '04028_E_Can'#039't take',' the address of constant expressions'#000+
  '04029_E_Argument cannot be assigned to'#000+
  '04030_E_Can'#039't assign local procedure/function to procedure variabl'+
  'e'#000+
  '04031_E_Can'#039't assign values to an address'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '04026_E_Method (variable) and Procedure (variable) are not com','patibl'+
=======
  '04026_E_Method (variable) and Procedu','re (variable) are not compatibl'+
>>>>>>> graemeg/fixes_2_2
=======
  '04026_E_Method (variable) and Procedu','re (variable) are not compatibl'+
>>>>>>> origin/fixes_2_2
  'e'#000+
  '04027_E_Illegal constant passed to internal math function'#000+
  '04028_E_Can'#039't take the address of constant expressions'#000+
  '04029_E_Argument can'#039't be assigned to'#000+
  '04030_E_Can'#039't assign local procedure/function to procedure variabl'+
<<<<<<< HEAD
<<<<<<< HEAD
  'e'#000+
  '04031_E_Can'#039't',' assign values to an address'#000+
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
  'e'#000,
  '04031_E_Can'#039't assign values to an address'#000+
>>>>>>> graemeg/fixes_2_2
=======
  'e'#000,
  '04031_E_Can'#039't assign values to an address'#000+
>>>>>>> origin/fixes_2_2
  '04032_E_Can'#039't assign values to const variable'#000+
  '04033_E_','Array type required'#000+
  '04034_E_interface type expected, but got "$1"'#000+
<<<<<<< HEAD
  '04035_H_Mixing signed expressions and longwords gives a 64bit result'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '04036_W_Mixing signed expressions and cardinals here may cause a range'+
  ' check error'#000+
  '04037_E_Typecast has d','ifferent size ($1 -> $2) in assignment'#000+
  '04038_E_enums with assignments cannot be used as array index'#000+
  '04039_E_Class or Object types "$1" and "$2" are not related'#000+
  '04040_W_Class types "$1" and "$2" are not related'#000+
  '04041_E_Class or interface typ','e expected, but got "$1"'#000+
  '04042_E_Type "$1" is not completely defined'#000+
  '04043_W_String literal has more characters than short string length'#000+
  '04044_W_Comparison might be always false due to range of constant and '+
  'expression'#000+
  '04045_W_Comparison mig','ht be always true due to range of constant and'+
  ' expression'#000+
  '04046_W_Constructing a class "$1" with abstract method "$2"'#000+
  '04047_H_The left operand of the IN operator should be byte sized'#000+
  '04048_W_Type size mismatch, possible loss of data / range',' check erro'+
  'r'#000+
  '04049_H_Type size mismatch, possible loss of data / range check error'#000+
  '04050_E_The address of an abstract method cannot be taken'#000+
  '04051_E_Assignments to formal parameters and open arrays are not possi'+
  'ble'#000+
  '04052_E_Constant Expressi','on expected'#000+
  '04053_E_Operation "$1" not supported for types "$2" and "$3"'#000+
  '04054_E_Illegal type conversion: "$1" to "$2"'#000+
  '04055_H_Conversion between ordinals and pointers is not portable'#000+
  '04056_W_Conversion between ordinals and pointers is not ','portable'#000+
  '04057_E_Can'#039't determine which overloaded function to call'#000+
  '04058_E_Illegal counter variable'#000+
  '04059_W_Converting constant real value to double for C variable argume'+
  'nt, add explicit typecast to prevent this.'#000+
  '04060_E_Class or COM interf','ace type expected, but got "$1"'#000+
  '04061_E_Constant packed arrays are not yet supported'#000+
  '04062_E_Incompatible type for arg no. $1: Got "$2" expected "(Bit)Pack'+
  'ed Array"'#000+
  '04063_E_Incompatible type for arg no. $1: Got "$2" expected "(not pack'+
  'ed) A','rray"'#000+
  '04064_E_Elements of packed arrays cannot be of a type which need to be'+
  ' initialised'#000+
  '04065_E_Constant packed records and objects are not yet supported'#000+
  '04066_W_Arithmetic "$1" on untyped pointer is unportable to {$T+}, sug'+
  'gest typecast'#000+
  '0','4076_E_Can'#039't take address of a subroutine marked as local'#000+
  '04077_E_Can'#039't export subroutine marked as local from a unit'#000+
  '04078_E_Type is not automatable: "$1"'#000+
  '04079_H_Converting the operands to "$1" before doing the add could pre'+
  'vent overflow ','errors.'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '04036_W_Mixing signed ','expressions and cardinals here may cause a ran'+
  'ge check error'#000+
  '04037_E_Typecast has different size ($1 -> $2) in assignment'#000+
  '04038_E_enums with assignments can'#039't be used as array index'#000+
  '04039_E_Class or Object types "$1" and "$2" are not relate','d'#000+
=======
  '04035_W_Mixing signed expressions and longwords gives a 64bit result'#000+
  '04036_W_M','ixing signed expressions and cardinals here may cause a ran'+
  'ge check error'#000+
  '04037_E_Typecast has different size ($1 -> $2) in assignment'#000+
  '04038_E_enums with assignments can'#039't be used as array index'#000+
  '04039_E_Class or Object types "$1" and "$2" a','re not related'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '04040_W_Class types "$1" and "$2" are not related'#000+
  '04041_E_Class or interface type expected, but got "$1"'#000+
  '04042_E_Type "$1" is not completely defined'#000+
  '04043_W_String literal has more characters than short string length'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04044_W_Comparison is',' always false due to range of values'#000+
=======
  '04044_W_','Comparison is always false due to range of values'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '04044_W_','Comparison is always false due to range of values'#000+
>>>>>>> origin/fixes_2_2
  '04045_W_Comparison is always true due to range of values'#000+
  '04046_W_Constructing a class "$1" with abstract method "$2"'#000+
  '04047_H_The left operand of the IN operator should be byte sized'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04048_W_Type size mis','match, possible loss of data / range check erro'+
  'r'#000+
  '04049_H_Type size mismatch, possible loss of data / range check error'#000+
  '04050_E_The address of an abstract method can'#039't be taken'#000+
  '04051_E_Assignments to formal parameters and open arrays are not',' pos'+
=======
  '04048_W_Type',' size mismatch, possible loss of data / range check erro'+
  'r'#000+
  '04049_H_Type size mismatch, possible loss of data / range check error'#000+
  '04050_E_The address of an abstract method can'#039't be taken'#000+
  '04051_E_Assignments to formal parameters and open array','s are not pos'+
>>>>>>> graemeg/fixes_2_2
=======
  '04048_W_Type',' size mismatch, possible loss of data / range check erro'+
  'r'#000+
  '04049_H_Type size mismatch, possible loss of data / range check error'#000+
  '04050_E_The address of an abstract method can'#039't be taken'#000+
  '04051_E_Assignments to formal parameters and open array','s are not pos'+
>>>>>>> origin/fixes_2_2
  'sible'#000+
  '04052_E_Constant Expression expected'#000+
  '04053_E_Operation "$1" not supported for types "$2" and "$3"'#000+
  '04054_E_Illegal type conversion: "$1" to "$2"'#000+
  '04055_H_Conversion between ordinals and pointers is not portable'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04056_W_Conversion be','tween ordinals and pointers is not portable'#000+
  '04057_E_Can'#039't determine which overloaded function to call'#000+
  '04058_E_Illegal counter variable'#000+
  '04059_W_Converting constant real value to double for C variable argume'+
  'nt, add explicit typecast to preven','t this.'#000+
=======
  '04056_W_Conv','ersion between ordinals and pointers is not portable'#000+
  '04057_E_Can'#039't determine which overloaded function to call'#000+
  '04058_E_Illegal counter variable'#000+
  '04059_W_Converting constant real value to double for C variable argume'+
  'nt, add explicit typecast ','to prevent this.'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '04056_W_Conv','ersion between ordinals and pointers is not portable'#000+
  '04057_E_Can'#039't determine which overloaded function to call'#000+
  '04058_E_Illegal counter variable'#000+
  '04059_W_Converting constant real value to double for C variable argume'+
  'nt, add explicit typecast ','to prevent this.'#000+
>>>>>>> origin/fixes_2_2
  '04060_E_Class or COM interface type expected, but got "$1"'#000+
  '04061_E_Constant packed arrays are not yet supported'#000+
  '04062_E_Incompatible type for arg no. $1: Got "$2" expected "(Bit)Pack'+
  'ed Array"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04063_E_Incompatible type for arg no. $1',': Got "$2" expected "(not pa'+
=======
  '04063_E_Incompatible type for a','rg no. $1: Got "$2" expected "(not pa'+
>>>>>>> graemeg/fixes_2_2
=======
  '04063_E_Incompatible type for a','rg no. $1: Got "$2" expected "(not pa'+
>>>>>>> origin/fixes_2_2
  'cked) Array"'#000+
  '04064_E_Elements of packed arrays cannot be of a type which need to be'+
  ' initialised'#000+
  '04065_E_Constant packed records and objects are not yet supported'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04066_W_Arithmetic "$1" on untyped pointer is unpo','rtable to {$T+}, s'+
=======
  '04066_W_Arithmetic "$1" on untyped pointe','r is unportable to {$T+}, s'+
>>>>>>> graemeg/fixes_2_2
=======
  '04066_W_Arithmetic "$1" on untyped pointe','r is unportable to {$T+}, s'+
>>>>>>> origin/fixes_2_2
  'uggest typecast'#000+
  '04076_E_Can'#039't take address of a subroutine marked as local'#000+
  '04077_E_Can'#039't export subroutine marked as local from a unit'#000+
  '04078_E_Type is not automatable: "$1"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '04079_H_Converting the operands to "$1" before do','ing the add could p'+
  'revent overflow errors.'#000+
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
  '04080_H_Converting the operands to "$1" before doing the subtract coul'+
  'd prevent overflow errors.'#000+
  '04081_H_Converting the operands to "$1" before doing the multiply coul'+
  'd prevent overflow errors.'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '04082_W_Converting pointers to signed ','integers may result in wrong c'+
  'omparison results and range errors, use an unsigned type instead.'#000+
  '04083_E_Interface type $1 has no valid GUID'#000+
  '04084_E_Invalid selector name "$1"'#000+
  '04085_E_Expected Objective-C method, but got $1'#000+
  '04086_E_Expected ','Objective-C method or constant method name'#000+
=======
  '040','82_W_Converting pointers to signed integers may result in wrong c'+
  'omparison results and range errors, use an unsigned type instead.'#000+
  '04083_E_Interface type $1 has no valid GUID'#000+
  '04084_E_Invalid selector name "$1"'#000+
  '04085_E_Expected Objective-C m','ethod, but got $1'#000+
  '04086_E_Expected Objective-C method or constant method name'#000+
>>>>>>> graemeg/cpstrnew
=======
  '040','82_W_Converting pointers to signed integers may result in wrong c'+
  'omparison results and range errors, use an unsigned type instead.'#000+
  '04083_E_Interface type $1 has no valid GUID'#000+
  '04084_E_Invalid selector name "$1"'#000+
  '04085_E_Expected Objective-C m','ethod, but got $1'#000+
  '04086_E_Expected Objective-C method or constant method name'#000+
>>>>>>> graemeg/cpstrnew
=======
  '040','82_W_Converting pointers to signed integers may result in wrong c'+
  'omparison results and range errors, use an unsigned type instead.'#000+
  '04083_E_Interface type $1 has no valid GUID'#000+
  '04084_E_Invalid selector name "$1"'#000+
  '04085_E_Expected Objective-C m','ethod, but got $1'#000+
  '04086_E_Expected Objective-C method or constant method name'#000+
>>>>>>> graemeg/cpstrnew
=======
  '040','82_W_Converting pointers to signed integers may result in wrong c'+
  'omparison results and range errors, use an unsigned type instead.'#000+
  '04083_E_Interface type $1 has no valid GUID'#000+
  '04084_E_Invalid selector name "$1"'#000+
  '04085_E_Expected Objective-C m','ethod, but got $1'#000+
  '04086_E_Expected Objective-C method or constant method name'#000+
>>>>>>> origin/cpstrnew
  '04087_E_No type info available for this type'#000+
  '04088_E_Ordinal or string expression expected'#000+
  '04089_E_String expression expected'#000+
  '04090_W_Converting 0 to NIL'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '04091_E_Objective-C protocol type expected,',' but got "$1"'#000+
  '04092_E_The type "$1" is not supported for interaction with the Object'+
  'ive-C and the blocks runtime.'#000+
  '04093_E_Class or objcclass type expected, but got "$1"'#000+
  '04094_E_Objcclass type expected'#000+
  '04095_W_Coerced univ parameter type in ','procedural variable may cause'+
  ' crash or memory corruption: $1 to $2'#000+
  '04096_E_Type parameters of specializations of generics cannot referenc'+
  'e the currently specialized type'#000+
  '04097_E_Type parameters are not allowed on non-generic class/record/ob',
  'ject procedure or function'#000+
  '04098_E_Generic declaration of "$1" differs from previous declaration'#000+
  '04099_E_Helper type expected'#000+
  '04100_E_Record type expected'#000+
  '04101_E_Derived class helper must extend a subclass of "$1" or the cla'+
  'ss itself'#000+
  '04102','_E_Derived record or type helper must extend "$1"'#000+
  '04103_E_Invalid assignment, procedures return no value'#000+
  '04104_W_Implicit string type conversion from "$1" to "$2"'#000+
  '04105_W_Implicit string type conversion with potential data loss from '+
  '"$1" to',' "$2"'#000+
  '04106_-W_Explicit string typecast from "$1" to "$2"'#000+
  '04107_-W_Explicit string typecast with potential data loss from "$1" t'+
  'o "$2"'#000+
  '04108_W_Unicode constant cast with potential data loss'#000+
  '04109_E_range check error while evaluating constan','ts ($1 must be bet'+
  'ween $2 and $3)'#000+
  '04110_W_range check error while evaluating constants ($1 must be betwe'+
  'en $2 and $3)'#000+
  '04111_E_This type is not supported for the Default() intrinsic'#000+
  '04112_E_JVM virtual class methods cannot be static'#000+
  '04113_E_','Final (class) fields can only be assigned in their class'#039+
  ' (class) constructor'#000+
  '04114_E_It is not possible to typecast untyped parameters on managed p'+
  'latforms, simply assign a value to them instead.'#000+
  '04115_E_The assignment side of an expressio','n cannot be typecasted to'+
  ' a supertype on managed platforms'#000+
  '04116_-W_The interface method "$1" raises the visibility of "$2" to pu'+
  'blic when accessed via an interface instance'#000+
  '04117_E_The interface method "$1" has a higher visibility (public)',' t'+
  'han "$2"'#000+
  '04118_E_TYPEOF can only be used on object types with VMT'#000+
  '04119_E_It is not possible to define a default value for a parameter o'+
  'f type "$1"'#000+
  '04120_E_Type "$1" cannot be extended by a type helper'#000+
  '04121_E_Procedure or function must b','e far in order to allow taking i'+
  'ts address: "$1"'#000+
  '04122_W_Creating an instance of abstract class "$1"'#000+
  '04123_E_Subroutine references cannot be declared as "of object" or "is'+
  ' nested", they can always refer to any kind of subroutine'#000+
  '05000_E_Ide','ntifier not found "$1"'#000+
  '05001_F_Internal Error in SymTableStack()'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '04079_H_Converting the operands to "$1" ','before doing the add could p'+
  'revent overflow errors.'#000+
  '04080_H_Converting the operands to "$1" before doing the subtract coul'+
  'd prevent overflow errors.'#000+
  '04081_H_Converting the operands to "$1" before doing the multiply coul'+
  'd prevent overflow er','rors.'#000+
  '04082_W_Converting pointers to signed integers may result in wrong com'+
  'parison results and range errors, use an unsigned type instead.'#000+
  '04083_E_Interface type $1 has no valid GUID'#000+
  '05000_E_Identifier not found "$1"'#000+
  '05001_F_Internal Error',' in SymTableStack()'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '05002_E_Duplicate identifier "$1"'#000+
  '05003_H_Identifier already defined in $1 at line $2'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '04091_E_','Objective-C protocol type expected, but got "$1"'#000+
  '04092_E_The type "$1" is not supported for interaction with the Object'+
  'ive-C runtime.'#000+
  '04093_E_Class or objcclass type expected, but got "$1"'#000+
  '04094_E_Objcclass type expected'#000+
  '04095_W_Coerced uni','v parameter type in procedural variable may cause'+
  ' crash or memory corruption: $1 to $2'#000+
  '04096_E_Type parameters of specializations of generics cannot referenc'+
  'e the currently specialized type'#000+
  '04097_E_Type parameters are not allowed on non-gen','eric class/record/'+
  'object procedure or function'#000+
  '04098_E_Generic declaration of "$1" differs from previous declaration'#000+
  '05000_E_Identifier not found "$1"'#000+
  '05001_F_Internal Error in SymTableStack()'#000+
  '05002_E_Duplicate identifier "$1"'#000+
  '05003_H_Ident','ifier already defined in $1 at line $2'#000+
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
  '05004_E_Unknown identifier "$1"'#000+
  '05005_E_Forward declaration not solved "$1"'#000+
<<<<<<< HEAD
  '05007_E_Error',' in type definition'#000+
  '05009_E_Forward type not resolved "$1"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  '05007_E_Error in type definition'#000+
  '05009_E_Forward type not ','resolved "$1"'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '05010_E_Only static variables can be used in static methods or outside'+
  ' methods'#000+
  '05012_E_Record or object or class type expected'#000+
  '05013_E_Instances of classes or objects with an abstra','ct method are '+
  'not allowed'#000+
  '05014_W_Label not defined "$1"'#000+
  '05015_E_Label used but not defined "$1"'#000+
  '05016_E_Illegal label declaration'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '05010_E_Only static variables can be used in static m','ethods or outsi'+
  'de methods'#000+
  '05012_F_record or class type expected'#000+
  '05013_E_Instances of classes or objects with an abstract method are no'+
  't allowed'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '05014_W_Label not defined "$1"'#000+
  '05015_E_Label used but not defined "$1"'#000+
  '05016_E_Illegal label dec','laration'#000+
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
=======
>>>>>>> origin/fixes_2_2
  '05014_W_Label not defined "$1','"'#000+
  '05015_E_Label used but not defined "$1"'#000+
  '05016_E_Illegal label declaration'#000+
>>>>>>> graemeg/fixes_2_2
  '05017_E_GOTO and LABEL are not supported (use switch -Sg)'#000+
  '05018_E_Label not found'#000+
  '05019_E_identifier isn'#039't a ','label'#000+
  '05020_E_label already defined'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '05021_E_illegal type declaration of set elements'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  '05021_E_illegal typ','e declaration of set elements'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '05021_E_illegal typ','e declaration of set elements'#000+
>>>>>>> origin/fixes_2_2
  '05022_E_Forward class definition not resolved "$1"'#000+
=======
  '05022_E_Forward class definition not ','resolved "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05022_E_Forward class definition not ','resolved "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05022_E_Forward class definition not ','resolved "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05022_E_Forward class definition not ','resolved "$1"'#000+
>>>>>>> origin/cpstrnew
  '05023_H_Unit "$1" not used in $2'#000+
  '05024_H_Parameter "$1" not used'#000+
  '05025_N_Local variable "$1" not used'#000+
<<<<<<< HEAD
  '05','026_H_Value parameter "$1" is assigned but never used'#000+
  '05027_N_Local variable "$1" is assigned but never used'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  '05026_H_Value parameter "$1" is assigned but never used'#000+
  '0','5027_N_Local variable "$1" is assigned but never used'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '05028_H_Local $1 "$2" is not used'#000+
=======
  '05028_H_Local',' $1 "$2" is not used'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05028_H_Local',' $1 "$2" is not used'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05028_H_Local',' $1 "$2" is not used'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05028_H_Local',' $1 "$2" is not used'#000+
>>>>>>> origin/cpstrnew
  '05029_N_Private field "$1.$2" is never used'#000+
  '05030_N_Private field "$1.$2" is assigned but never u','sed'#000+
  '05031_N_Private method "$1.$2" never used'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '05032_E_Set type expected'#000+
=======
  '05032_E_S','et type expected'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '05032_E_S','et type expected'#000+
>>>>>>> origin/fixes_2_2
  '05033_W_Function result does not seem to be set'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '05034_W_Type "$1" is not aligned correctly in current record for C'#000+
=======
  '05','034_W_Type "$1" is not aligned correctly in current record for C'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05','034_W_Type "$1" is not aligned correctly in current record for C'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05','034_W_Type "$1" is not aligned correctly in current record for C'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05','034_W_Type "$1" is not aligned correctly in current record for C'#000+
>>>>>>> origin/cpstrnew
  '05035_E_Unknown record field identifier "$1"'#000+
<<<<<<< HEAD
  '05036_W_','Local variable "$1" does not seem to be initialized'#000+
  '05037_W_Variable "$1" does not seem to be initialized'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  '05036_W_Local variable "$1" does not seem to be initialized'#000+
  '050','37_W_Variable "$1" does not seem to be initialized'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '05038_E_identifier idents no member "$1"'#000+
=======
  '05038_E_identifi','er idents no member "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05038_E_identifi','er idents no member "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05038_E_identifi','er idents no member "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05038_E_identifi','er idents no member "$1"'#000+
>>>>>>> origin/cpstrnew
  '05039_H_Found declaration: $1'#000+
  '05040_E_Data element too large'#000+
<<<<<<< HEAD
  '05042_E_No matching implementati','on for interface method "$1" found'#000+
  '05043_W_Symbol "$1" is deprecated'#000+
=======
  '05042_E_No matching implementation for interface method "$1" found'#000+
  '05043_W_Symbol "$1" ','is deprecated'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '05044_W_Symbol "$1" is not portable'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '05055_W_Symbol "$1" is not implemented'#000+
=======
  '05055_W_Symbol "$','1" is not implemented'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05055_W_Symbol "$','1" is not implemented'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05055_W_Symbol "$','1" is not implemented'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05055_W_Symbol "$','1" is not implemented'#000+
>>>>>>> origin/cpstrnew
  '05056_E_Can'#039't create unique type from this type'#000+
<<<<<<< HEAD
  '05057_H_Local variable "$1" does not seem to be ','initialized'#000+
  '05058_H_Variable "$1" does not seem to be initialized'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '05059_W_Function result variable does not seem to be initialized'#000+
  '05060_H_Function result variable does not seem to be initialized'#000+
  '05061_W_Variable "$1" read but nowhere assig','ned'#000+
  '05062_H_Found abstract method: $1'#000+
  '05063_W_Symbol "$1" is experimental'#000+
  '05064_W_Forward declaration "$1" not resolved, assumed external'#000+
  '05065_W_Symbol "$1" is belongs to a library'#000+
  '05066_W_Symbol "$1" is deprecated: "$2"'#000+
  '05067_E_Cannot fin','d an enumerator for the type "$1"'#000+
  '05068_E_Cannot find a "MoveNext" method in enumerator "$1"'#000+
  '05069_E_Cannot find a "Current" property in enumerator "$1"'#000+
  '05070_E_Mismatch between number of declared parameters and number of c'+
  'olons in message ','string.'#000+
  '05071_N_Private type "$1.$2" never used'#000+
  '05072_N_Private const "$1.$2" never used'#000+
  '05073_N_Private property "$1.$2" never used'#000+
  '05074_W_Unit "$1" is deprecated'#000+
  '05075_W_Unit "$1" is deprecated: "$2"'#000+
  '05076_W_Unit "$1" is not portable'#000+
  '050','77_W_Unit "$1" is belongs to a library'#000+
  '05078_W_Unit "$1" is not implemented'#000+
  '05079_W_Unit "$1" is experimental'#000+
  '05080_E_No full definition of the formally declared class "$1" is in s'+
  'cope. Add the unit containing its full definition to the use','s clause'+
  '.'#000+
  '05081_E_Gotos into initialization or finalization blocks of units are '+
  'not allowed'#000+
  '05082_E_Invalid external name "$1" for formal class "$2"'#000+
  '05083_E_Complete class definition with external name "$1" here'#000+
  '05084_W_Possible library con','flict: symbol "$1" from library "$2" als'+
  'o found in library "$3"'#000+
  '05085_E_Cannot add implicit constructor '#039'Create'#039' because ident'+
  'ifier already used by "$1"'#000+
  '05086_E_Cannot generate default constructor for class, because parent '+
  'has no parameterl','ess constructor'#000+
  '05087_D_Adding helper for $1'#000+
  '05088_E_Found declaration: $1'#000+
  '05089_W_Local variable "$1" of a managed type does not seem to be init'+
  'ialized'#000+
  '05090_W_Variable "$1" of a managed type does not seem to be initialize'+
  'd'#000+
  '05091_H_Local v','ariable "$1" of a managed type does not seem to be in'+
  'itialized'#000+
  '05092_H_Variable "$1" of a managed type does not seem to be initialize'+
  'd'#000+
  '05093_W_function result variable of a managed type does not seem to be'+
  ' initialized'#000+
  '05094_H_Function resul','t variable of a managed type does not seem to '+
  'be initialized'#000+
  '05095_W_Duplicate identifier "$1"'#000+
  '06009_E_Parameter list size exceeds 65535 bytes'#000+
  '06012_E_File types must be var parameters'#000+
  '06013_E_The use of a far pointer isn'#039't allowed there'#000+
  '06','015_E_EXPORT declared functions cannot be called'#000+
  '06016_W_Possible illegal call of constructor or destructor'#000+
=======
  '05059_W_Function result variable does not seem to initia','lized'#000+
  '05060_H_Function result variable does not seem to be initialized'#000+
  '05061_W_Variable "$1" read but nowhere assigned'#000+
  '05062_H_Found abstract method: $1'#000+
  '05063_W_Symbol "$1" is experimental'#000+
=======
  '05059_W_Function result variable does not seem to initia','lized'#000+
  '05060_H_Function result variable does not seem to be initialized'#000+
  '05061_W_Variable "$1" read but nowhere assigned'#000+
  '05062_H_Found abstract method: $1'#000+
  '05063_W_Symbol "$1" is experimental'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05059_W_Function result variable does not seem to initia','lized'#000+
  '05060_H_Function result variable does not seem to be initialized'#000+
  '05061_W_Variable "$1" read but nowhere assigned'#000+
  '05062_H_Found abstract method: $1'#000+
  '05063_W_Symbol "$1" is experimental'#000+
>>>>>>> graemeg/cpstrnew
=======
  '05059_W_Function result variable does not seem to initia','lized'#000+
  '05060_H_Function result variable does not seem to be initialized'#000+
  '05061_W_Variable "$1" read but nowhere assigned'#000+
  '05062_H_Found abstract method: $1'#000+
  '05063_W_Symbol "$1" is experimental'#000+
>>>>>>> origin/cpstrnew
  '05064_W_Forward declaration "$1" not resolved, assu','med external'#000+
  '05065_W_Symbol "$1" is belongs to a library'#000+
  '05066_W_Symbol "$1" is deprecated: "$2"'#000+
  '05067_E_Cannot find an enumerator for the type "$1"'#000+
  '05068_E_Cannot find a "MoveNext" method in enumerator "$1"'#000+
  '05069_E_Cannot find a "Current" ','property in enumerator "$1"'#000+
  '05070_E_Mismatch between number of declared parameters and number of c'+
  'olons in message string.'#000+
  '05071_N_Private type "$1.$2" never used'#000+
  '05072_N_Private const "$1.$2" never used'#000+
  '05073_N_Private property "$1.$2" nev','er used'#000+
  '05074_W_Unit "$1" is deprecated'#000+
  '05075_W_Unit "$1" is deprecated: "$2"'#000+
  '05076_W_Unit "$1" is not portable'#000+
  '05077_W_Unit "$1" is belongs to a library'#000+
  '05078_W_Unit "$1" is not implemented'#000+
  '05079_W_Unit "$1" is experimental'#000+
  '05080_E_No comp','lete definition of the formally declared objcclass "$'+
  '1" is in scope'#000+
  '05081_E_Gotos into initialization or finalization blocks of units are '+
  'not allowed'#000+
  '05082=E_Invalid external name "$1" for formal class "$2"'#000+
  '05083=E_Complete class definition',' with external name "$1" here'#000+
  '06009_E_Parameter list size exceeds 65535 bytes'#000+
  '06012_E_File types must be var parameters'#000+
  '06013_E_The use of a far pointer isn'#039't allowed there'#000+
  '06015_E_EXPORT declared functions can'#039't be called'#000+
  '06016_W_Possible ','illegal call of constructor or destructor'#000+
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
  '06017_N_Inefficient code'#000+
=======
  '05057_H_Local variable "$1" does not seem to be initialized'#000+
  '05058_H_Variable "$1" does not seem to be i','nitialized'#000+
  '05059_W_Function result variable does not seem to initialized'#000+
  '05060_H_Function result variable does not seem to be initialized'#000+
  '05061_W_Variable "$1" read but nowhere assigned'#000+
  '05062_H_Found abstract method: $1'#000+
  '06009_E_Parameter li','st size exceeds 65535 bytes'#000+
  '06012_E_File types must be var parameters'#000+
  '06013_E_The use of a far pointer isn'#039't allowed there'#000+
  '06015_E_EXPORT declared functions can'#039't be called'#000+
  '06016_W_Possible illegal call of constructor or destructor'#000+
  '06017_N_','Inefficient code'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '06018_W_unreachable code'#000+
  '06020_E_Abstract methods cannot be called directly'#000+
  '06027_DL_Register $1 weight $2 ','$3'#000+
  '06029_DL_Stack frame is omitted'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '06031_E_Object or class methods cannot be inline.'#000+
  '06032_E_Procvar calls cannot be inline.'#000+
  '06033_E_No code for inline procedure stored'#000+
  '06035_E_Element zero of an ansi/wide- or longstring cannot be accessed'+
  ',',' use (set)length instead'#000+
  '06037_E_Constructors or destructors cannot be called inside a '#039'wit'+
  'h'#039' clause'#000+
  '06038_E_Cannot call message handler methods directly'#000+
  '06039_E_Jump in or outside of an exception block'#000+
  '06040_E_Control flow statements are n','ot allowed in a finally block'#000+
  '06041_W_Parameters size exceeds limit for certain cpu'#039's'#000+
  '06042_W_Local variable size exceed limit for certain cpu'#039's'#000+
  '06043_E_Local variables size exceeds supported limit'#000+
  '06044_E_BREAK not allowed'#000+
  '06045_E_CONTINUE',' not allowed'#000+
  '06046_F_Unknown compilerproc "$1". Check if you use the correct run ti'+
  'me library.'#000+
  '06047_F_Cannot find system type "$1". Check if you use the correct run'+
  ' time library.'#000+
  '06048_H_Inherited call to abstract method ignored'#000+
  '06049_E_Go','to label "$1" not defined or optimized away'#000+
  '06050_F_Cannot find type "$1" in unit "$2". Check if you use the corre'+
  'ct run time library.'#000+
  '06051_E_Interprocedural gotos are allowed only to outer subroutines'#000+
  '06052_E_Label must be defined in the ','same scope as it is declared'#000+
  '06053_E_Leaving procedures containing explicit or implicit exceptions '+
  'frames using goto is not allowed'#000+
  '06054_E_In ISO mode, the mod operator is defined only for positive quo'+
  'tient'#000+
  '06055_DL_Auto inlining: $1'#000+
  '06056','_E_The function used, is not supported by the selected instruct'+
  'ion set: $1'#000+
  '06057_F_Maximum number of units ($1) reached for the current target'#000+
  '07000_DL_Starting $1 styled assembler parsing'#000+
  '07001_DL_Finished $1 styled assembler parsing'#000+
  '07002','_E_Non-label pattern contains @'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '06031_E_Object or class methods ','can'#039't be inline.'#000+
  '06032_E_Procvar calls cannot be inline.'#000+
  '06033_E_No code for inline procedure stored'#000+
  '06035_E_Element zero of an ansi/wide- or longstring can'#039't be acces'+
  'sed, use (set)length instead'#000+
  '06037_E_Constructors or destructors cannot b','e called inside a '#039'w'+
  'ith'#039' clause'#000+
  '06038_E_Cannot call message handler methods directly'#000+
  '06039_E_Jump in or outside of an exception block'#000+
  '06040_E_Control flow statements aren'#039't allowed in a finally block'#000+
  '06041_W_Parameters size exceeds limit fo','r certain cpu'#039's'#000+
  '06042_W_Local variable size exceed limit for certain cpu'#039's'#000+
  '06043_E_Local variables size exceeds supported limit'#000+
  '06044_E_BREAK not allowed'#000+
  '06045_E_CONTINUE not allowed'#000+
  '06046_F_Unknown compilerproc "$1". Check if you use the c','orrect run '+
  'time library.'#000+
  '06047_F_Cannot find system type "$1". Check if you use the correct run'+
  ' time library.'#000+
  '06048_H_Inherited call to abstract method ignored'#000+
  '06049_E_Goto label "$1" not defined or optimized away'#000+
  '06050_F_Cannot find type "','$1" in unit "$2". Check if you use the cor'+
  'rect run time library.'#000+
  '06051_E_Interprocedural gotos are allowed only to outer subroutines'#000+
  '06052_E_Label must be defined in the same scope as it is declared'#000+
  '06053_E_Leaving procedures containing exp','licit or implicit exception'+
  's frames using goto is not allowed'#000+
=======
  '06031_E_Object or class methods can'#039't be inline.'#000+
  '06032_E_Procvar calls cannot be i','nline.'#000+
  '06033_E_No code for inline procedure stored'#000+
  '06035_E_Element zero of an ansi/wide- or longstring can'#039't be acces'+
  'sed, use (set)length instead'#000+
  '06037_E_Constructors or destructors can not be called inside a '#039'wi'+
  'th'#039' clause'#000+
  '06038_E_Cannot ca','ll message handler methods directly'#000+
  '06039_E_Jump in or outside of an exception block'#000+
  '06040_E_Control flow statements aren'#039't allowed in a finally block'#000+
  '06041_W_Parameters size exceeds limit for certain cpu'#039's'#000+
  '06042_W_Local variable size excee','d limit for certain cpu'#039's'#000+
  '06043_E_Local variables size exceeds supported limit'#000+
  '06044_E_BREAK not allowed'#000+
  '06045_E_CONTINUE not allowed'#000+
  '06046_F_Unknown compilerproc "$1". Check if you use the correct run ti'+
  'me library.'#000+
  '06047_F_Cannot find syst','em type "$1". Check if you use the correct r'+
  'un time library.'#000+
  '06048_H_Inherited call to abstract method ignored'#000+
  '06049_E_Goto label "$1" not defined or optimized away'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
  '07000_DL_Starting $1 styled assembler parsing'#000+
  '07001_DL_Finished $1 styled a','ssembler parsing'#000+
  '07002_E_Non-label pattern contains @'#000+
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
  '07004_E_Error building record offset'#000+
  '07005_E_OFFS','ET used without identifier'#000+
  '07006_E_TYPE used without identifier'#000+
  '07007_E_Cannot use local variable or parameters here'#000+
  '07008_E_need to use OFFSET here'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07009_E_ne','ed to use $ here'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07009_E_need to use $ here'#000+
>>>>>>> graemeg/cpstrnew
  '07010_E_Cannot use multiple relocatable symbols'#000+
  '07011_E_Relocata','ble symbol can only be added'#000+
  '07012_E_Invalid constant expression'#000+
  '07013_E_Relocatable symbol is not allowed'#000+
  '07014_E_Invalid reference syntax'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07015_E_You cannot ','reach $1 from that code'#000+
  '07016_E_Local symbols/labels are not allowed as references'#000+
  '07017_E_Invalid base and index register usage'#000+
  '07018_W_Possible error in object field handling'#000+
  '07019_E_Wrong scale factor specified'#000+
  '07020_E_Multiple index reg','ister usage'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
  '07015_E_You cannot reach $1 from that code'#000+
  '07016_E_Local symbols/labels aren'#039't allowed as references',#000+
  '07017_E_Invalid base and index register usage'#000+
  '07018_W_Possible error in object field handling'#000+
  '07019_E_Wrong scale factor specified'#000+
<<<<<<< HEAD
=======
=======
>>>>>>> origin/cpstrnew
  '07015_E_You cannot reach $1 from that code'#000+
  '07016_E_Local symbols/labels aren'#039't allowed as references',#000+
  '07017_E_Invalid base and index register usage'#000+
  '07018_W_Possible error in object field handling'#000+
  '07019_E_Wrong scale factor specified'#000+
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07020_E_Multiple index register usage'#000+
>>>>>>> graemeg/cpstrnew
  '07021_E_Invalid operand type'#000+
  '07022_E_Invalid string as opcode operand:',' $1'#000+
  '07023_W_@CODE and @DATA not supported'#000+
  '07024_E_Null label references are not allowed'#000+
  '07025_E_Divide by zero in asm evaluator'#000+
  '07026_E_Illegal expression'#000+
  '070','27_E_escape sequence ignored: $1'#000+
  '07028_E_Invalid symbol reference'#000+
  '07029_W_Fwait ca','n cause emulation problems with emu387'#000+
  '07030_W_$1 without operand translated into $1P'#000+
  '07031_W_ENTER instruction is not supported by Linux kernel'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07032_W_Calli','ng an overload function in assembler'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07032_W_Calling an overload function in assembler'#000+
>>>>>>> graemeg/cpstrnew
  '07033_E_Unsupported symbol type for operand'#000+
  '0','7034_E_Constant value out of bounds'#000+
  '07035_E_Error converting decimal $1'#000+
  '07036_E_Error converting octal $1'#000+
  '07037_E_Error converting binary $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07038_E_Error con','verting hexadecimal $1'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07038_E_Error converting hexadecimal $1'#000+
>>>>>>> graemeg/cpstrnew
  '07039_H_$1 translated to $2'#000+
  '07040_W_$1 is associated to an ','overloaded function'#000+
  '07041_E_Cannot use SELF outside a method'#000+
  '07042_E_Cannot use OLDEBP outside a nested procedure'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07043_W_Procedures cannot return any value i','n asm code'#000+
=======
  '07043_W_Procedures can'#039't return any value in asm code'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07043_W_Procedures can'#039't return any value in asm code'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07043_W_Procedures can'#039't return any value in asm code'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07043_W_Procedures can'#039't return any value in asm code'#000+
>>>>>>> origin/cpstrnew
  '07044_E_SEG not supported'#000+
  '07045_E_Size suffix and destination or source ','size do not match'#000+
  '07046_W_Size suffix and destination or source size do not match'#000+
  '07047_E_Assembler syntax error'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07048_E_Invalid combination of opcode and op','erands'#000+
=======
  '07048_E_Invalid combination of opcode and operands'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07048_E_Invalid combination of opcode and operands'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07048_E_Invalid combination of opcode and operands'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07048_E_Invalid combination of opcode and operands'#000+
>>>>>>> origin/cpstrnew
  '07049_E_Assembler syntax error in operand'#000+
  '07050_E_Assembler syntax error in ','constant'#000+
  '07051_E_Invalid String expression'#000+
  '07052_W_constant with symbol $1 for address which is not on a pointer'#000+
  '07053_E_Unrecognized opcode $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07054_E_Inval','id or missing opcode'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07054_E_Invalid or missing opcode'#000+
>>>>>>> graemeg/cpstrnew
  '07055_E_Invalid combination of prefix and opcode: $1'#000+
  '07056_E_I','nvalid combination of override and opcode: $1'#000+
  '07057_E_Too many operands on line'#000+
  '07058_W_NEAR ignored'#000+
  '07059_W_FAR ignored'#000+
  '07060_E_Duplicate local symbol $1'#000+
  '07','061_E_Undefined local symbol $1'#000+
  '07062_E_Unknown label identifier $1'#000+
  '07063_E_Invalid',' register name'#000+
  '07064_E_Invalid floating point register name'#000+
  '07066_W_Modulo not supported'#000+
  '07067_E_Invalid floating point constant $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07068_E_Invalid floating ','point expression'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07068_E_Invalid floating point expression'#000+
>>>>>>> graemeg/cpstrnew
  '07069_E_Wrong symbol type'#000+
  '07070_E_Cannot index a local var or para','meter with a register'#000+
  '07071_E_Invalid segment override expression'#000+
  '07072_W_Identifier $1 supposed external'#000+
  '07073_E_Strings not allowed as constants'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07074_E_No',' type of variable specified'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07074_No type of variable specified'#000+
>>>>>>> graemeg/cpstrnew
  '07075_E_assembler code not returned to text section'#000+
  '07076','_E_Not a directive or local symbol $1'#000+
  '07077_E_Using a defined name as a local label'#000+
  '07078_E_Dollar token is used without an identifier'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07079_W_32bit consta','nt created for address'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07079_W_32bit constant created for address'#000+
>>>>>>> graemeg/cpstrnew
  '07080_N_.align is target specific, use .balign or .p2align'#000+
  '070','81_E_Can'#039't access fields directly for parameters'#000+
  '07082_E_Can'#039't access fields of objects/classes directly'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07083_E_No size specified and unable to determine ','the size of the op'+
  'erands'#000+
=======
  '07083_E_No size specified and unable to determine the size of the oper'+
  'ands'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07083_E_No size specified and unable to determine the size of the oper'+
  'ands'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07083_E_No size specified and unable to determine the size of the oper'+
  'ands'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07083_E_No size specified and unable to determine the size of the oper'+
  'ands'#000+
>>>>>>> origin/cpstrnew
  '07084_E_Cannot use RESULT in this function'#000+
  '07086_W_"$1" with','out operand translated into "$1 %st,%st(1)"'#000+
  '07087_W_"$1 %st(n)" translated into "$1 %st,%st(n)"'#000+
  '07088_W_"$1 %st(n)" translated into "$1 %st(n),%st"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07089_E','_Char < not allowed here'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '07089_E_Char < not allowed here'#000+
>>>>>>> graemeg/cpstrnew
  '07090_E_Char > not allowed here'#000+
  '07093_W_ALIGN not supported'#000,
  '07094_E_Inc and Dec cannot be together'#000+
  '07095_E_Invalid reglist for movem'#000+
  '07096_E_Reglist invalid for opcode'#000+
  '07097_E_Higher cpu mode required ($1)'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '07098_W_N','o size specified and unable to determine the size of the op'+
  'erands, using DWORD as default'#000+
  '07099_E_Syntax error while trying to parse a shifter operand'#000+
  '07100_E_Address of packed component is not at a byte boundary'#000+
  '07101_W_No size specified a','nd unable to determine the size of the op'+
=======
=======
  '07000_DL_Starting $1 styled assembler parsing'#000+
  '07001_DL_Finished $1 styled a','ssembler parsing'#000+
  '07002_E_Non-label pattern contains @'#000+
>>>>>>> origin/fixes_2_2
  '07004_E_Error building record offset'#000+
  '07005_E_OFFSET used without identifier'#000+
  '07006_E_TYPE used without identifier'#000+
  '07007_E_Cannot use local variable or parameters here'#000+
  '07008_E_need to use ','OFFSET here'#000+
  '07009_E_need to use $ here'#000+
  '07010_E_Cannot use multiple relocatable symbols'#000+
  '07011_E_Relocatable symbol can only be added'#000+
  '07012_E_Invalid constant expression'#000+
  '07013_E_Relocatable symbol is not allowed'#000+
  '07014_E_Invalid reference synt','ax'#000+
  '07015_E_You can not reach $1 from that code'#000+
  '07016_E_Local symbols/labels aren'#039't allowed as references'#000+
  '07017_E_Invalid base and index register usage'#000+
  '07018_W_Possible error in object field handling'#000+
  '07019_E_Wrong scale factor specified'#000+
  '0702','0_E_Multiple index register usage'#000+
  '07021_E_Invalid operand type'#000+
  '07022_E_Invalid string as opcode operand: $1'#000+
  '07023_W_@CODE and @DATA not supported'#000+
  '07024_E_Null label references are not allowed'#000+
  '07025_E_Divide by zero in asm evaluator'#000+
  '07026_E_','Illegal expression'#000+
  '07027_E_escape sequence ignored: $1'#000+
  '07028_E_Invalid symbol reference'#000+
  '07029_W_Fwait can cause emulation problems with emu387'#000+
  '07030_W_$1 without operand translated into $1P'#000+
  '07031_W_ENTER instruction is not supported by Linu','x kernel'#000+
  '07032_W_Calling an overload function in assembler'#000+
  '07033_E_Unsupported symbol type for operand'#000+
  '07034_E_Constant value out of bounds'#000+
  '07035_E_Error converting decimal $1'#000+
  '07036_E_Error converting octal $1'#000+
  '07037_E_Error converting binar','y $1'#000+
  '07038_E_Error converting hexadecimal $1'#000+
  '07039_H_$1 translated to $2'#000+
  '07040_W_$1 is associated to an overloaded function'#000+
  '07041_E_Cannot use SELF outside a method'#000+
  '07042_E_Cannot use OLDEBP outside a nested procedure'#000+
  '07043_W_Procedures can',#039't return any value in asm code'#000+
  '07044_E_SEG not supported'#000+
  '07045_E_Size suffix and destination or source size do not match'#000+
  '07046_W_Size suffix and destination or source size do not match'#000+
  '07047_E_Assembler syntax error'#000+
  '07048_E_Invalid combina','tion of opcode and operands'#000+
  '07049_E_Assembler syntax error in operand'#000+
  '07050_E_Assembler syntax error in constant'#000+
  '07051_E_Invalid String expression'#000+
  '07052_W_constant with symbol $1 for address which is not on a pointer'#000+
  '07053_E_Unrecognized op','code $1'#000+
  '07054_E_Invalid or missing opcode'#000+
  '07055_E_Invalid combination of prefix and opcode: $1'#000+
  '07056_E_Invalid combination of override and opcode: $1'#000+
  '07057_E_Too many operands on line'#000+
  '07058_W_NEAR ignored'#000+
  '07059_W_FAR ignored'#000+
  '07060_E_Duplica','te local symbol $1'#000+
  '07061_E_Undefined local symbol $1'#000+
  '07062_E_Unknown label identifier $1'#000+
  '07063_E_Invalid register name'#000+
  '07064_E_Invalid floating point register name'#000+
  '07066_W_Modulo not supported'#000+
  '07067_E_Invalid floating point constant $1'#000+
  '0706','8_E_Invalid floating point expression'#000+
  '07069_E_Wrong symbol type'#000+
  '07070_E_Cannot index a local var or parameter with a register'#000+
  '07071_E_Invalid segment override expression'#000+
  '07072_W_Identifier $1 supposed external'#000+
  '07073_E_Strings not allowed as',' constants'#000+
  '07074_No type of variable specified'#000+
  '07075_E_assembler code not returned to text section'#000+
  '07076_E_Not a directive or local symbol $1'#000+
  '07077_E_Using a defined name as a local label'#000+
  '07078_E_Dollar token is used without an identifier'#000+
  '0','7079_W_32bit constant created for address'#000+
  '07080_N_.align is target specific, use .balign or .p2align'#000+
  '07081_E_Can'#039't access fields directly for parameters'#000+
  '07082_E_Can'#039't access fields of objects/classes directly'#000+
  '07083_E_No size specified and u','nable to determine the size of the op'+
  'erands'#000+
  '07084_E_Cannot use RESULT in this function'#000+
  '07086_W_"$1" without operand translated into "$1 %st,%st(1)"'#000+
  '07087_W_"$1 %st(n)" translated into "$1 %st,%st(n)"'#000+
  '07088_W_"$1 %st(n)" translated into "$1 ','%st(n),%st"'#000+
  '07089_E_Char < not allowed here'#000+
  '07090_E_Char > not allowed here'#000+
  '07093_W_ALIGN not supported'#000+
  '07094_E_Inc and Dec cannot be together'#000+
  '07095_E_Invalid reglist for movem'#000+
  '07096_E_Reglist invalid for opcode'#000+
  '07097_E_Higher cpu mode requ','ired ($1)'#000+
  '07098_W_No size specified and unable to determine the size of the oper'+
  'ands, using DWORD as default'#000+
  '07099_E_Syntax error while trying to parse a shifter operand'#000+
  '07100_E_Address of packed component is not at a byte boundary'#000+
  '07101_W_','No size specified and unable to determine the size of the op'+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  'erands, using BYTE as default'#000+
  '07102_W_Use of +offset(%ebp) for parameters invalid here'#000+
  '07103_W_Use of +offset(%ebp) is not compatible with regcall convention'+
  #000+
<<<<<<< HEAD
<<<<<<< HEAD
  '07104_W_Use of -offset(%ebp) is not recom','mended for local variable a'+
  'ccess'#000+
  '07105_W_Use of -offset(%esp), access may cause a crash or value may be'+
  ' lost'#000+
  '07106_E_VMTOffset must be used in combination with a virtual method, a'+
  'nd "$1" is not virtual'#000+
  '07107_E_Generating PIC, but reference ','is not PIC-safe'#000+
  '07108_E_All registers in a register set must be of the same kind and w'+
  'idth'#000+
  '07109_E_A register set cannot be empty'#000+
  '07110_W_@GOTPCREL is useless and potentially dangerous for local symbo'+
  'ls'#000+
  '07111_W_Constant with general purpose',' segment register'#000+
  '07112_E_Invalid offset value for $1'#000+
  '07113_E_Invalid register for $1'#000+
  '07114_E_SEH directives are allowed only in pure assembler procedures'#000+
  '07115_E_Directive "$1" is not supported for the current target'#000+
  '07116_E_This function'#039,'s result location cannot be encoded directl'+
  'y in a single operand when "nostackframe" is used'#000+
  '07117_E_GOTPCREL references in Intel assembler syntax cannot contain a'+
  ' base or index register, and their offset must 0.'#000+
  '07118_E_The current target ','does not support GOTPCREL relocations'#000+
  '07119_W_Exported/global symbols should be accessed via the GOT'#000+
  '07120_W_Check size of memory operand "$1"'#000+
  '07121_W_Check size of memory operand "$1: memory-operand-size is $2 bi'+
  'ts, but expected [$3 bits]"',#000+
  '07122_W_Check size of memory operand "$1: memory-operand-size is $2 bi'+
  'ts, but expected [$3 bits + $4 byte offset]"'#000+
  '07123_W_Check "$1: offset of memory operand is negative "$2 byte"'#000+
  '07124_W_Check "$1: size of memory operand is empty, but es',' exists di'+
  'fferent definitions of the memory size =>> map to $2 (smallest option)'+
  '"'#000+
  '07125_E_Invalid register used in memory reference expression: "$1"'#000+
  '07126_E_SEG used without identifier'#000+
  '07127_E_@CODE and @DATA can only be used with the SEG o','perator'#000+
  '07128_E_Not enough space (16 bits required) for the segment constant o'+
  'f symbol $1'#000+
  '08000_F_Too many assembler files'#000+
  '08001_F_Selected assembler output not supported'#000+
  '08002_F_Comp not supported'#000+
  '08003_F_Direct not support for binary writ','ers'#000+
=======
  '07098_W_No size specified and unable to determine the size of the oper'+
  'ands, using DWORD as def','ault'#000+
  '07099_E_Syntax error while trying to parse a shifter operand'#000+
  '07100_E_Address of packed component is not at a byte boundary'#000+
  '07101_W_No size specified and unable to determine the size of the oper'+
  'ands, using BYTE as default'#000+
  '07102_W_Use of',' +offset(%ebp) for parameters invalid here'#000+
  '07103_W_Use of +offset(%ebp) is not compatible with regcall convention'+
  #000+
  '07104_W_Use of -offset(%ebp) is not recommended for local variable acc'+
  'ess'#000+
  '07105_W_Use of -offset(%esp), access may cause a cra','sh or value may '+
  'be lost'#000+
  '07106_E_VMTOffset must be used in combination with a virtual method, a'+
  'nd "$1" is not virtual'#000+
=======
  '07098_W_No size specified and unable to determine the size of the oper'+
  'ands, using DWORD as def','ault'#000+
  '07099_E_Syntax error while trying to parse a shifter operand'#000+
  '07100_E_Address of packed component is not at a byte boundary'#000+
  '07101_W_No size specified and unable to determine the size of the oper'+
  'ands, using BYTE as default'#000+
  '07102_W_Use of',' +offset(%ebp) for parameters invalid here'#000+
  '07103_W_Use of +offset(%ebp) is not compatible with regcall convention'+
  #000+
  '07104_W_Use of -offset(%ebp) is not recommended for local variable acc'+
  'ess'#000+
  '07105_W_Use of -offset(%esp), access may cause a cra','sh or value may '+
  'be lost'#000+
  '07106_E_VMTOffset must be used in combination with a virtual method, a'+
  'nd "$1" is not virtual'#000+
>>>>>>> graemeg/cpstrnew
=======
  '07098_W_No size specified and unable to determine the size of the oper'+
  'ands, using DWORD as def','ault'#000+
  '07099_E_Syntax error while trying to parse a shifter operand'#000+
  '07100_E_Address of packed component is not at a byte boundary'#000+
  '07101_W_No size specified and unable to determine the size of the oper'+
  'ands, using BYTE as default'#000+
  '07102_W_Use of',' +offset(%ebp) for parameters invalid here'#000+
  '07103_W_Use of +offset(%ebp) is not compatible with regcall convention'+
  #000+
  '07104_W_Use of -offset(%ebp) is not recommended for local variable acc'+
  'ess'#000+
  '07105_W_Use of -offset(%esp), access may cause a cra','sh or value may '+
  'be lost'#000+
  '07106_E_VMTOffset must be used in combination with a virtual method, a'+
  'nd "$1" is not virtual'#000+
>>>>>>> graemeg/cpstrnew
  '07107_E_Generating PIC, but reference is not PIC-safe'#000+
  '07108_E_All registers in a register set must be of the same kind and ',
  'width'#000+
  '07109_E_A register set cannot be empty'#000+
  '07110_W_@GOTPCREL is useless and potentially dangereous for local symb'+
  'ols'#000+
  '08000_F_Too many assembler files'#000+
  '08001_F_Selected assembler output not supported'#000+
  '08002_F_Comp not supported'#000+
  '08003_F_Direc','t not support for binary writers'#000+
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
  '07098_W_No size specified and unable to determine the size of the oper'+
  'ands, using DWORD as def','ault'#000+
  '07099_E_Syntax error while trying to parse a shifter operand'#000+
  '07100_E_Address of packed component is not at a byte boundary'#000+
  '07101_W_No size specified and unable to determine the size of the oper'+
  'ands, using BYTE as default'#000+
  '07102_W_Use of',' +offset(%ebp) for parameters invalid here'#000+
  '07103_W_Use of +offset(%ebp) is not compatible with regcall convention'+
  #000+
  '07104_W_Use of -offset(%ebp) is not recommended for local variable acc'+
  'ess'#000+
  '07105_W_Use of -offset(%esp), access may cause a cra','sh or value may '+
  'be lost'#000+
  '07106_E_VMTOffset must be used in combination with a virtual method, a'+
  'nd "$1" is not virtual'#000+
  '07107_E_Generating PIC, but reference is not PIC-safe'#000+
  '07108_E_All registers in a register set must be of the same kind and ',
  'width'#000+
  '07109_E_A register set cannot be empty'#000+
  '07110_W_@GOTPCREL is useless and potentially dangereous for local symb'+
  'ols'#000+
  '08000_F_Too many assembler files'#000+
  '08001_F_Selected assembler output not supported'#000+
  '08002_F_Comp not supported'#000+
  '08003_F_Direc','t not support for binary writers'#000+
>>>>>>> origin/cpstrnew
  '08004_E_Allocating of data is only allowed in bss section'#000+
  '08005_F_No binary writer selected'#000+
  '08006_E_Asm: Opcode $1 not in table'#000+
  '08007_E_Asm: $1 invalid combination of opcode and operands'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  '07104_W_Use of -offset','(%ebp) is not recommended for local variable a'+
  'ccess'#000+
  '07105_W_Use of -offset(%esp), access may cause a crash or value may be'+
  ' lost'#000+
  '07106_E_VMTOffset must be used in combination with a virtual method, a'+
  'nd "$1" is not virtual'#000+
  '07107_E_Generating ','PIC, but reference is not PIC-safe'#000+
  '08000_F_Too many assembler files'#000+
  '08001_F_Selected assembler output not supported'#000+
  '08002_F_Comp not supported'#000+
  '08003_F_Direct not support for binary writers'#000+
  '08004_E_Allocating of data is only allowed in bss s','ection'#000+
  '08005_F_No binary writer selected'#000+
  '08006_E_Asm: Opcode $1 not in table'#000+
  '08007_E_Asm: $1 invalid combination of opcode and operands'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '07104_W_Use of -offset','(%ebp) is not recommended for local variable a'+
  'ccess'#000+
  '07105_W_Use of -offset(%esp), access may cause a crash or value may be'+
  ' lost'#000+
  '07106_E_VMTOffset must be used in combination with a virtual method, a'+
  'nd "$1" is not virtual'#000+
  '07107_E_Generating ','PIC, but reference is not PIC-safe'#000+
  '08000_F_Too many assembler files'#000+
  '08001_F_Selected assembler output not supported'#000+
  '08002_F_Comp not supported'#000+
  '08003_F_Direct not support for binary writers'#000+
  '08004_E_Allocating of data is only allowed in bss s','ection'#000+
  '08005_F_No binary writer selected'#000+
  '08006_E_Asm: Opcode $1 not in table'#000+
  '08007_E_Asm: $1 invalid combination of opcode and operands'#000+
>>>>>>> origin/fixes_2_2
  '08008_E_Asm: 16 Bit references not supported'#000+
  '0800','9_E_Asm: Invalid effective address'#000+
=======
  '08008_E_Asm: 16 Bit ','references not supported'#000+
  '08009_E_Asm: Invalid effective address'#000+
>>>>>>> graemeg/cpstrnew
=======
  '08008_E_Asm: 16 Bit ','references not supported'#000+
  '08009_E_Asm: Invalid effective address'#000+
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  '08008_E_Asm: 16 Bit ','references not supported'#000+
  '08009_E_Asm: Invalid effective address'#000+
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  '08008_E_Asm: 16 Bit ','references not supported'#000+
  '08009_E_Asm: Invalid effective address'#000+
>>>>>>> origin/cpstrnew
  '08010_E_Asm: Immediate or reference expected'#000+
  '08011_E_Asm: $1 value exceeds bounds $2'#000+
  '08012_E_Asm: Short jump is out of range $1'#000+
  '08013_E_Asm: Undefined label $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '08014_E_Asm: Comp type not supported for this',' target'#000+
=======
  '08014_E_Asm: Com','p type not supported for this target'#000+
>>>>>>> graemeg/cpstrnew
=======
  '08014_E_Asm: Com','p type not supported for this target'#000+
>>>>>>> graemeg/cpstrnew
=======
  '08014_E_Asm: Com','p type not supported for this target'#000+
>>>>>>> graemeg/cpstrnew
=======
  '08014_E_Asm: Com','p type not supported for this target'#000+
>>>>>>> origin/cpstrnew
  '08015_E_Asm: Extended type not supported for this target'#000+
=======
  '08010_E_Asm: Immedia','te or reference expected'#000+
  '08011_E_Asm: $1 value exceeds bounds $2'#000+
  '08012_E_Asm: Short jump is out of range $1'#000+
  '08013_E_Asm: Undefined label $1'#000+
  '08014_E_Asm: Comp type not supported for this target'#000+
  '08015_E_Asm: Extended type not supported for th','is target'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '08010_E_Asm: Immedia','te or reference expected'#000+
  '08011_E_Asm: $1 value exceeds bounds $2'#000+
  '08012_E_Asm: Short jump is out of range $1'#000+
  '08013_E_Asm: Undefined label $1'#000+
  '08014_E_Asm: Comp type not supported for this target'#000+
  '08015_E_Asm: Extended type not supported for th','is target'#000+
>>>>>>> origin/fixes_2_2
  '08016_E_Asm: Duplicate label $1'#000+
  '08017_E_Asm: Redefined label $1'#000+
  '08018_E_Asm: First defined here'#000+
  '08019_E_Asm: Invalid register $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '08020_E_Asm: 16 or 32 Bit references not suppo','rted'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '08022_E_Asm: AH,BH,CH or DH cannot be used in an instruction requiring'+
  ' REX prefix'#000+
  '08023_E_Missing .seh_endprologue directive'#000+
  '08024_E_Function prologue exceeds 255 bytes'#000+
  '08025_E_.seh_handlerdat','a directive without preceding .seh_handler'#000+
  '08026_F_Relocation count for section $1 exceeds 65535'#000+
  '08027_N_Change of bind type of symbol $1 from $2 to $3 after use'#000+
  '08028_H_Change of bind type of symbol $1 from $2 to $3 after use'#000+
  '08029_E_Asm: ','32 Bit references not supported'#000+
  '08030_F_Code segment too large'#000+
  '08031_F_Data segment too large'#000+
  '09000_W_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
  '09003_E_Can'#039't create object file: $','1 (error code: $2)'#000+
=======
  '08020_E_Asm: 16 or 32 Bit references not supported'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '09000_W','_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
  '09003_E_Can'#039't create object file: $1'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '08020_E_Asm: 16 or 32 Bit references not supported'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '09000_W','_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
  '09003_E_Can'#039't create object file: $1'#000+
>>>>>>> origin/fixes_2_2
  '09004_E_Can'#039't create archive file: $1'#000+
  '09005_E_Assembler $1 not found, switching to external assembl','ing'#000+
  '09006_T_Using assembler: $1'#000+
  '09007_E_Error while assembling exitcode $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '09008_E_Can'#039't call the assembler, error $1 swit','ching to external'+
  ' assembling'#000+
=======
  '09008_E_Can'#039't call the assembler, error $1 switching to external a'+
  'ssembling'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '09008_E_Can'#039't call the assembler, error $1 switching to external a'+
  'ssembling'#000+
>>>>>>> origin/fixes_2_2
  '09009_I_Assembling $1'#000+
=======
  '08020_E_Asm: 16 o','r 32 Bit references not supported'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '09000_W_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
=======
  '08020_E_Asm: 16 o','r 32 Bit references not supported'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '09000_W_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
>>>>>>> graemeg/cpstrnew
=======
  '08020_E_Asm: 16 o','r 32 Bit references not supported'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '09000_W_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
>>>>>>> graemeg/cpstrnew
=======
  '08020_E_Asm: 16 o','r 32 Bit references not supported'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '09000_W_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
>>>>>>> origin/cpstrnew
  '09003_E_Can'#039't create object file: $1'#000+
  '09004_E_Can'#039't c','reate archive file: $1'#000+
  '09005_E_Assembler $1 not found, switching to external assembling'#000+
  '09006_T_Using assembler: $1'#000+
  '09007_E_Error while assembling exitcode $1'#000+
  '09008_E_Can'#039't call the assembler, error $1 switching to external a'+
  'ssembling'#000+
  '09009','_I_Assembling $1'#000+
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
  '09010_I_Assembling with smartlinking $1'#000+
  '09011_W_Object $1 not found',', Linking may fail !'#000+
  '09012_W_Library $1 not found, Linking may fail !'#000+
  '09013_E_Error while linking'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '09014_E_Can'#039't call the l','inker, switching to external linking'#000+
=======
  '09014_E_Can'#039't call the linker, switching to external linki','ng'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09014_E_Can'#039't call the linker, switching to external linki','ng'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09014_E_Can'#039't call the linker, switching to external linki','ng'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09014_E_Can'#039't call the linker, switching to external linki','ng'#000+
>>>>>>> origin/cpstrnew
=======
  '09014_E_Can'#039't call the linker, switching to external linking'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '09014_E_Can'#039't call the linker, switching to external linking'#000+
>>>>>>> origin/fixes_2_2
  '09015_I_Linking $1'#000+
  '09016_E_Util $1 not found, switching to external linking'#000+
  '09017','_T_Using util $1'#000+
  '09018_E_Creation of Executables not supported'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '09019_E_Creation of Dynamic/Shared Libraries not supported'#000,
  '09035_E_Creation of Static Libraries not supported'#000+
  '09020_I_Closing script $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '09021_E_resource compiler "$1" not found, switching to external mode'#000+
  '09022_I_Compiling resource $1'#000+
  '09023_T_unit $1 cannot be statically linked, switching to smart ','link'+
  'ing'#000+
  '09024_T_unit $1 cannot be smart linked, switching to static linking'#000+
  '09025_T_unit $1 cannot be shared linked, switching to static linking'#000+
  '09026_E_unit $1 cannot be smart or static linked'#000+
  '09027_E_unit $1 cannot be shared or static lin','ked'#000+
  '09028_D_Calling resource compiler "$1" with "$2" as command line'#000+
  '09029_E_Error while compiling resources'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '09021_E_','resource compiler "$1" not found, switching to external mode'+
  #000+
  '09022_I_Compiling resource $1'#000+
=======
  '09019_E_Creation of Dynamic/Shared Libraries not supported'#000+
  '09020_I_Closing script $1'#000+
  '09021_E_resource compiler not found, switching to external mode'#000+
  '09022_I_Compiling resource $','1'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '09019_E_Creation of Dynamic/Shared Libraries not supported'#000+
  '09020_I_Closing script $1'#000+
  '09021_E_resource compiler not found, switching to external mode'#000+
  '09022_I_Compiling resource $','1'#000+
>>>>>>> origin/fixes_2_2
  '09023_T_unit $1 can'#039't be statically linked, switching to smart lin'+
  'king'#000+
  '09024_T_unit $1 can'#039't be smart linked, switching to static linking'+
  #000+
<<<<<<< HEAD
<<<<<<< HEAD
  '09025_T_uni','t $1 can'#039't be shared linked, switching to static link'+
  'ing'#000+
  '09026_E_unit $1 can'#039't be smart or static linked'#000+
  '09027_E_unit $1 can'#039't be shared or static linked'#000+
  '09028_D_Calling resource compiler "$1" with "$2" as command line'#000+
  '09029_E_Error while c','ompiling resources'#000+
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
  '09030_E_Can'#039't call the resource compiler "$1", switching to extern'+
  'al mode'#000+
  '09031_E_Can'#039't open resource file "$1"'#000+
  '09032_E_Can'#039't write',' resource file "$1"'#000+
  '09033_N_File "$1" not found for backquoted cat command'#000+
  '09034_W_"$1" not found, this will probably cause a linking failure'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '09025_T_unit $1 can'#039't be shared linked, switching to static linkin'+
  'g'#000+
  '09026_E_unit $1 can'#039't be smart o','r static linked'#000+
  '09027_E_unit $1 can'#039't be shared or static linked'#000+
  '09028_D_Calling resource compiler "$1" with "$2" as command line'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '09128_F_Can'#039't post process executable $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '09129_F_Can'#039't open executable $1'#000+
  '09130_X_Size of Code: $1',' bytes'#000+
=======
  '09129_F_Can'#039't open executable',' $1'#000+
  '09130_X_Size of Code: $1 bytes'#000+
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  '09129_F_Can'#039't open executable',' $1'#000+
  '09130_X_Size of Code: $1 bytes'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09129_F_Can'#039't open executable',' $1'#000+
  '09130_X_Size of Code: $1 bytes'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09129_F_Can'#039't open executable',' $1'#000+
  '09130_X_Size of Code: $1 bytes'#000+
>>>>>>> origin/cpstrnew
  '09131_X_Size of initialized data: $1 bytes'#000+
=======
  '09131','_X_Size of initialized data: $1 bytes'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '09131','_X_Size of initialized data: $1 bytes'#000+
>>>>>>> origin/fixes_2_2
  '09132_X_Size of uninitialized data: $1 bytes'#000+
  '09133_X_Stack space reserved: $1 bytes'#000+
  '09134_X_Stack space committed: $1 bytes'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '09200_F_Executable image size is too big for $1 target.'#000+
  '09201_W_Ob','ject file "$1" contains 32-bit absolute relocation to symb'+
  'ol "$2".'#000+
  '09202_E_Program segment too large (exceeds 64k by $1 bytes)'#000+
  '09203_E_Code segment too large (exceeds 64k by $1 bytes)'#000+
  '09204_E_Data segment too large (exceeds 64k by $1 bytes)',#000+
  '09205_E_Segment "$1" too large (exceeds 64k by $2 bytes)'#000+
  '09206_E_Group "$1" too large (exceeds 64k by $2 bytes)'#000+
  '09207_E_Cannot create a .COM file, because the program contains segmen'+
  't relocations'#000+
=======
  '09200_F_Executable image size is too b','ig for $1 target.'#000+
  '09201_W_Object file "$1" contains 32-bit absolute relocation to symbol'+
  ' "$2".'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09200_F_Executable image size is too b','ig for $1 target.'#000+
  '09201_W_Object file "$1" contains 32-bit absolute relocation to symbol'+
  ' "$2".'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09200_F_Executable image size is too b','ig for $1 target.'#000+
  '09201_W_Object file "$1" contains 32-bit absolute relocation to symbol'+
  ' "$2".'#000+
>>>>>>> graemeg/cpstrnew
=======
  '09200_F_Executable image size is too b','ig for $1 target.'#000+
  '09201_W_Object file "$1" contains 32-bit absolute relocation to symbol'+
  ' "$2".'#000+
>>>>>>> origin/cpstrnew
  '10000_T_Unitsearch: $1'#000+
  '10001_T_PPU Loading ','$1'#000+
  '10002_U_PPU Name: $1'#000+
  '10003_U_PPU',' Flags: $1'#000+
  '10004_U_PPU Crc: $1'#000+
  '10005_U_PPU Time: $1'#000+
<<<<<<< HEAD
  '10006_U_PPU Fil','e too short'#000+
=======
  '10006_U_PPU File too short'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '10007_U_PPU Invalid Header (no PPU at the begin)'#000+
  '10008_U_PPU Invalid Version $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '10009_U_PPU is compiled for another processor'#000,
=======
  '10009_U_PPU is compiled for another processor'#000+
>>>>>>> graemeg/cpstrnew
=======
  '10009_U_PPU is compiled for another processor'#000+
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
  '10009_U_PPU is compiled for another processor'#000+
>>>>>>> graemeg/cpstrnew
=======
  '10009_U_PPU is compiled for another processor'#000+
>>>>>>> origin/cpstrnew
  '10010_U_PPU is compiled for another target'#000+
=======
  '10010_U_PPU is compiled for an othe','r target'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '10010_U_PPU is compiled for an othe','r target'#000+
>>>>>>> origin/fixes_2_2
  '10011_U_PPU Source: $1'#000+
  '10012_U_Writing $1'#000+
  '10013_F_Can'#039't Wri','te PPU-File'#000+
  '10014_F_Error reading PPU-File'#000+
  '10015_F_unexpected end of PPU-File'#000+
  '10016_F_Invalid PPU-File entry: $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '10017_F_PPU Dbx count pro','blem'#000+
  '10018_E_Illegal unit name: $1'#000+
  '10019_F_Too much units'#000+
  '10020_F_Circular unit reference between $1 a','nd $2'#000+
  '10021_F_Can'#039't compile unit $1, no sources available'#000+
  '10022_F_Can'#039't find unit $1 used by $2'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '10023_W_Unit $1 was not found but $2 exist','s'#000+
  '10024_F_Unit $1 searched but $2 found'#000+
  '10025_W_Compiling the system unit requires the -Us switch'#000+
  '10026_F_There were $1 errors compiling module, stopping'#000+
=======
  '10023_W_Unit $1 was not found but $2 exists'#000+
  '10024_F_Unit $1 searched but $2 found'#000+
  '10025_W_Compiling the system unit requires the -Us switch'#000+
  '1002','6_F_There were $1 errors compiling module, stopping'#000+
>>>>>>> graemeg/cpstrnew
=======
  '10023_W_Unit $1 was not found but $2 exists'#000+
  '10024_F_Unit $1 searched but $2 found'#000+
  '10025_W_Compiling the system unit requires the -Us switch'#000+
  '1002','6_F_There were $1 errors compiling module, stopping'#000+
>>>>>>> graemeg/cpstrnew
=======
  '10023_W_Unit $1 was not found but $2 exists'#000+
  '10024_F_Unit $1 searched but $2 found'#000+
  '10025_W_Compiling the system unit requires the -Us switch'#000+
  '1002','6_F_There were $1 errors compiling module, stopping'#000+
>>>>>>> graemeg/cpstrnew
=======
=======
  '10017_F_PPU Dbx count problem'#000+
  '10018_E_Illegal unit name: $1',#000+
  '10019_F_Too much units'#000+
  '10020_F_Circular unit reference between $1 and $2'#000+
  '10021_F_Can'#039't compile unit $1, no sources available'#000+
  '10022_F_Can'#039't find unit $1 used by $2'#000+
>>>>>>> origin/fixes_2_2
  '10023_W_Unit $1 was not found but $2 exists'#000+
  '10024_F_Unit $1 searched but $2 ','found'#000+
  '10025_W_Compiling the system unit requires the -Us switch'#000+
  '1002','6_F_There were $1 errors compiling module, stopping'#000+
>>>>>>> origin/cpstrnew
  '10027_U_Load from $1 ($2) unit $3'#000+
  '10028_U_Recompiling $1, checksum changed for $2'#000+
  '1002','9_U_Recompiling $1, source found only'#000+
=======
  '10017_F_PPU Dbx count problem'#000+
  '10018_E_Illegal unit name: $1',#000+
  '10019_F_Too much units'#000+
  '10020_F_Circular unit reference between $1 and $2'#000+
  '10021_F_Can'#039't compile unit $1, no sources available'#000+
  '10022_F_Can'#039't find unit $1 used by $2'#000+
  '10023_W_Unit $1 was not found but $2 exists'#000+
  '10024_F_Unit $1 searched but $2 ','found'#000+
  '10025_W_Compiling the system unit requires the -Us switch'#000+
  '10026_F_There were $1 errors compiling module, stopping'#000+
  '10027_U_Load from $1 ($2) unit $3'#000+
  '10028_U_Recompiling $1, checksum changed for $2'#000+
  '10029_U_Recompiling $1, source found o','nly'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '10030_U_Recompiling unit, static lib is older than ppufile'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '10031_U_Recompiling unit, shared lib is older than ppufile'#000+
  '10032_U_Recompiling unit, obj and asm are older than ppufile'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '10033_U_Recompiling uni','t, obj is older than asm'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '10031','_U_Recompiling unit, shared lib is older than ppufile'#000+
  '10032_U_Recompiling unit, obj and asm are older than ppufile'#000+
  '10033_U_Recompiling unit, obj is older than asm'#000+
>>>>>>> graemeg/cpstrnew
  '10034_U_Parsing interface of $1'#000+
=======
  '10033_U_Recompiling unit, obj is older than asm'#000+
  '10034_U_P','arsing interface of $1'#000+
>>>>>>> origin/fixes_2_2
  '10035_U_Parsing implementation of $1'#000+
  '10036_U_','Second load for unit $1'#000+
  '10037_U_PPU Check file $1 time $2'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '10040_W_Can'#039't recompile unit $1, but found modified include files'#000+
  '10041_U_File $','1 is newer than the one used for creating PPU file $2'#000+
  '10042_U_Trying to use a unit which was compiled with a different FPU m'+
  'ode'#000+
  '10043_U_Loading interface units from $1'#000+
  '10044_U_Loading implementation units from $1'#000+
  '10045_U_Interface CRC chang','ed for unit $1'#000+
  '10046_U_Implementation CRC changed for unit $1'#000+
  '10047_U_Finished compiling unit $1'#000+
  '10048_U_Adding dependency: $1 depends on $2'#000+
<<<<<<< HEAD
  '10049_U_No reload, is caller: $1'#000+
  '10050_U_No reload, already in second compile: $1'#000+
  '10051_U_Flag for ','reload: $1'#000+
=======
  '10040_W_Can'#039't recompile unit $1, but found modifed include files'#000+
  '10041_U_File $1 is newer than the one used for creating PPU file $2'#000+
  '10042_U_Trying to use a unit which was compiled w','ith a different FPU'+
  ' mode'#000+
  '10043_U_Loading interface units from $1'#000+
  '10044_U_Loading implementation units from $1'#000+
=======
  '10040_W_Can'#039't recompile unit $1, but found modifed include files'#000+
  '10041_U_File $1 is newer than the one used for creating PPU file $2'#000+
  '10042_U_Trying to use a unit which was compiled w','ith a different FPU'+
  ' mode'#000+
  '10043_U_Loading interface units from $1'#000+
  '10044_U_Loading implementation units from $1'#000+
>>>>>>> graemeg/cpstrnew
=======
  '10040_W_Can'#039't recompile unit $1, but found modifed include files'#000+
  '10041_U_File $1 is newer than the one used for creating PPU file $2'#000+
  '10042_U_Trying to use a unit which was compiled w','ith a different FPU'+
  ' mode'#000+
  '10043_U_Loading interface units from $1'#000+
  '10044_U_Loading implementation units from $1'#000+
>>>>>>> graemeg/cpstrnew
=======
  '10040_W_Can'#039't recompile unit $1, but found modifed include files'#000+
  '10041_U_File $1 is newer than the one used for creating PPU file $2'#000+
  '10042_U_Trying to use a unit which was compiled w','ith a different FPU'+
  ' mode'#000+
  '10043_U_Loading interface units from $1'#000+
  '10044_U_Loading implementation units from $1'#000+
>>>>>>> origin/cpstrnew
=======
  '10033_U_Recompiling unit, obj is older than asm'#000+
  '10034_U_P','arsing interface of $1'#000+
  '10035_U_Parsing implementation of $1'#000+
  '10036_U_Second load for unit $1'#000+
  '10037_U_PPU Check file $1 time $2'#000+
  '10040_W_Can'#039't recompile unit $1, but found modifed include files'#000+
  '10041_U_File $1 is newer than PPU file $2'#000+
  '10042_U','_Trying to use a unit which was compiled with a different FPU'+
  ' mode'#000+
  '10043_U_Loading interface units from $1'#000+
  '10044_U_Loading implementation units from $1'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '10045_U_Interface CRC changed for unit $1'#000+
  '10046_U_Implementation CRC changed for unit $1',#000+
  '10047_U_Finished compiling unit $1'#000+
  '10048_','U_Adding dependency: $1 depends on $2'#000+
  '10049_U_No reload, is caller: $1'#000+
  '10050_U_No reload, already in second compile: $1'#000+
=======
  '10049_U_No reload, is caller: $1'#000+
  '10050_U_No reload, alre','ady in second compile: $1'#000+
>>>>>>> origin/fixes_2.4
  '10051_U_Flag for reload: $1'#000+
>>>>>>> graemeg/cpstrnew
  '10052_U_Forced reloading'#000+
  '10053_U_Previous state of $1: $2'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '10054_U_Already compiling $1, sett','ing second compile'#000+
  '10055_U_Loading unit $1'#000+
  '10056_U_Finished loading unit $1'#000+
<<<<<<< HEAD
  '10057_U_Registering new unit $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '10058_U_Re-resolving unit $1'#000,
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  '1','0054_U_Already compiling $1, setting second compile'#000+
  '10055_U_Loading unit $1'#000+
  '10056_U_Finished loading unit $1'#000+
  '10057_U_Registering new unit $1'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  '10057_U_Registerin','g new unit $1'#000+
>>>>>>> origin/fixes_2.4
  '10058_U_Re-resolving unit $1'#000+
>>>>>>> graemeg/cpstrnew
  '10059_U_Skipping re-resolving unit $1, still loading used units'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '10060_U_Unloading resource unit $1 (not',' needed)'#000+
  '10061_E_Unit $1 was compiled using a different whole program optimizat'+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  'ion feedback input ($2, $3); recompile it without wpo or u','se the sam'+
  'e wpo feedback input file for this compilation invocation'#000+
  '10062_U_Indirect interface (objects/classes) CRC changed for unit $1'#000+
  '10063_U_PPU is compiled for another i8086 memory model'#000+
  '11000_O_$1 [options] <inputfile> [options]'#000+
  '11001','_W_Only one source file supported, changing source file to comp'+
  'ile from "$1" into "$2"'#000+
  '11002_W_DEF file can be created only for OS/2'#000+
  '11003_E_nested response files are not supported'#000+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  'ion feedback input ($2, $3); recompile it without wpo or use the same '+
  'wpo feedback input file for this compilation invocation'#000+
  '10062_U_Indirect interface (objects','/classes) CRC changed for unit $1'+
  #000+
=======
  '10060_','U_Unloading resource unit $1 (not needed)'#000+
>>>>>>> graemeg/fixes_2_2
=======
  '10060_','U_Unloading resource unit $1 (not needed)'#000+
>>>>>>> origin/fixes_2_2
  '11000_O_$1 [options] <inputfile> [options]'#000+
  '11001_W_Only one source file supported, changing source file to compil'+
  'e from "$1" into "$2"'#000+
  '11002_W_DEF file can be created only for OS/2'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '11003_E_nested response f','iles are not supported'#000+
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
  '11004_F_No source file name in command line'#000+
  '11005_N_No opti','on inside $1 config file'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '11003_E_nested response files are not supported'#000+
  '11004_F_No source file',' name in command line'#000+
  '11005_N_No option inside $1 config file'#000+
>>>>>>> graemeg/fixes_2_2
=======
  'ion feedback in','put ($2, $3); recompile it without wpo or use the sam'+
  'e wpo feedback input file for this compilation invocation'#000+
  '10062_U_Indirect interface (objects/classes) CRC changed for unit $1'#000+
  '11000_O_$1 [options] <inputfile> [options]'#000+
  '11001_W_Only one ','source file supported, changing source file to comp'+
  'ile from "$1" into "$2"'#000+
  '11002_W_DEF file can be created only for OS/2'#000+
  '11003_E_nested response files are not supported'#000+
  '11004_F_No source file name in command line'#000+
  '11005_N_No option inside $1',' config file'#000+
>>>>>>> origin/fixes_2.4
  '11006_E_Illegal parameter: $1'#000+
  '11007_H_-? writes help pages'#000+
  '11008_F_Too many config files nested'#000+
  '11009_F_Unable to open file $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '11010_','D_Reading further options from $1'#000+
  '11011_W_Target is already set to: $1'#000+
<<<<<<< HEAD
<<<<<<< HEAD
  '11012_W_Sha','red libs not supported on DOS platform, reverting to stat'+
  'ic'#000+
  '11013_F_In options file $1 at line $2 too many #IF(N)DEFs encountered'#000+
  '11014_F_In options file $1 at line $2 unexpected #ENDIFs encountered'#000+
  '11015_F_Open conditional at the end of th','e options file'#000+
=======
  '11012_W_Shared libs not supported on DOS platform, reverting to static'+
  #000+
  '11013_F_In options file $1 at line $2 too many \var{\#IF(N)DEFs} encou'+
  'ntered'#000+
  '11014_F_In options fi','le $1 at line $2 unexpected \var{\#ENDIFs} enco'+
  'untered'#000+
  '11015_F_Open conditional at the end of the options file'#000+
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
  '11012_W_Shared libs not',' supported on DOS platform, reverting to stat'+
  'ic'#000+
  '11013_F_In options file $1 at line $2 too many \var{\#IF(N)DEFs} encou'+
  'ntered'#000+
  '11014_F_In options file $1 at line $2 unexpected \var{\#ENDIFs} encoun'+
  'tered'#000+
  '11015_F_Open conditional at the end of ','the options file'#000+
>>>>>>> origin/fixes_2.4
  '11016_W_Debug information generation is not supported by this executab'+
  'le'#000+
  '11017_H_Try recompiling with -dGDB'#000+
  '11018_W_You are using',' the obsolete switch $1'#000+
  '11019_W_You are using the obsolete switch $1, please use $2'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '11020_N_Swit','ching assembler to default source writing assembler'#000+
=======
  '11020_N_Sw','itching assembler to default source writing assembler'#000+
>>>>>>> origin/fixes_2.4
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$1" assembler use forced'#000+
=======
  '11020_N_Switching assembler to default source writing assembler'#000+
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$1" assembler us','e forced'#000+
>>>>>>> graemeg/cpstrnew
=======
  '11020_N_Switching assembler to default source writing assembler'#000+
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$1" assembler us','e forced'#000+
>>>>>>> graemeg/cpstrnew
=======
  '11020_N_Switching assembler to default source writing assembler'#000+
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$1" assembler us','e forced'#000+
>>>>>>> graemeg/cpstrnew
=======
  '11020_N_Switching assembler to default source writing assembler'#000+
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$1" assembler us','e forced'#000+
>>>>>>> origin/cpstrnew
  '11026_T_Reading options from file $1'#000+
  '11027_T_Reading options from environment $1'#000+
<<<<<<< HEAD
  '11028_','D_Handling option "$1"'#000+
=======
  '1102','8_D_Handling option "$1"'#000+
>>>>>>> origin/fixes_2.4
  '11029_O_*** press enter ***'#000+
  '11030_H_Start of reading config file $1'#000+
  '11031_H_End of reading config file $1'#000+
  '11032_D_interpr','eting option "$1"'#000+
  '11036_D_interpreting firstpass option "$1"'#000+
<<<<<<< HEAD
  '11033_D_interpreting file option "$','1"'#000+
=======
  '11010_D_Reading further options from $1'#000+
  '11011_W_Tar','get is already set to: $1'#000+
  '11012_W_Shared libs not supported on DOS platform, reverting to static'+
  #000+
  '11013_F_In options file $1 at line $2 too many #IF(N)DEFs encountered'#000+
  '11014_F_In options file $1 at line $2 unexpected #ENDIFs encountered'#000+
  '1101','5_F_Open conditional at the end of the options file'#000+
  '11016_W_Debug information generation is not supported by this executab'+
  'le'#000+
  '11017_H_Try recompiling with -dGDB'#000+
  '11018_W_You are using the obsolete switch $1'#000+
  '11019_W_You are using the obsolete ','switch $1, please use $2'#000+
  '11020_N_Switching assembler to default source writing assembler'#000+
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$1" assembler use forced'#000+
=======
  '11010_D_Reading further options from $1'#000+
  '11011_W_Tar','get is already set to: $1'#000+
  '11012_W_Shared libs not supported on DOS platform, reverting to static'+
  #000+
  '11013_F_In options file $1 at line $2 too many #IF(N)DEFs encountered'#000+
  '11014_F_In options file $1 at line $2 unexpected #ENDIFs encountered'#000+
  '1101','5_F_Open conditional at the end of the options file'#000+
  '11016_W_Debug information generation is not supported by this executab'+
  'le'#000+
  '11017_H_Try recompiling with -dGDB'#000+
  '11018_W_You are using the obsolete switch $1'#000+
  '11019_W_You are using the obsolete ','switch $1, please use $2'#000+
  '11020_N_Switching assembler to default source writing assembler'#000+
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$1" assembler use forced'#000+
>>>>>>> origin/fixes_2_2
  '11026_T_Reading options from file $1'#000+
  '11027_T_Readi','ng options from environment $1'#000+
  '11028_D_Handling option "$1"'#000+
  '11029__*** press enter ***'#000+
  '11030_H_Start of reading config file $1'#000+
  '11031_H_End of reading config file $1'#000+
  '11032_D_interpreting option "$1"'#000+
  '11036_D_interpreting firstpass option "$1"',#000+
  '11033_D_interpreting file option "$1"'#000+
>>>>>>> graemeg/fixes_2_2
  '11034_D_Reading config file "$1"'#000+
  '11035_D_found source file name "$1"'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '11039_E_Unknown codepage "$1"'#000+
  '11040_F_Config file $1 is a directory'#000+
  '11041_W_Assembler output selected "$1" cannot generate debug info, deb'+
  'ugging disabled'#000+
  '11042_W_Use of',' ppc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 #ELSE directive without #IF(N)DE'+
  'F found'#000+
  '11044_F_Option "$1" is not, or not yet, supported on the current targe'+
  't platform'#000+
  '11045_F_The feature "$1" is',' not, or not yet, supported on the select'+
  'ed target platform'#000+
  '11046_N_DWARF debug information cannot be used with smart linking on t'+
  'his target, switching to static linking'#000+
  '11047_W_Option "$1" is ignored for the current target platform.'#000+
  '11048_','W_Disabling external debug information because it is unsupport'+
  'ed for the selected target/debug format combination.'#000+
  '11049_N_DWARF debug information cannot be used with smart linking with'+
  ' external assembler, disabling static library creation.',#000+
  '11050_E_Invalid value for MACOSX_DEPLOYMENT_TARGET environment variabl'+
  'e: $1'#000+
  '11051_E_Invalid value for IPHONEOS_DEPLOYMENT_TARGET environment varia'+
  'ble: $1'#000+
  '11052_E_You must use a FPU type of VFPV2, VFPV3 or VFPV3_D16 when usin'+
  'g the EABIHF AB','I target'#000+
  '11053_W_The selected debug format is not supported on the current targ'+
  'et, not changing the current setting'#000+
  '11054_E_argument to "$1" is missing'#000+
  '11055_E_malformed parameter: $1'#000+
  '11056_W_Smart linking requires external linker'#000+
  '11057_E_C','reating .COM files is not supported in the current memory m'+
  'odel. Only the tiny memory model supports making .COM files.'#000+
  '12000_F_Cannot open whole program optimization feedback file "$1"'#000+
  '12001_D_Processing whole program optimization informat','ion in wpo fee'+
  'dback file "$1"'#000+
=======
  '11033_D_interpreting file option ','"$1"'#000+
  '11034_D_Reading config file "$1"'#000+
  '11035_D_found source file name "$1"'#000+
  '11039_E_Unknown code page'#000+
  '11040_F_Config file $1 is a directory'#000+
  '11041_W_Assembler output selected "$1" cannot generate debug info, deb'+
  'ugging disabled'#000+
  '11042_W_Use of p','pc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 \var{\#ELSE} directive without \'+
  'var{\#IF(N)DEF} found'#000+
  '11044_F_Option "$1" is not, or not yet, supported on the current targe'+
  't platform'#000+
  '11045_F_The fea','ture "$1" is not, or not yet, supported on the select'+
  'ed target platform'#000+
  '11046_N_DWARF debug information cannot be used with smart linking on t'+
  'his target, switching to static linking'#000+
  '11047_W_Option "$1" is ignored for the current target plat','form.'#000+
  '12000_F_Cannot open whole program optimization feedback file "$1"'#000+
  '12001_D_Processing whole program optimization information in wpo feedb'+
  'ack file "$1"'#000+
>>>>>>> origin/fixes_2.4
  '12002_D_Finished processing the whole program optimization information'+
  ' in wpo feedba','ck file "$1"'#000+
  '12003_E_Expected section header, but got "$2" at line $1 of wpo feedba'+
  'ck file'#000+
<<<<<<< HEAD
  '12004_W_No handler registered for w','hole program optimization section'+
  ' "$2" at line $1 of wpo feedback file, ignoring'#000+
  '12005_D_Found whole program optimization section "$1" with information'+
  ' about "$2"'#000+
  '12006_F_The selected whole program optimizations require a previously '+
  'generat','ed feedback file (use -Fw to specify)'#000+
=======
  '12004_W_No handler registered for whole program optimization section "'+
  '$2" at line $1 of wpo feedback file, ignoring'#000+
  '12005_D_Found whole program optim','ization section "$1" with informati'+
  'on about "$2"'#000+
  '12006_F_The selected whole program optimizations require a previously '+
  'generated feedback file (use -Fw to specify)'#000+
>>>>>>> origin/fixes_2.4
  '12007_E_No collected information necessary to perform "$1" whole progr'+
  'am opt','imization found'#000+
  '12008_F_Specify a whole program optimization feedback file to store th'+
  'e generated info in (using -FW)'#000+
<<<<<<< HEAD
  '12009_E_','Not generating any whole program optimization information, y'+
  'et a feedback file was specified (using -FW)'#000+
  '12010_E_Not performing any whole program optimizations, yet an input f'+
  'eedback file was specified (using -Fw)'#000+
  '12011_D_Skipping whole pro','gram optimization section "$1", because no'+
  't needed by the requested optimizations'#000+
  '12012_W_Overriding previously read information for "$1" from feedback '+
  'input file using information in section "$2"'#000+
  '12013_E_Cannot extract symbol liveness info','rmation from program when'+
  ' stripping symbols, use -Xs-'#000+
=======
  '11039_E_Unknown code page'#000+
  '11040_F_Config file $1 is a directory'#000+
<<<<<<< HEAD
  '11041_W_','Assembler output selected "$1" cannot generate debug info, d'+
  'ebugging disabled'#000+
  '11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 \var{\#ELSE} directive without \'+
  'var{\#IF(N)DEF} found'#000+
  '11','044_F_Option "$1" is not, or not yet, supported on the current tar'+
  'get platform'#000+
  '11045_F_The feature "$1" is not, or not yet, supported on the selected'+
  ' target platform'#000+
  '11046_N_DWARF debug information cannot be used with smart linking on t'+
  'his ','target, switching to static linking'#000+
  '11047_W_Option "$1" is ignored for the current target platform.'#000+
  '11048_W_Disabling external debug information because it is unsupported'+
  ' for the selected target/debug format combination.'#000+
  '12000_F_Cannot open',' whole program optimization feedback file "$1"'#000+
  '12001_D_Processing whole program optimization information in wpo feedb'+
  'ack file "$1"'#000+
  '12002_D_Finished processing the whole program optimization information'+
  ' in wpo feedback file "$1"'#000+
  '12003_E_Expe','cted section header, but got "$2" at line $1 of wpo feed'+
  'back file'#000+
  '12004_W_No handler registered for whole program optimization section "'+
  '$2" at line $1 of wpo feedback file, ignoring'#000+
  '12005_D_Found whole program optimization section "$1" with',' informati'+
  'on about "$2"'#000+
  '12006_F_The selected whole program optimizations require a previously '+
  'generated feedback file (use -Fw to specify)'#000+
  '12007_E_No collected information necessary to perform "$1" whole progr'+
  'am optimization found'#000+
=======
  '11039_E_Unknown code page'#000+
  '11040_F_Config file $1 is a directory'#000+
  '11041_W_','Assembler output selected "$1" cannot generate debug info, d'+
  'ebugging disabled'#000+
  '11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 \var{\#ELSE} directive without \'+
  'var{\#IF(N)DEF} found'#000+
  '11','044_F_Option "$1" is not, or not yet, supported on the current tar'+
  'get platform'#000+
  '11045_F_The feature "$1" is not, or not yet, supported on the selected'+
  ' target platform'#000+
  '11046_N_DWARF debug information cannot be used with smart linking on t'+
  'his ','target, switching to static linking'#000+
  '11047_W_Option "$1" is ignored for the current target platform.'#000+
  '11048_W_Disabling external debug information because it is unsupported'+
  ' for the selected target/debug format combination.'#000+
  '12000_F_Cannot open',' whole program optimization feedback file "$1"'#000+
  '12001_D_Processing whole program optimization information in wpo feedb'+
  'ack file "$1"'#000+
  '12002_D_Finished processing the whole program optimization information'+
  ' in wpo feedback file "$1"'#000+
  '12003_E_Expe','cted section header, but got "$2" at line $1 of wpo feed'+
  'back file'#000+
  '12004_W_No handler registered for whole program optimization section "'+
  '$2" at line $1 of wpo feedback file, ignoring'#000+
  '12005_D_Found whole program optimization section "$1" with',' informati'+
  'on about "$2"'#000+
  '12006_F_The selected whole program optimizations require a previously '+
  'generated feedback file (use -Fw to specify)'#000+
  '12007_E_No collected information necessary to perform "$1" whole progr'+
  'am optimization found'#000+
>>>>>>> graemeg/cpstrnew
=======
  '11039_E_Unknown code page'#000+
  '11040_F_Config file $1 is a directory'#000+
  '11041_W_','Assembler output selected "$1" cannot generate debug info, d'+
  'ebugging disabled'#000+
  '11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 \var{\#ELSE} directive without \'+
  'var{\#IF(N)DEF} found'#000+
  '11','044_F_Option "$1" is not, or not yet, supported on the current tar'+
  'get platform'#000+
  '11045_F_The feature "$1" is not, or not yet, supported on the selected'+
  ' target platform'#000+
  '11046_N_DWARF debug information cannot be used with smart linking on t'+
  'his ','target, switching to static linking'#000+
  '11047_W_Option "$1" is ignored for the current target platform.'#000+
  '11048_W_Disabling external debug information because it is unsupported'+
  ' for the selected target/debug format combination.'#000+
  '12000_F_Cannot open',' whole program optimization feedback file "$1"'#000+
  '12001_D_Processing whole program optimization information in wpo feedb'+
  'ack file "$1"'#000+
  '12002_D_Finished processing the whole program optimization information'+
  ' in wpo feedback file "$1"'#000+
  '12003_E_Expe','cted section header, but got "$2" at line $1 of wpo feed'+
  'back file'#000+
  '12004_W_No handler registered for whole program optimization section "'+
  '$2" at line $1 of wpo feedback file, ignoring'#000+
  '12005_D_Found whole program optimization section "$1" with',' informati'+
  'on about "$2"'#000+
  '12006_F_The selected whole program optimizations require a previously '+
  'generated feedback file (use -Fw to specify)'#000+
  '12007_E_No collected information necessary to perform "$1" whole progr'+
  'am optimization found'#000+
>>>>>>> graemeg/cpstrnew
=======
  '11039_E_Unknown code page'#000+
  '11040_F_Config file $1 is a directory'#000+
<<<<<<< HEAD
  '11041_W_','Assembler output selected "$1" cannot generate debug info, d'+
  'ebugging disabled'#000+
  '11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 \var{\#ELSE} directive without \'+
  'var{\#IF(N)DEF} found'#000+
  '11','044_F_Option "$1" is not, or not yet, supported on the current tar'+
  'get platform'#000+
  '11045_F_The feature "$1" is not, or not yet, supported on the selected'+
  ' target platform'#000+
  '11046_N_DWARF debug information cannot be used with smart linking on t'+
  'his ','target, switching to static linking'#000+
  '11047_W_Option "$1" is ignored for the current target platform.'#000+
  '11048_W_Disabling external debug information because it is unsupported'+
  ' for the selected target/debug format combination.'#000+
  '12000_F_Cannot open',' whole program optimization feedback file "$1"'#000+
  '12001_D_Processing whole program optimization information in wpo feedb'+
  'ack file "$1"'#000+
  '12002_D_Finished processing the whole program optimization information'+
  ' in wpo feedback file "$1"'#000+
  '12003_E_Expe','cted section header, but got "$2" at line $1 of wpo feed'+
  'back file'#000+
  '12004_W_No handler registered for whole program optimization section "'+
  '$2" at line $1 of wpo feedback file, ignoring'#000+
  '12005_D_Found whole program optimization section "$1" with',' informati'+
  'on about "$2"'#000+
  '12006_F_The selected whole program optimizations require a previously '+
  'generated feedback file (use -Fw to specify)'#000+
  '12007_E_No collected information necessary to perform "$1" whole progr'+
  'am optimization found'#000+
>>>>>>> origin/cpstrnew
  '12008_F_S','pecify a whole program optimization feedback file to store '+
  'the generated info in (using -FW)'#000+
  '12009_E_Not generating any whole program optimization information, yet'+
  ' a feedback file was specified (using -FW)'#000+
  '12010_E_Not performing any whole p','rogram optimizations, yet an input'+
  ' feedback file was specified (using -Fw)'#000+
  '12011_D_Skipping whole program optimization section "$1", because not '+
  'needed by the requested optimizations'#000+
  '12012_W_Overriding previously read information for "$1" f','rom feedbac'+
  'k input file using information in section "$2"'#000+
  '12013_E_Cannot extract symbol liveness information from program when s'+
  'tripping symbols, use -Xs-'#000+
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
  '12014_E_Cannot extract symbol liveness information from program when w'+
  'hen not linking',#000+
  '12015_F_Cannot find "$1" or "$2" to extract symbol liveness informatio'+
  'n from linked program'#000+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '12016_E_','Error during reading symbol liveness information produced by'+
  ' "$1"'#000+
=======
  '12016_E_Error during reading symbol liveness information produced by "'+
  '$1"'#000+
>>>>>>> graemeg/cpstrnew
  '12017_F_Error executing "$1" (exitcode: $2) to extract symbol informat'+
  'ion',' from linked program'#000+
  '12018_E_Collection of symbol liveness information can only help when u'+
<<<<<<< HEAD
  'sing smart',' linking, use -CX -XX'#000+
  '12019_E_Cannot create specified whole program optimisation feedback fi'+
  'le "$1"'#000+
  '11023_Free Pascal Compiler version $FPCFULLVERSION [$FPCDATE] for $FPC'+
  'CPU'#010+
  'Copyright (c) 1993-2015 by Florian Klaempfl and others'#000+
  '11024_Free ','Pascal Compiler version $FPCVERSION'#010+
=======
  'sing smart linking, use -CX -XX'#000+
  '12019_E_Cannot create specified whole program optimisation feedback fi'+
  'le "$1"'#000+
=======
  '12016_E_Error during reading symbol liveness information produced by "'+
  '$1"'#000+
  '12017_F_Error executing "$1" (exitcode: $2) to extract symbol informat'+
  'ion',' from linked program'#000+
  '12018_E_Collection of symbol liveness information can only help when u'+
  'sing smart linking, use -CX -XX'#000+
  '12019_E_Cannot create specified whole program optimisation feedback fi'+
  'le "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '12016_E_Error during reading symbol liveness information produced by "'+
  '$1"'#000+
  '12017_F_Error executing "$1" (exitcode: $2) to extract symbol informat'+
  'ion',' from linked program'#000+
  '12018_E_Collection of symbol liveness information can only help when u'+
  'sing smart linking, use -CX -XX'#000+
  '12019_E_Cannot create specified whole program optimisation feedback fi'+
  'le "$1"'#000+
>>>>>>> graemeg/cpstrnew
=======
  '12016_E_Error during reading symbol liveness information produced by "'+
  '$1"'#000+
  '12017_F_Error executing "$1" (exitcode: $2) to extract symbol informat'+
  'ion',' from linked program'#000+
=======
  '12009_E_Not generating any whole program optimization information, yet'+
  ' a feedback file was specified (using -FW)'#000+
  '12010_E_N','ot performing any whole program optimizations, yet an input'+
  ' feedback file was specified (using -Fw)'#000+
  '12011_D_Skipping whole program optimization section "$1", because not '+
  'needed by the requested optimizations'#000+
  '12012_W_Overriding previously re','ad information for "$1" from feedbac'+
  'k input file using information in section "$2"'#000+
  '12013_E_Cannot extract symbol liveness information from program when s'+
  'tripping symbols, use -Xs-'#000+
  '12014_E_Cannot extract symbol liveness information from prog','ram when'+
  ' when not linking'#000+
  '12015_F_Cannot find "$1" or "$2" to extract symbol liveness informatio'+
  'n from linked program'#000+
  '12016_E_Error during reading symbol liveness information produced by "'+
  '$1"'#000+
  '12017_F_Error executing "$1" (exitcode: $2) to e','xtract symbol inform'+
  'ation from linked program'#000+
>>>>>>> origin/fixes_2.4
  '12018_E_Collection of symbol liveness information can only help when u'+
  'sing smart linking, use -CX -XX'#000+
  '12019_E_Cannot create specified whole program optimisation feedback fi'+
  'le "$1"'#000+
<<<<<<< HEAD
>>>>>>> origin/cpstrnew
  '11023_Free Pascal Compiler version $FPC','FULLVERSION [$FPCDATE] for $F'+
  'PCCPU'#010+
  'Copyright (c) 1993-2011 by Florian Klaempfl and others'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  '11041_W_Assembler output selected "$1" cannot generate debug info, d','e'+
  'bugging disabled'#000+
  '11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 #ELSE directive without #IF(N)DE'+
  'F found'#000+
  '11023_Free Pascal Compiler version $FPCFULLVERSION [$FPCDATE] for $FPC'+
  'CPU'#010+
  'Co','pyright (c) 1993-2008 by Florian Klaempfl'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '11024_Free Pascal Compiler version $FPCVERSION'#010+
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
  '11023_Free Pas','cal Compiler version $FPCFULLVERSION [$FPCDATE] for $F'+
  'PCCPU'#010+
  'Copyright (c) 1993-2009 by Florian Klaempfl'#000+
  '11024_Free Pascal Compiler version $FPCVERSION'#010+
>>>>>>> origin/fixes_2.4
  #010+
  'Compiler date      : $FPCDATE'#010+
  'Compiler CPU target: $FPCCPU'#010+
  #010+
<<<<<<< HEAD
  'Supported targets (targets marked with '#039'{*}'#039' are under develop'+
  'ment):'#010+
  '  $OSTARGETS'#010+
=======
  'Supported targets:'#010+
  '  $OSTARG','ETS'#010+
>>>>>>> origin/fixes_2.4
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
  'Supported',' CPU instruction sets:'#010+
  '  $INSTRUCTIONSETS'#010+
  #010+
  'Supporte','d FPU instruction sets:'#010+
=======
  'Supported CPU instruction sets:'#010+
  '  $INSTRUCTIONSETS'#010+
  #010+
  'Suppo','rted FPU instruction sets:'#010+
>>>>>>> graemeg/fixes_2_2
=======
  'Supported CPU instruction sets:'#010+
  '  $INSTRUCTIONSETS'#010+
  #010+
  'Suppo','rted FPU instruction sets:'#010+
>>>>>>> origin/fixes_2_2
  '  $FPUINSTRUCTIONSETS'#010+
  #010+
  'Supported inline assembler modes:'#010+
  '  $ASMMODES'#010+
  #010+
  'Recognized compiler and RTL features:'#010+
  '  $FEATURELIST'#010+
  #010+
  'Supported ABI targets:'#010+
  '  $ABITARGETS'#010+
  #010+
  'Supported Optimizations:'#010+
  '  $OPTIMIZATIONS'#010+
  #010+
  'Supported W','hole Program Optimizations:'#010+
  '  All'#010+
<<<<<<< HEAD
  '  $WPOPTIMIZATIO','NS'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  '  ','$WPOPTIMIZATIONS'#010+
>>>>>>> origin/fixes_2.4
  #010+
  'Supported Microcontroller types:'#010+
  '  $CONTROLLERTYPES'#010+
>>>>>>> graemeg/cpstrnew
  #010+
  'Supported Microcontroller types:$\n  $CONTROLLERTYPES$\n'#010+
  'This program comes under the GNU General Public Licence'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  'For more information read COPYING.v2'#010+
=======
  'For more information read COPYING.FPC'#010+
>>>>>>> graemeg/cpstrnew
=======
  'For more information read COPYING.FPC'#010+
>>>>>>> graemeg/cpstrnew
=======
  'For more information read COPYING.FPC'#010+
>>>>>>> graemeg/cpstrnew
=======
  'For more information read COPYING.FPC'#010+
>>>>>>> origin/cpstrnew
  #010+
  'Please report bugs in our bug track','er on:'#010+
  '                 http://bugs.freepascal.org'#010+
<<<<<<< HEAD
  #010+
  'More information may be found on our WWW pages (including directions'#010+
  'for mailing lists useful for asking questions or discussing potential'#010+
  'new features, etc.):'#010+
  '                 http://www.','freepascal.org'#000+
  '11025_F*0*_Only options valid for the default or selected platform are'+
  ' listed.'#010+
  '**0*_Put + after a boolean switch option to enable it, - to disable it'+
  '.'#010+
  '**1@<x>_Read compiler options from <x> in addition to the default fpc.'+
  'cfg'#010,
  '**1a_The compiler does not delete the generated assembler file'#010+
  '**2al_List sourcecode lines in assembler file'#010+
  '**2an_List node info in assembler file (-dEXTDEBUG compiler)'#010+
  '**2ao_Add an extra option to external assembler call (ignored for inte',
  'rnal)'#010+
=======
=======
  'For more information read COPYING.FPC'#010+
  #010+
  'Report bugs, suggestions, etc. to:'#010+
  '                 http://bugs.freepascal','.org'#010+
>>>>>>> origin/fixes_2.4
  'or'#010+
  '      ','           bugs@freepascal.org'#000+
=======
=======
>>>>>>> origin/fixes_2_2
  'For more information read COPYING.FPC'#010+
  #010+
  'Report bugs,suggestions etc to:'#010+
  '                 bug','s@freepascal.org'#000+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '11025_**0*_Put + after a boolean switch option to enable it, - to disa'+
  'ble it'#010+
  '**1a_The compiler doesn'#039't delete the generated assembler file'#010+
  '**2al_List sourcecode lines in assembler file'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2an_List node info in ','assembler file'#010+
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
  '**2an_List node info in assembler file',#010+
>>>>>>> graemeg/fixes_2_2
=======
  '**2an_List node info in assembler file',#010+
>>>>>>> origin/fixes_2_2
=======
  '**2an_List',' node info in assembler file'#010+
>>>>>>> origin/fixes_2.4
  '*L2ap_Use pipes instead of creating temporary assembler files'#010+
  '**2ar_List register allocation/release info in assembler file'#010+
  '**2at_List temp allocation/release info in assembler file'#010+
  '**1A<x>_Output format:'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Adefault_Use default assem','bler'#010+
  '3*2Aas_Assemble using GNU AS'#010+
  '3*2Amacho_Mach-O (Darwin, Intel 32 bit) using internal writer'#010+
  '8*2Anasm_Assemble using Nasm'#010+
  '8*2Anasmobj_Assemble using Nasm'#010+
  '3*2Anasm_Assemble using Nasm'#010+
  '3*2Anasmcoff_COFF (Go32v2) file using Nasm'#010+
  '3*2Anasmelf','_ELF32 (Linux) file using Nasm'#010+
  '3*2Anasmwin32_Win32 object file using Nasm'#010+
  '3*2Anasmwdosx_Win32/WDOSX object file using Nasm'#010+
  '3*2Anasmdarwin_macho32 object file using Nasm (experimental)'#010+
=======
  '**2Adefault_Use defa','ult assembler'#010+
  '3*2Aas_Assemble using GNU AS'#010+
  '3*2Amacho_Mach-O (Darwin, Intel 32 bit) using internal writer'#010+
  '3*2Anasmcoff_COFF (Go32v2) file using Nasm'#010+
  '3*2Anasmelf_ELF32 (Linux) file using Nasm'#010+
  '3*2Anasmwin32_Win32 object file using Nasm'#010+
  '3*2Anas','mwdosx_Win32/WDOSX object file using Nasm'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Adefault_Use defa','ult assembler'#010+
  '3*2Aas_Assemble using GNU AS'#010+
  '3*2Amacho_Mach-O (Darwin, Intel 32 bit) using internal writer'#010+
  '3*2Anasmcoff_COFF (Go32v2) file using Nasm'#010+
  '3*2Anasmelf_ELF32 (Linux) file using Nasm'#010+
  '3*2Anasmwin32_Win32 object file using Nasm'#010+
  '3*2Anas','mwdosx_Win32/WDOSX object file using Nasm'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Adefault_Use defa','ult assembler'#010+
  '3*2Aas_Assemble using GNU AS'#010+
  '3*2Amacho_Mach-O (Darwin, Intel 32 bit) using internal writer'#010+
  '3*2Anasmcoff_COFF (Go32v2) file using Nasm'#010+
  '3*2Anasmelf_ELF32 (Linux) file using Nasm'#010+
  '3*2Anasmwin32_Win32 object file using Nasm'#010+
  '3*2Anas','mwdosx_Win32/WDOSX object file using Nasm'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Adefault_Use defa','ult assembler'#010+
  '3*2Aas_Assemble using GNU AS'#010+
  '3*2Amacho_Mach-O (Darwin, Intel 32 bit) using internal writer'#010+
  '3*2Anasmcoff_COFF (Go32v2) file using Nasm'#010+
  '3*2Anasmelf_ELF32 (Linux) file using Nasm'#010+
  '3*2Anasmwin32_Win32 object file using Nasm'#010+
  '3*2Anas','mwdosx_Win32/WDOSX object file using Nasm'#010+
>>>>>>> origin/cpstrnew
  '3*2Awasm_Obj file using Wasm (Watcom)'#010+
  '3*2Anasmobj_Obj fi','le using Nasm'#010+
  '3*2Amasm_Obj file using Masm (Microsoft)'#010+
  '3*2Atasm_Obj file using Tasm (Borland)'#010+
  '3*2Aelf_ELF (Linux) using internal writer'#010+
  '3*2Aco','ff_COFF (Go32v2) using internal writer'#010+
  '3*2Apecoff_PE-COFF (Win32) using internal writer'#010+
  '3*2Ayasm_A','ssemble using Yasm (experimental)'#010+
  '4*2Aas_Assemble using GNU AS'#010+
  '4*2Agas_Assemble using GNU GAS'#010+
  '4*2Agas-darwin_Assemble darwin Mach-O64 using GNU GAS'#010+
  '4*2Amasm_Win64 object file using ml64 (Microsoft)'#010+
  '4*2Apecoff_PE-COFF (Win64) using internal ','writer'#010+
  '4*2Aelf_ELF (Linux-64bit) using internal writer'#010+
  '4*2Ayasm_Assemble using Yasm (experimental)'#010+
  '4*2Anasm_Assemble  using Nasm (experimental)'#010+
  '4*2Anasmwin64_Assemble Win64 object file using Nasm (experimental)'#010+
  '4*2Anasmelf_Assemble Linux-64','bit object file using Nasm (experimenta'+
  'l)'#010+
  '4*2Anasmdarwin_Assemble darwin macho64 object file using Nasm (experim'+
  'ental)'#010+
  '6*2Aas_Unix o-file using GNU AS'#010+
  '6*2Agas_GNU Motorola assembler'#010+
  '6*2Amit_MIT Syntax (old GAS)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '6*2Amot_Standard Motorola ass','embler'#010+
=======
  '6*2Amot_Standard Motorola assem','bler'#010+
>>>>>>> graemeg/cpstrnew
=======
  '6*2Amot_Standard Motorola assem','bler'#010+
>>>>>>> graemeg/cpstrnew
=======
  '6*2Amot_Standard Motorola assem','bler'#010+
>>>>>>> graemeg/cpstrnew
=======
  '6*2Amot_Standard Motorola assem','bler'#010+
>>>>>>> origin/cpstrnew
  'A*2Aas_Assemble using GNU AS'#010+
  'P*2Aas_Assemble using GNU AS'#010+
  'S*2Aas_Assemble using GNU AS'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '**2Adefault_Use default assembler'#010,
=======
  '**2Ade','fault_Use default assembler'#010+
>>>>>>> origin/fixes_2.4
  '3*2Aas_Assemble using GNU AS'#010+
  '3*2Anasmcoff_COFF (Go32v2) file using Nasm'#010+
  '3*2Anasmelf_ELF32 (Linux) file using Nasm'#010+
  '3*2Anasmwin32_Win32 object file using Nasm'#010+
  '3*2Anasmwdosx_Win32/WDOSX object file using Nasm'#010+
<<<<<<< HEAD
  '3*2Awasm_Obj file using Wasm (Watc','om)'#010+
=======
  '3*2Awa','sm_Obj file using Wasm (Watcom)'#010+
>>>>>>> origin/fixes_2.4
  '3*2Anasmobj_Obj file using Nasm'#010+
  '3*2Amasm_Obj file using Masm (Microsoft)'#010+
  '3*2Atasm_Obj file using Tasm (Borland)'#010+
  '3*2Aelf_ELF (Linux) using internal writer'#010+
  '3*2Acoff_COFF (Go32v2) using internal writer'#010+
<<<<<<< HEAD
  '3*2Apecoff_PE-COFF (Win32) using inte','rnal writer'#010+
=======
  '3*2Apecof','f_PE-COFF (Win32) using internal writer'#010+
>>>>>>> origin/fixes_2.4
  '4*2Aas_Assemble using GNU AS'#010+
  '6*2Aas_Unix o-file using GNU AS'#010+
  '6*2Agas_GNU Motorola assembler'#010+
  '6*2Amit_MIT Syntax (old GAS)'#010+
  '6*2Amot_Standard Motorola assembler'#010+
  'A*2Aas_Assemble using GNU AS'#010+
<<<<<<< HEAD
  'P*2Aas_Assemble using GNU AS'#010+
  'S*2Aas_Assemb','le using GNU AS'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  'P*2Aas_Assembl','e using GNU AS'#010+
  'S*2Aas_Assemble using GNU AS'#010+
>>>>>>> origin/fixes_2.4
  '**1b_Generate browser info'#010+
  '**2bl_Generate local symbol info'#010+
  '**1B_Build all modules'#010+
  '**1C<x>_Code generation options:'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2C3_Turn on ieee error check','ing for constants'#010+
  '**2Ca<x>_Select ABI; see fpc -i or fpc -ia for possible values'#010+
  '**2Cb_Generate code for a big-endian variant of the target architectur'+
  'e'#010+
  '**2Cc<x>_Set default calling convention to <x>'#010+
  '**2CD_Create also dynamic library (not s','upported)'#010+
  '**2Ce_Compilation with emulated floating point opcodes'#010+
  '**2Cf<x>_Select fpu instruction set to use; see fpc -i or fpc -if for '+
  'possible values'#010+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '**2C3<x>_Turn on ieee error chec','king for constants'#010+
=======
>>>>>>> origin/fixes_2.4
  '**2Ca<x>_Select ABI, see fpc -i for possible values'#010+
  '**2Cb_Generate big-endian co','de'#010+
  '**2Cc<x>_Set default calling convention to <x>'#010+
  '**2CD_Create also dynamic library (not supported)'#010+
<<<<<<< HEAD
  '**2Ce_Compilation with emulated floating ','point opcodes'#010+
  '**2Cf<x>_Select fpu instruction set to use, see fpc -i for possible va'+
  'lues'#010+
>>>>>>> graemeg/cpstrnew
  '**2CF<x>_Minimal floating point constant precision (default, 32, 64)'#010+
  '**2Cg_Generate PIC c','ode'#010+
  '**2Ch<n>_<n> bytes heap (between 1023 and 67107840)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Ci_IO-checking'#010+
  'A*2CI<x>_Select instruction set on ARM: ARM or THUMB'#010+
=======
  '**2Ci_','IO-checking'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Ci_','IO-checking'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Ci_','IO-checking'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Ci_','IO-checking'#010+
>>>>>>> origin/cpstrnew
  '**2Cn_Omit linking stage'#010+
  'P*2CN_Generate nil-pointer checks (AIX-only)'#010+
  '**2Co_Check overflow of integer operations'#010,
  '**2CO_Check for possible overflow of integer operations'#010+
  '**2Cp<x>_Select instruction set; see fpc -i or fpc -ic for possible va'+
  'lues'#010+
  '**2CP<x>=<y>_ packing settings'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**3CPPACKSET=<y>_ <y> set allocation: 0, 1 or DEFAULT or NORMAL, 2, 4 '+
  'and 8'#010+
  '**','3CPPACKENUM=<y>_ <y> enum packing: 0, 1, 2 and 4 or DEFAULT or NOR'+
  'MAL'#010+
  '**3CPPACKRECORD=<y>_ <y> record packing: 0 or DEFAULT or NORMAL, 1, 2,'+
  ' 4, 8, 16 and 32'#010+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '**3CPPACK','SET=<y>_ <y> set allocation: 0, 1 or DEFAULT or NORMAL, 2, '+
  '4 and 8'#010+
>>>>>>> graemeg/cpstrnew
  '**2Cr_Range checking'#010+
  '**2CR_Verify object method call validity'#010+
  '**2Cs<n>_Set stack ch','ecking size to <n>'#010+
  '**2Ct_Stack checking (for testing only, see manual)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '8*2CT<x>_Target-specific code generation options'#010+
  '3*2CT<x>_Target-specific code generation options'#010+
  '4*2CT<x>_Target-specific code generation options'#010+
  'p*2CT<x>_Target-specif','ic code generation options'#010+
  'P*2CT<x>_Target-specific code generation options'#010+
  'J*2CT<x>_Target-specific code generation options'#010+
  'A*2CT<x>_Target-specific code generation options'#010+
  'p*3CTsmalltoc_ Generate smaller TOCs at the expense of execution s','pe'+
  'ed (AIX)'#010+
  'P*3CTsmalltoc_ Generate smaller TOCs at the expense of execution speed'+
  ' (AIX)'#010+
  'J*3CTautogetterprefix=X_  Automatically create getters for properties '+
  'with prefix X (empty string disables)'#010+
  'J*3CTautosetterprefix=X_  Automatically crea','te setters for propertie'+
  's with prefix X (empty string disables)'#010+
  '8*3CTcld_                 Emit a CLD instruction before using the x86 '+
  'string instructions'#010+
  '3*3CTcld_                 Emit a CLD instruction before using the x86 '+
  'string instructi','ons'#010+
  '4*3CTcld_                 Emit a CLD instruction before using the x86 '+
  'string instructions'#010+
  '8*3CTfarprocspushoddbp_       Increment BP before pushing it in the pr'+
  'ologue of far functions'#010+
  'J*3CTcompactintarrayinit_ Generate smaller (but pote','ntially slower) '+
  'code for initializing integer array constants'#010+
  'J*3CTenumfieldinit_       Initialize enumeration fields in constructor'+
  's to enumtype(0), after calling inherited constructors'#010+
  'J*3CTinitlocals_          Initialize local variables ','that trigger a '+
  'JVM bytecode verification error if used uninitialized (slows down code'+
  ')'#010+
  'J*3CTlowercaseprocstart_  Lowercase the first character of procedure/f'+
  'unction/method names'#010+
  'A*3CTthumbinterworking_ Generate Thumb interworking-safe code ','if pos'+
  'sible'#010+
  'J*2Cv_Var/out parameter copy-out checking'#010+
  '**2CX_Create also smartlinked library'#010+
=======
  '**2CX_Create also s','martlinked library'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2CX_Create also s','martlinked library'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2CX_Create also s','martlinked library'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2CX_Create also s','martlinked library'#010+
>>>>>>> origin/cpstrnew
=======
  '**2Cc<x>_Set default calling convention to <x>'#010+
  '**2CD_Create also dynamic library (not supported)'#010+
  '**2Ce_Compi','lation with emulated floating point opcodes'#010+
  '**2Cf<x>_Select fpu instruction set to use, see fpc -i for possible va'+
  'lues'#010+
  '**2CF<x>_Minimal floating point constant precision (default, 32, 64)'#010+
  '**2Cg_Generate PIC code'#010+
  '**2Ch<n>_<n> bytes heap (bet','ween 1023 and 67107840)'#010+
  '**2Ci_IO-checking'#010+
=======
  '**2Cc<x>_Set default calling convention to <x>'#010+
  '**2CD_Create also dynamic library (not supported)'#010+
  '**2Ce_Compi','lation with emulated floating point opcodes'#010+
  '**2Cf<x>_Select fpu instruction set to use, see fpc -i for possible va'+
  'lues'#010+
  '**2CF<x>_Minimal floating point constant precision (default, 32, 64)'#010+
  '**2Cg_Generate PIC code'#010+
  '**2Ch<n>_<n> bytes heap (bet','ween 1023 and 67107840)'#010+
  '**2Ci_IO-checking'#010+
>>>>>>> origin/fixes_2_2
  '**2Cn_Omit linking stage'#010+
  '**2Co_Check overflow of integer operations'#010+
  '**2CO_Check for possible overflow of integer operations'#010+
  '**2Cp<x>_Select instruction set, see fpc -i for possible values'#010+
  '**2CP<x>=<','y>_ packing settings'#010+
  '**3CPPACKSET=<y>_ <y> set allocation: 0, 1 or DEFAULT or NORMAL, 2, 4 '+
  'and 8'#010+
  '**2Cr_Range checking'#010+
  '**2CR_Verify object method call validity'#010+
  '**2Cs<n>_Set stack size to <n>'#010+
  '**2Ct_Stack checking'#010+
  '**2CX_Create also smartlinked',' library'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
  '**2Ce_Compilation with emulated floating point opcodes'#010+
  '**2Cf<x>_Select fpu instruction set to use, see fpc -i for possible va'+
  'lues'#010+
  '**2CF<x>_M','inimal floating point constant precision (default, 32, 64)'+
  #010+
  '**2Cg_Generate PIC code'#010+
  '**2Ch<n>_<n> bytes heap (between 1023 and 67107840)'#010+
  '**2Ci_IO-checking'#010+
  '**2Cn_Omit linking stage'#010+
  '**2Co_Check overflow of integer operations'#010+
  '**2CO_Check for pos','sible overflow of integer operations'#010+
  '**2Cp<x>_Select instruction set, see fpc -i for possible values'#010+
  '**2CP<x>=<y>_ packing settings'#010+
  '**3CPPACKSET=<y>_ <y> set allocation: 0, 1 or DEFAULT or NORMAL, 2, 4 '+
  'and 8'#010+
  '**2Cr_Range checking'#010+
  '**2CR_Verif','y object method call validity'#010+
  '**2Cs<n>_Set stack checking size to <n>'#010+
  '**2Ct_Stack checking (for testing only, see manual)'#010+
  '**2CX_Create also smartlinked library'#010+
>>>>>>> origin/fixes_2.4
  '**1d<x>_Defines the symbol <x>'#010+
  '**1D_Generate a DEF file'#010+
  '**2Dd<x>_Set description',' to <x>'#010+
  '**2Dv<x>_Set DLL version to <x>'#010+
  '*O2Dw_PM application'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '**1e<x>','_Set path to executable'#010+
  '**1E_Same as -Cn'#010+
  '**1fPIC_Same as -Cg'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**1F<x>_Set file names and paths:'#010+
=======
  '**1F<x>_Set ','file names and paths:'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**1F<x>_Set ','file names and paths:'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**1F<x>_Set ','file names and paths:'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**1F<x>_Set ','file names and paths:'#010+
>>>>>>> origin/cpstrnew
  '**2Fa<x>[,y]_(for a program) load units <x> and [y] before uses is par'+
  'sed'#010+
  '**2Fc<x>_Set input codepage to <x>'#010+
  '**2FC<x>_Set RC compiler binary name',' to <x>'#010+
  '**2Fd_Disable the compiler'#039's internal directory cache'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2FD<x>_Set the directory where to search for compiler utilities'#010+
=======
  '**2FD<x>_Se','t the directory where to search for compiler utilities'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2FD<x>_Se','t the directory where to search for compiler utilities'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2FD<x>_Se','t the directory where to search for compiler utilities'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2FD<x>_Se','t the directory where to search for compiler utilities'#010+
>>>>>>> origin/cpstrnew
  '**2Fe<x>_Redirect error output to <x>'#010+
  '**2Ff<x>_Add <x> to framework path (Darwin only)'#010+
  '**2FE<x>_Set exe/unit out','put path to <x>'#010+
  '**2Fi<x>_Add <x> to include path'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Fl<x>_Add <x> to library path'#010+
=======
  '**2Fl<x>_Add <x> to libr','ary path'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Fl<x>_Add <x> to libr','ary path'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Fl<x>_Add <x> to libr','ary path'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Fl<x>_Add <x> to libr','ary path'#010+
>>>>>>> origin/cpstrnew
  '**2FL<x>_Use <x> as dynamic linker'#010+
  '**2Fm<x>_Load unicode conversion table from <x>.txt in the compiler di'+
  'r'#010+
  '**2FM<x>_Set the directory where to search for unic','ode binary files'#010+
  '**2Fo<x>_Add <x> to object path'#010+
  '**2Fr<x>_Load error message file <x>'#010+
  '**2FR<x>_Set resource (.res) linker to <x>'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Fu<x>_Add <x> to unit path'#010+
=======
  '**2Fu<x>_Add',' <x> to unit path'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Fu<x>_Add',' <x> to unit path'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Fu<x>_Add',' <x> to unit path'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Fu<x>_Add',' <x> to unit path'#010+
>>>>>>> origin/cpstrnew
=======
  '**1F<x>_Set file names and paths:'#010+
  '**2Fa<x>[,y]_(for a program) load units <x> and [y] before uses is par'+
  'sed'#010+
  '**2','Fc<x>_Set input codepage to <x>'#010+
  '**2FC<x>_Set RC compiler binary name to <x>'#010+
  '**2Fd_Disable the compiler'#039's internal directory cache'#010+
  '**2FD<x>_Set the directory where to search for compiler utilities'#010+
  '**2Fe<x>_Redirect error output to <x>'#010+
  '**2Ff<','x>_Add <x> to framework path (Darwin only)'#010+
  '**2FE<x>_Set exe/unit output path to <x>'#010+
  '**2Fi<x>_Add <x> to include path'#010+
  '**2Fl<x>_Add <x> to library path'#010+
  '**2FL<x>_Use <x> as dynamic linker'#010+
  '**2Fm<x>_Load unicode conversion table from <x>.txt in ','the compiler '+
  'dir'#010+
  '**2Fo<x>_Add <x> to object path'#010+
  '**2Fr<x>_Load error message file <x>'#010+
  '**2FR<x>_Set resource (.res) linker to <x>'#010+
  '**2Fu<x>_Add <x> to unit path'#010+
>>>>>>> origin/fixes_2.4
  '**2FU<x>_Set unit output path to <x>, overrides -FE'#010+
  '**2FW<x>_Store generated whol','e-program optimization feedback in <x>'#010+
  '**2Fw<x>_Load previously stored whole-program optimization feedback fr'+
  'om <x>'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '*g1g_Generate debug information (default format for target)'#010+
=======
  '*g1g_Generate debug infor','mation (default format for target)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '*g1g_Generate debug infor','mation (default format for target)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '*g1g_Generate debug infor','mation (default format for target)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '*g1g_Generate debug infor','mation (default format for target)'#010+
>>>>>>> origin/cpstrnew
=======
  '*g1g_Generate debug information (default format for target)'#010+
>>>>>>> origin/fixes_2.4
  '*g2gc_Generate checks for pointers'#010+
  '*g2gh_Use heaptrace unit (for',' memory leak/corruption debugging)'#010+
  '*g2gl_Use line info unit (show more info with backtraces)'#010+
  '*g2go<x>_Set debug information options'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
  '*g3godwarfsets_ Enable DWARF '#039'set'#039' type debug information (bre'+
  'aks gdb < 6.5)'#010+
  '*g3gostabsabsincludes_ Store ab','solute/full include file paths in Sta'+
  'bs'#010+
  '*g3godwarfmethodclassprefix_ Prefix method names in DWARF with class n'+
  'ame'#010+
  '*g2gp_Preserve case in stabs symbol names'#010+
  '*g2gs_Generate Stabs debug information'#010+
<<<<<<< HEAD
  '*g2gt_Trash local variables (to detect uninit','ialized uses; multiple '+
  #039't'#039' changes the trashing value)'#010+
=======
  '*g2gt_Trash local variables (to detect uninit','ialized uses)'#010+
>>>>>>> origin/fixes_2.4
  '*g2gv_Generates programs traceable with Valgrind'#010+
  '*g2gw_Generate DWARFv2 debug information (same as -gw2)'#010+
  '*g2gw2_Generate DWARFv2 debug information'#010+
  '*g2gw3_Generate DWARFv3 debug informat','ion'#010+
  '*g2gw4_Generate DWARFv4 debug information (experimental)'#010+
  '**1i_Information'#010+
  '**2iD_Return compiler date'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '**1e<x>_Set path to executable'#010+
  '**1E_Same as -Cn'#010+
  '**1fPIC_Same as -Cg'#010+
  '**1F<x>_Set file names',' and paths:'#010+
  '**2Fa<x>[,y]_(for a program) load units <x> and [y] before uses is par'+
  'sed'#010+
  '**2Fc<x>_Set input codepage to <x>'#010+
  '**2FC<x>_Set RC compiler binary name to <x>'#010+
  '**2FD<x>_Set the directory where to search for compiler utilities'#010+
  '**2Fe<x>_','Redirect error output to <x>'#010+
  '**2Ff<x>_Add <x> to framework path (Darwin only)'#010+
  '**2FE<x>_Set exe/unit output path to <x>'#010+
  '**2Fi<x>_Add <x> to include path'#010+
  '**2Fl<x>_Add <x> to library path'#010+
  '**2FL<x>_Use <x> as dynamic linker'#010+
  '**2Fm<x>_Load unicod','e conversion table from <x>.txt in the compiler '+
  'dir'#010+
  '**2Fo<x>_Add <x> to object path'#010+
  '**2Fr<x>_Load error message file <x>'#010+
  '**2FR<x>_Set resource (.res) linker to <x>'#010+
  '**2Fu<x>_Add <x> to unit path'#010+
  '**2FU<x>_Set unit output path to <x>, override','s -FE'#010+
  '*g1g_Generate debug information (default format for target)'#010+
  '*g2gc_Generate checks for pointers'#010+
  '*g2gh_Use heaptrace unit (for memory leak/corruption debugging)'#010+
  '*g2gl_Use line info unit (show more info with backtraces)'#010+
  '*g2go<x>_Set debu','g information options'#010+
  '*g3godwarfsets_ Enable Dwarf set debug information (breaks gdb < 6.5)'#010+
  '*g2gp_Preserve case in stabs symbol names'#010+
  '*g2gs_Generate stabs debug information'#010+
  '*g2gt_Trash local variables (to detect uninitialized uses)'#010+
  '*g2gv_Ge','nerates programs traceable with valgrind'#010+
  '*g2gw_Generate dwarf-2 debug information (same as -gw2)'#010+
  '*g2gw2_Generate dwarf-2 debug information'#010+
  '*g2gw3_Generate dwarf-3 debug information'#010+
  '**1i_Information'#010+
  '**2iD_Return compiler date'#010+
  '**2iV_Return sh','ort compiler version'#010+
  '**2iW_Return full compiler version'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '**2iSO_Return compiler OS'#010+
  '**2iSP_Return compiler host processor'#010+
  '**2iTO_Return target OS'#010+
  '**2iTP_Return target processor'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '**2iV_Return sho','rt compiler version'#010+
  '**2iW_Return full compiler version'#010+
  '**2ia_Return list of supported ABI targets'#010+
  '**2ic_Return list of supported CPU instruction sets'#010+
  '**2if_Return list of supported FPU instruction sets'#010+
  '**2ii_Return list of supported inline ','assembler modes'#010+
  '**2io_Return list of supported optimizations'#010+
  '**2ir_Return list of recognized compiler and RTL features'#010+
  '**2it_Return list of supported targets'#010+
  '**2iu_Return list of supported microcontroller types'#010+
  '**2iw_Return list of supporte','d whole program optimizations'#010+
=======
  '*g3godwar','fsets_ Enable DWARF '#039'set'#039' type debug information (b'+
  'reaks gdb < 6.5)'#010+
  '*g3gostabsabsincludes_ Store absolute/full include file paths in Stabs'+
  #010+
  '*g3godwarfmethodclassprefix_ Prefix method names in DWARF with class n'+
  'ame'#010+
  '*g2gp_Preserve case in stab','s symbol names'#010+
  '*g2gs_Generate Stabs debug information'#010+
  '*g2gt_Trash local variables (to detect uninitialized uses)'#010+
  '*g2gv_Generates programs traceable with Valgrind'#010+
  '*g2gw_Generate DWARFv2 debug information (same as -gw2)'#010+
=======
  '*g3godwar','fsets_ Enable DWARF '#039'set'#039' type debug information (b'+
  'reaks gdb < 6.5)'#010+
  '*g3gostabsabsincludes_ Store absolute/full include file paths in Stabs'+
  #010+
  '*g3godwarfmethodclassprefix_ Prefix method names in DWARF with class n'+
  'ame'#010+
  '*g2gp_Preserve case in stab','s symbol names'#010+
  '*g2gs_Generate Stabs debug information'#010+
  '*g2gt_Trash local variables (to detect uninitialized uses)'#010+
  '*g2gv_Generates programs traceable with Valgrind'#010+
  '*g2gw_Generate DWARFv2 debug information (same as -gw2)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '*g3godwar','fsets_ Enable DWARF '#039'set'#039' type debug information (b'+
  'reaks gdb < 6.5)'#010+
  '*g3gostabsabsincludes_ Store absolute/full include file paths in Stabs'+
  #010+
  '*g3godwarfmethodclassprefix_ Prefix method names in DWARF with class n'+
  'ame'#010+
  '*g2gp_Preserve case in stab','s symbol names'#010+
  '*g2gs_Generate Stabs debug information'#010+
  '*g2gt_Trash local variables (to detect uninitialized uses)'#010+
  '*g2gv_Generates programs traceable with Valgrind'#010+
  '*g2gw_Generate DWARFv2 debug information (same as -gw2)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '*g3godwar','fsets_ Enable DWARF '#039'set'#039' type debug information (b'+
  'reaks gdb < 6.5)'#010+
  '*g3gostabsabsincludes_ Store absolute/full include file paths in Stabs'+
  #010+
  '*g3godwarfmethodclassprefix_ Prefix method names in DWARF with class n'+
  'ame'#010+
  '*g2gp_Preserve case in stab','s symbol names'#010+
  '*g2gs_Generate Stabs debug information'#010+
  '*g2gt_Trash local variables (to detect uninitialized uses)'#010+
  '*g2gv_Generates programs traceable with Valgrind'#010+
  '*g2gw_Generate DWARFv2 debug information (same as -gw2)'#010+
>>>>>>> origin/cpstrnew
  '*g2gw2_Generate DWARFv','2 debug information'#010+
  '*g2gw3_Generate DWARFv3 debug information'#010+
  '*g2gw4_Generate DWARFv4 debug information (experimental)'#010+
  '**1i_Information'#010+
<<<<<<< HEAD
  '**2iD_Return compiler date'#010+
=======
  '**2iD_Return compile','r date'#010+
>>>>>>> origin/fixes_2.4
  '**2iV_Return short compiler version'#010+
  '**2iW_Return full compiler version'#010+
  '**2iSO','_Return compiler OS'#010+
  '**2iSP_Return compiler host processor'#010+
  '**2iTO_Return target OS'#010+
  '**2iTP_Return target processor'#010+
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
  '**1I<x>_Add <x> to include path'#010+
  '**1k<x>_Pass <x> to the linker'#010+
=======
  '**1I<x>_Add <x> to include path'#010+
  '**1k<x>_Pas','s <x> to the linker'#010+
>>>>>>> origin/fixes_2.4
  '**1l_Write logo'#010+
  '**1M<x>_Set language mode to <x>'#010+
  '**2Mfpc_Free Pa','scal dialect (default)'#010+
  '**2Mobjfpc_FPC mode with Object Pascal support'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Mdelphi_De','lphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_Macintosh Pascal dialects compatibility mode'#010+
=======
  '**2Mdelphi_Delphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_','Macintosh Pascal dialects compatibility mode'#010+
>>>>>>> origin/fixes_2.4
  '**1n_Do not read the default config files'#010+
  '**1o<x>_Change the name of the executable produced to <x>'#010+
  '**1O<x>_Optimizations:',#010+
=======
  '**2Mdelphi_Delphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_Macintosh Pascal dialects compatibility mode'#010+
=======
  '**2Mdelphi_Delphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_Macintosh Pascal dialects compatibility mode'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Mdelphi_Delphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_Macintosh Pascal dialects compatibility mode'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Mdelphi_Delphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_Macintosh Pascal dialects compatibility mode'#010+
>>>>>>> origin/cpstrnew
  '**1n_Do not read the default config fil','es'#010+
  '**1N<x>_Node tree optimizations'#010+
  '**2Nu_Unroll loops'#010+
  '**1o<x>_Change the name of the executable produced to <x>'#010+
  '**1O<x>_Optimizations:'#010+
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
  '**2O-_Disable optimizations'#010+
=======
  '**2O-_Disable optimiz','ations'#010+
>>>>>>> origin/fixes_2.4
  '**2O1_Level 1 optimizations (quick and debugger friendly)'#010+
  '**2O2_Level 2 optim','izations (-O1 + quick optimizations)'#010+
  '**2O3_Level 3 optimizations (-O2 + slow optimizations)'#010+
  '**2O4_Level 4 optimizations (-O3 + optimiz','ations which might have un'+
  'expected side effects)'#010+
  '**2Oa<x>=<y>_Set alignment'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Oo[NO]<x>_Enable or disable optimizations; see fpc -i or fpc -io fo'+
  'r possible values'#010+
  '**2Op<x>_Set target cpu for optimizing; see fpc -i or fpc -ic for poss'+
  'ible v','alues'#010+
  '**2OW<x>_Generate whole-program optimization feedback for optimization'+
  ' <x>; see fpc -i or fpc -iw for possible values'#010+
  '**2Ow<x>_Perform whole-program optimization <x>; see fpc -i or fpc -iw'+
  ' for possible values'#010+
  '**2Os_Optimize for size r','ather than speed'#010+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '**2Oo[NO]<x>_Enable or disable optimizations, see fpc -i for possible '+
  'values'#010+
  '**2Op<x>_Set target cpu for optimizing, see ','fpc -i for possible valu'+
  'es'#010+
  '**2OW<x>_Generate whole-program optimization feedback for optimization'+
  ' <x>, see fpc -i for possible values'#010+
  '**2Ow<x>_Perform whole-program optimization <x>, see fpc -i for possib'+
  'le values'#010+
  '**2Os_Optimize for size ra','ther than speed'#010+
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
  '**1pg_Generate profile code for gprof (defines FPC_PROFILE)'#010+
  'F*1P<x>_Target CPU / compiler related options:'#010+
  'F*2PB_Show default compiler binary'#010+
  'F*2PP_Show default target cpu'#010+
  'F*2P<x>_Set target CPU (arm,avr,i386,jvm,m68k,mips,','mipsel,powerpc,po'+
  'werpc64,sparc,x86_64)'#010+
  '**1R<x>_Assembler reading style:'#010+
  '**2Rdefault_Use default assembler for target'#010+
  '3*2Ratt_Read AT&T style assembler'#010+
  '3*2Rintel_Read Intel style assembler'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '6*2RMOT_Read motorola style assembler'#010+
  '**1S<x>_Syntax',' options:'#010+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '6*2RMOT_Read mo','torola style assembler'#010+
  '**1S<x>_Syntax options:'#010+
>>>>>>> graemeg/cpstrnew
  '**2S2_Same as -Mobjfpc'#010+
  '**2Sc_Support operators like C (*=,+=,/= and -=)'#010+
  '**2Sa_Turn on assertions'#010+
  '**2Sd_Same as -Mdelphi'#010+
  '**2Se<x>_Error options. <x> is a combination of the following:'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**3*_<n> : Compiler halts after the <n> errors ','(default is 1)'#010+
  '**3*_w : Compiler also halts after warnings'#010+
  '**3*_n : Compiler also halts after notes'#010+
  '**3*_h : Compiler also halts after hints'#010+
  '**2Sf_Enable certain features in compiler and RTL; see fpc -i or fpc -'+
  'ir for possible values)'#010+
  '**2Sg','_Enable LABEL and GOTO (default in -Mtp and -Mdelphi)'#010+
  '**2Sh_Use reference counted strings (ansistring by default) instead of'+
  ' shortstrings'#010+
  '**2Si_Turn on inlining of procedures/functions declared as "inline"'#010+
  '**2Sk_Load fpcylix unit'#010+
  '**2SI<x>_S','et interface style to <x>'#010+
=======
=======
>>>>>>> origin/cpstrnew
  '**3*_<n> :',' Compiler halts after the <n> errors (default is 1)'#010+
  '**3*_w : Compiler also halts after warnings'#010+
  '**3*_n : Compiler also halts after notes'#010+
  '**3*_h : Compiler also halts after hints'#010+
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
  '**3*_<n> :',' Compiler halts after the <n> errors (default is 1)'#010+
  '**3*_w : Compiler also halts after warnings'#010+
  '**3*_n : Compiler also halts after notes'#010+
  '**3*_h : Compiler also halts after hints'#010+
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '**2Sg_Enable LABEL and GOTO (default in -Mtp and -Mdelphi)'#010+
  '**2','Sh_Use ansistrings by default instead of shortstrings'#010+
  '**2Si_Turn on inlining of procedures/functions declared as "inline"'#010+
  '**2Sk_Load fpcylix unit'#010+
  '**2SI<x>_Set interface style to <x>'#010+
>>>>>>> graemeg/cpstrnew
  '**3SIcom_COM compatible interface (default)'#010+
  '**3SIcorba_COR','BA compatible interface'#010+
  '**2Sm_Support macros like C (global)'#010+
  '**2So_Same as -Mtp'#010+
  '**2Ss_Constructor name must be init (destructor must be done)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Sv_Support ','vector processing (use CPU vector extensions if availa'+
  'ble)'#010+
  '**2Sx_Enable exception keywords (default in Delphi/ObjFPC modes)'#010+
  '**2Sy_@<pointer> returns a typed pointer, same as $T+'#010+
  '**1s_Do not call assembler and linker'#010+
  '**2sh_Generate script to',' link on host'#010+
  '**2st_Generate script to link on target'#010+
  '**2sr_Skip register allocation phase (use with -alr)'#010+
  '**1T<x>_Target operating system:'#010+
  '3*2Tandroid_Android'#010+
  '3*2Taros_AROS'#010+
  '3*2Tbeos_BeOS'#010+
  '3*2Tdarwin_Darwin/Mac OS X'#010+
  '3*2Tembedded_Embedded'#010+
  '3*2','Temx_OS/2 via EMX (including EMX/RSX extender)'#010+
  '3*2Tfreebsd_FreeBSD'#010+
  '3*2Tgo32v2_Version 2 of DJ Delorie DOS extender'#010+
  '3*2Thaiku_Haiku'#010+
=======
  '**2Sx_Enable exception keywords (default in Delphi/ObjFPC modes)'#010+
=======
  '**2Sx_Enable exception keywords (default in Delphi/ObjFPC modes)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Sx_Enable exception keywords (default in Delphi/ObjFPC modes)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Sx_Enable exception keywords (default in Delphi/ObjFPC modes)'#010+
>>>>>>> origin/cpstrnew
  '**1s_Do not call assembler and li','nker'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '**1I<x>_Add <x> to include path'#010+
  '**1k<x>_Pass <x> to the linker'#010+
  '**','1l_Write logo'#010+
  '**1M<x>_Set language mode to <x>'#010+
  '**2Mfpc_Free Pascal dialect (default)'#010+
  '**2Mobjfpc_FPC mode with Object Pascal support'#010+
  '**2Mdelphi_Delphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_Macintosh Pascal diale','cts compatibility mode'#010+
  '**1n_Do not read the default config files'#010+
  '**1N<x>_Node tree optimizations'#010+
  '**2Nu_Unroll loops'#010+
  '**1o<x>_Change the name of the executable produced to <x>'#010+
  '**1O<x>_Optimizations:'#010+
  '**2O-_Disable optimizations'#010+
  '**2O1_Level 1 o','ptimizations (quick and debugger friendly)'#010+
  '**2O2_Level 2 optimizations (-O1 + quick optimizations)'#010+
  '**2O3_Level 3 optimizations (-O2 + slow optimizations)'#010+
  '**2Oa<x>=<y>_Set alignment'#010+
<<<<<<< HEAD
  '**2Oo[NO]<x>_Enable or disable optimizations, see fpc -i fo','r possibl'+
  'e values'#010+
  '**2Op<x>_Set target cpu for optimizing, see fpc -i for possible values'+
  #010+
  '**2Os_Optimize for size rather than speed'#010+
  '**1pg_Generate profile code for gprof (defines FPC_PROFILE)'#010+
  '**1R<x>_Assembler reading style:'#010+
  '**2Rdefault_Use ','default assembler for target'#010+
  '3*2Ratt_Read AT&T style assembler'#010+
=======
  '**2Oo[NO]<x>_Enable or disable optimi','zations, see fpc -i for possibl'+
  'e values'#010+
  '**2Op<x>_Set target cpu for optimizing, see fpc -i for possible values'+
  #010+
  '**2OW<x>_Generate whole-program optimization feedback for optimization'+
  ' <x>, see fpc -i for possible values'#010+
  '**2Ow<x>_Perform whole','-program optimization <x>, see fpc -i for poss'+
  'ible values'#010+
  '**2Os_Optimize for size rather than speed'#010+
  '**1pg_Generate profile code for gprof (defines FPC_PROFILE)'#010+
  '**1R<x>_Assembler reading style:'#010+
  '**2Rdefault_Use default assembler for target'#010+
  '3*','2Ratt_Read AT&T style assembler'#010+
>>>>>>> origin/fixes_2.4
  '3*2Rintel_Read Intel style assembler'#010+
  '6*2RMOT_Read motorola style assembler'#010+
  '**1S<x>_Syntax options:'#010+
  '**2S2_Same as -Mobjfpc'#010+
  '**2Sc_Support operators like C (*=,+=,/= and -=)'#010+
<<<<<<< HEAD
  '**2Sa_','Turn on assertions'#010+
  '**2Sd_Same as -Mdelphi'#010+
  '**2Se<x>_Error options. <x> is a combination of the following:'#010+
  '**3*_<n> : Compiler halts after the <n> errors (default is 1)'#010+
  '**3*_w : Compiler also halts after warnings'#010+
  '**3*_n : Compiler also halts ','after notes'#010+
  '**3*_h : Compiler also halts after hints'#010+
  '**2Sg_Enable LABEL and GOTO (default in -Mtp and -Mdelphi)'#010+
  '**2Sh_Use ansistrings by default instead of shortstrings'#010+
  '**2Si_Turn on inlining of procedures/functions declared as "inline"'#010+
  '**2','Sk_Load fpcylix unit'#010+
  '**2SI<x>_Set interface style to <x>'#010+
=======
  '**2Sa_Turn on assertions'#010+
  '**2Sd_Same a','s -Mdelphi'#010+
  '**2Se<x>_Error options. <x> is a combination of the following:'#010+
  '**3*_<n> : Compiler halts after the <n> errors (default is 1)'#010+
  '**3*_w : Compiler also halts after warnings'#010+
  '**3*_n : Compiler also halts after notes'#010+
  '**3*_h : Compiler a','lso halts after hints'#010+
  '**2Sg_Enable LABEL and GOTO (default in -Mtp and -Mdelphi)'#010+
  '**2Sh_Use ansistrings by default instead of shortstrings'#010+
  '**2Si_Turn on inlining of procedures/functions declared as "inline"'#010+
  '**2Sk_Load fpcylix unit'#010+
  '**2SI<x>_S','et interface style to <x>'#010+
>>>>>>> origin/fixes_2.4
  '**3SIcom_COM compatible interface (default)'#010+
  '**3SIcorba_CORBA compatible interface'#010+
  '**2Sm_Support macros like C (global)'#010+
  '**2So_Same as -Mtp'#010+
<<<<<<< HEAD
  '**2Ss_Constructor name must be init (destruct','or must be done)'#010+
  '**2St_Allow static keyword in objects'#010+
=======
  '**2Ss_Constructor name must be init (destructor must be done)'#010+
  '**2St_Allow st','atic keyword in objects'#010+
>>>>>>> origin/fixes_2.4
  '**2Sx_Enable exception keywords (default in Delphi/ObjFPC modes)'#010+
  '**1s_Do not call assembler and linker'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '**2sh_Generate script to link on host'#010+
  '**2st_Generate script to link on target'#010+
<<<<<<< HEAD
  '**2s','r_Skip register allocation phase (use with -alr)'#010+
=======
  '**2sr_Skip register allocation phas','e (use with -alr)'#010+
>>>>>>> origin/fixes_2.4
  '**1T<x>_Target operating system:'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '3*2Tdarwin_Darwin/Mac OS X'#010+
  '3*2Temx_OS/2 via EMX (including EMX/RSX exte','nder)'#010+
=======
  '3*2Temx_OS/2 via EMX (including EMX/RSX extender)'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '3*2Temx_OS/2 via EMX (including EMX/RSX extender)'#010+
>>>>>>> origin/fixes_2_2
  '3*2Tfreebsd_FreeBSD'#010+
  '3*2Tgo32v2_Version 2 of DJ Delorie DOS extender'#010+
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
  '3*2Tiphonesim_ iPhoneSimulator from iOS SDK 3.2+ (older versions: -Tda'+
  'rwin)'#010+
  '3*2Tlinux_Linux'#010+
  '3*2Tnativent_Nati','ve NT API (experimental)'#010+
  '3*2Tnetbsd_NetBSD'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '3*2Tnetware_Novell Netware Module (clib)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
  '3*2Tne','tware_Novell Netware Module (clib)'#010+
>>>>>>> graemeg/fixes_2_2
=======
  '3*2Tne','tware_Novell Netware Module (clib)'#010+
>>>>>>> origin/fixes_2_2
=======
  '3*2Tnetware_Novell Netware Module (cl','ib)'#010+
>>>>>>> origin/fixes_2.4
  '3*2Tnetwlibc_Novell Netware Module (libc)'#010+
=======
  '3*2Tnetwlibc_No','vell Netware Module (libc)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2Tnetwlibc_No','vell Netware Module (libc)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2Tnetwlibc_No','vell Netware Module (libc)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2Tnetwlibc_No','vell Netware Module (libc)'#010+
>>>>>>> origin/cpstrnew
  '3*2Topenbsd_OpenBSD'#010+
  '3*2Tos2_OS/2 / eComStation'#010+
  '3*2Tsunos_SunOS/Solaris'#010+
  '3*2Tsymbian_Symbian OS'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '3*2Tsolaris_Solaris'#010,
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '3*2Twatcom_Watcom compatible DOS extender'#010+
<<<<<<< HEAD
  '3*2Twdosx_WDOSX DOS extende','r'#010+
  '3*2Twin32_Windows 32 Bit'#010+
  '3*2Twince_Windows CE'#010+
  '4*2Tdarwin_Darwin/Mac OS X'#010+
  '4*2Tfreebsd_FreeBSD'#010+
  '4*2Tiphonesim_ iPhoneSimulator'#010+
  '4*2Tlinux_Linux'#010+
  '4*2Tnetbsd_NetBSD'#010+
  '4*2Topenbsd','_OpenBSD'#010+
  '4*2Tsolaris_Solaris'#010+
=======
  '3*2Tsolaris_Solaris'#010+
  '3*2Twatcom_Watcom compatible DOS extender'#010+
  '3*2Twdosx_WDOSX DOS extender'#010+
  '3*2Twin32_Windows 32 Bit'#010+
  '3*2','Twince_Windows CE'#010+
  '4*2Tdarwin_Darwin/Mac OS X'#010+
  '4*2Tlinux_Linux'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2Tsolaris_Solaris'#010+
  '3*2Twatcom_Watcom compatible DOS extender'#010+
  '3*2Twdosx_WDOSX DOS extender'#010+
  '3*2Twin32_Windows 32 Bit'#010+
  '3*2','Twince_Windows CE'#010+
  '4*2Tdarwin_Darwin/Mac OS X'#010+
  '4*2Tlinux_Linux'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2Tsolaris_Solaris'#010+
  '3*2Twatcom_Watcom compatible DOS extender'#010+
  '3*2Twdosx_WDOSX DOS extender'#010+
  '3*2Twin32_Windows 32 Bit'#010+
  '3*2','Twince_Windows CE'#010+
  '4*2Tdarwin_Darwin/Mac OS X'#010+
  '4*2Tlinux_Linux'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2Tsolaris_Solaris'#010+
  '3*2Twatcom_Watcom compatible DOS extender'#010+
  '3*2Twdosx_WDOSX DOS extender'#010+
  '3*2Twin32_Windows 32 Bit'#010+
  '3*2','Twince_Windows CE'#010+
  '4*2Tdarwin_Darwin/Mac OS X'#010+
=======
  '3*2Twdosx_WDOSX DOS extender'#010+
  '3*2Twin32_Windows 32 Bit'#010+
  '3*2T','wince_Windows CE'#010+
>>>>>>> origin/fixes_2.4
  '4*2Tlinux_Linux'#010+
>>>>>>> origin/cpstrnew
  '4*2Twin64_Win64 (64 bit Windows systems)'#010+
  '6*2Tamiga_Commodore Amiga'#010+
  '6*2Tatari_Atari ST/STe/TT'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '6*2Tlinux_Linux'#010+
=======
=======
>>>>>>> origin/fixes_2_2
  '6*2Tlinux_Linux/m68k'#010+
  '6*2Tmacos_Macintosh m68k (not supported)'#010+
>>>>>>> graemeg/fixes_2_2
  '6*2Tpalmos_PalmOS'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '8*2Tmsdos_MS-DOS (and compatible)'#010+
  'a*2Tdarwin_Darwin/iOS'#010+
  'A*2Tandroid_Android'#010+
  'A*2Tdarw','in_Darwin/iPhoneOS/iOS'#010+
  'A*2Tembedded_Embedded'#010+
  'A*2Tgba_Game Boy Advance'#010+
  'A*2Tlinux_Linux'#010+
  'A*2Tnds_Nintendo DS'#010+
  'A*2Tsymbian_Symbian'#010+
  'A*2Twince_Windows CE'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  'J*2Tandroid_Android'#010+
  'J*2Tjava_Java'#010+
  'm*2Tembedded_Embedded'#010+
  'M*2Tandroid_Android'#010+
  'M*2Tembedded_Embe','dded'#010+
  'M*2Tlinux_Linux'#010+
  'P*2Taix_AIX'#010+
=======
  'A*2Tdarwin_Darwin/iPhoneOS/iOS'#010+
  'A*2Tlinux_Linux'#010+
  'A*2Tw','ince_Windows CE'#010+
>>>>>>> graemeg/cpstrnew
=======
  'A*2Tdarwin_Darwin/iPhoneOS/iOS'#010+
  'A*2Tlinux_Linux'#010+
  'A*2Tw','ince_Windows CE'#010+
>>>>>>> graemeg/cpstrnew
=======
  'A*2Tdarwin_Darwin/iPhoneOS/iOS'#010+
  'A*2Tlinux_Linux'#010+
  'A*2Tw','ince_Windows CE'#010+
>>>>>>> graemeg/cpstrnew
=======
  'A*2Tdarwin_Darwin/iPhoneOS/iOS'#010+
  'A*2Tlinux_Linux'#010+
  'A*2Tw','ince_Windows CE'#010+
>>>>>>> origin/cpstrnew
  'P*2Tamiga_AmigaOS'#010+
  'P*2Tdarwin_Darwin/Mac OS X'#010+
  'P*2Tlinux_Linux'#010+
  'P*2Tmacos_Mac OS (classic)'#010+
=======
  'P*2Tamiga_AmigaOS on PowerPC'#010+
  'P*2Tdarwi','n_Darwin and Mac OS X on PowerPC'#010+
  'P*2Tlinux_Linux on PowerPC'#010+
  'P*2Tmacos_Mac OS (classic) on PowerPC'#010+
>>>>>>> origin/fixes_2.4
  'P*2Tmorphos_MorphOS'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  'P*2Tnetbsd_NetBSD'#010+
  'P*2Twii_Wii'#010+
  'p*2Taix_AIX'#010+
  'p*2Tdarwin_Darwin/Mac OS X'#010+
  'p*2Tlinux_Linux'#010+
  'S*2Tsolaris_So','laris'#010+
=======
  'S*2Tsolaris_Solaris'#010+
>>>>>>> graemeg/cpstrnew
=======
  'S*2Tsolaris_Solaris'#010+
>>>>>>> graemeg/cpstrnew
=======
  'S*2Tsolaris_Solaris'#010+
>>>>>>> graemeg/cpstrnew
=======
  'S*2Tsolaris_Solaris'#010+
>>>>>>> origin/cpstrnew
  'S*2Tlinux_Linux'#010+
  'V*2Tembedded_Embedded'#010+
  '**1u<x>_Undefines the symbol <x>'#010+
  '**1U_Unit options:'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Un_Do not check where the unit name matches the file name'#010+
=======
  '**2Un_Do not check where the',' unit name matches the file name'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Un_Do not check where the',' unit name matches the file name'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Un_Do not check where the',' unit name matches the file name'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2Un_Do not check where the',' unit name matches the file name'#010+
>>>>>>> origin/cpstrnew
=======
  '**2Un_Do not check where the unit name matches the fil','e name'#010+
>>>>>>> origin/fixes_2.4
  '**2Ur_Generate release unit files (never automatically recompiled)'#010+
  '**2Us_Compile a ','system unit'#010+
  '**1v<x>_Be verbose. <x> is a combination of the following letters:'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2*_e : Show errors (default)       0 : Show nothing (except errors)'#010+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '**2*_e : Show errors (default)       0 : Show',' nothing (except errors'+
  ')'#010+
>>>>>>> graemeg/cpstrnew
  '**2*_w : Show warnings               u : Show unit info'#010+
  '**2*_n : Show notes                ','  t : Show tried/used files'#010+
  '**2*_h : Show hints                  c : Show conditionals'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2*_i : Show general info           d : Show debug info'#010+
  '**2*_l : Show linenumbers            r : Rhide/GCC compatibility mode'#010+
  '**2*_s : Show time stamps ','           q : Show message numbers'#010+
  '**2*_a : Show everything             x : Show info about invoked tools'+
  #010+
  '**2*_b : Write file names messages   p : Write tree.log with parse tre'+
  'e'#010+
  '**2*_    with full path              v : Write fpcdebug.txt w','ith'#010+
  '**2*_z : Write output to stderr          lots of debugging info'#010+
  '**2*_m<x>,<y> : Do not show messages numbered <x> and <y>'#010+
  'F*1V<x>_Append '#039'-<x>'#039' to the used compiler binary name (e.g. f'+
  'or version)'#010+
  '**1W<x>_Target-specific options (targets',')'#010+
  '3*2WA_Specify native type application (Windows)'#010+
  '4*2WA_Specify native type application (Windows)'#010+
  'A*2WA_Specify native type application (Windows)'#010+
  '3*2Wb_Create a bundle instead of a library (Darwin)'#010+
  'P*2Wb_Create a bundle instead of a library',' (Darwin)'#010+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  '**2*_i : Show general info           ','d : Show debug info'#010+
  '**2*_l : Show linenumbers            r : Rhide/GCC compatibility mode'#010+
  '**2*_s : Show time stamps            q : Show message numbers'#010+
  '**2*_a : Show everything             x : Executable info (Win32 only)'#010+
  '**2*_b : Write fil','e names messages   p : Write tree.log with parse t'+
  'ree'#010+
  '**2*_    with full path              v : Write fpcdebug.txt with'#010+
  '**2*_                                    lots of debugging info'#010+
  '**2*_m<x>,<y> : Don'#039't show messages numbered <x> and <y>'#010,
  '3*1W<x>_Target-specific options (targets)'#010+
=======
  '**2*_e : Show errors (default)       0 : Show nothing (except errors)'#010+
  '*','*2*_w : Show warnings               u : Show unit info'#010+
  '**2*_n : Show notes                  t : Show tried/used files'#010+
  '**2*_h : Show hints                  c : Show conditionals'#010+
  '**2*_i : Show general info           d : Show debug info'#010+
  '**2*_l',' : Show linenumbers            r : Rhide/GCC compatibility mod'+
  'e'#010+
  '**2*_s : Show time stamps            q : Show message numbers'#010+
  '**2*_a : Show everything             x : Executable info (Win32 only)'#010+
  '**2*_b : Write file names messages   p : Wri','te tree.log with parse t'+
  'ree'#010+
  '**2*_    with full path              v : Write fpcdebug.txt with'#010+
  '**2*_                                    lots of debugging info'#010+
  '**2*_m<x>,<y> : Don'#039't show messages numbered <x> and <y>'#010+
  '3*1W<x>_Target-specific op','tions (targets)'#010+
>>>>>>> origin/fixes_2.4
  'A*1W<x>_Target-specific options (targets)'#010+
  'P*1W<x>_Target-specific options (targets)'#010+
  'p*1W<x>_Target-specific options (targets)'#010+
  '3*2Wb_Create a bundle instead of a library (Darwin)'#010+
<<<<<<< HEAD
  'P*2Wb_Create a bundl','e instead of a library (Darwin)'#010+
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
  'P*2Wb_Create a bundle instead of a library (Da','rwin)'#010+
>>>>>>> origin/fixes_2.4
  'p*2Wb_Create a bundle instead of a library (Darwin)'#010+
  'a*2Wb_Create a bundle instead of a library (Darwin)'#010+
  'A*2Wb_Create a bundle instead of a library (Darwin)'#010+
  '4*2Wb_Create a bundle instead of a library (Darwin)'#010+
  '3*2WB_Create a relocat','able image (Windows, Symbian)'#010+
  '3*2WB<x>_Set image base to <x> (Windows, Symbian)'#010+
  '4*2WB_Create a relocatable image (Windows)'#010+
  '4*2WB<x>_Set image base to <x> (Windows)'#010+
  'A*2WB_Create a relocatable image (Windows, Symbian)'#010+
  'A*2WB<x>_Set image base ','to <x> (Windows, Symbian)'#010+
  '3*2WC_Specify console type application (EMX, OS/2, Windows)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '4*2WC_Specify console type application (Windows)'#010+
  'A*2WC_Specify console type application (Windows)'#010+
  'P*2WC_Specify console type application (Classic Mac OS)'#010,
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  'A','*2WC_Specify console type application (Windows)'#010+
=======
  'A*2WC_Specify console type ','application (Windows)'#010+
>>>>>>> origin/fixes_2.4
  'P*2WC_Specify console type application (Classic Mac OS)'#010+
>>>>>>> graemeg/cpstrnew
  '3*2WD_Use DEFFILE to export functions of DLL or EXE (Windows)'#010+
  '4*2WD_Use DEFFILE to export functions of DLL or EXE (Windows)'#010+
  'A*2WD_Use DEFFILE to export functions of DLL or EXE (Windows)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '3*2We_Use external resources (Darwin)'#010+
  '4*2We_Use extern','al resources (Darwin)'#010+
  'a*2We_Use external resources (Darwin)'#010+
=======
  '3*2We_Use ex','ternal resources (Darwin)'#010+
  '4*2We_Use external resources (Darwin)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2We_Use ex','ternal resources (Darwin)'#010+
  '4*2We_Use external resources (Darwin)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2We_Use ex','ternal resources (Darwin)'#010+
  '4*2We_Use external resources (Darwin)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2We_Use ex','ternal resources (Darwin)'#010+
  '4*2We_Use external resources (Darwin)'#010+
>>>>>>> origin/cpstrnew
  'A*2We_Use external resources (Darwin)'#010+
  'P*2We_Use external resources (Darwin)'#010+
  'p*2We_Use external resources (Darwin)'#010+
  '3*2WF_Specify full-screen type application (EMX, OS/2)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '3*2WG_Speci','fy graphic type application (EMX, OS/2, Windows)'#010+
  '4*2WG_Specify graphic type application (Windows)'#010+
=======
  '3*2WG_S','pecify graphic type application (EMX, OS/2, Windows)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2WG_S','pecify graphic type application (EMX, OS/2, Windows)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2WG_S','pecify graphic type application (EMX, OS/2, Windows)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '3*2WG_S','pecify graphic type application (EMX, OS/2, Windows)'#010+
>>>>>>> origin/cpstrnew
  'A*2WG_Specify graphic type application (Windows)'#010+
=======
  '3*2We_Use external resources (Darwin)'#010,
  'P*2We_Use external resources (Darwin)'#010+
  'p*2We_Use external resources (Darwin)'#010+
  '3*2WF_Specify full-screen type application (EMX, OS/2)'#010+
  '3*2WG_Specify graphic type application (EMX, OS/2, Windows)'#010+
  'A*2WG_Specify graphic type application (Windows)'#010,
>>>>>>> origin/fixes_2.4
  'P*2WG_Specify graphic type application (Classic Mac OS)'#010+
  '3*2Wi_Use internal resources (Darwin)',#010+
  '4*2Wi_Use internal resources (Darwin)'#010+
  'a*2Wi_Use internal resources (Darwin)'#010+
  'A*2Wi_Use internal resources (Darwin)'#010+
  'P*2Wi_Use internal resources (Darwin)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  'p*2Wi_Use internal resources (Darwin)'#010+
<<<<<<< HEAD
  '3*2WI_Turn on/off the usage of import sections (W','indows)'#010+
  '4*2WI_Turn on/off the usage of import sections (Windows)'#010+
  'A*2WI_Turn on/off the usage of import sections (Windows)'#010+
  '8*2Wm<x>_Set memory model'#010+
  '8*3WmTiny_Tiny memory model'#010+
  '8*3WmSmall_Small memory model (default)'#010+
  '8*3WmMedium_Medium memor','y model'#010+
  '8*3WmCompact_Compact memory model'#010+
  '8*3WmLarge_Large memory model'#010+
  '8*3WmHuge_Huge memory model'#010+
  '3*2WM<x>_Minimum Mac OS X deployment version: 10.4, 10.5.1, ... (Darwi'+
  'n)'#010+
  '4*2WM<x>_Minimum Mac OS X deployment version: 10.4, 10.5.1, ... (Da','r'+
  'win)'#010+
  'p*2WM<x>_Minimum Mac OS X deployment version: 10.4, 10.5.1, ... (Darwi'+
  'n)'#010+
  'P*2WM<x>_Minimum Mac OS X deployment version: 10.4, 10.5.1, ... (Darwi'+
  'n)'#010+
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
  '4*2WN_Do not gener','ate relocation code, needed for debugging (Windows'+
  ')'#010+
=======
  'p*2Wi_','Use internal resources (Darwin)'#010+
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
>>>>>>> graemeg/cpstrnew
=======
  'p*2Wi_','Use internal resources (Darwin)'#010+
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
>>>>>>> graemeg/cpstrnew
=======
  'p*2Wi_','Use internal resources (Darwin)'#010+
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
>>>>>>> graemeg/cpstrnew
=======
  'p*2Wi_','Use internal resources (Darwin)'#010+
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
>>>>>>> origin/cpstrnew
=======
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)'#010,
>>>>>>> origin/fixes_2.4
  'A*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
  'A*2Wp<x>_Specify the controller type; see fpc -i or fpc -iu for possib'+
  'le values'#010+
  'm*2Wp<x>_Specify the controller type; ','see fpc -i or fpc -iu for poss'+
  'ible values'#010+
  'V*2Wp<x>_Specify the controller type; see fpc -i or fpc -iu for possib'+
  'le values'#010+
  '3*2WP<x>_Minimum iOS deployment version: 3.0, 5.0.1, ... (iphonesim)'#010+
  '4*2WP<x>_Minimum iOS deployment version: 8.0, 8.0','.2, ... (iphonesim)'+
  #010+
  'a*2WP<x>_Minimum iOS deployment version: 7.0, 7.1.2, ... (Darwin)'#010+
  'A*2WP<x>_Minimum iOS deployment version: 3.0, 5.0.1, ... (Darwin)'#010+
  '3*2WR_Generate relocation code (Windows)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '4*2WR_Generate relocation code (Windows)'#010+
  'A*2WR_','Generate relocation code (Windows)'#010+
  '8*2Wt<x>_Set the target executable format'#010+
  '8*3Wtexe_Create a DOS .EXE file (default)'#010+
  '8*3Wtcom_Create a DOS .COM file (requires tiny memory model)'#010+
  'P*2WT_Specify MPW tool type application (Classic Mac OS)'#010+
  '**2','WX_Enable executable stack (Linux)'#010+
  '**1X_Executable options:'#010+
  '**2X9_Generate linkerscript for GNU Binutils ld older than version 2.1'+
  '9.1 (Linux)'#010+
  '**2Xc_Pass --shared/-dynamic to the linker (BeOS, Darwin, FreeBSD, Lin'+
  'ux)'#010+
  '**2Xd_Do not search defa','ult library path (sometimes required for cro'+
  'ss-compiling when not using -XR)'#010+
  '**2Xe_Use external linker'#010+
  '**2Xf_Substitute pthread library name for linking (BSD)'#010+
  '**2Xg_Create debuginfo in a separate file and add a debuglink section '+
  'to executab','le'#010+
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  'A*2WR_Generate relocation c','ode (Windows)'#010+
  'P*2WT_Specify MPW tool type application (Classic Mac OS)'#010+
<<<<<<< HEAD
  '3*2WX_Enable executable stack (Linux)'#010+
=======
  '3*2WX_Enable executable stack (','Linux)'#010+
>>>>>>> origin/fixes_2.4
  'A*2WX_Enable executable stack (Linux)'#010+
  'p*2WX_Enable executable stack (Linux)'#010+
  'P*2WX_Enable executable stack (Linux)'#010+
  '**1X_Executable o','ptions:'#010+
=======
  'P*2Tami','ga_AmigaOS on PowerPC'#010+
  'P*2Tdarwin_Darwin and Mac OS X on PowerPC'#010+
  'P*2Tlinux_Linux on PowerPC'#010+
  'P*2Tmacos_Mac OS (classic) on PowerPC'#010+
  'P*2Tmorphos_MorphOS'#010+
  'S*2Tlinux_Linux'#010+
  '**1u<x>_Undefines the symbol <x>'#010+
  '**1U_Unit options:'#010+
  '**2Un_Do not check wher','e the unit name matches the file name'#010+
  '**2Ur_Generate release unit files (never automatically recompiled)'#010+
  '**2Us_Compile a system unit'#010+
  '**1v<x>_Be verbose. <x> is a combination of the following letters:'#010+
  '**2*_e : Show errors (default)       0 :',' Show nothing (except errors'+
  ')'#010+
  '**2*_w : Show warnings               u : Show unit info'#010+
  '**2*_n : Show notes                  t : Show tried/used files'#010+
  '**2*_h : Show hints                  c : Show conditionals'#010+
  '**2*_i : Show general info      ','     d : Show debug info'#010+
  '**2*_l : Show linenumbers            r : Rhide/GCC compatibility mode'#010+
=======
  'P*2Tami','ga_AmigaOS on PowerPC'#010+
  'P*2Tdarwin_Darwin and Mac OS X on PowerPC'#010+
  'P*2Tlinux_Linux on PowerPC'#010+
  'P*2Tmacos_Mac OS (classic) on PowerPC'#010+
  'P*2Tmorphos_MorphOS'#010+
  'S*2Tlinux_Linux'#010+
  '**1u<x>_Undefines the symbol <x>'#010+
  '**1U_Unit options:'#010+
  '**2Un_Do not check wher','e the unit name matches the file name'#010+
  '**2Ur_Generate release unit files (never automatically recompiled)'#010+
  '**2Us_Compile a system unit'#010+
  '**1v<x>_Be verbose. <x> is a combination of the following letters:'#010+
  '**2*_e : Show errors (default)       0 :',' Show nothing (except errors'+
  ')'#010+
  '**2*_w : Show warnings               u : Show unit info'#010+
  '**2*_n : Show notes                  t : Show tried/used files'#010+
  '**2*_h : Show hints                  c : Show conditionals'#010+
  '**2*_i : Show general info      ','     d : Show debug info'#010+
  '**2*_l : Show linenumbers            r : Rhide/GCC compatibility mode'#010+
>>>>>>> origin/fixes_2_2
  '**2*_a : Show everything             x : Executable info (Win32 only)'#010+
  '**2*_b : Write file names messages with full path'#010+
  '**2*_v : Write fpcdebug.t','xt with     p : Write tree.log with parse t'+
  'ree'#010+
  '**2*_    lots of debugging info'#010+
  '3*1W<x>_Target-specific options (targets)'#010+
  'A*1W<x>_Target-specific options (targets)'#010+
  'P*1W<x>_Target-specific options (targets)'#010+
  '3*2Wb_Create a bundle instead of a ','library (Darwin)'#010+
  'P*2Wb_Create a bundle instead of a library (Darwin)'#010+
  'p*2Wb_Create a bundle instead of a library (Darwin)'#010+
  '3*2WB_Create a relocatable image (Windows)'#010+
  'A*2WB_Create a relocatable image (Windows, Symbian)'#010+
  '3*2WC_Specify console ty','pe application (EMX, OS/2, Windows)'#010+
  'A*2WC_Specify console type application (Windows)'#010+
  'P*2WC_Specify console type application (Classic Mac OS)'#010+
  '3*2WD_Use DEFFILE to export functions of DLL or EXE (Windows)'#010+
  'A*2WD_Use DEFFILE to export functions',' of DLL or EXE (Windows)'#010+
  '3*2WF_Specify full-screen type application (EMX, OS/2)'#010+
  '3*2WG_Specify graphic type application (EMX, OS/2, Windows)'#010+
  'A*2WG_Specify graphic type application (Windows)'#010+
  'P*2WG_Specify graphic type application (Classic Mac',' OS)'#010+
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
  'A*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
  '3*2WR_Generate relocation code (Windows)'#010+
  'A*2WR_Generate relocation code (Windows)'#010+
  'P*2WT_Specify',' MPW tool type application (Classic Mac OS)'#010+
  '**1X_Executable options:'#010+
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  '**2Xc_Pass --shared/-dynamic to the linker (BeOS, Darwin, FreeBSD, Lin'+
  'ux)'#010+
<<<<<<< HEAD
  '**2Xd_Do not use standard library search path (needed for cross compil'+
  'e)'#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Xe_Use external linker'#010+
  '**2Xg_Create debuginfo in a separate file and add a debugli','nk sectio'+
  'n to executable'#010+
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
  '**2XD_Try to link units dynamically      (defines FPC_LINK_DYNAMIC)'#010+
  '**2Xi_Use internal linker'#010+
  '**2XLA_Define library substitutions for linking'#010+
  '**2XLO_Define order of library linking'#010+
  '**2XLD_Exclude default order of standard libraries'#010+
  '**2Xm','_Generate link map'#010+
  '**2XM<x>_Set the name of the '#039'main'#039' program routine (default i'+
  's '#039'main'#039')'#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '**2Xn_Use target system native linker instead of GNU ld (Solaris, AIX)'+
  #010+
  'F*2Xp<x>_First search for the compiler binary in the directory <x>'#010+
  '**2XP<x>_Pr','epend the binutils names with the prefix <x>'#010+
  '**2Xr<x>_Set the linker'#039's rlink-path to <x> (needed for cross comp'+
  'ile, see the ld manual for more information) (BeOS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (BeOS, Darwin, FreeBSD',
  ', Linux, Mac OS, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defines FPC_LINK_STATIC)'#010+
=======
  '**2Xd_Do not use sta','ndard library search path (needed for cross comp'+
  'ile)'#010+
  '**2Xe_Use external linker'#010+
  '**2Xg_Create debuginfo in a separate file and add a debuglink section '+
  'to executable'#010+
  '**2XD_Try to link units dynamically      (defines FPC_LINK_DYNAMIC)'#010+
  '**2Xi_Use',' internal linker'#010+
  '**2Xm_Generate link map'#010+
  '**2XM<x>_Set the name of the '#039'main'#039' program routine (default i'+
  's '#039'main'#039')'#010+
  '**2XP<x>_Prepend the binutils names with the prefix <x>'#010+
  '**2Xr<x>_Set the linker'#039's rlink-path to <x> (needed for cross comp'+
  'ile, ','see the ld manual for more information) (BeOS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (BeOS, Darwin, FreeBSD'+
  ', Linux, Mac OS, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defin','es FPC_LINK_STATIC'+
  ')'#010+
>>>>>>> origin/fixes_2.4
  '**2Xt_Link with static libraries (-static is passed to linker)'#010+
  '**2Xv_Generate table for Virtual Entry ca','lls'#010+
  '**2XV_Use VLink as external linker       (default on Amiga, MorphOS)'#010+
  '**2XX_Try to smartlink units             (defines FPC_LINK_SMART)'#010+
=======
  '**2XP<x>_Prepend the binu','tils names with the prefix <x>'#010+
  '**2Xr<x>_Set the linker'#039's rlink-path to <x> (needed for cross comp'+
  'ile, see the ld manual for more information) (BeOS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (BeOS, Darwin, FreeBSD'+
  ', Linux, Mac O','S, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defines FPC_LINK_STATIC)'#010+
  '**2Xt_Link with static libraries (-static is passed to linker)'#010+
  '**2XX_Try to smartlink units             (defines FPC_L','INK_SMART)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2XP<x>_Prepend the binu','tils names with the prefix <x>'#010+
  '**2Xr<x>_Set the linker'#039's rlink-path to <x> (needed for cross comp'+
  'ile, see the ld manual for more information) (BeOS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (BeOS, Darwin, FreeBSD'+
  ', Linux, Mac O','S, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defines FPC_LINK_STATIC)'#010+
  '**2Xt_Link with static libraries (-static is passed to linker)'#010+
  '**2XX_Try to smartlink units             (defines FPC_L','INK_SMART)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2XP<x>_Prepend the binu','tils names with the prefix <x>'#010+
  '**2Xr<x>_Set the linker'#039's rlink-path to <x> (needed for cross comp'+
  'ile, see the ld manual for more information) (BeOS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (BeOS, Darwin, FreeBSD'+
  ', Linux, Mac O','S, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defines FPC_LINK_STATIC)'#010+
  '**2Xt_Link with static libraries (-static is passed to linker)'#010+
  '**2XX_Try to smartlink units             (defines FPC_L','INK_SMART)'#010+
>>>>>>> graemeg/cpstrnew
=======
  '**2XP<x>_Prepend the binu','tils names with the prefix <x>'#010+
  '**2Xr<x>_Set the linker'#039's rlink-path to <x> (needed for cross comp'+
  'ile, see the ld manual for more information) (BeOS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (BeOS, Darwin, FreeBSD'+
  ', Linux, Mac O','S, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defines FPC_LINK_STATIC)'#010+
  '**2Xt_Link with static libraries (-static is passed to linker)'#010+
  '**2XX_Try to smartlink units             (defines FPC_L','INK_SMART)'#010+
>>>>>>> origin/cpstrnew
  '**1*_'#010+
  '**1?_Show this help'#010+
  '**1h_Shows this help without waiting'
=======
=======
>>>>>>> origin/fixes_2_2
  '**2Xe_Use external linke','r'#010+
  '**2Xg_Create debuginfo in a separate file and add a debuglink section '+
  'to executable'#010+
  '**2XD_Try to link units dynamically      (defines FPC_LINK_DYNAMIC)'#010+
  '**2Xi_Use internal linker'#010+
  '**2Xm_Generate link map'#010+
  '**2XM<x>_Set the name of the '#039'main'#039' ','program routine (default'+
  ' is '#039'main'#039')'#010+
  '**2XP<x>_Prepend the binutils names with the prefix <x>'#010+
  '**2Xr<x>_Set library search path to <x> (needed for cross compile) (Be'+
  'OS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (BeOS, Darwin, Free','B'+
  'SD, Linux, Mac OS, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defines FPC_LINK_STATIC)'#010+
  '**2Xt_Link with static libraries (-static is passed to linker)'#010+
  '**2XX_Try to smartlink units          ','   (defines FPC_LINK_SMART)'#010+
  '**1*_'#010+
  '**1?_Show this help'#010+
  '**1h_Shows this help without waiting'#000
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
);
