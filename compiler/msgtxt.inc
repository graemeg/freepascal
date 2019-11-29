const msgtxt_codepage=20127;
{$ifdef Delphi}
const msgtxt : array[0..000349] of string[240]=(
{$else Delphi}
const msgtxt : array[0..000349,1..240] of char=(
{$endif Delphi}
  '01000_T_Compiler: $1'#000+
  '01001_D_Compiler OS: $1'#000+
  '01002_I_Target OS: $1'#000+
  '01003_T_Using executable path: $1'#000+
  '01004_T_Using unit path: $1'#000+
  '01005_T_Using include path: $1'#000+
  '01006_T_Using library path: $1'#000+
  '01007_T_Using object path: $1'#000+
  '01008_I_$1 lines co','mpiled, $2 sec$3'#000+
  '01009_F_No memory left'#000+
  '01010_I_Writing Resource String Table file: $1'#000+
  '01011_E_Writing Resource String Table file: $1'#000+
  '01012_I_Fatal:'#000+
  '01013_I_Error:'#000+
  '01014_I_Warning:'#000+
  '01015_I_Note:'#000+
  '01016_I_Hint:'#000+
  '01017_E_Path "$1" does not exis','t'#000+
  '01018_F_Compilation aborted'#000+
  '01019_bytes code'#000+
  '01020_bytes data'#000+
  '01021_I_$1 warning(s) issued'#000+
  '01022_I_$1 hint(s) issued'#000+
  '01023_I_$1 note(s) issued'#000+
  '01024_F_I/O error: $1'#000+
  '01025_F_Operating system error: $1'#000+
  '01026_E_Compilation raised exception i','nternally'#000+
  '01027_T_Using unit scope: $1'#000+
  '02000_F_Unexpected end of file'#000+
  '02001_F_String exceeds line'#000+
  '02002_F_Illegal character "$1" ($2)'#000+
  '02003_F_Syntax error, "$1" expected but "$2" found'#000+
  '02004_TL_Start reading includefile $1'#000+
  '02005_W_Comment l','evel $1 found'#000+
  '02008_N_Ignored compiler switch "$1"'#000+
  '02009_W_Illegal compiler switch "$1"'#000+
  '02010_W_Misplaced global compiler switch, ignored'#000+
  '02011_E_Illegal char constant'#000+
  '02012_F_Cannot open file "$1"'#000+
  '02013_F_Cannot open include file "$1"'#000+
  '0201','5_E_Illegal record alignment specifier "$1"'#000+
  '02016_E_Illegal enum minimum-size specifier "$1"'#000+
  '02017_E_$ENDIF expected for $1 $2 defined in $3 line $4'#000+
  '02018_E_Syntax error while parsing a conditional compiling expression'#000+
  '02019_E_Evaluating a ','conditional compiling expression'#000+
  '02020_W_Macro contents are limited to 255 characters in length'#000+
  '02021_E_ENDIF without IF(N)DEF'#000+
  '02022_F_User defined: $1'#000+
  '02023_E_User defined: $1'#000+
  '02024_W_User defined: $1'#000+
  '02025_N_User defined: $1'#000+
  '02026_H_User ','defined: $1'#000+
  '02027_I_User defined: $1'#000+
  '02028_E_Keyword redefined as macro has no effect'#000+
  '02029_F_Macro buffer overflow while reading or expanding a macro'#000+
  '02030_W_Expanding of macros exceeds a depth of 16.'#000+
  '02031_W_Compiler switches are not supp','orted in // styled comments'#000+
  '02032_DL_Handling switch "$1"'#000+
  '02033_CL_ENDIF $1 found'#000+
  '02034_CL_IFDEF $1 found, $2'#000+
  '02035_CL_IFOPT $1 found, $2'#000+
  '02036_CL_IF $1 found, $2'#000+
  '02037_CL_IFNDEF $1 found, $2'#000+
  '02038_CL_ELSE $1 found, $2'#000+
  '02039_CL_Skipping unt','il...'#000+
  '02040_I_Press <return> to continue'#000+
  '02041_W_Unsupported switch "$1"'#000+
  '02042_W_Illegal compiler directive "$1"'#000+
  '02043_TL_Back in $1'#000+
  '02044_W_Unsupported application type: "$1"'#000+
  '02045_W_APPTYPE is not supported by the target OS'#000+
  '02046_W_DESCRI','PTION is not supported by the target OS'#000+
  '02047_N_VERSION is not supported by target OS'#000+
  '02048_N_VERSION only for exes or DLLs'#000+
  '02049_W_Wrong format for VERSION directive "$1"'#000+
  '02050_E_Illegal assembler style specified "$1"'#000+
  '02051_W_ASM reader sw','itch is not possible inside asm statement, "$1"'+
  ' will be effective only for next'#000+
  '02052_E_Wrong switch toggle, use ON/OFF or +/-'#000+
  '02053_E_Resource files are not supported for this target'#000+
  '02054_W_Include environment "$1" not found in environmen','t'#000+
  '02055_E_Illegal value for FPU register limit'#000+
  '02056_W_Only one resource file is supported for this target'#000+
  '02057_W_Macro support has been turned off'#000+
  '02058_E_Illegal interface type specified. Valids are COM, CORBA or DEF'+
  'AULT.'#000+
  '02059_W_APPID i','s only supported for PalmOS'#000+
  '02060_W_APPNAME is only supported for PalmOS'#000+
  '02061_E_Constant strings cannot be longer than 255 chars'#000+
  '02062_F_Including include files exceeds a depth of 16.'#000+
  '02063_F_Too many levels of PUSH'#000+
  '02064_E_A POP without a',' preceding PUSH'#000+
  '02065_E_Macro or compile time variable "$1" does not have any value'#000+
  '02066_E_Wrong switch toggle, use ON/OFF/DEFAULT or +/-/*'#000+
  '02067_E_Mode switch "$1" not allowed here'#000+
  '02068_E_Compile time variable or macro "$1" is not define','d.'#000+
  '02069_E_UTF-8 code greater than 65535 found'#000+
  '02070_E_Malformed UTF-8 string'#000+
  '02071_C_UTF-8 signature found, using UTF-8 encoding'#000+
  '02072_E_Compile time expression: Wanted $1 but got $2 at $3'#000+
  '02073_N_APPTYPE is not supported by the target OS'#000,
  '02074_E_Illegal optimization specified "$1"'#000+
  '02075_W_SETPEFLAGS is not supported by the target OS'#000+
  '02076_W_IMAGEBASE is not supported by the target OS'#000+
  '02077_W_MINSTACKSIZE is not supported by the target OS'#000+
  '02078_W_MAXSTACKSIZE is not supporte','d by the target OS'#000+
  '02079_E_Illegal state "$1" for $WARN directive'#000+
  '02080_E_Illegal set packing value'#000+
  '02081_W_PIC directive or switch ignored'#000+
  '02082_W_The switch "$1" is not supported by the currently selected tar'+
  'get'#000+
  '02084_W_Framework-related ','options are only supported for Darwin/Mac '+
  'OS X'#000+
  '02085_E_Illegal minimal floating point constant precision "$1"'#000+
  '02086_W_Overriding name of "main" procedure multiple times, was previo'+
  'usly set to "$1"'#000+
  '02087_W_Illegal identifier "$1" for $WARN d','irective'#000+
  '02088_E_Illegal alignment directive'#000+
  '02089_F_It is not possible to include a file that starts with an UTF-8'+
  ' BOM in a module that uses a different code page'#000+
  '02090_W_Directive "$1" is ignored for the current target platform'#000+
  '02091_W_Cu','rrent system codepage "$1" is not available for the compil'+
  'er. Switching default codepage back to "$2".'#000+
  '02092_W_SETPEOPTFLAGS is not supported by the target OS'#000+
  '02093_E_Illegal argument for SETPEFLAGS'#000+
  '02094_E_Illegal argument for SETPEOPTFLAG','S'#000+
  '02095_E_Directive $1 is not supported on this target'#000+
  '02096_W_The specified stack size is not within the valid range for the'+
  ' platform. Setting the stack size ignored.'#000+
  '02097_W_The specified HeapMax value is smaller than the HeapMin value.'+
  ' S','etting HeapMax ignored.'#000+
  '02098_E_Illegal argument for HUGEPOINTERNORMALIZATION'#000+
  '02099_E_Illegal assembler CPU instruction set specified "$1"'#000+
  '02100_W_Specified syscall convention is not useable on this target'#000+
  '02101_W_Invalid syscall convention',' specified'#000+
  '02102_W_SETPEUSERVERSION is not supported by the target OS'#000+
  '02103_W_SETPEOSVERSION is not supported by the target OS'#000+
  '02104_W_SETPESUBSYSVERSION is not supported by the target OS'#000+
  '02105_N_Changed CPU type to be consistent with speci','fied controller'#000+
  '03000_E_Parser - Syntax Error'#000+
  '03004_E_INTERRUPT procedure cannot be nested'#000+
  '03005_W_Procedure type "$1" ignored'#000+
  '03006_E_Not all declarations of "$1" are declared with OVERLOAD'#000+
  '03008_E_Duplicate exported function name "$1"'#000+
  '030','09_E_Duplicate exported function index $1'#000+
  '03010_E_Invalid index for exported function'#000+
  '03011_W_Relocatable DLL or executable $1 debug info does not work, dis'+
  'abled.'#000+
  '03012_W_To allow debugging for win32 code you need to disable relocati'+
  'on with',' -WN option'#000+
  '03013_E_Constructor name must be INIT'#000+
  '03014_E_Destructor name must be DONE'#000+
  '03016_E_Procedure type INLINE not supported'#000+
  '03018_W_Constructor should be public'#000+
  '03019_W_Destructor should be public'#000+
  '03020_N_Class should have one destru','ctor only'#000+
  '03021_E_Local class definitions are not allowed'#000+
  '03022_F_Anonymous class definitions are not allowed'#000+
  '03023_N_The object "$1" has no VMT'#000+
  '03024_E_Illegal parameter list'#000+
  '03026_E_Wrong number of parameters specified for call to "$1"'#000+
  '03','027_E_Overloaded identifier "$1" isn'#039't a function'#000+
  '03028_E_Overloaded functions have the same parameter list'#000+
  '03029_E_Function header doesn'#039't match the previous declaration "$1'+
  '"'#000+
  '03030_E_Function header "$1" doesn'#039't match forward : var name ch','a'+
  'nges $2 => $3'#000+
  '03031_N_Values in enumeration types have to be ascending'#000+
  '03033_E_With cannot be used for variables in a different segment'#000+
  '03034_E_Function nesting > 31'#000+
  '03035_E_Range check error while evaluating constants'#000+
  '03036_W_Range check ','error while evaluating constants'#000+
  '03037_E_Duplicate case label'#000+
  '03038_E_Upper bound of case range is less than lower bound'#000+
  '03039_E_Typed constants of classes or interfaces are not allowed'#000+
  '03040_E_Function variables of overloaded functions are',' not allowed'#000+
  '03041_E_String length must be a value from 1 to 255'#000+
  '03042_W_Use extended syntax of NEW and DISPOSE for instances of object'+
  's'#000+
  '03043_W_Use of NEW or DISPOSE for untyped pointers is meaningless'#000+
  '03044_E_Use of NEW or DISPOSE is not ','possible for untyped pointers'#000+
  '03045_E_Class identifier expected'#000+
  '03046_E_type identifier not allowed here'#000+
  '03047_E_Method identifier expected'#000+
  '03048_E_Function header doesn'#039't match any method of this class "$1'+
  '"'#000+
  '03049_DL_procedure/function $1'#000+
  '0','3050_E_Illegal floating point constant'#000+
  '03051_E_FAIL can be used in constructors only'#000+
  '03052_E_Destructors cannot have parameters'#000+
  '03053_E_Only class methods, class properties and class variables can b'+
  'e referred with class references'#000+
  '03054_E_O','nly class methods, class properties and class variables can'+
  ' be accessed in class methods'#000+
  '03055_E_Constant and CASE types do not match'#000+
  '03056_E_The symbol cannot be exported from a library'#000+
  '03057_W_An inherited method is hidden by "$1"'#000+
  '03058_E','_There is no method in an ancestor class to be overridden: "$'+
  '1"'#000+
  '03059_E_No member is provided to access property'#000+
  '03060_W_Stored property directive is not yet implemented'#000+
  '03061_E_Illegal symbol for property access'#000+
  '03062_E_Cannot access a pro','tected field of an object here'#000+
  '03063_E_Cannot access a private field of an object here'#000+
  '03066_E_Overridden methods must have the same return type: "$2" is ove'+
  'rridden by "$1" which has another return type'#000+
  '03067_E_EXPORT declared functions can','not be nested'#000+
  '03068_E_Methods cannot be EXPORTed'#000+
  '03069_E_Call by var for arg no. $1 has to match exactly: Got "$2" expe'+
  'cted "$3"'#000+
  '03070_E_Class isn'#039't a parent class of the current class'#000+
  '03071_E_SELF is only allowed in methods'#000+
  '03072_E_Methods',' can be only in other methods called direct with type'+
  ' identifier of the class'#000+
  '03073_E_Illegal use of '#039':'#039#000+
  '03074_E_Range check error in set constructor or duplicate set element'#000+
  '03075_E_Pointer to object expected'#000+
  '03076_E_Expression must be con','structor call'#000+
  '03077_E_Expression must be destructor call'#000+
  '03078_E_Illegal order of record elements'#000+
  '03079_E_Expression type must be class or record type, got $1'#000+
  '03080_E_Procedures cannot return a value'#000+
  '03081_E_constructors, destructors and cl','ass operators must be method'+
  's'#000+
  '03082_E_Operator is not overloaded'#000+
  '03083_E_Impossible to overload assignment for equal types'#000+
  '03084_E_Impossible operator overload'#000+
  '03085_E_Re-raise isn'#039't possible there'#000+
  '03086_E_The extended syntax of new or disp','ose isn'#039't allowed for '+
  'a class'#000+
  '03088_E_Procedure overloading is switched off'#000+
  '03089_E_It is not possible to overload this operator. Related overload'+
  'able operators (if any) are: $1'#000+
  '03090_E_Comparative operator must return a boolean value'#000+
  '03091','_E_Only virtual methods can be abstract'#000+
  '03092_F_Use of unsupported feature: "$1".'#000+
  '03093_E_The mix of different kind of objects (class, object, interface'+
  ', etc) isn'#039't allowed'#000+
  '03094_W_Unknown procedure directive had to be ignored: "$1"'#000+
  '03095_E','_$1 can be associated with only one variable'#000+
  '03096_E_absolute can only be associated with a var or const'#000+
  '03097_E_Only one variable can be initialized'#000+
  '03098_E_Abstract methods shouldn'#039't have any definition (with funct'+
  'ion body)'#000+
  '03099_E_This o','verloaded function cannot be local (must be exported)'#000+
  '03100_W_Virtual methods are used without a constructor in "$1"'#000+
  '03101_CL_Macro defined: $1'#000+
  '03102_CL_Macro undefined: $1'#000+
  '03103_CL_Macro $1 set to $2'#000+
  '03104_I_Compiling $1'#000+
  '03105_UL_Parsing i','nterface of unit $1'#000+
  '03106_UL_Parsing implementation of $1'#000+
  '03107_DL_Compiling $1 for the second time'#000+
  '03109_E_No property found to override'#000+
  '03110_E_Only one default property is allowed'#000+
  '03111_E_The default property must be an array property'#000+
  '03','112_E_Virtual constructors are only supported in class object mode'+
  'l'#000+
  '03113_E_No default property available'#000+
  '03114_E_The class cannot have a published section, use the {$M+} switc'+
  'h'#000+
  '03115_E_Forward declaration of class "$1" must be resolved her','e to u'+
  'se the class as ancestor'#000+
  '03116_E_Local operators not supported'#000+
  '03117_E_Procedure directive "$1" not allowed in interface section'#000+
  '03118_E_Procedure directive "$1" not allowed in implementation section'+
  #000+
  '03119_E_Procedure directive "$1" n','ot allowed in procvar declaration'#000+
  '03120_E_Function is already declared Public/Forward "$1"'#000+
  '03121_E_Cannot use both EXPORT and EXTERNAL'#000+
  '03123_N_"$1" not yet supported inside inline procedure/function'#000+
  '03124_H_Inlining disabled'#000+
  '03125_I_Writing',' Browser log $1'#000+
  '03126_H_Maybe pointer dereference is missing?'#000+
  '03127_F_Selected assembler reader not supported'#000+
  '03128_E_Procedure directive "$1" cannot be used with $2'#000+
  '03129_E_Calling convention doesn'#039't match forward'#000+
  '03131_E_Property cannot h','ave a default value'#000+
  '03132_E_The default value of a property must be constant'#000+
  '03133_E_Symbol cannot be published, can be only a class'#000+
  '03134_E_This kind of property cannot be published'#000+
  '03136_E_An import name is required'#000+
  '03138_E_Division by ze','ro'#000+
  '03139_E_Invalid floating point operation'#000+
  '03140_E_Upper bound of range is less than lower bound'#000+
  '03141_W_String "$1" is longer than "$2"'#000+
  '03142_E_String length is larger than array of char length'#000+
  '03143_E_Illegal expression after message dir','ective'#000+
  '03144_E_Message handlers can take only one call by ref. parameter'#000+
  '03145_E_Duplicate message label: "$1"'#000+
  '03146_E_Self can only be an explicit parameter in methods which are me'+
  'ssage handlers'#000+
  '03147_E_Threadvars can be only static or glo','bal'#000+
  '03148_F_Direct assembler not supported for binary output format'#000+
  '03149_W_Don'#039't load OBJPAS unit manually, use {$mode objfpc} or {$m'+
  'ode delphi} instead'#000+
  '03150_E_OVERRIDE cannot be used in objects'#000+
  '03151_E_Data types which require initializa','tion/finalization cannot '+
  'be used in variant records'#000+
  '03152_E_Resourcestrings can be only static or global'#000+
  '03153_E_Exit with argument cannot be used here'#000+
  '03154_E_The type of the storage symbol must be boolean'#000+
  '03155_E_This symbol isn'#039't allowed',' as storage symbol'#000+
  '03156_E_Only classes which are compiled in $M+ mode can be published'#000+
  '03157_E_Procedure directive expected'#000+
  '03158_E_The value for a property index must be of an ordinal type'#000+
  '03159_E_Procedure name too short to be exported'#000+
  '0','3160_E_No DEFFILE entry can be generated for unit global vars'#000+
  '03161_E_Compile without -WD option'#000+
  '03162_F_You need ObjFpc (-S2) or Delphi (-Sd) mode to compile this mod'+
  'ule'#000+
  '03163_E_Cannot export with index under $1'#000+
  '03164_E_Exporting of variab','les is not supported under $1'#000+
  '03165_E_Improper GUID syntax'#000+
  '03168_W_Procedure named "$1" not found that is suitable for implementi'+
  'ng the $2.$3'#000+
  '03169_E_Interface identifier expected'#000+
  '03170_E_Type "$1" cannot be used as array index type'#000+
  '03171_E','_Con- and destructors are not allowed in interfaces'#000+
  '03172_E_Access specifiers cannot be used in INTERFACEs and OBJCPROTOCO'+
  'Ls'#000+
  '03173_E_An interface, helper or Objective-C protocol or category canno'+
  't contain fields'#000+
  '03174_E_Cannot declare local',' procedure as EXTERNAL'#000+
  '03175_W_Some fields coming before "$1" were not initialized'#000+
  '03176_E_Some fields coming before "$1" were not initialized'#000+
  '03177_W_Some fields coming after "$1" were not initialized'#000+
  '03178_E_VarArgs directive (or '#039'...'#039' in',' MacPas) without CDecl'+
  '/CPPDecl/MWPascal/StdCall and External'#000+
  '03179_E_Self must be a normal (call-by-value) parameter'#000+
  '03180_E_Interface "$1" has no interface identification'#000+
  '03181_E_Unknown class field or method identifier "$1"'#000+
  '03182_W_Overri','ding calling convention "$1" with "$2"'#000+
  '03183_E_Typed constants of the type "procedure of object" can only be '+
  'initialized with NIL'#000+
  '03184_E_Default value can only be assigned to one parameter'#000+
  '03185_E_Default parameter required for "$1"'#000+
  '03186_','W_Use of unsupported feature!'#000+
  '03187_H_C arrays are passed by reference'#000+
  '03188_E_C array of const must be the last argument'#000+
  '03189_H_Type "$1" redefinition'#000+
  '03190_W_cdecl'#039'ared functions have no high parameter'#000+
  '03191_W_cdecl'#039'ared functions do not',' support open strings'#000+
  '03192_E_Cannot initialize variables declared as threadvar'#000+
  '03193_E_Message directive is only allowed in Classes'#000+
  '03194_E_Procedure or Function expected'#000+
  '03195_W_Calling convention directive ignored: "$1"'#000+
  '03196_E_REINTRODU','CE cannot be used in objects'#000+
  '03197_E_Each argument must have its own location'#000+
  '03198_E_Each argument must have an explicit location'#000+
  '03199_E_Unknown argument location'#000+
  '03200_E_32 Bit-Integer or pointer variable expected'#000+
  '03201_E_Goto statements',' are not allowed between different procedures'+
  #000+
  '03202_F_Procedure too complex, it requires too many registers'#000+
  '03203_E_Illegal expression'#000+
  '03204_E_Invalid integer expression'#000+
  '03205_E_Illegal qualifier'#000+
  '03206_E_High range limit < low range limit'#000+
  '0','3207_E_Exit'#039's parameter must be the name of the procedure it is'+
  ' used in or of a surrounding procedure'#000+
  '03208_E_Illegal assignment to for-loop variable "$1"'#000+
  '03209_E_Cannot declare local variable as EXTERNAL'#000+
  '03210_E_Procedure is already declar','ed EXTERNAL'#000+
  '03211_W_Implicit uses of Variants unit'#000+
  '03212_E_Class and static methods cannot be used in INTERFACES'#000+
  '03213_E_Overflow in arithmetic operation'#000+
  '03214_E_Protected or private expected'#000+
  '03215_E_SLICE cannot be used outside of paramete','r list'#000+
  '03216_E_A DISPINTERFACE cannot have a parent class'#000+
  '03217_E_A DISPINTERFACE needs a guid'#000+
  '03218_W_Overridden methods must have a related return type. This code '+
  'may crash, it depends on a Delphi parser bug ("$2" is overridden by "$'+
  '1" wh','ich has another return type)'#000+
  '03219_E_Dispatch IDs must be ordinal constants'#000+
  '03220_E_The range of the array is too large'#000+
  '03221_E_The address cannot be taken of bit packed array elements and r'+
  'ecord fields'#000+
  '03222_E_Dynamic arrays cannot be pack','ed'#000+
  '03223_E_Bit packed array elements and record fields cannot be used as '+
  'loop variables'#000+
  '03224_E_VAR, TYPE and CONST are allowed only in records, objects and c'+
  'lasses'#000+
  '03225_E_This type cannot be a generic'#000+
  '03226_W_Don'#039't load LINEINFO unit manu','ally, Use the -gl compiler '+
  'switch instead'#000+
  '03227_E_No function result type specified for function "$1"'#000+
  '03228_E_Specialization is only supported for generic types'#000+
  '03229_E_Generics cannot be used as parameters when specializing generi'+
  'cs'#000+
  '03230_','E_Constants of objects containing a VMT are not allowed'#000+
  '03231_E_Taking the address of labels defined outside the current scope'+
  ' isn'#039't allowed'#000+
  '03233_E_Cannot initialize variables declared as external'#000+
  '03234_E_Illegal function result type'#000+
  '03235','_E_No common type possible between "$1" and "$2"'#000+
  '03236_E_Generics without specialization cannot be used as a type for a'+
  ' variable'#000+
  '03237_W_Register list is ignored for pure assembler routines'#000+
  '03238_E_Implements property must have class or int','erface type'#000+
  '03239_E_Implements-property must implement interface of correct type, '+
  'found "$1" expected "$2"'#000+
  '03240_E_Implements-property must have read specifier'#000+
  '03241_E_Implements-property must not have write-specifier'#000+
  '03242_E_Implements-pro','perty must not have stored-specifier'#000+
  '03243_E_Implements-property used on unimplemented interface: "$1"'#000+
  '03244_E_Floating point not supported for this target'#000+
  '03245_E_Class "$1" does not implement interface "$2"'#000+
  '03246_E_Type used by implements',' must be an interface'#000+
  '03247_E_Variables cannot be exported with a different name on this tar'+
  'get, add the name to the declaration using the "export" directive (var'+
  'iable name: $1, declared export name: $2)'#000+
  '03248_E_Weak external symbols are no','t supported for the current targ'+
  'et'#000+
  '03249_E_Forward type definition does not match'#000+
  '03250_N_Virtual method "$1" has a lower visibility ($2) than parent cl'+
  'ass $3 ($4)'#000+
  '03251_E_Fields cannot appear after a method or property definition, st'+
  'art a ','new visibility section first'#000+
  '03252_E_Parameters or result types cannot contain local type definitio'+
  'ns. Use a separate type definition in a type block.'#000+
  '03253_E_ABSTRACT and SEALED conflict'#000+
  '03254_E_Cannot create a descendant of the sealed cla','ss "$1"'#000+
  '03255_E_SEALED class cannot have an ABSTRACT method'#000+
  '03256_E_Only virtual methods can be final'#000+
  '03257_E_Final method cannot be overridden: "$1"'#000+
  '03258_E_Only one message can be used per method.'#000+
  '03259_E_Invalid enumerator identifier: "$','1"'#000+
  '03260_E_Enumerator identifier required'#000+
  '03261_E_Enumerator MoveNext pattern method is not valid. Method must b'+
  'e a function with the Boolean return type and no required arguments.'#000+
  '03262_E_Enumerator Current pattern property is not valid. P','roperty m'+
  'ust have a getter.'#000+
  '03263_E_Only one enumerator MoveNext method is allowed per class/objec'+
  't'#000+
  '03264_E_Only one enumerator Current property is allowed per class/obje'+
  'ct'#000+
  '03265_E_For in loop cannot be used for the type "$1"'#000+
  '03266_E_Object','ive-C messages require their Objective-C selector name'+
  ' to be specified using the "message" directive.'#000+
  '03267_E_Objective-C does not have formal constructors nor destructors.'+
  ' Use the alloc, initXXX and dealloc messages.'#000+
  '03268_E_Message name i','s too long (max. 255 characters)'#000+
  '03269_E_Objective-C message symbol name for "$1" is too long'#000+
  '03270_H_Defining a new Objective-C root class. To derive from another '+
  'root class (e.g., NSObject), specify it as the parent class.'#000+
  '03271_E_Objecti','ve-C classes cannot have published sections.'#000+
  '03272_F_This module requires an Objective-C mode switch to be compiled'+
  #000+
  '03273_E_Inherited methods can only be overridden in Objective-C and Ja'+
  'va, add "override" (inherited method defined in $1)'#000+
  '03','274_H_Inherited methods can only be overridden in Objective-C and '+
  'Java, add "override" (inherited method defined in $1).'#000+
  '03275_E_Message name "$1" in inherited class is different from message'+
  ' name "$2" in current class.'#000+
  '03276_E_It is not ye','t possible to make unique copies of Objective-C '+
  'or Java types'#000+
  '03277_E_Objective-C categories and Object Pascal class helpers cannot '+
  'be used as types'#000+
  '03278_E_Categories do not override, but replace methods. Use "reintrod'+
  'uce" instead.'#000+
  '03279_E','_Replaced methods can only be reintroduced in Objective-C, ad'+
  'd "reintroduce" (replaced method defined in $1).'#000+
  '03280_H_Replaced methods can only be reintroduced in Objective-C, add '+
  '"reintroduce" (replaced method defined in $1).'#000+
  '03281_E_Gette','r for implements interface must use the target'#039's de'+
  'fault calling convention.'#000+
  '03282_E_Typed files cannot contain reference-counted types.'#000+
  '03283_E_Operator is not overloaded: $2 "$1"'#000+
  '03284_E_Operator is not overloaded: "$1" $2 "$3"'#000+
  '03285_E_Ex','pected another $1 array elements'#000+
  '03286_E_String constant too long while ansistrings are disabled'#000+
  '03287_E_Type cannot be used as univ parameter because its size is unkn'+
  'own at compile time: "$1"'#000+
  '03288_E_Only one class constructor can be decla','red in class: "$1"'#000+
  '03289_E_Only one class destructor can be declared in class: "$1"'#000+
  '03290_E_Class constructors cannot have parameters'#000+
  '03291_E_Class destructors cannot have parameters'#000+
  '03292_F_This construct requires the \{\$modeswitch object','ivec1\} mod'+
  'e switch to be active'#000+
  '03293_E_Unicodechar/string constants cannot be converted to ansi/short'+
  'string at compile-time'#000+
  '03294_E_For-in Objective-Pascal loops require \{\$modeswitch Objective'+
  'C2\} to be active'#000+
  '03295_E_The compiler canno','t find the NSFastEnumerationProtocol or NS'+
  'FastEnumerationState type in the CocoaAll unit'#000+
  '03296_E_Typed constants of the type '#039'procedure is nested'#039' can '+
  'only be initialized with NIL and global procedures/functions'#000+
  '03297_F_Declaration of gener','ic inside another generic is not allowed'+
  #000+
  '03298_E_Forward declaration "$1" must be resolved before a class can c'+
  'onform to or implement it'#000+
  '03299_E_Record types cannot have published sections'#000+
  '03300_E_Destructors are not allowed in records or h','elpers'#000+
  '03301_E_Class methods must be static in records'#000+
  '03302_E_Parameterless constructors are not allowed in records or recor'+
  'd/type helpers'#000+
  '03303_E_Either the result or at least one parameter must be of type "$'+
  '1"'#000+
  '03304_E_Type parameters may',' require initialization/finalization - ca'+
  'nnot be used in variant records'#000+
  '03305_E_Variables being declared as external cannot be in a custom sec'+
  'tion'#000+
  '03306_E_Non-static and non-global variables cannot have a section dire'+
  'ctive'#000+
  '03307_E_"$1" is ','not allowed in helper types'#000+
  '03308_E_Class constructors are not allowed in helpers'#000+
  '03309_E_The use of "inherited" is not allowed in a record'#000+
  '03310_E_Type declarations are not allowed in local or anonymous record'+
  's'#000+
  '03311_E_Duplicate implements',' clause for interface "$1"'#000+
  '03312_E_Interface "$1" cannot be delegated by "$2", it already has met'+
  'hod resolutions'#000+
  '03313_E_Interface "$1" cannot have method resolutions, "$2" already de'+
  'legates it'#000+
  '03314_E_Invalid codepage'#000+
  '03315_E_Only fields (','var-sections) and constants can be final in obj'+
  'ect types'#000+
  '03316_E_Final fields are currently only supported for external classes'+
  #000+
  '03317_E_Typed constants are not allowed here, only formal constants ar'+
  'e'#000+
  '03318_E_Constructors are not automatical','ly inherited in the JVM; exp'+
  'licitly add a constructor that calls the inherited one if you need it'#000+
  '03319_D_Parsing internally generated code: $1'#000+
  '03320_E_This language feature is not supported on managed VM targets'#000+
  '03321_E_Calling a virtual c','onstructor for the current instance insid'+
  'e another constructor is not possible on the JVM target'#000+
  '03322_E_Overriding method "$1" cannot have a lower visibility ($2) tha'+
  'n in parent class $3 ($4)'#000+
  '03323_E_Procedure/Function declared with call o','ption NOSTACKFRAME bu'+
  't without ASSEMBLER'#000+
  '03324_E_Procedure/Function declared with call option NOSTACKFRAME but '+
  'local stack size is $1'#000+
  '03325_E_Cannot generate property getter/setter $1 because its name cla'+
  'shes with existing identifier $2'#000+
  '033','26_W_Automatically generated property getter/setter $1 overrides '+
  'the same-named getter/setter in class $2'#000+
  '03327_W_Case mismatch between declared property getter/setter $1 and a'+
  'utomatically constructed name $2, not changing declared name'#000+
  '033','28_E_Constants declarations are not allowed in local or anonymous'+
  ' records'#000+
  '03329_E_Method declarations are not allowed in local or anonymous reco'+
  'rds'#000+
  '03330_E_Property declarations are not allowed in local or anonymous re'+
  'cords'#000+
  '03331_E_Class me','mber declarations are not allowed in local or anonym'+
  'ous records'#000+
  '03332_E_Visibility section "$1" not allowed in records'#000+
  '03333_E_Directive "$1" not allowed here'#000+
  '03334_E_Assembler blocks not allowed inside generics'#000+
  '03335_E_Properties can be on','ly static, global or inside structured t'+
  'ypes'#000+
  '03336_E_Overloaded routines have the same mangled name'#000+
  '03337_E_Default values can only be specified for value, const and cons'+
  'tref parameters'#000+
  '03338_W_Pointer type "$1" ignored'#000+
  '03339_E_Global Gener','ic template references static symtable'#000+
  '03340_UL_Unit $1 has been already compiled meanwhile.'#000+
  '03341_E_Explicit implementation of methods for specializations of gene'+
  'rics is not allowed'#000+
  '03342_E_Generic methods are not allowed in interfaces'#000+
  '033','43_E_Generic methods can not be virtual'#000+
  '03344_E_Dynamic packages not supported for target OS'#000+
  '03345_E_The HardFloat directive cannot be used if soft float code is g'+
  'enerated or fpu emulation is turned on'#000+
  '03346_E_Index $1 is not a valid intern','al function index'#000+
  '03347_W_Operator overload hidden by internal operator: "$1" $2 "$3"'#000+
  '03348_E_Thread variables inside classes or records must be class varia'+
  'bles'#000+
  '03349_E_Only static methods and static variables can be referenced thr'+
  'ough an o','bject type'#000+
  '03350_E_Cannot redeclare C-style variadic function "$1" as external on'+
  ' this platform; make its first declaration already external'#000+
  '03351_E_Unbound custom attribute: "$1".'#000+
  '03352_E_Enumeration symbols can only have values in the ran','ge of -2^'+
  '31 to 2^31-1'#000+
  '03353_W_Enumeration symbols can only have values in the range of -2^31'+
  ' to 2^31-1'#000+
  '04000_E_Type mismatch'#000+
  '04001_E_Incompatible types: got "$1" expected "$2"'#000+
  '04002_E_Type mismatch between "$1" and "$2"'#000+
  '04003_E_Type identif','ier expected'#000+
  '04004_E_Variable identifier expected'#000+
  '04005_E_Integer expression expected, but got "$1"'#000+
  '04006_E_Boolean expression expected, but got "$1"'#000+
  '04007_E_Ordinal expression expected'#000+
  '04008_E_Pointer type expected, but got "$1"'#000+
  '04009_E_Cl','ass type expected, but got "$1"'#000+
  '04011_E_Can'#039't evaluate constant expression'#000+
  '04012_E_Set elements are not compatible'#000+
  '04013_E_Operation not implemented for sets'#000+
  '04014_W_Automatic type conversion from floating type to COMP which is '+
  'an integer t','ype'#000+
  '04015_H_Use DIV instead to get an integer result'#000+
  '04016_E_String types have to match exactly in $V+ mode'#000+
  '04017_E_Succ or Pred on enums with assignments not possible'#000+
  '04018_E_Can'#039't read or write variables of this type'#000+
  '04019_E_Can'#039't use rea','dln or writeln on typed file'#000+
  '04020_E_Can'#039't use read or write on untyped file.'#000+
  '04021_E_Type conflict between set elements'#000+
  '04022_W_lo/hi(dword/qword) returns the upper/lower word/dword'#000+
  '04023_E_Integer or real expression expected'#000+
  '04024_E_Wrong',' type "$1" in array constructor'#000+
  '04025_E_Incompatible type for arg no. $1: Got "$2", expected "$3"'#000+
  '04026_E_Method (variable) and Procedure (variable) are not compatible'#000+
  '04027_E_Illegal constant passed to internal math function'#000+
  '04028_E_Can'#039't ','take the address of constant expressions'#000+
  '04029_E_Argument cannot be assigned to'#000+
  '04030_E_Can'#039't assign local procedure/function to procedure variabl'+
  'e'#000+
  '04031_E_Can'#039't assign values to an address'#000+
  '04032_E_Can'#039't assign values to const variable'#000+
  '0403','3_E_Array type required'#000+
  '04034_E_Interface type expected, but got "$1"'#000+
  '04035_H_Mixing signed expressions and longwords gives a 64bit result'#000+
  '04036_W_Mixing signed expressions and cardinals here may cause a range'+
  ' check error'#000+
  '04037_E_Typecast h','as different size ($1 -> $2) in assignment'#000+
  '04038_E_Enums with assignments cannot be used as array index'#000+
  '04039_E_Class or Object types "$1" and "$2" are not related'#000+
  '04040_W_Class types "$1" and "$2" are not related'#000+
  '04041_E_Class or interface',' type expected, but got "$1"'#000+
  '04042_E_Type "$1" is not completely defined'#000+
  '04043_W_String literal has more characters than short string length'#000+
  '04044_W_Comparison might be always false due to range of constant and '+
  'expression'#000+
  '04045_W_Comparison',' might be always true due to range of constant and'+
  ' expression'#000+
  '04046_W_Constructing a class "$1" with abstract method "$2"'#000+
  '04047_H_The left operand of the IN operator should be byte sized'#000+
  '04048_W_Type size mismatch, possible loss of data / r','ange check erro'+
  'r'#000+
  '04049_H_Type size mismatch, possible loss of data / range check error'#000+
  '04050_E_The address of an abstract method cannot be taken'#000+
  '04051_E_Assignments to formal parameters and open arrays are not possi'+
  'ble'#000+
  '04052_E_Constant Expr','ession expected'#000+
  '04053_E_Operation "$1" not supported for types "$2" and "$3"'#000+
  '04054_E_Illegal type conversion: "$1" to "$2"'#000+
  '04055_H_Conversion between ordinals and pointers is not portable'#000+
  '04056_W_Conversion between ordinals and pointers is ','not portable'#000+
  '04057_E_Can'#039't determine which overloaded function to call'#000+
  '04058_E_Illegal counter variable'#000+
  '04059_W_Converting constant real value to double for C variable argume'+
  'nt, add explicit typecast to prevent this.'#000+
  '04060_E_Class or COM in','terface type expected, but got "$1"'#000+
  '04061_E_Constant packed arrays are not yet supported'#000+
  '04062_E_Incompatible type for arg no. $1: Got "$2" expected "(Bit)Pack'+
  'ed Array"'#000+
  '04063_E_Incompatible type for arg no. $1: Got "$2" expected "(not pack'+
  'e','d) Array"'#000+
  '04064_E_Elements of packed arrays cannot be of a type which need to be'+
  ' initialised'#000+
  '04065_E_Constant packed records and objects are not yet supported'#000+
  '04066_W_Arithmetic "$1" on untyped pointer is unportable to {$T+}, sug'+
  'gest typeca','st'#000+
  '04076_E_Can'#039't take address of a subroutine marked as local'#000+
  '04077_E_Can'#039't export subroutine marked as local from a unit'#000+
  '04078_E_Type is not automatable: "$1"'#000+
  '04079_H_Converting the operands to "$1" before doing the add could pre'+
  'vent overf','low errors.'#000+
  '04080_H_Converting the operands to "$1" before doing the subtract coul'+
  'd prevent overflow errors.'#000+
  '04081_H_Converting the operands to "$1" before doing the multiply coul'+
  'd prevent overflow errors.'#000+
  '04082_W_Converting pointers to sig','ned integers may result in wrong c'+
  'omparison results and range errors, use an unsigned type instead.'#000+
  '04083_E_Interface type $1 has no valid GUID'#000+
  '04084_E_Invalid selector name "$1"'#000+
  '04085_E_Expected Objective-C method, but got $1'#000+
  '04086_E_Expec','ted Objective-C method or constant method name'#000+
  '04087_E_No type info available for this type'#000+
  '04088_E_Ordinal or string expression expected'#000+
  '04089_E_String expression expected'#000+
  '04090_W_Converting 0 to NIL'#000+
  '04091_E_Objective-C protocol type expec','ted, but got "$1"'#000+
  '04092_E_The type "$1" is not supported for interaction with the Object'+
  'ive-C and the blocks runtime.'#000+
  '04093_E_Class or objcclass type expected, but got "$1"'#000+
  '04094_E_Objcclass type expected'#000+
  '04095_W_Coerced univ parameter type',' in procedural variable may cause'+
  ' crash or memory corruption: $1 to $2'#000+
  '04096_E_Type parameters of specializations of generics cannot referenc'+
  'e the currently specialized type'#000+
  '04097_E_Type parameters are not allowed on non-generic class/recor','d/'+
  'object procedure or function'#000+
  '04098_E_Generic declaration of "$1" differs from previous declaration'#000+
  '04099_E_Helper type expected'#000+
  '04100_E_Record type expected'#000+
  '04101_E_Derived class helper must extend a subclass of "$1" or the cla'+
  'ss itself'#000+
  '0','4102_E_Derived record or type helper must extend "$1"'#000+
  '04103_E_Invalid assignment, procedures return no value'#000+
  '04104_W_Implicit string type conversion from "$1" to "$2"'#000+
  '04105_W_Implicit string type conversion with potential data loss from '+
  '"$1','" to "$2"'#000+
  '04106_-W_Explicit string typecast from "$1" to "$2"'#000+
  '04107_-W_Explicit string typecast with potential data loss from "$1" t'+
  'o "$2"'#000+
  '04108_W_Unicode constant cast with potential data loss'#000+
  '04109_E_Range check error while evaluating con','stants ($1 must be bet'+
  'ween $2 and $3)'#000+
  '04110_W_Range check error while evaluating constants ($1 must be betwe'+
  'en $2 and $3)'#000+
  '04111_E_This type is not supported for the Default() intrinsic'#000+
  '04112_E_JVM virtual class methods cannot be static'#000+
  '0411','3_E_Final (class) fields can only be assigned in their class'#039+
  ' (class) constructor'#000+
  '04114_E_It is not possible to typecast untyped parameters on managed p'+
  'latforms, simply assign a value to them instead.'#000+
  '04115_E_The assignment side of an expre','ssion cannot be typecasted to'+
  ' a supertype on managed platforms'#000+
  '04116_-W_The interface method "$1" raises the visibility of "$2" to pu'+
  'blic when accessed via an interface instance'#000+
  '04117_E_The interface method "$1" has a higher visibility (pub','lic) t'+
  'han "$2"'#000+
  '04118_E_TYPEOF can only be used on object types with VMT'#000+
  '04119_E_It is not possible to define a default value for a parameter o'+
  'f type "$1"'#000+
  '04120_E_Type "$1" cannot be extended by a type helper'#000+
  '04121_E_Procedure or function mu','st be far in order to allow taking i'+
  'ts address: "$1"'#000+
  '04122_W_Creating an instance of abstract class "$1"'#000+
  '04123_E_Subroutine references cannot be declared as "of object" or "is'+
  ' nested", they can always refer to any kind of subroutine'#000+
  '04124_E','_Procedure variables in that memory model do not store segmen'+
  't information'#000+
  '04125_W_The first value of a set constructur range is greater then the'+
  ' second value, so the range describes an empty set.'#000+
  '05000_E_Identifier not found "$1"'#000+
  '05001_F_I','nternal Error in SymTableStack()'#000+
  '05002_E_Duplicate identifier "$1"'#000+
  '05003_H_Identifier already defined in $1 at line $2'#000+
  '05004_E_Unknown identifier "$1"'#000+
  '05005_E_Forward declaration not solved "$1"'#000+
  '05007_E_Error in type definition'#000+
  '05009_E_Forw','ard type not resolved "$1"'#000+
  '05010_E_Only static variables can be used in static methods or outside'+
  ' methods'#000+
  '05012_E_Record or object or class type expected'#000+
  '05013_E_Instances of classes or objects with an abstract method are no'+
  't allowed'#000+
  '05014_','W_Label not defined "$1"'#000+
  '05015_E_Label used but not defined "$1"'#000+
  '05016_E_Illegal label declaration'#000+
  '05017_E_GOTO and LABEL are not supported (use switch -Sg)'#000+
  '05018_E_Label not found'#000+
  '05019_E_Identifier isn'#039't a label'#000+
  '05020_E_Label already defi','ned'#000+
  '05021_E_Illegal type declaration of set elements'#000+
  '05022_E_Forward class definition not resolved "$1"'#000+
  '05023_H_Unit "$1" not used in $2'#000+
  '05024_H_Parameter "$1" not used'#000+
  '05025_N_Local variable "$1" not used'#000+
  '05026_H_Value parameter "$1" is as','signed but never used'#000+
  '05027_N_Local variable "$1" is assigned but never used'#000+
  '05028_H_Local $1 "$2" is not used'#000+
  '05029_N_Private field "$1.$2" is never used'#000+
  '05030_N_Private field "$1.$2" is assigned but never used'#000+
  '05031_N_Private method "$1.$','2" never used'#000+
  '05032_E_Set type expected'#000+
  '05033_W_Function result does not seem to be set'#000+
  '05034_W_Type "$1" is not aligned correctly in current record for C'#000+
  '05035_E_Unknown record field identifier "$1"'#000+
  '05036_W_Local variable "$1" does not see','m to be initialized'#000+
  '05037_W_Variable "$1" does not seem to be initialized'#000+
  '05038_E_Identifier idents no member "$1"'#000+
  '05039_H_Found declaration: $1'#000+
  '05040_E_Data element too large'#000+
  '05042_E_No matching implementation for interface method "$1" fou','nd'#000+
  '05043_W_Symbol "$1" is deprecated'#000+
  '05044_W_Symbol "$1" is not portable'#000+
  '05055_W_Symbol "$1" is not implemented'#000+
  '05056_E_Can'#039't create unique type from this type'#000+
  '05057_H_Local variable "$1" does not seem to be initialized'#000+
  '05058_H_Variable "$1','" does not seem to be initialized'#000+
  '05059_W_Function result variable does not seem to be initialized'#000+
  '05060_H_Function result variable does not seem to be initialized'#000+
  '05061_W_Variable "$1" read but nowhere assigned'#000+
  '05062_H_Found abstract metho','d: $1'#000+
  '05063_W_Symbol "$1" is experimental'#000+
  '05064_W_Forward declaration "$1" not resolved, assumed external'#000+
  '05065_W_Symbol "$1" is belongs to a library'#000+
  '05066_W_Symbol "$1" is deprecated: "$2"'#000+
  '05067_E_Cannot find an enumerator for the type "$1','"'#000+
  '05068_E_Cannot find a "MoveNext" method in enumerator "$1"'#000+
  '05069_E_Cannot find a "Current" property in enumerator "$1"'#000+
  '05070_E_Mismatch between number of declared parameters and number of c'+
  'olons in message string.'#000+
  '05071_N_Private type "$1','.$2" never used'#000+
  '05072_N_Private const "$1.$2" never used'#000+
  '05073_N_Private property "$1.$2" never used'#000+
  '05074_W_Unit "$1" is deprecated'#000+
  '05075_W_Unit "$1" is deprecated: "$2"'#000+
  '05076_W_Unit "$1" is not portable'#000+
  '05077_W_Unit "$1" is belongs to a l','ibrary'#000+
  '05078_W_Unit "$1" is not implemented'#000+
  '05079_W_Unit "$1" is experimental'#000+
  '05080_E_No full definition of the formally declared class "$1" is in s'+
  'cope. Add the unit containing its full definition to the uses clause.'#000+
  '05081_E_Gotos into ini','tialization or finalization blocks of units ar'+
  'e not allowed'#000+
  '05082_E_Invalid external name "$1" for formal class "$2"'#000+
  '05083_E_Complete class definition with external name "$1" here'#000+
  '05084_W_Possible library conflict: symbol "$1" from library ','"$2" als'+
  'o found in library "$3"'#000+
  '05085_E_Cannot add implicit constructor '#039'Create'#039' because ident'+
  'ifier already used by "$1"'#000+
  '05086_E_Cannot generate default constructor for class, because parent '+
  'has no parameterless constructor'#000+
  '05087_D_Adding h','elper for $1'#000+
  '05088_E_Found declaration: $1'#000+
  '05089_W_Local variable "$1" of a managed type does not seem to be init'+
  'ialized'#000+
  '05090_W_Variable "$1" of a managed type does not seem to be initialize'+
  'd'#000+
  '05091_H_Local variable "$1" of a managed type d','oes not seem to be in'+
  'itialized'#000+
  '05092_H_Variable "$1" of a managed type does not seem to be initialize'+
  'd'#000+
  '05093_W_Function result variable of a managed type does not seem to be'+
  ' initialized'#000+
  '05094_H_Function result variable of a managed type doe','s not seem to '+
  'be initialized'#000+
  '05095_W_Duplicate identifier "$1"'#000+
  '05096_E_Generic type parameter "$1" does not match with the one in the'+
  ' declaration'#000+
  '05097_E_Generic type parameter declared as "$1"'#000+
  '05098_E_Record or object type expected'#000+
  '06009_E','_Parameter list size exceeds 65535 bytes'#000+
  '06012_E_File types must be var parameters'#000+
  '06013_E_The use of a far pointer isn'#039't allowed there'#000+
  '06015_E_EXPORT declared functions cannot be called'#000+
  '06016_W_Possible illegal call of constructor or destr','uctor'#000+
  '06017_N_Inefficient code'#000+
  '06018_W_Unreachable code'#000+
  '06020_E_Abstract methods cannot be called directly'#000+
  '06027_DL_Register $1 weight $2 $3'#000+
  '06029_DL_Stack frame is omitted'#000+
  '06031_E_Object or class methods cannot be inline.'#000+
  '06032_E_Procvar c','alls cannot be inline.'#000+
  '06033_E_No code for inline procedure stored'#000+
  '06035_E_Element zero of an ansi/wide- or longstring cannot be accessed'+
  ', use (set)length instead'#000+
  '06037_E_Constructors or destructors cannot be called inside a '#039'wit'+
  'h'#039' clause'#000+
  '0','6038_E_Cannot call message handler methods directly'#000+
  '06039_E_Jump in or outside of an exception block'#000+
  '06040_E_Control flow statements are not allowed in a finally block'#000+
  '06041_W_Parameters size exceeds limit for certain cpu'#039's'#000+
  '06042_W_Local va','riable size exceed limit for certain cpu'#039's'#000+
  '06043_E_Local variables size exceeds supported limit'#000+
  '06044_E_BREAK not allowed'#000+
  '06045_E_CONTINUE not allowed'#000+
  '06046_F_Unknown compilerproc "$1". Check if you use the correct run ti'+
  'me library.'#000+
  '06047_F','_Cannot find system type "$1". Check if you use the correct r'+
  'un time library.'#000+
  '06048_H_Inherited call to abstract method ignored'#000+
  '06049_E_Goto label "$1" not defined or optimized away'#000+
  '06050_F_Cannot find type "$1" in unit "$2". Check if you u','se the cor'+
  'rect run time library.'#000+
  '06051_E_Interprocedural gotos are allowed only to outer subroutines'#000+
  '06052_E_Label must be defined in the same scope as it is declared'#000+
  '06053_E_Leaving procedures containing explicit or implicit exceptions '+
  'fra','mes using goto is not allowed'#000+
  '06054_E_In ISO mode, the mod operator is defined only for positive quo'+
  'tient'#000+
  '06055_DL_Auto inlining: $1'#000+
  '06056_E_The function used, is not supported by the selected instructio'+
  'n set: $1'#000+
  '06057_F_Maximum number of u','nits ($1) reached for the current target'#000+
  '06058_N_Call to subroutine "$1" marked as inline is not inlined'#000+
  '06059_E_Case statement does not handle ordinal value "$1", and no else'+
  '/otherwise statement is present.'#000+
  '06060_W_Case statement does not ','handle all possible cases'#000+
  '07000_DL_Starting $1 styled assembler parsing'#000+
  '07001_DL_Finished $1 styled assembler parsing'#000+
  '07002_E_Non-label pattern contains @'#000+
  '07004_E_Error building record offset'#000+
  '07005_E_OFFSET used without identifier'#000+
  '07006_E_T','YPE used without identifier'#000+
  '07007_E_Cannot use local variable or parameters here'#000+
  '07008_E_Need to use OFFSET here'#000+
  '07009_E_Need to use $ here'#000+
  '07010_E_Cannot use multiple relocatable symbols'#000+
  '07011_E_Relocatable symbol can only be added'#000+
  '07012_E','_Invalid constant expression'#000+
  '07013_E_Relocatable symbol is not allowed'#000+
  '07014_E_Invalid reference syntax'#000+
  '07015_E_You cannot reach $1 from that code'#000+
  '07016_E_Local symbols/labels are not allowed as references'#000+
  '07017_E_Invalid base and index reg','ister usage'#000+
  '07018_W_Possible error in object field handling'#000+
  '07019_E_Wrong scale factor specified'#000+
  '07020_E_Multiple index register usage'#000+
  '07021_E_Invalid operand type'#000+
  '07022_E_Invalid string as opcode operand: $1'#000+
  '07023_W_@CODE and @DATA not sup','ported'#000+
  '07024_E_Null label references are not allowed'#000+
  '07025_E_Divide by zero in asm evaluator'#000+
  '07026_E_Illegal expression'#000+
  '07027_E_Escape sequence ignored: $1'#000+
  '07028_E_Invalid symbol reference'#000+
  '07029_W_Fwait can cause emulation problems with emu','387'#000+
  '07030_W_$1 without operand translated into $1P'#000+
  '07031_W_ENTER instruction is not supported by Linux kernel'#000+
  '07032_W_Calling an overload function in assembler'#000+
  '07033_E_Unsupported symbol type for operand'#000+
  '07034_E_Constant value out of bounds',#000+
  '07035_E_Error converting decimal $1'#000+
  '07036_E_Error converting octal $1'#000+
  '07037_E_Error converting binary $1'#000+
  '07038_E_Error converting hexadecimal $1'#000+
  '07039_H_$1 translated to $2'#000+
  '07040_W_$1 is associated to an overloaded function'#000+
  '07041_E_Cannot ','use SELF outside a method'#000+
  '07042_E_Cannot use OLDEBP outside a nested procedure'#000+
  '07043_W_Procedures cannot return any value in asm code'#000+
  '07044_E_SEG not supported'#000+
  '07045_E_Size suffix and destination or source size do not match'#000+
  '07046_W_Size suf','fix and destination or source size do not match'#000+
  '07047_E_Assembler syntax error'#000+
  '07048_E_Invalid combination of opcode and operands'#000+
  '07049_E_Assembler syntax error in operand'#000+
  '07050_E_Assembler syntax error in constant'#000+
  '07051_E_Invalid String ex','pression'#000+
  '07052_W_Constant with symbol $1 for address which is not on a pointer'#000+
  '07053_E_Unrecognized opcode $1'#000+
  '07054_E_Invalid or missing opcode'#000+
  '07055_E_Invalid combination of prefix and opcode: $1'#000+
  '07056_E_Invalid combination of override and',' opcode: $1'#000+
  '07057_E_Too many operands on line'#000+
  '07058_W_NEAR ignored'#000+
  '07059_W_FAR ignored'#000+
  '07060_E_Duplicate local symbol $1'#000+
  '07061_E_Undefined local symbol $1'#000+
  '07062_E_Unknown label identifier $1'#000+
  '07063_E_Invalid register name'#000+
  '07064_E_Invalid flo','ating point register name'#000+
  '07066_W_Modulo not supported'#000+
  '07067_E_Invalid floating point constant $1'#000+
  '07068_E_Invalid floating point expression'#000+
  '07069_E_Wrong symbol type'#000+
  '07070_E_Cannot index a local var or parameter with a register'#000+
  '07071_E_Inva','lid segment override expression'#000+
  '07072_W_Identifier $1 supposed external'#000+
  '07073_E_Strings not allowed as constants'#000+
  '07074_E_No type of variable specified'#000+
  '07075_E_Assembler code not returned to text section'#000+
  '07076_E_Not a directive or local symb','ol $1'#000+
  '07077_E_Using a defined name as a local label'#000+
  '07078_E_Dollar token is used without an identifier'#000+
  '07079_W_32bit constant created for address'#000+
  '07080_N_.align is target specific, use .balign or .p2align'#000+
  '07081_E_Cannot directly access fiel','ds of pointer-based parameters'#000+
  '07082_E_Can'#039't access fields of objects/classes directly'#000+
  '07083_E_No size specified and unable to determine the size of the oper'+
  'ands'#000+
  '07084_E_Cannot use RESULT in this function'#000+
  '07086_W_"$1" without operand transl','ated into "$1 %st,%st(1)"'#000+
  '07087_W_"$1 %st(n)" translated into "$1 %st,%st(n)"'#000+
  '07088_W_"$1 %st(n)" translated into "$1 %st(n),%st"'#000+
  '07089_E_Char < not allowed here'#000+
  '07090_E_Char > not allowed here'#000+
  '07093_W_ALIGN not supported'#000+
  '07094_E_Inc and De','c cannot be together'#000+
  '07095_E_Invalid register list for MOVEM or FMOVEM'#000+
  '07096_E_Reglist invalid for opcode'#000+
  '07097_E_Higher cpu mode required ($1)'#000+
  '07098_W_No size specified and unable to determine the size of the oper'+
  'ands, using DWORD as defau','lt'#000+
  '07099_E_Syntax error while trying to parse a shifter operand'#000+
  '07100_E_Address of packed component is not at a byte boundary'#000+
  '07101_W_No size specified and unable to determine the size of the oper'+
  'ands, using BYTE as default'#000+
  '07102_W_Use of $','1 for parameters invalid here'#000+
  '07103_W_Use of $1 is not compatible with regcall convention'#000+
  '07104_W_Use of $1 is not recommended for local variable access'#000+
  '07105_W_Use of $1, access may cause a crash or value may be lost'#000+
  '07106_E_VMTOffset must',' be used in combination with a virtual method,'+
  ' and "$1" is not virtual'#000+
  '07107_E_Generating PIC, but reference is not PIC-safe'#000+
  '07108_E_All registers in a register set must be of the same kind and w'+
  'idth'#000+
  '07109_E_A register set cannot be empty'#000+
  '0','7110_W_@GOTPCREL is useless and potentially dangerous for local sym'+
  'bols'#000+
  '07111_W_Constant with general purpose segment register'#000+
  '07112_E_Invalid offset value for $1'#000+
  '07113_E_Invalid register for $1'#000+
  '07114_E_SEH directives are allowed only in pu','re assembler procedures'+
  #000+
  '07115_E_Directive "$1" is not supported for the current target'#000+
  '07116_E_This function'#039's result location cannot be encoded directly'+
  ' in a single operand when "nostackframe" is used'#000+
  '07117_E_GOTPCREL references in Intel a','ssembler syntax cannot contain'+
  ' a base or index register, and their offset must 0.'#000+
  '07118_E_The current target does not support GOTPCREL relocations'#000+
  '07119_W_Exported/global symbols should be accessed via the GOT'#000+
  '07120_W_Check size of memory o','perand "$1"'#000+
  '07121_W_Check size of memory operand "$1: memory-operand-size is $2 bi'+
  'ts, but expected [$3 bits]"'#000+
  '07122_W_Check size of memory operand "$1: memory-operand-size is $2 bi'+
  'ts, but expected [$3 bits + $4 byte offset]"'#000+
  '07123_W_Check "','$1: offset of memory operand is negative "$2 byte"'#000+
  '07124_W_Check "$1: size of memory operand is empty, but es exists diff'+
  'erent definitions of the memory size =>> map to $2 (smallest option)"'#000+
  '07125_E_Invalid register used in memory reference',' expression: "$1"'#000+
  '07126_E_SEG used without identifier'#000+
  '07127_E_@CODE and @DATA can only be used with the SEG operator'#000+
  '07128_E_Not enough space (16 bits required) for the segment constant o'+
  'f symbol $1'#000+
  '07129_E_Invalid value of .code directive ','constant'#000+
  '07130_W_No size specified and unable to determine the size of the cons'+
  'tant, using BYTE as default'#000+
  '07131_W_No size specified and unable to determine the size of the cons'+
  'tant, using WORD as default'#000+
  '07132_E_Cannot override ES segment'#000,
  '07133_W_Reference is not valid here (expected "$1")'#000+
  '07134_E_Address sizes do not match'#000+
  '07135_E_Instruction "POP CS" is not valid for the current target'#000+
  '07136_W_Instruction "POP CS" is not portable (it only works on 8086 an'+
  'd 8088 CPUs)'#000+
  '07137','_E_Label $1 can only be declared public before it'#039's defined'+
  #000+
  '07138_E_Local label $1 cannot be declared public'#000+
  '07139_E_Cannot use multiple segment overrides'#000+
  '07140_W_Multiple segment overrides (only the last one will take effect'+
  ')'#000+
  '07141_W_Segme','nt base $1 will be generated, but is ignored by the CPU'+
  ' in 64-bit mode'#000+
  '07142_E_Mismatch broadcasting elements (expected: {$1} found: {$2})'#000+
  '08000_F_Too many assembler files'#000+
  '08001_F_Selected assembler output not supported'#000+
  '08002_F_Comp not sup','ported'#000+
  '08003_F_Direct not support for binary writers'#000+
  '08004_E_Allocating of data is only allowed in bss section'#000+
  '08005_F_No binary writer selected'#000+
  '08006_E_Asm: Opcode $1 not in table'#000+
  '08007_E_Asm: $1 invalid combination of opcode and operands'#000,
  '08008_E_Asm: 16 Bit references not supported'#000+
  '08009_E_Asm: Invalid effective address'#000+
  '08010_E_Asm: Immediate or reference expected'#000+
  '08011_E_Asm: $1 value exceeds bounds $2'#000+
  '08012_E_Asm: Short jump is out of range $1'#000+
  '08013_E_Asm: Undefined label',' $1'#000+
  '08014_E_Asm: Comp type not supported for this target'#000+
  '08015_E_Asm: Extended type not supported for this target'#000+
  '08016_E_Asm: Duplicate label $1'#000+
  '08017_E_Asm: Redefined label $1'#000+
  '08018_E_Asm: First defined here'#000+
  '08019_E_Asm: Invalid register ','$1'#000+
  '08020_E_Asm: 16 or 32 Bit references not supported'#000+
  '08021_E_Asm: 64 Bit operands not supported'#000+
  '08022_E_Asm: AH,BH,CH or DH cannot be used in an instruction requiring'+
  ' REX prefix'#000+
  '08023_E_Missing .seh_endprologue directive'#000+
  '08024_E_Function p','rologue exceeds 255 bytes'#000+
  '08025_E_.seh_handlerdata directive without preceding .seh_handler'#000+
  '08026_F_Relocation count for section $1 exceeds 65535'#000+
  '08027_N_Change of bind type of symbol $1 from $2 to $3 after use'#000+
  '08028_H_Change of bind type o','f symbol $1 from $2 to $3 after use'#000+
  '08029_E_Asm: 32 Bit references not supported'#000+
  '08030_F_Code segment too large'#000+
  '08031_F_Data segment too large'#000+
  '08032_E_Instruction not supported by the selected instruction set'#000+
  '08033_E_Asm: conditional branch',' destination is out of range'#000+
  '08034_E_Asm: RIP cannot be used as index register or with another regi'+
  'ster in a reference'#000+
  '09000_W_Source operating system redefined'#000+
  '09001_I_Assembling (pipe) $1'#000+
  '09002_E_Can'#039't create assembler file: $1'#000+
  '09003_E_Ca','n'#039't create object file: $1 (error code: $2)'#000+
  '09004_E_Can'#039't create archive file: $1'#000+
  '09005_E_Assembler $1 not found, switching to external assembling'#000+
  '09006_T_Using assembler: $1'#000+
  '09007_E_Error while assembling exitcode $1'#000+
  '09008_E_Can'#039't call the',' assembler, error $1 switching to external'+
  ' assembling'#000+
  '09009_I_Assembling $1'#000+
  '09010_I_Assembling with smartlinking $1'#000+
  '09011_W_Object $1 not found, Linking may fail !'#000+
  '09012_W_Library $1 not found, Linking may fail !'#000+
  '09013_E_Error while linking',#000+
  '09014_E_Can'#039't call the linker, switching to external linking'#000+
  '09015_I_Linking $1'#000+
  '09016_E_Util $1 not found, switching to external linking'#000+
  '09017_T_Using util $1'#000+
  '09018_E_Creation of Executables not supported'#000+
  '09019_E_Creation of Dynamic/Shared',' Libraries not supported'#000+
  '09035_E_Creation of Static Libraries not supported'#000+
  '09020_I_Closing script $1'#000+
  '09021_E_Resource compiler "$1" not found, switching to external mode'#000+
  '09022_I_Compiling resource $1'#000+
  '09023_T_Unit $1 cannot be statically li','nked, switching to smart link'+
  'ing'#000+
  '09024_T_Unit $1 cannot be smart linked, switching to static linking'#000+
  '09025_T_Unit $1 cannot be shared linked, switching to static linking'#000+
  '09026_E_Unit $1 cannot be smart or static linked'#000+
  '09027_E_Unit $1 canno','t be shared or static linked'#000+
  '09028_D_Calling resource compiler "$1" with "$2" as command line'#000+
  '09029_E_Error while compiling resources'#000+
  '09030_E_Can'#039't call the resource compiler "$1", switching to extern'+
  'al mode'#000+
  '09031_E_Can'#039't open resource file',' "$1"'#000+
  '09032_E_Can'#039't write resource file "$1"'#000+
  '09033_N_File "$1" not found for backquoted cat command'#000+
  '09034_W_"$1" not found, this will probably cause a linking failure'#000+
  '09128_F_Can'#039't post process executable $1'#000+
  '09129_F_Can'#039't open executable $1',#000+
  '09130_X_Size of Code: $1 bytes'#000+
  '09131_X_Size of initialized data: $1 bytes'#000+
  '09132_X_Size of uninitialized data: $1 bytes'#000+
  '09133_X_Stack space reserved: $1 bytes'#000+
  '09134_X_Stack space committed: $1 bytes'#000+
  '09200_F_Executable image size is too big ','for $1 target.'#000+
  '09201_W_Object file "$1" contains 32-bit absolute relocation to symbol'+
  ' "$2".'#000+
  '09202_E_Program segment too large (exceeds 64k by $1 bytes)'#000+
  '09203_E_Code segment "$1" too large (exceeds 64k by $2 bytes)'#000+
  '09204_E_Data segment "$1" ','too large (exceeds 64k by $2 bytes)'#000+
  '09205_E_Segment "$1" too large (exceeds 64k by $2 bytes)'#000+
  '09206_E_Group "$1" too large (exceeds 64k by $2 bytes)'#000+
  '09207_E_Cannot create a .COM file, because the program contains segmen'+
  't relocations'#000+
  '09208_W_','Program "$1" uses experimental CheckPointer option'#000+
  '09209_E_Multiple defined symbol "$1"'#000+
  '09210_E_COMDAT selection mode $1 not supported (section: "$1")'#000+
  '09211_E_Associative section expected for COMDAT section "$1"'#000+
  '09212_E_COMDAT section selec','tion mode doesn'#039't match for section '+
  '"$1" and symbol "$2"'#000+
  '09213_E_Associative COMDAT section for section "$1" not found'#000+
  '09214_D_Discarding duplicate symbol "$1" due to COMDAT selection mode'#000+
  '09215_D_Discarding duplicate symbol "$1" with same ','size due to COMDA'+
  'T selection mode'#000+
  '09216_D_Discarding duplicate symbol "$1" with same content due to COMD'+
  'AT selection mode'#000+
  '09217_D_Replacing duplicate symbol "$1" with smaller size due to COMDA'+
  'T selection mode'#000+
  '09218_E_Size of duplicate COMDA','T symbol "$1" differs'#000+
  '09219_E_Content of duplicate COMDAT symbol "$1" differs'#000+
  '09220_E_COMDAT selection mode for symbol "$1" differs'#000+
  '09221_E_Undefined symbol: $1 (first seen in $2)'#000+
  '09222_E_Undefined symbol: $1'#000+
  '10000_T_Unitsearch: $1'#000+
  '10001_T_','PPU Loading $1'#000+
  '10002_U_PPU Name: $1'#000+
  '10003_U_PPU Flags: $1'#000+
  '10004_U_PPU Crc: $1'#000+
  '10005_U_PPU Time: $1'#000+
  '10006_U_PPU File too short'#000+
  '10007_U_PPU Invalid Header (no PPU at the begin)'#000+
  '10008_U_PPU Invalid Version $1'#000+
  '10009_U_PPU is compiled for anothe','r processor'#000+
  '10010_U_PPU is compiled for another target'#000+
  '10011_U_PPU Source: $1'#000+
  '10012_U_Writing $1'#000+
  '10013_F_Can'#039't Write PPU-File'#000+
  '10014_F_Error reading PPU-File'#000+
  '10015_F_Unexpected end of PPU-File'#000+
  '10016_F_Invalid PPU-File entry: $1'#000+
  '10017_F_PPU D','bx count problem'#000+
  '10018_E_Illegal unit name: $1 (expecting $2)'#000+
  '10019_F_Too much units'#000+
  '10020_F_Circular unit reference between $1 and $2'#000+
  '10021_F_Can'#039't compile unit $1, no sources available'#000+
  '10022_F_Can'#039't find unit $1 used by $2'#000+
  '10023_W_Unit $1',' was not found but $2 exists'#000+
  '10024_F_Unit $1 searched but $2 found'#000+
  '10025_W_Compiling the system unit requires the -Us switch'#000+
  '10026_F_There were $1 errors compiling module, stopping'#000+
  '10027_U_Load from $1 ($2) unit $3'#000+
  '10028_U_Recompiling $1, c','hecksum changed for $2'#000+
  '10029_U_Recompiling $1, source found only'#000+
  '10030_U_Recompiling unit, static lib is older than ppufile'#000+
  '10031_U_Recompiling unit, shared lib is older than ppufile'#000+
  '10032_U_Recompiling unit, obj and asm are older than ppuf','ile'#000+
  '10033_U_Recompiling unit, obj is older than asm'#000+
  '10034_U_Parsing interface of $1'#000+
  '10035_U_Parsing implementation of $1'#000+
  '10036_U_Second load for unit $1'#000+
  '10037_U_PPU Check file $1 time $2'#000+
  '10040_W_Can'#039't recompile unit $1, but found modified i','nclude file'+
  's'#000+
  '10041_U_File $1 is newer than the one used for creating PPU file $2'#000+
  '10042_U_Trying to use a unit which was compiled with a different FPU m'+
  'ode'#000+
  '10043_U_Loading interface units from $1'#000+
  '10044_U_Loading implementation units from $1'#000,
  '10045_U_Interface CRC changed for unit $1'#000+
  '10046_U_Implementation CRC changed for unit $1'#000+
  '10047_U_Finished compiling unit $1'#000+
  '10048_U_Adding dependency: $1 depends on $2'#000+
  '10049_U_No reload, is caller: $1'#000+
  '10050_U_No reload, already in second co','mpile: $1'#000+
  '10051_U_Flag for reload: $1'#000+
  '10052_U_Forced reloading'#000+
  '10053_U_Previous state of $1: $2'#000+
  '10054_U_Already compiling $1, setting second compile'#000+
  '10055_U_Loading unit $1'#000+
  '10056_U_Finished loading unit $1'#000+
  '10057_U_Registering new unit $1'#000+
  '10','058_U_Re-resolving unit $1'#000+
  '10059_U_Skipping re-resolving unit $1, still loading used units'#000+
  '10060_U_Unloading resource unit $1 (not needed)'#000+
  '10061_E_Unit $1 was compiled using a different whole program optimizat'+
  'ion feedback input ($2, $3); re','compile it without wpo or use the sam'+
  'e wpo feedback input file for this compilation invocation'#000+
  '10062_U_Indirect interface (objects/classes) CRC changed for unit $1'#000+
  '10063_U_PPU is compiled for another i8086 memory model'#000+
  '10064_U_Loading unit ','$1 from package $2'#000+
  '10065_F_Internal type "$1" was not found. Check if you use the correct'+
  ' run time library.'#000+
  '10066_F_Internal type "$1" does not look as expected. Check if you use'+
  ' the correct run time library.'#000+
  '10067_U_Skipping unit, PPU and ','compiler have to be both compiled wit'+
  'h or without LLVM support'#000+
  '11000_O_$1 [options] <inputfile> [options]'#000+
  '11001_W_Only one source file supported, changing source file to compil'+
  'e from "$1" into "$2"'#000+
  '11002_W_DEF file can be created only for O','S/2'#000+
  '11003_E_Nested response files are not supported'#000+
  '11004_F_No source file name in command line'#000+
  '11005_N_No option inside $1 config file'#000+
  '11006_E_Illegal parameter: $1'#000+
  '11007_H_-? writes help pages'#000+
  '11008_F_Too many config files nested'#000+
  '11009_F_','Unable to open file $1'#000+
  '11010_D_Reading further options from $1'#000+
  '11011_W_Target is already set to: $1'#000+
  '11012_W_Shared libs not supported on DOS platform, reverting to static'+
  #000+
  '11013_F_In options file $1 at line $2 too many #IF(N)DEFs encountered',
  #000+
  '11014_F_In options file $1 at line $2 unexpected #ENDIFs encountered'#000+
  '11015_F_Open conditional at the end of the options file'#000+
  '11016_W_Debug information generation is not supported by this executab'+
  'le'#000+
  '11017_H_Try recompiling with -dGDB'#000+
  '11018_','W_You are using the obsolete switch $1'#000+
  '11019_W_You are using the obsolete switch $1, please use $2'#000+
  '11020_N_Switching assembler to default source writing assembler'#000+
  '11021_W_Assembler output selected "$1" is not compatible with "$2"'#000+
  '11022_W_"$','1" assembler use forced'#000+
  '11026_T_Reading options from file $1'#000+
  '11027_T_Reading options from environment $1'#000+
  '11028_D_Handling option "$1"'#000+
  '11029_O_*** press enter ***'#000+
  '11030_H_Start of reading config file $1'#000+
  '11031_H_End of reading config file $1'#000,
  '11032_D_Interpreting option "$1"'#000+
  '11036_D_Interpreting firstpass option "$1"'#000+
  '11033_D_Interpreting file option "$1"'#000+
  '11034_D_Reading config file "$1"'#000+
  '11035_D_Found source file name "$1"'#000+
  '11039_E_Unknown codepage "$1"'#000+
  '11040_F_Config file $1 is a',' directory'#000+
  '11041_W_Assembler output selected "$1" cannot generate debug info, deb'+
  'ugging disabled'#000+
  '11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead'#000+
  '11043_F_In options file $1 at line $2 #ELSE directive without #IF(N)DE'+
  'F fou','nd'#000+
  '11044_F_Option "$1" is not, or not yet, supported on the current targe'+
  't platform'#000+
  '11045_F_The feature "$1" is not, or not yet, supported on the selected'+
  ' target platform'#000+
  '11046_N_DWARF debug information cannot be used with smart linking on ',
  'this target, switching to static linking'#000+
  '11047_W_Option "$1" is ignored for the current target platform.'#000+
  '11048_W_Disabling external debug information because it is unsupported'+
  ' for the selected target/debug format combination.'#000+
  '11049_N_DWARF ','debug information cannot be used with smart linking wi'+
  'th external assembler, disabling static library creation.'#000+
  '11050_E_Invalid value for MACOSX_DEPLOYMENT_TARGET environment variabl'+
  'e: $1'#000+
  '11051_E_Invalid value for IPHONEOS_DEPLOYMENT_TARGET',' environment var'+
  'iable: $1'#000+
  '11052_E_You must use a FPU type of VFPV2, VFPV3 or VFPV3_D16 when usin'+
  'g the EABIHF ABI target'#000+
  '11053_W_The selected debug format is not supported on the current targ'+
  'et, not changing the current setting'#000+
  '11054_E_Argum','ent to "$1" is missing'#000+
  '11055_E_Malformed parameter: $1'#000+
  '11056_W_Smart linking requires external linker'#000+
  '11057_E_Creating .COM files is not supported in the current memory mod'+
  'el. Only the tiny memory model supports making .COM files.'#000+
  '11058_W_E','xperimental CheckPointer option not enabled because it is i'+
  'ncomptatible with -Ur option.'#000+
  '11059_E_Unsupported target architecture -P$1, invoke the "fpc" compile'+
  'r driver instead.'#000+
  '11060_E_Feature switches are only supported while compiling the',' syst'+
  'em unit.'#000+
  '11061_N_The selected debug format is not supported by the internal lin'+
  'ker, switching to external linking'#000+
  '12000_F_Cannot open whole program optimization feedback file "$1"'#000+
  '12001_D_Processing whole program optimization informati','on in wpo fee'+
  'dback file "$1"'#000+
  '12002_D_Finished processing the whole program optimization information'+
  ' in wpo feedback file "$1"'#000+
  '12003_E_Expected section header, but got "$2" at line $1 of wpo feedba'+
  'ck file'#000+
  '12004_W_No handler registered for wh','ole program optimization section'+
  ' "$2" at line $1 of wpo feedback file, ignoring'#000+
  '12005_D_Found whole program optimization section "$1" with information'+
  ' about "$2"'#000+
  '12006_F_The selected whole program optimizations require a previously '+
  'generate','d feedback file (use -Fw to specify)'#000+
  '12007_E_No collected information necessary to perform "$1" whole progr'+
  'am optimization found'#000+
  '12008_F_Specify a whole program optimization feedback file to store th'+
  'e generated info in (using -FW)'#000+
  '12009_E_N','ot generating any whole program optimization information, y'+
  'et a feedback file was specified (using -FW)'#000+
  '12010_E_Not performing any whole program optimizations, yet an input f'+
  'eedback file was specified (using -Fw)'#000+
  '12011_D_Skipping whole prog','ram optimization section "$1", because no'+
  't needed by the requested optimizations'#000+
  '12012_W_Overriding previously read information for "$1" from feedback '+
  'input file using information in section "$2"'#000+
  '12013_E_Cannot extract symbol liveness infor','mation from program when'+
  ' stripping symbols, use -Xs-'#000+
  '12014_E_Cannot extract symbol liveness information from program when w'+
  'hen not linking'#000+
  '12015_F_Cannot find "$1" or "$2" to extract symbol liveness informatio'+
  'n from linked program'#000+
  '12016_E_E','rror during reading symbol liveness information produced by'+
  ' "$1"'#000+
  '12017_F_Error executing "$1" (exitcode: $2) to extract symbol informat'+
  'ion from linked program'#000+
  '12018_E_Collection of symbol liveness information can only help when u'+
  'sing smart ','linking, use -CX -XX'#000+
  '12019_E_Cannot create specified whole program optimisation feedback fi'+
  'le "$1"'#000+
  '13001_F_Can'#039't find package $1'#000+
  '13002_U_PCP file for package $1 found'#000+
  '13003_E_Duplicate package $1'#000+
  '13004_E_Unit $1 can not be part of a package',#000+
  '13005_N_Unit $1 is implicitely imported into package $2'#000+
  '13006_F_Failed to create PCP file $2 for package $1'#000+
  '13007_F_Failed to read PCP file for package $1'#000+
  '13008_T_PCP loading $1'#000+
  '13009_U_PCP Name: $1'#000+
  '13010_U_PCP Flags: $1'#000+
  '13011_U_PCP Crc: $','1'#000+
  '13012_U_PCP Time: $1'#000+
  '13013_U_PCP File too short'#000+
  '13014_U_PCP Invalid Header (no PCP at the begin)'#000+
  '13015_U_PCP Invalid Version $1'#000+
  '13016_U_PCP is compiled for another processor'#000+
  '13017_U_PCP is compiled for another target'#000+
  '13018_U_Writing $1'#000+
  '13','019_F_Can'#039't Write PCP-File'#000+
  '13020_F_Error reading PCP-File'#000+
  '13021_F_Unexpected end of PCP-File'#000+
  '13022_F_Invalid PCP-File entry: $1'#000+
  '13023_U_Trying to use a unit which was compiled with a different FPU m'+
  'ode'#000+
  '13024_T_Packagesearch: $1'#000+
  '13025_U_Requ','ired package $1'#000+
  '13026_U_Contained unit $1'#000+
  '13027_E_Unit $1 is already contained in package $2'#000+
  '13028_W_Unit $1 is imported from indirectly required package $2'#000+
  '13029_U_PPL filename $1'#000+
  '11023_Free Pascal Compiler version $FPCFULLVERSION [$FPCDAT','E] for $F'+
  'PCCPU'#010+
  'Copyright (c) 1993-2019 by Florian Klaempfl and others'#000+
  '11024_Free Pascal Compiler version $FPCVERSION'#010+
  #010+
  'Compiler date      : $FPCDATE'#010+
  'Compiler CPU target: $FPCCPU'#010+
  #010+
  'Supported targets (targets marked with '#039'{*}'#039' are under develop',
  'ment):'#010+
  '  $OSTARGETS'#010+
  #010+
  'Supported CPU instruction sets:'#010+
  '  $INSTRUCTIONSETS'#010+
  #010+
  'Supported FPU instruction sets:'#010+
  '  $FPUINSTRUCTIONSETS'#010+
  #010+
  'Supported inline assembler modes:'#010+
  '  $ASMMODES'#010+
  #010+
  'Recognized compiler and RTL features:'#010+
  '  $FEATURELIST'#010+
  #010+
  'Supported A','BI targets:'#010+
  '  $ABITARGETS'#010+
  #010+
  'Supported Optimizations:'#010+
  '  $OPTIMIZATIONS'#010+
  #010+
  'Supported Whole Program Optimizations:'#010+
  '  All'#010+
  '  $WPOPTIMIZATIONS'#010+
  #010+
  'Supported Microcontroller types:$\n  $CONTROLLERTYPES$\n'#010+
  'This program comes under the GNU General Public ','Licence'#010+
  'For more information read COPYING.v2'#010+
  #010+
  'Please report bugs in our bug tracker on:'#010+
  '                 http://bugs.freepascal.org'#010+
  #010+
  'More information may be found on our WWW pages (including directions'#010+
  'for mailing lists useful for asking qu','estions or discussing potentia'+
  'l'#010+
  'new features, etc.):'#010+
  '                 http://www.freepascal.org'#000+
  '11025_F*0*_Only options valid for the default or selected platform are'+
  ' listed.'#010+
  '**0*_Put + after a boolean switch option to enable it, - to disab','le '+
  'it.'#010+
  '**1@<x>_Read compiler options from <x> in addition to the default fpc.'+
  'cfg'#010+
  '**1a_The compiler does not delete the generated assembler file'#010+
  '**2a5_Don'#039't generate Big Obj COFF files for GNU Binutils older tha'+
  'n 2.25 (Windows, NativeNT)'#010+
  '**2','al_List sourcecode lines in assembler file'#010+
  '**2an_List node info in assembler file (-dEXTDEBUG compiler)'#010+
  '**2ao_Add an extra option to external assembler call (ignored for inte'+
  'rnal)'#010+
  '*L2ap_Use pipes instead of creating temporary assembler file','s'#010+
  '**2ar_List register allocation/release info in assembler file'#010+
  '**2at_List temp allocation/release info in assembler file'#010+
  '**1A<x>_Output format:'#010+
  '**2Adefault_Use default assembler'#010+
  '3*2Aas_Assemble using GNU AS'#010+
  '3*2Amacho_Mach-O (Darwin, Intel ','32 bit) using internal writer'#010+
  '8*2Anasm_Assemble using Nasm'#010+
  '8*2Anasmobj_Assemble using Nasm'#010+
  '3*2Anasm_Assemble using Nasm'#010+
  '3*2Anasmcoff_COFF (Go32v2) file using Nasm'#010+
  '3*2Anasmelf_ELF32 (Linux) file using Nasm'#010+
  '3*2Anasmwin32_Win32 object file usi','ng Nasm'#010+
  '3*2Anasmwdosx_Win32/WDOSX object file using Nasm'#010+
  '3*2Anasmdarwin_macho32 object file using Nasm (experimental)'#010+
  '3*2Awasm_Obj file using Wasm (Watcom)'#010+
  '3*2Anasmobj_Obj file using Nasm'#010+
  '3*2Amasm_Obj file using Masm (Microsoft)'#010+
  '3*2Atasm_Ob','j file using Tasm (Borland)'#010+
  '3*2Aelf_ELF (Linux) using internal writer'#010+
  '3*2Acoff_COFF (Go32v2) using internal writer'#010+
  '3*2Apecoff_PE-COFF (Win32) using internal writer'#010+
  '3*2Ayasm_Assemble using Yasm (experimental)'#010+
  '4*2Aas_Assemble using GNU AS'#010+
  '4*2','Agas_Assemble using GNU GAS'#010+
  '4*2Agas-darwin_Assemble darwin Mach-O64 using GNU GAS'#010+
  '4*2Amasm_Win64 object file using ml64 (Microsoft)'#010+
  '4*2Apecoff_PE-COFF (Win64) using internal writer'#010+
  '4*2Aelf_ELF (Linux-64bit) using internal writer'#010+
  '4*2Ayasm_As','semble using Yasm (experimental)'#010+
  '4*2Anasm_Assemble using Nasm (experimental)'#010+
  '4*2Anasmwin64_Assemble Win64 object file using Nasm (experimental)'#010+
  '4*2Anasmelf_Assemble Linux-64bit object file using Nasm (experimental)'+
  #010+
  '4*2Anasmdarwin_Assemble d','arwin macho64 object file using Nasm (exper'+
  'imental)'#010+
  '6*2Aas_Unix o-file using GNU AS'#010+
  '6*2Agas_GNU Motorola assembler'#010+
  '6*2Amit_MIT Syntax (old GAS)'#010+
  '6*2Amot_Standard Motorola assembler'#010+
  '6*2Avasm_Use vasm to assemble'#010+
  'A*2Aas_Assemble using GNU AS'#010+
  'P','*2Aas_Assemble using GNU AS'#010+
  'S*2Aas_Assemble using GNU AS'#010+
  '**1b_Generate browser info'#010+
  '**2bl_Generate local symbol info'#010+
  '**1B_Build all modules'#010+
  '**1C<x>_Code generation options:'#010+
  '**2C3_Turn on ieee error checking for constants'#010+
  '**2Ca<x>_Select ABI','; see fpc -i or fpc -ia for possible values'#010+
  '**2Cb_Generate code for a big-endian variant of the target architectur'+
  'e'#010+
  '**2Cc<x>_Set default calling convention to <x>'#010+
  '**2CD_Create also dynamic library (not supported)'#010+
  '**2Ce_Compilation with emul','ated floating point opcodes'#010+
  '**2CE_Generate FPU code which can raise exceptions'#010+
  '**2Cf<x>_Select fpu instruction set to use; see fpc -i or fpc -if for '+
  'possible values'#010+
  '**2CF<x>_Minimal floating point constant precision (default, 32, 64)'#010+
  '**2Cg_','Generate PIC code'#010+
  '**2Ch<n>[,m]_<n> bytes min heap size (between 1023 and 67107840) and o'+
  'ptionally [m] max heap size'#010+
  '**2Ci_IO-checking'#010+
  'A*2CI<x>_Select instruction set on ARM: ARM or THUMB'#010+
  'L*2CL<x>_LLVM code generation options'#010+
  'L*3CLflto_Enabl','e Link-time optimisation (needed both when compiling '+
  'units and programs/libraries)'#010+
  'L*3CLfltonosystem_Disable LTO for the system unit (needed with at leas'+
  't Xcode 10.2 and earlier due to linker bugs)'#010+
  'L*3CLv<x>_LLVM target version: 3.3, 3.4, .','., Xcode-6.4, .., Xcode-10'+
  '.1, 7.0, 8.0'#010+
  '**2Cn_Omit linking stage'#010+
  'P*2CN_Generate nil-pointer checks (AIX-only)'#010+
  '**2Co_Check overflow of integer operations'#010+
  '**2CO_Check for possible overflow of integer operations'#010+
  '**2Cp<x>_Select instruction set;',' see fpc -i or fpc -ic for possible '+
  'values'#010+
  '**2CP<x>=<y>_ packing settings'#010+
  '**3CPPACKSET=<y>_ <y> set allocation: 0, 1 or DEFAULT or NORMAL, 2, 4 '+
  'and 8'#010+
  '**3CPPACKENUM=<y>_ <y> enum packing: 0, 1, 2 and 4 or DEFAULT or NORMA'+
  'L'#010+
  '**3CPPACKRECORD=<y','>_ <y> record packing: 0 or DEFAULT or NORMAL, 1, '+
  '2, 4, 8, 16 and 32'#010+
  '**2Cr_Range checking'#010+
  '**2CR_Verify object method call validity'#010+
  '**2Cs<n>_Set stack checking size to <n>'#010+
  '**2Ct_Stack checking (for testing only, see manual)'#010+
  '8*2CT<x>_Target-s','pecific code generation options'#010+
  '3*2CT<x>_Target-specific code generation options'#010+
  '4*2CT<x>_Target-specific code generation options'#010+
  'p*2CT<x>_Target-specific code generation options'#010+
  'P*2CT<x>_Target-specific code generation options'#010+
  'J*2CT<x>_Tar','get-specific code generation options'#010+
  'A*2CT<x>_Target-specific code generation options'#010+
  'p*3CTsmalltoc_ Generate smaller TOCs at the expense of execution speed'+
  ' (AIX)'#010+
  'P*3CTsmalltoc_ Generate smaller TOCs at the expense of execution speed'+
  ' (AIX)'#010,
  'J*3CTautogetterprefix=X_  Automatically create getters for properties '+
  'with prefix X (empty string disables)'#010+
  'J*3CTautosetterprefix=X_  Automatically create setters for properties '+
  'with prefix X (empty string disables)'#010+
  '8*3CTcld_               ','  Emit a CLD instruction before using the x8'+
  '6 string instructions'#010+
  '3*3CTcld_                 Emit a CLD instruction before using the x86 '+
  'string instructions'#010+
  '4*3CTcld_                 Emit a CLD instruction before using the x86 '+
  'string instruc','tions'#010+
  '8*3CTfarprocspushoddbp_       Increment BP before pushing it in the pr'+
  'ologue of far functions'#010+
  'J*3CTcompactintarrayinit_ Generate smaller (but potentially slower) co'+
  'de for initializing integer array constants'#010+
  'J*3CTenumfieldinit_       ','Initialize enumeration fields in construct'+
  'ors to enumtype(0), after calling inherited constructors'#010+
  'J*3CTinitlocals_          Initialize local variables that trigger a JV'+
  'M bytecode verification error if used uninitialized (slows down code)'#010+
  'J','*3CTlowercaseprocstart_  Lowercase the first character of procedure'+
  '/function/method names'#010+
  'A*3CTthumbinterworking_ Generate Thumb interworking-safe code if possi'+
  'ble'#010+
  'J*2Cv_Var/out parameter copy-out checking'#010+
  'A*2CV<x>_Set section threadvar mod','el to <x>'#010+
  '**2CX_Create also smartlinked library'#010+
  '**1d<x>_Defines the symbol <x>'#010+
  '**1D_Generate a DEF file'#010+
  '**2Dd<x>_Set description to <x>'#010+
  '**2Dv<x>_Set DLL version to <x>'#010+
  '*O2Dw_PM application'#010+
  '**1e<x>_Set path to executable'#010+
  '**1E_Same as -Cn'#010+
  '**1','fPIC_Same as -Cg'#010+
  '**1F<x>_Set file names and paths:'#010+
  '**2Fa<x>[,y]_(for a program) load units <x> and [y] before uses is par'+
  'sed'#010+
  '**2Fc<x>_Set input codepage to <x>'#010+
  '**2FC<x>_Set RC compiler binary name to <x>'#010+
  '**2Fd_Disable the compiler'#039's interna','l directory cache'#010+
  '**2FD<x>_Set the directory where to search for compiler utilities'#010+
  '**2Fe<x>_Redirect error output to <x>'#010+
  '**2Ff<x>_Add <x> to framework path (Darwin only)'#010+
  '**2FE<x>_Set exe/unit output path to <x>'#010+
  '**2Fi<x>_Add <x> to include ','path'#010+
  '**2Fl<x>_Add <x> to library path'#010+
  '**2FL<x>_Use <x> as dynamic linker'#010+
  '**2Fm<x>_Load unicode conversion table from <x>.txt in the compiler di'+
  'r'#010+
  '**2FM<x>_Set the directory where to search for unicode binary files'#010+
  '**2FN<x>_Add <x> to list of',' default unit scopes (namespaces)'#010+
  '**2Fo<x>_Add <x> to object path'#010+
  '**2Fr<x>_Load error message file <x>'#010+
  '**2FR<x>_Set resource (.res) linker to <x>'#010+
  '**2Fu<x>_Add <x> to unit path'#010+
  '**2FU<x>_Set unit output path to <x>, overrides -FE'#010+
  '**2FW<x>_Sto','re generated whole-program optimization feedback in <x>'#010+
  '**2Fw<x>_Load previously stored whole-program optimization feedback fr'+
  'om <x>'#010+
  '*g1g_Generate debug information (default format for target)'#010+
  '*g2gc_Generate checks for pointers (experimenta','l, only available on '+
  'some targets, might generate false positive)'#010+
  '*g2gh_Use heaptrace unit (for memory leak/corruption debugging)'#010+
  '*g2gl_Use line info unit (show more info with backtraces)'#010+
  '*g2gm_Generate Microsoft CodeView debug information ','(experimental)'#010+
  '*g2go<x>_Set debug information options'#010+
  '*g3godwarfsets_ Enable DWARF '#039'set'#039' type debug information (bre'+
  'aks gdb < 6.5)'#010+
  '*g3gostabsabsincludes_ Store absolute/full include file paths in Stabs'+
  #010+
  '*g3godwarfmethodclassprefix_ Prefix me','thod names in DWARF with class'+
  ' name'#010+
  '*g3godwarfcpp_ Simulate C++ debug information in DWARF'#010+
  '*g3godwarfomflinnum_ Generate line number information in OMF LINNUM re'+
  'cords in MS LINK format in addition to the DWARF debug information (Op'+
  'en Watcom',' Debugger/Linker compatibility)'#010+
  '*g2gp_Preserve case in stabs symbol names'#010+
  '*g2gs_Generate Stabs debug information'#010+
  '*g2gt_Trash local variables (to detect uninitialized uses; multiple '#039+
  't'#039' changes the trashing value)'#010+
  '*g2gv_Generates programs tr','aceable with Valgrind'#010+
  '*g2gw_Generate DWARFv2 debug information (same as -gw2)'#010+
  '*g2gw2_Generate DWARFv2 debug information'#010+
  '*g2gw3_Generate DWARFv3 debug information'#010+
  '*g2gw4_Generate DWARFv4 debug information (experimental)'#010+
  '**1i_Information'#010+
  '**2i','D_Return compiler date'#010+
  '**2iSO_Return compiler OS'#010+
  '**2iSP_Return compiler host processor'#010+
  '**2iTO_Return target OS'#010+
  '**2iTP_Return target processor'#010+
  '**2iV_Return short compiler version'#010+
  '**2iW_Return full compiler version'#010+
  '**2ia_Return list of suppor','ted ABI targets'#010+
  '**2ic_Return list of supported CPU instruction sets'#010+
  '**2if_Return list of supported FPU instruction sets'#010+
  '**2ii_Return list of supported inline assembler modes'#010+
  '**2io_Return list of supported optimizations'#010+
  '**2ir_Return list of ','recognized compiler and RTL features'#010+
  '**2it_Return list of supported targets'#010+
  '**2iu_Return list of supported microcontroller types'#010+
  '**2iw_Return list of supported whole program optimizations'#010+
  '**1I<x>_Add <x> to include path'#010+
  '**1k<x>_Pass <x> to ','the linker'#010+
  '**1l_Write logo'#010+
  '**1M<x>_Set language mode to <x>'#010+
  '**2Mfpc_Free Pascal dialect (default)'#010+
  '**2Mobjfpc_FPC mode with Object Pascal support'#010+
  '**2Mdelphi_Delphi 7 compatibility mode'#010+
  '**2Mtp_TP/BP 7.0 compatibility mode'#010+
  '**2Mmacpas_Macintosh',' Pascal dialects compatibility mode'#010+
  '**2Miso_ISO 7185 mode'#010+
  '**2Mextendedpascal_ISO 10206 mode'#010+
  '**2Mdelphiunicode_Delphi 2009 and later compatibility mode'#010+
  '**1n_Do not read the default config files'#010+
  '**1o<x>_Change the name of the executable produ','ced to <x>'#010+
  '**1O<x>_Optimizations:'#010+
  '**2O-_Disable optimizations'#010+
  '**2O1_Level 1 optimizations (quick and debugger friendly)'#010+
  '**2O2_Level 2 optimizations (-O1 + quick optimizations)'#010+
  '**2O3_Level 3 optimizations (-O2 + slow optimizations)'#010+
  '**2O4_Lev','el 4 optimizations (-O3 + optimizations which might have un'+
  'expected side effects)'#010+
  '**2Oa<x>=<y>_Set alignment'#010+
  '**2Oo[NO]<x>_Enable or disable optimizations; see fpc -i or fpc -io fo'+
  'r possible values'#010+
  '**2Op<x>_Set target cpu for optimizing; see',' fpc -i or fpc -ic for po'+
  'ssible values'#010+
  '**2OW<x>_Generate whole-program optimization feedback for optimization'+
  ' <x>; see fpc -i or fpc -iw for possible values'#010+
  '**2Ow<x>_Perform whole-program optimization <x>; see fpc -i or fpc -iw'+
  ' for possible',' values'#010+
  '**2Os_Optimize for size rather than speed'#010+
  '**1pg_Generate profile code for gprof (defines FPC_PROFILE)'#010+
  'F*1P<x>_Target CPU / compiler related options:'#010+
  'F*2PB_Show default compiler binary'#010+
  'F*2PP_Show default target cpu'#010+
  'F*2P<x>_Set target',' CPU (aarch64,arm,avr,i386,i8086,jvm,m68k,mips,mip'+
  'sel,powerpc,powerpc64,sparc,x86_64)'#010+
  '**1R<x>_Assembler reading style:'#010+
  '**2Rdefault_Use default assembler for target'#010+
  '3*2Ratt_Read AT&T style assembler'#010+
  '3*2Rintel_Read Intel style assembler'#010+
  '4*2Ra','tt_Read AT&T style assembler'#010+
  '4*2Rintel_Read Intel style assembler'#010+
  '8*2Ratt_Read AT&T style assembler'#010+
  '8*2Rintel_Read Intel style assembler'#010+
  '6*2RMOT_Read Motorola style assembler'#010+
  '**1S<x>_Syntax options:'#010+
  '**2S2_Same as -Mobjfpc'#010+
  '**2Sc_Support oper','ators like C (*=,+=,/= and -=)'#010+
  '**2Sa_Turn on assertions'#010+
  '**2Sd_Same as -Mdelphi'#010+
  '**2Se<x>_Error options. <x> is a combination of the following:'#010+
  '**3*_<n> : Compiler halts after the <n> errors (default is 1)'#010+
  '**3*_w : Compiler also halts after w','arnings'#010+
  '**3*_n : Compiler also halts after notes'#010+
  '**3*_h : Compiler also halts after hints'#010+
  '**2Sf_Enable certain features in compiler and RTL; see fpc -i or fpc -'+
  'ir for possible values)'#010+
  '**2Sg_Enable LABEL and GOTO (default in -Mtp and -Mdelph','i)'#010+
  '**2Sh_Use reference counted strings (ansistring by default) instead of'+
  ' shortstrings'#010+
  '**2Si_Turn on inlining of procedures/functions declared as "inline"'#010+
  '**2Sj_Allows typed constants to be writeable (default in all modes)'#010+
  '**2Sk_Load fpcyli','x unit'#010+
  '**2SI<x>_Set interface style to <x>'#010+
  '**3SIcom_COM compatible interface (default)'#010+
  '**3SIcorba_CORBA compatible interface'#010+
  '**2Sm_Support macros like C (global)'#010+
  '**2So_Same as -Mtp'#010+
  '**2Sr_Transparent file names in ISO mode'#010+
  '**2Ss_Constructor ','name must be init (destructor must be done)'#010+
  '**2Sv_Support vector processing (use CPU vector extensions if availabl'+
  'e)'#010+
  '**2Sx_Enable exception keywords (default in Delphi/ObjFPC modes)'#010+
  '**2Sy_@<pointer> returns a typed pointer, same as $T+'#010+
  '**1s','_Do not call assembler and linker'#010+
  '**2sh_Generate script to link on host'#010+
  '**2st_Generate script to link on target'#010+
  '**2sr_Skip register allocation phase (use with -alr)'#010+
  '**1T<x>_Target operating system:'#010+
  '3*2Tandroid_Android'#010+
  '3*2Taros_AROS'#010+
  '3*2Tbeos','_BeOS'#010+
  '3*2Tdarwin_Darwin/Mac OS X'#010+
  '3*2Tembedded_Embedded'#010+
  '3*2Temx_OS/2 via EMX (including EMX/RSX extender)'#010+
  '3*2Tfreebsd_FreeBSD'#010+
  '3*2Tgo32v2_Version 2 of DJ Delorie DOS extender'#010+
  '3*2Thaiku_Haiku'#010+
  '3*2Tiphonesim_iPhoneSimulator from iOS SDK 3.2+ (ol','der versions: -Td'+
  'arwin)'#010+
  '3*2Tlinux_Linux'#010+
  '3*2Tnativent_Native NT API (experimental)'#010+
  '3*2Tnetbsd_NetBSD'#010+
  '3*2Tnetware_Novell Netware Module (clib)'#010+
  '3*2Tnetwlibc_Novell Netware Module (libc)'#010+
  '3*2Topenbsd_OpenBSD'#010+
  '3*2Tos2_OS/2 / eComStation'#010+
  '3*2Tsymbia','n_Symbian OS'#010+
  '3*2Tsolaris_Solaris'#010+
  '3*2Twatcom_Watcom compatible DOS extender'#010+
  '3*2Twdosx_WDOSX DOS extender'#010+
  '3*2Twin32_Windows 32 Bit'#010+
  '3*2Twince_Windows CE'#010+
  '4*2Taros_AROS'#010+
  '4*2Tdarwin_Darwin/Mac OS X'#010+
  '4*2Tdragonfly_DragonFly BSD'#010+
  '4*2Tembedded_Embedded',#010+
  '4*2Tfreebsd_FreeBSD'#010+
  '4*2Tiphonesim_iPhoneSimulator'#010+
  '4*2Tlinux_Linux'#010+
  '4*2Tnetbsd_NetBSD'#010+
  '4*2Topenbsd_OpenBSD'#010+
  '4*2Tsolaris_Solaris'#010+
  '4*2Twin64_Win64 (64 bit Windows systems)'#010+
  '6*2Tamiga_Commodore Amiga'#010+
  '6*2Tatari_Atari ST/STe/TT'#010+
  '6*2Tembedded_Embedded'#010,
  '6*2Tlinux_Linux'#010+
  '6*2Tnetbsd_NetBSD'#010+
  '6*2Tmacos_Mac OS'#010+
  '6*2Tpalmos_PalmOS'#010+
  '8*2Tembedded_Embedded'#010+
  '8*2Tmsdos_MS-DOS (and compatible)'#010+
  '8*2Twin16_Windows 16 Bit'#010+
  'A*2Tandroid_Android'#010+
  'A*2Taros_AROS'#010+
  'A*2Tdarwin_Darwin/iPhoneOS/iOS'#010+
  'A*2Tembedded_Embedded'#010+
  'A*2','Tgba_Game Boy Advance'#010+
  'A*2Tlinux_Linux'#010+
  'A*2Tnds_Nintendo DS'#010+
  'A*2Tnetbsd_NetBSD'#010+
  'A*2Tpalmos_PalmOS'#010+
  'A*2Tsymbian_Symbian'#010+
  'A*2Twince_Windows CE'#010+
  'a*2Tdarwin_Darwin/iOS'#010+
  'a*2Tlinux_Linux'#010+
  'J*2Tandroid_Android'#010+
  'J*2Tjava_Java'#010+
  'm*2Tandroid_Android'#010+
  'm*2Tembedded_','Embedded'#010+
  'm*2Tlinux_Linux'#010+
  'M*2Tembedded_Embedded'#010+
  'M*2Tlinux_Linux'#010+
  'P*2Taix_AIX'#010+
  'P*2Tamiga_AmigaOS'#010+
  'P*2Tdarwin_Darwin/Mac OS X'#010+
  'P*2Tembedded_Embedded'#010+
  'P*2Tlinux_Linux'#010+
  'P*2Tmacos_Mac OS (classic)'#010+
  'P*2Tmorphos_MorphOS'#010+
  'P*2Tnetbsd_NetBSD'#010+
  'P*2Twii_Wii'#010+
  'p*2Ta','ix_AIX'#010+
  'p*2Tdarwin_Darwin/Mac OS X'#010+
  'p*2Tembedded_Embedded'#010+
  'p*2Tlinux_Linux'#010+
  'R*2Tlinux_Linux'#010+
  'R*2Tembedded_Embedded'#010+
  'r*2Tlinux_Linux'#010+
  'r*2Tembedded_Embedded'#010+
  'S*2Tlinux_Linux'#010+
  'S*2Tsolaris_Solaris'#010+
  's*2Tlinux_Linux'#010+
  'V*2Tembedded_Embedded'#010+
  '**1u<x>_Undefines ','the symbol <x>'#010+
  '**1U_Unit options:'#010+
  '**2Un_Do not check where the unit name matches the file name'#010+
  '**2Ur_Generate release unit files (never automatically recompiled)'#010+
  '**2Us_Compile a system unit'#010+
  '**1v<x>_Be verbose. <x> is a combination of the fo','llowing letters:'#010+
  '**2*_e : Show errors (default)       0 : Show nothing (except errors)'#010+
  '**2*_w : Show warnings               u : Show unit info'#010+
  '**2*_n : Show notes                  t : Show tried/used files'#010+
  '**2*_h : Show hints               ','   c : Show conditionals'#010+
  '**2*_i : Show general info           d : Show debug info'#010+
  '**2*_l : Show linenumbers            r : Rhide/GCC compatibility mode'#010+
  '**2*_s : Show time stamps            q : Show message numbers'#010+
  '**2*_a : Show everything  ','           x : Show info about invoked too'+
  'ls'#010+
  '**2*_b : Write file names messages   p : Write tree.log with parse tre'+
  'e'#010+
  '**2*_    with full path              v : Write fpcdebug.txt with'#010+
  '**2*_z : Write output to stderr          lots of debugging',' info'#010+
  '**2*_m<x>,<y> : Do not show messages numbered <x> and <y>'#010+
  'F*1V<x>_Append '#039'-<x>'#039' to the used compiler binary name (e.g. f'+
  'or version)'#010+
  '**1W<x>_Target-specific options (targets)'#010+
  '3*2WA_Specify native type application (Windows)'#010+
  '4*2WA_Specif','y native type application (Windows)'#010+
  'A*2WA_Specify native type application (Windows)'#010+
  '3*2Wb_Create a bundle instead of a library (Darwin)'#010+
  'P*2Wb_Create a bundle instead of a library (Darwin)'#010+
  'p*2Wb_Create a bundle instead of a library (Darwin)'#010,
  'a*2Wb_Create a bundle instead of a library (Darwin)'#010+
  'A*2Wb_Create a bundle instead of a library (Darwin)'#010+
  '4*2Wb_Create a bundle instead of a library (Darwin)'#010+
  '3*2WB_Create a relocatable image (Windows, Symbian)'#010+
  '3*2WB<x>_Set image base to <x> (','Windows, Symbian)'#010+
  '4*2WB_Create a relocatable image (Windows)'#010+
  '4*2WB<x>_Set image base to <x> (Windows)'#010+
  'A*2WB_Create a relocatable image (Windows, Symbian)'#010+
  'A*2WB<x>_Set image base to <x> (Windows, Symbian)'#010+
  '3*2WC_Specify console type applicati','on (EMX, OS/2, Windows)'#010+
  '4*2WC_Specify console type application (Windows)'#010+
  'A*2WC_Specify console type application (Windows)'#010+
  'P*2WC_Specify console type application (Classic Mac OS)'#010+
  '3*2WD_Use DEFFILE to export functions of DLL or EXE (Windows)'#010,
  '4*2WD_Use DEFFILE to export functions of DLL or EXE (Windows)'#010+
  'A*2WD_Use DEFFILE to export functions of DLL or EXE (Windows)'#010+
  '3*2We_Use external resources (Darwin)'#010+
  '4*2We_Use external resources (Darwin)'#010+
  'a*2We_Use external resources (Darwin)'#010+
  'A*','2We_Use external resources (Darwin)'#010+
  'P*2We_Use external resources (Darwin)'#010+
  'p*2We_Use external resources (Darwin)'#010+
  '3*2WF_Specify full-screen type application (EMX, OS/2)'#010+
  '3*2WG_Specify graphic type application (EMX, OS/2, Windows)'#010+
  '4*2WG_Specify',' graphic type application (Windows)'#010+
  'A*2WG_Specify graphic type application (Windows)'#010+
  'P*2WG_Specify graphic type application (Classic Mac OS)'#010+
  '3*2Wi_Use internal resources (Darwin)'#010+
  '4*2Wi_Use internal resources (Darwin)'#010+
  'a*2Wi_Use internal reso','urces (Darwin)'#010+
  'A*2Wi_Use internal resources (Darwin)'#010+
  'P*2Wi_Use internal resources (Darwin)'#010+
  'p*2Wi_Use internal resources (Darwin)'#010+
  '3*2WI_Turn on/off the usage of import sections (Windows)'#010+
  '4*2WI_Turn on/off the usage of import sections (Window','s)'#010+
  'A*2WI_Turn on/off the usage of import sections (Windows)'#010+
  '8*2Wh_Use huge code for units (ignored for models with CODE in a uniqu'+
  'e segment)'#010+
  '8*2Wm<x>_Set memory model'#010+
  '8*3WmTiny_Tiny memory model'#010+
  '8*3WmSmall_Small memory model (default)'#010+
  '8*3Wm','Medium_Medium memory model'#010+
  '8*3WmCompact_Compact memory model'#010+
  '8*3WmLarge_Large memory model'#010+
  '8*3WmHuge_Huge memory model'#010+
  '3*2WM<x>_Minimum Mac OS X deployment version: 10.4, 10.5.1, ... (Darwi'+
  'n)'#010+
  '4*2WM<x>_Minimum Mac OS X deployment version: 10','.4, 10.5.1, ... (Dar'+
  'win)'#010+
  'p*2WM<x>_Minimum Mac OS X deployment version: 10.4, 10.5.1, ... (Darwi'+
  'n)'#010+
  'P*2WM<x>_Minimum Mac OS X deployment version: 10.4, 10.5.1, ... (Darwi'+
  'n)'#010+
  '3*2WN_Do not generate relocation code, needed for debugging (Windows)',
  #010+
  '4*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
  'A*2WN_Do not generate relocation code, needed for debugging (Windows)'#010+
  'A*2Wp<x>_Specify the controller type; see fpc -i or fpc -iu for possib'+
  'le values'#010+
  'm*2Wp<x>_Specify th','e controller type; see fpc -i or fpc -iu for poss'+
  'ible values'#010+
  'V*2Wp<x>_Specify the controller type; see fpc -i or fpc -iu for possib'+
  'le values'#010+
  '3*2WP<x>_Minimum iOS deployment version: 3.0, 5.0.1, ... (iphonesim)'#010+
  '4*2WP<x>_Minimum iOS deploymen','t version: 8.0, 8.0.2, ... (iphonesim)'+
  #010+
  'a*2WP<x>_Minimum iOS deployment version: 7.0, 7.1.2, ... (Darwin)'#010+
  'A*2WP<x>_Minimum iOS deployment version: 3.0, 5.0.1, ... (Darwin)'#010+
  '3*2WR_Generate relocation code (Windows)'#010+
  '4*2WR_Generate relocation co','de (Windows)'#010+
  'A*2WR_Generate relocation code (Windows)'#010+
  '8*2Wt<x>_Set the target executable format'#010+
  '8*3Wtexe_Create a DOS .EXE file (default)'#010+
  '8*3Wtcom_Create a DOS .COM file (requires tiny memory model)'#010+
  'P*2WT_Specify MPW tool type application (','Classic Mac OS)'#010+
  '**2WX_Enable executable stack (Linux)'#010+
  '**1X_Executable options:'#010+
  '**2X9_Generate linkerscript for GNU Binutils ld older than version 2.1'+
  '9.1 (Linux)'#010+
  '**2Xc_Pass --shared/-dynamic to the linker (BeOS, Darwin, FreeBSD, Lin'+
  'ux)'#010+
  '**2Xd','_Do not search default library path (sometimes required for cro'+
  'ss-compiling when not using -XR)'#010+
  '**2Xe_Use external linker'#010+
  '**2Xf_Substitute pthread library name for linking (BSD)'#010+
  '**2Xg_Create debuginfo in a separate file and add a debuglink ','sectio'+
  'n to executable'#010+
  '**2XD_Try to link units dynamically      (defines FPC_LINK_DYNAMIC)'#010+
  '**2Xi_Use internal linker'#010+
  '**2XLA_Define library substitutions for linking'#010+
  '**2XLO_Define order of library linking'#010+
  '**2XLD_Exclude default order of stand','ard libraries'#010+
  '**2Xm_Generate link map'#010+
  '**2XM<x>_Set the name of the '#039'main'#039' program routine (default i'+
  's '#039'main'#039')'#010+
  '**2Xn_Use target system native linker instead of GNU ld (Solaris, AIX)'+
  #010+
  'F*2Xp<x>_First search for the compiler binary in the direct','ory <x>'#010+
  '**2XP<x>_Prepend the binutils names with the prefix <x>'#010+
  '**2Xr<x>_Set the linker'#039's rlink-path to <x> (needed for cross comp'+
  'ile, see the ld manual for more information) (BeOS, Linux)'#010+
  '**2XR<x>_Prepend <x> to all linker search paths (Be','OS, Darwin, FreeB'+
  'SD, Linux, Mac OS, Solaris)'#010+
  '**2Xs_Strip all symbols from executable'#010+
  '**2XS_Try to link units statically (default, defines FPC_LINK_STATIC)'#010+
  '**2Xt_Link with static libraries (-static is passed to linker)'#010+
  '**2Xv_Generate table f','or Virtual Entry calls'#010+
  '**2XV_Use VLink as external linker       (default on Amiga, MorphOS)'#010+
  '**2XX_Try to smartlink units             (defines FPC_LINK_SMART)'#010+
  '**1*_'#010+
  '**1?_Show this help'#010+
  '**1h_Shows this help without waiting'
);
