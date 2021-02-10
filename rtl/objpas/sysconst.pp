{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2003 by Florian Klaempfl
    member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
unit SysConst;

interface

{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
{$H+}
resourcestring
{$else FPC_HAS_FEATURE_ANSISTRINGS}
const
{$endif FPC_HAS_FEATURE_ANSISTRINGS}

{ from old str*.inc files }
  SListIndexError        = 'List index (%d) out of bounds';
  SParamIsNegative       = 'Parameter "%s" cannot be negative.';
  SListCapacityError     = 'List capacity (%d) exceeded.';
  SAbortError            = 'Operation aborted';
  SAbstractError         = 'Abstract method called';
  SAccessDenied          = 'Access denied';
  SAccessViolation       = 'Access violation';
  SArgumentMissing       = 'Missing argument in format "%s"';
  SAssertError           = '%s (%s, line %d)';
  SAssertionFailed       = 'Assertion failed';
  SBusError              = 'Bus error or misaligned data access';
  SCannotCreateEmptyDir  = 'Cannot create empty directory';
  SControlC              = 'Control-C hit';
  SDiskFull              = 'Disk Full';
  SDispatchError         = 'No variant method call dispatch';
  SDivByZero             = 'Division by zero';
  SEndOfFile             = 'Read past end of file';
  SErrInvalidDateMonthWeek = 'Year %d, month %d, Week %d and day %d is not a valid date.';
  SerrInvalidHourMinuteSecMsec = '%d:%d:%d.%d is not a valid time specification';
  SErrInvalidDateWeek    = '%d %d %d is not a valid dateweek';
  SErrInvalidDayOfWeek   = '%d is not a valid day of the week';
  SErrInvalidDayOfWeekInMonth = 'Year %d Month %d NDow %d DOW %d is not a valid date';
  SErrInvalidDayOfYear   = 'Year %d does not have a day number %d';
  SErrInvalidTimeStamp   = 'Invalid date/timestamp : "%s"';
  SInvalidJulianDate            = '%f Julian cannot be represented as a DateTime';
  SErrIllegalDateFormatString   = '"%s" is not a valid date format string';
  SErrInvalidTimeFormat  = '"%s" is not a valid time';
  SExceptionErrorMessage = 'Exception at %p: %s';
  SExceptionStack        = 'Exception stack error';
  SExecuteProcessFailed  = 'Failed to execute "%s", error code: %d';
  SExternalException     = 'External exception %x';
  SFileNotAssigned       = 'File not assigned';
  SFileNotFound          = 'File not found';
  SFileNotOpen           = 'File not open';
  SFileNotOpenForInput   = 'File not open for input';
  SFileNotOpenForOutput  = 'File not open for output';
  SInValidFileName       = 'Invalid filename';
  SIntOverflow           = 'Arithmetic overflow';
  SIntfCastError         = 'Interface not supported';
  SInvalidArgIndex       = 'Invalid argument index in format "%s"';
  SInvalidBCD            = '%x is an invalid BCD value';
  SInvalidBoolean        = '"%s" is not a valid boolean.';
  SInvalidCast           = 'Invalid type cast';
  SinvalidCurrency       = 'Invalid currency: "%s"';
  SInvalidDateTime       = '"%s" is not a valid date/time value.';
  SInvalidDateTimeFloat  = '%f is not a valid date/time value.';
  SInvalidDrive          = 'Invalid drive specified';
  SInvalidFileHandle     = 'Invalid file handle';
  SInvalidFloat          = '"%s" is an invalid float';
  SInvalidFormat         = 'Invalid format specifier : "%s"';
  SInvalidGUID           = '"%s" is not a valid GUID value';
  SInvalidInput          = 'Invalid input';
  SInvalidInteger        = '"%s" is an invalid integer';
  SInvalidOp             = 'Invalid floating point operation';
  SInvalidPointer        = 'Invalid pointer operation';
  SInvalidVarCast        = 'Invalid variant type cast';
  SInvalidVarNullOp      = 'Invalid NULL variant operation';
  SInvalidVarOp          = 'Invalid variant operation';
  SInvalidBinaryVarOp    = 'Invalid variant operation %s %s %s';
  SInvalidUnaryVarOp     = 'Invalid variant operation %s %s';
  SInvalidVarOpWithHResultWithPrefix = 'Invalid variant operation (%s%.8x)'+LineEnding+'%s';
  SNoError               = 'No error.';
  SNoThreadSupport       = 'Threads not supported. Recompile program with thread driver.';
  SNoDynLibsSupport      = 'Dynamic libraries not supported. Recompile program with dynamic library driver.';
  SMissingWStringManager = 'Widestring manager not available. Recompile program with appropriate manager.';
  SSigQuit               = 'SIGQUIT signal received.';
  SObjectCheckError      = 'Object reference is Nil or VMT is damaged';
  SOSError               = 'System error, (OS Code %d):'+LineEnding+'%s';
  SOutOfMemory           = 'Out of memory';
  SOverflow              = 'Floating point overflow';
  SPrivilege             = 'Privileged instruction';
  SRangeError            = 'Range check error';
  SStackOverflow         = 'Stack overflow or stack misalignment';
  SSafecallException     = 'Exception in safecall method';
  SiconvError            = 'iconv error';
  SThreadError           = 'Thread error';
  SSeekFailed            = 'Seek operation failed';

  STooManyOpenFiles      = 'Too many open files';
  SUnKnownRunTimeError   = 'Unknown Run-Time error : %3.3d';
  SUnderflow             = 'Floating point underflow';
  SUnkOSError            = 'An operating system call failed.';
  SUnknown               = 'Unknown run-time error code: ';
  SUnknownErrorCode      = 'Unknown error code: %d';
  SVarArrayBounds        = 'Variant array bounds error';
  SVarArrayCreate        = 'Variant array cannot be created';
  SVarArrayLocked        = 'Variant array locked';
  SVarBadType            = 'Invalid variant type';
  SVarInvalid            = 'Invalid argument';
  SVarInvalid1           = 'Invalid argument: %s';
  SVarNotArray           = 'Variant doesn''t contain an array';
  SVarNotImplemented     = 'Operation not supported';
  SVarOutOfMemory        = 'Variant operation ran out memory';
  SVarOverflow           = 'Variant overflow';
  SVarParamNotFound      = 'Variant Parameter not found';
  SVarTypeAlreadyUsedWithPrefix = 'Custom variant type (%s%.4x) already used by %s';
  SVarTypeConvertOverflow       = 'Overflow while converting variant of type (%s) into type (%s)';
  SVarTypeCouldNotConvert       = 'Could not convert variant of type (%s) into type (%s)';
  SVarTypeNotUsableWithPrefix   = 'Custom variant type (%s%.4x) is not usable';
  SVarTypeOutOfRangeWithPrefix  = 'Custom variant type (%s%.4x) is out of range';
  SVarTypeRangeCheck1           = 'Range check error for variant of type (%s)';
  SVarTypeRangeCheck2           = 'Range check error while converting variant of type (%s) into type (%s)';
  SVarTypeTooManyCustom         = 'Too many custom variant types have been registered';
  SVarUnexpected                = 'Unexpected variant error';
  SZeroDivide                   = 'Floating point division by zero';

  SFallbackError                = 'An error, whose error code is larger than can be returned to the OS, has occurred';

  SNoToolserver                 = 'Toolserver is not installed, cannot execute Tool';

  SNotValidCodePageName         = '%s is not a valid code page name';
  SInvalidCount                 = 'invalid count [%d]';
  SCharacterIndexOutOfBounds    = 'character index out of bounds [%d]';
  SInvalidDestinationArray      = 'invalid destination array';
  SInvalidDestinationIndex      = 'invalid destination index [%d]';

  SNoArrayMatch                 = 'Can''t match any allowed value at pattern position %d, string position %d.';
  SNoCharMatch                  = 'Mismatch char "%s" <> "%s" at pattern position %d, string position %d.';
  SHHMMError                    = 'mm in a sequence hh:mm is interpreted as minutes. No longer versions allowed! (Position : %d).' ;
  SFullpattern                  = 'Couldn''t match entire pattern string. Input too short at pattern position %d.';
  SPatternCharMismatch          = 'Pattern mismatch char "%s" at position %d.';
  SAMPMError                    = 'Hour >= 13 not allowed in AM/PM mode.';

  SShortMonthNameJan = 'Jan';
  SShortMonthNameFeb = 'Feb';
  SShortMonthNameMar = 'Mar';
  SShortMonthNameApr = 'Apr';
  SShortMonthNameMay = 'May';
  SShortMonthNameJun = 'Jun';
  SShortMonthNameJul = 'Jul';
  SShortMonthNameAug = 'Aug';
  SShortMonthNameSep = 'Sep';
  SShortMonthNameOct = 'Oct';
  SShortMonthNameNov = 'Nov';
  SShortMonthNameDec = 'Dec';

  SLongMonthNameJan = 'January';
  SLongMonthNameFeb = 'February';
  SLongMonthNameMar = 'March';
  SLongMonthNameApr = 'April';
  SLongMonthNameMay = 'May';
  SLongMonthNameJun = 'June';
  SLongMonthNameJul = 'July';
  SLongMonthNameAug = 'August';
  SLongMonthNameSep = 'September';
  SLongMonthNameOct = 'October';
  SLongMonthNameNov = 'November';
  SLongMonthNameDec = 'December';

  SShortDayNameMon = 'Mon';
  SShortDayNameTue = 'Tue';
  SShortDayNameWed = 'Wed';
  SShortDayNameThu = 'Thu';
  SShortDayNameFri = 'Fri';
  SShortDayNameSat = 'Sat';
  SShortDayNameSun = 'Sun';

  SLongDayNameMon = 'Monday';
  SLongDayNameTue = 'Tuesday';
  SLongDayNameWed = 'Wednesday';
  SLongDayNameThu = 'Thursday';
  SLongDayNameFri = 'Friday';
  SLongDayNameSat = 'Saturday';
  SLongDayNameSun = 'Sunday';

Function GetRunError(Errno : Word) : String;

Implementation

Function GetRunError(Errno : Word) : String;

begin
  Case Errno Of
     0  : Result:=SNoError;
     1  : Result:=SOutOfMemory;
     2  : Result:=SFileNotFound;
     3  : Result:=SInvalidFileName;
     4  : Result:=STooManyOpenFiles;
     5  : Result:=SAccessDenied;
     6  : Result:=SInvalidFileHandle;
     15 : Result:=SInvalidDrive;
     100 : Result:=SEndOfFile;
     101 : Result:=SDiskFull;
     102 : Result:=SFileNotAssigned;
     103 : Result:=SFileNotOpen;
     104 : Result:=SFileNotOpenForInput;
     105 : Result:=SFileNotOpenForOutput;
     106 : Result:=SInvalidInput;
     200 : Result:=SDivByZero;
     201 : Result:=SRangeError;
     203 : Result:=SOutOfMemory;
     204 : Result:=SInvalidPointer;
     205 : Result:=SOverFlow;
     206 : Result:=SUnderFlow;
     207 : Result:=SInvalidOp;
     211 : Result:=SAbstractError;
     214 : Result:=SBusError;
     215 : Result:=SIntOverFlow;
     216 : Result:=SAccessViolation;
     217 : Result:=SPrivilege;
     218 : Result:=SControlC;
     219 : Result:=SInvalidCast;
     220 : Result:=SInvalidVarCast;
     221 : Result:=SInvalidVarOp;
     222 : Result:=SDispatchError;
     223 : Result:=SVarArrayCreate;
     224 : Result:=SVarNotArray;
     225 : Result:=SVarArrayBounds;
     227 : Result:=SAssertionFailed;
     228 : Result:=SExternalException;
     229 : Result:=SIntfCastError;
     230 : Result:=SSafecallException;
     231 : Result:=SExceptionStack;
     232 : Result:=SNoThreadSupport;
     233 : Result:=SMissingWStringManager;
     235 : Result:=SNoDynLibsSupport;

     255 : Result:=SFallbackError;

     {Error codes larger than 255 cannot be returned as an exit code to the OS,
      for some OS's. If this happens, error 255 is returned instead.
      Errors for which it is important that they can be distinguished,
      shall be below 255}

     {Error in the range 900 - 999 is considered platform specific}

     900 : Result:=SNoToolserver;    {Mac OS specific}

  end;
  If length(Result)=0 then
    begin
      Str(Errno:3,Result);
      Result:=SUnknown+Result;
    end;
end;

end.
