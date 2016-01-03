{
    Copyright (c) 2002 by Mark Eckstein
    Copyright (c) 2003 by Yuri Prokushev (prokushev@freemail.ru)

    eCS Clock API (part of standard eCS installation)

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

1 **********************************************************************}

{
The Clock API is a set of functions, which offer you a variety of
time-related operations. You can use it like any other DLL provided
with OS/2 and eComStation. However, the Clock API DLL is not Rexx
enabled.
}

Unit clkdll;

Interface

Uses
  DosCalls;

{
If you need to know the universal world time call this API. The
value it returns is updated every 250 ms, so it's rather meant
for not-too-high precision tasks.
Return values:   0 ... ok
               128 ... failed
}
Function ClkQueryUTC(var Value: TDateTime): Cardinal; cdecl;
    external 'clkdll' name 'ClkQueryUTC'; {index 00001}

{
You can obtain the time zone offset using this API.
Return values:   0 ... ok
               128 ... failed
}

Function ClkQueryUTCOffset(var Value: Longint): Cardinal; cdecl;
    external 'clkdll' name 'ClkQueryUTCOffset'; {index 00002}

{
Get the difference between standard time and summer time in
seconds by calling this API.
Return values:   0 ... ok
               128 ... failed
}
Function ClkQueryDSTAdvance(var Value: Integer): Cardinal; cdecl;
    external 'clkdll' name 'ClkQueryDSTAdvance'; {index 00003}

{
This returns the standard time zone identifier and date and
time of the next change to standard time.
Return values:   0 ... ok
                30 ... ID only
                40 ... Date only
               128 ... failed
}
Function ClkQuerySTData(ID: PChar; var NextDate: TDateTime): Cardinal; cdecl;
    external 'clkdll' name 'ClkQuerySTData'; {index 00004}

{
This returns the summer time zone identifier and date and
time of the next change to summer time.
Return values:   0 ... ok
                30 ... ID only
                40 ... Date only
               128 ... failed
}
Function ClkQueryDSTData(ID: PChar; var NextDate: TDateTime): Cardinal; cdecl;
    external 'clkdll' name 'ClkQueryDSTData'; {index 00005}

{
This returns the TZ string that is currently in use, independent
from the actual TZ environment setting.
Return values:   0 ... ok
               128 ... failed
}
Function ClkQueryTimeZone(value: PChar): Cardinal; cdecl;
    external 'clkdll' name 'ClkQueryTimeZone'; {index 00006}

{
If you want to know in which hemisphere the computer is located
call this API.
Value is either 0 for southern or 1 for northern hemisphere.
Return values:   0 ... ok
               128 ... failed
}
Function ClkQueryHemisphere(var Value: Integer): Cardinal; cdecl;
    external 'clkdll' name 'ClkQueryHemisphere'; {index 00007}

{
The system's uptime is returned when calling this API. Opposed
to the OS/2 API for the same purpose this one's rollover will
be after approximately 130 years (untested!).
Return values:   0 ... ok
               128 ... failed
}
Function ClkQueryUptime(var Days, Hours, Minutes, Seconds, Hundredths: Cardinal): Cardinal; cdecl;
    external 'clkdll' name 'ClkQueryUptime'; {index 00008}

{
Returns, whether a synchronization can be done (Value == 1) or
not (Value == 0). This depends on the chosen connection type and
the current connection status.
Return values:   0 ... ok
               128 ... failed
}
Function ClkCanSynchronize(var Value: Integer): Cardinal; cdecl;
    external 'clkdll' name 'ClkCanSynchronize'; {index 00015}

{
Triggers a time synchronization.
Return values:   0 ... ok
                64 ... writing to pipe failed
                70 ... can't synchronize
               128 ... clkbasic version could not be determined
               129 ... clkbasic version unknow
}
Function ClkDoSynchronize: Cardinal; cdecl;
    external 'clkdll' name 'ClkDoSynchronize'; {index 00013}

{
Queries, whether clkbasic is currently in the process of
synchronizing (Value == 1: yes, Value == 0: no)
Return values:   0 ... ok
               128 ... failed
}
Function ClkIsSynchronizing(var Value: Integer): Cardinal; cdecl;
    external 'clkdll' name 'ClkIsSynchronizing'; {index 00014}

{
Returns the synchronization data. Date and time of the last
attempted synchronization, date and time of the last successful
synchronization, indicator for success of last attempt and the host
from where the time could be synched successfully.
Return values:   0 ... ok
               128 ... failed
}
Function ClkGetSynchData(var Last, LastSuccess: TDateTime;
                         var LastWasSuccessful: Integer;
                         LastSuccessfulHost: PChar): Cardinal; cdecl;
    external 'clkdll' name 'ClkGetSynchData'; {index 00016}

{ not documented
    external 'clkdll' name 'ClkQueryBasePath'; {index 00009}
    external 'clkdll' name 'ClkQueryLanguagePath'; {index 00010}
    external 'clkdll' name 'ClkQueryDialupState'; {index 00011}
    external 'clkdll' name 'ClkQueryVersion'; {index 00012}
    external 'clkdll' name 'ClkCallSettings'; {index 00017}
    external 'clkdll' name 'ClkCallSchedulerHelper'; {index 00018}
}

Implementation

End.
