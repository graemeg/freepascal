<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{%MainUnit ../symbian.pas}

{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by contributors of the Free Pascal Compiler

    This file is a pascal translation of the Symbian OS headers

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ e32\include\e32err.h

  Copyright (c) 1994-2001 Symbian Ltd. All rights reserved.
 }


//#include <e32def.h>




{
System wide error code - no error.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrNone=0;




{
System wide error code - item not found.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrNotFound=(-1); // Must remain set to -1




{
System wide error code - an error that has no specific categorisation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrGeneral=(-2);




{
System wide error code - indicates an operation that has been cancelled.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCancel=(-3);




{
System wide error code - an attempt to allocate memory has failed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrNoMemory=(-4);




{
System wide error code - some functionality is not supported in a given context.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

There may be many reasons for this; for example, a device may not support
some specific behaviour.
}
const KErrNotSupported=(-5);




{
System wide error code - an argument is out of range.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrArgument=(-6);




{
System wide error code - a calculation has lost precision.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

This error arises when converting from an internal 96-bit real representation
to a TReal32; the exponent of the internal representation is so small
that the 32-bit real cannot contain it.
}
const KErrTotalLossOfPrecision=(-7);




{
System wide error code - an invalid handle has been passed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

A function involving a resource owned by a server or the kernel has
specified an invalid handle.
}
const KErrBadHandle=(-8);




{
System wide error code - indicates an overflow in some operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

In the context of mathematical or time/date functions, indicates a calculation
that has produced arithmetic overflow exceeding the bounds allowed by
the representation.

In the context of data transfer, indicates that a buffer has over-filled
without being emptied soon enough.
}
const KErrOverflow=(-9);




{
System wide error code - indicates an underflow in some operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

In the context of mathematical or time/date functions, indicates a calculation
that has produced a result smaller than the smallest magnitude of
a finite number allowed by the representation.

In the context of data transfer, indicates that a buffer was under-filled
when data was required.
}
const KErrUnderflow=(-10);




{
System wide error code - an object already exists.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

An object of some name/type is to be created, but an object of
that name/type already exists.
}
const KErrAlreadyExists=(-11);




{
System wide error code - in the context of file operations, a path
was not found.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrPathNotFound=(-12);




{
System wide error code - a handle refers to a thread that has died.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDied=(-13);




{
System wide error code - a requested resource is already in exclusive use.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrInUse=(-14);




{
System wide error code - a client/server send/receive operation cannot run,
because the server has terminated.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrServerTerminated=(-15);




{
System wide error code - a client/server send/receive operation cannot run,
because the server is busy handling another request.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrServerBusy=(-16);




{
System wide error code - indicates that an operation is complete,
successfully or otherwise.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

This code may be used to indicate that some follow on operation can take place.
It does not necessarily indicate an error condition.
}
const KErrCompletion=(-17);




{
System wide error code - indicates that a device required by an i/o operation
is not ready to start operations.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

A common reason for returning this code is because a device has not been
initialised, or has no power.
}
const KErrNotReady=(-18);




{
System wide error code - a device is of unknown type.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrUnknown=(-19);




{
System wide error code - indicates that some media is not formatted properly,
or links between sections of it have been corrupted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCorrupt=(-20);




{
System wide error code - access to a file is denied, because the permissions on
the file do not allow the requested operation to be performed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrAccessDenied=(-21);




{
System wide error code - an operation cannot be performed, because the part
of the file to be read or written is locked.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrLocked=(-22);




{
System wide error code - during a file write operation, not all the data
could be written.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrWrite=(-23);




{
System wide error code - a volume which was to be used for a file system
operation has been dismounted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDisMounted=(-24);




{
System wide error code - indicates that end of file has been reached.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

Note that RFile::Read() is a higher-level interface. When the end of
the file is reached, it returns zero bytes in the destination descriptor, and
a KErrNone return value. KErrEof is not used for this purpose; other error
conditions are returned only if some other error condition was indicated on
the file.
}
const KErrEof=(-25);




{
System wide error code - a write operation cannot complete, because the disk
is full.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDiskFull=(-26);




{
System wide error code - a driver DLL is of the wrong type.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadDriver=(-27);




{
System wide error code - a file name or other object name does not conform to
the required syntax.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadName=(-28);




{
System wide error code - a communication line has failed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsLineFail=(-29);




{
System wide error code - a frame error has occurred in
a communications operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsFrame=(-30);




{
System wide error code - an overrun has been detected by
a communications driver.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsOverrun=(-31);




{
System wide error code - a parity error has occurred in communications.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsParity=(-32);




{
System wide error code - an operation has timed out.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrTimedOut=(-33);




{
System wide error code - a session could not connect.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCouldNotConnect=(-34);




{
System wide error code - a session could not disconnect.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCouldNotDisconnect=(-35);




{
System wide error code - a function could not be executed because the required
session was disconnected.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDisconnected=(-36);




{
System wide error code - a library entry point was not of the required type.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadLibraryEntryPoint=(-37);




{
System wide error code - a non-descriptor parameter was passed by
a client interface, when a server expected a descriptor.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadDescriptor=(-38);




{
System wide error code - an operation has been aborted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrAbort=(-39);




{
System wide error code - a number was too big.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrTooBig=(-40);




{
System wide error code - a divide-by-zero operation has been attempted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDivideByZero=(-41);		// Added by AnnW




{
System wide error code - insufficient power was available to
complete an operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadPower=(-42);




{
System wide error code - an operation on a directory has failed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDirFull=(-43);




{
System wide error code - an operation cannot be performed because
the necessary hardware is not available.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrHardwareNotAvailable=(-44);




{
System wide error code - the completion status when an outstanding
client/server message is completed because a shared session has been closed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrSessionClosed=(-45);




{
System wide error code - an operation cannot be performed due to
a potential security violation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrPermissionDenied=(-46);



{
System wide error code - a requested extension function is not
supported by the object concerned.

}
const KErrExtensionNotSupported=(-47);



{
System wide error code - a break has occurred in
a communications operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsBreak=(-48);

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{%MainUnit ../symbian.pas}

{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by contributors of the Free Pascal Compiler

    This file is a pascal translation of the Symbian OS headers

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ e32\include\e32err.h

  Copyright (c) 1994-2001 Symbian Ltd. All rights reserved.
 }


//#include <e32def.h>




{
System wide error code - no error.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrNone=0;




{
System wide error code - item not found.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrNotFound=(-1); // Must remain set to -1




{
System wide error code - an error that has no specific categorisation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrGeneral=(-2);




{
System wide error code - indicates an operation that has been cancelled.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCancel=(-3);




{
System wide error code - an attempt to allocate memory has failed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrNoMemory=(-4);




{
System wide error code - some functionality is not supported in a given context.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

There may be many reasons for this; for example, a device may not support
some specific behaviour.
}
const KErrNotSupported=(-5);




{
System wide error code - an argument is out of range.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrArgument=(-6);




{
System wide error code - a calculation has lost precision.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

This error arises when converting from an internal 96-bit real representation
to a TReal32; the exponent of the internal representation is so small
that the 32-bit real cannot contain it.
}
const KErrTotalLossOfPrecision=(-7);




{
System wide error code - an invalid handle has been passed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

A function involving a resource owned by a server or the kernel has
specified an invalid handle.
}
const KErrBadHandle=(-8);




{
System wide error code - indicates an overflow in some operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

In the context of mathematical or time/date functions, indicates a calculation
that has produced arithmetic overflow exceeding the bounds allowed by
the representation.

In the context of data transfer, indicates that a buffer has over-filled
without being emptied soon enough.
}
const KErrOverflow=(-9);




{
System wide error code - indicates an underflow in some operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

In the context of mathematical or time/date functions, indicates a calculation
that has produced a result smaller than the smallest magnitude of
a finite number allowed by the representation.

In the context of data transfer, indicates that a buffer was under-filled
when data was required.
}
const KErrUnderflow=(-10);




{
System wide error code - an object already exists.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

An object of some name/type is to be created, but an object of
that name/type already exists.
}
const KErrAlreadyExists=(-11);




{
System wide error code - in the context of file operations, a path
was not found.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrPathNotFound=(-12);




{
System wide error code - a handle refers to a thread that has died.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDied=(-13);




{
System wide error code - a requested resource is already in exclusive use.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrInUse=(-14);




{
System wide error code - a client/server send/receive operation cannot run,
because the server has terminated.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrServerTerminated=(-15);




{
System wide error code - a client/server send/receive operation cannot run,
because the server is busy handling another request.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrServerBusy=(-16);




{
System wide error code - indicates that an operation is complete,
successfully or otherwise.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

This code may be used to indicate that some follow on operation can take place.
It does not necessarily indicate an error condition.
}
const KErrCompletion=(-17);




{
System wide error code - indicates that a device required by an i/o operation
is not ready to start operations.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

A common reason for returning this code is because a device has not been
initialised, or has no power.
}
const KErrNotReady=(-18);




{
System wide error code - a device is of unknown type.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrUnknown=(-19);




{
System wide error code - indicates that some media is not formatted properly,
or links between sections of it have been corrupted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCorrupt=(-20);




{
System wide error code - access to a file is denied, because the permissions on
the file do not allow the requested operation to be performed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrAccessDenied=(-21);




{
System wide error code - an operation cannot be performed, because the part
of the file to be read or written is locked.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrLocked=(-22);




{
System wide error code - during a file write operation, not all the data
could be written.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrWrite=(-23);




{
System wide error code - a volume which was to be used for a file system
operation has been dismounted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDisMounted=(-24);




{
System wide error code - indicates that end of file has been reached.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.

Note that RFile::Read() is a higher-level interface. When the end of
the file is reached, it returns zero bytes in the destination descriptor, and
a KErrNone return value. KErrEof is not used for this purpose; other error
conditions are returned only if some other error condition was indicated on
the file.
}
const KErrEof=(-25);




{
System wide error code - a write operation cannot complete, because the disk
is full.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDiskFull=(-26);




{
System wide error code - a driver DLL is of the wrong type.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadDriver=(-27);




{
System wide error code - a file name or other object name does not conform to
the required syntax.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadName=(-28);




{
System wide error code - a communication line has failed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsLineFail=(-29);




{
System wide error code - a frame error has occurred in
a communications operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsFrame=(-30);




{
System wide error code - an overrun has been detected by
a communications driver.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsOverrun=(-31);




{
System wide error code - a parity error has occurred in communications.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsParity=(-32);




{
System wide error code - an operation has timed out.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrTimedOut=(-33);




{
System wide error code - a session could not connect.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCouldNotConnect=(-34);




{
System wide error code - a session could not disconnect.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCouldNotDisconnect=(-35);




{
System wide error code - a function could not be executed because the required
session was disconnected.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDisconnected=(-36);




{
System wide error code - a library entry point was not of the required type.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadLibraryEntryPoint=(-37);




{
System wide error code - a non-descriptor parameter was passed by
a client interface, when a server expected a descriptor.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadDescriptor=(-38);




{
System wide error code - an operation has been aborted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrAbort=(-39);




{
System wide error code - a number was too big.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrTooBig=(-40);




{
System wide error code - a divide-by-zero operation has been attempted.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDivideByZero=(-41);		// Added by AnnW




{
System wide error code - insufficient power was available to
complete an operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrBadPower=(-42);




{
System wide error code - an operation on a directory has failed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrDirFull=(-43);




{
System wide error code - an operation cannot be performed because
the necessary hardware is not available.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrHardwareNotAvailable=(-44);




{
System wide error code - the completion status when an outstanding
client/server message is completed because a shared session has been closed.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrSessionClosed=(-45);




{
System wide error code - an operation cannot be performed due to
a potential security violation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrPermissionDenied=(-46);



{
System wide error code - a requested extension function is not
supported by the object concerned.

}
const KErrExtensionNotSupported=(-47);



{
System wide error code - a break has occurred in
a communications operation.

A system wide error code indicates an error in the environment, or in
user input from which a program may recover.
}
const KErrCommsBreak=(-48);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
