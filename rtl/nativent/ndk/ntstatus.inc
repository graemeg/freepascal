{%MainUnit ndk.pas}
{
    Native Development Kit for Native NT

    This file is part of the Free Pascal run time library.
    This units contains the status codes used by NT
    Copyright (c) 2010 by Sven Barth

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
  STATUS_SUCCESS                          = NTSTATUS($00000000);

  STATUS_PENDING                          = NTSTATUS($00000103);

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  STATUS_NO_MORE_FILES                    = NTSTATUS($80000006);
  STATUS_NO_MORE_ENTRIES                  = NTSTATUS($8000001A);

  STATUS_INVALID_HANDLE                   = NTSTATUS($C0000008);
  STATUS_INVALID_PARAMETER                = NTSTATUS($C000000D);
  STATUS_OBJECT_TYPE_MISMATCH             = NTSTATUS($C0000024);
  STATUS_OBJECT_NAME_COLLISION            = NTSTATUS($C0000035);


=======
  STATUS_OBJECT_TYPE_MISMATCH             = NTSTATUS($C0000024);
  STATUS_OBJECT_NAME_COLLISION            = NTSTATUS($C0000035);

>>>>>>> graemeg/cpstrnew
=======
  STATUS_OBJECT_TYPE_MISMATCH             = NTSTATUS($C0000024);
  STATUS_OBJECT_NAME_COLLISION            = NTSTATUS($C0000035);

>>>>>>> graemeg/cpstrnew
=======
  STATUS_OBJECT_TYPE_MISMATCH             = NTSTATUS($C0000024);
  STATUS_OBJECT_NAME_COLLISION            = NTSTATUS($C0000035);

>>>>>>> graemeg/cpstrnew
=======
  STATUS_OBJECT_TYPE_MISMATCH             = NTSTATUS($C0000024);
  STATUS_OBJECT_NAME_COLLISION            = NTSTATUS($C0000035);

>>>>>>> origin/cpstrnew
