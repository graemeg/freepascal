{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2001 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


type
   IReadWriteSync = interface
{$ifdef FPC_HAS_FEATURE_THREADING}
     ['{7B108C52-1D8F-4CDB-9CDF-57E071193D3F}']
     procedure BeginRead;
     procedure EndRead;
     function BeginWrite : boolean;
     procedure EndWrite;
{$endif FPC_HAS_FEATURE_THREADING}
   end;

   TSimpleRWSync = class(TInterfacedObject,IReadWriteSync)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_THREADING}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   private
      crit : TRtlCriticalSection;
   public
      constructor Create; virtual;
      destructor  Destroy; override;
      function Beginwrite : boolean;
      procedure Endwrite;
      procedure Beginread;
      procedure Endread;
{$endif FPC_HAS_FEATURE_THREADING}
   end;

   TMultiReadExclusiveWriteSynchronizer = class(TInterfacedObject,IReadWriteSync)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_HAS_FEATURE_THREADING}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   private
      freaderqueue: peventstate;
      fwritelock : TRtlCriticalSection;
      fwaitingwriterlock: prtlevent;
      freadercount: cardinal;
      fwritelocked: longint;
   public
      constructor Create; virtual;
      destructor  Destroy; override;
      function Beginwrite : boolean;
      procedure Endwrite;
      procedure Beginread;
      procedure Endread;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_HAS_FEATURE_THREADING}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
   end;
