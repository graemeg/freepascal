<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: jtypes.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//---------------------------------------------------------------------------------
{
//---------------------------------------------------------------------------------
// libgba compatible section macros
//---------------------------------------------------------------------------------
#define ITCM_CODE	__attribute__((section(".itcm"), long_call))

#define DTCM_DATA	__attribute__((section(".dtcm")))
#define DTCM_BSS	__attribute__((section(".sbss")))
#define ALIGN(m)	__attribute__((aligned (m)))

#define PACKED __attribute__ ((packed))
#define packed_struct struct PACKED
}
//---------------------------------------------------------------------------------
// These are linked to the bin2o macro in the Makefile
//---------------------------------------------------------------------------------
{ ### To do
#define GETRAW(name)      (name)
#define GETRAWSIZE(name)  ((int)name##_size)
#define GETRAWEND(name)  ((int)name##_end)
}

{$ifdef NDS_INTERFACE}
type
  (* Unsigned types *)
  uint8  = cuint8;
  uint16 = cuint16;
  uint32 = cuint32;
  uint64 = cuint64;
  puint8 = pcuint8;
  puint16 = pcuint16;
  puint32 = pcuint32;
  puint64 = pcuint64;
  
  (* Signed types *)
  int8  = cint8;
  int16 = cint16;
  int32 = cint32;
  pint8 = pcint8;
  pint16 = pcint16;
  pint32 = pcint32;
  pint64 = pcint64;


  float32 = cfloat;

{
typedef volatile uint8          vuint8;
typedef volatile uint16         vuint16;
typedef volatile uint32         vuint32;
typedef volatile uint64         vuint64;

typedef volatile int8           vint8;
typedef volatile int16          vint16;
typedef volatile int32          vint32;
typedef volatile int64          vint64;

typedef volatile float32        vfloat32;
typedef volatile float64        vfloat64;
}


{
typedef volatile int32          vfixed;
}

  u8  = cuint8;
  u16 = cuint16;
  u32 = cuint32;
  u64 = cuint64;
  pu8 = pcuint8;
  pu16 = pcuint16;
  pu32 = pcuint32;
  pu64 = pcuint64;
  
  s8  = cint8;
  s16 = cint16;
  s32 = cint32;
  s64 = cint64;
  ps8 = pcint8;
  ps16 = pcint16;
  ps32 = pcint32;
  ps64 = pcint64;

{
typedef volatile u8          vu8;
typedef volatile u16         vu16;
typedef volatile u32         vu32;
typedef volatile u64         vu64;

typedef volatile s8           vs8;
typedef volatile s16          vs16;
typedef volatile s32          vs32;
typedef volatile s64          vs64;
}

  touchPosition = record
    x: cint16;
    y: cint16;
    px: cint16;
    py: cint16;
    z1: cint16;
    z2: cint16;
  end;
  TtouchPosition = touchPosition;
  PtouchPosition = ^touchPosition;

  //IntFn = function: pointer;
  VoidFunctionPointer = function: pointer; cdecl;
  //fp = function: pointer;

  IntFn = procedure;
  //VoidFunctionPointer = procedure; cdecl;
  fp = procedure;
{$endif NDS_INTERFACE}

{$ifdef NDS_IMPLEMENTATION} 
function BIT(n: cint): cint; inline;
begin
  BIT := (1 shl (n));
end;
{$endif NDS_IMPLEMENTATION}


{$ifdef NDS_INTERFACE}
function BIT(n: cint): cint; inline;
{$endif NDS_INTERFACE}
