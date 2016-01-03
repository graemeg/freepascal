<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: ndsmotion.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2007
  		Michael Noland (joat)
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
   
  Copyright (C) 2007  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifdef NDS_INTERFACE}
type
  MotionCalibration = record
    xoff, yoff, zoff, goff: cshort;
    xsens, ysens, zsens, gsens: cshort;
  end;
  TMotionCalibration = MotionCalibration;
  PMotionCalibration = ^MotionCalibration;
  
function motion_init(): cint; cdecl; external;
procedure motion_deinit(); cdecl; external;
function motion_read_x(): csint; cdecl; external;
function motion_read_y(): csint; cdecl; external;
function motion_read_z(): csint; cdecl; external;
function motion_read_gyro(): csint; cdecl; external;
function motion_acceleration_x(): cint; cdecl; external;
function motion_acceleration_y(): cint; cdecl; external;
function motion_acceleration_z(): cint; cdecl; external;
procedure motion_set_sens_x(sens: cint); cdecl; external;
procedure motion_set_sens_y(sens: cint); cdecl; external;
procedure motion_set_sens_z(sens: cint); cdecl; external;
procedure motion_set_sens_gyro(sens: cint); cdecl; external;
procedure motion_set_offs_x(); cdecl; external;
procedure motion_set_offs_y(); cdecl; external;
procedure motion_set_offs_z(); cdecl; external;
procedure motion_set_offs_gyro(); cdecl; external;
function motion_rotation(): cint; cdecl; external;
function motion_get_calibration(): PMotionCalibration; cdecl; external;
procedure motion_set_calibration(cal: PMotionCalibration); cdecl; external;
procedure motion_enable_ain_1(); cdecl; external;
procedure motion_enable_ain_2(); cdecl; external;
function motion_read_ain_1(): cint; cdecl; external;
function motion_read_ain_2(): cint; cdecl; external;
{$endif NDS_INTERFACE}
