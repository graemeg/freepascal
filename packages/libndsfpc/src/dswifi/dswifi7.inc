(*
  $Id: dswifi7.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  DSWifi Project - Arm7 Library Header file (dswifi7.h)
  (C) 2005-2006 Stephen Stair - sgstair@akkit.org - http://www.akkit.org
  ****************************************************************************** 
  DSWifi Lib and test materials are licenced under the MIT open source licence:
  Copyright (c) 2005-2006 Stephen Stair
  
  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the "Software"), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do
  so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  Comments:

*)

{$include dswifi_version.inc}

{$ifdef NDS_INTERFACE}

type
  WifiSyncHandler = procedure;

procedure Read_Flash(address: cint; destination: pcchar; length: cint); cdecl; external;
function PowerChip_ReadWrite(cmd, data: cint): cint; cdecl; external;
procedure Wifi_Interrupt(); cdecl; external;
procedure Wifi_Update(); cdecl; external;
procedure Wifi_Init(WifiData: culong); cdecl; external;
procedure Wifi_Deinit(); cdecl; external;
procedure Wifi_Sync(); cdecl; external;
procedure Wifi_SetSyncHandler(sh: WifiSyncHandler); cdecl; external;
//procedure Wifi_SetSyncHandler(sh: pointer); cdecl; external;

{$endif NDS_INTERFACE}

