(*
  $Id: in.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  DSWifi Project - socket emulation layer defines/prototypes (netinet/in.h)
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

{$ifdef NDS_INTERFACE}
const
  INADDR_ANY        = $00000000;
  INADDR_BROADCAST  = $FFFFFFFF;
  INADDR_NONE       = $FFFFFFFF;

type
  in_addr = packed record
    s_addr: culong;
  end;
  pin_addr = ^in_addr;

  sockaddr_in = packed record
    sin_family: cushort;
    sin_port: cushort;
    sin_addr: in_addr;
    sin_zero: array [0..7] of cuchar;
  end;


function inet_addr(const cp: pchar): culong; cdecl; external;
function inet_aton(const cp: pchar; inp: pin_addr): cint; cdecl; external;
function inet_ntoa(_in: in_addr): pcchar; cdecl; external;
{$endif NDS_INTERFACE}
