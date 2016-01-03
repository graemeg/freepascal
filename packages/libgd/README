This is the interface unit for the gd library, version 1.8.4.
The gd library is a library to produce graphics files (jpeg,png and wbmp)
available from http://www.boutell.com/gd/.

The gd unit is a straight translation of the library headers.
It has some additional calls which make it more pascal like:

{overloaded pascal functions}
function fopen(a,b:string):pFile;
procedure gdImageChar(im:gdImagePtr; f:gdFontPtr; x:longint; y:longint; c:char; color:longint);
procedure gdImageCharUp(im:gdImagePtr; f:gdFontPtr; x:longint; y:longint; c:char; color:longint); 
procedure gdImageString(im:gdImagePtr; f:gdFontPtr; x:longint; y:longint; s:string;  color:longint); 
procedure gdImageStringUp(im:gdImagePtr; f:gdFontPtr; x:longint; y:longint; s:string; color:longint); 
procedure gdImageString16(im:gdImagePtr; f:gdFontPtr; x:longint; y:longint; s:string; color:longint); 
procedure gdImageStringUp16(im:gdImagePtr; f:gdFontPtr; x:longint; y:longint; s:string; color:longint); 
{$ifdef hasttf}
function  gdImageStringTTF(im:PgdImage; brect:Plongint; fg:longint; fontlist:string; ptsize:double; angle:double; x:longint; y:longint; astring:string): string;
function  gdImageStringFT(im:PgdImage; brect:Plongint; fg:longint; fontlist:string; ptsize:double; angle:double; x:longint; y:longint; astring:string):string;
{$endif}

These functions will work with both ansistrings and shortstrings; The unit
can be compiled in both the {$H+} as the {$H-} state.

The hasttf define should be defined for libraries which have TTF2 lib 
support compiled in.

The gdtest example is adapted from the example of the gd unit by Mike Bradbery.
It shows how to use the pascal-like functions in the gd unit instead of the
raw C like functions which use pchars.

usage:
 gdtest chart.png

the gdtestcgi example shows how to output an image to standard output,
this can be used for CGI scripts.

Michael.

