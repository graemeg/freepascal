unit mikmod9;
{$mode objfpc} 
{$apptype arm9}
{$define arm9}

{$J+}
{$INLINE ON}
{$MACRO ON}
{$PACKRECORDS C}

interface

uses
  ctypes, nds9;

{$linklib nds9}
{$linklib mikmod9}

{$linklib c}
{$linklib gcc}
{$linklib sysbase}

{$define NDS_INTERFACE}
{$include inc/mikmod.inc}
{$undef NDS_INTERFACE}

implementation
{$define NDS_IMPLEMENTATION}
{$include inc/mikmod.inc}
{$undef NDS_IMPLEMENTATION}

end.
