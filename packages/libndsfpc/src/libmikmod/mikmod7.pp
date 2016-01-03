unit mikmod7;
{$mode objfpc} 
{$apptype arm7}
{$define arm7}

{$J+}
{$INLINE ON}
{$MACRO ON}
{$PACKRECORDS C}

interface

uses
  ctypes, nds7;

{$linklib nds7}
{$linklib mikmod7}

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
