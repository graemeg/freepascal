#!/usr/bin/perl
#
# find not used messages

unlink("./msgidx.inc");
unlink("./msgtxt.inc");
@compiler_src = (glob("./*.inc"),glob("./*/*.inc"),
                 glob("./*.pas"),glob("./*/*.pas"));

open(MESSAGE_FILE, "< ./msg/errore.msg") or
  die "Couldn't open <./msg/errore.msg> for reading: $!\n";

open(FOUND, "> MSG-OK.TXT") or
  die "Couldn't open <MSG-OK.TXT> for writing: $!\n";

select FOUND; $| = 1;

open(NOT_FOUND, "> MSG_BAD.TXT") or
  die "Couldn't open <MSG_BAD.TXT> for writing: $!\n";

select NOT_FOUND; $| = 1;

while (<MESSAGE_FILE>)
{
  if (/^(\w\w\w*?_\w\w*?_\w\w*?)=/)
  {
    $msg = $1;
    $found = `grep -il $msg @compiler_src`;
    if ($found) {
      print stderr "$msg\n";
      print FOUND $msg . "\n";
    }
    else {
      print stderr "NOT FOUND \t $msg\n";
      print NOT_FOUND $msg . "\n";
    }
  }
}

close(IN);
close(FOUND);
close(NOT_FOUND);
