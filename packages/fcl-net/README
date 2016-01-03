This directory contains a pure-pascal netdb implementation:
It is written mainly to be able to implement network applications that
do hostname lookups independent of the C library.

The uriparser unit contains a parser for URI strings: It decomposes the URI
in its various elements. The opposite can also be done: from various
elements create a complete URI

This provides the equivalent of the Inet unit, but the implementation is
written completely in pascal. It parses the hosts,services and networks
files just as the C library does (it should, anyway). 

The DNS routines also do a DNS lookup and parse /etc/resolv.conf
The 'domain' and 'search' entries in this file are parsed, but ignored.
Only the 'nameserver' entries are used at the moment.

The various test programs show how to use this.

Enjoy!

Michael.
