{ this program just links all externals, declared in the xext unit }
program xext_linktest;
uses
  xext;
begin
  halt(0);
  XSetExtensionErrorHandler(nil);
  XMissingExtension(nil,nil);
end.
