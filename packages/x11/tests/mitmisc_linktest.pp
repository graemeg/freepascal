{ this program just links all externals, declared in the mitmisc unit }
program mitmisc_linktest;
uses
  mitmisc;
begin
  halt(0);
  XMITMiscQueryExtension(nil,nil,nil);
  XMITMiscSetBugMode(nil,false);
  XMITMiscGetBugMode(nil);
end.
