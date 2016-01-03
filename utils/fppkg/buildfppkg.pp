unit buildfppkg;

  interface

  { 
    these units cause compilation errors on targets eg go32v2 for which fppkg 
    not implemented properly, so remove them by ifdef. jl Aug 2009 
  }

{$ifndef go32v2} 
    uses
      fprepos,fpxmlrep,pkgoptions,pkgglobals,pkgmessages,pkghandler,pkgmkconv,pkgdownload,pkgfpmake,pkgcommands,pkgrepos,pkgwget,pkglnet;
{$endif} 

  implementation

end.


