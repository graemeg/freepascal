{$IFDEF OGC_INTERFACE}
type
  FHANDLE = pointer;

  _tplfile = record
    type_ : cint;
    ntextures : cint;
    texdesc : pointer;
    tpl_file : FHANDLE;
  end;
  TPLFile = _tplfile;
  PTPLFile = ^TPLFile;


function TPL_OpenTPLFromFile(tdf: PTPLFile; file_name: pcchar): cint32; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function TPL_OpenTPLFromMemory(tdf: PTPLFile; memory: pointer; len: cuint32): cint32; cdecl; external;

function TPL_GetTexture(tdf: PTPLFile; id: cint32; texObj: PGXTexObj): cint32; cdecl; external;

function TPL_GetTextureCI(tdf: PTPLFile; id: cint32; texObj: PGXTexObj; tlutObj: PGXTlutObj; tluts: cuint8): cint32; cdecl; external;

function TPL_GetTextureInfo(tdf: PTPLFile; id: cint32; fmt: pcuint32; width, height: pcuint16): cint32; cdecl; external;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function TPL_OpenTPLFromMemory(tdf: PTPLFile; const memory: pointer; len: cuint32)
  : cint32; cdecl; external;

function TPL_GetTexture(tdf: PTPLFile; id: cint32; texObj: PGXTexObj): cint32; cdecl; external;

function TPL_GetTextureCI(tdf: PTPLFile; id: cint32; texObj: PGXTexObj;
  tlutObj: PGXTlutObj; tluts: cuint8): cint32; cdecl; external;

function TPL_GetTextureInfo(tdf: PTPLFile; id: cint32; fmt: pcuint32;
  width, height: pcuint16): cint32; cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

procedure TPL_CloseTPLFile(tdf: PTPLFile); cdecl; external;
{$ENDIF OGC_INTERFACE}

