{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{
    History:
    Added overlay functions for Pchar->Strings, functions
    and procedures.
    14 Jul 2000.

    Removed amigaoverlays, use smartlink instead.
    05 Nov 2002.

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening of
    the library.
    14 Jan 2003.

    Changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT rexx;

INTERFACE


USES exec;



{ === rexx/storage.h ==================================================
 *
 * Copyright (c) 1986, 1987 by William S. Hawes (All Rights Reserved)
 *
 * =====================================================================
 * Header file to define ARexx data structures.
}


{ The NexxStr structure is used to maintain the internal strings in REXX.
 * It includes the buffer area for the string and associated attributes.
 * This is actually a variable-length structure; it is allocated for a
 * specific length string, and the length is never modified thereafter
 * (since it's used for recycling).
 }

Type

    pNexxStr = ^tNexxStr;
    tNexxStr = record
    ns_Ivalue   : Longint;  { integer value                 }
    ns_Length   : Word;     { length in bytes (excl null)   }
    ns_Flags    : Byte;     { attribute flags               }
    ns_Hash     : Byte;     { hash code                     }
    ns_Buff     : Array [0..7] of Byte;
                    { buffer area for strings       }
    end;                { size: 16 bytes (minimum)      }

Const

    NXADDLEN    = 9;            { offset plus null byte         }

{ String attribute flag bit definitions                                }

    NSB_KEEP    = 0;            { permanent string?             }
    NSB_STRING  = 1;            { string form valid?            }
    NSB_NOTNUM  = 2;            { non-numeric?                  }
    NSB_NUMBER  = 3;            { a valid number?               }
    NSB_BINARY  = 4;            { integer value saved?          }
    NSB_FLOAT   = 5;            { floating point format?        }
    NSB_EXT = 6;            { an external string?           }
    NSB_SOURCE  = 7;            { part of the program source?   }

{ The flag form of the string attributes                               }

    NSF_KEEP    = 1;
    NSF_STRING  = 2;
    NSF_NOTNUM  = 4;
    NSF_NUMBER  = 8;
    NSF_BINARY  = 16;
    NSF_FLOAT   = 32;
    NSF_EXT = 64;
    NSF_SOURCE  = 128;

{ Combinations of flags                                                }

    NSF_INTNUM  = NSF_NUMBER + NSF_BINARY + NSF_STRING;
    NSF_DPNUM   = NSF_NUMBER + NSF_FLOAT;
    NSF_ALPHA   = NSF_NOTNUM + NSF_STRING;
    NSF_OWNED   = NSF_SOURCE + NSF_EXT    + NSF_KEEP;
    KEEPSTR = NSF_STRING + NSF_SOURCE + NSF_NOTNUM;
    KEEPNUM = NSF_STRING + NSF_SOURCE + NSF_NUMBER + NSF_BINARY;

{ The RexxArg structure is identical to the NexxStr structure, but
 * is allocated from system memory rather than from internal storage.
 * This structure is used for passing arguments to external programs.
 * It is usually passed as an "argstring", a pointer to the string buffer.
}

Type

    pRexxArg = ^tRexxArg;
    tRexxArg = record
    ra_Size     : Longint;  { total allocated length        }
    ra_Length   : Word;     { length of string              }
    ra_Flags    : Byte;     { attribute flags               }
    ra_Hash     : Byte;     { hash code                     }
    ra_Buff     : Array [0..7] of Byte;
                    { buffer area                   }
    end;                { size: 16 bytes (minimum)      }

{ The RexxMsg structure is used for all communications with REXX
 * programs.  It is an EXEC message with a parameter block appended.
}

    pRexxMsg = ^tRexxMsg;
    tRexxMsg = record
    rm_Node     : tMessage;  { EXEC message structure        }
    rm_TaskBlock    : Pointer;  { global structure (private)    }
    rm_LibBase  : Pointer;  { library base (private)        }
    rm_Action   : Longint;  { command (action) code         }
    rm_Result1  : Longint;  { primary result (return code)  }
    rm_Result2  : Longint;  { secondary result              }
    rm_Args     : Array [0..15] of STRPTR;
                    { argument block (ARG0-ARG15)   }

    rm_PassPort : pMsgPort;   { forwarding port               }
    rm_CommAddr : STRPTR;   { host address (port name)      }
    rm_FileExt  : STRPTR;   { file extension                }
    rm_Stdin    : Longint;  { input stream (filehandle)     }
    rm_Stdout   : Longint;  { output stream (filehandle)    }
    rm_avail    : Longint;  { future expansion              }
    end;                { size: 128 bytes               }

Const

    MAXRMARG        = 15;       { maximum arguments             }

{ Command (action) codes for message packets                           }

    RXCOMM      = $01000000;    { a command-level invocation    }
    RXFUNC      = $02000000;    { a function call               }
    RXCLOSE     = $03000000;    { close the REXX server         }
    RXQUERY     = $04000000;    { query for information         }
    RXADDFH     = $07000000;    { add a function host           }
    RXADDLIB        = $08000000;    { add a function library        }
    RXREMLIB        = $09000000;    { remove a function library     }
    RXADDCON        = $0A000000;    { add/update a ClipList string  }
    RXREMCON        = $0B000000;    { remove a ClipList string      }
    RXTCOPN     = $0C000000;    { open the trace console        }
    RXTCCLS     = $0D000000;    { close the trace console       }

{ Command modifier flag bits                                           }

    RXFB_NOIO       = 16;       { suppress I/O inheritance?     }
    RXFB_RESULT     = 17;       { result string expected?       }
    RXFB_STRING     = 18;       { program is a "string file"?   }
    RXFB_TOKEN      = 19;       { tokenize the command line?    }
    RXFB_NONRET     = 20;       { a "no-return" message?        }

{ The flag form of the command modifiers                               }

    RXFF_NOIO       = $00010000;
    RXFF_RESULT     = $00020000;
    RXFF_STRING     = $00040000;
    RXFF_TOKEN      = $00080000;
    RXFF_NONRET     = $00100000;

    RXCODEMASK      = $FF000000;
    RXARGMASK       = $0000000F;

{ The RexxRsrc structure is used to manage global resources.  Each node
 * has a name string created as a RexxArg structure, and the total size
 * of the node is saved in the "rr_Size" field.  The REXX systems library
 * provides functions to allocate and release resource nodes.  If special
 * deletion operations are required, an offset and base can be provided in
 * "rr_Func" and "rr_Base", respectively.  This "autodelete" function will
 * be called with the base in register A6 and the node in A0.
 }

Type

    pRexxRsrc = ^tRexxRsrc;
    tRexxRsrc = record
    rr_Node     : tNode;
    rr_Func     : smallint;    { "auto-delete" offset          }
    rr_Base     : Pointer;  { "auto-delete" base            }
    rr_Size     : Longint;  { total size of node            }
    rr_Arg1     : Longint;  { available ...                 }
    rr_Arg2     : Longint;  { available ...                 }
    end;                { size: 32 bytes                }

Const

{ Resource node types                                                  }

    RRT_ANY     = 0;        { any node type ...             }
    RRT_LIB     = 1;        { a function library            }
    RRT_PORT        = 2;        { a public port                 }
    RRT_FILE        = 3;        { a file IoBuff                 }
    RRT_HOST        = 4;        { a function host               }
    RRT_CLIP        = 5;        { a Clip List node              }

{ The RexxTask structure holds the fields used by REXX to communicate with
 * external processes, including the client task.  It includes the global
 * data structure (and the base environment).  The structure is passed to
 * the newly-created task in its "wake-up" message.
 }

    GLOBALSZ        = 200;      { total size of GlobalData      }

Type

    pRexxTask = ^tRexxTask;
    tRexxTask = record
    rt_Global   : Array [0..GLOBALSZ-1] of Byte;
                    { global data structure         }
    rt_MsgPort  : tMsgPort;  { global message port           }
    rt_Flags    : Byte;     { task flag bits                }
    rt_SigBit   : Shortint;     { signal bit                    }

    rt_ClientID : Pointer;  { the client's task ID          }
    rt_MsgPkt   : Pointer;  { the packet being processed    }
    rt_TaskID   : Pointer;  { our task ID                   }
    rt_RexxPort : Pointer;  { the REXX public port          }

    rt_ErrTrap  : Pointer;  { Error trap address            }
    rt_StackPtr : Pointer;  { stack pointer for traps       }

    rt_Header1  : tList;     { Environment list              }
    rt_Header2  : tList;     { Memory freelist               }
    rt_Header3  : tList;     { Memory allocation list        }
    rt_Header4  : tList;     { Files list                    }
    rt_Header5  : tList;     { Message Ports List            }
    end;

Const

{ Definitions for RexxTask flag bits                                   }

    RTFB_TRACE      = 0;        { external trace flag           }
    RTFB_HALT       = 1;        { external halt flag            }
    RTFB_SUSP       = 2;        { suspend task?                 }
    RTFB_TCUSE      = 3;        { trace console in use?         }
    RTFB_WAIT       = 6;        { waiting for reply?            }
    RTFB_CLOSE      = 7;        { task completed?               }

{ Definitions for memory allocation constants                          }

    MEMQUANT        = 16;       { quantum of memory space       }
    MEMMASK     = $FFFFFFF0;    { mask for rounding the size    }

    MEMQUICK        = 1;        { EXEC flags: MEMF_PUBLIC       }
    MEMCLEAR        = $00010000;    { EXEC flags: MEMF_CLEAR        }

{ The SrcNode is a temporary structure used to hold values destined for
 * a segment array.  It is also used to maintain the memory freelist.
}

Type

    pSrcNode = ^tSrcNode;
    tSrcNode = record
    sn_Succ     : pSrcNode; { next node                     }
    sn_Pred     : pSrcNode; { previous node                 }
    sn_Ptr      : Pointer;  { pointer value                 }
    sn_Size     : Longint;  { size of object                }
    end;                { size: 16 bytes                }

{ === rexx/rexxio.h ====================================================
 *
 * Copyright (c) 1986, 1987 by William S. Hawes.  All Rights Reserved.
 *
 * ======================================================================
 * Header file for ARexx Input/Output related structures
}

Const

    RXBUFFSZ    = 204;          { buffer length                 }

{
 * The IoBuff is a resource node used to maintain the File List.  Nodes
 * are allocated and linked into the list whenever a file is opened.
}

Type

    pIoBuff = ^tIoBuff;
    tIoBuff = record
    iobNode     : tRexxRsrc; { structure for files/strings   }
    iobRpt      : Pointer;  { read/write pointer            }
    iobRct      : Longint;  { character count               }
    iobDFH      : Longint;  { DOS filehandle                }
    iobLock     : Longint;  { DOS lock                      }
    iobBct      : Longint;  { buffer length                 }
    iobArea     : Array [0..RXBUFFSZ-1] of Byte;
                    { buffer area                   }
    end;                { size: 256 bytes               }

Const

{ Access mode definitions                                              }

    RXIO_EXIST      = -1;       { an external filehandle        }
    RXIO_STRF       = 0;        { a "string file"               }
    RXIO_READ       = 1;        { read-only access              }
    RXIO_WRITE      = 2;        { write mode                    }
    RXIO_APPEND     = 3;        { append mode (existing file)   }

{
 * Offset anchors for SeekF()
}

    RXIO_BEGIN      = -1;       { relative to start             }
    RXIO_CURR       = 0;        { relative to current position  }
    RXIO_END        = 1;        { relative to end               }

{
 * A message port structure, maintained as a resource node.  The ReplyList
 * holds packets that have been received but haven't been replied.
}

Type

    pRexxMsgPort = ^tRexxMsgPort;
    tRexxMsgPort = record
    rmp_Node    : tRexxRsrc; { linkage node                  }
    rmp_Port    : tMsgPort;  { the message port              }
    rmp_ReplyList   : tList;     { messages awaiting reply       }
    end;

Const

{
 * DOS Device types
}

    DT_DEV  = 0;            { a device                      }
    DT_DIR  = 1;            { an ASSIGNed directory         }
    DT_VOL  = 2;            { a volume                      }

{
 * Private DOS packet types
}

    ACTION_STACK    = 2002;     { stack a line                  }
    ACTION_QUEUE    = 2003;     { queue a line                  }

{ === rexx/rxslib.h ===================================================
 *
 * Copyright (c) 1986, 1987, 1989 by William S. Hawes (All Rights Reserved)
 *
 * =====================================================================
 * The header file for the REXX Systems Library
}

{ Some macro definitions                                               }

Const

    RXSNAME    : PChar = 'rexxsyslib.library';
    RXSID      : PChar = 'rexxsyslib 1.06 (07 MAR 88)';
    RXSDIR     : PChar = 'REXX';
    RXSTNAME   : PChar = 'ARexx';

{ The REXX systems library structure.  This should be considered as    }
{ semi-private and read-only, except for documented exceptions.        }

Type

    pRxsLib = ^tRxsLib;
    tRxsLib = record
    rl_Node     : tLibrary;  { EXEC library node             }
    rl_Flags    : Byte;     { global flags                  }
    rl_pad      : Byte;
    rl_SysBase  : Pointer;  { EXEC library base             }
    rl_DOSBase  : Pointer;  { DOS library base              }
    rl_IeeeDPBase   : Pointer;  { IEEE DP math library base     }
    rl_SegList  : Longint;  { library seglist               }
    rl_NIL      : Longint;  { global NIL: filehandle        }
    rl_Chunk    : Longint;  { allocation quantum            }
    rl_MaxNest  : Longint;  { maximum expression nesting    }
    rl_NULL     : pNexxStr;   { static string: NULL           }
    rl_FALSE    : pNexxStr;   { static string: FALSE          }
    rl_TRUE     : pNexxStr;   { static string: TRUE           }
    rl_REXX     : pNexxStr;   { static string: REXX           }
    rl_COMMAND  : pNexxStr;   { static string: COMMAND        }
    rl_STDIN    : pNexxStr;   { static string: STDIN          }
    rl_STDOUT   : pNexxStr;   { static string: STDOUT         }
    rl_STDERR   : pNexxStr;   { static string: STDERR         }
    rl_Version  : STRPTR;   { version/configuration string  }

    rl_TaskName : STRPTR;   { name string for tasks         }
    rl_TaskPri  : Longint;  { starting priority             }
    rl_TaskSeg  : Longint;  { startup seglist               }
    rl_StackSize    : Longint;  { stack size                    }
    rl_RexxDir  : STRPTR;   { REXX directory                }
    rl_CTABLE   : STRPTR;   { character attribute table     }
    rl_Notice   : STRPTR;   { copyright notice              }

    rl_RexxPort : tMsgPort;  { REXX public port              }
    rl_ReadLock : Word;    { lock count                    }
    rl_TraceFH  : Longint;  { global trace console          }
    rl_TaskList : tList;     { REXX task list                }
    rl_NumTask  : smallint;    { task count                    }
    rl_LibList  : tList;     { Library List header           }
    rl_NumLib   : smallint;    { library count                 }
    rl_ClipList : tList;     { ClipList header               }
    rl_NumClip  : smallint;    { clip node count               }
    rl_MsgList  : tList;     { pending messages              }
    rl_NumMsg   : smallint;    { pending count                 }
    rl_PgmList  : tList;     { cached programs               }
    rl_NumPgm   : smallint;    { program count                 }

    rl_TraceCnt : Word;    { usage count for trace console }
    rl_avail    : smallint;
    end;

Const

{ Global flag bit definitions for RexxMaster                           }
    RLFB_TRACE  = RTFB_TRACE;       { interactive tracing?          }
    RLFB_HALT   = RTFB_HALT;        { halt execution?               }
    RLFB_SUSP   = RTFB_SUSP;        { suspend execution?            }
    RLFB_STOP   = 6;            { deny further invocations      }
    RLFB_CLOSE  = 7;            { close the master              }

    RLFMASK = 1 + 2 + 4;

{ Initialization constants                                             }

    RXSVERS = 34;           { main version                  }
    RXSREV  = 7;            { revision                      }
    RXSALLOC    = $800000;      { maximum allocation            }
    RXSCHUNK    = 1024;         { allocation quantum            }
    RXSNEST = 32;           { expression nesting limit      }
    RXSTPRI = 0;            { task priority                 }
    RXSSTACK    = 4096;         { stack size                    }
    RXSLISTH    = 5;            { number of list headers        }

{ Character attribute flag bits used in REXX.                          }

    CTB_SPACE   = 0;            { white space characters        }
    CTB_DIGIT   = 1;            { decimal digits 0-9            }
    CTB_ALPHA   = 2;            { alphabetic characters         }
    CTB_REXXSYM = 3;            { REXX symbol characters        }
    CTB_REXXOPR = 4;            { REXX operator characters      }
    CTB_REXXSPC = 5;            { REXX special symbols          }
    CTB_UPPER   = 6;            { UPPERCASE alphabetic          }
    CTB_LOWER   = 7;            { lowercase alphabetic          }

{ Attribute flags                                                      }

    CTF_SPACE   = 1;
    CTF_DIGIT   = 2;
    CTF_ALPHA   = 4;
    CTF_REXXSYM = 8;
    CTF_REXXOPR = 16;
    CTF_REXXSPC = 32;
    CTF_UPPER   = 64;
    CTF_LOWER   = 128;


VAR RexxSysBase : pLibrary;

const
    REXXSYSLIBNAME : PChar = 'rexxsyslib.library';

PROCEDURE ClearRexxMsg(msgptr : pRexxMsg; count : ULONG);
FUNCTION CreateArgstring(const argstring : pCHAR; length : ULONG) : pCHAR;
FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : pCHAR; host : pCHAR) : pRexxMsg;
PROCEDURE DeleteArgstring(argstring : pCHAR);
PROCEDURE DeleteRexxMsg(packet : pRexxMsg);
FUNCTION FillRexxMsg(msgptr : pRexxMsg; count : ULONG; mask : ULONG) : BOOLEAN;
FUNCTION IsRexxMsg(const msgptr : pRexxMsg) : BOOLEAN;
FUNCTION LengthArgstring(const argstring : pCHAR) : ULONG;
PROCEDURE LockRexxBase(resource : ULONG);
PROCEDURE UnlockRexxBase(resource : ULONG);

FUNCTION CreateArgstring(const argstring : string; length : ULONG) : pCHAR;
FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : string; host : pCHAR) : pRexxMsg;
FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : pCHAR; host : string) : pRexxMsg;
FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : string; host : string) : pRexxMsg;
PROCEDURE DeleteArgstring(argstring : string);
FUNCTION LengthArgstring(const argstring : string) : ULONG;

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitREXXSYSLIBLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    REXXSYSLIBIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
pastoc;


PROCEDURE ClearRexxMsg(msgptr : pRexxMsg; count : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L msgptr,A0
    MOVE.L  count,D0
    MOVEA.L RexxSysBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION CreateArgstring(const argstring : pCHAR; length : ULONG) : pCHAR;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L argstring,A0
    MOVE.L  length,D0
    MOVEA.L RexxSysBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : pCHAR; host : pCHAR) : pRexxMsg;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L port,A0
    MOVEA.L extension,A1
    MOVE.L  host,D0
    MOVEA.L RexxSysBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE DeleteArgstring(argstring : pCHAR);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L argstring,A0
    MOVEA.L RexxSysBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE DeleteRexxMsg(packet : pRexxMsg);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L packet,A0
    MOVEA.L RexxSysBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION FillRexxMsg(msgptr : pRexxMsg; count : ULONG; mask : ULONG) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L msgptr,A0
    MOVE.L  count,D0
    MOVE.L  mask,D1
    MOVEA.L RexxSysBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION IsRexxMsg(const msgptr : pRexxMsg) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L msgptr,A0
    MOVEA.L RexxSysBase,A6
    JSR -168(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION LengthArgstring(const argstring : pCHAR) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L argstring,A0
    MOVEA.L RexxSysBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE LockRexxBase(resource : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  resource,D0
    MOVEA.L RexxSysBase,A6
    JSR -450(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE UnlockRexxBase(resource : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  resource,D0
    MOVEA.L RexxSysBase,A6
    JSR -456(A6)
    MOVEA.L (A7)+,A6
  END;
END;


FUNCTION CreateArgstring(const argstring : string; length : ULONG) : pCHAR;
begin
       CreateArgstring := CreateArgstring(pas2c(argstring),length);
end;

FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : string; host : pCHAR) : pRexxMsg;
begin
       CreateRexxMsg := CreateRexxMsg(port,pas2c(extension),host);
end;

FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : pCHAR; host : string) : pRexxMsg;
begin
       CreateRexxMsg := CreateRexxMsg(port,extension,pas2c(host));
end;

FUNCTION CreateRexxMsg(const port : pMsgPort;const extension : string; host : string) : pRexxMsg;
begin
       CreateRexxMsg := CreateRexxMsg(port,pas2c(extension),pas2c(host));
end;

PROCEDURE DeleteArgstring(argstring : string);
begin
       DeleteArgstring(pas2c(argstring));
end;

FUNCTION LengthArgstring(const argstring : string) : ULONG;
begin
       LengthArgstring := LengthArgstring(pas2c(argstring));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of rexxsyslib.library}
  {$Info don't forget to use InitREXXSYSLIBLibrary in the beginning of your program}

var
    rexxsyslib_exit : Pointer;

procedure CloserexxsyslibLibrary;
begin
    ExitProc := rexxsyslib_exit;
    if RexxSysBase <> nil then begin
        CloseLibrary(RexxSysBase);
        RexxSysBase := nil;
    end;
end;

procedure InitREXXSYSLIBLibrary;
begin
    RexxSysBase := nil;
    RexxSysBase := OpenLibrary(REXXSYSLIBNAME,LIBVERSION);
    if RexxSysBase <> nil then begin
        rexxsyslib_exit := ExitProc;
        ExitProc := @CloserexxsyslibLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open rexxsyslib.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    REXXSYSLIBIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of rexxsyslib.library}

var
    rexxsyslib_exit : Pointer;

procedure CloserexxsyslibLibrary;
begin
    ExitProc := rexxsyslib_exit;
    if RexxSysBase <> nil then begin
        CloseLibrary(RexxSysBase);
        RexxSysBase := nil;
    end;
end;

begin
    RexxSysBase := nil;
    RexxSysBase := OpenLibrary(REXXSYSLIBNAME,LIBVERSION);
    if RexxSysBase <> nil then begin
        rexxsyslib_exit := ExitProc;
        ExitProc := @CloserexxsyslibLibrary;
        REXXSYSLIBIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open rexxsyslib.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    REXXSYSLIBIsCompiledHow := 3;
   {$Warning No autoopening of rexxsyslib.library compiled}
   {$Warning Make sure you open rexxsyslib.library yourself}
{$endif dont_use_openlib}


END. (* UNIT REXXSYSLIB *)



