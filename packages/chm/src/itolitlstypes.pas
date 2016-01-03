{ Copyright (C) <2010> <Andrew Haines> itolitlstypes.pas

  This library is free software; you can redistribute it and/or modify it
  under the terms of the GNU Library General Public License as published by
  the Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE. See the GNU Library General Public License
  for more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; if not, write to the Free Software Foundation,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> graemeg/cpstrnew
=======
  Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
>>>>>>> origin/cpstrnew
}
{
  See the file COPYING.modifiedLGPL, included in this distribution,
  for details about the copyright.
}
unit ITOLITLSTypes;

{$mode objfpc}{$H+}
{$PACKRECORDS C}

interface

uses ChmBase;

type

  TSig = array[0..3] of char;
  TITOLITLSHeader = record
   Sig: Array [0..1] of TSig; //  ITLO/ITLS
   Version: DWord; // = 1
   HeaderSectionTableOffset: DWord;
   HeaderSectionEntryCount: DWord;
   PostHeaderTableSize: DWord;
   GUID: TGuid; // {0A9007C1-4076-11D3-8789-0000F8105754}
  end;

  TITOLITLSHeaderSectionEntry = record
    OffSet: QWord; // From Start of ITLO/ITLS
    Length: QWord;
  end;

  TChunkDirInfo = record
    TopAOLIChunkIndex: QWord; // -1 if none
    FirstAOLLChunkIndex,
    LastAOLLChunkIndex: QWord;
    Unknown0: QWord; // 0
    ChunkSize: DWord; // = $2000 if list $200 if Index
    QuickRefDensity: DWord; // = 2
    Unknown1: DWord; // = 0
    DirDepth: DWord; // 1 there is no index, 2 if there is one level of AOLI 3 if two index levels etc
    Unknown2: QWord; // 0
    DirEntryCount: QWord; // Number Of Directory Entries
  end;

  TITSFHeaderV4= record
    ITSFsig: array [0..3] of char;
    Version: LongWord;
    HeaderLength: LongWord;
    Unknown_1: LongWord;
    Section0Offset: QWord;
    TimeStamp: LongWord; //bigendian
    LanguageID: LongWord;
  end;

  TCAOLRec = record
    Sig: TSig; // CAOL
    Version: DWord; // 2
    CAOLSize: DWord; // includes ITSF section = $50
    CompilerID: array [0..1] of char; // = "HH"
    Unknown: Word; // 0
    Unknown1: DWord; // $43ED or 0
    DirChunkSize: DWord; // $2000
    DirIndexChunkSize: DWord; // $200
    Unknown2,       // $100000
    Unknown3: DWord;// $20000
    Unknown4,
    Unknown5,
    Unknown6: DWord; // = 0
    ITSFHeader: TITSFHeaderV4;
  end;

  TITOLITLSPostHeader = record
    Version: DWord; // 2
    CAOLOffset: DWord; // usually $98 (is from start of PostHeader)
    ListChunkInfo,
    IndexChunkInfo: TChunkDirInfo;
    Unknown3: DWord; // = $100000
    Unknown4: Dword; // =  $20000
    Unknown5: QWord; // 0

  end;

  THeaderSection0 = TITSPHeaderPrefix;

  TIFCMRec = record
    SIG: TSig; // = IFCM
    Version: DWord; // = 1
    ChunkSize: DWord; // = $2000
    UnKnown: DWord; // = $100000
    Unknown1: DWord; // = -1
    Unknown2: DWord; // = -1
    ChunkCount: Dword;//
    Unknown3: DWord; // = 0
  end;

  TAOLLChunkHeader = record
    Sig: TSig; // = AOLL
    QuickRefSize: DWord;
    ChunkIndex: QWord; // must be correct in the order written
    PrevChunkIndex: QWord;
    NextChunkIndex: QWord;
    FirstEntryIndex: QWord;
    Unknown0,       // = 1
    Unknown1: DWord;// = 0
    // entries
  end;

  TAOLIChunkHeader = record
   Sig: TSig; // = AOLI
   QuickRefSize: DWord;//    Length of quickref area at end of directory chunk
   ChunkIndex: QWord;//    Directory chunk number
   // entries
  end;

  const
    ITOLITLSGuid: TGuid = '{0A9007C1-4076-11D3-8789-0000F8105754}';

  type
  TLZXv3ControlData = record
    Sig: TSig;
    Version: DWord;
    ResetInterval: Dword;
    WindowSize: DWord;
    CacheSize: Dword;
    Unknown1,
    Unknown2: DWord; // 0
  end;

  TLZXv3ResetTable = record
    Version: Dword;
    EntryCount: DWord;
    EntrySize: DWord;
    EntryStart: DWord;
    UnCompressedSize,
    CompressedSize: QWord;
    BlockSize: QWord; // $8000
  end;


implementation

end.
