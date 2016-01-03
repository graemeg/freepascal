
{ *********************************************************************** }
{                                                                         }
{  elfresfix - Free Pascal Resource to ELF object compiler - fixup tool   }
{  Part of the Free Pascal and CrossFPC distributions                     }
{                                                                         }
{  Copyright (C) 2005 Simon Kissel                                        }
{                                                                         }
{  See the file COPYING.FPC, included in the FPC distribution,            }
{  for details about the copyright.                                       }
{                                                                         }
{  This program is distributed in the hope that it will be useful,        }
{  but WITHOUT ANY WARRANTY; without even the implied warranty of         }
{  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                   }
{                                                                         }
{ *********************************************************************** }

{
This tool will update the .fpc.resptrs section of an ELF executable to point
to the various resource sections in the file. This is done so that the FPC
RTL at runtime is able to get pointers to these sections.

This tool is automatically run on any fpc compiled ELF executable that
contains ELF resources.

fpcresfix builds with Delphi, Kylix and FPC.

Currently this only works on 32Bit targets, but support for 64Bit targets
is in the works. Support for big endian systems is completely missing,
though.
}
{$ifdef fpc}
{$mode objfpc}
{$endif}
{$h+}

unit elfresfix;

interface

uses
  SysUtils, Classes, elfbfd;
  
Type

  TLogEvent = Procedure(Const Msg : String) of object;
  
  { TElfResourceFixer }

  TElfResourceFixer = Class(TObject)
  private
    FFileName: String;
    FOnVerbose: TLogEvent;
    FVerbose: Boolean;
    Procedure DoVerbose(Msg : String);
  public
    Procedure FixFile(AFileName : String);
    Procedure DoFixStream(Stream : TStream); virtual; abstract;
    Property Verbose : Boolean read FVerbose write FVerbose;
    Property FileName : String Read FFileName;
    Property Onverbose : TLogEvent Read FOnVerbose Write FOnVerbose;
  end;
  
  { TElf32ResourceFixer }

  TElf32ResourceFixer = Class(TElfResourceFixer)
    Procedure DoFixStream(Stream : TStream); override;
  end;
  
  { TElf64ResourceFixer }

  TElf64ResourceFixer = Class(TElfResourceFixer)
    Procedure DoFixStream(Stream : TStream); override;
  end;

  EElfResFixError = Class(Exception);

Implementation

ResourceString
  SCheckingHeader           = 'Checking ELF Header... ';
  SReadingSectionHeaders    = 'Reading Section Headers...';
  SHeaderOK                 = 'ELF Header is OK';
  SCheckingHeaderTable      = 'Checking Section Header table...';
  SStrTabFound              = 'Found strtab...';
  SProcessingSection        = 'Processing section: ';
  SUpdatingResptrs          = 'Updating resptrs section...';
  SFileFixed                = 'File fixed successfully!';
  SNothingToFix             = 'There was nothing to fix in this file.';
  
  SErrUnsupportedHeaderSize = 'Unsupported Section Header size.';
  SErrInvalidELFHeader      = 'Not a valid linux ELF binary.';
  SErrResPtrsNotFound       = 'Unable to find resptrs section.';
  
Procedure DoError (Msg : String);

begin
  Raise EElfResFixError.Create(Msg);
end;

Procedure DoErrorFmt (Msg : String; Args : Array of const);

begin
  Raise EElfResFixError.CreateFmt(Msg,Args);
end;

{ TElfResourceFixer }

procedure TElfResourceFixer.DoVerbose(Msg: String);
begin
  If FVerbose and Assigned(FOnVerbose) then
    FOnVerbose(Msg);
end;

procedure TElfResourceFixer.FixFile(AFileName: String);

Var
  F : TStream;

begin
  FFileName:=AFileName;
  F:=TFileStream.Create(AFilename,fmOpenReadWrite or fmShareDenyWrite);
  Try
    DoFixStream(F);
  Finally
    F.Free;
  end;
end;

{ TElf32ResourceFixer }

procedure TElf32ResourceFixer.DoFixStream(Stream: TStream);
var
  ElfHeader:TElf32header;
  ResourceSectionTable: TElf32ResourceSectionTable;
  SectionHeaders: array of TElf32sechdr;

  i:integer;
  sn:string;
  SectionHeaderOffset:integer;
  fixed: boolean;
  strtab:string;
  SectionName: string;
  ResPtrsSection: integer;
  ResHashSection: integer;
  ResSymSection: integer;

  ResourceInfo: TELF32ResourceInfo;
  DataIndex, StringIndex: integer;
  SymString: string;

  procedure DoAlign(var value:integer; const a: integer);
  var i: integer;
  begin
    i:=(4 - (value MOD a)) MOD a;
    if (i>0) then inc(value,i);
  end;

begin
  Fixed:=False;
  Stream.Read(ElfHeader,sizeof(TElf32header));
  DoVerbose(SCheckingHeader);
  if (ElfHeader.magic0123<>$464C457F) then
    DoError(SErrInvalidELFheader);
  if ElfHeader.e_shentsize=sizeof(TElf32sechdr) then
    DoVerbose(SHeaderOK)
  else
    DoError(SErrUnSupportedHeaderSize);
  DoVerbose(SReadingSectionHeaders);

  setlength(SectionHeaders,ElfHeader.e_shnum);
  SectionHeaderOffset:=ElfHeader.e_shoff;
  Stream.Position:=SectionHeaderOffset;

  for i:=0 to ElfHeader.e_shnum-1 do
    begin
    Stream.Read(SectionHeaders[i],sizeof(TElf32sechdr));
    end;

  DoVerbose(SCheckingHeaderTable);

  // Get the section header strtab
  i:=ElfHeader.e_shstrndx;
  if SectionHeaders[i].sh_type=SHT_STRTAB then
    begin
    DoVerbose(SStrTabFound);
    // read the strtab
    Stream.Position:=SectionHeaders[i].sh_offset;
    setlength(strtab,SectionHeaders[i].sh_size);
    Stream.Read(strtab[1],SectionHeaders[i].sh_size);
  end
  else
  begin
    writeln('Error: Unable to find strtab.');
    halt(5);
  end;

  ResPtrsSection:=-1;
  ResHashSection:=-1;
  FillChar(ResourceSectionTable, sizeof(ResourceSectionTable), 0);
  ResourceSectionTable.version:=1;

  // Next cycle through all sections to gather pointers to all the resource
  // sections, and note the index of the resptrs section
  for i:=0 to ElfHeader.e_shnum-1 do
    begin
    SectionName:=copy(strtab,SectionHeaders[i].sh_name+1,32);
    SectionName:=copy(SectionName,1,pos(#0,SectionName)-1);
    DoVerbose(SProcessingSection+SectionName);
    sn:=Copy(SectionName,1,5);
    // FPC section ?
    if (sn='.fpc.') then
      begin
      sn:=SectionName;
      Delete(SN,1,5);
      if SN='resptrs' then
        begin
        ResPtrsSection:=i;
        end
      else if sn='ressym' then
        begin
        ResSymSection:=i;
        ResourceSectionTable.ressym.ptr:=SectionHeaders[i].sh_addr;
        ResourceSectionTable.ressym.size:=SectionHeaders[i].sh_size;
        end
      else if sn='reshash' then
        begin
        ResHashSection:=i;
        ResourceSectionTable.reshash.ptr:=SectionHeaders[i].sh_addr;
        ResourceSectionTable.reshash.size:=SectionHeaders[i].sh_size;
        ResourceSectionTable.resentries:=SectionHeaders[i].sh_size DIV sizeof(TELF32ResourceInfo);
        end
      else if sn='resdata' then
        begin
        ResourceSectionTable.resdata.ptr:=SectionHeaders[i].sh_addr;
        ResourceSectionTable.resdata.size:=SectionHeaders[i].sh_size;
        end
      else if sn='resspare' then
        begin
        ResourceSectionTable.resspare.ptr:=SectionHeaders[i].sh_addr;
        ResourceSectionTable.resspare.size:=SectionHeaders[i].sh_size;
        end
      else if SectionName='resstr' then
        begin
        ResourceSectionTable.resstr.ptr:=SectionHeaders[i].sh_addr;
        ResourceSectionTable.resstr.size:=SectionHeaders[i].sh_size;
        end;
      end
  end;

  // Ok, we now have pointers to all resource sections and also
  // know the number of resources.
  // Now update the resptrs table
  if (ResPtrsSection>-1) and (ResHashSection>-1) and (ResSymSection>-1) then
  begin
    Doverbose(SUpdatingResPtrs);
    Stream.Position:=SectionHeaders[ResPtrsSection].sh_offset;
    Stream.Write(ResourceSectionTable,sizeof(TELF32ResourceSectionTable));

    // LD might have merged the sections of several linked .or together
    // Therefore our data and stringtable offsets might be messed up and need to recalculated

    // First get the symbol string
    Stream.Position:=SectionHeaders[ResSymSection].sh_offset;
    setlength(SymString, SectionHeaders[ResSymSection].sh_size);
    Stream.Read(SymString[1], SectionHeaders[ResSymSection].sh_size);

    DataIndex:=0;
    StringIndex:=0;
    Stream.Position:=SectionHeaders[ResHashSection].sh_offset;

    for i:=0 to ResourceSectionTable.resentries-1 do
    begin
      Stream.Position:=SectionHeaders[ResHashSection].sh_offset+i*sizeof(TELF32ResourceInfo);
      Stream.Read(ResourceInfo, sizeof(TELF32ResourceInfo));
      ResourceInfo.ptr:=DataIndex;
      ResourceInfo.name:=StringIndex;

      // advance for next entry
      DataIndex:=DataIndex+ResourceInfo.size;
      DoAlign(DataIndex,4); // The data blocks are 32bit aligned

      // find end of current string
      while SymString[StringIndex+1]<>#0 do
        inc(StringIndex,1);
      inc(StringIndex,1);
      // this should be the start of the next string

      // write back the entry
      Stream.Position:=SectionHeaders[ResHashSection].sh_offset+i*sizeof(TELF32ResourceInfo);
      Stream.Write(ResourceInfo, sizeof(TELF32ResourceInfo));
    end;
    fixed:=true;
  end
  else
    DoError(SErrREsptrsNotFound);


  if fixed then
    DoVerbose(SFileFixed)
  else
    writeln(SNothingToFix);
end;

{ TElf64ResourceFixer }

procedure TElf64ResourceFixer.DoFixStream(Stream: TStream);
begin
  DoError('64-bit resources not yet supported');
end;

end.
