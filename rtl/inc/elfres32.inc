
const
  fpcres2elf_version=1;

type
  TFPCResourceSectionInfo = packed record
    ptr: pointer;     // This always contains the absolute memory address of the section at runtime
    size: longint;    // The size of the section in bytes
  end;
  PTFPCResourceSectionInfo = ^TFPCResourceSectionInfo;

  TFPCResourceSectionTable = packed record
    version: longint;
    resentries: longint;
    ressym: TFPCResourceSectionInfo;
    reshash: TFPCResourceSectionInfo;
    resdata: TFPCResourceSectionInfo;
    resspare: TFPCResourceSectionInfo;
    resstr: TFPCResourceSectionInfo;
  end;
  PFPCResourceSectionTable = ^TFPCResourceSectionTable;

  TFPCResourceInfo = packed record
    reshash: longint;   // always 32bit, contains an ELF hash of the resource entries name
    restype: longint;   // always 32bit, contains the resource type ID compatible with Windows RES IDs
    ptr:     pointer;   // This contains the offset to the resource inside the resdata
                        // section.
    name:    pChar;     // The byte offset to the the resource name inside the ressym section.
    size:    longint;   // The size of the resource entry - 32/64 Bit, depending on platform
  end;
  PFPCResourceInfo = ^TFPCResourceInfo;

Var
{$ifdef FPC_HAS_RESOURCES}
  FPCResourceSectionLocation : pFPCResourceSectionTable; external name 'FPC_RESLOCATION';
{$else}
  FPCResourceSectionLocation : pFPCResourceSectionTable = Nil;
{$endif}

const
  LCase: set of char = ['a'..'z'];

function HashELFUppercase(S: PChar) : longint;
{Note: this hash function is described in "Practical Algorithms For
       Programmers" by Andrew Binstock and John Rex, Addison Wesley,
       with modifications in Dr Dobbs Journal, April 1996}
var
  G: longint;
  C: Char;
begin
  Result := 0;
  while S^ <> #0 do begin
    C := S^;
    if C in LCase then Dec(ord(C), 32);
    Result := (Result shl 4) + ord(C);
    Inc(S);
    G := Result and $F0000000;
    if (G <> 0) then
      Result := Result xor (G shr 24);
    Result := Result and (not G);
  end;
end;

Function HINSTANCE : HMODULE;

begin
  Result:=0;
end;

function _StrIComp(S1, S2: PChar): LongInt;
var
  C1, C2: Char;
begin
  Result := 0;
  repeat
    C1 := S1^;
    C2 := S2^;
    Result := ord(C1) - ord(C2);
    if Result <> 0 then
    begin
      if C1 in LCase then Dec(ord(C1), 32);
      if C2 in LCase then Dec(ord(C2), 32);
      Result := ord(C1) - ord(C2);
    end;
    Inc(S1);
    Inc(S2);
  until (Result <> 0) or ((S1^ = #0) or (S2^ = #0));
end;


function FindResource(ModuleHandle: HMODULE; ResourceName: PChar; ResourceType: PChar): TResourceHandle;
var
  i:longint;
  searchhash:longint;
  ResEntry: PFPCResourceInfo;
  pResName: PChar;
  tmp: array[0..7] of char;
begin
  Result:=0;
  if (ResourceName=nil) or (FPCResourceSectionLocation = nil) then
    Exit;
    
  { This is a temporary fix to stay compatible with fpcres
    which currently converts all string types to RT_RCDATA. }
  if ResourceType > PChar($FFFF) then
    ResourceType := PChar(10);
    
  { support numeric resource IDs }
  if ResourceName <= PChar($FFFF) then
  begin
    { convert number to string inline, this should be faster than messing with strings }
    i := LongInt(ResourceName);
    ResourceName := @tmp[7];
    ResourceName^ := #0;
    Dec(ResourceName);
    repeat
      ResourceName^ := Char((i mod 10) + ord('0'));
      Dec(ResourceName);
      i := i div 10;
    until i = 0;
    ResourceName^ := '#';
  end;
  { resources aren't case sensitive }
  searchhash := HashELFUppercase(ResourceName);
  ResEntry := FPCResourceSectionLocation^.reshash.ptr;
  for i:=0 to FPCResourceSectionLocation^.resentries-1 do
    with ResEntry[I] do
    begin
      if (PChar(ResType) = ResourceType) and (reshash = searchhash) then
      begin
        pResName := PChar(FPCResourceSectionLocation^.ressym.ptr);
        Inc(pResName, PtrUInt(Name));
        if _StrIComp(pResName, ResourceName) = 0 then
        begin
          result:=i+1;
          break;
        end;
      end;
    end;
end;

function LoadResource(ModuleHandle: HMODULE; ResHandle: TResourceHandle): HGLOBAL;
var
  ResEntry: PFPCResourceInfo;
begin
  if FPCResourceSectionLocation = nil then
    Exit;
  if (ResHandle>0) and (LongInt(ResHandle)-1<=FPCResourceSectionLocation^.resentries) then
  begin
    ResEntry := FPCResourceSectionLocation^.reshash.ptr;
    result := HGLOBAL(PtrUInt(FPCResourceSectionLocation^.resdata.ptr) + PtrUInt(ResEntry[LongInt(ResHandle)-1].ptr));
  end
  else
    result:=0;
end;

function SizeofResource(ModuleHandle: HMODULE; ResHandle: TResourceHandle): Integer;
var
  ResEntry: PFPCResourceInfo;
begin
  if FPCResourceSectionLocation = nil then
    Exit;
  if (ResHandle>0) and (LongInt(ResHandle)-1<=FPCResourceSectionLocation^.resentries) then
  begin
    ResEntry := FPCResourceSectionLocation^.reshash.ptr;
    result := ResEntry[LongInt(ResHandle)-1].size;
  end
  else
    result:=0;
end;

function LockResource(ResData: HGLOBAL): Pointer;
begin
  result:=Pointer(ResData);
end;

function UnlockResource(ResData: HGLOBAL): LongBool;
begin
  result:=False;
end;

function FreeResource(ResData: HGLOBAL): LongBool;
begin
  result:=True;
end;
