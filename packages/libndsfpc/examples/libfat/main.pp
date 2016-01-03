program main;
{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  nds9, fat;

var
	i: integer;
	size: u32;
	text: string;
	handle: P_FILE;
begin
  consoleDemoInit();
  videoSetMode(MODE_FB0);
  vramSetBankA(VRAM_A_LCD);

  printf('fatInit()...');
  if fatInit(4, true) then
  begin
    printf(#9 + 'Success' + #10);

    handle := fopen('/test1.txt', 'r');
    if handle = nil then
    begin
      printf('Cannot open file' + #10);
    end else
    begin
      fseek(handle, 0, SEEK_END);  // Go to end of file
      size := ftell(handle);  // Get current position in file, because it is the end it will be the size
      fseek(handle, 0, SEEK_SET);  // Go to begining of file
      fread(@text, size, 1, handle); // Read all of file into memory
      printf(@text);
      fclose(handle); // Close file
    end;
  end else
    printf(#9 + 'Failure' + #10);

  for i := 0 to (256 * 192) - 1 do
    VRAM_A[i] := RGB15(31,0,0);

  while true do;


end.
