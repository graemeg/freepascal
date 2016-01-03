program nitrodir;

{$mode objfpc}

uses
  ctypes, nds9, filesystem;

procedure dirlist(path: pchar);
var
  MyDir: PDIR;
  dnbuf: pchar;
  pent: pdirent;  
  statbuf: PStat;        
begin
  GetMem(MyDir, sizeof(PDIR));
  GetMem(pent, sizeof(dirent));
  GetMem(statbuf, sizeof(PStat));
  MyDir := opendir(path);

  if (MyDir <> nil) then
  begin
    while true do
    begin
      pent := readdir(MyDir);
      if pent = nil then 
        exit;
  
      if (strcmp('.', pent^.d_name) <> 0) and (strcmp('..', pent^.d_name) <> 0) then
      begin
        dnbuf := malloc(strlen(pent^.d_name) + strlen(path) + 2);
        if (strcmp('/',path) = 0) then 
          sprintf(dnbuf, '%s/%s', '', pent^.d_name)
        else
          sprintf(dnbuf, '%s/%s', path, pent^.d_name);
        _stat(dnbuf, statbuf^);
      
        if (S_ISDIR(statbuf^.st_mode)) then
        begin
          printf('%s <DIR>'#10, dnbuf);
          dirlist(dnbuf);
        end else 
        begin
          printf('%s (%d)'#10, dnbuf, statbuf^.st_size);
        end;
        free(dnbuf);
        free(statbuf);
      end;
    end;

    closedir(MyDir);
  end else 
  begin
    printf('opendir() failure.'#10);
  end;

end;

var
  inf: P_File;
  len: cint;
  entireFile: pcchar;
begin	
  // Initialise the console, required for printf
  consoleDemoInit();
   
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  if nitroFSInit(nil) then
=======
  if (nitroFSInit()) then
>>>>>>> graemeg/cpstrnew
=======
  if (nitroFSInit()) then
>>>>>>> graemeg/cpstrnew
=======
  if (nitroFSInit()) then
>>>>>>> graemeg/cpstrnew
=======
  if (nitroFSInit()) then
>>>>>>> origin/cpstrnew
  begin
    dirlist('/');
    begin
			// now, try reading a file to make sure things are working OK.
			inf := fopen('file1.txt','rb');
			if inf <> nil then
			begin
				fseek(inf, 0, SEEK_END);
				len := ftell(inf);
				fseek(inf, 0, SEEK_SET);

				iprintf(#10'the following %d bytes message'#10'from file1.txt is'#10'brought to you by fread:'#10, len);
				begin
					entireFile := pcchar(malloc(len+1));
					entireFile[len] := 0;
					if (fread(entireFile, 1, len, inf) <> len) then
						iprintf('savage error reading the bytes from the file!'#10)
					else
						iprintf('%s'#10'-done-'#10, entireFile);
					free(entireFile);
				end;

				fclose(inf);
			end;
		end;

		iprintf('here is the dirlist once more:'#10);
    dirlist('/');
  end else 
    iprintf('nitroFSInit failure: terminating'#10);

  while true do
    swiWaitForVBlank();
end.