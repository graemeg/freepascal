{$mode iso}
program mytest;

procedure my_test1;
type byte_file = file of byte;
   
var test_file : byte_file;
   test_text  : text;
   loc	      : integer;
   len	      : integer;
   my_bits    : byte;
   pos	      : int64;
begin
   assign(test_text, 'tw34848.data');
   rewrite(test_text);
   write(test_text,'0123456789'#10);
   close(test_text);
   loc := 9;
   assign(test_file, 'tw34848.data');
   reset(test_file);
   len := filesize(test_file);
   writeln('File size: ', len);
   seek(test_file, loc);
   if EOF(test_file) then
      writeln('EOF reached');
   pos := filepos(test_file);
   if pos<>9 then
     halt(1);
   writeln('File position: ', pos);
   read(test_file, my_bits);
   writeln(my_bits);
   if my_bits<>57 then
     halt(1);
   read(test_file, my_bits);
   writeln(my_bits);
   if my_bits<>10 then
     halt(1);
   close(test_file);
end;
begin
   my_test1;
   writeln('ok');
end.
