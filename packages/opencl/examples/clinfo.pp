{$mode objfpc}
uses
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  ctypes, cl;
=======
  cl;
>>>>>>> graemeg/cpstrnew
=======
  cl;
>>>>>>> graemeg/cpstrnew
=======
  cl;
>>>>>>> graemeg/cpstrnew
=======
  cl;
>>>>>>> origin/cpstrnew

const
  device_str_info : array[1..5] of record id : dword; name : pchar end =
   ((id: CL_DEVICE_NAME; name : 'CL_DEVICE_NAME'),
    (id: CL_DEVICE_VENDOR; name : 'CL_DEVICE_VENDOR'),
    (id: CL_DEVICE_VERSION; name : 'CL_DEVICE_VERSION'),
    (id: CL_DEVICE_PROFILE; name : 'CL_DEVICE_PROFILE'),
    (id: CL_DEVICE_EXTENSIONS; name : 'CL_DEVICE_EXTENSIONS'));



var
  err     : Integer; // error code returned from api calls
  platformids : pcl_platform_id;
  platforms : cl_uint;
  devices : cl_uint;
  deviceids : pcl_device_id;
  i,j,k : Integer;
  buf : array[0..99999] of char;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  bufwritten : csize_t;
=======
  bufwritten : LongWord;
>>>>>>> graemeg/cpstrnew
=======
  bufwritten : LongWord;
>>>>>>> graemeg/cpstrnew
=======
  bufwritten : LongWord;
>>>>>>> graemeg/cpstrnew
=======
  bufwritten : LongWord;
>>>>>>> origin/cpstrnew


begin
  err:=clGetPlatformIDs(0,nil,@platforms);
  if (err <> CL_SUCCESS) then
    begin
      writeln('Error: Cannot get number of platforms!');
      Halt(1);
    end;
  getmem(platformids,platforms*sizeof(cl_platform_id));
  err:=clGetPlatformIDs(platforms,platformids,nil);
  if (err <> CL_SUCCESS) then
    begin
      writeln('Error: Cannot get platforms!');
      Halt(1);
    end;
  writeln(platforms,' platform(s) found');
  for i:=0 to platforms-1 do
    begin
      writeln('Platform info ',i);
      err:=clGetPlatformInfo(platformids[i],CL_PLATFORM_PROFILE,sizeof(buf),@buf,bufwritten);
      writeln('PROFILE: ',buf);
      err:=clGetPlatformInfo(platformids[i],CL_PLATFORM_VERSION,sizeof(buf),@buf,bufwritten);
      writeln('VERSION: ',buf);
      err:=clGetPlatformInfo(platformids[i],CL_PLATFORM_NAME,sizeof(buf),@buf,bufwritten);
      writeln('NAME: ',buf);
      err:=clGetPlatformInfo(platformids[i],CL_PLATFORM_VENDOR,sizeof(buf),@buf,bufwritten);
      writeln('VENDOR: ',buf);
      err:=clGetPlatformInfo(platformids[i],CL_PLATFORM_EXTENSIONS,sizeof(buf),@buf,bufwritten);
      writeln('EXTENSIONS: ',buf);

      err:=clGetDeviceIDs(platformids[i],CL_DEVICE_TYPE_ALL,0,nil,@devices);
      if (err <> CL_SUCCESS) then
        begin
          writeln('Error: Cannot get number of devices!');
          Halt(1);
        end;
      writeln(devices,' device(s) found');
      getmem(deviceids,devices*sizeof(cl_device_id));
      err:=clGetDeviceIDs(platformids[i],CL_DEVICE_TYPE_ALL,devices,deviceids,nil);
      for j:=0 to devices-1 do
        begin
          writeln('Device info ',j);
          for k:=low(device_str_info) to high(device_str_info) do
            begin
              err:=clGetDeviceInfo(deviceids[j],device_str_info[k].id,sizeof(buf),@buf,bufwritten);
              writeln(device_str_info[k].name,': ',buf);
            end;
          err:=clGetDeviceInfo(deviceids[j],CL_DEVICE_MAX_COMPUTE_UNITS,sizeof(buf),@buf,bufwritten);
          writeln('CL_DEVICE_MAX_COMPUTE_UNITS: ',pdword(@buf)^);
          err:=clGetDeviceInfo(deviceids[j],CL_DEVICE_IMAGE3D_MAX_WIDTH,sizeof(buf),@buf,bufwritten);
          writeln('CL_DEVICE_IMAGE3D_MAX_WIDTH: ',pdword(@buf)^);
          err:=clGetDeviceInfo(deviceids[j],CL_DEVICE_IMAGE3D_MAX_HEIGHT,sizeof(buf),@buf,bufwritten);
          writeln('CL_DEVICE_IMAGE3D_MAX_HEIGHT: ',pdword(@buf)^);
          err:=clGetDeviceInfo(deviceids[j],CL_DEVICE_GLOBAL_MEM_SIZE,sizeof(buf),@buf,bufwritten);
          writeln('CL_DEVICE_GLOBAL_MEM_SIZE: ',pdword(@buf)^);
        end;
    end;
  freemem(platformids);
end.
