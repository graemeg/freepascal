{
The sample is give at
http://developer.apple.com/mac/library/samplecode/OpenCL_Hello_World_Example/index.html

===========================================================================
DESCRIPTION:

A simple "Hello World" compute example showing basic usage of OpenCL which
calculates the mathematical square (X[i] = pow(X[i],2)) for a buffer of
floating point values.

For simplicity, this example is intended to be run from the command line.
If run from within XCode, open the Run Log (Command-Shift-R) to see the
output.  Alternatively, run the applications from within a Terminal.app
session to launch from the command line.

===========================================================================
BUILD REQUIREMENTS:

Mac OS X v10.6 or later
  or
Windows with NVidia OpenCL SDK Installed

===========================================================================
RUNTIME REQUIREMENTS:

Mac OS X v10.6 or later

  To use the GPU as a compute device, use one of the following devices:
  - MacBook Pro w/NVidia GeForce 8600M
  - Mac Pro w/NVidia GeForce 8800GT
  
  If you don't have powerful GPU you can try to use CPU instead: 
  by changing  
    gpu := CL_DEVICE_TYPE_GPU; 
    to
    gpu := CL_DEVICE_TYPE_CPU; 

Windows with NVidia OpenCL SDK Installed and libOpenCLXX.dll available

CL_DEVICE_TYPE_CPU doesn't seem to work for windows

===========================================================================
}
program testcl;

{$mode objfpc}{$H+}

uses
  ctypes, cl;


// Use a static data size for simplicity

const
  DATA_SIZE = 1024;

// Simple compute kernel which computes the square of an input array
const
  KernelSource : PChar = 
  '__kernel void square(                   '#10+
  '   __global float* input,               '#10+
  '   __global float* output,              '#10+
  '   const unsigned int count)            '#10+
  '{                                       '#10+
  '   int i = get_global_id(0);            '#10+
  '   if(i < count)                        '#10+
  '       output[i] = input[i] * input[i]; '#10+
  '} '#0;


var
  err     : Integer; // error code returned from api calls
  data    : array [0..DATA_SIZE-1] of single; // original data set given to device
  results : array [0..DATA_SIZE-1] of single; // results returned from device
  correct : LongWord; // number of correct results returned

  global  : csize_t; // global domain size for our calculation
  local   : csize_t; // local domain size for our calculation

  device_id : cl_device_id;      // compute device id
  context   : cl_context;        // compute context
  commands  : cl_command_queue;  // compute command queue
  prog      : cl_program;        // compute program
  kernel    : cl_kernel;         // compute kernel

  input   : cl_mem; // device memory used for the input array
  output  : cl_mem; // device memory used for the output array

  i     : Integer;
  count : Integer;
  gpu   : cl_device_type;

  tmpd  : single;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  platformids : Pcl_platform_id;
  num_platforms : cl_uint;
  
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  // Fill our data set with random float values
  count := DATA_SIZE;
  for i:=0 to count - 1 do
    data[i]:= random;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  err:=clGetPlatformIDs(0,nil,@num_platforms);
  Writeln('clGetPlatformIDs ', num_platforms);
  if (err <> CL_SUCCESS) then
  begin
      writeln('Error: Cannot get number of platforms!');
      Halt(1);
  end;

  getmem(platformids,num_platforms*sizeof(cl_platform_id));

  err := clGetPlatformIDs(num_platforms, platformids, nil);

  if (err <> CL_SUCCESS) then begin
      Writeln('Error: Failed to platforms!');
      Halt($FF);
  end;

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  // Connect to a compute device
  // change CL_DEVICE_TYPE_CPU to CL_DEVICE_TYPE_GPU is you have powerful video (GeForce 8800/8600M or higher)
  gpu := CL_DEVICE_TYPE_GPU;

  device_id:=nil;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  err := clGetDeviceIDs(platformids[0], gpu, 1, @device_id, nil);
=======
  err := clGetDeviceIDs(nil, gpu, 1, @device_id, nil);
>>>>>>> graemeg/cpstrnew
=======
  err := clGetDeviceIDs(nil, gpu, 1, @device_id, nil);
>>>>>>> graemeg/cpstrnew
=======
  err := clGetDeviceIDs(nil, gpu, 1, @device_id, nil);
>>>>>>> graemeg/cpstrnew
=======
  err := clGetDeviceIDs(nil, gpu, 1, @device_id, nil);
>>>>>>> origin/cpstrnew
  writeln('clGetDeviceIDs ', err);
  if (err <> CL_SUCCESS) then begin
    Writeln('Error: Failed to create a device group!');
    Halt($FF);
  end;

  // Create a compute context
  context := clCreateContext(nil, 1, @device_id, nil, nil, err);
  writeln('clCreateContext ', err);
  if context=nil then begin
    Writeln('Error: Failed to create a compute context!');
    Halt($FF);
  end;

  // Create a command commands
  commands := clCreateCommandQueue(context, device_id, 0, err);
  writeln('clCreateCommandQueue ', err);
  if commands=nil then begin
    Writeln('Error: Failed to create a command commands!');
    Halt($FF);
  end;

  // Create the compute program from the source buffer
  prog:= clCreateProgramWithSource(context, 1, PPChar(@KernelSource), nil, err);
  writeln('clCreateProgramWithSource ', err);
  if prog=nil then begin
    writeln('Error: Failed to create compute program! ');
    Halt($FF);
  end;

  // Build the program executable
  err := clBuildProgram(prog, 0, nil, nil, nil, nil);
  writeln('clBuildProgram ', err);
  if (err <> CL_SUCCESS) then begin
    writeln('Error: Failed to build program executable!');
    Halt(1);
  end;

  // Create the compute kernel in the program we wish to run
  kernel := clCreateKernel(prog, 'square', err);
  writeln('clCreateKernel ', err);
  if (kernel=nil) or (err <> CL_SUCCESS) then begin
    writeln('Error: Failed to create compute kernel!');
    Halt(1);
  end;
  
  err := clGetKernelWorkGroupInfo(kernel, device_id,  CL_KERNEL_WORK_GROUP_SIZE, sizeof(local), @local, nil);
  writeln('clGetKernelWorkGroupInfo ', err);
  if (err<>CL_SUCCESS) then begin
    writeln('Error: Failed to retrieve kernel work group info!');
    Halt(1);
  end;
  
  
  // Create the input and output arrays in device memory for our calculation
  input := clCreateBuffer(context,  CL_MEM_READ_ONLY,  sizeof(single) * count, nil, err);
  writeln('clCreateBuffer ', err);
  output := clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeof(single) * count, nil, err);
  writeln('clCreateBuffer ', err);
  if (input=nil) or (output=nil) then begin
    writeln('Error: Failed to allocate device memory!');
    Halt(1);
  end;

  // Write our data set into the input array in device memory
  err := clEnqueueWriteBuffer(commands, input, CL_TRUE, 0, sizeof(single) * count, @data, 0, nil, nil);
  writeln('clEnqueueWriteBuffer ', err);
  if (err <> CL_SUCCESS) then begin
    writeln('Error: Failed to write to source array!');
    Halt(1);
  end;

  // Set the arguments to our compute kernel
  err := 0;
  err := clSetKernelArg(kernel, 0, sizeof(cl_mem), @input);
  writeln('clSetKernelArg ', err);
  err := err or clSetKernelArg(kernel, 1, sizeof(cl_mem), @output);
  writeln('clSetKernelArg ', err);
  err := err or clSetKernelArg(kernel, 2, sizeof(longword), @count);
  writeln('clSetKernelArg ', err);
  if (err <> CL_SUCCESS) then begin
    writeln('Error: Failed to set kernel arguments! ');
    Halt(1);
  end;

  // Get the maximum work group size for executing the kernel on the device
  err := clGetKernelWorkGroupInfo(kernel, device_id,  CL_KERNEL_WORK_GROUP_SIZE, sizeof(local), @local, nil);
  writeln('clGetKernelWorkGroupInfo ', err);
  if (err<>CL_SUCCESS) then begin
    writeln('Error: Failed to retrieve kernel work group info!');
    Halt(1);
  end;
  
  // Execute the kernel over the entire range of our 1d input data set
  // using the maximum number of work group items for this device
  global := count;
  err := clEnqueueNDRangeKernel(commands, kernel, 1, nil, @global, @local, 0, nil, nil);
  writeln('clEnqueueNDRangeKernel ',err);
  if (err<>0) then begin
    writeln('Error: Failed to execute kernel!');
    Halt($FF);
  end;

  // Wait for the command commands to get serviced before reading back results
  err:=clFinish(commands);
  writeln('clFinish ',err);

  // Read back the results from the device to verify the output
  err := clEnqueueReadBuffer( commands, output, CL_TRUE, 0, sizeof(single) * count, @results, 0, nil, nil);
  writeln('clEnqueueReadBuffer ',err);
  if (err <> CL_SUCCESS) then begin
    writeln('Error: Failed to read output array! ', err);
    Halt(1);
  end;

  // Validate our results
  correct := 0;
  for i:= 0 to count - 1 do begin
    // FPU warning:
    //
    // the following check (as in original C sample)
    // if results[i] = data[i] * data[i] then
    //
    // return the incorrect result (FP accuracy?),
    // must store the result to single type variable first,
    // and then compare:
    tmpd:=data[i] * data[i];
    if results[i] = tmpd then inc(correct);
  end;

  // Print a brief summary detailing the results
  writeln('Computed ', correct, '/', count,' correct values!');

  // Shutdown and cleanup
  clReleaseMemObject(input);
  clReleaseMemObject(output);
  clReleaseProgram(prog);
  clReleaseKernel(kernel);
  clReleaseCommandQueue(commands);
  clReleaseContext(context);
end.

