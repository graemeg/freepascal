const
  clib = 'c';

type
  Pmsghdr = ^msghdr;
  msghdr = record
     msg_name : pointer;
     msg_namelen : socklen_t;
     msg_iov : piovec;
     msg_iovlen : size_t;
     msg_control : pointer;
     msg_controllen : socklen_t;
     msg_flags : cInt;
  end;

  Pcmsghdr = ^cmsghdr;
  cmsghdr = record
    cmsg_len   : socklen_t;
    cmsg_level : cInt;
    cmsg_type  : cInt;
  end;

  function sendmsg(__fd: cInt; __message: pmsghdr; __flags: cInt): ssize_t; cdecl; external clib name 'sendmsg';
  function recvmsg(__fd: cInt; __message: pmsghdr; __flags: cInt): ssize_t; cdecl; external clib name 'recvmsg';
  
  function CMSG_FIRSTHDR(mhdr: Pmsghdr): Pcmsghdr;
  function CMSG_NXTHDR(mhdr: Pmsghdr; cmsg: Pcmsghdr): Pcmsghdr;
  function CMSG_ALIGN(len: size_t): size_t;
  function CMSG_SPACE(len: size_t): size_t;
  function CMSG_LEN(len: size_t): size_t;
  function CMSG_DATA(cmsg : pcmsghdr) : Pbyte;


