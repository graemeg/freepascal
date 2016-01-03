{$ifdef Delphi}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
const fpcmakeini : array[0..242] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..242,1..240] of char=(
=======
const fpcmakeini : array[0..229] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..229,1..240] of char=(
>>>>>>> graemeg/cpstrnew
=======
const fpcmakeini : array[0..229] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..229,1..240] of char=(
>>>>>>> graemeg/cpstrnew
=======
const fpcmakeini : array[0..229] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..229,1..240] of char=(
>>>>>>> graemeg/cpstrnew
=======
const fpcmakeini : array[0..229] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..229,1..240] of char=(
>>>>>>> origin/cpstrnew
=======
const fpcmakeini : array[0..225] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..225,1..240] of char=(
>>>>>>> graemeg/fixes_2_2
=======
const fpcmakeini : array[0..225] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..225,1..240] of char=(
>>>>>>> origin/fixes_2_2
{$endif Delphi}
  ';'#013#010+
  '; Templates used by fpcmake to create a Makefile from Makefile.fpc'#013+
  #010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  '[defines]'#010+
  '#####################################################################'#010+
  '# Misc defines to be used by anyone'#010+
  '####################################################','################'+
  '#'#010+
  #010+
  '# OS categories'#010+
<<<<<<< HEAD
  'BSDs = freebsd netbsd openbsd darwin dragonfly'#010+
  'UNIXs = linux $(BSDs) solaris qnx haiku aix'#010+
  'LIMIT83fs = go32v2 os2 emx watcom msdos win16'#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#010+
  #010+
  '#Empty target for rules th','at always should run. Needed if'#010+
=======
  'BSDs = freebsd netbsd openbsd darwin'#010+
  'UNIXs = linux $(BSDs) solaris qnx haiku'#010+
  'LIMIT83fs = go32v2 os2 emx watcom'#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#010+
  #010+
  '#Empty target for rules that always should run. Need','ed if'#010+
>>>>>>> origin/fixes_2.4
  '#the target is non-phoney, and there is non-phony prereqisites.'#010+
  '#Then add FORCE as an prerequisite'#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#010+
  'FORCE:'#010+
  '.PHONY: FORCE'#010+
  #010+
  '[osdetect]'#010+
<<<<<<< HEAD
  '##########','##########################################################'+
  '#'#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#010+
  '# define inUnix when running under Unix like environment'#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,C','ygwin)'#010+
  '# define inWinNT when running under WinNT'#010+
=======
  '####################################','################################'+
  '#'#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#010+
  '# define inUnix when running under Unix like environment'#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,Cygwin)'#010+
  '# define inWinNT wh','en running under WinNT'#010+
>>>>>>> origin/fixes_2.4
  '# define inOS2 when running under OS/2'#010+
  '# define inCygwin when running under Cygwin32'#010+
  '#####################################################################'#010+
  #010+
<<<<<<< HEAD
  '# We need only / in the path also r','emove the current dir,'#010+
  '# also remove trailing /'#039's'#010+
=======
  '# We need only / in the path also remove the current dir,'#010+
  '# a','lso remove trailing /'#039's'#010+
>>>>>>> origin/fixes_2.4
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#010+
  #010+
  '# Detect unix'#010+
  '# Darwin is handled specially'#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#010+
  'inUnix=1 #darwin'#010+
  'SEARCHPATH:=$(filter-out .,$(subst ',':, ,$(PATH)))'#010+
  'else'#010+
<<<<<<< HEAD
  '# Determine if we'#039've a unix searchpath by looking for a ;'#010+
=======
  '# Deter','mine if we'#039've a unix searchpath by looking for a ;'#010+
>>>>>>> origin/fixes_2.4
  '# that normally doesn'#039't exists in the unix PATH var.'#010+
  'ifeq ($(findstring ;,$(PATH)),)'#010+
  'inUnix=1'#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#010+
  'else'#010+
  'SEARCHPATH:=$(s','ubst ;, ,$(PATH))'#010+
  'endif'#010+
  'en','dif'#010+
  #010+
  '# Add path were make is located'#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#010+
  #010+
  '# Search for PWD'#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#010+
  'ifeq ($(PWD),)'#010+
<<<<<<< HEAD
  'PWD:=$(strip $(wildcard $','(addsuffix /pwd,$(SEARCHPATH))))'#010+
=======
  'PWD:=$(strip $(wildcard $(addsuffix /pwd,$(SEARCHPA','TH))))'#010+
>>>>>>> origin/fixes_2.4
  'ifeq ($(PWD),)'#010+
  '$(error You need the GNU utils package to use this Makefile)'#010+
  'else'#010+
  'PWD:=$(firstword $(PWD))'#010+
  'SRCEXEEXT='#010+
  'endif'#010+
  'else'#010+
  'PWD:=$(firstword $(PWD))'#010+
  'SRCEXEEXT=.exe'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Detect NT - NT sets OS to Wind','ows_NT'#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#010+
=======
  '# Detect NT - NT sets OS to Windows_NT'#010+
  '# Detect OS/2 - OS/','2 has OS2_SHELL defined'#010+
>>>>>>> origin/fixes_2.4
  'ifndef inUnix'#010+
  'ifeq ($(OS),Windows_NT)'#010+
  'inWinNT=1'#010+
  'else'#010+
  'ifdef OS2_SHELL'#010+
  'inOS2=1'#010+
  'endif'#010+
  'endif'#010+
  'else'#010+
  'ifneq ($(findstring cygdrive,$(PATH)),)'#010+
  'inCygWin=1'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# The extension of batch files /',' scripts'#010+
  'ifdef inUnix'#010+
<<<<<<< HEAD
  'SRCBATCHEXT=.sh'#010+
=======
  'SRCB','ATCHEXT=.sh'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'ifdef inOS2'#010+
  'SRCBATCHEXT=.cmd'#010+
  'else'#010+
  'SRCBATCHEXT=.bat'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'ifdef COMSPEC'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(OSNeedsComspecToRunBatch)),)'#010+
  'ifndef RUNBATCH'#010+
  'RUNBATCH=$(COMSPEC) /C'#010+
  'endif'#010+
  'endif'#010+
  'endi','f'#010+
  #010+
<<<<<<< HEAD
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#010+
=======
  '# Path Separator, the s','ubst trick is necessary for the \ that can'#039+
  't exists'#010+
>>>>>>> origin/fixes_2.4
  '# at the end of a line'#010+
  'ifdef inUnix'#010+
  'PATHSEP=/'#010+
  'else'#010+
  'PATHSEP:=$(subst /,\,/)'#010+
  '# cygwin bash or sh can not handle backslashs'#010+
  'ifdef inCygWin'#010+
  'PATHSEP=/'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# B','ase dir'#010+
  'ifdef PWD'#010+
<<<<<<< HEAD
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#010+
=======
  'BASEDIR:','=$(subst \,/,$(shell $(PWD)))'#010+
>>>>>>> origin/fixes_2.4
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#010+
  'ifdef inCygWin'#010+
  'ifneq ($(findstring /cygdrive/,$(BASEDIR)),)'#010+
  'BASENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#010+
<<<<<<< HEAD
  'BASEDRIVE:=$(firstword ','$(subst /, ,$(BASENODIR)))'#010+
=======
  'BASEDRIVE:=$(firstword $(subst /, ,$(BASENODIR)))',#010+
>>>>>>> origin/fixes_2.4
  'BASEDIR:=$(subst /cygdrive/$(BASEDRIVE)/,$(BASEDRIVE):/,$(BASEDIR))'#010+
  'endif'#010+
  'endif'#010+
  'else'#010+
  'BASEDIR=.'#010+
  'endif'#010+
  #010+
  '# Echo is an internal command under OS/2 (and others), so it'#039's alw'+
  'ays found'#010+
  'ifdef inOS2'#010+
  'ifndef ECHO'#010+
<<<<<<< HEAD
  'ECHO:=$(str','ip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH'+
=======
  'ECHO:=$(strip $(wildcard $(addsuffix ','/gecho$(SRCEXEEXT),$(SEARCHPATH'+
>>>>>>> origin/fixes_2.4
  '))))'#010+
  'ifeq ($(ECHO),)'#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /echo$(SRCEXEEXT),$(SEARCHPATH)))'+
  ')'#010+
  'ifeq ($(ECHO),)'#010+
  'ECHO=echo'#010+
  'else'#010+
  'ECHO:=$(firstword $(ECHO))'#010+
  'endif'#010+
  'else'#010+
  'ECHO:=$(firstword $(E','CHO))'#010+
  'endif'#010+
  'endif'#010+
<<<<<<< HEAD
  'export ECHO'#010+
=======
  'export E','CHO'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '[fpcdetect]'#010+
  '#####################################################################'#010+
  '# FPC Binary and Version Detection'#010+
  '#####################################################################'#010+
  #010+
<<<<<<< HEAD
  '# Compatibility',' with old makefiles'#010+
  'ifndef FPC'#010+
=======
  '# Compatibility with old makefiles'#010+
  'ifndef',' FPC'#010+
>>>>>>> origin/fixes_2.4
  'ifdef PP'#010+
  'FPC=$(PP)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Try to detect the ppcXXX file to use by using "fpc -PB" option'#010+
  '# to query for the default ppcXXX the fpc executable tries. When'#010+
<<<<<<< HEAD
  '# fpc is not found use ppc386 by default. Also ','when fpc -PB gives'#010+
  '# an error ppc386 will be used.'#010+
=======
  '# fpc is not found use ppc386 by default. Also when fpc -PB gives'#010+
  '# an er','ror ppc386 will be used.'#010+
>>>>>>> origin/fixes_2.4
  'ifndef FPC'#010+
  '# check if fpc exists'#010+
  'FPCPROG:=$(strip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#010+
  'ifneq ($(FPCPROG),)'#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#010+
  'ifneq ($(CPU_TARGET),)'#010+
<<<<<<< HEAD
  'FPC:=$','(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#010+
=======
  'FPC:=$(shell $(FPCPROG) -P$(CPU_','TARGET) -PB)'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#010+
  'endif'#010+
  '# Older fpc executables didn'#039't support it and return'#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#010+
  'ifneq ($(findstring Error,$(FPC)),)'#010+
<<<<<<< HEAD
  'override ','FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRC'+
=======
  'override FPC=$(firstword $(strip $(','wildcard $(addsuffix /ppc386$(SRC'+
>>>>>>> origin/fixes_2.4
  'EXEEXT),$(SEARCHPATH)))))'#010+
  'else'#010+
  '# if the cross compiler is not found, fall back to fpc'#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#010+
  'FPC:=$(firstword $(FPCPROG))'#010+
  'endif'#010+
  'endif'#010+
  'else'#010+
<<<<<<< HEAD
  '# fpc binary ','not found, fallback to ppc386'#010+
=======
  '# fpc binary not found, fallback to ppc','386'#010+
>>>>>>> origin/fixes_2.4
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Get a clean executable name'#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#010+
  'override FPC:=$(subst \,/,$(F','PC))$(SRCEXEEXT)'#010+
  #010+
<<<<<<< HEAD
  '# Try to find the binary direct first, otherwise in the path,'#010+
=======
  '# Try to',' find the binary direct first, otherwise in the path,'#010+
>>>>>>> origin/fixes_2.4
  '# if not found give an error'#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#010+
  'ifeq ($(FOUNDFPC),)'#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#010+
<<<<<<< HEAD
  'ifeq ($(','FOUNDFPC),)'#010+
  '$(error Compiler $(FPC) not found)'#010+
=======
  'ifeq ($(FOUNDFPC),)'#010+
  '$(error Compil','er $(FPC) not found)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  '# For 1.1 and up we can use a single compiler call to retrieve'#010+
  '# all needed information'#010+
  'ifndef FPC_COMPILERINFO'#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#010+
  'endif'#010+
  #010+
  '# FPC version'#010+
<<<<<<< HEAD
  'i','fndef FPC_VERSION'#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPILERINFO))'#010+
=======
  'ifndef FPC_VERSION'#010+
  'FPC_VERS','ION:=$(word 1,$(FPC_COMPILERINFO))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  'export FPC FPC_VERSION FPC_COMPILERINFO'#010+
  '# CHECKDEPEND should not be exported'#010+
  '# This should limit multiple checks'#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#010+
  #010+
<<<<<<< HEAD
  '#####################','###############################################'+
=======
  '###############################################','#####################'+
>>>>>>> origin/fixes_2.4
  '#'#010+
  '# FPC Target Detection'#010+
  '#####################################################################'#010+
  #010+
  '# Fall back to default values if needed'#010+
  'ifndef CPU_TARGET'#010+
  'ifdef CPU_TARGET_DEFAULT'#010+
  'CPU_TARGET=$(C','PU_TARGET_DEFAULT)'#010+
  'endif'#010+
<<<<<<< HEAD
  'endif'#010+
=======
  'e','ndif'#010+
>>>>>>> origin/fixes_2.4
  'ifndef OS_TARGET'#010+
  'ifdef OS_TARGET_DEFAULT'#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
=======
  '# For 1.0.x we need to use extra calls to retrieve all info'#010+
  'ifneq ($(words $(FPC_COMPILERINFO)),5)'#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSP)'#010+
  'FPC_COMPILER','INFO+=$(shell $(FPC) -iTP)'#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSO)'#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTO)'#010+
  'endif'#010+
  #010+
>>>>>>> origin/fixes_2.4
  '# Retrieve Target/Source CPU and Target/Source OS'#010+
  'ifndef CPU_SOURCE'#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
<<<<<<< HEAD
  'ifndef C','PU_TARGET'#010+
=======
  'ifndef CPU_T','ARGET'#010+
>>>>>>> origin/fixes_2.4
  'CPU_TARGET:=$(word 3,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  'ifndef OS_SOURCE'#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  'ifndef OS_TARGET'#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#010+
<<<<<<< HEAD
  'FULL_SOURCE=$(C','PU_SOURCE)-$(OS_SOURCE)'#010+
=======
  'FULL_SOURCE=$(CPU_S','OURCE)-$(OS_SOURCE)'#010+
>>>>>>> origin/fixes_2.4
  #010+
  'ifeq ($(CPU_TARGET),armeb)'#010+
  'ARCH=arm'#010+
  'override FPCOPT+=-Cb'#010+
  'else'#010+
  'ifeq ($(CPU_TARGET),armel)'#010+
  'ARCH=arm'#010+
  'override FPCOPT+=-CaEABI'#010+
  'else'#010+
  'ARCH=$(CPU_TARGET)'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  'ifeq ($(FULL_TARGET),arm-embedded)'#010+
  'ifeq ($(SUBARCH),)'#010+
  '#',' we don'#039't add a default here, people should explicitly add the '+
  'SUBARCH'#010+
  '$(error When compiling for arm-embedded, a sub-architecture (e.g. SUBA'+
  'RCH=armv4t or SUBARCH=armv7m) must be defined)'#010+
  'endif'#010+
  'override FPCOPT+=-Cp$(SUBARCH)'#010+
  'endif'#010+
  #010+
  'ifeq ($(','FULL_TARGET),avr-embedded)'#010+
  'ifeq ($(SUBARCH),)'#010+
  '# we don'#039't add a default here, people should explicitly add the SU'+
  'BARCH'#010+
  '$(error When compiling for avr-embedded, a sub-architecture (e.g. SUBA'+
  'RCH=avr25 or SUBARCH=avr35) must be defined)'#010+
  'endif'#010+
  'o','verride FPCOPT+=-Cp$(SUBARCH)'#010+
  'endif'#010+
  #010+
  'ifeq ($(FULL_TARGET),mipsel-embedded)'#010+
  'ifeq ($(SUBARCH),)'#010+
  '# we don'#039't add a default here, people should explicitly add the SU'+
  'BARCH'#010+
  '$(error When compiling for mipsel-embedded, a sub-architecture (e.g. S'+
  'UBAR','CH=pic32mx) must be defined)'#010+
  'endif'#010+
  'override FPCOPT+=-Cp$(SUBARCH)'#010+
  'endif'#010+
  #010+
  '# Full name of the target, including CPU and OS. For OSs limited'#010+
=======
  '# Full name of the target, including CPU and OS. For OSs li','mited'#010+
>>>>>>> origin/fixes_2.4
  '# to 8.3 we only use the target OS'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#010+
  'TARGETSUFFIX=$(OS_','TARGET)'#010+
  'SOURCESUFFIX=$(OS_SOURCE)'#010+
  'else'#010+
  'ifneq ($(findstring $(OS_TARGET),$(LIMIT83fs)),)'#010+
  'TARGETSUFFIX=$(OS_TARGET)'#010+
  'else'#010+
  'TARGETSUFFIX=$(FULL_TARGET)'#010+
  'endif'#010+
  'SOURCESUFFIX=$(FULL_SOURCE)'#010+
  'endif'#010+
  #010+
  '# Cross compile flag'#010+
<<<<<<< HEAD
  'ifneq ($(FULL_TARGET),$(FULL_SO','URCE))'#010+
=======
  'ifneq ($(','FULL_TARGET),$(FULL_SOURCE))'#010+
>>>>>>> origin/fixes_2.4
  'CROSSCOMPILE=1'#010+
  'endif'#010+
  #010+
  '# Check if the Makefile supports this target, but not'#010+
  '# when the make target is to rebuild the makefile'#010+
  'ifeq ($(findstring makefile,$(MAKECMDGOALS)),)'#010+
<<<<<<< HEAD
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),)'#010+
  '$(er','ror The Makefile doesn'#039't support target $(FULL_TARGET), plea'+
  'se run fpcmake first)'#010+
=======
  'ifeq ($(findstring $(FULL_TARGET),$(MA','KEFILETARGETS)),)'#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  #010+
  '# Detect BSD, since BSD uses a slightly different directory hierarchy.'+
  #010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#010+
<<<<<<< HEAD
  'BSDhier=1'#010+
=======
  'BSDhier','=1'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# Detect Lin','ux, will also use its own directory hierarchy.'#010+
  'ifeq ($(OS_TARGET),linux)'#010+
  'linuxHier=1'#010+
  'endif'#010+
  #010+
  'ifndef CROSSCOMPILE'#010+
  'BUILDFULLNATIVE=1'#010+
  'export BUILDFULLNATIVE'#010+
  'endif'#010+
  #010+
  'ifdef BUILDFULLNATIVE'#010+
  'BUILDNATIVE=1'#010+
  'export BUILDNATIVE'#010+
  'endif'#010+
  #010+
  'export OS_TARGET O','S_SOURCE ARCH CPU_TARGET CPU_SOURCE FULL_TARGET FU'+
  'LL_SOURCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#010+
  #010+
<<<<<<< HEAD
  '[fpmakefpcdetect]'#010+
=======
  '[fpcdir','checkenv]'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  '# FPC Binary and Version Detection'#010+
  '#################','###################################################'+
  '#'#010+
  '# If FPCFPMAKE is not set and we'#039're cross-compiling, try to find a'+
  #010+
  '# native compiler.'#010+
  'ifndef FPCFPMAKE'#010+
  #010+
  'ifdef CROSSCOMPILE'#010+
  '# Search for the (native) ppc compiler which is used to do the l','ates'+
  't build'#010+
  '# of the native rtl'#010+
  #010+
  'ifeq ($(strip $(wildcard $(addsuffix /compiler/ppc$(SRCEXEEXT),$(FPCDI'+
  'R)))),)'#010+
  '# If that compiler is not found, use the common, installed native comp'+
  'iler'#010+
  '# Note that in this case FPCFPMAKE is not exported, be','cause it could'+
  ' be that'#010+
  '# a more suitable compiler is compiled later on'#010+
  '# check if fpc exists'#010+
  'FPCPROG:=$(strip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#010+
  'ifneq ($(FPCPROG),)'#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#010+
  'FPCFPMAKE:=$(shell $','(FPCPROG) -PB)'#010+
  '# if a native compiler is not found, fall back to fpc'#010+
  'ifeq ($(strip $(wildcard $(FPCFPMAKE))),)'#010+
  'FPCFPMAKE:=$(firstword $(FPCPROG))'#010+
  'endif'#010+
  'else'#010+
  '# fpc binary not found, fallback to ppc386'#010+
  'override FPCFPMAKE=$(firstword $(strip $','(wildcard $(addsuffix /ppc38'+
  '6$(SRCEXEEXT),$(SEARCHPATH)))))'#010+
  'endif'#010+
  'else'#010+
  '# Use the ppc compiler. Because the path to the compiler is relative, '+
  'do not'#010+
  '# evaluate the variable here, but do that on the fly.'#010+
  'FPCFPMAKE=$(strip $(wildcard $(addsuff','ix /compiler/ppc$(SRCEXEEXT),$'+
  '(FPCDIR))))'#010+
  'FPMAKE_SKIP_CONFIG=-n'#010+
  'export FPCFPMAKE'#010+
  'export FPMAKE_SKIP_CONFIG'#010+
  'endif'#010+
  #010+
  'else'#010+
  'FPMAKE_SKIP_CONFIG=-n'#010+
  'FPCFPMAKE=$(FPC)'#010+
  'endif'#010+
  #010+
  'endif'#010+
  #010+
  #010+
  '[fpcdircheckenv]'#010+
  '##################################################','##################'+
  '#'#010+
  '# FPCDIR Setting'#010+
  '#####################################################################'#010+
  #010+
  '# Test FPCDIR to look if the RTL dir exists'#010+
  'ifdef FPCDIR'#010+
<<<<<<< HEAD
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#010+
  'ifeq ($(wildcard $(addprefix $(FPCD','IR)/,rtl units)),)'#010+
=======
  'override FPCDIR',':=$(subst \,/,$(FPCDIR))'#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#010+
>>>>>>> origin/fixes_2.4
  'override FPCDIR=wrong'#010+
  'endif'#010+
  'else'#010+
  'override FPCDIR=wrong'#010+
  'endif'#010+
  #010+
  '# Default FPCDIR from Makefile.fpc'#010+
  'ifdef DEFAULT_FPCDIR'#010+
  'ifeq ($(FPCDIR),wrong)'#010+
<<<<<<< HEAD
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#010+
  'ifeq ($(wildcard $(addprefix $(F','PCDIR)/,rtl units)),)'#010+
=======
  'override FPCDIR:=$(s','ubst \,/,$(DEFAULT_FPCDIR))'#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#010+
>>>>>>> origin/fixes_2.4
  'override FPCDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '[fpcdirdetect]'#010+
  '# Detect FPCDIR'#010+
  'ifeq ($(FPCDIR),wrong)'#010+
  'ifdef inUnix'#010+
<<<<<<< HEAD
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#010+
  'override FPCDIR=/usr/lib','/fpc/$(FPC_VERSION)'#010+
=======
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#010,
  'ifeq ($(wildcard $(FPCDIR)/units),)'#010+
  'override FPCDIR=/usr/lib/fpc/$(FPC_VERSION)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'else'#010+
  'override FPCDIR:=$(subst /$(FPC),,$(firstword $(strip $(wildcard $(add'+
  'suffix /$(FPC),$(SEARCHPATH))))))'#010+
  'override FPCDIR:=$(FPCDIR)/..'#010+
<<<<<<< HEAD
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#010+
  'override FPCDIR:=$(FP','CDIR)/..'#010+
=======
  'ifeq ($(wildcar','d $(addprefix $(FPCDIR)/,rtl units)),)'#010+
  'override FPCDIR:=$(FPCDIR)/..'#010+
>>>>>>> origin/fixes_2.4
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#010+
  'override FPCDIR:=$(BASEDIR)'#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#010+
  'override FPCDIR=c:/pp'#010+
  'endif'#010+
  'endif'#010+
<<<<<<< HEAD
  'endif'#010+
  'endif'#010+
=======
  'e','ndif'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  '# Cross binaries dir'#010+
  'ifndef CROSSBINDIR'#010+
  'CR','OSSBINDIR:=$(wildcard $(FPCDIR)/bin/$(TARGETSUFFIX))'#010+
  'endif'#010+
  #010+
  '# Default binutils prefix for cross compile when the'#010+
  '# crossbindir is not set (except for Darwin)'#010+
<<<<<<< HEAD
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#010+
  'ifeq ($(OS_SOURCE),darwin)'#010+
  'DA','RWIN2DARWIN=1'#010+
=======
  'ifeq ($(OS_TARGET),dar','win)'#010+
  'ifeq ($(OS_SOURCE),darwin)'#010+
  'DARWIN2DARWIN=1'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  'ifndef BINUTILSPREFIX'#010+
  'ifndef CROSSBINDIR'#010+
  'ifdef CROSSCOMPILE'#010+
  'ifneq ($(OS_TARGET),msdos)'#010+
  'ifndef DARWIN2DARWIN'#010+
  'ifneq ($(CPU_TARGET),jvm)'#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#010+
  #010+
  '# For Android use standard NDK pref','ixes'#010+
  'ifeq ($(OS_TARGET),android)'#010+
  'ifeq ($(CPU_TARGET),arm)'#010+
  'BINUTILSPREFIX=arm-linux-androideabi-'#010+
  'else'#010+
  'ifeq ($(CPU_TARGET),i386)'#010+
  'BINUTILSPREFIX=i686-linux-android-'#010+
  'else'#010+
  'ifeq ($(CPU_TARGET),mipsel)'#010+
  'BINUTILSPREFIX=mipsel-linux-android-'#010+
  'endif'#010+
  'en','dif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'endif'#010+
  'endif'#010+
  'else'#010+
  'BINUTILSPREFIX=$(OS_TARGET)-'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Try first the full target name, otherwise try only'#010+
=======
  '# Try first the full target na','me, otherwise try only'#010+
>>>>>>> origin/fixes_2.4
  '# the OS for backwards compatibility'#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#010+
  'ifeq ($(','UNITSDIR),)'#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(OS_TARGET))'#010+
  'endif'#010+
  #010+
  '# Packages dir'#010+
<<<<<<< HEAD
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages $(FPCDIR)/package'+
  's/base $(FPCDIR)/packages/extra)'#010+
=======
  'PACKAGESDIR:=$(wildcard $(FPCDIR)',' $(FPCDIR)/packages $(FPCDIR)/packa'+
  'ges/base $(FPCDIR)/packages/extra)'#010+
>>>>>>> origin/fixes_2.4
  #010+
  #010+
  '[shelltools]'#010+
  '#####################################','###############################'+
  '#'#010+
  '# Shell tools'#010+
  '#####################################################################'#010+
<<<<<<< HEAD
  #010+
=======
  #010,
>>>>>>> origin/fixes_2.4
  '# Echo that can redir (must be able run in the default OS shell)'#010+
  'ifndef ECHOREDIR'#010+
  'ifndef inUnix'#010+
  'ECHOREDIR=echo'#010+
  'else'#010+
  'ECHORE','DIR=$(ECHO)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# To copy pograms'#010+
  'ifndef COPY'#010+
  'COPY:=$(CPPROG) -fp'#010+
  'endif'#010+
  #010+
  '# Copy a whole tree'#010+
<<<<<<< HEAD
  'ifndef COPYTREE'#010+
=======
  'ifndef COPYTREE'#010,
>>>>>>> origin/fixes_2.4
  'COPYTREE:=$(CPPROG) -Rfp'#010+
  'endif'#010+
  #010+
  '# Copy a whole tree'#010+
  'ifndef MKDIRTREE'#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#010+
  'endif'#010+
  #010+
  '# To move pograms'#010+
  'i','fndef MOVE'#010+
  'MOVE:=$(MVPROG) -f'#010+
  'endif'#010+
  #010+
  '# Check delete program'#010+
  'ifndef DEL'#010+
  'DEL:=$(RMPROG) -f'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Check deltree program'#010+
=======
  '# Check deltree progra','m'#010+
>>>>>>> origin/fixes_2.4
  'ifndef DELTREE'#010+
  'DELTREE:=$(RMPROG) -rf'#010+
  'endif'#010+
  #010+
  '# To install files'#010+
  'ifndef INSTALL'#010+
  'ifdef inUnix'#010+
  'INSTALL:=$(GINSTALL) -c -m 6','44'#010+
  'else'#010+
  'INSTALL:=$(COPY)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# To install programs'#010+
  'ifndef INSTALLEXE'#010+
  'ifdef inUnix'#010+
<<<<<<< HEAD
  'INSTALLEXE:=$(GINSTALL) -c -m 755'#010+
=======
  'INSTALLEXE:=$(GINSTALL) -c ','-m 755'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'INSTALLEXE:=$(COPY)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# To make a directory.'#010+
  'ifndef MKDIR'#010+
  'MKDIR:=$(GINSTALL) -m 755 -d'#010+
  'endif'#010+
  #010+
  'expor','t ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE M'+
  'KDIR'#010+
  #010+
  #010+
  '[defaultdirs]'#010+
<<<<<<< HEAD
=======
  '##################################','##################################'+
  '#'#010+
  '# Default Directories'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  '# Default Directories'#010+
  '################################################################','####'+
  '#'#010+
  #010+
  '# Units dir'#010+
  'ifdef REQUIRE_UNITSDIR'#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#010+
  'endif'#010+
  #010+
  '# Units dir'#010+
<<<<<<< HEAD
  'ifdef REQUIRE_PACKAGESDIR'#010+
=======
  'ifdef REQUIRE_PACK','AGESDIR'#010+
>>>>>>> origin/fixes_2.4
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#010+
  'endif'#010+
  #010+
  #010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#010+
  '# When zipping',' use the target os default, when normal install then'#010+
  '# use the source os as default'#010+
  'ifdef ZIPINSTALL'#010+
  '# Zipinstall'#010+
<<<<<<< HEAD
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#010+
=======
  'ifne','q ($(findstring $(OS_TARGET),$(UNIXs)),)'#010+
>>>>>>> origin/fixes_2.4
  'UNIXHier=1'#010+
  'endif'#010+
  'else'#010+
  '# Normal install'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs))',',)'#010+
  'UNIXHier=1'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# When install prefix is not set try to use prefix'#010+
  'ifndef INSTALL_PREFIX'#010+
  'ifdef PREFIX'#010+
<<<<<<< HEAD
  'INSTALL_PREFIX=$(PREFIX)'#010+
=======
  'INSTA','LL_PREFIX=$(PREFIX)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  '# set the prefix directory where to install everything'#010+
  'ifndef INSTALL_PREFIX'#010+
  'ifdef UNIXHi','er'#010+
  'INSTALL_PREFIX=/usr/local'#010+
  'else'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_BASEDIR:=/pp'#010+
  'else'#010+
<<<<<<< HEAD
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)'#010+
=======
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)',#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'export INSTALL_PREFIX'#010+
  #010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#010+
  '# used recursively for all ','subdirs'#010+
  'ifdef INSTALL_FPCSUBDIR'#010+
  'export INSTALL_FPCSUBDIR'#010+
  'endif'#010+
  #010+
  '# Where to place the resulting zip files'#010+
<<<<<<< HEAD
  'ifndef DIST_DESTDIR'#010+
=======
  'ifndef DIST_D','ESTDIR'#010+
>>>>>>> origin/fixes_2.4
  'DIST_DESTDIR:=$(BASEDIR)'#010+
  'endif'#010+
  'export DIST_DESTDIR'#010+
  #010+
  '# EXE/PPU Target directories'#010+
  'ifndef COMPILER_UNITTARGETDIR'#010+
  'ifde','f PACKAGEDIR_MAIN'#010+
  'COMPILER_UNITTARGETDIR=$(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#010+
  'else'#010+
<<<<<<< HEAD
  'COMPILER_UNITTARGETDIR=units/$(TARGETSUFFIX)'#010+
=======
  'COMPILER_UNITTARGETDIR=units/$(','TARGETSUFFIX)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  'ifndef COMPILER_TARGETDIR'#010+
  'COMPILER_TARGETDIR=.'#010+
  'endif'#010+
  #010+
  #010+
  '#########################################','###########################'+
  '#'#010+
  '# Install Directories'#010+
  '###################################################################','#'+
  '#'#010+
  #010+
  '# set the base directory where to install everything'#010+
  'ifndef INSTALL_BASEDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'IN','STALL_BASEDIR:=$(INSTALL_PREFIX)/lib/fpc/$(FPC_VERSION)'#010+
  'else'#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/$(PACKAGE_NAME)'#010+
<<<<<<< HEAD
  'endif'#010+
=======
  'en','dif'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# set the directory where to install the binaries'#010+
  'ifndef INSTALL_','BINDIR'#010+
  'ifdef UNIXHier'#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#010+
  'else'#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/bin'#010+
<<<<<<< HEAD
  '# for FPC packages install the binaries under their target subdir'#010+
=======
  '# for FPC pack','ages install the binaries under their target subdir'#010+
>>>>>>> origin/fixes_2.4
  'ifdef INSTALL_FPCPACKAGE'#010+
  'ifdef CROSSCOMPILE'#010+
  'ifdef CROSSINSTALL'#010+
  'INSTALL','_BINDIR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#010+
  'else'#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#010+
  'endif'#010+
  'else'#010+
<<<<<<< HEAD
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#010+
=======
  'INSTALL_B','INDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# set the directory where to install the units.'#010+
  'ifndef I','NSTALL_UNITDIR'#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'ifdef PACKAGE_NAME'#010+
<<<<<<< HEAD
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#010+
=======
  'I','NSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where to install shared libraries'#010+
  'ifndef INSTALL_L','IBDIR'#010+
  'ifdef UNIXHier'#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#010+
  'else'#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Where the source files will be stored'#010+
=======
  '# Wher','e the source files will be stored'#010+
>>>>>>> origin/fixes_2.4
  'ifndef INSTALL_SOURCEDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef BSDhier'#010+
  'SRCPREFIXDIR=share/src'#010+
  'else'#010+
  'ifdef ','linuxHier'#010+
  'SRCPREFIXDIR=share/src'#010+
  'else'#010+
  'SRCPREFIXDIR=src'#010+
  'endif'#010+
  'endif'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'ifdef INSTALL_FPCSUBDIR'#010+
<<<<<<< HEAD
=======
  'IN','STALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSI'+
  'ON)/$(INSTALL_FPCSUBDIR)/$(PACKAGE_NAME)'#010+
  'else'#010+
>>>>>>> origin/fixes_2.4
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INSTALL_FPCSUBDIR)/$(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_SO','URCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSI'+
  'ON)/$(PACKAGE_NAME)'#010+
  'endif'#010+
  'else'#010+
<<<<<<< HEAD
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#010+
=======
  'INSTALL_SOURCEDIR:=$(INSTALL_','PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)'+
  '-$(PACKAGE_VERSION)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'else'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'ifdef INSTALL_FPCSUBDIR'#010+
  'INS','TALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$('+
  'PACKAGE_NAME)'#010+
  'else'#010+
<<<<<<< HEAD
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#010+
=======
  'INSTALL_SOURCEDIR:=$(INSTALL_BASED','IR)/source/$(PACKAGE_NAME)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'else'#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where the doc file','s will be stored'#010+
  'ifndef INSTALL_DOCDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef BSDhier'#010+
  'DOCPREFIXDIR=share/doc'#010+
  'else'#010+
  'ifdef linuxHier'#010+
<<<<<<< HEAD
  'DOCPREFIXDIR=share/doc'#010+
=======
  'DOCPRE','FIXDIR=share/doc'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'DOCPREFIXDIR=doc'#010+
  'endif'#010+
  'endif'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXD','IR)/fpc-$(FPC_VERSION)'+
  '/$(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACKAGE_NAME)-$(PA'+
<<<<<<< HEAD
  'CKAGE_VERSION)'#010+
=======
  'CKAG','E_VERSION)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'else'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_DOCDIR:','=$(INSTALL_BASEDIR)/doc'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where to install the examples, under linux we use the doc dir'#010+
<<<<<<< HEAD
  '# because the copytree command will create a subdir itself'#010+
=======
  '# because t','he copytree command will create a subdir itself'#010+
>>>>>>> origin/fixes_2.4
  'ifndef INSTALL_EXAMPLEDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  #010+
  'ifdef B','SDhier'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#010+
  'else'#010+
  'ifdef linuxHier'#010+
<<<<<<< HEAD
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples'#010+
=======
  'IN','STALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(','PACKAGE_NAME)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'else'#010+
  #010+
  'ifdef BSDhier'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/$(PACKAGE_NAME)-$'+
<<<<<<< HEAD
  '(PACKAGE_VERSION)'#010+
=======
  '(','PACKAGE_VERSION)'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'ifdef linuxHier'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#010+
  'el','se'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#010+
  'endif'#010+
  #010+
  'endif'#010+
  'endif'#010+
  'else'#010+
<<<<<<< HEAD
  'ifdef INSTALL_FPCPACKAGE'#010+
=======
  'ifdef INSTALL_','FPCPACKAGE'#010+
>>>>>>> origin/fixes_2.4
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/exa','mples'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where the some extra (data)files will be stored'#010+
  'ifndef INSTALL_DATADIR'#010+
<<<<<<< HEAD
  'INSTALL_DATADIR=$(INSTALL_BASEDIR)'#010+
=======
  'INSTALL_DATADIR=$(IN','STALL_BASEDIR)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  'ifndef INSTALL_SHAREDDIR'#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '############################','########################################'+
=======
  '#####################################################################'#010+
  '# Cross compile dirs'#010+
  '#######################################################','#############'+
>>>>>>> origin/fixes_2.4
  '#'#010+
  '# Cross compile dirs'#010+
  '#####################################################################'#010+
  #010+
  'ifdef CROSSCOMPILE'#010+
  '# Directory where the cross compile tools are stored.'#010+
  '# First check if they are availab','le in FPCDIR. If no targets/ subdir'+
  #010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#010+
  'ifndef CROSSBINDIR'#010+
<<<<<<< HEAD
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#010+
=======
  'CROSSBIN','DIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#010+
>>>>>>> origin/fixes_2.4
  'ifeq ($(CROSSBINDIR),)'#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/c','ross/$(TARGETSUFFIX)/bin'+
  '/$(FULL_SOURCE))'#010+
  'endif'#010+
  'endif'#010+
  'else'#010+
  'CROSSBINDIR='#010+
  'endif'#010+
  #010+
  #010+
  '[dirlibc]'#010+
<<<<<<< HEAD
  '# On linux, try to find where libgcc.a is.'#010+
=======
  '# On linux, try to find where',' libgcc.a is.'#010+
>>>>>>> origin/fixes_2.4
  'ifeq ($(OS_SOURCE),linux)'#010+
  #010+
  '# Amd64 to i386?'#010+
  'ifndef GCCLIBDIR'#010+
  'ifeq ($(CPU_TARGET),i386)'#010+
  'ifneq ($(findstring x','86_64,$(shell uname -a)),)'#010+
  'ifeq ($(BINUTILSPREFIX),)'#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m32 -print-libgcc-file-name`)'#010+
<<<<<<< HEAD
  'endif'#010+
  'endif'#010+
=======
  'en','dif'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#010+
  'ifeq ($(BINUTILSPREFIX),)'#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgc','c-file-name`)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Try cross gcc'#010+
  'ifndef GCCLIBDIR'#010+
<<<<<<< HEAD
  'CROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSPREFIX)gcc$(SRCEXEE'+
  'XT),$(SEARCHPATH))))'#010+
=======
  'CROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSP','REFIX)gcc$(SRCEX'+
  'EEXT),$(SEARCHPATH))))'#010+
>>>>>>> origin/fixes_2.4
  'ifneq ($(CROSSGCC),)'#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-nam','e`)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Other libraries from ld.so.conf'#010+
  'ifndef OTHERLIBDIR'#010+
<<<<<<< HEAD
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.conf | awk '#039'{ ORS=" '+
  '"; print $1 }'#039')'#010+
=======
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.co','nf | awk '#039'{ ORS='+
  '" "; print $1 }'#039')'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  'ifdef inUnix'#010+
  'ifeq ($(OS_SOURCE),netbsd)'#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#010+
  'endif'#010+
  'exp','ort GCCLIBDIR OTHERLIB'#010+
  'endif'#010+
  #010+
  #010+
  '[extensions]'#010+
  '#####################################################################'#010+
<<<<<<< HEAD
  '# Default extensions'#010+
=======
  '# De','fault extensions'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  #010+
  '# Default needed extensions (Go32v','2,Linux)'#010+
  'BATCHEXT=.bat'#010+
  'LOADEREXT=.as'#010+
  'EXEEXT=.exe'#010+
  'PPLEXT=.ppl'#010+
  'PPUEXT=.ppu'#010+
  'OEXT=.o'#010+
  'ASMEXT=.s'#010+
  'SMARTEXT=.sl'#010+
<<<<<<< HEAD
  'STATICLIBEXT=.a'#010+
=======
  'STATICLIBEXT=.','a'#010+
>>>>>>> origin/fixes_2.4
  'SHAREDLIBEXT=.so'#010+
  'SHAREDLIBPREFIX=libfp'#010+
  'STATICLIBPREFIX=libp'#010+
  'IMPORTLIBPREFIX=libimp'#010+
  'RSTEXT=.rst'#010+
  '# external debug info for',' executable'#010+
  'EXEDBGEXT=.dbg'#010+
  '#DEBUGSYMEXT #for debugger symbol files, define only for targets which'+
  ' has this'#010+
  #010+
<<<<<<< HEAD
=======
  '# 1.0.x has target specific extensions for ppu files and objec','ts'#010+
  'ifeq ($(findstring 1.0.,$(FPC_VERSION)),)'#010+
  '# short version for 1.1 and above - no target specific extensions'#010+
  #010+
>>>>>>> origin/fixes_2.4
  '# Go32v1'#010+
  'ifeq ($(OS_TARGET),go32v1)'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=v1'#010+
  'endif'#010+
  #010+
  '# Go32v2'#010+
  'ifeq ($(OS_TARGET),go32v2)'#010+
<<<<<<< HEAD
  'STATICLIBPREFIX='#010+
  'SHOR','TSUFFIX=dos'#010+
=======
  'STATICLIBPREFIX='#010,
  'SHORTSUFFIX=dos'#010+
>>>>>>> origin/fixes_2.4
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# watcom'#010+
  'ifeq ($(OS_TARGET),watcom)'#010+
  'STATICLIBPREFIX='#010+
  'OEXT=.obj'#010+
  'ASMEXT=.asm'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=wat'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# Android native'#010+
  'ifneq ($(CPU_TARGET),jvm)'#010+
  'ifeq ($(OS_TARGET),android)'#010,
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=lnx'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Linux'#010+
  'ifeq ($(OS_TARGET),linux)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
<<<<<<< HEAD
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=lnx'#010+
  'endif'#010+
  #010+
  '# DragonFly BSD'#010+
  'ifeq ($(OS_TARGET),dragonfly)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010,
  'SHORTSUFFIX=df'#010+
=======
  'HASSHAREDLIB','=1'#010+
  'SHORTSUFFIX=lnx'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# FreeBSD'#010+
  'ifeq ($(OS_TARGET),freebsd)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=fbs'#010+
  'endif'#010+
  #010+
  '# NetBSD'#010+
  'ifeq ($(OS_TARGET),netbsd)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=nbs'#010+
  'endif'#010+
  #010+
  '# OpenBSD'#010+
<<<<<<< HEAD
  'ifeq ($(OS_TARGE','T),openbsd)'#010+
=======
  'ifeq ($(OS_T','ARGET),openbsd)'#010+
>>>>>>> origin/fixes_2.4
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=obs'#010+
  'endif'#010+
  #010+
  '# Win32'#010+
  'ifeq ($(OS_TARGET),win32)'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=w32'#010+
  'endif'#010+
  #010+
  '# OS/2'#010+
  'ifeq ($(OS_TARGET),os2)'#010+
  'BATCHEXT=.cmd'#010+
  'AOUTEXT=.out'#010+
  'STATICLIBPREFIX='#010+
<<<<<<< HEAD
  'SHAREDLIBEXT=.dll'#010+
  'S','HORTSUFFIX=os2'#010+
=======
  'SHAREDLIBEXT=.d','ll'#010+
  'SHORTSUFFIX=os2'#010+
>>>>>>> origin/fixes_2.4
  'ECHO=echo'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# EMX'#010+
  'ifeq ($(OS_TARGET),emx)'#010+
  'BATCHEXT=.cmd'#010+
  'AOUTEXT=.out'#010+
  'STATICLIBPREFIX='#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=emx'#010+
  'ECHO=echo'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# Amiga'#010+
  'ifeq ($(OS_TARGET),amiga)'#010+
  'EXEEXT='#010+
<<<<<<< HEAD
  'SHAREDL','IBEXT=.library'#010+
=======
  'SHA','REDLIBEXT=.library'#010+
>>>>>>> origin/fixes_2.4
  'SHORTSUFFIX=amg'#010+
  'endif'#010+
  #010+
  '# AROS'#010+
  'ifeq ($(OS_TARGET),aros)'#010+
  'EXEEXT='#010+
  'SHAREDLIBEXT=.library'#010+
  'SHORTSUFFIX=aros'#010+
  'endif'#010+
  #010+
  '# MorphOS'#010+
  'ifeq ($(OS_TARGET),morphos)'#010+
  'EXEEXT='#010+
  'SHAREDLIBEXT=.library'#010+
  'SHORTSUFFIX=mos'#010+
  'endif'#010+
  #010+
  '# Atari'#010+
  'ifeq ($(OS_TARGET','),atari)'#010+
  'EXEEXT=.ttp'#010+
  'SHORTSUFFIX=ata'#010+
  'endif'#010+
  #010+
  '# BeOS'#010+
  'ifeq ($(OS_TARGET),beos)'#010+
<<<<<<< HEAD
  'BATCHEXT=.sh'#010+
=======
  'BATCHE','XT=.sh'#010+
>>>>>>> origin/fixes_2.4
  'EXEEXT='#010+
  'SHORTSUFFIX=be'#010+
  'endif'#010+
  #010+
  '# Haiku'#010+
  'ifeq ($(OS_TARGET),haiku)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'SHORTSUFFIX=hai'#010+
  'endif'#010+
  #010+
  '# Solaris'#010+
  'ifeq ($(OS_TARGET),solaris)'#010+
  'BATCH','EXT=.sh'#010+
  'EXEEXT='#010+
  'SHORTSUFFIX=sun'#010+
  'endif'#010+
  #010+
  '# QNX'#010+
  'ifeq ($(OS_TARGET),qnx)'#010+
<<<<<<< HEAD
  'BATCHEXT=.sh'#010+
=======
  'BATCHEXT=.sh'#010,
>>>>>>> origin/fixes_2.4
  'EXEEXT='#010+
  'SHORTSUFFIX=qnx'#010+
  'endif'#010+
  #010+
  '# Netware clib'#010+
  'ifeq ($(OS_TARGET),netware)'#010+
  'EXEEXT=.nlm'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=nw'#010+
  'IMPORTLIBPREFIX=imp'#010+
  'endif'#010+
  #010+
  '# Netware lib','c'#010+
  'ifeq ($(OS_TARGET),netwlibc)'#010+
  'EXEEXT=.nlm'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=nwl'#010+
<<<<<<< HEAD
  'IMPORTLIBPREFIX=imp'#010+
=======
  'IMPORT','LIBPREFIX=imp'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# MacOS'#010+
  'ifeq ($(OS_TARGET),macos)'#010+
  'BATCHEXT='#010+
  'EXEEXT='#010+
  'DEBUGSYMEXT=.xcoff'#010+
  'SHORTSUFFIX=mac'#010+
  'IMPORTLIBPREFIX=imp'#010+
  'endif'#010+
  #010+
  '# Darwin'#010+
  'ifneq ($(finds','tring $(OS_TARGET),darwin iphonesim),)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=dwn'#010+
  'EXEDBGEXT=.dSYM'#010+
  'endif'#010+
  #010+
  '# gba'#010+
<<<<<<< HEAD
  'ifeq ($(OS_TARGET),gba)'#010+
=======
  'ifeq',' ($(OS_TARGET),gba)'#010+
>>>>>>> origin/fixes_2.4
  'EXEEXT=.gba'#010+
  'SHAREDLIBEXT=.so'#010+
  'SHORTSUFFIX=gba'#010+
  'endif'#010+
  #010+
  '# Symbian OS'#010+
  'ifeq ($(OS_TARGET),symbian)'#010+
  'SHA','REDLIBEXT=.dll'#010+
  'SHORTSUFFIX=symbian'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# NativeNT'#010+
  'ifeq ($(OS_TARGET),NativeNT)'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=nativent'#010+
  'endif'#010+
  #010+
  '# wii'#010+
  'ifeq ($(OS_TARGET),wii)'#010+
  'EXEEXT=.dol'#010+
  'SHAREDLIBEXT=.so'#010+
  'SHORTSUFFIX=wii'#010+
=======
  'else'#010+
  '# long version for 1.0.x - target specific extensions'#010+
  #010+
  '# Go32v1'#010+
  'ifeq ($(OS_TA','RGET),go32v1)'#010+
  'PPUEXT=.pp1'#010+
  'OEXT=.o1'#010+
  'ASMEXT=.s1'#010+
  'SMARTEXT=.sl1'#010+
  'STATICLIBEXT=.a1'#010+
  'SHAREDLIBEXT=.so1'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=v1'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# Go32v2'#010+
  'ifeq ($(OS_TARGET),go32v2)'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=dos'#010+
  'IMPORTLIBPREFIX='#010+
  'end','if'#010+
  #010+
  '# watcom'#010+
  'ifeq ($(OS_TARGET),watcom)'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=wat'#010+
  'IMPORTLIBPREFIX='#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# AIX'#010+
  'ifeq ($(OS_TARGET),aix)'#010,
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'SHORTSUFFIX=aix'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Java'#010+
  'ifeq ($(OS_TARGET),java)'#010+
  'OEXT=.class'#010+
  'ASMEXT=.j'#010+
  'SHAREDLIBEXT=.jar'#010+
  'SHORTSUFFIX=java'#010+
=======
  '# FreeBSD'#010+
  'ifeq ($(OS_TARGET),freebsd)'#010+
  'BATCHEXT=.sh',#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=fbs'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# Android java'#010+
  'ifeq ($(CPU_TARGET),jvm)'#010+
  'ifeq ($(OS_TARGET),android)'#010+
  'OEXT=.class'#010+
  'ASMEXT=.j'#010+
  'SHAREDLIBE','XT=.jar'#010+
  'SHORTSUFFIX=android'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# MS-DOS'#010+
  'ifeq ($(OS_TARGET),msdos)'#010+
  'STATICLIBPREFIX='#010+
  'STATICLIBEXT=.a'#010+
  'SHORTSUFFIX=d16'#010+
  'endif'#010+
  #010+
  '# Embedded'#010+
  'ifeq ($(OS_TARGET),embedded)'#010+
  'EXEEXT=.bin'#010+
  'SHORTSUFFIX=emb'#010+
  'endif'#010+
  #010+
  '# Win16'#010+
  'ifeq ($(OS_TARGET),win16)',#010+
  'STATICLIBPREFIX='#010+
=======
  '#',' Win32'#010+
  'ifeq ($(OS_TARGET),win32)'#010+
  'PPUEXT=.ppw'#010+
  'OEXT=.ow'#010+
  'ASMEXT=.sw'#010+
  'SMARTEXT=.slw'#010+
  'STATICLIBEXT=.aw'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=w32'#010+
  'endif'#010+
  #010+
  '# OS/2'#010+
  'ifeq ($(OS_TARGET),os2)'#010+
  'BATCHEXT=.cmd'#010+
  'PPUEXT=.ppo'#010+
  'ASMEXT=.so2'#010+
  'OEXT=.oo2'#010+
  'AOUTEXT=.out'#010+
  'SMARTEXT=.s','l2'#010+
  'STATICLIBPREFIX='#010+
  'STATICLIBEXT=.ao2'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=os2'#010+
  'ECHO=echo'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# Amiga'#010+
  'ifeq ($(OS_TARGET),amiga)'#010+
  'EXEEXT='#010+
  'PPUEXT=.ppu'#010+
  'ASMEXT=.s'#010+
  'OEXT=.o'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT=.a'#010+
  'SHAREDLIBEXT=.library'#010+
  'SHORTSUFFIX','=amg'#010+
  'endif'#010+
  #010+
  '# Atari'#010+
  'ifeq ($(OS_TARGET),atari)'#010+
  'PPUEXT=.ppu'#010+
  'ASMEXT=.s'#010+
  'OEXT=.o'#010+
  'SMARTEXT=.sl'#010+
>>>>>>> origin/fixes_2.4
  'STATICLIBEXT=.a'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=w16'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  #010+
  '# For 8.3 limited OS'#039's the short suffixes'#010+
  '# Otherwise use the full source/target names'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#010+
  'FPCMADE=fpcmade.$(SHORTSUFFI','X)'#010+
=======
  '# BeOS'#010+
  'ifeq ($(OS_TARGET),beos)'#010+
  'BATCHEXT=.sh'#010+
  'PPUEXT=.ppu'#010+
  'ASMEXT=.s'#010+
  'OEXT=.o'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT','=.a'#010+
  'EXEEXT='#010+
  'SHORTSUFFIX=be'#010+
  'endif'#010+
  #010+
  '# Solaris'#010+
  'ifeq ($(OS_TARGET),solaris)'#010+
  'BATCHEXT=.sh'#010+
  'PPUEXT=.ppu'#010+
  'ASMEXT=.s'#010+
  'OEXT=.o'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT=.a'#010+
  'EXEEXT='#010+
  'SHORTSUFFIX=sun'#010+
  'endif'#010+
  #010+
  '# QNX'#010+
  'ifeq ($(OS_TARGET),qnx)'#010+
  'BATCHEXT=.sh'#010+
  'PPUEXT=.ppu'#010+
  'ASMEXT=.s'#010,
  'OEXT=.o'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT=.a'#010+
  'EXEEXT='#010+
  'SHORTSUFFIX=qnx'#010+
  'endif'#010+
  #010+
  '# Netware'#010+
  'ifeq ($(OS_TARGET),netware)'#010+
  'STATICLIBPREFIX='#010+
  'PPUEXT=.ppu'#010+
  'OEXT=.o'#010+
  'ASMEXT=.s'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT=.a'#010+
  'SHAREDLIBEXT=.nlm'#010+
  'EXEEXT=.nlm'#010+
  'SHORTSUFFIX=nw'#010+
  'IMPORTLIBPREF','IX=imp'#010+
  'endif'#010+
  #010+
  '# Netware libc'#010+
  'ifeq ($(OS_TARGET),netwlibc)'#010+
  'STATICLIBPREFIX='#010+
  'PPUEXT=.ppu'#010+
  'OEXT=.o'#010+
  'ASMEXT=.s'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT=.a'#010+
  'SHAREDLIBEXT=.nlm'#010+
  'EXEEXT=.nlm'#010+
  'SHORTSUFFIX=nwl'#010+
  'IMPORTLIBPREFIX=imp'#010+
  'endif'#010+
  #010+
  '# MacOS'#010+
  'ifeq ($(OS_TARGET),macos)',#010+
  'BATCHEXT='#010+
  'PPUEXT=.ppu'#010+
  'ASMEXT=.s'#010+
  'OEXT=.o'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT=.a'#010+
  'EXEEXT='#010+
  'DEBUGSYMEXT=.xcoff'#010+
  'SHORTSUFFIX=mac'#010+
  'IMPORTLIBPREFIX=imp'#010+
  'endif'#010+
  #010+
  '#end of target specific settings'#010+
  'endif'#010+
  #010+
  '# For 8.3 limited OS'#039's the short suffixes'#010+
  '# Otherwise use th','e full source/target names'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)'#010+
>>>>>>> origin/fixes_2.4
  'ZIPSUFFIX=$(SHORTSUFFIX)'#010+
  'ZIPCROSSPREFIX='#010+
  'ZIPSOURCESUFFIX=src'#010+
  'ZIPEXAMPLESUFFIX=exm'#010+
  'else'#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#010+
<<<<<<< HEAD
  'ZIPSOURCESUFFIX=.source'#010+
  'ZIPEXAMPLESUFFIX=.examples'#010+
  'ifdef CROSSCOMPILE'#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#010+
  'ZIPCROSSPREFIX=$(TARG','ETSUFFIX)-'#010+
=======
  'ZIPSOURCESUFFI','X=.source'#010+
  'ZIPEXAMPLESUFFIX=.examples'#010+
  'ifdef CROSSCOMPILE'#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#010+
  'ZIPCROSSPREFIX=$(TARGETSUFFIX)-'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#010+
  'ZIPCROSSPREFIX='#010+
  'endif'#010+
  'endif'#010+
  #010+
  '[defaulttools]'#010+
<<<<<<< HEAD
=======
  '#################################################','###################'+
  '#'#010+
  '# Default Tools'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  '# Default Tools'#010+
  '###################################################################','#'+
  '#'#010+
  #010+
  '# Names of the binutils tools'#010+
  'ASNAME=$(BINUTILSPREFIX)as'#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#010+
<<<<<<< HEAD
  'RCNAME=$(BINUTILSPREFIX)rc'#010+
  'NASMNAME=$(BINUTILSPREFIX)nasm'#010+
  #010+
  '# assembler, redefine it if cross compiling'#010+
  'ifndef ASPROG'#010+
  'ifdef CR','OSSBINDIR'#010+
  'ASPROG=$(CROSSBINDIR)/$(ASNAME)$(SRCEXEEXT)'#010+
=======
  'RCNAME=$(BINUTILSPREF','IX)rc'#010+
  'ifneq ($(findstring 1.0.,$(FPC_VERSION)),)'#010+
  'ifeq ($(OS_TARGET),win32)'#010+
  'ifeq ($(CROSSBINDIR),)'#010+
  'ASNAME=asw'#010+
  'LDNAME=ldw'#010+
  'ARNAME=arw'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# assembler, redefine it if cross compiling'#010+
  'ifndef ASPROG'#010+
  'ifdef CROSSBINDIR'#010+
  'ASPROG=$(CROSS','BINDIR)/$(ASNAME)$(SRCEXEEXT)'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'ASPROG=$(ASNAME)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# linker, but probably not used'#010+
  'ifndef LDPROG'#010+
  'ifdef CROSSBINDIR'#010+
  'LDPROG=$(CROSSBINDIR)/$(LDNAME)$(SRCEXEEXT)'#010+
  'else'#010+
  'LDPROG=$(LDNAME)'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Resour','ce compiler'#010+
  'ifndef RCPROG'#010+
=======
  '# Resource compiler'#010+
  'ifndef RCPRO','G'#010+
>>>>>>> origin/fixes_2.4
  'ifdef CROSSBINDIR'#010+
  'RCPROG=$(CROSSBINDIR)/$(RCNAME)$(SRCEXEEXT)'#010+
  'else'#010+
  'RCPROG=$(RCNAME)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Archiver'#010+
  'ifndef ARPROG'#010+
  'ifdef CROSSBINDIR'#010+
  'ARPROG=$(CROSSBINDIR)/$(ARNAME)$(SRCEXEEXT)'#010+
  'else'#010+
  'ARPROG=$(ARNAME)'#010+
  'endif'#010+
  'en','dif'#010+
  #010+
  '# NASM assembler, redefine it if cross compiling'#010+
  'ifndef NASMPROG'#010+
  'ifdef CROSSBINDIR'#010+
  'NASMPROG=$(CROSSBINDIR)/$(NASMNAME)$(SRCEXEEXT)'#010+
  'else'#010+
  'NASMPROG=$(NASMNAME)'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Override defaults'#010+
=======
  '# Override defaults',#010+
>>>>>>> origin/fixes_2.4
  'AS=$(ASPROG)'#010+
  'LD=$(LDPROG)'#010+
  'RC=$(RCPROG)'#010+
  'AR=$(A','RPROG)'#010+
  'NASM=$(NASMPROG)'#010+
  #010+
  '# ppas.bat / ppas.sh'#010+
  'ifdef inUnix'#010+
  'PPAS=./ppas$(SRCBATCHEXT)'#010+
  'else'#010+
  'PPAS=ppas$(SRCBATCHEXT)'#010+
  'endif'#010+
  #010+
  '# ldconfig to rebuild .so cache'#010+
  'ifdef inUnix'#010+
  'LDCONFIG=ldconfig'#010+
  'else'#010+
  'LDCONFIG='#010+
  'endif'#010+
  #010+
  'ifdef DATE'#010+
  'DATESTR:=$(shell $(DATE',') +%Y%m%d)'#010+
  'else'#010+
<<<<<<< HEAD
  'DATESTR='#010+
  'endif'#010+
=======
  'DATES','TR='#010+
  'endif'#010+
  #010+
  '# Look if UPX is found for go32v2 and win32. We can'#039't use $UPX bec'+
  'uase'#010+
  '# upx uses that one itself (PFV)'#010+
  'ifndef UPXPROG'#010+
  'ifeq ($(OS_TARGET),go32v2)'#010+
  'UPXPROG:=1'#010+
  'endif'#010+
  'ifeq ($(OS_TARGET),win32)'#010+
  'UPXPROG:=1'#010+
  'endif'#010+
  'ifdef UPXPROG'#010+
  'UPXPROG:=','$(strip $(wildcard $(addsuffix /upx$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#010+
  'ifeq ($(UPXPROG),)'#010+
  'UPXPROG='#010+
  'else'#010+
  'UPXPROG:=$(firstword $(UPXPROG))'#010+
  'endif'#010+
  'else'#010+
  'UPXPROG='#010+
  'endif'#010+
  'endif'#010+
  'export UPXPROG'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '# Zip options'#010+
  'ZIPOPT=-9'#010+
  'ZIPEXT=.zip'#010+
  #010+
  '# Tar options'#010+
<<<<<<< HEAD
  'ifeq ($(USETAR),bz2)'#010+
=======
  'ifeq ($(US','ETAR),bz2)'#010+
>>>>>>> origin/fixes_2.4
  'TAROPT=vj'#010+
  'TAREXT=.tar.bz2'#010+
  'else'#010+
  'TAROPT=vz'#010+
  'TAREXT=.tar.gz'#010+
  'endif'#010+
  #010+
  #010+
  '[command_begin]'#010+
<<<<<<< HEAD
  '########################################################','############'+
=======
  '#####################################################################'#010+
  '# Compiler Command Line'#010+
  '#######################################################','#############'+
>>>>>>> origin/fixes_2.4
  '#'#010+
  '# Compiler Command Line'#010+
  '#####################################################################'#010+
  #010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#010+
  '# compiler this needs to be turned off'#010+
  'ifndef NOCPUDEF'#010+
  'override',' FPCOPTDEF=$(ARCH)'#010+
  'endif'#010+
  #010+
  #010+
  '# Load commandline OPT and add target and unit dir to be sure'#010+
<<<<<<< HEAD
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#010+
=======
  'ifne','q ($(OS_TARGET),$(OS_SOURCE))'#010+
>>>>>>> origin/fixes_2.4
  'override FPCOPT+=-T$(OS_TARGET)'#010+
  'endif'#010+
  #010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#010+
  'override FPCOPT+=-P$(ARCH)'#010+
  'endif'#010+
  #010+
  'ifeq ($(','OS_SOURCE),openbsd)'#010+
  'override FPCOPT+=-FD$(NEW_BINUTILS_PATH)'#010+
  'override FPCMAKEOPT+=-FD$(NEW_BINUTILS_PATH)'#010+
  'override FPMAKE_BUILD_OPT+=-FD$(NEW_BINUTILS_PATH)'#010+
  'endif'#010+
  #010+
  'ifndef CROSSBOOTSTRAP'#010+
<<<<<<< HEAD
  'ifneq ($(BINUTILSPREFIX),)'#010+
  'override FPCOPT+=-XP$(BINUT','ILSPREFIX)'#010+
=======
  'ifn','eq ($(BINUTILSPREFIX),)'#010+
  'override FPCOPT+=-XP$(BINUTILSPREFIX)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'ifneq ($(BINUTILSPREFIX),)'#010+
  'override FPCOPT+=-Xr$(RLINKPATH)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# When BINUTILSPREFIX is given and we are not cross-compiling then use'+
  #010+
  '# it while compiling the fpmake file. (For example to build i386-freeb'+
  'sd'#010+
  '# with',' BINUTILSPREFIX=i386-)'#010+
  'ifndef CROSSCOMPILE'#010+
  'ifneq ($(BINUTILSPREFIX),)'#010+
  'override FPCMAKEOPT+=-XP$(BINUTILSPREFIX)'#010+
  'override FPMAKE_BUILD_OPT+=-XP$(BINUTILSPREFIX)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# User dirs should be first, so they are looked at first'#010+
<<<<<<< HEAD
  'ifdef UNIT','DIR'#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#010+
=======
  'ifdef UNITDIR'#010+
  'override FPCOPT+=$(addprefix',' -Fu,$(UNITDIR))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'ifdef LIBDIR'#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#010+
  'endif'#010+
  'ifdef OBJDIR'#010+
  'override FPCOPT+=$(addprefix -Fo,$(OBJDIR))'#010+
  'endif'#010+
  'ifdef INCDIR'#010+
  'override FPCOPT+=$(addprefix -Fi,$(INCDIR))'#010+
  'en','dif'#010+
  #010+
  '# Smartlinking'#010+
<<<<<<< HEAD
  'ifdef LINKSMART'#010+
=======
  'ifdef LINKSM','ART'#010+
>>>>>>> origin/fixes_2.4
  'override FPCOPT+=-XX'#010+
  'endif'#010+
  #010+
  '# Smartlinking creation'#010+
  'ifdef CREATESMART'#010+
  'override FPCOPT+=-CX'#010+
  'endif'#010+
  #010+
  '# Debug'#010+
  'ifdef DEBUG'#010+
  'override FPCOPT+=-gl'#010+
  'override FPCOPTDEF+=DEBUG'#010+
  'endif'#010+
  #010+
  '# Release mode'#010+
<<<<<<< HEAD
  '# (strip, optimiz','e and don'#039't load fpc.cfg)'#010+
  '# fpc 2.1 has -O2 for all targets'#010+
=======
  '# (strip, optimize and don'#039't load fpc.cfg)'#010+
  '# fpc ','2.1 has -O2 for all targets'#010+
>>>>>>> origin/fixes_2.4
  'ifdef RELEASE'#010+
  'ifneq ($(findstring 2.0.,$(FPC_VERSION)),)'#010+
  'ifeq ($(CPU_TARGET),i386)'#010+
  'FPCCPUOPT:=-OG2p3'#010+
  'endif'#010+
  'ifeq ($(CPU_TARGET),powerpc)'#010+
  'FPCCPUOPT:=-O1r'#010+
  'endif'#010+
  'else'#010+
  'FPCCPUOPT:=-O2'#010+
<<<<<<< HEAD
  'en','dif'#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#010+
=======
  'endif'#010+
  'override FPCOPT+=-Ur -Xs $(F','PCCPUOPT) -n'#010+
>>>>>>> origin/fixes_2.4
  'override FPCOPTDEF+=RELEASE'#010+
  'endif'#010+
  #010+
  '# Strip'#010+
  'ifdef STRIP'#010+
  'override FPCOPT+=-Xs'#010+
  'endif'#010+
  #010+
  '# Optimizer'#010+
  'ifdef OPTIMIZE'#010+
  'override FPCOPT+=-O2'#010+
  'endif'#010+
  #010+
  '# Verbose settings (warning,note,info)'#010+
  'ifdef VERBOSE'#010+
  'over','ride FPCOPT+=-vwni'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Needed compiler options'#010+
=======
  '# Need','ed compiler options'#010+
>>>>>>> origin/fixes_2.4
  'ifdef COMPILER_OPTIONS'#010+
  'override FPCOPT+=$(COMPILER_OPTIONS)'#010+
  'endif'#010+
  'ifdef COMPILER_UNITDIR'#010+
  'override FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#010+
  'endif'#010+
  'ifdef COMPILER_LIBRARYDIR'#010+
<<<<<<< HEAD
  'override FPCO','PT+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#010+
=======
  'override FPCOPT+=$(addprefix -Fl,$(COMPILER_L','IBRARYDIR))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'ifdef COMPILER_OBJECTDIR'#010+
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#010+
  'endif'#010+
  'ifdef COMPILER_INCLUDEDIR'#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#010+
  'endif'#010+
  #010+
  '# Cross compi','ler utils'#010+
  'ifdef CROSSBINDIR'#010+
<<<<<<< HEAD
  'override FPCOPT+=-FD$(CROSSBINDIR)'#010+
=======
  'over','ride FPCOPT+=-FD$(CROSSBINDIR)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# Target dirs and the prefix to use for clean/install'#010+
  'ifdef COMPILER_TARGETDIR'#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#010+
  'override TARGETDIR','PREFIX='#010+
  'else'#010+
<<<<<<< HEAD
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR)/'#010+
=======
  'override TARGETDIRP','REFIX=$(COMPILER_TARGETDIR)/'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  '# Unit target dir (default is units/<cpu>-<os>/'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#010+
  'override',' UNITTARGETDIRPREFIX='#010+
  'else'#010+
<<<<<<< HEAD
  'override UNITTARGETDIRPREFIX=$(COMPILER_UNITTARGETDIR)/'#010+
=======
  'overr','ide UNITTARGETDIRPREFIX=$(COMPILER_UNITTARGETDIR)/'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'else'#010+
  'ifdef COMPILER_TARGETDIR'#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  'ifdef ','CREATESHARED'#010+
  'override FPCOPT+=-Cg'#010+
=======
  'ifdef CREATESHARED'#010+
  'override FPCOPT+=-C','g'#010+
  #010+
  'ifeq ($(CPU_TARGET),i386)'#010+
  'override FPCOPT+=-Aas'#010+
  'endif'#010+
  #010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# create always pic'#039'ed code on x86_64, mips and mipsel'#010+
  '# on unix-like systems'#010+
  'ifneq ($(findstring $(OS_TARGET),dragonfly freebsd openbsd netbsd linu'+
  'x solaris),)'#010+
  'ifneq ($(findstring $(CPU_TARGET),x86_','64 mips mipsel),)'#010+
  'override FPCOPT+=-Cg'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  'ifdef LINKSHARED'#010+
=======
  'i','fdef LINKSHARED'#010+
>>>>>>> origin/fixes_2.4
  '#!!! deactived for testing override FPCOPT+=-XD'#010+
  'endif'#010+
  #010+
  #010+
  '[command_libc]'#010+
  '# Add GCC lib path if asked'#010+
  'ifdef GCCLIBDIR'#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#010+
  'endif'#010+
  'ifdef OTHERLIBDIR',#010+
  'override FPCOPT+=$(addprefix -Fl,$(OTHERLIBDIR))'#010+
  'end','if'#010+
  #010+
  #010+
  '[command_end]'#010+
  '# Add commandline options last so they can override'#010+
  'ifdef OPT'#010+
  'override FPCOPT+=$(OPT)'#010+
  'endif'#010+
  #010+
  '# Override options to compile the fpmake-binary with command-line opti'+
  'ons p','rovided in FPMAKEBUILDOPT'#010+
  'ifdef FPMAKEBUILDOPT'#010+
  'override FPMAKE_BUILD_OPT+=$(FPMAKEBUILDOPT)'#010+
  'endif'#010+
  #010+
  '# Add defines from FPCOPTDEF to FPCOPT'#010+
  'ifdef FPCOPTDEF'#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOPTDEF))'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Was a config file specified ?'#010+
  'i','fdef CFGFILE'#010+
=======
  '# Was a config file ','specified ?'#010+
  'ifdef CFGFILE'#010+
>>>>>>> origin/fixes_2.4
  'override FPCOPT+=@$(CFGFILE)'#010+
  'endif'#010+
  #010+
  '# Use the environment to pass commandline options?'#010+
  'ifdef USEENV'#010+
  'override FPCEXTCMD:=$(FPCOPT)'#010+
  'override FPCOPT:=!FPCEXTCMD'#010+
  'export FPCEXTCMD'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARG','ET)'#010+
=======
  'override AFULL_TARGET=$(CPU_TARG','ET)-$(OS_TARGET)'#010+
>>>>>>> origin/fixes_2.4
  'override AFULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#010+
  #010+
  'ifneq ($(AFULL_TARGET),$(AFULL_SOURCE))'#010+
  'override ACROSSCOMPILE=1'#010+
  'endif'#010+
  #010+
  'ifdef ACROSSCOMPILE'#010+
  'override FPCOPT+=$(CROSSOPT)'#010+
  'endif'#010+
  #010+
  '# Compiler commandline'#010+
<<<<<<< HEAD
  'override COMPILER:=$(strip $(FPC) $','(FPCOPT))'#010+
=======
  'override COMPILER:=$(F','PC) $(FPCOPT)'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '# also call ppas if with command option -s'#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#010+
  '# or if -sh is used'#010+
  'ifneq (,$(findstring -sh ,$(COMPILER)))'#010+
  'UseEXECPPAS=1'#010+
  'endif'#010+
  'ifneq (,$(findstring -s ,$(COMPILER)))'#010+
  'ifeq ($(FUL','L_SOURCE),$(FULL_TARGET))'#010+
  'UseEXECPPAS=1'#010+
  'endif'#010+
  'endif'#010+
  'ifneq ($(UseEXECPPAS),1)'#010+
  'EXECPPAS='#010+
  'else'#010+
  'ifdef RUNBATCH'#010+
<<<<<<< HEAD
  'EXECPPAS:=@$(RUNBATCH) $(PPAS)'#010+
=======
  'EXECPPAS:=@$(RUNBATCH',') $(PPAS)'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'EXECPPAS:=@$(PPAS)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  #010+
  '[loaderrules]'#010+
  '##################################################','##################'+
  '#'#010+
  '# Loaders'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_loaders'#010+
<<<<<<< HEAD
  #010+
=======
  #010,
>>>>>>> origin/fixes_2.4
  'ifneq ($(TARGET_LOADERS),)'#010+
  'override ALLTARGET+=fpc_loaders'#010+
  'override CLEANTARGET+=fpc_loaders_clean'#010+
  'override INSTALLTAR','GET+=fpc_loaders_install'#010+
  #010+
  'override LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#010+
  'endif'#010+
  #010+
  '%$(OEXT): %$(LOADEREXT)'#010+
<<<<<<< HEAD
  'ifdef COMPILER_UNITTARGETDIR'#010+
=======
  'ifd','ef COMPILER_UNITTARGETDIR'#010+
>>>>>>> origin/fixes_2.4
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#010+
  'else'#010+
  '        $(AS) -o $*$(OEXT) $<'#010+
  'e','ndif'#010+
  #010+
  'fpc_loaders: $(COMPILER_UNITTARGETDIR) $(LOADEROFILES)'#010+
  #010+
  'fpc_loaders_clean:'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
<<<<<<< HEAD
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#010+
=======
  '        -$(D','EL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILE'+
  'S))'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  '        -$(DEL) $(LOADEROFILES)'#010+
  'endif'#010+
  #010+
  'fpc_loaders_in','stall:'#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
<<<<<<< HEAD
  '        $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFIL'+
  'ES)) $(INSTALL_UNITDIR)'#010+
=======
  '        $(INSTALL) $(addprefix $(COMPILER_UNITTARG','ETDIR)/,$(LOADEROF'+
  'ILES)) $(INSTALL_UNITDIR)'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#010+
  'endif'#010+
  #010+
  #010+
  '[unitru','les]'#010+
  '#####################################################################'#010+
  '# Units'#010+
<<<<<<< HEAD
  '#####################################################################'#010+
=======
  '#######################################','#############################'+
  '#'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '.PHONY: fpc_units'#010+
  #010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#010+
  'override ALLTARGET+','=fpc_units'#010+
  #010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_UNITS))'#010+
<<<<<<< HEAD
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#010+
=======
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEX','T),$(TARGET_IMPLICI'+
  'TUNITS))'#010+
>>>>>>> origin/fixes_2.4
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#010+
  'override CLEANPPUFILES+=','$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#010+
  'endif'#010+
  #010+
  'fpc_units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#010+
  #010+
  #010+
  '[exerules]'#010+
<<<<<<< HEAD
=======
  '#########','###########################################################'+
  '#'#010+
  '# Exes'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  '# Exes'#010+
  '##################################################','##################'+
  '#'#010+
  #010+
  '.PHONY: fpc_exes'#010+
  #010+
  '# Programs are not needed for a cross installation'#010+
  'ifndef CROSSINSTALL'#010+
<<<<<<< HEAD
  'ifneq ($(TARGET_PROGRAMS),)'#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#010+
  'override EXEOFILES:=$(addsuffix $(OEXT),$(','TARGET_PROGRAMS)) $(addpre'+
  'fix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
=======
  'ifneq ($(TAR','GET_PROGRAMS),)'#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#010+
  'override EXEOFILES:=$(addsuffix $(OEXT),$(TARGET_PROGRAMS)) $(addprefi'+
  'x $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))) '+
  '$(addprefix $(IMPORTLIBP','REFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
>>>>>>> origin/fixes_2.4
  'PROGRAMS)))'#010+
  'override EXEDBGFILES:=$(addsuffix $(EXEDBGEXT),$(TARGET_PROGRA','MS))'#010+
  #010+
  'override ALLTARGET+=fpc_exes'#010+
  'override INSTALLEXEFILES+=$(EXEFILES)'#010+
  'override CLEANEXEFILES+=$(EXEFILES) $(EXEOFILES)'#010+
  'override CLEANEXEDBGFILES+=$(EXEDBGFILES)'#010+
  'ifeq ($(OS_TARGET),os2)'#010+
<<<<<<< HEAD
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TAR','GET_PROGRAMS))'#010+
=======
  'override CLEANEXEFILES+=$(addsuffix $(AOUTE','XT),$(TARGET_PROGRAMS))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'ifeq ($(OS_TARGET),emx)'#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'fpc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES)'#010+
  #010+
  #010+
  '[rstrules]'#010+
<<<<<<< HEAD
  '##########################','##########################################'+
=======
  '#################','###################################################'+
>>>>>>> origin/fixes_2.4
  '#'#010+
  '# Resource strings'#010+
  '#####################################################################'#010+
  #010+
  'ifdef TARGET_RSTS'#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_RSTS))'#010+
  #010+
<<<<<<< HEAD
  'override CLEANRSTFILES+=$(RSTFI','LES)'#010+
=======
  'override CLEANRSTFILES','+=$(RSTFILES)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  #010+
  '[examplerules]'#010+
  '#####################################################################'#010+
  '# Examples'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_examples'#010+
  #010+
  'ifneq ($(TARGET_EXAMPLES),)'#010+
<<<<<<< HEAD
  'HASEXAMPLE','S=1'#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $(addsuffix .pp,$(TARGET_EXAMP'+
  'LES)) $(addsuffix .pas,$(TARGET_EXAMPLES)) $(addsuffix .lpr,$(TARGET_E'+
  'XAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#010+
  'override EXAMPLEFILES:=$(addsuffix $(EXEEXT),$(T','ARGET_EXAMPLES))'#010+
  'override EXAMPLEOFILES:=$(addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(addp'+
  'refix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPLES'+
  '))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGE'+
  'T_EXAMPLES)))',#010+
  'override EXAMPLEDBGFILES:=$(addsuffix $(EXEDBGEXT),$(TARGET_EXAMPLES))'+
  #010+
=======
  'H','ASEXAMPLES=1'#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $(addsuffix .pp,$(TARGET_EXAMP'+
  'LES)) $(addsuffix .pas,$(TARGET_EXAMPLES)) $(addsuffix .lpr,$(TARGET_E'+
  'XAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#010+
  'override EXAMPLEFILES:=$(addsuffix $(EX','EEXT),$(TARGET_EXAMPLES))'#010+
  'override EXAMPLEOFILES:=$(addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(addp'+
  'refix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPLES'+
  '))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGE'+
  'T_EX','AMPLES)))'#010+
>>>>>>> origin/fixes_2.4
  #010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES) $(EXAMPLEOFILES)'#010+
  'override CLEANEXEDBGFILES+=$(EXAMPLEDBGFILES)'#010+
  'ifeq ($(OS_TARGET),os2)'#010+
  'override CLEANEXEFILES+=$(addsuffix $(AO','UTEXT),$(TARGET_EXAMPLES))'#010+
  'endif'#010+
  'ifeq ($(OS_TARGET),emx)'#010+
<<<<<<< HEAD
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#010+
=======
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TA','RGET_EXAMPLES))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  'ifneq ($(TARGET_EXAMPLEDIRS),)'#010+
  'HASEXAMPLES=1'#010+
  'endif'#010+
  #010+
  'fpc_examples: all $(EXAMPLEFILES) $(addsuffix _all,$','(TARGET_EXAMPLED'+
  'IRS))'#010+
  #010+
  #010+
  '[compilerules]'#010+
<<<<<<< HEAD
  '#####################################################################'#010+
=======
  '#####################################################################',
  #010+
>>>>>>> origin/fixes_2.4
  '# General compile rules'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_all fpc_smart fpc_debug ','fpc_release fpc_shared'#010+
  #010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMA','DE)'#010+
  #010+
  'fpc_all: $(FPCMADE)'#010+
  #010+
  'fpc_smart:'#010+
  '        $(MAKE) all LINKSMART=1 CREATESMART=1'#010+
  #010+
  'fpc_debug:'#010+
  '        $(MAKE) all DEBUG=1'#010+
  #010+
  'fpc_rele','ase:'#010+
  '        $(MAKE) all RELEASE=1'#010+
  #010+
  '# General compile rules, available for both possible .pp and .pas exte'+
<<<<<<< HEAD
  'nsions'#010+
=======
  'ns','ions'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '.SUFFIXES: $(EXEEXT) $(PPUEXT) $(OEXT) .pas .lpr .dpr .pp .rc .res'#010+
  #010+
  '$(COMPILER_UNITTARGETDIR):'#010+
  '        $(MKDIRTREE) $(COMPILER','_UNITTARGETDIR)'#010+
  #010+
  '$(COMPILER_TARGETDIR):'#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#010+
  #010+
  '%$(PPUEXT): %.pp'#010+
<<<<<<< HEAD
  '        $(COMPILER) $<'#010+
=======
  '       ',' $(COMPILER) $<'#010+
>>>>>>> origin/fixes_2.4
  '        $(EXECPPAS)'#010+
  #010+
  '%$(PPUEXT): %.pas'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%$(EXEEXT): %.pp'#010+
  '        $(COMPIL','ER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%$(EXEEXT): %.pas'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
<<<<<<< HEAD
  '%$(EXEEXT): %.lpr'#010+
=======
  '%$(EXEEXT): %.lpr'#010,
>>>>>>> origin/fixes_2.4
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%$(EXEEXT): %.dpr'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%.res: %.rc'#010+
  '        windre','s -i $< -o $@'#010+
  #010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#010+
<<<<<<< HEAD
  'vpath %.pp $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#010+
=======
  'vpath %.pp $(COMPILER_SOURCEDIR) $(COMPILER_IN','CLUDEDIR)'#010+
>>>>>>> origin/fixes_2.4
  'vpath %.pas $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#010+
  'vpath %.','dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#010+
  'vpath %.inc $(COMPILER_INCLUDEDIR)'#010+
<<<<<<< HEAD
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#010+
=======
  'vpath %$(OEXT) $(COMPILE','R_UNITTARGETDIR)'#010+
>>>>>>> origin/fixes_2.4
  'vpath %$(PPUEXT) $(COMPILER_UNITTARGETDIR)'#010+
  #010+
  '[sharedrules]'#010+
  '#########################################################','###########'+
  '#'#010+
  '# Library'#010+
  '#####################################################################'#010+
  #010+
<<<<<<< HEAD
  '.PHONY: fpc_shared'#010+
=======
  '.PHONY: fpc_sh','ared'#010+
>>>>>>> origin/fixes_2.4
  #010+
  'override INSTALLTARGET+=fpc_shared_install'#010+
  #010+
  'ifndef SHARED_LIBVERSION'#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#010+
  'endif'#010+
  #010+
  'ifndef SHARED_LIB','NAME'#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#010+
  'endif'#010+
  #010+
  'ifndef SHARED_FULLNAME'#010+
<<<<<<< HEAD
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED_LIBNAME)-$(SHARED_LIBVERSIO'+
  'N)$(SHAREDLIBEXT)'#010+
=======
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED','_LIBNAME)-$(SHARED_LIBVERS'+
  'ION)$(SHAREDLIBEXT)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# Default sharedlib units are all unit objects'#010+
  'ifndef SHARED_LIBUNITS'#010+
<<<<<<< HEAD
  'SHARED_LI','BUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#010+
  'override SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),$(SHARED_L'+
  'IBUNITS))'#010+
=======
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#010+
  'override SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),','$(SHARED'+
  '_LIBUNITS))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  'fpc_shared:'#010+
  'ifdef HASSHAREDLIB'#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#010+
  'ifneq ($(SHA','RED_BUILD),n)'#010+
  '        $(PPUMOVE) -q $(SHARED_LIBUNITS) -i$(COMPILER_UNITTARGETDIR) -'+
<<<<<<< HEAD
  'o$(SHARED_FULLNAME) -d$(COMPILER_UNITTARGETDIR)'#010+
=======
  'o$(SHARED_FULLNAME) -d$(','COMPILER_UNITTARGETDIR)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'else'#010+
  '        @$(ECHO) Shared Libraries not supported'#010+
  'endif'#010+
  #010+
  'fpc_shared_install:'#010+
  'ifneq ($(SHARED_BUILD)',',n)'#010+
  'ifneq ($(SHARED_LIBUNITS),)'#010+
  'ifneq ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME)),)'#010+
<<<<<<< HEAD
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#010+
=======
  '        $(','INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(IN'+
  'STALL_SHAREDDIR)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '[installrules]'#010+
<<<<<<< HEAD
  '#######################','#############################################'+
=======
  '#####################################################################'#010+
  '# Install rules'#010+
  '#############################################','#######################'+
>>>>>>> origin/fixes_2.4
  '#'#010+
  '# Install rules'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#010+
  #010+
  'ifdef INSTALL_UNITS'#010+
  'override INSTALLPPUFILES+=$(','addsuffix $(PPUEXT),$(INSTALL_UNITS))'#010+
  'endif'#010+
  #010+
  'ifdef INSTALL_BUILDUNIT'#010+
<<<<<<< HEAD
  'override INSTALLPPUFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT),$'+
  '(INSTALLPPUFILES))'#010+
  'endif'#010+
  #010+
  'ifdef INSTALLPPUFILES'#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEX','T),$(INSTALLPPU'+
  'FILES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEX'+
  'T),$(INSTALLPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#010+
  'override INST','ALLPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(notdi'+
  'r $(INSTALLPPUFILES)))'#010+
  'override INSTALLPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPR'+
  'EFIX),$(notdir $(INSTALLPPULINKFILES))))'#010+
=======
  'override INSTALLPPUFILES:=$(filter-out ','$(INSTALL_BUILDUNIT)$(PPUEXT)'+
  ',$(INSTALLPPUFILES))'#010+
  'endif'#010+
  #010+
  'ifdef INSTALLPPUFILES'#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPPUFI'+
  'LES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT)'+
  ',$(INSTALLPPUFILES)))',' $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#010+
  'override INSTALLPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(notdir '+
  '$(INSTALLPPUFILES)))'#010+
  'override INSTALLPPULINKFILES:=','$(wildcard $(addprefix $(UNITTARGETDIR'+
  'PREFIX),$(notdir $(INSTALLPPULINKFILES))))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  '# Implicitly install Package.fpc'#010+
  'override INS','TALL_CREATEPACKAGEFPC=1'#010+
  'endif'#010+
  #010+
  'ifdef INSTALLEXEFILES'#010+
  'ifneq ($(TARGETDIRPREFIX),)'#010+
<<<<<<< HEAD
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#010+
=======
  'override INSTALLEXEFILES:=$','(addprefix $(TARGETDIRPREFIX),$(notdir $('+
  'INSTALLEXEFILES)))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  'fpc_install: all $(INSTALLTARGET)'#010+
  'ifdef INSTALLEXEFILES'#010+
<<<<<<< HEAD
  '   ','     $(MKDIR) $(INSTALL_BINDIR)'#010+
=======
  '        $(MKDIR) $(INSTALL_BINDIR)'#010+
  '# Compress the exes if upx is defined'#010+
  'ifdef UPXPROG'#010+
  '        -$(UPXPROG) $(IN','STALLEXEFILES)'#010+
  'endif'#010+
>>>>>>> origin/fixes_2.4
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#010+
  'endif'#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#010+
  'ifdef FPCMAKE'#010+
  '# If the fpcpackage variable is set then create and install Package.fp'+
  'c,'#010+
<<<<<<< HEAD
  '# a safety check is done ','if Makefile.fpc is available'#010+
=======
  '# a safety check is done if Makefile','.fpc is available'#010+
>>>>>>> origin/fixes_2.4
  'ifdef PACKAGE_VERSION'#010+
  'ifneq ($(wildcard Makefile.fpc),)'#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS_TARGET) Makefile.fpc'#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#010+
  '        $(INSTALL) Package.fpc $(INSTALL_UNITDIR)'#010+
  'endif',#010+
  'endif'#010+
  'endif'#010+
<<<<<<< HEAD
=======
  'endi','f'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'ifdef INSTALLPPUFILES'#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(INSTALL_UNITDIR)'#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#010+
  '        $(INSTALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#010+
  'endif'#010+
<<<<<<< HEAD
  'ifneq (','$(wildcard $(LIB_FULLNAME)),)'#010+
=======
  'ifneq ($(wildcard ','$(LIB_FULLNAME)),)'#010+
>>>>>>> origin/fixes_2.4
  '        $(MKDIR) $(INSTALL_LIBDIR)'#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LIBDIR)'#010+
  'ifdef inUnix'#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_LIBDIR)/$(LIB_NAME)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'ifdef INSTALL_FILES'#010+
<<<<<<< HEAD
  '        $(','MKDIR) $(INSTALL_DATADIR)'#010+
=======
  '        $(MKDIR) $(IN','STALL_DATADIR)'#010+
>>>>>>> origin/fixes_2.4
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#010+
  'endif'#010+
  #010+
  'fpc_sourceinstall: distclean'#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#010+
  '        $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#010+
  #010+
<<<<<<< HEAD
  'fpc_exampleinstall: $(EXAMPLEI','NSTALLTARGET) $(addsuffix _distclean,$'+
  '(TARGET_EXAMPLEDIRS))'#010+
=======
  'fpc_exampleinstall: $(addsuffix _distclea','n,$(TARGET_EXAMPLEDIRS))'#010+
>>>>>>> origin/fixes_2.4
  'ifdef HASEXAMPLES'#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#010+
  'endif'#010+
  'ifdef EXAMPLESOURCEFILES'#010+
  '        $(COPY) $(EXAMPLESOURCEFILES) $(INSTALL_EXAMPLEDIR)'#010+
  'endif'#010+
  'ifdef TARGET_EXAMPLEDIRS'#010+
<<<<<<< HEAD
  ' ','       $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_'+
=======
  '        $(COPYTREE) $(addsuffix /*,$','(TARGET_EXAMPLEDIRS)) $(INSTALL_'+
>>>>>>> origin/fixes_2.4
  'EXAMPLEDIR)'#010+
  'endif'#010+
  #010+
  '[distinstallrules]'#010+
  '#####################################################################'#010+
  '# Dist Install'#010+
  '##################################################','##################'+
  '#'#010+
  #010+
<<<<<<< HEAD
  '.PHONY: fpc_distinstall'#010+
=======
  '.PHONY: fpc_di','stinstall'#010+
>>>>>>> origin/fixes_2.4
  #010+
  'fpc_distinstall: install exampleinstall'#010+
  #010+
  #010+
  '[zipinstallrules]'#010+
  '#####################################################################'#010+
  '# Zip'#010+
  '##########################################################','##########'+
  '#'#010+
  #010+
<<<<<<< HEAD
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#010+
=======
  '.PHONY: fpc_zipinstall',' fpc_zipsourceinstall fpc_zipexampleinstall'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '# Temporary path to pack a file, can only use a single deep'#010+
  '# subdir, because the deltree can'#039't see the whole tree to remove'#010+
  'ifndef PACKDIR'#010+
  'ifndef inUnix'#010+
  'PACKDI','R=$(BASEDIR)/../fpc-pack'#010+
  'else'#010+
<<<<<<< HEAD
  'PACKDIR=/tmp/fpc-pack'#010+
=======
  'PACKD','IR=/tmp/fpc-pack'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'endif'#010+
  #010+
  '# Maybe create default zipname from packagename'#010+
  'ifndef ZIPNAME'#010+
  'ifdef DIST_ZIPNAME'#010+
  'ZIPNAME=$(DIST_ZIPNAME)'#010+
  'else'#010+
  'ZIPNAME=$(PACKAGE_NAME)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'ifndef FULLZIPNAME'#010+
<<<<<<< HEAD
  'FULLZIPN','AME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#010+
=======
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(','ZIPNAME)$(ZIPSUFFIX)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  '# ZipTarget'#010+
  'ifndef ZIPTARGET'#010+
  'ifdef DIST_ZIPTARGET'#010+
  'ZIPTARGET=DIST_ZIPTARGET'#010+
  'else'#010+
  'ZIPTARGET=install'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Use tar by default under linux'#010+
  'ifndef USEZIP'#010+
  'ifdef inUnix'#010+
  'USETAR','=1'#010+
  'endif'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Use a wrapper script by default for Os/2'#010+
=======
  '# Use a wrapper scr','ipt by default for Os/2'#010+
>>>>>>> origin/fixes_2.4
  'ifndef inUnix'#010+
  'USEZIPWRAPPER=1'#010+
  'endif'#010+
  #010+
  '# We need to be able to run in the current OS so fix'#010+
  '# the path separator'#010+
  'ifdef USEZIPWRAPPER'#010+
  'ZIPPATHSEP=$(PATHSEP)'#010+
<<<<<<< HEAD
  'ZIPWRAPPER=$(subst /,$(PATHS','EP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT)'+
=======
  'ZIPWRAPPER=$(subst /,$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATC','HEXT)'+
>>>>>>> origin/fixes_2.4
  ')'#010+
  'else'#010+
  'ZIPPATHSEP=/'#010+
  'endif'#010+
  #010+
  '# Create commands to create the zip/tar file'#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#010+
  'ifdef USETAR'#010+
<<<<<<< HEAD
  'ZIPDEST','FILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(TAREXT)'#010+
  'ZIPCMD_ZIP:=$(TARPROG) c$(TAROPT)f $(ZIPDESTFILE) *'#010+
=======
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME',')$(TAREXT)'#010+
  'ZIPCMD_ZIP:=$(TARPROG) cf$(TAROPT) $(ZIPDESTFILE) *'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
  'TFILE) *'#010+
  'endif',#010+
  #010+
  'fpc_zipinstall:'#010+
<<<<<<< HEAD
  '        $(MAKE) $(ZIPTARGET) INSTALL_PREFIX=$(PACKDIR) ZIPINSTALL=1'#010+
=======
  '        $(MAKE) $','(ZIPTARGET) INSTALL_PREFIX=$(PACKDIR) ZIPINSTALL=1'#010+
>>>>>>> origin/fixes_2.4
  '        $(MKDIR) $(DIST_DESTDIR)'#010+
  '        $(DEL) $(ZIPDESTFILE)'#010+
  'ifdef USEZIPWRAPPER'#010+
  '# Handle gecho separate as we need to espace \ with \\'#010+
<<<<<<< HEAD
  'ifneq ($(ECHORED','IR),echo)'#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDPACK))" > $(ZIPWRAPPE'+
  'R)'#010+
=======
  ';'#013#010+
  #013#010+
  '[defines]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Misc defines to be used by anyone'#013#010+
  '#############################################','#######################'+
  '#'#013#010+
  #013#010+
  '# OS categories'#013#010+
  'BSDs = freebsd netbsd openbsd darwin'#013#010+
  'UNIXs = linux $(BSDs) solaris qnx haiku'#013#010+
  'LIMIT83fs = go32v2 os2 emx watcom'#013#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#013#010+
  #013#010+
  '#Empty target for rules that always s','hould run. Needed if'#013#010+
  '#the target is non-phoney, and there is non-phony prereqisites.'#013#010+
  '#Then add FORCE as an prerequisite'#013#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#013#010+
  'FORCE:'#013#010+
  '.PHONY: FORCE'#013#010+
  #013#010+
  '[osdetect]'#013#010+
  '#############','#######################################################'+
  '#'#013#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#013+
  #010+
  '# define inUnix when running under Unix like environment'#013#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,C','ygwin)'#013#010+
  '# define inWinNT when running under WinNT'#013#010+
  '# define inOS2 when running under OS/2'#013#010+
  '# define inCygwin when running under Cygwin32'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# We need only / in the path ','also remove the current dir,'#013#010+
  '# also remove trailing /'#039's'#013#010+
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#013#010+
  #013#010+
  '# Detect unix'#013#010+
  '# Darwin is handled specially'#013#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#013#010+
  'inUnix=1 #darwin'#013#010+
  'SEARCHPATH:=$(filter-','out .,$(subst :, ,$(PATH)))'#013#010+
  'else'#013#010+
  '# Determine if we'#039've a unix searchpath by looking for a ;'#013#010+
  '# that normally doesn'#039't exists in the unix PATH var.'#013#010+
  'ifeq ($(findstring ;,$(PATH)),)'#013#010+
  'inUnix=1'#013#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#013,#010+
  'else'#013#010+
  'SEARCHPATH:=$(subst ;, ,$(PATH))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add path were make is located'#013#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#013#010+
  #013#010+
  '# Search for PWD'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#013#010+
  'ifeq ($(P','WD),)'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013#010+
  '$(error You need the GNU utils package to use this Makefile)'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT='#013#010+
  'endif'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT=.ex','e'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect NT - NT sets OS to Windows_NT'#013#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#013#010+
  'ifndef inUnix'#013#010+
  'ifeq ($(OS),Windows_NT)'#013#010+
  'inWinNT=1'#013#010+
  'else'#013#010+
  'ifdef OS2_SHELL'#013#010+
  'inOS2=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifneq ($(findstring cygdrive,$(PATH)),)'#013#010,
  'inCygWin=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# The extension of batch files / scripts'#013#010+
  'ifdef inUnix'#013#010+
  'SRCBATCHEXT=.sh'#013#010+
  'else'#013#010+
  'ifdef inOS2'#013#010+
  'SRCBATCHEXT=.cmd'#013#010+
  'else'#013#010+
  'SRCBATCHEXT=.bat'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef COMSPEC'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(OSNeedsComsp','ecToRunBatch)),)'#013#010+
  'ifndef RUNBATCH'#013#010+
  'RUNBATCH=$(COMSPEC) /C'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#013#010+
  '# at the end of a line'#013#010+
  'ifdef inUnix'#013#010+
  'PATHSEP=/'#013#010+
  'else'#013#010+
  'PATHSEP:=$(subst /,\,/)'#013#010+
  '# ','cygwin bash or sh can not handle backslashs'#013#010+
  'ifdef inCygWin'#013#010+
  'PATHSEP=/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Base dir'#013#010+
  'ifdef PWD'#013#010+
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#013#010+
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#013#010+
  'ifdef inCygWin'#013#010+
  'ifneq ($(findstring',' /cygdrive/,$(BASEDIR)),)'#013#010+
  'BASENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#013#010+
  'BASEDRIVE:=$(firstword $(subst /, ,$(BASENODIR)))'#013#010+
  'BASEDIR:=$(subst /cygdrive/$(BASEDRIVE)/,$(BASEDRIVE):/,$(BASEDIR))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'BASEDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  '# Ech','o is an internal command under OS/2 (and others), so it'#039's a'+
  'lways found'#013#010+
  'ifdef inOS2'#013#010+
  'ifndef ECHO'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH))'+
  '))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /echo$(SRCEXEE','XT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO=echo'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export ECHO'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdetect]'#013#010+
  '##################################################################','##'+
  '#'#013#010+
  '# FPC Binary and Version Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Compatibility with old makefiles'#013#010+
  'ifndef FPC'#013#010+
  'ifdef PP'#013#010+
  'FPC=$(PP)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try to detect the ppcXXX file to use by',' using "fpc -PB" option'#013+
  #010+
  '# to query for the default ppcXXX the fpc executable tries. When'#013#010+
  '# fpc is not found use ppc386 by default. Also when fpc -PB gives'#013#010+
  '# an error ppc386 will be used.'#013#010+
  'ifndef FPC'#013#010+
  '# check if fpc exists'#013#010+
  'FPCPROG:=$(str','ip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifneq ($(FPCPROG),)'#013#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#013#010+
  'ifneq ($(CPU_TARGET),)'#013#010+
  'FPC:=$(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#013#010+
  'else'#013#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#013#010+
  'endif'#013#010+
  '# Older fpc',' executables didn'#039't support it and return'#013#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#013#010+
  'ifneq ($(findstring Error,$(FPC)),)'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#013#010+
  'e','lse'#013#010+
  '# if the cross compiler is not found, fall back to fpc'#013#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#013#010+
  'FPC:=$(firstword $(FPCPROG))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# fpc binary not found, fallback to ppc386'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(ad','dsuffix /ppc386$(SRC'+
  'EXEEXT),$(SEARCHPATH)))))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Get a clean executable name'#013#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#013#010+
  'override FPC:=$(subst \,/,$(FPC))$(SRCEXEEXT)'#013#010+
  #013#010+
  '# Try to find the binary direct first, otherwise in ','the path,'#013#010+
  '# if not found give an error'#013#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  '$(error Compiler $(FPC) not found)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010,
  '# For 1.1 and up we can use a single compiler call to retrieve'#013#010+
  '# all needed information'#013#010+
  'ifndef FPC_COMPILERINFO'#013#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# FPC version'#013#010+
  'ifndef FPC_VERSION'#013#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPIL','ERINFO))'#013#010+
  'endif'#013#010+
  #013#010+
  'export FPC FPC_VERSION FPC_COMPILERINFO'#013#010+
  '# CHECKDEPEND should not be exported'#013#010+
  '# This should limit multiple checks'#013#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#013#010+
  #013#010+
  '#################################################################','###'+
  '#'#013#010+
  '# FPC Target Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Fall back to default values if needed'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'ifdef CPU_TARGET_DEFAULT'#013#010+
  'CPU_TARGET=$(CPU_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifn','def OS_TARGET'#013#010+
  'ifdef OS_TARGET_DEFAULT'#013#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.0.x we need to use extra calls to retrieve all info'#013#010+
  'ifneq ($(words $(FPC_COMPILERINFO)),5)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSP)'#013#010+
  'FPC_COMPILE','RINFO+=$(shell $(FPC) -iTP)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSO)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Retrieve Target/Source CPU and Target/Source OS'#013#010+
  'ifndef CPU_SOURCE'#013#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'if','ndef CPU_TARGET'#013#010+
  'CPU_TARGET:=$(word 3,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_SOURCE'#013#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013,#010+
  'FULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifeq ($(CPU_TARGET),armeb)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-Cb'#013#010+
  'else'#013#010+
  'ifeq ($(CPU_TARGET),armel)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-CaEABI'#013#010+
  'else'#013#010+
  'ARCH=$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Full name of the target',', including CPU and OS. For OSs limited'#013+
  #010+
  '# to 8.3 we only use the target OS'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'TARGETSUFFIX=$(OS_TARGET)'#013#010+
  'SOURCESUFFIX=$(OS_SOURCE)'#013#010+
  'else'#013#010+
  'TARGETSUFFIX=$(FULL_TARGET)'#013#010+
  'SOURCESUFFIX=$(FULL_SOUR','CE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compile flag'#013#010+
  'ifneq ($(FULL_TARGET),$(FULL_SOURCE))'#013#010+
  'CROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the Makefile supports this target, but not'#013#010+
  '# when the make target is to rebuild the makefile'#013#010+
  'ifeq ($(findstring makefile,$(MAKE','CMDGOALS)),)'#013#010+
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),)'#013#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Detect BSD, since BSD uses a slightly different directory hier','arch'+
  'y.'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#013#010+
  'BSDhier=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect Linux, will also use its own directory hierarchy.'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'linuxHier=1'#013#010+
  'endif'#013#010+
  #013#010+
  'export OS_TARGET OS_SOURCE ARCH CPU_TARGET CPU_SOURCE FULL','_TARGET FU'+
  'LL_SOURCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#013#010+
  #013#010+
  #013#010+
  '[fpcdircheckenv]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPCDIR Setting'#013#010+
  '#####################################################################',
  #013#010+
  #013#010+
  '# Test FPCDIR to look if the RTL dir exists'#013#010+
  'ifdef FPCDIR'#013#010+
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default FPC','DIR from Makefile.fpc'#013#010+
  'ifdef DEFAULT_FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdirdetect]'#013#010+
  '# Det','ect FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'ifdef inUnix'#013#010+
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#013#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#013#010+
  'override FPCDIR=/usr/lib/fpc/$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR:=$(subst /$(FPC),,$(firstwor','d $(strip $(wildcard $(a'+
  'ddsuffix /$(FPC),$(SEARCHPATH))))))'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'overrid','e FPCDIR:=$(BASEDIR)'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=c:/pp'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross binaries dir'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(FPCDIR)/bin/$(TARGETSUFFIX))'#013#010+
  'endif',#013#010+
  #013#010+
  '# Default binutils prefix for cross compile when the'#013#010+
  '# crossbindir is not set (except for Darwin)'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'ifeq ($(OS_SOURCE),darwin)'#013#010+
  'DARWIN2DARWIN=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef BINUTILSPREFIX',#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifndef DARWIN2DARWIN'#013#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try first the full target name, otherwise try only'#013#010+
  '# the OS for backwards compatibility'#013#010+
  'UNITSDIR:=','$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#013#010+
  'ifeq ($(UNITSDIR),)'#013#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(OS_TARGET))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Packages dir'#013#010+
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages $(FPCDIR)/package'+
  's/base $(FPCDIR)/packages/e','xtra)'#013#010+
  #013#010+
  #013#010+
  '[shelltools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Shell tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Echo that can redir (must be able run in the default O','S shell)'#013+
  #010+
  'ifndef ECHOREDIR'#013#010+
  'ifndef inUnix'#013#010+
  'ECHOREDIR=echo'#013#010+
  'else'#013#010+
  'ECHOREDIR=$(ECHO)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To copy pograms'#013#010+
  'ifndef COPY'#013#010+
  'COPY:=$(CPPROG) -fp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef COPYTREE'#013#010+
  'COPYTREE:=$(CPPROG) -Rfp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cop','y a whole tree'#013#010+
  'ifndef MKDIRTREE'#013#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#013#010+
  'endif'#013#010+
  #013#010+
  '# To move pograms'#013#010+
  'ifndef MOVE'#013#010+
  'MOVE:=$(MVPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check delete program'#013#010+
  'ifndef DEL'#013#010+
  'DEL:=$(RMPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check deltree program'#013#010+
  'ifndef DELTREE'#013#010+
  'DEL','TREE:=$(RMPROG) -rf'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install files'#013#010+
  'ifndef INSTALL'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALL:=$(GINSTALL) -c -m 644'#013#010+
  'else'#013#010+
  'INSTALL:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install programs'#013#010+
  'ifndef INSTALLEXE'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALLEXE:=$(GINSTALL) -c -m 75','5'#013#010+
  'else'#013#010+
  'INSTALLEXE:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To make a directory.'#013#010+
  'ifndef MKDIR'#013#010+
  'MKDIR:=$(GINSTALL) -m 755 -d'#013#010+
  'endif'#013#010+
  #013#010+
  'export ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE MKD'+
  'IR'#013#010+
  #013#010+
  #013#010+
  '[defaultdirs]'#013#010+
  '########################','############################################'+
  '#'#013#010+
  '# Default Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Units dir'#013#010+
  'ifdef REQUIRE_UNITSDIR'#013#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Units dir',#013#010+
  'ifdef REQUIRE_PACKAGESDIR'#013#010+
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#013#010+
  '# When zipping use the target os default, when normal install then'#013+
  #010+
  '# use the source os as default'#013#010+
  'ifdef ','ZIPINSTALL'#013#010+
  '# Zipinstall'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# Normal install'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# When install prefix is not set try to use prefix'#013,#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef PREFIX'#013#010+
  'INSTALL_PREFIX=$(PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the prefix directory where to install everything'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_PREFIX=/usr/local'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTAL','L_BASEDIR:=/pp'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export INSTALL_PREFIX'#013#010+
  #013#010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#013#010+
  '# used recursively for all subdirs'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'export INSTALL_FPCSUBD','IR'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to place the resulting zip files'#013#010+
  'ifndef DIST_DESTDIR'#013#010+
  'DIST_DESTDIR:=$(BASEDIR)'#013#010+
  'endif'#013#010+
  'export DIST_DESTDIR'#013#010+
  #013#010+
  '# EXE/PPU Target directories'#013#010+
  'ifndef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef PACKAGEDIR_MAIN'#013#010+
  'COMPILER_UNITTARGETDIR=$','(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#013#010+
  'else'#013#010+
  'COMPILER_UNITTARGETDIR=units/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef COMPILER_TARGETDIR'#013#010+
  'COMPILER_TARGETDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# ','Install Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# set the base directory where to install everything'#013#010+
  'ifndef INSTALL_BASEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL','_PREFIX)/lib/fpc/$(FPC_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the binaries'#013#010+
  'ifndef INSTALL_BINDIR'#013#010+
  'ifdef ','UNIXHier'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/bin'#013#010+
  '# for FPC packages install the binaries under their target subdir'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifdef CROSSINSTALL'#013#010+
  'INSTALL_BIND','IR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the units.'#013#010+
  'i','fndef INSTALL_UNITDIR'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef PACKAGE_NAME'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install shared libraries'#013,#010+
  'ifndef INSTALL_LIBDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#013#010+
  'else'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the source files will be stored'#013#010+
  'ifndef INSTALL_SOURCEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'SRCPR','EFIXDIR=share/src'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'SRCPREFIXDIR=src'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INSTALL_FPC','SUBDIR)/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef',' INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$(PA'+
  'CKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_B','ASEDIR)/source'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the doc files will be stored'#013#010+
  'ifndef INSTALL_DOCDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'DOCPREFIXDIR=doc'#013#010+
  'endif'#013#010+
  'endif'#013#010,
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/fpc-$(FPC_VERSION)/$'+
  '(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACKAGE_NAME)-$(PA'+
  'CKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAG','E'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install the examples, under linux we use the doc dir'#013#010+
  '# because the copytree command will create a su','bdir itself'#013#010+
  'ifndef INSTALL_EXAMPLEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL','_DOCDIR)/examples'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010,
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACK','AGE'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the some extra (data)files will be stored'#013#010+
  'ifndef INSTALL_DATADIR'#013#010+
  'INSTALL_DATADIR=$','(INSTALL_BASEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef INSTALL_SHAREDDIR'#013#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#013#010+
  'endif'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# Cross compile dirs'#013#010+
  '###########################################','#########################'+
  '#'#013#010+
  #013#010+
  'ifdef CROSSCOMPILE'#013#010+
  '# Directory where the cross compile tools are stored.'#013#010+
  '# First check if they are available in FPCDIR. If no targets/ subdir'#013+
  #010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#013#010+
  'ifndef CR','OSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/cross/$(TARGETSUFFIX)/bin/$'+
  '(FULL_SOURCE))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'CROSSBINDIR='#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[dirlibc]'#013,#010+
  '# On linux, try to find where libgcc.a is.'#013#010+
  'ifeq ($(OS_SOURCE),linux)'#013#010+
  #013#010+
  '# Amd64 to i386?'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'ifneq ($(findstring x86_64,$(shell uname -a)),)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname ','`gcc -m32 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try cross gcc'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'C','ROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSPREFIX)gcc$(SRCEX'+
  'EEXT),$(SEARCHPATH))))'#013#010+
  'ifneq ($(CROSSGCC),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Other libraries from ld.so.conf'#013#010+
  'ifndef OT','HERLIBDIR'#013#010+
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.conf | awk '#039'{ ORS=" '+
  '"; print $1 }'#039')'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef inUnix'#013#010+
  'ifeq ($(OS_SOURCE),netbsd)'#013#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#013#010+
  'endif'#013#010+
  'export GCCLIBDIR OTHERLIB'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[extensions]'#013#010+
  '##','##################################################################'+
  '#'#013#010+
  '# Default extensions'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Default needed extensions (Go32v2,Linux)'#013#010+
  'BATCHEXT=.bat'#013#010+
  'LOADEREXT=.as'#013#010+
  'EX','EEXT=.exe'#013#010+
  'PPLEXT=.ppl'#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHAREDLIBPREFIX=libfp'#013#010+
  'STATICLIBPREFIX=libp'#013#010+
  'IMPORTLIBPREFIX=libimp'#013#010+
  'RSTEXT=.rst'#013#010+
  '#DEBUGSYMEXT #for debugger symbol files, define only',' for targets whi'+
  'ch has this'#013#010+
  #013#010+
  '# 1.0.x has target specific extensions for ppu files and objects'#013#010+
  'ifeq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  '# short version for 1.1 and above - no target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TARGET),go','32v1)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'OEXT=.obj'#013#010+
  'ASMEXT=.asm'#013#010+
  'SHAREDLIBE','XT=.dll'#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSU','FFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq',' ($(OS_TARGET),win32)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# EMX'#013#010+
  'ifeq ($(OS_','TARGET),emx)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=emx'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# MorphOS'#013#010,
  'ifeq ($(OS_TARGET),morphos)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=mos'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif',#013#010+
  #013#010+
  '# Haiku'#013#010+
  'ifeq ($(OS_TARGET),haiku)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=hai'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSU','FFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware clib'#013#010+
  'ifeq ($(OS_TARGET),netware)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPOR','TLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Darwin'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSH','AREDLIB=1'#013#010+
  'SHORTSUFFIX=dwn'#013#010+
  'endif'#013#010+
  #013#010+
  '# gba'#013#010+
  'ifeq ($(OS_TARGET),gba)'#013#010+
  'EXEEXT=.gba'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=gba'#013#010+
  'endif'#013#010+
  #013#010+
  '# Symbian OS'#013#010+
  'ifeq ($(OS_TARGET),symbian)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=symbian'#013#010+
  'endif'#013#010+
  #013#010+
  '# NativeNT'#013#010+
  'ifeq ($(OS','_TARGET),NativeNT)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=nativent'#013#010+
  'endif'#013#010+
  #013#010+
  '# wii'#013#010+
  'ifeq ($(OS_TARGET),wii)'#013#010+
  'EXEEXT=.dol'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=wii'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  '# long version for 1.0.x - target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(','OS_TARGET),go32v1)'#013#010+
  'PPUEXT=.pp1'#013#010+
  'OEXT=.o1'#013#010+
  'ASMEXT=.s1'#013#010+
  'SMARTEXT=.sl1'#013#010+
  'STATICLIBEXT=.a1'#013#010+
  'SHAREDLIBEXT=.so1'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013,#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010,
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BAT','CHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'PPUEXT=.ppw'#013#010+
  'OEXT=.ow'#013#010+
  'ASMEXT=.sw'#013#010+
  'SMARTEXT=.slw'#013#010+
  'STATICLIBEXT=.aw'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os','2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'PPUEXT=.ppo'#013#010+
  'ASMEXT=.so2'#013#010+
  'OEXT=.oo2'#013#010+
  'AOUTEXT=.out'#013#010+
  'SMARTEXT=.sl2'#013#010+
  'STATICLIBPREFIX='#013#010+
  'STATICLIBEXT=.ao2'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=a','ta'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEX','T=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware'#013#010+
  'ifeq ($(OS_T','ARGET),netware)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'STATICLIBPREFIX='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'ST','ATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '#end of target specific settings'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 8.3 limited OS'#039's the short suffixes'#013#010+
  '# Otherwise use the full source/target names'#013#010+
  'ifneq ($(findstri','ng $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)'#013#010+
  'ZIPSUFFIX=$(SHORTSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'ZIPSOURCESUFFIX=src'#013#010+
  'ZIPEXAMPLESUFFIX=exm'#013#010+
  'else'#013#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#013#010+
  'ZIPSOURCESUFFIX=.source'#013#010+
  'ZIPEXAMPLESUFFIX=.example','s'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#013#010+
  'ZIPCROSSPREFIX=$(TARGETSUFFIX)-'#013#010+
  'else'#013#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[defaulttools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# ','Default Tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Names of the binutils tools'#013#010+
  'ASNAME=$(BINUTILSPREFIX)as'#013#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#013#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#013#010+
  'RCNAME=$(BINUTILSPREFIX)rc'#013#010+
  'ifneq ($(','findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'ASNAME=asw'#013#010+
  'LDNAME=ldw'#013#010+
  'ARNAME=arw'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# assembler, redefine it if cross compiling'#013#010+
  'ifndef ASPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ASPROG=$(CROSSBI','NDIR)/$(ASNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ASPROG=$(ASNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# linker, but probably not used'#013#010+
  'ifndef LDPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'LDPROG=$(CROSSBINDIR)/$(LDNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'LDPROG=$(LDNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Resource compiler',#013#010+
  'ifndef RCPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'RCPROG=$(CROSSBINDIR)/$(RCNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'RCPROG=$(RCNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Archiver'#013#010+
  'ifndef ARPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ARPROG=$(CROSSBINDIR)/$(ARNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ARPROG=$(ARNAME)'#013#010+
  'endi','f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Override defaults'#013#010+
  'AS=$(ASPROG)'#013#010+
  'LD=$(LDPROG)'#013#010+
  'RC=$(RCPROG)'#013#010+
  'AR=$(ARPROG)'#013#010+
  #013#010+
  '# ppas.bat / ppas.sh'#013#010+
  'PPAS=ppas$(SRCBATCHEXT)'#013#010+
  #013#010+
  '# ldconfig to rebuild .so cache'#013#010+
  'ifdef inUnix'#013#010+
  'LDCONFIG=ldconfig'#013#010+
  'else'#013#010+
  'LDCONFIG='#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef DA','TE'#013#010+
  'DATESTR:=$(shell $(DATE) +%Y%m%d)'#013#010+
  'else'#013#010+
  'DATESTR='#013#010+
  'endif'#013#010+
  #013#010+
  '# Look if UPX is found for go32v2 and win32. We can'#039't use $UPX bec'+
  'ause'#013#010+
  '# upx uses that one itself (PFV)'#013#010+
  'ifndef UPXPROG'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifeq ($','(OS_TARGET),win32)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifdef UPXPROG'#013#010+
  'UPXPROG:=$(strip $(wildcard $(addsuffix /upx$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(UPXPROG),)'#013#010+
  'UPXPROG='#013#010+
  'else'#013#010+
  'UPXPROG:=$(firstword $(UPXPROG))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'UPXPROG='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'expo','rt UPXPROG'#013#010+
  #013#010+
  '# Zip options'#013#010+
  'ZIPOPT=-9'#013#010+
  'ZIPEXT=.zip'#013#010+
  #013#010+
  '# Tar options'#013#010+
  'ifeq ($(USETAR),bz2)'#013#010+
  'TAROPT=vj'#013#010+
  'TAREXT=.tar.bz2'#013#010+
  'else'#013#010+
  'TAROPT=vz'#013#010+
  'TAREXT=.tar.gz'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_begin]'#013#010+
  '###########################################################','#########'+
  '#'#013#010+
  '# Compiler Command Line'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#013+
  #010+
  '# compiler this needs to be turned off'#013#010+
  'ifndef NOCPUDEF'#013#010+
  'over','ride FPCOPTDEF=$(ARCH)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Load commandline OPT and add target and unit dir to be sure'#013#010+
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#013#010+
  'override FPCOPT+=-T$(OS_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#013#010+
  'override FPCOPT+=-P$(ARCH)'#013#010,
  'endif'#013#010+
  #013#010+
  'ifeq ($(OS_SOURCE),openbsd)'#013#010+
  'override FPCOPT+=-FD$(NEW_BINUTILS_PATH)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef CROSSBOOTSTRAP'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-XP$(BINUTILSPREFIX)'#013#010+
  'endif'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-Xr$(RLIN','KPATH)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# User dirs should be first, so they are looked at first'#013#010+
  'ifdef UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef LIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#013#010+
  'endif'#013#010+
  'ifdef OBJDIR'#013#010+
  'override F','PCOPT+=$(addprefix -Fo,$(OBJDIR))'#013#010+
  'endif'#013#010+
  'ifdef INCDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(INCDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking'#013#010+
  'ifdef LINKSMART'#013#010+
  'override FPCOPT+=-XX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking creation'#013#010+
  'ifdef CREATESMART'#013#010+
  'override FPCOPT+=-CX'#013#010,
  'endif'#013#010+
  #013#010+
  '# Debug'#013#010+
  'ifdef DEBUG'#013#010+
  'override FPCOPT+=-gl'#013#010+
  'override FPCOPTDEF+=DEBUG'#013#010+
  'endif'#013#010+
  #013#010+
  '# Release mode'#013#010+
  '# (strip, optimize and don'#039't load fpc.cfg)'#013#010+
  '# fpc 2.1 has -O2 for all targets'#013#010+
  'ifdef RELEASE'#013#010+
  'ifneq ($(findstring 2.0.,$(FPC_VERSION)),',')'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'FPCCPUOPT:=-OG2p3'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc)'#013#010+
  'FPCCPUOPT:=-O1r'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'FPCCPUOPT:=-O2'#013#010+
  'endif'#013#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#013#010+
  'override FPCOPTDEF+=RELEASE'#013#010+
  'endif'#013#010+
  #013#010+
  '# Strip'#013#010+
  'ifdef STRIP'#013,#010+
  'override FPCOPT+=-Xs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Optimizer'#013#010+
  'ifdef OPTIMIZE'#013#010+
  'override FPCOPT+=-O2'#013#010+
  'endif'#013#010+
  #013#010+
  '# Verbose settings (warning,note,info)'#013#010+
  'ifdef VERBOSE'#013#010+
  'override FPCOPT+=-vwni'#013#010+
  'endif'#013#010+
  #013#010+
  '# Needed compiler options'#013#010+
  'ifdef COMPILER_OPTIONS'#013#010+
  'override ','FPCOPT+=$(COMPILER_OPTIONS)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_LIBRARYDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_OBJECTDIR'#013#010,
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_INCLUDEDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compiler utils'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'override FPCOPT+=-FD$(CROSSBINDIR)'#013#010+
  'end','if'#013#010+
  #013#010+
  '# Target dirs and the prefix to use for clean/install'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#013#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#013#010+
  'override TARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR',')/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Unit target dir (default is units/<cpu>-<os>/'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#013#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#013#010+
  'override UNITTARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override UNITTARGETDI','RPREFIX=$(COMPILER_UNITTARGETDIR)/'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#013#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CREATESHARED'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  #013#010+
  'i','feq ($(CPU_TARGET),i386)'#013#010+
  'override FPCOPT+=-Aas'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  #013#010+
  '# create always pic'#039'ed code on x86_64'#013#010+
  'ifeq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'ifeq ($(CPU_TARGET),x86_64)'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  'endif'#013#010+
  'endif'#013,#010+
  'endif'#013#010+
  #013#010+
  'ifdef LINKSHARED'#013#010+
  '#!!! deactived for testing override FPCOPT+=-XD'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_libc]'#013#010+
  '# Add GCC lib path if asked'#013#010+
  'ifdef GCCLIBDIR'#013#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#013#010+
  'endif'#013#010+
  'ifdef OTHERLIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -','Fl,$(OTHERLIBDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_end]'#013#010+
  '# Add commandline options last so they can override'#013#010+
  'ifdef OPT'#013#010+
  'override FPCOPT+=$(OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add defines from FPCOPTDEF to FPCOPT'#013#010+
  'ifdef FPCOPTDEF'#013#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOP','TDEF))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Was a config file specified ?'#013#010+
  'ifdef CFGFILE'#013#010+
  'override FPCOPT+=@$(CFGFILE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use the environment to pass commandline options?'#013#010+
  'ifdef USEENV'#013#010+
  'override FPCEXTCMD:=$(FPCOPT)'#013#010+
  'override FPCOPT:=!FPCEXTCMD'#013#010+
  'export FPC','EXTCMD'#013#010+
  'endif'#013#010+
  #013#010+
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013#010+
  'override AFULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifneq ($(AFULL_TARGET),$(AFULL_SOURCE))'#013#010+
  'override ACROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef ACROSSCOMPILE'#013#010+
  'override FPCOPT+=$(CROSS','OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Compiler commandline'#013#010+
  'override COMPILER:=$(FPC) $(FPCOPT)'#013#010+
  #013#010+
  '# also call ppas if with command option -s'#013#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#013#010+
  'ifeq (,$(findstring -s ,$(COMPILER)))'#013#010+
  'EXECPPAS='#013#010+
  'else'#013#010+
  'ifeq ','($(FULL_SOURCE),$(FULL_TARGET))'#013#010+
  'ifdef RUNBATCH'#013#010+
  'EXECPPAS:=@$(RUNBATCH) $(PPAS)'#013#010+
  'else'#013#010+
  'EXECPPAS:=@$(PPAS)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[loaderrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Loaders'#013#010+
  '###########','#########################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_loaders'#013#010+
  #013#010+
  'ifneq ($(TARGET_LOADERS),)'#013#010+
  'override ALLTARGET+=fpc_loaders'#013#010+
  'override CLEANTARGET+=fpc_loaders_clean'#013#010+
  'override INSTALLTARGET+=fpc_loaders_install'#013#010+
  #013#010+
  'overri','de LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#013#010+
  'endif'#013#010+
  #013#010+
  '%$(OEXT): %$(LOADEREXT)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#013#010+
  'else'#013#010+
  '        $(AS) -o $*$(OEXT) $<'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders: $(C','OMPILER_UNITTARGETDIR) $(LOADEROFILES)'#013#010+
  #013#010+
  'fpc_loaders_clean:'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#013#010+
  'else'#013#010+
  '        -$(DEL) $(LOADEROFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders_install:'#013#010+
  '     ','   $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFIL'+
  'ES)) $(INSTALL_UNITDIR)'#013#010+
  'else'#013#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[unitrules]',#013#010+
  '#####################################################################'#013+
  #010+
  '# Units'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_units'#013#010+
  #013#010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#013#010+
  'override ALLTAR','GET+=fpc_units'#013#010+
  #013#010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_UNITS))'#013#010+
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#013#010+
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'override CLEANP','PUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#013#010+
  #013#010+
  #013#010+
  '[exerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Exes'#013#010+
  '################################','####################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_exes'#013#010+
  #013#010+
  '# Programs are not needed for a cross installation'#013#010+
  'ifndef CROSSINSTALL'#013#010+
  'ifneq ($(TARGET_PROGRAMS),)'#013#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#013#010+
  'override EXEOFIL','ES:=$(addsuffix $(OEXT),$(TARGET_PROGRAMS)) $(addpre'+
  'fix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
  'PROGRAMS)))'#013#010+
  #013#010+
  'override ALLTARGET+=fpc_exes'#013#010+
  'ove','rride INSTALLEXEFILES+=$(EXEFILES)'#013#010+
  'override CLEANEXEFILES+=$(EXEFILES) $(EXEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(add','suffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES)'#013+
  #010+
  #013#010+
  #013#010+
  '[rstrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Resource strings'#013#010,
  '#####################################################################'#013+
  #010+
  #013#010+
  'ifdef TARGET_RSTS'#013#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_RSTS))'#013#010+
  #013#010+
  'override CLEANRSTFILES+=$(RSTFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[examplerules]'#013#010+
  '#########################','###########################################'+
  '#'#013#010+
  '# Examples'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_examples'#013#010+
  #013#010+
  'ifneq ($(TARGET_EXAMPLES),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $','(addsuffix .pp,$(TARGET_EXA'+
  'MPLES)) $(addsuffix .pas,$(TARGET_EXAMPLES)) $(addsuffix .lpr,$(TARGET'+
  '_EXAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#013#010+
  'override EXAMPLEFILES:=$(addsuffix $(EXEEXT),$(TARGET_EXAMPLES))'#013#010+
  'override EXAMPLEOFILES:=$(','addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(ad'+
  'dprefix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPL'+
  'ES))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TAR'+
  'GET_EXAMPLES)))'#013#010+
  #013#010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES)',' $(EXAMPLEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifneq ($(TARGE','T_EXAMPLEDIRS),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_examples: all $(EXAMPLEFILES) $(addsuffix _all,$(TARGET_EXAMPLEDIR'+
  'S))'#013#010+
  #013#010+
  #013#010+
  '[compilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# General compile rules'#013#010+
  '#######','#############################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_all fpc_smart fpc_debug fpc_release fpc_shared'#013#010+
  #013#010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#013#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMADE)'#013#010+
  #013#010+
  'fpc_all: $(FPCMADE)'#013,#010+
  #013#010+
  'fpc_smart:'#013#010+
  '        $(MAKE) all LINKSMART=1 CREATESMART=1'#013#010+
  #013#010+
  'fpc_debug:'#013#010+
  '        $(MAKE) all DEBUG=1'#013#010+
  #013#010+
  'fpc_release:'#013#010+
  '        $(MAKE) all RELEASE=1'#013#010+
  #013#010+
  '# General compile rules, available for both possible .pp and .pas exte'+
  'nsions'#013#010+
  #013#010+
  '.SUFFI','XES: $(EXEEXT) $(PPUEXT) $(OEXT) .pas .lpr .dpr .pp .rc .res'#013+
  #010+
  #013#010+
  '$(COMPILER_UNITTARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '$(COMPILER_TARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pp'#013#010+
  '        $(','COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXE','EXT): %.lpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.dpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%.res: %.rc'#013#010+
  '        windres -i $< -o $@'#013#010+
  #013#010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#013#010+
  'vpath %.pp $(COMPILER_','SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.pas $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.inc $(COMPILER_INCLUDED','IR)'#013#010+
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  'vpath %$(PPUEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '[sharedrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Library'#013#010+
  '##################################################','##################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_shared'#013#010+
  #013#010+
  'override INSTALLTARGET+=fpc_shared_install'#013#010+
  #013#010+
  'ifndef SHARED_LIBVERSION'#013#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_LIBNAME'#013#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_FU','LLNAME'#013#010+
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED_LIBNAME)-$(SHARED_LIBVERSIO'+
  'N)$(SHAREDLIBEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default sharedlib units are all unit objects'#013#010+
  'ifndef SHARED_LIBUNITS'#013#010+
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#013#010+
  'over','ride SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),$(SHARED'+
  '_LIBUNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared:'#013#010+
  'ifdef HASSHAREDLIB'#013#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  '        $(PPUMOVE) -q $(SHARED_','LIBUNITS) -i$(COMPILER_UNITTARGETDIR)'+
  ' -o$(SHARED_FULLNAME) -d$(COMPILER_UNITTARGETDIR)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '        @$(ECHO) Shared Libraries not supported'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared_install:'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  'ifneq ($(SHARED_LIBUNITS),)'#013#010+
  'ifne','q ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME)),)'#013+
  #010+
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[installrules]'#013#010+
  '####################################################','################'+
  '#'#013#010+
  '# Install rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#013#010+
  #013#010+
  'ifdef INSTALL_UNITS'#013#010+
  'override INSTALLPPUFILES+=$(addsuffix $(PPUEXT),$(','INSTALL_UNITS))'#013+
  #010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALL_BUILDUNIT'#013#010+
  'override INSTALLPPUFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT),$'+
  '(INSTALLPPUFILES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPP','U'+
  'FILES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEX'+
  'T),$(INSTALLPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#013#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#013#010+
  'override INSTALLPPUFILES:','=$(addprefix $(UNITTARGETDIRPREFIX),$(notdi'+
  'r $(INSTALLPPUFILES)))'#013#010+
  'override INSTALLPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPR'+
  'EFIX),$(notdir $(INSTALLPPULINKFILES))))'#013#010+
  'endif'#013#010+
  '# Implicitly install Package.fpc'#013#010+
  'override INSTALL_CRE','ATEPACKAGEFPC=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  'ifneq ($(TARGETDIRPREFIX),)'#013#010+
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_install: all $(INSTALLTARGET)'#013#010+
  'ifdef INSTALLEXEFILES'#013#010,
  '        $(MKDIR) $(INSTALL_BINDIR)'#013#010+
  '# Compress the exes if upx is defined'#013#010+
  'ifdef UPXPROG'#013#010+
  '        -$(UPXPROG) $(INSTALLEXEFILES)'#013#010+
  'endif'#013#010+
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#013#010+
  'ifde','f FPCMAKE'#013#010+
  '# If the fpcpackage variable is set then create and install Package.fp'+
  'c,'#013#010+
  '# a safety check is done if Makefile.fpc is available'#013#010+
  'ifdef PACKAGE_VERSION'#013#010+
  'ifneq ($(wildcard Makefile.fpc),)'#013#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS','_TARGET) Makefile.fpc'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) Package.fpc $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(I','NSTALL_UNITDIR)'#013#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#013#010+
  '        $(INSTALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'ifneq ($(wildcard $(LIB_FULLNAME)),)'#013#010+
  '        $(MKDIR) $(INSTALL_LIBDIR)'#013#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LI','BDIR)'#013#010+
  'ifdef inUnix'#013#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_LIBDIR)/$(LIB_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FILES'#013#010+
  '        $(MKDIR) $(INSTALL_DATADIR)'#013#010+
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_sourceins','tall: distclean'#013#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#013#010+
  '        $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#013#010+
  #013#010+
  'fpc_exampleinstall: $(addsuffix _distclean,$(TARGET_EXAMPLEDIRS))'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#013#010+
  'e','ndif'#013#010+
  'ifdef EXAMPLESOURCEFILES'#013#010+
  '        $(COPY) $(EXAMPLESOURCEFILES) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_EX'+
  'AMPLEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '[distinstallrules]'#013#010+
  '####','################################################################'+
  '#'#013#010+
  '# Dist Install'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_distinstall'#013#010+
  #013#010+
  'fpc_distinstall: install exampleinstall'#013#010+
  #013#010+
  #013#010+
  '[zipinstallr','ules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Zip'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#013#010+
  #013#010+
  '# Temporary p','ath to pack a file, can only use a single deep'#013#010+
  '# subdir, because the deltree can'#039't see the whole tree to remove'#013+
  #010+
  'ifndef PACKDIR'#013#010+
  'ifndef inUnix'#013#010+
  'PACKDIR=$(BASEDIR)/../fpc-pack'#013#010+
  'else'#013#010+
  'PACKDIR=/tmp/fpc-pack'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Maybe create def','ault zipname from packagename'#013#010+
  'ifndef ZIPNAME'#013#010+
  'ifdef DIST_ZIPNAME'#013#010+
  'ZIPNAME=$(DIST_ZIPNAME)'#013#010+
  'else'#013#010+
  'ZIPNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef FULLZIPNAME'#013#010+
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  '# ZipTa','rget'#013#010+
  'ifndef ZIPTARGET'#013#010+
  'ifdef DIST_ZIPTARGET'#013#010+
  'ZIPTARGET=DIST_ZIPTARGET'#013#010+
  'else'#013#010+
  'ZIPTARGET=install'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use tar by default under linux'#013#010+
  'ifndef USEZIP'#013#010+
  'ifdef inUnix'#013#010+
  'USETAR=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use a wrapper script by default for ','Os/2'#013#010+
  'ifndef inUnix'#013#010+
  'USEZIPWRAPPER=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# We need to be able to run in the current OS so fix'#013#010+
  '# the path separator'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  'ZIPPATHSEP=$(PATHSEP)'#013#010+
  'ZIPWRAPPER=$(subst /,$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT))'#013+
  #010+
  'el','se'#013#010+
  'ZIPPATHSEP=/'#013#010+
  'endif'#013#010+
  #013#010+
  '# Create commands to create the zip/tar file'#013#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#013#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#013#010+
  'ifdef USETAR'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)','$(TAREXT)'#013#010+
  'ZIPCMD_ZIP:=$(TARPROG) c$(TAROPT)f $(ZIPDESTFILE) *'#013#010+
  'else'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#013#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
=======
  ';'#013#010+
  #013#010+
  '[defines]'#013#010+
  '#####################################################################'#013+
=======
  'ifneq ($(ECHOREDIR),echo)'#010+
  '        $(ECHOREDIR) -e "','$(subst \,\\,$(ZIPCMD_CDPACK))" > $(ZIPWRAP'+
  'PER)'#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPER)'+
>>>>>>> origin/fixes_2.4
  #010+
  '# Misc defines to be used by anyone'#013#010+
  '#############################################','#######################'+
  '#'#013#010+
  #013#010+
  '# OS categories'#013#010+
  'BSDs = freebsd netbsd openbsd darwin'#013#010+
  'UNIXs = linux $(BSDs) solaris qnx haiku'#013#010+
  'LIMIT83fs = go32v2 os2 emx watcom'#013#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#013#010+
  #013#010+
  '#Empty target for rules that always s','hould run. Needed if'#013#010+
  '#the target is non-phoney, and there is non-phony prereqisites.'#013#010+
  '#Then add FORCE as an prerequisite'#013#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#013#010+
  'FORCE:'#013#010+
  '.PHONY: FORCE'#013#010+
  #013#010+
  '[osdetect]'#013#010+
  '#############','#######################################################'+
  '#'#013#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#013+
  #010+
  '# define inUnix when running under Unix like environment'#013#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,C','ygwin)'#013#010+
  '# define inWinNT when running under WinNT'#013#010+
  '# define inOS2 when running under OS/2'#013#010+
  '# define inCygwin when running under Cygwin32'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# We need only / in the path ','also remove the current dir,'#013#010+
  '# also remove trailing /'#039's'#013#010+
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#013#010+
  #013#010+
  '# Detect unix'#013#010+
  '# Darwin is handled specially'#013#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#013#010+
  'inUnix=1 #darwin'#013#010+
  'SEARCHPATH:=$(filter-','out .,$(subst :, ,$(PATH)))'#013#010+
  'else'#013#010+
  '# Determine if we'#039've a unix searchpath by looking for a ;'#013#010+
  '# that normally doesn'#039't exists in the unix PATH var.'#013#010+
  'ifeq ($(findstring ;,$(PATH)),)'#013#010+
  'inUnix=1'#013#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#013,#010+
  'else'#013#010+
  'SEARCHPATH:=$(subst ;, ,$(PATH))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add path were make is located'#013#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#013#010+
  #013#010+
  '# Search for PWD'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#013#010+
  'ifeq ($(P','WD),)'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013#010+
  '$(error You need the GNU utils package to use this Makefile)'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT='#013#010+
  'endif'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT=.ex','e'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect NT - NT sets OS to Windows_NT'#013#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#013#010+
  'ifndef inUnix'#013#010+
  'ifeq ($(OS),Windows_NT)'#013#010+
  'inWinNT=1'#013#010+
  'else'#013#010+
  'ifdef OS2_SHELL'#013#010+
  'inOS2=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifneq ($(findstring cygdrive,$(PATH)),)'#013#010,
  'inCygWin=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# The extension of batch files / scripts'#013#010+
  'ifdef inUnix'#013#010+
  'SRCBATCHEXT=.sh'#013#010+
  'else'#013#010+
  'ifdef inOS2'#013#010+
  'SRCBATCHEXT=.cmd'#013#010+
  'else'#013#010+
  'SRCBATCHEXT=.bat'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef COMSPEC'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(OSNeedsComsp','ecToRunBatch)),)'#013#010+
  'ifndef RUNBATCH'#013#010+
  'RUNBATCH=$(COMSPEC) /C'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#013#010+
  '# at the end of a line'#013#010+
  'ifdef inUnix'#013#010+
  'PATHSEP=/'#013#010+
  'else'#013#010+
  'PATHSEP:=$(subst /,\,/)'#013#010+
  '# ','cygwin bash or sh can not handle backslashs'#013#010+
  'ifdef inCygWin'#013#010+
  'PATHSEP=/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Base dir'#013#010+
  'ifdef PWD'#013#010+
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#013#010+
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#013#010+
  'ifdef inCygWin'#013#010+
  'ifneq ($(findstring',' /cygdrive/,$(BASEDIR)),)'#013#010+
  'BASENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#013#010+
  'BASEDRIVE:=$(firstword $(subst /, ,$(BASENODIR)))'#013#010+
  'BASEDIR:=$(subst /cygdrive/$(BASEDRIVE)/,$(BASEDRIVE):/,$(BASEDIR))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'BASEDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  '# Ech','o is an internal command under OS/2 (and others), so it'#039's a'+
  'lways found'#013#010+
  'ifdef inOS2'#013#010+
  'ifndef ECHO'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH))'+
  '))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /echo$(SRCEXEE','XT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO=echo'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export ECHO'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdetect]'#013#010+
  '##################################################################','##'+
  '#'#013#010+
  '# FPC Binary and Version Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Compatibility with old makefiles'#013#010+
  'ifndef FPC'#013#010+
  'ifdef PP'#013#010+
  'FPC=$(PP)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try to detect the ppcXXX file to use by',' using "fpc -PB" option'#013+
  #010+
  '# to query for the default ppcXXX the fpc executable tries. When'#013#010+
  '# fpc is not found use ppc386 by default. Also when fpc -PB gives'#013#010+
  '# an error ppc386 will be used.'#013#010+
  'ifndef FPC'#013#010+
  '# check if fpc exists'#013#010+
  'FPCPROG:=$(str','ip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifneq ($(FPCPROG),)'#013#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#013#010+
  'ifneq ($(CPU_TARGET),)'#013#010+
  'FPC:=$(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#013#010+
  'else'#013#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#013#010+
  'endif'#013#010+
  '# Older fpc',' executables didn'#039't support it and return'#013#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#013#010+
  'ifneq ($(findstring Error,$(FPC)),)'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#013#010+
  'e','lse'#013#010+
  '# if the cross compiler is not found, fall back to fpc'#013#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#013#010+
  'FPC:=$(firstword $(FPCPROG))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# fpc binary not found, fallback to ppc386'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(ad','dsuffix /ppc386$(SRC'+
  'EXEEXT),$(SEARCHPATH)))))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Get a clean executable name'#013#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#013#010+
  'override FPC:=$(subst \,/,$(FPC))$(SRCEXEEXT)'#013#010+
  #013#010+
  '# Try to find the binary direct first, otherwise in ','the path,'#013#010+
  '# if not found give an error'#013#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  '$(error Compiler $(FPC) not found)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010,
  '# For 1.1 and up we can use a single compiler call to retrieve'#013#010+
  '# all needed information'#013#010+
  'ifndef FPC_COMPILERINFO'#013#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# FPC version'#013#010+
  'ifndef FPC_VERSION'#013#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPIL','ERINFO))'#013#010+
  'endif'#013#010+
  #013#010+
  'export FPC FPC_VERSION FPC_COMPILERINFO'#013#010+
  '# CHECKDEPEND should not be exported'#013#010+
  '# This should limit multiple checks'#013#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#013#010+
  #013#010+
  '#################################################################','###'+
  '#'#013#010+
  '# FPC Target Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Fall back to default values if needed'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'ifdef CPU_TARGET_DEFAULT'#013#010+
  'CPU_TARGET=$(CPU_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifn','def OS_TARGET'#013#010+
  'ifdef OS_TARGET_DEFAULT'#013#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.0.x we need to use extra calls to retrieve all info'#013#010+
  'ifneq ($(words $(FPC_COMPILERINFO)),5)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSP)'#013#010+
  'FPC_COMPILE','RINFO+=$(shell $(FPC) -iTP)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSO)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Retrieve Target/Source CPU and Target/Source OS'#013#010+
  'ifndef CPU_SOURCE'#013#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'if','ndef CPU_TARGET'#013#010+
  'CPU_TARGET:=$(word 3,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_SOURCE'#013#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013,#010+
  'FULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifeq ($(CPU_TARGET),armeb)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-Cb'#013#010+
  'else'#013#010+
  'ifeq ($(CPU_TARGET),armel)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-CaEABI'#013#010+
  'else'#013#010+
  'ARCH=$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Full name of the target',', including CPU and OS. For OSs limited'#013+
  #010+
  '# to 8.3 we only use the target OS'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'TARGETSUFFIX=$(OS_TARGET)'#013#010+
  'SOURCESUFFIX=$(OS_SOURCE)'#013#010+
  'else'#013#010+
  'TARGETSUFFIX=$(FULL_TARGET)'#013#010+
  'SOURCESUFFIX=$(FULL_SOUR','CE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compile flag'#013#010+
  'ifneq ($(FULL_TARGET),$(FULL_SOURCE))'#013#010+
  'CROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the Makefile supports this target, but not'#013#010+
  '# when the make target is to rebuild the makefile'#013#010+
  'ifeq ($(findstring makefile,$(MAKE','CMDGOALS)),)'#013#010+
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),)'#013#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Detect BSD, since BSD uses a slightly different directory hier','arch'+
  'y.'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#013#010+
  'BSDhier=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect Linux, will also use its own directory hierarchy.'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'linuxHier=1'#013#010+
  'endif'#013#010+
  #013#010+
  'export OS_TARGET OS_SOURCE ARCH CPU_TARGET CPU_SOURCE FULL','_TARGET FU'+
  'LL_SOURCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#013#010+
  #013#010+
  #013#010+
  '[fpcdircheckenv]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPCDIR Setting'#013#010+
  '#####################################################################',
  #013#010+
  #013#010+
  '# Test FPCDIR to look if the RTL dir exists'#013#010+
  'ifdef FPCDIR'#013#010+
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default FPC','DIR from Makefile.fpc'#013#010+
  'ifdef DEFAULT_FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdirdetect]'#013#010+
  '# Det','ect FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'ifdef inUnix'#013#010+
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#013#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#013#010+
  'override FPCDIR=/usr/lib/fpc/$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR:=$(subst /$(FPC),,$(firstwor','d $(strip $(wildcard $(a'+
  'ddsuffix /$(FPC),$(SEARCHPATH))))))'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'overrid','e FPCDIR:=$(BASEDIR)'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=c:/pp'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross binaries dir'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(FPCDIR)/bin/$(TARGETSUFFIX))'#013#010+
  'endif',#013#010+
  #013#010+
  '# Default binutils prefix for cross compile when the'#013#010+
  '# crossbindir is not set (except for Darwin)'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'ifeq ($(OS_SOURCE),darwin)'#013#010+
  'DARWIN2DARWIN=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef BINUTILSPREFIX',#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifndef DARWIN2DARWIN'#013#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try first the full target name, otherwise try only'#013#010+
  '# the OS for backwards compatibility'#013#010+
  'UNITSDIR:=','$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#013#010+
  'ifeq ($(UNITSDIR),)'#013#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(OS_TARGET))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Packages dir'#013#010+
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages $(FPCDIR)/package'+
  's/base $(FPCDIR)/packages/e','xtra)'#013#010+
  #013#010+
  #013#010+
  '[shelltools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Shell tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Echo that can redir (must be able run in the default O','S shell)'#013+
  #010+
  'ifndef ECHOREDIR'#013#010+
  'ifndef inUnix'#013#010+
  'ECHOREDIR=echo'#013#010+
  'else'#013#010+
  'ECHOREDIR=$(ECHO)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To copy pograms'#013#010+
  'ifndef COPY'#013#010+
  'COPY:=$(CPPROG) -fp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef COPYTREE'#013#010+
  'COPYTREE:=$(CPPROG) -Rfp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cop','y a whole tree'#013#010+
  'ifndef MKDIRTREE'#013#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#013#010+
  'endif'#013#010+
  #013#010+
  '# To move pograms'#013#010+
  'ifndef MOVE'#013#010+
  'MOVE:=$(MVPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check delete program'#013#010+
  'ifndef DEL'#013#010+
  'DEL:=$(RMPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check deltree program'#013#010+
  'ifndef DELTREE'#013#010+
  'DEL','TREE:=$(RMPROG) -rf'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install files'#013#010+
  'ifndef INSTALL'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALL:=$(GINSTALL) -c -m 644'#013#010+
  'else'#013#010+
  'INSTALL:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install programs'#013#010+
  'ifndef INSTALLEXE'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALLEXE:=$(GINSTALL) -c -m 75','5'#013#010+
  'else'#013#010+
  'INSTALLEXE:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To make a directory.'#013#010+
  'ifndef MKDIR'#013#010+
  'MKDIR:=$(GINSTALL) -m 755 -d'#013#010+
  'endif'#013#010+
  #013#010+
  'export ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE MKD'+
  'IR'#013#010+
  #013#010+
  #013#010+
  '[defaultdirs]'#013#010+
  '########################','############################################'+
  '#'#013#010+
  '# Default Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Units dir'#013#010+
  'ifdef REQUIRE_UNITSDIR'#013#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Units dir',#013#010+
  'ifdef REQUIRE_PACKAGESDIR'#013#010+
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#013#010+
  '# When zipping use the target os default, when normal install then'#013+
  #010+
  '# use the source os as default'#013#010+
  'ifdef ','ZIPINSTALL'#013#010+
  '# Zipinstall'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# Normal install'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# When install prefix is not set try to use prefix'#013,#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef PREFIX'#013#010+
  'INSTALL_PREFIX=$(PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the prefix directory where to install everything'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_PREFIX=/usr/local'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTAL','L_BASEDIR:=/pp'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export INSTALL_PREFIX'#013#010+
  #013#010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#013#010+
  '# used recursively for all subdirs'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'export INSTALL_FPCSUBD','IR'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to place the resulting zip files'#013#010+
  'ifndef DIST_DESTDIR'#013#010+
  'DIST_DESTDIR:=$(BASEDIR)'#013#010+
  'endif'#013#010+
  'export DIST_DESTDIR'#013#010+
  #013#010+
  '# EXE/PPU Target directories'#013#010+
  'ifndef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef PACKAGEDIR_MAIN'#013#010+
  'COMPILER_UNITTARGETDIR=$','(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#013#010+
  'else'#013#010+
  'COMPILER_UNITTARGETDIR=units/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef COMPILER_TARGETDIR'#013#010+
  'COMPILER_TARGETDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# ','Install Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# set the base directory where to install everything'#013#010+
  'ifndef INSTALL_BASEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL','_PREFIX)/lib/fpc/$(FPC_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the binaries'#013#010+
  'ifndef INSTALL_BINDIR'#013#010+
  'ifdef ','UNIXHier'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/bin'#013#010+
  '# for FPC packages install the binaries under their target subdir'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifdef CROSSINSTALL'#013#010+
  'INSTALL_BIND','IR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the units.'#013#010+
  'i','fndef INSTALL_UNITDIR'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef PACKAGE_NAME'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install shared libraries'#013,#010+
  'ifndef INSTALL_LIBDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#013#010+
  'else'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the source files will be stored'#013#010+
  'ifndef INSTALL_SOURCEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'SRCPR','EFIXDIR=share/src'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'SRCPREFIXDIR=src'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INSTALL_FPC','SUBDIR)/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef',' INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$(PA'+
  'CKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_B','ASEDIR)/source'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the doc files will be stored'#013#010+
  'ifndef INSTALL_DOCDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'DOCPREFIXDIR=doc'#013#010+
  'endif'#013#010+
  'endif'#013#010,
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/fpc-$(FPC_VERSION)/$'+
  '(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACKAGE_NAME)-$(PA'+
  'CKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAG','E'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install the examples, under linux we use the doc dir'#013#010+
  '# because the copytree command will create a su','bdir itself'#013#010+
  'ifndef INSTALL_EXAMPLEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL','_DOCDIR)/examples'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010,
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACK','AGE'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the some extra (data)files will be stored'#013#010+
  'ifndef INSTALL_DATADIR'#013#010+
  'INSTALL_DATADIR=$','(INSTALL_BASEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef INSTALL_SHAREDDIR'#013#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#013#010+
  'endif'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# Cross compile dirs'#013#010+
  '###########################################','#########################'+
  '#'#013#010+
  #013#010+
  'ifdef CROSSCOMPILE'#013#010+
  '# Directory where the cross compile tools are stored.'#013#010+
  '# First check if they are available in FPCDIR. If no targets/ subdir'#013+
  #010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#013#010+
  'ifndef CR','OSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/cross/$(TARGETSUFFIX)/bin/$'+
  '(FULL_SOURCE))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'CROSSBINDIR='#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[dirlibc]'#013,#010+
  '# On linux, try to find where libgcc.a is.'#013#010+
  'ifeq ($(OS_SOURCE),linux)'#013#010+
  #013#010+
  '# Amd64 to i386?'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'ifneq ($(findstring x86_64,$(shell uname -a)),)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname ','`gcc -m32 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try cross gcc'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'C','ROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSPREFIX)gcc$(SRCEX'+
  'EEXT),$(SEARCHPATH))))'#013#010+
  'ifneq ($(CROSSGCC),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Other libraries from ld.so.conf'#013#010+
  'ifndef OT','HERLIBDIR'#013#010+
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.conf | awk '#039'{ ORS=" '+
  '"; print $1 }'#039')'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef inUnix'#013#010+
  'ifeq ($(OS_SOURCE),netbsd)'#013#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#013#010+
  'endif'#013#010+
  'export GCCLIBDIR OTHERLIB'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[extensions]'#013#010+
  '##','##################################################################'+
  '#'#013#010+
  '# Default extensions'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Default needed extensions (Go32v2,Linux)'#013#010+
  'BATCHEXT=.bat'#013#010+
  'LOADEREXT=.as'#013#010+
  'EX','EEXT=.exe'#013#010+
  'PPLEXT=.ppl'#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHAREDLIBPREFIX=libfp'#013#010+
  'STATICLIBPREFIX=libp'#013#010+
  'IMPORTLIBPREFIX=libimp'#013#010+
  'RSTEXT=.rst'#013#010+
  '#DEBUGSYMEXT #for debugger symbol files, define only',' for targets whi'+
  'ch has this'#013#010+
  #013#010+
  '# 1.0.x has target specific extensions for ppu files and objects'#013#010+
  'ifeq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  '# short version for 1.1 and above - no target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TARGET),go','32v1)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'OEXT=.obj'#013#010+
  'ASMEXT=.asm'#013#010+
  'SHAREDLIBE','XT=.dll'#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSU','FFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq',' ($(OS_TARGET),win32)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# EMX'#013#010+
  'ifeq ($(OS_','TARGET),emx)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=emx'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# MorphOS'#013#010,
  'ifeq ($(OS_TARGET),morphos)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=mos'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif',#013#010+
  #013#010+
  '# Haiku'#013#010+
  'ifeq ($(OS_TARGET),haiku)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=hai'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSU','FFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware clib'#013#010+
  'ifeq ($(OS_TARGET),netware)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPOR','TLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Darwin'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSH','AREDLIB=1'#013#010+
  'SHORTSUFFIX=dwn'#013#010+
  'endif'#013#010+
  #013#010+
  '# gba'#013#010+
  'ifeq ($(OS_TARGET),gba)'#013#010+
  'EXEEXT=.gba'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=gba'#013#010+
  'endif'#013#010+
  #013#010+
  '# Symbian OS'#013#010+
  'ifeq ($(OS_TARGET),symbian)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=symbian'#013#010+
  'endif'#013#010+
  #013#010+
  '# NativeNT'#013#010+
  'ifeq ($(OS','_TARGET),NativeNT)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=nativent'#013#010+
  'endif'#013#010+
  #013#010+
  '# wii'#013#010+
  'ifeq ($(OS_TARGET),wii)'#013#010+
  'EXEEXT=.dol'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=wii'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  '# long version for 1.0.x - target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(','OS_TARGET),go32v1)'#013#010+
  'PPUEXT=.pp1'#013#010+
  'OEXT=.o1'#013#010+
  'ASMEXT=.s1'#013#010+
  'SMARTEXT=.sl1'#013#010+
  'STATICLIBEXT=.a1'#013#010+
  'SHAREDLIBEXT=.so1'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013,#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010,
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BAT','CHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'PPUEXT=.ppw'#013#010+
  'OEXT=.ow'#013#010+
  'ASMEXT=.sw'#013#010+
  'SMARTEXT=.slw'#013#010+
  'STATICLIBEXT=.aw'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os','2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'PPUEXT=.ppo'#013#010+
  'ASMEXT=.so2'#013#010+
  'OEXT=.oo2'#013#010+
  'AOUTEXT=.out'#013#010+
  'SMARTEXT=.sl2'#013#010+
  'STATICLIBPREFIX='#013#010+
  'STATICLIBEXT=.ao2'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=a','ta'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEX','T=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware'#013#010+
  'ifeq ($(OS_T','ARGET),netware)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'STATICLIBPREFIX='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'ST','ATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '#end of target specific settings'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 8.3 limited OS'#039's the short suffixes'#013#010+
  '# Otherwise use the full source/target names'#013#010+
  'ifneq ($(findstri','ng $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)'#013#010+
  'ZIPSUFFIX=$(SHORTSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'ZIPSOURCESUFFIX=src'#013#010+
  'ZIPEXAMPLESUFFIX=exm'#013#010+
  'else'#013#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#013#010+
  'ZIPSOURCESUFFIX=.source'#013#010+
  'ZIPEXAMPLESUFFIX=.example','s'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#013#010+
  'ZIPCROSSPREFIX=$(TARGETSUFFIX)-'#013#010+
  'else'#013#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[defaulttools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# ','Default Tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Names of the binutils tools'#013#010+
  'ASNAME=$(BINUTILSPREFIX)as'#013#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#013#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#013#010+
  'RCNAME=$(BINUTILSPREFIX)rc'#013#010+
  'ifneq ($(','findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'ASNAME=asw'#013#010+
  'LDNAME=ldw'#013#010+
  'ARNAME=arw'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# assembler, redefine it if cross compiling'#013#010+
  'ifndef ASPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ASPROG=$(CROSSBI','NDIR)/$(ASNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ASPROG=$(ASNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# linker, but probably not used'#013#010+
  'ifndef LDPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'LDPROG=$(CROSSBINDIR)/$(LDNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'LDPROG=$(LDNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Resource compiler',#013#010+
  'ifndef RCPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'RCPROG=$(CROSSBINDIR)/$(RCNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'RCPROG=$(RCNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Archiver'#013#010+
  'ifndef ARPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ARPROG=$(CROSSBINDIR)/$(ARNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ARPROG=$(ARNAME)'#013#010+
  'endi','f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Override defaults'#013#010+
  'AS=$(ASPROG)'#013#010+
  'LD=$(LDPROG)'#013#010+
  'RC=$(RCPROG)'#013#010+
  'AR=$(ARPROG)'#013#010+
  #013#010+
  '# ppas.bat / ppas.sh'#013#010+
  'PPAS=ppas$(SRCBATCHEXT)'#013#010+
  #013#010+
  '# ldconfig to rebuild .so cache'#013#010+
  'ifdef inUnix'#013#010+
  'LDCONFIG=ldconfig'#013#010+
  'else'#013#010+
  'LDCONFIG='#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef DA','TE'#013#010+
  'DATESTR:=$(shell $(DATE) +%Y%m%d)'#013#010+
  'else'#013#010+
  'DATESTR='#013#010+
  'endif'#013#010+
  #013#010+
  '# Look if UPX is found for go32v2 and win32. We can'#039't use $UPX bec'+
  'ause'#013#010+
  '# upx uses that one itself (PFV)'#013#010+
  'ifndef UPXPROG'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifeq ($','(OS_TARGET),win32)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifdef UPXPROG'#013#010+
  'UPXPROG:=$(strip $(wildcard $(addsuffix /upx$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(UPXPROG),)'#013#010+
  'UPXPROG='#013#010+
  'else'#013#010+
  'UPXPROG:=$(firstword $(UPXPROG))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'UPXPROG='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'expo','rt UPXPROG'#013#010+
  #013#010+
  '# Zip options'#013#010+
  'ZIPOPT=-9'#013#010+
  'ZIPEXT=.zip'#013#010+
  #013#010+
  '# Tar options'#013#010+
  'ifeq ($(USETAR),bz2)'#013#010+
  'TAROPT=vj'#013#010+
  'TAREXT=.tar.bz2'#013#010+
  'else'#013#010+
  'TAROPT=vz'#013#010+
  'TAREXT=.tar.gz'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_begin]'#013#010+
  '###########################################################','#########'+
  '#'#013#010+
  '# Compiler Command Line'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#013+
  #010+
  '# compiler this needs to be turned off'#013#010+
  'ifndef NOCPUDEF'#013#010+
  'over','ride FPCOPTDEF=$(ARCH)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Load commandline OPT and add target and unit dir to be sure'#013#010+
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#013#010+
  'override FPCOPT+=-T$(OS_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#013#010+
  'override FPCOPT+=-P$(ARCH)'#013#010,
  'endif'#013#010+
  #013#010+
  'ifeq ($(OS_SOURCE),openbsd)'#013#010+
  'override FPCOPT+=-FD$(NEW_BINUTILS_PATH)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef CROSSBOOTSTRAP'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-XP$(BINUTILSPREFIX)'#013#010+
  'endif'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-Xr$(RLIN','KPATH)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# User dirs should be first, so they are looked at first'#013#010+
  'ifdef UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef LIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#013#010+
  'endif'#013#010+
  'ifdef OBJDIR'#013#010+
  'override F','PCOPT+=$(addprefix -Fo,$(OBJDIR))'#013#010+
  'endif'#013#010+
  'ifdef INCDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(INCDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking'#013#010+
  'ifdef LINKSMART'#013#010+
  'override FPCOPT+=-XX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking creation'#013#010+
  'ifdef CREATESMART'#013#010+
  'override FPCOPT+=-CX'#013#010,
  'endif'#013#010+
  #013#010+
  '# Debug'#013#010+
  'ifdef DEBUG'#013#010+
  'override FPCOPT+=-gl'#013#010+
  'override FPCOPTDEF+=DEBUG'#013#010+
  'endif'#013#010+
  #013#010+
  '# Release mode'#013#010+
  '# (strip, optimize and don'#039't load fpc.cfg)'#013#010+
  '# fpc 2.1 has -O2 for all targets'#013#010+
  'ifdef RELEASE'#013#010+
  'ifneq ($(findstring 2.0.,$(FPC_VERSION)),',')'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'FPCCPUOPT:=-OG2p3'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc)'#013#010+
  'FPCCPUOPT:=-O1r'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'FPCCPUOPT:=-O2'#013#010+
  'endif'#013#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#013#010+
  'override FPCOPTDEF+=RELEASE'#013#010+
  'endif'#013#010+
  #013#010+
  '# Strip'#013#010+
  'ifdef STRIP'#013,#010+
  'override FPCOPT+=-Xs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Optimizer'#013#010+
  'ifdef OPTIMIZE'#013#010+
  'override FPCOPT+=-O2'#013#010+
  'endif'#013#010+
  #013#010+
  '# Verbose settings (warning,note,info)'#013#010+
  'ifdef VERBOSE'#013#010+
  'override FPCOPT+=-vwni'#013#010+
  'endif'#013#010+
  #013#010+
  '# Needed compiler options'#013#010+
  'ifdef COMPILER_OPTIONS'#013#010+
  'override ','FPCOPT+=$(COMPILER_OPTIONS)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_LIBRARYDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_OBJECTDIR'#013#010,
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_INCLUDEDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compiler utils'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'override FPCOPT+=-FD$(CROSSBINDIR)'#013#010+
  'end','if'#013#010+
  #013#010+
  '# Target dirs and the prefix to use for clean/install'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#013#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#013#010+
  'override TARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR',')/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Unit target dir (default is units/<cpu>-<os>/'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#013#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#013#010+
  'override UNITTARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override UNITTARGETDI','RPREFIX=$(COMPILER_UNITTARGETDIR)/'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#013#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CREATESHARED'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  #013#010+
  'i','feq ($(CPU_TARGET),i386)'#013#010+
  'override FPCOPT+=-Aas'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  #013#010+
  '# create always pic'#039'ed code on x86_64'#013#010+
  'ifeq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'ifeq ($(CPU_TARGET),x86_64)'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  'endif'#013#010+
  'endif'#013,#010+
  'endif'#013#010+
  #013#010+
  'ifdef LINKSHARED'#013#010+
  '#!!! deactived for testing override FPCOPT+=-XD'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_libc]'#013#010+
  '# Add GCC lib path if asked'#013#010+
  'ifdef GCCLIBDIR'#013#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#013#010+
  'endif'#013#010+
  'ifdef OTHERLIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -','Fl,$(OTHERLIBDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_end]'#013#010+
  '# Add commandline options last so they can override'#013#010+
  'ifdef OPT'#013#010+
  'override FPCOPT+=$(OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add defines from FPCOPTDEF to FPCOPT'#013#010+
  'ifdef FPCOPTDEF'#013#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOP','TDEF))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Was a config file specified ?'#013#010+
  'ifdef CFGFILE'#013#010+
  'override FPCOPT+=@$(CFGFILE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use the environment to pass commandline options?'#013#010+
  'ifdef USEENV'#013#010+
  'override FPCEXTCMD:=$(FPCOPT)'#013#010+
  'override FPCOPT:=!FPCEXTCMD'#013#010+
  'export FPC','EXTCMD'#013#010+
  'endif'#013#010+
  #013#010+
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013#010+
  'override AFULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifneq ($(AFULL_TARGET),$(AFULL_SOURCE))'#013#010+
  'override ACROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef ACROSSCOMPILE'#013#010+
  'override FPCOPT+=$(CROSS','OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Compiler commandline'#013#010+
  'override COMPILER:=$(FPC) $(FPCOPT)'#013#010+
  #013#010+
  '# also call ppas if with command option -s'#013#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#013#010+
  'ifeq (,$(findstring -s ,$(COMPILER)))'#013#010+
  'EXECPPAS='#013#010+
  'else'#013#010+
  'ifeq ','($(FULL_SOURCE),$(FULL_TARGET))'#013#010+
  'ifdef RUNBATCH'#013#010+
  'EXECPPAS:=@$(RUNBATCH) $(PPAS)'#013#010+
  'else'#013#010+
  'EXECPPAS:=@$(PPAS)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[loaderrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Loaders'#013#010+
  '###########','#########################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_loaders'#013#010+
  #013#010+
  'ifneq ($(TARGET_LOADERS),)'#013#010+
  'override ALLTARGET+=fpc_loaders'#013#010+
  'override CLEANTARGET+=fpc_loaders_clean'#013#010+
  'override INSTALLTARGET+=fpc_loaders_install'#013#010+
  #013#010+
  'overri','de LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#013#010+
  'endif'#013#010+
  #013#010+
  '%$(OEXT): %$(LOADEREXT)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#013#010+
  'else'#013#010+
  '        $(AS) -o $*$(OEXT) $<'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders: $(C','OMPILER_UNITTARGETDIR) $(LOADEROFILES)'#013#010+
  #013#010+
  'fpc_loaders_clean:'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#013#010+
  'else'#013#010+
  '        -$(DEL) $(LOADEROFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders_install:'#013#010+
  '     ','   $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFIL'+
  'ES)) $(INSTALL_UNITDIR)'#013#010+
  'else'#013#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[unitrules]',#013#010+
  '#####################################################################'#013+
  #010+
  '# Units'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_units'#013#010+
  #013#010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#013#010+
  'override ALLTAR','GET+=fpc_units'#013#010+
  #013#010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_UNITS))'#013#010+
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#013#010+
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'override CLEANP','PUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#013#010+
  #013#010+
  #013#010+
  '[exerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Exes'#013#010+
  '################################','####################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_exes'#013#010+
  #013#010+
  '# Programs are not needed for a cross installation'#013#010+
  'ifndef CROSSINSTALL'#013#010+
  'ifneq ($(TARGET_PROGRAMS),)'#013#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#013#010+
  'override EXEOFIL','ES:=$(addsuffix $(OEXT),$(TARGET_PROGRAMS)) $(addpre'+
  'fix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
  'PROGRAMS)))'#013#010+
  #013#010+
  'override ALLTARGET+=fpc_exes'#013#010+
  'ove','rride INSTALLEXEFILES+=$(EXEFILES)'#013#010+
  'override CLEANEXEFILES+=$(EXEFILES) $(EXEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(add','suffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES)'#013+
  #010+
  #013#010+
  #013#010+
  '[rstrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Resource strings'#013#010,
  '#####################################################################'#013+
  #010+
  #013#010+
  'ifdef TARGET_RSTS'#013#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_RSTS))'#013#010+
  #013#010+
  'override CLEANRSTFILES+=$(RSTFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[examplerules]'#013#010+
  '#########################','###########################################'+
  '#'#013#010+
  '# Examples'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_examples'#013#010+
  #013#010+
  'ifneq ($(TARGET_EXAMPLES),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $','(addsuffix .pp,$(TARGET_EXA'+
  'MPLES)) $(addsuffix .pas,$(TARGET_EXAMPLES)) $(addsuffix .lpr,$(TARGET'+
  '_EXAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#013#010+
  'override EXAMPLEFILES:=$(addsuffix $(EXEEXT),$(TARGET_EXAMPLES))'#013#010+
  'override EXAMPLEOFILES:=$(','addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(ad'+
  'dprefix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPL'+
  'ES))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TAR'+
  'GET_EXAMPLES)))'#013#010+
  #013#010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES)',' $(EXAMPLEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifneq ($(TARGE','T_EXAMPLEDIRS),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_examples: all $(EXAMPLEFILES) $(addsuffix _all,$(TARGET_EXAMPLEDIR'+
  'S))'#013#010+
  #013#010+
  #013#010+
  '[compilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# General compile rules'#013#010+
  '#######','#############################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_all fpc_smart fpc_debug fpc_release fpc_shared'#013#010+
  #013#010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#013#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMADE)'#013#010+
  #013#010+
  'fpc_all: $(FPCMADE)'#013,#010+
  #013#010+
  'fpc_smart:'#013#010+
  '        $(MAKE) all LINKSMART=1 CREATESMART=1'#013#010+
  #013#010+
  'fpc_debug:'#013#010+
  '        $(MAKE) all DEBUG=1'#013#010+
  #013#010+
  'fpc_release:'#013#010+
  '        $(MAKE) all RELEASE=1'#013#010+
  #013#010+
  '# General compile rules, available for both possible .pp and .pas exte'+
  'nsions'#013#010+
  #013#010+
  '.SUFFI','XES: $(EXEEXT) $(PPUEXT) $(OEXT) .pas .lpr .dpr .pp .rc .res'#013+
  #010+
  #013#010+
  '$(COMPILER_UNITTARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '$(COMPILER_TARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pp'#013#010+
  '        $(','COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXE','EXT): %.lpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.dpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%.res: %.rc'#013#010+
  '        windres -i $< -o $@'#013#010+
  #013#010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#013#010+
  'vpath %.pp $(COMPILER_','SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.pas $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.inc $(COMPILER_INCLUDED','IR)'#013#010+
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  'vpath %$(PPUEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '[sharedrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Library'#013#010+
  '##################################################','##################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_shared'#013#010+
  #013#010+
  'override INSTALLTARGET+=fpc_shared_install'#013#010+
  #013#010+
  'ifndef SHARED_LIBVERSION'#013#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_LIBNAME'#013#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_FU','LLNAME'#013#010+
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED_LIBNAME)-$(SHARED_LIBVERSIO'+
  'N)$(SHAREDLIBEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default sharedlib units are all unit objects'#013#010+
  'ifndef SHARED_LIBUNITS'#013#010+
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#013#010+
  'over','ride SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),$(SHARED'+
  '_LIBUNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared:'#013#010+
  'ifdef HASSHAREDLIB'#013#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  '        $(PPUMOVE) -q $(SHARED_','LIBUNITS) -i$(COMPILER_UNITTARGETDIR)'+
  ' -o$(SHARED_FULLNAME) -d$(COMPILER_UNITTARGETDIR)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '        @$(ECHO) Shared Libraries not supported'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared_install:'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  'ifneq ($(SHARED_LIBUNITS),)'#013#010+
  'ifne','q ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME)),)'#013+
  #010+
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[installrules]'#013#010+
  '####################################################','################'+
  '#'#013#010+
  '# Install rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#013#010+
  #013#010+
  'ifdef INSTALL_UNITS'#013#010+
  'override INSTALLPPUFILES+=$(addsuffix $(PPUEXT),$(','INSTALL_UNITS))'#013+
  #010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALL_BUILDUNIT'#013#010+
  'override INSTALLPPUFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT),$'+
  '(INSTALLPPUFILES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPP','U'+
  'FILES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEX'+
  'T),$(INSTALLPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#013#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#013#010+
  'override INSTALLPPUFILES:','=$(addprefix $(UNITTARGETDIRPREFIX),$(notdi'+
  'r $(INSTALLPPUFILES)))'#013#010+
  'override INSTALLPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPR'+
  'EFIX),$(notdir $(INSTALLPPULINKFILES))))'#013#010+
  'endif'#013#010+
  '# Implicitly install Package.fpc'#013#010+
  'override INSTALL_CRE','ATEPACKAGEFPC=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  'ifneq ($(TARGETDIRPREFIX),)'#013#010+
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_install: all $(INSTALLTARGET)'#013#010+
  'ifdef INSTALLEXEFILES'#013#010,
  '        $(MKDIR) $(INSTALL_BINDIR)'#013#010+
  '# Compress the exes if upx is defined'#013#010+
  'ifdef UPXPROG'#013#010+
  '        -$(UPXPROG) $(INSTALLEXEFILES)'#013#010+
  'endif'#013#010+
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#013#010+
  'ifde','f FPCMAKE'#013#010+
  '# If the fpcpackage variable is set then create and install Package.fp'+
  'c,'#013#010+
  '# a safety check is done if Makefile.fpc is available'#013#010+
  'ifdef PACKAGE_VERSION'#013#010+
  'ifneq ($(wildcard Makefile.fpc),)'#013#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS','_TARGET) Makefile.fpc'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) Package.fpc $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(I','NSTALL_UNITDIR)'#013#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#013#010+
  '        $(INSTALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'ifneq ($(wildcard $(LIB_FULLNAME)),)'#013#010+
  '        $(MKDIR) $(INSTALL_LIBDIR)'#013#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LI','BDIR)'#013#010+
  'ifdef inUnix'#013#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_LIBDIR)/$(LIB_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FILES'#013#010+
  '        $(MKDIR) $(INSTALL_DATADIR)'#013#010+
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_sourceins','tall: distclean'#013#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#013#010+
  '        $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#013#010+
  #013#010+
  'fpc_exampleinstall: $(addsuffix _distclean,$(TARGET_EXAMPLEDIRS))'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#013#010+
  'e','ndif'#013#010+
  'ifdef EXAMPLESOURCEFILES'#013#010+
  '        $(COPY) $(EXAMPLESOURCEFILES) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_EX'+
  'AMPLEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '[distinstallrules]'#013#010+
  '####','################################################################'+
  '#'#013#010+
  '# Dist Install'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_distinstall'#013#010+
  #013#010+
  'fpc_distinstall: install exampleinstall'#013#010+
  #013#010+
  #013#010+
  '[zipinstallr','ules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Zip'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#013#010+
  #013#010+
  '# Temporary p','ath to pack a file, can only use a single deep'#013#010+
  '# subdir, because the deltree can'#039't see the whole tree to remove'#013+
  #010+
  'ifndef PACKDIR'#013#010+
  'ifndef inUnix'#013#010+
  'PACKDIR=$(BASEDIR)/../fpc-pack'#013#010+
  'else'#013#010+
  'PACKDIR=/tmp/fpc-pack'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Maybe create def','ault zipname from packagename'#013#010+
  'ifndef ZIPNAME'#013#010+
  'ifdef DIST_ZIPNAME'#013#010+
  'ZIPNAME=$(DIST_ZIPNAME)'#013#010+
  'else'#013#010+
  'ZIPNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef FULLZIPNAME'#013#010+
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  '# ZipTa','rget'#013#010+
  'ifndef ZIPTARGET'#013#010+
  'ifdef DIST_ZIPTARGET'#013#010+
  'ZIPTARGET=DIST_ZIPTARGET'#013#010+
  'else'#013#010+
  'ZIPTARGET=install'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use tar by default under linux'#013#010+
  'ifndef USEZIP'#013#010+
  'ifdef inUnix'#013#010+
  'USETAR=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use a wrapper script by default for ','Os/2'#013#010+
  'ifndef inUnix'#013#010+
  'USEZIPWRAPPER=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# We need to be able to run in the current OS so fix'#013#010+
  '# the path separator'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  'ZIPPATHSEP=$(PATHSEP)'#013#010+
  'ZIPWRAPPER=$(subst /,$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT))'#013+
  #010+
  'el','se'#013#010+
  'ZIPPATHSEP=/'#013#010+
  'endif'#013#010+
  #013#010+
  '# Create commands to create the zip/tar file'#013#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#013#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#013#010+
  'ifdef USETAR'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)','$(TAREXT)'#013#010+
  'ZIPCMD_ZIP:=$(TARPROG) c$(TAROPT)f $(ZIPDESTFILE) *'#013#010+
  'else'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#013#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
>>>>>>> graemeg/cpstrnew
=======
  ';'#013#010+
  #013#010+
  '[defines]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Misc defines to be used by anyone'#013#010+
  '#############################################','#######################'+
  '#'#013#010+
  #013#010+
  '# OS categories'#013#010+
  'BSDs = freebsd netbsd openbsd darwin'#013#010+
  'UNIXs = linux $(BSDs) solaris qnx haiku'#013#010+
  'LIMIT83fs = go32v2 os2 emx watcom'#013#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#013#010+
  #013#010+
  '#Empty target for rules that always s','hould run. Needed if'#013#010+
  '#the target is non-phoney, and there is non-phony prereqisites.'#013#010+
  '#Then add FORCE as an prerequisite'#013#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#013#010+
  'FORCE:'#013#010+
  '.PHONY: FORCE'#013#010+
  #013#010+
  '[osdetect]'#013#010+
  '#############','#######################################################'+
  '#'#013#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#013+
  #010+
  '# define inUnix when running under Unix like environment'#013#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,C','ygwin)'#013#010+
  '# define inWinNT when running under WinNT'#013#010+
  '# define inOS2 when running under OS/2'#013#010+
  '# define inCygwin when running under Cygwin32'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# We need only / in the path ','also remove the current dir,'#013#010+
  '# also remove trailing /'#039's'#013#010+
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#013#010+
  #013#010+
  '# Detect unix'#013#010+
  '# Darwin is handled specially'#013#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#013#010+
  'inUnix=1 #darwin'#013#010+
  'SEARCHPATH:=$(filter-','out .,$(subst :, ,$(PATH)))'#013#010+
  'else'#013#010+
  '# Determine if we'#039've a unix searchpath by looking for a ;'#013#010+
  '# that normally doesn'#039't exists in the unix PATH var.'#013#010+
  'ifeq ($(findstring ;,$(PATH)),)'#013#010+
  'inUnix=1'#013#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#013,#010+
  'else'#013#010+
  'SEARCHPATH:=$(subst ;, ,$(PATH))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add path were make is located'#013#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#013#010+
  #013#010+
  '# Search for PWD'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#013#010+
  'ifeq ($(P','WD),)'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013#010+
  '$(error You need the GNU utils package to use this Makefile)'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT='#013#010+
  'endif'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT=.ex','e'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect NT - NT sets OS to Windows_NT'#013#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#013#010+
  'ifndef inUnix'#013#010+
  'ifeq ($(OS),Windows_NT)'#013#010+
  'inWinNT=1'#013#010+
  'else'#013#010+
  'ifdef OS2_SHELL'#013#010+
  'inOS2=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifneq ($(findstring cygdrive,$(PATH)),)'#013#010,
  'inCygWin=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# The extension of batch files / scripts'#013#010+
  'ifdef inUnix'#013#010+
  'SRCBATCHEXT=.sh'#013#010+
  'else'#013#010+
  'ifdef inOS2'#013#010+
  'SRCBATCHEXT=.cmd'#013#010+
  'else'#013#010+
  'SRCBATCHEXT=.bat'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef COMSPEC'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(OSNeedsComsp','ecToRunBatch)),)'#013#010+
  'ifndef RUNBATCH'#013#010+
  'RUNBATCH=$(COMSPEC) /C'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#013#010+
  '# at the end of a line'#013#010+
  'ifdef inUnix'#013#010+
  'PATHSEP=/'#013#010+
  'else'#013#010+
  'PATHSEP:=$(subst /,\,/)'#013#010+
  '# ','cygwin bash or sh can not handle backslashs'#013#010+
  'ifdef inCygWin'#013#010+
  'PATHSEP=/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Base dir'#013#010+
  'ifdef PWD'#013#010+
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#013#010+
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#013#010+
  'ifdef inCygWin'#013#010+
  'ifneq ($(findstring',' /cygdrive/,$(BASEDIR)),)'#013#010+
  'BASENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#013#010+
  'BASEDRIVE:=$(firstword $(subst /, ,$(BASENODIR)))'#013#010+
  'BASEDIR:=$(subst /cygdrive/$(BASEDRIVE)/,$(BASEDRIVE):/,$(BASEDIR))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'BASEDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  '# Ech','o is an internal command under OS/2 (and others), so it'#039's a'+
  'lways found'#013#010+
  'ifdef inOS2'#013#010+
  'ifndef ECHO'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH))'+
  '))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /echo$(SRCEXEE','XT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO=echo'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export ECHO'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdetect]'#013#010+
  '##################################################################','##'+
  '#'#013#010+
  '# FPC Binary and Version Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Compatibility with old makefiles'#013#010+
  'ifndef FPC'#013#010+
  'ifdef PP'#013#010+
  'FPC=$(PP)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try to detect the ppcXXX file to use by',' using "fpc -PB" option'#013+
  #010+
  '# to query for the default ppcXXX the fpc executable tries. When'#013#010+
  '# fpc is not found use ppc386 by default. Also when fpc -PB gives'#013#010+
  '# an error ppc386 will be used.'#013#010+
  'ifndef FPC'#013#010+
  '# check if fpc exists'#013#010+
  'FPCPROG:=$(str','ip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifneq ($(FPCPROG),)'#013#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#013#010+
  'ifneq ($(CPU_TARGET),)'#013#010+
  'FPC:=$(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#013#010+
  'else'#013#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#013#010+
  'endif'#013#010+
  '# Older fpc',' executables didn'#039't support it and return'#013#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#013#010+
  'ifneq ($(findstring Error,$(FPC)),)'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#013#010+
  'e','lse'#013#010+
  '# if the cross compiler is not found, fall back to fpc'#013#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#013#010+
  'FPC:=$(firstword $(FPCPROG))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# fpc binary not found, fallback to ppc386'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(ad','dsuffix /ppc386$(SRC'+
  'EXEEXT),$(SEARCHPATH)))))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Get a clean executable name'#013#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#013#010+
  'override FPC:=$(subst \,/,$(FPC))$(SRCEXEEXT)'#013#010+
  #013#010+
  '# Try to find the binary direct first, otherwise in ','the path,'#013#010+
  '# if not found give an error'#013#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  '$(error Compiler $(FPC) not found)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010,
  '# For 1.1 and up we can use a single compiler call to retrieve'#013#010+
  '# all needed information'#013#010+
  'ifndef FPC_COMPILERINFO'#013#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# FPC version'#013#010+
  'ifndef FPC_VERSION'#013#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPIL','ERINFO))'#013#010+
  'endif'#013#010+
  #013#010+
  'export FPC FPC_VERSION FPC_COMPILERINFO'#013#010+
  '# CHECKDEPEND should not be exported'#013#010+
  '# This should limit multiple checks'#013#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#013#010+
  #013#010+
  '#################################################################','###'+
  '#'#013#010+
  '# FPC Target Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Fall back to default values if needed'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'ifdef CPU_TARGET_DEFAULT'#013#010+
  'CPU_TARGET=$(CPU_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifn','def OS_TARGET'#013#010+
  'ifdef OS_TARGET_DEFAULT'#013#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.0.x we need to use extra calls to retrieve all info'#013#010+
  'ifneq ($(words $(FPC_COMPILERINFO)),5)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSP)'#013#010+
  'FPC_COMPILE','RINFO+=$(shell $(FPC) -iTP)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSO)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Retrieve Target/Source CPU and Target/Source OS'#013#010+
  'ifndef CPU_SOURCE'#013#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'if','ndef CPU_TARGET'#013#010+
  'CPU_TARGET:=$(word 3,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_SOURCE'#013#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013,#010+
  'FULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifeq ($(CPU_TARGET),armeb)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-Cb'#013#010+
  'else'#013#010+
  'ifeq ($(CPU_TARGET),armel)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-CaEABI'#013#010+
  'else'#013#010+
  'ARCH=$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Full name of the target',', including CPU and OS. For OSs limited'#013+
  #010+
  '# to 8.3 we only use the target OS'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'TARGETSUFFIX=$(OS_TARGET)'#013#010+
  'SOURCESUFFIX=$(OS_SOURCE)'#013#010+
  'else'#013#010+
  'TARGETSUFFIX=$(FULL_TARGET)'#013#010+
  'SOURCESUFFIX=$(FULL_SOUR','CE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compile flag'#013#010+
  'ifneq ($(FULL_TARGET),$(FULL_SOURCE))'#013#010+
  'CROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the Makefile supports this target, but not'#013#010+
  '# when the make target is to rebuild the makefile'#013#010+
  'ifeq ($(findstring makefile,$(MAKE','CMDGOALS)),)'#013#010+
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),)'#013#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Detect BSD, since BSD uses a slightly different directory hier','arch'+
  'y.'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#013#010+
  'BSDhier=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect Linux, will also use its own directory hierarchy.'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'linuxHier=1'#013#010+
  'endif'#013#010+
  #013#010+
  'export OS_TARGET OS_SOURCE ARCH CPU_TARGET CPU_SOURCE FULL','_TARGET FU'+
  'LL_SOURCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#013#010+
  #013#010+
  #013#010+
  '[fpcdircheckenv]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPCDIR Setting'#013#010+
  '#####################################################################',
  #013#010+
  #013#010+
  '# Test FPCDIR to look if the RTL dir exists'#013#010+
  'ifdef FPCDIR'#013#010+
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default FPC','DIR from Makefile.fpc'#013#010+
  'ifdef DEFAULT_FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdirdetect]'#013#010+
  '# Det','ect FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'ifdef inUnix'#013#010+
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#013#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#013#010+
  'override FPCDIR=/usr/lib/fpc/$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR:=$(subst /$(FPC),,$(firstwor','d $(strip $(wildcard $(a'+
  'ddsuffix /$(FPC),$(SEARCHPATH))))))'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'overrid','e FPCDIR:=$(BASEDIR)'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=c:/pp'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross binaries dir'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(FPCDIR)/bin/$(TARGETSUFFIX))'#013#010+
  'endif',#013#010+
  #013#010+
  '# Default binutils prefix for cross compile when the'#013#010+
  '# crossbindir is not set (except for Darwin)'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'ifeq ($(OS_SOURCE),darwin)'#013#010+
  'DARWIN2DARWIN=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef BINUTILSPREFIX',#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifndef DARWIN2DARWIN'#013#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try first the full target name, otherwise try only'#013#010+
  '# the OS for backwards compatibility'#013#010+
  'UNITSDIR:=','$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#013#010+
  'ifeq ($(UNITSDIR),)'#013#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(OS_TARGET))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Packages dir'#013#010+
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages $(FPCDIR)/package'+
  's/base $(FPCDIR)/packages/e','xtra)'#013#010+
  #013#010+
  #013#010+
  '[shelltools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Shell tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Echo that can redir (must be able run in the default O','S shell)'#013+
  #010+
  'ifndef ECHOREDIR'#013#010+
  'ifndef inUnix'#013#010+
  'ECHOREDIR=echo'#013#010+
  'else'#013#010+
  'ECHOREDIR=$(ECHO)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To copy pograms'#013#010+
  'ifndef COPY'#013#010+
  'COPY:=$(CPPROG) -fp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef COPYTREE'#013#010+
  'COPYTREE:=$(CPPROG) -Rfp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cop','y a whole tree'#013#010+
  'ifndef MKDIRTREE'#013#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#013#010+
  'endif'#013#010+
  #013#010+
  '# To move pograms'#013#010+
  'ifndef MOVE'#013#010+
  'MOVE:=$(MVPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check delete program'#013#010+
  'ifndef DEL'#013#010+
  'DEL:=$(RMPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check deltree program'#013#010+
  'ifndef DELTREE'#013#010+
  'DEL','TREE:=$(RMPROG) -rf'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install files'#013#010+
  'ifndef INSTALL'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALL:=$(GINSTALL) -c -m 644'#013#010+
  'else'#013#010+
  'INSTALL:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install programs'#013#010+
  'ifndef INSTALLEXE'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALLEXE:=$(GINSTALL) -c -m 75','5'#013#010+
  'else'#013#010+
  'INSTALLEXE:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To make a directory.'#013#010+
  'ifndef MKDIR'#013#010+
  'MKDIR:=$(GINSTALL) -m 755 -d'#013#010+
  'endif'#013#010+
  #013#010+
  'export ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE MKD'+
  'IR'#013#010+
  #013#010+
  #013#010+
  '[defaultdirs]'#013#010+
  '########################','############################################'+
  '#'#013#010+
  '# Default Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Units dir'#013#010+
  'ifdef REQUIRE_UNITSDIR'#013#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Units dir',#013#010+
  'ifdef REQUIRE_PACKAGESDIR'#013#010+
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#013#010+
  '# When zipping use the target os default, when normal install then'#013+
  #010+
  '# use the source os as default'#013#010+
  'ifdef ','ZIPINSTALL'#013#010+
  '# Zipinstall'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# Normal install'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# When install prefix is not set try to use prefix'#013,#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef PREFIX'#013#010+
  'INSTALL_PREFIX=$(PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the prefix directory where to install everything'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_PREFIX=/usr/local'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTAL','L_BASEDIR:=/pp'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export INSTALL_PREFIX'#013#010+
  #013#010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#013#010+
  '# used recursively for all subdirs'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'export INSTALL_FPCSUBD','IR'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to place the resulting zip files'#013#010+
  'ifndef DIST_DESTDIR'#013#010+
  'DIST_DESTDIR:=$(BASEDIR)'#013#010+
  'endif'#013#010+
  'export DIST_DESTDIR'#013#010+
  #013#010+
  '# EXE/PPU Target directories'#013#010+
  'ifndef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef PACKAGEDIR_MAIN'#013#010+
  'COMPILER_UNITTARGETDIR=$','(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#013#010+
  'else'#013#010+
  'COMPILER_UNITTARGETDIR=units/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef COMPILER_TARGETDIR'#013#010+
  'COMPILER_TARGETDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# ','Install Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# set the base directory where to install everything'#013#010+
  'ifndef INSTALL_BASEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL','_PREFIX)/lib/fpc/$(FPC_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the binaries'#013#010+
  'ifndef INSTALL_BINDIR'#013#010+
  'ifdef ','UNIXHier'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/bin'#013#010+
  '# for FPC packages install the binaries under their target subdir'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifdef CROSSINSTALL'#013#010+
  'INSTALL_BIND','IR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the units.'#013#010+
  'i','fndef INSTALL_UNITDIR'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef PACKAGE_NAME'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install shared libraries'#013,#010+
  'ifndef INSTALL_LIBDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#013#010+
  'else'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the source files will be stored'#013#010+
  'ifndef INSTALL_SOURCEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'SRCPR','EFIXDIR=share/src'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'SRCPREFIXDIR=src'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INSTALL_FPC','SUBDIR)/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef',' INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$(PA'+
  'CKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_B','ASEDIR)/source'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the doc files will be stored'#013#010+
  'ifndef INSTALL_DOCDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'DOCPREFIXDIR=doc'#013#010+
  'endif'#013#010+
  'endif'#013#010,
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/fpc-$(FPC_VERSION)/$'+
  '(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACKAGE_NAME)-$(PA'+
  'CKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAG','E'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install the examples, under linux we use the doc dir'#013#010+
  '# because the copytree command will create a su','bdir itself'#013#010+
  'ifndef INSTALL_EXAMPLEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL','_DOCDIR)/examples'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010,
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACK','AGE'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the some extra (data)files will be stored'#013#010+
  'ifndef INSTALL_DATADIR'#013#010+
  'INSTALL_DATADIR=$','(INSTALL_BASEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef INSTALL_SHAREDDIR'#013#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#013#010+
  'endif'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# Cross compile dirs'#013#010+
  '###########################################','#########################'+
  '#'#013#010+
  #013#010+
  'ifdef CROSSCOMPILE'#013#010+
  '# Directory where the cross compile tools are stored.'#013#010+
  '# First check if they are available in FPCDIR. If no targets/ subdir'#013+
  #010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#013#010+
  'ifndef CR','OSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/cross/$(TARGETSUFFIX)/bin/$'+
  '(FULL_SOURCE))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'CROSSBINDIR='#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[dirlibc]'#013,#010+
  '# On linux, try to find where libgcc.a is.'#013#010+
  'ifeq ($(OS_SOURCE),linux)'#013#010+
  #013#010+
  '# Amd64 to i386?'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'ifneq ($(findstring x86_64,$(shell uname -a)),)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname ','`gcc -m32 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try cross gcc'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'C','ROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSPREFIX)gcc$(SRCEX'+
  'EEXT),$(SEARCHPATH))))'#013#010+
  'ifneq ($(CROSSGCC),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Other libraries from ld.so.conf'#013#010+
  'ifndef OT','HERLIBDIR'#013#010+
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.conf | awk '#039'{ ORS=" '+
  '"; print $1 }'#039')'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef inUnix'#013#010+
  'ifeq ($(OS_SOURCE),netbsd)'#013#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#013#010+
  'endif'#013#010+
  'export GCCLIBDIR OTHERLIB'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[extensions]'#013#010+
  '##','##################################################################'+
  '#'#013#010+
  '# Default extensions'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Default needed extensions (Go32v2,Linux)'#013#010+
  'BATCHEXT=.bat'#013#010+
  'LOADEREXT=.as'#013#010+
  'EX','EEXT=.exe'#013#010+
  'PPLEXT=.ppl'#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHAREDLIBPREFIX=libfp'#013#010+
  'STATICLIBPREFIX=libp'#013#010+
  'IMPORTLIBPREFIX=libimp'#013#010+
  'RSTEXT=.rst'#013#010+
  '#DEBUGSYMEXT #for debugger symbol files, define only',' for targets whi'+
  'ch has this'#013#010+
  #013#010+
  '# 1.0.x has target specific extensions for ppu files and objects'#013#010+
  'ifeq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  '# short version for 1.1 and above - no target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TARGET),go','32v1)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'OEXT=.obj'#013#010+
  'ASMEXT=.asm'#013#010+
  'SHAREDLIBE','XT=.dll'#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSU','FFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq',' ($(OS_TARGET),win32)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# EMX'#013#010+
  'ifeq ($(OS_','TARGET),emx)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=emx'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# MorphOS'#013#010,
  'ifeq ($(OS_TARGET),morphos)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=mos'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif',#013#010+
  #013#010+
  '# Haiku'#013#010+
  'ifeq ($(OS_TARGET),haiku)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=hai'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSU','FFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware clib'#013#010+
  'ifeq ($(OS_TARGET),netware)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPOR','TLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Darwin'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSH','AREDLIB=1'#013#010+
  'SHORTSUFFIX=dwn'#013#010+
  'endif'#013#010+
  #013#010+
  '# gba'#013#010+
  'ifeq ($(OS_TARGET),gba)'#013#010+
  'EXEEXT=.gba'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=gba'#013#010+
  'endif'#013#010+
  #013#010+
  '# Symbian OS'#013#010+
  'ifeq ($(OS_TARGET),symbian)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=symbian'#013#010+
  'endif'#013#010+
  #013#010+
  '# NativeNT'#013#010+
  'ifeq ($(OS','_TARGET),NativeNT)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=nativent'#013#010+
  'endif'#013#010+
  #013#010+
  '# wii'#013#010+
  'ifeq ($(OS_TARGET),wii)'#013#010+
  'EXEEXT=.dol'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=wii'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  '# long version for 1.0.x - target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(','OS_TARGET),go32v1)'#013#010+
  'PPUEXT=.pp1'#013#010+
  'OEXT=.o1'#013#010+
  'ASMEXT=.s1'#013#010+
  'SMARTEXT=.sl1'#013#010+
  'STATICLIBEXT=.a1'#013#010+
  'SHAREDLIBEXT=.so1'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013,#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010,
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BAT','CHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'PPUEXT=.ppw'#013#010+
  'OEXT=.ow'#013#010+
  'ASMEXT=.sw'#013#010+
  'SMARTEXT=.slw'#013#010+
  'STATICLIBEXT=.aw'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os','2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'PPUEXT=.ppo'#013#010+
  'ASMEXT=.so2'#013#010+
  'OEXT=.oo2'#013#010+
  'AOUTEXT=.out'#013#010+
  'SMARTEXT=.sl2'#013#010+
  'STATICLIBPREFIX='#013#010+
  'STATICLIBEXT=.ao2'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=a','ta'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEX','T=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware'#013#010+
  'ifeq ($(OS_T','ARGET),netware)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'STATICLIBPREFIX='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'ST','ATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '#end of target specific settings'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 8.3 limited OS'#039's the short suffixes'#013#010+
  '# Otherwise use the full source/target names'#013#010+
  'ifneq ($(findstri','ng $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)'#013#010+
  'ZIPSUFFIX=$(SHORTSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'ZIPSOURCESUFFIX=src'#013#010+
  'ZIPEXAMPLESUFFIX=exm'#013#010+
  'else'#013#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#013#010+
  'ZIPSOURCESUFFIX=.source'#013#010+
  'ZIPEXAMPLESUFFIX=.example','s'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#013#010+
  'ZIPCROSSPREFIX=$(TARGETSUFFIX)-'#013#010+
  'else'#013#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[defaulttools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# ','Default Tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Names of the binutils tools'#013#010+
  'ASNAME=$(BINUTILSPREFIX)as'#013#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#013#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#013#010+
  'RCNAME=$(BINUTILSPREFIX)rc'#013#010+
  'ifneq ($(','findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'ASNAME=asw'#013#010+
  'LDNAME=ldw'#013#010+
  'ARNAME=arw'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# assembler, redefine it if cross compiling'#013#010+
  'ifndef ASPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ASPROG=$(CROSSBI','NDIR)/$(ASNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ASPROG=$(ASNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# linker, but probably not used'#013#010+
  'ifndef LDPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'LDPROG=$(CROSSBINDIR)/$(LDNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'LDPROG=$(LDNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Resource compiler',#013#010+
  'ifndef RCPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'RCPROG=$(CROSSBINDIR)/$(RCNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'RCPROG=$(RCNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Archiver'#013#010+
  'ifndef ARPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ARPROG=$(CROSSBINDIR)/$(ARNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ARPROG=$(ARNAME)'#013#010+
  'endi','f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Override defaults'#013#010+
  'AS=$(ASPROG)'#013#010+
  'LD=$(LDPROG)'#013#010+
  'RC=$(RCPROG)'#013#010+
  'AR=$(ARPROG)'#013#010+
  #013#010+
  '# ppas.bat / ppas.sh'#013#010+
  'PPAS=ppas$(SRCBATCHEXT)'#013#010+
  #013#010+
  '# ldconfig to rebuild .so cache'#013#010+
  'ifdef inUnix'#013#010+
  'LDCONFIG=ldconfig'#013#010+
  'else'#013#010+
  'LDCONFIG='#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef DA','TE'#013#010+
  'DATESTR:=$(shell $(DATE) +%Y%m%d)'#013#010+
  'else'#013#010+
  'DATESTR='#013#010+
  'endif'#013#010+
  #013#010+
  '# Look if UPX is found for go32v2 and win32. We can'#039't use $UPX bec'+
  'ause'#013#010+
  '# upx uses that one itself (PFV)'#013#010+
  'ifndef UPXPROG'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifeq ($','(OS_TARGET),win32)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifdef UPXPROG'#013#010+
  'UPXPROG:=$(strip $(wildcard $(addsuffix /upx$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(UPXPROG),)'#013#010+
  'UPXPROG='#013#010+
  'else'#013#010+
  'UPXPROG:=$(firstword $(UPXPROG))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'UPXPROG='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'expo','rt UPXPROG'#013#010+
  #013#010+
  '# Zip options'#013#010+
  'ZIPOPT=-9'#013#010+
  'ZIPEXT=.zip'#013#010+
  #013#010+
  '# Tar options'#013#010+
  'ifeq ($(USETAR),bz2)'#013#010+
  'TAROPT=vj'#013#010+
  'TAREXT=.tar.bz2'#013#010+
  'else'#013#010+
  'TAROPT=vz'#013#010+
  'TAREXT=.tar.gz'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_begin]'#013#010+
  '###########################################################','#########'+
  '#'#013#010+
  '# Compiler Command Line'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#013+
  #010+
  '# compiler this needs to be turned off'#013#010+
  'ifndef NOCPUDEF'#013#010+
  'over','ride FPCOPTDEF=$(ARCH)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Load commandline OPT and add target and unit dir to be sure'#013#010+
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#013#010+
  'override FPCOPT+=-T$(OS_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#013#010+
  'override FPCOPT+=-P$(ARCH)'#013#010,
  'endif'#013#010+
  #013#010+
  'ifeq ($(OS_SOURCE),openbsd)'#013#010+
  'override FPCOPT+=-FD$(NEW_BINUTILS_PATH)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef CROSSBOOTSTRAP'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-XP$(BINUTILSPREFIX)'#013#010+
  'endif'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-Xr$(RLIN','KPATH)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# User dirs should be first, so they are looked at first'#013#010+
  'ifdef UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef LIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#013#010+
  'endif'#013#010+
  'ifdef OBJDIR'#013#010+
  'override F','PCOPT+=$(addprefix -Fo,$(OBJDIR))'#013#010+
  'endif'#013#010+
  'ifdef INCDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(INCDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking'#013#010+
  'ifdef LINKSMART'#013#010+
  'override FPCOPT+=-XX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking creation'#013#010+
  'ifdef CREATESMART'#013#010+
  'override FPCOPT+=-CX'#013#010,
  'endif'#013#010+
  #013#010+
  '# Debug'#013#010+
  'ifdef DEBUG'#013#010+
  'override FPCOPT+=-gl'#013#010+
  'override FPCOPTDEF+=DEBUG'#013#010+
  'endif'#013#010+
  #013#010+
  '# Release mode'#013#010+
  '# (strip, optimize and don'#039't load fpc.cfg)'#013#010+
  '# fpc 2.1 has -O2 for all targets'#013#010+
  'ifdef RELEASE'#013#010+
  'ifneq ($(findstring 2.0.,$(FPC_VERSION)),',')'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'FPCCPUOPT:=-OG2p3'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc)'#013#010+
  'FPCCPUOPT:=-O1r'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'FPCCPUOPT:=-O2'#013#010+
  'endif'#013#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#013#010+
  'override FPCOPTDEF+=RELEASE'#013#010+
  'endif'#013#010+
  #013#010+
  '# Strip'#013#010+
  'ifdef STRIP'#013,#010+
  'override FPCOPT+=-Xs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Optimizer'#013#010+
  'ifdef OPTIMIZE'#013#010+
  'override FPCOPT+=-O2'#013#010+
  'endif'#013#010+
  #013#010+
  '# Verbose settings (warning,note,info)'#013#010+
  'ifdef VERBOSE'#013#010+
  'override FPCOPT+=-vwni'#013#010+
  'endif'#013#010+
  #013#010+
  '# Needed compiler options'#013#010+
  'ifdef COMPILER_OPTIONS'#013#010+
  'override ','FPCOPT+=$(COMPILER_OPTIONS)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_LIBRARYDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_OBJECTDIR'#013#010,
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_INCLUDEDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compiler utils'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'override FPCOPT+=-FD$(CROSSBINDIR)'#013#010+
  'end','if'#013#010+
  #013#010+
  '# Target dirs and the prefix to use for clean/install'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#013#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#013#010+
  'override TARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR',')/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Unit target dir (default is units/<cpu>-<os>/'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#013#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#013#010+
  'override UNITTARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override UNITTARGETDI','RPREFIX=$(COMPILER_UNITTARGETDIR)/'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#013#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CREATESHARED'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  #013#010+
  'i','feq ($(CPU_TARGET),i386)'#013#010+
  'override FPCOPT+=-Aas'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  #013#010+
  '# create always pic'#039'ed code on x86_64'#013#010+
  'ifeq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'ifeq ($(CPU_TARGET),x86_64)'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  'endif'#013#010+
  'endif'#013,#010+
  'endif'#013#010+
  #013#010+
  'ifdef LINKSHARED'#013#010+
  '#!!! deactived for testing override FPCOPT+=-XD'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_libc]'#013#010+
  '# Add GCC lib path if asked'#013#010+
  'ifdef GCCLIBDIR'#013#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#013#010+
  'endif'#013#010+
  'ifdef OTHERLIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -','Fl,$(OTHERLIBDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_end]'#013#010+
  '# Add commandline options last so they can override'#013#010+
  'ifdef OPT'#013#010+
  'override FPCOPT+=$(OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add defines from FPCOPTDEF to FPCOPT'#013#010+
  'ifdef FPCOPTDEF'#013#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOP','TDEF))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Was a config file specified ?'#013#010+
  'ifdef CFGFILE'#013#010+
  'override FPCOPT+=@$(CFGFILE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use the environment to pass commandline options?'#013#010+
  'ifdef USEENV'#013#010+
  'override FPCEXTCMD:=$(FPCOPT)'#013#010+
  'override FPCOPT:=!FPCEXTCMD'#013#010+
  'export FPC','EXTCMD'#013#010+
  'endif'#013#010+
  #013#010+
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013#010+
  'override AFULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifneq ($(AFULL_TARGET),$(AFULL_SOURCE))'#013#010+
  'override ACROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef ACROSSCOMPILE'#013#010+
  'override FPCOPT+=$(CROSS','OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Compiler commandline'#013#010+
  'override COMPILER:=$(FPC) $(FPCOPT)'#013#010+
  #013#010+
  '# also call ppas if with command option -s'#013#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#013#010+
  'ifeq (,$(findstring -s ,$(COMPILER)))'#013#010+
  'EXECPPAS='#013#010+
  'else'#013#010+
  'ifeq ','($(FULL_SOURCE),$(FULL_TARGET))'#013#010+
  'ifdef RUNBATCH'#013#010+
  'EXECPPAS:=@$(RUNBATCH) $(PPAS)'#013#010+
  'else'#013#010+
  'EXECPPAS:=@$(PPAS)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[loaderrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Loaders'#013#010+
  '###########','#########################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_loaders'#013#010+
  #013#010+
  'ifneq ($(TARGET_LOADERS),)'#013#010+
  'override ALLTARGET+=fpc_loaders'#013#010+
  'override CLEANTARGET+=fpc_loaders_clean'#013#010+
  'override INSTALLTARGET+=fpc_loaders_install'#013#010+
  #013#010+
  'overri','de LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#013#010+
  'endif'#013#010+
  #013#010+
  '%$(OEXT): %$(LOADEREXT)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#013#010+
  'else'#013#010+
  '        $(AS) -o $*$(OEXT) $<'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders: $(C','OMPILER_UNITTARGETDIR) $(LOADEROFILES)'#013#010+
  #013#010+
  'fpc_loaders_clean:'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#013#010+
  'else'#013#010+
  '        -$(DEL) $(LOADEROFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders_install:'#013#010+
  '     ','   $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFIL'+
  'ES)) $(INSTALL_UNITDIR)'#013#010+
  'else'#013#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[unitrules]',#013#010+
  '#####################################################################'#013+
  #010+
  '# Units'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_units'#013#010+
  #013#010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#013#010+
  'override ALLTAR','GET+=fpc_units'#013#010+
  #013#010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_UNITS))'#013#010+
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#013#010+
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'override CLEANP','PUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#013#010+
  #013#010+
  #013#010+
  '[exerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Exes'#013#010+
  '################################','####################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_exes'#013#010+
  #013#010+
  '# Programs are not needed for a cross installation'#013#010+
  'ifndef CROSSINSTALL'#013#010+
  'ifneq ($(TARGET_PROGRAMS),)'#013#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#013#010+
  'override EXEOFIL','ES:=$(addsuffix $(OEXT),$(TARGET_PROGRAMS)) $(addpre'+
  'fix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
  'PROGRAMS)))'#013#010+
  #013#010+
  'override ALLTARGET+=fpc_exes'#013#010+
  'ove','rride INSTALLEXEFILES+=$(EXEFILES)'#013#010+
  'override CLEANEXEFILES+=$(EXEFILES) $(EXEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(add','suffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES)'#013+
  #010+
  #013#010+
  #013#010+
  '[rstrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Resource strings'#013#010,
  '#####################################################################'#013+
  #010+
  #013#010+
  'ifdef TARGET_RSTS'#013#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_RSTS))'#013#010+
  #013#010+
  'override CLEANRSTFILES+=$(RSTFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[examplerules]'#013#010+
  '#########################','###########################################'+
  '#'#013#010+
  '# Examples'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_examples'#013#010+
  #013#010+
  'ifneq ($(TARGET_EXAMPLES),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $','(addsuffix .pp,$(TARGET_EXA'+
  'MPLES)) $(addsuffix .pas,$(TARGET_EXAMPLES)) $(addsuffix .lpr,$(TARGET'+
  '_EXAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#013#010+
  'override EXAMPLEFILES:=$(addsuffix $(EXEEXT),$(TARGET_EXAMPLES))'#013#010+
  'override EXAMPLEOFILES:=$(','addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(ad'+
  'dprefix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPL'+
  'ES))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TAR'+
  'GET_EXAMPLES)))'#013#010+
  #013#010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES)',' $(EXAMPLEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifneq ($(TARGE','T_EXAMPLEDIRS),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_examples: all $(EXAMPLEFILES) $(addsuffix _all,$(TARGET_EXAMPLEDIR'+
  'S))'#013#010+
  #013#010+
  #013#010+
  '[compilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# General compile rules'#013#010+
  '#######','#############################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_all fpc_smart fpc_debug fpc_release fpc_shared'#013#010+
  #013#010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#013#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMADE)'#013#010+
  #013#010+
  'fpc_all: $(FPCMADE)'#013,#010+
  #013#010+
  'fpc_smart:'#013#010+
  '        $(MAKE) all LINKSMART=1 CREATESMART=1'#013#010+
  #013#010+
  'fpc_debug:'#013#010+
  '        $(MAKE) all DEBUG=1'#013#010+
  #013#010+
  'fpc_release:'#013#010+
  '        $(MAKE) all RELEASE=1'#013#010+
  #013#010+
  '# General compile rules, available for both possible .pp and .pas exte'+
  'nsions'#013#010+
  #013#010+
  '.SUFFI','XES: $(EXEEXT) $(PPUEXT) $(OEXT) .pas .lpr .dpr .pp .rc .res'#013+
  #010+
  #013#010+
  '$(COMPILER_UNITTARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '$(COMPILER_TARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pp'#013#010+
  '        $(','COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXE','EXT): %.lpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.dpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%.res: %.rc'#013#010+
  '        windres -i $< -o $@'#013#010+
  #013#010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#013#010+
  'vpath %.pp $(COMPILER_','SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.pas $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.inc $(COMPILER_INCLUDED','IR)'#013#010+
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  'vpath %$(PPUEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '[sharedrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Library'#013#010+
  '##################################################','##################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_shared'#013#010+
  #013#010+
  'override INSTALLTARGET+=fpc_shared_install'#013#010+
  #013#010+
  'ifndef SHARED_LIBVERSION'#013#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_LIBNAME'#013#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_FU','LLNAME'#013#010+
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED_LIBNAME)-$(SHARED_LIBVERSIO'+
  'N)$(SHAREDLIBEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default sharedlib units are all unit objects'#013#010+
  'ifndef SHARED_LIBUNITS'#013#010+
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#013#010+
  'over','ride SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),$(SHARED'+
  '_LIBUNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared:'#013#010+
  'ifdef HASSHAREDLIB'#013#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  '        $(PPUMOVE) -q $(SHARED_','LIBUNITS) -i$(COMPILER_UNITTARGETDIR)'+
  ' -o$(SHARED_FULLNAME) -d$(COMPILER_UNITTARGETDIR)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '        @$(ECHO) Shared Libraries not supported'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared_install:'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  'ifneq ($(SHARED_LIBUNITS),)'#013#010+
  'ifne','q ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME)),)'#013+
  #010+
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[installrules]'#013#010+
  '####################################################','################'+
  '#'#013#010+
  '# Install rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#013#010+
  #013#010+
  'ifdef INSTALL_UNITS'#013#010+
  'override INSTALLPPUFILES+=$(addsuffix $(PPUEXT),$(','INSTALL_UNITS))'#013+
  #010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALL_BUILDUNIT'#013#010+
  'override INSTALLPPUFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT),$'+
  '(INSTALLPPUFILES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPP','U'+
  'FILES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEX'+
  'T),$(INSTALLPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#013#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#013#010+
  'override INSTALLPPUFILES:','=$(addprefix $(UNITTARGETDIRPREFIX),$(notdi'+
  'r $(INSTALLPPUFILES)))'#013#010+
  'override INSTALLPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPR'+
  'EFIX),$(notdir $(INSTALLPPULINKFILES))))'#013#010+
  'endif'#013#010+
  '# Implicitly install Package.fpc'#013#010+
  'override INSTALL_CRE','ATEPACKAGEFPC=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  'ifneq ($(TARGETDIRPREFIX),)'#013#010+
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_install: all $(INSTALLTARGET)'#013#010+
  'ifdef INSTALLEXEFILES'#013#010,
  '        $(MKDIR) $(INSTALL_BINDIR)'#013#010+
  '# Compress the exes if upx is defined'#013#010+
  'ifdef UPXPROG'#013#010+
  '        -$(UPXPROG) $(INSTALLEXEFILES)'#013#010+
  'endif'#013#010+
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#013#010+
  'ifde','f FPCMAKE'#013#010+
  '# If the fpcpackage variable is set then create and install Package.fp'+
  'c,'#013#010+
  '# a safety check is done if Makefile.fpc is available'#013#010+
  'ifdef PACKAGE_VERSION'#013#010+
  'ifneq ($(wildcard Makefile.fpc),)'#013#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS','_TARGET) Makefile.fpc'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) Package.fpc $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(I','NSTALL_UNITDIR)'#013#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#013#010+
  '        $(INSTALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'ifneq ($(wildcard $(LIB_FULLNAME)),)'#013#010+
  '        $(MKDIR) $(INSTALL_LIBDIR)'#013#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LI','BDIR)'#013#010+
  'ifdef inUnix'#013#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_LIBDIR)/$(LIB_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FILES'#013#010+
  '        $(MKDIR) $(INSTALL_DATADIR)'#013#010+
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_sourceins','tall: distclean'#013#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#013#010+
  '        $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#013#010+
  #013#010+
  'fpc_exampleinstall: $(addsuffix _distclean,$(TARGET_EXAMPLEDIRS))'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#013#010+
  'e','ndif'#013#010+
  'ifdef EXAMPLESOURCEFILES'#013#010+
  '        $(COPY) $(EXAMPLESOURCEFILES) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_EX'+
  'AMPLEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '[distinstallrules]'#013#010+
  '####','################################################################'+
  '#'#013#010+
  '# Dist Install'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_distinstall'#013#010+
  #013#010+
  'fpc_distinstall: install exampleinstall'#013#010+
  #013#010+
  #013#010+
  '[zipinstallr','ules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Zip'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#013#010+
  #013#010+
  '# Temporary p','ath to pack a file, can only use a single deep'#013#010+
  '# subdir, because the deltree can'#039't see the whole tree to remove'#013+
  #010+
  'ifndef PACKDIR'#013#010+
  'ifndef inUnix'#013#010+
  'PACKDIR=$(BASEDIR)/../fpc-pack'#013#010+
  'else'#013#010+
  'PACKDIR=/tmp/fpc-pack'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Maybe create def','ault zipname from packagename'#013#010+
  'ifndef ZIPNAME'#013#010+
  'ifdef DIST_ZIPNAME'#013#010+
  'ZIPNAME=$(DIST_ZIPNAME)'#013#010+
  'else'#013#010+
  'ZIPNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef FULLZIPNAME'#013#010+
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  '# ZipTa','rget'#013#010+
  'ifndef ZIPTARGET'#013#010+
  'ifdef DIST_ZIPTARGET'#013#010+
  'ZIPTARGET=DIST_ZIPTARGET'#013#010+
  'else'#013#010+
  'ZIPTARGET=install'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use tar by default under linux'#013#010+
  'ifndef USEZIP'#013#010+
  'ifdef inUnix'#013#010+
  'USETAR=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use a wrapper script by default for ','Os/2'#013#010+
  'ifndef inUnix'#013#010+
  'USEZIPWRAPPER=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# We need to be able to run in the current OS so fix'#013#010+
  '# the path separator'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  'ZIPPATHSEP=$(PATHSEP)'#013#010+
  'ZIPWRAPPER=$(subst /,$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT))'#013+
  #010+
  'el','se'#013#010+
  'ZIPPATHSEP=/'#013#010+
  'endif'#013#010+
  #013#010+
  '# Create commands to create the zip/tar file'#013#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#013#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#013#010+
  'ifdef USETAR'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)','$(TAREXT)'#013#010+
  'ZIPCMD_ZIP:=$(TARPROG) c$(TAROPT)f $(ZIPDESTFILE) *'#013#010+
  'else'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#013#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
>>>>>>> graemeg/cpstrnew
=======
  ';'#013#010+
  #013#010+
  '[defines]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Misc defines to be used by anyone'#013#010+
  '#############################################','#######################'+
  '#'#013#010+
  #013#010+
  '# OS categories'#013#010+
  'BSDs = freebsd netbsd openbsd darwin'#013#010+
  'UNIXs = linux $(BSDs) solaris qnx haiku'#013#010+
  'LIMIT83fs = go32v2 os2 emx watcom'#013#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#013#010+
  #013#010+
  '#Empty target for rules that always s','hould run. Needed if'#013#010+
  '#the target is non-phoney, and there is non-phony prereqisites.'#013#010+
  '#Then add FORCE as an prerequisite'#013#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#013#010+
  'FORCE:'#013#010+
  '.PHONY: FORCE'#013#010+
  #013#010+
  '[osdetect]'#013#010+
  '#############','#######################################################'+
  '#'#013#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#013+
  #010+
  '# define inUnix when running under Unix like environment'#013#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,C','ygwin)'#013#010+
  '# define inWinNT when running under WinNT'#013#010+
  '# define inOS2 when running under OS/2'#013#010+
  '# define inCygwin when running under Cygwin32'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# We need only / in the path ','also remove the current dir,'#013#010+
  '# also remove trailing /'#039's'#013#010+
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#013#010+
  #013#010+
  '# Detect unix'#013#010+
  '# Darwin is handled specially'#013#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#013#010+
  'inUnix=1 #darwin'#013#010+
  'SEARCHPATH:=$(filter-','out .,$(subst :, ,$(PATH)))'#013#010+
  'else'#013#010+
  '# Determine if we'#039've a unix searchpath by looking for a ;'#013#010+
  '# that normally doesn'#039't exists in the unix PATH var.'#013#010+
  'ifeq ($(findstring ;,$(PATH)),)'#013#010+
  'inUnix=1'#013#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#013,#010+
  'else'#013#010+
  'SEARCHPATH:=$(subst ;, ,$(PATH))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add path were make is located'#013#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#013#010+
  #013#010+
  '# Search for PWD'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#013#010+
  'ifeq ($(P','WD),)'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013#010+
  '$(error You need the GNU utils package to use this Makefile)'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT='#013#010+
  'endif'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT=.ex','e'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect NT - NT sets OS to Windows_NT'#013#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#013#010+
  'ifndef inUnix'#013#010+
  'ifeq ($(OS),Windows_NT)'#013#010+
  'inWinNT=1'#013#010+
  'else'#013#010+
  'ifdef OS2_SHELL'#013#010+
  'inOS2=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifneq ($(findstring cygdrive,$(PATH)),)'#013#010,
  'inCygWin=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# The extension of batch files / scripts'#013#010+
  'ifdef inUnix'#013#010+
  'SRCBATCHEXT=.sh'#013#010+
  'else'#013#010+
  'ifdef inOS2'#013#010+
  'SRCBATCHEXT=.cmd'#013#010+
  'else'#013#010+
  'SRCBATCHEXT=.bat'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef COMSPEC'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(OSNeedsComsp','ecToRunBatch)),)'#013#010+
  'ifndef RUNBATCH'#013#010+
  'RUNBATCH=$(COMSPEC) /C'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#013#010+
  '# at the end of a line'#013#010+
  'ifdef inUnix'#013#010+
  'PATHSEP=/'#013#010+
  'else'#013#010+
  'PATHSEP:=$(subst /,\,/)'#013#010+
  '# ','cygwin bash or sh can not handle backslashs'#013#010+
  'ifdef inCygWin'#013#010+
  'PATHSEP=/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Base dir'#013#010+
  'ifdef PWD'#013#010+
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#013#010+
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#013#010+
  'ifdef inCygWin'#013#010+
  'ifneq ($(findstring',' /cygdrive/,$(BASEDIR)),)'#013#010+
  'BASENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#013#010+
  'BASEDRIVE:=$(firstword $(subst /, ,$(BASENODIR)))'#013#010+
  'BASEDIR:=$(subst /cygdrive/$(BASEDRIVE)/,$(BASEDRIVE):/,$(BASEDIR))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'BASEDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  '# Ech','o is an internal command under OS/2 (and others), so it'#039's a'+
  'lways found'#013#010+
  'ifdef inOS2'#013#010+
  'ifndef ECHO'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH))'+
  '))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /echo$(SRCEXEE','XT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO=echo'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export ECHO'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdetect]'#013#010+
  '##################################################################','##'+
  '#'#013#010+
  '# FPC Binary and Version Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Compatibility with old makefiles'#013#010+
  'ifndef FPC'#013#010+
  'ifdef PP'#013#010+
  'FPC=$(PP)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try to detect the ppcXXX file to use by',' using "fpc -PB" option'#013+
  #010+
  '# to query for the default ppcXXX the fpc executable tries. When'#013#010+
  '# fpc is not found use ppc386 by default. Also when fpc -PB gives'#013#010+
  '# an error ppc386 will be used.'#013#010+
  'ifndef FPC'#013#010+
  '# check if fpc exists'#013#010+
  'FPCPROG:=$(str','ip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifneq ($(FPCPROG),)'#013#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#013#010+
  'ifneq ($(CPU_TARGET),)'#013#010+
  'FPC:=$(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#013#010+
  'else'#013#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#013#010+
  'endif'#013#010+
  '# Older fpc',' executables didn'#039't support it and return'#013#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#013#010+
  'ifneq ($(findstring Error,$(FPC)),)'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#013#010+
  'e','lse'#013#010+
  '# if the cross compiler is not found, fall back to fpc'#013#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#013#010+
  'FPC:=$(firstword $(FPCPROG))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# fpc binary not found, fallback to ppc386'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(ad','dsuffix /ppc386$(SRC'+
  'EXEEXT),$(SEARCHPATH)))))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Get a clean executable name'#013#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#013#010+
  'override FPC:=$(subst \,/,$(FPC))$(SRCEXEEXT)'#013#010+
  #013#010+
  '# Try to find the binary direct first, otherwise in ','the path,'#013#010+
  '# if not found give an error'#013#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  '$(error Compiler $(FPC) not found)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010,
  '# For 1.1 and up we can use a single compiler call to retrieve'#013#010+
  '# all needed information'#013#010+
  'ifndef FPC_COMPILERINFO'#013#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# FPC version'#013#010+
  'ifndef FPC_VERSION'#013#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPIL','ERINFO))'#013#010+
  'endif'#013#010+
  #013#010+
  'export FPC FPC_VERSION FPC_COMPILERINFO'#013#010+
  '# CHECKDEPEND should not be exported'#013#010+
  '# This should limit multiple checks'#013#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#013#010+
  #013#010+
  '#################################################################','###'+
  '#'#013#010+
  '# FPC Target Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Fall back to default values if needed'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'ifdef CPU_TARGET_DEFAULT'#013#010+
  'CPU_TARGET=$(CPU_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifn','def OS_TARGET'#013#010+
  'ifdef OS_TARGET_DEFAULT'#013#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.0.x we need to use extra calls to retrieve all info'#013#010+
  'ifneq ($(words $(FPC_COMPILERINFO)),5)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSP)'#013#010+
  'FPC_COMPILE','RINFO+=$(shell $(FPC) -iTP)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSO)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Retrieve Target/Source CPU and Target/Source OS'#013#010+
  'ifndef CPU_SOURCE'#013#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'if','ndef CPU_TARGET'#013#010+
  'CPU_TARGET:=$(word 3,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_SOURCE'#013#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013,#010+
  'FULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifeq ($(CPU_TARGET),armeb)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-Cb'#013#010+
  'else'#013#010+
  'ifeq ($(CPU_TARGET),armel)'#013#010+
  'ARCH=arm'#013#010+
  'override FPCOPT+=-CaEABI'#013#010+
  'else'#013#010+
  'ARCH=$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Full name of the target',', including CPU and OS. For OSs limited'#013+
  #010+
  '# to 8.3 we only use the target OS'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'TARGETSUFFIX=$(OS_TARGET)'#013#010+
  'SOURCESUFFIX=$(OS_SOURCE)'#013#010+
  'else'#013#010+
  'TARGETSUFFIX=$(FULL_TARGET)'#013#010+
  'SOURCESUFFIX=$(FULL_SOUR','CE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compile flag'#013#010+
  'ifneq ($(FULL_TARGET),$(FULL_SOURCE))'#013#010+
  'CROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the Makefile supports this target, but not'#013#010+
  '# when the make target is to rebuild the makefile'#013#010+
  'ifeq ($(findstring makefile,$(MAKE','CMDGOALS)),)'#013#010+
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),)'#013#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Detect BSD, since BSD uses a slightly different directory hier','arch'+
  'y.'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#013#010+
  'BSDhier=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect Linux, will also use its own directory hierarchy.'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'linuxHier=1'#013#010+
  'endif'#013#010+
  #013#010+
  'export OS_TARGET OS_SOURCE ARCH CPU_TARGET CPU_SOURCE FULL','_TARGET FU'+
  'LL_SOURCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#013#010+
  #013#010+
  #013#010+
  '[fpcdircheckenv]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPCDIR Setting'#013#010+
  '#####################################################################',
  #013#010+
  #013#010+
  '# Test FPCDIR to look if the RTL dir exists'#013#010+
  'ifdef FPCDIR'#013#010+
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default FPC','DIR from Makefile.fpc'#013#010+
  'ifdef DEFAULT_FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdirdetect]'#013#010+
  '# Det','ect FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'ifdef inUnix'#013#010+
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#013#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#013#010+
  'override FPCDIR=/usr/lib/fpc/$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPCDIR:=$(subst /$(FPC),,$(firstwor','d $(strip $(wildcard $(a'+
  'ddsuffix /$(FPC),$(SEARCHPATH))))))'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'overrid','e FPCDIR:=$(BASEDIR)'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=c:/pp'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross binaries dir'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(FPCDIR)/bin/$(TARGETSUFFIX))'#013#010+
  'endif',#013#010+
  #013#010+
  '# Default binutils prefix for cross compile when the'#013#010+
  '# crossbindir is not set (except for Darwin)'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'ifeq ($(OS_SOURCE),darwin)'#013#010+
  'DARWIN2DARWIN=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef BINUTILSPREFIX',#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifndef DARWIN2DARWIN'#013#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try first the full target name, otherwise try only'#013#010+
  '# the OS for backwards compatibility'#013#010+
  'UNITSDIR:=','$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#013#010+
  'ifeq ($(UNITSDIR),)'#013#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(OS_TARGET))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Packages dir'#013#010+
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages $(FPCDIR)/package'+
  's/base $(FPCDIR)/packages/e','xtra)'#013#010+
  #013#010+
  #013#010+
  '[shelltools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Shell tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Echo that can redir (must be able run in the default O','S shell)'#013+
  #010+
  'ifndef ECHOREDIR'#013#010+
  'ifndef inUnix'#013#010+
  'ECHOREDIR=echo'#013#010+
  'else'#013#010+
  'ECHOREDIR=$(ECHO)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To copy pograms'#013#010+
  'ifndef COPY'#013#010+
  'COPY:=$(CPPROG) -fp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef COPYTREE'#013#010+
  'COPYTREE:=$(CPPROG) -Rfp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cop','y a whole tree'#013#010+
  'ifndef MKDIRTREE'#013#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#013#010+
  'endif'#013#010+
  #013#010+
  '# To move pograms'#013#010+
  'ifndef MOVE'#013#010+
  'MOVE:=$(MVPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check delete program'#013#010+
  'ifndef DEL'#013#010+
  'DEL:=$(RMPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check deltree program'#013#010+
  'ifndef DELTREE'#013#010+
  'DEL','TREE:=$(RMPROG) -rf'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install files'#013#010+
  'ifndef INSTALL'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALL:=$(GINSTALL) -c -m 644'#013#010+
  'else'#013#010+
  'INSTALL:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install programs'#013#010+
  'ifndef INSTALLEXE'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALLEXE:=$(GINSTALL) -c -m 75','5'#013#010+
  'else'#013#010+
  'INSTALLEXE:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To make a directory.'#013#010+
  'ifndef MKDIR'#013#010+
  'MKDIR:=$(GINSTALL) -m 755 -d'#013#010+
  'endif'#013#010+
  #013#010+
  'export ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE MKD'+
  'IR'#013#010+
  #013#010+
  #013#010+
  '[defaultdirs]'#013#010+
  '########################','############################################'+
  '#'#013#010+
  '# Default Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Units dir'#013#010+
  'ifdef REQUIRE_UNITSDIR'#013#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Units dir',#013#010+
  'ifdef REQUIRE_PACKAGESDIR'#013#010+
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#013#010+
  '# When zipping use the target os default, when normal install then'#013+
  #010+
  '# use the source os as default'#013#010+
  'ifdef ','ZIPINSTALL'#013#010+
  '# Zipinstall'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# Normal install'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# When install prefix is not set try to use prefix'#013,#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef PREFIX'#013#010+
  'INSTALL_PREFIX=$(PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the prefix directory where to install everything'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_PREFIX=/usr/local'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTAL','L_BASEDIR:=/pp'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export INSTALL_PREFIX'#013#010+
  #013#010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#013#010+
  '# used recursively for all subdirs'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'export INSTALL_FPCSUBD','IR'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to place the resulting zip files'#013#010+
  'ifndef DIST_DESTDIR'#013#010+
  'DIST_DESTDIR:=$(BASEDIR)'#013#010+
  'endif'#013#010+
  'export DIST_DESTDIR'#013#010+
  #013#010+
  '# EXE/PPU Target directories'#013#010+
  'ifndef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef PACKAGEDIR_MAIN'#013#010+
  'COMPILER_UNITTARGETDIR=$','(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#013#010+
  'else'#013#010+
  'COMPILER_UNITTARGETDIR=units/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef COMPILER_TARGETDIR'#013#010+
  'COMPILER_TARGETDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# ','Install Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# set the base directory where to install everything'#013#010+
  'ifndef INSTALL_BASEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL','_PREFIX)/lib/fpc/$(FPC_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the binaries'#013#010+
  'ifndef INSTALL_BINDIR'#013#010+
  'ifdef ','UNIXHier'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/bin'#013#010+
  '# for FPC packages install the binaries under their target subdir'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifdef CROSSINSTALL'#013#010+
  'INSTALL_BIND','IR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the units.'#013#010+
  'i','fndef INSTALL_UNITDIR'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef PACKAGE_NAME'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install shared libraries'#013,#010+
  'ifndef INSTALL_LIBDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#013#010+
  'else'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the source files will be stored'#013#010+
  'ifndef INSTALL_SOURCEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'SRCPR','EFIXDIR=share/src'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'SRCPREFIXDIR=src'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INSTALL_FPC','SUBDIR)/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef',' INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$(PA'+
  'CKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_B','ASEDIR)/source'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the doc files will be stored'#013#010+
  'ifndef INSTALL_DOCDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'DOCPREFIXDIR=doc'#013#010+
  'endif'#013#010+
  'endif'#013#010,
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/fpc-$(FPC_VERSION)/$'+
  '(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACKAGE_NAME)-$(PA'+
  'CKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAG','E'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install the examples, under linux we use the doc dir'#013#010+
  '# because the copytree command will create a su','bdir itself'#013#010+
  'ifndef INSTALL_EXAMPLEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL','_DOCDIR)/examples'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010,
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACK','AGE'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the some extra (data)files will be stored'#013#010+
  'ifndef INSTALL_DATADIR'#013#010+
  'INSTALL_DATADIR=$','(INSTALL_BASEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef INSTALL_SHAREDDIR'#013#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#013#010+
  'endif'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# Cross compile dirs'#013#010+
  '###########################################','#########################'+
  '#'#013#010+
  #013#010+
  'ifdef CROSSCOMPILE'#013#010+
  '# Directory where the cross compile tools are stored.'#013#010+
  '# First check if they are available in FPCDIR. If no targets/ subdir'#013+
  #010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#013#010+
  'ifndef CR','OSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/cross/$(TARGETSUFFIX)/bin/$'+
  '(FULL_SOURCE))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'CROSSBINDIR='#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[dirlibc]'#013,#010+
  '# On linux, try to find where libgcc.a is.'#013#010+
  'ifeq ($(OS_SOURCE),linux)'#013#010+
  #013#010+
  '# Amd64 to i386?'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'ifneq ($(findstring x86_64,$(shell uname -a)),)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname ','`gcc -m32 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try cross gcc'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'C','ROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSPREFIX)gcc$(SRCEX'+
  'EEXT),$(SEARCHPATH))))'#013#010+
  'ifneq ($(CROSSGCC),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Other libraries from ld.so.conf'#013#010+
  'ifndef OT','HERLIBDIR'#013#010+
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.conf | awk '#039'{ ORS=" '+
  '"; print $1 }'#039')'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef inUnix'#013#010+
  'ifeq ($(OS_SOURCE),netbsd)'#013#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#013#010+
  'endif'#013#010+
  'export GCCLIBDIR OTHERLIB'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[extensions]'#013#010+
  '##','##################################################################'+
  '#'#013#010+
  '# Default extensions'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Default needed extensions (Go32v2,Linux)'#013#010+
  'BATCHEXT=.bat'#013#010+
  'LOADEREXT=.as'#013#010+
  'EX','EEXT=.exe'#013#010+
  'PPLEXT=.ppl'#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHAREDLIBPREFIX=libfp'#013#010+
  'STATICLIBPREFIX=libp'#013#010+
  'IMPORTLIBPREFIX=libimp'#013#010+
  'RSTEXT=.rst'#013#010+
  '#DEBUGSYMEXT #for debugger symbol files, define only',' for targets whi'+
  'ch has this'#013#010+
  #013#010+
  '# 1.0.x has target specific extensions for ppu files and objects'#013#010+
  'ifeq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  '# short version for 1.1 and above - no target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TARGET),go','32v1)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'OEXT=.obj'#013#010+
  'ASMEXT=.asm'#013#010+
  'SHAREDLIBE','XT=.dll'#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSU','FFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq',' ($(OS_TARGET),win32)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# EMX'#013#010+
  'ifeq ($(OS_','TARGET),emx)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=emx'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# MorphOS'#013#010,
  'ifeq ($(OS_TARGET),morphos)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=mos'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif',#013#010+
  #013#010+
  '# Haiku'#013#010+
  'ifeq ($(OS_TARGET),haiku)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=hai'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSU','FFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware clib'#013#010+
  'ifeq ($(OS_TARGET),netware)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPOR','TLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Darwin'#013#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSH','AREDLIB=1'#013#010+
  'SHORTSUFFIX=dwn'#013#010+
  'endif'#013#010+
  #013#010+
  '# gba'#013#010+
  'ifeq ($(OS_TARGET),gba)'#013#010+
  'EXEEXT=.gba'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=gba'#013#010+
  'endif'#013#010+
  #013#010+
  '# Symbian OS'#013#010+
  'ifeq ($(OS_TARGET),symbian)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=symbian'#013#010+
  'endif'#013#010+
  #013#010+
  '# NativeNT'#013#010+
  'ifeq ($(OS','_TARGET),NativeNT)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=nativent'#013#010+
  'endif'#013#010+
  #013#010+
  '# wii'#013#010+
  'ifeq ($(OS_TARGET),wii)'#013#010+
  'EXEEXT=.dol'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=wii'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  '# long version for 1.0.x - target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(','OS_TARGET),go32v1)'#013#010+
  'PPUEXT=.pp1'#013#010+
  'OEXT=.o1'#013#010+
  'ASMEXT=.s1'#013#010+
  'SMARTEXT=.sl1'#013#010+
  'STATICLIBEXT=.a1'#013#010+
  'SHAREDLIBEXT=.so1'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013,#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010,
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BAT','CHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'PPUEXT=.ppw'#013#010+
  'OEXT=.ow'#013#010+
  'ASMEXT=.sw'#013#010+
  'SMARTEXT=.slw'#013#010+
  'STATICLIBEXT=.aw'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os','2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'PPUEXT=.ppo'#013#010+
  'ASMEXT=.so2'#013#010+
  'OEXT=.oo2'#013#010+
  'AOUTEXT=.out'#013#010+
  'SMARTEXT=.sl2'#013#010+
  'STATICLIBPREFIX='#013#010+
  'STATICLIBEXT=.ao2'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'IMPORTLIBPREFIX='#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=a','ta'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEX','T=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware'#013#010+
  'ifeq ($(OS_T','ARGET),netware)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'STATICLIBPREFIX='#013,#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'ST','ATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'IMPORTLIBPREFIX=imp'#013#010+
  'endif'#013#010+
  #013#010+
  '#end of target specific settings'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 8.3 limited OS'#039's the short suffixes'#013#010+
  '# Otherwise use the full source/target names'#013#010+
  'ifneq ($(findstri','ng $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)'#013#010+
  'ZIPSUFFIX=$(SHORTSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'ZIPSOURCESUFFIX=src'#013#010+
  'ZIPEXAMPLESUFFIX=exm'#013#010+
  'else'#013#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#013#010+
  'ZIPSOURCESUFFIX=.source'#013#010+
  'ZIPEXAMPLESUFFIX=.example','s'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#013#010+
  'ZIPCROSSPREFIX=$(TARGETSUFFIX)-'#013#010+
  'else'#013#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[defaulttools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# ','Default Tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Names of the binutils tools'#013#010+
  'ASNAME=$(BINUTILSPREFIX)as'#013#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#013#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#013#010+
  'RCNAME=$(BINUTILSPREFIX)rc'#013#010+
  'ifneq ($(','findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'ASNAME=asw'#013#010+
  'LDNAME=ldw'#013#010+
  'ARNAME=arw'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# assembler, redefine it if cross compiling'#013#010+
  'ifndef ASPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ASPROG=$(CROSSBI','NDIR)/$(ASNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ASPROG=$(ASNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# linker, but probably not used'#013#010+
  'ifndef LDPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'LDPROG=$(CROSSBINDIR)/$(LDNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'LDPROG=$(LDNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Resource compiler',#013#010+
  'ifndef RCPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'RCPROG=$(CROSSBINDIR)/$(RCNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'RCPROG=$(RCNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Archiver'#013#010+
  'ifndef ARPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ARPROG=$(CROSSBINDIR)/$(ARNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ARPROG=$(ARNAME)'#013#010+
  'endi','f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Override defaults'#013#010+
  'AS=$(ASPROG)'#013#010+
  'LD=$(LDPROG)'#013#010+
  'RC=$(RCPROG)'#013#010+
  'AR=$(ARPROG)'#013#010+
  #013#010+
  '# ppas.bat / ppas.sh'#013#010+
  'PPAS=ppas$(SRCBATCHEXT)'#013#010+
  #013#010+
  '# ldconfig to rebuild .so cache'#013#010+
  'ifdef inUnix'#013#010+
  'LDCONFIG=ldconfig'#013#010+
  'else'#013#010+
  'LDCONFIG='#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef DA','TE'#013#010+
  'DATESTR:=$(shell $(DATE) +%Y%m%d)'#013#010+
  'else'#013#010+
  'DATESTR='#013#010+
  'endif'#013#010+
  #013#010+
  '# Look if UPX is found for go32v2 and win32. We can'#039't use $UPX bec'+
  'ause'#013#010+
  '# upx uses that one itself (PFV)'#013#010+
  'ifndef UPXPROG'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifeq ($','(OS_TARGET),win32)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifdef UPXPROG'#013#010+
  'UPXPROG:=$(strip $(wildcard $(addsuffix /upx$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#013#010+
  'ifeq ($(UPXPROG),)'#013#010+
  'UPXPROG='#013#010+
  'else'#013#010+
  'UPXPROG:=$(firstword $(UPXPROG))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'UPXPROG='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'expo','rt UPXPROG'#013#010+
  #013#010+
  '# Zip options'#013#010+
  'ZIPOPT=-9'#013#010+
  'ZIPEXT=.zip'#013#010+
  #013#010+
  '# Tar options'#013#010+
  'ifeq ($(USETAR),bz2)'#013#010+
  'TAROPT=vj'#013#010+
  'TAREXT=.tar.bz2'#013#010+
  'else'#013#010+
  'TAROPT=vz'#013#010+
  'TAREXT=.tar.gz'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_begin]'#013#010+
  '###########################################################','#########'+
  '#'#013#010+
  '# Compiler Command Line'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#013+
  #010+
  '# compiler this needs to be turned off'#013#010+
  'ifndef NOCPUDEF'#013#010+
  'over','ride FPCOPTDEF=$(ARCH)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Load commandline OPT and add target and unit dir to be sure'#013#010+
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#013#010+
  'override FPCOPT+=-T$(OS_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#013#010+
  'override FPCOPT+=-P$(ARCH)'#013#010,
  'endif'#013#010+
  #013#010+
  'ifeq ($(OS_SOURCE),openbsd)'#013#010+
  'override FPCOPT+=-FD$(NEW_BINUTILS_PATH)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef CROSSBOOTSTRAP'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-XP$(BINUTILSPREFIX)'#013#010+
  'endif'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-Xr$(RLIN','KPATH)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# User dirs should be first, so they are looked at first'#013#010+
  'ifdef UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef LIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#013#010+
  'endif'#013#010+
  'ifdef OBJDIR'#013#010+
  'override F','PCOPT+=$(addprefix -Fo,$(OBJDIR))'#013#010+
  'endif'#013#010+
  'ifdef INCDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(INCDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking'#013#010+
  'ifdef LINKSMART'#013#010+
  'override FPCOPT+=-XX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking creation'#013#010+
  'ifdef CREATESMART'#013#010+
  'override FPCOPT+=-CX'#013#010,
  'endif'#013#010+
  #013#010+
  '# Debug'#013#010+
  'ifdef DEBUG'#013#010+
  'override FPCOPT+=-gl'#013#010+
  'override FPCOPTDEF+=DEBUG'#013#010+
  'endif'#013#010+
  #013#010+
  '# Release mode'#013#010+
  '# (strip, optimize and don'#039't load fpc.cfg)'#013#010+
  '# fpc 2.1 has -O2 for all targets'#013#010+
  'ifdef RELEASE'#013#010+
  'ifneq ($(findstring 2.0.,$(FPC_VERSION)),',')'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'FPCCPUOPT:=-OG2p3'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc)'#013#010+
  'FPCCPUOPT:=-O1r'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'FPCCPUOPT:=-O2'#013#010+
  'endif'#013#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#013#010+
  'override FPCOPTDEF+=RELEASE'#013#010+
  'endif'#013#010+
  #013#010+
  '# Strip'#013#010+
  'ifdef STRIP'#013,#010+
  'override FPCOPT+=-Xs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Optimizer'#013#010+
  'ifdef OPTIMIZE'#013#010+
  'override FPCOPT+=-O2'#013#010+
  'endif'#013#010+
  #013#010+
  '# Verbose settings (warning,note,info)'#013#010+
  'ifdef VERBOSE'#013#010+
  'override FPCOPT+=-vwni'#013#010+
  'endif'#013#010+
  #013#010+
  '# Needed compiler options'#013#010+
  'ifdef COMPILER_OPTIONS'#013#010+
  'override ','FPCOPT+=$(COMPILER_OPTIONS)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_LIBRARYDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_OBJECTDIR'#013#010,
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_INCLUDEDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compiler utils'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'override FPCOPT+=-FD$(CROSSBINDIR)'#013#010+
  'end','if'#013#010+
  #013#010+
  '# Target dirs and the prefix to use for clean/install'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#013#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#013#010+
  'override TARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR',')/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Unit target dir (default is units/<cpu>-<os>/'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#013#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#013#010+
  'override UNITTARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override UNITTARGETDI','RPREFIX=$(COMPILER_UNITTARGETDIR)/'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef COMPILER_TARGETDIR'#013#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#013#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CREATESHARED'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  #013#010+
  'i','feq ($(CPU_TARGET),i386)'#013#010+
  'override FPCOPT+=-Aas'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  #013#010+
  '# create always pic'#039'ed code on x86_64'#013#010+
  'ifeq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'ifeq ($(CPU_TARGET),x86_64)'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  'endif'#013#010+
  'endif'#013,#010+
  'endif'#013#010+
  #013#010+
  'ifdef LINKSHARED'#013#010+
  '#!!! deactived for testing override FPCOPT+=-XD'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_libc]'#013#010+
  '# Add GCC lib path if asked'#013#010+
  'ifdef GCCLIBDIR'#013#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#013#010+
  'endif'#013#010+
  'ifdef OTHERLIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -','Fl,$(OTHERLIBDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_end]'#013#010+
  '# Add commandline options last so they can override'#013#010+
  'ifdef OPT'#013#010+
  'override FPCOPT+=$(OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add defines from FPCOPTDEF to FPCOPT'#013#010+
  'ifdef FPCOPTDEF'#013#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOP','TDEF))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Was a config file specified ?'#013#010+
  'ifdef CFGFILE'#013#010+
  'override FPCOPT+=@$(CFGFILE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use the environment to pass commandline options?'#013#010+
  'ifdef USEENV'#013#010+
  'override FPCEXTCMD:=$(FPCOPT)'#013#010+
  'override FPCOPT:=!FPCEXTCMD'#013#010+
  'export FPC','EXTCMD'#013#010+
  'endif'#013#010+
  #013#010+
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013#010+
  'override AFULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifneq ($(AFULL_TARGET),$(AFULL_SOURCE))'#013#010+
  'override ACROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef ACROSSCOMPILE'#013#010+
  'override FPCOPT+=$(CROSS','OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Compiler commandline'#013#010+
  'override COMPILER:=$(FPC) $(FPCOPT)'#013#010+
  #013#010+
  '# also call ppas if with command option -s'#013#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#013#010+
  'ifeq (,$(findstring -s ,$(COMPILER)))'#013#010+
  'EXECPPAS='#013#010+
  'else'#013#010+
  'ifeq ','($(FULL_SOURCE),$(FULL_TARGET))'#013#010+
  'ifdef RUNBATCH'#013#010+
  'EXECPPAS:=@$(RUNBATCH) $(PPAS)'#013#010+
  'else'#013#010+
  'EXECPPAS:=@$(PPAS)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[loaderrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Loaders'#013#010+
  '###########','#########################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_loaders'#013#010+
  #013#010+
  'ifneq ($(TARGET_LOADERS),)'#013#010+
  'override ALLTARGET+=fpc_loaders'#013#010+
  'override CLEANTARGET+=fpc_loaders_clean'#013#010+
  'override INSTALLTARGET+=fpc_loaders_install'#013#010+
  #013#010+
  'overri','de LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#013#010+
  'endif'#013#010+
  #013#010+
  '%$(OEXT): %$(LOADEREXT)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#013#010+
  'else'#013#010+
  '        $(AS) -o $*$(OEXT) $<'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders: $(C','OMPILER_UNITTARGETDIR) $(LOADEROFILES)'#013#010+
  #013#010+
  'fpc_loaders_clean:'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#013#010+
  'else'#013#010+
  '        -$(DEL) $(LOADEROFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders_install:'#013#010+
  '     ','   $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFIL'+
  'ES)) $(INSTALL_UNITDIR)'#013#010+
  'else'#013#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[unitrules]',#013#010+
  '#####################################################################'#013+
  #010+
  '# Units'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_units'#013#010+
  #013#010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#013#010+
  'override ALLTAR','GET+=fpc_units'#013#010+
  #013#010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_UNITS))'#013#010+
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#013#010+
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'override CLEANP','PUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#013#010+
  #013#010+
  #013#010+
  '[exerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Exes'#013#010+
  '################################','####################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_exes'#013#010+
  #013#010+
  '# Programs are not needed for a cross installation'#013#010+
  'ifndef CROSSINSTALL'#013#010+
  'ifneq ($(TARGET_PROGRAMS),)'#013#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#013#010+
  'override EXEOFIL','ES:=$(addsuffix $(OEXT),$(TARGET_PROGRAMS)) $(addpre'+
  'fix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
  'PROGRAMS)))'#013#010+
  #013#010+
  'override ALLTARGET+=fpc_exes'#013#010+
  'ove','rride INSTALLEXEFILES+=$(EXEFILES)'#013#010+
  'override CLEANEXEFILES+=$(EXEFILES) $(EXEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(add','suffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES)'#013+
  #010+
  #013#010+
  #013#010+
  '[rstrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Resource strings'#013#010,
  '#####################################################################'#013+
  #010+
  #013#010+
  'ifdef TARGET_RSTS'#013#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_RSTS))'#013#010+
  #013#010+
  'override CLEANRSTFILES+=$(RSTFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[examplerules]'#013#010+
  '#########################','###########################################'+
  '#'#013#010+
  '# Examples'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_examples'#013#010+
  #013#010+
  'ifneq ($(TARGET_EXAMPLES),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $','(addsuffix .pp,$(TARGET_EXA'+
  'MPLES)) $(addsuffix .pas,$(TARGET_EXAMPLES)) $(addsuffix .lpr,$(TARGET'+
  '_EXAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#013#010+
  'override EXAMPLEFILES:=$(addsuffix $(EXEEXT),$(TARGET_EXAMPLES))'#013#010+
  'override EXAMPLEOFILES:=$(','addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(ad'+
  'dprefix $(STATICLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPL'+
  'ES))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TAR'+
  'GET_EXAMPLES)))'#013#010+
  #013#010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES)',' $(EXAMPLEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifneq ($(TARGE','T_EXAMPLEDIRS),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_examples: all $(EXAMPLEFILES) $(addsuffix _all,$(TARGET_EXAMPLEDIR'+
  'S))'#013#010+
  #013#010+
  #013#010+
  '[compilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# General compile rules'#013#010+
  '#######','#############################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_all fpc_smart fpc_debug fpc_release fpc_shared'#013#010+
  #013#010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#013#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMADE)'#013#010+
  #013#010+
  'fpc_all: $(FPCMADE)'#013,#010+
  #013#010+
  'fpc_smart:'#013#010+
  '        $(MAKE) all LINKSMART=1 CREATESMART=1'#013#010+
  #013#010+
  'fpc_debug:'#013#010+
  '        $(MAKE) all DEBUG=1'#013#010+
  #013#010+
  'fpc_release:'#013#010+
  '        $(MAKE) all RELEASE=1'#013#010+
  #013#010+
  '# General compile rules, available for both possible .pp and .pas exte'+
  'nsions'#013#010+
  #013#010+
  '.SUFFI','XES: $(EXEEXT) $(PPUEXT) $(OEXT) .pas .lpr .dpr .pp .rc .res'#013+
  #010+
  #013#010+
  '$(COMPILER_UNITTARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '$(COMPILER_TARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pp'#013#010+
  '        $(','COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXE','EXT): %.lpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.dpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%.res: %.rc'#013#010+
  '        windres -i $< -o $@'#013#010+
  #013#010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#013#010+
  'vpath %.pp $(COMPILER_','SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.pas $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.inc $(COMPILER_INCLUDED','IR)'#013#010+
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  'vpath %$(PPUEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '[sharedrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Library'#013#010+
  '##################################################','##################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_shared'#013#010+
  #013#010+
  'override INSTALLTARGET+=fpc_shared_install'#013#010+
  #013#010+
  'ifndef SHARED_LIBVERSION'#013#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_LIBNAME'#013#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_FU','LLNAME'#013#010+
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED_LIBNAME)-$(SHARED_LIBVERSIO'+
  'N)$(SHAREDLIBEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default sharedlib units are all unit objects'#013#010+
  'ifndef SHARED_LIBUNITS'#013#010+
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#013#010+
  'over','ride SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),$(SHARED'+
  '_LIBUNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared:'#013#010+
  'ifdef HASSHAREDLIB'#013#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  '        $(PPUMOVE) -q $(SHARED_','LIBUNITS) -i$(COMPILER_UNITTARGETDIR)'+
  ' -o$(SHARED_FULLNAME) -d$(COMPILER_UNITTARGETDIR)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '        @$(ECHO) Shared Libraries not supported'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared_install:'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  'ifneq ($(SHARED_LIBUNITS),)'#013#010+
  'ifne','q ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME)),)'#013+
  #010+
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[installrules]'#013#010+
  '####################################################','################'+
  '#'#013#010+
  '# Install rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#013#010+
  #013#010+
  'ifdef INSTALL_UNITS'#013#010+
  'override INSTALLPPUFILES+=$(addsuffix $(PPUEXT),$(','INSTALL_UNITS))'#013+
  #010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALL_BUILDUNIT'#013#010+
  'override INSTALLPPUFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT),$'+
  '(INSTALLPPUFILES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPP','U'+
  'FILES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEX'+
  'T),$(INSTALLPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#013#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#013#010+
  'override INSTALLPPUFILES:','=$(addprefix $(UNITTARGETDIRPREFIX),$(notdi'+
  'r $(INSTALLPPUFILES)))'#013#010+
  'override INSTALLPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPR'+
  'EFIX),$(notdir $(INSTALLPPULINKFILES))))'#013#010+
  'endif'#013#010+
  '# Implicitly install Package.fpc'#013#010+
  'override INSTALL_CRE','ATEPACKAGEFPC=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  'ifneq ($(TARGETDIRPREFIX),)'#013#010+
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_install: all $(INSTALLTARGET)'#013#010+
  'ifdef INSTALLEXEFILES'#013#010,
  '        $(MKDIR) $(INSTALL_BINDIR)'#013#010+
  '# Compress the exes if upx is defined'#013#010+
  'ifdef UPXPROG'#013#010+
  '        -$(UPXPROG) $(INSTALLEXEFILES)'#013#010+
  'endif'#013#010+
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#013#010+
  'ifde','f FPCMAKE'#013#010+
  '# If the fpcpackage variable is set then create and install Package.fp'+
  'c,'#013#010+
  '# a safety check is done if Makefile.fpc is available'#013#010+
  'ifdef PACKAGE_VERSION'#013#010+
  'ifneq ($(wildcard Makefile.fpc),)'#013#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS','_TARGET) Makefile.fpc'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) Package.fpc $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(I','NSTALL_UNITDIR)'#013#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#013#010+
  '        $(INSTALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'ifneq ($(wildcard $(LIB_FULLNAME)),)'#013#010+
  '        $(MKDIR) $(INSTALL_LIBDIR)'#013#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LI','BDIR)'#013#010+
  'ifdef inUnix'#013#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_LIBDIR)/$(LIB_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FILES'#013#010+
  '        $(MKDIR) $(INSTALL_DATADIR)'#013#010+
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_sourceins','tall: distclean'#013#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#013#010+
  '        $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#013#010+
  #013#010+
  'fpc_exampleinstall: $(addsuffix _distclean,$(TARGET_EXAMPLEDIRS))'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#013#010+
  'e','ndif'#013#010+
  'ifdef EXAMPLESOURCEFILES'#013#010+
  '        $(COPY) $(EXAMPLESOURCEFILES) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_EX'+
  'AMPLEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '[distinstallrules]'#013#010+
  '####','################################################################'+
  '#'#013#010+
  '# Dist Install'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_distinstall'#013#010+
  #013#010+
  'fpc_distinstall: install exampleinstall'#013#010+
  #013#010+
  #013#010+
  '[zipinstallr','ules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Zip'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#013#010+
  #013#010+
  '# Temporary p','ath to pack a file, can only use a single deep'#013#010+
  '# subdir, because the deltree can'#039't see the whole tree to remove'#013+
  #010+
  'ifndef PACKDIR'#013#010+
  'ifndef inUnix'#013#010+
  'PACKDIR=$(BASEDIR)/../fpc-pack'#013#010+
  'else'#013#010+
  'PACKDIR=/tmp/fpc-pack'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Maybe create def','ault zipname from packagename'#013#010+
  'ifndef ZIPNAME'#013#010+
  'ifdef DIST_ZIPNAME'#013#010+
  'ZIPNAME=$(DIST_ZIPNAME)'#013#010+
  'else'#013#010+
  'ZIPNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef FULLZIPNAME'#013#010+
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  '# ZipTa','rget'#013#010+
  'ifndef ZIPTARGET'#013#010+
  'ifdef DIST_ZIPTARGET'#013#010+
  'ZIPTARGET=DIST_ZIPTARGET'#013#010+
  'else'#013#010+
  'ZIPTARGET=install'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use tar by default under linux'#013#010+
  'ifndef USEZIP'#013#010+
  'ifdef inUnix'#013#010+
  'USETAR=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use a wrapper script by default for ','Os/2'#013#010+
  'ifndef inUnix'#013#010+
  'USEZIPWRAPPER=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# We need to be able to run in the current OS so fix'#013#010+
  '# the path separator'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  'ZIPPATHSEP=$(PATHSEP)'#013#010+
  'ZIPWRAPPER=$(subst /,$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT))'#013+
  #010+
  'el','se'#013#010+
  'ZIPPATHSEP=/'#013#010+
  'endif'#013#010+
  #013#010+
  '# Create commands to create the zip/tar file'#013#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#013#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#013#010+
  'ifdef USETAR'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)','$(TAREXT)'#013#010+
  'ZIPCMD_ZIP:=$(TARPROG) c$(TAROPT)f $(ZIPDESTFILE) *'#013#010+
  'else'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#013#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
>>>>>>> origin/cpstrnew
  'TFILE) *'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipinstall:'#013#010+
  '        $(','MAKE) $(ZIPTARGET) INSTALL_PREFIX=$(PACKDIR) ZIPINSTALL=1'#013+
  #010+
  '        $(MKDIR) $(DIST_DESTDIR)'#013#010+
  '        $(DEL) $(ZIPDESTFILE)'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  '# Handle gecho separate as we need to espace \ with \\'#013#010+
  'ifneq ($(ECHOREDIR),echo)'#013#010+
  '        $(EC','HOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDPACK))" > $(ZIPWRAP'+
  'PER)'#013#010+
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPER)'+
  #013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDBASE))" >> $(ZIPWRAPP'+
<<<<<<< HEAD
  'ER)'#010+
  'else'#010+
<<<<<<< HEAD
  '       ',' echo $(ZIPCMD_CDPACK) > $(ZIPWRAPPER)'#010+
=======
  '        echo $(ZIPCMD_CDPACK) > $(ZIPWRAPP','ER)'#010+
>>>>>>> origin/fixes_2.4
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPER)'#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#010+
  'endif'#010+
  'ifdef inUnix'#010+
  '        /bin/sh $(ZIPWRAPPER)'#010+
  'else'#010+
  'ifdef RUNBATCH'#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)'#010+
  'else'#010+
  '  ','      $(ZIPWRAPPER)'#010+
  'endif'#010+
  'endif'#010+
<<<<<<< HEAD
  '        $(DEL) $(ZIPWRAPPER)'#010+
=======
  '   ','     $(DEL) $(ZIPWRAPPER)'#010+
>>>>>>> origin/fixes_2.4
  'else'#010+
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#010+
  'endif'#010+
  '        $(DELTREE) $(PACKDIR)'#010+
  #010+
  'fpc_zipsourceinstall:'#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=sourceinstall ','ZIPSUFFIX=$(Z'+
  'IPSOURCESUFFIX)'#010+
  #010+
<<<<<<< HEAD
  'fpc_zipexampleinstall:'#010+
=======
  'fpc_z','ipexampleinstall:'#010+
>>>>>>> origin/fixes_2.4
  'ifdef HASEXAMPLES'#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#010+
  'endif'#010+
  #010+
  'fpc_zipdistinstall:'#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=distinstall'#010+
  #010+
  #010+
<<<<<<< HEAD
  '[','cleanrules]'#010+
  '#####################################################################'#010+
=======
  '[cleanrules]'#010+
  '#######################','#############################################'+
  '#'#010+
>>>>>>> origin/fixes_2.4
  '# Clean rules'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#010+
  #010+
  'ifdef EXEFILES'#010+
<<<<<<< HEAD
  'override CLE','ANEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEFIL'+
=======
  'override CLEANEXEFILES:=$(addprefix $(TARGETDIR','PREFIX),$(CLEANEXEFIL'+
>>>>>>> origin/fixes_2.4
  'ES))'#010+
  'override CLEANEXEDBGFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEDB'+
  'GFILES))'#010+
  'endif'#010+
  #010+
  'ifdef CLEAN_PROGRAMS'#010+
  'override CLEANEXEFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuffix $(E'+
  'XE','EXT), $(CLEAN_PROGRAMS)))'#010+
  'override CLEANEXEDBGFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuffix '+
  '$(EXEDBGEXT), $(CLEAN_PROGRAMS)))'#010+
  'endif'#010+
  #010+
  'ifdef CLEAN_UNITS'#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#010+
  'endif'#010+
  #010+
  'ifdef CLEANPPUFI','LES'#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
<<<<<<< HEAD
  ') $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(C'+
  'LEANPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(STA'+
  'TICLIBEXT),$(CLEANPPUFILES',')))'#010+
  'ifdef DEBUGSYMEXT'#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#010+
  'endif'#010+
  'override CLEANPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(CLEANPPUF'+
  'ILES))'#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNI','TTARGETDIRPR'+
  'EFIX),$(CLEANPPULINKFILES)))'#010+
=======
  ') $(addprefix $(STATICLIBPREFI','X),$(subst $(PPUEXT),$(STATICLIBEXT),$'+
  '(CLEANPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(S'+
  'TATICLIBEXT),$(CLEANPPUFILES)))'#010+
  'ifdef DEBUGSYMEXT'#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#010+
  'end','if'#010+
  'override CLEANPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(CLEANPPUF'+
  'ILES))'#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPREF'+
  'IX),$(CLEANPPULINKFILES)))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  'fpc_clean: $(CLEANTARGET)'#010+
  'ifdef CLEANEXEFILES'#010+
<<<<<<< HEAD
  '        -$(DEL) $(CLEANEXEFILES)'#010+
  'endif'#010+
  '# DELTREE instead of DEL because on Mac OS X these are directories'#010+
  'ifdef CLEANEXEDBGFILES'#010+
  '        -$(DELTRE','E) $(CLEANEXEDBGFILES)'#010+
=======
  '        -$','(DEL) $(CLEANEXEFILES)'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  'ifdef CLEANPPUFILES'#010+
  '        -$(DEL) $(CLEANPPUFILES)'#010+
  'endif'#010+
  'ifneq ($(CLEANPPULINKFILES),)'#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#010+
  'endif'#010+
  'ifdef CLEANRSTFILES'#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANR','STFILES))'+
  #010+
  'endif'#010+
  'ifdef CLEAN_FILES'#010+
  '        -$(DEL) $(CLEAN_FILES)'#010+
  'endif'#010+
  'ifdef LIB_NAME'#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#010+
  'endif'#010+
  '        -$(DEL) $(FPCMADE) Package.fpc $(PPAS) script.res link.res $(F'+
  'PCEXTFILE) $(REDIRFILE)'#010+
  '        -$','(DEL) *$(ASMEXT) *_ppas$(BATCHEXT)'#010+
  #010+
  'fpc_cleanall: $(CLEANTARGET)'#010+
  'ifdef CLEANEXEFILES'#010+
  '        -$(DEL) $(CLEANEXEFILES)'#010+
  'endif'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
  'ifdef CLEANPPUFILES'#010+
  '        -$(DEL) $(CLEANPPUFILES)'#010+
  'endif'#010+
  'ifneq ($(CLEANPPULINKFILES),',')'#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#010+
  'endif'#010+
  'ifdef CLEANRSTFILES'#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#010+
  'endif'#010+
  'endif'#010+
  'ifdef CLEAN_FILES'#010+
  '        -$(DEL) $(CLEAN_FILES)'#010+
  'endif'#010+
  '        -$(DELTREE) units'#010+
<<<<<<< HEAD
  '        -$(DE','L) *$(OEXT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL'+
  'IBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#010+
=======
  '        -$(DEL) *$(OEXT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL','I'+
  'BEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#010+
>>>>>>> origin/fixes_2.4
  'ifneq ($(PPUEXT),.ppu)'#010+
  '        -$(DEL) *.o *.ppu *.a'#010+
  'endif'#010+
  '        -$(DELTREE) *$(SMARTEXT)'#010+
<<<<<<< HEAD
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) script.res l','ink.res $('+
  'FPCEXTFILE) $(REDIRFILE)'#010+
  '        -$(DEL) *_ppas$(BATCHEXT)'#010+
=======
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) script.res link.res $(FP'+
  'CEXTFILE) $(REDIRFILE)'#010+
  '        -$(DEL) *_pp','as$(BATCHEXT)'#010+
>>>>>>> origin/fixes_2.4
  'ifdef AOUTEXT'#010+
  '        -$(DEL) *$(AOUTEXT)'#010+
  'endif'#010+
  'ifdef DEBUGSYMEXT'#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#010+
  'endif'#010+
  'ifdef LOCALFPMAKEBIN'#010+
  '        -$(DEL) $(LOCALFPMAKEBIN)'#010+
  '        -$(D','EL) $(FPMAKEBINOBJ)'#010+
  'endif'#010+
  #010+
  'fpc_distclean: cleanall'#010+
  #010+
  #010+
  '[baseinforules]'#010+
  '#####################################################################'#010+
<<<<<<< HEAD
  '# Base info rules'#010+
=======
  '# Base in','fo rules'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_','baseinfo'#010+
  #010+
  'override INFORULES+=fpc_baseinfo'#010+
  #010+
  'fpc_baseinfo:'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Package info =='#010+
<<<<<<< HEAD
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#010+
=======
  '        @$(ECHO)  Package Name..... ','$(PACKAGE_NAME)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#010+
  '        @$(ECHO)'#010+
  '    ','    @$(ECHO)  == Configuration info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  FPC.......... $(FPC)'#010+
  '        @$(ECHO)  FPC Version.. $(FPC_VERSION)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#010+
=======
  '      ','  @$(ECHO)  Source CPU... $(CPU_SOURCE)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#010+
  '      ','  @$(ECHO)  Source OS.... $(OS_SOURCE)'#010+
  '        @$(ECHO)  Target OS.... $(OS_TARGET)'#010+
  '        @$(ECHO)  Full Source.. $(FULL_SOURCE)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#010+
=======
  '        @$(ECHO) ',' Full Target.. $(FULL_TARGET)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#010+
  '        @$(ECH','O)  TargetSuffix. $(TARGETSUFFIX)'#010+
  '        @$(ECHO)  FPC fpmake... $(FPCFPMAKE)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Directory info =='#010+
  '        @$(ECHO)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#010+
=======
  '        @$(ECHO)  Required pkgs... $(REQU','IRE_PACKAGES)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)'#010+
  '        @$(ECHO)','  Basedir......... $(BASEDIR)'#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#010+
  '        @$(ECHO)  CrossBinDir..... $(CROSSBINDIR)'#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#010+
=======
  '        @$(ECHO)  Pac','kagesDir..... $(PACKAGESDIR)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)'#010+
  ' ','       @$(ECHO)  GCC library..... $(GCCLIBDIR)'#010+
  '        @$(ECHO)  Other library... $(OTHERLIBDIR)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Tools info =='#010+
  '        @$(ECHO)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  As........ $(AS)'#010+
=======
  '        @$(ECHO)  As.......','. $(AS)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  Ld........ $(LD)'#010+
  '    ','    @$(ECHO)  Ar........ $(AR)'#010+
  '        @$(ECHO)  Rc........ $(RC)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Mv........ $(MVPROG)'#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  Rm........ $(RMPROG)'#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)',#010+
=======
  '        @$(ECHO)  Rm........ $(R','MPROG)'#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  Echo...... $(ECHO)'#010+
  '        @$(ECHO)  Shell..... $(SHELL)'#010+
  '        @$(ECHO)  Date...... $(DATE)'#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#010+
=======
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#010,
  '        @$(ECHO)  Upx....... $(UPXPROG)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#010+
  '       ',' @$(ECHO)'#010+
  '        @$(ECHO)  == Object info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Target Loaders........ $(TARGET_LOADERS)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNI','TS)'#010+
  '        @$(ECHO)  Target Programs....... $(TARGET_PROGRAMS)'#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIR','S)'#010+
=======
  '        @$(ECHO)  Target Units.','......... $(TARGET_UNITS)'#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#010+
  '        @$(ECHO)  Target Programs....... $(TARGET_PROGRAMS)'#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#010+
  '        @$(ECHO)  Target Examples','....... $(TARGET_EXAMPLES)'#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#010+
  '        @$(ECHO)  Clean Files......... $(CLEAN_FILES)'#010+
  '        @$(ECHO)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#010+
  '        @$(ECHO)  Install Files....... ','$(INSTALL_FILES)'#010+
=======
  '        ','@$(ECHO)  Install Units....... $(INSTALL_UNITS)'#010+
  '        @$(ECHO)  Install Files....... $(INSTALL_FILES)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Install info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  DateStr.............. $(DATESTR)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPREFI','X)'#010+
  '        @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#010+
=======
  '        @$(ECH','O)  ZipName.............. $(ZIPNAME)'#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPREFIX)'#010+
  '        @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#010+
  '        @$(ECHO)  FullZipName..........',' $(FULLZIPNAME)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#010+
  '        @','$(ECHO)'#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#010+
  '        @$(ECHO)  Install binary dir... $(INSTALL_BINDIR)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#010+
  '        @$(ECHO)  Install units dir.... $(INSTALL_UNITDIR',')'#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#010+
  '        @$(ECHO)  Install data dir..... $(INSTALL_DATADIR',')'#010+
=======
  '        @$(ECHO)  Install li','brary dir.. $(INSTALL_LIBDIR)'#010+
  '        @$(ECHO)  Install units dir.... $(INSTALL_UNITDIR)'#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#010+
  '        @$(ECHO)  Install exampl','e dir.. $(INSTALL_EXAMPLEDIR)'#010+
  '        @$(ECHO)  Install data dir..... $(INSTALL_DATADIR)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#010+
  '        @$(ECHO)  Dist zip name........ $(DIST_ZIPNAME)'#010+
  '        @$(ECHO)'#010+
  #010+
<<<<<<< HEAD
  '[inforules]'#010+
=======
  '[inf','orules]'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  '# Info ru','les'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_info'#010+
  #010+
  'fpc_info: $(INFORULES)'#010+
  #010+
  '[makefilerules]'#010+
<<<<<<< HEAD
=======
  '####################','################################################'+
  '#'#010+
  '# Rebuild Makefile'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  '# Rebuild Makefile'#010+
  '##################','##################################################'+
  '#'#010+
  #010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1 fpc_makefile_sub2'+
  ' \'#010+
  '        fpc_makefile_dirs'#010+
<<<<<<< HEAD
  #010+
=======
  #010,
>>>>>>> origin/fixes_2.4
  'fpc_makefile:'#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) Makefile.fpc'#010+
  #010+
  'fpc_makefile_sub1:'#010+
  'ifd','ef TARGET_DIRS'#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_DIRS))'#010+
  'endif'#010+
  'ifdef TARGET_EXAMPLEDIRS'#010+
<<<<<<< HEAD
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_EXAMPLEDIRS))'#010+
=======
  '        $(FPCMAKE) -w -T$(OS','_TARGET) $(addsuffix /Makefile.fpc,$(TAR'+
  'GET_EXAMPLEDIRS))'#010+
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  'fpc_makefile_sub2: $(a','ddsuffix _makefile_dirs,$(TARGET_DIRS) $(TARGE'+
  'T_EXAMPLEDIRS))'#010+
  #010+
  'fpc_makefile_dirs: fpc_makefile_sub1 fpc_makefile_sub2'#010+
  #010+
<<<<<<< HEAD
  'fpc_makefiles: fpc_makefile fpc_makefile_dirs'#010+
=======
  'fpc_makefiles: fpc_makefile fpc_ma','kefile_dirs'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '[localmakefile]'#010+
  '##########################################################','##########'+
  '#'#010+
  '# Local Makefile'#010+
  '#####################################################################'#010+
  #010+
  'ifneq ($(wildcard fpcmake.loc),)'#010+
<<<<<<< HEAD
  'include fpcmake.loc'#010+
=======
  'include fpcmake.loc'#010,
>>>>>>> origin/fixes_2.4
  'endif'#010+
  #010+
  #010+
  '[userrules]'#010+
  '###################################################################','#'+
  '#'#010+
  '# Users rules'#010+
  '#####################################################################'#010+
  #010+
  '[lclrules]'#010+
<<<<<<< HEAD
=======
  '######################################################','##############'+
  '#'#010+
  '# LCL Rules'#010+
>>>>>>> origin/fixes_2.4
  '#####################################################################'#010+
  '# LCL Rules'#010+
  '###########################################################','#########'+
  '#'#010+
  #010+
  '# LCL Platform'#010+
  'ifndef LCL_PLATFORM'#010+
  'ifneq ($(findstring $(OS_TARGET),win32 win64),)'#010+
  'LCL_PLATFORM=win32'#010+
  'else'#010+
  'LCL_PLATFORM=gtk'#010+
  'endif'#010+
  'endif'#010+
<<<<<<< HEAD
  'export LCL_PLATFORM'#010+
=======
  'expor','t LCL_PLATFORM'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '# Check if the specified LCLDIR is correct'#010+
  'ifdef LCLDIR'#010+
  'override LCLDIR',':=$(subst \,/,$(LCLDIR))'#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  'else'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  #010+
<<<<<<< HEAD
  '# Check if the default LCLDIR is correct'#010+
=======
  '# Check if the',' default LCLDIR is correct'#010+
>>>>>>> origin/fixes_2.4
  'ifdef DEFAULT_LCLDIR'#010+
  'override LCLDIR:=$(subst \,/,$(DEFAULT_','LCLDIR))'#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Check for development version'#010+
  'ifeq ($(LCLDIR),wrong)'#010+
<<<<<<< HEAD
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
  ' $(addsuffix /units','/$(LCL_PLATFORM),$(BASEDIR)/lcl $(BASEDIR)))))'#010+
=======
  'ov','erride LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildca'+
  'rd $(addsuffix /units/$(LCL_PLATFORM),$(BASEDIR)/lcl $(BASEDIR)))))'#010+
>>>>>>> origin/fixes_2.4
  'ifeq ($(LCLDIR),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Check for release version'#010+
  'ifeq ($(LCLDIR),wrong)'#010+
<<<<<<< HEAD
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
  ' $(addsuffix /lib/l','azarus/units/$(LCL_PLATFORM),/usr/local /usr))))'#010+
=======
  'ov','erride LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildca'+
  'rd $(addsuffix /lib/lazarus/units/$(LCL_PLATFORM),/usr/local /usr))))'#010+
>>>>>>> origin/fixes_2.4
  'ifeq ($(LCLDIR),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Generate dirs'#010+
<<<<<<< HEAD
  'override LCLUNITDIR:=$(wildcard $(LCLDIR)/units/$(LCL_PLATFORM) $(LCLD'+
  'IR)/units)'#010+
  'override LCLCOMPONENTDIR:=$(wildcard $(LC','LDIR)/.. $(LCLDIR)/../compo'+
  'nents $(LCLDIR)/components)'#010+
  'export LCLDIR LCLUNITDIR LCLCOMPONENTDIR'#010+
  #010+
  '# Add LCL dirs to paths'#010+
  'override REQUIRE_PACKAGESDIR+=$(LCLCOMPONENTDIR)'#010+
=======
  'override LCLUNITDIR:=$(wildcard $(L','CLDIR)/units/$(LCL_PLATFORM) $(LC'+
  'LDIR)/units)'#010+
  'override LCLCOMPONENTDIR:=$(wildcard $(LCLDIR)/.. $(LCLDIR)/../compone'+
  'nts $(LCLDIR)/components)'#010+
  'export LCLDIR LCLUNITDIR LCLCOMPONENTDIR'#010+
  #010+
  '# Add LCL dirs to paths'#010+
  'override REQUIRE_PACKAGESDIR+=$(','LCLCOMPONENTDIR)'#010+
>>>>>>> origin/fixes_2.4
  'override COMPILER_UNITDIR+=$(LCLUNITDIR)'#010+
  #010+
  '[lclinforules]'#010+
  '#############','#######################################################'+
  '#'#010+
  '# LCL Info rules'#010+
  '#####################################################################'#010+
<<<<<<< HEAD
  'override INFORULES+=lclinfo'#010+
=======
  'override ','INFORULES+=lclinfo'#010+
>>>>>>> origin/fixes_2.4
  #010+
  '.PHONY: lclinfo'#010+
  #010+
  'lclinfo:'#010+
  '        @$(ECHO)  == LCL info =='#010+
  '        ','@$(ECHO)'#010+
  '        @$(ECHO)  Platform............. $(LCL_PLATFORM)'#010+
  '        @$(ECHO)  LCLDIR............... $(LCLDIR)'#010+
<<<<<<< HEAD
  '        @$(ECHO)  LCL Unit dir......... $(LCLUNITDIR)'#010+
=======
  '        @$(ECHO)  LCL Unit dir........','. $(LCLUNITDIR)'#010+
>>>>>>> origin/fixes_2.4
  '        @$(ECHO)  LCL Component dir.... $(LCLCOMPONENTDIR)'#010+
  '        @$(E','CHO)'#010+
  #010+
  '[fpmakeprerules]'#010+
  '#####################################################################'#010+
  '# fpmake prerules'#010+
  '#####################################################################'#010+
  'FPMAKEBIN=fpmake$(SRCEXEEXT)'#010+
  'FPMAKEBINOBJ=fpmake$(OEXT)'#010+
  'LOC','ALFPMAKEBIN=.$(PATHSEP)$(FPMAKEBIN)'#010+
  #010+
  '# Convert the OS_TARGET and CPU_TARGET options to fpmake'#039's --os an'+
  'd --cpu parameters'#010+
  'ifdef OS_TARGET'#010+
  'FPC_TARGETOPT+=--os=$(OS_TARGET)'#010+
  'endif'#010+
  'ifdef CPU_TARGET'#010+
  'FPC_TARGETOPT+=--cpu=$(CPU_TARGET)'#010+
  'endif'#010+
  #010+
  '# Ge','t the location of the bootstrap-fpmkunit units'#010+
  'PACKAGEDIR_FPMKUNIT:=$(firstword $(subst /Makefile.fpc,,$(strip $(wild'+
  'card $(addsuffix /fpmkunit/Makefile.fpc,$(PACKAGESDIR))))))'#010+
  'ifneq ($(PACKAGEDIR_FPMKUNIT),)'#010+
  'UNITDIR_FPMAKE_FPMKUNIT=$(PACKA','GEDIR_FPMKUNIT)/units_bs/$(SOURCESUFF'+
  'IX)'#010+
  'override COMPILER_FPMAKE_UNITDIR=$(UNITDIR_FPMAKE_FPMKUNIT)'#010+
  'FPMKUNIT_SRC=$(PACKAGEDIR_FPMKUNIT)/src/fpmkunit.pp'#010+
  'FPMKUNIT_PPU=$(UNITDIR_FPMAKE_FPMKUNIT)/fpmkunit.ppu'#010+
  'endif'#010+
  #010+
  'ifdef FPMAKE_SKIP_CONFIG'#010+
  'ov','erride FPMAKE_BUILD_OPT+=$(FPMAKE_SKIP_CONFIG)'#010+
  'endif'#010+
  #010+
  '[fpmakerules]'#010+
  '#####################################################################'#010+
  '# fpmake rules'#010+
  '#####################################################################'#010+
  '.PHONY: fpc_fpmak','e fpc_fpmake_clean fpc_fpmake_install fpc_fpmake_ex'+
  'ampleinstall'#010+
  #010+
  '# Do not pass the Makefile'#039's unit and binary target locations. fpm'+
  'ake uses it'#039's own.'#010+
  'override FPCOPT:=$(filter-out -FU%,$(FPCOPT))'#010+
  'override FPCOPT:=$(filter-out -FE%,$(FPCOPT)',')'#010+
  '# Compose general fpmake-parameters'#010+
  'ifdef FPMAKEOPT'#010+
  'FPMAKE_OPT+=$(FPMAKEOPT)'#010+
  'endif'#010+
  'FPMAKE_OPT+=--localunitdir=$(FPCDIR)'#010+
  'FPMAKE_OPT+=--globalunitdir=$(FPCDIR)/packages'#010+
  'FPMAKE_OPT+=$(FPC_TARGETOPT)'#010+
  'FPMAKE_OPT+=$(addprefix -o ,$(FPCOPT))'#010+
  'FPM','AKE_OPT+=--compiler=$(FPC)'#010+
  'FPMAKE_OPT+=-bu'#010+
  #010+
  'FPMAKE_INSTALL_OPT+=--unitinstalldir=$(INSTALL_UNITDIR)'#010+
  'ifdef UNIXHier'#010+
  'FPMAKE_INSTALL_OPT+=--prefix=$(INSTALL_PREFIX)'#010+
  'FPMAKE_INSTALL_OPT+=--baseinstalldir=$(INSTALL_LIBDIR)/fpc/$(FPC_VERSI'+
  'ON)'#010+
  'else',#010+
  'FPMAKE_INSTALL_OPT+=--prefix=$(INSTALL_BASEDIR)'#010+
  'endif'#010+
  #010+
  'override ALLTARGET+=fpc_fpmake'#010+
  'override INSTALLTARGET+=fpc_fpmake_install'#010+
  'override EXAMPLEINSTALLTARGET+=fpc_fpmake_exampleinstall'#010+
  '# If no fpmake exists and (dist)clean is called, do n','ot try to build'+
  ' fpmake, it will'#010+
  '# most often fail because the dependencies are cleared.'#010+
  '# In case of a clean, simply do nothing'#010+
  'ifneq ($(wildcard $(LOCALFPMAKEBIN)),)'#010+
  'override CLEANTARGET+=fpc_fpmake_clean'#010+
  'endif'#010+
  #010+
  '$(FPMKUNIT_PPU): $(FPMKUNIT','_SRC)'#010+
  '            $(MAKE) -C $(PACKAGEDIR_FPMKUNIT) bootstrap $(addprefix OP'+
  'T=,$(FPMAKE_BUILD_OPT))'#010+
  #010+
  '$(FPMAKEBIN): fpmake.pp $(FPMKUNIT_PPU)'#010+
  '        $(FPCFPMAKE) fpmake.pp $(addprefix -Fu,$(COMPILER_FPMAKE_UNITD'+
  'IR)) $(FPMAKE_BUILD_OPT)'#010+
  #010+
  'fpc','_fpmake: $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) compile $(FPMAKE_OPT)'#010+
  #010+
  'fpc_fpmake_clean: $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) clean $(FPMAKE_OPT)'#010+
  #010+
  'fpc_fpmake_install: $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) install $(FPMAKE_OPT',') $(FPMAKE_INSTALL'+
  '_OPT)'#010+
  #010+
  '# This is not completely valid. Exampleinstall should only install the'+
  ' examples, while'#010+
  '# fpmake -ie installs everything, including the examples. This also me'+
  'ans that on'#010+
  '# a distinstall fpmake install wil be called t','wice.'#010+
  'fpc_fpmake_exampleinstall: $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) install -ie $(FPMAKE_OPT) $(FPMAKE_INSTA'+
  'LL_OPT)'#010
=======
  'ER)'#013#010+
  'else'#013#010+
  '        echo $(ZIPCMD_CDP','ACK) > $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'ifdef inUnix'#013#010+
  '        /bin/sh $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  'ifdef RUNBATCH'#013#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(','ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  '        $(DEL) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#013#010+
  'endif'#013#010+
  '        $(DELTREE) $(PACKDIR)'#013#010+
  #013#010+
  'fpc_zipsourceinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=sourceinstal','l ZIPSUFFIX=$(Z'+
  'IPSOURCESUFFIX)'#013#010+
  #013#010+
  'fpc_zipexampleinstall:'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipdistinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=disti','nstall'#013#010+
  #013#010+
  #013#010+
  '[cleanrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Clean rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#013#010+
  #013#010+
  'ifdef E','XEFILES'#013#010+
  'override CLEANEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEFILES'+
  '))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_PROGRAMS'#013#010+
  'override CLEANEXEFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuffix $(E'+
  'XEEXT), $(CLEAN_PROGRAMS)))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_UNITS'#013,#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEANPPUFILES'#013#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
  ') $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(C'+
  'LEANPP','UFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(S'+
  'TATICLIBEXT),$(CLEANPPUFILES)))'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#013#010+
  'endif'#013#010+
  'override CLEANPPUFILES:=$(addprefix $(U','NITTARGETDIRPREFIX),$(CLEANPP'+
  'UFILES))'#013#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPREF'+
  'IX),$(CLEANPPULINKFILES)))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_clean: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef',' CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'ifdef CLE','AN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  'ifdef LIB_NAME'#013#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#013#010+
  'endif'#013#010+
  '        -$(DEL) $(FPCMADE) Package.fpc $(PPAS) script.res link.res $(F'+
  'PCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *$(ASMEXT) *','_ppas$(BATCHEXT)'#013#010+
  #013#010+
  'fpc_cleanall: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '     ','   -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef CLEAN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) units'#013#010+
  '        -$','(DEL) *$(OEXT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL'+
  'IBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#013#010+
  'ifneq ($(PPUEXT),.ppu)'#013#010+
  '        -$(DEL) *.o *.ppu *.a'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) *$(SMARTEXT)'#013#010+
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) scri','pt.res link.res $('+
  'FPCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *_ppas$(BATCHEXT)'#013#010+
  'ifdef AOUTEXT'#013#010+
  '        -$(DEL) *$(AOUTEXT)'#013#010+
  'endif'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_distclean: cleanall'#013#010+
  #013#010+
  #013#010+
  '[baseinforules]'#013#010+
  '###','#################################################################'+
  '#'#013#010+
  '# Base info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_baseinfo'#013#010+
  #013#010+
  'override INFORULES+=fpc_baseinfo'#013#010+
  #013#010+
  'fpc_baseinfo:'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  == Package info =='#013#010+
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#013#010+
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Configuration info =='#013#010+
  '        @$(ECHO)'#013#010,
  '        @$(ECHO)  FPC.......... $(FPC)'#013#010+
  '        @$(ECHO)  FPC Version.. $(FPC_VERSION)'#013#010+
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#013#010+
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#013#010+
  '        @$(ECHO)  Source OS.... $(OS_SOURCE)'#013#010+
  '        @$(E','CHO)  Target OS.... $(OS_TARGET)'#013#010+
  '        @$(ECHO)  Full Source.. $(FULL_SOURCE)'#013#010+
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#013#010+
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#013#010+
  '        @$(ECHO)  TargetSuffix. $(TARGETSUFFIX)'#013#010+
  '        @$(E','CHO)'#013#010+
  '        @$(ECHO)  == Directory info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Basedir......... $(BASEDIR)'#013#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#013#010+
  '        @','$(ECHO)  CrossBinDir..... $(CROSSBINDIR)'#013#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#013#010+
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  GCC library..... $(GCCLIBDIR)'#013#010+
  '        @$(ECHO)  Other library.','.. $(OTHERLIBDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Tools info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  As........ $(AS)'#013#010+
  '        @$(ECHO)  Ld........ $(LD)'#013#010+
  '        @$(ECHO)  Ar........ $(AR)'#013#010+
  '        @$(ECHO)  Rc........ $(RC)'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  Mv........ $(MVPROG)'#013#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#013#010+
  '        @$(ECHO)  Rm........ $(RMPROG)'#013#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#013#010+
  '        @$(ECHO)  Echo...... $(ECHO)'#013#010+
  '        @$(ECHO)  Shell....','. $(SHELL)'#013#010+
  '        @$(ECHO)  Date...... $(DATE)'#013#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#013#010+
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#013#010+
  '        @$(ECHO)  Upx....... $(UPXPROG)'#013#010+
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#013#010+
  '        @$(ECHO)'#013#010+
  '        ','@$(ECHO)  == Object info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Target Loaders........ $(TARGET_LOADERS)'#013#010+
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#013#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#013#010+
  '        @','$(ECHO)  Target Programs....... $(TARGET_PROGRAMS)'#013#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#013#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#013#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#013#010+
  '      ','  @$(ECHO)'#013#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#013#010+
  '        @$(ECHO)  Clean Files......... $(CLEAN_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#013#010+
  '        @$(ECHO)  Install Files....... $(IN','STALL_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Install info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  DateStr.............. $(DATESTR)'#013#010+
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#013#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPRE','FIX)'#013#010+
  '        @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#013#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#013#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#013#010+
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#013#010+
  '  ','      @$(ECHO)'#013#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#013#010+
  '        @$(ECHO)  Install binary dir... $(INSTALL_BINDIR)'#013#010+
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#013#010+
  '        @$(ECHO)  Install units dir.... $(INST','ALL_UNITDIR)'#013#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#013#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#013#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#013#010+
  '        @$(ECHO)  Install data dir..... $(','INSTALL_DATADIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#013#010+
  '        @$(ECHO)  Dist zip name........ $(DIST_ZIPNAME)'#013#010+
  '        @$(ECHO)'#013#010+
  #013#010+
  '[inforules]'#013#010+
  '#########################################################','###########'+
  '#'#013#010+
  '# Info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_info'#013#010+
  #013#010+
  'fpc_info: $(INFORULES)'#013#010+
  #013#010+
  '[makefilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Reb','uild Makefile'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1 fpc_makefile_sub2'+
  ' \'#013#010+
  '        fpc_makefile_dirs'#013#010+
  #013#010+
  'fpc_makefile:'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGE','T) Makefile.fpc'#013#010+
  #013#010+
  'fpc_makefile_sub1:'#013#010+
  'ifdef TARGET_DIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
=======
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPER)'+
  #013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDBASE))" >> $(ZIPWRAPP'+
  'ER)'#013#010+
  'else'#013#010+
  '        echo $(ZIPCMD_CDP','ACK) > $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'ifdef inUnix'#013#010+
  '        /bin/sh $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  'ifdef RUNBATCH'#013#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(','ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  '        $(DEL) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#013#010+
  'endif'#013#010+
  '        $(DELTREE) $(PACKDIR)'#013#010+
  #013#010+
  'fpc_zipsourceinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=sourceinstal','l ZIPSUFFIX=$(Z'+
  'IPSOURCESUFFIX)'#013#010+
  #013#010+
  'fpc_zipexampleinstall:'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipdistinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=disti','nstall'#013#010+
  #013#010+
  #013#010+
  '[cleanrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Clean rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#013#010+
  #013#010+
  'ifdef E','XEFILES'#013#010+
  'override CLEANEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEFILES'+
  '))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_PROGRAMS'#013#010+
  'override CLEANEXEFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuffix $(E'+
  'XEEXT), $(CLEAN_PROGRAMS)))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_UNITS'#013,#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEANPPUFILES'#013#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
  ') $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(C'+
  'LEANPP','UFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(S'+
  'TATICLIBEXT),$(CLEANPPUFILES)))'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#013#010+
  'endif'#013#010+
  'override CLEANPPUFILES:=$(addprefix $(U','NITTARGETDIRPREFIX),$(CLEANPP'+
  'UFILES))'#013#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPREF'+
  'IX),$(CLEANPPULINKFILES)))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_clean: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef',' CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'ifdef CLE','AN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  'ifdef LIB_NAME'#013#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#013#010+
  'endif'#013#010+
  '        -$(DEL) $(FPCMADE) Package.fpc $(PPAS) script.res link.res $(F'+
  'PCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *$(ASMEXT) *','_ppas$(BATCHEXT)'#013#010+
  #013#010+
  'fpc_cleanall: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '     ','   -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef CLEAN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) units'#013#010+
  '        -$','(DEL) *$(OEXT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL'+
  'IBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#013#010+
  'ifneq ($(PPUEXT),.ppu)'#013#010+
  '        -$(DEL) *.o *.ppu *.a'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) *$(SMARTEXT)'#013#010+
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) scri','pt.res link.res $('+
  'FPCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *_ppas$(BATCHEXT)'#013#010+
  'ifdef AOUTEXT'#013#010+
  '        -$(DEL) *$(AOUTEXT)'#013#010+
  'endif'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_distclean: cleanall'#013#010+
  #013#010+
  #013#010+
  '[baseinforules]'#013#010+
  '###','#################################################################'+
  '#'#013#010+
  '# Base info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_baseinfo'#013#010+
  #013#010+
  'override INFORULES+=fpc_baseinfo'#013#010+
  #013#010+
  'fpc_baseinfo:'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  == Package info =='#013#010+
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#013#010+
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Configuration info =='#013#010+
  '        @$(ECHO)'#013#010,
  '        @$(ECHO)  FPC.......... $(FPC)'#013#010+
  '        @$(ECHO)  FPC Version.. $(FPC_VERSION)'#013#010+
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#013#010+
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#013#010+
  '        @$(ECHO)  Source OS.... $(OS_SOURCE)'#013#010+
  '        @$(E','CHO)  Target OS.... $(OS_TARGET)'#013#010+
  '        @$(ECHO)  Full Source.. $(FULL_SOURCE)'#013#010+
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#013#010+
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#013#010+
  '        @$(ECHO)  TargetSuffix. $(TARGETSUFFIX)'#013#010+
  '        @$(E','CHO)'#013#010+
  '        @$(ECHO)  == Directory info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Basedir......... $(BASEDIR)'#013#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#013#010+
  '        @','$(ECHO)  CrossBinDir..... $(CROSSBINDIR)'#013#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#013#010+
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  GCC library..... $(GCCLIBDIR)'#013#010+
  '        @$(ECHO)  Other library.','.. $(OTHERLIBDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Tools info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  As........ $(AS)'#013#010+
  '        @$(ECHO)  Ld........ $(LD)'#013#010+
  '        @$(ECHO)  Ar........ $(AR)'#013#010+
  '        @$(ECHO)  Rc........ $(RC)'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  Mv........ $(MVPROG)'#013#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#013#010+
  '        @$(ECHO)  Rm........ $(RMPROG)'#013#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#013#010+
  '        @$(ECHO)  Echo...... $(ECHO)'#013#010+
  '        @$(ECHO)  Shell....','. $(SHELL)'#013#010+
  '        @$(ECHO)  Date...... $(DATE)'#013#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#013#010+
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#013#010+
  '        @$(ECHO)  Upx....... $(UPXPROG)'#013#010+
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#013#010+
  '        @$(ECHO)'#013#010+
  '        ','@$(ECHO)  == Object info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Target Loaders........ $(TARGET_LOADERS)'#013#010+
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#013#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#013#010+
  '        @','$(ECHO)  Target Programs....... $(TARGET_PROGRAMS)'#013#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#013#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#013#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#013#010+
  '      ','  @$(ECHO)'#013#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#013#010+
  '        @$(ECHO)  Clean Files......... $(CLEAN_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#013#010+
  '        @$(ECHO)  Install Files....... $(IN','STALL_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Install info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  DateStr.............. $(DATESTR)'#013#010+
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#013#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPRE','FIX)'#013#010+
  '        @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#013#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#013#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#013#010+
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#013#010+
  '  ','      @$(ECHO)'#013#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#013#010+
  '        @$(ECHO)  Install binary dir... $(INSTALL_BINDIR)'#013#010+
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#013#010+
  '        @$(ECHO)  Install units dir.... $(INST','ALL_UNITDIR)'#013#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#013#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#013#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#013#010+
  '        @$(ECHO)  Install data dir..... $(','INSTALL_DATADIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#013#010+
  '        @$(ECHO)  Dist zip name........ $(DIST_ZIPNAME)'#013#010+
  '        @$(ECHO)'#013#010+
  #013#010+
  '[inforules]'#013#010+
  '#########################################################','###########'+
  '#'#013#010+
  '# Info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_info'#013#010+
  #013#010+
  'fpc_info: $(INFORULES)'#013#010+
  #013#010+
  '[makefilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Reb','uild Makefile'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1 fpc_makefile_sub2'+
  ' \'#013#010+
  '        fpc_makefile_dirs'#013#010+
  #013#010+
  'fpc_makefile:'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGE','T) Makefile.fpc'#013#010+
  #013#010+
  'fpc_makefile_sub1:'#013#010+
  'ifdef TARGET_DIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
>>>>>>> graemeg/cpstrnew
=======
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPER)'+
  #013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDBASE))" >> $(ZIPWRAPP'+
  'ER)'#013#010+
  'else'#013#010+
  '        echo $(ZIPCMD_CDP','ACK) > $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'ifdef inUnix'#013#010+
  '        /bin/sh $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  'ifdef RUNBATCH'#013#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(','ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  '        $(DEL) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#013#010+
  'endif'#013#010+
  '        $(DELTREE) $(PACKDIR)'#013#010+
  #013#010+
  'fpc_zipsourceinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=sourceinstal','l ZIPSUFFIX=$(Z'+
  'IPSOURCESUFFIX)'#013#010+
  #013#010+
  'fpc_zipexampleinstall:'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipdistinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=disti','nstall'#013#010+
  #013#010+
  #013#010+
  '[cleanrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Clean rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#013#010+
  #013#010+
  'ifdef E','XEFILES'#013#010+
  'override CLEANEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEFILES'+
  '))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_PROGRAMS'#013#010+
  'override CLEANEXEFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuffix $(E'+
  'XEEXT), $(CLEAN_PROGRAMS)))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_UNITS'#013,#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEANPPUFILES'#013#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
  ') $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(C'+
  'LEANPP','UFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(S'+
  'TATICLIBEXT),$(CLEANPPUFILES)))'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#013#010+
  'endif'#013#010+
  'override CLEANPPUFILES:=$(addprefix $(U','NITTARGETDIRPREFIX),$(CLEANPP'+
  'UFILES))'#013#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPREF'+
  'IX),$(CLEANPPULINKFILES)))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_clean: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef',' CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'ifdef CLE','AN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  'ifdef LIB_NAME'#013#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#013#010+
  'endif'#013#010+
  '        -$(DEL) $(FPCMADE) Package.fpc $(PPAS) script.res link.res $(F'+
  'PCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *$(ASMEXT) *','_ppas$(BATCHEXT)'#013#010+
  #013#010+
  'fpc_cleanall: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '     ','   -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef CLEAN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) units'#013#010+
  '        -$','(DEL) *$(OEXT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL'+
  'IBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#013#010+
  'ifneq ($(PPUEXT),.ppu)'#013#010+
  '        -$(DEL) *.o *.ppu *.a'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) *$(SMARTEXT)'#013#010+
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) scri','pt.res link.res $('+
  'FPCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *_ppas$(BATCHEXT)'#013#010+
  'ifdef AOUTEXT'#013#010+
  '        -$(DEL) *$(AOUTEXT)'#013#010+
  'endif'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_distclean: cleanall'#013#010+
  #013#010+
  #013#010+
  '[baseinforules]'#013#010+
  '###','#################################################################'+
  '#'#013#010+
  '# Base info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_baseinfo'#013#010+
  #013#010+
  'override INFORULES+=fpc_baseinfo'#013#010+
  #013#010+
  'fpc_baseinfo:'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  == Package info =='#013#010+
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#013#010+
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Configuration info =='#013#010+
  '        @$(ECHO)'#013#010,
  '        @$(ECHO)  FPC.......... $(FPC)'#013#010+
  '        @$(ECHO)  FPC Version.. $(FPC_VERSION)'#013#010+
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#013#010+
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#013#010+
  '        @$(ECHO)  Source OS.... $(OS_SOURCE)'#013#010+
  '        @$(E','CHO)  Target OS.... $(OS_TARGET)'#013#010+
  '        @$(ECHO)  Full Source.. $(FULL_SOURCE)'#013#010+
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#013#010+
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#013#010+
  '        @$(ECHO)  TargetSuffix. $(TARGETSUFFIX)'#013#010+
  '        @$(E','CHO)'#013#010+
  '        @$(ECHO)  == Directory info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Basedir......... $(BASEDIR)'#013#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#013#010+
  '        @','$(ECHO)  CrossBinDir..... $(CROSSBINDIR)'#013#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#013#010+
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  GCC library..... $(GCCLIBDIR)'#013#010+
  '        @$(ECHO)  Other library.','.. $(OTHERLIBDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Tools info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  As........ $(AS)'#013#010+
  '        @$(ECHO)  Ld........ $(LD)'#013#010+
  '        @$(ECHO)  Ar........ $(AR)'#013#010+
  '        @$(ECHO)  Rc........ $(RC)'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  Mv........ $(MVPROG)'#013#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#013#010+
  '        @$(ECHO)  Rm........ $(RMPROG)'#013#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#013#010+
  '        @$(ECHO)  Echo...... $(ECHO)'#013#010+
  '        @$(ECHO)  Shell....','. $(SHELL)'#013#010+
  '        @$(ECHO)  Date...... $(DATE)'#013#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#013#010+
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#013#010+
  '        @$(ECHO)  Upx....... $(UPXPROG)'#013#010+
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#013#010+
  '        @$(ECHO)'#013#010+
  '        ','@$(ECHO)  == Object info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Target Loaders........ $(TARGET_LOADERS)'#013#010+
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#013#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#013#010+
  '        @','$(ECHO)  Target Programs....... $(TARGET_PROGRAMS)'#013#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#013#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#013#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#013#010+
  '      ','  @$(ECHO)'#013#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#013#010+
  '        @$(ECHO)  Clean Files......... $(CLEAN_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#013#010+
  '        @$(ECHO)  Install Files....... $(IN','STALL_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Install info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  DateStr.............. $(DATESTR)'#013#010+
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#013#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPRE','FIX)'#013#010+
  '        @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#013#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#013#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#013#010+
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#013#010+
  '  ','      @$(ECHO)'#013#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#013#010+
  '        @$(ECHO)  Install binary dir... $(INSTALL_BINDIR)'#013#010+
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#013#010+
  '        @$(ECHO)  Install units dir.... $(INST','ALL_UNITDIR)'#013#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#013#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#013#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#013#010+
  '        @$(ECHO)  Install data dir..... $(','INSTALL_DATADIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#013#010+
  '        @$(ECHO)  Dist zip name........ $(DIST_ZIPNAME)'#013#010+
  '        @$(ECHO)'#013#010+
  #013#010+
  '[inforules]'#013#010+
  '#########################################################','###########'+
  '#'#013#010+
  '# Info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_info'#013#010+
  #013#010+
  'fpc_info: $(INFORULES)'#013#010+
  #013#010+
  '[makefilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Reb','uild Makefile'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1 fpc_makefile_sub2'+
  ' \'#013#010+
  '        fpc_makefile_dirs'#013#010+
  #013#010+
  'fpc_makefile:'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGE','T) Makefile.fpc'#013#010+
  #013#010+
  'fpc_makefile_sub1:'#013#010+
  'ifdef TARGET_DIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
>>>>>>> graemeg/cpstrnew
=======
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPER)'+
  #013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDBASE))" >> $(ZIPWRAPP'+
  'ER)'#013#010+
  'else'#013#010+
  '        echo $(ZIPCMD_CDP','ACK) > $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'ifdef inUnix'#013#010+
  '        /bin/sh $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  'ifdef RUNBATCH'#013#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(','ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  '        $(DEL) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#013#010+
  'endif'#013#010+
  '        $(DELTREE) $(PACKDIR)'#013#010+
  #013#010+
  'fpc_zipsourceinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=sourceinstal','l ZIPSUFFIX=$(Z'+
  'IPSOURCESUFFIX)'#013#010+
  #013#010+
  'fpc_zipexampleinstall:'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipdistinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=disti','nstall'#013#010+
  #013#010+
  #013#010+
  '[cleanrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Clean rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#013#010+
  #013#010+
  'ifdef E','XEFILES'#013#010+
  'override CLEANEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEFILES'+
  '))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_PROGRAMS'#013#010+
  'override CLEANEXEFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuffix $(E'+
  'XEEXT), $(CLEAN_PROGRAMS)))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_UNITS'#013,#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEANPPUFILES'#013#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
  ') $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(C'+
  'LEANPP','UFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(S'+
  'TATICLIBEXT),$(CLEANPPUFILES)))'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#013#010+
  'endif'#013#010+
  'override CLEANPPUFILES:=$(addprefix $(U','NITTARGETDIRPREFIX),$(CLEANPP'+
  'UFILES))'#013#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPREF'+
  'IX),$(CLEANPPULINKFILES)))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_clean: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef',' CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'ifdef CLE','AN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  'ifdef LIB_NAME'#013#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#013#010+
  'endif'#013#010+
  '        -$(DEL) $(FPCMADE) Package.fpc $(PPAS) script.res link.res $(F'+
  'PCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *$(ASMEXT) *','_ppas$(BATCHEXT)'#013#010+
  #013#010+
  'fpc_cleanall: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '     ','   -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef CLEAN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) units'#013#010+
  '        -$','(DEL) *$(OEXT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL'+
  'IBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#013#010+
  'ifneq ($(PPUEXT),.ppu)'#013#010+
  '        -$(DEL) *.o *.ppu *.a'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) *$(SMARTEXT)'#013#010+
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) scri','pt.res link.res $('+
  'FPCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *_ppas$(BATCHEXT)'#013#010+
  'ifdef AOUTEXT'#013#010+
  '        -$(DEL) *$(AOUTEXT)'#013#010+
  'endif'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_distclean: cleanall'#013#010+
  #013#010+
  #013#010+
  '[baseinforules]'#013#010+
  '###','#################################################################'+
  '#'#013#010+
  '# Base info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_baseinfo'#013#010+
  #013#010+
  'override INFORULES+=fpc_baseinfo'#013#010+
  #013#010+
  'fpc_baseinfo:'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  == Package info =='#013#010+
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#013#010+
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Configuration info =='#013#010+
  '        @$(ECHO)'#013#010,
  '        @$(ECHO)  FPC.......... $(FPC)'#013#010+
  '        @$(ECHO)  FPC Version.. $(FPC_VERSION)'#013#010+
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#013#010+
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#013#010+
  '        @$(ECHO)  Source OS.... $(OS_SOURCE)'#013#010+
  '        @$(E','CHO)  Target OS.... $(OS_TARGET)'#013#010+
  '        @$(ECHO)  Full Source.. $(FULL_SOURCE)'#013#010+
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#013#010+
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#013#010+
  '        @$(ECHO)  TargetSuffix. $(TARGETSUFFIX)'#013#010+
  '        @$(E','CHO)'#013#010+
  '        @$(ECHO)  == Directory info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Basedir......... $(BASEDIR)'#013#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#013#010+
  '        @','$(ECHO)  CrossBinDir..... $(CROSSBINDIR)'#013#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#013#010+
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  GCC library..... $(GCCLIBDIR)'#013#010+
  '        @$(ECHO)  Other library.','.. $(OTHERLIBDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Tools info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  As........ $(AS)'#013#010+
  '        @$(ECHO)  Ld........ $(LD)'#013#010+
  '        @$(ECHO)  Ar........ $(AR)'#013#010+
  '        @$(ECHO)  Rc........ $(RC)'#013#010+
  '     ','   @$(ECHO)'#013#010+
  '        @$(ECHO)  Mv........ $(MVPROG)'#013#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#013#010+
  '        @$(ECHO)  Rm........ $(RMPROG)'#013#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#013#010+
  '        @$(ECHO)  Echo...... $(ECHO)'#013#010+
  '        @$(ECHO)  Shell....','. $(SHELL)'#013#010+
  '        @$(ECHO)  Date...... $(DATE)'#013#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#013#010+
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#013#010+
  '        @$(ECHO)  Upx....... $(UPXPROG)'#013#010+
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#013#010+
  '        @$(ECHO)'#013#010+
  '        ','@$(ECHO)  == Object info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Target Loaders........ $(TARGET_LOADERS)'#013#010+
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#013#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#013#010+
  '        @','$(ECHO)  Target Programs....... $(TARGET_PROGRAMS)'#013#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#013#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#013#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#013#010+
  '      ','  @$(ECHO)'#013#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#013#010+
  '        @$(ECHO)  Clean Files......... $(CLEAN_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#013#010+
  '        @$(ECHO)  Install Files....... $(IN','STALL_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Install info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  DateStr.............. $(DATESTR)'#013#010+
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#013#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPRE','FIX)'#013#010+
  '        @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#013#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#013#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#013#010+
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#013#010+
  '  ','      @$(ECHO)'#013#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#013#010+
  '        @$(ECHO)  Install binary dir... $(INSTALL_BINDIR)'#013#010+
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#013#010+
  '        @$(ECHO)  Install units dir.... $(INST','ALL_UNITDIR)'#013#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#013#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#013#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#013#010+
  '        @$(ECHO)  Install data dir..... $(','INSTALL_DATADIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#013#010+
  '        @$(ECHO)  Dist zip name........ $(DIST_ZIPNAME)'#013#010+
  '        @$(ECHO)'#013#010+
  #013#010+
  '[inforules]'#013#010+
  '#########################################################','###########'+
  '#'#013#010+
  '# Info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_info'#013#010+
  #013#010+
  'fpc_info: $(INFORULES)'#013#010+
  #013#010+
  '[makefilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Reb','uild Makefile'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1 fpc_makefile_sub2'+
  ' \'#013#010+
  '        fpc_makefile_dirs'#013#010+
  #013#010+
  'fpc_makefile:'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGE','T) Makefile.fpc'#013#010+
  #013#010+
  'fpc_makefile_sub1:'#013#010+
  'ifdef TARGET_DIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
>>>>>>> origin/cpstrnew
  'T_DIRS))'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARG',
  'ET_EXAMPLEDIRS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_makefile_sub2: $(addsuffix _makefile_dirs,$(TARGET_DIRS) $(TARGET_'+
  'EXAMPLEDIRS))'#013#010+
  #013#010+
  'fpc_makefile_dirs: fpc_makefile_sub1 fpc_makefile_sub2'#013#010+
  #013#010+
  'fpc_makefiles: fpc_makefile fpc_makefile_dirs'#013#010+
  #013#010+
  '[localmakefile]'#013#010+
  '##','##################################################################'+
  '#'#013#010+
  '# Local Makefile'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  'ifneq ($(wildcard fpcmake.loc),)'#013#010+
  'include fpcmake.loc'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[userrules]'#013#010+
  '#','###################################################################'+
  '#'#013#010+
  '# Users rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '[lclrules]'#013#010+
  '#####################################################################'#013,
  #010+
  '# LCL Rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# LCL Platform'#013#010+
  'ifndef LCL_PLATFORM'#013#010+
  'ifneq ($(findstring $(OS_TARGET),win32 win64),)'#013#010+
  'LCL_PLATFORM=win32'#013#010+
  'else'#013#010+
  'LCL_PLATFORM=gtk'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export LC','L_PLATFORM'#013#010+
  #013#010+
  '# Check if the specified LCLDIR is correct'#013#010+
  'ifdef LCLDIR'#013#010+
  'override LCLDIR:=$(subst \,/,$(LCLDIR))'#013#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Chec','k if the default LCLDIR is correct'#013#010+
  'ifdef DEFAULT_LCLDIR'#013#010+
  'override LCLDIR:=$(subst \,/,$(DEFAULT_LCLDIR))'#013#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check for development version'#013#010+
  'ifeq ($(','LCLDIR),wrong)'#013#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
  ' $(addsuffix /units/$(LCL_PLATFORM),$(BASEDIR)/lcl $(BASEDIR)))))'#013#010+
  'ifeq ($(LCLDIR),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check for release version'#013#010,
  'ifeq ($(LCLDIR),wrong)'#013#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
=======
  ';'#013#010+
  #013#010+
  '[defines]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Misc defines to be used by anyone'#013#010+
  '#############################################','#######################'+
  '#'#013#010+
  #013#010+
  '# OS categories'#013#010+
  'BSDs = freebsd netbsd openbsd darwin'#013#010+
  'UNIXs = linux $(BSDs) solaris qnx'#013#010+
  'LIMIT83fs = go32v2 os2 emx watcom'#013#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#013#010+
  #013#010+
  '#Empty target for rules that always should ','run. Needed if'#013#010+
  '#the target is non-phoney, and there is non-phony prereqisites.'#013#010+
  '#Then add FORCE as an prerequisite'#013#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#013#010+
  'FORCE:'#013#010+
  '.PHONY: FORCE'#013#010+
  #013#010+
  '[osdetect]'#013#010+
  '###################','#################################################'+
  '#'#013#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#013+
  #010+
  '# define inUnix when running under Unix like environment'#013#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,Cygwin)',#013#010+
  '# define inWinNT when running under WinNT'#013#010+
  '# define inOS2 when running under OS/2'#013#010+
  '# define inCygwin when running under Cygwin32'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# We need only / in the path also r','emove the current dir,'#013#010+
  '# also remove trailing /'#039's'#013#010+
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#013#010+
  #013#010+
  '# Detect unix'#013#010+
  '# Darwin is handled specially'#013#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#013#010+
  'inUnix=1 #darwin'#013#010+
  'SEARCHPATH:=$(filter-out .,','$(subst :, ,$(PATH)))'#013#010+
  'else'#013#010+
  '# Determine if we'#039've a unix searchpath by looking for a ;'#013#010+
  '# that normally doesn'#039't exists in the unix PATH var.'#013#010+
  'ifeq ($(findstring ;,$(PATH)),)'#013#010+
  'inUnix=1'#013#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#013#010+
  'else'#013,#010+
  'SEARCHPATH:=$(subst ;, ,$(PATH))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add path were make is located'#013#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#013#010+
  #013#010+
  '# Search for PWD'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013,#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013#010+
  '$(error You need the GNU utils package to use this Makefile)'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT='#013#010+
  'endif'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT=.exe'#013#010+
  'end','if'#013#010+
  #013#010+
  '# Detect NT - NT sets OS to Windows_NT'#013#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#013#010+
  'ifndef inUnix'#013#010+
  'ifeq ($(OS),Windows_NT)'#013#010+
  'inWinNT=1'#013#010+
  'else'#013#010+
  'ifdef OS2_SHELL'#013#010+
  'inOS2=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifneq ($(findstring cygdrive,$(PATH)),)'#013#010+
  'inCygW','in=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# The extension of batch files / scripts'#013#010+
  'ifdef inUnix'#013#010+
  'SRCBATCHEXT=.sh'#013#010+
  'else'#013#010+
  'ifdef inOS2'#013#010+
  'SRCBATCHEXT=.cmd'#013#010+
  'else'#013#010+
  'SRCBATCHEXT=.bat'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef COMSPEC'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(OSNeedsComspecToRu','nBatch)),)'#013#010+
  'RUNBATCH=$(COMSPEC) /C'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#013#010+
  '# at the end of a line'#013#010+
  'ifdef inUnix'#013#010+
  'PATHSEP=/'#013#010+
  'else'#013#010+
  'PATHSEP:=$(subst /,\,/)'#013#010+
  '# cygwin bash or sh can not hand','le backslashs'#013#010+
  'ifdef inCygWin'#013#010+
  'PATHSEP=/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Base dir'#013#010+
  'ifdef PWD'#013#010+
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#013#010+
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#013#010+
  'ifdef inCygWin'#013#010+
  'ifneq ($(findstring /cygdrive/,$(BASEDIR)),)'#013#010+
  'BAS','ENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#013#010+
  'BASEDRIVE:=$(firstword $(subst /, ,$(BASENODIR)))'#013#010+
  'BASEDIR:=$(subst /cygdrive/$(BASEDRIVE)/,$(BASEDRIVE):/,$(BASEDIR))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'BASEDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  '# Echo is an internal command under',' OS/2 (and others), so it'#039's a'+
  'lways found'#013#010+
  'ifdef inOS2'#013#010+
  'ifndef ECHO'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH))'+
  '))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /echo$(SRCEXEEXT),$(SEARCHPATH)))'+
  ')'#013#010+
  'ifeq ($(','ECHO),)'#013#010+
  'ECHO=echo'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export ECHO'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdetect]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPC Binary and Version ','Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Compatibility with old makefiles'#013#010+
  'ifndef FPC'#013#010+
  'ifdef PP'#013#010+
  'FPC=$(PP)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try to detect the ppcXXX file to use by using "fpc -PB" option'#013#010+
  '# to ','query for the default ppcXXX the fpc executable tries. When'#013+
  #010+
  '# fpc is not found use ppc386 by default. Also when fpc -PB gives'#013#010+
  '# an error ppc386 will be used.'#013#010+
  'ifndef FPC'#013#010+
  '# check if fpc exists'#013#010+
  'FPCPROG:=$(strip $(wildcard $(addsuffix /fpc','$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifneq ($(FPCPROG),)'#013#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#013#010+
  'ifneq ($(CPU_TARGET),)'#013#010+
  'FPC:=$(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#013#010+
  'else'#013#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#013#010+
  'endif'#013#010+
  '# Older fpc executables didn'#039't support it',' and return'#013#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#013#010+
  'ifneq ($(findstring Error,$(FPC)),)'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#013#010+
  'else'#013#010+
  '# if the cross compiler i','s not found, fall back to fpc'#013#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#013#010+
  'FPC:=$(firstword $(FPCPROG))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# fpc binary not found, fallback to ppc386'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(','SEARCHPATH)))))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Get a clean executable name'#013#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#013#010+
  'override FPC:=$(subst \,/,$(FPC))$(SRCEXEEXT)'#013#010+
  #013#010+
  '# Try to find the binary direct first, otherwise in the path,'#013#010+
  '# if not found give',' an error'#013#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  '$(error Compiler $(FPC) not found)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.1 and up we can use a ','single compiler call to retrieve'#013#010+
  '# all needed information'#013#010+
  'ifndef FPC_COMPILERINFO'#013#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# FPC version'#013#010+
  'ifndef FPC_VERSION'#013#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  #013#010+
  'export FPC ','FPC_VERSION FPC_COMPILERINFO'#013#010+
  '# CHECKDEPEND should not be exported'#013#010+
  '# This should limit multiple checks'#013#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# FPC Target Detection'#013#010,
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Fall back to default values if needed'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'ifdef CPU_TARGET_DEFAULT'#013#010+
  'CPU_TARGET=$(CPU_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'ifdef OS_TARGET','_DEFAULT'#013#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.0.x we need to use extra calls to retrieve all info'#013#010+
  'ifneq ($(words $(FPC_COMPILERINFO)),5)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSP)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTP)'#013#010+
  'F','PC_COMPILERINFO+=$(shell $(FPC) -iSO)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Retrieve Target/Source CPU and Target/Source OS'#013#010+
  'ifndef CPU_SOURCE'#013#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'CPU_TARGET:=$','(word 3,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_SOURCE'#013#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013#010+
  'FULL_SOURCE=$(CPU_SOURCE)-$(O','S_SOURCE)'#013#010+
  #013#010+
  '# Full name of the target, including CPU and OS. For OSs limited'#013#010+
  '# to 8.3 we only use the target OS'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'TARGETSUFFIX=$(OS_TARGET)'#013#010+
  'SOURCESUFFIX=$(OS_SOURCE)'#013#010+
  'else'#013#010+
  'TARGETSUFFIX=$(','FULL_TARGET)'#013#010+
  'SOURCESUFFIX=$(FULL_SOURCE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compile flag'#013#010+
  'ifneq ($(FULL_TARGET),$(FULL_SOURCE))'#013#010+
  'CROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the Makefile supports this target, but not'#013#010+
  '# when the make target is to rebuild the makefi','le'#013#010+
  'ifeq ($(findstring makefile,$(MAKECMDGOALS)),)'#013#010+
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),)'#013#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Detect BSD, since BSD us','es a slightly different directory hierarch'+
  'y.'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#013#010+
  'BSDhier=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect Linux, will also use its own directory hierarchy.'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'linuxHier=1'#013#010+
  'endif'#013#010+
  #013#010+
  'export OS_TARGET OS_','SOURCE CPU_TARGET CPU_SOURCE FULL_TARGET FULL_SO'+
  'URCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#013#010+
  #013#010+
  #013#010+
  '[fpcdircheckenv]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPCDIR Setting'#013#010+
  '####################################','################################'+
  '#'#013#010+
  #013#010+
  '# Test FPCDIR to look if the RTL dir exists'#013#010+
  'ifdef FPCDIR'#013#010+
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPC','DIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default FPCDIR from Makefile.fpc'#013#010+
  'ifdef DEFAULT_FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'endi','f'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdirdetect]'#013#010+
  '# Detect FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'ifdef inUnix'#013#010+
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#013#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#013#010+
  'override FPCDIR=/usr/lib/fpc/$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FP','CDIR:=$(subst /$(FPC),,$(firstword $(strip $(wildcard $(a'+
  'ddsuffix /$(FPC),$(SEARCHPATH))))))'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix ','$(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(BASEDIR)'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=c:/pp'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross binaries dir'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(FP','CDIR)/bin/$(TARGETSUFFIX))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default binutils prefix for cross compile when the'#013#010+
  '# crossbindir is not set (except for Darwin)'#013#010+
  'ifeq ($(OS_TARGET),darwin)'#013#010+
  'ifeq ($(OS_SOURCE),darwin)'#013#010+
  'DARWIN2DARWIN=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef BINUTIL','SPREFIX'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifndef DARWIN2DARWIN'#013#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try first the full target name, otherwise try only'#013#010+
  '# the OS for backwards compatibility'#013#010+
  'UNI','TSDIR:=$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#013#010+
  'ifeq ($(UNITSDIR),)'#013#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(OS_TARGET))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Packages dir'#013#010+
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages $(FPCDIR)/package'+
  's/base $(FPCDIR)/pac','kages/extra)'#013#010+
  #013#010+
  #013#010+
  '[shelltools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Shell tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Echo that can redir (must be able run in the de','fault OS shell)'#013+
  #010+
  'ifndef ECHOREDIR'#013#010+
  'ifndef inUnix'#013#010+
  'ECHOREDIR=echo'#013#010+
  'else'#013#010+
  'ECHOREDIR=$(ECHO)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To copy pograms'#013#010+
  'ifndef COPY'#013#010+
  'COPY:=$(CPPROG) -fp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef COPYTREE'#013#010+
  'COPYTREE:=$(CPPROG) -Rfp'#013#010+
  'endif'#013#010,
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef MKDIRTREE'#013#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#013#010+
  'endif'#013#010+
  #013#010+
  '# To move pograms'#013#010+
  'ifndef MOVE'#013#010+
  'MOVE:=$(MVPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check delete program'#013#010+
  'ifndef DEL'#013#010+
  'DEL:=$(RMPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check deltree program'#013#010+
  'ifndef DELTR','EE'#013#010+
  'DELTREE:=$(RMPROG) -rf'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install files'#013#010+
  'ifndef INSTALL'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALL:=$(GINSTALL) -c -m 644'#013#010+
  'else'#013#010+
  'INSTALL:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install programs'#013#010+
  'ifndef INSTALLEXE'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALLEXE:=$(GINSTALL) -','c -m 755'#013#010+
  'else'#013#010+
  'INSTALLEXE:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To make a directory.'#013#010+
  'ifndef MKDIR'#013#010+
  'MKDIR:=$(GINSTALL) -m 755 -d'#013#010+
  'endif'#013#010+
  #013#010+
  'export ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE MKD'+
  'IR'#013#010+
  #013#010+
  #013#010+
  '[defaultdirs]'#013#010+
  '#################','###################################################'+
  '#'#013#010+
  '# Default Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Units dir'#013#010+
  'ifdef REQUIRE_UNITSDIR'#013#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Un','its dir'#013#010+
  'ifdef REQUIRE_PACKAGESDIR'#013#010+
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#013#010+
  '# When zipping use the target os default, when normal install then'#013+
  #010+
  '# use the source os as default'#013,#010+
  'ifdef ZIPINSTALL'#013#010+
  '# Zipinstall'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# Normal install'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# When install prefix is not set try to use ','prefix'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef PREFIX'#013#010+
  'INSTALL_PREFIX=$(PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the prefix directory where to install everything'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_PREFIX=/usr/local'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013,#010+
  'INSTALL_BASEDIR:=/pp'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export INSTALL_PREFIX'#013#010+
  #013#010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#013#010+
  '# used recursively for all subdirs'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'export INSTALL_','FPCSUBDIR'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to place the resulting zip files'#013#010+
  'ifndef DIST_DESTDIR'#013#010+
  'DIST_DESTDIR:=$(BASEDIR)'#013#010+
  'endif'#013#010+
  'export DIST_DESTDIR'#013#010+
  #013#010+
  '# EXE/PPU Target directories'#013#010+
  'ifndef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef PACKAGEDIR_MAIN'#013#010+
  'COMPILER_UNITTARG','ETDIR=$(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#013#010+
  'else'#013#010+
  'COMPILER_UNITTARGETDIR=units/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef COMPILER_TARGETDIR'#013#010+
  'COMPILER_TARGETDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '##################################################################','##'+
  '#'#013#010+
  '# Install Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# set the base directory where to install everything'#013#010+
  'ifndef INSTALL_BASEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_BASEDIR:=$(','INSTALL_PREFIX)/lib/fpc/$(FPC_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the binaries'#013#010+
  'ifndef INSTALL_BINDIR'#013,#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/bin'#013#010+
  '# for FPC packages install the binaries under their target subdir'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifdef CROSSINSTALL'#013#010+
  'INSTA','LL_BINDIR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the un','its.'#013#010+
  'ifndef INSTALL_UNITDIR'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef PACKAGE_NAME'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install shared lib','raries'#013#010+
  'ifndef INSTALL_LIBDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#013#010+
  'else'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the source files will be stored'#013#010+
  'ifndef INSTALL_SOURCEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier',#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'SRCPREFIXDIR=src'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INST','ALL_FPCSUBDIR)/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else',#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$(PA'+
  'CKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(IN','STALL_BASEDIR)/source'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the doc files will be stored'#013#010+
  'ifndef INSTALL_DOCDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'DOCPREFIXDIR=doc'#013#010+
  'endif'#013#010,
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/fpc-$(FPC_VERSION)/$'+
  '(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACKAGE_NAME)-$(PA'+
  'CKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FP','CPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install the examples, under linux we use the doc dir'#013#010+
  '# because the copytree command will crea','te a subdir itself'#013#010+
  'ifndef INSTALL_EXAMPLEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(','INSTALL_DOCDIR)/examples'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VER','SION)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_','FPCPACKAGE'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the some extra (data)files will be stored'#013#010+
  'ifndef INSTALL_DATADIR'#013#010+
  'INSTALL_DA','TADIR=$(INSTALL_BASEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef INSTALL_SHAREDDIR'#013#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#013#010+
  'endif'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# Cross compile dirs'#013#010+
  '####################################','################################'+
  '#'#013#010+
  #013#010+
  'ifdef CROSSCOMPILE'#013#010+
  '# Directory where the cross compile tools are stored.'#013#010+
  '# First check if they are available in FPCDIR. If no targets/ subdir'#013+
  #010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#013#010+
  'if','ndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/cross/$(TARGETSUFFIX)/bin/$'+
  '(FULL_SOURCE))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'CROSSBINDIR='#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[di','rlibc]'#013#010+
  '# On linux, try to find where libgcc.a is.'#013#010+
  'ifeq ($(OS_SOURCE),linux)'#013#010+
  #013#010+
  '# Amd64 to i386?'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'ifneq ($(findstring x86_64,$(shell uname -a)),)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell d','irname `gcc -m32 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try cross gcc'#013#010+
  'ifndef GCCLI','BDIR'#013#010+
  'CROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSPREFIX)gcc$(SRCEXEE'+
  'XT),$(SEARCHPATH))))'#013#010+
  'ifneq ($(CROSSGCC),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Other libraries from ld.so.conf'#013#010+
  'if','ndef OTHERLIBDIR'#013#010+
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.conf | awk '#039'{ ORS=" '+
  '"; print $1 }'#039')'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef inUnix'#013#010+
  'ifeq ($(OS_SOURCE),netbsd)'#013#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#013#010+
  'endif'#013#010+
  'export GCCLIBDIR OTHERLIB'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[extensio','ns]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Default extensions'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Default needed extensions (Go32v2,Linux)'#013#010+
  'BATCHEXT=.bat'#013#010+
  'LOADEREXT=','.as'#013#010+
  'EXEEXT=.exe'#013#010+
  'PPLEXT=.ppl'#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHAREDLIBPREFIX=libfp'#013#010+
  'STATICLIBPREFIX=libp'#013#010+
  'IMPORTLIBPREFIX=libimp'#013#010+
  'RSTEXT=.rst'#013#010+
  '#DEBUGSYMEXT #for debugger symbol files, defi','ne only for targets whi'+
  'ch has this'#013#010+
  #013#010+
  '# 1.0.x has target specific extensions for ppu files and objects'#013#010+
  'ifeq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  '# short version for 1.1 and above - no target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TAR','GET),go32v1)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'OEXT=.obj'#013#010+
  'ASMEXT=.asm'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SH','ORTSUFFIX=wat'#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010,
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'SHARED','LIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'endif'#013#010+
  #013#010+
  '# EMX'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STAT','ICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=emx'#013#010+
  'ECHO=echo'#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# MorphOS'#013#010+
  'ifeq ($(OS_TARGET),morphos)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHOR','TSUFFIX=mos'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT=',#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware clib'#013#010+
  'ifeq ($(OS_TARGET),netware)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TAR','GET),netwlibc)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'endif'#013#010+
  #013#010+
  '# Darwin'#013#010+
  'ifeq ($(OS_TARGET),darwin)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASS','HAREDLIB=1'#013#010+
  'SHORTSUFFIX=dwn'#013#010+
  'endif'#013#010+
  #013#010+
  '# gba'#013#010+
  'ifeq ($(OS_TARGET),gba)'#013#010+
  'EXEEXT=.gba'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=gba'#013#010+
  'endif'#013#010+
  #013#010+
  '# Symbian OS'#013#010+
  'ifeq ($(OS_TARGET),symbian)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=symbian'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  '# long version ','for 1.0.x - target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TARGET),go32v1)'#013#010+
  'PPUEXT=.pp1'#013#010+
  'OEXT=.o1'#013#010+
  'ASMEXT=.s1'#013#010+
  'SMARTEXT=.sl1'#013#010+
  'STATICLIBEXT=.a1'#013#010+
  'SHAREDLIBEXT=.so1'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),g','o32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Fre','eBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd',')'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'PPUEXT=.ppw'#013#010+
  'OEXT=.ow'#013#010+
  'ASMEXT=.sw'#013#010+
  'SMARTEXT=.slw'#013#010+
  'STATICLIBEXT=.aw'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARG','ET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'PPUEXT=.ppo'#013#010+
  'ASMEXT=.so2'#013#010+
  'OEXT=.oo2'#013#010+
  'AOUTEXT=.out'#013#010+
  'SMARTEXT=.sl2'#013#010+
  'STATICLIBPREFIX='#013#010+
  'STATICLIBEXT=.ao2'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'PPUEXT=.ppu',#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013,#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTE','XT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware'#013#010+
  'ifeq ($(OS_TARGET),netwa','re)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013,#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFI','X=mac'#013#010+
  'endif'#013#010+
  #013#010+
  '#end of target specific settings'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 8.3 limited OS'#039's the short suffixes'#013#010+
  '# Otherwise use the full source/target names'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)'#013#010+
  'ZIPSUFFIX=','$(SHORTSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'ZIPSOURCESUFFIX=src'#013#010+
  'ZIPEXAMPLESUFFIX=exm'#013#010+
  'else'#013#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#013#010+
  'ZIPSOURCESUFFIX=.source'#013#010+
  'ZIPEXAMPLESUFFIX=.examples'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#013#010+
  'ZIPCROSSPREFIX=$(TARGETS','UFFIX)-'#013#010+
  'else'#013#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[defaulttools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Default Tools'#013#010+
  '############################################################','########'+
  '#'#013#010+
  #013#010+
  '# Names of the binutils tools'#013#010+
  'ASNAME=$(BINUTILSPREFIX)as'#013#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#013#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#013#010+
  'RCNAME=$(BINUTILSPREFIX)rc'#013#010+
  'ifneq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'ifeq ($(CROSS','BINDIR),)'#013#010+
  'ASNAME=asw'#013#010+
  'LDNAME=ldw'#013#010+
  'ARNAME=arw'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# assembler, redefine it if cross compiling'#013#010+
  'ifndef ASPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ASPROG=$(CROSSBINDIR)/$(ASNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ASPROG=$(ASNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# link','er, but probably not used'#013#010+
  'ifndef LDPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'LDPROG=$(CROSSBINDIR)/$(LDNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'LDPROG=$(LDNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Resource compiler'#013#010+
  'ifndef RCPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'RCPROG=$(CROSSBINDIR)/$(RCNAME)$(SRCEXE','EXT)'#013#010+
  'else'#013#010+
  'RCPROG=$(RCNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Archiver'#013#010+
  'ifndef ARPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ARPROG=$(CROSSBINDIR)/$(ARNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ARPROG=$(ARNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Override defaults'#013#010+
  'AS=$(ASPROG)'#013#010+
  'LD=$(LDPROG)'#013#010+
  'RC=$(RCPROG)'#013#010,
  'AR=$(ARPROG)'#013#010+
  #013#010+
  '# ppas.bat / ppas.sh'#013#010+
  'PPAS=ppas$(SRCBATCHEXT)'#013#010+
  #013#010+
  '# ldconfig to rebuild .so cache'#013#010+
  'ifdef inUnix'#013#010+
  'LDCONFIG=ldconfig'#013#010+
  'else'#013#010+
  'LDCONFIG='#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef DATE'#013#010+
  'DATESTR:=$(shell $(DATE) +%Y%m%d)'#013#010+
  'else'#013#010+
  'DATESTR='#013#010+
  'endif'#013#010+
  #013#010+
  '# Look if U','PX is found for go32v2 and win32. We can'#039't use $UPX b'+
  'ecuase'#013#010+
  '# upx uses that one itself (PFV)'#013#010+
  'ifndef UPXPROG'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifdef UPXPROG'#013#010+
  'UPXPROG:=$(strip $(wi','ldcard $(addsuffix /upx$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifeq ($(UPXPROG),)'#013#010+
  'UPXPROG='#013#010+
  'else'#013#010+
  'UPXPROG:=$(firstword $(UPXPROG))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'UPXPROG='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export UPXPROG'#013#010+
  #013#010+
  '# Zip options'#013#010+
  'ZIPOPT=-9'#013#010+
  'ZIPEXT=.zip'#013#010+
  #013#010+
  '# Tar options'#013#010+
  'ifeq ','($(USETAR),bz2)'#013#010+
  'TAROPT=vj'#013#010+
  'TAREXT=.tar.bz2'#013#010+
  'else'#013#010+
  'TAROPT=vz'#013#010+
  'TAREXT=.tar.gz'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_begin]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Compiler Command Line'#013#010+
  '######################################','##############################'+
  '#'#013#010+
  #013#010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#013+
  #010+
  '# compiler this needs to be turned off'#013#010+
  'ifndef NOCPUDEF'#013#010+
  'override FPCOPTDEF=$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Load commandline OPT and add tar','get and unit dir to be sure'#013#010+
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#013#010+
  'override FPCOPT+=-T$(OS_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#013#010+
  'override FPCOPT+=-P$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifeq ($(OS_SOURCE),openbsd)'#013#010+
  'override FPCOPT+=-FD$(NEW','_BINUTILS_PATH)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef CROSSBOOTSTRAP'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-XP$(BINUTILSPREFIX)'#013#010+
  'endif'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-Xr$(RLINKPATH)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# User dirs should be first, so they ar','e looked at first'#013#010+
  'ifdef UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef LIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#013#010+
  'endif'#013#010+
  'ifdef OBJDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fo,$(OBJDIR))'#013#010+
  'endif'#013#010+
  'ifdef INCDIR'#013#010+
  'overrid','e FPCOPT+=$(addprefix -Fi,$(INCDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking'#013#010+
  'ifdef LINKSMART'#013#010+
  'override FPCOPT+=-XX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking creation'#013#010+
  'ifdef CREATESMART'#013#010+
  'override FPCOPT+=-CX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Debug'#013#010+
  'ifdef DEBUG'#013#010+
  'override FPCOPT+=-gl'#013#010+
  'override F','PCOPTDEF+=DEBUG'#013#010+
  'endif'#013#010+
  #013#010+
  '# Release mode'#013#010+
  '# (strip, optimize and don'#039't load fpc.cfg)'#013#010+
  '# fpc 2.1 has -O2 for all targets'#013#010+
  'ifdef RELEASE'#013#010+
  'ifneq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'FPCCPUOPT:=-OG2p3'#013#010+
  'endif'#013#010+
  'ifeq ($','(CPU_TARGET),powerpc)'#013#010+
  'FPCCPUOPT:=-O1r'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'FPCCPUOPT:=-O2'#013#010+
  'endif'#013#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#013#010+
  'override FPCOPTDEF+=RELEASE'#013#010+
  'endif'#013#010+
  #013#010+
  '# Strip'#013#010+
  'ifdef STRIP'#013#010+
  'override FPCOPT+=-Xs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Optimizer'#013#010+
  'ifdef OPTIMIZE'#013#010+
  'ov','erride FPCOPT+=-O2'#013#010+
  'endif'#013#010+
  #013#010+
  '# Verbose settings (warning,note,info)'#013#010+
  'ifdef VERBOSE'#013#010+
  'override FPCOPT+=-vwni'#013#010+
  'endif'#013#010+
  #013#010+
  '# Needed compiler options'#013#010+
  'ifdef COMPILER_OPTIONS'#013#010+
  'override FPCOPT+=$(COMPILER_OPTIONS)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITDIR'#013#010+
  'ove','rride FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_LIBRARYDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_OBJECTDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#013#010+
  'endif'#013,#010+
  'ifdef COMPILER_INCLUDEDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compiler utils'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'override FPCOPT+=-FD$(CROSSBINDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Target dirs and the prefix to use for clean/install'#013#010+
  'if','def COMPILER_TARGETDIR'#013#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#013#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#013#010+
  'override TARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR)/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Unit target dir (default is units/<cpu>-<','os>/'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#013#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#013#010+
  'override UNITTARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override UNITTARGETDIRPREFIX=$(COMPILER_UNITTARGETDIR)/'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef COMPILER','_TARGETDIR'#013#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#013#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CREATESHARED'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  #013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'override FPCOPT+=-Aas'#013#010+
  'endif'#013#010+
  #013#010+
  'endif',#013#010+
  #013#010+
  '# create always pic'#039'ed code on x86_64'#013#010+
  'ifeq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'ifeq ($(CPU_TARGET),x86_64)'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef LINKSHARED'#013#010+
  '#!!! deactived for testing override',' FPCOPT+=-XD'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_libc]'#013#010+
  '# Add GCC lib path if asked'#013#010+
  'ifdef GCCLIBDIR'#013#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#013#010+
  'endif'#013#010+
  'ifdef OTHERLIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(OTHERLIBDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_end]'#013#010+
  '# Add commandline',' options last so they can override'#013#010+
  'ifdef OPT'#013#010+
  'override FPCOPT+=$(OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add defines from FPCOPTDEF to FPCOPT'#013#010+
  'ifdef FPCOPTDEF'#013#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOPTDEF))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Was a config file specified ?'#013#010+
  'ifdef CFGFILE',#013#010+
  'override FPCOPT+=@$(CFGFILE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use the environment to pass commandline options?'#013#010+
  'ifdef USEENV'#013#010+
  'override FPCEXTCMD:=$(FPCOPT)'#013#010+
  'override FPCOPT:=!FPCEXTCMD'#013#010+
  'export FPCEXTCMD'#013#010+
  'endif'#013#010+
  #013#010+
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARGE','T)'#013#010+
  'override AFULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifneq ($(AFULL_TARGET),$(AFULL_SOURCE))'#013#010+
  'override ACROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef ACROSSCOMPILE'#013#010+
  'override FPCOPT+=$(CROSSOPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Compiler commandline'#013#010+
  'override COMPILER:=$(FPC',') $(FPCOPT)'#013#010+
  #013#010+
  '# also call ppas if with command option -s'#013#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#013#010+
  'ifeq (,$(findstring -s ,$(COMPILER)))'#013#010+
  'EXECPPAS='#013#010+
  'else'#013#010+
  'ifeq ($(FULL_SOURCE),$(FULL_TARGET))'#013#010+
  'ifdef RUNBATCH'#013#010+
  'EXECPPAS:=@$(R','UNBATCH) $(PPAS)'#013#010+
  'else'#013#010+
  'EXECPPAS:=@$(PPAS)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[loaderrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Loaders'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.','PHONY: fpc_loaders'#013#010+
  #013#010+
  'ifneq ($(TARGET_LOADERS),)'#013#010+
  'override ALLTARGET+=fpc_loaders'#013#010+
  'override CLEANTARGET+=fpc_loaders_clean'#013#010+
  'override INSTALLTARGET+=fpc_loaders_install'#013#010+
  #013#010+
  'override LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#013#010+
  'endif'#013,#010+
  #013#010+
  '%$(OEXT): %$(LOADEREXT)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#013#010+
  'else'#013#010+
  '        $(AS) -o $*$(OEXT) $<'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders: $(COMPILER_UNITTARGETDIR) $(LOADEROFILES)'#013#010+
  #013#010+
  'fpc_loaders_clean:'#013#010+
  'i','fdef COMPILER_UNITTARGETDIR'#013#010+
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#013#010+
  'else'#013#010+
  '        -$(DEL) $(LOADEROFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders_install:'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  ' ','       $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROF'+
  'ILES)) $(INSTALL_UNITDIR)'#013#010+
  'else'#013#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[unitrules]'#013#010+
  '#############################################################','#######'+
  '#'#013#010+
  '# Units'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_units'#013#010+
  #013#010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#013#010+
  'override ALLTARGET+=fpc_units'#013#010+
  #013#010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$','(TARGET_UNITS))'#013#010+
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#013#010+
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'override CLEANPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_','units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#013#010+
  #013#010+
  #013#010+
  '[exerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Exes'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_exes'#013#010+
  #013#010+
  '# ','Programs are not needed for a cross installation'#013#010+
  'ifndef CROSSINSTALL'#013#010+
  'ifneq ($(TARGET_PROGRAMS),)'#013#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#013#010+
  'override EXEOFILES:=$(addsuffix $(OEXT),$(TARGET_PROGRAMS)) $(addprefi'+
  'x $(STATI','CLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
  'PROGRAMS)))'#013#010+
  #013#010+
  'override ALLTARGET+=fpc_exes'#013#010+
  'override INSTALLEXEFILES+=$(EXEFILES)'#013#010+
  'override CLEANEXEFILES+=$(E','XEFILES) $(EXEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'f','pc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES'+
  ')'#013#010+
  #013#010+
  #013#010+
  '[rstrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Resource strings'#013#010+
  '###############################################################','#####'+
  '#'#013#010+
  #013#010+
  'ifdef TARGET_RSTS'#013#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_RSTS))'#013#010+
  #013#010+
  'override CLEANRSTFILES+=$(RSTFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[examplerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Examples'#013#010+
  '#####','###############################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_examples'#013#010+
  #013#010+
  'ifneq ($(TARGET_EXAMPLES),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $(addsuffix .pp,$(TARGET_EXAMP'+
  'LES)) $(addsuffix .pas,$(TARGET_EX','AMPLES)) $(addsuffix .lpr,$(TARGET'+
  '_EXAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#013#010+
  'override EXAMPLEFILES:=$(addsuffix $(EXEEXT),$(TARGET_EXAMPLES))'#013#010+
  'override EXAMPLEOFILES:=$(addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(addp'+
  'refix $(STATICLIBPR','EFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPL'+
  'ES))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TAR'+
  'GET_EXAMPLES)))'#013#010+
  #013#010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES) $(EXAMPLEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFI','LES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifneq ($(TARGET_EXAMPLEDIRS),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_examples: all $(E','XAMPLEFILES) $(addsuffix _all,$(TARGET_EXAMPLED'+
  'IRS))'#013#010+
  #013#010+
  #013#010+
  '[compilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# General compile rules'#013#010+
  '#####################################################################'#013,
  #010+
  #013#010+
  '.PHONY: fpc_all fpc_smart fpc_debug fpc_release fpc_shared'#013#010+
  #013#010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#013#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMADE)'#013#010+
  #013#010+
  'fpc_all: $(FPCMADE)'#013#010+
  #013#010+
  'fpc_smart:'#013#010+
  '        $(MAKE) all LINKSMART=1 CREATESMART=1'#013#010+
  #013,#010+
  'fpc_debug:'#013#010+
  '        $(MAKE) all DEBUG=1'#013#010+
  #013#010+
  'fpc_release:'#013#010+
  '        $(MAKE) all RELEASE=1'#013#010+
  #013#010+
  '# General compile rules, available for both possible .pp and .pas exte'+
  'nsions'#013#010+
  #013#010+
  '.SUFFIXES: $(EXEEXT) $(PPUEXT) $(OEXT) .pas .lpr .dpr .pp .rc .res'#013+
  #010+
  #013,#010+
  '$(COMPILER_UNITTARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '$(COMPILER_TARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pas'#013#010+
  '       ',' $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.lpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(','EXEEXT): %.dpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%.res: %.rc'#013#010+
  '        windres -i $< -o $@'#013#010+
  #013#010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#013#010+
  'vpath %.pp $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.pas $(COMPILER_SOURC','EDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.inc $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  'vpath %$(PPUEXT)',' $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '[sharedrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Library'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_shared'#013#010+
  #013#010+
  'override INSTALLTA','RGET+=fpc_shared_install'#013#010+
  #013#010+
  'ifndef SHARED_LIBVERSION'#013#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_LIBNAME'#013#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_FULLNAME'#013#010+
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED_LIBNAME)-$(S','HARED_LIBVERS'+
  'ION)$(SHAREDLIBEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default sharedlib units are all unit objects'#013#010+
  'ifndef SHARED_LIBUNITS'#013#010+
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#013#010+
  'override SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),$(SHARE','D'+
  '_LIBUNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared:'#013#010+
  'ifdef HASSHAREDLIB'#013#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  '        $(PPUMOVE) -q $(SHARED_LIBUNITS) -i$(COMPILER_UNITTARGETDIR) -'+
  'o$(SHARED_FULLNAME) -d$(','COMPILER_UNITTARGETDIR)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '        @$(ECHO) Shared Libraries not supported'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared_install:'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  'ifneq ($(SHARED_LIBUNITS),)'#013#010+
  'ifneq ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME)),)'#013#010,
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[installrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Install rules'#013#010+
  '###########################','#########################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#013#010+
  #013#010+
  'ifdef INSTALL_UNITS'#013#010+
  'override INSTALLPPUFILES+=$(addsuffix $(PPUEXT),$(INSTALL_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALL_BUILDUNIT'#013#010+
  'override INS','TALLPPUFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT)'+
  ',$(INSTALLPPUFILES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPPUFI'+
  'LES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STA','TICLIBEX'+
  'T),$(INSTALLPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#013#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#013#010+
  'override INSTALLPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(notdir '+
  '$(INSTALLPPUFILES)','))'#013#010+
  'override INSTALLPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPR'+
  'EFIX),$(notdir $(INSTALLPPULINKFILES))))'#013#010+
  'endif'#013#010+
  '# Implicitly install Package.fpc'#013#010+
  'override INSTALL_CREATEPACKAGEFPC=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  'ifneq ($(TARGE','TDIRPREFIX),)'#013#010+
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_install: all $(INSTALLTARGET)'#013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  '        $(MKDIR) $(INSTALL_BINDIR)'#013#010+
  '# Compress the exes if upx ','is defined'#013#010+
  'ifdef UPXPROG'#013#010+
  '        -$(UPXPROG) $(INSTALLEXEFILES)'#013#010+
  'endif'#013#010+
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#013#010+
  'ifdef FPCMAKE'#013#010+
  '# If the fpcpackage variable is set then create and ','install Package.'+
  'fpc,'#013#010+
  '# a safety check is done if Makefile.fpc is available'#013#010+
  'ifdef PACKAGE_VERSION'#013#010+
  'ifneq ($(wildcard Makefile.fpc),)'#013#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS_TARGET) Makefile.fpc'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '   ','     $(INSTALL) Package.fpc $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(INSTALL_UNITDIR)'#013#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#013#010+
  '        $(INS','TALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'ifneq ($(wildcard $(LIB_FULLNAME)),)'#013#010+
  '        $(MKDIR) $(INSTALL_LIBDIR)'#013#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LIBDIR)'#013#010+
  'ifdef inUnix'#013#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_L','IBDIR)/$(LIB_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FILES'#013#010+
  '        $(MKDIR) $(INSTALL_DATADIR)'#013#010+
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_sourceinstall: distclean'#013#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#013#010+
  '       ',' $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#013#010+
  #013#010+
  'fpc_exampleinstall: $(addsuffix _distclean,$(TARGET_EXAMPLEDIRS))'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef EXAMPLESOURCEFILES'#013#010+
  '        $(COPY) $(EXAMPLESOURCE','FILES) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_EX'+
  'AMPLEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '[distinstallrules]'#013#010+
  '###################################################################','#'+
  '#'#013#010+
  '# Dist Install'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_distinstall'#013#010+
  #013#010+
  'fpc_distinstall: install exampleinstall'#013#010+
  #013#010+
  #013#010+
  '[zipinstallrules]'#013#010+
  '########################################################','############'+
  '#'#013#010+
  '# Zip'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#013#010+
  #013#010+
  '# Temporary path to pack a file, can only use a single deep'#013#010+
  '# subdir, becau','se the deltree can'#039't see the whole tree to remove'+
  #013#010+
  'ifndef PACKDIR'#013#010+
  'ifndef inUnix'#013#010+
  'PACKDIR=$(BASEDIR)/../fpc-pack'#013#010+
  'else'#013#010+
  'PACKDIR=/tmp/fpc-pack'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Maybe create default zipname from packagename'#013#010+
  'ifndef ZIPNAME'#013#010+
  'ifdef DIST_ZIPNA','ME'#013#010+
  'ZIPNAME=$(DIST_ZIPNAME)'#013#010+
  'else'#013#010+
  'ZIPNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef FULLZIPNAME'#013#010+
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  '# ZipTarget'#013#010+
  'ifndef ZIPTARGET'#013#010+
  'ifdef DIST_ZIPTARGET'#013#010+
  'ZIPTARGET=DIST_ZI','PTARGET'#013#010+
  'else'#013#010+
  'ZIPTARGET=install'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use tar by default under linux'#013#010+
  'ifndef USEZIP'#013#010+
  'ifdef inUnix'#013#010+
  'USETAR=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use a wrapper script by default for Os/2'#013#010+
  'ifndef inUnix'#013#010+
  'USEZIPWRAPPER=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# We need to be ','able to run in the current OS so fix'#013#010+
  '# the path separator'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  'ZIPPATHSEP=$(PATHSEP)'#013#010+
  'ZIPWRAPPER=$(subst /,$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT))'#013+
  #010+
  'else'#013#010+
  'ZIPPATHSEP=/'#013#010+
  'endif'#013#010+
  #013#010+
  '# Create commands to create the zip/','tar file'#013#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#013#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#013#010+
  'ifdef USETAR'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(TAREXT)'#013#010+
  'ZIPCMD_ZIP:=$(TARPROG) cf$(TAROPT) $(ZIPDESTFILE) *'#013,#010+
  'else'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#013#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
  'TFILE) *'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipinstall:'#013#010+
  '        $(MAKE) $(ZIPTARGET) INSTALL_PREFIX=$(PACKDIR) ZIPINSTALL=1'#013+
  #010+
  '    ','    $(MKDIR) $(DIST_DESTDIR)'#013#010+
  '        $(DEL) $(ZIPDESTFILE)'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  '# Handle gecho separate as we need to espace \ with \\'#013#010+
  'ifneq ($(ECHOREDIR),echo)'#013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDPACK))" > $(ZIPWRAPPE'+
  'R)'#013#010+
  ' ','       $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPE'+
  'R)'#013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDBASE))" >> $(ZIPWRAPP'+
  'ER)'#013#010+
  'else'#013#010+
  '        echo $(ZIPCMD_CDPACK) > $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPE','R)'#013#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'ifdef inUnix'#013#010+
  '        /bin/sh $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  'ifdef RUNBATCH'#013#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  '        $(DEL) $(ZIPWRAPPER)'#013#010+
  'else'#013#010,
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#013#010+
  'endif'#013#010+
  '        $(DELTREE) $(PACKDIR)'#013#010+
  #013#010+
  'fpc_zipsourceinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=sourceinstall ZIPSUFFIX=$(ZIP'+
  'SOURCESUFFIX)'#013#010+
  #013#010+
  'fpc_zipexampleinstall:'#013#010+
  'ifdef',' HASEXAMPLES'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipdistinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=distinstall'#013#010+
  #013#010+
  #013#010+
  '[cleanrules]'#013#010+
  '#####################################','###############################'+
  '#'#013#010+
  '# Clean rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#013#010+
  #013#010+
  'ifdef EXEFILES'#013#010+
  'override CLEANEXEFILES:=$(addprefix $(TARGETDIRPREFIX)',',$(CLEANEXEFIL'+
  'ES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_UNITS'#013#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEANPPUFILES'#013#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
  ') $(addprefix $(STATICLIBPREF','IX),$(subst $(PPUEXT),$(STATICLIBEXT),$'+
  '(CLEANPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(S'+
  'TATICLIBEXT),$(CLEANPPUFILES)))'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#013,#010+
  'endif'#013#010+
  'override CLEANPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(CLEANPPUF'+
  'ILES))'#013#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPREF'+
  'IX),$(CLEANPPULINKFILES)))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_clean: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013,#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGE','TDIRPREFIX),$(CLEANRSTFILES))'+
  #013#010+
  'endif'#013#010+
  'ifdef CLEAN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  'ifdef LIB_NAME'#013#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#013#010+
  'endif'#013#010+
  '        -$(DEL) $(FPCMADE) Package.fpc $(PPAS) script.res link.res $(F'+
  'PCEXTF','ILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *$(ASMEXT) *_ppas$(BATCHEXT)'#013#010+
  #013#010+
  'fpc_cleanall: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILE','S)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) units'#013#010+
  '        -$(DEL) *$(OE','XT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL'+
  'IBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#013#010+
  'ifneq ($(PPUEXT),.ppu)'#013#010+
  '        -$(DEL) *.o *.ppu *.a'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) *$(SMARTEXT)'#013#010+
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) script.res link','.res $('+
  'FPCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *_ppas$(BATCHEXT)'#013#010+
  'ifdef AOUTEXT'#013#010+
  '        -$(DEL) *$(AOUTEXT)'#013#010+
  'endif'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_distclean: cleanall'#013#010+
  #013#010+
  #013#010+
  '[baseinforules]'#013#010+
  '##############','######################################################'+
  '#'#013#010+
  '# Base info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_baseinfo'#013#010+
  #013#010+
  'override INFORULES+=fpc_baseinfo'#013#010+
  #013#010+
  'fpc_baseinfo:'#013#010+
  '        @$(ECHO)',#013#010+
  '        @$(ECHO)  == Package info =='#013#010+
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#013#010+
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Configuration info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(','ECHO)  FPC.......... $(FPC)'#013#010+
  '        @$(ECHO)  FPC Version.. $(FPC_VERSION)'#013#010+
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#013#010+
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#013#010+
  '        @$(ECHO)  Source OS.... $(OS_SOURCE)'#013#010+
  '        @$(ECHO)  Targe','t OS.... $(OS_TARGET)'#013#010+
  '        @$(ECHO)  Full Source.. $(FULL_SOURCE)'#013#010+
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#013#010+
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#013#010+
  '        @$(ECHO)  TargetSuffix. $(TARGETSUFFIX)'#013#010+
  '        @$(ECHO)'#013#010+
  '     ','   @$(ECHO)  == Directory info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Basedir......... $(BASEDIR)'#013#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#013#010+
  '        @$(ECHO)  Cr','ossBinDir..... $(CROSSBINDIR)'#013#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#013#010+
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  GCC library..... $(GCCLIBDIR)'#013#010+
  '        @$(ECHO)  Other library... $(OTHERL','IBDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Tools info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  As........ $(AS)'#013#010+
  '        @$(ECHO)  Ld........ $(LD)'#013#010+
  '        @$(ECHO)  Ar........ $(AR)'#013#010+
  '        @$(ECHO)  Rc........ $(RC)'#013#010+
  '        @$(ECHO)',#013#010+
  '        @$(ECHO)  Mv........ $(MVPROG)'#013#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#013#010+
  '        @$(ECHO)  Rm........ $(RMPROG)'#013#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#013#010+
  '        @$(ECHO)  Echo...... $(ECHO)'#013#010+
  '        @$(ECHO)  Shell..... $(SHELL)'#013,#010+
  '        @$(ECHO)  Date...... $(DATE)'#013#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#013#010+
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#013#010+
  '        @$(ECHO)  Upx....... $(UPXPROG)'#013#010+
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  =','= Object info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Target Loaders........ $(TARGET_LOADERS)'#013#010+
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#013#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#013#010+
  '        @$(ECHO)  Ta','rget Programs....... $(TARGET_PROGRAMS)'#013#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#013#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#013#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#013#010+
  '        @$(ECHO)'#013,#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#013#010+
  '        @$(ECHO)  Clean Files......... $(CLEAN_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#013#010+
  '        @$(ECHO)  Install Files....... $(INSTALL_FILES',')'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Install info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  DateStr.............. $(DATESTR)'#013#010+
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#013#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPREFIX)'#013#010+
  '     ','   @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#013#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#013#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#013#010+
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#013#010+
  '        @$(EC','HO)'#013#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#013#010+
  '        @$(ECHO)  Install binary dir... $(INSTALL_BINDIR)'#013#010+
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#013#010+
  '        @$(ECHO)  Install units dir.... $(INSTALL_UNITDIR',')'#013#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#013#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#013#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#013#010+
  '        @$(ECHO)  Install data dir..... $(INSTALL_DAT','ADIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#013#010+
  '        @$(ECHO)  Dist zip name........ $(DIST_ZIPNAME)'#013#010+
  '        @$(ECHO)'#013#010+
  #013#010+
  '[inforules]'#013#010+
  '####################################################################','#'+
  #013#010+
  '# Info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_info'#013#010+
  #013#010+
  'fpc_info: $(INFORULES)'#013#010+
  #013#010+
  '[makefilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Rebuild Makefi','le'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1 fpc_makefile_sub2'+
  ' \'#013#010+
  '        fpc_makefile_dirs'#013#010+
  #013#010+
  'fpc_makefile:'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) Makefile','.fpc'#013#010+
  #013#010+
  'fpc_makefile_sub1:'#013#010+
  'ifdef TARGET_DIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_DIRS))'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_EXAMPLED','IRS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_makefile_sub2: $(addsuffix _makefile_dirs,$(TARGET_DIRS) $(TARGET_'+
  'EXAMPLEDIRS))'#013#010+
  #013#010+
  'fpc_makefile_dirs: fpc_makefile_sub1 fpc_makefile_sub2'#013#010+
  #013#010+
  'fpc_makefiles: fpc_makefile fpc_makefile_dirs'#013#010+
  #013#010+
  '[localmakefile]'#013#010+
  '#############','#######################################################'+
  '#'#013#010+
  '# Local Makefile'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  'ifneq ($(wildcard fpcmake.loc),)'#013#010+
  'include fpcmake.loc'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[userrules]'#013#010+
  '############','########################################################'+
  '#'#013#010+
  '# Users rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '[lclrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# LCL Rule','s'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# LCL Platform'#013#010+
  'ifndef LCL_PLATFORM'#013#010+
  'ifneq ($(findstring $(OS_TARGET),win32 win64),)'#013#010+
  'LCL_PLATFORM=win32'#013#010+
  'else'#013#010+
  'LCL_PLATFORM=gtk'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export LCL_PLATFORM'#013,#010+
  #013#010+
  '# Check if the specified LCLDIR is correct'#013#010+
  'ifdef LCLDIR'#013#010+
  'override LCLDIR:=$(subst \,/,$(LCLDIR))'#013#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the de','fault LCLDIR is correct'#013#010+
  'ifdef DEFAULT_LCLDIR'#013#010+
  'override LCLDIR:=$(subst \,/,$(DEFAULT_LCLDIR))'#013#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check for development version'#013#010+
  'ifeq ($(LCLDIR),wro','ng)'#013#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
  ' $(addsuffix /units/$(LCL_PLATFORM),$(BASEDIR)/lcl $(BASEDIR)))))'#013#010+
  'ifeq ($(LCLDIR),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check for release version'#013#010+
  'ifeq ($(LCL','DIR),wrong)'#013#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
>>>>>>> graemeg/fixes_2_2
=======
  ';'#013#010+
  #013#010+
  '[defines]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Misc defines to be used by anyone'#013#010+
  '#############################################','#######################'+
  '#'#013#010+
  #013#010+
  '# OS categories'#013#010+
  'BSDs = freebsd netbsd openbsd darwin'#013#010+
  'UNIXs = linux $(BSDs) solaris qnx'#013#010+
  'LIMIT83fs = go32v2 os2 emx watcom'#013#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#013#010+
  #013#010+
  '#Empty target for rules that always should ','run. Needed if'#013#010+
  '#the target is non-phoney, and there is non-phony prereqisites.'#013#010+
  '#Then add FORCE as an prerequisite'#013#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#013#010+
  'FORCE:'#013#010+
  '.PHONY: FORCE'#013#010+
  #013#010+
  '[osdetect]'#013#010+
  '###################','#################################################'+
  '#'#013#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#013+
  #010+
  '# define inUnix when running under Unix like environment'#013#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Darwin,Cygwin)',#013#010+
  '# define inWinNT when running under WinNT'#013#010+
  '# define inOS2 when running under OS/2'#013#010+
  '# define inCygwin when running under Cygwin32'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# We need only / in the path also r','emove the current dir,'#013#010+
  '# also remove trailing /'#039's'#013#010+
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#013#010+
  #013#010+
  '# Detect unix'#013#010+
  '# Darwin is handled specially'#013#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#013#010+
  'inUnix=1 #darwin'#013#010+
  'SEARCHPATH:=$(filter-out .,','$(subst :, ,$(PATH)))'#013#010+
  'else'#013#010+
  '# Determine if we'#039've a unix searchpath by looking for a ;'#013#010+
  '# that normally doesn'#039't exists in the unix PATH var.'#013#010+
  'ifeq ($(findstring ;,$(PATH)),)'#013#010+
  'inUnix=1'#013#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#013#010+
  'else'#013,#010+
  'SEARCHPATH:=$(subst ;, ,$(PATH))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add path were make is located'#013#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#013#010+
  #013#010+
  '# Search for PWD'#013#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013,#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd,$(SEARCHPATH))))'#013#010+
  'ifeq ($(PWD),)'#013#010+
  '$(error You need the GNU utils package to use this Makefile)'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT='#013#010+
  'endif'#013#010+
  'else'#013#010+
  'PWD:=$(firstword $(PWD))'#013#010+
  'SRCEXEEXT=.exe'#013#010+
  'end','if'#013#010+
  #013#010+
  '# Detect NT - NT sets OS to Windows_NT'#013#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#013#010+
  'ifndef inUnix'#013#010+
  'ifeq ($(OS),Windows_NT)'#013#010+
  'inWinNT=1'#013#010+
  'else'#013#010+
  'ifdef OS2_SHELL'#013#010+
  'inOS2=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifneq ($(findstring cygdrive,$(PATH)),)'#013#010+
  'inCygW','in=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# The extension of batch files / scripts'#013#010+
  'ifdef inUnix'#013#010+
  'SRCBATCHEXT=.sh'#013#010+
  'else'#013#010+
  'ifdef inOS2'#013#010+
  'SRCBATCHEXT=.cmd'#013#010+
  'else'#013#010+
  'SRCBATCHEXT=.bat'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef COMSPEC'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(OSNeedsComspecToRu','nBatch)),)'#013#010+
  'RUNBATCH=$(COMSPEC) /C'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#013#010+
  '# at the end of a line'#013#010+
  'ifdef inUnix'#013#010+
  'PATHSEP=/'#013#010+
  'else'#013#010+
  'PATHSEP:=$(subst /,\,/)'#013#010+
  '# cygwin bash or sh can not hand','le backslashs'#013#010+
  'ifdef inCygWin'#013#010+
  'PATHSEP=/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Base dir'#013#010+
  'ifdef PWD'#013#010+
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#013#010+
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#013#010+
  'ifdef inCygWin'#013#010+
  'ifneq ($(findstring /cygdrive/,$(BASEDIR)),)'#013#010+
  'BAS','ENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#013#010+
  'BASEDRIVE:=$(firstword $(subst /, ,$(BASENODIR)))'#013#010+
  'BASEDIR:=$(subst /cygdrive/$(BASEDRIVE)/,$(BASEDRIVE):/,$(BASEDIR))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'BASEDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  '# Echo is an internal command under',' OS/2 (and others), so it'#039's a'+
  'lways found'#013#010+
  'ifdef inOS2'#013#010+
  'ifndef ECHO'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH))'+
  '))'#013#010+
  'ifeq ($(ECHO),)'#013#010+
  'ECHO:=$(strip $(wildcard $(addsuffix /echo$(SRCEXEEXT),$(SEARCHPATH)))'+
  ')'#013#010+
  'ifeq ($(','ECHO),)'#013#010+
  'ECHO=echo'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ECHO:=$(firstword $(ECHO))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export ECHO'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdetect]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPC Binary and Version ','Detection'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Compatibility with old makefiles'#013#010+
  'ifndef FPC'#013#010+
  'ifdef PP'#013#010+
  'FPC=$(PP)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try to detect the ppcXXX file to use by using "fpc -PB" option'#013#010+
  '# to ','query for the default ppcXXX the fpc executable tries. When'#013+
  #010+
  '# fpc is not found use ppc386 by default. Also when fpc -PB gives'#013#010+
  '# an error ppc386 will be used.'#013#010+
  'ifndef FPC'#013#010+
  '# check if fpc exists'#013#010+
  'FPCPROG:=$(strip $(wildcard $(addsuffix /fpc','$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifneq ($(FPCPROG),)'#013#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#013#010+
  'ifneq ($(CPU_TARGET),)'#013#010+
  'FPC:=$(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#013#010+
  'else'#013#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#013#010+
  'endif'#013#010+
  '# Older fpc executables didn'#039't support it',' and return'#013#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#013#010+
  'ifneq ($(findstring Error,$(FPC)),)'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#013#010+
  'else'#013#010+
  '# if the cross compiler i','s not found, fall back to fpc'#013#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#013#010+
  'FPC:=$(firstword $(FPCPROG))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# fpc binary not found, fallback to ppc386'#013#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(','SEARCHPATH)))))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Get a clean executable name'#013#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#013#010+
  'override FPC:=$(subst \,/,$(FPC))$(SRCEXEEXT)'#013#010+
  #013#010+
  '# Try to find the binary direct first, otherwise in the path,'#013#010+
  '# if not found give',' an error'#013#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#013#010+
  'ifeq ($(FOUNDFPC),)'#013#010+
  '$(error Compiler $(FPC) not found)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.1 and up we can use a ','single compiler call to retrieve'#013#010+
  '# all needed information'#013#010+
  'ifndef FPC_COMPILERINFO'#013#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# FPC version'#013#010+
  'ifndef FPC_VERSION'#013#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  #013#010+
  'export FPC ','FPC_VERSION FPC_COMPILERINFO'#013#010+
  '# CHECKDEPEND should not be exported'#013#010+
  '# This should limit multiple checks'#013#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# FPC Target Detection'#013#010,
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Fall back to default values if needed'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'ifdef CPU_TARGET_DEFAULT'#013#010+
  'CPU_TARGET=$(CPU_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'ifdef OS_TARGET','_DEFAULT'#013#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 1.0.x we need to use extra calls to retrieve all info'#013#010+
  'ifneq ($(words $(FPC_COMPILERINFO)),5)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iSP)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTP)'#013#010+
  'F','PC_COMPILERINFO+=$(shell $(FPC) -iSO)'#013#010+
  'FPC_COMPILERINFO+=$(shell $(FPC) -iTO)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Retrieve Target/Source CPU and Target/Source OS'#013#010+
  'ifndef CPU_SOURCE'#013#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef CPU_TARGET'#013#010+
  'CPU_TARGET:=$','(word 3,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_SOURCE'#013#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'ifndef OS_TARGET'#013#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#013#010+
  'endif'#013#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#013#010+
  'FULL_SOURCE=$(CPU_SOURCE)-$(O','S_SOURCE)'#013#010+
  #013#010+
  '# Full name of the target, including CPU and OS. For OSs limited'#013#010+
  '# to 8.3 we only use the target OS'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'TARGETSUFFIX=$(OS_TARGET)'#013#010+
  'SOURCESUFFIX=$(OS_SOURCE)'#013#010+
  'else'#013#010+
  'TARGETSUFFIX=$(','FULL_TARGET)'#013#010+
  'SOURCESUFFIX=$(FULL_SOURCE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compile flag'#013#010+
  'ifneq ($(FULL_TARGET),$(FULL_SOURCE))'#013#010+
  'CROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the Makefile supports this target, but not'#013#010+
  '# when the make target is to rebuild the makefi','le'#013#010+
  'ifeq ($(findstring makefile,$(MAKECMDGOALS)),)'#013#010+
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),)'#013#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Detect BSD, since BSD us','es a slightly different directory hierarch'+
  'y.'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#013#010+
  'BSDhier=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Detect Linux, will also use its own directory hierarchy.'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'linuxHier=1'#013#010+
  'endif'#013#010+
  #013#010+
  'export OS_TARGET OS_','SOURCE CPU_TARGET CPU_SOURCE FULL_TARGET FULL_SO'+
  'URCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#013#010+
  #013#010+
  #013#010+
  '[fpcdircheckenv]'#013#010+
  '#####################################################################'#013+
  #010+
  '# FPCDIR Setting'#013#010+
  '####################################','################################'+
  '#'#013#010+
  #013#010+
  '# Test FPCDIR to look if the RTL dir exists'#013#010+
  'ifdef FPCDIR'#013#010+
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FPC','DIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default FPCDIR from Makefile.fpc'#013#010+
  'ifdef DEFAULT_FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=wrong'#013#010+
  'endif'#013#010+
  'endi','f'#013#010+
  'endif'#013#010+
  #013#010+
  '[fpcdirdetect]'#013#010+
  '# Detect FPCDIR'#013#010+
  'ifeq ($(FPCDIR),wrong)'#013#010+
  'ifdef inUnix'#013#010+
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#013#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#013#010+
  'override FPCDIR=/usr/lib/fpc/$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override FP','CDIR:=$(subst /$(FPC),,$(firstword $(strip $(wildcard $(a'+
  'ddsuffix /$(FPC),$(SEARCHPATH))))))'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(FPCDIR)/..'#013#010+
  'ifeq ($(wildcard $(addprefix ','$(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR:=$(BASEDIR)'#013#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl units)),)'#013#010+
  'override FPCDIR=c:/pp'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross binaries dir'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(FP','CDIR)/bin/$(TARGETSUFFIX))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default binutils prefix for cross compile when the'#013#010+
  '# crossbindir is not set (except for Darwin)'#013#010+
  'ifeq ($(OS_TARGET),darwin)'#013#010+
  'ifeq ($(OS_SOURCE),darwin)'#013#010+
  'DARWIN2DARWIN=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef BINUTIL','SPREFIX'#013#010+
  'ifndef CROSSBINDIR'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifndef DARWIN2DARWIN'#013#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try first the full target name, otherwise try only'#013#010+
  '# the OS for backwards compatibility'#013#010+
  'UNI','TSDIR:=$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#013#010+
  'ifeq ($(UNITSDIR),)'#013#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(OS_TARGET))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Packages dir'#013#010+
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages $(FPCDIR)/package'+
  's/base $(FPCDIR)/pac','kages/extra)'#013#010+
  #013#010+
  #013#010+
  '[shelltools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Shell tools'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Echo that can redir (must be able run in the de','fault OS shell)'#013+
  #010+
  'ifndef ECHOREDIR'#013#010+
  'ifndef inUnix'#013#010+
  'ECHOREDIR=echo'#013#010+
  'else'#013#010+
  'ECHOREDIR=$(ECHO)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To copy pograms'#013#010+
  'ifndef COPY'#013#010+
  'COPY:=$(CPPROG) -fp'#013#010+
  'endif'#013#010+
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef COPYTREE'#013#010+
  'COPYTREE:=$(CPPROG) -Rfp'#013#010+
  'endif'#013#010,
  #013#010+
  '# Copy a whole tree'#013#010+
  'ifndef MKDIRTREE'#013#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#013#010+
  'endif'#013#010+
  #013#010+
  '# To move pograms'#013#010+
  'ifndef MOVE'#013#010+
  'MOVE:=$(MVPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check delete program'#013#010+
  'ifndef DEL'#013#010+
  'DEL:=$(RMPROG) -f'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check deltree program'#013#010+
  'ifndef DELTR','EE'#013#010+
  'DELTREE:=$(RMPROG) -rf'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install files'#013#010+
  'ifndef INSTALL'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALL:=$(GINSTALL) -c -m 644'#013#010+
  'else'#013#010+
  'INSTALL:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To install programs'#013#010+
  'ifndef INSTALLEXE'#013#010+
  'ifdef inUnix'#013#010+
  'INSTALLEXE:=$(GINSTALL) -','c -m 755'#013#010+
  'else'#013#010+
  'INSTALLEXE:=$(COPY)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# To make a directory.'#013#010+
  'ifndef MKDIR'#013#010+
  'MKDIR:=$(GINSTALL) -m 755 -d'#013#010+
  'endif'#013#010+
  #013#010+
  'export ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE MKD'+
  'IR'#013#010+
  #013#010+
  #013#010+
  '[defaultdirs]'#013#010+
  '#################','###################################################'+
  '#'#013#010+
  '# Default Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Units dir'#013#010+
  'ifdef REQUIRE_UNITSDIR'#013#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Un','its dir'#013#010+
  'ifdef REQUIRE_PACKAGESDIR'#013#010+
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#013#010+
  '# When zipping use the target os default, when normal install then'#013+
  #010+
  '# use the source os as default'#013,#010+
  'ifdef ZIPINSTALL'#013#010+
  '# Zipinstall'#013#010+
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '# Normal install'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs)),)'#013#010+
  'UNIXHier=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# When install prefix is not set try to use ','prefix'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef PREFIX'#013#010+
  'INSTALL_PREFIX=$(PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the prefix directory where to install everything'#013#010+
  'ifndef INSTALL_PREFIX'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_PREFIX=/usr/local'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013,#010+
  'INSTALL_BASEDIR:=/pp'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export INSTALL_PREFIX'#013#010+
  #013#010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#013#010+
  '# used recursively for all subdirs'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'export INSTALL_','FPCSUBDIR'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to place the resulting zip files'#013#010+
  'ifndef DIST_DESTDIR'#013#010+
  'DIST_DESTDIR:=$(BASEDIR)'#013#010+
  'endif'#013#010+
  'export DIST_DESTDIR'#013#010+
  #013#010+
  '# EXE/PPU Target directories'#013#010+
  'ifndef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef PACKAGEDIR_MAIN'#013#010+
  'COMPILER_UNITTARG','ETDIR=$(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#013#010+
  'else'#013#010+
  'COMPILER_UNITTARGETDIR=units/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifndef COMPILER_TARGETDIR'#013#010+
  'COMPILER_TARGETDIR=.'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '##################################################################','##'+
  '#'#013#010+
  '# Install Directories'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# set the base directory where to install everything'#013#010+
  'ifndef INSTALL_BASEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_BASEDIR:=$(','INSTALL_PREFIX)/lib/fpc/$(FPC_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the binaries'#013#010+
  'ifndef INSTALL_BINDIR'#013,#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/bin'#013#010+
  '# for FPC packages install the binaries under their target subdir'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ifdef CROSSINSTALL'#013#010+
  'INSTA','LL_BINDIR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# set the directory where to install the un','its.'#013#010+
  'ifndef INSTALL_UNITDIR'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef PACKAGE_NAME'#013#010+
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install shared lib','raries'#013#010+
  'ifndef INSTALL_LIBDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#013#010+
  'else'#013#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the source files will be stored'#013#010+
  'ifndef INSTALL_SOURCEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier',#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'SRCPREFIXDIR=share/src'#013#010+
  'else'#013#010+
  'SRCPREFIXDIR=src'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INST','ALL_FPCSUBDIR)/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else',#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'ifdef INSTALL_FPCSUBDIR'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$(PA'+
  'CKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'INSTALL_SOURCEDIR:=$(IN','STALL_BASEDIR)/source'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the doc files will be stored'#013#010+
  'ifndef INSTALL_DOCDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef BSDhier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'DOCPREFIXDIR=share/doc'#013#010+
  'else'#013#010+
  'DOCPREFIXDIR=doc'#013#010+
  'endif'#013#010,
  'endif'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/fpc-$(FPC_VERSION)/$'+
  '(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACKAGE_NAME)-$(PA'+
  'CKAGE_VERSION)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_FP','CPACKAGE'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where to install the examples, under linux we use the doc dir'#013#010+
  '# because the copytree command will crea','te a subdir itself'#013#010+
  'ifndef INSTALL_EXAMPLEDIR'#013#010+
  'ifdef UNIXHier'#013#010+
  'ifdef INSTALL_FPCPACKAGE'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(','INSTALL_DOCDIR)/examples'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  #013#010+
  'ifdef BSDhier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/$(PACKAGE_NAME)-$'+
  '(PACKAGE_VER','SION)'#013#010+
  'else'#013#010+
  'ifdef linuxHier'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef INSTALL_','FPCPACKAGE'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#013#010+
  'else'#013#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Where the some extra (data)files will be stored'#013#010+
  'ifndef INSTALL_DATADIR'#013#010+
  'INSTALL_DA','TADIR=$(INSTALL_BASEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef INSTALL_SHAREDDIR'#013#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#013#010+
  'endif'#013#010+
  #013#010+
  '#####################################################################'#013+
  #010+
  '# Cross compile dirs'#013#010+
  '####################################','################################'+
  '#'#013#010+
  #013#010+
  'ifdef CROSSCOMPILE'#013#010+
  '# Directory where the cross compile tools are stored.'#013#010+
  '# First check if they are available in FPCDIR. If no targets/ subdir'#013+
  #010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#013#010+
  'if','ndef CROSSBINDIR'#013#010+
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#013#010+
  'ifeq ($(CROSSBINDIR),)'#013#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/cross/$(TARGETSUFFIX)/bin/$'+
  '(FULL_SOURCE))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'CROSSBINDIR='#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[di','rlibc]'#013#010+
  '# On linux, try to find where libgcc.a is.'#013#010+
  'ifeq ($(OS_SOURCE),linux)'#013#010+
  #013#010+
  '# Amd64 to i386?'#013#010+
  'ifndef GCCLIBDIR'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'ifneq ($(findstring x86_64,$(shell uname -a)),)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell d','irname `gcc -m32 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#013#010+
  'ifeq ($(BINUTILSPREFIX),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Try cross gcc'#013#010+
  'ifndef GCCLI','BDIR'#013#010+
  'CROSSGCC=$(strip $(wildcard $(addsuffix /$(BINUTILSPREFIX)gcc$(SRCEXEE'+
  'XT),$(SEARCHPATH))))'#013#010+
  'ifneq ($(CROSSGCC),)'#013#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-name`)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Other libraries from ld.so.conf'#013#010+
  'if','ndef OTHERLIBDIR'#013#010+
  'OTHERLIBDIR:=$(shell grep -v "^\#" /etc/ld.so.conf | awk '#039'{ ORS=" '+
  '"; print $1 }'#039')'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef inUnix'#013#010+
  'ifeq ($(OS_SOURCE),netbsd)'#013#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#013#010+
  'endif'#013#010+
  'export GCCLIBDIR OTHERLIB'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[extensio','ns]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Default extensions'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# Default needed extensions (Go32v2,Linux)'#013#010+
  'BATCHEXT=.bat'#013#010+
  'LOADEREXT=','.as'#013#010+
  'EXEEXT=.exe'#013#010+
  'PPLEXT=.ppl'#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHAREDLIBPREFIX=libfp'#013#010+
  'STATICLIBPREFIX=libp'#013#010+
  'IMPORTLIBPREFIX=libimp'#013#010+
  'RSTEXT=.rst'#013#010+
  '#DEBUGSYMEXT #for debugger symbol files, defi','ne only for targets whi'+
  'ch has this'#013#010+
  #013#010+
  '# 1.0.x has target specific extensions for ppu files and objects'#013#010+
  'ifeq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  '# short version for 1.1 and above - no target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TAR','GET),go32v1)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'OEXT=.obj'#013#010+
  'ASMEXT=.asm'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SH','ORTSUFFIX=wat'#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# FreeBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010,
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'SHARED','LIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'endif'#013#010+
  #013#010+
  '# EMX'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'AOUTEXT=.out'#013#010+
  'STAT','ICLIBPREFIX='#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=emx'#013#010+
  'ECHO=echo'#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# MorphOS'#013#010+
  'ifeq ($(OS_TARGET),morphos)'#013#010+
  'EXEEXT='#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHOR','TSUFFIX=mos'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT=',#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware clib'#013#010+
  'ifeq ($(OS_TARGET),netware)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TAR','GET),netwlibc)'#013#010+
  'EXEEXT=.nlm'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFIX=mac'#013#010+
  'endif'#013#010+
  #013#010+
  '# Darwin'#013#010+
  'ifeq ($(OS_TARGET),darwin)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASS','HAREDLIB=1'#013#010+
  'SHORTSUFFIX=dwn'#013#010+
  'endif'#013#010+
  #013#010+
  '# gba'#013#010+
  'ifeq ($(OS_TARGET),gba)'#013#010+
  'EXEEXT=.gba'#013#010+
  'SHAREDLIBEXT=.so'#013#010+
  'SHORTSUFFIX=gba'#013#010+
  'endif'#013#010+
  #013#010+
  '# Symbian OS'#013#010+
  'ifeq ($(OS_TARGET),symbian)'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=symbian'#013#010+
  'endif'#013#010+
  #013#010+
  'else'#013#010+
  '# long version ','for 1.0.x - target specific extensions'#013#010+
  #013#010+
  '# Go32v1'#013#010+
  'ifeq ($(OS_TARGET),go32v1)'#013#010+
  'PPUEXT=.pp1'#013#010+
  'OEXT=.o1'#013#010+
  'ASMEXT=.s1'#013#010+
  'SMARTEXT=.sl1'#013#010+
  'STATICLIBEXT=.a1'#013#010+
  'SHAREDLIBEXT=.so1'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=v1'#013#010+
  'endif'#013#010+
  #013#010+
  '# Go32v2'#013#010+
  'ifeq ($(OS_TARGET),g','o32v2)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=dos'#013#010+
  'endif'#013#010+
  #013#010+
  '# watcom'#013#010+
  'ifeq ($(OS_TARGET),watcom)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'SHORTSUFFIX=wat'#013#010+
  'endif'#013#010+
  #013#010+
  '# Linux'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=lnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Fre','eBSD'#013#010+
  'ifeq ($(OS_TARGET),freebsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=fbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# NetBSD'#013#010+
  'ifeq ($(OS_TARGET),netbsd)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=nbs'#013#010+
  'endif'#013#010+
  #013#010+
  '# OpenBSD'#013#010+
  'ifeq ($(OS_TARGET),openbsd',')'#013#010+
  'BATCHEXT=.sh'#013#010+
  'EXEEXT='#013#010+
  'HASSHAREDLIB=1'#013#010+
  'SHORTSUFFIX=obs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Win32'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'PPUEXT=.ppw'#013#010+
  'OEXT=.ow'#013#010+
  'ASMEXT=.sw'#013#010+
  'SMARTEXT=.slw'#013#010+
  'STATICLIBEXT=.aw'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=w32'#013#010+
  'endif'#013#010+
  #013#010+
  '# OS/2'#013#010+
  'ifeq ($(OS_TARG','ET),os2)'#013#010+
  'BATCHEXT=.cmd'#013#010+
  'PPUEXT=.ppo'#013#010+
  'ASMEXT=.so2'#013#010+
  'OEXT=.oo2'#013#010+
  'AOUTEXT=.out'#013#010+
  'SMARTEXT=.sl2'#013#010+
  'STATICLIBPREFIX='#013#010+
  'STATICLIBEXT=.ao2'#013#010+
  'SHAREDLIBEXT=.dll'#013#010+
  'SHORTSUFFIX=os2'#013#010+
  'ECHO=echo'#013#010+
  'endif'#013#010+
  #013#010+
  '# Amiga'#013#010+
  'ifeq ($(OS_TARGET),amiga)'#013#010+
  'EXEEXT='#013#010+
  'PPUEXT=.ppu',#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.library'#013#010+
  'SHORTSUFFIX=amg'#013#010+
  'endif'#013#010+
  #013#010+
  '# Atari'#013#010+
  'ifeq ($(OS_TARGET),atari)'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT=.ttp'#013#010+
  'SHORTSUFFIX=ata'#013#010+
  'endif'#013#010+
  #013,#010+
  '# BeOS'#013#010+
  'ifeq ($(OS_TARGET),beos)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=be'#013#010+
  'endif'#013#010+
  #013#010+
  '# Solaris'#013#010+
  'ifeq ($(OS_TARGET),solaris)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTE','XT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=sun'#013#010+
  'endif'#013#010+
  #013#010+
  '# QNX'#013#010+
  'ifeq ($(OS_TARGET),qnx)'#013#010+
  'BATCHEXT=.sh'#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'SHORTSUFFIX=qnx'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware'#013#010+
  'ifeq ($(OS_TARGET),netwa','re)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nw'#013#010+
  'endif'#013#010+
  #013#010+
  '# Netware libc'#013#010+
  'ifeq ($(OS_TARGET),netwlibc)'#013#010+
  'STATICLIBPREFIX='#013#010+
  'PPUEXT=.ppu'#013#010+
  'OEXT=.o'#013#010+
  'ASMEXT=.s'#013,#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'SHAREDLIBEXT=.nlm'#013#010+
  'EXEEXT=.nlm'#013#010+
  'SHORTSUFFIX=nwl'#013#010+
  'endif'#013#010+
  #013#010+
  '# MacOS'#013#010+
  'ifeq ($(OS_TARGET),macos)'#013#010+
  'BATCHEXT='#013#010+
  'PPUEXT=.ppu'#013#010+
  'ASMEXT=.s'#013#010+
  'OEXT=.o'#013#010+
  'SMARTEXT=.sl'#013#010+
  'STATICLIBEXT=.a'#013#010+
  'EXEEXT='#013#010+
  'DEBUGSYMEXT=.xcoff'#013#010+
  'SHORTSUFFI','X=mac'#013#010+
  'endif'#013#010+
  #013#010+
  '#end of target specific settings'#013#010+
  'endif'#013#010+
  #013#010+
  '# For 8.3 limited OS'#039's the short suffixes'#013#010+
  '# Otherwise use the full source/target names'#013#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#013#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)'#013#010+
  'ZIPSUFFIX=','$(SHORTSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'ZIPSOURCESUFFIX=src'#013#010+
  'ZIPEXAMPLESUFFIX=exm'#013#010+
  'else'#013#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#013#010+
  'ZIPSOURCESUFFIX=.source'#013#010+
  'ZIPEXAMPLESUFFIX=.examples'#013#010+
  'ifdef CROSSCOMPILE'#013#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#013#010+
  'ZIPCROSSPREFIX=$(TARGETS','UFFIX)-'#013#010+
  'else'#013#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#013#010+
  'ZIPCROSSPREFIX='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[defaulttools]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Default Tools'#013#010+
  '############################################################','########'+
  '#'#013#010+
  #013#010+
  '# Names of the binutils tools'#013#010+
  'ASNAME=$(BINUTILSPREFIX)as'#013#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#013#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#013#010+
  'RCNAME=$(BINUTILSPREFIX)rc'#013#010+
  'ifneq ($(findstring 1.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'ifeq ($(CROSS','BINDIR),)'#013#010+
  'ASNAME=asw'#013#010+
  'LDNAME=ldw'#013#010+
  'ARNAME=arw'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# assembler, redefine it if cross compiling'#013#010+
  'ifndef ASPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ASPROG=$(CROSSBINDIR)/$(ASNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ASPROG=$(ASNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# link','er, but probably not used'#013#010+
  'ifndef LDPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'LDPROG=$(CROSSBINDIR)/$(LDNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'LDPROG=$(LDNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Resource compiler'#013#010+
  'ifndef RCPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'RCPROG=$(CROSSBINDIR)/$(RCNAME)$(SRCEXE','EXT)'#013#010+
  'else'#013#010+
  'RCPROG=$(RCNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Archiver'#013#010+
  'ifndef ARPROG'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'ARPROG=$(CROSSBINDIR)/$(ARNAME)$(SRCEXEEXT)'#013#010+
  'else'#013#010+
  'ARPROG=$(ARNAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Override defaults'#013#010+
  'AS=$(ASPROG)'#013#010+
  'LD=$(LDPROG)'#013#010+
  'RC=$(RCPROG)'#013#010,
  'AR=$(ARPROG)'#013#010+
  #013#010+
  '# ppas.bat / ppas.sh'#013#010+
  'PPAS=ppas$(SRCBATCHEXT)'#013#010+
  #013#010+
  '# ldconfig to rebuild .so cache'#013#010+
  'ifdef inUnix'#013#010+
  'LDCONFIG=ldconfig'#013#010+
  'else'#013#010+
  'LDCONFIG='#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef DATE'#013#010+
  'DATESTR:=$(shell $(DATE) +%Y%m%d)'#013#010+
  'else'#013#010+
  'DATESTR='#013#010+
  'endif'#013#010+
  #013#010+
  '# Look if U','PX is found for go32v2 and win32. We can'#039't use $UPX b'+
  'ecuase'#013#010+
  '# upx uses that one itself (PFV)'#013#010+
  'ifndef UPXPROG'#013#010+
  'ifeq ($(OS_TARGET),go32v2)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),win32)'#013#010+
  'UPXPROG:=1'#013#010+
  'endif'#013#010+
  'ifdef UPXPROG'#013#010+
  'UPXPROG:=$(strip $(wi','ldcard $(addsuffix /upx$(SRCEXEEXT),$(SEARCHPAT'+
  'H))))'#013#010+
  'ifeq ($(UPXPROG),)'#013#010+
  'UPXPROG='#013#010+
  'else'#013#010+
  'UPXPROG:=$(firstword $(UPXPROG))'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'UPXPROG='#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export UPXPROG'#013#010+
  #013#010+
  '# Zip options'#013#010+
  'ZIPOPT=-9'#013#010+
  'ZIPEXT=.zip'#013#010+
  #013#010+
  '# Tar options'#013#010+
  'ifeq ','($(USETAR),bz2)'#013#010+
  'TAROPT=vj'#013#010+
  'TAREXT=.tar.bz2'#013#010+
  'else'#013#010+
  'TAROPT=vz'#013#010+
  'TAREXT=.tar.gz'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_begin]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Compiler Command Line'#013#010+
  '######################################','##############################'+
  '#'#013#010+
  #013#010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#013+
  #010+
  '# compiler this needs to be turned off'#013#010+
  'ifndef NOCPUDEF'#013#010+
  'override FPCOPTDEF=$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '# Load commandline OPT and add tar','get and unit dir to be sure'#013#010+
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#013#010+
  'override FPCOPT+=-T$(OS_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#013#010+
  'override FPCOPT+=-P$(CPU_TARGET)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifeq ($(OS_SOURCE),openbsd)'#013#010+
  'override FPCOPT+=-FD$(NEW','_BINUTILS_PATH)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef CROSSBOOTSTRAP'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-XP$(BINUTILSPREFIX)'#013#010+
  'endif'#013#010+
  'ifneq ($(BINUTILSPREFIX),)'#013#010+
  'override FPCOPT+=-Xr$(RLINKPATH)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# User dirs should be first, so they ar','e looked at first'#013#010+
  'ifdef UNITDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef LIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#013#010+
  'endif'#013#010+
  'ifdef OBJDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fo,$(OBJDIR))'#013#010+
  'endif'#013#010+
  'ifdef INCDIR'#013#010+
  'overrid','e FPCOPT+=$(addprefix -Fi,$(INCDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking'#013#010+
  'ifdef LINKSMART'#013#010+
  'override FPCOPT+=-XX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Smartlinking creation'#013#010+
  'ifdef CREATESMART'#013#010+
  'override FPCOPT+=-CX'#013#010+
  'endif'#013#010+
  #013#010+
  '# Debug'#013#010+
  'ifdef DEBUG'#013#010+
  'override FPCOPT+=-gl'#013#010+
  'override F','PCOPTDEF+=DEBUG'#013#010+
  'endif'#013#010+
  #013#010+
  '# Release mode'#013#010+
  '# (strip, optimize and don'#039't load fpc.cfg)'#013#010+
  '# fpc 2.1 has -O2 for all targets'#013#010+
  'ifdef RELEASE'#013#010+
  'ifneq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'FPCCPUOPT:=-OG2p3'#013#010+
  'endif'#013#010+
  'ifeq ($','(CPU_TARGET),powerpc)'#013#010+
  'FPCCPUOPT:=-O1r'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'FPCCPUOPT:=-O2'#013#010+
  'endif'#013#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#013#010+
  'override FPCOPTDEF+=RELEASE'#013#010+
  'endif'#013#010+
  #013#010+
  '# Strip'#013#010+
  'ifdef STRIP'#013#010+
  'override FPCOPT+=-Xs'#013#010+
  'endif'#013#010+
  #013#010+
  '# Optimizer'#013#010+
  'ifdef OPTIMIZE'#013#010+
  'ov','erride FPCOPT+=-O2'#013#010+
  'endif'#013#010+
  #013#010+
  '# Verbose settings (warning,note,info)'#013#010+
  'ifdef VERBOSE'#013#010+
  'override FPCOPT+=-vwni'#013#010+
  'endif'#013#010+
  #013#010+
  '# Needed compiler options'#013#010+
  'ifdef COMPILER_OPTIONS'#013#010+
  'override FPCOPT+=$(COMPILER_OPTIONS)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITDIR'#013#010+
  'ove','rride FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_LIBRARYDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_OBJECTDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#013#010+
  'endif'#013,#010+
  'ifdef COMPILER_INCLUDEDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Cross compiler utils'#013#010+
  'ifdef CROSSBINDIR'#013#010+
  'override FPCOPT+=-FD$(CROSSBINDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Target dirs and the prefix to use for clean/install'#013#010+
  'if','def COMPILER_TARGETDIR'#013#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#013#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#013#010+
  'override TARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR)/'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Unit target dir (default is units/<cpu>-<','os>/'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#013#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#013#010+
  'override UNITTARGETDIRPREFIX='#013#010+
  'else'#013#010+
  'override UNITTARGETDIRPREFIX=$(COMPILER_UNITTARGETDIR)/'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'ifdef COMPILER','_TARGETDIR'#013#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#013#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CREATESHARED'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  #013#010+
  'ifeq ($(CPU_TARGET),i386)'#013#010+
  'override FPCOPT+=-Aas'#013#010+
  'endif'#013#010+
  #013#010+
  'endif',#013#010+
  #013#010+
  '# create always pic'#039'ed code on x86_64'#013#010+
  'ifeq ($(findstring 2.0.,$(FPC_VERSION)),)'#013#010+
  'ifeq ($(OS_TARGET),linux)'#013#010+
  'ifeq ($(CPU_TARGET),x86_64)'#013#010+
  'override FPCOPT+=-Cg'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef LINKSHARED'#013#010+
  '#!!! deactived for testing override',' FPCOPT+=-XD'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_libc]'#013#010+
  '# Add GCC lib path if asked'#013#010+
  'ifdef GCCLIBDIR'#013#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#013#010+
  'endif'#013#010+
  'ifdef OTHERLIBDIR'#013#010+
  'override FPCOPT+=$(addprefix -Fl,$(OTHERLIBDIR))'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[command_end]'#013#010+
  '# Add commandline',' options last so they can override'#013#010+
  'ifdef OPT'#013#010+
  'override FPCOPT+=$(OPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Add defines from FPCOPTDEF to FPCOPT'#013#010+
  'ifdef FPCOPTDEF'#013#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOPTDEF))'#013#010+
  'endif'#013#010+
  #013#010+
  '# Was a config file specified ?'#013#010+
  'ifdef CFGFILE',#013#010+
  'override FPCOPT+=@$(CFGFILE)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use the environment to pass commandline options?'#013#010+
  'ifdef USEENV'#013#010+
  'override FPCEXTCMD:=$(FPCOPT)'#013#010+
  'override FPCOPT:=!FPCEXTCMD'#013#010+
  'export FPCEXTCMD'#013#010+
  'endif'#013#010+
  #013#010+
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARGE','T)'#013#010+
  'override AFULL_SOURCE=$(CPU_SOURCE)-$(OS_SOURCE)'#013#010+
  #013#010+
  'ifneq ($(AFULL_TARGET),$(AFULL_SOURCE))'#013#010+
  'override ACROSSCOMPILE=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef ACROSSCOMPILE'#013#010+
  'override FPCOPT+=$(CROSSOPT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Compiler commandline'#013#010+
  'override COMPILER:=$(FPC',') $(FPCOPT)'#013#010+
  #013#010+
  '# also call ppas if with command option -s'#013#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#013#010+
  'ifeq (,$(findstring -s ,$(COMPILER)))'#013#010+
  'EXECPPAS='#013#010+
  'else'#013#010+
  'ifeq ($(FULL_SOURCE),$(FULL_TARGET))'#013#010+
  'ifdef RUNBATCH'#013#010+
  'EXECPPAS:=@$(R','UNBATCH) $(PPAS)'#013#010+
  'else'#013#010+
  'EXECPPAS:=@$(PPAS)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[loaderrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Loaders'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.','PHONY: fpc_loaders'#013#010+
  #013#010+
  'ifneq ($(TARGET_LOADERS),)'#013#010+
  'override ALLTARGET+=fpc_loaders'#013#010+
  'override CLEANTARGET+=fpc_loaders_clean'#013#010+
  'override INSTALLTARGET+=fpc_loaders_install'#013#010+
  #013#010+
  'override LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#013#010+
  'endif'#013,#010+
  #013#010+
  '%$(OEXT): %$(LOADEREXT)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#013#010+
  'else'#013#010+
  '        $(AS) -o $*$(OEXT) $<'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders: $(COMPILER_UNITTARGETDIR) $(LOADEROFILES)'#013#010+
  #013#010+
  'fpc_loaders_clean:'#013#010+
  'i','fdef COMPILER_UNITTARGETDIR'#013#010+
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#013#010+
  'else'#013#010+
  '        -$(DEL) $(LOADEROFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_loaders_install:'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  ' ','       $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROF'+
  'ILES)) $(INSTALL_UNITDIR)'#013#010+
  'else'#013#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[unitrules]'#013#010+
  '#############################################################','#######'+
  '#'#013#010+
  '# Units'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_units'#013#010+
  #013#010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#013#010+
  'override ALLTARGET+=fpc_units'#013#010+
  #013#010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$','(TARGET_UNITS))'#013#010+
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#013#010+
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'override CLEANPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_','units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#013#010+
  #013#010+
  #013#010+
  '[exerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Exes'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_exes'#013#010+
  #013#010+
  '# ','Programs are not needed for a cross installation'#013#010+
  'ifndef CROSSINSTALL'#013#010+
  'ifneq ($(TARGET_PROGRAMS),)'#013#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#013#010+
  'override EXEOFILES:=$(addsuffix $(OEXT),$(TARGET_PROGRAMS)) $(addprefi'+
  'x $(STATI','CLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_'+
  'PROGRAMS)))'#013#010+
  #013#010+
  'override ALLTARGET+=fpc_exes'#013#010+
  'override INSTALLEXEFILES+=$(EXEFILES)'#013#010+
  'override CLEANEXEFILES+=$(E','XEFILES) $(EXEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'f','pc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES'+
  ')'#013#010+
  #013#010+
  #013#010+
  '[rstrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Resource strings'#013#010+
  '###############################################################','#####'+
  '#'#013#010+
  #013#010+
  'ifdef TARGET_RSTS'#013#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_RSTS))'#013#010+
  #013#010+
  'override CLEANRSTFILES+=$(RSTFILES)'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[examplerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Examples'#013#010+
  '#####','###############################################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_examples'#013#010+
  #013#010+
  'ifneq ($(TARGET_EXAMPLES),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'override EXAMPLESOURCEFILES:=$(wildcard $(addsuffix .pp,$(TARGET_EXAMP'+
  'LES)) $(addsuffix .pas,$(TARGET_EX','AMPLES)) $(addsuffix .lpr,$(TARGET'+
  '_EXAMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#013#010+
  'override EXAMPLEFILES:=$(addsuffix $(EXEEXT),$(TARGET_EXAMPLES))'#013#010+
  'override EXAMPLEOFILES:=$(addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(addp'+
  'refix $(STATICLIBPR','EFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPL'+
  'ES))) $(addprefix $(IMPORTLIBPREFIX),$(addsuffix $(STATICLIBEXT),$(TAR'+
  'GET_EXAMPLES)))'#013#010+
  #013#010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES) $(EXAMPLEOFILES)'#013#010+
  'ifeq ($(OS_TARGET),os2)'#013#010+
  'override CLEANEXEFI','LES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'ifeq ($(OS_TARGET),emx)'#013#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  'ifneq ($(TARGET_EXAMPLEDIRS),)'#013#010+
  'HASEXAMPLES=1'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_examples: all $(E','XAMPLEFILES) $(addsuffix _all,$(TARGET_EXAMPLED'+
  'IRS))'#013#010+
  #013#010+
  #013#010+
  '[compilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# General compile rules'#013#010+
  '#####################################################################'#013,
  #010+
  #013#010+
  '.PHONY: fpc_all fpc_smart fpc_debug fpc_release fpc_shared'#013#010+
  #013#010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#013#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMADE)'#013#010+
  #013#010+
  'fpc_all: $(FPCMADE)'#013#010+
  #013#010+
  'fpc_smart:'#013#010+
  '        $(MAKE) all LINKSMART=1 CREATESMART=1'#013#010+
  #013,#010+
  'fpc_debug:'#013#010+
  '        $(MAKE) all DEBUG=1'#013#010+
  #013#010+
  'fpc_release:'#013#010+
  '        $(MAKE) all RELEASE=1'#013#010+
  #013#010+
  '# General compile rules, available for both possible .pp and .pas exte'+
  'nsions'#013#010+
  #013#010+
  '.SUFFIXES: $(EXEEXT) $(PPUEXT) $(OEXT) .pas .lpr .dpr .pp .rc .res'#013+
  #010+
  #013,#010+
  '$(COMPILER_UNITTARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '$(COMPILER_TARGETDIR):'#013#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(PPUEXT): %.pas'#013#010+
  '       ',' $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pp'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.pas'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(EXEEXT): %.lpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%$(','EXEEXT): %.dpr'#013#010+
  '        $(COMPILER) $<'#013#010+
  '        $(EXECPPAS)'#013#010+
  #013#010+
  '%.res: %.rc'#013#010+
  '        windres -i $< -o $@'#013#010+
  #013#010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#013#010+
  'vpath %.pp $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.pas $(COMPILER_SOURC','EDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %.inc $(COMPILER_INCLUDEDIR)'#013#010+
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#013#010+
  'vpath %$(PPUEXT)',' $(COMPILER_UNITTARGETDIR)'#013#010+
  #013#010+
  '[sharedrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Library'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_shared'#013#010+
  #013#010+
  'override INSTALLTA','RGET+=fpc_shared_install'#013#010+
  #013#010+
  'ifndef SHARED_LIBVERSION'#013#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_LIBNAME'#013#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef SHARED_FULLNAME'#013#010+
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARED_LIBNAME)-$(S','HARED_LIBVERS'+
  'ION)$(SHAREDLIBEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  '# Default sharedlib units are all unit objects'#013#010+
  'ifndef SHARED_LIBUNITS'#013#010+
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#013#010+
  'override SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT),$(SHARE','D'+
  '_LIBUNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared:'#013#010+
  'ifdef HASSHAREDLIB'#013#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  '        $(PPUMOVE) -q $(SHARED_LIBUNITS) -i$(COMPILER_UNITTARGETDIR) -'+
  'o$(SHARED_FULLNAME) -d$(','COMPILER_UNITTARGETDIR)'#013#010+
  'endif'#013#010+
  'else'#013#010+
  '        @$(ECHO) Shared Libraries not supported'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_shared_install:'#013#010+
  'ifneq ($(SHARED_BUILD),n)'#013#010+
  'ifneq ($(SHARED_LIBUNITS),)'#013#010+
  'ifneq ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME)),)'#013#010,
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '[installrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Install rules'#013#010+
  '###########################','#########################################'+
  '#'#013#010+
  #013#010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#013#010+
  #013#010+
  'ifdef INSTALL_UNITS'#013#010+
  'override INSTALLPPUFILES+=$(addsuffix $(PPUEXT),$(INSTALL_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALL_BUILDUNIT'#013#010+
  'override INS','TALLPPUFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT)'+
  ',$(INSTALLPPUFILES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPPUFI'+
  'LES)) $(addprefix $(STATICLIBPREFIX),$(subst $(PPUEXT),$(STA','TICLIBEX'+
  'T),$(INSTALLPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEX'+
  'T),$(STATICLIBEXT),$(INSTALLPPUFILES)))'#013#010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#013#010+
  'override INSTALLPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(notdir '+
  '$(INSTALLPPUFILES)','))'#013#010+
  'override INSTALLPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPR'+
  'EFIX),$(notdir $(INSTALLPPULINKFILES))))'#013#010+
  'endif'#013#010+
  '# Implicitly install Package.fpc'#013#010+
  'override INSTALL_CREATEPACKAGEFPC=1'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  'ifneq ($(TARGE','TDIRPREFIX),)'#013#010+
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_install: all $(INSTALLTARGET)'#013#010+
  'ifdef INSTALLEXEFILES'#013#010+
  '        $(MKDIR) $(INSTALL_BINDIR)'#013#010+
  '# Compress the exes if upx ','is defined'#013#010+
  'ifdef UPXPROG'#013#010+
  '        -$(UPXPROG) $(INSTALLEXEFILES)'#013#010+
  'endif'#013#010+
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#013#010+
  'ifdef FPCMAKE'#013#010+
  '# If the fpcpackage variable is set then create and ','install Package.'+
  'fpc,'#013#010+
  '# a safety check is done if Makefile.fpc is available'#013#010+
  'ifdef PACKAGE_VERSION'#013#010+
  'ifneq ($(wildcard Makefile.fpc),)'#013#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS_TARGET) Makefile.fpc'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '   ','     $(INSTALL) Package.fpc $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALLPPUFILES'#013#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#013#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(INSTALL_UNITDIR)'#013#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#013#010+
  '        $(INS','TALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#013#010+
  'endif'#013#010+
  'ifneq ($(wildcard $(LIB_FULLNAME)),)'#013#010+
  '        $(MKDIR) $(INSTALL_LIBDIR)'#013#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LIBDIR)'#013#010+
  'ifdef inUnix'#013#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_L','IBDIR)/$(LIB_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'ifdef INSTALL_FILES'#013#010+
  '        $(MKDIR) $(INSTALL_DATADIR)'#013#010+
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_sourceinstall: distclean'#013#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#013#010+
  '       ',' $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#013#010+
  #013#010+
  'fpc_exampleinstall: $(addsuffix _distclean,$(TARGET_EXAMPLEDIRS))'#013#010+
  'ifdef HASEXAMPLES'#013#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef EXAMPLESOURCEFILES'#013#010+
  '        $(COPY) $(EXAMPLESOURCE','FILES) $(INSTALL_EXAMPLEDIR)'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_EX'+
  'AMPLEDIR)'#013#010+
  'endif'#013#010+
  #013#010+
  '[distinstallrules]'#013#010+
  '###################################################################','#'+
  '#'#013#010+
  '# Dist Install'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_distinstall'#013#010+
  #013#010+
  'fpc_distinstall: install exampleinstall'#013#010+
  #013#010+
  #013#010+
  '[zipinstallrules]'#013#010+
  '########################################################','############'+
  '#'#013#010+
  '# Zip'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#013#010+
  #013#010+
  '# Temporary path to pack a file, can only use a single deep'#013#010+
  '# subdir, becau','se the deltree can'#039't see the whole tree to remove'+
  #013#010+
  'ifndef PACKDIR'#013#010+
  'ifndef inUnix'#013#010+
  'PACKDIR=$(BASEDIR)/../fpc-pack'#013#010+
  'else'#013#010+
  'PACKDIR=/tmp/fpc-pack'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Maybe create default zipname from packagename'#013#010+
  'ifndef ZIPNAME'#013#010+
  'ifdef DIST_ZIPNA','ME'#013#010+
  'ZIPNAME=$(DIST_ZIPNAME)'#013#010+
  'else'#013#010+
  'ZIPNAME=$(PACKAGE_NAME)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  'ifndef FULLZIPNAME'#013#010+
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  '# ZipTarget'#013#010+
  'ifndef ZIPTARGET'#013#010+
  'ifdef DIST_ZIPTARGET'#013#010+
  'ZIPTARGET=DIST_ZI','PTARGET'#013#010+
  'else'#013#010+
  'ZIPTARGET=install'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use tar by default under linux'#013#010+
  'ifndef USEZIP'#013#010+
  'ifdef inUnix'#013#010+
  'USETAR=1'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Use a wrapper script by default for Os/2'#013#010+
  'ifndef inUnix'#013#010+
  'USEZIPWRAPPER=1'#013#010+
  'endif'#013#010+
  #013#010+
  '# We need to be ','able to run in the current OS so fix'#013#010+
  '# the path separator'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  'ZIPPATHSEP=$(PATHSEP)'#013#010+
  'ZIPWRAPPER=$(subst /,$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT))'#013+
  #010+
  'else'#013#010+
  'ZIPPATHSEP=/'#013#010+
  'endif'#013#010+
  #013#010+
  '# Create commands to create the zip/','tar file'#013#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#013#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#013#010+
  'ifdef USETAR'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(TAREXT)'#013#010+
  'ZIPCMD_ZIP:=$(TARPROG) cf$(TAROPT) $(ZIPDESTFILE) *'#013,#010+
  'else'#013#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#013#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
  'TFILE) *'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipinstall:'#013#010+
  '        $(MAKE) $(ZIPTARGET) INSTALL_PREFIX=$(PACKDIR) ZIPINSTALL=1'#013+
  #010+
  '    ','    $(MKDIR) $(DIST_DESTDIR)'#013#010+
  '        $(DEL) $(ZIPDESTFILE)'#013#010+
  'ifdef USEZIPWRAPPER'#013#010+
  '# Handle gecho separate as we need to espace \ with \\'#013#010+
  'ifneq ($(ECHOREDIR),echo)'#013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDPACK))" > $(ZIPWRAPPE'+
  'R)'#013#010+
  ' ','       $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPE'+
  'R)'#013#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDBASE))" >> $(ZIPWRAPP'+
  'ER)'#013#010+
  'else'#013#010+
  '        echo $(ZIPCMD_CDPACK) > $(ZIPWRAPPER)'#013#010+
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPE','R)'#013#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'ifdef inUnix'#013#010+
  '        /bin/sh $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  'ifdef RUNBATCH'#013#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)'#013#010+
  'else'#013#010+
  '        $(ZIPWRAPPER)'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  '        $(DEL) $(ZIPWRAPPER)'#013#010+
  'else'#013#010,
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#013#010+
  'endif'#013#010+
  '        $(DELTREE) $(PACKDIR)'#013#010+
  #013#010+
  'fpc_zipsourceinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=sourceinstall ZIPSUFFIX=$(ZIP'+
  'SOURCESUFFIX)'#013#010+
  #013#010+
  'fpc_zipexampleinstall:'#013#010+
  'ifdef',' HASEXAMPLES'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_zipdistinstall:'#013#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=distinstall'#013#010+
  #013#010+
  #013#010+
  '[cleanrules]'#013#010+
  '#####################################','###############################'+
  '#'#013#010+
  '# Clean rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#013#010+
  #013#010+
  'ifdef EXEFILES'#013#010+
  'override CLEANEXEFILES:=$(addprefix $(TARGETDIRPREFIX)',',$(CLEANEXEFIL'+
  'ES))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEAN_UNITS'#013#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#013#010+
  'endif'#013#010+
  #013#010+
  'ifdef CLEANPPUFILES'#013#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
  ') $(addprefix $(STATICLIBPREF','IX),$(subst $(PPUEXT),$(STATICLIBEXT),$'+
  '(CLEANPPUFILES))) $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(S'+
  'TATICLIBEXT),$(CLEANPPUFILES)))'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#013,#010+
  'endif'#013#010+
  'override CLEANPPUFILES:=$(addprefix $(UNITTARGETDIRPREFIX),$(CLEANPPUF'+
  'ILES))'#013#010+
  'override CLEANPPULINKFILES:=$(wildcard $(addprefix $(UNITTARGETDIRPREF'+
  'IX),$(CLEANPPULINKFILES)))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_clean: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013,#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILES)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGE','TDIRPREFIX),$(CLEANRSTFILES))'+
  #013#010+
  'endif'#013#010+
  'ifdef CLEAN_FILES'#013#010+
  '        -$(DEL) $(CLEAN_FILES)'#013#010+
  'endif'#013#010+
  'ifdef LIB_NAME'#013#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#013#010+
  'endif'#013#010+
  '        -$(DEL) $(FPCMADE) Package.fpc $(PPAS) script.res link.res $(F'+
  'PCEXTF','ILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *$(ASMEXT) *_ppas$(BATCHEXT)'#013#010+
  #013#010+
  'fpc_cleanall: $(CLEANTARGET)'#013#010+
  'ifdef CLEANEXEFILES'#013#010+
  '        -$(DEL) $(CLEANEXEFILES)'#013#010+
  'endif'#013#010+
  'ifdef COMPILER_UNITTARGETDIR'#013#010+
  'ifdef CLEANPPUFILES'#013#010+
  '        -$(DEL) $(CLEANPPUFILE','S)'#013#010+
  'endif'#013#010+
  'ifneq ($(CLEANPPULINKFILES),)'#013#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#013#010+
  'endif'#013#010+
  'ifdef CLEANRSTFILES'#013#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#013+
  #010+
  'endif'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) units'#013#010+
  '        -$(DEL) *$(OE','XT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *$(STATICL'+
  'IBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#013#010+
  'ifneq ($(PPUEXT),.ppu)'#013#010+
  '        -$(DEL) *.o *.ppu *.a'#013#010+
  'endif'#013#010+
  '        -$(DELTREE) *$(SMARTEXT)'#013#010+
  '        -$(DEL) fpcmade.* Package.fpc $(PPAS) script.res link','.res $('+
  'FPCEXTFILE) $(REDIRFILE)'#013#010+
  '        -$(DEL) *_ppas$(BATCHEXT)'#013#010+
  'ifdef AOUTEXT'#013#010+
  '        -$(DEL) *$(AOUTEXT)'#013#010+
  'endif'#013#010+
  'ifdef DEBUGSYMEXT'#013#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_distclean: cleanall'#013#010+
  #013#010+
  #013#010+
  '[baseinforules]'#013#010+
  '##############','######################################################'+
  '#'#013#010+
  '# Base info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_baseinfo'#013#010+
  #013#010+
  'override INFORULES+=fpc_baseinfo'#013#010+
  #013#010+
  'fpc_baseinfo:'#013#010+
  '        @$(ECHO)',#013#010+
  '        @$(ECHO)  == Package info =='#013#010+
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#013#010+
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Configuration info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(','ECHO)  FPC.......... $(FPC)'#013#010+
  '        @$(ECHO)  FPC Version.. $(FPC_VERSION)'#013#010+
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#013#010+
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#013#010+
  '        @$(ECHO)  Source OS.... $(OS_SOURCE)'#013#010+
  '        @$(ECHO)  Targe','t OS.... $(OS_TARGET)'#013#010+
  '        @$(ECHO)  Full Source.. $(FULL_SOURCE)'#013#010+
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#013#010+
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#013#010+
  '        @$(ECHO)  TargetSuffix. $(TARGETSUFFIX)'#013#010+
  '        @$(ECHO)'#013#010+
  '     ','   @$(ECHO)  == Directory info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Basedir......... $(BASEDIR)'#013#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#013#010+
  '        @$(ECHO)  Cr','ossBinDir..... $(CROSSBINDIR)'#013#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#013#010+
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  GCC library..... $(GCCLIBDIR)'#013#010+
  '        @$(ECHO)  Other library... $(OTHERL','IBDIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Tools info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  As........ $(AS)'#013#010+
  '        @$(ECHO)  Ld........ $(LD)'#013#010+
  '        @$(ECHO)  Ar........ $(AR)'#013#010+
  '        @$(ECHO)  Rc........ $(RC)'#013#010+
  '        @$(ECHO)',#013#010+
  '        @$(ECHO)  Mv........ $(MVPROG)'#013#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#013#010+
  '        @$(ECHO)  Rm........ $(RMPROG)'#013#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#013#010+
  '        @$(ECHO)  Echo...... $(ECHO)'#013#010+
  '        @$(ECHO)  Shell..... $(SHELL)'#013,#010+
  '        @$(ECHO)  Date...... $(DATE)'#013#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#013#010+
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#013#010+
  '        @$(ECHO)  Upx....... $(UPXPROG)'#013#010+
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  =','= Object info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Target Loaders........ $(TARGET_LOADERS)'#013#010+
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#013#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#013#010+
  '        @$(ECHO)  Ta','rget Programs....... $(TARGET_PROGRAMS)'#013#010+
  '        @$(ECHO)  Target Dirs........... $(TARGET_DIRS)'#013#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#013#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#013#010+
  '        @$(ECHO)'#013,#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#013#010+
  '        @$(ECHO)  Clean Files......... $(CLEAN_FILES)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#013#010+
  '        @$(ECHO)  Install Files....... $(INSTALL_FILES',')'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  == Install info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  DateStr.............. $(DATESTR)'#013#010+
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#013#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPREFIX)'#013#010+
  '     ','   @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#013#010+
  '        @$(ECHO)  ZipSuffix............ $(ZIPSUFFIX)'#013#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#013#010+
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#013#010+
  '        @$(EC','HO)'#013#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#013#010+
  '        @$(ECHO)  Install binary dir... $(INSTALL_BINDIR)'#013#010+
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#013#010+
  '        @$(ECHO)  Install units dir.... $(INSTALL_UNITDIR',')'#013#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#013#010+
  '        @$(ECHO)  Install doc dir...... $(INSTALL_DOCDIR)'#013#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#013#010+
  '        @$(ECHO)  Install data dir..... $(INSTALL_DAT','ADIR)'#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#013#010+
  '        @$(ECHO)  Dist zip name........ $(DIST_ZIPNAME)'#013#010+
  '        @$(ECHO)'#013#010+
  #013#010+
  '[inforules]'#013#010+
  '####################################################################','#'+
  #013#010+
  '# Info rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_info'#013#010+
  #013#010+
  'fpc_info: $(INFORULES)'#013#010+
  #013#010+
  '[makefilerules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# Rebuild Makefi','le'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1 fpc_makefile_sub2'+
  ' \'#013#010+
  '        fpc_makefile_dirs'#013#010+
  #013#010+
  'fpc_makefile:'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) Makefile','.fpc'#013#010+
  #013#010+
  'fpc_makefile_sub1:'#013#010+
  'ifdef TARGET_DIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_DIRS))'#013#010+
  'endif'#013#010+
  'ifdef TARGET_EXAMPLEDIRS'#013#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_EXAMPLED','IRS))'#013#010+
  'endif'#013#010+
  #013#010+
  'fpc_makefile_sub2: $(addsuffix _makefile_dirs,$(TARGET_DIRS) $(TARGET_'+
  'EXAMPLEDIRS))'#013#010+
  #013#010+
  'fpc_makefile_dirs: fpc_makefile_sub1 fpc_makefile_sub2'#013#010+
  #013#010+
  'fpc_makefiles: fpc_makefile fpc_makefile_dirs'#013#010+
  #013#010+
  '[localmakefile]'#013#010+
  '#############','#######################################################'+
  '#'#013#010+
  '# Local Makefile'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  'ifneq ($(wildcard fpcmake.loc),)'#013#010+
  'include fpcmake.loc'#013#010+
  'endif'#013#010+
  #013#010+
  #013#010+
  '[userrules]'#013#010+
  '############','########################################################'+
  '#'#013#010+
  '# Users rules'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '[lclrules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# LCL Rule','s'#013#010+
  '#####################################################################'#013+
  #010+
  #013#010+
  '# LCL Platform'#013#010+
  'ifndef LCL_PLATFORM'#013#010+
  'ifneq ($(findstring $(OS_TARGET),win32 win64),)'#013#010+
  'LCL_PLATFORM=win32'#013#010+
  'else'#013#010+
  'LCL_PLATFORM=gtk'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  'export LCL_PLATFORM'#013,#010+
  #013#010+
  '# Check if the specified LCLDIR is correct'#013#010+
  'ifdef LCLDIR'#013#010+
  'override LCLDIR:=$(subst \,/,$(LCLDIR))'#013#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'else'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check if the de','fault LCLDIR is correct'#013#010+
  'ifdef DEFAULT_LCLDIR'#013#010+
  'override LCLDIR:=$(subst \,/,$(DEFAULT_LCLDIR))'#013#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check for development version'#013#010+
  'ifeq ($(LCLDIR),wro','ng)'#013#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
  ' $(addsuffix /units/$(LCL_PLATFORM),$(BASEDIR)/lcl $(BASEDIR)))))'#013#010+
  'ifeq ($(LCLDIR),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Check for release version'#013#010+
  'ifeq ($(LCL','DIR),wrong)'#013#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
>>>>>>> origin/fixes_2_2
  ' $(addsuffix /lib/lazarus/units/$(LCL_PLATFORM),/usr/local /usr))))'#013+
  #010+
  'ifeq ($(LCLDIR),)'#013#010+
  'override LCLDIR=wrong'#013#010+
  'endif'#013#010+
  'endif'#013#010+
  #013#010+
  '# Generate dirs'#013#010+
<<<<<<< HEAD
<<<<<<< HEAD
  'ov','erride LCLUNITDIR:=$(wildcard $(LCLDIR)/units/$(LCL_PLATFORM) $(LC'+
=======
  'override LCLU','NITDIR:=$(wildcard $(LCLDIR)/units/$(LCL_PLATFORM) $(LC'+
>>>>>>> graemeg/fixes_2_2
=======
  'override LCLU','NITDIR:=$(wildcard $(LCLDIR)/units/$(LCL_PLATFORM) $(LC'+
>>>>>>> origin/fixes_2_2
  'LDIR)/units)'#013#010+
  'override LCLCOMPONENTDIR:=$(wildcard $(LCLDIR)/.. $(LCLDIR)/../compone'+
  'nts $(LCLDIR)/components)'#013#010+
  'export LCLDIR LCLUNITDIR LCLCOMPONENTDIR'#013#010+
  #013#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '# Add LCL dirs to p','aths'#013#010+
  'override REQUIRE_PACKAGESDIR+=$(LCLCOMPONENTDIR)'#013#010+
=======
  '# Add LCL dirs to paths'#013#010+
  'overr','ide REQUIRE_PACKAGESDIR+=$(LCLCOMPONENTDIR)'#013#010+
>>>>>>> graemeg/fixes_2_2
=======
  '# Add LCL dirs to paths'#013#010+
  'overr','ide REQUIRE_PACKAGESDIR+=$(LCLCOMPONENTDIR)'#013#010+
>>>>>>> origin/fixes_2_2
  'override COMPILER_UNITDIR+=$(LCLUNITDIR)'#013#010+
  #013#010+
  '[lclinforules]'#013#010+
  '#####################################################################'#013+
  #010+
  '# LCL Info rules'#013#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '###################################','#################################'+
=======
  '##############################################','######################'+
>>>>>>> graemeg/fixes_2_2
=======
  '##############################################','######################'+
>>>>>>> origin/fixes_2_2
  '#'#013#010+
  'override INFORULES+=lclinfo'#013#010+
  #013#010+
  '.PHONY: lclinfo'#013#010+
  #013#010+
  'lclinfo:'#013#010+
  '        @$(ECHO)  == LCL info =='#013#010+
  '        @$(ECHO)'#013#010+
  '        @$(ECHO)  Platform............. $(LCL_PLATFORM)'#013#010+
<<<<<<< HEAD
<<<<<<< HEAD
  '        @$(ECHO)  LCLDIR...........','.... $(LCLDIR)'#013#010+
  '        @$(ECHO)  LCL Unit dir......... $(LCLUNITDIR)'#013#010+
  '        @$(ECHO)  LCL Component dir.... $(LCLCOMPONENTDIR)'#013#010+
  '        @$(ECHO)'#013#010
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  '        @$(ECHO)  LCLDIR............... $(LCLD','IR)'#013#010+
  '        @$(ECHO)  LCL Unit dir......... $(LCLUNITDIR)'#013#010+
  '        @$(ECHO)  LCL Component dir.... $(LCLCOMPONENTDIR)'#013#010+
  '        @$(ECHO)'#013#010
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
);
