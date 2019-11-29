{$ifdef Delphi}
const fpcmakeini : array[0..247] of string[240]=(
{$else Delphi}
const fpcmakeini : array[0..247,1..240] of char=(
{$endif Delphi}
  ';'#010+
  '; Templates used by fpcmake to create a Makefile from Makefile.fpc'#010+
  ';'#010+
  #010+
  '[defines]'#010+
  '#####################################################################'#010+
  '# Misc defines to be used by anyone'#010+
  '####################################################','################'+
  '#'#010+
  #010+
  '# OS categories'#010+
  'BSDs = freebsd netbsd openbsd darwin dragonfly'#010+
  'UNIXs = linux $(BSDs) solaris qnx haiku aix'#010+
  'LIMIT83fs = go32v2 os2 emx watcom msdos win16 atari'#010+
  'OSNeedsComspecToRunBatch = go32v2 watcom'#010+
  #010+
  '#Empty target for ru','les that always should run. Needed if'#010+
  '#the target is non-phoney, and there is non-phony prereqisites.'#010+
  '#Then add FORCE as an prerequisite'#010+
  '#See gnu make manual: 4.7 Rules without Commands or Prerequisites'#010+
  'FORCE:'#010+
  '.PHONY: FORCE'#010+
  #010+
  '[osdetect]'#010+
  '####','################################################################'+
  '#'#010+
  '# Autodetect source OS (Linux or Dos or Windows NT or OS/2 or other)'#010+
  '# define inUnix when running under Unix like environment'#010+
  '#               (Linux,FreeBSD,NetBSD,OpenBSD,Da','rwin,Cygwin)'#010+
  '# define inWinNT when running under WinNT'#010+
  '# define inOS2 when running under OS/2'#010+
  '# define inCygwin when running under Cygwin32'#010+
  '#####################################################################'#010+
  #010+
  '# We need only / in the path ','also remove the current dir,'#010+
  '# also remove trailing /'#039's'#010+
  'override PATH:=$(patsubst %/,%,$(subst \,/,$(PATH)))'#010+
  #010+
  '# Detect unix'#010+
  '# Darwin is handled specially'#010+
  'ifneq ($(findstring darwin,$(OSTYPE)),)'#010+
  'inUnix=1 #darwin'#010+
  'SEARCHPATH:=$(filter-out .,$(','subst :, ,$(PATH)))'#010+
  'else'#010+
  '# Determine if we'#039've a unix searchpath by looking for a ;'#010+
  '# that normally doesn'#039't exists in the unix PATH var.'#010+
  'ifeq ($(findstring ;,$(PATH)),)'#010+
  'inUnix=1'#010+
  'SEARCHPATH:=$(filter-out .,$(subst :, ,$(PATH)))'#010+
  'else'#010+
  'SEARCHPAT','H:=$(subst ;, ,$(PATH))'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Add path were make is located'#010+
  'SEARCHPATH+=$(patsubst %/,%,$(subst \,/,$(dir $(MAKE))))'#010+
  #010+
  '# Search for PWD'#010+
  'PWD:=$(strip $(wildcard $(addsuffix /pwd.exe,$(SEARCHPATH))))'#010+
  'ifeq ($(PWD),)'#010+
  'PWD:=$(strip $(wild','card $(addsuffix /pwd,$(SEARCHPATH))))'#010+
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
  '# Detect NT - NT sets OS t','o Windows_NT'#010+
  '# Detect OS/2 - OS/2 has OS2_SHELL defined'#010+
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
  '# The extension of batch f','iles / scripts'#010+
  'ifdef inUnix'#010+
  'SRCBATCHEXT=.sh'#010+
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
  'endi','f'#010+
  'endif'#010+
  #010+
  '# Path Separator, the subst trick is necessary for the \ that can'#039't'+
  ' exists'#010+
  '# at the end of a line'#010+
  'ifdef inUnix'#010+
  'PATHSEP=/'#010+
  'else'#010+
  'PATHSEP:=$(subst /,\,/)'#010+
  '# cygwin bash or sh can not handle backslashs'#010+
  'ifdef inCygWin'#010+
  'PATHSEP=/'#010+
  'endif'#010+
  'endi','f'#010+
  #010+
  '# Base dir'#010+
  'ifdef PWD'#010+
  'BASEDIR:=$(subst \,/,$(shell $(PWD)))'#010+
  '# For Cygwin we need to replace /cygdrive/c/ with c:/'#010+
  'ifdef inCygWin'#010+
  'ifneq ($(findstring /cygdrive/,$(BASEDIR)),)'#010+
  'BASENODIR:=$(patsubst /cygdrive%,%,$(BASEDIR))'#010+
  'BASEDRIVE:=$(firs','tword $(subst /, ,$(BASENODIR)))'#010+
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
  'ECHO:','=$(strip $(wildcard $(addsuffix /gecho$(SRCEXEEXT),$(SEARCHPATH'+
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
  'ECHO:=$(firstwo','rd $(ECHO))'#010+
  'endif'#010+
  'endif'#010+
  'export ECHO'#010+
  'endif'#010+
  #010+
  '[fpcdetect]'#010+
  '#####################################################################'#010+
  '# FPC Binary and Version Detection'#010+
  '#####################################################################'#010+
  #010+
  '# Compati','bility with old makefiles'#010+
  'ifndef FPC'#010+
  'ifdef PP'#010+
  'FPC=$(PP)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Try to detect the ppcXXX file to use by using "fpc -PB" option'#010+
  '# to query for the default ppcXXX the fpc executable tries. When'#010+
  '# fpc is not found use ppc386 by default.',' Also when fpc -PB gives'#010+
  '# an error ppc386 will be used.'#010+
  'ifndef FPC'#010+
  '# check if fpc exists'#010+
  'FPCPROG:=$(strip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#010+
  'ifneq ($(FPCPROG),)'#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#010+
  'ifneq ($(CPU_TARGET),)'#010,
  'FPC:=$(shell $(FPCPROG) -P$(CPU_TARGET) -PB)'#010+
  'else'#010+
  'FPC:=$(shell $(FPCPROG) -PB)'#010+
  'endif'#010+
  '# Older fpc executables didn'#039't support it and return'#010+
  '# Error: Illegal processor... If found then fallback to ppc386'#010+
  'ifneq ($(findstring Error,$(FPC)),)'#010+
  'ove','rride FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRC'+
  'EXEEXT),$(SEARCHPATH)))))'#010+
  'else'#010+
  '# if the cross compiler is not found, fall back to fpc'#010+
  'ifeq ($(strip $(wildcard $(FPC))),)'#010+
  'FPC:=$(firstword $(FPCPROG))'#010+
  'endif'#010+
  'endif'#010+
  'else'#010+
  '# fpc b','inary not found, fallback to ppc386'#010+
  'override FPC=$(firstword $(strip $(wildcard $(addsuffix /ppc386$(SRCEX'+
  'EEXT),$(SEARCHPATH)))))'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Get a clean executable name'#010+
  'override FPC:=$(subst $(SRCEXEEXT),,$(FPC))'#010+
  'override FPC:=$(subst \',',/,$(FPC))$(SRCEXEEXT)'#010+
  #010+
  '# Try to find the binary direct first, otherwise in the path,'#010+
  '# if not found give an error'#010+
  'FOUNDFPC:=$(strip $(wildcard $(FPC)))'#010+
  'ifeq ($(FOUNDFPC),)'#010+
  'FOUNDFPC=$(strip $(wildcard $(addsuffix /$(FPC),$(SEARCHPATH))))'#010+
  'if','eq ($(FOUNDFPC),)'#010+
  '$(error Compiler $(FPC) not found)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# For 1.1 and up we can use a single compiler call to retrieve'#010+
  '# all needed information'#010+
  'ifndef FPC_COMPILERINFO'#010+
  'FPC_COMPILERINFO:=$(shell $(FPC) -iVSPTPSOTO)'#010+
  'endif'#010+
  #010+
  '# FPC ver','sion'#010+
  'ifndef FPC_VERSION'#010+
  'FPC_VERSION:=$(word 1,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  #010+
  'export FPC FPC_VERSION FPC_COMPILERINFO'#010+
  '# CHECKDEPEND should not be exported'#010+
  '# This should limit multiple checks'#010+
  'unexport CHECKDEPEND ALLDEPENDENCIES'#010+
  #010+
  '###############','#####################################################'+
  '#'#010+
  '# FPC Target Detection'#010+
  '#####################################################################'#010+
  #010+
  '# Fall back to default values if needed'#010+
  'ifndef CPU_TARGET'#010+
  'ifdef CPU_TARGET_DEFAULT'#010+
  'CPU_TARG','ET=$(CPU_TARGET_DEFAULT)'#010+
  'endif'#010+
  'endif'#010+
  'ifndef OS_TARGET'#010+
  'ifdef OS_TARGET_DEFAULT'#010+
  'OS_TARGET=$(OS_TARGET_DEFAULT)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Retrieve Target/Source CPU and Target/Source OS'#010+
  'ifndef CPU_SOURCE'#010+
  'CPU_SOURCE:=$(word 2,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  'if','ndef CPU_TARGET'#010+
  'CPU_TARGET:=$(word 3,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  'ifndef OS_SOURCE'#010+
  'OS_SOURCE:=$(word 4,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  'ifndef OS_TARGET'#010+
  'OS_TARGET:=$(word 5,$(FPC_COMPILERINFO))'#010+
  'endif'#010+
  'FULL_TARGET=$(CPU_TARGET)-$(OS_TARGET)'#010+
  'FULL_SOUR','CE=$(CPU_SOURCE)-$(OS_SOURCE)'#010+
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
  'ifeq ($(FULL_TARGET),arm-embedded)'#010+
  'ifeq ($(SUBARC','H),)'#010+
  '# we don'#039't add a default here, people should explicitly add the SU'+
  'BARCH'#010+
  '$(error When compiling for arm-embedded, a sub-architecture (e.g. SUBA'+
  'RCH=armv4t or SUBARCH=armv7m) must be defined)'#010+
  'endif'#010+
  'override FPCOPT+=-Cp$(SUBARCH)'#010+
  'endif'#010+
  #010+
  'if','eq ($(FULL_TARGET),avr-embedded)'#010+
  'ifeq ($(SUBARCH),)'#010+
  '# we don'#039't add a default here, people should explicitly add the SU'+
  'BARCH'#010+
  '$(error When compiling for avr-embedded, a sub-architecture (e.g. SUBA'+
  'RCH=avr25 or SUBARCH=avr35) must be defined)'#010+
  'e','ndif'#010+
  'override FPCOPT+=-Cp$(SUBARCH)'#010+
  'endif'#010+
  #010+
  'ifeq ($(FULL_TARGET),mipsel-embedded)'#010+
  'ifeq ($(SUBARCH),)'#010+
  '# we don'#039't add a default here, people should explicitly add the SU'+
  'BARCH'#010+
  '$(error When compiling for mipsel-embedded, a sub-architecture (e.g.',' '+
  'SUBARCH=pic32mx) must be defined)'#010+
  'endif'#010+
  'override FPCOPT+=-Cp$(SUBARCH)'#010+
  'endif'#010+
  #010+
  '# Full name of the target, including CPU and OS. For OSs limited'#010+
  '# to 8.3 we only use the target OS'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#010+
  'TARGETSUFFIX','=$(OS_TARGET)'#010+
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
  'ifneq ($(FULL_TARGET),$(F','ULL_SOURCE))'#010+
  'CROSSCOMPILE=1'#010+
  'endif'#010+
  #010+
  '# Check if the Makefile supports this target, but not'#010+
  '# when the make target is to rebuild the makefile'#010+
  'ifeq ($(findstring makefile,$(MAKECMDGOALS)),)'#010+
  'ifeq ($(findstring $(FULL_TARGET),$(MAKEFILETARGETS)),',')'#010+
  '$(error The Makefile doesn'#039't support target $(FULL_TARGET), please'+
  ' run fpcmake first)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  #010+
  '# Detect BSD, since BSD uses a slightly different directory hierarchy.'+
  #010+
  'ifneq ($(findstring $(OS_TARGET),$(BSDs)),)'#010+
  'BSDhier=1'#010+
  'endif'#010+
  #010+
  '# Dete','ct Linux, will also use its own directory hierarchy.'#010+
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
  'export OS_TA','RGET OS_SOURCE ARCH CPU_TARGET CPU_SOURCE FULL_TARGET FU'+
  'LL_SOURCE TARGETSUFFIX SOURCESUFFIX CROSSCOMPILE'#010+
  #010+
  '[fpmakefpcdetect]'#010+
  '#####################################################################'#010+
  '# FPC Binary and Version Detection'#010+
  '###########','#########################################################'+
  '#'#010+
  '# If FPCFPMAKE is not set and we'#039're cross-compiling, try to find a'+
  #010+
  '# native compiler.'#010+
  'ifndef FPCFPMAKE'#010+
  #010+
  'ifdef CROSSCOMPILE'#010+
  '# Search for the (native) ppc compiler which is used to do',' the lates'+
  't build'#010+
  '# of the native rtl'#010+
  #010+
  'ifeq ($(strip $(wildcard $(addsuffix /compiler/ppc$(SRCEXEEXT),$(FPCDI'+
  'R)))),)'#010+
  '# If that compiler is not found, use the common, installed native comp'+
  'iler'#010+
  '# Note that in this case FPCFPMAKE is not export','ed, because it could'+
  ' be that'#010+
  '# a more suitable compiler is compiled later on'#010+
  '# check if fpc exists'#010+
  'FPCPROG:=$(strip $(wildcard $(addsuffix /fpc$(SRCEXEEXT),$(SEARCHPATH)'+
  ')))'#010+
  'ifneq ($(FPCPROG),)'#010+
  'FPCPROG:=$(firstword $(FPCPROG))'#010+
  'FPCFPMAKE:=$(s','hell $(FPCPROG) -PB)'#010+
  '# if a native compiler is not found, fall back to fpc'#010+
  'ifeq ($(strip $(wildcard $(FPCFPMAKE))),)'#010+
  'FPCFPMAKE:=$(firstword $(FPCPROG))'#010+
  'endif'#010+
  'else'#010+
  '# fpc binary not found, fallback to ppc386'#010+
  'override FPCFPMAKE=$(firstword $(s','trip $(wildcard $(addsuffix /ppc38'+
  '6$(SRCEXEEXT),$(SEARCHPATH)))))'#010+
  'endif'#010+
  'else'#010+
  '# Use the ppc compiler. Because the path to the compiler is relative, '+
  'do not'#010+
  '# evaluate the variable here, but do that on the fly.'#010+
  'FPCFPMAKE=$(strip $(wildcard $(a','ddsuffix /compiler/ppc$(SRCEXEEXT),$'+
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
  '############################################','########################'+
  '#'#010+
  '# FPCDIR Setting'#010+
  '#####################################################################'#010+
  #010+
  '# Test FPCDIR to look if the RTL dir exists'#010+
  'ifdef FPCDIR'#010+
  'override FPCDIR:=$(subst \,/,$(FPCDIR))'#010+
  'ifeq ($(wildcard $(addprefix ','$(FPCDIR)/,rtl)),)'#010+
  'override FPCDIR=wrong'#010+
  'endif'#010+
  'else'#010+
  'override FPCDIR=wrong'#010+
  'endif'#010+
  #010+
  '# Default FPCDIR from Makefile.fpc'#010+
  'ifdef DEFAULT_FPCDIR'#010+
  'ifeq ($(FPCDIR),wrong)'#010+
  'override FPCDIR:=$(subst \,/,$(DEFAULT_FPCDIR))'#010+
  'ifeq ($(wildcard $(addprefix $(F','PCDIR)/,rtl)),)'#010+
  'override FPCDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '[fpcdirdetect]'#010+
  '# Detect FPCDIR'#010+
  'ifeq ($(FPCDIR),wrong)'#010+
  'ifdef inUnix'#010+
  'override FPCDIR=/usr/local/lib/fpc/$(FPC_VERSION)'#010+
  'ifeq ($(wildcard $(FPCDIR)/units),)'#010+
  'override FPCDIR=/usr/lib/fpc/$','(FPC_VERSION)'#010+
  'endif'#010+
  'else'#010+
  'override FPCDIR:=$(subst /$(FPC),,$(firstword $(strip $(wildcard $(add'+
  'suffix /$(FPC),$(SEARCHPATH))))))'#010+
  'override FPCDIR:=$(FPCDIR)/..'#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl)),)'#010+
  'override FPCDIR:=$(FPCDIR)/..'#010+
  'ife','q ($(wildcard $(addprefix $(FPCDIR)/,rtl)),)'#010+
  'override FPCDIR:=$(BASEDIR)'#010+
  'ifeq ($(wildcard $(addprefix $(FPCDIR)/,rtl)),)'#010+
  'override FPCDIR=c:/pp'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Cross binaries dir'#010+
  'ifndef CROSSBINDIR'#010+
  'CROSSBINDIR:=$(wildcard $(','FPCDIR)/bin/$(TARGETSUFFIX))'#010+
  'endif'#010+
  #010+
  '# Default binutils prefix for cross compile when the'#010+
  '# crossbindir is not set (except for Darwin)'#010+
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#010+
  'ifeq ($(OS_SOURCE),darwin)'#010+
  'DARWIN2DARWIN=1'#010+
  'endif'#010+
  'endi','f'#010+
  #010+
  'ifndef BINUTILSPREFIX'#010+
  'ifndef CROSSBINDIR'#010+
  'ifdef CROSSCOMPILE'#010+
  'ifneq ($(OS_TARGET),msdos)'#010+
  'ifndef DARWIN2DARWIN'#010+
  'ifneq ($(CPU_TARGET),jvm)'#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-$(OS_TARGET)-'#010+
  #010+
  '# For Android use standard NDK prefixes'#010+
  'ifeq ($(OS_TARGET),','android)'#010+
  'ifeq ($(CPU_TARGET),arm)'#010+
  'BINUTILSPREFIX=arm-linux-androideabi-'#010+
  'else'#010+
  'ifeq ($(CPU_TARGET),i386)'#010+
  'BINUTILSPREFIX=i686-linux-android-'#010+
  'else'#010+
  'BINUTILSPREFIX=$(CPU_TARGET)-linux-android-'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'endif'#010+
  'endif'#010+
  'else'#010+
  'BINUTILSPREFIX=$(','OS_TARGET)-'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Try first the full target name, otherwise try only'#010+
  '# the OS for backwards compatibility'#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(TARGETSUFFIX))'#010+
  'ifeq ($(UNITSDIR),)'#010+
  'UNITSDIR:=$(wildcard $(FPCDIR)/units/$(','OS_TARGET))'#010+
  'endif'#010+
  #010+
  '# Packages dir'#010+
  'PACKAGESDIR:=$(wildcard $(FPCDIR) $(FPCDIR)/packages)'#010+
  #010+
  #010+
  '[shelltools]'#010+
  '#####################################################################'#010+
  '# Shell tools'#010+
  '#####################################################','###############'+
  '#'#010+
  #010+
  '# Echo that can redir (must be able run in the default OS shell)'#010+
  'ifndef ECHOREDIR'#010+
  'ifndef inUnix'#010+
  'ECHOREDIR=echo'#010+
  'else'#010+
  'ECHOREDIR=$(ECHO)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# To copy pograms'#010+
  'ifndef COPY'#010+
  'COPY:=$(CPPROG) -fp'#010+
  'endif'#010+
  #010+
  '# Copy a whole tre','e'#010+
  'ifndef COPYTREE'#010+
  'COPYTREE:=$(CPPROG) -Rfp'#010+
  'endif'#010+
  #010+
  '# Copy a whole tree'#010+
  'ifndef MKDIRTREE'#010+
  'MKDIRTREE:=$(MKDIRPROG) -p'#010+
  'endif'#010+
  #010+
  '# To move pograms'#010+
  'ifndef MOVE'#010+
  'MOVE:=$(MVPROG) -f'#010+
  'endif'#010+
  #010+
  '# Check delete program'#010+
  'ifndef DEL'#010+
  'DEL:=$(RMPROG) -f'#010+
  'endif'#010+
  #010+
  '# Ch','eck deltree program'#010+
  'ifndef DELTREE'#010+
  'DELTREE:=$(RMPROG) -rf'#010+
  'endif'#010+
  #010+
  '# To install files'#010+
  'ifndef INSTALL'#010+
  'ifdef inUnix'#010+
  'INSTALL:=$(GINSTALL) -c -m 644'#010+
  'else'#010+
  'INSTALL:=$(COPY)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# To install programs'#010+
  'ifndef INSTALLEXE'#010+
  'ifdef inUnix'#010+
  'INSTALLEX','E:=$(GINSTALL) -c -m 755'#010+
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
  'export ECHOREDIR COPY COPYTREE MOVE DEL DELTREE INSTALL INSTALLEXE MKD'+
  'IR'#010+
  #010+
  #010+
  '[defaultdirs]'#010+
  '################','####################################################'+
  '#'#010+
  '# Default Directories'#010+
  '#####################################################################'#010+
  #010+
  '# Units dir'#010+
  'ifdef REQUIRE_UNITSDIR'#010+
  'override UNITSDIR+=$(REQUIRE_UNITSDIR)'#010+
  'endif'#010+
  #010+
  '# Units dir'#010,
  'ifdef REQUIRE_PACKAGESDIR'#010+
  'override PACKAGESDIR+=$(REQUIRE_PACKAGESDIR)'#010+
  'endif'#010+
  #010+
  #010+
  '# Unixes use unix dirs with /usr/bin, /usr/lib'#010+
  '# When zipping use the target os default, when normal install then'#010+
  '# use the source os as default'#010+
  'ifdef ZIPINSTALL',#010+
  '# Zipinstall'#010+
  'ifneq ($(findstring $(OS_TARGET),$(UNIXs)),)'#010+
  'UNIXHier=1'#010+
  'endif'#010+
  'else'#010+
  '# Normal install'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(UNIXs)),)'#010+
  'UNIXHier=1'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# When install prefix is not set try to use prefix'#010+
  'ifndef INSTALL_PREFIX'#010,
  'ifdef PREFIX'#010+
  'INSTALL_PREFIX=$(PREFIX)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# set the prefix directory where to install everything'#010+
  'ifndef INSTALL_PREFIX'#010+
  'ifdef UNIXHier'#010+
  'INSTALL_PREFIX=/usr/local'#010+
  'else'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_BASEDIR:=/pp'#010+
  'else'#010+
  'INSTALL_BASEDIR',':=/$(PACKAGE_NAME)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'export INSTALL_PREFIX'#010+
  #010+
  '# Export also INSTALL_SOURCESUBDIR set so it will be'#010+
  '# used recursively for all subdirs'#010+
  'ifdef INSTALL_FPCSUBDIR'#010+
  'export INSTALL_FPCSUBDIR'#010+
  'endif'#010+
  #010+
  '# Where to place the resulting zip f','iles'#010+
  'ifndef DIST_DESTDIR'#010+
  'DIST_DESTDIR:=$(BASEDIR)'#010+
  'endif'#010+
  'export DIST_DESTDIR'#010+
  #010+
  '# EXE/PPU Target directories'#010+
  'ifndef COMPILER_UNITTARGETDIR'#010+
  'ifdef PACKAGEDIR_MAIN'#010+
  'COMPILER_UNITTARGETDIR=$(PACKAGEDIR_MAIN)/units/$(TARGETSUFFIX)'#010+
  'else'#010+
  'COMPILER_UNIT','TARGETDIR=units/$(TARGETSUFFIX)'#010+
  'endif'#010+
  'endif'#010+
  'ifndef COMPILER_TARGETDIR'#010+
  'COMPILER_TARGETDIR=.'#010+
  'endif'#010+
  #010+
  #010+
  '#####################################################################'#010+
  '# Install Directories'#010+
  '#################################################','###################'+
  '#'#010+
  #010+
  '# set the base directory where to install everything'#010+
  'ifndef INSTALL_BASEDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/fpc/$(FPC_VERSION)'#010+
  'else'#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)/lib/','$(PACKAGE_NAME)'#010+
  'endif'#010+
  'else'#010+
  'INSTALL_BASEDIR:=$(INSTALL_PREFIX)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# set the directory where to install the binaries'#010+
  'ifndef INSTALL_BINDIR'#010+
  'ifdef UNIXHier'#010+
  'INSTALL_BINDIR:=$(INSTALL_PREFIX)/bin'#010+
  'else'#010+
  'INSTALL_BINDIR:=$(INSTALL_BASEDIR)/','bin'#010+
  '# for FPC packages install the binaries under their target subdir'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'ifdef CROSSCOMPILE'#010+
  'ifdef CROSSINSTALL'#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(SOURCESUFFIX)'#010+
  'else'#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#010+
  'en','dif'#010+
  'else'#010+
  'INSTALL_BINDIR:=$(INSTALL_BINDIR)/$(TARGETSUFFIX)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# set the directory where to install the units.'#010+
  'ifndef INSTALL_UNITDIR'#010+
  'INSTALL_UNITDIR:=$(INSTALL_BASEDIR)/units/$(TARGETSUFFIX)'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'if','def PACKAGE_NAME'#010+
  'INSTALL_UNITDIR:=$(INSTALL_UNITDIR)/$(PACKAGE_NAME)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where to install shared libraries'#010+
  'ifndef INSTALL_LIBDIR'#010+
  'ifdef UNIXHier'#010+
  'INSTALL_LIBDIR:=$(INSTALL_PREFIX)/lib'#010+
  'else'#010+
  'INSTALL_LIBDIR:=$(INSTALL_UNITDIR)'#010+
  'e','ndif'#010+
  'endif'#010+
  #010+
  '# Where the source files will be stored'#010+
  'ifndef INSTALL_SOURCEDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef BSDhier'#010+
  'SRCPREFIXDIR=share/src'#010+
  'else'#010+
  'ifdef linuxHier'#010+
  'SRCPREFIXDIR=share/src'#010+
  'else'#010+
  'SRCPREFIXDIR=src'#010+
  'endif'#010+
  'endif'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'ifdef IN','STALL_FPCSUBDIR'#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(INSTALL_FPCSUBDIR)/$(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#010+
  'endif'#010+
  'else'#010+
  'INSTALL_SOU','RCEDIR:=$(INSTALL_PREFIX)/$(SRCPREFIXDIR)/$(PACKAGE_NAME)'+
  '-$(PACKAGE_VERSION)'#010+
  'endif'#010+
  'else'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'ifdef INSTALL_FPCSUBDIR'#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source/$(INSTALL_FPCSUBDIR)/$(PA'+
  'CKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_SOURCEDI','R:=$(INSTALL_BASEDIR)/source/$(PACKAGE_NAME)'#010+
  'endif'#010+
  'else'#010+
  'INSTALL_SOURCEDIR:=$(INSTALL_BASEDIR)/source'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where the doc files will be stored'#010+
  'ifndef INSTALL_DOCDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef BSDhier'#010+
  'DOCPREFIXDIR=share/doc'#010+
  'else'#010+
  'ifde','f linuxHier'#010+
  'DOCPREFIXDIR=share/doc'#010+
  'else'#010+
  'DOCPREFIXDIR=doc'#010+
  'endif'#010+
  'endif'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/fpc-$(FPC_VERSION)/$'+
  '(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_DOCDIR:=$(INSTALL_PREFIX)/$(DOCPREFIXDIR)/$(PACK','AGE_NAME)-$('+
  'PACKAGE_VERSION)'#010+
  'endif'#010+
  'else'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc/$(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_DOCDIR:=$(INSTALL_BASEDIR)/doc'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where to install the examples, under linux we use the d','oc dir'#010+
  '# because the copytree command will create a subdir itself'#010+
  'ifndef INSTALL_EXAMPLEDIR'#010+
  'ifdef UNIXHier'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  #010+
  'ifdef BSDhier'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/fpc-$(FPC_VERSION'+
  ')/$(PACKAGE_NAME)'#010+
  'else'#010,
  'ifdef linuxHier'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples'#010+
  'else'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/fpc-$(FPC_VERSION)/examples/'+
  '$(PACKAGE_NAME)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'else'#010+
  #010+
  'ifdef BSDhier'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/share/examples/','$(PACKAGE_NAME)'+
  '-$(PACKAGE_VERSION)'#010+
  'else'#010+
  'ifdef linuxHier'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_DOCDIR)/examples/$(PACKAGE_NAME)-$(PACKA'+
  'GE_VERSION)'#010+
  'else'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_PREFIX)/doc/$(PACKAGE_NAME)-$(PACKAGE_VE'+
  'RSION)'#010+
  'endif'#010+
  #010+
  'endif'#010+
  'endif'#010+
  'e','lse'#010+
  'ifdef INSTALL_FPCPACKAGE'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples/$(PACKAGE_NAME)'#010+
  'else'#010+
  'INSTALL_EXAMPLEDIR:=$(INSTALL_BASEDIR)/examples'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Where the some extra (data)files will be stored'#010+
  'ifndef INSTALL_DATADIR'#010+
  'IN','STALL_DATADIR=$(INSTALL_BASEDIR)'#010+
  'endif'#010+
  #010+
  'ifndef INSTALL_SHAREDDIR'#010+
  'INSTALL_SHAREDDIR=$(INSTALL_PREFIX)/lib'#010+
  'endif'#010+
  #010+
  '#####################################################################'#010+
  '# Cross compile dirs'#010+
  '#####################################','###############################'+
  '#'#010+
  #010+
  'ifdef CROSSCOMPILE'#010+
  '# Directory where the cross compile tools are stored.'#010+
  '# First check if they are available in FPCDIR. If no targets/ subdir'#010+
  '# is found use the targets/ subdir in INSTALL_BASEDIR.'#010+
  'ifndef CR','OSSBINDIR'#010+
  'CROSSBINDIR:=$(wildcard $(CROSSTARGETDIR)/bin/$(SOURCESUFFIX))'#010+
  'ifeq ($(CROSSBINDIR),)'#010+
  'CROSSBINDIR:=$(wildcard $(INSTALL_BASEDIR)/cross/$(TARGETSUFFIX)/bin/$'+
  '(FULL_SOURCE))'#010+
  'endif'#010+
  'endif'#010+
  'else'#010+
  'CROSSBINDIR='#010+
  'endif'#010+
  #010+
  #010+
  '[dirlibc]'#010+
  '# On linux,',' try to find where libgcc.a is.'#010+
  'ifeq ($(OS_SOURCE),linux)'#010+
  #010+
  '# Amd64 to i386?'#010+
  'ifndef GCCLIBDIR'#010+
  'ifeq ($(CPU_TARGET),i386)'#010+
  'ifneq ($(findstring x86_64,$(shell uname -a)),)'#010+
  'ifeq ($(BINUTILSPREFIX),)'#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m32 -print-lib','gcc-file-name`)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'ifeq ($(CPU_TARGET),powerpc64)'#010+
  'ifeq ($(BINUTILSPREFIX),)'#010+
  'GCCLIBDIR:=$(shell dirname `gcc -m64 -print-libgcc-file-name`)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Try cross gcc'#010+
  'ifndef GCCLIBDIR'#010+
  'CROSSGCC=$(strip $(wildcard $(adds','uffix /$(BINUTILSPREFIX)gcc$(SRCEX'+
  'EEXT),$(SEARCHPATH))))'#010+
  'ifneq ($(CROSSGCC),)'#010+
  'GCCLIBDIR:=$(shell dirname `$(CROSSGCC) -print-libgcc-file-name`)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Other libraries from ld.so.conf'#010+
  'ifndef OTHERLIBDIR'#010+
  'OTHERLIBDIR:=$(shell grep -v "','^\#" /etc/ld.so.conf | awk '#039'{ ORS='+
  '" "; print $1 }'#039')'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'ifdef inUnix'#010+
  'ifeq ($(OS_SOURCE),netbsd)'#010+
  'OTHERLIBDIR+=/usr/pkg/lib'#010+
  'endif'#010+
  'export GCCLIBDIR OTHERLIB'#010+
  'endif'#010+
  #010+
  #010+
  '[extensions]'#010+
  '########################################################','############'+
  '#'#010+
  '# Default extensions'#010+
  '#####################################################################'#010+
  #010+
  '# Default needed extensions (Go32v2,Linux)'#010+
  'BATCHEXT=.bat'#010+
  'LOADEREXT=.as'#010+
  'EXEEXT=.exe'#010+
  'PPLEXT=.ppl'#010+
  'PPUEXT=.ppu'#010+
  'OEXT=.o'#010+
  'LTOEXT=.bc'#010+
  'ASMEXT=.','s'#010+
  'SMARTEXT=.sl'#010+
  'STATICLIBEXT=.a'#010+
  'SHAREDLIBEXT=.so'#010+
  'SHAREDLIBPREFIX=libfp'#010+
  'STATICLIBPREFIX=libp'#010+
  'IMPORTLIBPREFIX=libimp'#010+
  'RSTEXT=.rst'#010+
  '# external debug info for executable'#010+
  'EXEDBGEXT=.dbg'#010+
  '#DEBUGSYMEXT #for debugger symbol files, define only for targe','ts whi'+
  'ch has this'#010+
  #010+
  '# Go32v1'#010+
  'ifeq ($(OS_TARGET),go32v1)'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=v1'#010+
  'endif'#010+
  #010+
  '# Go32v2'#010+
  'ifeq ($(OS_TARGET),go32v2)'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=dos'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# watcom'#010+
  'ifeq ($(OS_TARGET),watcom)'#010+
  'STATICLIBPREFIX='#010,
  'OEXT=.obj'#010+
  'ASMEXT=.asm'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=wat'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# Android native'#010+
  'ifneq ($(CPU_TARGET),jvm)'#010+
  'ifeq ($(OS_TARGET),android)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=lnx'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Linux'#010+
  'ifeq ($(OS_TARGE','T),linux)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=lnx'#010+
  'endif'#010+
  #010+
  '# DragonFly BSD'#010+
  'ifeq ($(OS_TARGET),dragonfly)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=df'#010+
  'endif'#010+
  #010+
  '# FreeBSD'#010+
  'ifeq ($(OS_TARGET),freebsd)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHARE','DLIB=1'#010+
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
  'ifeq ($(OS_TARGET),openbsd)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=obs'#010+
  'endif'#010+
  #010+
  '# Win32'#010+
  'ifeq ($(OS','_TARGET),win32)'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=w32'#010+
  'endif'#010+
  #010+
  '# OS/2'#010+
  'ifeq ($(OS_TARGET),os2)'#010+
  'BATCHEXT=.cmd'#010+
  'AOUTEXT=.out'#010+
  'STATICLIBPREFIX='#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=os2'#010+
  'ECHO=echo'#010+
  'IMPORTLIBPREFIX='#010+
  'endif'#010+
  #010+
  '# EMX'#010+
  'ifeq ($(OS_TARGET),emx)'#010+
  'BATCHEXT=.','cmd'#010+
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
  'SHAREDLIBEXT=.library'#010+
  'SHORTSUFFIX=amg'#010+
  'endif'#010+
  #010+
  '# AROS'#010+
  'ifeq ($(OS_TARGET),aros)'#010+
  'EXEEXT='#010+
  'SHAREDLIBEX','T=.library'#010+
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
  'ifeq ($(OS_TARGET),atari)'#010+
  'EXEEXT=.ttp'#010+
  'SHORTSUFFIX=ata'#010+
  'endif'#010+
  #010+
  '# BeOS'#010+
  'ifeq ($(OS_TARGET),beos)'#010+
  'BATCHEXT=.sh'#010,
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
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'SHORTSUFFIX=sun'#010+
  'endif'#010+
  #010+
  '# QNX'#010+
  'ifeq ($(OS_TARGET),qnx)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT=',#010+
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
  '# Netware libc'#010+
  'ifeq ($(OS_TARGET),netwlibc)'#010+
  'EXEEXT=.nlm'#010+
  'STATICLIBPREFIX='#010+
  'SHORTSUFFIX=nwl'#010+
  'IMPORTLIBPREF','IX=imp'#010+
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
  'ifneq ($(findstring $(OS_TARGET),darwin iphonesim),)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'HASSHAREDLIB=1'#010+
  'SHORTSUFFIX=dw','n'#010+
  'EXEDBGEXT=.dSYM'#010+
  'endif'#010+
  #010+
  '# gba'#010+
  'ifeq ($(OS_TARGET),gba)'#010+
  'EXEEXT=.gba'#010+
  'SHAREDLIBEXT=.so'#010+
  'SHORTSUFFIX=gba'#010+
  'endif'#010+
  #010+
  '# Symbian OS'#010+
  'ifeq ($(OS_TARGET),symbian)'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=symbian'#010+
  'endif'#010+
  #010+
  '# NativeNT'#010+
  'ifeq ($(OS_TARGET),NativeNT)'#010+
  'SHAREDL','IBEXT=.dll'#010+
  'SHORTSUFFIX=nativent'#010+
  'endif'#010+
  #010+
  '# wii'#010+
  'ifeq ($(OS_TARGET),wii)'#010+
  'EXEEXT=.dol'#010+
  'SHAREDLIBEXT=.so'#010+
  'SHORTSUFFIX=wii'#010+
  'endif'#010+
  #010+
  '# AIX'#010+
  'ifeq ($(OS_TARGET),aix)'#010+
  'BATCHEXT=.sh'#010+
  'EXEEXT='#010+
  'SHAREDLIBEXT=.a'#010+
  'SHORTSUFFIX=aix'#010+
  'endif'#010+
  #010+
  '# Java'#010+
  'ifeq ($(OS_TARGET),jav','a)'#010+
  'OEXT=.class'#010+
  'ASMEXT=.j'#010+
  'SHAREDLIBEXT=.jar'#010+
  'SHORTSUFFIX=java'#010+
  'endif'#010+
  #010+
  '# Android java'#010+
  'ifeq ($(CPU_TARGET),jvm)'#010+
  'ifeq ($(OS_TARGET),android)'#010+
  'OEXT=.class'#010+
  'ASMEXT=.j'#010+
  'SHAREDLIBEXT=.jar'#010+
  'SHORTSUFFIX=android'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# MS-DOS'#010+
  'ifeq ($(OS_TARGET),msdo','s)'#010+
  'STATICLIBPREFIX='#010+
  'STATICLIBEXT=.a'#010+
  'SHORTSUFFIX=d16'#010+
  'endif'#010+
  #010+
  '# Embedded'#010+
  'ifeq ($(OS_TARGET),embedded)'#010+
  'ifeq ($(CPU_TARGET),i8086)'#010+
  'STATICLIBPREFIX='#010+
  'STATICLIBEXT=.a'#010+
  'else'#010+
  'EXEEXT=.bin'#010+
  'endif'#010+
  'SHORTSUFFIX=emb'#010+
  'endif'#010+
  #010+
  '# Win16'#010+
  'ifeq ($(OS_TARGET),win16)'#010+
  'S','TATICLIBPREFIX='#010+
  'STATICLIBEXT=.a'#010+
  'SHAREDLIBEXT=.dll'#010+
  'SHORTSUFFIX=w16'#010+
  'endif'#010+
  #010+
  #010+
  '# For 8.3 limited OS'#039's the short suffixes'#010+
  '# Otherwise use the full source/target names'#010+
  'ifneq ($(findstring $(OS_SOURCE),$(LIMIT83fs)),)'#010+
  'FPCMADE=fpcmade.$(SHORTSUFFIX)',#010+
  'ZIPSUFFIX=$(SHORTSUFFIX)'#010+
  'ZIPCROSSPREFIX='#010+
  'ZIPSOURCESUFFIX=src'#010+
  'ZIPEXAMPLESUFFIX=exm'#010+
  'else'#010+
  'FPCMADE=fpcmade.$(TARGETSUFFIX)'#010+
  'ZIPSOURCESUFFIX=.source'#010+
  'ZIPEXAMPLESUFFIX=.examples'#010+
  'ifdef CROSSCOMPILE'#010+
  'ZIPSUFFIX=.$(SOURCESUFFIX)'#010+
  'ZIPCROSSPREFIX=$(TARGET','SUFFIX)-'#010+
  'else'#010+
  'ZIPSUFFIX=.$(TARGETSUFFIX)'#010+
  'ZIPCROSSPREFIX='#010+
  'endif'#010+
  'endif'#010+
  #010+
  '[defaulttools]'#010+
  '#####################################################################'#010+
  '# Default Tools'#010+
  '#####################################################################',
  #010+
  #010+
  '# Names of the binutils tools'#010+
  'ASNAME=$(BINUTILSPREFIX)as'#010+
  'LDNAME=$(BINUTILSPREFIX)ld'#010+
  'ARNAME=$(BINUTILSPREFIX)ar'#010+
  'RCNAME=$(BINUTILSPREFIX)rc'#010+
  'NASMNAME=$(BINUTILSPREFIX)nasm'#010+
  #010+
  '# assembler, redefine it if cross compiling'#010+
  'ifndef ASPROG'#010+
  'ifdef CROS','SBINDIR'#010+
  'ASPROG=$(CROSSBINDIR)/$(ASNAME)$(SRCEXEEXT)'#010+
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
  '# Resource',' compiler'#010+
  'ifndef RCPROG'#010+
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
  'endi','f'#010+
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
  '# Override defaults'#010+
  'AS=$(ASPROG)'#010+
  'LD=$(LDPROG)'#010+
  'RC=$(RCPROG)'#010+
  'AR=$(ARP','ROG)'#010+
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
  'DATESTR:=$(shell $(DATE) ','+%Y%m%d)'#010+
  'else'#010+
  'DATESTR='#010+
  'endif'#010+
  #010+
  '# Zip options'#010+
  'ZIPOPT=-9'#010+
  'ZIPEXT=.zip'#010+
  #010+
  '# Tar options'#010+
  'ifeq ($(USETAR),bz2)'#010+
  'TAROPT=vj'#010+
  'TAREXT=.tar.bz2'#010+
  'else'#010+
  'TAROPT=vz'#010+
  'TAREXT=.tar.gz'#010+
  'endif'#010+
  #010+
  #010+
  '[command_begin]'#010+
  '##########################################################','##########'+
  '#'#010+
  '# Compiler Command Line'#010+
  '#####################################################################'#010+
  #010+
  '# Load commandline OPTDEF and add FPC_CPU define, for compiling the'#010+
  '# compiler this needs to be turned off'#010+
  'ifndef NOCPUDEF'#010+
  'override F','PCOPTDEF=$(ARCH)'#010+
  'endif'#010+
  #010+
  #010+
  '# Load commandline OPT and add target and unit dir to be sure'#010+
  'ifneq ($(OS_TARGET),$(OS_SOURCE))'#010+
  'override FPCOPT+=-T$(OS_TARGET)'#010+
  'endif'#010+
  #010+
  'ifneq ($(CPU_TARGET),$(CPU_SOURCE))'#010+
  'override FPCOPT+=-P$(ARCH)'#010+
  'endif'#010+
  #010+
  'ifeq ($(OS','_SOURCE),openbsd)'#010+
  'override FPCOPT+=-FD$(NEW_BINUTILS_PATH)'#010+
  'override FPCMAKEOPT+=-FD$(NEW_BINUTILS_PATH)'#010+
  'override FPMAKE_BUILD_OPT+=-FD$(NEW_BINUTILS_PATH)'#010+
  'endif'#010+
  #010+
  'ifndef CROSSBOOTSTRAP'#010+
  'ifneq ($(BINUTILSPREFIX),)'#010+
  'override FPCOPT+=-XP$(BINUTIL','SPREFIX)'#010+
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
  '# with B','INUTILSPREFIX=i386-)'#010+
  'ifndef CROSSCOMPILE'#010+
  'ifneq ($(BINUTILSPREFIX),)'#010+
  'override FPCMAKEOPT+=-XP$(BINUTILSPREFIX)'#010+
  'override FPMAKE_BUILD_OPT+=-XP$(BINUTILSPREFIX)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# User dirs should be first, so they are looked at first'#010+
  'ifdef UNITDI','R'#010+
  'override FPCOPT+=$(addprefix -Fu,$(UNITDIR))'#010+
  'endif'#010+
  'ifdef LIBDIR'#010+
  'override FPCOPT+=$(addprefix -Fl,$(LIBDIR))'#010+
  'endif'#010+
  'ifdef OBJDIR'#010+
  'override FPCOPT+=$(addprefix -Fo,$(OBJDIR))'#010+
  'endif'#010+
  'ifdef INCDIR'#010+
  'override FPCOPT+=$(addprefix -Fi,$(INCDIR))'#010+
  'endi','f'#010+
  #010+
  '# Smartlinking'#010+
  'ifdef LINKSMART'#010+
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
  '# (strip, optimize ','and don'#039't load fpc.cfg)'#010+
  '# fpc 2.1 has -O2 for all targets'#010+
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
  'endi','f'#010+
  'override FPCOPT+=-Ur -Xs $(FPCCPUOPT) -n'#010+
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
  'overri','de FPCOPT+=-vwni'#010+
  'endif'#010+
  #010+
  '# Needed compiler options'#010+
  'ifdef COMPILER_OPTIONS'#010+
  'override FPCOPT+=$(COMPILER_OPTIONS)'#010+
  'endif'#010+
  'ifdef COMPILER_UNITDIR'#010+
  'override FPCOPT+=$(addprefix -Fu,$(COMPILER_UNITDIR))'#010+
  'endif'#010+
  'ifdef COMPILER_LIBRARYDIR'#010+
  'override FPCOPT','+=$(addprefix -Fl,$(COMPILER_LIBRARYDIR))'#010+
  'endif'#010+
  'ifdef COMPILER_OBJECTDIR'#010+
  'override FPCOPT+=$(addprefix -Fo,$(COMPILER_OBJECTDIR))'#010+
  'endif'#010+
  'ifdef COMPILER_INCLUDEDIR'#010+
  'override FPCOPT+=$(addprefix -Fi,$(COMPILER_INCLUDEDIR))'#010+
  'endif'#010+
  #010+
  '# Cross compile','r utils'#010+
  'ifdef CROSSBINDIR'#010+
  'override FPCOPT+=-FD$(CROSSBINDIR)'#010+
  'endif'#010+
  #010+
  '# Target dirs and the prefix to use for clean/install'#010+
  'ifdef COMPILER_TARGETDIR'#010+
  'override FPCOPT+=-FE$(COMPILER_TARGETDIR)'#010+
  'ifeq ($(COMPILER_TARGETDIR),.)'#010+
  'override TARGETDIRPR','EFIX='#010+
  'else'#010+
  'override TARGETDIRPREFIX=$(COMPILER_TARGETDIR)/'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Unit target dir (default is units/<cpu>-<os>/'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
  'override FPCOPT+=-FU$(COMPILER_UNITTARGETDIR)'#010+
  'ifeq ($(COMPILER_UNITTARGETDIR),.)'#010+
  'override U','NITTARGETDIRPREFIX='#010+
  'else'#010+
  'override UNITTARGETDIRPREFIX=$(COMPILER_UNITTARGETDIR)/'#010+
  'endif'#010+
  'else'#010+
  'ifdef COMPILER_TARGETDIR'#010+
  'override COMPILER_UNITTARGETDIR=$(COMPILER_TARGETDIR)'#010+
  'override UNITTARGETDIRPREFIX=$(TARGETDIRPREFIX)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'ifdef CR','EATESHARED'#010+
  'override FPCOPT+=-Cg'#010+
  'endif'#010+
  #010+
  '# create always pic'#039'ed code on x86_64, mips and mipsel'#010+
  '# on unix-like systems'#010+
  'ifneq ($(findstring $(OS_TARGET),dragonfly freebsd openbsd netbsd linu'+
  'x solaris),)'#010+
  'ifneq ($(findstring $(CPU_TARGET),x86_64',' mips mipsel),)'#010+
  'override FPCOPT+=-Cg'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'ifdef LINKSHARED'#010+
  '#!!! deactived for testing override FPCOPT+=-XD'#010+
  'endif'#010+
  #010+
  #010+
  '[command_libc]'#010+
  '# Add GCC lib path if asked'#010+
  'ifdef GCCLIBDIR'#010+
  'override FPCOPT+=-Fl$(GCCLIBDIR)'#010+
  'endif'#010+
  'ifdef OTHERLIBDIR'#010+
  'o','verride FPCOPT+=$(addprefix -Fl,$(OTHERLIBDIR))'#010+
  'endif'#010+
  #010+
  #010+
  '[command_end]'#010+
  '# Add commandline options last so they can override'#010+
  'ifdef OPT'#010+
  'override FPCOPT+=$(OPT)'#010+
  'endif'#010+
  #010+
  '# Override options to compile the fpmake-binary with command-line opti'+
  'ons pro','vided in FPMAKEBUILDOPT'#010+
  'ifdef FPMAKEBUILDOPT'#010+
  'override FPMAKE_BUILD_OPT+=$(FPMAKEBUILDOPT)'#010+
  'endif'#010+
  #010+
  '# Add defines from FPCOPTDEF to FPCOPT'#010+
  'ifdef FPCOPTDEF'#010+
  'override FPCOPT+=$(addprefix -d,$(FPCOPTDEF))'#010+
  'endif'#010+
  #010+
  '# Was a config file specified ?'#010+
  'ifd','ef CFGFILE'#010+
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
  'override AFULL_TARGET=$(CPU_TARGET)-$(OS_TARGET',')'#010+
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
  'override COMPILER:=$(strip $(FPC) $(F','PCOPT))'#010+
  #010+
  '# also call ppas if with command option -s'#010+
  '# but only if the FULL_SOURCE and FULL_TARGET are equal'#010+
  '# or if -sh is used'#010+
  'ifneq (,$(findstring -sh ,$(COMPILER)))'#010+
  'UseEXECPPAS=1'#010+
  'endif'#010+
  'ifneq (,$(findstring -s ,$(COMPILER)))'#010+
  'ifeq ($(FULL_','SOURCE),$(FULL_TARGET))'#010+
  'UseEXECPPAS=1'#010+
  'endif'#010+
  'endif'#010+
  'ifneq ($(UseEXECPPAS),1)'#010+
  'EXECPPAS='#010+
  'else'#010+
  'ifdef RUNBATCH'#010+
  'EXECPPAS:=@$(RUNBATCH) $(PPAS)'#010+
  'else'#010+
  'EXECPPAS:=@$(PPAS)'#010+
  'endif'#010+
  'endif'#010+
  #010+
  #010+
  '[loaderrules]'#010+
  '####################################################','################'+
  '#'#010+
  '# Loaders'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_loaders'#010+
  #010+
  'ifneq ($(TARGET_LOADERS),)'#010+
  'override ALLTARGET+=fpc_loaders'#010+
  'override CLEANTARGET+=fpc_loaders_clean'#010+
  'override INSTALLTARGE','T+=fpc_loaders_install'#010+
  #010+
  'override LOADEROFILES:=$(addsuffix $(OEXT),$(TARGET_LOADERS))'#010+
  'endif'#010+
  #010+
  '%$(OEXT): %$(LOADEREXT)'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
  '        $(AS) -o $(COMPILER_UNITTARGETDIR)/$*$(OEXT) $<'#010+
  'else'#010+
  '        $(AS) -o $*$(OEXT) $<'#010+
  'end','if'#010+
  #010+
  'fpc_loaders: $(COMPILER_UNITTARGETDIR) $(LOADEROFILES)'#010+
  #010+
  'fpc_loaders_clean:'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
  '        -$(DEL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFILES)'+
  ')'#010+
  'else'#010+
  '        -$(DEL) $(LOADEROFILES)'#010+
  'endif'#010+
  #010+
  'fpc_loaders_inst','all:'#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
  '        $(INSTALL) $(addprefix $(COMPILER_UNITTARGETDIR)/,$(LOADEROFIL'+
  'ES)) $(INSTALL_UNITDIR)'#010+
  'else'#010+
  '        $(INSTALL) $(LOADEROFILES) $(INSTALL_UNITDIR)'#010+
  'endif'#010+
  #010+
  #010+
  '[unitrule','s]'#010+
  '#####################################################################'#010+
  '# Units'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_units'#010+
  #010+
  'ifneq ($(TARGET_UNITS)$(TARGET_IMPLICITUNITS),)'#010+
  'override ALLTARGET+=f','pc_units'#010+
  #010+
  'override UNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_UNITS))'#010+
  'override IMPLICITUNITPPUFILES=$(addsuffix $(PPUEXT),$(TARGET_IMPLICITU'+
  'NITS))'#010+
  'override INSTALLPPUFILES+=$(UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#010+
  'override CLEANPPUFILES+=$(','UNITPPUFILES) $(IMPLICITUNITPPUFILES)'#010+
  'endif'#010+
  #010+
  'fpc_units: $(COMPILER_UNITTARGETDIR) $(UNITPPUFILES)'#010+
  #010+
  #010+
  '[exerules]'#010+
  '#####################################################################'#010+
  '# Exes'#010+
  '####################################################','################'+
  '#'#010+
  #010+
  '.PHONY: fpc_exes'#010+
  #010+
  '# Programs are not needed for a cross installation'#010+
  'ifndef CROSSINSTALL'#010+
  'ifneq ($(TARGET_PROGRAMS),)'#010+
  'override EXEFILES=$(addsuffix $(EXEEXT),$(TARGET_PROGRAMS))'#010+
  'override EXEOFILES:=$(addsuffix $(OEXT),$(TA','RGET_PROGRAMS)) $(addsuf'+
  'fix $(LTOEXT),$(TARGET_PROGRAMS))$(addprefix $(STATICLIBPREFIX),$(adds'+
  'uffix $(STATICLIBEXT),$(TARGET_PROGRAMS))) $(addprefix $(IMPORTLIBPREF'+
  'IX),$(addsuffix $(STATICLIBEXT),$(TARGET_PROGRAMS)))'#010+
  'override EXEDBGFILES:=$','(addsuffix $(EXEDBGEXT),$(TARGET_PROGRAMS))'#010+
  #010+
  'override ALLTARGET+=fpc_exes'#010+
  'override INSTALLEXEFILES+=$(EXEFILES)'#010+
  'override CLEANEXEFILES+=$(EXEFILES) $(EXEOFILES)'#010+
  'override CLEANEXEDBGFILES+=$(EXEDBGFILES)'#010+
  'ifeq ($(OS_TARGET),os2)'#010+
  'override CLEA','NEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#010+
  'endif'#010+
  'ifeq ($(OS_TARGET),emx)'#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_PROGRAMS))'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'fpc_exes: $(COMPILER_TARGETDIR) $(COMPILER_UNITTARGETDIR) $(EXEFILES)'#010,
  #010+
  #010+
  '[rstrules]'#010+
  '#####################################################################'#010+
  '# Resource strings'#010+
  '#####################################################################'#010+
  #010+
  'ifdef TARGET_RSTS'#010+
  'override RSTFILES=$(addsuffix $(RSTEXT),$(TARGET_','RSTS))'#010+
  #010+
  'override CLEANRSTFILES+=$(RSTFILES)'#010+
  'endif'#010+
  #010+
  #010+
  '[examplerules]'#010+
  '#####################################################################'#010+
  '# Examples'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_examples'#010,
  #010+
  'ifneq ($(TARGET_EXAMPLES),)'#010+
  'HASEXAMPLES=1'#010+
  'override EXAMPLESOURCEFILES=$(wildcard $(addsuffix .pp,$(TARGET_EXAMPL'+
  'ES)) $(addsuffix .pas,$(TARGET_EXAMPLES)) $(addsuffix .lpr,$(TARGET_EX'+
  'AMPLES)) $(addsuffix .dpr,$(TARGET_EXAMPLES)))'#010+
  'override E','XAMPLEFILES:=$(addsuffix $(EXEEXT),$(TARGET_EXAMPLES))'#010+
  'override EXAMPLEOFILES:=$(addsuffix $(OEXT),$(TARGET_EXAMPLES)) $(adds'+
  'uffix $(LTOEXT),$(TARGET_EXAMPLES)) $(addprefix $(STATICLIBPREFIX),$(a'+
  'ddsuffix $(STATICLIBEXT),$(TARGET_EXAMPLES)))',' $(addprefix $(IMPORTLI'+
  'BPREFIX),$(addsuffix $(STATICLIBEXT),$(TARGET_EXAMPLES)))'#010+
  'override EXAMPLEDBGFILES:=$(addsuffix $(EXEDBGEXT),$(TARGET_EXAMPLES))'+
  #010+
  #010+
  'override CLEANEXEFILES+=$(EXAMPLEFILES) $(EXAMPLEOFILES)'#010+
  'override CLEANEXEDBGFILES+=$(E','XAMPLEDBGFILES)'#010+
  'ifeq ($(OS_TARGET),os2)'#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#010+
  'endif'#010+
  'ifeq ($(OS_TARGET),emx)'#010+
  'override CLEANEXEFILES+=$(addsuffix $(AOUTEXT),$(TARGET_EXAMPLES))'#010+
  'endif'#010+
  'endif'#010+
  'ifneq ($(TARGET_EXAMPLED','IRS),)'#010+
  'HASEXAMPLES=1'#010+
  'endif'#010+
  #010+
  'fpc_examples: all $(EXAMPLEFILES) $(addsuffix _all,$(TARGET_EXAMPLEDIR'+
  'S))'#010+
  #010+
  #010+
  '[compilerules]'#010+
  '#####################################################################'#010+
  '# General compile rules'#010+
  '###########################','#########################################'+
  '#'#010+
  #010+
  '.PHONY: fpc_all fpc_smart fpc_debug fpc_release fpc_shared'#010+
  #010+
  '$(FPCMADE): $(ALLDEPENDENCIES) $(ALLTARGET)'#010+
  '        @$(ECHOREDIR) Compiled > $(FPCMADE)'#010+
  #010+
  'fpc_all: $(FPCMADE)'#010+
  #010+
  'fpc_smart:'#010+
  '        $(MAKE)',' all LINKSMART=1 CREATESMART=1'#010+
  #010+
  'fpc_debug:'#010+
  '        $(MAKE) all DEBUG=1'#010+
  #010+
  'fpc_release:'#010+
  '        $(MAKE) all RELEASE=1'#010+
  #010+
  '# General compile rules, available for both possible .pp and .pas exte'+
  'nsions'#010+
  #010+
  '.SUFFIXES: $(EXEEXT) $(PPUEXT) $(OEXT) $(LTOEX','T) .pas .lpr .dpr .pp '+
  '.rc .res'#010+
  #010+
  '$(COMPILER_UNITTARGETDIR):'#010+
  '        $(MKDIRTREE) $(COMPILER_UNITTARGETDIR)'#010+
  #010+
  '$(COMPILER_TARGETDIR):'#010+
  '        $(MKDIRTREE) $(COMPILER_TARGETDIR)'#010+
  #010+
  '%$(PPUEXT): %.pp'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%$(PP','UEXT): %.pas'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%$(EXEEXT): %.pp'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%$(EXEEXT): %.pas'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%$(EXEEXT): %.lpr'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)',#010+
  #010+
  '%$(EXEEXT): %.dpr'#010+
  '        $(COMPILER) $<'#010+
  '        $(EXECPPAS)'#010+
  #010+
  '%.res: %.rc'#010+
  '        windres -i $< -o $@'#010+
  #010+
  '# Search paths for .ppu, .pp, .pas, .lpr, .dpr'#010+
  'vpath %.pp $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#010+
  'vpath %.pas $(COMPILER_SOURCEDIR',') $(COMPILER_INCLUDEDIR)'#010+
  'vpath %.lpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#010+
  'vpath %.dpr $(COMPILER_SOURCEDIR) $(COMPILER_INCLUDEDIR)'#010+
  'vpath %.inc $(COMPILER_INCLUDEDIR)'#010+
  'vpath %$(OEXT) $(COMPILER_UNITTARGETDIR)'#010+
  'vpath %$(LTOEXT) $(COMPIL','ER_UNITTARGETDIR)'#010+
  'vpath %$(PPUEXT) $(COMPILER_UNITTARGETDIR)'#010+
  #010+
  '[sharedrules]'#010+
  '#####################################################################'#010+
  '# Library'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_s','hared'#010+
  #010+
  'override INSTALLTARGET+=fpc_shared_install'#010+
  #010+
  'ifndef SHARED_LIBVERSION'#010+
  'SHARED_LIBVERSION=$(FPC_VERSION)'#010+
  'endif'#010+
  #010+
  'ifndef SHARED_LIBNAME'#010+
  'SHARED_LIBNAME=$(PACKAGE_NAME)'#010+
  'endif'#010+
  #010+
  'ifndef SHARED_FULLNAME'#010+
  'SHARED_FULLNAME=$(SHAREDLIBPREFIX)$(SHARE','D_LIBNAME)-$(SHARED_LIBVERS'+
  'ION)$(SHAREDLIBEXT)'#010+
  'endif'#010+
  #010+
  '# Default sharedlib units are all unit objects'#010+
  'ifndef SHARED_LIBUNITS'#010+
  'SHARED_LIBUNITS:=$(TARGET_UNITS) $(TARGET_IMPLICITUNITS)'#010+
  'override SHARED_LIBUNITS:=$(filter-out $(INSTALL_BUILDUNIT)',',$(SHARED'+
  '_LIBUNITS))'#010+
  'endif'#010+
  #010+
  'fpc_shared:'#010+
  'ifdef HASSHAREDLIB'#010+
  '        $(MAKE) all CREATESHARED=1 LINKSHARED=1 CREATESMART=1'#010+
  'ifneq ($(SHARED_BUILD),n)'#010+
  '        $(PPUMOVE) -q $(SHARED_LIBUNITS) -i$(COMPILER_UNITTARGETDIR) -'+
  'o$(SHARED_FULLNAME) -d$','(COMPILER_UNITTARGETDIR) -P$(BINUTILSPREFIX)'#010+
  'endif'#010+
  'else'#010+
  '        @$(ECHO) Shared Libraries not supported'#010+
  'endif'#010+
  #010+
  'fpc_shared_install:'#010+
  'ifneq ($(SHARED_BUILD),n)'#010+
  'ifneq ($(SHARED_LIBUNITS),)'#010+
  'ifneq ($(wildcard $(COMPILER_UNITTARGETDIR)/$(SHARED_FU','LLNAME)),)'#010+
  '        $(INSTALL) $(COMPILER_UNITTARGETDIR)/$(SHARED_FULLNAME) $(INST'+
  'ALL_SHAREDDIR)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '[installrules]'#010+
  '#####################################################################'#010+
  '# Install rules'#010+
  '########################','############################################'+
  '#'#010+
  #010+
  '.PHONY: fpc_install fpc_sourceinstall fpc_exampleinstall'#010+
  #010+
  'ifdef INSTALL_UNITS'#010+
  'override INSTALLPPUFILES+=$(addsuffix $(PPUEXT),$(INSTALL_UNITS))'#010+
  'endif'#010+
  #010+
  'ifdef INSTALL_BUILDUNIT'#010+
  'override INSTALLPP','UFILES:=$(filter-out $(INSTALL_BUILDUNIT)$(PPUEXT)'+
  ',$(INSTALLPPUFILES))'#010+
  'endif'#010+
  #010+
  'ifdef INSTALLPPUFILES'#010+
  '# Avoid getting the same name twice as some install EXE fail'#010+
  '# in this case, happends for instance for OS/2 target'#010+
  'ifneq ($(IMPORTLIBPREFIX)','-$(STATICLIBEXT),$(STATICLIBPREFIX)-$(STATI'+
  'CLIBEXT))'#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPPUFI'+
  'LES)) $(subst $(PPUEXT),$(LTOEXT),$(INSTALLPPUFILES)) $(addprefix $(ST'+
  'ATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT','),$(INSTALLPPUFILES))'+
  ') $(addprefix $(IMPORTLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(I'+
  'NSTALLPPUFILES)))'#010+
  'else'#010+
  'override INSTALLPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(INSTALLPPUFI'+
  'LES)) $(subst $(PPUEXT),$(LTOEXT),$(INSTALLPPUFILES)) $(','addprefix $('+
  'STATICLIBPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(INSTALLPPUFILES))'+
  ')'#010+
  'endif'#010+
  #010+
  'ifneq ($(UNITTARGETDIRPREFIX),)'#010+
  'override INSTALLPPUFILENAMES:=$(notdir $(INSTALLPPUFILES))'#010+
  'override INSTALLPPULINKFILENAMES:=$(notdir $(INSTALLPPULIN','KFILES))'#010+
  'override INSTALLPPUFILES=$(addprefix $(UNITTARGETDIRPREFIX),$(INSTALLP'+
  'PUFILENAMES))'#010+
  'override INSTALLPPULINKFILES=$(wildcard $(addprefix $(UNITTARGETDIRPRE'+
  'FIX),$(INSTALLPPULINKFILENAMES)))'#010+
  'endif'#010+
  '# Implicitly install Package.fpc'#010+
  'over','ride INSTALL_CREATEPACKAGEFPC=1'#010+
  'endif'#010+
  #010+
  'ifdef INSTALLEXEFILES'#010+
  'ifneq ($(TARGETDIRPREFIX),)'#010+
  'override INSTALLEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(notdir $(IN'+
  'STALLEXEFILES)))'#010+
  'endif'#010+
  'endif'#010+
  #010+
  'fpc_install: all $(INSTALLTARGET)'#010+
  'ifdef INSTALLEXEF','ILES'#010+
  '        $(MKDIR) $(INSTALL_BINDIR)'#010+
  '        $(INSTALLEXE) $(INSTALLEXEFILES) $(INSTALL_BINDIR)'#010+
  'endif'#010+
  'ifdef INSTALL_CREATEPACKAGEFPC'#010+
  'ifdef FPCMAKE'#010+
  '# If the fpcpackage variable is set then create and install Package.fp'+
  'c,'#010+
  '# a safety check ','is done if Makefile.fpc is available'#010+
  'ifdef PACKAGE_VERSION'#010+
  'ifneq ($(wildcard Makefile.fpc),)'#010+
  '        $(FPCMAKE) -p -T$(CPU_TARGET)-$(OS_TARGET) Makefile.fpc'#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#010+
  '        $(INSTALL) Package.fpc $(INSTALL_UNITDI','R)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'ifdef INSTALLPPUFILES'#010+
  '        $(MKDIR) $(INSTALL_UNITDIR)'#010+
  '        $(INSTALL) $(INSTALLPPUFILES) $(INSTALL_UNITDIR)'#010+
  'ifneq ($(INSTALLPPULINKFILES),)'#010+
  '        $(INSTALL) $(INSTALLPPULINKFILES) $(INSTALL_UNITDIR)'#010+
  'endif',#010+
  'ifneq ($(wildcard $(LIB_FULLNAME)),)'#010+
  '        $(MKDIR) $(INSTALL_LIBDIR)'#010+
  '        $(INSTALL) $(LIB_FULLNAME) $(INSTALL_LIBDIR)'#010+
  'ifdef inUnix'#010+
  '        ln -sf $(LIB_FULLNAME) $(INSTALL_LIBDIR)/$(LIB_NAME)'#010+
  'endif'#010+
  'endif'#010+
  'endif'#010+
  'ifdef INSTALL_FILES'#010+
  '  ','      $(MKDIR) $(INSTALL_DATADIR)'#010+
  '        $(INSTALL) $(INSTALL_FILES) $(INSTALL_DATADIR)'#010+
  'endif'#010+
  #010+
  'fpc_sourceinstall: distclean'#010+
  '        $(MKDIR) $(INSTALL_SOURCEDIR)'#010+
  '        $(COPYTREE) $(BASEDIR)/* $(INSTALL_SOURCEDIR)'#010+
  #010+
  'fpc_exampleinstall: $(','EXAMPLEINSTALLTARGET) $(addsuffix _distclean,$'+
  '(TARGET_EXAMPLEDIRS))'#010+
  'ifdef HASEXAMPLES'#010+
  '        $(MKDIR) $(INSTALL_EXAMPLEDIR)'#010+
  'endif'#010+
  'ifdef EXAMPLESOURCEFILES'#010+
  '        $(COPY) $(EXAMPLESOURCEFILES) $(INSTALL_EXAMPLEDIR)'#010+
  'endif'#010+
  'ifdef TARGET_EXAMP','LEDIRS'#010+
  '        $(COPYTREE) $(addsuffix /*,$(TARGET_EXAMPLEDIRS)) $(INSTALL_EX'+
  'AMPLEDIR)'#010+
  'endif'#010+
  #010+
  '[distinstallrules]'#010+
  '#####################################################################'#010+
  '# Dist Install'#010+
  '##########################################','##########################'+
  '#'#010+
  #010+
  '.PHONY: fpc_distinstall'#010+
  #010+
  'fpc_distinstall: install exampleinstall'#010+
  #010+
  #010+
  '[zipinstallrules]'#010+
  '#####################################################################'#010+
  '# Zip'#010+
  '##################################################','##################'+
  '#'#010+
  #010+
  '.PHONY: fpc_zipinstall fpc_zipsourceinstall fpc_zipexampleinstall'#010+
  #010+
  '# Temporary path to pack a file, can only use a single deep'#010+
  '# subdir, because the deltree can'#039't see the whole tree to remove'#010+
  'ifndef PACKDIR'#010+
  'ifndef inUni','x'#010+
  'PACKDIR=$(BASEDIR)/../fpc-pack'#010+
  'else'#010+
  'PACKDIR=/tmp/fpc-pack'#010+
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
  'ifndef FULLZIPNAME'#010,
  'FULLZIPNAME=$(ZIPCROSSPREFIX)$(ZIPPREFIX)$(ZIPNAME)$(ZIPSUFFIX)'#010+
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
  'ifdef inUni','x'#010+
  'USETAR=1'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Use a wrapper script by default for Os/2'#010+
  'ifndef inUnix'#010+
  'USEZIPWRAPPER=1'#010+
  'endif'#010+
  #010+
  '# We need to be able to run in the current OS so fix'#010+
  '# the path separator'#010+
  'ifdef USEZIPWRAPPER'#010+
  'ZIPPATHSEP=$(PATHSEP)'#010+
  'ZIPWRAPPER=$(subst /',',$(PATHSEP),$(DIST_DESTDIR)/fpczip$(SRCBATCHEXT)'+
  ')'#010+
  'else'#010+
  'ZIPPATHSEP=/'#010+
  'endif'#010+
  #010+
  '# Create commands to create the zip/tar file'#010+
  'ZIPCMD_CDPACK:=cd $(subst /,$(ZIPPATHSEP),$(PACKDIR))'#010+
  'ZIPCMD_CDBASE:=cd $(subst /,$(ZIPPATHSEP),$(BASEDIR))'#010+
  'ifdef USETAR',#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(TAREXT)'#010+
  'ZIPCMD_ZIP:=$(TARPROG) c$(TAROPT)f $(ZIPDESTFILE) *'#010+
  'else'#010+
  'ZIPDESTFILE:=$(DIST_DESTDIR)/$(FULLZIPNAME)$(ZIPEXT)'#010+
  'ZIPCMD_ZIP:=$(subst /,$(ZIPPATHSEP),$(ZIPPROG)) -Dr $(ZIPOPT) $(ZIPDES'+
  'TFILE)',' *'#010+
  'endif'#010+
  #010+
  'fpc_zipinstall:'#010+
  '        $(MAKE) $(ZIPTARGET) INSTALL_PREFIX=$(PACKDIR) ZIPINSTALL=1'#010+
  '        $(MKDIR) $(DIST_DESTDIR)'#010+
  '        $(DEL) $(ZIPDESTFILE)'#010+
  'ifdef USEZIPWRAPPER'#010+
  '# Handle gecho separate as we need to espace \ with \\'#010+
  'ifneq ($','(ECHOREDIR),echo)'#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDPACK))" > $(ZIPWRAPPE'+
  'R)'#010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_ZIP))" >> $(ZIPWRAPPER)'+
  #010+
  '        $(ECHOREDIR) -e "$(subst \,\\,$(ZIPCMD_CDBASE))" >> $(ZIPWRAPP'+
  'ER)'#010+
  'else',#010+
  '        echo $(ZIPCMD_CDPACK) > $(ZIPWRAPPER)'#010+
  '        echo $(ZIPCMD_ZIP) >> $(ZIPWRAPPER)'#010+
  '        echo $(ZIPCMD_CDBASE) >> $(ZIPWRAPPER)'#010+
  'endif'#010+
  'ifdef inUnix'#010+
  '        /bin/sh $(ZIPWRAPPER)'#010+
  'else'#010+
  'ifdef RUNBATCH'#010+
  '        $(RUNBATCH) $(ZIPWRAPPER)',#010+
  'else'#010+
  '        $(ZIPWRAPPER)'#010+
  'endif'#010+
  'endif'#010+
  '        $(DEL) $(ZIPWRAPPER)'#010+
  'else'#010+
  '        $(ZIPCMD_CDPACK) ; $(ZIPCMD_ZIP) ; $(ZIPCMD_CDBASE)'#010+
  'endif'#010+
  '        $(DELTREE) $(PACKDIR)'#010+
  #010+
  'fpc_zipsourceinstall:'#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=source','install ZIPSUFFIX=$(Z'+
  'IPSOURCESUFFIX)'#010+
  #010+
  'fpc_zipexampleinstall:'#010+
  'ifdef HASEXAMPLES'#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=exampleinstall ZIPSUFFIX=$(ZI'+
  'PEXAMPLESUFFIX)'#010+
  'endif'#010+
  #010+
  'fpc_zipdistinstall:'#010+
  '        $(MAKE) fpc_zipinstall ZIPTARGET=distins','tall'#010+
  #010+
  #010+
  '[cleanrules]'#010+
  '#####################################################################'#010+
  '# Clean rules'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_clean fpc_cleanall fpc_distclean'#010+
  #010+
  'ifdef EXEFILES'#010+
  'over','ride CLEANEXEFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEFIL'+
  'ES))'#010+
  'override CLEANEXEDBGFILES:=$(addprefix $(TARGETDIRPREFIX),$(CLEANEXEDB'+
  'GFILES))'#010+
  'endif'#010+
  #010+
  'ifdef CLEAN_PROGRAMS'#010+
  'override CLEANEXEFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuff','ix $'+
  '(EXEEXT), $(CLEAN_PROGRAMS)))'#010+
  'override CLEANEXEDBGFILES+=$(addprefix $(TARGETDIRPREFIX),$(addsuffix '+
  '$(EXEDBGEXT), $(CLEAN_PROGRAMS)))'#010+
  'endif'#010+
  #010+
  'ifdef CLEAN_UNITS'#010+
  'override CLEANPPUFILES+=$(addsuffix $(PPUEXT),$(CLEAN_UNITS))'#010+
  'endif'#010+
  #010+
  'ifdef CL','EANPPUFILES'#010+
  'override CLEANPPULINKFILES:=$(subst $(PPUEXT),$(OEXT),$(CLEANPPUFILES)'+
  ') $(subst $(PPUEXT),$(LTOEXT),$(CLEANPPUFILES)) $(addprefix $(STATICLI'+
  'BPREFIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(CLEANPPUFILES))) $(addpre'+
  'fix $(IMPORTLIBPRE','FIX),$(subst $(PPUEXT),$(STATICLIBEXT),$(CLEANPPUF'+
  'ILES)))'#010+
  'ifdef DEBUGSYMEXT'#010+
  'override CLEANPPULINKFILES+=$(subst $(PPUEXT),$(DEBUGSYMEXT),$(CLEANPP'+
  'UFILES))'#010+
  'endif'#010+
  'override CLEANPPUFILENAMES:=$(CLEANPPUFILES)'#010+
  'override CLEANPPUFILES=$(addprefix',' $(UNITTARGETDIRPREFIX),$(CLEANPPU'+
  'FILENAMES))'#010+
  'override CLEANPPULINKFILENAMES:=$(CLEANPPULINKFILES)'#010+
  'override CLEANPPULINKFILES=$(wildcard $(addprefix $(UNITTARGETDIRPREFI'+
  'X),$(CLEANPPULINKFILENAMES)))'#010+
  'endif'#010+
  #010+
  'fpc_clean: $(CLEANTARGET)'#010+
  'ifdef CL','EANEXEFILES'#010+
  '        -$(DEL) $(CLEANEXEFILES)'#010+
  'endif'#010+
  '# DELTREE instead of DEL because on Mac OS X these are directories'#010+
  'ifdef CLEANEXEDBGFILES'#010+
  '        -$(DELTREE) $(CLEANEXEDBGFILES)'#010+
  'endif'#010+
  'ifdef CLEANPPUFILES'#010+
  '        -$(DEL) $(CLEANPPUFILES)'#010,
  'endif'#010+
  'ifneq ($(CLEANPPULINKFILES),)'#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#010+
  'endif'#010+
  'ifdef CLEANRSTFILES'#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#010+
  'endif'#010+
  'ifdef CLEAN_FILES'#010+
  '        -$(DEL) $(CLEAN_FILES)'#010+
  'endif'#010+
  'ifdef LIB_N','AME'#010+
  '        -$(DEL) $(LIB_NAME) $(LIB_FULLNAME)'#010+
  'endif'#010+
  '        -$(DEL) $(FPCMADE) *$(FULL_TARGET).fpm Package.fpc *$(ASMEXT)'#010+
  '        -$(DEL) $(FPCEXTFILE) $(REDIRFILE) script*.res link*.res *_scr'+
  'ipt.res *_link.res'#010+
  '        -$(DEL) $(PPAS) *_p','pas$(BATCHEXT) ppas$(BATCHEXT) ppaslink$('+
  'BATCHEXT)'#010+
  #010+
  'fpc_cleanall: $(CLEANTARGET)'#010+
  'ifdef CLEANEXEFILES'#010+
  '        -$(DEL) $(CLEANEXEFILES)'#010+
  'endif'#010+
  'ifdef COMPILER_UNITTARGETDIR'#010+
  'ifdef CLEANPPUFILES'#010+
  '        -$(DEL) $(CLEANPPUFILES)'#010+
  'endif'#010+
  'ifneq ($(CLE','ANPPULINKFILES),)'#010+
  '        -$(DEL) $(CLEANPPULINKFILES)'#010+
  'endif'#010+
  'ifdef CLEANRSTFILES'#010+
  '        -$(DEL) $(addprefix $(UNITTARGETDIRPREFIX),$(CLEANRSTFILES))'#010+
  'endif'#010+
  'endif'#010+
  'ifdef CLEAN_FILES'#010+
  '        -$(DEL) $(CLEAN_FILES)'#010+
  'endif'#010+
  '        -$(DELTREE) uni','ts'#010+
  '        -$(DELTREE) bin'#010+
  '        -$(DEL) *$(OEXT) *$(LTOEXT) *$(PPUEXT) *$(RSTEXT) *$(ASMEXT) *'+
  '$(STATICLIBEXT) *$(SHAREDLIBEXT) *$(PPLEXT)'#010+
  'ifneq ($(PPUEXT),.ppu)'#010+
  '        -$(DEL) *.o *.ppu *.a'#010+
  'endif'#010+
  '        -$(DELTREE) *$(SMARTEXT)'#010+
  '       ',' -$(DEL) fpcmade.* Package.fpc *.fpm'#010+
  '        -$(DEL) $(FPCEXTFILE) $(REDIRFILE) script*.res link*.res *_scr'+
  'ipt.res *_link.res'#010+
  '        -$(DEL) $(PPAS) *_ppas$(BATCHEXT) ppas$(BATCHEXT) ppaslink$(BA'+
  'TCHEXT)'#010+
  'ifdef AOUTEXT'#010+
  '        -$(DEL) *$(AOU','TEXT)'#010+
  'endif'#010+
  'ifdef DEBUGSYMEXT'#010+
  '        -$(DEL) *$(DEBUGSYMEXT)'#010+
  'endif'#010+
  'ifdef LOCALFPMAKEBIN'#010+
  '        -$(DEL) $(LOCALFPMAKEBIN)'#010+
  '        -$(DEL) $(FPMAKEBINOBJ)'#010+
  'endif'#010+
  #010+
  'fpc_distclean: cleanall'#010+
  #010+
  #010+
  '[baseinforules]'#010+
  '####################################','################################'+
  '#'#010+
  '# Base info rules'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_baseinfo'#010+
  #010+
  'override INFORULES+=fpc_baseinfo'#010+
  #010+
  'fpc_baseinfo:'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Package i','nfo =='#010+
  '        @$(ECHO)  Package Name..... $(PACKAGE_NAME)'#010+
  '        @$(ECHO)  Package Version.. $(PACKAGE_VERSION)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Configuration info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  FPC.......... $(FPC)'#010+
  '        @$','(ECHO)  FPC Version.. $(FPC_VERSION)'#010+
  '        @$(ECHO)  Source CPU... $(CPU_SOURCE)'#010+
  '        @$(ECHO)  Target CPU... $(CPU_TARGET)'#010+
  '        @$(ECHO)  Source OS.... $(OS_SOURCE)'#010+
  '        @$(ECHO)  Target OS.... $(OS_TARGET)'#010+
  '        @$(ECHO)  Ful','l Source.. $(FULL_SOURCE)'#010+
  '        @$(ECHO)  Full Target.. $(FULL_TARGET)'#010+
  '        @$(ECHO)  SourceSuffix. $(SOURCESUFFIX)'#010+
  '        @$(ECHO)  TargetSuffix. $(TARGETSUFFIX)'#010+
  '        @$(ECHO)  FPC fpmake... $(FPCFPMAKE)'#010+
  '        @$(ECHO)'#010+
  '        @','$(ECHO)  == Directory info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Required pkgs... $(REQUIRE_PACKAGES)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Basedir......... $(BASEDIR)'#010+
  '        @$(ECHO)  FPCDir.......... $(FPCDIR)'#010+
  '        @$(ECHO)  CrossBinDir.','.... $(CROSSBINDIR)'#010+
  '        @$(ECHO)  UnitsDir........ $(UNITSDIR)'#010+
  '        @$(ECHO)  PackagesDir..... $(PACKAGESDIR)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  GCC library..... $(GCCLIBDIR)'#010+
  '        @$(ECHO)  Other library... $(OTHERLIBDIR)'#010+
  '        ','@$(ECHO)'#010+
  '        @$(ECHO)  == Tools info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  As........ $(AS)'#010+
  '        @$(ECHO)  Ld........ $(LD)'#010+
  '        @$(ECHO)  Ar........ $(AR)'#010+
  '        @$(ECHO)  Rc........ $(RC)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Mv..','...... $(MVPROG)'#010+
  '        @$(ECHO)  Cp........ $(CPPROG)'#010+
  '        @$(ECHO)  Rm........ $(RMPROG)'#010+
  '        @$(ECHO)  GInstall.. $(GINSTALL)'#010+
  '        @$(ECHO)  Echo...... $(ECHO)'#010+
  '        @$(ECHO)  Shell..... $(SHELL)'#010+
  '        @$(ECHO)  Date...... ','$(DATE)'#010+
  '        @$(ECHO)  FPCMake... $(FPCMAKE)'#010+
  '        @$(ECHO)  PPUMove... $(PPUMOVE)'#010+
  '        @$(ECHO)  Zip....... $(ZIPPROG)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Object info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Target Loaders........ $','(TARGET_LOADERS)'#010+
  '        @$(ECHO)  Target Units.......... $(TARGET_UNITS)'#010+
  '        @$(ECHO)  Target Implicit Units. $(TARGET_IMPLICITUNITS)'#010+
  '        @$(ECHO)  Target Programs....... $(TARGET_PROGRAMS)'#010+
  '        @$(ECHO)  Target Dirs........... ','$(TARGET_DIRS)'#010+
  '        @$(ECHO)  Target Examples....... $(TARGET_EXAMPLES)'#010+
  '        @$(ECHO)  Target ExampleDirs.... $(TARGET_EXAMPLEDIRS)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Clean Units......... $(CLEAN_UNITS)'#010+
  '        @$(ECHO)  Clean Files..','....... $(CLEAN_FILES)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Install Units....... $(INSTALL_UNITS)'#010+
  '        @$(ECHO)  Install Files....... $(INSTALL_FILES)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  == Install info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO) ',' DateStr.............. $(DATESTR)'#010+
  '        @$(ECHO)  ZipName.............. $(ZIPNAME)'#010+
  '        @$(ECHO)  ZipPrefix............ $(ZIPPREFIX)'#010+
  '        @$(ECHO)  ZipCrossPrefix....... $(ZIPCROSSPREFIX)'#010+
  '        @$(ECHO)  ZipSuffix............ $(ZI','PSUFFIX)'#010+
  '        @$(ECHO)  FullZipName.......... $(FULLZIPNAME)'#010+
  '        @$(ECHO)  Install FPC Package.. $(INSTALL_FPCPACKAGE)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Install base dir..... $(INSTALL_BASEDIR)'#010+
  '        @$(ECHO)  Install binary dir..','. $(INSTALL_BINDIR)'#010+
  '        @$(ECHO)  Install library dir.. $(INSTALL_LIBDIR)'#010+
  '        @$(ECHO)  Install units dir.... $(INSTALL_UNITDIR)'#010+
  '        @$(ECHO)  Install source dir... $(INSTALL_SOURCEDIR)'#010+
  '        @$(ECHO)  Install doc dir...... $(','INSTALL_DOCDIR)'#010+
  '        @$(ECHO)  Install example dir.. $(INSTALL_EXAMPLEDIR)'#010+
  '        @$(ECHO)  Install data dir..... $(INSTALL_DATADIR)'#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Dist destination dir. $(DIST_DESTDIR)'#010+
  '        @$(ECHO)  Dist zip nam','e........ $(DIST_ZIPNAME)'#010+
  '        @$(ECHO)'#010+
  #010+
  '[inforules]'#010+
  '#####################################################################'#010+
  '# Info rules'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_info'#010+
  #010+
  'fpc_info: $(','INFORULES)'#010+
  #010+
  '[makefilerules]'#010+
  '#####################################################################'#010+
  '# Rebuild Makefile'#010+
  '#####################################################################'#010+
  #010+
  '.PHONY: fpc_makefile fpc_makefiles fpc_makefile_sub1',' fpc_makefile_su'+
  'b2 \'#010+
  '        fpc_makefile_dirs'#010+
  #010+
  'fpc_makefile:'#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) Makefile.fpc'#010+
  #010+
  'fpc_makefile_sub1:'#010+
  'ifdef TARGET_DIRS'#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_DIRS))'#010+
  'endif'#010+
  'ifdef',' TARGET_EXAMPLEDIRS'#010+
  '        $(FPCMAKE) -w -T$(OS_TARGET) $(addsuffix /Makefile.fpc,$(TARGE'+
  'T_EXAMPLEDIRS))'#010+
  'endif'#010+
  #010+
  'fpc_makefile_sub2: $(addsuffix _makefile_dirs,$(TARGET_DIRS) $(TARGET_'+
  'EXAMPLEDIRS))'#010+
  #010+
  'fpc_makefile_dirs: fpc_makefile_sub1 fpc_m','akefile_sub2'#010+
  #010+
  'fpc_makefiles: fpc_makefile fpc_makefile_dirs'#010+
  #010+
  '[localmakefile]'#010+
  '#####################################################################'#010+
  '# Local Makefile'#010+
  '#####################################################################'#010+
  #010+
  'ifneq',' ($(wildcard fpcmake.loc),)'#010+
  'include fpcmake.loc'#010+
  'endif'#010+
  #010+
  #010+
  '[userrules]'#010+
  '#####################################################################'#010+
  '# Users rules'#010+
  '#####################################################################'#010+
  #010+
  '[lclrules]'#010+
  '######','##############################################################'+
  '#'#010+
  '# LCL Rules'#010+
  '#####################################################################'#010+
  #010+
  '# LCL Platform'#010+
  'ifndef LCL_PLATFORM'#010+
  'ifneq ($(findstring $(OS_TARGET),win32 win64),)'#010+
  'LCL_PLATFO','RM=win32'#010+
  'else'#010+
  'LCL_PLATFORM=gtk'#010+
  'endif'#010+
  'endif'#010+
  'export LCL_PLATFORM'#010+
  #010+
  '# Check if the specified LCLDIR is correct'#010+
  'ifdef LCLDIR'#010+
  'override LCLDIR:=$(subst \,/,$(LCLDIR))'#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010,
  'else'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  #010+
  '# Check if the default LCLDIR is correct'#010+
  'ifdef DEFAULT_LCLDIR'#010+
  'override LCLDIR:=$(subst \,/,$(DEFAULT_LCLDIR))'#010+
  'ifeq ($(wildcard $(LCLDIR)/units/$(LCL_PLATFORM)),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Check f','or development version'#010+
  'ifeq ($(LCLDIR),wrong)'#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
  ' $(addsuffix /units/$(LCL_PLATFORM),$(BASEDIR)/lcl $(BASEDIR)))))'#010+
  'ifeq ($(LCLDIR),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# Che','ck for release version'#010+
  'ifeq ($(LCLDIR),wrong)'#010+
  'override LCLDIR=$(subst /units/$(LCL_PLATFORM),,$(firstword $(wildcard'+
  ' $(addsuffix /lib/lazarus/units/$(LCL_PLATFORM),/usr/local /usr))))'#010+
  'ifeq ($(LCLDIR),)'#010+
  'override LCLDIR=wrong'#010+
  'endif'#010+
  'endif'#010+
  #010+
  '# G','enerate dirs'#010+
  'override LCLUNITDIR=$(wildcard $(LCLDIR)/units/$(LCL_PLATFORM) $(LCLDI'+
  'R)/units)'#010+
  'override LCLCOMPONENTDIR=$(wildcard $(LCLDIR)/.. $(LCLDIR)/../componen'+
  'ts $(LCLDIR)/components)'#010+
  'export LCLDIR LCLUNITDIR LCLCOMPONENTDIR'#010+
  #010+
  '# Add LCL ','dirs to paths'#010+
  'override REQUIRE_PACKAGESDIR+=$(LCLCOMPONENTDIR)'#010+
  'override COMPILER_UNITDIR+=$(LCLUNITDIR)'#010+
  #010+
  '[lclinforules]'#010+
  '#####################################################################'#010+
  '# LCL Info rules'#010+
  '#################################','###################################'+
  '#'#010+
  'override INFORULES+=lclinfo'#010+
  #010+
  '.PHONY: lclinfo'#010+
  #010+
  'lclinfo:'#010+
  '        @$(ECHO)  == LCL info =='#010+
  '        @$(ECHO)'#010+
  '        @$(ECHO)  Platform............. $(LCL_PLATFORM)'#010+
  '        @$(ECHO)  LCLDIR............... $(','LCLDIR)'#010+
  '        @$(ECHO)  LCL Unit dir......... $(LCLUNITDIR)'#010+
  '        @$(ECHO)  LCL Component dir.... $(LCLCOMPONENTDIR)'#010+
  '        @$(ECHO)'#010+
  #010+
  '[fpmakeprerules]'#010+
  '#####################################################################'#010+
  '# fpmake preru','les'#010+
  '#####################################################################'#010+
  'FPMAKEBIN=fpmake$(SRCEXEEXT)'#010+
  'FPMAKEBINOBJ=fpmake$(OEXT) fpmake$(LTOEXT)'#010+
  'LOCALFPMAKEBIN=.$(PATHSEP)$(FPMAKEBIN)'#010+
  #010+
  '# Convert the OS_TARGET and CPU_TARGET options to fpma','ke'#039's --os '+
  'and --cpu parameters'#010+
  'ifdef OS_TARGET'#010+
  'FPC_TARGETOPT+=--os=$(OS_TARGET)'#010+
  'endif'#010+
  'ifdef CPU_TARGET'#010+
  'FPC_TARGETOPT+=--cpu=$(CPU_TARGET)'#010+
  'endif'#010+
  #010+
  '# Get the location of the bootstrap-fpmkunit units'#010+
  'PACKAGEDIR_FPMKUNIT:=$(firstword $(subst /Ma','kefile.fpc,,$(strip $(wi'+
  'ldcard $(addsuffix /fpmkunit/Makefile.fpc,$(PACKAGESDIR))))))'#010+
  'ifneq ($(PACKAGEDIR_FPMKUNIT),)'#010+
  'UNITDIR_FPMAKE_FPMKUNIT=$(PACKAGEDIR_FPMKUNIT)/units_bs/$(SOURCESUFFIX'+
  ')'#010+
  'override COMPILER_FPMAKE_UNITDIR=$(UNITDIR_FPMAKE_','FPMKUNIT)'#010+
  'FPMKUNIT_SRC=$(PACKAGEDIR_FPMKUNIT)/src/fpmkunit.pp'#010+
  'FPMKUNIT_PPU=$(UNITDIR_FPMAKE_FPMKUNIT)/fpmkunit.ppu'#010+
  'endif'#010+
  #010+
  'ifdef FPMAKE_SKIP_CONFIG'#010+
  'override FPMAKE_BUILD_OPT+=$(FPMAKE_SKIP_CONFIG)'#010+
  'endif'#010+
  #010+
  '[fpmakerules]'#010+
  '#######################','#############################################'+
  '#'#010+
  '# fpmake rules'#010+
  '#####################################################################'#010+
  '.PHONY: fpc_fpmake fpc_fpmake_clean fpc_fpmake_install fpc_fpmake_exam'+
  'pleinstall'#010+
  #010+
  '# Do not pass the Makefile',#039's unit and binary target locations. fpm'+
  'ake uses it'#039's own.'#010+
  'override FPCOPT:=$(filter-out -FU%,$(FPCOPT))'#010+
  'override FPCOPT:=$(filter-out -FE%,$(FPCOPT))'#010+
  '# Compose general fpmake-parameters'#010+
  'ifdef FPMAKEOPT'#010+
  'FPMAKE_OPT+=$(FPMAKEOPT)'#010+
  'endif'#010+
  'FPMAKE','_OPT+=--localunitdir=$(FPCDIR)'#010+
  'FPMAKE_OPT+=--globalunitdir=$(FPCDIR)/packages'#010+
  'FPMAKE_OPT+=$(FPC_TARGETOPT)'#010+
  'FPMAKE_OPT+=$(addprefix -o ,$(FPCOPT))'#010+
  'FPMAKE_OPT+=--compiler=$(FPC)'#010+
  'FPMAKE_OPT+=-bu'#010+
  #010+
  'FPMAKE_INSTALL_OPT+=--unitinstalldir=$(INSTALL_','UNITDIR)'#010+
  'ifdef UNIXHier'#010+
  'FPMAKE_INSTALL_OPT+=--prefix=$(INSTALL_PREFIX)'#010+
  'FPMAKE_INSTALL_OPT+=--baseinstalldir=$(INSTALL_LIBDIR)/fpc/$(FPC_VERSI'+
  'ON)'#010+
  'else'#010+
  'FPMAKE_INSTALL_OPT+=--prefix=$(INSTALL_BASEDIR)'#010+
  'endif'#010+
  #010+
  'override ALLTARGET+=fpc_fpmake'#010+
  'over','ride INSTALLTARGET+=fpc_fpmake_install'#010+
  'override EXAMPLEINSTALLTARGET+=fpc_fpmake_exampleinstall'#010+
  '# If no fpmake exists and (dist)clean is called, do not try to build f'+
  'pmake, it will'#010+
  '# most often fail because the dependencies are cleared.'#010+
  '# I','n case of a clean, simply do nothing'#010+
  'ifneq ($(wildcard $(LOCALFPMAKEBIN)),)'#010+
  'override CLEANTARGET+=fpc_fpmake_clean'#010+
  'endif'#010+
  #010+
  '$(FPMKUNIT_PPU): $(FPMKUNIT_SRC)'#010+
  '            $(MAKE) -C $(PACKAGEDIR_FPMKUNIT) bootstrap $(addprefix OP'+
  'T=,$(FPMAKE_BUI','LD_OPT))'#010+
  #010+
  '$(FPMAKEBIN): fpmake.pp $(FPMKUNIT_PPU)'#010+
  '        $(FPCFPMAKE) fpmake.pp $(addprefix -Fu,$(COMPILER_FPMAKE_UNITD'+
  'IR)) $(FPMAKE_BUILD_OPT)'#010+
  #010+
  'fpc_fpmake: $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) compile $(FPMAKE_OPT)'#010+
  #010+
  'fpc_fpmake_clean',': $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) clean $(FPMAKE_OPT)'#010+
  #010+
  'fpc_fpmake_install: $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) install $(FPMAKE_OPT) $(FPMAKE_INSTALL_O'+
  'PT)'#010+
  #010+
  '# This is not completely valid. Exampleinstall should only install',' t'+
  'he examples, while'#010+
  '# fpmake -ie installs everything, including the examples. This also me'+
  'ans that on'#010+
  '# a distinstall fpmake install wil be called twice.'#010+
  'fpc_fpmake_exampleinstall: $(FPMAKEBIN)'#010+
  '            $(LOCALFPMAKEBIN) install -ie $(F','PMAKE_OPT) $(FPMAKE_INS'+
  'TALL_OPT)'#010
);
