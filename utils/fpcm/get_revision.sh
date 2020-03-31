#!/bin/sh

# List of files which should be considered imprtant
# for the behavior of fpcmake binary
# fpcmake.inc and Makefile should be excluded from that list
if [ "$#" -ge 1 ] ; then
important_files="$*"
else
important_sources=`ls -1 fpcm*.pp fpcmake.ini Makefile.fpc`
fi

LANG=C
export LANG

tmpfiles=

# git repository?
if [ -d ../../.git ] ; then
  USEGIT=1
  echo Using git repository
  # we just look for the last commit date here
  for f in $important_sources ; do
    tmpfile=.tmp.$f
    tmpfiles="$tmpfiles $tmpfile"
    echo "Change information for $f: "
    git log -1 --pretty="format:%ci %h" $f > $tmpfile
    echo >> $tmpfile
    cat $tmpfile
  done
  git_date=`cat $tmpfiles | sort -n | tail -1 | gawk '{ print $1 }'`

  git_hash=`cat $tmpfiles | sort -n | tail -1 | gawk '{ print $4 }'`
  
  echo "Last date is $git_date, hash is $git_hash"
  echo "'$git_date hash $git_hash'" > revision.inc
else
  for f in $important_sources ; do
    tmpfile=.tmp.$f
    tmpfiles="$tmpfiles $tmpfile"
    svn info $f > $tmpfile
  done

  # echo "svn_info is $svn_info"
  svn_date=`gawk '/Last Changed Date: / {print $4 }' $tmpfiles | sort -n | tail -1`
  svn_rev=`gawk '/Last Changed Rev: / {print $4 }' $tmpfiles | sort -n | tail -1`
  echo "for files $important_sources, date is $svn_date, rev is $svn_rev"
  echo "'$svn_date rev $svn_rev'" > revision.inc
fi

rm -Rf $tmpfiles
