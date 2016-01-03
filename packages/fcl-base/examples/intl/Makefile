#
# Makefile to make the internationalization files for the resstr example
# Add a 2 letter code when adding a language to DEMOLANGUAGES
#

<<<<<<< HEAD
<<<<<<< HEAD
DEMOLANGUAGES=fr nl de pb ru cs
=======
DEMOLANGUAGES=fr nl de pb
>>>>>>> graemeg/fixes_2_2
=======
DEMOLANGUAGES=fr nl de pb
>>>>>>> origin/fixes_2_2

#
# No need to add after this line.
#

OBJECTS=$(addprefix restest.,$(DEMOLANGUAGES))
MOOBJECTS=$(addsuffix .mo,$(OBJECTS))

.SUFFIXES: .mo .po

%.mo: %.po
	msgfmt -o $@ $?

all: $(MOOBJECTS)
