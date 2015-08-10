#!/bin/bash
set -e

SCRIPT=$(basename $0)
ROOT=$(readlink -e $0 | xargs dirname)
LANGUAGES="en_US ja_JP"
C_CPP_DIRS="src/libopenrave src/libopenrave-core python/bindings"
PYTHON_DIRS="python"

function usage
{
    echo "Usage: ${SCRIPT}"
    echo "       Maintainer script to update locale files for openrave"
    echo
    exit 1
}

function make_pot
{
    local DIR

	mkdir -p locale
	rm -f locale/openrave.pot

	# first make an empty pot
	xgettext -d openrave --force-po --output locale/openrave.pot --language=c /dev/null

    # extract strings from c/c++ sources
    for DIR in ${C_CPP_DIRS}; do
        if [ -d "${DIR}" ]; then
            find "${DIR}" -type f \( -iname '*.c' -or -iname '*.cpp' -or -iname '*.h' \) -print | xargs -r \
	        	xgettext -d openrave --force-po --join-existing --sort-output --width=9999 --output locale/openrave.pot --language=c++ -k_
        fi
    done

    # extract strings from python sources
    for DIR in ${PYTHON_DIRS}; do
        if [ -d "${DIR}" ]; then
    	    find "${DIR}" -type f -iname '*.py' -print | xargs -r \
	        	xgettext -d openrave --force-po --join-existing --sort-output --width=9999 --output locale/openrave.pot --language=python -k -k_ -kugettext -kugettext_lazy -kungettext -kungettext_lazy
        fi
    done
}

function make_po
{
	local L
	L="$1"

	mkdir -p locale/${L}/LC_MESSAGES

	# initialize the po file if it does not exist
	if [ ! -f "locale/${L}/LC_MESSAGES/openrave.po" ]; then
		echo -n "  "
		msginit --locale=${L} --no-translator --width=9999 --input=locale/openrave.pot --output-file=locale/${L}/LC_MESSAGES/openrave.po
	fi

	# if update is needed, update the po file
	echo -n "  "
	msgmerge --verbose --silent --lang=${L} --force-po --update --sort-output --width=9999 locale/${L}/LC_MESSAGES/openrave.po locale/openrave.pot
}

function make_mo
{
	local L
	L="$1"

	mkdir -p locale/${L}/LC_MESSAGES
	if [ ! -f "locale/${L}/LC_MESSAGES/openrave.po" ]; then
		return
	fi

	echo -n "  "
	msgfmt --verbose --check-format --check-domain --output-file=locale/${L}/LC_MESSAGES/openrave.mo locale/${L}/LC_MESSAGES/openrave.po
}

# go to root of repository
cd "${ROOT}"

# make pot
make_pot

# make po for each language
for L in ${LANGUAGES}; do
	echo "${SCRIPT}: ${L}"
	make_po ${L}
	make_mo ${L}
	echo
done

