#!/bin/bash
set -e

SCRIPT=$(basename $0)
ROOT=$(readlink -e $0 | xargs dirname | xargs dirname)
LANGUAGES="en_US ja_JP"
DOMAINS=""

function usage
{
    echo "Usage: ${SCRIPT}"
    echo "       Maintainer script to update locale files for openrave"
    echo
    exit 1
}

function make_pot
{
    local D DIRS DIR
    D="$1"

    shift
    DIRS=$*

    mkdir -p locale
    rm -f locale/${D}.pot

    # first make an empty pot
    xgettext -d ${D} --force-po --output locale/${D}.pot --language=c /dev/null

    # extract strings from c/c++ sources
    for DIR in ${DIRS}; do
        if [ -d "${DIR}" ]; then
            find "${DIR}" -type f \( -iname '*.c' -or -iname '*.cpp' -or -iname '*.h' \) -print | xargs -r \
                xgettext -d ${D} --force-po --join-existing --sort-output --width=9999 --output locale/${D}.pot --language=c++ -k_
        fi
    done

    # extract strings from python sources
    for DIR in ${DIRS}; do
        if [ -d "${DIR}" ]; then
            find "${DIR}" -type f -iname '*.py' -print | xargs -r \
                xgettext -d ${D} --force-po --join-existing --sort-output --width=9999 --output locale/${D}.pot --language=python -k -k_ -kugettext -kugettext_lazy -kungettext -kungettext_lazy
        fi
    done
}

function make_po
{
    local D L
    D="$1"
    L="$2"

    mkdir -p locale/${L}/LC_MESSAGES

    # initialize the po file if it does not exist
    if [ ! -f "locale/${L}/LC_MESSAGES/${D}.po" ]; then
        echo -n "  "
        msginit --locale=${L} --no-translator --width=9999 --input=locale/${D}.pot --output-file=locale/${L}/LC_MESSAGES/${D}.po
    fi

    # if update is needed, update the po file
    echo -n "  "
    msgmerge --verbose --silent --lang=${L} --force-po --update --sort-output --width=9999 locale/${L}/LC_MESSAGES/${D}.po locale/${D}.pot
}

function make_mo
{
    local D L
    D="$1"
    L="$2"

    mkdir -p locale/${L}/LC_MESSAGES
    if [ ! -f "locale/${L}/LC_MESSAGES/${D}.po" ]; then
        return
    fi

    echo -n "  "
    msgfmt --verbose --check-format --check-domain --output-file=locale/${L}/LC_MESSAGES/${D}.mo locale/${L}/LC_MESSAGES/${D}.po
}

# go to root of repository
cd "${ROOT}"

# make pot
DOMAIN=openrave
DOMAINS="${DOMAINS} ${DOMAIN}"
make_pot ${DOMAIN} \
    src/libopenrave \
    src/libopenrave-core \
    python

DOMAIN=openrave_plugins_configurationcache
DOMAINS="${DOMAINS} ${DOMAIN}"
make_pot ${DOMAIN} \
    plugins/configurationcache

DOMAIN=openrave_plugins_ikfastsolvers
DOMAINS="${DOMAINS} ${DOMAIN}"
make_pot ${DOMAIN} \
    plugins/ikfastsolvers

DOMAIN=openrave_plugins_oderave
DOMAINS="${DOMAINS} ${DOMAIN}"
make_pot ${DOMAIN} \
    plugins/oderave

DOMAIN=openrave_plugins_rplanners
DOMAINS="${DOMAINS} ${DOMAIN}"
make_pot ${DOMAIN} \
    plugins/rplanners

# make po for each language
for L in ${LANGUAGES}; do
    for D in ${DOMAINS}; do
        echo "${SCRIPT}: ${L}: ${D}"
        make_po ${D} ${L}
        echo
    done
done

