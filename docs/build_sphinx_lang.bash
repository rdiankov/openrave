#!/bin/bash
# argument $1 is language: en, ja
export SPHINXLANGUAGE=$1

rm -rf build/$SPHINXLANGUAGE/main

echo "language='$SPHINXLANGUAGE'
locale_dirs = ['locale']" > tempconf_$SPHINXLANGUAGE.py
sphinx-gettext-helper -c tempconf_$SPHINXLANGUAGE.py -p build/locale --update --build --statistics
if [ "$?" -ne 0 ]; then echo "sphinx-gettext-helper failed"; exit 1; fi
rm -f tempconf_$SPHINXLANGUAGE.py

# have to remove all doctrees in order for translations to be regenerated
rm -rf build/$SPHINXLANGUAGE/main/.doctrees

# manual method:
# mkdir -p locale/$SPHINXLANGUAGE/LC_MESSAGES
# msginit --locale=ja --input=build/locale/index.pot --output=locale/ja/LC_MESSAGES/index.po
# msgmerge -U locale/ja/LC_MESSAGES/index.po build/locale/index.pot
# msgfmt locale/ja/LC_MESSAGES/environment_variables.po -o locale/ja/LC_MESSAGES/environment_variables.mo

# necessary to export LANG in order to get the correct openravepy_int translations
if [ "$SPHINXLANGUAGE" == "ja" ]; then
    export LANG=ja_JP.UTF-8
else
    export LANG=en_US.UTF-8
fi

ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
sphinx-build -D language=$SPHINXLANGUAGE -b html -c . source build/$SPHINXLANGUAGE/main
rm openravepy
if [ "$?" -ne 0 ]; then echo "sphinx-build failed"; exit 1; fi
