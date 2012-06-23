language?=en
ikfaststats?=ikfaststats.pp
doxycommands="STRIP_FROM_PATH = $(PWD)\\n\
PROJECT_NUMBER = `openrave-config --version`\\n\
ALIASES += openraveversion=`openrave-config --version`\\n\
"

openrave.org: sphinxjson_installed_$(language) doxygenhtml_installed_$(language)

html: doxygenhtml_installed_$(language) sphinxhtml_installed_$(language)

images/openrave_architecture.pdf: images/openrave_architecture.dia
	dia -e images/openrave_architecture.eps images/openrave_architecture.dia
	dia -e images/openrave_architecture.png -s 700 images/openrave_architecture.dia
	ps2pdf -dEPSCrop images/openrave_architecture.eps images/openrave_architecture.pdf

images/examples/visibilityplanning_framework.png: images/examples/visibilityplanning_framework.dia
	dia -e images/examples/visibilityplanning_framework.png -s 1024 images/examples/visibilityplanning_framework.dia

images/openrave_architecture_jp.pdf: images/openrave_architecture_jp.dia
	dia -e images/openrave_architecture_jp.eps images/openrave_architecture_jp.dia
	dia -e images/openrave_architecture_jp.png -s 700 images/openrave_architecture_jp.dia
	ps2pdf -dEPSCrop images/openrave_architecture_jp.eps images/openrave_architecture_jp.pdf

images/openrave_documentation.png: images/openrave_documentation.dia
	dia -e images/openrave_documentation.eps images/openrave_documentation.dia
	dia -e images/openrave_documentation.png -s 700 images/openrave_documentation.dia

images_base_installed:
	cp -f ../resources/openrave_banner_400.png _static/
	cp -f ../resources/openrave_icon*.png _static/
	touch images_base_installed

images_all_installed: images_base_installed images/openrave_architecture.pdf images/examples/visibilityplanning_framework.png images/openrave_architecture_jp.pdf images/openrave_documentation.png
	touch images_all_installed

doxygenhtml_installed_$(language): images_all_installed Doxyfile.html Doxyfile.$(language)
	rm -f doxygenhtml_installed_$(language)
	mkdir -p build
	echo "$(doxycommands)" | cat Doxyfile.html Doxyfile.$(language) - > build/Doxyfile.html.$(language)
	echo "$(doxycommands)" | cat Doxyfile.latex Doxyfile.$(language) - > build/Doxyfile.latex.$(language)
	doxygen build/Doxyfile.html.$(language)
	python preprocess_doxygen.py --outdir=build/$(language)
	cp -f ../resources/openrave_banner_dark.png build/$(language)/coreapihtml/
	touch doxygenhtml_installed_$(language)

doxygenhtml: doxygenhtml_installed_$(language)

openravepy_changed:
	touch openravepy_changed

openrave_plugins_changed:
	touch openrave_plugins_changed

openravepy_internal_doc: openravepy_changed
	rm -f openravepy_internal_doc
	python build_openravepy_internal.py --languagecode en --languagecode ja
	touch openravepy_internal_doc

source/interface_types.rst source/interface_types: openrave_plugins_changed
	python build_interfaces.py --outdir=source

source/openravepy/openravepy.rst: openravepy_changed
	mkdir -p source/openravepy
	ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	PYTHONPATH=`pwd`:$PYTHONPATH python sphinx-autopackage-script/generate_modules.py --dest-dir=source/openravepy --suffix=rst --maxdepth=3 --no-toc --sep-files `pwd`/openravepy
	rm openravepy

source/ikfast/index.rst: $(ikfaststats)
	python build_ikdatabase.py --ikfaststats=$(ikfaststats) --outdir=source/ikfast

ikfast: source/ikfast/index.rst

# create/update internationalization files
# have to set LANG to en, or otherwise the automodule stuff might not return english translations
# set OPENRAVE_INTERNAL_COMMENTS=0 in order to avoid the doxygen translated comments from popping up in the translations
build_locale_installed: openravepy_changed source/*.rst source/devel/*.rst source/tutorials/*.rst source/architecture/*.rst conf.py
	rm -f build_locale_installed
	OPENRAVE_INTERNAL_COMMENTS=0 LANG=en_US.UTF-8 sphinx-build -D language=en -b gettext -c . source build/locale
	touch build_locale_installed

# manual method:
# mkdir -p locale/$SPHINXLANGUAGE/LC_MESSAGES
# msginit --locale=ja --input=build/locale/index.pot --output=locale/ja/LC_MESSAGES/index.po
# msgmerge -U locale/ja/LC_MESSAGES/index.po build/locale/index.pot
# msgfmt locale/ja/LC_MESSAGES/environment_variables.po -o locale/ja/LC_MESSAGES/environment_variables.mo
#
# have to remove all doctrees in order for translations to be regenerated
locale/$(language)/LC_MESSAGES/openravesphinx.mo: build_locale_installed
ifneq ($(language),en)
	echo "language='$(language)'\n\
locale_dirs = ['locale']" > tempconf_$(language).py
	sphinx-gettext-helper -c tempconf_$(language).py -p build/locale --update --build --statistics
	rm -f tempconf_$(language).py
	msgfmt locale/$(language)/LC_MESSAGES/openravesphinx.po -o locale/$(language)/LC_MESSAGES/openravesphinx.mo
	rm -rf build/$(language)/sphinxmain/.doctrees build/$(language)/sphinxjson/.doctrees
endif

sphinxjson_installed_$(language): build_locale_installed source/*.rst source/devel/*.rst source/tutorials/*.rst source/architecture/*.rst locale/$(language)/LC_MESSAGES/openravesphinx.mo conf.py _templates/*.html ikfast
	rm -f sphinxjson_installed_$(language)
	ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	sphinx-build -D language=$(language) -b json -c . source build/$(language)/sphinxjson
	rm -f openravepy
	touch sphinxjson_installed_$(language)

sphinxjson: sphinxjson_installed_$(language)

sphinxhtml_installed_$(language): build_locale_installed source/*.rst source/devel/*.rst source/tutorials/*.rst source/architecture/*.rst locale/$(language)/LC_MESSAGES/openravesphinx.mo conf.py _templates/*.html ikfast
	rm -f sphinxhtml_installed_$(language)
	ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	sphinx-build -D language=$(language) -b html -c . source build/$(language)/sphinxhtml
	rm -f openravepy
	touch sphinxhtml_installed_$(language)

sphinxhtml: sphinxhtml_installed_$(language)

sendtoserver: sphinxjson_installed_$(language) doxygenhtml_installed_$(language)
	tar cjf openravedocs.tgz build/$(language)/coreapihtml build/$(language)/sphinxjson
	scp openravedocs.tgz $server:$targetdir
	ssh $server "cd $targetdir; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old build"
#scp openravedocs.tgz diankov@programmingvision.com:"~/openrave/"
#ssh diankov@programmingvision.com "cd ~/openrave; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old build"

clean_doxygen:
	rm -rf build/Doxyfile.*.$(language) build/$(language)

clean_sphinx:
	rm -rf _templates/examples.html _templates/databases.html _templates/database_generator_template.rst source/openravepy build/locale build/$(language)/sphinxhtml build/$(language)/sphinxjson

clean:
	rm -rf build *_installed_*
