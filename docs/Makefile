ikfaststats?=ikfaststats.pp
openrave_version=$(shell openrave-config --version)
doxycommands="STRIP_FROM_PATH = $(PWD)\\n\
PROJECT_NUMBER = $(openrave_version)\\n\
ALIASES += openraveversion=`openrave-config --version`\\n\
"

.PHONY: json_% json html_% html clean clean_doxygen clean_sphinx

# this empty line prevents from any intermediate files from getting deleted.... weird
.SECONDARY:

all: html_en

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

build/%/coreapihtml/index.html build/%/coreapixml/index.xml: images_all_installed Doxyfile.html Doxyfile.% ../include/openrave/*.h
	rm -f doxygenhtml_installed_$*
	mkdir -p build
	echo "$(doxycommands)" | cat Doxyfile.html Doxyfile.$* - > build/Doxyfile.html.$*
	echo "$(doxycommands)" | cat Doxyfile.latex Doxyfile.$* - > build/Doxyfile.latex.$*
	doxygen build/Doxyfile.html.$*
	python preprocess_doxygen.py --outdir=build/$*
	cp -f ../resources/openrave_banner_dark.png build/$*/coreapihtml/
# store the symlinks for later zip compression
	mkdir -p build/openravehtml-$(openrave_version)/$*
	ln -f -s ../../$*/coreapihtml build/openravehtml-$(openrave_version)/$*/coreapihtml
	touch doxygenhtml_installed_$*

doxygenhtml_installed_%: build/%/coreapihtml/index.html ;

openravepy_changed:
	touch openravepy_changed

openrave_plugins_changed:
	touch openrave_plugins_changed

openravepy_internal_doc: openravepy_changed build/en/coreapixml/index.xml build/ja/coreapixml/index.xml ../python/bindings/*.cpp
	rm -f openravepy_internal_doc
	python build_openravepy_internal.py --languagecode en --languagecode ja
	#touch openravepy_internal_doc

source/interface_types.rst source/interface_types: openrave_plugins_changed build_interfaces.py
	python build_interfaces.py --outdir=source

source/openravepy/openravepy.rst: openravepy_changed
	mkdir -p source/openravepy
	ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	PYTHONPATH=`pwd`:$PYTHONPATH python sphinx-autopackage-script/generate_modules.py --dest-dir=source/openravepy --suffix=rst --maxdepth=3 --no-toc --sep-files `pwd`/openravepy
	#rm -f openravepy

source/ikfast/index.rst: $(ikfaststats) build_ikdatabase.py
	rm -f ikfast_installed openravepy # need to remove openravepy link since ikfaststats.pp has object links with the real openravepy dir
	#ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	rm -f openravepy
	python build_ikdatabase.py --ikfaststats=$(ikfaststats) --outdir=source/ikfast
	touch ikfast_installed

ikfast_installed: source/ikfast/index.rst ;

# create/update internationalization files
# have to set LANG to en, or otherwise the automodule stuff might not return english translations
# set OPENRAVE_INTERNAL_COMMENTS=0 in order to avoid the doxygen translated comments from popping up in the translations
build/locale/index.pot: openravepy_changed source/*.rst source/devel/*.rst source/tutorials/*.rst source/architecture/*.rst conf.py source/openravepy/openravepy.rst source/interface_types.rst
	rm -f build_locale_installed
	ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	OPENRAVE_INTERNAL_COMMENTS=0 LANG=en_US.UTF-8 PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -t gettext -D language=en -b gettext -c . source build/locale
	#rm -f openravepy
	touch build_locale_installed

build_locale_installed: build/locale/index.pot ;

# manual method:
# mkdir -p locale/$SPHINXLANGUAGE/LC_MESSAGES
# msginit --locale=ja --input=build/locale/index.pot --output=locale/ja/LC_MESSAGES/index.po
# msgmerge -U locale/ja/LC_MESSAGES/index.po build/locale/index.pot
# msgfmt locale/ja/LC_MESSAGES/environment_variables.po -o locale/ja/LC_MESSAGES/environment_variables.mo
#
# have to remove all doctrees in order for translations to be regenerated
# filter out en
$(filter-out locale/en/LC_MESSAGES/openravesphinx.po, locale/%/LC_MESSAGES/openravesphinx.po): build_locale_installed
	echo "language='$*'\n\
locale_dirs = ['locale']" > tempconf_$*.py
	sphinx-gettext-helper -c tempconf_$*.py -p build/locale --update --statistics
	rm -f tempconf_$*.py

$(filter-out locale/en/LC_MESSAGES/openravesphinx.mo, locale/%/LC_MESSAGES/openravesphinx.mo): locale/%/LC_MESSAGES/*.po
	echo "language='$*'\n\
locale_dirs = ['locale']" > tempconf_$*.py
	sphinx-gettext-helper -c tempconf_$*.py -p build/locale --build --statistics
	rm -f tempconf_$*.py
	msgfmt locale/$*/LC_MESSAGES/openravesphinx.po -o locale/$*/LC_MESSAGES/openravesphinx.mo
	rm -rf build/$*/sphinxhtml/.doctrees build/$*/sphinxjson/.doctrees

build/openravejson-$(openrave_version)/%/sphinxjson build/%/sphinxjson/index.fjson: source/*.rst source/devel/*.rst source/tutorials/*.rst source/architecture/*.rst conf.py _templates/*.html ikfast_installed sphinxext/*.py source/openravepy/openravepy.rst locale/%/LC_MESSAGES/openravesphinx.mo locale/%/LC_MESSAGES/*.po source/interface_types.rst
	rm -f sphinxjson_installed_$* build/$*/sphinxjson/index.fjson
	ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -D language=$* -t jsontag -b json -c . source build/$*/sphinxjson
	#rm -f openravepy
	# store the symlinks for later zip compression
	mkdir -p build/openravejson-$(openrave_version)/$*
	rm -f build/openravejson-$(openrave_version)/$*/sphinxjson
	ln -f -s ../../$*/sphinxjson build/openravejson-$(openrave_version)/$*/sphinxjson
	touch sphinxjson_installed_$*

sphinxjson_installed_%: build/openravejson-$(openrave_version)/%/sphinxjson build/%/sphinxjson/index.fjson ;

build/openravehtml-$(openrave_version)/%/sphinxhtml build/%/sphinxhtml/index.html: source/*.rst source/devel/*.rst source/tutorials/*.rst source/architecture/*.rst conf.py _templates/*.html ikfast_installed sphinxext/*.py source/openravepy/openravepy.rst locale/%/LC_MESSAGES/openravesphinx.mo locale/%/LC_MESSAGES/*.po source/interface_types.rst
	rm -f sphinxhtml_installed_$* build/$*/sphinxhtml/index.html
	ln -s -f `openrave-config --python-dir`/openravepy/_openravepy_ openravepy
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -D language=$* -t htmltag -b html -c . source build/$*/sphinxhtml
	#rm -f openravepy
	mkdir -p build/openravehtml-$(openrave_version)/$*
	rm -f build/openravehtml-$(openrave_version)/$*/sphinxhtml
	ln -f -s ../../$*/sphinxhtml build/openravehtml-$(openrave_version)/$*/sphinxhtml
	touch sphinxhtml_installed_$*

sphinxhtml_installed_%: build/openravehtml-$(openrave_version)/%/sphinxhtml build/%/sphinxhtml/index.html ;

json_%: sphinxjson_installed_% doxygenhtml_installed_% ;

jsonall: json_en json_ja ;
html_%: doxygenhtml_installed_% sphinxhtml_installed_% ;
htmlall: html_en html_ja ;

# temporary directory with symlinks should already be created, so just zip it
openravejsonzip_installed openravejson-$(openrave_version).zip: jsonall
	rm -f openravejsonzip_installed
	cd build; zip -r ../openravejson-$(openrave_version).zip openravejson-$(openrave_version)
	touch openravejsonzip_installed

openravejsonzip: openravejsonzip_installed openravejson-$(openrave_version).zip ;

# temporary directory with symlinks should already be created, so just zip it
openravehtmlzip_installed openravehtml-$(openrave_version).zip: htmlall
	rm -f openravehtmlzip_installed
	cd build; zip -r ../openravehtml-$(openrave_version).zip openravehtml-$(openrave_version)

openravehtmlzip: openravehtmlzip_installed openravehtml-$(openrave_version).zip ;

openravejson-latest_stable.zip openravehtml-latest_stable.zip: jsonall htmlall
	rm -f latest_stablezip
	ln -s -f openravejson-$(openrave_version) build/openravejson-latest_stable
	ln -s -f openravehtml-$(openrave_version) build/openravehtml-latest_stable
	cd build; zip -r ../openravejson-latest_stable.zip openravejson-latest_stable
	cd build; zip -r ../openravehtml-latest_stable.zip openravehtml-latest_stable
	touch latest_stablezip

latest_stablezip: openravejson-latest_stable.zip openravehtml-latest_stable.zip ;

#sendtoserver: jsonall htmlall
#	tar cjf openravedocs.tgz build/$(language)/coreapihtml build/$(language)/sphinxjson
#	scp openravedocs.tgz $server:$targetdir
#	ssh $server "cd $targetdir; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old build"
#scp openravedocs.tgz diankov@programmingvision.com:"~/openrave/"
#ssh diankov@programmingvision.com "cd ~/openrave; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old build"

clean_doxygen_%:
	rm -rf build/Doxyfile.*.$* build/$*

clean_sphinx_%:
	rm -rf _templates/examples.html _templates/databases.html _templates/database_generator_template.rst source/openravepy build/locale build/$*/sphinxhtml build/$*/sphinxjson

clean:
	rm -rf build *_installed_*
