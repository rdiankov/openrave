#!/bin/bash
trunk=$1
revision=`python -c "import pysvn; print pysvn.Client().info('$trunk').revision.number"`
basename="openrave-`openrave-config --version`-r$revision"
svn export $trunk "$basename-linux-src"
rm -rf "$basename-linux-src"/msvc_files.tgz # too big to include into openrave
tar cjf "$basename-linux-src.tar.bz2" "$basename-linux-src"
mkdir -p latest_donotdownload
mv "$basename-linux-src.tar.bz2" latest_donotdownload/
tar cf latest_donotdownload.tgz latest_donotdownload
rm -rf "$basename-linux-src" latest_donotdownload

ssh rdiankov,openrave@shell.sourceforge.net create # always create
scp latest_donotdownload.tgz rdiankov,openrave@frs.sourceforge.net:/home/frs/project/o/op/openrave/
ssh rdiankov,openrave@shell.sourceforge.net "cd /home/frs/project/o/op/openrave; tar xf latest_donotdownload.tgz; rm -rf latest latest_donotdownload.tgz; mv latest_donotdownload latest"

#ssh-keygen -t dsa -f ~/.ssh/id_dsa.rdiankov.sf -P "" -C "rdiankov@shell.sourceforge.net"
