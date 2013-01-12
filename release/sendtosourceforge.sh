#!/bin/bash
# Puts the newest files into latest_stable, and deletes any files that are more than 7 days old.
trunk=$1
VERSION=`python $trunk/release/extractversion.py $trunk/CMakeLists.txt`

mkdir -p latest_stable
cd $trunk
git archive --format=tar --prefix=openrave-$VERSION-src/ master | bzip2 -9 > openrave-$VERSION-src.tar.bz2
mv "openrave-$VERSION-src.tar.bz2" ../latest_stable/
cd ..
cp *.exe latest_stable/ # windows setup files
cp $trunk/release/README.rst .
tar cf latest_stable.tgz latest_stable README.rst
rm -rf latest_stable README.rst

ssh openravetesting,openrave@shell.sourceforge.net create # always create
scp latest_stable.tgz openravetesting,openrave@frs.sourceforge.net:/home/frs/project/o/op/openrave/
# remove files 7 or more days old
ssh openravetesting,openrave@shell.sourceforge.net "cd /home/frs/project/o/op/openrave; tar xf latest_stable.tgz; chmod -R g+w latest_stable; rm -f latest_stable.tgz; find latest_stable -mtime +30 -type f -exec rm -rf {} \;"
rm -f latest_stable.tgz

# update the latest_stable branch
cd $trunk
git checkout latest_stable
git pull origin latest_stable # just in case
git merge master
git push origin latest_stable
git checkout master

#fi
#ssh-keygen -t dsa -f ~/.ssh/id_dsa.openravetesting.sf -P "" -C "openravetesting@shell.sourceforge.net"
