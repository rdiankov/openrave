#!/bin/bash
trunk=$1
revision=`python -c "import pysvn; print pysvn.Client().info('$trunk').revision.number"`
basename="openrave-r$revision"
svn export $trunk "$basename-linux-src"
rm -rf "$basename-linux-src"/msvc_files.tgz # too big to include into openrave
tar cjf "$basename-linux-src.tar.bz2" "$basename-linux-src"
mkdir -p latest_donotdownload
mv "$basename-linux-src.tar.bz2" latest_donotdownload/
tar cf latest_donotdownload.tgz latest_donotdownload
rm -rf "$basename-linux-src" latest_donotdownload

ssh openravetesting,openrave@shell.sourceforge.net create # always create
scp latest_donotdownload.tgz openravetesting,openrave@frs.sourceforge.net:/home/frs/project/o/op/openrave/
ssh openravetesting,openrave@shell.sourceforge.net "cd /home/frs/project/o/op/openrave; tar xf latest_donotdownload.tgz; chmod -R g+w latest_donotdownload; rm -rf latest_stable latest_donotdownload.tgz; mv latest_donotdownload latest_stable"
rm -f latest_donotdownload.tgz

svn rm --non-interactive --username openravetesting -m "Delete Latest Stable Tab (Tagged by Jenkins)." https://openrave.svn.sourceforge.net/svnroot/openrave/tags/latest_stable
svn cp --non-interactive --username openravetesting -m "Latest Stable Tab (Tagged by Jenkins). Revision: $SVN_REVISION" $trunk https://openrave.svn.sourceforge.net/svnroot/openrave/tags/latest_stable

#ssh-keygen -t dsa -f ~/.ssh/id_dsa.openravetesting.sf -P "" -C "openravetesting@shell.sourceforge.net"

