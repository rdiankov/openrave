#!/bin/bash
trunk=$1
latest_stable="https://openrave.svn.sourceforge.net/svnroot/openrave/tags/latest_stable"
# only update latest_stable if the revisions changed
revision=`python -c "import pysvn; print pysvn.Client().info('$trunk').commit_revision.number"`
prevrevision=`python -c "import pysvn; revision=pysvn.Client().info2('$latest_stable',recurse=False)[0][1].rev; print pysvn.Client().log('$latest_stable',revision,revision)[0].message.split()[-1]"`

if [ "$revision" != "$prevrevision" ]; then
    basename="openrave-r$revision"
    svn export $trunk "$basename-linux-src"
    rm -rf "$basename-linux-src"/msvc_files.tgz # too big to include into openrave
    tar cjf "$basename-linux-src.tar.bz2" "$basename-linux-src"
    mkdir -p latest_donotdownload
    mv "$basename-linux-src.tar.bz2" latest_donotdownload/
    mv *.exe latest_donotdownload/ # windows setup files
    tar cf latest_donotdownload.tgz latest_donotdownload
    rm -rf "$basename-linux-src" latest_donotdownload

    ssh openravetesting,openrave@shell.sourceforge.net create # always create
    scp latest_donotdownload.tgz openravetesting,openrave@frs.sourceforge.net:/home/frs/project/o/op/openrave/
    ssh openravetesting,openrave@shell.sourceforge.net "cd /home/frs/project/o/op/openrave; tar xf latest_donotdownload.tgz; chmod -R g+w latest_donotdownload; rm -rf latest_stable latest_donotdownload.tgz; mv latest_donotdownload latest_stable"
    rm -f latest_donotdownload.tgz

    svn rm --non-interactive --username openravetesting -m "Delete Latest Stable Tab (Tagged by Jenkins)." $latest_stable
    svn cp --non-interactive --username openravetesting -m "Latest Stable Tab (Tagged by Jenkins). Revision: $SVN_REVISION" $trunk $latest_stable
fi

#ssh-keygen -t dsa -f ~/.ssh/id_dsa.openravetesting.sf -P "" -C "openravetesting@shell.sourceforge.net"
