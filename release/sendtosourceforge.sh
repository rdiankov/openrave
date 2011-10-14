#!/bin/bash
# Puts the newest files into latest_stable, and deletes any files that are more than 7 days old.
trunk=$1
latest_stable="https://openrave.svn.sourceforge.net/svnroot/openrave/tags/latest_stable"
# only update latest_stable if the revisions changed
revision=`python -c "import pysvn; print pysvn.Client().info('$trunk').commit_revision.number"`
prevrevision=`python -c "import pysvn; revision=pysvn.Client().info2('$latest_stable',recurse=False)[0][1].rev; print pysvn.Client().log('$latest_stable',revision,revision)[0].message.split()[-1]"`

if [ "$revision" != "$prevrevision" ]; then
    basename="openrave-r$revision"
    svn export $trunk "$basename-linux-src"
    rm -rf "$basename-linux-src"/msvc_* # too big to include into openrave
    tar cjf "$basename-linux-src.tar.bz2" "$basename-linux-src"
    mkdir -p latest_stable
    mv "$basename-linux-src.tar.bz2" latest_stable/
    cp *.exe latest_stable/ # windows setup files
    cp $trunk/release/README.rst .
    tar cf latest_stable.tgz latest_stable README.rst
    rm -rf "$basename-linux-src" latest_stable README.rst

    ssh openravetesting,openrave@shell.sourceforge.net create # always create
    scp latest_stable.tgz openravetesting,openrave@frs.sourceforge.net:/home/frs/project/o/op/openrave/
    # remove files 7 or more days old
    ssh openravetesting,openrave@shell.sourceforge.net "cd /home/frs/project/o/op/openrave; tar xf latest_stable.tgz; chmod -R g+w latest_stable; rm -f latest_stable.tgz; find latest_stable -mtime +7 -type f -exec rm -rf {} \;"
    rm -f latest_stable.tgz

    svn rm --non-interactive --username openravetesting -m "Delete Latest Stable Tab (Tagged by Jenkins)." $latest_stable
    svn cp --non-interactive --username openravetesting -m "Latest Stable Tab (Tagged by Jenkins). Revision: $revision" $trunk $latest_stable
fi

#ssh-keygen -t dsa -f ~/.ssh/id_dsa.openravetesting.sf -P "" -C "openravetesting@shell.sourceforge.net"
