#!/bin/bash
if test -z "$1"
then
    server="rdiankov@openrave.org"
else
    server=$1
fi
if test -z "$2"
then
    targetdir="/var/openrave/"
else
    targetdir=$2
fi
tar cjf openravedocs.tgz build/en/coreapihtml build/en/main build/ja/coreapihtml build/ja/main 
scp openravedocs.tgz $server:$targetdir
ssh $server "cd $targetdir; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old build"

scp openravedocs.tgz diankov@programmingvision.com:"~/openrave/"
ssh diankov@programmingvision.com "cd ~/openrave; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old build"
