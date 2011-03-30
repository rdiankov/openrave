#!/bin/bash
if test -z "$1"
then
    server="diankov@programmingvision.com"
else
    server=$1
fi
tar cjf openravedocs.tgz build/en/coreapihtml build/en/main build/ja/coreapihtml build/ja/main 
scp openravedocs.tgz $server:~/openrave/
ssh $server "cd ~/openrave; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old openravedocs.tgz build"
