#!/bin/bash
tar cjf openravedocs.tgz build/en/coreapihtml build/en/main build/ja/coreapihtml build/ja/main 
scp openravedocs.tgz diankov@programmingvision.com:~/openrave/
ssh diankov@programmingvision.com "cd ~/openrave; rm -rf build; tar xjf openravedocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old openravedocs.tgz build"
