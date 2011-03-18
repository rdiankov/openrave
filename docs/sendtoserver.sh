#!/bin/bash
tar cjf ordocs.tgz build/en/coreapihtml build/en/main build/ja/coreapihtml build/ja/main 
scp ordocs.tgz diankov@programmingvision.com:~/openrave/
ssh diankov@programmingvision.com "cd ~/openrave; rm -rf build; tar xjf ordocs.tgz; mv -f en en_old; mv -f ja ja_old; mv build/en .; mv build/ja .; rm -rf en_old ja_old ordocs.tgz build"
