#!/bin/bash
tar cjf ordocs.tgz build/en/coreapihtml build/en/main build/ja/coreapihtml build/ja/main 

# send to server
scp ordocs.tgz diankov@programmingvision.com:~/openrave/
ssh diankov@programmingvision.com "cd ~/openrave; rm -rf ordocs_old; mkdir -p ordocs2; tar xjf ordocs.tgz -C ordocs2; mv ordocs2/images_temp ordocs2/images; mv ordocs ordocs_old; mv ordocs2 ordocs; rm -rf ordocs_old ordocs.tgz"
