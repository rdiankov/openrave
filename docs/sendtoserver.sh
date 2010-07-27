#!/bin/bash
rm -rf images_temp; svn export images images_temp
tar czf ordocs.tgz en/html en/openravepy-html en/openrave.pdf images_temp ja/html ja/openravepy-html sphinx/sphinx-docs
rm -rf images_temp

# send to server
scp ordocs.tgz diankov@programmingvision.com:~/openrave/
ssh diankov@programmingvision.com "cd ~/openrave; rm -rf ordocs_old; mkdir -p ordocs2; tar xzf ordocs.tgz -C ordocs2; mv ordocs2/images_temp ordocs2/images; mv ordocs ordocs_old; mv ordocs2 ordocs; rm -rf ordocs_old ordocs.tgz"
