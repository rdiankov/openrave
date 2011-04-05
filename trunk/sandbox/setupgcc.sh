#!/bin/bash
# install ccache and colorgcc if they are not installed
if [ -z "`which ccache`" ]; then
  echo Installing ColorGCC ...
  sudo aptitude install ccache
fi

# Color GCC
if [ -z "`which colorgcc`" ]; then
  echo Installing ColorGCC ...
  sudo aptitude install colorgcc
fi
if [ ! -z "`which colorgcc`" ]; then
  export CC="colorgcc"
  alias gcc='colorgcc'
  for C in `grep /usr/bin /etc/colorgcc/colorgccrc | sed -e 's/# //' -e 's/:.*//'`; do
    if [ ! -e /usr/local/bin/${C} ]; then
      echo "Installing colorgcc wrapper in $HOME/bin for ${C}... "
      sudo ln -s /usr/bin/colorgcc /usr/local/bin/${C}
    fi
  done
fi

# generate .colorgcc by inserting ccache with .*: /usr/bin/.*  (Ryohei Ueda)
looks_like_path() {
    local arg=$1
    local result=`echo "$arg" | grep -c /`
    if [ "$result" != "0" ]; then
        echo true
        return 0
    else
        echo false
        return 1
    fi
}

# clear you .colorgccrc
if [ -e $HOME/.colorgccrc ]; then
    echo "clearing $HOME/.colorgccrc"
    echo > $HOME/.colorgccrc
fi

while read line
do
    if [ "`echo $line | grep -c -e '^[a-zA-Z0-9].*:'`" != "0" ]; then
        local second_arg="`echo $line | cut -f2 -d:`"
        if [ "`looks_like_path $second_arg`" = "true" ]; then
            
            echo `echo $line | sed "s/: /: ccache /g"` >> $HOME/.colorgccrc
        else 
            echo $line >> $HOME/.colorgccrc
        fi
    else
        echo $line >> $HOME/.colorgccrc
    fi
    
done < /etc/colorgcc/colorgccrc

### Color Make
#if [ -z "`which colormake`" ]; then
#  echo Installing ColorMake ...
#  sudo aptitude install colormake
#fi
#if [ ! -z "`which colormake`" ]; then
#  alias make='colormake'
#  if [ ! -e /usr/local/bin/make ]; then
#    sudo ln -s /usr/bin/colormake /usr/local/bin/make
#  fi
#fi
