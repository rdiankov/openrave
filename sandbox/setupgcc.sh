#!/bin/bash
if [ -z "`which ccache`" ]; then
  echo Installing ColorGCC ...
  sudo aptitude install ccache
fi

### Color GCC
if [ -z "`which colorgcc`" ]; then
  echo Installing ColorGCC ...
  sudo aptitude install colorgcc
fi

if [ ! -z "`which colorgcc`" ]; then
  export CC="colorgcc"
  alias gcc='colorgcc'
  #cp /etc/colorgcc/colorgccrc $HOME/.colorgccrc
  for C in `grep /usr/bin /etc/colorgcc/colorgccrc | sed -e 's/# //' -e 's/:.*//'`; do
    if [ ! -e /usr/local/bin/${C} ]; then
      echo "Installing colorgcc wrapper in $HOME/bin for ${C}... "
      sudo ln -s /usr/bin/colorgcc /usr/local/bin/${C}
    fi
  done
fi

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
