#!/bin/bash

if [ -z $1 ]; then src="arcsys2"; else src=$(echo $1 | sed -e "s/\.[^.]*$//g"); fi
pkgdir=$(cd "$(dirname $0)"; pwd)

rosrun xacro xacro.py ${pkgdir}/urdf/${src}.xacro > ${pkgdir}/urdf/${src}.urdf && check_urdf ${pkgdir}/urdf/${src}.urdf

echo "launch? [y/n]" && read yn

case $yn in
  "y") roslaunch manipulator manipulator.launch ;;
  *) exit 0 ;;
esac
