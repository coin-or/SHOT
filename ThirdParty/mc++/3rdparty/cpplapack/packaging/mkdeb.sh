#!/bin/sh

######## base ########
version=`grep Version ./control | awk '{print $2}'`
name=cpplapack_"$version"_all
dir=/tmp/$name
rm -rf $dir
mkdir $dir

######## DEBIAN ########
mkdir $dir/DEBIAN
cp ./control $dir/DEBIAN

######## include/cpplapack ########
mkdir $dir/usr/
mkdir $dir/usr/include
mkdir $dir/usr/include/cpplapack
cp -r ../include/* $dir/usr/include/cpplapack
cat $dir/usr/include/cpplapack/cpplapack.h | sed -e 's/\#include\ \"/\#include\ \"cpplapack\//g'\ > $dir/usr/include/cpplapack.h
rm $dir/usr/include/cpplapack/cpplapack.h
rm -rf `find $dir -type d -name .svn`
sudo chown -R root:root $dir

######## dpkg-deb ########
cd /tmp
rm -f $name.deb
dpkg -b $name
sudo rm -rf $dir
echo "/tmp/$name.deb was successfully created."

## sudo dpkg -i cpplapack
## sudo dpkg -r cpplapack
