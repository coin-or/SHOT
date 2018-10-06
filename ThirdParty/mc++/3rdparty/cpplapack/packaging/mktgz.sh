#!/bin/sh

version=2010.03.27

name=cpplapack-$version
dir=/tmp/$name
mkdir $dir
cp -r ../* $dir
rm -rf `find $dir -type d -name .svn`

cd /tmp
rm -f $name.tar.gz
tar czf $name.tar.gz $name
rm -rf $dir
echo "/tmp/$name.tar.gz was successfully created."
