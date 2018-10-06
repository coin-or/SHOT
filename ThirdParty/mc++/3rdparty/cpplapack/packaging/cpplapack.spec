%define prefix /usr
%define version 2010.03.27
%define sourcedir /home/yuki/local/cpplapack

###############################################################################
Summary: CPPLapack library header files
Name: cpplapack
Version: %{version}
Release: 1
Vendor: Yuki ONISHI
URL: http://sourceforge.net/projects/cpplapack/
Source: http://sourceforge.net/projects/cpplapack/files/
#Patch: 
License: GPL
Group: Applications/Engineering
#Packager: 
BuildArch: noarch
Buildroot: %{_tmppath}/%{name}-root

#%package doc
#Summary: Documentation of CPPLapack
#Group: Applications/Engineering

%description
CPPLapack is a C++ matrix library designed as a class wrapper for BLAS and LAPACK.
Visit http://cpplapack.sourceforge.net/ to obtain the documentation, update information, and the latest version in the subversion (svn) repository.

#%description doc
#This package contains the html documentation, sample Makefies, 
#test programs, and benchmark programs of CPPLapack.
#Please visit http://cpplapack.sourceforge.net/ to check update 
#information and to obtain the latest version.

###############################################################################
%prep
echo "%prep"
rm -rf $RPM_BUILD_ROOT

%build
echo "%build"
echo Nothing to make since CPPLapack is a set of header files.

%install
echo "%install"
mkdir -p $RPM_BUILD_ROOT/%{prefix}/include/cpplapack
cd $RPM_BUILD_ROOT/%{prefix}/include
cp -r %{sourcedir}/include/* cpplapack/
rm -rf `find -type d -name .svn`
cat cpplapack/cpplapack.h\
 | sed -e 's/\#include\ \"/\#include\ \"cpplapack\//g'\
 > ./cpplapack.h
rm cpplapack/cpplapack.h

%clean
echo "%clean"
#rm -rf $RPM_BUILD_ROOT

###############################################################################
%files
%defattr(-,root,root)
/usr/include/cpplapack
/usr/include/cpplapack.h

#%files doc
#%doc benchmark doc makefiles test
