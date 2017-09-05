#!/bin/bash
#
# Installs SHOT binary as solver in GAMS (Windows).
# Usage: ./gamsinstwin.sh /path/to/gamssysdir /path/to/SHOTbinary

gamspath="$1"
shotbin="${2:-SHOT.exe}"

if test -z "$gamspath" ; then
  echo "Need to get path of GAMS system as first argument."
  exit 1
fi

gmscmp=${gamspath}/gmscmpNT.txt
gmscmporig="${gmscmp}.orig"
gmscmpbak="${gmscmp}.bak"

if ! test -r "$gmscmp" ;
then
   echo "File $gmscmp not found or not readable, cannot edit."
   exit 1
fi

if ! test -e "$shotbin"
then
   echo "Solver binary $shotbin not found, cannot install."
   exit 1
fi

echo "Adding or updating entry for SHOT into $gmscmp."

# keep backup of original gmscmpun.txt file
if ! test -r "$gmscmporig"
then
   cp "$gmscmp" "$gmscmporig"
fi

# keep backup of current gmscmpun.txt file
cp -f "$gmscmp" "$gmscmpbak"

awk '
BEGIN {
   fileType      = 111;
   dictType      = 5;
   licCodes      = "0001020304";
   defaultOkFlag = 1;
   hiddenFlag    = 0;
   scriptCmd = "gmssh_nt.cmd";
   execCmd   = "SHOT.exe";

   written["SHOT"] = 0;
   libid["SHOT"] = "sh_";
   dicttype["SHOT"] = 5;
   modeltypes["SHOT"] = "MINLP MIQCP";

   startBlock = 0;
}

function writeConfig(solverID) {
   print solverID, fileType, dicttype[solverID], licCodes, defaultOkFlag, hiddenFlag, "1", modeltypes[solverID];
   print scriptCmd;
   print execCmd;
   written[solverID] = 1;
}

(/^\*/ || /^ *$/) { print $0 }

/^DEFAULTS/ {
   for( solverID in written )
      if( !written[solverID] )
      {
         writeConfig(solverID)
         print "";
      }
   print;
   next;
}

!(/^*/ || /^ *$/) {
   if( startBlock < 0 )
   {
      startBlock++;
      next;
   }
   if( $1 in written && !written[$1] )
   {
      writeConfig($1)
      startBlock = -($7+1);
      next;
   }
   print;
}
' "$gmscmpbak" > "$gmscmp"

cat > "${gamspath}/gmssh_nt.cmd" <<EOF
@echo off
SHOT.exe "%~4"
if not %errorlevel% == 0 echo err: solver rc %errorlevel% 1>&2
EOF
chmod +x "${gamspath}/gmssh_nt.cmd"

cp ${shotbin} "${gamspath}/SHOT.exe"
