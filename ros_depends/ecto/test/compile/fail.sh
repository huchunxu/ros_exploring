#!/bin/sh 

$*

if [ "$?" = 0 ] ; then
    echo "didn't fail as expected"
    exit 1
else
    exit 0
fi


