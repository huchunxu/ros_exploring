#!/bin/sh 

$*

if [ "$?" = 0 ] ; then
    exit 0
else
    echo "didn't pass as expected"
    exit 1
fi


