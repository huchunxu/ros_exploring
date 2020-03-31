#!/bin/sh -ex

TOP=$(cd `dirname $0` ; pwd)
echo "TOP=$TOP"

for i in $*
do
    echo $i
    pushd $i
    for d in *.dot
    do
        dot -Tsvg $d -o$(basename $d .dot).svg
        LAST=$(basename $d .dot)
    done
    sed -e "s/MAXINDEX/$LAST/" $TOP/viewer.html > index.html
    popd
done

