#!/bin/sh

dostats () {
    PYFILES=$(find $1 -type f -name '*.py' | grep -v /breathe/)
    if [ -n "$PYFILES" ] ; then
        NPYFILES=$(echo $PYFILES | wc -w)
        NPYLINES=$(wc -l $PYFILES | tail -1)
        echo "$NPYFILES Python files, $NPYLINES lines"
    fi

    CPPFILES=$(find $1 -type f -name '*.cpp' -or -name '*.hpp')

    if [ -n "$CPPFILES" ] ; then
        NCPPFILES=$(echo $CPPFILES | wc -w)
        NCPPLINES=$(wc -l $CPPFILES | tail -1)
        echo "$NCPPFILES C++ files, $NCPPLINES lines"
    fi
}

echo "Total:"
dostats $1

echo "Testing:"
dostats $1/test

echo "Examples:"
dostats $1/samples

echo "Source:"
dostats $1/src

echo "Headers:"
dostats $1/include

echo "Python"
dostats $1/python