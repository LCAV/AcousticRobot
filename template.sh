#!/bin/bash

echo "reading top and bottom html files..."
top=$(<top.html)
bottom=$(<bottom.html)
for f in *.html 
do
    echo "$f"
    rm pages/$f
    echo "$top" >> pages/$f
    cat $f >> pages/$f
    echo "$bottom" >> pages/$f
done
echo "saved above files in pages/"

