#!/bin/bash

echo "reading top and bottom html files..."
top=$(<top.html)
bottom=$(<bottom.html)
for f in *.html 
do
    echo "removing old results..."
    rm pages/$f
    echo "copying into new file..."
    echo "$top" >> pages/$f
    cat $f >> pages/$f
    echo "$bottom" >> pages/$f
done


