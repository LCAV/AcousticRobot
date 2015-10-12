#!/bin/bash

top=$(<top.html)
bottom=$(<bottom.html)
echo "text to be copied:"
echo "$top"
echo "$bottom"

for f in *.html 
do
    echo "removing old results..."
    rm test/$f
    echo "file content before insertion:"
    cat $f
    echo "copying into new file..."
    echo "$top" >> test/$f
    cat $f >> test/$f
    echo "$bottom" >> test/$f
done


