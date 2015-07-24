#!/bin/bash

for i in 1 5 10 20 50 100 150 200 2501
do
	python houghcircles.py -i test.jpg -p2 $i

done

