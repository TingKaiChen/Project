#!/bin/bash
g++ -o readData readData.cpp
./readData $1
fn=$1
txtFileName="${fn/cpev/cpev.txt}"
echo $txtFileName
python parser.py $txtFileName

loadName="${fn/cpev/csv}"
matName="${fn/cpev/mat}"
echo "Making" $matName
/usr/local/MATLAB/R2017a/bin/matlab \
	-nosplash -nodesktop -r "parser('$loadName','$matName')"
echo "Done.  " $fn