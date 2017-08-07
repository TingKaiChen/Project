#!/bin/bash
fn=$1
csvName=$2
echo $2
python parser_v17.py $1 $2

matName="${2/csv/mat}"
echo "Making" $matName
~/Matlab/bin/matlab \
	-nosplash -nodesktop -r "parser('$2','$matName')"
echo "Done.  " $3