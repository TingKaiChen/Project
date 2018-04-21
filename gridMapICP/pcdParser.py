import sys
import csv
import scipy.io as sio
import numpy as np

outputName = ['','']

# File input
if len(sys.argv) == 2:
	outputName[0] = sys.argv[1].replace('.mat','_target.pcd')
	outputName[1] = sys.argv[1].replace('.mat','_source.pcd')
else:
	print 'usage: python pcdParser.py InputFile'
	print 'InputFile: .mat file'
	quit()


print "Convert from .mat to .pcd ..."

# Read in file
matdata = sio.loadmat(sys.argv[1]) 

# Target point cloud
header = [['#','.PCD','v0.7','-','Point','Cloud','Data','file','format'],
		  ['VERSION',0.7],
		  ['FIELDS','x','y','z'],
		  ['SIZE',4,4,4],
		  ['TYPE','F','F','F'],
		  ['COUNT',1,1,1],
		  ['WIDTH',len(matdata['P_tar'])],
		  ['HEIGHT',1],
		  ['VIEWPOINT',0,0,0,1,0,0,0],
		  ['POINTS',len(matdata['P_tar'])],
		  ['DATA','ascii']]

with open(outputName[0],'wb') as csvfile:
	csvwriter = csv.writer(csvfile,delimiter=' ')
	csvwriter.writerows(header)
	csvwriter.writerows(matdata['P_tar'])

# Source point cloud
header = [['#','.PCD','v0.7','-','Point','Cloud','Data','file','format'],
		  ['VERSION',0.7],
		  ['FIELDS','x','y','z'],
		  ['SIZE',4,4,4],
		  ['TYPE','F','F','F'],
		  ['COUNT',1,1,1],
		  ['WIDTH',len(matdata['P_src_origin'])],
		  ['HEIGHT',1],
		  ['VIEWPOINT',0,0,0,1,0,0,0],
		  ['POINTS',len(matdata['P_src_origin'])],
		  ['DATA','ascii']]
		  
with open(outputName[1],'wb') as csvfile:
	csvwriter = csv.writer(csvfile,delimiter=' ')
	csvwriter.writerows(header)
	csvwriter.writerows(matdata['P_src_origin'])

print 'Done.'
