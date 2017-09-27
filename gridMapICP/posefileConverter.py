"""
Parser for pose-graph (.mat-->.g2o and .g2o-->.mat)
"""
import sys
import csv
import scipy.io as sio
import numpy as np

def setFlag(arglist):
	"""
	Distinguish the file and transformation type
	"""
	if 'result' in arglist[1] and '.g2o' in arglist[1]:
		cvtflag = 'g2o2mat'
	else:
		cvtflag = 'mat2g2o'
	return cvtflag


outputName = ''
cvtflag = ''

# Input file selection
if len(sys.argv) == 3:
	cvtflag = setFlag(sys.argv)
	outputName = sys.argv[2]
elif len(sys.argv) == 2:
	cvtflag = setFlag(sys.argv)
	if cvtflag == 'mat2g2o':
		outputName = sys.argv[1].replace('.mat','.g2o')
	elif cvtflag == 'g2o2mat':
		outputName = sys.argv[1].replace('.g2o','.mat')
else:
	print 'usage: python mat2g2o.py InputFile [OutputFile]'
	print 'InputFile: .mat file'
	print 'OutputFile: .g2o file'
	quit()

if cvtflag == 'mat2g2o':
	# Read in file
	matdata = sio.loadmat(sys.argv[1]) 
	vertex = matdata['vertex']
	edges = matdata['edges']

	print "Convert from .mat to .g2o ..."

	with open(outputName,'wb') as csvfile:
		csvwriter = csv.writer(csvfile,delimiter=' ')

		# Rows of vertex
		v_header = np.tile(['VERTEX_SE3:QUAT'],(vertex.shape[0],1))
		vertexrows = np.hstack((v_header,vertex))
		csvwriter.writerows(vertexrows)
		# Rows of edges
		e_header = np.tile(['EDGE_SE3:QUAT'],(edges.shape[0],1))
		infoarr = np.array([10000,0,0,0,0,0,10000,0,0,0,0,10000,0,0,0,40000,0,0,40000,0,40000])
		infomat = np.tile(infoarr,(edges.shape[0],1))
		edgesrows = np.hstack((e_header,edges,infomat))
		csvwriter.writerows(edgesrows)
elif cvtflag == 'g2o2mat':
	vertex = []
	edges  = []
	# Read in file
	with open(sys.argv[1]) as g2ofile:
		csvreader = csv.reader(g2ofile,delimiter=' ')
		for row in csvreader:
			if 'VERTEX_SE3:QUAT' in row:
				vertex.append([int(row[1])]+map(float,row[2:5])+[float(row[8])]+map(float,row[5:8]))
			elif 'EDGE_SE3:QUAT' in row:
				edges.append(map(int,row[1:3])+map(float,row[3:6])+[float(row[9])]+map(float,row[6:9]))

		print "Convert from .g2o to .mat ..."
		sio.savemat(outputName,{'vertex':vertex,'edges':edges})

print 'Done.'