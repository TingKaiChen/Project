"""
Parser of 2017 data
"""
import sys
import csv

def lidar2list(dataStr):
    """
    Parse data string of LiDAR into integer list
    """
    if len(dataStr)<10:		# Empty input
        return [0,0]
    else:
        intdata = dataStr.strip().split(' ')
        del intdata[0:3]	# Remove the first 3 items
        return map(int, intdata)
def degAndVal(lidarData):
    """
    Return a tuple of two list: (degList, valList)
    where degList is a list of degree list,
    and valList is a list of value list
    """
    degList = []
    valList = []
    cutIndex = 0
    for i in xrange(len(lidarData)/2):
        degList.append(lidarData[2*i])
        valList.append(lidarData[2*i+1])
    return (degList, valList)
def imuParser(imuData):
    """
    Return a list of IMU data
    """
    imuData = imuData.strip('\n').split(',')
    return map(float, imuData)
def gpsParser(gpsData):
    """
    Return a list of GPS data (number only)
    """
    gpsData = gpsData.strip('\n').split(',')
    # print gpsData
    return [float(gpsData[i]) for i in [1,2,4,6,7,8,9,11]]

fileName = ""
outputName = ""
# Input file selection
if len(sys.argv) == 3:
    fileName = sys.argv[1]
    outputName = sys.argv[2]
elif len(sys.argv) == 2:
    fileName = sys.argv[1]
    outputName = sys.argv[1].replace('txt', 'csv')
else:
    print 'usage: python parser_v17.py fileName [outfile]'
    quit()

rowdata = []	# List of every row of the file

# Read in file
with open(fileName, mode='r') as cpevData:
    rowdata = cpevData.readlines()

# # Delete the incomplete data
# while "$GPHDT" not in rowdata[1]:
# 	del rowdata[0]

# Write in a CSV file
# imuName = outputName.replace('.csv', '_imu.csv')
# gpsName = outputName.replace('.csv', '_gps.csv')

csvFile = open(outputName, 'wb')
# imufile = open(imuName, 'wb') 
# gpsfile = open(gpsName, 'wb')
writer = csv.writer(csvFile)
# imucsv = csv.writer(imufile)
# gpscsv = csv.writer(gpsfile)

# gpsdata = []
for i in range(len(rowdata)):
    lidarData = lidar2list(rowdata[i])
    degList, valList = degAndVal(lidarData)
    writer.writerow(degList)
    writer.writerow(valList)

csvFile.close()
# imufile.close()
# gpsfile.close()



