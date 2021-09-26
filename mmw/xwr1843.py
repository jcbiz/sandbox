import os
import sys
try:
    import serial
    import time
    import numpy as np
    from pyqtgraph.Qt import QtCore, QtGui
    import pyqtgraph.opengl as gl
    import pyqtgraph as pg
except ImportError as e:
    print(e, file=sys.stderr, flush=True)
    sys.exit(3)

# Change the configuration file name
#configFileName = 'AWR1843config.cfg'
#configFileName = 'ss.cfg'
#configFileName = 'profile_3d.cfg'
configFileName = 'profile_3d.1.cfg'

BYTE_VEC_ACC_MAX_SIZE = 2**15

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(BYTE_VEC_ACC_MAX_SIZE, dtype = 'uint8')
byteBufferLength = 0

# word array to convert 4 bytes to a 32 bit number
word = [1, 2**8, 2**16, 2**24]
short = [1, 2**8]

# message output types
MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1
MMWDEMO_OUTPUT_MSG_RANGE_PROFILE = 2
MMWDEMO_OUTPUT_MSG_NOISE_PROFILE = 3
MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP = 4
MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP = 5
MMWDEMO_OUTPUT_MSG_STATS = 6
MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO = 7
MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP = 8
MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS = 9
# Magic pattern
magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
# antenna counts
# Hard code the number of antennas, change if other configuration is used
numRxAnt = 4
numTxAnt = 3
numVAnt = numRxAnt * numTxAnt

# QtAPPfor the plot
qt = None
widgetDetectedPoints = None

# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Windows
    CLIport = serial.Serial('/dev/ttyS4', 115200)
    Dataport = serial.Serial('/dev/ttyS5', 921600)

    # make sure it stops, and dran the data
    CLIport.write(('sensorStop\n').encode())
    time.sleep(1)
    Dataport.read(Dataport.in_waiting)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for line in config:
        # skip empty lines and comments
        sline = line.strip()
        if len(sline) == 0 or sline.startswith('%'):  continue
        # write the command
        CLIport.write((line+'\n').encode())
        print(line)
        time.sleep(0.02)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to count set bits in an integer
def countSetBits(n):
    sets = 0
    while (n):
        sets += n & 1
        n >>= 1
    return sets

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2
                
            digOutSampleRate = int(splitWords[11])
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = float(splitWords[5])

        # Get the information about the Tx/Rx antenna 
        elif "channelCfg" in splitWords[0]:
            rxChannelEn = int(splitWords[1])
            txChannelEn = int(splitWords[2])
            cascading = int(splitWords[3])
            numRxAnt = countSetBits(rxChannelEn)
            numTxAnt = countSetBits(txChannelEn)
            numVAnt = numRxAnt * numTxAnt
            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame // numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)

    return configParameters
   
# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseData18xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12

    tlvHeaderLengthInBytes = 8
    pointLengthInBytes = 16
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    frameNumber = 0
    # this is a variable to store object attributes from different data types
    dataList = []
    detObj = {"type": MMWDEMO_OUTPUT_MSG_DETECTED_POINTS, "numObj": 0}
    mapStats = {"type": MMWDEMO_OUTPUT_MSG_STATS}
    
    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < BYTE_VEC_ACC_MAX_SIZE:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
        
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc+8]
            if np.all(check == magicWord):
                startIdx.append(loc)
               
        # Check that startIdx is not empty
        if startIdx:
            
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength-startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
                
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                

            
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[12:12+4], word)
            
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # Initialize the pointer index
        idX = 0
        
        # Read the header
        magicNumber = byteBuffer[idX:idX+8]

        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX+4],word),'x')
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        timeCpuCycles = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numDetectedObj = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        numTLVs = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX:idX+4],word)
        idX += 4

        # Read the TLV messages
        for tlvIdx in range(numTLVs):
            # Check the header of the TLV message
            tlv_type = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4
            tlv_length = np.matmul(byteBuffer[idX:idX+4],word)
            idX += 4

            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
                #print(f'tlv_type: List of detected points / len: {tlv_length}')

                # Initialize the arrays
                x = np.zeros(numDetectedObj,dtype=np.float32)
                y = np.zeros(numDetectedObj,dtype=np.float32)
                z = np.zeros(numDetectedObj,dtype=np.float32)
                velocity = np.zeros(numDetectedObj,dtype=np.float32)
                endPos = idX + tlv_length
                
                for objectNum in range(numDetectedObj):
                    # Read the data for each object
                    if idX + 4 > endPos:  break
                    x[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    if idX + 4 > endPos:  break
                    y[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    if idX + 4 > endPos:  break
                    z[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    if idX + 4 > endPos:  break
                    velocity[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                idX = endPos

                # Store the data in the detObj dictionary
                detObj["x"] = x
                detObj["y"] = y
                detObj["z"] = z
                detObj["velocity"] = velocity
                if detObj["numObj"] == 0 and numDetectedObj != 0:
                    detObj["numObj"] = numDetectedObj
                    dataList.append(detObj)
 
            elif tlv_type == MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
                #print(f'tlv_type: Range profile / len: {tlv_length}')

                # Get the number of bytes to read - Length: (Range FFT size) x (size of uint16_t)
                payload = byteBuffer[idX:idX + tlv_length]
                idX += tlv_length
                numBytes = configParameters["numRangeBins"] * 2
                #print(f'  numBytes: {numBytes}')
                rangeFFT = payload.copy(order='C')
                if numBytes > tlv_length:
                    rangeFFT = np.pad(rangeFFT, (0, numBytes - tlv_length), 'constant', constant_values=(0))

                # Convert it into 1D uint16 rangeFFT array
                rangeFFT = rangeFFT.astype(np.uint16)
                # Create the data for return
                mapRangeFFT = {"type": tlv_type, "numRangeBin": configParameters["numRangeBins"], "matrix": rangeFFT}
                dataList.append(mapRangeFFT)

            elif tlv_type == MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
                #print(f'tlv_type: Noise floor profile / len: {tlv_length}')

                # Get the number of bytes to read - Length: (Range FFT size) x (size of uint16_t)
                payload = byteBuffer[idX:idX + tlv_length]
                idX += tlv_length
                numBytes = configParameters["numRangeBins"] * 2
                #print(f'  numBytes: {numBytes}')
                noiseFFT = payload.copy(order='C')
                if numBytes > tlv_length:
                    noiseFFT = np.pad(noiseFFT, (0, numBytes - tlv_length), 'constant', constant_values=(0))

                # Convert it into 1D uint16 noiseFFT array
                noiseFFT = noiseFFT.astype(np.uint16)
                # Create the data for return
                mapNoiseFFT = {"type": tlv_type, "numRangeBin": configParameters["numRangeBins"], "matrix": noiseFFT}
                dataList.append(mapNoiseFFT)

            elif tlv_type == MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP:
                '''
                Imag(ant 0, range 0), Real(ant 0, range 0),...,Imag(ant N-1, range 0),Real(ant N-1, range 0)
                    ...
                Imag(ant 0, range R-1), Real(ant 0, range R-1),...,Imag(ant N-1, range R-1),Real(ant N-1, range R-1)
                '''
                #print(f'tlv_type: Samples to calculate static azimuth heatmap / len: {tlv_length}')
                
                # Get the number of bytes to read - Length: (Range FFT size) x (Number of virtual antennas) (size of cmplx16ImRe_t_) - 2 int16
                payload = byteBuffer[idX:idX + tlv_length]
                idX += tlv_length
                #staticAzimuth = payload.copy(order='C')
                numBytes = configParameters["numRangeBins"] * numVAnt * 4
                #print(f'  numBytes: {numBytes}')

                staticAzimuth = payload.copy(order='C')
                if numBytes > tlv_length:
                    staticAzimuth = np.pad(staticAzimuth, (0, numBytes - tlv_length), 'constant', constant_values=(0))

                # Convert the Azimuth array to a 4D matrix
                staticAzimuth = np.reshape(staticAzimuth, (configParameters["numRangeBins"], numVAnt, 2, 2), 'C')
                # Create the data for return
                mapStaticAzimuth = {"type": tlv_type, "numRangeBin": configParameters["numRangeBins"], "numVAnt": numVAnt, "matrix": staticAzimuth}
                dataList.append(mapStaticAzimuth)

            elif tlv_type == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
                '''
                X(range bin 0, Doppler bin 0),...,X(range bin 0, Doppler bin D-1),
                    ...
                X(range bin R-1, Doppler bin 0),...,X(range bin R-1, Doppler bin D-1)
                '''
                #print(f'tlv_type: Range/Doppler detection matrix / len: {tlv_length}')

                # Get the number of bytes to read - Length: (Range FFT size) x (Doppler FFT size) (size of uint16_t)
                numBytes = 2 * configParameters["numRangeBins"] * configParameters["numDopplerBins"]
                #print(f'  numBytes: {numBytes}')

                # Copy out the raw data
                payload = byteBuffer[idX:idX + numBytes]
                idX += tlv_length

                rangeDoppler = payload.copy(order='C')
                if numBytes > tlv_length:
                    rangeDoppler = np.pad(rangeDoppler, (0, numBytes - tlv_length), 'constant', constant_values=(0))

                # Convert the range doppler array to a 3D matrix
                rangeDoppler = np.reshape(rangeDoppler, (configParameters["numRangeBins"], configParameters["numDopplerBins"], 2), 'C')
                # Create the data for return
                mapRangeDoppler = {"type": tlv_type, "numRangeBin": configParameters["numRangeBins"], "numDopplerBin": configParameters["numDopplerBins"], "matrix": rangeDoppler}
                dataList.append(mapRangeDoppler)

            elif tlv_type == MMWDEMO_OUTPUT_MSG_STATS:
                '''
                uint32_t 	interFrameProcessingTime
 	                Interframe processing time in usec.
                uint32_t 	transmitOutputTime
                	Transmission time of output detection information in usec.
                uint32_t 	interFrameProcessingMargin
 	                Interframe processing margin in usec.
                uint32_t 	interChirpProcessingMargin
 	                Interchirp processing margin in usec.
                uint32_t 	activeFrameCPULoad
 	                CPU Load (%) during active frame duration.
                uint32_t 	interFrameCPULoad
 	                CPU Load (%) during inter frame duration.
                '''
                #print(f'tlv_type: Stats information / len: {tlv_length}')
                endPos = idX + tlv_length

                # Copy out the raw data
                if idX + 4 <= endPos:
                    mapStats["interFrameProcessingTime"] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                if idX + 4 <= endPos:
                    mapStats["transmitOutputTime"] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                if idX + 4 <= endPos:
                    mapStats["interFrameProcessingMargin"] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                if idX + 4 <= endPos:
                    mapStats["interChirpProcessingMargin"] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                if idX + 4 <= endPos:
                    mapStats["activeFrameCPULoad"] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                if idX + 4 <= endPos:
                    mapStats["interFrameCPULoad"] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                idX = endPos
                # add if this is the first record
                if "tempReport" not in mapStats:
                    dataList.append(mapStats)

            elif tlv_type == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
                #print(f'tlv_type: List of detected points side info / len: {tlv_length}')

                # Initialize the arrays
                snr = np.zeros(numDetectedObj,dtype=np.int16)
                noise = np.zeros(numDetectedObj,dtype=np.int16)
                endPos = idX + tlv_length

                for objectNum in range(numDetectedObj):
                    # Read the data for each object
                    if idX + 2 > endPos:  break
                    snr[objectNum] = byteBuffer[idX:idX + 2].view(dtype=np.int16)
                    idX += 2
                    if idX + 2 > endPos:  break
                    noise[objectNum] = byteBuffer[idX:idX + 2].view(dtype=np.int16)
                    idX += 2
                idX = endPos

                # Store the data in the detObjSInfo dictionary
                detObj["snr"] = snr
                detObj["noise"] = noise
                if detObj["numObj"] == 0 and numDetectedObj != 0:
                    detObj["numObj"] = numDetectedObj
                    dataList.append(detObj)

            elif tlv_type == MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP:
                #print(f'tlv_type: Samples to calculate static azimuth/elevation heatmap / len: {tlv_length}')
                idX += tlv_length

            elif tlv_type == MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS:
                '''
                int32_t 	tempReportValid
 	                retVal from API rlRfTempData_t - can be used to know if values in temperatureReport are valid
                rlRfTempData_t 	temperatureReport
                    detailed temperature report - snapshot taken just before shipping data over UART
                        rlUInt32_t time - radarSS local Time from device powerup. 1 LSB = 1 ms
                        rlInt16_t tmpRx0Sens - RX0 temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpRx1Sens - RX1 temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpRx2Sens - RX2 temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpRx3Sens - RX3 temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpTx0Sens - TX0 temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpTx1Sens - TX1 temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpTx2Sens - TX2 temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpPmSens - PM temperature sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpDig0Sens - Digital temp sensor reading (signed value). 1 LSB = 1 deg C
                        rlInt16_t tmpDig1Sens - Second digital temp sensor reading (signed value).( applicable only in xWR1642/xWR6843/xWR1843.); 1 LSB = 1 deg C
                '''
                #print(f'tlv_type: temperature stats from Radar front end / len: {tlv_length}')
                endPos = idX + tlv_length
                # Copy out the raw data
                if idX + 4 <= endPos:
                    tempReportValid = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                # insert the data only if it's valid (tempReportValid == 0)
                if tempReportValid == 0:
                    mapTempStats = {"type": MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS}
                    if idX + 4 <= endPos:
                        mapTempStats["time"] = np.matmul(byteBuffer[idX:idX + 4], word)
                        idX += 4
                    if idX + 2 <= endPos:
                        mapTempStats["tmpRx0Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpRx1Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpRx2Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpRx3Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpTx0Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpTx1Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpTx2Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpPmSens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpDig0Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    if idX + 2 <= endPos:
                        mapTempStats["tmpDig1Sens"] = np.matmul(byteBuffer[idX:idX + 2], short)
                        idX += 2
                    mapStats["tempReport"] = mapTempStats
                idX = endPos
                # add if this is the first record
                if "interFrameProcessingTime" not in mapStats:
                    dataList.append(mapStats)

            else:
                print(f'tlv_type: Unknown {tlv_type} / len: {tlv_length}')
                break

        # Remove already processed data
        if idX > 0 and byteBufferLength>idX:
            shiftSize = totalPacketLen
                
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
            
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0         

    return dataList

# ------------------------------------------------------------------

def plotDetectedPointsGraph(data):
    if not hasattr(plotDetectedPointsGraph,"sp"):
        plotDetectedPointsGraph.sp = [None, None] # keep 2 copies
        plotDetectedPointsGraph.spIndex = 0;

    global widgetDetectedPoints
    if widgetDetectedPoints == None:
        widgetDetectedPoints = gl.GLViewWidget()
        widgetDetectedPoints.opts['distance'] = 20 # distance of camera from center
        widgetDetectedPoints.opts['fov'] = 60  # horizontal field of view in degrees
        widgetDetectedPoints.opts['azimuth'] = -75 # camera's azimuthal angle in degrees, 仰俯角
        widgetDetectedPoints.opts['elevation'] = 30 # camera's angle of elevation in degrees, 方位角
        widgetDetectedPoints.setGeometry(100, 100, 800, 800)
        widgetDetectedPoints.show()
        widgetDetectedPoints.setWindowTitle('Detected Points')
        gridX = gl.GLGridItem()
        gridX.rotate(0, 0, 1, 0)
        gridX.translate(0, 0, 0)
        widgetDetectedPoints.addItem(gridX)
        # center dot
        pos = np.array([0.0, 0.0, 0.0])
        size = np.array([0.2])
        color = np.array([1.0, 0.0, 0.0, 0.5])
        center  = gl.GLScatterPlotItem(pos=pos, size=size, color=color, pxMode=False)
        widgetDetectedPoints.addItem(center)
        # axis
        axis = gl.GLAxisItem(antialias=True, glOptions='translucent')
        axis.setSize(x=10, y=10, z=10)
        widgetDetectedPoints.addItem(axis)

    if data["type"] == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS and len(data["x"]) > 0:
        x = data["x"]
        y = data["y"]
        z = data["z"]
        v = data["velocity"]
        s = data["snr"]
        n = data["noise"]

        pos = np.empty((len(data["x"]), 3))
        size = np.empty(len(data["x"]))
        color = np.empty((len(data["x"]), 4))
        for i in range(len(x)):
            pos[i] = (x[i], y[i], z[i])
            size[i] = s[i] / 2000
            if v[i]>= 0:
                color[i] = (0.0, v[i], 0.0, 1.0) #color (r,g,b,a)
            else:
                color[i] = (-v[i], 0.0, 0.0, 1.0) #color (r,g,b,a)

        sp1 = gl.GLScatterPlotItem(pos=pos, size=size, color=color, pxMode=False)
        #sp1.translate(0, 0, 0)

        if plotDetectedPointsGraph.sp[plotDetectedPointsGraph.spIndex] != None:
            sp2 = plotDetectedPointsGraph.sp[plotDetectedPointsGraph.spIndex]
            widgetDetectedPoints.removeItem(sp2)

        plotDetectedPointsGraph.sp[plotDetectedPointsGraph.spIndex] = sp1
        widgetDetectedPoints.addItem(sp1)
        plotDetectedPointsGraph.spIndex = (plotDetectedPointsGraph.spIndex + 1) % len(plotDetectedPointsGraph.sp)


# Function to dump numpy matrix
def printData(data):
    #print(data)
    return None

# Function to dump objects into CSV
def printDetectedPointsCSV(data):
    x = data["x"]
    y = data["y"]
    z = data["z"]
    v = data["velocity"]
    s = data["snr"]
    n = data["noise"]
    ts = time.time() * 1000
    for i in range(len(x)):
        print(f"Point,{ts},{x[i]},{y[i]},{z[i]},{v[i]},{s[i]},{n[i]}")

# Funtion to update the data and display in the plot
def update():
    x = []
    y = []
    z = []

    # Read and parse the received data
    dataList = readAndParseData18xx(Dataport, configParameters)
    for data in dataList:
        if data["type"] == MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
            printData(data)
            plotDetectedPointsGraph(data)
            #printDetectedPointsCSV(data)

        elif data["type"] == MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
            printData(data)

        elif data["type"] == MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
            printData(data)

        elif data["type"] == MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP:
            printData(data)

        elif data["type"] == MMWDEMO_OUTPUT_MSG_STATS:
            printData(data)

        elif data["type"] == MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
            printData(data)
            '''
            # Generate the range and doppler arrays for the plot
            rangeArray = np.array(range(configParameters["numRangeBins"]))*configParameters["rangeIdxToMeters"]
            dopplerArray = np.multiply(np.arange(-configParameters["numDopplerBins"]/2 , configParameters["numDopplerBins"]/2), configParameters["dopplerResolutionMps"])

            plt.clf()
            cs = plt.contourf(rangeArray,dopplerArray,rangeDoppler)
            fig.colorbar(cs, shrink=0.9)
            fig.canvas.draw()
            plt.pause(0.1)
            '''

    return len(dataList)


# START QtAPPfor the plot if None
def startQt():
    global qt
    if qt == None:
        qt = QtGui.QApplication([])
        pg.mkQApp()
        #pg.setConfigOption('background','w')
        t = QtCore.QTimer()
        t.timeout.connect(update)
        t.start(50)
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()


# -------------------------    MAIN   -----------------------------------------  
if __name__ == '__main__':
    # Configurate the serial port
    CLIport, Dataport = serialConfig(configFileName)

    # Get the configuration parameters from the configuration file
    configParameters = parseConfigFile(configFileName)

    # Main loop 
    try:
        startQt()
        '''
        while True:
            # Update the data and check if the data is okay
            dataCount = update()

            time.sleep(0.05) # Sampling frequency of 30 Hz
        '''

    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())

    # clean up
    CLIport.write(('sensorStop\n').encode())
    CLIport.close()
    Dataport.close()
