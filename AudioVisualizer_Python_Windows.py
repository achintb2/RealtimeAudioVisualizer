# Achint's Python3 Arduino FastLED Audio Visualizer File
# This code is a mess! Will implement it properly with classes in a future update!
# Also plan to design a fully functional GUI, with a lot more control in future :)

import pyaudio
import time
import numpy as np

import scipy.signal as signal
import serial as ps
import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
print("\nHere is a list of all Serial devices and their COM ports:")
devList = []
for port, desc, hwid in sorted(ports):
    print("{}: {}".format(port, desc, hwid))
    devList.append("{}: {}".format(port, desc, hwid))

print("\nEnter Arduino's COM Port!")
print("Example: If COM port is 'com12' then enter: com12")
comPort = str(input("Enter COM port now:\n"))

while True:
    try:
        _ = ps.Serial(comPort, 128000)
        _ = None
        break
    except(AttributeError):
        estr = ""
        estr += "Serial Error! \n"
        estr += "MAKE SURE YOU STARTED IDLE WITH ANACONDA COMMAND PROMPT!\n"
        estr += "To do that, press START, type 'anaconda', and enter 'idle' in it!\n\n"
        estr += "IF YOU HAVE ALREADY STARTED IDLE THAT WAY, then try these steps:\n"
        estr += "Please uninstall both PySerial and Serial"
        estr += "and then, reinstall pyserial only.\n"
        estr += "Uninstall Serial: 'pip uninstall serial'\n"
        estr += "Uninstall PySerial: 'pip uninstall pyserial'\n"
        estr += "Install PySerial again: 'pip install pyserial'\n"
        print(estr)
        raise AttributeError
    except(FileNotFoundError):
        print(e)
        estr = ""
        estr += "FileNotFoundError: System cannot find the file specified."
        estr += "The COM port you entered, does not exist"
        print(estr)
        continue

import colorsys

from scipy.interpolate import interp1d
from scipy.io.wavfile import read, write
from scipy.signal import butter, sosfilt, sosfreqz

from time import sleep
from datetime import datetime

from ctypes import cast, POINTER
from comtypes import CLSCTX_ALL

from scipy.signal import butter, lfilter, freqz

# ----------------------------------- Space Formatting Function ----------------------------------- #

# This function will add spaces to any number provided into it, so that the output looks clean.
# Will be replaced when I integrate GUI into the entire program but for now, this helps! :)

def space_formatting(theNumber):
    '''Will add spaces before a number so output looks clean. Supports max 3 digits so far.'''
    theNumber = str(theNumber)
    if(len(theNumber) < 3):
        howShort = 3-len(theNumber)
        theString = " "*howShort + theNumber
        return(theString)
    else:
        return(theNumber)

def dynamicSpaceFormattingBefore(theValue, maxLength):
    '''Will add spaces before anything so output looks clean.'''
    theValue = str(theValue)
    if(len(theValue) < maxLength):
        howShort = maxLength-len(theValue)
        theString = " "*howShort + theValue
        return(theString)
    else:
        return(theValue)

def dynamicSpaceFormattingAfter(theValue, maxLength):
    '''Will add spaces before anything so output looks clean.'''
    theValue = str(theValue)
    if(len(theValue) < maxLength):
        howShort = maxLength-len(theValue)
        theString = theValue + " "*howShort
        return(theString)
    else:
        return(theValue)
# ----------------------------------- Updating Scaler Function ----------------------------------- #

def returnScaler(rAudioValue):
    '''Returns a value multiplied by scaling factor: 12.75x by default. Can change if updated!'''
    return(rAudioValue * 12.75)


timeList = []
timeList.append(datetime.now())

# ----------------------------------- Test Sound Callback Function ----------------------------------- #

# A minimalistic function responsible for simply testing audio output.

def testSound(in_data, frame_count, time_info, flag):
    indata = np.fromstring(in_data, dtype=np.float32)
    vol = np.linalg.norm(indata)
    currentTime = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    print(f"{currentTime} :: {vol}")
    return(in_data, pyaudio.paContinue)

# ----------------------------------- Open Stream Function ----------------------------------- #

# This function is used as an easy and neat way to start a PyAudio stream.
# Arguments are self explanatory :)

def openStream(deviceIndexIn, numChannels, samplingRate, isOutputOn, isInputOn, callbackFunction, useLoopback):
    '''Used to open a PyAudio Stream to the selected device.'''
    try:
        temp = p
    except(NameError):
        p = pyaudio.PyAudio()
        print("PyAudio wasn't initialized yet. Initializing now...")
    stream = p.open (
                        format = pyaudio.paFloat32,
                        channels = numChannels,
                        rate = samplingRate,
                        input_device_index = deviceIndexIn,
                        output = isOutputOn,
                        input = isInputOn,
                        stream_callback = callbackFunction,
                        as_loopback = useLoopback
                    )
    print("Audio Stream Opened.")
    return(stream)

# ----------------------------------- PyAudio Device Info Functions ----------------------------------- #

# This function returns total number of audio devices, both input and output.
def checkMaxDevices():
    '''Returns total number of audio devices, both input and output.'''
    p = pyaudio.PyAudio()
    for i in range(100):
        try:
            rt = p.get_device_info_by_index(i)
        except(OSError):
            return(i)

# This function prints a list of all sound devices and their index,
# Trying best to fit that info in a console window.
def listAllDevices():
    for i in range(checkMaxDevices()+1):
        try:
            rt = p.get_device_info_by_index(i)
            maxIn = rt['maxInputChannels']
            maxOut = rt['maxOutputChannels']
            sample = rt['defaultSampleRate']
            devName = rt['name']
            print(f"{i} = Ch In: {maxIn} | Ch Out: {maxOut} | Hz: {sample} | Name: {devName}")
        except(OSError):
            return(i-1)
            break

# This function gives details about a specific device by index.
def selectSpecificDevice(devIndex):
    i = devIndex
    print("Selected Device is:")
    try:
        rt = p.get_device_info_by_index(i)
        maxIn = rt['maxInputChannels']
        maxOut = rt['maxOutputChannels']
        sample = rt['defaultSampleRate']
        devName = rt['name']
        print(f"{i} = Ch In: {maxIn} | Ch Out: {maxOut} | Hz: {sample} | Name: {devName}")
        outCHANNELS = maxOut
        inCHANNELS = maxIn
        RATE = 48000
        return([outCHANNELS, inCHANNELS, RATE, i])
    except Exception as e:
        print(f"ERROR: {e}")
        print("Occurred in listSpecificDevice Function")

# This function first lists all devices, then prompts user to give selected device's
# index. Then the samplerate and channel variables are automatically set accordingly.
def selectDeviceListAll():
    listAllDevices()
    print("\nEnter the device number to select a device!\n")
    devIndex = int(input("> "))
    print()
    parameters = selectSpecificDevice(devIndex)
    return(parameters)


# ----------------------------------- Check Devices Function ----------------------------------- #

def openTestStream(deviceIndexIn, numChannels, samplingRate, isOutputOn, isInputOn, callbackFunction, useLoopback):
    try:
        temp = tp
    except(NameError):
        tp = pyaudio.PyAudio()
    stream = tp.open(format = pyaudio.paFloat32,
                    channels = numChannels,
                    rate = samplingRate,
                    input_device_index = deviceIndexIn,
                    output = isOutputOn,
                    input = isInputOn,
                    stream_callback = callbackFunction,
                    as_loopback = useLoopback)
    return(stream)

# This function will:
# - Get a list of all device indexes
# - Check them all by opening an audio stream to each device.
# - Store a list of device index which caused an error to remember.

def getWorkingDevices():
    p = pyaudio.PyAudio()
    maxIndex = checkMaxDevices()
    workingDevicesIDs = []
    notWorkingDevices = []
    for j in range(maxIndex):
        try:
            tp = pyaudio.PyAudio()
            devIndex = j
            i = devIndex
            rt = tp.get_device_info_by_index(i)
            maxIn = rt['maxInputChannels']
            maxOut = rt['maxOutputChannels']
            sample = rt['defaultSampleRate']
            devName = rt['name']
            channels = max(int(maxIn), int(maxOut))
            sampleRate = rt['defaultSampleRate']
            stream = openTestStream(i, channels, int(sampleRate), False, True, testSound, True)
            stream.close()
            tp.terminate()
            workingDevicesIDs.append(j)
        except:
            notWorkingDevices.append(j)

    print("Device Check Complete")
    return([workingDevicesIDs, notWorkingDevices])

# This function will:
# - Get a list of all working device indexes
# - Display all devices that didn't cause an error.
# - Make sure that user selects a valid device.

def getAllDeviceNames():
    '''Get a list of all device names'''
    maxIndex = checkMaxDevices()
    ptemp = pyaudio.PyAudio()
    nameList = []
    for i in range(maxIndex):
        rt = p.get_device_info_by_index(i)
        devName = rt['name']
        nameList.append(devName)
    return(nameList)

def listWorkingDevices():
    check = getWorkingDevices()
    goodList = check[0]
    badList = check[1]
    print("These devices have been checked and tested, and are working fine!")
    maxDevNameLength = max([len(i) for i in getAllDeviceNames()])
    for i in range(checkMaxDevices()+1):
        if(i not in badList and i in goodList):
            rt = p.get_device_info_by_index(i)
            maxIn = rt['maxInputChannels']
            maxOut = rt['maxOutputChannels']
            sample = rt['defaultSampleRate']
            devName = rt['name']
            channels = max(int(maxIn), int(maxOut))
            fi = dynamicSpaceFormattingBefore(i, 2)
            fName = dynamicSpaceFormattingAfter(devName, maxDevNameLength)
            print(f"{fi} = Name: {fName} | Channels: {channels} | Hz: {sample}")

def listNotWorkingDevices():
    check = getWorkingDevices()
    goodList = check[0]
    badList = check[1]
    print("These devices have been checked and tested, and are working fine!")
    maxDevNameLength = max([len(i) for i in getAllDeviceNames()])
    for i in range(checkMaxDevices()+1):
        if(i in badList and i not in goodList):
            rt = p.get_device_info_by_index(i)
            maxIn = rt['maxInputChannels']
            maxOut = rt['maxOutputChannels']
            sample = rt['defaultSampleRate']
            devName = rt['name']
            channels = max(int(maxIn), int(maxOut))
            fi = dynamicSpaceFormattingBefore(i, 2)
            fName = dynamicSpaceFormattingAfter(devName, maxDevNameLength)
            print(f"{fi} = Name: {fName} | Channels: {channels} | Hz: {sample}")

def selectDevice():
    listWorkingDevices()
    print("\nEnter the device number to select a device!\n")
    devIndex = int(input("> "))
    print()
    parameters = selectSpecificDevice(devIndex)
    return(parameters)


def updateScaler(multiplier):
    mp = multiplier
    def returnScaler(rAudioValue):
        return(rAudioValue*mp)
    return(returnScaler)

# ----------------------------------- Butterworth Functions ----------------------------------- #

def butter_bandpass(lowcut, highcut, fs, order):
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        sos = butter(order, [low, high], analog=False, btype='band', output='sos')
        return sos

def butter_bandpass_filter(data, lowcut, highcut, fs, order):
        sos = butter_bandpass(lowcut, highcut, fs, order=order)
        y = sosfilt(sos, data)
        return y

###############################################################################

bas_loCut, bas_hiCut = 16, 256
mid_loCut, mid_hiCut = 256, 1536
tre_loCut, tre_hiCut = 1536, 16384
order = 5

def getValuesMono(data_in):
    indata = data_in
    aL = indata
    aR = indata
    aS = np.array([[i,j] for i,j in zip(aL, aR)])
    aV = np.linalg.norm(aS)
    
    bL = butter_bandpass_filter(aL, bas_loCut, bas_hiCut, framerate, order)
    bR = butter_bandpass_filter(aR, bas_loCut, bas_hiCut, framerate, order)
    bS = np.array([np.array([i,j]).astype('float32') for i,j in zip(bL, bR)])
    bV = np.linalg.norm(bS)
    
    mL = butter_bandpass_filter(aL, mid_loCut, mid_hiCut, framerate, order)
    mR = butter_bandpass_filter(aR, mid_loCut, mid_hiCut, framerate, order)
    mS = np.array([np.array([i,j]).astype('float32') for i,j in zip(mL, mR)])
    mV = np.linalg.norm(mS)
    
    tL = butter_bandpass_filter(aL, tre_loCut, tre_hiCut, framerate, order)
    tR = butter_bandpass_filter(aR, tre_loCut, tre_hiCut, framerate, order)
    tS = np.array([np.array([i,j]).astype('float32') for i,j in zip(tL, tR)])
    tV = np.linalg.norm(tS)

    return(np.array([aV, bV, mV, tV]))

def getValuesStereo(data_in):
    indata = data_in
    aL = np.array([indata[i] for i in range(len(indata)) if i%2 != 0])
    aR = np.array([indata[i] for i in range(len(indata)) if i%2 == 0])
    aS = np.array([[i,j] for i,j in zip(aL, aR)])
    aV = np.linalg.norm(aS)
    
    bL = butter_bandpass_filter(aL, bas_loCut, bas_hiCut, framerate, order)
    bR = butter_bandpass_filter(aR, bas_loCut, bas_hiCut, framerate, order)
    bS = np.array([np.array([i,j]).astype('float32') for i,j in zip(bL, bR)])
    bV = np.linalg.norm(bS)
    
    mL = butter_bandpass_filter(aL, mid_loCut, mid_hiCut, framerate, order)
    mR = butter_bandpass_filter(aR, mid_loCut, mid_hiCut, framerate, order)
    mS = np.array([np.array([i,j]).astype('float32') for i,j in zip(mL, mR)])
    mV = np.linalg.norm(mS)
    
    tL = butter_bandpass_filter(aL, tre_loCut, tre_hiCut, framerate, order)
    tR = butter_bandpass_filter(aR, tre_loCut, tre_hiCut, framerate, order)
    tS = np.array([np.array([i,j]).astype('float32') for i,j in zip(tL, tR)])
    tV = np.linalg.norm(tS)

    return(np.array([aV, bV, mV, tV]))

def getValues51(data_in):
    indata = data_in
    aL = np.array([indata[i] for i in range(len(indata)) if i%6 == 0])
    aR = np.array([indata[i] for i in range(len(indata)) if i%6 == 1])
    aS = np.array([[i,j] for i,j in zip(aL, aR)])
    aV = np.linalg.norm(aS)
    
    bL = butter_bandpass_filter(aL, bas_loCut, bas_hiCut, framerate, order)
    bR = butter_bandpass_filter(aR, bas_loCut, bas_hiCut, framerate, order)
    bS = np.array([np.array([i,j]).astype('float32') for i,j in zip(bL, bR)])
    bV = np.linalg.norm(bS)
    
    mL = butter_bandpass_filter(aL, mid_loCut, mid_hiCut, framerate, order)
    mR = butter_bandpass_filter(aR, mid_loCut, mid_hiCut, framerate, order)
    mS = np.array([np.array([i,j]).astype('float32') for i,j in zip(mL, mR)])
    mV = np.linalg.norm(mS)
    
    tL = butter_bandpass_filter(aL, tre_loCut, tre_hiCut, framerate, order)
    tR = butter_bandpass_filter(aR, tre_loCut, tre_hiCut, framerate, order)
    tS = np.array([np.array([i,j]).astype('float32') for i,j in zip(tL, tR)])
    tV = np.linalg.norm(tS)

    return(np.array([aV, bV, mV, tV]))

def getValues71(data_in):
    indata = data_in
    aL = np.array([indata[i] for i in range(len(indata)) if i%8 == 0])
    aR = np.array([indata[i] for i in range(len(indata)) if i%8 == 1])
    aS = np.array([[i,j] for i,j in zip(aL, aR)])
    aV = np.linalg.norm(aS)
    
    bL = butter_bandpass_filter(aL, bas_loCut, bas_hiCut, framerate, order)
    bR = butter_bandpass_filter(aR, bas_loCut, bas_hiCut, framerate, order)
    bS = np.array([np.array([i,j]).astype('float32') for i,j in zip(bL, bR)])
    bV = np.linalg.norm(bS)
    
    mL = butter_bandpass_filter(aL, mid_loCut, mid_hiCut, framerate, order)
    mR = butter_bandpass_filter(aR, mid_loCut, mid_hiCut, framerate, order)
    mS = np.array([np.array([i,j]).astype('float32') for i,j in zip(mL, mR)])
    mV = np.linalg.norm(mS)
    
    tL = butter_bandpass_filter(aL, tre_loCut, tre_hiCut, framerate, order)
    tR = butter_bandpass_filter(aR, tre_loCut, tre_hiCut, framerate, order)
    tS = np.array([np.array([i,j]).astype('float32') for i,j in zip(tL, tR)])
    tV = np.linalg.norm(tS)

    return(np.array([aV, bV, mV, tV]))

timeList = []
timeList.append(datetime.now())
maxBass, maxTreb, maxMidd = [],[],[]
RGB2HSV = colorsys.rgb_to_hsv

### You can change these parameters if you want! :) ###

# This will give output in HSV format instead of RGB.
# WARNING: You *WILL* have to use RGB version of the code with Arduino!
hsvMode = True

# This number decides what value is considered "extra".
# The extra bass/mid/treb values, make lights white, as an effect of being "too loud".
highThresh = 170

# This switch decides what the output will look like.
# I recommend keeping minimal output since that's what you need.
# Here's the explanation of what a line will look like:
# CURRENT TIME | Latency| Bass   | Mid    | Treble | Hue    |Saturate| Value
# 17:36:32.057 | L:  20 | b:  18 | m:  30 | t:   8 | h:  66 | s: 187 | l:  30 
minimalMode = True
# NOTE: These print statements are critical, since they are what pauses the script,
#       in case you hit Ctrl+C to pause the script.
#
#       Without print statements, all interrupts won't work!

# This switch automatically delays a bit, waiting for Arduino Serial Buffers
# to clear themselves. This is required, because any random delay caused,
# will mess up the data received by Arduino and cause super bright white colors
# in the LED strip.

# NOTE:
# This might not work properly sometimes.
# To fix it yourself, just randomly click and hold the IDLE Window for fractions of
# seconds, which will pause the script and fix the delay automatically.
autoPing = False
# YOU CAN TURN IT OFF IF YOU PREFER FIXING GLITCHES YOURSELF!


lightweight = True

refreshtime = []
refreshtime.append(datetime.now())

bufferWaited = False
initialDelay = False
wasScriptPaused = False

check = False

def freqSound(in_data, frame_count, time_info, flag):
    global bufferWaited
    global initialDelay
    global wasScriptPaused
    global autoPing
    global check
    
    before = datetime.now()
    rt_before = refreshtime[0]
    secondsPassed = (datetime.now()-rt_before).seconds

    if(secondsPassed > 29):
        refreshtime.clear()
        refreshtime.append(datetime.now())
        ard.flushInput()
        ard.flushOutput()
        print("\n\nBuffers Flushed!\n\n")

    indata = np.fromstring(in_data, dtype=np.float32)

    if(len(indata) == 2048):
        stereo = getValuesStereo(indata)
        aV, bV, mV, tV = stereo[0], stereo[1], stereo[2], stereo[3]

    elif(len(indata) == 8192):
        octa = getValues71(indata)
        aV, bV, mV, tV = octa[0], octa[1], octa[2], octa[3]

    elif(len(indata) == 1024):
        mono = getValuesMono(indata)
        aV, bV, mV, tV = mono[0], mono[1], mono[2], mono[3]
        
    elif(len(indata) == 6144):
        hexa = getValues51(indata)
        aV, bV, mV, tV = hexa[0], hexa[1], hexa[2], hexa[3]

    faV = 12.75*aV
    sbV = 12.75/1.15
    smV = 12.75
    stV = 12.75/0.68

    volList = [bV*sbV, mV*smV, tV*stV]
    
    fbV = bV * sbV
    fmV = mV * smV
    ftV = tV * stV
    ftt = fbV + fmV + ftV

    iaV = int(np.round(faV, 0))
    ibV = int(np.round(fbV, 0))
    imV = int(np.round(fmV, 0))
    itV = int(np.round(ftV, 0))
    itt = ibV + imV + itV
        
    xbV = max(0, fbV-highThresh)
    xmV = max(0, fmV-highThresh)
    xtV = max(0, ftV-highThresh)

    ibV -= (xmV + xtV)
    imV -= (xbV + xtV)
    itV -= (xbV + xmV)

    ibV = max(0, ibV)
    imV = max(0, imV)
    itV = max(0, itV)

    ibV = int(np.round(ibV, 0))
    imV = int(np.round(imV, 0))
    itV = int(np.round(itV, 0))

    if(not lightweight):
        maxBass.append(ibV)
        maxTreb.append(itV)
        maxMidd.append(imV)

    bassInt = max(2, ibV)
    middInt = max(2, imV)
    trebInt = max(2, itV)

    hsv = list(RGB2HSV(bassInt, middInt, trebInt))
    hsv[0] = int(np.round(hsv[0]*255, 0))
    hsv[1] = int(np.round(hsv[1]*255, 0))
    hsv[2] = int(np.round(hsv[2], 0))
    
    hue, sat, lit = hsv[0], hsv[1], hsv[2]
    
    if(hsvMode == True):
        audioString = ""
        audioString += str(space_formatting(hue))+"\n"
        audioString += str(space_formatting(sat))+"\n"
        audioString += str(space_formatting(lit))+"\n"
        if(check):
            audioString = audioString + "*"
        SerialValue = bytes(audioString, 'utf-8')
    else:
        audioString = ""
        audioString += str(space_formatting(bassInt))+"\n"
        audioString += str(space_formatting(middInt))+"\n"
        audioString += str(space_formatting(trebInt))+"\n"
        if(check):
            audioString = audioString + "*"
        SerialValue = bytes(audioString, 'utf-8')

    if(initialDelay == False):
        print("Initial Delay...")
        time.sleep(53/60)
        initialDelay = True

    sf_aV = space_formatting(iaV)
    sf_bV = space_formatting(ibV)
    sf_mV = space_formatting(imV)
    sf_tV = space_formatting(itV)
    sf_it = space_formatting(itt)
    sf_ft = space_formatting(ftt)

    after = datetime.now()
    timedif = after - before
    lT = float(str(timedif)[6:12])
    ms = float(str(timedif.microseconds)[:-3])
    ilT = int(np.round(ms, 0))
    sf_lT = space_formatting(ilT)

    if(len(timeList) > 16384):
        timeList.clear()
        timeList.append(datetime.now())

    if(not lightweight):
        temp_b = max(maxBass)
        maxBass.clear()
        temp_t = max(maxTreb)
        maxTreb.clear()
        temp_m = max(maxMidd)
        maxMidd.clear()

        maxBass.append(max(ibV, temp_b))
        maxMidd.append(max(imV, temp_m))
        maxTreb.append(max(itV, temp_t))

    rtl = datetime.now() - timeList[-1]
    rts = int(rtl.total_seconds()*1000)
    sf_rt = space_formatting(rts)

    if(not minimalMode):
        ixb = int(np.round(xbV, 0))
        ixm = int(np.round(xmV, 0))
        ixt = int(np.round(xtV, 0))
    
    sf_h = space_formatting(str(hue))
    sf_s = space_formatting(str(sat))
    sf_l = space_formatting(str(lit))

    outTime = datetime.now()
    timeList.append(outTime)
    outTime = str(outTime)[11:-3]
    
    outString = ""
    outString += f"{outTime} "
    if(minimalMode == True):
        outString += f"| L: {sf_rt} "
        outString += f"| b: {sf_bV} "
        outString += f"| m: {sf_mV} "
        outString += f"| t: {sf_tV} "
        outString += f"| h: {sf_h} "
        outString += f"| s: {sf_s} "
        outString += f"| l: {sf_l} "
        
    else:
        outString += f"| Ping: {sf_rt} "
        if(hsvMode == True):
            outString += f"| hue: {sf_h} "
            outString += f"| sat: {sf_s} "
            outString += f"| lit: {sf_l} "
            outString += f"| bas: {sf_bV} "
            outString += f"| mid: {sf_mV} "
            outString += f"| tre: {sf_tV} "
            
        else:
            outString += f"| BassVal: {sf_bV} "
            outString += f"| MiddVal: {sf_mV} "
            outString += f"| TrebVal: {sf_tV} "
    
    if(rts > 100 and bufferWaited == False and wasScriptPaused == False and autoPing == True):
        print("\n\nLatency Spike Detected! Waiting for Buffer Timeout!\n\n")
        time.sleep(1)
        bufferWaited = True
        wasScriptPaused = True
        SendData = ard.write(SerialValue)
        print(outString)
    else:
        bufferWaited = False
        wasScriptPaused = False
        SendData = ard.write(SerialValue)
        print(outString)
    
    return(np.array(in_data), pyaudio.paContinue)

# ----------------------------------- This is where main stuff is ----------------------------------- #


# This is for frequency cutoff version.
p = pyaudio.PyAudio()

params = selectDevice()
INDEX, CHANNELS, RATE = params[3], max(params[0], params[1]), params[2]
    
deviceName = p.get_device_info_by_index(INDEX)['name']
    
soundArray = []
framerate = RATE

baudRate = 128000

print("\nStarting Serial Communication on {} at {} Baud...\n".format(comPort, baudRate))
ard = None
ard = ps.Serial(comPort, baudRate)
ard.flushInput()
ard.flushOutput()


print("Waiting for 5 seconds to let Serial comm initialize properly...")

deLoopback = None

sleep(5)
if("Stereo Mix" in deviceName):
    stream = openStream(INDEX, CHANNELS, RATE, False, True, freqSound, False)
    deLoopback = False
else:
    stream = openStream(INDEX, CHANNELS, RATE, False, True, freqSound, True)
    deLoopback = True

stream.start_stream()
while True:
    try:
        time.sleep(1)
    except(KeyboardInterrupt):
        print("\n\nKeyboard Interrupt. Stream Paused.")
        print("\n\nType 'ping' to toggle automatic delay detection.")
        whatToDoNow = str(input("Type 'yes' to continue or 'no' to stop.\n\n"))
        if("yes" in whatToDoNow or "Yes" in whatToDoNow):
            wasScriptPaused = True
            print("\nResuming Stream\n")
            continue
        
        elif("no" in whatToDoNow or "No" in whatToDoNow):
            print("\nExiting...\n")
            stream.close()
            print("Stream closed")
            p.terminate()
            print("PyAudio Terminated")
            break

        elif("ping" or "Ping" in whatToDoNow):
            print(f"Latency Detection was {autoPing}. Now it is {not autoPing}.\n")
            autoPing = not autoPing
            continue

        else:
            print("\n\nInvalid input. Stop again -.-\n\n")
            continue
