import threading
import DobotDllType as dType
import time

def initconst():
    global CON_STR
    global Towers
    global completeTowers
    global towerKey
    global towerCoords
    global towerHeight
    global reverse_source
    global reverse_target

    CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
    }
    Towers = [[0],[0],[0]]
    completeTowers = [[0],[0],[0]]
    towerKey = ["A","B","C"]
    towerCoords = [
    ["A",[267,-104,50,-32]],
    ["B",[267,-3,50,-32]],
    ["C",[260,100,50,-32]]
    ]
    towerHeight = [-49,-43,-38,-32,-26,-20,-15,-9]
    reverse_source = []
    reverse_target = []

def initarm():
    global api

    api = dType.load()
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("Connect status:",CON_STR[state])

    if (state == dType.DobotConnect.DobotConnect_NoError):
        dType.SetHOMEParams(api,250,0,50,0,isQueued = 1)
        dType.SetPTPJointParams(api, 600, 600, 600, 600, 600, 600, 600, 600, isQueued = 1)
        dType.SetPTPCoordinateParams(api, 1000, 1000, 1000,  1000,  isQueued=1)
        dType.SetPTPCommonParams(api, 1000, 1000, isQueued = 1)
        dType.SetARCParams(api,1000,1000,1000,1000,isQueued = 1)
        dType.SetQueuedCmdClear(api)
        dType.SetHOMECmd(api, temp = 0, isQueued = 1)
        print("Setting Home")

        lastIndex = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,264,5,50,-38,isQueued = 1)[0] #initial coordinates

        dType.SetQueuedCmdStartExec(api)

        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(10)
        
        #Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)

def inittower():
    global disks

    disks = int(input('Enter Tower Height: '))
    for x in range(disks):
        Towers[0].append(x+1)
        completeTowers[2].append(x+1)

def middlecoord(currentarmcoords, nextarmcoords, highesttower):
    return [((currentarmcoords[0] + nextarmcoords[0])/2),((currentarmcoords[1] + nextarmcoords[1])/2),(towerHeight[highesttower] + 75),((currentarmcoords[3] + nextarmcoords[3])/2)]

def gomiddle():
    lastIndex = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,267,-3,50,-32,isQueued = 1)[0]

    dType.SetQueuedCmdStartExec(api)

    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(10)

    dType.SetQueuedCmdStopExec(api)

def difference(a, b):
    if a > 0 and b > 0:
        return abs(a - b)
    elif a < 0 and b < 0:
        return abs(abs(a) - abs(b))
    elif (a > 0 and b < 0) or (a < 0 and b > 0):
        return abs(abs(a)+abs(b))

def robotmovement(sourceTower, targetTower):
    global Towers
    global towerCoords
    global towerHeight

    #Clear Queue Commands
    dType.SetQueuedCmdClear(api)
    lastIndex = 0

    #Dobot Arm Movement
    highesttower = max(len(Towers[0]),len(Towers[1]),len(Towers[2])) - 2

    #XYZR Coordinates
    source_XCOOR = towerCoords[sourceTower][1][0]
    source_YCOOR = towerCoords[sourceTower][1][1]
    source_ZCOOR = towerHeight[len(Towers[sourceTower])-2]
    source_RCOOR = towerCoords[sourceTower][1][3]
    target_XCOOR = towerCoords[targetTower][1][0]
    target_YCOOR = towerCoords[targetTower][1][1]
    target_ZCOOR = towerHeight[len(Towers[targetTower])-1]
    target_RCOOR = towerCoords[targetTower][1][3]

    #Z is offset to prevent collison with tower
    sourcecords = [source_XCOOR,source_YCOOR,source_ZCOOR+25,source_RCOOR]
    targetcoords = [target_XCOOR,target_YCOOR,target_ZCOOR+20,target_RCOOR] 
    
    nextarmcoords = sourcecords
    currentarmcoords = [dType.GetPose(api)[0],dType.GetPose(api)[1],dType.GetPose(api)[2],dType.GetPose(api)[3]]
    midcoords = middlecoord(currentarmcoords, nextarmcoords, highesttower)
    result = difference(sourcecords[1],currentarmcoords[1])
    if result > 50.0 :
        #to source form current
        lastIndex = dType.SetARCCmd(api,midcoords,sourcecords,isQueued = 1)[0]
    lastIndex = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,source_XCOOR,source_YCOOR,source_ZCOOR,source_RCOOR,isQueued = 1)[0]
    lastIndex = dType.SetEndEffectorSuctionCup(api,1,1,isQueued = 1)[0]
    #Z is offset to prevent collison with tower
    lastIndex = dType.SetPTPCmd(api,dType.PTPMode.PTPMOVLXYZMode,source_XCOOR,source_YCOOR,source_ZCOOR+25,source_RCOOR,isQueued = 1)[0]

    #swap target coords
    currentarmcoords = sourcecords
    nextarmcoords = targetcoords
    midcoords = middlecoord(currentarmcoords, nextarmcoords, highesttower)

    lastIndex = dType.SetARCCmd(api,midcoords,targetcoords,isQueued = 1)[0] # to target from source
    lastIndex = dType.SetEndEffectorSuctionCup(api,1,0,isQueued = 1)[0]

    dType.SetQueuedCmdStartExec(api)

    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(10)

    #Stop to Execute Command Queued
    dType.SetQueuedCmdStopExec(api)
    Towers[targetTower].append(Towers[sourceTower].pop())

def hanoi(disks, source, auxiliary, target):
    if disks == 1:
        #print('Move disk 1 from peg {} to peg {}.'.format(source, target))
        reverse_source.append(target)
        reverse_target.append(source)
        robotmovement(source, target)
        return

    hanoi(disks - 1, source, target, auxiliary)
    #print('Move disk {} from peg {} to peg {}.'.format(disks, source, target))
    reverse_source.append(target)
    reverse_target.append(source)
    robotmovement(source, target)
    hanoi(disks - 1, auxiliary, source, target)

def reversetower():
    global reverse_source
    global reverse_target
    reverse_source.reverse()
    reverse_target.reverse()
    gomiddle()
    for i in range(len(reverse_source)):
        sourceTower = reverse_source[i]
        targetTower = reverse_target[i]
        robotmovement(sourceTower, targetTower)
    gomiddle()
    reverse_source = []
    reverse_target = []

def main():
    initconst()
    initarm()
    inittower()
    while True:
        hanoi(disks, 0, 1, 2)
        reversetower()

if __name__ == "__main__":
    main()