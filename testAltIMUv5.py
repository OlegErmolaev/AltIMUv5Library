import AltIMUv5
import time
controller = AltIMUv5.AltIMU10v5(gyroResolution=0b00)
#controller.start()
run = True
try:
    count = 0
    data = 0
    while run:
        count+=1
        data =controller.getGyroCurr()[0]
        if data is not None:
            print((data))
            count = 0
            data = 0
        else:
            pass
            #print('None')
        time.sleep(0.5)
except KeyboardInterrupt:
    run = False
controller.stop()
