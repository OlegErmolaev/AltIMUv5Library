import AltIMUv5
import time
controller = AltIMUv5.AltIMU10v5()
#controller.start()
run = True
try:
    while run:
        data = controller.getAngles()
        if data is not None:
            print(data)
        else:
            pass
            #print('None')
        time.sleep(0.1)
except KeyboardInterrupt:
    run = False
controller.stop()
