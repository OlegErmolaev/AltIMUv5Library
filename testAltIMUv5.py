import AltIMUv5
import time
controller = AltIMUv5.AltIMU10v5()
#controller.start()
run = True
try:
    while run:
        data = controller.getXlCurr()[2]
        if data is not None:
            print("SHIT: %d" % (data))
        else:
            pass
            #print('None')
        time.sleep(0.5)
except KeyboardInterrupt:
    run = False
controller.stop()
