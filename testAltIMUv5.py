import AltIMUv5
import time
controller = AltIMUv5.AltIMU10v5()
#controller.start()
run = True
try:
    count = 0
    data = 0
    while run:
        count+=1
        data +=controller.getAltCurr()
        if data is not None and count == 15:
            print('%.3f' % (data/count))
            count = 0
            data = 0
        else:
            pass
            #print('None')
        time.sleep(0.05)
except KeyboardInterrupt:
    run = False
controller.stop()
