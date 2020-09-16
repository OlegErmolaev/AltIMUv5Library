import smbus as i2c
import time
import threading
from math import *
from enum import IntEnum
import os
from scipy.spatial.transform import Rotation


deltat = 0.001
gyroMeasError = 3.14159265358979 * (5.0 / 180.0)
beta = sqrt(3.0 / 4.0) * gyroMeasError

class gyroXlSensorException(Exception):
    def __init__(self, message):
        super().__init__(message)

class baroSensorException(Exception):
    def __init__(self, message):
        super().__init__(message)

class readGyroException(Exception):
    pass
class readXlException(Exception):
    pass

class _I2C:
    def __init__(self):
        self._bus = i2c.SMBus(1)
    def readRaw(self, addr: int, cmd: int, len: int):
        """
        Чтение "сырых" данных из i2c
        :param addr: адрес устройства
        :param cmd: код комманды
        :param len: сколько байт считать
        :return: считанные данные
        """
        return self._bus.read_i2c_block_data(addr, cmd, len)

    def readU8(self, addr: int, register: int):
        """
        Чтение unsigned byte из i2c.
        :param addr: адрес устройства
        :param register: регистр для чтения
        :return: считанные данные
        """
        return self._bus.read_byte_data(addr, register) & 0xFF

    def writeByte(self, addr: int, value: int):
        """
        Отправка одного байта данных в шину i2c.
        :param addr: адрес устройства
        :param value: значение для отправки
        """
        return self._bus.write_byte(addr, value)

    def writeByteData(self, addr: int, register: int, value: int):
        """
        Запись одного байта данных в заданный регистр устройства.
        :param addr: адрес устройства
        :param register: регистр для записи
        :param value: значение для записи
        """
        value = value & 0xFF
        self._bus.write_byte_data(addr, register, value)

    def writeList(self, addr: int, register: int, data: list):
        """
        Запись списка байтов в заданный регистр устройства.
        :param addr: адрес устройства
        :param register: регистр для записи
        :param data: список данных
        """
        for i in range(len(data)):
            self._bus.write_byte_data(addr, register, data[i])
    
class AltIMU10v5 (threading.Thread):
    """
    класс для работы с датчиком AltIMU10-v5
    :param xlResolution: разрешение аксселлерометра (00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g)
    :param gyroResolution: разрешение гироскопа (00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps)
    """
    def __init__(self, xlResolution = 0b10, gyroResolution = 0b10):
        threading.Thread.__init__(self)
        

        self._i2c = _I2C()

        ### devices addreses 
        self._GYRO_ACCEL = 0x6b
        self._MAGNET = 0x1e
        self._BAR = 0x5d

        
        self._xlSensDict = {0b00:0.061,0b01:0.4888,0b10:0.122,0b11:0.244} #mg/LSB
        self._gSendDict = {0b00:8.75,0b01:17.50,0b10:35,0b11:70} #mdps/LSB

        self._xlSens = self._xlSensDict.get(xlResolution) / 1000 #g/LSB
        self._gSens = self._gSendDict.get(gyroResolution) / 1000 #dps/LSB
        #self._barSens = 4096 #lsb/hPa
        #self._tempSens = 480 #lsb/celDeg

        # проверка шины
        try:
            if(self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.WHO_AM_I) != 0x69):
                raise gyroXlSensorException("Imcorrect device with addr %s" % str(hex(self._GYRO_ACCEL)))
        except OSError:
            raise gyroXlSensorException("Unable to find on bus addr")

        '''try:
            if(self._i2c.readU8(self._BAR, _LPS25H.WHO_AM_I) != 0xbd):
                raise baroSensorException("Imcorrect device with addr %s" % str(hex(self._BAR)))
        except OSError:
            raise baroSensorException("Unable to find on bus addr")'''

        #инициализация гироскопа/аксселлерометра
        self.init_gyro()
        
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.ORIENT_CFG_G, 0b00001000) # меняем знак по z

        #инициализация барометра
        #self._i2c.writeByteData(self._BAR, _LPS25H.CTRL_REG1, 0b11000000) # 25 Hz power on


        self._prevTimeXl = 0 # время от предыдущего измерения для акселлерометра
        self._prevTimeG = 0

        self.SEq_1 = 1.0
        self.SEq_2 = 0.0 
        self.SEq_3 = 0.0
        self.SEq_4 = 0.0

        self.currAlt = None

        self._running = True
        self._readXlThread = threading.Thread(target=self._readXl)
        self._readXlThread.start()

    def init_gyro(self):
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL3_C, 0x44) #IF_INC and BDU enable
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.FIFO_CTRL5, 0x00) #FIFO mode bypass
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL2_G, _LSM6DS33.ACC_GYRO_ODR_G_POWER_DOWN)#выключаем датчик
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL10_C, 0x38) #включим оси
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL2_G, _LSM6DS33.ACC_GYRO_ODR_G_833Hz | _LSM6DS33.GYRO_FS_G_500dps)# включаем на скорость 833 Гц

        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL1_XL, (0b1000 << 2 | 0b10) << 2 & 0xFC) # 1.66 kHz ±4 g by default
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL2_G, (0b1000 << 2 | 0b10) << 2 & 0xFE) # 1.66 kHz  ±250 dps by default
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL3_C, 0x04)
        '''self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL7_G, 0x60) 
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL5_C, 0x6c)
        '''
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL1_XL, (0b1000 << 2 | 0b10) << 2 & 0xFC) # 1.66 kHz ±4 g by default
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL2_G, (0b1000 << 2 | 0b10) << 2 & 0xFC) # 1.66 kHz  ±250 dps by default
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL3_C, 0x04)
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.ORIENT_CFG_G, 0b00001000) # меняем знак по z

    def getGyroCurr(self):
        """
        возвращает текущее угловое ускорение в град/с
        :output вывод [x,y,z]
        """
        if(self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.STATUS_REG) & 0x02 == 0x02):
            try:
                #uint = signed  + 2^n
                x = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_L_G))
                y = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_L_G))
                z = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_L_G))
                if(x>2**15-1):
                    x -=2**16
                if(y>2**15-1):
                    y -=2**16
                if(z>2**15-1):
                    z -=2**16
                x*= self._gSens/180*3.14159265358979
                y*= self._gSens/180*3.14159265358979
                z*= self._gSens/180*3.14159265358979
                return [x, y, z]
            except:
                self.stop()
                raise readGyroException("Unable to read data from gyro")
        else:
            return None

    def getXlCurr(self):
        """
        возвращает текущее ускорение по осям
        :output вывод [x,y,z]
        """
        _state = self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.STATUS_REG)
        if(_state & 0x01 == 0x01):
            try:
                x = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_H_XL) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_L_XL))
                y = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_H_XL) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_L_XL))
                z = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_H_XL) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_L_XL))
                if(x>2**15-1):
                    x -=2**16
                if(y>2**15-1):
                    y -=2**16
                if(z>2**15-1):
                    z -=2**16
                x*= self._xlSens
                y*= self._xlSens
                z*= self._xlSens
                return [x, y, z]
            except:
                self.stop()
                raise readXlException("Unable to read data from xl")
        else:
            print(str(bin(_state)))
            return None

    def getTemp(self): # какая-то дичь выдает не пойми что
        return (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_H_XL) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_L_XL))/16
    
    def getAltCurr(self):
        p0 = 1013.25
        d = self._getCurrBaro()/p0
        d = d**(1/5.257)
        d = 1-d
        t = 25#self._getCurrTemp()
        h = d*(42023.1/(1-d)+t)
        return h

    '''def _getCurrBaro(self):
        return ((self._i2c.readU8(self._BAR, _LPS25H.PRESS_OUT_H) << 8 | self._i2c.readU8(self._BAR, _LPS25H.PRESS_OUT_XL)) << 8 | self._i2c.readU8(self._BAR, _LPS25H.PRESS_OUT_H))/self._barSens

    def _getCurrTemp(self):
        return (self._i2c.readU8(self._BAR, _LPS25H.TEMP_OUT_L) << 8 | self._i2c.readU8(self._BAR, _LPS25H.TEMP_OUT_H))/self._tempSens
    '''

    def filter(self,w_x, w_y, w_z, a_x, a_y, a_z):
        #Axulirary variables to avoid reapeated calcualtions
        halfSEq_1 = 0.5 * self.SEq_1
        halfSEq_2 = 0.5 * self.SEq_2
        halfSEq_3 = 0.5 * self.SEq_3
        halfSEq_4 = 0.5 * self.SEq_4
        twoSEq_1 = 2.0 * self.SEq_1
        twoSEq_2 = 2.0 * self.SEq_2
        twoSEq_3 = 2.0 * self.SEq_3
        #Normalise the accelerometer measurement
        norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z)
        a_x /= norm
        a_y /= norm
        a_z /= norm
        #Compute the objective function and Jacobian
        f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x
        f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y
        f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z
        J_11or24 = twoSEq_3;
        J_12or23 = 2.0 * SEq_4
        J_13or22 = twoSEq_1
        J_14or21 = twoSEq_2
        J_32 = 2.0 * J_14or21
        J_33 = 2.0 * J_11or24
        #Compute the gradient (matrix multiplication)
        SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1
        SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3
        SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1
        SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2
        #Normalise the gradient
        norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4)
        SEqHatDot_1 /= norm
        SEqHatDot_2 /= norm
        SEqHatDot_3 /= norm
        SEqHatDot_4 /= norm
        #Compute the quaternion derrivative measured by gyroscopes
        SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z
        SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y
        SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x
        SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x
        #Compute then integrate the estimated quaternion derrivative
        SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat
        SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat
        SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat
        SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat
        #Normalise quaternion
        norm = sqrt(self.SEq_1 * self.SEq_1 + self.SEq_2 * self.SEq_2 + self.SEq_3 * self.SEq_3 + self.SEq_4 * self.SEq_4)
        self.SEq_1 /= norm
        self.SEq_2 /= norm
        self.SEq_3 /= norm
        self.SEq_4 /= norm

    def getAngles(self):
        w_x, w_y, w_z = self.getGyroCurr()
        a_x, a_y, a_z = self.getXlCurr()
        print(w_x)
        print(a_z)
        #self.filter(w_x,w_y,w_z,a_x,a_y,a_z)
        rot_quat = [self.SEq_1,self.SEq_2,self.SEq_3,self.SEq_4]       
        rot = Rotation.from_quat(rot_quat)
        return rot.as_euler('xyz', degrees=True)

    def _readXl(self):
        pass
        while self._running:
            pass

    def stop(self):
        self._running = False
        #self._readXlThread.stop()
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL1_XL, 0x00)
        #self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL2_G, 0x00)


class _LSM6DS33(IntEnum):       
    ### Регистры для LSM6DS33 гироскоп/аксселерометр ###
    WHO_AM_I = 0x0f #who i am reg 0x69 fixed 
    CTRL1_XL = 0x10 #режим работы аксселлерометра 1000XXXX - 1.66 kHz 1001XXXX - 3.33 kHz 1010XXXX - 6.66 kHz XXXX(00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g)00
    CTRL2_G = 0x11 #режим работы гироскопа 1000XXXX - 1.66 kHz XXXX(00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps)00
    CTRL3_C = 0x12 # управляющий регистр 0x04 для работы i2c
    CTRL5_C = 0x14 #
    CTRL7_G = 0x16 # настройка производительности гироскопа
    CTRL10_C = 0x19 #

    FIFO_CTRL5 = 0x0A # управление ODR FIFO

    STATUS_REG = 0x1e #наличие новых данных с датчиков b00000TDA,GDA,XLDA

    OUT_TEMP_L = 0x20 #температура
    OUT_TEMP_H = 0x21

    OUTX_L_G = 0x22 # Х поворот
    OUTX_H_G = 0x23
    OUTY_L_G = 0x24 # Y поворот
    OUTY_H_G = 0x25
    OUTZ_L_G = 0x26 # Z поворот
    OUTZ_H_G = 0x27

    ORIENT_CFG_G = 0x0b # знак гироскопа

    OUTX_L_XL = 0x28 # X ускорение 
    OUTX_H_XL = 0x29
    OUTY_L_XL = 0x2A # Y ускорение 
    OUTY_H_XL = 0x2B
    OUTZ_L_XL = 0x2C # Z ускорение
    OUTZ_H_XL = 0x2D   
    
    ACC_GYRO_ODR_G_POWER_DOWN = 0x00 #CTRL2_G/CTRL1_XL ODR ODR;ODR;ODR;ODR;xxxx 
    ACC_GYRO_ODR_G_13Hz = 0x10
    ACC_GYRO_ODR_G_26Hz = 0x20
    ACC_GYRO_ODR_G_52Hz = 0x30
    ACC_GYRO_ODR_G_104Hz = 0x40
    ACC_GYRO_ODR_G_208Hz = 0x50
    ACC_GYRO_ODR_G_416Hz = 0x60
    ACC_GYRO_ODR_G_833Hz = 0x70
    ACC_GYRO_ODR_G_1660Hz = 0x80

    GYRO_FS_G_245dps = 0x00 # CTRL2_G FS selection
    GYRO_FS_G_500dps = 0x04
    GYRO_FS_G_1000dps = 0x08
    GYRO_FS_G_2000dps = 0x0C

class _LPS25H(IntEnum):
    ### Регистры для LPS25H барометр ###
    WHO_AM_I = 0x0f # 0xbd fixed
    CTRL_REG1 = 0x20

    PRESS_OUT_XL = 0x28
    PRESS_OUT_L = 0x29
    PRESS_OUT_H = 0x2A

    TEMP_OUT_L = 0x2b
    TEMP_OUT_H = 0x2c
