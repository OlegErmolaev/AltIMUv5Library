import smbus as i2c
import time
import threading
from math import *
from enum import IntEnum
import os

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
        self._barSens = 4096 #lsb/hPa
        self._tempSens = 480 #lsb/celDeg

        # проверка шины
        try:
            if(self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.WHO_AM_I) != 0x69):
                raise gyroXlSensorException("Imcorrect device with addr %s" % str(hex(self._GYRO_ACCEL)))
        except OSError:
            raise gyroXlSensorException("Unable to find on bus addr")

        try:
            if(self._i2c.readU8(self._BAR, _LPS25H.WHO_AM_I) != 0xbd):
                raise baroSensorException("Imcorrect device with addr %s" % str(hex(self._BAR)))
        except OSError:
            raise baroSensorException("Unable to find on bus addr")

        #инициализация гироскопа/аксселлерометра
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL1_XL, (0b1000 << 2 | xlResolution) << 2 & 0xFC) # 1.66 kHz ±4 g by default
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL2_G, (0b1000 << 2 | gyroResolution) << 2 & 0xFC) # 1.66 kHz  ±250 dps by default
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL3_C, 0x04)
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL7_G, 0x60) 
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL5_C, 0x6c)
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.ORIENT_CFG_G, 0b00001000) # меняем знак по z

        #инициализация барометра
        self._i2c.writeByteData(self._BAR, _LPS25H.CTRL_REG1, 0b11000000) # 25 Hz power on


        self._prevTimeXl = 0 # время от предыдущего измерения для акселлерометра
        self._prevTimeG = 0

        self.currAlt = None

        self._running = True
        self._readXlThread = threading.Thread(target=self._readXl)
        self._readXlThread.start()

    def getGyroCurr(self):
        """
        возвращает текущее угловое ускорение в град/с
        :output вывод [x,y,z]
        """
        if(self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.STATUS_REG) & 0x02 == 0x02):
            try:
                x = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_L_G))*self._gSens
                y = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_L_G))*self._gSens
                z = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_L_G))*self._gSens

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
        print(_state)
        if(_state & 0x01 == 0x01):
            try:
                x = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_H_XL) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTX_L_XL))*self._xlSens
                y = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_H_XL) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTY_L_XL))*self._xlSens
                z = (self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_H_XL) << 8 | self._i2c.readU8(self._GYRO_ACCEL, _LSM6DS33.OUTZ_L_XL))*self._xlSens

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

    def _getCurrBaro(self):
        return ((self._i2c.readU8(self._BAR, _LPS25H.PRESS_OUT_H) << 8 | self._i2c.readU8(self._BAR, _LPS25H.PRESS_OUT_XL)) << 8 | self._i2c.readU8(self._BAR, _LPS25H.PRESS_OUT_H))/self._barSens

    def _getCurrTemp(self):
        return (self._i2c.readU8(self._BAR, _LPS25H.TEMP_OUT_L) << 8 | self._i2c.readU8(self._BAR, _LPS25H.TEMP_OUT_H))/self._tempSens

    def _readXl(self):
        pass
        while self._running:
            pass

    def stop(self):
        self._running = False
        #self._readXlThread.stop()
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL1_XL, 0x00)
        self._i2c.writeByteData(self._GYRO_ACCEL, _LSM6DS33.CTRL2_G, 0x00)


class _LSM6DS33(IntEnum):       
    ### Регистры для LSM6DS33 гироскоп/аксселерометр ###
    WHO_AM_I = 0x0f #who i am reg 0x69 fixed 
    CTRL1_XL = 0x10 #режим работы аксселлерометра 1000XXXX - 1.66 kHz 1001XXXX - 3.33 kHz 1010XXXX - 6.66 kHz XXXX(00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g)00
    CTRL2_G = 0x11 #режим работы гироскопа 1000XXXX - 1.66 kHz XXXX(00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps)00
    CTRL3_C = 0x12 # управляющий регистр 0x04 для работы i2c
    CTRL5_C = 0x14 #
    CTRL7_G = 0x16 # настройка производительности гироскопа

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

class _LPS25H(IntEnum):
    ### Регистры для LPS25H барометр ###
    WHO_AM_I = 0x0f # 0xbd fixed
    CTRL_REG1 = 0x20

    PRESS_OUT_XL = 0x28
    PRESS_OUT_L = 0x29
    PRESS_OUT_H = 0x2A

    TEMP_OUT_L = 0x2b
    TEMP_OUT_H = 0x2c
