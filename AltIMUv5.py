import smbus as i2c
import time
import threading
from math import *

class gyroXlSensorException(Exception):
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
        self._BARO = 0x5d

        ### Регистры для LSM6DS33 гироскоп/аксселерометр ###
        self._WHO_AM_I = 0x0f #who i am reg 0x69 fixed 
        self._CTRL1_XL = 0x10 #режим работы аксселлерометра 1000XXXX - 1.66 kHz 1001XXXX - 3.33 kHz 1010XXXX - 6.66 kHz XXXX(00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g)00
        self._CTRL2_G = 0x11 #режим работы гироскопа 1000XXXX - 1.66 kHz XXXX(00: 250 dps; 01: 500 dps; 10: 1000 dps; 11: 2000 dps)00
        self._CTRL3_C = 0x12 # управляющий регистр 0x04 для работы i2c

        self._STATUS_REG = 0x1e #наличие новых данных с датчиков b00000TDA,GDA,XLDA

        self._OUTX_L_G = 0x22 # Х поворот
        self._OUTX_H_G = 0x23
        self._OUTY_L_G = 0x24 # Y поворот
        self._OUTY_H_G = 0x25
        self._OUTZ_L_G = 0x26 # Z поворот
        self._OUTZ_H_G = 0x27

        self._OUTX_L_XL = 0x28 # X ускорение 
        self._OUTX_H_XL = 0x29
        self._OUTY_L_XL = 0x2A # Y ускорение 
        self._OUTY_H_XL = 0x2B
        self._OUTZ_L_XL = 0x2C # Z ускорение
        self._OUTZ_H_XL = 0x2D

        self._xlSensDict = {0b00:0.061,0b01:0.4888,0b10:0.122,0b11:0.244} #mg/LSB
        self._gSendDict = {0b00:8.75,0b01:17.50,0b10:35,0b11:70} #mdps/LSB

        self._xlSens = self._xlSensDict.get(xlResolution) * 1000 #g/LSB
        self._gSens = self._gSendDict.get(gyroResolution) * 1000 #dps/LSB

        # проверка шины
        try:
            if(self._i2c.readU8(self._GYRO_ACCEL, self._WHO_AM_I) != 0x69):
                raise gyroXlSensorException("Imcorrect device with addr %s" % str(hex(self._GYRO_ACCEL)))
        except OSError:
            raise gyroXlSensorException("Unable to find on bus addr")

        #инициализация гироскопа/аксселлерометра
        self._i2c.writeByteData(self._GYRO_ACCEL, self._CTRL1_XL, (0b1000 << 2 | xlResolution) << 2 | 0x00) # 1.66 kHz ±4 g by default
        self._i2c.writeByteData(self._GYRO_ACCEL, self._CTRL2_G, (0b1000 << 2 | gyroResolution) << 2 | 0x00) # 1.66 kHz  ±250 dps by default
        self._i2c.writeByteData(self._GYRO_ACCEL, self._CTRL3_C, 0x04)

        self._prevTime = 0 # время от предыдущего измерения для акселлерометра
        self._running = True
        self._readXl()

    def getGyro(self):
        """
        возвращает угловое ускорение в град/с
        :param вывод [x,y,z]
        """
        
        if(self._i2c.readU8(self._GYRO_ACCEL, self._STATUS_REG) & 0x02 == 0x02):
            x = (self._i2c.readU8(self._GYRO_ACCEL, self._OUTX_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, self._OUTX_L_G))*self._gSens
            y = (self._i2c.readU8(self._GYRO_ACCEL, self._OUTY_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, self._OUTY_L_G))*self._gSens
            z = (self._i2c.readU8(self._GYRO_ACCEL, self._OUTZ_H_G) << 8 | self._i2c.readU8(self._GYRO_ACCEL, self._OUTZ_L_G))*self._gSens

            return [x, y, z]
        else:
            raise readreadGyroException

    def getXl(self):
        """
        возвращает ускорение по осям
        :param вывод [x,y,z]
        """
        pass

    def _readXl(self):
        pass
        while self._running:
            pass

    def stop(self):
        self._running = False

        
        
