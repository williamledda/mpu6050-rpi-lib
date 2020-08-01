import smbus
import sys
import time


class MPU6050Calib():
    GYRO_X_REG = 0x43
    GYRO_Y_REG = 0x45
    GYRO_Z_REG = 0x47

    ACC_X_REG = 0x3b
    ACC_Y_REG = 0x3d
    ACC_Z_REG = 0x3f

    POWER_MGMT_1_REG = 0x6b

    GYRO_SCALE = 131.0
    ACC_SCALE = 16384.0

    measures = {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'rx': 0.0, 'ry': 0.0, 'rz': 0.0
    }

    means = {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'rx': 0.0, 'ry': 0.0, 'rz': 0.0
    }

    offsets = {
        'ax': 0.0, 'ay': 0.0, 'az': 0.0,
        'rx': 0.0, 'ry': 0.0, 'rz': 0.0
    }

    def __init__(self, addr=0x68, buffer=1000, acc_dead_zone=8.0, gyro_dead_zone=1.0):
        self.address = addr
        self.buffer_size = buffer
        self.acc_dead_zone = acc_dead_zone
        self.gyro_dead_zone = gyro_dead_zone
        self.bus = smbus.SMBus(1)  # or bus = smbus.SMBus(1) for Revision 2 boards
        # Now wake the 6050 up as it starts in sleep mode
        self.bus.write_byte_data(self.address, self.POWER_MGMT_1_REG, 0)

    def read_byte(self, adr):
        return self.bus.read_byte_data(self.address, adr)

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr + 1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def compute_avg(self):
        i = 0
        buff_ax = 0
        buff_ay = 0
        buff_az = 0
        buff_rx = 0
        buff_ry = 0
        buff_rz = 0

        while i < (self.buffer_size + 101):
            # read raw accel / gyro measurements from device
            self.sensor_readings()

            # First 100 measures are discarded
            if 100 < i <= (self.buffer_size + 100):
                buff_ax = buff_ax + self.measures['ax']
                buff_ay = buff_ay + self.measures['ay']
                buff_az = buff_az + self.measures['az']
                buff_rx = buff_rx + self.measures['rx']
                buff_ry = buff_ry + self.measures['ry']
                buff_rz = buff_rz + self.measures['rz']

            if i == (self.buffer_size + 100):
                self.means['ax'] = buff_ax / self.buffer_size
                self.means['ay'] = buff_ay / self.buffer_size
                self.means['az'] = buff_az / self.buffer_size
                self.means['rx'] = buff_rx / self.buffer_size
                self.means['ry'] = buff_ry / self.buffer_size
                self.means['rz'] = buff_rz / self.buffer_size

            i = i + 1
            sys.stdout.write('. ')
            sys.stdout.flush()
            time.sleep(0.002)  # 2ms Needed so we don't get repeated measures

        print('')

    def compute_offsets(self):
        ax_offset = -self.means['ax']
        ay_offset = -self.means['ay']
        az_offset = (16384 - self.means['az'])

        rx_offset = -self.means['rx']
        ry_offset = -self.means['ry']
        rz_offset = -self.means['rz']

        self.offsets['ax'] = ax_offset
        self.offsets['ay'] = ay_offset
        self.offsets['az'] = az_offset

        self.offsets['rx'] = rx_offset
        self.offsets['ry'] = ry_offset
        self.offsets['rz'] = rz_offset

    def remove_zero_offset(self):
        ax_offset = -self.means['ax'] / 8.0
        ay_offset = -self.means['ay'] / 8.0
        az_offset = (16384 - self.means['az']) / 8.0

        rx_offset = -self.means['rx'] / 4.0
        ry_offset = -self.means['ry'] / 4.0
        rz_offset = -self.means['rz'] / 4.0

        while True:
            ready = 0

            self.offsets['ax'] = ax_offset
            self.offsets['ay'] = ay_offset
            self.offsets['az'] = az_offset

            self.offsets['rx'] = rx_offset
            self.offsets['ry'] = ry_offset
            self.offsets['rz'] = rz_offset

            self.compute_avg()

            if abs(self.means['ax']) <= self.acc_dead_zone:
                ready = ready + 1
            else:
                ax_offset = ax_offset - self.means['ax'] / self.acc_dead_zone

            if abs(self.means['ay']) <= self.acc_dead_zone:
                ready = ready + 1
            else:
                ay_offset = ay_offset - self.means['ay'] / self.acc_dead_zone

            if abs(16384 - self.means['az']) <= self.acc_dead_zone:
                ready = ready + 1
            else:
                az_offset = az_offset + (16384 - self.means['az']) / self.acc_dead_zone

            if abs(self.means['rx']) <= self.gyro_dead_zone:
                ready = ready + 1
            else:
                rx_offset = rx_offset - self.means['rx'] / (self.gyro_dead_zone + 1)

            if abs(self.means['ry']) <= self.gyro_dead_zone:
                ready = ready + 1
            else:
                ry_offset = ry_offset - self.means['ry'] / (self.gyro_dead_zone + 1)

            if abs(self.means['rz']) <= self.gyro_dead_zone:
                ready = ready + 1
            else:
                rz_offset = rz_offset - self.means['rz'] / (self.gyro_dead_zone + 1)

            # self.print_offsets()
            print('avgs: ' +
                  str(self.means['ax']) + ', ' +
                  str(self.means['ay']) + ', ' +
                  str(self.means['az']) + ', ' +
                  str(self.means['rx']) + ', ' +
                  str(self.means['ry']) + ', ' +
                  str(self.means['rz']) + ' -> ' +
                  'Ready: ' + str(ready))

            if ready == 6:
                break

    def sensor_readings(self, use_offset=True):
        offset_factor = 0.0

        if use_offset:
            offset_factor = 1.0

        self.measures['ax'] = self.read_word_2c(self.ACC_X_REG) + self.offsets['ax'] * offset_factor
        self.measures['ay'] = self.read_word_2c(self.ACC_Y_REG) + self.offsets['ay'] * offset_factor
        self.measures['az'] = self.read_word_2c(self.ACC_Z_REG) + self.offsets['az'] * offset_factor

        self.measures['rx'] = self.read_word_2c(self.GYRO_X_REG) + self.offsets['rx'] * offset_factor
        self.measures['ry'] = self.read_word_2c(self.GYRO_Y_REG) + self.offsets['ry'] * offset_factor
        self.measures['rz'] = self.read_word_2c(self.GYRO_Z_REG) + self.offsets['rz'] * offset_factor

    def print_measures(self):
        print('ax out: ' + str(self.measures['ax']) + ' -> ' + str(self.measures['ax'] / self.ACC_SCALE) + ' g')
        print('ay out: ' + str(self.measures['ay']) + ' -> ' + str(self.measures['ay'] / self.ACC_SCALE) + ' g')
        print('az out: ' + str(self.measures['az']) + ' -> ' + str(self.measures['az'] / self.ACC_SCALE) + ' g')

        print('rx out: ' + str(self.measures['rx']) + ' -> ' + str(self.measures['rx'] / self.GYRO_SCALE) + ' rad/s')
        print('ry out: ' + str(self.measures['ry']) + ' -> ' + str(self.measures['ry'] / self.GYRO_SCALE) + ' rad/s')
        print('rz out: ' + str(self.measures['rz']) + ' -> ' + str(self.measures['rz'] / self.GYRO_SCALE) + ' rad/s')

        print('')

    def print_avg(self):
        print('ax avg: ' + str(self.means['ax']) + ' -> ' + str(self.means['ax'] / self.ACC_SCALE) + ' g')
        print('ay avg: ' + str(self.means['ay']) + ' -> ' + str(self.means['ay'] / self.ACC_SCALE) + ' g')
        print('az avg: ' + str(self.means['az']) + ' -> ' + str(self.means['az'] / self.ACC_SCALE) + ' g')

        print('rx avg: ' + str(self.means['rx']) + ' -> ' + str(self.means['rx'] / self.GYRO_SCALE) + ' rad/s')
        print('ry avg: ' + str(self.means['ry']) + ' -> ' + str(self.means['ry'] / self.GYRO_SCALE) + ' rad/s')
        print('rz avg: ' + str(self.means['rz']) + ' -> ' + str(self.means['rz'] / self.GYRO_SCALE) + ' rad/s')

        print('')

    def print_offsets(self):
        print('ax offset: ' + str(self.offsets['ax']) + ' -> ' + str(self.offsets['ax'] / self.ACC_SCALE) + ' g')
        print('ay offset: ' + str(self.offsets['ay']) + ' -> ' + str(self.offsets['ay'] / self.ACC_SCALE) + ' g')
        print('az offset: ' + str(self.offsets['az']) + ' -> ' + str(self.offsets['az'] / self.ACC_SCALE) + ' g')

        print('rx offset: ' + str(self.offsets['rx']) + ' -> ' + str(self.offsets['rx'] / self.GYRO_SCALE) + ' rad/s')
        print('ry offset: ' + str(self.offsets['ry']) + ' -> ' + str(self.offsets['ry'] / self.GYRO_SCALE) + ' rad/s')
        print('rz offset: ' + str(self.offsets['rz']) + ' -> ' + str(self.offsets['rz'] / self.GYRO_SCALE) + ' rad/s')

        print('')

    def apply_offset(self):
        self.measures['ax'] = self.measures['ax'] + self.offsets['ax']
        self.measures['ay'] = self.measures['ax'] + self.offsets['ax']
        self.measures['az'] = self.measures['ax'] + self.offsets['ax']

        self.measures['rx'] = self.measures['ax'] + self.offsets['ax']
        self.measures['ry'] = self.measures['ax'] + self.offsets['ax']
        self.measures['rz'] = self.measures['ax'] + self.offsets['ax']

    def run(self):
        print('Current sensor readings (before calibration)')
        self.sensor_readings()
        self.print_measures()

        print('Collecting initial averages ...')
        self.compute_avg()
        self.print_avg()
        time.sleep(1.0)

        print('Calibrating sensor ...')
        self.compute_offsets()
        # self.remove_zero_offset()
        self.print_offsets()
        time.sleep(1.0)

        print('Calibration done!\n')

        print('Current sensor readings (after calibration)')
        self.sensor_readings()
        self.print_measures()


if __name__ == '__main__':
    MPU6050Calib(buffer=2000, acc_dead_zone=8.0, gyro_dead_zone=1.0).run()
