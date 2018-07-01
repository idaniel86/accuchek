from serial import Serial
from datetime import datetime, date, time
from decimal import Decimal
from enum import IntEnum
import logging


logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
# create console handler with a higher log level
ch = logging.StreamHandler()
ch.setLevel(logging.WARNING)
# create formatter and add it to the handlers
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
# add the handlers to the logger
logger.addHandler(ch)


class ParseError(Exception):
    pass


class AccuChekStatus(IntEnum):
    NO_ERROR = 0
    COMMAND_CANCELED = 240
    STX_EXPECTED_ERROR = 241
    LENGTH_EXPECTED_ERROR = 242
    IR_DATA_OVERRUN = 246
    INVALID_NUMBER_OF_BYTES = 247
    INVALID_PARAMETER = 248
    INVALID_NUMBER_OF_PARAMETERS = 249
    RECEIVE_BUFFER_FULL = 250
    COMMUNICATION_TIMEOUT = 251
    COMMAND_NOT_IMPLEMENTED = 252
    COMMAND_ABORTED = 253
    NO_VALID_COMMAND = 254
    INITIAL_COMMUNICATION = 255


class AccuChekPerformaNano:
    """
    Class for handling Accu Chek Performa Nano serial protocol.
    """

    STX = b'\x02'
    ETX = b'\x03'
    EOT = b'\x04'
    ACK = b'\x06'
    TAB = b'\x09'
    NAK = b'\x15'
    CAN = b'\x18'
    GET_AND_CLEAR_STATUS = b'\x0B'
    SET_METER_SETTING = b'\x0C'
    CR = b'\x0D'
    TURN_OFF = b'\x1D'
    GET_FIELD_DATE = b'\x31'
    GET_FIELD_TIME = b'\x32'
    GET_FIELD_UNITS = b'\x33'
    GET_SERIAL_NUMBER = b'\x33'
    GET_METER_NUMBER = b'\x34'
    GET_METER_CONFIGURATION = b'\x43'
    GET_METER_SETTING = b'\x53'
    GET_METER_NAME = b'\x49'
    GET_READING_COUNT = b'\x60'
    GET_READINGS = b'\x61'
    CLEAR_READINGS = b'\x52'
    FIELD_SEPARATOR = b'\x09'

    def __init__(self, port):
        self._serial = Serial(port, timeout=1)
        self._retries = 5

    def _send(self, data: bytes):
        logger.debug('send: %s' % ' '.join('%02X' % byte for byte in data))
        return self._serial.write(data)

    def _receive(self, size: int=1):
        data = self._serial.read(size)
        logger.debug('recv: %s' % ' '.join('%02X' % byte for byte in data))
        return data

    def _send_command(self, cmd: bytes, *fields: bytes):
        # send command byte by byte and receive echo
        for data in self.FIELD_SEPARATOR.join([cmd]+list(fields)):
            self._receive(self._send(bytes([data])))

        # send CR byte and receive either ACK or NAK
        received = self._receive(self._send(self.CR))
        if received not in [self.ACK, self.NAK]:
            logger.warning('expected ACK or NAK, got 0x%02X' % received)

        # expecting data
        return received == self.ACK

    def _receive_packet(self):
        packet = bytearray()
        in_packet = False
        retries = self._retries
        _data = list()
        while True:
            received = self._receive(self._serial.in_waiting or 1)
            if not received:
                raise TimeoutError('no byte received in %d second timeout' % self._serial.timeout)

            for byte in received:
                byte = bytes([byte])
                if byte == self.STX:
                    in_packet = True
                elif byte in [self.ETX, self.EOT]:
                    try:
                        # receive length of data
                        if len(packet) < 2:
                            raise ParseError('expected 2 length bytes, got %d' % len(packet))
                        length = int(packet[:2].decode(), 16)
                        # receive data
                        if len(packet) < 2 + length:
                            raise ParseError('expected %d data bytes, got %d' % (length, len(packet) - 2))
                        data = packet[2:2 + length]
                        # receive checksum
                        if len(packet) < 2 + length + 2:
                            raise ParseError('expected 2 checksum bytes, got %d' % len(packet) - 2 - length)
                        checksum = int(packet[2 + length: 2 + length + 2].decode(), 16)
                        if checksum != self._checksum(data):
                            raise ParseError('expected 0x%02X checksum, got 0x%02X' % (self._checksum(data), checksum))

                        # send packet ACK
                        self._send(self.ACK)

                        # add data and remove FIELD SEPARATORS on data start and end
                        _data.append(data.strip(self.FIELD_SEPARATOR))

                        # if last packet received check for ACK
                        if byte == self.EOT:
                            byte = self._receive()
                            if byte != self.ACK:
                                raise ParseError('expected ACK, got 0x%02X' % byte)
                            return _data

                        # clear packet data
                        del packet[:]
                        in_packet = False
                    except Exception as ex:
                        logger.error(ex)
                        if retries:
                            retries -= 1
                            # send packet NAK
                            self._send(self.NAK)
                        else:
                            return
                elif in_packet:
                    packet.extend(byte)
                else:
                    logger.warning('dumping out-of-packet byte 0x%02X' % byte)

    def _send_packet(self, *fields: bytes):
        retries = self._retries
        packet = bytes()
        if len(fields):
            data = self.FIELD_SEPARATOR + self.FIELD_SEPARATOR.join(fields) + self.FIELD_SEPARATOR
            length = bytes([ord(o) for o in str(len(data)).zfill(2)])
            checksum = bytes([ord(o) for o in "%02X" % (self._checksum(data))])
            packet = self.STX + length + data + checksum + self.EOT

        while retries:
            retries -= 1
            # send packet
            if packet:
                self._send(packet)

            # expect ACK or NAK of the packet
            received = self._receive()
            if received == self.NAK:
                continue
            elif received == self.ACK:
                self._send(self.ACK)
            else:
                logger.warning('expected ACK or NAK, got 0x%02X' % received)
                self._send(self.NAK)
                break

        # expect ACK or NAK of the operation result
        received = self._receive()
        if received not in [self.ACK, self.NAK]:
            logger.warning('expected ACK or NAK, got 0x%02X' % received)
        return received == self.ACK

    def get_and_clear_status(self):
        if self._send_command(self.GET_AND_CLEAR_STATUS):
            data = self._receive_packet()
            return AccuChekStatus(int(data[0].decode(), 16))

    def get_meter_name(self):
        if self._send_command(self.GET_METER_NAME):
            data = self._receive_packet()
            return data[0].decode()

    def get_meter_number(self):
        if self._send_command(self.GET_METER_CONFIGURATION, self.GET_METER_NUMBER):
            data = self._receive_packet()
            return data[0].decode()

    def get_serial_number(self):
        if self._send_command(self.GET_METER_CONFIGURATION, self.GET_SERIAL_NUMBER):
            data = self._receive_packet()
            return data[0].decode()

    def get_current_date(self):
        if self._send_command(self.GET_METER_SETTING, self.GET_FIELD_DATE):
            data = self._receive_packet()
            return datetime.strptime(data[0].decode(), '%y%m%d').date()

    def get_current_time(self):
        if self._send_command(self.GET_METER_SETTING, self.GET_FIELD_TIME):
            data = self._receive_packet()
            return datetime.strptime(data[0].decode(), '%H%M%S').time()

    def get_meter_units(self):
        if self._send_command(self.GET_METER_SETTING, self.GET_FIELD_UNITS):
            data = self._receive_packet()
            return data[0].decode()

    def get_reading_count(self):
        if self._send_command(self.GET_READING_COUNT):
            data = self._receive_packet()
            return int(data[0].decode())

    def get_readings(self, start: int, end: int):
        if self._send_command(self.GET_READINGS, bytes([ord(o) for o in str(start)]), bytes([ord(o) for o in str(end)])):
            data = self._receive_packet()
            entries = []
            for entry in data:
                fields = entry.split(self.FIELD_SEPARATOR)
                glucose = Decimal(int(fields[0].decode()) * 0.0555).quantize(Decimal(10)**-1)
                timestamp = datetime.strptime((fields[1]+fields[2]).decode(), '%H%M%y%m%d')
                flags = int(fields[3].decode(), 16)
                entries.append((glucose, timestamp, flags))
            return entries

    def set_current_date(self, value: date):
        data = value.strftime('%y%m%d')
        if self._send_command(self.SET_METER_SETTING, self.GET_FIELD_DATE):
            return self._send_packet(data.encode())

    def set_current_time(self, value: time):
        data = time.value('%H%M%S')
        if self._send_command(self.SET_METER_SETTING, self.GET_FIELD_TIME):
            return self._send_packet(data.encode())

    def turn_off(self):
        return self._send_command(self.TURN_OFF)

    def clear_readings(self):
        if self._send_command(self.CLEAR_READINGS):
            return self._send_packet()

    @staticmethod
    def _checksum(data: bytes):
        checksum = 110
        for byte in data:
            checksum ^= byte
        return checksum


if __name__ == '__main__':
    ac = AccuChekPerformaNano('COM3')
    print(ac.get_and_clear_status())
    print(ac.get_meter_name())
    print(ac.get_meter_number())
    print(ac.get_serial_number())
    print(ac.get_current_date())
    print(ac.get_current_time())
    print(ac.get_meter_units())
    count = ac.get_reading_count()
    if count > 0:
        with open('readings.csv', 'a') as f:
            for reading in ac.get_readings(1, count):
                f.write(reading[1].strftime('%d.%m.%y %H:%M') + '; ' + str(reading[0]) + '\n')
    print("DONE")
