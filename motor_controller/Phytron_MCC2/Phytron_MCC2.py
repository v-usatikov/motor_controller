# coding= utf-8
import threading
from motor_controller.interface import *

if __name__ == '__main__':
    logscolor.init_config()


# noinspection PyPep8Naming
def MCC2SerialConnector(port: str, timeout: float = 0.2, baudrate: float = 115200) -> SerialConnector:
    return SerialConnector(port=port, beg_symbol=b'\x02', end_symbol=b'\x03', timeout=timeout, baudrate=baudrate)


class MCC2Communicator(ContrCommunicator):
    """Diese Klasse beschreibt die Sprache, die man braucht, um mit MCC2 Controller zu kommunizieren.
    Hier sind alle MCC2-spezifische Eigenschaften zusammen gesammelt"""

    # Dict, der für jeder Parameter dazugehöriger Nummer ausgibt.
    PARAMETER_NUMBER = {'Lauffrequenz': 14, 'Stoppstrom': 40, 'Laufstrom': 41, 'Booststrom': 42, 'Initiatortyp': 27,
                        'Umrechnungsfaktor(Contr)': 3}
    # Dict, mit den Beschreibungen der Parametern.
    PARAMETER_DESCRIPTION = {'Lauffrequenz': 'ein int Wert in Hz (max 40 000)',
                             'Stoppstrom': '',
                             'Laufstrom': '',
                             'Booststrom': '',
                             'Initiatortyp': '0 = PNP-Öffner oder 1 = PNP-Schließer',
                             'Umrechnungsfaktor(Contr)': ''}
    # Dict, mit den Defaultwerten der Parametern.
    PARAMETER_DEFAULT = {'Lauffrequenz': 400.0, 'Stoppstrom': 2, 'Laufstrom': 2, 'Booststrom': 2, 'Initiatortyp': 0,
                         'Umrechnungsfaktor(Contr)': 1}

    tolerance = 1.1  # Für MCC2 akzeptabele Abweichung bei Positionierung der Motoren (in Controller Einheiten)
    calibration_shift = 500000

    def __init__(self, connector: Connector):
        self.connector = connector
        self.__mutex = threading.Lock()

    def go(self, shift: float, bus: int, axis: int):
        """Verschiebt den angegeben Motor um die angegebene Verschiebung."""

        command = str(shift).encode()
        self.command_without_reply(command, bus, axis)

    def go_to(self, destination: float, bus: int, axis: int):
        """Schickt den angegeben Motor zur angegebene absolute Position."""

        command = f"A{destination}".encode()
        self.command_without_reply(command, bus, axis)

    def stop(self, bus: int, axis: int):
        """Stoppt den angegebenen Motor."""

        command = b"S"
        self.command_without_reply(command, bus, axis)

    def get_position(self, bus: int, axis: int) -> float:
        """Gibt die Position des angegebenen Motors zurück."""

        command = b"P20R"
        return self.command_with_float_reply(command, bus, axis)

    def set_position(self, new_position: float, bus: int, axis: int):
        """Ändert den Wert des Positionzählers im Controller für die angegebene Achse."""

        command = f"P20S{new_position}".encode()
        self.command_without_reply(command, bus, axis)

    def get_parameter(self, parameter_name: str, bus: int, axis: int) -> float:
        """Liest den Wert des angegebenen Parameters."""

        if parameter_name not in self.PARAMETER_NUMBER.keys():
            raise ValueError(f'Falscher Name des Parameters: "{parameter_name}"')

        param_number = self.PARAMETER_NUMBER[parameter_name]
        command = f"P{param_number}R".encode()
        return self.command_with_float_reply(command, bus, axis)

    def set_parameter(self, parameter_name: str, neu_value: float, bus: int, axis: int):
        """Ändert den Wert des angegebenen Parameters."""

        if parameter_name not in self.PARAMETER_NUMBER.keys():
            raise ValueError(f'Falscher Name des Parameters: "{parameter_name}"')

        param_number = self.PARAMETER_NUMBER[parameter_name]
        command = f"P{param_number}S{neu_value}".encode()
        self.command_without_reply(command, bus, axis)

    def motor_stand(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Motor im Moment steht(True) oder fährt(False)."""

        command = b'=H'
        return self.command_with_bool_reply(command, bus, axis)

    def motor_at_the_beg(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Anfang-Initiator im Moment aktiviert ist."""

        command = b'=I-'
        return self.command_with_bool_reply(command, bus, axis)

    def motor_at_the_end(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der End-Initiator im Moment aktiviert ist."""

        command = b'=I+'
        return self.command_with_bool_reply(command, bus, axis)

    def read_reply(self) -> (bool, Union[bytes, None]):
        """Antwort lesen, der nach einem Befehl erscheint."""

        reply = self.connector.read()
        if reply is None:
            return None, None
        elif reply[:1] == b'\x06':
            return True, reply[1:]
        elif reply == b'\x15':
            return False, None
        else:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply[1]}')

    def bus_list(self) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Bus-Nummern zurück."""

        bus_list = []
        for i in range(16):
            for j in range(2):
                check = self.bus_check(i)
                if check[0]:
                    bus_list.append(i)
                    break
            # noinspection PyUnboundLocalVariable
            if not check[0]:
                logging.warning(f'Bei Bus Nummer {i} keinen Kontroller gefunden. Controller Antwort:{check[1]}')
        if not bus_list:
            raise SerialError("Es wurde keine Controller gefunden!")
        return tuple(bus_list)

    def axes_list(self, bus: int) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Achsen zurück."""

        command = b"IAR"
        n_axes = int(self.command_with_float_reply(command, bus))
        return tuple(range(1, n_axes+1))

    def check_connection(self) -> (bool, bytes):
        """Prüft ob es bei dem Com-Port tatsächlich ein Controller gibt, und gibt die Version davon zurück."""

        check = False
        for i in range(16):
            for j in range(4):
                check = self.bus_check(i)
                # print(check)
                if check[0]:
                    return check
        return check

    def command_to_box(self, command: bytes) -> (bool, Union[bytes, None]):
        """Ausführt ein Befehl ohne Adressieren und gibt die Antwort zurück."""

        self.__mutex.acquire()
        self.connector.send(command)
        reply = self.read_reply()
        self.__mutex.release()
        return reply

    def command_to_modul(self, command: bytes, bus: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Modul adressierte Befehl und gibt die Antwort zurück."""

        command = self.__contr_prefix(bus) + command
        return self.command_to_box(command)

    def command_to_motor(self, command: bytes, bus: int, axis: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Motor adressierte Befehl und gibt die Antwort zurück."""

        command = self.__axis_prefix(axis) + command
        return self.command_to_modul(command, bus)

    def command(self, command: bytes, bus: int, axis: int = None) -> (bool, Union[bytes, None]):
        """Ausführt ein Befehl für Motor oder für Controller und gibt die Antwort zurück."""

        if axis is None:
            return self.command_to_modul(command, bus)
        else:
            return self.command_to_motor(command, bus, axis)

    # TODO schreiben check_raw_input_data zu Ende
    def check_raw_input_data(self, raw_input_data: List[dict]) -> (bool, str):
        """Prüft ob die rohe Daten aus der input-Datei kompatibel sind."""

        for motor_line in raw_input_data:
            init_status = motor_line['Mit Initiatoren(0 oder 1)']
            message = f'"Mit Initiatoren" muss 0 oder 1 sein, und kein "{init_status}"'
            if init_status != '':
                try:
                    init_status = bool(int(motor_line['Mit Initiatoren(0 oder 1)']))
                except ValueError:
                    return False, message
                if init_status not in (0, 1):
                    return False, message

            units_per_step = motor_line['Umrechnungsfaktor']
            message = f'"Einheiten pro Schritt" muss ein float Wert haben, und kein "{units_per_step}"'
            if units_per_step != '':
                try:
                    float(motor_line['Mit Initiatoren(0 oder 1)'])
                except ValueError:
                    return False, message

        return True, ""

    def bus_check(self, bus: int) -> (bool, str):
        """Prüft ob es bei dem Bus-Nummer ein Controller gibt, und gibt die Version davon zurück."""

        try:
            command = b"IVR"
            reply = self.command_to_modul(command, bus)
        except ReplyError as err:
            logging.error(str(err))
            return False, str(err)

        if reply[0] is None:
            return False, None
        elif reply[0] is False:
            return False, f'Unerwartete Antwort vom Controller: {reply[1]}!'
        elif reply[1][0:3] == b'MCC':
            return True, reply[1]
        else:
            return False, reply[1]

    def command_with_float_reply(self, command: bytes, bus: int, axis: int = None) -> float:
        """Ausführt ein Befehl mit einer erwarteten Fließkommazahl-Antwort
        und gibt die erhaltene Fließkommazahl zurück.
        """

        reply = self.command(command, bus, axis)
        return self.__transform_float_reply(reply)

    def command_with_bool_reply(self, command: bytes, bus: int, axis: int = None) -> bool:
        """Ausführt ein Befehl mit einer erwarteten bool Antwort
        und gibt den erhaltenen bool Wert zurück.
        """

        reply = self.command(command, bus, axis)
        return self.__transform_bool_reply(reply)

    def command_without_reply(self, command: bytes, bus: int, axis: int = None):
        """Ausführt ein Befehl ohne erwartete Antwort."""

        reply = self.command(command, bus, axis)
        self.__check_command_result(reply)

    @staticmethod
    def __check_command_result(reply: Tuple[bool, Union[bytes, None]]):
        """Prüft die Antwort vom Controller für ein Befehl ohne erwartete Antwort."""

        if reply[0] is None:
            raise NoReplyError('Der Controller antwortet nicht!')
        elif reply[0] is False:
            raise ControllerError('Controller hat den Befehl negativ quittiert!')
        elif reply[0] is True:
            if reply[1]:
                raise ReplyError(f'Unerwartete Antwort vom Controller: {reply[1]}!')

    @staticmethod
    def __transform_float_reply(reply: Tuple[bool, Union[bytes, None]]) -> float:
        """Prüft die Antwort vom Controller und kriegt die enthaltene Fließkommazahl heraus.
        """

        if reply[0] is None:
            raise NoReplyError('Der Controller antwortet nicht!')
        elif reply[0] is False:
            raise ControllerError('Controller hat den Befehl negativ quittiert!')
        elif reply[0] is True:
            try:
                return float(reply[1])
            except ValueError:
                raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

    @staticmethod
    def __transform_bool_reply(reply: Tuple[bool, Union[bytes, None]]) -> bool:
        """Prüft die Antwort vom Controller und kriegt den enthaltenen boll Wert heraus.
        """

        if reply[0] is None:
            raise NoReplyError('Der Controller antwortet nicht!')
        elif reply[0] is False:
            raise ControllerError('Controller hat den Befehl negativ quittiert!')
        elif reply[1] == b'E':
            return True
        elif reply[1] == b'N':
            return False
        else:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply[1]}')

    @staticmethod
    def __contr_prefix(bus: int) -> bytes:
        """Gibt ein Präfix zurück, das einen Befehl an den gewünschten Controller adressiert."""

        if bus > 15 or bus < 0:
            raise ValueError(f'bus muss ein Wert zwischen 0 und 15 haben und kein {bus}')
        return f'{bus:x}'.encode()

    @staticmethod
    def __axis_prefix(axis: int) -> bytes:
        """Gibt ein Teil des Präfixes zurück, das einen Befehl an die gewünschte Achse adressiert."""

        if axis > 9 or axis < 0:
            raise ValueError(f'axis muss ein Wert zwischen 0 und 9 haben und kein {axis}')
        return str(axis).encode()


# noinspection PyPep8Naming
def MCC2BoxSerial(port: str, timeout: float = 0.2, baudrate: float = 115200, input_file: str = None) -> Box:
    connector = MCC2SerialConnector(port=port, timeout=timeout, baudrate=baudrate)
    communicator = MCC2Communicator(connector)
    return Box(communicator=communicator, input_file=input_file)


def is_h_digit(symbol: Union[str, bytes]):
    """Zeigt ob Symbol ein Hexodecimal-Zahlzeichen ist."""
    symbol = str(symbol)
    if len(symbol) != 1:
        return False
    else:
        return symbol in '0123456789ABCDEFabcdef'


class MCC2BoxEmulator(SerialEmulator, ContrCommunicator):

    PARAMETER_NUMBER = deepcopy(MCC2Communicator.PARAMETER_NUMBER)
    PARAMETER_DEFAULT = deepcopy(MCC2Communicator.PARAMETER_DEFAULT)
    tolerance = MCC2Communicator.tolerance
    calibration_shift = MCC2Communicator.calibration_shift

    def __init__(self, n_bus: int = 3, n_axes: int = 2, realtime: bool = False):
        self.realtime = realtime
        self.controller: Dict[int, MCC2ControllerEmulator] = {}
        for i in range(n_bus):
            self.controller[i] = MCC2ControllerEmulator(self, n_axes)

        self.buffer: bytes = b''
        self.last_command = b''

    def go(self, shift: float, bus: int, axis: int):
        """Verschiebt den angegeben Motor um die angegebene Verschiebung."""

        self.__get_motor(bus, axis).go(shift)

    def go_to(self, destination: float, bus: int, axis: int):
        """Schickt den angegeben Motor zur angegebene absolute Position."""

        self.__get_motor(bus, axis).go_to(destination)

    def stop(self, bus: int, axis: int):
        """Stoppt den angegebenen Motor."""

        self.__get_motor(bus, axis).stop()

    def get_position(self, bus: int, axis: int) -> float:
        """Gibt die Position des angegebenen Motors zurück."""

        return self.__get_motor(bus, axis).get_position()

    def set_position(self, new_position: float, bus: int, axis: int):
        """Ändert den Wert des Positionzählers im Controller für die angegebene Achse."""

        self.__get_motor(bus, axis).set_position(new_position)

    def get_parameter(self, parameter_name: str, bus: int, axis: int) -> float:
        return self.__get_motor(bus, axis).get_parameter(self.PARAMETER_NUMBER[parameter_name])

    def set_parameter(self, parameter_name: str, neu_value: float, bus: int, axis: int):
        self.__get_motor(bus, axis).set_parameter(self.PARAMETER_NUMBER[parameter_name], neu_value)

    def motor_stand(self, bus: int, axis: int) -> bool:
        return self.__get_motor(bus, axis).stand()

    def motor_at_the_beg(self, bus: int, axis: int) -> bool:
        return self.__get_motor(bus, axis).at_the_beg()

    def motor_at_the_end(self, bus: int, axis: int) -> bool:
        return self.__get_motor(bus, axis).at_the_end()

    def read_reply(self) -> (bool, bytes):
        self.read_until(b'\x03')

    def bus_list(self) -> Tuple[int]:
        return tuple(self.controller.keys())

    def axes_list(self, bus: int) -> Tuple[int]:
        return tuple(self.controller[bus].motor.keys())

    def check_connection(self) -> (bool, bytes):
        return True, b''

    def command_to_box(self, command: bytes) -> bytes:
        self.write(b'\x02' + command + b'\x03')
        return self.__read_reply()

    def command_to_modul(self, command: bytes, bus: int) -> bytes:
        self.write(b'\x02' + f'{bus:x}{command}'.encode() + b'\x03')
        return self.__read_reply()

    def command_to_motor(self, command: bytes, bus: int, axis: int) -> bytes:
        self.write(b'\x02' + f'{bus:x}{axis}{command}'.encode() + b'\x03')
        return self.__read_reply()

    def __read_reply(self):
        if self.buffer:
            return self.read_until(b'\x03')[1:-1]
        else:
            return self.buffer

    def check_raw_input_data(self, raw_input_data: List[dict]) -> (bool, str):
        return True, ''

    def flushInput(self):
        self.buffer = b''

    def close(self):
        pass

    def read_until(self, end_symbol: bytes) -> bytes:
        if end_symbol in self.buffer:
            answer, self.buffer = self.buffer.split(end_symbol, 1)
            return answer + end_symbol
        else:
            if self.realtime:
                time.sleep(self.timeout)
            return self.buffer

    def write(self, command: bytes):
        """Liest, interpretiert und ausführt den angegebenen Befehl."""
        def read_axis_number(digit: str):
            if len(digit) > 1:
                raise ValueError(f'"digit" muss ein einzigen Symbol sein und ken: {digit}')
            elif not str.isdigit(digit) and digit not in 'XYxy':
                raise ValueError(f'"digit" muss ein int Ziffer oder gleich "X" oder "Y" sein und kein: {digit}')
            if digit in 'Xx':
                return 1
            elif digit in 'Yy':
                return 2
            else:
                return int(digit)

        def unknown_command():
            self.__answer(denial)
            print(f'MCC2BoxEmulator: Unbekannter Befehl für einem Controller: {command_to_modul}')

        def check_parameter_number(n):
            if n in list(motor.PARAMETER_NAME.keys()) + [20]:
                return True
            else:
                self.__answer(denial)
                print(f'MCC2BoxEmulator: Falsches Parameter Nummer: {n}')
                return False

        def is_number(s):
            try:
                float(s)
                return True
            except ValueError:
                return False

        self.last_command = command

        confirm = b'\x06'
        denial = b'\x15'

        if command[:1] == b'\x02' and command[-1:] == b'\x03':
            command = command[1:-1].decode().upper()  # End- und Anfangsymbol abschneiden
            if is_h_digit(command[0]):
                bus = int(command[:1], 16)  # Bus-Nummer lesen
                command_to_modul = command[1:]  # Bus-Nummer abschneiden
                if bus not in self.controller.keys():  # prüfen ob solche Bus-Nummer vorhanden
                    return

                if command_to_modul:  # prüfen ob der Befehl zum Modul nicht leer ist
                    if command_to_modul == 'IVR':
                        self.__answer(confirm + self.__controller_version())
                        return
                    elif command_to_modul == 'IAR':
                        self.__answer(confirm + str(len(self.controller[bus].motor)).encode())
                        return
                    elif str.isdigit(command_to_modul[0]) or command_to_modul[0] in 'XY':
                        axis = read_axis_number(command_to_modul[0])  # Achse-Nummer lesen
                        command_to_motor = command_to_modul[1:]  # Achse-Nummer abschneiden
                        if axis not in self.controller[bus].motor.keys():  # prüfen ob solche Achse-Nummer vorhanden
                            self.__answer(denial)
                            print(f'Falsche Achsenummer: {axis}')
                            return
                        motor = self.controller[bus].motor[axis]

                        if command_to_motor:  # prüfen ob der Befehl zum Motor nicht leer ist
                            if is_number(command_to_motor):  # "go" Befehl
                                shift = float(command_to_motor)
                                motor.go(shift)
                                self.__answer(confirm)
                                return
                            elif command_to_motor[0] == 'A':  # "go_to" Befehl
                                try:
                                    destination = float(command_to_motor[1:])
                                except ValueError:
                                    unknown_command()
                                    return
                                else:
                                    motor.go_to(destination)
                                    self.__answer(confirm)
                                    return
                            elif command_to_motor == 'S':  # Stop Befehl
                                motor.stop()
                                self.__answer(confirm)
                                return
                            elif command_to_motor[0] == 'P':  # Parameter-Befehl
                                parameter_command = command_to_motor[1:]
                                if 'S' in parameter_command:  # Parameter ändern
                                    if len(parameter_command.split('S')) > 2:
                                        unknown_command()
                                        return
                                    else:
                                        param_num, param_value = parameter_command.split('S')
                                        try:
                                            param_num = int(param_num)
                                            param_value = float(param_value)
                                        except ValueError:
                                            unknown_command()
                                            return
                                        else:
                                            if check_parameter_number(param_num):
                                                motor.set_parameter(param_num, param_value)
                                                self.__answer(confirm)
                                                return
                                            else:
                                                return
                                elif parameter_command[-1] == 'R':  # Parameter lesen
                                    try:
                                        param_num = int(parameter_command[:-1])
                                    except ValueError:
                                        self.__answer(denial)
                                        return
                                    else:
                                        if check_parameter_number(param_num):
                                            self.__answer(confirm + str(motor.get_parameter(param_num)).encode())
                                            return
                                        else:
                                            return
                            elif command_to_motor == '=H':  # 'motor_stand' Befehl
                                if motor.stand():
                                    self.__answer(confirm + b'E')
                                else:
                                    self.__answer(confirm + b'N')
                                return
                            elif command_to_motor == '=I-':  # 'motor_at_the_beg' Befehl
                                if motor.at_the_beg():
                                    self.__answer(confirm + b'E')
                                else:
                                    self.__answer(confirm + b'N')
                                return
                            elif command_to_motor == '=I+':  # 'motor_at_the_end' Befehl
                                if motor.at_the_end():
                                    self.__answer(confirm + b'E')
                                else:
                                    self.__answer(confirm + b'N')
                                return

                unknown_command()
                return

    def __answer(self, answer: bytes):
        self.buffer += b'\x02' + answer + b'\x03'

    @staticmethod
    def __controller_version() -> bytes:
        return b'MCC2 Emulator v1.0'

    def __get_motor(self, bus, axis):
        return self.controller[bus].motor[axis]


class MCC2ControllerEmulator:
    def __init__(self, box: MCC2BoxEmulator, n_axes: int = 2):
        self.box = box
        self.motor: Dict[int, MCC2MotorEmulator] = {}
        for i in range(1, n_axes+1):
            self.motor[i] = MCC2MotorEmulator(box)


class MCC2MotorEmulator:

    PARAMETER_NAME = {v: k for k, v in MCC2BoxEmulator.PARAMETER_NUMBER.items()}

    def __init__(self, box: MCC2BoxEmulator):
        self.box = box
        self.__position_steps = 0  # aktuelle Position in Schritten
        self._stand = True
        self._beg_initiator = False
        self._end_initiator = False
        self.__destination = 0
        self.__stop = False

        self.parameter_values = deepcopy(MCC2BoxEmulator.PARAMETER_DEFAULT)

        self.beginning = -10000
        self.end = 10000

    def stand(self):
        return self._stand

    def at_the_beg(self):
        return self._beg_initiator

    def at_the_end(self):
        return self._end_initiator

    def get_position(self):
        return self.__position_steps * self.parameter_values['Umrechnungsfaktor(Contr)']

    def set_position(self, value: float):
        self.__position_steps = value / self.parameter_values['Umrechnungsfaktor(Contr)']

    def set_parameter(self, n: int, value: Union[float, int]):
        if n == 20:
            self.set_position(value)
        else:
            self.parameter_values[self.PARAMETER_NAME[n]] = value

    def get_parameter(self, n: int) -> Union[float, int]:
        if n == 20:
            return self.get_position()
        else:
            return self.parameter_values[self.PARAMETER_NAME[n]]

    def go_to(self, destination: float):
        self.__destination = destination/self.parameter_values['Umrechnungsfaktor(Contr)']
        if self.stand():
            self._stand = False
            threading.Thread(target=self.__move).start()

    def go(self, shift: float):
        self.go_to(self.get_position() + shift)

    def stop(self):
        self.__stop = True

    def sleep_one_step(self):
        time.sleep(1/self.__freq())

    def sleep_steps(self, n):
        for i in range(n):
            time.sleep(1 / self.__freq())

    def wait_stop(self):
        while not self._stand:
            self.sleep_one_step()

    def __move(self):
        self.__stop = False
        self._stand = False
        while abs(self.__position_steps - self.__destination) > 0.5:
            if self.__position_steps > self.__destination:
                self.__step_back()
            else:
                self.__step_forward()
            if self.box.realtime:
                self.sleep_one_step()
            if self.__stop:
                break
        self._stand = True

    def __step_forward(self):
        self.__initiators_sensor()
        if not self._end_initiator:
            self.__position_steps += 1
        else:
            self.stop()

    def __step_back(self):
        self.__initiators_sensor()
        if not self._beg_initiator:
            self.__position_steps -= 1
        else:
            self.stop()

    def __initiators_sensor(self):
        if self.__position_steps >= self.end:
            self._end_initiator = True
        else:
            self._end_initiator = False

        if self.__position_steps <= self.beginning:
            self._beg_initiator = True
        else:
            self._beg_initiator = False

    def __freq(self):
        return self.parameter_values['Lauffrequenz']


# if __name__ == '__main__':
