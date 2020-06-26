import logging
import threading
import time
from copy import deepcopy
from typing import Union, Dict, List, Tuple

from motor_controller.MotorControllerInterface import ContrCommunicator, SerialEmulator, Connector, NoReplyError, \
    ReplyError, ControllerError, EthernetConnector, Box
from motor_controller.Phytron_MCC2 import is_h_digit, MCC2Communicator

import logscolor

if __name__ == '__main__':
    logscolor.init_config()


class MCS2Communicator(ContrCommunicator):

    tolerance = 10**6  # Für MCS2 akzeptabele Abweichung bei Positionierung der Motoren (in Controller Einheiten)
    calibration_shift = 50*10**9

    # Dict, mit den Defaultwerten der Parametern.
    PARAMETER_DEFAULT = {'Positioner Type': 300, 'Velocity': 0, 'Acceleration': 0}
    PARAMETER_COMMAND = {'Positioner Type': b':PTYPe', 'Velocity': b':VEL', 'Acceleration': b':ACC'}

    def __init__(self, connector: Connector):
        self.connector = connector
        self.__mutex1 = threading.Lock()
        self.__mutex2 = threading.Lock()
        self.__mutex3 = threading.Lock()

        self.__axen_list = []
        self.__get_axen_list()

    def __get_axen_list(self):
        """Liest wie viel Achsen jeder Modul hat,
        und sichert dieser Information für weitere berechnung der Nummern von Chennels.
        """
        self.__axen_list = []
        for bus in self.bus_list():
            self.__axen_list.append(len(self.axes_list(bus)))

    def __ch_n(self, bus: int, axis: int) -> int:
        return sum(self.__axen_list[:bus]) + axis

    def command_with_float_reply(self, command: bytes, bus: int = None, axis: int = None) -> float:
        """Ausführt ein Befehl mit einer erwarteten Fließkommazahl-Antwort
        und gibt die erhaltene Fließkommazahl zurück.
        """

        reply = self.command(command, bus, axis)
        try:
            return float(reply)
        except ValueError:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

    def command_with_int_reply(self, command: bytes, bus: int = None, axis: int = None) -> int:
        """Ausführt ein Befehl mit einer erwarteten int-Antwort
        und gibt die erhaltene int zurück.
        """

        value = self.command_with_float_reply(command, bus, axis)
        return int(value)

    def command_without_reply(self, command: bytes, bus: int = None, axis: int = None):
        """Ausführt ein Befehl mit einer erwarteten bool Antwort
        und gibt den erhaltenen bool Wert zurück.
        """

        reply = self.command(command, bus, axis)
        if reply is not None:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

    def __command(self, command: bytes, clear_buffer: bool = True) -> bytes:
        """Ausführt ein Befehl und gibt die Antwort zurück, ohne Prüfung der Ausführung."""

        self.__mutex1.acquire()
        self.connector.send(command, clear_buffer)
        reply = self.connector.read()
        self.__mutex1.release()
        return reply

    # def __get_errors_and_raise(self):
    #     """Ruft eine Ausnahme mit Fehlermeldung vom Controller."""
    #
    #     errors = self.get_errors()
    #     if errors:
    #         raise NoReplyError(f'Der Controller antwortet nicht!\n Fehlermeldung vom Controller: {errors}')
    #     else:
    #         raise NoReplyError(f'Der Controller antwortet nicht!\n Der Controller berichtet keine Fehler.')

    def get_errors(self) -> bytes:
        """Fragt den Controller ob Fehler gibt, und gibt ein Bericht mit allen Fehlern zurück.
        Wenn keine Fehler gibt, dann gibt leere bytes-string zurück.
        """

        self.__mutex2.acquire()
        reply = self.__command(b':SYST:ERR:COUN?')

        if reply is None:
            NoReplyError('Der Controller antwortet nicht!')
        try:
            errors_count = int(reply)
        except ValueError:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

        errors = b''
        if errors_count:
            for i in range(errors_count):
                reply = self.__command(b':SYST:ERR:NEXT?')
                # reply_ = reply.split(b',')
                # if len(reply_) != 2:
                #     raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')
                # error_n, mess = reply_
                # try:
                #     error_n = int(error_n)
                # except ValueError:
                #     raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')
                errors += reply + b'\n'
        self.__mutex2.release()
        return errors

    def go(self, shift: float, bus: int, axis: int):
        """Verschiebt den angegeben Motor um die angegebene Verschiebung."""
        channel = self.__ch_n(bus, axis)
        self.command_without_reply(b':MMOD 1', bus, axis)
        self.command_without_reply(f':MOVE{channel} {shift}'.encode())

    def go_to(self, destination: float, bus: int, axis: int):
        """Schickt den angegeben Motor zur angegebene absolute Position."""
        channel = self.__ch_n(bus, axis)
        self.command_without_reply(b':MMOD 0', bus, axis)
        self.command_without_reply(f':MOVE{channel} {destination}'.encode())

    def stop(self, bus: int, axis: int):
        """Stoppt den angegebenen Motor."""
        channel = self.__ch_n(bus, axis)
        self.command_without_reply(f':STOP{channel}'.encode())

    def get_position(self, bus: int, axis: int) -> float:
        """Gibt die Position des angegebenen Motors zurück."""
        return self.command_with_float_reply(b':POS?', bus, axis)

    def set_position(self, new_position: float, bus: int, axis: int):
        """Ändert den Wert des Positionzählers im Controller für die angegebene Achse."""
        self.command_without_reply(f':POS {new_position}'.encode(), bus, axis)

    def get_parameter(self, parameter_name: str, bus: int, axis: int) -> float:
        """Liest den Wert des angegebenen Parameters."""
        return self.command_with_float_reply(self.PARAMETER_COMMAND[parameter_name]+b'?')

    def set_parameter(self, parameter_name: str, neu_value: float, bus: int, axis: int):
        """Ändert den Wert des angegebenen Parameters."""
        self.command_without_reply(self.PARAMETER_COMMAND[parameter_name] + f' {neu_value}'.encode(), bus, axis)

    def __get_chan_prop(self, bus: int, axis: int) -> Tuple[int]:
        int32 = self.command_with_int_reply(b':STATe?', bus, axis)
        bits_feld = list(map(int, format(int32, '016b')))
        bits_feld.reverse()
        return tuple(bits_feld)

    def motor_stand(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Motor im Moment steht(True) oder fährt(False)."""
        return not bool(self.__get_chan_prop(bus, axis)[0])

    def motor_at_the_beg(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Anfang-Initiator im Moment aktiviert ist."""
        return bool(self.__get_chan_prop(bus, axis)[8])

    def motor_at_the_end(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der End-Initiator im Moment aktiviert ist."""
        return self.motor_at_the_beg(bus, axis)

    def bus_list(self) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Bus-Nummern zurück."""
        n_moduls = self.command_with_int_reply(b':DEV:NOBM?')
        return tuple(range(n_moduls))

    def axes_list(self, bus: int) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Achsen zurück."""
        n_axis = self.command_with_int_reply(b':NOMC?', bus)
        return tuple(range(n_axis))

    def check_connection(self) -> (bool, bytes):
        """Prüft ob es bei dem Port tatsächlich ein Controller gibt, und gibt die Version davon zurück."""
        reply = self.__command(b'*IDN?')
        if reply[:7] == b'SmarAct':
            return True
        else:
            return False

    def command_to_box(self, command: bytes) -> (bool, Union[bytes, None]):
        """Ausführt ein Befehl ohne Adressieren und gibt die Antwort zurück."""
        self.__mutex3.acquire()
        try:
            self.get_errors()
            reply = self.__command(command)

            errors = self.get_errors()
            if errors:
                return False, errors
            else:
                return True, reply

        finally:
            self.__mutex3.release()

    def command_to_modul(self, command: bytes, bus: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Modul adressierte Befehl und gibt die Antwort zurück."""
        command = f':MOD{bus}'.encode() + command
        return self.command_to_box(command)

    def command_to_motor(self, command: bytes, bus: int, axis: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Motor adressierte Befehl und gibt die Antwort zurück."""
        chanel = self.__ch_n(bus, axis)
        command = f':CHAN{chanel}'.encode() + command
        return self.command_to_box(command)

    def command(self, command: bytes, bus: int = None, axis: int = None) -> Union[bytes, None]:
        """Ausführt ein Befehl für Motor oder für Controller und gibt die Antwort zurück."""
        if axis is not None and bus is None:
            logging.warning(f'Die Eingabe von axis wurde ignoriert, wenn bus None ist!')
        if bus is None:
            success, reply = self.command_to_box(command)
        elif axis is None:
            success, reply = self.command_to_modul(command, bus)
        else:
            success, reply = self.command_to_motor(command, bus, axis)
        if not success:
            raise ControllerError(f'Der Controller hat einen Fehler gemeldet: {reply}')
        return reply

    def check_raw_input_data(self, raw_input_data: List[dict]) -> (bool, str):
        """Prüft ob die rohe Daten aus der input-Datei kompatibel sind."""
        raise NotImplementedError

    def calibrate(self, bus: int, axis: int):
        """Die standarte Justierungmaßnahmen durchführen, wenn der Kontroller welche unterstützt."""
        channel = self.__ch_n(bus, axis)
        self.command_without_reply(f':CHAN{channel}:CAL:OPT 0'.encode())
        self.command_without_reply(f':CAL{channel}'.encode())
        # self.command_without_reply(b'*WAI')


class MCC2BoxEmulator(SerialEmulator, ContrCommunicator):

    PARAMETER_NUMBER = deepcopy(MCC2Communicator.PARAMETER_NUMBER)
    PARAMETER_DEFAULT = deepcopy(MCC2Communicator.PARAMETER_DEFAULT)
    tolerance = MCC2Communicator.rec_tolerance

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


if __name__ == '__main__':
    ip = '192.168.1.200'
    port = 55551
    connector = EthernetConnector(ip, port, end_symbol=b'\r\n', timeout=0.004)
    communicator = MCS2Communicator(connector)

    # print('connection', communicator.check_connection())
    # print(communicator.bus_list())
    # print(communicator.axes_list(0))
    # communicator.go_to(0*10**9, 0, 0)
    # time.sleep(0.5)
    # print(communicator.motor_stand(0, 0))
    # print(communicator.motor_at_the_beg(0, 0))
    # # communicator.stop(0, 0)
    # print(communicator.get_position(0, 0)/10**9)


    # communicator.calibrate(0, 2)
    # time.sleep(3)
    # print(communicator.motor_stand(0, 2))

    box = Box(communicator)
