import concurrent.futures
import csv
import logging
import socket
import telnetlib
import threading
import time
from copy import deepcopy
from typing import Dict, List, Tuple, Union, Set, Callable, Iterable

import serial.tools.list_ports
from math import isclose
from serial import Serial

import logscolor

if __name__ == '__main__':
    logscolor.init_config()


class Connector:
    """Ein Objekt, durch das die Kommunikation zwischen dem Programm und dem Controller stattfindet."""

    beg_symbol: bytes
    end_symbol: bytes

    def message_format(self, message: bytes) -> bytes:
        return self.beg_symbol + message + self.end_symbol

    def reply_format(self, reply: bytes) -> Union[bytes, None]:
        if reply == b'':
            return None
        f_reply = deepcopy(reply)
        err = ReplyError(f'Unerwartete Antwort: "{reply}"')

        if self.beg_symbol:
            if reply[:len(self.beg_symbol)] == self.beg_symbol:
                f_reply = f_reply[len(self.beg_symbol):]
            else:
                raise err
        if self.end_symbol:
            if reply[-len(self.end_symbol):] == self.end_symbol:
                f_reply = f_reply[:-len(self.end_symbol)]
            else:
                raise err
        return f_reply

    def send(self, message: bytes, clear_buffer=True):
        """Schickt ein Nachricht zum Controller."""
        raise NotImplementedError

    def read(self) -> Union[bytes, None]:
        """Liest ein Nachricht von dem Controller bis zum End-Symbol oder bis zum maximale Anzahl von Bytes
         und gibt das zurück."""
        raise NotImplementedError

    def clear_buffer(self):
        """Löscht alle vorher empfangene information aus Buffer"""
        raise NotImplementedError

    def set_timeout(self, timeout: float):
        """Einstellt das Time-out"""
        raise NotImplementedError

    def get_timeout(self) -> float:
        """Einstellt das Time-out"""
        raise NotImplementedError

    def close(self):
        """Führt alle nötige Aktivitäten am Ende der Arbeit durch."""
        raise NotImplementedError


def com_list() -> List[str]:
    """Gibt eine Liste der verfügbaren COM-Ports"""

    comlist = serial.tools.list_ports.comports()
    n_list = []
    for element in comlist:
        n_list.append(element.device)
    return n_list


class SerialEmulator:
    """Interface für eine Emulation von einer Serial-Verbindung."""

    timeout: float = 0

    def write(self, command: bytes):
        raise NotImplementedError

    def read_until(self, end_symbol: bytes) -> bytes:
        raise NotImplementedError

    # noinspection PyPep8Naming
    def flushInput(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError


class SerialConnector(Connector):
    """Connector Objekt für eine Verbindung durch Serial Port."""

    def __init__(self,
                 port: str = '',
                 beg_symbol: bytes = b'',
                 end_symbol: bytes = b'\n',
                 timeout: float = 0.2,
                 baudrate: float = 115200,
                 emulator: SerialEmulator = None):
        if emulator is not None:
            self.ser = emulator
        elif not port:
            raise ValueError('Port muss angegeben werden!')
        else:
            self.ser = Serial(port, baudrate, timeout=timeout)
        self.beg_symbol = beg_symbol
        self.end_symbol = end_symbol

    def send(self, message: bytes, clear_buffer=True):
        """Schickt ein Nachricht zum Controller."""

        if clear_buffer:
            self.clear_buffer()
        self.ser.write(self.message_format(message))

    def read(self) -> Union[bytes, None]:
        """Liest ein Nachricht von dem Controller bis zum bestimmten End-Symbol und gibt das zurück."""

        return self.reply_format(self.ser.read_until(self.end_symbol))

    def clear_buffer(self):
        """Löscht alle vorher empfangene information aus Buffer"""

        self.ser.flushInput()

    def set_timeout(self, timeout: float):
        """Stellt das Time-out ein"""

        self.ser.timeout = timeout

    def get_timeout(self) -> float:
        """Gibt den Wert des Time-outs zurück"""

        return self.ser.timeout

    def close(self):
        """Führt alle nötige Aktivitäten am Ende der Arbeit durch."""
        self.ser.close()


class EthernetConnector(Connector):
    """Connector Objekt für eine Verbindung durch Ethernet."""

    def __init__(self, ip: str, port: int, timeout: float = 1, reply_delay: float = 0.002,
                 beg_symbol: bytes = b'', end_symbol: bytes = b'\r\n'):
        self.tn = telnetlib.Telnet(ip, port, timeout=timeout)
        self.__timeout = timeout
        self.reply_delay = reply_delay
        self.beg_symbol = beg_symbol
        self.end_symbol = end_symbol

    def send(self, message: bytes, clear_buffer=True):
        """Schickt ein Nachricht zum Controller."""
        if clear_buffer:
            self.clear_buffer()
        self.tn.write(self.message_format(message))
        # print(self.message_format(message))

    def read(self) -> bytes:
        """Liest ein Nachricht von dem Controller und gibt das zurück."""
        return self.reply_format(self.tn.read_until(self.end_symbol, self.__timeout))

    def clear_buffer(self):
        """Löscht alle vorher empfangene information aus Buffer"""
        time.sleep(self.reply_delay)
        self.tn.read_very_eager()

    def set_timeout(self, timeout: float):
        """Stellt das Time-out ein"""
        self.tn.timeout = timeout
        self.__timeout = timeout

    def get_timeout(self) -> float:
        """Gibt den Wert des Time-outs zurück"""
        return self.__timeout

    def close(self):
        """Führt alle nötige Aktivitäten am Ende der Arbeit durch."""
        self.tn.close()


class ContrCommunicator:
    """Diese Klasse beschreibt die Sprache, die man braucht, um mit einem Controller zu kommunizieren.
    Hier sind alle herstellerspezifische Eigenschaften und Algorithmen zusammen gesammelt"""

    PARAMETER_DEFAULT: Dict
    tolerance: float  # Für diese Controller akzeptabele Abweichung bei Positionierung der Motoren (in Controller Einheiten)
    calibration_shift: float

    def go(self, shift: float, bus: int, axis: int):
        """Verschiebt den angegeben Motor um die angegebene Verschiebung."""
        raise NotImplementedError

    def go_to(self, destination: float, bus: int, axis: int):
        """Schickt den angegeben Motor zur angegebene absolute Position."""
        raise NotImplementedError

    def stop(self, bus: int, axis: int):
        """Stoppt den angegebenen Motor."""
        raise NotImplementedError

    def get_position(self, bus: int, axis: int) -> float:
        """Gibt die Position des angegebenen Motors zurück."""
        raise NotImplementedError

    def set_position(self, new_position: float, bus: int, axis: int):
        """Ändert den Wert des Positionzählers im Controller für die angegebene Achse."""
        raise NotImplementedError

    def get_parameter(self, parameter_name: str, bus: int, axis: int) -> float:
        """Liest den Wert des angegebenen Parameters."""
        raise NotImplementedError

    def set_parameter(self, parameter_name: str, neu_value: float, bus: int, axis: int):
        """Ändert den Wert des angegebenen Parameters."""
        raise NotImplementedError

    def motor_stand(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Motor im Moment steht(True) oder fährt(False)."""
        raise NotImplementedError

    def motor_at_the_beg(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Anfang-Initiator im Moment aktiviert ist."""
        raise NotImplementedError

    def motor_at_the_end(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der End-Initiator im Moment aktiviert ist."""
        raise NotImplementedError

    def bus_check(self, bus: int) -> (bool, str):
        """Prüft ob ein Modul mit angegebenen Bus-Nummer vorhanden/verbunden ist. Gibt ein bool-Wert
        und ein Nachricht zurück, falls kein Modul gefunden wurde."""

        if bus in self.bus_list():
            return True, ""
        else:
            return False, f"Bus {bus} ist nicht vorhanden."

    def bus_list(self) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Bus-Nummern zurück."""
        raise NotImplementedError

    def axes_list(self, bus: int) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Achsen zurück."""
        raise NotImplementedError

    def check_connection(self) -> (bool, bytes):
        """Prüft ob es bei dem Port tatsächlich ein Controller gibt, und gibt die Version davon zurück."""
        raise NotImplementedError

    def command_to_box(self, command: bytes) -> (bool, Union[bytes, None]):
        """Ausführt ein Befehl ohne Adressieren und gibt die Antwort zurück."""
        raise NotImplementedError

    def command_to_modul(self, command: bytes, bus: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Modul adressierte Befehl und gibt die Antwort zurück."""
        raise NotImplementedError

    def command_to_motor(self, command: bytes, bus: int, axis: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Motor adressierte Befehl und gibt die Antwort zurück."""
        raise NotImplementedError

    def check_raw_input_data(self, raw_input_data: List[dict]) -> (bool, str):
        """Prüft ob die rohe Daten aus der input-Datei kompatibel sind."""
        raise NotImplementedError

    def calibrate(self, bus: int, axis: int):
        """Die standarte Justierungmaßnahmen durchführen, wenn der Kontroller welche unterstützt."""
        pass


M_Coord = Tuple[int, int]
Param_Val = Dict[str, float]


def read_csv(address: str, delimiter: str = ';') -> List[Dict[str, str]]:
    """Liest CSV-Datei, und gibt die Liste von Dicts für jede Reihe."""

    with open(address, newline='') as config_file:
        csv.register_dialect('my', delimiter=delimiter)
        data_from_file = list(csv.DictReader(config_file, dialect='my'))

    # Datei prüfen
    defect_error = FileReadError('Die CSV-Datei ist defekt und kann nicht gelesen werden!')
    if not data_from_file:
        raise defect_error
    if None in list(file_row.values() for file_row in data_from_file):
        raise defect_error

    n_columns = len(data_from_file[0])
    for file_row in data_from_file:
        if len(file_row) != n_columns:
            raise defect_error

    return data_from_file


def write_csv_from_dict(addres: str, dict_to_save: dict, delimiter: str = ';'):
    """Schreibt das angegebenes Dict in die CSV-Datei"""
    with open(addres, 'w', newline='') as csvfile:
        csv.register_dialect('my', delimiter=delimiter)
        writer = csv.DictWriter(csvfile, fieldnames=dict_to_save.keys(), dialect='my')
        writer.writeheader()
        writer.writerow(dict_to_save)


def __raw_saved_session_data_is_ok(raw_motors_data: List[dict]) -> bool:
    right_header = ['name', 'position', 'norm_per_contr', 'min_limit', 'max_limit']
    if list(raw_motors_data[0].keys()) != right_header:
        return False

    for motor_line in raw_motors_data:
        if motor_line['min_limit'] != 'None':
            try:
                float(motor_line['min_limit'])
            except ValueError:
                return False
        if motor_line['max_limit'] != 'None':
            try:
                float(motor_line['max_limit'])
            except ValueError:
                return False

        for key, value in motor_line.items():
            if key not in ['name', 'min_limit', 'max_limit']:
                try:
                    float(motor_line[key])
                except ValueError:
                    return False

    # Prüfen ob die Namen einzigartig sind.
    names = []
    for motor_line in raw_motors_data:
        names.append(motor_line['name'])
    if len(names) != len(set(names)):
        return False

    return True


def __transform_raw_saved_session_data(raw_motors_data: List[dict]) \
        -> Dict[str, Tuple[float, float, tuple]]:

    transformed_motors_data = {}
    for motor_line in raw_motors_data:
        name = motor_line['name']
        position = float(motor_line['position'])
        norm_per_contr = float(motor_line['norm_per_contr'])

        min_limit = float(motor_line['min_limit']) if motor_line['min_limit'] != 'None' else None
        max_limit = float(motor_line['max_limit']) if motor_line['max_limit'] != 'None' else None
        soft_limits = (min_limit, max_limit)

        transformed_motors_data[name] = (position, norm_per_contr, soft_limits)

    return transformed_motors_data


def read_saved_session_data_from_file(address: str = 'data/saved_session_data.csv') \
        -> Dict[str, Tuple[float, float, tuple]]:

    raw_data = read_csv(address)
    if __raw_saved_session_data_is_ok(raw_data):
        return __transform_raw_saved_session_data(raw_data)
    else:
        raise FileReadError('Die gelesene Data ist defekt oder inkompatibel!')


def __transform_raw_input_data(raw_config_data: List[dict], communicator: ContrCommunicator) -> List[dict]:
    for motor_line in raw_config_data:

        if motor_line['Mit Initiatoren(0 oder 1)'] != '':
            motor_line['Mit Initiatoren(0 oder 1)'] = int(motor_line['Mit Initiatoren(0 oder 1)'])
        else:
            motor_line['Mit Initiatoren(0 oder 1)'] = Motor.DEFAULT_MOTOR_CONFIG['with_initiators']

        if motor_line['Mit Encoder(0 oder 1)'] != '':
            motor_line['Mit Encoder(0 oder 1)'] = int(motor_line['Mit Encoder(0 oder 1)'])
        else:
            motor_line['Mit Encoder(0 oder 1)'] = Motor.DEFAULT_MOTOR_CONFIG['with_encoder']

        if motor_line['Einheiten'] != '':
            motor_line['Einheiten'] = motor_line['Einheiten']
        else:
            motor_line['Einheiten'] = Motor.DEFAULT_MOTOR_CONFIG['display_units']

        if motor_line['Umrechnungsfaktor'] != '':
            motor_line['Umrechnungsfaktor'] = float(motor_line['Umrechnungsfaktor'])
        else:
            motor_line['Umrechnungsfaktor'] = Motor.DEFAULT_MOTOR_CONFIG['displ_per_contr']

        if 'Inversion(0 oder 1)' in motor_line.keys():
            if motor_line['Inversion(0 oder 1)'] != '':
                motor_line['Inversion(0 oder 1)'] = bool(int(motor_line['Inversion(0 oder 1)']))
            else:
                motor_line['Inversion(0 oder 1)'] = Motor.DEFAULT_MOTOR_CONFIG['inversion']
        else:
            motor_line['Inversion(0 oder 1)'] = Motor.DEFAULT_MOTOR_CONFIG['inversion']

        for parameter_name in communicator.PARAMETER_DEFAULT.keys():
            if parameter_name not in motor_line.keys():
                pass
            elif motor_line[parameter_name] != '':
                motor_line[parameter_name] = float(motor_line[parameter_name])
            else:
                motor_line[parameter_name] = communicator.PARAMETER_DEFAULT[parameter_name]
    return raw_config_data


def read_input_config_from_file(communicator: ContrCommunicator, address: str = 'input/Phytron_Motoren_config.csv') \
        -> (List[int], List[M_Coord], Dict[M_Coord, dict], Dict[M_Coord, Param_Val]):
    raw_config_data = read_csv(address)

    correct, message = communicator.check_raw_input_data(raw_config_data)
    if not correct:
        raise ReadConfigError("Datei hat inkorrekte Data. " + message)

    config_data = __transform_raw_input_data(raw_config_data, communicator)

    # for row in config_data:
    #     print(row.keys(), row.values())

    controllers_to_init = []
    motors_to_init = []
    motors_config = {}
    motors_parameters = {}

    for motor_line in config_data:
        motor_coord = (int(motor_line['Bus']), int(motor_line['Achse']))
        motors_to_init.append(motor_coord)

        if motor_coord[0] not in controllers_to_init:
            controllers_to_init.append(motor_coord[0])

        motor_config = {'name': motor_line['Motor Name'],
                        'with_initiators': motor_line['Mit Initiatoren(0 oder 1)'],
                        'with_encoder': motor_line['Mit Encoder(0 oder 1)'],
                        'inversion': motor_line['Inversion(0 oder 1)'],
                        'display_units': motor_line['Einheiten'],
                        'displ_per_contr': motor_line['Umrechnungsfaktor']}
        motors_config[motor_coord] = motor_config

        motor_parameters = {}
        for parameter_name in communicator.PARAMETER_DEFAULT.keys():
            if parameter_name in motor_line.keys():
                motor_parameters[parameter_name] = motor_line[parameter_name]
        motors_parameters[motor_coord] = motor_parameters

    return controllers_to_init, motors_to_init, motors_config, motors_parameters


class StopIndicator:
    """Durch dieses Objekt kann man Erwartung von dem Stop von allen Motoren abbrechen.
    Es wird als argument für PBox.wait_all_motors_stop() verwendet."""

    def has_stop_requested(self) -> bool:
        raise NotImplementedError


class StandardStopIndicator(StopIndicator):
    """Durch dieses Objekt kann man Erwartung von dem Stop von allen Motoren abbrechen.
    Es wird als argument für PBox.wait_all_motors_stop() verwendet."""

    def __init__(self):
        self.stop_requested = False

    def stop(self):
        self.stop_requested = True

    def restore(self):
        self.stop_requested = False

    def has_stop_requested(self) -> bool:
        return self.stop_requested


class WaitReporter:
    """Durch dieses Objekt kann man die Liste der im Moment laufenden Motoren bekommen.
        Es wird als argument für PBox.wait_all_motors_stop() und andere Funktionen verwendet."""

    def set_wait_list(self, wait_list: Set[str]):
        """Wird am Anfang angerufen. und es wird dadurch """
        raise NotImplementedError

    def motor_is_done(self, motor_name: str):
        raise NotImplementedError


class Controller:
    """Diese Klasse entspricht einem Controller-Modul mit Busnummer 'bus' innerhalb eines Boxes."""

    def __init__(self, communicator: ContrCommunicator, bus: int):

        self.communicator = communicator
        self.bus = bus
        self.motor: Dict[int, Motor] = {}

    def __iter__(self):
        return (motor for motor in self.motor.values())

    def command(self, command: bytes) -> (bool, bytes):
        """Befehl für den Controller ausführen"""

        return self.communicator.command_to_modul(command, self.bus)

    def motors_stand(self) -> bool:
        """Gibt zurück der Status der Motoren, ob die Motoren in Lauf sind."""

        for motor in self:
            if not motor.stand():
                return False
        return True

    def wait_stop(self):
        """Haltet die programme, bis die Motoren stoppen."""

        while not self.motors_stand():
            time.sleep(0.5)

    def make_motors(self):
        """Erstellt Objekten für alle verfügbare Motoren"""

        axes_list = self.communicator.axes_list(self.bus)
        self.motor = {}
        for i in axes_list:
            self.motor[i] = Motor(self, i)
        logging.info(
            f'Controller hat {len(axes_list)} Motor Objekten für alle verfügbare Achsen erstellt, nämlich {axes_list}.')

    def stop(self):
        """Stoppt alle Achsen des Controllers"""

        for motor in self:
            motor.stop()


class Motor:
    """Diese Klasse entspricht einem Motor, der mit einem Controller verbunden ist."""

    DEFAULT_MOTOR_CONFIG = {'with_initiators': 0,
                            'with_encoder': 0,
                            'display_units': 'Schritte',
                            'inversion': False,
                            'norm_per_contr': 1.0,
                            'displ_per_contr': 1.0,
                            'displ_null': 500.0,  # Anzeiger Null in normierte Einheiten
                            'null_position': 0.0  # Position von Anfang in Controller Einheiten
                            }

    def __init__(self, controller: Controller, axis: int):
        self.controller = controller
        self.communicator = self.controller.communicator
        self.axis = axis

        self.name = f'Motor{self.controller.bus}.{self.axis}'
        self.config = deepcopy(self.DEFAULT_MOTOR_CONFIG)
        self.set_parameters()

        self.soft_limits: Tuple[Union[None, float], Union[None, float]] = (None, None)

    def tol(self):
        """Gibt die für diese Controller akzeptabele Abweichung bei Positionierung der Motoren (in displ Einheiten)"""
        return self.transform_units(self.communicator.tolerance, 'contr', to='displ', rel=True)

    def coord(self) -> (int, int):
        """Gibt die Koordinaten des Motors zurück"""

        return self.controller.bus, self.axis

    def go_to(self, destination: float,
              units: str = 'norm',
              wait: bool = False,
              check: bool = False,
              stop_indicator: StopIndicator = None,
              reporter: WaitReporter = None) -> (bool, str):
        """Bewegt den motor zur absoluten Position, die als destination gegeben wird."""

        destination = float(destination)

        # zu normierte einheiten transformieren
        destination = self.transform_units(destination, units, to='norm')

        # Soft Limits prüfen
        bottom, top = self.soft_limits
        if bottom is not None and top is not None:
            if top - bottom < 0:
                err_mess = f'Soft Limits Fehler: Obere Grenze ist kleiner als Untere! ' \
                           f'(Motor {self.axis} beim Controller {self.controller.bus})'
                logging.error(err_mess)
                return False, err_mess
        if bottom is not None:
            if destination < bottom:
                destination = bottom
                if check:
                    return False, f'Der Zielpunkt des Motors "{self.name}" liegt außerhalb der Soft Limits!'
        if top is not None:
            if destination > top:
                destination = top
                if check:
                    return False, f'Der Zielpunkt des Motors "{self.name}" liegt außerhalb der Soft Limits!'

        if not wait:
            self.__go_to(destination, 'norm')
            return True, ""
        else:
            for _ in range(3):
                self.__go_to(destination, 'norm')
                self.wait_motor_stop(stop_indicator)
                if stop_indicator is not None:
                    if stop_indicator.has_stop_requested():
                        self.stop()
                        return False, "Der Vorgang wurde vom Benutzer abgebrochen."

                if not check:
                    if reporter is not None:
                        reporter.motor_is_done(self.name)
                    return True, ""
                else:
                    tolerance_n = self.transform_units(self.communicator.tolerance, 'contr', to='norm', rel=True)
                    if abs(self.position('norm') - destination) <= tolerance_n:
                        if reporter is not None:
                            reporter.motor_is_done(self.name)
                        return True, ""
            return False, f'Der Zielpunkt des Motors "{self.name}" wurde nicht erreicht.'

    def __go_to(self, destination: float, units: str = 'norm'):
        destination = self.transform_units(destination, units, to='contr')
        self.communicator.go_to(self.__invert()*destination, *self.coord())
        logging.info(f'Motor {self.axis} beim Controller {self.controller.bus} wurde zu {destination} geschickt.')

    def go(self, shift: float,
           units: str = 'norm',
           wait: bool = False,
           check: bool = False,
           stop_indicator: StopIndicator = None,
           reporter: WaitReporter = None,
           calibrate: bool = False) -> (bool, str):
        """Bewegt den motor relativ um gegebener Verschiebung."""

        shift = float(shift)
        if shift == 0:
            return True, ""

        if (self.soft_limits != (None, None) and not calibrate) or wait or check:
            shift = self.transform_units(shift, units, to='norm', rel=True)
            position = self.position('norm')
            destination = position + shift
            return self.go_to(destination, 'norm', wait, check, stop_indicator, reporter)

        shift = self.__invert()*self.transform_units(shift, units, to='contr', rel=True)
        self.communicator.go(shift, *self.coord())
        logging.info(f'Motor {self.axis} beim Controller {self.controller.bus} wurde um {shift} verschoben. ')
        return True, ""

    def stop(self):
        """Stoppt die Achse"""

        self.communicator.stop(*self.coord())
        logging.info(f'Motor {self.axis} beim Controller {self.controller.bus} wurde gestoppt.')

    def stand(self):
        """Gibt zurück bool Wert ob Motor steht"""

        return self.communicator.motor_stand(*self.coord())

    def command(self, text):
        """Befehl für den Motor ausführen"""

        return self.communicator.command_to_motor(text, *self.coord())

    def read_parameter(self, parameter_name: str) -> float:
        """Liest einen Parameter Nummer number für die Achse"""

        return self.communicator.get_parameter(parameter_name, *self.coord())

    def set_parameter(self, parameter_name: str, new_value: float):
        """Ändert einen Parameter Nummer number für die Achse"""

        self.communicator.set_parameter(parameter_name, new_value, *self.coord())

    def position(self, units: str = 'norm') -> float:
        """Gibt die aktuelle __position zurück"""

        position = self.__invert() * self.communicator.get_position(*self.coord())
        return self.transform_units(position, 'contr', to=units)

    def __invert(self):

        if self.config['inversion']:
            return -1
        else:
            return 1

    def at_the_end(self):
        """Gibt zurück einen bool Wert, ob der End-Initiator aktiviert ist."""

        if self.with_initiators():
            if self.config['inversion']:
                return self.communicator.motor_at_the_beg(*self.coord())
            else:
                return self.communicator.motor_at_the_end(*self.coord())
        elif self.with_encoder():
            position0 = self.position(units='contr')
            tol = self.communicator.tolerance
            step = 3*tol

            self.go_to(position0 + step, units='contr', wait=True)
            if self.position(units='contr') - position0 > tol:
                self.go_to(position0, units='contr', wait=True)
                return False

            self.go_to(position0 - step, units='contr', wait=True)
            if self.position(units='contr') - position0 < -tol:
                self.go_to(position0, units='contr', wait=True)
                return True
            else:
                raise MotorError("Der Motor sitzt fest und kann sich nicht bewegen.")

        else:
            raise NotSupportedError("Der Motor hat keine Initiators oder Encoder.")

    def at_the_beginning(self):
        """Gibt zurück einen bool Wert, ob der Anfang-Initiator aktiviert ist."""

        if self.with_initiators():
            if self.config['inversion']:
                return self.communicator.motor_at_the_end(*self.coord())
            else:
                return self.communicator.motor_at_the_beg(*self.coord())
        elif self.with_encoder():
            position0 = self.position(units='contr')
            tol = self.communicator.tolerance
            step = 3 * tol

            self.go_to(position0 - step, units='contr', wait=True)
            if self.position(units='contr') - position0 < -tol:
                self.go_to(position0, units='contr', wait=True)
                return False

            self.go_to(position0 + step, units='contr', wait=True)
            if self.position(units='contr') - position0 > tol:
                self.go_to(position0, units='contr', wait=True)
                return True
            else:
                raise MotorError("Der Motor sitzt fest und kann sich nicht bewegen.")

        else:
            raise NotSupportedError("Der Motor hat keine Initiators oder Encoder.")

    def set_position(self, position: float, units: str = 'norm'):
        """Ändern die Zähler der aktuelle Position zu angegebenen Wert"""

        position = float(position)
        self.communicator.set_position(self.transform_units(position, units, to='contr'), *self.coord())
        logging.info(f'__position wurde eingestellt. ({position})')

    def base_calibration(self):
        """Die standarte Justierungmaßnahmen für den Motor durchführen, wenn der Kontroller welche unterstützt."""

        self.communicator.calibrate(*self.coord())
        self.wait_motor_stop()

    def calibrate(self, stop_indicator: StopIndicator = None, reporter: WaitReporter = None, go_to_middle: bool = True):
        """Kalibrierung von den gegebenen Motoren"""
        if self.with_initiators() or self.with_encoder():
            logging.info(f'Kalibrierung vom Motor {self.name} wurde angefangen.')

            calibration_shift = self.communicator.calibration_shift

            self.base_calibration()
            self.wait_motor_stop()
            # Bis zum Ende laufen
            while True:
                self.go(calibration_shift, units='contr', calibrate=True)
                self.wait_motor_stop(stop_indicator)
                if stop_indicator is not None:
                    if stop_indicator.has_stop_requested():
                        if reporter is not None:
                            reporter.motor_is_done(self.name)
                        self.stop()
                        return
                if self.at_the_end():
                    break
            end = self.position('contr')

            # Bis zum Anfang laufen
            while True:
                self.go(-calibration_shift, units='contr', calibrate=True)
                self.wait_motor_stop(stop_indicator)
                if stop_indicator is not None:
                    if stop_indicator.has_stop_requested():
                        if reporter is not None:
                            reporter.motor_is_done(self.name)
                        self.stop()
                        return
                if self.at_the_beginning():
                    break
            beginning = self.position('contr')

            if isclose(end, beginning):
                raise CalibrationError(
                    "Kalibrierung ist fehlgeschlagen, die Endposition ist gleich die Anfangsposition!")

            # Skala normieren
            self.config['norm_per_contr'] = 1000 / (end - beginning)
            self.config['null_position'] = beginning

            # in die Mitte fahren
            if go_to_middle:
                self.go_to(500, 'norm', check=True, wait=True, stop_indicator=stop_indicator)

            if reporter is not None:
                reporter.motor_is_done(self.name)
            logging.info(f'Kalibrierung von Motor {self.name} wurde abgeschlossen.')
        else:
            raise NotSupportedError(f'Motor {self.name} hat keine Initiators und kein Encoder '
                                    f'und kann nicht kalibriert werden!')

    def soft_limits_einstellen(self, soft_limits: Tuple[Union[float, None], Union[float, None]], units: str = 'norm'):
        """soft limits einstellen"""

        def transform(val):
            return self.transform_units(val, units, to='norm') if val is not None else None

        self.soft_limits = tuple(map(transform, soft_limits))

    def wait_motor_stop(self, stop_indicator: Union[StopIndicator, None] = None):
        """Haltet die programme, bis alle Motoren stoppen."""

        time.sleep(0.1)
        while not self.stand():
            if stop_indicator is not None:
                if stop_indicator.has_stop_requested():
                    return
            time.sleep(0.2)

    def set_display_null(self, displ_null: float = None):
        """Anzeiger Null in Normierte Einheiten einstellen"""

        if displ_null is None:
            self.config['displ_null'] = self.position()
        else:
            self.config['displ_null'] = displ_null

    def with_initiators(self) -> bool:
        """Zeigt ob der Motor mit Initiatoren ist."""

        return bool(self.config['with_initiators'])

    def with_encoder(self) -> bool:
        """Zeigt ob der Motor mit Initiatoren ist."""

        return bool(self.config['with_encoder'])

    def is_calibratable(self) -> bool:
        """Zeigt, ob der Motor kalibriert werden können,
        d.h. Initiatoren oder ein Encoder hat.
        """

        return self.with_initiators() or self.with_encoder()

    def set_config(self, motor_config: dict = None):
        """Einstellt Name, Initiatoren Status, display_units, display_u_per_step anhand angegebene Dict"""

        if motor_config is None:
            self.config = deepcopy(self.DEFAULT_MOTOR_CONFIG)
        else:
            for key, value in motor_config.items():
                if key == 'name':
                    self.name = value
                elif key in self.config.keys():
                    self.config[key] = value
                else:
                    raise ValueError(f'Falsche config-key: "{key}"')

    def get_parameters(self) -> Dict[str, float]:
        """Liest die Parametern aus Controller und gibt zurück Dict mit Parameterwerten"""

        parameters_values = {}
        for par_name in self.communicator.PARAMETER_DEFAULT.keys():
            parameter_value = self.read_parameter(par_name)
            parameters_values[par_name] = parameter_value
        return parameters_values

    def set_parameters(self, parameters_values: Dict[str, float] = None):
        """Die Parametern einstellen laut angegebene Dict mit Parameterwerten"""

        # Parameter_Werte = {'Lauffrequenz': 4000, 'Stoppstrom': 5, 'Laufstrom': 11, 'Booststrom': 18}

        if parameters_values is None:
            parameters_values = deepcopy(self.communicator.PARAMETER_DEFAULT)

        for name, value in parameters_values.items():
            self.set_parameter(name, value)

    def transform_units(self, value: float, current_u: str, to: str, rel: bool = False) -> float:
        """Transformiert einen Wert in andere Einheiten.

        Durch diese Funktion erfolgt die Transformation zwischen 3 Einheiten:
        'contr' - Einheiten des Controllers
        'norm' - normierte Einheiten
        'displ' - Anzeiger Einheiten, die durch Benutzer eingestellt werden können.

        Außerdem, wenn rel=False, erfolgt eine vollständige Koordinatentransformation
        mit Berücksichtigung von den Nullstellen des Controllers, des normierten Systems
        und des durch Benutzer eingestellten Anzeiger-Systems.
        """

        units_list = ["norm", "displ", "contr"]
        if current_u not in units_list:
            raise ValueError(f'Unbekante Einheiten! {units_list} wurde erwartet und kein: "{current_u}".')
        elif to not in units_list:
            raise ValueError(f'Unbekante Einheiten! {units_list} wurde erwartet und kein: "{to}".')
        if current_u == to:
            return value

        if current_u == "contr" and to == "norm":
            return self.__contr_to_norm(value, rel)
        elif current_u == "norm" and to == "contr":
            return self.__norm_to_contr(value, rel)
        elif current_u == "norm" and to == "displ":
            value = self.__norm_to_contr(value, rel)
            return self.__contr_to_displ(value, rel)
        elif current_u == "displ" and to == "norm":
            value = self.__displ_to_contr(value, rel)
            return self.__contr_to_norm(value, rel)
        elif current_u == "contr" and to == "displ":
            return self.__contr_to_displ(value, rel)
        elif current_u == "displ" and to == "contr":
            return self.__displ_to_contr(value, rel)

    def __contr_to_norm(self, value: float, rel: bool = False) -> float:
        if not rel:
            value = value - self.config['null_position']
        value *= self.config['norm_per_contr']
        return value

    def __norm_to_contr(self, value: float, rel: bool = False) -> float:
        value /= self.config['norm_per_contr']
        if not rel:
            value = value + self.config['null_position']
        return value

    def __contr_to_displ(self, value: float, rel: bool = False) -> float:
        if not rel:
            value = value - self.__norm_to_contr(self.config['displ_null'])
        value *= self.config['displ_per_contr']
        return value

    def __displ_to_contr(self, value: float, rel: bool = False) -> float:
        value /= self.config['displ_per_contr']
        if not rel:
            value = value + self.__norm_to_contr(self.config['displ_null'])
        return value


class MotorsCluster:
    """Diese Klasse vereint mehrere Motoren aus desselbe oder verschiedene Kontroller-Boxen
    und lässt die bequem zusammen steuern.

    Wichtig: Namen der Motoren müssen einzigartig sein, da die Motoren mit ihren Namen identifiziert werden.
    """

    def __init__(self, motors: List[Motor] = None):

        if motors is None:
            motors = []

        self.motors: Dict[str, Motor] = {}
        self.add_motors(motors)
        self.__mutex = threading.Lock()

    def __iter__(self):
        return self.motors.values().__iter__()

    def names(self) -> Tuple[str]:
        return tuple(self.motors.keys())

    def add_motors(self, motors: Iterable[Motor]):
        """Fügt neue Motoren in den Cluster hinzu."""

        for motor in motors:
            if motor.name not in self.motors.keys():
                self.motors[motor.name] = motor
            elif self.motors[motor.name] is not motor:
                raise MotorNamesError(f'Es gibt die wiederholte Namen der Motoren! '
                                      f'Der Name "{motor.name}" ist mehrmals getroffen.')

    def remove_motors(self, motors: Iterable[Motor]):
        """Entfernt die eingegebene Motoren aus dem Cluster, wenn die da vorhanden sind."""

        m_keys = {v: k for k, v in self.motors.items()}
        for motor in motors:
            if motor in self.motors.values():
                del self.motors[m_keys[motor]]

    def get_motor(self, name: str) -> Motor:
        if name not in self.motors.keys():
            raise ValueError(f"Es gibt kein Motor mit solchem Name: {name}")
        else:
            return self.motors[name]

    def get_motors(self, names: List[str]) -> List[Motor]:
        motors = []
        for name in names:
            if name not in self.motors.keys():
                raise ValueError(f"Es gibt kein Motor mit solchem Name: {name}")
            else:
                motors.append(self.motors[name])
        return motors

    def stop(self):
        """Stoppt alle Motoren im Cluster."""

        for motor in self:
            motor.stop()

    def go_to(self, destinations: Dict[str, float],
              units: str = 'norm',
              wait: bool = False,
              check: bool = True,
              stop_indicator: StopIndicator = None,
              reporter: WaitReporter = None
              ) -> (bool, str):
        """Schickt die angegebene Motoren zu den angegebenen Positionen. Nimmt neue Ziel-Koordinaten im Format von
        Dict {Motorname: Zielposition, ...}
        """

        return self.__move(destinations, 'go_to', units, wait, check, stop_indicator, reporter)

    def go(self, shifts: Dict[str, float],
           units: str = 'norm',
           wait: bool = False,
           check: bool = True,
           stop_indicator: StopIndicator = None,
           reporter: WaitReporter = None
           ) -> (bool, str):
        """Verschiebt die angegebene Motoren zu den angegebenen Verschiebungen im Format von
        Dict {Motorname: Verschiebung, ...}
        """

        return self.__move(shifts, 'go', units, wait, check, stop_indicator, reporter)

    def __move(self, values: Dict[str, float],
               m_type: str,
               units: str = 'norm',
               wait: bool = False,
               check: bool = True,
               stop_indicator: StopIndicator = None,
               reporter: WaitReporter = None
               ) -> (bool, str):
        """Ruft go oder go_to Methode für die angegebene Motoren."""

        def call_movement(name: str, value: float) -> (bool, str):
            if m_type == 'go_to':
                return self.motors[name].go_to(value, units, wait, check, stop_indicator, reporter)
            elif m_type == 'go':
                return self.motors[name].go(value, units, wait, check, stop_indicator, reporter)
            else:
                raise ValueError

        # Namen in Dict prüfen
        self.__check_names_list(values.keys())

        # Motoren zu den Ziel-Koordinaten schicken
        if not wait:
            for name, destination in values.items():
                call_movement(name, destination)
            return True, ""
        else:
            with concurrent.futures.ThreadPoolExecutor() as executor:
                results = executor.map(call_movement, values.keys(), values.values())

                success = True
                message = ""
                for result in results:
                    if not result[0]:
                        success = False
                        message += result[1] + '\n'
                return success, message

    def path_travel(self, path: List[Dict[str, float]],
                    action: Callable,
                    units: str = 'norm',
                    stop_indicator: StopIndicator = None,
                    reporter: WaitReporter = None) -> list:
        """Bewegt die Motoren zu den Punkten in 'path' und ausführt 'action' in jedem Punkt."""

        res = []
        for point in path:
            if reporter is not None:
                reporter.set_wait_list(set(point.keys()))
            success, mess = self.go_to(point, units, True, True, stop_indicator, reporter)
            if stop_indicator is not None:
                if stop_indicator.has_stop_requested():
                    return res
            if not success:
                raise TravelError(mess)
            res.append(action())
        return res

    def positions(self, units: str = 'norm', motors_list: List[str] = None) -> Dict[str, float]:
        """Gibt zurück die Posotionen der angegebene Motoren. Wenn nichts angegeben wird, dann von allen Motoren."""

        if motors_list is None:
            motors_list = self.motors.keys()
        else:
            # Namen in der Liste prüfen
            self.__check_names_list(motors_list)

        positions = {}
        for motor_name in motors_list:
            positions[motor_name] = self.motors[motor_name].position(units)
        return positions

    def __check_names_list(self, motors_list: List[str]):
        """Prüft ob die Motoren mit angegebenen Namen vorhanden sind."""

        motors_not_in_cluster = set(motors_list) - set(self.motors.keys())
        if motors_not_in_cluster:
            raise ValueError(f"Es gibt keine Motoren mit den Namen: {motors_not_in_cluster}")

    def save_current_positions_in_file(self, address: str, units: str = 'norm', delimiter: str = ';'):
        """Schreibt die jetzige Positionen der Motoren in eine Datei"""

        write_csv_from_dict(address, self.positions(units))

    def read_path_from_file(self, address: str,
                            delimiter: str = ';',
                            decimal: str = '.',
                            check: bool = False) -> List[Dict[str, float]]:
        """Liest die Positionen der Motoren aus einer Datei"""

        data = read_csv(address, delimiter)
        if check:
            try:
                self.__check_names_list(list(data[0].keys()))
            except ValueError as err:
                raise FileReadError(err.args[0])
        for positions in data:
            for key in positions:
                if decimal != '.':
                    positions[key] = positions[key].replace(decimal, '.')
                positions[key] = float(positions[key])
        return data

    def read_positions_from_file(self, address: str,
                                 delimiter: str = ';',
                                 decimal: str = '.',
                                 check: bool = True) -> Dict[str, float]:
        """Liest die Positionen der Motoren aus einer Datei"""

        path = self.read_path_from_file(address, delimiter, decimal, check)
        if len(path) > 1:
            raise FileReadError(f'Es ist mehr als eine Zeile in Datei, nämlich {len(path)} statt eine.')
        return path[0]

    def path_travel_from_file(self, address: str,
                              action: Callable,
                              units: str = 'norm',
                              stop_indicator: StopIndicator = None,
                              reporter: WaitReporter = None,
                              delimiter: str = ';',
                              decimal: str = '.') -> list:
        """Bewegt die Motoren zu den Punkten aus der Datei und ausführt 'action' in jedem Punkt."""

        path = self.read_path_from_file(address, delimiter, decimal)
        return self.path_travel(path, action, units, stop_indicator, reporter)
    
    def all_motors_stand(self) -> bool:
        """Gibt bool Wert zurück, ob alle Motoren stehen."""

        for motor in self:
            if not motor.stand():
                return False
        return True

    def wait_all_motors_stop(self, stop_indicator: Union[StopIndicator, None] = None):
        """Haltet die programme, bis alle Motoren stoppen."""

        while not self.all_motors_stand():
            if stop_indicator is not None:
                if stop_indicator.has_stop_requested():
                    return
            time.sleep(0.5)

    def calibratable_motors(self) -> List[Motor]:
        """Gibt zurück eine Liste der allen Motoren, die kalibriert werden können,
        d.h. Initiatoren oder ein Encoder haben.
        """

        motors_list = []
        for motor in self:
            if motor.is_calibratable():
                motors_list.append(motor)
        return motors_list

    def not_calibratable_motors(self) -> List[Motor]:
        """Gibt zurück eine Liste der allen Motoren, die nicht kalibriert werden können,
        d.h. keine Initiatoren oder kein Encoder haben.
        """

        motors_list = []
        for motor in self:
            if not motor.is_calibratable():
                motors_list.append(motor)
        return motors_list

    def base_calibration(self, motors_to_calibration: List[Motor] = None):
        """Die standarte Justierungmaßnahmen für die angegebene Motoren durchführen,
        wenn der Kontroller welche unterstützt. Wenn nichts angegeben ist, dan von allen Motoren.
        """

        if motors_to_calibration is None:
            motors_to_calibration = self.motors.values()
        for motor in motors_to_calibration:
            motor.base_calibration()

    def calibrate_motors(self, motors_to_calibration: List[Motor] = None,
                         names_to_calibration: List[str] = None,
                         parallel: bool = True,
                         stop_indicator: StopIndicator = None,
                         reporter: WaitReporter = None,
                         go_to_middle: bool = True):
        """Kalibrierung von den gegebenen Motoren. Wenn nichts angegeben ist, dan von allen Motoren."""

        all_motors = False
        if names_to_calibration is None and motors_to_calibration is None:
            motors_to_calibration = self.calibratable_motors()
            self.base_calibration(self.not_calibratable_motors())
            all_motors = True
        elif motors_to_calibration is None:
            motors_to_calibration = self.get_motors(names_to_calibration)

        wait_list = set()
        for motor in motors_to_calibration:
            wait_list.add(motor.name)
        if reporter is not None:
            reporter.set_wait_list(wait_list)

        if parallel:
            with concurrent.futures.ThreadPoolExecutor() as executor:
                executor.map(lambda motor: motor.calibrate(stop_indicator, reporter, go_to_middle), motors_to_calibration)
                executor.shutdown(wait=True)
        else:
            for motor in motors_to_calibration:
                motor.calibrate(stop_indicator, reporter, go_to_middle)

        if all_motors:
            logging.info('Kalibrierung von allen Motoren wurde abgeschlossen.')

    def save_session_data(self, address: str = "data/saved_session_data.txt"):
        """Sichert die Daten der jetzigen Sitzung (wie Soft Limits, Positionen, Kalibrierungsdaten) in einer Datei."""

        def make_csv_row(list_to_convert: list) -> str:
            str_list = list(map(str, list_to_convert))
            return ';'.join(str_list) + '\n'

        # Bevor die Datei geändert wurde, die Daten daraus sichern.
        try:
            saved_data = read_saved_session_data_from_file(address)
        except FileNotFoundError:
            saved_data = {}
        except Exception as err:
            logging.exception(err)
            saved_data = {}

        # Erstmal Positionen und andere Daten von Motoren ablesen, bevor Datei zu ändern (für Sicherheit)
        rows = []
        for name, motor in self.motors.items():
            rows.append([name, motor.position('norm'), motor.config['norm_per_contr'], *motor.soft_limits])

        # Daten der vorhandenen Motoren in die Datei schreiben
        self.__mutex.acquire()
        f = open(address, "wt")

        header = ['name', 'position', 'norm_per_contr', 'min_limit', 'max_limit']
        f.write(make_csv_row(header))

        for row in rows:
            f.write(make_csv_row(row))

        # Daten von den abwesenden Motoren zurück in die Datei schreiben
        if saved_data:
            absent_motors = set(saved_data.keys()) - set(self.motors.keys())
            for name in absent_motors:
                position, norm_per_contr, soft_limits = saved_data[name]
                row = [name, position, norm_per_contr, *soft_limits]
                f.write(make_csv_row(row))

        f.close()
        self.__mutex.release()

        logging.info(f'Die Sitzungsdaten wurde in "{address}" gespeichert.')

    def read_saved_session_data(self, address: str = "data/saved_session_data.txt") -> List[Tuple[int, int]]:
        """Liest die gesicherte Sitzung aus einer Datei und gibt die Liste der Motoren zurück,
        für die keine Daten gefunden wurde.
        """

        saved_data = read_saved_session_data_from_file(address)
        list_to_calibration = []
        success_list = []

        for name, motor in self.motors.items():
            if name in saved_data.keys():
                position, norm_per_contr, soft_limits = saved_data[name]

                motor.config['norm_per_contr'] = norm_per_contr
                motor.config['null_position'] = motor.position('contr') - position / norm_per_contr
                motor.soft_limits = soft_limits

                success_list.append(motor.coord())
            else:
                list_to_calibration.append(motor.coord())

        logging.info(f'Gesicherte Sitzungsdaten für Motoren {success_list} wurde geladen.')
        if list_to_calibration:
            logging.info(f'Motoren {list_to_calibration} brauchen Kalibrierung.')

        return list_to_calibration

    def close(self):
        """Alle nötige am Ende der Arbeit Operationen ausführen."""

        self.stop()
        del self


def make_empty_input_file(communicator: ContrCommunicator,
                          motors: List[Motor],
                          address: str = 'input/input_data(Vorlage).csv'):
    """Erstellt eine Vorlage-Datei mit einer leeren Konfigurationstabelle"""

    f = open(address, "wt")
    separator = ';'

    # Motor liste schreiben
    header = ['Motor Name', 'Bus', 'Achse', 'Mit Initiatoren(0 oder 1)', 'Mit Encoder(0 oder 1)',
              'Einheiten', 'Umrechnungsfaktor']
    for parameter_name in communicator.PARAMETER_DEFAULT.keys():
        header.append(parameter_name)
    header_length = len(header)
    header = separator.join(header)
    f.write(header + '\n')

    for motor in motors:
        motorline = [''] * header_length
        motorline[0] = motor.name
        motorline[1] = str(motor.controller.bus)
        motorline[2] = str(motor.axis)
        motorline = separator.join(motorline)
        f.write(motorline + '\n')

    logging.info('Eine Datei mit einer leeren Konfigurationstabelle wurde erstellt.')


class Box:
    """Diese Klasse entspricht einer Box, die mehrere Controller-Modulen im Busbetrieb enthaltet."""

    def __init__(self, communicator: ContrCommunicator, input_file: str = None, tolerance: float = None):
        self.communicator = communicator

        self.report = ""
        self.controller: Dict[int, Controller] = {}


        if tolerance is None:
            self.tolerance = communicator.tolerance
        else:
            self.tolerance = tolerance

        if input_file is not None:
            self.initialize_with_input_file(input_file)
        else:
            self.initialize()

        self.motors_cluster = MotorsCluster(list(self.motors()))

    def __iter__(self):
        return (controller for controller in self.controller.values())

    def set_tolerance(self, tolerance: float):
        """stellt """
        self.communicator.tolerance = tolerance

    def motors(self):
        """Ein Iterator, der durch alle Motoren des Boxes läuft."""
        for controller in self:
            for motor in controller:
                yield motor

    def get_motors_cluster(self):
        return deepcopy(self.motors_cluster)

    def command(self, text: bytes) -> bytes:
        """Befehl für die Box ausführen"""

        return self.communicator.command_to_box(text)

    def initialize(self):
        """Sucht und macht Objekte für alle verfügbare Controller und Motoren und gibt ein Bericht zurück."""

        logging.info('Box Initialisierung wurde gestartet.')

        report = ""
        n_axes = 0

        self.controller = {}
        for i in self.communicator.bus_list():
            self.controller[i] = Controller(self.communicator, i)

        for controller in self:
            controller.make_motors()
            axes_in_controller = len(controller.motor)
            n_axes += axes_in_controller

            report += f"Controller {controller.bus} ({axes_in_controller} Achsen)\n"

        report = f"Box wurde initialisiert. {len(self.controller)} Controller und {n_axes} Achsen gefunden:\n" + report
        logging.info(report)
        self.report = report
        return report

    def initialize_with_input_file(self, config_file: str = 'input/Phytron_Motoren_config.csv'):
        """Sucht und macht Objekte für alle verfügbare Controller und Motoren. Gibt ein Bericht zurück."""

        def del_motor_from_init(bus: int, axis: int):
            del motors_config[bus, axis]
            del motors_parameters[bus, axis]

        logging.info('Box Initialisierung wurde gestartet.')

        report = ""
        n_motors = 0
        n_controllers = 0
        self.controller = {}

        input_config = read_input_config_from_file(self.communicator, config_file)
        controllers_to_init, motors_to_init, motors_config, motors_parameters = input_config

        # Controller initialisieren
        absent_bus = []
        for bus in controllers_to_init:
            bus_is_available, massage = self.communicator.bus_check(bus)
            if bus_is_available:
                self.controller[bus] = Controller(self.communicator, bus)
                n_controllers += 1
            else:
                if bus not in absent_bus:
                    logging.error(f'Bei Bus Nummer {bus} keinen Kontroller gefunden. Controller Antwort:{massage}')
                    absent_bus.append(bus)
        if len(absent_bus) > 1:
            report += f"Controller {absent_bus} sind nicht verbunden und wurden nicht initialisiert.\n"
            print(report)
        elif len(absent_bus) == 1:
            report += f"Controller {absent_bus[0]} ist nicht verbunden und wurde nicht initialisiert.\n"

        # Motoren initialisieren
        for bus, axis in motors_to_init:
            if bus in absent_bus:
                del_motor_from_init(bus, axis)
            else:
                if axis in self.communicator.axes_list(bus):
                    self.controller[bus].motor[axis] = Motor(self.controller[bus], axis)
                    n_motors += 1
                else:
                    del_motor_from_init(bus, axis)
                    report += f"Achse {axis} ist beim Controller {bus} nicht vorhanden, " \
                              f"der Motor wurde nicht initialisiert.\n"

        self.set_motors_config(motors_config)
        self.set_parameters(motors_parameters)

        report = f"{n_controllers} Controller und {n_motors} Motoren wurde initialisiert:\n" + report
        for controller in self:
            report += f'Controller {controller.bus}: '
            more_then_one = False
            for motor in controller:
                if more_then_one:
                    report += ', '
                report += motor.name
                more_then_one = True
            report += '\n'

        self.report = report
        return report

    def get_motor(self, coordinates: (int, int) = None) -> Motor:
        """Gibt den Motor objekt zurück aus Koordinaten in Format (bus, Achse)"""
        if coordinates not in self.motors_list():
            raise ValueError(f"Es gibt kein Motor mit solchen Koordinaten: {coordinates}")
        bus, axis = coordinates
        return self.controller[bus].motor[axis]

    def get_motor_by_name(self, name: str) -> Motor:
        """Gibt den Motor objekt zurück aus dem Name davon."""

        return self.motors_cluster.get_motor(name)

    def stop(self):
        """Stoppt alle Achsen"""

        for controller in self:
            controller.stop()

    def set_motors_config(self, motors_config: Dict[M_Coord, dict]):
        """Einstellt Name, Initiatoren Status, display_units, AE_in_Schritt der Motoren anhand angegebene Dict"""

        for motor_coord, motor_config in motors_config.items():
            motor = self.get_motor(motor_coord)
            motor.set_config(motor_config)

    def set_parameters(self, motors_config: Dict[M_Coord, Param_Val]):
        """Die Parametern einstellen laut angegebene Dict in Format {(bus, Achse) : Parameterwerte,}"""

        available_motors = self.motors_list()
        for motor_coord, param_values in motors_config.items():
            if motor_coord in available_motors:
                self.get_motor(motor_coord).set_parameters(param_values)
            else:
                logging.warning(f"Motor {motor_coord} ist nicht verbunden und kann nicht konfiguriert werden.")

    def get_parameters(self) -> Dict[M_Coord, Param_Val]:
        """Liest die Parametern aus Controller und gibt zurück Dict mit Parameterwerten"""

        motors_parameters = {}

        for controller in self:
            for motor in controller:
                motors_parameters[(controller.bus, motor.axis)] = motor.get_parameters()

        return motors_parameters

    def get_motors(self, coords: List[M_Coord] = None):
        """Gibt zurück die Liste der Motoren anhand von der Liste der Koordinaten davon."""
        motors = []
        for coord in coords:
            motors.append(self.get_motor(coord))
        return motors

    def get_motors_by_names(self, names: List[str] = None):
        """Gibt zurück die Liste der Motoren anhand von der Liste der Namen davon."""
        motors = []
        for name in names:
            motors.append(self.get_motor_by_name(name))
        return motors

    def make_empty_input_file(self, address: str = 'input/input_data(Vorlage).csv'):
        """Erstellt eine Vorlage-Datei mit einer leeren Konfigurationstabelle"""

        make_empty_input_file(self.communicator, list(self.motors_cluster.motors.values()), address)

    def motors_list(self) -> Tuple[M_Coord]:
        """Gibt zurück eine Liste der allen Motoren in Format: [(bus, Achse), …]"""

        m_list = []
        for controller in self:
            for motor in controller:
                m_list.append(motor.coord())
        return tuple(m_list)

    def controllers_list(self) -> List[int]:
        """Gibt zurück eine Liste der allen Controllern in Format: [bus, ...]"""

        controllers_list = []
        for controller in self:
            controllers_list.append(controller.bus)
        return controllers_list

    def close(self):
        """Alle nötige am Ende der Arbeit Operationen ausführen."""

        self.motors_cluster.stop()
        del self


def add_box_prefix(boxes: Dict[str, Box]):
    """Addiert die Namen der Boxen als Präfixe zu den Namen der Motoren."""

    for box_name, box in boxes.items():
        for controller in box:
            for motor in controller:
                motor.name = box_name + "|" + motor.name


class BoxesCluster(MotorsCluster):
    """Diese Klasse vereint mehrere Kontroller-Boxen und lässt die bequem zusammen steuern."""

    def __init__(self, boxes: Dict[str, Box] = None, box_prefix_is_needed: bool = False):

        super().__init__()
        if boxes is None:
            self.boxes = {}
        else:
            self.boxes = boxes

        for name, box in self.boxes.items():
            self.add_box(box, name, box_prefix_is_needed)

    def add_box(self, box: Box, name: str, box_prefix_is_needed: bool = False):
        """Addiert ein neues Box zum Cluster"""

        if box_prefix_is_needed:
            add_box_prefix({name: box})
        self.boxes[name] = box
        self.add_motors(box.motors())

    def remove_box(self, box: Box):
        """Entfernt die eingegebene Box und die Motoren davon aus dem Cluster, wenn die da vorhanden sind."""

        b_keys = {v: k for k, v in self.boxes.items()}
        if box in self.boxes.values():
            self.remove_motors(box.motors())
            del self.boxes[b_keys[box]]


class SerialError(Exception):
    """Base class for serial port related exceptions."""


class ConnectError(Exception):
    """Grundklasse für die Fehler bei der Verbindung mit einem Controller"""


class ReplyError(Exception):
    """Grundklasse für die Fehler bei der Verbindung mit einem Controller"""


class NoReplyError(ReplyError):
    """Grundklasse für die Fehler bei der Verbindung mit einem Controller"""


class ControllerError(ReplyError):
    """Grundklasse für alle Fehler mit den Controllern"""


class MotorError(Exception):
    """Grundklasse für alle Fehler mit den Motoren"""


class NoMotorError(MotorError):
    """Grundklasse für die Fehler wann Motor nicht verbunden oder kaputt ist"""


class MotorNamesError(Exception):
    """Grundklasse für die Fehler, wann es Problemen mit den Namen der Motoren gibt."""


class FileReadError(Exception):
    """Grundklasse für alle Fehler mit der Datei"""


class PlantError(FileReadError):
    """Grundklasse für die Fehler wenn das Aufbau in Datei falsch beschrieben wurde."""


class ReadConfigError(Exception):
    """Grundklasse für alle Fehler mit Lesen der Configuration aus Datei"""


class UnitsTransformError(Exception):
    """Grundklasse für alle Fehler mit Transformation der Einheiten"""


class CalibrationError(Exception):
    """Grundklasse für alle Fehler mit Transformation der Einheiten"""


class TravelError(Exception):
    """Grundklasse für die Fehler bei der Ausführung von path_travel Methode"""


class NotSupportedError(Exception):
    """Grundklasse für die Fehler, wenn die Ausfürung nicht möglich ist,
    weil das das benutzte Gerät nicht unterschtützt.
    """
