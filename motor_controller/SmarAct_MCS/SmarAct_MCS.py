import logging
import threading
import time
from copy import deepcopy
from typing import Union, Dict, List, Tuple

from motor_controller.interface.interface import ContrCommunicator, SerialConnector, Connector, ReplyError, \
    NoReplyError, ControllerError, NotSupportedError, Box

import logscolor

if __name__ == '__main__':
    logscolor.init_config()


def decode_error(er_code: int):
    """Entziffert die Fehlernummer"""
    if er_code == 1:
        return "Syntax Error"
    elif er_code == 2:
        return "Invalid Command Error"
    elif er_code == 3:
        return "Overflow Error"
    elif er_code == 4:
        return "Parse Error"
    elif er_code == 5:
        return "Too Few Parameters Error"
    elif er_code == 6:
        return "Too Many Parameters Error"
    elif er_code == 7:
        return "Invalid Parameter Error"
    elif er_code == 8:
        return "Wrong Mode Error"
    elif er_code == 129:
        return "No Sensor Present Error"
    elif er_code == 140:
        return "Sensor Disabled Error"
    elif er_code == 141:
        return "Command Overridden Error"
    elif er_code == 142:
        return "End Stop Reached Error"
    elif er_code == 143:
        return "Wrong Sensor Type Error"
    elif er_code == 144:
        return "Could Not Find Reference Mark Error"
    else:
        return f'Unknown error code "{er_code}"'


def MCS_SerialConnector(port: str, timeout: float = 0.2, baudrate: float = 115200) -> SerialConnector:
    return SerialConnector(port=port, beg_symbol=b':', end_symbol=b'\n', timeout=timeout, baudrate=baudrate)


def MCSBoxSerial(port: str, timeout: float = 0.2, baudrate: float = 115200, input_file: str = None) -> Box:
    connector = MCS_SerialConnector(port=port, timeout=timeout, baudrate=baudrate)
    communicator = MCSCommunicator(connector)
    return Box(communicator=communicator, input_file=input_file)


class MCSCommunicator(ContrCommunicator):

    tolerance = 30*10**3  # Für MCS akzeptabele Abweichung bei Positionierung der Motoren (in Controller Einheiten: nm)
    calibration_shift = 500 * 10**6

    # Dict, mit den Defaultwerten der Parametern.
    PARAMETER_DEFAULT = {'Sensor Type': 1, 'max Frequency': 5000, 'max move Speed': 0}
    PARAMETER_COMMAND = {'Sensor Type': b'ST', 'max Frequency': b'CLF', 'max move Speed': b'CLS'}

    def __init__(self, connector: Connector):
        self.connector = connector
        self.__mutex = threading.Lock()

        self.__set_sync_communication_mode()

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

    def __set_sync_communication_mode(self):
        self.command_with_proof_reply(b'SCM0')

    def __get_int_parameter(self, command_root: bytes, reply_root: bytes, bus: int, axis: int) -> int:
        command = command_root + str(axis).encode()
        reply = self.command(command, bus)

        prefix = reply_root + str(axis).encode() + b','
        if len(reply) < len(prefix) + 1:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

        try:
            value = int(reply[len(prefix):])
        except ValueError:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

        return value

    def go(self, shift: float, bus: int, axis: int):
        """Verschiebt den angegeben Motor um die angegebene Verschiebung."""

        command = f'MPR{axis},{round(shift)},0'.encode()
        self.command_with_proof_reply(command, bus)

    def go_to(self, destination: float, bus: int, axis: int):
        """Schickt den angegeben Motor zur angegebene absolute Position."""

        command = f'MPA{axis},{round(destination)},0'.encode()
        self.command_with_proof_reply(command, bus)

    def stop(self, bus: int, axis: int):
        """Stoppt den angegebenen Motor."""

        command = f'S{axis}'.encode()
        self.command_with_proof_reply(command, bus)

    def get_position(self, bus: int, axis: int) -> float:
        """Gibt die Position des angegebenen Motors zurück."""

        return self.__get_int_parameter(b'GP', b'P', bus, axis)

    def set_position(self, new_position: float, bus: int, axis: int):
        """Ändert den Wert des Positionzählers im Controller für die angegebene Achse."""

        self.command(f'SP{axis},{round(new_position)}'.encode(), bus)

    def get_parameter(self, parameter_name: str, bus: int, axis: int) -> float:
        """Liest den Wert des angegebenen Parameters."""

        root = self.PARAMETER_COMMAND[parameter_name]
        return self.__get_int_parameter(b'G'+root, root, bus, axis)

    def set_parameter(self, parameter_name: str, neu_value: float, bus: int, axis: int):
        """Ändert den Wert des angegebenen Parameters."""

        root = self.PARAMETER_COMMAND[parameter_name]
        self.command(f'S{root.decode()}{axis},{round(neu_value)}'.encode(), bus)

    def __get_status_code(self, bus: int, axis: int) -> int:
        """Liest ChannelStatusCode des Motors und gibt den zurück."""

        return self.__get_int_parameter(b'GS', b'S', bus, axis)

    def motor_stand(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Motor im Moment steht(True) oder fährt(False)."""

        status_code = self.__get_status_code(bus, axis)
        if status_code == 0:
            return True
        else:
            return False

    def motor_at_the_beg(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Anfang-Initiator im Moment aktiviert ist."""
        raise NotSupportedError("MCS-Kontroller unterschtützt Initiatoren nicht.")

    def motor_at_the_end(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der End-Initiator im Moment aktiviert ist."""
        raise NotSupportedError("MCS-Kontroller unterschtützt Initiatoren nicht.")

    def bus_list(self) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Bus-Nummern zurück."""
        return (0,)

    def bus_check(self, bus: int) -> (bool, str):
        """Prüft ob ein Modul mit angegebenen Bus-Nummer vorhanden/verbunden ist. Gibt ein bool-Wert
        und ein Nachricht zurück, falls kein Modul gefunden wurde."""

        if bus in self.bus_list():
            return True, ""
        else:
            return False, f"Bus {bus} ist nicht vorhanden."

    def axes_list(self, bus: int) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Achsen zurück."""

        reply = self.command(b'GNC', bus)
        reply = reply.decode()
        if reply[0] != 'N':
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

        try:
            n = int(reply[1:])
        except ValueError:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply}!')

        return tuple(range(n))

    def check_connection(self) -> (bool, bytes):
        """Prüft ob es bei dem Port tatsächlich ein Controller gibt, und gibt die Version davon zurück."""

        success, reply = self.command_to_box(b'GIV')
        if success and reply is not None:
            reply_str = reply.decode()
            if reply_str[:2] == 'IV':
                return True, reply
        return False, reply

    def command_to_box(self, command: bytes) -> (bool, Union[bytes, None]):
        """Ausführt ein Befehl ohne Adressieren und gibt die Antwort zurück."""

        self.__mutex.acquire()
        self.connector.send(command)
        reply = self.connector.read()
        self.__mutex.release()

        if reply == b'':
            return False, None
        else:
            reply_s = reply.decode()
            if reply_s[0] == "E":
                split_reply = reply_s[1:].split(',')
                if len(split_reply) == 2:
                    if split_reply[0] == '-1' or split_reply[0].isdigit():
                        er_code = int(split_reply[1])
                        if er_code == 0:
                            return True, None
                        else:
                            return False, f"Der Controller meldet: {decode_error(er_code)}".encode()

                return False, f"Unerwartete Antwort vom Controller: {reply}".encode()
            else:
                return True, reply

    def command_to_modul(self, command: bytes, bus: int = 0) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Modul adressierte Befehl und gibt die Antwort zurück."""
        if bus != 0:
            raise ValueError(f'Bei MCC ist nur bus = 0 verfügbar, und kein "{bus}"!')
        return self.command_to_box(command)

    def command_to_motor(self, command: bytes, bus: int, axis: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Motor adressierte Befehl und gibt die Antwort zurück."""
        raise NotSupportedError("Ein Befehl zum Motor ist bei MCC semantisch nicht implementiert.")

    def command(self, command: bytes, bus: int = 0) -> Union[bytes, None]:

        success, reply = self.command_to_modul(command, bus)
        if not success:
            if reply is None:
                raise NoReplyError('Der Controller antwortet nicht!')
            else:
                raise ControllerError(reply.decode())
        else:
            return reply

    def command_with_proof_reply(self, command: bytes, bus: int = 0):
        """Ausführt ein Befehl ohne erwartete Antwort"""
        reply = self.command(command, bus)
        if reply is not None:
            raise ReplyError(f'Unerwartete Antwort vom Controller: {reply[1]}!')

    # TODO schreiben check_raw_input_data zu Ende
    def check_raw_input_data(self, raw_input_data: List[dict]) -> (bool, str):
        """Prüft ob die rohe Daten aus der input-Datei kompatibel sind."""

        return True, ""

    def calibrate(self, bus: int, axis: int):
        """Die standarte Justierungmaßnahmen durchführen, wenn der Kontroller welche unterstützt."""

        self.command(f'CS{axis}'.encode(), bus=bus)

    def find_reference_mark(self, bus: int, axis: int):

        self.command(f'FRM{axis},0,0,0'.encode(), bus=bus)


if __name__ == '__main__':

    port = '/dev/cu.usbserial-1430'
    connector = MCS_SerialConnector(port)
    ch = 1

    # connector.send(b'GSI')
    # print(connector.read())
    #
    # connector.send(b'GIV')
    # print(connector.read())
    #
    # connector.send(f'S{ch}'.encode())
    # print(connector.read())
    #
    # connector.send(f'SCLS{ch},2000'.encode())
    # print(connector.read())
    #
    # connector.send(f'GCLS{ch}'.encode())
    # print(connector.read())

    # connector.send(f'CS{ch}'.encode())
    # print(connector.read())

    # connector.send(f'FRM{ch},0,2000,1'.encode())
    # print(connector.read())
    #
    # connector.send(b'MPR2,500000000,0')
    # print(connector.read())

    # connector.send(f'MPA{ch},-500000000,10000'.encode())
    # print(connector.read())

    # connector.send(f'MST{ch},3000000,4095,18500'.encode())
    # print(connector.read())

    # while True:
    #
    #     connector.send(f'GS{ch}'.encode())
    #     print(connector.read())
    #     connector.send(f'GP{ch}'.encode())
    #     print(connector.read())
    #     time.sleep(0.5)

    # connector.send(b'SST0,1')
    # print(connector.read())
    #
    connector.send(b'GST2')
    print(connector.read())


    # while True:
    #     print('move!')
    #     connector.send(f'MST{ch},-20000,4095,18500'.encode())
    #     print(connector.read())
    #
    #     status = b''
    #     while status != f'S{ch},0'.encode():
    #         # connector.send(f'MST{ch},-20000,4095,18500'.encode())
    #         # print(connector.read())
    #
    #         connector.send(f'GS{ch}'.encode())
    #         status = connector.read()
    #         print(status)
    #         connector.send(f'GP{ch}'.encode())
    #         print(connector.read())
    #         time.sleep(0.5)
    #     time.sleep(0.5)
