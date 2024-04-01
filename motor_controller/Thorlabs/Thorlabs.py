# coding= utf-8
import threading
from motor_controller.interface import *
import thorlabs_apt as apt
from motor_controller.interface import ContrCommunicator

if __name__ == '__main__':
    logscolor.init_config()

class TDC001Communicator(ContrCommunicator):
    """Diese Klasse beschreibt die Sprache, die man braucht, um mit einem Thorlabs TDC001 Controller zu kommunizieren.
    Hier sind alle herstellerspezifische Eigenschaften und Algorithmen zusammen gesammelt"""

    PARAMETER_DEFAULT: Dict = {"min_pos": 5, "max_pos": 50, "units": 1, "pitch": 0.5, "accn": 0, "max_vel": 0}
    tolerance: float  # Für diese Controller akzeptabele Abweichung bei Positionierung der Motoren (in Controller Einheiten)
    calibration_shift: float = 1

    def __init__(self, serial_number: int):

        self._axes_list = apt.list_available_devices()
        self.apt_motor: Dict[int, apt.Motor] = {}
        self.parameters: Dict[int, Dict] = {}
        for axis in self._axes_list:
            self.apt_motor[axis] = apt.Motor(axis)
            self.parameters[axis] = self.PARAMETER_DEFAULT.copy()

    def _check_bus_axis(self, bus: int, axis: int):

        if bus not in self.bus_list():
            raise ValueError(f"Bus {bus} ist nicht vorhanden.")
        if axis not in self.axes_list(bus):
            raise ValueError(f"Achse {axis} ist nicht vorhanden.")

    def go(self, shift: float, bus: int, axis: int):
        """Verschiebt den angegeben Motor um die angegebene Verschiebung."""

        self._check_bus_axis(bus, axis)
        self.apt_motor[axis].move_by(shift)

    def go_to(self, destination: float, bus: int, axis: int):
        """Schickt den angegeben Motor zur angegebene absolute Position."""

        self._check_bus_axis(bus, axis)
        self.apt_motor[axis].move_to(destination)

    def stop(self, bus: int, axis: int):
        """Stoppt den angegebenen Motor."""

        self._check_bus_axis(bus, axis)
        self.apt_motor[axis].stop_profiled()

    def get_position(self, bus: int, axis: int) -> float:
        """Gibt die Position des angegebenen Motors zurück."""

        self._check_bus_axis(bus, axis)
        return self.apt_motor[axis].position

    def set_position(self, new_position: float, bus: int, axis: int):
        """Ändert den Wert des Positionzählers im Controller für die angegebene Achse."""

        raise NotImplementedError('Der Controller unterstützt diese Funktion nicht.')

    def get_parameter(self, parameter_name: str, bus: int, axis: int) -> float:
        """Liest den Wert des angegebenen Parameters."""

        self._check_bus_axis(bus, axis)
        if parameter_name not in self.parameters[axis]:
            raise ValueError(f"Parameter {parameter_name} ist nicht vorhanden.")
         = self.apt_motor[axis].get_stage_axis_info()

    def set_parameter(self, parameter_name: str, neu_value: float, bus: int, axis: int):
        """Ändert den Wert des angegebenen Parameters."""
        raise NotImplementedError

    def motor_stand(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Motor im Moment steht(True) oder fährt(False)."""

        self._check_bus_axis(bus, axis)
        return not self.apt_motor[axis].is_in_motion

    def motor_at_the_beg(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Anfang-Initiator im Moment aktiviert ist."""

        self._check_bus_axis(bus, axis)
        return self.apt_motor[axis].is_reverse_hardware_limit_switch_active

    def motor_at_the_end(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der End-Initiator im Moment aktiviert ist."""

        return self.apt_motor[axis].is_reverse_hardware_limit_switch_active

    def bus_check(self, bus: int) -> (bool, str):
        """Prüft ob ein Modul mit angegebenen Bus-Nummer vorhanden/verbunden ist. Gibt ein bool-Wert
        und ein Nachricht zurück, falls kein Modul gefunden wurde."""

        return bus == 0, ''

    def bus_list(self) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Bus-Nummern zurück."""

        return (0,)

    def axes_list(self, bus: int) -> Tuple[int]:
        """Gibt die Liste der allen verfügbaren Achsen zurück."""

        return tuple(self._axes_list)

    def check_connection(self) -> (bool, bytes):
        """Prüft ob es bei dem Port tatsächlich ein Controller gibt, und gibt die Version davon zurück."""

        for apt_motor in self.apt_motor.values():
            try:
                info = apt_motor.hardware_info
                return True, info
            except Exception:
                pass
        return False, b''

    def command_to_box(self, command: bytes) -> (bool, Union[bytes, None]):
        """Ausführt ein Befehl ohne Adressieren und gibt die Antwort zurück."""

        NotImplementedError('Der Controller unterstützt diese Funktion nicht.')

    def command_to_modul(self, command: bytes, bus: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Modul adressierte Befehl und gibt die Antwort zurück."""

        NotImplementedError('Der Controller unterstützt diese Funktion nicht.')

    def command_to_motor(self, command: bytes, bus: int, axis: int) -> (bool, Union[bytes, None]):
        """Ausführt ein zum Motor adressierte Befehl und gibt die Antwort zurück."""

        NotImplementedError('Der Controller unterstützt diese Funktion nicht.')

    def check_raw_input_data(self, raw_input_data: List[dict]) -> (bool, str):
        """Prüft ob die rohe Daten aus der input-Datei kompatibel sind."""

        return True, ''

    def calibrate(self, bus: int, axis: int):
        """Die standarte Justierungmaßnahmen durchführen, wenn der Kontroller welche unterstützt."""
        pass


# if __name__ == '__main__':
