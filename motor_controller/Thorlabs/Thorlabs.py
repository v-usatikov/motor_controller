# coding= utf-8
import platform
import threading
from motor_controller.interface import *
from motor_controller.interface import ContrCommunicator

if platform.system() == 'Windows':
    import thorlabs_apt as apt
else:
    apt = None

if __name__ == '__main__':
    logscolor.init_config()


STATUS_IS_AVAILABLE: bool = True


def get_list_of_available_devices() -> List[int]:
    """Gibt die Liste der Seriennummern aller verfügbaren Geräte zurück.
    Wenn keine Geräte gefunden wurden, gibt eine leere Liste zurück.
    """

    if apt is None:
        return []
    apt_list = apt.list_available_devices()
    serial_list = []
    for apt_device in apt_list:
        serial_list.append(apt_device[1])
    return serial_list


class TDC001Communicator(ContrCommunicator):
    """Diese Klasse beschreibt die Sprache, die man braucht, um mit einem Thorlabs TDC001 Controller zu kommunizieren.
    Hier sind alle herstellerspezifische Eigenschaften und Algorithmen zusammen gesammelt"""

    PARAMETER_DEFAULT: Dict = {"min_pos": -1000, "max_pos": 1000, "units": 1, "pitch": 0.5, "accn": 0.4, "max_vel": 0.3}
    tolerance: float = 0.001 # Für diese Controller akzeptabele Abweichung bei Positionierung der Motoren (in Controller Einheiten)
    calibration_shift: float = 20

    def __init__(self):

        if apt is None:
            raise ImportError('Die Thorlabs APT-Bibliothek ist auf dieser Plattform nicht verfügbar.')

        self._axes_list = get_list_of_available_devices()
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
        min_pos, max_pos, units, pitch = self.apt_motor[axis].get_stage_axis_info()
        min_vel, accn, max_vel = self.apt_motor[axis].get_velocity_parameters()

        self.parameters[axis] = {"min_pos": min_pos, "max_pos": max_pos, "units": units, "pitch": pitch, "accn": accn, "max_vel": max_vel}
        return self.parameters[axis][parameter_name]


    def set_parameter(self, parameter_name: str, neu_value: float, bus: int, axis: int):
        """Ändert den Wert des angegebenen Parameters."""

        self._check_bus_axis(bus, axis)
        if parameter_name not in self.parameters[axis]:
            raise ValueError(f"Parameter {parameter_name} ist nicht vorhanden.")

        parameters = self.parameters[axis]
        parameters[parameter_name] = neu_value
        self.apt_motor[axis].set_stage_axis_info(min_pos=parameters["min_pos"], max_pos=parameters["max_pos"], units=parameters["units"], pitch=parameters["pitch"])
        self.apt_motor[axis].set_velocity_parameters(0, parameters["accn"], parameters["max_vel"])
        self.parameters[axis] = parameters

    def motor_stand(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Motor im Moment steht(True) oder fährt(False)."""

        if STATUS_IS_AVAILABLE:
            self._check_bus_axis(bus, axis)
            return not self.apt_motor[axis].is_in_motion
        else:
            return True

    def motor_at_the_beg(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der Anfang-Initiator im Moment aktiviert ist."""

        if STATUS_IS_AVAILABLE:
            self._check_bus_axis(bus, axis)
            return self.apt_motor[axis].is_reverse_hardware_limit_switch_active
        else:
            return False

    def motor_at_the_end(self, bus: int, axis: int) -> bool:
        """Zeigt, ob der End-Initiator im Moment aktiviert ist."""

        if STATUS_IS_AVAILABLE:
            return self.apt_motor[axis].is_reverse_hardware_limit_switch_active
        else:
            return False

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
