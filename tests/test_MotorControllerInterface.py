import concurrent.futures
import os
import socket
import threading
from copy import deepcopy
from threading import Thread
from time import sleep
from typing import Set
from unittest import TestCase, main

# from MotorController.MotorControllerInterface import *
from motor_controller.interface import Connector, ReplyError, Controller, Motor, CalibrationError, Box, \
    read_input_config_from_file, read_saved_session_data_from_file, read_csv, EthernetConnector, MotorNamesError, \
    BoxesCluster, StopIndicator, WaitReporter, FileReadError, NotSupportedError
from motor_controller.Phytron_MCC2 import MCC2BoxEmulator, MCC2Communicator


class TCPServerEmulator(threading.Thread):

    def __init__(self, ip: str = 'localhost', port: int = 8001, listen=True):
        super().__init__()
        self.s_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_out.bind((ip, port))
        self.__stop_signal = False
        self.last_message = b''
        self.conn = None
        self.listen = listen
        self.start()

    def run(self) -> None:
        self.s_out.listen(1)
        self.conn, addr = self.s_out.accept()
        self.s_out.settimeout(0.01)
        while not self.__stop_signal:
            if self.listen:
                try:
                    message = self.conn.recv(1000)
                    # print('server got: ', message)
                except socket.timeout:
                    pass
                else:
                    for line in message.split(b'\r\n'):
                        if line:
                            self.last_message = line
                            self.conn.send(b'Reply to:' + line + b'\r\n')
                            # print('server got line: ', line)
            else:
                sleep(0.01)
        self.conn.close()
        self.s_out.close()

    def send(self, message: bytes):
        if self.conn is None:
            raise Exception('There is no connection!')
        self.conn.send(message)

    def close(self):
        self.__stop_signal = True
        sleep(0.015)
        # self.conn.shutdown(socket.SHUT_RDWR)
        # self.conn.close()
        # self.s_out.close()


class TestConnector(TestCase):
    connector = Connector()
    connector.end_symbol = b' egg'
    connector.beg_symbol = b'bgg '

    def test_reply_format(self):
        res = self.connector.reply_format(b'bgg hello! egg')
        self.assertEqual(res, b'hello!')

    def test_mess_format(self):
        res = self.connector.message_format(b'hello!')
        self.assertEqual(res, b'bgg hello! egg')

    def none_reply(self):
        res = self.connector.reply_format(b'')
        self.assertEqual(res, None)

    def test_false_reply1(self):
        with self.assertRaises(ReplyError):
            self.connector.reply_format(b'egghello! egg')

    def test_false_reply2(self):
        with self.assertRaises(ReplyError):
            self.connector.reply_format(b'bgg hello! eg')


class TestEthernetConnector(TestCase):
    def test_read(self):
        port = 8005
        server = TCPServerEmulator(port=port, listen=False)
        connector = EthernetConnector('localhost', port, 0.01, end_symbol=b'\r\n')
        try:
            sleep(0.01)
            server.send(b'hello!\r\n')
            self.assertEqual(b'hello!', connector.read())
            self.assertEqual(None, connector.read())

            with self.assertRaises(ReplyError):
                server.send(b'hello!')
                print(connector.read())

            server.send(b'hello1\r\nhello2\r\nhello3\r\n')
            self.assertEqual(b'hello1', connector.read())
            self.assertEqual(b'hello2', connector.read())
            self.assertEqual(b'hello3', connector.read())
        finally:
            connector.close()
            server.close()

    def test_clear_buffer(self):
        port = 8005
        server = TCPServerEmulator(port=port, listen=False)
        connector = EthernetConnector('localhost', port, 0.01, end_symbol=b'\r\n')
        try:
            sleep(0.01)
            server.send(b'junk')
            server.send(b'hello!\r\n')
            self.assertEqual(b'junkhello!', connector.read())

            server.send(b'junk')
            sleep(0.01)
            connector.clear_buffer()
            server.send(b'hello!\r\n')
            self.assertEqual(b'hello!', connector.read())
        finally:
            connector.close()
            server.close()

    def test_send(self):
        port = 8005
        server = TCPServerEmulator(port=port)
        connector = EthernetConnector('localhost', port, 0.01, end_symbol=b'\r\n')
        try:
            connector.send(b'hello1', clear_buffer=False)
            connector.send(b'hello2', clear_buffer=False)
            sleep(0.1)
            self.assertEqual(b'Reply to:hello1', connector.read())
            self.assertEqual(b'Reply to:hello2', connector.read())
        finally:
            connector.close()
            server.close()


class TestController(TestCase):
    def test_make_motors(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=5, realtime=False)
        controller = Controller(emulator_box, 1)

        controller.make_motors()
        self.assertEqual(5, len(controller.motor))
        self.assertEqual([1, 2, 3, 4, 5], list(controller.motor.keys()))
        for i in range(1, 6):
            motor = controller.motor[i]
            self.assertEqual(Motor, type(motor))
            self.assertEqual(1, motor.controller.bus)
            self.assertEqual(i, motor.axis)
            self.assertEqual(Motor.DEFAULT_MOTOR_CONFIG, motor.config)


def preparation_to_test(realtime=False):
    emulator = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=realtime)
    controller = Controller(emulator, 1)
    motor = Motor(controller, 2)

    # 'contr' = Kelvin, 'norm' = Celsius, 'displ' = Fahrenheit
    motor.set_config({'display_units': 'F',
                      'norm_per_contr': 1,
                      'displ_per_contr': 9 / 5,
                      'displ_null': -17.7778,  # Anzeiger Null in normierte Einheiten
                      'null_position': 273.15  # Position von Anfang in Controller Einheiten
                      })
    step = 0.1
    emulator.set_parameter('Umrechnungsfaktor(Contr)', step, 1, 2)

    motor_emulator = emulator.controller[1].motor[2]
    return motor, step, motor_emulator


class StopTestIndicator(StopIndicator):
    """StopIndocator für Testen"""

    def __init__(self):
        self.stop = False

    def has_stop_requested(self) -> bool:
        return self.stop


class WaitTestReporter(WaitReporter):
    """WaitReporter für Testen"""

    def __init__(self):
        self.wait_list = []
        self.motors_done = []

    def set_wait_list(self, wait_list: Set[str]):
        """Wird am Anfang angerufen. und es wird dadurch """
        self.wait_list = wait_list

    def motor_is_done(self, motor_name: str):
        self.motors_done.append(motor_name)


class TrevelTestReporter(WaitReporter):
    """WaitReporter für Testen"""

    def __init__(self):
        self.wait_list = set()
        self.history = []

    def set_wait_list(self, wait_list: Set[str]):
        """Wird am Anfang angerufen. und es wird dadurch """
        self.wait_list = wait_list
        self.history.append(deepcopy(self.wait_list))

    def motor_is_done(self, motor_name: str):
        self.wait_list -= {motor_name}
        self.history.append(deepcopy(self.wait_list))


class TestMotor(TestCase):
    def test_set_config(self):
        emulator_box = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        controller = Controller(emulator_box, 1)
        motor = Motor(controller, 2)

        self.assertEqual(Motor.DEFAULT_MOTOR_CONFIG, motor.config)

        new_config = {'with_initiators': 1,
                      'with_encoder': 1,
                      'display_units': 'Pupkini',
                      'norm_per_contr': 4678.34,
                      'displ_per_contr': 123.489,
                      'displ_null': 3245.3,
                      'null_position': 123123.5}
        motor.set_config(new_config)
        self.assertEqual(new_config, motor.config)

        motor.set_config({'display_units': 'ml von Borschtsch', 'displ_null': 225.55, 'name': 'super motor'})
        self.assertEqual('ml von Borschtsch', motor.config['display_units'])
        self.assertEqual(225.55, motor.config['displ_null'])
        self.assertEqual('super motor', motor.name)

        with self.assertRaises(ValueError):
            motor.set_config({'abrakadabra!': 546})

        motor.set_config()
        self.assertEqual(Motor.DEFAULT_MOTOR_CONFIG, motor.config)

    def test_transform_units(self):
        emulator_box = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        controller = Controller(emulator_box, 1)
        motor = Motor(controller, 2)

        # 'contr' = Kelvin*0.123, 'norm' = Celsius, 'displ' = Fahrenheit
        motor.set_config({'display_units': 'F',
                          'norm_per_contr': 1 / 0.123,
                          'displ_per_contr': (9 / 5) / 0.123,
                          'displ_null': -17.7778,  # Anzeiger Null in normierte Einheiten
                          'null_position': 273.15 * 0.123  # Position von Anfang in Controller Einheiten
                          })

        self.assertEqual(round(373.15 * 0.123, 4), round(motor.transform_units(100, 'norm', to='contr'), 4))
        self.assertEqual(212, round(motor.transform_units(100, 'norm', to='displ'), 4))
        self.assertEqual(212, round(motor.transform_units(373.15 * 0.123, 'contr', to='displ'), 4))

        self.assertEqual(-40, round(motor.transform_units(-40, 'norm', to='displ'), 4))
        self.assertEqual(-40, round(motor.transform_units(-40, 'displ', to='norm'), 4))

        self.assertEqual(0, round(motor.transform_units(-459.67, 'displ', to='contr'), 4))

        self.assertEqual(-50, round(motor.transform_units(223.15 * 0.123, 'contr', to='norm'), 4))

        value = 3567.345
        value = motor.transform_units(value, 'contr', to='norm')
        value = motor.transform_units(value, 'norm', to='displ')
        value = motor.transform_units(value, 'displ', to='contr')
        self.assertEqual(3567.345, round(value, 4))

        # relative Transformation
        self.assertEqual(round(1 / 0.123, 4), round(motor.transform_units(1, 'contr', to='norm', rel=True), 4))
        self.assertEqual(0.123, round(motor.transform_units(1, 'norm', to='contr', rel=True), 4))
        self.assertEqual(round((9 / 5) / 0.123, 4), round(motor.transform_units(1, 'contr', to='displ', rel=True), 4))
        self.assertEqual(1, round(motor.transform_units(round((9 / 5) / 0.123, 4), 'displ', to='contr', rel=True), 4))
        self.assertEqual(round(9 / 5, 4), round(motor.transform_units(1, 'norm', to='displ', rel=True), 4))
        self.assertEqual(1, round(motor.transform_units(round(9 / 5, 4), 'displ', to='norm', rel=True), 4))

    def test_get_set_position(self):
        motor = preparation_to_test()[0]

        motor.set_position(0, 'norm')
        self.assertEqual(0, round(motor.position('norm'), 4))
        self.assertEqual(273.15, round(motor.position('contr'), 4))
        self.assertEqual(32, round(motor.position('displ'), 4))

        motor.set_position(373.15, 'contr')
        self.assertEqual(100, round(motor.position('norm'), 4))
        motor.set_position(0, 'norm')
        self.assertEqual(0, round(motor.position('norm'), 4))
        motor.set_position(212, 'displ')
        self.assertEqual(100, round(motor.position('norm'), 4))

    def test_go_to_units(self):
        motor, step, motor_emulator = preparation_to_test()

        motor.go_to(373.15, 'contr')
        motor_emulator.wait_stop()
        self.assertTrue(abs(100 - motor.position('norm')) < 2 * step, f"Ist bei {motor.position('norm')} statt 100")

        motor.go_to(0, 'norm')
        motor_emulator.wait_stop()
        self.assertTrue(abs(motor.position('norm')) < 2 * step, f"Ist bei {motor.position('norm')} statt 0")

        motor.go_to(212, 'displ')
        motor_emulator.wait_stop()
        self.assertTrue(abs(100 - motor.position('norm')) < 2 * step, f"Ist bei {motor.position('norm')} statt 100")

    def test_go_to_wait(self):
        motor, step, motor_emulator = preparation_to_test()
        reporter = WaitTestReporter()
        stoper = StopTestIndicator()

        # Test wait-option
        motor_emulator.set_position(-1000)
        motor.go_to(373.15, 'contr')
        self.assertTrue(motor.position('contr') < 200)

        motor_emulator.set_position(-1000)
        motor.go_to(373.15, 'contr', wait=True)
        self.assertAlmostEqual(100, motor.position('norm'), delta=2 * step)

        # Test stop
        motor_emulator.box.realtime = True
        motor_emulator.set_position(0)
        with concurrent.futures.ThreadPoolExecutor() as executor:
            res = executor.submit(motor.go_to, 373.15, 'contr', True, False, stoper, reporter)
            stoper.stop = True
            self.assertEqual((False, 'Der Vorgang wurde vom Benutzer abgebrochen.'), res.result())
            self.assertTrue(motor.position('contr') < 200)

        # Test Reporter
        motor_emulator.box.realtime = True
        stoper.stop = False
        motor_emulator.set_position(0)
        with concurrent.futures.ThreadPoolExecutor() as executor:
            res = executor.submit(motor.go_to, 373.15, 'contr', True, False, stoper, reporter)
            sleep(0.1)
            self.assertEqual([], reporter.motors_done)
            motor_emulator.box.realtime = False
            self.assertEqual((True, ''), res.result())
            self.assertEqual([motor.name], reporter.motors_done)
            self.assertAlmostEqual(100, motor.position('norm'), delta=2 * step)

    def test_go_to_check(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=True)
        motor_emulator = emulator.controller[1].motor[2]
        motor_emulator.beginning = -10
        motor_emulator.end = 10
        controller = Controller(emulator, 1)
        motor = Motor(controller, 2)
        emulator.set_parameter('Lauffrequenz', 50, 1, 2)

        # Test normal case
        motor_emulator.set_position(0)
        self.assertEqual((True, ''), motor.go_to(5, 'contr', True, True))
        self.assertAlmostEqual(5, motor.position('contr'), delta=2)

        # Test mit unerwartete Abbruch
        motor_emulator.set_position(-10)
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # ohne check
            # TODO herausfinden, wieso der Testergebniss nicht stabil ist.
            res = executor.submit(motor.go_to, 9, 'contr', True, False)
            motor_emulator.stop()
            self.assertEqual((True, ''), res.result())
            self.assertTrue(motor.position('contr') < 8)

            # mit check
            res = executor.submit(motor.go_to, 9, 'contr', True, True)
            motor_emulator.stop()
            self.assertEqual((True, ''), res.result())
            self.assertAlmostEqual(9, motor.position('contr'), delta=2)

        # Test mit einer unerreichbaren Zielposition
        motor_emulator.set_position(0)
        self.assertEqual((False, f'Der Zielpunkt des Motors "{motor.name}" wurde nicht erreicht.'),
                         motor.go_to(2000, 'contr', True, True))
        self.assertAlmostEqual(10, motor.position('contr'), delta=2)

        # Test mit Soft Limits
        motor_emulator.set_position(0)
        motor.soft_limits = (-6.2, 3.2)
        self.assertEqual((False, f'Der Zielpunkt des Motors "{motor.name}" liegt außerhalb der Soft Limits!'),
                         motor.go_to(100, 'contr', True, True))
        self.assertAlmostEqual(0, motor.position('contr'), delta=2)

        self.assertEqual((False, f'Der Zielpunkt des Motors "{motor.name}" liegt außerhalb der Soft Limits!'),
                         motor.go_to(-50.2, 'contr', True, True))
        self.assertAlmostEqual(0, motor.position('contr'), delta=2)

        self.assertEqual((True, ''), motor.go_to(-5, 'contr', True, True))
        self.assertAlmostEqual(-5, motor.position('contr'), delta=2)

        motor_emulator.set_position(0)
        motor.soft_limits = (3, -2)
        self.assertEqual((False, 'Soft Limits Fehler: '
                                 'Obere Grenze ist kleiner als Untere! (Motor 2 beim Controller 1)'),
                         motor.go_to(9, 'contr', True, True))
        self.assertAlmostEqual(0, motor.position('contr'), delta=2)

    def test_go(self):
        motor, step, motor_emulator = preparation_to_test()
        motor.set_position(0, 'norm')

        motor.go(-10.4, 'contr')
        motor_emulator.wait_stop()
        self.assertAlmostEqual(-10.4, motor.position('norm'), delta=2 * step)
        motor.go(30.4, 'norm')
        motor_emulator.wait_stop()
        self.assertAlmostEqual(20, motor.position('norm'), delta=2 * step)
        motor.go(10, 'displ')
        motor_emulator.wait_stop()
        self.assertAlmostEqual(20 + 10 * 5 / 9, motor.position('norm'), delta=2 * step)

    def test_stand(self):
        motor, step, motor_emulator = preparation_to_test(realtime=True)

        motor.set_position(0)

        self.assertTrue(motor.stand())
        motor.go_to(1000)
        self.assertFalse(motor.stand())
        motor_emulator.box.realtime = False
        motor_emulator.wait_stop()
        self.assertTrue(motor.stand())

    def test_stop(self):
        motor, step, motor_emulator = preparation_to_test(realtime=True)

        motor.set_position(0)
        motor.go_to(1000)
        motor_emulator.sleep_steps(2)
        self.assertFalse(motor_emulator.stand())
        motor.stop()
        motor_emulator.sleep_steps(2)
        self.assertTrue(motor_emulator.stand())

    def test_read_parameter(self):
        motor, step, motor_emulator = preparation_to_test()

        for param_name in MCC2Communicator.PARAMETER_DEFAULT.keys():
            motor_emulator.box.set_parameter(param_name, 134.5, 1, 2)
            self.assertEqual(134.5, motor.read_parameter(param_name))

    def test_set_parameter(self):
        motor, step, motor_emulator = preparation_to_test()

        for param_name in MCC2Communicator.PARAMETER_DEFAULT.keys():
            motor.set_parameter(param_name, 134.5)
            self.assertEqual(134.5, motor_emulator.box.get_parameter(param_name, 1, 2))

    def test_at_the_end(self):
        motor, step, motor_emulator = preparation_to_test()

        # ohne Initiatoren ohne Encoder
        with self.assertRaises(NotSupportedError):
            motor.at_the_end()

        # mit Initiatoren
        motor.config['with_initiators'] = 1

        motor_emulator._end_initiator = True
        self.assertTrue(motor.at_the_end())

        motor_emulator._end_initiator = False
        self.assertFalse(motor.at_the_end())

        # ohne Initiatoren mit Encoder
        motor.config['with_initiators'] = 0
        motor.config['with_encoder'] = 1

        motor_emulator.set_position(0)
        self.assertFalse(motor.at_the_end())

        motor_emulator.set_position(motor_emulator.beginning)
        self.assertFalse(motor.at_the_end())

        motor_emulator.set_position(motor_emulator.end)
        self.assertTrue(motor.at_the_end())

    def test_at_the_beginning(self):
        motor, step, motor_emulator = preparation_to_test()

        # ohne Initiatoren ohne Encoder
        with self.assertRaises(NotSupportedError):
            motor.at_the_beginning()

        # mit Initiatoren
        motor.config['with_initiators'] = 1

        motor_emulator._beg_initiator = True
        self.assertTrue(motor.at_the_beginning())

        motor_emulator._beg_initiator = False
        self.assertFalse(motor.at_the_beginning())

        # ohne Initiatoren mit Encoder
        motor.config['with_initiators'] = 0
        motor.config['with_encoder'] = 1

        motor_emulator.set_position(0)
        self.assertFalse(motor.at_the_beginning())

        motor_emulator.set_position(motor_emulator.end)
        self.assertFalse(motor.at_the_beginning())

        motor_emulator.set_position(motor_emulator.beginning)
        self.assertTrue(motor.at_the_beginning())

    def test_calibrate(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        controller = Controller(emulator, 1)
        motor = Motor(controller, 2)
        motor_emulator = emulator.controller[1].motor[2]

        motor.config['with_initiators'] = False
        with self.assertRaises(CalibrationError):
            motor.calibrate()

        motor.config['with_initiators'] = True
        motor.calibrate()
        motor.go_to(0, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(-10000, motor.position('contr'))

        motor.go_to(1000, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(10000, motor.position('contr'))

    def test_soft_limits(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        controller = Controller(emulator, 1)
        motor = Motor(controller, 2)
        motor_emulator = emulator.controller[1].motor[2]
        motor.set_config({'norm_per_contr': 0.05, 'displ_per_contr': 1.0, 'null_position': -10000})

        motor.go_to(600, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(600, round(motor.position('norm'), 4))
        motor.go_to(400, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(400, round(motor.position('norm'), 4))

        motor.soft_limits = (430.2, 560.4)
        motor.go_to(600, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(560.4, round(motor.position('norm'), 4))
        motor.go_to(400, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(430.2, round(motor.position('norm'), 4))

        motor.soft_limits = (None, 560.4)
        motor.go_to(600, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(560.4, round(motor.position('norm'), 4))
        motor.go_to(400, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(400, round(motor.position('norm'), 4))

        motor.soft_limits = (430.2, None)
        motor.go_to(600, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(600, round(motor.position('norm'), 4))
        motor.go_to(400, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(430.2, round(motor.position('norm'), 4))

        motor.soft_limits = (None, None)
        motor.go_to(600, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(600, round(motor.position('norm'), 4))
        motor.go_to(400, 'norm')
        motor_emulator.wait_stop()
        self.assertEqual(400, round(motor.position('norm'), 4))

    def test_soft_limits_einstellen(self):
        motor, step, motor_emulator = preparation_to_test()

        motor.soft_limits_einstellen((123.45, 345.67), 'norm')
        self.assertEqual((123.45, 345.67), motor.soft_limits)

        motor.soft_limits_einstellen((273.15, 373.15), 'contr')
        self.assertEqual((0, 100), motor.soft_limits)

        motor.soft_limits_einstellen((32, 212), 'displ')
        self.assertEqual((0, 100), tuple(map(round, motor.soft_limits)))

        motor.soft_limits_einstellen((None, 345.67), 'norm')
        self.assertEqual((None, 345.67), motor.soft_limits)

        motor.soft_limits_einstellen((123.45, None), 'norm')
        self.assertEqual((123.45, None), motor.soft_limits)

        motor.soft_limits_einstellen((None, None), 'norm')
        self.assertEqual((None, None), motor.soft_limits)

        motor.soft_limits_einstellen((None, 212), 'displ')
        beg, end = motor.soft_limits
        end = round(end, 4)
        self.assertEqual((None, 100), (beg, end))

        motor.soft_limits_einstellen((32, None), 'displ')
        beg, end = motor.soft_limits
        beg = round(beg, 4)
        self.assertEqual((0, None), (beg, end))

        motor.soft_limits_einstellen((None, None), 'displ')
        self.assertEqual((None, None), motor.soft_limits)

    def test_get_parameters(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        controller = Controller(emulator, 1)
        motor = Motor(controller, 2)

        self.assertEqual(MCC2Communicator.PARAMETER_DEFAULT, motor.get_parameters())

    def test_set_parameters(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        controller = Controller(emulator, 1)
        motor = Motor(controller, 2)

        new_parameters = {'Lauffrequenz': 3435.4,
                          'Stoppstrom': 56456.3454,
                          'Laufstrom': 76576.445,
                          'Booststrom': 3424.45,
                          'Initiatortyp': 1}

        motor.set_parameters(new_parameters)
        parameters = motor.get_parameters()
        for key, value in new_parameters.items():
            self.assertEqual(value, parameters[key])


class TestBox(TestCase):
    def test_initialize(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=5)
        box = Box(emulator)

        right_report = "Box wurde initialisiert. 15 Controller und 75 Achsen gefunden:\n"
        for i in range(15):
            right_report += f"Controller {i} (5 Achsen)\n"
        self.assertEqual(right_report, box.report)
        self.assertEqual(15, len(box.controller))

        for i in range(15):
            controller = box.controller[i]
            self.assertEqual(Controller, type(controller))
            self.assertEqual(5, len(controller.motor))
            for j in range(1, 6):
                motor = controller.motor[j]
                self.assertEqual(Motor, type(motor))
                self.assertEqual(MCC2Communicator.PARAMETER_DEFAULT, motor.get_parameters())
                self.assertEqual(Motor.DEFAULT_MOTOR_CONFIG, motor.config)
                self.assertEqual(f'Motor{i}.{j}', motor.name)

    def test_motor_list(self):
        emulator = MCC2BoxEmulator(n_bus=3, n_axes=2)
        box = Box(emulator)

        right_set = {(0, 1), (0, 2), (1, 1), (1, 2), (2, 1), (2, 2)}

        self.assertEqual(right_set, set(box.motors_list()))

    def test_get_motor(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=5)
        box = Box(emulator)

        for coord in box.motors_list():
            motor = box.get_motor(coord)
            self.assertEqual(coord, motor.coord())

        with self.assertRaises(ValueError):
            box.get_motor((7, 8))

    def test_get_motor_by_name(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=5)
        box = Box(emulator)

        for coord in box.motors_list():
            motor = box.get_motor(coord)
            bus, axis = coord
            self.assertTrue(box.get_motor_by_name(f'Motor{bus}.{axis}') is motor)

        with self.assertRaises(ValueError):
            box.get_motor_by_name(f'FalscheMotor')

    def test_get_parameters(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=5)
        box = Box(emulator)

        parameters = box.get_parameters()

        self.assertEqual(5 * 15, len(parameters))
        self.assertEqual(set(box.motors_list()), set(parameters.keys()))
        for element in parameters.values():
            self.assertEqual(MCC2Communicator.PARAMETER_DEFAULT, element)

        box.get_motor((2, 4)).set_parameter('Lauffrequenz', 34567.34)
        self.assertEqual(34567.34, box.get_parameters()[(2, 4)]['Lauffrequenz'])
        self.assertEqual(MCC2Communicator.PARAMETER_DEFAULT, box.get_parameters()[(0, 2)])

    def test_set_parameters(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=9)
        box = Box(emulator)

        parameters = {
            (0, 1): {'Lauffrequenz': 1000, 'Stoppstrom': 1, 'Laufstrom': 2, 'Booststrom': 3, 'Initiatortyp': 0},
            (12, 4): {'Lauffrequenz': 2000, 'Stoppstrom': 2, 'Laufstrom': 3, 'Booststrom': 4, 'Initiatortyp': 1},
            (3, 6): {'Lauffrequenz': 3000, 'Stoppstrom': 3, 'Laufstrom': 4, 'Booststrom': 5, 'Initiatortyp': 0},
            (12, 3): {'Lauffrequenz': 4001, 'Stoppstrom': 4, 'Laufstrom': 5, 'Booststrom': 6, 'Initiatortyp': 1}}

        box.set_parameters(parameters)
        for coord, param_line in parameters.items():
            param_line_from_box = box.get_motor(coord).get_parameters()
            for name, value in param_line.items():
                self.assertEqual(value, param_line_from_box[name])

        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        box = Box(emulator)

        parameters = {
            (0, 1): {'Lauffrequenz': 1000, 'Stoppstrom': 1, 'Laufstrom': 2, 'Booststrom': 3, 'Initiatortyp': 0},
            (0, 2): {'Lauffrequenz': 2000, 'Stoppstrom': 2, 'Laufstrom': 3, 'Booststrom': 4, 'Initiatortyp': 1},
            (1, 1): {'Lauffrequenz': 3000, 'Stoppstrom': 3, 'Laufstrom': 4, 'Booststrom': 5, 'Initiatortyp': 0},
            (1, 2): {'Lauffrequenz': 4001, 'Stoppstrom': 4, 'Laufstrom': 5, 'Booststrom': 6, 'Initiatortyp': 1}}

        box.set_parameters(parameters)
        for coord, param_line in parameters.items():
            param_line_from_box = box.get_motor(coord).get_parameters()
            for name, value in param_line.items():
                self.assertEqual(value, param_line_from_box[name])

    def test_set_motors_config(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=9)
        box = Box(emulator)

        new_config = {(0, 1): {'name': 'new_motor1',
                               'with_initiators': 1,
                               'with_encoder': 1,
                               'display_units': 'einh1',
                               'norm_per_contr': 1.0,
                               'displ_per_contr': 22.222,
                               'displ_null': 0.0,
                               'null_position': 0.0},
                      (12, 4): {'name': 'new_motor2',
                                'with_initiators': 0,
                                'with_encoder': 1,
                                'display_units': 'einh2',
                                'norm_per_contr': 1.0,
                                'displ_per_contr': 33.333,
                                'displ_null': 0.0,
                                'null_position': 0.0},
                      (3, 6): {'name': 'new_motor3',
                               'with_initiators': 1,
                               'with_encoder': 1,
                               'display_units': 'einh3',
                               'norm_per_contr': 1.0,
                               'displ_per_contr': 44.444,
                               'displ_null': 0.0,
                               'null_position': 0.0},
                      (12, 3): {'name': 'new_motor4',
                                'with_initiators': 1,
                                'with_encoder': 1,
                                'display_units': 'einh4',
                                'norm_per_contr': 1.0,
                                'displ_per_contr': 55.555,
                                'displ_null': 0.0,
                                'null_position': 0.0}}

        box.set_motors_config(new_config)

        for coord, values in new_config.items():
            motor = box.get_motor(coord)
            self.assertEqual(values, {**motor.config, 'name': motor.name})

    def test_read_input_config_from_file(self):
        emulator = MCC2BoxEmulator(n_bus=4, n_axes=2)

        controllers_to_init = [0, 1, 2, 3]
        motors_to_init = [(0, 1), (0, 2), (1, 1), (1, 2), (2, 1), (2, 2), (3, 1), (3, 2)]
        motors_config = {(0, 1): {'name': 'TestMotor0',
                                  'with_initiators': 0,
                                  'with_encoder': 1,
                                  'display_units': '1',
                                  'displ_per_contr': 1.0, },
                         (0, 2): {'name': 'TestMotor1',
                                  'with_initiators': 1,
                                  'with_encoder': 0,
                                  'display_units': 'Schritte',
                                  'displ_per_contr': 1.0},
                         (1, 1): {'name': 'TestMotor2',
                                  'with_initiators': 1,
                                  'with_encoder': 1,
                                  'display_units': '1',
                                  'displ_per_contr': 1.0},
                         (1, 2): {'name': 'TestMotor3',
                                  'with_initiators': 1,
                                  'with_encoder': 0,
                                  'display_units': '1',
                                  'displ_per_contr': 1.0},
                         (2, 1): {'name': 'TestMotor4',
                                  'with_encoder': 0,
                                  'with_initiators': 1,
                                  'display_units': '1',
                                  'displ_per_contr': 1.0},
                         (2, 2): {'name': 'TestMotor5',
                                  'with_initiators': 1,
                                  'with_encoder': 0,
                                  'display_units': '1',
                                  'displ_per_contr': 1.0},
                         (3, 1): {'name': 'TestMotor6',
                                  'with_initiators': 1,
                                  'with_encoder': 0,
                                  'display_units': '1',
                                  'displ_per_contr': 1.0},
                         (3, 2): {'name': 'TestMotor7',
                                  'with_initiators': 1,
                                  'with_encoder': 0,
                                  'display_units': '1',
                                  'displ_per_contr': 1.0}}
        motors_parameters = {
            (0, 1): {'Lauffrequenz': 1.0, 'Stoppstrom': 1.0, 'Laufstrom': 1.0, 'Booststrom': 1.0, 'Initiatortyp': 1.0},
            (0, 2): {'Lauffrequenz': 1.0, 'Stoppstrom': 1.0, 'Laufstrom': 1.0, 'Booststrom': 1.0, 'Initiatortyp': 1.0},
            (1, 1): {'Lauffrequenz': 1.0, 'Stoppstrom': 1.0, 'Laufstrom': 1.0, 'Booststrom': 1.0, 'Initiatortyp': 1.0},
            (1, 2): {'Lauffrequenz': 400, 'Stoppstrom': 1.0, 'Laufstrom': 1.0, 'Booststrom': 1.0, 'Initiatortyp': 1.0},
            (2, 1): {'Lauffrequenz': 1.0, 'Stoppstrom': 2, 'Laufstrom': 1.0, 'Booststrom': 1.0, 'Initiatortyp': 1.0},
            (2, 2): {'Lauffrequenz': 1.0, 'Stoppstrom': 1.0, 'Laufstrom': 2, 'Booststrom': 1.0, 'Initiatortyp': 1.0},
            (3, 1): {'Lauffrequenz': 1.0, 'Stoppstrom': 1.0, 'Laufstrom': 1.0, 'Booststrom': 2, 'Initiatortyp': 1.0},
            (3, 2): {'Lauffrequenz': 1.0, 'Stoppstrom': 1.0, 'Laufstrom': 1.0, 'Booststrom': 1.0, 'Initiatortyp': 0.0}}

        controllers_to_init_f, motors_to_init_f, motors_config_f, motors_parameters_f = \
            read_input_config_from_file(emulator, 'test_input/test_input_file2.csv')

        self.assertEqual(controllers_to_init, controllers_to_init_f)
        self.assertEqual(motors_to_init, motors_to_init_f)
        self.assertEqual(motors_config, motors_config_f)
        self.assertEqual(motors_parameters, motors_parameters_f)

    def test_initialize_with_input_file(self):
        emulator = MCC2BoxEmulator(n_bus=13, n_axes=7)
        box = Box(emulator, input_file='test_input/test_input_file.csv')

        motors_list = [(0, 1), (12, 4), (3, 6), (12, 3)]
        config_from_file = {0: {'name': 'TestMotor0',
                                'with_initiators': 1,
                                'with_encoder': 0,
                                'display_units': 'einh1',
                                'norm_per_contr': 1.0,
                                'displ_per_contr': 22.222,
                                'displ_null': 500.0,
                                'null_position': 0.0},
                            1: {'name': 'TestMotor1',
                                'with_initiators': 0,
                                'with_encoder': 0,
                                'display_units': 'einh2',
                                'norm_per_contr': 1.0,
                                'displ_per_contr': 33.333,
                                'displ_null': 500.0,
                                'null_position': 0.0},
                            2: {'name': 'TestMotor2',
                                'with_initiators': 1,
                                'with_encoder': 1,
                                'display_units': 'einh3',
                                'norm_per_contr': 1.0,
                                'displ_per_contr': 44.444,
                                'displ_null': 500.0,
                                'null_position': 0.0},
                            3: {'name': 'TestMotor3',
                                'with_initiators': 1,
                                'with_encoder': 1,
                                'display_units': 'einh4',
                                'norm_per_contr': 1.0,
                                'displ_per_contr': 55.555,
                                'displ_null': 500.0,
                                'null_position': 0.0}}
        parameters = {
            (0, 1): {'Lauffrequenz': 1000, 'Stoppstrom': 1, 'Laufstrom': 2, 'Booststrom': 3, 'Initiatortyp': 1},
            (12, 4): {'Lauffrequenz': 2000, 'Stoppstrom': 2, 'Laufstrom': 3, 'Booststrom': 4, 'Initiatortyp': 1},
            (3, 6): {'Lauffrequenz': 3000, 'Stoppstrom': 3, 'Laufstrom': 4, 'Booststrom': 5, 'Initiatortyp': 0},
            (12, 3): {'Lauffrequenz': 4000, 'Stoppstrom': 4, 'Laufstrom': 5, 'Booststrom': 6, 'Initiatortyp': 1}}

        right_report = f"{4} Controller und {4} Motoren wurde initialisiert:\n"
        right_report += f"Controller {14} ist nicht verbunden und wurde nicht initialisiert.\n"
        right_report += f"Achse {8} ist beim Controller {5} nicht vorhanden, " \
                        f"der Motor wurde nicht initialisiert.\n"
        right_report += f'Controller {0}: TestMotor0\n'
        right_report += f'Controller {12}: TestMotor1, TestMotor3\n'
        right_report += f'Controller {3}: TestMotor2\n'
        right_report += f'Controller {5}: \n'

        self.assertEqual(right_report, box.report)
        self.assertEqual(4, len(box.controller))

        self.assertEqual(1, len(box.controller[0].motor))
        self.assertEqual(2, len(box.controller[12].motor))
        self.assertEqual(1, len(box.controller[3].motor))
        self.assertEqual(0, len(box.controller[5].motor))

        for coord, param_line in parameters.items():
            param_line_from_box = box.get_motor(coord).get_parameters()
            for name, value in param_line.items():
                self.assertEqual(value, param_line_from_box[name])

        for i, coord in enumerate(motors_list):
            motor = box.get_motor(coord)
            self.assertEqual(config_from_file[i], {**motor.config, 'name': motor.name})
            # self.assertEqual(parameters[i], motor.get_parameters())

    def test_all_motors_stand(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=9)
        box = Box(emulator)

        self.assertTrue(box.all_motors_stand())

        emulator.controller[2].motor[3]._stand = False
        emulator.controller[9].motor[1]._stand = False
        self.assertFalse(box.all_motors_stand())

        emulator.controller[2].motor[3]._stand = True
        self.assertFalse(box.all_motors_stand())

        emulator.controller[9].motor[1]._stand = True
        self.assertTrue(box.all_motors_stand())

    def test_calibrate_motors(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=True)
        box = Box(emulator, input_file='test_input/test_input_calibr.csv')
        emulator.controller[0].motor[1].beginning = -500
        emulator.controller[0].motor[1].end = 500

        calibr_thread = Thread(target=box.calibrate_motors)
        calibr_thread.start()
        emulator.controller[0].motor[1].sleep_steps(8)
        self.assertFalse(emulator.controller[0].motor[1].stand())
        self.assertTrue(emulator.controller[0].motor[2].stand())
        self.assertFalse(emulator.controller[1].motor[1].stand())
        self.assertFalse(emulator.controller[1].motor[2].stand())
        emulator.realtime = False
        calibr_thread.join()

        self.assertEqual(-500, box.get_motor((0, 1)).config['null_position'])
        self.assertEqual(0, box.get_motor((0, 2)).config['null_position'])
        self.assertEqual(-10000, box.get_motor((1, 1)).config['null_position'])
        self.assertEqual(-10000, box.get_motor((1, 2)).config['null_position'])

        self.assertEqual(1, round(box.get_motor((0, 1)).config['norm_per_contr'], 4))
        self.assertEqual(1, round(box.get_motor((0, 2)).config['norm_per_contr'], 4))
        self.assertEqual(0.05, round(box.get_motor((1, 1)).config['norm_per_contr'], 4))
        self.assertEqual(0.05, round(box.get_motor((1, 2)).config['norm_per_contr'], 4))

    def test_read_saved_session_data_from_file(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)

        data = {(0, 1): (153.0, 0.05, (None, None)),
                (0, 2): (253.0, 1.0, (100, None)),
                (1, 1): (353.0, 1.0, (None, 200)),
                (1, 2): (463.5, 0.05, (50.5, 200.56))}

        self.assertEqual(data, read_saved_session_data_from_file('test_data/test_saved_motors_data.txt'))

    def test_read_saved_session_data(self):
        emulator = MCC2BoxEmulator(n_bus=3, n_axes=3)
        box = Box(emulator)

        data = {(0, 1): (153.0, 0.05, (None, None)),
                (0, 2): (253.0, 1.0, (100, None)),
                (1, 1): (353.0, 1.0, (None, 200)),
                (1, 2): (463.5, 0.05, (50.5, 200.56))}

        list_to_calibration = [(0, 3), (1, 3), (2, 1), (2, 2), (2, 3)]
        self.assertEqual(list_to_calibration, box.read_saved_session_data('test_data/test_saved_motors_data.txt'))

        for coord in data:
            motor = box.get_motor(coord)
            motor_data = (motor.position(), motor.config['norm_per_contr'], motor.soft_limits)
            self.assertEqual(data[coord], motor_data)

    def test_save_session_data(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        box = Box(emulator)
        if os.path.isfile('test_data/test_saved_motors_data2.txt'):
            os.remove('test_data/test_saved_motors_data2.txt')
        data = read_saved_session_data_from_file('test_data/test_saved_motors_data.txt')

        box.read_saved_session_data('test_data/test_saved_motors_data.txt')
        box.save_session_data('test_data/test_saved_motors_data2.txt')
        self.assertEqual(data, read_saved_session_data_from_file('test_data/test_saved_motors_data2.txt'))

        emulator = MCC2BoxEmulator(n_bus=1, n_axes=1)
        box = Box(emulator)
        box.read_saved_session_data('test_data/test_saved_motors_data.txt')
        box.save_session_data('test_data/test_saved_motors_data2.txt')
        self.assertEqual(data, read_saved_session_data_from_file('test_data/test_saved_motors_data2.txt'))

    def test_make_empty_input_file(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        box = Box(emulator)
        if os.path.isfile('test_input/test_input_data(Vorlage).csv'):
            os.remove('test_input/test_input_data(Vorlage).csv')

        box.make_empty_input_file('test_input/test_input_data(Vorlage).csv')

        header = ['Motor Name', 'Bus', 'Achse', 'Mit Initiatoren(0 oder 1)', 'Mit Encoder(0 oder 1)', 'Einheiten', 'Umrechnungsfaktor']
        for parameter_name in MCC2Communicator.PARAMETER_DEFAULT.keys():
            header.append(parameter_name)

        file = read_csv('test_input/test_input_data(Vorlage).csv')
        self.assertEqual(4, len(file))
        self.assertEqual(header, list(file[0].keys()))
        self.assertEqual(('Motor0.1', '0', '1'), (file[0]['Motor Name'], file[0]['Bus'], file[0]['Achse']))
        self.assertEqual(('Motor0.2', '0', '2'), (file[1]['Motor Name'], file[1]['Bus'], file[1]['Achse']))
        self.assertEqual(('Motor1.1', '1', '1'), (file[2]['Motor Name'], file[2]['Bus'], file[2]['Achse']))
        self.assertEqual(('Motor1.2', '1', '2'), (file[3]['Motor Name'], file[3]['Bus'], file[3]['Achse']))

    def test_motors_names_list(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=3)
        box = Box(emulator)

        names_list = {'Motor0.1', 'Motor0.2', 'Motor0.3', 'Motor1.1', 'Motor1.2', 'Motor1.3'}
        self.assertEqual(names_list, set(box.motors_names_list()))

    def test_controllers_list(self):
        emulator = MCC2BoxEmulator(n_bus=14, n_axes=3)
        box = Box(emulator)

        controller_list = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13}
        self.assertEqual(controller_list, set(box.controllers_list()))

    def test_motors_without_initiators(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=True)
        box = Box(emulator, input_file='test_input/test_input_file3.csv')

        motors_without_initiators = [(0, 2), (1, 1)]
        self.assertEqual(set(box.get_motors(motors_without_initiators)), set(box.motors_without_initiators()))

    def test_motors_with_initiators(self):
        emulator = MCC2BoxEmulator(n_bus=3, n_axes=2, realtime=True)
        box = Box(emulator, input_file='test_input/test_input_file3.csv')

        motors_with_initiators = [(0, 1), (1, 2), (2, 1), (2, 2)]
        self.assertEqual(set(box.get_motors(motors_with_initiators)), set(box.motors_with_initiators()))


class TestBoxCluster(TestCase):
    def test__check_motors_names(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)

        with self.assertRaises(MotorNamesError):
            BoxesCluster({"box1": box1, "box2": box2})

        box2.get_motor((0, 1)).name = "Axe01"
        box2.get_motor((0, 2)).name = "Axe02"
        box2.get_motor((1, 1)).name = "Axe11"
        box2.get_motor((1, 2)).name = "Axe12"
        cluster = BoxesCluster({"box1": box1, "box2": box2})
        self.assertEqual(8, len(cluster.motors))

        box2.get_motor((0, 2)).name = "Axe11"
        with self.assertRaises(MotorNamesError) as err:
            BoxesCluster({"box1": box1, "box2": box2})
        err_mess = 'Es gibt die wiederholte Namen der Motoren! Der Name "Axe11" ist mehrmals getroffen.'
        self.assertEqual(err.exception.args[0], err_mess)

    def test___init__with_prefix(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=16, n_axes=3, realtime=False)
        box_emulator3 = MCC2BoxEmulator(n_bus=5, n_axes=2, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)
        box3 = Box(box_emulator3)

        cluster = BoxesCluster({"box1": box1, "box2": box2, "box3": box3}, add_box_prefix=True)
        self.assertEqual(2 * 2 + 16 * 3 + 5 * 2, len(cluster.motors))
        self.assertEqual("box2|Motor3.3", box2.controller[3].motor[3].name)

        for box_name, box in cluster.boxes.items():
            for controller in box:
                for motor in controller:
                    self.assertEqual(f"{box_name}|Motor{controller.bus}.{motor.axis}", motor.name)

    def test_go_to(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=16, n_axes=3, realtime=False)
        box_emulator3 = MCC2BoxEmulator(n_bus=5, n_axes=2, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)
        box3 = Box(box_emulator3)

        cluster = BoxesCluster({"box1": box1, "box2": box2, "box3": box3}, add_box_prefix=True)

        dest = {'box1|Motor1.1': 3000, 'box2|Motor9.1': 300, 'box2|Motor15.3': 10, 'box3|Motor4.2': 10000}

        self.assertEqual((True, ''), cluster.go_to(dest, 'contr'))
        box1.wait_all_motors_stop()
        box2.wait_all_motors_stop()
        box3.wait_all_motors_stop()
        self.assertAlmostEqual(3000, cluster.motors['box1|Motor1.1'].position('contr'))
        self.assertAlmostEqual(300, cluster.motors['box2|Motor9.1'].position('contr'))
        self.assertAlmostEqual(10, cluster.motors['box2|Motor15.3'].position('contr'))
        self.assertAlmostEqual(10000, cluster.motors['box3|Motor4.2'].position('contr'))

        dest = {'box1|Motor1.1': 3000, 'box2|Motor9.1': 300, 'box2|Motor155.3': 10, 'box3|Motor4.2': 10000}
        with self.assertRaises(ValueError) as err:
            cluster.go_to(dest, 'contr')
        err_mess = "Es gibt keine Motoren mit den Namen: {'box2|Motor155.3'}"
        self.assertEqual(err_mess, err.exception.args[0])

    def test_go_to_wait(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=True)
        box_emulator2 = MCC2BoxEmulator(n_bus=16, n_axes=3, realtime=True)
        box_emulator3 = MCC2BoxEmulator(n_bus=5, n_axes=2, realtime=True)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)
        box3 = Box(box_emulator3)
        freq = 10
        box_emulator1.set_parameter('Lauffrequenz', freq, 1, 1)
        box_emulator2.set_parameter('Lauffrequenz', freq, 9, 1)
        box_emulator2.set_parameter('Lauffrequenz', freq, 15, 3)
        box_emulator3.set_parameter('Lauffrequenz', freq, 4, 2)

        cluster = BoxesCluster({"box1": box1, "box2": box2, "box3": box3}, add_box_prefix=True)

        dest = {'box1|Motor1.1': 30, 'box2|Motor9.1': 10, 'box2|Motor15.3': 1, 'box3|Motor4.2': 45}

        reporter = WaitTestReporter()
        self.assertEqual((True, ''), cluster.go_to(dest, 'contr', True, True, reporter=reporter))
        self.assertAlmostEqual(30, cluster.motors['box1|Motor1.1'].position('contr'))
        self.assertAlmostEqual(10, cluster.motors['box2|Motor9.1'].position('contr'))
        self.assertAlmostEqual(1, cluster.motors['box2|Motor15.3'].position('contr'))
        self.assertAlmostEqual(45, cluster.motors['box3|Motor4.2'].position('contr'))
        self.assertEqual(['box2|Motor15.3', 'box2|Motor9.1', 'box1|Motor1.1', 'box3|Motor4.2'], reporter.motors_done)

    def test_go_to_check(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=16, n_axes=3, realtime=False)
        box_emulator3 = MCC2BoxEmulator(n_bus=5, n_axes=2, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)
        box3 = Box(box_emulator3)

        cluster = BoxesCluster({"box1": box1, "box2": box2, "box3": box3}, add_box_prefix=True)

        dest = {'box1|Motor1.1': 3000, 'box2|Motor9.1': 300, 'box2|Motor15.3': 10, 'box3|Motor4.2': 10000}
        box_emulator3.controller[4].motor[2].end = 9800
        cluster.motors['box2|Motor9.1'].soft_limits = (-10, 20)
        self.assertEqual((False, 'Der Zielpunkt des Motors "box2|Motor9.1" liegt außerhalb der Soft Limits!\n'
                                 'Der Zielpunkt des Motors "box3|Motor4.2" wurde nicht erreicht.\n'),
                         cluster.go_to(dest, 'contr', True, True))

    def test_positions(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=2, n_axes=1, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)

        cluster = BoxesCluster({"box1": box1, "box2": box2}, add_box_prefix=True)

        positions = {'box1|Motor0.1': 100, 'box1|Motor0.2': 50, 'box1|Motor1.1': -60, 'box1|Motor1.2': 1000,
                     'box2|Motor0.1': -745, 'box2|Motor1.1': 356}
        cluster.go_to(positions, 'contr', wait=True)
        self.assertEqual(positions, cluster.positions('contr'))

        positions2 = {'box1|Motor0.1': 100, 'box1|Motor0.2': 50, 'box1|Motor1.2': 1000, 'box2|Motor0.1': -745}
        self.assertEqual(positions2, cluster.positions('contr', list(positions2.keys())))

    def test_path_travel(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=2, n_axes=1, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)

        cluster = BoxesCluster({"box1": box1, "box2": box2}, add_box_prefix=True)
        reporter = TrevelTestReporter()
        stoper = StopTestIndicator()

        path = [{'box1|Motor0.1': 100, 'box1|Motor0.2': 50, 'box1|Motor1.1': -60, 'box1|Motor1.2': 1000,
                     'box2|Motor0.1': -745, 'box2|Motor1.1': 356},

                {'box1|Motor0.1': 200, 'box1|Motor0.2': -550, 'box1|Motor1.1': 70, 'box1|Motor1.2': 500,
                 'box2|Motor0.1': -400, 'box2|Motor1.1': -5},

                {'box1|Motor0.1': 10, 'box1|Motor0.2': -50, 'box1|Motor1.1': 80, 'box1|Motor1.2': -250,
                 'box2|Motor0.1': 30, 'box2|Motor1.1': -467}]

        def action():
            return cluster.positions('contr')

        self.assertEqual(path, cluster.path_travel(path, action, 'contr', reporter=reporter))

        # Prüfung von reporter
        self.assertEqual((6 + 1)*3, len(reporter.history))
        len_history = list(map(len, reporter.history))
        right_len_history = [6, 5, 4, 3, 2, 1, 0]*3
        self.assertEqual(right_len_history, len_history)

        def action2():
            self.index += 3
            return self.index

        self.index = 0
        self.assertEqual([3, 6, 9], cluster.path_travel(path, action2, 'contr'))

        # Stop prüfen
        box_emulator1.realtime = True
        box_emulator2.realtime = True
        with concurrent.futures.ThreadPoolExecutor() as executor:
            res = executor.submit(cluster.path_travel, path, action, 'contr', stoper)
            stoper.stop = True
            self.assertEqual([], res.result())

    def test_read_path_from_file(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=2, n_axes=1, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)
        cluster = BoxesCluster({"box1": box1, "box2": box2}, add_box_prefix=True)

        path = [{'box1|Motor0.1': 100, 'box1|Motor0.2': 50.1, 'box1|Motor1.1': -60, 'box1|Motor1.2': 1000,
                 'box2|Motor0.1': -745, 'box2|Motor1.1': 356},

                {'box1|Motor0.1': 200, 'box1|Motor0.2': -550, 'box1|Motor1.1': 70, 'box1|Motor1.2': 500,
                 'box2|Motor0.1': -400, 'box2|Motor1.1': -5},

                {'box1|Motor0.1': 10, 'box1|Motor0.2': -50, 'box1|Motor1.1': 80.1, 'box1|Motor1.2': -250,
                 'box2|Motor0.1': 30, 'box2|Motor1.1': -467}]

        self.assertEqual(path, cluster.read_path_from_file('test_input/test_path.csv', decimal=',', check=True))

        # with wrong motor name
        with self.assertRaises(FileReadError) as err:
            cluster.read_path_from_file('test_input/test_positions_wrong.csv', check=True)
        err_mess = "Es gibt keine Motoren mit den Namen: {'box111|Motor1.1'}"
        self.assertEqual(err_mess, err.exception.args[0])

    def test_read_positions_from_file(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=2, n_axes=1, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)
        cluster = BoxesCluster({"box1": box1, "box2": box2}, add_box_prefix=True)

        positions = {'box1|Motor0.1': 100, 'box1|Motor0.2': 50.1, 'box1|Motor1.1': -60, 'box1|Motor1.2': 1000,
                     'box2|Motor0.1': -745, 'box2|Motor1.1': 356}

        self.assertEqual(positions, cluster.read_positions_from_file('test_input/test_positions.csv', decimal=','))

        # mit mehrere Zeilen
        with self.assertRaises(FileReadError) as err:
            self.assertEqual(positions, cluster.read_positions_from_file('test_input/test_path.csv', decimal=','))
        err_mess = 'Es ist mehr als eine Zeile in Datei, nämlich 3 statt eine.'
        self.assertEqual(err_mess, err.exception.args[0])

    def test_save_current_positions_in_file(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=2, n_axes=1, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)
        cluster = BoxesCluster({"box1": box1, "box2": box2}, add_box_prefix=True)

        positions = {'box1|Motor0.1': 100, 'box1|Motor0.2': 50.1, 'box1|Motor1.1': -60, 'box1|Motor1.2': 1000,
                     'box2|Motor0.1': -745, 'box2|Motor1.1': 356}
        cluster.go_to(positions, 'contr', wait=True)
        cluster.motors['box1|Motor0.2'].set_position(50.1)

        cluster.save_current_positions_in_file('test_input/test_positions_saved.csv', 'contr')
        self.assertEqual(positions, cluster.read_positions_from_file('test_input/test_positions_saved.csv'))

    def test_path_travel_from_file(self):
        box_emulator1 = MCC2BoxEmulator(n_bus=2, n_axes=2, realtime=False)
        box_emulator2 = MCC2BoxEmulator(n_bus=2, n_axes=1, realtime=False)
        box1 = Box(box_emulator1)
        box2 = Box(box_emulator2)

        cluster = BoxesCluster({"box1": box1, "box2": box2}, add_box_prefix=True)

        path = [{'box1|Motor0.1': 100, 'box1|Motor0.2': 50, 'box1|Motor1.1': -60, 'box1|Motor1.2': 1000,
                     'box2|Motor0.1': -745, 'box2|Motor1.1': 356},

                {'box1|Motor0.1': 200, 'box1|Motor0.2': -550, 'box1|Motor1.1': 70, 'box1|Motor1.2': 500,
                 'box2|Motor0.1': -400, 'box2|Motor1.1': -5},

                {'box1|Motor0.1': 10, 'box1|Motor0.2': -50, 'box1|Motor1.1': 80, 'box1|Motor1.2': -250,
                 'box2|Motor0.1': 30, 'box2|Motor1.1': -467}]

        def action():
            return cluster.positions('contr')

        self.assertEqual(path, cluster.path_travel_from_file('test_input/test_path.csv', action, 'contr', decimal=','))


if __name__ == '__main__':
    main()
