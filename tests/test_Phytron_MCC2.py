from unittest import TestCase, main
import timeout_decorator

from motor_controller.Phytron_MCC2 import *


class Test__MCC2MotorEmulator(TestCase):

    @timeout_decorator.timeout(0.1)
    def test_move_to_fast(self):
        box = MCC2BoxEmulator()
        motor = MCC2MotorEmulator(box)
        init_position = motor.get_position()
        if init_position == 5000:
            motor.go_to(-5000)
            motor.wait_stop()
            self.assertEqual(motor.get_position(), -5000)
        else:
            motor.go_to(5000)
            motor.wait_stop()
            self.assertEqual(motor.get_position(), 5000)

    def test_set_position(self):
        box = MCC2BoxEmulator()
        motor = MCC2MotorEmulator(box)
        motor.set_position(0)
        self.assertEqual(motor.get_position(), 0)
        motor.set_position(500)
        self.assertEqual(motor.get_position(), 500)

    def test_move_to_thread(self):
        box = MCC2BoxEmulator(realtime=True)
        motor = MCC2MotorEmulator(box)
        motor.set_position(0)
        motor.go_to(400)
        motor.sleep_steps(2)
        self.assertTrue(0 < motor.get_position() < 350)
        motor.stop()

    def test_stop(self):
        box = MCC2BoxEmulator(realtime=True)
        motor = MCC2MotorEmulator(box)
        motor.set_position(0)
        motor.go_to(400)
        motor.sleep_steps(2)
        position_before_stop = motor.get_position()
        motor.sleep_steps(2)
        motor.stop()
        stop_position = motor.get_position()
        motor.sleep_steps(2)

        self.assertTrue(position_before_stop > 0)
        self.assertTrue(position_before_stop < stop_position)
        self.assertTrue(stop_position == motor.get_position())

    def test_stand(self):
        box = MCC2BoxEmulator(realtime=True)
        motor = MCC2MotorEmulator(box)
        motor.go_to(400)
        motor.sleep_one_step()
        self.assertFalse(motor.stand())
        motor.stop()
        motor.sleep_steps(2)
        self.assertTrue(motor.stand())

    @timeout_decorator.timeout(0.2)
    def test_initiators(self):
        box = MCC2BoxEmulator(realtime=False)
        motor = MCC2MotorEmulator(box)
        motor.go_to(30000)
        motor.wait_stop()
        self.assertEqual(motor.get_position(), 10000)
        motor.go_to(-30000)
        motor.wait_stop()
        self.assertEqual(motor.get_position(), -10000)


class TestMCC2BoxEmulator(TestCase):
    def test_read_until(self):
        emulator_box = MCC2BoxEmulator(n_bus=3, n_axes=2, realtime=False)
        emulator_box.buffer = b'\x02command1\x03\x02command2\x03\x02command3\x03'

        self.assertEqual(b'\x02command1\x03', emulator_box.read_until(b'\x03'))
        self.assertEqual(b'\x02command2\x03\x02command3\x03', emulator_box.buffer)

        self.assertEqual(b'\x02command2\x03', emulator_box.read_until(b'\x03'))
        self.assertEqual(b'\x02command3\x03', emulator_box.buffer,)

        self.assertEqual(emulator_box.read_until(b'\x03'), b'\x02command3\x03', emulator_box.read_until(b'\x03'))
        self.assertEqual(b'', emulator_box.buffer)

        self.assertEqual(b'', emulator_box.read_until(b'\x03'))
        self.assertEqual(b'', emulator_box.buffer)

    def test_contr_version(self):
        emulator_box = MCC2BoxEmulator(n_bus=13, n_axes=2, realtime=False)
        emulator_box.flushInput()

        vers = b'\x02\x06MCC2 Emulator v1.0\x03'

        for i in range(13):
            bus_index = f'{i:x}'.encode()
            emulator_box.write(b'\x02' + bus_index + b'IVR\x03')
            self.assertEqual(vers, emulator_box.read_until(b'\x03'), f'Fehler beim Modul {i}')

        emulator_box.write(b'\x02DIVR\x03')
        self.assertEqual(b'', emulator_box.read_until(b'\x03'))

    def test_axes_numder(self):
        emulator_box = MCC2BoxEmulator(n_bus=13, n_axes=4, realtime=False)
        emulator_box.flushInput()

        for i in range(13):
            bus_index = f'{i:x}'.encode()
            emulator_box.write(b'\x02' + bus_index + b'IAR\x03')
            self.assertEqual(b'\x02\x064\x03', emulator_box.read_until(b'\x03'), f'Fehler beim Modul {i}')

    def test_get_set_parameter(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=2, realtime=False)
        emulator_box.flushInput()
        for bus in range(16):
            for axis in (1, 2):
                for param_n in MCC2Communicator.PARAMETER_NUMBER.values():
                    # set 5.1
                    emulator_box.write(f'\x02{bus:x}{axis}P{param_n}S{5.1}\x03'.encode())
                    self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'))
                    # read 5.1
                    emulator_box.write(f'\x02{bus:x}{axis}P{param_n}R\x03'.encode())
                    self.assertEqual(b'\x02\x065.1\x03', emulator_box.read_until(b'\x03'))

                    # set 2560.89
                    emulator_box.write(f'\x02{bus:x}{axis}P{param_n}S{2560.89}\x03'.encode())
                    self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'))
                    # read 2560.89
                    emulator_box.write(f'\x02{bus:x}{axis}P{param_n}R\x03'.encode())
                    self.assertEqual(b'\x02\x062560.89\x03', emulator_box.read_until(b'\x03'))

        bus, axis = 0, 1
        # Unkorrekte Eingang1
        emulator_box.write(f'\x02{bus:x}{axis}P{3}Sf324sf\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))
        # Unkorrekte Eingang2
        emulator_box.write(f'\x02{bus:x}{axis}Pf3d4sf2d\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))
        # Unkorrekte Parameternummer get
        emulator_box.write(f'\x02{bus:x}{axis}P104\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))
        # Unkorrekte Parameternummer set
        emulator_box.write(f'\x02{bus:x}{axis}P103S20\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))

    def test_get_set_position(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=2, realtime=False)
        for bus in range(16):
            for axis in (1, 2):
                # Umrechnungsfactor gleich 1 einstellen
                emulator_box.write(f'\x02{bus:x}{axis}P{3}S{1}\x03'.encode())
                emulator_box.flushInput()

                # set -5.1
                emulator_box.write(f'\x02{bus:x}{axis}P{20}S{-5.1}\x03'.encode())
                self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'))
                # read -5.1
                emulator_box.write(f'\x02{bus:x}{axis}P{20}R\x03'.encode())
                self.assertEqual(b'\x02\x06-5.1\x03', emulator_box.read_until(b'\x03'))

                # set 600.52
                emulator_box.write(f'\x02{bus:x}{axis}P{20}S{600.52}\x03'.encode())
                self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'))
                # read 600.52
                emulator_box.write(f'\x02{bus:x}{axis}P{20}R\x03'.encode())
                self.assertEqual(b'\x02\x06600.52\x03', emulator_box.read_until(b'\x03'))

                # Umrechnungsfaktor gleich 0.5 einstellen
                emulator_box.write(f'\x02{bus:x}{axis}P{3}S{0.5}\x03'.encode())
                emulator_box.flushInput()
                emulator_box.write(f'\x02{bus:x}{axis}P{20}R\x03'.encode())
                self.assertEqual(b'\x02\x06300.26\x03', emulator_box.read_until(b'\x03'))

    @timeout_decorator.timeout(1)
    def test_go_to(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=2, realtime=False)

        for bus in range(16):
            for axis in (1, 2):
                motor = emulator_box.controller[bus].motor[axis]
                motor.set_parameter(3, 1)
                # go to 200
                emulator_box.write(f'\x02{bus:x}{axis}A{200}\x03'.encode())
                self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'), f'Fehler beim Motor ({bus},{axis})')
                motor.wait_stop()
                self.assertEqual(200, motor.get_position(), f'Fehler beim Motor ({bus},{axis})')

                # go to -5000
                emulator_box.write(f'\x02{bus:x}{axis}A{-5000}\x03'.encode())
                self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'), f'Fehler beim Motor ({bus},{axis})')
                motor.wait_stop()
                self.assertEqual(-5000, motor.get_position()), f'Fehler beim Motor ({bus},{axis})'

        # Unkorrekte Eingang1
        emulator_box.write(f'\x02{0}{1}A324fg\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))
        # Unkorrekte Eingang2
        emulator_box.write(f'\x02{0}{1}A\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))

    @timeout_decorator.timeout(1)
    def test_go(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=2, realtime=False)

        for bus in range(16):
            for axis in (1, 2):
                motor = emulator_box.controller[bus].motor[axis]
                motor.set_parameter(3, 1)
                motor.set_position(0)

                # go 300
                emulator_box.write(f'\x02{bus:x}{axis}{300}\x03'.encode())
                self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'), f'Fehler beim Motor ({bus},{axis})')
                motor.wait_stop()
                self.assertEqual(300, motor.get_position(), f'Fehler beim Motor ({bus},{axis})')

                # go -4300
                emulator_box.write(f'\x02{bus:x}{axis}{-4300}\x03'.encode())
                self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'), f'Fehler beim Motor ({bus},{axis})')
                motor.wait_stop()
                self.assertEqual(-4000, motor.get_position(), f'Fehler beim Motor ({bus},{axis})')

        # Unkorrekte Eingang1
        emulator_box.write(f'\x02{0}{1}324fg\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))
        # Unkorrekte Eingang2
        emulator_box.write(f'\x02{0}{1}\x03'.encode())
        self.assertEqual(b'\x02\x15\x03', emulator_box.read_until(b'\x03'))

    def test_stop(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=2, realtime=True)

        for bus in range(16):
            for axis in (1, 2):
                motor = emulator_box.controller[bus].motor[axis]
                motor.set_parameter(3, 1)
                motor.set_position(0)
                motor.go_to(30000)
                self.assertFalse(motor.stand())
                emulator_box.write(f'\x02{bus:x}{axis}S\x03'.encode())
                self.assertEqual(b'\x02\x06\x03', emulator_box.read_until(b'\x03'), f'Fehler beim Motor ({bus},{axis})')
                motor.sleep_steps(2)
                self.assertTrue(motor.stand())

    def test_stand(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=2, realtime=True)

        for bus in range(16):
            for axis in (1, 2):
                motor = emulator_box.controller[bus].motor[axis]
                motor.set_parameter(3, 1)
                motor.set_position(0)

                emulator_box.write(f'\x02{bus:x}{axis}=H\x03'.encode())
                self.assertEqual(b'\x02\x06E\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')

                motor.go_to(30000)
                emulator_box.write(f'\x02{bus:x}{axis}=H\x03'.encode())
                self.assertEqual(b'\x02\x06N\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')

                motor.stop()
                motor.sleep_steps(2)
                emulator_box.write(f'\x02{bus:x}{axis}=H\x03'.encode())
                self.assertEqual(b'\x02\x06E\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')

    @timeout_decorator.timeout(0.2)
    def test_initiators(self):
        emulator_box = MCC2BoxEmulator(n_bus=16, n_axes=2, realtime=False)
        emulator_box.flushInput()

        for bus in range(16):
            for axis in (1, 2):
                motor = emulator_box.controller[bus].motor[axis]
                motor.set_parameter(3, 1)
                motor.set_position(0)

                emulator_box.write(f'\x02{bus:x}{axis}=I-\x03'.encode())
                self.assertEqual(b'\x02\x06N\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')
                emulator_box.write(f'\x02{bus:x}{axis}=I+\x03'.encode())
                self.assertEqual(b'\x02\x06N\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')

                motor.set_position(14998)
                motor.go_to(30000)
                motor.wait_stop()

                emulator_box.write(f'\x02{bus:x}{axis}=I-\x03'.encode())
                self.assertEqual(b'\x02\x06N\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')
                emulator_box.write(f'\x02{bus:x}{axis}=I+\x03'.encode())
                self.assertEqual(b'\x02\x06E\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')

                motor.set_position(-14998)
                motor.go_to(-30000)
                motor.wait_stop()

                emulator_box.write(f'\x02{bus:x}{axis}=I-\x03'.encode())
                self.assertEqual(b'\x02\x06E\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')
                emulator_box.write(f'\x02{bus:x}{axis}=I+\x03'.encode())
                self.assertEqual(b'\x02\x06N\x03', emulator_box.read_until(b'\x03'),
                                 f'Fehler beim Motor ({bus},{axis})')


class TestMCC2Communicator(TestCase):
    def test_read_reply(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        emulator.buffer = b'\x02\x06Antwort\x03'
        self.assertEqual((True, b'Antwort'), communicator.read_reply())

        emulator.buffer = b'\x02\x06\x03'
        self.assertEqual((True, b''), communicator.read_reply())

        emulator.buffer = b'\x02\x15\x03'
        self.assertEqual((False, None), communicator.read_reply())

        emulator.buffer = b''
        self.assertEqual((None, None), communicator.read_reply())

        with self.assertRaises(ReplyError):
            emulator.buffer = b'\x02\x15dfsd\x03'
            communicator.read_reply()

        with self.assertRaises(ReplyError):
            emulator.buffer = b'\x02dfsd\x03'
            communicator.read_reply()

        with self.assertRaises(ReplyError):
            emulator.buffer = b'sdfefef'
            communicator.read_reply()

    def test_command_to_box(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.command_to_box(b'Lelouch Vi Brittania commands you')
        self.assertEqual(b'\x02Lelouch Vi Brittania commands you\x03', emulator.last_command)

    def test_command_to_modul(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.command_to_modul(b'Obey Me', 2)
        self.assertEqual(b'\x022Obey Me\x03', emulator.last_command)

        communicator.command_to_modul(b'Obey Me', 13)
        self.assertEqual(b'\x02dObey Me\x03', emulator.last_command)

    def test_command_to_motor(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.command_to_motor(b'World', 2, 4)
        self.assertEqual(b'\x0224World\x03', emulator.last_command)

        communicator.command_to_motor(b'World', 13, 8)
        self.assertEqual(b'\x02d8World\x03', emulator.last_command)

    def test_command_without_reply(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.command_without_reply(b'S', 1, 2)
        self.assertEqual(b'\x0212S\x03', emulator.last_command)

        with self.assertRaises(ReplyError):
            communicator.command_without_reply(b'S', 5, 2)

        with self.assertRaises(ReplyError):
            communicator.command_without_reply(b'S', 5, 2)

        with self.assertRaises(ControllerError):
            communicator.command_without_reply(b'Sdfd', 1, 2)

    def test_command_with_bool_reply(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        self.assertTrue(communicator.command_with_bool_reply(b'=H', 1, 2))
        self.assertEqual(b'\x0212=H\x03', emulator.last_command)

        emulator.controller[1].motor[2]._stand = False
        self.assertFalse(communicator.command_with_bool_reply(b'=H', 1, 2))

        with self.assertRaises(NoReplyError):
            communicator.command_without_reply(b'=H', 5, 2)

        with self.assertRaises(ReplyError):
            communicator.command_without_reply(b'S', 5, 2)

        with self.assertRaises(ControllerError):
            communicator.command_without_reply(b'Sdfd', 1, 2)

    def test_command_with_float_reply(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        emulator.controller[1].motor[2].set_position(234.56)
        self.assertEqual(234.56, communicator.command_with_float_reply(b'P20R', 1, 2))
        self.assertEqual(b'\x0212P20R\x03', emulator.last_command)

        with self.assertRaises(NoReplyError):
            communicator.command_without_reply(b'=H', 5, 2)

        with self.assertRaises(ReplyError):
            communicator.command_without_reply(b'=H', 1, 2)

        with self.assertRaises(ReplyError):
            communicator.command_without_reply(b'S', 5, 2)

        with self.assertRaises(ControllerError):
            communicator.command_without_reply(b'Sdfd', 1, 2)

    def test_bus_check(self):
        emulator = MCC2BoxEmulator(n_bus=4, n_axes=2)
        del emulator.controller[1]
        del emulator.controller[3]
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        self.assertEqual((True, b'MCC2 Emulator v1.0'), communicator.bus_check(0))
        self.assertEqual(b'\x020IVR\x03', emulator.last_command)

        self.assertEqual((False, None), communicator.bus_check(1))
        self.assertEqual(b'\x021IVR\x03', emulator.last_command)

        self.assertEqual((True, b'MCC2 Emulator v1.0'), communicator.bus_check(2))
        self.assertEqual(b'\x022IVR\x03', emulator.last_command)

        self.assertEqual((False, None), communicator.bus_check(3))
        self.assertEqual(b'\x023IVR\x03', emulator.last_command)

    def test_check_connection(self):
        emulator = MCC2BoxEmulator(n_bus=0, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        self.assertEqual((False, None), communicator.check_connection())
        self.assertEqual(b'\x02fIVR\x03', emulator.last_command)

        emulator = MCC2BoxEmulator(n_bus=2, n_axes=2)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        self.assertEqual((True, b'MCC2 Emulator v1.0'), communicator.check_connection())
        self.assertEqual(b'\x020IVR\x03', emulator.last_command)

        emulator = MCC2BoxEmulator(n_bus=3, n_axes=2)
        del emulator.controller[0]
        del emulator.controller[1]
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        self.assertEqual((True, b'MCC2 Emulator v1.0'), communicator.check_connection())
        self.assertEqual(b'\x022IVR\x03', emulator.last_command)

    def test_bus_list(self):
        emulator = MCC2BoxEmulator(n_bus=15, n_axes=2)
        del emulator.controller[0]
        del emulator.controller[5]
        del emulator.controller[12]
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        self.assertEqual({1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 13, 14}, set(communicator.bus_list()))

    def test_axes_list(self):
        emulator = MCC2BoxEmulator(n_bus=2, n_axes=5)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        self.assertEqual({1, 2, 3, 4, 5}, set(communicator.axes_list(1)))

    def test_go(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.go(234.56, 1, 2)
        self.assertEqual(b'\x0212234.56\x03', emulator.last_command)

        communicator.go(4354.546, 13, 3)
        self.assertEqual(b'\x02d34354.546\x03', emulator.last_command)

    def test_go_to(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.go_to(234.56, 1, 2)
        self.assertEqual(b'\x0212A234.56\x03', emulator.last_command)

        communicator.go_to(4354.546, 13, 3)
        self.assertEqual(b'\x02d3A4354.546\x03', emulator.last_command)

    def test_stop(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.stop(1, 2)
        self.assertEqual(b'\x0212S\x03', emulator.last_command)

        communicator.stop(13, 3)
        self.assertEqual(b'\x02d3S\x03', emulator.last_command)

    def test_get_position(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        emulator.controller[1].motor[2].set_position(345.67)
        self.assertEqual(345.67, communicator.get_position(1, 2))
        self.assertEqual(b'\x0212P20R\x03', emulator.last_command)

        emulator.controller[14].motor[1].set_position(-3545.68)
        self.assertEqual(-3545.68, communicator.get_position(14, 1))
        self.assertEqual(b'\x02e1P20R\x03', emulator.last_command)

    def test_set_position(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.set_position(345.67, 1, 2)
        self.assertEqual(345.67, emulator.get_position(1, 2))
        self.assertEqual(b'\x0212P20S345.67\x03', emulator.last_command)

        communicator.set_position(-3545.68, 14, 1)
        self.assertEqual(-3545.68, emulator.get_position(14, 1))
        self.assertEqual(b'\x02e1P20S-3545.68\x03', emulator.last_command)

    def test_set_parameter(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        communicator.set_parameter('Lauffrequenz', 345.67, 1, 2)
        self.assertEqual(345.67, emulator.get_parameter('Lauffrequenz', 1, 2))
        self.assertEqual(b'\x0212P14S345.67\x03', emulator.last_command)

        communicator.set_parameter('Lauffrequenz', -3545.68, 14, 1)
        self.assertEqual(-3545.68, emulator.get_parameter('Lauffrequenz', 14, 1))
        self.assertEqual(b'\x02e1P14S-3545.68\x03', emulator.last_command)

        with self.assertRaises(ValueError):
            communicator.set_parameter('Frömmigkeit', -3545.68, 14, 1)

    def test_get_parameter(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        emulator.controller[1].motor[2].set_parameter(14, 345.67)
        self.assertEqual(345.67, communicator.get_parameter('Lauffrequenz', 1, 2))
        self.assertEqual(b'\x0212P14R\x03', emulator.last_command)

        emulator.controller[14].motor[1].set_parameter(14, -3545.68)
        self.assertEqual(-3545.68, communicator.get_parameter('Lauffrequenz', 14, 1))
        self.assertEqual(b'\x02e1P14R\x03', emulator.last_command)

        with self.assertRaises(ValueError):
            communicator.get_parameter('Frömmigkeit', 14, 1)

    def test_motor_stand(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        emulator.controller[0].motor[2]._stand = True
        self.assertEqual(True, communicator.motor_stand(0, 2))
        self.assertEqual(b'\x0202=H\x03', emulator.last_command)

        emulator.controller[0].motor[2]._stand = False
        self.assertEqual(False, communicator.motor_stand(0, 2))
        self.assertEqual(b'\x0202=H\x03', emulator.last_command)

        emulator.controller[14].motor[3]._stand = False
        self.assertEqual(False, communicator.motor_stand(14, 3))
        self.assertEqual(b'\x02e3=H\x03', emulator.last_command)

    def test_motor_at_the_beg(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        emulator.controller[0].motor[2]._beg_initiator = True
        self.assertEqual(True, communicator.motor_at_the_beg(0, 2))
        self.assertEqual(b'\x0202=I-\x03', emulator.last_command)

        emulator.controller[0].motor[2]._beg_initiator = False
        self.assertEqual(False, communicator.motor_at_the_beg(0, 2))
        self.assertEqual(b'\x0202=I-\x03', emulator.last_command)

        emulator.controller[14].motor[3]._beg_initiator = False
        self.assertEqual(False, communicator.motor_at_the_beg(14, 3))
        self.assertEqual(b'\x02e3=I-\x03', emulator.last_command)

    def test_motor_at_the_end(self):
        emulator = MCC2BoxEmulator(n_bus=16, n_axes=3)
        connector = SerialConnector(emulator=emulator)
        communicator = MCC2Communicator(connector)

        emulator.controller[0].motor[2]._end_initiator = True
        self.assertEqual(True, communicator.motor_at_the_end(0, 2))
        self.assertEqual(b'\x0202=I+\x03', emulator.last_command)

        emulator.controller[0].motor[2]._end_initiator = False
        self.assertEqual(False, communicator.motor_at_the_end(0, 2))
        self.assertEqual(b'\x0202=I+\x03', emulator.last_command)

        emulator.controller[14].motor[3]._end_initiator = False
        self.assertEqual(False, communicator.motor_at_the_end(14, 3))
        self.assertEqual(b'\x02e3=I+\x03', emulator.last_command)


if __name__ == '__main__':
    main()
