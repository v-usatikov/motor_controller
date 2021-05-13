# coding= utf-8
import logging
import time
from typing import List, Set

from PyQt5 import QtGui, QtCore
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QMainWindow, QApplication, QScrollBar, QStatusBar
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtCore import Qt, QThread, pyqtSignal
# from pyqt_led import Led
import sys
# import pylab
import serial, serial.tools.list_ports
# import pyqtgraph
from PyQt5.uic import loadUi

from motor_controller.interface import SerialConnector
from motor_controller.Phytron_MCC2 import Box, StopIndicator, WaitReporter, MCC2BoxSerial, MCC2BoxEmulator, \
    MCC2Communicator
import logscolor

if __name__ == '__main__':
    logscolor.init_config()


class AktPositionSlider(QScrollBar):
    def __init__(self, parent=None):
        super(AktPositionSlider, self).__init__(Qt.Horizontal, parent)
        self.U_x = 0
        self.O_x = 100
        self.setInvertedAppearance(True)

    def paintEvent(self, e):
        # super().paintEvent(e)
        qp = QPainter()
        qp.begin(self)
        qp.setRenderHint(QPainter.Antialiasing)
        self.drawLines(qp)
        qp.end()

    def drawLines(self, qp):
        pen = QPen(Qt.gray, 2, Qt.SolidLine)

        # Parametern
        d_icon = 8
        h_icon = 4
        rund_k = 1
        h_hint = 7
        height = self.height()
        hcenter = height / 2
        width = self.width()
        v_range = width - 2 * d_icon
        x_icon = d_icon + self.value() * v_range / 1000
        # x_icon = d_icon + self.U_x * v_range / 1000

        # Hintergrund malen
        pen.setWidth(1)
        pen.setColor(Qt.white)
        qp.setPen(pen)
        qp.setBrush(Qt.white)
        qp.drawRect(0, hcenter - h_hint, width, 2 * h_hint)

        # Icon malen
        pen.setWidth(0)
        pen.setColor(Qt.gray)
        qp.setPen(pen)
        qp.setBrush(Qt.gray)
        qp.drawRoundedRect(x_icon - d_icon, hcenter - h_icon, 2 * d_icon, 2 * h_icon, rund_k * h_icon, rund_k * h_icon)
        # print(x_icon+d_icon, width)

        # Soft Limits malen
        pen.setWidth(2)
        qp.setPen(pen)
        U_pixel_x = d_icon + self.U_x * v_range / 1000
        O_pixel_x = d_icon + self.O_x * v_range / 1000
        try:
            qp.drawLine(U_pixel_x, hcenter - h_hint, U_pixel_x, hcenter + h_hint)
            qp.drawLine(O_pixel_x, hcenter - h_hint, O_pixel_x, hcenter + h_hint)
        except OverflowError:
            logging.error("OverflowError, kann nicht Soft Limits malen.")

    def Soft_Limits_einstellen(self, U_x, O_x):
        if U_x is None:
            self.U_x = -50
        else:
            self.U_x = U_x
        if O_x is None:
            self.O_x = 1050
        else:
            self.O_x = O_x
        self.update()

        # M_d = 10.5
        # size = self.frameGeometry().width()
        # self.U_x = size - U_x * size / 1000
        # self.O_x = size - O_x * size / 1000
        # self.update()


class KalibrierungThread(QThread):
    """Thread für Kalibrierung der Motoren"""
    Kalibrierung_gestartet = pyqtSignal()
    Kalibrierung_fertig = pyqtSignal()
    Kalibrierung_unterbrochen = pyqtSignal()
    Kalibrierung_Status_Nachricht = pyqtSignal()
    stop = False
    status = ''
    box = None

    def start(self, box):
        super().start()
        self.box = box
        self.stop = False

    def run(self):
        calibration_reporter = GuiCalibrationReporter(self)
        stop_indicator = GuiStopIndicator(self)
        self.box.calibrate_motors(stop_indicator=stop_indicator, reporter=calibration_reporter)

        if self.stop:
            self.box.stop()
            self.Kalibrierung_unterbrochen.emit()
        else:
            self.Kalibrierung_fertig.emit()

    def report(self, text):
        self.status = text
        print(self.status)
        self.Kalibrierung_Status_Nachricht.emit()


class GuiCalibrationReporter(WaitReporter):
    """Durch dieses Objekt kann man während eine Kalibrierung die Liste der im Moment laufenden Motoren bekommen.
            Es wird als argument für PBox.calibrate_motors() verwendet."""

    def __init__(self, kal_thread: KalibrierungThread):
        self.kal_thread = kal_thread
        self.wait_list = set()

    def set_wait_list(self, wait_list: Set[str]):
        self.wait_list = wait_list
        self.__report()

    def motor_is_done(self, motor_name: str):
        self.wait_list -= {motor_name}
        self.__report()

    def __report(self):
        report = f'Wartet auf Motoren: {", ".join(self.wait_list)}'
        self.kal_thread.report(report)


class GuiStopIndicator(StopIndicator):
    """Durch dieses Objekt kann man Kalibrierung abbrechen.
    Es wird als argument für PBox.calibrate_motors() verwendet."""

    def __init__(self, kal_thread: KalibrierungThread):
        self.kal_thread = kal_thread

    def has_stop_requested(self) -> bool:
        return self.kal_thread.stop


class ExampleApp(QMainWindow):
    def __init__(self, parent=None):
        super(ExampleApp, self).__init__(parent)
        loadUi('GUI_form/mainwindow.ui', self)

        self.StatusBar = QStatusBar(self)
        self.StatusBar.setObjectName("statusbar")
        self.setStatusBar(self.StatusBar)

        self.Kal_Thread = KalibrierungThread()
        self.Kal_Thread.Kalibrierung_unterbrochen.connect(self.Kalibrierung_unterbrochen)
        self.Kal_Thread.Kalibrierung_fertig.connect(self.Kalibrierung_fertig)
        self.Kal_Thread.Kalibrierung_Status_Nachricht.connect(self.Kalibrierung_Status_zeigen)

        # self.c = Communicate()
        self.horizontalScrollBar1.setParent(None)

        self.horizontalScrollBar1 = AktPositionSlider()
        self.horizontalLayout_4.addWidget(self.horizontalScrollBar1)
        self.horizontalScrollBar1.setEnabled(False)
        self.horizontalScrollBar1.setMaximum(1000)
        self.horizontalScrollBar1.Soft_Limits_einstellen(None, None)

        self.refrBtn.clicked.connect(self.ports_lesen)
        self.VerbButton.clicked.connect(self.Verbinden)
        self.KalibrBtn.clicked.connect(self.Kalibrierung)
        self.StopButton.clicked.connect(self.Stop)
        self.minusBtn1.clicked.connect(self.Minus1)
        self.plusBtn1.clicked.connect(self.Plus1)
        self.NullBtn1.clicked.connect(self.Null_einstellen)
        self.configButton.clicked.connect(self.Config)

        self.horizontalSlider1.sliderReleased.connect(self.Schieber_geh_zu)
        self.GeheZuEdit1.returnPressed.connect(self.geh_zu)
        self.SL_U_Edit.textEdited.connect(self.Soft_Limits_einstellen)
        self.SL_O_Edit.textEdited.connect(self.Soft_Limits_einstellen)
        self.EinheitenBox1.stateChanged.connect(self.Einheiten_wechseln)
        self.MotorCBox.currentTextChanged.connect(self.Motor_wechseln)
        # self.c.Kal_done.connect(self.Position_lesen)

        self.NullBtn1.setEnabled(False)

        # self.line_Th_Einst.editingFinished.connect(self.Thermo_Einst_schicken)

        self.SchrittEdit1.setValidator(QtGui.QDoubleValidator())
        self.GeheZuEdit1.setValidator(QtGui.QDoubleValidator())
        self.SL_U_Edit.setValidator(QtGui.QDoubleValidator())
        self.SL_O_Edit.setValidator(QtGui.QDoubleValidator())

        self.Kal_in_Lauf = False
        self.Position_erneuern = True
        self.verbunden = False
        self.comm_emulation = False
        self.serial_emulation = False

        self.units = 'norm'

        self.ports_lesen()

        # self.horizontalSlider1.setTickPosition(300)
        # self.horizontalSlider1.setTickInterval(100)
        # self.drawLines()

        # self.horizontalSlider1.drawLines = drawLines
        # self.horizontalSlider1.paintEvent = paintEvent

    def Kalibrierung_Status_zeigen(self):
        self.StatusBar.showMessage(self.Kal_Thread.status)

    def Motoren_Namen_laden(self):
        self.MotorCBox.clear()
        Namen = self.Box.motors_names_list()
        for Name in Namen:
            self.MotorCBox.addItem(Name)

    def Motor_wechseln(self):
        if self.MotorCBox.currentText() != '':
            self.Motor = self.Box.get_motor_by_name(self.MotorCBox.currentText())
            self.init_Soft_Limits()
            self.Position_lesen(single_shot=True)
            if not self.Motor.with_initiators():
                self.horizontalSlider1.setEnabled(False)
                self.horizontalScrollBar1.setValue(0)
                self.horizontalSlider1.setValue(0)
                self.Motor1Box.setTitle(self.Motor.name)
            else:
                self.horizontalSlider1.setEnabled(True)
                self.set_HSlider_tr(int(self.Position))
                self.Motor1Box.setTitle(self.Motor.name)
            self.Motor1Box.setEnabled(True)
        else:
            self.Motor1Box.setEnabled(False)

    def Soft_Limits_Lines_Einheiten_anpassen(self):
        Motor = self.Motor

        U_Grenze = Motor.soft_limits[0]
        O_Grenze = Motor.soft_limits[1]

        if U_Grenze is not None:
            if self.EinheitenBox1.checkState():
                U_Grenze = Motor.transform_units(U_Grenze, 'norm', to='displ')
            self.SL_U_Edit.setText(str(round(U_Grenze, 4)))
        if O_Grenze is not None:
            if self.EinheitenBox1.checkState():
                O_Grenze = Motor.transform_units(O_Grenze, 'norm', to='displ')
            self.SL_O_Edit.setText(str(round(O_Grenze, 4)))

    def Soft_Limits_einstellen(self):
        Motor = self.Motor
        U_Grenze = self.SL_U_Edit.text()
        O_Grenze = self.SL_O_Edit.text()

        try:
            float(U_Grenze)
        except ValueError:
            U_Grenze = ''
            self.SL_U_Edit.setStyleSheet("color: red;")

        try:
            float(O_Grenze)
        except ValueError:
            O_Grenze = ''
            self.SL_O_Edit.setStyleSheet("color: red;")

        # print('Einstellen', Motor.Name, U_Grenze, O_Grenze)
        # print(U_Grenze, O_Grenze)

        if self.EinheitenBox1.checkState():
            pass
            U_Grenze = Motor.transform_units(float(U_Grenze), 'displ', to='norm') if U_Grenze != '' else None
            O_Grenze = Motor.transform_units(float(O_Grenze), 'displ', to='norm') if O_Grenze != '' else None
        else:
            U_Grenze = float(U_Grenze) if U_Grenze != '' else None
            O_Grenze = float(O_Grenze) if O_Grenze != '' else None

        Motor.soft_limits = (U_Grenze, O_Grenze)

        if U_Grenze is not None and O_Grenze is not None:
            if O_Grenze - U_Grenze < 0:
                self.SL_U_Edit.setStyleSheet("color: red;")
                self.SL_O_Edit.setStyleSheet("color: red;")
            else:
                self.SL_U_Edit.setStyleSheet("color:;")
                self.SL_O_Edit.setStyleSheet("color:;")
        else:
            self.SL_U_Edit.setStyleSheet("color:;")
            self.SL_O_Edit.setStyleSheet("color:;")

        if U_Grenze is None:
            self.SL_U_Edit.setStyleSheet("color: red;")
        if O_Grenze is None:
            self.SL_O_Edit.setStyleSheet("color: red;")

        self.horizontalScrollBar1.Soft_Limits_einstellen(U_Grenze, O_Grenze)

    def init_Soft_Limits(self):
        Motor = self.Motor

        U_Grenze = Motor.soft_limits[0]
        O_Grenze = Motor.soft_limits[1]
        # print('Init', Motor.Name, U_Grenze, O_Grenze)

        if U_Grenze is not None:
            if self.EinheitenBox1.checkState():
                U_Grenze = Motor.transform_units(U_Grenze, 'norm', to='displ')
            self.SL_U_Edit.setText(str(round(U_Grenze, 4)))
        else:
            self.SL_U_Edit.setText('')
        if O_Grenze is not None:
            if self.EinheitenBox1.checkState():
                O_Grenze = Motor.transform_units(O_Grenze, 'norm', to='displ')
            self.SL_O_Edit.setText(str(round(O_Grenze, 4)))
        else:
            self.SL_O_Edit.setText('')

        self.Soft_Limits_einstellen()

    def Einheiten_wechseln(self):
        if self.EinheitenBox1.checkState():
            self.Einheit_label.setText(self.Motor.config['display_units'])
            self.units = 'displ'
        else:
            self.Einheit_label.setText('NE')
            self.units = 'norm'
        self.NullBtn1.setEnabled(self.EinheitenBox1.checkState())
        self.Soft_Limits_Lines_Einheiten_anpassen()

    def geh_zu(self):
        self.Motor.go_to(float(self.GeheZuEdit1.text()), self.units)
        self.set_HSlider_tr(int(float(self.GeheZuEdit1.text())))

    def NE_aus_AE(self, AE):
        return self.Motor.transform_units(AE, 'displ', to='norm')

    def set_HSlider_tr(self, Val):
        if self.EinheitenBox1.checkState():
            self.set_HSlider(int(self.NE_aus_AE(Val)))
        else:
            self.set_HSlider(Val)

    def set_HSlider(self, Val):
        if self.Motor.with_initiators():
            self.horizontalSlider1.setValue(Val)

    def Plus1(self):
        self.Motor.go(float(self.SchrittEdit1.text()), self.units)
        self.set_HSlider_tr(self.Position + float(self.SchrittEdit1.text()))

    def Minus1(self):
        self.Motor.go(-float(self.SchrittEdit1.text()), self.units)
        self.set_HSlider_tr(self.Position - float(self.SchrittEdit1.text()))

    def Stop(self):
        self.Box.stop()
        self.set_HSlider(self.Position_NE)

    def Schieber_geh_zu(self):
        self.Motor.go_to(self.horizontalSlider1.value())
        if self.EinheitenBox1.checkState():
            self.GeheZuEdit1.setText(
                str(round(self.Motor.transform_units(self.horizontalSlider1.value(), 'norm', to='displ'), 4)))
        else:
            self.GeheZuEdit1.setText(str(self.horizontalSlider1.value()))

    def Null_einstellen(self):
        self.Motor.set_display_null()
        self.Soft_Limits_Lines_Einheiten_anpassen()

    def Config(self):
        self.Stop()
        fileName = QFileDialog.getOpenFileName(self, 'Config Datei öffnen', '',
                                               "Text Files(*.csv)")
        if fileName[0] == '':
            return

        self.Verbinden(config_Datei=fileName[0])

    def Verbinden(self, arg=None, config_Datei='input/Phytron_Motoren_config.csv'):

        if self.verbunden:
            self.Box.close()

        self.verbunden = True
        emulator = MCC2BoxEmulator(n_bus=5, n_axes=2, realtime=True)
        input_file = 'input/Phytron_Motoren_config.csv'
        # input_file = 'input/Jet_box_config.csv'
        emulator_input_file = 'input/Emulator_config.csv'
        if self.PortBox.currentText() == 'CommunicatorEmulator':
            self.Box = Box(emulator, input_file=emulator_input_file)
            self.comm_emulation = True
        elif self.PortBox.currentText() == 'SerialEmulator':
            connector = SerialConnector(emulator=emulator, beg_symbol=b'\x02', end_symbol=b'\x03')
            emul_communicator = MCC2Communicator(connector)
            self.Box = Box(emul_communicator, input_file=emulator_input_file)
            self.serial_emulation = True
        else:
            self.Box = MCC2BoxSerial(self.PortBox.currentText(), input_file=input_file)
        try:
            self.Box.read_saved_session_data()
        except FileNotFoundError:
            pass

        self.Motoren_Namen_laden()
        self.KalibrBtn.setEnabled(True)
        self.configButton.setEnabled(True)
        self.Motor1Box.setEnabled(True)
        self.MotorCBox.setEnabled(True)
        self.Motorlabel.setEnabled(True)
        self.Position_lesen()

        self.set_HSlider(int(self.Position))
        self.Motor1Box.setTitle(self.Motor.name)
        QMessageBox.information(self, "Verbindung abgeschlossen!",
                                self.Box.report)

    def Position_lesen(self, single_shot=False):
        if self.Position_erneuern:
            self.Position = self.Motor.position(self.units)
            self.Position_NE = self.Motor.position()
            self.AktPosEdit1.setText(str(round(self.Position, 4)))
            if self.Motor.with_initiators():
                self.horizontalScrollBar1.setValue(int(self.Position_NE))
        if not single_shot:
            QtCore.QTimer.singleShot(100, self.Position_lesen)

    def Kalibrierung_fertig(self):

        self.Kalibrierung_unterbrochen()
        self.StatusBar.clearMessage()

    def Kalibrierung_unterbrochen(self):
        self.Motor1Box.setEnabled(True)
        self.MotorCBox.setEnabled(True)
        self.Motorlabel.setEnabled(True)
        self.Position_erneuern = True
        self.Position_lesen(single_shot=True)
        self.set_HSlider(int(self.Position))

        self.VerbButton.setEnabled(True)
        self.configButton.setEnabled(True)
        self.Kal_in_Lauf = False
        self.KalibrBtn.setText("alle kalibrieren")
        self.StatusBar.showMessage("Kalibrierung wurde unterbrochen")

        if self.comm_emulation:
            self.Box.communicator.realtime = True
        elif self.serial_emulation:
            self.Box.communicator.connector.ser.realtime = True

    def Kalibrierung(self):

        if not self.Kal_in_Lauf:
            self.Kal_in_Lauf = True
            self.VerbButton.setEnabled(False)
            self.configButton.setEnabled(False)
            self.Motor1Box.setEnabled(False)
            self.MotorCBox.setEnabled(False)
            self.Motorlabel.setEnabled(False)
            self.KalibrBtn.setText("Stop")
            self.Position_erneuern = False

            if self.comm_emulation:
                self.Box.communicator.realtime = False
            elif self.serial_emulation:
                self.Box.communicator.connector.ser.realtime = False

            self.Kal_Thread.start(self.Box)
            print("Thr started")

            # Kal_Thread(q, self.Box)


        else:
            self.Kal_Thread.stop = True

    def ports_lesen(self):
        self.PortBox.clear()
        comlist = serial.tools.list_ports.comports()

        for element in comlist:
            self.PortBox.addItem(element.device)
        self.PortBox.addItem('SerialEmulator')
        self.PortBox.addItem('CommunicatorEmulator')

        if not self.Kal_in_Lauf:
            self.VerbButton.setEnabled(True)


    # QtCore.QTimer.singleShot(1000, self.weiter) # QUICKLY repeat


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('macintosh')
    form = ExampleApp()
    form.show()
    # form.update() #start with something
    app.exec_()

    if form.verbunden:
        form.Box.close()
    # try:
    #     form.Box.close()
    #     print("Exit ohne Fehler")
    # except:
    #     print("Exit mit Fehler")

    print("DONE")
