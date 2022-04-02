import os
import serial
import pygame
import time
from threading import Thread

ser = serial.Serial('COM3', 9600)

velocidade = 0
direcao = 0
check = False


def sendSerial(trama):
    global ser
    ser.write(trama.encode())


def filter(direcao):
    if direcao < 10 and direcao > -10:
        direcao = 0
    direcao = (int)(direcao/5)
    direcao = direcao * 5
    return direcao


def read():
    global ser
    while True:
        if ser.in_waiting:
            a = ser.readline()
            print("r: |" + str(a) + "|")
        time.sleep(0.1)


class PS4Controller(object):
    controller = None
    axis_data = None
    button_data = None
    hat_data = None
    time.sleep(1.8)

    def init(self):
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()

    def listen(self):
        global velocidade
        global direcao
        global check

        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    self.axis_data[event.axis] = round(event.value, 2)
                    if event.axis == 0:
                        direcao = -(int)(self.axis_data[event.axis]*100)
                        direcao = filter(direcao)
                        buffer_ = 'D.' + str(direcao) + '\r'
                        print(buffer_)
                        sendSerial(buffer_)

                    if event.axis == 5:
                        if check == True:
                            velocidade = (int)(
                                self.axis_data[event.axis]*50)+50
                            # corrige deslocamento do comando
                            velocidade = (velocidade/85)*100
                            if velocidade > 100:
                                velocidade = 100
                            if velocidade < 0:
                                velocidade = 0
                            velocidade = round(velocidade, 0)
                            buffer_ = 'V.' + str(velocidade) + '\r'
                            print(buffer_)
                            sendSerial(buffer_)
                        else:
                            print("Please click de X button to allow him to move")

                    # if event.axis == 4:
                        #velocidade = (int)(self.axis_data[event.axis]*50)+50
                        #buffer_ = 'V2.' + str(velocidade) + '\r'
                        # sendSerial(buffer_)

                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:
                        self.button_data[event.button] = True
                        check = True
                    if event.button == 3:
                        buffer_ = 'P.0\r'
                        sendSerial(buffer_)
                        print(buffer_)
                    if event.button == 2:
                        buffer_ = 'P.' + str(1) + '\r'
                        sendSerial(buffer_)
                        print(buffer_)

                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == 0:
                        check = False
                        buffer_ = 'V.' + str(0) + '\r'
                        sendSerial(buffer_)
                        print(buffer_)
                        buffer_ = 'B.' + str(1) + '\r'
                        print(buffer_)
                        sendSerial(buffer_)
                        self.button_data[event.button] = False

                elif event.type == pygame.JOYHATMOTION:
                    print("Joy Hat Motion")
                    self.hat_data[event.hat] = event.value

                # time.sleep(0.1)


if __name__ == "__main__":
    Thread(target=read).start()
    ps4 = PS4Controller()
    ps4.init()
    ps4.listen()
