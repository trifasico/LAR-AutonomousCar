import serial
import threading
command = ""

Comando = serial.Serial('COM7', 57600)  # 115200

estado_Jetson = ""

def read():
    global estado_Jetson
    while Comando.is_open():
        if Comando.in_waiting:
            a = Comando.readline().decode()
            if(a == "12_E\n"):  #SETA ESQUERDA
                print("Seta Esquerda")
            elif(a == "12_D\n"):  #SETA DIREITA
                print("Seta DIREITA")
            elif(a == "12_F\n"):  #SETA FRENTE
                print("Seta FRENTE")
            elif(a == "13\n"):  #STOP
                print("STOP")
            elif(a == "14\n"):  #GO
                print("GO")
            elif(a == "15\n"):  #ESTACIONAMENTO
                print("ESTACIONAMENTO")


if __name__ == '__main__':
    readThread = threading.Thread(target=read)
    readThread.start()
    #while(readThread.is_alive()):
    #    command = input()
    #    command += "\r\n"
    #    print(str.encode("s: |" + command + "|"))
    #    Comando.write(str.encode(command))
