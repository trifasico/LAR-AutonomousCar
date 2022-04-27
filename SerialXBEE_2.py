import serial
import threading
Comando = serial.Serial('COM8', 57600)  # 115200

command = ""

def read():
    global command
    while command != "stop\r":
        if Comando.in_waiting:
            a = Comando.readline()
            print("r: |" + str(a) + "|")
    print("sai")


if __name__ == '__main__':
    readThread = threading.Thread(target=read)
    readThread.start()
    while(readThread.is_alive()):
        command = input()
        command += "\n"
        print(str.encode("s: |" + command + "|"))
        Comando.write(str.encode(command))
