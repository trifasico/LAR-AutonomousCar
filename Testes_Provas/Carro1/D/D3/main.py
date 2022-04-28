import threading
import cv2
import numpy as np
import math
import serial
import time

# Configs Debug
CAMERA_ON = 1
SERIAL_ON = 1


# import matplotlib.pyplot as plt
#
# Teste de diferenca dos lados

meio_x = 640
meio_y = 60


# COM JETSON
Comando = serial.Serial('/dev/ttyUSB0', 57600)  # 115200

estado_Jetson = ""


# noinspection PyShadowingNames
def Get_object():
    ret, img = cap.read()
    cv2.imshow("Frame1", img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow("Frame2", hsv)
    mask = cv2.inRange(hsv, (70, 150, 40), (90, 255, 255))
    cv2.imshow("Frame3", mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 0), 3)
    ox = -1
    oy = -1
    for i in contours:
        if cv2.contourArea(i) > 1000:
            m = cv2.moments(i)
            ox = int(m['m10'] / m['m00'])
            oy = int(m['m01'] / m['m00'])
            print(ox, oy)
    return ox, oy


# noinspection PyShadowingNames
def Get_Filtred_Image():
    ret, frame = cap.read()
    # cv2.imshow('frame3', frame)
    h, status = cv2.findHomography(pts_src, pts_dst)
    im_dst = cv2.warpPerspective(frame, h, (frame.shape[1], frame.shape[0]))
    cv2.imshow('frame4', im_dst)
    # 2- HSV2
    hsv = cv2.cvtColor(im_dst, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    # cv2.imshow('frameh', h)
    # cv2.imshow('frames', s)
    # cv2.imshow('framev', v)
    # 3- Filter (Remove noise)

    ret, thresh1 = cv2.threshold(v, 220, 255, cv2.THRESH_BINARY)
    # th3 = cv2.adaptiveThreshold(v,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,21,50)

    thresh1 = cv2.fillPoly(thresh1, [car_pts], 0)
    # cv2.imshow('Threshold',thresh1)
    opening = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, open_elem)
    cv2.imshow('frame2', opening)
    return opening, im_dst


# MACROS RETANGULOS
Max_Rect_area = 230
Min_Rect_area = 130
MARGEM_RECT = 5


# noinspection PyShadowingNames
def Get_Rectangles_Points(contours, im_dst):
    Pontos_Final = np.empty((0, 2), int)
    n = 0
    for contour in contours:

        approx = cv2.approxPolyDP(contour, 0.035 * cv2.arcLength(contour, True), True)
        cv2.drawContours(im_dst, [approx], 0, (0, 255, 0), 3)

        if len(approx) == 4 and Min_Rect_area < cv2.arcLength(contour, True) < Max_Rect_area:
            cv2.drawContours(im_dst, [approx], 0, (0, 0, 255), 3)
            x1 = int((approx[0][0][0] + approx[1][0][0]) / 2)
            x2 = int((approx[1][0][0] + approx[2][0][0]) / 2)
            x3 = int((approx[2][0][0] + approx[3][0][0]) / 2)
            x4 = int((approx[3][0][0] + approx[0][0][0]) / 2)
            y1 = int((approx[0][0][1] + approx[1][0][1]) / 2)
            y2 = int((approx[1][0][1] + approx[2][0][1]) / 2)
            y3 = int((approx[2][0][1] + approx[3][0][1]) / 2)
            y4 = int((approx[3][0][1] + approx[0][0][1]) / 2)

            l1 = math.sqrt((int(x1) - int(x3)) ** 2 + (int(y1) - int(y3)) ** 2)
            l2 = math.sqrt((int(x2) - int(x4)) ** 2 + (int(y2) - int(y4)) ** 2)
            d1 = math.sqrt((approx[0][0][0] - approx[1][0][0]) ** 2 + (approx[0][0][1] - approx[1][0][1]) ** 2)
            d2 = math.sqrt((approx[1][0][0] - approx[2][0][0]) ** 2 + (approx[1][0][1] - approx[2][0][1]) ** 2)
            d3 = math.sqrt((approx[2][0][0] - approx[3][0][0]) ** 2 + (approx[2][0][1] - approx[3][0][1]) ** 2)
            d4 = math.sqrt((approx[3][0][0] - approx[0][0][0]) ** 2 + (approx[3][0][1] - approx[0][0][1]) ** 2)

            if abs(d1 - d3) < MARGEM_RECT and abs(d2 - d4) < MARGEM_RECT:
                n += 1
                cv2.drawContours(im_dst, [approx], 0, (255, 0, 0), 3)
                if l2 > l1:
                    Pontos_Final = np.append(Pontos_Final, np.array([[x2, y2]]), axis=0)
                    Pontos_Final = np.append(Pontos_Final, np.array([[x4, y4]]), axis=0)
                    im_dst = cv2.circle(im_dst, (x2, y2), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x4, y4), 7, (255, 255, 0), 3)
                else:
                    Pontos_Final = np.append(Pontos_Final, np.array([[x1, y1]]), axis=0)
                    Pontos_Final = np.append(Pontos_Final, np.array([[x3, y3]]), axis=0)
                    im_dst = cv2.circle(im_dst, (x1, y1), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x3, y3), 7, (255, 255, 0), 3)
    return Pontos_Final, n, im_dst


Max_Passadeira_area = 450
Min_Passadeira_area = 250


# noinspection PyShadowingNames
def Get_Rectangles_Points_Passadeira(contours, im_dst):
    Pontos_Final = np.empty((0, 2), int)
    Pontos_Passadeira = np.empty((0, 2), int)
    n = 0
    n1 = 0
    for contour in contours:

        approx = cv2.approxPolyDP(contour, 0.035 * cv2.arcLength(contour, True), True)

        if len(approx) == 4 and Min_Passadeira_area < cv2.arcLength(contour, True) < Max_Passadeira_area:
            cv2.drawContours(im_dst, [approx], 0, (0, 0, 255), 3)
            x1 = int((approx[0][0][0] + approx[1][0][0]) / 2)
            x2 = int((approx[1][0][0] + approx[2][0][0]) / 2)
            x3 = int((approx[2][0][0] + approx[3][0][0]) / 2)
            x4 = int((approx[3][0][0] + approx[0][0][0]) / 2)
            y1 = int((approx[0][0][1] + approx[1][0][1]) / 2)
            y2 = int((approx[1][0][1] + approx[2][0][1]) / 2)
            y3 = int((approx[2][0][1] + approx[3][0][1]) / 2)
            y4 = int((approx[3][0][1] + approx[0][0][1]) / 2)

            l1 = math.sqrt((int(x1) - int(x3)) ** 2 + (int(y1) - int(y3)) ** 2)
            l2 = math.sqrt((int(x2) - int(x4)) ** 2 + (int(y2) - int(y4)) ** 2)
            d1 = math.sqrt((approx[0][0][0] - approx[1][0][0]) ** 2 + (approx[0][0][1] - approx[1][0][1]) ** 2)
            d2 = math.sqrt((approx[1][0][0] - approx[2][0][0]) ** 2 + (approx[1][0][1] - approx[2][0][1]) ** 2)
            d3 = math.sqrt((approx[2][0][0] - approx[3][0][0]) ** 2 + (approx[2][0][1] - approx[3][0][1]) ** 2)
            d4 = math.sqrt((approx[3][0][0] - approx[0][0][0]) ** 2 + (approx[3][0][1] - approx[0][0][1]) ** 2)

            # print("PASSADERIA",cv2.arcLength(contour, True))
            if abs(d1 - d3) < 3 + MARGEM_RECT and abs(d2 - d4) < 3 + MARGEM_RECT:
                n1 += 1
                cv2.drawContours(im_dst, [approx], 0, (255, 255, 0), 3)

                if l2 > l1:
                    Pontos_Passadeira = np.append(Pontos_Passadeira, np.array([[(x2 + x4) / 2, (y2 + y4) / 2]]), axis=0)
                    # Pontos_Final=np.append(Pontos_Final,np.array([[x4,y4]]),axis=0)
                    # print(x2,y2,x4,y4)
                    im_dst = cv2.circle(im_dst, (x2, y2), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x4, y4), 7, (255, 255, 0), 3)
                else:
                    Pontos_Passadeira = np.append(Pontos_Passadeira, np.array([[(x1 + x3) / 2, (y1 + y3) / 2]]), axis=0)
                    # Pontos_Final=np.append(Pontos_Final,np.array([[x3,y3]]),axis=0)
                    # print(x1,y1,x3,y3)
                    im_dst = cv2.circle(im_dst, (x1, y1), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x3, y3), 7, (255, 255, 0), 3)

        if len(approx) == 4 and cv2.arcLength(contour, True) > Min_Rect_area and cv2.arcLength(contour,
                                                                                               True) < Max_Rect_area:
            cv2.drawContours(im_dst, [approx], 0, (0, 0, 255), 3)

            # print(cv2.arcLength(contour, True))
            # cv2.drawContours(im_dst, [approx], 0, (255, 0, 0), 5)
            # np.append(xpts,1)
            # print(approx)
            # print(approx[0])
            # print(approx[0][0])
            # print(approx[0][0][0])
            x1 = int((approx[0][0][0] + approx[1][0][0]) / 2)
            x2 = int((approx[1][0][0] + approx[2][0][0]) / 2)
            x3 = int((approx[2][0][0] + approx[3][0][0]) / 2)
            x4 = int((approx[3][0][0] + approx[0][0][0]) / 2)
            y1 = int((approx[0][0][1] + approx[1][0][1]) / 2)
            y2 = int((approx[1][0][1] + approx[2][0][1]) / 2)
            y3 = int((approx[2][0][1] + approx[3][0][1]) / 2)
            y4 = int((approx[3][0][1] + approx[0][0][1]) / 2)

            l1 = math.sqrt((int(x1) - int(x3)) ** 2 + (int(y1) - int(y3)) ** 2)
            l2 = math.sqrt((int(x2) - int(x4)) ** 2 + (int(y2) - int(y4)) ** 2)
            d1 = math.sqrt((approx[0][0][0] - approx[1][0][0]) ** 2 + (approx[0][0][1] - approx[1][0][1]) ** 2)
            d2 = math.sqrt((approx[1][0][0] - approx[2][0][0]) ** 2 + (approx[1][0][1] - approx[2][0][1]) ** 2)
            d3 = math.sqrt((approx[2][0][0] - approx[3][0][0]) ** 2 + (approx[2][0][1] - approx[3][0][1]) ** 2)
            d4 = math.sqrt((approx[3][0][0] - approx[0][0][0]) ** 2 + (approx[3][0][1] - approx[0][0][1]) ** 2)

            if abs(d1 - d3) < MARGEM_RECT and abs(d2 - d4) < MARGEM_RECT:
                # print("Rectangulo",cv2.arcLength(contour, True))
                n += 1
                cv2.drawContours(im_dst, [approx], 0, (255, 0, 0), 3)
                if l2 > l1:
                    Pontos_Final = np.append(Pontos_Final, np.array([[x2, y2]]), axis=0)
                    Pontos_Final = np.append(Pontos_Final, np.array([[x4, y4]]), axis=0)
                    # print(x2,y2,x4,y4)
                    im_dst = cv2.circle(im_dst, (x2, y2), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x4, y4), 7, (255, 255, 0), 3)
                else:
                    Pontos_Final = np.append(Pontos_Final, np.array([[x1, y1]]), axis=0)
                    Pontos_Final = np.append(Pontos_Final, np.array([[x3, y3]]), axis=0)
                    # print(x1,y1,x3,y3)
                    im_dst = cv2.circle(im_dst, (x1, y1), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x3, y3), 7, (255, 255, 0), 3)
    return Pontos_Final, n, Pontos_Passadeira, n1, im_dst


# noinspection PyShadowingNames
def Find_Lateral_Ref(opening):
    Pontos_Final = np.empty((0, 2), int)
    j_y = 479
    ant_x = 0
    ant_y = 0
    print("Novo")
    while opening[j_y][640] != 255 and j_y > 0:
        print(j_y)
        if j_y % 10 == 0:
            j_x = 0
            white_deteted = 0
            while not white_deteted and j_y < camHeight and 640 + j_x < camWidth - 1:
                print("Entrou")
                if opening[j_y][640 + j_x] == 255:
                    # im_dst = cv2.circle(im_dst, (int(640+j_x),int(j_y)), 7, (255,0,255), 3)
                    if ant_x != 0 and ant_y != 0:

                        xpts_dist = ant_x - 640 - j_x
                        ypts_dist = ant_y - j_y + (np.random.uniform() / 100)
                        if math.sqrt(xpts_dist ** 2 + ypts_dist ** 2) < 50:

                            m = xpts_dist / ypts_dist
                            # print("Aqui",xpts_dist,ypts_dist,m)
                            mean_y = (ant_y + j_y) / 2
                            mean_x = (ant_x + j_x + 640) / 2
                            # im_dst = cv2.circle(im_dst, (int(mean_x),int(mean_y)), 7, (255,255,255), 3)
                            i_x = 0  # mean_x
                            i_y = 0  # mean_y

                            l3 = 0  # math.sqrt((i_x)**2+(480-i_y)**2)

                            while l3 < 55:
                                # print("L3",l3)
                                i_y = int((i_x * m) + mean_y)
                                # print(i_y,mean_y)
                                l3 = math.sqrt(i_x ** 2 + (i_y - mean_y) ** 2)
                                # im_dst = cv2.circle(im_dst, (int(mean_x-i_x),int(i_y)), 1, (255,255,255), 1)
                                i_x += 1

                            Pontos_Final = np.append(Pontos_Final, np.array([[int(mean_x - i_x), int(i_y)]]), axis=0)
                        # im_dst = cv2.circle(im_dst, (int(mean_x-i_x),int(i_y)), 3, (255,255,255), 3)
                        else:
                            return Pontos_Final
                    white_deteted = 1

                    ant_x = 640 + j_x
                    ant_y = j_y

                j_x += 1
        j_y -= 1
    return Pontos_Final


# MACROS
Y_to_find = 500


# noinspection PyShadowingNames
def Find_closest_point(Pontos_Final, Xpoint, Ypoint):
    melhor_ponto = Pontos_Final[0]
    dst_ant = 500
    n = 0
    # print(Pontos_Final.shape[0])
    for i in range(Pontos_Final.shape[0] - 1):
        pnt = Pontos_Final[i + 1]
        dst = math.sqrt((pnt[0] - Xpoint) ** 2 + (pnt[1] - Ypoint) ** 2)
        if dst_ant > dst and pnt[1] > 200:
            dst_ant = dst
            melhor_ponto = pnt
            n = i + 1

    if n % 2 == 1:
        # print("Melhor1:",melhor_ponto, Pontos_Final[n-1])
        return melhor_ponto, Pontos_Final[n - 1]
    else:
        # print("Melhor#########:",melhor_ponto, Pontos_Final[n+1])
        return melhor_ponto, Pontos_Final[n + 1]


Kp = 0.03
OFFSET = 640  # 640   #700


def Calc_direction(error):
    if error - OFFSET > 0:
        return ((error - OFFSET) * ((error - OFFSET) / 4)) * Kp
    else:
        return -((error - OFFSET) * ((error - OFFSET) / 4)) * Kp


# noinspection PyShadowingNames
def Set_direction(Direcao):
    if Direcao > 100:
        Direcao = 100
    if Direcao < -100:
        Direcao = -100
    arduino.write(("D." + str(Direcao) + "\r").encode())
    return Direcao


def Set_state(state):
    arduino.write(("S." + str(state) + "\r").encode())


passadeira_h = cv2.imread('passadeira2.jpg')
THRESH_PASSADEIRA = 30


# noinspection PyShadowingNames
def Get_Rect_Angulo(melhor_ponto, outro_ponto):
    if melhor_ponto[1] > outro_ponto[1]:
        angulo_atual = math.atan2(melhor_ponto[1] - outro_ponto[1], melhor_ponto[0] - outro_ponto[0]) / np.pi * 180
    # print(angulo_atual)
    else:
        angulo_atual = math.atan2(outro_ponto[1] - melhor_ponto[1], outro_ponto[0] - melhor_ponto[0]) / np.pi * 180
    # print(angulo_atual)
    return angulo_atual


# ======= Init Camera
if CAMERA_ON:
    cap = cv2.VideoCapture(-1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
else:
    cap = cv2.VideoCapture('recorded2.mp4')

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(width, height)

cv2.namedWindow('frame')
# Variables Initialization:
# 1- Homografy
camHeight = height
camWidth = width
pts_src = np.array([[0, 0], [camWidth, 0], [camWidth, camHeight], [0, camHeight]])
rs = int(camWidth * 0.385)
pts_dst = np.array([[0, 0], [camWidth, 0], [camWidth - rs, camHeight], [rs, camHeight]])
h, status = cv2.findHomography(pts_src, pts_dst)
im_dst = np.zeros((camHeight, camWidth))
# 2- HSV
# 3- Filter
im_filtred = np.zeros((camHeight, camWidth))
open_elem = np.ones((3, 3), np.uint8)
# 3.1 Remove Car Shape
car_pts = np.array([[270, 479], [270, 445], [294, 445], [364, 445], [386, 445], [387, 479]], np.int32)
car_pts = car_pts.reshape((-1, 1, 2))
# 4- Find Rectangles
# 5- Draw Rectangles Line
# 6- Clean Rectangles
# 7- Find Side Lines
# 8- Validate side lines
# Init Serial
if SERIAL_ON:
    arduino = serial.Serial('/dev/ttyACM0', 115200)  # /dev/ttyACM0
    arduino.close()
    arduino.open()
    while not arduino.isOpen():
        print("Waiting")
    time.sleep(2)
    arduino.write("V.60".encode())

Direcao = 0
Direcao_ant = 0
Passadeira_Encontrada = False
Seta_Direita = False
Seta_Esquerda = False
Parque = True


def read():
    global estado_Jetson
    global Parque
    while 1:
        if Comando.in_waiting:
            a = Comando.readline().decode()
            estado_Jetson = a
            print(estado_Jetson.encode())
            if a == "12_E\r\n":  # SETA ESQUERDA
                print("Seta Esquerda")
            elif a == "12_D\r\n":  # SETA DIREITA
                print("Seta DIREITA")

            elif a == "12_F\r\n":  # SETA FRENTE
                print("Seta FRENTE")
            elif a == "13\r\n":  # STOP
                print("STOP")
            elif a == "14\r\n":  # GO
                print("GO")
            elif a == "15\r\n":  # ESTACIONAMENTO
                Parque = 1
                print("ESTACIONAMENTO")


readThread = threading.Thread(target=read)
readThread.start()
first_time = True

while True:

    opening, im_dst = Get_Filtred_Image()

    x, y = Get_object()
    print(x,y)
    # ---------------NORMAL---------------
    contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # 2- Find Rectangles
    Pontos_Final, n, Pontos_Passadeira, n1, im_dst = Get_Rectangles_Points_Passadeira(contours, im_dst)
    # Pontos_Final , n,im_dst = Get_Rectangles_Points(contours,im_dst)

    # ---------------SE REFERENCIA---------------
    if n or n1:

        # ---------------SE PASSADEIRA---------------
        if n1 > 4:
            # Set_state(0)
            soma_x = 0
            count = 0
            for ponto in Pontos_Passadeira:
                soma_x += ponto[0]
                count += 1
            soma_x /= count
            Direcao = Calc_direction(soma_x)
        elif n > 0:
            melhor_ponto, outro_ponto = Find_closest_point(Pontos_Final, 640, Y_to_find)

            angulo_atual = Get_Rect_Angulo(melhor_ponto, outro_ponto)
            im_dst = cv2.circle(im_dst, (int(melhor_ponto[0]), int(melhor_ponto[1])), 3, (255, 0, 255), 2)
            print("ANGULO:", angulo_atual)
            # ---------------SAIR DO CRUZAMENTO PARA ESQUERDA---------------
            if angulo_atual < 40:
                Direcao = -50  # (angulo_atual-40)*3-25 #-70
                print("Angulo:", Direcao)

            # ---------------SAIR DO CRUZAMENTO PARA DIREITA---------------
            elif angulo_atual > 135:
                Direcao = 50  # (angulo_atual-140)*4+25 #50
                print("Angulo2:", Direcao)

            # ---------------SEGUIR LINHA---------------
            else:
                # Set_state(2)
                Direcao = Calc_direction(melhor_ponto[0])

            if x != -1 and y != -1:
                if x > meio_x and y > meio_y:
                    
                    if first_time:
                        arduino.write("V.30".encode())
                        OFFSET = 760
                        Kp = 0.15
                        first_time = False
                elif x < meio_x and y > meio_y:
                   
                    if first_time:
                        arduino.write("V.30".encode())
                        Kp = 0.15
                        OFFSET = 560
                        first_time = False

        Direcao = Direcao * 0.9 + Direcao_ant * 0.1
    #
    # im_dst = cv2.circle(im_dst, (int(outro_ponto[0]),int(outro_ponto[1])), 3, (0,255,255), 2)

    # ---------------SE NAO HOUVER REFERENCIA---------------
    else:
        if Direcao > 0:
            Direcao += 10
        else:
            Direcao -= 10
    Direcao = Set_direction(Direcao)
    print(Direcao)
    Direcao_ant = Direcao
    # print("Direção:",Direcao)

    im_dst = cv2.circle(im_dst, (int(640), int(Y_to_find)), 3, (0, 0, 0), 2)
    cv2.imshow('frame', im_dst)
    cv2.waitKey(20)
