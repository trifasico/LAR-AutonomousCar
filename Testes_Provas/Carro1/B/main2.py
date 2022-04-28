import cv2
import numpy as np
import math
import serial
import time
import datetime

# Configs Debug
CAMERA_ON = 1
SERIAL_ON = 1

# ------------------------Maquina de estados------------------------
# ----------Estados----------
idle_s = 0
normal_s = 1
virar_s = 2
cones_s = 3
obstaculo_s = 4
parques_s = 5

# ----------Inputs----------
start = 1
stop = 0
s_i = 0
c_i = 0
ox_i = 0
oy_i = 0
p_i = 0
T = 0
inline = 0

next_state = idle_s

# ------------------------Macro Imagem------------------------
midlle_x = 640
midlle_y = 200

# ------------------------Macro Parques------------------------
select_park = 0  # 0 para paralelo e 1 para o P


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
    h, status = cv2.findHomography(pts_src, pts_dst)
    im_dst = cv2.warpPerspective(frame, h, (frame.shape[1], frame.shape[0]))
    cv2.imshow('frame4', im_dst)
    hsv = cv2.cvtColor(im_dst, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    ret, thresh1 = cv2.threshold(v, 220, 255, cv2.THRESH_BINARY)
    thresh1 = cv2.fillPoly(thresh1, [car_pts], 0)
    opening = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, open_elem)
    cv2.imshow('frame2', opening)
    return opening, im_dst


# MACROS RETANGULOS
Max_Rect_area = 230
Min_Rect_area = 130
MARGEM_RECT = 5


# noinspection PyShadowingNames
def Get_Rectangles_Points(contours, im_dst):
    pontos_final = np.empty((0, 2), int)
    n = 0
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.05 * cv2.arcLength(contour, True), True)
        cv2.drawContours(im_dst, [approx], 0, (0, 255, 0), 3)
        if len(approx) == 4 and cv2.arcLength(contour, True) > Min_Rect_area and cv2.arcLength(contour,
                                                                                               True) < Max_Rect_area:
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
                    pontos_final = np.append(pontos_final, np.array([[x2, y2]]), axis=0)
                    pontos_final = np.append(pontos_final, np.array([[x4, y4]]), axis=0)
                    im_dst = cv2.circle(im_dst, (x2, y2), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x4, y4), 7, (255, 255, 0), 3)
                else:
                    pontos_final = np.append(pontos_final, np.array([[x1, y1]]), axis=0)
                    pontos_final = np.append(pontos_final, np.array([[x3, y3]]), axis=0)
                    im_dst = cv2.circle(im_dst, (x1, y1), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x3, y3), 7, (255, 255, 0), 3)
    return pontos_final, n, im_dst


Max_Passadeira_area = 450
Min_Passadeira_area = 250


# noinspection PyShadowingNames
def Get_Rectangles_Points_Passadeira(contours, im_dst):
    pontos_final = np.empty((0, 2), int)
    pontos_passadeira = np.empty((0, 2), int)
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
                    pontos_passadeira = np.append(pontos_passadeira, np.array([[(x2 + x4) / 2, (y2 + y4) / 2]]), axis=0)
                    # Pontos_Final=np.append(Pontos_Final,np.array([[x4,y4]]),axis=0)
                    # print(x2,y2,x4,y4)
                    im_dst = cv2.circle(im_dst, (x2, y2), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x4, y4), 7, (255, 255, 0), 3)
                else:
                    pontos_passadeira = np.append(pontos_passadeira, np.array([[(x1 + x3) / 2, (y1 + y3) / 2]]), axis=0)
                    # Pontos_Final=np.append(Pontos_Final,np.array([[x3,y3]]),axis=0)
                    # print(x1,y1,x3,y3)
                    im_dst = cv2.circle(im_dst, (x1, y1), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x3, y3), 7, (255, 255, 0), 3)

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
                # print("Rectangulo",cv2.arcLength(contour, True))
                n += 1
                cv2.drawContours(im_dst, [approx], 0, (255, 0, 0), 3)
                if l2 > l1:
                    pontos_final = np.append(pontos_final, np.array([[x2, y2]]), axis=0)
                    pontos_final = np.append(pontos_final, np.array([[x4, y4]]), axis=0)
                    # print(x2,y2,x4,y4)
                    im_dst = cv2.circle(im_dst, (x2, y2), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x4, y4), 7, (255, 255, 0), 3)
                else:
                    pontos_final = np.append(pontos_final, np.array([[x1, y1]]), axis=0)
                    pontos_final = np.append(pontos_final, np.array([[x3, y3]]), axis=0)
                    im_dst = cv2.circle(im_dst, (x1, y1), 7, (255, 255, 0), 3)
                    im_dst = cv2.circle(im_dst, (x3, y3), 7, (255, 255, 0), 3)
    return Pontos_Final, n, Pontos_Passadeira, n1, im_dst


# noinspection PyShadowingNames
def Find_Lateral_Ref(opening):
    pontos_final = np.empty((0, 2), int)
    j_y = 479
    ant_x = 0
    ant_y = 0
    while opening[j_y][320] != 255 and j_y > 0:
        print(j_y)
        if j_y % 10 == 0:
            j_x = 0
            white_deteted = 0
            while not white_deteted and j_y < camHeight and 320 + j_x < camWidth - 1:
                if opening[j_y][320 + j_x] == 255:
                    if ant_x != 0 and ant_y != 0:
                        xpts_dist = ant_x - 320 - j_x
                        ypts_dist = ant_y - j_y + (np.random.uniform() / 100)
                        if math.sqrt(xpts_dist ** 2 + ypts_dist ** 2) < 50:
                            m = xpts_dist / ypts_dist
                            mean_y = (ant_y + j_y) / 2
                            mean_x = (ant_x + j_x + 320) / 2
                            i_x = 0  # mean_x
                            i_y = 0  # mean_y
                            l3 = 0  # math.sqrt((i_x)**2+(480-i_y)**2)
                            while l3 < 55:
                                i_y = int((i_x * m) + mean_y)
                                l3 = math.sqrt(i_x ** 2 + (i_y - mean_y) ** 2)
                                i_x += 1

                            pontos_final = np.append(pontos_final, np.array([[int(mean_x - i_x), int(i_y)]]), axis=0)
                        else:
                            return pontos_final
                    white_deteted = 1
                    ant_x = 320 + j_x
                    ant_y = j_y
                j_x += 1
        j_y -= 1
    return pontos_final


# MACROS
Y_to_find = 500


# noinspection PyShadowingNames
def Find_closest_point(Pontos_Final, Xpoint, Ypoint):
    melhor_ponto = Pontos_Final[0]
    dst_ant = 500
    n = 0
    for i in range(Pontos_Final.shape[0] - 1):
        pnt = Pontos_Final[i + 1]
        dst = math.sqrt((pnt[0] - Xpoint) ** 2 + (pnt[1] - Ypoint) ** 2)
        if dst_ant > dst and pnt[1] > 200:
            dst_ant = dst
            melhor_ponto = pnt
            n = i + 1

    if n % 2 == 1:
        return melhor_ponto, Pontos_Final[n - 1]
    else:
        return melhor_ponto, Pontos_Final[n + 1]


# MACROS
Kp = 0.03
OFFSET = 640  # 640   #700


def Calc_direction(error):
    if error - OFFSET > 0:
        return ((error - OFFSET) * ((error - OFFSET) / 2)) * Kp
    else:
        return -((error - OFFSET) * ((error - OFFSET) / 2)) * Kp


# noinspection PyShadowingNames
def Set_direction(direcao):
    if direcao > 100:
        direcao = 100
    if direcao < -100:
        direcao = -100
    arduino.write(("D." + str(direcao) + "\r").encode())
    return direcao


# noinspection PyShadowingNames
def Set_state(state):
    arduino.write(("S." + str(state) + "\r").encode())


# noinspection PyShadowingNames
def Get_Rect_Angulo(melhor_ponto, outro_ponto):
    if melhor_ponto[1] > outro_ponto[1]:
        angulo_atual = math.atan2(melhor_ponto[1] - outro_ponto[1], melhor_ponto[0] - outro_ponto[0]) / np.pi * 180
    else:
        angulo_atual = math.atan2(outro_ponto[1] - melhor_ponto[1], outro_ponto[0] - melhor_ponto[0]) / np.pi * 180

    return angulo_atual


# noinspection PyShadowingNames
def Get_cones(img_aux):
    cv2.imshow("", img_aux)
    mask = cv2.inRange(img_aux, (5, 100, 100), (25, 255, 255))
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for i in contours:
        print(cv2.contourArea(i))
        if cv2.contourArea(i) > 10000:
            m = cv2.moments(i)
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            print(cx, cy)
            return cx, cy
    return 0, 0


# ======= Init Camera
if CAMERA_ON:
    cap = cv2.VideoCapture(-1)
else:
    cap = cv2.VideoCapture('recorded2.mp4')

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

# Init Serial
if SERIAL_ON:
    arduino = serial.Serial('/dev/ttyACM0', 115200)  # /dev/ttyACM0
    arduino.close()
    arduino.open()
    while not arduino.isOpen():
        print("Waiting")
    time.sleep(2)
    arduino.write("V.30".encode())

Direcao = 0
Direcao_ant = 0
Passadeira_Encontrada = False
Seta_Direita = 0
Seta_Esquerda = 0

while True:
    park_cnt = 0
    state = next_state
    Passadeira_Encontrada = False
    if state != idle_s:
        opening, im_dst = Get_Filtred_Image()
        # ------------NORMAL------------
        contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # 2- Find Rectangles
        pontos_final, n, pontos_passadeira, n1, im_dst = Get_Rectangles_Points_Passadeira(contours, im_dst)
        # Pontos_Final , n,im_dst = Get_Rectangles_Points(contours,im_dst)

        # ------------SE REFERENCIA------------
        if n or n1:

            # ------------SE PASSADEIRA------------
            if n1 > 4:
                soma_x = 0
                count = 0
                Passadeira_Encontrada = True
                for ponto in pontos_passadeira:
                    soma_x += ponto[0]
                    count += 1
                soma_x /= count
                Direcao = Calc_direction(soma_x)
                tempo_passadeira = datetime.datetime.now() + datetime.timedelta(seconds=3)

            elif n > 0:
                melhor_ponto, outro_ponto = Find_closest_point(pontos_final, 640, Y_to_find)

                angulo_atual = Get_Rect_Angulo(melhor_ponto, outro_ponto)
                im_dst = cv2.circle(im_dst, (int(melhor_ponto[0]), int(melhor_ponto[1])), 3, (255, 0, 255), 2)
                print("ANGULO:", angulo_atual)
                # ------------SAIR DO CRUZAMENTO PARA ESQUERDA------------Tiver uma linha perpendicular
                if angulo_atual < 40:
                    Direcao = -50  # (angulo_atual-40)*3-25 #-70
                    print("Angulo:", Direcao)

                # ------------SAIR DO CRUZAMENTO PARA DIREITA------------
                elif angulo_atual > 135:
                    Direcao = 50  # (angulo_atual-140)*4+25 #50
                    print("Angulo2:", Direcao)

                # ------------SEGUIR LINHA------------
                else:
                    # Set_state(2)
                    Direcao = Calc_direction(melhor_ponto[0])

            Direcao = Direcao * 0.2 + Direcao_ant * 0.8
        # ---------------SE NAO HOUVER REFERENCIA---------------
        else:
            if Direcao > 0:
                Direcao += 10
            else:
                Direcao -= 10
        Direcao = Set_direction(Direcao)
        Direcao_ant = Direcao
        im_dst = cv2.circle(im_dst, (int(640), int(Y_to_find)), 3, (0, 0, 0), 2)
        cv2.imshow('frame', im_dst)
        ox_i, oy_i = Get_object()

    if state == idle_s:
        print("Idle")
        arduino.write("V.0".encode())
        if start:
            next_state = normal_s
        else:
            next_state = idle_s

    elif state == normal_s:
        # -----------------Mudar de estado---------------------
        start = 0
        if stop:
            next_state = idle_s
        elif Seta_Direita or Seta_Esquerda:
            next_state = virar_s
        elif c_i:
            next_state = cones_s
        elif ox_i != -1:
            next_state = obstaculo_s
        elif p_i:
            next_state = parques_s
        else:
            next_state = normal_s

    elif state == virar_s:
        print("Virar")
        if Passadeira_Encontrada:
            Set_state(1)
            if Seta_Direita:
                Direcao = Set_direction(30)
                if datetime.datetime.now() > tempo_passadeira:
                    Passadeira_Encontrada = False
                    Seta_Direita = 0
            elif Seta_Esquerda:
                Direcao = Set_direction(-30)
                if datetime.datetime.now() > tempo_passadeira:
                    Passadeira_Encontrada = False
                    Seta_Esquerda = 0

    elif state == cones_s:
        print("Cones")
        if T:
            next_state = normal_s
        else:
            next_state = cones_s

    elif state == obstaculo_s:
        print("Obstaculo")
        t0 = time.time()
        total = 0
        while total < 5:
            if ox_i > midlle_x and oy_i > midlle_y:
                opening, im_dst = Get_Filtred_Image()
                # ------------NORMAL------------
                contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                # 2- Find Rectangles
                Pontos_Final, n, Pontos_Passadeira, n1, im_dst = Get_Rectangles_Points_Passadeira(contours, im_dst)
                # Pontos_Final , n,im_dst = Get_Rectangles_Points(contours,im_dst)

                # ------------SE REFERENCIA------------
                if n or n1:

                    # ------------SE PASSADEIRA------------
                    if n1 > 4:
                        soma_x = 0
                        count = 0
                        for ponto in Pontos_Passadeira:
                            soma_x += ponto[0]
                            count += 1
                        soma_x /= count
                        Direcao = Calc_direction(soma_x)

                    elif n > 0:
                        melhor_ponto, outro_ponto = Find_closest_point(Pontos_Final, 640, Y_to_find)
                        melhor_ponto[0] = melhor_ponto[0] - 100
                        outro_ponto[0] = outro_ponto[0] - 100
                        angulo_atual = Get_Rect_Angulo(melhor_ponto, outro_ponto)
                        im_dst = cv2.circle(im_dst, (int(melhor_ponto[0]), int(melhor_ponto[1])), 3, (255, 0, 255), 2)
                        print("ANGULO:", angulo_atual)
                        # ------------SAIR DO CRUZAMENTO PARA ESQUERDA------------Tiver uma linha perpendicular
                        if angulo_atual < 40:
                            Direcao = -50  # (angulo_atual-40)*3-25 #-70
                            print("Angulo:", Direcao)

                        # ------------SAIR DO CRUZAMENTO PARA DIREITA------------
                        elif angulo_atual > 135:
                            Direcao = 50  # (angulo_atual-140)*4+25 #50
                            print("Angulo2:", Direcao)

                        # ------------SEGUIR LINHA------------
                        else:
                            # Set_state(2)
                            Direcao = Calc_direction(melhor_ponto[0])

                    Direcao = Direcao * 0.2 + Direcao_ant * 0.8
                # ---------------SE NAO HOUVER REFERENCIA---------------
                else:
                    if Direcao > 0:
                        Direcao += 10
                    else:
                        Direcao -= 10
                Direcao = Set_direction(Direcao)
                Direcao_ant = Direcao
                im_dst = cv2.circle(im_dst, (int(640), int(Y_to_find)), 3, (0, 0, 0), 2)
                cv2.imshow('frame', im_dst)
            else:
                opening, im_dst = Get_Filtred_Image()
                # ------------NORMAL------------
                contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                # 2- Find Rectangles
                pontos_final, n, pontos_passadeira, n1, im_dst = Get_Rectangles_Points_Passadeira(contours, im_dst)
                # Pontos_Final , n,im_dst = Get_Rectangles_Points(contours,im_dst)

                # ------------SE REFERENCIA------------
                if n or n1:

                    # ------------SE PASSADEIRA------------
                    if n1 > 4:
                        soma_x = 0
                        count = 0
                        for ponto in pontos_passadeira:
                            soma_x += ponto[0]
                            count += 1
                        soma_x /= count
                        Direcao = Calc_direction(soma_x)

                    elif n > 0:
                        melhor_ponto, outro_ponto = Find_closest_point(Pontos_Final, 640, Y_to_find)
                        melhor_ponto[0] = melhor_ponto[0] + 100
                        outro_ponto[0] = outro_ponto[0] + 100
                        angulo_atual = Get_Rect_Angulo(melhor_ponto, outro_ponto)
                        im_dst = cv2.circle(im_dst, (int(melhor_ponto[0]), int(melhor_ponto[1])), 3, (255, 0, 255), 2)
                        print("ANGULO:", angulo_atual)
                        # ------------SAIR DO CRUZAMENTO PARA ESQUERDA------------Tiver uma linha perpendicular
                        if angulo_atual < 40:
                            Direcao = -50  # (angulo_atual-40)*3-25 #-70
                            print("Angulo:", Direcao)

                        # ------------SAIR DO CRUZAMENTO PARA DIREITA------------
                        elif angulo_atual > 135:
                            Direcao = 50  # (angulo_atual-140)*4+25 #50
                            print("Angulo2:", Direcao)

                        # ------------SEGUIR LINHA------------
                        else:
                            # Set_state(2)
                            Direcao = Calc_direction(melhor_ponto[0])

                    Direcao = Direcao * 0.2 + Direcao_ant * 0.8
                # ---------------SE NAO HOUVER REFERENCIA---------------
                else:
                    if Direcao > 0:
                        Direcao += 10
                    else:
                        Direcao -= 10
                Direcao = Set_direction(Direcao)
                Direcao_ant = Direcao
                im_dst = cv2.circle(im_dst, (int(640), int(Y_to_find)), 3, (0, 0, 0), 2)
                cv2.imshow('frame', im_dst)
            t1 = time.time()
            total = t1 - t0

        next_state = normal_s

    elif state == parques_s:
        # ------------SE PASSADEIRA------------
        if Passadeira_Encontrada:
            time.sleep(1)
            arduino.write("V.15".encode())
            if not select_park:
                print("Paralelo park")
                direcao = 50
                direcao = Set_direction(direcao)
                time.sleep(1.5)
                if park_cnt > 15:
                    park_cnt = 0
                    total = 0
                    t0 = time.time()
                    while total < 2:
                        t1 = time.time()
                        total = t1 - t0
                        opening, im_dst = Get_Filtred_Image()

                        # --------------------NORMAL--------------------
                        contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                        # 2- Find Rectangles
                        pontos_final, n, im_dst = Get_Rectangles_Points(contours, im_dst)

                        # ----------------SE REFERENCIA-----------------
                        if n:
                            melhor_ponto, outro_ponto = Find_closest_point(pontos_final)
                            melhor_ponto[0] = melhor_ponto[0] + 100
                            outro_ponto[0] = outro_ponto[0] + 100
                            angulo_atual = 0
                            if melhor_ponto[1] > outro_ponto[1]:
                                angulo_atual = math.atan2(melhor_ponto[1] - outro_ponto[1],
                                                          melhor_ponto[0] - outro_ponto[0]) / np.pi * 180
                                print(angulo_atual)
                            else:
                                angulo_atual = math.atan2(outro_ponto[1] - melhor_ponto[1],
                                                          outro_ponto[0] - melhor_ponto[0]) / np.pi * 180
                                print(angulo_atual)

                            # Tiver uma linha perpendicular
                            if angulo_atual < 40:

                                direcao = -70  # (angulo_atual-40)*3-25 #-70
                                print("Angulo:", direcao)
                            elif angulo_atual > 135:
                                direcao = 65  # (angulo_atual-140)*4+25 #50
                                print("Angulo2:", direcao)
                            else:
                                direcao = Calc_direction(melhor_ponto[0])
                            # print(direcao)
                            direcao = direcao * 0.2 + direcao_ant * 0.8
                            im_dst = cv2.circle(im_dst, (int(melhor_ponto[0]), int(melhor_ponto[1])), 3, (255, 0, 0), 2)
                            im_dst = cv2.circle(im_dst, (int(outro_ponto[0]), int(outro_ponto[1])), 3, (0, 255, 255), 2)

                        # -----------------------SE NAO HOUVER REFERENCIA----------------------------
                        else:
                            if direcao > 0:
                                direcao += 5
                            else:
                                direcao -= 1
                        direcao = Set_direction(direcao)
                        direcao_ant = direcao
                        print("Direção:", direcao)

                        im_dst = cv2.circle(im_dst, (int(320), int(Y_to_find)), 3, (0, 0, 0), 2)
                        cv2.imshow('frame', im_dst)

                    next_state = idle_s
                park_cnt += 1
            else:
                print("Park P")
                next_state = idle_s

cap.release()
cv2.destroyAllWindows()
