import cv2
import numpy as np
import math
import serial
import time

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
midlle_x = 320
midlle_y = 100

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
    # 1 - Homografy
    h, status = cv2.findHomography(pts_src, pts_dst)
    im_dst = cv2.warpPerspective(frame, h, (frame.shape[1], frame.shape[0]))
    cv2.imshow('frame4', im_dst)
    # 2- HSV2
    hsv = cv2.cvtColor(im_dst, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    # 3- Filter (Remove noise)
    ret, thresh1 = cv2.threshold(v, 230, 255, cv2.THRESH_BINARY)

    thresh1 = cv2.fillPoly(thresh1, [car_pts], 0)
    opening = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, open_elem)
    cv2.imshow('frame2', opening)
    return opening, im_dst


# MACROS RETANGULOS
Max_Rect_area = 160
Min_Rect_area = 110
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
Y_to_find = 380


# noinspection PyShadowingNames
def Find_closest_point(pontos_final):
    melhor_ponto = pontos_final[0]
    dst_ant = 500
    n = 0
    # print(pontos_final.shape[0])
    for i in range(pontos_final.shape[0] - 1):
        pnt = pontos_final[i + 1]
        dst = math.sqrt((pnt[0] - 320) ** 2 + (pnt[1] - Y_to_find) ** 2)
        if dst_ant > dst and pnt[1] > 200:
            dst_ant = dst
            melhor_ponto = pnt
            n = i + 1

    if n % 2 == 1:
        return melhor_ponto, pontos_final[n - 1]
    else:
        return melhor_ponto, pontos_final[n + 1]


# MACROS
Kp = 0.08


def Calc_direction(error):
    if error - 320 > 0:
        return ((error - 320) * ((error - 320) / 4)) * Kp
    else:
        return -((error - 320) * ((error - 320) / 4)) * Kp


# noinspection PyShadowingNames
def Set_direction(direcao):
    if direcao > 100:
        direcao = 100
    if direcao < -100:
        direcao = -100
    arduino.write(("D." + str(direcao) + "\r").encode())
    return direcao


# Macro
THRESH_PASSADEIRA = 60

passadeira = cv2.imread('passadeira2.jpg')

# ======= Init Camera
if CAMERA_ON:
    cap = cv2.VideoCapture(-1)
else:
    cap = cv2.VideoCapture('recorded2.mp4')

width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(width, height)

cv2.namedWindow('frame')
# Variables Initialization:
# 1- Homografy
camHeight = 480
camWidth = 640
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
    arduino = serial.Serial('/dev/ttyACM0', 115200)
    arduino.close()
    arduino.open()
    while not arduino.isOpen():
        print("Waiting")
    time.sleep(2)
    arduino.write("V.140".encode())

direcao = 0
direcao_ant = 0

while True:
    park_cnt = 0
    state = next_state
    if state != idle_s:
        opening, im_dst = Get_Filtred_Image()

        # --------------------NORMAL--------------------
        contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # 2- Find Rectangles
        pontos_final, n, im_dst = Get_Rectangles_Points(contours, im_dst)

        # ----------------SE REFERENCIA-----------------
        if n:
            melhor_ponto, outro_ponto = Find_closest_point(pontos_final)
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
        cv2.waitKey(20)

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
        elif s_i:
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
        if inline:
            next_state = normal_s
        else:
            next_state = virar_s

    elif state == cones_s:
        print("Cones")
        if T:
            next_state = normal_s
        else:
            next_state = cones_s

    elif state == obstaculo_s:
        print("Obstaculo")
        t0 = time.time()
        while total < 5:
            if ox_i > midlle_x and oy_i > midlle_y:
                opening, im_dst = Get_Filtred_Image()

                # --------------------NORMAL--------------------
                contours, hierarchy = cv2.findContours(opening, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                # 2- Find Rectangles
                pontos_final, n, im_dst = Get_Rectangles_Points(contours, im_dst)

                # ----------------SE REFERENCIA-----------------
                if n:
                    melhor_ponto, outro_ponto = Find_closest_point(pontos_final)
                    melhor_ponto[0] = melhor_ponto[0] - 100
                    outro_ponto[0] = outro_ponto[0] - 100
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
            else:
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
            t1 = time.time()
            total = t1 - t0

        next_state = normal_s

    elif state == parques_s:
        # colocar a verificar quando passa a passadeira!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
