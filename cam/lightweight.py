# Coat and goal

import sensor, image, time, math, lcd
from machine import Timer, PWM, UART
from Maix import GPIO
from fpioa_manager import fm
from board import board_info
from fpioa_manager import fm

tim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
tim2 = Timer(Timer.TIMER0, Timer.CHANNEL1, mode=Timer.MODE_PWM)
tim3 = Timer(Timer.TIMER0, Timer.CHANNEL2, mode=Timer.MODE_PWM)
led_r = PWM (tim, freq=500000, duty=100, pin=13)
led_g = PWM (tim2, freq=500000, duty=100, pin=12)
led_b = PWM (tim3, freq=500000, duty=100, pin=14)

fm.register (board_info.PIN10, fm.fpioa.UART1_TX, force=True)
fm.register(board_info.PIN9, fm.fpioa.UART1_RX, force=True)
uart1 = UART(UART.UART1, 115200,8,0,0, timeout=1000, read_buf_len=4096)

lcd.init(freq=15000000)
lcd.direction(lcd.YX_LRUD)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_contrast(0)#コントラスト
sensor.set_brightness(-2)#明るさ
sensor.set_saturation(+1)#彩
#sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

white = 0
r = 1
g = 2
b = 3

def led_control(a,c):
    if a == white:
        led_r.duty (c)
        led_g.duty (c)
        led_b.duty (c)
    if a == r:
        led_r.duty (c)
    if a == g:
        led_g.duty (c)
    if a == b:
        led_b.duty (c)
        led_g.enable ()
        led_r.enable ()
        led_b.enable ()
threshold_index = 0

thresholds_coat = [(54, 95, -51, 6, -12, 37)]#coat（green）
thresholds_goal = [(15, 84, 6, 27, 16, 49)]#yellow
#thresholds_goal = [(10, 47, 1, 48, -78, -39)]#blue
center_x = 150 #Cam center ｘ coordinate
center_y = 120 #Cam center y coordinate
mirror_center_x = 155 #tmp
mirror_center_y = 120 #tmp
distance_min = 0
distance_max = 125

while(True):
#---setting variable-------------------------------------------------------------------------------
    coat_count = 0
    goal_count = 0

    coat_Sortnumber = 0
    goal_Sortnumber = 0

    areapf=0

    density_max=0#Stores the maximum density of the acquired block
    areaA_max=0#Stores the maximum area of the acquired block
    areaB_max=0#Stores the maximum area of the acquired block
    #areaC_max=0#Stores the maximum area of the acquired block

    cxA=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ボール）の中心のX座標のみを配列にして1つの関数に保存
    cxB=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の中心のX座標のみを配列にして1つの関数に保存
    #cxC=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の中心のX座標のみを配列にして1つの関数に保存

    cyA=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ボール）の中心のY座標のみを配列にして1つの関数に保存
    cyB=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の中心のY座標のみを配列にして1つの関数に保存
    #cyC=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の中心のY座標のみを配列にして1つの関数に保存

    rectA=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ボール）の中心のX座標、Y座標、ブロックの横幅、縦幅の値を配列にして1つの関数に保存
    rectB=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の中心のX座標、Y座標、ブロックの横幅、縦幅の値を配列にして1つの関数に保存
    #rectC=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の中心のX座標、Y座標、ブロックの横幅、縦幅の値を配列にして1つの関数に保存

    object_center_x=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#画面中心のX座標
    object_center_y=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#画面中心のY座標

    r=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#atan2を使って出した角度（単位ラジアン）

    #distanceA=[0,0,0,0,0,0,0,0,0,0,0,0,0,0]#原点からの距離（値の補正はなし）
    #distanceC=[0,0,0,0,0,0,0,0,0,0,0,0,0,0]#原点からの距離（値の補正はなし）

    coat_rads=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#ｒによって出た角度を絶対値に
    goal_rads=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#ｒによって出た角度を絶対値に

    densityA=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#ボールの色の取った面積に対する密度
    densityB=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#ゴールの色の取った面積に対する密度
    #densityC=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#ゴールの色の取った面積に対する密度

    areaA=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ボール）の面積を配列にして1つの関数に保存
    areaB=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の面積を配列にして1つの関数に保存
    #areaC=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#取得したブロック（ゴール）の面積を配列にして1つの関数に保存

    coat_riole=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#ゴールの左右有無の判別用
    goal_riole=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#ゴールの左右有無の判別用

#---------------------------------------------------------------------------------------------------
    clock.tick()
    img = sensor.snapshot()
    img.draw_circle(mirror_center_x, mirror_center_y, 185, color = (0, 0, 0),thickness = 80)
#---Coat sensing-----------------------------------------------------------------------------------
    for blob in img.find_blobs([thresholds_coat[threshold_index]], pixels_threshold=1, area_threshold=1, merge=True, margin=30):
        coat_count+=1
        rectA[coat_count]=blob.rect()
        cxA[coat_count]= int(rectA[coat_count][0] + (rectA[coat_count][2] / 2 ))
        cyA[coat_count]= int(rectA[coat_count][1] + (rectA[coat_count][3] / 2 ))
        #object_center_x[coat_count]=blob.cx()-center_x#Set the center coordinates to (0,0)
        object_center_x[coat_count]= cxA[coat_count] -center_x
        #object_center_y[coat_count]=blob.cy()-center_y
        object_center_y[coat_count]= cyA[coat_count]-center_y
        r[coat_count]=int(math.atan2(-object_center_y[coat_count],-object_center_x[coat_count])*100*0.31)#範囲の調整片側１８０度
        #distanceA[coat_count]=int(math.sqrt((math.pow(object_center_x[coat_count],2))+(math.pow(object_center_y[coat_count],2))))#２点間の距離の公式
        coat_rads[coat_count]=abs(r[coat_count])#atan2で出した値を絶対値に修正
        densityA[coat_count]=blob.density()
        areaA[coat_count]=blob.area()
        count=blob.count()#使ってない
        if object_center_y[coat_count]>0:
            coat_riole[coat_count]=100#light or left
        else:
            coat_riole[coat_count]=0
    areaA_max = (max(areaA[:]))
    #area.append(blob.area())
#---goal sensing-------------------------------------------------------------------------------
    object_center_x=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#画面中心のX座標
    object_center_y=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#画面中心のY座標
    r=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]#atan2を使って出した角度（単位ラジアン）
    for blob in img.find_blobs([thresholds_goal[threshold_index]], pixels_threshold=1, area_threshold=1, merge=True,margin=30):
        goal_count+=1
        cxB[goal_count]=blob.cx()
        cyB[goal_count]=blob.cy()
        rectB[goal_count]=blob.rect()
        object_center_x[goal_count]=blob.cx()-center_x#Set the center coordinates to (0,0)
        object_center_y[goal_count]=blob.cy()-center_y
        r[goal_count]=int(math.atan2(-object_center_y[goal_count],-object_center_x[goal_count])*100*0.31)#範囲の調整片側１８０
        goal_rads[goal_count]=abs(r[goal_count])
        densityB[goal_count]=blob.density()
        areaB[goal_count]=blob.area()
        if object_center_y[goal_count]>0:
            goal_riole[goal_count]=100#light or left
        else:
            goal_riole[goal_count]=0
        led_control(g,100)
    led_control(g,0)
    areaB_max = (max(areaB[:]))
#---Coat sort--------------------------------------------------------------------------------------
    for coat_Sortnumber in range(1,16):
        if areaA[coat_Sortnumber]==areaA_max!=0:
            break
        elif coat_Sortnumber == 15:
            coat_Sortnumber = 0
            break
#---goal sort--------------------------------------------------------------------------------------
    for goal_Sortnumber in range(1,16):
        if areaB[goal_Sortnumber]== areaB_max!=0:
            break
        elif goal_Sortnumber == 15:
            goal_Sortnumber = 0
            break
#---draw shape & variable Deformation--------------------------------------------------------------
    if coat_Sortnumber != 0:
        img.draw_rectangle(rectA[coat_Sortnumber])#長方形の生成
        img.draw_cross(cxA[coat_Sortnumber], cyA[coat_Sortnumber]) #中央のバツの生成
        if coat_rads[coat_Sortnumber] <= 6.0625:
            angle_coat = 1
        else:
            for angle_coat in range(2,16):
                if coat_rads[coat_Sortnumber] <= (angle_coat*12.125 - 6.0625):
                    if coat_riole[coat_Sortnumber] == 100:
                        angle_coat = angle_coat
                        break
                    else:
                        angle_coat = 18 - angle_coat
                        break
    else:
        angle_coat = 0

    if goal_Sortnumber != 0:
        img.draw_rectangle(rectB[goal_Sortnumber])#長方形の生成
        img.draw_cross(cxB[goal_Sortnumber], cyB[goal_Sortnumber]) #中央のバツの生成
        if goal_rads[goal_Sortnumber] <= 6.0625:
            angle_goal = 1
        else:
            for angle_goal in range(2,16):
                if goal_rads[goal_Sortnumber] <= (angle_goal*12.125 - 6.0625):
                    if goal_riole[goal_Sortnumber] == 100:
                        angle_goal = angle_goal
                        break
                    else:
                        angle_goal = 18 - angle_goal
                        break
    else:
        angle_goal = 0
#---variable Deformation---------------------------------------------------------------------------
#---Output sensor numeric--------------------------------------------------------------------------
    #print(angle_coat)
    led_control(r,20)
    uart1.write(bytearray([int(angle_coat)]))
    uart1.write(bytearray([int((angle_goal) + 17)]))
    img.draw_line(0,center_y,320,center_y)#横線108
    img.draw_line(center_x,0,center_x,240)#縦線170
    img.draw_circle(center_x,center_y,distance_min)
    #lcd.display(img)
