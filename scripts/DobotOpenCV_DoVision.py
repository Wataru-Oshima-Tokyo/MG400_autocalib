
# coding: utf-8

# Created on Tue Feb 21 12:29 2017<br>
# Updated on Mon Feb 27 17:56 2017<br>
# Updated on Thu Sep 06 05:58 2018<br>
# @author: Kennosuke Wada

# In[433]:


import numpy as np
import cv2
import sys, threading, time, math
import DobotDllType as dType


# In[434]:


workSignal = True
CIRCLE_PARAMS = []


# In[435]:


def PeriodicTask():
    if workSignal:
        dType.PeriodicTask(api)
        threading.Timer(0.05, PeriodicTask).start()


# In[436]:


def GetPoseTask():
    if workSignal:
        global gPose
        gPose = dType.GetPose(api)
        # [x, y, z, rHead, joint1Angle, joint2Angle, joint3Angle, joint4Angle]
        threading.Timer(0.2, GetPoseTask).start()


# In[437]:


def getCircle(target_num, hsv):
    hsv_min = np.array(CIRCLE_PARAMS[target_num]["MIN"])
    hsv_max = np.array(CIRCLE_PARAMS[target_num]["MAX"])
    
    # マスク画像を用いて元画像から指定した色を抽出
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
    if target_num == 0:
        cv2.imshow("Arm", mask)
    else:
        cv2.imshow("Target", mask)

    '''
    area = cv2.countNonZero(mask)
    print(label, ": ", area)
    radius = math.sqrt(area / math.pi)
    print("radius: ", radius)
    '''
    
    hough = CIRCLE_PARAMS[target_num]["HOUGH"]
    
    # ハフ変換を用いて，グレースケール画像から円を検出する。
    # 検出された円はベクトル形式で出力される。
    # 各ベクトルは，3要素の浮動小数点型ベクトル  [x, y, radius] としてエンコードされる。
    circles = cv2.HoughCircles(
        mask,                   # 8ビット，シングルチャンネル，グレースケールの入力画像
        cv2.HOUGH_GRADIENT,     # 現在のところ， CV_HOUGH_GRADIENT メソッドのみが実装されている。
        1,                      # dp - 画像分解能に対する投票分解能の比率の逆数
                                # 例えば， dp=1 の場合は，投票空間は入力画像と同じ分解能を持つ。．
                                # また dp=2 の場合は，投票空間の幅と高さは半分になる。
        150,                    # minDist ? 検出される円の中心同士の最小距離．
                                # このパラメータが小さすぎると，正しい円の周辺に別の円が
                                # 複数誤って検出されることになる。
                                # 逆に大きすぎると，検出できない円がでてくる可能性がある。
        param1=hough[0],        # 手法依存の 1 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
                                # Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値を表す。
                                # （小さい閾値は，この値の半分になる。）．
        param2=hough[1],        # 手法依存の 2 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
                                # 円の中心を検出する際の投票数の閾値を表す。これが小さくなるほど，
                                # より多くの誤検出が起こる可能性がある。
                                # より多くの投票を獲得した円が，最初に出力される。
        minRadius=hough[2],     # 円の半径の最小値
        maxRadius=hough[3]      # 円の半径の最大値
    )
    
    try:
        cnt = len(circles[0])
    except:
        return None
    
    global AVERAGE_LIST
    AVERAGE_LIST = []
    for i in range(len(CIRCLE_PARAMS)):
        AVERAGE_LIST.append(np.empty((0, 3), int))

    circles = np.uint16(np.around(circles))
    for circle in circles[0, :]:
        x, y, r = circle
        
        if len(AVERAGE_LIST[target_num]) < AVERAGE_COUNT:
            AVERAGE_LIST[target_num] = np.append(AVERAGE_LIST[target_num], np.array([[x, y, r]]), axis=0)
        else:
            AVERAGE_LIST[target_num] = np.append(AVERAGE_LIST[target_num], np.array([[x, y, r]]), axis=0)
            AVERAGE_LIST[target_num] = np.delete(AVERAGE_LIST[target_num], 0, axis=0)
    
    aveArray = np.mean(AVERAGE_LIST[target_num], axis=0)
    
    return aveArray


# In[438]:


def drawMark(frame, x, y, radius):
    # 囲み線を描く
    cv2.circle(frame, (x, y), radius, (0, 0, 255), 2)
    
    # 中心点を描く
    cv2.circle(frame, (x, y),      3, (255, 0, 0), 3)


# In[439]:


def fineTuning(x, y, TUNING_SCALE_X, TUNING_SCALE_Y):
    print("\nFine Tuning...")
    print("\tPOSE: ", gPose[0], gPose[1], gPose[2], gPose[3])
    
    tx = x - SCREEN_WIDTH  / 2
    ty = y - SCREEN_HEIGHT / 2
    
    vec_x = TUNING_SCALE_X * tx
    vec_y = TUNING_SCALE_Y * ty
    
    # カメラ画像の座標系 (x, y, z) とアームの座標系 (X, Y, Z) との関係は、
    #  (X, Y, Z) = (-y, -x, z)
    dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZINCMode, -vec_y, -vec_x, 0, 0, True)
    
    while gPose[2] > BOTTOM_Z + OBJ_HEIGHT:
        dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZINCMode, 0, 0, -TUNING_DZ, 0, True)
    
    dType.SetEndEffectorSuctionCup(api, True, True, True)  # 吸盤 ON
    time.sleep(1)
    
    # 最後の isQueued = True が重要、False だと吸盤がすぐに OFF になる
    dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZINCMode, 0,  0, 60.0, 0, True)
    time.sleep(1)
    
    # 絶対座標での落とす場所
    dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJXYZMode, 100.0, 170.0, 20.0, 0.0, True)
    
    dType.SetEndEffectorSuctionCup(api, False, False, True)  # 吸盤 OFF

    dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJXYZMode, 200.0, 0.0, -20.0, 0.0, True)

    print("Task Completed")


# In[440]:


def setColorRange(index, x, y):
    print("\tPOS: ", x, y)
    
    ret, img = cap.read()
    bgr = img[y:y + 1, x:x + 1]
    bgr00 = bgr[0][0]
    
    print("\tRGB: ", bgr00)
    
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV_FULL)
    hsv00 = hsv[0][0]
    print("\tHSV: ", hsv00)
    
    #################################################################
    #  HSV 空間における、指定色から最小値と最大値                   #
    #################################################################
    MIN_DH, MIN_DS, MIN_DV = [15,  60, 60]
    MAX_DH, MAX_DS, MAX_DV = [15, 150, 60]
    #################################################################

    global CIRCLE_PARAMS
    
    CIRCLE_PARAMS[index]["MIN"][0] = max([0, hsv00[0] - MIN_DH])
    CIRCLE_PARAMS[index]["MIN"][1] = max([0, hsv00[1] - MIN_DS])
    CIRCLE_PARAMS[index]["MIN"][2] = max([0, hsv00[2] - MIN_DV])
    print("\tMIN: ", np.array(CIRCLE_PARAMS[index]["MIN"]))
    
    CIRCLE_PARAMS[index]["MAX"][0] = min([255, hsv00[0] + MAX_DH])
    CIRCLE_PARAMS[index]["MAX"][1] = min([255, hsv00[1] + MAX_DS])
    CIRCLE_PARAMS[index]["MAX"][2] = min([255, hsv00[2] + MAX_DV])
    print("\tMAX: ", np.array(CIRCLE_PARAMS[index]["MAX"]))


# In[441]:


def setDobotParams():
    PARAM_1 = 200  # Velocity           (Default: 200)
    PARAM_2 = 200  # Acceleration       (Default: 200)
    
    dType.SetJOGJointParams(api,
                            PARAM_1, PARAM_1, PARAM_1, PARAM_1,
                            PARAM_2, PARAM_2, PARAM_2, PARAM_2)
    
    PARAM_3 = 200  # Velocity           (Default: 200)
    PARAM_4 = 200  # Acceleration       (Default: 200)
    
    dType.SetJOGCoordinateParams(api,
                            PARAM_3, PARAM_3, PARAM_3, PARAM_3,
                            PARAM_4, PARAM_4, PARAM_4, PARAM_4)
    
    PARAM_5 = 100  # Velocity Ratio     (Default: 100)
    PARAM_6 = 100  # Acceleration Ratio (Default: 100)
    
    dType.SetJOGCommonParams(api,
                            PARAM_5, PARAM_6)
    
    # MOVJ
    PARAM_7 = 200  # Velocity           (Default: 200)
    PARAM_8 = 200  # Acceleration       (Default: 200)
    
    dType.SetPTPJointParams(api,
                            PARAM_7, PARAM_7, PARAM_7, PARAM_7,
                            PARAM_8, PARAM_8, PARAM_8, PARAM_8)
    
    # MOVL
    PARAM_9  = 200 # xyz Velocity       (Default: 100)
    PARAM_10 = 200 # xyz Acceleration   (Default: 100)
    PARAM_11 = 200 # r Velocity         (Default: 100)
    PARAM_12 = 200 # r Acceleration     (Default: 100)
    
    dType.SetPTPCoordinateParams(api,
                            PARAM_9, PARAM_10, PARAM_11, PARAM_12)
    # JUMP
    PARAM_13 = 20  # Jump Height        (Default:  20)
    PARAM_14 = 100 # Max Jump Height    (Default: 100)
    
    dType.SetPTPJumpParams(api,
                            PARAM_13, PARAM_14)
    
    PARAM_15 = 50  # Velocity Ratio     (Default:  30)
    PARAM_16 = 50  # Acceleration Ratio (Default:  30)
    
    dType.SetPTPCommonParams(api,
                            PARAM_15, PARAM_16)


# In[442]:


def setOpenCVParams():
    #################################################################
    #  カメラの高さ = 500 mm  [台からカメラレンズの下側まで]        #
    #################################################################
    HOUGH_1      = 30  # 手法依存の 1 番目のパラメータ．: CV_HOUGH_GRADIENT の場合は，
                       # Canny() エッジ検出器に渡される2つの閾値の内，大きい方の閾値を表す
    HOUGH_2      = 10  # 円の中心を検出する際の投票数の閾値を表す。これが小さくなるほど，
                       # より多くの誤検出が起こる可能性がある。
                       # より多くの投票を獲得した円が，最初に出力される。
    
    ARM_SIZE_MIN = 12   # アームの円形のマークの最小半径  (5)
    ARM_SIZE_MAX = 25   # アームの円形のマークの最大半径  (20)
    
    OBJ_SIZE_MIN = 30  # 円形のピッキング・オブジェクトの最小半径  (20)
    OBJ_SIZE_MAX = 42  # 円形のピッキング・オブジェクトの最大半径  (50)
    #################################################################

    global CIRCLE_PARAMS
    CIRCLE_PARAMS = []
    
    CIRCLE_PARAMS.append({  # Arm
        "HOUGH": [HOUGH_1, HOUGH_2, ARM_SIZE_MIN, ARM_SIZE_MAX],
        "MIN": [0, 0, 0],
        "MAX": [0, 0, 0]})
    
    CIRCLE_PARAMS.append({  # Obj
        "HOUGH": [HOUGH_1, HOUGH_2, OBJ_SIZE_MIN, OBJ_SIZE_MAX],
        "MIN": [0, 0, 0],
        "MAX": [0, 0, 0]})


# In[443]:


def mouseEvent(event, x, y, flags, param):
    global isSetArm, isSetObj
    
    if event == cv2.EVENT_LBUTTONUP:
        print("\nL-Button: ARM")
        setColorRange(0, x, y)
        isSetArm = True
    
    if event == cv2.EVENT_RBUTTONUP:
        print("\nR-Button: OBJ")
        setColorRange(1, x, y)
        isSetObj = True
    
    if event == cv2.EVENT_MBUTTONUP:
        isSetArm = False
        isSetObj = False
        dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJXYZMode, 200.0, 0.0, -10.0, 0.0, True)


# In[444]:


threading.Timer(0.1, PeriodicTask).start()
threading.Timer(0.5, GetPoseTask ).start()

api = dType.load()
result = dType.ConnectDobot(api, "", 115200)
errorString = ['Success', 'NotFound', 'Occupied']
print("Connect status:", errorString[result[0]])

dType.SetCmdTimeout(api, 3000)

# WEBカメラのデバイス番号 (Surface Pro 3 の場合は 2)
VIDEO_DEVICE_NO = 0

cap = cv2.VideoCapture(VIDEO_DEVICE_NO)
ret, frame = cap.read()

# 画像のサイズ情報に注意！ (HEIGHT, WIDTH) の順で格納されている。
global SCREEN_WIDTH, SCREEN_HEIGHT
SCREEN_HEIGHT, SCREEN_WIDTH = frame.shape[:2]

setOpenCVParams()

global AVERAGE_COUNT
AVERAGE_COUNT = 1

arm_x = arm_y = arm_r = 0.0  # アームに付けた円形のマークの座標
obj_x = obj_y = obj_r = 0.0  # ピッキングする円形のオブジェクトの座標

global isSetArm, isSetObj
isSetArm = False             # アームの色をピッキングしたら True
isSetObj = False             # ピッキング・オブジェクトの色をピッキングしたら True

setDobotParams()

#################################################################
#  カメラの高さ = 530 mm  [台からカメラレンズの下側まで]        #
#################################################################
FIND_THRESHOLD  = 20.0    # アームとピッキング・オブジェクトの距離から
                          # 近さを判定するための閾値  (20.0)

MOVING_MAX_0    = 5.0     # 近いときの、１回での最大の移動量  (5.0)
MOVING_COEFF_0  = 0.02    # 近いときの、カメラ座標でのズレを、
                          # アームの移動量に換算するためのスケール  (0.02)

MOVING_MAX_1    = 50.0    # 遠いときの、１回での最大の移動量  (50.0)
MOVING_COEFF_1  = 0.20    # 遠いときの、カメラ座標でのズレを、
                          # アームの移動量に換算するためのスケール  (0.20)

TUNING_SCALE_X  = 0.25    # 視差により位置ずれの修正を行うためのＸ軸方向のスケール  (0.21)
TUNING_SCALE_Y  = 0.28    # 視差により位置ずれの修正を行うためのＹ軸方向のスケール  (0.21)

TUNING_DZ       = 16.0     # 十分近いづいた後に、アームを毎回下げる距離        (8.0)
BOTTOM_Z        = -70.0   # アームの台座のＺ座標（ホームポジションが基準）   (-48.0)
OBJ_HEIGHT      = 1.0    # ピッキングするオブジェクトの高さ（全て均一）     (10.0)
#################################################################

dType.SetHOMECmdEx(api, 0, True)
dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJXYZMode, 200.0, 0.0, -10.0, 0.0, True)
print("Start Pos = ", gPose)

while True:
    # 作業スペースの画像をキャプチャ
    ret, frame = cap.read()
    
    # マウスによるイベントハンドラーを設定
    cv2.setMouseCallback("Dobot", mouseEvent)
    
    # 認識の精度を上げるために画像を平滑化
    blur = cv2.GaussianBlur(frame, (33, 33), 1)
    
    # RGB色空間の画像を、HSV色空間に変換
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV_FULL)
    
    # マニピュレータのアームに貼り付けた円形マークの座標を検出
    result_arm = getCircle(0, hsv)
    if isinstance(result_arm, np.ndarray):
        arm_x, arm_y, arm_r = result_arm
    
    # 円形の形状をしたピッキング・オブジェクトの座標を検出
    result_obj = getCircle(1, hsv)
    if isinstance(result_obj, np.ndarray):
        obj_x, obj_y, obj_r = result_obj
    
    if isSetArm:
        x, y, r = np.int16(np.around([arm_x, arm_y, arm_r]))
        drawMark(frame, x, y, r)
    else:
        cv2.putText(frame, "L-Click : Mark of Arm",
                    (20, 450), cv2.FONT_HERSHEY_PLAIN, 1.5, (0,255,255), 2)
    
    if isSetObj:
        x, y, r = np.int16(np.around([obj_x, obj_y, obj_r]))
        drawMark(frame, x, y, r)
    else:
        cv2.putText(frame, "R-Click : Mark of Target",
                    (20, 470), cv2.FONT_HERSHEY_PLAIN, 1.5, (0,255,255), 2)
    
    if isSetArm and isSetObj:
        cv2.putText(frame, "M-Click : Reset All Marks",
                    (20, 470), cv2.FONT_HERSHEY_PLAIN, 1.5, (0,255,255), 2)
    else:
        # 中央に点を描画する
        cv2.circle(frame, (SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2), 3, (255, 255, 0), 2)

    cv2.putText(frame, "Esc Key -> Exit",
                (490, 20), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 0, 255), 1)
    
    # キャプチャした画像にマークや使用法などを記述して描画
    cv2.imshow("Dobot", frame)
    
    # アームとピッキング・オブジェクトの両方が検出された場合
    if isSetArm == True and isSetObj == True:
        diff_x = arm_x - obj_x
        diff_y = arm_y - obj_y
        
        dist2 = diff_x * diff_x + diff_y * diff_y
        
        # アームがピッキング・オブジェクトに十分重なって見えた場合
        if dist2 < FIND_THRESHOLD:
            # 視差により位置ずれの修正とピッキング
            fineTuning(arm_x, arm_y, TUNING_SCALE_X, TUNING_SCALE_Y)
            
            # フラグを初期化
            isSetArm = False
            isSetObj = False
        else:
            # ある程度、近くまで移動できた場合
            if dist2 < FIND_THRESHOLD * 5:
                # 視差による位置ずれより、移動量を計算
                mx = np.clip(diff_x * MOVING_COEFF_0, -MOVING_MAX_0, MOVING_MAX_0)
                my = np.clip(diff_y * MOVING_COEFF_0, -MOVING_MAX_0, MOVING_MAX_0)
                
                # MOVL：Linear Movements [isQueued = True, 同期モード]
                dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZINCMode, my, mx, 0, 0, True)
            # アームがピッキング・オブジェクトから遠い場合
            else:
                # 視差による位置ずれより、移動量を計算
                mx = np.clip(diff_x * MOVING_COEFF_1, -MOVING_MAX_1, MOVING_MAX_1)
                my = np.clip(diff_y * MOVING_COEFF_1, -MOVING_MAX_1, MOVING_MAX_1)
                
                # MOVJ：Joint Movements  [isQueued = False, 非同期モード]
                dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVJXYZINCMode, my, mx, 0, 0, False)
                
    if cv2.waitKey(1) & 0xFF == 27: # Esc キー
            break

workSignal = False
dType.DisconnectDobot(api)

cap.release()
cv2.destroyAllWindows()

print("\n*** Exit ***")


# -----------------------------------------------------------
# 開発環境： <br>
#     Anaconda3 [Windows 64bit版] <br>
# 		https://www.anaconda.com/download/ <br>
#         Python 3.6 version  [Python 3.6.6] <br>
# 		64-Bit Graphical Installer <br>
#         Anaconda3-5.2.0-Windows-x86_64.exe <br>
# 		C:\Anacoda3 にインストール <br>
# 
# Jupyter Notebook: <br>
#     StartJupyer.bat のような名前のバッチファイルを作成し、以下のように記述する。<br>
#     
#     %echo off
#     jupyter notebook --notebook-dir="."
# 
# -----------------------------------------------------------
# 必要なモジュール： <br>
# 
#     setuptools      40.0.0 <br>
#     numpy           1.15.0 <br>
#     opencv-python   3.4.2 <br>
#     
# 上記のパッケージがインストールされていない場合は、
# 
#     $ pip install opencv-python
# 
# パッケージをアップデートしたい場合は、
# 
#     $ pip install -U opencv-python
# 
# アンインストールしたい場合は、
# 
#     $ pip uninstall opencv-python
# 
# 特定のパッケージのバージョンを確認したい場合、
# 
#     $ pip list | find "opencv-python"
# 
# -----------------------------------------------------------
# 必要な DLL: <br>
#     http://dobot.cc/download-center/dobot-magician.html <br>
# 	DobotDemoV2.0\DobotDemoForPython\DobotDemoForPython <br>
# 		msvcp120.dll <br>
# 		msvcr120.dll <br>
# 		Qt5Core.dll <br>
# 		Qt5Network.dll <br>
# 		Qt5SerialPort.dll <br>
# 		
# 	【注意】
# 		このフォルダ内の <br>
# 			DobotDll.dll <br>
# 		を使うと、実行時に <br>
# 			dType.PeriodicTask(api) <br>
# 		の箇所でエラーが発生するので、このファイルは使わず、 <br>
# 		以前のバージョンの dll ファイルを使うこと。 <br>
# 			DobotDll.dll <br>
# 				Filesize: <br>
# 					88,064 バイト <br>
# 				SHA-1: <br>
# 					D1E961B6866A2F4D91A13CFAF7023BA124D41359 <br>
# 
# -----------------------------------------------------------
# 必要なソースコード： <br>
#     http://dobot.cc/download-center/dobot-magician.html <br>
# 	DobotDemo\2.0\DobotDemoForPython\DobotDemoForPython <br>
#     のフォルダ内の DLL ラッパーのソースコード <br>
#         DobotDllType.py <br>
#     1行目に <br>
#         # -*- coding: utf-8 -*- <br>
#     を追加してから、途中の中国語のコメントを全て削除する必要がある。 <br>
# 
# 以上の DLL とソースコードを、このコードと同じフォルダに配置し、 <br>
# File -> Import Settings... で、これらの DLL とソースコードを <br>
# このコードが参照できるように設定する。

# ■  isQueued = True
#     
#     XYZ 相対移動
#         dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZINCMode , x, y, z, 0, True )
#     XYZ 絶対移動
#         dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode    , x, y, z, 0, True )
#     吸盤 ON
#         dType.SetEndEffectorSuctionCupEx(api, 1, True)
#     吸盤 OFF
#         dType.SetEndEffectorSuctionCupEx(api, 0, True)
# 
# ■  isQueued = Fales
#     
#     XYZ 相対移動
#         dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZINCMode , x, y, z, 0, False)
#     XYZ 絶対移動
#         dType.SetPTPCmdEx(api, dType.PTPMode.PTPMOVLXYZMode    , x, y, z, 0, False)
#     吸盤 ON
#         dType.SetEndEffectorSuctionCupEx(api, 1, False)
#     吸盤 OFF
#         dType.SetEndEffectorSuctionCupEx(api, 0, False)
# 
# ■  Playback Mode
#     
#     JUMP：
#         from point A to B, the trajectory is shown below,
#         the end effector will lift upwards by amount of Height (in mm)
#         and move horizontally to a point that is above B by Height
#         and then move down to Point B.
#         Value of Height can be configured in the Playback tab of
#         the Config Dobot Module, of which the default value is 20 mm.
#         Click send to configure Dobot after changing the value.
#     
#     MOVJ：Joint Movements
#         From point A to B, each joint will run from initial angle its target angle,
#         regardless of the trajectory.
#         The motion time for all joints are the same which means will start and finish at time.
#     
#     MOVL：Linear Movements
#         The joints will cooperate in order to perform a line trajectory from A to pointB.
# 
# ■  PTPMode
# 
#     PTPJUMPXYZMode      = 0
#     PTPMOVJXYZMode      = 1
#     PTPMOVLXYZMode      = 2
# 
#     PTPJUMPANGLEMode    = 3
#     PTPMOVJANGLEMode    = 4
#     PTPMOVLANGLEMode    = 5
#     PTPMOVJANGLEINCMode = 6
# 
#     PTPMOVLXYZINCMode   = 7
#     PTPMOVJXYZINCMode   = 8
#     PTPJUMPMOVLXYZMode  = 9
# 
# 参考：
#     http://dobot.cc/download-center/dobot-magician.html
#     http://dobot.cc/upload/ue_upload/files/2016-05-27/Dobot%20API%20Reference%20and%20Developing%20Examples_en.pdf
#     https://syncer.jp/color-converter
