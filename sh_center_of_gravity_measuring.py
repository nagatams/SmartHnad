#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# スマートハンドによる重心位置計測用プログラム 

import rospy
import copy
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import WrenchStamped
from moveit_commander import MoveGroupCommander
from time import sleep
import numpy as np

# 爪が開ききった/閉じきったことを判定するためのしきい値
# 適宜調整すること
OPENING_WIDTH_UNDER_LIMIT = 150.0
OPENING_WIDTH_OVER_LIMIT = 100.0

# ワーク把持前の力・トルクを格納する2次元配列
sensor_data_init = np.zeros((2,6))

class SmartHnadControl():
    def __init__(self):
        # 'MoveGroupCommander'オブジェクトのインスタンス化
        self.group = MoveGroupCommander("manipulator")
        self.exec_vel = 0.01

        # Publisherを作成
        self.pub = rospy.Publisher('hand_move_control', String, queue_size=10)
        # Subscriberを作成
        #self.sub = rospy.Subscriber('force', WrenchStamped, self.callback)

    # def callback(self, data):
        # callback時の処理
        # self.publish(data)

    def arm_homepotision(self):
        # 目標位置・姿勢の設定
        # 設定の初期化
        waypoints = []

        # 現在の手先位置・姿勢を取得
        group = self.group
        wpose = group.get_current_pose().pose
        print("the potision before homeing :")
        print(wpose)

        ### ******************** 【重要】教示点を測定して設定すること ********************
        # 目標位置・姿勢を設定　
        wpose.position.x = -0.0712933839629
        wpose.position.y = 0.386411284241
        wpose.position.z = 0.568792785331
        wpose.orientation.x = -0.713524543506
        wpose.orientation.y = -0.700622833553
        wpose.orientation.z = 0.00094226678374
        wpose.orientation.w = 0.00307945630185
        waypoints.append(copy.deepcopy(wpose))

        # 指定されたポーズ列（waypoint）を線形補完した動作をさせるための動作計画を実施
        # 第2引数：動作を計算する間隔[m] 
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)

        plan = group.retime_trajectory(group.get_current_state(), plan, 0.01)

        rospy.loginfo("home position start")
        group.execute(plan)
        rospy.loginfo("home position end")

    def publish_hand_state(self, state):
        # stateの値が　1:ハンドopen 0:ハンドclose と対応
        self.pub.publish(state)

        # 爪が動いている間待機
        sleep(2)

        # 爪の開き幅取得
        opening_width = rospy.wait_for_message('opening_width', Float32, timeout=None)
        print("hand opening-width is %f" % (opening_width.data))

        return opening_width.data

    def arm_move(self, x, y, z):
        # 目標座標と姿勢の設定開始
        # 目標座標と姿勢の初期化
        waypoints = [] 

        # 目標座標へ移動開始前の手先位置姿勢を取得
        group = self.group
        wpose = group.get_current_pose().pose 

        #pose
        #   position
        #        x
        #        y
        #        z
        #    orientation（クォータニオン）
        #        x
        #        y
        #        z
        #        w

        # orientation(x, y, z, w)について
        #
        # 大きさ1の回転軸ベクトルを(x1, y1, z1)とし、回転角度をaとすると、
        # x = x1 * sin(a/2)
        # y = y1 * sin(a/2)
        # z = z1 * sin(a/2)
        # w = cos(a/2)

        # 目標座標をwposeに格納
        wpose.position.x = x
        wpose.position.y = y
        wpose.position.z = z

        # waypointsに目標座標と姿勢をセット
        waypoints.append(copy.deepcopy(wpose)) 

        # 目的座標と姿勢, 隣り合う中継地点までの距離, ...
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0) 

        # ここでcartesian_path軌道を移動中の速度を指定する
        plan = group.retime_trajectory(group.get_current_state(), plan, 0.03) 

        rospy.loginfo("[ arm_move ]: pose start")
        # cartesian_pathで生成した軌道で動作実行
        group.execute(plan) 
        rospy.loginfo("[ arm_move ]: pose end")
        # 目標座標と姿勢への移動終了

    def arm_orientation(self, x, y, z, w):
        # 目標座標と姿勢の設定開始
        # 目標座標と姿勢の初期化
        waypoints = [] 

        # 目標座標へ移動開始前の手先位置姿勢を取得
        group = self.group
        wpose = group.get_current_pose().pose 

        #pose
        #   position
        #        x
        #        y
        #        z
        #    orientation（クォータニオン）
        #        x
        #        y
        #        z
        #        w

        # orientation(x, y, z, w)について
        #
        # 大きさ1の回転軸ベクトルを(x1, y1, z1)とし、回転角度をaとすると、
        # x = x1 * sin(a/2)
        # y = y1 * sin(a/2)
        # z = z1 * sin(a/2)
        # w = cos(a/2)

        # 目標座標をwposeに格納
        wpose.orientation.x = x
        wpose.orientation.y = y
        wpose.orientation.z = z
        wpose.orientation.w = w

        # waypointsに目標座標と姿勢をセット
        waypoints.append(copy.deepcopy(wpose)) 

        # 目的座標と姿勢, 隣り合う中継地点までの距離, ...
        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0) 

        # ここでcartesian_path軌道を移動中の速度を指定する
        plan = group.retime_trajectory(group.get_current_state(), plan, 0.03) 

        rospy.loginfo("[ arm_orientation ]: pose start")
        # cartesian_pathで生成した軌道で動作実行
        group.execute(plan) 
        rospy.loginfo("[ arm_orientation ]: pose end")
        # 目標座標と姿勢への移動終了

    def set_init_sensor_data(self, i):
        num = 10 # 平滑化のサンプリング数
        sensor_data_raw = np.zeros(6) # 平滑化の計算用配列, [0. 0. 0. 0. 0. 0]
        global sensor_data_init

        # num回分データを測定し，平均値を求めて記録用の配列に格納しておく(numHz)
        for j in range(num):
            data = rospy.wait_for_message('force', WrenchStamped, timeout=None)
            append_list = np.array([[data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]])
            sensor_data_raw = sensor_data_raw + append_list
            #print(sensor_data_raw)
            sleep(1 / num)
        #print("sensor_data_raw:")
        #print(sensor_data_raw)
        
        sensor_data_init[i] = sensor_data_raw / num
        print("sensor_data_init:")
        print(sensor_data_init)
        #print(sensor_data_init[0])

    def center_of_gravity_measurement(self, i):
        num = 10 # 平滑化のサンプリング数
        sensor_data_difference = np.zeros(6) # [0. 0. 0. 0. 0. 0]
        init_list = sensor_data_init[i]
        print("list: {}".format(init_list))

        # num回分データを測定し，把持前の値との差の平均を求める(numHz)
        for i in range(num):
            data = rospy.wait_for_message('force', WrenchStamped, timeout=None)
            append_list = np.array([[data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]])
            sensor_data_difference = sensor_data_difference + (append_list - init_list)
            #print(sensor_data_difference)
            sleep(0.1)

        print("sensor_data_difference:")
        print(sensor_data_difference)
        
        sensor_data_difference = sensor_data_difference / num
        print("sensor_data_difference_mean:")
        print(sensor_data_difference)

        return sensor_data_difference

    def calc_center_of_gravity(self, list1, list2):
        a1 = (list1[0][0]**2/list1[0][2]**2) + (list1[0][1]**2/list1[0][2]**2) + 1
        a2 = (list1[0][0]*list2[0][0])/(list1[0][2]*list2[0][2]) + (list1[0][1]*list2[0][1])/(list1[0][2]*list2[0][2]) + 1
        a3 = (-(list1[0][4]/list1[0][2])+(list2[0][4]/list2[0][2]))*(list1[0][0]/list1[0][2]) + ((list1[0][3]/list1[0][2])-(list2[0][3]/list2[0][2]))*(list1[0][1]/list1[0][2]) 
        b1 = a2
        b2 = (list2[0][0]**2/list2[0][2]**2) + (list2[0][1]**2/list2[0][2]**2) + 1
        b3 = (-(list2[0][4]/list2[0][2])+(list1[0][4]/list1[0][2]))*(list2[0][0]/list2[0][2]) + ((list2[0][3]/list2[0][2])-(list1[0][3]/list1[0][2]))*(list2[0][1]/list2[0][2]) 

        s = (a2*b3+a3*b2)/(a2*b1-a1*b2)
        t = (a1*b3+a3*b1)/(a2*b1-a1*b2)

        # 姿勢1で求めた直線における，姿勢2で求めた直線への最接近位置
        x1 = -list1[0][4]/list1[0][2] + s*(list1[0][0]/list1[0][2])
        y1 = list1[0][3]/list1[0][2] + s*(list1[0][1]/list1[0][2])
        z1 = s
        point1 = np.array([x1, y1, z1])

        # 姿勢2で求めた直線における，姿勢1で求めた直線への最接近位置
        x2 = -list2[0][4]/list2[0][2] + t*(list2[0][0]/list2[0][2])
        y2 = list2[0][3]/list2[0][2] + t*(list2[0][1]/list2[0][2])
        z2 = t
        point2 = np.array([x2, y2, z2])

        center_of_gravity = (point1 + point2) / 2

        print("")
        print("[ calc_center_of_gravity ] calcuration result: ")
        print("[ calc_center_of_gravity ] point1: {}, point2: {}".format(point1, point2))
        print("[ calc_center_of_gravity ] center_of_gravity: {}".format(center_of_gravity))
        
        return center_of_gravity

def main():
    try:
        ## 一連の処理
        print("")
        print("----------------------------------------------------")
        print("---------- Smart-Hand Measurement Demonstration ----------")
        print("----------------------------------------------------")
        print("")
        print("Press 'Enter' to begin the demonstration ...")
        raw_input()

        # ノード立ち上げ
        rospy.init_node('smart_hand_manipulator_operation', anonymous=True)

        # クラスの作成
        smarthand = SmartHnadControl()
        group = smarthand.group

        # 【ステップ０】ホーミング動作実行
        print("Press 'Enter' : home potision start ...")
        raw_input()
        smarthand.arm_homepotision()

        while not rospy.is_shutdown():
            print("")
            print("-------------------------------------")
            print("Center of Gravity Measuring start!")

            # 【ステップ１】把持前に力・トルクを測定しておく
            wpose = group.get_current_pose().pose
            smarthand.set_init_sensor_data(0)
            sleep(1)
            smarthand.arm_orientation(0.679073820901, 0.667739432353, 0.221996218983, 0.209046585729)
            smarthand.set_init_sensor_data(1)
            smarthand.arm_orientation(wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w)

            # 【ステップ２】ハンドを開く
            print("STEP2: hand open ...")
            opening_width = smarthand.publish_hand_state("1")
            if opening_width < OPENING_WIDTH_UNDER_LIMIT:
                print("WARNING: hand is not opened!")
        
            # 【ステップ３】バイスによる固定位置の上方まで移動する
            ### ******************** 【重要】教示点を測定して設定すること ********************
            print("STEP3: move over work potision")
            smarthand.arm_move(-0.146220189754, 0.60789243192, 0.510334030623)

            raw_input("Press 'Enter' >> ")

            # 【ステップ４】ハンドを把持位置まで降ろす
            ### ******************** 【重要】教示点を測定して設定すること ********************
            print("STEP4: move to work-picking potision")
            wpose = group.get_current_pose().pose
            smarthand.arm_move(wpose.position.x, wpose.position.y, 0.403239777524)
            smarthand.arm_move(-0.142214853057, wpose.position.y, 0.403239777524)

            # 【ステップ５】ハンドを閉じる
            print("STEP5: hand close ...")
            opening_width = smarthand.publish_hand_state("0")
            if opening_width > OPENING_WIDTH_OVER_LIMIT:
                print("WARNING: hand is not closed!")

            print("重心位置測定前準備：バイスの固定を外すこと")
            raw_input("バイスによる固定を解除してから Enter >> ")

            # 【ステップ６】動摩擦測定：ハンドを上方へ移動する
            ### ******************** 【重要】教示点を測定して設定すること ********************
            print("STEP6: move over work potision")
            wpose = group.get_current_pose().pose
            smarthand.arm_move(wpose.position.x, wpose.position.y, 0.579320232505)

            sleep(0.5)

            # 【ステップ7】重心位置測定：ワークによる力・モーメントを計測する
            # 2つの姿勢で測定する
            ### ******************** 【重要】教示点を測定して設定すること ********************
            wpose = group.get_current_pose().pose
            cog_pose1 = smarthand.center_of_gravity_measurement(0)
            raw_input("Press 'Enter' >> ")
            smarthand.arm_orientation(0.679073820901, 0.667739432353, 0.221996218983, 0.209046585729)
            cog_pose2 = smarthand.center_of_gravity_measurement(1)
            center_of_gravity = smarthand.calc_center_of_gravity(cog_pose1, cog_pose2)

            raw_input("Press 'Enter' >> ")

            # 【ステップ8】姿勢をもとに戻す
            smarthand.arm_orientation(wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w)

            #raw_input("Press 'Enter' >> ")

            # 【ステップ１】ホーミング動作実行
            print("Press 'Enter' : home potision start ...")
            raw_input()
            smarthand.arm_homepotision()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
    