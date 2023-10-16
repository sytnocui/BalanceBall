import vgamepad as vg
import time
import serial
import struct

## 串口相关
# 打开串口，并得到串口对象
ser = serial.Serial("COM9", 115200, timeout=None)

# 虚拟手柄
gamepad = vg.VX360Gamepad()

# -------- Main Program Loop -----------
while True:

    #############################################################################
    msg = ser.readline()
    if msg:
        str_msg = msg.decode('utf-8')
        # print(str_msg)
        AttitudeDataArray = str_msg.split(",")
        # 弧度制后差不多就不用缩放范围了
        roll = float(AttitudeDataArray[0]) / 180 * 3.14
        pitch = float(AttitudeDataArray[1]) / 180 * 3.14
        yaw = float(AttitudeDataArray[2]) / 180 * 3.14

        # 赋值
        # 假设从某个输入源获取了 X 和 Y 轴的数据，范围为 -1.0 到 1.0
        axis_Throttle = 0.0
        axis_Brake = 0.0
        axis_Clutch = 0.0

        # 融合模式，容易飘
        # axis_SteeringWheel = (roll + (-yaw)) / 2
        # 单独roll决定方向，这个效果可以
        axis_SteeringWheel = roll
        if pitch >= 0:
            axis_Brake = pitch
        else:
            axis_Throttle = -pitch

        #############################################################################

        # press a button to wake the device up
        # if button_Upshift:
        #     gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
        # else:
        #     gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
        #
        # if button_Downshift:
        #     gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
        # else:
        #     gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
        #
        # if button_Handbrake:
        #     gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
        # else:
        #     gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)

        gamepad.left_trigger_float(value_float=axis_Brake)
        gamepad.right_trigger_float(value_float=axis_Throttle)
        gamepad.left_joystick_float(x_value_float=axis_SteeringWheel, y_value_float=0.0)
        # gamepad.right_joystick_float(x_value_float=-1.0, y_value_float=1.0)

    gamepad.update()

    # 给下位机发送数据
    msg = struct.pack("<2H3f", 2000, 1000,
                      0.0, 0.0, 0.0)
    # print(msg)
    result = ser.write(msg)  # 串口写数据

    time.sleep(0.01)  # 适当的延迟，避免数据过于频繁发送
