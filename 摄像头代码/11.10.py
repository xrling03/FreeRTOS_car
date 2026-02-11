from maix import camera, display, image,time
from maix.v1.machine import UART

#模式标记
#模式1二维码
#模式2车道定位
#模式3圆环定位
#模式4物料识别
#模式5空白
#模式6物料坐标定位
#Mode_dir=1

#黄色区域色块阈值：
#saidao_lab = 50, 72, -6, 1, -2, 11 [(59, 68, -9, 4, 3, 25)](64, 70, -9, -2, 8, 23)  40, 82, -25, 5, 4, 28   [(52, 81, -24, 5, 5, 28   //200   5, 85, -25, 5, 4, 28展示
saidao_lab = [(40, 85, -25, 8, 3, 28)]
saidao_lab_2 = [(100,255)]
#物料阈值：
#geen[(10, 28, -29, -5, -5, 11)]red(8, 33, 16, 45, -2, 22)bule(5, 29, 6, 36, -57, -27)5, 45, -67, -7, -19, 9
wuliao_lab = [(12, 56, 13, 44, 1, 25),#red
              (5, 60, -45, -7, -19, 38),#green5, 60, -45, -7, -19, 38亮      5, 60, -34, -7, -19, 9暗
              ((10, 68, 8, 39, -71, -23))]#blue
#物料阈值：
wuliaoPos_lab=[(20, 61, 16, 55, 5, 29),
               (6, 50, 16, 45, 5, 25)]

#圆环阈值：25, 80, -30, -6, -19, 38     (38, 71, -31, -15, 7, 45    28, 75, -27, -4, -15, 38
#YuanHuan_lab = [(28, 80, -25, -6, -20, 38)](39, 71, -27, -5, 7, 38(25, 75, -22, -5, -20, 40)   20, 80, -30, -6, 19, 36
YuanHuan_lab = [(25, 85, -11, -3, 3, 15)]
image.load_font("sourcehansans", "/maixapp/share/font/SourceHanSansCN-Regular.otf", size = 16)
print("fonts:", image.fonts())
image.set_default_font("sourcehansans")

class Comm:                                      #串口类
    def __init__(self):
        uart = UART("/dev/ttyS0", 115200)
        time.sleep_ms(100) # wait uart ready
        self.uart = uart
        self.Data_Temp = [0,0]

    def Split_data(self, UART_Data):             #拆分数据防止大于256
        if UART_Data != None:
            length = len(UART_Data)
            for i in range(length):
                self.Data_Temp[i] = 0
                if UART_Data[i] > 252:
                    self.Data_Temp[i] = UART_Data[i] - 252
                    UART_Data[i] = 252
                if self.Data_Temp[i] > 255:
                    self.Data_Temp[i] = 255
                if UART_Data[i] < 0:
                    UART_Data[i] = 0
        else:
            print("UART_Data is None!")

    def send_detect(self, Mode, Data):  #发送数据包
        if Mode==1:
            data = bytes([0xb3, 0xb3, 0x01, int(Data[0]), int(Data[1]), int(Data[2]), int(Data[3]), 0,0x5b])
            self.uart.write(data)
        if Mode==2:
            data = bytes([0xb3, 0xb3, 0x02, int(Data[0]), int(Data[1]), int(Data[2]), int(Data[3]), 0, 0x5b])
            self.uart.write(data)
        if Mode==3:
            data = bytes([0xb3, 0xb3, 0x03, int(Data[0]), int(Data[1]), int(Data[2]), int(Data[3]), int(Data[4]), 0x5b])
            self.uart.write(data)
        if Mode==4:
            data = bytes([0xb3, 0xb3, 0x04, int(Data), 0, 0, 0, 0, 0x5b])
            self.uart.write(data)

    def read_detect(self):                         #接受数据
        #print('RX_Data True')
        data = self.uart.read()
        # print('data:', data)
        if data == b'\x01':
            data = 1
        elif data == b'\x02':
            data = 2
        elif data == b'\x03':
            data = 3
        elif data == b'\x04':
            data = 4
        elif data == b'\x05':
            data = 5
        elif data == b'\x09':
            data = 9
        else:
            data = 0
        #print('data:', data)
        return data

def camera_Init():
    cam = camera.Camera(320, 240, fps=60)
    cam.awb_mode(0)#关闭白平衡
    cam.exp_mode(1)#关闭自动增益
    cam.exposure(300)#设置曝光率
    cam.skip_frames(30) #等待摄像头稳定
    return cam

LineData=[0,0,0,0]
#获取车道定位信息
def GetLine(img, comm):
    global LineData
    # LineData=[0,0,0,0]
    line_blob0 = img.find_blobs(saidao_lab_2, x_stride=50, y_stride=50,area_threshold=1000,pixels_threshold=1000)
    for blob in line_blob0:
        img.draw_rect(blob[0], blob[1], blob[2], blob[3], color=image.Color.from_rgb(255, 0, 0))       #圈出巡线区块
        img.draw_cross(blob.cx(), blob.cy(),image.Color.from_rgb(255, 0, 0), size=12, thickness=2)    #圈出中心十字

    if line_blob0:
        LineData[1] = int(line_blob0[0].w())
        LineData[3] = int(line_blob0[0].h())
        if LineData[1]>255:
            LineData[0]=LineData[1]-255
            LineData[1]=255
        if LineData[3]>255:
            LineData[2]=LineData[3]-255
            LineData[3]=255

        comm.send_detect(0x02, LineData)
        # print(LineData)

Yuanxy=[0,0,0,0,0]
#得到寻找圆环坐标函数：
def GetYuan(img, comm):
    global Yuanxy
    center_x=0
    center_y=0
    # Yuanxy=[0,0,0,0] 
    c=img.find_circles(threshold=6000, r_margin=5,r_min=28, r_max=30, r_step=1)
    '''
    for c1 in c:
        img.draw_circle(c1.x(), c1.y(), c1.r(), color=image.COLOR_WHITE)
        img.draw_circle(c1.x(), c1.y(), 2, color=image.COLOR_WHITE)
        print(c1.magnitude())
    '''
    if c:
        count = 0
        for circle in c:
            center_x += circle.x()
            center_y += circle.y()
            print(circle.r())
            img.draw_circle(int(circle.x()), int(circle.y()), int(circle.r()), color=image.Color.from_rgb(255, 0, 0))  # 标注圆
            count += 1
        center_x /= count
        center_y /= count
        # img.draw_circle(int(center_x), int(center_y), 2, color=image.COLOR_WHITE)
        img.draw_cross(int(center_x), int(center_y), image.Color.from_rgb(255, 0, 0), size=12, thickness=2)

        Yuanxy[1] = int(center_x)
        Yuanxy[3] = int(center_y)
        if Yuanxy[1]>255:
            Yuanxy[0]=Yuanxy[1]-255
            Yuanxy[1]=255
        if Yuanxy[3]>255:
            Yuanxy[2]=Yuanxy[3]-255
            Yuanxy[3]=255

        print(Yuanxy)
        comm.send_detect(0x03, Yuanxy)

Yuanxy2=[0,0,0,0,0]
#得到寻找圆环坐标函数：
def GetYuan2(img, comm):
    center_x=0
    center_y=0
    global Yuanxy2
    # Yuanxy=[0,0,0,0]
    c=img.find_circles(threshold=7000, x_margin=5, y_margin=5, r_margin=5,
                      r_min=20, r_max=100, r_step=2)
    '''
    for c1 in c:
        img.draw_circle(c1.x(), c1.y(), c1.r(), color=image.COLOR_WHITE)
        img.draw_circle(c1.x(), c1.y(), 2, color=image.COLOR_WHITE)
        print(c1.magnitude())
    '''
    if c:
        count = 0
        for circle in c:
            center_x += circle.x()
            center_y += circle.y()
            img.draw_circle(int(circle.x()), int(circle.y()), int(circle.r()), color=image.COLOR_YELLOW)
            count += 1
        center_x /= count
        center_y /= count
        # img.draw_circle(int(center_x), int(center_y), 2, color=image.COLOR_WHITE)
        img.draw_cross(int(center_x), int(center_y), image.Color.from_rgb(255, 0, 0), size=14, thickness=1)

        Yuanxy2[1] = int(center_x)
        Yuanxy2[3] = int(center_y)
        if Yuanxy2[1]>255:
            Yuanxy2[0]=Yuanxy2[1]-255
            Yuanxy2[1]=255
        if Yuanxy2[3]>255:
            Yuanxy2[2]=Yuanxy2[3]-255
            Yuanxy2[3]=255

        print(Yuanxy2)
        comm.send_detect(0x03, Yuanxy2)

LineDatax=[0, 0, 0, 0, 0]
#得到物料颜色函数：
def GetColor(img, comm):
    b=0
    global LineDatax
    WuLiao = img.find_blobs(wuliao_lab, area_threshold=4000,merge=False)
    for blob in  WuLiao:
        if blob:
            if(blob.code()==1):  #红色
                img.draw_rect(blob.rect()[0], blob.rect()[1], blob.rect()[2], blob.rect()[3],color=image.COLOR_WHITE)      #如果识别到了颜色，则在色块周围画一个框框
                img.draw_cross(blob.cx(), blob.cy(),color=image.COLOR_WHITE) #在色块中心打印一个十字，参数就是中心坐标
                #img.draw_string(blob.cx(),(blob.cy()+20),"红色",color=image.COLOR_WHITE)
                img.draw_string(blob.x(),blob.y()-18,"红色",color=image.COLOR_WHITE)
                b = 0x01
                print(b)

            elif(blob.code()==2):  #绿色
                img.draw_rect(blob.rect()[0], blob.rect()[1], blob.rect()[2], blob.rect()[3],color=image.COLOR_WHITE) 
                img.draw_cross(blob.cx(), blob.cy(),color=image.COLOR_WHITE)
                img.draw_string(blob.x(),blob.y()-18,"绿色",color=image.COLOR_WHITE)
                #img.draw_string(blob.cx(),(blob.cy()+20),"绿色",color=image.COLOR_WHITE)
                b = 0x02
                print(b)

            elif(blob.code()==4):  #蓝色
                img.draw_rect(blob.rect()[0], blob.rect()[1], blob.rect()[2], blob.rect()[3],color=image.COLOR_WHITE) 
                img.draw_cross(blob.cx(), blob.cy(),color=image.COLOR_WHITE)
                img.draw_string(blob.x(),blob.y()-18,"蓝色",color=image.COLOR_WHITE)
                #img.draw_string(blob.cx(),(blob.cy()+20),"蓝色",color=image.COLOR_WHITE)
                b = 0x03
                print(b)

            LineDatax[1] = int(blob.cx())
            LineDatax[3] = int(blob.cy())
            if LineDatax[1]>255:
                LineDatax[0]=LineDatax[1]-255
                LineDatax[1]=255
            if LineDatax[3]>255:
                LineDatax[2]=LineDatax[3]-255
                LineDatax[3]=255
            LineDatax[4] = b
            print(LineDatax)
            comm.send_detect(0x03, LineDatax)

###########################################  主函数  ################################################

t = 1
t2 = 1
Mode_dir=2

def main():
    global Mode_dir
    global LineData
    global LineDatax
    global Yuanxy
    global Yuanxy2

    global t
    global t2

    comm = Comm()#串口初始化
    cam = camera_Init()#摄像头初始化
    disp = display.Display()

    while(True):
        if Mode_dir==9:
            while(True):
                t = time.ticks_ms()
                img = cam.read()#.lens_corr(strength = 1.5, zoom = 1.0)#减弱鱼眼
                #comm.send_detect(0x01, LineData)
                # #屏幕显示
                img.draw_string(2, 2, "fps:%.2f " % (1000.0/t2), image.Color.from_rgba(255, 255, 0, 0.8))
                img.draw_string(120, 2, "模式1:无功能", image.Color.from_rgba(255, 255, 0, 0.8))
                # print("mode1", Mode_dir)
                disp.show(img)  
                Mode_dir = comm.read_detect() 
                if Mode_dir == 1 or Mode_dir == 2  or Mode_dir == 4 :
                    break 

                t2 = time.ticks_ms() - t

                print(Mode_dir)
                #print(int(1000.0/t2))
    #模式2识别车道
        if Mode_dir==1:
            # cam.awb_mode(0)#关闭白平衡
            # cam.exp_mode(1)#关闭自动增益
            cam.exposure(350)#设置曝光率30
            cam.skip_frames(30) #等待摄像头稳定
            while(True):
                t = time.ticks_ms()
                #img = cam.read().lens_corr(strength = 1.5, zoom = 1.0)#减弱鱼眼
                img = cam.read().lens_corr(strength = 1.5, zoom = 1.0).binary(saidao_lab).dilate(1)
                GetLine(img, comm)                        #从图像中得到赛道rho
                img.draw_string(2, 2, "fps:%.2f " % (1000.0/t2), image.Color.from_rgba(255, 255, 0, 0.8))
                img.draw_string(120, 2, "模式2:巡线定位", image.Color.from_rgba(255, 255, 0, 0.8))
                img.draw_string(2, 20, "w:%d  h:%d" % (LineData[0]+LineData[1], LineData[2]+LineData[3]), image.Color.from_rgba(255, 255, 0, 0.8))
                disp.show(img)
                Mode_dir = comm.read_detect()
                print(Mode_dir)
                #Mode_dir = comm.read_detect()
                '''
                if Mode_dir!=1:
                    break
                '''
                if Mode_dir == 2 or Mode_dir == 3 or Mode_dir == 4 or Mode_dir == 5 or Mode_dir == 9:
                    break
                t2 = time.ticks_ms() - t

        #模式3圆环定位
        elif Mode_dir==2:
            cam.exposure(200)#设置曝光率
            cam.skip_frames(100) #等待摄像头稳定
            while(True):
                t = time.ticks_ms()
                #img = cam.read()#.lens_corr(strength = 1.5, zoom = 1.0)#减弱鱼眼
                img = cam.read().binary(YuanHuan_lab).dilate(2)#读取一张图片
                GetYuan(img, comm)
             
                img.draw_string(2, 2, "fps:%.2f " % (1000.0/t2), image.Color.from_rgba(255, 255, 0, 0.8))
                img.draw_string(120, 2, "模式3:识别圆环定位", image.Color.from_rgba(255, 255, 0, 0.8))
                img.draw_string(2, 20, "cx:%d  cy:%d" % (Yuanxy[0]+Yuanxy[1], Yuanxy[2]+Yuanxy[3]), image.Color.from_rgba(255, 255, 0, 0.8))
                disp.show(img)
                # print("mode3", Mode_dir)
                Mode_dir = comm.read_detect()
                '''
                if Mode_dir!=1:
                    break
                '''
                if Mode_dir == 1 or Mode_dir == 3 or Mode_dir == 4 or Mode_dir == 5 or Mode_dir == 9:
                    break
                t2 = time.ticks_ms() - t
                print(int(1000.0/t2))

        #模式4识别物料
        elif Mode_dir==4:
            cam.exposure(400)#设置曝光率
            cam.skip_frames(150) #等待摄像头稳定
            while(True):
                t = time.ticks_ms()
                img = cam.read()#.lens_corr(strength = 1.5, zoom = 1.0)#减弱鱼眼
                GetColor(img, comm)
                img.draw_string(2, 2, "fps:%.2f " % (1000.0/t2), image.Color.from_rgba(255, 255, 0, 0.8))
                img.draw_string(2, 20, "cx:%d  cy:%d" % (LineDatax[0]+LineDatax[1], LineDatax[2]+LineDatax[3]), image.Color.from_rgba(255, 255, 0, 0.8))
                img.draw_string(120, 2, "模式5:识别物料", image.Color.from_rgba(255, 255, 0, 0.8))
                disp.show(img)
                # print("mode5", Mode_dir)
                Mode_dir = comm.read_detect()
                '''
                if Mode_dir!=1:
                    break
                '''
                if Mode_dir == 1 or Mode_dir == 2 or Mode_dir == 3 or Mode_dir == 5 or Mode_dir == 9:
                    break

                t2 = time.ticks_ms() - t
                print(int(1000.0/t2))

if __name__ == "__main__":
    main()