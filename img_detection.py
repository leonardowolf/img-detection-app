import sensor,image,lcd,time
import KPU as kpu
from machine import Timer,PWM
import time
from board import board_info

safe_w = 120
safe_h = 180
s_radius = 5
green = (170,219,30)
red = (235,33,46)
color = green
classes = ['aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor']
unsupressed_classes = ['bicycle', 'bird', 'bottle', 'bus', 'car', 'cat', 'chair', 'cow', 'dog', 'horse', 'motorbike', 'person']
task = kpu.load(0x800000)
anchor = (1.08, 1.19, 3.42, 4.41, 6.63, 11.38, 9.42, 5.11, 16.62, 10.52)
danger = False
tougle = True
isenable = False
counter = 0


lcd.init()
lcd.rotation(0)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_vflip(0)
sensor.set_hmirror(0)
sensor.set_vflip(0)
sensor.run(1)
clock = time.clock()

a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)


#function defines
def on_timer(timer):
    global tougle
    global counter
    global isenable

    if counter == 3:
        isenable = False
        bzr.disable()
        tim.stop()

    if tougle:
        if isenable:
            bzr.enable()
            isenable = True
        tougle = False
        counter = counter + 1
    else:
        bzr.disable()
        tougle = True

def audioSignal():
    bzr.enable()
    tim.start()

def drawBounds(img):
    #representacao visual limite seguro
    global color
    global danger

    a = img.draw_rectangle((160 - safe_w//2),240 - safe_h,safe_w,safe_h,color,3)
    a = lcd.display(img)
    #lcd.draw_string(0, 200, ( "danger" if danger else "safe") , lcd.RED, lcd.WHITE)

def senseDanger(t_bounds):
    x,y,w,h = t_bounds
    global danger
    global color
    global s_radius, safe_w, safe_h

    dzonex_start = 160 - safe_w//2
    dzonex_end = 160 + safe_w//2
    dzoney_start = 120 - safe_h//2

    xcenter_obj = x + w//2
    ycenter_obj = y + y//2

    if xcenter_obj < dzonex_start - s_radius:
        danger = False
        color = green
    elif xcenter_obj == dzonex_start - s_radius or xcenter_obj <= dzonex_end+s_radius:
        if ycenter_obj >= dzoney_start + s_radius//2:
            danger = True
            color = red
    elif xcenter_obj > dzonex_end+s_radius:
        danger = False
        color = green
    else:
        danger = False
        color = green

    if danger:
        audioSignal()



#timers
tim = Timer(Timer.TIMER1, Timer.CHANNEL0, mode=Timer.MODE_PWM)
bzr = PWM(tim, freq=800, duty=50, pin=11)


tim = Timer(Timer.TIMER2, Timer.CHANNEL0, mode=Timer.MODE_PERIODIC, period=500, callback=on_timer, arg=on_timer)
tim.stop()


#main loop
while(True):
    clock.tick()
    img = sensor.snapshot()
    drawBounds(img)
    code = kpu.run_yolo2(task, img)
    print(clock.fps())



    if code:
        for i in code:
            if classes[i.classid()] in unsupressed_classes:
                    a=img.draw_rectangle(i.rect())
                    a = lcd.display(img)
                    senseDanger(i.rect())

                    for i in code:
                        lcd.draw_string(i.x(), i.y(), classes[i.classid()], lcd.RED, lcd.WHITE)
                        print( classes[i.classid()])
                        #lcd.draw_string(i.x(), i.y()+12, '%f1.3'%i.value(), lcd.RED, lcd.WHITE)
    else:
        a = lcd.display(img)


a = kpu.deinit(task)
