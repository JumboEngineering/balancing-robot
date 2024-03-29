from machine import ADC, Pin
import time
import math

Richtung = Pin(0, Pin.OUT)
Geschwindigkeit=machine.Pin(1)
anschalten= Pin(2, Pin.OUT)
Poti= Pin(14, Pin.OUT)
adc = ADC(Pin(27))
anschalten2 = Pin(3, Pin.OUT)
Richtung2 = Pin(4, Pin.OUT)
Geschwindigkeit2 =machine.Pin(5)


I=0
v=0
winkel=0
winkelx=0
winkely=0
letzte_zeit=time.ticks_ms()
Kalmanngain=4 
winkeloriginal=0
zehn=0
kalibrieren=0


def limit_to_range(value):
    if value > 0.5:
        return 0.5
    elif value < -0.5:
        return -0.5
    else:
        return value



i2c = machine.I2C(0, scl=machine.Pin(13), sda=machine.Pin(12))
i2c.scan()  # Dies sollte die Adresse 0x68 für den MPU6050 zurückgeben

MPU6050_ADDR = 0x68  # I2C-Adresse des MPU6050
MPU6050_REG_DATA = 0x3B  # Registeradresse für Rohdaten
i2c.writeto(MPU6050_ADDR, bytearray([0x6B, 0x00]))  # MPU6050 aktivieren (Stromversorgung einschalten)

anschalten.off()
anschalten2.off()
pwmv= machine.PWM(Geschwindigkeit)
pwmv2= machine.PWM(Geschwindigkeit2)
pwmv.freq(int(300*32))
pwmv.duty_u16(int(65535/2))
pwmv2.duty_u16(int(65535/2))

Grenze=50000

while True:
    # Daten aus dem MPU6050 auslesen
    data = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_REG_DATA, 14)

    # Daten interpretieren (Beschleunigung und Gyroskop)
    accel_x = (data[0] << 8) | data[1]
    accel_y = (data[2] << 8) | data[3]
    accel_z = (data[4] << 8) | data[5]

    gyro_x = (data[8] << 8) | data[9]
    gyro_y = (data[10] << 8) | data[11]
    gyro_z = (data[12] << 8) | data[13]

    # MPU6050 gibt Daten im 16-Bit-Zweierkomplementformat zurück, daher Konvertierung
    accel_x = accel_x if accel_x < 32768 else accel_x - 65536
    accel_y = accel_y if accel_y < 32768 else accel_y - 65536
    accel_z = accel_z if accel_z < 32768 else accel_z - 65536

    gyro_x = gyro_x if gyro_x < 32768 else gyro_x - 65536
    gyro_y = gyro_y if gyro_y < 32768 else gyro_y - 65536
    gyro_z = gyro_z if gyro_z < 32768 else gyro_z - 65536
    
    
    dt=(time.ticks_ms()-letzte_zeit)/1000
    letzte_zeit=time.ticks_ms()
    gyroaccel=1-Kalmanngain*dt
    if accel_x != 0:
        winkelx_accel=math.degrees(math.atan(accel_y/(math.sqrt(accel_x*accel_x+accel_z*accel_z))))
    if accel_y != 0:
        winkely_accel=math.degrees(math.atan(-accel_x/(math.sqrt(accel_y*accel_y+accel_z*accel_z))))
    winkeloriginal=(winkeloriginal+gyro_y/16384*dt/3.14159*360)
    winkelx=(winkelx+gyro_x/16384*dt/3.14159*360)*gyroaccel+winkelx_accel*(1-gyroaccel)
    winkely=(winkely+gyro_y/16384*dt/3.14159*360)*gyroaccel+winkely_accel*(1-gyroaccel)
    
    time.sleep_ms(50)
    print(winkeloriginal,winkely_accel,winkely)     #Demo "Best of both worlds"
    
    I=I+winkely*dt/8
    I=limit_to_range(I)
    #winkel=(winkely+4.5)/2+gyro_y/10000
    winkel=limit_to_range((winkely)/120+I)
    
    kalibrieren=kalibrieren+gyro_y
    
    
    
    
    
    drehschalter=winkel+0.5
    
    
    #drehschalter=(adc.read_u16()+1)/(2**16)
    
    if -20 <= winkely <= 20:
        anschalten.off()
        anschalten2.off() 
    else:
        anschalten.on()
        anschalten2.on()
    
    #print(drehschalter)
    #time.sleep_ms(50)
    
    if drehschalter<0.5:
        Richtung.on()
        Richtung2.on()
        pwmv.freq(int(8+(Grenze/2-drehschalter*Grenze)))
        pwmv2.freq(int(8+(Grenze/2-drehschalter*Grenze)))
    else:
        Richtung.off()
        Richtung2.off()
        pwmv.freq(int(8+drehschalter*Grenze-Grenze/2))
        pwmv2.freq(int(8+drehschalter*Grenze-Grenze/2))
    #pwmv.freq(10+adc.read_u16())#/ (2**16 - 1))
    #time.sleep_ms(50)
    #print(winkely)

