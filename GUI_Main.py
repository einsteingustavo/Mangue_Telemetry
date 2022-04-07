from struct import unpack
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import threading
import serial
import serial.tools.list_ports
import time
import pandas as pd
from numpy.random import randn
from numpy.fft import rfft
from scipy import signal

ID = 11

 
serial_ports_raw = serial.tools.list_ports.comports()

serial_ports = [str(x)[:4] for x in serial_ports_raw]

#print(serial_ports) #debug
#['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
SIZE = 60
FORMAT = '<BHHHHHHHHHHHHHHHHHHHHHHHHHHBBLB'

time = deque(200*[0], 200)
accx = deque(200*[0], 200)
accy = deque(200*[0], 200)
accz = deque(200*[0], 200)
rpm = deque(200*[0], 200)
speed = deque(200*[0], 200)
temp = deque(200*[0], 200)
car = deque(200*[''], 200)
eixo = deque(200*[0], 200)

b, a = signal.butter(1, 0.1, analog=False)


time_save = []
accx_save = []
accy_save = []
accz_save = []
rpm_save = []
speed_save = []
temp_save = []
car_save = []



class Receiver(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self, name = name)
        self.com = self.connectSerial(serial_ports)
        print(f'Connected into {self.com}')

    def connectSerial(self, USB_PORT):
        for usb in USB_PORT:
            try:
                com = serial.Serial(f'{usb}', 115200)
            except:
                print("Tentativa...")
                com = []
            if com:
                break
        
        if not com:
            raise Exception("Não há nenhuma porta serial disponível")
        else:
            return com

    def run(self):
        self.com.flush()

        while True:
            self.checkData()
    
    def checkData(self):
        c = 0
        while c != b'\xff':
            c = self.com.read()
            #print(f'trying, {c}')
        msg = self.com.read(SIZE)
        #print(msg)
        pckt = list(unpack(FORMAT, msg))
        #print(pckt)
        #print((pckt[25]/65535)*5000)
        if pckt[0] == 22:
            car.append("MB2")
            accx.append(pckt[19]*0.061/1000)
            accy.append(pckt[20]*0.061/1000)
            accz.append(pckt[21]*0.061/1000)
            rpm.append((pckt[25]/65535)*5000)
            #print((pckt[25]/65535)*5000)
            speed.append((pckt[26]/65535)*60)
            temp.append(pckt[27])
            time.append(pckt[29])
            car_save.append(pckt[0])
            accx_save.append(pckt[19]*0.061/1000)
            accy_save.append(pckt[20]*0.061/1000)
            accz_save.append(pckt[21]*0.061/1000)
            rpm_save.append((pckt[25]/65535)*5000)
            speed_save.append((pckt[26]/65535)*60)
            temp_save.append(pckt[27])
            time_save.append(pckt[29])
        if pckt[0] == 11:
            car.append("MB1")
            accx.append(pckt[19]*0.061/1000)
            accy.append(pckt[20]*0.061/1000)
            accz.append(pckt[21]*0.061/1000)
            rpm.append((pckt[25]/65535)*5000)
            speed.append((pckt[26]/65535)*60)
            temp.append(pckt[27])
            time.append(pckt[29])
            car_save.append(pckt[0])
            accx_save.append(pckt[19]*0.061/1000)
            accy_save.append(pckt[20]*0.061/1000)
            accz_save.append(pckt[21]*0.061/1000)
            rpm_save.append((pckt[25]/65535)*5000)
            speed_save.append((pckt[26]/65535)*60)
            temp_save.append(pckt[27])
            time_save.append(pckt[29])

        #print(temp)


        #print(accx_save)  
            
    
        data = {
        'Tempo': time_save,
        'Carro': car_save,
        'Aceleração X': accx_save,
        'Aceleração Y': accy_save,
        'Aceleração Z': accz_save, 
        'RPM': rpm_save,
        'Velocidade': speed_save,
        'Temperatura': temp_save
        }      
        csv = pd.DataFrame(data, columns=['Tempo','Carro', 'Aceleração X', 'Aceleração Y', 'Aceleração Z', 'RPM', 'Velocidade', 'Temperatura'])
        csv.to_csv('dados_telemetria.csv')

        

if __name__ == "__main__":
    box = Receiver(name = 'serial_port')
    box.start()

    cont = 0
    rpm_plt = plt.subplot2grid((3, 3), (1, 2), rowspan=2)
    speed_plt = plt.subplot2grid((3, 3), (0, 1), colspan=2)
    temp_plt = plt.subplot2grid((3, 3), (0, 0))
    imu_plt = plt.subplot2grid((3, 3), (1, 0), colspan=2, rowspan=2)



    while True:
        rpm_plt.clear()
        speed_plt.clear()
        imu_plt.clear()
        temp_plt.clear()

        cont += 1
        eixo.append(cont)

        sig_rpm = signal.filtfilt(b, a, rpm)
        sig_speed = signal.filtfilt(b, a, speed)
        sig_accx = signal.filtfilt(b, a, accx)
        sig_accy = signal.filtfilt(b, a, accy)
        sig_accz = signal.filtfilt(b, a, accz)

        #print(temp)
        temp_plt.plot(eixo, temp, 'c-', marker="h")
        temp_plt.set_title('Temperatura ' + car[-1])
        temp_plt.set_xlim(-50 + cont, cont)
        temp_plt.set_ylim(0, 150)

        rpm_plt.plot(eixo, sig_rpm, 'c-', marker="h")
        rpm_plt.set_title('Rotação do motor ' + car[-1])
        rpm_plt.set_xlim(-50 + cont, cont)
        rpm_plt.set_ylim(0, 6000)

        speed_plt.plot(eixo, sig_speed, 'k-', marker="h")
        speed_plt.set_title('Velocidade '+ car[-1])
        speed_plt.set_xlim(-50 + cont, cont)
        speed_plt.set_ylim(0, 80)
        plt.grid(True)

        imu_plt.plot(eixo, sig_accx, 'b-', marker="h", label='Eixo X')
        imu_plt.plot(eixo, sig_accy, 'r-', marker="h", label='Eixo Y')
        imu_plt.plot(eixo, sig_accz, 'g-', marker="h", label='Eixo Z')
        imu_plt.set_title('Aceleração ' + car[-1])
        imu_plt.set_xlim(-200 + cont, cont)
        imu_plt.set_ylim(-4,4)
        imu_plt.legend()

     
  

        plt.pause(0.05)