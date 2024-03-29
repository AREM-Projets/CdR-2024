"""Programme principal du robot de la CDR 2024
Auteur(s):
- Antoine Chassaigne



https://rplidar.readthedocs.io/en/latest/

angles mesures en deg
distances mesures en mm !!
"""

import time
import serial
from multiprocessing import Process, Queue
from rplidar import RPLidar




def lidar(file_scans):
    print("Demarrage LIDAR...")
    lidar = RPLidar('/dev/rplidar')

    lidar.reset()
    time.sleep(1)

    while True:
        try:

            for i, scan in enumerate(lidar.iter_scans()):
                #print('%d: Got %d measurments' % (i, len(scan)))
                
                for mesure in scan:
                    
                    try:
                        date = time.monotonic()
                        angle = int(mesure[1])
                        distance = int(mesure[2])
                        
                        #publication de la mesure du lidar uniquement si on detecte un obstacle a moins de 20 cm
                        if(0 < distance < 200):
                            file_scans.put(f"{date},{angle},{distance}")
                        
                    except:
                        pass
                    
                lidar.clean_input()
                    
        except:
            #si le lidar plante pour une quelconque raison on le redemarre
            lidar.stop()
            print("Fail du LIDAR, restart...")
            lidar.reset()
            time.sleep(2)

        



def main(file_scans):
    DUREE_VIE_SCAN_LIDAR = 2 #duree de vie d'un scan lidar en secondes (au bout de ce temps une mesure du lidar ne sera pas prise en compte)

    print("Demarrage MAIN...")

    #connextion a la carte des actionneurs / capteurs
    port = serial.Serial('/dev/ttyUSB_STM32_ACTIONNEURS', 9600)

    #test de la connexion
    port.write(b'test')
    time.sleep(0.2)  #petit delai pour que la stm puisse publier sur le port usb apres reception du 'test'
    print(port.readline() == b'Received : test\r\n')

    #boucle principale de controle du robot
    while True:
        scan = file_scans.get().split(',') #recuperation du dernier scan du lidar

        if(time.monotonic() - float(scan[0]) < DUREE_VIE_SCAN_LIDAR):
            #si le dernier scan n'est pas perime, on fait des trucs avec
            print("\nSCAN: ({}) {}".format(time.monotonic(), scan))

            #on envoie juste le scan a la carte stm qui nous reponds si elle le recois bien
            message = "f {} {}".format(scan[1], scan[2])
            port.write(bytes(message, "utf8"))
            time.sleep(0.2)
            print(port.readline())





if __name__ == "__main__":
    #gestion au plus haut niveau du fonctionnement du robot

    # Files de communication entre processus
    file_scans = Queue() # stocke les valeurs envoyees par le lidar si il detecte

    # Creation des processus concourants pour le robot
    lidar_process = Process(target=lidar, args=(file_scans,))
    main_process = Process(target=main, args=(file_scans,))
        

    # Demarrage des processus, attente de 90 secondes et arret des processus
    print("Demarrage des processus...")
    lidar_process.start()
    main_process.start()
    print("Processus demarres.\n")

    time.sleep(90)

    print("Arret des processus...")
    lidar_process.terminate()
    main_process.terminate()
    print("Processus arretes.\n")



