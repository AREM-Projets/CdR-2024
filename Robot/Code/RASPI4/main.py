"""Programme principal du robot de la CDR 2024



https://rplidar.readthedocs.io/en/latest/

angles mesures en deg
distances mesures en mm !!
"""

import time
import serial
from multiprocessing import Process, Queue
from rplidar import RPLidar
import RPi.GPIO as GPIO
from tkinter import *



PIN_TIRETTE = 16
PIN_SELECTEUR_EQUIPE = 15 #23


#commandes embase
INIT = b'i' #restart sans redemarrer la carte
START = b's'
WAIT = b'w'

#parametre LIDAR
SEUIL_DETECTION = 400 #mm



def affichage_score(file_score):
    """
    Processus qui affiche et met a jour le score du robot
    """
    print("Demarrage affichage score...")

    def update_affichage_score():
        score_label.config(text="Score robot: "+str(file_score.get()))
        fenetre.after(100, update_affichage_score)



    fenetre = Tk()
    score_label = Label(fenetre, text="Score robot: 0", font='Times 50')
    score_label.pack()

    fenetre.after(100, update_affichage_score)


    fenetre.mainloop()







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
                        if(0 < distance < SEUIL_DETECTION):
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















def main(file_scans, file_score):
    print("Demarrage MAIN...")

    score = 30 #6 panneaux et retour zone finale
    DUREE_VIE_SCAN_LIDAR = 2 #duree de vie d'un scan lidar en secondes (au bout de ce temps une mesure du lidar ne sera pas prise en compte)



    #setup de la tirette et du selecteur equipe
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(PIN_TIRETTE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 16 en input pulldown pour la tirette
    GPIO.setup(PIN_SELECTEUR_EQUIPE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 26 en input pulldown pour le sélecteur équipe à 0 par défaut : 0:Bleu; 1:Jaune

    equipe = GPIO.input(PIN_SELECTEUR_EQUIPE) #0: bleu; 1:jaune

    print("tirette:", GPIO.input(PIN_TIRETTE))
    print("équipe:", equipe)



    #connexion a l'embase
    port_embase = serial.Serial('/dev/embase', 115200)
    print("[OK] Connexion embase")
    port_embase.write(INIT)


    #connextion a la carte des actionneurs
    port_actionneur = serial.Serial('/dev/actionneur', 9600)
    print("[OK] Connexion actionneur")

    #bloquage tant que la tirette n'est pas tiree
    while(GPIO.input(PIN_TIRETTE)): pass






    port_actionneur.write(b'e') #activation actionneur

    time.sleep(7)


    #if (file_scans.empty()):
    port_embase.write(START)
    #test

    print("boucle principale")

    while(1):
        #score +=1
        file_score.put(score)

        if not file_scans.empty():
            scan = file_scans.get().split(',') #recuperation du dernier scan du lidar

            if(time.monotonic() - float(scan[0]) < DUREE_VIE_SCAN_LIDAR):
                #si le dernier scan n'est pas perime, on fait des trucs avec
                print("\nSCAN: ({}) {}".format(time.monotonic(), scan))
                port_embase.write(WAIT)





















if __name__ == "__main__":
    #gestion au plus haut niveau du fonctionnement du robot


    # Files de communication entre processus
    file_scans = Queue() # stocke les valeurs envoyees par le lidar si il detecte
    file_score = Queue() #stocke les valeurs de score calculées pour les passer au processus d'affichage

    # Creation des processus concourants pour le robot
    lidar_process =             Process(target=lidar, args=(file_scans,))
    affichage_score_process =   Process(target=affichage_score, args=(file_score,))
    main_process =              Process(target=main, args=(file_scans, file_score))


    # Demarrage des processus, attente de 90 secondes et arret des processus
    print("Demarrage des processus...")

    lidar_process.start()
    affichage_score_process.start()
    main_process.start()
    print("Processus demarres.\n")

    time.sleep(90)

    print("Arret des processus...")
    lidar_process.terminate()
    main_process.terminate()
    print("Processus arretes.\n")