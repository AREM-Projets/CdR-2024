"""Programme principal du robot de la CDR 2024



https://rplidar.readthedocs.io/en/latest/

angles mesures en deg
distances mesures en mm !!



TODO:
- mettre toutes les connexions necessaires avant le while de la tirette et afficher que tout a reussi a bien se connecter
"""

import time
import serial
from multiprocessing import Process, Queue
from rplidar import RPLidar
import RPi.GPIO as GPIO
from tkinter import *



PIN_TIRETTE = 16
PIN_SELECTEUR_EQUIPE = 15 #23



def affichage_score(file_score):
    """
    Fonction qui affiche et met a jour le score du robot sur l'écran
    Cette fonction est executee dans un processus independant
    L'echange de donnees avec le processus main est realise via la file de donnees file_score

    C'est dans cette fonction que l'on peut ajouter des trucs sur l'écran
    """
    UPDATE_PERIOD = 100 #periode d'update de l'affichage du score a l'ecran en ms

    def update_affichage_score():
        """
        Fonction appelée toute les UPDATE_PERIOD pour mettre a jour le score du robot
        La fonction lit le score publie sur la file file_score par le processus principal
        La fonction s'auto-apelle avec la méthode .after(...)
        """
        score_label.config(text="Score robot: "+str(file_score.get()))
        fenetre.after(UPDATE_PERIOD, update_affichage_score)



    fenetre = Tk()
    score_label = Label(fenetre, text="Score robot: 0", font='Times 50') #creation du label d'affichage de score
    score_label.pack()

    fenetre.after(UPDATE_PERIOD, update_affichage_score) #premier appel de la fonction de mise a jour du score au bout de UPDATE_PERIOD


    fenetre.mainloop()







def lidar(file_scans):
    """
    Comme son nom l'indique, fonction qui gere le LIDAR et qui est executee dans un processus independant
    
    La fonction communique avec la fonction main via la file de donnees file_scans sur laquelle la fonction lidar publie les mesures du LIDAR sous le format:
    [date (s), angle (deg), distance (mm)]
    """

    print("Demarrage LIDAR...")
    # c'est ici que le nom du port usb du lidar doit etre renseigne.
    # Normalement il est renomme dans un fichier de config de la raspi en "rplidar"
    # mais si il y a reinstallation du code il faut le refaire (cf la partie doc du git)

    lidar = RPLidar('/dev/rplidar')

    lidar.reset()
    time.sleep(1)

    while True:
        # le LIDAR a la facheuse manie de planter parfois sans raison (probablement le cablage foireux usb mais on y peut rien)
        # la solution choisie est donc de gerer la recuperation des scans lidar dans une boucle infinie que voici
        # dans laquelle on tente de lire les mesures du lidar et si on peut pas c'est qu'il a plante. Dans ce cas
        # on le reboot automatiquement. On perds une ou deux secondes de mesures
 
        try:

            for i, scan in enumerate(lidar.iter_scans()):
                # le lidar dispose de deux methodes pour recuperer les mesures
                # la premiere : iter_mesures (sintaxe a verifier) renvoie une liste de mesures qui se remplit constamment
                # la seconde: iter_scans() utilisee ici renvoie une liste de scans qui sont les mesures effectuees en un tour du lidar
                # il faut noter que les listes renvoyees par ces deux methodes sont dynamiques et remplie en permanence par le lidar.
                # Par consequent si on ne lit pas plus vite que le lidar ne publie de mesure (ce qui est le cas),
                # la boucle for de parcours des scans est une boucle infinie. D'ou la necessite de mettre la gestion du lidar dans un processus isole


                for mesure in scan:
                    
                    try:
                        date = time.monotonic()
                        angle = int(mesure[1])
                        distance = int(mesure[2])
                        
                        #publication de la mesure du lidar uniquement si on detecte un obstacle a moins de 20 cm
                        if(0 < distance < 300):
                            file_scans.put(f"{date},{angle},{distance}")
                        
                    except:
                        # il se peut que le lidar renvoie nimporte quoi parfois donc on se protege pour plus de robustesse
                        # ne pas oublier que l'on ne peut pas intervenir si le code crash
 
                        pass
                    
                lidar.clean_input() # cette methode vide le buffer des mesures du lidar pour eviter un buffer overflow mais il semble d'apres la doc que cela soit fait automatiquement en cas de debordement
                    
                    
        except:
            #si le lidar plante pour une quelconque raison on le redemarre
            lidar.stop()
            print("Fail du LIDAR, restart...")
            lidar.reset()
            time.sleep(2) #delai obligatoire et irreductible lie au fait que le moteur a de l'inertie et que le lidar ne peut pas s'arreter instantanement ni redemarrer instantanement


        













def main(file_scans, file_score):
    """
    Comme son nom l'indique: fonction principale

    Cette fonction, executee comme les autres dans un processus isole, communique via deux files:
    - file_scans : RASPI <-- LIDAR
    - file_score : RASPI --> ECRAN
    """
    
    print("Demarrage MAIN...")

    score = 0
    DUREE_VIE_SCAN_LIDAR = 2 #duree de vie d'un scan lidar en secondes (au bout de ce temps une mesure du lidar ne sera pas prise en compte)



    #setup de la tirette et du selecteur equipe
    GPIO.setmode(GPIO.BCM) #deux mappings de GPIO possibles, celui la correspond au standard

    GPIO.setup(PIN_TIRETTE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 16 en input pulldown pour la tirette
    GPIO.setup(PIN_SELECTEUR_EQUIPE, GPIO.IN, pull_up_down = GPIO.PUD_UP) #set la pin 26 en input pulldown pour le sélecteur équipe à 0 par défaut : 0:Bleu; 1:Jaune

    equipe = GPIO.input(PIN_SELECTEUR_EQUIPE) #0: bleu; 1:jaune

    print("tirette:", GPIO.input(PIN_TIRETTE))
    print("selecteur:", equipe)


    #bloquage tant que la tirette n'est pas tiree
    while(not GPIO.input(PIN_TIRETTE)): pass

    
    

    #connextion a la carte des actionneurs (a mettre avant le tirage de tirette ptet. pas vu de difference aux tests mais on sait jamais)
    port = serial.Serial('/dev/actionneur', 9600)

    port.write(b'e') #test de commande pour l'actionneur (ici autorise l'actionneur a bouger si il detecte qqchose au tof)

    print("boucle principale")

    while(1):
        score +=1
        file_score.put(score) #mise a jour du score pour l'ecran

        if not file_scans.empty():
            # il se trouve que .get() est bloquant alors le score ne s'affichait pas tant que le lidar n'avait pas detecte qqchose. la on passe notre chemin si rien n'est dans la file
            scan = file_scans.get().split(',') #recuperation du dernier scan du lidar

            if(time.monotonic() - float(scan[0]) < DUREE_VIE_SCAN_LIDAR):
                #si le dernier scan n'est pas perime, on fait des trucs avec
                print("\nSCAN: ({}) {}".format(time.monotonic(), scan))
        




















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
    affichage_score_process.terminate()
    main_process.terminate()
    print("Processus arretes.\n")



