## API entre l'embase (STM32) et la Raspberry Pi 

La Raspberry Pi envoie des commandes par liaison série à la STM32. Chaque commande est de la forme suivante : 

```
commande arg1 arg2 ...
```

Chaque commande se termine par un passage à la ligne (`\n`). Les arguments sont séparés par des espaces.

## Liste des commandes 

| Commande           | Nombre d'arguments | Arguments                                                  |
| ------------------ | ------------------ | ---------------------------------------------------------- |
| ma (move absolute) | 3                  | (int)x (int)y (int)theta_degres                            |
| mr (move relative) | 3                  | (int)x (int)y (int)theta_degres                            |
| s (start)          | 0                  |                                                            |
| w (wait)           | 1                  | (int)time_ms, if time_ms = 0 then wait for a start command |
| gp (get position)  | 0                  |                                                            |
| gt (get task)      | 0                  |                                                            |

## Description détaillée 
