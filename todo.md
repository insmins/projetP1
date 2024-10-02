# Todos

- calibrer la cam avec le damier (demander a M.Boonaert) --> ok
- avoir transfos cam2gripper et gripper2base (calibrer) --> ok
- voir pk cam sur fond blanc = nul --> idéalement besoin fond non blanc, lisse et shiny ?  --> fini (truc chelou en nuance de gris)
- point avec M.Boonaert (quels sont les objets à prendre, où, disposer comment?) --> lundi 16/09 aprem --> fini
- M.Motetse : explication pince (gripper) --> lundi 16/09 matin --> fini
- recup data a partir du nuage de points --> en cours
- pypmeshlab --> voir si utile pour traiter un stl --> osef
- décision face à prendre (ses de l'obj, cinématique inverse)  --> voir coppelia pour simu 
- définir lieu dépôt (coin haut gauche /robot) --> ok 
- avoir position cube dans le mesh --> (ines)
- simuler déplacement robot poiuur voir si il peut choper le cube (voir sur coppelia) --> (matteo)
- definir la prise du robot avec la position du cube



## point julien (lundi 16/09)
- pour l'instant utiliser la pince avec une commande pour lancer un script de la tablette
- il essaye d'avoir un devis pour installer un autre module pour que la pince fonctionne.

## point boonaert (lundi 16/09):
- solution 1 : trouver la normale de la projetion + centre de gravité, reconstituer ce qui pourrait etre un cube
- solution 2 : utiliser les points d'intérêt (arêtes etc)

- calibration plus tard, d'abord se concentrer sur les nuages de points sur la caméra
- disposition un peu libre (choisir (éventuellement configurable selon la taille des éléments))

- creer des cubes plus petits --> en fait pas besoin car la pince s'ouvre suffisamment

## point boonaert (lundi 23/09):
- avoir vrai cube, sans trou
- coppelia pour simu robot


## liens :
- calibrer la caméra : https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance
## video pour la segmentation du cube
https://youtu.be/-OSVKbSsqT0?si=YsiDMUULMWDmbrLX



## présentation
expliquer le code de la pince, commenter, faire des parties claires etc
readme
expliquer nos calculs
pour que quel que soit le robot on puisse adapter le code
presentation ppt (pas enormement de slides)
références etc
préciser calibration

