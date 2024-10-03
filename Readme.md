# Projet P1 Inès EL HADRI et Mattéo CAUX    

## Description du projet

L’objectif de ce projet est de développer une application pilotant un robot UR-5 afin que ce dernier soit capable de saisir et de ranger un ensemble de cubes posés en « vrac » dans l’espace de travail du robot : les cubes sont initialement placés de manière désordonnée, si bien que leur orientation peut être quelconque (c’est-à-dire que les normales aux faces des cubes peuvent être en dehors d’un plan horizontal ou d’un plan vertical). A l’aide d’une caméra de profondeur de type « real sense », le système doit d’abord analyser son environnement en construisant un nuage de points. Sur la base de ce dernier, il tente de détecter et d’estimer la pose des cubes présents. Cette pose étant disponible, l’application doit sélectionner les cubes jugés « préhensible » (critères à définir) par le robot, déterminer une trajectoire d’accostage et de « capture » de ces cubes pour les disposer de manière régulière dans un contenant ou sur l’espace de travail.

La fiche du projet est disponible ici : [fiche projet](./UV_Projet_tri-robotise-prise-pieces-vrac.pdf)

## Etapes de réalisation

### Prise de photo

Le robot possède 6 positions enregistrées pour prendre des photos.

La librairie utilisée est `pyrealsense`.
Lorsqu'une position est atteinte par le robot, la caméra prend une photo et crée une liste de points. Chaque point est stocké sous la forme "`[pixel x, pixel y, profondeur]`" 

### Création d'un nuage de points à partir des photos

Une fois que les listes de points de chaque photo ont été créées, les pixels x et y sont convertis en positions x et y selon le repère de la caméra grâce aux fonctions de `pyrealsense`. Puis une liste mêlant les points de toutes les images est créée en ramenant chaque point dans le repère du robot.

Le repère de la caméra est comme suit :

![Axes de la caméra](https://www.intelrealsense.com/wp-content/uploads/2019/02/LRS_CS_axis_base.png "Axes de la caméra").

Il est possible de stocker ces 6 listes de points en .txt grâce au paramètre "save" de la fonction `cube.create_points`.

### Repérage d'un cube avec un Ransac

#### Pré-traitement
La librairie Open3D permet d'effectuer un grand nombre d'opérations sur les nuages de points. Voici la liste des opérations de pré-traitement effectuées :
- Création d'un objet `PointCloud`contenant les points 3D obtenus par la caméra
- Suppression des points statistiquement aberrants
- Sous-échantillonnage du nuage de points pour alléger les calculs
- Calcul des normales des points

#### La méthode Ransac
RANSAC, ou Random Sample Consensus est une méthode pour estimer des paramètres mathématiques. Ici, on utilise cette méthode pour reconnaître un cube dans un nuage de points. La méthode consiste en un choix aléatoire de paramètres, effectué un nombre conséquent de fois, pour ne garder que les paramètres les plus proches de la réalité.

Dans le cas de la reconnaissance du cube, on estime une position du centre du cube et on calcule le nombre de points entre une distance minimale et une distance maximale. Dans un cube, cela correspond à la distance entre le centre et le centre d'une face pour le minimum, et entre le centre et un coin du cube pour le maximum.
Ainsi, à la fin de ce Ransac, on obtient la position du centre, mais l'orientation de ce cube n'est pas assez précise.

![Ransac de carré](/illu_ransac.png "Ransac de carré").

#### Recherche de l'angle
Pour rendre l'orientation du cube plus précise, on récupère de la méthode précédente la position du centre du cube ainsi que les points appartenant au cube trouvé (les points à une certaine distance du centre).
Les deux fonctions de Open3D utiles dans la recherche de l'angle sont le calcul de la normale de chaque point en fonction de ses voisins d'une part, et l'algorithme de Ransac fourni dans la librairie pour détecter un plan.

La stratégie est de repérer un premier plan qui correspond à une face du cube. En calculant la normale des points de ce plan et en faisant la moyenne des normales, on obtient la normale de la face. Il suffit ensuite de recommencer 2 fois en s'assurant de ne pas prendre une face parallèle à une déjà prise pour obtenir une famille de vecteurs plus ou moins orthogonale. Une fois 3 vecteurs obtenus, on les orthogonalise suivant le procédé de Gram-Schmidt et on normalise la base pour avoir des vecteurs unitaires. Enfin, on vérifie que cette base est directe.

### Prise du cube et dépôt

Une fois une base directe obtenue, on place les 3 vecteurs de la base en colonne dans une matrice 3x3 pour obtenir une matrice de rotation par rapport à la base du robot. On concatène la position du centre pour obtenir une matrice de passage 4x4 (la dernière ligne étant [ 0, 0, 0, 1]).
Une fois cette matrice de passage obtenu, on définit une position du robot au dessus du cube et une position de prise du cube.
Une fois le cube pris, on le dépose à la position de rangement calculée en fonction du nombre de cube déja pris.

Une fois l'opération de dépôt effectuée, on réitère le procédé jusqu'à ce que la caméra ne trouve plus de cube.

## Rendu 

[Lien du gitlab](https://gvipers.imt-nord-europe.fr/ines.el.hadri/projetp1)

### Robot.py

### Pince.py

### cube.py

### Camera.py

## Installation

### Environnement Virtuel
Le projet a été entièrement réalisé en Python. Le programme est fonctionnel pour les versions de Python entre `3.9.13` et `3.11.7`

Les librairies nécessaires sont : (installables avec : `pip install nomdelalibrairie`)
- `opencv-python` : pour le traitement de la caméra
- `pyrealsense2` : API de la caméra
- `numpy` : pour des calculs simples de matrices
- `open3d` : pour le traitement et l'affichage des nuages de points en 3d
- `ur-rtde` : API du robot UR-5

### Environnement Physique



