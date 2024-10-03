# Projet P1 Inès EL HADRI et Mattéo CAUX    

## Description du projet

L’objectif de ce projet est de développer une application pilotant un robot UR-5 afin que ce dernier soit capable de saisir et de ranger un ensemble de cubes posés en « vrac » dans l’espace de travail du robot : les cubes sont initialement placés de manière désordonnée, si bien que leur orientation peut être quelconque (c’est-à-dire que les normales aux faces des cubes peuvent être en dehors d’un plan horizontal ou d’un plan vertical). A l’aide d’une caméra de profondeur de type « real sense », le système doit d’abord analyser son environnement en construisant un nuage de points. Sur la base de ce dernier, il tente de détecter et d’estimer la pose des cubes présents. Cette pose étant disponible, l’application doit sélectionner les cubes jugés « préhensible » (critères à définir) par le robot, déterminer une trajectoire d’accostage et de « capture » de ces cubes pour les disposer de manière régulière dans un contenant ou sur l’espace de travail.

La fiche du projet est disponible ici : [fiche projet](./UV_Projet_tri-robotise-prise-pieces-vrac.pdf)

## Etapes de réalisation

### Prise de photo

Le robot possède 6 positions enregistrées pour prendre des photos.
Lorsqu'une position est atteinte par le robot, la caméra prend une photo et crée une liste de points. Chaque point est stocké sous la forme "[pixel x, pixel y, profondeur]". Il est possible de stocker ces 6 listes de points en .txt grâce au paramètre "save" de la fonction cube.create_points.

### Création d'un nuage de points à partir des photos

Une fois que les listes de points de chaque photo ont été créées, les pixels x et y sont convertis en positions x et y selon le repère de la caméra. Puis une liste mélant les points de toutes les images est créée en ramenant chaque point dans le repère du robot.

### Repérage d'un cube avec un ransac

oé Inès pcq je sais pas comment qu'on a fait

### Détermination de la pose du robot pour prendre le cube en fonction de la base du cube

Une fois le robot identifié ainsi que son centre. On isole un face, on calcule pour chaque point de cette face le vecteur normal, puis on calcule le vecteur moyen pour trouver la normale de la face. On réitère cette opération jusqu'à avoir 3 vecteurs suffisamment éloignés (dans des directions différentes). Une fois 3 vecteurs obtenus, on les orthogonalise suivant le procédé de Gram-Schmidt et on normalise la base pour avoir des vecteurs unitaires. Enfin, on vérifie que cette base est directe.

### Prise du cube et dépôt

Une fois une base directe obtenu, on place les 3 vecteurs de la base en colonne dans une matrice 3x3 pour obtenir une matrice de rotation par rapport à la base du robot. On concatène la position du centre pour obtenir une matrice de passage 4x4 (la dernière ligne étant [ 0, 0, 0, 1]).
Une fois cette matrice de passage obtenu, on définit une position du robot au dessus du cube et une position de prise du cube.
Une fois le cube pris, on le dépose à la position de rangement calculée en fonction du nombre de cube déja pris.

Une fois l'opération de dépôt effectuée, on réitère le procédé jusqu'à ce que la caméra ne trouve plus de cube.

## Rendu 

[Lien du gitlab](https://gvipers.imt-nord-europe.fr/ines.el.hadri/projetp1)

### Robot.py

### Pince.py

### cube.py

### Camera.py

