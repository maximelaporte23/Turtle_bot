# **TurtleBot3 avec ROS2**

## Projet du TurtleBot3

L'équipe:
* Beauvais Florian
* Bouffanais Hugo
* Dellau Landry
* Gonzalez Célia
* Martinez Mathieu
* Laporte Maxime


## Le rendu

2 dossiers sont dans le git: 1 pour la Simulation et 1 pour le Robot_physique.

* Simulation

4 fichiers .py, avec les vidéos correspondantes:

    - sans_nav.py avec le follow me, go home et le dock sans détection d'obstacles
         vidéo : sans_nav.webm

    - go_home_obstacle.py avec le follow me, le go home avec détection d'obstacles
         vidéos : go_home_couloir.webm et go_home_porte.webm

    - dock.py avec le dock, principe du rumba
         vidéo : dock.webm

    - nav.py avec la nav. 
    Le follow me et le go home fonctionnent mais arrivé à la position, le robot y va mais il n'atteint jamais la position. Problème de tuning du robot en lui même, par manque de temps on ne pourra pas le régler et il y a trop de paramètres. Le dock est fonctionnel individuellement mais il ne se lance pas parce que le go home atteint jamais la position.
         vidéo pas enregistrée


Pour lancer le fichier:

Dans le dossier follow me, il y a un code qui s'appelle __publish_cmd_vel.py__ . Il faut copier et coller le code à tester dans __publish_cmd_vel__ . Il faut toujours lancer ce code avec la commande __ros2 run follow_me publish_cmd_vel__ .
Pour tous les fichiers, il faut ouvrir un terminal et lancer cette ligne de commande __ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml__
Pour  __ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py__


* Robot_physique

1 fichier fonctionnel avec 2 vidéos:

    - robot_physique.py avec le follow me et le go home sans obstacles
         vidéos : follow_me.mp4 et go_home.mp4



Pour lancer le fichier :

Se connecter en ssh au robot. Lancer __ros2 launch turtlebot3_bringup robot.launch.py__ sur le même terminal.
Ouvrir un deuxième terminal et lancer __pyhton3 robot_physique.py__ .
