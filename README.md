
#  Rapport

  

Nathan Falcao Carvalho, Matheus Siqueira Franco

  

Ce projet est un petit jeu de course en 3D. Le joueur choisit la couleur de sa voiture, puis lance une course contre plusieurs voitures adverses.

Ce rapport, ainsi que la logique de chargement du modèle `assets/car.obj`, ont été réalisés avec l’assistance de Codex.

  

Le code principal est organisé autour de trois fichiers :

  

-  `src/car.cpp` : mouvement des voitures, contrôles, IA des adversaires et collisions entre voitures.

-  `src/terrain.cpp` : génération de la piste, murs, textures et collisions avec les bords de la piste.

-  `src/scene.cpp` : rendu, menu, skybox, chargement des ressources et boucle principale du jeu.

  

##  1. Structure générale

  

Le jeu commence par un menu plein écran réalisé avec ImGui. Ce menu permet de choisir :

  

- la carte ;

- la couleur de la voiture du joueur ;

- le lancement de la course.

  

Après avoir appuyé sur `Play`, un compte à rebours `3, 2, 1, GO` est affiché avant le début réel de la course. Pendant ce temps, les voitures restent à la ligne de départ.

  

Pendant la course, `scene.cpp` coordonne les différentes parties du jeu :

  

1. Mettre à jour le temps, la caméra et la lumière.

2. Mettre à jour la voiture du joueur.

3. Mettre à jour les voitures adverses.

4. Vérifier les collisions avec les murs et entre voitures.

5. Dessiner la piste, le sol, les murs, les voitures, les roues, la skybox et les outils de debug.


  

##  2. Voitures

  

La logique des voitures est principalement dans `src/car.cpp`.

  

Chaque voiture stocke :

  

- sa position ;

- sa vitesse ;

- son accélération ;

- sa direction avant, appelée `forward` ;

- ses directions locales `right` et `up` ;

- son angle de braquage ;

- la rotation visuelle des roues ;

- son état de collision.

  

###  Mouvement

  

Le mouvement utilise un modèle simplifié de voiture.

  

À chaque mise à jour :

  

1. Les directions locales de la voiture sont recalculées.

2. Les entrées d’accélération et de direction sont lues.

3. La vitesse vers l’avant est calculée avec une projection sur `forward`.

4. Le glissement latéral est calculé avec une projection sur `right`.

5. Une accélération vers l’avant est appliquée.

6. Une traînée réduit progressivement la vitesse.

7. Une traînée latérale réduit le dérapage.

8. Le braquage fait tourner la voiture selon sa vitesse et son empattement.

9. La position et la vitesse sont intégrées avec le pas de temps `dt`.

10. L’angle de rotation des roues est mis à jour pour l’affichage.

  

La formule principale du braquage est :

  

```cpp

angular_speed = forward_speed *  tan(steering_angle)  / wheel_base;

```

  

Elle correspond à un modèle de type bicyclette : plus la voiture va vite et plus l’angle de braquage est grand, plus elle tourne rapidement.

  

###  Contrôles du joueur

  

La voiture du joueur utilise `player_car::action_keyboard()`.

  

Les touches sont :

  

- haut : accélérer ;

- bas : reculer ou freiner ;

- gauche : braquer à gauche ;

- droite : braquer à droite.

  

###  Voitures adverses

  

Les voitures adverses suivent automatiquement la piste.

  

Leur logique est hybride :

  

1. L’adversaire est projeté sur le point le plus proche de la ligne centrale de la piste.

2. S’il est proche de cette ligne centrale, il cherche à s’aligner avec la tangente locale de la piste.

3. S’il est trop loin de la ligne centrale, il vise un point situé un peu plus loin sur la piste.

4. Il calcule l’erreur d’angle entre sa direction actuelle et la direction cible.

5. Il choisit une entrée de braquage selon cette erreur.

6. Il ralentit dans les virages serrés pour éviter de sortir de la trajectoire.

  

Cette logique permet d’avoir deux comportements utiles :

  

- sur la trajectoire, les adversaires suivent naturellement la forme de la piste ;

- hors trajectoire, ils essaient de revenir vers la ligne centrale.

  

##  3. Collisions

  

Les collisions entre voitures sont aussi gérées dans `src/car.cpp`.

  

Chaque voiture possède une hitbox rectangulaire orientée. Elle est centrée sur la position de la voiture et tourne avec elle. Elle utilise donc les directions `forward` et `right`, au lieu d’être simplement alignée avec les axes du monde.

  

###  Construction de la hitbox

  

La hitbox utilise quatre coins :

  

- avant droit ;

- avant gauche ;

- arrière droit ;

- arrière gauche.

  

Ces coins sont calculés avec :

  

- la demi-longueur de la voiture ;

- la demi-largeur de la voiture ;

- la direction `forward` ;

- la direction `right`.

  

Ces mêmes quatre points peuvent être affichés dans le mode debug. Cela permet de voir directement les points utilisés par la collision.

  

###  Test d’intersection

  

Pour savoir si deux voitures se touchent, le code fait un test simple :

  

1. Prendre les quatre coins de la première voiture.

2. Vérifier si l’un de ces coins est à l’intérieur de la hitbox de la deuxième voiture.

3. Faire le même test dans l’autre sens.

4. Si un coin est trouvé à l’intérieur, les deux hitboxes sont considérées en collision.

  

Pour tester si un point est dans une hitbox orientée, le point est d’abord exprimé dans le repère local de la voiture :

  

1. On calcule le vecteur entre le centre de la voiture et le point.

2. On projette ce vecteur sur `forward`.

3. On projette ce vecteur sur `right`.

4. Si les deux valeurs sont plus petites que la demi-longueur et la demi-largeur, le point est dans le rectangle.

  

Cette méthode est pratique car elle fonctionne même lorsque la voiture est tournée.

  

###  Réponse à la collision

  

Quand une collision est détectée :

  

1. Le code calcule une normale de collision entre les deux voitures.

2. Il calcule la vitesse relative entre elles.

3. Si les voitures s’éloignent déjà, aucune correction n’est appliquée.

4. Si elles se rapprochent, une impulsion élastique est appliquée.

5. Les vitesses des deux voitures sont modifiées dans des directions opposées.

6. Une rotation temporaire est ajoutée selon le point de contact.

  

La rotation de collision donne un effet plus vivant : si une voiture est touchée sur le côté ou sur un coin, elle peut pivoter au lieu de seulement reculer.

  

##  4. Terrain, piste et murs

  

La logique du terrain est dans `src/terrain.cpp`.

  

La piste est générée à partir d’une ligne centrale. Le programme peut ensuite construire autour de cette ligne :

  

- la route asphaltée ;

- les murs latéraux ;

- les coordonnées de texture ;

- les informations utiles pour l’IA et les collisions.

  

###  Projection sur la piste

  

La fonction `closest_track_projection()` cherche le point de la ligne centrale le plus proche d’une position donnée.

  

Elle renvoie notamment :

  

- le point projeté ;

- la tangente de la piste ;

- la direction latérale ;

- la distance latérale par rapport au centre de la piste ;

- une estimation de la courbure.

  

Cette fonction est utilisée pour :

  

- guider les adversaires ;

- détecter si une voiture s’éloigne de la piste ;

- gérer les collisions avec les murs.

  

###  Maillages et textures

  

Deux maillages principaux sont créés :

  

-  `create_asphalt_mesh()` construit la route ;

-  `create_barrier_mesh()` construit les murs.

  

Les textures utilisées sont :

  

-  `assets/asphalt.jpg` pour la route ;

-  `assets/brick.jpg` pour les murs.

  

Les coordonnées UV sont calculées avec la distance le long de la piste. Cela permet aux textures de se répéter correctement au lieu d’être trop étirées.

  

###  Collision avec les murs

  

Pour garder les voitures sur la piste :

  

1. Le code teste les quatre points de hitbox de chaque voiture.

2. Il vérifie si un point dépasse la largeur autorisée de la piste.

3. Si oui, il calcule la profondeur de pénétration.

4. La voiture est replacée à l’intérieur de la piste.

5. Sa vitesse est réfléchie contre la normale du mur.

6. Une friction est appliquée pour réduire l’énergie après le contact.

  

##  5. Rendu et ressources

  

Le rendu est principalement organisé dans `src/scene.cpp`.

  

La scène initialise :

  

- le sol ;

- la piste ;

- les murs ;

- la skybox ;

- le modèle 3D de la voiture ;

- les roues et les jantes ;

- les textures ;

- les points de debug des hitboxes.

  

Le modèle de voiture est chargé depuis `assets/car.obj`. Les fenêtres sont séparées du reste du modèle grâce au matériau `Window`. Cela permet de leur appliquer un rendu différent, plus sombre et réfléchissant, pendant que la carrosserie garde la couleur choisie par le joueur.

  

La scène affiche aussi les outils de debug :

  

- affichage du repère global ;

- vue de dessus ;

- affichage des points de hitbox ;

- valeurs de vitesse, accélération, braquage et collision dans l’interface.

  

##  Conclusion

  

Le projet combine plusieurs éléments classiques d’un jeu de course :

  

- une piste générée procéduralement ;

- des voitures contrôlables ;

- des adversaires autonomes ;

- des collisions avec les murs et entre voitures ;

- un menu de départ ;

- un compte à rebours ;

- un rendu avec textures, skybox et modèle OBJ.

  

La séparation entre `car.cpp`, `terrain.cpp` et `scene.cpp` permet de garder une structure claire : les voitures gèrent leur physique, le terrain gère la piste, et la scène coordonne le jeu et l’affichage.
