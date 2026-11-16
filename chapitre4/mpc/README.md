# Arrêter un robot avant une ligne : MPC

GEN6323, chapitre 4, séance 4. Complément facultatif, 16 novembre 2026.

## Expérience

[Ouvrir le laboratoire](https://sberkane.github.io/GEN6323-simulations/chapitre4/mpc/index.html), ou ouvrir `index.html` hors ligne : le fichier est autonome, sans installation. Choisir l'horizon N de 1 à 20, la vitesse initiale de 0,5 à 5 m/s, la distance à la ligne de 0,5 à 20 m et le poids de l'effort (0,01; 0,1; 1; 10). Chaque modification remet l'expérience au départ. La vitesse souhaitée reste égale à la vitesse initiale choisie; le pas vaut une seconde.

Avancer pas à pas ou lancer la lecture. Les traits pleins montrent les états obtenus, les pointillés la prédiction recalculée. Seule la première commande de la suite est appliquée.

- **Exemple du cours** : vitesse 2 m/s, ligne à 2 m. Avec N=1, l'optimum initial est a=0. Le robot atteint (2,2), puis le problème devient infaisable. Avec tout N≥2, les contraintes imposent le freinage maximal : (0,2) → (1,5;1) → (2,0). Ces résultats sont identiques pour les quatre poids proposés.
- **Plus loin, plus vite** : vitesse 4 m/s, ligne à 8 m, poids 0,1. Avec N=2, le robot conserve d'abord sa vitesse puis se retrouve dans une impasse. Avec N=3, il freine immédiatement et s'arrête à 8 m après quatre pas. La prédiction n'a pas besoin de couvrir tout l'arrêt pour produire ici la bonne première commande.
- Modifier ensuite un seul paramètre et comparer plusieurs horizons. Un horizon plus long ne peut corriger une distance initiale insuffisante. Il ne garantit pas non plus un arrêt exactement à la ligne : aucun objectif de position finale n'est imposé. Par exemple, avec vitesse 4 m/s, ligne à 12 m, N=4 et poids 0,1, la boucle tend vers un arrêt avant la ligne.

Une interruption pour infaisabilité ou échec numérique arrête le calcul, pas physiquement le robot. Un quasi-arrêt est signalé lorsque la vitesse et la prochaine accélération sont toutes deux inférieures à 0,000002 en valeur absolue; une limite de 120 pas évite une lecture indéfinie.

## Modèle et distance de freinage

Les unités sont SI; le coût normalise les vitesses par 1 m/s et les accélérations par 1 m/s². L'accélération est réalisée instantanément et maintenue constante pendant chaque pas, sans perturbation ni retard. L'intégration exacte est

`x_{i+1} = x_i + Ts*v_i + Ts²*a_i/2`, `v_{i+1} = v_i + Ts*a_i`.

Le freinage maximal admissible applique `a_i = -min(1,v_i/Ts)`, puis reste à vitesse nulle. Il minimise chaque position future. Pour Ts=1 s et une décélération maximale de 1 m/s², la distance minimale vaut `v²/2 + r*(1-r)/2`, avec `r = v-floor(v)` (valeurs numériques en unités SI). À 1,5 m/s, elle vaut donc 1,25 m. La formule continue v²/2 donnerait 1,125 m, mais supposerait de pouvoir changer l'accélération au milieu d'un pas. Le diagnostic affiché respecte le maintien par pas du laboratoire.

La vitesse est affine et non négative entre deux échantillons si elle l'est aux extrémités. La position est donc croissante : les contraintes sont respectées aussi entre les échantillons du modèle. Aucune contrainte terminale n'est ajoutée; ces expériences ne constituent pas une garantie générale de faisabilité récursive.

## Construire le problème quadratique

À chaque mesure (x,v), chercher `a=(a0,...,aN-1)` minimisant

`sum_i ((v_{i+1}-vref)^2 + rho*a_i^2)`

sous `-1 <= a_i <= 1`, `v_{i+1} >= 0`, `x_{i+1} <= limit`.

En empilant les prédictions :

- `V = v*ones(N,1) + S*a`, avec `S(i,j)=Ts` si j≤i, sinon zéro;
- `X = x*ones(N,1) + Ts*v*(1:N)' + P*a`, avec `P(i,j)=Ts²*(i-j+1/2)` si j≤i, sinon zéro;
- `H=2*(S'*S+rho*I)`, `g=2*S'*(v-vref)*ones(N,1)` pour `min 0.5*a'*H*a+g'*a`;
- `C=[P;-S]`, `h=[limit-x-Ts*v*(1:N)'; v*ones(N,1)]`, avec bornes `lb=-1`, `ub=1`.

Pour N=2, Ts=1 : `S=[1 0;1 1]`, `P=[0.5 0;1.5 0.5]`.

`simulation_mpc.m` construit ce problème avec `quadprog` (Optimization Toolbox), vérifie le retour du solveur et les contraintes, applique uniquement a(1), puis recommence. Les paramètres sont au début du script. Le script a été relu, mais n'a pas été exécuté dans MATLAB lors de cette mise à jour.

## Solveur du navigateur

`mpc.js` résout le même QP continu, strictement convexe, pour N de 1 à 20, sans grille d'accélérations ni bibliothèque externe. Le calcul utilise les vitesses futures comme variables : le coût a une Hessienne tridiagonale et les bornes d'accélération portent sur les différences de vitesses. Puisque les vitesses sont non négatives, la dernière contrainte de position implique les précédentes.

Le freinage maximal fournit un point admissible ou un certificat d'infaisabilité. Si sa position finale atteint exactement la limite, la suite est unique. Sinon, une méthode d'ensemble actif primal calcule l'optimum et contrôle faisabilité, stationnarité, multiplicateurs et complémentarité. Un problème numérique est distingué d'une infaisabilité mathématique; aucune commande non validée n'est appliquée. Le problème renvoyé par l'API reste exprimé en accélérations pour faciliter les comparaisons.

La page embarque une copie exacte de `mpc.js` dans `<script id="mpc-solver">`. Après modification, remplacer cette copie; le vérificateur détecte toute différence.

## Vérification

Exécuter `node verifier_mpc.js` pour vérifier le solveur et les gestionnaires de l'interface, ou `node verifier_solver.js` pour le solveur seul. Les tests couvrent les horizons 1 à 20, les optima connus, la faisabilité par freinage maximal, des certificats d'optimalité, les contraintes entre échantillons, les changements de paramètres, les deux préréglages et les interruptions. Ils affichent leur bilan. Les tests de DOM ne remplacent pas une inspection graphique dans un navigateur. Ces vérifications concernent le modèle numérique, pas la sécurité d'un robot physique.
