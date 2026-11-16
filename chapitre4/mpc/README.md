# Arrêter un robot avant une ligne : MPC

GEN6323, chapitre 4, séance 4. Complément facultatif, 16 novembre 2026.

## Expérience

Ouvrir `index.html` dans un navigateur : ce fichier est autonome, sans installation ni connexion. Choisir un horizon, puis avancer pas à pas ou lancer la lecture. Les traits pleins montrent les états obtenus; les pointillés montrent la prédiction recalculée à partir de l'état actuel. Le tableau sépare la suite prévue de la première commande appliquée.

- **Un pas** : depuis `(x,v)=(0,2)`, l'optimum est `a=0`. Le robot atteint `(2,2)`. Au calcul suivant, aucune accélération admissible ne respecte la ligne; la simulation s'interrompt. Le robot conserve une vitesse de 2 m/s : l'arrêt du calcul ne représente pas un arrêt physique.
- **Deux pas** : la première suite optimale est `(-1,-1)`. Appliquer seulement `-1`, puis recalculer. Le robot passe par `(1.5,1)` et s'arrête à `(2,0)`.

Le poids de l'effort vaut `rho=0.1` par défaut; le sélecteur propose 0.01, 0.1, 1 et 10, et remet l'expérience au départ. Dans ce cas précis, les contraintes imposent le même freinage pour tous ces poids avec deux pas; le coût change. Les grandeurs sont en unités SI, avec normalisation par 1 m/s et 1 m/s² dans le coût. Le modèle suppose une accélération réalisée instantanément, constante sur chaque seconde, sans perturbation ni retard. Il n'inclut aucune contrainte terminale. Le succès de l'horizon deux dans cet exemple ne garantit pas la faisabilité récursive dans d'autres situations.

## Construire le problème quadratique

À chaque mesure `(x,v)`, chercher `a=(a0,...,aN-1)` minimisant

`sum_i ((v_{i+1}-2)^2 + rho*a_i^2)`

sous `-1 <= a_i <= 1`, `v_{i+1} >= 0`, `x_{i+1} <= 2`.

L'intégration exacte est `x_{i+1}=x_i+Ts*v_i+Ts²*a_i/2`, `v_{i+1}=v_i+Ts*a_i`.

En empilant les prédictions :

- `V = v*ones(N,1) + S*a`, avec `S(i,j)=Ts` si `j<=i`, sinon zéro;
- `X = x*ones(N,1) + Ts*v*(1:N)' + P*a`, avec `P(i,j)=Ts²*(i-j+1/2)` si `j<=i`, sinon zéro;
- `H=2*(S'*S+rho*I)`, `g=2*S'*(v-vref)*ones(N,1)` pour la convention `min 0.5*a'*H*a+g'*a`;
- contraintes `C=[P;-S]`, `h=[2-x-Ts*v*(1:N)'; v*ones(N,1)]`, avec bornes `lb=-1`, `ub=1`.

Pour `N=2`, `Ts=1` : `S=[1 0; 1 1]`, `P=[0.5 0; 1.5 0.5]`.

`simulation_mpc.m` réalise cette construction avec `quadprog` (Optimization Toolbox), vérifie le retour du solveur et les contraintes, applique uniquement `a(1)`, puis recommence. Changer `N` pour comparer les horizons.

## Solveur du navigateur

`mpc.js` résout le même problème continu pour `N=1` ou `N=2`. Le coût est strictement convexe. L'optimum appartient à l'intérieur, à une arête ou à un sommet de l'ensemble admissible. Le code évalue donc le minimum sans contrainte, le minimum sur chaque frontière et les intersections de deux frontières; il conserve le meilleur candidat admissible. Il n'y a aucune grille d'accélérations. Les tolérances flottantes sont de l'ordre de `1e-8`.

La page contient une copie exacte du solveur, afin de pouvoir être distribuée seule. Après toute modification de `mpc.js`, remplacer le contenu de la balise `<script id="mpc-solver">` dans la page. Le vérificateur détecte une différence.

## Vérification

Exécuter `node verifier_mpc.js`. Le programme vérifie les solutions analytiques des diapositives pour quatre poids, 600 cas déterministes avec certificats KKT pour les optima trouvés, les contraintes entre les échantillons, la faisabilité par freinage maximal et l'identité du solveur embarqué. Les commandes pas à pas, changement d'horizon, remise au départ et lecture/pause sont également exécutées dans un DOM minimal; cette vérification ne remplace pas une inspection visuelle dans le navigateur. Ces vérifications portent sur le modèle numérique; elles ne constituent pas une preuve de sécurité d'un robot physique.
