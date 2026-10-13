# Laboratoire interactif — Du but au mouvement

GEN6323, chapitre 4, séances 2 à 4. Version du 13 octobre 2026.

## Ouvrir et utiliser

Décompresser l'archive, puis ouvrir `index.html` dans un navigateur récent. Garder les fichiers voisins dans le même dossier. Aucun serveur, compte, téléchargement ou connexion n'est nécessaire. Les trois expériences partagent la lecture/pause, le retour au début, le curseur temporel et la vitesse de lecture. Les paramètres recalculent l'essai depuis son état initial; ils ne modifient pas un essai déjà en cours.

Dans les missions à but fixe, cliquer sur la scène choisit une nouvelle cible et lance le mouvement. Le robot repart du même état initial pour permettre des comparaisons. Les courbes montrent tout l'essai; le trait vertical marque l'instant animé. Les indicateurs « Bilan de l'essai complet » décrivent le résultat final, indépendamment du curseur. Le bouton CSV exporte les courbes avec leurs temps, noms et unités (séparateur point-virgule, décimales françaises).

## Liens directs vers une expérience

Depuis la racine du dépôt, utiliser `chapitre4/interactif/index.html` suivi du fragment ci-dessous. Les mêmes liens fonctionnent sur le site hébergé et dans une copie locale complète du dépôt.

| Expérience | Lien depuis ce dossier |
| --- | --- |
| Point cible, cap libre | [index.html#planar](index.html#planar) |
| Pose cible, cap imposé | [index.html#planar/pose](index.html#planar/pose) |
| Trajectoire chronométrée du robot | [index.html#planar/trajectory](index.html#planar/trajectory) |
| Chemin géométrique du robot | [index.html#planar/path](index.html#planar/path) |
| Boucle de vitesse P, PI et PID | [index.html#vitesse](index.html#vitesse) |
| Quadrirotor : point cible | [index.html#quad](index.html#quad) |
| Quadrirotor : cercle chronométré | [index.html#quad/trajectory](index.html#quad/trajectory) |

`#quad/trajectory` choisit la mission interne `circle`; `#quad/circle` est également accepté et est normalisé vers ce lien public. `#planar/goal` et `#quad/goal` sont des alias des buts fixes. Un fragment vide ou inconnu ouvre le point cible du robot (`#planar`). Aucun autre paramètre n'est lu dans l'adresse.

L'adresse suit les changements d'onglet et de mission. Les boutons précédent/suivant du navigateur restaurent la mission correspondante; les autres réglages restent en mémoire tant que la page est ouverte. Changer réellement de mission relance le calcul et met la lecture en pause. Rester sur la même mission conserve l'instant affiché et la lecture. Un lien partagé choisit une expérience, pas les gains ou le point cliqué.

Le lien « Accueil » du bandeau vise `../../index.html`, à la racine du dépôt; le guide reste accessible par `guide.html`. Si seul le dossier du laboratoire est copié isolément, le laboratoire et son guide restent autonomes, mais le lien d'accueil requiert l'arborescence complète.

## 1. Déplacer le robot

Quatre tâches :

- **Point cible, cap libre.** Le robot rejoint (x_g,y_g), sans orientation finale prescrite. On compare la cinématique idéale, une boucle de vitesse P ou une boucle PI, avec des couples limités pour les deux modèles dynamiques.
- **Pose cible, cap imposé.** Le même but reçoit une orientation. Le contrôleur hybride alterne approche, freinage, alignement et maintien. Le mode courant et les erreurs réelles sont affichés.
- **Huit chronométré.** La référence x_d=3 sin(2πt/T), y_d=1,6 sin(4πt/T) fixe une position à chaque instant. Le robot commence sur cette référence avec sa vitesse initiale. Raccourcir T rend le mouvement plus exigeant.
- **Cercle sans horaire.** Pure Pursuit vise un point situé en avant sur un cercle de rayon 2 m; il utilise la distance réelle au point visé pour calculer la courbure. Le paramètre d'anticipation est une longueur d'arc, distincte de cette corde. Le départ est (2,8;−0,6) m, orienté vers +y, au repos pour le modèle dynamique.

Proposition de manipulation : choisir le même point en mode cap libre puis cap imposé à 90°. Comparer les orientations finales et les phases. Passer ensuite de la cinématique idéale au PI et diminuer les couples. Enfin, comparer le huit lent (32 s) et rapide (8 s), puis le cercle sans horaire.

Le modèle différentiel reprend m_e=12 kg, J_e=0,70 kg m², c_v=1 N s/m, c_ω=0,08 N m s, r=0,10 m et b=0,45 m. La commande de position est v_c=0,6ρ cosα et ω_c=1,5α+0,6 sinα cosα; elle autorise le recul. Les PI normalisés ont les gains (4,4) pour v et (6,9) pour ω. Les couples des deux roues sont limités séparément; le rétro-calcul utilise les efforts effectivement réalisés, avec T_aw=0,30 s. Aucun arrêt artificiel n'est appliqué au mode point près du but.

La pose utilise une entrée en freinage à ρ≤3 cm, puis un alignement lorsque |v|<0,015 m/s et |ω|<0,03 rad/s. Pendant l'alignement, v_c=0 et ω_c=1,5 wrap(θ_g−θ). Le maintien est déclenché à |erreur de cap|<0,03 rad et vitesses faibles; on reprend l'approche si ρ>6 cm, ou l'alignement si l'erreur angulaire dépasse 0,06 rad (3,44°). Les intégrales sont conservées : une dérive résiduelle dans cette bande est possible. Il s'agit de tolérances pratiques, sans affirmation de convergence exacte ou de stabilité globale. Le cas par défaut atteint environ 3,296 cm et 2,384° après 24 s.

Pour le suivi temporel, les erreurs sont exprimées dans le repère réel du robot : v_c=v_d cos(e_θ)+1,5e_x, ω_c=ω_d+1,2v_d e_y+2 sin(e_θ). La loi ne suffit pas à garantir une convergence vers une pose fixe lorsque la référence s'arrête. Le sujet est distingué dans les diapositives de séance 3.

## 2. Commander les moteurs

La consigne passe à 1 m/s au départ; une charge résistante apparaît à 5 s. Comparer P, PI et PID filtré. Le P de référence utilise le même gain proportionnel, la même charge et les mêmes limites que le correcteur choisi. Il n'est pas limité différemment pour rendre la comparaison plus favorable.

La force demandée est F_a*=c_v v+m_e(k_P e+k_I I−k_D z), avec e=1−v, İ=e et T_f ż=v̇−z. La dérivée porte sur la mesure, avec T_f=0,05 s. La force est convertie en deux couples égaux; leur saturation borne la force totale. Avec anti-enroulement, İ reçoit en plus (F_a−F_a*)/(m_e k_I T_aw).

Expériences utiles :

- Sans saturation, charge de 3 N, k_P=4 : le P conserve 0,0625 m/s d'erreur. Le PI compense cette charge grâce à son intégrale.
- Ajouter k_D=0,4 au réglage (k_P,k_I)=(4,4) n'améliore pas automatiquement la réponse. Avec ces valeurs, le dépassement initial augmente d'environ 13,53 % à 16,22 %.
- Pour retrouver l'expérience d'anti-enroulement des diapositives, mettre la charge à zéro, limiter chaque couple à 0,40 N m et choisir PI (4,4). Comparer l'intégrale et le dépassement avec puis sans rétro-calcul.

La route défile avec la position réelle du robot. Les flèches montrent la vitesse demandée, la vitesse réelle, l'effort moteur et la charge; leur longueur est plafonnée pour rester visible.

## 3. Déplacer le quadrirotor

Le modèle est une coupe verticale du mouvement 3D. Il utilise x horizontal, z vers le haut et le tangage ϑ positif vers la droite. Les équations sont ẍ=(f/m)sinϑ, z̈=(f/m)cosϑ−g et ϑ̈=τ_y/J_y. Paramètres : m=1,5 kg, J_y=0,035 kg m², ℓ=0,25 m; g=9,81 m/s² reste toujours présent dans la physique.

La boucle de position donne v_c=v_d+k_pos(p_d−p). La boucle de vitesse donne a_c=a_d+2,2(v_c−v). On calcule ensuite F=(m a_x,c, m(a_z,c+g)), f*=||F|| et ϑ_c=atan2(F_x,F_z). La case « Compenser la gravité » retire uniquement le terme +g de cette commande. Observer le départ, l'inclinaison pour avancer, le freinage, puis le retour au stationnaire.

Le gain de position et la rapidité de l'orientation sont réglables. Le PD de tangage utilise les gains rapidité² et 1,8×rapidité. Les limites agissent sur l'angle demandé, la poussée totale, le moment (±0,8 N m), puis sur deux groupes de poussée équivalents. Chaque groupe est borné entre zéro et la moitié de la poussée maximale. La physique reçoit les efforts reconstruits après cette allocation, et non les demandes initiales.

Lorsque l'angle demandé est limité, le contrôleur conserve la norme de force avant sa propre limitation : l'accélération réalisée peut donc changer dans les deux directions. Une inclinaison maximale demandée n'est pas une borne rigide sur l'angle réel. Les flèches de poussée et de poids ont la même échelle graphique, plafonnée pour rester lisible; les courbes conservent les valeurs complètes.

Le point fixe utilise un départ (−3;1,5) m au repos et une cible initiale (2,5;3) m. La trajectoire circulaire a un rayon de 1,2 m, un centre (0;3) m et une pulsation de 0,45 rad/s; le drone part de (1,2;3) m au repos. Elle illustre le suivi avec anticipation de vitesse et d'accélération.

Les arrêts sont explicites : arrivée du centre à z=0 (sans modèle de contact ou rebond), force verticale demandée négative hors du domaine debout, inclinaison réelle supérieure à 75°, ou sortie de la zone pédagogique |x|≤8 m et z≤8 m. La vue s'adapte aux états effectivement calculés. Un réglage trop exigeant peut interrompre l'essai; le programme ne ramène pas artificiellement le drone à sa cible.

Proposition : comparer avec/sans compensation de gravité; puis réduire la rapidité de l'orientation et observer la différence entre angle demandé et réel. Enfin réduire la poussée disponible. Le but par défaut termine à environ 1 µm d'erreur dans ce modèle idéalement mesuré; ce chiffre ne prédit pas la précision d'un vrai drone. Le pas divisé par deux change au maximum les deux trajectoires de contrôle de 4,42 µm en position et 2,15 µrad en angle.

## Hypothèses communes

Paramètres synthétiques à but pédagogique, mesures parfaites, modèle continu. Aucun glissement, bruit de capteur, retard de calcul ou dynamique électrique des moteurs n'est simulé. Les schémas de robots ne sont pas des modèles identifiés de produits commerciaux. Les résultats numériques illustrent des conditions précises; ils ne prouvent pas la stabilité générale d'une cascade saturée.

Les modules calculent les équations du mouvement à chaque essai avec RK4 et conservent un échantillon toutes les 20 ms. L'animation affiche ces échantillons sans modifier leur physique. Une protection interrompt les essais qui sortent du domaine numérique retenu et l'indique à l'écran; les trajectoires ne sont pas corrigées artificiellement pour rejoindre les cibles.

## Fichiers et reproduction

- `index.html`, `app.js` : interface, commandes de lecture et graphiques.
- `planar.js`, `vitesse.js`, `quad.js` : modèles, calculs et dessin de chaque expérience.
- `guide.html` : version lisible de ce guide dans le navigateur.
- `verifier_laboratoire.js` : vérifications numériques autonomes avec Node.js, sans bibliothèque additionnelle.

Pour vérifier les calculs : `node verifier_laboratoire.js`. Les figures PDF de séance 2 ont également un programme Python indépendant dans `../seance2/simulation_seance2.py`. Le profil P est également comparé à sa solution analytique.
